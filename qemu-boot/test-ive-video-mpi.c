/*
 * IVE Motion Detection + Abandoned Object on Real Hardware via MPI API
 *
 * Full hardware pipeline — no CPU pixel loops in the processing path:
 *   MD:        SAD → CCL → read blob struct
 *   Abandoned: Sub → Thresh → Erode → Dilate → SAD(cur,prev) → CCL → blob
 *
 * CCL constraint: 64×64 to 720×640. SAD output at 4×4 blocks from 960×528
 * is 240×132, well within range. Abandoned detection also uses SAD+CCL on
 * the binary mask level to avoid CPU pixel scanning.
 *
 * Reads Y4M (Cmono) or legacy raw .bin. Resolution from Y4M header.
 *
 * Build:
 *   CC=arm-openipc-linux-musleabi-gcc
 *   SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
 *   LIBS=~/git/firmware/output-hi3516ev300_lite/target/usr/lib
 *   $CC -o test-ive-video-mpi test-ive-video-mpi.c \
 *       -I$SDK/include -L$LIBS -lmpi -live -lVoiceEngine -lupvqe -ldnvqe -lsecurec \
 *       -Wl,-rpath,/usr/lib -Wl,--allow-shlib-undefined
 *
 * Run: killall majestic; ./test-ive-video-mpi input.y4m [md|abandoned]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdint.h>
#include <sys/time.h>

/* uclibc→musl shims */
int memcpy_s(void *d, size_t dn, const void *s, size_t n) { memcpy(d, s, n < dn ? n : dn); return 0; }
int memset_s(void *d, size_t dn, int c, size_t n) { memset(d, c, n < dn ? n : dn); return 0; }
int memmove_s(void *d, size_t dn, const void *s, size_t n) { memmove(d, s, n < dn ? n : dn); return 0; }
int strncpy_s(char *d, size_t dn, const char *s, size_t n) { strncpy(d, s, n < dn ? n : dn); return 0; }
int snprintf_s(char *d, size_t dn, size_t n, const char *f, ...) {
    va_list a; va_start(a, f); int r = vsnprintf(d, dn, f, a); va_end(a); return r;
}
const unsigned short int *__ctype_b;
size_t _stdlib_mb_cur_max(void) { return 0; }
int __fputc_unlocked(int c, FILE *stream) { return fputc(c, stream); }
int __fgetc_unlocked(FILE *stream) { return fgetc(stream); }

#include "hi_type.h"
#include "hi_common.h"
#include "hi_comm_ive.h"
#include "hi_ive.h"
#include "mpi_sys.h"
#include "mpi_ive.h"

#define BLOCK   4
#define LEARN_FRAMES    50
#define ABANDON_FRAMES  30

#define REF_DIFF_THR    40
#define REF_SAD_THR     200     /* Majestic default for 4×4 blocks */

static long long usec_now(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000000 + tv.tv_usec;
}

static HI_S32 ive_wait(IVE_HANDLE h) {
    HI_BOOL fin = HI_FALSE;
    HI_S32 ret;
    do {
        ret = HI_MPI_IVE_Query(h, &fin, HI_TRUE);
    } while (ret == HI_ERR_IVE_QUERY_TIMEOUT);
    return ret;
}

static HI_S32 alloc_img(IVE_IMAGE_S *img, HI_U32 w, HI_U32 h) {
    memset(img, 0, sizeof(*img));
    img->enType = IVE_IMAGE_TYPE_U8C1;
    img->u32Width = w; img->u32Height = h;
    img->au32Stride[0] = (w + 15) & ~15;
    return HI_MPI_SYS_MmzAlloc(&img->au64PhyAddr[0],
        (HI_VOID **)&img->au64VirAddr[0], NULL, HI_NULL,
        img->au32Stride[0] * h);
}

static void free_img(IVE_IMAGE_S *img) {
    if (img->au64PhyAddr[0])
        HI_MPI_SYS_MmzFree(img->au64PhyAddr[0], (HI_VOID *)(HI_UL)img->au64VirAddr[0]);
}

static void fill_img(IVE_IMAGE_S *img, const uint8_t *data) {
    uint8_t *v = (uint8_t *)(HI_UL)img->au64VirAddr[0];
    for (HI_U32 y = 0; y < img->u32Height; y++)
        memcpy(v + y * img->au32Stride[0], data + y * img->u32Width, img->u32Width);
    HI_MPI_SYS_MmzFlushCache(img->au64PhyAddr[0], v, img->au32Stride[0] * img->u32Height);
}

static int read_y4m_frame(FILE *fp, uint8_t *buf, int frame_sz, int is_y4m) {
    if (is_y4m) {
        char tag[8];
        if (fread(tag, 1, 6, fp) != 6) return 0;
        if (memcmp(tag, "FRAME\n", 6) != 0) return 0;
    }
    return fread(buf, 1, frame_sz, fp) == (size_t)frame_sz;
}

/* Stationary blob tracker */
typedef struct { int cx, cy, dur; } Blob;
static Blob tracked[64], prev_tracked[64];
static int ntracked, prev_ntracked;

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <input.y4m|frames.bin> [md|abandoned|lpr] [--diff-thr=N] [--sad-thr=N] [--area-thr=N]\n", argv[0]);
        return 1;
    }
    const char *mode = (argc > 2) ? argv[2] : "md";

    /* Parse optional threshold overrides: --diff-thr=N --sad-thr=N --area-thr=N */
    int opt_diff_thr = -1, opt_sad_thr = -1, opt_area_thr = -1;
    for (int a = 3; a < argc; a++) {
        if (strncmp(argv[a], "--diff-thr=", 11) == 0) opt_diff_thr = atoi(argv[a] + 11);
        else if (strncmp(argv[a], "--sad-thr=", 10) == 0) opt_sad_thr = atoi(argv[a] + 10);
        else if (strncmp(argv[a], "--area-thr=", 11) == 0) opt_area_thr = atoi(argv[a] + 11);
    }

    FILE *fp = fopen(argv[1], "rb");
    if (!fp) { perror(argv[1]); return 1; }

    /* Detect Y4M or legacy raw format */
    int W = 352, H = 288, is_y4m = 0;
    char hdr[256];
    if (fread(hdr, 1, 10, fp) == 10 && memcmp(hdr, "YUV4MPEG2 ", 10) == 0) {
        int pos = 10;
        while (pos < (int)sizeof(hdr) - 1) {
            int c = fgetc(fp);
            if (c == '\n' || c == EOF) break;
            hdr[pos++] = c;
        }
        hdr[pos] = '\0';
        char *p = hdr + 10;
        while (*p) {
            if (*p == 'W') W = atoi(p + 1);
            else if (*p == 'H') H = atoi(p + 1);
            while (*p && *p != ' ') p++;
            while (*p == ' ') p++;
        }
        is_y4m = 1;
    } else {
        fseek(fp, 0, SEEK_SET);
    }

    int frame_sz = W * H;
    int bw = W / BLOCK, bh = H / BLOCK;
    int sad_thr = (opt_sad_thr >= 0) ? opt_sad_thr : REF_SAD_THR;
    int diff_thr = (opt_diff_thr >= 0) ? opt_diff_thr : REF_DIFF_THR;
    int ccl_area_thr = (opt_area_thr >= 0) ? opt_area_thr : 4;

    fprintf(stderr, "Format: %s %dx%d (%dx%d blocks)\n",
            is_y4m ? "Y4M" : "raw", W, H, bw, bh);
    fprintf(stderr, "mode=%s sad_thr=%d diff_thr=%d area_thr=%d\n",
            mode, sad_thr, diff_thr, ccl_area_thr);

    /* CCL constraint: 64×64 to 720×640 */
    if (bw < 64 || bh < 64) {
        fprintf(stderr, "WARNING: SAD output %dx%d < 64x64, CCL disabled\n", bw, bh);
    }
    int ccl_ok = (bw >= 64 && bh >= 64 && bw <= 720 && bh <= 640);

    long file_pos = ftell(fp);
    fseek(fp, 0, SEEK_END);
    long file_end = ftell(fp);
    fseek(fp, file_pos, SEEK_SET);
    int nframes = (int)((file_end - file_pos) / (frame_sz + (is_y4m ? 6 : 0)));
    fprintf(stderr, "frames=%d\n", nframes);

    HI_S32 ret = HI_MPI_SYS_Init();
    if (ret != HI_SUCCESS) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* Allocate IVE images */
    IVE_IMAGE_S img_cur, img_prev, img_ref, img_diff, img_bin;
    IVE_IMAGE_S img_sad, img_sad16;     /* SAD threshold (U8) and value (U16) outputs */
    alloc_img(&img_cur, W, H);
    alloc_img(&img_prev, W, H);
    alloc_img(&img_ref, W, H);
    alloc_img(&img_diff, W, H);
    alloc_img(&img_bin, W, H);

    /* SAD output images (block-level) */
    alloc_img(&img_sad, bw, bh);
    memset(&img_sad16, 0, sizeof(img_sad16));
    img_sad16.enType = IVE_IMAGE_TYPE_U16C1;
    img_sad16.u32Width = bw; img_sad16.u32Height = bh;
    img_sad16.au32Stride[0] = (bw * 2 + 15) & ~15;
    HI_MPI_SYS_MmzAlloc(&img_sad16.au64PhyAddr[0],
        (HI_VOID **)&img_sad16.au64VirAddr[0], NULL, HI_NULL,
        img_sad16.au32Stride[0] * bh);

    /* Sobel S16 output images for LPR */
    IVE_IMAGE_S img_sobel_h, img_sobel_v, img_edge;
    memset(&img_sobel_h, 0, sizeof(img_sobel_h));
    img_sobel_h.enType = IVE_IMAGE_TYPE_S16C1;
    img_sobel_h.u32Width = W; img_sobel_h.u32Height = H;
    img_sobel_h.au32Stride[0] = ((W * 2) + 15) & ~15;
    HI_MPI_SYS_MmzAlloc(&img_sobel_h.au64PhyAddr[0],
        (HI_VOID **)&img_sobel_h.au64VirAddr[0], NULL, HI_NULL,
        img_sobel_h.au32Stride[0] * H);
    img_sobel_v = img_sobel_h;
    HI_MPI_SYS_MmzAlloc(&img_sobel_v.au64PhyAddr[0],
        (HI_VOID **)&img_sobel_v.au64VirAddr[0], NULL, HI_NULL,
        img_sobel_v.au32Stride[0] * H);
    alloc_img(&img_edge, W, H);  /* U8 edge magnitude after 16BitTo8Bit */

    /* Block-level images for abandoned: foreground blocks + motion blocks */
    IVE_IMAGE_S img_fg_blk, img_mot_blk, img_stat_fg;
    IVE_IMAGE_S img_zero;     /* all-zero image for SAD(mask, zero) trick */
    IVE_IMAGE_S img_sad16_b;  /* throwaway U16 output for SAD */
    alloc_img(&img_fg_blk, bw, bh);
    alloc_img(&img_mot_blk, bw, bh);
    alloc_img(&img_stat_fg, bw, bh);
    alloc_img(&img_zero, W, H);
    /* Zero out the zero image */
    memset((void *)(HI_UL)img_zero.au64VirAddr[0], 0, img_zero.au32Stride[0] * H);
    HI_MPI_SYS_MmzFlushCache(img_zero.au64PhyAddr[0],
        (HI_VOID *)(HI_UL)img_zero.au64VirAddr[0], img_zero.au32Stride[0] * H);

    memset(&img_sad16_b, 0, sizeof(img_sad16_b));
    img_sad16_b.enType = IVE_IMAGE_TYPE_U16C1;
    img_sad16_b.u32Width = bw; img_sad16_b.u32Height = bh;
    img_sad16_b.au32Stride[0] = (bw * 2 + 15) & ~15;
    HI_MPI_SYS_MmzAlloc(&img_sad16_b.au64PhyAddr[0],
        (HI_VOID **)&img_sad16_b.au64VirAddr[0], NULL, HI_NULL,
        img_sad16_b.au32Stride[0] * bh);

    /* CCL blob output */
    IVE_MEM_INFO_S blob_mem;
    memset(&blob_mem, 0, sizeof(blob_mem));
    blob_mem.u32Size = sizeof(IVE_CCBLOB_S);
    HI_MPI_SYS_MmzAlloc(&blob_mem.u64PhyAddr, (HI_VOID **)&blob_mem.u64VirAddr,
                         NULL, HI_NULL, blob_mem.u32Size);

    /* Frame buffer for I/O only */
    uint8_t *frame_data = malloc(frame_sz);
    IVE_HANDLE handle;

    /* Benchmarking accumulators (microseconds) */
    long long t_io = 0, t_ive = 0, t_cpu = 0;

    if (strcmp(mode, "abandoned") == 0) {
        /* Phase 1: Learn reference background (CPU — only done once) */
        int *bg_sum = calloc(frame_sz, sizeof(int));
        int n_learn = LEARN_FRAMES < nframes ? LEARN_FRAMES : nframes;
        for (int i = 0; i < n_learn; i++) {
            read_y4m_frame(fp, frame_data, frame_sz, is_y4m);
            for (int j = 0; j < frame_sz; j++) bg_sum[j] += frame_data[j];
        }
        uint8_t *ref_data = malloc(frame_sz);
        for (int j = 0; j < frame_sz; j++) ref_data[j] = bg_sum[j] / n_learn;
        fill_img(&img_ref, ref_data);
        free(bg_sum); free(ref_data);
        /* Rewind */
        fseek(fp, file_pos, SEEK_SET);
    }

    /* Read first frame */
    long long t0 = usec_now();
    read_y4m_frame(fp, frame_data, frame_sz, is_y4m);
    t_io += usec_now() - t0;
    fill_img(&img_prev, frame_data);

    int detected_frames = 0;

    for (int i = 1; i < nframes; i++) {
        /* --- I/O: read frame from NFS --- */
        t0 = usec_now();
        if (!read_y4m_frame(fp, frame_data, frame_sz, is_y4m)) break;
        long long t1 = usec_now();
        t_io += t1 - t0;

        /* --- IVE: copy frame to MMZ --- */
        fill_img(&img_cur, frame_data);

        if (strcmp(mode, "md") == 0) {
            /* === MOTION DETECTION: SAD → CCL (all hardware) === */
            IVE_SAD_CTRL_S sad_ctrl = {
                .enMode = IVE_SAD_MODE_MB_4X4,
                .enOutCtrl = IVE_SAD_OUT_CTRL_THRESH,
                .u16Thr = sad_thr, .u8MinVal = 0, .u8MaxVal = 255
            };
            ret = HI_MPI_IVE_SAD(&handle, &img_cur, &img_prev,
                                  &img_sad16, &img_sad, &sad_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            if (ccl_ok) {
                /* CCL on SAD threshold output — fully in hardware */
                IVE_CCL_CTRL_S ccl_ctrl = {
                    .enMode = IVE_CCL_MODE_4C,
                    .u16InitAreaThr = ccl_area_thr,
                    .u16Step = 2
                };
                ret = HI_MPI_IVE_CCL(&handle, &img_sad, &blob_mem,
                                      &ccl_ctrl, HI_TRUE);
                if (ret == HI_SUCCESS) ive_wait(handle);
                long long t2 = usec_now();
                t_ive += t2 - t1;

                /* CPU: just read the small blob struct (no pixel scanning) */
                HI_MPI_SYS_MmzFlushCache(blob_mem.u64PhyAddr,
                    (HI_VOID *)(HI_UL)blob_mem.u64VirAddr, blob_mem.u32Size);
                IVE_CCBLOB_S *blob = (IVE_CCBLOB_S *)(HI_UL)blob_mem.u64VirAddr;

                for (int r = 0; r < IVE_MAX_REGION_NUM; r++) {
                    if (blob->astRegion[r].u32Area == 0) continue;
                    printf("FRAME %d: (%d,%d)-(%d,%d) area=%d\n", i,
                           blob->astRegion[r].u16Left * BLOCK,
                           blob->astRegion[r].u16Top * BLOCK,
                           (blob->astRegion[r].u16Right + 1) * BLOCK,
                           (blob->astRegion[r].u16Bottom + 1) * BLOCK,
                           blob->astRegion[r].u32Area);
                    detected_frames++;
                }
                t_cpu += usec_now() - t2;
            }

        } else if (strcmp(mode, "abandoned") == 0) {
            /* === ABANDONED: full hardware pipeline ===
             * 1. Sub(cur, ref) → foreground diff
             * 2. Thresh → binary foreground mask
             * 3. Erode → remove noise
             * 4. Dilate → restore objects (result in img_bin)
             * 5. SAD(mask, zero) → blockify foreground (960×528 → 240×132)
             * 6. SAD(cur, prev) → motion blocks
             * 7. Thresh(invert motion) → stationary mask
             * 8. AND(foreground_blocks, stationary_blocks) → stationary foreground
             * 9. CCL → blob regions (all at 240×132, within CCL limits)
             */

            /* 1. Sub(cur, ref) */
            IVE_SUB_CTRL_S sub_ctrl = { .enMode = IVE_SUB_MODE_ABS };
            ret = HI_MPI_IVE_Sub(&handle, &img_cur, &img_ref, &img_diff,
                                  &sub_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 2. Thresh */
            IVE_THRESH_CTRL_S thr_ctrl = {
                .enMode = IVE_THRESH_MODE_BINARY,
                .u8LowThr = diff_thr, .u8MinVal = 0, .u8MaxVal = 255
            };
            ret = HI_MPI_IVE_Thresh(&handle, &img_diff, &img_bin,
                                     &thr_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 3. Erode */
            IVE_ERODE_CTRL_S erode_ctrl;
            memset(erode_ctrl.au8Mask, 255, 25);
            ret = HI_MPI_IVE_Erode(&handle, &img_bin, &img_diff, &erode_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 4. Dilate (result back in img_bin) */
            IVE_DILATE_CTRL_S dilate_ctrl;
            memset(dilate_ctrl.au8Mask, 255, 25);
            ret = HI_MPI_IVE_Dilate(&handle, &img_diff, &img_bin, &dilate_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 5. SAD(mask, zero) → blockify foreground to bw×bh.
             * Each block's SAD = sum of 255s in that 4×4 region.
             * Threshold at 255*4 = any block with >4 FG pixels → foreground block. */
            {
                IVE_SAD_CTRL_S fg_sad = {
                    .enMode = IVE_SAD_MODE_MB_4X4,
                    .enOutCtrl = IVE_SAD_OUT_CTRL_THRESH,
                    .u16Thr = 255 * 4, .u8MinVal = 0, .u8MaxVal = 255
                };
                ret = HI_MPI_IVE_SAD(&handle, &img_bin, &img_zero,
                                      &img_sad16_b, &img_fg_blk, &fg_sad, HI_TRUE);
                if (ret == HI_SUCCESS) ive_wait(handle);
            }

            /* 6. SAD(cur, prev) → motion blocks */
            {
                IVE_SAD_CTRL_S mot_sad = {
                    .enMode = IVE_SAD_MODE_MB_4X4,
                    .enOutCtrl = IVE_SAD_OUT_CTRL_THRESH,
                    .u16Thr = sad_thr, .u8MinVal = 0, .u8MaxVal = 255
                };
                ret = HI_MPI_IVE_SAD(&handle, &img_cur, &img_prev,
                                      &img_sad16_b, &img_mot_blk, &mot_sad, HI_TRUE);
                if (ret == HI_SUCCESS) ive_wait(handle);
            }

            /* 7. Invert motion: moving=255 → 0, stationary=0 → 255 */
            {
                IVE_THRESH_CTRL_S inv = {
                    .enMode = IVE_THRESH_MODE_BINARY,
                    .u8LowThr = 1, .u8MinVal = 255, .u8MaxVal = 0
                };
                ret = HI_MPI_IVE_Thresh(&handle, &img_mot_blk, &img_mot_blk,
                                         &inv, HI_TRUE);
                if (ret == HI_SUCCESS) ive_wait(handle);
            }

            /* 8. AND(foreground_blocks, stationary_blocks) */
            ret = HI_MPI_IVE_And(&handle, &img_fg_blk, &img_mot_blk,
                                  &img_stat_fg, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 9. CCL on stationary foreground blocks (bw×bh, within 64-720 range) */
            if (ccl_ok) {
                IVE_CCL_CTRL_S ccl_ctrl = {
                    .enMode = IVE_CCL_MODE_4C,
                    .u16InitAreaThr = ccl_area_thr,
                    .u16Step = 2
                };
                ret = HI_MPI_IVE_CCL(&handle, &img_stat_fg, &blob_mem,
                                      &ccl_ctrl, HI_TRUE);
                if (ret == HI_SUCCESS) ive_wait(handle);
            }

            long long t2 = usec_now();
            t_ive += t2 - t1;

            /* CPU: read blob struct and track stationarity (block-level coords) */
            if (ccl_ok) {
                HI_MPI_SYS_MmzFlushCache(blob_mem.u64PhyAddr,
                    (HI_VOID *)(HI_UL)blob_mem.u64VirAddr, blob_mem.u32Size);
                IVE_CCBLOB_S *blob = (IVE_CCBLOB_S *)(HI_UL)blob_mem.u64VirAddr;

                for (int r = 0; r < IVE_MAX_REGION_NUM; r++) {
                    if (blob->astRegion[r].u32Area == 0) continue;
                    int left = blob->astRegion[r].u16Left;
                    int top = blob->astRegion[r].u16Top;
                    int right = blob->astRegion[r].u16Right;
                    int bottom = blob->astRegion[r].u16Bottom;
                    int area = blob->astRegion[r].u32Area;

                    /* Track centroid for duration (block-level coords) */
                    int cx = (left + right) / 2, cy = (top + bottom) / 2;
                    int dur = 1;
                    for (int t = 0; t < prev_ntracked; t++) {
                        if (abs(prev_tracked[t].cx - cx) + abs(prev_tracked[t].cy - cy) <= 5) {
                            dur = prev_tracked[t].dur + 1;
                            break;
                        }
                    }
                    if (ntracked < 64)
                        tracked[ntracked++] = (Blob){cx, cy, dur};

                    if (dur >= ABANDON_FRAMES) {
                        printf("FRAME %d: ABANDONED (%d,%d)-(%d,%d) area=%d dur=%d\n",
                               i, left * BLOCK, top * BLOCK,
                               (right + 1) * BLOCK, (bottom + 1) * BLOCK,
                               area, dur);
                        detected_frames++;
                    }
                }
            }

            memcpy(prev_tracked, tracked, sizeof(Blob) * ntracked);
            prev_ntracked = ntracked;
            ntracked = 0;
            t_cpu += usec_now() - t2;

        } else if (strcmp(mode, "lpr") == 0) {
            /* === LICENSE PLATE REGION: Sobel → 16to8 → Thresh → Dilate → Erode → CCL === */

            /* 1. Sobel (horizontal edges — character strokes are horizontal) */
            IVE_SOBEL_CTRL_S sobel_ctrl = { .enOutCtrl = IVE_SOBEL_OUT_CTRL_VER };
            HI_S8 sobel_mask[25] = {0,0,0,0,0, 0,-1,-2,-1,0, 0,0,0,0,0, 0,1,2,1,0, 0,0,0,0,0};
            memcpy(sobel_ctrl.as8Mask, sobel_mask, 25);
            ret = HI_MPI_IVE_Sobel(&handle, &img_cur, &img_sobel_h, &img_sobel_v,
                                    &sobel_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 2. 16BitTo8Bit (S16→U8 absolute value = edge magnitude) */
            IVE_16BIT_TO_8BIT_CTRL_S cvt_ctrl = {
                .enMode = IVE_16BIT_TO_8BIT_MODE_S16_TO_U8_ABS,
                .u16Denominator = 1, .u8Numerator = 1, .s8Bias = 0
            };
            ret = HI_MPI_IVE_16BitTo8Bit(&handle, &img_sobel_v, &img_edge,
                                          &cvt_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 3. Thresh (binary edge map) */
            IVE_THRESH_CTRL_S edge_thr = {
                .enMode = IVE_THRESH_MODE_BINARY,
                .u8LowThr = diff_thr, .u8MinVal = 0, .u8MaxVal = 255
            };
            ret = HI_MPI_IVE_Thresh(&handle, &img_edge, &img_bin,
                                     &edge_thr, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 4. Dilate (close gaps between characters → merge into plate blob) */
            IVE_DILATE_CTRL_S dil_ctrl;
            memset(dil_ctrl.au8Mask, 255, 25);
            ret = HI_MPI_IVE_Dilate(&handle, &img_bin, &img_diff, &dil_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 5. Erode (remove noise, tighten boundaries) */
            IVE_ERODE_CTRL_S ero_ctrl;
            memset(ero_ctrl.au8Mask, 255, 25);
            ret = HI_MPI_IVE_Erode(&handle, &img_diff, &img_bin, &ero_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* 6. CCL on edge blobs (full frame if within 720px, else blockify) */
            int ccl_on_full = (W >= 64 && H >= 64 && W <= 720 && H <= 640);
            if (ccl_on_full) {
                IVE_CCL_CTRL_S ccl_ctrl = {
                    .enMode = IVE_CCL_MODE_4C,
                    .u16InitAreaThr = ccl_area_thr,
                    .u16Step = 2
                };
                ret = HI_MPI_IVE_CCL(&handle, &img_bin, &blob_mem,
                                      &ccl_ctrl, HI_TRUE);
                if (ret == HI_SUCCESS) ive_wait(handle);
            } else {
                /* Blockify via SAD(mask,zero) then CCL on block-level */
                IVE_SAD_CTRL_S blk_sad = {
                    .enMode = IVE_SAD_MODE_MB_4X4,
                    .enOutCtrl = IVE_SAD_OUT_CTRL_THRESH,
                    .u16Thr = 255 * 4, .u8MinVal = 0, .u8MaxVal = 255
                };
                ret = HI_MPI_IVE_SAD(&handle, &img_bin, &img_zero,
                                      &img_sad16_b, &img_fg_blk, &blk_sad, HI_TRUE);
                if (ret == HI_SUCCESS) ive_wait(handle);

                if (ccl_ok) {
                    IVE_CCL_CTRL_S ccl_ctrl = {
                        .enMode = IVE_CCL_MODE_4C,
                        .u16InitAreaThr = ccl_area_thr,
                        .u16Step = 2
                    };
                    ret = HI_MPI_IVE_CCL(&handle, &img_fg_blk, &blob_mem,
                                          &ccl_ctrl, HI_TRUE);
                    if (ret == HI_SUCCESS) ive_wait(handle);
                }
            }

            long long t2 = usec_now();
            t_ive += t2 - t1;

            /* 7. CPU: read blob struct, filter by plate aspect ratio */
            HI_MPI_SYS_MmzFlushCache(blob_mem.u64PhyAddr,
                (HI_VOID *)(HI_UL)blob_mem.u64VirAddr, blob_mem.u32Size);
            IVE_CCBLOB_S *blob = (IVE_CCBLOB_S *)(HI_UL)blob_mem.u64VirAddr;

            int scale = ccl_on_full ? 1 : BLOCK;
            for (int r = 0; r < IVE_MAX_REGION_NUM; r++) {
                if (blob->astRegion[r].u32Area == 0) continue;
                int left = blob->astRegion[r].u16Left * scale;
                int top = blob->astRegion[r].u16Top * scale;
                int right = blob->astRegion[r].u16Right * scale + (ccl_on_full ? 0 : BLOCK);
                int bottom = blob->astRegion[r].u16Bottom * scale + (ccl_on_full ? 0 : BLOCK);
                int area = blob->astRegion[r].u32Area;
                int rw = right - left;
                int rh = bottom - top;
                if (rh < 1) continue;
                float ratio = (float)rw / rh;

                /* Filter: plate-like aspect ratio 2.0-6.0, minimum size, bottom 80% of frame */
                if (ratio >= 2.0f && ratio <= 6.0f && area >= 8 && top > H / 5) {
                    printf("FRAME %d: PLATE (%d,%d)-(%d,%d) area=%d ratio=%.1f\n",
                           i, left, top, right, bottom, area, ratio);
                    detected_frames++;
                }
            }
            t_cpu += usec_now() - t2;
        }

        /* Swap: IVE DMA copy cur→prev (hardware, no CPU memcpy) */
        t0 = usec_now();
        IVE_DMA_CTRL_S dma_ctrl = { .enMode = IVE_DMA_MODE_DIRECT_COPY };
        IVE_DATA_S dma_src, dma_dst;
        dma_src.u64PhyAddr = img_cur.au64PhyAddr[0];
        dma_src.u64VirAddr = img_cur.au64VirAddr[0];
        dma_src.u32Width = W; dma_src.u32Height = H;
        dma_src.u32Stride = img_cur.au32Stride[0];
        dma_dst.u64PhyAddr = img_prev.au64PhyAddr[0];
        dma_dst.u64VirAddr = img_prev.au64VirAddr[0];
        dma_dst.u32Width = W; dma_dst.u32Height = H;
        dma_dst.u32Stride = img_prev.au32Stride[0];
        ret = HI_MPI_IVE_DMA(&handle, &dma_src, &dma_dst, &dma_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) ive_wait(handle);
        t_ive += usec_now() - t0;
    }

    fclose(fp);

    /* Print benchmark results */
    int processed = nframes - 1;
    fprintf(stderr, "\n=== Benchmark (%d frames, %dx%d) ===\n", processed, W, H);
    fprintf(stderr, "  I/O (NFS read):     %lld ms (%.1f ms/frame)\n",
            t_io / 1000, (double)t_io / 1000 / processed);
    fprintf(stderr, "  IVE hardware:       %lld ms (%.1f ms/frame)\n",
            t_ive / 1000, (double)t_ive / 1000 / processed);
    fprintf(stderr, "  CPU (blob parse):   %lld ms (%.1f ms/frame)\n",
            t_cpu / 1000, (double)t_cpu / 1000 / processed);
    fprintf(stderr, "  Total:              %lld ms (%.1f ms/frame = %.1f fps)\n",
            (t_io + t_ive + t_cpu) / 1000,
            (double)(t_io + t_ive + t_cpu) / 1000 / processed,
            1000000.0 * processed / (t_io + t_ive + t_cpu));
    fprintf(stderr, "  Detections:         %d\n", detected_frames);

    free(frame_data);
    free_img(&img_cur); free_img(&img_prev); free_img(&img_ref);
    free_img(&img_diff); free_img(&img_bin); free_img(&img_sad);
    free_img(&img_fg_blk); free_img(&img_mot_blk); free_img(&img_stat_fg);
    free_img(&img_zero);
    HI_MPI_SYS_MmzFree(blob_mem.u64PhyAddr, (HI_VOID *)(HI_UL)blob_mem.u64VirAddr);
    HI_MPI_SYS_Exit();
    return 0;
}
