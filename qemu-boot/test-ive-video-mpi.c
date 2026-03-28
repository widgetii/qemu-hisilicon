/*
 * IVE Motion Detection + Abandoned Object on Real Hardware via MPI API
 *
 * Processes raw grayscale frames using real IVE silicon through the
 * official HiSilicon MPI API (libmpi.so + libive.so → kernel module).
 *
 * Output format matches test-ive-video.c (QEMU version) for comparison.
 *
 * Build:
 *   CC=arm-openipc-linux-musleabi-gcc
 *   SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
 *   LIBS=~/git/firmware/output-hi3516ev300_lite/target/usr/lib
 *   $CC -o test-ive-video-mpi test-ive-video-mpi.c \
 *       -I$SDK/include -L$LIBS -lmpi -live -lVoiceEngine -lupvqe -ldnvqe -lsecurec \
 *       -Wl,-rpath,/usr/lib -Wl,--allow-shlib-undefined
 *
 * Run: load_hisilicon -i; killall majestic; ./test-ive-video-mpi /tmp/frames.bin
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdint.h>

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
#include "mpi_sys.h"
#include "mpi_ive.h"

#define W       64
#define H       64
#define FRAME_SZ (W * H)
#define BLOCK   4
#define BW      (W / BLOCK)
#define BH      (H / BLOCK)
#define SAD_SZ  (BW * BH)
#define BLOB_SZ 3056
#define STRIDE  ((W + 15) & ~15)

/* Abandon detection params */
#define LEARN_FRAMES    20
#define ABANDON_FRAMES  30
#define DIFF_THR        40
#define MOTION_THR      100
#define AREA_THR        8

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

static void read_img(IVE_IMAGE_S *img, uint8_t *data) {
    HI_MPI_SYS_MmzFlushCache(img->au64PhyAddr[0],
        (HI_VOID *)(HI_UL)img->au64VirAddr[0],
        img->au32Stride[0] * img->u32Height);
    uint8_t *v = (uint8_t *)(HI_UL)img->au64VirAddr[0];
    for (HI_U32 y = 0; y < img->u32Height; y++)
        memcpy(data + y * img->u32Width, v + y * img->au32Stride[0], img->u32Width);
}

/* Stationary blob tracker */
typedef struct { int cx, cy, dur; } Blob;
static Blob tracked[16], prev_tracked[16];
static int ntracked, prev_ntracked;

static void parse_ccl_and_track(uint8_t *blob_data, uint8_t *cur, uint8_t *prev,
                                int frame_idx, const char *mode) {
    uint8_t nreg = blob_data[5];
    Blob new_tracked[16];
    int new_n = 0;

    for (int r = 0; r < nreg && r < 16; r++) {
        uint8_t *p = blob_data + 8 + r * 12;
        uint32_t area = p[0]|(p[1]<<8)|(p[2]<<16)|(p[3]<<24);
        uint16_t left = p[4]|(p[5]<<8), top = p[6]|(p[7]<<8);
        uint16_t right = p[8]|(p[9]<<8), bottom = p[10]|(p[11]<<8);

        if (strcmp(mode, "md") == 0) {
            /* Motion detection: scale block coords to pixels */
            printf("FRAME %d: (%d,%d)-(%d,%d) area=%d\n", frame_idx,
                   left * BLOCK, top * BLOCK, (right+1) * BLOCK, (bottom+1) * BLOCK, area);
        } else {
            /* Abandoned: check if stationary (low SAD with prev frame) */
            int sad = 0;
            if (prev) {
                for (int y = top; y <= bottom && y < H; y++)
                    for (int x = left; x <= right && x < W; x++)
                        sad += abs((int)cur[y*W+x] - (int)prev[y*W+x]);
            }
            if (sad >= MOTION_THR) continue; /* moving → skip */

            int cx = (left + right) / 2, cy = (top + bottom) / 2;
            int dur = 1;
            for (int t = 0; t < ntracked; t++) {
                if (abs(tracked[t].cx - cx) + abs(tracked[t].cy - cy) <= 3) {
                    dur = tracked[t].dur + 1; break;
                }
            }
            if (new_n < 16) new_tracked[new_n++] = (Blob){cx, cy, dur};

            if (dur >= ABANDON_FRAMES) {
                printf("FRAME %d: ABANDONED (%d,%d)-(%d,%d) area=%d dur=%d\n",
                       frame_idx, left, top, right, bottom, area, dur);
            }
        }
    }

    if (strcmp(mode, "abandoned") == 0) {
        memcpy(tracked, new_tracked, sizeof(Blob) * new_n);
        ntracked = new_n;
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <frames.bin> [md|abandoned]\n", argv[0]);
        return 1;
    }
    const char *mode = (argc > 2) ? argv[2] : "md";

    FILE *fp = fopen(argv[1], "rb");
    if (!fp) { perror(argv[1]); return 1; }
    fseek(fp, 0, SEEK_END);
    int nframes = ftell(fp) / FRAME_SZ;
    fseek(fp, 0, SEEK_SET);

    HI_S32 ret = HI_MPI_SYS_Init();
    if (ret != HI_SUCCESS) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* Allocate images */
    IVE_IMAGE_S img_cur, img_prev, img_ref, img_diff, img_bin, img_sad, img_sad16;
    IVE_MEM_INFO_S blob_mem;
    alloc_img(&img_cur, W, H);
    alloc_img(&img_prev, W, H);
    alloc_img(&img_ref, W, H);
    alloc_img(&img_diff, W, H);
    alloc_img(&img_bin, W, H);
    alloc_img(&img_sad, BW, BH);
    /* SAD output is 16-bit */
    memset(&img_sad16, 0, sizeof(img_sad16));
    img_sad16.enType = IVE_IMAGE_TYPE_U16C1;
    img_sad16.u32Width = BW; img_sad16.u32Height = BH;
    img_sad16.au32Stride[0] = (BW * 2 + 15) & ~15;
    HI_MPI_SYS_MmzAlloc(&img_sad16.au64PhyAddr[0],
        (HI_VOID **)&img_sad16.au64VirAddr[0], NULL, HI_NULL,
        img_sad16.au32Stride[0] * BH);
    memset(&blob_mem, 0, sizeof(blob_mem));
    blob_mem.u32Size = BLOB_SZ;
    HI_MPI_SYS_MmzAlloc(&blob_mem.u64PhyAddr, (HI_VOID **)&blob_mem.u64VirAddr,
                         NULL, HI_NULL, BLOB_SZ);

    uint8_t frame_data[FRAME_SZ], prev_data[FRAME_SZ], ref_data[FRAME_SZ];
    IVE_HANDLE handle;

    if (strcmp(mode, "abandoned") == 0) {
        /* Phase 1: Learn reference background */
        int bg_sum[FRAME_SZ];
        memset(bg_sum, 0, sizeof(bg_sum));
        for (int i = 0; i < LEARN_FRAMES && i < nframes; i++) {
            fread(frame_data, 1, FRAME_SZ, fp);
            for (int j = 0; j < FRAME_SZ; j++) bg_sum[j] += frame_data[j];
        }
        for (int j = 0; j < FRAME_SZ; j++) ref_data[j] = bg_sum[j] / LEARN_FRAMES;
        fill_img(&img_ref, ref_data);
        fseek(fp, 0, SEEK_SET);
        memset(prev_data, 0, FRAME_SZ);
    }

    /* Read first frame */
    fread(prev_data, 1, FRAME_SZ, fp);
    fill_img(&img_prev, prev_data);

    for (int i = 1; i < nframes; i++) {
        fread(frame_data, 1, FRAME_SZ, fp);
        fill_img(&img_cur, frame_data);

        if (strcmp(mode, "md") == 0) {
            /* Motion detection: SAD(cur, prev) → CCL */
            IVE_SAD_CTRL_S sad_ctrl = {
                .enMode = IVE_SAD_MODE_MB_4X4,
                .enOutCtrl = IVE_SAD_OUT_CTRL_THRESH,
                .u16Thr = 50, .u8MinVal = 0, .u8MaxVal = 255
            };
            ret = HI_MPI_IVE_SAD(&handle, &img_cur, &img_prev,
                                  &img_sad16, &img_sad, &sad_ctrl, HI_TRUE);
            if (ret != HI_SUCCESS) {
                if (i <= 3) fprintf(stderr, "SAD err=0x%x frame=%d\n", ret, i);
            } else {
                ive_wait(handle);
            }
            if (i <= 3) {
                /* Debug: check SAD threshold output */
                uint8_t sad_dbg[64];
                read_img(&img_sad, sad_dbg);
                int nz = 0;
                for (int j = 0; j < BW*BH && j < 64; j++) if (sad_dbg[j]) nz++;
                fprintf(stderr, "frame %d: SAD ret=0x%x nonzero=%d/%d thr[0..3]=%d,%d,%d,%d\n",
                    i, ret, nz, BW*BH, sad_dbg[0], sad_dbg[1], sad_dbg[2], sad_dbg[3]);
            }

            /* Read SAD threshold output and find motion regions directly.
             * CCL requires min 64×64 but SAD output is BW×BH (16×16).
             * Scan non-zero blocks and compute bounding boxes manually. */
            {
                uint8_t sad_out[BW * BH];
                read_img(&img_sad, sad_out);
                /* Find bounding box of all non-zero blocks */
                int minx = BW, miny = BH, maxx = 0, maxy = 0, area = 0;
                for (int y = 0; y < BH; y++)
                    for (int x = 0; x < BW; x++)
                        if (sad_out[y * BW + x]) {
                            area++;
                            if (x < minx) minx = x;
                            if (y < miny) miny = y;
                            if (x > maxx) maxx = x;
                            if (y > maxy) maxy = y;
                        }
                if (area >= 4) {
                    printf("FRAME %d: (%d,%d)-(%d,%d) area=%d\n", i,
                           minx * BLOCK, miny * BLOCK,
                           (maxx + 1) * BLOCK, (maxy + 1) * BLOCK, area);
                }
            }

        } else {
            /* Abandoned: Sub(cur, ref) → Thresh → CCL */
            IVE_SUB_CTRL_S sub_ctrl = { .enMode = IVE_SUB_MODE_ABS };
            ret = HI_MPI_IVE_Sub(&handle, &img_cur, &img_ref, &img_diff,
                                  &sub_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* Debug Sub output */
            if (i == 50 || i == 80 || i == 100) {
                uint8_t dbg[64];
                read_img(&img_diff, dbg);
                int nz = 0;
                for (int j = 0; j < W*H; j++) {
                    uint8_t *v = (uint8_t *)(HI_UL)img_diff.au64VirAddr[0];
                    if (v[j]) nz++;
                }
                fprintf(stderr, "frame %d: Sub nonzero=%d diff[0..3]=%d,%d,%d,%d\n",
                    i, nz, dbg[0], dbg[1], dbg[2], dbg[3]);
            }

            IVE_THRESH_CTRL_S thr_ctrl = {
                .enMode = IVE_THRESH_MODE_BINARY,
                .u8LowThr = DIFF_THR, .u8MinVal = 0, .u8MaxVal = 255
            };
            ret = HI_MPI_IVE_Thresh(&handle, &img_diff, &img_bin,
                                     &thr_ctrl, HI_TRUE);
            if (ret == HI_SUCCESS) ive_wait(handle);

            /* Debug Thresh output */
            if (i == 50 || i == 80 || i == 100) {
                uint8_t *v = (uint8_t *)(HI_UL)img_bin.au64VirAddr[0];
                int nz = 0;
                for (int j = 0; j < W*H; j++) if (v[j]) nz++;
                fprintf(stderr, "frame %d: Thresh nonzero=%d\n", i, nz);
            }

            /* Read binary mask and find stationary foreground regions.
             * CCL requires min 64×64, our images are 64×64 but Sub→Thresh
             * output may still have issues. Scan directly for robustness. */
            {
                uint8_t bin_out[W * H];
                read_img(&img_bin, bin_out);
                /* Find bounding box of foreground */
                int minx = W, miny = H, maxx = 0, maxy = 0, area = 0;
                for (int y = 0; y < H; y++)
                    for (int x = 0; x < W; x++)
                        if (bin_out[y * W + x]) {
                            area++;
                            if (x < minx) minx = x;
                            if (y < miny) miny = y;
                            if (x > maxx) maxx = x;
                            if (y > maxy) maxy = y;
                        }
                if (area >= AREA_THR) {
                    /* Check if stationary (low SAD with prev frame in this region) */
                    int sad = 0;
                    for (int y = miny; y <= maxy; y++)
                        for (int x = minx; x <= maxx; x++)
                            sad += abs((int)frame_data[y*W+x] - (int)prev_data[y*W+x]);

                    if (sad < MOTION_THR) {
                        int cx = (minx + maxx) / 2, cy = (miny + maxy) / 2;

                        /* Match against previous frame's tracked blobs */
                        int dur = 1;
                        for (int t = 0; t < prev_ntracked; t++) {
                            if (abs(prev_tracked[t].cx - cx) + abs(prev_tracked[t].cy - cy) <= 5) {
                                dur = prev_tracked[t].dur + 1;
                                break;
                            }
                        }

                        /* Store for next frame */
                        if (ntracked < 16)
                            tracked[ntracked++] = (Blob){cx, cy, dur};

                        if (dur >= ABANDON_FRAMES) {
                            printf("FRAME %d: ABANDONED (%d,%d)-(%d,%d) area=%d dur=%d\n",
                                   i, minx, miny, maxx, maxy, area, dur);
                        }
                    }
                }
                /* Save current tracked for next frame comparison */
                memcpy(prev_tracked, tracked, sizeof(Blob) * ntracked);
                prev_ntracked = ntracked;
                ntracked = 0;
            }
        }

        /* Swap */
        memcpy(prev_data, frame_data, FRAME_SZ);
        fill_img(&img_prev, prev_data);
    }

    fclose(fp);
    HI_MPI_SYS_MmzFree(blob_mem.u64PhyAddr, (HI_VOID *)(HI_UL)blob_mem.u64VirAddr);
    free_img(&img_cur); free_img(&img_prev); free_img(&img_ref);
    free_img(&img_diff); free_img(&img_bin); free_img(&img_sad);
    HI_MPI_SYS_Exit();
    return 0;
}
