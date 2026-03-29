/*
 * IVE-Accelerated QR Code Recognition PoC
 *
 * Benchmarks two approaches side-by-side on the same input:
 *   CPU-only:  quirc decodes every frame (baseline)
 *   IVE-gated: IVE Thresh→Erode→Dilate→CCL detects square-ish blobs;
 *              quirc only runs when ≥N candidates found (saves CPU)
 *
 * Build (from qemu-boot/):
 *   CC=arm-openipc-linux-musleabi-gcc
 *   SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
 *   LIBS=~/git/firmware/output-hi3516ev300_lite/target/usr/lib
 *   $CC -O2 -o test-qr-ive test-qr-ive.c \
 *       quirc/lib/quirc.c quirc/lib/identify.c quirc/lib/decode.c quirc/lib/version_db.c \
 *       -I$SDK/include -Iquirc/lib \
 *       -L$LIBS -lmpi -live -lVoiceEngine -lupvqe -ldnvqe -lsecurec \
 *       -Wl,-rpath,/usr/lib -Wl,--allow-shlib-undefined -lm
 *
 * Run: killall majestic; ./test-qr-ive input.y4m [--thresh=128] [--min-area=16] [--min-candidates=3] [-v]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdint.h>
#include <sys/time.h>
#include <math.h>

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

#include "quirc.h"

/* ---------- Utilities (from test-ive-video-mpi.c) ---------- */

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

/* ---------- IVE candidate detection ---------- */

/*
 * Detect square-ish blobs using IVE hardware pipeline.
 * Returns number of blobs with roughly square aspect ratio.
 *
 * Pipeline: Thresh → Erode → Dilate → CCL → filter by aspect ratio
 *
 * For frames wider than 720px (CCL limit), uses SAD 4×4 block mode
 * to downscale first.
 */
static int ive_detect_qr_candidates(
    IVE_IMAGE_S *img_src,
    IVE_IMAGE_S *img_bin,
    IVE_IMAGE_S *img_tmp,
    IVE_MEM_INFO_S *blob_mem,
    /* For wide frames: SAD downscale buffers (may be NULL if W<=720) */
    IVE_IMAGE_S *img_blk,
    IVE_IMAGE_S *img_sad16,
    IVE_IMAGE_S *img_zero,
    int W, int H,
    int thresh,
    int min_blob_area,
    int min_candidates,
    int verbose)
{
    IVE_HANDLE handle;
    HI_S32 ret;

    /* 1. Thresh: binarize (dark pixels = QR modules) */
    IVE_THRESH_CTRL_S thr_ctrl = {
        .enMode = IVE_THRESH_MODE_BINARY,
        .u8LowThr = thresh, .u8MinVal = 255, .u8MaxVal = 0
    };
    ret = HI_MPI_IVE_Thresh(&handle, img_src, img_bin, &thr_ctrl, HI_TRUE);
    if (ret == HI_SUCCESS) ive_wait(handle);

    /* 2. Erode (5×5) — remove noise, separate thin features */
    IVE_ERODE_CTRL_S erode_ctrl;
    memset(erode_ctrl.au8Mask, 255, 25);
    ret = HI_MPI_IVE_Erode(&handle, img_bin, img_tmp, &erode_ctrl, HI_TRUE);
    if (ret == HI_SUCCESS) ive_wait(handle);

    /* 3. Dilate (5×5) — merge nearby dark regions */
    IVE_DILATE_CTRL_S dilate_ctrl;
    memset(dilate_ctrl.au8Mask, 255, 25);
    ret = HI_MPI_IVE_Dilate(&handle, img_tmp, img_bin, &dilate_ctrl, HI_TRUE);
    if (ret == HI_SUCCESS) ive_wait(handle);

    /* 4. CCL — find connected components */
    int use_sad = (W > 720 || H > 640);
    IVE_IMAGE_S *ccl_input = img_bin;
    int scale = 1;

    if (use_sad && img_blk && img_sad16 && img_zero) {
        /* Downscale via SAD(mask, zero) to block-level */
        IVE_SAD_CTRL_S sad_ctrl = {
            .enMode = IVE_SAD_MODE_MB_4X4,
            .enOutCtrl = IVE_SAD_OUT_CTRL_THRESH,
            .u16Thr = 255 * 4, .u8MinVal = 0, .u8MaxVal = 255
        };
        ret = HI_MPI_IVE_SAD(&handle, img_bin, img_zero,
                              img_sad16, img_blk, &sad_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) ive_wait(handle);
        ccl_input = img_blk;
        scale = 4;
    }

    int cw = ccl_input->u32Width, ch = ccl_input->u32Height;
    if (cw < 64 || ch < 64 || cw > 720 || ch > 640) {
        if (verbose)
            fprintf(stderr, "  CCL skip: %dx%d out of range\n", cw, ch);
        return 0;
    }

    IVE_CCL_CTRL_S ccl_ctrl = {
        .enMode = IVE_CCL_MODE_4C,
        .u16InitAreaThr = min_blob_area,
        .u16Step = 2
    };
    ret = HI_MPI_IVE_CCL(&handle, ccl_input, blob_mem, &ccl_ctrl, HI_TRUE);
    if (ret == HI_SUCCESS) ive_wait(handle);

    /* 5. CPU: read blob struct, count square-ish blobs */
    HI_MPI_SYS_MmzFlushCache(blob_mem->u64PhyAddr,
        (HI_VOID *)(HI_UL)blob_mem->u64VirAddr, blob_mem->u32Size);
    IVE_CCBLOB_S *blob = (IVE_CCBLOB_S *)(HI_UL)blob_mem->u64VirAddr;

    int n_square = 0;
    for (int r = 0; r < IVE_MAX_REGION_NUM; r++) {
        if (blob->astRegion[r].u32Area == 0) continue;
        int rw = (blob->astRegion[r].u16Right - blob->astRegion[r].u16Left + 1) * scale;
        int rh = (blob->astRegion[r].u16Bottom - blob->astRegion[r].u16Top + 1) * scale;
        if (rh < 1 || rw < 1) continue;
        float aspect = (float)rw / rh;
        if (aspect >= 0.3f && aspect <= 3.0f) {
            n_square++;
            if (verbose)
                fprintf(stderr, "  blob[%d]: (%d,%d)-(%d,%d) %dx%d aspect=%.2f area=%d\n",
                        r,
                        blob->astRegion[r].u16Left * scale,
                        blob->astRegion[r].u16Top * scale,
                        (blob->astRegion[r].u16Right + 1) * scale,
                        (blob->astRegion[r].u16Bottom + 1) * scale,
                        rw, rh, aspect, blob->astRegion[r].u32Area);
        }
    }
    return n_square;
}

/* ---------- quirc decode helper ---------- */

static const char *ecc_names[] = {"M", "L", "H", "Q"};

static int quirc_decode_frame(struct quirc *qr, const uint8_t *frame,
                              int W, int H, int frame_idx)
{
    uint8_t *buf = quirc_begin(qr, NULL, NULL);
    memcpy(buf, frame, W * H);
    quirc_end(qr);

    int nqr = quirc_count(qr);
    int decoded = 0;
    for (int j = 0; j < nqr; j++) {
        struct quirc_code code;
        struct quirc_data data;
        quirc_extract(qr, j, &code);
        if (quirc_decode(&code, &data) == QUIRC_SUCCESS) {
            printf("FRAME %d: QR \"%.*s\" (v%d, ECC-%s)\n",
                   frame_idx, data.payload_len, data.payload,
                   data.version,
                   (data.ecc_level >= 0 && data.ecc_level <= 3)
                       ? ecc_names[data.ecc_level] : "?");
            decoded++;
        }
    }
    return decoded;
}

/* ---------- main ---------- */

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,
            "Usage: %s <input.y4m> [--thresh=128] [--min-area=16] "
            "[--min-candidates=3] [-v]\n", argv[0]);
        return 1;
    }

    /* Parse options */
    int thresh = 128, min_area = 16, min_candidates = 3, verbose = 0;
    for (int a = 2; a < argc; a++) {
        if (strncmp(argv[a], "--thresh=", 9) == 0) thresh = atoi(argv[a] + 9);
        else if (strncmp(argv[a], "--min-area=", 11) == 0) min_area = atoi(argv[a] + 11);
        else if (strncmp(argv[a], "--min-candidates=", 17) == 0) min_candidates = atoi(argv[a] + 17);
        else if (strcmp(argv[a], "-v") == 0) verbose = 1;
    }

    FILE *fp = fopen(argv[1], "rb");
    if (!fp) { perror(argv[1]); return 1; }

    /* Parse Y4M header */
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

    /* Count frames */
    long file_pos = ftell(fp);
    fseek(fp, 0, SEEK_END);
    long file_end = ftell(fp);
    fseek(fp, file_pos, SEEK_SET);
    int nframes = (int)((file_end - file_pos) / (frame_sz + (is_y4m ? 6 : 0)));

    fprintf(stderr, "Input: %s %dx%d, %d frames\n",
            is_y4m ? "Y4M" : "raw", W, H, nframes);
    fprintf(stderr, "Params: thresh=%d min_area=%d min_candidates=%d\n",
            thresh, min_area, min_candidates);

    /* Init MPI */
    HI_S32 ret = HI_MPI_SYS_Init();
    if (ret != HI_SUCCESS) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* Allocate IVE images */
    IVE_IMAGE_S img_cur, img_bin, img_tmp;
    alloc_img(&img_cur, W, H);
    alloc_img(&img_bin, W, H);
    alloc_img(&img_tmp, W, H);

    /* SAD downscale buffers (only needed if W>720 or H>640) */
    int need_sad = (W > 720 || H > 640);
    int bw = W / 4, bh = H / 4;
    IVE_IMAGE_S img_blk, img_zero, img_sad16;
    memset(&img_blk, 0, sizeof(img_blk));
    memset(&img_zero, 0, sizeof(img_zero));
    memset(&img_sad16, 0, sizeof(img_sad16));

    if (need_sad) {
        alloc_img(&img_blk, bw, bh);
        alloc_img(&img_zero, W, H);
        memset((void *)(HI_UL)img_zero.au64VirAddr[0], 0, img_zero.au32Stride[0] * H);
        HI_MPI_SYS_MmzFlushCache(img_zero.au64PhyAddr[0],
            (HI_VOID *)(HI_UL)img_zero.au64VirAddr[0], img_zero.au32Stride[0] * H);

        memset(&img_sad16, 0, sizeof(img_sad16));
        img_sad16.enType = IVE_IMAGE_TYPE_U16C1;
        img_sad16.u32Width = bw; img_sad16.u32Height = bh;
        img_sad16.au32Stride[0] = (bw * 2 + 15) & ~15;
        HI_MPI_SYS_MmzAlloc(&img_sad16.au64PhyAddr[0],
            (HI_VOID **)&img_sad16.au64VirAddr[0], NULL, HI_NULL,
            img_sad16.au32Stride[0] * bh);
    }

    /* CCL blob output */
    IVE_MEM_INFO_S blob_mem;
    memset(&blob_mem, 0, sizeof(blob_mem));
    blob_mem.u32Size = sizeof(IVE_CCBLOB_S);
    HI_MPI_SYS_MmzAlloc(&blob_mem.u64PhyAddr, (HI_VOID **)&blob_mem.u64VirAddr,
                         NULL, HI_NULL, blob_mem.u32Size);

    /* Init quirc (two instances: one for CPU-only pass, one for IVE-gated pass) */
    struct quirc *qr_cpu = quirc_new();
    struct quirc *qr_ive = quirc_new();
    quirc_resize(qr_cpu, W, H);
    quirc_resize(qr_ive, W, H);

    /* Frame buffer */
    uint8_t *frame_data = malloc(frame_sz);

    /* Benchmark accumulators (microseconds) */
    long long t_cpu_total = 0;
    long long t_ive_gate = 0, t_ive_quirc = 0;
    int cpu_decoded = 0, ive_decoded = 0;
    int ive_candidates_frames = 0;

    fprintf(stderr, "\n--- Processing %d frames ---\n", nframes);

    for (int i = 0; i < nframes; i++) {
        if (!read_y4m_frame(fp, frame_data, frame_sz, is_y4m)) break;

        /* ========== Pass 1: CPU-only (quirc every frame) ========== */
        long long t0 = usec_now();
        cpu_decoded += quirc_decode_frame(qr_cpu, frame_data, W, H, i);
        t_cpu_total += usec_now() - t0;

        /* ========== Pass 2: IVE-gated ========== */

        /* Copy frame to MMZ for IVE */
        fill_img(&img_cur, frame_data);

        /* IVE candidate detection */
        t0 = usec_now();
        int ncand = ive_detect_qr_candidates(
            &img_cur, &img_bin, &img_tmp, &blob_mem,
            need_sad ? &img_blk : NULL,
            need_sad ? &img_sad16 : NULL,
            need_sad ? &img_zero : NULL,
            W, H, thresh, min_area, min_candidates, verbose);
        long long t_gate = usec_now() - t0;
        t_ive_gate += t_gate;

        if (verbose)
            fprintf(stderr, "FRAME %d: %d candidates (gate %.1f ms)\n",
                    i, ncand, t_gate / 1000.0);

        if (ncand >= min_candidates) {
            ive_candidates_frames++;

            /* Run quirc only on frames with candidates */
            t0 = usec_now();
            ive_decoded += quirc_decode_frame(qr_ive, frame_data, W, H, i);
            t_ive_quirc += usec_now() - t0;
        }
    }

    fclose(fp);

    /* ========== Benchmark summary ========== */
    fprintf(stderr, "\n=== QR Detection Benchmark (%d frames, %dx%d) ===\n\n",
            nframes, W, H);

    fprintf(stderr, "  CPU-only (quirc every frame):\n");
    fprintf(stderr, "    Total:            %lld ms (%.1f ms/frame)\n",
            t_cpu_total / 1000, (double)t_cpu_total / 1000 / nframes);
    fprintf(stderr, "    Decoded:          %d QR codes\n\n", cpu_decoded);

    fprintf(stderr, "  IVE-gated:\n");
    fprintf(stderr, "    IVE gate:         %lld ms (%.1f ms/frame)\n",
            t_ive_gate / 1000, (double)t_ive_gate / 1000 / nframes);
    if (ive_candidates_frames > 0)
        fprintf(stderr, "    quirc decode:     %lld ms (%.1f ms/invocation, %d invocations)\n",
                t_ive_quirc / 1000, (double)t_ive_quirc / 1000 / ive_candidates_frames,
                ive_candidates_frames);
    else
        fprintf(stderr, "    quirc decode:     0 ms (0 invocations)\n");
    long long t_ive_total = t_ive_gate + t_ive_quirc;
    fprintf(stderr, "    Total:            %lld ms (%.1f ms/frame)\n",
            t_ive_total / 1000, (double)t_ive_total / 1000 / nframes);
    fprintf(stderr, "    Frames filtered:  %d / %d (%.0f%% skipped quirc)\n",
            nframes - ive_candidates_frames, nframes,
            100.0 * (nframes - ive_candidates_frames) / nframes);
    fprintf(stderr, "    Decoded:          %d QR codes\n\n", ive_decoded);

    if (t_ive_total > 0 && t_cpu_total > 0) {
        fprintf(stderr, "  Speedup:            %.2fx overall\n",
                (double)t_cpu_total / t_ive_total);
        fprintf(stderr, "  CPU savings:        quirc avoided on %.0f%% of frames\n",
                100.0 * (nframes - ive_candidates_frames) / nframes);
    }

    if (cpu_decoded != ive_decoded) {
        fprintf(stderr, "\n  WARNING: decode mismatch! CPU=%d IVE-gated=%d\n",
                cpu_decoded, ive_decoded);
    }

    /* Cleanup */
    quirc_destroy(qr_cpu);
    quirc_destroy(qr_ive);
    free(frame_data);
    free_img(&img_cur); free_img(&img_bin); free_img(&img_tmp);
    if (need_sad) { free_img(&img_blk); free_img(&img_zero); }
    if (img_sad16.au64PhyAddr[0])
        HI_MPI_SYS_MmzFree(img_sad16.au64PhyAddr[0],
            (HI_VOID *)(HI_UL)img_sad16.au64VirAddr[0]);
    HI_MPI_SYS_MmzFree(blob_mem.u64PhyAddr, (HI_VOID *)(HI_UL)blob_mem.u64VirAddr);
    HI_MPI_SYS_Exit();
    return 0;
}
