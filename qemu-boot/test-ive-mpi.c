/*
 * IVE Hardware Test via HiSilicon MPI API
 *
 * Calls real IVE hardware through libmpi.so + libive.so on EV300 board.
 * Output format matches test-ive-ops.c (QEMU test) for direct comparison.
 *
 * Build:
 *   CC=arm-openipc-linux-musleabi-gcc
 *   SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
 *   $CC -o test-ive-mpi test-ive-mpi.c -I$SDK/include -L$SDK/lib -lmpi -live
 *
 * Run (stop majestic first — it owns MPI):
 *   killall majestic; sleep 1; ./test-ive-mpi; majestic &
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

/*
 * uclibc→musl compatibility shims (from majestic src/libc/uclibc.c).
 * The SDK's libmpi.so/libsecurec.so were built against uclibc and
 * reference symbols that don't exist in musl libc.
 */

/* Secure string functions used by libmpi.so/libsecurec.so */
int memcpy_s(void *d, size_t dn, const void *s, size_t n) { memcpy(d, s, n < dn ? n : dn); return 0; }
int memset_s(void *d, size_t dn, int c, size_t n) { memset(d, c, n < dn ? n : dn); return 0; }
int memmove_s(void *d, size_t dn, const void *s, size_t n) { memmove(d, s, n < dn ? n : dn); return 0; }
int strncpy_s(char *d, size_t dn, const char *s, size_t n) { strncpy(d, s, n < dn ? n : dn); return 0; }
int snprintf_s(char *d, size_t dn, size_t n, const char *f, ...) {
    va_list a; va_start(a, f); int r = vsnprintf(d, dn, f, a); va_end(a); return r;
}

/* uclibc ctype/stdio/stdlib stubs needed by libsecurec.so */
const unsigned short int *__ctype_b;
size_t _stdlib_mb_cur_max(void) { return 0; }
int __fputc_unlocked(int c, FILE *stream) { return fputc(c, stream); }
int __fgetc_unlocked(FILE *stream) { return fgetc(stream); }

#include "hi_type.h"
#include "hi_common.h"
#include "hi_comm_ive.h"
#include "mpi_sys.h"
#include "mpi_ive.h"

#define W  64
#define H  64
#define SZ (W * H)

/* Stride must be 16-byte aligned */
#define STRIDE ((W + 15) & ~15)

static uint32_t cksum(const uint8_t *data, int len) {
    uint32_t s = 0;
    for (int i = 0; i < len; i++) s = s * 31 + data[i];
    return s;
}

/* Allocate an IVE image with MMZ memory */
static HI_S32 alloc_image(IVE_IMAGE_S *img, HI_U32 w, HI_U32 h) {
    memset(img, 0, sizeof(*img));
    img->enType = IVE_IMAGE_TYPE_U8C1;
    img->u32Width = w;
    img->u32Height = h;
    img->au32Stride[0] = STRIDE;
    HI_U32 size = STRIDE * h;
    HI_S32 ret = HI_MPI_SYS_MmzAlloc(&img->au64PhyAddr[0],
                                       (HI_VOID **)&img->au64VirAddr[0],
                                       NULL, HI_NULL, size);
    if (ret != HI_SUCCESS) {
        fprintf(stderr, "MmzAlloc failed: 0x%x\n", ret);
    }
    return ret;
}

static void free_image(IVE_IMAGE_S *img) {
    if (img->au64PhyAddr[0]) {
        HI_MPI_SYS_MmzFree(img->au64PhyAddr[0],
                            (HI_VOID *)(HI_UL)img->au64VirAddr[0]);
    }
}

static HI_S32 alloc_mem(IVE_MEM_INFO_S *mem, HI_U32 size) {
    memset(mem, 0, sizeof(*mem));
    mem->u32Size = size;
    return HI_MPI_SYS_MmzAlloc(&mem->u64PhyAddr,
                                (HI_VOID **)&mem->u64VirAddr,
                                NULL, HI_NULL, size);
}

static void free_mem(IVE_MEM_INFO_S *mem) {
    if (mem->u64PhyAddr) {
        HI_MPI_SYS_MmzFree(mem->u64PhyAddr, (HI_VOID *)(HI_UL)mem->u64VirAddr);
    }
}

/* Flush cache for MMZ buffer so CPU sees IVE's DMA output */
static void flush_cache(IVE_IMAGE_S *img) {
    HI_MPI_SYS_MmzFlushCache(img->au64PhyAddr[0],
                              (HI_VOID *)(HI_UL)img->au64VirAddr[0],
                              img->au32Stride[0] * img->u32Height);
}

/* Wait for IVE operation to complete */
static HI_S32 ive_wait(IVE_HANDLE handle) {
    HI_BOOL finish = HI_FALSE;
    HI_S32 ret;
    do {
        ret = HI_MPI_IVE_Query(handle, &finish, HI_TRUE);
        if (ret == HI_ERR_IVE_QUERY_TIMEOUT) {
            usleep(100);
            continue;
        }
        break;
    } while (1);
    return ret;
}

/* Fill image with data pattern (stride-aware) */
static void fill_image(IVE_IMAGE_S *img, const uint8_t *pattern, int len) {
    uint8_t *vir = (uint8_t *)(HI_UL)img->au64VirAddr[0];
    for (HI_U32 y = 0; y < img->u32Height; y++) {
        for (HI_U32 x = 0; x < img->u32Width; x++) {
            int idx = y * img->u32Width + x;
            vir[y * img->au32Stride[0] + x] = (idx < len) ? pattern[idx] : 0;
        }
    }
}

/* Read image data (stride-aware) */
static void read_image(IVE_IMAGE_S *img, uint8_t *out, int len) {
    uint8_t *vir = (uint8_t *)(HI_UL)img->au64VirAddr[0];
    for (HI_U32 y = 0; y < img->u32Height; y++) {
        for (HI_U32 x = 0; x < img->u32Width; x++) {
            int idx = y * img->u32Width + x;
            if (idx < len) out[idx] = vir[y * img->au32Stride[0] + x];
        }
    }
}

int main(void) {
    HI_S32 ret;
    IVE_HANDLE handle;
    int fails = 0;

    /* Init MPI system */
    HI_MPI_SYS_Exit(); /* clean up any previous state */
    ret = HI_MPI_SYS_Init();
    if (ret != HI_SUCCESS) {
        fprintf(stderr, "SYS_Init failed: 0x%x\n", ret);
        return 1;
    }
    fprintf(stderr, "SYS_Init OK\n");

    /* Allocate images — use cached memory + explicit flush for DMA coherence */
    IVE_IMAGE_S src1, src2, dst;
    if (alloc_image(&src1, W, H) || alloc_image(&src2, W, H) || alloc_image(&dst, W, H)) {
        fprintf(stderr, "Image alloc failed\n");
        goto cleanup;
    }

    /* Fill test patterns — same as test-ive-ops.c */
    uint8_t pat1[SZ], pat2[SZ], result[SZ];
    for (int i = 0; i < SZ; i++) {
        pat1[i] = (i * 7 + 13) & 0xFF;
        pat2[i] = (i * 11 + 31) & 0xFF;
    }
    fill_image(&src1, pat1, SZ);
    fill_image(&src2, pat2, SZ);

    /* Verify source data was written */
    uint8_t *v1 = (uint8_t *)(HI_UL)src1.au64VirAddr[0];
    fprintf(stderr, "src1 virt=%p phys=0x%llx stride=%d data[0..3]=%d,%d,%d,%d\n",
        v1, (unsigned long long)src1.au64PhyAddr[0], src1.au32Stride[0],
        v1[0], v1[1], v1[2], v1[3]);

    printf("\n========================================\n");
    printf("IVE MPI Hardware Test (real IVE silicon)\n");
    printf("========================================\n");

    /* === SUB === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_SUB_CTRL_S ctrl = { .enMode = IVE_SUB_MODE_ABS };
        ret = HI_MPI_IVE_Sub(&handle, &src1, &src2, &dst, &ctrl, HI_TRUE);
        fprintf(stderr, "Sub ret=0x%x handle=%d\n", ret, handle);
        if (ret == HI_SUCCESS) {
            HI_S32 qret = ive_wait(handle);
            fprintf(stderr, "Query ret=0x%x\n", qret);
            flush_cache(&dst);
        }
        read_image(&dst, result, SZ);
        fprintf(stderr, "dst[0..3]=%d,%d,%d,%d\n", result[0], result[1], result[2], result[3]);
        printf("  %-10s cksum=0x%08X  [%d,%d,%d,%d]  %s\n", "sub",
               cksum(result, SZ), result[0], result[1], result[2], result[3],
               (ret == HI_SUCCESS) ? "PASS" : "FAIL");
        fails += (ret != HI_SUCCESS);
    }

    /* === AND === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        ret = HI_MPI_IVE_And(&handle, &src1, &src2, &dst, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        printf("  %-10s cksum=0x%08X  [%d,%d,%d,%d]  %s\n", "and",
               cksum(result, SZ), result[0], result[1], result[2], result[3],
               (ret == HI_SUCCESS) ? "PASS" : "FAIL");
        fails += (ret != HI_SUCCESS);
    }

    /* === OR === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        ret = HI_MPI_IVE_Or(&handle, &src1, &src2, &dst, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        printf("  %-10s cksum=0x%08X  [%d,%d,%d,%d]  %s\n", "or",
               cksum(result, SZ), result[0], result[1], result[2], result[3],
               (ret == HI_SUCCESS) ? "PASS" : "FAIL");
        fails += (ret != HI_SUCCESS);
    }

    /* === XOR === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        ret = HI_MPI_IVE_Xor(&handle, &src1, &src2, &dst, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        printf("  %-10s cksum=0x%08X  [%d,%d,%d,%d]  %s\n", "xor",
               cksum(result, SZ), result[0], result[1], result[2], result[3],
               (ret == HI_SUCCESS) ? "PASS" : "FAIL");
        fails += (ret != HI_SUCCESS);
    }

    /* === THRESH === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_THRESH_CTRL_S ctrl = {
            .enMode = IVE_THRESH_MODE_BINARY,
            .u8LowThr = 128,
            .u8MinVal = 0,
            .u8MaxVal = 255
        };
        ret = HI_MPI_IVE_Thresh(&handle, &src1, &dst, &ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        printf("  %-10s cksum=0x%08X  [%d,%d,%d,%d]  %s\n", "thresh",
               cksum(result, SZ), result[0], result[1], result[2], result[3],
               (ret == HI_SUCCESS) ? "PASS" : "FAIL");
        fails += (ret != HI_SUCCESS);
    }

    /* === HIST === */
    {
        IVE_MEM_INFO_S hist_mem;
        if (alloc_mem(&hist_mem, 256 * sizeof(HI_U32)) == HI_SUCCESS) {
            ret = HI_MPI_IVE_Hist(&handle, &src1, (IVE_DST_MEM_INFO_S *)&hist_mem, HI_TRUE);
            if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
            HI_U32 *hist = (HI_U32 *)(HI_UL)hist_mem.u64VirAddr;
            HI_U32 total = 0;
            for (int i = 0; i < 256; i++) total += hist[i];
            printf("  %-10s total=%d (expect %d)  %s\n", "hist", total, SZ,
                   (total == SZ && ret == HI_SUCCESS) ? "PASS" : "FAIL");
            fails += (total != SZ || ret != HI_SUCCESS);
            free_mem(&hist_mem);
        }
    }

    /* === SOBEL === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_SOBEL_CTRL_S ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.enOutCtrl = IVE_SOBEL_OUT_CTRL_BOTH;
        /* 3x3 Sobel masks */
        HI_S8 mask_h[25] = {0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};
        HI_S8 mask_v[25] = {0,0,0,0,0, 0,-1,-2,-1,0, 0,0,0,0,0, 0,1,2,1,0, 0,0,0,0,0};
        memcpy(ctrl.as8Mask, mask_h, 25);

        /* Sobel outputs S16C1, need different image type */
        IVE_DST_IMAGE_S dst_h, dst_v;
        memset(&dst_h, 0, sizeof(dst_h));
        memset(&dst_v, 0, sizeof(dst_v));
        dst_h.enType = IVE_IMAGE_TYPE_S16C1;
        dst_h.u32Width = W; dst_h.u32Height = H;
        dst_h.au32Stride[0] = STRIDE * 2;
        HI_MPI_SYS_MmzAlloc(&dst_h.au64PhyAddr[0], (HI_VOID **)&dst_h.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * 2 * H);
        dst_v = dst_h;
        HI_MPI_SYS_MmzAlloc(&dst_v.au64PhyAddr[0], (HI_VOID **)&dst_v.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * 2 * H);

        ret = HI_MPI_IVE_Sobel(&handle, &src1, &dst_h, &dst_v, &ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }

        /* Read back horizontal gradient magnitude */
        int16_t *sobel_h = (int16_t *)(HI_UL)dst_h.au64VirAddr[0];
        uint8_t sobel_out[SZ];
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                int v = abs(sobel_h[y * (STRIDE) + x]);
                sobel_out[y * W + x] = (v > 255) ? 255 : v;
            }

        printf("  %-10s cksum=0x%08X  [%d,%d,%d,%d]  %s\n", "sobel",
               cksum(sobel_out, SZ), sobel_out[0], sobel_out[1], sobel_out[2], sobel_out[3],
               (ret == HI_SUCCESS) ? "PASS" : "FAIL");
        fails += (ret != HI_SUCCESS);

        HI_MPI_SYS_MmzFree(dst_h.au64PhyAddr[0], (HI_VOID *)(HI_UL)dst_h.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(dst_v.au64PhyAddr[0], (HI_VOID *)(HI_UL)dst_v.au64VirAddr[0]);
    }

    /* === DILATE === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_DILATE_CTRL_S ctrl;
        memset(ctrl.au8Mask, 255, 25); /* 5x5 all-ones structuring element */
        ret = HI_MPI_IVE_Dilate(&handle, &src1, &dst, &ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        printf("  %-10s cksum=0x%08X  [%d,%d,%d,%d]  %s\n", "dilate",
               cksum(result, SZ), result[0], result[1], result[2], result[3],
               (ret == HI_SUCCESS) ? "PASS" : "FAIL");
        fails += (ret != HI_SUCCESS);
    }

    /* === ERODE === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_ERODE_CTRL_S ctrl;
        memset(ctrl.au8Mask, 255, 25); /* 5x5 all-ones structuring element */
        ret = HI_MPI_IVE_Erode(&handle, &src1, &dst, &ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        printf("  %-10s cksum=0x%08X  [%d,%d,%d,%d]  %s\n", "erode",
               cksum(result, SZ), result[0], result[1], result[2], result[3],
               (ret == HI_SUCCESS) ? "PASS" : "FAIL");
        fails += (ret != HI_SUCCESS);
    }

    printf("========================================\n");
    printf("Result: %d/%d passed\n", 9 - fails, 9);
    printf("========================================\n");

cleanup:
    free_image(&src1);
    free_image(&src2);
    free_image(&dst);
    HI_MPI_SYS_Exit();
    return fails;
}
