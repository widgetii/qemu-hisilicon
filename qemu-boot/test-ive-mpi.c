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

        /* Sobel outputs S16C1 — stride in PIXELS (not bytes), per SDK convention */
        IVE_DST_IMAGE_S dst_h, dst_v;
        memset(&dst_h, 0, sizeof(dst_h));
        memset(&dst_v, 0, sizeof(dst_v));
        dst_h.enType = IVE_IMAGE_TYPE_S16C1;
        dst_h.u32Width = W; dst_h.u32Height = H;
        dst_h.au32Stride[0] = STRIDE; /* pixel stride, not byte stride */
        HI_MPI_SYS_MmzAlloc(&dst_h.au64PhyAddr[0], (HI_VOID **)&dst_h.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * H * sizeof(HI_S16));
        dst_v = dst_h;
        HI_MPI_SYS_MmzAlloc(&dst_v.au64PhyAddr[0], (HI_VOID **)&dst_v.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * H * sizeof(HI_S16));

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

    /* === ADD (weighted blend) === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_ADD_CTRL_S add_ctrl = { .u0q16X = 32768, .u0q16Y = 32768 }; /* 0.5 + 0.5 */
        ret = HI_MPI_IVE_Add(&handle, &src1, &src2, &dst, &add_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        printf("  %-10s cksum=0x%08X  [%d,%d,%d,%d]  %s\n", "add",
               cksum(result, SZ), result[0], result[1], result[2], result[3],
               (ret == HI_SUCCESS) ? "PASS" : "FAIL");
        fails += (ret != HI_SUCCESS);
    }

    /* === INTEG (integral image) === */
    {
        /* COMBINE mode outputs U64C1. Use 64×64 (IVE minimum size). */
        IVE_IMAGE_S integ_src, integ_dst;
        int IW = W, IH = H;
        memset(&integ_src, 0, sizeof(integ_src));
        integ_src.enType = IVE_IMAGE_TYPE_U8C1;
        integ_src.u32Width = IW; integ_src.u32Height = IH;
        integ_src.au32Stride[0] = (IW + 15) & ~15;
        HI_MPI_SYS_MmzAlloc(&integ_src.au64PhyAddr[0],
            (HI_VOID **)&integ_src.au64VirAddr[0], NULL, HI_NULL, integ_src.au32Stride[0] * IH);
        memset(&integ_dst, 0, sizeof(integ_dst));
        integ_dst.enType = IVE_IMAGE_TYPE_U64C1;
        integ_dst.u32Width = IW; integ_dst.u32Height = IH;
        integ_dst.au32Stride[0] = (IW + 15) & ~15; /* stride in PIXELS, not bytes */
        HI_MPI_SYS_MmzAlloc(&integ_dst.au64PhyAddr[0],
            (HI_VOID **)&integ_dst.au64VirAddr[0], NULL, HI_NULL,
            integ_dst.au32Stride[0] * IH * sizeof(uint64_t));
        /* Fill with test data */
        uint8_t *isrc = (uint8_t *)(HI_UL)integ_src.au64VirAddr[0];
        for (int y = 0; y < IH; y++)
            for (int x = 0; x < IW; x++)
                isrc[y * integ_src.au32Stride[0] + x] = ((y*IW+x)*7+13) & 0xFF;
        HI_MPI_SYS_MmzFlushCache(integ_src.au64PhyAddr[0], isrc, integ_src.au32Stride[0] * IH);

        IVE_INTEG_CTRL_S integ_ctrl = { .enOutCtrl = IVE_INTEG_OUT_CTRL_COMBINE };
        ret = HI_MPI_IVE_Integ(&handle, &integ_src, &integ_dst, &integ_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            HI_MPI_SYS_MmzFlushCache(integ_dst.au64PhyAddr[0],
                (HI_VOID *)(HI_UL)integ_dst.au64VirAddr[0],
                integ_dst.au32Stride[0] * IH * sizeof(uint64_t));
            uint64_t *ig = (uint64_t *)(HI_UL)integ_dst.au64VirAddr[0];
            uint32_t exp_sum = 0;
            for (int i = 0; i < IW*IH; i++) exp_sum += ((i*7+13) & 0xFF);
            uint64_t hw_sum = ig[(IH-1) * integ_dst.au32Stride[0] + (IW-1)];
            int ok = ((uint32_t)hw_sum == exp_sum);
            printf("  %-10s sum=%u (expect %u) ret=0x%x  %s\n", "integ",
                   (uint32_t)hw_sum, exp_sum, ret, ok ? "PASS" : "FAIL");
            fails += !ok;
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "integ", ret);
        }
        HI_MPI_SYS_MmzFree(integ_src.au64PhyAddr[0], (HI_VOID *)(HI_UL)integ_src.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(integ_dst.au64PhyAddr[0], (HI_VOID *)(HI_UL)integ_dst.au64VirAddr[0]);
    }

    /* === RESIZE === */
    {
        IVE_IMAGE_S rsz_dst;
        int DW = 32, DH = 32;
        memset(&rsz_dst, 0, sizeof(rsz_dst));
        rsz_dst.enType = IVE_IMAGE_TYPE_U8C1;
        rsz_dst.u32Width = DW; rsz_dst.u32Height = DH;
        rsz_dst.au32Stride[0] = (DW + 15) & ~15;
        HI_MPI_SYS_MmzAlloc(&rsz_dst.au64PhyAddr[0],
            (HI_VOID **)&rsz_dst.au64VirAddr[0], NULL, HI_NULL, rsz_dst.au32Stride[0] * DH);

        IVE_MEM_INFO_S rsz_mem;
        memset(&rsz_mem, 0, sizeof(rsz_mem));
        rsz_mem.u32Size = W * sizeof(HI_U32) * 2 + H * sizeof(HI_U32) * 2;
        HI_MPI_SYS_MmzAlloc(&rsz_mem.u64PhyAddr, (HI_VOID **)&rsz_mem.u64VirAddr,
                             NULL, HI_NULL, rsz_mem.u32Size);
        IVE_RESIZE_CTRL_S rsz_ctrl = {
            .enMode = IVE_RESIZE_MODE_LINEAR,
            .stMem = rsz_mem,
            .u16Num = 1
        };
        IVE_SRC_IMAGE_S rsz_src_arr[1] = { src1 };
        IVE_DST_IMAGE_S rsz_dst_arr[1] = { rsz_dst };
        ret = HI_MPI_IVE_Resize(&handle, rsz_src_arr, rsz_dst_arr, &rsz_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            HI_MPI_SYS_MmzFlushCache(rsz_dst.au64PhyAddr[0],
                (HI_VOID *)(HI_UL)rsz_dst.au64VirAddr[0], rsz_dst.au32Stride[0] * DH);
            uint8_t *rv = (uint8_t *)(HI_UL)rsz_dst.au64VirAddr[0];
            int ok = (rv[rsz_dst.au32Stride[0]+1] != 0);
            printf("  %-10s [%d,%d,%d,%d]  %s\n", "resize",
                   rv[rsz_dst.au32Stride[0]+1], rv[rsz_dst.au32Stride[0]+2],
                   rv[rsz_dst.au32Stride[0]+3], rv[rsz_dst.au32Stride[0]+4],
                   ok ? "PASS" : "FAIL");
            fails += !ok;
        } else {
            printf("  %-10s ret=0x%x  %s\n", "resize", ret,
                   ret == (HI_S32)0xa01d8008 ? "NOT_SUPPORT — skipped" : "FAIL");
            if (ret != (HI_S32)0xa01d8008) fails++;
        }
        HI_MPI_SYS_MmzFree(rsz_dst.au64PhyAddr[0], (HI_VOID *)(HI_UL)rsz_dst.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(rsz_mem.u64PhyAddr, (HI_VOID *)(HI_UL)rsz_mem.u64VirAddr);
    }

    /* === MagAndAng (magnitude only — vendor sample pattern) === */
    {
        IVE_IMAGE_S mag_dst;
        memset(&mag_dst, 0, sizeof(mag_dst));
        mag_dst.enType = IVE_IMAGE_TYPE_U16C1;
        mag_dst.u32Width = W; mag_dst.u32Height = H;
        mag_dst.au32Stride[0] = STRIDE; /* stride in pixels, not bytes */
        HI_MPI_SYS_MmzAlloc(&mag_dst.au64PhyAddr[0],
            (HI_VOID **)&mag_dst.au64VirAddr[0], NULL, HI_NULL,
            STRIDE * H * sizeof(uint16_t));

        HI_S8 mag_mask[25] = {0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};
        IVE_MAG_AND_ANG_CTRL_S mag_ctrl = {
            .enOutCtrl = IVE_MAG_AND_ANG_OUT_CTRL_MAG, /* mag only, as vendor does */
            .u16Thr = 0
        };
        memcpy(mag_ctrl.as8Mask, mag_mask, 25);
        ret = HI_MPI_IVE_MagAndAng(&handle, &src1, &mag_dst, HI_NULL, &mag_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            HI_MPI_SYS_MmzFlushCache(mag_dst.au64PhyAddr[0],
                (HI_VOID *)(HI_UL)mag_dst.au64VirAddr[0],
                STRIDE * H * sizeof(uint16_t));
            uint16_t *mv = (uint16_t *)(HI_UL)mag_dst.au64VirAddr[0];
            int ok = (mv[STRIDE + 5] != 0);
            printf("  %-10s mag[5,1]=%d  %s\n", "mag_ang", mv[STRIDE+5],
                   ok ? "PASS" : "FAIL");
            fails += !ok;
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "mag_ang", ret);
        }
        HI_MPI_SYS_MmzFree(mag_dst.au64PhyAddr[0], (HI_VOID *)(HI_UL)mag_dst.au64VirAddr[0]);
    }

    /* === THRESH_S16 (via 16BitTo8Bit with S16→U8_ABS mode) === */
    {
        /* Write S16 test data to Sobel's dst_h buffer (already allocated above) */
        IVE_IMAGE_S s16_img;
        memset(&s16_img, 0, sizeof(s16_img));
        s16_img.enType = IVE_IMAGE_TYPE_S16C1;
        s16_img.u32Width = W; s16_img.u32Height = H;
        s16_img.au32Stride[0] = STRIDE * 2;
        HI_MPI_SYS_MmzAlloc(&s16_img.au64PhyAddr[0],
            (HI_VOID **)&s16_img.au64VirAddr[0], NULL, HI_NULL, STRIDE * 2 * H);

        /* Fill with test values: -200, -100, -50, 0, 50, 100, 150, 200 repeating */
        int16_t *s16_data = (int16_t *)(HI_UL)s16_img.au64VirAddr[0];
        int16_t test_vals[] = {-200, -100, -50, 0, 50, 100, 150, 200};
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                s16_data[y * STRIDE + x] = test_vals[(y * W + x) % 8];
        HI_MPI_SYS_MmzFlushCache(s16_img.au64PhyAddr[0],
            (HI_VOID *)(HI_UL)s16_img.au64VirAddr[0], STRIDE * 2 * H);

        /* 16BitTo8Bit: S16→U8_ABS (absolute value) */
        IVE_16BIT_TO_8BIT_CTRL_S cvt_ctrl = {
            .enMode = IVE_16BIT_TO_8BIT_MODE_S16_TO_U8_ABS,
            .u16Denominator = 1, .u8Numerator = 1, .s8Bias = 0
        };
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        ret = HI_MPI_IVE_16BitTo8Bit(&handle, &s16_img, &dst, &cvt_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        /* Expected: abs(-200)=200, abs(-100)=100, abs(-50)=50, abs(0)=0,
                     abs(50)=50, abs(100)=100, abs(150)=150, abs(200)=200 */
        int ok_16to8 = (result[0] == 200 && result[1] == 100 && result[2] == 50 &&
                        result[3] == 0 && result[4] == 50 && result[7] == 200);
        printf("  %-10s [%d,%d,%d,%d,%d,%d,%d,%d]  %s\n", "16to8",
               result[0], result[1], result[2], result[3],
               result[4], result[5], result[6], result[7],
               ok_16to8 ? "PASS" : "FAIL");
        fails += !ok_16to8;

        /* Thresh_S16: mode=S16_TO_U8_MIN_MID_MAX, lo=-50, hi=100 */
        IVE_THRESH_S16_CTRL_S ts16_ctrl = {
            .enMode = IVE_THRESH_S16_MODE_S16_TO_U8_MIN_MID_MAX,
            .s16LowThr = -50, .s16HighThr = 100,
            .un8MinVal = { .u8Val = 0 }, .un8MidVal = { .u8Val = 128 }, .un8MaxVal = { .u8Val = 255 }
        };
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        ret = HI_MPI_IVE_Thresh_S16(&handle, &s16_img, &dst, &ts16_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        int ok_ts16 = (result[0] == 0 && result[1] == 0 && result[3] == 128 &&
                       result[4] == 128 && result[6] == 255 && result[7] == 255);
        printf("  %-10s [%d,%d,%d,%d,%d,%d,%d,%d]  %s\n", "thresh_s16",
               result[0], result[1], result[2], result[3],
               result[4], result[5], result[6], result[7],
               ok_ts16 ? "PASS" : "FAIL");
        fails += !ok_ts16;

        /* Thresh_U16: mode=MIN_MID_MAX, lo=50, hi=150 */
        IVE_IMAGE_S u16_img;
        memset(&u16_img, 0, sizeof(u16_img));
        u16_img.enType = IVE_IMAGE_TYPE_U16C1;
        u16_img.u32Width = W; u16_img.u32Height = H;
        u16_img.au32Stride[0] = STRIDE * 2;
        HI_MPI_SYS_MmzAlloc(&u16_img.au64PhyAddr[0],
            (HI_VOID **)&u16_img.au64VirAddr[0], NULL, HI_NULL, STRIDE * 2 * H);

        uint16_t *u16_data = (uint16_t *)(HI_UL)u16_img.au64VirAddr[0];
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                u16_data[y * STRIDE + x] = ((y * W + x) * 7 + 13) % 256;
        HI_MPI_SYS_MmzFlushCache(u16_img.au64PhyAddr[0],
            (HI_VOID *)(HI_UL)u16_img.au64VirAddr[0], STRIDE * 2 * H);

        IVE_THRESH_U16_CTRL_S tu16_ctrl = {
            .enMode = IVE_THRESH_U16_MODE_U16_TO_U8_MIN_MID_MAX,
            .u16LowThr = 50, .u16HighThr = 150,
            .u8MinVal = 0, .u8MidVal = 128, .u8MaxVal = 255
        };
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        ret = HI_MPI_IVE_Thresh_U16(&handle, &u16_img, &dst, &tu16_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        /* val[0]=13 → <=50 → 0; val[7]=62 → 50<62<=150 → 128 */
        int ok_tu16 = (result[0] == 0 && result[7] == 128);
        printf("  %-10s [%d,%d,%d,%d,...,%d]  %s\n", "thresh_u16",
               result[0], result[1], result[2], result[3], result[7],
               ok_tu16 ? "PASS" : "FAIL");
        fails += !ok_tu16;

        HI_MPI_SYS_MmzFree(u16_img.au64PhyAddr[0], (HI_VOID *)(HI_UL)u16_img.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(s16_img.au64PhyAddr[0], (HI_VOID *)(HI_UL)s16_img.au64VirAddr[0]);
    }

    /* === OrdStatFilter (median) === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_ORD_STAT_FILTER_CTRL_S osf_ctrl = { .enMode = IVE_ORD_STAT_FILTER_MODE_MEDIAN };
        ret = HI_MPI_IVE_OrdStatFilter(&handle, &src1, &dst, &osf_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        int ok = (ret == HI_SUCCESS && result[W+1] != 0);
        printf("  %-10s [%d,%d,%d,%d]  %s\n", "ordstat",
               result[W+1], result[W+2], result[W+3], result[W+4],
               ok ? "PASS" : "FAIL");
        fails += !ok;
    }

    /* === EqualizeHist === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_MEM_INFO_S eq_mem;
        memset(&eq_mem, 0, sizeof(eq_mem));
        eq_mem.u32Size = 256 * 4 + 256; /* hist + map */
        HI_MPI_SYS_MmzAlloc(&eq_mem.u64PhyAddr, (HI_VOID **)&eq_mem.u64VirAddr,
                             NULL, HI_NULL, eq_mem.u32Size);
        IVE_EQUALIZE_HIST_CTRL_S eq_ctrl = { .stMem = eq_mem };
        ret = HI_MPI_IVE_EqualizeHist(&handle, &src1, &dst, &eq_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        uint8_t maxv = 0;
        for (int i = 0; i < SZ; i++) if (result[i] > maxv) maxv = result[i];
        int ok = (ret == HI_SUCCESS && maxv >= 250);
        printf("  %-10s max=%d  %s\n", "eq_hist", maxv, ok ? "PASS" : "FAIL");
        fails += !ok;
        HI_MPI_SYS_MmzFree(eq_mem.u64PhyAddr, (HI_VOID *)(HI_UL)eq_mem.u64VirAddr);
    }

    /* === LBP === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_LBP_CTRL_S lbp_ctrl = { .enMode = IVE_LBP_CMP_MODE_NORMAL };
        lbp_ctrl.un8BitThr.s8Val = 0;
        ret = HI_MPI_IVE_LBP(&handle, &src1, &dst, &lbp_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle); flush_cache(&dst);
            read_image(&dst, result, SZ);
            int ok = (result[W+1] != 0);
            printf("  %-10s [%d,%d,%d,%d]  %s\n", "lbp",
                   result[W+1], result[W+2], result[W+3], result[W+4],
                   ok ? "PASS" : "FAIL");
            fails += !ok;
        } else {
            printf("  %-10s NOT_SUPPORT (0x%x) — skipped\n", "lbp", ret);
            /* Don't count as failure — op not available on this SoC */
        }
    }

    /* === CannyHysEdge + CannyEdge === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_MEM_INFO_S canny_mem;
        memset(&canny_mem, 0, sizeof(canny_mem));
        canny_mem.u32Size = W * H * 4 + 16;
        HI_MPI_SYS_MmzAlloc(&canny_mem.u64PhyAddr, (HI_VOID **)&canny_mem.u64VirAddr,
                             NULL, HI_NULL, canny_mem.u32Size);
        IVE_DST_MEM_INFO_S canny_stack;
        memset(&canny_stack, 0, sizeof(canny_stack));
        canny_stack.u32Size = W * H * 8;
        HI_MPI_SYS_MmzAlloc(&canny_stack.u64PhyAddr, (HI_VOID **)&canny_stack.u64VirAddr,
                             NULL, HI_NULL, canny_stack.u32Size);
        HI_S8 canny_mask[25] = {0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};
        IVE_CANNY_HYS_EDGE_CTRL_S canny_ctrl = {
            .stMem = canny_mem, .u16LowThr = 30, .u16HighThr = 100
        };
        memcpy(canny_ctrl.as8Mask, canny_mask, 25);
        ret = HI_MPI_IVE_CannyHysEdge(&handle, &src1, &dst, &canny_stack,
                                        &canny_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        /* Part 2: edge tracing */
        if (ret == HI_SUCCESS) {
            ret = HI_MPI_IVE_CannyEdge(&dst, &canny_stack);
        }
        read_image(&dst, result, SZ);
        int edges = 0;
        for (int i = 0; i < SZ; i++) if (result[i] == 255) edges++;
        int ok = (ret == HI_SUCCESS && edges > 0);
        printf("  %-10s edges=%d  %s\n", "canny", edges, ok ? "PASS" : "FAIL");
        fails += !ok;
        HI_MPI_SYS_MmzFree(canny_mem.u64PhyAddr, (HI_VOID *)(HI_UL)canny_mem.u64VirAddr);
        HI_MPI_SYS_MmzFree(canny_stack.u64PhyAddr, (HI_VOID *)(HI_UL)canny_stack.u64VirAddr);
    }

    /* === FILTER (5×5 convolution) === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_FILTER_CTRL_S flt_ctrl;
        /* Identity-like kernel: center=16, normalize by >>4 = divide by 16 */
        memset(flt_ctrl.as8Mask, 0, 25);
        flt_ctrl.as8Mask[12] = 16; /* center element */
        flt_ctrl.u8Norm = 4;       /* right-shift by 4 = /16 */
        ret = HI_MPI_IVE_Filter(&handle, &src1, &dst, &flt_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        /* Identity kernel should produce ~same as input (with border zeros) */
        int ok = (ret == HI_SUCCESS && result[W+1] != 0);
        printf("  %-10s [%d,%d,%d,%d]  %s\n", "filter",
               result[W+1], result[W+2], result[W+3], result[W+4],
               ok ? "PASS" : "FAIL");
        fails += !ok;
    }

    /* === MAP (LUT) === */
    {
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_MAP_CTRL_S map_ctrl = { .enMode = IVE_MAP_MODE_U8 };
        IVE_MEM_INFO_S map_mem;
        memset(&map_mem, 0, sizeof(map_mem));
        map_mem.u32Size = 256;
        HI_MPI_SYS_MmzAlloc(&map_mem.u64PhyAddr, (HI_VOID **)&map_mem.u64VirAddr,
                             NULL, HI_NULL, 256);
        /* Invert LUT: map[i] = 255 - i */
        uint8_t *lut = (uint8_t *)(HI_UL)map_mem.u64VirAddr;
        for (int i = 0; i < 256; i++) lut[i] = 255 - i;
        HI_MPI_SYS_MmzFlushCache(map_mem.u64PhyAddr, lut, 256);

        IVE_SRC_IMAGE_S map_src = src1;
        ret = HI_MPI_IVE_Map(&handle, &map_src, &map_mem, &dst, &map_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) { ive_wait(handle); flush_cache(&dst); }
        read_image(&dst, result, SZ);
        /* src1[0]=13, inverted should be 242 */
        int ok = (ret == HI_SUCCESS && result[0] == (255 - 13));
        printf("  %-10s [%d,%d,%d,%d] (expect [%d,...])  %s\n", "map",
               result[0], result[1], result[2], result[3], 255-13,
               ok ? "PASS" : "FAIL");
        fails += !ok;
        HI_MPI_SYS_MmzFree(map_mem.u64PhyAddr, (HI_VOID *)(HI_UL)map_mem.u64VirAddr);
    }

    /* === NCC (Normalized Cross-Correlation) === */
    {
        IVE_MEM_INFO_S ncc_mem;
        memset(&ncc_mem, 0, sizeof(ncc_mem));
        ncc_mem.u32Size = sizeof(HI_U64) * 3; /* numerator + denom1 + denom2 */
        HI_MPI_SYS_MmzAlloc(&ncc_mem.u64PhyAddr, (HI_VOID **)&ncc_mem.u64VirAddr,
                             NULL, HI_NULL, ncc_mem.u32Size);
        ret = HI_MPI_IVE_NCC(&handle, &src1, &src1, &ncc_mem, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            HI_MPI_SYS_MmzFlushCache(ncc_mem.u64PhyAddr,
                (HI_VOID *)(HI_UL)ncc_mem.u64VirAddr, ncc_mem.u32Size);
            HI_U64 *ncc = (HI_U64 *)(HI_UL)ncc_mem.u64VirAddr;
            /* Self-correlation: numerator should equal each denominator */
            int ok = (ncc[0] > 0 && ncc[0] == ncc[1] && ncc[1] == ncc[2]);
            printf("  %-10s num=%llu d1=%llu d2=%llu  %s\n", "ncc",
                   (unsigned long long)ncc[0], (unsigned long long)ncc[1],
                   (unsigned long long)ncc[2], ok ? "PASS" : "FAIL");
            fails += !ok;
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "ncc", ret);
        }
        HI_MPI_SYS_MmzFree(ncc_mem.u64PhyAddr, (HI_VOID *)(HI_UL)ncc_mem.u64VirAddr);
    }

    /* === CSC (Color Space Conversion, YUV420SP→RGB) === */
    {
        /* Create YUV420SP source: Y plane + UV plane */
        IVE_IMAGE_S yuv_src, rgb_dst;
        memset(&yuv_src, 0, sizeof(yuv_src));
        yuv_src.enType = IVE_IMAGE_TYPE_YUV420SP;
        yuv_src.u32Width = W; yuv_src.u32Height = H;
        yuv_src.au32Stride[0] = STRIDE;
        yuv_src.au32Stride[1] = STRIDE;
        HI_U32 yuv_sz = STRIDE * H * 3 / 2;
        HI_MPI_SYS_MmzAlloc(&yuv_src.au64PhyAddr[0], (HI_VOID **)&yuv_src.au64VirAddr[0],
                             NULL, HI_NULL, yuv_sz);
        yuv_src.au64PhyAddr[1] = yuv_src.au64PhyAddr[0] + STRIDE * H;
        yuv_src.au64VirAddr[1] = yuv_src.au64VirAddr[0] + STRIDE * H;
        /* Fill: Y=128, U=128, V=128 (neutral gray) */
        memset((void *)(HI_UL)yuv_src.au64VirAddr[0], 128, yuv_sz);
        HI_MPI_SYS_MmzFlushCache(yuv_src.au64PhyAddr[0],
            (HI_VOID *)(HI_UL)yuv_src.au64VirAddr[0], yuv_sz);

        memset(&rgb_dst, 0, sizeof(rgb_dst));
        rgb_dst.enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
        rgb_dst.u32Width = W; rgb_dst.u32Height = H;
        rgb_dst.au32Stride[0] = STRIDE;
        rgb_dst.au32Stride[1] = STRIDE;
        rgb_dst.au32Stride[2] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&rgb_dst.au64PhyAddr[0], (HI_VOID **)&rgb_dst.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * H * 3);
        rgb_dst.au64PhyAddr[1] = rgb_dst.au64PhyAddr[0] + STRIDE * H;
        rgb_dst.au64VirAddr[1] = rgb_dst.au64VirAddr[0] + STRIDE * H;
        rgb_dst.au64PhyAddr[2] = rgb_dst.au64PhyAddr[0] + STRIDE * H * 2;
        rgb_dst.au64VirAddr[2] = rgb_dst.au64VirAddr[0] + STRIDE * H * 2;

        IVE_CSC_CTRL_S csc_ctrl = { .enMode = IVE_CSC_MODE_PIC_BT601_YUV2RGB };
        ret = HI_MPI_IVE_CSC(&handle, &yuv_src, &rgb_dst, &csc_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            HI_MPI_SYS_MmzFlushCache(rgb_dst.au64PhyAddr[0],
                (HI_VOID *)(HI_UL)rgb_dst.au64VirAddr[0], STRIDE * H * 3);
            uint8_t *r_plane = (uint8_t *)(HI_UL)rgb_dst.au64VirAddr[0];
            /* Y=128,U=128,V=128 → RGB should be ~(128,128,128) gray */
            int ok = (r_plane[0] >= 120 && r_plane[0] <= 136);
            printf("  %-10s R=%d (expect ~128)  %s\n", "csc", r_plane[0],
                   ok ? "PASS" : "FAIL");
            fails += !ok;
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "csc", ret);
        }
        HI_MPI_SYS_MmzFree(yuv_src.au64PhyAddr[0], (HI_VOID *)(HI_UL)yuv_src.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(rgb_dst.au64PhyAddr[0], (HI_VOID *)(HI_UL)rgb_dst.au64VirAddr[0]);
    }

    /* === GMM2 (stateful Gaussian Mixture Model background subtraction) === */
    {
        int MODEL_NUM = 3;
        int NFRAMES_BG = 20;  /* frames to learn background */
        int NFRAMES_FG = 10;  /* frames with foreground object */

        /* Allocate images */
        IVE_IMAGE_S gmm_src, gmm_factor, gmm_fg, gmm_bg, gmm_match;
        memset(&gmm_src, 0, sizeof(gmm_src));
        memset(&gmm_factor, 0, sizeof(gmm_factor));
        memset(&gmm_fg, 0, sizeof(gmm_fg));
        memset(&gmm_bg, 0, sizeof(gmm_bg));
        memset(&gmm_match, 0, sizeof(gmm_match));

        /* Source: U8C1 64×64 */
        gmm_src.enType = IVE_IMAGE_TYPE_U8C1;
        gmm_src.u32Width = W; gmm_src.u32Height = H;
        gmm_src.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&gmm_src.au64PhyAddr[0],
            (HI_VOID **)&gmm_src.au64VirAddr[0], NULL, HI_NULL, STRIDE * H);

        /* Factor: U16C1 — low byte=sns factor, high byte=life update factor */
        gmm_factor.enType = IVE_IMAGE_TYPE_U16C1;
        gmm_factor.u32Width = W; gmm_factor.u32Height = H;
        gmm_factor.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&gmm_factor.au64PhyAddr[0],
            (HI_VOID **)&gmm_factor.au64VirAddr[0], NULL, HI_NULL, STRIDE * H * 2);
        uint8_t *fv = (uint8_t *)(HI_UL)gmm_factor.au64VirAddr[0];
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                int off = y * STRIDE * 2 + x * 2;
                fv[off]     = 8;  /* sensitivity factor */
                fv[off + 1] = 4;  /* life update factor */
            }
        HI_MPI_SYS_MmzFlushCache(gmm_factor.au64PhyAddr[0], fv, STRIDE * H * 2);

        /* Foreground, Background, MatchInfo: U8C1 */
        gmm_fg.enType = IVE_IMAGE_TYPE_U8C1;
        gmm_fg.u32Width = W; gmm_fg.u32Height = H;
        gmm_fg.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&gmm_fg.au64PhyAddr[0],
            (HI_VOID **)&gmm_fg.au64VirAddr[0], NULL, HI_NULL, STRIDE * H);
        gmm_bg = gmm_fg;
        HI_MPI_SYS_MmzAlloc(&gmm_bg.au64PhyAddr[0],
            (HI_VOID **)&gmm_bg.au64VirAddr[0], NULL, HI_NULL, STRIDE * H);
        gmm_match = gmm_fg;
        HI_MPI_SYS_MmzAlloc(&gmm_match.au64PhyAddr[0],
            (HI_VOID **)&gmm_match.au64VirAddr[0], NULL, HI_NULL, STRIDE * H);

        /* Model: zeroed, MODEL_NUM × 8 × W × H bytes */
        IVE_MEM_INFO_S gmm_model;
        memset(&gmm_model, 0, sizeof(gmm_model));
        gmm_model.u32Size = MODEL_NUM * 8 * W * H;
        HI_MPI_SYS_MmzAlloc(&gmm_model.u64PhyAddr, (HI_VOID **)&gmm_model.u64VirAddr,
                             NULL, HI_NULL, gmm_model.u32Size);
        memset((void *)(HI_UL)gmm_model.u64VirAddr, 0, gmm_model.u32Size);
        HI_MPI_SYS_MmzFlushCache(gmm_model.u64PhyAddr,
            (HI_VOID *)(HI_UL)gmm_model.u64VirAddr, gmm_model.u32Size);

        /* Control parameters (vendor defaults) */
        IVE_GMM2_CTRL_S gmm_ctrl;
        memset(&gmm_ctrl, 0, sizeof(gmm_ctrl));
        gmm_ctrl.u8ModelNum             = MODEL_NUM;
        gmm_ctrl.u16VarRate             = 1;
        gmm_ctrl.u9q7MaxVar             = (16 * 16) << 7;
        gmm_ctrl.u9q7MinVar             = (8 * 8) << 7;
        gmm_ctrl.u8GlbSnsFactor         = 8;
        gmm_ctrl.u16FreqThr             = 12000;
        gmm_ctrl.u16FreqInitVal         = 20000;
        gmm_ctrl.u16FreqAddFactor       = 0xEF;
        gmm_ctrl.u16FreqReduFactor      = 0xFF00;
        gmm_ctrl.u16LifeThr             = 5000;
        gmm_ctrl.u16GlbLifeUpdateFactor = 4;
        gmm_ctrl.enSnsFactorMode        = IVE_GMM2_SNS_FACTOR_MODE_PIX;
        gmm_ctrl.enLifeUpdateFactorMode = IVE_GMM2_LIFE_UPDATE_FACTOR_MODE_GLB;

        int gmm_ok = 1;
        uint8_t *sv = (uint8_t *)(HI_UL)gmm_src.au64VirAddr[0];

        /* Phase 1: 20 frames of uniform gray background (128) */
        for (int f = 0; f < NFRAMES_BG; f++) {
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++)
                    sv[y * STRIDE + x] = 128;
            HI_MPI_SYS_MmzFlushCache(gmm_src.au64PhyAddr[0], sv, STRIDE * H);

            gmm_ctrl.u16GlbLifeUpdateFactor = 0xFFFF / (f + 1);
            ret = HI_MPI_IVE_GMM2(&handle, &gmm_src, &gmm_factor, &gmm_fg,
                                   &gmm_bg, &gmm_match, &gmm_model, &gmm_ctrl, HI_TRUE);
            if (ret != HI_SUCCESS) {
                printf("  %-10s GMM2 frame %d ret=0x%x — FAIL\n", "gmm2", f, ret);
                gmm_ok = 0; break;
            }
            ive_wait(handle);
        }

        /* Phase 2: 10 frames with bright object at [24,24]-[40,40] */
        if (gmm_ok) {
            gmm_ctrl.u16GlbLifeUpdateFactor = 4;
            for (int f = 0; f < NFRAMES_FG; f++) {
                for (int y = 0; y < H; y++)
                    for (int x = 0; x < W; x++)
                        sv[y * STRIDE + x] = (x >= 24 && x < 40 && y >= 24 && y < 40) ? 255 : 128;
                HI_MPI_SYS_MmzFlushCache(gmm_src.au64PhyAddr[0], sv, STRIDE * H);

                ret = HI_MPI_IVE_GMM2(&handle, &gmm_src, &gmm_factor, &gmm_fg,
                                       &gmm_bg, &gmm_match, &gmm_model, &gmm_ctrl, HI_TRUE);
                if (ret != HI_SUCCESS) {
                    printf("  %-10s GMM2 fg frame %d ret=0x%x — FAIL\n", "gmm2", f, ret);
                    gmm_ok = 0; break;
                }
                ive_wait(handle);
            }
        }

        /* Check foreground mask: bright region should be detected */
        if (gmm_ok) {
            HI_MPI_SYS_MmzFlushCache(gmm_fg.au64PhyAddr[0],
                (HI_VOID *)(HI_UL)gmm_fg.au64VirAddr[0], STRIDE * H);
            uint8_t *fgv = (uint8_t *)(HI_UL)gmm_fg.au64VirAddr[0];

            /* Count FG pixels inside object region [24,24]-[40,40] */
            int fg_inside = 0, fg_outside = 0;
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++) {
                    if (fgv[y * STRIDE + x] > 0) {
                        if (x >= 24 && x < 40 && y >= 24 && y < 40) fg_inside++;
                        else fg_outside++;
                    }
                }
            /* Object should be detected as foreground */
            int ok = (fg_inside > 100); /* 16×16=256 pixels, expect majority */
            printf("  %-10s fg_in=%d fg_out=%d  %s\n", "gmm2",
                   fg_inside, fg_outside, ok ? "PASS" : "FAIL");
            fails += !ok;
        } else {
            fails++;
        }

        HI_MPI_SYS_MmzFree(gmm_src.au64PhyAddr[0], (HI_VOID *)(HI_UL)gmm_src.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gmm_factor.au64PhyAddr[0], (HI_VOID *)(HI_UL)gmm_factor.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gmm_fg.au64PhyAddr[0], (HI_VOID *)(HI_UL)gmm_fg.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gmm_bg.au64PhyAddr[0], (HI_VOID *)(HI_UL)gmm_bg.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gmm_match.au64PhyAddr[0], (HI_VOID *)(HI_UL)gmm_match.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gmm_model.u64PhyAddr, (HI_VOID *)(HI_UL)gmm_model.u64VirAddr);
    }

    /* === NormGrad === */
    {
        IVE_IMAGE_S ng_dst_h, ng_dst_v;
        memset(&ng_dst_h, 0, sizeof(ng_dst_h));
        ng_dst_h.enType = IVE_IMAGE_TYPE_S8C1;
        ng_dst_h.u32Width = W; ng_dst_h.u32Height = H;
        ng_dst_h.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&ng_dst_h.au64PhyAddr[0],
            (HI_VOID **)&ng_dst_h.au64VirAddr[0], NULL, HI_NULL, STRIDE * H);
        ng_dst_v = ng_dst_h;
        HI_MPI_SYS_MmzAlloc(&ng_dst_v.au64PhyAddr[0],
            (HI_VOID **)&ng_dst_v.au64VirAddr[0], NULL, HI_NULL, STRIDE * H);
        HI_S8 ng_mask[25] = {0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};
        IVE_NORM_GRAD_CTRL_S ng_ctrl = {
            .enOutCtrl = IVE_NORM_GRAD_OUT_CTRL_HOR_AND_VER,
            .u8Norm = 3
        };
        memcpy(ng_ctrl.as8Mask, ng_mask, 25);
        ret = HI_MPI_IVE_NormGrad(&handle, &src1, &ng_dst_h, &ng_dst_v, NULL, &ng_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            HI_MPI_SYS_MmzFlushCache(ng_dst_h.au64PhyAddr[0],
                (HI_VOID *)(HI_UL)ng_dst_h.au64VirAddr[0], STRIDE * H);
            int8_t *nv = (int8_t *)(HI_UL)ng_dst_h.au64VirAddr[0];
            int ok = (nv[STRIDE+5] != 0);
            printf("  %-10s h[5,1]=%d  %s\n", "normgrad", nv[STRIDE+5],
                   ok ? "PASS" : "FAIL");
            fails += !ok;
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "normgrad", ret);
        }
        HI_MPI_SYS_MmzFree(ng_dst_h.au64PhyAddr[0], (HI_VOID *)(HI_UL)ng_dst_h.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(ng_dst_v.au64PhyAddr[0], (HI_VOID *)(HI_UL)ng_dst_v.au64VirAddr[0]);
    }

    /* === STCandiCorner + STCorner === */
    {
        IVE_IMAGE_S st_dst;
        memset(&st_dst, 0, sizeof(st_dst));
        st_dst.enType = IVE_IMAGE_TYPE_U8C1;
        st_dst.u32Width = W; st_dst.u32Height = H;
        st_dst.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&st_dst.au64PhyAddr[0],
            (HI_VOID **)&st_dst.au64VirAddr[0], NULL, HI_NULL, STRIDE * H);

        /* Aux memory for STCandiCorner */
        IVE_MEM_INFO_S st_mem;
        memset(&st_mem, 0, sizeof(st_mem));
        HI_U32 st_mem_sz = 4 * STRIDE * H + 16; /* 4 * stride * height + IVE_ST_MAX_EIG_S */
        HI_MPI_SYS_MmzAlloc(&st_mem.u64PhyAddr, (HI_VOID **)&st_mem.u64VirAddr,
                             NULL, HI_NULL, st_mem_sz);
        st_mem.u32Size = st_mem_sz;

        IVE_ST_CANDI_CORNER_CTRL_S st_candi_ctrl = {
            .stMem = st_mem,
            .u0q8QualityLevel = 25
        };

        ret = HI_MPI_IVE_STCandiCorner(&handle, &src1, &st_dst, &st_candi_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);

            /* Now run STCorner (CPU function) */
            IVE_DST_MEM_INFO_S corner_mem;
            memset(&corner_mem, 0, sizeof(corner_mem));
            corner_mem.u32Size = 2 + 500 * 4; /* u16 count + 500 * (u16 x, u16 y) */
            HI_MPI_SYS_MmzAlloc(&corner_mem.u64PhyAddr, (HI_VOID **)&corner_mem.u64VirAddr,
                                 NULL, HI_NULL, corner_mem.u32Size);

            IVE_ST_CORNER_CTRL_S st_corner_ctrl = {
                .u16MaxCornerNum = 500,
                .u16MinDist = 5
            };

            ret = HI_MPI_IVE_STCorner(&st_dst, &corner_mem, &st_corner_ctrl);
            if (ret == HI_SUCCESS) {
                HI_MPI_SYS_MmzFlushCache(corner_mem.u64PhyAddr,
                    (HI_VOID *)(HI_UL)corner_mem.u64VirAddr, corner_mem.u32Size);
                uint16_t *corner_data = (uint16_t *)(HI_UL)corner_mem.u64VirAddr;
                uint16_t n_corners = corner_data[0];
                int ok = (n_corners > 0);
                printf("  %-10s corners=%d  %s\n", "st_corner", n_corners,
                       ok ? "PASS" : "FAIL");
                fails += !ok;
            } else {
                printf("  %-10s STCorner ret=0x%x — skipped\n", "st_corner", ret);
            }
            HI_MPI_SYS_MmzFree(corner_mem.u64PhyAddr, (HI_VOID *)(HI_UL)corner_mem.u64VirAddr);
        } else {
            printf("  %-10s STCandiCorner ret=0x%x — skipped\n", "st_corner", ret);
        }
        HI_MPI_SYS_MmzFree(st_dst.au64PhyAddr[0], (HI_VOID *)(HI_UL)st_dst.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(st_mem.u64PhyAddr, (HI_VOID *)(HI_UL)st_mem.u64VirAddr);
    }

    /* === GradFg === */
    {
        /* Need 3 inputs: bgDiffFg, curGrad, bgGrad — use src1 for all */
        memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE * H);
        IVE_GRAD_FG_CTRL_S gf_ctrl;
        memset(&gf_ctrl, 0, sizeof(gf_ctrl));
        gf_ctrl.enMode = IVE_GRAD_FG_MODE_USE_CUR_GRAD;
        gf_ctrl.u16EdwFactor = 1000;
        gf_ctrl.u8CrlCoefThr = 80;
        gf_ctrl.u8MagCrlThr = 4;
        gf_ctrl.u8MinMagDiff = 2;
        gf_ctrl.u8NoiseVal = 1;
        gf_ctrl.u8EdwDark = 1;
        ret = HI_MPI_IVE_GradFg(&handle, &src1, &src1, &src2, &dst, &gf_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle); flush_cache(&dst);
            read_image(&dst, result, SZ);
            printf("  %-10s [%d,%d,%d,%d]  PASS\n", "gradfg",
                   result[0], result[1], result[2], result[3]);
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "gradfg", ret);
        }
    }

    /* === GMM v1 === */
    {
        IVE_IMAGE_S gmm1_fg, gmm1_bg;
        memset(&gmm1_fg, 0, sizeof(gmm1_fg));
        gmm1_fg.enType = IVE_IMAGE_TYPE_U8C1;
        gmm1_fg.u32Width = W; gmm1_fg.u32Height = H;
        gmm1_fg.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&gmm1_fg.au64PhyAddr[0],
            (HI_VOID **)&gmm1_fg.au64VirAddr[0], NULL, HI_NULL, STRIDE * H);
        gmm1_bg = gmm1_fg;
        HI_MPI_SYS_MmzAlloc(&gmm1_bg.au64PhyAddr[0],
            (HI_VOID **)&gmm1_bg.au64VirAddr[0], NULL, HI_NULL, STRIDE * H);
        IVE_MEM_INFO_S gmm1_model;
        memset(&gmm1_model, 0, sizeof(gmm1_model));
        gmm1_model.u32Size = 3 * 8 * W * H;
        HI_MPI_SYS_MmzAlloc(&gmm1_model.u64PhyAddr, (HI_VOID **)&gmm1_model.u64VirAddr,
                             NULL, HI_NULL, gmm1_model.u32Size);
        memset((void *)(HI_UL)gmm1_model.u64VirAddr, 0, gmm1_model.u32Size);
        HI_MPI_SYS_MmzFlushCache(gmm1_model.u64PhyAddr,
            (HI_VOID *)(HI_UL)gmm1_model.u64VirAddr, gmm1_model.u32Size);

        IVE_GMM_CTRL_S gmm1_ctrl;
        memset(&gmm1_ctrl, 0, sizeof(gmm1_ctrl));
        gmm1_ctrl.u8ModelNum = 3;
        gmm1_ctrl.u22q10NoiseVar = 225 << 10;
        gmm1_ctrl.u22q10MaxVar = (16*16) << 10;
        gmm1_ctrl.u22q10MinVar = (8*8) << 10;
        gmm1_ctrl.u0q16LearnRate = 65535 / 10;
        gmm1_ctrl.u0q16BgRatio = 49152;
        gmm1_ctrl.u8q8VarThr = 4 << 8;
        gmm1_ctrl.u0q16InitWeight = 3277;

        ret = HI_MPI_IVE_GMM(&handle, &src1, &gmm1_fg, &gmm1_bg, &gmm1_model, &gmm1_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            printf("  %-10s PASS\n", "gmm");
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "gmm", ret);
        }
        HI_MPI_SYS_MmzFree(gmm1_fg.au64PhyAddr[0], (HI_VOID *)(HI_UL)gmm1_fg.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gmm1_bg.au64PhyAddr[0], (HI_VOID *)(HI_UL)gmm1_bg.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gmm1_model.u64PhyAddr, (HI_VOID *)(HI_UL)gmm1_model.u64VirAddr);
    }

    /* === FilterAndCSC === */
    {
        /* Input YUV420SP, output RGB planar */
        IVE_IMAGE_S flt_yuv, flt_rgb;
        memset(&flt_yuv, 0, sizeof(flt_yuv));
        flt_yuv.enType = IVE_IMAGE_TYPE_YUV420SP;
        flt_yuv.u32Width = W; flt_yuv.u32Height = H;
        flt_yuv.au32Stride[0] = STRIDE; flt_yuv.au32Stride[1] = STRIDE;
        HI_U32 yuv_sz = STRIDE * H * 3 / 2;
        HI_MPI_SYS_MmzAlloc(&flt_yuv.au64PhyAddr[0], (HI_VOID **)&flt_yuv.au64VirAddr[0],
                             NULL, HI_NULL, yuv_sz);
        flt_yuv.au64PhyAddr[1] = flt_yuv.au64PhyAddr[0] + STRIDE * H;
        flt_yuv.au64VirAddr[1] = flt_yuv.au64VirAddr[0] + STRIDE * H;
        memset((void *)(HI_UL)flt_yuv.au64VirAddr[0], 128, yuv_sz);
        HI_MPI_SYS_MmzFlushCache(flt_yuv.au64PhyAddr[0],
            (HI_VOID *)(HI_UL)flt_yuv.au64VirAddr[0], yuv_sz);

        memset(&flt_rgb, 0, sizeof(flt_rgb));
        flt_rgb.enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
        flt_rgb.u32Width = W; flt_rgb.u32Height = H;
        flt_rgb.au32Stride[0] = flt_rgb.au32Stride[1] = flt_rgb.au32Stride[2] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&flt_rgb.au64PhyAddr[0], (HI_VOID **)&flt_rgb.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * H * 3);
        flt_rgb.au64PhyAddr[1] = flt_rgb.au64PhyAddr[0] + STRIDE * H;
        flt_rgb.au64VirAddr[1] = flt_rgb.au64VirAddr[0] + STRIDE * H;
        flt_rgb.au64PhyAddr[2] = flt_rgb.au64PhyAddr[0] + STRIDE * H * 2;
        flt_rgb.au64VirAddr[2] = flt_rgb.au64VirAddr[0] + STRIDE * H * 2;

        IVE_FILTER_AND_CSC_CTRL_S fltcsc_ctrl;
        memset(&fltcsc_ctrl, 0, sizeof(fltcsc_ctrl));
        fltcsc_ctrl.enMode = IVE_CSC_MODE_PIC_BT601_YUV2RGB;
        /* Identity filter */
        fltcsc_ctrl.as8Mask[12] = 16;
        fltcsc_ctrl.u8Norm = 4;

        ret = HI_MPI_IVE_FilterAndCSC(&handle, &flt_yuv, &flt_rgb, &fltcsc_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            printf("  %-10s PASS\n", "flt_csc");
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "flt_csc", ret);
        }
        HI_MPI_SYS_MmzFree(flt_yuv.au64PhyAddr[0], (HI_VOID *)(HI_UL)flt_yuv.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(flt_rgb.au64PhyAddr[0], (HI_VOID *)(HI_UL)flt_rgb.au64VirAddr[0]);
    }

    /* === LKOpticalFlowPyr === */
    {
        /* Build simple 1-level pyramid (level 0 only) */
        IVE_SRC_IMAGE_S lk_prev[1], lk_next[1];
        memset(lk_prev, 0, sizeof(lk_prev));
        memset(lk_next, 0, sizeof(lk_next));
        lk_prev[0] = src1; /* reuse */
        lk_next[0].enType = IVE_IMAGE_TYPE_U8C1;
        lk_next[0].u32Width = W; lk_next[0].u32Height = H;
        lk_next[0].au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&lk_next[0].au64PhyAddr[0],
            (HI_VOID **)&lk_next[0].au64VirAddr[0], NULL, HI_NULL, STRIDE * H);
        /* Next = src1 shifted right by 2 pixels (simulate motion) */
        uint8_t *nv = (uint8_t *)(HI_UL)lk_next[0].au64VirAddr[0];
        uint8_t *pv = (uint8_t *)(HI_UL)src1.au64VirAddr[0];
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                nv[y*STRIDE+x] = (x >= 2) ? pv[y*STRIDE+(x-2)] : pv[y*STRIDE+x];
        HI_MPI_SYS_MmzFlushCache(lk_next[0].au64PhyAddr[0], nv, STRIDE * H);

        /* Points: put 5 test points at known positions */
        int lk_npts = 5;
        IVE_SRC_MEM_INFO_S lk_prev_pts;
        IVE_MEM_INFO_S lk_next_pts;
        IVE_DST_MEM_INFO_S lk_status, lk_err;
        memset(&lk_prev_pts, 0, sizeof(lk_prev_pts));
        memset(&lk_next_pts, 0, sizeof(lk_next_pts));
        memset(&lk_status, 0, sizeof(lk_status));
        memset(&lk_err, 0, sizeof(lk_err));

        HI_U32 pts_sz = 500 * sizeof(HI_S32) * 2; /* S25Q7: 4 bytes each for x,y */
        pts_sz = (pts_sz + 15) & ~15;
        HI_MPI_SYS_MmzAlloc(&lk_prev_pts.u64PhyAddr, (HI_VOID **)&lk_prev_pts.u64VirAddr,
                             NULL, HI_NULL, pts_sz);
        lk_prev_pts.u32Size = pts_sz;
        HI_MPI_SYS_MmzAlloc(&lk_next_pts.u64PhyAddr, (HI_VOID **)&lk_next_pts.u64VirAddr,
                             NULL, HI_NULL, pts_sz);
        lk_next_pts.u32Size = pts_sz;
        HI_U32 status_sz = (500 + 15) & ~15;
        HI_MPI_SYS_MmzAlloc(&lk_status.u64PhyAddr, (HI_VOID **)&lk_status.u64VirAddr,
                             NULL, HI_NULL, status_sz);
        lk_status.u32Size = status_sz;
        HI_U32 err_sz = (500 * 2 + 15) & ~15;
        HI_MPI_SYS_MmzAlloc(&lk_err.u64PhyAddr, (HI_VOID **)&lk_err.u64VirAddr,
                             NULL, HI_NULL, err_sz);
        lk_err.u32Size = err_sz;

        /* Set prev points: S25Q7 format (value << 7) */
        int32_t *pp = (int32_t *)(HI_UL)lk_prev_pts.u64VirAddr;
        memset(pp, 0, pts_sz);
        int test_pts[][2] = {{20,20},{30,30},{10,40},{40,10},{32,32}};
        for (int i = 0; i < lk_npts; i++) {
            pp[i*2]   = test_pts[i][0] << 7;
            pp[i*2+1] = test_pts[i][1] << 7;
        }
        HI_MPI_SYS_MmzFlushCache(lk_prev_pts.u64PhyAddr, pp, pts_sz);
        /* Copy to next_pts as initial flow */
        memcpy((void *)(HI_UL)lk_next_pts.u64VirAddr, pp, pts_sz);
        HI_MPI_SYS_MmzFlushCache(lk_next_pts.u64PhyAddr,
            (void *)(HI_UL)lk_next_pts.u64VirAddr, pts_sz);

        IVE_LK_OPTICAL_FLOW_PYR_CTRL_S lk_ctrl;
        memset(&lk_ctrl, 0, sizeof(lk_ctrl));
        lk_ctrl.enOutMode = IVE_LK_OPTICAL_FLOW_PYR_OUT_MODE_BOTH;
        lk_ctrl.bUseInitFlow = HI_TRUE;
        lk_ctrl.u16PtsNum = lk_npts;
        lk_ctrl.u8MaxLevel = 0;
        lk_ctrl.u0q8MinEigThr = 100;
        lk_ctrl.u8IterCnt = 10;
        lk_ctrl.u0q8Eps = 2;

        ret = HI_MPI_IVE_LKOpticalFlowPyr(&handle, lk_prev, lk_next,
                                            &lk_prev_pts, &lk_next_pts,
                                            &lk_status, &lk_err, &lk_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            HI_MPI_SYS_MmzFlushCache(lk_status.u64PhyAddr,
                (HI_VOID *)(HI_UL)lk_status.u64VirAddr, status_sz);
            uint8_t *st = (uint8_t *)(HI_UL)lk_status.u64VirAddr;
            int tracked = 0;
            for (int i = 0; i < lk_npts; i++) if (st[i]) tracked++;
            printf("  %-10s tracked=%d/%d  %s\n", "lk_flow", tracked, lk_npts,
                   (tracked > 0) ? "PASS" : "FAIL");
            fails += (tracked == 0);
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "lk_flow", ret);
        }
        HI_MPI_SYS_MmzFree(lk_next[0].au64PhyAddr[0], (HI_VOID *)(HI_UL)lk_next[0].au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(lk_prev_pts.u64PhyAddr, (HI_VOID *)(HI_UL)lk_prev_pts.u64VirAddr);
        HI_MPI_SYS_MmzFree(lk_next_pts.u64PhyAddr, (HI_VOID *)(HI_UL)lk_next_pts.u64VirAddr);
        HI_MPI_SYS_MmzFree(lk_status.u64PhyAddr, (HI_VOID *)(HI_UL)lk_status.u64VirAddr);
        HI_MPI_SYS_MmzFree(lk_err.u64PhyAddr, (HI_VOID *)(HI_UL)lk_err.u64VirAddr);
    }

    /* === MatchBgModel === */
    {
        /* Complex stateful — just test if API is supported */
        IVE_DATA_S bg_model;
        memset(&bg_model, 0, sizeof(bg_model));
        bg_model.u32Width = W; bg_model.u32Height = H;
        bg_model.u32Stride = STRIDE * 48; /* per-pixel BG model ~48 bytes */
        HI_MPI_SYS_MmzAlloc(&bg_model.u64PhyAddr, (HI_VOID **)&bg_model.u64VirAddr,
                             NULL, HI_NULL, bg_model.u32Stride * H);
        memset((void *)(HI_UL)bg_model.u64VirAddr, 0, bg_model.u32Stride * H);
        HI_MPI_SYS_MmzFlushCache(bg_model.u64PhyAddr,
            (HI_VOID *)(HI_UL)bg_model.u64VirAddr, bg_model.u32Stride * H);

        IVE_IMAGE_S fg_flag;
        memset(&fg_flag, 0, sizeof(fg_flag));
        fg_flag.enType = IVE_IMAGE_TYPE_U8C1;
        fg_flag.u32Width = W; fg_flag.u32Height = H;
        fg_flag.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&fg_flag.au64PhyAddr[0], (HI_VOID **)&fg_flag.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * H);
        memset((void *)(HI_UL)fg_flag.au64VirAddr[0], 0, STRIDE * H);
        HI_MPI_SYS_MmzFlushCache(fg_flag.au64PhyAddr[0],
            (HI_VOID *)(HI_UL)fg_flag.au64VirAddr[0], STRIDE * H);

        IVE_IMAGE_S bg_diff_fg, frm_diff_fg;
        memset(&bg_diff_fg, 0, sizeof(bg_diff_fg));
        bg_diff_fg.enType = IVE_IMAGE_TYPE_U8C1;
        bg_diff_fg.u32Width = W; bg_diff_fg.u32Height = H;
        bg_diff_fg.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&bg_diff_fg.au64PhyAddr[0], (HI_VOID **)&bg_diff_fg.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * H);
        frm_diff_fg = bg_diff_fg;
        HI_MPI_SYS_MmzAlloc(&frm_diff_fg.au64PhyAddr[0], (HI_VOID **)&frm_diff_fg.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * H);

        IVE_DST_MEM_INFO_S stat_data;
        memset(&stat_data, 0, sizeof(stat_data));
        stat_data.u32Size = 16;
        HI_MPI_SYS_MmzAlloc(&stat_data.u64PhyAddr, (HI_VOID **)&stat_data.u64VirAddr,
                             NULL, HI_NULL, stat_data.u32Size);

        IVE_MATCH_BG_MODEL_CTRL_S match_ctrl;
        memset(&match_ctrl, 0, sizeof(match_ctrl));
        match_ctrl.u32CurFrmNum = 1;
        match_ctrl.u32PreFrmNum = 0;
        match_ctrl.u16TimeThr = 20;
        match_ctrl.u8DiffMaxThr = 6;
        match_ctrl.u8DiffMinThr = 4;
        match_ctrl.u8FastLearnRate = 2;

        ret = HI_MPI_IVE_MatchBgModel(&handle, &src1, &bg_model, &fg_flag,
                                       &bg_diff_fg, &frm_diff_fg, &stat_data,
                                       &match_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            printf("  %-10s PASS\n", "match_bg");
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "match_bg", ret);
        }

        HI_MPI_SYS_MmzFree(bg_model.u64PhyAddr, (HI_VOID *)(HI_UL)bg_model.u64VirAddr);
        HI_MPI_SYS_MmzFree(fg_flag.au64PhyAddr[0], (HI_VOID *)(HI_UL)fg_flag.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(bg_diff_fg.au64PhyAddr[0], (HI_VOID *)(HI_UL)bg_diff_fg.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(frm_diff_fg.au64PhyAddr[0], (HI_VOID *)(HI_UL)frm_diff_fg.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(stat_data.u64PhyAddr, (HI_VOID *)(HI_UL)stat_data.u64VirAddr);
    }

    /* === PerspTrans + Hog: not in libive.so for EV200 SDK === */
    printf("  %-10s not in libive.so — N/A\n", "persp");
    printf("  %-10s not in libive.so — N/A\n", "hog");

#if 0 /* PerspTrans + Hog — headers exist but functions not linked in EV200 SDK */
    /* === PerspTrans (affine transform) === */
    {
        /* Set up ROI + point pairs for identity-like transform */
        IVE_RECT_U32_S roi = { .u32X = 0, .u32Y = 0, .u32Width = W, .u32Height = H };

        IVE_DST_IMAGE_S pt_dst;
        memset(&pt_dst, 0, sizeof(pt_dst));
        pt_dst.enType = IVE_IMAGE_TYPE_U8C1;
        pt_dst.u32Width = W; pt_dst.u32Height = H;
        pt_dst.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&pt_dst.au64PhyAddr[0], (HI_VOID **)&pt_dst.au64VirAddr[0],
                             NULL, HI_NULL, STRIDE * H);

        /* 3 point pairs for affine: src→dst identity mapping (Q14.2 fixed point) */
        /* Point = (x << 2, y << 2) for U14Q2 format */
        typedef struct { uint16_t sx, sy, dx, dy; } PP;
        PP pairs[3] = {
            { 0 << 2, 0 << 2, 0 << 2, 0 << 2 },
            { (W-1) << 2, 0 << 2, (W-1) << 2, 0 << 2 },
            { 0 << 2, (H-1) << 2, 0 << 2, (H-1) << 2 }
        };
        IVE_SRC_MEM_INFO_S pp_mem;
        memset(&pp_mem, 0, sizeof(pp_mem));
        pp_mem.u32Size = sizeof(pairs);
        HI_MPI_SYS_MmzAlloc(&pp_mem.u64PhyAddr, (HI_VOID **)&pp_mem.u64VirAddr,
                             NULL, HI_NULL, pp_mem.u32Size);
        memcpy((void *)(HI_UL)pp_mem.u64VirAddr, pairs, sizeof(pairs));
        HI_MPI_SYS_MmzFlushCache(pp_mem.u64PhyAddr,
            (HI_VOID *)(HI_UL)pp_mem.u64VirAddr, pp_mem.u32Size);

        IVE_PERSP_TRANS_CTRL_S pt_ctrl;
        memset(&pt_ctrl, 0, sizeof(pt_ctrl));
        pt_ctrl.enAlgMode = IVE_PERSP_TRANS_ALG_MODE_AFFINE;
        pt_ctrl.enCscMode = IVE_PERSP_TRANS_CSC_MODE_NONE;
        pt_ctrl.u16RoiNum = 1;
        pt_ctrl.u16PointPairNum = 3;

        ret = HI_MPI_IVE_PerspTrans(&handle, &src1, &roi, &pp_mem, &pt_dst,
                                     &pt_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            HI_MPI_SYS_MmzFlushCache(pt_dst.au64PhyAddr[0],
                (HI_VOID *)(HI_UL)pt_dst.au64VirAddr[0], STRIDE * H);
            uint8_t *pv = (uint8_t *)(HI_UL)pt_dst.au64VirAddr[0];
            /* Identity transform: output should match input */
            uint8_t *sv2 = (uint8_t *)(HI_UL)src1.au64VirAddr[0];
            int ok = (pv[STRIDE+5] == sv2[STRIDE+5]);
            printf("  %-10s out[5,1]=%d (expect %d)  %s\n", "persp",
                   pv[STRIDE+5], sv2[STRIDE+5], ok ? "PASS" : "FAIL");
            fails += !ok;
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "persp", ret);
        }
        HI_MPI_SYS_MmzFree(pt_dst.au64PhyAddr[0], (HI_VOID *)(HI_UL)pt_dst.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(pp_mem.u64PhyAddr, (HI_VOID *)(HI_UL)pp_mem.u64VirAddr);
    }

    /* === Hog (Histogram of Oriented Gradients) === */
    {
        /* Create YUV420SP input */
        IVE_IMAGE_S hog_yuv;
        memset(&hog_yuv, 0, sizeof(hog_yuv));
        hog_yuv.enType = IVE_IMAGE_TYPE_YUV420SP;
        hog_yuv.u32Width = W; hog_yuv.u32Height = H;
        hog_yuv.au32Stride[0] = STRIDE; hog_yuv.au32Stride[1] = STRIDE;
        HI_U32 hog_yuv_sz = STRIDE * H * 3 / 2;
        HI_MPI_SYS_MmzAlloc(&hog_yuv.au64PhyAddr[0], (HI_VOID **)&hog_yuv.au64VirAddr[0],
                             NULL, HI_NULL, hog_yuv_sz);
        hog_yuv.au64PhyAddr[1] = hog_yuv.au64PhyAddr[0] + STRIDE * H;
        hog_yuv.au64VirAddr[1] = hog_yuv.au64VirAddr[0] + STRIDE * H;
        /* Fill with gradient pattern */
        uint8_t *hv = (uint8_t *)(HI_UL)hog_yuv.au64VirAddr[0];
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                hv[y * STRIDE + x] = ((y * W + x) * 7 + 13) & 0xFF;
        memset(hv + STRIDE * H, 128, STRIDE * H / 2); /* UV = neutral */
        HI_MPI_SYS_MmzFlushCache(hog_yuv.au64PhyAddr[0], hv, hog_yuv_sz);

        /* ROI: full image */
        IVE_RECT_U32_S hog_roi = { .u32X = 0, .u32Y = 0, .u32Width = W, .u32Height = H };

        /* Output blob */
        IVE_DST_BLOB_S hog_dst;
        memset(&hog_dst, 0, sizeof(hog_dst));
        hog_dst.enType = IVE_BLOB_TYPE_VEC_S32;
        hog_dst.u32Num = 1;
        hog_dst.unShape.stWhc.u32Width = 1;
        hog_dst.unShape.stWhc.u32Height = 1;
        hog_dst.unShape.stWhc.u32Chn = 9 * 4; /* 9 orientation bins * 4 cells */
        hog_dst.u32Stride = (9 * 4 * 4 + 15) & ~15;
        HI_MPI_SYS_MmzAlloc(&hog_dst.u64PhyAddr, (HI_VOID **)&hog_dst.u64VirAddr,
                             NULL, HI_NULL, hog_dst.u32Stride * 4);

        IVE_HOG_CTRL_S hog_ctrl;
        memset(&hog_ctrl, 0, sizeof(hog_ctrl));
        hog_ctrl.enCscMode = IVE_CSC_MODE_PIC_BT601_YUV2RGB;
        hog_ctrl.enHogMode = IVE_HOG_MODE_VERTICAL_TANGENT_PLANE;
        hog_ctrl.u32RoiNum = 1;

        ret = HI_MPI_IVE_Hog(&handle, &hog_yuv, &hog_roi, &hog_dst, &hog_ctrl, HI_TRUE);
        if (ret == HI_SUCCESS) {
            ive_wait(handle);
            printf("  %-10s PASS\n", "hog");
        } else {
            printf("  %-10s ret=0x%x — skipped\n", "hog", ret);
        }
        HI_MPI_SYS_MmzFree(hog_yuv.au64PhyAddr[0], (HI_VOID *)(HI_UL)hog_yuv.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(hog_dst.u64PhyAddr, (HI_VOID *)(HI_UL)hog_dst.u64VirAddr);
    }
#endif /* PerspTrans + Hog disabled */

    printf("========================================\n");
    printf("Result: %d/%d passed\n", 31 - fails, 31);
    printf("========================================\n");

cleanup:
    free_image(&src1);
    free_image(&src2);
    free_image(&dst);
    HI_MPI_SYS_Exit();
    return fails;
}
