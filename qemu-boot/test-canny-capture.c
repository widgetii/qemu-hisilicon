/*
 * Canny Edge Capture — dumps pixel-level output for QEMU/HW comparison.
 *
 * Captures from real hardware:
 *   1. MagAndAng gradient magnitude (U16C1, 64×64)
 *   2. CannyHysEdge + CannyEdge final edge image (U8C1, 64×64)
 *
 * Both use identical test pattern: src[i] = (i*7+13) & 0xFF
 * Same 5×5 Sobel mask, same thresholds (lo=30, hi=100).
 *
 * Output: writes raw binary to stdout (magnitude U16 + edge U8).
 *
 * Build:
 *   $CC -o test-canny-capture test-canny-capture.c \
 *       -I$SDK/include -L$LIBS -lmpi -live ... -Wl,--allow-shlib-undefined
 *
 * Run on real board:
 *   ./test-canny-capture > /utils/ive-test/canny_hw.bin 2>/dev/null
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#include "hi_ive.h"
#include "mpi_sys.h"
#include "mpi_ive.h"

#define W 64
#define H 64
#define SZ (W * H)
#define STRIDE ((W + 15) & ~15)  /* 64 */

static void ive_wait(IVE_HANDLE h) {
    HI_BOOL fin;
    HI_S32 ret;
    do { ret = HI_MPI_IVE_Query(h, &fin, HI_TRUE); } while (ret == HI_ERR_IVE_QUERY_TIMEOUT);
}

int main(int argc, char **argv) {
    HI_S32 ret;
    IVE_HANDLE handle;
    HI_S8 mask[25] = {0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};

    ret = HI_MPI_SYS_Init();
    if (ret != HI_SUCCESS) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* Allocate source image */
    IVE_IMAGE_S src;
    memset(&src, 0, sizeof(src));
    src.enType = IVE_IMAGE_TYPE_U8C1;
    src.u32Width = W; src.u32Height = H;
    src.au32Stride[0] = STRIDE;
    HI_MPI_SYS_MmzAlloc(&src.au64PhyAddr[0], (HI_VOID **)&src.au64VirAddr[0],
                         NULL, HI_NULL, STRIDE * H);

    /* Fill with test pattern: (i*7+13) & 0xFF */
    uint8_t *sv = (uint8_t *)(HI_UL)src.au64VirAddr[0];
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            sv[y * STRIDE + x] = ((y * W + x) * 7 + 13) & 0xFF;
    HI_MPI_SYS_MmzFlushCache(src.au64PhyAddr[0], sv, STRIDE * H);

    /* === 1. MagAndAng — capture gradient magnitude === */
    IVE_IMAGE_S mag_dst;
    memset(&mag_dst, 0, sizeof(mag_dst));
    mag_dst.enType = IVE_IMAGE_TYPE_U16C1;
    mag_dst.u32Width = W; mag_dst.u32Height = H;
    mag_dst.au32Stride[0] = STRIDE;
    HI_MPI_SYS_MmzAlloc(&mag_dst.au64PhyAddr[0], (HI_VOID **)&mag_dst.au64VirAddr[0],
                         NULL, HI_NULL, STRIDE * H * 2);

    IVE_MAG_AND_ANG_CTRL_S mag_ctrl = {
        .enOutCtrl = IVE_MAG_AND_ANG_OUT_CTRL_MAG,
        .u16Thr = 0
    };
    memcpy(mag_ctrl.as8Mask, mask, 25);

    ret = HI_MPI_IVE_MagAndAng(&handle, &src, &mag_dst, HI_NULL, &mag_ctrl, HI_TRUE);
    if (ret != HI_SUCCESS) { fprintf(stderr, "MagAndAng failed: 0x%x\n", ret); return 1; }
    ive_wait(handle);

    HI_MPI_SYS_MmzFlushCache(mag_dst.au64PhyAddr[0],
        (HI_VOID *)(HI_UL)mag_dst.au64VirAddr[0], STRIDE * H * 2);

    /* Write magnitude (W*H U16 values, row by row without stride padding) */
    uint16_t *mv = (uint16_t *)(HI_UL)mag_dst.au64VirAddr[0];
    for (int y = 0; y < H; y++)
        fwrite(mv + y * STRIDE, sizeof(uint16_t), W, stdout);

    fprintf(stderr, "MagAndAng: mag[1,1]=%d mag[5,5]=%d mag[32,32]=%d\n",
            mv[STRIDE + 1], mv[5 * STRIDE + 5], mv[32 * STRIDE + 32]);

    /* === 2. CannyHysEdge + CannyEdge — capture edge output === */
    IVE_IMAGE_S edge;
    memset(&edge, 0, sizeof(edge));
    edge.enType = IVE_IMAGE_TYPE_U8C1;
    edge.u32Width = W; edge.u32Height = H;
    edge.au32Stride[0] = STRIDE;
    HI_MPI_SYS_MmzAlloc(&edge.au64PhyAddr[0], (HI_VOID **)&edge.au64VirAddr[0],
                         NULL, HI_NULL, STRIDE * H);

    IVE_MEM_INFO_S canny_mem;
    memset(&canny_mem, 0, sizeof(canny_mem));
    canny_mem.u32Size = STRIDE * H * 3;
    HI_MPI_SYS_MmzAlloc(&canny_mem.u64PhyAddr, (HI_VOID **)&canny_mem.u64VirAddr,
                         NULL, HI_NULL, canny_mem.u32Size);

    IVE_DST_MEM_INFO_S canny_stack;
    memset(&canny_stack, 0, sizeof(canny_stack));
    canny_stack.u32Size = STRIDE * H * 4 + 16;
    HI_MPI_SYS_MmzAlloc(&canny_stack.u64PhyAddr, (HI_VOID **)&canny_stack.u64VirAddr,
                         NULL, HI_NULL, canny_stack.u32Size);

    IVE_CANNY_HYS_EDGE_CTRL_S canny_ctrl;
    memset(&canny_ctrl, 0, sizeof(canny_ctrl));
    canny_ctrl.stMem = canny_mem;
    canny_ctrl.u16LowThr = 30;
    canny_ctrl.u16HighThr = 100;
    memcpy(canny_ctrl.as8Mask, mask, 25);

    ret = HI_MPI_IVE_CannyHysEdge(&handle, &src, &edge, &canny_stack, &canny_ctrl, HI_TRUE);
    if (ret != HI_SUCCESS) { fprintf(stderr, "CannyHysEdge failed: 0x%x\n", ret); return 1; }
    ive_wait(handle);

    ret = HI_MPI_IVE_CannyEdge(&edge, &canny_stack);
    if (ret != HI_SUCCESS) { fprintf(stderr, "CannyEdge failed: 0x%x\n", ret); return 1; }

    HI_MPI_SYS_MmzFlushCache(edge.au64PhyAddr[0],
        (HI_VOID *)(HI_UL)edge.au64VirAddr[0], STRIDE * H);

    /* Write edge image (W*H U8 values, row by row without stride padding) */
    uint8_t *ev = (uint8_t *)(HI_UL)edge.au64VirAddr[0];
    for (int y = 0; y < H; y++)
        fwrite(ev + y * STRIDE, 1, W, stdout);

    int edges = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (ev[y * STRIDE + x] == 255) edges++;
    fprintf(stderr, "Canny: %d edges (lo=%d hi=%d)\n", edges,
            canny_ctrl.u16LowThr, canny_ctrl.u16HighThr);

    /* Cleanup */
    HI_MPI_SYS_MmzFree(src.au64PhyAddr[0], (HI_VOID *)(HI_UL)src.au64VirAddr[0]);
    HI_MPI_SYS_MmzFree(mag_dst.au64PhyAddr[0], (HI_VOID *)(HI_UL)mag_dst.au64VirAddr[0]);
    HI_MPI_SYS_MmzFree(edge.au64PhyAddr[0], (HI_VOID *)(HI_UL)edge.au64VirAddr[0]);
    HI_MPI_SYS_MmzFree(canny_mem.u64PhyAddr, (HI_VOID *)(HI_UL)canny_mem.u64VirAddr);
    HI_MPI_SYS_MmzFree(canny_stack.u64PhyAddr, (HI_VOID *)(HI_UL)canny_stack.u64VirAddr);
    HI_MPI_SYS_Exit();
    return 0;
}
