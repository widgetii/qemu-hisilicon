/*
 * IVE Output Capture — dumps full pixel output for byte-exact comparison.
 *
 * For each op, runs on real HW via MPI, writes raw output to stdout.
 * Companion script reads the binary and compares against QEMU output.
 *
 * Output format: for each op, W*H bytes (U8) in row-major order.
 * Header: 4-byte magic "IVE\0", then per-op: 4-byte op_id + W*H bytes data.
 *
 * Build: same as test-ive-mpi.c
 * Run: ./test-ive-capture > /utils/ive-test/hw_capture.bin 2>/dev/null
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

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
#define STRIDE 64

static void ive_wait(IVE_HANDLE h) {
    HI_BOOL fin;
    HI_S32 ret;
    do { ret = HI_MPI_IVE_Query(h, &fin, HI_TRUE); } while (ret == HI_ERR_IVE_QUERY_TIMEOUT);
}

static void write_op(uint32_t id, const uint8_t *data, int sz) {
    fwrite(&id, 4, 1, stdout);
    fwrite(data, 1, sz, stdout);
}

static void read_image_raw(IVE_IMAGE_S *img, uint8_t *buf) {
    HI_MPI_SYS_MmzFlushCache(img->au64PhyAddr[0],
        (HI_VOID *)(HI_UL)img->au64VirAddr[0], img->au32Stride[0] * H);
    uint8_t *v = (uint8_t *)(HI_UL)img->au64VirAddr[0];
    for (int y = 0; y < H; y++)
        memcpy(buf + y * W, v + y * img->au32Stride[0], W);
}

int main() {
    HI_S32 ret;
    IVE_HANDLE handle;
    uint8_t result[SZ];

    ret = HI_MPI_SYS_Init();
    if (ret) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* Allocate images */
    IVE_IMAGE_S src1, src2, dst;
    memset(&src1, 0, sizeof(src1)); memset(&src2, 0, sizeof(src2)); memset(&dst, 0, sizeof(dst));
    src1.enType = src2.enType = dst.enType = IVE_IMAGE_TYPE_U8C1;
    src1.u32Width = src2.u32Width = dst.u32Width = W;
    src1.u32Height = src2.u32Height = dst.u32Height = H;
    src1.au32Stride[0] = src2.au32Stride[0] = dst.au32Stride[0] = STRIDE;
    HI_MPI_SYS_MmzAlloc(&src1.au64PhyAddr[0], (HI_VOID **)&src1.au64VirAddr[0], NULL, NULL, STRIDE*H);
    HI_MPI_SYS_MmzAlloc(&src2.au64PhyAddr[0], (HI_VOID **)&src2.au64VirAddr[0], NULL, NULL, STRIDE*H);
    HI_MPI_SYS_MmzAlloc(&dst.au64PhyAddr[0], (HI_VOID **)&dst.au64VirAddr[0], NULL, NULL, STRIDE*H);

    uint8_t *sv1 = (uint8_t *)(HI_UL)src1.au64VirAddr[0];
    uint8_t *sv2 = (uint8_t *)(HI_UL)src2.au64VirAddr[0];
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            sv1[y*STRIDE+x] = ((y*W+x)*7+13) & 0xFF;
            sv2[y*STRIDE+x] = ((y*W+x)*11+31) & 0xFF;
        }
    HI_MPI_SYS_MmzFlushCache(src1.au64PhyAddr[0], sv1, STRIDE*H);
    HI_MPI_SYS_MmzFlushCache(src2.au64PhyAddr[0], sv2, STRIDE*H);

    /* Write header */
    fwrite("IVE\0", 4, 1, stdout);

    /* Dilate */
    memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE*H);
    IVE_DILATE_CTRL_S dil = {{0}}; memset(dil.au8Mask, 255, 25);
    ret = HI_MPI_IVE_Dilate(&handle, &src1, &dst, &dil, HI_TRUE);
    if (ret == 0) { ive_wait(handle); read_image_raw(&dst, result); write_op(1, result, SZ); }
    fprintf(stderr, "dilate: %s\n", ret==0 ? "captured" : "FAIL");

    /* Erode */
    memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE*H);
    IVE_ERODE_CTRL_S ero = {{0}}; memset(ero.au8Mask, 255, 25);
    ret = HI_MPI_IVE_Erode(&handle, &src1, &dst, &ero, HI_TRUE);
    if (ret == 0) { ive_wait(handle); read_image_raw(&dst, result); write_op(2, result, SZ); }
    fprintf(stderr, "erode: %s\n", ret==0 ? "captured" : "FAIL");

    /* EqualizeHist */
    memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE*H);
    IVE_MEM_INFO_S eq_mem; memset(&eq_mem, 0, sizeof(eq_mem));
    eq_mem.u32Size = 256*4+256;
    HI_MPI_SYS_MmzAlloc(&eq_mem.u64PhyAddr, (HI_VOID **)&eq_mem.u64VirAddr, NULL, NULL, eq_mem.u32Size);
    IVE_EQUALIZE_HIST_CTRL_S eq_ctrl = { .stMem = eq_mem };
    ret = HI_MPI_IVE_EqualizeHist(&handle, &src1, &dst, &eq_ctrl, HI_TRUE);
    if (ret == 0) { ive_wait(handle); read_image_raw(&dst, result); write_op(3, result, SZ); }
    fprintf(stderr, "eq_hist: %s\n", ret==0 ? "captured" : "FAIL");
    HI_MPI_SYS_MmzFree(eq_mem.u64PhyAddr, (HI_VOID *)(HI_UL)eq_mem.u64VirAddr);

    /* OrdStatFilter (median) */
    memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE*H);
    IVE_ORD_STAT_FILTER_CTRL_S osf = { .enMode = IVE_ORD_STAT_FILTER_MODE_MEDIAN };
    ret = HI_MPI_IVE_OrdStatFilter(&handle, &src1, &dst, &osf, HI_TRUE);
    if (ret == 0) { ive_wait(handle); read_image_raw(&dst, result); write_op(4, result, SZ); }
    fprintf(stderr, "ordstat: %s\n", ret==0 ? "captured" : "FAIL");

    /* Filter (identity kernel) */
    memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE*H);
    IVE_FILTER_CTRL_S flt; memset(&flt, 0, sizeof(flt));
    flt.as8Mask[12] = 16; flt.u8Norm = 4;
    ret = HI_MPI_IVE_Filter(&handle, &src1, &dst, &flt, HI_TRUE);
    if (ret == 0) { ive_wait(handle); read_image_raw(&dst, result); write_op(5, result, SZ); }
    fprintf(stderr, "filter: %s\n", ret==0 ? "captured" : "FAIL");

    /* Sub */
    memset((void *)(HI_UL)dst.au64VirAddr[0], 0, STRIDE*H);
    IVE_SUB_CTRL_S sub_ctrl = { .enMode = IVE_SUB_MODE_ABS };
    ret = HI_MPI_IVE_Sub(&handle, &src1, &src2, &dst, &sub_ctrl, HI_TRUE);
    if (ret == 0) { ive_wait(handle); read_image_raw(&dst, result); write_op(6, result, SZ); }
    fprintf(stderr, "sub: %s\n", ret==0 ? "captured" : "FAIL");

    HI_MPI_SYS_MmzFree(src1.au64PhyAddr[0], (HI_VOID *)(HI_UL)src1.au64VirAddr[0]);
    HI_MPI_SYS_MmzFree(src2.au64PhyAddr[0], (HI_VOID *)(HI_UL)src2.au64VirAddr[0]);
    HI_MPI_SYS_MmzFree(dst.au64PhyAddr[0], (HI_VOID *)(HI_UL)dst.au64VirAddr[0]);
    HI_MPI_SYS_Exit();
    return 0;
}
