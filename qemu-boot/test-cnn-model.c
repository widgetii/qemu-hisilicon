/*
 * Test IVE CNN API with vendor's .bin model files
 *
 * Uses HI_MPI_IVE_CNN_LoadModel / CNN_Predict / CNN_GetResult
 * instead of the XNN API. The CNN API handles weight transformation
 * internally, bypassing the OMS format issues.
 *
 * Build:
 *   CC=arm-openipc-linux-musleabi-gcc
 *   SDK=Hi3516EV200_SDK_V1.0.1.2/mpp
 *   LIBS=output/target/usr/lib
 *   $CC -O2 -o test-cnn-model test-cnn-model.c \
 *       -I$SDK/include -L$LIBS -lmpi -live \
 *       -lVoiceEngine -lupvqe -ldnvqe -lsecurec \
 *       -Wl,-rpath,/usr/lib -Wl,--allow-shlib-undefined -lm
 *
 * Run: killall majestic; ./test-cnn-model model.bin [image.yuv]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdint.h>
#include <sys/syscall.h>
#include <sys/time.h>

/* mmap shim */
#define PROT_READ 0x1
#define PROT_WRITE 0x2
#define MAP_SHARED 0x01
#define MAP_FAILED ((void *)-1)
int munmap(void *addr, size_t len);
void *mmap(void *start, size_t len, int prot, int flags, int fd, uint32_t off) {
    return (void *)syscall(SYS_mmap2, start, len, prot, flags, fd, off >> 12);
}

/* uclibc shims */
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
#include "mpi_sys.h"
#include "mpi_ive.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <model.bin> [image.yuv]\n", argv[0]);
        return 1;
    }

    /* Init */
    HI_MPI_SYS_Exit();
    HI_S32 ret = HI_MPI_SYS_Init();
    if (ret) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* Skip IVE subsystem init — CNN API may not need it */

    /* Load CNN model */
    IVE_CNN_MODEL_S model;
    memset(&model, 0, sizeof(model));
    ret = HI_MPI_IVE_CNN_LoadModel(argv[1], &model);
    if (ret) {
        fprintf(stderr, "CNN_LoadModel: 0x%x\n", ret);
        goto cleanup_sys;
    }

    fprintf(stderr, "Model loaded: %ux%u type=%d, %d conv layers, %d classes\n",
            model.u32Width, model.u32Height, model.enType,
            model.u8ConvPoolLayerNum, model.u16ClassCount);
    fprintf(stderr, "  ConvKernelBias: phys=0x%llx size=%u\n",
            (unsigned long long)model.stConvKernelBias.u64PhyAddr, model.u32ConvKernelBiasSize);
    fprintf(stderr, "  FCLWgtBias: phys=0x%llx size=%u\n",
            (unsigned long long)model.stFCLWgtBias.u64PhyAddr, model.u32FCLWgtBiasSize);
    fprintf(stderr, "  Total: %u bytes\n", model.u32TotalMemSize);
    /* Dump model as hex to find missing fields */
    fprintf(stderr, "  Raw model struct (%zu bytes):\n", sizeof(model));
    uint8_t *mp = (uint8_t *)&model;
    for (int i = 0; i < (int)sizeof(model) && i < 256; i += 16) {
        fprintf(stderr, "    +%03x:", i);
        for (int j = 0; j < 16 && i+j < (int)sizeof(model); j++)
            fprintf(stderr, " %02x", mp[i+j]);
        fprintf(stderr, "\n");
    }

    /* Prepare input image */
    uint32_t w = model.u32Width;
    uint32_t h = model.u32Height;
    uint32_t stride = (w + 15) & ~15;

    IVE_SRC_IMAGE_S src;
    memset(&src, 0, sizeof(src));
    src.enType = IVE_IMAGE_TYPE_U8C1;
    src.u32Width = w;
    src.u32Height = h;
    src.au32Stride[0] = stride;

    HI_U64 frm_p, frm_v;
    uint32_t frm_size = stride * h;
    HI_MPI_SYS_MmzAlloc(&frm_p, (void **)&frm_v, "F", NULL, frm_size);
    src.au64PhyAddr[0] = frm_p;
    src.au64VirAddr[0] = frm_v;

    /* Fill with test pattern or load from file */
    if (argc > 2) {
        FILE *ff = fopen(argv[2], "rb");
        if (ff) {
            size_t got = fread((void *)(uintptr_t)frm_v, 1, frm_size, ff);
            fclose(ff);
            fprintf(stderr, "Loaded image: %s (%zu bytes)\n", argv[2], got);
        } else {
            memset((void *)(uintptr_t)frm_v, 128, frm_size);
        }
    } else {
        memset((void *)(uintptr_t)frm_v, 128, frm_size);
    }
    HI_MPI_SYS_MmzFlushCache(frm_p, (void *)(uintptr_t)frm_v, frm_size);

    /* Prepare FC output buffer */
    IVE_DST_DATA_S dst;
    memset(&dst, 0, sizeof(dst));
    /* Width = last FC layer neuron count * sizeof(HI_S32) */
    uint32_t fc_out_cnt = model.stFullConnect.au16LayerCnt[model.stFullConnect.u8LayerNum - 1];
    dst.u32Width = fc_out_cnt * sizeof(HI_S32);
    dst.u32Height = 1;  /* 1 input image */
    dst.u32Stride = (dst.u32Width + 15) & ~15;
    uint32_t dst_size = dst.u32Stride * dst.u32Height;
    if (dst_size < 256) dst_size = 256;
    HI_U64 dst_p, dst_v;
    HI_MPI_SYS_MmzAlloc(&dst_p, (void **)&dst_v, "D", NULL, dst_size);
    dst.u64PhyAddr = dst_p;
    dst.u64VirAddr = dst_v;
    memset((void *)(uintptr_t)dst_v, 0, dst_size);
    HI_MPI_SYS_MmzFlushCache(dst_p, (void *)(uintptr_t)dst_v, dst_size);
    fprintf(stderr, "FC output: %u neurons, width=%u stride=%u\n",
            fc_out_cnt, dst.u32Width, dst.u32Stride);

    /* CNN control — needs assist memory */
    IVE_CNN_CTRL_S ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.u32Num = 1;  /* 1 input image */
    /* Allocate assist memory — needs to be large enough for CNN ping-pong buffers.
     * u32TotalMemSize includes weights; the working buffer needs much more. */
    uint32_t assist_size = model.u32TotalMemSize * 4 + 262144;
    HI_U64 assist_p, assist_v;
    HI_MPI_SYS_MmzAlloc(&assist_p, (void **)&assist_v, "A", NULL, assist_size);
    ctrl.stMem.u64PhyAddr = assist_p;
    ctrl.stMem.u64VirAddr = assist_v;
    ctrl.stMem.u32Size = assist_size;

    /* Run prediction */
    IVE_HANDLE handle = 0;
    fprintf(stderr, "Running CNN_Predict...\n");
    ret = HI_MPI_IVE_CNN_Predict(&handle, &src, &model, &dst, &ctrl, HI_TRUE);
    fprintf(stderr, "CNN_Predict ret=0x%x handle=%d\n", ret, handle);

    if (ret == 0) {
        /* Wait for completion */
        HI_BOOL finished = HI_FALSE;
        HI_S32 qret = HI_MPI_IVE_Query(handle, &finished, HI_TRUE);
        int retries = 0;
        while (qret == (HI_S32)0xa01d8041 && retries < 10000) {
            usleep(100);
            qret = HI_MPI_IVE_Query(handle, &finished, HI_TRUE);
            retries++;
        }
        fprintf(stderr, "Query ret=0x%x finished=%d retries=%d\n", qret, finished, retries);

        /* Read output */
        HI_MPI_SYS_MmzFlushCache(dst_p, (void *)(uintptr_t)dst_v, dst_size);
        uint8_t *out = (uint8_t *)(uintptr_t)dst_v;
        fprintf(stderr, "Output (first 64 bytes):\n  ");
        for (int i = 0; i < 64; i++) {
            fprintf(stderr, "%02x ", out[i]);
            if (i % 16 == 15) fprintf(stderr, "\n  ");
        }
        fprintf(stderr, "\n");

        /* Get classification result */
        IVE_SRC_DATA_S result_src;
        memset(&result_src, 0, sizeof(result_src));
        result_src.u64PhyAddr = dst_p;
        result_src.u64VirAddr = dst_v;
        result_src.u32Stride = dst.u32Stride;
        result_src.u32Width = dst.u32Width;
        result_src.u32Height = dst.u32Height;

        IVE_DST_MEM_INFO_S result_dst;
        memset(&result_dst, 0, sizeof(result_dst));
        uint32_t result_size = 1024;
        HI_U64 res_p, res_v;
        HI_MPI_SYS_MmzAlloc(&res_p, (void **)&res_v, "R", NULL, result_size);
        result_dst.u64PhyAddr = res_p;
        result_dst.u64VirAddr = res_v;
        result_dst.u32Size = result_size;
        memset((void *)(uintptr_t)res_v, 0, result_size);

        ret = HI_MPI_IVE_CNN_GetResult(&result_src, &result_dst, &model, &ctrl);
        fprintf(stderr, "GetResult ret=0x%x\n", ret);
        if (ret == 0) {
            uint32_t *results = (uint32_t *)(uintptr_t)res_v;
            fprintf(stderr, "Classification results:\n");
            for (int i = 0; i < model.u16ClassCount && i < 20; i++) {
                fprintf(stderr, "  class %d: label=%u confidence=%u\n",
                        i, results[i*2], results[i*2+1]);
            }
        }
        HI_MPI_SYS_MmzFree(res_p, (void *)(uintptr_t)res_v);
    }

    /* Cleanup */
    HI_MPI_IVE_CNN_UnloadModel(&model);
    HI_MPI_SYS_MmzFree(dst_p, (void *)(uintptr_t)dst_v);
    HI_MPI_SYS_MmzFree(frm_p, (void *)(uintptr_t)frm_v);
cleanup_sys:
    HI_MPI_SYS_Exit();
    return ret ? 1 : 0;
}
