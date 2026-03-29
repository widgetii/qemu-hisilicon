/*
 * Test OMS model loading — validates our generated .oms files
 *
 * Loads an OMS file via mpi_ive_xnn_loadmodel and reports success/failure.
 * Scans for the segment header automatically (supports v1003/v1006+ headers).
 *
 * Build:
 *   CC=~/git/firmware/output-hi3516ev300_lite/host/bin/arm-openipc-linux-musleabi-gcc
 *   SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
 *   LIBS=~/git/firmware/output-hi3516ev300_lite/target/usr/lib
 *   $CC -O2 -o test-oms-load test-oms-load.c \
 *       -I$SDK/include -L$LIBS -lmpi -live \
 *       -lVoiceEngine -lupvqe -ldnvqe -lsecurec \
 *       -Wl,-rpath,/usr/lib -Wl,--allow-shlib-undefined -lm
 *
 * Run: killall majestic; ./test-oms-load model.oms
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdint.h>
#include <sys/syscall.h>

#define PROT_READ   0x1
#define PROT_WRITE  0x2
#define MAP_SHARED  0x01
#define MAP_FAILED  ((void *)-1)
int munmap(void *addr, size_t len);
void *mmap(void *start, size_t len, int prot, int flags, int fd, uint32_t off) {
    return (void *)syscall(SYS_mmap2, start, len, prot, flags, fd, off >> 12);
}

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

typedef struct { uint64_t phys, virt; uint32_t size, pad; } xnn_mem_t;

extern int mpi_ive_xnn_loadmodel(void *model_mem, void *tmp_mem, void *model_params);
extern void mpi_ive_xnn_unloadmodel(void *model_params);
extern int mp_ive_svp_alg_proc_init(void *a, void *b);
extern void mp_ive_svp_alg_proc_exit(void);

/* Find segment start by scanning for valid header */
static int find_segment(const uint8_t *data, int file_size) {
    for (int off = 0x40; off < file_size - 80 && off < 0x200; off += 0x10) {
        uint16_t scale = *(uint16_t *)(data + off + 48);
        uint16_t layers = *(uint16_t *)(data + off + 50);
        uint8_t src = data[off + 52];
        uint8_t dst = data[off + 53];
        if (scale >= 1 && scale <= 16 && layers >= 1 && layers <= 500 &&
            src >= 1 && src <= 16 && dst >= 1 && dst <= 16) {
            uint32_t seg_size = *(uint32_t *)(data + off + 4);
            if (seg_size > 0 && seg_size <= (uint32_t)file_size) {
                return off;
            }
        }
    }
    return -1;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <model.oms>\n", argv[0]);
        return 1;
    }

    FILE *fp = fopen(argv[1], "rb");
    if (!fp) { perror(argv[1]); return 1; }
    fseek(fp, 0, SEEK_END);
    long file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    uint8_t *file_data = malloc(file_size);
    fread(file_data, 1, file_size, fp);
    fclose(fp);

    fprintf(stderr, "File: %s (%ld bytes)\n", argv[1], file_size);

    /* Find segment */
    int seg_off = find_segment(file_data, file_size);
    if (seg_off < 0) {
        fprintf(stderr, "No valid segment header found\n");
        return 1;
    }

    uint16_t layers = *(uint16_t *)(file_data + seg_off + 50);
    uint8_t src_num = file_data[seg_off + 52];
    uint8_t dst_num = file_data[seg_off + 53];
    uint32_t seg_size = *(uint32_t *)(file_data + seg_off + 4);
    uint32_t tmp_size = *(uint32_t *)(file_data + seg_off + 12);

    fprintf(stderr, "Segment at offset 0x%x: %u layers, src=%u dst=%u size=%u tmp=%u\n",
            seg_off, layers, src_num, dst_num, seg_size, tmp_size);

    /* Init MPI */
    int ret = HI_MPI_SYS_Init();
    if (ret != 0) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* SVP init */
    uint8_t a[16] = {0}, b[16] = {0};
    mp_ive_svp_alg_proc_init(a, b);

    /* Allocate model in MMZ */
    xnn_mem_t model_mem = {0};
    uint32_t seg_data_size = file_size - seg_off;
    ret = HI_MPI_SYS_MmzAlloc(&model_mem.phys, (void **)&model_mem.virt,
                                "MODEL", NULL, seg_data_size);
    if (ret != 0) { fprintf(stderr, "MmzAlloc model: 0x%x\n", ret); return 1; }
    model_mem.size = seg_data_size;
    memcpy((void *)(uintptr_t)model_mem.virt, file_data + seg_off, seg_data_size);

    /* Allocate tmp buffer */
    if (tmp_size < 4096) tmp_size = 4096;
    xnn_mem_t tmp_mem = {0};
    ret = HI_MPI_SYS_MmzAlloc(&tmp_mem.phys, (void **)&tmp_mem.virt,
                                "TMP", NULL, tmp_size);
    if (ret != 0) { fprintf(stderr, "MmzAlloc tmp: 0x%x\n", ret); return 1; }
    tmp_mem.size = tmp_size;

    /* Load model */
    uint8_t model_params[2160];
    memset(model_params, 0, sizeof(model_params));

    fprintf(stderr, "Loading model (seg_off=0x%x, size=%u, tmp=%u)...\n",
            seg_off, seg_data_size, tmp_size);

    /* Dump first 80 bytes of segment for debugging */
    fprintf(stderr, "Segment header:\n");
    uint8_t *seg = file_data + seg_off;
    for (int i = 0; i < 80; i += 16) {
        fprintf(stderr, "  +0x%02x:", i);
        for (int j = 0; j < 16; j++) fprintf(stderr, " %02x", seg[i+j]);
        fprintf(stderr, "\n");
    }

    ret = mpi_ive_xnn_loadmodel(&model_mem, &tmp_mem, model_params);
    fprintf(stderr, "mpi_ive_xnn_loadmodel: ret=0x%x\n", ret);

    if (ret == 0) {
        fprintf(stderr, "SUCCESS! Model loaded.\n");
        fprintf(stderr, "  model_params[0x04] (tmp_size): %u\n",
                *(uint32_t *)(model_params + 0x04));
        fprintf(stderr, "  model_params[0x08] (src_num):  %u\n",
                *(uint32_t *)(model_params + 0x08));
        fprintf(stderr, "  model_params[0x14] (width):    %u\n",
                *(uint32_t *)(model_params + 0x14));
        fprintf(stderr, "  model_params[0x18] (height):   %u\n",
                *(uint32_t *)(model_params + 0x18));
        fprintf(stderr, "  model_params[0x1C] (channels): %u\n",
                *(uint32_t *)(model_params + 0x1C));
        fprintf(stderr, "  model_params[0x24] (name):     %.32s\n",
                model_params + 0x24);
        mpi_ive_xnn_unloadmodel(model_params);
    } else {
        fprintf(stderr, "FAILED. Error 0x%x\n", ret);
        /* Common errors:
         * 0xa01d8003 = ILLEGAL_PARAM (header validation failed)
         * 0xa01d8006 = NULL_PTR
         */
    }

    HI_MPI_SYS_MmzFree(tmp_mem.phys, (void *)(uintptr_t)tmp_mem.virt);
    HI_MPI_SYS_MmzFree(model_mem.phys, (void *)(uintptr_t)model_mem.virt);
    mp_ive_svp_alg_proc_exit();
    free(file_data);
    HI_MPI_SYS_Exit();
    return ret != 0;
}
