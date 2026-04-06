/*
 * Test custom FC model on IVE XNN — uses only vendor SDK functions
 *
 * Follows the exact call pattern from svp_alg_forward_slice:
 *   mp_ive_svp_alg_proc_init → mpi_ive_xnn_loadmodel →
 *   mpi_ive_xnn_forward_slice → HI_MPI_IVE_Query (retry loop) →
 *   mpi_ive_xnn_unloadmodel → mp_ive_svp_alg_proc_exit
 *
 * No raw ioctls. No fd scanning. Just the libive.so functions.
 *
 * Build:
 *   CC=~/git/firmware/output-hi3516ev300_lite/host/bin/arm-openipc-linux-musleabi-gcc
 *   SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
 *   LIBS=~/git/firmware/output-hi3516ev300_lite/target/usr/lib
 *   $CC -O2 -o test-fc-model test-fc-model.c \
 *       -I$SDK/include -L$LIBS -lmpi -live \
 *       -lVoiceEngine -lupvqe -ldnvqe -lsecurec \
 *       -Wl,-rpath,/usr/lib -Wl,--allow-shlib-undefined -lm
 *
 * Run: killall majestic; ./test-fc-model model.oms
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

typedef struct { HI_U64 phys, virt; HI_U32 size, pad; } xnn_mem_t;

/* Private libive.so functions */
extern HI_S32 mpi_ive_xnn_loadmodel(xnn_mem_t *model, xnn_mem_t *tmp, void *params);
extern HI_S32 mpi_ive_xnn_forward_slice(
    HI_S32 *handle, void *src, void *dst, void *model,
    xnn_mem_t *tmp, void *report, void *ctrl, HI_S32 instant);
extern HI_VOID mpi_ive_xnn_unloadmodel(void *params);
extern HI_S32 mp_ive_svp_alg_proc_init(void *a, void *b);
extern HI_VOID mp_ive_svp_alg_proc_exit(void);

/* IVE_BLOB_S — 48 bytes */
typedef struct {
    HI_U32 type;
    HI_U32 stride;
    HI_U64 virt;
    HI_U64 phys;
    HI_U32 num;
    HI_U32 reserved;
    HI_U32 width;
    HI_U32 height;
    HI_U32 chn;
    HI_U32 pad;
} xnn_blob_t;

static int find_segment(const uint8_t *data, int size) {
    for (int off = 0x40; off < size - 80 && off < 0x200; off += 0x10) {
        uint16_t s = *(uint16_t *)(data + off + 48);
        uint16_t l = *(uint16_t *)(data + off + 50);
        uint8_t sn = data[off + 52], dn = data[off + 53];
        if (s >= 1 && s <= 16 && l >= 1 && l <= 500 && sn >= 1 && sn <= 16 && dn >= 1 && dn <= 16)
            return off;
    }
    return -1;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <model.oms>\n", argv[0]);
        return 1;
    }

    /* Read OMS file */
    FILE *fp = fopen(argv[1], "rb");
    if (!fp) { perror(argv[1]); return 1; }
    fseek(fp, 0, SEEK_END);
    long file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    uint8_t *file_data = malloc(file_size);
    fread(file_data, 1, file_size, fp);
    fclose(fp);

    int seg_off = find_segment(file_data, file_size);
    if (seg_off < 0) { fprintf(stderr, "No segment found\n"); return 1; }

    uint32_t seg_size = *(uint32_t *)(file_data + seg_off + 4);
    uint32_t tmp_size = *(uint32_t *)(file_data + seg_off + 12);
    fprintf(stderr, "Segment at 0x%x, %u layers, tmp=%u\n",
            seg_off, *(uint16_t *)(file_data + seg_off + 50), tmp_size);

    /* Init */
    HI_MPI_SYS_Exit();
    HI_S32 ret = HI_MPI_SYS_Init();
    if (ret) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    uint8_t sa[16]={0}, sb[16]={0};
    mp_ive_svp_alg_proc_init(sa, sb);

    /* Alloc model in MMZ */
    xnn_mem_t model_mem = {0};
    uint32_t seg_data_size = file_size - seg_off;
    HI_MPI_SYS_MmzAlloc(&model_mem.phys, (void **)&model_mem.virt, "M", NULL, seg_data_size);
    model_mem.size = seg_data_size;
    memcpy((void *)(uintptr_t)model_mem.virt, file_data + seg_off, seg_data_size);

    /* Alloc tmp */
    xnn_mem_t tmp_mem = {0};
    HI_MPI_SYS_MmzAlloc(&tmp_mem.phys, (void **)&tmp_mem.virt, "T", NULL, tmp_size);
    tmp_mem.size = tmp_size;

    /* Load model */
    uint8_t params[2160] = {0};
    ret = mpi_ive_xnn_loadmodel(&model_mem, &tmp_mem, params);
    if (ret) { fprintf(stderr, "loadmodel: 0x%x\n", ret); goto cleanup; }

    /* Read input dimensions from model_params (src_node_arr[0]) */
    uint32_t in_w = *(uint32_t *)(params + 0x14);  /* +20: width */
    uint32_t in_h = *(uint32_t *)(params + 0x18);  /* +24: height */
    uint32_t in_chn = *(uint32_t *)(params + 0x1C); /* +28: chn */

    /* Read output dimensions from model_params (dst_node at +1232) */
    uint32_t out_type = *(uint32_t *)(params + 1232); /* blob_type */
    uint32_t out_w = *(uint32_t *)(params + 1236);
    uint32_t out_h = *(uint32_t *)(params + 1240);
    uint32_t out_chn = *(uint32_t *)(params + 1244);
    /* Vendor check: stride >= align16(4 * width), always 4 bytes/element */
    uint32_t out_stride = ((out_w * 4) + 15) & ~15;

    fprintf(stderr, "Model: in=%ux%ux%u out=%ux%ux%u type=%u stride=%u\n",
            in_w, in_h, in_chn, out_w, out_h, out_chn, out_type, out_stride);

    /* Dump full model_params for debugging */
    fprintf(stderr, "params dump (non-zero regions):\n");
    for (int i = 0; i < 2160; i += 16) {
        int nz = 0;
        for (int j = 0; j < 16 && i+j < 2160; j++)
            if (params[i+j]) nz++;
        if (nz) {
            fprintf(stderr, "  +%04d: ", i);
            for (int j = 0; j < 16 && i+j < 2160; j++)
                fprintf(stderr, "%02x ", params[i+j]);
            fprintf(stderr, "\n");
        }
    }

    /* Alloc frame (YUV420SP) */
    uint32_t stride = (in_w + 15) & ~15;
    uint32_t y_sz = stride * in_h;
    uint32_t uv_sz = stride * in_h / 2;
    HI_U64 frm_p, frm_v;
    HI_MPI_SYS_MmzAlloc(&frm_p, (void **)&frm_v, "F", NULL, y_sz + uv_sz);
    if (argc > 2) {
        FILE *ff = fopen(argv[2], "rb");
        if (ff) {
            size_t got = fread((void *)(uintptr_t)frm_v, 1, y_sz + uv_sz, ff);
            fclose(ff);
            fprintf(stderr, "Loaded frame: %s (%zu bytes)\n", argv[2], got);
        } else {
            memset((void *)(uintptr_t)frm_v, 128, y_sz);
            memset((void *)(uintptr_t)frm_v + y_sz, 128, uv_sz);
        }
    } else {
        memset((void *)(uintptr_t)frm_v, 128, y_sz);
        memset((void *)(uintptr_t)frm_v + y_sz, 128, uv_sz);
    }
    HI_MPI_SYS_MmzFlushCache(frm_p, (void *)(uintptr_t)frm_v, y_sz + uv_sz);

    /* Alloc output */
    uint32_t out_buf_size = out_stride * out_h * out_chn;
    if (out_buf_size < 256) out_buf_size = 256;
    HI_U64 out_p, out_v;
    HI_MPI_SYS_MmzAlloc(&out_p, (void **)&out_v, "O", NULL, out_buf_size);
    memset((void *)(uintptr_t)out_v, 0xAA, out_buf_size);
    HI_MPI_SYS_MmzFlushCache(out_p, (void *)(uintptr_t)out_v, out_buf_size);

    /* Fill tmp with 0xAA pattern */
    memset((void *)(uintptr_t)tmp_mem.virt, 0xAA, tmp_size);
    HI_MPI_SYS_MmzFlushCache(tmp_mem.phys, (void *)(uintptr_t)tmp_mem.virt, tmp_size);

    /* Build forward args */
    xnn_blob_t src = {0};
    src.type = 2; /* YVU420SP */
    src.stride = stride;
    src.virt = frm_v;
    src.phys = frm_p;
    src.num = 1;
    src.width = in_w;
    src.height = in_h;
    src.chn = in_chn ? in_chn : 3;

    xnn_blob_t dst = {0};
    dst.type = out_type; /* 0=S32, 7=U8, etc — use model's actual type */
    dst.stride = out_stride;
    dst.virt = out_v;
    dst.phys = out_p;
    dst.num = 1;
    dst.width = out_w;
    dst.height = out_h;
    dst.chn = out_chn;

    /* ctrl: a5[0]=0, a5[1]=src_num, a5[2]=has_roi, a5[3]=dst_num */
    uint32_t ctrl[4] = {0, 1, 0, 1};

    HI_S32 handle = 0;

    /* Verify struct layout */
    fprintf(stderr, "sizeof(xnn_blob_t)=%zu\n", sizeof(xnn_blob_t));
    fprintf(stderr, "src blob bytes: ");
    for (int i = 0; i < (int)sizeof(xnn_blob_t); i++)
        fprintf(stderr, "%02x ", ((uint8_t *)&src)[i]);
    fprintf(stderr, "\ndst blob bytes: ");
    for (int i = 0; i < (int)sizeof(xnn_blob_t); i++)
        fprintf(stderr, "%02x ", ((uint8_t *)&dst)[i]);
    fprintf(stderr, "\n");
    fprintf(stderr, "Running forward...\n");
    ret = mpi_ive_xnn_forward_slice(
        &handle, &src, &dst, params,
        &tmp_mem, &dst, ctrl, 1);
    fprintf(stderr, "forward ret=0x%x handle=%d\n", ret, handle);

    /* Query loop — only if handle is non-zero (async task pending).
     * handle=0 with ret=0 means synchronous completion (instant=1). */
    if (ret == 0 && handle != 0) {
        HI_BOOL finished = HI_FALSE;
        HI_S32 qret = HI_MPI_IVE_Query(handle, &finished, HI_TRUE);
        int retries = 0;
        while (qret == (HI_S32)0xa01d8041 && retries < 10000) {
            usleep(100);
            qret = HI_MPI_IVE_Query(handle, &finished, HI_TRUE);
            retries++;
        }
        fprintf(stderr, "query ret=0x%x finished=%d retries=%d\n", qret, finished, retries);
    } else if (ret == 0) {
        fprintf(stderr, "instant completion (handle=0), querying anyway...\n");
        HI_BOOL finished = HI_FALSE;
        HI_MPI_IVE_Query(handle, &finished, HI_TRUE);
        fprintf(stderr, "query finished=%d\n", finished);
        usleep(100000);
    }

    /* Read results */
    HI_MPI_SYS_MmzFlushCache(tmp_mem.phys, (void *)(uintptr_t)tmp_mem.virt, tmp_size);
    HI_MPI_SYS_MmzFlushCache(out_p, (void *)(uintptr_t)out_v, 65536);

    /* Dump tmp buffer to file for offline analysis */
    FILE *df = fopen("/tmp/tmp_dump.bin", "wb");
    if (df) {
        fwrite((void *)(uintptr_t)tmp_mem.virt, 1, tmp_size, df);
        fclose(df);
        fprintf(stderr, "Dumped %u bytes of tmp to /tmp/tmp_dump.bin\n", tmp_size);
    }

    /* Read HW task pointer and dump task chain via /dev/mem */
    {
        int memfd = open("/dev/mem", 2); /* O_RDWR */
        if (memfd >= 0) {
            /* Map IVE HW regs at 0x11320000 */
            volatile uint32_t *regs = mmap(NULL, 0x100, PROT_READ, MAP_SHARED, memfd, 0x11320000);
            if (regs != MAP_FAILED) {
                uint32_t task_ptr = regs[4]; /* reg 0x10 = task pointer */
                fprintf(stderr, "HW task_ptr=0x%08x\n", task_ptr);
                munmap((void *)regs, 0x100);

                if (task_ptr >= 0x42000000 && task_ptr < 0x44000000) {
                    /* Map task chain region */
                    uint32_t page = task_ptr & ~0xFFF;
                    volatile uint8_t *task = mmap(NULL, 0x2000, PROT_READ, MAP_SHARED, memfd, page);
                    if (task != MAP_FAILED) {
                        uint32_t off = task_ptr - page;
                        FILE *tf = fopen("/tmp/task_dump.bin", "wb");
                        if (tf) {
                            fwrite((void *)(task + off), 1, 0x1000, tf);
                            fclose(tf);
                            fprintf(stderr, "Dumped task chain to /tmp/task_dump.bin\n");
                        }
                        /* Print first 2 nodes */
                        for (int n = 0; n < 2; n++) {
                            fprintf(stderr, "Task node %d:\n", n);
                            for (int r = 0; r < 128; r += 16) {
                                fprintf(stderr, "  +%02x:", r);
                                for (int b = 0; b < 16; b++)
                                    fprintf(stderr, " %02x", task[off + n*128 + r + b]);
                                fprintf(stderr, "\n");
                            }
                        }
                        munmap((void *)task, 0x2000);
                    }
                }
            }
            close(memfd);
        }
    }

    /* Scan tmp for HW-written regions (not 0xAA), dump each */
    int8_t *tmp = (int8_t *)(uintptr_t)tmp_mem.virt;
    fprintf(stderr, "HW-written regions:\n");
    int rs = -1;
    for (uint32_t i = 0; i < tmp_size; i += 16) {
        int changed = 0;
        for (int j = 0; j < 16 && i+j < tmp_size; j++)
            if ((uint8_t)tmp[i+j] != 0xAA) changed++;
        if (changed && rs < 0) rs = i;
        else if (!changed && rs >= 0) {
            int sz = (int)i - rs;
            fprintf(stderr, "  0x%04x-0x%04x (%d bytes)\n", rs, (int)i, sz);
            /* Dump first 64 bytes */
            fprintf(stderr, "    ");
            for (int j = rs; j < rs + 64 && j < (int)i; j++) {
                fprintf(stderr, "%02x ", (uint8_t)tmp[j]);
                if ((j - rs) % 16 == 15) fprintf(stderr, "\n    ");
            }
            if (sz > 128) {
                fprintf(stderr, "  ...\n    ");
                for (int j = (int)i - 64; j < (int)i; j++) {
                    fprintf(stderr, "%02x ", (uint8_t)tmp[j]);
                    if ((j - ((int)i - 64)) % 16 == 15) fprintf(stderr, "\n    ");
                }
            }
            rs = -1;
        }
    }
    if (rs >= 0)
        fprintf(stderr, "  0x%04x-end (%d bytes)\n", rs, (int)(tmp_size - rs));

    /* Dump all HW-written regions with data */
    rs = -1;
    for (uint32_t i = 0; i < tmp_size; i += 16) {
        int changed = 0;
        for (int j = 0; j < 16 && i+j < tmp_size; j++)
            if ((uint8_t)tmp[i+j] != 0xAA) changed++;
        if (changed && rs < 0) rs = i;
        else if (!changed && rs >= 0) {
            int region_sz = i - rs;
            fprintf(stderr, "  Region 0x%04x-0x%04x (%d bytes):", rs, (int)i, region_sz);
            /* Dump first and last 32 bytes */
            fprintf(stderr, "\n    first: ");
            for (int j = rs; j < rs + 32 && j < (int)i; j++)
                fprintf(stderr, "%02x ", (uint8_t)tmp[j]);
            if (region_sz > 64) {
                fprintf(stderr, "\n    last:  ");
                for (int j = (int)i - 32; j < (int)i; j++)
                    fprintf(stderr, "%02x ", (uint8_t)tmp[j]);
            }
            fprintf(stderr, "\n");
            rs = -1;
        }
    }
    if (rs >= 0) {
        fprintf(stderr, "  Region 0x%04x-end:\n    last: ", rs);
        int end = tmp_size < (uint32_t)(rs + 256) ? tmp_size : rs + 256;
        for (int j = rs; j < end; j++)
            fprintf(stderr, "%02x ", (uint8_t)tmp[j]);
        fprintf(stderr, "\n");
    }

    /* Check output buffer */
    uint8_t *ob = (uint8_t *)(uintptr_t)out_v;
    int ob_changed = 0;
    for (int i = 0; i < 256; i++) if (ob[i] != 0xAA) ob_changed++;
    fprintf(stderr, "out_buf changed: %d/256\n", ob_changed);
    if (ob_changed) {
        fprintf(stderr, "out_buf: ");
        for (int i = 0; i < 64; i++) fprintf(stderr, "%02x ", ob[i]);
        fprintf(stderr, "\n");
    }

    /* Cleanup — unload may need two calls if forward left ref_cnt=2 */
    mpi_ive_xnn_unloadmodel(params);
    mpi_ive_xnn_unloadmodel(params);
    HI_MPI_SYS_MmzFree(out_p, (void *)(uintptr_t)out_v);
    HI_MPI_SYS_MmzFree(frm_p, (void *)(uintptr_t)frm_v);

cleanup:
    HI_MPI_SYS_MmzFree(tmp_mem.phys, (void *)(uintptr_t)tmp_mem.virt);
    HI_MPI_SYS_MmzFree(model_mem.phys, (void *)(uintptr_t)model_mem.virt);
    mp_ive_svp_alg_proc_exit();
    free(file_data);
    HI_MPI_SYS_Exit();
    return 0;
}
