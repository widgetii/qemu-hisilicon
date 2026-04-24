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
/* 8 args: handle, src, dst, model, tmp_buf, report, ctrl, instant */
extern int mpi_ive_xnn_forward_slice(
    void *handle, void *src, void *dst, void *model,
    void *tmp, void *report, void *ctrl, int instant);
extern void mpi_ive_xnn_unloadmodel(void *model_params);
extern int mp_ive_svp_alg_proc_init(void *a, void *b);
extern void mp_ive_svp_alg_proc_exit(void);
extern int ioctl(int fd, unsigned long request, ...);

/* HI_MPI_IVE_Query — waits for IVE task completion and cleans up */
extern int HI_MPI_IVE_Query(int handle, int *finished, int block);

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

    /* Init MPI — call Exit first to clean up any previous crashed session */
    HI_MPI_SYS_Exit();
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

        /* Dump Xnn_Task buffer from vendor driver BEFORE forward */
        {
            FILE *mm = fopen("/proc/media-mem", "r");
            if (mm) {
                char line[256];
                while (fgets(line, sizeof(line), mm)) {
                    unsigned int p1, p2;
                    char name[64] = {0};
                    if (sscanf(line, "   |-MMB: phys(0x%x, 0x%x)%*[^,], length=%*[^,],%*[ ]name=\"%63[^\"]\"",
                               &p1, &p2, name) == 3) {
                        uint32_t len = p2 - p1 + 1;
                        if (strstr(name, "Xnn") || strstr(name, "IVE_TEMP") || strstr(name, "IVE_QUEUE")) {
                            void *v = HI_MPI_SYS_Mmap(p1, len);
                            if (v) {
                                uint32_t dumplen = len > 512 ? 512 : len;
                                fprintf(stderr, "\n=== %s phys=0x%x len=%u (before forward) ===\n", name, p1, len);
                                uint8_t *p = (uint8_t *)v;
                                for (uint32_t i = 0; i < dumplen; i += 16) {
                                    fprintf(stderr, "%04x:", i);
                                    for (int j = 0; j < 16 && i+j < dumplen; j++)
                                        fprintf(stderr, " %02x", p[i+j]);
                                    fprintf(stderr, "\n");
                                }
                                HI_MPI_SYS_Munmap(v, len);
                            }
                        }
                    }
                }
                fclose(mm);
            }
        }

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
        /* Try forward inference */
        uint32_t in_w = *(uint32_t *)(model_params + 0x14);
        uint32_t in_h = *(uint32_t *)(model_params + 0x18);
        uint32_t in_c = *(uint32_t *)(model_params + 0x1C);
        uint32_t stride = (in_w + 15) & ~15;
        uint32_t y_sz = stride * in_h;
        uint32_t uv_sz = stride * in_h / 2;

        /* Allocate frame buffer */
        uint64_t frm_p, frm_v;
        ret = HI_MPI_SYS_MmzAlloc(&frm_p, (void **)&frm_v, "FRM", NULL, y_sz + uv_sz);
        if (ret == 0) {
            /* Fill frame with gray 128 */
            memset((void *)(uintptr_t)frm_v, 128, y_sz);
            memset((void *)(uintptr_t)frm_v + y_sz, 128, uv_sz);
            HI_MPI_SYS_MmzFlushCache(frm_p, (void *)(uintptr_t)frm_v, y_sz + uv_sz);

            /* ASAN-style: fill tmp_buf and out_buf with 0xAA pattern.
             * After forward, anything != 0xAA was written by hardware. */
            memset((void *)(uintptr_t)tmp_mem.virt, 0xAA, tmp_size);
            HI_MPI_SYS_MmzFlushCache(tmp_mem.phys, (void *)(uintptr_t)tmp_mem.virt, tmp_size);

            /* Allocate output buffer in MMZ for detection results */
            uint64_t out_p, out_v;
            uint32_t out_size = 65536;
            ret = HI_MPI_SYS_MmzAlloc(&out_p, (void **)&out_v, "OUT", NULL, out_size);
            if (ret != 0) out_p = out_v = 0;
            else memset((void *)(uintptr_t)out_v, 0, out_size);

            /* Build forward_slice ioctl buffer (2416 bytes)
             *
             * Layout from kernel RE (ive_xnn_forward_data_slice):
             *   +0x000: handle (out)
             *   +0x008: src_blobs[16] (48B each) — input
             *   +0x308: roi_info
             *   +0x338: model_id
             *   +0x340: dst_blobs_template[16] (48B each) — kernel copies to internal
             *   +0x640: dst_blobs_out[16] (48B each) — output destination
             *   +0x940: u64 tmp_phys
             *   +0x948: u64 tmp_virt (used for report/output virt addr base)
             *   +0x950: u32 tmp_size
             *   +0x958: ctrl {u32 src_num, u32 dst_num}
             *   +0x960: u32 has_roi (0=picture)
             *   +0x968: u32 is_instant (1=sync)
             */
            uint8_t fwd[2416];
            memset(fwd, 0, sizeof(fwd));

            /* src_blob[0] at +0x008 (IVE_BLOB_S, 48 bytes) */
            *(uint32_t *)(fwd + 0x008) = 2;         /* enType = YVU420SP */
            *(uint32_t *)(fwd + 0x00C) = stride;    /* u32Stride */
            *(uint64_t *)(fwd + 0x010) = frm_v;     /* u64VirAddr */
            *(uint64_t *)(fwd + 0x018) = frm_p;     /* u64PhyAddr */
            *(uint32_t *)(fwd + 0x020) = 1;         /* u32Num */
            *(uint32_t *)(fwd + 0x028) = in_w;      /* u32Width */
            *(uint32_t *)(fwd + 0x02C) = in_h;      /* u32Height */
            *(uint32_t *)(fwd + 0x030) = in_c;      /* u32Chn */

            /* model_id at +0x338 */
            *(uint32_t *)(fwd + 0x338) = 0;

            /* dst_blobs_template[0] at +0x340 — kernel uses this for CPU preproc path.
             * The kernel validates stride against model's dst_node dimensions.
             * stride must be >= max(align16(output_w * type_size), 48). */
            uint32_t out_stride = ((10 * 4 + 15) / 16) * 16;  /* 48 for 10 values */
            if (out_stride < 48) out_stride = 48;
            *(uint32_t *)(fwd + 0x340) = 7;         /* enType = S8 */
            *(uint32_t *)(fwd + 0x344) = out_stride; /* u32Stride */
            *(uint64_t *)(fwd + 0x348) = out_v;     /* u64VirAddr */
            *(uint64_t *)(fwd + 0x350) = out_p;     /* u64PhyAddr */
            *(uint32_t *)(fwd + 0x358) = 1;         /* u32Num */
            *(uint32_t *)(fwd + 0x360) = 10;        /* u32Width */
            *(uint32_t *)(fwd + 0x364) = 1;         /* u32Height */
            *(uint32_t *)(fwd + 0x368) = 1;         /* u32Chn */

            /* dst_blobs_out[0] at +0x640 — same as template */
            memcpy(fwd + 0x640, fwd + 0x340, 48);

            /* tmp_buf at +0x940 */
            *(uint64_t *)(fwd + 0x940) = tmp_mem.phys;
            *(uint64_t *)(fwd + 0x948) = tmp_mem.virt;
            *(uint32_t *)(fwd + 0x950) = tmp_size;

            /* ctrl starts at +0x958. From kernel: a5[1]=src_num, a5[3]=dst_num
             * a5[0] = +0x958, a5[1] = +0x95C, a5[2] = +0x960, a5[3] = +0x964 */
            *(uint32_t *)(fwd + 0x95C) = 1;  /* a5[1] = src_num */
            *(uint32_t *)(fwd + 0x964) = 1;  /* a5[3] = dst_num */

            /* has_roi: from decompiled, checked at a1[600] = buf+0x960
             * But a1[600] = 600*4 = 2400 = 0x960. Overlaps with ctrl!
             * Actually, the ctrl area and has_roi/instant are separate:
             * a1[600] = 0x960 (has_roi) is a5[2] in the ctrl view */
            *(uint32_t *)(fwd + 0x960) = 0;  /* a5[2] / has_roi = 0 */

            /* is_instant: a1[602] = 602*4 = 2408 = 0x968 */
            *(uint32_t *)(fwd + 0x968) = 1;  /* synchronous */

            {
                fprintf(stderr, "\nRunning forward (%ux%u)...\n", in_w, in_h);
                /* Verify buffer before ioctl */
                fprintf(stderr, "  fwd+0x958: src=%u dst=%u\n",
                        *(uint32_t *)(fwd + 0x958), *(uint32_t *)(fwd + 0x95C));
                fprintf(stderr, "  fwd+0x960: roi=%u instant=%u\n",
                        *(uint32_t *)(fwd + 0x960), *(uint32_t *)(fwd + 0x968));
                fprintf(stderr, "  fwd+0x340 dst_tpl: type=%u stride=%u\n",
                        *(uint32_t *)(fwd + 0x340), *(uint32_t *)(fwd + 0x344));
                /* Build args for mpi_ive_xnn_forward_slice (8-arg function).
                 * It handles ioctl + task completion + ref_cnt cleanup. */
                int32_t fwd_handle = 0;

                /* src blob (48 bytes on stack) */
                uint8_t src_blob[48];
                memcpy(src_blob, fwd + 0x008, 48);

                /* dst blob template */
                uint8_t dst_blob[48];
                memcpy(dst_blob, fwd + 0x340, 48);

                /* ctrl: {padding, src_num, has_roi, dst_num, ...} from +0x958 */
                uint32_t ctrl[4];
                ctrl[0] = 0;  /* a5[0] */
                ctrl[1] = 1;  /* a5[1] = src_num */
                ctrl[2] = 0;  /* a5[2] = has_roi */
                ctrl[3] = 1;  /* a5[3] = dst_num */

                /* tmp_buf descriptor (24 bytes) */
                xnn_mem_t fwd_tmp = tmp_mem;

                ret = mpi_ive_xnn_forward_slice(
                    &fwd_handle,    /* r0: handle out */
                    src_blob,       /* r1: src blobs */
                    dst_blob,       /* r2: dst blobs */
                    model_params,   /* r3: model params from loadmodel */
                    &fwd_tmp,       /* [sp+0]: tmp buffer */
                    dst_blob,       /* [sp+4]: report/output blob */
                    ctrl,           /* [sp+8]: ctrl */
                    1);             /* [sp+12]: instant */
                fprintf(stderr, "  forward ret=0x%x handle=%d\n", ret, fwd_handle);
                /* Wait for completion — same pattern as svp_alg_forward_slice:
                 * retry HI_MPI_IVE_Query with usleep(100) until not TIMEOUT */
                if (ret == 0) {
                    int finished = 0;
                    int qret = HI_MPI_IVE_Query(fwd_handle, &finished, 1);
                    int retries = 0;
                    while (qret == (int)0xa01d8041 && retries < 1000) {
                        usleep(100);
                        qret = HI_MPI_IVE_Query(fwd_handle, &finished, 1);
                        retries++;
                    }
                    fprintf(stderr, "  query ret=0x%x finished=%d retries=%d\n",
                            qret, finished, retries);
                }

                /* Check tmp_buf for output */
                HI_MPI_SYS_MmzFlushCache(tmp_mem.phys, (void *)(uintptr_t)tmp_mem.virt, 1024);
                int8_t *out = (int8_t *)(uintptr_t)tmp_mem.virt;
                int nz = 0;
                for (int i = 0; i < 512; i++) if (out[i] != 0) nz++;
                fprintf(stderr, "  tmp_buf nonzero: %d/512\n", nz);
                fprintf(stderr, "  first 32 bytes: ");
                for (int i = 0; i < 32; i++) fprintf(stderr, "%02x ", (uint8_t)out[i]);
                fprintf(stderr, "\n");

                /* Scan full tmp_buf for output regions */
                HI_MPI_SYS_MmzFlushCache(tmp_mem.phys, (void *)(uintptr_t)tmp_mem.virt, tmp_size);
                int nz2 = 0;
                for (uint32_t i = 0; i < tmp_size; i++) if (out[i] != 0) nz2++;
                fprintf(stderr, "  tmp_buf total nonzero: %d/%u\n", nz2, tmp_size);

                /* Find non-zero regions */
                int rgn_start = -1;
                for (uint32_t i = 0; i < tmp_size; i += 64) {
                    uint32_t end = i + 64 < tmp_size ? i + 64 : tmp_size;
                    int nz3 = 0;
                    for (uint32_t j = i; j < end; j++) if (out[j]) nz3++;
                    if (nz3 > 0 && rgn_start < 0) rgn_start = i;
                    else if (nz3 == 0 && rgn_start >= 0) {
                        fprintf(stderr, "  region: 0x%04x-0x%04x (%d bytes)\n",
                                rgn_start, i, i - rgn_start);
                        /* Dump first 32 bytes of each region */
                        fprintf(stderr, "    data: ");
                        for (int j = 0; j < 32 && rgn_start + j < (int)tmp_size; j++)
                            fprintf(stderr, "%02x ", (uint8_t)out[rgn_start + j]);
                        fprintf(stderr, "\n");
                        rgn_start = -1;
                    }
                }
                if (rgn_start >= 0)
                    fprintf(stderr, "  region: 0x%04x-0x%04x (%d bytes)\n",
                            rgn_start, tmp_size, tmp_size - rgn_start);

                /* Scan for bytes != 0xAA (written by hardware) */
                fprintf(stderr, "  HW-written regions (not 0xAA):\n");
                int rgn_s = -1;
                for (uint32_t i = 0; i < tmp_size; i += 16) {
                    int changed = 0;
                    for (int j = 0; j < 16 && i + j < tmp_size; j++)
                        if ((uint8_t)out[i + j] != 0xAA) changed++;
                    if (changed > 0 && rgn_s < 0) rgn_s = i;
                    else if (changed == 0 && rgn_s >= 0) {
                        fprintf(stderr, "    +0x%04x - +0x%04x (%d bytes)\n",
                                rgn_s, (int)i, (int)i - rgn_s);
                        /* Dump first and last 16 bytes of each region */
                        fprintf(stderr, "      first: ");
                        for (int j = 0; j < 16; j++) fprintf(stderr, "%02x ", (uint8_t)out[rgn_s + j]);
                        fprintf(stderr, "\n");
                        if ((int)i - rgn_s > 16) {
                            fprintf(stderr, "      last:  ");
                            for (int j = 0; j < 16; j++) fprintf(stderr, "%02x ", (uint8_t)out[i - 16 + j]);
                            fprintf(stderr, "\n");
                        }
                        rgn_s = -1;
                    }
                }
                if (rgn_s >= 0)
                    fprintf(stderr, "    +0x%04x - +0x%04x (%d bytes)\n",
                            rgn_s, tmp_size, tmp_size - rgn_s);

                /* Also dump the output buffer we allocated */

                /* Also check our allocated output buffer */
                HI_MPI_SYS_MmzFlushCache(out_p, (void *)(uintptr_t)out_v, 256);
                int8_t *outbuf = (int8_t *)(uintptr_t)out_v;
                int out_nz = 0;
                for (int i = 0; i < 256; i++) if (outbuf[i]) out_nz++;
                fprintf(stderr, "  out_buf nonzero: %d/256\n", out_nz);
                if (out_nz) {
                    fprintf(stderr, "  out_buf first 32: ");
                    for (int i = 0; i < 32; i++) fprintf(stderr, "%02x ", (uint8_t)outbuf[i]);
                    fprintf(stderr, "\n");
                    /* Print as signed int8 for FC output interpretation */
                    fprintf(stderr, "  out_buf as int8:  ");
                    for (int i = 0; i < 16; i++) fprintf(stderr, "%4d ", outbuf[i]);
                    fprintf(stderr, "\n");
                }
            }

            if (out_p) HI_MPI_SYS_MmzFree(out_p, (void *)(uintptr_t)out_v);
            HI_MPI_SYS_MmzFree(frm_p, (void *)(uintptr_t)frm_v);
        }

        /* Unload model — may need to call twice if forward incremented ref_cnt */
        mpi_ive_xnn_unloadmodel(model_params);
        mpi_ive_xnn_unloadmodel(model_params); /* second call for ref_cnt=2 */
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
