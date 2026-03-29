/*
 * Direct IVE XNN CNN Inference via raw ioctl replay
 *
 * Uses a captured forward_slice ioctl buffer (fwd_slice.bin) from a working
 * IVP inference as a template. Patches only the src_blob addresses per frame,
 * issues the raw ioctl to /dev/ive, and reads the detection output.
 *
 * This bypasses libivp.a AND the mpi_ive_xnn_* functions entirely — just
 * raw ioctl to the kernel driver.
 *
 * Requires:
 *   1. fwd_slice.bin (2416 bytes, captured from working IVP via xnn-hook.so)
 *   2. OMS model loaded via mpi_ive_xnn_loadmodel (still uses libive.so for this)
 *   3. Y4M grayscale input video
 *
 * Build (from qemu-boot/):
 *   CC=~/git/firmware/output-hi3516ev300_lite/host/bin/arm-openipc-linux-musleabi-gcc
 *   SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
 *   LIBS=~/git/firmware/output-hi3516ev300_lite/target/usr/lib
 *   $CC -O2 -o test-xnn-direct test-xnn-direct.c \
 *       -I$SDK/include -L$LIBS -lmpi -live \
 *       -lVoiceEngine -lupvqe -ldnvqe -lsecurec \
 *       -Wl,-rpath,/usr/lib -Wl,--allow-shlib-undefined -lm
 *
 * Run: killall majestic; ./test-xnn-direct model.oms fwd_slice.bin input.y4m
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/syscall.h>
extern int ioctl(int fd, unsigned long request, ...);

/* mmap shim — uclibc ABI compat */
#define PROT_READ   0x1
#define PROT_WRITE  0x2
#define MAP_SHARED  0x01
#define MAP_PRIVATE 0x02
#define MAP_FAILED  ((void *)-1)
int munmap(void *addr, size_t len);
void *mmap(void *start, size_t len, int prot, int flags, int fd, uint32_t off) {
    return (void *)syscall(SYS_mmap2, start, len, prot, flags, fd, off >> 12);
}

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
#include "mpi_sys.h"

/* ioctl commands */
#define IVE_IOC_FORWARD_SLICE  0xc9704638

/* Forward_slice buffer offsets (from kernel RE + captured buffer) */
#define FWD_SRC_BLOB    0x008   /* IVE_BLOB_S src_blobs[0] */
#define FWD_MODEL_ID    0x338
#define FWD_DST_TPL     0x340   /* dst_blobs_template[0] */
#define FWD_DST_OUT     0x640   /* dst_blobs_out[0] — output tensor */
#define FWD_TMP_PHYS    0x940
#define FWD_TMP_SIZE    0x950
#define FWD_CTRL        0x958   /* {src_num, dst_num} */
#define FWD_HAS_ROI     0x960
#define FWD_INSTANT     0x968
#define FWD_BUF_SIZE    2416

/* Blob field offsets within IVE_BLOB_S (48 bytes) */
#define BLOB_TYPE       0x00
#define BLOB_STRIDE     0x04
#define BLOB_VIRADDR    0x08    /* u64 */
#define BLOB_PHYADDR    0x10    /* u64 */
#define BLOB_NUM        0x18
#define BLOB_WIDTH      0x20
#define BLOB_HEIGHT     0x24
#define BLOB_CHN        0x28

/* Private XNN API from libive.so */
extern int mpi_ive_xnn_loadmodel(void *model_mem, void *tmp_mem, void *model_params);
extern int mpi_ive_xnn_forward_slice(void *buf);
extern void mpi_ive_xnn_unloadmodel(void *model_params);
extern int mp_ive_svp_alg_proc_init(void *a, void *b);
extern void mp_ive_svp_alg_proc_exit(void);

static long long usec_now(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000000 + tv.tv_usec;
}

static int read_y4m_frame(FILE *fp, uint8_t *buf, int frame_sz, int is_y4m) {
    if (is_y4m) {
        char tag[8];
        if (fread(tag, 1, 6, fp) != 6) return 0;
        if (memcmp(tag, "FRAME\n", 6) != 0) return 0;
    }
    return fread(buf, 1, frame_sz, fp) == (size_t)frame_sz;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s <model.oms> <fwd_slice.bin> <input.y4m>\n", argv[0]);
        return 1;
    }

    /* Load captured forward_slice template */
    FILE *tf = fopen(argv[2], "rb");
    if (!tf) { perror(argv[2]); return 1; }
    uint8_t tpl[FWD_BUF_SIZE];
    if (fread(tpl, 1, FWD_BUF_SIZE, tf) != FWD_BUF_SIZE) {
        fprintf(stderr, "fwd_slice.bin must be %d bytes\n", FWD_BUF_SIZE);
        return 1;
    }
    fclose(tf);

    fprintf(stderr, "Template loaded: model_id=%u, src=%ux%u, dst=%ux%ux%u\n",
            *(uint32_t *)(tpl + FWD_MODEL_ID),
            *(uint32_t *)(tpl + FWD_SRC_BLOB + BLOB_WIDTH),
            *(uint32_t *)(tpl + FWD_SRC_BLOB + BLOB_HEIGHT),
            *(uint32_t *)(tpl + FWD_DST_OUT + BLOB_WIDTH),
            *(uint32_t *)(tpl + FWD_DST_OUT + BLOB_HEIGHT),
            *(uint32_t *)(tpl + FWD_DST_OUT + BLOB_CHN));

    /* Init MPI */
    HI_S32 ret = HI_MPI_SYS_Init();
    if (ret != HI_SUCCESS) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* SVP init */
    uint8_t svp_a[16] = {0}, svp_b[16] = {0};
    mp_ive_svp_alg_proc_init(svp_a, svp_b);

    /* Load OMS model */
    FILE *mf = fopen(argv[1], "rb");
    if (!mf) { perror(argv[1]); return 1; }
    fseek(mf, 0, SEEK_END);
    long model_sz = ftell(mf);
    fseek(mf, 0, SEEK_SET);

    typedef struct { uint64_t phys, virt; uint32_t size, pad; } xnn_mem_t;

    xnn_mem_t model_mem = {0};
    HI_MPI_SYS_MmzAlloc(&model_mem.phys, (void **)&model_mem.virt, "MODEL", NULL, model_sz);
    model_mem.size = model_sz;
    fread((void *)(uintptr_t)model_mem.virt, 1, model_sz, mf);
    fclose(mf);

    /* Tmp buffer */
    uint32_t tmp_size = *(uint32_t *)(tpl + FWD_TMP_SIZE);
    xnn_mem_t tmp_mem = {0};
    HI_MPI_SYS_MmzAlloc(&tmp_mem.phys, (void **)&tmp_mem.virt, "TMP", NULL, tmp_size);
    tmp_mem.size = tmp_size;

    /* Load backbone (segment 1 at OMS offset 0x50) */
    xnn_mem_t seg1 = {
        .phys = model_mem.phys + 0x50,
        .virt = model_mem.virt + 0x50,
        .size = model_sz - 0x50
    };
    uint8_t model_params[2160];
    memset(model_params, 0, sizeof(model_params));
    ret = mpi_ive_xnn_loadmodel(&seg1, &tmp_mem, model_params);
    if (ret) { fprintf(stderr, "loadmodel failed: 0x%x\n", ret); return 1; }
    fprintf(stderr, "Model loaded: %ux%ux%u\n",
            *(uint32_t *)(model_params + 0x14),
            *(uint32_t *)(model_params + 0x18),
            *(uint32_t *)(model_params + 0x1C));

    /* Parse Y4M */
    FILE *fp = fopen(argv[3], "rb");
    if (!fp) { perror(argv[3]); return 1; }
    int W = 640, H = 360, is_y4m = 0;
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
    uint32_t stride = (W + 15) & ~15;
    uint32_t y_size = stride * H;
    uint32_t uv_size = stride * H / 2;

    /* Frame buffer in MMZ */
    uint64_t frame_phys, frame_virt;
    HI_MPI_SYS_MmzAlloc(&frame_phys, (void **)&frame_virt, "FRAME", NULL, y_size + uv_size);
    memset((void *)(uintptr_t)frame_virt + y_size, 0x80, uv_size);

    /* Output buffer in MMZ — dst_out blob points here via tmp_buf offset */
    /* From template: dst_out.phyAddr = 0x42080000, tmp_phys = 0x42200000
     * offset = 0x42080000 - 0x42200000 would be negative — so dst_out uses
     * a separate allocation, not tmp_buf. Let's allocate our own output. */
    uint32_t out_w = *(uint32_t *)(tpl + FWD_DST_OUT + BLOB_WIDTH);
    uint32_t out_h = *(uint32_t *)(tpl + FWD_DST_OUT + BLOB_HEIGHT);
    uint32_t out_c = *(uint32_t *)(tpl + FWD_DST_OUT + BLOB_CHN);
    uint32_t out_stride = *(uint32_t *)(tpl + FWD_DST_OUT + BLOB_STRIDE);
    uint32_t out_size = out_stride * out_h * out_c;
    if (out_size < 65536) out_size = 65536;

    uint64_t out_phys, out_virt;
    HI_MPI_SYS_MmzAlloc(&out_phys, (void **)&out_virt, "OUTPUT", NULL, out_size);
    memset((void *)(uintptr_t)out_virt, 0, out_size);

    fprintf(stderr, "Output: %ux%ux%u, stride=%u, phys=0x%llx, size=%u\n",
            out_w, out_h, out_c, out_stride,
            (unsigned long long)out_phys, out_size);

    /* Find the internal /dev/ive fd opened by HI_MPI_SYS_Init / libive.
     * g_ive_fd is a file-scope static in libive — can't access directly.
     * Scan /proc/self/fd for the ive device (major 218, minor 17). */
    int ive_fd = -1;
    {
        char path[64], link[64];
        for (int fd = 3; fd < 64; fd++) {
            snprintf(path, sizeof(path), "/proc/self/fd/%d", fd);
            int n = readlink(path, link, sizeof(link) - 1);
            if (n > 0) {
                link[n] = '\0';
                if (strstr(link, "ive")) { ive_fd = fd; break; }
            }
        }
    }
    if (ive_fd < 0) {
        fprintf(stderr, "Could not find /dev/ive fd\n");
        return 1;
    }
    fprintf(stderr, "Found /dev/ive fd=%d\n", ive_fd);

    uint8_t *frame_data = malloc(frame_sz);
    long long t_total = 0;
    int nframes = 0;

    fprintf(stderr, "\n--- Processing %dx%d frames ---\n", W, H);

    while (read_y4m_frame(fp, frame_data, frame_sz, is_y4m) && nframes < 10) {
        /* Copy Y plane to MMZ */
        uint8_t *dst = (uint8_t *)(uintptr_t)frame_virt;
        for (int y = 0; y < H; y++)
            memcpy(dst + y * stride, frame_data + y * W, W);
        HI_MPI_SYS_MmzFlushCache(frame_phys, (void *)(uintptr_t)frame_virt, y_size + uv_size);

        /* Build ioctl buffer from template */
        uint8_t buf[FWD_BUF_SIZE];
        memcpy(buf, tpl, FWD_BUF_SIZE);

        /* Patch src_blob with our frame addresses */
        *(uint64_t *)(buf + FWD_SRC_BLOB + BLOB_VIRADDR) = frame_virt;
        *(uint64_t *)(buf + FWD_SRC_BLOB + BLOB_PHYADDR) = frame_phys;

        /* Patch dst_out blob with our output buffer */
        *(uint64_t *)(buf + FWD_DST_OUT + BLOB_VIRADDR) = out_virt;
        *(uint64_t *)(buf + FWD_DST_OUT + BLOB_PHYADDR) = out_phys;

        /* Patch dst_template too (CPU path copies this to dst_out) */
        *(uint64_t *)(buf + FWD_DST_TPL + BLOB_VIRADDR) = out_virt;
        *(uint64_t *)(buf + FWD_DST_TPL + BLOB_PHYADDR) = out_phys;

        /* Patch tmp_buf with our allocation */
        *(uint64_t *)(buf + FWD_TMP_PHYS) = tmp_mem.phys;
        *(uint64_t *)(buf + FWD_TMP_PHYS + 8) = tmp_mem.virt; /* report addr */

        /* Clear output */
        memset((void *)(uintptr_t)out_virt, 0, out_size);

        long long t0 = usec_now();
        int r = ioctl(ive_fd, IVE_IOC_FORWARD_SLICE, buf);
        long long dt = usec_now() - t0;
        t_total += dt;

        /* Check ioctl return and also handle/status at buf[0] */
        int32_t handle_out = *(int32_t *)buf;
        if (r != 0 || handle_out < 0) {
            fprintf(stderr, "FRAME %d: ioctl ret=%d handle=0x%x\n",
                    nframes, r, handle_out);
            /* Dump beginning of returned buffer */
            fprintf(stderr, "  buf[0..15]: ");
            for (int j = 0; j < 16; j++) fprintf(stderr, "%02x ", buf[j]);
            fprintf(stderr, "\n");
            break;
        }

        /* Check output */
        HI_MPI_SYS_MmzFlushCache(out_phys, (void *)(uintptr_t)out_virt, 256);
        int8_t *out = (int8_t *)(uintptr_t)out_virt;
        int nonzero = 0;
        for (int i = 0; i < 256; i++) if (out[i] != 0) nonzero++;

        if (nframes == 0) {
            fprintf(stderr, "FRAME 0: ioctl OK (%.1f ms), output nonzero=%d/256\n",
                    dt / 1000.0, nonzero);
            fprintf(stderr, "  out_buf first 64 bytes: ");
            for (int i = 0; i < 64; i++) fprintf(stderr, "%02x ", (uint8_t)out[i]);
            fprintf(stderr, "\n");

            /* Scan tmp_buf to find output regions.
             * Kernel writes CNN output at tmp_phys + output_offset_table[dst_idx].
             * Scan for non-zero regions to find the detection output. */
            HI_MPI_SYS_MmzFlushCache(tmp_mem.phys, (void *)(uintptr_t)tmp_mem.virt, tmp_size);
            uint8_t *tmp = (uint8_t *)(uintptr_t)tmp_mem.virt;

            /* Find non-zero regions in tmp_buf */
            int region_start = -1;
            fprintf(stderr, "  tmp_buf scan (%.0f KB):\n", tmp_size / 1024.0);
            for (uint32_t off = 0; off < tmp_size; off += 4096) {
                int nz = 0;
                uint32_t end = off + 4096 < tmp_size ? off + 4096 : tmp_size;
                for (uint32_t j = off; j < end; j++) if (tmp[j] != 0) nz++;
                if (nz > 0 && region_start < 0) {
                    region_start = off;
                } else if (nz == 0 && region_start >= 0) {
                    fprintf(stderr, "    region: 0x%06x - 0x%06x (%d KB)\n",
                            region_start, off, (off - region_start) / 1024);
                    region_start = -1;
                }
            }
            if (region_start >= 0)
                fprintf(stderr, "    region: 0x%06x - 0x%06x (%d KB)\n",
                        region_start, tmp_size, (tmp_size - region_start) / 1024);

            /* The last region should be the detection output.
             * Expected: 40*23*63 = 57960 bytes for a 40x23x63 tensor.
             * Dump the last non-zero region's first bytes. */
            fprintf(stderr, "  tmp_buf first 64: ");
            for (int i = 0; i < 64; i++) fprintf(stderr, "%02x ", tmp[i]);
            fprintf(stderr, "\n");

            /* Save detection output (last 58K of tmp_buf = 40x23x63 tensor).
             * The output_offset is at model_ctx+0x3A0 in kernel — we can't read it.
             * But from the captured dst_out blob: phyAddr=0x42080000, tmp_phys=0x42200000.
             * offset = 0x42080000 - 0x42200000 would be negative — so the original IVP
             * used a DIFFERENT tmp_buf base. The detection output is at the end of
             * the intermediate activations.
             *
             * For a 40x23 grid with stride=160, the output tensor is:
             *   stride * height * channels = 160 * 23 * 63 = 231840 bytes
             * But the total tmp_buf is 2534400. The output offset for this model
             * is stored in model_ctx+0x3A0 (kernel only).
             *
             * Alternative: dump the full tmp_buf to disk for offline analysis. */
            {
                FILE *tf = fopen("/utils/tmp_buf_frame0.bin", "wb");
                if (tf) {
                    fwrite(tmp, 1, tmp_size, tf);
                    fclose(tf);
                    fprintf(stderr, "  Saved tmp_buf (%u bytes) to /utils/tmp_buf_frame0.bin\n", tmp_size);
                }
            }

            /* Also check the dst_out blob that kernel may have updated */
            uint8_t *dst_out_buf = buf + FWD_DST_OUT;
            uint64_t dst_phys = *(uint64_t *)(dst_out_buf + 0x10);
            fprintf(stderr, "  dst_out.phyAddr after ioctl: 0x%llx\n",
                    (unsigned long long)dst_phys);
            /* If kernel updated dst_out.phyAddr to point into tmp_buf,
             * we can compute the output offset */
            if (dst_phys >= tmp_mem.phys && dst_phys < tmp_mem.phys + tmp_size) {
                uint32_t det_off = (uint32_t)(dst_phys - tmp_mem.phys);
                fprintf(stderr, "  detection at tmp_buf+0x%x (%u)\n", det_off, det_off);
                fprintf(stderr, "  first 64 bytes of detection: ");
                for (int i = 0; i < 64; i++)
                    fprintf(stderr, "%02x ", tmp[det_off + i]);
                fprintf(stderr, "\n");
            } else {
                fprintf(stderr, "  dst_out.phyAddr NOT in tmp_buf range\n");
            }

            /* Also check dst_out blob in returned buffer */
            fprintf(stderr, "  dst_out after: type=%u stride=%u w=%u h=%u c=%u phys=0x%llx\n",
                    *(uint32_t *)(buf + FWD_DST_OUT + BLOB_TYPE),
                    *(uint32_t *)(buf + FWD_DST_OUT + BLOB_STRIDE),
                    *(uint32_t *)(buf + FWD_DST_OUT + BLOB_WIDTH),
                    *(uint32_t *)(buf + FWD_DST_OUT + BLOB_HEIGHT),
                    *(uint32_t *)(buf + FWD_DST_OUT + BLOB_CHN),
                    (unsigned long long)*(uint64_t *)(buf + FWD_DST_OUT + BLOB_PHYADDR));
        } else if (nonzero > 0) {
            fprintf(stderr, "FRAME %d: nonzero=%d (%.1f ms)\n", nframes, nonzero, dt / 1000.0);
        }

        nframes++;
    }

    fclose(fp);

    if (nframes > 0) {
        fprintf(stderr, "\n=== Direct XNN Benchmark (%d frames, %dx%d) ===\n", nframes, W, H);
        fprintf(stderr, "  Inference: %lld ms (%.1f ms/frame)\n",
                t_total / 1000, (double)t_total / 1000 / nframes);
        fprintf(stderr, "  Throughput: %.1f fps\n",
                nframes > 0 ? 1000000.0 * nframes / t_total : 0);
    }

    /* Cleanup */
    free(frame_data);
    HI_MPI_SYS_MmzFree(frame_phys, (void *)(uintptr_t)frame_virt);
    HI_MPI_SYS_MmzFree(out_phys, (void *)(uintptr_t)out_virt);
    mpi_ive_xnn_unloadmodel(model_params);
    HI_MPI_SYS_MmzFree(tmp_mem.phys, (void *)(uintptr_t)tmp_mem.virt);
    HI_MPI_SYS_MmzFree(model_mem.phys, (void *)(uintptr_t)model_mem.virt);
    mp_ive_svp_alg_proc_exit();
    HI_MPI_SYS_Exit();
    return 0;
}
