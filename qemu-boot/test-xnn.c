/*
 * Direct IVE XNN Test — Intercept XNN ioctl calls via LD_PRELOAD hook
 *
 * Instead of reconstructing all struct layouts from scratch, we hook ioctl()
 * to capture the exact bytes that hi_ivp_process_ex() passes to the XNN
 * engine. This gives us the complete blob layout for free.
 *
 * Approach:
 * 1. Run the IVP pipeline (test-ivp) normally
 * 2. But with this LD_PRELOAD shim that intercepts ioctl calls
 * 3. When it sees XNN ioctl commands (0x463a forward, 0x4636 load), dump the buffer
 *
 * Build:
 *   CC=~/git/firmware/output-hi3516ev300_lite/host/bin/arm-openipc-linux-musleabi-gcc
 *   SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
 *   $CC -shared -fPIC -o xnn-hook.so test-xnn.c -I$SDK/include -ldl
 *
 * Run:
 *   LD_PRELOAD=/utils/xnn-hook.so /utils/test-ivp /utils/model.oms /utils/input.y4m
 *
 * Or: standalone test that calls XNN directly using captured blob data.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdint.h>
#include <dlfcn.h>
#include <sys/syscall.h>

/* mmap shim for uclibc-compiled vendor libs */
#define PROT_READ   0x1
#define PROT_WRITE  0x2
#define MAP_SHARED  0x01
#define MAP_PRIVATE 0x02
#define MAP_FAILED  ((void *)-1)
int munmap(void *addr, size_t len);

void *mmap(void *start, size_t len, int prot, int flags, int fd, uint32_t off) {
    return (void *)syscall(SYS_mmap2, start, len, prot, flags, fd, off >> 12);
}

/* XNN ioctl command numbers (from RE) */
#define XNN_IOC_LOADMODEL  0xc8a04636
#define XNN_IOC_FORWARD    0xc620463a
#define XNN_IOC_SVP_INIT   0x8010463b

/* Known IVE ioctl magic: 'F' = 0x46, commands 0x36-0x3b are XNN range */
#define IOC_TYPE_MASK   0x0000ff00
#define IOC_NR_MASK     0x000000ff
#define IOC_TYPE(cmd)   (((cmd) >> 8) & 0xff)
#define IOC_NR(cmd)     ((cmd) & 0xff)
#define IOC_SIZE(cmd)   (((cmd) >> 16) & 0x3fff)

static void hexdump(const char *label, const void *data, int len) {
    const uint8_t *p = data;
    fprintf(stderr, "--- %s (%d bytes) ---\n", label, len);
    for (int i = 0; i < len && i < 256; i += 16) {
        fprintf(stderr, "  %04x:", i);
        for (int j = 0; j < 16 && i + j < len; j++)
            fprintf(stderr, " %02x", p[i + j]);
        fprintf(stderr, "  ");
        for (int j = 0; j < 16 && i + j < len; j++)
            fprintf(stderr, "%c", (p[i+j] >= 32 && p[i+j] < 127) ? p[i+j] : '.');
        fprintf(stderr, "\n");
    }
    if (len > 256)
        fprintf(stderr, "  ... (%d more bytes)\n", len - 256);
}

static void dump_xnn_loadmodel(const void *buf) {
    const uint8_t *p = buf;
    uint64_t model_phys = *(uint64_t *)(p + 0);
    uint64_t model_virt = *(uint64_t *)(p + 8);
    uint32_t model_size = *(uint32_t *)(p + 16);
    uint64_t tmp_phys   = *(uint64_t *)(p + 24);
    uint64_t tmp_virt   = *(uint64_t *)(p + 32);
    uint32_t tmp_size   = *(uint32_t *)(p + 40);

    fprintf(stderr, "[XNN] LOADMODEL:\n");
    fprintf(stderr, "  model: phys=0x%llx virt=0x%llx size=%u\n",
            (unsigned long long)model_phys, (unsigned long long)model_virt, model_size);
    fprintf(stderr, "  tmp:   phys=0x%llx virt=0x%llx size=%u\n",
            (unsigned long long)tmp_phys, (unsigned long long)tmp_virt, tmp_size);
    hexdump("model_params (first 256 bytes)", p + 40, 256);
}

static void dump_xnn_forward(const void *buf, int size) {
    const uint8_t *p = buf;

    /* ctrl at offset 0x618 (from RE): src_num, dst_num */
    uint32_t src_num = *(uint32_t *)(p + 0x610);
    uint32_t dst_num = *(uint32_t *)(p + 0x614);
    uint32_t instant = *(uint32_t *)(p + 0x620);

    fprintf(stderr, "[XNN] FORWARD: src_num=%u dst_num=%u instant=%u\n",
            src_num, dst_num, instant);

    /* src blobs at offset 0x10, 48 bytes each */
    for (uint32_t i = 0; i < src_num && i < 4; i++) {
        const uint8_t *blob = p + 0x08 + i * 48;
        fprintf(stderr, "  src[%u]:\n", i);
        hexdump("  blob", blob, 48);
    }

    /* dst blobs at offset 0x318 */
    for (uint32_t i = 0; i < dst_num && i < 4; i++) {
        const uint8_t *blob = p + 0x318 + i * 48;
        fprintf(stderr, "  dst[%u]:\n", i);
        hexdump("  blob", blob, 48);
    }

    /* model handle at offset 0x310 */
    uint32_t model_handle = *(uint32_t *)(p + 0x310);
    fprintf(stderr, "  model_handle=0x%x\n", model_handle);
}

/* Hooked ioctl — intercepts XNN commands, passes everything else through */
typedef int (*real_ioctl_t)(int fd, unsigned long request, ...);

int ioctl(int fd, unsigned long request, ...) {
    va_list ap;
    va_start(ap, request);
    void *arg = va_arg(ap, void *);
    va_end(ap);

    /* Get real ioctl */
    static real_ioctl_t real_ioctl = NULL;
    if (!real_ioctl) {
        real_ioctl = (real_ioctl_t)dlsym(RTLD_NEXT, "ioctl");
        if (!real_ioctl) {
            fprintf(stderr, "[XNN-HOOK] dlsym(ioctl) failed!\n");
            return -1;
        }
    }

    /* Check if this is an IVE XNN ioctl */
    uint32_t cmd = (uint32_t)request;
    if (IOC_TYPE(cmd) == 0x46) { /* 'F' = IVE */
        uint32_t nr = IOC_NR(cmd);
        uint32_t sz = IOC_SIZE(cmd);

        if (nr == 0x36) { /* loadmodel */
            fprintf(stderr, "\n[XNN-HOOK] ioctl LOADMODEL (cmd=0x%x, size=%u)\n", cmd, sz);
            dump_xnn_loadmodel(arg);
            int r = real_ioctl(fd, request, arg);
            fprintf(stderr, "[XNN-HOOK] LOADMODEL returned %d\n", r);
            hexdump("model_params AFTER kernel fill", (uint8_t *)arg + 40, 128);
            return r;
        } else if (nr == 0x38) { /* forward_slice — the one actually used */
            static int fwd_count = 0;
            if (fwd_count < 2) {
                fprintf(stderr, "\n[XNN-HOOK] ioctl FORWARD_SLICE #%d (cmd=0x%x, size=%u)\n",
                        fwd_count, cmd, sz);
                hexdump("forward_slice BEFORE", arg, 128);
                int r = real_ioctl(fd, request, arg);
                fprintf(stderr, "[XNN-HOOK] FORWARD_SLICE returned %d\n", r);
                hexdump("forward_slice AFTER", arg, 128);
                fwd_count++;
                return r;
            }
            fwd_count++;
        } else if (nr == 0x3a) { /* forward */
            fprintf(stderr, "\n[XNN-HOOK] ioctl FORWARD (cmd=0x%x, size=%u)\n", cmd, sz);
            dump_xnn_forward(arg, sz);
        } else if (nr == 0x3b) { /* svp_init */
            fprintf(stderr, "\n[XNN-HOOK] ioctl SVP_INIT (cmd=0x%x, size=%u)\n", cmd, sz);
            hexdump("svp_init BEFORE", arg, 16);
            int r = real_ioctl(fd, request, arg);
            fprintf(stderr, "[XNN-HOOK] SVP_INIT returned %d\n", r);
            hexdump("svp_init AFTER", arg, 16);
            return r;
        } else if (nr == 0xc8) { /* open_dev */
            fprintf(stderr, "\n[XNN-HOOK] ioctl OPEN_DEV (cmd=0x%x, size=%u)\n", cmd, sz);
            hexdump("open BEFORE", arg, sz);
            int r = real_ioctl(fd, request, arg);
            fprintf(stderr, "[XNN-HOOK] OPEN_DEV returned %d\n", r);
            hexdump("open AFTER", arg, sz);
            return r;
        } else if (nr >= 0x30) {
            fprintf(stderr, "\n[XNN-HOOK] ioctl IVE cmd=0x%x nr=0x%x size=%u\n", cmd, nr, sz);
        }
    }

    return real_ioctl(fd, request, arg);
}
