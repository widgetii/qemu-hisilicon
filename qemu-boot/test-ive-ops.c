/*
 * IVE Extended Operations Test Suite
 *
 * Tests all implemented IVE operations (Phase 1-4) via register
 * programming. Runs as PID 1 in QEMU initramfs or standalone
 * with --sw flag on real boards.
 *
 * Build: arm-openipc-linux-musleabi-gcc -static -O2 -o test-ive-ops test-ive-ops.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/reboot.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/reboot.h>

/* IVE registers */
#define IVE_SW_FIRE     0x0008
#define IVE_CMD_DONE    0x0018
#define IVE_BLEND_WTS   0x005C
#define IVE_HW_ID       0x0080
#define IVE_OP_TYPE     0x0104
#define IVE_DIMENSIONS  0x0108
#define IVE_SRC1_ADDR   0x0110
#define IVE_DST1_ADDR   0x0114
#define IVE_SRC2_ADDR   0x0118
#define IVE_STRIDES_0   0x0128
#define IVE_STRIDES_1   0x012C
#define IVE_THRESH_VAL  0x0138
#define IVE_THRESH_HI   0x0140
#define IVE_THRESH_MIN  0x0144
#define IVE_THRESH_MID  0x0148
#define IVE_THRESH_MAX  0x014C
#define IVE_CCL_CFG0    0x0160
#define IVE_EXT_MODE    0x0484

/* Op types (matching hisi-ive.c) */
#define OP_DMA      0
#define OP_SAD      1
#define OP_CCL      2
#define OP_SUB      3
#define OP_ADD      4
#define OP_AND      5
#define OP_OR       6
#define OP_XOR      7
#define OP_THRESH   8
#define OP_HIST     9
#define OP_FILTER   10
#define OP_SOBEL    11
#define OP_DILATE   12
#define OP_ERODE    13
#define OP_INTEG    14
#define OP_MAP      15
#define OP_NCC      16
#define OP_THRESH_S16    18
#define OP_THRESH_U16    19
#define OP_16BIT_TO_8BIT 20

#define W 64
#define H 64
#define SZ (W * H)

static volatile uint32_t *ive;
static int sw_mode;

static void ive_w(uint32_t off, uint32_t val) { if (!sw_mode) ive[off/4] = val; }
static uint32_t ive_r(uint32_t off) { return sw_mode ? 0 : ive[off/4]; }
static void ive_fire(void) {
    if (sw_mode) return;
    ive_w(IVE_SW_FIRE, 1);
    for (int i = 0; i < 1000 && !(ive_r(IVE_CMD_DONE) & 1); i++) usleep(1);
    usleep(100);
}

static uint64_t v2p(void *va) {
    uint64_t val;
    int fd = open("/proc/self/pagemap", O_RDONLY);
    if (fd < 0) return 0;
    if (pread(fd, &val, 8, ((uintptr_t)va / 4096) * 8) != 8) { close(fd); return 0; }
    close(fd);
    return (val & (1ULL<<63)) ? ((val & ((1ULL<<55)-1)) * 4096) : 0;
}

/* Buffers — 3 pages: src1, src2, dst (each 4KB = 64×64 exactly) */
static uint8_t *va_src1, *va_src2, *va_dst;
static uint64_t pa_src1, pa_src2, pa_dst;

static void *alloc_buf(uint64_t *pa) {
    void *p = mmap(NULL, 4096, PROT_READ|PROT_WRITE,
                   MAP_PRIVATE|MAP_ANONYMOUS|MAP_LOCKED, -1, 0);
    if (p == MAP_FAILED) return NULL;
    memset(p, 0, 4096);
    *pa = v2p(p);
    return (*pa) ? p : NULL;
}

static void setup_ive(int op, int w, int h) {
    ive_w(IVE_OP_TYPE, op);
    ive_w(IVE_DIMENSIONS, (h << 16) | w);
    ive_w(IVE_SRC1_ADDR, pa_src1);
    ive_w(IVE_SRC2_ADDR, pa_src2);
    ive_w(IVE_DST1_ADDR, pa_dst);
    ive_w(IVE_STRIDES_0, (w << 16) | w);
    ive_w(IVE_STRIDES_1, (w << 16));
}

/* Compute checksum for comparison */
static uint32_t cksum(const void *data, int len) {
    const uint8_t *p = data;
    uint32_t s = 0;
    for (int i = 0; i < len; i++) s = s * 31 + p[i];
    return s;
}

/* ── Software reference implementations ──────────────────────── */

static void sw_sub(uint8_t *dst, const uint8_t *a, const uint8_t *b, int n) {
    for (int i = 0; i < n; i++) dst[i] = abs((int)a[i] - (int)b[i]);
}

static void sw_add(uint8_t *dst, const uint8_t *a, const uint8_t *b, int n,
                   uint16_t wx, uint16_t wy) {
    for (int i = 0; i < n; i++) {
        int v = ((int)a[i] * wx + (int)b[i] * wy) >> 16;
        dst[i] = (v > 255) ? 255 : v;
    }
}

static void sw_and(uint8_t *dst, const uint8_t *a, const uint8_t *b, int n) {
    for (int i = 0; i < n; i++) dst[i] = a[i] & b[i];
}
static void sw_or(uint8_t *dst, const uint8_t *a, const uint8_t *b, int n) {
    for (int i = 0; i < n; i++) dst[i] = a[i] | b[i];
}
static void sw_xor(uint8_t *dst, const uint8_t *a, const uint8_t *b, int n) {
    for (int i = 0; i < n; i++) dst[i] = a[i] ^ b[i];
}

static void sw_thresh(uint8_t *dst, const uint8_t *src, int n, uint8_t thr) {
    for (int i = 0; i < n; i++) dst[i] = (src[i] > thr) ? 255 : 0;
}

static void sw_hist(uint32_t *hist, const uint8_t *src, int n) {
    memset(hist, 0, 256 * sizeof(uint32_t));
    for (int i = 0; i < n; i++) hist[src[i]]++;
}

static void sw_sobel(uint8_t *dst, const uint8_t *src, int w, int h) {
    memset(dst, 0, w * h);
    for (int y = 1; y < h-1; y++)
        for (int x = 1; x < w-1; x++) {
            int gx = -src[(y-1)*w+x-1] + src[(y-1)*w+x+1]
                     -2*src[y*w+x-1] + 2*src[y*w+x+1]
                     -src[(y+1)*w+x-1] + src[(y+1)*w+x+1];
            int gy = -src[(y-1)*w+x-1] - 2*src[(y-1)*w+x] - src[(y-1)*w+x+1]
                     +src[(y+1)*w+x-1] + 2*src[(y+1)*w+x] + src[(y+1)*w+x+1];
            int mag = (abs(gx) + abs(gy)) >> 1;
            dst[y*w+x] = (mag > 255) ? 255 : mag;
        }
}

static void sw_dilate(uint8_t *dst, const uint8_t *src, int w, int h) {
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint8_t mx = 0;
            for (int ky=-2; ky<=2; ky++) for (int kx=-2; kx<=2; kx++) {
                int sy=y+ky, sx=x+kx;
                if (sy>=0 && sy<h && sx>=0 && sx<w) {
                    uint8_t v=src[sy*w+sx]; if(v>mx) mx=v;
                }
            }
            dst[y*w+x] = mx;
        }
}

static void sw_erode(uint8_t *dst, const uint8_t *src, int w, int h) {
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint8_t mn = 255;
            for (int ky=-2; ky<=2; ky++) for (int kx=-2; kx<=2; kx++) {
                int sy=y+ky, sx=x+kx;
                if (sy>=0 && sy<h && sx>=0 && sx<w) {
                    uint8_t v=src[sy*w+sx]; if(v<mn) mn=v;
                }
            }
            dst[y*w+x] = mn;
        }
}

static void sw_integ(uint32_t *dst, const uint8_t *src, int w, int h) {
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint32_t v = src[y*w+x];
            if (x > 0) v += dst[y*w+x-1];
            if (y > 0) v += dst[(y-1)*w+x];
            if (x > 0 && y > 0) v -= dst[(y-1)*w+x-1];
            dst[y*w+x] = v;
        }
}

/* ── Test runner ─────────────────────────────────────────────── */

static int run_test(const char *name, int op, void (*sw_fn)(void)) {
    /* Clear dst */
    memset(va_dst, 0, 4096);

    if (sw_mode) {
        sw_fn();
    } else {
        setup_ive(op, W, H);
        ive_fire();
    }

    uint32_t cs = cksum(va_dst, SZ);
    printf("  %-10s cksum=0x%08X", name, cs);

    /* Spot-check first few bytes */
    printf("  [%d,%d,%d,%d]", va_dst[0], va_dst[1], va_dst[2], va_dst[3]);

    int ok = 0;
    for (int i = 0; i < SZ; i++) if (va_dst[i] != 0) { ok = 1; break; }
    printf("  %s\n", ok ? "PASS" : "FAIL(all-zero)");
    return !ok;
}

int main(int argc, char **argv) {
    int is_init = (getpid() == 1);
    for (int a = 1; a < argc; a++)
        if (!strcmp(argv[a], "--sw")) sw_mode = 1;

    if (is_init) {
        mkdir("/dev", 0755);
        mkdir("/proc", 0755);
        mount("devtmpfs", "/dev", "devtmpfs", 0, NULL);
        mount("proc", "/proc", "proc", 0, NULL);
    }

    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd >= 0 && !sw_mode) {
        ive = mmap(NULL, 0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0x11320000);
        if (ive == MAP_FAILED) sw_mode = 1;
    } else {
        sw_mode = 1;
    }

    va_src1 = alloc_buf(&pa_src1);
    va_src2 = alloc_buf(&pa_src2);
    va_dst  = alloc_buf(&pa_dst);
    if (!va_src1 || !va_src2 || !va_dst) {
        printf("ERROR: buffer allocation failed\n");
        goto done;
    }

    /* Fill test patterns */
    for (int i = 0; i < SZ; i++) {
        va_src1[i] = (i * 7 + 13) & 0xFF;       /* pseudo-random pattern A */
        va_src2[i] = (i * 11 + 31) & 0xFF;       /* pseudo-random pattern B */
    }

    printf("\n========================================\n");
    printf("IVE Extended Operations Test (%s)\n", sw_mode ? "SW" : "HW");
    printf("========================================\n");

    int fails = 0;

    /* Phase 1: Pixel-wise */
    fails += run_test("sub", OP_SUB, ({void f(void){sw_sub(va_dst,va_src1,va_src2,SZ);} f;}));
    fails += run_test("add", OP_ADD, ({void f(void){
        ive_w(IVE_BLEND_WTS, (32768) | (32768 << 16));
        sw_add(va_dst,va_src1,va_src2,SZ,32768,32768);} f;}));
    fails += run_test("and", OP_AND, ({void f(void){sw_and(va_dst,va_src1,va_src2,SZ);} f;}));
    fails += run_test("or",  OP_OR,  ({void f(void){sw_or(va_dst,va_src1,va_src2,SZ);} f;}));
    fails += run_test("xor", OP_XOR, ({void f(void){sw_xor(va_dst,va_src1,va_src2,SZ);} f;}));

    /* Phase 2: Thresh + Hist */
    /* Thresh: setup registers before fire */
    ive_w(IVE_THRESH_VAL, 0x00008000); /* thr=128 in bits[15:8] */
    ive_w(IVE_THRESH_MAX, 255);
    fails += run_test("thresh", OP_THRESH, ({void f(void){
        sw_thresh(va_dst,va_src1,SZ,128);} f;}));

    /* Hist — output is 256×u32 = 1024 bytes */
    memset(va_dst, 0, 4096);
    if (sw_mode) {
        uint32_t hist[256];
        sw_hist(hist, va_src1, SZ);
        memcpy(va_dst, hist, sizeof(hist));
    } else {
        setup_ive(OP_HIST, W, H);
        ive_fire();
    }
    {
        uint32_t *h = (uint32_t *)va_dst;
        uint32_t total = 0;
        for (int i = 0; i < 256; i++) total += h[i];
        int ok = (total == SZ);
        printf("  %-10s total=%d (expect %d)  %s\n", "hist", total, SZ, ok ? "PASS" : "FAIL");
        fails += !ok;
    }

    /* Phase 3: Sobel, Dilate, Erode */
    fails += run_test("sobel", OP_SOBEL,
        ({void f(void){sw_sobel(va_dst,va_src1,W,H);} f;}));
    fails += run_test("dilate", OP_DILATE,
        ({void f(void){sw_dilate(va_dst,va_src1,W,H);} f;}));
    fails += run_test("erode", OP_ERODE,
        ({void f(void){sw_erode(va_dst,va_src1,W,H);} f;}));

    /* Phase 4: Integ — use 32×32 to fit U32 output in one page (32×32×4=4096) */
    {
        int IW = 32, IH = 32, ISZ = 32 * 32;
        memset(va_dst, 0, 4096);
        if (sw_mode) {
            sw_integ((uint32_t *)va_dst, va_src1, IW, IH);
        } else {
            setup_ive(OP_INTEG, IW, IH);
            ive_fire();
        }
        uint32_t *ig = (uint32_t *)va_dst;
        uint32_t expected_sum = 0;
        for (int i = 0; i < ISZ; i++) expected_sum += va_src1[i];
        int ok = (ig[ISZ-1] == expected_sum);
        printf("  %-10s sum=%u (expect %u)  %s\n", "integ", ig[ISZ-1], expected_sum,
               ok ? "PASS" : "FAIL");
        fails += !ok;
    }

    /* ── 16-bit ops: Thresh_S16, Thresh_U16, 16BitTo8Bit ── */
    /* Use half-height (64×32 = 2048 pixels) so S16 data fits in one page.
     * Reset buffers after integ test (which overflows va_dst with U32 data). */
    memset(va_src1, 0, 4096);
    memset(va_dst, 0, 4096);
    /* Reset strides to default (W bytes) */
    if (!sw_mode) ive_w(IVE_STRIDES_0, (W << 16) | W);
    printf("--- 16-bit ops ---\n");
    {
        int W16 = W, H16 = H / 2, SZ16 = W * (H / 2); /* 64×32 = 2048 */
        int16_t *s16_src = (int16_t *)va_src1; /* 2048 * 2 = 4096 bytes = 1 page */

        /* Write S16 test data: values -200, -100, -50, 0, 50, 100, 150, 200 repeating */
        int16_t test_vals[] = {-200, -100, -50, 0, 50, 100, 150, 200};
        for (int i = 0; i < SZ16; i++) s16_src[i] = test_vals[i % 8];

        /* Thresh_S16: mode=2 (S16→U8 min/mid/max), lo=-50, hi=100 */
        if (sw_mode) {
            for (int i = 0; i < SZ16; i++) {
                int16_t v = s16_src[i];
                va_dst[i] = (v < -50) ? 0 : (v > 100) ? 255 : 128;
            }
        } else {
            ive_w(IVE_EXT_MODE, 2);
            ive_w(IVE_THRESH_VAL, (uint16_t)(-50));
            ive_w(IVE_THRESH_HI, 100);
            ive_w(IVE_THRESH_MIN, 0);
            ive_w(IVE_THRESH_MID, 128);
            ive_w(IVE_THRESH_MAX, 255);
            ive_w(IVE_STRIDES_0, ((W16*2) << 16) | W16);
            setup_ive(OP_THRESH_S16, W16, H16);
            ive_fire();
        }
        /* Boundary: -50 <= lo(-50) → min=0; 100 NOT > hi(100) → mid=128; 150 > hi(100) → max=255 */
        int ok = (va_dst[0] == 0 && va_dst[1] == 0 && va_dst[2] == 0 &&
                  va_dst[3] == 128 && va_dst[4] == 128 && va_dst[5] == 128 &&
                  va_dst[6] == 255 && va_dst[7] == 255);
        printf("  %-10s [%d,%d,%d,%d,%d,%d,%d,%d]  %s\n", "thresh_s16",
               va_dst[0], va_dst[1], va_dst[2], va_dst[3],
               va_dst[4], va_dst[5], va_dst[6], va_dst[7],
               ok ? "PASS" : "FAIL");
        fails += !ok;

        /* Thresh_U16: mode=0 (min/mid/max), lo=50, hi=150 */
        uint16_t *u16_src = (uint16_t *)va_src1;
        for (int i = 0; i < SZ16; i++) u16_src[i] = (i * 7 + 13) % 256;
        if (sw_mode) {
            for (int i = 0; i < SZ16; i++) {
                uint16_t v = u16_src[i];
                va_dst[i] = (v < 50) ? 0 : (v > 150) ? 255 : 128;
            }
        } else {
            ive_w(IVE_EXT_MODE, 0);
            ive_w(IVE_THRESH_VAL, 50);
            ive_w(IVE_THRESH_HI, 150);
            ive_w(IVE_THRESH_MIN, 0);
            ive_w(IVE_THRESH_MID, 128);
            ive_w(IVE_THRESH_MAX, 255);
            ive_w(IVE_STRIDES_0, ((W16*2) << 16) | W16);
            setup_ive(OP_THRESH_U16, W16, H16);
            ive_fire();
        }
        ok = (va_dst[0] == 0 && va_dst[7] == 128);
        printf("  %-10s [%d,%d,%d,%d]  %s\n", "thresh_u16",
               va_dst[0], va_dst[1], va_dst[2], va_dst[7],
               ok ? "PASS" : "FAIL");
        fails += !ok;

        /* 16BitTo8Bit: mode=1 (S16→U8_ABS), numerator=1, denominator=1 */
        for (int i = 0; i < SZ16; i++) s16_src[i] = test_vals[i % 8];
        if (sw_mode) {
            for (int i = 0; i < SZ16; i++) {
                int v = abs((int)s16_src[i]);
                va_dst[i] = v > 255 ? 255 : v;
            }
        } else {
            ive_w(IVE_EXT_MODE, 1);
            ive_w(IVE_THRESH_VAL, (1 << 8) | 1);
            ive_w(IVE_THRESH_MIN, 0);
            ive_w(IVE_STRIDES_0, ((W16*2) << 16) | W16);
            setup_ive(OP_16BIT_TO_8BIT, W16, H16);
            ive_fire();
        }
        ok = (va_dst[0] == 200 && va_dst[1] == 100 && va_dst[2] == 50 &&
              va_dst[3] == 0 && va_dst[4] == 50 && va_dst[7] == 200);
        printf("  %-10s [%d,%d,%d,%d,%d,%d,%d,%d]  %s\n", "16to8",
               va_dst[0], va_dst[1], va_dst[2], va_dst[3],
               va_dst[4], va_dst[5], va_dst[6], va_dst[7],
               ok ? "PASS" : "FAIL");
        fails += !ok;
    }

    printf("========================================\n");
    printf("Result: %d/%d passed\n", 15 - fails, 15);
    printf("========================================\n");

done:
    if (is_init) {
        sync();
        reboot(LINUX_REBOOT_CMD_POWER_OFF);
        for (;;) sleep(1);
    }
    return fails;
}
