/*
 * IVE Hardware Test — runs as PID 1 in minimal initramfs.
 * Tests DMA, SAD, CCL operations on the QEMU hisi-ive device.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/reboot.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/reboot.h>

#define IVE_SW_FIRE     0x0008
#define IVE_CMD_DONE    0x0018
#define IVE_TOTAL_OPS   0x0044
#define IVE_HW_ID       0x0080
#define IVE_OP_TYPE     0x0104
#define IVE_DIMENSIONS  0x0108
#define IVE_SRC1_ADDR   0x0110
#define IVE_DST1_ADDR   0x0114
#define IVE_SRC2_ADDR   0x0118
#define IVE_STRIDES_0   0x0128
#define IVE_STRIDES_1   0x012C
#define IVE_THRESH_VAL  0x0138
#define IVE_CCL_CFG0    0x0160

#define PAGE_SZ 0x10000

static volatile uint32_t *ive;

static void ive_w(uint32_t off, uint32_t val) { ive[off/4] = val; }
static uint32_t ive_r(uint32_t off) { return ive[off/4]; }
static void ive_fire(void) {
    ive_w(IVE_SW_FIRE, 1);
    for (int i = 0; i < 100000 && !(ive_r(IVE_CMD_DONE) & 1); i++) usleep(1);
}

static uint64_t v2p(void *va) {
    uint64_t val;
    int fd = open("/proc/self/pagemap", O_RDONLY);
    if (fd < 0) return 0;
    if (pread(fd, &val, 8, ((uintptr_t)va / 4096) * 8) != 8) { close(fd); return 0; }
    close(fd);
    return (val & (1ULL<<63)) ? ((val & ((1ULL<<55)-1)) * 4096) : 0;
}

/* Allocate a page, lock it, fault it in, return virt + phys */
static void *alloc_page(uint64_t *phys) {
    void *p = mmap(NULL, PAGE_SZ, PROT_READ|PROT_WRITE,
                   MAP_PRIVATE|MAP_ANONYMOUS|MAP_LOCKED, -1, 0);
    if (p == MAP_FAILED) return NULL;
    memset(p, 0, PAGE_SZ);
    *phys = v2p(p);
    return (*phys) ? p : NULL;
}

int main(void) {
    mkdir("/dev", 0755);
    mkdir("/proc", 0755);
    mount("devtmpfs", "/dev", "devtmpfs", 0, NULL);
    mount("proc", "/proc", "proc", 0, NULL);

    printf("\n========================================\n");
    printf("IVE Hardware Motion Detection Test\n");
    printf("========================================\n");

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); goto done; }
    ive = mmap(NULL, 0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x11320000);
    if (ive == MAP_FAILED) { perror("mmap ive"); goto done; }

    /* Allocate 4 separate page-aligned buffers with known phys addrs */
    uint64_t pa[4];
    void *va[4];
    const char *names[] = {"frame1", "frame2", "sad_out", "blob"};
    for (int i = 0; i < 4; i++) {
        va[i] = alloc_page(&pa[i]);
        if (!va[i]) { printf("  ERROR: alloc_page(%s) failed\n", names[i]); goto done; }
        printf("  %s: phys=0x%08llx\n", names[i], (unsigned long long)pa[i]);
    }

    int fails = 0;

    /* TEST 1: HW_ID */
    {
        uint32_t id = ive_r(IVE_HW_ID);
        int ok = (id == 0x11E1A300);
        printf("  hw_id:  0x%08X  %s\n", id, ok ? "PASS" : "FAIL");
        fails += !ok;
    }

    /* TEST 2: DMA — copy 256 bytes from frame1 to frame2 */
    {
        uint8_t *src = va[0], *dst = va[1];
        for (int i = 0; i < 256; i++) src[i] = i;
        memset(dst, 0, 256);

        ive_w(IVE_OP_TYPE, 0);
        ive_w(IVE_DIMENSIONS, (16 << 16) | 16);
        ive_w(IVE_SRC1_ADDR, pa[0]);
        ive_w(IVE_DST1_ADDR, pa[1]);
        ive_w(IVE_STRIDES_0, (16 << 16) | 16);
        ive_fire();

        int ok = (memcmp(src, dst, 256) == 0);
        printf("  dma:    dst[0..3]=%d,%d,%d,%d  %s\n",
               dst[0], dst[1], dst[2], dst[3], ok ? "PASS" : "FAIL");
        fails += !ok;
    }

    /* TEST 3: SAD — detect motion rectangle */
    {
        #define W 64
        #define H 64
        #define BW (W/4)
        #define BH (H/4)
        uint8_t *f1 = va[0], *f2 = va[1], *out = va[2];
        memset(f1, 0, W*H);
        memset(f2, 0, W*H);
        memset(out, 0x77, BW*BH); /* fill with canary to detect writes */
        /* 24×24 bright rectangle at (20,20) in frame2 */
        for (int y = 20; y < 44; y++)
            for (int x = 20; x < 44; x++)
                f2[y*W + x] = 200;

        ive_w(IVE_OP_TYPE, 1);
        ive_w(IVE_DIMENSIONS, (H << 16) | W);
        ive_w(IVE_SRC1_ADDR, pa[0]);
        ive_w(IVE_SRC2_ADDR, pa[1]);
        ive_w(IVE_DST1_ADDR, pa[2]);
        ive_w(IVE_STRIDES_0, (W << 16) | BW);
        ive_w(IVE_STRIDES_1, (W << 16));
        ive_w(IVE_THRESH_VAL, 0x0000C800);
        ive_fire();

        int motion = 0;
        for (int i = 0; i < BW*BH; i++) if (out[i] == 255) motion++;
        int ok = (motion > 0 && motion < BW*BH);
        printf("  sad:    motion_blocks=%d/%d  %s\n", motion, BW*BH, ok ? "PASS" : "FAIL");
        fails += !ok;
        #undef W
        #undef H
        #undef BW
        #undef BH
    }

    /* TEST 4: CCL — find two connected components */
    {
        #define BW 16
        #define BH 16
        uint8_t *bin = va[2], *blob = va[3];
        memset(bin, 0, BW*BH);
        memset(blob, 0, 3056);
        /* Blob 1: 3×3 at (2,2) */
        for (int y = 2; y <= 4; y++)
            for (int x = 2; x <= 4; x++)
                bin[y*BW + x] = 255;
        /* Blob 2: 4×2 at (10,10) */
        for (int y = 10; y <= 11; y++)
            for (int x = 10; x <= 13; x++)
                bin[y*BW + x] = 255;

        ive_w(IVE_OP_TYPE, 2);
        ive_w(IVE_DIMENSIONS, (BH << 16) | BW);
        ive_w(IVE_SRC1_ADDR, pa[2]);
        ive_w(IVE_DST1_ADDR, pa[3]);
        ive_w(IVE_CCL_CFG0, (4 << 16));
        ive_fire();

        uint8_t nreg = blob[5];
        int ok = (nreg == 2);
        printf("  ccl:    regions=%d  %s\n", nreg, ok ? "PASS" : "FAIL");
        if (nreg > 0 && nreg <= 4) {
            for (int i = 0; i < nreg; i++) {
                uint8_t *r = blob + 8 + i*12;
                uint32_t a = r[0]|(r[1]<<8)|(r[2]<<16)|(r[3]<<24);
                printf("          r%d: [%d,%d]-[%d,%d] area=%d\n", i,
                       r[4]|(r[5]<<8), r[6]|(r[7]<<8),
                       r[8]|(r[9]<<8), r[10]|(r[11]<<8), a);
            }
        }
        fails += !ok;
        #undef BW
        #undef BH
    }

    printf("========================================\n");
    printf("Result: %d/4 passed\n", 4 - fails);
    printf("========================================\n");

done:
    sync();
    reboot(LINUX_REBOOT_CMD_POWER_OFF);
    for (;;) sleep(1);
}
