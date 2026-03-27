/*
 * IVE Hardware Motion Detection Test
 *
 * Tests the QEMU hisi-ive device by directly programming registers
 * via /dev/mem. Creates two synthetic grayscale frames (one with a
 * bright rectangle), runs SAD + CCL, and verifies the detected region.
 *
 * Usage: test-ive [ive_base_hex]
 *   Default base: 0x11320000 (EV300)
 *
 * Build: arm-openipc-linux-musleabi-gcc -static -O2 -o test-ive test-ive.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>

/* IVE register offsets (from live EV300 capture) */
#define IVE_SW_FIRE     0x0008
#define IVE_CMD_DONE    0x0018
#define IVE_TOTAL_OPS   0x0044
#define IVE_HW_ID       0x0080
#define IVE_OP_TYPE     0x0104
#define IVE_DIMENSIONS  0x0108
#define IVE_BLOCK_CFG   0x010C
#define IVE_SRC1_ADDR   0x0110
#define IVE_DST1_ADDR   0x0114
#define IVE_SRC2_ADDR   0x0118
#define IVE_STRIDES_0   0x0128
#define IVE_STRIDES_1   0x012C
#define IVE_THRESH_VAL  0x0138
#define IVE_CCL_CFG0    0x0160
#define IVE_IRQ_STATUS  0x025C

#define IVE_OP_DMA  0
#define IVE_OP_SAD  1
#define IVE_OP_CCL  2

/* Test image: 64x64 grayscale */
#define IMG_W   64
#define IMG_H   64
#define IMG_SZ  (IMG_W * IMG_H)

/* SAD output: 16x16 (64/4 blocks) */
#define SAD_W   (IMG_W / 4)
#define SAD_H   (IMG_H / 4)
#define SAD_SZ  (SAD_W * SAD_H)

/* CCL blob output (IVE_CCBLOB_S) */
#define BLOB_SZ 3056

/* Physical addresses for test buffers (use high MMZ area) */
#define BUF_BASE    0x47000000
#define BUF_FRAME1  (BUF_BASE + 0x00000)
#define BUF_FRAME2  (BUF_BASE + 0x10000)
#define BUF_SAD_OUT (BUF_BASE + 0x20000)
#define BUF_BLOB    (BUF_BASE + 0x30000)

static volatile uint32_t *ive;
static void *membuf;
static int mem_fd;

static void ive_write(uint32_t offset, uint32_t val) {
    ive[offset / 4] = val;
}

static uint32_t ive_read(uint32_t offset) {
    return ive[offset / 4];
}

static void ive_fire_and_wait(void) {
    ive_write(IVE_SW_FIRE, 1);
    /* Poll for completion */
    int timeout = 100000;
    while (!(ive_read(IVE_CMD_DONE) & 1) && --timeout > 0) {
        usleep(1);
    }
    if (timeout == 0) {
        printf("  TIMEOUT waiting for IVE completion!\n");
    }
}

static void write_phys(uint64_t phys, const void *buf, size_t len) {
    void *p = mmap(NULL, len + (phys & 0xFFF), PROT_WRITE,
                   MAP_SHARED, mem_fd, phys & ~0xFFF);
    if (p == MAP_FAILED) { perror("mmap write"); return; }
    memcpy((char *)p + (phys & 0xFFF), buf, len);
    munmap(p, len + (phys & 0xFFF));
}

static void read_phys(uint64_t phys, void *buf, size_t len) {
    void *p = mmap(NULL, len + (phys & 0xFFF), PROT_READ,
                   MAP_SHARED, mem_fd, phys & ~0xFFF);
    if (p == MAP_FAILED) { perror("mmap read"); return; }
    memcpy(buf, (char *)p + (phys & 0xFFF), len);
    munmap(p, len + (phys & 0xFFF));
}

static int test_hwid(void) {
    uint32_t hwid = ive_read(IVE_HW_ID);
    printf("TEST hw_id: 0x%08X ... ", hwid);
    if (hwid == 0x11E1A300) {
        printf("PASS\n");
        return 0;
    }
    printf("FAIL (expected 0x11E1A300)\n");
    return 1;
}

static int test_sad(void) {
    printf("TEST sad: ");

    /* Create frame1: all zeros (black) */
    uint8_t frame1[IMG_SZ];
    memset(frame1, 0, sizeof(frame1));

    /* Create frame2: bright rectangle at (20,20)-(44,44) */
    uint8_t frame2[IMG_SZ];
    memset(frame2, 0, sizeof(frame2));
    for (int y = 20; y < 44; y++) {
        for (int x = 20; x < 44; x++) {
            frame2[y * IMG_W + x] = 200;
        }
    }

    /* Write frames to guest physical memory */
    write_phys(BUF_FRAME1, frame1, IMG_SZ);
    write_phys(BUF_FRAME2, frame2, IMG_SZ);

    /* Clear SAD output */
    uint8_t zeros[SAD_SZ];
    memset(zeros, 0, sizeof(zeros));
    write_phys(BUF_SAD_OUT, zeros, SAD_SZ);

    /* Program SAD operation */
    ive_write(IVE_OP_TYPE,    IVE_OP_SAD);
    ive_write(IVE_DIMENSIONS, (IMG_H << 16) | IMG_W);
    ive_write(IVE_SRC1_ADDR,  BUF_FRAME1);
    ive_write(IVE_SRC2_ADDR,  BUF_FRAME2);
    ive_write(IVE_DST1_ADDR,  BUF_SAD_OUT);
    ive_write(IVE_STRIDES_0,  (IMG_W << 16) | SAD_W);
    ive_write(IVE_STRIDES_1,  (IMG_W << 16));
    ive_write(IVE_THRESH_VAL, 0x0000C800);  /* threshold ~200 in packed format */

    uint32_t ops_before = ive_read(IVE_TOTAL_OPS);
    ive_fire_and_wait();
    uint32_t ops_after = ive_read(IVE_TOTAL_OPS);

    /* Read SAD output */
    uint8_t sad_out[SAD_SZ];
    read_phys(BUF_SAD_OUT, sad_out, SAD_SZ);

    /* The rectangle at (20,20)-(44,44) spans blocks (5,5)-(10,10) in 4x4 grid */
    int motion_pixels = 0;
    int total_pixels = 0;
    for (int y = 0; y < SAD_H; y++) {
        for (int x = 0; x < SAD_W; x++) {
            total_pixels++;
            if (sad_out[y * SAD_W + x] > 0) {
                motion_pixels++;
            }
        }
    }

    int ops_inc = ops_after - ops_before;
    printf("ops_inc=%d, motion_blocks=%d/%d ... ", ops_inc, motion_pixels, total_pixels);

    if (ops_inc >= 1 && motion_pixels > 0 && motion_pixels < total_pixels) {
        printf("PASS\n");
        return 0;
    }
    printf("FAIL\n");
    return 1;
}

static int test_ccl(void) {
    printf("TEST ccl: ");

    /* Create a binary image with two separate blobs */
    uint8_t binary[SAD_SZ];
    memset(binary, 0, sizeof(binary));

    /* Blob 1: 3x3 block at (2,2) */
    for (int y = 2; y <= 4; y++)
        for (int x = 2; x <= 4; x++)
            binary[y * SAD_W + x] = 255;

    /* Blob 2: 4x2 block at (10,10) */
    for (int y = 10; y <= 11; y++)
        for (int x = 10; x <= 13; x++)
            binary[y * SAD_W + x] = 255;

    write_phys(BUF_SAD_OUT, binary, SAD_SZ);

    /* Clear blob output */
    uint8_t blob_zero[BLOB_SZ];
    memset(blob_zero, 0, sizeof(blob_zero));
    write_phys(BUF_BLOB, blob_zero, BLOB_SZ);

    /* Program CCL operation */
    ive_write(IVE_OP_TYPE,    IVE_OP_CCL);
    ive_write(IVE_DIMENSIONS, (SAD_H << 16) | SAD_W);
    ive_write(IVE_SRC1_ADDR,  BUF_SAD_OUT);
    ive_write(IVE_DST1_ADDR,  BUF_BLOB);
    ive_write(IVE_CCL_CFG0,   (4 << 16));  /* area threshold = 4 */

    ive_fire_and_wait();

    /* Read blob output */
    uint8_t blob[BLOB_SZ];
    read_phys(BUF_BLOB, blob, BLOB_SZ);

    /* Parse IVE_CCBLOB_S header */
    uint32_t area_thr = blob[0] | (blob[1] << 8) | (blob[2] << 16) | (blob[3] << 24);
    int8_t label_status = (int8_t)blob[4];
    uint8_t region_num = blob[5];

    printf("regions=%d, area_thr=%d, status=%d", region_num, area_thr, label_status);

    /* Parse first two regions */
    if (region_num >= 2) {
        for (int i = 0; i < 2; i++) {
            uint8_t *r = blob + 8 + i * 12;
            uint32_t area = r[0] | (r[1] << 8) | (r[2] << 16) | (r[3] << 24);
            uint16_t left = r[4] | (r[5] << 8);
            uint16_t top = r[6] | (r[7] << 8);
            uint16_t right = r[8] | (r[9] << 8);
            uint16_t bottom = r[10] | (r[11] << 8);
            printf(", r%d=[%d,%d-%d,%d area=%d]", i, left, top, right, bottom, area);
        }
    }

    if (region_num == 2 && label_status == 0) {
        printf(" ... PASS\n");
        return 0;
    }
    printf(" ... FAIL (expected 2 regions)\n");
    return 1;
}

static int test_dma(void) {
    printf("TEST dma: ");

    /* Write a pattern to source */
    uint8_t src[256];
    for (int i = 0; i < 256; i++) src[i] = i;
    write_phys(BUF_FRAME1, src, 256);

    /* Clear destination */
    uint8_t zeros[256];
    memset(zeros, 0, sizeof(zeros));
    write_phys(BUF_FRAME2, zeros, 256);

    /* Program DMA copy: 16x16 block */
    ive_write(IVE_OP_TYPE,    IVE_OP_DMA);
    ive_write(IVE_DIMENSIONS, (16 << 16) | 16);
    ive_write(IVE_SRC1_ADDR,  BUF_FRAME1);
    ive_write(IVE_DST1_ADDR,  BUF_FRAME2);
    ive_write(IVE_STRIDES_0,  (16 << 16) | 16);

    ive_fire_and_wait();

    /* Read back destination */
    uint8_t dst[256];
    read_phys(BUF_FRAME2, dst, 256);

    if (memcmp(src, dst, 256) == 0) {
        printf("PASS\n");
        return 0;
    }
    printf("FAIL (data mismatch)\n");
    return 1;
}

int main(int argc, char **argv) {
    uint32_t ive_base = 0x11320000;
    if (argc > 1) ive_base = strtoul(argv[1], NULL, 0);

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) { perror("/dev/mem"); return 1; }

    ive = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE,
               MAP_SHARED, mem_fd, ive_base);
    if (ive == MAP_FAILED) { perror("mmap ive"); return 1; }

    printf("=== IVE Hardware Test (base=0x%08X) ===\n", ive_base);

    int fails = 0;
    fails += test_hwid();
    fails += test_dma();
    fails += test_sad();
    fails += test_ccl();

    printf("=== %d/%d tests passed ===\n", 4 - fails, 4);

    munmap((void *)ive, 0x10000);
    close(mem_fd);
    return fails;
}
