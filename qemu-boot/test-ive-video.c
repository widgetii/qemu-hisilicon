/*
 * IVE Motion Detection on Video Frames
 *
 * Reads raw grayscale frames from /frames.bin (embedded in initramfs),
 * runs SAD+CCL via IVE hardware registers, prints bounding boxes.
 * Same binary works on real EV300 board and in QEMU.
 *
 * Output format (one line per frame with motion):
 *   FRAME N: (l,t)-(r,b) area=A [(l,t)-(r,b) area=A ...]
 *
 * Build: arm-openipc-linux-musleabi-gcc -static -O2 -o test-ive-video test-ive-video.c
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
#define IVE_OP_TYPE     0x0104
#define IVE_DIMENSIONS  0x0108
#define IVE_SRC1_ADDR   0x0110
#define IVE_DST1_ADDR   0x0114
#define IVE_SRC2_ADDR   0x0118
#define IVE_STRIDES_0   0x0128
#define IVE_STRIDES_1   0x012C
#define IVE_THRESH_VAL  0x0138
#define IVE_CCL_CFG0    0x0160

#define W       64
#define H       48
#define BLOCK   4
#define BW      (W / BLOCK)
#define BH      (H / BLOCK)
#define FRAME_SZ (W * H)
#define SAD_SZ  (BW * BH)
#define BLOB_SZ 3056

static volatile uint32_t *ive;
static int mem_fd;

static void ive_w(uint32_t off, uint32_t val) { ive[off/4] = val; }
static uint32_t ive_r(uint32_t off) { return ive[off/4]; }

static void ive_fire(void) {
    ive_w(IVE_SW_FIRE, 1);
    for (int i = 0; i < 100000 && !(ive_r(IVE_CMD_DONE) & 1); i++)
        usleep(1);
}

static uint64_t v2p(void *va) {
    uint64_t val;
    int fd = open("/proc/self/pagemap", O_RDONLY);
    if (fd < 0) return 0;
    if (pread(fd, &val, 8, ((uintptr_t)va / 4096) * 8) != 8) { close(fd); return 0; }
    close(fd);
    return (val & (1ULL<<63)) ? ((val & ((1ULL<<55)-1)) * 4096) : 0;
}

static void *alloc_page(uint64_t *phys, size_t size) {
    void *p = mmap(NULL, size, PROT_READ|PROT_WRITE,
                   MAP_PRIVATE|MAP_ANONYMOUS|MAP_LOCKED, -1, 0);
    if (p == MAP_FAILED) return NULL;
    memset(p, 0, size);
    *phys = v2p(p);
    return (*phys) ? p : NULL;
}

int main(void) {
    int is_init = (getpid() == 1);

    if (is_init) {
        mkdir("/dev", 0755);
        mkdir("/proc", 0755);
        mount("devtmpfs", "/dev", "devtmpfs", 0, NULL);
        mount("proc", "/proc", "proc", 0, NULL);
    }

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) { perror("/dev/mem"); goto done; }

    ive = mmap(NULL, 0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0x11320000);
    if (ive == MAP_FAILED) { perror("mmap ive"); goto done; }

    /* Open frames file */
    FILE *fp = fopen("/frames.bin", "rb");
    if (!fp) { perror("/frames.bin"); goto done; }

    fseek(fp, 0, SEEK_END);
    long file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    int nframes = file_size / FRAME_SZ;

    /* Allocate buffers */
    uint64_t pa_cur, pa_prev, pa_sad, pa_blob;
    void *va_cur  = alloc_page(&pa_cur,  FRAME_SZ);
    void *va_prev = alloc_page(&pa_prev, FRAME_SZ);
    void *va_sad  = alloc_page(&pa_sad,  SAD_SZ);
    void *va_blob = alloc_page(&pa_blob, BLOB_SZ);

    if (!va_cur || !va_prev || !va_sad || !va_blob) {
        printf("ERROR: buffer allocation failed\n");
        goto done;
    }

    uint8_t *cur = va_cur, *prev = va_prev;
    uint8_t *sad_out = va_sad, *blob = va_blob;

    /* Read first frame as previous */
    if (fread(prev, 1, FRAME_SZ, fp) != FRAME_SZ) goto done;

    for (int i = 1; i < nframes; i++) {
        if (fread(cur, 1, FRAME_SZ, fp) != FRAME_SZ) break;

        /* SAD operation */
        ive_w(IVE_OP_TYPE, 1);
        ive_w(IVE_DIMENSIONS, (H << 16) | W);
        ive_w(IVE_SRC1_ADDR, pa_prev);
        ive_w(IVE_SRC2_ADDR, pa_cur);
        ive_w(IVE_DST1_ADDR, pa_sad);
        ive_w(IVE_STRIDES_0, (W << 16) | BW);
        ive_w(IVE_STRIDES_1, (W << 16));
        ive_w(IVE_THRESH_VAL, 0x00003200);  /* threshold=50 for 64x48 frames */
        memset(sad_out, 0, SAD_SZ);
        ive_fire();

        /* CCL operation */
        ive_w(IVE_OP_TYPE, 2);
        ive_w(IVE_DIMENSIONS, (BH << 16) | BW);
        ive_w(IVE_SRC1_ADDR, pa_sad);
        ive_w(IVE_DST1_ADDR, pa_blob);
        ive_w(IVE_CCL_CFG0, (4 << 16));   /* area_thr=4 for small frames */
        memset(blob, 0, BLOB_SZ);
        ive_fire();

        /* Parse IVE_CCBLOB_S */
        uint8_t nreg = blob[5];
        if (nreg > 0) {
            printf("FRAME %d:", i);
            for (int r = 0; r < nreg && r < 10; r++) {
                uint8_t *p = blob + 8 + r * 12;
                uint32_t area = p[0]|(p[1]<<8)|(p[2]<<16)|(p[3]<<24);
                uint16_t left   = (p[4]|(p[5]<<8)) * BLOCK;
                uint16_t top    = (p[6]|(p[7]<<8)) * BLOCK;
                uint16_t right  = ((p[8]|(p[9]<<8)) + 1) * BLOCK;
                uint16_t bottom = ((p[10]|(p[11]<<8)) + 1) * BLOCK;
                printf(" (%d,%d)-(%d,%d) area=%d", left, top, right, bottom, area);
            }
            printf("\n");
        }

        /* Swap: current becomes previous */
        uint8_t *tmp_v = prev; prev = cur; cur = tmp_v;
        uint64_t tmp_p = pa_prev; pa_prev = pa_cur; pa_cur = tmp_p;
    }

    fclose(fp);

done:
    if (is_init) {
        sync();
        reboot(LINUX_REBOOT_CMD_POWER_OFF);
        for (;;) sleep(1);
    }
    return 0;
}
