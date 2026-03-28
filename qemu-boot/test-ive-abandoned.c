/*
 * IVE Abandoned Object Detection Test
 *
 * Uses Sub + Thresh + CCL + SAD via IVE registers (QEMU) or
 * software (--sw for real board). Same algorithm as abandoned_demo.py.
 *
 * Build: arm-openipc-linux-musleabi-gcc -static -O2 -o test-ive-abandoned test-ive-abandoned.c
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
#define IVE_THRESH_MAX  0x014C
#define IVE_CCL_CFG0    0x0160

#define OP_SUB      3
#define OP_THRESH   8
#define OP_CCL      2
#define OP_SAD      1

#define W 64
#define H 64
#define SZ (W * H)
#define BLOB_SZ 3056

#define LEARN_FRAMES    20
#define ABANDON_FRAMES  30
#define DIFF_THR        40
#define MOTION_THR      100
#define AREA_THR        8

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

static void *alloc_buf(uint64_t *pa, int size) {
    void *p = mmap(NULL, size, PROT_READ|PROT_WRITE,
                   MAP_PRIVATE|MAP_ANONYMOUS|MAP_LOCKED, -1, 0);
    if (p == MAP_FAILED) return NULL;
    memset(p, 0, size);
    *pa = v2p(p);
    return (*pa) ? p : NULL;
}

/* Stationary blob tracker */
#define MAX_BLOBS 16
typedef struct { int cx, cy, duration; } Blob;
static Blob tracked[MAX_BLOBS];
static int ntracked = 0;

static void track_update(int cx, int cy) {
    /* Find nearest tracked blob */
    for (int i = 0; i < ntracked; i++) {
        if (abs(tracked[i].cx - cx) + abs(tracked[i].cy - cy) <= 3) {
            tracked[i].cx = cx;
            tracked[i].cy = cy;
            tracked[i].duration++;
            return;
        }
    }
    /* New blob */
    if (ntracked < MAX_BLOBS) {
        tracked[ntracked++] = (Blob){cx, cy, 1};
    }
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

    FILE *fp = fopen("/tmp/frames.bin", "rb");
    if (!fp) fp = fopen("/frames.bin", "rb");
    if (!fp) { perror("frames.bin"); goto done; }
    fseek(fp, 0, SEEK_END);
    int nframes = ftell(fp) / SZ;
    fseek(fp, 0, SEEK_SET);

    /* Buffers */
    uint64_t pa_cur, pa_ref, pa_diff, pa_bin, pa_blob, pa_prev;
    uint8_t *va_cur  = alloc_buf(&pa_cur, SZ);
    uint8_t *va_ref  = alloc_buf(&pa_ref, SZ);
    uint8_t *va_diff = alloc_buf(&pa_diff, SZ);
    uint8_t *va_bin  = alloc_buf(&pa_bin, SZ);
    uint8_t *va_blob = alloc_buf(&pa_blob, BLOB_SZ);
    uint8_t *va_prev = alloc_buf(&pa_prev, SZ);
    if (!va_cur || !va_ref || !va_diff || !va_bin || !va_blob || !va_prev) {
        fprintf(stderr, "ERROR: alloc failed\n"); goto done;
    }

    fprintf(stderr, "Abandoned object detection (%s), %d frames\n",
            sw_mode ? "SW" : "HW", nframes);

    /* Phase 1: Learn reference background */
    int bg_sum[SZ];
    memset(bg_sum, 0, sizeof(bg_sum));
    for (int i = 0; i < LEARN_FRAMES && i < nframes; i++) {
        fread(va_cur, 1, SZ, fp);
        for (int j = 0; j < SZ; j++) bg_sum[j] += va_cur[j];
    }
    for (int j = 0; j < SZ; j++) va_ref[j] = bg_sum[j] / LEARN_FRAMES;

    /* Rewind and process all frames */
    fseek(fp, 0, SEEK_SET);
    memset(va_prev, 0, SZ);

    for (int i = 0; i < nframes; i++) {
        fread(va_cur, 1, SZ, fp);

        /* Sub: |current - reference| */
        if (sw_mode) {
            for (int j = 0; j < SZ; j++)
                va_diff[j] = abs((int)va_cur[j] - (int)va_ref[j]);
        } else {
            ive_w(IVE_OP_TYPE, OP_SUB);
            ive_w(IVE_DIMENSIONS, (H << 16) | W);
            ive_w(IVE_SRC1_ADDR, pa_cur);
            ive_w(IVE_SRC2_ADDR, pa_ref);
            ive_w(IVE_DST1_ADDR, pa_diff);
            ive_w(IVE_STRIDES_0, (W << 16) | W);
            ive_w(IVE_STRIDES_1, (W << 16));
            ive_fire();
        }

        /* Thresh: binary mask */
        if (sw_mode) {
            for (int j = 0; j < SZ; j++)
                va_bin[j] = (va_diff[j] > DIFF_THR) ? 255 : 0;
        } else {
            ive_w(IVE_OP_TYPE, OP_THRESH);
            ive_w(IVE_DIMENSIONS, (H << 16) | W);
            ive_w(IVE_SRC1_ADDR, pa_diff);
            ive_w(IVE_DST1_ADDR, pa_bin);
            ive_w(IVE_STRIDES_0, (W << 16) | W);
            ive_w(IVE_THRESH_VAL, DIFF_THR << 8);
            ive_w(IVE_THRESH_MAX, 255);
            ive_fire();
        }

        /* CCL: find regions */
        int16_t labels[SZ], parent[256];
        uint32_t area[256];
        uint16_t bl[256], bt[256], br[256], bb[256];
        memset(labels, 0, sizeof(labels));
        for (int j = 0; j < 256; j++) { parent[j] = j; bl[j]=W; bt[j]=H; br[j]=0; bb[j]=0; }
        memset(area, 0, sizeof(area));
        int nl = 1;

        #define FIND(x) ({ int16_t _r=(x); while(parent[_r]!=_r) _r=parent[_r]; _r; })
        #define UNION(a,b) do { int16_t _a=FIND(a),_b=FIND(b); if(_a!=_b) parent[_a]=_b; } while(0)

        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                if (!va_bin[y*W+x]) continue;
                int16_t left = (x>0) ? labels[y*W+x-1] : 0;
                int16_t up = (y>0) ? labels[(y-1)*W+x] : 0;
                if (left && up) { labels[y*W+x]=left; UNION(left,up); }
                else if (left) labels[y*W+x]=left;
                else if (up) labels[y*W+x]=up;
                else if (nl<254) labels[y*W+x]=nl++;
            }

        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                int16_t l = labels[y*W+x]; if (!l) continue;
                l = FIND(l);
                area[l]++;
                if (x < bl[l]) bl[l] = x;
                if (y < bt[l]) bt[l] = y;
                if (x > br[l]) br[l] = x;
                if (y > bb[l]) bb[l] = y;
            }
        #undef FIND
        #undef UNION

        /* Filter: only stationary regions (low SAD with prev frame) */
        ntracked = 0; /* rebuild each frame for simplicity */

        /* Carry forward tracked blobs from static storage */
        static Blob prev_tracked[MAX_BLOBS];
        static int prev_ntracked = 0;

        Blob new_tracked[MAX_BLOBS];
        int new_ntracked = 0;

        for (int r = 1; r < nl; r++) {
            if (area[r] < AREA_THR) continue;

            /* SAD between current and previous frame in this region */
            int sad = 0;
            for (int y = bt[r]; y <= bb[r]; y++)
                for (int x = bl[r]; x <= br[r]; x++)
                    sad += abs((int)va_cur[y*W+x] - (int)va_prev[y*W+x]);

            if (sad >= MOTION_THR) continue; /* moving → skip */

            int cx = (bl[r] + br[r]) / 2;
            int cy = (bt[r] + bb[r]) / 2;

            /* Match to previous tracked blob */
            int dur = 1;
            for (int t = 0; t < prev_ntracked; t++) {
                if (abs(prev_tracked[t].cx - cx) + abs(prev_tracked[t].cy - cy) <= 3) {
                    dur = prev_tracked[t].duration + 1;
                    break;
                }
            }

            if (new_ntracked < MAX_BLOBS) {
                new_tracked[new_ntracked++] = (Blob){cx, cy, dur};
            }

            if (dur >= ABANDON_FRAMES) {
                printf("FRAME %d: ABANDONED (%d,%d)-(%d,%d) area=%d dur=%d\n",
                       i, bl[r], bt[r], br[r], bb[r], area[r], dur);
            }
        }

        memcpy(prev_tracked, new_tracked, sizeof(Blob) * new_ntracked);
        prev_ntracked = new_ntracked;
        memcpy(va_prev, va_cur, SZ);
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
