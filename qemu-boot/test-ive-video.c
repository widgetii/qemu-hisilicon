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
static int sw_mode; /* 1 = software-only (no IVE registers) */

static void ive_w(uint32_t off, uint32_t val) { ive[off/4] = val; }
static uint32_t ive_r(uint32_t off) { return ive[off/4]; }

static void ive_fire(void) {
    ive_w(IVE_SW_FIRE, 1);
    /* Poll CMD_DONE (works in QEMU); on real HW the operation completes
     * before we can even poll, so timeout after 1ms is fine. */
    for (int i = 0; i < 1000 && !(ive_r(IVE_CMD_DONE) & 1); i++)
        usleep(1);
    usleep(100); /* extra settle time for real HW DMA */
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

int main(int argc, char **argv) {
    int is_init = (getpid() == 1);
    /* --sw flag forces software mode */
    for (int a = 1; a < argc; a++)
        if (!strcmp(argv[a], "--sw")) sw_mode = 1;

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

    /* Open frames file — try /tmp first (real board), fallback to / (initramfs) */
    FILE *fp = fopen("/tmp/frames.bin", "rb");
    if (!fp) fp = fopen("/frames.bin", "rb");
    if (!fp) { perror("frames.bin"); goto done; }

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
        fprintf(stderr, "ERROR: buffer allocation failed\n");
        fprintf(stderr, "  cur=%p/%llx prev=%p/%llx sad=%p/%llx blob=%p/%llx\n",
            va_cur, (unsigned long long)pa_cur, va_prev, (unsigned long long)pa_prev,
            va_sad, (unsigned long long)pa_sad, va_blob, (unsigned long long)pa_blob);
        goto done;
    }
    fprintf(stderr, "Buffers OK: cur=0x%llx prev=0x%llx sad=0x%llx blob=0x%llx nframes=%d\n",
        (unsigned long long)pa_cur, (unsigned long long)pa_prev,
        (unsigned long long)pa_sad, (unsigned long long)pa_blob, nframes);

    uint8_t *cur = va_cur, *prev = va_prev;
    uint8_t *sad_out = va_sad, *blob = va_blob;

    /* Check for --sw flag */
    /* (when running as init, argc isn't available; check /proc/cmdline) */
    {
        int cfd = open("/proc/cmdline", O_RDONLY);
        if (cfd >= 0) {
            char cmdline[512] = {0};
            read(cfd, cmdline, sizeof(cmdline)-1);
            close(cfd);
            if (strstr(cmdline, "ive_sw=1")) sw_mode = 1;
        }
        /* Also check argv for non-init usage */
        if (!is_init) {
            /* Simple: if IVE registers fail to map, use SW mode */
            if (ive == MAP_FAILED || ive == NULL) sw_mode = 1;
        }
    }
    if (sw_mode) fprintf(stderr, "Running in SOFTWARE mode (no IVE HW)\n");

    /* Read first frame as previous */
    if (fread(prev, 1, FRAME_SZ, fp) != FRAME_SZ) goto done;
    fprintf(stderr, "Processing %d frames...\n", nframes);

    for (int i = 1; i < nframes; i++) {
        if (fread(cur, 1, FRAME_SZ, fp) != FRAME_SZ) break;
        if (i <= 3) fprintf(stderr, "  frame %d: cur[0]=%d prev[0]=%d\n", i, cur[0], prev[0]);

        if (sw_mode) {
            /* Software SAD: 4×4 blocks */
            memset(sad_out, 0, SAD_SZ);
            for (int by = 0; by < BH; by++) {
                for (int bx = 0; bx < BW; bx++) {
                    int sad = 0;
                    for (int dy = 0; dy < BLOCK; dy++)
                        for (int dx = 0; dx < BLOCK; dx++) {
                            int y = by*BLOCK+dy, x = bx*BLOCK+dx;
                            sad += abs((int)cur[y*W+x] - (int)prev[y*W+x]);
                        }
                    sad_out[by*BW+bx] = (sad > 50) ? 255 : 0;
                }
            }
            /* Software CCL: 4-connected union-find */
            int16_t labels[BW*BH], parent[256];
            memset(labels, 0, sizeof(labels));
            for (int j = 0; j < 256; j++) parent[j] = j;
            int nl = 1;
            #define FIND(x) ({ int16_t _r=(x); while(parent[_r]!=_r) _r=parent[_r]; _r; })
            #define UNION(a,b) do { int16_t _a=FIND(a),_b=FIND(b); if(_a!=_b) parent[_a]=_b; } while(0)
            for (int y = 0; y < BH; y++)
                for (int x = 0; x < BW; x++) {
                    if (!sad_out[y*BW+x]) continue;
                    int16_t left = (x>0) ? labels[y*BW+x-1] : 0;
                    int16_t up = (y>0) ? labels[(y-1)*BW+x] : 0;
                    if (left && up) { labels[y*BW+x]=left; UNION(left,up); }
                    else if (left) labels[y*BW+x]=left;
                    else if (up) labels[y*BW+x]=up;
                    else if (nl<254) labels[y*BW+x]=nl++;
                }
            /* Compute bboxes */
            uint32_t area[256]={0}; uint16_t bl[256],bt[256],br[256],bb[256];
            for (int j=0;j<256;j++) { bl[j]=BW; bt[j]=BH; br[j]=0; bb[j]=0; }
            for (int y=0;y<BH;y++) for (int x=0;x<BW;x++) {
                int16_t l=labels[y*BW+x]; if(!l) continue; l=FIND(l);
                area[l]++; if(x<bl[l])bl[l]=x; if(y<bt[l])bt[l]=y;
                if(x>br[l])br[l]=x; if(y>bb[l])bb[l]=y;
            }
            #undef FIND
            #undef UNION
            int nreg = 0;
            int printed = 0;
            for (int j=1;j<nl;j++) {
                if (area[j]>=4) {
                    if (!printed) { printf("FRAME %d:", i); printed=1; }
                    printf(" (%d,%d)-(%d,%d) area=%d",
                        bl[j]*BLOCK, bt[j]*BLOCK, (br[j]+1)*BLOCK, (bb[j]+1)*BLOCK, area[j]);
                    nreg++;
                }
            }
            if (printed) printf("\n");
        } else {

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

        } /* end else (IVE hardware path) */

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
