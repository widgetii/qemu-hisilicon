/*
 * IVP (Intelligent Video Processing) Test — Run vendor CNN model on Y4M frames
 *
 * Uses the public hi_ivp_* API to load the vendor OMS model and run object
 * detection on offline Y4M input. Reports detections with bboxes and timing.
 *
 * The IVP framework internally uses the IVE XNN CNN accelerator to run
 * the YOLO-based detection network at hardware speed.
 *
 * Build (from qemu-boot/):
 *   CC=~/git/firmware/output-hi3516ev300_lite/host/bin/arm-openipc-linux-musleabi-gcc
 *   SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
 *   LIBS=~/git/firmware/output-hi3516ev300_lite/target/usr/lib
 *   $CC -O2 -o test-ivp test-ivp.c \
 *       $SDK/lib/libivp.a $SDK/lib/libqr.a \
 *       -I$SDK/include -L$LIBS \
 *       -lmpi -live -lVoiceEngine -lupvqe -ldnvqe -lsecurec \
 *       -Wl,-rpath,/usr/lib -Wl,--allow-shlib-undefined -lm -lpthread
 *
 * Run: killall majestic; ./test-ivp model.oms input.y4m
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/syscall.h>

/* We override mmap below, so provide the constants ourselves instead of mman.h */
#define PROT_READ   0x1
#define PROT_WRITE  0x2
#define MAP_SHARED  0x01
#define MAP_PRIVATE 0x02
#define MAP_ANONYMOUS 0x20
#define MAP_LOCKED  0x2000
#define MAP_FAILED  ((void *)-1)
int munmap(void *addr, size_t len);

/*
 * uclibc→musl shims.
 *
 * Vendor SDK libraries (libivp.a, libive.a, libmpi.so) are compiled with
 * uclibc which has different ABI for mmap (uint32_t offset vs musl's off_t)
 * and provides _s string functions, __ctype_b, etc. We override them here
 * so vendor static libs link and run correctly on musl-based OpenIPC.
 *
 * mmap shim is critical: uclibc mmap passes offset as uint32_t (4 bytes),
 * musl mmap expects off_t (8 bytes). On ARM32 calling convention this means
 * the offset lands in the wrong register. We call SYS_mmap2 directly.
 * See: ~/git/majestic/src/libc/uclibc.c
 */

/* mmap: uclibc passes uint32_t off, must translate to mmap2 syscall */
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
#include "hi_comm_video.h"
#include "hi_ivp.h"
#include "mpi_sys.h"

/* Stub: IVP calls ISP_SetSmartInfo for adaptive ISP, which we don't need */
HI_S32 ISP_SetSmartInfo(HI_S32 pipe, void *info) { return 0; }
/* Stub: IVP calls VGS for drawing, which we don't need */
HI_S32 HI_MPI_VGS_BeginJob(void *h) { return 0; }
HI_S32 HI_MPI_VGS_EndJob(void *h) { return 0; }
HI_S32 HI_MPI_VGS_CancelJob(void *h) { return 0; }
HI_S32 HI_MPI_VGS_AddLumaTaskArray(void *h, void *t) { return 0; }
HI_S32 HI_MPI_VENC_GetFd(HI_S32 ch) { return -1; }

#define MAX_CLASS_NUM 2
#define MAX_RECT_NUM  10

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

/* Load OMS model file into MMZ and init IVP */
static HI_S32 load_model(const char *model_path, hi_s32 *handle,
                          hi_ivp_mem_info *mem)
{
    FILE *fp = fopen(model_path, "rb");
    if (!fp) { perror(model_path); return HI_FAILURE; }
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    HI_S32 ret = HI_MPI_SYS_MmzAlloc(&mem->physical_addr,
        (HI_VOID **)&mem->virtual_addr, "IVP_MODEL", NULL, sz);
    if (ret != HI_SUCCESS) {
        fprintf(stderr, "MmzAlloc for model: 0x%x\n", ret);
        fclose(fp); return ret;
    }
    mem->memory_size = sz;

    if (fread((void *)(HI_UL)mem->virtual_addr, 1, sz, fp) != (size_t)sz) {
        fprintf(stderr, "fread model failed\n");
        fclose(fp); return HI_FAILURE;
    }
    fclose(fp);

    ret = hi_ivp_load_resource_from_memory(mem, handle);
    if (ret != HI_SUCCESS) {
        fprintf(stderr, "hi_ivp_load_resource_from_memory: 0x%x\n", ret);
        HI_MPI_SYS_MmzFree(mem->physical_addr, (HI_VOID *)(HI_UL)mem->virtual_addr);
    }
    return ret;
}

/*
 * Build a VIDEO_FRAME_INFO_S from a Y4M grayscale frame.
 * IVP expects YUV420SP (NV21). We allocate Y + UV planes in MMZ.
 * Y plane = frame data, UV plane = 0x80 (neutral chroma).
 */
static HI_S32 build_frame(VIDEO_FRAME_INFO_S *fi, int W, int H,
                           HI_U64 *phys, HI_U64 *virt)
{
    HI_U32 stride = (W + 15) & ~15;
    HI_U32 y_size = stride * H;
    HI_U32 uv_size = stride * H / 2;
    HI_U32 total = y_size + uv_size;

    HI_S32 ret = HI_MPI_SYS_MmzAlloc(phys, (HI_VOID **)virt,
        "IVP_FRAME", NULL, total);
    if (ret != HI_SUCCESS) return ret;

    /* Fill UV plane with 0x80 (once) */
    memset((uint8_t *)(HI_UL)*virt + y_size, 0x80, uv_size);

    memset(fi, 0, sizeof(*fi));
    fi->stVFrame.u32Width = W;
    fi->stVFrame.u32Height = H;
    fi->stVFrame.enField = VIDEO_FIELD_FRAME;
    fi->stVFrame.enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    fi->stVFrame.enVideoFormat = VIDEO_FORMAT_LINEAR;
    fi->stVFrame.enCompressMode = COMPRESS_MODE_NONE;
    fi->stVFrame.enDynamicRange = DYNAMIC_RANGE_SDR8;
    fi->stVFrame.enColorGamut = COLOR_GAMUT_BT601;
    fi->stVFrame.u32Stride[0] = stride;
    fi->stVFrame.u32Stride[1] = stride;
    fi->stVFrame.u64PhyAddr[0] = *phys;
    fi->stVFrame.u64VirAddr[0] = *virt;
    fi->stVFrame.u64PhyAddr[1] = *phys + y_size;
    fi->stVFrame.u64VirAddr[1] = *virt + y_size;
    fi->u32PoolId = (HI_U32)-1;

    return HI_SUCCESS;
}

static void fill_y_plane(VIDEO_FRAME_INFO_S *fi, const uint8_t *data,
                          int W, int H)
{
    uint8_t *v = (uint8_t *)(HI_UL)fi->stVFrame.u64VirAddr[0];
    HI_U32 stride = fi->stVFrame.u32Stride[0];
    for (int y = 0; y < H; y++)
        memcpy(v + y * stride, data + y * W, W);
    HI_U32 total = stride * H + stride * H / 2;
    HI_MPI_SYS_MmzFlushCache(fi->stVFrame.u64PhyAddr[0],
        (HI_VOID *)(HI_UL)fi->stVFrame.u64VirAddr[0], total);
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <model.oms> <input.y4m>\n", argv[0]);
        return 1;
    }
    const char *model_path = argv[1];
    const char *input_path = argv[2];

    FILE *fp = fopen(input_path, "rb");
    if (!fp) { perror(input_path); return 1; }

    /* Parse Y4M header */
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

    int frame_sz = W * H; /* grayscale Y4M = Y-only */
    long file_pos = ftell(fp);
    fseek(fp, 0, SEEK_END);
    long file_end = ftell(fp);
    fseek(fp, file_pos, SEEK_SET);
    int nframes = (int)((file_end - file_pos) / (frame_sz + (is_y4m ? 6 : 0)));

    fprintf(stderr, "Input: %s %dx%d, %d frames\n",
            is_y4m ? "Y4M" : "raw", W, H, nframes);

    /* Init MPI */
    HI_S32 ret = HI_MPI_SYS_Init();
    if (ret != HI_SUCCESS) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* Init IVP */
    ret = hi_ivp_init();
    if (ret != HI_SUCCESS) { fprintf(stderr, "hi_ivp_init: 0x%x\n", ret); return 1; }

    /* Load model */
    hi_s32 ivp_handle;
    hi_ivp_mem_info model_mem = {0};
    ret = load_model(model_path, &ivp_handle, &model_mem);
    if (ret != HI_SUCCESS) { fprintf(stderr, "load_model failed: 0x%x\n", ret); return 1; }
    fprintf(stderr, "Model loaded: %s (%u bytes)\n", model_path, model_mem.memory_size);

    /* Set detection threshold */
    hi_ivp_ctrl_attr ctrl = { .threshold = 0.5f };
    hi_ivp_set_ctrl_attr(ivp_handle, &ctrl);

    /* Build frame buffer */
    VIDEO_FRAME_INFO_S frame_info;
    HI_U64 frame_phys, frame_virt;
    ret = build_frame(&frame_info, W, H, &frame_phys, &frame_virt);
    if (ret != HI_SUCCESS) { fprintf(stderr, "build_frame: 0x%x\n", ret); return 1; }

    /* Prepare obj_array */
    hi_ivp_obj_array obj_array;
    hi_ivp_obj rects[MAX_CLASS_NUM][MAX_RECT_NUM];

    uint8_t *frame_data = malloc(frame_sz);

    long long t_total = 0;
    int total_detections = 0;

    fprintf(stderr, "\n--- Processing %d frames ---\n", nframes);

    for (int i = 0; i < nframes; i++) {
        if (!read_y4m_frame(fp, frame_data, frame_sz, is_y4m)) break;

        /* Fill Y plane into MMZ frame buffer */
        fill_y_plane(&frame_info, frame_data, W, H);

        /* Setup obj_array for this frame */
        memset(&obj_array, 0, sizeof(obj_array));
        for (int c = 0; c < MAX_CLASS_NUM; c++) {
            obj_array.obj_class[c].rect_capcity = MAX_RECT_NUM;
            obj_array.obj_class[c].objs = rects[c];
            memset(rects[c], 0, sizeof(rects[c]));
        }

        /* Run IVP inference */
        long long t0 = usec_now();
        ret = hi_ivp_process_ex(ivp_handle, &frame_info, &obj_array);
        long long dt = usec_now() - t0;
        t_total += dt;

        if (ret != HI_SUCCESS) {
            fprintf(stderr, "FRAME %d: hi_ivp_process_ex failed: 0x%x\n", i, ret);
            continue;
        }

        /* Print detections */
        for (int c = 0; c < obj_array.class_num; c++) {
            hi_ivp_obj_of_one_class *cls = &obj_array.obj_class[c];
            for (HI_U32 r = 0; r < cls->rect_num; r++) {
                printf("FRAME %d: class=%d \"%s\" bbox=(%d,%d,%d,%d) conf=%.3f\n",
                       i, c, cls->class_name,
                       cls->objs[r].rect.x, cls->objs[r].rect.y,
                       cls->objs[r].rect.width, cls->objs[r].rect.height,
                       cls->objs[r].quality);
                total_detections++;
            }
        }
    }

    fclose(fp);

    /* Benchmark */
    fprintf(stderr, "\n=== IVP Benchmark (%d frames, %dx%d) ===\n", nframes, W, H);
    fprintf(stderr, "  Inference:    %lld ms (%.1f ms/frame)\n",
            t_total / 1000, (double)t_total / 1000 / nframes);
    fprintf(stderr, "  Throughput:   %.1f fps\n",
            1000000.0 * nframes / t_total);
    fprintf(stderr, "  Detections:   %d total\n", total_detections);

    /* Cleanup */
    free(frame_data);
    HI_MPI_SYS_MmzFree(frame_phys, (HI_VOID *)(HI_UL)frame_virt);
    hi_ivp_unload_resource(ivp_handle);
    HI_MPI_SYS_MmzFree(model_mem.physical_addr,
        (HI_VOID *)(HI_UL)model_mem.virtual_addr);
    hi_ivp_deinit();
    HI_MPI_SYS_Exit();
    return 0;
}
