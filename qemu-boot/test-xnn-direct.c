/*
 * Direct IVE XNN CNN Inference — bypasses libivp.a entirely
 *
 * Issues raw ioctls to /dev/ive to load an OMS model and run inference
 * using the XNN CNN accelerator hardware. Proves we can use the CNN
 * engine without the vendor IVP framework.
 *
 * Ioctl sequence reverse-engineered from LD_PRELOAD hook on real EV300.
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
 * Run: killall majestic; ./test-xnn-direct model.oms input.y4m
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
#include <errno.h>

/* mmap/munmap shim — uclibc ABI compat (uint32_t off → SYS_mmap2) */
#define PROT_READ   0x1
#define PROT_WRITE  0x2
#define MAP_SHARED  0x01
#define MAP_PRIVATE 0x02
#define MAP_FAILED  ((void *)-1)
int munmap(void *addr, size_t len);

void *mmap(void *start, size_t len, int prot, int flags, int fd, uint32_t off) {
    return (void *)syscall(SYS_mmap2, start, len, prot, flags, fd, off >> 12);
}

/* uclibc→musl string shims */
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

/* ioctl commands (reverse-engineered) */
#define IVE_IOC_SVP_INIT       0x8010463b
#define IVE_IOC_OPEN_DEV       0x801046c8
#define IVE_IOC_CLOSE          0x46c9
#define IVE_IOC_ENABLE         0x46ca
#define IVE_IOC_LOADMODEL      0xc8a04636
#define IVE_IOC_FORWARD_SLICE  0xc9704638
#define IVE_IOC_QUERY          0x463c
#define IVE_IOC_FINISH         0x463d

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

/*
 * OMS model has internal segment offsets. The kernel parses the header at
 * the beginning. We load the whole file, then pass segment pointers.
 *
 * From captured traffic, the OMS contains two models at different offsets.
 * model_mem points to the segment start (not necessarily file start).
 * The svp_alg_load_model() in libivp.a parses the OMS header to find them.
 *
 * For now, we'll use the HI_MPI_SYS_* API for memory management and
 * the private mpi_ive_xnn_loadmodel from libive.a directly.
 */

/* Declare the private XNN functions from libive.a/libive.so */
extern HI_S32 mpi_ive_xnn_loadmodel(void *model_mem, void *tmp_mem, void *model_params);
/*
 * mpi_ive_xnn_forward_slice — 8 args (r0-r3 + 4 on stack):
 *   r0: HI_S32 *handle        — IVE handle (output)
 *   r1: xnn_blob_t *src       — input blob (48 bytes)
 *   r2: xnn_blob_t *dst       — output blob (kernel fills)
 *   r3: model_params *model   — loaded model (2160 bytes)
 *  [sp+0]:  xnn_mem_t *tmp    — temp buffer (phys+virt+size)
 *  [sp+4]:  xnn_blob_t *report— output report/detection blob
 *  [sp+8]:  void *ctrl        — forward control (src_num, dst_num, src blob copy)
 *  [sp+12]: HI_BOOL instant   — synchronous flag (1=wait for completion)
 */
extern HI_S32 mpi_ive_xnn_forward_slice(HI_S32 *handle,
                                          void *src, void *dst,
                                          void *model,
                                          void *tmp_buf,
                                          void *report,
                                          void *ctrl,
                                          HI_BOOL instant);
extern HI_VOID mpi_ive_xnn_unloadmodel(void *model_params);
extern HI_S32 mpi_ive_xnn_get_tmpbuf_size(void *model_params, HI_U32 *size);

/* SVP alg init/exit — maps /dev/mem for XNN HW registers */
extern HI_S32 mp_ive_svp_alg_proc_init(void *phys_out, void *handle_out);
extern HI_VOID mp_ive_svp_alg_proc_exit(void);

/* Model memory descriptor (24 bytes, same as hi_ivp_mem_info) */
typedef struct {
    HI_U64 phys_addr;
    HI_U64 virt_addr;
    HI_U32 size;
    HI_U32 reserved;
} xnn_mem_t;

/* Model params (2160 bytes, filled by kernel) */
typedef struct {
    HI_U8  raw[2160];  /* opaque — kernel fills during loadmodel */
} xnn_model_params_t;

/* Accessors for model_params fields (offsets from raw struct dump on EV300) */
#define XNN_MP_TMP_BUF_SIZE(p)  (*(HI_U32 *)((p)->raw + 0x04))
#define XNN_MP_SRC_NUM(p)       (*(HI_U32 *)((p)->raw + 0x08))
#define XNN_MP_FLAGS(p)         (*(HI_U32 *)((p)->raw + 0x0C))
#define XNN_MP_BLOB_TYPE(p)     (*(HI_U32 *)((p)->raw + 0x10))
#define XNN_MP_INPUT_W(p)       (*(HI_U32 *)((p)->raw + 0x14))
#define XNN_MP_INPUT_H(p)       (*(HI_U32 *)((p)->raw + 0x18))
#define XNN_MP_INPUT_C(p)       (*(HI_U32 *)((p)->raw + 0x1C))
#define XNN_MP_INPUT_NAME(p)    ((char *)((p)->raw + 0x24))

/* Forward-slice input blob (48 bytes, from ioctl capture) */
typedef struct {
    HI_U32 blob_type;   /* +0x00: 2 = YUV420SP */
    HI_U32 width;       /* +0x04: pixels */
    HI_U32 virt_addr;   /* +0x08: userspace ptr (32-bit) */
    HI_U32 virt_hi;     /* +0x0C: upper bits or secondary addr */
    HI_U64 phys_addr;   /* +0x10: physical address */
    HI_U32 num;         /* +0x18: batch count (1) */
    HI_U32 reserved;    /* +0x1C */
    HI_U32 stride;      /* +0x20: row stride */
    HI_U32 height;      /* +0x24: pixels */
    HI_U32 channels;    /* +0x28: 3 for YUV */
    HI_U32 pad;         /* +0x2C */
} xnn_blob_t;

/* Forward-slice ioctl buffer (2416 bytes) */
typedef struct {
    HI_U32 handle;             /* +0x000: in/out handle for query */
    HI_U32 handle_hi;         /* +0x004 */
    xnn_blob_t src_blobs[16]; /* +0x008: src array (48*16 = 768) */
    xnn_blob_t dst_blobs[16]; /* +0x308: dst array (48*16 = 768) */
    /* At +0x608: model ref, ctrl, etc. - needs more RE */
    HI_U8 tail[2416 - 8 - 768*2];
} xnn_forward_buf_t;

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <model.oms> <input.y4m>\n", argv[0]);
        return 1;
    }

    /* Init MPI */
    HI_S32 ret = HI_MPI_SYS_Init();
    if (ret != HI_SUCCESS) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* SVP init — maps XNN hardware registers via /dev/mem */
    uint8_t svp_buf[16] = {0};
    uint8_t open_buf[16] = {0};
    ret = mp_ive_svp_alg_proc_init(svp_buf, open_buf);
    if (ret != 0) {
        fprintf(stderr, "svp_alg_proc_init failed: %d\n", ret);
        /* Try without — may work if majestic already initialized it */
    }
    fprintf(stderr, "SVP init OK\n");

    /* Load OMS model file into MMZ */
    FILE *mf = fopen(argv[1], "rb");
    if (!mf) { perror(argv[1]); return 1; }
    fseek(mf, 0, SEEK_END);
    long model_file_sz = ftell(mf);
    fseek(mf, 0, SEEK_SET);

    xnn_mem_t model_mem = {0};
    ret = HI_MPI_SYS_MmzAlloc(&model_mem.phys_addr,
        (HI_VOID **)&model_mem.virt_addr, "XNN_MODEL", NULL, model_file_sz);
    if (ret != HI_SUCCESS) { fprintf(stderr, "MmzAlloc model: 0x%x\n", ret); return 1; }
    model_mem.size = model_file_sz;
    fread((void *)(HI_UL)model_mem.virt_addr, 1, model_file_sz, mf);
    fclose(mf);

    /* Allocate temp buffer (2.5 MB — from captured loadmodel output) */
    xnn_mem_t tmp_mem = {0};
    HI_U32 tmp_size = 2534400; /* 0x26ac00, from kernel output */
    ret = HI_MPI_SYS_MmzAlloc(&tmp_mem.phys_addr,
        (HI_VOID **)&tmp_mem.virt_addr, "XNN_TMP", NULL, tmp_size);
    if (ret != HI_SUCCESS) { fprintf(stderr, "MmzAlloc tmp: 0x%x\n", ret); return 1; }
    tmp_mem.size = tmp_size;

    /* Load model 1 (backbone) — OMS offset 0x50 (after 0x50-byte header) */
    /* The OMS header at offset 0x24 contains the total payload size */
    uint8_t *oms_data = (uint8_t *)(HI_UL)model_mem.virt_addr;

    /* Parse OMS header to find segment offsets */
    /* From captured traffic: model1 size=1034208, model2 offset=0x93230 size=431616 */
    /* The header at 0x24 has total size, at 0x48 has segment 2 offset */
    uint32_t seg1_offset = 0x50; /* OMS header size */
    uint32_t seg1_size = *(uint32_t *)(oms_data + 0x24) - 0x50 + 0x30;

    /* For now, use the known offsets from our RE */
    /* Segment 1: offset 0x50, size = 0xFC810 - 0x50 = 1034176 */
    /* Segment 2: offset 0x93230, to end */
    uint32_t total_payload = *(uint32_t *)(oms_data + 0x24);
    fprintf(stderr, "OMS total payload: %u bytes\n", total_payload);

    /*
     * The OMS file has a 0x50-byte outer header. svp_alg_load_model skips it
     * and passes offset 0x50 as the model segment start to mpi_ive_xnn_loadmodel.
     *
     * OMS header layout (from RE):
     *   0x04: total file size
     *   0x24: payload size (file_size - 0x20)
     *   0x34: segment 1 weight data size
     *   0x38: tmp buffer size needed (2534400)
     *   0x48: segment 2 offset from file start
     *
     * Segment 1 starts at file offset 0x50.
     * Segment 2 starts at file offset [0x48] + 0x20.
     */
    uint8_t *oms = (uint8_t *)(HI_UL)model_mem.virt_addr;
    uint32_t seg2_off_raw = *(uint32_t *)(oms + 0x48);

    /* Model 1 (backbone): skip 0x50-byte header */
    xnn_mem_t seg1_mem = {
        .phys_addr = model_mem.phys_addr + 0x50,
        .virt_addr = model_mem.virt_addr + 0x50,
        .size = model_file_sz - 0x50
    };

    xnn_model_params_t model1_params;
    memset(&model1_params, 0, sizeof(model1_params));

    fprintf(stderr, "Loading model 1 (backbone, offset 0x50, %u bytes)...\n", seg1_mem.size);
    ret = mpi_ive_xnn_loadmodel(&seg1_mem, &tmp_mem, &model1_params);
    fprintf(stderr, "mpi_ive_xnn_loadmodel[0]: ret=0x%x\n", ret);

    if (ret != 0) {
        fprintf(stderr, "LOADMODEL[0] failed: 0x%x\n", ret);
        goto cleanup;
    }

    /* Dump first 128 bytes of model_params to find correct offsets */
    fprintf(stderr, "Model 1 params raw dump:\n");
    for (int i = 0; i < 128; i += 16) {
        fprintf(stderr, "  %04x:", i);
        for (int j = 0; j < 16; j++) fprintf(stderr, " %02x", model1_params.raw[i+j]);
        fprintf(stderr, "  ");
        for (int j = 0; j < 16; j++) {
            uint8_t c = model1_params.raw[i+j];
            fprintf(stderr, "%c", (c >= 32 && c < 127) ? c : '.');
        }
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "Model 1 loaded: w=%u h=%u c=%u type=%u name=\"%s\"\n",
            XNN_MP_INPUT_W(&model1_params), XNN_MP_INPUT_H(&model1_params),
            XNN_MP_INPUT_C(&model1_params), XNN_MP_BLOB_TYPE(&model1_params),
            XNN_MP_INPUT_NAME(&model1_params));

    /* Model 2 (refinement head): offset from OMS header */
    uint32_t seg2_file_off = seg2_off_raw + 0x20;
    xnn_mem_t seg2_mem = {
        .phys_addr = model_mem.phys_addr + seg2_file_off,
        .virt_addr = model_mem.virt_addr + seg2_file_off,
        .size = model_file_sz - seg2_file_off
    };

    xnn_model_params_t model2_params;
    memset(&model2_params, 0, sizeof(model2_params));

    fprintf(stderr, "Loading model 2 (head, offset 0x%x, %u bytes)...\n",
            seg2_file_off, seg2_mem.size);
    ret = mpi_ive_xnn_loadmodel(&seg2_mem, &tmp_mem, &model2_params);
    fprintf(stderr, "mpi_ive_xnn_loadmodel[1]: ret=0x%x\n", ret);

    if (ret != 0) {
        fprintf(stderr, "LOADMODEL[1] failed: 0x%x (non-fatal, backbone-only mode)\n", ret);
    } else {
        fprintf(stderr, "Model 2 loaded: input=%ux%ux%u type=%u idx=%u name=\"%s\"\n",
                XNN_MP_INPUT_W(&model2_params), XNN_MP_INPUT_H(&model2_params),
                XNN_MP_INPUT_C(&model2_params), XNN_MP_BLOB_TYPE(&model2_params),
                (*(HI_U32 *)(model2_params.raw + 0x00)), XNN_MP_INPUT_NAME(&model2_params));
    }

    /* Parse input Y4M */
    FILE *fp = fopen(argv[2], "rb");
    if (!fp) { perror(argv[2]); return 1; }
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
    HI_U32 stride = (W + 15) & ~15;
    HI_U32 y_size = stride * H;
    HI_U32 uv_size = stride * H / 2;

    /* Allocate frame buffer in MMZ (Y + UV planes) */
    HI_U64 frame_phys, frame_virt;
    ret = HI_MPI_SYS_MmzAlloc(&frame_phys, (HI_VOID **)&frame_virt,
        "XNN_FRAME", NULL, y_size + uv_size);
    if (ret != HI_SUCCESS) { fprintf(stderr, "MmzAlloc frame: 0x%x\n", ret); goto cleanup; }

    /* Fill UV with neutral chroma */
    memset((uint8_t *)(HI_UL)frame_virt + y_size, 0x80, uv_size);

    /* Allocate output report blob in MMZ
     * From captured forward_slice arg5: {type=0, size=0xa0=160, w=0x28=40, h=0x17=23, c=0x3f=63}
     * Total output = w * h * c = 40 * 23 * 63 = 57960 bytes (round up to 64K) */
    HI_U64 report_phys, report_virt;
    HI_U32 report_size = 65536;
    ret = HI_MPI_SYS_MmzAlloc(&report_phys, (HI_VOID **)&report_virt,
        "XNN_REPORT", NULL, report_size);
    if (ret != HI_SUCCESS) { fprintf(stderr, "MmzAlloc report: 0x%x\n", ret); goto cleanup; }
    memset((void *)(HI_UL)report_virt, 0, report_size);

    /* Build report blob (matching captured layout) */
    xnn_blob_t report_blob;
    memset(&report_blob, 0, sizeof(report_blob));
    report_blob.blob_type = 0;
    report_blob.width = 160;     /* 0xa0 — might be total size, not width */
    report_blob.virt_addr = (uint32_t)report_virt;
    report_blob.virt_hi = 0;
    report_blob.phys_addr = report_phys;
    report_blob.num = 1;
    report_blob.stride = 40;     /* 0x28 */
    report_blob.height = 23;     /* 0x17 */
    report_blob.channels = 63;   /* 0x3f */

    uint8_t *frame_data = malloc(frame_sz);
    long long t_total = 0;
    int nframes = 0;

    fprintf(stderr, "Processing %dx%d frames...\n", W, H);

    while (read_y4m_frame(fp, frame_data, frame_sz, is_y4m)) {
        /* Copy Y plane */
        uint8_t *dst = (uint8_t *)(HI_UL)frame_virt;
        for (int y = 0; y < H; y++)
            memcpy(dst + y * stride, frame_data + y * W, W);
        HI_MPI_SYS_MmzFlushCache(frame_phys, (HI_VOID *)(HI_UL)frame_virt, y_size + uv_size);

        /* Build src blob (48 bytes, matches captured layout exactly) */
        xnn_blob_t src_blob;
        memset(&src_blob, 0, sizeof(src_blob));
        src_blob.blob_type = XNN_MP_BLOB_TYPE(&model1_params);
        src_blob.width = W;
        src_blob.virt_addr = (uint32_t)frame_virt;
        src_blob.virt_hi = 0;
        src_blob.phys_addr = frame_phys;
        src_blob.num = 1;
        src_blob.stride = stride;
        src_blob.height = H;
        src_blob.channels = XNN_MP_INPUT_C(&model1_params);

        /* Dst blob — kernel fills during forward */
        xnn_blob_t dst_blob;
        memset(&dst_blob, 0, sizeof(dst_blob));

        /* Output report blob — detection results
         * From captured arg5: type=0, size=160(0xa0), w=40(0x28), h=23(0x17), c=63(0x3f)
         * This is the network's output tensor — pre-allocated in MMZ */

        /* Forward control — from capture: starts with {src_num=1, dst_num=1, 0, 1}
         * then contains a copy of the src blob. Total ~64+ bytes. */
        uint8_t ctrl[128];
        memset(ctrl, 0, sizeof(ctrl));
        uint32_t *ctrl32 = (uint32_t *)ctrl;
        ctrl32[0] = 1;  /* src_num */
        ctrl32[1] = 1;  /* dst_num */
        ctrl32[2] = 0;
        ctrl32[3] = 1;  /* ??? (flags) */
        /* Copy src blob into ctrl at offset 16 */
        memcpy(ctrl + 16, &src_blob, sizeof(src_blob));

        HI_S32 handle = 0;

        long long t0 = usec_now();
        ret = mpi_ive_xnn_forward_slice(&handle,
                                         &src_blob, &dst_blob,
                                         &model1_params,
                                         &tmp_mem,      /* temp buffer */
                                         &report_blob,  /* output report */
                                         ctrl,          /* forward ctrl */
                                         HI_TRUE);      /* instant */
        long long dt = usec_now() - t0;
        t_total += dt;

        if (ret != 0) {
            fprintf(stderr, "FRAME %d: forward_slice failed: 0x%x\n", nframes, ret);
            if (nframes == 0) {
                fprintf(stderr, "(first failure — stopping)\n");
                break;
            }
            continue;
        }

        if (nframes == 0) {
            fprintf(stderr, "FRAME 0: forward_slice OK, handle=%d (%.1f ms)\n",
                    handle, dt / 1000.0);
            /* Dump output blobs to check if inference actually ran */
            HI_MPI_SYS_MmzFlushCache(report_phys, (void *)(HI_UL)report_virt, 256);
            fprintf(stderr, "  report blob first 64 bytes:\n  ");
            uint8_t *rp = (uint8_t *)(HI_UL)report_virt;
            for (int j = 0; j < 64; j++) fprintf(stderr, "%02x ", rp[j]);
            fprintf(stderr, "\n  dst blob:\n  ");
            /* Check dst_blob fields */
            fprintf(stderr, "type=%u w=%u h=%u c=%u stride=%u phys=0x%llx\n",
                    dst_blob.blob_type, dst_blob.width, dst_blob.height,
                    dst_blob.channels, dst_blob.stride,
                    (unsigned long long)dst_blob.phys_addr);
        }

        nframes++;
        if (nframes >= 10) break; /* limit for initial test */
    }

    fclose(fp);

    if (nframes > 0) {
        fprintf(stderr, "\n=== Direct XNN Benchmark (%d frames, %dx%d) ===\n", nframes, W, H);
        fprintf(stderr, "  Inference: %lld ms (%.1f ms/frame)\n",
                t_total / 1000, (double)t_total / 1000 / nframes);
        fprintf(stderr, "  Throughput: %.1f fps\n",
                1000000.0 * nframes / t_total);
    }

cleanup:
    free(frame_data);
    HI_MPI_SYS_MmzFree(frame_phys, (HI_VOID *)(HI_UL)frame_virt);
    mpi_ive_xnn_unloadmodel(&model1_params);
    HI_MPI_SYS_MmzFree(tmp_mem.phys_addr, (HI_VOID *)(HI_UL)tmp_mem.virt_addr);
    HI_MPI_SYS_MmzFree(model_mem.phys_addr, (HI_VOID *)(HI_UL)model_mem.virt_addr);
    mp_ive_svp_alg_proc_exit();
    HI_MPI_SYS_Exit();
    return 0;
}
