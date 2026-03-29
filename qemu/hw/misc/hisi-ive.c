/*
 * HiSilicon IVE (Intelligent Video Engine) — Motion Detection MVP
 *
 * Emulates the IVE hardware accelerator for V4 SoCs (EV300/CV500).
 * Register map reverse-engineered from live EV300 hardware capture
 * (see docs/ive-registers.md).
 *
 * MVP scope: DMA, SAD (with threshold), CCL operations — enough
 * for majestic's HI_IVS_MD_Process() motion detection pipeline.
 *
 * Copyright (c) 2026 OpenIPC / Dmitry Ilyin
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "system/dma.h"

#define TYPE_HISI_IVE "hisi-ive"
OBJECT_DECLARE_SIMPLE_TYPE(HisiIveState, HISI_IVE)

/* Register offsets (from live hardware capture) */
#define IVE_CTRL        0x0004
#define IVE_SW_FIRE     0x0008
#define IVE_OP_DESC     0x0010
#define IVE_CMD_READY   0x0014
#define IVE_CMD_DONE    0x0018
#define IVE_OP_PARAMS   0x001C
#define IVE_CFG0        0x002C
#define IVE_STATUS      0x003C
#define IVE_FRAME_CNT   0x0040
#define IVE_TOTAL_OPS   0x0044
#define IVE_OP_ACTIVE   0x0048
#define IVE_BLEND_WTS   0x005C
#define IVE_IRQ_MASK    0x0060
#define IVE_SAD_PARAMS  0x0064
#define IVE_HW_ID       0x0080
#define IVE_HW_VER0     0x0084
#define IVE_HW_VER1     0x0088
#define IVE_HW_CAP      0x0090

/* Command parameter registers */
#define IVE_OP_TYPE     0x0104
#define IVE_DIMENSIONS  0x0108
#define IVE_BLOCK_CFG   0x010C
#define IVE_SRC1_ADDR   0x0110
#define IVE_DST1_ADDR   0x0114
#define IVE_SRC2_ADDR   0x0118
#define IVE_DST2_ADDR   0x011C
#define IVE_STRIDES_0   0x0128
#define IVE_STRIDES_1   0x012C
#define IVE_STRIDES_2   0x0130
#define IVE_STRIDES_3   0x0134
#define IVE_THRESH_VAL  0x0138
#define IVE_CCL_CFG0    0x0160
#define IVE_CCL_CFG1    0x0164

/* Interrupt */
#define IVE_IRQ_STATUS  0x025C
#define IVE_IRQ_FRAME_DONE  (1 << 24)

/* Additional command registers */
#define IVE_BLEND_WTS   0x005C      /* weighted add X/Y (Q0.16 packed) */
#define IVE_THRESH_LO   0x013C      /* threshold low value */
#define IVE_THRESH_HI   0x0140      /* threshold high value */
#define IVE_THRESH_MIN  0x0144      /* threshold output min */
#define IVE_THRESH_MID  0x0148      /* threshold output mid */
#define IVE_THRESH_MAX  0x014C      /* threshold output max */
#define IVE_FILTER_MASK 0x0200      /* 25-byte filter mask (5×5 kernel) */
#define IVE_MAP_LUT     0x0300      /* 256-byte LUT for map operation */
#define IVE_NCC_OUTPUT  0x0180      /* NCC result output address */

/* GMM2 registers */
#define IVE_GMM2_MODEL  0x0190      /* model buffer physical address */
#define IVE_GMM2_FG     0x0194      /* foreground output address */
#define IVE_GMM2_BG     0x0198      /* background output address */
#define IVE_GMM2_FACTOR 0x019C      /* factor image address */
#define IVE_GMM2_NMODEL 0x01A0      /* number of Gaussians per pixel (1-5) */
#define IVE_GMM2_VARMAX 0x01A4      /* max variance (Q9.7) */
#define IVE_GMM2_VARMIN 0x01A8      /* min variance (Q9.7) */
#define IVE_GMM2_SNS    0x01AC      /* sensitivity factor */
#define IVE_GMM2_FTHR   0x01B0      /* frequency threshold */
#define IVE_GMM2_FINIT  0x01B4      /* frequency init value */
#define IVE_GMM2_FADD   0x01B8      /* frequency add factor */
#define IVE_GMM2_FREDU  0x01BC      /* frequency reduction factor */
#define IVE_GMM2_LTHR   0x01C0      /* life threshold */
#define IVE_GMM2_VRATE  0x01C4      /* variance update rate */

/* Extended op registers */
#define IVE_EXT_CTRL    0x040C
#define IVE_EXT_COEFF0  0x0414
#define IVE_EXT_COEFF1  0x0418
#define IVE_EXT_MODE    0x0484

/* Operation types (from OP_TYPE register) */
#define IVE_OP_DMA      0
#define IVE_OP_SAD      1
#define IVE_OP_CCL      2
#define IVE_OP_SUB      3
#define IVE_OP_ADD      4
#define IVE_OP_AND      5
#define IVE_OP_OR       6
#define IVE_OP_XOR      7
#define IVE_OP_THRESH   8
#define IVE_OP_HIST     9
#define IVE_OP_FILTER   10
#define IVE_OP_SOBEL    11
#define IVE_OP_DILATE   12
#define IVE_OP_ERODE    13
#define IVE_OP_INTEG    14
#define IVE_OP_MAP      15
#define IVE_OP_NCC      16
#define IVE_OP_GMM2     17
#define IVE_OP_THRESH_S16   18
#define IVE_OP_THRESH_U16   19
#define IVE_OP_16BIT_TO_8BIT 20

/* CCL output structure matches hi_ive.h IVE_CCBLOB_S */
#define IVE_MAX_REGION_NUM  254
#define CCBLOB_SIZE         (4 + 2 + 2 + IVE_MAX_REGION_NUM * 12)

#define IVE_MMIO_SIZE   0x10000
#define IVE_NREGS       (IVE_MMIO_SIZE / 4)

struct HisiIveState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[IVE_NREGS];
    uint32_t total_ops;
};

/* ── DMA operation ────────────────────────────────────────────── */

static void ive_op_dma(HisiIveState *s)
{
    uint32_t src = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    uint32_t strides = s->regs[IVE_STRIDES_0 / 4];
    uint16_t src_stride = strides >> 16;
    uint16_t dst_stride = strides & 0xFFFF;
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t height = (dim >> 16) & 0xFFF;
    uint16_t width = dim & 0xFFF;

    if (!src || !dst || !width || !height) {
        return;
    }
    if (!src_stride) {
        src_stride = width;
    }
    if (!dst_stride) {
        dst_stride = width;
    }

    uint8_t line[4096];
    uint16_t copy_w = MIN(width, sizeof(line));

    for (uint16_t y = 0; y < height; y++) {
        dma_memory_read(&address_space_memory,
                        src + (uint64_t)y * src_stride,
                        line, copy_w, MEMTXATTRS_UNSPECIFIED);
        dma_memory_write(&address_space_memory,
                         dst + (uint64_t)y * dst_stride,
                         line, copy_w, MEMTXATTRS_UNSPECIFIED);
    }
}

/* ── SAD operation (with optional threshold) ──────────────────── */

static void ive_op_sad(HisiIveState *s)
{
    uint32_t src1 = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t src2 = s->regs[IVE_SRC2_ADDR / 4];
    uint32_t dst  = s->regs[IVE_DST1_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint32_t strides1 = s->regs[IVE_STRIDES_1 / 4];
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint32_t thresh_reg = s->regs[IVE_THRESH_VAL / 4];
    uint16_t threshold = (thresh_reg >> 8) & 0xFF;  /* extract from packed */

    uint16_t height = (dim >> 16) & 0xFFF;
    uint16_t width = dim & 0xFFF;
    uint16_t src1_stride = (strides0 >> 16) ? (strides0 >> 16) : width;
    uint16_t src2_stride = (strides1 >> 16) ? (strides1 >> 16) : width;

    if (!src1 || !src2 || !dst || !width || !height) {
        return;
    }

    /* SAD with 4×4 macro blocks, threshold output */
    uint16_t bw = width / 4;
    uint16_t bh = height / 4;
    uint16_t dst_stride = bw;

    uint8_t *frame1 = g_malloc(width * height);
    uint8_t *frame2 = g_malloc(width * height);
    uint8_t *out = g_malloc0(bw * bh);

    /* Read both frames */
    for (uint16_t y = 0; y < height; y++) {
        dma_memory_read(&address_space_memory,
                        src1 + (uint64_t)y * src1_stride,
                        frame1 + y * width, width,
                        MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory,
                        src2 + (uint64_t)y * src2_stride,
                        frame2 + y * width, width,
                        MEMTXATTRS_UNSPECIFIED);
    }

    /* Compute SAD per 4×4 block */
    for (uint16_t by = 0; by < bh; by++) {
        for (uint16_t bx = 0; bx < bw; bx++) {
            uint32_t sad = 0;
            for (int dy = 0; dy < 4; dy++) {
                for (int dx = 0; dx < 4; dx++) {
                    int y = by * 4 + dy;
                    int x = bx * 4 + dx;
                    if (y < height && x < width) {
                        int a = frame1[y * width + x];
                        int b = frame2[y * width + x];
                        sad += abs(a - b);
                    }
                }
            }
            /* Threshold mode: binary output */
            out[by * bw + bx] = (sad > threshold) ? 255 : 0;
        }
    }

    /* Write output */
    for (uint16_t y = 0; y < bh; y++) {
        dma_memory_write(&address_space_memory,
                         dst + (uint64_t)y * dst_stride,
                         out + y * bw, bw,
                         MEMTXATTRS_UNSPECIFIED);
    }

    g_free(frame1);
    g_free(frame2);
    g_free(out);
}

/* ── CCL operation (Connected Component Labeling) ─────────────── */

/*
 * Output format: IVE_CCBLOB_S (from hi_ive.h)
 *   u32 u32CurAreaThr;
 *   s8  s8LabelStatus;  (0=ok, -1=overflow)
 *   u8  u8RegionNum;
 *   u16 reserved;
 *   struct { u32 area; u16 left,top,right,bottom; } astRegion[254];
 */

static void ive_op_ccl(HisiIveState *s)
{
    uint32_t src = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t height = (dim >> 16) & 0xFFF;
    uint16_t width = dim & 0xFFF;
    uint32_t area_thr = (s->regs[IVE_CCL_CFG0 / 4] >> 16) & 0xFFFF;

    if (!src || !dst || !width || !height) {
        return;
    }
    if (area_thr == 0) {
        area_thr = 16;  /* default: 4×4 block area */
    }

    uint8_t *img = g_malloc(width * height);
    int16_t *labels = g_malloc0(width * height * sizeof(int16_t));

    /* Read binary image */
    for (uint16_t y = 0; y < height; y++) {
        dma_memory_read(&address_space_memory,
                        src + (uint64_t)y * width,
                        img + y * width, width,
                        MEMTXATTRS_UNSPECIFIED);
    }

    /* Two-pass CCL with union-find */
    int16_t *parent = g_malloc(IVE_MAX_REGION_NUM * sizeof(int16_t));
    int next_label = 1;

    /* Find root with path compression */
    #define FIND(x) ({ \
        int16_t _r = (x); \
        while (parent[_r] != _r) _r = parent[_r]; \
        int16_t _i = (x); \
        while (parent[_i] != _r) { int16_t _t = parent[_i]; parent[_i] = _r; _i = _t; } \
        _r; \
    })

    #define UNION(a, b) do { \
        int16_t _ra = FIND(a), _rb = FIND(b); \
        if (_ra != _rb) parent[_ra] = _rb; \
    } while (0)

    for (int i = 0; i < IVE_MAX_REGION_NUM; i++) {
        parent[i] = i;
    }

    /* Pass 1: label and merge */
    for (uint16_t y = 0; y < height; y++) {
        for (uint16_t x = 0; x < width; x++) {
            if (img[y * width + x] == 0) {
                continue;
            }
            int16_t left = (x > 0) ? labels[y * width + x - 1] : 0;
            int16_t up   = (y > 0) ? labels[(y - 1) * width + x] : 0;

            if (left && up) {
                labels[y * width + x] = left;
                UNION(left, up);
            } else if (left) {
                labels[y * width + x] = left;
            } else if (up) {
                labels[y * width + x] = up;
            } else {
                if (next_label < IVE_MAX_REGION_NUM) {
                    labels[y * width + x] = next_label++;
                }
            }
        }
    }

    /* Pass 2: flatten labels, compute bounding boxes */
    uint32_t area[IVE_MAX_REGION_NUM];
    uint16_t left[IVE_MAX_REGION_NUM], top[IVE_MAX_REGION_NUM];
    uint16_t right[IVE_MAX_REGION_NUM], bottom[IVE_MAX_REGION_NUM];

    memset(area, 0, sizeof(area));
    for (int i = 0; i < IVE_MAX_REGION_NUM; i++) {
        left[i] = width;
        top[i] = height;
        right[i] = 0;
        bottom[i] = 0;
    }

    for (uint16_t y = 0; y < height; y++) {
        for (uint16_t x = 0; x < width; x++) {
            int16_t l = labels[y * width + x];
            if (l == 0) {
                continue;
            }
            l = FIND(l);
            if (l >= IVE_MAX_REGION_NUM) {
                continue;
            }
            area[l]++;
            if (x < left[l])   left[l]   = x;
            if (y < top[l])    top[l]    = y;
            if (x > right[l])  right[l]  = x;
            if (y > bottom[l]) bottom[l] = y;
        }
    }

    #undef FIND
    #undef UNION

    /* Build IVE_CCBLOB_S output */
    uint8_t blob[CCBLOB_SIZE];
    memset(blob, 0, sizeof(blob));

    uint8_t region_num = 0;
    uint8_t *p = blob + 8;  /* skip header (8 bytes) */

    for (int i = 1; i < next_label && region_num < IVE_MAX_REGION_NUM; i++) {
        if (area[i] >= area_thr) {
            /* u32 area */
            p[0] = area[i] & 0xFF;
            p[1] = (area[i] >> 8) & 0xFF;
            p[2] = (area[i] >> 16) & 0xFF;
            p[3] = (area[i] >> 24) & 0xFF;
            /* u16 left */
            p[4] = left[i] & 0xFF;
            p[5] = (left[i] >> 8) & 0xFF;
            /* u16 top */
            p[6] = top[i] & 0xFF;
            p[7] = (top[i] >> 8) & 0xFF;
            /* u16 right */
            p[8] = right[i] & 0xFF;
            p[9] = (right[i] >> 8) & 0xFF;
            /* u16 bottom */
            p[10] = bottom[i] & 0xFF;
            p[11] = (bottom[i] >> 8) & 0xFF;
            p += 12;
            region_num++;
        }
    }

    /* Header: u32 curAreaThr, s8 labelStatus, u8 regionNum, u16 reserved */
    blob[0] = area_thr & 0xFF;
    blob[1] = (area_thr >> 8) & 0xFF;
    blob[2] = (area_thr >> 16) & 0xFF;
    blob[3] = (area_thr >> 24) & 0xFF;
    blob[4] = 0;           /* s8LabelStatus = OK */
    blob[5] = region_num;  /* u8RegionNum */
    blob[6] = 0;           /* reserved */
    blob[7] = 0;

    dma_memory_write(&address_space_memory, dst,
                     blob, sizeof(blob), MEMTXATTRS_UNSPECIFIED);

    g_free(img);
    g_free(labels);
    g_free(parent);
}

/* ── Helper: read two frames from guest memory ────────────────── */

static void ive_read_frames(HisiIveState *s, uint8_t **f1, uint8_t **f2,
                            uint16_t *pw, uint16_t *ph)
{
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t w = dim & 0xFFF, h = (dim >> 16) & 0xFFF;
    uint32_t src1 = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t src2 = s->regs[IVE_SRC2_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t s1 = (strides0 >> 16) ? (strides0 >> 16) : w;
    uint16_t s2 = s->regs[IVE_STRIDES_1 / 4] >> 16;
    if (!s2) s2 = w;
    *pw = w; *ph = h;
    *f1 = g_malloc(w * h);
    if (f2) *f2 = g_malloc(w * h); else if (f2) *f2 = NULL;
    for (uint16_t y = 0; y < h; y++) {
        dma_memory_read(&address_space_memory, src1 + (uint64_t)y * s1,
                        *f1 + y * w, w, MEMTXATTRS_UNSPECIFIED);
        if (f2 && src2) {
            dma_memory_read(&address_space_memory, src2 + (uint64_t)y * s2,
                            *f2 + y * w, w, MEMTXATTRS_UNSPECIFIED);
        }
    }
}

static void ive_write_frame(HisiIveState *s, const uint8_t *data,
                            uint16_t w, uint16_t h)
{
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t ds = (strides0 & 0xFFFF) ? (strides0 & 0xFFFF) : w;
    for (uint16_t y = 0; y < h; y++) {
        dma_memory_write(&address_space_memory, dst + (uint64_t)y * ds,
                         data + y * w, w, MEMTXATTRS_UNSPECIFIED);
    }
}

/* ── Phase 1: Pixel-wise operations ──────────────────────────── */

static void ive_op_sub(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1, *f2;
    ive_read_frames(s, &f1, &f2, &w, &h);
    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        out[i] = (uint8_t)abs((int)f1[i] - (int)f2[i]);
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(f2); g_free(out);
}

static void ive_op_add(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1, *f2;
    ive_read_frames(s, &f1, &f2, &w, &h);
    uint32_t wts = s->regs[IVE_BLEND_WTS / 4];
    uint16_t wx = wts & 0xFFFF, wy = wts >> 16;
    if (!wx && !wy) { wx = 32768; wy = 32768; } /* default 50/50 */
    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        int val = ((int)f1[i] * wx + (int)f2[i] * wy) >> 16;
        out[i] = (val > 255) ? 255 : val;
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(f2); g_free(out);
}

static void ive_op_bitwise(HisiIveState *s, int op)
{
    uint16_t w, h;
    uint8_t *f1, *f2;
    ive_read_frames(s, &f1, &f2, &w, &h);
    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        switch (op) {
        case IVE_OP_AND: out[i] = f1[i] & f2[i]; break;
        case IVE_OP_OR:  out[i] = f1[i] | f2[i]; break;
        case IVE_OP_XOR: out[i] = f1[i] ^ f2[i]; break;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(f2); g_free(out);
}

/* ── Phase 2: Threshold + Histogram ──────────────────────────── */

static void ive_op_thresh(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t thr_reg = s->regs[IVE_THRESH_VAL / 4];
    uint8_t thr = (thr_reg >> 8) & 0xFF;
    uint8_t minv = s->regs[IVE_THRESH_MIN / 4] & 0xFF;
    uint8_t maxv = s->regs[IVE_THRESH_MAX / 4] & 0xFF;
    if (!maxv) maxv = 255;
    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        /* Default: binary threshold */
        out[i] = (f1[i] > thr) ? maxv : minv;
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

static void ive_op_hist(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t hist[256] = {0};
    for (int i = 0; i < w * h; i++) {
        hist[f1[i]]++;
    }
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    dma_memory_write(&address_space_memory, dst,
                     hist, sizeof(hist), MEMTXATTRS_UNSPECIFIED);
    g_free(f1);
}

/* ── Phase 2b: 16-bit Threshold + Conversion ─────────────────── */

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* Read S16 frame from guest memory (src1, 2 bytes per pixel) */
static int16_t *ive_read_frame_s16(HisiIveState *s, uint16_t *pw, uint16_t *ph)
{
    uint32_t dim = s->regs[IVE_DIMENSIONS / 4];
    uint16_t w = dim & 0xFFF, h = (dim >> 16) & 0xFFF;
    uint32_t src = s->regs[IVE_SRC1_ADDR / 4];
    uint32_t strides0 = s->regs[IVE_STRIDES_0 / 4];
    uint16_t stride = (strides0 >> 16) ? (strides0 >> 16) : (w * 2);
    *pw = w; *ph = h;
    int16_t *f = g_malloc(w * h * 2);
    for (uint16_t y = 0; y < h; y++) {
        dma_memory_read(&address_space_memory, src + (uint64_t)y * stride,
                        f + y * w, w * 2, MEMTXATTRS_UNSPECIFIED);
    }
    return f;
}

/* Read U16 frame from guest memory */
static uint16_t *ive_read_frame_u16(HisiIveState *s, uint16_t *pw, uint16_t *ph)
{
    return (uint16_t *)ive_read_frame_s16(s, pw, ph);
}

/*
 * Thresh_S16: S16 input → U8 output
 * Modes (from SDK hi_ive.h IVE_THRESH_S16_MODE_E):
 *   0: S16→S8  min/mid/max
 *   1: S16→S8  min/ori/max
 *   2: S16→U8  min/mid/max  (most useful — binary threshold on S16)
 *   3: S16→U8  min/ori/max
 *
 * Uses same threshold registers as Thresh:
 *   IVE_THRESH_VAL bits[23:8]  = low threshold (signed, as int16)
 *   IVE_THRESH_HI  bits[15:0]  = high threshold
 *   IVE_THRESH_MIN = min output val
 *   IVE_THRESH_MID = mid output val
 *   IVE_THRESH_MAX = max output val
 *   IVE_EXT_MODE   = mode (0-3)
 */
static void ive_op_thresh_s16(HisiIveState *s)
{
    uint16_t w, h;
    int16_t *f = ive_read_frame_s16(s, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF;
    int16_t lo_thr = (int16_t)(s->regs[IVE_THRESH_VAL / 4] & 0xFFFF);
    int16_t hi_thr = (int16_t)(s->regs[IVE_THRESH_HI / 4] & 0xFFFF);
    uint8_t minv = s->regs[IVE_THRESH_MIN / 4] & 0xFF;
    uint8_t midv = s->regs[IVE_THRESH_MID / 4] & 0xFF;
    uint8_t maxv = s->regs[IVE_THRESH_MAX / 4] & 0xFF;
    if (!maxv) maxv = 255;

    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        int16_t v = f[i];
        switch (mode) {
        case 0: /* S16→S8 min/mid/max */
        case 2: /* S16→U8 min/mid/max */
            out[i] = (v <= lo_thr) ? minv : (v > hi_thr) ? maxv : midv;
            break;
        case 1: /* S16→S8 min/ori/max */
        case 3: /* S16→U8 min/ori/max */
            if (v <= lo_thr) out[i] = minv;
            else if (v > hi_thr) out[i] = maxv;
            else out[i] = (mode <= 1) ? (int8_t)v : (uint8_t)clamp_i32(v, 0, 255);
            break;
        default:
            out[i] = (v <= lo_thr) ? minv : (v > hi_thr) ? maxv : midv;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f); g_free(out);
}

/*
 * Thresh_U16: U16 input → U8 output
 * Same logic as Thresh_S16 but unsigned input.
 */
static void ive_op_thresh_u16(HisiIveState *s)
{
    uint16_t w, h;
    uint16_t *f = ive_read_frame_u16(s, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF;
    uint16_t lo_thr = s->regs[IVE_THRESH_VAL / 4] & 0xFFFF;
    uint16_t hi_thr = s->regs[IVE_THRESH_HI / 4] & 0xFFFF;
    uint8_t minv = s->regs[IVE_THRESH_MIN / 4] & 0xFF;
    uint8_t midv = s->regs[IVE_THRESH_MID / 4] & 0xFF;
    uint8_t maxv = s->regs[IVE_THRESH_MAX / 4] & 0xFF;
    if (!maxv) maxv = 255;

    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        uint16_t v = f[i];
        switch (mode) {
        case 0: /* min/mid/max */
            out[i] = (v < lo_thr) ? minv : (v > hi_thr) ? maxv : midv;
            break;
        case 1: /* min/ori/max */
            if (v < lo_thr) out[i] = minv;
            else if (v > hi_thr) out[i] = maxv;
            else out[i] = (uint8_t)clamp_i32(v, 0, 255);
            break;
        default:
            out[i] = (v < lo_thr) ? minv : (v > hi_thr) ? maxv : midv;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f); g_free(out);
}

/*
 * 16BitTo8Bit: S16/U16 input → U8 output with gain/bias
 * Modes (from SDK hi_ive.h IVE_16BIT_TO_8BIT_MODE_E):
 *   0: S16→S8:       out = (val * numerator / denominator) clamped to [-128,127]
 *   1: S16→U8_ABS:   out = abs(val) * numerator / denominator, clamped to [0,255]
 *   2: S16→U8_BIAS:  out = val * numerator / denominator + bias, clamped to [0,255]
 *   3: U16→U8:       out = val * numerator / denominator, clamped to [0,255]
 *
 * Registers:
 *   IVE_EXT_MODE      = mode (0-3)
 *   IVE_THRESH_VAL[7:0]  = numerator (u8)
 *   IVE_THRESH_VAL[23:8] = denominator (u16)
 *   IVE_THRESH_MIN[7:0]  = bias (s8)
 */
static void ive_op_16bit_to_8bit(HisiIveState *s)
{
    uint16_t w, h;
    int16_t *f = ive_read_frame_s16(s, &w, &h);
    uint32_t mode = s->regs[IVE_EXT_MODE / 4] & 0xF;
    uint32_t thr_reg = s->regs[IVE_THRESH_VAL / 4];
    uint8_t numer = thr_reg & 0xFF;
    uint16_t denom = (thr_reg >> 8) & 0xFFFF;
    int8_t bias = (int8_t)(s->regs[IVE_THRESH_MIN / 4] & 0xFF);
    if (denom == 0) denom = 1;
    if (numer == 0) numer = 1;

    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        int32_t v;
        switch (mode) {
        case 0: /* S16→S8 */
            v = (int32_t)f[i] * numer / denom;
            out[i] = (uint8_t)(int8_t)clamp_i32(v, -128, 127);
            break;
        case 1: /* S16→U8_ABS */
            v = abs((int32_t)f[i]) * numer / denom;
            out[i] = (uint8_t)clamp_i32(v, 0, 255);
            break;
        case 2: /* S16→U8_BIAS */
            v = (int32_t)f[i] * numer / denom + bias;
            out[i] = (uint8_t)clamp_i32(v, 0, 255);
            break;
        case 3: /* U16→U8 */
            v = (int32_t)(uint16_t)f[i] * numer / denom;
            out[i] = (uint8_t)clamp_i32(v, 0, 255);
            break;
        default:
            v = abs((int32_t)f[i]) * numer / denom;
            out[i] = (uint8_t)clamp_i32(v, 0, 255);
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f); g_free(out);
}

/* ── Phase 3: Convolution + Morphology ───────────────────────── */

static void ive_op_filter(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    /* Read 5×5 kernel from mask registers (packed as bytes) */
    int8_t mask[25];
    for (int i = 0; i < 25; i++) {
        mask[i] = (int8_t)(s->regs[(IVE_FILTER_MASK + i) / 4] >>
                           ((i % 4) * 8));
    }
    uint8_t *out = g_malloc(w * h);
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            int sum = 0;
            for (int ky = -2; ky <= 2; ky++) {
                for (int kx = -2; kx <= 2; kx++) {
                    int sy = y + ky, sx = x + kx;
                    if (sy < 0) sy = 0;
                    if (sy >= h) sy = h - 1;
                    if (sx < 0) sx = 0;
                    if (sx >= w) sx = w - 1;
                    sum += (int)f1[sy * w + sx] * mask[(ky+2)*5 + (kx+2)];
                }
            }
            sum >>= 8; /* normalize */
            out[y * w + x] = (sum < 0) ? 0 : (sum > 255) ? 255 : sum;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

static void ive_op_sobel(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    /* Sobel 3×3: Gx and Gy, output magnitude */
    uint8_t *out = g_malloc(w * h);
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            if (y == 0 || y == h-1 || x == 0 || x == w-1) {
                out[y * w + x] = 0;
                continue;
            }
            int gx = -f1[(y-1)*w+x-1] + f1[(y-1)*w+x+1]
                     -2*f1[y*w+x-1]   + 2*f1[y*w+x+1]
                     -f1[(y+1)*w+x-1]  + f1[(y+1)*w+x+1];
            int gy = -f1[(y-1)*w+x-1] - 2*f1[(y-1)*w+x] - f1[(y-1)*w+x+1]
                     +f1[(y+1)*w+x-1]  + 2*f1[(y+1)*w+x] + f1[(y+1)*w+x+1];
            int mag = (abs(gx) + abs(gy)) >> 1;
            out[y * w + x] = (mag > 255) ? 255 : mag;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

static void ive_op_dilate(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint8_t *out = g_malloc(w * h);
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            uint8_t maxv = 0;
            for (int ky = -2; ky <= 2; ky++) {
                for (int kx = -2; kx <= 2; kx++) {
                    int sy = y + ky, sx = x + kx;
                    if (sy >= 0 && sy < h && sx >= 0 && sx < w) {
                        uint8_t v = f1[sy * w + sx];
                        if (v > maxv) maxv = v;
                    }
                }
            }
            out[y * w + x] = maxv;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

static void ive_op_erode(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint8_t *out = g_malloc(w * h);
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            uint8_t minv = 255;
            for (int ky = -2; ky <= 2; ky++) {
                for (int kx = -2; kx <= 2; kx++) {
                    int sy = y + ky, sx = x + kx;
                    if (sy >= 0 && sy < h && sx >= 0 && sx < w) {
                        uint8_t v = f1[sy * w + sx];
                        if (v < minv) minv = v;
                    }
                }
            }
            out[y * w + x] = minv;
        }
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

/* ── Phase 4: Analysis ───────────────────────────────────────── */

static void ive_op_integ(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    uint32_t *out = g_malloc0(w * h * sizeof(uint32_t));
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            uint32_t val = f1[y * w + x];
            if (x > 0) val += out[y * w + x - 1];
            if (y > 0) val += out[(y-1) * w + x];
            if (x > 0 && y > 0) val -= out[(y-1) * w + x - 1];
            out[y * w + x] = val;
        }
    }
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    dma_memory_write(&address_space_memory, dst,
                     out, w * h * sizeof(uint32_t), MEMTXATTRS_UNSPECIFIED);
    g_free(f1); g_free(out);
}

static void ive_op_map(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1;
    ive_read_frames(s, &f1, NULL, &w, &h);
    /* Read 256-byte LUT from src2 address */
    uint8_t lut[256];
    uint32_t lut_addr = s->regs[IVE_SRC2_ADDR / 4];
    dma_memory_read(&address_space_memory, lut_addr,
                    lut, sizeof(lut), MEMTXATTRS_UNSPECIFIED);
    uint8_t *out = g_malloc(w * h);
    for (int i = 0; i < w * h; i++) {
        out[i] = lut[f1[i]];
    }
    ive_write_frame(s, out, w, h);
    g_free(f1); g_free(out);
}

static void ive_op_ncc(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *f1, *f2;
    ive_read_frames(s, &f1, &f2, &w, &h);
    /* NCC: sum(a*b) / sqrt(sum(a*a) * sum(b*b)) scaled to Q1.15 */
    int64_t sum_ab = 0, sum_aa = 0, sum_bb = 0;
    for (int i = 0; i < w * h; i++) {
        sum_ab += (int64_t)f1[i] * f2[i];
        sum_aa += (int64_t)f1[i] * f1[i];
        sum_bb += (int64_t)f2[i] * f2[i];
    }
    /* Write 3 u64 values: sum_ab, sum_aa, sum_bb to dst */
    uint64_t result[3] = { sum_ab, sum_aa, sum_bb };
    uint32_t dst = s->regs[IVE_DST1_ADDR / 4];
    dma_memory_write(&address_space_memory, dst,
                     result, sizeof(result), MEMTXATTRS_UNSPECIFIED);
    g_free(f1); g_free(f2);
}

/* ── GMM2 (Gaussian Mixture Model, stateful background subtraction) ── */

/*
 * Per-pixel model: K Gaussians, each 8 bytes:
 *   Byte 0:   mean (U8)
 *   Byte 1-2: variance (U16, Q9.7)
 *   Byte 3-4: frequency (U16)
 *   Byte 5-6: life (U16)
 *   Byte 7:   reserved
 */
#define GMM2_GAUSS_SIZE 8

static void ive_op_gmm2(HisiIveState *s)
{
    uint16_t w, h;
    uint8_t *frame;
    ive_read_frames(s, &frame, NULL, &w, &h);
    int npix = w * h;

    uint8_t K = s->regs[IVE_GMM2_NMODEL / 4] & 0xFF;
    if (K < 1) K = 3;
    if (K > 5) K = 5;

    uint32_t model_addr  = s->regs[IVE_GMM2_MODEL / 4];
    uint32_t fg_addr     = s->regs[IVE_GMM2_FG / 4];
    uint32_t bg_addr     = s->regs[IVE_GMM2_BG / 4];
    uint16_t var_max     = s->regs[IVE_GMM2_VARMAX / 4] & 0xFFFF;
    uint16_t var_min     = s->regs[IVE_GMM2_VARMIN / 4] & 0xFFFF;
    uint8_t  sns_factor  = s->regs[IVE_GMM2_SNS / 4] & 0xFF;
    uint16_t freq_thr    = s->regs[IVE_GMM2_FTHR / 4] & 0xFFFF;
    uint16_t freq_init   = s->regs[IVE_GMM2_FINIT / 4] & 0xFFFF;
    uint16_t freq_add    = s->regs[IVE_GMM2_FADD / 4] & 0xFFFF;
    uint16_t freq_redu   = s->regs[IVE_GMM2_FREDU / 4] & 0xFFFF;
    uint16_t life_thr    = s->regs[IVE_GMM2_LTHR / 4] & 0xFFFF;
    uint16_t var_rate    = s->regs[IVE_GMM2_VRATE / 4] & 0xFFFF;

    /* Defaults from SDK sample */
    if (!var_max)   var_max   = (16 * 16) << 7;  /* 32768 */
    if (!var_min)   var_min   = (8 * 8) << 7;    /* 8192 */
    if (!sns_factor) sns_factor = 8;
    if (!freq_thr)  freq_thr  = 12000;
    if (!freq_init) freq_init = 20000;
    if (!freq_add)  freq_add  = 0xEF;
    if (!freq_redu) freq_redu = 0xFF00;
    if (!life_thr)  life_thr  = 5000;
    if (!var_rate)  var_rate  = 1;

    int model_size = K * GMM2_GAUSS_SIZE * npix;
    uint8_t *model = g_malloc(model_size);
    uint8_t *fg = g_malloc0(npix);
    uint8_t *bg = g_malloc0(npix);

    /* Read model from guest memory */
    dma_memory_read(&address_space_memory, model_addr,
                    model, model_size, MEMTXATTRS_UNSPECIFIED);

    for (int p = 0; p < npix; p++) {
        uint8_t pixel = frame[p];
        uint8_t *pmodel = model + p * K * GMM2_GAUSS_SIZE;
        int matched = 0;
        int best_k = 0;
        uint16_t best_freq = 0;

        /* Try to match pixel against each Gaussian */
        for (int k = 0; k < K; k++) {
            uint8_t *g = pmodel + k * GMM2_GAUSS_SIZE;
            uint8_t  mean = g[0];
            uint16_t var  = g[1] | (g[2] << 8);
            uint16_t freq = g[3] | (g[4] << 8);
            uint16_t life = g[5] | (g[6] << 8);

            if (freq == 0 && life == 0 && var == 0) {
                continue; /* uninitialized slot */
            }

            /* Match criterion: |pixel - mean| < sns_factor * sqrt(var)
             * Simplified: (pixel - mean)^2 < sns_factor^2 * var / 128 */
            int diff = (int)pixel - (int)mean;
            int diff2 = diff * diff;
            int var_real = (var > 0) ? var : var_min;
            int threshold = ((int)sns_factor * sns_factor * var_real) >> 7;

            if (diff2 < threshold) {
                /* Matched — update mean and variance */
                int new_mean = mean + (diff > 0 ? var_rate : -var_rate);
                if (new_mean < 0) new_mean = 0;
                if (new_mean > 255) new_mean = 255;
                g[0] = new_mean;

                /* Update variance */
                int new_var = var + var_rate * (diff2 - (var >> 7));
                if (new_var < var_min) new_var = var_min;
                if (new_var > var_max) new_var = var_max;
                g[1] = new_var & 0xFF;
                g[2] = (new_var >> 8) & 0xFF;

                /* Increase frequency */
                int new_freq = freq + freq_add;
                if (new_freq > 65535) new_freq = 65535;
                g[3] = new_freq & 0xFF;
                g[4] = (new_freq >> 8) & 0xFF;

                /* Reset life */
                g[5] = life_thr & 0xFF;
                g[6] = (life_thr >> 8) & 0xFF;

                matched = 1;

                /* Track best (highest freq) for background */
                if (new_freq > best_freq) {
                    best_freq = new_freq;
                    best_k = k;
                }
                break; /* first match wins */
            } else {
                /* Not matched — reduce frequency */
                int new_freq = (int)freq * freq_redu >> 16;
                g[3] = new_freq & 0xFF;
                g[4] = (new_freq >> 8) & 0xFF;

                /* Decrease life */
                if (life > 0) {
                    life--;
                    g[5] = life & 0xFF;
                    g[6] = (life >> 8) & 0xFF;
                }

                if (freq > best_freq) {
                    best_freq = freq;
                    best_k = k;
                }
            }
        }

        if (!matched) {
            /* No match — replace weakest Gaussian */
            int weakest = 0;
            uint16_t min_freq = 65535;
            for (int k = 0; k < K; k++) {
                uint8_t *g = pmodel + k * GMM2_GAUSS_SIZE;
                uint16_t freq = g[3] | (g[4] << 8);
                if (freq < min_freq) { min_freq = freq; weakest = k; }
            }
            uint8_t *g = pmodel + weakest * GMM2_GAUSS_SIZE;
            g[0] = pixel;                           /* mean = current pixel */
            g[1] = var_max & 0xFF;                  /* high initial variance */
            g[2] = (var_max >> 8) & 0xFF;
            g[3] = freq_init & 0xFF;                /* initial frequency */
            g[4] = (freq_init >> 8) & 0xFF;
            g[5] = life_thr & 0xFF;                 /* full life */
            g[6] = (life_thr >> 8) & 0xFF;
            g[7] = 0;

            /* Foreground: pixel doesn't match any stable model */
            fg[p] = 255;
        } else {
            /* Background: matched a model — check if model is stable */
            uint8_t *g = pmodel + best_k * GMM2_GAUSS_SIZE;
            uint16_t freq = g[3] | (g[4] << 8);
            fg[p] = (freq >= freq_thr) ? 0 : 255;
        }

        /* Background image: mean of best-matching Gaussian */
        bg[p] = pmodel[best_k * GMM2_GAUSS_SIZE]; /* mean */
    }

    /* Write outputs back to guest memory */
    dma_memory_write(&address_space_memory, model_addr,
                     model, model_size, MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, fg_addr,
                     fg, npix, MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(&address_space_memory, bg_addr,
                     bg, npix, MEMTXATTRS_UNSPECIFIED);

    g_free(model);
    g_free(fg);
    g_free(bg);
    g_free(frame);
}

/* ── Fire handler ─────────────────────────────────────────────── */

static void ive_fire(HisiIveState *s)
{
    uint8_t op_type = s->regs[IVE_OP_TYPE / 4] & 0xFF;

    switch (op_type) {
    case IVE_OP_DMA:    ive_op_dma(s); break;
    case IVE_OP_SAD:    ive_op_sad(s); break;
    case IVE_OP_CCL:    ive_op_ccl(s); break;
    case IVE_OP_SUB:    ive_op_sub(s); break;
    case IVE_OP_ADD:    ive_op_add(s); break;
    case IVE_OP_AND:
    case IVE_OP_OR:
    case IVE_OP_XOR:    ive_op_bitwise(s, op_type); break;
    case IVE_OP_THRESH: ive_op_thresh(s); break;
    case IVE_OP_HIST:   ive_op_hist(s); break;
    case IVE_OP_FILTER: ive_op_filter(s); break;
    case IVE_OP_SOBEL:  ive_op_sobel(s); break;
    case IVE_OP_DILATE: ive_op_dilate(s); break;
    case IVE_OP_ERODE:  ive_op_erode(s); break;
    case IVE_OP_INTEG:  ive_op_integ(s); break;
    case IVE_OP_MAP:    ive_op_map(s); break;
    case IVE_OP_NCC:    ive_op_ncc(s); break;
    case IVE_OP_GMM2:   ive_op_gmm2(s); break;
    case IVE_OP_THRESH_S16:    ive_op_thresh_s16(s); break;
    case IVE_OP_THRESH_U16:    ive_op_thresh_u16(s); break;
    case IVE_OP_16BIT_TO_8BIT: ive_op_16bit_to_8bit(s); break;
    default:
        qemu_log_mask(LOG_UNIMP, "hisi-ive: unimplemented op_type %d\n",
                      op_type);
        break;
    }

    /* Signal completion */
    s->regs[IVE_CMD_DONE / 4] = 1;
    s->regs[IVE_OP_ACTIVE / 4] = 0;
    s->regs[IVE_IRQ_STATUS / 4] |= IVE_IRQ_FRAME_DONE;
    s->total_ops++;
    s->regs[IVE_TOTAL_OPS / 4] = s->total_ops;

    /* Assert IRQ if unmasked */
    if (!(s->regs[IVE_IRQ_MASK / 4] & IVE_IRQ_FRAME_DONE)) {
        qemu_irq_raise(s->irq);
    }
}

/* ── MMIO handlers ────────────────────────────────────────────── */

static uint64_t hisi_ive_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiIveState *s = HISI_IVE(opaque);

    if (offset >= IVE_MMIO_SIZE) {
        return 0;
    }

    return s->regs[offset / 4];
}

static void hisi_ive_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    HisiIveState *s = HISI_IVE(opaque);

    if (offset >= IVE_MMIO_SIZE) {
        return;
    }

    s->regs[offset / 4] = value;

    switch (offset) {
    case IVE_SW_FIRE:
        if (value & 1) {
            s->regs[IVE_SW_FIRE / 4] = 0;  /* auto-clear */
            s->regs[IVE_CMD_DONE / 4] = 0;
            s->regs[IVE_OP_ACTIVE / 4] = 1;
            ive_fire(s);
        }
        break;

    case IVE_IRQ_STATUS:
        /* Write-one-clear for interrupt bits */
        s->regs[IVE_IRQ_STATUS / 4] &= ~value;
        if (!(s->regs[IVE_IRQ_STATUS / 4] & IVE_IRQ_FRAME_DONE)) {
            qemu_irq_lower(s->irq);
        }
        break;

    default:
        break;
    }
}

static const MemoryRegionOps hisi_ive_ops = {
    .read = hisi_ive_read,
    .write = hisi_ive_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── Device lifecycle ─────────────────────────────────────────── */

static void hisi_ive_reset(DeviceState *dev)
{
    HisiIveState *s = HISI_IVE(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->total_ops = 0;

    /* Power-on defaults (from live hardware capture) */
    s->regs[IVE_CTRL / 4]     = 0x00000006;
    s->regs[0x0034 / 4]       = 0x00313307;  /* STATIC_CFG */
    s->regs[0x0054 / 4]       = 0x00003F07;  /* IRQ_CFG */
    s->regs[IVE_IRQ_MASK / 4] = 0xFFFFFFFF;  /* all masked */
    s->regs[IVE_HW_ID / 4]    = 0x11E1A300;
    s->regs[IVE_HW_VER0 / 4]  = 0x00000001;
    s->regs[IVE_HW_VER1 / 4]  = 0x00000001;
    s->regs[IVE_HW_CAP / 4]   = 0x01AB5159;
    s->regs[0x0300 / 4]       = 0x00100000;  /* RESULT0 default */
}

static void hisi_ive_init(Object *obj)
{
    HisiIveState *s = HISI_IVE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_ive_ops, s,
                          "hisi-ive", IVE_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    /* Set power-on defaults immediately (from live hardware capture) */
    s->regs[IVE_CTRL / 4]     = 0x00000006;
    s->regs[0x0034 / 4]       = 0x00313307;
    s->regs[0x0054 / 4]       = 0x00003F07;
    s->regs[IVE_IRQ_MASK / 4] = 0xFFFFFFFF;
    s->regs[IVE_HW_ID / 4]    = 0x11E1A300;
    s->regs[IVE_HW_VER0 / 4]  = 0x00000001;
    s->regs[IVE_HW_VER1 / 4]  = 0x00000001;
    s->regs[IVE_HW_CAP / 4]   = 0x01AB5159;
    s->regs[0x0300 / 4]       = 0x00100000;
}

static void hisi_ive_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_ive_reset);
}

static const TypeInfo hisi_ive_info = {
    .name          = TYPE_HISI_IVE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiIveState),
    .instance_init = hisi_ive_init,
    .class_init    = hisi_ive_class_init,
};

static void hisi_ive_register_types(void)
{
    type_register_static(&hisi_ive_info);
}

type_init(hisi_ive_register_types)
