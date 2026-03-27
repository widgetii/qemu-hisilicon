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

/* Extended op registers */
#define IVE_EXT_CTRL    0x040C
#define IVE_EXT_COEFF0  0x0414
#define IVE_EXT_COEFF1  0x0418
#define IVE_EXT_MODE    0x0484

/* Operation types (from OP_TYPE register) */
#define IVE_OP_DMA      0
#define IVE_OP_SAD      1
#define IVE_OP_CCL      2

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

/* ── Fire handler ─────────────────────────────────────────────── */

static void ive_fire(HisiIveState *s)
{
    uint8_t op_type = s->regs[IVE_OP_TYPE / 4] & 0xFF;

    switch (op_type) {
    case IVE_OP_DMA:
        ive_op_dma(s);
        break;
    case IVE_OP_SAD:
        ive_op_sad(s);
        break;
    case IVE_OP_CCL:
        ive_op_ccl(s);
        break;
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
