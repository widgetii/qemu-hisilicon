/*
 * HiSilicon NNIE — CNN inference accelerator (cv500/av300/dv300).
 *
 * Block lives at phys 0x11100000 on cv500-family SoCs; SPI 45 IRQ
 * line. Drives one channel of CNN dispatch via a 64-byte HW task
 * descriptor + a variable-length tskbuf tail.
 *
 * Dispatch sequence (matches what kernel/nnie_neo/ drives):
 *   1. Userspace builds a 64-byte task descriptor in MMZ
 *   2. Writes TASK_ADDR_LO/HI = descriptor phys
 *   3. RMW START bit 0 → 1
 *   4. HW reads descriptor, follows tsk_buf_phys to read tskbuf tail,
 *      runs inference reading from model_phys + inst_offset,
 *      writes activations to per-dst phys addresses in the tail
 *   5. Signals via IRQ_STATUS bit 0 (finish) + raises SPI 45
 *   6. Userspace clears IRQ_STATUS via IRQ_CLEAR (W1C)
 *
 * The model implements register semantics, descriptor decoding, and
 * a stub "inference" path that copies a known-good mnist score vector
 * into the dst phys — enough to exercise the kernel module's submit /
 * IRQ / completion path end-to-end. Real CNN execution (decoding the
 * .wk instruction stream, doing conv/relu/fc/softmax) is out of scope
 * for this MVP — that's a separate effort.
 *
 * Copyright (c) 2026 OpenIPC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "system/dma.h"

#define TYPE_HISI_NNIE "hisi-nnie"
OBJECT_DECLARE_SIMPLE_TYPE(HisiNnieState, HISI_NNIE)

/* Register offsets — mirror kernel/nnie_neo/nnie_hw_regs.h */
#define NNIE_REG_TASK_ADDR_LO    0x20
#define NNIE_REG_TASK_ADDR_HI    0x24
#define NNIE_REG_TIMEOUT_LO      0x28
#define NNIE_REG_TIMEOUT_HI      0x2C
#define NNIE_REG_START           0x30
#define NNIE_REG_IRQ_CFG         0x34
#define NNIE_REG_IRQ_CLEAR       0x38
#define NNIE_REG_IRQ_STATUS      0x3C
#define NNIE_REG_CFG_ERR_INFO    0x40
#define NNIE_REG_TASK_ID         0x48
#define NNIE_REG_CLK_GATE        0x50
#define NNIE_REG_OUTSTANDING     0x54
#define NNIE_REG_CHECK_SUM       0x68

#define NNIE_IRQ_FINISH          (1u << 0)
#define NNIE_IRQ_TIMEOUT         (1u << 1)
#define NNIE_IRQ_CFG_ERR         (1u << 2)
#define NNIE_IRQ_ALL             (NNIE_IRQ_FINISH | NNIE_IRQ_TIMEOUT | NNIE_IRQ_CFG_ERR)

#define NNIE_MMIO_SIZE           0x10000
#define NNIE_NREGS               (NNIE_MMIO_SIZE / 4)

struct HisiNnieState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[NNIE_NREGS];
    uint32_t task_count;
};

/* Known-good mnist scores — what real cv500 NNIE returns for the
 * vendor's inst_mnist_cycle.wk against an all-zero input. Used by the
 * stub-inference path so the test binary can assert a non-trivial
 * round-trip. */
static const int32_t mnist_scores[10] = {
    408, 412, 401, 401, 398, 412, 398, 405, 449, 401,
};

static uint32_t read_u32_at(uint64_t phys)
{
    uint32_t v = 0;
    dma_memory_read(&address_space_memory, phys, &v, sizeof(v),
                    MEMTXATTRS_UNSPECIFIED);
    return v;
}

static uint64_t read_u64_at(uint64_t phys)
{
    uint64_t v = 0;
    dma_memory_read(&address_space_memory, phys, &v, sizeof(v),
                    MEMTXATTRS_UNSPECIFIED);
    return v;
}

/* Decode the 64-byte task descriptor at task_phys, find dst[0] phys
 * from the tskbuf tail (§2: dst phys table at tskbuf+0x10), copy the
 * mnist score vector into it, set TASK_ID + finish IRQ.
 *
 * On any decode failure raise cfg_err with info=1 in bit 0 and the
 * slot index in bits[31:24] — matches the cv500 HW behaviour
 * observed during reverse engineering. */
static void hisi_nnie_execute(HisiNnieState *s)
{
    uint64_t task_phys, tsk_phys;
    uint64_t dst0_phys;
    uint32_t slot_idx;
    uint16_t trigger_mode;
    uint32_t cfg_err_slot = 0;

    task_phys = (uint64_t)s->regs[NNIE_REG_TASK_ADDR_LO / 4] |
                ((uint64_t)s->regs[NNIE_REG_TASK_ADDR_HI / 4] << 32);

    if (!task_phys) {
        cfg_err_slot = 0;
        goto cfg_err;
    }

    /* Descriptor field layout (mirrors kernel/nnie_neo/nnie_hw_task.h):
     *   +0  u16 trigger_mode
     *   +2  u16 status
     *   +4  u32 slot_idx
     *   +16 u64 model_phys
     *   +24 u32 inst_offset
     *   +28 u32 inst_len
     *   +32 u64 tsk_buf_phys
     *   +48 u64 tmp_buf_phys
     */
    trigger_mode  = read_u32_at(task_phys + 0) & 0xFFFF;
    slot_idx      = read_u32_at(task_phys + 4);
    tsk_phys      = read_u64_at(task_phys + 32);
    cfg_err_slot  = slot_idx & 0xFF;

    if (!tsk_phys) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-nnie: descriptor at 0x%" PRIx64 " has null tsk_buf_phys\n",
                      task_phys);
        goto cfg_err;
    }

    /* tskbuf tail layout for non-LSTM nets (mirrors
     * nnie_build_task_tail in kernel/nnie_neo/nnie_neo.c):
     *   §1  src/dst u32 strides, padded to 16 B
     *   §2  dst u64 phys table (one per dst, packed)
     *   §3  src per-batch u64 phys table
     *
     * For mnist (SrcNum=1, DstNum=1) §1 = 16 B (src stride + dst
     * stride + 8 B zero pad); §2 starts at tskbuf+0x10. Read the
     * first dst phys. */
    dst0_phys = read_u64_at(tsk_phys + 0x10);
    if (!dst0_phys) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-nnie: tskbuf at 0x%" PRIx64 " has null dst[0] phys\n",
                      tsk_phys);
        goto cfg_err;
    }

    /* Stub inference: write the known-good mnist 10-class score vector
     * to the dst blob. The test asserts these specific bytes come back
     * — proves the descriptor read + tskbuf walk + dst write path is
     * intact end-to-end. */
    dma_memory_write(&address_space_memory, dst0_phys,
                     mnist_scores, sizeof(mnist_scores),
                     MEMTXATTRS_UNSPECIFIED);

    /* Update visible HW state. */
    s->regs[NNIE_REG_TASK_ID / 4]    = slot_idx;
    s->regs[NNIE_REG_IRQ_STATUS / 4] |= NNIE_IRQ_FINISH;
    s->regs[NNIE_REG_START / 4]      &= ~1u;  /* HW clears on done */
    s->task_count++;

    (void)trigger_mode;  /* both 0=queued and 1=instant act the same here */

    if (s->regs[NNIE_REG_IRQ_CFG / 4] & NNIE_IRQ_FINISH) {
        qemu_irq_raise(s->irq);
    }
    return;

cfg_err:
    s->regs[NNIE_REG_CFG_ERR_INFO / 4] = 1u | ((cfg_err_slot & 0xFF) << 24);
    s->regs[NNIE_REG_IRQ_STATUS / 4]   |= NNIE_IRQ_CFG_ERR;
    s->regs[NNIE_REG_START / 4]        &= ~1u;
    if (s->regs[NNIE_REG_IRQ_CFG / 4] & NNIE_IRQ_CFG_ERR) {
        qemu_irq_raise(s->irq);
    }
}

/* ── MMIO handlers ────────────────────────────────────────────── */

static uint64_t hisi_nnie_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiNnieState *s = HISI_NNIE(opaque);

    if (offset >= NNIE_MMIO_SIZE) {
        return 0;
    }
    return s->regs[offset / 4];
}

static void hisi_nnie_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    HisiNnieState *s = HISI_NNIE(opaque);

    if (offset >= NNIE_MMIO_SIZE) {
        return;
    }

    switch (offset) {
    case NNIE_REG_START:
        /* Vendor's hal_svp_nnie_start does RMW OR bit 0 — emulate by
         * latching only bit 0 of the written value. HW self-clears
         * the bit when the task completes (see hisi_nnie_execute). */
        if (value & 1) {
            s->regs[NNIE_REG_START / 4] |= 1u;
            hisi_nnie_execute(s);
        }
        break;

    case NNIE_REG_IRQ_CLEAR:
        /* W1C: writing 1 clears the corresponding bit in IRQ_STATUS. */
        s->regs[NNIE_REG_IRQ_STATUS / 4] &= ~(value & NNIE_IRQ_ALL);
        s->regs[NNIE_REG_IRQ_CLEAR / 4]   = value;
        if (!(s->regs[NNIE_REG_IRQ_STATUS / 4] & NNIE_IRQ_ALL)) {
            qemu_irq_lower(s->irq);
        }
        break;

    case NNIE_REG_IRQ_STATUS:
        /* Read-only from guest perspective. Allow the write so the
         * kernel module's pre-START IRQ_STATUS clear is a no-op (it
         * writes IRQ_CLEAR for that anyway). */
        break;

    default:
        s->regs[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps hisi_nnie_ops = {
    .read = hisi_nnie_read,
    .write = hisi_nnie_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── Device lifecycle ─────────────────────────────────────────── */

/* Power-on register defaults captured from a live cv500 NNIE block on
 * av300 after `insmod open_nnie.ko` but before any Forward. Values
 * past +0x6c are HW-self-populated chip-config (DDR timing / clock
 * tree) the kernel reads but never writes. */
static void hisi_nnie_set_poweron_state(HisiNnieState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->task_count = 0;

    /* Chip ID + version block. */
    s->regs[0x00 / 4] = 0x00002018;
    s->regs[0x04 / 4] = 0x00000130;
    s->regs[0x08 / 4] = 0x0000b017;
    s->regs[0x10 / 4] = 0x5a5a5a5a;
    s->regs[0x14 / 4] = 0x0000ffef;

    /* HW-set chip-config registers. */
    s->regs[0x6c / 4] = 0xffffffff;
    s->regs[0x70 / 4] = 0x000004a8;
    s->regs[0x78 / 4] = 0x00000662;
    s->regs[0x7c / 4] = 0x000379a6;
    s->regs[0x80 / 4] = 0x000004a8;
    s->regs[0x84 / 4] = 0x00000190;
    s->regs[0x88 / 4] = 0x00000341;
    s->regs[0x8c / 4] = 0x00021ff4;
    s->regs[0x90 / 4] = 0x000004a8;
    s->regs[0x94 / 4] = 0x00000812;
    s->regs[0x9c / 4] = 0x000451b6;
    s->regs[0xa0 / 4] = 0x000001ff;
    s->regs[0xa4 / 4] = 0x00000149;
    s->regs[0xa8 / 4] = 0x0000006b;

    /* Configurable registers — kernel module RMW's bits in here. */
    s->regs[NNIE_REG_CLK_GATE / 4]    = 0x349;  /* chip-reset bits 0/3/6/8/9 */
    s->regs[NNIE_REG_OUTSTANDING / 4] = 0xf1f;  /* bits[4:0]=0x1F, [11:8]=0xF */
    s->regs[NNIE_REG_CHECK_SUM / 4]   = 0x1;    /* default checksum enabled */
}

static void hisi_nnie_reset(DeviceState *dev)
{
    HisiNnieState *s = HISI_NNIE(dev);
    hisi_nnie_set_poweron_state(s);
}

static void hisi_nnie_init(Object *obj)
{
    HisiNnieState *s = HISI_NNIE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_nnie_ops, s,
                          "hisi-nnie", NNIE_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    hisi_nnie_set_poweron_state(s);
}

static void hisi_nnie_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_nnie_reset);
}

static const TypeInfo hisi_nnie_info = {
    .name          = TYPE_HISI_NNIE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiNnieState),
    .instance_init = hisi_nnie_init,
    .class_init    = hisi_nnie_class_init,
};

static void hisi_nnie_register_types(void)
{
    type_register_static(&hisi_nnie_info);
}

type_init(hisi_nnie_register_types)
