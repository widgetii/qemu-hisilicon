/*
 * HiSilicon MIPI RX (CSI-2 / LVDS receiver) controller stub.
 *
 * Provides a RAM-backed 64KB MMIO region matching the register layout
 * used by the vendor mipi_rx.ko kernel module on Hi3516EV200/EV300 and
 * Goke GK7205V200/V300/GK7605V100 SoCs.
 *
 * No actual video reception — interrupt status registers always read 0
 * (no errors) so the driver's ISR and polling loops see a clean state.
 *
 * Register map (from mipi_rx_reg.h):
 *   0x000-0x804  PHY config      (mode, skew, enable, CIL FSM, PHY ints)
 *   0x808-0x1000 System regs     (PHY en, lane en, CIL ctrl, colorbar)
 *   0x1004-0x12fc MIPI ctrl      (lanes, DI, pkt/frame ints, crop, imgsize)
 *   0x1300-0x1ffc LVDS ctrl      (WDR, sync codes, lane ID, align ints)
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"

#define TYPE_HISI_MIPI_RX "hisi-mipi-rx"
OBJECT_DECLARE_SIMPLE_TYPE(HisiMipiRxState, HISI_MIPI_RX)

#define MIPI_RX_REG_SIZE   0x10000
#define MIPI_RX_REG_COUNT  (MIPI_RX_REG_SIZE / 4)

/* PHY registers */
#define PHY_MODE_LINK           0x000
#define PHY_SKEW_LINK           0x004
#define CIL_FSM0_LINK           0x104
#define CIL_FSM_ST0_LINK        0x108   /* data lane FSM state (RO) */
#define CIL_FSM_ST1_LINK        0x10c   /* clock lane FSM state (RO) */
#define CIL_INT_RAW_LINK        0x1f0   /* PHY CIL raw interrupt */
#define CIL_INT_LINK            0x1f4   /* PHY CIL masked interrupt */
#define CIL_INT_MSK_LINK        0x1f8   /* PHY CIL interrupt mask */

/* System registers */
#define MIPI_INT_RAW            0xff0   /* global raw interrupt */
#define MIPI_INT_ST             0xff4   /* global masked interrupt */
#define MIPI_INT_MSK            0xff8   /* global interrupt mask */

/* MIPI ctrl registers (channel 0, base 0x1000) */
#define MIPI_LANES_NUM          0x1004
#define MIPI_MAIN_INT_ST        0x100c
#define MIPI_DI_1               0x1010
#define MIPI_DI_2               0x1014
#define MIPI_PKT_INTR_ST        0x1060
#define MIPI_PKT_INTR2_ST       0x1070
#define MIPI_FRAME_INTR_ST      0x1080
#define MIPI_LINE_INTR_ST       0x1090
#define MIPI_CTRL_MODE_HS       0x1108
#define MIPI_IMGSIZE            0x1224
#define MIPI_CTRL_MODE_PIXEL    0x1230
#define MIPI_IMGSIZE0_STATIS    0x1250
#define MIPI_IMGSIZE1_STATIS    0x1254
#define MIPI_CTRL_INT_RAW       0x12f0
#define MIPI_CTRL_INT           0x12f4

/* LVDS ctrl registers */
#define LVDS_CTRL_INT_RAW       0x17f0
#define LVDS_CTRL_INT           0x17f4
#define ALIGN_INT_RAW           0x18f0
#define ALIGN_INT               0x18f4
#define CHN_INT_RAW             0x1ff0
#define CHN_INT                 0x1ff4

struct HisiMipiRxState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[MIPI_RX_REG_COUNT];
};

/*
 * Check if offset is an interrupt status register.
 * These always return 0 — no errors, no pending interrupts.
 */
static bool is_int_status_reg(hwaddr offset)
{
    switch (offset) {
    /* PHY CIL interrupts */
    case CIL_INT_RAW_LINK:
    case CIL_INT_LINK:
    /* Global interrupts */
    case MIPI_INT_RAW:
    case MIPI_INT_ST:
    /* MIPI ctrl interrupts */
    case MIPI_MAIN_INT_ST:
    case MIPI_PKT_INTR_ST:
    case MIPI_PKT_INTR2_ST:
    case MIPI_FRAME_INTR_ST:
    case MIPI_LINE_INTR_ST:
    case MIPI_CTRL_INT_RAW:
    case MIPI_CTRL_INT:
    /* LVDS ctrl interrupts */
    case LVDS_CTRL_INT_RAW:
    case LVDS_CTRL_INT:
    case ALIGN_INT_RAW:
    case ALIGN_INT:
    /* Channel interrupts */
    case CHN_INT_RAW:
    case CHN_INT:
        return true;
    default:
        return false;
    }
}

static uint64_t hisi_mipi_rx_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiMipiRxState *s = HISI_MIPI_RX(opaque);

    if (offset >= MIPI_RX_REG_SIZE) {
        return 0;
    }

    /* Interrupt status registers: always clean (no errors) */
    if (is_int_status_reg(offset)) {
        return 0;
    }

    return s->regs[offset / 4];
}

static void hisi_mipi_rx_write(void *opaque, hwaddr offset,
                                uint64_t val, unsigned size)
{
    HisiMipiRxState *s = HISI_MIPI_RX(opaque);

    if (offset >= MIPI_RX_REG_SIZE) {
        return;
    }

    s->regs[offset / 4] = val;
}

static const MemoryRegionOps hisi_mipi_rx_ops = {
    .read = hisi_mipi_rx_read,
    .write = hisi_mipi_rx_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_mipi_rx_init(Object *obj)
{
    HisiMipiRxState *s = HISI_MIPI_RX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_mipi_rx_ops, s,
                          "hisi-mipi-rx", MIPI_RX_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void hisi_mipi_rx_reset(DeviceState *dev)
{
    HisiMipiRxState *s = HISI_MIPI_RX(dev);
    memset(s->regs, 0, sizeof(s->regs));
}

static void hisi_mipi_rx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_mipi_rx_reset);
}

static const TypeInfo hisi_mipi_rx_info = {
    .name          = TYPE_HISI_MIPI_RX,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiMipiRxState),
    .instance_init = hisi_mipi_rx_init,
    .class_init    = hisi_mipi_rx_class_init,
};

static void hisi_mipi_rx_register_types(void)
{
    type_register_static(&hisi_mipi_rx_info);
}

type_init(hisi_mipi_rx_register_types)
