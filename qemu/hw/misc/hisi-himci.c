/*
 * HiSilicon himciv200 (DW MMC) stub for Hi3516CV300.
 *
 * Minimal emulation: reports "no card" so the kernel driver probes
 * cleanly without timeouts. No actual SD/MMC data transfer.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"

#define TYPE_HISI_HIMCI "hisi-himci"
OBJECT_DECLARE_SIMPLE_TYPE(HisiHimciState, HISI_HIMCI)

#define HIMCI_REG_SIZE   0x1000
#define HIMCI_REG_COUNT  (HIMCI_REG_SIZE / 4)

/* Register offsets */
#define MCI_CTRL      0x00
#define MCI_INTMASK   0x24
#define MCI_CMD       0x2C
#define MCI_MINTSTS   0x40
#define MCI_RINTSTS   0x44
#define MCI_STATUS    0x48
#define MCI_CDETECT   0x50
#define MCI_WRTPRT    0x54
#define MCI_VERID     0x6C
#define MCI_HCON      0x70
#define MCI_BMOD      0x80

/* Bits */
#define CTRL_RESET_BITS  0x7      /* bits[2:0]: controller/FIFO/DMA reset */
#define CMD_START_CMD    (1u << 31)
#define BMOD_SWR         (1u << 0)

struct HisiHimciState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[HIMCI_REG_COUNT];
};

static uint64_t hisi_himci_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiHimciState *s = HISI_HIMCI(opaque);

    if (offset >= HIMCI_REG_SIZE) {
        return 0;
    }

    switch (offset) {
    case MCI_CTRL:
        return s->regs[offset / 4] & ~CTRL_RESET_BITS;
    case MCI_CMD:
        return s->regs[offset / 4] & ~CMD_START_CMD;
    case MCI_MINTSTS:
        return s->regs[MCI_RINTSTS / 4] & s->regs[MCI_INTMASK / 4];
    case MCI_RINTSTS:
        return s->regs[offset / 4];
    case MCI_STATUS:
        return 0; /* not busy */
    case MCI_CDETECT:
        return 0x1; /* bit0=1: no card present */
    case MCI_WRTPRT:
        return 0;
    case MCI_VERID:
        return 0x5342270A;
    case MCI_HCON:
        return 0;
    case MCI_BMOD:
        return s->regs[offset / 4] & ~BMOD_SWR;
    default:
        return s->regs[offset / 4];
    }
}

static void hisi_himci_write(void *opaque, hwaddr offset,
                              uint64_t val, unsigned size)
{
    HisiHimciState *s = HISI_HIMCI(opaque);

    if (offset >= HIMCI_REG_SIZE) {
        return;
    }

    switch (offset) {
    case MCI_CTRL:
        s->regs[offset / 4] = val & ~CTRL_RESET_BITS;
        break;
    case MCI_CMD:
        s->regs[offset / 4] = val & ~CMD_START_CMD;
        break;
    case MCI_RINTSTS:
        s->regs[offset / 4] &= ~(uint32_t)val; /* write-1-to-clear */
        break;
    case MCI_BMOD:
        s->regs[offset / 4] = val & ~BMOD_SWR;
        break;
    default:
        s->regs[offset / 4] = val;
        break;
    }
}

static const MemoryRegionOps hisi_himci_ops = {
    .read = hisi_himci_read,
    .write = hisi_himci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_himci_init(Object *obj)
{
    HisiHimciState *s = HISI_HIMCI(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_himci_ops, s,
                          TYPE_HISI_HIMCI, HIMCI_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void hisi_himci_reset(DeviceState *dev)
{
    HisiHimciState *s = HISI_HIMCI(dev);
    memset(s->regs, 0, sizeof(s->regs));
}

static void hisi_himci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_himci_reset);
}

static const TypeInfo hisi_himci_info = {
    .name          = TYPE_HISI_HIMCI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiHimciState),
    .instance_init = hisi_himci_init,
    .class_init    = hisi_himci_class_init,
};

static void hisi_himci_register_types(void)
{
    type_register_static(&hisi_himci_info);
}

type_init(hisi_himci_register_types)
