/*
 * HiSilicon generic RAM-backed register bank.
 *
 * Provides a simple read/write MMIO region that stores values in RAM.
 * Used for register blocks that firmware writes via read-modify-write
 * (pin mux, DDR PHY cal, PWM, etc.) where returning 0 on read would
 * break the RMW pattern.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"

#define TYPE_HISI_REGBANK "hisi-regbank"
OBJECT_DECLARE_SIMPLE_TYPE(HisiRegbankState, HISI_REGBANK)

struct HisiRegbankState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t *regs;
    uint32_t size;
    char *name;
};

static uint64_t hisi_regbank_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiRegbankState *s = HISI_REGBANK(opaque);
    return s->regs[offset / 4];
}

static void hisi_regbank_write(void *opaque, hwaddr offset,
                               uint64_t val, unsigned size)
{
    HisiRegbankState *s = HISI_REGBANK(opaque);
    s->regs[offset / 4] = val;
}

static const MemoryRegionOps hisi_regbank_ops = {
    .read = hisi_regbank_read,
    .write = hisi_regbank_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_regbank_realize(DeviceState *dev, Error **errp)
{
    HisiRegbankState *s = HISI_REGBANK(dev);

    if (s->size == 0 || (s->size & 3)) {
        error_setg(errp, "hisi-regbank: size must be non-zero and 4-aligned");
        return;
    }

    s->regs = g_new0(uint32_t, s->size / 4);

    memory_region_init_io(&s->iomem, OBJECT(dev), &hisi_regbank_ops, s,
                          s->name ? s->name : TYPE_HISI_REGBANK, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void hisi_regbank_reset(DeviceState *dev)
{
    HisiRegbankState *s = HISI_REGBANK(dev);
    if (s->regs) {
        memset(s->regs, 0, s->size);
    }
}

static const Property hisi_regbank_properties[] = {
    DEFINE_PROP_UINT32("size", HisiRegbankState, size, 0),
    DEFINE_PROP_STRING("name", HisiRegbankState, name),
};

static void hisi_regbank_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_regbank_reset);
    device_class_set_props(dc, hisi_regbank_properties);
    dc->realize = hisi_regbank_realize;
}

static const TypeInfo hisi_regbank_info = {
    .name          = TYPE_HISI_REGBANK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiRegbankState),
    .class_init    = hisi_regbank_class_init,
};

static void hisi_regbank_register_types(void)
{
    type_register_static(&hisi_regbank_info);
}

type_init(hisi_regbank_register_types)
