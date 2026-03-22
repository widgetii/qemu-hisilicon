/*
 * HiSilicon VEDU (Video Encoder) + JPGE stub.
 *
 * Provides RAM-backed MMIO regions so the vendor kernel modules
 * (hi3516ev200_vedu.ko, hi3516ev200_jpege.ko) can probe without
 * crashing.  No actual encoding — just enough to satisfy ioremap
 * and the reset/status polling loops.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"

#define TYPE_HISI_VEDU "hisi-vedu"
OBJECT_DECLARE_SIMPLE_TYPE(HisiVeduState, HISI_VEDU)

#define HISI_VEDU_REG_SIZE  0x10000
#define HISI_VEDU_REG_COUNT (HISI_VEDU_REG_SIZE / 4)

/* VEDU register offsets with special behaviour */
#define VEDU_ENCODING_STATUS  0x3004
#define VEDU_RESET_STATUS     0x3010
#define VEDU_RESET_TRIGGER    0x004C

struct HisiVeduState {
    SysBusDevice parent_obj;
    MemoryRegion vedu_iomem;
    MemoryRegion jpge_iomem;
    qemu_irq vedu_irq;
    qemu_irq jpge_irq;
    uint32_t vedu_regs[HISI_VEDU_REG_COUNT];
    uint32_t jpge_regs[HISI_VEDU_REG_COUNT];
};

static uint64_t hisi_vedu_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiVeduState *s = HISI_VEDU(opaque);

    if (offset >= HISI_VEDU_REG_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_vedu_read: offset 0x%x out of range\n",
                      (int)offset);
        return 0;
    }

    uint32_t val = s->vedu_regs[offset / 4];

    switch (offset) {
    case VEDU_RESET_STATUS:
        /* Bit 0 = 0 means reset complete / ready */
        val &= ~1u;
        break;
    case VEDU_ENCODING_STATUS:
        /* Bit 0 = 0: not busy; bits 29-31 = 0: no errors */
        val &= ~(1u | (7u << 29));
        break;
    }

    return val;
}

static void hisi_vedu_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    HisiVeduState *s = HISI_VEDU(opaque);

    if (offset >= HISI_VEDU_REG_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_vedu_write: offset 0x%x out of range\n",
                      (int)offset);
        return;
    }

    if (offset == VEDU_RESET_TRIGGER) {
        qemu_log_mask(LOG_UNIMP, "hisi_vedu: reset triggered (0x%x)\n",
                      (uint32_t)val);
    }

    s->vedu_regs[offset / 4] = val;
}

static uint64_t hisi_jpge_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiVeduState *s = HISI_VEDU(opaque);

    if (offset >= HISI_VEDU_REG_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_jpge_read: offset 0x%x out of range\n",
                      (int)offset);
        return 0;
    }

    return s->jpge_regs[offset / 4];
}

static void hisi_jpge_write(void *opaque, hwaddr offset,
                             uint64_t val, unsigned size)
{
    HisiVeduState *s = HISI_VEDU(opaque);

    if (offset >= HISI_VEDU_REG_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_jpge_write: offset 0x%x out of range\n",
                      (int)offset);
        return;
    }

    s->jpge_regs[offset / 4] = val;
}

static const MemoryRegionOps hisi_vedu_ops = {
    .read = hisi_vedu_read,
    .write = hisi_vedu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps hisi_jpge_ops = {
    .read = hisi_jpge_read,
    .write = hisi_jpge_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_vedu_init(Object *obj)
{
    HisiVeduState *s = HISI_VEDU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->vedu_iomem, obj, &hisi_vedu_ops, s,
                          "hisi-vedu", HISI_VEDU_REG_SIZE);
    sysbus_init_mmio(sbd, &s->vedu_iomem);

    memory_region_init_io(&s->jpge_iomem, obj, &hisi_jpge_ops, s,
                          "hisi-jpge", HISI_VEDU_REG_SIZE);
    sysbus_init_mmio(sbd, &s->jpge_iomem);

    sysbus_init_irq(sbd, &s->vedu_irq);
    sysbus_init_irq(sbd, &s->jpge_irq);
}

static void hisi_vedu_reset(DeviceState *dev)
{
    HisiVeduState *s = HISI_VEDU(dev);
    memset(s->vedu_regs, 0, sizeof(s->vedu_regs));
    memset(s->jpge_regs, 0, sizeof(s->jpge_regs));
}

static void hisi_vedu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_vedu_reset);
}

static const TypeInfo hisi_vedu_info = {
    .name          = TYPE_HISI_VEDU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiVeduState),
    .instance_init = hisi_vedu_init,
    .class_init    = hisi_vedu_class_init,
};

static void hisi_vedu_register_types(void)
{
    type_register_static(&hisi_vedu_info);
}

type_init(hisi_vedu_register_types)
