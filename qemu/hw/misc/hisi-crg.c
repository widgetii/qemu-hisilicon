/*
 * HiSilicon Clock/Reset Generator (CRG) stub.
 *
 * Provides a simple read/write register bank so the kernel's clock
 * driver can probe without hanging. PLL status registers always
 * report "locked" (bit 28 set).
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

#define TYPE_HISI_CRG "hisi-crg"
OBJECT_DECLARE_SIMPLE_TYPE(HisiCrgState, HISI_CRG)

#define HISI_CRG_REG_SIZE  0x10000
#define HISI_CRG_REG_COUNT (HISI_CRG_REG_SIZE / 4)

/*
 * PLL lock bit — HiSilicon CRG drivers check bit 28 of PLL status
 * registers to confirm PLL has locked before using the clock.
 */
#define PLL_LOCK_BIT  (1 << 28)

/* Known PLL status register offsets (from SDK clock drivers) */
#define HISI_PLL_APLL_STATUS  0x0004  /* APLL */
#define HISI_PLL_DPLL_STATUS  0x0014  /* DPLL */
#define HISI_PLL_VPLL_STATUS  0x0024  /* VPLL */
#define HISI_PLL_EPLL_STATUS  0x0034  /* EPLL */

struct HisiCrgState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[HISI_CRG_REG_COUNT];
};

static bool is_pll_status_reg(hwaddr offset)
{
    switch (offset) {
    case HISI_PLL_APLL_STATUS:
    case HISI_PLL_DPLL_STATUS:
    case HISI_PLL_VPLL_STATUS:
    case HISI_PLL_EPLL_STATUS:
        return true;
    default:
        return false;
    }
}

static uint64_t hisi_crg_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiCrgState *s = HISI_CRG(opaque);

    if (offset >= HISI_CRG_REG_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_crg_read: offset 0x%x out of range\n",
                      (int)offset);
        return 0;
    }

    uint32_t val = s->regs[offset / 4];

    /* PLL status registers always report locked */
    if (is_pll_status_reg(offset)) {
        val |= PLL_LOCK_BIT;
    }

    return val;
}

static void hisi_crg_write(void *opaque, hwaddr offset,
                            uint64_t val, unsigned size)
{
    HisiCrgState *s = HISI_CRG(opaque);

    if (offset >= HISI_CRG_REG_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_crg_write: offset 0x%x out of range\n",
                      (int)offset);
        return;
    }

    s->regs[offset / 4] = val;
}

static const MemoryRegionOps hisi_crg_ops = {
    .read = hisi_crg_read,
    .write = hisi_crg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_crg_init(Object *obj)
{
    HisiCrgState *s = HISI_CRG(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_crg_ops, s,
                          TYPE_HISI_CRG, HISI_CRG_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void hisi_crg_class_init(ObjectClass *klass, const void *data)
{
    (void)klass;
}

static const TypeInfo hisi_crg_info = {
    .name          = TYPE_HISI_CRG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiCrgState),
    .instance_init = hisi_crg_init,
    .class_init    = hisi_crg_class_init,
};

static void hisi_crg_register_types(void)
{
    type_register_static(&hisi_crg_info);
}

type_init(hisi_crg_register_types)
