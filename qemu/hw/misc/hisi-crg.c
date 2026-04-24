/*
 * HiSilicon Clock/Reset Generator (CRG).
 *
 * Provides a simple read/write register bank so the kernel's clock
 * driver can probe without hanging. PLL status registers always
 * report "locked" (bit 28 set).
 *
 * When cpu_srst_offset is set (e.g. 0x78 for CV500/DV300), the CPU
 * soft-reset register is initialized with secondary CPUs held in
 * reset.  SMP bringup is handled by the boot stub polling address 0x4
 * where the kernel writes secondary_startup.
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
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "target/arm/arm-powerctl.h"

#define TYPE_HISI_CRG "hisi-crg"
OBJECT_DECLARE_SIMPLE_TYPE(HisiCrgState, HISI_CRG)

#define HISI_CRG_REG_SIZE  0x10000
#define HISI_CRG_REG_COUNT (HISI_CRG_REG_SIZE / 4)

#define CPU1_SRST_REQ  (1 << 2)
#define DBG1_SRST_REQ  (1 << 4)

#define PLL_LOCK_BIT  (1 << 28)

#define HISI_PLL_APLL_STATUS  0x0004
#define HISI_PLL_DPLL_STATUS  0x0014
#define HISI_PLL_VPLL_STATUS  0x0024
#define HISI_PLL_EPLL_STATUS  0x0034

struct HisiCrgState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[HISI_CRG_REG_COUNT];
    uint32_t cpu_srst_offset;
    uint32_t smp_entry_addr;
    QEMUTimer *smp_timer;
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

    uint32_t old = s->regs[offset / 4];
    s->regs[offset / 4] = (uint32_t)val;

    /* Deferred CPU1 start: when bit 2 transitions 1->0, schedule
     * arm_set_cpu_on after 1 second to let kernel finish init */
    if (s->cpu_srst_offset && offset == s->cpu_srst_offset &&
        s->smp_entry_addr && s->smp_timer) {
        if ((old & (1 << 2)) && !(val & (1 << 2))) {
            timer_mod(s->smp_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                      + NANOSECONDS_PER_SECOND);
        }
    }
}

static void hisi_crg_smp_cb(void *opaque)
{
    HisiCrgState *s = HISI_CRG(opaque);
    uint32_t entry = address_space_ldl(&address_space_memory,
                        s->smp_entry_addr, MEMTXATTRS_UNSPECIFIED, NULL);
    if (entry >= 0x80000000) {
        arm_set_cpu_on(1, entry, 0, 1, false);
    }
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

static void hisi_crg_realize(DeviceState *dev, Error **errp)
{
    HisiCrgState *s = HISI_CRG(dev);
    if (s->smp_entry_addr) {
        s->smp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, hisi_crg_smp_cb, s);
    }
}

static void hisi_crg_reset(DeviceState *dev)
{
    HisiCrgState *s = HISI_CRG(dev);
    if (s->cpu_srst_offset) {
        s->regs[s->cpu_srst_offset / 4] = CPU1_SRST_REQ | DBG1_SRST_REQ;
    }
}

static const Property hisi_crg_properties[] = {
    DEFINE_PROP_UINT32("cpu-srst-offset", HisiCrgState, cpu_srst_offset, 0),
    DEFINE_PROP_UINT32("smp-entry-addr", HisiCrgState, smp_entry_addr, 0),
};

static void hisi_crg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, hisi_crg_properties);
    device_class_set_legacy_reset(dc, hisi_crg_reset);
    dc->realize = hisi_crg_realize;
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
