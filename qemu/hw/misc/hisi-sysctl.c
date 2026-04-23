/*
 * HiSilicon System Controller (SysCtrl) emulation.
 *
 * Provides SoC identification, system reset, and general-purpose
 * registers for all supported HiSilicon SoC generations (V2/V3/V4).
 *
 * Copyright (c) 2020-2021, 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "system/runstate.h"

#define TYPE_HISI_SYSCTL "hisi-sysctl"
OBJECT_DECLARE_SIMPLE_TYPE(HisiSysctlState, HISI_SYSCTL)

/*
 * Register space is 64K to cover the full range mapped by firmware.
 * V2 DTS maps 0x20050000 size 0x10000; vendor modules do write-then-read
 * cycles across the full range — a 4K window loses writes beyond 0x1000
 * and causes poll loops to hang.
 */
#define HISI_SYSCTL_MMIO_SIZE   0x10000

/* Number of 32-bit general-purpose storage words */
#define HISI_SYSCTL_NREGS       (HISI_SYSCTL_MMIO_SIZE / 4)

struct HisiSysctlState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t soc_id;
    uint32_t regs[HISI_SYSCTL_NREGS];
};

static uint64_t hisi_sysctl_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiSysctlState *s = HISI_SYSCTL(opaque);

    switch (offset) {
    case 0x00: /* SC_CTRL */
        return s->regs[0];
    case 0x04: /* SC_SYSRES — system reset (write-only, read returns 0) */
        return 0;
    case 0x8C: /* REG_SYSSTAT — boot mode (0 = SPI NOR boot) */
        return s->regs[0x8C / 4];
    case 0xEE0: /* SCSYSID0 — full 32-bit chip ID */
        /*
         * Vendor SDK (e.g. EV200 sys.o SYS_HAL_GetChipID) reads this as
         * a single u32 and expects the packed chip ID (e.g. 0x3516E200).
         * On real silicon SCSYSID0..3 are four contiguous bytes; reading
         * 0xEE0 as a word naturally yields the full ID. We return the
         * whole 32-bit value here for the same reason.
         */
        return s->soc_id;
    case 0xEE4: /* SCSYSID1 — second word of the SCSYSID block, unused */
    case 0xEE8: /* SCSYSID2 */
    case 0xEEC: /* SCSYSID3 */
        return 0;
    default:
        if (offset < HISI_SYSCTL_MMIO_SIZE) {
            return s->regs[offset / 4];
        }
        return 0;
    }
}

static void hisi_sysctl_write(void *opaque, hwaddr offset,
                               uint64_t val, unsigned size)
{
    HisiSysctlState *s = HISI_SYSCTL(opaque);

    switch (offset) {
    case 0x04: /* SC_SYSRES — system reset */
        qemu_log_mask(LOG_UNIMP,
                      "hisi-sysctl: SC_SYSRES reset triggered (val=0x%08x)\n",
                      (uint32_t)val);
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        break;
    default:
        if (offset < HISI_SYSCTL_MMIO_SIZE) {
            s->regs[offset / 4] = (uint32_t)val;
        }
        break;
    }
}

static const MemoryRegionOps hisi_sysctl_ops = {
    .read = hisi_sysctl_read,
    .write = hisi_sysctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_sysctl_init(Object *obj)
{
    HisiSysctlState *s = HISI_SYSCTL(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_sysctl_ops, s,
                          TYPE_HISI_SYSCTL, HISI_SYSCTL_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const Property hisi_sysctl_properties[] = {
    DEFINE_PROP_UINT32("soc-id", HisiSysctlState, soc_id, 0),
};

static void hisi_sysctl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, hisi_sysctl_properties);
}

static const TypeInfo hisi_sysctl_info = {
    .name          = TYPE_HISI_SYSCTL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiSysctlState),
    .instance_init = hisi_sysctl_init,
    .class_init    = hisi_sysctl_class_init,
};

static void hisi_sysctl_register_types(void)
{
    type_register_static(&hisi_sysctl_info);
}

type_init(hisi_sysctl_register_types)
