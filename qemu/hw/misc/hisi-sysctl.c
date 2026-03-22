/*
 * HiSilicon System Controller (SysCtrl) emulation.
 *
 * Provides SoC identification and system reset for CV300/EV300.
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

struct HisiSysctlState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t soc_id;
};

static uint64_t hisi_sysctl_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiSysctlState *s = HISI_SYSCTL(opaque);

    switch (offset) {
    case 0x00: /* SCSYSID0 / SoC ID */
        return s->soc_id;
    case 0x04: /* SC_SYSRES — system reset (write-only, read returns 0) */
        return 0;
    case 0xEE0: /* SCSYSID0 */
        return s->soc_id & 0xFF;
    case 0xEE4: /* SCSYSID1 */
        return (s->soc_id >> 8) & 0xFF;
    case 0xEE8: /* SCSYSID2 */
        return (s->soc_id >> 16) & 0xFF;
    case 0xEEC: /* SCSYSID3 */
        return (s->soc_id >> 24) & 0xFF;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_sysctl_read: Bad offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void hisi_sysctl_write(void *opaque, hwaddr offset,
                               uint64_t val, unsigned size)
{
    switch (offset) {
    case 0x04: /* SC_SYSRES — system reset */
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi_sysctl_write: Bad offset 0x%x\n", (int)offset);
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
                          TYPE_HISI_SYSCTL, 0x1000);
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
