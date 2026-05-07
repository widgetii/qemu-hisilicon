/*
 * HiSilicon SF (single-FIFO) Ethernet controller — minimal QEMU stub.
 *
 * Found on Hi3520D / Hi3536C / Hi3536DV100 family (per
 * openipc/linux:hisilicon-hi3520dv200's `drivers/net/hieth-sf/`,
 * selected via CONFIG_HIETH=y).  Vendor MMIO at 0x10090000 size 0x10000.
 *
 * The driver hang we satisfy here is in arch/arm/.../sys-hi3520d.c
 * `set_phy_valtage()` and `revise_led_shine()`, both of which do:
 *
 *     do {
 *         writel(reg_value, IO_MDIO_RWCTRL);
 *         udelay(10);
 *     } while (!(readl(IO_MDIO_RWCTRL) & (0x1 << 15)));
 *
 * with no timeout — a regbank stub returning 0 spins forever.  Hardware
 * sets bit 15 of MDIO_RWCTRL when the MDIO operation completes, so this
 * model latches bit 15 of MDIO_RWCTRL on every write, making the busy
 * wait exit on the next read.
 *
 * Other operations (PHY ID reads via hieth_mdio_read, register settings,
 * GLB_SOFT_RESET writes) follow store-then-poll-or-just-store patterns
 * that a flat regbank handles correctly.  We return 0xFFFF for
 * MDIO_RO_DATA so PHY autoscan sees "no PHY" and the driver exits the
 * probe with -ENODEV instead of touching DMA descriptors.  The kernel
 * prints "no dev probed" and continues booting; that's enough for shell.
 *
 * No TX/RX path is modeled — `-nic user` does not attach to this device.
 *
 * Copyright (c) 2026 OpenIPC.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_HISI_HIETH_SF "hisi-hieth-sf"
OBJECT_DECLARE_SIMPLE_TYPE(HisiHiethSfState, HISI_HIETH_SF)

#define HISI_HIETH_SF_MMIO_SIZE   0x10000

/* Register offsets (drivers/net/hieth-sf/{mdio,glb}.h) */
#define R_MDIO_RWCTRL            0x1100
#define R_MDIO_RO_DATA           0x1104

#define B_MDIO_READY             15

struct HisiHiethSfState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[HISI_HIETH_SF_MMIO_SIZE / 4];
};

static uint64_t hisi_hieth_sf_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiHiethSfState *s = HISI_HIETH_SF(opaque);

    if (offset >= HISI_HIETH_SF_MMIO_SIZE) {
        return 0;
    }

    switch (offset) {
    case R_MDIO_RO_DATA:
        /*
         * 0xFFFF on every read register (e.g. PHY ID 1 / 2) makes the
         * Linux PHY framework treat each PHY address as absent, so
         * mdiobus_scan finds nothing and phy_connect returns -ENODEV.
         */
        return 0xFFFF;

    default:
        return s->regs[offset / 4];
    }
}

static void hisi_hieth_sf_write(void *opaque, hwaddr offset,
                                uint64_t val, unsigned size)
{
    HisiHiethSfState *s = HISI_HIETH_SF(opaque);

    if (offset >= HISI_HIETH_SF_MMIO_SIZE) {
        return;
    }

    switch (offset) {
    case R_MDIO_RWCTRL:
        /*
         * Latch the "operation done" bit so the no-timeout busy waits in
         * sys-hi3520d.c (set_phy_valtage / revise_led_shine) and the
         * timed waits in mdio.c wait_mdio_ready() exit on first read.
         */
        s->regs[offset / 4] = (uint32_t)val | (1u << B_MDIO_READY);
        break;

    default:
        s->regs[offset / 4] = (uint32_t)val;
        break;
    }
}

static const MemoryRegionOps hisi_hieth_sf_ops = {
    .read = hisi_hieth_sf_read,
    .write = hisi_hieth_sf_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_hieth_sf_init(Object *obj)
{
    HisiHiethSfState *s = HISI_HIETH_SF(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_hieth_sf_ops, s,
                          TYPE_HISI_HIETH_SF, HISI_HIETH_SF_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void hisi_hieth_sf_reset(DeviceState *dev)
{
    HisiHiethSfState *s = HISI_HIETH_SF(dev);
    memset(s->regs, 0, sizeof(s->regs));
}

static void hisi_hieth_sf_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_hieth_sf_reset);
}

static const TypeInfo hisi_hieth_sf_info = {
    .name          = TYPE_HISI_HIETH_SF,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiHiethSfState),
    .instance_init = hisi_hieth_sf_init,
    .class_init    = hisi_hieth_sf_class_init,
};

static void hisi_hieth_sf_register_types(void)
{
    type_register_static(&hisi_hieth_sf_info);
}

type_init(hisi_hieth_sf_register_types)
