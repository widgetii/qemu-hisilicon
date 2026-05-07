/*
 * HiSilicon HIL2V200 L2 cache controller — minimal QEMU emulation.
 *
 * Found on Hi3520D family (and openipc/linux:hisilicon-hi3520dv200's
 * `arch/arm/mm/cache-hil2v200.c` driver, selected via CONFIG_CACHE_HIL2V200).
 * Vendor MMIO at 0x20700000 size 0x10000.
 *
 * Driver pattern that this stub satisfies:
 *
 *   1. l2cache_init() writes 0 to REG_L2_CTRL (disable), then enables monitors,
 *      then calls __l2cache_inv_all() which loops L2_WAY_NUM times calling
 *      l2_invalid_auto(way):
 *
 *         reg = (way << BIT_L2_MAINT_AUTO_WAYADDRESS) | (0x1 << BIT_L2_MAINT_AUTO_START);
 *         writel(reg, base + REG_L2_MAINT_AUTO);
 *         while (!(readl(base + REG_L2_RINT) & (1 << BIT_L2_RINT_AUTO_END)))
 *             ;
 *         reg = readl(base + REG_L2_RINT);
 *         writel(reg, base + REG_L2_INTCLR);
 *
 *   2. After all-ways-invalidated, writes 1 to REG_L2_CTRL (enable).
 *
 * The polling loop on REG_L2_RINT.AUTO_END is the hang point — a regbank
 * stub returning 0 spins forever.  This model latches AUTO_END when the
 * driver writes to REG_L2_MAINT_AUTO so the poll terminates immediately.
 *
 * Other operations (sync, clean range, invalidate range) write to
 * different registers but follow the same trigger-then-wait-on-RINT
 * pattern; we apply the same trick.
 *
 * Copyright (c) 2026 OpenIPC.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_HISI_L2CACHE "hisi-l2cache"
OBJECT_DECLARE_SIMPLE_TYPE(HisiL2CacheState, HISI_L2CACHE)

#define HISI_L2CACHE_MMIO_SIZE   0x10000

/* Register offsets (cache-hil2v200.h) */
#define R_L2_CTRL                0x0000
#define R_L2_AUCTRL              0x0004
#define R_L2_STATUS              0x0008
#define R_L2_INTMASK             0x0100
#define R_L2_MINT                0x0104
#define R_L2_RINT                0x0108
#define R_L2_INTCLR              0x010C
#define R_L2_SYNC                0x0200
#define R_L2_INVALID             0x0210
#define R_L2_CLEAN               0x0214
#define R_L2_MAINT_AUTO          0x020C

/* Bit positions in *_RINT registers */
#define B_L2_RINT_AUTO_END       14

struct HisiL2CacheState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[HISI_L2CACHE_MMIO_SIZE / 4];
};

static uint64_t hisi_l2cache_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiL2CacheState *s = HISI_L2CACHE(opaque);

    if (offset >= HISI_L2CACHE_MMIO_SIZE) {
        return 0;
    }
    return s->regs[offset / 4];
}

static void hisi_l2cache_write(void *opaque, hwaddr offset,
                               uint64_t val, unsigned size)
{
    HisiL2CacheState *s = HISI_L2CACHE(opaque);

    if (offset >= HISI_L2CACHE_MMIO_SIZE) {
        return;
    }

    switch (offset) {
    case R_L2_MAINT_AUTO:
    case R_L2_SYNC:
    case R_L2_INVALID:
    case R_L2_CLEAN:
        /*
         * Any maintenance trigger immediately latches AUTO_END in RINT
         * so the driver's spin-wait completes on the first read.
         */
        s->regs[offset / 4] = (uint32_t)val;
        s->regs[R_L2_RINT / 4] |= (1u << B_L2_RINT_AUTO_END);
        break;

    case R_L2_INTCLR:
        /*
         * Driver clears RINT bits by writing the current RINT value into
         * INTCLR; honour the W1C semantics (clear matching bits in RINT).
         */
        s->regs[R_L2_RINT / 4] &= ~(uint32_t)val;
        s->regs[offset / 4] = (uint32_t)val;
        break;

    default:
        s->regs[offset / 4] = (uint32_t)val;
        break;
    }
}

static const MemoryRegionOps hisi_l2cache_ops = {
    .read = hisi_l2cache_read,
    .write = hisi_l2cache_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_l2cache_init(Object *obj)
{
    HisiL2CacheState *s = HISI_L2CACHE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_l2cache_ops, s,
                          TYPE_HISI_L2CACHE, HISI_L2CACHE_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void hisi_l2cache_reset(DeviceState *dev)
{
    HisiL2CacheState *s = HISI_L2CACHE(dev);
    memset(s->regs, 0, sizeof(s->regs));
}

static void hisi_l2cache_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_l2cache_reset);
}

static const TypeInfo hisi_l2cache_info = {
    .name          = TYPE_HISI_L2CACHE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiL2CacheState),
    .instance_init = hisi_l2cache_init,
    .class_init    = hisi_l2cache_class_init,
};

static void hisi_l2cache_register_types(void)
{
    type_register_static(&hisi_l2cache_info);
}

type_init(hisi_l2cache_register_types)
