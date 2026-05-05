/*
 * HiSilicon Hardware True Random Number Generator (HWRNG / TRNG).
 *
 * Two register layouts exist in the silicon, both modelled here via the
 * "data-offset" QOM property:
 *
 *   V3+ (HISEC_TRNG_CTRL block):
 *     base + 0x000  HISEC_COM_TRNG_CTRL       — control (osc_sel, drbg_enable, …)
 *     base + 0x204  HISEC_COM_TRNG_FIFO_DATA  — fresh random word per read
 *     base + 0x208  HISEC_COM_TRNG_DATA_ST    — fifo_data_count [13:8]
 *
 *   V2 (RNG_GEN block on Hi3516CV200):
 *     base + 0x000  RNG_CTRL
 *     base + 0x004  RNG_FIFO_DATA
 *     base + 0x008  RNG_STAT                  — rng_data_count [2:0]
 *
 * The OpenIPC out-of-tree driver (openhisilicon/kernel/hisi-hwrng) is
 * naive: it ioremaps the data register directly and never checks the
 * status register, so its behaviour is satisfied by always returning a
 * fresh random word on data-register reads.  We additionally answer the
 * status register with a non-empty FIFO indication so any future driver
 * that polls properly will also see "data available".
 *
 * Random source: qemu_guest_getrandom_nofail() — cryptographic, never
 * blocks the guest, exactly 4 bytes per access.
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
#include "qemu/guest-random.h"
#include "qemu/module.h"

#define TYPE_HISI_HWRNG "hisi-hwrng"
OBJECT_DECLARE_SIMPLE_TYPE(HisiHwrngState, HISI_HWRNG)

/* Datasheet allocation is 64 KiB across every documented variant. */
#define HISI_HWRNG_REGION_SIZE  0x10000

/* Status register sits 4 bytes above the data register on every variant. */
#define HISI_HWRNG_STATUS_DELTA 0x004

/* Pretend the FIFO has 32 words queued: V3+ uses fifo_data_count[13:8]
 * (so value = 0x20 << 8), V2 uses rng_data_count[2:0] (low 3 bits = 7).
 * We OR both fields so the same constant satisfies either layout. */
#define HISI_HWRNG_STATUS_VALUE ((0x20u << 8) | 0x7u)

struct HisiHwrngState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* QOM property: register-map variant.
     *   0x204 — V3+ HISEC_TRNG_CTRL (CV300/CV500/3519V101/AV200/AV300/V4)
     *   0x004 — V2  RNG_GEN         (CV200) */
    uint32_t data_offset;

    /* Last value written to the control register (offset 0x000).
     * Real silicon has writable bits (osc_sel, drbg_enable, mix_enable,
     * …); we don't model them but echo writes back so any driver that
     * reads the control register doesn't see a surprise zero. */
    uint32_t ctrl;
};

static uint64_t hisi_hwrng_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiHwrngState *s = HISI_HWRNG(opaque);

    if (offset == s->data_offset) {
        uint32_t word;
        qemu_guest_getrandom_nofail(&word, sizeof(word));
        return word;
    }

    if (offset == s->data_offset + HISI_HWRNG_STATUS_DELTA) {
        return HISI_HWRNG_STATUS_VALUE;
    }

    if (offset == 0x000) {
        return s->ctrl;
    }

    qemu_log_mask(LOG_UNIMP,
                  "hisi-hwrng: read from unimplemented offset 0x%" HWADDR_PRIx
                  "\n", offset);
    return 0;
}

static void hisi_hwrng_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    HisiHwrngState *s = HISI_HWRNG(opaque);

    if (offset == 0x000) {
        s->ctrl = (uint32_t)val;
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  "hisi-hwrng: write 0x%" PRIx64 " to unimplemented offset "
                  "0x%" HWADDR_PRIx "\n", val, offset);
}

static const MemoryRegionOps hisi_hwrng_ops = {
    .read = hisi_hwrng_read,
    .write = hisi_hwrng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_hwrng_reset(DeviceState *dev)
{
    HisiHwrngState *s = HISI_HWRNG(dev);
    s->ctrl = 0;
}

static void hisi_hwrng_init(Object *obj)
{
    HisiHwrngState *s = HISI_HWRNG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_hwrng_ops, s,
                          "hisi-hwrng", HISI_HWRNG_REGION_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const Property hisi_hwrng_properties[] = {
    DEFINE_PROP_UINT32("data-offset", HisiHwrngState, data_offset, 0x204),
};

static void hisi_hwrng_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_hwrng_reset);
    device_class_set_props(dc, hisi_hwrng_properties);
}

static const TypeInfo hisi_hwrng_info = {
    .name          = TYPE_HISI_HWRNG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiHwrngState),
    .instance_init = hisi_hwrng_init,
    .class_init    = hisi_hwrng_class_init,
};

static void hisi_hwrng_register_types(void)
{
    type_register_static(&hisi_hwrng_info);
}

type_init(hisi_hwrng_register_types)
