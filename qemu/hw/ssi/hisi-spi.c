/*
 * HiSilicon Synchronous Serial Port (PL022 variant).
 *
 * Drop-in replacement for QEMU's upstream "pl022" device, with two
 * HiSilicon-specific extensions baked in:
 *
 *   1. Receive-timeout (RT) interrupt is asserted whenever the RX FIFO
 *      holds data and the TX engine is idle.  Vendor PL022 drivers use
 *      RT to terminate transfers shorter than the RX FIFO threshold (4),
 *      e.g. the 3-byte Sony image-sensor SPI protocol.  Without it the
 *      kernel hangs forever waiting on a threshold that is never reached.
 *
 *   2. DMA threshold registers at offsets 0x28 (TX) and 0x2C (RX).
 *      Upstream PL022 logs these as bad offsets; the HiSilicon variant
 *      accepts the writes (DMA itself is not implemented).
 *
 * Apart from those two changes this device is byte-compatible with the
 * upstream PL022 register interface, including the PrimeCell ID bytes
 * at 0xfe0..0xffc, so the kernel's amba_bus matching keeps working.
 *
 * Copyright (c) 2007 CodeSourcery (original PL022 model by Paul Brook).
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_HISI_SPI "hisi-spi"
OBJECT_DECLARE_SIMPLE_TYPE(HisiSpiState, HISI_SPI)

#define HISI_SPI_CR1_LBM 0x01
#define HISI_SPI_CR1_SSE 0x02
#define HISI_SPI_CR1_MS  0x04
#define HISI_SPI_CR1_SDO 0x08

#define HISI_SPI_SR_TFE  0x01
#define HISI_SPI_SR_TNF  0x02
#define HISI_SPI_SR_RNE  0x04
#define HISI_SPI_SR_RFF  0x08
#define HISI_SPI_SR_BSY  0x10

#define HISI_SPI_INT_ROR 0x01
#define HISI_SPI_INT_RT  0x02
#define HISI_SPI_INT_RX  0x04
#define HISI_SPI_INT_TX  0x08

struct HisiSpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t cr0;
    uint32_t cr1;
    uint32_t bitmask;
    uint32_t sr;
    uint32_t cpsr;
    uint32_t is;
    uint32_t im;
    int      tx_fifo_head;
    int      rx_fifo_head;
    int      tx_fifo_len;
    int      rx_fifo_len;
    uint16_t tx_fifo[8];
    uint16_t rx_fifo[8];
    qemu_irq irq;
    SSIBus  *ssi;
};

static const unsigned char hisi_spi_id[8] =
  { 0x22, 0x10, 0x04, 0x00, 0x0d, 0xf0, 0x05, 0xb1 };

static void hisi_spi_update(HisiSpiState *s)
{
    s->sr = 0;
    if (s->tx_fifo_len == 0) {
        s->sr |= HISI_SPI_SR_TFE;
    }
    if (s->tx_fifo_len != 8) {
        s->sr |= HISI_SPI_SR_TNF;
    }
    if (s->rx_fifo_len != 0) {
        s->sr |= HISI_SPI_SR_RNE;
    }
    if (s->rx_fifo_len == 8) {
        s->sr |= HISI_SPI_SR_RFF;
    }
    if (s->tx_fifo_len) {
        s->sr |= HISI_SPI_SR_BSY;
    }

    s->is = 0;
    if (s->rx_fifo_len >= 4) {
        s->is |= HISI_SPI_INT_RX;
    }
    if (s->tx_fifo_len <= 4) {
        s->is |= HISI_SPI_INT_TX;
    }
    /* HiSilicon vendor extension: receive-timeout for sub-threshold reads. */
    if (s->rx_fifo_len > 0 && s->tx_fifo_len == 0) {
        s->is |= HISI_SPI_INT_RT;
    }

    qemu_set_irq(s->irq, (s->is & s->im) != 0);
}

static void hisi_spi_xfer(HisiSpiState *s)
{
    int i, o, val;

    if ((s->cr1 & HISI_SPI_CR1_SSE) == 0) {
        hisi_spi_update(s);
        return;
    }

    i = (s->tx_fifo_head - s->tx_fifo_len) & 7;
    o = s->rx_fifo_head;
    while (s->tx_fifo_len && s->rx_fifo_len < 8) {
        val = s->tx_fifo[i];
        if (s->cr1 & HISI_SPI_CR1_LBM) {
            /* Loopback */
        } else {
            val = ssi_transfer(s->ssi, val);
        }
        s->rx_fifo[o] = val & s->bitmask;
        i = (i + 1) & 7;
        o = (o + 1) & 7;
        s->tx_fifo_len--;
        s->rx_fifo_len++;
    }
    s->rx_fifo_head = o;
    hisi_spi_update(s);
}

static uint64_t hisi_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiSpiState *s = opaque;
    int val;

    if (offset >= 0xfe0 && offset < 0x1000) {
        return hisi_spi_id[(offset - 0xfe0) >> 2];
    }
    switch (offset) {
    case 0x00: /* CR0 */
        return s->cr0;
    case 0x04: /* CR1 */
        return s->cr1;
    case 0x08: /* DR */
        if (s->rx_fifo_len) {
            val = s->rx_fifo[(s->rx_fifo_head - s->rx_fifo_len) & 7];
            s->rx_fifo_len--;
            hisi_spi_xfer(s);
        } else {
            val = 0;
        }
        return val;
    case 0x0c: /* SR */
        return s->sr;
    case 0x10: /* CPSR */
        return s->cpsr;
    case 0x14: /* IMSC */
        return s->im;
    case 0x18: /* RIS */
        return s->is;
    case 0x1c: /* MIS */
        return s->im & s->is;
    case 0x24: /* DMACR */
        return 0;
    case 0x28: /* DMA TX threshold (HiSilicon) */
    case 0x2c: /* DMA RX threshold (HiSilicon) */
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-spi: bad read offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void hisi_spi_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    HisiSpiState *s = opaque;

    switch (offset) {
    case 0x00: /* CR0 */
        s->cr0 = value;
        s->bitmask = (1 << ((value & 15) + 1)) - 1;
        break;
    case 0x04: /* CR1 */
        s->cr1 = value;
        if ((s->cr1 & (HISI_SPI_CR1_MS | HISI_SPI_CR1_SSE))
                   == (HISI_SPI_CR1_MS | HISI_SPI_CR1_SSE)) {
            qemu_log_mask(LOG_UNIMP,
                          "hisi-spi: peripheral mode not implemented\n");
        }
        hisi_spi_xfer(s);
        break;
    case 0x08: /* DR */
        if (s->tx_fifo_len < 8) {
            s->tx_fifo[s->tx_fifo_head] = value & s->bitmask;
            s->tx_fifo_head = (s->tx_fifo_head + 1) & 7;
            s->tx_fifo_len++;
            hisi_spi_xfer(s);
        }
        break;
    case 0x10: /* CPSR */
        s->cpsr = value & 0xff;
        break;
    case 0x14: /* IMSC */
        s->im = value;
        hisi_spi_update(s);
        break;
    case 0x20: /* ICR — write-1-to-clear ROR/RT */
        value &= HISI_SPI_INT_ROR | HISI_SPI_INT_RT;
        s->is &= ~value;
        break;
    case 0x24: /* DMACR */
        if (value) {
            qemu_log_mask(LOG_UNIMP, "hisi-spi: DMA not implemented\n");
        }
        break;
    case 0x28: /* DMA TX threshold (HiSilicon) */
    case 0x2c: /* DMA RX threshold (HiSilicon) */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-spi: bad write offset 0x%x\n", (int)offset);
    }
}

static void hisi_spi_reset(DeviceState *dev)
{
    HisiSpiState *s = HISI_SPI(dev);

    s->rx_fifo_len = 0;
    s->tx_fifo_len = 0;
    s->im = 0;
    s->is = HISI_SPI_INT_TX;
    s->sr = HISI_SPI_SR_TFE | HISI_SPI_SR_TNF;
}

static const MemoryRegionOps hisi_spi_ops = {
    .read       = hisi_spi_read,
    .write      = hisi_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int hisi_spi_post_load(void *opaque, int version_id)
{
    HisiSpiState *s = opaque;

    if (s->tx_fifo_head < 0 ||
        s->tx_fifo_head >= ARRAY_SIZE(s->tx_fifo) ||
        s->rx_fifo_head < 0 ||
        s->rx_fifo_head >= ARRAY_SIZE(s->rx_fifo)) {
        return -1;
    }
    return 0;
}

static const VMStateDescription vmstate_hisi_spi = {
    .name = "hisi-spi",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = hisi_spi_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr0, HisiSpiState),
        VMSTATE_UINT32(cr1, HisiSpiState),
        VMSTATE_UINT32(bitmask, HisiSpiState),
        VMSTATE_UINT32(sr, HisiSpiState),
        VMSTATE_UINT32(cpsr, HisiSpiState),
        VMSTATE_UINT32(is, HisiSpiState),
        VMSTATE_UINT32(im, HisiSpiState),
        VMSTATE_INT32(tx_fifo_head, HisiSpiState),
        VMSTATE_INT32(rx_fifo_head, HisiSpiState),
        VMSTATE_INT32(tx_fifo_len, HisiSpiState),
        VMSTATE_INT32(rx_fifo_len, HisiSpiState),
        VMSTATE_UINT16_ARRAY(tx_fifo, HisiSpiState, 8),
        VMSTATE_UINT16_ARRAY(rx_fifo, HisiSpiState, 8),
        VMSTATE_END_OF_LIST()
    }
};

static void hisi_spi_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    HisiSpiState *s = HISI_SPI(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &hisi_spi_ops, s,
                          "hisi-spi", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->ssi = ssi_create_bus(dev, "ssi");
}

static void hisi_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_spi_reset);
    dc->vmsd = &vmstate_hisi_spi;
    dc->realize = hisi_spi_realize;
}

static const TypeInfo hisi_spi_info = {
    .name          = TYPE_HISI_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiSpiState),
    .class_init    = hisi_spi_class_init,
};

static void hisi_spi_register_types(void)
{
    type_register_static(&hisi_spi_info);
}

type_init(hisi_spi_register_types)
