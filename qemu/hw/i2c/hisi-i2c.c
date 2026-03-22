/*
 * HiSilicon HiBVT I2C controller emulation.
 *
 * Translates hibvt register-level I2C protocol into QEMU I2CBus
 * transactions, allowing I2CSlave devices (sensors, EEPROMs, etc.)
 * to be attached.
 *
 * Register layout matches i2c-hibvt.c from the HiSilicon SDK kernel.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"

#define TYPE_HISI_I2C "hisi-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(HisiI2cState, HISI_I2C)

#define HISI_I2C_REG_SIZE   0x1000
#define HISI_I2C_REG_COUNT  (HISI_I2C_REG_SIZE / 4)

/* Register offsets */
#define HIBVT_I2C_DATA1     0x10
#define HIBVT_I2C_TXF       0x20
#define HIBVT_I2C_RXF       0x24
#define HIBVT_I2C_LOOP1     0xB0
#define HIBVT_I2C_CTRL1     0xD0
#define HIBVT_I2C_CTRL2     0xD4
#define HIBVT_I2C_STAT      0xD8
#define HIBVT_I2C_INTR_RAW  0xE0
#define HIBVT_I2C_INTR_EN   0xE4
#define HIBVT_I2C_INTR_STAT 0xE8

/* Bit masks */
#define STAT_RXF_NOE        (1 << 16)   /* RX FIFO not empty */
#define STAT_TXF_NOF        (1 << 19)   /* TX FIFO not full */
#define CTRL1_CMD_START     (1 << 0)
#define CTRL2_CHECK_SDA_IN  (1 << 16)
#define INTR_CMD_DONE       (1 << 12)
#define INTR_ABORT          ((1 << 0) | (1 << 11))

#define I2C_TX_BUF_SIZE     64
#define I2C_RX_BUF_SIZE     64

struct HisiI2cState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus *bus;
    uint32_t regs[HISI_I2C_REG_COUNT];
    uint32_t intr_raw;

    /* I2C transfer state */
    uint8_t  slave_addr;
    bool     is_read;
    bool     transfer_active;
    bool     loop1_written;

    uint8_t  tx_buf[I2C_TX_BUF_SIZE];
    uint32_t tx_count;

    uint8_t  rx_buf[I2C_RX_BUF_SIZE];
    uint32_t rx_count;
    uint32_t rx_ptr;
};

static void hisi_i2c_do_write_transfer(HisiI2cState *s)
{
    if (i2c_start_send(s->bus, s->slave_addr) != 0) {
        s->intr_raw |= INTR_ABORT;
        return;
    }
    s->transfer_active = true;

    /* Send any pre-buffered TX bytes */
    for (uint32_t i = 0; i < s->tx_count; i++) {
        i2c_send(s->bus, s->tx_buf[i]);
    }
    s->tx_count = 0;
}

static void hisi_i2c_do_read_transfer(HisiI2cState *s)
{
    uint32_t loop1 = s->regs[HIBVT_I2C_LOOP1 / 4];

    if (s->loop1_written) {
        s->rx_count = loop1 + 2;
    } else {
        s->rx_count = 1;
    }

    if (s->rx_count > I2C_RX_BUF_SIZE) {
        s->rx_count = I2C_RX_BUF_SIZE;
    }

    s->rx_ptr = 0;

    if (i2c_start_recv(s->bus, s->slave_addr) != 0) {
        /* No device at this address — signal NACK/abort */
        s->rx_count = 0;
        s->intr_raw |= INTR_ABORT;
        return;
    }

    for (uint32_t i = 0; i < s->rx_count; i++) {
        s->rx_buf[i] = i2c_recv(s->bus);
    }
    i2c_end_transfer(s->bus);
}

static uint64_t hisi_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiI2cState *s = HISI_I2C(opaque);

    if (offset >= HISI_I2C_REG_SIZE) {
        return 0;
    }

    switch (offset) {
    case HIBVT_I2C_RXF:
        if (s->rx_ptr < s->rx_count) {
            return s->rx_buf[s->rx_ptr++];
        }
        return 0xFF;
    case HIBVT_I2C_STAT:
        return STAT_TXF_NOF |
               (s->rx_ptr < s->rx_count ? STAT_RXF_NOE : 0);
    case HIBVT_I2C_CTRL2:
        return s->regs[offset / 4] | CTRL2_CHECK_SDA_IN;
    case HIBVT_I2C_INTR_RAW:
        return s->intr_raw;
    case HIBVT_I2C_INTR_STAT:
        return s->intr_raw & s->regs[HIBVT_I2C_INTR_EN / 4];
    default:
        return s->regs[offset / 4];
    }
}

static void hisi_i2c_write(void *opaque, hwaddr offset,
                            uint64_t val, unsigned size)
{
    HisiI2cState *s = HISI_I2C(opaque);

    if (offset >= HISI_I2C_REG_SIZE) {
        return;
    }

    switch (offset) {
    case HIBVT_I2C_DATA1:
        /* End any previous transfer */
        if (s->transfer_active) {
            i2c_end_transfer(s->bus);
            s->transfer_active = false;
        }
        s->slave_addr = (val >> 1) & 0x7F;
        s->is_read = val & 1;
        s->tx_count = 0;
        s->rx_ptr = 0;
        s->rx_count = 0;
        s->loop1_written = false;
        s->regs[offset / 4] = val;
        break;

    case HIBVT_I2C_TXF:
        if (s->transfer_active && !s->is_read) {
            /* Write transfer already started — send directly */
            i2c_send(s->bus, val & 0xFF);
        } else if (s->tx_count < I2C_TX_BUF_SIZE) {
            /* Buffer for later */
            s->tx_buf[s->tx_count++] = val & 0xFF;
        }
        break;

    case HIBVT_I2C_LOOP1:
        s->regs[offset / 4] = val;
        s->loop1_written = true;
        break;

    case HIBVT_I2C_CTRL1:
        s->regs[offset / 4] = val;
        if (val & CTRL1_CMD_START) {
            if (s->is_read) {
                hisi_i2c_do_read_transfer(s);
            } else {
                hisi_i2c_do_write_transfer(s);
            }
            s->intr_raw |= INTR_CMD_DONE;
            s->regs[offset / 4] &= ~CTRL1_CMD_START;
        }
        break;

    case HIBVT_I2C_INTR_RAW:
        s->intr_raw &= ~val;  /* W1C */
        break;

    default:
        s->regs[offset / 4] = val;
        break;
    }
}

static const MemoryRegionOps hisi_i2c_ops = {
    .read = hisi_i2c_read,
    .write = hisi_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_i2c_init(Object *obj)
{
    HisiI2cState *s = HISI_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_i2c_ops, s,
                          TYPE_HISI_I2C, HISI_I2C_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static void hisi_i2c_reset(DeviceState *dev)
{
    HisiI2cState *s = HISI_I2C(dev);
    memset(s->regs, 0, sizeof(s->regs));
    s->intr_raw = 0;
    s->slave_addr = 0;
    s->is_read = false;
    s->transfer_active = false;
    s->loop1_written = false;
    s->tx_count = 0;
    s->rx_count = 0;
    s->rx_ptr = 0;
}

static void hisi_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_i2c_reset);
}

static const TypeInfo hisi_i2c_info = {
    .name          = TYPE_HISI_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiI2cState),
    .instance_init = hisi_i2c_init,
    .class_init    = hisi_i2c_class_init,
};

static void hisi_i2c_register_types(void)
{
    type_register_static(&hisi_i2c_info);
}

type_init(hisi_i2c_register_types)
