/*
 * HiSilicon V1 I2C controller (vendor "hi_i2c" driver target).
 *
 * Used on CV100 family SoCs (Hi3516CV100, Hi3518CV100, Hi3518EV100).
 * Mapped at 0x200D0000.  The vendor `hi_i2c.ko` kernel module exposes
 * /dev/hi_i2c on top of this controller; OpenIPC's ipctool issues
 * CMD_I2C_READ / CMD_I2C_WRITE ioctls that translate into the register
 * sequence below.  No interrupt is wired — the vendor driver polls
 * SR.OVER (and SR.RECEIVE on reads) every iteration.
 *
 * Reference: hisilicon_mpp/hi3518v100_mpp_1.0.A.0/extdrv/hi_i2c/hii2c.c
 *
 * Register layout:
 *   0x000 CTRL     b8 ENABLE, b7 UNMASK_TOTAL, b6..b0 per-source unmask
 *   0x004 COM      b3 START, b2 READ, b1 WRITE, b0 STOP, b4 SEND_ACK (active-low)
 *   0x008 ICR      write-1-to-clear of SR bits
 *   0x00C SR       b7 BUSY, b6 START, b5 END, b4 SEND, b3 RECEIVE,
 *                  b2 ACK (=NACK seen), b1 ARB, b0 OVER
 *   0x010 SCL_H    high-period prescaler (timing — ignored)
 *   0x014 SCL_L    low-period prescaler  (timing — ignored)
 *   0x018 TXR      byte to transmit (incl. address byte with R/W LSB)
 *   0x01C RXR      last received byte
 *
 * The TXR byte already carries the standard I2C address-with-R/W
 * encoding (bit 0 = read), so a START with WRITE flag translates into
 * either i2c_start_send() or i2c_start_recv() depending on TXR's LSB.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_HISI_I2C_V1 "hisi-i2c-v1"
OBJECT_DECLARE_SIMPLE_TYPE(HisiI2cV1State, HISI_I2C_V1)

#define HISI_I2C_V1_REG_SIZE 0x1000

/* Register offsets */
#define R_CTRL   0x000
#define R_COM    0x004
#define R_ICR    0x008
#define R_SR     0x00C
#define R_SCL_H  0x010
#define R_SCL_L  0x014
#define R_TXR    0x018
#define R_RXR    0x01C

/* COM register bits */
#define COM_SEND_ACK (1 << 4)
#define COM_START    (1 << 3)
#define COM_READ     (1 << 2)
#define COM_WRITE    (1 << 1)
#define COM_STOP     (1 << 0)

/* SR / ICR bits (same layout) */
#define SR_BUSY      (1 << 7)
#define SR_START     (1 << 6)
#define SR_END       (1 << 5)
#define SR_SEND      (1 << 4)
#define SR_RECEIVE   (1 << 3)
#define SR_ACK       (1 << 2)   /* NACK observed on the bus */
#define SR_ARB       (1 << 1)
#define SR_OVER      (1 << 0)
#define SR_ALL       0x7F

struct HisiI2cV1State {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus      *bus;
    qemu_irq     irq;

    uint32_t     ctrl;
    uint32_t     sr;
    uint32_t     scl_h;
    uint32_t     scl_l;
    uint32_t     txr;
    uint32_t     rxr;

    bool         active;   /* an i2c transfer is currently open */
    bool         is_read;  /* the open transfer is read-direction */
};

static uint64_t hisi_i2c_v1_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiI2cV1State *s = opaque;

    switch (offset) {
    case R_CTRL:  return s->ctrl;
    case R_SR:    return s->sr;
    case R_SCL_H: return s->scl_h;
    case R_SCL_L: return s->scl_l;
    case R_TXR:   return s->txr;
    case R_RXR:   return s->rxr;
    case R_COM:
    case R_ICR:
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-i2c-v1: bad read offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void hisi_i2c_v1_do_com(HisiI2cV1State *s, uint8_t com)
{
    /* START condition: open a new transfer using the byte in TXR. */
    if (com & COM_START) {
        uint8_t addr = (s->txr >> 1) & 0x7F;
        bool    want_read = (s->txr & 1) != 0;
        int     ret;

        if (s->active) {
            /* Repeated start — close the prior transfer first.  QEMU's
             * i2c_start_*() will issue STOP+START internally, but
             * end_transfer here makes our state machine match. */
            i2c_end_transfer(s->bus);
            s->active = false;
        }

        ret = want_read ? i2c_start_recv(s->bus, addr)
                        : i2c_start_send(s->bus, addr);
        if (ret == 0) {
            s->active  = true;
            s->is_read = want_read;
            s->sr |= SR_START;
        } else {
            /* No slave at this address — flag NACK to userspace. */
            s->sr |= SR_ACK;
        }
    }

    /* Plain WRITE (no START) — push one byte to the open send transfer. */
    if ((com & COM_WRITE) && !(com & COM_START) && s->active && !s->is_read) {
        i2c_send(s->bus, s->txr & 0xFF);
        s->sr |= SR_SEND;
    }

    /* READ — clock one byte in from the open recv transfer. */
    if ((com & COM_READ) && s->active && s->is_read) {
        s->rxr = i2c_recv(s->bus);
        s->sr |= SR_RECEIVE;
    }

    /* STOP — close the open transfer (if any). */
    if (com & COM_STOP) {
        if (s->active) {
            i2c_end_transfer(s->bus);
            s->active = false;
        }
        s->sr |= SR_END;
    }

    /* Every COM trigger completes "immediately" from the kernel's
     * viewpoint — set OVER so I2C_DRV_WaitWriteEnd's polling loop
     * exits on its first iteration. */
    s->sr |= SR_OVER;
}

static void hisi_i2c_v1_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    HisiI2cV1State *s = opaque;

    switch (offset) {
    case R_CTRL:
        s->ctrl = value & 0x1FF;
        break;
    case R_COM:
        hisi_i2c_v1_do_com(s, value & 0x1F);
        break;
    case R_ICR:
        s->sr &= ~(value & SR_ALL);
        break;
    case R_SCL_H:
        s->scl_h = value;
        break;
    case R_SCL_L:
        s->scl_l = value;
        break;
    case R_TXR:
        s->txr = value & 0xFF;
        break;
    case R_SR:
    case R_RXR:
        /* Read-only on real hardware — ignore. */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-i2c-v1: bad write offset 0x%x val=0x%x\n",
                      (int)offset, (unsigned)value);
        break;
    }
}

static const MemoryRegionOps hisi_i2c_v1_ops = {
    .read  = hisi_i2c_v1_read,
    .write = hisi_i2c_v1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_i2c_v1_reset(DeviceState *dev)
{
    HisiI2cV1State *s = HISI_I2C_V1(dev);

    if (s->active) {
        i2c_end_transfer(s->bus);
    }
    s->ctrl = 0;
    s->sr = 0;
    s->scl_h = 0;
    s->scl_l = 0;
    s->txr = 0;
    s->rxr = 0;
    s->active = false;
    s->is_read = false;
}

static void hisi_i2c_v1_init(Object *obj)
{
    HisiI2cV1State *s = HISI_I2C_V1(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_i2c_v1_ops, s,
                          TYPE_HISI_I2C_V1, HISI_I2C_V1_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static void hisi_i2c_v1_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_i2c_v1_reset);
}

static const TypeInfo hisi_i2c_v1_info = {
    .name          = TYPE_HISI_I2C_V1,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiI2cV1State),
    .instance_init = hisi_i2c_v1_init,
    .class_init    = hisi_i2c_v1_class_init,
};

static void hisi_i2c_v1_register_types(void)
{
    type_register_static(&hisi_i2c_v1_info);
}

type_init(hisi_i2c_v1_register_types)
