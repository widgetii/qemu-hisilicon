/*
 * Sony IMX307 image sensor I2C stub.
 *
 * Sofia (XiongMai) detects IMX307 by:
 *   1. Setting I2C addr 0x34 (8-bit) = 0x1A (7-bit slave addr)
 *   2. Reading reg 0x31DC (16-bit reg, 8-bit data)
 *   3. Checking (val & 6) == 4   (bit 2 set, bit 1 clear)
 *
 * Returning 0x04 at 0x31DC satisfies the probe.  All other registers
 * read back as 0xFF (uninitialized).
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_HISI_IMX307 "hisi-imx307"
OBJECT_DECLARE_SIMPLE_TYPE(HisiIMX307State, HISI_IMX307)

#define IMX307_REG_BASE  0x3000
#define IMX307_REG_COUNT 0x1000

struct HisiIMX307State {
    I2CSlave parent_obj;
    uint8_t  reg_buf[2];
    uint8_t  reg_ptr;
    uint16_t cur_reg;
    uint8_t  regs[IMX307_REG_COUNT];
};

static void hisi_imx307_reset_regs(HisiIMX307State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    /* Sofia checks reg 0x31DC: (val & 6) == 4 → bit 2 set, bit 1 clear */
    s->regs[0x31DC - IMX307_REG_BASE] = 0x04;
}

static int hisi_imx307_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiIMX307State *s = HISI_IMX307(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->reg_ptr = 0;
        break;
    case I2C_START_RECV:
        s->cur_reg = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];
        break;
    default:
        break;
    }
    return 0;
}

static int hisi_imx307_send(I2CSlave *i2c, uint8_t data)
{
    HisiIMX307State *s = HISI_IMX307(i2c);

    if (s->reg_ptr < 2) {
        s->reg_buf[s->reg_ptr++] = data;
    } else {
        uint16_t addr = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];
        if (addr >= IMX307_REG_BASE &&
            addr < IMX307_REG_BASE + IMX307_REG_COUNT) {
            s->regs[addr - IMX307_REG_BASE] = data;
        }
        s->reg_buf[1]++;
        if (s->reg_buf[1] == 0) {
            s->reg_buf[0]++;
        }
    }
    return 0;
}

static uint8_t hisi_imx307_recv(I2CSlave *i2c)
{
    HisiIMX307State *s = HISI_IMX307(i2c);
    uint8_t val = 0xFF;

    if (s->cur_reg >= IMX307_REG_BASE &&
        s->cur_reg < IMX307_REG_BASE + IMX307_REG_COUNT) {
        val = s->regs[s->cur_reg - IMX307_REG_BASE];
    }
    s->cur_reg++;
    return val;
}

static void hisi_imx307_reset(DeviceState *dev)
{
    HisiIMX307State *s = HISI_IMX307(dev);

    s->reg_ptr = 0;
    s->cur_reg = 0;
    memset(s->reg_buf, 0, sizeof(s->reg_buf));
    hisi_imx307_reset_regs(s);
}

static void hisi_imx307_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_imx307_reset);
    k->event = hisi_imx307_event;
    k->recv  = hisi_imx307_recv;
    k->send  = hisi_imx307_send;
}

static const TypeInfo hisi_imx307_info = {
    .name          = TYPE_HISI_IMX307,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiIMX307State),
    .class_init    = hisi_imx307_class_init,
};

static void hisi_imx307_register_types(void)
{
    type_register_static(&hisi_imx307_info);
}

type_init(hisi_imx307_register_types)
