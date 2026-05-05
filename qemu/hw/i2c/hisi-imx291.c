/*
 * Sony IMX291 image sensor I2C stub.
 *
 * Sony IMX291 is the standard 1080p Starvis sensor on Hi3516CV300-class
 * boards.  ipctool / vendor SDK detect IMX291 by:
 *   1. Setting I2C addr 0x34 (8-bit) = 0x1A (7-bit slave addr)
 *   2. Reading reg 0x31DC: must be != 0xFF and (val & 6) must be neither
 *      4 (IMX307) nor 6 (IMX327)
 *   3. Reading reg 0x301E == 0xB2 and reg 0x301F == 0x01
 *   4. Reading reg 0x309C: 0x20 or 0x22 → IMX291, 0x00 → IMX290
 *
 * Returning 0x00 at 0x31DC, 0xB2 at 0x301E, 0x01 at 0x301F, 0x20 at
 * 0x309C satisfies the IMX291 branch.  All other registers read back
 * as 0xFF (uninitialized) so earlier branches in the probe (IMX347,
 * IMX334, IMX335, IMX415, IMX178, IMX274, IMX185, IMX294, IMX226,
 * IMX122) all fall through.
 *
 * Reference: ipctool/src/sensors.c detect_sony_sensor()
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

#define TYPE_HISI_IMX291 "hisi-imx291"
OBJECT_DECLARE_SIMPLE_TYPE(HisiIMX291State, HISI_IMX291)

#define IMX291_REG_BASE  0x3000
#define IMX291_REG_COUNT 0x1000

struct HisiIMX291State {
    I2CSlave parent_obj;
    uint8_t  reg_buf[2];
    uint8_t  reg_ptr;
    uint16_t cur_reg;
    uint8_t  regs[IMX291_REG_COUNT];
};

static void hisi_imx291_reset_regs(HisiIMX291State *s)
{
    /* Default 0xFF (uninitialized) so unrelated probe paths fall through. */
    memset(s->regs, 0xFF, sizeof(s->regs));

    /* Tag for ipctool's "default" branch in detect_sony_sensor:
     * 0x31DC != 0xFF and (val & 6) == 0 (i.e., not IMX307=4, not IMX327=6) */
    s->regs[0x31DC - IMX291_REG_BASE] = 0x00;
    /* IMX291/IMX290 confirm pair */
    s->regs[0x301E - IMX291_REG_BASE] = 0xB2;
    s->regs[0x301F - IMX291_REG_BASE] = 0x01;
    /* IMX291 sub-variant: 0x20 = IMX291, 0x00 = IMX290, 0x22 = IMX291 alt */
    s->regs[0x309C - IMX291_REG_BASE] = 0x20;
}

static int hisi_imx291_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiIMX291State *s = HISI_IMX291(i2c);

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

static int hisi_imx291_send(I2CSlave *i2c, uint8_t data)
{
    HisiIMX291State *s = HISI_IMX291(i2c);

    if (s->reg_ptr < 2) {
        s->reg_buf[s->reg_ptr++] = data;
    } else {
        uint16_t addr = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];
        if (addr >= IMX291_REG_BASE &&
            addr < IMX291_REG_BASE + IMX291_REG_COUNT) {
            s->regs[addr - IMX291_REG_BASE] = data;
        }
        s->reg_buf[1]++;
        if (s->reg_buf[1] == 0) {
            s->reg_buf[0]++;
        }
    }
    return 0;
}

static uint8_t hisi_imx291_recv(I2CSlave *i2c)
{
    HisiIMX291State *s = HISI_IMX291(i2c);
    uint8_t val = 0xFF;

    if (s->cur_reg >= IMX291_REG_BASE &&
        s->cur_reg < IMX291_REG_BASE + IMX291_REG_COUNT) {
        val = s->regs[s->cur_reg - IMX291_REG_BASE];
    }
    s->cur_reg++;
    return val;
}

static void hisi_imx291_reset(DeviceState *dev)
{
    HisiIMX291State *s = HISI_IMX291(dev);

    s->reg_ptr = 0;
    s->cur_reg = 0;
    memset(s->reg_buf, 0, sizeof(s->reg_buf));
    hisi_imx291_reset_regs(s);
}

static void hisi_imx291_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_imx291_reset);
    k->event = hisi_imx291_event;
    k->recv  = hisi_imx291_recv;
    k->send  = hisi_imx291_send;
}

static const TypeInfo hisi_imx291_info = {
    .name          = TYPE_HISI_IMX291,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiIMX291State),
    .class_init    = hisi_imx291_class_init,
};

static void hisi_imx291_register_types(void)
{
    type_register_static(&hisi_imx291_info);
}

type_init(hisi_imx291_register_types)
