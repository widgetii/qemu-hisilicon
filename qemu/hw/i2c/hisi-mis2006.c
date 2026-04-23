/*
 * BYD MIS2006 image sensor I2C stub.
 *
 * Sofia (XiongMai) detects MIS2006 by:
 *   1. I2C addr inherited from previous SmartSens probe = 0x60 (8-bit) = 0x30 (7-bit)
 *   2. Reading reg 0x3000 (high byte) and 0x3001 (low byte) — 16-bit reg, 8-bit data
 *   3. Checking (high << 8) | low == 0x2006
 *
 * Returning 0x20 at 0x3000 and 0x06 at 0x3001 satisfies the probe.
 *
 * Note: this sensor shares I2C addr 0x30 with the SmartSens family.
 * Only one device can be attached at a time via -machine sensor=...
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

#define TYPE_HISI_MIS2006 "hisi-mis2006"
OBJECT_DECLARE_SIMPLE_TYPE(HisiMIS2006State, HISI_MIS2006)

#define MIS2006_REG_BASE  0x3000
#define MIS2006_REG_COUNT 0x1000

struct HisiMIS2006State {
    I2CSlave parent_obj;
    uint8_t  reg_buf[2];
    uint8_t  reg_ptr;
    uint16_t cur_reg;
    uint8_t  regs[MIS2006_REG_COUNT];
};

static void hisi_mis2006_reset_regs(HisiMIS2006State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    /* Sofia checks regs 0x3000:0x3001 → expects 0x2006 */
    s->regs[0x3000 - MIS2006_REG_BASE] = 0x20;
    s->regs[0x3001 - MIS2006_REG_BASE] = 0x06;
}

static int hisi_mis2006_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiMIS2006State *s = HISI_MIS2006(i2c);

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

static int hisi_mis2006_send(I2CSlave *i2c, uint8_t data)
{
    HisiMIS2006State *s = HISI_MIS2006(i2c);

    if (s->reg_ptr < 2) {
        s->reg_buf[s->reg_ptr++] = data;
    } else {
        uint16_t addr = ((uint16_t)s->reg_buf[0] << 8) | s->reg_buf[1];
        if (addr >= MIS2006_REG_BASE &&
            addr < MIS2006_REG_BASE + MIS2006_REG_COUNT) {
            s->regs[addr - MIS2006_REG_BASE] = data;
        }
        s->reg_buf[1]++;
        if (s->reg_buf[1] == 0) {
            s->reg_buf[0]++;
        }
    }
    return 0;
}

static uint8_t hisi_mis2006_recv(I2CSlave *i2c)
{
    HisiMIS2006State *s = HISI_MIS2006(i2c);
    uint8_t val = 0xFF;

    if (s->cur_reg >= MIS2006_REG_BASE &&
        s->cur_reg < MIS2006_REG_BASE + MIS2006_REG_COUNT) {
        val = s->regs[s->cur_reg - MIS2006_REG_BASE];
    }
    s->cur_reg++;
    return val;
}

static void hisi_mis2006_reset(DeviceState *dev)
{
    HisiMIS2006State *s = HISI_MIS2006(dev);

    s->reg_ptr = 0;
    s->cur_reg = 0;
    memset(s->reg_buf, 0, sizeof(s->reg_buf));
    hisi_mis2006_reset_regs(s);
}

static void hisi_mis2006_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_mis2006_reset);
    k->event = hisi_mis2006_event;
    k->recv  = hisi_mis2006_recv;
    k->send  = hisi_mis2006_send;
}

static const TypeInfo hisi_mis2006_info = {
    .name          = TYPE_HISI_MIS2006,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiMIS2006State),
    .class_init    = hisi_mis2006_class_init,
};

static void hisi_mis2006_register_types(void)
{
    type_register_static(&hisi_mis2006_info);
}

type_init(hisi_mis2006_register_types)
