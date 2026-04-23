/*
 * SOI F37 image sensor I2C stub.
 *
 * Sofia (XiongMai) detects F37 by:
 *   1. Setting I2C addr 0x80 (8-bit) = 0x40 (7-bit slave addr)
 *   2. Reading reg 0x0A (high byte) and 0x0B (low byte) — 8-bit reg, 8-bit data
 *   3. Checking (high << 8) | low == 0x0F37
 *
 * Returning 0x0F at 0x0A and 0x37 at 0x0B satisfies the probe.
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

#define TYPE_HISI_F37 "hisi-f37"
OBJECT_DECLARE_SIMPLE_TYPE(HisiF37State, HISI_F37)

#define F37_REG_COUNT 0x100

struct HisiF37State {
    I2CSlave parent_obj;
    uint8_t  cur_reg;
    bool     have_reg;
    uint8_t  regs[F37_REG_COUNT];
};

static void hisi_f37_reset_regs(HisiF37State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    /* Sofia checks regs 0x0A:0x0B → expects 0x0F37 */
    s->regs[0x0A] = 0x0F;
    s->regs[0x0B] = 0x37;
}

static int hisi_f37_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiF37State *s = HISI_F37(i2c);

    if (event == I2C_START_SEND) {
        s->have_reg = false;
    }
    return 0;
}

static int hisi_f37_send(I2CSlave *i2c, uint8_t data)
{
    HisiF37State *s = HISI_F37(i2c);

    if (!s->have_reg) {
        s->cur_reg = data;
        s->have_reg = true;
    } else {
        s->regs[s->cur_reg] = data;
        s->cur_reg++;
    }
    return 0;
}

static uint8_t hisi_f37_recv(I2CSlave *i2c)
{
    HisiF37State *s = HISI_F37(i2c);
    uint8_t val = s->regs[s->cur_reg];
    s->cur_reg++;
    return val;
}

static void hisi_f37_reset(DeviceState *dev)
{
    HisiF37State *s = HISI_F37(dev);

    s->cur_reg = 0;
    s->have_reg = false;
    hisi_f37_reset_regs(s);
}

static void hisi_f37_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_f37_reset);
    k->event = hisi_f37_event;
    k->recv  = hisi_f37_recv;
    k->send  = hisi_f37_send;
}

static const TypeInfo hisi_f37_info = {
    .name          = TYPE_HISI_F37,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiF37State),
    .class_init    = hisi_f37_class_init,
};

static void hisi_f37_register_types(void)
{
    type_register_static(&hisi_f37_info);
}

type_init(hisi_f37_register_types)
