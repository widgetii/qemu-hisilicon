/*
 * SmartSens SP2305 image sensor I2C stub.
 *
 * Sofia (XiongMai) detects SP2305 by:
 *   1. Setting I2C addr 0x78 (8-bit) = 0x3C (7-bit slave addr)
 *   2. Writing 0 to reg 0xFD (page select)
 *   3. Reading reg 0x02 (high byte) and 0x03 (low byte) — 8-bit reg, 8-bit data
 *   4. Checking (high << 8) | low == 0x2735
 *
 * Returning 0x27 at 0x02 and 0x35 at 0x03 satisfies the probe.
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

#define TYPE_HISI_SP2305 "hisi-sp2305"
OBJECT_DECLARE_SIMPLE_TYPE(HisiSP2305State, HISI_SP2305)

#define SP2305_REG_COUNT 0x100

struct HisiSP2305State {
    I2CSlave parent_obj;
    uint8_t  cur_reg;
    bool     have_reg;
    uint8_t  regs[SP2305_REG_COUNT];
};

static void hisi_sp2305_reset_regs(HisiSP2305State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    /* Sofia checks regs 0x02:0x03 → expects 0x2735 */
    s->regs[0x02] = 0x27;
    s->regs[0x03] = 0x35;
}

static int hisi_sp2305_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiSP2305State *s = HISI_SP2305(i2c);

    if (event == I2C_START_SEND) {
        s->have_reg = false;
    }
    return 0;
}

static int hisi_sp2305_send(I2CSlave *i2c, uint8_t data)
{
    HisiSP2305State *s = HISI_SP2305(i2c);

    if (!s->have_reg) {
        s->cur_reg = data;
        s->have_reg = true;
    } else {
        s->regs[s->cur_reg] = data;
        s->cur_reg++;
    }
    return 0;
}

static uint8_t hisi_sp2305_recv(I2CSlave *i2c)
{
    HisiSP2305State *s = HISI_SP2305(i2c);
    uint8_t val = s->regs[s->cur_reg];
    s->cur_reg++;
    return val;
}

static void hisi_sp2305_reset(DeviceState *dev)
{
    HisiSP2305State *s = HISI_SP2305(dev);

    s->cur_reg = 0;
    s->have_reg = false;
    hisi_sp2305_reset_regs(s);
}

static void hisi_sp2305_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_sp2305_reset);
    k->event = hisi_sp2305_event;
    k->recv  = hisi_sp2305_recv;
    k->send  = hisi_sp2305_send;
}

static const TypeInfo hisi_sp2305_info = {
    .name          = TYPE_HISI_SP2305,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiSP2305State),
    .class_init    = hisi_sp2305_class_init,
};

static void hisi_sp2305_register_types(void)
{
    type_register_static(&hisi_sp2305_info);
}

type_init(hisi_sp2305_register_types)
