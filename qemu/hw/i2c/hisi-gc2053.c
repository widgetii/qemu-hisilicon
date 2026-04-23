/*
 * GalaxyCore GC2053 image sensor I2C stub.
 *
 * Sofia (XiongMai) detects GC2053 by:
 *   1. Setting I2C addr 0x6E (8-bit) = 0x37 (7-bit slave addr)
 *   2. Reading reg 0xF0 (high byte) and 0xF1 (low byte) — 8-bit reg, 8-bit data
 *   3. Checking (high << 8) | low == 0x2053
 *
 * Returning 0x20 at 0xF0 and 0x53 at 0xF1 satisfies the probe.
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

#define TYPE_HISI_GC2053 "hisi-gc2053"
OBJECT_DECLARE_SIMPLE_TYPE(HisiGC2053State, HISI_GC2053)

#define GC2053_REG_COUNT 0x100

struct HisiGC2053State {
    I2CSlave parent_obj;
    uint8_t  cur_reg;
    bool     have_reg;
    uint8_t  regs[GC2053_REG_COUNT];
};

static void hisi_gc2053_reset_regs(HisiGC2053State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    /* Sofia checks regs 0xF0:0xF1 → expects 0x2053 */
    s->regs[0xF0] = 0x20;
    s->regs[0xF1] = 0x53;
}

static int hisi_gc2053_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiGC2053State *s = HISI_GC2053(i2c);

    if (event == I2C_START_SEND) {
        s->have_reg = false;
    }
    return 0;
}

static int hisi_gc2053_send(I2CSlave *i2c, uint8_t data)
{
    HisiGC2053State *s = HISI_GC2053(i2c);

    if (!s->have_reg) {
        s->cur_reg = data;
        s->have_reg = true;
    } else {
        s->regs[s->cur_reg] = data;
        s->cur_reg++;
    }
    return 0;
}

static uint8_t hisi_gc2053_recv(I2CSlave *i2c)
{
    HisiGC2053State *s = HISI_GC2053(i2c);
    uint8_t val = s->regs[s->cur_reg];
    s->cur_reg++;
    return val;
}

static void hisi_gc2053_reset(DeviceState *dev)
{
    HisiGC2053State *s = HISI_GC2053(dev);

    s->cur_reg = 0;
    s->have_reg = false;
    hisi_gc2053_reset_regs(s);
}

static void hisi_gc2053_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_gc2053_reset);
    k->event = hisi_gc2053_event;
    k->recv  = hisi_gc2053_recv;
    k->send  = hisi_gc2053_send;
}

static const TypeInfo hisi_gc2053_info = {
    .name          = TYPE_HISI_GC2053,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiGC2053State),
    .class_init    = hisi_gc2053_class_init,
};

static void hisi_gc2053_register_types(void)
{
    type_register_static(&hisi_gc2053_info);
}

type_init(hisi_gc2053_register_types)
