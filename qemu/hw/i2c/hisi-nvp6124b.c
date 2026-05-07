/*
 * Nextchip NVP6124B AHD/CVI/TVI 4-channel analog video decoder I2C stub.
 *
 * Used externally on Hi3521A / Hi3520Dv300 / Hi3531A DVR boards to decode
 * up to 4 analog HD camera inputs into 8-bit BT.656/BT.1120 streams that
 * feed the SoC's VIU.  The vendor "AHD2.0 EXT Driver" probes the chip
 * via I2C immediately after rmmod/insmod from the rootfs:
 *
 *   1. Write reg 0xff = 0x00 (page select 0)
 *   2. Read reg 0xf4 → expect 0x84 (NVP6124), 0x85 (NVP6114A), or 0x86 (NVP6124B)
 *   3. Read reg 0xf5 → revision byte (0x00 = R0)
 *
 * On a real board there are up to four dies sharing the same SPI/I2C
 * bus at 7-bit slave addresses 0x60 / 0x62 / 0x64 / 0x66 (one per die).
 * For boot-to-shell we emulate one die at 0x60: the driver finds the
 * single chip, logs "AHD2.0 Device (0x60) ID OK... 86", and proceeds
 * with channel-config writes that we accept and discard.
 *
 * No video data is produced — VIU sees no frame interrupts so userland
 * MPP samples that depend on captured video won't run.  Phase 2 scope.
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

#define TYPE_HISI_NVP6124B "hisi-nvp6124b"
OBJECT_DECLARE_SIMPLE_TYPE(HisiNvp6124bState, HISI_NVP6124B)

/* 256 8-bit registers, paginated via reg 0xff.  Vendor driver touches
 * pages 0–5; we keep one shadow per page so writes round-trip cleanly. */
#define NVP6124B_NUM_PAGES 8

struct HisiNvp6124bState {
    I2CSlave parent_obj;
    uint8_t  page;            /* current page (reg 0xff) */
    uint8_t  reg_ptr;         /* current reg address (set on first write byte) */
    bool     reg_ptr_set;     /* false until first write byte arrives */
    uint8_t  regs[NVP6124B_NUM_PAGES][256];
};

static void hisi_nvp6124b_reset_regs(HisiNvp6124bState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    /* Page 0:
     *   0xf4 = chip ID (0x86 = NVP6124B R0)
     *   0xf5 = revision (0x00) */
    s->regs[0][0xf4] = 0x86;
    s->regs[0][0xf5] = 0x00;
    /* Page register defaults to 0; mirrored at [0xff] of every page so
     * read-back of 0xff returns the active page. */
    for (int p = 0; p < NVP6124B_NUM_PAGES; p++) {
        s->regs[p][0xff] = 0;
    }
    s->page = 0;
}

static int hisi_nvp6124b_event(I2CSlave *i2c, enum i2c_event event)
{
    HisiNvp6124bState *s = HISI_NVP6124B(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->reg_ptr_set = false;
        break;
    case I2C_START_RECV:
        /* repeated start after a 1-byte write: reg_ptr already has the
         * read target.  If no write preceded, recv reads the last reg
         * (typical I2C device behaviour). */
        break;
    default:
        break;
    }
    return 0;
}

static int hisi_nvp6124b_send(I2CSlave *i2c, uint8_t data)
{
    HisiNvp6124bState *s = HISI_NVP6124B(i2c);

    if (!s->reg_ptr_set) {
        s->reg_ptr = data;
        s->reg_ptr_set = true;
        return 0;
    }

    /* Subsequent bytes are register-data writes with auto-increment */
    if (s->reg_ptr == 0xff) {
        /* Page select */
        uint8_t newpage = data & 0x07;
        s->page = newpage;
        for (int p = 0; p < NVP6124B_NUM_PAGES; p++) {
            s->regs[p][0xff] = newpage;
        }
    } else {
        s->regs[s->page][s->reg_ptr] = data;
    }
    s->reg_ptr++;
    return 0;
}

static uint8_t hisi_nvp6124b_recv(I2CSlave *i2c)
{
    HisiNvp6124bState *s = HISI_NVP6124B(i2c);
    uint8_t val = s->regs[s->page][s->reg_ptr];
    s->reg_ptr++;
    return val;
}

static void hisi_nvp6124b_reset(DeviceState *dev)
{
    HisiNvp6124bState *s = HISI_NVP6124B(dev);

    s->reg_ptr = 0;
    s->reg_ptr_set = false;
    hisi_nvp6124b_reset_regs(s);
}

static void hisi_nvp6124b_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_nvp6124b_reset);
    k->event = hisi_nvp6124b_event;
    k->recv  = hisi_nvp6124b_recv;
    k->send  = hisi_nvp6124b_send;
}

static const TypeInfo hisi_nvp6124b_info = {
    .name          = TYPE_HISI_NVP6124B,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HisiNvp6124bState),
    .class_init    = hisi_nvp6124b_class_init,
};

static void hisi_nvp6124b_register_types(void)
{
    type_register_static(&hisi_nvp6124b_info);
}

type_init(hisi_nvp6124b_register_types)
