/*
 * Sony IMX122 / IMX222 image sensor SPI stub.
 *
 * IMX222 is the pin/package/format-compatible drop-in successor for
 * IMX122; both share the same Sony 3-wire SPI register interface and
 * are reported as "IMX122" by OpenIPC's ipctool.
 *
 * Detection contract used by ipctool's detect_sony_sensor() on CV100:
 *   - I2C-style address 0x30CE → SPI offset 0x02CE (per sony_i2c_to_spi()).
 *   - Register 0x02CE must read back 0x16 to identify IMX122/IMX222.
 *
 * Wire protocol driven by ssp_sony.ko, three 8-bit transfers per frame:
 *   byte 0:  (addr >> 8) | 0x80  ← bit 7 = read flag
 *   byte 1:  addr & 0xff
 *   byte 2:  data byte (0x00 dummy on read; clocked back is the response)
 *
 * The vendor PL022 driver does not toggle the controller's built-in CS
 * line — chip-select is controlled by a dedicated GPIO outside the SPI
 * controller's view.  We therefore track frame boundaries by counting
 * bytes mod 3 instead of relying on set_cs().
 *
 * Bit ordering: real Sony 3-wire SPI is LSB-first, but PL022 is MSB-only.
 * The vendor ssp_sony.ko driver compensates by bit-reversing every byte
 * before TX and after RX.  At the QEMU SSI bus level we see those
 * pre-reversed bytes, so this device bit-reverses both directions to
 * recover the logical Sony byte stream.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_HISI_IMX122 "hisi-imx122"
OBJECT_DECLARE_SIMPLE_TYPE(HisiIMX122State, HISI_IMX122)

#define IMX122_REG_COUNT  0x400  /* SPI address space 0x000..0x3FF */
#define IMX122_ID_REG     0x02CE
#define IMX122_ID_VALUE   0x16

struct HisiIMX122State {
    SSIPeripheral parent_obj;

    uint8_t  byte_idx;     /* 0..2 within current 3-byte frame */
    bool     is_read;
    uint16_t cur_addr;
    uint8_t  regs[IMX122_REG_COUNT];
};

static void hisi_imx122_reset_regs(HisiIMX122State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[IMX122_ID_REG] = IMX122_ID_VALUE;
}

static uint8_t bitrev8(uint8_t v)
{
    v = ((v >> 1) & 0x55) | ((v & 0x55) << 1);
    v = ((v >> 2) & 0x33) | ((v & 0x33) << 2);
    v = ((v >> 4) & 0x0F) | ((v & 0x0F) << 4);
    return v;
}

static uint32_t hisi_imx122_transfer(SSIPeripheral *dev, uint32_t val)
{
    HisiIMX122State *s = HISI_IMX122(dev);
    uint8_t in = bitrev8(val & 0xff);
    uint8_t out = 0;

    switch (s->byte_idx) {
    case 0:
        s->is_read  = (in & 0x80) != 0;
        s->cur_addr = ((uint16_t)(in & 0x7f)) << 8;
        s->byte_idx = 1;
        break;
    case 1:
        s->cur_addr |= in;
        s->byte_idx = 2;
        break;
    case 2:
    default:
        if (s->cur_addr < IMX122_REG_COUNT) {
            if (s->is_read) {
                out = s->regs[s->cur_addr];
            } else {
                s->regs[s->cur_addr] = in;
            }
        }
        s->byte_idx = 0;
        break;
    }
    return bitrev8(out);
}

static void hisi_imx122_reset(DeviceState *dev)
{
    HisiIMX122State *s = HISI_IMX122(dev);

    s->byte_idx = 0;
    s->is_read  = false;
    s->cur_addr = 0;
    hisi_imx122_reset_regs(s);
}

static void hisi_imx122_realize(SSIPeripheral *dev, Error **errp)
{
    /* Nothing to do — base class handles bus attach.  This callback
     * exists only because ssi_peripheral_realize() dereferences it
     * unconditionally. */
}

static const VMStateDescription vmstate_hisi_imx122 = {
    .name = "hisi-imx122",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, HisiIMX122State),
        VMSTATE_UINT8(byte_idx, HisiIMX122State),
        VMSTATE_BOOL(is_read, HisiIMX122State),
        VMSTATE_UINT16(cur_addr, HisiIMX122State),
        VMSTATE_UINT8_ARRAY(regs, HisiIMX122State, IMX122_REG_COUNT),
        VMSTATE_END_OF_LIST()
    }
};

static void hisi_imx122_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    device_class_set_legacy_reset(dc, hisi_imx122_reset);
    dc->vmsd = &vmstate_hisi_imx122;
    k->realize = hisi_imx122_realize;
    k->transfer = hisi_imx122_transfer;
    k->cs_polarity = SSI_CS_NONE;
}

static const TypeInfo hisi_imx122_info = {
    .name          = TYPE_HISI_IMX122,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(HisiIMX122State),
    .class_init    = hisi_imx122_class_init,
};

static void hisi_imx122_register_types(void)
{
    type_register_static(&hisi_imx122_info);
}

type_init(hisi_imx122_register_types)
