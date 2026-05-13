/*
 * HiSilicon SF (single-FIFO) Ethernet controller — QOM subclass of
 * hisi-femac.
 *
 * The Hi3520Dv200 DVR/NVR SoC ships the V1-era "hieth-sf" controller
 * at 0x10090000.  Its register layout (port @ 0x0000, MDIO @ 0x1100,
 * GLB @ 0x1300, TX EQ_ADDR+EQFRM_LEN at 0x0360/0x0364, RX IQ_ADDR @
 * 0x0358 / RO_IQFRM_DES @ 0x0354, IRQ_ENA/RAW/STAT @ 0x1334/1338/1330)
 * is byte-identical to the FEMAC controller modelled in hisi-femac.c:
 * compare the offsets in the vendor `drivers/net/hieth-sf/{glb,frag,
 * port}.h` headers with `drivers/net/ethernet/hisilicon/hisi_femac.c`.
 *
 * The only behavioural difference exposed to the guest is the MDIO
 * address of the integrated PHY: vendor `drivers/net/hieth-sf/mdio.c`
 * scans the himii bus and binds at address 3 ("himii:03"), whereas
 * upstream hisi_femac binds at address 1.  We model that by inheriting
 * everything from hisi-femac and only overriding the `phy-addr`
 * property default from 1 to 3.
 *
 * Copyright (c) 2026 OpenIPC.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_HISI_FEMAC      "hisi-femac"
#define TYPE_HISI_HIETH_SF   "hisi-hieth-sf"

/* PHY address the vendor Linux hieth driver scans on the himii bus. */
#define HIETH_SF_PHY_ADDR    3

static void hisi_hieth_sf_instance_init(Object *obj)
{
    /*
     * Override the parent `phy-addr` property default (1, for femac)
     * with 3 so the vendor 3.0.x hieth driver finds the PHY at
     * himii:03.  SoC init code may still override this via
     * qdev_prop_set_uint8() before realize().
     */
    object_property_set_uint(obj, "phy-addr", HIETH_SF_PHY_ADDR,
                             &error_abort);
}

static const TypeInfo hisi_hieth_sf_info = {
    .name          = TYPE_HISI_HIETH_SF,
    .parent        = TYPE_HISI_FEMAC,
    .instance_init = hisi_hieth_sf_instance_init,
    /* No fields beyond parent: instance_size + class_init inherited. */
};

static void hisi_hieth_sf_register_types(void)
{
    type_register_static(&hisi_hieth_sf_info);
}

type_init(hisi_hieth_sf_register_types)
