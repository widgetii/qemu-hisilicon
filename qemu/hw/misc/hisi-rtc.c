/*
 * HiSilicon BVT RTC (SPI-bridge interface).
 *
 * The V4 SoC family (Hi3516EV200/EV300, GK7205V200/V300, GK7605V100)
 * has an internal RTC at 0x120e0000 accessed via an SPI command register.
 * Only two MMIO registers exist:
 *
 *   0x000  SPI_CLK_DIV   SPI clock divider
 *   0x004  SPI_RW        SPI command / status / data
 *
 * All RTC registers (time counters, alarm, control) are behind the SPI
 * interface.  The driver (rtc-hibvt.c) writes a command word to SPI_RW,
 * polls spi_busy until clear, then reads back spi_rdata.
 *
 * This emulation completes SPI transactions instantly (spi_busy=0) and
 * tracks time using QEMU's host clock plus a user-settable offset.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "system/rtc.h"

#define TYPE_HISI_RTC "hisi-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(HisiRtcState, HISI_RTC)

/* MMIO register offsets */
#define SPI_CLK_DIV     0x000
#define SPI_RW          0x004

/* SPI_RW bitfield positions */
#define SPI_WDATA_SHIFT     0
#define SPI_WDATA_MASK      0xFF
#define SPI_RDATA_SHIFT     8
#define SPI_RDATA_MASK      0xFF
#define SPI_ADDR_SHIFT      16
#define SPI_ADDR_MASK       0x7F
#define SPI_RW_BIT          (1 << 23)   /* 0=write, 1=read */
#define SPI_START_BIT       (1 << 24)
#define SPI_BUSY_BIT        (1u << 31)

/* Internal RTC register addresses (SPI address space) */
#define RTC_10MS_COUN   0x00
#define RTC_S_COUNT     0x01
#define RTC_M_COUNT     0x02
#define RTC_H_COUNT     0x03
#define RTC_D_COUNT_L   0x04
#define RTC_D_COUNT_H   0x05

#define RTC_MR_10MS     0x06
#define RTC_MR_S        0x07
#define RTC_MR_M        0x08
#define RTC_MR_H        0x09
#define RTC_MR_D_L      0x0A
#define RTC_MR_D_H      0x0B

#define RTC_LR_10MS     0x0C
#define RTC_LR_S        0x0D
#define RTC_LR_M        0x0E
#define RTC_LR_H        0x0F
#define RTC_LR_D_L      0x10
#define RTC_LR_D_H      0x11

#define RTC_LORD        0x12
#define RTC_IMSC        0x13
#define RTC_INT_CLR     0x14
#define RTC_INT         0x15
#define RTC_INT_RAW     0x16
#define RTC_CLK         0x17
#define RTC_POR_N       0x18
#define RTC_SAR_CTRL    0x1A
#define RTC_CLK_CFG     0x1B

#define RTC_FREQ_H      0x51
#define RTC_FREQ_L      0x52

#define RTC_REG_LOCK1   0x64
#define RTC_REG_LOCK2   0x65
#define RTC_REG_LOCK3   0x66
#define RTC_REG_LOCK4   0x67

/* LORD register bits */
#define REG_LOAD_STAT   (1 << 0)
#define REG_LOCK_STAT   (1 << 1)
#define REG_LOCK_BYPASS (1 << 2)

#define RTC_REG_COUNT   128

struct HisiRtcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t spi_clk_div;
    uint32_t spi_rw;
    uint8_t regs[RTC_REG_COUNT];
    int64_t tick_offset;    /* seconds to add to host time */
};

/*
 * Get current RTC time as total seconds since epoch.
 * Uses QEMU's host clock adjusted by tick_offset.
 */
static int64_t hisi_rtc_get_seconds(HisiRtcState *s)
{
    struct tm tm;
    qemu_get_timedate(&tm, s->tick_offset);
    return (int64_t)mktimegm(&tm);
}

/*
 * Fill the time counter registers (0x00-0x05) from current time.
 */
static void hisi_rtc_update_counters(HisiRtcState *s)
{
    int64_t total_secs = hisi_rtc_get_seconds(s);
    int64_t days = total_secs / 86400;
    int rem = total_secs % 86400;
    int hours = rem / 3600;
    int minutes = (rem % 3600) / 60;
    int seconds = rem % 60;

    s->regs[RTC_10MS_COUN] = 0;
    s->regs[RTC_S_COUNT] = seconds;
    s->regs[RTC_M_COUNT] = minutes;
    s->regs[RTC_H_COUNT] = hours;
    s->regs[RTC_D_COUNT_L] = days & 0xFF;
    s->regs[RTC_D_COUNT_H] = (days >> 8) & 0xFF;
}

/*
 * Handle a load operation: read time from LR_* registers and
 * compute a new tick_offset so subsequent reads reflect it.
 */
static void hisi_rtc_do_load(HisiRtcState *s)
{
    int sec = s->regs[RTC_LR_S];
    int min = s->regs[RTC_LR_M];
    int hour = s->regs[RTC_LR_H];
    int day = s->regs[RTC_LR_D_L] | (s->regs[RTC_LR_D_H] << 8);
    int64_t new_secs = (int64_t)day * 86400 + hour * 3600 + min * 60 + sec;

    /* Compute offset: new_secs = host_epoch + tick_offset
     * → tick_offset = new_secs - host_epoch */
    struct tm now;
    qemu_get_timedate(&now, 0);
    int64_t host_secs = (int64_t)mktimegm(&now);
    s->tick_offset = new_secs - host_secs;
}

/*
 * Handle SPI write to an internal register.
 */
static void hisi_rtc_spi_write(HisiRtcState *s, uint8_t addr, uint8_t val)
{
    if (addr >= RTC_REG_COUNT) {
        return;
    }

    s->regs[addr] = val;

    switch (addr) {
    case RTC_LORD:
        if (val & REG_LOAD_STAT) {
            hisi_rtc_do_load(s);
            /* Auto-clear load_stat (load complete) */
            s->regs[RTC_LORD] &= ~REG_LOAD_STAT;
        }
        if (val & REG_LOCK_STAT) {
            /* Freeze counters (update them now), then auto-clear lock */
            hisi_rtc_update_counters(s);
            s->regs[RTC_LORD] &= ~REG_LOCK_STAT;
        }
        break;
    case RTC_INT_CLR:
        s->regs[RTC_INT] = 0;
        s->regs[RTC_INT_RAW] = 0;
        break;
    }
}

/*
 * Handle SPI read from an internal register.
 */
static uint8_t hisi_rtc_spi_read(HisiRtcState *s, uint8_t addr)
{
    if (addr >= RTC_REG_COUNT) {
        return 0;
    }

    /* Time counter registers: compute from current clock */
    if (addr <= RTC_D_COUNT_H) {
        hisi_rtc_update_counters(s);
    }

    /* LORD: auto-clear lock_stat on read (instant lock completion) */
    if (addr == RTC_LORD) {
        uint8_t val = s->regs[RTC_LORD];
        s->regs[RTC_LORD] &= ~REG_LOCK_STAT;
        return val & ~REG_LOCK_STAT;
    }

    return s->regs[addr];
}

static uint64_t hisi_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiRtcState *s = HISI_RTC(opaque);

    switch (offset) {
    case SPI_CLK_DIV:
        return s->spi_clk_div;
    case SPI_RW:
        return s->spi_rw;
    default:
        return 0;
    }
}

static void hisi_rtc_write(void *opaque, hwaddr offset,
                            uint64_t val, unsigned size)
{
    HisiRtcState *s = HISI_RTC(opaque);

    switch (offset) {
    case SPI_CLK_DIV:
        s->spi_clk_div = val;
        break;
    case SPI_RW:
        if (val & SPI_START_BIT) {
            uint8_t addr = (val >> SPI_ADDR_SHIFT) & SPI_ADDR_MASK;
            uint32_t result = val;

            if (val & SPI_RW_BIT) {
                /* SPI read */
                uint8_t rdata = hisi_rtc_spi_read(s, addr);
                result &= ~(SPI_RDATA_MASK << SPI_RDATA_SHIFT);
                result |= (uint32_t)rdata << SPI_RDATA_SHIFT;
            } else {
                /* SPI write */
                uint8_t wdata = (val >> SPI_WDATA_SHIFT) & SPI_WDATA_MASK;
                hisi_rtc_spi_write(s, addr, wdata);
            }

            /* Transaction complete: clear start and busy */
            result &= ~(SPI_START_BIT | SPI_BUSY_BIT);
            s->spi_rw = result;
        } else {
            s->spi_rw = val;
        }
        break;
    }
}

static const MemoryRegionOps hisi_rtc_ops = {
    .read = hisi_rtc_read,
    .write = hisi_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_rtc_init(Object *obj)
{
    HisiRtcState *s = HISI_RTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_rtc_ops, s,
                          "hisi-rtc", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void hisi_rtc_reset(DeviceState *dev)
{
    HisiRtcState *s = HISI_RTC(dev);
    s->spi_clk_div = 0;
    s->spi_rw = 0;
    s->tick_offset = 0;
    memset(s->regs, 0, sizeof(s->regs));
}

static void hisi_rtc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_rtc_reset);
}

static const TypeInfo hisi_rtc_info = {
    .name          = TYPE_HISI_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiRtcState),
    .instance_init = hisi_rtc_init,
    .class_init    = hisi_rtc_class_init,
};

static void hisi_rtc_register_types(void)
{
    type_register_static(&hisi_rtc_info);
}

type_init(hisi_rtc_register_types)
