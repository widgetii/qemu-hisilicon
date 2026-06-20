/*
 * XMedia/Goke "xmsp804" timer — QEMU emulation.
 *
 * The Goke V500 family (gk7205v500/v510/v530, gk7202v330) ships an OpenIPC
 * Linux 4.9.37 kernel whose timer driver
 * (drivers/xmedia/timer/xmedia_sp804_timer.c, compatible "xmedia,sp804")
 * differs from a stock ARM SP804: instead of two timers packed at offsets
 * 0x00/0x20 in one 0x20-stride block, it exposes FOUR independent
 * single-timer blocks at a 0x100 stride:
 *
 *   reg = <base+0x000 0x30>,  timer0: free-running clocksource
 *         <base+0x100 0x30>,  timer1: per-CPU periodic clockevent
 *         <base+0x200 0x30>,  timer2
 *         <base+0x300 0x30>;  timer3
 *   interrupts = <0 27 4>, <0 6 4>, <0 28 4>;  // timer1/2/3 (timer0 has none)
 *
 * Each block uses the *normal* SP804 register map (LOAD 0x00, VALUE 0x04,
 * CTRL 0x08, INTCLR 0x0C, RIS 0x10, MIS 0x14, BGLOAD 0x18).  On a single-core
 * A7 the kernel runs block0 as the clocksource and block1 as the periodic
 * clockevent (GIC SPI 27); the stock QEMU "sp804" model only decodes
 * 0x00-0x3F, so the kernel's writes to block1 (0x100/0x108) hit "Bad offset"
 * and the periodic IRQ is never delivered — Linux hangs at
 * "Calibrating delay loop".
 *
 * This device decodes the block from offset bits [9:8] (n = offset >> 8) and
 * applies the per-timer register semantics to one ptimer per block, exposing
 * one sysbus IRQ per block so each can be wired to its own GIC SPI.
 *
 * Copyright (c) 2026 OpenIPC.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

/* Per-timer control bits (identical to ARM SP804). */
#define TIMER_CTRL_ONESHOT      (1 << 0)
#define TIMER_CTRL_32BIT        (1 << 1)
#define TIMER_CTRL_DIV1         (0 << 2)
#define TIMER_CTRL_DIV16        (1 << 2)
#define TIMER_CTRL_DIV256       (2 << 2)
#define TIMER_CTRL_IE           (1 << 5)
#define TIMER_CTRL_PERIODIC     (1 << 6)
#define TIMER_CTRL_ENABLE       (1 << 7)

#define HISI_XMSP804_NUM_TIMERS  4
#define HISI_XMSP804_STRIDE      0x100
#define HISI_XMSP804_MMIO_SIZE   (HISI_XMSP804_NUM_TIMERS * HISI_XMSP804_STRIDE)

#define TYPE_HISI_XMSP804 "hisi-xmsp804"
OBJECT_DECLARE_SIMPLE_TYPE(HisiXmsp804State, HISI_XMSP804)

typedef struct HisiXmsp804Timer {
    ptimer_state *ptimer;
    uint32_t control;
    uint32_t limit;
    int int_level;
    qemu_irq irq;
    int freq;
} HisiXmsp804Timer;

struct HisiXmsp804State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    HisiXmsp804Timer timer[HISI_XMSP804_NUM_TIMERS];
    uint32_t freq;
};

/* Raise/lower the block IRQ from the latched interrupt + mask. */
static void xmsp804_timer_update(HisiXmsp804Timer *t)
{
    if (t->int_level && (t->control & TIMER_CTRL_IE)) {
        qemu_irq_raise(t->irq);
    } else {
        qemu_irq_lower(t->irq);
    }
}

/*
 * Recompute the ptimer limit after a settings change.
 * Must be called inside a ptimer transaction.
 */
static void xmsp804_timer_recalibrate(HisiXmsp804Timer *t, int reload)
{
    uint32_t limit;

    if ((t->control & (TIMER_CTRL_PERIODIC | TIMER_CTRL_ONESHOT)) == 0) {
        /* Free running. */
        limit = (t->control & TIMER_CTRL_32BIT) ? 0xffffffff : 0xffff;
    } else {
        /* Periodic / one-shot. */
        limit = t->limit;
    }
    ptimer_set_limit(t->ptimer, limit, reload);
}

static uint32_t xmsp804_timer_read(HisiXmsp804Timer *t, hwaddr offset)
{
    switch (offset >> 2) {
    case 0: /* TimerLoad */
    case 6: /* TimerBGLoad */
        return t->limit;
    case 1: /* TimerValue */
        return ptimer_get_count(t->ptimer);
    case 2: /* TimerControl */
        return t->control;
    case 4: /* TimerRIS */
        return t->int_level;
    case 5: /* TimerMIS */
        if ((t->control & TIMER_CTRL_IE) == 0) {
            return 0;
        }
        return t->int_level;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset %x\n", __func__, (int)offset);
        return 0;
    }
}

static void xmsp804_timer_write(HisiXmsp804Timer *t, hwaddr offset,
                                uint32_t value)
{
    int freq;

    switch (offset >> 2) {
    case 0: /* TimerLoad */
        t->limit = value;
        ptimer_transaction_begin(t->ptimer);
        xmsp804_timer_recalibrate(t, 1);
        ptimer_transaction_commit(t->ptimer);
        break;
    case 1: /* TimerValue — read-only; Linux pokes it, ignore. */
        break;
    case 2: /* TimerControl */
        ptimer_transaction_begin(t->ptimer);
        if (t->control & TIMER_CTRL_ENABLE) {
            /* Pause while reprogramming, mirroring arm_timer.c. */
            ptimer_stop(t->ptimer);
        }
        t->control = value;
        freq = t->freq;
        switch ((value >> 2) & 3) {
        case 1: freq >>= 4; break;
        case 2: freq >>= 8; break;
        }
        xmsp804_timer_recalibrate(t, t->control & TIMER_CTRL_ENABLE);
        ptimer_set_freq(t->ptimer, freq);
        if (t->control & TIMER_CTRL_ENABLE) {
            ptimer_run(t->ptimer, (t->control & TIMER_CTRL_ONESHOT) != 0);
        }
        ptimer_transaction_commit(t->ptimer);
        break;
    case 3: /* TimerIntClr */
        t->int_level = 0;
        break;
    case 6: /* TimerBGLoad */
        t->limit = value;
        ptimer_transaction_begin(t->ptimer);
        xmsp804_timer_recalibrate(t, 0);
        ptimer_transaction_commit(t->ptimer);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset %x\n", __func__, (int)offset);
        break;
    }
    xmsp804_timer_update(t);
}

static uint64_t hisi_xmsp804_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiXmsp804State *s = HISI_XMSP804(opaque);
    int n = offset >> 8;

    if (n >= HISI_XMSP804_NUM_TIMERS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad timer %d\n", __func__, n);
        return 0;
    }
    return xmsp804_timer_read(&s->timer[n], offset & 0xff);
}

static void hisi_xmsp804_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    HisiXmsp804State *s = HISI_XMSP804(opaque);
    int n = offset >> 8;

    if (n >= HISI_XMSP804_NUM_TIMERS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad timer %d\n", __func__, n);
        return;
    }
    xmsp804_timer_write(&s->timer[n], offset & 0xff, (uint32_t)value);
}

static const MemoryRegionOps hisi_xmsp804_ops = {
    .read = hisi_xmsp804_read,
    .write = hisi_xmsp804_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void hisi_xmsp804_tick(void *opaque)
{
    HisiXmsp804Timer *t = opaque;
    t->int_level = 1;
    xmsp804_timer_update(t);
}

static void hisi_xmsp804_init(Object *obj)
{
    HisiXmsp804State *s = HISI_XMSP804(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    for (i = 0; i < HISI_XMSP804_NUM_TIMERS; i++) {
        sysbus_init_irq(sbd, &s->timer[i].irq);
    }
    memory_region_init_io(&s->iomem, obj, &hisi_xmsp804_ops, s,
                          TYPE_HISI_XMSP804, HISI_XMSP804_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void hisi_xmsp804_realize(DeviceState *dev, Error **errp)
{
    HisiXmsp804State *s = HISI_XMSP804(dev);
    int i;

    for (i = 0; i < HISI_XMSP804_NUM_TIMERS; i++) {
        HisiXmsp804Timer *t = &s->timer[i];

        t->freq = s->freq;
        t->control = TIMER_CTRL_IE;
        /*
         * CONTINUOUS_TRIGGER matches real SP804 silicon (and the project's
         * arm_timer.c patch): periodic mode with LOAD=0 keeps reloading
         * instead of tripping the ptimer "delta zero, disabling" guard.
         */
        t->ptimer = ptimer_init(hisi_xmsp804_tick, t,
                                PTIMER_POLICY_CONTINUOUS_TRIGGER);
    }
}

static const VMStateDescription vmstate_hisi_xmsp804_timer = {
    .name = "hisi-xmsp804-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(control, HisiXmsp804Timer),
        VMSTATE_UINT32(limit, HisiXmsp804Timer),
        VMSTATE_INT32(int_level, HisiXmsp804Timer),
        VMSTATE_PTIMER(ptimer, HisiXmsp804Timer),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_hisi_xmsp804 = {
    .name = "hisi-xmsp804",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(timer, HisiXmsp804State, HISI_XMSP804_NUM_TIMERS,
                             1, vmstate_hisi_xmsp804_timer, HisiXmsp804Timer),
        VMSTATE_END_OF_LIST()
    }
};

static const Property hisi_xmsp804_properties[] = {
    DEFINE_PROP_UINT32("freq", HisiXmsp804State, freq, 24000000),
};

static void hisi_xmsp804_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hisi_xmsp804_realize;
    dc->vmsd = &vmstate_hisi_xmsp804;
    device_class_set_props(dc, hisi_xmsp804_properties);
}

static const TypeInfo hisi_xmsp804_info = {
    .name          = TYPE_HISI_XMSP804,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiXmsp804State),
    .instance_init = hisi_xmsp804_init,
    .class_init    = hisi_xmsp804_class_init,
};

static void hisi_xmsp804_register_types(void)
{
    type_register_static(&hisi_xmsp804_info);
}

type_init(hisi_xmsp804_register_types)
