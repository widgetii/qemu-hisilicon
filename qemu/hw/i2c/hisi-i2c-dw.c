/*
 * HiSilicon V2 / V2A I2C controller (DesignWare-derived, "hi_i2c" kernel
 * driver target).
 *
 * Used on CV200 family SoCs (Hi3516CV200, Hi3518EV200, Hi3516AV100, ...).
 * Mapped at 0x200D0000/0x20240000/0x20250000.  The vendor Linux driver
 * drivers/i2c/busses/i2c-hisilicon.c runs in AUTO mode: it programs a
 * mode register with (write|read, reg_width, data_width) flags and then
 * issues a single TX_RX write that carries the entire transaction
 * payload (register address + data for writes, register address for
 * reads).  The controller performs start/addr/reg/data/stop on the bus
 * and latches the RX value for subsequent TX_RX reads.
 *
 * Register layout (subset that the vendor driver touches):
 *
 *   0x000 CON          mode config (fast/slow, master/slave)
 *   0x004 TAR          target slave address (7-bit)
 *   0x010 DATA_CMD     unused by AUTO mode (manual DMA path)
 *   0x01C SCL_H        SCL high-period divisor (ignored)
 *   0x020 SCL_L        SCL low-period divisor  (ignored)
 *   0x02C INTR_STAT    masked interrupt status
 *   0x030 INTR_MASK
 *   0x034 INTR_RAW     raw interrupt status, bit 6 = TX_ABORT
 *   0x038 RX_TL        RX FIFO threshold (ignored)
 *   0x03C TX_TL        TX FIFO threshold (ignored)
 *   0x040 CLR_INTR     write any = clear all interrupts
 *   0x048 CLR_RX_OVER  w1c (no-op here)
 *   0x04C CLR_TX_OVER  w1c (no-op here)
 *   0x06C ENABLE       bit 0 = controller enable
 *   0x070 STATUS       bit 0 = controller is working
 *   0x074 TXFLR        TX FIFO level
 *   0x078 RXFLR        RX FIFO level
 *   0x07C SDA_HOLD     SDA hold time (ignored)
 *   0x080 TX_ABRT_SRC  abort reason
 *   0x088 DMA_CTRL     DMA control (unused in AUTO mode)
 *   0x09C ENABLE_STATUS
 *   0x0A8 LPIF_STATE   low-power state
 *   0x0AC LOCK         write 0x1ACCE551 to unlock register writes
 *   0x0B0 AUTO         AUTO-mode flags:
 *                        bit 31 = WRITE marker
 *                        bit 30 = READ flag (set for reads)
 *                        bit 29 = 16-bit register address
 *                        bit 28 = 16-bit data width
 *                        bits 27:24 = 0xF when auto mode is OFF
 *                        bits 21:20 = TX FIFO not-full / empty (read-back)
 *                        bit 8      = RX FIFO not-empty (read-back)
 *                        bit 23     = TX_ABRT latch
 *   0x0B4 TX_RX        transfer trigger (writes: reg<<16 | data;
 *                      reads: last received byte/word)
 *   0x0B8 DMA_CMD0     DMA-mode activator (ignored)
 *   0x0BC DMA_CMD1     DMA reg addr
 *   0x0C0 DMA_CMD2     DMA length
 *
 * The emulation performs each transfer synchronously on the TX_RX write,
 * so FIFOs are effectively size-1 and the guest's wait loops (TX_NOT_FULL
 * and STATUS=idle) exit on their first iteration.  Write to CLR_INTR
 * clears the interrupt latches.
 *
 * Copyright (c) 2026 OpenIPC.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_HISI_I2C_DW "hisi-i2c-dw"
OBJECT_DECLARE_SIMPLE_TYPE(HisiI2cDwState, HISI_I2C_DW)

#define HISI_I2C_DW_REG_SIZE  0x1000

/* Register offsets */
#define R_CON           0x000
#define R_TAR           0x004
#define R_DATA_CMD      0x010
#define R_SCL_H         0x01C
#define R_SCL_L         0x020
#define R_INTR_STAT     0x02C
#define R_INTR_MASK     0x030
#define R_INTR_RAW      0x034
#define R_RX_TL         0x038
#define R_TX_TL         0x03C
#define R_CLR_INTR      0x040
#define R_CLR_RX_OVER   0x048
#define R_CLR_TX_OVER   0x04C
#define R_ENABLE        0x06C
#define R_STATUS        0x070
#define R_TXFLR         0x074
#define R_RXFLR         0x078
#define R_SDA_HOLD      0x07C
#define R_TX_ABRT_SRC   0x080
#define R_DMA_CTRL      0x088
#define R_DMA_TDLR      0x08C
#define R_DMA_RDLR      0x090
#define R_ENABLE_STATUS 0x09C
#define R_LPIF_STATE    0x0A8
#define R_LOCK          0x0AC
#define R_AUTO          0x0B0
#define R_TX_RX         0x0B4
#define R_DMA_CMD0      0x0B8
#define R_DMA_CMD1      0x0BC
#define R_DMA_CMD2      0x0C0

/* LOCK register magic */
#define LOCK_UNLOCK_KEY 0x1ACCE551

/* ENABLE register */
#define ENABLE_EN       (1U << 0)

/* STATUS register */
#define STATUS_WORKING  (1U << 0)

/* AUTO register bits */
#define AUTO_WRITE      (1U << 31)
#define AUTO_READ_FLAG  (1U << 30)
#define AUTO_ADDR16     (1U << 29)
#define AUTO_DATA16     (1U << 28)
#define AUTO_MODE_OFF   0x0F000000U
#define AUTO_TX_ABRT    (1U << 23)
#define AUTO_TX_NFULL   (1U << 21)
#define AUTO_TX_EMPTY   (1U << 20)
#define AUTO_RX_NEMPTY  (1U << 8)

/* INTR_RAW bits */
#define INTR_TX_ABRT    (1U << 6)

struct HisiI2cDwState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    I2CBus      *bus;
    qemu_irq     irq;

    uint32_t     con;
    uint32_t     tar;
    uint32_t     auto_reg;
    uint32_t     scl_h;
    uint32_t     scl_l;
    uint32_t     intr_raw;
    uint32_t     intr_mask;
    uint32_t     rx_tl;
    uint32_t     tx_tl;
    uint32_t     sda_hold;
    uint32_t     tx_abrt_src;
    uint32_t     enable;
    uint32_t     lpif_state;
    bool         unlocked;

    uint32_t     rx_data;     /* latched for next TX_RX read */
    bool         rx_valid;
};

static void hisi_i2c_dw_update_irq(HisiI2cDwState *s)
{
    qemu_set_irq(s->irq, (s->intr_raw & ~s->intr_mask) != 0);
}

/*
 * Execute one AUTO-mode transaction.  TX_RX payload encodes either
 *   write:  [31:16] = register address, [15:0] = data
 *   read :  [31:16] = register address (data comes back in rx_data)
 */
static void hisi_i2c_dw_do_transfer(HisiI2cDwState *s, uint32_t tx_rx)
{
    if (!(s->enable & ENABLE_EN)) {
        return;
    }
    if (!(s->auto_reg & AUTO_WRITE)) {
        /* AUTO mode not armed (e.g. 0x0f000000 placeholder). */
        return;
    }

    bool is_read = (s->auto_reg & AUTO_READ_FLAG) != 0;
    bool addr16  = (s->auto_reg & AUTO_ADDR16) != 0;
    bool data16  = (s->auto_reg & AUTO_DATA16) != 0;
    uint8_t slave = s->tar & 0x7F;

    /*
     * TX_RX layout (as composed by the vendor driver):
     *
     *   bits [31:24]  = reg_addr byte 0 (wire-second for 16-bit addr)
     *   bits [23:16]  = reg_addr byte 1 (wire-first  for 16-bit addr,
     *                                    or sole byte for 8-bit addr)
     *   bits [15:8]   = data byte 0     (wire-first  for 16-bit data)
     *   bits [7:0]    = data byte 1     (wire-second, or sole data byte)
     *
     * The vendor kernel driver swaps the user's little-endian recvbuf
     * (buf[0]=LOW, buf[1]=HIGH) into temp_reg via buf[0]<<8|buf[1], so
     * the bits we receive are already pre-swapped.  Emitting them on
     * the wire in the order below gives the sensor the original
     * big-endian register address the user intended.
     */
    uint8_t reg_b0 = (tx_rx >> 24) & 0xFF;  /* wire byte 2 (16-bit only) */
    uint8_t reg_b1 = (tx_rx >> 16) & 0xFF;  /* wire byte 1               */

    if (i2c_start_send(s->bus, slave) != 0) {
        s->intr_raw |= INTR_TX_ABRT;
        goto abort;
    }
    if (addr16) {
        i2c_send(s->bus, reg_b1);   /* emit bits [23:16] first */
        i2c_send(s->bus, reg_b0);   /* then bits [31:24]       */
    } else {
        i2c_send(s->bus, reg_b1);   /* 8-bit reg lives in [23:16] */
    }

    if (is_read) {
        i2c_end_transfer(s->bus);
        if (i2c_start_recv(s->bus, slave) != 0) {
            s->intr_raw |= INTR_TX_ABRT;
            s->rx_data = 0xFFFF;
            s->rx_valid = true;
            goto abort;
        }
        if (data16) {
            uint16_t hi = i2c_recv(s->bus);
            uint16_t lo = i2c_recv(s->bus);
            s->rx_data = (hi << 8) | lo;
        } else {
            s->rx_data = i2c_recv(s->bus) & 0xFF;
        }
        i2c_end_transfer(s->bus);
        s->rx_valid = true;
    } else {
        /* For 16-bit data the kernel puts LOW in [15:8], HIGH in [7:0];
         * same swap rule as the register address. */
        if (data16) {
            i2c_send(s->bus, tx_rx & 0xFF);
            i2c_send(s->bus, (tx_rx >> 8) & 0xFF);
        } else {
            i2c_send(s->bus, tx_rx & 0xFF);
        }
        i2c_end_transfer(s->bus);
    }
    hisi_i2c_dw_update_irq(s);
    return;

abort:
    hisi_i2c_dw_update_irq(s);
}

static uint64_t hisi_i2c_dw_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiI2cDwState *s = HISI_I2C_DW(opaque);
    uint32_t v;

    switch (offset) {
    case R_CON:           return s->con;
    case R_TAR:           return s->tar;
    case R_SCL_H:         return s->scl_h;
    case R_SCL_L:         return s->scl_l;
    case R_INTR_STAT:     return s->intr_raw & ~s->intr_mask;
    case R_INTR_MASK:     return s->intr_mask;
    case R_INTR_RAW:      return s->intr_raw;
    case R_RX_TL:         return s->rx_tl;
    case R_TX_TL:         return s->tx_tl;
    case R_ENABLE:        return s->enable;
    case R_STATUS:        return 0;      /* always idle */
    case R_TXFLR:         return 0;
    case R_RXFLR:         return s->rx_valid ? 1 : 0;
    case R_SDA_HOLD:      return s->sda_hold;
    case R_TX_ABRT_SRC:   return s->tx_abrt_src;
    case R_ENABLE_STATUS: return s->enable & ENABLE_EN;
    case R_LPIF_STATE:    return s->lpif_state;
    case R_LOCK:          return s->unlocked ? 1 : 0;

    case R_AUTO:
        v = s->auto_reg;
        v |= AUTO_TX_EMPTY | AUTO_TX_NFULL;
        if (s->rx_valid) {
            v |= AUTO_RX_NEMPTY;
        }
        if (s->intr_raw & INTR_TX_ABRT) {
            v |= AUTO_TX_ABRT;
        }
        return v;

    case R_TX_RX:
        if (s->rx_valid) {
            v = s->rx_data;
            s->rx_valid = false;
            return v;
        }
        return 0;

    case R_DATA_CMD:
    case R_CLR_INTR:
    case R_CLR_RX_OVER:
    case R_CLR_TX_OVER:
    case R_DMA_CTRL:
    case R_DMA_TDLR:
    case R_DMA_RDLR:
    case R_DMA_CMD0:
    case R_DMA_CMD1:
    case R_DMA_CMD2:
        return 0;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-i2c-dw: bad read offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void hisi_i2c_dw_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    HisiI2cDwState *s = HISI_I2C_DW(opaque);

    switch (offset) {
    case R_LOCK:
        s->unlocked = (value == LOCK_UNLOCK_KEY);
        break;
    case R_CON:           s->con = value; break;
    case R_TAR:           s->tar = value & 0x3FF; break;
    case R_SCL_H:         s->scl_h = value; break;
    case R_SCL_L:         s->scl_l = value; break;
    case R_INTR_MASK:     s->intr_mask = value;
                          hisi_i2c_dw_update_irq(s);
                          break;
    case R_RX_TL:         s->rx_tl = value & 0xFF; break;
    case R_TX_TL:         s->tx_tl = value & 0xFF; break;
    case R_CLR_INTR:      s->intr_raw = 0;
                          s->tx_abrt_src = 0;
                          hisi_i2c_dw_update_irq(s);
                          break;
    case R_CLR_RX_OVER:
    case R_CLR_TX_OVER:
        break;
    case R_ENABLE:        s->enable = value & ENABLE_EN; break;
    case R_SDA_HOLD:      s->sda_hold = value; break;

    case R_AUTO:
        s->auto_reg = value;
        /* Writing AUTO_MODE_OFF (0x0f000000) also clears any latched
         * abort status — vendor driver uses this as "reset error". */
        if ((value & 0xFF000000) == AUTO_MODE_OFF) {
            s->intr_raw &= ~INTR_TX_ABRT;
            hisi_i2c_dw_update_irq(s);
        }
        break;

    case R_TX_RX:
        hisi_i2c_dw_do_transfer(s, value);
        break;

    case R_INTR_RAW:
    case R_INTR_STAT:
    case R_STATUS:
    case R_TXFLR:
    case R_RXFLR:
    case R_TX_ABRT_SRC:
    case R_ENABLE_STATUS:
    case R_DATA_CMD:
    case R_DMA_CTRL:
    case R_DMA_TDLR:
    case R_DMA_RDLR:
    case R_DMA_CMD0:
    case R_DMA_CMD1:
    case R_DMA_CMD2:
    case R_LPIF_STATE:
        /* Read-only or unused in AUTO-mode; accept silently. */
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-i2c-dw: bad write offset 0x%x val=0x%x\n",
                      (int)offset, (unsigned)value);
        break;
    }
}

static const MemoryRegionOps hisi_i2c_dw_ops = {
    .read  = hisi_i2c_dw_read,
    .write = hisi_i2c_dw_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void hisi_i2c_dw_reset(DeviceState *dev)
{
    HisiI2cDwState *s = HISI_I2C_DW(dev);

    s->con = 0;
    s->tar = 0;
    s->auto_reg = AUTO_MODE_OFF;
    s->scl_h = 0;
    s->scl_l = 0;
    s->intr_raw = 0;
    s->intr_mask = 0;
    s->rx_tl = 0;
    s->tx_tl = 0;
    s->sda_hold = 0;
    s->tx_abrt_src = 0;
    s->enable = 0;
    s->lpif_state = 0;
    s->unlocked = false;
    s->rx_data = 0;
    s->rx_valid = false;
}

static void hisi_i2c_dw_init(Object *obj)
{
    HisiI2cDwState *s = HISI_I2C_DW(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_i2c_dw_ops, s,
                          TYPE_HISI_I2C_DW, HISI_I2C_DW_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static void hisi_i2c_dw_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_legacy_reset(dc, hisi_i2c_dw_reset);
}

static const TypeInfo hisi_i2c_dw_info = {
    .name          = TYPE_HISI_I2C_DW,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiI2cDwState),
    .instance_init = hisi_i2c_dw_init,
    .class_init    = hisi_i2c_dw_class_init,
};

static void hisi_i2c_dw_register_types(void)
{
    type_register_static(&hisi_i2c_dw_info);
}

type_init(hisi_i2c_dw_register_types)
