/*
 * HiSilicon HiFMC V100 Flash Memory Controller emulation.
 *
 * Emulates the FMC found on Hi3516CV300 / Hi3516EV300 SoCs, with a
 * RAM-backed SPI NOR flash (Winbond W25Q64, 8 MiB).  Supports both
 * register-mode (small reads via memory window) and DMA-mode transfers.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "system/dma.h"

/* ── Register offsets ────────────────────────────────────────────────── */

#define FMC_CFG             0x00
#define FMC_GLOBAL_CFG      0x04
#define FMC_SPI_TIMING_CFG  0x08
#define FMC_INT             0x18
#define FMC_INT_EN          0x1C
#define FMC_INT_CLR         0x20
#define FMC_CMD             0x24
#define FMC_ADDRH           0x28
#define FMC_ADDRL           0x2C
#define FMC_OP_CFG          0x30
#define FMC_SPI_OP_ADDR     0x34
#define FMC_DATA_NUM        0x38
#define FMC_OP              0x3C
#define FMC_DMA_LEN         0x40
#define FMC_DMA_SADDR_D0    0x4C
#define FMC_OP_CTRL         0x68
#define FMC_STATUS          0xAC
#define FMC_VERSION         0xBC

/* FMC_OP bits */
#define FMC_OP_REG_OP_START     BIT(0)
#define FMC_OP_READ_STATUS_EN   BIT(1)
#define FMC_OP_READ_DATA_EN     BIT(2)
#define FMC_OP_WRITE_DATA_EN    BIT(5)
#define FMC_OP_ADDR_EN          BIT(6)
#define FMC_OP_CMD1_EN          BIT(7)

/* FMC_OP_CTRL bits */
#define FMC_OP_CTRL_DMA_OP_READY BIT(0)
#define FMC_OP_CTRL_RW_OP        BIT(1)
#define FMC_OP_CTRL_DMA_OP       BIT(2)
#define FMC_OP_CTRL_RD_OPCODE_SHIFT 16

/* FMC_INT bits */
#define FMC_INT_OP_DONE     BIT(0)

/* SPI NOR commands */
#define SPI_CMD_WRITE_ENABLE  0x06
#define SPI_CMD_READ_STATUS   0x05
#define SPI_CMD_READ_ID       0x9F
#define SPI_CMD_READ          0x03
#define SPI_CMD_FAST_READ     0x0B
#define SPI_CMD_PAGE_PROGRAM  0x02
#define SPI_CMD_SECTOR_ERASE  0xD8
#define SPI_CMD_CHIP_ERASE    0xC7

/* SPI status register bits */
#define SPI_SR_WEL          BIT(1)

/* Flash identity: Winbond W25Q64 (8 MiB) */
#define FLASH_JEDEC_0       0xEF    /* Winbond */
#define FLASH_JEDEC_1       0x40
#define FLASH_JEDEC_2       0x17    /* 64Mbit = 8 MiB */
#define FLASH_SIZE_DEFAULT  (8 * 1024 * 1024)
#define FLASH_SECTOR_SIZE   (64 * 1024)
#define FLASH_PAGE_SIZE     256

#define IOBUF_SIZE          256
#define CTRL_REG_SIZE       0x1000
#define MEM_WINDOW_SIZE     0x10000

/* ── Device state ────────────────────────────────────────────────────── */

#define TYPE_HISI_FMC "hisi-fmc"
OBJECT_DECLARE_SIMPLE_TYPE(HisiFmcState, HISI_FMC)

struct HisiFmcState {
    SysBusDevice parent_obj;

    MemoryRegion ctrl_iomem;
    MemoryRegion mem_iomem;

    /* Control registers */
    uint32_t cfg;
    uint32_t global_cfg;
    uint32_t spi_timing;
    uint32_t fmc_int;
    uint32_t int_en;
    uint32_t cmd;
    uint32_t addrh;
    uint32_t addrl;
    uint32_t op_cfg;
    uint32_t spi_op_addr;
    uint32_t data_num;
    uint32_t dma_len;
    uint32_t dma_saddr;
    uint32_t op_ctrl;
    uint32_t status;

    /* SPI NOR state */
    uint8_t  sr;
    uint8_t *flash;
    uint32_t flash_size;

    /* Register-mode I/O buffer (shared via memory window) */
    uint8_t  iobuf[IOBUF_SIZE];
};

/* ── SPI command execution (register mode) ───────────────────────────── */

static void hisi_fmc_exec_reg_op(HisiFmcState *s)
{
    uint8_t spi_cmd = s->cmd & 0xFF;
    uint32_t addr = s->addrl;
    uint32_t len = s->data_num & 0x3FFF;

    if (len > IOBUF_SIZE) {
        len = IOBUF_SIZE;
    }

    switch (spi_cmd) {
    case SPI_CMD_READ_ID:
        s->iobuf[0] = FLASH_JEDEC_0;
        s->iobuf[1] = FLASH_JEDEC_1;
        s->iobuf[2] = FLASH_JEDEC_2;
        /* Pad remaining bytes with 0 */
        if (len > 3) {
            memset(&s->iobuf[3], 0, len - 3);
        }
        break;

    case SPI_CMD_READ_STATUS:
        s->iobuf[0] = s->sr;
        break;

    case SPI_CMD_READ:
    case SPI_CMD_FAST_READ:
        if (addr + len > s->flash_size) {
            len = (addr < s->flash_size) ? s->flash_size - addr : 0;
        }
        if (len) {
            memcpy(s->iobuf, &s->flash[addr], len);
        }
        break;

    case SPI_CMD_WRITE_ENABLE:
        s->sr |= SPI_SR_WEL;
        break;

    case SPI_CMD_PAGE_PROGRAM:
        if ((s->sr & SPI_SR_WEL) && addr < s->flash_size) {
            uint32_t end = addr + len;
            if (end > s->flash_size) {
                end = s->flash_size;
            }
            for (uint32_t i = addr; i < end; i++) {
                /* NOR flash: can only clear bits (AND with data) */
                s->flash[i] &= s->iobuf[i - addr];
            }
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_SECTOR_ERASE:
        if (s->sr & SPI_SR_WEL) {
            uint32_t base = addr & ~(FLASH_SECTOR_SIZE - 1);
            uint32_t end = base + FLASH_SECTOR_SIZE;
            if (end > s->flash_size) {
                end = s->flash_size;
            }
            if (base < s->flash_size) {
                memset(&s->flash[base], 0xFF, end - base);
            }
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_CHIP_ERASE:
        if (s->sr & SPI_SR_WEL) {
            memset(s->flash, 0xFF, s->flash_size);
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "hisi-fmc: unhandled SPI command 0x%02x\n", spi_cmd);
        break;
    }

    /* Operation completes instantly */
    s->fmc_int |= FMC_INT_OP_DONE;
}

/* ── DMA command execution ───────────────────────────────────────────── */

static void hisi_fmc_exec_dma_op(HisiFmcState *s)
{
    uint32_t addr = s->addrl;
    uint32_t len = s->dma_len;
    bool is_write = s->op_ctrl & FMC_OP_CTRL_RW_OP;

    if (addr + len > s->flash_size) {
        len = (addr < s->flash_size) ? s->flash_size - addr : 0;
    }

    if (len == 0) {
        s->fmc_int |= FMC_INT_OP_DONE;
        return;
    }

    if (!is_write) {
        /* DMA read: flash → guest memory */
        dma_memory_write(&address_space_memory, s->dma_saddr,
                         &s->flash[addr], len, MEMTXATTRS_UNSPECIFIED);
    } else {
        /* DMA write: guest memory → flash */
        if (s->sr & SPI_SR_WEL) {
            uint8_t *buf = g_malloc(len);
            dma_memory_read(&address_space_memory, s->dma_saddr,
                            buf, len, MEMTXATTRS_UNSPECIFIED);
            for (uint32_t i = 0; i < len; i++) {
                s->flash[addr + i] &= buf[i];
            }
            g_free(buf);
            s->sr &= ~SPI_SR_WEL;
        }
    }

    s->fmc_int |= FMC_INT_OP_DONE;
}

/* ── Control register access ─────────────────────────────────────────── */

static uint64_t hisi_fmc_ctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiFmcState *s = HISI_FMC(opaque);

    switch (offset) {
    case FMC_CFG:           return s->cfg;
    case FMC_GLOBAL_CFG:    return s->global_cfg;
    case FMC_SPI_TIMING_CFG: return s->spi_timing;
    case FMC_INT:           return s->fmc_int;
    case FMC_INT_EN:        return s->int_en;
    case FMC_CMD:           return s->cmd;
    case FMC_ADDRH:         return s->addrh;
    case FMC_ADDRL:         return s->addrl;
    case FMC_OP_CFG:        return s->op_cfg;
    case FMC_SPI_OP_ADDR:   return s->spi_op_addr;
    case FMC_DATA_NUM:      return s->data_num;
    case FMC_OP:            return 0; /* REG_OP_START always reads as clear */
    case FMC_DMA_LEN:       return s->dma_len;
    case FMC_DMA_SADDR_D0:  return s->dma_saddr;
    case FMC_OP_CTRL:       return 0; /* DMA_OP_READY always reads as clear */
    case FMC_STATUS:        return s->status;
    case FMC_VERSION:       return 0x100; /* HIFMC_VER_100 */
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: read from unknown reg 0x%03x\n",
                      (unsigned)offset);
        return 0;
    }
}

static void hisi_fmc_ctrl_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    HisiFmcState *s = HISI_FMC(opaque);

    switch (offset) {
    case FMC_CFG:           s->cfg = value;           break;
    case FMC_GLOBAL_CFG:    s->global_cfg = value;    break;
    case FMC_SPI_TIMING_CFG: s->spi_timing = value;   break;
    case FMC_INT_EN:        s->int_en = value;        break;
    case FMC_INT_CLR:       s->fmc_int &= ~value;     break;
    case FMC_CMD:           s->cmd = value;           break;
    case FMC_ADDRH:         s->addrh = value;         break;
    case FMC_ADDRL:         s->addrl = value;         break;
    case FMC_OP_CFG:        s->op_cfg = value;        break;
    case FMC_SPI_OP_ADDR:   s->spi_op_addr = value;   break;
    case FMC_DATA_NUM:      s->data_num = value;      break;
    case FMC_DMA_LEN:       s->dma_len = value;       break;
    case FMC_DMA_SADDR_D0:  s->dma_saddr = value;     break;

    case FMC_OP:
        if (value & FMC_OP_REG_OP_START) {
            if (value & FMC_OP_READ_STATUS_EN) {
                /* Hardware reads SPI status register directly into FMC_STATUS */
                s->status = s->sr;
            } else {
                hisi_fmc_exec_reg_op(s);
            }
            s->fmc_int |= FMC_INT_OP_DONE;
        }
        break;

    case FMC_OP_CTRL:
        s->op_ctrl = value;
        if (value & FMC_OP_CTRL_DMA_OP_READY) {
            hisi_fmc_exec_dma_op(s);
        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: write to unknown reg 0x%03x = 0x%08x\n",
                      (unsigned)offset, (unsigned)value);
        break;
    }
}

static const MemoryRegionOps hisi_fmc_ctrl_ops = {
    .read  = hisi_fmc_ctrl_read,
    .write = hisi_fmc_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── Memory window access (register-mode I/O buffer) ─────────────────── */

static uint64_t hisi_fmc_mem_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiFmcState *s = HISI_FMC(opaque);

    if (offset < IOBUF_SIZE) {
        if (size == 4 && (offset & 3) == 0) {
            return ldl_le_p(&s->iobuf[offset]);
        }
        return s->iobuf[offset];
    }
    return 0;
}

static void hisi_fmc_mem_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    HisiFmcState *s = HISI_FMC(opaque);

    if (offset < IOBUF_SIZE) {
        if (size == 4 && (offset & 3) == 0) {
            stl_le_p(&s->iobuf[offset], value);
        } else {
            s->iobuf[offset] = value & 0xFF;
        }
    }
}

static const MemoryRegionOps hisi_fmc_mem_ops = {
    .read  = hisi_fmc_mem_read,
    .write = hisi_fmc_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ── Device lifecycle ────────────────────────────────────────────────── */

static void hisi_fmc_realize(DeviceState *dev, Error **errp)
{
    HisiFmcState *s = HISI_FMC(dev);

    s->flash = g_malloc(s->flash_size);
    memset(s->flash, 0xFF, s->flash_size);
}

static void hisi_fmc_init(Object *obj)
{
    HisiFmcState *s = HISI_FMC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->ctrl_iomem, obj, &hisi_fmc_ctrl_ops, s,
                          "hisi-fmc.ctrl", CTRL_REG_SIZE);
    sysbus_init_mmio(sbd, &s->ctrl_iomem);

    memory_region_init_io(&s->mem_iomem, obj, &hisi_fmc_mem_ops, s,
                          "hisi-fmc.mem", MEM_WINDOW_SIZE);
    sysbus_init_mmio(sbd, &s->mem_iomem);
}

static void hisi_fmc_finalize(Object *obj)
{
    HisiFmcState *s = HISI_FMC(obj);
    g_free(s->flash);
}

static const Property hisi_fmc_properties[] = {
    DEFINE_PROP_UINT32("flash-size", HisiFmcState, flash_size,
                       FLASH_SIZE_DEFAULT),
};

static void hisi_fmc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = hisi_fmc_realize;
    device_class_set_props(dc, hisi_fmc_properties);
}

static const TypeInfo hisi_fmc_info = {
    .name          = TYPE_HISI_FMC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiFmcState),
    .instance_init = hisi_fmc_init,
    .instance_finalize = hisi_fmc_finalize,
    .class_init    = hisi_fmc_class_init,
};

static void hisi_fmc_register_types(void)
{
    type_register_static(&hisi_fmc_info);
}

type_init(hisi_fmc_register_types)
