/*
 * HiSilicon HiFMC V100 Flash Memory Controller emulation.
 *
 * Emulates the FMC found on Hi3516CV300 / Hi3516EV300 SoCs, with a
 * RAM-backed SPI NOR flash (Winbond W25Q64, 8 MiB) or SPI NAND flash
 * (Winbond W25N01GV, 128 MiB).  Supports both register-mode (small
 * reads via memory window) and DMA-mode transfers.
 *
 * The flash-type property selects which flash is physically present:
 *   0 = SPI NOR  (default, W25Q64 8 MiB)
 *   1 = SPI NAND (W25N01GV 128 MiB, 2K+64 pages)
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
#include <sys/stat.h>

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
#define FMC_DMA_AHB_CTRL    0x48
#define FMC_DMA_SADDR_D0    0x4C
#define FMC_DMA_SADDR_OOB   0x5C
#define FMC_OP_CTRL         0x68
#define FMC_STATUS          0xAC
#define FMC_VERSION         0xBC
#define FMC_DMA_SADDRH_D0   0x200
#define FMC_DMA_SADDRH_OOB  0x210

/* FMC_CFG bits */
#define FMC_CFG_FLASH_SEL_SHIFT 1
#define FMC_CFG_FLASH_SEL_MASK  (0x3 << FMC_CFG_FLASH_SEL_SHIFT)
#define FLASH_TYPE_SPI_NOR      0
#define FLASH_TYPE_SPI_NAND     1

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
#define FMC_OP_CTRL_RD_OP_SEL_SHIFT 4
#define FMC_OP_CTRL_RD_OPCODE_SHIFT 16

/* FMC_INT bits */
#define FMC_INT_OP_DONE     BIT(0)

/* SPI commands (shared NOR + NAND) */
#define SPI_CMD_WRITE_ENABLE  0x06
#define SPI_CMD_READ_STATUS   0x05
#define SPI_CMD_READ_ID       0x9F
#define SPI_CMD_READ          0x03
#define SPI_CMD_FAST_READ     0x0B
#define SPI_CMD_PAGE_PROGRAM  0x02
#define SPI_CMD_SECTOR_ERASE  0xD8
#define SPI_CMD_CHIP_ERASE    0xC7

/* SPI NAND specific commands */
#define SPI_CMD_RESET         0xFF
#define SPI_CMD_GET_FEATURES  0x0F
#define SPI_CMD_SET_FEATURE   0x1F

/* NAND feature register addresses */
#define NAND_FEATURE_PROTECT  0xA0
#define NAND_FEATURE_FEATURE  0xB0
#define NAND_FEATURE_STATUS   0xC0

/* SPI NOR additional commands */
#define SPI_CMD_WRITE_DISABLE 0x04
#define SPI_CMD_WRITE_STATUS1 0x01
#define SPI_CMD_READ_STATUS2  0x35
#define SPI_CMD_WRITE_STATUS2 0x31
#define SPI_CMD_READ_STATUS3  0x15
#define SPI_CMD_WRITE_STATUS3 0x11
#define SPI_CMD_READ_SFDP     0x5A

/* SPI status register bits */
#define SPI_SR_WEL          BIT(1)

/* NOR flash identity defaults: Winbond W25Q64 (8 MiB) */
#define NOR_JEDEC_0         0xEF    /* Winbond */
#define NOR_JEDEC_1         0x40
#define NOR_JEDEC_2_8M      0x17    /* 64Mbit = 8 MiB (W25Q64) */
#define NOR_JEDEC_2_16M     0x18    /* 128Mbit = 16 MiB (W25Q128) */
#define NOR_FLASH_SIZE      (8 * 1024 * 1024)
#define NOR_SECTOR_SIZE     (64 * 1024)
#define NOR_PAGE_SIZE       256

/* NAND flash identity: Winbond W25N01GV (128 MiB) */
#define NAND_JEDEC_0        0xEF    /* Winbond */
#define NAND_JEDEC_1        0xAA
#define NAND_JEDEC_2        0x21
#define NAND_FLASH_SIZE     (128 * 1024 * 1024)
#define NAND_PAGE_SIZE      2048
#define NAND_OOB_SIZE       64
#define NAND_PAGES_PER_BLOCK 64
#define NAND_BLOCK_SIZE     (NAND_PAGE_SIZE * NAND_PAGES_PER_BLOCK)
#define NAND_BLOCK_COUNT    1024
#define NAND_TOTAL_PAGES    (NAND_BLOCK_COUNT * NAND_PAGES_PER_BLOCK)

#define IOBUF_SIZE          256
#define CTRL_REG_SIZE       0x1000
#define MEM_WINDOW_SIZE     0x1000000   /* 16 MiB — covers full SPI NOR flash */

/* ── Device state ────────────────────────────────────────────────────── */

#define TYPE_HISI_FMC "hisi-fmc"
OBJECT_DECLARE_SIMPLE_TYPE(HisiFmcState, HISI_FMC)

struct HisiFmcState {
    SysBusDevice parent_obj;

    MemoryRegion ctrl_iomem;
    MemoryRegion mem_iomem;

    /* Configuration properties */
    uint32_t flash_type;          /* 0=NOR, 1=NAND */
    char    *flash_file;          /* optional: path to flash dump to load */
    uint32_t flash_jedec;         /* optional: JEDEC ID override (e.g. 0xEF4018) */

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
    uint32_t dma_ahb_ctrl;
    uint32_t dma_saddr_oob;
    uint32_t dma_saddrh_d0;
    uint32_t dma_saddrh_oob;
    uint32_t op_ctrl;
    uint32_t status;

    /* SPI flash state */
    uint8_t  sr;
    uint8_t  sr2;                 /* Status Register-2 (QE bit at bit 1) */
    uint8_t  sr3;                 /* Status Register-3 */
    uint8_t *flash;
    uint32_t flash_size;

    /* NAND-specific state */
    uint8_t *nand_oob;            /* OOB area: NAND_OOB_SIZE per page */
    uint8_t  nand_feature_protect; /* feature reg 0xA0 */
    uint8_t  nand_feature_config;  /* feature reg 0xB0 */

    /* Register-mode I/O buffer (shared via memory window) */
    bool     iobuf_valid;         /* true after reg-mode op, cleared on next read */
    uint8_t  iobuf[IOBUF_SIZE];
};

/* ── Helpers ─────────────────────────────────────────────────────────── */

/* Return current flash selection from FMC_CFG register */
static inline int hisi_fmc_current_flash_sel(HisiFmcState *s)
{
    return (s->cfg >> FMC_CFG_FLASH_SEL_SHIFT) & 0x3;
}

/* Decode NAND page address from ADDRL/ADDRH */
static void hisi_fmc_nand_decode_addr(HisiFmcState *s,
                                       uint32_t *block_out,
                                       uint32_t *page_out,
                                       uint32_t *column_out)
{
    uint32_t block_lo = (s->addrl >> 22) & 0x3FF;
    uint32_t block_hi = (s->addrh & 0xFF) << 10;
    *block_out = block_hi | block_lo;
    *page_out = (s->addrl >> 16) & 0x3F;
    *column_out = s->addrl & 0xFFF;
}

/* ── SPI NOR command execution (register mode) ──────────────────────── */

/* Returns true if the command wrote results to iobuf */
static bool hisi_fmc_exec_nor_reg_op(HisiFmcState *s)
{
    uint8_t spi_cmd = s->cmd & 0xFF;
    uint32_t addr = s->addrl;
    uint32_t len = s->data_num & 0x3FFF;

    if (len > IOBUF_SIZE) {
        len = IOBUF_SIZE;
    }

    switch (spi_cmd) {
    case SPI_CMD_READ_ID:
        if (s->flash_jedec) {
            /* User-specified JEDEC ID (e.g. 0x0B4018 for XTX XT25F128B) */
            s->iobuf[0] = (s->flash_jedec >> 16) & 0xFF;
            s->iobuf[1] = (s->flash_jedec >> 8) & 0xFF;
            s->iobuf[2] = s->flash_jedec & 0xFF;
        } else {
            /* Default: Winbond, size derived from flash */
            s->iobuf[0] = NOR_JEDEC_0;
            s->iobuf[1] = NOR_JEDEC_1;
            s->iobuf[2] = (s->flash_size > NOR_FLASH_SIZE)
                           ? NOR_JEDEC_2_16M : NOR_JEDEC_2_8M;
        }
        if (len > 3) {
            memset(&s->iobuf[3], 0, len - 3);
        }
        break;

    case SPI_CMD_READ_STATUS:
        s->iobuf[0] = s->sr;
        break;

    case SPI_CMD_READ_STATUS2:
        s->iobuf[0] = s->sr2;
        break;

    case SPI_CMD_WRITE_STATUS2:
        if (s->sr & SPI_SR_WEL) {
            s->sr2 = s->iobuf[0];
            s->sr &= ~SPI_SR_WEL;
        }
        break;

    case SPI_CMD_READ_STATUS3:
        s->iobuf[0] = s->sr3;
        break;

    case SPI_CMD_WRITE_STATUS3:
        if (s->sr & SPI_SR_WEL) {
            s->sr3 = s->iobuf[0];
            s->sr &= ~SPI_SR_WEL;
        }
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

    case SPI_CMD_WRITE_DISABLE:
        s->sr &= ~SPI_SR_WEL;
        break;

    case SPI_CMD_WRITE_STATUS1:
        if (s->sr & SPI_SR_WEL) {
            s->sr = (s->iobuf[0] & ~SPI_SR_WEL);
        }
        break;

    case SPI_CMD_READ_SFDP:
    case 0x98: /* Global Block Unlock / Resume from Suspend */
        /* Not implemented — return 0xFF (empty/no-op) */
        memset(s->iobuf, 0xFF, len);
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
            uint32_t base = addr & ~(NOR_SECTOR_SIZE - 1);
            uint32_t end = base + NOR_SECTOR_SIZE;
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
                      "hisi-fmc: NOR unhandled SPI command 0x%02x\n", spi_cmd);
        return false;
    }
    return true;
}

/* ── SPI NAND command execution (register mode) ─────────────────────── */

static void hisi_fmc_exec_nand_reg_op(HisiFmcState *s)
{
    uint8_t spi_cmd = s->cmd & 0xFF;
    uint32_t len = s->data_num & 0x3FFF;
    uint8_t feat_addr;

    if (len > IOBUF_SIZE) {
        len = IOBUF_SIZE;
    }

    switch (spi_cmd) {
    case SPI_CMD_READ_ID:
        s->iobuf[0] = NAND_JEDEC_0;
        s->iobuf[1] = NAND_JEDEC_1;
        s->iobuf[2] = NAND_JEDEC_2;
        if (len > 3) {
            memset(&s->iobuf[3], 0, len - 3);
        }
        break;

    case SPI_CMD_RESET:
        s->sr = 0;
        s->nand_feature_protect = 0;
        s->nand_feature_config = 0;
        break;

    case SPI_CMD_WRITE_ENABLE:
        s->sr |= SPI_SR_WEL;
        break;

    case SPI_CMD_GET_FEATURES:
        feat_addr = s->addrl & 0xFF;
        switch (feat_addr) {
        case NAND_FEATURE_PROTECT:
            s->iobuf[0] = s->nand_feature_protect;
            break;
        case NAND_FEATURE_FEATURE:
            s->iobuf[0] = s->nand_feature_config;
            break;
        case NAND_FEATURE_STATUS:
            s->iobuf[0] = s->sr;
            break;
        default:
            s->iobuf[0] = 0;
            break;
        }
        break;

    case SPI_CMD_SET_FEATURE:
        feat_addr = s->addrl & 0xFF;
        switch (feat_addr) {
        case NAND_FEATURE_PROTECT:
            s->nand_feature_protect = s->iobuf[0];
            break;
        case NAND_FEATURE_FEATURE:
            s->nand_feature_config = s->iobuf[0];
            break;
        case NAND_FEATURE_STATUS:
            /* Status register: only WEL is writable by SET_FEATURE in practice */
            break;
        default:
            break;
        }
        break;

    case SPI_CMD_SECTOR_ERASE: { /* 0xD8 = Block Erase for NAND */
        if (s->sr & SPI_SR_WEL) {
            uint32_t block, page, column;
            hisi_fmc_nand_decode_addr(s, &block, &page, &column);
            if (block < NAND_BLOCK_COUNT) {
                uint32_t flash_off = block * NAND_BLOCK_SIZE;
                uint32_t oob_off = block * NAND_PAGES_PER_BLOCK * NAND_OOB_SIZE;
                memset(&s->flash[flash_off], 0xFF, NAND_BLOCK_SIZE);
                memset(&s->nand_oob[oob_off], 0xFF,
                       NAND_PAGES_PER_BLOCK * NAND_OOB_SIZE);
            }
            s->sr &= ~SPI_SR_WEL;
        }
        break;
    }

    default:
        qemu_log_mask(LOG_UNIMP,
                      "hisi-fmc: NAND unhandled SPI command 0x%02x\n", spi_cmd);
        break;
    }
}

/* ── Register-mode dispatch ──────────────────────────────────────────── */

/* Returns true if the command wrote results to iobuf */
static bool hisi_fmc_exec_reg_op(HisiFmcState *s)
{
    bool wrote_iobuf = true;
    if (s->flash_type == FLASH_TYPE_SPI_NAND &&
        hisi_fmc_current_flash_sel(s) == FLASH_TYPE_SPI_NAND) {
        hisi_fmc_exec_nand_reg_op(s);
    } else if (s->flash_type == FLASH_TYPE_SPI_NOR &&
               hisi_fmc_current_flash_sel(s) == FLASH_TYPE_SPI_NOR) {
        wrote_iobuf = hisi_fmc_exec_nor_reg_op(s);
    } else {
        /* Wrong flash selected — no chip responds, return 0x00 ID */
        uint32_t len = s->data_num & 0x3FFF;
        if (len > IOBUF_SIZE) {
            len = IOBUF_SIZE;
        }
        memset(s->iobuf, 0x00, len);
    }
    s->fmc_int |= FMC_INT_OP_DONE;
    return wrote_iobuf;
}

/* ── DMA command execution ───────────────────────────────────────────── */

static void hisi_fmc_exec_dma_nor(HisiFmcState *s)
{
    uint32_t addr = s->addrl;
    uint32_t len = s->dma_len;
    bool is_write = s->op_ctrl & FMC_OP_CTRL_RW_OP;
    uint64_t dma_addr = ((uint64_t)s->dma_saddrh_d0 << 32) | s->dma_saddr;

    if (addr + len > s->flash_size) {
        len = (addr < s->flash_size) ? s->flash_size - addr : 0;
    }

    if (len == 0) {
        return;
    }

    if (!is_write) {
        dma_memory_write(&address_space_memory, dma_addr,
                         &s->flash[addr], len, MEMTXATTRS_UNSPECIFIED);
    } else {
        if (s->sr & SPI_SR_WEL) {
            uint8_t *buf = g_malloc(len);
            dma_memory_read(&address_space_memory, dma_addr,
                            buf, len, MEMTXATTRS_UNSPECIFIED);
            for (uint32_t i = 0; i < len; i++) {
                s->flash[addr + i] &= buf[i];
            }
            g_free(buf);
            s->sr &= ~SPI_SR_WEL;
        }
    }
}

static void hisi_fmc_exec_dma_nand(HisiFmcState *s)
{
    uint32_t block, page, column;
    bool is_write = s->op_ctrl & FMC_OP_CTRL_RW_OP;
    uint64_t dma_addr = ((uint64_t)s->dma_saddrh_d0 << 32) | s->dma_saddr;
    uint64_t dma_oob_addr = ((uint64_t)s->dma_saddrh_oob << 32)
                            | s->dma_saddr_oob;

    hisi_fmc_nand_decode_addr(s, &block, &page, &column);

    if (block >= NAND_BLOCK_COUNT || page >= NAND_PAGES_PER_BLOCK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-fmc: NAND DMA out of range block=%u page=%u\n",
                      block, page);
        return;
    }

    uint32_t page_idx = block * NAND_PAGES_PER_BLOCK + page;
    uint32_t flash_off = page_idx * NAND_PAGE_SIZE;
    uint32_t oob_off = page_idx * NAND_OOB_SIZE;
    uint32_t dma_len = s->dma_len;

    if (dma_len > NAND_PAGE_SIZE) {
        dma_len = NAND_PAGE_SIZE;
    }

    if (!is_write) {
        /* DMA read: flash → guest memory (page data) */
        if (dma_len > 0) {
            dma_memory_write(&address_space_memory, dma_addr,
                             &s->flash[flash_off], dma_len,
                             MEMTXATTRS_UNSPECIFIED);
        }
        /* OOB → guest memory */
        if (s->dma_saddr_oob) {
            dma_memory_write(&address_space_memory, dma_oob_addr,
                             &s->nand_oob[oob_off], NAND_OOB_SIZE,
                             MEMTXATTRS_UNSPECIFIED);
        }
    } else {
        /* DMA write: guest memory → flash (page program) */
        if (s->sr & SPI_SR_WEL) {
            if (dma_len > 0) {
                uint8_t *buf = g_malloc(dma_len);
                dma_memory_read(&address_space_memory, dma_addr,
                                buf, dma_len, MEMTXATTRS_UNSPECIFIED);
                /* NAND program: can only clear bits */
                for (uint32_t i = 0; i < dma_len; i++) {
                    s->flash[flash_off + i] &= buf[i];
                }
                g_free(buf);
            }
            /* OOB write */
            if (s->dma_saddr_oob) {
                uint8_t oob_buf[NAND_OOB_SIZE];
                dma_memory_read(&address_space_memory, dma_oob_addr,
                                oob_buf, NAND_OOB_SIZE,
                                MEMTXATTRS_UNSPECIFIED);
                for (int i = 0; i < NAND_OOB_SIZE; i++) {
                    s->nand_oob[oob_off + i] &= oob_buf[i];
                }
            }
            s->sr &= ~SPI_SR_WEL;
        }
    }
}

static void hisi_fmc_exec_dma_op(HisiFmcState *s)
{
    if (s->flash_type == FLASH_TYPE_SPI_NAND &&
        hisi_fmc_current_flash_sel(s) == FLASH_TYPE_SPI_NAND) {
        hisi_fmc_exec_dma_nand(s);
    } else if (s->flash_type == FLASH_TYPE_SPI_NOR &&
               hisi_fmc_current_flash_sel(s) == FLASH_TYPE_SPI_NOR) {
        hisi_fmc_exec_dma_nor(s);
    }
    /* else: wrong flash type selected, no-op */

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
    case FMC_DMA_AHB_CTRL:  return s->dma_ahb_ctrl;
    case FMC_DMA_SADDR_D0:  return s->dma_saddr;
    case FMC_DMA_SADDR_OOB: return s->dma_saddr_oob;
    case FMC_OP_CTRL:       return 0; /* DMA_OP_READY always reads as clear */
    case FMC_STATUS:        return s->status;
    case FMC_VERSION:       return 0x100; /* HIFMC_VER_100 */
    case FMC_DMA_SADDRH_D0: return s->dma_saddrh_d0;
    case FMC_DMA_SADDRH_OOB: return s->dma_saddrh_oob;
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
    case FMC_DMA_AHB_CTRL:  s->dma_ahb_ctrl = value;  break;
    case FMC_DMA_SADDR_D0:  s->dma_saddr = value;     break;
    case FMC_DMA_SADDR_OOB: s->dma_saddr_oob = value; break;
    case FMC_DMA_SADDRH_D0: s->dma_saddrh_d0 = value; break;
    case FMC_DMA_SADDRH_OOB: s->dma_saddrh_oob = value; break;

    case FMC_OP:
        s->iobuf_valid = false;
        if (value & FMC_OP_REG_OP_START) {
            if (value & FMC_OP_READ_STATUS_EN) {
                /* Hardware reads SPI status register directly into FMC_STATUS */
                s->status = s->sr;
                s->iobuf_valid = true;
            } else {
                /* Only mark iobuf valid if the command actually wrote to it.
                 * Unknown commands (block lock, etc.) must NOT set iobuf_valid
                 * or subsequent memory-window reads return stale iobuf data
                 * instead of actual flash content. */
                s->iobuf_valid = hisi_fmc_exec_reg_op(s);
            }
            s->fmc_int |= FMC_INT_OP_DONE;
        }
        break;

    case FMC_OP_CTRL:
        s->op_ctrl = value;
        s->iobuf_valid = false;
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

    /* After a register-mode SPI operation, the result is in iobuf.
     * U-Boot/kernel reads the result from low offsets of the memory window.
     * The flag stays set until the next FMC_OP write clears it. */
    if (s->iobuf_valid && offset < IOBUF_SIZE) {
        if (size == 4 && (offset & 3) == 0) {
            return ldl_le_p(&s->iobuf[offset]);
        }
        return s->iobuf[offset];
    }

    /* Direct flash read — memory-mapped XIP access used by boot ROM and
     * U-Boot to read flash contents from the FMC memory window. */
    if (s->flash && offset < s->flash_size) {
        if (size == 4 && (offset & 3) == 0) {
            return ldl_le_p(&s->flash[offset]);
        }
        if (size == 2 && (offset & 1) == 0) {
            return lduw_le_p(&s->flash[offset]);
        }
        return s->flash[offset];
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

    if (s->flash_type == FLASH_TYPE_SPI_NAND) {
        s->flash_size = NAND_FLASH_SIZE;
        s->flash = g_malloc(NAND_FLASH_SIZE);
        memset(s->flash, 0xFF, NAND_FLASH_SIZE);
        s->nand_oob = g_malloc(NAND_TOTAL_PAGES * NAND_OOB_SIZE);
        memset(s->nand_oob, 0xFF, NAND_TOTAL_PAGES * NAND_OOB_SIZE);
        /* Set initial FMC_CFG to SPI_NAND */
        s->cfg = (s->cfg & ~FMC_CFG_FLASH_SEL_MASK) |
                 (FLASH_TYPE_SPI_NAND << FMC_CFG_FLASH_SEL_SHIFT);
    } else {
        s->flash_size = NOR_FLASH_SIZE;
        s->flash = g_malloc(NOR_FLASH_SIZE);
        memset(s->flash, 0xFF, NOR_FLASH_SIZE);
        /* FMC_CFG defaults to 0 (SPI_NOR) */
        /* Many NOR flash chips ship with QE (Quad Enable) bit set in SR2.
         * XM firmware expects to read SR2 and find QE=1, then disables it.
         * Default 0x02 matches common factory programming. */
        s->sr2 = 0x02;
    }

    /* Load flash contents from file if specified */
    if (s->flash_file && s->flash_file[0]) {
        struct stat st;
        if (stat(s->flash_file, &st) != 0) {
            error_setg(errp, "hisi-fmc: cannot stat flash file '%s'",
                       s->flash_file);
            return;
        }
        uint32_t file_size = (uint32_t)st.st_size;
        if (file_size > s->flash_size) {
            /* Re-allocate to fit the larger dump */
            g_free(s->flash);
            s->flash_size = file_size;
            s->flash = g_malloc(file_size);
            memset(s->flash, 0xFF, file_size);
        }
        FILE *f = fopen(s->flash_file, "rb");
        if (!f) {
            error_setg(errp, "hisi-fmc: cannot open flash file '%s'",
                       s->flash_file);
            return;
        }
        size_t nread = fread(s->flash, 1, file_size, f);
        fclose(f);
        if (nread != file_size) {
            error_setg(errp, "hisi-fmc: short read from '%s' "
                       "(got %zu, expected %u)", s->flash_file,
                       nread, file_size);
            return;
        }
        qemu_log("hisi-fmc: loaded %u bytes from '%s'\n",
                 file_size, s->flash_file);
    }
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
    g_free(s->nand_oob);
}

static const Property hisi_fmc_properties[] = {
    DEFINE_PROP_UINT32("flash-type", HisiFmcState, flash_type,
                       FLASH_TYPE_SPI_NOR),
    DEFINE_PROP_STRING("flash-file", HisiFmcState, flash_file),
    DEFINE_PROP_UINT32("flash-jedec", HisiFmcState, flash_jedec, 0),
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
