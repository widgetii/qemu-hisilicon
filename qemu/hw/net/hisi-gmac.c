/*
 * HiSilicon Gigabit Ethernet MAC (HiGMAC) emulation.
 *
 * Emulates the HiGMAC V200 found in Hi3516AV100 and Hi3519V101 SoCs.
 * Compatible with kernel drivers:
 *   - drivers/net/ethernet/hisilicon/higmac/   (MAC + DMA)
 *   - drivers/net/phy/mdio-hisi-gemac.c        (MDIO bus)
 *
 * The device exposes a 4 KiB MMIO region containing:
 *   0x0000-0x004F  MAC configuration (station addr, port mode/enable)
 *   0x0064-0x006F  Receive filter, multicast address
 *   0x0500-0x05CF  DMA descriptor ring control (RX_FQ, RX_BQ, TX_BQ, TX_RQ)
 *   0x05C0-0x05E8  Interrupt and control registers
 *   0x03C0-0x03D4  MDIO registers (separate kernel driver maps this sub-region)
 *
 * DMA uses four descriptor rings:
 *   RX_FQ: SW pushes empty buffers, HW pops for receiving
 *   RX_BQ: HW pushes received packets, SW pops and processes
 *   TX_BQ: SW pushes packets to send, HW pops and transmits
 *   TX_RQ: HW pushes completed TX descriptors, SW reclaims
 *
 * Descriptor format (4-word, 16 bytes):
 *   word0: data_buff_addr (physical)
 *   word1: buffer_len[10:0] | reserved[15:11] | data_len[26:16] |
 *          reserved[28:27] | fl[30:29] | descvid[31]
 *   word2: rxhash
 *   word3: reserved[7:0] | l3_hash[8] | has_hash[9] | skb_id[23:10] | reserved[31:24]
 *
 * MDIO: integrated PHY stub at address 1 presenting standard MII
 * registers.  Reports 100 Mbps full-duplex link permanently up.
 *
 * Copyright (c) 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "net/net.h"
#include "net/eth.h"
#include "net/checksum.h"
#include "system/dma.h"

/* ── MAC registers ──────────────────────────────────────────────────── */

#define STATION_ADDR_LOW        0x0000
#define STATION_ADDR_HIGH       0x0004
#define MAC_DUPLEX_HALF_CTRL    0x0008
#define PORT_MODE               0x0040
#define PORT_EN                 0x0044
#define  BITS_TX_EN             BIT(2)
#define  BITS_RX_EN             BIT(1)
#define REC_FILT_CONTROL        0x0064
#define PORT_MC_ADDR_LOW        0x0068
#define PORT_MC_ADDR_HIGH       0x006C
#define MODE_CHANGE_EN          0x01B4
#define CRF_MIN_PACKET          0x0210
#define CONTROL_WORD            0x0214
#define TSO_COE_CTRL            0x02E8

/* ── DMA descriptor ring registers ──────────────────────────────────── */

#define RX_FQ_START_ADDR        0x0500
#define RX_FQ_DEPTH             0x0504
#define RX_FQ_WR_ADDR           0x0508
#define RX_FQ_RD_ADDR           0x050C
#define RX_FQ_VLDDESC_CNT       0x0510
#define RX_FQ_ALEMPTY_TH        0x0514
#define RX_FQ_REG_EN            0x0518
#define RX_FQ_ALFULL_TH         0x051C

#define RX_BQ_START_ADDR        0x0520
#define RX_BQ_DEPTH             0x0524
#define RX_BQ_WR_ADDR           0x0528
#define RX_BQ_RD_ADDR           0x052C
#define RX_BQ_FREE_DESC_CNT     0x0530
#define RX_BQ_ALEMPTY_TH        0x0534
#define RX_BQ_REG_EN            0x0538
#define RX_BQ_ALFULL_TH         0x053C

#define TX_BQ_START_ADDR        0x0580
#define TX_BQ_DEPTH             0x0584
#define TX_BQ_WR_ADDR           0x0588
#define TX_BQ_RD_ADDR           0x058C
#define TX_BQ_VLDDESC_CNT       0x0590
#define TX_BQ_ALEMPTY_TH        0x0594
#define TX_BQ_REG_EN            0x0598
#define TX_BQ_ALFULL_TH         0x059C

#define TX_RQ_START_ADDR        0x05A0
#define TX_RQ_DEPTH             0x05A4
#define TX_RQ_WR_ADDR           0x05A8
#define TX_RQ_RD_ADDR           0x05AC
#define TX_RQ_FREE_DESC_CNT     0x05B0
#define TX_RQ_ALEMPTY_TH        0x05B4
#define TX_RQ_REG_EN            0x05B8
#define TX_RQ_ALFULL_TH         0x05BC

/* ── Interrupt / control registers ──────────────────────────────────── */

#define RAW_PMU_INT             0x05C0
#define ENA_PMU_INT             0x05C4
#define DESC_WR_RD_ENA          0x05CC
#define STOP_CMD                0x05E8

#define RX_BQ_IN_INT            BIT(17)
#define TX_RQ_IN_INT            BIT(19)
#define RX_BQ_IN_TIMEOUT_INT    BIT(28)
#define TX_RQ_IN_TIMEOUT_INT    BIT(29)

/* ── MDIO registers (at offset 0x3C0 from GMAC base) ───────────────── */

#define MDIO_BASE               0x03C0
#define MDIO_SINGLE_CMD         0x0000
#define MDIO_SINGLE_DATA        0x0004
#define MDIO_RDATA_STATUS       0x0010
#define MDIO_START              BIT(20)
#define MDIO_READ               BIT(17)
#define MDIO_WRITE              BIT(16)
#define BIT_PHY_ADDR_OFFSET     8

/* ── Descriptor format ──────────────────────────────────────────────── */

/* 4-word (16 byte) descriptors */
#define DESC_SIZE               16
#define DESC_BYTE_SHIFT         4
#define DESC_VLD_FREE           0
#define DESC_VLD_BUSY           1
#define DESC_FL_FULL            3
#define DESC_NUM                1024
#define DESC_RING_SIZE          (DESC_NUM * DESC_SIZE)

/* ── PHY stub ───────────────────────────────────────────────────────── */

#define PHY_ADDR                1
#define PHY_ID1                 0x0044
#define PHY_ID2                 0x6161

/* ── Device state ───────────────────────────────────────────────────── */

#define TYPE_HISI_GMAC "hisi-gmac"
OBJECT_DECLARE_SIMPLE_TYPE(HisiGmacState, HISI_GMAC)

#define MMIO_SIZE               0x1000

struct HisiGmacState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq     irq;
    NICState    *nic;
    NICConf      conf;

    /* MAC registers */
    uint32_t station_addr_low;
    uint32_t station_addr_high;
    uint32_t port_mode;
    uint32_t port_en;
    uint32_t rec_filt;
    uint32_t mc_addr_low;
    uint32_t mc_addr_high;
    uint32_t control_word;

    /* Descriptor ring base addresses and pointers */
    uint32_t rx_fq_start;
    uint32_t rx_fq_depth;
    uint32_t rx_fq_wr;      /* software writes (pushes free bufs) */
    uint32_t rx_fq_rd;      /* hardware reads  (pops free bufs)   */

    uint32_t rx_bq_start;
    uint32_t rx_bq_depth;
    uint32_t rx_bq_wr;      /* hardware writes (pushes recv pkts) */
    uint32_t rx_bq_rd;      /* software reads  (pops recv pkts)   */

    uint32_t tx_bq_start;
    uint32_t tx_bq_depth;
    uint32_t tx_bq_wr;      /* software writes (pushes tx pkts)   */
    uint32_t tx_bq_rd;      /* hardware reads  (pops tx pkts)     */

    uint32_t tx_rq_start;
    uint32_t tx_rq_depth;
    uint32_t tx_rq_wr;      /* hardware writes (pushes tx done)   */
    uint32_t tx_rq_rd;      /* software reads  (reclaims tx done) */

    /* Interrupt state */
    uint32_t raw_pmu_int;
    uint32_t ena_pmu_int;
    uint32_t desc_wr_rd_ena;

    /* MDIO state */
    uint32_t mdio_cmd;
    uint32_t mdio_data;

    /* PHY registers */
    uint16_t phy_regs[32];

    /* Backing register array for misc registers */
    uint32_t regs[MMIO_SIZE / 4];
};

/* ── PHY emulation ──────────────────────────────────────────────────── */

static void hisi_gmac_phy_init(HisiGmacState *s)
{
    s->phy_regs[0]  = 0x3100;  /* BMCR: auto-neg, 100M, full-duplex */
    s->phy_regs[1]  = 0x786D;  /* BMSR: link up, auto-neg complete */
    s->phy_regs[2]  = PHY_ID1;
    s->phy_regs[3]  = PHY_ID2;
    s->phy_regs[4]  = 0x01E1;  /* ANAR */
    s->phy_regs[5]  = 0x45E1;  /* ANLPAR: partner 100M FD */
    s->phy_regs[6]  = 0x0001;  /* ANER */
    s->phy_regs[17] = 0x4000;  /* Vendor: speed indication 100M */
}

static uint16_t hisi_gmac_phy_read(HisiGmacState *s, int reg)
{
    if (reg < 32) {
        return s->phy_regs[reg];
    }
    return 0xFFFF;
}

static void hisi_gmac_phy_write(HisiGmacState *s, int reg, uint16_t val)
{
    if (reg == 0) {
        /* BMCR — accept writes but keep link up */
        s->phy_regs[0] = val & ~BIT(15); /* clear reset bit */
        s->phy_regs[1] |= BIT(2) | BIT(5); /* link up, auto-neg complete */
    } else if (reg < 32 && reg >= 4) {
        s->phy_regs[reg] = val;
    }
}

/* ── MDIO command execution ─────────────────────────────────────────── */

static void hisi_gmac_mdio_exec(HisiGmacState *s)
{
    uint32_t cmd = s->mdio_cmd;
    int phy_addr = (cmd >> BIT_PHY_ADDR_OFFSET) & 0x1F;
    int reg = cmd & 0x1F;

    if (phy_addr != PHY_ADDR) {
        /* No chip at this address */
        s->mdio_data = 0xFFFF << 16;
        s->mdio_cmd &= ~MDIO_START;
        return;
    }

    if (cmd & MDIO_WRITE) {
        hisi_gmac_phy_write(s, reg, s->mdio_data & 0xFFFF);
    } else if (cmd & MDIO_READ) {
        s->mdio_data = (uint32_t)hisi_gmac_phy_read(s, reg) << 16;
    }

    s->mdio_cmd &= ~MDIO_START;
}

/* ── IRQ handling ───────────────────────────────────────────────────── */

static void hisi_gmac_update_irq(HisiGmacState *s)
{
    bool level = (s->raw_pmu_int & s->ena_pmu_int) != 0;
    qemu_set_irq(s->irq, level);
}

/* ── Descriptor ring helpers ────────────────────────────────────────── */

static inline uint32_t desc_cnt(uint32_t byte_off)
{
    return byte_off >> DESC_BYTE_SHIFT;
}

static inline uint32_t desc_byte(uint32_t cnt)
{
    return cnt << DESC_BYTE_SHIFT;
}

static inline uint32_t ring_incr(uint32_t pos, uint32_t size)
{
    return (pos + 1) & (size - 1);
}

static inline uint32_t ring_space(uint32_t wr, uint32_t rd, uint32_t size)
{
    return (rd - wr - 1) & (size - 1);
}

static inline uint32_t ring_count(uint32_t wr, uint32_t rd, uint32_t size)
{
    return (wr - rd) & (size - 1);
}

/* ── TX path: process packets from TX_BQ ring ───────────────────────── */

static void hisi_gmac_tx(HisiGmacState *s)
{
    uint32_t depth = desc_cnt(s->tx_bq_depth);
    uint32_t rd, wr;
    uint8_t buf[2048];

    if (!depth || !(s->port_en & BITS_TX_EN) || !s->desc_wr_rd_ena) {
        return;
    }

    rd = desc_cnt(s->tx_bq_rd);
    wr = desc_cnt(s->tx_bq_wr);

    while (rd != wr) {
        uint32_t desc_addr = s->tx_bq_start + rd * DESC_SIZE;
        uint32_t desc[4];

        /* Read descriptor from guest memory */
        dma_memory_read(&address_space_memory, desc_addr,
                        desc, DESC_SIZE, MEMTXATTRS_UNSPECIFIED);

        uint32_t data_addr = desc[0];
        uint32_t w1 = desc[1];
        uint32_t data_len = (w1 >> 16) & 0x7FF;
        uint32_t descvid = (w1 >> 31) & 1;

        if (descvid != DESC_VLD_BUSY || data_len == 0 ||
            data_len > sizeof(buf)) {
            break;
        }

        /* Read packet data from guest memory */
        dma_memory_read(&address_space_memory, data_addr,
                        buf, data_len, MEMTXATTRS_UNSPECIFIED);

        /* Checksum offload */
        net_checksum_calculate(buf, data_len, CSUM_ALL);

        /* Send packet */
        qemu_send_packet(qemu_get_queue(s->nic), buf, data_len);

        /* Write completion descriptor to TX_RQ ring */
        uint32_t rq_depth = desc_cnt(s->tx_rq_depth);
        if (rq_depth) {
            uint32_t rq_wr = desc_cnt(s->tx_rq_wr);
            uint32_t rq_addr = s->tx_rq_start + rq_wr * DESC_SIZE;

            /* Copy descriptor to reclaim queue */
            desc[1] = (w1 & ~(1u << 31)); /* clear descvid */
            dma_memory_write(&address_space_memory, rq_addr,
                             desc, DESC_SIZE, MEMTXATTRS_UNSPECIFIED);

            rq_wr = ring_incr(rq_wr, rq_depth);
            s->tx_rq_wr = desc_byte(rq_wr);
        }

        rd = ring_incr(rd, depth);
    }

    s->tx_bq_rd = desc_byte(rd);

    /* Raise TX completion interrupt */
    if (desc_cnt(s->tx_rq_wr) != desc_cnt(s->tx_rq_rd)) {
        s->raw_pmu_int |= TX_RQ_IN_INT;
        hisi_gmac_update_irq(s);
    }
}

/* ── RX path: receive packet from network ───────────────────────────── */

static bool hisi_gmac_can_receive(NetClientState *nc)
{
    HisiGmacState *s = qemu_get_nic_opaque(nc);

    if (!(s->port_en & BITS_RX_EN) || !s->desc_wr_rd_ena) {
        return false;
    }

    /* Check if there are free buffers in RX_FQ */
    uint32_t depth = desc_cnt(s->rx_fq_depth);
    if (!depth) {
        return false;
    }
    uint32_t fq_rd = desc_cnt(s->rx_fq_rd);
    uint32_t fq_wr = desc_cnt(s->rx_fq_wr);
    return fq_rd != fq_wr;
}

static ssize_t hisi_gmac_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    HisiGmacState *s = qemu_get_nic_opaque(nc);
    uint32_t fq_depth = desc_cnt(s->rx_fq_depth);
    uint32_t bq_depth = desc_cnt(s->rx_bq_depth);

    if (!fq_depth || !bq_depth) {
        return -1;
    }
    if (!(s->port_en & BITS_RX_EN) || !s->desc_wr_rd_ena) {
        return -1;
    }

    uint32_t fq_rd = desc_cnt(s->rx_fq_rd);
    uint32_t fq_wr = desc_cnt(s->rx_fq_wr);

    if (fq_rd == fq_wr) {
        return -1; /* No free buffers */
    }

    /* Pop a descriptor from RX_FQ */
    uint32_t fq_addr = s->rx_fq_start + fq_rd * DESC_SIZE;
    uint32_t desc[4];
    dma_memory_read(&address_space_memory, fq_addr,
                    desc, DESC_SIZE, MEMTXATTRS_UNSPECIFIED);

    uint32_t data_addr = desc[0];
    uint32_t w1 = desc[1];
    uint32_t buffer_len = (w1 & 0x7FF) + 1;

    if (size > buffer_len) {
        return -1; /* Packet too large for buffer */
    }

    /* Write packet data to guest buffer */
    dma_memory_write(&address_space_memory, data_addr,
                     buf, size, MEMTXATTRS_UNSPECIFIED);

    /* Advance RX_FQ read pointer */
    fq_rd = ring_incr(fq_rd, fq_depth);
    s->rx_fq_rd = desc_byte(fq_rd);

    /* Push descriptor to RX_BQ with received length */
    uint32_t bq_wr = desc_cnt(s->rx_bq_wr);
    uint32_t bq_addr = s->rx_bq_start + bq_wr * DESC_SIZE;

    /* Update descriptor: set data_len, fl=FULL, descvid=BUSY */
    w1 = (w1 & 0x7FF) |            /* keep buffer_len */
         ((uint32_t)size << 16) |   /* data_len */
         (DESC_FL_FULL << 29) |     /* fl = full frame */
         (DESC_VLD_BUSY << 31);     /* descvid = busy */
    desc[1] = w1;

    dma_memory_write(&address_space_memory, bq_addr,
                     desc, DESC_SIZE, MEMTXATTRS_UNSPECIFIED);

    bq_wr = ring_incr(bq_wr, bq_depth);
    s->rx_bq_wr = desc_byte(bq_wr);

    /* Raise RX interrupt */
    s->raw_pmu_int |= RX_BQ_IN_INT;
    hisi_gmac_update_irq(s);

    return size;
}

/* ── MMIO read ──────────────────────────────────────────────────────── */

static uint64_t hisi_gmac_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiGmacState *s = HISI_GMAC(opaque);

    /* MDIO registers (separate kernel driver at base+0x3C0) */
    if (offset >= MDIO_BASE && offset < MDIO_BASE + 0x20) {
        switch (offset - MDIO_BASE) {
        case MDIO_SINGLE_CMD:   return s->mdio_cmd;
        case MDIO_SINGLE_DATA:  return s->mdio_data;
        case MDIO_RDATA_STATUS: return 0; /* valid */
        default: return 0;
        }
    }

    switch (offset) {
    case STATION_ADDR_LOW:   return s->station_addr_low;
    case STATION_ADDR_HIGH:  return s->station_addr_high;
    case PORT_MODE:          return s->port_mode;
    case PORT_EN:            return s->port_en;
    case REC_FILT_CONTROL:   return s->rec_filt;
    case PORT_MC_ADDR_LOW:   return s->mc_addr_low;
    case PORT_MC_ADDR_HIGH:  return s->mc_addr_high;
    case CONTROL_WORD:       return s->control_word;

    /* RX_FQ */
    case RX_FQ_START_ADDR:   return s->rx_fq_start;
    case RX_FQ_DEPTH:        return s->rx_fq_depth;
    case RX_FQ_WR_ADDR:      return s->rx_fq_wr;
    case RX_FQ_RD_ADDR:      return s->rx_fq_rd;
    case RX_FQ_REG_EN:       return 0;

    /* RX_BQ */
    case RX_BQ_START_ADDR:   return s->rx_bq_start;
    case RX_BQ_DEPTH:        return s->rx_bq_depth;
    case RX_BQ_WR_ADDR:      return s->rx_bq_wr;
    case RX_BQ_RD_ADDR:      return s->rx_bq_rd;
    case RX_BQ_REG_EN:       return 0;

    /* TX_BQ */
    case TX_BQ_START_ADDR:   return s->tx_bq_start;
    case TX_BQ_DEPTH:        return s->tx_bq_depth;
    case TX_BQ_WR_ADDR:      return s->tx_bq_wr;
    case TX_BQ_RD_ADDR:      return s->tx_bq_rd;
    case TX_BQ_REG_EN:       return 0;

    /* TX_RQ */
    case TX_RQ_START_ADDR:   return s->tx_rq_start;
    case TX_RQ_DEPTH:        return s->tx_rq_depth;
    case TX_RQ_WR_ADDR:      return s->tx_rq_wr;
    case TX_RQ_RD_ADDR:      return s->tx_rq_rd;
    case TX_RQ_REG_EN:       return 0;

    /* Interrupts */
    case RAW_PMU_INT:        return s->raw_pmu_int;
    case ENA_PMU_INT:        return s->ena_pmu_int;
    case DESC_WR_RD_ENA:     return s->desc_wr_rd_ena;
    case STOP_CMD:           return 0;

    default:
        if (offset < MMIO_SIZE) {
            return s->regs[offset / 4];
        }
        return 0;
    }
}

/* ── MMIO write ─────────────────────────────────────────────────────── */

static void hisi_gmac_write(void *opaque, hwaddr offset, uint64_t val,
                             unsigned size)
{
    HisiGmacState *s = HISI_GMAC(opaque);

    /* MDIO registers */
    if (offset >= MDIO_BASE && offset < MDIO_BASE + 0x20) {
        switch (offset - MDIO_BASE) {
        case MDIO_SINGLE_CMD:
            s->mdio_cmd = val;
            if (val & MDIO_START) {
                hisi_gmac_mdio_exec(s);
            }
            break;
        case MDIO_SINGLE_DATA:
            s->mdio_data = val;
            break;
        }
        return;
    }

    switch (offset) {
    case STATION_ADDR_LOW:   s->station_addr_low = val; break;
    case STATION_ADDR_HIGH:  s->station_addr_high = val; break;
    case PORT_MODE:          s->port_mode = val; break;
    case PORT_EN:            s->port_en = val; break;
    case REC_FILT_CONTROL:   s->rec_filt = val; break;
    case PORT_MC_ADDR_LOW:   s->mc_addr_low = val; break;
    case PORT_MC_ADDR_HIGH:  s->mc_addr_high = val; break;
    case CONTROL_WORD:       s->control_word = val; break;

    /* RX_FQ — writes gated by REG_EN */
    case RX_FQ_REG_EN:
        /* REG_EN latches one write then auto-clears */
        break;
    case RX_FQ_START_ADDR:   s->rx_fq_start = val; break;
    case RX_FQ_DEPTH:        s->rx_fq_depth = val; break;
    case RX_FQ_WR_ADDR:
        s->rx_fq_wr = val;
        /* New free buffers available — check if we can receive now */
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        break;
    case RX_FQ_RD_ADDR:      s->rx_fq_rd = val; break;

    /* RX_BQ */
    case RX_BQ_REG_EN:       break;
    case RX_BQ_START_ADDR:   s->rx_bq_start = val; break;
    case RX_BQ_DEPTH:        s->rx_bq_depth = val; break;
    case RX_BQ_WR_ADDR:      s->rx_bq_wr = val; break;
    case RX_BQ_RD_ADDR:      s->rx_bq_rd = val; break;

    /* TX_BQ */
    case TX_BQ_REG_EN:       break;
    case TX_BQ_START_ADDR:   s->tx_bq_start = val; break;
    case TX_BQ_DEPTH:        s->tx_bq_depth = val; break;
    case TX_BQ_WR_ADDR:
        s->tx_bq_wr = val;
        /* New TX packets queued — process them */
        hisi_gmac_tx(s);
        break;
    case TX_BQ_RD_ADDR:      s->tx_bq_rd = val; break;

    /* TX_RQ */
    case TX_RQ_REG_EN:       break;
    case TX_RQ_START_ADDR:   s->tx_rq_start = val; break;
    case TX_RQ_DEPTH:        s->tx_rq_depth = val; break;
    case TX_RQ_WR_ADDR:      s->tx_rq_wr = val; break;
    case TX_RQ_RD_ADDR:      s->tx_rq_rd = val; break;

    /* Interrupts */
    case RAW_PMU_INT:
        s->raw_pmu_int &= ~val; /* W1C */
        hisi_gmac_update_irq(s);
        break;
    case ENA_PMU_INT:
        s->ena_pmu_int = val;
        hisi_gmac_update_irq(s);
        break;
    case DESC_WR_RD_ENA:
        s->desc_wr_rd_ena = val;
        break;
    case STOP_CMD:
        if (val & BIT(0)) { /* RX stop */
            s->port_en &= ~BITS_RX_EN;
        }
        if (val & BIT(1)) { /* TX stop */
            s->port_en &= ~BITS_TX_EN;
        }
        break;

    default:
        if (offset < MMIO_SIZE) {
            s->regs[offset / 4] = val;
        }
        break;
    }
}

static const MemoryRegionOps hisi_gmac_ops = {
    .read = hisi_gmac_read,
    .write = hisi_gmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── NIC callbacks ──────────────────────────────────────────────────── */

static void hisi_gmac_set_link(NetClientState *nc)
{
    HisiGmacState *s = qemu_get_nic_opaque(nc);
    if (nc->link_down) {
        s->phy_regs[1] &= ~BIT(2); /* link down */
    } else {
        s->phy_regs[1] |= BIT(2);  /* link up */
    }
}

static NetClientInfo hisi_gmac_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = hisi_gmac_can_receive,
    .receive = hisi_gmac_receive,
    .link_status_changed = hisi_gmac_set_link,
};

/* ── Device lifecycle ───────────────────────────────────────────────── */

static void hisi_gmac_init(Object *obj)
{
    HisiGmacState *s = HISI_GMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_gmac_ops, s,
                          "hisi-gmac", MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void hisi_gmac_realize(DeviceState *dev, Error **errp)
{
    HisiGmacState *s = HISI_GMAC(dev);

    hisi_gmac_phy_init(s);

    s->nic = qemu_new_nic(&hisi_gmac_net_info, &s->conf,
                           object_get_typename(OBJECT(dev)),
                           dev->id, &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static const Property hisi_gmac_properties[] = {
    DEFINE_NIC_PROPERTIES(HisiGmacState, conf),
};

static void hisi_gmac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = hisi_gmac_realize;
    device_class_set_props(dc, hisi_gmac_properties);
}

static const TypeInfo hisi_gmac_info = {
    .name          = TYPE_HISI_GMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiGmacState),
    .instance_init = hisi_gmac_init,
    .class_init    = hisi_gmac_class_init,
};

static void hisi_gmac_register_types(void)
{
    type_register_static(&hisi_gmac_info);
}

type_init(hisi_gmac_register_types)
