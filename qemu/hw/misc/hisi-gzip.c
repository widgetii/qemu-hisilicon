/*
 * HiSilicon Hardware GZIP Decompressor emulation.
 *
 * Found on Hi3516EV300 and similar V4/V3.5 SoCs at 0x11310000 (EV300)
 * or 0x11310000 (CV500-era).  Used by U-Boot's hw_compressed first-stage
 * loader to decompress the real U-Boot binary from SPI NOR flash into DDR.
 *
 * The hardware is a DMA engine that reads gzip-compressed data from a
 * source address, decompresses it, and writes the result to a destination
 * address.  U-Boot polls the interrupt status register for completion.
 *
 * Register map derived from u-boot-2016.11/lib/hw_dec/hw_decompress_v1.h.
 *
 * Copyright (c) 2026 OpenIPC.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "system/dma.h"
#include <zlib.h>

/* ── Register offsets ──────────────────────────────────────────────── */

/* EMAR interface config */
#define EAMR_WORK_EN        0x0100
#define EAMR_RID            0x0108
#define EAMR_ROSD           0x010C
#define EAMR_WID            0x0110
#define EAMR_WOSD           0x0114

/* Interrupt registers */
#define INT_STATUS          0x0124
#define INT_EN              0x0128
#define INT_CLEAR           0x0130

/* Decompress output (direct address mode) */
#define DPRS_DATA_RTN_BADDR  0x2020
#define DPRS_DATA_RTN_LEN    0x2024

/* Decompress output (page mode) */
#define DPRS_DATA_INFO_BADDR 0x2028
#define DPRS_DATA_INFO_LEN   0x202C

#define DPRS_DATA_CRC32     0x2030

/* Decompress input */
#define DPRS_DATA_SRC_BADDR 0x2040
#define DPRS_DATA_SRC_LEN   0x2044

/* Status / result */
#define BUF_INFO            0x2080
#define DPRS_RTN_INFO       0x2084
#define DPRS_RTN_LEN        0x2088
#define BUF_INFO_CLR        0x2090
#define RLT_INFO_CLR        0x2094

/* CRC */
#define CRC_CHECK_EN        0x4000

/* Register region size */
#define GZIP_REG_SIZE       0x8000

/* ── Device state ──────────────────────────────────────────────────── */

#define TYPE_HISI_GZIP "hisi-gzip"
OBJECT_DECLARE_SIMPLE_TYPE(HisiGzipState, HISI_GZIP)

struct HisiGzipState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* EMAR config */
    uint32_t work_en;
    uint32_t eamr_rid;
    uint32_t eamr_rosd;
    uint32_t eamr_wid;
    uint32_t eamr_wosd;

    /* Interrupt */
    uint32_t int_en;
    uint32_t int_status;

    /* Decompress parameters */
    uint32_t dst_baddr;       /* output base address (direct mode) */
    uint32_t dst_len;         /* output max length */
    uint32_t info_baddr;      /* page mode base addr */
    uint32_t info_len;        /* page mode length */
    uint32_t src_baddr;       /* input base address */
    uint32_t crc32_val;
    uint32_t crc_en;

    /* Result */
    uint32_t buf_info;
    uint32_t rtn_info;        /* bit 31 = avail, bits [7:0] = err */
    uint32_t rtn_len;         /* decompressed length */
};

/* ── Decompress engine ─────────────────────────────────────────────── */

static void hisi_gzip_do_decompress(HisiGzipState *s, uint32_t src_len_reg)
{
    uint32_t src_len = src_len_reg & 0x0FFFFFFF;
    bool is_direct = (src_len_reg >> 31) & 1;  /* bit 31: 1=direct, 0=page */
    uint32_t dst_addr, dst_max;

    if (is_direct) {
        dst_addr = s->dst_baddr;
        dst_max  = s->dst_len;
    } else {
        dst_addr = s->info_baddr;
        dst_max  = s->info_len;
    }

    if (src_len == 0 || dst_max == 0) {
        s->rtn_info = (1u << 31) | 1;  /* error */
        s->rtn_len = 0;
        s->int_status |= 1;  /* task interrupt */
        return;
    }

    /* Read compressed data from guest memory */
    uint8_t *src_buf = g_malloc(src_len);
    dma_memory_read(&address_space_memory, s->src_baddr, src_buf, src_len,
                    MEMTXATTRS_UNSPECIFIED);

    /* Allocate output buffer */
    uint8_t *dst_buf = g_malloc(dst_max);

    /* Decompress using zlib (gzip format) */
    z_stream strm = {0};
    strm.next_in = src_buf;
    strm.avail_in = src_len;
    strm.next_out = dst_buf;
    strm.avail_out = dst_max;

    /* windowBits=31 means gzip auto-detect (16 + MAX_WBITS) */
    int ret = inflateInit2(&strm, 31);
    if (ret != Z_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-gzip: inflateInit2 failed: %d\n", ret);
        s->rtn_info = (1u << 31) | 1;
        s->rtn_len = 0;
        s->int_status |= 1;
        g_free(src_buf);
        g_free(dst_buf);
        return;
    }

    ret = inflate(&strm, Z_FINISH);
    uint32_t out_len = strm.total_out;
    inflateEnd(&strm);

    if (ret != Z_STREAM_END && ret != Z_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "hisi-gzip: inflate failed: %d (produced %u bytes)\n",
                      ret, out_len);
        /* Still write whatever was produced — some firmwares tolerate
         * partial decompression */
        if (out_len == 0) {
            s->rtn_info = (1u << 31) | 1;
            s->rtn_len = 0;
            s->int_status |= 1;
            g_free(src_buf);
            g_free(dst_buf);
            return;
        }
    }

    /* Write decompressed data to guest memory */
    dma_memory_write(&address_space_memory, dst_addr, dst_buf, out_len,
                     MEMTXATTRS_UNSPECIFIED);

    s->rtn_len = out_len;
    s->rtn_info = (1u << 31);  /* avail=1, err=0 => success */
    s->int_status |= 1;        /* task interrupt */

    qemu_log("hisi-gzip: decompressed %u -> %u bytes "
             "(src=0x%08x dst=0x%08x)\n",
             src_len, out_len, s->src_baddr, dst_addr);

    g_free(src_buf);
    g_free(dst_buf);
}

/* ── Register access ───────────────────────────────────────────────── */

static uint64_t hisi_gzip_read(void *opaque, hwaddr offset, unsigned size)
{
    HisiGzipState *s = HISI_GZIP(opaque);

    switch (offset) {
    case EAMR_WORK_EN:      return s->work_en;
    case EAMR_RID:          return s->eamr_rid;
    case EAMR_ROSD:         return s->eamr_rosd;
    case EAMR_WID:          return s->eamr_wid;
    case EAMR_WOSD:         return s->eamr_wosd;

    case INT_STATUS:        return s->int_status;
    case INT_EN:            return s->int_en;

    case DPRS_DATA_RTN_BADDR: return s->dst_baddr;
    case DPRS_DATA_RTN_LEN:   return s->dst_len;
    case DPRS_DATA_INFO_BADDR: return s->info_baddr;
    case DPRS_DATA_INFO_LEN:   return s->info_len;
    case DPRS_DATA_CRC32:     return s->crc32_val;
    case DPRS_DATA_SRC_BADDR: return s->src_baddr;
    case DPRS_DATA_SRC_LEN:   return 0;

    case BUF_INFO:          return s->buf_info;
    case DPRS_RTN_INFO:     return s->rtn_info;
    case DPRS_RTN_LEN:      return s->rtn_len;

    case CRC_CHECK_EN:      return s->crc_en;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "hisi-gzip: read from unknown reg 0x%04x\n",
                      (unsigned)offset);
        return 0;
    }
}

static void hisi_gzip_write(void *opaque, hwaddr offset,
                             uint64_t value, unsigned size)
{
    HisiGzipState *s = HISI_GZIP(opaque);

    switch (offset) {
    case EAMR_WORK_EN:      s->work_en = value; break;
    case EAMR_RID:          s->eamr_rid = value; break;
    case EAMR_ROSD:         s->eamr_rosd = value; break;
    case EAMR_WID:          s->eamr_wid = value; break;
    case EAMR_WOSD:         s->eamr_wosd = value; break;

    case INT_EN:            s->int_en = value; break;
    case INT_CLEAR:
        if (value & 1) s->int_status &= ~1;  /* task_intrpt_clr */
        if (value & 2) s->int_status &= ~2;  /* block_intrpt_clr */
        break;

    case DPRS_DATA_RTN_BADDR: s->dst_baddr = value; break;
    case DPRS_DATA_RTN_LEN:   s->dst_len = value; break;
    case DPRS_DATA_INFO_BADDR: s->info_baddr = value; break;
    case DPRS_DATA_INFO_LEN:   s->info_len = value; break;
    case DPRS_DATA_CRC32:     s->crc32_val = value; break;
    case CRC_CHECK_EN:        s->crc_en = value; break;

    case DPRS_DATA_SRC_BADDR:
        s->src_baddr = value;
        break;

    case DPRS_DATA_SRC_LEN:
        /* Writing src_len triggers the decompress operation */
        hisi_gzip_do_decompress(s, value);
        break;

    case BUF_INFO_CLR:
        s->buf_info = 0;
        break;

    case RLT_INFO_CLR:
        s->rtn_info = 0;
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "hisi-gzip: write 0x%08x to unknown reg 0x%04x\n",
                      (unsigned)value, (unsigned)offset);
        break;
    }
}

static const MemoryRegionOps hisi_gzip_ops = {
    .read  = hisi_gzip_read,
    .write = hisi_gzip_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ── Device lifecycle ──────────────────────────────────────────────── */

static void hisi_gzip_init(Object *obj)
{
    HisiGzipState *s = HISI_GZIP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &hisi_gzip_ops, s,
                          "hisi-gzip", GZIP_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const TypeInfo hisi_gzip_info = {
    .name          = TYPE_HISI_GZIP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HisiGzipState),
    .instance_init = hisi_gzip_init,
};

static void hisi_gzip_register(void)
{
    type_register_static(&hisi_gzip_info);
}

type_init(hisi_gzip_register)
