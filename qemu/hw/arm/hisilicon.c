/*
 * HiSilicon IP camera SoC emulation.
 *
 * Table-driven: each SoC variant is a HisiSoCConfig struct.
 * One shared init function handles VIC/GIC, peripherals, boot.
 *
 * Currently supported:
 *   hi3516cv300  (V3, ARM926EJ-S + PL190 VIC)
 *   hi3516ev300  (V4, Cortex-A7 + GICv2)
 *   gk7605v100   (Goke rebrand of V4)
 *
 * Copyright (c) 2020-2021, 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "cpu.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/arm/boot.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/char/pl011.h"
#include "hw/intc/arm_gic_common.h"
#include "hw/intc/arm_gic.h"
#include "hw/arm/hisilicon.h"
#include "hw/arm/machines-qom.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "hw/sd/sdhci.h"
#include "hw/sd/sd.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "net/net.h"
#include "hw/i2c/i2c.h"
#include "hw/clock.h"
#include "hw/qdev-clock.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/gtimer.h"

/* ── SoC configuration tables ──────────────────────────────────────── */

/*
 * Hi3516CV100 (V1): ARM926EJ-S + PL190-compatible VIC, oldest generation.
 * VIC at 0x10140000 (not 0x100D0000 like V2).  HISFC350 flash controller
 * (not HiFMC V100) — set fmc_ctrl_base=0 to skip FMC creation.
 * 12 GPIO banks (GPIO0-11).  Single himciv100 SD controller.
 * UART0 and UART1 share IRQ 5.  No SPI (PL022) controllers.
 * Kernel 3.0.8, MACHINE_START (no DT), computes bus clock from BPLL PLL.
 */
static const HisiSoCConfig hi3516cv100_soc = {
    .name               = "hi3516cv100",
    .desc               = "HiSilicon Hi3516CV100 (ARM926EJ-S)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm926"),
    .soc_id             = HISI_SOC_ID_CV100,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = false,
    .vic_base           = 0x10140000,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,

    .num_uarts          = 3,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000 },
    .uart_irqs          = { 5, 5, 25 },    /* UART0 and UART1 share IRQ 5 */

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 50000000,     /* 50 MHz (AXI 100MHz / prescale 2) */

    /* No PL022 SPI controllers on V1 */
    .num_spis           = 0,

    /* HISFC350 — not HiFMC V100; skip FMC creation */
    .fmc_ctrl_base      = 0,
    .fmc_mem_base       = 0,

    .gpio_base          = 0x20140000,
    .gpio_count         = 12,
    .gpio_stride        = 0x10000,
    .gpio_irq           = 31,           /* shared for all ports (VIC) */

    .dma_base           = 0x100D0000,
    .dma_irq            = 14,

    .femac_base         = 0x10090000,
    .femac_irq          = 12,

    .num_himci          = 1,
    .himci_bases        = { 0x10020000 },
    .himci_irqs         = { 18 },

    /* No I2C buses in V1 platform headers */
    .num_i2c            = 0,

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    /*
     * BPLL register defaults for 100 MHz AXI bus clock.
     * Kernel computes: busclk = 24M * fbdiv / (2 * refdiv * pstdiv1 * pstdiv2)
     * With refdiv=3, fbdiv=25, pstdiv1=1, pstdiv2=1: busclk = 100 MHz.
     * Timer clock = busclk / prescale(2) = 50 MHz.
     */
    .num_crg_defaults   = 2,
    .crg_defaults       = {
        { 0x10, (1 << 24) | (1 << 27) },   /* CRG4: pstdiv1=1, pstdiv2=1 */
        { 0x14, (3 << 12) | 25 },           /* CRG5: refdiv=3, fbdiv=25 */
    },

    /* NANDC + SFC350 stubs — U-Boot probes both before detecting flash type */
    .num_regbanks       = 2,
    .regbanks           = {
        { "hisi-nandc",  0x10000000, 0x10000 },
        { "hisi-sfc350", 0x10010000, 0x10000 },
    },
};

/*
 * Hi3516CV200 (V2): ARM926EJ-S + PL190 VIC, 0x20xxxxxx peripheral space.
 * Also known as Hi3518EV200.  Uses hieth-sf in vendor kernel but FEMAC
 * in OpenIPC's 4.9+ kernel.  FMC memory window at 0x58000000 (not 0x14000000).
 */
static const HisiSoCConfig hi3516cv200_soc = {
    .name               = "hi3516cv200",
    .desc               = "HiSilicon Hi3516CV200 (ARM926EJ-S)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm926"),
    .soc_id             = HISI_SOC_ID_CV200,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = false,
    .vic_base           = 0x100D0000,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,

    .num_uarts          = 3,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000 },
    .uart_irqs          = { 5, 30, 25 },

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 3000000,      /* 3 MHz (APB bus / 4) */

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 6, 7 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,

    .gpio_base          = 0x20140000,
    .gpio_count         = 9,
    .gpio_stride        = 0x10000,
    .gpio_irq           = 31,           /* shared for all ports (VIC) */

    .dma_base           = 0x10060000,
    .dma_irq            = 14,

    .femac_base         = 0x10090000,
    .femac_irq          = 12,

    .num_himci          = 2,
    .himci_bases        = { 0x10020000, 0x10030000 },
    .himci_irqs         = { 18, 8 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x200D0000, 0x20240000, 0x20250000 },

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    /*
     * CRG register defaults — mimic what U-Boot sets before booting Linux.
     * The 4.9 kernel uses DT-based clocks; gate bits must be pre-enabled
     * or the PL011 UART driver can't get its clock and the console is dead.
     */
    .num_crg_defaults   = 4,
    .crg_defaults       = {
        /* UART0/1/2 + SPI0/1 clk enable, UART mux=24MHz */
        { 0xe4, (1 << 13) | (1 << 14) | (1 << 15) |
                (1 << 16) | (1 << 17) | (1 << 19) },
        { 0xc0, (1 << 1) },            /* FMC clk enable */
        { 0xec, (1 << 1) },            /* ETH clk enable */
        { 0xc4, (1 << 1) | (1 << 9) }, /* MMC0 + MMC1 clk enable */
    },
};

static const HisiSoCConfig hi3516cv300_soc = {
    .name               = "hi3516cv300",
    .desc               = "HiSilicon Hi3516CV300 (ARM926EJ-S)",
    .cpu_type           = ARM_CPU_TYPE_NAME("arm926"),
    .soc_id             = HISI_SOC_ID_CV300,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = false,
    .vic_base           = 0x10040000,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12100000, 0x12101000, 0x12102000 },
    .uart_irqs          = { 5, 30, 25 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 24000000,     /* 24 MHz */

    .num_spis           = 2,
    .spi_bases          = { 0x12120000, 0x12121000 },
    .spi_irqs           = { 6, 7 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x12140000,
    .gpio_count         = 9,
    .gpio_stride        = 0x1000,
    .gpio_irq           = 31,           /* shared for all ports (VIC) */

    .dma_base           = 0x10030000,
    .dma_irq            = 14,

    .femac_base         = 0x10050000,
    .femac_irq          = 12,

    .num_i2c            = 2,
    .i2c_bases          = { 0x12110000, 0x12112000 },

    .num_himci          = 3,
    .himci_bases        = { 0x100c0000, 0x100d0000, 0x100e0000 },
    .himci_irqs         = { 18, 27, 27 },

    .wdt_base           = 0x12080000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,
};

static const HisiSoCConfig hi3516ev300_soc = {
    .name               = "hi3516ev300",
    .desc               = "HiSilicon Hi3516EV300 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_EV300,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 },
    .uart_irqs          = { 7, 8, 9 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 5, 6 },

    .num_spis           = 2,
    .spi_bases          = { 0x12070000, 0x12071000 },
    .spi_irqs           = { 14, 15 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120b0000,
    .gpio_count         = 10,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,           /* per-port: SPI 16..25 (GIC) */

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .mipi_rx_base       = 0x11240000,
    .mipi_rx_irq        = 45,

    .rtc_base           = 0x120e0000,
    .rtc_irq            = 0,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 },

    .vedu_base          = 0x11410000,
    .jpge_base          = 0x11420000,
    .vedu_irq           = 47,
    .jpge_irq           = 48,

    .wdt_base           = 0x12030000,
    .wdt_irq            = 2,
    .wdt_freq           = 3000000,
};

/*
 * Hi3516EV200: economy variant — 8 GPIO groups, 64MB DDR2.
 * Same peripheral addresses as EV300.
 */
static const HisiSoCConfig hi3516ev200_soc = {
    .name               = "hi3516ev200",
    .desc               = "HiSilicon Hi3516EV200 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_EV200,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 },
    .uart_irqs          = { 7, 8, 9 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 5, 6 },

    .num_spis           = 2,
    .spi_bases          = { 0x12070000, 0x12071000 },
    .spi_irqs           = { 14, 15 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120b0000,
    .gpio_count         = 8,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .mipi_rx_base       = 0x11240000,
    .mipi_rx_irq        = 45,

    .rtc_base           = 0x120e0000,
    .rtc_irq            = 0,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 },

    .vedu_base          = 0x11410000,
    .jpge_base          = 0x11420000,
    .vedu_irq           = 47,
    .jpge_irq           = 48,

    .wdt_base           = 0x12030000,
    .wdt_irq            = 2,
    .wdt_freq           = 3000000,
};

/*
 * Hi3518EV300: consumer variant — 8 GPIO groups, no integrated FE PHY.
 * FEMAC controller exists but there's no on-chip PHY; boards may have
 * an external PHY or no Ethernet at all.  We still instantiate FEMAC
 * so the driver probe succeeds (it just won't link up without a real PHY).
 */
static const HisiSoCConfig hi3518ev300_soc = {
    .name               = "hi3518ev300",
    .desc               = "HiSilicon Hi3518EV300 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_18EV300,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 },
    .uart_irqs          = { 7, 8, 9 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 5, 6 },

    .num_spis           = 2,
    .spi_bases          = { 0x12070000, 0x12071000 },
    .spi_irqs           = { 14, 15 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120b0000,
    .gpio_count         = 8,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .mipi_rx_base       = 0x11240000,
    .mipi_rx_irq        = 45,

    .rtc_base           = 0x120e0000,
    .rtc_irq            = 0,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 },

    .vedu_base          = 0x11410000,
    .jpge_base          = 0x11420000,
    .vedu_irq           = 47,
    .jpge_irq           = 48,

    .wdt_base           = 0x12030000,
    .wdt_irq            = 2,
    .wdt_freq           = 3000000,
};

/*
 * Hi3516DV200: professional variant — 10 GPIO groups, up to 512MB DDR.
 * Same peripheral layout as EV300, higher video resolution.
 */
static const HisiSoCConfig hi3516dv200_soc = {
    .name               = "hi3516dv200",
    .desc               = "HiSilicon Hi3516DV200 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_DV200,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x40000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 },
    .uart_irqs          = { 7, 8, 9 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 5, 6 },

    .num_spis           = 2,
    .spi_bases          = { 0x12070000, 0x12071000 },
    .spi_irqs           = { 14, 15 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120b0000,
    .gpio_count         = 10,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .mipi_rx_base       = 0x11240000,
    .mipi_rx_irq        = 45,

    .rtc_base           = 0x120e0000,
    .rtc_irq            = 0,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 },

    .vedu_base          = 0x11410000,
    .jpge_base          = 0x11420000,
    .vedu_irq           = 47,
    .jpge_irq           = 48,

    .wdt_base           = 0x12030000,
    .wdt_irq            = 2,
    .wdt_freq           = 3000000,
};

/*
 * Goke variants — die-identical V4 silicon, only SoC ID differs.
 * Hardware addresses, IRQs, GPIO counts all match the HiSilicon original.
 */

/* Common V4 peripheral block — shared by all V4 & Goke configs */
#define HISI_V4_COMMON_PERIPH                               \
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),   \
    .ram_size_default   = 64 * MiB,                         \
    .ram_base           = 0x40000000,                       \
    .sram_base          = 0x04010000,                       \
    .sram_size          = 64 * KiB,                         \
    .use_gic            = true,                             \
    .gic_dist_base      = 0x10301000,                       \
    .gic_cpu_base       = 0x10302000,                       \
    .gic_num_spi        = 128,                              \
    .sysctl_base        = 0x12020000,                       \
    .crg_base           = 0x12010000,                       \
    .num_uarts          = 3,                                \
    .uart_bases         = { 0x12040000, 0x12041000, 0x12042000 }, \
    .uart_irqs          = { 7, 8, 9 },                     \
    .num_timers         = 2,                                \
    .timer_bases        = { 0x12000000, 0x12001000 },       \
    .timer_irqs         = { 5, 6 },                        \
    .num_spis           = 2,                                \
    .spi_bases          = { 0x12070000, 0x12071000 },       \
    .spi_irqs           = { 14, 15 },                      \
    .fmc_ctrl_base      = 0x10000000,                       \
    .fmc_mem_base       = 0x14000000,                       \
    .gpio_base          = 0x120b0000,                       \
    .gpio_stride        = 0x1000,                           \
    .gpio_irq_start     = 16,                              \
    .femac_base         = 0x10040000,                       \
    .femac_irq          = 33,                              \
    .mipi_rx_base       = 0x11240000,                      \
    .mipi_rx_irq        = 45,                              \
    .rtc_base           = 0x120e0000,                      \
    .rtc_irq            = 0,                               \
    .num_sdhci          = 2,                                \
    .sdhci_bases        = { 0x10010000, 0x10020000 },       \
    .sdhci_irqs         = { 30, 31 },                       \
    .num_i2c            = 3,                                \
    .i2c_bases          = { 0x12060000, 0x12061000, 0x12062000 }, \
    .vedu_base          = 0x11410000,                       \
    .jpge_base          = 0x11420000,                       \
    .vedu_irq           = 47,                              \
    .jpge_irq           = 48,                              \
    .wdt_base           = 0x12030000,                       \
    .wdt_irq            = 2,                                \
    .wdt_freq           = 3000000,                          \
    .num_regbanks       = 14,                               \
    .regbanks           = {                                 \
        { "hisi-misc",       0x12028000, 0x8000  },         \
        { "hisi-ddr",        0x120d0000, 0x10000 },         \
        { "hisi-iocfg-vio",  0x112c0000, 0x10000 },         \
        { "hisi-iocfg-core", 0x120c0000, 0x10000 },         \
        { "hisi-iocfg-ahb",  0x100c0000, 0x10000 },         \
        { "hisi-pwm",        0x12080000, 0x10000 },         \
        { "hisi-usb3",       0x10030000, 0x10000 },         \
        { "hisi-aiao",       0x100e0000, 0x10000 },         \
        { "hisi-acodec",     0x100f0000, 0x10000 },         \
        { "hisi-vi-cap",     0x11000000, 0x200000 },        \
        { "hisi-vi-proc",    0x11200000, 0x40000 },         \
        { "hisi-vgs",        0x11300000, 0x10000 },         \
        { "hisi-ive",        0x11320000, 0x10000 },         \
        { "hisi-vpss",       0x11400000, 0x10000 },         \
    }

static const HisiSoCConfig gk7205v200_soc = {
    .name               = "gk7205v200",
    .desc               = "Goke GK7205V200 (Cortex-A7, ~Hi3516EV200)",
    .soc_id             = GOKE_SOC_ID_7205V200,
    .gpio_count         = 8,
    HISI_V4_COMMON_PERIPH,
};

static const HisiSoCConfig gk7205v300_soc = {
    .name               = "gk7205v300",
    .desc               = "Goke GK7205V300 (Cortex-A7, ~Hi3516EV300)",
    .soc_id             = GOKE_SOC_ID_7205V300,
    .gpio_count         = 10,
    HISI_V4_COMMON_PERIPH,
};

static const HisiSoCConfig gk7202v300_soc = {
    .name               = "gk7202v300",
    .desc               = "Goke GK7202V300 (Cortex-A7, ~Hi3518EV300)",
    .soc_id             = GOKE_SOC_ID_7202V300,
    .gpio_count         = 8,
    HISI_V4_COMMON_PERIPH,
};

static const HisiSoCConfig gk7605v100_soc = {
    .name               = "gk7605v100",
    .desc               = "Goke GK7605V100 (Cortex-A7, ~Hi3516DV200)",
    .soc_id             = GOKE_SOC_ID_7605V100,
    .gpio_count         = 10,
    HISI_V4_COMMON_PERIPH,
};

/* ── Machine state with sensor property ────────────────────────────── */

/* Extra state appended to MachineState for the sensor property */
typedef struct {
    MachineState parent_obj;
    char *sensor;
} HisiMachineState;

/* ── Shared machine init ───────────────────────────────────────────── */

/* PPI numbers — same for all GICv2 HiSilicon SoCs */
#define HISI_PPI_HYPTIMER   10
#define HISI_PPI_VIRTTIMER  11
#define HISI_PPI_SECTIMER   13
#define HISI_PPI_PHYSTIMER  14

static struct arm_boot_info hisilicon_binfo;

static void hisilicon_common_init(MachineState *machine,
                                  const HisiSoCConfig *c)
{
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    ARMCPU *cpu;
    qemu_irq pic[256];
    int num_pic = 0;
    int n;

    /*
     * Trap page at address 0 — U-Boot driver model may call through NULL
     * function pointers for absent hardware.  Place "mov r0, #0; bx lr"
     * so those calls harmlessly return 0 instead of executing garbage.
     */
    {
        MemoryRegion *trap = g_new(MemoryRegion, 1);
        memory_region_init_rom(trap, NULL, "hisilicon.trapnull",
                               0x1000, &error_fatal);
        memory_region_add_subregion(sysmem, 0, trap);
        uint32_t insn[2] = { cpu_to_le32(0xe3a00000),   /* mov r0, #0 */
                             cpu_to_le32(0xe12fff1e) };  /* bx lr      */
        address_space_write_rom(&address_space_memory, 0,
                                MEMTXATTRS_UNSPECIFIED, insn, 8);
    }

    /* SRAM */
    {
        MemoryRegion *sram = g_new(MemoryRegion, 1);
        memory_region_init_ram(sram, NULL, "hisilicon.sram",
                               c->sram_size, &error_fatal);
        memory_region_add_subregion(sysmem, c->sram_base, sram);
    }

    /* FMC */
    if (c->fmc_ctrl_base) {
        DeviceState *fmc = qdev_new("hisi-fmc");
        SysBusDevice *fmcbus = SYS_BUS_DEVICE(fmc);
        sysbus_realize_and_unref(fmcbus, &error_fatal);
        sysbus_mmio_map(fmcbus, 0, c->fmc_ctrl_base);
        sysbus_mmio_map(fmcbus, 1, c->fmc_mem_base);
    }

    /* RAM */
    memory_region_add_subregion(sysmem, c->ram_base, machine->ram);

    /* CPU */
    cpuobj = object_new(machine->cpu_type);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    /* Interrupt controller */
    if (c->use_gic) {
        int num_irq = c->gic_num_spi + GIC_INTERNAL;
        DeviceState *gicdev = qdev_new(gic_class_name());
        qdev_prop_set_uint32(gicdev, "revision", 2);
        qdev_prop_set_uint32(gicdev, "num-cpu", 1);
        qdev_prop_set_uint32(gicdev, "num-irq", num_irq);
        qdev_prop_set_bit(gicdev, "has-security-extensions", false);
        SysBusDevice *gicbus = SYS_BUS_DEVICE(gicdev);
        sysbus_realize_and_unref(gicbus, &error_fatal);
        sysbus_mmio_map(gicbus, 0, c->gic_dist_base);
        sysbus_mmio_map(gicbus, 1, c->gic_cpu_base);

        /* GIC outputs → CPU */
        sysbus_connect_irq(gicbus, 0,
                           qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
        sysbus_connect_irq(gicbus, 1,
                           qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));
        sysbus_connect_irq(gicbus, 2,
                           qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_VIRQ));
        sysbus_connect_irq(gicbus, 3,
                           qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_VFIQ));

        /* CPU timer PPIs → GIC */
        {
            DeviceState *cpudev = DEVICE(cpu);
            int ppibase = c->gic_num_spi + GIC_NR_SGIS;
            const int timer_ppi[] = {
                [GTIMER_PHYS] = HISI_PPI_PHYSTIMER,
                [GTIMER_VIRT] = HISI_PPI_VIRTTIMER,
                [GTIMER_HYP]  = HISI_PPI_HYPTIMER,
                [GTIMER_SEC]  = HISI_PPI_SECTIMER,
            };
            for (int i = 0; i < ARRAY_SIZE(timer_ppi); i++) {
                qdev_connect_gpio_out(cpudev, i,
                    qdev_get_gpio_in(gicdev, ppibase + timer_ppi[i]));
            }
            /* PMU */
            qdev_connect_gpio_out_named(cpudev, "pmu-interrupt", 0,
                qdev_get_gpio_in(gicdev, ppibase + 7));
        }

        /* SPI IRQ array */
        num_pic = c->gic_num_spi;
        for (n = 0; n < num_pic; n++) {
            pic[n] = qdev_get_gpio_in(gicdev, n);
        }
    } else {
        /* PL190 VIC */
        DeviceState *vic = sysbus_create_varargs("pl190", c->vic_base,
                qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ),
                qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ),
                NULL);
        num_pic = 32;
        for (n = 0; n < num_pic; n++) {
            pic[n] = qdev_get_gpio_in(vic, n);
        }
    }

    /* SysCtrl */
    {
        DeviceState *sysctl = qdev_new("hisi-sysctl");
        qdev_prop_set_uint32(sysctl, "soc-id", c->soc_id);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sysctl), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(sysctl), 0, c->sysctl_base);
    }

    /* CRG */
    {
        DeviceState *crg = qdev_new("hisi-crg");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(crg), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(crg), 0, c->crg_base);

        /* Pre-enable clocks (mimics U-Boot init before kernel boot) */
        for (n = 0; n < c->num_crg_defaults; n++) {
            address_space_stl(&address_space_memory,
                              c->crg_base + c->crg_defaults[n].offset,
                              c->crg_defaults[n].value,
                              MEMTXATTRS_UNSPECIFIED, NULL);
        }
    }

    /* UARTs */
    for (n = 0; n < c->num_uarts; n++) {
        pl011_create(c->uart_bases[n], pic[c->uart_irqs[n]], serial_hd(n));
    }

    /* Timers (SP804) */
    for (n = 0; n < c->num_timers; n++) {
        if (c->timer_freq) {
            DeviceState *t = qdev_new("sp804");
            qdev_prop_set_uint32(t, "freq0", c->timer_freq);
            qdev_prop_set_uint32(t, "freq1", c->timer_freq);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(t), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(t), 0, c->timer_bases[n]);
            sysbus_connect_irq(SYS_BUS_DEVICE(t), 0, pic[c->timer_irqs[n]]);
        } else {
            sysbus_create_simple("sp804", c->timer_bases[n],
                                 pic[c->timer_irqs[n]]);
        }
    }

    /* SPI (PL022) */
    for (n = 0; n < c->num_spis; n++) {
        sysbus_create_simple("pl022", c->spi_bases[n], pic[c->spi_irqs[n]]);
    }

    /* DMA (PL080) */
    if (c->dma_base) {
        DeviceState *dma = qdev_new("pl080");
        object_property_set_link(OBJECT(dma), "downstream", OBJECT(sysmem),
                                 &error_fatal);
        SysBusDevice *busdev = SYS_BUS_DEVICE(dma);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->dma_base);
        sysbus_connect_irq(busdev, 0, pic[c->dma_irq]);
    }

    /* GPIOs (PL061) */
    for (n = 0; n < c->gpio_count; n++) {
        qemu_irq irq;
        if (c->use_gic) {
            irq = pic[c->gpio_irq_start + n];
        } else {
            irq = pic[c->gpio_irq];
        }
        sysbus_create_simple("pl061",
                             c->gpio_base + n * c->gpio_stride, irq);
    }

    /* FEMAC */
    if (c->femac_base) {
        DeviceState *femac = qdev_new("hisi-femac");
        qemu_configure_nic_device(femac, true, NULL);
        SysBusDevice *busdev = SYS_BUS_DEVICE(femac);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->femac_base);
        sysbus_connect_irq(busdev, 0, pic[c->femac_irq]);
    }

    /* SD/MMC — himciv200 */
    for (n = 0; n < c->num_himci; n++) {
        DeviceState *mmc = qdev_new("hisi-himci");
        SysBusDevice *busdev = SYS_BUS_DEVICE(mmc);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->himci_bases[n]);
        if (c->himci_irqs[n]) {
            sysbus_connect_irq(busdev, 0, pic[c->himci_irqs[n]]);
        }

        DriveInfo *di = drive_get(IF_SD, 0, n);
        if (di) {
            BusState *bus = qdev_get_child_bus(DEVICE(mmc), "sd-bus");
            DeviceState *card = qdev_new(TYPE_SD_CARD);
            qdev_prop_set_drive_err(card, "drive",
                                    blk_by_legacy_dinfo(di), &error_fatal);
            qdev_realize_and_unref(card, bus, &error_fatal);
        }
    }

    /* SD/MMC — SDHCI */
    for (n = 0; n < c->num_sdhci; n++) {
        DeviceState *sdhci = qdev_new(TYPE_SYSBUS_SDHCI);
        qdev_prop_set_uint8(sdhci, "sd-spec-version", 3);
        qdev_prop_set_uint8(sdhci, "uhs", UHS_I);
        qdev_prop_set_uint64(sdhci, "capareg", 0x017834b4);
        SysBusDevice *busdev = SYS_BUS_DEVICE(sdhci);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->sdhci_bases[n]);
        if (c->sdhci_irqs[n]) {
            sysbus_connect_irq(busdev, 0, pic[c->sdhci_irqs[n]]);
        }

        DriveInfo *di = drive_get(IF_SD, 0, n);
        if (di) {
            BusState *bus = qdev_get_child_bus(sdhci, "sd-bus");
            DeviceState *card = qdev_new(TYPE_SD_CARD);
            qdev_prop_set_drive_err(card, "drive",
                                    blk_by_legacy_dinfo(di), &error_fatal);
            qdev_realize_and_unref(card, bus, &error_fatal);
        }
    }

    /* I2C (HiBVT) */
    DeviceState *i2c_devs[HISI_MAX_I2C] = { NULL };
    for (n = 0; n < c->num_i2c; n++) {
        i2c_devs[n] = qdev_new("hisi-i2c");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c_devs[n]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(i2c_devs[n]), 0, c->i2c_bases[n]);
    }

    /* MIPI RX */
    if (c->mipi_rx_base) {
        DeviceState *mipi = qdev_new("hisi-mipi-rx");
        SysBusDevice *busdev = SYS_BUS_DEVICE(mipi);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->mipi_rx_base);
        sysbus_connect_irq(busdev, 0, pic[c->mipi_rx_irq]);
    }

    /* RTC (SPI-bridge) */
    if (c->rtc_base) {
        DeviceState *rtc = qdev_new("hisi-rtc");
        SysBusDevice *busdev = SYS_BUS_DEVICE(rtc);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->rtc_base);
        sysbus_connect_irq(busdev, 0, pic[c->rtc_irq]);
    }

    /* VEDU + JPGE */
    if (c->vedu_base) {
        DeviceState *vedu = qdev_new("hisi-vedu");
        SysBusDevice *busdev = SYS_BUS_DEVICE(vedu);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->vedu_base);
        sysbus_mmio_map(busdev, 1, c->jpge_base);
        sysbus_connect_irq(busdev, 0, pic[c->vedu_irq]);
        sysbus_connect_irq(busdev, 1, pic[c->jpge_irq]);
    }

    /* Watchdog (SP805-compatible, reuse cmsdk-apb-watchdog) */
    if (c->wdt_base) {
        Clock *wdt_clk = clock_new(OBJECT(machine), "wdt-clk");
        clock_set_hz(wdt_clk, c->wdt_freq);
        DeviceState *wdt = qdev_new("cmsdk-apb-watchdog");
        qdev_connect_clock_in(wdt, "WDOGCLK", wdt_clk);
        SysBusDevice *busdev = SYS_BUS_DEVICE(wdt);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->wdt_base);
        if (c->wdt_irq >= 0) {
            sysbus_connect_irq(busdev, 0, pic[c->wdt_irq]);
        }
    }

    /* Generic register banks (pin mux, DDR PHY, PWM, etc.) */
    for (n = 0; n < c->num_regbanks; n++) {
        if (c->regbanks[n].base) {
            DeviceState *rb = qdev_new("hisi-regbank");
            qdev_prop_set_uint32(rb, "size", c->regbanks[n].size);
            qdev_prop_set_string(rb, "name", c->regbanks[n].name);

            /*
             * NANDC: offset 0x20 bit 0 = OP_DONE, must read 1.
             * SFC350: CMD_CONFIG (0x300) bit 0 = START, auto-clears
             *         when command completes — use autoclear feature.
             */
            if (!strcmp(c->regbanks[n].name, "hisi-sfc350")) {
                qdev_prop_set_uint32(rb, "autoclear-offset", 0x300);
                qdev_prop_set_uint32(rb, "autoclear-mask", 0x01);
            }

            sysbus_realize_and_unref(SYS_BUS_DEVICE(rb), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(rb), 0, c->regbanks[n].base);

            if (!strcmp(c->regbanks[n].name, "hisi-nandc")) {
                address_space_stl(&address_space_memory,
                                  c->regbanks[n].base + 0x20, 0x01,
                                  MEMTXATTRS_UNSPECIFIED, NULL);
            }
        }
    }

    /* Sensor auto-attach via -machine sensor=imx335 */
    {
        HisiMachineState *hms = (HisiMachineState *)machine;
        if (hms->sensor && c->num_i2c > 0 && i2c_devs[0]) {
            if (!strcmp(hms->sensor, "imx335")) {
                BusState *i2c_bus = qdev_get_child_bus(i2c_devs[0], "i2c");
                DeviceState *sensor = qdev_new("hisi-imx335");
                qdev_prop_set_uint8(sensor, "address", 0x1A);
                qdev_realize_and_unref(sensor, i2c_bus, &error_fatal);
            } else {
                error_report("Unknown sensor '%s' (supported: imx335)",
                             hms->sensor);
                exit(1);
            }
        }
    }

    /* Boot */
    hisilicon_binfo.ram_size = machine->ram_size;
    hisilicon_binfo.loader_start = c->ram_base;
    arm_load_kernel(cpu, machine, &hisilicon_binfo);
}

/* ── Sensor property accessors ─────────────────────────────────────── */

static char *hisi_machine_get_sensor(Object *obj, Error **errp)
{
    HisiMachineState *s = (HisiMachineState *)obj;
    return g_strdup(s->sensor);
}

static void hisi_machine_set_sensor(Object *obj, const char *value,
                                     Error **errp)
{
    HisiMachineState *s = (HisiMachineState *)obj;
    g_free(s->sensor);
    s->sensor = g_strdup(value);
}

/* ── Per-machine wrappers ──────────────────────────────────────────── */

/*
 * All HiSilicon machines use HisiMachineState (for the sensor property)
 * and the ARM interface array (for arm/aarch64 dual-build).
 */
#define DEFINE_HISI_MACHINE(namestr, tag, config)                    \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        hisilicon_common_init(machine, &config);                     \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        ObjectClass *oc = OBJECT_CLASS(mc);                          \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = config.cpu_type;                      \
        mc->default_ram_size = config.ram_size_default;              \
        mc->default_ram_id = "hisilicon.ram";                        \
        mc->block_default_type = IF_MTD;                             \
        mc->ignore_memory_transaction_failures = true;               \
        object_class_property_add_str(oc, "sensor",                  \
            hisi_machine_get_sensor, hisi_machine_set_sensor);       \
        object_class_property_set_description(oc, "sensor",          \
            "Image sensor to attach (e.g. imx335)");                 \
    }                                                                \
    DEFINE_MACHINE_EXTENDED(namestr, MACHINE, HisiMachineState,      \
                            tag##_class_init, false,                  \
                            arm_machine_interfaces)

DEFINE_HISI_MACHINE("hi3516cv100", hi3516cv100, hi3516cv100_soc)
DEFINE_HISI_MACHINE("hi3516cv200", hi3516cv200, hi3516cv200_soc)
DEFINE_HISI_MACHINE("hi3516cv300", hi3516cv300, hi3516cv300_soc)
DEFINE_HISI_MACHINE("hi3516ev300", hi3516ev300, hi3516ev300_soc)
DEFINE_HISI_MACHINE("hi3516ev200", hi3516ev200, hi3516ev200_soc)
DEFINE_HISI_MACHINE("hi3518ev300", hi3518ev300, hi3518ev300_soc)
DEFINE_HISI_MACHINE("hi3516dv200", hi3516dv200, hi3516dv200_soc)
DEFINE_HISI_MACHINE("gk7205v200", gk7205v200, gk7205v200_soc)
DEFINE_HISI_MACHINE("gk7205v300", gk7205v300, gk7205v300_soc)
DEFINE_HISI_MACHINE("gk7202v300", gk7202v300, gk7202v300_soc)
DEFINE_HISI_MACHINE("gk7605v100", gk7605v100, gk7605v100_soc)
