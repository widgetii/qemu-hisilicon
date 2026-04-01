/*
 * HiSilicon IP camera SoC emulation.
 *
 * Table-driven: each SoC variant is a HisiSoCConfig struct.
 * One shared init function handles VIC/GIC, peripherals, boot.
 *
 * 21 machine types across six HiSilicon IPC SoC generations (V1–V5)
 * plus Goke V4 rebrands and Goke next-gen NPU SoCs.
 * See roadmap table below for specs & timeline.
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
#include "hw/misc/hisi-fastboot.h"
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
#include <libfdt.h>
#include "hw/loader.h"
#include <zlib.h>  /* crc32() for uImage header fixup */


/* ── SoC configuration tables ──────────────────────────────────────── */

/*
 * HiSilicon IPC SoC product roadmap — release timeline & key specs
 * (source: vendor roadmap + OpenIPC firmware CI platform groupings)
 * ─────────────────────────────────────────────────────────────────────
 *
 *  Gen   SoC         Year    CPU                 Video            Tier
 *  ───── ─────────── ─────── ─────────────────── ──────────────── ─────
 *  V1    CV100       <2015   ARM926 @550MHz      H.264 1080P@30   2M
 *  V2    CV200       <2015   ARM926 @550MHz      H.264 960P@30    1M
 *  V2A   AV100       <2015   A7 @600MHz          H.265 5M@25      5M
 *        DV100*      <2015   A7 @600MHz          H.265 4M@25      4M
 *  V3    CV300       ~2017   ARM926 @800MHz      H.265 1080P@30   2M
 *        EV100*      ~2018   ARM9 @800MHz        H.265 1080P@20   1M
 *  V3A   3519V101    ~2017   A17@1.2G+A7@800M    H.265 4K@30+2M   4K
 *        AV200*      ~2017   A17@1.2G+A7@800M    H.265 5M@30      5M
 *  V3.5  CV500       2018    Dual A7 @900MHz      H.265/264 3M     3M
 *        AV300*      2019    Dual A7 @900MHz      H.265/264 4K     4K
 *        DV300*      2019    Dual A7 @900MHz      H.265/264 5M     5M
 *  V4    EV200       2018Q4  A7 @900MHz          H.265 3M@20      3M
 *        EV300       2018Q4  A7 @900MHz          H.265 5M@15      5M
 *        18EV300     2018Q4  A7 @900MHz          H.265 3M@20      3M
 *        DV200       2018Q4  A7 @900MHz          H.265 5M@20      5M
 *  V4g   GK7205V200  2021    (die-identical V4 — Goke rebrand)
 *  V4n   GK7205V500  2022    A7 @1GHz            H.265 5M@25      5M+NPU
 *  V5    CV608       ~2023   Dual A7 MP2         H.265 3M         3M
 *        CV610       ~2023   Dual A7 MP2         H.265 5M         5M
 *        CV613       ~2023   Dual A7 MP2         H.265 4K         4K
 *
 *  Entries marked * share a platform SDK with the line above but are
 *  not emulated here.  Hi3519A (4K Smart Vision) and Hi3559A (8K,
 *  2xA73+2xA53) also appear on the roadmap but are not emulated.
 *
 * Platform family groupings (same SDK, same address map):
 *   CV100 platform:  hi3516cv100, hi3518cv100, hi3518ev100
 *   CV200 platform:  hi3516cv200, hi3518ev200
 *   AV100 platform:  hi3516av100, hi3516dv100
 *   CV300 platform:  hi3516cv300, hi3516ev100
 *   3519V101 platf:  hi3519v101, hi3516av200
 *   CV500 platform:  hi3516cv500, hi3516av300, hi3516dv300
 *   EV200 platform:  hi3516ev200, hi3516ev300, hi3518ev300, hi3516dv200
 *   GK7205V200:      gk7205v200, gk7205v210, gk7205v300, gk7202v300, gk7605v100
 *   GK7205V500:      gk7205v500, gk7205v510, gk7205v530, gk7202v330 (NPU)
 *
 * Address map evolution:
 *   V1/V2/V2A: 0x20xxxxxx peripherals, RAM @ 0x80000000
 *   V3/V3A:    0x12xxxxxx peripherals, RAM @ 0x80000000
 *   V3.5:      0x12xxxxxx (variant), GIC @ 0x10301000, RAM @ 0x80000000
 *   V4:        0x12xxxxxx peripherals, RAM @ 0x40000000
 *   V5:        0x11xxxxxx peripherals, GIC @ 0x124xxxxx, RAM @ 0x40000000
 */

/*
 * Hi3516CV100 (V1): pre-2015, 2M mainstream.  ARM926EJ-S @550MHz.
 * Video: H.264, 1080P@30fps.  Memory: 64MB DDR2.
 * Platform family: hi3518cv100, hi3518ev100 (same SDK).
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

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 6, 7 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,
    .fmc_type           = "hisi-sfc350",

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

    /*
     * Register bank stubs — vendor .ko modules access video/peripheral
     * hardware during init.  Without mapped regions, reads return 0
     * from QEMU's "unimplemented" handler, causing poll loops to hang.
     */
    .num_regbanks       = 9,
    .regbanks           = {
        { "hisi-misc",   0x20120000, 0x10000 },
        { "hisi-ddr",    0x20110000, 0x10000 },
        { "hisi-pwm",    0x20130000, 0x10000 },
        { "hisi-nandc",  0x10000000, 0x10000 },
        { "hisi-i2c-v1", 0x200D0000, 0x1000 },
        { "hisi-viu",    0x20580000, 0x40000 },
        { "hisi-vpss",   0x20600000, 0x10000 },
        { "hisi-vedu",   0x20620000, 0x10000 },
        { "hisi-aiao",   0x20650000, 0x10000 },
    },
};

/*
 * Hi3516CV200 (V2): pre-2015, 1M economy.  ARM926EJ-S @550MHz.
 * Video: H.264, 960P@30fps.  Memory: 32/64MB DDR2 integrated.
 * Platform family: hi3518ev200 (same die, =hi3516cv200).
 *
 * 0x20xxxxxx peripheral space.  Uses hieth-sf in vendor kernel but FEMAC
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

    /*
     * Register bank stubs — vendor .ko modules access video/peripheral
     * hardware during init.  Without mapped regions, reads return 0
     * from QEMU's "unimplemented" handler, causing poll loops to hang.
     */
    .num_regbanks       = 7,
    .regbanks           = {
        { "hisi-misc",   0x20120000, 0x10000 },
        { "hisi-ddr",    0x20110000, 0x10000 },
        { "hisi-pwm",    0x20130000, 0x10000 },
        { "hisi-nandc",  0x10000000, 0x10000 },
        { "hisi-viu",    0x20580000, 0x40000 },
        { "hisi-vpss",   0x20600000, 0x10000 },
        { "hisi-aiao",   0x20650000, 0x10000 },
    },
};

/*
 * Hi3516AV100 (V2A): pre-2015, 5M professional.  Cortex-A7 @600MHz.
 * Video: H.265, WDR, 5M@25fps.  First HiSilicon IPC SoC with H.265.
 * Platform family: hi3516dv100 (4M@25fps, same SDK).
 *
 * Cortex-A7 + GICv2 but with V1/V2-era 0x20xxxxxx peripheral addresses.
 * Uses HISFC350 flash controller (like CV100) and GMAC (not FEMAC).
 * No ARM arch timer — SP804 at 50 MHz is primary clocksource.
 */
static const HisiSoCConfig hi3516av100_soc = {
    .name               = "hi3516av100",
    .desc               = "HiSilicon Hi3516AV100 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_AV100,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x20301000,       /* V1-era address space! */
    .gic_cpu_base       = 0x20302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x20050000,
    .crg_base           = 0x20030000,

    .num_uarts          = 4,
    .uart_bases         = { 0x20080000, 0x20090000, 0x200A0000, 0x20230000 },
    .uart_irqs          = { 8, 9, 10, 11 },

    .num_timers         = 2,
    .timer_bases        = { 0x20000000, 0x20010000 },
    .timer_irqs         = { 3, 4 },
    .timer_freq         = 3000000,          /* 3 MHz — matches DT clocksource;
                                             * clockevent 16.7x slow but avoids
                                             * timer storm from clocksource speedup */

    .num_spis           = 2,
    .spi_bases          = { 0x200C0000, 0x200E0000 },
    .spi_irqs           = { 12, 13 },

    .fmc_ctrl_base      = 0x10010000,
    .fmc_mem_base       = 0x58000000,
    .fmc_type           = "hisi-sfc350",    /* HISFC350, same as CV100 */

    .gpio_base          = 0x20140000,
    .gpio_count         = 15,               /* ports 0-14, skip port 15 at 0x20260000 */
    .gpio_stride        = 0x10000,
    .gpio_irq           = 47,               /* shared IRQ for all ports (GIC) */

    /* No FEMAC — uses GMAC (higmac) which is not emulated */

    .num_himci          = 2,
    .himci_bases        = { 0x206E0000, 0x206F0000 },
    .himci_irqs         = { 19, 20 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x200D0000, 0x20240000, 0x20250000 },

    .mipi_rx_base       = 0x20680000,
    .mipi_rx_irq        = 34,

    .rtc_base           = 0x20060000,
    .rtc_irq            = 7,

    .vedu_base          = 0x20640000,
    .jpge_base          = 0x20660000,
    .vedu_irq           = 43,
    .jpge_irq           = 41,

    .wdt_base           = 0x20040000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_regbanks       = 9,
    .regbanks           = {
        { "hisi-misc",       0x20120000, 0x10000 },
        { "hisi-ddr",        0x20110000, 0x10000 },
        { "hisi-pwm",        0x20130000, 0x10000 },
        { "hisi-gmac",       0x10090000, 0x10000 },
        { "hisi-nandc",      0x10000000, 0x1000  },
        /* USB/SNAND left unmapped — regbanks store poll-bit writes
         * causing infinite loops in EHCI handshake and SPI NAND OP */
        { "hisi-regulator",  0x20270000, 0x1000  },
        { "hisi-viu",        0x20580000, 0x40000 },
        { "hisi-vpss",       0x20600000, 0x10000 },
        { "hisi-aiao",       0x20650000, 0x10000 },
    },
};

/*
 * Hi3516CV300 (V3): ~2017, 2M mainstream.  ARM926EJ-S @800MHz.
 * Video: H.265, WDR, 1080P@30fps, 2K fisheye VI.  First ARM9 with H.265.
 * Platform family: hi3516ev100 (1M, H.265 1080P@20fps, 64MB DRAM, LiteOS).
 *
 * New 0x12xxxxxx address map (breaks from V1/V2).
 * VIC at 0x10040000.  Timer at 24 MHz.
 */
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

/*
 * Hi3516CV500 (V3.5): 2018, 3M smart-vision.  Dual Cortex-A7 @900MHz.
 * Video: H.265/H.264, 3M(2304x1296)@20fps / 1080P@30fps.  NPU: 0.5 TOPS.
 * Memory: ext DDR3(L)/DDR4 up to 8Gbit.  280-pin TFBGA 12x12mm.
 * Platform family: hi3516av300 (4K, 1.0 TOPS), hi3516dv300 (5M, 1.0 TOPS).
 *
 * Cortex-A7 + GICv2 (like V4), but unique peripheral address map distinct
 * from both V3 and V4.  RAM at 0x80000000 (like V1-V3), himciv200 MMC, 40 KB SRAM.
 */
static const HisiSoCConfig hi3516cv500_soc = {
    .name               = "hi3516cv500",
    .desc               = "HiSilicon Hi3516CV500 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_CV500,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 40 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x120A0000, 0x120A1000, 0x120A2000 },
    .uart_irqs          = { 6, 7, 8 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 1, 2 },

    .num_spis           = 3,
    .spi_bases          = { 0x120C0000, 0x120C1000, 0x120C2000 },
    .spi_irqs           = { 68, 69, 70 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x120D0000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 16,           /* per-port: SPI 16..26 (GIC) */

    .femac_base         = 0x10010000,
    .femac_irq          = 32,

    .num_himci          = 3,
    .himci_bases        = { 0x10100000, 0x100F0000, 0x10020000 },
    .himci_irqs         = { 64, 30, 31 },

    .num_i2c            = 7,
    .i2c_bases          = { 0x120B0000, 0x120B1000, 0x120B2000, 0x120B3000,
                            0x120B5000, 0x120B6000, 0x120B7000 },

    .mipi_rx_base       = 0x113A0000,
    .mipi_rx_irq        = 57,

    .rtc_base           = 0x12080000,
    .rtc_irq            = 5,

    .vedu_base          = 0x11500000,
    .jpge_base          = 0x11220000,
    .vedu_irq           = 40,
    .jpge_irq           = 36,

    .wdt_base           = 0x12051000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 3,
    .crg_defaults       = {
        { 0x1B8, (1 << 18) },          /* UART clock: 24 MHz select */
        { 0x144, 0x02 },               /* FMC clock enable */
        { 0x16C, 0x02 },               /* ETH clock enable */
    },

    .num_regbanks       = 10,
    .regbanks           = {
        { "hisi-misc",       0x12030000, 0x8000  },
        { "hisi-ddr",        0x12060000, 0x10000 },
        { "hisi-iocfg",      0x12040000, 0x10000 },
        { "hisi-iocfg2",     0x10FF0000, 0x10000 },
        { "hisi-pwm",        0x12070000, 0x10000 },
        { "hisi-usb3",       0x100E0000, 0x10000 },
        { "hisi-vi-cap",     0x11300000, 0x100000 },
        { "hisi-vi-proc",    0x11000000, 0x40000 },
        { "hisi-vpss",       0x11040000, 0x10000 },
        { "hisi-aiao",       0x113B0000, 0x20000 },
    },
};

/*
 * Hi3519V101 (V3A): ~2017, 4K professional.
 * CPU: Cortex-A17 @1.25GHz + Cortex-A7 @800MHz (big.LITTLE).
 * Video: H.265, 4-frame WDR, 4K@30fps + 2M@30fps simultaneous.
 * Sensor: dual input, max 16M pixels combined.
 * Platform family: hi3516av200 (5M@30fps+720P@30fps, same big.LITTLE).
 *
 * V3A generation — big.LITTLE + GICv2, V3-era peripheral addresses
 * (0x121xxxxx UARTs/SPI/GPIO like CV300).
 * Uses GMAC (not FEMAC) for Ethernet; GMAC not yet emulated.
 */
static const HisiSoCConfig hi3519v101_soc = {
    .name               = "hi3519v101",
    .desc               = "HiSilicon Hi3519V101 (Cortex-A7)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_19V101,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x80000000,
    .sram_base          = 0x04010000,
    .sram_size          = 64 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x10301000,
    .gic_cpu_base       = 0x10302000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x12020000,
    .crg_base           = 0x12010000,

    .num_uarts          = 5,
    .uart_bases         = { 0x12100000, 0x12101000, 0x12102000,
                            0x12103000, 0x12104000 },
    .uart_irqs          = { 4, 5, 6, 7, 8 },

    .num_timers         = 2,
    .timer_bases        = { 0x12000000, 0x12001000 },
    .timer_irqs         = { 64, 66 },
    .timer_freq         = 3000000,          /* 3 MHz */

    .num_spis           = 4,
    .spi_bases          = { 0x12120000, 0x12121000, 0x12122000, 0x12123000 },
    .spi_irqs           = { 9, 10, 11, 12 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x14000000,

    .gpio_base          = 0x12140000,
    .gpio_count         = 17,               /* ports 0-14, phantom 15, port 16 */
    .gpio_stride        = 0x1000,
    .gpio_irq           = 43,               /* shared IRQ for all ports (GIC) */

    /* No FEMAC — uses GMAC (higmac) which is not yet emulated */

    .num_himci          = 3,
    .himci_bases        = { 0x100c0000, 0x100d0000, 0x100e0000 },
    .himci_irqs         = { 23, 24, 13 },

    .num_i2c            = 4,
    .i2c_bases          = { 0x12110000, 0x12111000, 0x12112000, 0x12113000 },

    .mipi_rx_base       = 0x11300000,
    .mipi_rx_irq        = 28,

    .rtc_base           = 0x12090000,
    .rtc_irq            = 1,

    .vedu_base          = 0x11280000,
    .jpge_base          = 0x11200000,
    .vedu_irq           = 37,
    .jpge_irq           = 38,

    .wdt_base           = 0x12080000,
    .wdt_irq            = -1,
    .wdt_freq           = 3000000,

    .num_crg_defaults   = 1,
    .crg_defaults       = {
        /* APLL ctrl_reg2: fbdiv=792, refdiv=24 → 792 MHz (prevents div-by-zero) */
        { 0x04, (24 << 12) | 792 },
    },

    .num_regbanks       = 11,
    .regbanks           = {
        { "hisi-misc",       0x12030000, 0x10000 },
        { "hisi-ddr",        0x12060000, 0x10000 },
        { "hisi-iocfg",      0x12160000, 0x10000 },
        { "hisi-pwm",        0x12130000, 0x10000 },
        { "hisi-usb-ehci",   0x10120000, 0x10000 },
        { "hisi-usb-ohci",   0x10110000, 0x10000 },
        { "hisi-gmac",       0x10050000, 0x10000 },
        { "hisi-vi-cap",     0x11380000, 0x100000 },
        { "hisi-vou",        0x11000000, 0x20000 },
        { "hisi-vpss",       0x11180000, 0x10000 },
        { "hisi-aiao",       0x11080000, 0x10000 },
    },
};

/*
 * Hi3516EV300 (V4): 2019, 5M professional.  Cortex-A7 @900MHz.
 * Video: H.265/H.264, 5M(2592x1944)@15fps / 4M(2688x1520)@25fps.
 * Memory: 1Gbit DDR3L integrated (128MB).  279-pin TFBGA.
 * V4 generation — new RAM base at 0x40000000.  SDHCI replaces himciv200.
 * 10 GPIO groups.  Per-port GIC IRQs (SPI 16+).
 */
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

    .gzip_base          = 0x11310000,

    .num_regbanks       = 1,
    .regbanks           = {
        { "hisi-ive",    0x11320000, 0x10000 },
    },
};

/*
 * Hi3516EV200 (V4): 2019, 3M economy.  Cortex-A7 @900MHz.
 * Video: H.265/H.264, 3M(2304x1296)@20fps.  Memory: 512Mb DDR2 (64MB).
 * Economy variant — 8 GPIO groups.  Same peripheral addresses as EV300.
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
 * Hi3518EV300 (V4): 2019, 3M consumer/IoT.  Cortex-A7 @900MHz.
 * Video: H.265/H.264, 3M(2304x1296)@20fps.  Memory: 512Mb DDR2 (64MB).
 * No integrated FE PHY — boards need external PHY or go Ethernet-less.
 * 8 GPIO groups.
 * FEMAC controller exists but no on-chip PHY; we still instantiate FEMAC
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
 * Hi3516DV200 (V4): 2019, 5M professional.  Cortex-A7 @900MHz.
 * Video: H.265/H.264, 5M(3072x1728)@20fps.  Memory: ext DDR3/DDR3L up to 4Gbit.
 * Professional variant — 10 GPIO groups.  Same peripheral layout as EV300.
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
 * Goke (国科微) variants — die-identical V4 HiSilicon silicon rebranded.
 * Hardware addresses, IRQs, GPIO counts all match the HiSilicon original.
 *
 * Goke V4 (2021, emulated here):
 *   GK7205V200 = Hi3516EV200  3M@25fps,  512Mb DDR2
 *   GK7205V210 (not emulated) 4M@20fps,  512Mb DDR
 *   GK7205V300 = Hi3516EV300  5M@25fps,  1Gb DDR
 *   GK7202V300 = Hi3518EV300  4M@20fps,  512Mb DDR
 *   GK7605V100 = Hi3516DV200  5M@25fps
 *
 * Goke next-gen (2022+, NOT emulated, separate Goke designs with NPU):
 *   GK7205V500  5M@25fps  0.5 TOPS  512Mb DDR   (2022)
 *   GK7205V510  5M@30fps  1 TOPS    1Gb DDR     (2022)
 *   GK7205V530  5M@30fps  1 TOPS    ext DDR     (2022)
 *   GK7202V330  5M@25fps  0.5 TOPS  512Mb DDR   (2022)
 *   GK7606V100  4K@60fps  2 TOPS                (2022)
 *   GK7609V100  8K@30fps  4 TOPS                (2022)
 *   GK7608V100  4K@60fps  4+4 TOPS              (2021)
 *   GK7605V200  4K@30fps  2 TOPS                (2023)
 *   GK7205V600  4K@20fps  1 TOPS                (2024)
 *   GK7606V200  4K@60fps  4 TOPS                (2024)
 *   GK7609V200  8K@30fps  8 TOPS    H.266       (2024)
 *   GK7608V200  4K@60fps  8 TOPS    H.266       (2025)
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
    .num_regbanks       = 15,                               \
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
        { "hisi-npu",        0x11340000, 0x10000 },         \
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

/*
 * Goke next-gen (2022+) — Goke's own designs with NPU, V4-compatible address map.
 * CPU: Cortex-A7 @1GHz.  Video: H.265/H.264, 5M@25-30fps.
 * NPU: 0.5 TOPS (V500, V330) or 1.0 TOPS (V510, V530).
 * Same peripheral addresses as V4; NPU added at 0x11340000.
 * SDK: XMediaIPCLinuxV100R002C00SPC020, kernel 5.10.
 *
 * GK7205V500:  5M@25fps, 0.5 TOPS, 512Mb DDR2 MCP, FEPHY
 * GK7205V510:  5M@30fps, 1.0 TOPS, 1Gb DDR3 MCP, FEPHY
 * GK7205V530:  5M@30fps, 1.0 TOPS, ext DDR, FEPHY
 * GK7202V330:  5M@25fps, 0.5 TOPS, 512Mb DDR2 MCP, no FEPHY
 */

static const HisiSoCConfig gk7205v500_soc = {
    .name               = "gk7205v500",
    .desc               = "Goke GK7205V500 (Cortex-A7, 0.5 TOPS NPU)",
    .soc_id             = GOKE_SOC_ID_7205V500,
    .gpio_count         = 8,
    HISI_V4_COMMON_PERIPH,
};

static const HisiSoCConfig gk7205v510_soc = {
    .name               = "gk7205v510",
    .desc               = "Goke GK7205V510 (Cortex-A7, 1.0 TOPS NPU)",
    .soc_id             = GOKE_SOC_ID_7205V510,
    .gpio_count         = 8,
    HISI_V4_COMMON_PERIPH,
};

static const HisiSoCConfig gk7205v530_soc = {
    .name               = "gk7205v530",
    .desc               = "Goke GK7205V530 (Cortex-A7, 1.0 TOPS NPU, ext DDR)",
    .soc_id             = GOKE_SOC_ID_7205V530,
    .gpio_count         = 8,
    HISI_V4_COMMON_PERIPH,
};

static const HisiSoCConfig gk7202v330_soc = {
    .name               = "gk7202v330",
    .desc               = "Goke GK7202V330 (Cortex-A7, 0.5 TOPS NPU, no FEPHY)",
    .soc_id             = GOKE_SOC_ID_7202V330,
    .gpio_count         = 8,
    HISI_V4_COMMON_PERIPH,
};

/*
 * V5 family (~2023): Hi3516CV608 / CV610 / CV613 — Dual Cortex-A7 MP2 + NPU.
 * Video: H.265/H.264, up to 4K.  New 0x11xxxxxx address map, GIC @ 0x124xxxxx.
 * NPU: 0.2 TOPS (CV608), 0.5 TOPS (CV610), 1 TOPS (CV613).
 * Same die, different feature tiers.
 * All share identical peripheral addresses; only SoC ID differs.
 *
 * Datasheet model suffixes → chip IDs (from Section 1.2.14):
 *   10B  → Hi3516CV610  0x3516C610  0.5 TOPS, 5M,  DDR2, QFN
 *   20S  → Hi3516CV613  0x3516C613  1 TOPS,   4K,  DDR3, QFN
 *   00S  → unknown ID                1 TOPS,   4K,  DDR3, QFN
 *   20G  → unknown ID                1 TOPS,   4K,  DDR3, QFN, GB35114
 *   00G  → unknown ID                1 TOPS,   4K,  ext DDR3, BGA, GB35114
 *   (separate chip) Hi3516CV608  0x3516C608  0.2 TOPS, 3M, DDR2, QFN
 *
 * ipctool also lists Hi3516DV500 (0x3516D500) and Hi3519DV500 (0x3519D500)
 * as HISI_OT generation — likely same V5 address map, awaiting SDK/lab.
 */
static const HisiSoCConfig hi3516cv608_soc = {
    .name               = "hi3516cv608",
    .desc               = "HiSilicon Hi3516CV608 (Cortex-A7 MP2, ~CV610)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_CV608,
    .ram_size_default   = 64 * MiB,

    .ram_base           = 0x40000000,
    .sram_base          = 0x04020000,
    .sram_size          = 80 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x12401000,
    .gic_cpu_base       = 0x12402000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x11020000,
    .crg_base           = 0x11010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x11040000, 0x11041000, 0x11042000 },
    .uart_irqs          = { 10, 11, 12 },

    .num_timers         = 0,

    .num_spis           = 2,
    .spi_bases          = { 0x11070000, 0x11071000 },
    .spi_irqs           = { 19, 20 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x0F000000,

    .gpio_base          = 0x11090000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 23,

    .femac_base         = 0x10290000,
    .femac_irq          = 44,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10030000, 0x10040000 },
    .sdhci_irqs         = { 42, 43 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x11060000, 0x11061000, 0x11062000 },

    .mipi_rx_base       = 0x173C0000,
    .mipi_rx_irq        = 64,

    .rtc_base           = 0x11110000,
    .rtc_irq            = 37,

    .vedu_base          = 0x17140000,
    .jpge_base          = 0x171C0000,
    .vedu_irq           = 69,
    .jpge_irq           = 70,

    .wdt_base           = 0x11030000,
    .wdt_irq            = 3,
    .wdt_freq           = 3000000,

    .num_regbanks       = 12,
    .regbanks           = {
        { "hisi-misc",       0x11024000, 0x5000  },
        { "hisi-ddr",        0x11140000, 0x20000 },
        { "hisi-iocfg0",     0x10260000, 0x10000 },
        { "hisi-iocfg1",     0x11130000, 0x10000 },
        { "hisi-pwm",        0x11080000, 0x1000  },
        { "hisi-usb2",       0x10300000, 0x10000 },
        { "hisi-npu",        0x14000000, 0x800000 },
        { "hisi-vi-cap",     0x17400000, 0x100000 },
        { "hisi-vi-proc",    0x17800000, 0x40000 },
        { "hisi-vpss",       0x17900000, 0x10000 },
        { "hisi-vgs",        0x17240000, 0x10000 },
        { "hisi-aiao",       0x17C00000, 0x50000 },
    },
};

/*
 * Hi3516CV610: V5 generation (~2023) — Dual Cortex-A7 MP2, GICv2.
 * Completely new address map (0x11xxxxxx peripherals, GIC @ 0x124xxxxx).
 * No SP804 timer — uses ARM arch timer exclusively.
 * FEMAC-v2 with integrated FEPHY, "nebula,sdhci" for SD/MMC.
 * FMC memory window below registers at 0x0F000000.
 */
static const HisiSoCConfig hi3516cv610_soc = {
    .name               = "hi3516cv610",
    .desc               = "HiSilicon Hi3516CV610 (Cortex-A7 MP2)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_CV610,
    .ram_size_default   = 128 * MiB,

    .ram_base           = 0x40000000,
    .sram_base          = 0x04020000,
    .sram_size          = 80 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x12401000,
    .gic_cpu_base       = 0x12402000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x11020000,
    .crg_base           = 0x11010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x11040000, 0x11041000, 0x11042000 },
    .uart_irqs          = { 10, 11, 12 },

    /* No SP804 timers — ARM arch timer only (24 MHz) */
    .num_timers         = 0,

    .num_spis           = 2,
    .spi_bases          = { 0x11070000, 0x11071000 },
    .spi_irqs           = { 19, 20 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x0F000000,

    .gpio_base          = 0x11090000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 23,           /* per-port: SPI 23..33 */

    .femac_base         = 0x10290000,
    .femac_irq          = 44,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10030000, 0x10040000 },
    .sdhci_irqs         = { 42, 43 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x11060000, 0x11061000, 0x11062000 },

    .mipi_rx_base       = 0x173C0000,
    .mipi_rx_irq        = 64,

    .rtc_base           = 0x11110000,
    .rtc_irq            = 37,

    .vedu_base          = 0x17140000,
    .jpge_base          = 0x171C0000,
    .vedu_irq           = 69,
    .jpge_irq           = 70,

    .wdt_base           = 0x11030000,
    .wdt_irq            = 3,
    .wdt_freq           = 3000000,

    .num_regbanks       = 12,
    .regbanks           = {
        { "hisi-misc",       0x11024000, 0x5000  },
        { "hisi-ddr",        0x11140000, 0x20000 },
        { "hisi-iocfg0",     0x10260000, 0x10000 },
        { "hisi-iocfg1",     0x11130000, 0x10000 },
        { "hisi-pwm",        0x11080000, 0x1000  },
        { "hisi-usb2",       0x10300000, 0x10000 },
        { "hisi-npu",        0x14000000, 0x800000 },
        { "hisi-vi-cap",     0x17400000, 0x100000 },
        { "hisi-vi-proc",    0x17800000, 0x40000 },
        { "hisi-vpss",       0x17900000, 0x10000 },
        { "hisi-vgs",        0x17240000, 0x10000 },
        { "hisi-aiao",       0x17C00000, 0x50000 },
    },
};

static const HisiSoCConfig hi3516cv613_soc = {
    .name               = "hi3516cv613",
    .desc               = "HiSilicon Hi3516CV613 (Cortex-A7 MP2, ~CV610)",
    .cpu_type           = ARM_CPU_TYPE_NAME("cortex-a7"),
    .soc_id             = HISI_SOC_ID_CV613,
    .ram_size_default   = 128 * MiB,

    .ram_base           = 0x40000000,
    .sram_base          = 0x04020000,
    .sram_size          = 80 * KiB,

    .use_gic            = true,
    .gic_dist_base      = 0x12401000,
    .gic_cpu_base       = 0x12402000,
    .gic_num_spi        = 128,

    .sysctl_base        = 0x11020000,
    .crg_base           = 0x11010000,

    .num_uarts          = 3,
    .uart_bases         = { 0x11040000, 0x11041000, 0x11042000 },
    .uart_irqs          = { 10, 11, 12 },

    .num_timers         = 0,

    .num_spis           = 2,
    .spi_bases          = { 0x11070000, 0x11071000 },
    .spi_irqs           = { 19, 20 },

    .fmc_ctrl_base      = 0x10000000,
    .fmc_mem_base       = 0x0F000000,

    .gpio_base          = 0x11090000,
    .gpio_count         = 11,
    .gpio_stride        = 0x1000,
    .gpio_irq_start     = 23,

    .femac_base         = 0x10290000,
    .femac_irq          = 44,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10030000, 0x10040000 },
    .sdhci_irqs         = { 42, 43 },

    .num_i2c            = 3,
    .i2c_bases          = { 0x11060000, 0x11061000, 0x11062000 },

    .mipi_rx_base       = 0x173C0000,
    .mipi_rx_irq        = 64,

    .rtc_base           = 0x11110000,
    .rtc_irq            = 37,

    .vedu_base          = 0x17140000,
    .jpge_base          = 0x171C0000,
    .vedu_irq           = 69,
    .jpge_irq           = 70,

    .wdt_base           = 0x11030000,
    .wdt_irq            = 3,
    .wdt_freq           = 3000000,

    .num_regbanks       = 12,
    .regbanks           = {
        { "hisi-misc",       0x11024000, 0x5000  },
        { "hisi-ddr",        0x11140000, 0x20000 },
        { "hisi-iocfg0",     0x10260000, 0x10000 },
        { "hisi-iocfg1",     0x11130000, 0x10000 },
        { "hisi-pwm",        0x11080000, 0x1000  },
        { "hisi-usb2",       0x10300000, 0x10000 },
        { "hisi-npu",        0x14000000, 0x800000 },
        { "hisi-vi-cap",     0x17400000, 0x100000 },
        { "hisi-vi-proc",    0x17800000, 0x40000 },
        { "hisi-vpss",       0x17900000, 0x10000 },
        { "hisi-vgs",        0x17240000, 0x10000 },
        { "hisi-aiao",       0x17C00000, 0x50000 },
    },
};

/* ── Machine state with sensor property ────────────────────────────── */

/* Extra state appended to MachineState for the sensor property */
typedef struct {
    MachineState parent_obj;
    char *sensor;
} HisiMachineState;

/* ── Appended DTB fixup ────────────────────────────────────────────── */

/*
 * Fixup appended DTB in the loaded kernel payload.
 *
 * Many OpenIPC uImages have a device tree appended with zero padding
 * and no /chosen node.  The kernel's atags_to_fdt() tries to patch
 * the DTB in-place with bootargs and initrd info; without padding it
 * silently corrupts the DTB and boot fails.  Some DTBs also lack a
 * /chosen { stdout-path } node needed for console output.
 *
 * This runs as a reset handler (after ROM blobs are loaded into RAM)
 * so the kernel payload is already in guest memory.
 */
/*
 * Read the kernel file, find the appended DTB, patch it (add /chosen
 * with stdout-path, add padding for atags_to_fdt), and write a temp
 * file with the patched kernel.  Returns the temp file path (caller
 * frees), or NULL if no patching was needed.
 */
static char *hisilicon_patch_appended_dtb(const char *kernel_filename,
                                           hwaddr load_addr,
                                           const HisiSoCConfig *c)
{
    gsize file_size;
    gchar *buf;
    (void)load_addr; /* reserved for future use */

    if (!kernel_filename ||
        !g_file_get_contents(kernel_filename, &buf, &file_size, NULL)) {
        return NULL;
    }

    /* Skip uImage header if present */
    size_t hdr_off = 0;
    if (file_size > 64 &&
        (uint8_t)buf[0] == 0x27 && (uint8_t)buf[1] == 0x05 &&
        (uint8_t)buf[2] == 0x19 && (uint8_t)buf[3] == 0x56) {
        hdr_off = 64;
    }

    /* Scan payload for FDT magic (take the last match = appended DTB) */
    size_t dtb_off = 0;
    uint32_t dtb_size = 0;
    for (size_t i = hdr_off + 256 * 1024; i + 8 < file_size; i += 4) {
        if ((uint8_t)buf[i]   == 0xd0 && (uint8_t)buf[i+1] == 0x0d &&
            (uint8_t)buf[i+2] == 0xfe && (uint8_t)buf[i+3] == 0xed) {
            uint32_t sz = ((uint8_t)buf[i+4] << 24) |
                          ((uint8_t)buf[i+5] << 16) |
                          ((uint8_t)buf[i+6] << 8)  |
                          (uint8_t)buf[i+7];
            if (sz > 1024 && sz < 256 * 1024 && i + sz <= file_size) {
                dtb_off = i;
                dtb_size = sz;
            }
        }
    }

    if (!dtb_off) {
        g_free(buf);
        return NULL;
    }

    /* Patch the DTB */
    uint32_t new_dtb_size = dtb_size + 4096;
    void *dtb = g_malloc0(new_dtb_size);
    memcpy(dtb, buf + dtb_off, dtb_size);

    if (fdt_open_into(dtb, dtb, new_dtb_size) != 0) {
        g_free(dtb);
        g_free(buf);
        return NULL;
    }

    /* Ensure /chosen with stdout-path exists */
    if (fdt_path_offset(dtb, "/chosen") < 0) {
        fdt_add_subnode(dtb, 0, "chosen");
    }
    int chosen = fdt_path_offset(dtb, "/chosen");
    if (chosen >= 0 && !fdt_getprop(dtb, chosen, "stdout-path", NULL)) {
        fdt_setprop_string(dtb, chosen, "stdout-path",
                           "serial0:115200n8");
    }

    /*
     * If timer_freq is set, patch fixed-clock "clk_apb" to match.
     * Some SoCs (hi3516av100) have SP804 with split clocks (50 MHz + 3 MHz)
     * that QEMU can't model — unify them to avoid timer storms.
     */
    if (c->timer_freq) {
        int soc_node = fdt_path_offset(dtb, "/soc");
        if (soc_node >= 0) {
            int node;
            fdt_for_each_subnode(node, dtb, soc_node) {
                const char *compat = fdt_getprop(dtb, node, "compatible", NULL);
                const char *name = fdt_get_name(dtb, node, NULL);
                if (compat && name &&
                    !strcmp(compat, "fixed-clock") &&
                    !strcmp(name, "clk_apb")) {
                    uint32_t freq = cpu_to_be32(c->timer_freq);
                    fdt_setprop_inplace(dtb, node, "clock-frequency",
                                        &freq, sizeof(freq));
                }
            }
        }
    }

    /*
     * Disable NAND controllers that we don't emulate — their drivers
     * poll hardware status registers that cause hangs on regbank stubs.
     */
    {
        const char *disable_paths[] = {
            "/soc/spi_nand_controller",
            "/soc/nand_controller",
            NULL
        };
        for (int i = 0; disable_paths[i]; i++) {
            int node = fdt_path_offset(dtb, disable_paths[i]);
            if (node >= 0) {
                fdt_setprop_string(dtb, node, "status", "disabled");
            }
        }
    }

    /* Pack then re-expand with padding for kernel's atags_to_fdt() */
    fdt_pack(dtb);
    uint32_t packed = fdt_totalsize(dtb);
    fdt_open_into(dtb, dtb, packed + 4096);
    uint32_t final_dtb_size = fdt_totalsize(dtb);

    /*
     * Build new uImage: original_header + kernel_before_dtb + patched_dtb.
     * For raw (non-uImage) kernels, hdr_off=0 so the header is absent.
     * We must preserve the uImage header so QEMU's load_uimage_as()
     * recognizes the file format and uses the correct load address.
     * The uImage data-size and CRC fields are updated to match.
     */
    size_t new_payload_size = (dtb_off - hdr_off) + final_dtb_size;
    size_t new_file_size = hdr_off + new_payload_size;
    gchar *new_buf = g_malloc(new_file_size);

    /* Copy header (if any) */
    if (hdr_off) {
        memcpy(new_buf, buf, hdr_off);
    }
    /* Copy kernel before DTB */
    memcpy(new_buf + hdr_off, buf + hdr_off, dtb_off - hdr_off);
    /* Copy patched DTB */
    memcpy(new_buf + dtb_off, dtb, final_dtb_size);

    /* Update uImage header if present */
    if (hdr_off == 64) {
        /* data size at offset 12 (big-endian) */
        uint32_t data_size = new_payload_size;
        new_buf[12] = (data_size >> 24) & 0xff;
        new_buf[13] = (data_size >> 16) & 0xff;
        new_buf[14] = (data_size >> 8) & 0xff;
        new_buf[15] = data_size & 0xff;
        /* Zero header CRC for recalculation */
        memset(new_buf + 4, 0, 4);
        /* Recalculate data CRC at offset 24 */
        uint32_t dcrc = crc32(0, (const uint8_t *)new_buf + hdr_off, data_size);
        new_buf[24] = (dcrc >> 24) & 0xff;
        new_buf[25] = (dcrc >> 16) & 0xff;
        new_buf[26] = (dcrc >> 8) & 0xff;
        new_buf[27] = dcrc & 0xff;
        /* Recalculate header CRC */
        uint32_t hcrc = crc32(0, (uint8_t *)new_buf, 64);
        new_buf[4] = (hcrc >> 24) & 0xff;
        new_buf[5] = (hcrc >> 16) & 0xff;
        new_buf[6] = (hcrc >> 8) & 0xff;
        new_buf[7] = hcrc & 0xff;
    }

    /* Write temp file */
    GError *err = NULL;
    char *tmppath = NULL;
    int fd = g_file_open_tmp("hisi-kernel-XXXXXX", &tmppath, &err);
    if (fd < 0) {
        g_free(new_buf);
        g_free(dtb);
        g_free(buf);
        return NULL;
    }
    if (write(fd, new_buf, new_file_size) != (ssize_t)new_file_size) {
        close(fd);
        unlink(tmppath);
        g_free(tmppath);
        g_free(new_buf);
        g_free(dtb);
        g_free(buf);
        return NULL;
    }
    close(fd);

    g_free(new_buf);
    g_free(dtb);
    g_free(buf);
    return tmppath;
}


/* ── Shared machine init ───────────────────────────────────────────── */

/* PPI numbers — same for all GICv2 HiSilicon SoCs */
#define HISI_PPI_HYPTIMER   10
#define HISI_PPI_VIRTTIMER  11
#define HISI_PPI_SECTIMER   13
#define HISI_PPI_PHYSTIMER  14

static struct arm_boot_info hisilicon_binfo;

/* ARM instruction encoding helpers for boot ROM generation */
static inline uint32_t arm_movw(int rd, uint16_t imm16)
{
    /* MOVW Rd, #imm16 (ARMv7): Rd[15:0] = imm16, Rd[31:16] = 0 */
    return 0xe3000000 | ((imm16 >> 12) << 16) | (rd << 12) | (imm16 & 0xfff);
}

static inline uint32_t arm_movt(int rd, uint16_t imm16)
{
    /* MOVT Rd, #imm16 (ARMv7): Rd[31:16] = imm16, Rd[15:0] unchanged */
    return 0xe3400000 | ((imm16 >> 12) << 16) | (rd << 12) | (imm16 & 0xfff);
}

/*
 * Write a boot ROM at address 0 that emulates the HiSilicon boot ROM:
 * copies U-Boot from the SPI NOR flash memory window to DDR and jumps to it.
 *
 * Real HiSilicon boot ROMs (e.g. Hi3516CV500) do a two-stage load:
 *   1) 24 KB from flash to SRAM (contains DDR init + BSBM header)
 *   2) 512 KB from flash+0x6000 to DDR
 * Since QEMU DDR is already available, we simplify to a single copy
 * of the boot partition (256 KB) from flash window to DDR base.
 */
static void hisilicon_write_bootrom(MemoryRegion *sysmem,
                                     const HisiSoCConfig *c)
{
    hwaddr flash_src = c->fmc_mem_base;         /* e.g. 0x14000000 */
    hwaddr ram_dst   = c->ram_base;             /* e.g. 0x40000000 */
    uint32_t copy_sz = 0x40000;                 /* 256 KB boot partition */

    uint32_t rom[] = {
        /* Set SVC mode, disable IRQ/FIQ */
        cpu_to_le32(0xe321f0d3),                /* msr cpsr_c, #0xD3      */

        /* r0 = flash source address */
        cpu_to_le32(arm_movw(0, flash_src & 0xffff)),
        cpu_to_le32(arm_movt(0, flash_src >> 16)),

        /* r1 = DDR destination address */
        cpu_to_le32(arm_movw(1, ram_dst & 0xffff)),
        cpu_to_le32(arm_movt(1, ram_dst >> 16)),

        /* r2 = byte count */
        cpu_to_le32(arm_movw(2, copy_sz & 0xffff)),
        cpu_to_le32(arm_movt(2, copy_sz >> 16)),

        /* r3 = end pointer */
        cpu_to_le32(0xe0813002),                /* add r3, r1, r2         */

        /* copy_loop: */
        cpu_to_le32(0xe4904004),                /* ldr r4, [r0], #4       */
        cpu_to_le32(0xe4814004),                /* str r4, [r1], #4       */
        cpu_to_le32(0xe1510003),                /* cmp r1, r3             */
        cpu_to_le32(0x1afffffb),                /* bne copy_loop          */

        /* Jump to DDR (reload r1 since it was used as pointer) */
        cpu_to_le32(arm_movw(1, ram_dst & 0xffff)),
        cpu_to_le32(arm_movt(1, ram_dst >> 16)),
        cpu_to_le32(0xe12fff11),                /* bx r1                  */
    };

    MemoryRegion *bootrom = g_new(MemoryRegion, 1);
    memory_region_init_rom(bootrom, NULL, "hisilicon.bootrom",
                           0x1000, &error_fatal);
    memory_region_add_subregion(sysmem, 0, bootrom);
    /* ROM is read-only after initial write, matching real hardware where
     * address 0 is internal boot ROM.  U-Boot on Cortex-A7 uses VBAR for
     * exception vectors (not address 0).  Firmware with NULL pointer bugs
     * that write to address 0 will have writes silently dropped — same as
     * real silicon where the boot ROM is not writable. */
    address_space_write_rom(&address_space_memory, 0,
                            MEMTXATTRS_UNSPECIFIED, rom, sizeof(rom));
}

static void hisilicon_common_init(MachineState *machine,
                                  const HisiSoCConfig *c)
{
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    ARMCPU *cpu;
    qemu_irq pic[256];
    int num_pic = 0;
    int n;
    bool flash_boot = false;  /* true when booting from SPI NOR flash dump */

    /* SRAM */
    {
        MemoryRegion *sram = g_new(MemoryRegion, 1);
        memory_region_init_ram(sram, NULL, "hisilicon.sram",
                               c->sram_size, &error_fatal);
        memory_region_add_subregion(sysmem, c->sram_base, sram);
    }

    /* Flash controller (HiFMC V100 or HISFC350) */
    if (c->fmc_ctrl_base) {
        const char *fmc_type = c->fmc_type ? c->fmc_type : "hisi-fmc";
        DeviceState *fmc = qdev_new(fmc_type);
        SysBusDevice *fmcbus = SYS_BUS_DEVICE(fmc);
        sysbus_realize_and_unref(fmcbus, &error_fatal);
        sysbus_mmio_map(fmcbus, 0, c->fmc_ctrl_base);
        sysbus_mmio_map(fmcbus, 1, c->fmc_mem_base);

        /* Check if flash-file was set (via -global hisi-fmc.flash-file=...) */
        if (!machine->kernel_filename) {
            char *ff = object_property_get_str(OBJECT(fmc), "flash-file", NULL);
            if (ff && ff[0]) {
                flash_boot = true;
            }
            g_free(ff);
        }
    }

    /*
     * Address 0 page — either a boot ROM (for flash boot) or a trap page
     * (for normal -kernel boot where NULL function pointers must be safe).
     */
    if (flash_boot) {
        hisilicon_write_bootrom(sysmem, c);
    } else {
        MemoryRegion *trap = g_new(MemoryRegion, 1);
        memory_region_init_rom(trap, NULL, "hisilicon.trapnull",
                               0x1000, &error_fatal);
        memory_region_add_subregion(sysmem, 0, trap);
        uint32_t insn[2] = { cpu_to_le32(0xe3a00000),   /* mov r0, #0 */
                             cpu_to_le32(0xe12fff1e) };  /* bx lr      */
        address_space_write_rom(&address_space_memory, 0,
                                MEMTXATTRS_UNSPECIFIED, insn, 8);
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

    /* Fastboot mode: when no -kernel is given, no flash-file, and a serial
     * backend exists, emulate the boot ROM download protocol on UART0
     * instead of creating PL011 immediately.  The hisi-fastboot device will
     * hand the chardev off to a newly-created PL011 after firmware upload
     * completes.  Flash boot skips fastboot — the boot ROM runs U-Boot
     * from flash directly. */
    bool fastboot_mode = !machine->kernel_filename && !flash_boot
                         && serial_hd(0);
    qemu_irq uart0_irq = NULL;

    /* UARTs */
    for (n = 0; n < c->num_uarts; n++) {
        if (n == 0 && fastboot_mode) {
            uart0_irq = pic[c->uart_irqs[0]];
            continue;   /* UART0 PL011 created later by fastboot device */
        }
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
        if (c->gpio_irq_start) {
            irq = pic[c->gpio_irq_start + n]; /* per-port IRQs (GIC) */
        } else {
            irq = pic[c->gpio_irq];           /* shared IRQ (VIC or GIC) */
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

    /* Hardware GZIP decompressor (used by U-Boot hw_compressed first stage) */
    if (c->gzip_base) {
        DeviceState *gzip = qdev_new("hisi-gzip");
        SysBusDevice *busdev = SYS_BUS_DEVICE(gzip);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, c->gzip_base);
    }

    /* Generic register banks (pin mux, DDR PHY, PWM, etc.) */
    for (n = 0; n < c->num_regbanks; n++) {
        if (c->regbanks[n].base) {
            /* IVE gets a proper functional device instead of regbank */
            if (!strcmp(c->regbanks[n].name, "hisi-ive")) {
                DeviceState *ive = qdev_new("hisi-ive");
                SysBusDevice *busdev = SYS_BUS_DEVICE(ive);
                sysbus_realize_and_unref(busdev, &error_fatal);
                sysbus_mmio_map(busdev, 0, c->regbanks[n].base);
                /* IVE IRQ: EV300=SPI51, CV500=SPI37 — extract from DTS */
                continue;
            }

            DeviceState *rb = qdev_new("hisi-regbank");
            qdev_prop_set_uint32(rb, "size", c->regbanks[n].size);
            qdev_prop_set_string(rb, "name", c->regbanks[n].name);

            sysbus_realize_and_unref(SYS_BUS_DEVICE(rb), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(rb), 0, c->regbanks[n].base);

            if (!strcmp(c->regbanks[n].name, "hisi-nandc")) {
                address_space_stl(&address_space_memory,
                                  c->regbanks[n].base + 0x20, 0x01,
                                  MEMTXATTRS_UNSPECIFIED, NULL);
            }
            /*
             * V1 I2C controller: pre-set I2C_OVER_INTR (bit 0) in the
             * status register (offset 0x0C) so poll loops in the vendor
             * hi_i2c.ko module exit immediately instead of spinning
             * through 4096-iteration timeouts per I2C probe address.
             */
            if (!strcmp(c->regbanks[n].name, "hisi-i2c-v1")) {
                address_space_stl(&address_space_memory,
                                  c->regbanks[n].base + 0x0C, 0x01,
                                  MEMTXATTRS_UNSPECIFIED, NULL);
            }
            /*
             * VICAP/ISP: pre-set PT_INTF_MOD (offset 0x4000) so the
             * ISP driver's init doesn't poll a zero-valued register.
             * Also set ISP interrupt status at 0x41F8 to indicate
             * "frame done" so ISP init completes immediately.
             */
            if (!strcmp(c->regbanks[n].name, "hisi-viu")) {
                address_space_stl(&address_space_memory,
                                  c->regbanks[n].base + 0x41F8, 0xFFFFFFFF,
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

    if (flash_boot) {
        /* Boot from SPI NOR flash dump: boot ROM at 0x0 copies U-Boot from
         * flash window to DDR and jumps to it.  CPU starts at reset vector. */
    } else if (fastboot_mode) {
        /* Boot ROM fastboot: halt CPU and wait for serial firmware upload */
        DeviceState *fb = qdev_new(TYPE_HISI_FASTBOOT);
        qdev_prop_set_chr(fb, "chardev", serial_hd(0));
        hisi_fastboot_setup(fb, CPU(cpu), serial_hd(0),
                            c->uart_bases[0], uart0_irq);
        qdev_realize(fb, NULL, &error_fatal);

        CPUState *cs = CPU(cpu);
        cs->halted = 1;
    } else {
        /* Normal boot path: patch DTB and load kernel */
        char *patched_kernel = NULL;
        if (machine->kernel_filename) {
            patched_kernel = hisilicon_patch_appended_dtb(
                machine->kernel_filename, c->ram_base + 0x8000, c);
            if (patched_kernel) {
                machine->kernel_filename = patched_kernel;
            }
        }

        /*
         * Limit initrd placement to the kernel-visible RAM region.
         * When mem=NM is smaller than -m (e.g., mem=64M with -m 128M
         * to leave room for MMZ), QEMU must place the initrd within
         * the kernel's addressable range, not at the top of all RAM.
         */
        hisilicon_binfo.ram_size = machine->ram_size;
        if (machine->kernel_cmdline) {
            const char *memarg = strstr(machine->kernel_cmdline, "mem=");
            if (memarg) {
                unsigned long mem_mb = strtoul(memarg + 4, NULL, 10);
                if (mem_mb > 0 && mem_mb * MiB < machine->ram_size) {
                    hisilicon_binfo.ram_size = mem_mb * MiB;
                }
            }
        }
        hisilicon_binfo.loader_start = c->ram_base;
        arm_load_kernel(cpu, machine, &hisilicon_binfo);
    }
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
DEFINE_HISI_MACHINE("hi3516av100", hi3516av100, hi3516av100_soc)
DEFINE_HISI_MACHINE("hi3516cv300", hi3516cv300, hi3516cv300_soc)
DEFINE_HISI_MACHINE("hi3516cv500", hi3516cv500, hi3516cv500_soc)
DEFINE_HISI_MACHINE("hi3519v101", hi3519v101, hi3519v101_soc)
DEFINE_HISI_MACHINE("hi3516ev300", hi3516ev300, hi3516ev300_soc)
DEFINE_HISI_MACHINE("hi3516ev200", hi3516ev200, hi3516ev200_soc)
DEFINE_HISI_MACHINE("hi3518ev300", hi3518ev300, hi3518ev300_soc)
DEFINE_HISI_MACHINE("hi3516dv200", hi3516dv200, hi3516dv200_soc)
DEFINE_HISI_MACHINE("gk7205v200", gk7205v200, gk7205v200_soc)
DEFINE_HISI_MACHINE("gk7205v300", gk7205v300, gk7205v300_soc)
DEFINE_HISI_MACHINE("gk7202v300", gk7202v300, gk7202v300_soc)
DEFINE_HISI_MACHINE("gk7605v100", gk7605v100, gk7605v100_soc)
DEFINE_HISI_MACHINE("gk7205v500", gk7205v500, gk7205v500_soc)
DEFINE_HISI_MACHINE("gk7205v510", gk7205v510, gk7205v510_soc)
DEFINE_HISI_MACHINE("gk7205v530", gk7205v530, gk7205v530_soc)
DEFINE_HISI_MACHINE("gk7202v330", gk7202v330, gk7202v330_soc)
DEFINE_HISI_MACHINE("hi3516cv608", hi3516cv608, hi3516cv608_soc)
DEFINE_HISI_MACHINE("hi3516cv610", hi3516cv610, hi3516cv610_soc)
DEFINE_HISI_MACHINE("hi3516cv613", hi3516cv613, hi3516cv613_soc)
