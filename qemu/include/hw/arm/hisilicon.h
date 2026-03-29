/*
 * HiSilicon IP camera SoC family definitions.
 *
 * Table-driven configuration for all supported SoC variants.
 * Add new SoCs by defining a HisiSoCConfig instance — the shared
 * hisilicon_common_init() handles the rest.
 *
 * Copyright (c) 2020-2021, 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_HISILICON_H
#define HW_ARM_HISILICON_H

#include "qemu/units.h"
#include "exec/hwaddr.h"

/* Maximum peripheral counts */
#define HISI_MAX_UARTS    5
#define HISI_MAX_TIMERS   2
#define HISI_MAX_SPIS     4
#define HISI_MAX_HIMCI    3
#define HISI_MAX_SDHCI    2
#define HISI_MAX_I2C      8
#define HISI_MAX_REGBANKS 16
#define HISI_MAX_CRG_DEFAULTS 8

typedef struct HisiRegbankEntry {
    const char *name;
    hwaddr      base;     /* 0 = skip */
    uint32_t    size;
} HisiRegbankEntry;

typedef struct HisiSoCConfig {
    const char     *name;
    const char     *desc;
    const char     *cpu_type;       /* full QOM type, set at runtime */
    uint32_t        soc_id;
    ram_addr_t      ram_size_default;

    /* Memory regions */
    hwaddr          ram_base;
    hwaddr          sram_base;
    size_t          sram_size;

    /* Interrupt controller — VIC or GIC, selected by use_gic */
    bool            use_gic;
    hwaddr          vic_base;       /* PL190, when !use_gic */
    hwaddr          gic_dist_base;  /* GICv2, when use_gic */
    hwaddr          gic_cpu_base;
    int             gic_num_spi;

    /* System controller + CRG */
    hwaddr          sysctl_base;
    hwaddr          crg_base;

    /* UARTs (PL011) */
    int             num_uarts;
    hwaddr          uart_bases[HISI_MAX_UARTS];
    int             uart_irqs[HISI_MAX_UARTS];

    /* Timers (SP804) */
    int             num_timers;
    hwaddr          timer_bases[HISI_MAX_TIMERS];
    int             timer_irqs[HISI_MAX_TIMERS];
    uint32_t        timer_freq;     /* 0 = device default */

    /* SPI (PL022) */
    int             num_spis;
    hwaddr          spi_bases[HISI_MAX_SPIS];
    int             spi_irqs[HISI_MAX_SPIS];

    /* Flash Memory Controller (HiFMC or HISFC350) */
    hwaddr          fmc_ctrl_base;  /* 0 = no flash controller */
    hwaddr          fmc_mem_base;
    const char     *fmc_type;       /* NULL = "hisi-fmc", or "hisi-sfc350" */

    /* GPIO (PL061) */
    hwaddr          gpio_base;
    int             gpio_count;
    int             gpio_stride;    /* address step between ports (0x1000 or 0x10000) */
    int             gpio_irq;       /* VIC: shared IRQ for all ports */
    int             gpio_irq_start; /* GIC: first SPI, one per port */

    /* DMA (PL080) */
    hwaddr          dma_base;       /* 0 = no DMA controller */
    int             dma_irq;

    /* FEMAC (Fast Ethernet MAC) */
    hwaddr          femac_base;     /* 0 = no FEMAC */
    int             femac_irq;

    /* SD/MMC — himciv200 (older SoCs) */
    int             num_himci;
    hwaddr          himci_bases[HISI_MAX_HIMCI];
    int             himci_irqs[HISI_MAX_HIMCI];

    /* SD/MMC — SDHCI (newer SoCs) */
    int             num_sdhci;
    hwaddr          sdhci_bases[HISI_MAX_SDHCI];
    int             sdhci_irqs[HISI_MAX_SDHCI];

    /* I2C (HiBVT) */
    int             num_i2c;
    hwaddr          i2c_bases[HISI_MAX_I2C];

    /* MIPI RX (CSI-2 / LVDS receiver) */
    hwaddr          mipi_rx_base;   /* 0 = no MIPI RX */
    int             mipi_rx_irq;

    /* RTC (SPI-bridge internal RTC) */
    hwaddr          rtc_base;       /* 0 = no RTC */
    int             rtc_irq;

    /* VEDU (Video Encoder) + JPGE */
    hwaddr          vedu_base;      /* 0 = no VEDU */
    hwaddr          jpge_base;
    int             vedu_irq;
    int             jpge_irq;

    /* Watchdog (SP805 / cmsdk-apb-watchdog) */
    hwaddr          wdt_base;       /* 0 = no watchdog */
    int             wdt_irq;        /* -1 = no IRQ */
    uint32_t        wdt_freq;       /* clock frequency in Hz */

    /* CRG register defaults (mimics U-Boot clock init before kernel boot) */
    int             num_crg_defaults;
    struct { uint32_t offset; uint32_t value; } crg_defaults[HISI_MAX_CRG_DEFAULTS];

    /* Generic RAM-backed register banks (pin mux, DDR PHY, PWM, etc.) */
    int             num_regbanks;
    HisiRegbankEntry regbanks[HISI_MAX_REGBANKS];
} HisiSoCConfig;

/*
 * SoC identification values — written to SCSYSID register at
 * sysctl_base + 0xEE0.  Software (U-Boot, kernel, ipctool)
 * reads this to auto-detect the chip model.
 */
#define HISI_SOC_ID_CV300       0x3516C300
#define HISI_SOC_ID_EV200       0x3516E200
#define HISI_SOC_ID_EV300       0x3516E300
#define HISI_SOC_ID_18EV300     0x3518E300
#define HISI_SOC_ID_DV200       0x3516D200

/* V1 generation */
#define HISI_SOC_ID_CV100       0x35180100

/* V2 generation */
#define HISI_SOC_ID_CV200       0x3518E200

/* V2A generation (Cortex-A7 + GIC, V1-era 0x20xxxxxx address map) */
#define HISI_SOC_ID_AV100       0x3516A100

/* V3.5 generation (Cortex-A7 + GIC, but V3-era address map) */
#define HISI_SOC_ID_CV500       0x3516C500

/* V3A generation (Cortex-A7/A17 big.LITTLE + GIC, V3-era address map) */
#define HISI_SOC_ID_19V101      0x35190101

/*
 * V5 generation (Cortex-A7 MP2 + GIC, new 0x11xxxxxx address map, ~2023)
 * Same die, different feature tiers.  Model suffixes:
 *   10B=CV610, 20S=CV613, 00S/20G/00G=unknown IDs (not yet in lab)
 * Also HISI_OT: Hi3516DV500 (0x3516D500), Hi3519DV500 (0x3519D500)
 */
#define HISI_SOC_ID_CV608       0x3516C608  /* consumer, 0.2 TOPS, 3M */
#define HISI_SOC_ID_CV610       0x3516C610  /* 10B: 0.5 TOPS, 5M */
#define HISI_SOC_ID_CV613       0x3516C613  /* 20S: 1 TOPS, 4K */

/* Goke chips — die-identical V4 silicon with different SoC IDs */
#define GOKE_SOC_ID_7205V200    0x72050200  /* = hi3516ev200 */
#define GOKE_SOC_ID_7205V300    0x72050300  /* = hi3516ev300 */
#define GOKE_SOC_ID_7202V300    0x72020300  /* = hi3518ev300 */
#define GOKE_SOC_ID_7605V100    0x76050100  /* = hi3516dv200 */

#endif /* HW_ARM_HISILICON_H */
