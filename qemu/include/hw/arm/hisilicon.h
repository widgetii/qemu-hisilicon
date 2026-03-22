/*
 * HiSilicon CV300/EV300 SoC memory map constants.
 *
 * Copyright (c) 2020-2021, 2026 OpenIPC.
 * Written by Dmitry Ilyin
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_HISILICON_H
#define HW_ARM_HISILICON_H

/* SoC identification */
#define HISI_SOC_ID_CV300       0x3516C300
#define HISI_SOC_ID_EV300       0x3516E300

/* ── CV300 (ARM926EJ-S, ARMv5) ─────────────────────────────────────── */

#define CV300_SRAM_BASE         0x04010000
#define CV300_SRAM_SIZE         (64 * KiB)

#define CV300_RAM_BASE          0x80000000
#define CV300_RAM_SIZE_DEFAULT  (64 * MiB)

/* Interrupt controller: PL190 VIC */
#define CV300_VIC_BASE          0x10040000

/* UARTs (PL011) */
#define CV300_UART0_BASE        0x12100000
#define CV300_UART0_IRQ         5
#define CV300_UART1_BASE        0x12101000
#define CV300_UART1_IRQ         30
#define CV300_UART2_BASE        0x12102000
#define CV300_UART2_IRQ         25

/* Timers (SP804) */
#define CV300_TIMER01_BASE      0x12000000
#define CV300_TIMER01_IRQ       3
#define CV300_TIMER23_BASE      0x12001000
#define CV300_TIMER23_IRQ       4

/* Timer frequency (bus clock) */
#define CV300_TIMER_FREQ        24000000  /* 24 MHz */

/* System controller */
#define CV300_SYSCTL_BASE       0x12020000

/* Clock/Reset generator */
#define CV300_CRG_BASE          0x12010000

/* GPIO (PL061) — 9 ports, all share IRQ 31 */
#define CV300_GPIO_BASE         0x12140000
#define CV300_GPIO_COUNT        9
#define CV300_GPIO_IRQ          31

/* SPI (PL022) */
#define CV300_SPI0_BASE         0x12120000
#define CV300_SPI0_IRQ          6
#define CV300_SPI1_BASE         0x12121000
#define CV300_SPI1_IRQ          7

/* Flash Memory Controller */
#define CV300_FMC_CTRL_BASE     0x10000000
#define CV300_FMC_MEM_BASE      0x14000000

/* DMA (PL080) */
#define CV300_DMA_BASE          0x10030000
#define CV300_DMA_IRQ           14

/* ── EV300 (Cortex-A7, ARMv7-A) ───────────────────────────────────── */

#define EV300_SRAM_BASE         0x04010000
#define EV300_SRAM_SIZE         (64 * KiB)

#define EV300_RAM_BASE          0x40000000
#define EV300_RAM_SIZE_DEFAULT  (64 * MiB)

/* GICv2 */
#define EV300_GIC_DIST_BASE     0x10301000
#define EV300_GIC_CPU_BASE      0x10302000
#define EV300_GIC_NUM_SPI       128
#define EV300_GIC_NUM_IRQ       (EV300_GIC_NUM_SPI + GIC_INTERNAL) /* 160 */

/* ARM generic timer PPIs */
#define EV300_PPI_MAINT         9
#define EV300_PPI_HYPTIMER      10
#define EV300_PPI_VIRTTIMER     11
#define EV300_PPI_SECTIMER      13
#define EV300_PPI_PHYSTIMER     14

/* UARTs (PL011) — IRQs are GIC SPI numbers */
#define EV300_UART0_BASE        0x12040000
#define EV300_UART0_IRQ         7
#define EV300_UART1_BASE        0x12041000
#define EV300_UART1_IRQ         8
#define EV300_UART2_BASE        0x12042000
#define EV300_UART2_IRQ         9

/* Timers (SP804) */
#define EV300_TIMER01_BASE      0x12000000
#define EV300_TIMER01_IRQ       5
#define EV300_TIMER23_BASE      0x12001000
#define EV300_TIMER23_IRQ       6

/* System controller */
#define EV300_SYSCTL_BASE       0x12020000

/* Clock/Reset generator */
#define EV300_CRG_BASE          0x12010000

/* GPIO (PL061) — 10 ports, SPI 16-25 */
#define EV300_GPIO_BASE         0x120b0000
#define EV300_GPIO_COUNT        10
#define EV300_GPIO_IRQ_START    16

/* SPI (PL022) */
#define EV300_SPI0_BASE         0x12070000
#define EV300_SPI0_IRQ          14
#define EV300_SPI1_BASE         0x12071000
#define EV300_SPI1_IRQ          15

/* Flash Memory Controller */
#define EV300_FMC_CTRL_BASE     0x10000000
#define EV300_FMC_MEM_BASE      0x14000000

/* ARM generic timer frequency */
#define EV300_TIMER_FREQ        50000000  /* 50 MHz */

/* SD/MMC (himciv200 DW MMC) */
#define CV300_MMC0_BASE         0x100c0000  /* SD0 */
#define CV300_MMC1_BASE         0x100d0000  /* SD1 */
#define CV300_MMC2_BASE         0x100e0000  /* eMMC */

/* SDHCI */
#define EV300_SDHCI0_BASE       0x10010000  /* eMMC/SD0 */
#define EV300_SDHCI1_BASE       0x10020000  /* SDIO1 */

/* FEMAC (Fast Ethernet MAC) */
#define CV300_FEMAC_BASE        0x10050000
#define CV300_FEMAC_IRQ         12

#define EV300_FEMAC_BASE        0x10040000
#define EV300_FEMAC_IRQ         33

#endif /* HW_ARM_HISILICON_H */
