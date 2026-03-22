/*
 * HiSilicon IP camera SoC emulation.
 *
 * Table-driven: each SoC variant is a HisiSoCConfig struct.
 * One shared init function handles VIC/GIC, peripherals, boot.
 *
 * Currently supported:
 *   hi3516cv300  (V3, ARM926EJ-S + PL190 VIC)
 *   hi3516ev300  (V4, Cortex-A7 + GICv2)
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
#include "target/arm/cpu-qom.h"
#include "target/arm/gtimer.h"

/* ── SoC configuration tables ──────────────────────────────────────── */

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
    .gpio_irq           = 31,           /* shared for all ports (VIC) */

    .dma_base           = 0x10030000,
    .dma_irq            = 14,

    .femac_base         = 0x10050000,
    .femac_irq          = 12,

    .num_himci          = 3,
    .himci_bases        = { 0x100c0000, 0x100d0000, 0x100e0000 },
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
    .gpio_irq_start     = 16,           /* per-port: SPI 16..25 (GIC) */

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },
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
    .gpio_irq_start     = 16,

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },
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
    .gpio_irq_start     = 16,

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },
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
    .gpio_irq_start     = 16,

    .femac_base         = 0x10040000,
    .femac_irq          = 33,

    .num_sdhci          = 2,
    .sdhci_bases        = { 0x10010000, 0x10020000 },
    .sdhci_irqs         = { 30, 31 },
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
    .gpio_irq_start     = 16,                              \
    .femac_base         = 0x10040000,                       \
    .femac_irq          = 33,                              \
    .num_sdhci          = 2,                                \
    .sdhci_bases        = { 0x10010000, 0x10020000 },       \
    .sdhci_irqs         = { 30, 31 }

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
        sysbus_create_simple("pl061", c->gpio_base + n * 0x1000, irq);
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

    /* Boot */
    hisilicon_binfo.ram_size = machine->ram_size;
    hisilicon_binfo.loader_start = c->ram_base;
    arm_load_kernel(cpu, machine, &hisilicon_binfo);
}

/* ── Per-machine wrappers ──────────────────────────────────────────── */

static void hi3516cv300_init(MachineState *machine)
{
    hisilicon_common_init(machine, &hi3516cv300_soc);
}

static void hi3516cv300_class_init(MachineClass *mc)
{
    mc->desc = hi3516cv300_soc.desc;
    mc->init = hi3516cv300_init;
    mc->default_cpu_type = hi3516cv300_soc.cpu_type;
    mc->default_ram_size = hi3516cv300_soc.ram_size_default;
    mc->default_ram_id = "hisilicon.ram";
    mc->block_default_type = IF_MTD;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("hi3516cv300", hi3516cv300_class_init)

static void hi3516ev300_init(MachineState *machine)
{
    hisilicon_common_init(machine, &hi3516ev300_soc);
}

static void hi3516ev300_class_init(MachineClass *mc)
{
    mc->desc = hi3516ev300_soc.desc;
    mc->init = hi3516ev300_init;
    mc->default_cpu_type = hi3516ev300_soc.cpu_type;
    mc->default_ram_size = hi3516ev300_soc.ram_size_default;
    mc->default_ram_id = "hisilicon.ram";
    mc->block_default_type = IF_MTD;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("hi3516ev300", hi3516ev300_class_init)

static void hi3516ev200_init(MachineState *machine)
{
    hisilicon_common_init(machine, &hi3516ev200_soc);
}

static void hi3516ev200_class_init(MachineClass *mc)
{
    mc->desc = hi3516ev200_soc.desc;
    mc->init = hi3516ev200_init;
    mc->default_cpu_type = hi3516ev200_soc.cpu_type;
    mc->default_ram_size = hi3516ev200_soc.ram_size_default;
    mc->default_ram_id = "hisilicon.ram";
    mc->block_default_type = IF_MTD;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("hi3516ev200", hi3516ev200_class_init)

static void hi3518ev300_init(MachineState *machine)
{
    hisilicon_common_init(machine, &hi3518ev300_soc);
}

static void hi3518ev300_class_init(MachineClass *mc)
{
    mc->desc = hi3518ev300_soc.desc;
    mc->init = hi3518ev300_init;
    mc->default_cpu_type = hi3518ev300_soc.cpu_type;
    mc->default_ram_size = hi3518ev300_soc.ram_size_default;
    mc->default_ram_id = "hisilicon.ram";
    mc->block_default_type = IF_MTD;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("hi3518ev300", hi3518ev300_class_init)

static void hi3516dv200_init(MachineState *machine)
{
    hisilicon_common_init(machine, &hi3516dv200_soc);
}

static void hi3516dv200_class_init(MachineClass *mc)
{
    mc->desc = hi3516dv200_soc.desc;
    mc->init = hi3516dv200_init;
    mc->default_cpu_type = hi3516dv200_soc.cpu_type;
    mc->default_ram_size = hi3516dv200_soc.ram_size_default;
    mc->default_ram_id = "hisilicon.ram";
    mc->block_default_type = IF_MTD;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("hi3516dv200", hi3516dv200_class_init)

/* Goke machines — identical hardware, different SoC ID branding */

#define DEFINE_HISI_MACHINE(namestr, tag, config)                    \
    static void tag##_init(MachineState *machine)                    \
    {                                                                \
        hisilicon_common_init(machine, &config);                     \
    }                                                                \
    static void tag##_class_init(MachineClass *mc)                   \
    {                                                                \
        mc->desc = config.desc;                                      \
        mc->init = tag##_init;                                       \
        mc->default_cpu_type = config.cpu_type;                      \
        mc->default_ram_size = config.ram_size_default;              \
        mc->default_ram_id = "hisilicon.ram";                        \
        mc->block_default_type = IF_MTD;                             \
        mc->ignore_memory_transaction_failures = true;               \
    }                                                                \
    DEFINE_MACHINE_ARM(namestr, tag##_class_init)

DEFINE_HISI_MACHINE("gk7205v200", gk7205v200, gk7205v200_soc)
DEFINE_HISI_MACHINE("gk7205v300", gk7205v300, gk7205v300_soc)
DEFINE_HISI_MACHINE("gk7202v300", gk7202v300, gk7202v300_soc)
DEFINE_HISI_MACHINE("gk7605v100", gk7605v100, gk7605v100_soc)
