/*
 * HiSilicon Hi3516CV300 / Hi3516EV300 SoC emulation.
 *
 * CV300: ARM926EJ-S + PL190 VIC
 * EV300: Cortex-A7 + GICv2
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
#include "net/net.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/gtimer.h"

static struct arm_boot_info hisilicon_binfo;

/* ── Hi3516CV300 (ARM926EJ-S + PL190 VIC) ───────────────────────────── */

static void hi3516cv300_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    ARMCPU *cpu;
    DeviceState *vic, *sysctl, *crg;
    qemu_irq pic[32];
    int n;

    /* Trap page at address 0 (see EV300 comment) */
    {
        MemoryRegion *trap = g_new(MemoryRegion, 1);
        memory_region_init_rom(trap, NULL, "hisilicon.trapnull",
                               0x1000, &error_fatal);
        memory_region_add_subregion(sysmem, 0, trap);
        uint32_t insn[2] = { cpu_to_le32(0xe3a00000),
                             cpu_to_le32(0xe12fff1e) };
        address_space_write_rom(&address_space_memory, 0,
                                MEMTXATTRS_UNSPECIFIED, insn, 8);
    }

    /* SRAM (used by U-Boot SPL for early stack / global data) */
    {
        MemoryRegion *sram = g_new(MemoryRegion, 1);
        memory_region_init_ram(sram, NULL, "hisilicon.sram",
                               CV300_SRAM_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, CV300_SRAM_BASE, sram);
    }

    /* FMC (Flash Memory Controller) */
    {
        DeviceState *fmc = qdev_new("hisi-fmc");
        SysBusDevice *fmcbus = SYS_BUS_DEVICE(fmc);
        sysbus_realize_and_unref(fmcbus, &error_fatal);
        sysbus_mmio_map(fmcbus, 0, CV300_FMC_CTRL_BASE);
        sysbus_mmio_map(fmcbus, 1, CV300_FMC_MEM_BASE);
    }

    /* RAM */
    memory_region_add_subregion(sysmem, CV300_RAM_BASE, machine->ram);

    /* CPU */
    cpuobj = object_new(machine->cpu_type);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    /* PL190 VIC */
    vic = sysbus_create_varargs("pl190", CV300_VIC_BASE,
            qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ),
            qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ),
            NULL);

    for (n = 0; n < 32; n++) {
        pic[n] = qdev_get_gpio_in(vic, n);
    }

    /* SysCtrl */
    sysctl = qdev_new("hisi-sysctl");
    qdev_prop_set_uint32(sysctl, "soc-id", HISI_SOC_ID_CV300);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(sysctl), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(sysctl), 0, CV300_SYSCTL_BASE);

    /* CRG */
    crg = qdev_new("hisi-crg");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(crg), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(crg), 0, CV300_CRG_BASE);

    /* UARTs */
    pl011_create(CV300_UART0_BASE, pic[CV300_UART0_IRQ], serial_hd(0));
    pl011_create(CV300_UART1_BASE, pic[CV300_UART1_IRQ], serial_hd(1));
    pl011_create(CV300_UART2_BASE, pic[CV300_UART2_IRQ], serial_hd(2));

    /* Timers (SP804 @ 24 MHz bus clock — kernel reads from DT/CRG) */
    {
        DeviceState *t0 = qdev_new("sp804");
        DeviceState *t1 = qdev_new("sp804");
        qdev_prop_set_uint32(t0, "freq0", CV300_TIMER_FREQ);
        qdev_prop_set_uint32(t0, "freq1", CV300_TIMER_FREQ);
        qdev_prop_set_uint32(t1, "freq0", CV300_TIMER_FREQ);
        qdev_prop_set_uint32(t1, "freq1", CV300_TIMER_FREQ);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(t0), &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(t1), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(t0), 0, CV300_TIMER01_BASE);
        sysbus_mmio_map(SYS_BUS_DEVICE(t1), 0, CV300_TIMER23_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(t0), 0, pic[CV300_TIMER01_IRQ]);
        sysbus_connect_irq(SYS_BUS_DEVICE(t1), 0, pic[CV300_TIMER23_IRQ]);
    }

    /* SPI */
    sysbus_create_simple("pl022", CV300_SPI0_BASE, pic[CV300_SPI0_IRQ]);
    sysbus_create_simple("pl022", CV300_SPI1_BASE, pic[CV300_SPI1_IRQ]);

    /* DMA */
    {
        DeviceState *dma = qdev_new("pl080");
        object_property_set_link(OBJECT(dma), "downstream", OBJECT(sysmem),
                                 &error_fatal);
        SysBusDevice *busdev = SYS_BUS_DEVICE(dma);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, CV300_DMA_BASE);
        sysbus_connect_irq(busdev, 0, pic[CV300_DMA_IRQ]);
    }

    /* GPIOs */
    for (n = 0; n < CV300_GPIO_COUNT; n++) {
        sysbus_create_simple("pl061", CV300_GPIO_BASE + n * 0x1000,
                             pic[CV300_GPIO_IRQ]);
    }

    /* FEMAC */
    {
        DeviceState *femac = qdev_new("hisi-femac");
        qemu_configure_nic_device(femac, true, NULL);
        SysBusDevice *busdev = SYS_BUS_DEVICE(femac);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, CV300_FEMAC_BASE);
        sysbus_connect_irq(busdev, 0, pic[CV300_FEMAC_IRQ]);
    }

    /* SD/MMC (himciv200) */
    {
        static const hwaddr mmc_bases[] = {
            CV300_MMC0_BASE, CV300_MMC1_BASE, CV300_MMC2_BASE,
        };
        for (int i = 0; i < ARRAY_SIZE(mmc_bases); i++) {
            DeviceState *mmc = qdev_new("hisi-himci");
            SysBusDevice *busdev = SYS_BUS_DEVICE(mmc);
            sysbus_realize_and_unref(busdev, &error_fatal);
            sysbus_mmio_map(busdev, 0, mmc_bases[i]);
        }
    }

    /* Boot */
    hisilicon_binfo.ram_size = machine->ram_size;
    hisilicon_binfo.loader_start = CV300_RAM_BASE;
    arm_load_kernel(cpu, machine, &hisilicon_binfo);
}

static void hi3516cv300_class_init(MachineClass *mc)
{
    mc->desc = "HiSilicon Hi3516CV300 (ARM926EJ-S)";
    mc->init = hi3516cv300_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_size = CV300_RAM_SIZE_DEFAULT;
    mc->default_ram_id = "hisilicon.ram";
    mc->block_default_type = IF_MTD;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("hi3516cv300", hi3516cv300_class_init)

/* ── Hi3516EV300 (Cortex-A7 + GICv2) ────────────────────────────────── */

static void hi3516ev300_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    ARMCPU *cpu;
    DeviceState *gicdev, *sysctl, *crg;
    SysBusDevice *gicbus;
    qemu_irq pic[EV300_GIC_NUM_SPI];
    int n;

    /* Trap page at address 0 — U-Boot driver model may call through NULL
     * function pointers for absent hardware.  Place "mov r0, #0; bx lr"
     * so those calls harmlessly return 0 instead of executing garbage. */
    {
        MemoryRegion *trap = g_new(MemoryRegion, 1);
        memory_region_init_rom(trap, NULL, "hisilicon.trapnull",
                               0x1000, &error_fatal);
        memory_region_add_subregion(sysmem, 0, trap);
        /* ARM: mov r0, #0 (0xe3a00000); bx lr (0xe12fff1e) */
        uint32_t insn[2] = { cpu_to_le32(0xe3a00000),
                             cpu_to_le32(0xe12fff1e) };
        address_space_write_rom(&address_space_memory, 0,
                                MEMTXATTRS_UNSPECIFIED, insn, 8);
    }

    /* SRAM (used by U-Boot SPL for early stack / global data) */
    {
        MemoryRegion *sram = g_new(MemoryRegion, 1);
        memory_region_init_ram(sram, NULL, "hisilicon.sram",
                               EV300_SRAM_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, EV300_SRAM_BASE, sram);
    }

    /* FMC (Flash Memory Controller) */
    {
        DeviceState *fmc = qdev_new("hisi-fmc");
        SysBusDevice *fmcbus = SYS_BUS_DEVICE(fmc);
        sysbus_realize_and_unref(fmcbus, &error_fatal);
        sysbus_mmio_map(fmcbus, 0, EV300_FMC_CTRL_BASE);
        sysbus_mmio_map(fmcbus, 1, EV300_FMC_MEM_BASE);
    }

    /* RAM */
    memory_region_add_subregion(sysmem, EV300_RAM_BASE, machine->ram);

    /* CPU */
    cpuobj = object_new(machine->cpu_type);
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    /* GICv2 */
    gicdev = qdev_new(gic_class_name());
    qdev_prop_set_uint32(gicdev, "revision", 2);
    qdev_prop_set_uint32(gicdev, "num-cpu", 1);
    qdev_prop_set_uint32(gicdev, "num-irq", EV300_GIC_NUM_IRQ);
    qdev_prop_set_bit(gicdev, "has-security-extensions", false);
    gicbus = SYS_BUS_DEVICE(gicdev);
    sysbus_realize_and_unref(gicbus, &error_fatal);
    sysbus_mmio_map(gicbus, 0, EV300_GIC_DIST_BASE);
    sysbus_mmio_map(gicbus, 1, EV300_GIC_CPU_BASE);

    /* Wire GIC outputs → CPU inputs */
    sysbus_connect_irq(gicbus, 0,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(gicbus, 1,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));
    sysbus_connect_irq(gicbus, 2,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_VIRQ));
    sysbus_connect_irq(gicbus, 3,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_VFIQ));

    /* Wire CPU timer PPIs → GIC */
    {
        DeviceState *cpudev = DEVICE(cpu);
        int ppibase = EV300_GIC_NUM_SPI + GIC_NR_SGIS; /* 128 + 16 = 144 */
        const int timer_irq[] = {
            [GTIMER_PHYS] = EV300_PPI_PHYSTIMER,
            [GTIMER_VIRT] = EV300_PPI_VIRTTIMER,
            [GTIMER_HYP]  = EV300_PPI_HYPTIMER,
            [GTIMER_SEC]  = EV300_PPI_SECTIMER,
        };

        for (int i = 0; i < ARRAY_SIZE(timer_irq); i++) {
            qdev_connect_gpio_out(cpudev, i,
                                  qdev_get_gpio_in(gicdev,
                                                   ppibase + timer_irq[i]));
        }

        /* PMU */
        qdev_connect_gpio_out_named(cpudev, "pmu-interrupt", 0,
                                    qdev_get_gpio_in(gicdev, ppibase + 7));
    }

    /* Build SPI IRQ array */
    for (n = 0; n < EV300_GIC_NUM_SPI; n++) {
        pic[n] = qdev_get_gpio_in(gicdev, n);
    }

    /* SysCtrl */
    sysctl = qdev_new("hisi-sysctl");
    qdev_prop_set_uint32(sysctl, "soc-id", HISI_SOC_ID_EV300);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(sysctl), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(sysctl), 0, EV300_SYSCTL_BASE);

    /* CRG */
    crg = qdev_new("hisi-crg");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(crg), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(crg), 0, EV300_CRG_BASE);

    /* UARTs */
    pl011_create(EV300_UART0_BASE, pic[EV300_UART0_IRQ], serial_hd(0));
    pl011_create(EV300_UART1_BASE, pic[EV300_UART1_IRQ], serial_hd(1));
    pl011_create(EV300_UART2_BASE, pic[EV300_UART2_IRQ], serial_hd(2));

    /* Timers */
    sysbus_create_simple("sp804", EV300_TIMER01_BASE, pic[EV300_TIMER01_IRQ]);
    sysbus_create_simple("sp804", EV300_TIMER23_BASE, pic[EV300_TIMER23_IRQ]);

    /* SPI */
    sysbus_create_simple("pl022", EV300_SPI0_BASE, pic[EV300_SPI0_IRQ]);
    sysbus_create_simple("pl022", EV300_SPI1_BASE, pic[EV300_SPI1_IRQ]);

    /* GPIOs */
    for (n = 0; n < EV300_GPIO_COUNT; n++) {
        sysbus_create_simple("pl061", EV300_GPIO_BASE + n * 0x1000,
                             pic[EV300_GPIO_IRQ_START + n]);
    }

    /* FEMAC */
    {
        DeviceState *femac = qdev_new("hisi-femac");
        qemu_configure_nic_device(femac, true, NULL);
        SysBusDevice *busdev = SYS_BUS_DEVICE(femac);
        sysbus_realize_and_unref(busdev, &error_fatal);
        sysbus_mmio_map(busdev, 0, EV300_FEMAC_BASE);
        sysbus_connect_irq(busdev, 0, pic[EV300_FEMAC_IRQ]);
    }

    /* SDHCI */
    {
        static const hwaddr sdhci_bases[] = {
            EV300_SDHCI0_BASE, EV300_SDHCI1_BASE,
        };
        for (int i = 0; i < ARRAY_SIZE(sdhci_bases); i++) {
            DeviceState *sdhci = qdev_new(TYPE_SYSBUS_SDHCI);
            qdev_prop_set_uint8(sdhci, "sd-spec-version", 3);
            SysBusDevice *busdev = SYS_BUS_DEVICE(sdhci);
            sysbus_realize_and_unref(busdev, &error_fatal);
            sysbus_mmio_map(busdev, 0, sdhci_bases[i]);
        }
    }

    /* Boot */
    hisilicon_binfo.ram_size = machine->ram_size;
    hisilicon_binfo.loader_start = EV300_RAM_BASE;
    arm_load_kernel(cpu, machine, &hisilicon_binfo);
}

static void hi3516ev300_class_init(MachineClass *mc)
{
    mc->desc = "HiSilicon Hi3516EV300 (Cortex-A7)";
    mc->init = hi3516ev300_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a7");
    mc->default_ram_size = EV300_RAM_SIZE_DEFAULT;
    mc->default_ram_id = "hisilicon.ram";
    mc->block_default_type = IF_MTD;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE_ARM("hi3516ev300", hi3516ev300_class_init)
