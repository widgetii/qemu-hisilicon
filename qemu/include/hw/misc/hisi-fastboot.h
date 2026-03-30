/*
 * HiSilicon Boot ROM fastboot serial protocol — public interface.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_HISI_FASTBOOT_H
#define HW_MISC_HISI_FASTBOOT_H

#define TYPE_HISI_FASTBOOT "hisi-fastboot"

/*
 * Configure the fastboot device after qdev_new() but before realize.
 */
void hisi_fastboot_setup(DeviceState *dev, CPUState *cpu,
                         Chardev *chardev, hwaddr uart0_base,
                         qemu_irq uart0_irq);

#endif /* HW_MISC_HISI_FASTBOOT_H */
