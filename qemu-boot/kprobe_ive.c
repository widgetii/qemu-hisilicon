/*
 * kprobe_ive — Dump full task chain for comparison
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <asm/io.h>

static int write_pre(struct kprobe *p, struct pt_regs *regs)
{
	u32 phys = regs->ARM_r0;
	u8 *virt;
	int n, i;

	if (phys < 0x40000000 || phys > 0x50000000) return 0;

	virt = phys_to_virt(phys);
	if (!virt) return 0;

	for (n = 0; n < 8; n++) {
		pr_info("kp_ive: NODE[%d] type=%d phys=0x%08x\n", n, virt[11], phys);
		for (i = 0; i < 128; i += 16)
			pr_info("kp_ive:  +%02x: %*ph\n", i, 16, virt + i);
		phys = *(u32 *)virt;
		if (!phys || phys < 0x42000000) break;
		virt = phys_to_virt(phys);
		if (!virt) break;
	}
	return 0;
}

static struct kprobe kp = { .symbol_name = "drv_ive_write_regs" };
static int __init kprobe_ive_init(void) {
	kp.pre_handler = write_pre;
	return register_kprobe(&kp) ?: (pr_info("kp_ive: ready\n"), 0);
}
static void __exit kprobe_ive_exit(void) { unregister_kprobe(&kp); }
module_init(kprobe_ive_init);
module_exit(kprobe_ive_exit);
MODULE_LICENSE("GPL");
