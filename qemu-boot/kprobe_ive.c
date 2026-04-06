/*
 * kprobe_ive — Capture vendor's HW register writes + task nodes
 *
 * Probes drv_ive_write_regs (task submit) AND drv_ive_read_regs (status read).
 * Also probes writel to capture ALL register writes to IVE space (0x11320000).
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <asm/io.h>

static int call_count;
static void __iomem *ive_base;

/* Capture task node submission */
static int write_pre(struct kprobe *p, struct pt_regs *regs)
{
	u32 phys = regs->ARM_r0;
	u8 *virt;
	int n, i;

	if (phys < 0x40000000 || phys > 0x50000000)
		return 0;

	pr_info("kp_ive: SUBMIT phys=0x%08x\n", phys);

	for (n = 0; n < 16; n++) {
		virt = phys_to_virt(phys);
		if (!virt) break;
		pr_info("kp_ive: chain[%d] phys=0x%08x type=%d\n",
			n, phys, virt[11]);
		for (i = 0; i < 128; i += 16)
			pr_info("kp_ive:   +%02x: %*ph\n", i, 16, virt + i);
		phys = *(u32 *)virt;
		if (!phys || phys < 0x40000000) break;
	}

	/* Dump IVE regs snapshot before submit */
	if (ive_base) {
		pr_info("kp_ive: REGS before submit:\n");
		for (i = 0; i < 0x90; i += 4)
			pr_info("kp_ive:   reg[0x%02x]=0x%08x\n",
				i, readl(ive_base + i));
	}

	call_count++;
	return 0;
}

static struct kprobe kp_write = {
	.symbol_name = "drv_ive_write_regs",
};

static int __init kprobe_ive_init(void)
{
	int ret;

	ive_base = ioremap(0x11320000, 0x100);

	kp_write.pre_handler = write_pre;
	ret = register_kprobe(&kp_write);
	if (ret < 0) {
		pr_err("kp_ive: register_kprobe failed: %d\n", ret);
		if (ive_base) iounmap(ive_base);
		return ret;
	}

	call_count = 0;
	pr_info("kp_ive: probing at %pS, ive_base=%p\n",
		kp_write.addr, ive_base);
	return 0;
}

static void __exit kprobe_ive_exit(void)
{
	unregister_kprobe(&kp_write);
	if (ive_base) iounmap(ive_base);
	pr_info("kp_ive: removed, %d calls\n", call_count);
}

module_init(kprobe_ive_init);
module_exit(kprobe_ive_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("kprobe to capture IVE register state during Conv dispatch");
