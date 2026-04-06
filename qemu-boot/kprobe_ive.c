/*
 * kprobe_ive — Hijack vendor's task chain to test Conv parameters
 *
 * Hooks drv_ive_write_regs to modify the Conv task node before HW submit.
 * Keeps the vendor module loaded (no rmmod), so all HW state is intact.
 *
 * Module param "test_weight_addr" — if set, overrides Conv node[24] (weight address).
 * This lets us test different weight data while using the vendor's infrastructure.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <asm/io.h>

static unsigned int test_weight_addr;
module_param(test_weight_addr, uint, S_IRUGO | S_IWUSR);

static int submit_count;

static int write_pre(struct kprobe *p, struct pt_regs *regs)
{
	u32 phys = regs->ARM_r0;
	u8 *virt;

	if (phys < 0x40000000 || phys > 0x50000000) return 0;

	virt = phys_to_virt(phys);
	if (!virt) return 0;

	/* Check if this is a Conv node (marker=0x36, type=0) */
	if (virt[10] == 0x36 && virt[11] == 0) {
		pr_info("kp_ive: Conv node submit #%d, weight=0x%08x\n",
			submit_count, *(u32 *)(virt + 24));

		if (test_weight_addr) {
			pr_info("kp_ive: OVERRIDE weight 0x%08x -> 0x%08x\n",
				*(u32 *)(virt + 24), test_weight_addr);
			*(u32 *)(virt + 24) = test_weight_addr;
		}
	}

	submit_count++;
	return 0;
}

static struct kprobe kp = { .symbol_name = "drv_ive_write_regs" };

static int __init kprobe_ive_init(void)
{
	int ret;
	kp.pre_handler = write_pre;
	ret = register_kprobe(&kp);
	if (ret < 0) {
		pr_err("kp_ive: failed: %d\n", ret);
		return ret;
	}
	submit_count = 0;
	pr_info("kp_ive: hijack mode, test_weight_addr=0x%x\n", test_weight_addr);
	return 0;
}

static void __exit kprobe_ive_exit(void)
{
	unregister_kprobe(&kp);
	pr_info("kp_ive: removed after %d submits\n", submit_count);
}

module_init(kprobe_ive_init);
module_exit(kprobe_ive_exit);
MODULE_LICENSE("GPL");
