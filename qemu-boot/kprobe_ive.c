/*
 * kprobe_ive — Check if loadmodel writes transformed weights to s[8] area
 *
 * At drv_ive_write_regs time, read from model memory at arg_off and s[8]
 * to see if the loadmodel wrote transformed weights.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <asm/io.h>

static int write_pre(struct kprobe *p, struct pt_regs *regs)
{
	u32 phys = regs->ARM_r0;
	u8 *virt;
	u32 node24, node28, node12;

	if (phys < 0x40000000 || phys > 0x50000000) return 0;

	virt = phys_to_virt(phys);
	if (!virt || virt[10] != 0x36) return 0;

	/* Only process Conv nodes (type=0) */
	if (virt[11] != 0) return 0;

	node24 = *(u32 *)(virt + 24);  /* weight/arg phys addr */
	node28 = *(u32 *)(virt + 28);  /* arg_len */
	node12 = *(u32 *)(virt + 12);  /* s[8] runtime value */

	pr_info("kp_ive: Conv node24(wt)=0x%x node28(len)=%d node12(s8)=0x%x\n",
		node24, node28, node12);

	/* Dump arg data at node24 (biases+pad+weights) */
	if (node24 >= 0x42000000) {
		u8 *arg = phys_to_virt(node24);
		if (arg) {
			pr_info("kp_ive: arg[0..15] (biases): %*ph\n", 16, arg);
			pr_info("kp_ive: arg[32..47] (pad):    %*ph\n", 16, arg+32);
			pr_info("kp_ive: arg[64..79] (weights): %*ph\n", 16, arg+64);
		}
	}

	/* Now check what's at model_phys + s[8] from DESCRIPTOR (not node[12] which is 0)
	 * The descriptor s[8] was patched to hw_weight_off.
	 * model_phys = node24 - arg_off. We need to compute this.
	 * But we don't know arg_off at this point. Instead, scan backwards
	 * from node24 to find the segment start (CRC + size pattern). */

	/* Try: the model segment data starts at a page boundary before node24.
	 * Or: dump memory at several offsets from node24 to find s[8] data. */
	{
		u8 *model_base;
		u32 seg_size;
		int off;

		/* Model data typically starts at a page-aligned address.
		 * node24 = model_phys + arg_off. arg_off is small (< 0x1000).
		 * So model_phys ≈ node24 & ~0xFFF. */
		model_base = phys_to_virt(node24 & ~0xFFF);
		if (!model_base) return 0;

		/* Read segment size to find actual base */
		seg_size = *(u32 *)(model_base + 4);
		pr_info("kp_ive: model_base guess=%p seg_size=%d\n",
			model_base, seg_size);

		/* Dump at offsets 0x3C0 (our hw_weight_off for small models) */
		for (off = 0x160; off <= 0x400; off += 0x40) {
			u8 *p = model_base + off;
			int nz = 0, j;
			for (j = 0; j < 16; j++) if (p[j]) nz++;
			if (nz)
				pr_info("kp_ive: model+%03x: %*ph\n", off, 16, p);
		}
	}

	return 0;
}

static struct kprobe kp = { .symbol_name = "drv_ive_write_regs" };

static int __init kprobe_ive_init(void)
{
	kp.pre_handler = write_pre;
	return register_kprobe(&kp) < 0 ? -1 :
		(pr_info("kp_ive: ready\n"), 0);
}

static void __exit kprobe_ive_exit(void) { unregister_kprobe(&kp); }
module_init(kprobe_ive_init);
module_exit(kprobe_ive_exit);
MODULE_LICENSE("GPL");
