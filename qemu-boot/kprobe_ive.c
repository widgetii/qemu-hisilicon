/*
 * kprobe_ive — Dump model_ctx from ive_start_task
 *
 * Places kprobe at ive_start_task+0x68 (after r6 = model_ctx is computed)
 * to capture the per-model context fields that control Conv tiling.
 *
 * At offset 0x68: r6 = model_ctx_base + model_idx * 1080
 * We read r7 = task node virt pointer, and dump model_ctx at 0x400-0x440.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <asm/io.h>

/* Probe on drv_ive_write_regs — simple, reliable.
 * Manually walk globals to find model_ctx. */

static int call_count;

/* Use hardcoded address from kallsyms for the global */
static unsigned long g_xnn_ctx_addr;
module_param(g_xnn_ctx_addr, ulong, S_IRUGO);

static int write_pre(struct kprobe *p, struct pt_regs *regs)
{
	u32 phys = regs->ARM_r0;
	u8 *virt;
	u8 *base;
	u32 ctx_arr_ptr;
	u16 model_idx;
	u8 *mctx;
	int i;

	if (phys < 0x40000000 || phys > 0x50000000) return 0;

	virt = phys_to_virt(phys);
	if (!virt) return 0;

	/* node[6..7] = model context index */
	model_idx = *(u16 *)(virt + 6);

	pr_info("kp_ive: SUBMIT phys=0x%x type=%d model_idx=%d\n",
		phys, virt[11], model_idx);

	if (g_xnn_ctx_addr) {
		base = (u8 *)g_xnn_ctx_addr;

		/* Dump relevant offsets from g_xnn_ctx to find model_ctx_arr */
		pr_info("kp_ive: g_xnn[0x064]=%08x\n", *(u32*)(base+0x64));
		pr_info("kp_ive: g_xnn[0x1c0]=%08x\n", *(u32*)(base+0x1c0));
		pr_info("kp_ive: g_xnn[0x1cc]=%08x\n", *(u32*)(base+0x1cc));

		ctx_arr_ptr = *(u32*)(base + 0x1c0);
		if (ctx_arr_ptr) {
			mctx = (u8*)(unsigned long)ctx_arr_ptr +
				(u32)model_idx * 1080;
			pr_info("kp_ive: model_ctx at %p (arr=0x%x + %d*1080)\n",
				mctx, ctx_arr_ptr, model_idx);

			/* Dump 0x400-0x440 */
			for (i = 0x400; i < 0x440; i += 16)
				pr_info("kp_ive: mctx+%03x: %*ph\n",
					i, 16, mctx + i);
		} else {
			pr_info("kp_ive: model_ctx_arr is NULL\n");
		}
	}

	/* Also dump first node */
	pr_info("kp_ive: node +00: %*ph\n", 16, virt);
	pr_info("kp_ive: node +10: %*ph\n", 16, virt+16);

	call_count++;
	return 0;
}

static struct kprobe kp = { .symbol_name = "drv_ive_write_regs" };

static int __init kprobe_ive_init(void)
{
	int ret;

	if (!g_xnn_ctx_addr) {
		/* Try to find from kallsyms */
		g_xnn_ctx_addr = kallsyms_lookup_name("g_ive_xnn_ctx");
		if (!g_xnn_ctx_addr)
			g_xnn_ctx_addr = kallsyms_lookup_name("g_svp_alg_xnn_ctx");
	}

	kp.pre_handler = write_pre;
	ret = register_kprobe(&kp);
	if (ret < 0) return ret;

	pr_info("kp_ive: ready, g_xnn_ctx=0x%lx\n", g_xnn_ctx_addr);
	return 0;
}

static void __exit kprobe_ive_exit(void) { unregister_kprobe(&kp); }
module_init(kprobe_ive_init);
module_exit(kprobe_ive_exit);
MODULE_LICENSE("GPL");
