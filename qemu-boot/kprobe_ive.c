/*
 * kprobe_ive — Capture tile data table from ive_start_task
 *
 * Probes at ive_start_task+0x68 (bcc) where r4=global, r7=task_node.
 * Reads model_ctx and tile table to understand Conv HW configuration.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/kallsyms.h>
#include <asm/io.h>

static int call_count;

static int start_handler(struct kprobe *p, struct pt_regs *regs)
{
	u32 r4 = regs->ARM_r4;  /* global base (g_ive_md_ctx) */
	u32 r7 = regs->ARM_r7;  /* task node virtual addr */
	u8 *node = (u8 *)(unsigned long)r7;
	u32 ctx_arr, node12;
	u8 *mctx;

	if (!node || node[10] != 0x36) return 0;

	node12 = *(u32 *)(node + 12);
	pr_info("kp_ive: node type=%d node[12]=0x%x\n", node[11], node12);

	ctx_arr = *(u32 *)((u8 *)r4 + 0x1c0);
	if (ctx_arr) {
		u16 model_idx = *(u16 *)(node + 6);
		u32 field420, base18;
		u8 *table;

		mctx = (u8 *)(unsigned long)ctx_arr + model_idx * 1080;

		field420 = *(u32 *)(mctx + 0x420);
		base18 = *(u32 *)(mctx + 0x18);

		pr_info("kp_ive: mctx=%p field420=%d base18=0x%x\n",
			mctx, field420, base18);
		pr_info("kp_ive: mctx+420: %*ph\n", 16, mctx + 0x420);

		/* Tile table = node[12] * field420 + base18 */
		table = (u8 *)(unsigned long)(base18 + node12 * field420);
		pr_info("kp_ive: tile_table = 0x%x * %d + 0x%x = %p\n",
			node12, field420, base18, table);

		/* Dump first 3 table entries (20 bytes each) */
		pr_info("kp_ive: entry[0]: %*ph\n", 20, table);
		pr_info("kp_ive: entry[1]: %*ph\n", 20, table + 20);
		pr_info("kp_ive: entry[2]: %*ph\n", 20, table + 40);
	}

	call_count++;
	return 0;
}

static struct kprobe kp_start;

static int __init kprobe_ive_init(void)
{
	int ret;
	unsigned long addr = kallsyms_lookup_name("ive_start_task");
	if (!addr) return -ENOENT;

	kp_start.addr = (void *)(addr + 0x68);
	kp_start.pre_handler = start_handler;
	ret = register_kprobe(&kp_start);
	if (ret < 0) return ret;

	pr_info("kp_ive: probing at %pS\n", kp_start.addr);
	return 0;
}

static void __exit kprobe_ive_exit(void)
{
	unregister_kprobe(&kp_start);
}

module_init(kprobe_ive_init);
module_exit(kprobe_ive_exit);
MODULE_LICENSE("GPL");
