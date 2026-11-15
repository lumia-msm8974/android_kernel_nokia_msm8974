/*
 * Lumia 930 / 1520: CPU power collapse through the Windows Phone firmware.
 *
 * Copyright (c) 2026, Lorenzo Giannoccaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * EXPERIMENTAL. Everything here is off unless lumia_tzpc.enable=1 (and the
 * core is in lumia_tzpc.cpus). See SUSPEND-TZ-WINDOWS-RE.md for the
 * analysis this is based on.
 *
 * What Windows does (qcpep8974.sys) and what the TrustZone expects:
 *  - the SCM atomic call TERMINATE_PC (service 1, command 2, flags in the
 *    third register) is the same one the Android kernel makes: the Windows
 *    TrustZone does the final WFI in monitor mode after the SPM (SAW2) has
 *    been armed by the OS with the standalone power collapse sequence;
 *  - nobody calls SET_BOOT_ADDR: the UEFI already registered, for every
 *    core, a warm boot address in the TrustZone, the park loop that sits in
 *    the core's mailbox page (0x300000 + (cpu + 1) * 0x1000, offset 0x800);
 *  - the park loop compares mailbox +0 with the core id, and when they are
 *    equal and +8 is not zero it jumps there in ARM state, MMU and caches
 *    off, r0 = mailbox, after clearing +8/+12. So the OS tells the firmware
 *    where to resume by publishing the entry in the mailbox before the SCM
 *    call: entry at +8, core id at +0 last.
 *
 * Here the entry is lumia_pm_boot_entry (idle-v7.S): it puts the mailbox id
 * back to idle (0xffffffff, the value the SMP bring-up expects) and joins
 * the usual msm_pm_boot_entry, which jumps to msm_pm_boot_vector[cpu]
 * (cpu_resume).
 *
 * The L2 stays on unless lumia_tzpc.l2=2 is given on the kernel command
 * line. Then lpm_levels may pick its L2 GDHS system level once all the
 * online cores are in standalone power collapse: it arms the L2 SAW2 with
 * the GDHS sequence (the same bytes as Windows) and the last core passes
 * the GDHS flag (3) to TERMINATE_PC. GDHS keeps the L2 contents, so
 * nothing is flushed. L2 power collapse, the RPM sleep set and XO shutdown
 * are not allowed yet.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/percpu.h>
#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <mach/tz-restricted.h>
#include "idle.h"
#include "pm.h"
#include "spm.h"
#include "lumia-tzpc.h"

static bool tzpc_enable;
module_param_named(enable, tzpc_enable, bool, 0644);
MODULE_PARM_DESC(enable, "allow the power collapse of the cores in cpus");

/* Bit n = Linux CPU n may collapse. Default: none. Start with 0x8 (CPU3). */
static unsigned int tzpc_cpus;
module_param_named(cpus, tzpc_cpus, uint, 0644);

static atomic_t tzpc_entered = ATOMIC_INIT(0);
static atomic_t tzpc_resumed = ATOMIC_INIT(0);
static atomic_t tzpc_fallthru = ATOMIC_INIT(0);
static atomic_t tzpc_nobox = ATOMIC_INIT(0);
module_param_named(entered, tzpc_entered.counter, int, 0444);
module_param_named(resumed, tzpc_resumed.counter, int, 0444);
module_param_named(fallthru, tzpc_fallthru.counter, int, 0444);
module_param_named(nobox, tzpc_nobox.counter, int, 0444);

/*
 * Krait CP15 state Windows saves around a power collapse (qcpep8974.sys,
 * .pwrclps+0xfc): CPMR (c15,c0,5) and five more registers of unknown
 * meaning. They are read before the collapse and compared after it; only
 * CPMR (hardware clock gating, ETM clock, WFE fixup: Android sets it up
 * at run time) is written back. The other five are only reported in
 * lumia_tzpc.c15diff (bit n = register n changed).
 */
static unsigned int tzpc_c15diff;
module_param_named(c15diff, tzpc_c15diff, uint, 0444);
static bool tzpc_c15 = true;
module_param_named(c15, tzpc_c15, bool, 0644);
struct tzpc_c15_regs { u32 v[6]; };
static DEFINE_PER_CPU(struct tzpc_c15_regs, tzpc_c15_saved);

static void tzpc_c15_read(u32 *v)
{
	asm volatile("mrc p15, 7, %0, c15, c0, 5" : "=r" (v[0]));
	asm volatile("mrc p15, 7, %0, c15, c2, 7" : "=r" (v[1]));
	asm volatile("mrc p15, 7, %0, c15, c3, 1" : "=r" (v[2]));
	asm volatile("mrc p15, 7, %0, c15, c3, 2" : "=r" (v[3]));
	asm volatile("mrc p15, 7, %0, c15, c3, 0" : "=r" (v[4]));
	asm volatile("mrc p15, 7, %0, c15, c2, 6" : "=r" (v[5]));
}

bool lumia_tzpc_cpu_allowed(unsigned int cpu)
{
	return tzpc_enable && cpu < 4 && (tzpc_cpus & BIT(cpu)) &&
		lumia_park_mailbox(cpu);
}

/*
 * Deepest L2 low power mode lpm_levels may use, as MSM_SPM_L2_MODE_*.
 * Boot time only (it decides how lpm_levels counts the cores in sync):
 * 0 = none (default), 2 = GDHS. Deeper modes are refused for now.
 */
static unsigned int tzpc_l2;
module_param_named(l2, tzpc_l2, uint, 0444);
static atomic_t tzpc_l2_entered = ATOMIC_INIT(0);
module_param_named(l2_entered, tzpc_l2_entered.counter, int, 0444);

int lumia_tzpc_l2_max(void)
{
	if (!msm_tz_restricted() || tzpc_l2 < MSM_SPM_L2_MODE_GDHS)
		return -1;
	return MSM_SPM_L2_MODE_GDHS;
}

/*
 * SCM flag for TERMINATE_PC. want is what lpm_levels asked for the last
 * core (msm_pm_get_l2_flush_flag()); anything but an allowed GDHS becomes
 * MSM_SCM_L2_ON (low nibble 0, the L2 stays on).
 */
unsigned int lumia_tzpc_l2_flag(unsigned int want)
{
	if (want == MSM_SCM_L2_GDHS && tzpc_enable &&
	    lumia_tzpc_l2_max() >= MSM_SPM_L2_MODE_GDHS) {
		atomic_inc(&tzpc_l2_entered);
		return MSM_SCM_L2_GDHS;
	}
	return MSM_SCM_L2_ON;
}

void lumia_tzpc_before_pc(unsigned int cpu, unsigned long entry)
{
	void __iomem *box;

	if (!tzpc_enable || !msm_tz_restricted())
		return;

	box = lumia_park_mailbox(cpu);
	if (!box) {
		atomic_inc(&tzpc_nobox);
		return;
	}

	if (tzpc_c15)
		tzpc_c15_read(this_cpu_ptr(&tzpc_c15_saved)->v);

	/* Where lumia_pm_boot_entry sends the core (physical address) */
	msm_pm_boot_vector[cpu] = entry;
	dmac_clean_range((void *)&msm_pm_boot_vector[cpu],
			 (void *)(&msm_pm_boot_vector[cpu] + 1));

	/*
	 * Publish the resume entry, the core id last (same order as the SMP
	 * bring-up). The mailbox must be idle: if not, leave it alone, the
	 * collapse then comes back at once through the SCM fall-through.
	 */
	if (readl_relaxed(box) != 0xffffffff || readl_relaxed(box + 8)) {
		atomic_inc(&tzpc_nobox);
		return;
	}
	writel_relaxed(0, box + 12);
	writel_relaxed(virt_to_phys((void *)lumia_pm_boot_entry), box + 8);
	wmb();
	writel_relaxed(cpu_logical_map(cpu), box);
	dsb();
	atomic_inc(&tzpc_entered);
}

void lumia_tzpc_after_pc(unsigned int cpu)
{
	void __iomem *box;

	if (!tzpc_enable || !msm_tz_restricted())
		return;

	box = lumia_park_mailbox(cpu);
	if (!box)
		return;

	if (readl_relaxed(box) == cpu_logical_map(cpu)) {
		/*
		 * Still published: the core did not collapse (the SCM call
		 * returned). Put the mailbox back to idle.
		 */
		writel_relaxed(0, box + 8);
		writel_relaxed(0, box + 12);
		wmb();
		writel_relaxed(0xffffffff, box);
		dsb();
		atomic_inc(&tzpc_fallthru);
	} else {
		atomic_inc(&tzpc_resumed);
		if (tzpc_c15) {
			u32 now[6], *old = this_cpu_ptr(&tzpc_c15_saved)->v;
			int i;

			tzpc_c15_read(now);
			for (i = 0; i < 6; i++)
				if (now[i] != old[i])
					tzpc_c15diff |= BIT(i);
			if (now[0] != old[0]) {
				asm volatile("mcr p15, 7, %0, c15, c0, 5"
					     : : "r" (old[0]));
				isb();
			}
		}
	}
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

/*
 * /sys/kernel/debug/lumia_tzpc_mailbox: read-only dump of the UEFI parking
 * mailbox of every core: the first four words and the park loop at +0x800,
 * which must start with "mov sl, pc" (e1a0a00f) before power collapse is tried.
 */
static int tzpc_mailbox_show(struct seq_file *m, void *unused)
{
	unsigned int cpu, i;

	for (cpu = 0; cpu < 4; cpu++) {
		void __iomem *box = lumia_park_mailbox(cpu);

		if (!box) {
			seq_printf(m, "cpu%u: no mailbox\n", cpu);
			continue;
		}
		seq_printf(m, "cpu%u: +0 %08x +4 %08x +8 %08x +c %08x\n", cpu,
			   readl_relaxed(box), readl_relaxed(box + 4),
			   readl_relaxed(box + 8), readl_relaxed(box + 12));
		seq_printf(m, "cpu%u: +800", cpu);
		for (i = 0; i < 0x74; i += 4)
			seq_printf(m, " %08x", readl_relaxed(box + 0x800 + i));
		seq_puts(m, "\n");
	}
	return 0;
}

static int tzpc_mailbox_open(struct inode *inode, struct file *file)
{
	return single_open(file, tzpc_mailbox_show, NULL);
}

static const struct file_operations tzpc_mailbox_fops = {
	.open		= tzpc_mailbox_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tzpc_debugfs_init(void)
{
	if (msm_tz_restricted())
		debugfs_create_file("lumia_tzpc_mailbox", 0400, NULL, NULL,
				    &tzpc_mailbox_fops);
	return 0;
}
late_initcall(tzpc_debugfs_init);
#endif
