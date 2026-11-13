/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/qpnp/power-on.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>

#include <asm/mach-types.h>
#include <asm/cacheflush.h>

#include <mach/msm_iomap.h>
#include <mach/restart.h>
#include <mach/socinfo.h>
#include <mach/irqs.h>
#include <mach/scm.h>
#include "msm_watchdog.h"
#include "timer.h"
#include "wdog_debug.h"

#define WDT0_RST	0x38
#define WDT0_EN		0x40
#define WDT0_BARK_TIME	0x4C
#define WDT0_BITE_TIME	0x5C

#define PSHOLD_CTL_SU (MSM_TLMM_BASE + 0x820)

#define RESTART_REASON_ADDR 0x65C
#define DLOAD_MODE_ADDR     0x0
#define EMERGENCY_DLOAD_MODE_ADDR    0xFE0
#define EMERGENCY_DLOAD_MAGIC1    0x322A4F99
#define EMERGENCY_DLOAD_MAGIC2    0xC67E4350
#define EMERGENCY_DLOAD_MAGIC3    0x77777777

#define SCM_IO_DISABLE_PMIC_ARBITER	1

#ifdef CONFIG_MSM_RESTART_V2
#define use_restart_v2()	1
#else
#define use_restart_v2()	0
#endif

static int restart_mode;
void *restart_reason;

int pmic_reset_irq;
static void __iomem *msm_tmr0_base;

#ifdef CONFIG_MSM_DLOAD_MODE
static int in_panic;
static void *dload_mode_addr;
static bool dload_mode_enabled;
static void *emergency_dload_mode_addr;

/* Download mode master kill-switch */
static int dload_set(const char *val, struct kernel_param *kp);
static int download_mode = 1;
module_param_call(download_mode, dload_set, param_get_int,
			&download_mode, 0644);
static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

static void set_dload_mode(int on)
{
	if (dload_mode_addr) {
		__raw_writel(on ? 0xE47B337D : 0, dload_mode_addr);
		__raw_writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		mb();
		dload_mode_enabled = on;
	}
}

#ifndef CONFIG_MACH_OPPO
static bool get_dload_mode(void)
{
	return dload_mode_enabled;
}
#endif

static void enable_emergency_dload_mode(void)
{
	if (emergency_dload_mode_addr) {
		__raw_writel(EMERGENCY_DLOAD_MAGIC1,
				emergency_dload_mode_addr);
		__raw_writel(EMERGENCY_DLOAD_MAGIC2,
				emergency_dload_mode_addr +
				sizeof(unsigned int));
		__raw_writel(EMERGENCY_DLOAD_MAGIC3,
				emergency_dload_mode_addr +
				(2 * sizeof(unsigned int)));

		/* Need disable the pmic wdt, then the emergency dload mode
		 * will not auto reset. */
		qpnp_pon_wd_config(0);
		mb();
	}
}

static int dload_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = download_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not zero or one, ignore. */
	if (download_mode >> 1) {
		download_mode = old_val;
		return -EINVAL;
	}

	set_dload_mode(download_mode);

	return 0;
}
#else
#define set_dload_mode(x) do {} while (0)

static void enable_emergency_dload_mode(void)
{
	printk(KERN_ERR "dload mode is not enabled on target\n");
}

#ifndef CONFIG_MACH_OPPO
static bool get_dload_mode(void)
{
	return false;
}
#endif
#endif

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

static bool scm_pmic_arbiter_disable_supported;
/*
 * Force the SPMI PMIC arbiter to shutdown so that no more SPMI transactions
 * are sent from the MSM to the PMIC.  This is required in order to avoid an
 * SPMI lockup on certain PMIC chips if PS_HOLD is lowered in the middle of
 * an SPMI transaction.
 */
static void halt_spmi_pmic_arbiter(void)
{
	if (scm_pmic_arbiter_disable_supported) {
		pr_crit("Calling SCM to disable SPMI PMIC arbiter\n");
		scm_call_atomic1(SCM_SVC_PWR, SCM_IO_DISABLE_PMIC_ARBITER, 0);
	}
}

static void __msm_power_off(int lower_pshold)
{
	printk(KERN_CRIT "Powering off the SoC\n");
#ifdef CONFIG_MSM_DLOAD_MODE
	set_dload_mode(0);
#endif
	pm8xxx_reset_pwr_off(0);
	qpnp_pon_system_pwr_off(PON_POWER_OFF_SHUTDOWN);

	if (lower_pshold) {
		if (!use_restart_v2()) {
			__raw_writel(0, PSHOLD_CTL_SU);
		} else {
			halt_spmi_pmic_arbiter();
			__raw_writel(0, MSM_MPM2_PSHOLD_BASE);
		}

		mdelay(10000);
		printk(KERN_ERR "Powering off has failed\n");
	}
	return;
}

static void msm_power_off(void)
{
	/* MSM initiated power off, lower ps_hold */
	__msm_power_off(1);
}

static void cpu_power_off(void *data)
{
	int rc;

	pr_err("PMIC Initiated shutdown %s cpu=%d\n", __func__,
						smp_processor_id());
	if (smp_processor_id() == 0) {
		/*
		 * PMIC initiated power off, do not lower ps_hold, pmic will
		 * shut msm down
		 */
		__msm_power_off(0);

		pet_watchdog();
		pr_err("Calling scm to disable arbiter\n");
		/* call secure manager to disable arbiter and never return */
		rc = scm_call_atomic1(SCM_SVC_PWR,
						SCM_IO_DISABLE_PMIC_ARBITER, 1);

		pr_err("SCM returned even when asked to busy loop rc=%d\n", rc);
		pr_err("waiting on pmic to shut msm down\n");
	}

	preempt_disable();
	while (1)
		;
}

static irqreturn_t resout_irq_handler(int irq, void *dev_id)
{
	pr_warn("%s PMIC Initiated shutdown\n", __func__);
	oops_in_progress = 1;
	smp_call_function_many(cpu_online_mask, cpu_power_off, NULL, 0);
	if (smp_processor_id() == 0)
		cpu_power_off(NULL);
	preempt_disable();
	while (1)
		;
	return IRQ_HANDLED;
}

/*
 * Lumia test: the Nokia UEFI shows the empty battery icon after every
 * restart from Linux, never after a cold boot. Shared IMEM survives a
 * PMIC reset but not a power-off, so keep a copy of what the bootloader
 * left there, show what Linux changed (debugfs lumia_imem/diff) and,
 * with restore_imem=1, put the copy back right before the reset.
 * restore_imem_off >= 0 restores only the 32-bit word at that offset.
 */
#define LUMIA_IMEM_SIZE SZ_4K
static u32 lumia_imem_boot[LUMIA_IMEM_SIZE / 4];
static bool lumia_imem_saved;
static int restore_imem;
module_param(restore_imem, int, 0644);
static int restore_imem_off = -1;
module_param(restore_imem_off, int, 0644);

static void lumia_imem_save(void)
{
	int i;

	for (i = 0; i < LUMIA_IMEM_SIZE / 4; i++)
		lumia_imem_boot[i] = __raw_readl(MSM_IMEM_BASE + 4 * i);
	lumia_imem_saved = true;
}

static void lumia_imem_restore(void)
{
	int i, n = 0;

	if (!lumia_imem_saved || !restore_imem)
		return;
	for (i = 0; i < LUMIA_IMEM_SIZE / 4; i++) {
		u32 now = __raw_readl(MSM_IMEM_BASE + 4 * i);

		if (now == lumia_imem_boot[i])
			continue;
		if (restore_imem_off >= 0 && restore_imem_off != 4 * i)
			continue;
		pr_notice("lumia_imem: 0x%03x %08x -> %08x\n", 4 * i, now,
			  lumia_imem_boot[i]);
		__raw_writel(lumia_imem_boot[i], MSM_IMEM_BASE + 4 * i);
		n++;
	}
	mb();
	pr_notice("lumia_imem: restored %d words\n", n);
}

static int lumia_imem_diff_show(struct seq_file *m, void *unused)
{
	int i;

	seq_printf(m, "# saved=%d restore_imem=%d restore_imem_off=%d\n",
		   lumia_imem_saved, restore_imem, restore_imem_off);
	seq_puts(m, "# offset boot     now\n");
	for (i = 0; i < LUMIA_IMEM_SIZE / 4; i++) {
		u32 now = __raw_readl(MSM_IMEM_BASE + 4 * i);

		if (now != lumia_imem_boot[i])
			seq_printf(m, "0x%03x %08x %08x\n", 4 * i,
				   lumia_imem_boot[i], now);
	}
	return 0;
}

static int lumia_imem_diff_open(struct inode *inode, struct file *file)
{
	return single_open(file, lumia_imem_diff_show, NULL);
}

static const struct file_operations lumia_imem_diff_fops = {
	.open = lumia_imem_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct debugfs_blob_wrapper lumia_imem_blob = {
	.data = lumia_imem_boot,
	.size = sizeof(lumia_imem_boot),
};

static int __init lumia_imem_debugfs_init(void)
{
	struct dentry *d = debugfs_create_dir("lumia_imem", NULL);

	if (IS_ERR_OR_NULL(d))
		return 0;
	debugfs_create_file("diff", 0400, d, NULL, &lumia_imem_diff_fops);
	debugfs_create_blob("boot", 0400, d, &lumia_imem_blob);
	return 0;
}
late_initcall(lumia_imem_debugfs_init);

static void msm_restart_prepare(const char *cmd)
{
#ifdef CONFIG_MSM_DLOAD_MODE

	/* This looks like a normal reboot at this point. */
	set_dload_mode(0);

	/* Write download mode flags if we're panic'ing */
	set_dload_mode(in_panic);

	/* Write download mode flags if restart_mode says so */
	if (restart_mode == RESTART_DLOAD)
		set_dload_mode(1);

	/* Kill download mode if master-kill switch is set */
	if (!download_mode)
		set_dload_mode(0);
#endif

	pm8xxx_reset_pwr_off(1);

	/* Hard reset the PMIC unless memory contents must be maintained. */
#ifdef CONFIG_MACH_OPPO
	qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);
#else
	if (get_dload_mode() || (cmd != NULL && cmd[0] != '\0'))
		qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);
	else
		qpnp_pon_system_pwr_off(PON_POWER_OFF_HARD_RESET);
#endif

	if (cmd != NULL) {
		if (!strncmp(cmd, "bootloader", 10)) {
			__raw_writel(0x77665500, restart_reason);
		} else if (!strncmp(cmd, "recovery", 8)) {
			__raw_writel(0x77665502, restart_reason);
		} else if (!strcmp(cmd, "rtc")) {
			__raw_writel(0x77665503, restart_reason);
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned long code;
			code = simple_strtoul(cmd + 4, NULL, 16) & 0xff;
			__raw_writel(0x6f656d00 | code, restart_reason);
		} else if (!strncmp(cmd, "edl", 3)) {
			enable_emergency_dload_mode();
#ifdef CONFIG_MACH_OPPO
		} else if (!strncmp(cmd, "ftm", 3)) {
			__raw_writel(0x77665504, restart_reason);
		} else if (!strncmp(cmd, "wlan", 4)) {
			__raw_writel(0x77665505, restart_reason);
		} else if (!strncmp(cmd, "rf", 2)) {
			__raw_writel(0x77665506, restart_reason);
		} else if (!strncmp(cmd, "mos", 3)) {
			__raw_writel(0x77665507, restart_reason);
		} else if (!strncmp(cmd, "kernel", 6)) {
			__raw_writel(0x7766550a, restart_reason);
		} else if (!strncmp(cmd, "modem", 5)) {
			__raw_writel(0x7766550b, restart_reason);
		} else if (!strncmp(cmd, "android", 7)) {
			__raw_writel(0x7766550c, restart_reason);
#endif
		} else {
			__raw_writel(0x77665501, restart_reason);
		}
	}

	lumia_imem_restore();

	flush_cache_all();
	outer_flush_all();
}

void msm_restart(char mode, const char *cmd)
{
	printk(KERN_NOTICE "Going down for restart now\n");

	msm_restart_prepare(cmd);

	if (!use_restart_v2()) {
		__raw_writel(0, msm_tmr0_base + WDT0_EN);
		if (!(machine_is_msm8x60_fusion() ||
		      machine_is_msm8x60_fusn_ffa())) {
			mb();
			 /* Actually reset the chip */
			__raw_writel(0, PSHOLD_CTL_SU);
			mdelay(5000);
			pr_notice("PS_HOLD didn't work, falling back to watchdog\n");
		}

		__raw_writel(1, msm_tmr0_base + WDT0_RST);
		__raw_writel(5*0x31F3, msm_tmr0_base + WDT0_BARK_TIME);
		__raw_writel(0x31F3, msm_tmr0_base + WDT0_BITE_TIME);
		__raw_writel(1, msm_tmr0_base + WDT0_EN);
	} else {
		/* Needed to bypass debug image on some chips */
		msm_disable_wdog_debug();
		halt_spmi_pmic_arbiter();
		__raw_writel(0, MSM_MPM2_PSHOLD_BASE);
	}

	mdelay(10000);
	printk(KERN_ERR "Restarting has failed\n");
}

static int __init msm_pmic_restart_init(void)
{
	int rc;

	if (use_restart_v2())
		return 0;

	if (pmic_reset_irq != 0) {
		rc = request_any_context_irq(pmic_reset_irq,
					resout_irq_handler, IRQF_TRIGGER_HIGH,
					"restart_from_pmic", NULL);
		if (rc < 0)
			pr_err("pmic restart irq fail rc = %d\n", rc);
	} else {
		pr_warn("no pmic restart interrupt specified\n");
	}

	return 0;
}

late_initcall(msm_pmic_restart_init);

static int __init msm_restart_init(void)
{
	lumia_imem_save();
#ifdef CONFIG_MSM_DLOAD_MODE
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	dload_mode_addr = MSM_IMEM_BASE + DLOAD_MODE_ADDR;
	emergency_dload_mode_addr = MSM_IMEM_BASE +
		EMERGENCY_DLOAD_MODE_ADDR;
	set_dload_mode(download_mode);
#endif
	msm_tmr0_base = msm_timer_get_timer0_base();
	restart_reason = MSM_IMEM_BASE + RESTART_REASON_ADDR;
#ifdef CONFIG_MACH_OPPO
	__raw_writel(0x7766550a, restart_reason);
#endif
	pm_power_off = msm_power_off;

	if (scm_is_call_available(SCM_SVC_PWR, SCM_IO_DISABLE_PMIC_ARBITER) > 0)
		scm_pmic_arbiter_disable_supported = true;

	return 0;
}
early_initcall(msm_restart_init);
