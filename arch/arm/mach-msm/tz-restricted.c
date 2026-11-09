/*
 * Phones booted by the Windows Phone firmware (Nokia Lumia 1520 "bandit",
 * Nokia Lumia 930 "martini").
 *
 * Copyright (c) 2026, Lorenzo Giannoccaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * The Lumia TrustZone does not implement the SCM services that Android
 * MSM8974 kernels use for:
 *  - the CPU cold/warm boot address (scm_set_boot_addr): the call resets
 *    the SoC before any output, so secondary CPUs and power collapse are
 *    unavailable;
 *  - deep idle states that enter power collapse through TrustZone.
 * PAS/PIL authentication works (the ADSP boots on this firmware).
 *
 * Findings on real hardware by the mainline bring-up of the Lumia 1520
 * (KorelisLabs, lumia-1520-mainline, docs/trustzone-quirks.md).
 *
 * Detection: DT root compatible "nokia,bandit" or "nokia,martini", or
 * "msm_tz_restricted=<0|1>" on the command line to force it either way.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/freezer.h>
#include <linux/notifier.h>
#include <asm/smp.h>
#include <mach/tz-restricted.h>

static int tz_restricted_param = -1;

/* Boards booted by the Windows Phone firmware */
static const char * const tz_restricted_boards[] = {
	"nokia,bandit",
	"nokia,martini",
};

static int __init msm_tz_restricted_setup(char *str)
{
	get_option(&str, &tz_restricted_param);
	return 0;
}
early_param("msm_tz_restricted", msm_tz_restricted_setup);

bool msm_tz_restricted(void)
{
	int i;

	if (tz_restricted_param >= 0)
		return tz_restricted_param;
	for (i = 0; i < ARRAY_SIZE(tz_restricted_boards); i++)
		if (of_machine_is_compatible(tz_restricted_boards[i]))
			return true;
	return false;
}

bool __init msm_tz_restricted_early(void)
{
	int i;

	if (tz_restricted_param >= 0)
		return tz_restricted_param;
	for (i = 0; i < ARRAY_SIZE(tz_restricted_boards); i++)
		if (of_flat_dt_is_compatible(of_get_flat_dt_root(),
					     tz_restricted_boards[i]))
			return true;
	return false;
}

/*
 * System suspend hangs on the Lumia 1520: a few seconds after the last
 * wakeup source goes away (screen off, charger unplugged) userspace stays
 * frozen and the phone never comes back, with the shipped kernel too. Until
 * the hang is understood, hold a wakeup source from boot so that Android
 * never starts a suspend: the phone then idles in WFI as it does on USB.
 * For tests: echo 1 > /sys/module/tz_restricted/parameters/allow_suspend
 */
static struct wakeup_source tz_nosuspend_ws;
static bool tz_nosuspend_held;

static int allow_suspend_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (ret || !tz_nosuspend_held)
		return ret;
	if (*(bool *)kp->arg)
		__pm_relax(&tz_nosuspend_ws);
	else
		__pm_stay_awake(&tz_nosuspend_ws);
	return 0;
}

static struct kernel_param_ops allow_suspend_ops = {
	.set = allow_suspend_set,
	.get = param_get_bool,
};

static bool allow_suspend;
module_param_cb(allow_suspend, &allow_suspend_ops, &allow_suspend, 0644);

static int __init tz_restricted_nosuspend_init(void)
{
	if (!msm_tz_restricted())
		return 0;
	wakeup_source_init(&tz_nosuspend_ws, "tz_restricted_nosuspend");
	tz_nosuspend_held = true;
	if (!allow_suspend)
		__pm_stay_awake(&tz_nosuspend_ws);
	pr_info("tz_restricted: system suspend %s\n",
		allow_suspend ? "allowed" : "blocked (it hangs on this firmware)");
	return 0;
}
late_initcall(tz_restricted_nosuspend_init);

/*
 * Suspend watchdog: if a suspend/resume cycle has not finished after
 * suspend_wd_secs (time spent asleep does not count, jiffies stop), dump
 * the suspending task, the blocked tasks that are not frozen and every CPU,
 * then panic. The panic reboot is warm on this phone, so the dump survives
 * in the ram_console (/proc/last_kmsg). 0 disables it.
 */
static unsigned int suspend_wd_secs = 60;
module_param(suspend_wd_secs, uint, 0644);

static struct timer_list suspend_wd;
static struct task_struct *suspend_task;

static void suspend_wd_fire(unsigned long data)
{
	struct task_struct *g, *p;

	pr_emerg("tz_restricted: suspend not finished after %u s\n",
		 suspend_wd_secs);
	if (suspend_task) {
		pr_emerg("suspending task:\n");
		sched_show_task(suspend_task);
	}
	pr_emerg("blocked tasks that are not frozen:\n");
	rcu_read_lock();
	do_each_thread(g, p) {
		if (p->state == TASK_UNINTERRUPTIBLE && !frozen(p))
			sched_show_task(p);
	} while_each_thread(g, p);
	rcu_read_unlock();
	pr_emerg("all CPUs:\n");
	smp_send_all_cpu_backtrace();
	panic("tz_restricted: system suspend hang");
}

static int suspend_wd_notify(struct notifier_block *nb, unsigned long event,
			     void *unused)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		if (!suspend_wd_secs)
			break;
		get_task_struct(current);
		suspend_task = current;
		mod_timer(&suspend_wd, jiffies + suspend_wd_secs * HZ);
		break;
	case PM_POST_SUSPEND:
		del_timer_sync(&suspend_wd);
		if (suspend_task) {
			put_task_struct(suspend_task);
			suspend_task = NULL;
		}
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block suspend_wd_nb = {
	.notifier_call = suspend_wd_notify,
};

static int __init tz_restricted_suspend_wd_init(void)
{
	if (!msm_tz_restricted())
		return 0;
	setup_timer(&suspend_wd, suspend_wd_fire, 0);
	return register_pm_notifier(&suspend_wd_nb);
}
late_initcall(tz_restricted_suspend_wd_init);
