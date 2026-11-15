/*
 * Screen-off power for the MSM8974 Lumias (1520 "bandit", 930 "martini")
 * without system suspend.
 *
 * Copyright (c) 2026, Lorenzo Giannoccaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * System suspend hangs on these phones (tz-restricted.c keeps it blocked)
 * and the CPUs idle only in WFI, so with the screen off the SoC stays at
 * the operating point that the UEFI handoff votes leave: BIMC/DDR and the
 * NoCs at their maximum, CX at its top corner, the Krait rails at their
 * highest voltage (clock.c, msm_clock_handoff_set()). This driver drops the
 * selected groups of those votes and, optionally, caps the CPU frequency:
 *
 *   groups     HANDOFF_GRP_* mask of the votes to drop (0 = none, default)
 *   always     0: drop them only while the screen is off (default),
 *              1: drop them now and keep them dropped
 *   max_freq   CPU frequency cap in kHz while the screen is off (0 = none)
 *   doze_groups  the part of "groups" also dropped while the panel dozes
 *              (always-on display); default bus and Krait voltages: the
 *              display keeps scanning out and votes its own bandwidth, so
 *              the multimedia clocks (MMSS) must stay
 *   delay_ms   wait after the screen goes off before dropping (default 2000)
 *
 * Every write is applied at once (groups not in the mask get their votes
 * back), so "echo 0 > groups" undoes everything. The screen state comes from
 * the FB_EVENT_BLANK notifier of fb0: FB_BLANK_POWERDOWN is off,
 * FB_BLANK_VSYNC_SUSPEND and FB_BLANK_NORMAL (the MDSS low power states that
 * Android's doze uses) are doze. /sys/kernel/debug/lumia_screenoff lists the
 * handoff votes and their state.
 */

#define pr_fmt(fmt) "lumia_screenoff: " fmt

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/fb.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <mach/tz-restricted.h>
#include "clock.h"

static DEFINE_MUTEX(screenoff_lock);
static bool screenoff_ready;	/* set once boot is settled */
enum { SCREEN_ON, SCREEN_DOZE, SCREEN_OFF };
static const char *const screen_names[] = { "on", "doze", "off" };
static int screen_state;
static unsigned int applied_groups;
static unsigned int applied_cap;	/* read by the cpufreq notifier */

static unsigned int groups;
static bool always;
static unsigned int max_freq;
static unsigned int doze_groups = HANDOFF_GRP_BUS | HANDOFF_GRP_KRAIT;
static unsigned int delay_ms = 2000;

static void screenoff_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(screenoff_work, screenoff_work_fn);

static void screenoff_kick(unsigned int delay)
{
	if (!screenoff_ready)
		return;
	cancel_delayed_work(&screenoff_work);
	schedule_delayed_work(&screenoff_work, msecs_to_jiffies(delay));
}

static void screenoff_work_fn(struct work_struct *work)
{
	unsigned int want, cap;
	int cpu, rc;

	mutex_lock(&screenoff_lock);

	if (always || screen_state == SCREEN_OFF)
		want = groups & HANDOFF_GRP_ALL;
	else if (screen_state == SCREEN_DOZE)
		want = groups & doze_groups & HANDOFF_GRP_ALL;
	else
		want = 0;
	if (want != applied_groups) {
		rc = msm_clock_handoff_set(want);
		if (rc)
			pr_err("handoff groups %#x: error %d\n", want, rc);
		applied_groups = want;
		pr_info("handoff votes dropped: %#x (screen %s)\n", want,
			screen_names[screen_state]);
	}

	cap = screen_state == SCREEN_OFF ? max_freq : 0;
	if (cap != applied_cap) {
		ACCESS_ONCE(applied_cap) = cap;
		get_online_cpus();
		for_each_online_cpu(cpu)
			cpufreq_update_policy(cpu);
		put_online_cpus();
		pr_info("CPU cap %u kHz\n", cap);
	}

	mutex_unlock(&screenoff_lock);
}

static int screenoff_cpufreq_cb(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int cap = ACCESS_ONCE(applied_cap);

	if (event == CPUFREQ_ADJUST && cap)
		cpufreq_verify_within_limits(policy, 0, cap);
	return NOTIFY_OK;
}

static struct notifier_block screenoff_cpufreq_nb = {
	.notifier_call = screenoff_cpufreq_cb,
};

static int screenoff_fb_cb(struct notifier_block *nb, unsigned long event,
			   void *data)
{
	struct fb_event *evdata = data;
	int state;

	if (event != FB_EVENT_BLANK || !evdata || !evdata->data ||
	    !evdata->info || evdata->info->node != 0)
		return NOTIFY_DONE;

	switch (*(int *)evdata->data) {
	case FB_BLANK_POWERDOWN:
		state = SCREEN_OFF;
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		state = SCREEN_DOZE;
		break;
	default:
		state = SCREEN_ON;
		break;
	}
	if (state == screen_state)
		return NOTIFY_OK;
	screen_state = state;
	/* back to the firmware votes at once, drop them once settled */
	screenoff_kick(state == SCREEN_ON ? 0 : delay_ms);
	return NOTIFY_OK;
}

static struct notifier_block screenoff_fb_nb = {
	.notifier_call = screenoff_fb_cb,
};

static int screenoff_param_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	if (kp->arg == &always)
		ret = param_set_bool(val, kp);
	else
		ret = param_set_uint(val, kp);
	if (!ret)
		screenoff_kick(0);
	return ret;
}

static int screenoff_param_get(char *buf, const struct kernel_param *kp)
{
	if (kp->arg == &always)
		return param_get_bool(buf, kp);
	return param_get_uint(buf, kp);
}

static struct kernel_param_ops screenoff_param_ops = {
	.set = screenoff_param_set,
	.get = screenoff_param_get,
};

module_param_cb(groups, &screenoff_param_ops, &groups, 0644);
MODULE_PARM_DESC(groups, "handoff vote groups to drop (HANDOFF_GRP_* mask)");
module_param_cb(always, &screenoff_param_ops, &always, 0644);
MODULE_PARM_DESC(always, "drop them with the screen on too");
module_param_cb(max_freq, &screenoff_param_ops, &max_freq, 0644);
MODULE_PARM_DESC(max_freq, "CPU frequency cap with the screen off (kHz)");
module_param_cb(doze_groups, &screenoff_param_ops, &doze_groups, 0644);
MODULE_PARM_DESC(doze_groups, "part of groups also dropped while the panel dozes");
module_param(delay_ms, uint, 0644);
MODULE_PARM_DESC(delay_ms, "wait after screen off before dropping (ms)");

static int screenoff_show(struct seq_file *m, void *unused)
{
	mutex_lock(&screenoff_lock);
	seq_printf(m, "screen %s, groups %#x, always %d, dropped %#x, cap %u kHz\n",
		   screen_names[screen_state], groups, always, applied_groups,
		   applied_cap);
	mutex_unlock(&screenoff_lock);
	seq_puts(m, "groups: 01 bus, 02 qdss, 04 gpu/ocmem, 08 mmss, 10 vdd_dig, 20 krait, 40 rest, 00 kept\n");
	msm_clock_handoff_show(m);
	return 0;
}

static int screenoff_open(struct inode *inode, struct file *file)
{
	return single_open(file, screenoff_show, NULL);
}

static const struct file_operations screenoff_fops = {
	.open = screenoff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* Settled boot: drivers have taken their own votes, apply cmdline values */
static void screenoff_boot_fn(struct work_struct *work)
{
	screenoff_ready = true;
	screenoff_kick(0);
}
static DECLARE_DELAYED_WORK(screenoff_boot_work, screenoff_boot_fn);

static int __init lumia_screenoff_init(void)
{
	if (!msm_tz_restricted())
		return 0;
	cpufreq_register_notifier(&screenoff_cpufreq_nb,
				  CPUFREQ_POLICY_NOTIFIER);
	fb_register_client(&screenoff_fb_nb);
	debugfs_create_file("lumia_screenoff", 0444, NULL, NULL,
			    &screenoff_fops);
	schedule_delayed_work(&screenoff_boot_work, 30 * HZ);
	pr_info("ready in 30 s (groups %#x, always %d, cap %u kHz)\n",
		groups, always, max_freq);
	return 0;
}
late_initcall_sync(lumia_screenoff_init);
