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
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/string.h>
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
 * System suspend hung the Lumia 1520 with the shipped kernel: a few seconds
 * after the last wakeup source went away (screen off, charger unplugged) the
 * phone stopped answering and needed a forced reboot. By default a wakeup
 * source is held from boot so that Android never starts a suspend: the phone
 * then idles in WFI as it does on USB.
 *
 * allow_suspend=1 (module parameter, also on the command line as
 * tz_restricted.allow_suspend=1) releases it and lets Android suspend with
 * the safe path below: secondary CPUs wait in WFI (cpu_down=2), every CPU at
 * its lowest frequency, a CPU0 WFI as the only sleep state, watchdogs that
 * turn a hang into a panic and a reboot, an optional RTC wake as a fallback
 * and a list of devices that stay out of the suspend (suspend_skip).
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
 * CPU hotplug. A secondary core released from the UEFI parking mailbox
 * cannot be parked or powered down again, but it can wait in WFI inside the
 * kernel (platform_do_lowpower()) until boot_secondary() releases it from the
 * pen. System suspend needs this: it takes every non-boot CPU down.
 *   0: refuse in platform_cpu_disable() (old behaviour)
 *   1: refuse in CPU_DOWN_PREPARE, before any notifier runs
 *   2: WFI hotplug during system suspend only, refuse at runtime (default)
 *   3: WFI hotplug always (tests)
 */
static int cpu_down_mode = 2;
module_param_named(cpu_down, cpu_down_mode, int, 0644);

bool msm_tz_restricted_cpu_down_ok(bool tasks_frozen)
{
	switch (cpu_down_mode) {
	case 0:		/* checked in platform_cpu_disable() */
	case 3:
		return true;
	case 2:
		return tasks_frozen;
	default:
		return false;
	}
}

bool msm_tz_restricted_cpu_disable_ok(void)
{
	return cpu_down_mode != 0;
}

/*
 * Suspend watchdog: if a suspend/resume cycle has not finished after
 * suspend_wd_secs (time spent asleep does not count, jiffies stop), dump
 * the suspending task, the blocked tasks that are not frozen and every CPU,
 * then panic, so the phone reboots by itself instead of hanging. The
 * firmware clears RAM on every reset: the dump is only seen in a log that
 * is streamed while it happens. 0 disables it.
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

/*
 * Safe system suspend. The Windows Phone TrustZone allows no power collapse,
 * so the deepest state is WFI: the RPM never sees the applications processor
 * asleep and keeps its active set (XO on, no vdd-min). What suspend still
 * saves over screen-off idle: userspace and freezable work stop, drivers drop
 * their clock, bus and regulator votes, and the CPUs stop waking up for
 * timers. These knobs make that state as cheap and as safe as possible.
 */
static bool tz_suspend_ready;

/*
 * suspend_min_freq: from PM_SUSPEND_PREPARE to PM_POST_SUSPEND every policy
 * is clamped to its lowest frequency, so the cores that wait in WFI (and the
 * Krait rail) sit at the lowest voltage instead of the last governor choice.
 */
static bool suspend_min_freq = true;
module_param(suspend_min_freq, bool, 0644);
static bool min_freq_clamped;

static int tz_cpufreq_policy_notify(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (event == CPUFREQ_ADJUST && min_freq_clamped)
		cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
					     policy->cpuinfo.min_freq);
	return NOTIFY_OK;
}

static struct notifier_block tz_cpufreq_policy_nb = {
	.notifier_call = tz_cpufreq_policy_notify,
};

static void tz_cpufreq_clamp(bool clamp)
{
	int cpu;

	min_freq_clamped = clamp;
	get_online_cpus();
	for_each_online_cpu(cpu)
		cpufreq_update_policy(cpu);
	put_online_cpus();
}

/*
 * suspend_wake_secs: optional RTC alarm armed for every suspend, a fallback
 * in case no other wakeup interrupt reaches CPU0 (Android normally sets its
 * own RTC alarms through the alarmtimer). 0 disables it.
 */
static unsigned int suspend_wake_secs;
module_param(suspend_wake_secs, uint, 0644);
static struct rtc_timer wake_rtc_timer;
static struct rtc_device *wake_rtc;
static bool wake_rtc_armed;
static struct wakeup_source tz_wake_ws;

static void tz_wake_rtc_fired(void *unused)
{
	/* aborts a suspend in progress, or wakes userspace after resume */
	__pm_wakeup_event(&tz_wake_ws, 500);
}

static void tz_wake_rtc_arm(void)
{
	struct rtc_time tm;
	ktime_t expires;

	if (!suspend_wake_secs)
		return;
	if (!wake_rtc)
		wake_rtc = alarmtimer_get_rtcdev();
	if (!wake_rtc || rtc_read_time(wake_rtc, &tm)) {
		pr_warn("tz_restricted: no RTC for the fallback wake\n");
		return;
	}
	expires = ktime_add(rtc_tm_to_ktime(tm),
			    ktime_set(suspend_wake_secs, 0));
	if (!rtc_timer_start(wake_rtc, &wake_rtc_timer, expires,
			     ktime_set(0, 0)))
		wake_rtc_armed = true;
}

static void tz_wake_rtc_cancel(void)
{
	if (!wake_rtc_armed)
		return;
	rtc_timer_cancel(wake_rtc, &wake_rtc_timer);
	wake_rtc_armed = false;
}

/*
 * suspend_skip: comma separated device names (as in /sys/devices, "name*"
 * matches a prefix) whose system sleep callbacks do not run. Their parents
 * stay awake too. Used to find, by elimination, the driver whose
 * suspend/resume breaks the phone, and to keep it out afterwards.
 */
static char suspend_skip[256];
module_param_string(suspend_skip, suspend_skip, sizeof(suspend_skip), 0644);

static bool tz_skip_match(const char *name)
{
	const char *p = suspend_skip;
	size_t len, nlen = strlen(name);

	while (*p) {
		len = strcspn(p, ", \n");
		if (len && p[len - 1] == '*') {
			if (!strncmp(p, name, len - 1))
				return true;
		} else if (len && len == nlen && !strncmp(p, name, len)) {
			return true;
		}
		p += len;
		p += strspn(p, ", \n");
	}
	return false;
}

bool dpm_skip_device(struct device *dev)
{
	const char *name;

	if (!tz_suspend_ready || !suspend_skip[0])
		return false;
	name = dev_name(dev);
	if (!name || !tz_skip_match(name))
		return false;
	pr_info("tz_restricted: %s stays out of suspend\n", name);
	return true;
}

/* Runs before the cpufreq driver locks frequency changes for the suspend */
static int tz_suspend_prepare_notify(struct notifier_block *nb,
				     unsigned long event, void *unused)
{
	if (event != PM_SUSPEND_PREPARE)
		return NOTIFY_DONE;
	pr_info("tz_restricted: suspend (cpu_down=%d min_freq=%d wake=%us skip=%s)\n",
		cpu_down_mode, suspend_min_freq, suspend_wake_secs,
		suspend_skip[0] ? suspend_skip : "-");
	if (suspend_min_freq)
		tz_cpufreq_clamp(true);
	tz_wake_rtc_arm();
	return NOTIFY_DONE;
}

/* Runs after the cpufreq driver accepts frequency changes again */
static int tz_suspend_post_notify(struct notifier_block *nb,
				  unsigned long event, void *unused)
{
	if (event != PM_POST_SUSPEND)
		return NOTIFY_DONE;
	tz_wake_rtc_cancel();
	if (min_freq_clamped)
		tz_cpufreq_clamp(false);
	return NOTIFY_DONE;
}

static struct notifier_block tz_suspend_prepare_nb = {
	.notifier_call = tz_suspend_prepare_notify,
	.priority = INT_MAX,
};

static struct notifier_block tz_suspend_post_nb = {
	.notifier_call = tz_suspend_post_notify,
	.priority = INT_MIN,
};

static int __init tz_restricted_safe_suspend_init(void)
{
	if (!msm_tz_restricted())
		return 0;
	wakeup_source_init(&tz_wake_ws, "tz_restricted_wake");
	rtc_timer_init(&wake_rtc_timer, tz_wake_rtc_fired, NULL);
	cpufreq_register_notifier(&tz_cpufreq_policy_nb,
				  CPUFREQ_POLICY_NOTIFIER);
	register_pm_notifier(&tz_suspend_prepare_nb);
	register_pm_notifier(&tz_suspend_post_nb);
	tz_suspend_ready = true;
	return 0;
}
late_initcall(tz_restricted_safe_suspend_init);
