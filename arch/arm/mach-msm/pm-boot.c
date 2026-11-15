/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#include <linux/init.h>
#include <asm/cacheflush.h>

#include "scm-boot.h"
#include "idle.h"
#include "pm-boot.h"
#include <mach/tz-restricted.h>
#include "lumia-tzpc.h"

static void (*msm_pm_boot_before_pc)(unsigned int cpu, unsigned long entry);
static void (*msm_pm_boot_after_pc)(unsigned int cpu);

static int __devinit msm_pm_tz_boot_init(void)
{
	unsigned int flag = 0;
	if (num_possible_cpus() == 1)
		flag = SCM_FLAG_WARMBOOT_CPU0;
	else if (num_possible_cpus() == 2)
		flag = SCM_FLAG_WARMBOOT_CPU0 | SCM_FLAG_WARMBOOT_CPU1;
	else if (num_possible_cpus() == 4)
		flag = SCM_FLAG_WARMBOOT_CPU0 | SCM_FLAG_WARMBOOT_CPU1 |
				SCM_FLAG_WARMBOOT_CPU2 | SCM_FLAG_WARMBOOT_CPU3;
	else
		__WARN();

	return scm_set_boot_addr(virt_to_phys(msm_pm_boot_entry), flag);
}

static void msm_pm_write_boot_vector(unsigned int cpu, unsigned long address)
{
	msm_pm_boot_vector[cpu] = address;

	dmac_clean_range((void *)&msm_pm_boot_vector[cpu],
			(void *)(&msm_pm_boot_vector[cpu] +
				sizeof(msm_pm_boot_vector[cpu])));
}

static void msm_pm_config_tz_before_pc(unsigned int cpu,
		unsigned long entry)
{
	msm_pm_write_boot_vector(cpu, entry);
}

void msm_pm_boot_config_before_pc(unsigned int cpu, unsigned long entry)
{
	if (msm_pm_boot_before_pc)
		msm_pm_boot_before_pc(cpu, entry);
}

void msm_pm_boot_config_after_pc(unsigned int cpu)
{
	if (msm_pm_boot_after_pc)
		msm_pm_boot_after_pc(cpu);
}

static int __init msm_pm_boot_init(void)
{
	int ret = 0;

	/*
	 * The warm-boot address is set through TrustZone; on firmware that
	 * rejects the call the SoC resets. Power collapse is disabled there,
	 * so no warm-boot vector is needed.
	 */
	if (msm_tz_restricted()) {
		/*
		 * The firmware already programmed the warm boot address of
		 * every core (its parking code); resume goes through the
		 * parking mailbox. The hooks do nothing unless
		 * lumia_tzpc.enable is set (lumia-tzpc.c).
		 */
		msm_pm_boot_before_pc = lumia_tzpc_before_pc;
		msm_pm_boot_after_pc = lumia_tzpc_after_pc;
		pr_info("pm-boot: TrustZone restricted, warm boot via the UEFI parking mailbox\n");
		return 0;
	}

	ret = msm_pm_tz_boot_init();
	msm_pm_boot_before_pc = msm_pm_config_tz_before_pc;
	msm_pm_boot_after_pc = NULL;

	return ret;
}
device_initcall(msm_pm_boot_init);
