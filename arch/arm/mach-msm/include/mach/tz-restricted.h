/*
 * Copyright (c) 2026, Lorenzo Giannoccaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef __MACH_TZ_RESTRICTED_H
#define __MACH_TZ_RESTRICTED_H

#include <linux/types.h>

#ifdef CONFIG_MACH_NOKIA_LUMIA8974
/* True on firmware whose TrustZone lacks the Android SCM services */
bool msm_tz_restricted(void);
/* Same check usable from ->reserve(), before the DT is unflattened */
bool msm_tz_restricted_early(void);
/*
 * May a secondary CPU go offline? It cannot be powered down on this
 * firmware, so an offline CPU waits in WFI inside the kernel.
 * @tasks_frozen: the request comes from system suspend.
 */
bool msm_tz_restricted_cpu_down_ok(bool tasks_frozen);
/* False when platform_cpu_disable() must refuse (cpu_down=0) */
bool msm_tz_restricted_cpu_disable_ok(void);
#else
static inline bool msm_tz_restricted(void) { return false; }
static inline bool msm_tz_restricted_early(void) { return false; }
static inline bool msm_tz_restricted_cpu_down_ok(bool tasks_frozen)
{
	return true;
}
static inline bool msm_tz_restricted_cpu_disable_ok(void) { return true; }
#endif

#endif
