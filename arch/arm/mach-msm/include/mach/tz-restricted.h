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
 * Experimental CPU power collapse through the Windows Phone firmware
 * (lumia-tzpc.c). All off unless the lumia_tzpc.enable parameter is set.
 */
bool lumia_tzpc_cpu_allowed(unsigned int cpu);
unsigned int lumia_tzpc_l2_flag(void);
#else
static inline bool msm_tz_restricted(void) { return false; }
static inline bool msm_tz_restricted_early(void) { return false; }
static inline bool lumia_tzpc_cpu_allowed(unsigned int cpu) { return false; }
static inline unsigned int lumia_tzpc_l2_flag(void) { return 0; }
#endif

#endif
