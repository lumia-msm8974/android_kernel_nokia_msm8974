/*
 * Lumia 930 / 1520: CPU power collapse through the Windows Phone firmware.
 *
 * Copyright (c) 2026, Lorenzo Giannoccaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _ARCH_ARM_MACH_MSM_LUMIA_TZPC_H_
#define _ARCH_ARM_MACH_MSM_LUMIA_TZPC_H_

#ifdef CONFIG_MACH_NOKIA_LUMIA8974
void lumia_tzpc_before_pc(unsigned int cpu, unsigned long entry);
void lumia_tzpc_after_pc(unsigned int cpu);
void __iomem *lumia_park_mailbox(unsigned int cpu);
void lumia_pm_boot_entry(void);
#else
static inline void lumia_tzpc_before_pc(unsigned int cpu, unsigned long entry) { }
static inline void lumia_tzpc_after_pc(unsigned int cpu) { }
#endif

#endif
