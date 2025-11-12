/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <string.h>
#include <soc.h>

#define SCU1_REG		DT_REG_ADDR(DT_NODELABEL(syscon1))
#define SCU1_RSTLOG0		(SCU1_REG + 0x050)
#define   SCU1_RSTLOG0_SRST	BIT(0)

extern char __RAM_NC_start[];
extern char __RAM_NC_end[];

void z_arm_platform_init(void)
{
	/* clear non-cached .bss */
	(void)memset(__RAM_NC_start, 0, __RAM_NC_end - __RAM_NC_start);
}

/* DMA address translation */
#if defined(CONFIG_SOC_AST1040_BOOTMCU)
uint64_t ast27xx_soc_virt_addr_to_phy_addr(uintptr_t addr)
{
	if (addr < 0x80000000) {
		return addr;
	}

	return (uint64_t)(addr & ~0x80000000) | 0x400000000ULL;
}

uintptr_t ast27xx_soc_phy_addr_to_virt_addr(uint64_t addr)
{
	if (addr < 0x400000000) {
		return addr;
	}

	return (uintptr_t)(addr - 0x400000000ULL) | 0x80000000UL;
}
#endif
