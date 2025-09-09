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

#define SCU1_REG		DT_REG_ADDR(DT_NODELABEL(syscon))
#define SCU1_RSTLOG0		(SCU1_REG + 0x050)
#define   SCU1_RSTLOG0_SRST	BIT(0)
#define SCU1_MISC		(SCU1_REG + 0x0c0)
#define   SCU1_MISC_UARTBMC_SEL	BIT(20)

extern char __RAM_NC_start[];
extern char __RAM_NC_end[];

void z_arm_platform_init(void)
{
	/* clear non-cached .bss */
	(void)memset(__RAM_NC_start, 0, __RAM_NC_end - __RAM_NC_start);

	/* TODO: Set SCU_UARTBMC_SEL to BMC UART. This is a WA. */
	sys_write32(SCU1_MISC_UARTBMC_SEL, SCU1_MISC);
}
