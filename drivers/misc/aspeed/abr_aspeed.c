/*
 * Copyright (c) 2022 - 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/abr_aspeed.h>

#define ABR_CTRL_REG	0x7e620064
#define WDT_EVENT_CNT_CLEAR_MASK 0xff000000
#define WDT_EVENT_CNT_CLEAR_MAGIC (0xea << 24)
#define ABR_INDICATOR_CLEAR_MASK 0x00ff0000
#define ABR_INDICATOR_CLEAR_MAGIC (0xea << 16)

enum boot_indicator get_boot_indicator(void)
{
	if (sys_read32(ABR_CTRL_REG) & BIT(4))
		return BOOT_FROM_ALTERNATE_PART;

	return BOOT_FROM_PRIMARY_PART;
}

void disable_abr_wdt(void)
{
	uint32_t reg_val;

	reg_val = sys_read32(ABR_CTRL_REG);
	reg_val &= ~(BIT(0));
	sys_write32(reg_val, ABR_CTRL_REG);
}

void clear_abr_event_count(void)
{
	uint32_t reg_val;

	reg_val = sys_read32(ABR_CTRL_REG);
	reg_val &= ~WDT_EVENT_CNT_CLEAR_MASK;
	reg_val |= WDT_EVENT_CNT_CLEAR_MAGIC;
	sys_write32(reg_val, ABR_CTRL_REG);
}

void clear_abr_indicator(void)
{
	uint32_t reg_val;

	reg_val = sys_read32(ABR_CTRL_REG);
	reg_val &= ~ABR_INDICATOR_CLEAR_MASK;
	reg_val |= ABR_INDICATOR_CLEAR_MAGIC;
	sys_write32(reg_val, ABR_CTRL_REG);
}

