/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2026 ASPEED Technology Inc.
 */

#include <soc.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/misc/aspeed/ast2700_spim.h>

void sipm_demo(void)
{
	const struct device *spim_dev = device_get_binding("spim@1");

	spim_isr_callback_install(spim_dev, ast2700_spim_blocked_log_parser);
}
