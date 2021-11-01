/*
 * Copyright 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>

void main(void)
{
	printk("%s demo\n", CONFIG_BOARD);

	pfr_bmc_rst_flash(1);
	pfr_bmc_rst_enable_ctrl(false);
}
