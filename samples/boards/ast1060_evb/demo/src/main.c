/*
 * Copyright 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>
#include "spi.h"

void main(void)
{
	int ret;
	printk("%s demo\n", CONFIG_BOARD);

	pfr_bmc_rst_enable_ctrl(true);

	pfr_bmc_rst_flash(1);

	ret = test_spi_host_read();
	if (ret)
		return;

	pfr_bmc_rst_enable_ctrl(false);
}
