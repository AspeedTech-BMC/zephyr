/*
 * Copyright 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <soc.h>
#include <zephyr/drivers/misc/aspeed/abr_aspeed.h>

void main(void)
{
	printk("%s demo\n", CONFIG_BOARD);
	aspeed_print_sysrst_info();
	aspeed_soc_show_chip_id();
	disable_abr_wdt();
}

