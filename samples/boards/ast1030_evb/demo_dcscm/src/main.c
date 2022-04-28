/*
 * Copyright 2022 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <soc.h>
#include <drivers/misc/aspeed/abr_aspeed.h>

void main(void)
{
	printk("%s demo\n", CONFIG_BOARD);
	aspeed_print_sysrst_info();
	disable_abr_wdt();
}
