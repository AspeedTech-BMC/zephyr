/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/abr_aspeed.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>
#include "demo_gpio.h"
#include <drivers/watchdog.h>
#include <soc.h>

extern struct k_work rst_work;

void demo_spim_irq_init(void);
void demo_wdt(void);
void ast1060_rst_demo_passthrough(struct k_work *item);
void rst_gpio_event_register(void);
void ast1060_i2c_demo_flt(void);

void main(void)
{
	printk("%s demo\n", CONFIG_BOARD);
	aspeed_print_sysrst_info();
	disable_abr_wdt();

	/* reset BMC first */
	pfr_bmc_srst_enable_ctrl(true);

	rst_gpio_event_register();

	k_work_init(&rst_work, ast1060_rst_demo_passthrough);

	demo_spim_irq_init();

	ast1060_rst_demo_passthrough(NULL);

#if CONFIG_I2C_PFR_FILTER
	ast1060_i2c_demo_flt();
#endif

	/* the following demo part should be allocated
	 * at the final step of main function.
	 */
#if CONFIG_WDT_DEMO_ASPEED
	demo_wdt();
#endif

}

