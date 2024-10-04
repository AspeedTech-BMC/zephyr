/*
 * Copyright (c) 2021 - 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/misc/aspeed/pfr_aspeed.h>
#include "demo_gpio.h"

static struct gpio_callback gpio_cb[RST_IND_NUM];
struct k_work rst_work;

static void rst_ind_callback(const struct device *dev,
		       struct gpio_callback *gpio_cb, uint32_t pins)
{
	/* reset BMC and PCH first */
	pfr_bmc_extrst_enable_ctrl(true);
	pfr_bmc_srst_enable_ctrl(true);
	pfr_pch_rst_enable_ctrl(true);

	k_work_submit(&rst_work);

	ARG_UNUSED(*gpio_cb);
	ARG_UNUSED(pins);
}

void rst_ind_gpio_event_register(void)
{
	int ret;
	const struct device *gpio_devs[RST_IND_NUM] = {
			BMC_RSTIND_GPIO_DEV,
			BIC_RSTIND_GPIO_DEV,
			CPU0_RSTIND_GPIO_DEV,
			CPU1_RSTIND_GPIO_DEV,
	};
	gpio_pin_t gpio_pin[RST_IND_NUM] = {
			BMC_RSTIND_GPIO_PIN_IN,
			BIC_RSTIND_GPIO_PIN_IN,
			CPU0_RSTIND_GPIO_PIN_IN,
			CPU1_RSTIND_GPIO_PIN_IN
	};
	int i;

	for (i = 0; i < RST_IND_NUM; i++) {
		ret = gpio_pin_interrupt_configure(gpio_devs[i], gpio_pin[i],
						   GPIO_INT_DISABLE);
		if (ret != 0) {
			printk("demo_err: cannot configure GPIO INT(disabled).\n");
			return;
		}

		ret = gpio_pin_configure(gpio_devs[i], gpio_pin[i], GPIO_INPUT);
		if (ret != 0) {
			printk("demo_err: cannot configure gpio device.\n");
			return;
		}

		gpio_init_callback(&gpio_cb[i], rst_ind_callback, BIT(gpio_pin[i]));
		ret = gpio_add_callback(gpio_devs[i], &gpio_cb[i]);
		if (ret != 0) {
			printk("demo_err: cannot add gpio callback.\n");
			return;
		}

		ret = gpio_pin_interrupt_configure(gpio_devs[i], gpio_pin[i],
						   GPIO_INT_EDGE_TO_INACTIVE);
		if (ret != 0) {
			printk("demo_err: cannot configure GPIO INT(1->0).\n");
			return;
		}
	}
}
