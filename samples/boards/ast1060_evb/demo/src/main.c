/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>
#include "spi.h"
#include "demo_gpio.h"

static struct k_work rst_work;
static struct gpio_callback gpio_cb;

static void ast1060_rst_demo_passthrough(struct k_work *item)
{
	int ret;
	const struct device *spim_dev = NULL;
	uint32_t i;
	static char *spim_devs[4] = {
		"spi_m1",
		"spi_m2",
		"spi_m3",
		"spi_m4"
	};

	printk("[demo] BMC is AC on or reset!\n");

	spim_dev = device_get_binding("spi_m1");
	if (!spim_dev) {
		printk("[demo_err]: cannot get device, spi_m1.\n");
		return;
	}

	/* reset BMC and flash */
	pfr_bmc_rst_enable_ctrl(true);
	spim_rst_flash(spim_dev, 500);

	/* disable passthrough mode for SPI monitors */
	for (i = 0; i < 4; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("[demo_err]: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		spim_passthrough_enable(spim_dev, 0, false);
	}

	/* emulate PFR read flash each flash content for verification purpose */
	ret = test_spi_host_read();
	if (ret)
		return;

	/* set up passthrough mode for SPI monitors */
	for (i = 0; i < 4; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("[demo_err]: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		spim_passthrough_enable(spim_dev, SPIM_SINGLE_PASSTHROUGH, true);
	}

	pfr_bmc_rst_enable_ctrl(false);

}

static void gpioo3_callback(const struct device *dev,
		       struct gpio_callback *gpio_cb, uint32_t pins)
{
	/* reset BMC first */
	pfr_bmc_rst_enable_ctrl(true);

	k_work_submit(&rst_work);

	ARG_UNUSED(*gpio_cb);
	ARG_UNUSED(pins);
}

static void rst_gpio_event_register(void)
{
	int ret;
	const struct device *gpio_dev = device_get_binding(GPIOO3_DEV_NAME);

	if (!gpio_dev) {
		printk("[demo_err]: cannot get device, %s.\n", gpio_dev->name);
		return;
	}

	ret = gpio_pin_interrupt_configure(gpio_dev, GPIOO3_PIN_IN, GPIO_INT_DISABLE);
	if (ret != 0) {
		printk("[demo_err]: cannot configure GPIO INT(disabled).\n");
		return;
	}

	ret = gpio_pin_configure(gpio_dev, GPIOO3_PIN_IN, GPIO_INPUT);
	if (ret != 0) {
		printk("[demo_err]: cannot configure gpio device.\n");
		return;
	}

	gpio_init_callback(&gpio_cb, gpioo3_callback, BIT(GPIOO3_PIN_IN));
	ret = gpio_add_callback(gpio_dev, &gpio_cb);
	if (ret != 0) {
		printk("[demo_err]: cannot add gpio callback.\n");
		return;
	}

	ret = gpio_pin_interrupt_configure(gpio_dev, GPIOO3_PIN_IN, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret != 0) {
		printk("[demo_err]: cannot configure GPIO INT(1->0).\n");
		return;
	}
}

void main(void)
{
	printk("%s demo\n", CONFIG_BOARD);
	/* reset BMC first */
	pfr_bmc_rst_enable_ctrl(true);

	rst_gpio_event_register();

	k_work_init(&rst_work, ast1060_rst_demo_passthrough);

	ast1060_rst_demo_passthrough(NULL);
}
