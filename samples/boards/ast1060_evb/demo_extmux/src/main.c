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
#include <soc.h>

static struct k_work rst_work;
static struct gpio_callback gpio_cb;

void demo_spim_irq_init(void);
void demo_wdt(void);

static void ast1060_rst_demo_ext_mux(struct k_work *item)
{
	int ret;
	const struct device *spim_dev1 = NULL;
	const struct device *spim_dev = NULL;
	uint32_t i;
	static char *spim_devs[3] = {
		"spi_m2",
		"spi_m3",
		"spi_m4"
	};

	spim_dev1 = device_get_binding("spi_m1");
	if (!spim_dev1) {
		printk("demo_err: cannot get device, spi_m1.\n");
		return;
	}

	spim_rst_flash(spim_dev1, 1000);

	/* config SPI1 CS0 as master */
	spim_passthrough_config(spim_dev1, 0, false);
	spim_ext_mux_config(spim_dev1, SPIM_MASTER_MODE);

	/* disable passthrough mode for other SPI monitors */
	for (i = 0; i < 3; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("demo_err: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		spim_passthrough_config(spim_dev, 0, false);
	}

	/* emulate PFR read flash each flash content for verification purpose */
	ret = test_spi_host_read();
	if (ret)
		return;

	/* config spim1 as SPI monitor */
	spim_ext_mux_config(spim_dev1, SPIM_MONITOR_MODE);

	/* set up passthrough mode for other SPI monitors */
	for (i = 0; i < 3; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("demo_err: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		spim_passthrough_config(spim_dev, SPIM_SINGLE_PASSTHROUGH, true);
	}

	pfr_bmc_rst_enable_ctrl(false);
	ARG_UNUSED(item);
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
		printk("demo_err: cannot get device, %s.\n", gpio_dev->name);
		return;
	}

	ret = gpio_pin_interrupt_configure(gpio_dev, GPIOO3_PIN_IN, GPIO_INT_DISABLE);
	if (ret != 0) {
		printk("demo_err: cannot configure GPIO INT(disabled).\n");
		return;
	}

	ret = gpio_pin_configure(gpio_dev, GPIOO3_PIN_IN, GPIO_INPUT);
	if (ret != 0) {
		printk("demo_err: cannot configure gpio device.\n");
		return;
	}

	gpio_init_callback(&gpio_cb, gpioo3_callback, BIT(GPIOO3_PIN_IN));
	ret = gpio_add_callback(gpio_dev, &gpio_cb);
	if (ret != 0) {
		printk("demo_err: cannot add gpio callback.\n");
		return;
	}

	ret = gpio_pin_interrupt_configure(gpio_dev, GPIOO3_PIN_IN, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret != 0) {
		printk("demo_err: cannot configure GPIO INT(1->0).\n");
		return;
	}
}

void main(void)
{
	printk("%s demo\n", CONFIG_BOARD);
	aspeed_print_sysrst_info();

	/* reset BMC first */
	pfr_bmc_rst_enable_ctrl(true);

	rst_gpio_event_register();

	k_work_init(&rst_work, ast1060_rst_demo_ext_mux);
	demo_spim_irq_init();

	ast1060_rst_demo_ext_mux(NULL);

	/* the following demo part should be allocated
	 * at the final step of main function.
	 */
#if CONFIG_WDT_DEMO_ASPEED
	demo_wdt();
#endif
}

