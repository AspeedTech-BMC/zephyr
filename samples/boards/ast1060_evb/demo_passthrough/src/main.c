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
#include <drivers/watchdog.h>
#include <soc.h>

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

	spim_dev = device_get_binding("spi_m1");
	if (!spim_dev) {
		printk("demo_err: cannot get device, spi_m1.\n");
		return;
	}

	/* reset BMC and flash */
	pfr_bmc_rst_enable_ctrl(true);
	spim_rst_flash(spim_dev, 500);

	/* disable passthrough mode for SPI monitors */
	for (i = 0; i < 4; i++) {
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

	/* set up passthrough mode for SPI monitors */
	for (i = 0; i < 4; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("demo_err: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		spim_passthrough_config(spim_dev, SPIM_SINGLE_PASSTHROUGH, true);
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

#if CONFIG_WDT_DEMO_ASPEED
static void demo_wdt_callback(const struct device *dev, int channel_id)
{
	printk("[%s] timeout triggered.\n", dev->name);
}

static void wdt_demo(void)
{
	struct wdt_timeout_cfg wdt_config;
	const struct device *wdt3_dev;
	const struct device *wdt4_dev;
	int ret;
	uint32_t count = 0;

	printk("wdt demo started.\n");

	wdt3_dev = device_get_binding("wdt3");
	if (!wdt3_dev) {
		printk("demo_err: cannot find wdt3 device.\n");
		return;
	}

	wdt4_dev = device_get_binding("wdt4");
	if (!wdt3_dev) {
		printk("demo_err: cannot find wdt4 device.\n");
		return;
	}

	wdt_config.window.min = 0U;
	wdt_config.window.max = 1000; /* 1 s */
	wdt_config.callback = demo_wdt_callback;
	ret = wdt_install_timeout(wdt3_dev, &wdt_config);
	if (ret != 0) {
		printk("demo_err: fail to install wdt3 timeout.\n");
		return;
	}

	wdt_config.window.min = 0U;
	wdt_config.window.max = 3000; /* 3s */
	wdt_config.callback = NULL;
	ret = wdt_install_timeout(wdt4_dev, &wdt_config);
	if (ret != 0) {
		printk("demo_err: fail to install wdt4 timeout.\n");
		return;
	}

	ret = wdt_setup(wdt3_dev, WDT_FLAG_RESET_NONE);
	if (ret != 0) {
		printk("demo_err: fail to setup wdt3.\n");
		return;
	}

	ret = wdt_setup(wdt4_dev, WDT_FLAG_RESET_CPU_CORE);
	if (ret != 0) {
		printk("demo_err: fail to setup wdt4.\n");
		return;
	}

	while (1) {
		if (count == 10) {
			printk("wdt_demo getting stuck...\n");
			k_sleep(K_FOREVER);
		}

		printk("wdt_demo ... %d\n", count);

		wdt_feed(wdt3_dev, 0);
		wdt_feed(wdt4_dev, 0);
		if (count < 5)
			k_sleep(K_MSEC(500));
		else
			k_sleep(K_MSEC(2000));

		count++;
	}
}
#endif

void main(void)
{
	printk("%s demo\n", CONFIG_BOARD);
	aspeed_print_sysrst_info();

	/* reset BMC first */
	pfr_bmc_rst_enable_ctrl(true);

	rst_gpio_event_register();

	k_work_init(&rst_work, ast1060_rst_demo_passthrough);

	ast1060_rst_demo_passthrough(NULL);

	/* the following demo part should be allocated
	 * at the final step of main function.
	 */
#if CONFIG_WDT_DEMO_ASPEED
	wdt_demo();
#endif
}

void wdt_demo_background_thread(void)
{
	int ret = 0;
	const struct device *wdt2_dev = NULL;
	struct wdt_timeout_cfg wdt_config;

	wdt2_dev = device_get_binding("wdt2");
	if (!wdt2_dev) {
		printk("demo_err: cannot find wdt2 device.\n");
		return;
	}

	wdt_config.window.min = 0U;
	wdt_config.window.max = 5000; /* 5s */
	wdt_config.callback = NULL;
	ret = wdt_install_timeout(wdt2_dev, &wdt_config);
	if (ret != 0) {
		printk("demo_err: fail to install wdt2 timeout.\n");
		return;
	}

	ret = wdt_setup(wdt2_dev, WDT_FLAG_RESET_CPU_CORE);
	if (ret != 0) {
		printk("demo_err: fail to setup wdt2.\n");
		return;
	}

	while (1) {
#if CONFIG_WDT_DEMO_ASPEED
		printk("wdt_demo thread still alive...\n");
#endif
		wdt_feed(wdt2_dev, 0);
		k_sleep(K_MSEC(3000));
	}
}

K_THREAD_DEFINE(wdt_background, 1024, wdt_demo_background_thread,
	NULL, NULL, NULL, -1, 0, 1000);

