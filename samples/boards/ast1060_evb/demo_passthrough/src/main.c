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
#include <drivers/i2c.h>
#include <drivers/i2c/pfr/i2c_filter.h>
#include <drivers/watchdog.h>
#include <soc.h>

static struct k_work rst_work;
static struct gpio_callback gpio_cb;
static struct ast_i2c_f_bitmap data_flt[] = {
{
{	/* block all (index 0) */
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000
}
},
{
{	/* accept all (index 1) */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
}
},
{
{	/* block every 16 byte (index 2) */
	0xFFFF0000, 0xFFFF0000, 0xFFFF0000, 0xFFFF0000,
	0xFFFF0000, 0xFFFF0000, 0xFFFF0000, 0xFFFF0000
}
},
{
{	/* block first 16 byte (index 3) */
	0xFFFF0000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
}
},
{
{	/* block first 128 byte (index 4) */
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
}
},
{
{	/* block last 128 byte (index 5) */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
}
}
};

void demo_spim_irq_init(void);
void demo_wdt(void);

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

static void ast1060_i2c_demo_flt(void)
{
	uint8_t EEPROM_COUNT = 8;
	uint8_t EEPROM_PASS_TBL[] = {0, 0, 0, 1, 1, 4, 4, 5};
	const struct device *pfr_flt_dev = NULL;
	int ret = 0, i = 0;

	/* initial flt */
	pfr_flt_dev = device_get_binding("I2C_FILTER_0");
	if (!pfr_flt_dev) {
		printk("I2C PFR : FLT Device driver not found.");
		return;
	}

	if (pfr_flt_dev != NULL) {
		ret = ast_i2c_filter_init(pfr_flt_dev);
		if (ret) {
			printk("I2C PFR : FLT Device Initial failed.");
			return;
		}

		ret = ast_i2c_filter_en(pfr_flt_dev, 1, 1, 0, 0);
		if (ret) {
			printk("I2C PFR : FLT Device Enable / Disable failed.");
			return;
		}

		ret = ast_i2c_filter_default(pfr_flt_dev, 0);
		if (ret) {
			printk("I2C PFR : FLT Device Set Default failed.");
			return;
		}

		for (i = 0x0; i < EEPROM_COUNT; i++) {
			ret = ast_i2c_filter_update(pfr_flt_dev, (uint8_t)(i),
			(uint8_t) (0x50 + i), &data_flt[EEPROM_PASS_TBL[i]]);
			if (ret) {
				printk("I2C PFR : FLT Device Update failed.");
				return;
			}
		}
	}
}

void main(void)
{
	printk("%s demo\n", CONFIG_BOARD);
	aspeed_print_sysrst_info();

	/* reset BMC first */
	pfr_bmc_rst_enable_ctrl(true);

	rst_gpio_event_register();

	k_work_init(&rst_work, ast1060_rst_demo_passthrough);

	demo_spim_irq_init();

	ast1060_rst_demo_passthrough(NULL);

	/* the following demo part should be allocated
	 * at the final step of main function.
	 */
#if CONFIG_WDT_DEMO_ASPEED
	demo_wdt();
#endif

#if CONFIG_I2C_PFR_FILTER
	ast1060_i2c_demo_flt();
#endif
}

