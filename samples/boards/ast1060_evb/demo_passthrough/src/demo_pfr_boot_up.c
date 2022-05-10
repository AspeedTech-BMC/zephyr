/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>

int demo_spi_host_read(void);
void demo_rst_log_ptr(const struct device *dev);

void ast1060_rst_demo_passthrough(struct k_work *item)
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
	pfr_bmc_srst_enable_ctrl(true);
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

	/* emulate PFR reads each flash content for verification purpose */
	ret = demo_spi_host_read();
	if (ret)
		return;

	/* set up passthrough mode for SPI monitors */
	for (i = 0; i < 4; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("demo_err: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		aspeed_spi_monitor_sw_rst(spim_dev);
		demo_rst_log_ptr(spim_dev);
		spim_passthrough_config(spim_dev, SPIM_SINGLE_PASSTHROUGH, true);
	}

	pfr_bmc_srst_enable_ctrl(false);

}

