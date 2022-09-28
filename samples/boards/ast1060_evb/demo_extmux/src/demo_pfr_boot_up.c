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

void ast1060_rst_demo_ext_mux(struct k_work *item)
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
	spim_ext_mux_config(spim_dev1, 0);

	/* disable passthrough mode for other SPI monitors */
	for (i = 0; i < 3; i++) {
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

	aspeed_spi_monitor_sw_rst(spim_dev1);
	demo_rst_log_ptr(spim_dev1);

	/* config spim1 as SPI monitor */
	spim_ext_mux_config(spim_dev1, 1);

	/* set up passthrough mode for other SPI monitors */
	for (i = 0; i < 3; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("demo_err: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		aspeed_spi_monitor_sw_rst(spim_dev);
		demo_rst_log_ptr(spim_dev);
		spim_passthrough_config(spim_dev, SPIM_SINGLE_PASSTHROUGH, true);
	}

	/*
	 * wait for 50ms after configuring external MUX
	 * in order to get overall stable system.
	 */
	k_busy_wait(50000);

	pfr_bmc_srst_enable_ctrl(false);
	ARG_UNUSED(item);
}
