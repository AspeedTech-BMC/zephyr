/*
 * Copyright 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>
#include "spi.h"

void main(void)
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

	printk("%s demo\n", CONFIG_BOARD);

	spim_dev1 = device_get_binding("spi_m1");
	if (!spim_dev1) {
		printk("[demo_err]: cannot get device, spi_m1.\n");
		return;
	}

	/* reset BMC and flash */
	pfr_bmc_rst_enable_ctrl(true);
	spim_rst_flash(spim_dev1, 500);

	/* config SPI1 CS0 as master */
	spim_passthrough_enable(spim_dev1, 0, false);
	spim_ext_mux_config(spim_dev1, SPIM_MASTER_MODE);

	/* disable passthrough mode for other SPI monitors */
	for (i = 0; i < 3; i++) {
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

	/* config spim1 as SPI monitor */
	spim_ext_mux_config(spim_dev1, SPIM_MONITOR_MODE);

	/* set up passthrough mode for other SPI monitors */
	for (i = 0; i < 3; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("[demo_err]: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		spim_passthrough_enable(spim_dev, SPIM_SINGLE_PASSTHROUGH, true);
	}

	pfr_bmc_rst_enable_ctrl(false);
}
