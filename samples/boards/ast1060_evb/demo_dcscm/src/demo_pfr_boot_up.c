/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdlib.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>
#include <drivers/gpio.h>

#define HOST_SPI_MONITOR_NUM 3

int demo_spi_host_read(void);
int ast1030_img_xfer(void);

void aspeed_dcscm_rst_demo(struct k_work *item)
{
	int ret;
	const struct device *spim_dev = NULL;
	uint32_t i;
	static char *spim_devs[HOST_SPI_MONITOR_NUM] = {
		"spi_m1",
		"spi_m3",
		"spi_m4"
	};

	/* disable passthrough mode for other SPI monitors */
	for (i = 0; i < HOST_SPI_MONITOR_NUM; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("demo_err: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		spim_rst_flash(spim_dev, 1000);

		spim_passthrough_config(spim_dev, 0, false);
		/* config all spi monitor as master mode */
		spim_ext_mux_config(spim_dev, 1);
	}

	/* emulate PFR reads each flash content for verification purpose */
	ret = demo_spi_host_read();
	if (ret)
		return;

	/* set up monitor mode for all SPI monitors */
	for (i = 0; i < HOST_SPI_MONITOR_NUM; i++) {
		spim_dev = device_get_binding(spim_devs[i]);
		if (!spim_dev) {
			printk("demo_err: cannot get device, %s.\n", spim_devs[i]);
			return;
		}

		/* config spim1 as SPI monitor */
		spim_ext_mux_config(spim_dev, 0);
	}

	ret = ast1030_img_xfer();
	if (ret != 0) {
		printk("demo_err: fail to transfer binary to ast1030.\n");
		return;
	}

	pfr_pch_rst_enable_ctrl(false);
	ARG_UNUSED(item);
}
