/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>
#include <drivers/gpio.h>

#define HOST_SPI_MONITOR_NUM 3

int demo_spi_host_read(void);
void demo_rst_log_ptr(const struct device *dev);

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

		/* spim_rst_flash(spim_dev, 1000); */

		spim_passthrough_config(spim_dev, 0, false);
		spim_scu_passthrough_mode(spim_dev, 0, false);
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

		aspeed_spi_monitor_sw_rst(spim_dev);
		demo_rst_log_ptr(spim_dev);

		/* config spim as SPI monitor */
		spim_ext_mux_config(spim_dev, 0);
	}

	/* config SPIM1 scu passthrough mode */
	spim_dev = device_get_binding(spim_devs[0]);
	if (!spim_dev) {
		printk("demo_err: cannot get device, %s.\n", spim_devs[0]);
		return;
	}

	spim_scu_passthrough_mode(spim_dev, 0, true);

	/* config SPIM3 scu passthrough mode */
	spim_dev = device_get_binding(spim_devs[1]);
	if (!spim_dev) {
		printk("demo_err: cannot get device, %s.\n", spim_devs[1]);
		return;
	}

	spim_scu_passthrough_mode(spim_dev, 0, true);

	/*
	 * wait for 50ms after configuring external MUX
	 * in order to get overall stable system.
	 */
	k_busy_wait(50000);

	pfr_bmc_srst_enable_ctrl(false);
	pfr_bmc_extrst_enable_ctrl(false);

	pfr_pch_rst_enable_ctrl(false);
	ARG_UNUSED(item);
}
