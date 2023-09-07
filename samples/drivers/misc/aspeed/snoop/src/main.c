/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/misc/aspeed/snoop_aspeed.h>
#include <zephyr/kernel.h>

static void snoop_rx_callback(const uint8_t *snoop0, const uint8_t *snoop1)
{
	if (snoop0)
		printk("snoop ch0: %02x\n", *snoop0);

	if (snoop1)
		printk("snoop ch1: %02x\n", *snoop1);
}

void main(void)
{
	int rc;
	const struct device *snoop_dev;

	snoop_dev = device_get_binding("snoop");
	if (!snoop_dev) {
		printk("No snoop device found\n");
		return;
	}

	rc = snoop_aspeed_register_rx_callback(snoop_dev, snoop_rx_callback);
	if (rc) {
		printk("Cannot register RX callback\n");
		return;
	}

	printk("Incoming Snoop data ...\n");

	while (1)
		k_msleep(100);
}
