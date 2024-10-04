/*
 * Copyright (c) 2022 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/misc/aspeed/pcc_aspeed.h>

static uint32_t count;

static void pcc_rx_callback(const uint8_t *rb, uint32_t rb_sz, uint32_t st_idx, uint32_t ed_idx)
{
	uint32_t i = st_idx;

	do {
		if (count % 16 == 0)
			printk("\n");

		printk("%02x ", rb[i]);

		i = (i + 1) % rb_sz;
		++count;
	} while (i != ed_idx);
}

void main(void)
{
	int rc;
	const struct device *pcc_dev;

	pcc_dev = device_get_binding("pcc");
	if (!pcc_dev) {
		printk("No PCC device found\n");
		return;
	}

	rc = pcc_aspeed_register_rx_callback(pcc_dev, pcc_rx_callback);
	if (rc) {
		printk("Cannot register RX callback\n");
		return;
	}

	printk("Incoming PCC data ... ");

	while (1)
		k_msleep(100);
}
