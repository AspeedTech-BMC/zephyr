/*
 * Copyright (c) 2022 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <sys/printk.h>
#include <string.h>
#include <device.h>
#include <drivers/misc/aspeed/pcc_aspeed.h>

void main(void)
{
	int rc;
	uint32_t count = 0;
	uint8_t data;
	const struct device *pcc_dev;

	pcc_dev = device_get_binding(DT_LABEL(DT_NODELABEL(pcc)));
	if (!pcc_dev) {
		printk("No PCC device found\n");
		return;
	}

	printk("Reading PCC data ...\n");

	while (1) {
		rc = pcc_aspeed_read(pcc_dev, &data, true);
		if (rc == 0) {
			printk("%02x ", data);
			++count;
		}

		if (count % 16 == 0)
			printk("\n");
	}
}
