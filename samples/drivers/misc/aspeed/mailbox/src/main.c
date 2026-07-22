/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/misc/aspeed/mbox_aspeed.h>

#define MBOX_DR_NUM			MBX_DAT_REG_NUM
#define MBOX_DR_PART		(MBOX_DR_NUM / 2)
#define VERBOSE				1
#define POLL_INTERVAL_MS	10

void main(void)
{
	int i, rc;
	const struct device *mbox_dev;
	uint8_t dr_buf[MBOX_DR_NUM];

	mbox_dev = device_get_binding("mbox");
	if (!mbox_dev) {
		printk("No mbox device found\n");
		return;
	}

	printk("waiting mailbox input\n");

	while (1) {
		rc = mbox_aspeed_read(mbox_dev, dr_buf, MBOX_DR_PART, 0);
		if (rc) {
			k_msleep(POLL_INTERVAL_MS);
			continue;
		}

		memcpy(dr_buf + MBOX_DR_PART, dr_buf, MBOX_DR_PART);

		if (VERBOSE)
			for (i = 0; i < MBOX_DR_PART; ++i)
				printk("H2B: dr[%d]=0x%02x\n", i, dr_buf[i]);

		rc = mbox_aspeed_write(mbox_dev, dr_buf + MBOX_DR_PART, MBOX_DR_PART, MBOX_DR_PART);
		if (rc) {
			printk("error while writing mailbox loopback data, rc=%d\n", rc);
			continue;
		}

		if (VERBOSE)
			for (i = MBOX_DR_PART; i < MBOX_DR_NUM; ++i)
				printk("B2H: dr[%d]=0x%02x\n", i, dr_buf[i]);
	}
}
