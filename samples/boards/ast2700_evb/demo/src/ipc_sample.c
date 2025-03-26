/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#define DEFAULT_LINE_LENGTH_BYTES (16)
void test_ipm_cb(const struct device *ipmdev, void *user_data,
			uint32_t id, void *msg_data)
{
	int i;
	int width = 4;
	int linelen = DEFAULT_LINE_LENGTH_BYTES / width;
	int max_msg_data_size = ipm_max_data_size_get(ipmdev);
	uint32_t *buf = (uint32_t *)msg_data;

	printk("%s:msg id %x, msg data at %p, msg size 0x%x\n",
	       __func__, id, msg_data, max_msg_data_size);
	while (max_msg_data_size) {
		printk("%p:", buf);

		for (i = 0; i < linelen; i++)
			printk(" %08x ", buf[i]);
		printk("\n");
		buf += linelen;
		max_msg_data_size -= linelen * width;
	}
}

int ipc_test(void)
{
	const struct device *ipmdev;
	char ipc_name[32];
	int device_id, enable, rc;

	strcpy(ipc_name, "ipc1@400");
	ipmdev = device_get_binding(&ipc_name[0]);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = 1;
		goto fail;
	}

	device_id = 0;
	enable = 1;
	ipm_register_id_callback(ipmdev, device_id, (void *)test_ipm_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, enable);
	if (rc) {
		printk("%s: cannot ipm_set_enabled\n", ipc_name);
		rc = 1;
		goto fail;
	}

fail:
	return rc;
}
