/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/ipm.h>

LOG_MODULE_REGISTER(ipc_test, CONFIG_SOC_LOG_LEVEL);

#define DEFAULT_LINE_LENGTH_BYTES	(16)
#define IPC_DEV_SSP_BOOTMCU		"ipc1@400"
#define IPC_CHANNEL_ID			0

static void ch0_ipm_cb(const struct device *ipmdev, void *user_data,
		       uint32_t id, volatile void *msg_data)
{
	int i;
	int width = 4;
	int linelen = DEFAULT_LINE_LENGTH_BYTES / width;
	int max_msg_data_size = ipm_max_data_size_get(ipmdev);
	uint32_t *buf = (uint32_t *)msg_data;

	LOG_INF("%s:msg id %x, msg data at %p, msg size 0x%x\n",
		__func__, id, msg_data, max_msg_data_size);
	while (max_msg_data_size) {
		LOG_INF("%p:", buf);

		for (i = 0; i < linelen; i++)
			LOG_INF(" %08x ", buf[i]);
		LOG_INF("\n");
		buf += linelen;
		max_msg_data_size -= linelen * width;
	}

	max_msg_data_size = ipm_max_data_size_get(ipmdev);
	ipm_send(ipmdev, 0, id, (void *)msg_data, max_msg_data_size);
}

int ipc_test(void)
{
	char ipc_name[32] = IPC_DEV_SSP_BOOTMCU;
	int device_id = IPC_CHANNEL_ID;
	const struct device *ipmdev;
	int rc;

	LOG_INF("%s", __func__);

	ipmdev = device_get_binding(ipc_name);
	if (!ipmdev) {
		LOG_ERR("%s: device_get_binding failed to find device\n", ipc_name);
		rc = 1;
		return rc;
	}

	ipm_register_id_callback(ipmdev, device_id, ch0_ipm_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, 1);
	if (rc) {
		LOG_ERR("%s: cannot ipm_set_enabled\n", ipc_name);
		rc = 1;
		return rc;
	}

	return 0;
}
