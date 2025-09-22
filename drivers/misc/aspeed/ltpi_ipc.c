/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/ltpi_ipc.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/ipm_ast.h>

LOG_MODULE_REGISTER(ltpi_ipc, CONFIG_MISC_ASPEED_LOG_LEVEL);

#define IPC_DEV_NCA35_BOOTMCU		"ipc1@200"
#define IPC_CHANNEL_ID_LTPI		0
#define LTPISHEADER 0xf0000000
#define LTPIMISCREAD 0x00000000
#define LTPIMISCWRITE 0x00000001
#define LTPICHANNEL0 0x10000
#define LTPICHANNEL1 0x20000
#define LTPICH0ADDRSHOW 0x30000000
#define LTPICH1ADDRSHOW 0x50000000
#define LTPICH0ADDR 0x50000000
#define LTPICH1ADDR 0x60000000

struct ltpi_ipc_data {
	uint32_t misc;
	uintptr_t port;
	uint32_t value;
};

static void ltpi_ipc_cb(const struct device *ipmdev, void *user_data,
			 uint32_t id, volatile void *msg_data)
{
	uint32_t *buf = (uint32_t *)msg_data;
	uint32_t count = buf[0] & 0xff;
	struct ltpi_ipc_data *data = NULL;
	int cpy_size = 0, size = 0;
	uint32_t *handle_buff = NULL;
	uint32_t offset = 0;

	/* check message header */
	if ((buf[0] & LTPISHEADER) != LTPISHEADER) {
		LOG_ERR("msg 0 is invalid 0x%08x\n", buf[0]);
		goto ltpi_cb_fail;
	}

	cpy_size = sizeof(struct ltpi_ipc_data) * count;

	LOG_DBG("id: 0x%x", id);

	/* read the rx buffer into buffer */
	size = ast_ipm_shmem_read(ipmdev, id, 0x0, (void *)&handle_buff, cpy_size);
	if (cpy_size != size) {
		LOG_ERR("ltpi read rx buffer failed.");
		goto ltpi_cb_fail;
	}

	data = (struct ltpi_ipc_data *)handle_buff;

	for (int i = 0; i < count; i++) {
		/* Check the ltpi address setting */
		if ((data->misc & (LTPICHANNEL0 | LTPICHANNEL1)) != 0) {
			offset = data->port & GENMASK(23, 0);

			if (data->misc & LTPICHANNEL0) {
				data->port = (offset | LTPICH0ADDRSHOW);
				offset |= LTPICH0ADDR;
			} else {
				data->port = (offset | LTPICH1ADDRSHOW);
				offset |= LTPICH1ADDR;
			}

			if ((data->misc & LTPIMISCWRITE) == LTPIMISCWRITE) {
				sys_write32(data->value, offset);
			} else {
				data->value = sys_read32(offset);
			}
		}
		data++;
	}

	/* write the tx buffer data into share */
	size = ast_ipm_shmem_write(ipmdev, id, 0x0, (void *)handle_buff, cpy_size);
	if (cpy_size != size) {
		LOG_ERR("ltpi write tx buffer failed.");
		goto ltpi_cb_fail;
	}

ltpi_cb_fail:
}

int ltpi_ipc_enable(void)
{
	char ipc_name[32] = IPC_DEV_NCA35_BOOTMCU;
	int device_id = IPC_CHANNEL_ID_LTPI;
	const struct device *ipmdev;
	int rc = 0;

	LOG_DBG("%s", __func__);

	ipmdev = device_get_binding(ipc_name);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = -1;
		return rc;
	}

	ipm_register_id_callback(ipmdev, device_id, ltpi_ipc_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, 1);
	if (rc) {
		printk("%s: cannot ipm_set_enabled\n", ipc_name);
		return rc;
	}

	ast_ipm_list(ipmdev);

	return 0;
}
