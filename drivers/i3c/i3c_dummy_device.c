/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT i3c_dummy_device

#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i3c.h>

#include <zephyr/logging/log.h>
#define LOG_MODULE_NAME				i3c_dummy_device
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_I3C_ASPEED_LOG_LEVEL);

#define IBI_MDB_GROUP				GENMASK(7, 5)
#define   IBI_MDB_GROUP_PENDING_READ_NOTI	5
#define IBI_MDB_SPEC_ID				GENMASK(4, 0)

#define MAX_READ_DATA_LEN			64

struct i3c_dummy_device_config {
	const struct device *bus;
	const uint64_t pid;
	uint16_t max_ibi_payload;
};

struct i3c_dummy_device_data {
	struct i3c_device_desc *desc;
	struct k_work work;
	uint8_t buf[MAX_READ_DATA_LEN];
};

static void i3c_dummy_device_work(struct k_work *work)
{
	struct i3c_dummy_device_data *data =
		CONTAINER_OF(work, struct i3c_dummy_device_data, work);
	struct i3c_msg msg;

	msg.buf = data->buf;
	msg.len = MAX_READ_DATA_LEN;
	msg.flags = I3C_MSG_READ | I3C_MSG_STOP;
	i3c_transfer(data->desc, &msg, 1);
	LOG_HEXDUMP_INF(msg.buf, msg.len, "Pending read data:");
}

static int i3c_dummy_device_ibi_cb(struct i3c_device_desc *target, struct i3c_ibi_payload *payload)
{
	const struct device *dev = target->dev;
	struct i3c_dummy_device_data *data = dev->data;

	if (payload->payload_len) {
		LOG_HEXDUMP_INF(payload->payload, payload->payload_len, "IBI payload:");

		if (FIELD_GET(IBI_MDB_GROUP, payload->payload[0]) ==
		    IBI_MDB_GROUP_PENDING_READ_NOTI) {
			k_work_submit(&data->work);
		}
	}

	return 0;
}

static int i3c_dummy_device_init(const struct device *dev)
{
	const struct i3c_dummy_device_config *config = dev->config;
	struct i3c_dummy_device_data *data = dev->data;
	uint64_t pid = config->pid;
	const struct i3c_device_id i3c_id = I3C_DEVICE_ID(pid);

	LOG_INF("%s (%012llx) belongs to bus %s", dev->name, config->pid, config->bus->name);

	data->desc = i3c_device_find(config->bus, &i3c_id);
	if (!data->desc) {
		LOG_ERR("%s: target descriptor not found on %s", dev->name, config->bus->name);
		return -ENODEV;
	}

	/*
	 * Wire up the IBI callback AND the maximum IBI payload size BEFORE
	 * enabling IBI. The MIPI I3C HCI controller commits the DAT entry's
	 * IBI_PAYLOAD bit and the per-device IBI accounting size from these
	 * fields at i3c_ibi_enable() time; with max_ibi == 0 the controller
	 * rejects every IBI byte the target emits (including the MDB byte
	 * raised by pending-read-notify drivers like i3c-target-mqueue).
	 */
	data->desc->ibi_cb = i3c_dummy_device_ibi_cb;
	data->desc->data_length.max_ibi = config->max_ibi_payload;
	k_work_init(&data->work, i3c_dummy_device_work);

	return i3c_ibi_enable(data->desc);
}

#define I3C_DEVICE_ASPEED_INIT(n)                                                                  \
	static const struct i3c_dummy_device_config i3c_dummy_device_config_##n = {              \
		.bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
		.pid = ((uint64_t)DT_PROP_BY_IDX(DT_DRV_INST(n), reg, 1) << 32) |                  \
		       DT_PROP_BY_IDX(DT_DRV_INST(n), reg, 2),                                     \
		.max_ibi_payload = DT_INST_PROP(n, max_ibi_payload),                               \
	};                                                                                         \
                                                                                                   \
	static struct i3c_dummy_device_data i3c_dummy_device_data_##n;                           \
	DEVICE_DT_INST_DEFINE(n, i3c_dummy_device_init, NULL, &i3c_dummy_device_data_##n,        \
			      &i3c_dummy_device_config_##n, POST_KERNEL, 51, NULL);

DT_INST_FOREACH_STATUS_OKAY(I3C_DEVICE_ASPEED_INIT)
