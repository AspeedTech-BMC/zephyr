/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
*/
#define DT_DRV_COMPAT aspeed_ipmb

#include <sys/util.h>
#include <kernel.h>
#include <errno.h>
#include <drivers/i2c.h>
#include <string.h>
#include <stdlib.h>
#include <drivers/i2c/slave/ipmb.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_slave_ipmb);

struct ipmb_msg_package {
	uint8_t msg_length;
	struct ipmb_msg msg;
};

struct i2c_ipmb_slave_data {
	const struct device *i2c_controller;
	struct i2c_slave_config config;
	struct ipmb_msg_package *buffer;
	struct ipmb_msg_package *current;
	uint32_t buffer_idx;
	uint32_t max_msg_count;
	uint32_t cur_read_count;
	uint32_t cur_write_count;
};

struct i2c_ipmb_slave_config {
	char *controller_dev_name;
	uint8_t address;
	uint32_t ipmb_msg_length;
};

/* convenience defines */
#define DEV_CFG(dev)				     \
	((const struct i2c_ipmb_slave_config *const) \
	 (dev)->config)
#define DEV_DATA(dev) \
	((struct i2c_ipmb_slave_data *const)(dev)->data)

static int ipmb_slave_write_requested(struct i2c_slave_config *config)
{
	struct i2c_ipmb_slave_data *data = CONTAINER_OF(config,
							struct i2c_ipmb_slave_data,
							config);
	uint32_t cur_write_next = 0;

	/* bondary condition */
	if (data->cur_write_count == data->max_msg_count) {
		cur_write_next = 0;
	} else {
		cur_write_next = data->cur_write_count + 1;
	}

	/* check buffer full or not (next != R ) */
	if (cur_write_next != data->cur_read_count) {
		/* assign free buffer */
		data->current =
		&(data->buffer[data->cur_write_count]);
		LOG_DBG("ipmb: slave write data->buffer %x",
		(uint32_t)(data->current));
		data->cur_write_count = cur_write_next;
	} else {
		data->current = NULL;
		LOG_DBG("ipmb: buffer full");
		return 1;
	}

	uint8_t *buf = (uint8_t *)(&(data->current->msg));

	LOG_DBG("ipmb: write req");

	/*fill data into ipmb buffer*/
	data->current->msg_length = 0;
	memset(buf, 0x0, sizeof(struct ipmb_msg));
	data->buffer_idx = 0;

	/*skip the first length parameter*/
	buf[data->buffer_idx++] = GET_ADDR(config->address);

	return 0;
}


static int ipmb_slave_write_received(struct i2c_slave_config *config,
				     uint8_t val)
{
	struct i2c_ipmb_slave_data *data = CONTAINER_OF(config,
							struct i2c_ipmb_slave_data,
							config);
	uint8_t *buf;

	if (data->current) {
		buf = (uint8_t *)(&(data->current->msg));

		if (data->buffer_idx >= sizeof(struct ipmb_msg)) {
			return 1;
		}

		LOG_DBG("ipmb: write received, val=0x%x", val);

		/* fill data */
		buf[data->buffer_idx++] = val;
	}

	return 0;
}

static int ipmb_slave_stop(struct i2c_slave_config *config)
{
	struct i2c_ipmb_slave_data *data = CONTAINER_OF(config,
							struct i2c_ipmb_slave_data,
							config);

	if (data->current) {
		data->current->msg_length = data->buffer_idx;
		LOG_DBG("ipmb: stop");
	}

	return 0;
}

int ipmb_slave_read(const struct device *dev, struct ipmb_msg **ipmb_data, uint8_t *length)
{
	struct i2c_ipmb_slave_data *data = DEV_DATA(dev);
	struct ipmb_msg_package *local_buf = NULL;
	uint32_t cur_read_next = 0;
	uint32_t cur_write_next = 0;

	/* bondary condition */
	if (data->cur_read_count == data->max_msg_count) {
		cur_read_next = 0;
	} else {
		cur_read_next = data->cur_read_count + 1;
	}

	if (data->cur_write_count == data->max_msg_count) {
		cur_write_next = 0;
	} else {
		cur_write_next = data->cur_write_count + 1;
	}

	/* check buffer full or not (next R != next W) */
	if (cur_read_next != cur_write_next) {
		local_buf =
		&(data->buffer[data->cur_read_count]);

		LOG_DBG("ipmb: slave read %x", (uint32_t)local_buf);

		*ipmb_data = &(local_buf->msg);
		*length = local_buf->msg_length;

		data->cur_read_count = cur_read_next;
		return 0;
	} else {
		LOG_DBG("ipmb slave read: buffer empty!");
		return 1;
	}

	return 0;
}

static int ipmb_slave_register(const struct device *dev)
{
	struct i2c_ipmb_slave_data *data = dev->data;

	/* initial r/w pointer */
	data->cur_read_count = 0;
	data->cur_write_count = 0;

	return i2c_slave_register(data->i2c_controller, &data->config);
}

static int ipmb_slave_unregister(const struct device *dev)
{
	struct i2c_ipmb_slave_data *data = dev->data;

	return i2c_slave_unregister(data->i2c_controller, &data->config);
}

static const struct i2c_slave_driver_api api_ipmb_funcs = {
	.driver_register = ipmb_slave_register,
	.driver_unregister = ipmb_slave_unregister,
};

static const struct i2c_slave_callbacks ipmb_callbacks = {
	.write_requested = ipmb_slave_write_requested,
	.read_requested = NULL,
	.write_received = ipmb_slave_write_received,
	.read_processed = NULL,
	.stop = ipmb_slave_stop,
};

static int i2c_ipmb_slave_init(const struct device *dev)
{
	struct i2c_ipmb_slave_data *data = DEV_DATA(dev);
	const struct i2c_ipmb_slave_config *cfg = DEV_CFG(dev);

	if (!cfg->ipmb_msg_length) {
		LOG_ERR("i2c ipmb buffer size is zero");
		return -EINVAL;
	}

	data->i2c_controller =
		device_get_binding(cfg->controller_dev_name);
	if (!data->i2c_controller) {
		LOG_ERR("i2c controller not found: %s",
			cfg->controller_dev_name);
		return -EINVAL;
	}

	data->max_msg_count = cfg->ipmb_msg_length;
	data->config.address = cfg->address;
	data->config.callbacks = &ipmb_callbacks;

	LOG_DBG("i2c ipmb length %d", data->max_msg_count);
	LOG_DBG("i2c ipmb size %x", sizeof(struct ipmb_msg_package) * (data->max_msg_count));

	/* malloc the message buffer */
	data->buffer = k_malloc(sizeof(struct ipmb_msg_package) * (data->max_msg_count));
	if (!data->buffer) {
		LOG_ERR("i2c could not alloc enougth messgae queue");
		return -EINVAL;
	}

	return 0;
}

#define I2C_IPMB_INIT(inst)					 \
	static struct i2c_ipmb_slave_data			 \
		i2c_ipmb_slave_##inst##_dev_data;		 \
								 \
								 \
	static const struct i2c_ipmb_slave_config		 \
		i2c_ipmb_slave_##inst##_cfg = {			 \
		.controller_dev_name = DT_INST_BUS_LABEL(inst),	 \
		.address = DT_INST_REG_ADDR(inst),		 \
		.ipmb_msg_length = DT_INST_PROP(inst, size),	 \
	};							 \
								 \
	DEVICE_DT_INST_DEFINE(inst,				 \
			      &i2c_ipmb_slave_init,		 \
			      NULL,				 \
			      &i2c_ipmb_slave_##inst##_dev_data, \
			      &i2c_ipmb_slave_##inst##_cfg,	 \
			      POST_KERNEL,			 \
			      CONFIG_I2C_SLAVE_INIT_PRIORITY,	 \
			      &api_ipmb_funcs);

DT_INST_FOREACH_STATUS_OKAY(I2C_IPMB_INIT)
