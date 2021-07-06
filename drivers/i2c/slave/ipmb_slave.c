/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
*/
#define DT_DRV_COMPAT aspeed_ipmb

#include <sys/util.h>
#include <sys/slist.h>
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
	sys_snode_t list;
	uint8_t msg_length;
	struct ipmb_msg msg;
};

struct i2c_ipmb_slave_data {
	const struct device *i2c_controller;
	struct i2c_slave_config config;
	sys_slist_t list_head;
	struct ipmb_msg_package *buffer;
	uint32_t buffer_idx;
	uint32_t max_msg_count;
	uint32_t cur_msg_count;
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

	/* check the max msg length */
	if (data->cur_msg_count < data->max_msg_count) {

		data->buffer = malloc(sizeof(struct ipmb_msg_package));

		LOG_DBG("ipmb: slave write data->buffer %x", (uint32_t)(data->buffer));

		if (data->buffer != NULL) {
			sys_slist_append(&(data->list_head), &(data->buffer->list));
			data->cur_msg_count++;
		} else {
			return 1;
		}
	} else {
		LOG_ERR("ipmb: buffer full");
		data->buffer = NULL;
		return 1;
	}

	uint8_t *buf = (uint8_t *)(&(data->buffer->msg));

	LOG_DBG("ipmb: write req");

	/*fill data into ipmb buffer*/
	data->buffer->msg_length = 0;
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

	if (data->buffer != NULL) {

		uint8_t *buf = (uint8_t *)(&(data->buffer->msg));

		if (data->buffer_idx >= sizeof(struct ipmb_msg)) {
			return 1;
		}

		LOG_DBG("ipmb: write received, val=0x%x", val);

		/* fill data */
		buf[data->buffer_idx++] = val;
		return 0;
	} else {
		return 1;
	}
}

static int ipmb_slave_stop(struct i2c_slave_config *config)
{
	struct i2c_ipmb_slave_data *data = CONTAINER_OF(config,
							struct i2c_ipmb_slave_data,
							config);

	if (data->buffer != NULL) {

		data->buffer->msg_length = data->buffer_idx;

		LOG_DBG("ipmb: stop");

		return 0;
	}

	return 1;
}

int ipmb_slave_read(const struct device *dev, struct ipmb_msg **ipmb_data, uint8_t *length)
{
	struct i2c_ipmb_slave_data *data = DEV_DATA(dev);
	sys_snode_t *list_node = NULL;
	struct ipmb_msg_package *pack = NULL;

	list_node = sys_slist_peek_head(&(data->list_head));

	LOG_DBG("ipmb: slave read %x", (uint32_t)list_node);

	if (list_node != NULL) {
		pack = (struct ipmb_msg_package *)(list_node);
		*ipmb_data = &(pack->msg);
		*length = pack->msg_length;
		return 0;
	} else {
		LOG_ERR("ipmb slave read: buffer empty!");
		return 1;
	}

	return 0;
}

int ipmb_slave_remove(const struct device *dev)
{
	struct i2c_ipmb_slave_data *data = DEV_DATA(dev);
	sys_snode_t *list_node = NULL;

	list_node = sys_slist_peek_head(&(data->list_head));

	LOG_DBG("ipmb: slave remove %x", (uint32_t)list_node);

	if (list_node != NULL) {
		/* remove this item from list */
		sys_slist_find_and_remove(&(data->list_head), list_node);
		data->cur_msg_count--;
		free(list_node);

		LOG_DBG("ipmb: slave remove successful.");

		return 0;
	} else {
		return 1;
	}

	return 0;
}


static int ipmb_slave_register(const struct device *dev)
{
	struct i2c_ipmb_slave_data *data = dev->data;

	return i2c_slave_register(data->i2c_controller, &data->config);
}

static int ipmb_slave_unregister(const struct device *dev)
{
	struct i2c_ipmb_slave_data *data = dev->data;
	sys_snode_t *list_node = NULL;

	/* free malloc package */
	do {
		list_node = sys_slist_peek_head(&(data->list_head));

		if (list_node != NULL) {
			/* remove this item from list */
			sys_slist_find_and_remove(&(data->list_head), list_node);
			data->cur_msg_count--;
			free(list_node);
		}

	} while (list_node != NULL);

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

	/* initial single list structure*/
	sys_slist_init(&(data->list_head));

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
