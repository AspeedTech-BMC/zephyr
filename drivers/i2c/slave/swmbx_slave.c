/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
*/
#define DT_DRV_COMPAT aspeed_swmbx_dev

#include <sys/util.h>
#include <kernel.h>
#include <errno.h>
#include <drivers/i2c.h>
#include <string.h>
#include <drivers/i2c/pfr/swmbx.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_slave_swmbx);

struct i2c_swmbx_slave_data {
	const struct device *i2c_controller;
	struct i2c_slave_config config;
	uint32_t buffer_size;
	uint8_t port;
	uint32_t buffer_idx;
	bool first_write;
};

struct i2c_swmbx_slave_config {
	char *controller_dev_name;
	uint8_t address;
	uint32_t buffer_size;
	uint8_t port;
};

/* convenience defines */
#define DEV_CFG(dev)							\
	((const struct i2c_swmbx_slave_config * const)			\
		(dev)->config)
#define DEV_DATA(dev)							\
	((struct i2c_swmbx_slave_data * const)(dev)->data)

static int swmbx_slave_write_requested(struct i2c_slave_config *config)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	LOG_DBG("swmbx: write req");

	data->first_write = true;

	return 0;
}

static int swmbx_slave_read_requested(struct i2c_slave_config *config,
				       uint8_t *val)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	/* send swmbx get message */
	swmbx_get_msg(data->port, data->buffer_idx, val);

	LOG_DBG("swmbx: read req, val=0x%x", *val);

	return 0;
}

static int swmbx_slave_write_received(struct i2c_slave_config *config,
				       uint8_t val)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	LOG_DBG("swmbx: write done, val=0x%x", val);

	if (data->first_write) {
		data->buffer_idx = val;
		data->first_write = false;

		/* send swmbx start message */
		swmbx_send_start(data->port, data->buffer_idx);
	} else {
		/* send swmbx send message */
		swmbx_send_msg(data->port, data->buffer_idx++, &val);
	}

	data->buffer_idx = data->buffer_idx % data->buffer_size;

	return 0;
}

static int swmbx_slave_read_processed(struct i2c_slave_config *config,
				       uint8_t *val)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	/* Increment here */
	data->buffer_idx = (data->buffer_idx + 1) % data->buffer_size;

	/* send swmbx get message */
	swmbx_get_msg(data->port, data->buffer_idx, val);

	LOG_DBG("swmbx: read done, val=0x%x", *val);

	return 0;
}

static int swmbx_slave_stop(struct i2c_slave_config *config)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	LOG_DBG("swmbx: stop");

	/* send swmbx stop message */
	swmbx_send_stop(data->port);

	data->first_write = true;

	return 0;
}

static int swmbx_slave_register(const struct device *dev)
{
	struct i2c_swmbx_slave_data *data = dev->data;

	return i2c_slave_register(data->i2c_controller, &data->config);
}

static int swmbx_slave_unregister(const struct device *dev)
{
	struct i2c_swmbx_slave_data *data = dev->data;

	return i2c_slave_unregister(data->i2c_controller, &data->config);
}

static const struct i2c_slave_driver_api api_swmbx_funcs = {
	.driver_register = swmbx_slave_register,
	.driver_unregister = swmbx_slave_unregister,
};

static const struct i2c_slave_callbacks swmbx_callbacks = {
	.write_requested = swmbx_slave_write_requested,
	.read_requested = swmbx_slave_read_requested,
	.write_received = swmbx_slave_write_received,
	.read_processed = swmbx_slave_read_processed,
	.stop = swmbx_slave_stop,
};

static int i2c_swmbx_slave_init(const struct device *dev)
{
	struct i2c_swmbx_slave_data *data = DEV_DATA(dev);
	const struct i2c_swmbx_slave_config *cfg = DEV_CFG(dev);

	data->i2c_controller =
		device_get_binding(cfg->controller_dev_name);
	if (!data->i2c_controller) {
		LOG_ERR("i2c controller not found: %s",
			    cfg->controller_dev_name);
		return -EINVAL;
	}

	data->buffer_size = cfg->buffer_size;
	data->port = cfg->port;
	data->config.address = cfg->address;
	data->config.callbacks = &swmbx_callbacks;

	return 0;
}

#define I2C_SWMBX_INIT(inst)						\
	static struct i2c_swmbx_slave_data				\
		i2c_swmbx_slave_##inst##_dev_data;			\
									\
	static const struct i2c_swmbx_slave_config			\
		i2c_swmbx_slave_##inst##_cfg = {			\
		.controller_dev_name = DT_INST_BUS_LABEL(inst),		\
		.address = DT_INST_REG_ADDR(inst),			\
		.buffer_size = DT_INST_PROP(inst, size),		\
		.port = DT_INST_PROP(inst, port),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &i2c_swmbx_slave_init,			\
			    NULL,			\
			    &i2c_swmbx_slave_##inst##_dev_data,	\
			    &i2c_swmbx_slave_##inst##_cfg,		\
			    POST_KERNEL,				\
			    CONFIG_I2C_SLAVE_INIT_PRIORITY,		\
			    &api_swmbx_funcs);

DT_INST_FOREACH_STATUS_OKAY(I2C_SWMBX_INIT)
