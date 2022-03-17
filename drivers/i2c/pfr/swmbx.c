/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_swmbx-ctrl

#include <sys/util.h>
#include <sys/slist.h>
#include <kernel.h>
#include <errno.h>
#include <device.h>
#include <string.h>
#include <drivers/i2c/slave/swmbx.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(swmbx_ctrl);

struct swmbx_ctrl_data {
	const struct device *swmbx_controller;
	uint32_t buffer_size;
	uint8_t *buffer;
	uint8_t mbx_en;
	uint32_t buffer_idx;
	uint8_t mbx_protect[SWMBX_PROTECT_COUNT];
};

struct swmbx_ctrl_config {
	char *controller_dev_name;
	uint32_t buffer_size;
	uintptr_t buffer;
};

/* convenience defines */
#define DEV_CFG(dev)		\
	((const struct swmbx_ctrl_config *const)	\
	(dev)->config)
#define DEV_DATA(dev)	\
	((struct swmbx_ctrl_data *const)(dev)->data)

/* general control */
int swmbx_enable_behavior(const struct device *dev, uint32_t item_flag, uint8_t enable)
{
	if ((dev == NULL) || ((item_flag & FLAG_MASK) == 0))
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;

	if (enable) {
		data->mbx_en |= item_flag;
	} else {
		data->mbx_en &= ~(item_flag);
	}

	return 0;
}

static int swmbx_ctrl_init(const struct device *dev)
{
	struct swmbx_ctrl_data *data = DEV_DATA(dev);
	const struct swmbx_ctrl_config *cfg = DEV_CFG(dev);

	data->swmbx_controller =
		device_get_binding(cfg->controller_dev_name);
	if (!data->swmbx_controller) {
		LOG_ERR("swmbx controller not found: %s",
			    cfg->controller_dev_name);
		return -EINVAL;
	}

	data->buffer = (uint8_t *)(cfg->buffer);
	data->buffer_size = cfg->buffer_size;

	return 0;
}

#define SWMBX_INIT(inst)						\
	static struct swmbx_ctrl_data				\
		swmbx_ctrl_##inst##_dev_data;			\
									\
	static const struct swmbx_ctrl_config			\
		swmbx_ctrl_##inst##_cfg = {			\
		.controller_dev_name = DT_INST_BUS_LABEL(inst),		\
		.buffer = DT_INST_PROP(inst, base),			\
		.buffer_size = DT_INST_PROP(inst, size),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &swmbx_ctrl_init,			\
			    NULL,			\
			    &i2c_swmbx_slave_##inst##_dev_data,	\
			    &i2c_swmbx_slave_##inst##_cfg,		\
			    POST_KERNEL,				\
			    CONFIG_I2C_SLAVE_INIT_PRIORITY,		\
			    NULL);

DT_INST_FOREACH_STATUS_OKAY(SWMBX_INIT)
