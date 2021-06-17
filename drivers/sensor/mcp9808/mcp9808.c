/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_mcp9808

#include <errno.h>

#include <kernel.h>
#include <drivers/i2c.h>
#include <init.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "mcp9808.h"

LOG_MODULE_REGISTER(MCP9808, CONFIG_SENSOR_LOG_LEVEL);

int mcp9808_reg_read(const struct device *dev, uint8_t reg, uint16_t *val)
{
	struct mcp9808_data *data = dev->data;
	const struct mcp9808_config *cfg = dev->config;
	int rc = i2c_write_read(data->i2c_master, cfg->i2c_addr,
				&reg, sizeof(reg),
				val, sizeof(*val));

	if (rc == 0) {
		*val = sys_be16_to_cpu(*val);
	}

	return rc;
}

int mcp9808_reg_write_16bit(const struct device *dev, uint8_t reg,
			    uint16_t val)
{
	struct mcp9808_data *data = dev->data;
	const struct mcp9808_config *cfg = dev->config;

	uint8_t buf[3];

	buf[0] = reg;
	sys_put_be16(val, &buf[1]);

	return i2c_write(data->i2c_master, buf, sizeof(buf), cfg->i2c_addr);
}

int mcp9808_reg_write_8bit(const struct device *dev, uint8_t reg,
			   uint8_t val)
{
	struct mcp9808_data *data = dev->data;
	const struct mcp9808_config *cfg = dev->config;
	uint8_t buf[2] = {
		reg,
		val,
	};

	return i2c_write(data->i2c_master, buf, sizeof(buf), cfg->i2c_addr);
}

static int mcp9808_set_temperature_resolution(const struct device *dev,
					      uint8_t resolution)
{
	return mcp9808_reg_write_8bit(dev, MCP9808_REG_RESOLUTION, resolution);
}

static int mcp9808_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct mcp9808_data *data = dev->data;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_AMBIENT_TEMP);

	return mcp9808_reg_read(dev, MCP9808_REG_TEMP_AMB, &data->reg_val);
}

static int mcp9808_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	const struct mcp9808_data *data = dev->data;
	int temp = mcp9808_temp_signed_from_reg(data->reg_val);

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP);

	val->val1 = temp / MCP9808_TEMP_SCALE_CEL;
	temp -= val->val1 * MCP9808_TEMP_SCALE_CEL;
	val->val2 = (temp * 1000000) / MCP9808_TEMP_SCALE_CEL;

	return 0;
}

static const struct sensor_driver_api mcp9808_api_funcs = {
	.sample_fetch = mcp9808_sample_fetch,
	.channel_get = mcp9808_channel_get,
#ifdef CONFIG_MCP9808_TRIGGER
	.attr_set = mcp9808_attr_set,
	.trigger_set = mcp9808_trigger_set,
#endif /* CONFIG_MCP9808_TRIGGER */
};

int mcp9808_init(const struct device *dev)
{
	struct mcp9808_data *data = dev->data;
	const struct mcp9808_config *cfg = dev->config;
	int rc = 0;

	data->i2c_master = device_get_binding(cfg->i2c_bus);
	if (!data->i2c_master) {
		LOG_ERR("mcp9808: i2c master not found: %s", cfg->i2c_bus);
		return -EINVAL;
	}

	rc = mcp9808_set_temperature_resolution(dev, cfg->resolution);
	if (rc) {
		LOG_ERR("Could not set the resolution of mcp9808 module");
		return rc;
	}

#ifdef CONFIG_MCP9808_TRIGGER
	rc = mcp9808_setup_interrupt(dev);
#endif /* CONFIG_MCP9808_TRIGGER */

	return rc;
}

static struct mcp9808_data mcp9808_data;
static const struct mcp9808_config mcp9808_cfg = {
	.i2c_bus = DT_INST_BUS_LABEL(0),
	.i2c_addr = DT_INST_REG_ADDR(0),
	.resolution = DT_INST_PROP(0, resolution),
#ifdef CONFIG_MCP9808_TRIGGER
	.alert_pin = DT_INST_GPIO_PIN(0, int_gpios),
	.alert_flags = DT_INST_GPIO_FLAGS(0, int_gpios),
	.alert_controller = DT_INST_GPIO_LABEL(0, int_gpios),
#endif /* CONFIG_MCP9808_TRIGGER */
};

DEVICE_DT_INST_DEFINE(0, mcp9808_init, NULL,
		      &mcp9808_data, &mcp9808_cfg, POST_KERNEL,
		      CONFIG_SENSOR_INIT_PRIORITY, &mcp9808_api_funcs);
