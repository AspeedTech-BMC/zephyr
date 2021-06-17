/*
 * Copyright (c) 2019 Actinius
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_opt3001

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <sys/__assert.h>

#include "opt3001.h"

LOG_MODULE_REGISTER(opt3001, CONFIG_SENSOR_LOG_LEVEL);

static int opt3001_reg_read(struct opt3001_data *drv_data, uint8_t reg,
			    uint16_t *val)
{
	uint8_t value[2];

	if (i2c_burst_read(drv_data->i2c, DT_INST_REG_ADDR(0),
		reg, value, 2) != 0) {
		return -EIO;
	}

	*val = ((uint16_t)value[0] << 8) + value[1];

	return 0;
}

static int opt3001_reg_write(struct opt3001_data *drv_data, uint8_t reg,
			     uint16_t val)
{
	uint8_t new_value[2];

	new_value[0] = val >> 8;
	new_value[1] = val & 0xff;

	uint8_t tx_buf[3] = { reg, new_value[0], new_value[1] };

	return i2c_write(drv_data->i2c, tx_buf, sizeof(tx_buf),
			 DT_INST_REG_ADDR(0));
}

static int opt3001_reg_update(struct opt3001_data *drv_data, uint8_t reg,
			      uint16_t mask, uint16_t val)
{
	uint16_t old_val;
	uint16_t new_val;

	if (opt3001_reg_read(drv_data, reg, &old_val) != 0) {
		return -EIO;
	}

	new_val = old_val & ~mask;
	new_val |= val & mask;

	return opt3001_reg_write(drv_data, reg, new_val);
}

static int opt3001_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct opt3001_data *drv_data = dev->data;
	uint16_t value;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_LIGHT);

	drv_data->sample = 0U;

	if (opt3001_reg_read(drv_data, OPT3001_REG_RESULT, &value) != 0) {
		return -EIO;
	}

	drv_data->sample = value;

	return 0;
}

static int opt3001_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct opt3001_data *drv_data = dev->data;
	int32_t uval;

	if (chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	/**
	 * sample consists of 4 bits of exponent and 12 bits of mantissa
	 * bits 15 to 12 are exponent bits
	 * bits 11 to 0 are the mantissa bits
	 *
	 * lux is the integer obtained using the following formula:
	 * (2^(exponent value)) * 0.01 * mantisa value
	 */
	uval = (1 << (drv_data->sample >> OPT3001_SAMPLE_EXPONENT_SHIFT))
		* (drv_data->sample & OPT3001_MANTISSA_MASK);
	val->val1 = uval / 100;
	val->val2 = (uval % 100) * 10000;

	return 0;
}

static const struct sensor_driver_api opt3001_driver_api = {
	.sample_fetch = opt3001_sample_fetch,
	.channel_get = opt3001_channel_get,
};

static int opt3001_chip_init(const struct device *dev)
{
	struct opt3001_data *drv_data = dev->data;
	uint16_t value;

	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
			DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	if (opt3001_reg_read(drv_data, OPT3001_REG_MANUFACTURER_ID,
		&value) != 0) {
		return -EIO;
	}

	if (value != OPT3001_MANUFACTURER_ID_VALUE) {
		LOG_ERR("Bad manufacturer id 0x%x", value);
		return -ENOTSUP;
	}

	if (opt3001_reg_read(drv_data, OPT3001_REG_DEVICE_ID,
		&value) != 0) {
		return -EIO;
	}

	if (value != OPT3001_DEVICE_ID_VALUE) {
		LOG_ERR("Bad device id 0x%x", value);
		return -ENOTSUP;
	}

	if (opt3001_reg_update(drv_data, OPT3001_REG_CONFIG,
			       OPT3001_CONVERSION_MODE_MASK,
			       OPT3001_CONVERSION_MODE_CONTINUOUS) != 0) {
		LOG_ERR("Failed to set mode to continuous conversion");
		return -EIO;
	}

	return 0;
}

int opt3001_init(const struct device *dev)
{
	if (opt3001_chip_init(dev) < 0) {
		return -EINVAL;
	}

	return 0;
}

static struct opt3001_data opt3001_drv_data;

DEVICE_DT_INST_DEFINE(0, opt3001_init, NULL,
		    &opt3001_drv_data, NULL, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &opt3001_driver_api);
