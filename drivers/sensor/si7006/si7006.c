/*
 * Copyright (c) 2019 Electronut Labs
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_si7006

#include <drivers/sensor.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include "si7006.h"

LOG_MODULE_REGISTER(si7006, CONFIG_SENSOR_LOG_LEVEL);

struct si7006_data {
	const struct device *i2c_dev;
	uint16_t temperature;
	uint16_t humidity;
};

/**
 * @brief function to get relative humidity
 *
 * @return int 0 on success
 */
static int si7006_get_humidity(const struct device *i2c_dev,
			       struct si7006_data *si_data)
{
	int retval;
	uint8_t hum[2];

	retval = i2c_burst_read(i2c_dev, DT_INST_REG_ADDR(0),
		SI7006_MEAS_REL_HUMIDITY_MASTER_MODE, hum, sizeof(hum));

	if (retval == 0) {
		si_data->humidity = (hum[0] << 8) | hum[1];
	} else {
		LOG_ERR("read register err");
	}

	return retval;
}

/**
 * @brief function to get temperature
 *
 * Note that si7006_get_humidity must be called before calling
 * si7006_get_old_temperature.
 *
 * @return int 0 on success
 */

static int si7006_get_old_temperature(const struct device *i2c_dev,
				      struct si7006_data *si_data)
{
	uint8_t temp[2];
	int retval;

	retval = i2c_burst_read(i2c_dev, DT_INST_REG_ADDR(0),
		SI7006_READ_OLD_TEMP, temp, sizeof(temp));

	if (retval == 0) {
		si_data->temperature = (temp[0] << 8) | temp[1];
	} else {
		LOG_ERR("read register err");
	}

	return retval;
}

/**
 * @brief fetch a sample from the sensor
 *
 * @return 0
 */
static int si7006_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	int retval;
	struct si7006_data *si_data = dev->data;

	retval = si7006_get_humidity(si_data->i2c_dev, si_data);
	if (retval == 0) {
		retval = si7006_get_old_temperature(si_data->i2c_dev, si_data);
	}

	return retval;
}

/**
 * @brief sensor value get
 *
 * @return -ENOTSUP for unsupported channels
 */
static int si7006_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct si7006_data *si_data = dev->data;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {

		int32_t temp_ucelcius = (((17572 * (int32_t)si_data->temperature)
					/ 65536) - 4685) * 10000;

		val->val1 = temp_ucelcius / 1000000;
		val->val2 = temp_ucelcius % 1000000;

		LOG_DBG("temperature = val1:%d, val2:%d", val->val1, val->val2);

		return 0;
	} else if (chan == SENSOR_CHAN_HUMIDITY) {

		int32_t relative_humidity = (((125 * (int32_t)si_data->humidity)
					    / 65536) - 6) * 1000000;

		val->val1 = relative_humidity / 1000000;
		val->val2 = relative_humidity % 1000000;

		LOG_DBG("humidity = val1:%d, val2:%d", val->val1, val->val2);

		return 0;
	} else {
		return -ENOTSUP;
	}
}

static const struct sensor_driver_api si7006_api = {
	.sample_fetch = &si7006_sample_fetch,
	.channel_get = &si7006_channel_get,
};

/**
 * @brief initiasize the sensor
 *
 * @return 0 for success
 */

static int si7006_init(const struct device *dev)
{
	struct si7006_data *drv_data = dev->data;

	drv_data->i2c_dev = device_get_binding(
		DT_INST_BUS_LABEL(0));

	if (!drv_data->i2c_dev) {
		LOG_ERR("i2c master not found.");
		return -EINVAL;
	}

	LOG_DBG("si7006 init ok");

	return 0;
}

static struct si7006_data si_data;

DEVICE_DT_INST_DEFINE(0, si7006_init, NULL,
	&si_data, NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &si7006_api);
