/* bme280.c - Driver for Bosch BME280 temperature and pressure sensor */

/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>

#include <logging/log.h>

#include "bme280.h"

LOG_MODULE_REGISTER(BME280, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "BME280 driver enabled without any devices"
#endif

struct bme280_data {
	/* Compensation parameters. */
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	uint8_t dig_h1;
	int16_t dig_h2;
	uint8_t dig_h3;
	int16_t dig_h4;
	int16_t dig_h5;
	int8_t dig_h6;

	/* Compensated values. */
	int32_t comp_temp;
	uint32_t comp_press;
	uint32_t comp_humidity;

	/* Carryover between temperature and pressure/humidity compensation. */
	int32_t t_fine;

	uint8_t chip_id;

#ifdef CONFIG_PM_DEVICE
	uint32_t pm_state; /* Current power state */
#endif
};

struct bme280_config {
	const struct device *bus;
	const struct bme280_bus_io *bus_io;
	const union bme280_bus_config bus_config;
};

static inline struct bme280_data *to_data(const struct device *dev)
{
	return dev->data;
}

static inline const struct bme280_config *to_config(const struct device *dev)
{
	return dev->config;
}

static inline const struct device *to_bus(const struct device *dev)
{
	return to_config(dev)->bus;
}

static inline const union bme280_bus_config*
to_bus_config(const struct device *dev)
{
	return &to_config(dev)->bus_config;
}

static inline int bme280_bus_check(const struct device *dev)
{
	return to_config(dev)->bus_io->check(to_bus(dev), to_bus_config(dev));
}

static inline int bme280_reg_read(const struct device *dev,
				  uint8_t start, uint8_t *buf, int size)
{
	return to_config(dev)->bus_io->read(to_bus(dev), to_bus_config(dev),
					    start, buf, size);
}

static inline int bme280_reg_write(const struct device *dev, uint8_t reg,
				   uint8_t val)
{
	return to_config(dev)->bus_io->write(to_bus(dev), to_bus_config(dev),
					     reg, val);
}

/*
 * Compensation code taken from BME280 datasheet, Section 4.2.3
 * "Compensation formula".
 */
static void bme280_compensate_temp(struct bme280_data *data, int32_t adc_temp)
{
	int32_t var1, var2;

	var1 = (((adc_temp >> 3) - ((int32_t)data->dig_t1 << 1)) *
		((int32_t)data->dig_t2)) >> 11;
	var2 = (((((adc_temp >> 4) - ((int32_t)data->dig_t1)) *
		  ((adc_temp >> 4) - ((int32_t)data->dig_t1))) >> 12) *
		((int32_t)data->dig_t3)) >> 14;

	data->t_fine = var1 + var2;
	data->comp_temp = (data->t_fine * 5 + 128) >> 8;
}

static void bme280_compensate_press(struct bme280_data *data, int32_t adc_press)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)data->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)data->dig_p6;
	var2 = var2 + ((var1 * (int64_t)data->dig_p5) << 17);
	var2 = var2 + (((int64_t)data->dig_p4) << 35);
	var1 = ((var1 * var1 * (int64_t)data->dig_p3) >> 8) +
		((var1 * (int64_t)data->dig_p2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)data->dig_p1) >> 33;

	/* Avoid exception caused by division by zero. */
	if (var1 == 0) {
		data->comp_press = 0U;
		return;
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)data->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)data->dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)data->dig_p7) << 4);

	data->comp_press = (uint32_t)p;
}

static void bme280_compensate_humidity(struct bme280_data *data,
				       int32_t adc_humidity)
{
	int32_t h;

	h = (data->t_fine - ((int32_t)76800));
	h = ((((adc_humidity << 14) - (((int32_t)data->dig_h4) << 20) -
		(((int32_t)data->dig_h5) * h)) + ((int32_t)16384)) >> 15) *
		(((((((h * ((int32_t)data->dig_h6)) >> 10) * (((h *
		((int32_t)data->dig_h3)) >> 11) + ((int32_t)32768))) >> 10) +
		((int32_t)2097152)) * ((int32_t)data->dig_h2) + 8192) >> 14);
	h = (h - (((((h >> 15) * (h >> 15)) >> 7) *
		((int32_t)data->dig_h1)) >> 4));
	h = (h > 419430400 ? 419430400 : h);

	data->comp_humidity = (uint32_t)(h >> 12);
}

static int bme280_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct bme280_data *data = to_data(dev);
	uint8_t buf[8];
	int32_t adc_press, adc_temp, adc_humidity;
	int size = 6;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

#ifdef CONFIG_PM_DEVICE
	/* Do not allow sample fetching from OFF state */
	if (data->pm_state == PM_DEVICE_STATE_OFF)
		return -EIO;
#endif

#ifdef CONFIG_BME280_MODE_FORCED
	ret = bme280_reg_write(dev, BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_VAL);
	if (ret < 0) {
		return ret;
	}

	do {
		k_sleep(K_MSEC(3));
		ret = bme280_reg_read(dev, BME280_REG_STATUS, buf, 1);
		if (ret < 0) {
			return ret;
		}
	} while (buf[0] & 0x08);
#endif

	if (data->chip_id == BME280_CHIP_ID) {
		size = 8;
	}
	ret = bme280_reg_read(dev, BME280_REG_PRESS_MSB, buf, size);
	if (ret < 0) {
		return ret;
	}

	adc_press = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	adc_temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

	bme280_compensate_temp(data, adc_temp);
	bme280_compensate_press(data, adc_press);

	if (data->chip_id == BME280_CHIP_ID) {
		adc_humidity = (buf[6] << 8) | buf[7];
		bme280_compensate_humidity(data, adc_humidity);
	}

	return 0;
}

static int bme280_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bme280_data *data = to_data(dev);

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/*
		 * data->comp_temp has a resolution of 0.01 degC.  So
		 * 5123 equals 51.23 degC.
		 */
		val->val1 = data->comp_temp / 100;
		val->val2 = data->comp_temp % 100 * 10000;
		break;
	case SENSOR_CHAN_PRESS:
		/*
		 * data->comp_press has 24 integer bits and 8
		 * fractional.  Output value of 24674867 represents
		 * 24674867/256 = 96386.2 Pa = 963.862 hPa
		 */
		val->val1 = (data->comp_press >> 8) / 1000U;
		val->val2 = (data->comp_press >> 8) % 1000 * 1000U +
			(((data->comp_press & 0xff) * 1000U) >> 8);
		break;
	case SENSOR_CHAN_HUMIDITY:
		/*
		 * data->comp_humidity has 22 integer bits and 10
		 * fractional.  Output value of 47445 represents
		 * 47445/1024 = 46.333 %RH
		 */
		val->val1 = (data->comp_humidity >> 10);
		val->val2 = (((data->comp_humidity & 0x3ff) * 1000U * 1000U) >> 10);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api bme280_api_funcs = {
	.sample_fetch = bme280_sample_fetch,
	.channel_get = bme280_channel_get,
};

static int bme280_read_compensation(const struct device *dev)
{
	struct bme280_data *data = to_data(dev);
	uint16_t buf[12];
	uint8_t hbuf[7];
	int err = 0;

	err = bme280_reg_read(dev, BME280_REG_COMP_START,
			      (uint8_t *)buf, sizeof(buf));

	if (err < 0) {
		LOG_DBG("COMP_START read failed: %d", err);
		return err;
	}

	data->dig_t1 = sys_le16_to_cpu(buf[0]);
	data->dig_t2 = sys_le16_to_cpu(buf[1]);
	data->dig_t3 = sys_le16_to_cpu(buf[2]);

	data->dig_p1 = sys_le16_to_cpu(buf[3]);
	data->dig_p2 = sys_le16_to_cpu(buf[4]);
	data->dig_p3 = sys_le16_to_cpu(buf[5]);
	data->dig_p4 = sys_le16_to_cpu(buf[6]);
	data->dig_p5 = sys_le16_to_cpu(buf[7]);
	data->dig_p6 = sys_le16_to_cpu(buf[8]);
	data->dig_p7 = sys_le16_to_cpu(buf[9]);
	data->dig_p8 = sys_le16_to_cpu(buf[10]);
	data->dig_p9 = sys_le16_to_cpu(buf[11]);

	if (data->chip_id == BME280_CHIP_ID) {
		err = bme280_reg_read(dev, BME280_REG_HUM_COMP_PART1,
				      &data->dig_h1, 1);
		if (err < 0) {
			LOG_DBG("HUM_COMP_PART1 read failed: %d", err);
			return err;
		}

		err = bme280_reg_read(dev, BME280_REG_HUM_COMP_PART2, hbuf, 7);
		if (err < 0) {
			LOG_DBG("HUM_COMP_PART2 read failed: %d", err);
			return err;
		}

		data->dig_h2 = (hbuf[1] << 8) | hbuf[0];
		data->dig_h3 = hbuf[2];
		data->dig_h4 = (hbuf[3] << 4) | (hbuf[4] & 0x0F);
		data->dig_h5 = ((hbuf[4] >> 4) & 0x0F) | (hbuf[5] << 4);
		data->dig_h6 = hbuf[6];
	}

	return 0;
}

static int bme280_chip_init(const struct device *dev)
{
	struct bme280_data *data = to_data(dev);
	int err;

	LOG_DBG("initializing \"%s\" on bus \"%s\"",
		dev->name, to_bus(dev)->name);

	err = bme280_bus_check(dev);
	if (err < 0) {
		LOG_DBG("bus check failed: %d", err);
		return err;
	}

	err = bme280_reg_read(dev, BME280_REG_ID, &data->chip_id, 1);
	if (err < 0) {
		LOG_DBG("ID read failed: %d", err);
		return err;
	}

	if (data->chip_id == BME280_CHIP_ID) {
		LOG_DBG("ID OK");
	} else if (data->chip_id == BMP280_CHIP_ID_MP ||
		   data->chip_id == BMP280_CHIP_ID_SAMPLE_1) {
		LOG_DBG("ID OK (BMP280)");
	} else {
		LOG_DBG("bad chip id 0x%x", data->chip_id);
		return -ENOTSUP;
	}

	err = bme280_read_compensation(dev);
	if (err < 0) {
		return err;
	}

	if (data->chip_id == BME280_CHIP_ID) {
		err = bme280_reg_write(dev, BME280_REG_CTRL_HUM,
				       BME280_HUMIDITY_OVER);
		if (err < 0) {
			LOG_DBG("CTRL_HUM write failed: %d", err);
			return err;
		}
	}

	err = bme280_reg_write(dev, BME280_REG_CTRL_MEAS,
			       BME280_CTRL_MEAS_VAL);
	if (err < 0) {
		LOG_DBG("CTRL_MEAS write failed: %d", err);
		return err;
	}

	err = bme280_reg_write(dev, BME280_REG_CONFIG,
			       BME280_CONFIG_VAL);
	if (err < 0) {
		LOG_DBG("CONFIG write failed: %d", err);
		return err;
	}

#ifdef CONFIG_PM_DEVICE
	/* Set power state to ACTIVE */
	data->pm_state = PM_DEVICE_STATE_ACTIVE;
#endif
	LOG_DBG("\"%s\" OK", dev->name);
	return 0;
}

#ifdef CONFIG_PM_DEVICE
int bme280_pm_ctrl(const struct device *dev, uint32_t ctrl_command,
		   uint32_t *state, pm_device_cb cb, void *arg)
{
	struct bme280_data *data = to_data(dev);

	int ret = 0;

	/* Set power state */
	if (ctrl_command == PM_DEVICE_STATE_SET) {
		uint32_t new_pm_state = *state;

		if (new_pm_state != data->pm_state) {

			/* Switching from OFF to any */
			if (data->pm_state == PM_DEVICE_STATE_OFF) {

				/* Re-initialize the chip */
				ret = bme280_chip_init(dev);
			}
			/* Switching to OFF from any */
			else if (new_pm_state == PM_DEVICE_STATE_OFF) {

				/* Put the chip into sleep mode */
				ret = bme280_reg_write(dev,
					BME280_REG_CTRL_MEAS,
					BME280_CTRL_MEAS_OFF_VAL);

				if (ret < 0)
					LOG_DBG("CTRL_MEAS write failed: %d",
						ret);
			}

			/* Store the new state */
			if (!ret)
				data->pm_state = new_pm_state;
		}
	}
	/* Get power state */
	else {
		__ASSERT_NO_MSG(ctrl_command == PM_DEVICE_STATE_GET);
		*state = data->pm_state;
	}

	/* Invoke callback if any */
	if (cb)
		cb(dev, ret, state, arg);

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* Initializes a struct bme280_config for an instance on a SPI bus. */
#define BME280_CONFIG_SPI(inst)						\
	{								\
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),		\
		.bus_io = &bme280_bus_io_spi,				\
		.bus_config.spi_cfg =					\
			SPI_CONFIG_DT_INST(inst,			\
					   BME280_SPI_OPERATION,	\
					   0),				\
	}

/* Initializes a struct bme280_config for an instance on an I2C bus. */
#define BME280_CONFIG_I2C(inst)						\
	{								\
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),		\
		.bus_io = &bme280_bus_io_i2c,				\
		.bus_config.i2c_addr = DT_INST_REG_ADDR(inst),		\
	}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define BME280_DEFINE(inst)						\
	static struct bme280_data bme280_data_##inst;			\
	static const struct bme280_config bme280_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			    (BME280_CONFIG_SPI(inst)),			\
			    (BME280_CONFIG_I2C(inst)));			\
	DEVICE_DT_INST_DEFINE(inst,					\
			 bme280_chip_init,				\
			 bme280_pm_ctrl,				\
			 &bme280_data_##inst,				\
			 &bme280_config_##inst,				\
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &bme280_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(BME280_DEFINE)
