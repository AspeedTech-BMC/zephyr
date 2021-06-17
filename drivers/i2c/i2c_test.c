/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This is not a real I2C driver. It is used to instantiate struct
 * devices for the "vnd,i2c" devicetree compatible used in test code.
 */

#include <zephyr.h>
#include <drivers/i2c.h>

#define DT_DRV_COMPAT vnd_i2c

static int vnd_i2c_configure(const struct device *dev,
			     uint32_t dev_config)
{
	return -ENOTSUP;
}

static int vnd_i2c_transfer(const struct device *dev,
			    struct i2c_msg *msgs,
			    uint8_t num_msgs, uint16_t addr)
{
	return -ENOTSUP;
}

static const struct i2c_driver_api vnd_i2c_api = {
	.configure = vnd_i2c_configure,
	.transfer  = vnd_i2c_transfer,
};

static int vnd_i2c_init(const struct device *dev)
{
	return 0;
}

#define VND_I2C_INIT(n)						\
	DEVICE_DT_INST_DEFINE(n, &vnd_i2c_init, NULL,			\
			      NULL, NULL, POST_KERNEL,			\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &vnd_i2c_api);

DT_INST_FOREACH_STATUS_OKAY(VND_I2C_INIT)
