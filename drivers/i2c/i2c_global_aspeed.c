/*
 * Copyright (c) ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_i2c_global

#include <drivers/clock_control.h>
#include <drivers/reset_control.h>
#include <drivers/i2c.h>
#include "soc.h"

#include <logging/log.h>
#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
LOG_MODULE_REGISTER(i2c_global);

#include <sys/sys_io.h>
#include <device.h>

/* Device config */
struct i2c_global_config {
	uintptr_t base; /* i2c controller base address */
	const clock_control_subsys_t clk_id;
	const reset_control_subsys_t rst_id;
};

#define DEV_CFG(dev)				 \
	((const struct i2c_global_config *const) \
	 (dev)->config)


/* I2C controller driver registration */
static int i2c_global_init(const struct device *dev)
{
	uint32_t i2c_global_base = DEV_CFG(dev)->base;

	const struct device *reset_dev = device_get_binding(ASPEED_RST_CTRL_NAME);

	reset_control_assert(reset_dev, DEV_CFG(dev)->rst_id);
	k_msleep(1);
	reset_control_deassert(reset_dev, DEV_CFG(dev)->rst_id);
	k_msleep(1);

	/* TODO check delay */
	sys_write32(0x16, i2c_global_base + 0x0C);
	sys_write32(0x03020100, i2c_global_base + 0x10);

	return 0;
}

static const struct i2c_global_config i2c_aspeed_config = {
	.base = DT_REG_ADDR(DT_NODELABEL(i2c_gr)),
	.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, clk_id),
	.rst_id = (reset_control_subsys_t)DT_INST_RESETS_CELL(0, rst_id),
};


DEVICE_DT_INST_DEFINE(0, &i2c_global_init, NULL,
		      NULL, &i2c_aspeed_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
