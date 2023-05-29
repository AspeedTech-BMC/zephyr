/*
 * Copyright (c) ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_i2c_global
#include <drivers/hwinfo.h>
#include <drivers/clock_control.h>
#include <drivers/reset_control.h>
#include <drivers/i2c.h>
#include "soc.h"

#include <logging/log.h>
#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
LOG_MODULE_REGISTER(i2c_global);

#include <sys/sys_io.h>
#include <device.h>

#define ASPEED_I2CG_CONTROL		0x0C
#define ASPEED_I2CG_NEW_CLK_DIV	0x10

#define ASPEED_I2C_SRAM_BASE		0x7e7b0e00
#define ASPEED_I2C_SRAM_SIZE		0x40

#define CLK_NEW_MODE		BIT(1)
#define REG_NEW_MODE		BIT(2)
#define ISSUE_NAK_EMPTY	BIT(4)

#define I2CG_SET	(CLK_NEW_MODE |\
			REG_NEW_MODE |\
			ISSUE_NAK_EMPTY)

#define AST2600ID 0x05000000

/* Device config */
struct i2c_global_config {
	uintptr_t base; /* i2c controller base address */
	const struct device *clock_dev;
	const clock_control_subsys_t clk_id;
	const reset_control_subsys_t rst_id;
	uint32_t clk_src;
	uint32_t clk_divider;
};

#define DEV_CFG(dev)			 \
	((struct i2c_global_config *) \
	 (dev)->config)

/* #define I2CG_DIV_CTRL 0x62220803 */
/* This clock divider setting has been removed into dtsi file */
/*
 * APB clk : 50Mhz
 * div  : scl       : baseclk [APB/((div/2) + 1)] : tBuf [1/bclk * 16]
 * I2CG10[31:24] base clk4 for i2c auto recovery timeout counter (0x62)
 * I2CG10[23:16] base clk3 for Standard-mode (100Khz) min tBuf 4.7us
 * 0x1d : 100.8Khz  : 3.225Mhz                    : 4.96us
 * 0x1e : 97.66Khz  : 3.125Mhz                    : 5.12us
 * 0x1f : 97.85Khz  : 3.03Mhz                     : 5.28us
 * 0x20 : 98.04Khz  : 2.94Mhz                     : 5.44us
 * 0x21 : 98.61Khz  : 2.857Mhz                    : 5.6us
 * 0x22 : 99.21Khz  : 2.77Mhz                     : 5.76us (default)
 * I2CG10[15:8] base clk2 for Fast-mode (400Khz) min tBuf 1.3us
 * 0x08 : 400Khz    : 10Mhz                       : 1.6us
 * I2CG10[7:0] base clk1 for Fast-mode Plus (1Mhz) min tBuf 0.5us
 * 0x03 : 1Mhz      : 20Mhz                       : 0.8us
 */

/* I2C controller driver registration */
static int i2c_global_init(const struct device *dev)
{
	struct i2c_global_config *config = DEV_CFG(dev);
	uint64_t rev_id;
	size_t len;
	uint32_t i2c_global_base = config->base;
	uint32_t *base = (uint32_t *)ASPEED_I2C_SRAM_BASE;

	const struct device *reset_dev = device_get_binding(ASPEED_RST_CTRL_NAME);

	/* check chip id*/
	len = hwinfo_get_device_id((uint8_t *)&rev_id, sizeof(rev_id));

	/* skip i2c common config change when the zephyr is running on co-processer */
	if (((uint32_t)rev_id & 0xFF000000) != AST2600ID) {
		/* i2c controller reset / de-reset */
		reset_control_assert(reset_dev, DEV_CFG(dev)->rst_id);
		reset_control_deassert(reset_dev, DEV_CFG(dev)->rst_id);

		/* set i2c global setting */
		sys_write32(I2CG_SET, i2c_global_base + ASPEED_I2CG_CONTROL);

		/* divider parameter */
		sys_write32(config->clk_divider, i2c_global_base + ASPEED_I2CG_NEW_CLK_DIV);

		/* initial i2c sram region */
		for (int i = 0; i < ASPEED_I2C_SRAM_SIZE; i++)
			*(base + i) = 0;
	}

	return 0;
}

static const struct i2c_global_config i2c_aspeed_config = {
	.base = DT_REG_ADDR(DT_NODELABEL(i2c_gr)),
	.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, clk_id),
	.rst_id = (reset_control_subsys_t)DT_INST_RESETS_CELL(0, rst_id),
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clk_divider = DT_INST_PROP(0, clk_divider),
};


DEVICE_DT_INST_DEFINE(0, &i2c_global_init, NULL,
		      NULL, &i2c_aspeed_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
