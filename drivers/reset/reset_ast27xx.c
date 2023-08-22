/*
 * Copyright (c) 2022 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast27xx_reset
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/reset.h>

#define RESET_CTRL0_ASSERT		0x00
#define RESET_CTRL0_DEASSERT		0x04
#define RESET_CTRL1_ASSERT		0x20
#define RESET_CTRL1_DEASSERT		0x24

struct reset_ast27xx_config {
	uintptr_t base;
};

static int ast27xx_reset_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct reset_ast27xx_config *config = dev->config;
	uint32_t addr = RESET_CTRL0_ASSERT;

	if (id >= 32) {
		id -= 32;
		addr = RESET_CTRL1_ASSERT;
	}

	*status = !!sys_test_bit(config->base + addr, ((id) & 0x1FU));

	return 0;
}

static int ast27xx_reset_line_assert(const struct device *dev, uint32_t id)
{
	const struct reset_ast27xx_config *config = dev->config;
	uint32_t addr = RESET_CTRL0_ASSERT;

	if (id >= 32) {
		id -= 32;
		addr = RESET_CTRL1_ASSERT;
	}

	sys_set_bit(config->base + addr, ((id) & 0x1FU));

	return 0;
}

static int ast27xx_reset_line_deassert(const struct device *dev, uint32_t id)
{
	const struct reset_ast27xx_config *config = dev->config;
	uint32_t addr = RESET_CTRL0_DEASSERT;

	if (id >= 32) {
		id -= 32;
		addr = RESET_CTRL1_DEASSERT;
	}

	sys_set_bit(config->base + addr, ((id) & 0x1FU));

	return 0;
}

static int ast27xx_reset_line_toggle(const struct device *dev, uint32_t id)
{
	ast27xx_reset_line_assert(dev, id);
	ast27xx_reset_line_deassert(dev, id);

	return 0;
}

static const struct reset_driver_api ast27xx_reset_api = {
	.status = ast27xx_reset_status,
	.line_assert = ast27xx_reset_line_assert,
	.line_deassert = ast27xx_reset_line_deassert,
	.line_toggle = ast27xx_reset_line_toggle
};

static const struct reset_ast27xx_config reset_ast27xx_config = {
	.base = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &reset_ast27xx_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &ast27xx_reset_api)

