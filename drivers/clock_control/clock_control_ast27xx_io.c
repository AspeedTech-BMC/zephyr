/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast27xx_io_clock
#include <errno.h>
#include <zephyr/dt-bindings/clock/ast27xx_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_ast27xx_io);

#define HPLL_FREQ			MHZ(1000)

#define CLK_STOP_CTRL0_SET		0x00
#define CLK_STOP_CTRL0_CLEAR		0x04

#define CLK_STOP_CTRL1_SET		0x20
#define CLK_STOP_CTRL1_CLEAR		0x24

#define CLK_SELECTION_REG1		0x40
#define CLK_SELECTION_REG2		0x44

struct clock_ast27xx_io_config {
	uintptr_t base;
};

static int ast27xx_io_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast27xx_io_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	if (clk_gate > 32)
		sys_set_bit(config->base + CLK_STOP_CTRL0_CLEAR, ((clk_gate - 32) & 0x1FU));
	else
		sys_set_bit(config->base + CLK_STOP_CTRL1_CLEAR, (clk_gate & 0x1FU));

	return 0;
}

static int ast27xx_io_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast27xx_io_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	if (clk_gate > 32)
		sys_set_bit(config->base + CLK_STOP_CTRL0_SET, ((clk_gate - 32) & 0x1FU));
	else
		sys_set_bit(config->base + CLK_STOP_CTRL1_SET, (clk_gate & 0x1FU));

	return 0;
}

static int ast27xx_io_clock_control_get_rate(const struct device *dev,
					     clock_control_subsys_t sub_system, uint32_t *rate)
{
	uint32_t clk_id = (uint32_t)sub_system;

	switch (clk_id) {
	case AST2700_IO_CLK_HPLL:
		*rate = MHZ(1000);
		break;
	case AST2700_IO_CLK_APLL:
		*rate = MHZ(800);
		break;
	case AST2700_IO_CLK_AHB:
		*rate = MHZ(200);
		break;
	case AST2700_IO_CLK_APB:
		*rate = MHZ(100);
		break;
	case AST2700_IO_CLK_UART0:
	case AST2700_IO_CLK_UART1:
	case AST2700_IO_CLK_UART2:
	case AST2700_IO_CLK_UART3:
	case AST2700_IO_CLK_UART5:
	case AST2700_IO_CLK_UART6:
	case AST2700_IO_CLK_UART7:
	case AST2700_IO_CLK_UART8:
	case AST2700_IO_CLK_UART9:
	case AST2700_IO_CLK_UART10:
	case AST2700_IO_CLK_UART11:
	case AST2700_IO_CLK_UART12:
		*rate = MHZ(24) / 13;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct clock_control_driver_api aspeed_clk_io_api = {
	.on = ast27xx_io_clock_control_on,
	.off = ast27xx_io_clock_control_off,
	.get_rate = ast27xx_io_clock_control_get_rate,
};

static const struct clock_ast27xx_io_config clock_ast27xx_io_config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)) + DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &clock_ast27xx_io_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &aspeed_clk_io_api);
