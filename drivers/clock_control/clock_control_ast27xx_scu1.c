/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast27xx_scu1_clock
#include <errno.h>
#include <zephyr/dt-bindings/clock/ast27xx_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_ast27xx_scu1);

#define HPLL_FREQ			MHZ(1000)

#define CLK_STOP_CTRL0_SET		0x240
#define CLK_STOP_CTRL0_CLEAR		0x244

#define CLK_STOP_CTRL1_SET		0x260
#define CLK_STOP_CTRL1_CLEAR		0x264

#define CLK_SELECTION_REG1		0x280
#define CLK_SELECTION_REG2		0x284

struct clock_ast27xx_scu1_config {
	uintptr_t base;
};

static int
ast27xx_scu1_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast27xx_scu1_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	if (clk_gate > 32)
		sys_set_bit(config->base + CLK_STOP_CTRL0_CLEAR, BIT(clk_gate - 32));
	else
		sys_set_bit(config->base + CLK_STOP_CTRL1_CLEAR, BIT(clk_gate));

	return 0;
}

static int
ast27xx_scu1_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast27xx_scu1_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	if (clk_gate > 32)
		sys_set_bit(config->base + CLK_STOP_CTRL0_SET, BIT(clk_gate - 32));
	else
		sys_set_bit(config->base + CLK_STOP_CTRL1_SET, BIT(clk_gate));

	return 0;
}

static int ast27xx_scu1_clock_control_get_rate(const struct device *dev,
					     clock_control_subsys_t sub_system, uint32_t *rate)
{
	uint32_t clk_id = (uint32_t)sub_system;

	switch (clk_id) {
	case SCU1_CLK_HPLL:
		*rate = MHZ(1000);
		break;
	case SCU1_CLK_APLL:
		*rate = MHZ(800);
		break;
	case SCU1_CLK_AHB:
		*rate = MHZ(200);
		break;
	case SCU1_CLK_APB:
		*rate = MHZ(100);
		break;
	case SCU1_CLK_GATE_UART0CLK:
	case SCU1_CLK_GATE_UART1CLK:
	case SCU1_CLK_GATE_UART2CLK:
	case SCU1_CLK_GATE_UART3CLK:
	case SCU1_CLK_GATE_UART5CLK:
	case SCU1_CLK_GATE_UART6CLK:
	case SCU1_CLK_GATE_UART7CLK:
	case SCU1_CLK_GATE_UART8CLK:
	case SCU1_CLK_GATE_UART9CLK:
	case SCU1_CLK_GATE_UART10CLK:
	case SCU1_CLK_GATE_UART11CLK:
	case SCU1_CLK_GATE_UART12CLK:
		*rate = MHZ(24) / 13;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct clock_control_driver_api aspeed_clk_scu1_api = {
	.on = ast27xx_scu1_clock_control_on,
	.off = ast27xx_scu1_clock_control_off,
	.get_rate = ast27xx_scu1_clock_control_get_rate,
};

static const struct clock_ast27xx_scu1_config clock_scu1_config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &clock_scu1_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &aspeed_clk_scu1_api);
