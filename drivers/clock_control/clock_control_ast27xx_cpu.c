/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast27xx_cpu_clock
#include <errno.h>
#include <zephyr/dt-bindings/clock/ast27xx_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_ast27xx_cpu);

#define HPLL_FREQ			MHZ(1000)

#define CLK_STOP_CTRL_SET		0x00
#define CLK_STOP_CTRL_CLEAR		0x04

#define AST2700_CPU_CLK_SEL1 0x40
#define AST2700_CPU_CLK_SEL2 0x44
#define UART_DIV13_EN BIT(30)
#define AST2700_CPU_HPLL_PARAM 0xC0
#define AST2700_CPU_DPLL_PARAM 0xC8
#define AST2700_CPU_MPLL_PARAM 0xD0
#define AST2700_CPU_D1CLK_PARAM 0xE0
#define AST2700_CPU_D2CLK_PARAM 0xF0
#define AST2700_CPU_CRT1CLK_PARAM 0x100
#define AST2700_CPU_CRT2CLK_PARAM 0x110
#define AST2700_CPU_MPHYCLK_PARAM 0x120

struct clock_ast27xx_cpu_config {
	uintptr_t base;
};

static int
ast27xx_cpu_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast27xx_cpu_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	sys_set_bit(config->base + CLK_STOP_CTRL_CLEAR, (clk_gate & 0x1FU));

	return 0;
}

static int
ast27xx_cpu_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast27xx_cpu_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	sys_set_bit(config->base + CLK_STOP_CTRL_SET, (clk_gate & 0x1FU));

	return 0;
}

static int ast27xx_cpu_clock_control_get_rate(const struct device *dev,
					      clock_control_subsys_t sub_system, uint32_t *rate)
{
	uint32_t clk_id = (uint32_t)sub_system;

	switch (clk_id) {
	case AST2700_CPU_CLK_HPLL:
		/* HPLL/DPLL: 2000Mhz(default) */
		*rate = MHZ(2000);
		break;
	case AST2700_CPU_CLK_AHB:
		/* AHB CLK mpll/4 = 400Mhz*/
		*rate = MHZ(1600) / 4;
		break;
	case AST2700_CPU_CLK_APB:
		/* APB CLK MPLL/16 = 100Mhz */
		*rate = MHZ(1600) / 16;
		break;
	case AST2700_CPU_CLK_GATE_UART4CLK:
		*rate = MHZ(24) / 13;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct clock_control_driver_api ast27xx_cpu_clk_api = {
	.on = ast27xx_cpu_clock_control_on,
	.off = ast27xx_cpu_clock_control_off,
	.get_rate = ast27xx_cpu_clock_control_get_rate,
};

static const struct clock_ast27xx_cpu_config clock_ast27xx_cpu_config = {
	.base = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &clock_ast27xx_cpu_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &ast27xx_cpu_clk_api);
