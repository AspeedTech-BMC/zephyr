/*
 * Copyright (c) 2021, ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast10x0_clk
#include <errno.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <dt-bindings/clock/ast1030_clock.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(clock_control_aspeed);



static int aspeed_clock_control_on(const struct device *dev,
			      clock_control_subsys_t sub_system)
{
	return 0;
}

static int aspeed_clock_control_off(const struct device *dev,
			       clock_control_subsys_t sub_system)
{
	return 0;
}

static int aspeed_clock_control_get_rate(
					const struct device *dev,
				    clock_control_subsys_t sub_system,
				    uint32_t *rate)
{
	uint32_t clk_id = (uint32_t) sub_system;

	switch (clk_id) {
	case ASPEED_CLK_HCLK:
		*rate = 200000000;
		break;
	case ASPEED_CLK_PCLK:
		*rate = 50000000;
		break;
	case ASPEED_CLK_UART1:
	case ASPEED_CLK_UART2:
	case ASPEED_CLK_UART3:
	case ASPEED_CLK_UART4:
	case ASPEED_CLK_UART5:
	case ASPEED_CLK_UART6:
	case ASPEED_CLK_UART7:
	case ASPEED_CLK_UART8:
	case ASPEED_CLK_UART9:
	case ASPEED_CLK_UART10:
	case ASPEED_CLK_UART11:
	case ASPEED_CLK_UART12:
	case ASPEED_CLK_UART13:
		*rate = 24000000 / 13;
		break;
	case ASPEED_CLK_APB1:
		*rate = 50000000;
		break;
	default:
		LOG_ERR("Missing feature define for %d!", clk_id);
		break;
	}

	return 0;
}

static int aspeed_clock_control_init(const struct device *dev)
{
	return 0;
}

static const struct clock_control_driver_api aspeed_clk_api = {
	.on = aspeed_clock_control_on,
	.off = aspeed_clock_control_off,
	.get_rate = aspeed_clock_control_get_rate,
};

#define ASPEED_CLOCK_INIT(n) \
	\
DEVICE_DT_INST_DEFINE(n, \
		    &aspeed_clock_control_init, \
		    device_pm_control_nop, \
		    NULL, NULL, \
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
		    &aspeed_clk_api);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CLOCK_INIT)
