/*
 * Copyright (c) 2021, ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast10x0_clk
#include <errno.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <dt-bindings/clock/ast10x0_clock.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(clock_control_aspeed);

#define HPLL_FREQ			1000000000U
#define CLK_SELECTION_REG4		0x310
#define   I3C_CLK_SRC_SEL		BIT(31)
#define     I3C_CLK_SRC_HPLL		0
#define     I3C_CLK_SRC_480M		1
#define   I3C_CLK_DIV_SEL		GENMASK(30, 28)
#define CLK_SELECTION_REG5		0x314

struct clock_aspeed_config {
	uintptr_t base;
};

#define DEV_CFG(dev)				   \
	((const struct clock_aspeed_config *const) \
	 (dev)->config)

static int aspeed_clock_i3c_div_tbl[8] = {2, 2, 3, 4, 5, 6, 7, 8};

static int aspeed_clock_control_on(const struct device *dev,
				   clock_control_subsys_t sub_system)
{
	uint32_t clk_gate = (uint32_t)sub_system;
	uint32_t addr = DEV_CFG(dev)->base + 0x84;

	if (clk_gate >= ASPEED_CLK_GRP_2_OFFSET) {
		return 0;
	}

	if (clk_gate >= ASPEED_CLK_GRP_1_OFFSET) {
		clk_gate -= ASPEED_CLK_GRP_1_OFFSET;
		addr += 0x10;
	}

	sys_write32(BIT(clk_gate), addr);

	return 0;
}

static int aspeed_clock_control_off(const struct device *dev,
				    clock_control_subsys_t sub_system)
{
	uint32_t clk_gate = (uint32_t)sub_system;
	uint32_t addr = DEV_CFG(dev)->base + 0x80;

	if (clk_gate >= ASPEED_CLK_GRP_2_OFFSET) {
		return 0;
	}

	if (clk_gate >= ASPEED_CLK_GRP_1_OFFSET) {
		clk_gate -= ASPEED_CLK_GRP_1_OFFSET;
		addr += 0x10;
	}

	sys_write32(BIT(clk_gate), addr);

	return 0;
}

static int aspeed_clock_control_get_rate(const struct device *dev,
					 clock_control_subsys_t sub_system, uint32_t *rate)
{
	uint32_t clk_id = (uint32_t) sub_system;
	uint32_t reg, src;
	int index, div;

	switch (clk_id) {
	case ASPEED_CLK_GATE_I3C0CLK:
	case ASPEED_CLK_GATE_I3C1CLK:
	case ASPEED_CLK_GATE_I3C2CLK:
	case ASPEED_CLK_GATE_I3C3CLK:
		reg = sys_read32(DEV_CFG(dev)->base + CLK_SELECTION_REG4);
		if (FIELD_GET(I3C_CLK_SRC_SEL, reg) == I3C_CLK_SRC_HPLL) {
			src = HPLL_FREQ;
		} else {
			src = 480000000;
		}
		index = FIELD_GET(I3C_CLK_DIV_SEL, reg);
		div = aspeed_clock_i3c_div_tbl[index];
		*rate = src / div;
		break;
	case ASPEED_CLK_HCLK:
		*rate = 200000000;
		break;
	case ASPEED_CLK_PCLK:
		*rate = 50000000;
		break;
	case ASPEED_CLK_GATE_UART1CLK:
	case ASPEED_CLK_GATE_UART2CLK:
	case ASPEED_CLK_GATE_UART3CLK:
	case ASPEED_CLK_GATE_UART4CLK:
		reg = sys_read32(0x7e789098);
		reg &= ~(BIT(clk_id - ASPEED_CLK_GATE_UART1CLK + 4));
		sys_write32(reg, 0x7e789098);
	case ASPEED_CLK_UART5:
	case ASPEED_CLK_GATE_UART6CLK:
	case ASPEED_CLK_GATE_UART7CLK:
	case ASPEED_CLK_GATE_UART8CLK:
	case ASPEED_CLK_GATE_UART9CLK:
	case ASPEED_CLK_GATE_UART10CLK:
	case ASPEED_CLK_GATE_UART11CLK:
	case ASPEED_CLK_GATE_UART12CLK:
	case ASPEED_CLK_GATE_UART13CLK:
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

static const struct clock_aspeed_config clock_aspeed_cfg = {
	.base = DT_REG_ADDR(DT_NODELABEL(syscon)),
};

#define ASPEED_CLOCK_INIT(n)							\
										\
	DEVICE_DT_INST_DEFINE(n,						\
			      &aspeed_clock_control_init,			\
			      NULL,						\
			      NULL, &clock_aspeed_cfg,				\
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &aspeed_clk_api);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CLOCK_INIT)
