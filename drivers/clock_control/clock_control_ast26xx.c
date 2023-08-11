/*
 * Copyright (c) 2021, ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast26xx_clk
#include <errno.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <dt-bindings/clock/ast26xx_clock.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(clock_control_aspeed);

/* fixed clock sources */
#define CLKIN_FREQ			MHZ(25)
#define HPLL_FREQ			MHZ(1200)
#define HCLK_FREQ			MHZ(200)

#define APLL_PARAM_REG			0x210
#define   PLL_PARAM_BYPASS_MODE		BIT(24)
#define   PLL_PARAM_P			GENMASK(22, 19)
#define   PLL_PARAM_N			GENMASK(18, 13)
#define   PLL_PARAM_M			GENMASK(12, 0)

#define CLK_SELECTION_REG0		0x300
#define   APB1_DIV_SEL			GENMASK(25, 23)
#define     APB1_DIV_REG_TO_NUM(x)	((x + 1) * 4)

#define CLK_SELECTION_REG4		0x310
#define   APB2_DIV_SEL			GENMASK(11, 9)
#define     APB2_DIV_REG_TO_NUM(x)	((x + 1) * 2)

#define CLK_SELECTION_REG5		0x314
#define   I3C_CLK_SRC_SEL		BIT(31)
#define     I3C_CLK_SRC_HCLK		0
#define     I3C_CLK_SRC_APLL_DIV	1
#define   I3C_CLK_APLL_DIV_SEL		GENMASK(30, 28)
#define     I3C_CLK_APLL_DIV_REG_TO_NUM(x) ((x == 0) ? 2 : (x + 1))

struct clock_aspeed_config {
	uintptr_t base;
};

#define DEV_CFG(dev)				   \
	((const struct clock_aspeed_config *const) \
	 (dev)->config)

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

static int aspeed_clock_control_get_pll_freq(uint32_t reg_value)
{
	uint32_t mult, div;

	if (reg_value & PLL_PARAM_BYPASS_MODE) {
		mult = div = 1;
	} else {
		uint32_t m = FIELD_GET(PLL_PARAM_M, reg_value);
		uint32_t n = FIELD_GET(PLL_PARAM_N, reg_value);
		uint32_t p = FIELD_GET(PLL_PARAM_P, reg_value);

		mult = (m + 1) / (n + 1);
		div = p + 1;
	}

	return (CLKIN_FREQ * mult / div);
}

static int aspeed_clock_control_get_rate(const struct device *dev,
					 clock_control_subsys_t sub_system, uint32_t *rate)
{
	uint32_t clk_id = (uint32_t)sub_system;
	uint32_t base = DEV_CFG(dev)->base;
	uint32_t reg, src, clk_div;

	switch (clk_id) {
	case ASPEED_CLK_GATE_I3C0CLK:
	case ASPEED_CLK_GATE_I3C1CLK:
	case ASPEED_CLK_GATE_I3C2CLK:
	case ASPEED_CLK_GATE_I3C3CLK:
	case ASPEED_CLK_GATE_I3C4CLK:
	case ASPEED_CLK_GATE_I3C5CLK:
		reg = sys_read32(base + CLK_SELECTION_REG5);
		if (FIELD_GET(I3C_CLK_SRC_SEL, reg) == I3C_CLK_SRC_APLL_DIV) {
			src = aspeed_clock_control_get_pll_freq(sys_read32(base + APLL_PARAM_REG));
			clk_div = I3C_CLK_APLL_DIV_REG_TO_NUM(FIELD_GET(I3C_CLK_APLL_DIV_SEL, reg));
			*rate = src / clk_div;
			break;
		}
		__fallthrough;
	case ASPEED_CLK_HCLK:
		*rate = HCLK_FREQ;
		break;
	case ASPEED_CLK_APB1:
		src = HPLL_FREQ;
		reg = sys_read32(base + CLK_SELECTION_REG0);
		clk_div = APB1_DIV_REG_TO_NUM(FIELD_GET(APB1_DIV_SEL, reg));
		*rate = src / clk_div;
		break;
	case ASPEED_CLK_APB2:
		src = HCLK_FREQ;
		reg = sys_read32(base + CLK_SELECTION_REG4);
		clk_div = APB2_DIV_REG_TO_NUM(FIELD_GET(APB2_DIV_SEL, reg));
		*rate = src / clk_div;
		break;
	case ASPEED_CLK_GATE_UART1CLK:
	case ASPEED_CLK_GATE_UART2CLK:
	case ASPEED_CLK_GATE_UART3CLK:
	case ASPEED_CLK_GATE_UART4CLK:
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
