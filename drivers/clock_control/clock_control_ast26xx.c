/*
 * Copyright (c) 2022 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast26xx_clock
#include <errno.h>
#include <zephyr/dt-bindings/clock/ast26xx_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_ast26xx);

#define CLKIN_FREQ			MHZ(25)
#define HPLL_FREQ			MHZ(1200)
#define HCLK_FREQ			MHZ(200)

/*
 * CLK_STOP_CTRL0/1_SET registers:
 *   - Each bit in these registers controls a clock gate
 *   - Write '1' to a bit: turn OFF the corresponding clock
 *   - Write '0' to a bit: no effect
 * CLK_STOP_CTRL0/1_CLEAR register:
 *   - Write '1' to a bit: clear the corresponding bit in CLK_STOP_CTRL0/1.
 *                         (turn ON the corresponding clock)
 */
#define CLK_STOP_CTRL0_SET		0x80
#define CLK_STOP_CTRL0_CLEAR		0x84
#define CLK_STOP_CTRL1_SET		0x90
#define CLK_STOP_CTRL1_CLEAR		0x94

#define APLL_PARAM_REG			0x210
#define   PLL_PARAM_BYPASS_MODE		BIT(24)
#define   PLL_PARAM_P			GENMASK(22, 19)
#define   PLL_PARAM_N			GENMASK(18, 13)
#define   PLL_PARAM_M			GENMASK(12, 0)

#define CLK_SELECTION_REG0		0x300
#define   APB1_DIV_SEL			GENMASK(25, 23)
#define     APB1_DIV_REG_TO_NUM(x)	(((x) + 1) * 4)

#define CLK_SELECTION_REG4		0x310
#define   APB2_DIV_SEL			GENMASK(11, 9)
#define     APB2_DIV_REG_TO_NUM(x)	(((x) + 1) * 2)

#define CLK_SELECTION_REG5		0x314
#define   I3C_CLK_SRC_SEL		BIT(31)
#define     I3C_CLK_SRC_HCLK		0
#define     I3C_CLK_SRC_APLL_DIV	1
#define   I3C_CLK_APLL_DIV_SEL		GENMASK(30, 28)
#define     I3C_CLK_APLL_DIV_REG_TO_NUM(x) (((x) == 0) ? 2 : ((x) + 1))

struct clock_aspeed_config {
	const struct device *syscon;
};

#define DEV_CFG(dev) ((const struct clock_aspeed_config *const)(dev)->config)

static int aspeed_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct device *syscon = DEV_CFG(dev)->syscon;
	uint32_t clk_gate = (uint32_t)sub_system;
	uint32_t addr = CLK_STOP_CTRL0_CLEAR;

	/* there is no on/off control for group2 clocks */
	if (clk_gate >= ASPEED_CLK_GRP_2_OFFSET) {
		return 0;
	}

	if (clk_gate >= ASPEED_CLK_GRP_1_OFFSET) {
		clk_gate -= ASPEED_CLK_GRP_1_OFFSET;
		addr = CLK_STOP_CTRL1_CLEAR;
	}

	syscon_write_reg(syscon, addr, BIT(clk_gate));

	return 0;
}

static int aspeed_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct device *syscon = DEV_CFG(dev)->syscon;
	uint32_t clk_gate = (uint32_t)sub_system;
	uint32_t addr = CLK_STOP_CTRL0_SET;

	/* there is no on/off control for group2 clocks */
	if (clk_gate >= ASPEED_CLK_GRP_2_OFFSET) {
		return 0;
	}

	if (clk_gate >= ASPEED_CLK_GRP_1_OFFSET) {
		clk_gate -= ASPEED_CLK_GRP_1_OFFSET;
		addr = CLK_STOP_CTRL1_SET;
	}

	syscon_write_reg(syscon, addr, BIT(clk_gate));

	return 0;
}

static int aspeed_clock_control_get_pll_freq(uint32_t reg_value)
{
	uint32_t mult, div;

	if (reg_value & PLL_PARAM_BYPASS_MODE) {
		mult = 1;
		div = 1;
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
	const struct device *syscon = DEV_CFG(dev)->syscon;
	uint32_t clk_id = (uint32_t)sub_system;
	uint32_t reg, src, clk_div;

	switch (clk_id) {
	case ASPEED_CLK_GATE_I3C0CLK:
	case ASPEED_CLK_GATE_I3C1CLK:
	case ASPEED_CLK_GATE_I3C2CLK:
	case ASPEED_CLK_GATE_I3C3CLK:
	case ASPEED_CLK_GATE_I3C4CLK:
	case ASPEED_CLK_GATE_I3C5CLK:
		syscon_read_reg(syscon, CLK_SELECTION_REG5, &reg);
		if (FIELD_GET(I3C_CLK_SRC_SEL, reg) == I3C_CLK_SRC_APLL_DIV) {
			uint32_t apll;

			syscon_read_reg(syscon, APLL_PARAM_REG, &apll);
			src = aspeed_clock_control_get_pll_freq(apll);
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
		syscon_read_reg(syscon, CLK_SELECTION_REG0, &reg);
		clk_div = APB1_DIV_REG_TO_NUM(FIELD_GET(APB1_DIV_SEL, reg));
		*rate = src / clk_div;
		break;
	case ASPEED_CLK_APB2:
		src = HCLK_FREQ;
		syscon_read_reg(syscon, CLK_SELECTION_REG4, &reg);
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
		*rate = MHZ(24) / 13;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct clock_control_driver_api aspeed_clk_api = {
	.on = aspeed_clock_control_on,
	.off = aspeed_clock_control_off,
	.get_rate = aspeed_clock_control_get_rate,
};

#define ASPEED_CLOCK_INIT(n)                                                                       \
	static const struct clock_aspeed_config clock_aspeed_cfg_##n = {                           \
		.syscon = DEVICE_DT_GET(DT_NODELABEL(syscon)),                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &clock_aspeed_cfg_##n, PRE_KERNEL_1,            \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &aspeed_clk_api);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CLOCK_INIT)
