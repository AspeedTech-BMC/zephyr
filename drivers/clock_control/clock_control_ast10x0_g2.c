/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast10x0_g2_clock

#include <errno.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/ast10x0_g2_clock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define CLK_STOP_CTRL0_SET		0x240
#define CLK_STOP_CTRL0_CLEAR		0x244
#define CLK_STOP_CTRL1_SET		0x260
#define CLK_STOP_CTRL1_CLEAR		0x264
#define SCU_CLK_SEL1			0x280
#define SCU_CLK_SEL2			0x284
#define SCU_HPLL_PARAM			0x300
#define SCU_UART_CLKGEN			0x330
#define SCU_HUART_CLKGEN		0x334

#define SCU_HPLL_M_MASK			GENMASK(12, 0)
#define SCU_HPLL_N_MASK			GENMASK(18, 13)
#define SCU_HPLL_P_MASK			GENMASK(22, 19)
#define SCU_HPLL_OFF			BIT(23)
#define SCU_HPLL_BYPASS			BIT(24)
#define SCU_HPLL_RESET			BIT(25)

#define SCU_CLK_STOP1_VALID_MASK	(BIT(0) | BIT(2) | BIT(6) | GENMASK(23, 11))
#define SCU_CLK_STOP2_VALID_MASK	(GENMASK(7, 0) | BIT(12) | BIT(17) | BIT(18))

#define SCU_CLKSEL1_PCLK_DIV_MASK	GENMASK(20, 18)
#define SCU_CLKSEL1_MEMCLK_DIV_MASK	GENMASK(16, 14)
#define SCU_CLKSEL2_SPIS1CLK_DIV_MASK	GENMASK(31, 29)
#define SCU_CLKSEL2_SPIS0CLK_DIV_MASK	GENMASK(28, 26)
#define SCU_CLKSEL2_I3CCLK_DIV_MASK	GENMASK(25, 23)
#define SCU_CLKSEL2_HCLK_DIV_MASK	GENMASK(22, 20)
#define SCU_CLKSEL2_PECICLK_SEL		BIT(16)
#define SCU_CLKSEL2_SSPCLK_DIV_MASK	GENMASK(10, 8)
#define SCU_CLKSEL2_PSPCLK_DIV_MASK	GENMASK(7, 5)
#define SCU_CLKSEL2_HUXCLK_SEL_MASK	GENMASK(4, 3)
#define SCU_CLKSEL2_UXCLK_SEL_MASK	GENMASK(1, 0)

#define SCU_UART_CLKGEN_N_MASK		GENMASK(17, 8)
#define SCU_UART_CLKGEN_R_MASK		GENMASK(7, 0)

struct clock_ast10x0_g2_config {
	uintptr_t base;
};

static int ast10x0_g2_set_clock_gate(const struct device *dev,
				     clock_control_subsys_t sub_system, bool enable)
{
	const struct clock_ast10x0_g2_config *config = dev->config;
	uintptr_t clk_gate = (uintptr_t)sub_system;
	uint32_t valid_mask;
	uint32_t offset;
	uint32_t bit;

	if (clk_gate < 32U) {
		valid_mask = SCU_CLK_STOP1_VALID_MASK;
		offset = enable ? CLK_STOP_CTRL0_CLEAR : CLK_STOP_CTRL0_SET;
		bit = (uint32_t)clk_gate;
	} else if (clk_gate < SCU1_CLK_GATE_NUM) {
		valid_mask = SCU_CLK_STOP2_VALID_MASK;
		offset = enable ? CLK_STOP_CTRL1_CLEAR : CLK_STOP_CTRL1_SET;
		bit = (uint32_t)(clk_gate - 32U);
	} else {
		return -EINVAL;
	}

	if ((valid_mask & BIT(bit)) == 0U) {
		return -EINVAL;
	}

	/* Clock stop registers are write-one set/clear aliases. */
	sys_write32(BIT(bit), config->base + offset);

	return 0;
}

static int ast10x0_g2_clock_control_on(const struct device *dev,
				       clock_control_subsys_t sub_system)
{
	return ast10x0_g2_set_clock_gate(dev, sub_system, true);
}

static int ast10x0_g2_clock_control_off(const struct device *dev,
					clock_control_subsys_t sub_system)
{
	return ast10x0_g2_set_clock_gate(dev, sub_system, false);
}

static uint32_t ast10x0_g2_get_hpll_rate(uintptr_t base)
{
	uint32_t reg = sys_read32(base + SCU_HPLL_PARAM);
	uint32_t m = FIELD_GET(SCU_HPLL_M_MASK, reg);
	uint32_t n = FIELD_GET(SCU_HPLL_N_MASK, reg);
	uint32_t p = FIELD_GET(SCU_HPLL_P_MASK, reg);
	uint64_t rate;

	if ((reg & (SCU_HPLL_OFF | SCU_HPLL_RESET)) != 0U) {
		return 0U;
	}

	if ((reg & SCU_HPLL_BYPASS) != 0U) {
		return MHZ(25);
	}

	rate = (uint64_t)MHZ(25) * (m + 1U);
	rate /= n + 1U;
	rate /= p + 1U;

	return (uint32_t)rate;
}

static uint32_t ast10x0_g2_get_i3cclk_rate(uintptr_t base)
{
	static const uint8_t dividers[] = { 2, 2, 3, 4, 5, 6, 7, 8 };
	uint32_t clk_sel2 = sys_read32(base + SCU_CLK_SEL2);
	uint32_t div_sel = FIELD_GET(SCU_CLKSEL2_I3CCLK_DIV_MASK, clk_sel2);
	uint32_t hpll_rate = ast10x0_g2_get_hpll_rate(base);
	uint32_t rate = hpll_rate / dividers[div_sel];

	return rate;
}

static void ast10x0_g2_configure_i3c_clk(uintptr_t base)
{
	uint32_t clk_sel2 = sys_read32(base + SCU_CLK_SEL2);

	clk_sel2 &= ~SCU_CLKSEL2_I3CCLK_DIV_MASK;
	clk_sel2 |= FIELD_PREP(SCU_CLKSEL2_I3CCLK_DIV_MASK, 7U);
	sys_write32(clk_sel2, base + SCU_CLK_SEL2);
}

static uint32_t ast10x0_g2_get_hclk_rate(uintptr_t base)
{
	static const uint8_t dividers[] = { 4, 4, 6, 8, 10, 12, 14, 16 };
	uint32_t clk_sel2 = sys_read32(base + SCU_CLK_SEL2);
	uint32_t div_sel = FIELD_GET(SCU_CLKSEL2_HCLK_DIV_MASK, clk_sel2);
	uint32_t hpll_rate = ast10x0_g2_get_hpll_rate(base);
	uint32_t rate = hpll_rate / dividers[div_sel];

	return rate;
}

static uint32_t ast10x0_g2_get_pclk_rate(uintptr_t base)
{
	uint32_t clk_sel1 = sys_read32(base + SCU_CLK_SEL1);
	uint32_t div_sel = FIELD_GET(SCU_CLKSEL1_PCLK_DIV_MASK, clk_sel1);
	uint32_t divider = (div_sel + 1U) * 4U;
	uint32_t hpll_rate = ast10x0_g2_get_hpll_rate(base);
	uint32_t rate = hpll_rate / divider;

	return rate;
}

static uint32_t ast10x0_g2_get_memclk_rate(uintptr_t base)
{
	static const uint8_t dividers[] = { 8, 8, 12, 16, 20, 24, 28, 32 };
	uint32_t clk_sel1 = sys_read32(base + SCU_CLK_SEL1);
	uint32_t div_sel = FIELD_GET(SCU_CLKSEL1_MEMCLK_DIV_MASK, clk_sel1);
	uint32_t hpll_rate = ast10x0_g2_get_hpll_rate(base);
	uint32_t rate = hpll_rate / dividers[div_sel];

	return rate;
}

static uint32_t ast10x0_g2_get_spisclk_rate(uintptr_t base, bool spis1)
{
	static const uint8_t dividers[] = { 2, 2, 3, 4, 5, 6, 7, 8 };
	uint32_t clk_sel2 = sys_read32(base + SCU_CLK_SEL2);
	uint32_t div_sel = FIELD_GET(spis1 ? SCU_CLKSEL2_SPIS1CLK_DIV_MASK :
					      SCU_CLKSEL2_SPIS0CLK_DIV_MASK,
				     clk_sel2);
	uint32_t hpll_rate = ast10x0_g2_get_hpll_rate(base);
	uint32_t rate = hpll_rate / dividers[div_sel];

	return rate;
}

static uint32_t ast10x0_g2_get_peciclk_rate(uintptr_t base)
{
	uint32_t clk_sel2 = sys_read32(base + SCU_CLK_SEL2);
	uint32_t rate;

	if ((clk_sel2 & SCU_CLKSEL2_PECICLK_SEL) != 0U) {
		rate = ast10x0_g2_get_hpll_rate(base) / 8U;
	} else {
		rate = MHZ(25);
	}

	return rate;
}

static uint32_t ast10x0_g2_get_security_clk_rate(uintptr_t base, bool ssp)
{
	static const uint8_t dividers[] = { 2, 2, 3, 4, 5, 6, 7, 8 };
	uint32_t clk_sel2 = sys_read32(base + SCU_CLK_SEL2);
	uint32_t div_sel = FIELD_GET(ssp ? SCU_CLKSEL2_SSPCLK_DIV_MASK :
					    SCU_CLKSEL2_PSPCLK_DIV_MASK,
				     clk_sel2);
	uint32_t hpll_rate = ast10x0_g2_get_hpll_rate(base);
	uint32_t rate = hpll_rate / dividers[div_sel];

	return rate;
}

static uint32_t ast10x0_g2_get_uxclk_rate(uintptr_t base, bool high_rate)
{
	static const uint8_t dividers[] = { 10, 5, 2, 1 };
	uint32_t clk_sel2 = sys_read32(base + SCU_CLK_SEL2);
	uint32_t div_sel;
	uint32_t hpll_rate;
	uint32_t rate;

	if (high_rate) {
		div_sel = FIELD_GET(SCU_CLKSEL2_HUXCLK_SEL_MASK, clk_sel2);
	} else {
		div_sel = FIELD_GET(SCU_CLKSEL2_UXCLK_SEL_MASK, clk_sel2);
	}

	hpll_rate = ast10x0_g2_get_hpll_rate(base);
	rate = hpll_rate / dividers[div_sel];

	return rate;
}

static int ast10x0_g2_get_uart_gen_rate(uintptr_t base, bool high_rate, uint32_t *rate)
{
	uint32_t reg = sys_read32(base +
				  (high_rate ? SCU_HUART_CLKGEN : SCU_UART_CLKGEN));
	uint32_t n = FIELD_GET(SCU_UART_CLKGEN_N_MASK, reg);
	uint32_t r = FIELD_GET(SCU_UART_CLKGEN_R_MASK, reg);
	uint64_t input_rate;

	if (n == 0U) {
		return -EINVAL;
	}

	input_rate = ast10x0_g2_get_uxclk_rate(base, high_rate);
	*rate = (uint32_t)((input_rate * r) / (n * 2U));

	return 0;
}

static int ast10x0_g2_get_uart_clk_rate(uintptr_t base, uint32_t uart_idx, uint32_t *rate)
{
	bool high_rate = (sys_read32(base + SCU_CLK_SEL1) & BIT(uart_idx)) != 0U;

	return ast10x0_g2_get_uart_gen_rate(base, high_rate, rate);
}

static int ast10x0_g2_clock_control_get_rate(const struct device *dev,
					      clock_control_subsys_t sub_system,
					      uint32_t *rate)
{
	const struct clock_ast10x0_g2_config *config = dev->config;
	uint32_t clk_id = (uint32_t)(uintptr_t)sub_system;
	int ret = 0;

	if (rate == NULL) {
		return -EINVAL;
	}

	switch (clk_id) {
	case SCU1_CLKIN:
		*rate = MHZ(25);
		break;
	case SCU1_CLK_HPLL:
		*rate = ast10x0_g2_get_hpll_rate(config->base);
		break;
	case SCU1_CLK_UXCLK:
		*rate = ast10x0_g2_get_uxclk_rate(config->base, false);
		break;
	case SCU1_CLK_HUXCLK:
		*rate = ast10x0_g2_get_uxclk_rate(config->base, true);
		break;
	case SCU1_CLK_UARTX:
		ret = ast10x0_g2_get_uart_gen_rate(config->base, false, rate);
		break;
	case SCU1_CLK_HUARTX:
		ret = ast10x0_g2_get_uart_gen_rate(config->base, true, rate);
		break;
	case SCU1_CLK_AHB:
		*rate = ast10x0_g2_get_hclk_rate(config->base);
		break;
	case SCU1_CLK_APB:
		*rate = ast10x0_g2_get_pclk_rate(config->base);
		break;
	case SCU1_CLK_MEMCLK:
		*rate = ast10x0_g2_get_memclk_rate(config->base);
		break;
	case SCU1_CLK_SPIS0:
		*rate = ast10x0_g2_get_spisclk_rate(config->base, false);
		break;
	case SCU1_CLK_SPIS1:
		*rate = ast10x0_g2_get_spisclk_rate(config->base, true);
		break;
	case SCU1_CLK_PECI:
		*rate = ast10x0_g2_get_peciclk_rate(config->base);
		break;
	case SCU1_CLK_SSP:
		*rate = ast10x0_g2_get_security_clk_rate(config->base, true);
		break;
	case SCU1_CLK_PSP:
		*rate = ast10x0_g2_get_security_clk_rate(config->base, false);
		break;
	case SCU1_CLK_UART4:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 4U, rate);
		break;
	case SCU1_CLK_I3C:
		*rate = ast10x0_g2_get_i3cclk_rate(config->base);
		break;
	case SCU1_CLK_GATE_UART0CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 0U, rate);
		break;
	case SCU1_CLK_GATE_UART1CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 1U, rate);
		break;
	case SCU1_CLK_GATE_UART2CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 2U, rate);
		break;
	case SCU1_CLK_GATE_UART3CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 3U, rate);
		break;
	case SCU1_CLK_GATE_UART5CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 5U, rate);
		break;
	case SCU1_CLK_GATE_UART6CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 6U, rate);
		break;
	case SCU1_CLK_GATE_UART7CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 7U, rate);
		break;
	case SCU1_CLK_GATE_UART8CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 8U, rate);
		break;
	case SCU1_CLK_GATE_UART9CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 9U, rate);
		break;
	case SCU1_CLK_GATE_UART10CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 10U, rate);
		break;
	case SCU1_CLK_GATE_UART11CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 11U, rate);
		break;
	case SCU1_CLK_GATE_UART12CLK:
		ret = ast10x0_g2_get_uart_clk_rate(config->base, 12U, rate);
		break;
	case SCU1_CLK_GATE_I3C0CLK:
	case SCU1_CLK_GATE_I3C1CLK:
	case SCU1_CLK_GATE_I3C2CLK:
	case SCU1_CLK_GATE_I3C3CLK:
	case SCU1_CLK_GATE_I3C4CLK:
	case SCU1_CLK_GATE_I3C5CLK:
	case SCU1_CLK_GATE_I3C6CLK:
	case SCU1_CLK_GATE_I3C7CLK:
		*rate = ast10x0_g2_get_i3cclk_rate(config->base);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ast10x0_g2_clock_control_init(const struct device *dev)
{
	const struct clock_ast10x0_g2_config *config = dev->config;

	ast10x0_g2_configure_i3c_clk(config->base);

	return 0;
}

static const struct clock_control_driver_api aspeed_clk_scu1_api = {
	.on = ast10x0_g2_clock_control_on,
	.off = ast10x0_g2_clock_control_off,
	.get_rate = ast10x0_g2_clock_control_get_rate,
};

static const struct clock_ast10x0_g2_config clock_scu1_config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, ast10x0_g2_clock_control_init, NULL, NULL, &clock_scu1_config,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &aspeed_clk_scu1_api);
