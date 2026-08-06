/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast10x0_g2_clock
#include <errno.h>
#include <zephyr/dt-bindings/clock/ast27xx_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_ast10x0_g2);

#define CLK_STOP_CTRL0_SET		0x240
#define CLK_STOP_CTRL0_CLEAR		0x244

#define CLK_STOP_CTRL1_SET		0x260
#define CLK_STOP_CTRL1_CLEAR		0x264

struct ast10x0_g2_scu {
	uint32_t chip_id1;		/* 0x000 */
	uint32_t rsv_0x04[3];		/* 0x004 ~ 0x00C */
	uint32_t hwstrap1;		/* 0x010 */
	uint32_t hwstrap1_clr;		/* 0x014 */
	uint32_t rsv_0x18[2];		/* 0x018 ~ 0x01C */
	uint32_t hwstrap1_lock;		/* 0x020 */
	uint32_t hwstrap1_sec1;	/* 0x024 */
	uint32_t hwstrap1_sec2;	/* 0x028 */
	uint32_t hwstrap1_sec3;	/* 0x02C */
	uint32_t hwstrap2;		/* 0x030 */
	uint32_t hwstrap2_clr;		/* 0x034 */
	uint32_t rsv_0x38[2];		/* 0x038 ~ 0x03C */
	uint32_t hwstrap2_lock;		/* 0x040 */
	uint32_t hwstrap2_sec1;	/* 0x044 */
	uint32_t hwstrap2_sec2;	/* 0x048 */
	uint32_t hwstrap2_sec3;	/* 0x04C */
	uint32_t sysrest_log1;		/* 0x050 */
	uint32_t sysrest_log1_sec1;	/* 0x054 */
	uint32_t sysrest_log1_sec2;	/* 0x058 */
	uint32_t sysrest_log1_sec3;	/* 0x05C */
	uint32_t sysrest_log2;		/* 0x060 */
	uint32_t sysrest_log2_sec1;	/* 0x064 */
	uint32_t sysrest_log2_sec2;	/* 0x068 */
	uint32_t sysrest_log2_sec3;	/* 0x06C */
	uint32_t sysrest_log3;		/* 0x070 */
	uint32_t sysrest_log3_sec1; /* 0x074 */
	uint32_t sysrest_log3_sec2; /* 0x078 */
	uint32_t sysrest_log3_sec3; /* 0x07C */
	uint32_t sysrest_log4;		/* 0x080 */
	uint32_t sysrest_log4_sec1; /* 0x084 */
	uint32_t sysrest_log4_sec2; /* 0x088 */
	uint32_t sysrest_log4_sec3; /* 0x08C */
	uint32_t rsv_0x90[7];		/* 0x090 ~ 0xA8 */
	uint32_t uart_dbg_rate;		/* 0x0AC */
	uint32_t rsv_0xB0[4];		/* 0x0B0 ~ 0xBC*/
	uint32_t misc;			/* 0x0C0 */
	uint32_t rsv_0xC4;		/* 0x0C4 */
	uint32_t debug_ctrl;		/* 0x0C8 */
	uint32_t rsv_0xCC;		/* 0x0CC */
	uint32_t dac_ctrl;		/* 0x0D0 */
	uint32_t dac_crc_ctrl;		/* 0x0D4 */
	uint32_t rsv_0xD8[2];		/* 0x0D8 ~ 0x0DC */
	uint32_t video_input_ctrl;		/* 0x0E0 */
	uint32_t rsv_0xE4[3];		/* 0x0E4 ~ 0x0EC */
	uint32_t random_num_ctrl;	/* 0x0F0 */
	uint32_t random_num_data;	/* 0x0F4 */
	uint32_t rsv_0xF0[2];		/* 0x0F8 ~ 0x0FC */
	uint32_t rsv_0x100[64];		/* 0x100 ~ 0x1FC */
	uint32_t modrst1_ctrl;		/* 0x200 */
	uint32_t modrst1_clr;		/* 0x204 */
	uint32_t rsv_0x208[2];		/* 0x208 ~ 0x20C */
	uint32_t modrst_lock1;		/* 0x210 */
	uint32_t modrst1_sec1;		/* 0x214 */
	uint32_t modrst1_sec2;		/* 0x218 */
	uint32_t modrst1_sec3;		/* 0x21C */
	uint32_t modrst2_ctrl;		/* 0x220 */
	uint32_t modrst2_clr;		/* 0x224 */
	uint32_t rsv_0x228[2];		/* 0x228 ~ 0x22C */
	uint32_t modrst2_lock;		/* 0x230 */
	uint32_t modrst2_prot1;		/* 0x234 */
	uint32_t modrst2_prot2;		/* 0x238 */
	uint32_t modrst2_prot3;		/* 0x23C */
	uint32_t clkgate_ctrl1;		/* 0x240 */
	uint32_t clkgate_clr1;		/* 0x244 */
	uint32_t rsv_0x248[2];		/* 0x248 */
	uint32_t clkgate_lock1;		/* 0x250 */
	uint32_t clkgate_secure11;		/* 0x254 */
	uint32_t clkgate_secure12;		/* 0x258 */
	uint32_t clkgate_secure13;		/* 0x25c */
	uint32_t clkgate_ctrl2;		/* 0x260 */
	uint32_t clkgate_clr2;		/* 0x264 */
	uint32_t rsv_0x268[2];		/* 0x268 */
	uint32_t clkgate_lock2;		/* 0x270 */
	uint32_t clkgate_secure21;		/* 0x274 */
	uint32_t clkgate_secure22;		/* 0x278 */
	uint32_t clkgate_secure23;		/* 0x27c */
	uint32_t clk_sel1;		/* 0x280 */
	uint32_t clk_sel2;		/* 0x284 */
	uint32_t rsv_0x288[2];		/* 0x288 */
	uint32_t clk_sel1_lock;		/* 0x290 */
	uint32_t clk_sel2_lock;		/* 0x294 */
	uint32_t rsv_0x298[2];		/* 0x298 */
	uint32_t clk_sel1_secure1;		/* 0x2a0 */
	uint32_t clk_sel1_secure2;		/* 0x2a4 */
	uint32_t rsv_0x2a8[2];		/* 0x2a8 */
	uint32_t clk_sel2_secure1;		/* 0x2b0 */
	uint32_t clk_sel2_secure2;		/* 0x2b4 */
	uint32_t rsv_0x2b8[2];		/* 0x2b8 */
	uint32_t clk_sel3_secure1;		/* 0x2c0 */
	uint32_t clk_sel3_secure2;		/* 0x2c4 */
	uint32_t rsv_0x2c8[10];		/* 0x2c8 */
	uint32_t extrst_sel1;		/* 0x2f0 */
	uint32_t extrst_sel2;		/* 0x2f4 */
	uint32_t rsv_0x2f8[2];		/* 0x2f8 */
	uint32_t hpll;			/* 0x300 */
	uint32_t hpll_ext;		/* 0x304 */
	uint32_t rsv_0x308[2];		/* 0x308 ~ 0x30C */
	uint32_t apll;			/* 0x310 */
	uint32_t apll_ext;		/* 0x314 */
	uint32_t rsv_0x318[2];		/* 0x318 ~ 0x31C */
	uint32_t dpll;			/* 0x320 */
	uint32_t dpll_ext;		/* 0x324 */
	uint32_t rsv_0x328[2];		/* 0x328 ~ 0x32C */
	uint32_t uxclk_ctrl;		/* 0x330 */
	uint32_t huxclk_ctrl;		/* 0x334 */
	uint32_t rsv_0x338[18];		/* 0x338 ~ 0x37C */
	uint32_t clkduty_meas_ctrl;	/* 0x380 */
	uint32_t clkduty1;		/* 0x384 */
	uint32_t clkduty2;		/* 0x388 */
	uint32_t rsv_0x38c;		/* 0x38c */
	uint32_t mac_delay;		/* 0x390 */
	uint32_t mac_100m_delay;		/* 0x394 */
	uint32_t mac_10m_delay;		/* 0x398 */
	uint32_t rsv_0x39c;		/* 0x39c */
	uint32_t freq_counter_ctrl;	/* 0x3a0 */
	uint32_t freq_counter_cmp;	/* 0x3a4 */
	uint32_t rsv_0x3a8[2];		/* 0x3a8 ~ 0x3aC */
};

/*
 * Clock divider/multiplier configuration struct.
 * For H-PLL and M-PLL the formula is
 * (Output Frequency) = CLKIN * ((M + 1) / (N + 1)) / (P + 1)
 * M - Numerator
 * N - Denumerator
 * P - Post Divider
 * They have the same layout in their control register.
 *
 */
union ast10x0_g2_pll_reg {
	uint32_t w;
	struct {
		uint16_t m : 13;		/* bit[12:0]	*/
		uint8_t n : 6;			/* bit[18:13]	*/
		uint8_t p : 4;			/* bit[22:19]	*/
		uint8_t off : 1;		/* bit[23]	*/
		uint8_t bypass : 1;		/* bit[24]	*/
		uint8_t reset : 1;		/* bit[25]	*/
		uint8_t reserved : 6;		/* bit[31:26]	*/
	} b;
};

struct clock_ast10x0_g2_config {
	uintptr_t base;
};

static int
ast10x0_g2_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast10x0_g2_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	if (clk_gate > 32)
		sys_set_bit(config->base + CLK_STOP_CTRL1_CLEAR, clk_gate - 32);
	else
		sys_set_bit(config->base + CLK_STOP_CTRL0_CLEAR, clk_gate);

	return 0;
}

static int
ast10x0_g2_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast10x0_g2_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	if (clk_gate > 32)
		sys_set_bit(config->base + CLK_STOP_CTRL1_SET, clk_gate - 32);
	else
		sys_set_bit(config->base + CLK_STOP_CTRL0_SET, clk_gate);

	return 0;
}

static uint32_t ast10x0_g2_get_pll_rate(struct ast10x0_g2_scu *scu, int pll_idx)
{
	union ast10x0_g2_pll_reg pll_reg;
	uint32_t mul = 1, div = 1;

	switch (pll_idx) {
	case SCU1_CLK_HPLL:
		pll_reg.w = scu->hpll;
		break;
	case SCU1_CLK_APLL:
		pll_reg.w = scu->apll;
		break;
	case SCU1_CLK_DPLL:
		pll_reg.w = scu->dpll;
		break;
	}

	if (!pll_reg.b.bypass) {
		mul = (pll_reg.b.m + 1) / (pll_reg.b.n + 1);
		div = (pll_reg.b.p + 1);
	}

	return ((MHZ(25) * mul) / div);
}

#define SCU_CLKSEL2_I3CCLK_DIV_MASK		GENMASK(25, 23)
#define SCU_CLKSEL2_I3CCLK_DIV_SHIFT		23

static uint32_t ast10x0_g2_get_i3cclk_rate(struct ast10x0_g2_scu *scu)
{
	uint32_t rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_HPLL);
	uint32_t clk_sel2 = scu->clk_sel2;
	uint32_t i3cclk_div = (clk_sel2 & SCU_CLKSEL2_I3CCLK_DIV_MASK) >>
			      SCU_CLKSEL2_I3CCLK_DIV_SHIFT;

	if (!i3cclk_div)
		i3cclk_div = 2;
	else
		i3cclk_div++;

	return (rate / i3cclk_div);
}

static void ast10x0_g2_configure_i3c_clk(struct ast10x0_g2_scu *scu)
{
	uint32_t clk_sel2 = scu->clk_sel2;

	clk_sel2 &= ~SCU_CLKSEL2_I3CCLK_DIV_MASK;
	clk_sel2 |= FIELD_PREP(SCU_CLKSEL2_I3CCLK_DIV_MASK, 7); /* divide by 8 */
	scu->clk_sel2 = clk_sel2;
}

#define SCU_CLKSEL2_HCLK_DIV_MASK		GENMASK(22, 20)
#define SCU_CLKSEL2_HCLK_DIV_SHIFT		20

static uint32_t ast10x0_g2_get_hclk_rate(struct ast10x0_g2_scu *scu)
{
	uint32_t rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_HPLL);
	uint32_t clk_sel2 = scu->clk_sel2;
	uint32_t hclk_div = (clk_sel2 & SCU_CLKSEL2_HCLK_DIV_MASK) >>
			     SCU_CLKSEL2_HCLK_DIV_SHIFT;

	if (!hclk_div)
		hclk_div = 2;
	else
		hclk_div++;

	return (rate / hclk_div);
}

#define SCU1_CLKSEL1_PCLK_DIV_MASK		GENMASK(20, 18)
#define SCU1_CLKSEL1_PCLK_DIV_SHIFT		18

static uint32_t ast10x0_g2_get_pclk_rate(struct ast10x0_g2_scu *scu)
{
	uint32_t rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_HPLL);

	uint32_t clk_sel1 = scu->clk_sel1;
	uint32_t pclk_div = (clk_sel1 & SCU1_CLKSEL1_PCLK_DIV_MASK) >>
			     SCU1_CLKSEL1_PCLK_DIV_SHIFT;

	return (rate / ((pclk_div + 1) * 2));
}

#define SCU_UART_CLKGEN_N_MASK			GENMASK(17, 8)
#define SCU_UART_CLKGEN_N_SHIFT			8
#define SCU_UART_CLKGEN_R_MASK			GENMASK(7, 0)
#define SCU_UART_CLKGEN_R_SHIFT			0

static uint32_t ast10x0_g2_get_uart_uxclk_rate(struct ast10x0_g2_scu *scu)
{
	uint32_t uxclk_sel = scu->clk_sel2 & GENMASK(1, 0);
	uint32_t uxclk_ctrl = scu->uxclk_ctrl;
	uint32_t rate;

	switch (uxclk_sel) {
	case 0:
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_APLL) / 4;
		break;
	case 1:
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_APLL) / 2;
		break;
	case 2:
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_APLL);
		break;
	case 3:
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_HPLL);
		break;
	}

	uint32_t n = (uxclk_ctrl & SCU_UART_CLKGEN_N_MASK) >>
		      SCU_UART_CLKGEN_N_SHIFT;
	uint32_t r = (uxclk_ctrl & SCU_UART_CLKGEN_R_MASK) >>
		      SCU_UART_CLKGEN_R_SHIFT;

	return ((rate * r) / (n * 2));
}

#define SCU_HUART_CLKGEN_N_MASK			GENMASK(17, 8)
#define SCU_HUART_CLKGEN_N_SHIFT		8
#define SCU_HUART_CLKGEN_R_MASK			GENMASK(7, 0)
#define SCU_HUART_CLKGEN_R_SHIFT		0

static uint32_t ast10x0_g2_get_uart_huxclk_rate(struct ast10x0_g2_scu *scu)
{
	uint32_t huxclk_sel = (scu->clk_sel2 & GENMASK(4, 3)) >> 3;
	uint32_t huxclk_ctrl = scu->huxclk_ctrl;
	uint32_t n = (huxclk_ctrl & SCU_HUART_CLKGEN_N_MASK) >>
		      SCU_HUART_CLKGEN_N_SHIFT;
	uint32_t r = (huxclk_ctrl & SCU_HUART_CLKGEN_R_MASK) >>
		      SCU_HUART_CLKGEN_R_SHIFT;
	uint32_t rate;

	switch (huxclk_sel) {
	case 0:
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_APLL) / 4;
		break;
	case 1:
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_APLL) / 2;
		break;
	case 2:
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_APLL);
		break;
	case 3:
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_HPLL);
		break;
	}

	return ((rate * r) / (n * 2));
}

#define SCU_CLKSRC1_SDIO_DIV_MASK		GENMASK(16, 14)
#define SCU_CLKSRC1_SDIO_DIV_SHIFT		14
#define SCU_CLKSRC1_SDIO_SEL			BIT(13)
const int ast10x0_g2_sd_div_tbl[] = {
	2, 2, 3, 4, 5, 6, 7, 8
};

static uint32_t ast10x0_g2_get_sdio_clk_rate(struct ast10x0_g2_scu *scu)
{
	uint32_t rate = 0;
	uint32_t clk_sel1 = scu->clk_sel1;
	uint32_t div = (clk_sel1 & SCU_CLKSRC1_SDIO_DIV_MASK) >>
			     SCU_CLKSRC1_SDIO_DIV_SHIFT;

	if (clk_sel1 & SCU_CLKSRC1_SDIO_SEL)
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_APLL);
	else
		rate = ast10x0_g2_get_pll_rate(scu, SCU1_CLK_HPLL);

	if (!div)
		div = 1;

	div++;

	return (rate / div);
}

static uint32_t
ast10x0_g2_get_uart_clk_rate(struct ast10x0_g2_scu *scu, int uart_idx)
{
	uint32_t rate = 0;

	if (scu->clk_sel1 & BIT(uart_idx))
		rate = ast10x0_g2_get_uart_huxclk_rate(scu);
	else
		rate = ast10x0_g2_get_uart_uxclk_rate(scu);

	return rate;
}

static int ast10x0_g2_clock_control_get_rate(const struct device *dev,
					     clock_control_subsys_t sub_system, uint32_t *rate)
{
	const struct clock_ast10x0_g2_config *config = dev->config;
	struct ast10x0_g2_scu *scu = (struct ast10x0_g2_scu *)config->base;
	uint32_t clk_id = (uint32_t)sub_system;

	switch (clk_id) {
	case SCU1_CLK_HPLL:
	case SCU1_CLK_APLL:
	case SCU1_CLK_DPLL:
		*rate = ast10x0_g2_get_pll_rate(scu, clk_id);
		break;
	case SCU1_CLK_AHB:
		*rate = ast10x0_g2_get_hclk_rate(scu);
		break;
	case SCU1_CLK_APB:
		*rate = ast10x0_g2_get_pclk_rate(scu);
		break;
	case SCU1_CLK_GATE_UART0CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 0);
		break;
	case SCU1_CLK_GATE_UART1CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 1);
		break;
	case SCU1_CLK_GATE_UART2CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 2);
		break;
	case SCU1_CLK_GATE_UART3CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 3);
		break;
	case SCU1_CLK_GATE_UART5CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 5);
		break;
	case SCU1_CLK_GATE_UART6CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 6);
		break;
	case SCU1_CLK_GATE_UART7CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 7);
		break;
	case SCU1_CLK_GATE_UART8CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 8);
		break;
	case SCU1_CLK_GATE_UART9CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 9);
		break;
	case SCU1_CLK_GATE_UART10CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 10);
		break;
	case SCU1_CLK_GATE_UART11CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 11);
		break;
	case SCU1_CLK_GATE_UART12CLK:
		*rate = ast10x0_g2_get_uart_clk_rate(scu, 12);
		break;
	case SCU1_CLK_GATE_SDCLK:
		*rate = ast10x0_g2_get_sdio_clk_rate(scu);
		break;
	case SCU1_CLK_UXCLK:
		*rate = ast10x0_g2_get_uart_uxclk_rate(scu);
		break;
	case SCU1_CLK_HUXCLK:
		*rate = ast10x0_g2_get_uart_huxclk_rate(scu);
		break;
	case SCU1_CLK_GATE_I3C0CLK:
	case SCU1_CLK_GATE_I3C1CLK:
	case SCU1_CLK_GATE_I3C2CLK:
	case SCU1_CLK_GATE_I3C3CLK:
	case SCU1_CLK_GATE_I3C4CLK:
	case SCU1_CLK_GATE_I3C5CLK:
	case SCU1_CLK_GATE_I3C6CLK:
	case SCU1_CLK_GATE_I3C7CLK:
	case SCU1_CLK_GATE_I3C8CLK:
	case SCU1_CLK_GATE_I3C9CLK:
	case SCU1_CLK_GATE_I3C10CLK:
	case SCU1_CLK_GATE_I3C11CLK:
	case SCU1_CLK_GATE_I3C12CLK:
	case SCU1_CLK_GATE_I3C13CLK:
	case SCU1_CLK_GATE_I3C14CLK:
	case SCU1_CLK_GATE_I3C15CLK:
		*rate = ast10x0_g2_get_i3cclk_rate(scu);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int ast10x0_g2_clock_control_init(const struct device *dev)
{
	const struct clock_ast10x0_g2_config *config = dev->config;
	struct ast10x0_g2_scu *scu = (struct ast10x0_g2_scu *)config->base;

	ast10x0_g2_configure_i3c_clk(scu);

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
