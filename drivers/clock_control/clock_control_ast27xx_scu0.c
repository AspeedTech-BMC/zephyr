/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast27xx_scu0_clock
#include <errno.h>
#include <zephyr/dt-bindings/clock/ast27xx_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_ast27xx_scu0);

#define CLK_STOP_CTRL_SET		0x240
#define CLK_STOP_CTRL_CLEAR		0x244

struct ast2700_scu0 {
	uint32_t chip_id1;		/* 0x000 */
	uint32_t rsv_0x04[3];		/* 0x004 ~ 0x00C */
	uint32_t hwstrap1;		/* 0x010 */
	uint32_t hwstrap1_clr;		/* 0x014 */
	uint32_t rsv_0x18[2];		/* 0x018 ~ 0x01C */
	uint32_t hwstrap1_lock;		/* 0x020 */
	uint32_t hwstrap1_sec1;	/* 0x024 */
	uint32_t hwstrap1_sec2;	/* 0x028 */
	uint32_t hwstrap1_sec3;	/* 0x02C */
	uint32_t rsv_0x30[8];		/* 0x030 ~ 0x4C */
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
	uint32_t rsv_0x80[8];		/* 0x080 ~ 0x9C */
	uint32_t probe_sig_select;	/* 0x0A0 */
	uint32_t probe_sig_enable1;	/* 0x0A4 */
	uint32_t probe_sig_enable2; /* 0x0A8 */
	uint32_t uart_dbg_rate;		/* 0x0AC */
	uint32_t rsv_0xB0[4];		/* 0x0B0 ~ 0xBC*/
	uint32_t misc;			/* 0x0C0 */
	uint32_t rsv_0xC4;		/* 0x0C4 */
	uint32_t debug_ctrl;		/* 0x0C8 */
	uint32_t rsv_0xCC[5];		/* 0x0CC ~ 0x0DC */
	uint32_t free_counter_read_low;		/* 0x0E0 */
	uint32_t free_counter_read_high;	/* 0x0E4 */
	uint32_t rsv_0xE8[2];		/* 0x0E8 ~ 0x0EC */
	uint32_t random_num_ctrl;	/* 0x0F0 */
	uint32_t random_num_data;	/* 0x0F4 */
	uint32_t rsv_0xF8[10];		/* 0x0F8 ~ 0x11C */
	uint32_t ssp_ctrl_1;		/* 0x120 */
	uint32_t ssp_ctrl_2;		/* 0x124 */
	uint32_t ssp_ctrl_3;		/* 0x128 */
	uint32_t ssp_ctrl_4;		/* 0x12C */
	uint32_t ssp_ctrl_5;		/* 0x130 */
	uint32_t ssp_ctrl_6;		/* 0x134 */
	uint32_t ssp_ctrl_7;		/* 0x138 */
	uint32_t rsv_0x13c[1];		/* 0x13C */
	uint32_t ssp_remap0_base;	/* 0x140 */
	uint32_t ssp_remap0_size;	/* 0x144 */
	uint32_t ssp_remap1_base;	/* 0x148 */
	uint32_t ssp_remap1_size;	/* 0x14c */
	uint32_t ssp_remap2_base;	/* 0x150 */
	uint32_t ssp_remap2_size;	/* 0x154 */
	uint32_t rsv_0x158[2];		/* 0x158 ~ 0x15C */
	uint32_t tsp_ctrl_1;		/* 0x160 */
	uint32_t rsv_0x164[1];		/* 0x164 */
	uint32_t tsp_ctrl_3;		/* 0x168 */
	uint32_t tsp_ctrl_4;		/* 0x16C */
	uint32_t tsp_ctrl_5;		/* 0x170 */
	uint32_t tsp_ctrl_6;		/* 0x174 */
	uint32_t tsp_ctrl_7;		/* 0x178 */
	uint32_t rsv_0x17c[6];		/* 0x17C ~ 0x190 */
	uint32_t tsp_remap_size;	/* 0x194 */
	uint32_t rsv_0x198[26];		/* 0x198 ~ 0x1FC */
	uint32_t modrst1_ctrl;		/* 0x200 */
	uint32_t modrst1_clr;		/* 0x204 */
	uint32_t rsv_0x208[2];		/* 0x208 ~ 0x20C */
	uint32_t modrst1_lock;		/* 0x210 */
	uint32_t modrst1_prot1;		/* 0x214 */
	uint32_t modrst1_prot2;		/* 0x218 */
	uint32_t modrst1_prot3;		/* 0x21C */
	uint32_t modrst2_ctrl;		/* 0x220 */
	uint32_t modrst2_clr;		/* 0x224 */
	uint32_t rsv_0x228[2];		/* 0x228 ~ 0x22C */
	uint32_t modrst2_lock;		/* 0x230 */
	uint32_t modrst2_prot1;		/* 0x234 */
	uint32_t modrst2_prot2;		/* 0x238 */
	uint32_t modrst2_prot3;		/* 0x23C */
	uint32_t clkgate_ctrl;		/* 0x240 */
	uint32_t clkgate_clr;		/* 0x244 */
	uint32_t rsv_0x248[2];		/* 0x248 */
	uint32_t clkgate_lock;		/* 0x250 */
	uint32_t clkgate_secure1;	/* 0x254 */
	uint32_t clkgate_secure2;	/* 0x258 */
	uint32_t clkgate_secure3;	/* 0x25c */
	uint32_t rsv_0x260[8];		/* 0x260 */
	uint32_t clk_sel1;		/* 0x280 */
	uint32_t clk_sel2;		/* 0x284 */
	uint32_t clk_sel3;		/* 0x288 */
	uint32_t rsv_0x28c;		/* 0x28c */
	uint32_t clk_sel1_lock;		/* 0x290 */
	uint32_t clk_sel2_lock;		/* 0x294 */
	uint32_t clk_sel3_lock;		/* 0x298 */
	uint32_t rsv_0x29c;		/* 0x29c */
	uint32_t clk_sel1_secure1;	/* 0x2a0 */
	uint32_t clk_sel1_secure2;	/* 0x2a4 */
	uint32_t clk_sel1_secure3;	/* 0x2a8 */
	uint32_t rsv_0x2ac;		/* 0x2ac */
	uint32_t clk_sel2_secure1;	/* 0x2b0 */
	uint32_t clk_sel2_secure2;	/* 0x2b4 */
	uint32_t clk_sel2_secure3;	/* 0x2b8 */
	uint32_t rsv_0x2bc;		/* 0x2bc */
	uint32_t clk_sel3_secure1;	/* 0x2c0 */
	uint32_t clk_sel3_secure2;	/* 0x2c4 */
	uint32_t clk_sel3_secure3;	/* 0x2c8 */
	uint32_t rsv_0x2cc[9];		/* 0x2cc */
	uint32_t extrst_sel;		/* 0x2f0 */
	uint32_t rsv_0x2f4[3];		/* 0x2f4 */
	uint32_t hpll;			/* 0x300 */
	uint32_t hpll_ext;		/* 0x304 */
	uint32_t dpll;			/* 0x308 */
	uint32_t dpll_ext;		/* 0x30C */
	uint32_t mpll;			/* 0x310 */
	uint32_t mpll_ext;		/* 0x314 */
	uint32_t rsv_0x318[2];		/* 0x318 ~ 0x31C */
	uint32_t d1clk_para;		/* 0x320 */
	uint32_t rsv_0x324[3];		/* 0x324 ~ 0x32C */
	uint32_t d2clk_para;		/* 0x330 */
	uint32_t rsv_0x334[3];		/* 0x334 ~ 0x33C */
	uint32_t crt1clk_para;		/* 0x340 */
	uint32_t rsv_0x344[3];		/* 0x344 ~ 0x34C */
	uint32_t crt2clk_para;		/* 0x350 */
	uint32_t rsv_0x354[3];		/* 0x354 ~ 0x35C */
	uint32_t mphyclk_para;		/* 0x360 */
	uint32_t rsv_0x364[7];		/* 0x364 ~ 0x37C */
	uint32_t clkduty_meas_ctrl;	/* 0x380 */
	uint32_t clkduty1;		/* 0x384 */
	uint32_t clkduty2;		/* 0x368 */
	uint32_t clkduty_meas_res;	/* 0x38c */
	uint32_t rsv_0x390[4];		/* 0x390 ~ 0x39C */
	uint32_t freq_counter_ctrl;	/* 0x3a0 */
	uint32_t freq_counter_cmp;	/* 0x3a4 */
	uint32_t prog_delay_ring_ctrl0;	/* 0x3a8 */
	uint32_t prog_delay_ring_ctrl1;	/* 0x3ac */
	uint32_t freq_counter_readback;	/* 0x3b0 */
	uint32_t rsv_0x3b4[19];		/* 0x3b4 */
	uint32_t pinmux1;		/* 0x400 */
	uint32_t pinmux2;		/* 0x404 */
	uint32_t pinmux3;		/* 0x408 */
	uint32_t rsv_0x40c;		/* 0x40C */
	uint32_t pinmux4;		/* 0x410 */
	uint32_t vga_func_ctrl;		/* 0x414 */
	uint32_t rsv_0x418[314];	/* 0x418 ~ 0x8FC */
	uint32_t vga0_scratch1[4];	/* 0x900 ~ 0x90C */
	uint32_t vga1_scratch1[4];	/* 0x910 ~ 0x91C */
	uint32_t vga0_scratch2[8];	/* 0x920 ~ 0x93C */
	uint32_t vga1_scratch2[8];	/* 0x940 ~ 0x95C */
	uint32_t pci_cfg1[3];		/* 0x960 ~ 0x968 */
	uint32_t rsv_0x96c;		/* 0x96C */
	uint32_t pcie_cfg1;		/* 0x970 */
	uint32_t mmio_decode1;		/* 0x974 */
	uint32_t reloc_ctrl_decode1[2];	/* 0x978 ~ 0x97C */
	uint32_t rsv_0x980[4];		/* 0x980 ~ 0x98C */
	uint32_t mbox_decode1;		/* 0x990 */
	uint32_t shared_sram_decode1[2];/* 0x994 ~ 0x998 */
	uint32_t rsv_0x99c;		/* 0x99C */
	uint32_t pci_cfg2[3];		/* 0x9A0 ~ 0x9A8 */
	uint32_t rsv_0x9ac;		/* 0x9AC */
	uint32_t pcie_cfg2;		/* 0x9B0 */
	uint32_t mmio_decode2;		/* 0x9B4 */
	uint32_t reloc_ctrl_decode2[2];	/* 0x9B8 ~ 0x9BC */
	uint32_t rsv_0x9c0[4];		/* 0x9C0 ~ 0x9CC */
	uint32_t mbox_decode2;		/* 0x9D0 */
	uint32_t shared_sram_decode2[2];/* 0x9D4 ~ 0x9D8 */
	uint32_t rsv_0x9dc[9];		/* 0x9DC ~ 0x9FC */
	uint32_t pci0_misc[32];		/* 0xA00 ~ 0xA7C */
	uint32_t pci1_misc[32];		/* 0xA80 ~ 0xAFC */
};

struct clock_ast27xx_scu0_config {
	uintptr_t base;
};

static int
ast27xx_scu0_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast27xx_scu0_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	sys_set_bit(config->base + CLK_STOP_CTRL_CLEAR, clk_gate);

	return 0;
}

static int
ast27xx_scu0_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	const struct clock_ast27xx_scu0_config *config = dev->config;
	uint32_t clk_gate = (uint32_t)sub_system;

	sys_set_bit(config->base + CLK_STOP_CTRL_SET, clk_gate);

	return 0;
}

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
union ast2700_pll_reg {
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

#define SCU_HW_REVISION_ID		GENMASK(23, 16)
#define SCU_CPUCLK_MASK		GENMASK(4, 2)
#define SCU_CPUCLK_SHIFT	2
static uint32_t
ast2700_soc0_get_hpll_rate(struct ast2700_scu0 *scu)
{
	union ast2700_pll_reg pll_reg;
	uint32_t mul = 1, div = 1;
	uint32_t rate;

	pll_reg.w = scu->hpll;

	if ((scu->chip_id1 & SCU_HW_REVISION_ID) && (scu->hwstrap1 & BIT(3))) {
		switch ((scu->hwstrap1 & GENMASK(4, 2)) >> 2) {
		case 2:
			rate = 1800000000;
			break;
		case 3:
			rate = 1700000000;
			break;
		case 6:
			rate = 1200000000;
			break;
		case 7:
			rate = 800000000;
			break;
		default:
			rate = 1600000000;
		}
	} else if (scu->hwstrap1 & GENMASK(3, 2)) {
		switch ((scu->hwstrap1 & GENMASK(3, 2)) >> 2) {
		case 1U:
			rate = 1900000000;
			break;
		case 2U:
			rate = 1800000000;
			break;
		case 3U:
			rate = 1700000000;
			break;
		default:
			rate = 1600000000;
			break;
		}
	} else {
		if (pll_reg.b.bypass == 0U) {
			/* F = 25Mhz * [(M + 2) / 2 * (n + 1)] / (p + 1) */
			mul = (pll_reg.b.m + 1) / ((pll_reg.b.n + 1) * 2);
			div = (pll_reg.b.p + 1);
		}
		rate = ((MHZ(25) * mul) / div);
	}

	return rate;
}

static uint32_t ast2700_soc0_get_pll_rate(struct ast2700_scu0 *scu, int pll_idx)
{
	union ast2700_pll_reg pll_reg;
	uint32_t mul = 1, div = 1;
	uint32_t rate;

	switch (pll_idx) {
	case SCU0_CLK_DPLL:
		pll_reg.w = scu->dpll;
		break;
	case SCU0_CLK_MPLL:
		pll_reg.w = scu->mpll;
		break;
	default:
		return -EINVAL;
	}

	if (pll_reg.b.bypass == 0U) {
		if (pll_idx == SCU0_CLK_MPLL) {
			/* F = 25Mhz * [M / (n + 1)] / (p + 1) */
			mul = (pll_reg.b.m) / ((pll_reg.b.n + 1));
			div = (pll_reg.b.p + 1);
		} else {
			/* F = 25Mhz * [(M + 2) / 2 * (n + 1)] / (p + 1) */
			mul = (pll_reg.b.m + 1) / ((pll_reg.b.n + 1) * 2);
			div = (pll_reg.b.p + 1);
		}
	}

	rate = ((MHZ(25) * mul) / div);

	return rate;
}

/*
 * AST2700A1
 * SCU010[4:2]:
 * 000: CPUCLK=MPLL=1.6GHz (MPLL default setting with SCU310, SCU314)
 * 001: CPUCLK=HPLL=2.0GHz (HPLL default setting with SCU300, SCU304)
 * 010: CPUCLK=HPLL=1.8GHz (HPLL frequency is constance and is not controlled by SCU300, SCU304)
 * 011: CPUCLK=HPLL=1.7GHz (HPLL frequency is constance and is not controlled by SCU300, SCU304)
 * 100: CPUCLK=MPLL/2=800MHz (MPLL default setting with SCU310, SCU314)
 * 101: CPUCLK=HPLL/2=1.0GHz (HPLL default setting with SCU300, SCU304)
 * 110: CPUCLK=HPLL=1.2GHz (HPLL frequency is constance and is not controlled by SCU300, SCU304)
 * 111: CPUCLK=HPLL=800MHz (HPLL frequency is constance and is not controlled by SCU300, SCU304)
 */

static uint32_t ast2700_soc0_get_pspclk_rate(struct ast2700_scu0 *scu)
{
	uint32_t rate;
	int cpuclk_set;

	if (scu->chip_id1 & SCU_HW_REVISION_ID) {
		cpuclk_set = (scu->hwstrap1 & SCU_CPUCLK_MASK) >> SCU_CPUCLK_SHIFT;
		switch (cpuclk_set) {
		case 0:
			rate = ast2700_soc0_get_pll_rate(scu, SCU0_CLK_MPLL);
			break;
		case 1:
		case 2:
		case 3:
		case 6:
		case 7:
			rate = ast2700_soc0_get_hpll_rate(scu);
			break;
		case 4:
			rate = ast2700_soc0_get_pll_rate(scu, SCU0_CLK_MPLL) / 2;
			break;
		case 5:
			rate = ast2700_soc0_get_hpll_rate(scu) / 2;
			break;
		default:
			rate = ast2700_soc0_get_hpll_rate(scu);
			break;
		}
	} else {
		if (scu->hwstrap1 & BIT(4))
			rate = ast2700_soc0_get_hpll_rate(scu);
		else
			rate = ast2700_soc0_get_pll_rate(scu, SCU0_CLK_MPLL);
	}
	return rate;
}

static uint32_t ast2700_soc0_get_axi0clk_rate(struct ast2700_scu0 *scu)
{
	return ast2700_soc0_get_pspclk_rate(scu) / 2;
}

#define SCU_AHB_DIV_MASK		GENMASK(6, 5)
#define SCU_AHB_DIV_SHIFT		5
static uint32_t hclk_ast2700a1_div_table[] = {
	6, 5, 4, 7,
};

static uint32_t ast2700_soc0_get_hclk_rate(struct ast2700_scu0 *scu)
{
	uint32_t hwstrap1 = scu->hwstrap1;
	uint32_t src_clk;
	int div;

	if (scu->chip_id1 & SCU_HW_REVISION_ID) {
		if (hwstrap1 & BIT(7))
			src_clk = ast2700_soc0_get_hpll_rate(scu);
		else
			src_clk = ast2700_soc0_get_pll_rate(scu, SCU0_CLK_MPLL);

		div = (hwstrap1 & SCU_AHB_DIV_MASK) >> SCU_AHB_DIV_SHIFT;
		div = hclk_ast2700a1_div_table[div];
	} else {
		if (hwstrap1 & BIT(7))
			src_clk = ast2700_soc0_get_hpll_rate(scu);
		else
			src_clk = ast2700_soc0_get_pll_rate(scu, SCU0_CLK_MPLL);

		div = (hwstrap1 & SCU_AHB_DIV_MASK) >> SCU_AHB_DIV_SHIFT;

		if (!div)
			div = 4;
		else
			div = (div + 1) * 2;
	}
	return (src_clk / div);
}

static uint32_t ast2700_soc0_get_axi1clk_rate(struct ast2700_scu0 *scu)
{
	if (scu->chip_id1 & SCU_HW_REVISION_ID)
		return ast2700_soc0_get_pll_rate(scu, SCU0_CLK_MPLL) / 4;
	else
		return ast2700_soc0_get_hclk_rate(scu);
}

#define SCU0_CLKSEL1_PCLK_DIV_MASK		GENMASK(25, 23)
#define SCU0_CLKSEL1_PCLK_DIV_SHIFT		23

static uint32_t ast2700_soc0_get_pclk_rate(struct ast2700_scu0 *scu)
{
	uint32_t rate = ast2700_soc0_get_axi0clk_rate(scu);
	uint32_t clksel1 = scu->clk_sel1;
	int div;

	div = (clksel1 & SCU0_CLKSEL1_PCLK_DIV_MASK) >>
			    SCU0_CLKSEL1_PCLK_DIV_SHIFT;

	return (rate / ((div + 1) * 2));
}

#define SCU_CLKSRC1_EMMC_DIV_MASK		GENMASK(14, 12)
#define SCU_CLKSRC1_EMMC_DIV_SHIFT		12
#define SCU_CLKSRC1_EMMC_SEL			BIT(11)
static uint32_t ast2700_soc0_get_emmcclk_rate(struct ast2700_scu0 *scu)
{
	uint32_t clksel1 = scu->clk_sel1;
	uint32_t rate;
	int div;

	div = (clksel1 & SCU_CLKSRC1_EMMC_DIV_MASK) >> SCU_CLKSRC1_EMMC_DIV_SHIFT;

	if (clksel1 & SCU_CLKSRC1_EMMC_SEL)
		rate = ast2700_soc0_get_hpll_rate(scu) / 4;
	else
		rate = ast2700_soc0_get_pll_rate(scu, SCU0_CLK_MPLL) / 4;

	return (rate / ((div + 1) * 2));
}

static uint32_t ast2700_soc0_get_uartclk_rate(struct ast2700_scu0 *scu)
{
	uint32_t clksel2 = scu->clk_sel2;
	uint32_t div = 1;
	uint32_t rate;

	if (clksel2 & BIT(15))
		rate = 192000000;
	else
		rate = 24000000;

	if (clksel2 & BIT(30))
		div = 13;
	return (rate / div);
}

static int ast27xx_scu0_clock_control_get_rate(const struct device *dev,
					      clock_control_subsys_t sub_system, uint32_t *rate)
{
	const struct clock_ast27xx_scu0_config *config = dev->config;
	struct ast2700_scu0 *scu = (struct ast2700_scu0 *)config->base;
	uint32_t clk_id = (uint32_t)sub_system;

	switch (clk_id) {
	case SCU0_CLK_HPLL:
		*rate = ast2700_soc0_get_hpll_rate(scu);
		break;
	case SCU0_CLK_AHB:
		*rate = ast2700_soc0_get_hclk_rate(scu);
		break;
	case SCU0_CLK_APB:
		*rate = ast2700_soc0_get_pclk_rate(scu);
		break;
	case SCU0_CLK_GATE_EMMCCLK:
		*rate = ast2700_soc0_get_emmcclk_rate(scu);
		break;
	case SCU0_CLK_GATE_UART4CLK:
		*rate = ast2700_soc0_get_uartclk_rate(scu);
		break;
	case SCU0_CLK_AXI1:
		*rate = ast2700_soc0_get_axi1clk_rate(scu);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct clock_control_driver_api ast27xx_scu0_clk_api = {
	.on = ast27xx_scu0_clock_control_on,
	.off = ast27xx_scu0_clock_control_off,
	.get_rate = ast27xx_scu0_clock_control_get_rate,
};

static const struct clock_ast27xx_scu0_config clock_scu0_config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &clock_scu0_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &ast27xx_scu0_clk_api);
