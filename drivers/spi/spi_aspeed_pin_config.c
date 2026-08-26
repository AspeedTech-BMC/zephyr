/*
 * Copyright (c) 2021 - 2023 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/misc/aspeed/pfr_aspeed.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(spi_aspeed, CONFIG_SPI_LOG_LEVEL);

#include "spi_aspeed.h"

#define SPIM_GPIO_INFO(__scu_reg__, __scu_bit__, __gpio_reg__, __gpio_bit__)	\
	{									\
		.scu_reg_addr = __scu_reg__,				\
		.scu_bit_mask = __scu_bit__,				\
		.gpio_addr = __gpio_reg__,				\
		.gpio_bit_mask = __gpio_bit__,				\
		.gpio_ori_val = 0,					\
	}

/*
 * SPIM1CLKOUT: SCU690[7]   GPIO_A7 0x7e780000[7]
 * SPIM2CLKOUT: SCU690[21]  GPIO_C5 0x7e780000[21]
 * SPIM3CLKOUT: SCU694[3]   GPIO_E3 0x7e780020[3]
 * SPIM4CLKOUT: SCU694[17]  GPIO_G1 0x7e780020[17]
 */
static struct spim_gpio_info g_ast1060_spim_clk_gpio[4] = {
	SPIM_GPIO_INFO(0x7e6e2690, BIT(7), 0x7e780000, BIT(7)),
	SPIM_GPIO_INFO(0x7e6e2690, BIT(21), 0x7e780000, BIT(21)),
	SPIM_GPIO_INFO(0x7e6e2694, BIT(3), 0x7e780020, BIT(3)),
	SPIM_GPIO_INFO(0x7e6e2694, BIT(17), 0x7e780020, BIT(17)),
};

/*
 * SPIM1CSOUT: SCU690[1]   GPIO_A6 0x7e780000[6]
 * SPIM2CSOUT: SCU690[20]  GPIO_C4 0x7e780000[20]
 * SPIM3CSOUT: SCU694[2]   GPIO_E2 0x7e780020[2]
 * SPIM4CSOUT: SCU694[16]  GPIO_G0 0x7e780020[16]
 */
static struct spim_gpio_info g_ast1060_spim_cs_gpio[4] = {
	SPIM_GPIO_INFO(0x7e6e2690, BIT(1), 0x7e780000, BIT(6)),
	SPIM_GPIO_INFO(0x7e6e2690, BIT(20), 0x7e780000, BIT(20)),
	SPIM_GPIO_INFO(0x7e6e2694, BIT(2), 0x7e780020, BIT(2)),
	SPIM_GPIO_INFO(0x7e6e2694, BIT(16), 0x7e780020, BIT(16)),
};

static void aspeed_ast1060_spim_proprietary_pre_config(void)
{
	uint32_t scu0f0_val;
	uint32_t spim_idx;
	uint32_t reg_val;
	uint32_t op_idx;

	scu0f0_val = sys_read32(0x7e6e20f0);
	/* configure SPI CLK pin to GPIO input */
	if ((scu0f0_val & 0x7) == 0) {
		return;
	}

	spim_idx = (scu0f0_val & 0x7) - 1;
	if (spim_idx == 2) {
		op_idx = 3;
	} else if (spim_idx == 3) {
		op_idx = 2;
	} else {
		return;
	}

	/* change multiple function pin to GPIO mode. */
	reg_val = sys_read32(g_ast1060_spim_clk_gpio[op_idx].scu_reg_addr);
	reg_val &= ~(g_ast1060_spim_clk_gpio[op_idx].scu_bit_mask);
	sys_write32(reg_val, g_ast1060_spim_clk_gpio[op_idx].scu_reg_addr);

	/* change GPIO related to spim clk output pin to input mode. */
	reg_val = sys_read32(g_ast1060_spim_clk_gpio[op_idx].gpio_addr + 0x4);
	g_ast1060_spim_clk_gpio[op_idx].gpio_ori_val =
		(reg_val & g_ast1060_spim_clk_gpio[op_idx].gpio_bit_mask);
	reg_val &= ~(g_ast1060_spim_clk_gpio[op_idx].gpio_bit_mask);
	sys_write32(reg_val, g_ast1060_spim_clk_gpio[op_idx].gpio_addr + 0x4);

	/* change GPIO related to spim CS output pin to output mode. */
	reg_val = sys_read32(g_ast1060_spim_cs_gpio[op_idx].gpio_addr);
	reg_val |= g_ast1060_spim_cs_gpio[op_idx].gpio_bit_mask;
	sys_write32(reg_val, g_ast1060_spim_cs_gpio[op_idx].gpio_addr);

	reg_val = sys_read32(g_ast1060_spim_cs_gpio[op_idx].gpio_addr + 0x4);
	reg_val |= g_ast1060_spim_cs_gpio[op_idx].gpio_bit_mask;
	sys_write32(reg_val, g_ast1060_spim_cs_gpio[op_idx].gpio_addr + 0x4);

	/* change multiple function pin to GPIO mode. */
	reg_val = sys_read32(g_ast1060_spim_cs_gpio[op_idx].scu_reg_addr);
	reg_val &= ~(g_ast1060_spim_cs_gpio[op_idx].scu_bit_mask);
	sys_write32(reg_val, g_ast1060_spim_cs_gpio[op_idx].scu_reg_addr);
}

static void aspeed_ast1060_spim_proprietary_post_config(const struct device *dev, uint32_t cs)
{
	uint32_t scu0f0_val;
	uint32_t spim_idx;
	uint32_t reg_val;
	uint32_t op_idx;

	ARG_UNUSED(dev);
	ARG_UNUSED(cs);

	scu0f0_val = sys_read32(0x7e6e20f0);
	if ((scu0f0_val & 0x7) == 0) {
		return;
	}

	spim_idx = (scu0f0_val & 0x7) - 1;
	if (spim_idx == 2) {
		op_idx = 3;
	} else if (spim_idx == 3) {
		op_idx = 2;
	} else {
		return;
	}

	/* restore its GPIO value related to spim clk output pin. */
	reg_val = sys_read32(g_ast1060_spim_clk_gpio[op_idx].gpio_addr + 0x4);
	reg_val &= ~(g_ast1060_spim_clk_gpio[op_idx].gpio_bit_mask);
	reg_val |= g_ast1060_spim_clk_gpio[op_idx].gpio_ori_val;
	sys_write32(reg_val, g_ast1060_spim_clk_gpio[op_idx].gpio_addr + 0x4);

	/* change multiple function pin back to SPIM mode. */
	reg_val = sys_read32(g_ast1060_spim_clk_gpio[op_idx].scu_reg_addr);
	reg_val |= g_ast1060_spim_clk_gpio[op_idx].scu_bit_mask;
	sys_write32(reg_val, g_ast1060_spim_clk_gpio[op_idx].scu_reg_addr);

	/* change multiple function pin back to SPIM mode. */
	reg_val = sys_read32(g_ast1060_spim_cs_gpio[op_idx].scu_reg_addr);
	reg_val |= g_ast1060_spim_cs_gpio[op_idx].scu_bit_mask;
	sys_write32(reg_val, g_ast1060_spim_cs_gpio[op_idx].scu_reg_addr);
}

void ast1060_spi_proprietary_config_init(const struct aspeed_spi_config *config,
					 struct aspeed_spi_data *data)
{
	if (!config->aspeed_spim_proprietary_config_enable) {
		return;
	}

	data->aspeed_spim_proprietary_pre_config =
		aspeed_ast1060_spim_proprietary_pre_config;
	data->aspeed_spim_proprietary_post_config =
		aspeed_ast1060_spim_proprietary_post_config;
}

/*
 * AST1080 SPI0 pin-mux setup.
 *
 * Only a devicetree node with the `cs-group-analog-mux-enable` property
 * set (the AST1080 SPI0 node) actually wires the CS0/CS1 <-> CS2/CS3
 * analog mux; every other instance (AST1040 SPI0, FMC, SPI1) leaves it
 * unset and these stay no-ops there.
 */
#define AST10X0_G2_SCU0D0              0x74C020D0
#define AST10X0_G2_SCU0D4              0x74C020D4

void ast10x0_g2_proprietary_post_config(const struct device *dev, uint32_t cs)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cs);

	uint32_t reg_val;

	reg_val = sys_read32(AST10X0_G2_SCU0D4);
	reg_val |= (BIT(0) | BIT(1) | BIT(2));
	sys_write32(reg_val, AST10X0_G2_SCU0D4);
}

void ast10x0_g2_spi_proprietary_config_init(const struct aspeed_spi_config *config,
					 struct aspeed_spi_data *data)
{
	data->aspeed_spim_proprietary_post_config = ast10x0_g2_proprietary_post_config;

	if (!config->cs_group_analog_mux_enable) {
		return;
	}

	/* SPIF0 pins: disable pull-up and pull-down */
	sys_write32(0x02050201, 0x74c025FC);
	sys_write32(0x02040205, 0x74c02600);
	sys_write32(0x02050205, 0x74c025CC);
	sys_write32(0x02050205, 0x74c025D0);
	sys_write32(0x02040205, 0x74c025D4);

	/* SPIF1 pins: disable pull-up and pull-down */
	sys_write32(0x02050205, 0x74c024d8);
	sys_write32(0x02050205, 0x74c024dc);
	sys_write32(0x02050205, 0x74c024e0);
	sys_write32(0x02040205, 0x74c024e4);

	/* SPIF2 pins: disable pull-up and pull-down */
	sys_write32(0x02050201, 0x74c025F8);
	sys_write32(0x02050205, 0x74c025FC);
	sys_write32(0x02050205, 0x74c025A4);
	sys_write32(0x02050205, 0x74c025A8);
	sys_write32(0x02040205, 0x74c025AC);
}

/*
 * Select the CS0/CS1 or CS2/CS3 analog mux group ahead of a SPI0
 * transaction on the given cs. SCU0D0 is the analog mux mode register
 * (7 bits per group) and SCU0D4 is the SPI mode register (1 bit per
 * group). The two groups are mutually exclusive, so the inactive
 * group's bits are always cleared.
 */
void ast10x0_g2_spi_cs_group_select(const struct device *dev, uint32_t cs)
{
	const struct aspeed_spi_config *config = dev->config;
	uint32_t group = cs >> 1;
	uint32_t reg_val;

	if (!config->cs_group_analog_mux_enable) {
		return;
	}

	/*
	 * SCU0D0 (analog mux mode) is intentionally left untouched here;
	 * it's owned by the SPI monitor's ext-mux control instead (see
	 * ast1080_ext_mux_config() in spi_monitor_aspeed.c) to avoid the
	 * two drivers racing on the same register.
	 */

	reg_val = sys_read32(AST10X0_G2_SCU0D4);
	reg_val &= ~(BIT(0) | BIT(1));
	reg_val |= BIT(group);
	sys_write32(reg_val, AST10X0_G2_SCU0D4);
}
