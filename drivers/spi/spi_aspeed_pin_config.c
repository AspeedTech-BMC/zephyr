/*
 * Copyright (c) 2021 - 2023 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/misc/aspeed/pfr_aspeed.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(spi_aspeed, CONFIG_SPI_LOG_LEVEL);

#include "spi_aspeed.h"

#define FMC_CTRL_BASE               (0x14000000)
#define SPI0_CTRL_BASE              (0x14010000)
#define SPI1_CTRL_BASE              (0x14020000)

#define FMC_PINCTRL_SCU_REG         0x14c02450
#define SPI0_PINCTRL_SCU_REG        0x14c02434
#define SPI1_PINCTRL_SCU_REG        0x14c02438
#define SCU1_REG                    0x14c02000
#define ASPEED_IO_FWSPI_DRIVING     (SCU1_REG + 0x4E0)
#define ASPEED_IO_SPI0_DRIVING      (SCU1_REG + 0x4CC)
#define ASPEED_IO_SPI1_DRIVING      (SCU1_REG + 0x4CC)
#define ASPEED_IO_SPI2_DRIVING      (SCU1_REG + 0x4D0)

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

static void ast2700_spi_pinctrl_early_init(const struct device *dev)
{
	const struct aspeed_spi_config *config = dev->config;
	uint32_t reg_val;

	switch (config->ctrl_base) {
	case SPI0_CTRL_BASE:
		reg_val = sys_read32(SPI0_PINCTRL_SCU_REG);
		reg_val &= ~0x00000fff;
		reg_val |= BIT(0) | BIT(4) | BIT(8);
		sys_write32(reg_val, SPI0_PINCTRL_SCU_REG);
		break;

	case SPI1_CTRL_BASE:
		reg_val = sys_read32(SPI1_PINCTRL_SCU_REG);
		reg_val &= ~0x00f00fff;
		reg_val |= BIT(0) | BIT(4) | BIT(8);
		if (config->max_cs >= 2) {
			reg_val |= BIT(20);
		}
		sys_write32(reg_val, SPI1_PINCTRL_SCU_REG);
		break;

	default:
		break;
	}
}

static void ast2700_spi_pinctrl_post_init(const struct device *dev,
					  uint32_t max_bus_width)
{
	const struct aspeed_spi_config *config = dev->config;
	uint32_t reg_val;

	switch (config->ctrl_base) {
	case FMC_CTRL_BASE:
		if (max_bus_width > 2) {
			reg_val = sys_read32(FMC_PINCTRL_SCU_REG);
			reg_val &= ~0xff000000;
			reg_val |= BIT(24) | BIT(28);
			sys_write32(reg_val, FMC_PINCTRL_SCU_REG);
		}
		break;

	case SPI0_CTRL_BASE:
		if (max_bus_width > 2) {
			reg_val = sys_read32(SPI0_PINCTRL_SCU_REG);
			reg_val &= ~0x000ff000;
			reg_val |= BIT(12) | BIT(16);
			sys_write32(reg_val, SPI0_PINCTRL_SCU_REG);
		}
		break;

	case SPI1_CTRL_BASE:
		if (max_bus_width > 2) {
			reg_val = sys_read32(SPI1_PINCTRL_SCU_REG);
			reg_val &= ~0x000ff000;
			reg_val |= BIT(12) | BIT(16);
			sys_write32(reg_val, SPI1_PINCTRL_SCU_REG);
		}
		break;

	default:
		break;
	}
}

static void ast2700_spi_adjust_driving_strength(void)
{
	uint32_t reg;

	/* FMC driving strength: SCUIO_4E0[15:0] */
	reg = sys_read32(ASPEED_IO_FWSPI_DRIVING);
	reg &= ~(0x0000ffff);
	reg |= 0x0000aaaa;
	sys_write32(reg, ASPEED_IO_FWSPI_DRIVING);

	/* SPI0 driving strength: SCUIO_4CC[11:0] */
	reg = sys_read32(ASPEED_IO_SPI0_DRIVING);
	reg &= ~(0x00000fff);
	reg |= 0x00000aaa;
	sys_write32(reg, ASPEED_IO_SPI0_DRIVING);

	/* SPI1 driving strength: SCUIO_4CC[27:16] */
	reg = sys_read32(ASPEED_IO_SPI1_DRIVING);
	reg &= ~(0x0fff0000);
	reg |= 0x0aaa0000;
	sys_write32(reg, ASPEED_IO_SPI1_DRIVING);

	/* SPI2 driving strength: SCUIO_4D0[15:0] */
	reg = sys_read32(ASPEED_IO_SPI2_DRIVING);
	reg &= ~(0x0000ffff);
	reg |= 0x00002aaa;
	sys_write32(reg, ASPEED_IO_SPI2_DRIVING);
}

int ast2700_spi_lite_pinctrl_init(const struct device *dev)
{
	ast2700_spi_pinctrl_early_init(dev);
	ast2700_spi_adjust_driving_strength();

	return 0;
}

void ast2700_spi_lite_pinctrl_post_init(const struct device *dev,
					 uint32_t max_bus_width)
{
	ast2700_spi_pinctrl_post_init(dev, max_bus_width);
}

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

static void aspeed_ast1060_spim_proprietary_post_config(void)
{
	uint32_t scu0f0_val;
	uint32_t spim_idx;
	uint32_t reg_val;
	uint32_t op_idx;

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
