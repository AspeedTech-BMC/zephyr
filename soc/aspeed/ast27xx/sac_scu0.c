/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <errno.h>

#include <zephyr/dt-bindings/clock/ast27xx_clock.h>
#include <zephyr/dt-bindings/reset/ast27xx_reset.h>
#include <zephyr/logging/log.h>

#include "sac_scu.h"

LOG_MODULE_REGISTER(sac_scu0);

/******************************************************************************
 *                       Aspeed SCU Clock Policy Callback                     *
 ******************************************************************************/
static int sac_scu0_clk_load(struct sac_ctrl *ctrl, struct sac_cfg_list *cfg)
{
	int i = 0;
	uint16_t *list = cfg->list;
	uint32_t clk_bit = 0;
	struct sac_reg_map *reg = ctrl->reg_map;

	for (i = 0; i < cfg->len; i++) {
		LOG_INF("SCU0 clk policy master %d list[%d]: %d", cfg->master, i, list[i]);
		clk_bit = list[i] % 32;

		if (list[i] > SCU0_CLK_GATE_RVAS1CLK) {
			LOG_WRN("Invalid policy (M: %d, S: %d)", cfg->master, list[i]);
			continue;
		}

		if (((reg[0].val | reg[1].val | reg[2].val) & BIT(clk_bit)) != 0) {
			LOG_WRN("Duplicated policy (M: %d, S: %d)", cfg->master, list[i]);
			continue;
		}

		reg[0].val |= CHECK_SCU_POLICY_SEC0(cfg->master) ? BIT(clk_bit) : 0;
		reg[1].val |= CHECK_SCU_POLICY_SEC1(cfg->master) ? BIT(clk_bit) : 0;
		reg[2].val |= CHECK_SCU_POLICY_SEC2(cfg->master) ? BIT(clk_bit) : 0;
	}

	LOG_DBG("clk_load: %x %x %x", reg[0].val, reg[1].val, reg[2].val);

	return 0;
}

/******************************************************************************
 *                       Aspeed SCU Reset Policy Callback                     *
 ******************************************************************************/
static int sac_scu0_rst_load(struct sac_ctrl *ctrl, struct sac_cfg_list *cfg)
{
	int i = 0;
	uint16_t *list = cfg->list;
	uint32_t idx = 0;
	uint32_t bit = 0;
	struct sac_reg_map *reg[] = {
		&ctrl->reg_map[0],
		&ctrl->reg_map[3],
	};

	for (i = 0; i < cfg->len; i++) {
		LOG_INF("SCU0 rst policy master %d list[%d]: %d", cfg->master, i, list[i]);
		idx = list[i] / 32;
		bit = list[i] % 32;

		if (list[i] > SCU0_RESET_VLINK) {
			LOG_WRN("Invalid policy (M: %d, S: %d)", cfg->master, list[i]);
			continue;
		}

		if (((reg[idx][0].val | reg[idx][1].val | reg[idx][2].val) & BIT(bit)) != 0) {
			LOG_WRN("Duplicated policy (M: %d, S: %d)", cfg->master, list[i]);
			continue;
		}

		reg[idx][0].val |= CHECK_SCU_POLICY_SEC0(cfg->master) ? BIT(bit) : 0;
		reg[idx][1].val |= CHECK_SCU_POLICY_SEC1(cfg->master) ? BIT(bit) : 0;
		reg[idx][2].val |= CHECK_SCU_POLICY_SEC2(cfg->master) ? BIT(bit) : 0;
	}

	LOG_DBG("rst_load: %x %x %x", reg[0][0].val, reg[0][1].val, reg[0][2].val);
	LOG_DBG("rst_load: %x %x %x", reg[1][0].val, reg[1][1].val, reg[1][2].val);

	return 0;
}

/******************************************************************************
 *                            Aspeed SCU0 Clock                               *
 ******************************************************************************/
static struct sac_cfg_list scu0_clk_cfg[] = {
	{
		SCU_PSP_GROUP,
		DT_PROP_LEN(SCU0_CLK_NODE, psp),
		(uint16_t[])DT_PROP(SCU0_CLK_NODE, psp)
	},
	{
		SCU_SEC_PSP_GROUP,
		DT_PROP_LEN(SCU0_CLK_NODE, sec_psp),
		(uint16_t[])DT_PROP(SCU0_CLK_NODE, sec_psp)
	},
	{
		SCU_SSP_GROUP,
		DT_PROP_LEN(SCU0_CLK_NODE, ssp),
		(uint16_t[])DT_PROP(SCU0_CLK_NODE, ssp)
	},
	{
		SCU_TSP_GROUP,
		DT_PROP_LEN(SCU0_CLK_NODE, tsp),
		(uint16_t[])DT_PROP(SCU0_CLK_NODE, tsp)
	},
	{
		SCU_PSP_SSP_GROUP,
		DT_PROP_LEN(SCU0_CLK_NODE, psp_ssp),
		(uint16_t[])DT_PROP(SCU0_CLK_NODE, psp_ssp)
	},
	{
		SCU_SSP_TSP_GROUP,
		DT_PROP_LEN(SCU0_CLK_NODE, ssp_tsp),
		(uint16_t[])DT_PROP(SCU0_CLK_NODE, ssp_tsp)
	},
	{
		SCU_BOOTMCU_GROUP,
		DT_PROP_LEN(SCU0_CLK_NODE, bmcu),
		(uint16_t[])DT_PROP(SCU0_CLK_NODE, bmcu)
	},
};

static struct sac_reg_map scu0_clk_reg_map[] = {
	{0x254, 0x0}, /* sec0 register */
	{0x258, 0x0}, /* sec1 register */
	{0x25C, 0x0}, /* sec2 register */
};

static struct sac_reg_map scu0_clk_lock_reg_map[] = {
	{0xE10, SCU_POLICY_CLK0_LOCK}, /* sec0/1/2 lock register */
};

static struct sac_ctrl scu0_clk_ctrl = {
	.base = DT_REG_ADDR(DT_PARENT(SCU0_CLK_NODE)),
	.cfg_num = ARRAY_SIZE(scu0_clk_cfg),
	.cfg = scu0_clk_cfg,
	.reg_num = ARRAY_SIZE(scu0_clk_reg_map),
	.reg_map = scu0_clk_reg_map,
	.lock_reg_num = ARRAY_SIZE(scu0_clk_lock_reg_map),
	.reg_lock_map = scu0_clk_lock_reg_map,
	.init_sac = NULL,
	.load_sac = sac_scu0_clk_load,
	.apply_sac = NULL,
};

/******************************************************************************
 *                            Aspeed SCU0 Reset                               *
 ******************************************************************************/
static struct sac_cfg_list scu0_rst_cfg[] = {
	{
		SCU_PSP_GROUP,
		DT_PROP_LEN(SCU0_RST_NODE, psp),
		(uint16_t[])DT_PROP(SCU0_RST_NODE, psp)
	},
	{
		SCU_SEC_PSP_GROUP,
		DT_PROP_LEN(SCU0_RST_NODE, sec_psp),
		(uint16_t[])DT_PROP(SCU0_RST_NODE, sec_psp)
	},
	{
		SCU_SSP_GROUP,
		DT_PROP_LEN(SCU0_RST_NODE, ssp),
		(uint16_t[])DT_PROP(SCU0_RST_NODE, ssp)
	},
	{
		SCU_TSP_GROUP,
		DT_PROP_LEN(SCU0_RST_NODE, tsp),
		(uint16_t[])DT_PROP(SCU0_RST_NODE, tsp)
	},
	{
		SCU_PSP_SSP_GROUP,
		DT_PROP_LEN(SCU0_RST_NODE, psp_ssp),
		(uint16_t[])DT_PROP(SCU0_RST_NODE, psp_ssp)
	},
	{
		SCU_SSP_TSP_GROUP,
		DT_PROP_LEN(SCU0_RST_NODE, ssp_tsp),
		(uint16_t[])DT_PROP(SCU0_RST_NODE, ssp_tsp)
	},
	{
		SCU_BOOTMCU_GROUP,
		DT_PROP_LEN(SCU0_RST_NODE, bmcu),
		(uint16_t[])DT_PROP(SCU0_RST_NODE, bmcu)
	},
};

static struct sac_reg_map scu0_rst_reg_map[] = {
	{0x214, 0x0}, /* rst0 sec0 register */
	{0x218, 0x0}, /* rst0 sec1 register */
	{0x21C, 0x0}, /* rst0 sec2 register */
	{0x234, 0x0}, /* rst1 sec0 register */
	{0x238, 0x0}, /* rst1 sec1 register */
	{0x23C, 0x0}, /* rst1 sec2 register */
};

static struct sac_reg_map scu0_rst_lock_reg_map[] = {
	{0xE10, SCU_POLICY_RESET_LOCK}, /* rst0/1 sec0/1/2 lock register */
};

static struct sac_ctrl scu0_rst_ctrl = {
	.base = DT_REG_ADDR(DT_PARENT(SCU0_RST_NODE)),
	.cfg_num = ARRAY_SIZE(scu0_rst_cfg),
	.cfg = scu0_rst_cfg,
	.reg_num = ARRAY_SIZE(scu0_rst_reg_map),
	.reg_map = scu0_rst_reg_map,
	.lock_reg_num = ARRAY_SIZE(scu0_rst_lock_reg_map),
	.reg_lock_map = scu0_rst_lock_reg_map,
	.init_sac = NULL,
	.load_sac = sac_scu0_rst_load,
	.apply_sac = NULL,
};

/******************************************************************************
 *                            Aspeed SCU0 Reset                               *
 ******************************************************************************/
static struct sac_reg_map scu0_clk_sel_lock_reg_map[] = {
	{0x290, SCU_POLICY_CLK0_SEL1_LOCK}, /* clk sel1 lock register */
	{0x294, SCU_POLICY_CLK0_SEL2_LOCK}, /* clk sel2 lock register */
	{0x298, SCU_POLICY_CLK0_SEL3_LOCK}, /* clk sel3 lock register */
};

static struct sac_ctrl scu0_clk_sel_ctrl = {
	.base = DT_REG_ADDR(DT_PARENT(SCU0_CLK_NODE)),
	.lock_reg_num = ARRAY_SIZE(scu0_clk_sel_lock_reg_map),
	.reg_lock_map = scu0_clk_sel_lock_reg_map,
};

/******************************************************************************
 *                          Aspeed SCU0 Initialization                        *
 ******************************************************************************/
static int sac_scu0_init(void)
{
	int ret = 0;

	ret = sac_aspeed_enable(&scu0_clk_ctrl);
	if (ret)
		LOG_ERR("SCU0 clock policy enable fail(%d).", ret);

	ret = sac_aspeed_enable(&scu0_rst_ctrl);
	if (ret)
		LOG_ERR("SCU0 reset policy enable fail(%d).", ret);

	ret = sac_aspeed_enable(&scu0_clk_sel_ctrl);
	if (ret)
		LOG_ERR("SCU0 reset policy enable fail(%d).", ret);

	return ret;
}

SYS_INIT(sac_scu0_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
