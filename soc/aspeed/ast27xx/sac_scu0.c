/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <errno.h>

#include <zephyr/dt-bindings/clock/ast27xx_clock.h>
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
		LOG_INF("SCU0 policy master %d list[%d]: %d", cfg->master, i, list[i]);
		clk_bit = list[i] % 32;

		if (list[i] > SCU0_CLK_GATE_RVAS1CLK) {
			LOG_WRN("Invalid policy (%d: %d)", cfg->master, list[i]);
			continue;
		}

		if (((reg[0].val | reg[1].val | reg[2].val) & BIT(clk_bit)) != 0) {
			LOG_WRN("Duplicated policy (%d: %d)", cfg->master, list[i]);
			continue;
		}

		reg[0].val |= CHECK_SCU_POLICY_SEC0(cfg->master) ? BIT(clk_bit) : 0;
		reg[1].val |= CHECK_SCU_POLICY_SEC1(cfg->master) ? BIT(clk_bit) : 0;
		reg[2].val |= CHECK_SCU_POLICY_SEC2(cfg->master) ? BIT(clk_bit) : 0;
	}

	return 0;
}

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
	{0xE10, SYS_POLICY_CLK0_LOCK}, /* sec0/1/2 lock register */
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

static int sac_scu0_init(void)
{
	int ret = 0;

	ret = sac_aspeed_enable(&scu0_clk_ctrl);
	if (ret)
		LOG_ERR("SCU0 clock policy enable fail(%d).", ret);

	return ret;
}

SYS_INIT(sac_scu0_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
