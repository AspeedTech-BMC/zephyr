/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_SOC_ASPEED_AST27XX_SAC_SCU_H_
#define ZEPHYR_SOC_ASPEED_AST27XX_SAC_SCU_H_

#include "sac.h"

#include <zephyr/device.h>

/* Common secure register definition */
#define CHECK_SCU_POLICY_SEC(_sec, _grp) !!(((_grp) + 1) & BIT(_sec))
#define CHECK_SCU_POLICY_SEC0(_grp)      CHECK_SCU_POLICY_SEC(0, _grp)
#define CHECK_SCU_POLICY_SEC1(_grp)      CHECK_SCU_POLICY_SEC(1, _grp)
#define CHECK_SCU_POLICY_SEC2(_grp)      CHECK_SCU_POLICY_SEC(2, _grp)

/* SCU0 device node definition */
#define SCU0_CLK_NODE DT_NODELABEL(sysclk0)
#define SCU0_RST_NODE DT_NODELABEL(sysrst0)
#define SCU1_CLK_NODE DT_NODELABEL(sysclk1)
#define SCU1_RST_NODE DT_NODELABEL(sysrst1)

/* SCU0 poliicy operation definition */
#define SCU_POLICY_CLK0_LOCK (GENMASK(23, 21))
#define SCU_POLICY_CLK1_LOCK (GENMASK(23, 21) | GENMASK(31, 29))
#define SCU_POLICY_RESET_LOCK (GENMASK(15, 13) | GENMASK(7, 5))
#define SCU_POLICY_CLK0_SEL1_LOCK GENMASK(15, 0)
#define SCU_POLICY_CLK0_SEL2_LOCK GENMASK(12, 0)
#define SCU_POLICY_CLK0_SEL3_LOCK GENMASK(1, 0)
#define SCU_POLICY_CLK1_SEL1_LOCK (GENMASK(14, 13) | BIT(18) | BIT(21) | \
				   BIT(25) | BIT(29))
#define SCU_POLICY_CLK1_SEL2_LOCK (BIT(0) | BIT(3) | BIT(8) | BIT(12) | \
				   GENMASK(20, 15) | BIT(23))

enum {
	SCU_SEC_PSP_GROUP = 0,
	SCU_SSP_GROUP,
	SCU_PSP_GROUP,
	SCU_TSP_GROUP,
	SCU_PSP_SSP_GROUP,
	SCU_SSP_TSP_GROUP,
	SCU_BOOTMCU_GROUP,
};

#endif /* ZEPHYR_SOC_ASPEED_AST27XX_SAC_SCU_H_ */
