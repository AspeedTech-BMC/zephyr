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

/* SCU0 poliicy operation definition */
#define SYS_POLICY_CLK0_LOCK (GENMASK(23, 21))

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
