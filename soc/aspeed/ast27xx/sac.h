/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_SOC_ASPEED_AST27XX_SAC_H_
#define ZEPHYR_SOC_ASPEED_AST27XX_SAC_H_

#include <zephyr/sys/util.h>

struct sac_cfg_list {
	uint8_t master;
	uint16_t len;
	uint16_t *list;
};

struct sac_reg_map {
	uint16_t ofst;
	uint32_t val;
};

struct sac_ctrl {
	/* Hardware base address */
	uintptr_t base;

	/* Master configuration */
	uint16_t cfg_num;
	struct sac_cfg_list *cfg;

	/* Register map configuration*/
	uint16_t reg_num;
	struct sac_reg_map *reg_map;

	/* Lock configuration */
	uint16_t lock_reg_num;
	struct sac_reg_map *reg_lock_map;

	/* Callback function */
	int (*init_sac)(struct sac_ctrl *ctrl);
	int (*load_sac)(struct sac_ctrl *ctrl, struct sac_cfg_list *cfg);
	int (*apply_sac)(struct sac_ctrl *ctrl);
};

int sac_aspeed_enable(struct sac_ctrl *ctrl);

#endif /* ZEPHYR_SOC_ASPEED_AST27XX_SAC_H_ */
