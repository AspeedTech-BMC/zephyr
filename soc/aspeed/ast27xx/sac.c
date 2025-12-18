/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <errno.h>
#include <zephyr/logging/log.h>

#include "sac.h"

LOG_MODULE_REGISTER(sac_core);

static int sac_aspeed_init(struct sac_ctrl *ctrl)
{
	if (ctrl->init_sac)
		return ctrl->init_sac(ctrl);

	return 0;
}

static int sac_aspeed_load(struct sac_ctrl *ctrl)
{
	int cfg_idx = 0;
	struct sac_cfg_list *cfg = ctrl->cfg;

	if (ctrl->load_sac) {
		for (cfg_idx = 0; cfg_idx < ctrl->cfg_num; cfg_idx++) {
			ctrl->load_sac(ctrl, &cfg[cfg_idx]);
		}
	}

	return 0;
}

static int sac_aspeed_apply(struct sac_ctrl *ctrl)
{
	int i = 0;
	struct sac_reg_map *reg_map = ctrl->reg_map;
	struct sac_reg_map *reg_lock_map = ctrl->reg_lock_map;

	if (ctrl->apply_sac)
		return ctrl->apply_sac(ctrl);

	for (i = 0; i < ctrl->reg_num; i++)
		sys_write32(reg_map[i].val, ctrl->base + reg_map[i].ofst);

	for (i = 0; i < ctrl->lock_reg_num; i++)
		sys_write32(reg_lock_map[i].val, ctrl->base + reg_lock_map[i].ofst);

	return 0;
}

int sac_aspeed_enable(struct sac_ctrl *ctrl)
{
	int ret = 0;

	if (!ctrl)
		return -EINVAL;

	ret = sac_aspeed_init(ctrl);
	if (ret)
		goto exit;

	ret = sac_aspeed_load(ctrl);
	if (ret)
		goto exit;

	ret = sac_aspeed_apply(ctrl);
	if (ret)
		goto exit;

exit:
	return ret;
}
