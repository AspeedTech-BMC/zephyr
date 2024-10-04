/*
 * Copyright (c) 2021 - 2023 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/misc/aspeed/pfr_aspeed.h>
#include <soc.h>
#include <zephyr/kernel.h>

static int bmc_rst_demo(const struct shell *shell, size_t argc, char *argv[])
{

	pfr_bmc_extrst_enable_ctrl(true);
	pfr_bmc_extrst_enable_ctrl(false);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(pfr_cmds,
	SHELL_CMD_ARG(rst_bmc, NULL, "reset BMC demo", bmc_rst_demo, 1, 0),

	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pfr, &pfr_cmds, "PFR shell commands", NULL);

