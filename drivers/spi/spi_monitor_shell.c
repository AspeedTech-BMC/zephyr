/*
 * Copyright (c) 2021 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <shell/shell.h>
#include <sys/util.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>
#include <soc.h>
#include <kernel.h>

static const struct device *spim_device;

static int probe_parse_helper(const struct shell *shell, size_t *argc,
		char **argv[], const struct device **spim_dev)
{
	*spim_dev = device_get_binding((*argv)[1]);
	if (!*spim_dev) {
		shell_error(shell, "SPI monitor device/driver is not found!");
		return -ENODEV;
	}

	return 0;
}

static int cmd_parse_helper(const struct shell *shell, size_t *argc,
		char **argv[], uint8_t *cmd)
{
	char *endptr;

	if (*argc < 2) {
		shell_error(shell, "Missing command.");
		return -EINVAL;
	}

	*cmd = strtoul((*argv)[1], &endptr, 16);

	return 0;
}

static int cmd_probe(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;

	ret = probe_parse_helper(shell, &argc, &argv, &spim_device);
	if (ret)
		goto end;

	shell_print(shell, "SPI monitor device, %s, is found!", spim_device->name);

end:
	return ret;
}

static int dump_valid_cmd_table(const struct shell *shell, size_t argc, char *argv[])
{
	if (spim_device == NULL) {
		shell_error(shell, "Please probe the device first");
		return -ENODEV;
	}

	spim_dump_valid_command_table(spim_device);

	return 0;
}

static int add_valid_cmd(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	uint8_t cmd = 0;

	if (spim_device == NULL) {
		shell_error(shell, "Please probe the device first");
		return -ENODEV;
	}

	ret = cmd_parse_helper(shell, &argc, &argv, &cmd);
	if (ret)
		goto end;

	if (argc == 3 && strncmp(argv[2], "once", 4) == 0)
		ret = spim_add_valid_command(spim_device, cmd, FLAG_CMD_TABLE_VALID_ONCE);
	else
		ret = spim_add_valid_command(spim_device, cmd, FLAG_CMD_TABLE_VALID);

end:
	return ret;
}

static int remove_valid_cmd(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	uint8_t cmd = 0;

	if (spim_device == NULL) {
		shell_error(shell, "Please probe the device first");
		return -ENODEV;
	}

	ret = cmd_parse_helper(shell, &argc, &argv, &cmd);
	if (ret)
		goto end;

	ret = spim_remove_valid_command(spim_device, cmd);
	if (ret)
		goto end;

end:
	return ret;
}

static int lock_valid_cmd(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	uint8_t cmd = 0;

	if (spim_device == NULL) {
		shell_error(shell, "Please probe the device first");
		return -ENODEV;
	}

	if (strncmp(argv[1], "all", 3) == 0) {
		/* lock individual register */
		ret = spim_lock_valid_command_table(spim_device, 0, FLAG_CMD_TABLE_LOCK_ALL);
		goto end;
	}

	ret = cmd_parse_helper(shell, &argc, &argv, &cmd);
	if (ret)
		goto end;

	/* lock individual register */
	ret = spim_lock_valid_command_table(spim_device, cmd, 0);

end:
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_spim_cmds,
	SHELL_CMD_ARG(dump, NULL, "\"dump\"", dump_valid_cmd_table, 1, 0),
	SHELL_CMD_ARG(add, NULL, "<command>", add_valid_cmd, 2, 1),
	SHELL_CMD_ARG(rm, NULL, "<command>", remove_valid_cmd, 2, 0),
	SHELL_CMD_ARG(lock, NULL, "<command>", lock_valid_cmd, 2, 0),

	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(spim_cmds,
	SHELL_CMD_ARG(set_dev, NULL, "<device>", cmd_probe, 2, 0),
	SHELL_CMD(cmd, &sub_spim_cmds, "cmd table related operations", NULL),

	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(spim, &spim_cmds, "SPI monitor shell commands", NULL);

