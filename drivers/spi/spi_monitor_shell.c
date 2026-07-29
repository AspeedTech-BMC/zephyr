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
#if defined(CONFIG_SOC_AST1060)
#include <zephyr/drivers/misc/aspeed/pfr_aspeed.h>
#elif defined(CONFIG_SOC_AST1040_CM4) || defined(CONFIG_SOC_AST1080_CM4)
/*
 * AST1080's addr-priv table follows the AST2700 layout (see
 * ast1080_spim_ops in spi_monitor_aspeed.c), but it still has the
 * ext-mux control that AST2700 lacks, so pull in both headers.
 */
#include <zephyr/drivers/misc/aspeed/pfr_aspeed.h>
#include <zephyr/drivers/misc/aspeed/ast2700_spim.h>
#elif defined(CONFIG_SOC_AST2700) || defined(CONFIG_SOC_AST2705)
#include <zephyr/drivers/misc/aspeed/ast2700_spim.h>
#endif
#include <soc.h>
#include <zephyr/kernel.h>

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
	unsigned long val;

	if (*argc < 2) {
		shell_error(shell, "Missing command.");
		return -EINVAL;
	}

	val = strtoul((*argv)[1], &endptr, 16);
	if (endptr == (*argv)[1] || *endptr != '\0' || val > 0xff) {
		shell_error(shell, "Invalid command \"%s\", expected a 2-digit hex byte.",
			(*argv)[1]);
		return -EINVAL;
	}

	*cmd = val;

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

static int dump_allow_cmd_table(const struct shell *shell, size_t argc, char *argv[])
{
	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	spim_dump_allow_command_table(spim_device);

	return 0;
}

static int add_allow_cmd(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	uint8_t cmd = 0;

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	ret = cmd_parse_helper(shell, &argc, &argv, &cmd);
	if (ret)
		goto end;

	if (argc == 3 && strncmp(argv[2], "once", 4) == 0)
		ret = spim_add_allow_command(spim_device, cmd, FLAG_CMD_TABLE_VALID_ONCE);
	else
		ret = spim_add_allow_command(spim_device, cmd, FLAG_CMD_TABLE_VALID);

end:
	return ret;
}

static int remove_allow_cmd(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	uint8_t cmd = 0;

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	ret = cmd_parse_helper(shell, &argc, &argv, &cmd);
	if (ret)
		goto end;

	ret = spim_remove_allow_command(spim_device, cmd);
	if (ret)
		goto end;

end:
	return ret;
}

static int lock_allow_cmd(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	uint8_t cmd = 0;

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	if (strncmp(argv[1], "all", 3) == 0) {
		/* lock individual register */
		ret = spim_lock_allow_command_table(spim_device, 0, FLAG_CMD_TABLE_LOCK_ALL);
		goto end;
	}

	ret = cmd_parse_helper(shell, &argc, &argv, &cmd);
	if (ret)
		goto end;

	/* lock individual register */
	ret = spim_lock_allow_command_table(spim_device, cmd, 0);

end:
	return ret;
}

static int dump_addr_priv_table(const struct shell *shell, size_t argc, char *argv[])
{
	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	spim_dump_addr_priv_table(spim_device);

	return 0;
}

#if defined(CONFIG_SOC_AST1060)
static int addr_parse_helper(const struct shell *shell, size_t *argc,
		char **argv[], bool *enable, mm_reg_t *addr, uint32_t *len)
{
	char *endptr;

	if (*argc < 4) {
		shell_error(shell, "Missing address or length parameter.");
		return -EINVAL;
	}

	*enable = false;
	if (strncmp((*argv)[1], "enable", 6) == 0)
		*enable = true;

	*addr = strtoul((*argv)[2], &endptr, 16);
	*len = strtoul((*argv)[3], &endptr, 16);

	return 0;
}

static int read_addr_priv_table_config(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	mm_reg_t addr = 0;
	uint32_t len = 0;
	bool enable = false;
	enum addr_priv_op op;

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	ret = addr_parse_helper(shell, &argc, &argv, &enable, &addr, &len);
	if (ret)
		goto end;

	printk("read: %s, addr: 0x%08lx, len: 0x%08x\n",
		enable ? "enable" : "disable", addr, len);

	if (enable)
		op = FLAG_ADDR_PRIV_ENABLE;
	else
		op = FLAG_ADDR_PRIV_DISABLE;

	ret = spim_address_privilege_config(spim_device,
					    FLAG_ADDR_PRIV_READ_SELECT,
					    op, addr, len);

end:
	return ret;
}

static int write_addr_priv_table_config(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	mm_reg_t addr = 0;
	uint32_t len = 0;
	bool enable = false;
	enum addr_priv_op op;

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	ret = addr_parse_helper(shell, &argc, &argv, &enable, &addr, &len);
	if (ret)
		goto end;

	printk("write: %s, addr: 0x%08lx, len: 0x%08x\n",
		enable ? "enable" : "disable", addr, len);

	if (enable)
		op = FLAG_ADDR_PRIV_ENABLE;
	else
		op = FLAG_ADDR_PRIV_DISABLE;

	ret = spim_address_privilege_config(spim_device,
					    FLAG_ADDR_PRIV_WRITE_SELECT,
					    op, addr, len);

end:
	return ret;
}
#endif

#if defined(CONFIG_SOC_AST2700) || defined(CONFIG_SOC_AST2705) || \
	defined(CONFIG_SOC_AST1040_CM4) || defined(CONFIG_SOC_AST1080_CM4)
static int ast2700_addr_priv_config(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;
	mm_reg_t addr = 0;
	uint32_t len = 0;
	uint32_t attr = 0;
	char *endptr;

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	if (argc < 4) {
		shell_error(shell, "Missing addr, len or attr parameter.");
		return -EINVAL;
	}

	addr = strtoul(argv[1], &endptr, 16);
	if (endptr == argv[1] || *endptr != '\0') {
		shell_error(shell, "Invalid addr \"%s\".", argv[1]);
		return -EINVAL;
	}

	len = strtoul(argv[2], &endptr, 16);
	if (endptr == argv[2] || *endptr != '\0') {
		shell_error(shell, "Invalid len \"%s\".", argv[2]);
		return -EINVAL;
	}

	if (strncmp(argv[3], "remove", 6) == 0) {
		ret = ast2700_address_privilege_remove(spim_device, addr, len);
		goto end;
	} else if (strncmp(argv[3], "r_dis", 5) == 0) {
		attr = FLAG_ADDR_PRIV_READ_DIS;
	} else if (strncmp(argv[3], "w_dis", 5) == 0) {
		attr = FLAG_ADDR_PRIV_WRITE_DIS;
	} else if (strncmp(argv[3], "rw_dis", 6) == 0) {
		attr = FLAG_ADDR_PRIV_READ_DIS | FLAG_ADDR_PRIV_WRITE_DIS;
	} else {
		printk("invalid attribute\n");
		return -1;
	}

	printk("addr: 0x%08lx, len: 0x%08x, attr: 0x%08x\n",
		addr, len, attr);

	ret = ast2700_address_privilege_config(spim_device, addr, len, attr);
end:
	return ret;
}
#endif

static int cmd_lock(const struct shell *shell, size_t argc, char *argv[])
{

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	spim_lock_common(spim_device);

	return 0;
}

static int spi_monitor_enabled(const struct shell *shell, size_t argc, char *argv[])
{

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	spim_monitor_enable(spim_device, true);

	return 0;
}

static int spi_monitor_disabled(const struct shell *shell, size_t argc, char *argv[])
{

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	spim_monitor_enable(spim_device, false);

	return 0;
}

#if defined(CONFIG_SOC_AST1060) || defined(CONFIG_SOC_AST1040_CM4) || \
	defined(CONFIG_SOC_AST1080_CM4)
static int ext_mux_config(const struct shell *shell, size_t argc, char *argv[])
{
	uint32_t flag;
	char *endptr;

	if (!spim_device) {
		shell_error(shell, "Please set the device first.");
		return -ENODEV;
	}

	flag = strtoul(argv[1], &endptr, 16);

	if (flag == 0)
		spim_ext_mux_config(spim_device, 0);
	else
		spim_ext_mux_config(spim_device, 1);

	return 0;
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(sub_spim_cmds,
	SHELL_CMD_ARG(dump, NULL, "\"dump\"", dump_allow_cmd_table, 1, 0),
	SHELL_CMD_ARG(add, NULL, "<command>", add_allow_cmd, 2, 1),
	SHELL_CMD_ARG(rm, NULL, "<command>", remove_allow_cmd, 2, 0),
	SHELL_CMD_ARG(lock, NULL, "<command>", lock_allow_cmd, 2, 0),

	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_spim_addr,
	SHELL_CMD_ARG(dump, NULL, "\"dump\"", dump_addr_priv_table, 1, 0),
#if defined(CONFIG_SOC_AST1060)
	SHELL_CMD_ARG(read, NULL, "<enable/disable> <addr> <len>",
		read_addr_priv_table_config, 4, 0),
	SHELL_CMD_ARG(write, NULL, "<enable/disable> <addr> <len>",
		write_addr_priv_table_config, 4, 0),
#elif defined(CONFIG_SOC_AST2700) || defined(CONFIG_SOC_AST2705) || \
	defined(CONFIG_SOC_AST1040_CM4) || defined(CONFIG_SOC_AST1080_CM4)
	SHELL_CMD_ARG(config, NULL, "<addr> <len> <attr>",
		ast2700_addr_priv_config, 4, 0),
#endif

	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_spim_config,
	SHELL_CMD_ARG(enable, NULL, "\"enable\"", spi_monitor_enabled, 1, 0),
	SHELL_CMD_ARG(disable, NULL, "\"disable\"", spi_monitor_disabled, 1, 0),
#if defined(CONFIG_SOC_AST1060) || defined(CONFIG_SOC_AST1040_CM4) || \
	defined(CONFIG_SOC_AST1080_CM4)
	SHELL_CMD_ARG(extmux, NULL, "<0/1> for clear/set", ext_mux_config, 2, 0),
#endif
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(spim_cmds,
	SHELL_CMD_ARG(set_dev, NULL, "<device>", cmd_probe, 2, 0),
	SHELL_CMD(cmd, &sub_spim_cmds, "cmd table related operations", NULL),
	SHELL_CMD(addr, &sub_spim_addr, "address privilege table related operations", NULL),
	SHELL_CMD_ARG(lock, NULL, "lock", cmd_lock, 1, 0),
	SHELL_CMD(config, &sub_spim_config, "SPI monitor configuration", NULL),

	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(spim, &spim_cmds, "SPI monitor shell commands", NULL);
