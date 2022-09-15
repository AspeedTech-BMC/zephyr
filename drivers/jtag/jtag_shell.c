/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief JTAG shell commands.
 */

#include <shell/shell.h>
#include <drivers/jtag.h>
#include <stdlib.h>

#define JTAG_DEVICE_PREFIX              "JTAG"

static uint8_t tdi_buffer[512];

static int cmd_ir_scan(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	uint32_t bit_len, byte_len;
	uint32_t value;
	int err, index;

	dev = device_get_binding(argv[-1]);
	if (!dev) {
		shell_error(shell, "JTAG device not found");
		return -EINVAL;
	}

	bit_len = strtoul(argv[1], NULL, 0);
	value = strtoul(argv[2], NULL, 16);

	err = jtag_ir_scan(dev, bit_len, (uint8_t *)&value, tdi_buffer,
			   TAP_IDLE);
	if (err) {
		shell_error(shell, "failed to IR scan (err %d)", err);
		return err;
	}
	byte_len = (bit_len + 7) >> 3;
	for (index = byte_len - 1; index >= 0; index--) {
		shell_print(shell, "%x", tdi_buffer[index]);
	}
	return 0;
}

static int cmd_dr_scan(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	uint32_t bit_len, byte_len;
	uint32_t value;
	int err, index;

	dev = device_get_binding(argv[-1]);
	if (!dev) {
		shell_error(shell, "JTAG device not found");
		return -EINVAL;
	}

	bit_len = strtoul(argv[1], NULL, 0);
	value = strtoul(argv[2], NULL, 16);

	err = jtag_dr_scan(dev, bit_len, (uint8_t *)&value, tdi_buffer,
			   TAP_IDLE);
	if (err) {
		shell_error(shell, "failed to DR scan (err %d)", err);
		return err;
	}
	byte_len = (bit_len + 7) >> 3;
	for (index = byte_len - 1; index >= 0; index--) {
		shell_print(shell, "%x", tdi_buffer[index]);
	}
	return 0;
}

static int cmd_frequency(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	uint32_t freq;
	int err;

	dev = device_get_binding(argv[-1]);
	if (!dev) {
		shell_error(shell, "JTAG device not found");
		return -EINVAL;
	}

	freq = strtoul(argv[1], NULL, 0);

	err = jtag_freq_set(dev, freq);
	if (err) {
		shell_error(shell, "failed to setup JTAG frequency(err %d)",
			    err);
		return err;
	}
	err = jtag_freq_get(dev, &freq);
	if (err) {
		shell_error(shell, "failed to get JTAG frequency (err %d)",
			    err);
		return err;
	}
	shell_print(shell, "%d\n", freq);

	return 0;
}

static int cmd_tap_state(const struct shell *shell, size_t argc, char **argv, void *data)
{
	const struct device *dev;
	enum tap_state state = (enum tap_state) data;
	int err;

	dev = device_get_binding(argv[-2]);
	if (!dev) {
		shell_error(shell, "JTAG device not found");
		return -EINVAL;
	}

	err = jtag_tap_set(dev, state);
	if (err) {
		shell_error(shell, "failed to set JTAG tap_state to %d(err %d)",
			    state, err);
		return err;
	}

	return 0;
}

static int cmd_sw_xfer(const struct shell *shell, size_t argc, char **argv, void *data)
{
	const struct device *dev;
	enum jtag_pin pin = (enum jtag_pin) data;
	uint8_t value;
	int err;

	dev = device_get_binding(argv[-3]);
	if (!dev) {
		shell_error(shell, "JTAG device not found");
		return -EINVAL;
	}
	value = strcmp("high", argv[-1]) ? 0 : 1;

	err = jtag_sw_xfer(dev, pin, value);
	if (err) {
		shell_error(shell, "failed to transfer pin%d = %d(err %d)",
			    pin, value, err);
		return err;
	}

	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_tap_cmds, cmd_tap_state,
	(DREXIT2, TAP_DREXIT2),
	(DREXIT1, TAP_DREXIT1),
	(DRSHIFT, TAP_DRSHIFT),
	(DRPAUSE, TAP_DRPAUSE),
	(IRSELECT, TAP_IRSELECT),
	(DRUPDATE, TAP_DRUPDATE),
	(DRCAPTURE, TAP_DRCAPTURE),
	(DRSELECT, TAP_DRSELECT),
	(IREXIT2, TAP_IREXIT2),
	(IREXIT1, TAP_IREXIT1),
	(IRSHIFT, TAP_IRSHIFT),
	(IRPAUSE, TAP_IRPAUSE),
	(IDLE, TAP_IDLE),
	(IRUPDATE, TAP_IRUPDATE),
	(IRCAPTURE, TAP_IRCAPTURE),
	(RESET, TAP_RESET)
);

SHELL_SUBCMD_DICT_SET_CREATE(sub_pin_cmds, cmd_sw_xfer,
	(TDI, JTAG_TDI),
	(TCK, JTAG_TCK),
	(TMS, JTAG_TMS),
	(TRST, JTAG_TRST)
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_action_cmds,
	SHELL_CMD(high, &sub_pin_cmds, "<pin>", NULL),
	SHELL_CMD(low, &sub_pin_cmds, "<pin>", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_jtag_cmds,
	SHELL_CMD_ARG(frequency, NULL, "<frequency>", cmd_frequency, 2,
		      0),
	SHELL_CMD_ARG(ir_scan, NULL, "<len> <value>", cmd_ir_scan, 3,
		      0),
	SHELL_CMD_ARG(dr_scan, NULL, "<len> <value>", cmd_dr_scan, 3,
		      0),
	SHELL_CMD(tap_set, &sub_tap_cmds, "<tap_state>", NULL),
	SHELL_CMD(sw_xfer, &sub_action_cmds, "<high/low> <pin>", NULL),
	SHELL_SUBCMD_SET_END);

static void cmd_jtag_dev_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, JTAG_DEVICE_PREFIX);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = "Select JTAG device for subcommand.\n";
	entry->subcmd = &sub_jtag_cmds;
}
SHELL_DYNAMIC_CMD_CREATE(sub_jtag_dev, cmd_jtag_dev_get);

SHELL_CMD_REGISTER(jtag, &sub_jtag_dev, "JTAG shell commands", NULL);
