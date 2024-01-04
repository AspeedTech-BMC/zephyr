/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/shell/shell.h>
#include <zephyr/posix/unistd.h>
#include <getopt.h>
#include <ctype.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/target/i3c_target_mqueue.h>

#define I3C_DEVICE_PREFIX		"i3c"
#define I3C_SHELL_MAX_XFER_NUM		2
#define I3C_SHELL_MAX_BUF_SIZE		16

static uint8_t data_buf[I3C_SHELL_MAX_XFER_NUM][I3C_SHELL_MAX_BUF_SIZE];

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, I3C_DEVICE_PREFIX);

	entry->syntax = dev ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

static uint32_t args_to_wdata(char *arg, uint8_t *buf)
{
	char *data_ptrs[I3C_SHELL_MAX_BUF_SIZE];
	char *state;
	int i = 0, len = 0;

	data_ptrs[i] = strtok_r(arg, ",", &state);
	while (data_ptrs[i] && i < I3C_SHELL_MAX_BUF_SIZE - 1) {
		data_ptrs[++i] = strtok_r(NULL, ",", &state);
	}

	for (len = 0; len < i; len++) {
		buf[len] = strtoul(data_ptrs[len], NULL, 0);
	}

	return len;
}

static const char priv_xfer_helper[] = "i3c xfer <dev> -a <addr> -w <wdata> -r <read length>";
static int cmd_priv_xfer(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	struct i3c_driver_data *data;
	struct i3c_device_desc *desc;
	struct getopt_state *state;
	struct i3c_msg xfers[I3C_SHELL_MAX_XFER_NUM];
	int nxfers = 0;
	int addr = -1;
	int c, ret;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I3C: Device %s not found.", argv[1]);
		return -ENODEV;
	}

	while ((c = getopt(argc - 1, &argv[1], "ha:w:r:")) != -1) {
		state = getopt_state_get();
		switch (c) {
		case 'a':
			addr = strtoul(state->optarg, NULL, 0);
			break;
		case 'w':
			xfers[nxfers].flags = 0;
			xfers[nxfers].buf = data_buf[nxfers];
			xfers[nxfers].len = args_to_wdata(state->optarg, data_buf[nxfers]);
			nxfers++;
			break;
		case 'r':
			xfers[nxfers].flags = I3C_MSG_READ;
			xfers[nxfers].buf = data_buf[nxfers];
			xfers[nxfers].len = atoi(state->optarg);
			nxfers++;
			break;
		case 'h':
			shell_help(shell);
			return SHELL_CMD_HELP_PRINTED;
		case '?':
			if ((state->optopt == 'a') || (state->optopt == 'w') ||
			    (state->optopt == 'r')) {
				shell_print(shell, "Option -%c requires an argument.",
					    state->optopt);
			} else if (isprint(state->optopt)) {
				shell_print(shell, "Unknown option `-%c'.", state->optopt);
			} else {
				shell_print(shell, "Unknown option character `\\x%x'.",
					    state->optopt);
			}
			return 1;
		default:
			break;
		}
	}

	if (addr < 0) {
		shell_error(shell, "I3C: address not assigned. Use '-a <target addr>'");
		return -ENODEV;
	}

	data = (struct i3c_driver_data *)dev->data;
	desc = i3c_dev_list_i3c_addr_find(&data->attached_dev, addr);
	if (!desc) {
		shell_error(shell, "I3C: target device with addr 0x%x not found.", addr);
		return -ENODEV;
	}

	shell_print(shell, "Private transfer to address 0x%02x", addr);
	ret = i3c_transfer(desc, xfers, nxfers);
	if (ret) {
		shell_print(shell, "Failed to private transfer: %d", ret);
	}

	for (int i = 0; i < nxfers; i++) {
		if (xfers[i].flags & I3C_MSG_READ) {
			shell_hexdump(shell, xfers[i].buf, xfers[i].len);
		}
	}

	return ret;
}

static const char tmq_xfer_helper[] =
	"i3c tmq <dev> -b 1               | register i3c-target-mqueue driver\n"
	"i3c tmq <dev> -b 0               | unregister i3c-target-mqueue driver\n"
	"i3c tmq <dev> -w <b0, b1,... bn> | write byte stream\n"
	"i3c tmq <dev> -r <length>        | read length of byte from mqueue";

static int cmd_tmq_xfer(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	struct getopt_state *state;
	int c, len, ret;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I3C: Device %s not found.", argv[1]);
		return -ENODEV;
	}

	while ((c = getopt(argc - 1, &argv[1], "w:r:b:h")) != -1) {
		state = getopt_state_get();
		switch (c) {
		case 'w':
			len = args_to_wdata(state->optarg, data_buf[0]);
			i3c_target_mqueue_write(dev, data_buf[0], len);
			return 0;
		case 'r':
			len = strtoul(state->optarg, NULL, 0);
			ret = i3c_target_mqueue_read(dev, data_buf[0], len);
			if (ret) {
				shell_hexdump(shell, data_buf[0], ret);
			}
			return 0;
		case 'b':
			const struct i3c_target_driver_api *api =
				(const struct i3c_target_driver_api *)dev->api;
			if (strtoul(state->optarg, NULL, 0)) {
				api->driver_register(dev);
			} else {
				api->driver_unregister(dev);
			}
			return 0;
		case 'h':
			shell_help(shell);
			return SHELL_CMD_HELP_PRINTED;
		case '?':
			if ((state->optopt == 'r') || (state->optopt == 'w')) {
				shell_print(shell, "Option -%c requires an argument.",
					    state->optopt);
			} else if (isprint(state->optopt)) {
				shell_print(shell, "Unknown option `-%c'.", state->optopt);
			} else {
				shell_print(shell, "Unknown option character `\\x%x'.",
					    state->optopt);
			}
			return 1;
		default:
			break;
		}
	}

	return 0;
}

static const char do_daa_helper[] = "i3c daa <dev>                    | do DAA process";
static int cmd_do_daa(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I3C: Device %s not found.", argv[1]);
		return -ENODEV;
	}

	ret = i3c_do_daa(dev);

	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_i3c_cmds,
			       SHELL_CMD(xfer, &dsub_device_name, priv_xfer_helper, cmd_priv_xfer),
			       SHELL_CMD(tmq, &dsub_device_name, tmq_xfer_helper, cmd_tmq_xfer),
			       SHELL_CMD(daa, &dsub_device_name, do_daa_helper, cmd_do_daa),
			       SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(i3c, &sub_i3c_cmds, "I3C commands", NULL);
