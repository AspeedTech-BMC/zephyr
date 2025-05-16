/*
 * Copyright (c) 2025 ASPEED Technology Inc.
 *
 * Implement common and specified ipm usage for aspeed chips.
 *
 * ipm : ast2700
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/ipm.h>
LOG_MODULE_REGISTER(ipm_shell, CONFIG_LOG_DEFAULT_LEVEL);

#define IPC_NUM_OF_ID	4
#define IPC_MAX_MSG_SIZE	0x20
#define DEFAULT_LINE_LENGTH_BYTES	(16)

/* sample irq call back*/
static void ch_ipm_cb(const struct device *ipmdev, void *user_data,
		       uint32_t id, volatile void *msg_data)
{
	int i;
	int width = 4;
	int linelen = DEFAULT_LINE_LENGTH_BYTES / width;
	int max_msg_data_size = ipm_max_data_size_get(ipmdev);
	uint32_t *buf = (uint32_t *)msg_data;

	LOG_INF("%s:msg id %x, msg data at %p, msg size 0x%x\n",
		__func__, id, (uint32_t *)msg_data, max_msg_data_size);
	while (max_msg_data_size) {
		LOG_INF("%p:", buf);

		for (i = 0; i < linelen; i++)
			LOG_INF(" %08x ", buf[i]);
		LOG_INF("\n");
		buf += linelen;
		max_msg_data_size -= linelen * width;
	}

	max_msg_data_size = ipm_max_data_size_get(ipmdev);
	ipm_send(ipmdev, 0, id, (void *)msg_data, max_msg_data_size);
}

/*
 * IPM enable / disable function
 */
static int cmd_ipm_enable(const struct shell *shell,
			size_t argc, char **argv)
{
	const struct device *ipmdev = NULL;
	int channel = 0, enable = 0;

	ipmdev = device_get_binding(argv[1]);
	if (!ipmdev) {
		shell_error(shell, "IPM: Device %s not found.",
				argv[1]);
		return -ENODEV;
	}

	channel = strtol(argv[2], NULL, 16);
	if (channel > 4) {
		shell_error(shell, "IPM: Channel %d not found.",
				channel);
		return -EINVAL;
	}

	enable = strtol(argv[3], NULL, 16);
	ipm_set_id_enabled(ipmdev, channel, enable);

	return 0;
}

/*
 * IPM attach callback function
 */
static int cmd_ipm_attach(const struct shell *shell,
			size_t argc, char **argv)
{
	const struct device *ipmdev = NULL;
	int channel = 0, value = 0;
	void *user_data = NULL;
	int callback = 0;

	ipmdev = device_get_binding(argv[1]);

	if (!ipmdev) {
		shell_error(shell, "IPM: Device %s not found.",
				argv[1]);
		return -ENODEV;
	}

	channel = strtol(argv[2], NULL, 16);
	if (channel > 4) {
		shell_error(shell, "IPM: Channel %d not found.",
				channel);
		return -EINVAL;
	}

	value = strtol(argv[3], NULL, 16);
	if (argv[3] != 0) {
		callback = (int)(value);
	} else {
		/* assign default callback */
		callback = (int)ch_ipm_cb;
	}

	value = strtol(argv[4], NULL, 16);
	if (argv[4] != 0) {
		user_data = (void *)(value);
	}

	/* attach ipm call back*/
	ipm_register_id_callback(ipmdev, channel, (void *)callback, user_data);

	return 0;
}

 /*
  * IPM detach callback function
  */
static int cmd_ipm_detach(const struct shell *shell,
			 size_t argc, char **argv)
{
	const struct device *ipmdev = NULL;
	int channel = 0;

	ipmdev = device_get_binding(argv[1]);

	if (!ipmdev) {
		shell_error(shell, "IPM: Device %s not found.",
				argv[1]);
		return -ENODEV;
	}

	channel = strtol(argv[2], NULL, 16);
	if (channel > 4) {
		shell_error(shell, "IPM: Channel %d not found.",
				 channel);
		return -EINVAL;
	}

	/* detach ipm call back*/
	ipm_register_id_callback(ipmdev, channel, NULL, NULL);
	return 0;
}

/*
 * IPM send
 */
static int cmd_ipm_send(const struct shell *shell,
			size_t argc, char **argv)
{
	const struct device *ipmdev = NULL;
	int channel = 0, num_data = 0, max_data = 0;
	uint32_t buf[IPC_MAX_MSG_SIZE];

	ipmdev = device_get_binding(argv[1]);
	if (!ipmdev) {
		shell_error(shell, "IPM: Device %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	max_data = ipm_max_data_size_get(ipmdev);

	channel = strtol(argv[2], NULL, 16);
	if (channel > 4) {
		shell_error(shell, "IPM: Channel %d not found.",
				channel);
		return -EINVAL;
	}

	num_data = argc - 3;
	if (num_data > max_data) {
		num_data  = max_data;
	}

	for (int i = 0; i < num_data; i++) {
		buf[i] = (uint32_t)strtol(argv[3 + i], NULL, 16);
	}

	ipm_send(ipmdev, 0, channel, (void *)buf, num_data);
	return 0;
}

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ipm_cmds,
			       SHELL_CMD(enable, &dsub_device_name,
					 "Enable / Disable IPM function", cmd_ipm_enable),
			       SHELL_CMD(attach, &dsub_device_name,
					 "Attach IPM callback function", cmd_ipm_attach),
			       SHELL_CMD(detach, &dsub_device_name,
					 "Detach IPM callback function", cmd_ipm_detach),
			       SHELL_CMD(send, &dsub_device_name,
					 "Send IPM command", cmd_ipm_send),
			       SHELL_SUBCMD_SET_END     /* Array terminated. */
			       );

SHELL_CMD_REGISTER(ipm, &sub_ipm_cmds, "ASPEED IPM commands", NULL);
