/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Shell commands demonstrating how an application is expected to use the
 * psp_reset_notify driver API:
 *
 *   psp_reset register   - register a callback (here: just logs)
 *   psp_reset enable     - arm the interrupt
 *   psp_reset disable    - mask the interrupt
 *
 * Typical flow: "psp_reset register" once, then "psp_reset enable" to start
 * waiting. Every time the interrupt fires it is auto-masked and the callback
 * runs (printing a log line here); type "psp_reset enable" again once the
 * PSP reset condition has been handled/released to re-arm for the next one.
 */

#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/psp_reset_notify.h>

LOG_MODULE_REGISTER(psp_reset_notify_shell, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const psp_reset_dev =
	DEVICE_DT_GET(DT_NODELABEL(psp_reset_notify));

/* The callback runs in ISR context; keep it to just a log line. */
static void psp_reset_shell_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	LOG_INF("PSP reset interrupt received");
}

static int cmd_psp_reset_register(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(psp_reset_dev)) {
		shell_error(sh, "psp_reset_notify device not ready");
		return -ENODEV;
	}

	psp_reset_notify_set_callback(psp_reset_dev, psp_reset_shell_cb, NULL);
	shell_print(sh, "callback registered, will log on every PSP reset interrupt");

	return 0;
}

static int cmd_psp_reset_enable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(psp_reset_dev)) {
		shell_error(sh, "psp_reset_notify device not ready");
		return -ENODEV;
	}

	psp_reset_notify_irq_enable(psp_reset_dev);
	shell_print(sh, "armed, waiting for the next PSP reset interrupt");

	return 0;
}

static int cmd_psp_reset_disable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(psp_reset_dev)) {
		shell_error(sh, "psp_reset_notify device not ready");
		return -ENODEV;
	}

	psp_reset_notify_irq_disable(psp_reset_dev);
	shell_print(sh, "masked");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_psp_reset_cmds,
			       SHELL_CMD(register, NULL,
					 "register a log-only callback",
					 cmd_psp_reset_register),
			       SHELL_CMD(enable, NULL,
					 "arm the PSP reset interrupt (call again to re-arm after each firing)",
					 cmd_psp_reset_enable),
			       SHELL_CMD(disable, NULL,
					 "mask the PSP reset interrupt",
					 cmd_psp_reset_disable),
			       SHELL_SUBCMD_SET_END
			       );

SHELL_CMD_REGISTER(psp_reset, &sub_psp_reset_cmds,
		   "PSP reset notification driver test commands", NULL);
