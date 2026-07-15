/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/misc/aspeed/uart_routing_aspeed.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

static const struct device *uart_routing_dev;

static int cmd_uart_routing_get(const struct shell *shell, size_t argc, char **argv)
{
	const char *target;
	int ret;

	if (!device_is_ready(uart_routing_dev)) {
		shell_error(shell, "uart_routing device not ready");
		return -ENODEV;
	}

	ret = uart_routing_aspeed_get(uart_routing_dev, argv[1], &target);
	if (ret) {
		shell_error(shell, "failed to get \"%s\" (%d)", argv[1], ret);
		return ret;
	}

	/* argv[1] is fed by target's output, e.g. "io0 <- uart1" means
	 * uart1's output is wired into io0's input.
	 */
	shell_print(shell, "%s <- %s", argv[1], target);

	return 0;
}

static int cmd_uart_routing_set(const struct shell *shell, size_t argc, char **argv)
{
	int ret;

	if (!device_is_ready(uart_routing_dev)) {
		shell_error(shell, "uart_routing device not ready");
		return -ENODEV;
	}

	ret = uart_routing_aspeed_set(uart_routing_dev, argv[1], argv[2]);
	if (ret) {
		shell_error(shell, "failed to route \"%s\" <- \"%s\" (%d)", argv[1], argv[2], ret);
		return ret;
	}

	/* argv[2]'s output is now wired into argv[1]'s input. */
	shell_print(shell, "%s <- %s", argv[1], argv[2]);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(uart_routing_cmds,
	SHELL_CMD_ARG(get, NULL, "uart_routing get <ch/io>", cmd_uart_routing_get, 2, 0),
	SHELL_CMD_ARG(set, NULL,
		"uart_routing set <ch/io> <val>: feed <val>'s output into <ch/io>'s input",
		cmd_uart_routing_set, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(uart_routing, &uart_routing_cmds, "ASPEED UART routing shell commands", NULL);

void main(void)
{
	uart_routing_dev = DEVICE_DT_GET(DT_NODELABEL(uart_routing));
	if (!device_is_ready(uart_routing_dev)) {
		printk("uart_routing device not ready\n");
		return;
	}

	printk("uart_routing ready, try \"uart_routing get uart0\"\n");
}
