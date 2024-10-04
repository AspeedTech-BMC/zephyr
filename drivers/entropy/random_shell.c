/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#include <zephyr/shell/shell.h>
#include <zephyr/random/rand32.h>
#include <stdlib.h>

#define SHELL_HELP_GET_RANDOM		"Get random number"
#define BUFFER_LENGTH			16

static void random_get(int len)
{
	uint8_t buffer[BUFFER_LENGTH] = {0};
	int count;

	while (len) {
		count = MIN(len, BUFFER_LENGTH);
		sys_rand_get(buffer, count);
		len -= count;

		for (int i = 0; i < count; i++) {
			printk(" 0x%02x", buffer[i]);
		}

		printk("\n");
	}
}

static void cmd_random_get(const struct shell *shell, size_t argc, char **argv)
{
	int count = 0;

	if (argc == 2)
		count = strtol(argv[1], NULL, 10);

	if (count == 0)
		printk(" 0x%x\n", sys_rand32_get());
	else
		random_get(count);
}

SHELL_STATIC_SUBCMD_SET_CREATE(random_cmds,
	SHELL_CMD(get, NULL, SHELL_HELP_GET_RANDOM, cmd_random_get),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(random, &random_cmds, "Random shell commands", NULL);
