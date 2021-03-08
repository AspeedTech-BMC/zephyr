/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <shell/shell.h>
#include <string.h>
#include <stdio.h>

static int cmd_mem_wr(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t addr, data;
	char *endptr;

	if (argc > 2) {
		addr = strtol(argv[1], &endptr, 16);
		data = strtol(argv[2], &endptr, 16);
		*(uint32_t *)addr = data;
 	} else {
		return -EINVAL;
	}

	return 0;
}

static int cmd_mem_dump(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t addr, length = 16;
	char *endptr;
	int i, offset = 0;

	if (argc > 1) {
		addr = strtol(argv[1], &endptr, 16);
		if (argc > 2)
			length = strtol(argv[2], &endptr, 16);
	} else {
		return -EINVAL;
	}

	endptr = malloc((length + 2) * 5);
	for (i = 0; i < length; i++) {
		if ((i & 0x3) == 0)
			offset += sprintf(endptr + offset, "\n[%08x] ", addr + (i << 2));
		offset += sprintf(endptr + offset, "%08x ", *(uint32_t *)(addr + (i << 2)));
	}

	shell_print(shell, "%s\n", endptr);
	free(endptr);

	return 0;
}

SHELL_CMD_REGISTER(md, NULL, "Mem Dump command", cmd_mem_dump);
SHELL_CMD_REGISTER(mw, NULL, "Mem Write command", cmd_mem_wr);
