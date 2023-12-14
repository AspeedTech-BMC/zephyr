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
		addr = strtoul(argv[1], &endptr, 16);
		data = strtoul(argv[2], &endptr, 16);
		sys_write32(data, addr);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int cmd_mem_md(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t addr, length = 16;
	char *endptr;
	int i;

	if (argc < 2)
		return -EINVAL;

	addr = strtoul(argv[1], &endptr, 16);
	if (argc > 2)
		length = strtoul(argv[2], &endptr, 16);

	for (i = 0; i < length; i++) {
		if ((i & 0x3) == 0x0) {
			shell_fprintf(shell, SHELL_NORMAL, "[%08x] ", addr + (i << 2));
		}
		shell_fprintf(shell, SHELL_NORMAL, "%08x ", sys_read32(addr + (i << 2)));
		if ((i & 0x3) == 0x3 || i == length - 1) {
			shell_fprintf(shell, SHELL_NORMAL, "\n");
		}
	}

	return 0;
}

SHELL_CMD_REGISTER(md, NULL, "Mem Display command", cmd_mem_md);
SHELL_CMD_REGISTER(mw, NULL, "Mem Write command", cmd_mem_wr);
