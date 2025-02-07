/*
 * Copyright 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	printk("%s demo\n", CONFIG_BOARD);

#if defined(CONFIG_LOAD_FIT_ENABLED)
	void *func = (void *)CONFIG_AST_EXT_LOADER_ADDR;
	((void (*)(uint32_t))func)((uint32_t)CONFIG_LOAD_FIT_ADDR);
#endif

	return 0;
}
