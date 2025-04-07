/*
 * Copyright 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/misc/aspeed/cptra_ipc.h>
#include <string.h>
#if defined(CONFIG_IPC_SAMPLE)
#include "ipc_sample.h"
#endif
#if defined(CONFIG_CPTRA_SAMPLE)
#include "cptra_sample.h"
#endif

int aspeed_load_image(void);

int main(void)
{
	int rc = 0;

	printk("%s demo\n", CONFIG_BOARD);

#if defined(CONFIG_LOAD_FIT_ENABLED)
	void *func = (void *)CONFIG_AST_EXT_LOADER_ADDR;
	((void (*)(uint32_t))func)((uint32_t)CONFIG_LOAD_FIT_ADDR);
#endif

#if defined(CONFIG_CPTRA_IPC) || defined(CONFIG_CPTRA_IPC_SSP)
	cptra_ipc_enable();
#endif
#if defined(CONFIG_IPC_SAMPLE)
	ipc_test();
#endif

#if defined(CONFIG_CPTRA_SAMPLE)
	cptra_test();
#endif

#if defined(CONFIG_AST2700_IROT_LOAD_IMAGE)
	aspeed_load_image();
#endif

	return rc;
}
