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
#if defined(CONFIG_CPTRA_SAMPLE)
#include "cptra_sample.h"
#endif

#define DEFAULT_LINE_LENGTH_BYTES (16)
static void test_ipm_cb(const struct device *ipmdev, void *user_data,
			uint32_t id, volatile void *msg_data)
{
	int i;
	int check = 0;
	int width = 4;
	int linelen = DEFAULT_LINE_LENGTH_BYTES / width;
	int max_msg_data_size = ipm_max_data_size_get(ipmdev);
	uint32_t *buf = (uint32_t *)msg_data;

	printk("%s:msg id %x, msg data at %p\n", __func__, id, msg_data);

	for (i = 0; i < max_msg_data_size / sizeof(buf) ; i++) {
		/* FAIL when buf is not golden value */
		if (buf[i] != 0x1688a8a8)
			check = 1;
	}

	/* Check golden fail to print the msg data */
	if (check)
		while (max_msg_data_size) {
			printk("%p:", buf);

			for (i = 0; i < linelen; i++)
				printk(" %08x ", buf[i]);
			printk("\n");
			buf += linelen;
			max_msg_data_size -= linelen * width;
		}
	else
		printk("Check msg data: pass.\n");
}

int main(void)
{
	int rc = 0;

	printk("%s demo\n", CONFIG_BOARD);

#if defined(CONFIG_LOAD_FIT_ENABLED)
	void *func = (void *)CONFIG_AST_EXT_LOADER_ADDR;
	((void (*)(uint32_t))func)((uint32_t)CONFIG_LOAD_FIT_ADDR);
#endif

	cptra_ipc_enable();

#if defined(CONFIG_IPC_SAMPLE)
	const struct device *ipmdev;
	char ipc_name[32];
	int device_id, enable;

	strcpy(ipc_name, "ipc1@400");
	ipmdev = device_get_binding(&ipc_name[0]);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = 1;
		goto fail;
	}

	device_id = 0;
	enable = 1;
	ipm_register_id_callback(ipmdev, device_id, test_ipm_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, enable);
	if (rc) {
		printk("%s: cannot ipm_set_enabled\n", ipc_name);
		goto fail;
	}
#endif

#if defined(CONFIG_CPTRA_SAMPLE)
	cptra_test();
#endif

fail:
	return rc;
}
