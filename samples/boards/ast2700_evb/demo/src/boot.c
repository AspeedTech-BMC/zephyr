/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <soc.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>

#define LOG_MODULE_NAME boot_demo

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define ASPEED_CPU_CA35_RVBAR0 (0x12C02110 + 0x60000000)
#define ASPEED_CPU_CA35_RVBAR1 (0x12C02114 + 0x60000000)
#define ASPEED_CPU_CA35_RVBAR2 (0x12C02118 + 0x60000000)
#define ASPEED_CPU_CA35_RVBAR3 (0x12C0211c + 0x60000000)

#define ASPEED_CPU_SMP_EP0 (0x12C02780 + 0x60000000)
#define ASPEED_CPU_SMP_EP1 (0x12C02788 + 0x60000000)
#define ASPEED_CPU_SMP_EP2 (0x12C02790 + 0x60000000)
#define ASPEED_CPU_SMP_EP3 (0x12C02798 + 0x60000000)

#define ASPEED_CPU_CA35_REL (0x12C0210C + 0x60000000)

struct image_info {
	char *name;
	uint32_t dst;
	uint32_t src;
	uint32_t len;
};

struct image_info image_tbl[] = {
	{"ATF", 0x04000000, 0x00200000, 0x10000},    /* 64KB */
	{"UBOOT", 0x05880000, 0x00210000, 0x100000}, /* 1MB */
	{"TEE", 0x04080000, 0x00310000, 0x80000},    /* 512KB */
};

void aspeed_prepare_for_boot(void)
{
	/* Given CA35 reset vector */
	sys_write32(0x43000000, ASPEED_CPU_CA35_RVBAR0);
	sys_write32(0x43000000, ASPEED_CPU_CA35_RVBAR1);
	sys_write32(0x43000000, ASPEED_CPU_CA35_RVBAR2);
	sys_write32(0x43000000, ASPEED_CPU_CA35_RVBAR3);

	sys_write32(0x00000000, ASPEED_CPU_SMP_EP0);
	sys_write32(0x00000004, ASPEED_CPU_SMP_EP0 + 4);
	sys_write32(0x00000000, ASPEED_CPU_SMP_EP1);
	sys_write32(0x00000000, ASPEED_CPU_SMP_EP1 + 4);
	sys_write32(0x00000000, ASPEED_CPU_SMP_EP2);
	sys_write32(0x00000000, ASPEED_CPU_SMP_EP2 + 4);
	sys_write32(0x00000000, ASPEED_CPU_SMP_EP3);
	sys_write32(0x00000000, ASPEED_CPU_SMP_EP3 + 4);

	/* Release CA35 */
	sys_write32(1, ASPEED_CPU_CA35_REL);
}

int aspeed_load_image(void)
{
	const struct device *flash_dev;
	uint32_t dst;
	uint32_t src;
	uint32_t len;
	uint32_t ret;
	int i;

	flash_dev = device_get_binding("fmc@0");
	if (!flash_dev) {
		LOG_ERR("No device named fmc@0.");
		return -ENXIO;
	}

	LOG_INF("Loading images...");

	for (i = 0; i < ARRAY_SIZE(image_tbl); i++) {
		LOG_INF("Image[%d]: %s", i, image_tbl[i].name);

		dst = image_tbl[i].dst;
		src = image_tbl[i].src;
		len = image_tbl[i].len;

		ret = flash_read(flash_dev, src, (void *)dst, len);
		if (ret) {
			LOG_ERR("fail to read %s from flash fmc@0", image_tbl[i].name);
			return -1;
		}
	}

	aspeed_prepare_for_boot();

	return 0;
}

