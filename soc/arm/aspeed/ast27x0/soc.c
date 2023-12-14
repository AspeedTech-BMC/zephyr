/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <soc.h>

extern char __bss_nc_start__[];
extern char __bss_nc_end__[];
#ifdef CONFIG_XIP
extern char _flash_used[];
#else
extern char __data_region_end[];
#endif

/* secure boot header : provide image size to bootROM for SPI boot */
struct sb_header {
	uint32_t key_location;
	uint32_t enc_img_addr;
	uint32_t img_size;
	uint32_t sign_location;
	uint32_t header_rev[2];
	uint32_t patch_location;
	uint32_t checksum;
};

struct sb_header sbh __attribute((used, section(".sboot"))) = {
#ifdef CONFIG_XIP
	.img_size = (uint32_t)&_flash_used,
#else
	.img_size = (uint32_t)&__data_region_end,
#endif
};

void z_arm_platform_init(void)
{
	/* clear non-cached .bss */
	if (CONFIG_SRAM_NC_SIZE > 0) {
		(void)memset(__bss_nc_start__, 0, __bss_nc_end__ - __bss_nc_start__);
	}

	sys_cache_instr_enable();
	sys_cache_data_enable();
}
