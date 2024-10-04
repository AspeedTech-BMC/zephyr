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
#include <zephyr/drivers/hwinfo.h>
#include <soc.h>

extern char __RAM_NC_start[];
extern char __RAM_NC_end[];
#ifdef CONFIG_XIP
extern char _flash_used[];
#else
extern char __data_region_end[];
#endif

/*WDT0 registers*/
#define WDT0_BASE			0x7e785000
#define WDT_SOFTWARE_RESET_REG		0x24
#define   WDT_TRIGGER_KEY		0xaeedf123
#define WDT_SOFTWARE_RESET_MASK_REG	0x28

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
	cache_instr_enable();

	/* clear non-cached .bss */
	(void)memset(__RAM_NC_start, 0, __RAM_NC_end - __RAM_NC_start);
}

void sys_arch_reboot(int type)
{
	sys_write32(0x3fffff1, WDT0_BASE + WDT_SOFTWARE_RESET_MASK_REG);
	sys_write32(WDT_TRIGGER_KEY, WDT0_BASE + WDT_SOFTWARE_RESET_REG);
	ARG_UNUSED(type);
}

#define SOC_ID(str, rev) { .name = str, .rev_id = rev, }

struct soc_id {
	const char *name;
	uint64_t rev_id;
};

static struct soc_id soc_map_table[] = {
	SOC_ID("AST2600-A0", ASPEED_SOC_ID_AST2600A0),
	SOC_ID("AST2600-A1", ASPEED_SOC_ID_AST2600A1),
	SOC_ID("AST2620-A1", ASPEED_SOC_ID_AST2620A1),
	SOC_ID("AST2600-A2", ASPEED_SOC_ID_AST2600A2),
	SOC_ID("AST2620-A2", ASPEED_SOC_ID_AST2620A2),
	SOC_ID("AST2600-A3", ASPEED_SOC_ID_AST2600A3),
	SOC_ID("AST2620-A3", ASPEED_SOC_ID_AST2620A3),
	SOC_ID("Unknown",    0x0000000000000000),
};

void aspeed_soc_show_chip_id(void)
{
	uint64_t rev_id;
	size_t len;
	int i;

	len = hwinfo_get_device_id((uint8_t *)&rev_id, sizeof(rev_id));
	if (len < 0) {
		return;
	}

	for (i = 0; i < ARRAY_SIZE(soc_map_table); i++) {
		if (rev_id == soc_map_table[i].rev_id) {
			break;
		}
	}

	printk("SOC: %s\n", soc_map_table[i].name);
}
