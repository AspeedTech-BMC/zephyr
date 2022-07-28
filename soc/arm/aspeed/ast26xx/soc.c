/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2021 ASPEED Technology Inc.
 */

#include <init.h>
#include <kernel.h>
#include <stdint.h>
#include <string.h>
#include <linker/linker-defs.h>
#include <cache.h>
#include <drivers/hwinfo.h>
#include <soc.h>

extern char __bss_nc_start__[];
extern char __bss_nc_end__[];

/*WDT0 registers*/
#define WDT0_BASE 0x7e785000

#define WDT_SOFTWARE_RESET_MASK_REG 0x28

#define WDT_SOFTWARE_RESET_REG 0x24
#define WDT_TRIGGER_KEY 0xAEEDF123

/* SCU registers */
#define HW_STRAP_SET            0x500
#define HW_STRAP_CLR            0x504
#define CORTEX_A7_RESET         BIT(0)

/* secure boot header : provide image size to bootROM for SPI boot */
struct sb_header {
	uint32_t key_location;
	uint32_t enc_img_addr;
	uint32_t img_size;
	uint32_t sign_location;
	uint32_t header_rev[2];
	uint32_t patch_location;        /* address of the rom patch */
	uint32_t checksum;
};

struct sb_header sbh __attribute((used, section(".sboot"))) = {
	.img_size = (uint32_t)&_image_rom_end,
};

void z_platform_init(void)
{
	cache_instr_enable();

	if (CONFIG_SRAM_NC_SIZE > 0) {
		(void)memset(__bss_nc_start__, 0, __bss_nc_end__ - __bss_nc_start__);
	}
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
