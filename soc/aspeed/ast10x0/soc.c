/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2021 ASPEED Technology Inc.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <soc.h>
#include <zephyr/drivers/hwinfo.h>

extern char __RAM_NC_start[];
extern char __RAM_NC_end[];
#ifdef CONFIG_XIP
extern char _flash_used[];
#else
extern char __data_region_end[];
#endif

/* SCU registers */
#define JTAG_PINMUX_REG		0x41c

/* GPIO_I_L*/
#define GPIO_I_L_DAT_VAL_REG   0x7e780070
#define GPIO_I_L_DIR_REG       0x7e780074
#define GPIO_I_L_DAT_READ_REG  0x7e7800c8

/* ASPEED System reset contrl/status register */
#define SYS_WDT4_SW_RESET	BIT(31)
#define SYS_WDT4_ARM_RESET	BIT(30)
#define SYS_WDT4_FULL_RESET	BIT(29)
#define SYS_WDT4_SOC_RESET	BIT(28)
#define SYS_WDT3_SW_RESET	BIT(27)
#define SYS_WDT3_ARM_RESET	BIT(26)
#define SYS_WDT3_FULL_RESET	BIT(25)
#define SYS_WDT3_SOC_RESET	BIT(24)
#define SYS_WDT2_SW_RESET	BIT(23)
#define SYS_WDT2_ARM_RESET	BIT(22)
#define SYS_WDT2_FULL_RESET	BIT(21)
#define SYS_WDT2_SOC_RESET	BIT(20)
#define SYS_WDT1_SW_RESET	BIT(19)
#define SYS_WDT1_ARM_RESET	BIT(18)
#define SYS_WDT1_FULL_RESET	BIT(17)
#define SYS_WDT1_SOC_RESET	BIT(16)

#define SYS_FLASH_ABR_RESET	BIT(2)
#define SYS_EXT_RESET		BIT(1)
#define SYS_PWR_RESET_FLAG	BIT(0)

#define SYS_RESET_LOG_CLEAR(_bitmask, _reg)                                                        \
	COND_CODE_1(CONFIG_HWINFO_ASPEED, (sys_write32(_bitmask, _reg);), ());

#define BIT_WDT_SOC(x)		SYS_WDT##x##_SOC_RESET
#define BIT_WDT_FULL(x)		SYS_WDT##x##_FULL_RESET
#define BIT_WDT_ARM(x)		SYS_WDT##x##_ARM_RESET
#define BIT_WDT_SW(x)		SYS_WDT##x##_SW_RESET

#define HANDLE_WDTx_RESET(x, event_log, event_log_reg)                                             \
	if ((event_log) & (BIT_WDT_SOC(x) | BIT_WDT_FULL(x) | BIT_WDT_ARM(x) | BIT_WDT_SW(x))) {   \
		printk("RST: WDT%d ", x);                                                          \
		if ((event_log) & BIT_WDT_SOC(x)) {                                                \
			printk("SOC ");                                                            \
			COND_CODE_1(CONFIG_HWINFO_ASPEED,                                          \
				    (sys_write32(BIT_WDT_SOC(x), event_log_reg);), ());            \
		}                                                                                  \
		if ((event_log) & BIT_WDT_FULL(x)) {                                               \
			printk("FULL ");                                                           \
			COND_CODE_1(CONFIG_HWINFO_ASPEED,                                          \
				    (sys_write32(BIT_WDT_FULL(x), event_log_reg);), ());           \
		}                                                                                  \
		if ((event_log) & BIT_WDT_ARM(x)) {                                                \
			printk("ARM ");                                                            \
			COND_CODE_1(CONFIG_HWINFO_ASPEED,                                          \
				    (sys_write32(BIT_WDT_ARM(x), event_log_reg);), ());            \
		}                                                                                  \
		if ((event_log) & BIT_WDT_SW(x)) {                                                 \
			printk("SW ");                                                             \
			COND_CODE_1(CONFIG_HWINFO_ASPEED,                                          \
				    (sys_write32(BIT_WDT_SW(x), event_log_reg);), ());             \
		}                                                                                  \
		printk("\n");                                                                      \
	}                                                                                          \
	(void)(x)

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
	uint32_t jtag_pinmux;
	uint32_t base = DT_REG_ADDR(DT_NODELABEL(syscon));

	/* enable JTAG pins */
	jtag_pinmux = sys_read32(base + JTAG_PINMUX_REG);
	jtag_pinmux |= (0x1f << 25);
	sys_write32(jtag_pinmux, base + JTAG_PINMUX_REG);

	/* clear non-cached .bss */
	(void)memset(__RAM_NC_start, 0, __RAM_NC_end - __RAM_NC_start);

	sys_cache_instr_enable();

#if defined(CONFIG_ASPEED_DC_SCM)
	sys_write32(sys_read32(GPIO_I_L_DIR_REG) | BIT(26) | BIT(27),
				GPIO_I_L_DIR_REG);
	sys_write32((sys_read32(GPIO_I_L_DAT_READ_REG) | BIT(26) | BIT(27)),
				GPIO_I_L_DAT_VAL_REG);

	if ((sys_read32(GPIO_I_L_DAT_READ_REG) & (BIT(26) | BIT(27))) !=
	    (BIT(26) | BIT(27)))
		printk("Fail to enable flash power\n");
#endif

	/* restore UART routing to align datasheet default value */
	sys_write32(0xa30, LPC_HICR9);
	sys_write32(0x0, LPC_HICRA);
}

void aspeed_print_abr_wdt_mode(void)
{
	/* ABR enable */
	if (sys_read32(HW_STRAP2_SCU510) & BIT(11)) {
		printk("FMC ABR: Enable");
		if (sys_read32(HW_STRAP2_SCU510) & BIT(12))
			printk(", Single flash");
		else
			printk(", Dual flashes");

		printk(", Source: %s (%d)",
			(sys_read32(ASPEED_FMC_WDT2_CTRL) & BIT(4)) ? "Alternate" : "Primary",
			(sys_read32(HW_STRAP1_SCU500) & BIT(3)) ? 1 : 0);

		if (sys_read32(HW_STRAP2_SCU510) & GENMASK(15, 13)) {
			printk(", bspi sz: %ldMB",
				BIT((sys_read32(HW_STRAP2_SCU510) >> 13) & 0x7) / 2);
		}
		printk("\n");
	}
}

void aspeed_print_sysrst_info(void)
{
	uint32_t rest1 = sys_read32(SYS_RESET_LOG_REG1);
	uint32_t rest2 = sys_read32(SYS_RESET_LOG_REG2);

	if (rest1 & SYS_PWR_RESET_FLAG) {
		printk("RST: Power On\n");
		SYS_RESET_LOG_CLEAR(rest1, SYS_RESET_LOG_REG1);
	} else {
		HANDLE_WDTx_RESET(4, rest1, SYS_RESET_LOG_REG1);
		HANDLE_WDTx_RESET(3, rest1, SYS_RESET_LOG_REG1);
		HANDLE_WDTx_RESET(2, rest1, SYS_RESET_LOG_REG1);
		HANDLE_WDTx_RESET(1, rest1, SYS_RESET_LOG_REG1);

		if (rest1 & SYS_FLASH_ABR_RESET) {
			printk("RST: SYS_FLASH_ABR_RESET\n");
			SYS_RESET_LOG_CLEAR(SYS_FLASH_ABR_RESET, SYS_RESET_LOG_REG1);
		}

		if (rest1 & SYS_EXT_RESET) {
			printk("RST: External\n");
			SYS_RESET_LOG_CLEAR(SYS_EXT_RESET, SYS_RESET_LOG_REG1);
		}
	}

	ARG_UNUSED(rest2);

	aspeed_print_abr_wdt_mode();
}

#define SOC_ID(str, rev) { .name = str, .rev_id = rev, }

struct soc_id {
	const char *name;
	uint64_t rev_id;
};

static struct soc_id soc_map_table[] = {
	SOC_ID("AST1030-A0", ASPEED_SOC_ID_AST1030A0),
	SOC_ID("AST1030-A1", ASPEED_SOC_ID_AST1030A1),
	SOC_ID("AST1035-A1", ASPEED_SOC_ID_AST1035A1),
	SOC_ID("AST1060-A1", ASPEED_SOC_ID_AST1060A1),
	SOC_ID("AST1060-A2", ASPEED_SOC_ID_AST1060A2),
	SOC_ID("AST1060-A2-ENG", ASPEED_SOC_ID_AST1060A2_ENG),
	SOC_ID("Unknown",    0x0000000000000000),
};

void aspeed_soc_show_chip_id(void)
{
	int i;

	if (IS_ENABLED(CONFIG_HWINFO_ASPEED)) {
		uint64_t rev_id;
		size_t len;

		len = hwinfo_get_device_id((uint8_t *)&rev_id, sizeof(rev_id));
		if (len < 0) {
			return;
		}

		for (i = 0; i < ARRAY_SIZE(soc_map_table); i++) {
			if (rev_id == soc_map_table[i].rev_id) {
				break;
			}
		}

		if (i == ARRAY_SIZE(soc_map_table) && i > 0) {
			i--;
		}
	} else {
		i = ARRAY_SIZE(soc_map_table) -  1;
	}

	printk("SOC: %s\n", soc_map_table[i].name);
}

#if defined(CONFIG_WDT_ASPEED)
void aspeed_wdt_reboot_device(const struct device *dev, int type);

void sys_arch_reboot(int type)
{
	const struct device *dev;
	const char *name = "wdt@1";

	dev = device_get_binding(name);
	if (!dev) {
		printk("No device named %s.\n", name);
		return;
	}

	aspeed_wdt_reboot_device(dev, type);
}
#endif
