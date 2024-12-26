/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <string.h>
#include <soc.h>

#define SCU0_REG		DT_REG_ADDR(DT_NODELABEL(syscon0))
#define SCU1_REG		DT_REG_ADDR(DT_NODELABEL(syscon1))
#define SCU1_RSTLOG0		(SCU1_REG + 0x050)
#define   SCU1_RSTLOG0_SRST	BIT(0)

extern char __RAM_NC_start[];
extern char __RAM_NC_end[];

#if IS_ENABLED(CONFIG_DT_HAS_ASPEED_AST_WATCHDOG_G7_ENABLED)
#define WDT_REG			DT_REG_ADDR(DT_NODELABEL(wdt0))

#define WDT_DEVS		9
#define WDT_RSTMASK_1_VAL	0x0203e779
#define WDT_RSTMASK_2_VAL	0x000003f6
#define WDT_RSTMASK_3_VAL	0x000093ec
#define WDT_RSTMASK_4_VAL	0x00303803
#define WDT_RSTMASK_5_VAL	0x00020000

#define WDT_RSTMASK1(x)		(WDT_REG + ((x) * 0x80) + 0x1c)
#define WDT_RSTMASK2(x)		(WDT_REG + ((x) * 0x80) + 0x20)
#define WDT_RSTMASK3(x)		(WDT_REG + ((x) * 0x80) + 0x24)
#define WDT_RSTMASK4(x)		(WDT_REG + ((x) * 0x80) + 0x28)
#define WDT_RSTMASK5(x)		(WDT_REG + ((x) * 0x80) + 0x2c)
#define WDT_SW_RSTMASK1(x)	(WDT_REG + ((x) * 0x80) + 0x34)
#define WDT_SW_RSTMASK2(x)	(WDT_REG + ((x) * 0x80) + 0x38)
#define WDT_SW_RSTMASK3(x)	(WDT_REG + ((x) * 0x80) + 0x3c)
#define WDT_SW_RSTMASK4(x)	(WDT_REG + ((x) * 0x80) + 0x40)
#define WDT_SW_RSTMASK5(x)	(WDT_REG + ((x) * 0x80) + 0x44)

static void soc_wdt_mask_init(void)
{
	/*
	 * FIXME:
	 * The SRST reset log flag has been cleared by ROM code.
	 * There is no way to know whether the SoC boots from SRST.
	 *
	 * if (!(sys_read32(SCU1_RSTLOG0) & SCU1_RSTLOG0_SRST))
	 *      return;
	 */

	for (int idx = 0; idx < WDT_DEVS; idx++) {
		/* SoC reset mask */
		sys_write32(WDT_RSTMASK_1_VAL, WDT_RSTMASK1(idx));
		sys_write32(WDT_RSTMASK_2_VAL, WDT_RSTMASK2(idx));
		sys_write32(WDT_RSTMASK_3_VAL, WDT_RSTMASK3(idx));
		sys_write32(WDT_RSTMASK_4_VAL, WDT_RSTMASK4(idx));
		sys_write32(WDT_RSTMASK_5_VAL, WDT_RSTMASK5(idx));

		/* SW reset mask */
		sys_write32(WDT_RSTMASK_1_VAL, WDT_SW_RSTMASK1(idx));
		sys_write32(WDT_RSTMASK_2_VAL, WDT_SW_RSTMASK2(idx));
		sys_write32(WDT_RSTMASK_3_VAL, WDT_SW_RSTMASK3(idx));
		sys_write32(WDT_RSTMASK_4_VAL, WDT_SW_RSTMASK4(idx));
		sys_write32(WDT_RSTMASK_5_VAL, WDT_SW_RSTMASK5(idx));
	}
}

void sys_arch_reboot(int type)
{
	/*
	 * FIXME:
	 * Use aspeed_wdt_reboot_device once if the watchdog driver is ready
	 */
	sys_write32(0, WDT_REG + 0xc);
	sys_write32(0x100, WDT_REG + 0x4);
	sys_write32(0x4755, WDT_REG + 0x8);
	sys_write32(0x13, WDT_REG + 0xc);
}
#endif


#if defined(CONFIG_ARM) && defined(CONFIG_PLATFORM_SPECIFIC_INIT)
void z_arm_platform_init(void)
{
	/* clear non-cached .bss */
	(void)memset(__RAM_NC_start, 0, __RAM_NC_end - __RAM_NC_start);

	sys_cache_instr_enable();
	sys_cache_data_enable();
#if IS_ENABLED(CONFIG_DT_HAS_ASPEED_AST_WATCHDOG_G7_ENABLED)
	soc_wdt_mask_init();
#endif
}
#endif

#if defined(CONFIG_RISCV)
static int soc_init(void)
{
#if IS_ENABLED(CONFIG_DT_HAS_ASPEED_AST_WATCHDOG_G7_ENABLED)
	soc_wdt_mask_init();
#endif
	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, 0);
#endif
