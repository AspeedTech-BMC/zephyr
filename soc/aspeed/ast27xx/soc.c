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

void sys_arch_reboot(int type)
{
	/*
	 * FIXME:
	 * Use aspeed_wdt_reboot_device once if the watchdog driver is ready
	 */
	sys_write32(0, WDT_REG + 0xc);
	k_usleep(5);
	sys_write32(0x100, WDT_REG + 0x4);
	k_usleep(5);
	sys_write32(0x4755, WDT_REG + 0x8);
	k_usleep(5);
	sys_write32(0x13, WDT_REG + 0xc);
}
#endif

/* DMA address translation */
#if defined(CONFIG_SOC_AST2700_BOOTMCU)
uint64_t ast27xx_soc_virt_addr_to_phy_addr(uintptr_t addr)
{
	if (addr < 0x80000000) {
		return addr;
	}

	return (uint64_t)(addr & ~0x80000000) | 0x400000000ULL;
}

uintptr_t ast27xx_soc_phy_addr_to_virt_addr(uint64_t addr)
{
	if (addr < 0x400000000) {
		return addr;
	}

	return (uintptr_t)(addr - 0x400000000ULL) | 0x80000000UL;
}
#endif

#if defined(CONFIG_SOC_AST2700_TSP) || defined(CONFIG_SOC_AST2700_A0_TSP)
uint64_t ast27xx_soc_virt_addr_to_phy_addr(uintptr_t addr)
{
	return ((uint64_t)sys_read32(SCU0_REG + 0x168) << 4) + addr;
}

uintptr_t ast27xx_soc_phy_addr_to_virt_addr(uint64_t addr)
{
	return addr - ((uint64_t)sys_read32(SCU0_REG + 0x168) << 4);
}
#endif

#if defined(CONFIG_SOC_AST2700_SSP) || defined(CONFIG_SOC_AST2700_A0_SSP)
uint64_t ast27xx_soc_virt_addr_to_phy_addr(uintptr_t addr)
{
	uintptr_t base = sys_read32(SCU0_REG + 0x150);
	uintptr_t limit = base + sys_read32(SCU0_REG + 0x154);
	uint64_t phy_dram_base;

	if (addr >= 0x70000000 && addr < 0x70020000)
		return addr;

	/* addr is in MBUS remap region */
	if (addr >= base && addr < limit) {
		phy_dram_base = (uint64_t)sys_read32(SCU0_REG + 0x128) << 4;
		goto out;
	}

	/* addr is in AHB remap region */
	base = sys_read32(SCU0_REG + 0x148);
	limit = base + sys_read32(SCU0_REG + 0x14c);
	if (addr >= base && addr < limit) {
		phy_dram_base = (uint64_t)sys_read32(SCU0_REG + 0x124) << 4;
		addr -= base;
		goto out;
	}

	/* Never reach here */
	printk("\nFailed to map physical memory\n");
	CODE_UNREACHABLE;
out:
	return (uint64_t)addr + phy_dram_base;
}

uintptr_t ast27xx_soc_phy_addr_to_virt_addr(uint64_t addr)
{
	uint64_t base = (uint64_t)sys_read32(SCU0_REG + 0x128) << 4;
	uint64_t limit = base + sys_read32(SCU0_REG + 0x154);

	/* addr is in MBUS remap region */
	if (addr >= base && addr < limit) {
		goto out;
	}

	/* addr is in AHB remap region */
	base = (uint64_t)sys_read32(SCU0_REG + 0x124) << 4;
	limit = base + sys_read32(SCU0_REG + 0x14c);
	if (addr >= base && addr < limit) {
		base += sys_read32(SCU0_REG + 0x148);
		goto out;
	}

	/* Never reach here */
	printk("\nFailed to map virtual memory\n");
	CODE_UNREACHABLE;
out:
	return (uintptr_t)(addr - base);
}
#endif

#if defined(CONFIG_ARM) && defined(CONFIG_PLATFORM_SPECIFIC_INIT)
void z_arm_platform_init(void)
{
	/* clear non-cached .bss */
	(void)memset(__RAM_NC_start, 0, __RAM_NC_end - __RAM_NC_start);
}
#endif

#if defined(CONFIG_RISCV)
static int soc_init(void)
{
	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, 0);
#endif
