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
#include <zephyr/sys/reboot.h>

#define SCU1_REG		DT_REG_ADDR(DT_NODELABEL(syscon1))
#define SCU1_RSTLOG0		(SCU1_REG + 0x050)
#define   SCU1_RSTLOG0_SRST	BIT(0)

extern char __RAM_NC_start[];
extern char __RAM_NC_end[];

void z_arm_platform_init(void)
{
	/* clear non-cached .bss */
	(void)memset(__RAM_NC_start, 0, __RAM_NC_end - __RAM_NC_start);
}

#if IS_ENABLED(CONFIG_WDT_ASPEED) && defined(CONFIG_SOC_SERIES_AST10x0_G2_CM4) && \
	DT_NODE_HAS_STATUS(DT_NODELABEL(wdt0), okay)
void aspeed_wdt_reboot_device(const struct device *dev, int type);

void sys_arch_reboot(int type)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));

	aspeed_wdt_reboot_device(dev, type);
}
#endif

/* DMA address translation */
#if defined(CONFIG_SOC_SERIES_AST10x0_G2_BOOTMCU)
uint64_t ast10x0_g2_soc_virt_addr_to_phy_addr(uintptr_t addr)
{
	if (addr < 0x80000000) {
		return addr;
	}

	return (uint64_t)(addr & ~0x80000000) | 0x400000000ULL;
}

uintptr_t ast10x0_g2_soc_phy_addr_to_virt_addr(uint64_t addr)
{
	if (addr < 0x400000000) {
		return addr;
	}

	return (uintptr_t)(addr - 0x400000000ULL) | 0x80000000UL;
}
#elif defined(CONFIG_SOC_SERIES_AST10x0_G2_CM4)
/*
 * AST10x0-G2 Cortex-M4 reaches the bus through two remap windows.
 * Anything outside both windows is identity-mapped.
 *
 *   System memory window  (HyperRAM / DRAM aperture)
 *     CPU view : 0x00000000 .. 0x00ffffff  (low 16 MiB)
 *     Bus view : 0xf0000000 .. 0xf0ffffff
 *
 *     The CM4 sees the system memory bank at the bottom of its address
 *     space, but on the bus side it sits in the 0xf0000000+ aperture.
 *
 *   S-Bus -> AHB aperture  (peripheral access)
 *     CPU view : 0x60000000 .. 0xdfffffff  (2 GiB)
 *     Bus view : 0x00000000 .. 0x7fffffff  (AHB starts at 0x0)
 *
 *     The CM4 talks to AHB peripherals through the S-Bus aperture; the
 *     S-Bus bridge subtracts the aperture base so that CPU 0x60000000
 *     lands on AHB 0x00000000.
 */
#define CM4_SYSMEM_VIRT_BASE	0x00000000UL
#define CM4_SYSMEM_SIZE		0x01000000UL          /* 16 MiB */
#define CM4_SYSMEM_PHYS_BASE	0xf0000000UL

#define CM4_SBUS_VIRT_BASE	0x60000000UL
#define CM4_SBUS_SIZE		0x80000000UL          /* 2 GiB S-Bus aperture */
#define CM4_AHB_PHYS_BASE	0x00000000UL

uint64_t ast10x0_g2_soc_virt_addr_to_phy_addr(uintptr_t addr)
{
	if (addr >= CM4_SYSMEM_VIRT_BASE &&
	    addr < CM4_SYSMEM_VIRT_BASE + CM4_SYSMEM_SIZE) {
		return (uint64_t)(addr - CM4_SYSMEM_VIRT_BASE +
				  CM4_SYSMEM_PHYS_BASE);
	}

	if (addr >= CM4_SBUS_VIRT_BASE &&
	    addr < CM4_SBUS_VIRT_BASE + CM4_SBUS_SIZE) {
		return (uint64_t)(addr - CM4_SBUS_VIRT_BASE +
				  CM4_AHB_PHYS_BASE);
	}

	return (uint64_t)addr;
}

uintptr_t ast10x0_g2_soc_phy_addr_to_virt_addr(uint64_t addr)
{
	if (addr >= CM4_SYSMEM_PHYS_BASE &&
	    addr < CM4_SYSMEM_PHYS_BASE + CM4_SYSMEM_SIZE) {
		return (uintptr_t)(addr - CM4_SYSMEM_PHYS_BASE +
				   CM4_SYSMEM_VIRT_BASE);
	}

	if (addr >= CM4_AHB_PHYS_BASE &&
	    addr < CM4_AHB_PHYS_BASE + CM4_SBUS_SIZE) {
		return (uintptr_t)(addr - CM4_AHB_PHYS_BASE +
				   CM4_SBUS_VIRT_BASE);
	}

	return (uintptr_t)addr;
}
#endif
