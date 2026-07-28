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
/* SSP/TSP use a fixed virtual window for direct AHB matrix access. */
#define AST27XX_SSP_TSP_AHB_MATRIX_VIRT_BASE  0x60000000UL
#define AST27XX_SSP_TSP_AHB_MATRIX_VIRT_LIMIT 0xE0000000UL
#define AST27XX_SSP_TSP_AHB_MATRIX_ADDR_LIMIT 0x40000000ULL
#define AST27XX_IS_SSP_TSP_AHB_MATRIX_VIRT_ADDR(addr)                                              \
	((addr) >= AST27XX_SSP_TSP_AHB_MATRIX_VIRT_BASE &&                                         \
	 (addr) < AST27XX_SSP_TSP_AHB_MATRIX_VIRT_LIMIT)
#define AST27XX_IS_SSP_TSP_AHB_MATRIX_ADDR(addr) ((addr) < AST27XX_SSP_TSP_AHB_MATRIX_ADDR_LIMIT)
#define AST27XX_SSP_TSP_AHB_MATRIX_VIRT_TO_ADDR(addr)                                              \
	((uint64_t)(addr) - AST27XX_SSP_TSP_AHB_MATRIX_VIRT_BASE)
#define AST27XX_SSP_TSP_AHB_MATRIX_ADDR_TO_VIRT(addr)                                              \
	((uintptr_t)(addr) + AST27XX_SSP_TSP_AHB_MATRIX_VIRT_BASE)

extern char __RAM_NC_start[];
extern char __RAM_NC_end[];

#if IS_ENABLED(CONFIG_WDT_ASPEED) && DT_NODE_HAS_STATUS(DT_NODELABEL(wdt0), okay)
void aspeed_wdt_reboot_device(const struct device *dev, int type);

void sys_arch_reboot(int type)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));

	aspeed_wdt_reboot_device(dev, type);
}
#endif

/* DMA address translation */
#if defined(CONFIG_SOC_AST2700_BOOTMCU) || defined(CONFIG_SOC_AST2705_BOOTMCU)
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

#if defined(CONFIG_SOC_AST2700_TSP) || defined(CONFIG_SOC_AST2700_A1_TSP) || \
	defined(CONFIG_SOC_AST2705_TSP)
uint64_t ast27xx_soc_virt_addr_to_phy_addr(uintptr_t addr)
{
	if (AST27XX_IS_SSP_TSP_AHB_MATRIX_VIRT_ADDR(addr))
		return AST27XX_SSP_TSP_AHB_MATRIX_VIRT_TO_ADDR(addr);

	return ((uint64_t)sys_read32(SCU0_REG + 0x168) << 4) + addr;
}

uintptr_t ast27xx_soc_phy_addr_to_virt_addr(uint64_t addr)
{
	if (AST27XX_IS_SSP_TSP_AHB_MATRIX_ADDR(addr))
		return AST27XX_SSP_TSP_AHB_MATRIX_ADDR_TO_VIRT(addr);

	return addr - ((uint64_t)sys_read32(SCU0_REG + 0x168) << 4);
}
#endif

#if defined(CONFIG_SOC_AST2700_SSP) || defined(CONFIG_SOC_AST2700_A1_SSP) || \
	defined(CONFIG_SOC_AST2705_SSP)
uint64_t ast27xx_soc_virt_addr_to_phy_addr(uintptr_t addr)
{
	uintptr_t base;
	uintptr_t limit;
	uint64_t phy_dram_base;

	if (AST27XX_IS_SSP_TSP_AHB_MATRIX_VIRT_ADDR(addr))
		return AST27XX_SSP_TSP_AHB_MATRIX_VIRT_TO_ADDR(addr);

	/* ssp tcm remap region */
	base = sys_read32(SCU0_REG + 0x140);
	limit = base + sys_read32(SCU0_REG + 0x144);
	if (addr >= base && addr < limit)
		return 0x10800000 + addr - base;

	/* addr is in MBUS remap region */
	base = sys_read32(SCU0_REG + 0x150);
	limit = base + sys_read32(SCU0_REG + 0x154);
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

	if (AST27XX_IS_SSP_TSP_AHB_MATRIX_ADDR(addr))
		return AST27XX_SSP_TSP_AHB_MATRIX_ADDR_TO_VIRT(addr);

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
/*
 * Runs from reset.S before z_prep_c(), i.e. before .bss is zeroed, .data is
 * copied, and __stack_chk_guard is initialised in z_cstart(). The stack
 * canary must be disabled here: with CONFIG_STACK_CANARIES the prologue/epilogue
 * would check a guard that is uninitialised (and may even be cleared by the
 * memset below if it lands in this region), tripping __stack_chk_fail().
 */
FUNC_NO_STACK_PROTECTOR void z_arm_platform_init(void)
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
