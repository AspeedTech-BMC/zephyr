/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_SOC_ARM_ASPEED_AST26XX_SOC_H_
#define ZEPHYR_SOC_ARM_ASPEED_AST26XX_SOC_H_
#include <aspeed_util.h>

#define __VTOR_PRESENT			1U
#define __FPU_PRESENT			CONFIG_CPU_HAS_FPU
#define __MPU_PRESENT			CONFIG_CPU_HAS_ARM_MPU

#define PHY_SRAM_ADDR			(*(volatile uint32_t *)(0x7e6e2a04))
#define PHY_SRAM_IMEM_LIMIT		(*(volatile uint32_t *)(0x7e6e2a08))
#define PHY_SRAM_DMEM_LIMIT		(*(volatile uint32_t *)(0x7e6e2a0c))
#define TO_PHY_ADDR(addr)		(PHY_SRAM_ADDR + (uint32_t)(addr))
#define TO_VIR_ADDR(addr)		((uint32_t)(addr) - PHY_SRAM_ADDR)

void aspeed_soc_show_chip_id(void);

#endif /* ZEPHYR_SOC_ARM_ASPEED_AST26XX_SOC_H_*/
