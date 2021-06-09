/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2021 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_SOC_ARM_ASPEED_AST10X0_SOC_H_
#define ZEPHYR_SOC_ARM_ASPEED_AST10X0_SOC_H_
#include <aspeed_util.h>

#define __CM4_REV		0x0001U
#define __VTOR_PRESENT		1U
#define __NVIC_PRIO_BITS	NUM_IRQ_PRIO_BITS
#define __Vendor_SysTickConfig	0U
#define __FPU_PRESENT		CONFIG_CPU_HAS_FPU
#define __MPU_PRESENT		CONFIG_CPU_HAS_ARM_MPU

#define PHY_SRAM_ADDR           0x80000000UL
#define TO_PHY_ADDR(addr)       (PHY_SRAM_ADDR + (uint32_t)(addr))
#define TO_VIR_ADDR(addr)       ((uint32_t)(addr) - PHY_SRAM_ADDR)

#endif /* ZEPHYR_SOC_ARM_ASPEED_AST10X0_SOC_H_*/
