/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2021 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_SOC_ARM_ASPEED_AST10X0_SOC_H_
#define ZEPHYR_SOC_ARM_ASPEED_AST10X0_SOC_H_

#include <sys/util.h>
#include <devicetree.h>

/* Common clock control device name for all Ast10x0 series */
#define ASPEED_CLK_CTRL_NAME DT_INST_CLOCKS_LABEL(0)
/* Common reset control device name for all Ast10x0 series */
#define ASPEED_RST_CTRL_NAME DT_INST_RESETS_LABEL(0)

/* CMSIS required definitions */
#define __FPU_PRESENT  CONFIG_CPU_HAS_FPU
#define __MPU_PRESENT  CONFIG_CPU_HAS_ARM_MPU

/* non-cached (DMA) memory */
#if (CONFIG_SRAM_NC_SIZE > 0)
#define NON_CACHED_BSS	__attribute__((section(".nocache.bss")))
#define NON_CACHED_BSS_ALIGN16	__attribute__((aligned(16), section(".nocache.bss")))
#else
#define NON_CACHED_BSS
#define NON_CACHED_BSS_ALIGN16	__attribute__((aligned(16)))
#endif /* end of "#if (CONFIG_SRAM_NC_SIZE > 0)" */
#endif /* ZEPHYR_SOC_ARM_ASPEED_AST10X0_SOC_H_*/
