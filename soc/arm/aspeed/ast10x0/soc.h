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

#define SYS_RESET_LOG_REG1	0x7e6e2074
#define SYS_RESET_LOG_REG2	0x7e6e2078

#define HW_STRAP1_SCU500	0x7e6e2500
#define HW_STRAP2_SCU510	0x7e6e2510
#define ASPEED_FMC_WDT2_CTRL	0x7e620064

void aspeed_print_sysrst_info(void);
void aspeed_soc_show_chip_id(void);

int aspeed_otp_read_data(uint32_t offset, uint32_t *buf, uint32_t len);
int aspeed_otp_read_conf(uint32_t offset, uint32_t *buf, uint32_t len);
int aspeed_otp_prog_data(uint32_t offset, uint32_t *buf, uint32_t len);
int aspeed_otp_prog_conf(uint32_t offset, uint32_t *buf, uint32_t len);

#endif /* ZEPHYR_SOC_ARM_ASPEED_AST10X0_SOC_H_*/
