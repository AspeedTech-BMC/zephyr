/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2026 ASPEED Technology Inc.
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/sys_io.h>

#include "scu1_clk_lock.h"

#define SCU1_REG			DT_REG_ADDR(DT_NODELABEL(syscon1))

/* Clock gate1 lock register: locks the SCU1_240 clock-stop control bits */
#define SCU1_CLKGATE_LOCK1		(SCU1_REG + 0x250)
#define   SCU1_CLKGATE_LOCK1_OTP_BIT	6

/* Clock select1 lock register: locks the SCU1_280 clock division fields */
#define SCU1_CLKSEL1_LOCK		(SCU1_REG + 0x2A0)
#define   SCU1_CLKSEL1_LOCK_CALIPTRA_BIT	18

/* Clock select2 lock register: locks the SCU1_284 clock division fields */
#define SCU1_CLKSEL2_LOCK		(SCU1_REG + 0x2B0)
#define   SCU1_CLKSEL2_LOCK_IO_BOOTMCU_BIT	20

void ast27xx_scu1_clk_lock(void)
{
	/* OTP (REFCLK 25M) clock-stop control: lock SCU1_240[6] */
	sys_set_bit(SCU1_CLKGATE_LOCK1, SCU1_CLKGATE_LOCK1_OTP_BIT);

	/* Caliptra (PCLK) clock division: lock SCU1_280[20:18] */
	sys_set_bit(SCU1_CLKSEL1_LOCK, SCU1_CLKSEL1_LOCK_CALIPTRA_BIT);

	/* IO_SRAM/IO_AHB/BootMCU (HCLK) clock division: lock SCU1_284[22:20] */
	sys_set_bit(SCU1_CLKSEL2_LOCK, SCU1_CLKSEL2_LOCK_IO_BOOTMCU_BIT);
}
