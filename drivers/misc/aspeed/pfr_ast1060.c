/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <kernel.h>
#include <soc.h>
#include <errno.h>
#include <string.h>
#include <logging/log.h>
#include <drivers/clock_control.h>
#include <drivers/gpio.h>

#define PFR_SCU_CTRL_REG	0x7e6e20f0

void pfr_bmc_rst_enable_ctrl(bool enable)
{
	const struct device *gpio_dev = NULL;

	/* GPIOM5 */
	gpio_dev = device_get_binding("GPIO0_M_P");
	if (gpio_dev == NULL) {
		printk("[%d]Fail to get GPIO0_M_P", __LINE__);
		return;
	}
	gpio_pin_configure(gpio_dev, 5, GPIO_OUTPUT);
	k_busy_wait(10000); /* 10ms */

	if (enable)
		gpio_pin_set(gpio_dev, 5, 0);
	else
		gpio_pin_set(gpio_dev, 5, 1);

	k_busy_wait(50000); /* 50ms */
}

void pfr_bmc_rst_flash(uint32_t flash_idx)
{
	uint32_t reg_val;
	uint32_t bit_off = 1 << (flash_idx - 1);

	/* Using SCU0F0 to enable flash rst
	 * SCU0F0[23:20]: Reset source selection
	 * SCU0F0[27:24]: Enable reset signal output
	 */
	reg_val = sys_read32(PFR_SCU_CTRL_REG);
	reg_val |= (bit_off << 20) | (bit_off << 24);
	sys_write32(reg_val, PFR_SCU_CTRL_REG);

	/* SCU0F0[19:16]: output value */
	/* reset flash */
	reg_val = sys_read32(PFR_SCU_CTRL_REG);
	reg_val &= ~(bit_off << 16);
	sys_write32(reg_val, PFR_SCU_CTRL_REG);

	k_busy_wait(500000); /* 500ms */

	/* release reset */
	reg_val = sys_read32(PFR_SCU_CTRL_REG);
	reg_val |= (bit_off << 16);
	sys_write32(reg_val, PFR_SCU_CTRL_REG);
	k_busy_wait(50000); /* 50ms */
}

