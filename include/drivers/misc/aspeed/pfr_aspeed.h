/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_PFR_ASPEED_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_PFR_ASPEED_H_

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>

void  pfr_bmc_rst_enable_ctrl(bool enable);
void pfr_bmc_rst_flash(uint32_t flash_idx);

#endif
