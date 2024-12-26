/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_DRIVERS_MISC_ASPEED_CPTRA_MBOX_H_
#define ZEPHYR_DRIVERS_MISC_ASPEED_CPTRA_MBOX_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

uint32_t cptra_mbox_csum(uint32_t csum, uint8_t *data, uint32_t dlen);
uint32_t cptra_mbox_status(void);
int cptra_mbox_lock(void);
int cptra_mbox_unlock(void);

#endif /* ZEPHYR_DRIVERS_MISC_ASPEED_CPTRA_MBOX_H_ */
