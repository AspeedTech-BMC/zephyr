/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_I3C_TARGET_MQUEUE_H_
#define ZEPHYR_INCLUDE_DRIVERS_I3C_TARGET_MQUEUE_H_
int i3c_target_mqueue_write(const struct device *dev, uint8_t *buf, int len);
int i3c_target_mqueue_read(const struct device *dev, uint8_t *buf, int budget);
#endif
