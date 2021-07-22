/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define MBX_DAT_REG_NUM	32

int mbox_aspeed_read(const struct device *dev, uint8_t *buf, size_t count, uint32_t off);
int mbox_aspeed_write(const struct device *dev, uint8_t *buf, size_t count, uint32_t off);
