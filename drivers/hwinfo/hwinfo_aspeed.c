/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>
#include <string.h>

#define ASPEED_REVISION_ID0     0x04
#define ASPEED_REVISION_ID1     0x14

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint32_t base = DT_REG_ADDR(DT_NODELABEL(syscon));
	uint64_t rev_id;

	if (length < sizeof(rev_id))
		return -1;

	rev_id = sys_read32(base + ASPEED_REVISION_ID0);
	rev_id = ((uint64_t)sys_read32(base + ASPEED_REVISION_ID1) << 32) | rev_id;
	memcpy(buffer, &rev_id, sizeof(rev_id));

	return sizeof(rev_id);
}
