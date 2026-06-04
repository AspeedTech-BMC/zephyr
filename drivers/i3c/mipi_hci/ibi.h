/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Ported to Zephyr from the Linux mipi-i3c-hci driver
 * (drivers/i3c/master/mipi-i3c-hci/).
 */

#ifndef ZEPHYR_DRIVERS_I3C_MIPI_HCI_IBI_H_
#define ZEPHYR_DRIVERS_I3C_MIPI_HCI_IBI_H_

#include "hci.h"

#ifndef I3C_HOT_JOIN_ADDR
#define I3C_HOT_JOIN_ADDR 0x02U
#endif

#define IBI_STS BIT(31)
#define IBI_ERROR BIT(30)
#define IBI_STATUS_TYPE BIT(29)
#define IBI_HW_CONTEXT GENMASK(28, 26)
#define IBI_TS BIT(25)
#define IBI_LAST_STATUS BIT(24)
#define IBI_CHUNKS GENMASK(23, 16)
#define IBI_ID GENMASK(15, 8)
#define IBI_TARGET_ADDR GENMASK(15, 9)
#define IBI_TARGET_RNW BIT(8)
#define IBI_DATA_LENGTH GENMASK(7, 0)

#define IBI_TYPE_HJ(a, rnw) (((a) == I3C_HOT_JOIN_ADDR) && !(rnw))
#define IBI_TYPE_CR(a, rnw) (((a) != I3C_HOT_JOIN_ADDR) && !(rnw))

static inline struct i3c_device_desc *i3c_hci_addr_to_dev(struct i3c_hci *hci,
							  unsigned int addr)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(addr);
	return NULL;
}

#endif /* ZEPHYR_DRIVERS_I3C_MIPI_HCI_IBI_H_ */
