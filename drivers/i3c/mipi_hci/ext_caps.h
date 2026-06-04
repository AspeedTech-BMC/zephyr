/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Ported to Zephyr from the Linux mipi-i3c-hci driver
 * (drivers/i3c/master/mipi-i3c-hci/).
 */

#ifndef ZEPHYR_DRIVERS_I3C_MIPI_HCI_EXT_CAPS_H_
#define ZEPHYR_DRIVERS_I3C_MIPI_HCI_EXT_CAPS_H_

#include "hci.h"

#define MIPI_VENDOR_NXP 0x11b
#define MIPI_VENDOR_ASPEED 0x3f6

int i3c_hci_parse_ext_caps(struct i3c_hci *hci);

#endif /* ZEPHYR_DRIVERS_I3C_MIPI_HCI_EXT_CAPS_H_ */
