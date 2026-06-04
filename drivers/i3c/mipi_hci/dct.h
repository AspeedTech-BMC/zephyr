/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Ported to Zephyr from the Linux mipi-i3c-hci driver
 * (drivers/i3c/master/mipi-i3c-hci/).
 */

#ifndef ZEPHYR_DRIVERS_I3C_MIPI_HCI_DCT_H_
#define ZEPHYR_DRIVERS_I3C_MIPI_HCI_DCT_H_

#include "hci.h"

struct hci_dct_ops {
	void (*read_entry)(struct i3c_hci *hci, unsigned int dct_idx,
			   uint64_t *pid, unsigned int *dcr, unsigned int *bcr);
};

void i3c_hci_dct_get_val(struct i3c_hci *hci, unsigned int dct_idx,
			 uint64_t *pid, unsigned int *dcr, unsigned int *bcr);

extern const struct hci_dct_ops mipi_i3c_hci_dct_v1;

#endif /* ZEPHYR_DRIVERS_I3C_MIPI_HCI_DCT_H_ */
