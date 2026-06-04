/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Ported to Zephyr from the Linux mipi-i3c-hci driver
 * (drivers/i3c/master/mipi-i3c-hci/).
 */

#include <zephyr/logging/log.h>

#include "dct.h"

LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

#define DCT_V1_ENTRY_WORDS 4U
#define DCT_V1_ENTRY_SIZE (DCT_V1_ENTRY_WORDS * sizeof(uint32_t))

static void hci_dct_v1_read_entry(struct i3c_hci *hci, unsigned int dct_idx,
				  uint64_t *pid, unsigned int *dcr, unsigned int *bcr)
{
	uintptr_t reg;
	uint32_t dct_entry_data[DCT_V1_ENTRY_WORDS] = {0};

	if (!hci || hci->DCT_regs == 0U ||
	    dct_idx >= hci->DCT_entries || hci->DCT_entry_size != DCT_V1_ENTRY_SIZE) {
		if (pid) {
			*pid = 0;
		}
		if (dcr) {
			*dcr = 0;
		}
		if (bcr) {
			*bcr = 0;
		}
		return;
	}

	reg = hci->DCT_regs + (uintptr_t)dct_idx * DCT_V1_ENTRY_SIZE;

	for (unsigned int i = 0; i < DCT_V1_ENTRY_WORDS; i++) {
		dct_entry_data[i] = sys_read32((mem_addr_t)(reg + i * sizeof(uint32_t)));
	}

	if (pid) {
		*pid = ((uint64_t)dct_entry_data[0] << 16) |
		       FIELD_GET(W1_MASK(47, 32), dct_entry_data[1]);
	}
	if (dcr) {
		*dcr = FIELD_GET(W2_MASK(71, 64), dct_entry_data[2]);
	}
	if (bcr) {
		*bcr = FIELD_GET(W2_MASK(79, 72), dct_entry_data[2]);
	}
}

void i3c_hci_dct_get_val(struct i3c_hci *hci, unsigned int dct_idx,
			 uint64_t *pid, unsigned int *dcr, unsigned int *bcr)
{
	mipi_i3c_hci_dct_v1.read_entry(hci, dct_idx, pid, dcr, bcr);
}

const struct hci_dct_ops mipi_i3c_hci_dct_v1 = {
	.read_entry = hci_dct_v1_read_entry,
};
