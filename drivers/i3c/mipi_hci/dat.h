/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Ported to Zephyr from the Linux mipi-i3c-hci driver
 * (drivers/i3c/master/mipi-i3c-hci/).
 */

#ifndef ZEPHYR_DRIVERS_I3C_MIPI_HCI_DAT_H_
#define ZEPHYR_DRIVERS_I3C_MIPI_HCI_DAT_H_

#include "hci.h"

#define DAT_0_I2C_DEVICE W0_BIT_(31)
#define DAT_0_SIR_REJECT W0_BIT_(13)
#define DAT_0_IBI_PAYLOAD W0_BIT_(12)

struct hci_dat_ops {
	int (*init)(struct i3c_hci *hci);
	void (*cleanup)(struct i3c_hci *hci);
	/*
	 * Allocate a DAT slot. `preferred_slot` is a hint: if non-negative
	 * and that slot is free, alloc_entry uses it; otherwise it falls
	 * back to the first-free slot. Callers without a preference pass
	 * -1. The hint exists for vendor-specific DAT layouts where the
	 * silicon expects DAT[dynamic_addr] to hold the device (e.g.
	 * ASPEED G7).
	 */
	int (*alloc_entry)(struct i3c_hci *hci, int preferred_slot);
	void (*free_entry)(struct i3c_hci *hci, unsigned int dat_idx);
	void (*set_dynamic_addr)(struct i3c_hci *hci, unsigned int dat_idx, uint8_t addr);
	void (*set_static_addr)(struct i3c_hci *hci, unsigned int dat_idx, uint8_t addr);
	void (*set_flags)(struct i3c_hci *hci, unsigned int dat_idx,
			  uint32_t w0, uint32_t w1);
	void (*clear_flags)(struct i3c_hci *hci, unsigned int dat_idx,
			    uint32_t w0, uint32_t w1);
	int (*get_index)(struct i3c_hci *hci, uint8_t address);
	void (*mark_for_sw_retry)(struct i3c_hci *hci, unsigned int dat_idx, bool mark);
};

extern const struct hci_dat_ops mipi_i3c_hci_dat_v1;

#endif /* ZEPHYR_DRIVERS_I3C_MIPI_HCI_DAT_H_ */
