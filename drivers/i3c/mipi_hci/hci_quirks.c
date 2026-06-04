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

#include "ext_caps.h"
#include "hci.h"

LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

/* Timing registers used by the AMD AMDI5017 platform quirk. */
#define HCI_SCL_I3C_OD_TIMING 0x214
#define HCI_SCL_I3C_PP_TIMING 0x218
#define HCI_SDA_HOLD_SWITCH_DLY_TIMING 0x230

/* Timing values configure a 9 MHz bus on the affected AMD platforms. */
#define AMD_SCL_I3C_OD_TIMING 0x00cf00cf
#define AMD_SCL_I3C_PP_TIMING 0x00160016

#define QUEUE_THLD_CTRL 0xd0

void mipi_i3c_hci_resume(struct i3c_hci *hci)
{
	hci_reg_set(hci, HC_CONTROL, HC_CONTROL_RESUME);
}

void mipi_i3c_hci_pio_reset(struct i3c_hci *hci)
{
	hci_reg_write(hci, RESET_CONTROL,
		      RX_FIFO_RST | TX_FIFO_RST | RESP_QUEUE_RST | CMD_QUEUE_RST);
}

void mipi_i3c_hci_pio_ibi_reset(struct i3c_hci *hci)
{
	hci_reg_write(hci, RESET_CONTROL, IBI_QUEUE_RST);
}

void mipi_i3c_hci_dct_index_reset(struct i3c_hci *hci)
{
	hci_reg_write(hci, DCT_SECTION, FIELD_PREP(DCT_TABLE_INDEX, 0));
}

void amd_set_od_pp_timing(struct i3c_hci *hci)
{
	uint32_t data;

	hci_reg_write(hci, HCI_SCL_I3C_OD_TIMING, AMD_SCL_I3C_OD_TIMING);
	hci_reg_write(hci, HCI_SCL_I3C_PP_TIMING, AMD_SCL_I3C_PP_TIMING);

	data = hci_reg_read(hci, HCI_SDA_HOLD_SWITCH_DLY_TIMING);
	data |= W0_MASK(18, 16);
	hci_reg_write(hci, HCI_SDA_HOLD_SWITCH_DLY_TIMING, data);
}

void amd_set_resp_buf_thld(struct i3c_hci *hci)
{
	uint32_t data;

	data = hci_reg_read(hci, QUEUE_THLD_CTRL);
	data &= ~W0_MASK(15, 8);
	hci_reg_write(hci, QUEUE_THLD_CTRL, data);
}

void mipi_i3c_hci_hj_ctrl(struct i3c_hci *hci, bool ack_nack)
{
	if (ack_nack) {
		hci_reg_clear(hci, HC_CONTROL, HC_CONTROL_HOT_JOIN_CTRL);
	} else {
		hci_reg_set(hci, HC_CONTROL, HC_CONTROL_HOT_JOIN_CTRL);
	}
}

void mipi_i3c_hci_apply_quirks(struct i3c_hci *hci)
{
	if (hci->config) {
		hci->quirks |= hci->config->quirks;
	}

	switch (hci->vendor_mipi_id) {
	case MIPI_VENDOR_NXP:
		hci->quirks |= HCI_QUIRK_RAW_CCC;
		LOG_DBG("raw CCC quirk enabled for NXP HCI");
		break;
	default:
		break;
	}
}
