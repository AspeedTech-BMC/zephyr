/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Ported to Zephyr from the Linux mipi-i3c-hci driver
 * (drivers/i3c/master/mipi-i3c-hci/).
 */

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "cmd.h"
#include "dat.h"
#include "dct.h"

LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

#define HCI_CMD_DAA_TIMEOUT_MS 1000

/*
 * Address Assignment Command
 */

#define CMD_ATTR(v) FIELD_PREP(CMD_0_ATTR, (v))

#define CMD_A0_DEV_COUNT(v) FIELD_PREP(W0_MASK(29, 26), (v))
#define CMD_A0_DEV_INDEX(v) FIELD_PREP(W0_MASK(22, 16), (v))
#define CMD_A0_CMD(v) FIELD_PREP(W0_MASK(14, 7), (v))
#define CMD_A0_TID(v) FIELD_PREP(W0_MASK(6, 3), (v))

/*
 * Immediate Data Transfer Command
 */

#define CMD_I1_DATA_BYTE_4(v) FIELD_PREP(W1_MASK(63, 56), (v))
#define CMD_I1_DATA_BYTE_3(v) FIELD_PREP(W1_MASK(55, 48), (v))
#define CMD_I1_DATA_BYTE_2(v) FIELD_PREP(W1_MASK(47, 40), (v))
#define CMD_I1_DATA_BYTE_1(v) FIELD_PREP(W1_MASK(39, 32), (v))
#define CMD_I0_RNW W0_BIT_(29)
#define CMD_I0_MODE(v) FIELD_PREP(W0_MASK(28, 26), (v))
#define CMD_I0_DTT(v) FIELD_PREP(W0_MASK(25, 23), (v))
#define CMD_I0_DEV_INDEX(v) FIELD_PREP(W0_MASK(22, 16), (v))
#define CMD_I0_CP W0_BIT_(15)
#define CMD_I0_CMD(v) FIELD_PREP(W0_MASK(14, 7), (v))
#define CMD_I0_TID(v) FIELD_PREP(W0_MASK(6, 3), (v))

/*
 * Regular Data Transfer Command
 */

#define CMD_R1_DATA_LENGTH(v) FIELD_PREP(W1_MASK(63, 48), (v))
#define CMD_R1_DEF_BYTE(v) FIELD_PREP(W1_MASK(39, 32), (v))
#define CMD_R0_RNW W0_BIT_(29)
#define CMD_R0_MODE(v) FIELD_PREP(W0_MASK(28, 26), (v))
#define CMD_R0_DBP W0_BIT_(25)
#define CMD_R0_DEV_INDEX(v) FIELD_PREP(W0_MASK(22, 16), (v))
#define CMD_R0_CP W0_BIT_(15)
#define CMD_R0_CMD(v) FIELD_PREP(W0_MASK(14, 7), (v))
#define CMD_R0_TID(v) FIELD_PREP(W0_MASK(6, 3), (v))

/*
 * Internal Control Command
 */

#define CMD_M0_MIPI_RESERVED(v) FIELD_PREP(W0_MASK(31, 12), (v))
#define CMD_M0_MIPI_CMD(v) FIELD_PREP(W0_MASK(11, 8), (v))
#define CMD_M0_VENDOR_INFO_PRESENT W0_BIT_(7)
#define CMD_M0_TID(v) FIELD_PREP(W0_MASK(6, 3), (v))

/*
 * Target Transfer Command
 */

#define CMD_T0_DATA_LENGTH(v) FIELD_PREP(W0_MASK(31, 16), (v))
#define CMD_T0_MDB(v) FIELD_PREP(W0_MASK(15, 8), (v))
#define CMD_T0_MDB_EN W0_BIT_(6)

enum hci_cmd_mode {
	MODE_I3C_SDR0 = 0x0,
	MODE_I3C_SDR1 = 0x1,
	MODE_I3C_SDR2 = 0x2,
	MODE_I3C_SDR3 = 0x3,
	MODE_I3C_SDR4 = 0x4,
	MODE_I3C_HDR_TS_X = 0x5,
	MODE_I3C_HDR_DDR = 0x6,
	MODE_I3C_HDR_BT = 0x7,
	MODE_I3C_FM_FMP = 0x8,
	MODE_I2C_FM = 0x0,
	MODE_I2C_FMP = 0x1,
};

static enum hci_cmd_mode hci_cmd_v1_get_i3c_mode(struct i3c_hci *hci)
{
	uint32_t rate = hci->common.ctrl_config.scl.i3c;

	if (rate > 8000000U) {
		return MODE_I3C_SDR0;
	}
	if (rate > 6000000U) {
		return MODE_I3C_SDR1;
	}
	if (rate > 4000000U) {
		return MODE_I3C_SDR2;
	}
	if (rate > 2000000U) {
		return MODE_I3C_SDR3;
	}

	return MODE_I3C_SDR4;
}

static enum hci_cmd_mode hci_cmd_v1_get_i2c_mode(struct i3c_hci *hci)
{
	if (hci->common.ctrl_config.scl.i2c >= 1000000U) {
		return MODE_I2C_FMP;
	}

	return MODE_I2C_FM;
}

static enum hci_cmd_mode hci_cmd_v1_get_hdr_mode(uint8_t hdr_mode)
{
	switch (hdr_mode) {
	case I3C_MSG_HDR_DDR:
	case 0:
		return MODE_I3C_HDR_DDR;
	case I3C_MSG_HDR_TSP:
	case I3C_MSG_HDR_TSL:
		return MODE_I3C_HDR_TS_X;
	case I3C_MSG_HDR_BT:
		return MODE_I3C_HDR_BT;
	default:
		return MODE_I3C_HDR_DDR;
	}
}

static void hci_cmd_v1_clear_desc(struct hci_xfer *xfer)
{
	(void)memset(xfer->cmd_desc, 0, sizeof(xfer->cmd_desc));
}

static void hci_cmd_v1_fill_data_bytes(struct hci_xfer *xfer, const uint8_t *data,
				       unsigned int data_len)
{
	xfer->cmd_desc[1] = 0;

	if (!data && data_len != 0U) {
		return;
	}

	switch (data_len) {
	case 4:
		xfer->cmd_desc[1] |= CMD_I1_DATA_BYTE_4(data[3]);
		__fallthrough;
	case 3:
		xfer->cmd_desc[1] |= CMD_I1_DATA_BYTE_3(data[2]);
		__fallthrough;
	case 2:
		xfer->cmd_desc[1] |= CMD_I1_DATA_BYTE_2(data[1]);
		__fallthrough;
	case 1:
		xfer->cmd_desc[1] |= CMD_I1_DATA_BYTE_1(data[0]);
		__fallthrough;
	case 0:
		break;
	default:
		break;
	}

	xfer->data = NULL;
}

static int hci_cmd_v1_dat_index(struct i3c_hci *hci, uint8_t addr,
				unsigned int *dat_idx)
{
	int ret;

	if (addr == I3C_BROADCAST_ADDR) {
		*dat_idx = 0;
		return 0;
	}

	if (!hci->dat || !hci->dat->get_index) {
		return -ENODEV;
	}

	ret = hci->dat->get_index(hci, addr);
	if (ret < 0) {
		return ret;
	}

	*dat_idx = (unsigned int)ret;
	return 0;
}

static int hci_cmd_v1_prep_ccc(struct i3c_hci *hci, struct hci_xfer *xfer,
			       uint8_t ccc_addr, uint8_t ccc_cmd, bool ccc_dbp,
			       uint8_t ccc_db, bool raw)
{
	const uint8_t *data = xfer->data;
	unsigned int data_len = xfer->data_len;
	enum hci_cmd_mode mode = hci_cmd_v1_get_i3c_mode(hci);
	unsigned int dat_idx = 0;
	bool rnw = xfer->rnw;
	int ret;

	if (raw) {
		return -EINVAL;
	}

	ret = hci_cmd_v1_dat_index(hci, ccc_addr, &dat_idx);
	if (ret != 0) {
		return ret;
	}

	hci_cmd_v1_clear_desc(xfer);
	xfer->cmd_tid = hci_get_tid(hci);

	if (!rnw && data_len <= 4U) {
		xfer->cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_I) |
			CMD_I0_TID(xfer->cmd_tid) |
			CMD_I0_CMD(ccc_cmd) |
			CMD_I0_CP |
			CMD_I0_DEV_INDEX(dat_idx) |
			CMD_I0_DTT(data_len) |
			CMD_I0_MODE(mode);
		hci_cmd_v1_fill_data_bytes(xfer, data, data_len);
	} else {
		xfer->cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_R) |
			CMD_R0_TID(xfer->cmd_tid) |
			CMD_R0_CMD(ccc_cmd) |
			CMD_R0_CP |
			CMD_R0_DEV_INDEX(dat_idx) |
			(ccc_dbp ? CMD_R0_DBP : 0U) |
			CMD_R0_MODE(mode) |
			(rnw ? CMD_R0_RNW : 0U);
		xfer->cmd_desc[1] = CMD_R1_DATA_LENGTH(data_len);
		if (ccc_dbp) {
			xfer->cmd_desc[1] |= CMD_R1_DEF_BYTE(ccc_db);
		}
	}

	return 0;
}

static int hci_cmd_v1_prep_hdr(struct i3c_hci *hci, struct hci_xfer *xfer,
			       uint8_t addr, uint8_t code, uint8_t hdr_mode)
{
	const uint8_t *data = xfer->data;
	unsigned int data_len = xfer->data_len;
	enum hci_cmd_mode mode = hci_cmd_v1_get_hdr_mode(hdr_mode);
	unsigned int dat_idx = 0;
	bool rnw = xfer->rnw;
	int ret;

	ret = hci_cmd_v1_dat_index(hci, addr, &dat_idx);
	if (ret != 0) {
		return ret;
	}

	hci_cmd_v1_clear_desc(xfer);
	xfer->cmd_tid = hci_get_tid(hci);

	if (!rnw && data_len <= 4U) {
		xfer->cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_I) |
			CMD_I0_TID(xfer->cmd_tid) |
			CMD_I0_CMD(code & 0x7fU) |
			CMD_I0_CP |
			CMD_I0_DEV_INDEX(dat_idx) |
			CMD_I0_DTT(data_len) |
			CMD_I0_MODE(mode);
		hci_cmd_v1_fill_data_bytes(xfer, data, data_len);
	} else {
		xfer->cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_R) |
			CMD_R0_TID(xfer->cmd_tid) |
			CMD_R0_CMD(code & 0x7fU) |
			CMD_R0_CP |
			CMD_R0_DEV_INDEX(dat_idx) |
			CMD_R0_MODE(mode) |
			(rnw ? CMD_R0_RNW : 0U);
		xfer->cmd_desc[1] = CMD_R1_DATA_LENGTH(data_len);
	}

	return 0;
}

static void hci_cmd_v1_prep_ibi_xfer(struct i3c_hci *hci,
				     struct i3c_device_desc *target,
				     struct hci_xfer *xfer)
{
	const uint8_t *data = xfer->data;
	unsigned int data_len = xfer->data_len > 0U ? xfer->data_len - 1U : 0U;
	uint8_t mdb = (data && xfer->data_len != 0U) ? data[0] : 0;

	ARG_UNUSED(hci);
	ARG_UNUSED(target);

	hci_cmd_v1_clear_desc(xfer);
	xfer->cmd_desc[0] =
		CMD_ATTR(CMD_0_ATTR_I) |
		CMD_T0_TID(xfer->cmd_tid) |
		CMD_T0_MDB_EN |
		CMD_T0_MDB(mdb) |
		CMD_T0_DATA_LENGTH(data_len);
}

static void hci_cmd_v1_prep_i3c_xfer(struct i3c_hci *hci,
				     struct i3c_device_desc *target,
				     struct hci_xfer *xfer)
{
	struct i3c_hci_dev_data *dev_data;
	const uint8_t *data = xfer->data;
	unsigned int data_len = xfer->data_len;
	enum hci_cmd_mode mode = hci_cmd_v1_get_i3c_mode(hci);
	unsigned int dat_idx = 0;
	bool rnw = xfer->rnw;
	int ret;

	dev_data = target ? target->controller_priv : NULL;
	if ((dev_data) && dev_data->dat_idx >= 0) {
		dat_idx = (unsigned int)dev_data->dat_idx;
	} else if (target) {
		ret = hci_cmd_v1_dat_index(hci, target->dynamic_addr, &dat_idx);
		if (ret != 0) {
			return;
		}
	}

	hci_cmd_v1_clear_desc(xfer);
	xfer->cmd_tid = hci_get_tid(hci);

	if (!rnw && data_len <= 4U) {
		xfer->cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_I) |
			CMD_I0_TID(xfer->cmd_tid) |
			CMD_I0_DEV_INDEX(dat_idx) |
			CMD_I0_DTT(data_len) |
			CMD_I0_MODE(mode);
		hci_cmd_v1_fill_data_bytes(xfer, data, data_len);
	} else {
		xfer->cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_R) |
			CMD_R0_TID(xfer->cmd_tid) |
			CMD_R0_DEV_INDEX(dat_idx) |
			CMD_R0_MODE(mode) |
			(rnw ? CMD_R0_RNW : 0U);
		xfer->cmd_desc[1] = CMD_R1_DATA_LENGTH(data_len);
	}
}

static void hci_cmd_v1_prep_i2c_xfer(struct i3c_hci *hci,
				     struct i3c_i2c_device_desc *target,
				     struct hci_xfer *xfer)
{
	struct i3c_hci_dev_data *dev_data;
	const uint8_t *data = xfer->data;
	unsigned int data_len = xfer->data_len;
	enum hci_cmd_mode mode = hci_cmd_v1_get_i2c_mode(hci);
	unsigned int dat_idx = 0;
	bool rnw = xfer->rnw;

	dev_data = target ? target->controller_priv : NULL;
	if ((dev_data) && dev_data->dat_idx >= 0) {
		dat_idx = (unsigned int)dev_data->dat_idx;
	}

	hci_cmd_v1_clear_desc(xfer);
	xfer->cmd_tid = hci_get_tid(hci);

	if (!rnw && data_len <= 4U) {
		xfer->cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_I) |
			CMD_I0_TID(xfer->cmd_tid) |
			CMD_I0_DEV_INDEX(dat_idx) |
			CMD_I0_DTT(data_len) |
			CMD_I0_MODE(mode);
		hci_cmd_v1_fill_data_bytes(xfer, data, data_len);
	} else {
		xfer->cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_R) |
			CMD_R0_TID(xfer->cmd_tid) |
			CMD_R0_DEV_INDEX(dat_idx) |
			CMD_R0_MODE(mode) |
			(rnw ? CMD_R0_RNW : 0U);
		xfer->cmd_desc[1] = CMD_R1_DATA_LENGTH(data_len);
	}
}

static void hci_cmd_v1_prep_internal(struct i3c_hci *hci, struct hci_xfer *xfer,
				     uint8_t sub_cmd, uint32_t param)
{
	hci_cmd_v1_clear_desc(xfer);
	xfer->cmd_tid = hci_get_tid(hci);
	xfer->cmd_desc[0] =
		CMD_ATTR(CMD_0_ATTR_M) |
		CMD_M0_TID(xfer->cmd_tid) |
		CMD_M0_MIPI_CMD(sub_cmd) |
		CMD_M0_MIPI_RESERVED(param);
}

static void hci_cmd_v1_update_daa_target(struct i3c_hci *hci, uint8_t addr,
					 uint64_t pid, unsigned int dcr,
					 unsigned int bcr)
{
	const struct i3c_device_id id = I3C_DEVICE_ID(pid);
	struct i3c_device_desc *target = NULL;
	struct i3c_hci_dev_data *dev_data;
	uint8_t old_addr = 0;

	if (hci->config) {
		target = i3c_dev_list_find(&hci->config->common.dev_list, &id);
	}

	if (!target) {
		/*
		 * The wire-level assignment already happened; without a
		 * matching devicetree child the address is only reserved.
		 * Say so loudly - a PID mismatch here is the usual reason a
		 * DT-described target "has no dynamic address" after DAA.
		 */
		LOG_WRN("DAA: no DT child matches PID 0x%012llx (BCR %#x DCR %#x); "
			"address 0x%02x reserved but not attached",
			(unsigned long long)pid, bcr, dcr, addr);
		i3c_addr_slots_mark_i3c(&hci->common.attached_dev.addr_slots, addr);
		return;
	}

	dev_data = target->controller_priv;
	if (target->dynamic_addr != 0U) {
		old_addr = target->dynamic_addr;
	}

	target->dynamic_addr = addr;
	target->dcr = (uint8_t)dcr;
	target->bcr = (uint8_t)bcr;

	if (old_addr != 0U && old_addr != addr) {
		i3c_addr_slots_mark_free(&hci->common.attached_dev.addr_slots, old_addr);
	}
	i3c_addr_slots_mark_i3c(&hci->common.attached_dev.addr_slots, addr);

	if ((dev_data) && dev_data->dat_idx >= 0 &&
	    hci->dat && hci->dat->set_dynamic_addr) {
		hci->dat->set_dynamic_addr(hci, (unsigned int)dev_data->dat_idx, addr);
		/*
		 * For vendors whose DAT is indexed by dynamic address (ASPEED
		 * G7), set_dynamic_addr relocates the entry from the
		 * originally-allocated slot to `addr`. Sync the cached
		 * dev_data->dat_idx so subsequent CCC / private xfers refer to
		 * the live slot.
		 */
		if (hci->vendor && hci->vendor->dat_wants_addr_indexed &&
		    hci->vendor->dat_wants_addr_indexed(hci)) {
			dev_data->dat_idx = (int)addr;
		}
	}
}

static int hci_cmd_v1_daa(struct i3c_hci *hci)
{
	struct hci_xfer *xfer;
	struct k_sem done;
	uint8_t next_addr = 0x09U;
	uint64_t pid;
	unsigned int dcr;
	unsigned int bcr;
	int dat_idx = -1;
	int ret = 0;

	if (hci->common.ctrl_config.is_secondary) {
		return -ENOTSUP;
	}

	if (!hci->io || !hci->io->queue_xfer) {
		return -ENOSYS;
	}

	xfer = hci_alloc_xfer(1);
	if (!xfer) {
		return -ENOMEM;
	}

	k_sem_init(&done, 0, 1);

	for (;;) {
		ret = i3c_addr_slots_next_free_find(&hci->common.attached_dev.addr_slots,
						    next_addr);
		if (ret == 0) {
			ret = -ENOSPC;
			break;
		}
		next_addr = (uint8_t)ret;

		{
			int preferred_slot = -1;

			if (hci->vendor && hci->vendor->dat_pick_slot) {
				preferred_slot = hci->vendor->dat_pick_slot(hci, next_addr);
			}
			dat_idx = mipi_i3c_hci_dat_v1.alloc_entry(hci, preferred_slot);
		}
		if (dat_idx < 0) {
			ret = dat_idx;
			break;
		}

		if (hci->vendor && hci->vendor->prep_daa_step) {
			hci->vendor->prep_daa_step(hci, next_addr);
		}
		mipi_i3c_hci_dat_v1.set_dynamic_addr(hci, (unsigned int)dat_idx, next_addr);
		mipi_i3c_hci_dct_index_reset(hci);

		hci_cmd_v1_clear_desc(xfer);
		xfer->cmd_tid = hci_get_tid(hci);
		xfer->cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_A) |
			CMD_A0_TID(xfer->cmd_tid) |
			CMD_A0_CMD(I3C_CCC_ENTDAA) |
			CMD_A0_DEV_INDEX(dat_idx) |
			CMD_A0_DEV_COUNT(1) |
			CMD_0_ROC |
			CMD_0_TOC;
		xfer->completion = &done;
		k_sem_reset(&done);

		ret = hci->io->queue_xfer(hci, xfer, 1);
		if (ret != 0) {
			break;
		}

		ret = k_sem_take(&done, K_MSEC(HCI_CMD_DAA_TIMEOUT_MS));
		if (ret != 0) {
			if (hci->io->dequeue_xfer) {
				(void)hci->io->dequeue_xfer(hci, xfer, 1);
			}
			ret = -ETIMEDOUT;
			break;
		}

		if (((RESP_STATUS(xfer->response) == RESP_ERR_ADDR_HEADER) ||
		     (RESP_STATUS(xfer->response) == RESP_ERR_NACK)) &&
		    (RESP_DATA_LENGTH(xfer->response) == 1U)) {
			ret = 0;
			break;
		}

		if (RESP_STATUS(xfer->response) != RESP_SUCCESS) {
			ret = (RESP_STATUS(xfer->response) == RESP_ERR_ADDR_HEADER) ?
				      -ENXIO : -EIO;
			break;
		}

		i3c_hci_dct_get_val(hci, 0, &pid, &dcr, &bcr);
		LOG_DBG("assigned address %#x to PID 0x%012llx DCR %#x BCR %#x",
			next_addr, (unsigned long long)pid, dcr, bcr);

		mipi_i3c_hci_dat_v1.free_entry(hci, (unsigned int)dat_idx);
		dat_idx = -1;
		hci_cmd_v1_update_daa_target(hci, next_addr, pid, dcr, bcr);
	}

	if (dat_idx >= 0) {
		mipi_i3c_hci_dat_v1.free_entry(hci, (unsigned int)dat_idx);
	}

	hci_free_xfer(xfer, 1);
	return ret;
}

const struct hci_cmd_ops mipi_i3c_hci_cmd_v1 = {
	.prep_ccc = hci_cmd_v1_prep_ccc,
	.prep_hdr = hci_cmd_v1_prep_hdr,
	.prep_i3c_xfer = hci_cmd_v1_prep_i3c_xfer,
	.prep_ibi_xfer = hci_cmd_v1_prep_ibi_xfer,
	.prep_i2c_xfer = hci_cmd_v1_prep_i2c_xfer,
	.prep_internal = hci_cmd_v1_prep_internal,
	.perform_daa = hci_cmd_v1_daa,
};
