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
#include "xfer_mode_rate.h"

LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

#define HCI_CMD_DAA_TIMEOUT_MS 1000

#define CMD_ATTR(v) FIELD_PREP(CMD_0_ATTR, (v))

/*
 * Unified Data Transfer Command
 */

#define CMD_U3_HDR_TSP_ML_CTRL(v) FIELD_PREP(W3_MASK(107, 104), (v))
#define CMD_U3_IDB4(v) FIELD_PREP(W3_MASK(103, 96), (v))
#define CMD_U3_HDR_CMD(v) FIELD_PREP(W3_MASK(103, 96), (v))
#define CMD_U2_IDB3(v) FIELD_PREP(W2_MASK(95, 88), (v))
#define CMD_U2_HDR_BT(v) FIELD_PREP(W2_MASK(95, 88), (v))
#define CMD_U2_IDB2(v) FIELD_PREP(W2_MASK(87, 80), (v))
#define CMD_U2_BT_CMD2(v) FIELD_PREP(W2_MASK(87, 80), (v))
#define CMD_U2_IDB1(v) FIELD_PREP(W2_MASK(79, 72), (v))
#define CMD_U2_BT_CMD1(v) FIELD_PREP(W2_MASK(79, 72), (v))
#define CMD_U2_IDB0(v) FIELD_PREP(W2_MASK(71, 64), (v))
#define CMD_U2_BT_CMD0(v) FIELD_PREP(W2_MASK(71, 64), (v))
#define CMD_U1_ERR_HANDLING(v) FIELD_PREP(W1_MASK(63, 62), (v))
#define CMD_U1_ADD_FUNC(v) FIELD_PREP(W1_MASK(61, 56), (v))
#define CMD_U1_COMBO_XFER W1_BIT_(55)
#define CMD_U1_DATA_LENGTH(v) FIELD_PREP(W1_MASK(53, 32), (v))
#define CMD_U0_MAY_YIELD W0_BIT_(29)
#define CMD_U0_NACK_RCNT(v) FIELD_PREP(W0_MASK(28, 27), (v))
#define CMD_U0_IDB_COUNT(v) FIELD_PREP(W0_MASK(26, 24), (v))
#define CMD_U0_MODE_INDEX(v) FIELD_PREP(W0_MASK(22, 18), (v))
#define CMD_U0_XFER_RATE(v) FIELD_PREP(W0_MASK(17, 15), (v))
#define CMD_U0_DEV_ADDRESS(v) FIELD_PREP(W0_MASK(14, 8), (v))
#define CMD_U0_RNW W0_BIT_(7)
#define CMD_U0_TID(v) FIELD_PREP(W0_MASK(6, 3), (v))

/*
 * Address Assignment Command
 */

#define CMD_A1_DATA_LENGTH(v) FIELD_PREP(W1_MASK(53, 32), (v))
#define CMD_A0_XFER_RATE(v) FIELD_PREP(W0_MASK(17, 15), (v))
#define CMD_A0_ASSIGN_ADDRESS(v) FIELD_PREP(W0_MASK(14, 8), (v))
#define CMD_A0_TID(v) FIELD_PREP(W0_MASK(6, 3), (v))

/*
 * Internal and target command helpers kept for the ops shared with v1.
 */

#define CMD_M0_MIPI_RESERVED(v) FIELD_PREP(W0_MASK(31, 12), (v))
#define CMD_M0_MIPI_CMD(v) FIELD_PREP(W0_MASK(11, 8), (v))
#define CMD_M0_TID(v) FIELD_PREP(W0_MASK(6, 3), (v))
#define CMD_T0_DATA_LENGTH(v) FIELD_PREP(W0_MASK(31, 16), (v))
#define CMD_T0_MDB(v) FIELD_PREP(W0_MASK(15, 8), (v))
#define CMD_T0_MDB_EN W0_BIT_(6)

static unsigned int hci_cmd_v2_get_i3c_rate_idx(struct i3c_hci *hci)
{
	uint32_t rate = hci->common.ctrl_config.scl.i3c;

	if (rate >= 12000000U) {
		return XFERRATE_I3C_SDR0;
	}
	if (rate > 8000000U) {
		return XFERRATE_I3C_SDR1;
	}
	if (rate > 6000000U) {
		return XFERRATE_I3C_SDR2;
	}
	if (rate > 4000000U) {
		return XFERRATE_I3C_SDR3;
	}
	if (rate > 2000000U) {
		return XFERRATE_I3C_SDR4;
	}

	return XFERRATE_I3C_SDR_FM_FMP;
}

static unsigned int hci_cmd_v2_get_i2c_rate_idx(struct i3c_hci *hci)
{
	if (hci->common.ctrl_config.scl.i2c >= 1000000U) {
		return XFERRATE_I2C_FMP;
	}

	return XFERRATE_I2C_FM;
}

static unsigned int hci_cmd_v2_hdr_mode_idx(uint8_t hdr_mode)
{
	switch (hdr_mode) {
	case I3C_MSG_HDR_DDR:
	case 0:
		return XFERMODE_IDX_I3C_HDR_DDR;
	case I3C_MSG_HDR_TSP:
	case I3C_MSG_HDR_TSL:
		return XFERMODE_IDX_I3C_HDR_T;
	case I3C_MSG_HDR_BT:
		return XFERMODE_IDX_I3C_HDR_BT;
	default:
		return XFERMODE_IDX_I3C_HDR_DDR;
	}
}

static void hci_cmd_v2_clear_desc(struct hci_xfer *xfer)
{
	(void)memset(xfer->cmd_desc, 0, sizeof(xfer->cmd_desc));
}

static void hci_cmd_v2_fill_idbs(struct hci_xfer *xfer, const uint8_t *data,
				 unsigned int data_len)
{
	if (!data && data_len != 0U) {
		return;
	}

	switch (data_len) {
	case 5:
		xfer->cmd_desc[3] |= CMD_U3_IDB4(data[4]);
		__fallthrough;
	case 4:
		xfer->cmd_desc[2] |= CMD_U2_IDB3(data[3]);
		__fallthrough;
	case 3:
		xfer->cmd_desc[2] |= CMD_U2_IDB2(data[2]);
		__fallthrough;
	case 2:
		xfer->cmd_desc[2] |= CMD_U2_IDB1(data[1]);
		__fallthrough;
	case 1:
		xfer->cmd_desc[2] |= CMD_U2_IDB0(data[0]);
		__fallthrough;
	case 0:
		break;
	default:
		break;
	}
}

static void hci_cmd_v2_prep_private_xfer(struct i3c_hci *hci, struct hci_xfer *xfer,
					 uint8_t addr, unsigned int mode,
					 unsigned int rate)
{
	const uint8_t *data = xfer->data;
	unsigned int data_len = xfer->data_len;
	bool rnw = xfer->rnw;

	hci_cmd_v2_clear_desc(xfer);
	xfer->cmd_tid = hci_get_tid(hci);

	if (!rnw && data_len <= 5U) {
		xfer->cmd_desc[0] =
			CMD_ATTR(0x4U) |
			CMD_U0_TID(xfer->cmd_tid) |
			CMD_U0_DEV_ADDRESS(addr) |
			CMD_U0_XFER_RATE(rate) |
			CMD_U0_MODE_INDEX(mode) |
			CMD_U0_IDB_COUNT(data_len);
		xfer->cmd_desc[1] = CMD_U1_DATA_LENGTH(0);
		hci_cmd_v2_fill_idbs(xfer, data, data_len);
		xfer->data = NULL;
	} else {
		xfer->cmd_desc[0] =
			CMD_ATTR(0x4U) |
			CMD_U0_TID(xfer->cmd_tid) |
			(rnw ? CMD_U0_RNW : 0U) |
			CMD_U0_DEV_ADDRESS(addr) |
			CMD_U0_XFER_RATE(rate) |
			CMD_U0_MODE_INDEX(mode);
		xfer->cmd_desc[1] = CMD_U1_DATA_LENGTH(data_len);
	}
}

static int hci_cmd_v2_prep_ccc(struct i3c_hci *hci, struct hci_xfer *xfer,
			       uint8_t ccc_addr, uint8_t ccc_cmd, bool dbp,
			       uint8_t db, bool raw)
{
	const uint8_t *data = xfer->data;
	unsigned int data_len = xfer->data_len;
	unsigned int mode = XFERMODE_IDX_I3C_SDR;
	unsigned int rate = hci_cmd_v2_get_i3c_rate_idx(hci);
	bool rnw = xfer->rnw;

	ARG_UNUSED(dbp);
	ARG_UNUSED(db);

	if (raw && ccc_addr != I3C_BROADCAST_ADDR) {
		hci_cmd_v2_prep_private_xfer(hci, xfer, ccc_addr, mode, rate);
		return 0;
	}

	hci_cmd_v2_clear_desc(xfer);
	xfer->cmd_tid = hci_get_tid(hci);

	if (!rnw && data_len <= 4U) {
		xfer->cmd_desc[0] =
			CMD_ATTR(0x4U) |
			CMD_U0_TID(xfer->cmd_tid) |
			CMD_U0_DEV_ADDRESS(ccc_addr) |
			CMD_U0_XFER_RATE(rate) |
			CMD_U0_MODE_INDEX(mode) |
			CMD_U0_IDB_COUNT(data_len + (raw ? 1U : 0U));
		xfer->cmd_desc[1] = CMD_U1_DATA_LENGTH(0);
		xfer->cmd_desc[2] = CMD_U2_IDB0(ccc_cmd);

		switch (data_len) {
		case 4:
			xfer->cmd_desc[3] |= CMD_U3_IDB4(data[3]);
			__fallthrough;
		case 3:
			xfer->cmd_desc[2] |= CMD_U2_IDB3(data[2]);
			__fallthrough;
		case 2:
			xfer->cmd_desc[2] |= CMD_U2_IDB2(data[1]);
			__fallthrough;
		case 1:
			xfer->cmd_desc[2] |= CMD_U2_IDB1(data[0]);
			__fallthrough;
		case 0:
			break;
		default:
			break;
		}
		xfer->data = NULL;
	} else {
		xfer->cmd_desc[0] =
			CMD_ATTR(0x4U) |
			CMD_U0_TID(xfer->cmd_tid) |
			(rnw ? CMD_U0_RNW : 0U) |
			CMD_U0_DEV_ADDRESS(ccc_addr) |
			CMD_U0_XFER_RATE(rate) |
			CMD_U0_MODE_INDEX(mode) |
			CMD_U0_IDB_COUNT(raw ? 1U : 0U);
		xfer->cmd_desc[1] = CMD_U1_DATA_LENGTH(data_len);
		xfer->cmd_desc[2] = CMD_U2_IDB0(ccc_cmd);
	}

	return 0;
}

static void hci_cmd_v2_prep_hdr_xfer(struct i3c_hci *hci, struct hci_xfer *xfer,
				     uint8_t addr, uint8_t code, unsigned int mode,
				     unsigned int rate)
{
	const uint8_t *data = xfer->data;
	unsigned int data_len = xfer->data_len;
	bool rnw = xfer->rnw;
	unsigned int max_idb = mode == XFERMODE_IDX_I3C_HDR_BT ? 0U : 4U;

	hci_cmd_v2_clear_desc(xfer);
	xfer->cmd_tid = hci_get_tid(hci);

	if (!rnw && data_len <= max_idb) {
		xfer->cmd_desc[0] =
			CMD_ATTR(0x4U) |
			CMD_U0_TID(xfer->cmd_tid) |
			CMD_U0_DEV_ADDRESS(addr) |
			CMD_U0_XFER_RATE(rate) |
			CMD_U0_MODE_INDEX(mode) |
			CMD_U0_IDB_COUNT(data_len);
		xfer->cmd_desc[1] = CMD_U1_DATA_LENGTH(0);
		hci_cmd_v2_fill_idbs(xfer, data, data_len);
		xfer->data = NULL;
	} else {
		xfer->cmd_desc[0] =
			CMD_ATTR(0x4U) |
			CMD_U0_TID(xfer->cmd_tid) |
			(rnw ? CMD_U0_RNW : 0U) |
			CMD_U0_DEV_ADDRESS(addr) |
			CMD_U0_XFER_RATE(rate) |
			CMD_U0_MODE_INDEX(mode);
		xfer->cmd_desc[1] = CMD_U1_DATA_LENGTH(data_len);
	}

	if (mode == XFERMODE_IDX_I3C_HDR_BT) {
		xfer->cmd_desc[2] |= CMD_U2_BT_CMD0(code & 0x7fU);
	} else {
		xfer->cmd_desc[3] |= CMD_U3_HDR_CMD(code & 0x7fU);
	}
}

static int hci_cmd_v2_prep_hdr(struct i3c_hci *hci, struct hci_xfer *xfer,
			       uint8_t addr, uint8_t code, uint8_t hdr_mode)
{
	unsigned int mode = hci_cmd_v2_hdr_mode_idx(hdr_mode);
	unsigned int rate = hci_cmd_v2_get_i3c_rate_idx(hci);

	switch (hdr_mode) {
	case I3C_MSG_HDR_DDR:
	case I3C_MSG_HDR_TSP:
	case I3C_MSG_HDR_TSL:
	case I3C_MSG_HDR_BT:
	case 0:
		break;
	default:
		return -EINVAL;
	}

	hci_cmd_v2_prep_hdr_xfer(hci, xfer, addr, code, mode, rate);

	return 0;
}

static void hci_cmd_v2_prep_i3c_xfer(struct i3c_hci *hci,
				     struct i3c_device_desc *target,
				     struct hci_xfer *xfer)
{
	unsigned int mode = XFERMODE_IDX_I3C_SDR;
	unsigned int rate = hci_cmd_v2_get_i3c_rate_idx(hci);
	uint8_t addr = target ? target->dynamic_addr : 0;

	hci_cmd_v2_prep_private_xfer(hci, xfer, addr, mode, rate);
}

static void hci_cmd_v2_prep_ibi_xfer(struct i3c_hci *hci,
				     struct i3c_device_desc *target,
				     struct hci_xfer *xfer)
{
	const uint8_t *data = xfer->data;
	unsigned int data_len = xfer->data_len > 0U ? xfer->data_len - 1U : 0U;
	uint8_t mdb = (data && xfer->data_len != 0U) ? data[0] : 0;

	ARG_UNUSED(hci);
	ARG_UNUSED(target);

	hci_cmd_v2_clear_desc(xfer);
	xfer->cmd_desc[0] =
		CMD_ATTR(CMD_0_ATTR_I) |
		CMD_T0_TID(xfer->cmd_tid) |
		CMD_T0_MDB_EN |
		CMD_T0_MDB(mdb) |
		CMD_T0_DATA_LENGTH(data_len);
}

static void hci_cmd_v2_prep_i2c_xfer(struct i3c_hci *hci,
				     struct i3c_i2c_device_desc *target,
				     struct hci_xfer *xfer)
{
	unsigned int mode = XFERMODE_IDX_I2C;
	unsigned int rate = hci_cmd_v2_get_i2c_rate_idx(hci);
	uint8_t addr = target ? (uint8_t)target->addr : 0;

	hci_cmd_v2_prep_private_xfer(hci, xfer, addr, mode, rate);
}

static void hci_cmd_v2_prep_internal(struct i3c_hci *hci, struct hci_xfer *xfer,
				     uint8_t sub_cmd, uint32_t param)
{
	hci_cmd_v2_clear_desc(xfer);
	xfer->cmd_tid = hci_get_tid(hci);
	xfer->cmd_desc[0] =
		CMD_ATTR(CMD_0_ATTR_M) |
		CMD_M0_TID(xfer->cmd_tid) |
		CMD_M0_MIPI_CMD(sub_cmd) |
		CMD_M0_MIPI_RESERVED(param);
}

static void hci_cmd_v2_update_daa_target(struct i3c_hci *hci, uint8_t addr,
					 uint64_t pid, unsigned int dcr,
					 unsigned int bcr)
{
	const struct i3c_device_id id = I3C_DEVICE_ID(pid);
	struct i3c_device_desc *target = NULL;
	uint8_t old_addr = 0;

	if (hci->config) {
		target = i3c_dev_list_find(&hci->config->common.dev_list, &id);
	}

	if (!target) {
		i3c_addr_slots_mark_i3c(&hci->common.attached_dev.addr_slots, addr);
		return;
	}

	if (target->dynamic_addr != 0U) {
		old_addr = target->dynamic_addr;
	} else if (target->init_dynamic_addr != 0U) {
		old_addr = target->init_dynamic_addr;
	} else if (target->static_addr != 0U) {
		old_addr = target->static_addr;
	}

	target->dynamic_addr = addr;
	target->dcr = (uint8_t)dcr;
	target->bcr = (uint8_t)bcr;

	if (old_addr != 0U && old_addr != addr) {
		i3c_addr_slots_mark_free(&hci->common.attached_dev.addr_slots, old_addr);
	}
	i3c_addr_slots_mark_i3c(&hci->common.attached_dev.addr_slots, addr);
}

static int hci_cmd_v2_daa(struct i3c_hci *hci)
{
	struct hci_xfer *xfer;
	struct k_sem done;
	uint32_t device_id[2] = {0};
	uint8_t next_addr = 0;
	uint64_t pid;
	unsigned int dcr;
	unsigned int bcr;
	int ret = 0;

	if (hci->common.ctrl_config.is_secondary) {
		return -ENOTSUP;
	}

	if (!hci->io || !hci->io->queue_xfer) {
		return -ENOSYS;
	}

	xfer = hci_alloc_xfer(2);
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

		(void)memset(device_id, 0, sizeof(device_id));
		(void)memset(xfer, 0, 2U * sizeof(*xfer));

		xfer[0].data = device_id;
		xfer[0].data_len = sizeof(device_id);
		xfer[0].rnw = true;
		xfer[0].cmd_tid = hci_get_tid(hci);
		xfer[0].cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_A) |
			CMD_A0_TID(xfer[0].cmd_tid) |
			CMD_0_ROC;
		xfer[0].cmd_desc[1] = CMD_A1_DATA_LENGTH(sizeof(device_id));

		xfer[1].cmd_tid = hci_get_tid(hci);
		xfer[1].cmd_desc[0] =
			CMD_ATTR(CMD_0_ATTR_A) |
			CMD_A0_TID(xfer[1].cmd_tid) |
			CMD_A0_ASSIGN_ADDRESS(next_addr) |
			CMD_0_ROC |
			CMD_0_TOC;
		xfer[1].completion = &done;
		k_sem_reset(&done);

		ret = hci->io->queue_xfer(hci, xfer, 2);
		if (ret != 0) {
			break;
		}

		ret = k_sem_take(&done, K_MSEC(HCI_CMD_DAA_TIMEOUT_MS));
		if (ret != 0) {
			if (hci->io->dequeue_xfer) {
				(void)hci->io->dequeue_xfer(hci, xfer, 2);
			}
			ret = -ETIMEDOUT;
			break;
		}

		if (RESP_STATUS(xfer[0].response) != RESP_SUCCESS) {
			ret = 0;
			break;
		}
		if (RESP_STATUS(xfer[1].response) != RESP_SUCCESS) {
			ret = -EIO;
			break;
		}

		pid = FIELD_GET(W1_MASK(47, 32), device_id[1]);
		pid = (pid << 32) | device_id[0];
		bcr = FIELD_GET(W1_MASK(55, 48), device_id[1]);
		dcr = FIELD_GET(W1_MASK(63, 56), device_id[1]);

		LOG_DBG("%s assigned address %#x to PID 0x%012llx DCR %#x BCR %#x",
			hci->dev->name, next_addr, (unsigned long long)pid, dcr, bcr);
		hci_cmd_v2_update_daa_target(hci, next_addr, pid, dcr, bcr);
	}

	hci_free_xfer(xfer, 2);
	return ret;
}

const struct hci_cmd_ops mipi_i3c_hci_cmd_v2 = {
	.prep_ccc = hci_cmd_v2_prep_ccc,
	.prep_hdr = hci_cmd_v2_prep_hdr,
	.prep_i3c_xfer = hci_cmd_v2_prep_i3c_xfer,
	.prep_ibi_xfer = hci_cmd_v2_prep_ibi_xfer,
	.prep_i2c_xfer = hci_cmd_v2_prep_i2c_xfer,
	.prep_internal = hci_cmd_v2_prep_internal,
	.perform_daa = hci_cmd_v2_daa,
};
