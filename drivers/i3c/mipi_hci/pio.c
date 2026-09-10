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
#include "hci.h"
LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);
#define PIO_COMMAND_QUEUE_PORT 0x00
#define PIO_RESPONSE_QUEUE_PORT 0x04
#define PIO_XFER_DATA_PORT 0x08
#define PIO_IBI_PORT 0x0c
#define PIO_QUEUE_THLD_CTRL 0x10
#define QUEUE_IBI_STATUS_THLD GENMASK(31, 24)
#define QUEUE_IBI_DATA_THLD GENMASK(23, 16)
#define QUEUE_RESP_BUF_THLD GENMASK(15, 8)
#define QUEUE_CMD_EMPTY_BUF_THLD GENMASK(7, 0)
#define PIO_DATA_BUFFER_THLD_CTRL 0x14
#define DATA_RX_BUF_THLD GENMASK(10, 8)
#define DATA_TX_BUF_THLD GENMASK(2, 0)
#define PIO_QUEUE_SIZE 0x18
#define TX_DATA_BUFFER_SIZE GENMASK(31, 24)
#define RX_DATA_BUFFER_SIZE GENMASK(23, 16)
#define IBI_STATUS_SIZE GENMASK(15, 8)
#define CR_QUEUE_SIZE GENMASK(7, 0)
#define PIO_INTR_STATUS 0x20
#define PIO_INTR_STATUS_ENABLE 0x24
#define PIO_INTR_SIGNAL_ENABLE 0x28
#define STAT_TRANSFER_BLOCKED BIT(25)
#define STAT_PERR_RESP_UFLOW BIT(24)
#define STAT_PERR_CMD_OFLOW BIT(23)
#define STAT_PERR_IBI_UFLOW BIT(22)
#define STAT_PERR_RX_UFLOW BIT(21)
#define STAT_PERR_TX_OFLOW BIT(20)
#define STAT_ERR_RESP_QUEUE_FULL BIT(19)
#define STAT_WARN_RESP_QUEUE_FULL BIT(18)
#define STAT_ERR_IBI_QUEUE_FULL BIT(17)
#define STAT_WARN_IBI_QUEUE_FULL BIT(16)
#define STAT_ERR_RX_DATA_FULL BIT(15)
#define STAT_WARN_RX_DATA_FULL BIT(14)
#define STAT_ERR_TX_DATA_EMPTY BIT(13)
#define STAT_WARN_TX_DATA_EMPTY BIT(12)
#define STAT_TRANSFER_ERR BIT(9)
#define STAT_WARN_INS_STOP_MODE BIT(7)
#define STAT_TRANSFER_ABORT BIT(5)
#define STAT_RESP_READY BIT(4)
#define STAT_CMD_QUEUE_READY BIT(3)
#define STAT_IBI_STATUS_THLD BIT(2)
#define STAT_RX_THLD BIT(1)
#define STAT_TX_THLD BIT(0)
#define PIO_QUEUE_CUR_STATUS 0x38
#define CUR_IBI_Q_LEVEL GENMASK(28, 20)
#define CUR_RESP_Q_LEVEL GENMASK(18, 10)
#define CUR_CMD_Q_EMPTY_LEVEL GENMASK(8, 0)
#define PIO_DATA_BUFFER_CUR_STATUS 0x3c
#define CUR_RX_BUF_LVL GENMASK(26, 16)
#define CUR_TX_BUF_LVL GENMASK(10, 0)
#define STAT_LATENCY_WARNINGS \
	(STAT_WARN_RESP_QUEUE_FULL | STAT_WARN_IBI_QUEUE_FULL | \
	 STAT_WARN_RX_DATA_FULL | STAT_WARN_TX_DATA_EMPTY | STAT_WARN_INS_STOP_MODE)
#define STAT_LATENCY_ERRORS \
	(STAT_ERR_RESP_QUEUE_FULL | STAT_ERR_IBI_QUEUE_FULL | \
	 STAT_ERR_RX_DATA_FULL | STAT_ERR_TX_DATA_EMPTY)
#define STAT_PROG_ERRORS \
	(STAT_TRANSFER_BLOCKED | STAT_PERR_RESP_UFLOW | STAT_PERR_CMD_OFLOW | \
	 STAT_PERR_IBI_UFLOW | STAT_PERR_RX_UFLOW | STAT_PERR_TX_OFLOW)
#define STAT_ALL_ERRORS (STAT_TRANSFER_ABORT | STAT_TRANSFER_ERR | \
			 STAT_LATENCY_ERRORS | STAT_PROG_ERRORS)
#define IBI_ERROR BIT(30)
#define IBI_LAST_STATUS BIT(24)
#define IBI_TARGET_ADDR GENMASK(15, 9)
#define IBI_TARGET_RNW BIT(8)
#define IBI_DATA_LENGTH GENMASK(7, 0)
#ifndef I3C_HOT_JOIN_ADDR
#define I3C_HOT_JOIN_ADDR 0x02U
#endif
#define IBI_TYPE_HJ(addr, rnw) (((addr) == I3C_HOT_JOIN_ADDR) && !(rnw))
#define IBI_TYPE_CR(addr, rnw) (((addr) != I3C_HOT_JOIN_ADDR) && !(rnw))
#if defined(CONFIG_I3C_IBI_WORKQUEUE)
#define HCI_PIO_IBI_SLOT_COUNT CONFIG_I3C_IBI_WORKQUEUE_LENGTH
#else
#define HCI_PIO_IBI_SLOT_COUNT 4
#endif
struct hci_pio_ibi_slot {
	struct hci_pio_ibi_slot *next;
	struct i3c_device_desc *target;
	struct i3c_ibi_payload payload;
};

struct hci_pio_dev_ibi_data {
	struct hci_pio_ibi_slot *slots;
	struct hci_pio_ibi_slot *free_slots;
	unsigned int num_slots;
	unsigned int max_len;
};

struct hci_pio_ibi_data {
	struct hci_pio_ibi_slot *slot;
	uint8_t *data_ptr;
	unsigned int addr;
	unsigned int seg_len;
	unsigned int seg_cnt;
	unsigned int max_len;
	bool last_seg;
};

struct hci_pio_data {
	struct hci_xfer *curr_xfer;
	struct hci_xfer *xfer_tail;
	struct hci_xfer *curr_rx;
	struct hci_xfer *rx_tail;
	struct hci_xfer *curr_tx;
	struct hci_xfer *tx_tail;
	struct hci_xfer *curr_resp;
	struct hci_xfer *resp_tail;
	struct hci_pio_ibi_data ibi;
	unsigned int rx_thresh_size;
	unsigned int tx_thresh_size;
	unsigned int max_ibi_thresh;
	uint32_t reg_queue_thresh;
	uint32_t enabled_irqs;
	unsigned int ibi_subscribers;
	bool hj_enabled;
};

static inline uint32_t hci_pio_read(struct i3c_hci *hci, uint32_t reg)
{
	return sys_read32((mem_addr_t)(hci->PIO_regs + reg));
}

static inline void hci_pio_write(struct i3c_hci *hci, uint32_t reg, uint32_t val)
{
	sys_write32(val, (mem_addr_t)(hci->PIO_regs + reg));
}

static unsigned int hci_pio_cmd_desc_words(struct i3c_hci *hci)
{
	if (hci->is_target) {
		return 1U;
	}
	return (hci->cmd == &mipi_i3c_hci_cmd_v2) ? 4U : 2U;
}

static void hci_pio_set_signal(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	hci_pio_write(hci, PIO_INTR_SIGNAL_ENABLE, pio->enabled_irqs);
}

static void hci_pio_word_to_buf(void *buf, uint32_t word, unsigned int len)
{
	(void)memcpy(buf, &word, len);
}

static uint32_t hci_pio_word_from_buf(const void *buf, unsigned int len)
{
	uint32_t word = 0;
	(void)memcpy(&word, buf, len);
	return word;
}

static void hci_pio_read_data_word(struct i3c_hci *hci, uint8_t **buf,
				   unsigned int len)
{
	uint32_t word = hci_pio_read(hci, PIO_XFER_DATA_PORT);

	hci_pio_word_to_buf(*buf, word, len);
	*buf += len;
}

static void hci_pio_write_data_word(struct i3c_hci *hci, const uint8_t **buf,
				    unsigned int len)
{
	uint32_t word = hci_pio_word_from_buf(*buf, len);

	hci_pio_write(hci, PIO_XFER_DATA_PORT, word);
	*buf += len;
}

static void hci_pio_read_ibi_word(struct i3c_hci *hci, uint8_t **buf,
				  unsigned int len)
{
	uint32_t word = hci_pio_read(hci, PIO_IBI_PORT);

	hci_pio_word_to_buf(*buf, word, len);
	*buf += len;
}

static void hci_pio_write_cmd(struct i3c_hci *hci, struct hci_xfer *xfer)
{
	unsigned int desc_words = hci_pio_cmd_desc_words(hci);

	for (unsigned int i = 0; i < desc_words; i++) {
		LOG_DBG("%s cmd_desc[%u] = %#x", hci->dev->name, i, xfer->cmd_desc[i]);
		hci_pio_write(hci, PIO_COMMAND_QUEUE_PORT, xfer->cmd_desc[i]);
	}
}

static bool hci_pio_do_rx(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	struct hci_xfer *xfer = pio->curr_rx;
	uint8_t *buf;

	if (!xfer || !xfer->data) {
		return true;
	}
	buf = (uint8_t *)xfer->data + (xfer->data_len - xfer->data_left);
	while (xfer->data_left >= 4U) {
		unsigned int nr_words;

		if ((hci_pio_read(hci, PIO_INTR_STATUS) & STAT_RX_THLD) == 0U) {
			return false;
		}
		nr_words = MIN(xfer->data_left / 4U, pio->rx_thresh_size);
		xfer->data_left -= nr_words * 4U;
		while (nr_words-- != 0U) {
			hci_pio_read_data_word(hci, &buf, 4U);
		}
	}
	return xfer->data_left == 0U;
}

static void hci_pio_target_read_rx_fifo(struct i3c_hci *hci, unsigned int count)
{
	uint8_t *buf = hci->target_rx.buf;
	unsigned int copy_len = MIN(count, (unsigned int)hci->target_rx.max_len);
	unsigned int remaining = count;

	if (!buf || hci->target_rx.max_len == 0U) {
		copy_len = 0U;
	}
	while (remaining >= 4U) {
		uint32_t word = hci_pio_read(hci, PIO_XFER_DATA_PORT);

		if (copy_len >= 4U) {
			hci_pio_word_to_buf(buf, word, 4U);
			buf += 4U;
			copy_len -= 4U;
		}
		remaining -= 4U;
	}
	if (remaining != 0U) {
		uint32_t word = hci_pio_read(hci, PIO_XFER_DATA_PORT);

		if (copy_len != 0U) {
			hci_pio_word_to_buf(buf, word, MIN(copy_len, remaining));
		}
	}
}

static void hci_pio_do_trailing_rx(struct i3c_hci *hci, struct hci_pio_data *pio,
				   unsigned int count)
{
	struct hci_xfer *xfer = pio->curr_rx;
	uint8_t *buf;

	if (!xfer || !xfer->data || count == 0U) {
		return;
	}
	buf = (uint8_t *)xfer->data + (xfer->data_len - xfer->data_left);
	while (count >= 4U) {
		hci_pio_read_data_word(hci, &buf, 4U);
		xfer->data_left -= 4U;
		count -= 4U;
	}
	if (count != 0U) {
		uint32_t word = hci_pio_read(hci, PIO_XFER_DATA_PORT);

		xfer->data_word_before_partial = word;
		hci_pio_word_to_buf(buf, word, count);
		xfer->data_left -= count;
	}
}

static void hci_pio_move_extra_rx(struct i3c_hci *hci, struct hci_xfer *xfer,
				  unsigned int expected)
{
	unsigned int received = xfer->data_len - xfer->data_left;
	unsigned int extra;
	uint8_t *src;
	struct hci_xfer *next;

	ARG_UNUSED(hci);
	if (received <= expected || !xfer->data) {
		return;
	}
	extra = received - expected;
	src = (uint8_t *)xfer->data + expected;
	xfer->data_left = 0U;
	next = xfer->next_data;
	while (next && extra != 0U) {
		unsigned int next_received = next->data_len - next->data_left;
		unsigned int room = next->data_len - next_received;
		unsigned int chunk = MIN(extra, room);
		uint8_t *dst = next->data;

		if (!dst || room == 0U) {
			next = next->next_data;
			continue;
		}
		if (next_received != 0U) {
			(void)memmove(dst + chunk, dst, next_received);
		}
		(void)memcpy(dst, src, chunk);
		next->data_left -= chunk;
		src += chunk;
		extra -= chunk;
		next = next->next_data;
	}
	if (extra != 0U) {
		LOG_ERR("%s dropping %u RX bytes after short read", hci->dev->name, extra);
	}
}

static bool hci_pio_do_tx(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	struct hci_xfer *xfer = pio->curr_tx;
	const uint8_t *buf;

	if (!xfer || !xfer->data) {
		return true;
	}
	buf = (const uint8_t *)xfer->data + (xfer->data_len - xfer->data_left);
	while (xfer->data_left >= 4U) {
		unsigned int nr_words;

		if ((hci_pio_read(hci, PIO_INTR_STATUS) & STAT_TX_THLD) == 0U) {
			return false;
		}
		nr_words = MIN(xfer->data_left / 4U, pio->tx_thresh_size);
		xfer->data_left -= nr_words * 4U;
		while (nr_words-- != 0U) {
			hci_pio_write_data_word(hci, &buf, 4U);
		}
	}
	if (xfer->data_left != 0U) {
		if ((hci_pio_read(hci, PIO_INTR_STATUS) & STAT_TX_THLD) == 0U) {
			return false;
		}
		hci_pio_write_data_word(hci, &buf, xfer->data_left);
		xfer->data_left = 0U;
	}
	return true;
}

static bool hci_pio_process_rx(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	while (pio->curr_rx && hci_pio_do_rx(hci, pio)) {
		pio->curr_rx = pio->curr_rx->next_data;
	}
	if (!pio->curr_rx) {
		pio->rx_tail = NULL;
	}
	return !pio->curr_rx;
}

static bool hci_pio_process_tx(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	while (pio->curr_tx && hci_pio_do_tx(hci, pio)) {
		pio->curr_tx = pio->curr_tx->next_data;
	}
	if (!pio->curr_tx) {
		pio->tx_tail = NULL;
	}
	return !pio->curr_tx;
}

static void hci_pio_queue_data(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	struct hci_xfer *xfer = pio->curr_xfer;

	if (!xfer) {
		return;
	}
	if (!xfer->data || xfer->data_len == 0U) {
		xfer->data_left = 0U;
		return;
	}
	if (xfer->rnw) {
		if (pio->curr_rx) {
			pio->rx_tail->next_data = xfer;
			pio->rx_tail = xfer;
		} else {
			pio->curr_rx = xfer;
			pio->rx_tail = xfer;
			if (!hci_pio_process_rx(hci, pio)) {
				pio->enabled_irqs |= STAT_RX_THLD;
			}
		}
	} else {
		if (pio->curr_tx) {
			pio->tx_tail->next_data = xfer;
			pio->tx_tail = xfer;
		} else {
			pio->curr_tx = xfer;
			pio->tx_tail = xfer;
			if (!hci_pio_process_tx(hci, pio)) {
				pio->enabled_irqs |= STAT_TX_THLD;
			}
		}
	}
}

static bool hci_pio_xfer_has_response(struct hci_xfer *xfer)
{
	if ((xfer->cmd_desc[0] & CMD_0_ROC) != 0U) {
		return true;
	}
	return (xfer->cmd_desc[0] & CMD_0_ATTR) == CMD_0_ATTR_M;
}

static void hci_pio_err(struct i3c_hci *hci, struct hci_pio_data *pio,
			uint32_t status);
static void hci_pio_target_handle_response(struct i3c_hci *hci,
					   struct hci_pio_data *pio,
					   uint32_t resp)
{
	unsigned int nbytes = TARGET_RESP_DATA_LENGTH(resp);
	unsigned int status = TARGET_RESP_STATUS(resp);
	/*
	 * TARGET_RESP_XFER_TYPE is target-centric: TYPE_R means the target
	 * received data (= controller wrote to us); TYPE_W means the target
	 * transmitted data (= controller read from us). Use a clear name.
	 */
	bool target_received = TARGET_RESP_XFER_TYPE(resp) == TARGET_RESP_XFER_TYPE_R;
	bool ccc = TARGET_RESP_CCC_INDICATE(resp) != 0U;
	unsigned int tid = TARGET_RESP_TID(resp);

	LOG_DBG("%s target resp=%#x status=%u type=%u tid=%u ccc=%u len=%u",
		hci->dev->name, resp, status, (unsigned int)TARGET_RESP_XFER_TYPE(resp),
		tid, ccc, nbytes);
	if (target_received) {
		if (nbytes > hci->target_rx.max_len) {
			LOG_ERR("%s target write length %u exceeds RX buffer %u",
				hci->dev->name, nbytes, hci->target_rx.max_len);
			hci_pio_target_read_rx_fifo(hci, nbytes);
		} else {
			hci_pio_target_read_rx_fifo(hci, nbytes);
			if (ccc) {
				if (hci->vendor && hci->vendor->ccc_handler) {
					hci->vendor->ccc_handler(hci,
								 TARGET_RESP_CCC_HDR(resp));
				}
			} else {
				mipi_i3c_hci_target_rx_data(hci, hci->target_rx.buf,
							    nbytes);
			}
		}
	} else if (status == TARGET_RESP_SUCCESS) {
		if (tid == TID_TARGET_IBI) {
			k_sem_give(&hci->ibi_comp);
		} else if (tid == TID_TARGET_RD_DATA) {
			k_sem_give(&hci->pending_r_comp);
		}
	}
	if (status >= TARGET_RESP_ERR_CRC &&
	    status <= TARGET_RESP_ERR_I2C_READ_TOO_MUCH) {
		LOG_ERR("%s target transfer error %#x", hci->dev->name, status);
		hci_pio_err(hci, pio, 0U);
	}
}

static bool hci_pio_process_resp(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	if (hci->is_target) {
		while ((hci_pio_read(hci, PIO_INTR_STATUS) & STAT_RESP_READY) != 0U) {
			uint32_t resp = hci_pio_read(hci, PIO_RESPONSE_QUEUE_PORT);

			hci_pio_target_handle_response(hci, pio, resp);
		}
		return false;
	}
	while (pio->curr_resp &&
	       (hci_pio_read(hci, PIO_INTR_STATUS) & STAT_RESP_READY) != 0U) {
		struct hci_xfer *xfer = pio->curr_resp;
		uint32_t resp = hci_pio_read(hci, PIO_RESPONSE_QUEUE_PORT);
		unsigned int tid = RESP_TID(resp);

		LOG_DBG("%s resp=%#x", hci->dev->name, resp);
		if (tid != xfer->cmd_tid) {
			LOG_ERR("%s response tid=%u when expecting %u",
				hci->dev->name, tid, xfer->cmd_tid);
			hci_pio_err(hci, pio, STAT_PROG_ERRORS);
			return false;
		}
		xfer->response = resp;
		if (pio->curr_rx == xfer) {
			unsigned int received = xfer->data_len - xfer->data_left;
			unsigned int expected = RESP_DATA_LENGTH(resp);

			if (expected > xfer->data_len) {
				expected = xfer->data_len;
			}
			if (expected > received) {
				hci_pio_do_trailing_rx(hci, pio, expected - received);
			} else if (received > expected) {
				hci_pio_move_extra_rx(hci, xfer, expected);
			}
			if (hci_pio_process_rx(hci, pio)) {
				pio->enabled_irqs &= ~STAT_RX_THLD;
			}
		}
		if (pio->curr_rx == xfer) {
			pio->curr_rx = pio->curr_rx->next_data;
			if (!pio->curr_rx) {
				pio->rx_tail = NULL;
			}
		} else if (pio->curr_tx == xfer) {
			pio->curr_tx = pio->curr_tx->next_data;
			if (!pio->curr_tx) {
				pio->tx_tail = NULL;
			}
		} else if (xfer->data_left != 0U) {
			LOG_DBG("%s PIO xfer has %u bytes left after response",
				hci->dev->name, xfer->data_left);
		}
		pio->curr_resp = xfer->next_resp;
		if (!pio->curr_resp) {
			pio->resp_tail = NULL;
		}
		xfer->next_resp = NULL;
		if (xfer->completion) {
			k_sem_give(xfer->completion);
		}
	}
	return !pio->curr_resp;
}

static void hci_pio_queue_resp(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	struct hci_xfer *xfer = pio->curr_xfer;

	if (!xfer || !hci_pio_xfer_has_response(xfer)) {
		return;
	}
	if (pio->curr_resp) {
		pio->resp_tail->next_resp = xfer;
		pio->resp_tail = xfer;
	} else {
		pio->curr_resp = xfer;
		pio->resp_tail = xfer;
		if (!hci_pio_process_resp(hci, pio)) {
			pio->enabled_irqs |= STAT_RESP_READY;
		}
	}
}

static bool hci_pio_process_cmd(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	while (pio->curr_xfer &&
	       (hci_pio_read(hci, PIO_INTR_STATUS) & STAT_CMD_QUEUE_READY) != 0U) {
		struct hci_xfer *xfer = pio->curr_xfer;

		hci_pio_queue_data(hci, pio);
		hci_pio_queue_resp(hci, pio);
		hci_pio_write_cmd(hci, xfer);
		pio->curr_xfer = xfer->next_xfer;
		xfer->next_xfer = NULL;
	}
	if (!pio->curr_xfer) {
		pio->xfer_tail = NULL;
	}
	return !pio->curr_xfer;
}

static void hci_pio_complete_xfer(struct hci_xfer *xfer)
{
	xfer->response = FIELD_PREP(RESP_ERR_FIELD, RESP_ERR_HC_TERMINATED);
	if (xfer->completion) {
		k_sem_give(xfer->completion);
	}
}

static void hci_pio_cancel_all(struct hci_pio_data *pio)
{
	struct hci_xfer *xfer;

	for (xfer = pio->curr_resp; xfer; xfer = xfer->next_resp) {
		hci_pio_complete_xfer(xfer);
	}
	for (xfer = pio->curr_xfer; xfer; xfer = xfer->next_xfer) {
		hci_pio_complete_xfer(xfer);
	}
	pio->curr_xfer = NULL;
	pio->xfer_tail = NULL;
	pio->curr_rx = NULL;
	pio->rx_tail = NULL;
	pio->curr_tx = NULL;
	pio->tx_tail = NULL;
	pio->curr_resp = NULL;
	pio->resp_tail = NULL;
}

static bool hci_pio_dequeue_xfer_common(struct i3c_hci *hci,
					struct hci_pio_data *pio,
					struct hci_xfer *xfer, int n)
{
	struct hci_xfer *p;
	struct hci_xfer **prev_next;

	ARG_UNUSED(hci);
	if (!xfer || n <= 0) {
		hci_pio_cancel_all(pio);
		return true;
	}
	for (p = pio->curr_resp; p; p = p->next_resp) {
		for (int i = 0; i < n; i++) {
			if (p == &xfer[i]) {
				goto pio_screwed;
			}
		}
	}
	for (p = pio->curr_rx; p; p = p->next_data) {
		for (int i = 0; i < n; i++) {
			if (p == &xfer[i]) {
				goto pio_screwed;
			}
		}
	}
	for (p = pio->curr_tx; p; p = p->next_data) {
		for (int i = 0; i < n; i++) {
			if (p == &xfer[i]) {
				goto pio_screwed;
			}
		}
	}
	prev_next = &pio->curr_xfer;
	for (p = pio->curr_xfer; p; p = p->next_xfer) {
		if (p == xfer) {
			*prev_next = xfer[n - 1].next_xfer;
			if (pio->xfer_tail == &xfer[n - 1]) {
				pio->xfer_tail = pio->curr_xfer;
				while (pio->xfer_tail &&
				       pio->xfer_tail->next_xfer) {
					pio->xfer_tail = pio->xfer_tail->next_xfer;
				}
			}
			return true;
		}
		prev_next = &p->next_xfer;
	}
	return false;
pio_screwed:
	hci_pio_cancel_all(pio);
	return true;
}

static void hci_pio_err(struct i3c_hci *hci, struct hci_pio_data *pio,
			uint32_t status)
{
	if ((hci_pio_read(hci, PIO_INTR_STATUS) & STAT_RESP_READY) != 0U) {
		uint32_t resp = hci_pio_read(hci, PIO_RESPONSE_QUEUE_PORT);

		LOG_ERR("%s orphan response %#x on PIO error", hci->dev->name, resp);
	}
	if ((status & STAT_PROG_ERRORS) != 0U) {
		if (hci->VENDOR_regs != 0U && hci->vendor &&
		    hci->vendor->pio_log_prog_error) {
			hci->vendor->pio_log_prog_error(hci,
							status & STAT_PROG_ERRORS);
		} else {
			uint32_t queue = hci_pio_read(hci, PIO_QUEUE_CUR_STATUS);
			uint32_t data = hci_pio_read(hci, PIO_DATA_BUFFER_CUR_STATUS);

			LOG_ERR("%s PIO prog error %#x C/R/I=%u/%u/%u TX/RX=%u/%u",
				hci->dev->name, (uint32_t)(status & STAT_PROG_ERRORS),
				(unsigned int)FIELD_GET(CUR_CMD_Q_EMPTY_LEVEL, queue),
				(unsigned int)FIELD_GET(CUR_RESP_Q_LEVEL, queue),
				(unsigned int)FIELD_GET(CUR_IBI_Q_LEVEL, queue),
				(unsigned int)FIELD_GET(CUR_TX_BUF_LVL, data),
				(unsigned int)FIELD_GET(CUR_RX_BUF_LVL, data));
		}
	}
	hci_pio_cancel_all(pio);
	mipi_i3c_hci_pio_reset(hci);
	mipi_i3c_hci_resume(hci);
}

static inline bool hci_pio_ibi_thld_status_only(struct i3c_hci *hci)
{
	return hci->vendor && hci->vendor->pio_ibi_thld_status_only &&
	       hci->vendor->pio_ibi_thld_status_only(hci);
}

static void hci_pio_set_ibi_thresh(struct i3c_hci *hci,
				   struct hci_pio_data *pio,
				   unsigned int thresh_val)
{
	uint32_t regval = pio->reg_queue_thresh;

	regval &= ~QUEUE_IBI_STATUS_THLD;
	regval |= FIELD_PREP(QUEUE_IBI_STATUS_THLD, thresh_val);
	/*
	 * On silicon whose STATUS_THLD reflects only the status FIFO (see
	 * vendor_ops::pio_ibi_thld_status_only), the IBI data threshold has
	 * to be lowered alongside the status threshold so STATUS_THLD still
	 * asserts for IBI payloads smaller than max_ibi_thresh; otherwise
	 * the data threshold stays at the init value and STATUS_THLD never
	 * asserts for small payloads, leaving IBI data stuck in FIFO and
	 * pending-read-notify timing out.
	 */
	if (hci_pio_ibi_thld_status_only(hci)) {
		regval &= ~QUEUE_IBI_DATA_THLD;
		regval |= FIELD_PREP(QUEUE_IBI_DATA_THLD, thresh_val);
	}
	if (regval != pio->reg_queue_thresh) {
		hci_pio_write(hci, PIO_QUEUE_THLD_CTRL, regval);
		pio->reg_queue_thresh = regval;
	}
}

static struct hci_pio_ibi_slot *
hci_pio_ibi_get_slot(struct hci_pio_dev_ibi_data *dev_ibi)
{
	struct hci_pio_ibi_slot *slot = dev_ibi->free_slots;

	if (slot) {
		dev_ibi->free_slots = slot->next;
		slot->next = NULL;
		slot->payload.payload_len = 0U;
	}
	return slot;
}

static void hci_pio_ibi_recycle_slot(struct hci_pio_dev_ibi_data *dev_ibi,
				     struct hci_pio_ibi_slot *slot)
{
	if (!dev_ibi || !slot) {
		return;
	}
	slot->target = NULL;
	slot->payload.payload_len = 0U;
	slot->next = dev_ibi->free_slots;
	dev_ibi->free_slots = slot;
}

static int hci_pio_submit_ibi_slot(struct i3c_hci *hci,
				   struct hci_pio_dev_ibi_data *dev_ibi,
				   struct hci_pio_ibi_slot *slot)
{
	int ret = -ENOTSUP;

	if (!slot) {
		return 0;
	}
#if defined(CONFIG_I3C_USE_IBI)
	if (!slot->target || !slot->target->ibi_cb) {
		ret = -ENODEV;
		goto out;
	}
#if defined(CONFIG_I3C_IBI_WORKQUEUE)
	ret = i3c_ibi_work_enqueue_target_irq(slot->target, slot->payload.payload,
					      slot->payload.payload_len);
#else
	ret = slot->target->ibi_cb(slot->target,
				   slot->payload.payload_len != 0U ? &slot->payload : NULL);
#endif
#else
	ARG_UNUSED(hci);
#endif
#if defined(CONFIG_I3C_USE_IBI)
out:
#endif
	if (ret != 0) {
		LOG_ERR("%s failed to submit IBI from 0x%02x: %d",
			hci->dev->name, slot->target ? slot->target->dynamic_addr : 0U, ret);
	}
	hci_pio_ibi_recycle_slot(dev_ibi, slot);
	return ret;
}

static bool hci_pio_get_ibi_segment(struct i3c_hci *hci,
				    struct hci_pio_data *pio)
{
	struct hci_pio_ibi_data *ibi = &pio->ibi;

	if (hci_pio_ibi_thld_status_only(hci)) {
		/*
		 * STATUS_THLD only tracks the status FIFO on this silicon, so
		 * polling it between data reads is wrong (the bit clears as
		 * soon as prep_new_ibi pops the status word while the IBI
		 * data FIFO still holds the payload). Drain the segment using
		 * the length advertised by the status word.
		 */
		while (ibi->seg_cnt >= 4U) {
			hci_pio_read_ibi_word(hci, &ibi->data_ptr, 4U);
			ibi->seg_cnt -= 4U;
		}
		if (ibi->seg_cnt != 0U) {
			unsigned int count = ibi->seg_cnt;

			hci_pio_read_ibi_word(hci, &ibi->data_ptr, count);
			ibi->seg_cnt = 0U;
		}
		return true;
	}

	/* Standard MIPI HCI path: poll STATUS_THLD between data reads. */
	while (ibi->seg_cnt >= 4U) {
		unsigned int nr_words = ibi->seg_cnt / 4U;
		unsigned int thresh_val = MIN(nr_words, pio->max_ibi_thresh);

		hci_pio_set_ibi_thresh(hci, pio, thresh_val);
		if ((hci_pio_read(hci, PIO_INTR_STATUS) & STAT_IBI_STATUS_THLD) == 0U) {
			return false;
		}
		nr_words = thresh_val;
		ibi->seg_cnt -= nr_words * 4U;
		while (nr_words-- != 0U) {
			hci_pio_read_ibi_word(hci, &ibi->data_ptr, 4U);
		}
	}
	if (ibi->seg_cnt != 0U) {
		unsigned int count = ibi->seg_cnt;

		hci_pio_set_ibi_thresh(hci, pio, 1U);
		if ((hci_pio_read(hci, PIO_INTR_STATUS) & STAT_IBI_STATUS_THLD) == 0U) {
			return false;
		}
		hci_pio_read_ibi_word(hci, &ibi->data_ptr, count);
		ibi->seg_cnt = 0U;
	}
	return true;
}

static void hci_pio_drain_ibi_segment(struct i3c_hci *hci,
				      struct hci_pio_data *pio)
{
	struct hci_pio_ibi_data *ibi = &pio->ibi;

	while (ibi->seg_cnt != 0U) {
		unsigned int count = MIN(ibi->seg_cnt, 4U);

		hci_pio_set_ibi_thresh(hci, pio, 1U);
		if ((hci_pio_read(hci, PIO_INTR_STATUS) & STAT_IBI_STATUS_THLD) == 0U) {
			return;
		}
		(void)hci_pio_read(hci, PIO_IBI_PORT);
		ibi->seg_cnt -= count;
	}
}

static void hci_pio_free_current_ibi_slot(struct hci_pio_data *pio)
{
	struct hci_pio_ibi_data *ibi = &pio->ibi;
	struct i3c_hci_dev_data *dev_data;

	if (!ibi->slot || !ibi->slot->target) {
		ibi->slot = NULL;
		return;
	}
	dev_data = ibi->slot->target->controller_priv;
	if (dev_data) {
		hci_pio_ibi_recycle_slot(dev_data->ibi_data, ibi->slot);
	}
	ibi->slot = NULL;
}

static bool hci_pio_submit_hotjoin(struct i3c_hci *hci)
{
#if defined(CONFIG_I3C_IBI_WORKQUEUE)
	return i3c_ibi_work_enqueue_hotjoin(hci->dev) == 0;
#else
	return k_work_submit(&hci->hj_work) >= 0;
#endif
}

static bool hci_pio_prep_new_ibi(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	struct hci_pio_ibi_data *ibi = &pio->ibi;
	struct i3c_hci_dev_data *dev_data;
	struct hci_pio_dev_ibi_data *dev_ibi;
	struct i3c_device_desc *target;
	uint32_t ibi_status = hci_pio_read(hci, PIO_IBI_PORT);
	unsigned int ibi_addr = FIELD_GET(IBI_TARGET_ADDR, ibi_status);
	bool ibi_rnw = FIELD_GET(IBI_TARGET_RNW, ibi_status) != 0U;

	LOG_DBG("%s IBI status=%#x", hci->dev->name, ibi_status);
	if (IBI_TYPE_HJ(ibi_addr, ibi_rnw)) {
		if (!hci_pio_submit_hotjoin(hci)) {
			LOG_ERR("%s failed to enqueue hot-join work", hci->dev->name);
		}
		return false;
	}
	if (IBI_TYPE_CR(ibi_addr, ibi_rnw)) {
		LOG_INF("%s controller-role request from 0x%02x", hci->dev->name, ibi_addr);
		return false;
	}
	ibi->addr = ibi_addr;
	ibi->last_seg = (ibi_status & IBI_LAST_STATUS) != 0U;
	ibi->seg_len = FIELD_GET(IBI_DATA_LENGTH, ibi_status);
	ibi->seg_cnt = ibi->seg_len;
	ibi->slot = NULL;
	ibi->data_ptr = NULL;
	if ((ibi_status & IBI_ERROR) != 0U) {
		LOG_ERR("%s IBI error from 0x%02x", hci->dev->name, ibi_addr);
		return ibi->seg_cnt != 0U;
	}
	target = i3c_dev_list_i3c_addr_find(&hci->common.attached_dev, (uint8_t)ibi_addr);
	if (!target) {
		LOG_ERR("%s IBI for unknown target 0x%02x", hci->dev->name, ibi_addr);
		return true;
	}
	dev_data = target->controller_priv;
	dev_ibi = dev_data ? dev_data->ibi_data : NULL;
	if (!dev_ibi) {
		LOG_ERR("%s IBI for target 0x%02x without setup", hci->dev->name, ibi_addr);
		return true;
	}
	ibi->max_len = dev_ibi->max_len;
	if (ibi->seg_len > ibi->max_len ||
	    ibi->seg_len > CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE) {
		LOG_ERR("%s IBI payload too big (%u > %u)", hci->dev->name,
			ibi->seg_len,
			MIN(ibi->max_len, (unsigned int)CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE));
		return true;
	}
	ibi->slot = hci_pio_ibi_get_slot(dev_ibi);
	if (!ibi->slot) {
		LOG_ERR("%s no free IBI slot for target 0x%02x", hci->dev->name, ibi_addr);
		return true;
	}
	ibi->slot->target = target;
	ibi->data_ptr = ibi->slot->payload.payload;
	return true;
}

static bool hci_pio_process_ibi(struct i3c_hci *hci, struct hci_pio_data *pio)
{
	struct hci_pio_ibi_data *ibi = &pio->ibi;

	if (!ibi->slot && ibi->seg_cnt == 0U &&
	    !hci_pio_prep_new_ibi(hci, pio)) {
		return false;
	}
	for (;;) {
		if (ibi->slot) {
			struct i3c_hci_dev_data *dev_data = ibi->slot->target->controller_priv;
			struct hci_pio_dev_ibi_data *dev_ibi =
				dev_data ? dev_data->ibi_data : NULL;
			if (!hci_pio_get_ibi_segment(hci, pio)) {
				return false;
			}
			ibi->slot->payload.payload_len += ibi->seg_len;
			if (ibi->last_seg) {
				(void)hci_pio_submit_ibi_slot(hci, dev_ibi, ibi->slot);
				ibi->slot = NULL;
				ibi->data_ptr = NULL;
				hci_pio_set_ibi_thresh(hci, pio, 1U);
				return true;
			}
		} else if (ibi->seg_cnt != 0U) {
			hci_pio_drain_ibi_segment(hci, pio);
			if (ibi->seg_cnt != 0U) {
				return false;
			}
			if (ibi->last_seg) {
				return true;
			}
		}
		hci_pio_set_ibi_thresh(hci, pio, 1U);
		if ((hci_pio_read(hci, PIO_INTR_STATUS) & STAT_IBI_STATUS_THLD) == 0U) {
			return false;
		}
		uint32_t ibi_status = hci_pio_read(hci, PIO_IBI_PORT);
		unsigned int ibi_addr = FIELD_GET(IBI_TARGET_ADDR, ibi_status);

		if (ibi_addr != ibi->addr) {
			LOG_ERR("%s IBI address changed from 0x%02x to 0x%02x",
				hci->dev->name, ibi->addr, ibi_addr);
			hci_pio_free_current_ibi_slot(pio);
		}
		ibi->last_seg = (ibi_status & IBI_LAST_STATUS) != 0U;
		ibi->seg_len = FIELD_GET(IBI_DATA_LENGTH, ibi_status);
		ibi->seg_cnt = ibi->seg_len;
		if (ibi->slot &&
		    (ibi->slot->payload.payload_len + ibi->seg_len > ibi->max_len ||
		     ibi->slot->payload.payload_len + ibi->seg_len >
			     CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE)) {
			LOG_ERR("%s IBI payload too big (%u > %u)",
				hci->dev->name,
				ibi->slot->payload.payload_len + ibi->seg_len,
				MIN(ibi->max_len,
				    (unsigned int)CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE));
			hci_pio_free_current_ibi_slot(pio);
		}
	}
}

static int hci_pio_init(struct i3c_hci *hci)
{
	struct hci_pio_data *pio;
	uint32_t val;
	uint32_t size_val;
	uint32_t rx_thresh;
	uint32_t tx_thresh;
	uint32_t ibi_val;

	if (hci->PIO_regs == 0U) {
		return -ENODEV;
	}
	pio = k_calloc(1, sizeof(*pio));
	if (!pio) {
		return -ENOMEM;
	}
	hci->io_data = pio;
	size_val = hci_pio_read(hci, PIO_QUEUE_SIZE);
	LOG_INF("%s PIO CMD/RESP FIFO = %u entries", hci->dev->name,
		(unsigned int)FIELD_GET(CR_QUEUE_SIZE, size_val));
	LOG_INF("%s PIO IBI FIFO = %u bytes", hci->dev->name,
		(unsigned int)(4U * FIELD_GET(IBI_STATUS_SIZE, size_val)));
	LOG_INF("%s PIO RX data FIFO = %u bytes", hci->dev->name,
		(unsigned int)(4U * (2U << FIELD_GET(RX_DATA_BUFFER_SIZE, size_val))));
	LOG_INF("%s PIO TX data FIFO = %u bytes", hci->dev->name,
		(unsigned int)(4U * (2U << FIELD_GET(TX_DATA_BUFFER_SIZE, size_val))));
	if (hci->is_target && !hci->target_rx.buf) {
		hci->target_rx.max_len = 4U * (2U << FIELD_GET(TX_DATA_BUFFER_SIZE, size_val));
		hci->target_rx.buf = k_malloc(hci->target_rx.max_len);
		if (!hci->target_rx.buf) {
			hci->io_data = NULL;
			k_free(pio);
			return -ENOMEM;
		}
	}
	rx_thresh = FIELD_GET(RX_DATA_BUFFER_SIZE, size_val);
	tx_thresh = FIELD_GET(TX_DATA_BUFFER_SIZE, size_val);
	if (hci->version_major == 1U) {
		if (rx_thresh != 0U) {
			rx_thresh--;
		}
		if (tx_thresh != 0U) {
			tx_thresh--;
		}
		pio->rx_thresh_size = 2U << rx_thresh;
		pio->tx_thresh_size = 2U << tx_thresh;
	} else {
		pio->rx_thresh_size = 1U << rx_thresh;
		pio->tx_thresh_size = 1U << tx_thresh;
	}
	val = FIELD_PREP(DATA_RX_BUF_THLD, rx_thresh) |
	      FIELD_PREP(DATA_TX_BUF_THLD, tx_thresh);
	hci_pio_write(hci, PIO_DATA_BUFFER_THLD_CTRL, val);
	ibi_val = FIELD_GET(IBI_STATUS_SIZE, size_val);
	pio->max_ibi_thresh = ibi_val / 2U;
	if (pio->max_ibi_thresh == 0U) {
		pio->max_ibi_thresh = 1U;
	}
	if (pio->max_ibi_thresh > 63U) {
		pio->max_ibi_thresh = 63U;
	}
	val = FIELD_PREP(QUEUE_IBI_STATUS_THLD, 1U) |
	      FIELD_PREP(QUEUE_IBI_DATA_THLD, pio->max_ibi_thresh) |
	      FIELD_PREP(QUEUE_RESP_BUF_THLD, 1U);
	val |= FIELD_PREP(QUEUE_CMD_EMPTY_BUF_THLD, 1U);
	hci_pio_write(hci, PIO_QUEUE_THLD_CTRL, val);
	pio->reg_queue_thresh = val;
	hci_pio_write(hci, PIO_INTR_SIGNAL_ENABLE, 0U);
	hci_pio_write(hci, PIO_INTR_STATUS, 0xffffffffU);
	hci_pio_write(hci, PIO_INTR_STATUS_ENABLE, 0xffffffffU);
	pio->enabled_irqs = STAT_ALL_ERRORS;
	if (hci->is_target) {
		pio->enabled_irqs |= STAT_RESP_READY;
	} else {
		mipi_i3c_hci_hj_ctrl(hci, false);
	}
	hci_reg_set(hci, HC_CONTROL, HC_CONTROL_PIO_MODE);
	hci_pio_set_signal(hci, pio);
	return 0;
}

static void hci_pio_cleanup(struct i3c_hci *hci)
{
	struct hci_pio_data *pio = hci->io_data;

	if (hci->PIO_regs != 0U) {
		hci_pio_write(hci, PIO_INTR_SIGNAL_ENABLE, 0U);
	}
	if (!pio) {
		return;
	}
	if (pio->curr_xfer || pio->curr_rx ||
	    pio->curr_tx || pio->curr_resp) {
		LOG_WRN("%s cleaning up PIO with pending transfers", hci->dev->name);
	}
	hci_pio_cancel_all(pio);
	hci->io_data = NULL;
	k_free(pio);
}

static int hci_pio_queue_xfer(struct i3c_hci *hci, struct hci_xfer *xfer, int n)
{
	struct hci_pio_data *pio = hci->io_data;
	k_spinlock_key_t key;

	if (!pio || !xfer || n <= 0) {
		return -EINVAL;
	}
	for (int i = 0; i < n; i++) {
		xfer[i].next_xfer = (i + 1 < n) ? &xfer[i + 1] : NULL;
		xfer[i].next_data = NULL;
		xfer[i].next_resp = NULL;
		xfer[i].data_left = xfer[i].data_len;
		xfer[i].data_word_before_partial = 0U;
	}
	key = k_spin_lock(&hci->lock);
	if (pio->curr_xfer) {
		pio->xfer_tail->next_xfer = xfer;
		pio->xfer_tail = &xfer[n - 1];
	} else {
		pio->curr_xfer = xfer;
		pio->xfer_tail = &xfer[n - 1];
		if (hci_pio_process_cmd(hci, pio)) {
			pio->enabled_irqs &= ~STAT_CMD_QUEUE_READY;
		} else {
			pio->enabled_irqs |= STAT_CMD_QUEUE_READY;
		}
		hci_pio_set_signal(hci, pio);
	}
	k_spin_unlock(&hci->lock, key);
	return 0;
}

static bool hci_pio_dequeue_xfer(struct i3c_hci *hci, struct hci_xfer *xfer, int n)
{
	struct hci_pio_data *pio = hci->io_data;
	k_spinlock_key_t key;
	bool ret;

	if (!pio) {
		return false;
	}
	key = k_spin_lock(&hci->lock);
	ret = hci_pio_dequeue_xfer_common(hci, pio, xfer, n);
	if (!pio->curr_xfer) {
		pio->enabled_irqs &= ~STAT_CMD_QUEUE_READY;
	}
	if (!pio->curr_rx) {
		pio->enabled_irqs &= ~STAT_RX_THLD;
	}
	if (!pio->curr_tx) {
		pio->enabled_irqs &= ~STAT_TX_THLD;
	}
	if (!pio->curr_resp && !hci->is_target) {
		pio->enabled_irqs &= ~STAT_RESP_READY;
	}
	hci_pio_set_signal(hci, pio);
	k_spin_unlock(&hci->lock, key);
	return ret;
}

static int hci_pio_request_ibi(struct i3c_hci *hci, struct i3c_device_desc *target,
			       const struct i3c_ibi *request)
{
	struct hci_pio_data *pio = hci->io_data;
	struct i3c_hci_dev_data *dev_data;
	struct hci_pio_dev_ibi_data *dev_ibi;
	unsigned int max_len;
	int dat_idx = -1;
	k_spinlock_key_t key;

	if (!pio || !target || !request) {
		return -EINVAL;
	}
	if (request->ibi_type != I3C_IBI_TARGET_INTR) {
		return -ENOTSUP;
	}
	max_len = request->payload_len;
	if (max_len > CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE) {
		return -EMSGSIZE;
	}
	dev_data = target->controller_priv;
	if (!dev_data) {
		return -ENODEV;
	}
	if (dev_data->ibi_data) {
		return -EALREADY;
	}
	dev_ibi = k_calloc(1, sizeof(*dev_ibi));
	if (!dev_ibi) {
		return -ENOMEM;
	}
	dev_ibi->num_slots = HCI_PIO_IBI_SLOT_COUNT;
	dev_ibi->max_len = max_len;
	dev_ibi->slots = k_calloc(dev_ibi->num_slots, sizeof(*dev_ibi->slots));
	if (!dev_ibi->slots) {
		k_free(dev_ibi);
		return -ENOMEM;
	}
	for (unsigned int i = 0; i < dev_ibi->num_slots; i++) {
		hci_pio_ibi_recycle_slot(dev_ibi, &dev_ibi->slots[i]);
	}
	dev_data->ibi_data = dev_ibi;
	if (hci->dat && hci->dat->get_index) {
		dat_idx = hci->dat->get_index(hci, target->dynamic_addr);
	}
	/*
	 * Set DAT_0_IBI_PAYLOAD whenever either the host explicitly asked
	 * for an IBI payload (max_len > 0) or the target advertised
	 * IBI_PAYLOAD_HAS_DATA_BYTE in its BCR. The latter matches what the
	 * AST10x0 i3c driver does (drivers/i3c/i3c_aspeed.c:708) and lets
	 * targets that emit MDB-only IBIs (e.g. pending-read-notify from
	 * i3c-target-mqueue) be accepted even when the host descriptor is
	 * left with max_ibi == 0 by a device driver that omits to set it.
	 */
	bool ibi_has_payload = (max_len != 0U) ||
			       (target->bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE);
	if (dat_idx >= 0 && hci->dat) {
		if (ibi_has_payload && hci->dat->set_flags) {
			hci->dat->set_flags(hci, (unsigned int)dat_idx,
					    DAT_0_IBI_PAYLOAD, 0U);
		} else if (hci->dat->clear_flags) {
			hci->dat->clear_flags(hci, (unsigned int)dat_idx,
					      DAT_0_IBI_PAYLOAD, 0U);
		}
		if (hci->dat->clear_flags) {
			hci->dat->clear_flags(hci, (unsigned int)dat_idx,
					      DAT_0_SIR_REJECT, 0U);
		}
	}
	if (max_len != 0U) {
		if (hci->vendor && hci->vendor->set_ibi_terminate_len) {
			hci->vendor->set_ibi_terminate_len(hci, (uint16_t)max_len);
		}
	}
	key = k_spin_lock(&hci->lock);
	pio->ibi_subscribers++;
	pio->enabled_irqs |= STAT_IBI_STATUS_THLD;
	hci_pio_set_signal(hci, pio);
	k_spin_unlock(&hci->lock, key);
	return 0;
}

static void hci_pio_free_ibi(struct i3c_hci *hci, struct i3c_device_desc *target)
{
	struct hci_pio_data *pio = hci->io_data;
	struct i3c_hci_dev_data *dev_data;
	struct hci_pio_dev_ibi_data *dev_ibi;
	int dat_idx = -1;
	k_spinlock_key_t key;

	if (!target) {
		return;
	}
	dev_data = target->controller_priv;
	if (!dev_data) {
		return;
	}
	dev_ibi = dev_data->ibi_data;
	dev_data->ibi_data = NULL;
	if (hci->dat && hci->dat->get_index) {
		dat_idx = hci->dat->get_index(hci, target->dynamic_addr);
	}
	if (dat_idx >= 0 && hci->dat) {
		if (hci->dat->set_flags) {
			hci->dat->set_flags(hci, (unsigned int)dat_idx,
					    DAT_0_SIR_REJECT, 0U);
		}
		if (hci->dat->clear_flags) {
			hci->dat->clear_flags(hci, (unsigned int)dat_idx,
					      DAT_0_IBI_PAYLOAD, 0U);
		}
	}
	if (pio) {
		key = k_spin_lock(&hci->lock);
		if (pio->ibi_subscribers > 0U) {
			pio->ibi_subscribers--;
		}
		if (pio->ibi_subscribers == 0U && !pio->hj_enabled) {
			pio->enabled_irqs &= ~STAT_IBI_STATUS_THLD;
			hci_pio_set_signal(hci, pio);
		}
		k_spin_unlock(&hci->lock, key);
	}
	if (dev_ibi) {
		k_free(dev_ibi->slots);
		k_free(dev_ibi);
	}
}

static int hci_pio_request_hj(struct i3c_hci *hci)
{
	struct hci_pio_data *pio = hci->io_data;
	k_spinlock_key_t key;

	if (!pio) {
		return -EINVAL;
	}
	key = k_spin_lock(&hci->lock);
	pio->hj_enabled = true;
	pio->enabled_irqs |= STAT_IBI_STATUS_THLD;
	hci_pio_set_signal(hci, pio);
	k_spin_unlock(&hci->lock, key);
	mipi_i3c_hci_hj_ctrl(hci, true);
	return 0;
}

static void hci_pio_free_hj(struct i3c_hci *hci)
{
	struct hci_pio_data *pio = hci->io_data;
	k_spinlock_key_t key;

	if (!pio) {
		return;
	}
	key = k_spin_lock(&hci->lock);
	pio->hj_enabled = false;
	pio->enabled_irqs &= ~STAT_IBI_STATUS_THLD;
	hci_pio_set_signal(hci, pio);
	k_spin_unlock(&hci->lock, key);
	mipi_i3c_hci_hj_ctrl(hci, false);
}

static void hci_pio_recycle_ibi_slot(struct i3c_hci *hci,
				     struct i3c_device_desc *target,
				     struct i3c_ibi_payload *payload)
{
	struct i3c_hci_dev_data *dev_data;
	struct hci_pio_dev_ibi_data *dev_ibi;
	struct hci_pio_ibi_slot *slot;

	ARG_UNUSED(hci);
	if (!target || !payload) {
		return;
	}
	dev_data = target->controller_priv;
	dev_ibi = dev_data ? dev_data->ibi_data : NULL;
	if (!dev_ibi) {
		return;
	}
	slot = CONTAINER_OF(payload, struct hci_pio_ibi_slot, payload);
	if (slot < dev_ibi->slots || slot >= &dev_ibi->slots[dev_ibi->num_slots]) {
		return;
	}
	hci_pio_ibi_recycle_slot(dev_ibi, slot);
}

static bool hci_pio_irq_handler(struct i3c_hci *hci)
{
	struct hci_pio_data *pio = hci->io_data;
	k_spinlock_key_t key;
	uint32_t status;

	if (!pio) {
		return false;
	}
	key = k_spin_lock(&hci->lock);
	status = hci_pio_read(hci, PIO_INTR_STATUS);
	LOG_DBG("%s PIO_INTR_STATUS %#x/%#x", hci->dev->name, status, pio->enabled_irqs);
	status &= pio->enabled_irqs | STAT_LATENCY_WARNINGS;
	if (status == 0U) {
		k_spin_unlock(&hci->lock, key);
		return false;
	}
	if ((status & STAT_IBI_STATUS_THLD) != 0U) {
		(void)hci_pio_process_ibi(hci, pio);
	}
	if ((status & STAT_RX_THLD) != 0U) {
		if (hci_pio_process_rx(hci, pio)) {
			pio->enabled_irqs &= ~STAT_RX_THLD;
		}
	}
	if ((status & STAT_TX_THLD) != 0U) {
		if (hci_pio_process_tx(hci, pio)) {
			pio->enabled_irqs &= ~STAT_TX_THLD;
		}
	}
	if ((status & STAT_RESP_READY) != 0U) {
		if (hci_pio_process_resp(hci, pio) && !hci->is_target) {
			pio->enabled_irqs &= ~STAT_RESP_READY;
		}
	}
	if ((status & STAT_LATENCY_WARNINGS) != 0U) {
		hci_pio_write(hci, PIO_INTR_STATUS, status & STAT_LATENCY_WARNINGS);
		LOG_WRN("%s PIO warning condition %#x",
			hci->dev->name, (uint32_t)(status & STAT_LATENCY_WARNINGS));
	}
	if ((status & STAT_ALL_ERRORS) != 0U) {
		hci_pio_write(hci, PIO_INTR_STATUS, status & STAT_ALL_ERRORS);
		hci_pio_err(hci, pio, status & STAT_ALL_ERRORS);
	}
	if ((status & STAT_CMD_QUEUE_READY) != 0U) {
		if (hci_pio_process_cmd(hci, pio)) {
			pio->enabled_irqs &= ~STAT_CMD_QUEUE_READY;
		}
	}
	hci_pio_set_signal(hci, pio);
	k_spin_unlock(&hci->lock, key);
	return true;
}

const struct hci_io_ops mipi_i3c_hci_pio = {
	.irq_handler = hci_pio_irq_handler,
	.queue_xfer = hci_pio_queue_xfer,
	.dequeue_xfer = hci_pio_dequeue_xfer,
	.request_ibi = hci_pio_request_ibi,
	.free_ibi = hci_pio_free_ibi,
	.request_hj = hci_pio_request_hj,
	.free_hj = hci_pio_free_hj,
	.recycle_ibi_slot = hci_pio_recycle_ibi_slot,
	.init = hci_pio_init,
	.cleanup = hci_pio_cleanup,
};
