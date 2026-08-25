/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_mci_mbox

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>

LOG_MODULE_REGISTER(cptra_mci_mbox, CONFIG_LOG_DEFAULT_LEVEL);

/* #define DEBUG */
#ifdef DEBUG
#define MCI_MBOX_DBG(fmt, ...) \
	printk(fmt, ##__VA_ARGS__)
#else
#define MCI_MBOX_DBG(fmt, ...)
#endif

/* Device config */
struct cptra_mci_mbox_config {
	uintptr_t base;		/* MCI mbox paged window base (64KB) */
	uintptr_t scu_base;	/* SCU1 base address */
};

static uint32_t cptra_mci_mbox_base;
static uint32_t cptra_mci_mbox_scu_base;
static struct k_mutex cptra_mci_mbox_mutex;

#define DEV_CFG(dev)				\
	((struct cptra_mci_mbox_config *)	\
	(dev)->config)

/*
 * The mailbox CSR block and its data SRAM are aliased onto the same 64KB
 * local window; only one of them is reachable at a time depending on which
 * page is currently selected.
 */
static int cptra_mci_mbox_select_page(uint32_t page)
{
	sys_write32(page, cptra_mci_mbox_scu_base + SCU1_CPTRA_MCI_WIN);

	if (sys_read32(cptra_mci_mbox_scu_base + SCU1_CPTRA_MCI_WIN) != page) {
		LOG_ERR("Failed to select MCI mbox page 0x%x", page);
		return -EIO;
	}

	return 0;
}

static void cptra_mci_mbox_sram_write(const void *data, uint32_t len, uint32_t offset)
{
	const uint8_t *p8 = data;
	uint32_t word;
	uint32_t i;

	for (i = 0; (i + sizeof(uint32_t)) <= len; i += sizeof(uint32_t)) {
		memcpy(&word, p8 + i, sizeof(word));
		sys_write32(word, cptra_mci_mbox_base + offset + i);
	}

	if (i < len) {
		word = 0;
		memcpy(&word, p8 + i, len - i);
		sys_write32(word, cptra_mci_mbox_base + offset + i);
	}
}

static void cptra_mci_mbox_sram_read(void *data, uint32_t len)
{
	uint8_t *p8 = data;
	uint32_t word;
	uint32_t i;

	for (i = 0; (i + sizeof(uint32_t)) <= len; i += sizeof(uint32_t)) {
		word = sys_read32(cptra_mci_mbox_base + i);
		memcpy(p8 + i, &word, sizeof(word));
	}

	if (i < len) {
		word = sys_read32(cptra_mci_mbox_base + i);
		memcpy(p8 + i, &word, len - i);
	}
}

/*
 * Extends an in-progress checksum accumulator with more bytes. Lets callers
 * whose request is split across multiple buffers (e.g. a small fixed header
 * plus a separate large payload passed to cptra_mci_mbox_execute_sg()) fold
 * each piece in without first copying everything into one contiguous buffer.
 */
uint32_t cptra_mci_mbox_checksum_ext(uint32_t checksum, const void *data, uint32_t len)
{
	const uint8_t *p8 = data;
	uint32_t i;

	for (i = 0; i < len; i++)
		checksum -= p8[i];

	return checksum;
}

uint32_t cptra_mci_mbox_checksum(uint32_t cmd, const void *data, uint32_t len)
{
	uint32_t checksum = cptra_mci_mbox_checksum_ext(0, &cmd, sizeof(cmd));

	return cptra_mci_mbox_checksum_ext(checksum, data, len);
}

int cptra_mci_mbox_lock(void)
{
	int ret;

	ret = cptra_mci_mbox_select_page(CPTRA_MCI_MBOX_CSR_PAGE);
	if (ret)
		return ret;

	/*
	 * LOCK reads 0 (free) and atomically latches to locked as a side
	 * effect of the read. A second read confirming 1 is required to know
	 * the lock actually latched, rather than trusting the first read.
	 */
	if (sys_read32(cptra_mci_mbox_base + CPTRA_MCI_MBOX_LOCK))
		return -EBUSY;

	if (!sys_read32(cptra_mci_mbox_base + CPTRA_MCI_MBOX_LOCK)) {
		/*
		 * The first read above already latched the lock as a side
		 * effect, regardless of what this confirmation read reports.
		 * Release it here so a glitched confirmation read doesn't
		 * wedge the mailbox for every later caller.
		 */
		cptra_mci_mbox_unlock();
		return -EIO;
	}

	return 0;
}

int cptra_mci_mbox_unlock(void)
{
	int ret;

	ret = cptra_mci_mbox_select_page(CPTRA_MCI_MBOX_CSR_PAGE);
	if (ret)
		return ret;

	sys_write32(0x0, cptra_mci_mbox_base + CPTRA_MCI_MBOX_EXECUTE);

	return 0;
}

/*
 * Some CM_* wrappers keep their request and/or response structs in
 * function-local `static` storage instead of on the stack (several are too
 * large -- up to ~4.7KB for an ML-DSA signature -- to put on a shell/app
 * thread's stack). cptra_mci_mbox_execute()/execute_sg() only hold
 * cptra_mci_mbox_mutex around the hardware transaction itself, which is too
 * narrow to protect those statics: a wrapper fills its static req before
 * calling execute(), and reads its static resp after execute() returns --
 * both outside the hardware critical section. Two threads calling the same
 * wrapper concurrently can then interleave writes into the same static req,
 * or have one thread's execute() overwrite a static resp out from under
 * another thread still reading it. Exposing the same mutex here lets those
 * wrappers hold it across their *entire* body (first touch of the static
 * struct through the last read of it), not just the hardware step -- safe
 * to nest with execute()'s own internal lock/unlock since k_mutex allows the
 * owning thread to re-lock it.
 */
void cptra_mci_mbox_txn_begin(void)
{
	k_mutex_lock(&cptra_mci_mbox_mutex, K_FOREVER);
}

void cptra_mci_mbox_txn_end(void)
{
	k_mutex_unlock(&cptra_mci_mbox_mutex);
}

uint32_t cptra_mci_mbox_status(void)
{
	if (cptra_mci_mbox_select_page(CPTRA_MCI_MBOX_CSR_PAGE))
		return CPTRA_MCI_MBSTS_CMD_FAILURE;

	return FIELD_GET(CPTRA_MCI_MBOX_CMD_STATUS_PS,
			 sys_read32(cptra_mci_mbox_base + CPTRA_MCI_MBOX_CMD_STATUS));
}

struct cptra_mci_mbox_iov {
	const void *base;
	uint32_t len;
};

/*
 * Shared implementation. req is scattered across up to two buffers (a small
 * fixed-size header struct plus a separate, possibly large, payload buffer)
 * so callers never need to memcpy a large payload into one contiguous
 * request struct just to hand it to this function.
 */
static int cptra_mci_mbox_execute_iov(uint32_t cmd, const struct cptra_mci_mbox_iov *iov,
				      int iovcnt, void *resp, uint32_t resp_buf_len,
				      uint32_t *resp_len)
{
	uint32_t sts, dlen, req_len, off;
	int64_t deadline;
	int ret, i;

	req_len = 0;
	for (i = 0; i < iovcnt; i++)
		req_len += iov[i].len;

	MCI_MBOX_DBG("cptra_mci_mbox_execute: cmd=0x%x req_len=0x%x resp_buf_len=0x%x\n",
		     cmd, req_len, resp_buf_len);

	if (req_len > CPTRA_MCI_MBOX_SRAM_SIZE)
		return -EINVAL;

	k_mutex_lock(&cptra_mci_mbox_mutex, K_FOREVER);

	deadline = k_uptime_get() + CPTRA_MCI_MBOX_LOCK_TIMEOUT_MS;

	while ((ret = cptra_mci_mbox_lock()) == -EBUSY) {
		if (k_uptime_get() > deadline) {
			LOG_ERR("Timed out waiting for MCI mbox lock");
			ret = -ETIMEDOUT;
			break;
		}
	}

	if (ret) {
		k_mutex_unlock(&cptra_mci_mbox_mutex);
		return ret;
	}

	sys_write32(cmd, cptra_mci_mbox_base + CPTRA_MCI_MBOX_CMD);
	sys_write32(req_len, cptra_mci_mbox_base + CPTRA_MCI_MBOX_DLEN);

	ret = cptra_mci_mbox_select_page(CPTRA_MCI_MBOX_SRAM_PAGE);
	if (ret)
		goto unlock;

	off = 0;
	for (i = 0; i < iovcnt; i++) {
		cptra_mci_mbox_sram_write(iov[i].base, iov[i].len, off);
		off += iov[i].len;
	}

	ret = cptra_mci_mbox_select_page(CPTRA_MCI_MBOX_CSR_PAGE);
	if (ret)
		goto unlock;

	sys_write32(0x1, cptra_mci_mbox_base + CPTRA_MCI_MBOX_EXECUTE);

	do {
		sts = FIELD_GET(CPTRA_MCI_MBOX_CMD_STATUS_PS,
				sys_read32(cptra_mci_mbox_base + CPTRA_MCI_MBOX_CMD_STATUS));
	} while (sts == CPTRA_MCI_MBSTS_CMD_BUSY);

	MCI_MBOX_DBG("cptra_mci_mbox_execute: cmd_status=0x%x\n", sts);

	if (sts == CPTRA_MCI_MBSTS_CMD_FAILURE) {
		LOG_ERR("MCI mbox cmd 0x%x failed", cmd);
		ret = -EIO;
		goto unlock;
	}

	dlen = sys_read32(cptra_mci_mbox_base + CPTRA_MCI_MBOX_DLEN);

	MCI_MBOX_DBG("cptra_mci_mbox_execute: dlen=0x%x\n", dlen);

	if (dlen > CPTRA_MCI_MBOX_SRAM_SIZE) {
		LOG_ERR("MCI mbox reported invalid dlen 0x%x", dlen);
		ret = -EIO;
		goto unlock;
	}

	if (dlen > resp_buf_len) {
		LOG_ERR("MCI mbox response 0x%x exceeds buffer 0x%x", dlen, resp_buf_len);
		ret = -ENOSPC;
		goto unlock;
	}

	ret = cptra_mci_mbox_select_page(CPTRA_MCI_MBOX_SRAM_PAGE);
	if (ret)
		goto unlock;

	cptra_mci_mbox_sram_read(resp, dlen);

	if (resp_len)
		*resp_len = dlen;

	ret = 0;

unlock:
	cptra_mci_mbox_select_page(CPTRA_MCI_MBOX_CSR_PAGE);
	cptra_mci_mbox_unlock();
	k_mutex_unlock(&cptra_mci_mbox_mutex);

	MCI_MBOX_DBG("cptra_mci_mbox_execute: ret=%d resp_len=0x%x\n",
		     ret, resp_len ? *resp_len : 0);

	return ret;
}

int cptra_mci_mbox_execute(uint32_t cmd, const void *req, uint32_t req_len,
			   void *resp, uint32_t resp_buf_len, uint32_t *resp_len)
{
	struct cptra_mci_mbox_iov iov = {
		.base = req,
		.len = req_len,
	};

	return cptra_mci_mbox_execute_iov(cmd, &iov, 1, resp, resp_buf_len, resp_len);
}

int cptra_mci_mbox_execute_sg(uint32_t cmd, const void *hdr, uint32_t hdr_len,
			      const void *data, uint32_t data_len,
			      void *resp, uint32_t resp_buf_len, uint32_t *resp_len)
{
	struct cptra_mci_mbox_iov iov[2] = {
		{ .base = hdr, .len = hdr_len },
		{ .base = data, .len = data_len },
	};

	return cptra_mci_mbox_execute_iov(cmd, iov, 2, resp, resp_buf_len, resp_len);
}

static int cptra_mci_mbox_init(const struct device *dev)
{
	struct cptra_mci_mbox_config *cfg = DEV_CFG(dev);

	cptra_mci_mbox_base = (uint32_t)cfg->base;
	cptra_mci_mbox_scu_base = (uint32_t)cfg->scu_base;

	k_mutex_init(&cptra_mci_mbox_mutex);

	LOG_DBG("0x%x: Initialized", (uint32_t)cfg->base);

	return 0;
}

static const struct cptra_mci_mbox_config cptra_mci_mbox_config = {
	.base = DT_REG_ADDR(DT_DRV_INST(0)),
	.scu_base = DT_REG_ADDR(DT_INST_PHANDLE(0, aspeed_scu)),
};

DEVICE_DT_INST_DEFINE(0, cptra_mci_mbox_init, NULL, NULL,
		      &cptra_mci_mbox_config,
		      POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
