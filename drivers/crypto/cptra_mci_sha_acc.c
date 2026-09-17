/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SHA_ACC driver: Caliptra DMA-path hashing (MC_SHA_ACC_INIT/UPDATE/FINAL).
 * Caliptra reads source data directly over AXI — no data is copied through
 * the mailbox buffer.  The caller passes the AXI address of the source region
 * as pkt->in_buf; pkt->in_len is the byte count of that region.
 *
 * Session lifecycle:
 *   hash_begin_session()  – records the algorithm; no mailbox call yet.
 *   hash_update(pkt)      – first call → SHA_ACC_INIT; subsequent calls → SHA_ACC_UPDATE.
 *   hash_compute(pkt)     – like hash_update but also issues SHA_ACC_FINAL and
 *                           writes the digest to pkt->out_buf.
 *   hash_free_session()   – marks the driver available.
 */

#define DT_DRV_COMPAT aspeed_cptra_mci_sha_acc

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>

LOG_MODULE_REGISTER(cptra_mci_sha_acc, CONFIG_LOG_DEFAULT_LEVEL);

struct cptra_mci_sha_acc_drv_state {
	bool in_use;
	bool initialized;
	uint32_t mci_algo;
	uint32_t dgst_len;
};

#define DEV_DATA(dev) \
	((struct cptra_mci_sha_acc_drv_state *)(dev)->data)

/* ---- mailbox helpers ---- */

static int sha_acc_do_init(uint32_t algo, uint32_t addr_lo, uint32_t addr_hi,
			   uint32_t data_size)
{
	struct cptra_mci_sha_acc_init_req req = {
		.hash_algorithm = algo,
		.data_size = data_size,
		.addr_lo = addr_lo,
		.addr_hi = addr_hi,
	};
	struct cptra_mci_mbox_resp_hdr resp = { 0 };
	uint32_t resp_len;
	int ret;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_SHA_ACC_INIT,
						  (uint8_t *)&req + sizeof(req.hdr),
						  sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_SHA_ACC_INIT,
				     &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret)
		LOG_ERR("SHA_ACC_INIT failed: %d", ret);
	return ret;
}

static int sha_acc_do_update(uint32_t addr_lo, uint32_t addr_hi, uint32_t data_size)
{
	struct cptra_mci_sha_acc_data_req req = {
		.data_size = data_size,
		.addr_lo = addr_lo,
		.addr_hi = addr_hi,
	};
	struct cptra_mci_mbox_resp_hdr resp = { 0 };
	uint32_t resp_len;
	int ret;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_SHA_ACC_UPDATE,
						  (uint8_t *)&req + sizeof(req.hdr),
						  sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_SHA_ACC_UPDATE,
				     &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret)
		LOG_ERR("SHA_ACC_UPDATE failed: %d", ret);
	return ret;
}

static int sha_acc_do_final(uint8_t *digest, uint32_t dgst_len)
{
	struct cptra_mci_sha_acc_data_req req = {
		.data_size = 0,
		.addr_lo = 0,
		.addr_hi = 0,
	};
	struct cptra_mci_sha_final_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_SHA_ACC_FINAL,
						  (uint8_t *)&req + sizeof(req.hdr),
						  sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_SHA_ACC_FINAL,
				     &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("SHA_ACC_FINAL failed: %d", ret);
		return ret;
	}

	memcpy(digest, resp.hash, MIN(dgst_len, sizeof(resp.hash)));
	return 0;
}

/* ---- Zephyr crypto API implementation ---- */

static int cptra_mci_sha_acc_compute(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	struct cptra_mci_sha_acc_drv_state *state = DEV_DATA(ctx->device);
	int ret;

	if (pkt->in_len > 0) {
		uint32_t axi_lo = (uint32_t)(uintptr_t)pkt->in_buf;
		uint32_t axi_hi = (uint32_t)((uint64_t)(uintptr_t)pkt->in_buf >> 32);

		if (!state->initialized) {
			ret = sha_acc_do_init(state->mci_algo, axi_lo, axi_hi,
					      (uint32_t)pkt->in_len);
			if (ret)
				return ret;
			state->initialized = true;
		} else {
			ret = sha_acc_do_update(axi_lo, axi_hi, (uint32_t)pkt->in_len);
			if (ret)
				return ret;
		}
	}

	if (!finish)
		return 0;

	if (!state->initialized) {
		LOG_ERR("SHA_ACC: no data added before final");
		return -EINVAL;
	}

	ret = sha_acc_do_final(pkt->out_buf, state->dgst_len);
	state->initialized = false;
	return ret;
}

static int cptra_mci_sha_acc_session_setup(const struct device *dev, struct hash_ctx *ctx,
					   enum hash_algo algo)
{
	struct cptra_mci_sha_acc_drv_state *state = DEV_DATA(dev);

	if (state->in_use) {
		LOG_ERR("SHA_ACC peripheral in use");
		return -EBUSY;
	}

	switch (algo) {
	case CRYPTO_HASH_ALGO_SHA384:
		state->mci_algo = CPTRA_MCI_SHA_ALGO_SHA384;
		state->dgst_len = CPTRA_MCI_SHA384_DIGEST_SIZE;
		break;
	case CRYPTO_HASH_ALGO_SHA512:
		state->mci_algo = CPTRA_MCI_SHA_ALGO_SHA512;
		state->dgst_len = CPTRA_MCI_SHA512_DIGEST_SIZE;
		break;
	default:
		LOG_ERR("SHA_ACC unsupported algo: %d", algo);
		return -EINVAL;
	}

	state->initialized = false;
	state->in_use = true;
	ctx->device = dev;
	ctx->hash_hndlr = cptra_mci_sha_acc_compute;

	return 0;
}

static int cptra_mci_sha_acc_session_free(const struct device *dev, struct hash_ctx *ctx)
{
	struct cptra_mci_sha_acc_drv_state *state = DEV_DATA(dev);

	ARG_UNUSED(ctx);
	state->in_use = false;
	state->initialized = false;

	return 0;
}

static int cptra_mci_sha_acc_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS;
}

static struct crypto_driver_api cptra_mci_sha_acc_funcs = {
	.hash_begin_session = cptra_mci_sha_acc_session_setup,
	.hash_free_session = cptra_mci_sha_acc_session_free,
	.query_hw_caps = cptra_mci_sha_acc_query_hw_caps,
};

static struct cptra_mci_sha_acc_drv_state cptra_mci_sha_acc_state;

#define ASPEED_CPTRA_MCI_SHA_ACC_INIT(inst)					\
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL,					\
			      &cptra_mci_sha_acc_state, NULL,			\
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,		\
			      &cptra_mci_sha_acc_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_MCI_SHA_ACC_INIT)
