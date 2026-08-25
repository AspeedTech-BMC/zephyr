/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_mci_sha

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>

LOG_MODULE_REGISTER(cptra_mci_sha, CONFIG_LOG_DEFAULT_LEVEL);

#define ASPEED_HASH_CAPS_SUPPORT	(CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

struct cptra_mci_sha_ctx {
	enum hash_algo algo;
	uint32_t dgst_len;
	uint8_t context[CPTRA_MCI_SHA_CONTEXT_SIZE];
};

struct cptra_mci_sha_drv_state {
	struct cptra_mci_sha_ctx data;
	bool in_use;
};

#define DEV_DATA(dev)				\
	((struct cptra_mci_sha_drv_state *)	\
	(dev)->data)

static int cptra_mci_sha_do_init(struct cptra_mci_sha_ctx *ctx, uint32_t algo)
{
	struct cptra_mci_sha_init_hdr req = {
		.hash_algorithm = algo,
		.input_size = 0,
	};
	struct cptra_mci_sha_ctx_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_SHA_INIT,
						 (uint8_t *)&req + sizeof(req.hdr),
						 sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_SHA_INIT, &req, sizeof(req),
				    &resp, sizeof(resp), &resp_len);
	if (ret)
		return ret;

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_SHA_INIT response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(ctx->context, resp.context, sizeof(ctx->context));

	return 0;
}

static int cptra_mci_sha_do_update(struct cptra_mci_sha_ctx *ctx, const uint8_t *data,
				   uint32_t len)
{
	struct cptra_mci_sha_data_hdr req = {
		.input_size = len,
	};
	struct cptra_mci_sha_ctx_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	memcpy(req.context, ctx->context, sizeof(req.context));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_SHA_UPDATE,
				       (uint8_t *)&req + sizeof(req.hdr),
				       sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, data, len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_SHA_UPDATE, &req, sizeof(req),
				       data, len, &resp, sizeof(resp), &resp_len);
	if (ret)
		return ret;

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_SHA_UPDATE response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(ctx->context, resp.context, sizeof(ctx->context));

	return 0;
}

static int cptra_mci_sha_do_final(struct cptra_mci_sha_ctx *ctx, const uint8_t *data,
				  uint32_t len, uint8_t *digest)
{
	struct cptra_mci_sha_data_hdr req = {
		.input_size = len,
	};
	struct cptra_mci_sha_final_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	memcpy(req.context, ctx->context, sizeof(req.context));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_SHA_FINAL,
				       (uint8_t *)&req + sizeof(req.hdr),
				       sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, data, len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_SHA_FINAL, &req, sizeof(req),
				       data, len, &resp, sizeof(resp), &resp_len);
	if (ret)
		return ret;

	if (resp_len < sizeof(resp.hdr) + MIN(ctx->dgst_len, sizeof(resp.hash))) {
		LOG_ERR("MC_SHA_FINAL response shorter than the expected digest");
		return -EIO;
	}

	memcpy(digest, resp.hash, MIN(ctx->dgst_len, sizeof(resp.hash)));

	return 0;
}

static int cptra_mci_sha_compute(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	struct cptra_mci_sha_drv_state *state = DEV_DATA(ctx->device);
	const uint8_t *p = pkt->in_buf;
	size_t remaining = pkt->in_len;
	uint32_t chunk;
	int ret;

	do {
		chunk = MIN(remaining, CPTRA_MCI_MBOX_MAX_INPUT_SIZE);

		if (finish && chunk == remaining)
			ret = cptra_mci_sha_do_final(&state->data, p, chunk, pkt->out_buf);
		else
			ret = cptra_mci_sha_do_update(&state->data, p, chunk);

		if (ret)
			return ret;

		p += chunk;
		remaining -= chunk;
	} while (remaining > 0);

	return 0;
}

static int cptra_mci_sha_session_setup(const struct device *dev, struct hash_ctx *ctx,
				       enum hash_algo algo)
{
	struct cptra_mci_sha_drv_state *state = DEV_DATA(dev);
	uint32_t mci_algo;
	uint32_t dgst_len;
	int ret;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	switch (algo) {
	case CRYPTO_HASH_ALGO_SHA384:
		mci_algo = CPTRA_MCI_SHA_ALGO_SHA384;
		dgst_len = CPTRA_MCI_SHA384_DIGEST_SIZE;
		break;
	case CRYPTO_HASH_ALGO_SHA512:
		mci_algo = CPTRA_MCI_SHA_ALGO_SHA512;
		dgst_len = CPTRA_MCI_SHA512_DIGEST_SIZE;
		break;
	default:
		LOG_ERR("Unsupported algo: %d", algo);
		return -EINVAL;
	}

	ret = cptra_mci_sha_do_init(&state->data, mci_algo);
	if (ret)
		return ret;

	state->in_use = true;
	state->data.algo = algo;
	state->data.dgst_len = dgst_len;

	ctx->device = dev;
	ctx->hash_hndlr = cptra_mci_sha_compute;

	return 0;
}

static int cptra_mci_sha_session_free(const struct device *dev, struct hash_ctx *ctx)
{
	struct cptra_mci_sha_drv_state *state = DEV_DATA(dev);

	ARG_UNUSED(ctx);
	state->in_use = false;

	return 0;
}

static int cptra_mci_sha_query_hw_caps(const struct device *dev)
{
	return ASPEED_HASH_CAPS_SUPPORT;
}

static struct crypto_driver_api cptra_mci_sha_funcs = {
	.hash_begin_session = cptra_mci_sha_session_setup,
	.hash_free_session = cptra_mci_sha_session_free,
	.query_hw_caps = cptra_mci_sha_query_hw_caps,
};

static struct cptra_mci_sha_drv_state cptra_mci_sha_state;

#define ASPEED_CPTRA_MCI_SHA_INIT(inst)					\
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL,				\
		      &cptra_mci_sha_state, NULL,				\
		      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,		\
		      &cptra_mci_sha_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_MCI_SHA_INIT)
