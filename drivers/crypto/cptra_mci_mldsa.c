/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_mci_mldsa

#include <string.h>
#include <zephyr/crypto/mldsa.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>

LOG_MODULE_REGISTER(cptra_mci_mldsa, CONFIG_LOG_DEFAULT_LEVEL);

struct cptra_mci_mldsa_drv_state {
	uint8_t pub_key[CPTRA_MCI_MLDSA87_PUBKEY_SIZE];
	bool in_use;
};

#define DEV_DATA(dev)				\
	((struct cptra_mci_mldsa_drv_state *)	\
	(dev)->data)

static int cptra_mci_mldsa_op_sign(struct mldsa_ctx *ctx, struct mldsa_pkt *pkt)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(pkt);

	/*
	 * MC_MLDSA87_SIG_VERIFY (what this driver talks to) has no signing
	 * counterpart for a raw public key -- Caliptra-SS's ML-DSA-87
	 * signing is only exposed through the Cmk-managed CM_MLDSA_CMK_SIGN
	 * command, which needs an opaque Cmk handle this raw-pubkey session
	 * has no way to carry.
	 */
	return -ENOSYS;
}

static int cptra_mci_mldsa_op_verify(struct mldsa_ctx *ctx, struct mldsa_pkt *pkt)
{
	struct cptra_mci_mldsa_drv_state *state = DEV_DATA(ctx->device);
	/*
	 * pub_key (2592 bytes) + signature (4628 bytes) make this request
	 * 7228 bytes -- kept off the stack. req is static (shared across
	 * calls), so the lock has to cover everything from the first write
	 * below through the execute() call -- see
	 * cptra_mci_mbox_txn_begin()'s comment.
	 */
	static struct cptra_mci_mldsa87_sig_verify_hdr req;
	struct cptra_mci_mldsa87_sig_verify_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (pkt->m_len < 0 || (size_t)pkt->m_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE) {
		LOG_ERR("Unsupported message length %d", pkt->m_len);
		return -EINVAL;
	}

	cptra_mci_mbox_txn_begin();

	memcpy(req.pub_key, state->pub_key, sizeof(req.pub_key));
	memcpy(req.signature, pkt->sig, sizeof(req.signature));
	req.message_size = pkt->m_len;

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_MLDSA87_SIG_VERIFY,
				       &req.pub_key, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, pkt->m, pkt->m_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_MLDSA87_SIG_VERIFY, &req, sizeof(req),
					pkt->m, pkt->m_len, &resp, sizeof(resp), &resp_len);

	cptra_mci_mbox_txn_end();

	if (ret) {
		LOG_ERR("MC_MLDSA87_SIG_VERIFY failed: %d", ret);
		return ret;
	}

	return 0;
}

static int aspeed_mci_mldsa_session_setup(const struct device *dev, struct mldsa_ctx *ctx,
					  struct mldsa_pub_key *key)
{
	struct cptra_mci_mldsa_drv_state *state = DEV_DATA(dev);

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	memcpy(state->pub_key, key->key, sizeof(state->pub_key));

	state->in_use = true;

	ctx->ops.sign = cptra_mci_mldsa_op_sign;
	ctx->ops.verify = cptra_mci_mldsa_op_verify;

	return 0;
}

static int aspeed_mci_mldsa_session_free(const struct device *dev, struct mldsa_ctx *ctx)
{
	struct cptra_mci_mldsa_drv_state *state = DEV_DATA(dev);

	ARG_UNUSED(ctx);
	state->in_use = false;

	return 0;
}

static int aspeed_mci_mldsa_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static struct mldsa_driver_api cptra_mci_mldsa_funcs = {
	.begin_session = aspeed_mci_mldsa_session_setup,
	.free_session = aspeed_mci_mldsa_session_free,
	.query_hw_caps = aspeed_mci_mldsa_query_hw_caps,
};

static struct cptra_mci_mldsa_drv_state cptra_mci_mldsa_state;

#define ASPEED_CPTRA_MCI_MLDSA_INIT(inst)					\
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL,				\
			      &cptra_mci_mldsa_state, NULL,			\
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,	\
			      &cptra_mci_mldsa_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_MCI_MLDSA_INIT)
