/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_mci_ecdsa

#include <string.h>
#include <zephyr/crypto/ecdsa.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>

LOG_MODULE_REGISTER(cptra_mci_ecdsa, CONFIG_LOG_DEFAULT_LEVEL);

struct cptra_mci_ecdsa_drv_state {
	uint8_t qx[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t qy[CPTRA_MCI_ECC384_SCALAR_SIZE];
	bool in_use;
};

#define DEV_DATA(dev)				\
	((struct cptra_mci_ecdsa_drv_state *)	\
	(dev)->data)

static int cptra_mci_ecdsa_op_sign(struct ecdsa_ctx *ctx, struct ecdsa_pkt *pkt)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(pkt);

	/*
	 * MC_ECDSA384_SIG_VERIFY (what this driver talks to) has no signing
	 * counterpart for a raw (qx, qy) key pair -- Caliptra-SS's ECDSA
	 * signing is only exposed through the Cmk-managed CM_ECDSA_CMK_SIGN
	 * command, which needs an opaque Cmk handle this raw-pubkey session
	 * has no way to carry.
	 */
	return -ENOSYS;
}

static int cptra_mci_ecdsa_op_verify(struct ecdsa_ctx *ctx, struct ecdsa_pkt *pkt)
{
	struct cptra_mci_ecdsa_drv_state *state = DEV_DATA(ctx->device);
	struct cptra_mci_ecdsa384_sig_verify_req req = { 0 };
	struct cptra_mci_ecdsa384_sig_verify_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	if (pkt->m_len != CPTRA_MCI_ECC384_SCALAR_SIZE ||
	    pkt->r_len != CPTRA_MCI_ECC384_SCALAR_SIZE ||
	    pkt->s_len != CPTRA_MCI_ECC384_SCALAR_SIZE) {
		LOG_ERR("Unsupported ECDSA packet size (m=%d r=%d s=%d, want %d each)",
			pkt->m_len, pkt->r_len, pkt->s_len,
			(int)CPTRA_MCI_ECC384_SCALAR_SIZE);
		return -EINVAL;
	}

	/*
	 * pkt->m is the already-hashed digest (e.g. SHA-384), not a raw
	 * message -- MC_ECDSA384_SIG_VERIFY does no hashing of its own.
	 */
	memcpy(req.pub_key_x, state->qx, sizeof(req.pub_key_x));
	memcpy(req.pub_key_y, state->qy, sizeof(req.pub_key_y));
	memcpy(req.signature_r, pkt->r, sizeof(req.signature_r));
	memcpy(req.signature_s, pkt->s, sizeof(req.signature_s));
	memcpy(req.hash, pkt->m, sizeof(req.hash));

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_ECDSA384_SIG_VERIFY,
						 &req.pub_key_x, sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_ECDSA384_SIG_VERIFY, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_ECDSA384_SIG_VERIFY failed: %d", ret);
		return ret;
	}

	return 0;
}

static int aspeed_mci_ecdsa_session_setup(const struct device *dev, struct ecdsa_ctx *ctx,
					  struct ecdsa_key *key)
{
	struct cptra_mci_ecdsa_drv_state *state = DEV_DATA(dev);

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	if (key->curve_id != ECC_CURVE_NIST_P384) {
		LOG_ERR("Unsupported curve id: %u", key->curve_id);
		return -EINVAL;
	}

	/*
	 * struct ecdsa_key carries qx/qy as bare pointers with no length
	 * field -- the caller is trusted to have CPTRA_MCI_ECC384_SCALAR_SIZE
	 * bytes behind each, same assumption the key->qx/qy consumers in
	 * this API family already make elsewhere.
	 */
	memcpy(state->qx, key->qx, sizeof(state->qx));
	memcpy(state->qy, key->qy, sizeof(state->qy));

	state->in_use = true;

	ctx->ops.sign = cptra_mci_ecdsa_op_sign;
	ctx->ops.verify = cptra_mci_ecdsa_op_verify;

	return 0;
}

static int aspeed_mci_ecdsa_session_free(const struct device *dev, struct ecdsa_ctx *ctx)
{
	struct cptra_mci_ecdsa_drv_state *state = DEV_DATA(dev);

	ARG_UNUSED(ctx);
	state->in_use = false;

	return 0;
}

static int aspeed_mci_ecdsa_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static struct ecdsa_driver_api cptra_mci_ecdsa_funcs = {
	.begin_session = aspeed_mci_ecdsa_session_setup,
	.free_session = aspeed_mci_ecdsa_session_free,
	.query_hw_caps = aspeed_mci_ecdsa_query_hw_caps,
};

static struct cptra_mci_ecdsa_drv_state cptra_mci_ecdsa_state;

#define ASPEED_CPTRA_MCI_ECDSA_INIT(inst)					\
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL,				\
			      &cptra_mci_ecdsa_state, NULL,			\
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,	\
			      &cptra_mci_ecdsa_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_MCI_ECDSA_INIT)
