/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_mci_lms

#include <string.h>
#include <zephyr/crypto/lms.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>

LOG_MODULE_REGISTER(cptra_mci_lms, CONFIG_LOG_DEFAULT_LEVEL);

struct cptra_mci_lms_drv_state {
	struct lms_pub_key key;
	bool in_use;
};

#define DEV_DATA(dev)				\
	((struct cptra_mci_lms_drv_state *)	\
	(dev)->data)

static int cptra_mci_lms_op_sign(struct lms_ctx *ctx, struct lms_pkt *pkt)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(pkt);

	/*
	 * LMS keys are generated offline (e.g. for firmware signing) --
	 * Caliptra-SS exposes no on-device LMS signing command at all, only
	 * MC_LMS_SIG_VERIFY.
	 */
	return -ENOSYS;
}

static int cptra_mci_lms_op_verify(struct lms_ctx *ctx, struct lms_pkt *pkt)
{
	struct cptra_mci_lms_drv_state *state = DEV_DATA(ctx->device);
	/*
	 * signature_ots (1252 bytes) + signature_tree_path (360 bytes) make
	 * this request 1720 bytes -- kept off the stack. req is static
	 * (shared across calls), so the lock has to cover everything from
	 * the first write below through the execute() call -- see
	 * cptra_mci_mbox_txn_begin()'s comment.
	 */
	static struct cptra_mci_lms_verify_req req;
	struct cptra_mci_lms_verify_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	if (pkt->sig.tree_type != state->key.pub_key_tree_type) {
		LOG_ERR("Signature tree type 0x%x does not match key tree type 0x%x",
			pkt->sig.tree_type, state->key.pub_key_tree_type);
		return -EINVAL;
	}

	if (pkt->m_len != CPTRA_MCI_LMS_HASH_SIZE) {
		LOG_ERR("Unsupported digest size %d (want %d)", pkt->m_len,
			(int)CPTRA_MCI_LMS_HASH_SIZE);
		return -EINVAL;
	}

	cptra_mci_mbox_txn_begin();

	req.pub_key_tree_type = state->key.pub_key_tree_type;
	req.pub_key_ots_type = state->key.pub_key_ots_type;
	memcpy(req.pub_key_id, state->key.pub_key_id, sizeof(req.pub_key_id));
	memcpy(req.pub_key_digest, state->key.pub_key_digest, sizeof(req.pub_key_digest));
	req.signature_q = pkt->sig.q;
	memcpy(req.signature_ots, pkt->sig.ots, sizeof(req.signature_ots));
	req.signature_tree_type = pkt->sig.tree_type;
	memcpy(req.signature_tree_path, pkt->sig.tree_path, sizeof(req.signature_tree_path));
	memcpy(req.hash, pkt->m, sizeof(req.hash));

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_LMS_SIG_VERIFY,
						 &req.pub_key_tree_type,
						 sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_LMS_SIG_VERIFY, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);

	cptra_mci_mbox_txn_end();

	if (ret) {
		LOG_ERR("MC_LMS_SIG_VERIFY failed: %d", ret);
		return ret;
	}

	return 0;
}

static int aspeed_mci_lms_session_setup(const struct device *dev, struct lms_ctx *ctx,
					struct lms_pub_key *key)
{
	struct cptra_mci_lms_drv_state *state = DEV_DATA(dev);

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	/*
	 * Caliptra-SS's runtime hard-codes and rejects every LMS/LM-OTS
	 * parameter set except this one -- see CPTRA_MCI_LMS_TREE_TYPE_FIXED/
	 * CPTRA_MCI_LMS_OTS_TYPE_FIXED in cptra_mci_mbox.h. These are plain
	 * native uint32_t values here (12 / 7), not byte-swapped -- unlike
	 * the older cptra_lms.c driver (a different transport, kept as-is),
	 * this one does not expect pub_key_tree_type/pub_key_ots_type to be
	 * pre-encoded big-endian by the caller.
	 */
	if (key->pub_key_tree_type != CPTRA_MCI_LMS_TREE_TYPE_FIXED ||
	    key->pub_key_ots_type != CPTRA_MCI_LMS_OTS_TYPE_FIXED) {
		LOG_ERR("Unsupported LMS key type (tree_type=%u ots_type=%u)",
			key->pub_key_tree_type, key->pub_key_ots_type);
		return -EINVAL;
	}

	memcpy(&state->key, key, sizeof(state->key));

	state->in_use = true;

	ctx->ops.sign = cptra_mci_lms_op_sign;
	ctx->ops.verify = cptra_mci_lms_op_verify;

	return 0;
}

static int aspeed_mci_lms_session_free(const struct device *dev, struct lms_ctx *ctx)
{
	struct cptra_mci_lms_drv_state *state = DEV_DATA(dev);

	ARG_UNUSED(ctx);
	state->in_use = false;

	return 0;
}

static int aspeed_mci_lms_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static struct lms_driver_api cptra_mci_lms_funcs = {
	.begin_session = aspeed_mci_lms_session_setup,
	.free_session = aspeed_mci_lms_session_free,
	.query_hw_caps = aspeed_mci_lms_query_hw_caps,
};

static struct cptra_mci_lms_drv_state cptra_mci_lms_state;

#define ASPEED_CPTRA_MCI_LMS_INIT(inst)					\
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL,				\
			      &cptra_mci_lms_state, NULL,			\
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,	\
			      &cptra_mci_lms_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_MCI_LMS_INIT)
