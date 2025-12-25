/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_lms

#include <soc.h>
#include <zephyr/crypto/lms.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/cptra.h>
#include <zephyr/drivers/misc/aspeed/cptra_mbox.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(cptra_lms, CONFIG_LOG_DEFAULT_LEVEL);

#define LMS_PUB_KEY_TREE_TYPE		12
#define LMS_PUB_KEY_OTS_TYPE		7

/* Device config */
struct cptra_lms_config {
	uintptr_t base;		/* LMS engine base address */
	uintptr_t scu_base;	/* SCU1 base address */
};

struct cptra_lms_drv_state {
	struct lms_pub_key key;
	bool in_use;
};

#define DEV_CFG(dev)				\
	((struct cptra_lms_config *)		\
	(dev)->config)

#define DEV_DATA(dev)				\
	((struct cptra_lms_drv_state *)	\
	(dev)->data)

static int cptra_lms_verify_trigger(const struct device *dev, struct lms_pkt *pkt)
{
	struct cptra_lms_drv_state *state = DEV_DATA(dev);
	struct cptra_lms_config *cfg = DEV_CFG(dev);
	uint32_t cmd, csum;
	uint32_t *p32;
	uint32_t ms;

	struct lms_pub_key *lms_key = &state->key;
	uint32_t pk_tree_type = sys_be32_to_cpu(lms_key->pub_key_tree_type);
	uint32_t pk_ots_type = sys_be32_to_cpu(lms_key->pub_key_ots_type);
	uint8_t *pk_id = lms_key->pub_key_id;
	uint8_t *pk_dgst = lms_key->pub_key_digest;
	uint32_t sig_q = pkt->sig.q;
	uint8_t *sig_ots = pkt->sig.ots;
	uint32_t sig_tree_type = pkt->sig.tree_type;
	uint8_t *sig_tree_path = pkt->sig.tree_path;
	int rc = 0;

	LOG_DBG("Caliptra LMS verify");

	/* Check LMS public key algorithm type */
	if (pk_tree_type != LMS_PUB_KEY_TREE_TYPE ||
	    pk_ots_type != LMS_PUB_KEY_OTS_TYPE)
		return -EINVAL;

	/* Check LMS signature algorithm type */
	if (sig_tree_type != LMS_PUB_KEY_TREE_TYPE)
		return -EINVAL;

	/* init mbox parameters */
	cmd = CPTRA_MBCMD_LMS_SIGNATURE_VERIFY;
	csum = 0;

	/* calculate checksum */
	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)&pk_tree_type, sizeof(pk_tree_type));
	csum = cptra_mbox_csum(csum, (uint8_t *)&pk_ots_type, sizeof(pk_ots_type));
	csum = cptra_mbox_csum(csum, pk_id, LMS_PUB_KEY_ID_LEN);
	csum = cptra_mbox_csum(csum, pk_dgst, LMS_PUB_KEY_DGST);
	csum = cptra_mbox_csum(csum, (uint8_t *)&sig_q, sizeof(sig_q));
	csum = cptra_mbox_csum(csum, sig_ots, LMS_SIG_OTS_LEN);
	csum = cptra_mbox_csum(csum, (uint8_t *)&sig_tree_type, sizeof(sig_tree_type));
	csum = cptra_mbox_csum(csum, sig_tree_path, LMS_SIG_TREE_PATH);

	while (cptra_mbox_lock())
		;

	sys_write32(cmd, cfg->base + CPTRA_MBOX_CMD);
	sys_write32(sizeof(csum) + sizeof(struct lms_pub_key) + sizeof(struct lms_signature),
		    cfg->base + CPTRA_MBOX_DLEN);

	sys_write32(csum, cfg->base + CPTRA_MBOX_DATAIN);
	sys_write32(pk_tree_type, cfg->base + CPTRA_MBOX_DATAIN);
	sys_write32(pk_ots_type, cfg->base + CPTRA_MBOX_DATAIN);

	p32 = (uint32_t *)pk_id;
	for (int i = 0; i < LMS_PUB_KEY_ID_LEN / 4; ++i)
		sys_write32(p32[i], cfg->base + CPTRA_MBOX_DATAIN);

	p32 = (uint32_t *)pk_dgst;
	for (int i = 0; i < LMS_PUB_KEY_DGST / 4; ++i)
		sys_write32(p32[i], cfg->base + CPTRA_MBOX_DATAIN);

	sys_write32(sig_q, cfg->base + CPTRA_MBOX_DATAIN);

	p32 = (uint32_t *)sig_ots;
	for (int i = 0; i < LMS_SIG_OTS_LEN / 4; ++i)
		sys_write32(p32[i], cfg->base + CPTRA_MBOX_DATAIN);

	sys_write32(sig_tree_type, cfg->base + CPTRA_MBOX_DATAIN);

	p32 = (uint32_t *)sig_tree_path;
	for (int i = 0; i < LMS_SIG_TREE_PATH / 4; ++i)
		sys_write32(p32[i], cfg->base + CPTRA_MBOX_DATAIN);

	sys_write32(0x1, cfg->base + CPTRA_MBOX_EXEC);

	while (1) {
		ms = FIELD_GET(CPTRA_MBOX_STS_PS, cptra_mbox_status());
		if (ms != CPTRA_MBSTS_CMD_BUSY)
			break;
	};

	/* Check if verify is passed */
	if (ms == CPTRA_MBSTS_CMD_FAILURE)
		rc = -EINVAL;

	while (cptra_mbox_unlock())
		;

	return rc;
}

int cptra_lms_verify(struct lms_ctx *ctx, struct lms_pkt *pkt)
{
	LOG_DBG("Caliptra LMS verify");

	if (pkt->sig.tree_type != LMS_PUB_KEY_TREE_TYPE) {
		LOG_ERR("This signature tree type 0x%x is not supported",
			pkt->sig.tree_type);
		return -EINVAL;
	}

	return cptra_lms_verify_trigger(ctx->device, pkt);
}

static int aspeed_lms_session_setup(const struct device *dev, struct lms_ctx *ctx,
				    struct lms_pub_key *input_key)
{
	struct cptra_lms_drv_state *state = DEV_DATA(dev);
	struct lms_pub_key *key;

	LOG_DBG("Caliptra LMS session setup");

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	key = &state->key;

	/*
	 * Caliptra LMS supports only the following key types:
	 * 12 = LMS_SHA256_N24_H15
	 * 7 = LMS_SHA256_N24_W4
	 */
	if (sys_be32_to_cpu(input_key->pub_key_tree_type) != LMS_PUB_KEY_TREE_TYPE ||
	    sys_be32_to_cpu(input_key->pub_key_ots_type) != LMS_PUB_KEY_OTS_TYPE) {
		LOG_ERR("This key type is not supported, tree type: 0x%x, ots type: 0x%x",
			sys_be32_to_cpu(input_key->pub_key_tree_type),
			sys_be32_to_cpu(input_key->pub_key_ots_type));
		return -EINVAL;
	}

	state->in_use = true;

	memcpy(key, input_key, sizeof(struct lms_pub_key));

	ctx->ops.verify = cptra_lms_verify;

	return 0;
}

static int aspeed_lms_session_free(const struct device *dev, struct lms_ctx *ctx)
{
	struct cptra_lms_drv_state *state = DEV_DATA(dev);

	ARG_UNUSED(ctx);
	state->in_use = false;

	return 0;
}

int cptra_lms_init(const struct device *dev)
{
	struct cptra_lms_drv_state *state = DEV_DATA(dev);
	struct cptra_lms_config *cfg = DEV_CFG(dev);

	state->in_use = false;

	if (!(sys_read32(cfg->scu_base + SCU1_CPTRA) & SCU1_CPTRA_RDY_FOR_RT)) {
		LOG_WRN("Caliptra is unavailable");
		return -ENODEV;
	}

	LOG_INF("\t0x%x: Caliptra LMS HW accelerator registered",
		(uint32_t)cfg->base);

	return 0;
}

static struct lms_driver_api lms_funcs = {
	.begin_session = aspeed_lms_session_setup,
	.free_session = aspeed_lms_session_free,
	.query_hw_caps = NULL,
};

static const struct cptra_lms_config cptra_lms_config = {
	.base = DT_REG_ADDR(DT_PARENT(DT_DRV_INST(0))),
	.scu_base = DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PARENT(DT_DRV_INST(0)), aspeed_scu, 0)),
};

static struct cptra_lms_drv_state cptra_lms_state;

#define ASPEED_CPTRA_LMS_INIT(inst)					\
DEVICE_DT_INST_DEFINE(inst, cptra_lms_init, NULL,			\
		      &cptra_lms_state, &cptra_lms_config,		\
		      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,		\
		      (void *)&lms_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_LMS_INIT)
