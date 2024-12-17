/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_ecdsa

#include <soc.h>
#include <zephyr/crypto/ecdsa.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/cptra.h>
#include "ecdsa_aspeed_priv.h"

LOG_MODULE_REGISTER(cptra_ecdsa, CONFIG_LOG_DEFAULT_LEVEL);

/* Device config */
struct cptra_ecdsa_config {
	uintptr_t base;		/* ECDSA engine base address */
	uintptr_t scu_base;	/* SCU1 base address */
};

struct cptra_ecdsa_drv_state {
	struct aspeed_ecdsa_ctx data;
	bool in_use;
};

#define DEV_CFG(dev)				\
	((struct cptra_ecdsa_config *)		\
	(dev)->config)

#define DEV_DATA(dev)				\
	((struct cptra_ecdsa_drv_state *)	\
	(dev)->data)

static uint32_t mbox_csum(uint32_t csum, uint8_t *data, uint32_t dlen)
{
	uint32_t i;

	if (!data)
		return csum;

	for (i = 0; i < dlen; ++i)
		csum -= data[i];

	return csum;
}

static int cptra_ecdsa_verify_trigger(const struct device *dev,
				      char *m, char *r, char *s,
				      char *qx, char *qy)
{
	struct cptra_ecdsa_config *cfg = DEV_CFG(dev);
	struct cptra_mbox_register_s *mbox_register;
	union cptra_mbox_lock_s mbox_lock;
	uint32_t cmd, csum;
	uint32_t sts;
	uint32_t *p32;
	int i;

	mbox_register = (struct cptra_mbox_register_s *)cfg->base;

	/* get CPTRA MBOX lock */
	if (reg_read_poll_timeout(mbox_register, mbox_lock, mbox_lock,
				  mbox_lock.fields.lock == 0, 10, 1000))
		return -EBUSY;

	/* check MBOX is ready for command */
	sts = sys_read32(cfg->base + CPTRA_MBOX_STS);
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	/* init mbox parameters */
	cmd = CPTRA_MBCMD_ECDSA384_SIGNATURE_VERIFY;
	csum = 0;

	/* calculate checksum */
	csum = mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = mbox_csum(csum, qx, CPTRA_ECDSA_SIG_LEN / 2);
	csum = mbox_csum(csum, qy, CPTRA_ECDSA_SIG_LEN / 2);
	csum = mbox_csum(csum, r, CPTRA_ECDSA_SIG_LEN / 2);
	csum = mbox_csum(csum, s, CPTRA_ECDSA_SIG_LEN / 2);

	/* write command, data length */
	sys_write32(cmd, cfg->base + CPTRA_MBOX_CMD);
	sys_write32(sizeof(csum) + (CPTRA_ECDSA_SIG_LEN << 1), cfg->base + CPTRA_MBOX_DLEN);

	/* write ECDSA384_SIGNATURE_VERIFY command parameters */
	sys_write32(csum, cfg->base + CPTRA_MBOX_DATAIN);

	for (i = 0, p32 = (uint32_t *)qx; i < ((CPTRA_ECDSA_SIG_LEN / 2) / sizeof(*p32)); ++i)
		sys_write32(p32[i], cfg->base + CPTRA_MBOX_DATAIN);

	for (i = 0, p32 = (uint32_t *)qy; i < ((CPTRA_ECDSA_SIG_LEN / 2) / sizeof(*p32)); ++i)
		sys_write32(p32[i], cfg->base + CPTRA_MBOX_DATAIN);

	for (i = 0, p32 = (uint32_t *)r; i < ((CPTRA_ECDSA_SIG_LEN / 2) / sizeof(*p32)); ++i)
		sys_write32(p32[i], cfg->base + CPTRA_MBOX_DATAIN);

	for (i = 0, p32 = (uint32_t *)s; i < ((CPTRA_ECDSA_SIG_LEN / 2) / sizeof(*p32)); ++i)
		sys_write32(p32[i], cfg->base + CPTRA_MBOX_DATAIN);

	/* trigger mbox command */
	sys_write32(0x1, cfg->base + CPTRA_MBOX_EXEC);

	/* poll for result */
	while (1) {
		sts = FIELD_GET(CPTRA_MBOX_STS_PS, sys_read32(cfg->base + CPTRA_MBOX_STS));
		if (sts != CPTRA_MBSTS_CMD_BUSY)
			break;
	}

	/* unlock mbox */
	sys_write32(0x0, cfg->base + CPTRA_MBOX_EXEC);

	return (sts == CPTRA_MBSTS_CMD_FAILURE) ? sts : 0;
}

int cptra_ecdsa_verify(struct ecdsa_ctx *ctx, struct ecdsa_pkt *pkt)
{
	struct cptra_ecdsa_drv_state *state = DEV_DATA(ctx->device);
	struct ecdsa_key *key = &state->data.key;

	if (pkt->r_len != 48 || pkt->s_len != 48 ||
	    pkt->m_len != 48 || key->curve_id != ECC_CURVE_NIST_P384) {
		LOG_ERR("This packet is not supported");
		return -EINVAL;

	}

	return cptra_ecdsa_verify_trigger(ctx->device, pkt->m, pkt->r, pkt->s, key->qx, key->qy);
}

static int aspeed_ecdsa_session_setup(const struct device *dev,
				      struct ecdsa_ctx *ctx,
				      struct ecdsa_key *key)
{
	struct cptra_ecdsa_drv_state *state = DEV_DATA(dev);
	struct aspeed_ecdsa_ctx *data;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	data = &state->data;
	state->in_use = true;

	memcpy(&data->key, key, sizeof(struct ecdsa_key));

	ctx->ops.verify = cptra_ecdsa_verify;
	ctx->device = dev;

	return 0;
}

static int aspeed_ecdsa_session_free(const struct device *dev, struct ecdsa_ctx *ctx)
{
	struct cptra_ecdsa_drv_state *state = DEV_DATA(dev);

	ARG_UNUSED(ctx);
	state->in_use = false;

	return 0;
}

int cptra_ecdsa_init(const struct device *dev)
{
	struct cptra_ecdsa_drv_state *state = DEV_DATA(dev);
	struct cptra_ecdsa_config *cfg = DEV_CFG(dev);

	state->in_use = false;

	if (!(sys_read32(cfg->scu_base + SCU1_CPTRA) & SCU1_CPTRA_RDY_FOR_RT)) {
		LOG_ERR("Caliptra is unavailable\n");
		return -ENODEV;
	}

	LOG_INF("0x%x: Aspeed Caliptra ECDSA Hardware Accelerator successfully registered",
		(uint32_t)cfg->base);

	return 0;
}

static struct ecdsa_driver_api ecdsa_funcs = {
	.begin_session = aspeed_ecdsa_session_setup,
	.free_session = aspeed_ecdsa_session_free,
	.query_hw_caps = NULL,
};

static const struct cptra_ecdsa_config cptra_ecdsa_config = {
	.base = DT_REG_ADDR(DT_DRV_INST(0)),
	.scu_base = DT_REG_ADDR(DT_INST_PHANDLE(0, aspeed_scu)),
};

static struct cptra_ecdsa_drv_state cptra_ecdsa_state;

DEVICE_DT_INST_DEFINE(0, cptra_ecdsa_init, NULL,
		      &cptra_ecdsa_state, &cptra_ecdsa_config,
		      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,
		      (void *)&ecdsa_funcs);

