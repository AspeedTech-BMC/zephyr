/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_sha

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/cptra.h>

LOG_MODULE_REGISTER(cptra_sha, CONFIG_LOG_DEFAULT_LEVEL);

enum cptra_sha_modes {
	CPTRA_SHA384_STREAM,
	CPTRA_SHA512_STREAM,
};

struct cptra_sha_ctx {
	enum hash_algo algo;
	uint32_t dgst_len;
};

struct cptra_sha_config {
	uintptr_t base;		/* SHA engine base address */
	uintptr_t scu_base;	/* SCU base address */
};

struct cptra_sha_drv_state {
	struct cptra_sha_ctx data;
	bool in_use;
};

#define DEV_CFG(dev)				\
	((struct cptra_sha_config *)		\
	(dev)->config)

#define DEV_DATA(dev)				\
	((struct cptra_sha_drv_state *)		\
	(dev)->data)

#define ASPEED_HASH_CAPS_SUPPORT	(CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

static int cptra_sha_update(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	struct cptra_sha_config *cfg = DEV_CFG(ctx->device);
	uint32_t din_be;
	uint32_t dlen_sum;
	uint8_t *p8;
	uint32_t i;

	/* update length */
	dlen_sum = sys_read32(cfg->base + CPTRA_SHA_DLEN) + pkt->in_len;
	sys_write32(dlen_sum, cfg->base + CPTRA_SHA_DLEN);

	din_be = 0;
	for (i = 0, p8 = (uint8_t *)pkt->in_buf; i < pkt->in_len; ++i) {
		if (i && (i % sizeof(din_be) == 0)) {
			sys_write32(din_be, cfg->base + CPTRA_SHA_DATAIN);
			din_be = 0;
		}

		din_be <<= 8;
		din_be |= p8[i];
	}

	if (i % sizeof(din_be))
		din_be <<= (8 * (sizeof(din_be) - (i % sizeof(din_be))));

	sys_write32(din_be, cfg->base + CPTRA_SHA_DATAIN);

	return 0;
}

static int cptra_sha_final(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	struct cptra_sha_drv_state *state = DEV_DATA(ctx->device);
	struct cptra_sha_config *cfg = DEV_CFG(ctx->device);
	uint32_t i, *p32;
	uint32_t sts;

	/* trigger SHA calculation */
	sys_write32(0x1, cfg->base + CPTRA_SHA_EXEC);

	/* wait for completion */
	while (1) {
		sts = sys_read32(cfg->base + CPTRA_SHA_STS);
		if (sts & CPTRA_SHA_STS_VLD)
			break;
	}

	/* get the SHA digest in big-endian */
	p32 = (uint32_t *)pkt->out_buf;
	for (i = 0; i < (state->data.dgst_len / sizeof(*p32)); ++i, p32++)
		*p32 = sys_be32_to_cpu(sys_read32(cfg->base + CPTRA_SHA_DIGEST(i)));

	/* release CPTRA SHA lock */
	sys_write32(0x1, cfg->base + CPTRA_SHA_LOCK);

	return 0;
}

static int cptra_sha_compute(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	if (finish)
		return cptra_sha_final(ctx, pkt);
	else
		return cptra_sha_update(ctx, pkt);
}

static int cptra_sha_session_setup(const struct device *dev,
				   struct hash_ctx *ctx,
				   enum hash_algo algo)
{
	struct cptra_sha_drv_state *state = DEV_DATA(dev);
	struct cptra_sha_config *cfg = DEV_CFG(dev);
	struct cptra_sha_ctx *cs_ctx = &state->data;
	struct cptra_sha_register_s *sha_register;
	union cptra_sha_lock_s sha_lock;
	uint32_t mode;
	uint32_t reg;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	sha_register = (struct cptra_sha_register_s *)cfg->base;
	state->in_use = true;
	cs_ctx->algo = algo;

	switch (algo) {
	case CRYPTO_HASH_ALGO_SHA384:
		mode = CPTRA_SHA384_STREAM;
		cs_ctx->dgst_len = 48;
		break;
	case CRYPTO_HASH_ALGO_SHA512:
		mode = CPTRA_SHA512_STREAM;
		cs_ctx->dgst_len = 64;
		break;
	default:
		rc = -EINVAL;
		goto free_n_out;
	};

	/* get CPTRA SHA lock */
	if (reg_read_poll_timeout(sha_register, sha_lock, sha_lock,
				  sha_lock.fields.lock == 0, 10, 1000))
		return -EBUSY;

	/* zero clear SHA */
	sys_write32(CPTRA_SHA_CTRL_ZEROIZE, cfg->base + CPTRA_SHA_CTRL);

	/* zero clear length */
	sys_write32(0x0, cfg->base + CPTRA_SHA_DLEN);

	/* set SHA mode */
	reg = sys_read32(cfg->base + CPTRA_SHA_MODE);
	reg &= ~(CPTRA_SHA_MODE_SEL);
	reg |= FIELD_PREP(CPTRA_SHA_MODE_SEL, mode);
	sys_write32(reg, cfg->base + CPTRA_SHA_MODE);

	ctx->device = dev;
	ctx->hash_hndlr = cptra_sha_compute;

	return 0;

free_n_out:
	state->in_use = false;

	return rc;
}

static int cptra_sha_session_free(const struct device *dev,
				  struct hash_ctx *ctx)
{
	struct cptra_sha_drv_state *state = DEV_DATA(dev);

	ARG_UNUSED(ctx);
	state->in_use = false;

	return 0;
}

static int cptra_sha_query_hw_caps(const struct device *dev)
{
	return ASPEED_HASH_CAPS_SUPPORT;
}

static int cptra_sha_init(const struct device *dev)
{
	struct cptra_sha_drv_state *state = DEV_DATA(dev);
	struct cptra_sha_config *cfg = DEV_CFG(dev);

	state->in_use = false;

	if (!(sys_read32(cfg->scu_base + SCU1_CPTRA) & SCU1_CPTRA_RDY_FOR_RT)) {
		LOG_ERR("Caliptra is unavailable\n");
		return -ENODEV;
	}

	LOG_INF("0x%x: Aspeed Caliptra SHA Hardware Accelerator successfully registered",
		(uint32_t)cfg->base);

	return 0;
}

static struct crypto_driver_api hash_funcs = {
	.hash_begin_session = cptra_sha_session_setup,
	.hash_free_session = cptra_sha_session_free,
	.query_hw_caps = cptra_sha_query_hw_caps,
};

static const struct cptra_sha_config cptra_sha_config = {
	.base = DT_REG_ADDR(DT_DRV_INST(0)),
	.scu_base = DT_REG_ADDR(DT_INST_PHANDLE(0, aspeed_scu)),
};

static struct cptra_sha_drv_state cptra_sha_state;

DEVICE_DT_INST_DEFINE(0, cptra_sha_init, NULL,
		      &cptra_sha_state, &cptra_sha_config,
		      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,
		      (void *)&hash_funcs);
