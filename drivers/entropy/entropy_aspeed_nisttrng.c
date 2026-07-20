/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * NIST TRNG (SP 800-90A DRBG) entropy driver for AST2700.
 * The hardware implements a CTR_DRBG with AES-256 and provides
 * 128 bits (RAND0-RAND3) per GEN_RANDOM command.
 */

#define DT_DRV_COMPAT aspeed_ast2700_nisttrng

#include <stdbool.h>
#include <string.h>

#include <zephyr/drivers/entropy.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nisttrng_aspeed, CONFIG_ENTROPY_LOG_LEVEL);

/* NIST TRNG register offsets */
#define NIST_TRNG_CTRL			0x00
#define   CTRL_CMD_NOP			0x0
#define   CTRL_CMD_GEN_NOISE		0x1
#define   CTRL_CMD_CREATE_STATE		0x3
#define   CTRL_CMD_GEN_RANDOM		0x6
#define   CTRL_CMD_ADVANCE_STATE	0x7
#define   CTRL_CMD_ZEROIZE		0xf

#define NIST_TRNG_MODE			0x04
#define   MODE_SEC_ALG_AES128		0
#define   MODE_SEC_ALG_AES256		BIT(0)

#define NIST_TRNG_STAT			0x0c
#define   STAT_BUSY			BIT(31)

#define NIST_TRNG_ISTAT			0x14
#define   ISTAT_DONE			BIT(4)
#define   ISTAT_ZEROIZED		BIT(0)

/* 128-bit random output (4 x 32-bit words) */
#define NIST_TRNG_RAND0			0x24
#define NIST_TRNG_RAND1			0x28
#define NIST_TRNG_RAND2			0x2c
#define NIST_TRNG_RAND3			0x30

/* Timeout per hardware command: 10 ms */
#define NIST_TRNG_TIMEOUT_US		10000U

struct aspeed_nisttrng_config {
	mm_reg_t base;
};

struct aspeed_nisttrng_data {
	bool ready;
};

static inline uint32_t trng_read(const struct aspeed_nisttrng_config *cfg,
				 uint32_t off)
{
	return sys_read32(cfg->base + off);
}

static inline void trng_write(const struct aspeed_nisttrng_config *cfg,
			      uint32_t off, uint32_t val)
{
	sys_write32(val, cfg->base + off);
}

/**
 * wait_istat - poll ISTAT until the given bit(s) are set, with timeout.
 *
 * Returns 0 on success, -ETIMEDOUT if the bit never set.
 */
static int wait_istat(const struct aspeed_nisttrng_config *cfg, uint32_t mask)
{
	bool done = WAIT_FOR((trng_read(cfg, NIST_TRNG_ISTAT) & mask) == mask,
			     NIST_TRNG_TIMEOUT_US, k_busy_wait(1));

	if (!done) {
		LOG_ERR("timeout: ISTAT=0x%08x want=0x%08x",
			trng_read(cfg, NIST_TRNG_ISTAT), mask);
		return -ETIMEDOUT;
	}

	return 0;
}

static int nisttrng_init_hw(const struct device *dev)
{
	const struct aspeed_nisttrng_config *cfg = dev->config;
	struct aspeed_nisttrng_data *data = dev->data;
	uint32_t val;
	int rc;

	/* Wait for hardware not busy */
	bool idle = WAIT_FOR(!(trng_read(cfg, NIST_TRNG_STAT) & STAT_BUSY),
			     NIST_TRNG_TIMEOUT_US, k_busy_wait(1));

	if (!idle) {
		LOG_ERR("busy at init, STAT=0x%08x", trng_read(cfg, NIST_TRNG_STAT));
		return -EBUSY;
	}

	/* Step 1: Zeroize — clears internal state */
	trng_write(cfg, NIST_TRNG_CTRL, CTRL_CMD_ZEROIZE);
	rc = wait_istat(cfg, ISTAT_ZEROIZED);
	if (rc)
		return rc;

	/* W1C: clear ZEROIZED status */
	trng_write(cfg, NIST_TRNG_ISTAT, ISTAT_ZEROIZED);

	/* Step 2: Select AES-256 security strength */
	val = trng_read(cfg, NIST_TRNG_MODE);
	val = (val & ~MODE_SEC_ALG_AES128) | MODE_SEC_ALG_AES256;
	trng_write(cfg, NIST_TRNG_MODE, val);

	/* Step 3: Generate noise (entropy source conditioning) */
	trng_write(cfg, NIST_TRNG_CTRL, CTRL_CMD_GEN_NOISE);
	rc = wait_istat(cfg, ISTAT_DONE);
	if (rc)
		return rc;

	trng_write(cfg, NIST_TRNG_ISTAT, ISTAT_DONE);

	/* Step 4: Instantiate the DRBG (create internal state) */
	trng_write(cfg, NIST_TRNG_CTRL, CTRL_CMD_CREATE_STATE);
	rc = wait_istat(cfg, ISTAT_DONE);
	if (rc)
		return rc;

	trng_write(cfg, NIST_TRNG_ISTAT, ISTAT_DONE);

	data->ready = true;

	LOG_DBG("NIST TRNG initialized");

	return 0;
}

/**
 * generate_128bits - issue one GEN_RANDOM command and copy up to 16 bytes
 *                    of output into @buf.
 *
 * @buf:     destination buffer
 * @len:     how many bytes to copy (1..16)
 *
 * Returns 0 on success, negative errno on error.
 */
static int generate_128bits(const struct aspeed_nisttrng_config *cfg,
			     uint8_t *buf, size_t len)
{
	uint32_t rand_words[4];
	int rc;

	/* Issue GEN_RANDOM command */
	trng_write(cfg, NIST_TRNG_CTRL, CTRL_CMD_GEN_RANDOM);

	rc = wait_istat(cfg, ISTAT_DONE);
	if (rc)
		return rc;

	/* W1C: clear DONE */
	trng_write(cfg, NIST_TRNG_ISTAT, ISTAT_DONE);

	/* Read 128-bit output */
	rand_words[0] = trng_read(cfg, NIST_TRNG_RAND0);
	rand_words[1] = trng_read(cfg, NIST_TRNG_RAND1);
	rand_words[2] = trng_read(cfg, NIST_TRNG_RAND2);
	rand_words[3] = trng_read(cfg, NIST_TRNG_RAND3);

	memcpy(buf, rand_words, MIN(len, sizeof(rand_words)));

	return 0;
}

static int entropy_aspeed_nisttrng_get_entropy(const struct device *dev,
					       uint8_t *buffer, uint16_t length)
{
	const struct aspeed_nisttrng_config *cfg = dev->config;
	struct aspeed_nisttrng_data *data = dev->data;
	int rc;

	if (!data->ready) {
		rc = nisttrng_init_hw(dev);
		if (rc)
			return rc;
	}

	while (length > 0) {
		size_t chunk = MIN((size_t)length, 16U);

		rc = generate_128bits(cfg, buffer, chunk);
		if (rc)
			return rc;

		buffer += chunk;
		length -= chunk;
	}

	return 0;
}

static int entropy_aspeed_nisttrng_get_entropy_isr(const struct device *dev,
						   uint8_t *buf, uint16_t len,
						   uint32_t flags)
{
	struct aspeed_nisttrng_data *data = dev->data;
	int rc;

	if (!data->ready)
		return -EAGAIN;

	if ((flags & ENTROPY_BUSYWAIT) == 0U) {
		/* Non-blocking path: busy-poll for one GEN_RANDOM, no sleeping. */
		size_t chunk = MIN((size_t)len, 16U);

		rc = generate_128bits(dev->config, buf, chunk);
		if (rc)
			return rc;

		return (int)chunk;
	}

	/* Busy-wait path: fulfil the full request */
	rc = entropy_aspeed_nisttrng_get_entropy(dev, buf, len);
	if (rc)
		return rc;

	return len;
}

static int entropy_aspeed_nisttrng_init(const struct device *dev)
{
	uint8_t buf[16];
	int rc;

	rc = nisttrng_init_hw(dev);
	if (rc)
		return rc;

	rc = generate_128bits(dev->config, buf, sizeof(buf));
	if (rc) {
		LOG_ERR("self-test read failed: %d", rc);
		return rc;
	}

	LOG_INF("ready, sample: "
		"%02x%02x%02x%02x %02x%02x%02x%02x "
		"%02x%02x%02x%02x %02x%02x%02x%02x",
		buf[0],  buf[1],  buf[2],  buf[3],
		buf[4],  buf[5],  buf[6],  buf[7],
		buf[8],  buf[9],  buf[10], buf[11],
		buf[12], buf[13], buf[14], buf[15]);

	return 0;
}

static const struct entropy_driver_api entropy_aspeed_nisttrng_api = {
	.get_entropy     = entropy_aspeed_nisttrng_get_entropy,
	.get_entropy_isr = entropy_aspeed_nisttrng_get_entropy_isr,
};

#define ASPEED_NISTTRNG_INIT(n)						\
	static const struct aspeed_nisttrng_config			\
		aspeed_nisttrng_##n##_config = {			\
		.base = DT_INST_REG_ADDR(n),				\
	};								\
									\
	static struct aspeed_nisttrng_data aspeed_nisttrng_##n##_data;	\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			      entropy_aspeed_nisttrng_init,		\
			      NULL,					\
			      &aspeed_nisttrng_##n##_data,		\
			      &aspeed_nisttrng_##n##_config,		\
			      PRE_KERNEL_1,				\
			      CONFIG_ENTROPY_INIT_PRIORITY,		\
			      &entropy_aspeed_nisttrng_api);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_NISTTRNG_INIT)
