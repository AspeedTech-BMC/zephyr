/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_hwrng

#include <zephyr/drivers/entropy.h>
#include <zephyr/kernel.h>
#include <string.h>

#define LOG_LEVEL		0
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hwrng_aspeed);

#define AST_RNG_CTRL		0x0
#define AST_RNG_DATA		0x4

#define RNG_DISABLE		BIT(0)
#define RNG_MODE		0x18
#define RNG_SET_MODE(x)		(((x) & 0x1F) << 0x1)
#define RNG_READY		BIT(31)

struct aspeed_entropy_config {
	uint32_t	base;
};

static bool rng_init;

static int entropy_aspeed_trng_read(const struct device *dev,
				    uint8_t *output, size_t len)
{
	const struct aspeed_entropy_config *config = dev->config;
	uint32_t *data = (uint32_t *)output;
	uint32_t ctrl, tmp;
	int timeout = 10;

	while (len > 4) {
		ctrl = sys_read32(config->base + AST_RNG_CTRL);
		if (ctrl & RNG_READY) {
			*data = sys_read32(config->base + AST_RNG_DATA);
			len -= 4;
			data++;
			timeout = 10;

		} else {
			timeout--;
			if (timeout <= 0)
				return -EINVAL;
			k_usleep(100);
		}
	}

	/* Handle the case where len is not a multiple of 4 */
	while (len > 0 && timeout > 0) {
		ctrl = sys_read32(config->base + AST_RNG_CTRL);
		if (ctrl & RNG_READY)
			tmp = sys_read32(config->base + AST_RNG_DATA);
		else {
			timeout--;
			if (timeout <= 0) {
				LOG_ERR("rng is not ready, len:%d", len);
				return -EINVAL;
			}
			k_usleep(100);
			continue;
		}

		output = (uint8_t *)data;
		memcpy(output, &tmp, len);
		break;
	}

	return 0;
}

static int entropy_aspeed_trng_init(const struct device *dev)
{
	const struct aspeed_entropy_config *config = dev->config;
	uint32_t ctrl;

	LOG_DBG("%s: trng_base:0x%x\n", __func__, config->base);

	/* Enable RNG */
	ctrl = sys_read32(config->base + AST_RNG_CTRL);
	ctrl &= ~RNG_DISABLE;

	/* Set RNG mode */
	ctrl |= RNG_SET_MODE(RNG_MODE);

	sys_write32(ctrl, config->base + AST_RNG_CTRL);
	rng_init = true;

	return 0;
}

static int entropy_aspeed_trng_get_entropy(const struct device *dev,
					   uint8_t *buffer,
					   uint16_t length)
{
	int ret;

	if (!rng_init)
		entropy_aspeed_trng_init(dev);

	ret = entropy_aspeed_trng_read(dev, buffer, length);

	return ret;
}

static int entropy_aspeed_trng_get_entropy_isr(const struct device *dev,
					       uint8_t *buf,
					       uint16_t len, uint32_t flags)
{
	size_t count;
	int ret;

	if (!rng_init)
		entropy_aspeed_trng_init(dev);

	if ((flags & ENTROPY_BUSYWAIT) == 0U) {

		/* No busy wait; return whatever data is available. */
		count = MIN(len, 4);
		ret = entropy_aspeed_trng_read(dev, buf, count);

	} else {
		/* Allowed to busy-wait */
		ret = entropy_aspeed_trng_get_entropy(dev, buf, len);
		count = len;
	}

	/* Data retrieved successfully. */
	if (!ret)
		return count;

	return ret;
}

static struct entropy_driver_api entropy_aspeed_trng_api_funcs = {
	.get_entropy = entropy_aspeed_trng_get_entropy,
	.get_entropy_isr = entropy_aspeed_trng_get_entropy_isr
};

static const struct aspeed_entropy_config entropy_aspeed_config = {
	.base = (uint32_t)DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0,
			entropy_aspeed_trng_init, NULL, NULL,
			&entropy_aspeed_config,
			PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
			&entropy_aspeed_trng_api_funcs);
