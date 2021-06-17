/*
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_rng

#include <device.h>
#include <drivers/entropy.h>
#include <random/rand32.h>
#include <init.h>

#include "fsl_rng.h"

struct mcux_entropy_config {
	RNG_Type *base;
};

static int entropy_mcux_rng_get_entropy(const struct device *dev,
					 uint8_t *buffer,
					 uint16_t length)
{
	const struct mcux_entropy_config *config = dev->config;
	status_t status;

	ARG_UNUSED(dev);

	status = RNG_GetRandomData(config->base, buffer, length);
	__ASSERT_NO_MSG(!status);

	return 0;
}

static const struct entropy_driver_api entropy_mcux_rng_api_funcs = {
	.get_entropy = entropy_mcux_rng_get_entropy
};

static const struct mcux_entropy_config entropy_mcux_config = {
	.base = (RNG_Type *)DT_INST_REG_ADDR(0)
};

static int entropy_mcux_rng_init(const struct device *);

DEVICE_DT_INST_DEFINE(0,
		    entropy_mcux_rng_init, NULL, NULL,
		    &entropy_mcux_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &entropy_mcux_rng_api_funcs);

static int entropy_mcux_rng_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	RNG_Init(entropy_mcux_config.base);

	return 0;
}
