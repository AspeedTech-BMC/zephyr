/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_hace
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>

#include "hace_aspeed.h"

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hace_global);

/* Device config */
struct hace_config {
	uintptr_t base; /* Hash and crypto engine base address */
	uintptr_t sbase; /* Secure Boot engine base address */
	const struct device *clock_dev;
	const clock_control_subsys_t clk_id;
	const struct reset_dt_spec reset;
};

struct aspeed_hace_engine hace_eng;

#define DEV_CFG(dev)				 \
	((const struct hace_config *const) \
	 (dev)->config)

/* Crypto controller driver registration */
static int hace_init(const struct device *dev)
{
	const struct hace_config *config = DEV_CFG(dev);
	uint32_t hace_base = DEV_CFG(dev)->base;

	reset_line_assert_dt(&config->reset);

	k_usleep(100);

	clock_control_on(config->clock_dev, DEV_CFG(dev)->clk_id);

	k_msleep(10);

	reset_line_deassert_dt(&config->reset);

	hace_eng.base = (struct hace_register_s *)hace_base;

	return 0;
}

static const struct hace_config hace_aspeed_config = {
	.base = DT_REG_ADDR_BY_IDX(DT_NODELABEL(hace), 0),
	.sbase = DT_REG_ADDR_BY_IDX(DT_NODELABEL(hace), 1),
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, clk_id),
	.reset = RESET_DT_SPEC_INST_GET(0),
};

DEVICE_DT_INST_DEFINE(0, hace_init, NULL, NULL, &hace_aspeed_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
