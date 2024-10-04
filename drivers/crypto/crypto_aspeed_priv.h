/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

/**
 * @file
 * @brief tinycrypt driver context info
 *
 * The file defines the structure which is used to store per session context
 * by the driver. Placed in common location so that crypto applications
 * can allocate memory for the required number of sessions, to free driver
 * from dynamic memory allocation.
 */

#ifndef ZEPHYR_DRIVERS_CRYPTO_ASPEED_PRIV_H_
#define ZEPHYR_DRIVERS_CRYPTO_ASPEED_PRIV_H_

struct aspeed_sg {
	uint32_t len;
	uint32_t addr;
};

struct aspeed_crypto_ctx {
	uint8_t ctx[64];
	struct aspeed_sg src_sg;
	struct aspeed_sg dst_sg;
	uint32_t cmd;
};

struct aspeed_crypto_drv_state {
	struct aspeed_crypto_ctx data;
	bool in_use;
};

#endif  /* ZEPHYR_DRIVERS_CRYPTO_ASPEED_PRIV_H_ */
