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

#ifndef ZEPHYR_DRIVERS_CRYPTO_ECDSA_ASPEED_PRIV_H_
#define ZEPHYR_DRIVERS_CRYPTO_ECDSA_ASPEED_PRIV_H_

struct aspeed_ecdsa_ctx {
	struct ecdsa_key key;
	uint32_t len;
};

struct aspeed_ecdsa_drv_state {
	struct aspeed_ecdsa_ctx data;
	uint8_t sram;
	bool in_use;
};

#endif  /* ZEPHYR_DRIVERS_CRYPTO_ECDSA_ASPEED_PRIV_H_ */
