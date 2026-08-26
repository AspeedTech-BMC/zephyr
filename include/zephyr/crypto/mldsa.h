/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

/**
 * @file
 * @brief ML-DSA APIs
 *
 * This file contains the ML-DSA Abstraction layer APIs.
 *
 * [Experimental] Users should note that the APIs can change
 * as a part of ongoing development.
 */

#ifndef ZEPHYR_INCLUDE_CRYPTO_MLDSA_H_
#define ZEPHYR_INCLUDE_CRYPTO_MLDSA_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include "mldsa_structs.h"

/* The API a mldsa driver should implement */
__subsystem struct mldsa_driver_api {
	int (*query_hw_caps)(const struct device *dev);

	/* Setup a mldsa session */
	int (*begin_session)(const struct device *dev, struct mldsa_ctx *ctx,
			     struct mldsa_pub_key *key);

	/* Tear down an established session */
	int (*free_session)(const struct device *dev, struct mldsa_ctx *ctx);

	/* Perform ML-DSA sign */
	int (*sign)(struct mldsa_ctx *ctx, struct mldsa_pkt *pkt);

	/* Perform ML-DSA verify */
	int (*verify)(struct mldsa_ctx *ctx, struct mldsa_pkt *pkt);
};

/**
 * @brief Query the mldsa hardware capabilities
 *
 * This API is used by the app to query the capabilities supported by the
 * mldsa device. Based on this the app can specify a subset of the supported
 * options to be honored for a session during mldsa_begin_session().
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @return bitmask of supported options.
 */
static inline int mldsa_query_hwcaps(const struct device *dev)
{
	struct mldsa_driver_api *api;
	int tmp;

	api = (struct mldsa_driver_api *)dev->api;

	tmp = api->query_hw_caps(dev);

	return tmp;
}

/**
 * @brief Setup a mldsa session
 *
 * Initializes one time parameters, like the session key, algorithm and cipher
 * mode which may remain constant for all operations in the session. The state
 * may be cached in hardware and/or driver data state variables.
 *
 * @param  dev      Pointer to the device structure for the driver instance.
 * @param  ctx      Pointer to the context structure.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int mldsa_begin_session(const struct device *dev,
				      struct mldsa_ctx *ctx, struct mldsa_pub_key *key)
{
	struct mldsa_driver_api *api;

	api = (struct mldsa_driver_api *)dev->api;
	ctx->device = dev;

	return api->begin_session(dev, ctx, key);
}

/**
 * @brief Cleanup a mldsa session
 *
 * Clears the hardware and/or driver state of a previous session.
 *
 * @param  dev      Pointer to the device structure for the driver instance.
 * @param  ctx      Pointer to the mldsa context structure of the session
 *			to be freed.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int mldsa_free_session(const struct device *dev,
				     struct mldsa_ctx *ctx)
{
	struct mldsa_driver_api *api;

	api = (struct mldsa_driver_api *)dev->api;

	return api->free_session(dev, ctx);
}

/**
 * @brief Perform ML-DSA sign.
 *
 * @param  ctx   Pointer to the mldsa context of this op.
 * @param  pkt   Structure holding the input/output buffer pointers.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int mldsa_sign(struct mldsa_ctx *ctx,
			     struct mldsa_pkt *pkt)
{
	pkt->ctx = ctx;
	return ctx->ops.sign(ctx, pkt);
}

/**
 * @brief Perform ML-DSA verify.
 *
 * @param  ctx   Pointer to the mldsa context of this op.
 * @param  pkt   Structure holding the input/output buffer pointers.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int mldsa_verify(struct mldsa_ctx *ctx,
			       struct mldsa_pkt *pkt)
{
	pkt->ctx = ctx;
	return ctx->ops.verify(ctx, pkt);
}

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_CRYPTO_MLDSA_H_ */
