/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

/**
 * @file
 * @brief LMS APIs
 *
 * This file contains the LMS Abstraction layer APIs.
 *
 * [Experimental] Users should note that the APIs can change
 * as a part of ongoing development.
 */

#ifndef ZEPHYR_INCLUDE_CRYPTO_LMS_H_
#define ZEPHYR_INCLUDE_CRYPTO_LMS_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include "lms_structs.h"

/* The API a lms driver should implement */
__subsystem struct lms_driver_api {
	int (*query_hw_caps)(const struct device *dev);

	/* Setup a lms session */
	int (*begin_session)(const struct device *dev, struct lms_ctx *ctx,
			     struct lms_pub_key *key);

	/* Tear down an established session */
	int (*free_session)(const struct device *dev, struct lms_ctx *ctx);

	/* Perform LMS sign */
	int (*sign)(struct lms_ctx *ctx, struct lms_pkt *pkt);

	/* Perform LMS verify */
	int (*verify)(struct lms_ctx *ctx, struct lms_pkt *pkt);
};

/**
 * @brief Query the lms hardware capabilities
 *
 * This API is used by the app to query the capabilities supported by the
 * lms device. Based on this the app can specify a subset of the supported
 * options to be honored for a session during lms_begin_session().
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @return bitmask of supported options.
 */
static inline int lms_query_hwcaps(const struct device *dev)
{
	struct lms_driver_api *api;
	int tmp;

	api = (struct lms_driver_api *)dev->api;

	tmp = api->query_hw_caps(dev);

	return tmp;
}

/**
 * @brief Setup a lms session
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
static inline int lms_begin_session(const struct device *dev,
				    struct lms_ctx *ctx, struct lms_pub_key *key)
{
	struct lms_driver_api *api;

	api = (struct lms_driver_api *)dev->api;
	ctx->device = dev;

	return api->begin_session(dev, ctx, key);
}

/**
 * @brief Cleanup a lms session
 *
 * Clears the hardware and/or driver state of a previous session.
 *
 * @param  dev      Pointer to the device structure for the driver instance.
 * @param  ctx      Pointer to the lms context structure of the session
 *			to be freed.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int lms_free_session(const struct device *dev,
				   struct lms_ctx *ctx)
{
	struct lms_driver_api *api;

	api = (struct lms_driver_api *)dev->api;

	return api->free_session(dev, ctx);
}

/**
 * @brief Perform LMS sign.
 *
 * @param  ctx   Pointer to the lms context of this op.
 * @param  pkt   Structure holding the input/output buffer pointers.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int lms_sign(struct lms_ctx *ctx,
			   struct lms_pkt *pkt)
{
	pkt->ctx = ctx;
	return ctx->ops.sign(ctx, pkt);
}

/**
 * @brief Perform LMS verify.
 *
 * @param  ctx   Pointer to the lms context of this op.
 * @param  pkt   Structure holding the input/output buffer pointers.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int lms_verify(struct lms_ctx *ctx,
			     struct lms_pkt *pkt)
{
	pkt->ctx = ctx;
	return ctx->ops.verify(ctx, pkt);
}

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_CRYPTO_LMS_H_ */
