/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

/**
 * @file
 * @brief Crypto LMS structure definitions
 *
 * This file contains the LMS Abstraction layer structures.
 *
 * [Experimental] Users should note that the Structures can change
 * as a part of ongoing development.
 */

#ifndef ZEPHYR_INCLUDE_CRYPTO_LMS_STRUCTS_H_
#define ZEPHYR_INCLUDE_CRYPTO_LMS_STRUCTS_H_

/**
 * @addtogroup crypto_lms
 * @{
 */

/* Forward declarations */
struct lms_ctx;
struct lms_pkt;

#define LMS_PUB_KEY_ID_LEN		16
#define LMS_PUB_KEY_DGST		24
#define LMS_SIG_OTS_LEN			1252
#define LMS_SIG_TREE_PATH		360

struct lms_pub_key {
	uint32_t pub_key_tree_type;
	uint32_t pub_key_ots_type;
	uint8_t pub_key_id[LMS_PUB_KEY_ID_LEN];
	uint8_t pub_key_digest[LMS_PUB_KEY_DGST];
};

struct lms_signature {
	uint32_t q;
	uint8_t ots[LMS_SIG_OTS_LEN];
	uint32_t tree_type;
	uint8_t tree_path[LMS_SIG_TREE_PATH];
};

struct lms_ops {
	int (*sign)(struct lms_ctx *ctx, struct lms_pkt *pkt);
	int (*verify)(struct lms_ctx *ctx, struct lms_pkt *pkt);
};

/**
 * Structure encoding session parameters.
 *
 * Refer to comments for individual fields to know the contract
 * in terms of who fills what and when w.r.t begin_session() call.
 */
struct lms_ctx {

	/** Place for driver to return function pointers to be invoked per
	 * cipher operation. To be populated by crypto driver on return from
	 * begin_session() based on the algo/mode chosen by the app.
	 */
	struct lms_ops ops;

	/** The device driver instance this crypto context relates to. Will be
	 * populated by the begin_session() API.
	 */
	const struct device *device;

	/** If the driver supports multiple simultaneously crypto sessions, this
	 * will identify the specific driver state this crypto session relates
	 * to. Since dynamic memory allocation is not possible, it is
	 * suggested that at build time drivers allocate space for the
	 * max simultaneous sessions they intend to support. To be populated
	 * by the driver on return from begin_session().
	 */
	void *drv_sessn_state;

	/** Place for the user app to put info relevant stuff for resuming when
	 * completion callback happens for async ops. Totally managed by the
	 * app.
	 */
	void *app_sessn_state;
};

/**
 * Structure encoding IO parameters of one cryptographic
 * operation like sign/verify.
 *
 * The fields which has not been explicitly called out has to
 * be filled up by the app before making the cipher_xxx_op()
 * call.
 */
struct lms_pkt {

	struct lms_signature sig;

	/** Context this packet relates to. This can be useful to get the
	 * session details, especially for async ops. Will be populated by the
	 * cipher_xxx_op() API based on the ctx parameter.
	 */
	struct lms_ctx *ctx;
};

/**
 * @}
 */
#endif /* ZEPHYR_INCLUDE_CRYPTO_LMS_STRUCTS_H_ */
