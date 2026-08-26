/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

/**
 * @file
 * @brief Crypto ML-DSA structure definitions
 *
 * This file contains the ML-DSA Abstraction layer structures.
 *
 * [Experimental] Users should note that the Structures can change
 * as a part of ongoing development.
 */

#ifndef ZEPHYR_INCLUDE_CRYPTO_MLDSA_STRUCTS_H_
#define ZEPHYR_INCLUDE_CRYPTO_MLDSA_STRUCTS_H_

/**
 * @addtogroup crypto_mldsa
 * @{
 */

/* Forward declarations */
struct mldsa_ctx;
struct mldsa_pkt;

#define MLDSA87_PUB_KEY_LEN		2592
#define MLDSA87_SIG_LEN			4628

struct mldsa_pub_key {
	uint8_t key[MLDSA87_PUB_KEY_LEN];
};

struct mldsa_ops {
	int (*sign)(struct mldsa_ctx *ctx, struct mldsa_pkt *pkt);
	int (*verify)(struct mldsa_ctx *ctx, struct mldsa_pkt *pkt);
};

/**
 * Structure encoding session parameters.
 *
 * Refer to comments for individual fields to know the contract
 * in terms of who fills what and when w.r.t begin_session() call.
 */
struct mldsa_ctx {

	/** Place for driver to return function pointers to be invoked per
	 * cipher operation. To be populated by crypto driver on return from
	 * begin_session() based on the algo/mode chosen by the app.
	 */
	struct mldsa_ops ops;

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
struct mldsa_pkt {

	uint8_t sig[MLDSA87_SIG_LEN];

	/** Message being verified. Unlike ECDSA384/LMS raw verify, which
	 * take a pre-hashed digest, MC_MLDSA87_SIG_VERIFY hashes the message
	 * internally -- this is the raw message bytes, not a digest. Must be
	 * filled in by the app before calling mldsa_verify().
	 */
	uint8_t *m;
	int m_len;

	/** Context this packet relates to. This can be useful to get the
	 * session details, especially for async ops. Will be populated by the
	 * cipher_xxx_op() API based on the ctx parameter.
	 */
	struct mldsa_ctx *ctx;
};

/**
 * @}
 */
#endif /* ZEPHYR_INCLUDE_CRYPTO_MLDSA_STRUCTS_H_ */
