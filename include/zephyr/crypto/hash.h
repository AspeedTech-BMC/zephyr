/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Crypto Hash APIs
 *
 * This file contains the Crypto Abstraction layer APIs.
 */
#ifndef ZEPHYR_INCLUDE_CRYPTO_HASH_H_
#define ZEPHYR_INCLUDE_CRYPTO_HASH_H_


/**
 * @addtogroup crypto_hash
 * @{
 */

/* Hash digest/block size definition */
#define SHA1_DIGEST_SIZE	20
#define SHA1_BLOCK_SIZE		64
#define SHA1_IV_SIZE		32

#define SHA224_DIGEST_SIZE      28
#define SHA224_BLOCK_SIZE       64
#define SHA224_IV_SIZE		32

#define SHA256_DIGEST_SIZE      32
#define SHA256_BLOCK_SIZE       64
#define SHA256_IV_SIZE		32

#define SHA384_DIGEST_SIZE      48
#define SHA384_BLOCK_SIZE       128
#define SHA384_IV_SIZE		64

#define SHA512_DIGEST_SIZE      64
#define SHA512_BLOCK_SIZE       128
#define SHA512_IV_SIZE		64

#define HMAC_IPAD_VALUE		0x36
#define HMAC_OPAD_VALUE		0x5c

/**
 * Hash algorithm
 */
enum hash_algo {
	CRYPTO_HASH_ALGO_SHA1 = 1,
	CRYPTO_HASH_ALGO_SHA224,
	CRYPTO_HASH_ALGO_SHA256,
	CRYPTO_HASH_ALGO_SHA384,
	CRYPTO_HASH_ALGO_SHA512,
	CRYPTO_HASH_ALGO_SHA512_224,
	CRYPTO_HASH_ALGO_SHA512_256,
};

/* Forward declarations */
struct hash_ctx;
struct hash_pkt;


typedef int (*hash_op_t)(struct hash_ctx *ctx, struct hash_pkt *pkt,
			 bool finish);

typedef int (*hash_setkey_t)(struct hash_ctx *ctx, struct hash_pkt *pkt);
typedef int (*hash_digest_hmac_t)(struct hash_ctx *ctx, struct hash_pkt *pkt);

struct hash_ops {
	hash_setkey_t           setkey_hndlr;
	hash_digest_hmac_t      digest_hmac_hndlr;
};

/**
 * Structure encoding session parameters.
 *
 * Refer to comments for individual fields to know the contract
 * in terms of who fills what and when w.r.t begin_session() call.
 */
struct hash_ctx {
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

	/**
	 * Hash handler set up when the session begins.
	 */
	hash_op_t hash_hndlr;

	/**
	 * If it has started a multipart hash operation.
	 */
	bool started;

	/** How certain fields are to be interpreted for this session.
	 * (A bitmask of CAP_* below.)
	 * To be populated by the app before calling hash_begin_session().
	 * An app can obtain the capability flags supported by a hw/driver
	 * by calling crypto_query_hwcaps().
	 */
	uint16_t flags;

	/*
	 * Extra operations for hmac.
	 */
	struct hash_ops ops;

	uint32_t digest_size;
};

/**
 * Structure encoding IO parameters of a hash
 * operation.
 *
 * The fields which has not been explicitly called out has to
 * be filled up by the app before calling hash_compute().
 */
struct hash_pkt {

	/** Start address of key buffer */
	uint8_t *key_buf;

	/** Bytes to be operated upon */
	int key_len;

	/** Start address of input buffer */
	uint8_t *in_buf;

	/** Bytes to be operated upon */
	size_t  in_len;

	/**
	 * Start of the output buffer, to be allocated by
	 * the application. Can be NULL for in-place ops. To be populated
	 * with contents by the driver on return from op / async callback.
	 */
	uint8_t *out_buf;

	/**
	 * Context this packet relates to. This can be useful to get the
	 * session details, especially for async ops.
	 */
	struct hash_ctx *ctx;
};

/* Prototype for the application function to be invoked by the crypto driver
 * on completion of an async request. The app may get the session context
 * via the pkt->ctx field.
 */
typedef void (*hash_completion_cb)(struct hash_pkt *completed, int status);


/**
 * @}
 */
#endif /* ZEPHYR_INCLUDE_CRYPTO_HASH_H_ */
