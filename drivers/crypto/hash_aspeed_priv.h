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

#ifndef ZEPHYR_DRIVERS_CRYPTO_HASH_ASPEED_PRIV_H_
#define ZEPHYR_DRIVERS_CRYPTO_HASH_ASPEED_PRIV_H_

#define HASH_TMP_BUFF_SIZE	256

static const uint32_t sha1_iv[8] = {
	0x01234567UL, 0x89abcdefUL, 0xfedcba98UL, 0x76543210UL,
	0xf0e1d2c3UL, 0, 0, 0
};

static const uint32_t sha224_iv[8] = {
	0xd89e05c1UL, 0x07d57c36UL, 0x17dd7030UL, 0x39590ef7UL,
	0x310bc0ffUL, 0x11155868UL, 0xa78ff964UL, 0xa44ffabeUL
};

static const uint32_t sha256_iv[8] = {
	0x67e6096aUL, 0x85ae67bbUL, 0x72f36e3cUL, 0x3af54fa5UL,
	0x7f520e51UL, 0x8c68059bUL, 0xabd9831fUL, 0x19cde05bUL
};

static const uint32_t sha384_iv[16] = {
	0x5d9dbbcbUL, 0xd89e05c1UL, 0x2a299a62UL, 0x07d57c36UL,
	0x5a015991UL, 0x17dd7030UL, 0xd8ec2f15UL, 0x39590ef7UL,
	0x67263367UL, 0x310bc0ffUL, 0x874ab48eUL, 0x11155868UL,
	0x0d2e0cdbUL, 0xa78ff964UL, 0x1d48b547UL, 0xa44ffabeUL
};

static const uint32_t sha512_iv[16] = {
	0x67e6096aUL, 0x08c9bcf3UL, 0x85ae67bbUL, 0x3ba7ca84UL,
	0x72f36e3cUL, 0x2bf894feUL, 0x3af54fa5UL, 0xf1361d5fUL,
	0x7f520e51UL, 0xd182e6adUL, 0x8c68059bUL, 0x1f6c3e2bUL,
	0xabd9831fUL, 0x6bbd41fbUL, 0x19cde05bUL, 0x79217e13UL
};

static const uint32_t sha512_224_iv[16] = {
	0xC8373D8CUL, 0xA24D5419UL, 0x6699E173UL, 0xD6D4DC89UL,
	0xAEB7FA1DUL, 0x829CFF32UL, 0x14D59D67UL, 0xCF9F2F58UL,
	0x692B6D0FUL, 0xA84DD47BUL, 0x736FE377UL, 0x4289C404UL,
	0xA8859D3FUL, 0xC8361D6AUL, 0xADE61211UL, 0xA192D691UL
};

static const uint32_t sha512_256_iv[16] = {
	0x94213122UL, 0x2CF72BFCUL, 0xA35F559FUL, 0xC2644CC8UL,
	0x6BB89323UL, 0x51B1536FUL, 0x19773896UL, 0xBDEA4059UL,
	0xE23E2896UL, 0xE3FF8EA8UL, 0x251E5EBEUL, 0x92398653UL,
	0xFC99012BUL, 0xAAB8852CUL, 0xDC2DB70EUL, 0xA22CC581UL
};

struct aspeed_hmac_ctx {
	bool		setkey;
	uint8_t		key_buff[SHA512_BLOCK_SIZE];

	uint8_t		ipad[SHA512_BLOCK_SIZE];
	uint8_t		opad[SHA512_BLOCK_SIZE];
};

struct aspeed_hash_ctx {
	/* source base address: Scatter-Gather or Direct Access Mode */
	struct		aspeed_sg sg[2]; /* Must be 8 byte aligned */
	uint8_t		digest[64];	/* Must be 8 byte aligned */
	uint32_t	method;
	uint32_t	block_size;
	uint64_t	digcnt[2];	/* total length */
	uint32_t	bufcnt;
	uint8_t		buffer[HASH_TMP_BUFF_SIZE];

	uint32_t	*iv;
	uint8_t		iv_size;

	struct aspeed_hmac_ctx	hmac_data;
};

struct aspeed_hash_drv_state {
	struct aspeed_hash_ctx data;
	bool in_use;
};

#endif  /* ZEPHYR_DRIVERS_CRYPTO_HASH_ASPEED_PRIV_H_ */
