/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_DRIVERS_CRYPTO_HACE_ASPEED_H_
#define ZEPHYR_DRIVERS_CRYPTO_HACE_ASPEED_H_

/* Crypto control registers */
#define ASPEED_HACE_SRC					0x00
#define ASPEED_HACE_DEST				0x04
#define ASPEED_HACE_CONTEXT				0x08	/* 8 byte aligned*/
#define ASPEED_HACE_DATA_LEN				0x0C
#define ASPEED_HACE_CMD					0x10
#define HACE_CMD_AES_KEY_FROM_OTP			BIT(24)	/*G6*/
#define HACE_CMD_MBUS_REQ_SYNC_EN			BIT(20)	/*G6*/
#define HACE_CMD_DES_SG_CTRL				BIT(19)	/*G6*/
#define HACE_CMD_SRC_SG_CTRL				BIT(18)	/*G6*/
#define HACE_CMD_SINGLE_DES				0
#define HACE_CMD_TRIPLE_DES				BIT(17)
#define HACE_CMD_AES_SELECT				0
#define HACE_CMD_DES_SELECT				BIT(16)
#define HACE_CMD_CTR_IV_AES_128				0	/*G6*/
#define HACE_CMD_AES_KEY_HW_EXP				BIT(13)	/*G6*/
#define HACE_CMD_ISR_EN					BIT(12)
#define HACE_CMD_DECRYPT				(0)
#define HACE_CMD_ENCRYPT				BIT(7)
#define HACE_CMD_ECB					(0)
#define HACE_CMD_CBC					(0x1 << 4)
#define HACE_CMD_CFB					(0x2 << 4)
#define HACE_CMD_OFB					(0x3 << 4)
#define HACE_CMD_CTR					(0x4 << 4)
#define HACE_CMD_AES128					(0)
#define HACE_CMD_AES192					(0x1 << 2)
#define HACE_CMD_AES256					(0x2 << 2)

#define ASPEED_HACE_STS					0x1C
#define HACE_CRYPTO_ISR					BIT(12)
#define HACE_HASH_ISR					BIT(9)
#define HACE_CRYPTO_BUSY				BIT(1)
#define HACE_HASH_BUSY					BIT(0)

#define ASPEED_HACE_HASH_SRC				0x20
#define ASPEED_HACE_HASH_DIGEST_BUFF			0x24

/* 64 byte aligned,g6 16 byte aligned */
#define ASPEED_HACE_HASH_KEY_BUFF			0x28
#define ASPEED_HACE_HASH_DATA_LEN			0x2C
#define HACE_SG_LAST					BIT(31)
#define ASPEED_HACE_HASH_CMD				0x30
#define HACE_SHA_BE_EN					BIT(3)
#define HACE_MD5_LE_EN					BIT(2)
#define HACE_ALGO_MD5					0
#define HACE_ALGO_SHA1					BIT(5)
#define HACE_ALGO_SHA224				BIT(6)
#define HACE_ALGO_SHA256				(BIT(4) | BIT(6))
#define HACE_ALGO_SHA512				(BIT(5) | BIT(6))
#define HACE_ALGO_SHA384				(BIT(5) | BIT(6) | BIT(10))
#define HACE_ALGO_SHA512_224				(BIT(5) | BIT(6) | BIT(10) | BIT(11))
#define HACE_ALGO_SHA512_256				(BIT(5) | BIT(6) | BIT(11))
#define HASH_CMD_MODE_MASK				(BIT(7) | BIT(8))
#define HASH_CMD_DIG_W_HMAC				BIT(7)
#define HASH_CMD_ACC_MODE				BIT(8)
#define HASH_CMD_HMAC_KEY				(BIT(7) | BIT(8))
#define HACE_SG_EN					BIT(18)

union crypto_data_src_s {
	volatile uint32_t value;
}; /* 00000000 */

union crypto_data_dst_s {
	volatile uint32_t value;
}; /* 000000004 */

union crypto_ctx_base_s {
	volatile uint32_t value;
}; /* 000000008 */

union crypto_data_len_s {
	volatile uint32_t value;
}; /* 00000000c */

union crypto_cmd_reg_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t engine_op_mode : 2;		/*[0-1]*/
		volatile uint32_t aes_key_len : 2;			/*[2-3]*/
		volatile uint32_t cipher_mode : 3;			/*[4-6]*/
		volatile uint32_t crypto_mode : 1;			/*[7-7]*/
		volatile uint32_t alg : 1;					/*[8-8]*/
		volatile uint32_t saving_ctx : 1;			/*[9-9]*/
		volatile uint32_t reserved0 : 2;			/*[10-11]*/
		volatile uint32_t enable_int : 1;			/*[12-12]*/
		volatile uint32_t aes_key_expansion : 1;	/*[13-13]*/
		volatile uint32_t ctr_mode : 2;				/*[14-15]*/
		volatile uint32_t aes_des_mode : 1;			/*[16-16]*/
		volatile uint32_t enable_triple_des : 1;	/*[17-17]*/
		volatile uint32_t src_sg_mode : 1;			/*[18-18]*/
		volatile uint32_t dst_sg_mode : 1;			/*[19-19]*/
		volatile uint32_t mbus_req_sync : 1;		/*[20-20]*/
		volatile uint32_t gcm_tag_addr_sel : 1;		/*[21-21]*/
		volatile uint32_t ghash_padding_sel : 1;	/*[22-22]*/
		volatile uint32_t ghash_tag_xor : 1;		/*[23-23]*/
		volatile uint32_t aes_key_src_sel : 1;		/*[24-24]*/
		volatile uint32_t reserved1 : 7;			/*[25-31]*/
	} fields;
}; /* 000000010 */

union crypto_gcm_add_len_s {
	volatile uint32_t value;
}; /* 000000014 */

union crypto_gcm_tag_dst_s {
	volatile uint32_t value;
}; /* 000000018 */

union hace_sts_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t hash_engine_sts : 1;		/*[0-0]*/
		volatile uint32_t crypto_engine_sts : 1;	/*[1-1]*/
		volatile uint32_t reserved0 : 1;			/*[2-2]*/
		volatile uint32_t cmd_que_sts : 1;			/*[3-3]*/
		volatile uint32_t reserved1 : 5;			/*[4-8]*/
		volatile uint32_t hash_int : 1;				/*[9-9]*/
		volatile uint32_t reserved2 : 2;			/*[10-11]*/
		volatile uint32_t crypto_int : 1;			/*[12-12]*/
		volatile uint32_t reserved3 : 19;			/*[13-31]*/
	} fields;
}; /* 00000001c */

union hash_data_src_s {
	volatile uint32_t value;
}; /* 000000020 */

union hash_dgst_dst_s {
	volatile uint32_t value;
}; /* 000000024 */

union hash_key_buf_s {
	volatile uint32_t value;
}; /* 000000028 */

union hash_data_len_s {
	volatile uint32_t value;
}; /* 00000002c */

union hash_cmd_reg_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t engine_op_mode : 2;		/*[0-1]*/
		volatile uint32_t byte_swap : 2;			/*[2-3]*/
		volatile uint32_t hash_alg : 3;				/*[4-6]*/
		volatile uint32_t hamc_cmd_mode : 2;		/*[7-8]*/
		volatile uint32_t enable_int : 1;			/*[9-9]*/
		volatile uint32_t sha512_alg_sel : 2;		/*[10-12]*/
		volatile uint32_t acc_first_block : 1;		/*[13-13]*/
		volatile uint32_t acc_last_block : 1;		/*[14-14]*/
		volatile uint32_t reserved0 : 3;			/*[15-17]*/
		volatile uint32_t src_sg_mode : 1;			/*[18-18]*/
		volatile uint32_t reserved1 : 13;			/*[19-31]*/
	} fields;
}; /* 000000030 */

union hash_padding_len_s {
	volatile uint32_t value;
}; /* 000000034 */

struct hace_register_s {
	union crypto_data_src_s crypto_data_src;			/* 00000000 */
	union crypto_data_dst_s crypto_data_dst;			/* 00000004 */
	union crypto_ctx_base_s crypto_ctx_base;			/* 00000008 */
	union crypto_data_len_s crypto_data_len;			/* 0000000c */
	union crypto_cmd_reg_s crypto_cmd_reg;				/* 00000010 */
	union crypto_gcm_add_len_s crypto_gcm_add_len;			/* 00000014 */
	union crypto_gcm_tag_dst_s crypto_gcm_tag_dst;			/* 00000018 */
	union hace_sts_s hace_sts;					/* 0000001c */
	union hash_data_src_s hash_data_src;				/* 00000020 */
	union hash_dgst_dst_s hash_dgst_dst;				/* 00000024 */
	union hash_key_buf_s hash_key_buf;				/* 00000028 */
	union hash_data_len_s hash_data_len;				/* 0000002c */
	union hash_cmd_reg_s hash_cmd_reg;				/* 00000030 */
	union hash_padding_len_s hash_padding_len;			/* 00000034 */
};

struct aspeed_hace_engine {
	struct hace_register_s *base; /* Hash and crypto engine base address */
	uintptr_t sbase; /* Hash and crypto engine base address */
	int busy_flag;
};

extern struct aspeed_hace_engine hace_eng;

#define SELECT_VAL_KEY_1() do { \
	sys_write32(sys_read32(hace_eng.sbase + 0xc) & ~(BIT(0)), hace_eng.sbase + 0xc); \
} while (false)

#define SELECT_VAL_KEY_2() do { \
	sys_write32(sys_read32(hace_eng.sbase + 0xc) & BIT(0), hace_eng.sbase + 0xc); \
} while (false)

#endif  /* ZEPHYR_DRIVERS_CRYPTO_HACE_ASPEED_H_ */
