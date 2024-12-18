/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_hace
#include <soc.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>
#include "hace_aspeed.h"
#include "hash_aspeed_priv.h"
#include "crypto_aspeed_priv.h"

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hace_global);

/* Device config */
struct aspeed_hace_config {
	uintptr_t base;			/* Hash and crypto engine base address */
	uintptr_t sbase;		/* Secure Boot engine base address */
	const struct device *clock_dev;
	const clock_control_subsys_t clk_id;
	const struct reset_dt_spec reset;
};

struct aspeed_hace_drv_state {
	struct aspeed_crypto_drv_state crypto_drv_state;
	struct aspeed_hash_drv_state hash_drv_state;
};

static struct aspeed_hace_drv_state hace_drv_state NON_CACHED_BSS_ALIGN16;

#define DEV_CFG(dev)					\
	((struct aspeed_hace_config *)			\
	 (dev)->config)

#define DEV_CRYPTO_DATA(dev)							\
	struct aspeed_hace_drv_state *drv_state;				\
	struct aspeed_crypto_drv_state *state;					\
										\
	drv_state = (struct aspeed_hace_drv_state *)(dev)->data;		\
	state = &drv_state->crypto_drv_state;

#define DEV_HASH_DATA(dev)							\
	struct aspeed_hace_drv_state *drv_state;				\
	struct aspeed_hash_drv_state *state;					\
										\
	drv_state = (struct aspeed_hace_drv_state *)(dev)->data;		\
	state = &drv_state->hash_drv_state;

#define HACE_CAPS_SUPPORT	(CAP_OPAQUE_KEY_HNDL | CAP_RAW_KEY |		\
				 CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

static int aspeed_crypto_wait_completion(struct device *dev, int timeout_ms)
{
	struct aspeed_hace_config *config = DEV_CFG(dev);
	struct hace_register_s *hace_register = (struct hace_register_s *)config->base;
	union hace_sts_s hace_sts;
	int ret;

	ret = reg_read_poll_timeout(hace_register, hace_sts, hace_sts,
				    hace_sts.fields.crypto_int, 1, timeout_ms);
	LOG_INF("HACE_STS: %x", hace_register->hace_sts.value);
	if (ret)
		LOG_ERR("HACE poll timeout");
	return ret;
}

static int crypto_trigger(struct device *dev, struct aspeed_crypto_ctx *data)
{
	struct aspeed_hace_config *config = DEV_CFG(dev);
	struct hace_register_s *hace_register = (struct hace_register_s *)config->base;

	if (hace_register->hace_sts.fields.crypto_engine_sts) {
		LOG_ERR("HACE error: engine busy");
		return -EBUSY;
	}

	hace_register->hace_sts.value = HACE_CRYPTO_ISR;

	hace_register->crypto_data_src.value = (uint32_t)&data->src_sg;
	hace_register->crypto_data_dst.value = (uint32_t)&data->dst_sg;
	hace_register->crypto_ctx_base.value = (uint32_t)data->ctx;
	hace_register->crypto_data_len.value = data->src_sg.len;
	hace_register->crypto_cmd_reg.value = data->cmd;

	LOG_INF("crypto_data_src: %x", (uint32_t)&data->src_sg);
	LOG_INF("crypto_data_dst: %x", (uint32_t)&data->dst_sg);
	LOG_INF("crypto_ctx_base: %x", (uint32_t)data->ctx);
	LOG_INF("crypto_data_len: %x", data->src_sg.len);
	LOG_INF("crypto_cmd_reg:  %x", data->cmd);

	return aspeed_crypto_wait_completion(dev, 3000);
}

static int aspeed_aes_crypt(struct cipher_ctx *ctx, unsigned char *in_buf,
			    int in_len, unsigned char *out_buf, int out_len)
{
	DEV_CRYPTO_DATA(ctx->device);
	struct aspeed_hace_config *config = DEV_CFG(ctx->device);
	struct aspeed_crypto_ctx *data = &state->data;
	int ret;

	if (ctx->flags & CAP_RAW_KEY) {
		memcpy(data->ctx + 16, ctx->key.bit_stream, ctx->keylen);

	} else { /*use secret vault key*/
		uint8_t key_id = *((uint8_t *)ctx->key.handle);

		if (key_id == 1) {
			SELECT_VAL_KEY_1(config->sbase);
		} else if (key_id == 2) {
			SELECT_VAL_KEY_2(config->sbase);
		} else {
			LOG_ERR("key_id %x is invalid", key_id);
			return -EINVAL;
		}
		data->cmd |= HACE_CMD_AES_KEY_FROM_OTP;
	}

	data->src_sg.addr = (uint32_t)in_buf;
	data->dst_sg.addr = (uint32_t)out_buf;
	data->src_sg.len = in_len | BIT(31);
	data->dst_sg.len = in_len | BIT(31);

	ret = crypto_trigger((struct device *)ctx->device, data);

	if (ret)
		return ret;

	cache_data_invd_all();

	return 0;
}

static int aspeed_des_crypt(struct cipher_ctx *ctx, unsigned char *in_buf,
			    int in_len, unsigned char *out_buf, int out_len)
{
	DEV_CRYPTO_DATA(ctx->device);
	struct aspeed_crypto_ctx *data = &state->data;
	int ret;

	/* Copy DES key to 0x10 offset */
	memcpy(data->ctx + 16, ctx->key.bit_stream, ctx->keylen);

	data->src_sg.addr = (uint32_t)in_buf;
	data->dst_sg.addr = (uint32_t)out_buf;
	data->src_sg.len = in_len | BIT(31);
	data->dst_sg.len = in_len | BIT(31);

	ret = crypto_trigger((struct device *)ctx->device, data);

	if (ret)
		return ret;

	cache_data_invd_all();

	return 0;
}

static int aspeed_aes_crypt_ecb(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	DEV_CRYPTO_DATA(ctx->device);

	pkt->out_len = pkt->in_len;

	return aspeed_aes_crypt(ctx, pkt->in_buf, pkt->in_len,
				pkt->out_buf, pkt->out_buf_max);
}

static int aspeed_aes_encrypt_cbc(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				  uint8_t *iv)
{
	DEV_CRYPTO_DATA(ctx->device);
	struct aspeed_crypto_ctx *data = &state->data;

	memcpy(data->ctx, iv, 16);
	memcpy(pkt->out_buf, iv, 16);
	pkt->out_len = pkt->in_len + 16;

	return aspeed_aes_crypt(ctx, pkt->in_buf, pkt->in_len,
				pkt->out_buf + 16, pkt->out_buf_max);
}

static int aspeed_aes_decrypt_cbc(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				  uint8_t *iv)
{
	DEV_CRYPTO_DATA(ctx->device);
	struct aspeed_crypto_ctx *data = &state->data;

	memcpy(data->ctx, iv, 16);
	pkt->out_len = pkt->in_len - 16;

	return aspeed_aes_crypt(ctx, pkt->in_buf + 16, pkt->in_len - 16,
				pkt->out_buf, pkt->out_buf_max);
}

static int aspeed_des_crypt_ecb(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	pkt->out_len = pkt->in_len;

	return aspeed_des_crypt(ctx, pkt->in_buf, pkt->in_len,
				pkt->out_buf, pkt->out_buf_max);
}

static int aspeed_des_encrypt_cbc(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				  uint8_t *iv)
{
	DEV_CRYPTO_DATA(ctx->device);
	struct aspeed_crypto_ctx *data = &state->data;

	memcpy(data->ctx + 8, iv, 8);
	memcpy(pkt->out_buf + 8, iv, 8);
	pkt->out_len = pkt->in_len + 16;

	return aspeed_des_crypt(ctx, pkt->in_buf, pkt->in_len,
				pkt->out_buf + 16, pkt->out_buf_max);
}

static int aspeed_des_decrypt_cbc(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				  uint8_t *iv)
{
	DEV_CRYPTO_DATA(ctx->device);
	struct aspeed_crypto_ctx *data = &state->data;

	memcpy(data->ctx + 8, iv, 8);
	pkt->out_len = pkt->in_len - 16;

	return aspeed_des_crypt(ctx, pkt->in_buf + 16, pkt->in_len - 16,
				pkt->out_buf, pkt->out_buf_max);
}

static int aspeed_crypto_session_setup(const struct device *dev,
				       struct cipher_ctx *ctx,
				       enum cipher_algo algo,
				       enum cipher_mode mode,
				       enum cipher_op op_type)
{
	DEV_CRYPTO_DATA(dev);
	struct aspeed_crypto_ctx *data = &state->data;
	cbc_op_t cbc_encrypt_handler = NULL;
	cbc_op_t cbc_decrypt_handler = NULL;

	LOG_INF("aspeed_crypto_session_setup");
	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	if (!(ctx->flags & CAP_SYNC_OPS)) {
		LOG_ERR("Async not supported by this driver");
		return -EINVAL;
	}

	data->cmd = HACE_CMD_DES_SG_CTRL | HACE_CMD_SRC_SG_CTRL |
			HACE_CMD_MBUS_REQ_SYNC_EN;

	switch (algo) {
	case CRYPTO_CIPHER_ALGO_AES:
		data->cmd |= HACE_CMD_AES_KEY_HW_EXP | HACE_CMD_AES_SELECT;
		cbc_encrypt_handler = aspeed_aes_encrypt_cbc;
		cbc_decrypt_handler = aspeed_aes_decrypt_cbc;
		break;
	case CRYPTO_CIPHER_ALGO_DES:
		data->cmd |= HACE_CMD_DES_SELECT;
		cbc_encrypt_handler = aspeed_des_encrypt_cbc;
		cbc_decrypt_handler = aspeed_des_decrypt_cbc;
		break;
	case CRYPTO_CIPHER_ALGO_TDES:
		data->cmd |= HACE_CMD_DES_SELECT | HACE_CMD_TRIPLE_DES;
		cbc_encrypt_handler = aspeed_des_encrypt_cbc;
		cbc_decrypt_handler = aspeed_des_decrypt_cbc;
		break;
	default:
		LOG_ERR("unsupported algorithm");
		return -EINVAL;
	}

	if (algo == CRYPTO_CIPHER_ALGO_AES) {
		switch (ctx->keylen) {
		case 16:
			data->cmd |= HACE_CMD_AES128;
			break;
		case 24:
			data->cmd |= HACE_CMD_AES192;
			break;
		case 32:
			data->cmd |= HACE_CMD_AES256;
			break;
		default:
			LOG_ERR("unsupported key size");
			return -EINVAL;
		}
	}

	switch (mode) {
	case CRYPTO_CIPHER_MODE_ECB:
		data->cmd |= HACE_CMD_ECB;
		if (op_type == CRYPTO_CIPHER_OP_ENCRYPT)
			data->cmd |= HACE_CMD_ENCRYPT;
		else
			data->cmd |= HACE_CMD_DECRYPT;
		if (data->cmd & HACE_CMD_DES_SELECT)
			ctx->ops.block_crypt_hndlr = aspeed_des_crypt_ecb;
		else
			ctx->ops.block_crypt_hndlr = aspeed_aes_crypt_ecb;
		break;
	case CRYPTO_CIPHER_MODE_CBC:
		data->cmd |= HACE_CMD_CBC;
		if (op_type == CRYPTO_CIPHER_OP_ENCRYPT) {
			data->cmd |= HACE_CMD_ENCRYPT;
			ctx->ops.cbc_crypt_hndlr = cbc_encrypt_handler;
		} else {
			data->cmd |= HACE_CMD_DECRYPT;
			ctx->ops.cbc_crypt_hndlr = cbc_decrypt_handler;
		}
		break;
	case CRYPTO_CIPHER_MODE_CFB:
		data->cmd |= HACE_CMD_CFB;
		if (op_type == CRYPTO_CIPHER_OP_ENCRYPT) {
			data->cmd |= HACE_CMD_ENCRYPT;
			ctx->ops.cbc_crypt_hndlr = cbc_encrypt_handler;
		} else {
			data->cmd |= HACE_CMD_DECRYPT;
			ctx->ops.cbc_crypt_hndlr = cbc_decrypt_handler;
		}
		break;
	case CRYPTO_CIPHER_MODE_OFB:
		data->cmd |= HACE_CMD_OFB;
		if (op_type == CRYPTO_CIPHER_OP_ENCRYPT) {
			data->cmd |= HACE_CMD_ENCRYPT;
			ctx->ops.cbc_crypt_hndlr = cbc_encrypt_handler;
		} else {
			data->cmd |= HACE_CMD_DECRYPT;
			ctx->ops.cbc_crypt_hndlr = cbc_decrypt_handler;
		}
		break;
	case CRYPTO_CIPHER_MODE_CTR:
		data->cmd |= HACE_CMD_CTR;
		if (op_type == CRYPTO_CIPHER_OP_ENCRYPT) {
			data->cmd |= HACE_CMD_ENCRYPT;
			ctx->ops.cbc_crypt_hndlr = cbc_encrypt_handler;
		} else {
			data->cmd |= HACE_CMD_DECRYPT;
			ctx->ops.cbc_crypt_hndlr = cbc_decrypt_handler;
		}
		break;
	default:
		LOG_ERR("unsupported mode");
		return -EINVAL;

	}
	LOG_INF("data->cmd: %x", data->cmd);
	state->in_use = true;
	ctx->ops.cipher_mode = mode;
	ctx->device = dev;

	return 0;
}

static int aspeed_crypto_session_free(const struct device *dev, struct cipher_ctx *ctx)
{
	DEV_CRYPTO_DATA(dev);
	ARG_UNUSED(ctx);
	LOG_INF("aspeed_crypto_session_free");
	state->in_use = false;

	return 0;
}

static int aspeed_hash_wait_completion(struct device *dev, int timeout_ms)
{
	struct aspeed_hace_config *config = DEV_CFG(dev);
	struct hace_register_s *hace_register = (struct hace_register_s *)config->base;
	union hace_sts_s hace_sts;
	int ret;

	ret = reg_read_poll_timeout(hace_register, hace_sts, hace_sts,
				    hace_sts.fields.hash_int, 1, timeout_ms);
	if (ret)
		LOG_ERR("HACE poll timeout\n");

	return ret;
}

static void aspeed_ahash_fill_padding(struct aspeed_hash_ctx *ctx,
				      unsigned int remainder)
{
	unsigned int index, padlen;
	uint64_t bits[2];

	if (ctx->block_size == 64) {
		bits[0] = sys_cpu_to_be64(ctx->digcnt[0] << 3);
		index = (ctx->bufcnt + remainder) & 0x3f;
		padlen = (index < 56) ? (56 - index) : ((64 + 56) - index);
		*(ctx->buffer + ctx->bufcnt) = 0x80;
		memset(ctx->buffer + ctx->bufcnt + 1, 0, padlen - 1);
		memcpy(ctx->buffer + ctx->bufcnt + padlen, bits, 8);
		ctx->bufcnt += padlen + 8;

	} else {
		bits[1] = sys_cpu_to_be64(ctx->digcnt[0] << 3);
		bits[0] = sys_cpu_to_be64(ctx->digcnt[1] << 3 | ctx->digcnt[0] >> 61);
		index = (ctx->bufcnt + remainder) & 0x7f;
		padlen = (index < 112) ? (112 - index) : ((128 + 112) - index);
		*(ctx->buffer + ctx->bufcnt) = 0x80;
		memset(ctx->buffer + ctx->bufcnt + 1, 0, padlen - 1);
		memcpy(ctx->buffer + ctx->bufcnt + padlen, bits, 16);
		ctx->bufcnt += padlen + 16;
	}
}

static int hash_trigger(struct device *dev, struct aspeed_hash_ctx *data, int len)
{
	struct aspeed_hace_config *config = DEV_CFG(dev);
	struct hace_register_s *hace_register = (struct hace_register_s *)config->base;
	int ret;

	if (hace_register->hace_sts.fields.hash_engine_sts) {
		LOG_ERR("HACE error: engine busy\n");
		return -EBUSY;
	}
	/* Clear pending completion status */
	hace_register->hace_sts.value = HACE_HASH_ISR;

	if (data->method & HACE_SG_EN)
		hace_register->hash_data_src.value = (uint32_t)data->sg;
	else
		hace_register->hash_data_src.value = (uint32_t)data->buffer;

	hace_register->hash_dgst_dst.value = (uint32_t)data->digest;
	hace_register->hash_key_buf.value = (uint32_t)data->digest;

	hace_register->hash_data_len.value = len;
	hace_register->hash_cmd_reg.value = data->method;

	ret = aspeed_hash_wait_completion(dev, 3000);

	cache_data_invd_range(data->digest, 64);

	return ret;
}

static int aspeed_hash_update(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	DEV_HASH_DATA(ctx->device);
	struct aspeed_hash_ctx *data = &state->data;
	struct aspeed_sg *sg = data->sg;
	int rc;
	int remainder;
	int total_len;
	int i;

	data->digcnt[0] += pkt->in_len;
	if (data->digcnt[0] < pkt->in_len)
		data->digcnt[1]++;

	if (data->bufcnt + pkt->in_len < data->block_size) {
		memcpy(data->buffer + data->bufcnt, pkt->in_buf, pkt->in_len);
		data->bufcnt += pkt->in_len;
		return 0;
	}
	remainder = (pkt->in_len + data->bufcnt) % data->block_size;
	total_len = pkt->in_len + data->bufcnt - remainder;
	i = 0;
	if (data->bufcnt != 0) {
		sg[0].addr = (uint32_t)data->buffer;
		sg[0].len = data->bufcnt;
		if (total_len == data->bufcnt)
			sg[0].len |= HACE_SG_LAST;
		i++;
	}

	if (total_len != data->bufcnt) {
		sg[i].addr = (uint32_t)pkt->in_buf;
		sg[i].len = (total_len - data->bufcnt) | HACE_SG_LAST;
	}

	rc = hash_trigger((struct device *)ctx->device, data, total_len);
	if (remainder != 0) {
		memcpy(data->buffer, pkt->in_buf + (total_len - data->bufcnt), remainder);
		data->bufcnt = remainder;
	}

	return rc;
}

static int aspeed_hash_final(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	DEV_HASH_DATA(ctx->device);
	struct aspeed_hash_ctx *data = &state->data;
	struct aspeed_sg *sg = data->sg;
	int rc;

	aspeed_ahash_fill_padding(data, 0);

	sg[0].addr = (uint32_t)data->buffer;
	sg[0].len = data->bufcnt | HACE_SG_LAST;

	rc = hash_trigger((struct device *)ctx->device, data, data->bufcnt);
	if (rc) {
		return rc;
	}
	memcpy(pkt->out_buf, data->digest, ctx->digest_size);

	return 0;
}

static int aspeed_hash_compute(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	if (finish)
		return aspeed_hash_final(ctx, pkt);
	else
		return aspeed_hash_update(ctx, pkt);
}

static int aspeed_hash_digest_hmac(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	DEV_HASH_DATA(ctx->device);
	struct aspeed_hash_ctx *data = &state->data;
	int bs = data->block_size;
	int ds = ctx->digest_size;
	int len;
	int rc;

	/* H(ipad + message) */
	data->digcnt[0] = bs;
	data->bufcnt = bs;
	memcpy(data->buffer, data->hmac_data.ipad, bs);

	len = data->bufcnt + pkt->in_len;
	if (len > HASH_TMP_BUFF_SIZE) {
		LOG_ERR("%s: data buffer Out-of-Range, bufcnt:0x%x, in_len:0x%x\n",
			__func__, data->bufcnt, pkt->in_len);
		return -EINVAL;
	}

	memcpy(data->buffer + data->bufcnt, pkt->in_buf, pkt->in_len);
	data->digcnt[0] += pkt->in_len;
	data->bufcnt += pkt->in_len;

	aspeed_ahash_fill_padding(data, 0);

	/* Use Initial Vector */
	memcpy(data->digest, data->iv, data->iv_size);

	/* Direct Access Mode / ACC Mode */
	data->method &= ~(HACE_SG_EN);
	rc = hash_trigger((struct device *)ctx->device, data, data->bufcnt);
	if (rc) {
		LOG_ERR("%s: hash 1 failed, rc=%d\n", __func__, rc);
		goto end;
	}

	/* H(opad + hash sum 1) */
	data->digcnt[0] = bs + ds;
	data->bufcnt = bs + ds;

	memcpy(data->buffer, data->hmac_data.opad, bs);
	memcpy(data->buffer + bs, data->digest, ds);
	len = data->bufcnt;

	aspeed_ahash_fill_padding(data, 0);

	/* Use Initial Vector */
	memcpy(data->digest, data->iv, data->iv_size);

	rc = hash_trigger((struct device *)ctx->device, data, data->bufcnt);
	if (rc) {
		LOG_ERR("%s: hash 2 failed, rc=%d\n", __func__, rc);
		goto end;
	}

	memcpy(pkt->out_buf, data->digest, ds);

end:
	return rc;
}

static int aspeed_hash_digest(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	DEV_HASH_DATA(ctx->device);
	struct aspeed_hash_ctx *data = &state->data;
	int len = pkt->in_len;
	int rc;

	/* Copy input data into buffer */
	memcpy(data->buffer, pkt->in_buf, len);
	data->digcnt[0] = len;
	data->bufcnt = len;

	/* Use Initial Vector */
	memcpy(data->digest, data->iv, data->iv_size);

	aspeed_ahash_fill_padding(data, 0);

	/* Direct Access Mode / ACC Mode */
	data->method &= ~(HACE_SG_EN);
	rc = hash_trigger((struct device *)ctx->device, data, data->bufcnt);
	if (rc) {
		LOG_ERR("%s: failed, rc=%d\n", __func__, rc);
		return rc;
	}

	memcpy(pkt->out_buf, data->digest, ctx->digest_size);

	return 0;
}

static int aspeed_hash_setkey(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	DEV_HASH_DATA(ctx->device);
	struct aspeed_hash_ctx *data = &state->data;
	struct hash_pkt pkt_key;
	int bs = data->block_size;
	int ds = ctx->digest_size;
	int rc;

	if (pkt->key_len > bs) {
		/* Do H(K) first */
		pkt_key.in_buf = pkt->key_buf;
		pkt_key.in_len = pkt->key_len;
		pkt_key.out_buf = data->hmac_data.key_buff;
		rc = aspeed_hash_digest(ctx, &pkt_key);
		if (rc) {
			printk("aspeed_hash_digest() failed, rc:%d\n", rc);
			goto end;
		}

		pkt->key_len = ds;

	} else {
		memcpy(data->hmac_data.key_buff, pkt->key_buf, pkt->key_len);
	}

	memset(data->hmac_data.key_buff + pkt->key_len, 0, bs - pkt->key_len);

	memcpy(data->hmac_data.ipad, data->hmac_data.key_buff, bs);
	memcpy(data->hmac_data.opad, data->hmac_data.key_buff, bs);

	for (int i = 0; i < bs; i++) {
		data->hmac_data.ipad[i] ^= HMAC_IPAD_VALUE;
		data->hmac_data.opad[i] ^= HMAC_OPAD_VALUE;
	}

	rc = 0;

end:
	return rc;
}

static int aspeed_hash_session_setup(const struct device *dev,
				     struct hash_ctx *ctx,
				     enum hash_algo algo)
{
	DEV_HASH_DATA(dev);
	struct aspeed_hash_ctx *data;

	ARG_UNUSED(dev);

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	data = &state->data;
	state->in_use = true;

	data->method = HASH_CMD_ACC_MODE | HACE_SHA_BE_EN | HACE_SG_EN;
	switch (algo) {
	case CRYPTO_HASH_ALGO_SHA1:
		ctx->digest_size = SHA1_DIGEST_SIZE;
		data->block_size = SHA1_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA1;
		data->iv = (uint32_t *)sha1_iv;
		data->iv_size = SHA1_IV_SIZE;
		break;
	case CRYPTO_HASH_ALGO_SHA224:
		ctx->digest_size = SHA224_DIGEST_SIZE;
		data->block_size = SHA224_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA224;
		data->iv = (uint32_t *)sha224_iv;
		data->iv_size = SHA224_IV_SIZE;
		break;
	case CRYPTO_HASH_ALGO_SHA256:
		ctx->digest_size = SHA256_DIGEST_SIZE;
		data->block_size = SHA256_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA256;
		data->iv = (uint32_t *)sha256_iv;
		data->iv_size = SHA256_IV_SIZE;
		break;
	case CRYPTO_HASH_ALGO_SHA384:
		ctx->digest_size = SHA384_DIGEST_SIZE;
		data->block_size = SHA384_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA384;
		data->iv = (uint32_t *)sha384_iv;
		data->iv_size = SHA384_IV_SIZE;
		break;
	case CRYPTO_HASH_ALGO_SHA512:
		ctx->digest_size = SHA512_DIGEST_SIZE;
		data->block_size = SHA512_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA512;
		data->iv = (uint32_t *)sha512_iv;
		data->iv_size = SHA512_IV_SIZE;
		break;
	case CRYPTO_HASH_ALGO_SHA512_224:
		ctx->digest_size = SHA224_DIGEST_SIZE;
		data->block_size = SHA512_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA512_224;
		data->iv = (uint32_t *)sha512_224_iv;
		data->iv_size = SHA512_IV_SIZE;
		break;
	case CRYPTO_HASH_ALGO_SHA512_256:
		ctx->digest_size = SHA256_DIGEST_SIZE;
		data->block_size = SHA512_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA512_256;
		data->iv = (uint32_t *)sha512_256_iv;
		data->iv_size = SHA512_IV_SIZE;
		break;
	default:
		LOG_ERR("ASPEED HASH Unsupported mode");
		return -EINVAL;
	}

	ctx->ops.setkey_hndlr = aspeed_hash_setkey;
	ctx->ops.digest_hmac_hndlr = aspeed_hash_digest_hmac;
	ctx->hash_hndlr = aspeed_hash_compute;
	ctx->device = dev;

	memcpy(data->digest, data->iv, data->iv_size);

	data->bufcnt = 0;
	data->digcnt[0] = 0;
	data->digcnt[1] = 0;

	return 0;
}

static int aspeed_hash_session_free(const struct device *dev,
				    struct hash_ctx *ctx)
{
	DEV_HASH_DATA(dev);
	struct aspeed_hace_config *config = DEV_CFG(dev);
	struct hace_register_s *hace_register = (struct hace_register_s *)config->base;
	struct aspeed_hash_ctx *data = &state->data;

	ARG_UNUSED(ctx);
	hace_register->hash_cmd_reg.value = 0x0;
	state->in_use = false;
	memset(data->buffer, 0, HASH_TMP_BUFF_SIZE);
	data->bufcnt = 0;
	data->digcnt[0] = 0;
	data->digcnt[1] = 0;

	return 0;
}

static int aspeed_hace_query_caps(const struct device *dev)
{
	return HACE_CAPS_SUPPORT;
}

/* Crypto controller driver registration */
static int hace_init(const struct device *dev)
{
	const struct aspeed_hace_config *config = DEV_CFG(dev);

	reset_line_assert_dt(&config->reset);

	k_usleep(100);
	clock_control_on(config->clock_dev, DEV_CFG(dev)->clk_id);
	k_msleep(10);

	reset_line_deassert_dt(&config->reset);

	return 0;
}

static const struct aspeed_hace_config hace_config = {
	.base = DT_REG_ADDR_BY_IDX(DT_NODELABEL(hace), 0),
	.sbase = DT_REG_ADDR_BY_IDX(DT_NODELABEL(hace), 1),
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, clk_id),
	.reset = RESET_DT_SPEC_INST_GET(0),
};

static struct crypto_driver_api crypto_funcs = {
	.cipher_begin_session = aspeed_crypto_session_setup,
	.cipher_free_session = aspeed_crypto_session_free,
	.cipher_async_callback_set = NULL,
	.hash_begin_session = aspeed_hash_session_setup,
	.hash_free_session = aspeed_hash_session_free,
	.query_hw_caps = aspeed_hace_query_caps,
};

#define ASPEED_HACE_INIT(inst)								\
	DEVICE_DT_INST_DEFINE(inst, hace_init, NULL, &hace_drv_state, &hace_config,	\
			      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,			\
			      (void *)&crypto_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_HACE_INIT)
