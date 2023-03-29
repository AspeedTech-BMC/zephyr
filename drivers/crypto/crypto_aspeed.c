/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <kernel.h>
#include <string.h>
#include <device.h>
#include <drivers/i2c.h>
#include <sys/__assert.h>
#include <crypto/cipher.h>
#include <logging/log.h>
#include <cache.h>

#include "hace_aspeed.h"
#include "crypto_aspeed_priv.h"

LOG_MODULE_DECLARE(hace_global, CONFIG_CRYPTO_LOG_LEVEL);

static struct aspeed_crypto_drv_state drv_state NON_CACHED_BSS_ALIGN16;

static int aspeed_crypto_wait_completion(int timeout_ms)
{
	struct hace_register_s *hace_register = hace_eng.base;
	union hace_sts_s hace_sts;
	int ret;

	ret = reg_read_poll_timeout(hace_register, hace_sts, hace_sts,
				    hace_sts.fields.crypto_int, 100, timeout_ms);
	LOG_INF("HACE_STS: %x", hace_register->hace_sts.value);
	if (ret)
		LOG_ERR("HACE poll timeout");
	return ret;
}

static int crypto_trigger(struct aspeed_crypto_ctx *data)
{
	struct hace_register_s *hace_register = hace_eng.base;

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

	return aspeed_crypto_wait_completion(3000);
}

static int aspeed_aes_crypt(struct cipher_ctx *ctx, unsigned char *in_buf,
			    int in_len, unsigned char *out_buf, int out_len)
{
	struct aspeed_crypto_ctx *data = &drv_state.data;
	int ret;

	if (ctx->flags & CAP_RAW_KEY) {
		memcpy(data->ctx + 16, ctx->key.bit_stream, ctx->keylen);

	} else { /*use secret vault key*/
		uint8_t key_id = *((uint8_t *)ctx->key.handle);

		if (key_id == 1) {
			SELECT_VAL_KEY_1();
		} else if (key_id == 2) {
			SELECT_VAL_KEY_2();
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

	ret = crypto_trigger(data);

	if (ret)
		return ret;

	cache_data_range(out_buf, out_len, K_CACHE_INVD);

	return 0;
}

static int aspeed_des_crypt(struct cipher_ctx *ctx, unsigned char *in_buf,
			    int in_len, unsigned char *out_buf, int out_len)
{
	struct aspeed_crypto_ctx *data = &drv_state.data;
	int ret;

	/* Copy DES key to 0x10 offset */
	memcpy(data->ctx + 16, ctx->key.bit_stream, ctx->keylen);

	data->src_sg.addr = (uint32_t)in_buf;
	data->dst_sg.addr = (uint32_t)out_buf;
	data->src_sg.len = in_len | BIT(31);
	data->dst_sg.len = in_len | BIT(31);

	ret = crypto_trigger(data);

	if (ret)
		return ret;

	cache_data_range(out_buf, out_len, K_CACHE_INVD);

	return 0;
}

static int aspeed_aes_crypt_ecb(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	pkt->out_len = pkt->in_len;

	return aspeed_aes_crypt(ctx, pkt->in_buf, pkt->in_len,
				pkt->out_buf, pkt->out_buf_max);
}

static int aspeed_aes_encrypt_cbc(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				  uint8_t *iv)
{
	struct aspeed_crypto_ctx *data = &drv_state.data;

	memcpy(data->ctx, iv, 16);
	memcpy(pkt->out_buf, iv, 16);
	pkt->out_len = pkt->in_len + 16;

	return aspeed_aes_crypt(ctx, pkt->in_buf, pkt->in_len,
				pkt->out_buf + 16, pkt->out_buf_max);
}

static int aspeed_aes_decrypt_cbc(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				  uint8_t *iv)
{
	struct aspeed_crypto_ctx *data = &drv_state.data;

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
	struct aspeed_crypto_ctx *data = &drv_state.data;

	memcpy(data->ctx + 8, iv, 8);
	memcpy(pkt->out_buf + 8, iv, 8);
	pkt->out_len = pkt->in_len + 16;

	return aspeed_des_crypt(ctx, pkt->in_buf, pkt->in_len,
				pkt->out_buf + 16, pkt->out_buf_max);
}

static int aspeed_des_decrypt_cbc(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				  uint8_t *iv)
{
	struct aspeed_crypto_ctx *data = &drv_state.data;

	memcpy(data->ctx + 8, iv, 8);
	pkt->out_len = pkt->in_len - 16;

	return aspeed_des_crypt(ctx, pkt->in_buf + 16, pkt->in_len - 16,
				pkt->out_buf, pkt->out_buf_max);
}

static int aspeed_crypto_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	drv_state.in_use = false;

	return 0;
}

static int aspeed_crypto_session_setup(const struct device *dev,
				       struct cipher_ctx *ctx,
				       enum cipher_algo algo, enum cipher_mode mode,
				       enum cipher_op op_type)
{
	struct aspeed_crypto_ctx *data;
	cbc_op_t cbc_encrypt_handler = NULL;
	cbc_op_t cbc_decrypt_handler = NULL;

	ARG_UNUSED(dev);
	LOG_INF("aspeed_crypto_session_setup");
	if (drv_state.in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	if (!(ctx->flags & CAP_SYNC_OPS)) {
		LOG_ERR("Async not supported by this driver");
		return -EINVAL;
	}

	data = &drv_state.data;

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
	drv_state.in_use = true;
	ctx->ops.cipher_mode = mode;

	return 0;
}

static int aspeed_crypto_session_free(const struct device *dev, struct cipher_ctx *ctx)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ctx);

	LOG_INF("aspeed_crypto_session_free");
	drv_state.in_use = false;

	return 0;
}

static int aspeed_crypto_query_caps(const struct device *dev)
{
	return (CAP_OPAQUE_KEY_HNDL | CAP_RAW_KEY |
			CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS);
}

static struct crypto_driver_api crypto_funcs = {
	.begin_session = aspeed_crypto_session_setup,
	.free_session = aspeed_crypto_session_free,
	.crypto_async_callback_set = NULL,
	.query_hw_caps = aspeed_crypto_query_caps,
};

DEVICE_DEFINE(crypto_aspeed, CONFIG_CRYPTO_ASPEED_DRV_NAME,
			  aspeed_crypto_init, NULL, NULL, NULL,
			  POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,
			  (void *)&crypto_funcs);
