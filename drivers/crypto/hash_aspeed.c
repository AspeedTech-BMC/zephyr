/*
 * Copyright (c) 2021 ASPEED Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <sys/__assert.h>
#include <crypto/hash.h>
#include <logging/log.h>
#include <sys/byteorder.h>

#include "hace_aspeed.h"
#include "hash_aspeed_priv.h"

LOG_MODULE_DECLARE(hace_global, CONFIG_CRYPTO_LOG_LEVEL);

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

static struct aspeed_hash_drv_state drv_state NON_CACHED_BSS_ALIGN16;

static int aspeed_hash_wait_completion(int timeout_ms)
{
	struct hace_register_s *hace_register = hace_eng.base;
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

static int hash_trigger(struct aspeed_hash_ctx *data, int len)
{
	struct hace_register_s *hace_register = hace_eng.base;

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

	return aspeed_hash_wait_completion(3000);
}

static int aspeed_hash_update(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	struct aspeed_hash_ctx *data = &drv_state.data;
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

	rc = hash_trigger(data, total_len);
	if (remainder != 0) {
		memcpy(data->buffer, pkt->in_buf + (total_len - data->bufcnt), remainder);
		data->bufcnt = remainder;
	}
	if (rc)
		return rc;

	return 0;
}

static int aspeed_hash_final(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	struct aspeed_hash_ctx *data = &drv_state.data;
	struct aspeed_sg *sg = data->sg;
	int rc;

	if (pkt->out_buf_max < ctx->digest_size) {
		LOG_ERR("HACE error: insufficient size on destination buffer\n");
		return -EINVAL;
	}
	aspeed_ahash_fill_padding(data, 0);

	sg[0].addr = (uint32_t)data->buffer;
	sg[0].len = data->bufcnt | HACE_SG_LAST;

	rc = hash_trigger(data, data->bufcnt);
	if (rc) {
		return rc;
	}
	memcpy(pkt->out_buf, data->digest, ctx->digest_size);

	return 0;
}

static int aspeed_hash_digest_hmac(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	struct aspeed_hash_ctx *data = &drv_state.data;
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
	rc = hash_trigger(data, data->bufcnt);
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

	rc = hash_trigger(data, data->bufcnt);
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
	struct aspeed_hash_ctx *data = &drv_state.data;
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
	rc = hash_trigger(data, data->bufcnt);
	if (rc) {
		LOG_ERR("%s: failed, rc=%d\n", __func__, rc);
		return rc;
	}

	memcpy(pkt->out_buf, data->digest, ctx->digest_size);

	return 0;
}

static int aspeed_hash_setkey(struct hash_ctx *ctx, struct hash_pkt *pkt)
{
	struct aspeed_hash_ctx *data = &drv_state.data;
	struct hash_pkt pkt_key;
	int bs = data->block_size;
	int ds = ctx->digest_size;
	int rc;

	if (pkt->key_len > bs) {
		/* Do H(K) first */
		pkt_key.in_buf = pkt->key_buf;
		pkt_key.in_len = pkt->key_len;
		pkt_key.out_buf = data->hmac_data.key_buff;
		pkt_key.out_buf_max = pkt->out_buf_max;
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

static int aspeed_hash_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	drv_state.in_use = false;

	return 0;
}

static int aspeed_hash_session_setup(const struct device *dev,
				     struct hash_ctx *ctx,
				     enum hash_algo algo)
{
	struct aspeed_hash_ctx *data;

	ARG_UNUSED(dev);

	if (drv_state.in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	data = &drv_state.data;
	drv_state.in_use = true;

	data->method = HASH_CMD_ACC_MODE | HACE_SHA_BE_EN | HACE_SG_EN;
	switch (algo) {
	case HASH_SHA1:
		ctx->digest_size = SHA1_DIGEST_SIZE;
		data->block_size = SHA1_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA1;
		data->iv = (uint32_t *)sha1_iv;
		data->iv_size = SHA1_IV_SIZE;
		break;
	case HASH_SHA224:
		ctx->digest_size = SHA224_DIGEST_SIZE;
		data->block_size = SHA224_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA224;
		data->iv = (uint32_t *)sha224_iv;
		data->iv_size = SHA224_IV_SIZE;
		break;
	case HASH_SHA256:
		ctx->digest_size = SHA256_DIGEST_SIZE;
		data->block_size = SHA256_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA256;
		data->iv = (uint32_t *)sha256_iv;
		data->iv_size = SHA256_IV_SIZE;
		break;
	case HASH_SHA384:
		ctx->digest_size = SHA384_DIGEST_SIZE;
		data->block_size = SHA384_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA384;
		data->iv = (uint32_t *)sha384_iv;
		data->iv_size = SHA384_IV_SIZE;
		break;
	case HASH_SHA512:
		ctx->digest_size = SHA512_DIGEST_SIZE;
		data->block_size = SHA512_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA512;
		data->iv = (uint32_t *)sha512_iv;
		data->iv_size = SHA512_IV_SIZE;
		break;
	case HASH_SHA512_224:
		ctx->digest_size = SHA224_DIGEST_SIZE;
		data->block_size = SHA512_BLOCK_SIZE;
		data->method |= HACE_ALGO_SHA512_224;
		data->iv = (uint32_t *)sha512_224_iv;
		data->iv_size = SHA512_IV_SIZE;
		break;
	case HASH_SHA512_256:
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
	ctx->ops.update_hndlr = aspeed_hash_update;
	ctx->ops.final_hndlr = aspeed_hash_final;

	memcpy(data->digest, data->iv, data->iv_size);

	data->bufcnt = 0;
	data->digcnt[0] = 0;
	data->digcnt[1] = 0;

	return 0;
}

static int aspeed_hash_session_free(const struct device *dev,
				    struct hash_ctx *ctx)
{
	struct hace_register_s *hace_register = hace_eng.base;
	struct aspeed_hash_ctx *data = &drv_state.data;

	ARG_UNUSED(dev);
	ARG_UNUSED(ctx);

	hace_register->hash_cmd_reg.value = 0x0;
	drv_state.in_use = false;
	memset(data->buffer, 0, HASH_TMP_BUFF_SIZE);
	data->bufcnt = 0;
	data->digcnt[0] = 0;
	data->digcnt[1] = 0;

	return 0;
}

static struct hash_driver_api hash_funcs = {
	.begin_session = aspeed_hash_session_setup,
	.free_session = aspeed_hash_session_free,
	.query_hw_caps = NULL,
};

DEVICE_DEFINE(hash_aspeed, CONFIG_CRYPTO_ASPEED_HASH_DRV_NAME,
			  aspeed_hash_init, NULL, NULL, NULL,
			  POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,
			  (void *)&hash_funcs);
