/*
 * Copyright (c) 2024 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: MIT
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#ifdef CONFIG_ECDSA_ASPEED
#define ECDSA_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_ecdsa))
#elif CONFIG_CPTRA_ECDSA
#define ECDSA_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_ecdsa))
#define HASH_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_sha))
#endif

#include <zephyr/crypto/ecdsa.h>
#include <zephyr/crypto/ecdsa_structs.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/hash.h>
#include <crypto.h>
#include "mbedtls/ecdsa.h"
#include "mbedtls/error.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(crypto, CONFIG_CRYPTO_LOG_LEVEL);

static void dump_pubkey(const char *title, mbedtls_ecdsa_context *key)
{
	unsigned char buf[300];
	size_t len;

	if (mbedtls_ecp_point_write_binary(&key->MBEDTLS_PRIVATE(grp), &key->MBEDTLS_PRIVATE(Q),
					   MBEDTLS_ECP_PF_UNCOMPRESSED, &len, buf,
					   sizeof(buf)) != 0) {
		LOG_DBG("internal error\n");
		return;
	}
	LOG_HEXDUMP_DBG(buf, len, title);

	mbedtls_mpi_write_binary(&key->MBEDTLS_PRIVATE(Q).MBEDTLS_PRIVATE(X), buf, 48);
	LOG_HEXDUMP_DBG(buf, 48, "  + qx: ");
	mbedtls_mpi_write_binary(&key->MBEDTLS_PRIVATE(Q).MBEDTLS_PRIVATE(Y), buf, 48);
	LOG_HEXDUMP_DBG(buf, 48, "  + qy: ");
	mbedtls_mpi_write_binary(&key->MBEDTLS_PRIVATE(Q).MBEDTLS_PRIVATE(Z), buf, 48);
	LOG_HEXDUMP_DBG(buf, 48, "  + qz: ");
}

static int mbedtls_ecdsa_test(const struct ecdsa_testvec *tv, int tv_size)
{
	mbedtls_ecdsa_context ctx_verify;
	unsigned char hash[48];
	mbedtls_mpi r, s;
	int i, ret = 0, rc = 0;
	char z = 1;

	LOG_INF("Start...");
	for (i = 0; i < tv_size; i++) {
		mbedtls_ecdsa_init(&ctx_verify);
		mbedtls_mpi_init(&r);
		mbedtls_mpi_init(&s);

		mbedtls_mpi_read_binary(&ctx_verify.MBEDTLS_PRIVATE(Q).MBEDTLS_PRIVATE(X),
					tv[i].qx, 48);
		mbedtls_mpi_read_binary(&ctx_verify.MBEDTLS_PRIVATE(Q).MBEDTLS_PRIVATE(Y),
					tv[i].qy, 48);
		mbedtls_mpi_read_binary(&ctx_verify.MBEDTLS_PRIVATE(Q).MBEDTLS_PRIVATE(Z), &z, 1);
		mbedtls_mpi_read_binary(&r, tv[i].r, 48);
		mbedtls_mpi_read_binary(&s, tv[i].s, 48);

		LOG_DBG(" start load curve");
		mbedtls_ecp_group_load(&ctx_verify.MBEDTLS_PRIVATE(grp), MBEDTLS_ECP_DP_SECP384R1);

		dump_pubkey("  + Public key: ", &ctx_verify);

		LOG_DBG(" signature");
		LOG_HEXDUMP_DBG(tv[i].r, 48, "  + r: ");
		LOG_HEXDUMP_DBG(tv[i].s, 48, "  + s: ");

		memcpy(hash, tv[i].m, tv[i].m_size);

		LOG_DBG(" start verify");
		ret = mbedtls_ecdsa_verify(&ctx_verify.MBEDTLS_PRIVATE(grp), hash, tv[i].m_size,
					   &ctx_verify.MBEDTLS_PRIVATE(Q), &r, &s);
		if (ret) {
#if defined(MBEDTLS_ERROR_C)
			unsigned char tmp[200];

			mbedtls_strerror(ret, (char *)tmp, sizeof(tmp));
			LOG_DBG(" Failed: %s", tmp);
#else
			LOG_DBG(" Failed: %d\n", ret);
#endif
		}

		if ((ret && !tv[i].result) || (ret == 0 && tv[i].result))
			LOG_DBG(" Test Pass\n");
		else {
			LOG_DBG(" Test Failed\n");
			rc = -1;
		}

		mbedtls_ecdsa_free(&ctx_verify);
		mbedtls_mpi_free(&r);
		mbedtls_mpi_free(&s);

		if (rc) {
			LOG_INF("Failure");
			return rc;
		}
	}

	LOG_INF("Pass");

	return 0;
}

#ifdef CONFIG_CPTRA_ECDSA
static int hw_gen_sha(uint8_t *msg, int msg_size, uint8_t *d, int d_size)
{
	const struct device *dev = device_get_binding(HASH_DRV_NAME);
	enum hash_algo algo = CRYPTO_HASH_ALGO_SHA384;
	struct hash_ctx ini;
	struct hash_pkt pkt;
	uint8_t digest[64];
	int ret;

	LOG_DBG("Start feeding raw data");
	ini.flags = crypto_query_hwcaps(dev);
	pkt.in_buf = msg;
	pkt.in_len = msg_size;
	pkt.out_buf = digest;

	ret = hash_begin_session(dev, &ini, algo);
	if (ret) {
		LOG_ERR("hash_begin_session error");
		return ret;
	}

	ret = hash_update(&ini, &pkt);
	if (ret)
		LOG_ERR("hash_update error");

	/* final */
	ret = hash_compute(&ini, &pkt);
	if (ret)
		LOG_ERR("hash_compute error");

	hash_free_session(dev, &ini);

	if (!memcmp(digest, d, d_size))
		LOG_DBG("digest compare - PASS");
	else
		LOG_DBG("digest compare - FAIL");

	return 0;
}
#endif

static int hw_ecdsa_test(const struct ecdsa_testvec *tv, int tv_size)
{
	const struct device *dev = device_get_binding(ECDSA_DRV_NAME);
	struct ecdsa_ctx ini;
	struct ecdsa_pkt pkt;
	struct ecdsa_key ek;
	int ret, rc = 0;

	LOG_INF("Start...");
	for (int i = 0; i < tv_size; i++) {
		/* Doing hash first for Caliptra secure IP case */
#ifdef CONFIG_CPTRA_ECDSA
		hw_gen_sha((uint8_t *)tv[i].raw, tv[i].raw_size,
			   (uint8_t *)tv[i].m, tv[i].m_size);
#endif

		ek.curve_id = ECC_CURVE_NIST_P384;
		ek.qx = (char *)tv[i].qx;
		ek.qy = (char *)tv[i].qy;
		pkt.m = (char *)tv[i].m;
		pkt.r = (char *)tv[i].r;
		pkt.s = (char *)tv[i].s;
		pkt.m_len = 48;
		pkt.r_len = 48;
		pkt.s_len = 48;

		LOG_DBG("Test case %d...", i);
		ret = ecdsa_begin_session(dev, &ini, &ek);
		if (ret)
			LOG_INF("ecdsa_begin_session fail: %d", ret);

		ret = ecdsa_verify(&ini, &pkt);
		if (ret && !tv[i].result)
			LOG_DBG(" result expected (failed), Pass\n");
		else if (ret == 0 && tv[i].result)
			LOG_DBG(" result expected (pass), Pass\n");
		else {
			LOG_DBG(" result unexpected (ret=%d), Failed\n", ret);
			rc = -1;
		}

		ecdsa_free_session(dev, &ini);

		if (rc) {
			LOG_INF("Failure");
			return rc;
		}
	}

	LOG_INF("Pass");

	return 0;
}

static int ecdsa_selftest(void)
{
	int ret;

	ret = mbedtls_ecdsa_test(secp384r1_tv, ARRAY_SIZE(secp384r1_tv));
	if (ret) {
		LOG_ERR("mbedtls ecdsa test failed");
		return ret;
	}

	ret = hw_ecdsa_test(secp384r1_tv, ARRAY_SIZE(secp384r1_tv));
	if (ret) {
		LOG_ERR("hw ecdsa test failed");
		return ret;
	}

	return 0;
}

int crypto_selftest(void)
{
	int ret;

	LOG_DBG("");

	/* ECDSA self-test */
	ret = ecdsa_selftest();

	return ret;
}

SYS_INIT(crypto_selftest, APPLICATION, 0);
