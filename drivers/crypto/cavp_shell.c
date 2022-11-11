/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include <crypto/hash.h>
#include <crypto/cipher.h>
#include <crypto/rsa.h>
#include <crypto/ecdsa.h>
#include <device.h>
#include <drivers/uart.h>
#include <random/rand32.h>
#include <shell/shell.h>
#include <stdio.h>
#include <stdlib.h>
#include <soc.h>
#include <sys/ring_buffer.h>
#include <usb/usb_device.h>

#include "mbedtls/rsa.h"
#include "mbedtls/md.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"

#ifdef CONFIG_CRYPTO_ASPEED
#define HASH_DRV_NAME		CONFIG_CRYPTO_ASPEED_HASH_DRV_NAME
#define CRYPTO_DRV_NAME		CONFIG_CRYPTO_ASPEED_DRV_NAME
#else
#error "You need to enable ASPEED crypto device"
#endif

#ifdef CONFIG_ECDSA_ASPEED
#define ECDSA_DRV_NAME		DT_LABEL(DT_INST(0, aspeed_ecdsa))
#endif

struct hash_testvec {
	const char *plaintext;
	size_t psize;
	const char *key;
	size_t ksize;
};

#define RX_BUFF_SIZE		KB(64)
#define RING_BUF_SIZE		256

#define INPUT_TIMEOUT		1000	/* 100 MSEC */
#define BITS_PER_HEX		4

#define DEBUG 0

RING_BUF_DECLARE(databuf, RING_BUF_SIZE);

static bool is_usb_init;
static int input_len;		/* bits */
static int recv_len;		/* bytes */
static uint8_t inputbuf_hex[RX_BUFF_SIZE] NON_CACHED_BSS;
static uint8_t inputbuf[RX_BUFF_SIZE/2] NON_CACHED_BSS;

uint32_t crypto_cap_flags;

#if DEBUG
static void print_buffer(const struct shell *shell, uint8_t *buf, size_t size)
{
	for (int i = 0; i < size; i++) {
		if (i % 32 == 0)
			shell_fprintf(shell, SHELL_NORMAL, "\n");
		shell_fprintf(shell, SHELL_NORMAL, "%02x", buf[i]);
	}
	shell_print(shell, "\n");
}
#endif

static void data_handle(void)
{
	int data_print = 0;
	int rb_len;

	while (!ring_buf_is_empty(&databuf)) {
		rb_len = ring_buf_get(&databuf, inputbuf_hex + recv_len * 2,
				      RING_BUF_SIZE);

		char *in = inputbuf_hex + recv_len * 2;
		char *out = inputbuf + recv_len;

		hex2bin(in, rb_len, out, rb_len/2);

		if (data_print) {
			printk("Print Data: ");
			for (int i = 0; i < rb_len; i++)
				printk("%c", in[i]);
			printk("\n");

			printk("Print Data: ");
			for (int i = 0; i < rb_len/2; i++)
				printk("%02x", out[i]);
			printk("\n");
		}

		recv_len += rb_len/2;
	}
}

static void interrupt_handler(const struct device *dev, void *user_data)
{
	uint8_t rx_buff[RX_BUFF_SIZE];
	int len, rb_len;

	ARG_UNUSED(user_data);

	while (uart_irq_is_pending(dev) && uart_irq_rx_ready(dev)) {
		len = uart_fifo_read(dev, rx_buff, sizeof(rx_buff));

		if (len) {
			rb_len = ring_buf_put(&databuf, rx_buff, len);
			if (rb_len < len) {
				printk("Drop %u bytes\n", len - rb_len);
			}

		} else {
			break;
		}

		data_handle();
	}
}

static int usb_init(const struct shell *shell)
{
	const struct device *dev;
	int ret;

	ret = usb_enable(NULL);
	if (ret != 0) {
		shell_print(shell, "Failed to enable USB");
		return -1;
	}

	dev = device_get_binding("CDC_ACM_0");
	if (dev) {
		shell_print(shell, "This device supports USB CDC_ACM class.\n");

		uart_irq_callback_set(dev, interrupt_handler);
		/* Enable rx interrupts */
		uart_irq_rx_enable(dev);
	}

	is_usb_init = true;

	return 0;
}

static int _sha_test(const struct shell *shell, struct hash_ctx *ini,
		     struct hash_pkt *pkt, enum hash_algo algo)
{
	const struct device *dev = device_get_binding(HASH_DRV_NAME);
	int ret;

	ret = hash_begin_session(dev, ini, algo);
	if (ret) {
		shell_print(shell, "hash_begin_session error");
		goto out;
	}

	ret = hash_update(ini, pkt);
	if (ret) {
		shell_print(shell, "hash_update error");
		goto out;
	}

	ret = hash_final(ini, pkt);
	if (ret) {
		shell_print(shell, "hash_final error");
		goto out;
	}

	hash_free_session(dev, ini);
	return 0;
out:
	hash_free_session(dev, ini);
	return ret;
}

/*
 * argv[1]: Input Data Size
 * Data would transfer through USB-Uart
 */
static int sha_test_param(const struct shell *shell, enum hash_algo algo,
			  size_t argc, char **argv)
{
	struct hash_testvec tv;
	struct hash_ctx ini;
	struct hash_pkt pkt;
	uint8_t digest[64];
	int ret;

	argc--;
	argv++;

	if (!is_usb_init) {
		shell_print(shell, "USB is not initialized");
		return -EINVAL;
	}

	input_len = strtol(argv[0], NULL, 10);
	tv.psize = input_len/8;

	if ((recv_len * 8) < input_len) {
		shell_print(shell, "Input is not enough. recv:%d btyes, input:%d bits",
			    recv_len, input_len);
		recv_len = 0;
		return -EINVAL;
		/* padding 0 */
	}

	shell_print(shell, "%s: psize:%d", __func__, tv.psize);
	if (tv.psize == 0)
		tv.plaintext = "";
	else
		tv.plaintext = inputbuf;

	recv_len = 0;

	pkt.in_buf = (uint8_t *)tv.plaintext;
	pkt.in_len = tv.psize;
	pkt.out_buf = digest;
	pkt.out_buf_max = sizeof(digest);

	ret = _sha_test(shell, &ini, &pkt, algo);

	shell_fprintf(shell, SHELL_NORMAL, "\ndigest:");
	for (int j = 0; j < ini.digest_size; j++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", digest[j]);
	}
	shell_print(shell, "\n");

	return ret;
}

static int sha_test_param_mct(const struct shell *shell, enum hash_algo algo,
			      size_t argc, char **argv)
{
	uint8_t *seed_buf, *a_buf, *b_buf, *c_buf;
	uint8_t *plain_buf;
	struct hash_ctx ini;
	struct hash_pkt pkt;
	uint8_t digest[64];
	int seed_size;
	int offset;
	int ret;

	argc--;
	argv++;

	if (!is_usb_init) {
		shell_print(shell, "USB is not initialized");
		return -EINVAL;
	}

	input_len = strtol(argv[0], NULL, 10);
	seed_size = input_len/8;

	if ((recv_len * 8) < input_len) {
		shell_print(shell, "Input is not enough. recv:%d btyes, input:%d bits",
			    recv_len, input_len);
		recv_len = 0;
		return -EINVAL;
		/* padding 0 */
	}

	recv_len = 0;

	/* inputbuf layout: seed + A + B + C */
	/* MSG = A || B || C */
	seed_buf = inputbuf;
	offset = seed_size;

	/* A */
	a_buf = inputbuf + offset;
	plain_buf = a_buf;
	offset += seed_size;

	/* B */
	b_buf = inputbuf + offset;
	offset += seed_size;

	/* C */
	c_buf = inputbuf + offset;
	offset += seed_size;

	pkt.in_buf = plain_buf;
	pkt.in_len = seed_size * 3;
	pkt.out_buf = digest;
	pkt.out_buf_max = sizeof(digest);

	for (int i = 0; i < 100; i++) {
		memcpy(a_buf, seed_buf, seed_size);
		memcpy(b_buf, seed_buf, seed_size);
		memcpy(c_buf, seed_buf, seed_size);

		for (int j = 0; j < 1000; j++) {
			/* MD = SHA(MSG) */
			ret = _sha_test(shell, &ini, &pkt, algo);
			if (ret) {
				shell_print(shell, "HW sha failed, ret:0x%x", ret);
				goto end;
			}

			/* A = B */
			memcpy(a_buf, b_buf, seed_size);
			/* B = C */
			memcpy(b_buf, c_buf, seed_size);
			/* C = MD */
			memcpy(c_buf, digest, seed_size);
		}

		/* Output MD */
		shell_fprintf(shell, SHELL_NORMAL, "digest:");
		for (int j = 0; j < ini.digest_size; j++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", digest[j]);
		}
		shell_print(shell, "");

		/* SEED = MD */
		memcpy(seed_buf, digest, seed_size);
	}

end:
	return ret;
}

static int sha512_256_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	argc--;
	argv++;

	if (strcmp(argv[0], "aft") == 0)
		return sha_test_param(shell, HASH_SHA512_256, argc, argv);
	else if (strcmp(argv[0], "mct") == 0)
		return sha_test_param_mct(shell, HASH_SHA512_256, argc, argv);

	return -EINVAL;
}

static int sha512_224_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	argc--;
	argv++;

	if (strcmp(argv[0], "aft") == 0)
		return sha_test_param(shell, HASH_SHA512_224, argc, argv);
	else if (strcmp(argv[0], "mct") == 0)
		return sha_test_param_mct(shell, HASH_SHA512_224, argc, argv);

	return -EINVAL;
}

static int sha512_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	argc--;
	argv++;

	if (strcmp(argv[0], "aft") == 0)
		return sha_test_param(shell, HASH_SHA512, argc, argv);
	else if (strcmp(argv[0], "mct") == 0)
		return sha_test_param_mct(shell, HASH_SHA512, argc, argv);

	return -EINVAL;
}

static int sha384_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	argc--;
	argv++;

	if (strcmp(argv[0], "aft") == 0)
		return sha_test_param(shell, HASH_SHA384, argc, argv);
	else if (strcmp(argv[0], "mct") == 0)
		return sha_test_param_mct(shell, HASH_SHA384, argc, argv);

	return -EINVAL;
}

static int sha256_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	argc--;
	argv++;

	if (strcmp(argv[0], "aft") == 0)
		return sha_test_param(shell, HASH_SHA256, argc, argv);
	else if (strcmp(argv[0], "mct") == 0)
		return sha_test_param_mct(shell, HASH_SHA256, argc, argv);

	return -EINVAL;
}

static int sha224_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	argc--;
	argv++;

	if (strcmp(argv[0], "aft") == 0)
		return sha_test_param(shell, HASH_SHA224, argc, argv);
	else if (strcmp(argv[0], "mct") == 0)
		return sha_test_param_mct(shell, HASH_SHA224, argc, argv);

	return -EINVAL;
}

static int sha1_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	argc--;
	argv++;

	if (strcmp(argv[0], "aft") == 0)
		return sha_test_param(shell, HASH_SHA1, argc, argv);
	else if (strcmp(argv[0], "mct") == 0)
		return sha_test_param_mct(shell, HASH_SHA1, argc, argv);

	return -EINVAL;
}

static int _hmac_test(const struct shell *shell, enum hash_algo algo,
		      const struct hash_testvec *tv)
{
	const struct device *dev = device_get_binding(HASH_DRV_NAME);
	struct hash_ctx ini;
	struct hash_pkt pkt;
	uint8_t digest[64];
	int ret;

	pkt.key_buf = (uint8_t *)tv[0].key;
	pkt.key_len = tv[0].ksize/8;
	pkt.in_buf = (uint8_t *)tv[0].plaintext;
	pkt.in_len = tv[0].psize/8;
	pkt.out_buf = digest;
	pkt.out_buf_max = sizeof(digest);


	ret = hash_begin_session(dev, &ini, algo);
	if (ret) {
		shell_print(shell, "hash_begin_session error");
		goto out;
	}

	ret = hash_setkey(&ini, &pkt);
	if (ret) {
		shell_print(shell, "hash_setkey error");
		goto out;
	}

	ret = hash_digest_hmac(&ini, &pkt);
	if (ret) {
		shell_print(shell, "hash_digest error");
		goto out;
	}

	hash_free_session(dev, &ini);

	shell_fprintf(shell, SHELL_NORMAL, "\nhmac:");
	for (int j = 0; j < ini.digest_size; j++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", digest[j]);
	}
	shell_print(shell, "\n");

	return 0;
out:
	hash_free_session(dev, &ini);
	return ret;
}

/*
 * argv[1]: Key Size
 * argv[2]: Key
 * argv[3]: Message Size
 * argv[4]: Message
 */
static int hmac_test_param(const struct shell *shell, enum hash_algo algo,
			   size_t argc, char **argv)
{
	struct hash_testvec tv;
	int rc;

	if (argc != 5) {
		shell_print(shell, "Wrong parameters");
		return -EINVAL;
	}

	tv.ksize = strtol(argv[1], NULL, 10);
	tv.key = malloc(tv.ksize * sizeof(char));

	hex2bin(argv[2], tv.ksize/4, (uint8_t *)tv.key, tv.ksize/8);

	tv.psize = strtol(argv[3], NULL, 10);
	tv.plaintext = malloc(tv.psize * sizeof(char));

	hex2bin(argv[4], tv.psize/4, (uint8_t *)tv.plaintext, tv.psize/8);

#if DEBUG
	shell_print(shell, "Input Key:");
	for (int i = 0; i < tv.ksize/8; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", tv.key[i]);
	}
	shell_print(shell, "\n");
	shell_print(shell, "Input Msg:");
	for (int i = 0; i < tv.psize/8; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", tv.plaintext[i]);
	}
	shell_print(shell, "\n");
#endif

	rc = _hmac_test(shell, algo, &tv);

	if (tv.key)
		free((void *)tv.key);
	if (tv.plaintext)
		free((void *)tv.plaintext);

	return rc;
}

static int hmac_sha1_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return hmac_test_param(shell, HASH_SHA1, argc, argv);
}

static int hmac_sha224_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return hmac_test_param(shell, HASH_SHA224, argc, argv);
}

static int hmac_sha256_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return hmac_test_param(shell, HASH_SHA256, argc, argv);
}

static int hmac_sha384_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return hmac_test_param(shell, HASH_SHA384, argc, argv);
}

static int hmac_sha512_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return hmac_test_param(shell, HASH_SHA512, argc, argv);
}

static int hmac_sha512_224_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return hmac_test_param(shell, HASH_SHA512_224, argc, argv);
}

static int hmac_sha512_256_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return hmac_test_param(shell, HASH_SHA512_256, argc, argv);
}

static int crypto_test_param(const struct shell *shell, size_t argc, char **argv,
			     enum cipher_algo algo, enum cipher_mode mode,
			     enum cipher_op op)
{
	const struct device *dev = device_get_binding(CRYPTO_DRV_NAME);
	uint8_t *iv_buf, *key_buf, *text_buf;
	uint8_t *input_buf, *output_buf;
	struct cipher_pkt pkt;
	struct cipher_ctx ini;
	int key_size, text_size, input_size;
	int output_offset;
	int iv_size = 0;
	int ret = 0;

	argc--;
	argv++;

	if (mode == CRYPTO_CIPHER_MODE_ECB) {
		/* Parse key & pt/ct */
		key_size = strtol(argv[0], NULL, 10);	/* bits */
		hex2bin(argv[1], key_size/4, inputbuf, key_size/8);

		text_size = strtol(argv[2], NULL, 10);	/* bits */
		hex2bin(argv[3], text_size/4, inputbuf + key_size/8, text_size/8);

	} else {
		/* Parse iv & key & pt/ct */
		iv_size = strtol(argv[0], NULL, 10);	/* bits */
		hex2bin(argv[1], iv_size/4, inputbuf, iv_size/8);

		key_size = strtol(argv[2], NULL, 10);	/* bits */
		hex2bin(argv[3], key_size/4, inputbuf + iv_size/8, key_size/8);

		text_size = strtol(argv[4], NULL, 10);	/* bits */
		hex2bin(argv[5], text_size/4, inputbuf + iv_size/8 + key_size/8, text_size/8);
	}

	if (iv_size) {
		iv_buf = inputbuf;
#if DEBUG
		shell_print(shell, "%s: iv_size: %d bits", __func__, iv_size);
		shell_print(shell, "Input IV:");
		for (int i = 0; i < iv_size/8; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", iv_buf[i]);
		}
		shell_print(shell, "\n");
#endif
	}

	int offset = iv_size/8;

	key_buf = inputbuf + offset;

#if DEBUG
	shell_print(shell, "%s: key_size: %d bits", __func__, key_size);
	shell_print(shell, "Input key:");
	for (int i = 0; i < key_size/8; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", key_buf[i]);
	}
	shell_print(shell, "\n");
#endif
	offset += key_size/8;
	text_buf = inputbuf + offset;

#if DEBUG
	shell_print(shell, "%s: text_size: %d bits", __func__, text_size);
	shell_print(shell, "Input text:");
	for (int i = 0; i < text_size/8; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", text_buf[i]);
	}
	shell_print(shell, "\n");
#endif

	/* first 16 bytes is for IV if necessary */
	input_buf = malloc(16 + text_size/8);
	if (!input_buf) {
		shell_print(shell, "Alloate input buffer failed");
		return -ENOMEM;
	}

	if (mode == CRYPTO_CIPHER_MODE_ECB || op == CRYPTO_CIPHER_OP_ENCRYPT) {
		memcpy(input_buf, text_buf, text_size/8);
		input_size = text_size/8;
	} else {
		memcpy(input_buf, iv_buf, iv_size/8);
		memcpy(input_buf + 16, text_buf, text_size/8);
		input_size = 16 + text_size/8;
	}

	ini.keylen = key_size/8;
	ini.key.bit_stream = key_buf;
	ini.flags = crypto_cap_flags;
	pkt.in_buf = input_buf;
	pkt.in_len = input_size;

	/* first 16 bytes is for IV if necessary */
	output_buf = malloc(16 + text_size/8);
	if (!output_buf) {
		shell_print(shell, "Alloate output buffer failed");
		goto end;
	}

	pkt.out_buf_max = text_size/8;
	pkt.out_buf = output_buf;

	ret = cipher_begin_session(dev, &ini, algo, mode, op);
	if (ret) {
		shell_print(shell, "Cipher begin failed");
		goto end;
	}

	switch (mode) {
	case CRYPTO_CIPHER_MODE_ECB:
		ret = cipher_block_op(&ini, &pkt);
		break;
	case CRYPTO_CIPHER_MODE_CBC:
		ret = cipher_cbc_op(&ini, &pkt, iv_buf);
		break;
	case CRYPTO_CIPHER_MODE_CFB:
		ret = cipher_cbc_op(&ini, &pkt, iv_buf);
		break;
	case CRYPTO_CIPHER_MODE_OFB:
		ret = cipher_cbc_op(&ini, &pkt, iv_buf);
		break;
	case CRYPTO_CIPHER_MODE_CTR:
		ret = cipher_cbc_op(&ini, &pkt, iv_buf);
		break;
	default:
		shell_print(shell, "mode 0x%x is not supported", mode);
		break;
	}

	if (ret) {
		shell_print(shell, "Cipher op:0x%x failed", op);
		goto end;
	}

	if (mode != CRYPTO_CIPHER_MODE_ECB && op == CRYPTO_CIPHER_OP_ENCRYPT)
		output_offset = 16;
	else
		output_offset = 0;

	shell_fprintf(shell, SHELL_NORMAL, "Output:");
	for (int i = output_offset; i < pkt.out_len; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", output_buf[i]);
	}
	shell_print(shell, "\n");

	cipher_free_session(dev, &ini);

end:
	if (input_buf)
		free((void *)input_buf);
	if (output_buf)
		free((void *)output_buf);

	return ret;
}

static int cipher_ops(const struct shell *shell, const struct device *dev,
		enum cipher_algo algo, enum cipher_mode mode, enum cipher_op op,
		struct cipher_pkt *pkt, struct cipher_ctx *ini, uint8_t *iv_buf)
{
	int ret;

	ret = cipher_begin_session(dev, ini, algo, mode, op);
	if (ret) {
		shell_print(shell, "Cipher begin failed");
		goto end;
	}

	switch (mode) {
	case CRYPTO_CIPHER_MODE_ECB:
		ret = cipher_block_op(ini, pkt);
		break;
	case CRYPTO_CIPHER_MODE_CBC:
		ret = cipher_cbc_op(ini, pkt, iv_buf);
		break;
	case CRYPTO_CIPHER_MODE_CFB:
		ret = cipher_cbc_op(ini, pkt, iv_buf);
		break;
	case CRYPTO_CIPHER_MODE_OFB:
		ret = cipher_cbc_op(ini, pkt, iv_buf);
		break;
	case CRYPTO_CIPHER_MODE_CTR:
		ret = cipher_cbc_op(ini, pkt, iv_buf);
		break;
	default:
		shell_print(shell, "mode 0x%x is not supported", mode);
		break;
	}

end:
	cipher_free_session(dev, ini);
	return ret;
}

static int aes_test_param_mct(const struct shell *shell, size_t argc, char **argv,
				 enum cipher_algo algo, enum cipher_mode mode,
				 enum cipher_op op)
{
	const struct device *dev = device_get_binding(CRYPTO_DRV_NAME);
	uint8_t *iv_buf, *key_buf, *text_buf, *next_key_buf;
	uint8_t *input_buf, *output_buf, *pre_output_buf;
	struct cipher_pkt pkt;
	struct cipher_ctx ini;
	int key_size, text_size, input_size;
	int output_offset;
	int input_offset;
	int iv_size = 0;
	int ret = 0;

	argc--;
	argv++;

	shell_print(shell, "%s: mode:%d, op:%d", __func__, mode, op);
	if (mode == CRYPTO_CIPHER_MODE_ECB) {
		/* Parse key & pt/ct */
		key_size = strtol(argv[0], NULL, 10);	/* bits */
		hex2bin(argv[1], key_size/4, inputbuf, key_size/8);

		text_size = strtol(argv[2], NULL, 10);	/* bits */
		hex2bin(argv[3], text_size/4, inputbuf + key_size/8, text_size/8);

	} else {
		/* Parse iv & key & pt/ct */
		iv_size = strtol(argv[0], NULL, 10);	/* bits */
		hex2bin(argv[1], iv_size/4, inputbuf, iv_size/8);

		key_size = strtol(argv[2], NULL, 10);	/* bits */
		hex2bin(argv[3], key_size/4, inputbuf + iv_size/8, key_size/8);

		text_size = strtol(argv[4], NULL, 10);	/* bits */
		hex2bin(argv[5], text_size/4, inputbuf + iv_size/8 + key_size/8, text_size/8);
	}

	if (iv_size) {
		iv_buf = inputbuf;
#if DEBUG
		shell_print(shell, "%s: iv_size: %d bits", __func__, iv_size);
		shell_print(shell, "Input IV:");
		for (int i = 0; i < iv_size/8; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", iv_buf[i]);
		}
		shell_print(shell, "\n");
#endif
	}

	int offset = iv_size/8;

	key_buf = inputbuf + offset;

#if DEBUG
	shell_print(shell, "%s: key_size: %d bits", __func__, key_size);
	shell_print(shell, "Input key:");
	for (int i = 0; i < key_size/8; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", key_buf[i]);
	}
	shell_print(shell, "\n");
#endif
	offset += key_size/8;
	text_buf = inputbuf + offset;

#if DEBUG
	shell_print(shell, "%s: text_size: %d bits", __func__, text_size);
	shell_print(shell, "Input text:");
	for (int i = 0; i < text_size/8; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", text_buf[i]);
	}
	shell_print(shell, "\n");
#endif

	/* first 16 bytes is for IV if necessary */
	offset += text_size/8;
	input_buf = inputbuf + offset;

	if (mode == CRYPTO_CIPHER_MODE_ECB || op == CRYPTO_CIPHER_OP_ENCRYPT) {
		memcpy(input_buf, text_buf, text_size/8);
		input_size = text_size/8;
	} else {
		memcpy(input_buf, iv_buf, iv_size/8);
		memcpy(input_buf + 16, text_buf, text_size/8);
		input_size = 16 + text_size/8;
	}

	/* first 16 bytes is for IV if necessary */
	offset += 16 + text_size/8;
	output_buf = inputbuf + offset;

	offset += 16 + text_size/8;
	next_key_buf = inputbuf + offset;

	offset += key_size/8;
	pre_output_buf = inputbuf + offset;

	if (mode != CRYPTO_CIPHER_MODE_ECB && op == CRYPTO_CIPHER_OP_ENCRYPT)
		output_offset = 16;
	else
		output_offset = 0;

	if (mode == CRYPTO_CIPHER_MODE_ECB)
		input_offset = 0;
	else {
		if (op == CRYPTO_CIPHER_OP_ENCRYPT)
			input_offset = 0;
		else
			input_offset = 16;
	}

	for (int i = 0; i < 100; i++) {
		/* output key[i] */
		if (i)
			memcpy(key_buf, next_key_buf, key_size/8);

		shell_fprintf(shell, SHELL_NORMAL, "Key:");
		for (int i = 0; i < key_size/8; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", key_buf[i]);
		}
		shell_print(shell, "");

		/* Output IV[i] */
		if (mode != CRYPTO_CIPHER_MODE_ECB) {
			shell_fprintf(shell, SHELL_NORMAL, "IV:");
			for (int i = 0; i < iv_size/8; i++) {
				shell_fprintf(shell, SHELL_NORMAL, "%02x", iv_buf[i]);
			}
			shell_print(shell, "");
		}

		/* Output PT[0] */
		shell_fprintf(shell, SHELL_NORMAL, "Input:");
		for (int i = 0; i < text_size/8; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", text_buf[i]);
		}
		shell_print(shell, "");

		if (mode == CRYPTO_CIPHER_MODE_ECB || op == CRYPTO_CIPHER_OP_ENCRYPT) {
			memcpy(input_buf, text_buf, text_size/8);
			input_size = text_size/8;
		} else {
			memcpy(input_buf, iv_buf, iv_size/8);
			memcpy(input_buf + 16, text_buf, text_size/8);
			input_size = 16 + text_size/8;
		}

		ini.keylen = key_size/8;
		ini.key.bit_stream = key_buf;
		ini.flags = crypto_cap_flags;
		pkt.in_buf = input_buf;
		pkt.in_len = input_size;
		pkt.out_buf_max = text_size/8;
		pkt.out_buf = output_buf;

		for (int j = 0; j < 1000; j++) {
			if (j == 999 || mode != CRYPTO_CIPHER_MODE_ECB) {
				memcpy(pre_output_buf, output_buf + output_offset, text_size/8);
#if DEBUG
				shell_fprintf(shell, SHELL_NORMAL, "Pre Output:");
				for (int i = 0; i < text_size/8; i++) {
					shell_fprintf(shell, SHELL_NORMAL, "%02x", pre_output_buf[i]);
				}
				shell_print(shell, "");
#endif
			}

			if (mode != CRYPTO_CIPHER_MODE_ECB) {
				if (j != 0) {
					if (mode == CRYPTO_CIPHER_MODE_OFB) {
						/* iv = pre-output ^ pre-input */
						for (int i = 0; i < iv_size/8; i++)
							iv_buf[i] = pre_output_buf[i] ^ text_buf[i];

					} else {
						if (op == CRYPTO_CIPHER_OP_ENCRYPT)
							/* iv = pre-output */
							memcpy(iv_buf, pre_output_buf, iv_size/8);
						else
							/* iv = pre-input */
							memcpy(iv_buf, text_buf, text_size/8);
					}
				}
#if DEBUG
				shell_fprintf(shell, SHELL_NORMAL, "iv_buf:");
				for (int i = 0; i < iv_size/8; i++) {
					shell_fprintf(shell, SHELL_NORMAL, "%02x", iv_buf[i]);
				}
				shell_print(shell, "");
				shell_fprintf(shell, SHELL_NORMAL, "key_buf:");
				for (int i = 0; i < key_size/8; i++) {
					shell_fprintf(shell, SHELL_NORMAL, "%02x", key_buf[i]);
				}
				shell_print(shell, "");
				shell_fprintf(shell, SHELL_NORMAL, "input_buf:");
				for (int i = input_offset; i < input_size; i++) {
					shell_fprintf(shell, SHELL_NORMAL, "%02x", input_buf[i]);
				}
				shell_print(shell, "");
#endif
				ret = cipher_ops(shell, dev, algo, mode, op, &pkt, &ini, iv_buf);
				if (ret) {
					shell_print(shell, "Cipher op:0x%x failed", op);
					goto end;
				}

				if (op == CRYPTO_CIPHER_OP_ENCRYPT)
					memcpy(text_buf, input_buf, text_size/8);
				else if (op == CRYPTO_CIPHER_OP_DECRYPT)
					memcpy(text_buf, input_buf + 16, text_size/8);

				if (j == 0) {
					/* PT[j+1] = IV[i] */
					if (op == CRYPTO_CIPHER_OP_ENCRYPT)
						memcpy(input_buf, iv_buf, iv_size/8);
					else
						memcpy(input_buf + 16, iv_buf, iv_size/8);

				} else {
					/* PT[j+1] = CT[j-1] */
					if (op == CRYPTO_CIPHER_OP_ENCRYPT)
						memcpy(input_buf, pre_output_buf, text_size/8);
					else
						memcpy(input_buf + 16, pre_output_buf, text_size/8);
				}
#if DEBUG
				/* Output CT[j] */
				shell_fprintf(shell, SHELL_NORMAL, "Output[%d]:", j);
				for (int i = output_offset; i < pkt.out_len; i++) {
					shell_fprintf(shell, SHELL_NORMAL, "%02x", output_buf[i]);
				}
				shell_print(shell, "");
#endif
			} else {
				ret = cipher_ops(shell, dev, algo, mode, op, &pkt, &ini, iv_buf);
				if (ret) {
					shell_print(shell, "Cipher op:0x%x failed", op);
					goto end;
				}

				/* PT[j+1] = CT[j] */
				memcpy(input_buf, output_buf, pkt.out_len);
			}
		}

		/* Output CT[j] */
		shell_fprintf(shell, SHELL_NORMAL, "Output:");
		for (int i = output_offset; i < pkt.out_len; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", output_buf[i]);
		}
		shell_print(shell, "");

		/* AES_KEY_SHUFFLE(Key, CT) */
		if (ini.keylen == 16) {
			for (int i = 0; i < 16; i++)
				next_key_buf[i] = key_buf[i] ^ (output_buf + output_offset)[i];

		} else if (ini.keylen == 24) {
			for (int i = 0; i < 8; i++)
				next_key_buf[i] = key_buf[i] ^ pre_output_buf[text_size/8 - 8 + i];
			for (int i = 8; i < 24; i++)
				next_key_buf[i] = key_buf[i] ^ (output_buf + output_offset)[i - 8];

		} else if (ini.keylen == 32) {
			for (int i = 0; i < 16; i++)
				next_key_buf[i] = key_buf[i] ^ pre_output_buf[i];
			for (int i = 16; i < 32; i++)
				next_key_buf[i] = key_buf[i] ^ (output_buf + output_offset)[i - 16];

		}

		if (mode != CRYPTO_CIPHER_MODE_ECB) {
			/* IV[i+1] = CT[j] */
			/* PT[0] = CT[j-1] */
			memcpy(iv_buf, output_buf + output_offset, iv_size/8);
			memcpy(text_buf, pre_output_buf, text_size/8);

		} else {
			/* PT[0] = CT[j] */
			memcpy(text_buf, output_buf, text_size/8);
		}
	}

end:
	return ret;
}

void des_key_parity_set(uint8_t *key, int len)
{
	int odd;

	for (int i = 0; i < len; i++) {
		odd = 0;
		for (int j = 0; j < 8; j++) {
			if (key[i] & (0x1 << j))
				odd = !odd;
		}

		if (!odd)
			key[i] ^= 0x1;
	}
}

static int tdes_test_param_mct(const struct shell *shell, size_t argc, char **argv,
				enum cipher_algo algo, enum cipher_mode mode,
				enum cipher_op op)
{
	const struct device *dev = device_get_binding(CRYPTO_DRV_NAME);
	uint8_t *iv_buf, *key_buf, *text_buf, *next_key_buf;
	uint8_t *input_buf, *output_buf, *pre_input_buf, *pre_output_buf;
	uint8_t *pre_pre_output_buf;
	struct cipher_pkt pkt;
	struct cipher_ctx ini;
	int key_size, text_size, input_size;
	int output_offset, input_offset;
	int klen;
	int iv_size = 0;
	int ret = 0;

	argc--;
	argv++;

	if (mode == CRYPTO_CIPHER_MODE_ECB) {
		/* Parse key & pt/ct */
		key_size = strtol(argv[0], NULL, 10);	/* bits */
		hex2bin(argv[1], key_size/4, inputbuf, key_size/8);

		text_size = strtol(argv[2], NULL, 10);	/* bits */
		hex2bin(argv[3], text_size/4, inputbuf + key_size/8, text_size/8);

	} else {
		/* Parse iv & key & pt/ct */
		iv_size = strtol(argv[0], NULL, 10);	/* bits */
		hex2bin(argv[1], iv_size/4, inputbuf, iv_size/8);

		key_size = strtol(argv[2], NULL, 10);	/* bits */
		hex2bin(argv[3], key_size/4, inputbuf + iv_size/8, key_size/8);

		text_size = strtol(argv[4], NULL, 10);	/* bits */
		hex2bin(argv[5], text_size/4, inputbuf + iv_size/8 + key_size/8, text_size/8);
	}

	klen = key_size/24;

	if (iv_size) {
		iv_buf = inputbuf;
#if DEBUG
		shell_print(shell, "%s: iv_size: %d bits", __func__, iv_size);
		shell_print(shell, "Input IV:");
		for (int i = 0; i < iv_size/8; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", iv_buf[i]);
		}
		shell_print(shell, "\n");
#endif
	}

	int offset = iv_size/8;

	key_buf = inputbuf + offset;

#if DEBUG
	shell_print(shell, "%s: key_size: %d bits", __func__, key_size);
	shell_print(shell, "Input key:");
	for (int i = 0; i < key_size/8; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", key_buf[i]);
	}
	shell_print(shell, "\n");
#endif
	offset += key_size/8;
	text_buf = inputbuf + offset;

#if DEBUG
	shell_print(shell, "%s: text_size: %d bits", __func__, text_size);
	shell_print(shell, "Input text:");
	for (int i = 0; i < text_size/8; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", text_buf[i]);
	}
	shell_print(shell, "\n");
#endif

	/* first 16 bytes is for IV if necessary */
	offset += text_size/8;
	input_buf = inputbuf + offset;

	if (mode == CRYPTO_CIPHER_MODE_ECB || op == CRYPTO_CIPHER_OP_ENCRYPT) {
		memcpy(input_buf, text_buf, text_size/8);
		input_size = text_size/8;
	} else {
		memcpy(input_buf, iv_buf, iv_size/8);
		memcpy(input_buf + 16, text_buf, text_size/8);
		input_size = 16 + text_size/8;
	}

	/* first 16 bytes is for IV if necessary */
	offset += 16 + text_size/8;
	output_buf = inputbuf + offset;

	offset += 16 + text_size/8;
	next_key_buf = inputbuf + offset;

	offset += key_size/8;
	pre_output_buf = inputbuf + offset;

	offset += text_size/8;
	pre_pre_output_buf = inputbuf + offset;

	offset += text_size/8;
	pre_input_buf = inputbuf + offset;

	if (mode != CRYPTO_CIPHER_MODE_ECB && op == CRYPTO_CIPHER_OP_ENCRYPT)
		output_offset = 16;
	else
		output_offset = 0;

	if (mode == CRYPTO_CIPHER_MODE_ECB)
		input_offset = 0;
	else {
		if (op == CRYPTO_CIPHER_OP_ENCRYPT)
			input_offset = 0;
		else
			input_offset = 16;
	}

	for (int i = 0; i < 400; i++) {
		/* output key[i] */
		if (i)
			memcpy(key_buf, next_key_buf, key_size/8);

		shell_fprintf(shell, SHELL_NORMAL, "Key1:");
		for (int i = 0; i < klen; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", key_buf[i]);
		}
		shell_print(shell, "");
		shell_fprintf(shell, SHELL_NORMAL, "Key2:");
		for (int i = klen; i < klen * 2; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", key_buf[i]);
		}
		shell_print(shell, "");
		shell_fprintf(shell, SHELL_NORMAL, "Key3:");
		for (int i = klen * 2; i < klen * 3; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", key_buf[i]);
		}
		shell_print(shell, "");

		/* Output IV[i] */
		if (mode != CRYPTO_CIPHER_MODE_ECB) {
			shell_fprintf(shell, SHELL_NORMAL, "IV:");
			for (int i = 0; i < iv_size/8; i++) {
				shell_fprintf(shell, SHELL_NORMAL, "%02x", iv_buf[i]);
			}
			shell_print(shell, "");
		}

		/* Output PT[0] */
		shell_fprintf(shell, SHELL_NORMAL, "Input:");
		for (int i = 0; i < text_size/8; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", text_buf[i]);
		}
		shell_print(shell, "");

		if (mode == CRYPTO_CIPHER_MODE_ECB || op == CRYPTO_CIPHER_OP_ENCRYPT) {
			memcpy(input_buf, text_buf, text_size/8);
			input_size = text_size/8;
		} else {
			memcpy(input_buf, iv_buf, iv_size/8);
			memcpy(input_buf + 16, text_buf, text_size/8);
			input_size = 16 + text_size/8;
		}

		ini.keylen = key_size/8;
		ini.key.bit_stream = key_buf;
		ini.flags = crypto_cap_flags;
		pkt.in_buf = input_buf;
		pkt.in_len = input_size;
		pkt.out_buf_max = text_size/8;
		pkt.out_buf = output_buf;

		for (int j = 0; j < 10000; j++) {
			if (j == 9998)
				memcpy(pre_pre_output_buf, output_buf + output_offset, text_size/8);

			if (j == 9999 || mode != CRYPTO_CIPHER_MODE_ECB) {
				memcpy(pre_output_buf, output_buf + output_offset, text_size/8);
#if DEBUG
				shell_fprintf(shell, SHELL_NORMAL, "Pre Output:");
				for (int i = 0; i < text_size/8; i++) {
					shell_fprintf(shell, SHELL_NORMAL, "%02x", pre_output_buf[i]);
				}
				shell_print(shell, "");
#endif
			}

			if (mode == CRYPTO_CIPHER_MODE_CBC) {
				ret = cipher_ops(shell, dev, algo, mode, op, &pkt, &ini, iv_buf);
				if (ret) {
					shell_print(shell, "Cipher op:0x%x failed", op);
					goto end;
				}

				if (op == CRYPTO_CIPHER_OP_ENCRYPT) {
					if (j == 0) {
						/* PT[j+1] = IV[0] */
						memcpy(input_buf, iv_buf, iv_size/8);

					} else {
						/* PT[j+1] = CT[j-1] */
						memcpy(input_buf, pre_output_buf, text_size/8);
					}

					/* IV[j+1] = CT[j] */
					memcpy(iv_buf, output_buf + output_offset, iv_size/8);

				} else {
					/* IV[j+1] = CT[j] */
					memcpy(iv_buf, input_buf + 16, iv_size/8);
					/* CT[j+1] = PT[j] */
					memcpy(input_buf + 16, output_buf + output_offset, text_size/8);
				}

			} else if (mode == CRYPTO_CIPHER_MODE_ECB) {
				ret = cipher_ops(shell, dev, algo, mode, op, &pkt, &ini, iv_buf);
				if (ret) {
					shell_print(shell, "Cipher op:0x%x failed", op);
					goto end;
				}

				/* PT[j+1] = CT[j] */
				memcpy(input_buf, output_buf, pkt.out_len);

			} else if (mode == CRYPTO_CIPHER_MODE_OFB) {
				if (j != 0) {
					/* IV[j+1] = O[j] */
					/* iv = pre-output ^ pre-input */
					for (int i = 0; i < iv_size/8; i++)
						iv_buf[i] = pre_output_buf[i] ^ pre_input_buf[i];
				}

				if (op == CRYPTO_CIPHER_OP_ENCRYPT)
					memcpy(pre_input_buf, input_buf, text_size/8);
				else
					memcpy(pre_input_buf, input_buf + 16, text_size/8);

				ret = cipher_ops(shell, dev, algo, mode, op, &pkt, &ini, iv_buf);
				if (ret) {
					shell_print(shell, "Cipher op:0x%x failed", op);
					goto end;
				}

				/* PT[j+1] = IV[j] */
				if (op == CRYPTO_CIPHER_OP_ENCRYPT)
					memcpy(input_buf, iv_buf, iv_size/8);
				else
					memcpy(input_buf + 16, iv_buf, iv_size/8);

			} else if (mode == CRYPTO_CIPHER_MODE_CFB) {
				ret = cipher_ops(shell, dev, algo, mode, op, &pkt, &ini, iv_buf);
				if (ret) {
					shell_print(shell, "Cipher op:0x%x failed", op);
					goto end;
				}

				if (op == CRYPTO_CIPHER_OP_ENCRYPT) {
					/* PT[j+1] = LeftMost_K_Bits(IV[j]) */
					memcpy(input_buf, iv_buf, iv_size/8);
					/* IV[j+1] = RightMost_64-K_Bits(IV[j]) || CT[j] */
					if (j != 9999)
						memcpy(iv_buf, output_buf + output_offset, text_size/8);
				} else {
					/* O[j] = C[J] XOR P[J] */
					/* CT[j] */
					memcpy(pre_input_buf, input_buf + 16, text_size/8);

					/* IV[j+1] = RightMost_64-K_Bits(IV[j]) || CT[j] */
					memcpy(iv_buf, input_buf + 16, iv_size/8);
					/* CT[j+1] = LeftMost_K_Bits(O[j]) */
					for (int i = 0; i < text_size/8; i++)
						(input_buf + 16)[i] = pre_input_buf[i] ^ (output_buf + output_offset)[i];
				}
			}
		}

		/* Output CT[j] */
		shell_fprintf(shell, SHELL_NORMAL, "Output:");
		for (int i = output_offset; i < pkt.out_len; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", output_buf[i]);
		}
		shell_print(shell, "");

		/* KEY_SHUFFLE */
		for (int i = 0; i < klen; i++)
			next_key_buf[i] = key_buf[i] ^ (output_buf + output_offset)[i];

		for (int i = klen; i < klen * 2; i++)
			next_key_buf[i] = key_buf[i] ^ pre_output_buf[i - klen];

		for (int i = klen * 2; i < klen * 3; i++)
			next_key_buf[i] = key_buf[i] ^ pre_pre_output_buf[i - klen * 2];

		des_key_parity_set(next_key_buf, klen * 3);

		if (mode == CRYPTO_CIPHER_MODE_CFB) {
			if (op == CRYPTO_CIPHER_OP_ENCRYPT) {
				/* PT[0] = LeftMost_K_Bits(IV[j]) */
				memcpy(text_buf, iv_buf, iv_size/8);
				/* IV[0] = RightMost_64-K_Bits(IV[j]) || CT[j] */
				memcpy(iv_buf, output_buf + output_offset, text_size/8);
			} else {
				/* CT[0] = LeftMost_K_Bits(O[j]) */
				memcpy(text_buf, input_buf + 16, text_size/8);

				/* IV[0] = RightMost_64-K_Bits(IV[j]) || CT[j] */
				memcpy(iv_buf, pre_input_buf, iv_size/8);
			}

		} else if (mode == CRYPTO_CIPHER_MODE_OFB) {
			/* PT[0] = PT[0] ^ IV[j] */
			/* IV[0] = O[j] */
			for (int i = 0; i < text_size/8; i++)
				text_buf[i] = text_buf[i] ^ iv_buf[i];
			for (int i = 0; i < iv_size/8; i++)
				iv_buf[i] = (output_buf + output_offset)[i] ^ pre_input_buf[i];

		} else if (mode == CRYPTO_CIPHER_MODE_CBC) {
			/* PT[0] = CT[j-1] */
			/* IV[0] = CT[j] */
			if (op == CRYPTO_CIPHER_OP_ENCRYPT) {
				memcpy(text_buf, pre_output_buf, text_size/8);
				memcpy(iv_buf, output_buf + output_offset, iv_size/8);
			} else {
				memcpy(text_buf, output_buf + output_offset, text_size/8);
			}

		} else if (mode == CRYPTO_CIPHER_MODE_ECB) {
			/* PT[0] = CT[j] */
			memcpy(text_buf, output_buf, text_size/8);
		}
	}

end:
	return ret;
}

static int aes_test(const struct shell *shell, size_t argc, char **argv,
		    enum cipher_algo algo, enum cipher_mode mode)
{
	enum cipher_op op = CRYPTO_CIPHER_OP_INVALID;

	argc--;
	argv++;

	if (strcmp(argv[0], "encrypt") == 0)
		op = CRYPTO_CIPHER_OP_ENCRYPT;
	if (strcmp(argv[0], "decrypt") == 0)
		op = CRYPTO_CIPHER_OP_DECRYPT;

	if (op == CRYPTO_CIPHER_OP_INVALID) {
		shell_print(shell, "%s: invalid parameter: %s", __func__, argv[0]);
		return -EINVAL;
	}

	crypto_cap_flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

	argc--;
	argv++;

	if (strcmp(argv[0], "aft") == 0)
		return crypto_test_param(shell, argc, argv, algo, mode, op);
	else if (strcmp(argv[0], "mct") == 0)
		return aes_test_param_mct(shell, argc, argv, algo, mode, op);

	return -EINVAL;
}

static int aes_ecb_test(const struct shell *shell, size_t argc, char **argv)
{
	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_ECB);
}

static int aes_cbc_test(const struct shell *shell, size_t argc, char **argv)
{
	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_CBC);
}

static int aes_cfb_test(const struct shell *shell, size_t argc, char **argv)
{
	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_CFB);
}

static int aes_ofb_test(const struct shell *shell, size_t argc, char **argv)
{
	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_OFB);
}

static int aes_ctr_test(const struct shell *shell, size_t argc, char **argv)
{
	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_CTR);
}

static int tdes_test(const struct shell *shell, size_t argc, char **argv,
		    enum cipher_algo algo, enum cipher_mode mode)
{
	enum cipher_op op = CRYPTO_CIPHER_OP_INVALID;

	argc--;
	argv++;

	if (strcmp(argv[0], "encrypt") == 0)
		op = CRYPTO_CIPHER_OP_ENCRYPT;
	if (strcmp(argv[0], "decrypt") == 0)
		op = CRYPTO_CIPHER_OP_DECRYPT;

	if (op == CRYPTO_CIPHER_OP_INVALID) {
		shell_print(shell, "%s: invalid parameter: %s", __func__, argv[0]);
		return -EINVAL;
	}

	crypto_cap_flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

	argc--;
	argv++;

	if (strcmp(argv[0], "aft") == 0)
		return crypto_test_param(shell, argc, argv, algo, mode, op);
	else if (strcmp(argv[0], "mct") == 0)
		return tdes_test_param_mct(shell, argc, argv, algo, mode, op);

	return -EINVAL;
}

static int tdes_ecb_test(const struct shell *shell, size_t argc, char **argv)
{
	return tdes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_TDES,
			CRYPTO_CIPHER_MODE_ECB);
}

static int tdes_cbc_test(const struct shell *shell, size_t argc, char **argv)
{
	return tdes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_TDES,
			CRYPTO_CIPHER_MODE_CBC);
}

static int tdes_cfb_test(const struct shell *shell, size_t argc, char **argv)
{
	return tdes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_TDES,
			CRYPTO_CIPHER_MODE_CFB);
}

static int tdes_ofb_test(const struct shell *shell, size_t argc, char **argv)
{
	return tdes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_TDES,
			CRYPTO_CIPHER_MODE_OFB);
}

static int myrand(void *rng_state, unsigned char *output, size_t len)
{
	sys_rand_get(output, len);

	return 0;
}

static int ecdsa_verify_test(const struct shell *shell, size_t argc, char **argv,
			     enum rsa_ssa ssa)
{
	const struct device *dev = device_get_binding(HASH_DRV_NAME);
	uint32_t qx_size, qy_size, r_size, s_size, m_size;
	uint8_t *qx_buf, *qy_buf, *r_buf, *s_buf, *m_buf;
	struct ecdsa_ctx ini;
	struct ecdsa_pkt pkt;
	struct ecdsa_key ek;
	int offset, ret;

	argc--;
	argv++;

	if (strcmp(argv[0], "verify"))
		return -EINVAL;

	argc--;
	argv++;

	/* Qx */
	qx_size = strtol(argv[0], NULL, 10);	/* bits */
	qx_buf = inputbuf;
	offset = qx_size/8;
#if DEBUG
	shell_print(shell, "%s: Print Qx:", __func__);
	print_buffer(shell, qx_buf, qx_size/8);
#endif
	argc--;
	argv++;

	/* Qy */
	qy_size = strtol(argv[0], NULL, 10);	/* bits */
	qy_buf = inputbuf + offset;
	offset += qy_size/8;
#if DEBUG
	shell_print(shell, "%s: Print Qy:", __func__);
	print_buffer(shell, qy_buf, qy_size/8);
#endif
	argc--;
	argv++;

	/* r */
	r_size = strtol(argv[0], NULL, 10);	/* bits */
	r_buf = inputbuf + offset;
	offset += r_size/8;
#if DEBUG
	shell_print(shell, "%s: Print r:", __func__);
	print_buffer(shell, r_buf, r_size/8);
#endif
	argc--;
	argv++;

	/* s */
	s_size = strtol(argv[0], NULL, 10);	/* bits */
	s_buf = inputbuf + offset;
	offset += s_size/8;
#if DEBUG
	shell_print(shell, "%s: Print s:", __func__);
	print_buffer(shell, s_buf, s_size/8);
#endif
	argc--;
	argv++;

	/* m */
	m_size = strtol(argv[0], NULL, 10);	/* bits */
	m_buf = inputbuf + offset;
	offset += m_size/8;
#if DEBUG
	shell_print(shell, "%s: Print m:", __func__);
	print_buffer(shell, m_buf, m_size/8);
#endif
	argc--;
	argv++;

	if (recv_len != offset) {
		shell_print(shell, "Input data is not match. recv:%d bytes, offset:%d bytes",
			    recv_len, offset);
		recv_len = 0;
		return -EINVAL;
	}

	recv_len = 0;

	/* Do sha384 */
	uint8_t digest[64];
	struct hash_ctx hash_ini;
	struct hash_pkt hash_pkt;

	hash_pkt.in_buf = (uint8_t *)m_buf;
	hash_pkt.in_len = m_size/8;
	hash_pkt.out_buf = digest;
	hash_pkt.out_buf_max = sizeof(digest);

	ret = hash_begin_session(dev, &hash_ini, HASH_SHA384);
	if (ret) {
		shell_print(shell, "hash_begin_session error");
		goto out;
	}

	ret = hash_update(&hash_ini, &hash_pkt);
	if (ret) {
		shell_print(shell, "hash_update error");
		goto out;
	}

	ret = hash_final(&hash_ini, &hash_pkt);
	if (ret) {
		shell_print(shell, "hash_final error");
		goto out;
	}

	hash_free_session(dev, &hash_ini);
#if DEBUG
	shell_fprintf(shell, SHELL_NORMAL, "\ndigest:");
	for (int j = 0; j < hash_ini.digest_size; j++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x", digest[j]);
	}
	shell_print(shell, "\n");
#endif

	/* check input size */
	if (qx_size/8 != SHA384_DIGEST_SIZE ||
	    qy_size/8 != SHA384_DIGEST_SIZE ||
	    r_size/8 != SHA384_DIGEST_SIZE ||
	    s_size/8 != SHA384_DIGEST_SIZE) {
		shell_print(shell, "%s: Wrong input size", __func__);
		return -EINVAL;
	}

	ek.curve_id = ECC_CURVE_NIST_P384;
	ek.qx = (char *)qx_buf;
	ek.qy = (char *)qy_buf;
	pkt.m = (char *)digest;
	pkt.r = (char *)r_buf;
	pkt.s = (char *)s_buf;
	pkt.m_len = pkt.r_len = pkt.s_len = SHA384_DIGEST_SIZE;
	dev = device_get_binding(ECDSA_DRV_NAME);

	shell_fprintf(shell, SHELL_NORMAL, " start verify\n");
	ret = ecdsa_begin_session(dev, &ini, &ek);
	if (ret)
		shell_print(shell, "ecdsa_begin_session fail: %d", ret);

	ret = ecdsa_verify(&ini, &pkt);
	if (ret)
		shell_print(shell, "%s: ECDSA verify failed !", __func__);
	else
		shell_print(shell, "%s: ECDSA verify PASS !", __func__);

	shell_print(shell, "Output:%d", ret);

	ecdsa_free_session(dev, &ini);

out:
	return ret;
}

static int rsa_sign_test(const struct shell *shell, size_t argc, char **argv,
			 enum rsa_ssa ssa)
{
	uint32_t n_size, e_size, p_size, q_size, msg_size;
	uint8_t *n_buf, *e_buf, *p_buf, *q_buf, *msg_buf, *sign_buf;
	uint8_t mdsum[64] = {0};
	mbedtls_md_type_t md_alg;
	mbedtls_rsa_context rsa;
	mbedtls_mpi K;
	int hashlen = 0;
	int offset;
	int ret;

	argc--;
	argv++;

	mbedtls_mpi_init(&K);
	mbedtls_rsa_init(&rsa);

	/* hash algo */
	shell_print(shell, "%s: hash: %s", __func__, argv[0]);
	if (!strcmp(argv[0], "sha1")) {
		md_alg = MBEDTLS_MD_SHA1;
	} else if (!strcmp(argv[0], "sha224")) {
		md_alg = MBEDTLS_MD_SHA224;
	} else if (!strcmp(argv[0], "sha256")) {
		md_alg = MBEDTLS_MD_SHA256;
	} else if (!strcmp(argv[0], "sha384")) {
		md_alg = MBEDTLS_MD_SHA384;
	} else if (!strcmp(argv[0], "sha512")) {
		md_alg = MBEDTLS_MD_SHA512;
	} else {
		shell_print(shell, "%s: hash %s is not supported", __func__, argv[0]);
		return -EINVAL;
	}

	/* set padding */
	if (ssa == RSA_PKCS1_V15)
		mbedtls_rsa_set_padding(&rsa, MBEDTLS_RSA_PKCS_V15, md_alg);
	else if (ssa == RSA_PKCS1_V21)
		mbedtls_rsa_set_padding(&rsa, MBEDTLS_RSA_PKCS_V21, md_alg);

	shell_print(shell, "%s: md_alg: %d", __func__, (int)md_alg);
	argc--;
	argv++;

	/* key n */
	n_size = strtol(argv[0], NULL, 10);	/* bits */
	n_buf = inputbuf;
	offset = n_size/8;

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(&K, n_buf, n_size/8));
	MBEDTLS_MPI_CHK(mbedtls_rsa_import(&rsa, &K, NULL, NULL, NULL, NULL));
#if DEBUG
	shell_print(shell, "%s: Print n(0x%x, %d bytes):", __func__, n_buf, n_size/8);
	print_buffer(shell, n_buf, n_size/8);
#endif
	argc--;
	argv++;

	/* key e */
	e_size = strtol(argv[0], NULL, 10);	/* bits */
	e_buf = inputbuf + offset;
	offset += e_size/8;

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(&K, e_buf, e_size/8));
	MBEDTLS_MPI_CHK(mbedtls_rsa_import(&rsa, NULL, NULL, NULL, NULL, &K));
#if DEBUG
	shell_print(shell, "%s: Print e(0x%x, %d bytes):", __func__, e_buf, e_size/8);
	print_buffer(shell, e_buf, e_size/8);
#endif
	argc--;
	argv++;

	/* key p */
	p_size = strtol(argv[0], NULL, 10);	/* bits */
	p_buf = inputbuf + offset;
	offset += p_size/8;

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(&K, p_buf, p_size/8));
	MBEDTLS_MPI_CHK(mbedtls_rsa_import(&rsa, NULL, &K, NULL, NULL, NULL));
#if DEBUG
	shell_print(shell, "%s: Print p(0x%x, %d bytes):", __func__, p_buf, p_size/8);
	print_buffer(shell, p_buf, p_size/8);
#endif
	argc--;
	argv++;

	/* key q */
	q_size = strtol(argv[0], NULL, 10);	/* bits */
	q_buf = inputbuf + offset;
	offset += q_size/8;

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(&K, q_buf, q_size/8));
	MBEDTLS_MPI_CHK(mbedtls_rsa_import(&rsa, NULL, NULL, &K, NULL, NULL));
#if DEBUG
	shell_print(shell, "%s: Print q(0x%x, %d bytes):", __func__, q_buf, q_size/8);
	print_buffer(shell, q_buf, q_size/8);
#endif
	argc--;
	argv++;

	/* msg */
	msg_size = strtol(argv[0], NULL, 10);	/* bits */
	msg_buf = inputbuf + offset;
	offset += msg_size/8;
#if DEBUG
	shell_print(shell, "%s: Print msg(0x%x, %d bytes):", __func__, msg_buf, msg_size/8);
	print_buffer(shell, msg_buf, msg_size/8);
#endif

	/* signature */
	sign_buf = inputbuf + offset;

	if (recv_len != offset) {
		shell_print(shell, "Input data is not match. recv:%d bytes, offset:%d bytes",
			    recv_len, offset);
		recv_len = 0;
		return -EINVAL;
	}

	recv_len = 0;

	MBEDTLS_MPI_CHK(mbedtls_rsa_complete(&rsa));
	MBEDTLS_MPI_CHK(mbedtls_rsa_check_privkey(&rsa));

	/* Calculate message digest */
	switch (md_alg) {
	case MBEDTLS_MD_SHA1:
		if (mbedtls_sha1(msg_buf, msg_size/8, mdsum) != 0) {
			shell_print(shell, "%s: SHA1 failed", __func__);
			return -EINVAL;
		}
		hashlen = 20;
		break;
	case MBEDTLS_MD_SHA224:
		if (mbedtls_sha256(msg_buf, msg_size/8, mdsum, 1) != 0) {
			shell_print(shell, "%s: SHA224 failed", __func__);
			return -EINVAL;
		}
		hashlen = 28;
		break;
	case MBEDTLS_MD_SHA256:
		if (mbedtls_sha256(msg_buf, msg_size/8, mdsum, 0) != 0) {
			shell_print(shell, "%s: SHA256 failed", __func__);
			return -EINVAL;
		}
		hashlen = 32;
		break;
	case MBEDTLS_MD_SHA384:
		if (mbedtls_sha512(msg_buf, msg_size/8, mdsum, 1) != 0) {
			shell_print(shell, "%s: SHA384 failed", __func__);
			return -EINVAL;
		}
		hashlen = 48;
		break;
	case MBEDTLS_MD_SHA512:
		if (mbedtls_sha512(msg_buf, msg_size/8, mdsum, 0) != 0) {
			shell_print(shell, "%s: SHA512 failed", __func__);
			return -EINVAL;
		}
		hashlen = 64;
		break;
	default:
		shell_print(shell, "%s: md %d is not supported", __func__, md_alg);
		return -EINVAL;
	}

#if DEBUG
	shell_print(shell, "%s: Print mdsum(%d):", __func__, hashlen);
	print_buffer(shell, mdsum, hashlen);
#endif
	/* Do RSA signature pkcs1 */
	if (ssa == RSA_PKCS1_V15)
		ret = mbedtls_rsa_rsassa_pkcs1_v15_sign(&rsa, myrand, NULL,
				md_alg, hashlen,
				mdsum, sign_buf);
	else if (ssa == RSA_PKCS1_V21)
		ret = mbedtls_rsa_rsassa_pss_sign_ext(&rsa, myrand, NULL,
				md_alg, hashlen,
				mdsum, 20, sign_buf);

	if (ret) {
		shell_print(shell, "%s: RSA SigGen ssa %d failed !", __func__, (int)ssa);
		goto cleanup;
	} else {
		shell_print(shell, "%s: RSA SigGen ssa %d PASS !", __func__, (int)ssa);
	}

	shell_fprintf(shell, SHELL_NORMAL, "Output:");
	for (int i = 0; i < n_size/8; i++)
		shell_fprintf(shell, SHELL_NORMAL, "%02x", sign_buf[i]);

cleanup:
	if (ret)
		shell_print(shell, "ret:0x%x", ret);

	memset(inputbuf, 0, offset);
	mbedtls_mpi_free(&K);
	mbedtls_rsa_free(&rsa);

	return ret;
}

static int rsa_verify_test(const struct shell *shell, size_t argc, char **argv,
			   enum rsa_ssa ssa)
{
	uint32_t n_size, e_size, msg_size, sign_size;
	uint8_t *n_buf, *e_buf, *msg_buf, *sign_buf;
	uint8_t mdsum[64] = {0};
	mbedtls_md_type_t md_alg;
	mbedtls_rsa_context rsa;
	mbedtls_mpi K;
	int hashlen = 0;
	int offset;
	int ret;

	argc--;
	argv++;

	mbedtls_mpi_init(&K);
	mbedtls_rsa_init(&rsa);

	/* hash algo */
	shell_print(shell, "%s: hash: %s", __func__, argv[0]);
	if (!strcmp(argv[0], "sha1")) {
		md_alg = MBEDTLS_MD_SHA1;
	} else if (!strcmp(argv[0], "sha224")) {
		md_alg = MBEDTLS_MD_SHA224;
	} else if (!strcmp(argv[0], "sha256")) {
		md_alg = MBEDTLS_MD_SHA256;
	} else if (!strcmp(argv[0], "sha384")) {
		md_alg = MBEDTLS_MD_SHA384;
	} else if (!strcmp(argv[0], "sha512")) {
		md_alg = MBEDTLS_MD_SHA512;
	} else {
		shell_print(shell, "%s: hash %s is not supported", __func__, argv[0]);
		return -EINVAL;
	}

	/* set padding */
	if (ssa == RSA_PKCS1_V15)
		mbedtls_rsa_set_padding(&rsa, MBEDTLS_RSA_PKCS_V15, md_alg);
	else if (ssa == RSA_PKCS1_V21)
		mbedtls_rsa_set_padding(&rsa, MBEDTLS_RSA_PKCS_V21, md_alg);

	shell_print(shell, "%s: md_alg: %d", __func__, (int)md_alg);
	argc--;
	argv++;

	/* key n */
	n_size = strtol(argv[0], NULL, 10);	/* bits */
	n_buf = inputbuf;
	offset = n_size/8;
#if DEBUG
	shell_print(shell, "%s: Print n:", __func__);
	print_buffer(shell, n_buf, n_size/8);
#endif
	MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(&K, n_buf, n_size/8));
	MBEDTLS_MPI_CHK(mbedtls_rsa_import(&rsa, &K, NULL, NULL, NULL, NULL));

	argc--;
	argv++;

	/* key e */
	e_size = strtol(argv[0], NULL, 10);	/* bits */
	e_buf = inputbuf + offset;
	offset += e_size/8;
#if DEBUG
	shell_print(shell, "%s: Print e:", __func__);
	print_buffer(shell, e_buf, e_size/8);
#endif
	MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(&K, e_buf, e_size/8));
	MBEDTLS_MPI_CHK(mbedtls_rsa_import(&rsa, NULL, NULL, NULL, NULL, &K));

	argc--;
	argv++;

	/* msg */
	msg_size = strtol(argv[0], NULL, 10);	/* bits */
	msg_buf = inputbuf + offset;
	offset += msg_size/8;
#if DEBUG
	shell_print(shell, "%s: Print msg:", __func__);
	print_buffer(shell, msg_buf, msg_size/8);
#endif
	argc--;
	argv++;

	/* signature */
	sign_size = strtol(argv[0], NULL, 10);	/* bits */
	sign_buf = inputbuf + offset;
	offset += sign_size/8;
#if DEBUG
	shell_print(shell, "%s: Print signature:", __func__);
	print_buffer(shell, sign_buf, sign_size/8);
#endif
	argc--;
	argv++;

	MBEDTLS_MPI_CHK(mbedtls_rsa_complete(&rsa));

	if (recv_len != offset) {
		shell_print(shell, "Input data is not match. recv:%d bytes, offset:%d bytes",
			    recv_len, offset);
		recv_len = 0;
		return -EINVAL;
	}

	recv_len = 0;

	if (mbedtls_rsa_check_pubkey(&rsa)) {
		shell_print(shell, "%s: Check RSA key failed", __func__);
		return -EINVAL;
	}

	/* Calculate message digest */
	switch (md_alg) {
	case MBEDTLS_MD_SHA1:
		if (mbedtls_sha1(msg_buf, msg_size/8, mdsum) != 0) {
			shell_print(shell, "%s: SHA1 failed", __func__);
			return -EINVAL;
		}
		hashlen = 20;
		break;
	case MBEDTLS_MD_SHA224:
		if (mbedtls_sha256(msg_buf, msg_size/8, mdsum, 1) != 0) {
			shell_print(shell, "%s: SHA224 failed", __func__);
			return -EINVAL;
		}
		hashlen = 28;
		break;
	case MBEDTLS_MD_SHA256:
		if (mbedtls_sha256(msg_buf, msg_size/8, mdsum, 0) != 0) {
			shell_print(shell, "%s: SHA256 failed", __func__);
			return -EINVAL;
		}
		hashlen = 32;
		break;
	case MBEDTLS_MD_SHA384:
		if (mbedtls_sha512(msg_buf, msg_size/8, mdsum, 1) != 0) {
			shell_print(shell, "%s: SHA384 failed", __func__);
			return -EINVAL;
		}
		hashlen = 48;
		break;
	case MBEDTLS_MD_SHA512:
		if (mbedtls_sha512(msg_buf, msg_size/8, mdsum, 0) != 0) {
			shell_print(shell, "%s: SHA512 failed", __func__);
			return -EINVAL;
		}
		hashlen = 64;
		break;
	default:
		shell_print(shell, "%s: md %d is not supported", __func__, md_alg);
		return -EINVAL;
	}

#if DEBUG
	shell_print(shell, "%s: Print mdsum:", __func__);
	print_buffer(shell, mdsum, 64);
#endif
	/* Do RSA pkcs1 verification */
	ret = mbedtls_rsa_pkcs1_verify(&rsa, md_alg, hashlen,
				       mdsum, sign_buf);
	if (ret) {
		shell_print(shell, "%s: RSA SigVer ssa %d failed !", __func__, (int)ssa);
	} else {
		shell_print(shell, "%s: RSA SigVer ssa %d PASS !", __func__, (int)ssa);
	}

	shell_print(shell, "Output:%d", ret);

cleanup:
	mbedtls_mpi_free(&K);
	mbedtls_rsa_free(&rsa);

	return 0;
}

static int rsa_pkcs1_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s: argc: %d", __func__, argc);

	argc--;
	argv++;

	/* sign/verify */
	if (!strcmp(argv[0], "sign"))
		return rsa_sign_test(shell, argc, argv, RSA_PKCS1_V15);
	else if (!strcmp(argv[0], "verify"))
		return rsa_verify_test(shell, argc, argv, RSA_PKCS1_V15);
	else
		return -EINVAL;
}

static int rsa_pss_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	argc--;
	argv++;

	/* sign/verify */
	if (!strcmp(argv[0], "sign"))
		return rsa_sign_test(shell, argc, argv, RSA_PKCS1_V21);
	else if (!strcmp(argv[0], "verify"))
		return rsa_verify_test(shell, argc, argv, RSA_PKCS1_V21);
	else
		return -EINVAL;
}

static int cavp_init(const struct shell *shell, size_t argc, char **argv)
{
	int ret;

	ret = usb_init(shell);

	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(cavp_cmds,
	/* hash */
	SHELL_CMD_ARG(sha1, NULL, "", sha1_test, 1, 2),
	SHELL_CMD_ARG(sha224, NULL, "", sha224_test, 1, 2),
	SHELL_CMD_ARG(sha256, NULL, "", sha256_test, 1, 2),
	SHELL_CMD_ARG(sha384, NULL, "", sha384_test, 1, 2),
	SHELL_CMD_ARG(sha512, NULL, "", sha512_test, 1, 2),
	SHELL_CMD_ARG(sha512_224, NULL, "", sha512_224_test, 1, 2),
	SHELL_CMD_ARG(sha512_256, NULL, "", sha512_256_test, 1, 2),
	SHELL_CMD_ARG(hmac-sha1, NULL, "", hmac_sha1_test, 1, 5),
	SHELL_CMD_ARG(hmac-sha224, NULL, "", hmac_sha224_test, 1, 5),
	SHELL_CMD_ARG(hmac-sha256, NULL, "", hmac_sha256_test, 1, 5),
	SHELL_CMD_ARG(hmac-sha384, NULL, "", hmac_sha384_test, 1, 5),
	SHELL_CMD_ARG(hmac-sha512, NULL, "", hmac_sha512_test, 1, 5),
	SHELL_CMD_ARG(hmac-sha512_224, NULL, "", hmac_sha512_224_test, 1, 5),
	SHELL_CMD_ARG(hmac-sha512_256, NULL, "", hmac_sha512_256_test, 1, 5),

	/* aes */
	SHELL_CMD_ARG(aes-ecb, NULL, "", aes_ecb_test, 5, 5),
	SHELL_CMD_ARG(aes-cbc, NULL, "", aes_cbc_test, 5, 5),
	SHELL_CMD_ARG(aes-cfb, NULL, "", aes_cfb_test, 5, 5),
	SHELL_CMD_ARG(aes-ofb, NULL, "", aes_ofb_test, 5, 5),
	SHELL_CMD_ARG(aes-ctr, NULL, "", aes_ctr_test, 5, 5),
	SHELL_CMD_ARG(tdes-ecb, NULL, "", tdes_ecb_test, 5, 5),
	SHELL_CMD_ARG(tdes-cbc, NULL, "", tdes_cbc_test, 5, 5),
	SHELL_CMD_ARG(tdes-cfb, NULL, "", tdes_cfb_test, 5, 5),
	SHELL_CMD_ARG(tdes-ofb, NULL, "", tdes_ofb_test, 5, 5),

	/* rsa */
	SHELL_CMD_ARG(rsa-pkcs1, NULL, "", rsa_pkcs1_test, 6, 8),
	SHELL_CMD_ARG(rsa-pss, NULL, "", rsa_pss_test, 6, 8),

	/* ecdsa */
	SHELL_CMD_ARG(ecdsa, NULL, "", ecdsa_verify_test, 6, 6),

	SHELL_CMD(init, NULL, "", cavp_init),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(cavp, &cavp_cmds, "CAVP shell commands", NULL);
