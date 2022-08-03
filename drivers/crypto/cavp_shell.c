/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <crypto/hash.h>
#include <crypto/cipher.h>
#include <device.h>
#include <drivers/uart.h>
#include <shell/shell.h>
#include <stdio.h>
#include <stdlib.h>
#include <soc.h>
#include <sys/ring_buffer.h>
#include <usb/usb_device.h>

#ifdef CONFIG_CRYPTO_ASPEED
#define HASH_DRV_NAME		CONFIG_CRYPTO_ASPEED_HASH_DRV_NAME
#define CRYPTO_DRV_NAME		CONFIG_CRYPTO_ASPEED_DRV_NAME
#else
#error "You need to enable ASPEED crypto device"
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

RING_BUF_DECLARE(databuf, RING_BUF_SIZE);

static bool is_usb_init;
static int input_len;		/* bits */
static int recv_len;		/* bytes */
static uint8_t inputbuf_hex[RX_BUFF_SIZE] NON_CACHED_BSS;
static uint8_t inputbuf[RX_BUFF_SIZE/2] NON_CACHED_BSS;

uint32_t crypto_cap_flags;

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
				printk("%02x", in[i]);
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

static int _sha_test(const struct shell *shell, const struct hash_testvec *tv, int tv_len,
		     enum hash_algo algo)
{
	const struct device *dev = device_get_binding(HASH_DRV_NAME);
	uint8_t digest[64];
	int ret;
	int i;

	struct hash_ctx ini;
	struct hash_pkt pkt;

	for (i = 0; i < tv_len; i++) {
		pkt.in_buf = (uint8_t *)tv[i].plaintext;
		pkt.in_len = tv[i].psize;
		pkt.out_buf = digest;
		pkt.out_buf_max = sizeof(digest);

		ret = hash_begin_session(dev, &ini, algo);
		if (ret) {
			shell_print(shell, "hash_begin_session error");
			goto out;
		}

		ret = hash_update(&ini, &pkt);
		if (ret) {
			shell_print(shell, "hash_update error");
			goto out;
		}

		ret = hash_final(&ini, &pkt);
		if (ret) {
			shell_print(shell, "hash_final error");
			goto out;
		}

		hash_free_session(dev, &ini);

		shell_fprintf(shell, SHELL_NORMAL, "\ndigest:");
		for (int j = 0; j < ini.digest_size; j++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02x", digest[j]);
		}
		shell_print(shell, "\n");
	}

	return 0;
out:
	hash_free_session(dev, &ini);
	return ret;
}

/*
 * argv[1]: Input Data Size
 * Data would transfer through USB-Uart
 */
static int sha_test_param(const struct shell *shell, enum hash_algo algo,
			  size_t argc, char **argv)
{
	struct hash_testvec tv[1];
	char *buffer;

	if (argc != 2) {
		shell_print(shell, "Wrong parameters");
		return -EINVAL;
	}

	if (!is_usb_init) {
		shell_print(shell, "USB is not initialized");
		return -EINVAL;
	}

	input_len = strtol(argv[1], NULL, 10);
	tv[0].psize = input_len/8;

	if ((recv_len * 8) < input_len) {
		shell_print(shell, "Input is not enough. recv:%d btyes, input:%d bits",
			    recv_len, input_len);
		recv_len = 0;
		return -EINVAL;
		/* padding 0 */
	}

	if (tv[0].psize == 0)
		tv[0].plaintext = "";
	else {
		buffer = malloc(tv[0].psize);
		tv[0].plaintext = inputbuf;
	}

	recv_len = 0;

	return _sha_test(shell, tv, 1, algo);
}

static int sha512_256_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return sha_test_param(shell, HASH_SHA512_256, argc, argv);
}

static int sha512_224_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return sha_test_param(shell, HASH_SHA512_224, argc, argv);
}

static int sha512_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return sha_test_param(shell, HASH_SHA512, argc, argv);
}

static int sha384_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return sha_test_param(shell, HASH_SHA384, argc, argv);
}

static int sha256_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return sha_test_param(shell, HASH_SHA256, argc, argv);
}

static int sha224_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return sha_test_param(shell, HASH_SHA224, argc, argv);
}

static int sha1_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);

	return sha_test_param(shell, HASH_SHA1, argc, argv);
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

	return crypto_test_param(shell, argc, argv, algo, mode, op);
}

static int aes_ecb_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);
	if (argc != 6) {
		shell_print(shell, "%s: Wrong parameters, argc %d", __func__, argc);
		return -EINVAL;
	}

	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_ECB);
}

static int aes_cbc_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);
	if (argc != 8) {
		shell_print(shell, "%s: Wrong parameters, argc %d", __func__, argc);
		return -EINVAL;
	}

	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_CBC);
}

static int aes_cfb_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);
	if (argc != 8) {
		shell_print(shell, "%s: Wrong parameters, argc %d", __func__, argc);
		return -EINVAL;
	}

	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_CFB);
}

static int aes_ofb_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);
	if (argc != 8) {
		shell_print(shell, "%s: Wrong parameters, argc %d", __func__, argc);
		return -EINVAL;
	}

	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_OFB);
}

static int aes_ctr_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);
	if (argc != 8) {
		shell_print(shell, "%s: Wrong parameters, argc %d", __func__, argc);
		return -EINVAL;
	}

	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_AES,
			CRYPTO_CIPHER_MODE_CTR);
}

static int tdes_ecb_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);
	if (argc != 6) {
		shell_print(shell, "%s: Wrong parameters, argc %d", __func__, argc);
		return -EINVAL;
	}

	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_TDES,
			CRYPTO_CIPHER_MODE_ECB);
}

static int tdes_cbc_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);
	if (argc != 8) {
		shell_print(shell, "%s: Wrong parameters, argc %d", __func__, argc);
		return -EINVAL;
	}

	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_TDES,
			CRYPTO_CIPHER_MODE_CBC);
}

static int tdes_cfb_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);
	if (argc != 8) {
		shell_print(shell, "%s: Wrong parameters, argc %d", __func__, argc);
		return -EINVAL;
	}

	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_TDES,
			CRYPTO_CIPHER_MODE_CFB);
}

static int tdes_ofb_test(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "%s", __func__);
	if (argc != 8) {
		shell_print(shell, "%s: Wrong parameters, argc %d", __func__, argc);
		return -EINVAL;
	}

	return aes_test(shell, argc, argv, CRYPTO_CIPHER_ALGO_TDES,
			CRYPTO_CIPHER_MODE_OFB);
}

static int cavp_init(const struct shell *shell, size_t argc, char **argv)
{
	int ret;

	ret = usb_init(shell);

	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(cavp_cmds,
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

	SHELL_CMD_ARG(aes-ecb, NULL, "", aes_ecb_test, 5, 5),
	SHELL_CMD_ARG(aes-cbc, NULL, "", aes_cbc_test, 5, 5),
	SHELL_CMD_ARG(aes-cfb, NULL, "", aes_cfb_test, 5, 5),
	SHELL_CMD_ARG(aes-ofb, NULL, "", aes_ofb_test, 5, 5),
	SHELL_CMD_ARG(aes-ctr, NULL, "", aes_ctr_test, 5, 5),
	SHELL_CMD_ARG(tdes-ecb, NULL, "", tdes_ecb_test, 5, 5),
	SHELL_CMD_ARG(tdes-cbc, NULL, "", tdes_cbc_test, 5, 5),
	SHELL_CMD_ARG(tdes-cfb, NULL, "", tdes_cfb_test, 5, 5),
	SHELL_CMD_ARG(tdes-ofb, NULL, "", tdes_ofb_test, 5, 5),

	SHELL_CMD(init, NULL, "", cavp_init),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(cavp, &cavp_cmds, "CAVP shell commands", NULL);
