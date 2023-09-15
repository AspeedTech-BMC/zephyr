/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

/**
 * @file
 * @brief AES shell commands.
 */

#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/cipher.h>
#include <zephyr/shell/shell.h>

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(crypto);

#ifdef CONFIG_CRYPTO_ASPEED
#define CRYPTO_DRV_NAME CONFIG_CRYPTO_ASPEED_DRV_NAME
#else
#error "You need to enable ASPEED crypto device"
#endif

struct cipher_testvec {
	const char *key;
	const char *iv;
	const char *iv_out;
	const char *ptext;
	const char *ctext;
	unsigned short klen;
	unsigned int len;
};

uint32_t cap_flags;

static const struct cipher_testvec aes256cbc_tv[] = {
	{
		.key	=
		"\x60\x3d\xeb\x10\x15\xca\x71\xbe"
		"\x2b\x73\xae\xf0\x85\x7d\x77\x81"
		"\x1f\x35\x2c\x07\x3b\x61\x08\xd7"
		"\x2d\x98\x10\xa3\x09\x14\xdf\xf4",
		.klen	= 32,
		.iv	=
		"\x00\x01\x02\x03\x04\x05\x06\x07"
		"\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f",
		.iv_out	=
		"\xb2\xeb\x05\xe2\xc3\x9b\xe9\xfc"
		"\xda\x6c\x19\x07\x8c\x6a\x9d\x1b",
		.ptext	=
		"\x6b\xc1\xbe\xe2\x2e\x40\x9f\x96"
		"\xe9\x3d\x7e\x11\x73\x93\x17\x2a"
		"\xae\x2d\x8a\x57\x1e\x03\xac\x9c"
		"\x9e\xb7\x6f\xac\x45\xaf\x8e\x51"
		"\x30\xc8\x1c\x46\xa3\x5c\xe4\x11"
		"\xe5\xfb\xc1\x19\x1a\x0a\x52\xef"
		"\xf6\x9f\x24\x45\xdf\x4f\x9b\x17"
		"\xad\x2b\x41\x7b\xe6\x6c\x37\x10",
		.ctext	=
		"\xf5\x8c\x4c\x04\xd6\xe5\xf1\xba"
		"\x77\x9e\xab\xfb\x5f\x7b\xfb\xd6"
		"\x9c\xfc\x4e\x96\x7e\xdb\x80\x8d"
		"\x67\x9f\x77\x7b\xc6\x70\x2c\x7d"
		"\x39\xf2\x33\x69\xa9\xd9\xba\xcf"
		"\xa5\x30\xe2\x63\x04\x23\x14\x61"
		"\xb2\xeb\x05\xe2\xc3\x9b\xe9\xfc"
		"\xda\x6c\x19\x07\x8c\x6a\x9d\x1b",
		.len	= 64,
	},
};

static void print_buffer_comparison(const uint8_t *wanted_result,
				    uint8_t *result, size_t length)
{
	int i, j;

	printk("Was waiting for:\n");

	for (i = 0, j = 1; i < length; i++, j++) {
		printk("%02x ", wanted_result[i]);

		if (j == 0x10) {
			printk("\n");
			j = 0;
		}
	}

	printk("\n But got:\n");

	for (i = 0, j = 1; i < length; i++, j++) {
		printk("%02x ", result[i]);

		if (j == 0x10) {
			printk("\n");
			j = 0;
		}
	}

	printk("\n");
}

static int validate_hw_compatibility(const struct device *dev)
{
	uint32_t flags = 0U;

	flags = crypto_query_hwcaps(dev);
	if ((flags & CAP_RAW_KEY) == 0U) {
		LOG_INF("Please provision the key separately "
				"as the module doesn't support a raw key");
		return -1;
	}

	if ((flags & CAP_SYNC_OPS) == 0U) {
		LOG_ERR("The app assumes sync semantics. "
				"Please rewrite the app accordingly before proceeding");
		return -1;
	}

	if ((flags & CAP_SEPARATE_IO_BUFS) == 0U) {
		LOG_ERR("The app assumes distinct IO buffers. "
				"Please rewrite the app accordingly before proceeding");
		return -1;
	}

	cap_flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

	return 0;

}

void aes_cbc(const struct device *dev)
{
	uint8_t encrypted[80] = {0};
	uint8_t decrypted[64] = {0};
	struct cipher_ctx ini;
	struct cipher_pkt encrypt, decrypt;
	int i;

	for (i = 0; i < ARRAY_SIZE(aes256cbc_tv); i++) {
		ini.keylen = aes256cbc_tv[i].klen;
		ini.key.bit_stream = (uint8_t *)aes256cbc_tv[i].key;
		ini.flags = cap_flags;
		encrypt.in_buf = (uint8_t *)aes256cbc_tv[i].ptext;
		encrypt.in_len = aes256cbc_tv[i].len;
		encrypt.out_buf_max = sizeof(encrypted);
		encrypt.out_buf = encrypted;

		decrypt.in_buf = encrypt.out_buf;
		decrypt.in_len = sizeof(encrypted);
		decrypt.out_buf = decrypted;
		decrypt.out_buf_max = sizeof(decrypted);

		if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
					 CRYPTO_CIPHER_MODE_CBC,
					 CRYPTO_CIPHER_OP_ENCRYPT)) {
			return;
		}
		if (cipher_cbc_op(&ini, &encrypt, (uint8_t *)aes256cbc_tv[i].iv)) {
			LOG_ERR("CBC mode ENCRYPT - Failed");
			goto out;
		}
		LOG_INF("Output length (encryption): %d", encrypt.out_len);
		/* First 16 byte is IV, which is default behavior of Zephyr
		 * crypto API.
		 */
		if (memcmp(encrypt.out_buf + 16, aes256cbc_tv[i].ctext, aes256cbc_tv[i].len)) {
			LOG_ERR("CBC mode ENCRYPT DATA - Mismatch between expected and "
					"returned cipher text");
			print_buffer_comparison(aes256cbc_tv[i].ctext, encrypt.out_buf + 16,
						aes256cbc_tv[i].len);
			goto out;
		}

		LOG_INF("CBC mode ENCRYPT - Match");
		cipher_free_session(dev, &ini);

		if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
					 CRYPTO_CIPHER_MODE_CBC,
					 CRYPTO_CIPHER_OP_DECRYPT)) {
			return;
		}

		/* TinyCrypt keeps IV at the start of encrypted buffer */
		if (cipher_cbc_op(&ini, &decrypt, encrypted)) {
			LOG_ERR("CBC mode DECRYPT - Failed");
			goto out;
		}

		LOG_INF("Output length (decryption): %d", decrypt.out_len);

		if (memcmp(decrypt.out_buf, aes256cbc_tv[i].ptext, aes256cbc_tv[i].len)) {
			LOG_ERR("CBC mode DECRYPT - Mismatch between plaintext and "
					"decrypted cipher text");
			print_buffer_comparison(aes256cbc_tv[i].ptext, decrypt.out_buf,
						aes256cbc_tv[i].len);
			goto out;
		}

		LOG_INF("CBC mode DECRYPT - Match");
	}
out:
	cipher_free_session(dev, &ini);
}

void aes_cbc_vault(const struct device *dev)
{
	uint8_t *ptext = (uint8_t *)aes256cbc_tv[0].ptext;
	uint8_t *iv = (uint8_t *)aes256cbc_tv[0].iv;
	unsigned int len = aes256cbc_tv[0].len;
	uint8_t key_id = 1;
	uint8_t encrypted[80] = {0};
	uint8_t decrypted[64] = {0};
	struct cipher_ctx ini;
	struct cipher_pkt encrypt, decrypt;

	ini.keylen = 32;
	ini.key.handle = &key_id;
	ini.flags = CAP_OPAQUE_KEY_HNDL | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	encrypt.in_buf = ptext;
	encrypt.in_len = len;
	encrypt.out_buf_max = sizeof(encrypted);
	encrypt.out_buf = encrypted;

	decrypt.in_buf = encrypt.out_buf;
	decrypt.in_len = sizeof(encrypted);
	decrypt.out_buf = decrypted;
	decrypt.out_buf_max = sizeof(decrypted);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CBC,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}
	if (cipher_cbc_op(&ini, &encrypt, iv)) {
		LOG_ERR("CBC mode ENCRYPT - Failed");
		goto out;
	}
	LOG_INF("Output length (encryption): %d", encrypt.out_len);
	/* First 16 byte is IV, which is default behavior of Zephyr
	 * crypto API.
	 */
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CBC,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	/* TinyCrypt keeps IV at the start of encrypted buffer */
	if (cipher_cbc_op(&ini, &decrypt, encrypted)) {
		LOG_ERR("CBC mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, ptext, len)) {
		LOG_ERR("CBC mode DECRYPT - Mismatch between plaintext and "
				"decrypted cipher text");
		print_buffer_comparison(ptext, decrypt.out_buf,
					len);
		goto out;
	}

	LOG_INF("CBC mode ENCRYPT and DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

static void aes_test(const struct device *dev, enum cipher_mode mode, int vault_key)
{
	switch (mode) {
	case CRYPTO_CIPHER_MODE_CBC:
		if (vault_key)
			aes_cbc_vault(dev);
		else
			aes_cbc(dev);
		break;
	default:
		LOG_ERR("Incompatible mode");
	}
}

static void _crypto_test(const struct shell *shell, enum cipher_algo algo,
			 enum cipher_mode mode, int vault)
{
	const struct device *dev = device_get_binding(CRYPTO_DRV_NAME);

	if (validate_hw_compatibility(dev)) {
		LOG_ERR("Incompatible h/w");
		return;
	}

	switch (algo) {
	case CRYPTO_CIPHER_ALGO_AES:
		aes_test(dev, mode, vault);
		break;
	default:
		LOG_ERR("Incompatible algo");
	}
}

static void aes256_cbc(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "aes256_cbc");
	_crypto_test(shell, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_CBC, 0);
}

static void aes256_cbc_vault(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "aes256_cbc vault key 1");
	_crypto_test(shell, CRYPTO_CIPHER_ALGO_AES, CRYPTO_CIPHER_MODE_CBC, 1);
}

SHELL_STATIC_SUBCMD_SET_CREATE(crypto_cmds,
			       SHELL_CMD_ARG(aes256_cbc, NULL, "", aes256_cbc, 1, 0),
			       SHELL_CMD_ARG(aes256_cbc_vault, NULL, "", aes256_cbc_vault, 1, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(crypto, &crypto_cmds, "Crypto shell commands", NULL);
