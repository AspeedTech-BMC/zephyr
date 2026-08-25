/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>

LOG_MODULE_REGISTER(cptra_mci_cryptographic_mbox, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Caliptra-SS's MC_IMPORT validation (RUNTIME_CMB_INVALID_KEY_USAGE_AND_SIZE
 * in the RT firmware) accepts exactly one or two input_size values per
 * key_usage: Aes/Mldsa -> 32, Ecdsa -> 48, Hmac -> 48 or 64, Mlkem -> 64.
 * Enforced here so a bad combination fails locally instead of burning a
 * mailbox round trip.
 */
static bool cptra_mci_import_key_size_valid(enum cptra_mci_key_usage key_usage, size_t key_len)
{
	switch (key_usage) {
	case CPTRA_MCI_KEY_USAGE_AES:
	case CPTRA_MCI_KEY_USAGE_MLDSA:
		return key_len == 32;
	case CPTRA_MCI_KEY_USAGE_ECDSA:
		return key_len == 48;
	case CPTRA_MCI_KEY_USAGE_HMAC:
		return key_len == 48 || key_len == 64;
	case CPTRA_MCI_KEY_USAGE_MLKEM:
		return key_len == 64;
	default:
		return false;
	}
}

int cptra_mci_cm_status(uint32_t *used_usage_storage, uint32_t *total_usage_storage)
{
	struct cptra_mci_cm_status_req req = { 0 };
	struct cptra_mci_cm_status_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_CM_STATUS, NULL, 0);

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_CM_STATUS, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_CM_STATUS failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_CM_STATUS response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	if (used_usage_storage)
		*used_usage_storage = resp.used_usage_storage;
	if (total_usage_storage)
		*total_usage_storage = resp.total_usage_storage;

	return 0;
}

int cptra_mci_random_stir(const uint8_t *input, size_t input_len)
{
	struct cptra_mci_random_stir_hdr req = {
		.input_size = input_len,
	};
	struct cptra_mci_random_stir_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (input_len == 0 || input_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_RANDOM_STIR,
				       &req.input_size, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, input, input_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_RANDOM_STIR, &req, sizeof(req),
				       input, input_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_RANDOM_STIR failed: %d", ret);
		return ret;
	}

	return 0;
}

int cptra_mci_random_generate(uint8_t *data, size_t len, size_t *data_len)
{
	struct cptra_mci_random_generate_req req = {
		.size = len,
	};
	/* data[] alone is CPTRA_MCI_MBOX_MAX_INPUT_SIZE (4096) bytes -- kept off the stack. */
	static struct cptra_mci_random_generate_resp resp;
	uint32_t resp_len;
	uint32_t dlen;
	int ret;

	if (len == 0 || len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_RANDOM_GENERATE,
						 &req.size, sizeof(req) - sizeof(req.hdr));

	/*
	 * resp is static (shared across calls), so the lock has to cover
	 * everything from here through the last read of resp below -- not
	 * just the hardware step inside execute() -- or two concurrent
	 * callers can hand each other's random bytes to one another.
	 */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_RANDOM_GENERATE, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_RANDOM_GENERATE failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_RANDOM_GENERATE response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	dlen = MIN(resp.hdr.data_len, sizeof(resp.data));
	dlen = MIN(dlen, len);

	if (resp_len < sizeof(resp.hdr) + dlen) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_RANDOM_GENERATE response shorter than reported data_len");
		return -EIO;
	}

	memcpy(data, resp.data, dlen);

	cptra_mci_mbox_txn_end();

	if (data_len)
		*data_len = dlen;

	return 0;
}

int cptra_mci_import_key(enum cptra_mci_key_usage key_usage, const uint8_t *key, size_t key_len,
			 uint8_t cmk[CPTRA_MCI_CMK_SIZE])
{
	struct cptra_mci_import_hdr req = {
		.key_usage = key_usage,
		.input_size = key_len,
	};
	struct cptra_mci_import_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (!cptra_mci_import_key_size_valid(key_usage, key_len))
		return -EINVAL;

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_IMPORT,
				       &req.key_usage, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, key, key_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_IMPORT, &req, sizeof(req),
				       key, key_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_IMPORT failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_IMPORT response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(cmk, resp.cmk.value, sizeof(resp.cmk.value));

	return 0;
}

int cptra_mci_ecdh_generate(uint8_t context[CPTRA_MCI_ECDH_CONTEXT_SIZE],
			   uint8_t exchange_data[CPTRA_MCI_ECDH_EXCHANGE_DATA_SIZE])
{
	struct cptra_mci_ecdh_generate_req req = { 0 };
	struct cptra_mci_ecdh_generate_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_ECDH_GENERATE, NULL, 0);

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_ECDH_GENERATE, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_ECDH_GENERATE failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_ECDH_GENERATE response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(context, resp.context, sizeof(resp.context));
	memcpy(exchange_data, resp.exchange_data, sizeof(resp.exchange_data));

	return 0;
}

int cptra_mci_ecdh_finish(const uint8_t context[CPTRA_MCI_ECDH_CONTEXT_SIZE],
			 enum cptra_mci_key_usage key_usage,
			 const uint8_t incoming_exchange_data[CPTRA_MCI_ECDH_EXCHANGE_DATA_SIZE],
			 uint8_t output[CPTRA_MCI_CMK_SIZE])
{
	struct cptra_mci_ecdh_finish_req req = {
		.key_usage = key_usage,
	};
	struct cptra_mci_ecdh_finish_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	memcpy(req.context, context, sizeof(req.context));
	memcpy(req.incoming_exchange_data, incoming_exchange_data,
	       sizeof(req.incoming_exchange_data));

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_ECDH_FINISH,
						 &req.context, sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_ECDH_FINISH, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_ECDH_FINISH failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_ECDH_FINISH response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(output, resp.output.value, sizeof(resp.output.value));

	return 0;
}

int cptra_mci_ecdsa_cmk_public_key(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
				   uint8_t qx[CPTRA_MCI_ECC384_SCALAR_SIZE],
				   uint8_t qy[CPTRA_MCI_ECC384_SCALAR_SIZE])
{
	struct cptra_mci_ecdsa_pubkey_req req = { 0 };
	struct cptra_mci_ecdsa_pubkey_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_ECDSA_CMK_PUBLIC_KEY,
						 &req.cmk, sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_ECDSA_CMK_PUBLIC_KEY, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_ECDSA_CMK_PUBLIC_KEY failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_ECDSA_CMK_PUBLIC_KEY response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(qx, resp.qx, sizeof(resp.qx));
	memcpy(qy, resp.qy, sizeof(resp.qy));

	return 0;
}

int cptra_mci_ecdsa_cmk_sign(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
			    const uint8_t *message, size_t message_len,
			    uint8_t r[CPTRA_MCI_ECC384_SCALAR_SIZE],
			    uint8_t s[CPTRA_MCI_ECC384_SCALAR_SIZE])
{
	struct cptra_mci_ecdsa_sign_hdr req = {
		.message_size = message_len,
	};
	struct cptra_mci_ecdsa_sign_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (message_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_ECDSA_CMK_SIGN,
				       &req.cmk, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, message, message_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_ECDSA_CMK_SIGN, &req, sizeof(req),
				       message, message_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_ECDSA_CMK_SIGN failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_ECDSA_CMK_SIGN response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(r, resp.r, sizeof(resp.r));
	memcpy(s, resp.s, sizeof(resp.s));

	return 0;
}

int cptra_mci_ecdsa_cmk_verify(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
			      const uint8_t r[CPTRA_MCI_ECC384_SCALAR_SIZE],
			      const uint8_t s[CPTRA_MCI_ECC384_SCALAR_SIZE],
			      const uint8_t *message, size_t message_len)
{
	struct cptra_mci_ecdsa_verify_hdr req = {
		.message_size = message_len,
	};
	struct cptra_mci_ecdsa_verify_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (message_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));
	memcpy(req.r, r, sizeof(req.r));
	memcpy(req.s, s, sizeof(req.s));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_ECDSA_CMK_VERIFY,
				       &req.cmk, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, message, message_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_ECDSA_CMK_VERIFY, &req, sizeof(req),
				       message, message_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_ECDSA_CMK_VERIFY failed: %d", ret);
		return ret;
	}

	return 0;
}

int cptra_mci_hmac(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], enum cptra_mci_sha_algo hash_algorithm,
		   const uint8_t *data, size_t data_len,
		   uint8_t *mac, size_t mac_buf_len, size_t *mac_len)
{
	struct cptra_mci_hmac_hdr req = {
		.hash_algorithm = hash_algorithm,
		.data_size = data_len,
	};
	struct cptra_mci_hmac_resp resp = { 0 };
	uint32_t resp_len, csum, len;
	int ret;

	if (data_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE || mac_buf_len == 0)
		return -EINVAL;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_HMAC,
				       &req.cmk, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, data, data_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_HMAC, &req, sizeof(req),
				       data, data_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_HMAC failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		LOG_ERR("MC_HMAC response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	len = MIN(resp.hdr.data_len, sizeof(resp.mac));
	len = MIN(len, mac_buf_len);

	if (resp_len < sizeof(resp.hdr) + len) {
		LOG_ERR("MC_HMAC response shorter than reported data_len");
		return -EIO;
	}

	memcpy(mac, resp.mac, len);

	if (mac_len)
		*mac_len = len;

	return 0;
}

int cptra_mci_hmac_kdf_counter(const uint8_t kin[CPTRA_MCI_CMK_SIZE],
			       enum cptra_mci_sha_algo hash_algorithm,
			       enum cptra_mci_key_usage key_usage, uint32_t key_size,
			       const uint8_t *label, size_t label_len,
			       uint8_t kout[CPTRA_MCI_CMK_SIZE])
{
	struct cptra_mci_hmac_kdf_counter_hdr req = {
		.hash_algorithm = hash_algorithm,
		.key_usage = key_usage,
		.key_size = key_size,
		.label_size = label_len,
	};
	struct cptra_mci_hmac_kdf_counter_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (label_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.kin.value, kin, sizeof(req.kin.value));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_HMAC_KDF_COUNTER,
				       &req.kin, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, label, label_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_HMAC_KDF_COUNTER, &req, sizeof(req),
				       label, label_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_HMAC_KDF_COUNTER failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_HMAC_KDF_COUNTER response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(kout, resp.kout.value, sizeof(resp.kout.value));

	return 0;
}

int cptra_mci_hkdf_extract(enum cptra_mci_sha_algo hash_algorithm,
			   const uint8_t salt[CPTRA_MCI_CMK_SIZE],
			   const uint8_t ikm[CPTRA_MCI_CMK_SIZE],
			   uint8_t prk[CPTRA_MCI_CMK_SIZE])
{
	struct cptra_mci_hkdf_extract_req req = {
		.hash_algorithm = hash_algorithm,
	};
	struct cptra_mci_hkdf_extract_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	memcpy(req.salt.value, salt, sizeof(req.salt.value));
	memcpy(req.ikm.value, ikm, sizeof(req.ikm.value));

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_HKDF_EXTRACT,
						 &req.hash_algorithm,
						 sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_HKDF_EXTRACT, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_HKDF_EXTRACT failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_HKDF_EXTRACT response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(prk, resp.prk.value, sizeof(resp.prk.value));

	return 0;
}

int cptra_mci_hkdf_expand(const uint8_t prk[CPTRA_MCI_CMK_SIZE],
			  enum cptra_mci_sha_algo hash_algorithm,
			  enum cptra_mci_key_usage key_usage, uint32_t key_size,
			  const uint8_t *info, size_t info_len,
			  uint8_t okm[CPTRA_MCI_CMK_SIZE])
{
	struct cptra_mci_hkdf_expand_hdr req = {
		.hash_algorithm = hash_algorithm,
		.key_usage = key_usage,
		.key_size = key_size,
		.info_size = info_len,
	};
	struct cptra_mci_hkdf_expand_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (info_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.prk.value, prk, sizeof(req.prk.value));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_HKDF_EXPAND,
				       &req.prk, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, info, info_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_HKDF_EXPAND, &req, sizeof(req),
				       info, info_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_HKDF_EXPAND failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_HKDF_EXPAND response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(okm, resp.okm.value, sizeof(resp.okm.value));

	return 0;
}

int cptra_mci_aes_encrypt_init(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], enum cptra_mci_aes_mode mode,
			      const uint8_t *plaintext, size_t plaintext_len,
			      uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE],
			      uint8_t iv[CPTRA_MCI_AES_IV_SIZE],
			      uint8_t *ciphertext, size_t ciphertext_buf_len,
			      size_t *ciphertext_len)
{
	struct cptra_mci_aes_encrypt_init_hdr req = {
		.mode = mode,
		.plaintext_size = plaintext_len,
	};
	/* output[] alone is CPTRA_MCI_MBOX_MAX_INPUT_SIZE (4096) bytes -- kept off the stack. */
	static struct cptra_mci_aes_init_resp resp;
	uint32_t resp_len, csum, len;
	int ret;

	if (plaintext_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_ENCRYPT_INIT,
				       &req.cmk, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, plaintext, plaintext_len);

	/* resp is static -- see the comment on cptra_mci_random_generate() above. */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_ENCRYPT_INIT, &req, sizeof(req),
				       plaintext, plaintext_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_ENCRYPT_INIT failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_ENCRYPT_INIT response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	len = MIN(resp.hdr.output_size, sizeof(resp.output));
	len = MIN(len, ciphertext_buf_len);

	if (resp_len < sizeof(resp.hdr) + len) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_ENCRYPT_INIT response shorter than reported output_size");
		return -EIO;
	}

	memcpy(context, resp.hdr.context, sizeof(resp.hdr.context));
	memcpy(iv, resp.hdr.iv, sizeof(resp.hdr.iv));
	memcpy(ciphertext, resp.output, len);

	cptra_mci_mbox_txn_end();

	if (ciphertext_len)
		*ciphertext_len = len;

	return 0;
}

int cptra_mci_aes_encrypt_update(uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE],
				const uint8_t *plaintext, size_t plaintext_len,
				uint8_t *ciphertext, size_t ciphertext_buf_len,
				size_t *ciphertext_len)
{
	struct cptra_mci_aes_update_hdr req = {
		.size = plaintext_len,
	};
	static struct cptra_mci_aes_resp resp;
	uint32_t resp_len, csum, len;
	int ret;

	if (plaintext_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.context, context, sizeof(req.context));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_ENCRYPT_UPDATE,
				       &req.context, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, plaintext, plaintext_len);

	/* resp is static -- see the comment on cptra_mci_random_generate() above. */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_ENCRYPT_UPDATE, &req, sizeof(req),
				       plaintext, plaintext_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_ENCRYPT_UPDATE failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_ENCRYPT_UPDATE response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	len = MIN(resp.hdr.output_size, sizeof(resp.output));
	len = MIN(len, ciphertext_buf_len);

	if (resp_len < sizeof(resp.hdr) + len) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_ENCRYPT_UPDATE response shorter than reported output_size");
		return -EIO;
	}

	memcpy(context, resp.hdr.context, sizeof(resp.hdr.context));
	memcpy(ciphertext, resp.output, len);

	cptra_mci_mbox_txn_end();

	if (ciphertext_len)
		*ciphertext_len = len;

	return 0;
}

int cptra_mci_aes_decrypt_init(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], enum cptra_mci_aes_mode mode,
			      const uint8_t iv[CPTRA_MCI_AES_IV_SIZE],
			      const uint8_t *ciphertext, size_t ciphertext_len,
			      uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE],
			      uint8_t *plaintext, size_t plaintext_buf_len,
			      size_t *plaintext_len)
{
	struct cptra_mci_aes_decrypt_init_hdr req = {
		.mode = mode,
		.ciphertext_size = ciphertext_len,
	};
	/*
	 * On real hardware this response is shaped like CmAesResp (no iv field) --
	 * NOT like CmAesEncryptInitResp, despite the upstream wrapper type
	 * (McuAesDecryptInitResp(pub CmAesEncryptInitResp), "reuse encrypt init
	 * resp if needed") suggesting otherwise. That extra 16-byte iv field
	 * doesn't exist on the wire here; parsing against it shifted every field
	 * after context by 16 bytes, confirmed against a real hardware capture.
	 */
	static struct cptra_mci_aes_resp resp;
	uint32_t resp_len, csum, len;
	int ret;

	if (ciphertext_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));
	memcpy(req.iv, iv, sizeof(req.iv));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_DECRYPT_INIT,
				       &req.cmk, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, ciphertext, ciphertext_len);

	/* resp is static -- see the comment on cptra_mci_random_generate() above. */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_DECRYPT_INIT, &req, sizeof(req),
				       ciphertext, ciphertext_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_DECRYPT_INIT failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_DECRYPT_INIT response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	len = MIN(resp.hdr.output_size, sizeof(resp.output));
	len = MIN(len, plaintext_buf_len);

	if (resp_len < sizeof(resp.hdr) + len) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_DECRYPT_INIT response shorter than reported output_size");
		return -EIO;
	}

	memcpy(context, resp.hdr.context, sizeof(resp.hdr.context));
	memcpy(plaintext, resp.output, len);

	cptra_mci_mbox_txn_end();

	if (plaintext_len)
		*plaintext_len = len;

	return 0;
}

int cptra_mci_aes_decrypt_update(uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE],
				const uint8_t *ciphertext, size_t ciphertext_len,
				uint8_t *plaintext, size_t plaintext_buf_len,
				size_t *plaintext_len)
{
	struct cptra_mci_aes_update_hdr req = {
		.size = ciphertext_len,
	};
	static struct cptra_mci_aes_resp resp;
	uint32_t resp_len, csum, len;
	int ret;

	if (ciphertext_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.context, context, sizeof(req.context));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_DECRYPT_UPDATE,
				       &req.context, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, ciphertext, ciphertext_len);

	/* resp is static -- see the comment on cptra_mci_random_generate() above. */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_DECRYPT_UPDATE, &req, sizeof(req),
				       ciphertext, ciphertext_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_DECRYPT_UPDATE failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_DECRYPT_UPDATE response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	len = MIN(resp.hdr.output_size, sizeof(resp.output));
	len = MIN(len, plaintext_buf_len);

	if (resp_len < sizeof(resp.hdr) + len) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_DECRYPT_UPDATE response shorter than reported output_size");
		return -EIO;
	}

	memcpy(context, resp.hdr.context, sizeof(resp.hdr.context));
	memcpy(plaintext, resp.output, len);

	cptra_mci_mbox_txn_end();

	if (plaintext_len)
		*plaintext_len = len;

	return 0;
}

int cptra_mci_aes_gcm_encrypt_init(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], uint32_t flags,
				  const uint8_t *aad, size_t aad_len,
				  uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				  uint8_t iv[CPTRA_MCI_AES_GCM_IV_SIZE])
{
	struct cptra_mci_aes_gcm_encrypt_init_hdr req = {
		.flags = flags,
		.aad_size = aad_len,
	};
	struct cptra_mci_aes_gcm_init_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (aad_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_GCM_ENCRYPT_INIT,
				       &req.flags, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, aad, aad_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_GCM_ENCRYPT_INIT, &req, sizeof(req),
				       aad, aad_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_AES_GCM_ENCRYPT_INIT failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_AES_GCM_ENCRYPT_INIT response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(context, resp.context, sizeof(resp.context));
	memcpy(iv, resp.iv, sizeof(resp.iv));

	return 0;
}

int cptra_mci_aes_gcm_encrypt_update(uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				    const uint8_t *plaintext, size_t plaintext_len,
				    uint8_t *ciphertext, size_t ciphertext_buf_len,
				    size_t *ciphertext_len)
{
	struct cptra_mci_aes_gcm_data_hdr req = {
		.size = plaintext_len,
	};
	/* output[] alone is CPTRA_MCI_AES_GCM_MAX_OUTPUT_SIZE (4112) bytes --
	 * kept off the stack.
	 */
	static struct cptra_mci_aes_gcm_update_resp resp;
	uint32_t resp_len, csum, len;
	int ret;

	if (plaintext_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.context, context, sizeof(req.context));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_GCM_ENCRYPT_UPDATE,
				       &req.context, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, plaintext, plaintext_len);

	/* resp is static -- see the comment on cptra_mci_random_generate() above. */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_GCM_ENCRYPT_UPDATE, &req, sizeof(req),
				       plaintext, plaintext_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_ENCRYPT_UPDATE failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_ENCRYPT_UPDATE response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	len = MIN(resp.hdr.output_size, sizeof(resp.output));
	len = MIN(len, ciphertext_buf_len);

	if (resp_len < sizeof(resp.hdr) + len) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_ENCRYPT_UPDATE response shorter than reported output_size");
		return -EIO;
	}

	memcpy(context, resp.hdr.context, sizeof(resp.hdr.context));
	memcpy(ciphertext, resp.output, len);

	cptra_mci_mbox_txn_end();

	if (ciphertext_len)
		*ciphertext_len = len;

	return 0;
}

int cptra_mci_aes_gcm_encrypt_final(uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				   const uint8_t *plaintext, size_t plaintext_len,
				   uint8_t *ciphertext, size_t ciphertext_buf_len,
				   size_t *ciphertext_len,
				   uint8_t tag[CPTRA_MCI_AES_GCM_TAG_SIZE])
{
	struct cptra_mci_aes_gcm_data_hdr req = {
		.size = plaintext_len,
	};
	static struct cptra_mci_aes_gcm_encrypt_final_resp resp;
	uint32_t resp_len, csum, len;
	int ret;

	if (plaintext_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.context, context, sizeof(req.context));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_GCM_ENCRYPT_FINAL,
				       &req.context, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, plaintext, plaintext_len);

	/* resp is static -- see the comment on cptra_mci_random_generate() above. */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_GCM_ENCRYPT_FINAL, &req, sizeof(req),
				       plaintext, plaintext_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_ENCRYPT_FINAL failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_ENCRYPT_FINAL response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	len = MIN(resp.hdr.output_size, sizeof(resp.output));
	len = MIN(len, ciphertext_buf_len);

	if (resp_len < sizeof(resp.hdr) + len) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_ENCRYPT_FINAL response shorter than reported output_size");
		return -EIO;
	}

	memcpy(tag, resp.hdr.tag, sizeof(resp.hdr.tag));
	memcpy(ciphertext, resp.output, len);

	cptra_mci_mbox_txn_end();

	if (ciphertext_len)
		*ciphertext_len = len;

	return 0;
}

int cptra_mci_aes_gcm_decrypt_init(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], uint32_t flags,
				  const uint8_t iv[CPTRA_MCI_AES_GCM_IV_SIZE],
				  const uint8_t *aad, size_t aad_len,
				  uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE])
{
	struct cptra_mci_aes_gcm_decrypt_init_hdr req = {
		.flags = flags,
		.aad_size = aad_len,
	};
	/* Reuses the same wire shape as encrypt_init's response -- see the struct comment. */
	struct cptra_mci_aes_gcm_init_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (aad_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));
	memcpy(req.iv, iv, sizeof(req.iv));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_GCM_DECRYPT_INIT,
				       &req.flags, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, aad, aad_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_GCM_DECRYPT_INIT, &req, sizeof(req),
				       aad, aad_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_AES_GCM_DECRYPT_INIT failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_AES_GCM_DECRYPT_INIT response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(context, resp.context, sizeof(resp.context));

	return 0;
}

int cptra_mci_aes_gcm_decrypt_update(uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				    const uint8_t *ciphertext, size_t ciphertext_len,
				    uint8_t *plaintext, size_t plaintext_buf_len,
				    size_t *plaintext_len)
{
	struct cptra_mci_aes_gcm_data_hdr req = {
		.size = ciphertext_len,
	};
	static struct cptra_mci_aes_gcm_update_resp resp;
	uint32_t resp_len, csum, len;
	int ret;

	if (ciphertext_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.context, context, sizeof(req.context));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_GCM_DECRYPT_UPDATE,
				       &req.context, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, ciphertext, ciphertext_len);

	/* resp is static -- see the comment on cptra_mci_random_generate() above. */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_GCM_DECRYPT_UPDATE, &req, sizeof(req),
				       ciphertext, ciphertext_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_DECRYPT_UPDATE failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_DECRYPT_UPDATE response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	len = MIN(resp.hdr.output_size, sizeof(resp.output));
	len = MIN(len, plaintext_buf_len);

	if (resp_len < sizeof(resp.hdr) + len) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_DECRYPT_UPDATE response shorter than reported output_size");
		return -EIO;
	}

	memcpy(context, resp.hdr.context, sizeof(resp.hdr.context));
	memcpy(plaintext, resp.output, len);

	cptra_mci_mbox_txn_end();

	if (plaintext_len)
		*plaintext_len = len;

	return 0;
}

int cptra_mci_aes_gcm_decrypt_final(uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				   const uint8_t tag[CPTRA_MCI_AES_GCM_TAG_SIZE],
				   const uint8_t *ciphertext, size_t ciphertext_len,
				   uint8_t *plaintext, size_t plaintext_buf_len,
				   size_t *plaintext_len)
{
	struct cptra_mci_aes_gcm_decrypt_final_hdr req = {
		.tag_len = CPTRA_MCI_AES_GCM_TAG_SIZE,
		.ciphertext_size = ciphertext_len,
	};
	static struct cptra_mci_aes_gcm_decrypt_final_resp resp;
	uint32_t resp_len, csum, len;
	int ret;

	if (ciphertext_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.context, context, sizeof(req.context));
	memcpy(req.tag, tag, sizeof(req.tag));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_AES_GCM_DECRYPT_FINAL,
				       &req.context, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, ciphertext, ciphertext_len);

	/*
	 * resp is static -- see the comment on cptra_mci_random_generate()
	 * above. Doubly important here: resp.hdr.tag_verified is a trusted
	 * pass/fail flag, so a stale copy from a previous call must never be
	 * read while this call's transaction is still in flight.
	 */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_AES_GCM_DECRYPT_FINAL, &req, sizeof(req),
				       ciphertext, ciphertext_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_DECRYPT_FINAL failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_DECRYPT_FINAL response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	if (!resp.hdr.tag_verified) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_DECRYPT_FINAL: tag verification failed");
		if (plaintext_len)
			*plaintext_len = 0;
		return -EBADMSG;
	}

	len = MIN(resp.hdr.output_size, sizeof(resp.output));
	len = MIN(len, plaintext_buf_len);

	if (resp_len < sizeof(resp.hdr) + len) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_AES_GCM_DECRYPT_FINAL response shorter than reported output_size");
		return -EIO;
	}

	memcpy(plaintext, resp.output, len);

	cptra_mci_mbox_txn_end();

	if (plaintext_len)
		*plaintext_len = len;

	return 0;
}

int cptra_mci_mldsa_cmk_public_key(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
				   uint8_t public_key[CPTRA_MCI_MLDSA87_PUBKEY_SIZE])
{
	struct cptra_mci_mldsa_pubkey_req req = { 0 };
	/* public_key alone is 2592 bytes -- kept off the stack, as with get_log/export_csr. */
	static struct cptra_mci_mldsa_pubkey_resp resp;
	uint32_t resp_len;
	int ret;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_MLDSA_CMK_PUBLIC_KEY,
						 &req.cmk, sizeof(req) - sizeof(req.hdr));

	/* resp is static -- see the comment on cptra_mci_random_generate() above. */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_MLDSA_CMK_PUBLIC_KEY, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_MLDSA_CMK_PUBLIC_KEY failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_MLDSA_CMK_PUBLIC_KEY response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(public_key, resp.public_key, sizeof(resp.public_key));

	cptra_mci_mbox_txn_end();

	return 0;
}

int cptra_mci_mldsa_cmk_sign(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
			    const uint8_t *message, size_t message_len,
			    uint8_t signature[CPTRA_MCI_MLDSA87_SIGNATURE_SIZE])
{
	struct cptra_mci_mldsa_sign_hdr req = {
		.message_size = message_len,
	};
	/* signature alone is 4628 bytes -- kept off the stack, as above. */
	static struct cptra_mci_mldsa_sign_resp resp;
	uint32_t resp_len, csum;
	int ret;

	if (message_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_MLDSA_CMK_SIGN,
				       &req.cmk, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, message, message_len);

	/* resp is static -- see the comment on cptra_mci_random_generate() above. */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_MLDSA_CMK_SIGN, &req, sizeof(req),
				       message, message_len, &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_MLDSA_CMK_SIGN failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_MLDSA_CMK_SIGN response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	memcpy(signature, resp.signature, sizeof(resp.signature));

	cptra_mci_mbox_txn_end();

	return 0;
}

int cptra_mci_mldsa_cmk_verify(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
			      const uint8_t signature[CPTRA_MCI_MLDSA87_SIGNATURE_SIZE],
			      const uint8_t *message, size_t message_len)
{
	/*
	 * The signature field alone makes this request header 4764 bytes --
	 * kept off the stack, as with the responses above. Every field is
	 * written below before use, so no explicit zero-init is needed.
	 * req is static (shared across calls), so the lock has to cover
	 * everything from the first write below through the execute() call --
	 * see the comment on cptra_mci_random_generate() above for why.
	 */
	static struct cptra_mci_mldsa_verify_hdr req;
	struct cptra_mci_mldsa_verify_resp resp = { 0 };
	uint32_t resp_len, csum;
	int ret;

	if (message_len > CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
		return -EINVAL;

	cptra_mci_mbox_txn_begin();

	req.message_size = message_len;
	memcpy(req.cmk.value, cmk, sizeof(req.cmk.value));
	memcpy(req.signature, signature, sizeof(req.signature));

	csum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_MLDSA_CMK_VERIFY,
				       &req.cmk, sizeof(req) - sizeof(req.hdr));
	req.hdr.chksum = cptra_mci_mbox_checksum_ext(csum, message, message_len);

	ret = cptra_mci_mbox_execute_sg(CPTRA_MCI_MBCMD_MLDSA_CMK_VERIFY, &req, sizeof(req),
				       message, message_len, &resp, sizeof(resp), &resp_len);

	cptra_mci_mbox_txn_end();

	if (ret) {
		LOG_ERR("MC_MLDSA_CMK_VERIFY failed: %d", ret);
		return ret;
	}

	return 0;
}
