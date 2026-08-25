/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>

LOG_MODULE_REGISTER(cptra_mci_misc, CONFIG_LOG_DEFAULT_LEVEL);

int cptra_mci_get_firmware_version(enum cptra_mci_fw_index index, char *version, size_t len)
{
	struct cptra_mci_fw_version_req req = {
		.index = index,
	};
	struct cptra_mci_fw_version_resp resp = { 0 };
	uint32_t resp_len;
	uint32_t vlen;
	int ret;

	if (len == 0)
		return -EINVAL;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_FIRMWARE_VERSION,
						 &req.index, sizeof(req.index));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_FIRMWARE_VERSION, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_FIRMWARE_VERSION failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		LOG_ERR("MC_FIRMWARE_VERSION response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	vlen = MIN(resp.hdr.data_len, sizeof(resp.version));
	vlen = MIN(vlen, len - 1);

	if (resp_len < sizeof(resp.hdr) + vlen) {
		LOG_ERR("MC_FIRMWARE_VERSION response shorter than reported data_len");
		return -EIO;
	}

	memset(version, 0, len);
	memcpy(version, resp.version, vlen);

	return 0;
}

int cptra_mci_get_device_capabilities(uint8_t *caps, size_t len)
{
	struct cptra_mci_device_caps_req req = { 0 };
	struct cptra_mci_device_caps_resp resp = { 0 };
	uint32_t resp_len;
	uint32_t clen;
	int ret;

	if (len == 0)
		return -EINVAL;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_DEVICE_CAPABILITIES, NULL, 0);

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_DEVICE_CAPABILITIES, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_DEVICE_CAPABILITIES failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_DEVICE_CAPABILITIES response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	clen = MIN(sizeof(resp.caps), len);
	memcpy(caps, resp.caps, clen);

	return 0;
}

int cptra_mci_get_log(uint8_t *log, size_t len, size_t *log_len)
{
	/* data[] alone is CPTRA_MCI_MAX_RESP_DATA_SIZE (4096) bytes -- kept off the stack. */
	static struct cptra_mci_get_log_resp resp;
	struct cptra_mci_get_log_req req = { 0 };
	uint32_t resp_len;
	uint32_t dlen;
	int ret;

	if (len == 0)
		return -EINVAL;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_GET_LOG, NULL, 0);

	/*
	 * resp is static (shared across calls), so the lock has to cover
	 * everything from here through the last read of resp below -- not
	 * just the hardware step inside execute() -- or two concurrent
	 * callers can hand each other's log data to one another.
	 */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_GET_LOG, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_GET_LOG failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_GET_LOG response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	dlen = MIN(resp.hdr.data_len, sizeof(resp.data));
	dlen = MIN(dlen, len);

	if (resp_len < sizeof(resp.hdr) + dlen) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_GET_LOG response shorter than reported data_len");
		return -EIO;
	}

	memcpy(log, resp.data, dlen);

	cptra_mci_mbox_txn_end();

	if (log_len)
		*log_len = dlen;

	return 0;
}

int cptra_mci_clear_log(void)
{
	struct cptra_mci_clear_log_req req = { 0 };
	struct cptra_mci_clear_log_resp resp = { 0 };
	uint32_t resp_len;
	int ret;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_CLEAR_LOG, NULL, 0);

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_CLEAR_LOG, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_CLEAR_LOG failed: %d", ret);
		return ret;
	}

	return 0;
}

int cptra_mci_get_auth_cmd_challenge(uint32_t flags, uint8_t *challenge, size_t len)
{
	struct cptra_mci_auth_cmd_challenge_req req = {
		.flags = flags,
	};
	struct cptra_mci_auth_cmd_challenge_resp resp = { 0 };
	uint32_t resp_len;
	uint32_t clen;
	int ret;

	if (len == 0)
		return -EINVAL;

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_GET_AUTH_CMD_CHALLENGE,
						 &req.flags, sizeof(req) - sizeof(req.hdr));

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_GET_AUTH_CMD_CHALLENGE, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		LOG_ERR("MC_GET_AUTH_CMD_CHALLENGE failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp)) {
		LOG_ERR("MC_GET_AUTH_CMD_CHALLENGE response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp));
		return -EIO;
	}

	clen = MIN(sizeof(resp.challenge), len);
	memcpy(challenge, resp.challenge, clen);

	return 0;
}

int cptra_mci_export_attested_csr(enum cptra_mci_device_key_id device_key_id,
				  enum cptra_mci_csr_algo algorithm,
				  const uint8_t nonce[CPTRA_MCI_CSR_NONCE_SIZE],
				  uint8_t *csr, size_t len, size_t *csr_len)
{
	/* data[] alone is CPTRA_MCI_MAX_RESP_DATA_SIZE (4096) bytes -- kept off the stack. */
	static struct cptra_mci_export_csr_resp resp;
	struct cptra_mci_export_csr_req req = {
		.device_key_id = device_key_id,
		.algorithm = algorithm,
	};
	uint32_t resp_len;
	uint32_t clen;
	int ret;

	if (len == 0)
		return -EINVAL;

	memcpy(req.nonce, nonce, sizeof(req.nonce));

	req.hdr.chksum = cptra_mci_mbox_checksum(CPTRA_MCI_MBCMD_EXPORT_ATTESTED_CSR,
						 &req.device_key_id, sizeof(req) - sizeof(req.hdr));

	/*
	 * resp is static (shared across calls), so the lock has to cover
	 * everything from here through the last read of resp below -- not
	 * just the hardware step inside execute() -- or two concurrent
	 * callers can hand each other's CSR data to one another.
	 */
	cptra_mci_mbox_txn_begin();

	ret = cptra_mci_mbox_execute(CPTRA_MCI_MBCMD_EXPORT_ATTESTED_CSR, &req, sizeof(req),
				     &resp, sizeof(resp), &resp_len);
	if (ret) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_EXPORT_ATTESTED_CSR failed: %d", ret);
		return ret;
	}

	if (resp_len < sizeof(resp.hdr)) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_EXPORT_ATTESTED_CSR response too short (%u < %u)", resp_len,
			(uint32_t)sizeof(resp.hdr));
		return -EIO;
	}

	clen = MIN(resp.hdr.data_len, sizeof(resp.data));
	clen = MIN(clen, len);

	if (resp_len < sizeof(resp.hdr) + clen) {
		cptra_mci_mbox_txn_end();
		LOG_ERR("MC_EXPORT_ATTESTED_CSR response shorter than reported data_len");
		return -EIO;
	}

	memcpy(csr, resp.data, clen);

	cptra_mci_mbox_txn_end();

	if (csr_len)
		*csr_len = clen;

	return 0;
}
