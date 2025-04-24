/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_DRIVERS_MISC_ASPEED_CPTRA_IPC_H_
#define ZEPHYR_DRIVERS_MISC_ASPEED_CPTRA_IPC_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

enum cptra_ipc_cmd {
	CPTRA_IPCCMD_ECDSA384_SIGNATURE_VERIFY = 0,
	CPTRA_IPCCMD_SHA384,
	CPTRA_IPCCMD_LMS_SIGNATURE_VERIFY,
	CPTRA_IPCCMD_CALIPTRA_FW_LOAD,
	CPTRA_IPCCMD_STASH_MEASUREMENT,
	CPTRA_IPCCMD_QUOTE_PCRS,
	CPTRA_IPCCMD_GET_IDEV_CERT,
	CPTRA_IPCCMD_GET_IDEV_INFO,
	CPTRA_IPCCMD_POPULATE_IDEV_CERT,
	CPTRA_IPCCMD_GET_LDEV_CERT,
	CPTRA_IPCCMD_GET_FMC_ALIAS_CERT,
	CPTRA_IPCCMD_GET_RT_ALIAS_CERT,
	CPTRA_IPCCMD_INVOKE_DPE_COMMAND,
	CPTRA_IPCCMD_DISABLE_ATTESTATION,
	CPTRA_IPCCMD_FW_INFO,
	CPTRA_IPCCMD_DPE_TAG_TCI,
	CPTRA_IPCCMD_DPE_GET_TAGGED_TCI,
	CPTRA_IPCCMD_INCREMENT_PCR_RESET_COUNTER,
	CPTRA_IPCCMD_EXTEND_PCR,
	CPTRA_IPCCMD_ADD_SUBJECT_ALT_NAME,
	CPTRA_IPCCMD_CERTIFY_KEY_EXTENDED,
	CPTRA_IPCCMD_FIPS_VERSION,
	CPTRA_IPCCMD_SHUTDOWN,
	CPTRA_IPCCMD_CAPABILITIES,
	CPTRA_IPCCMD_SET_AUTH_MANIFEST,
};

enum cptra_ipc_rx_type {
	CPTRA_IPC_RX_TYPE_INTERNAL = 0,
	CPTRA_IPC_RX_TYPE_EXTERNAL = 1,
};

int cptra_ipc_enable(void);
int cptra_ipc_trigger(enum cptra_ipc_cmd cmd, void *input, int input_size);
int cptra_ipc_receive(enum cptra_ipc_rx_type type, void *output, int output_size);
int cptra_ipc_transfer(enum cptra_ipc_cmd cmd, void *input, int input_size,
		       enum cptra_ipc_rx_type type, void *output, int output_size);

struct cptra_ecdsa_ctx {
	int qx_len;
	uint8_t *qx;
	int qy_len;
	uint8_t *qy;
	int r_len;
	uint8_t *r;
	int s_len;
	uint8_t *s;
	int  m_len;
	uint8_t *m;
};

struct cptra_hash_ctx {
	uint32_t algo;
	int in_len;
	uint8_t *in_buf;
	int out_len;
	uint8_t *out_buf;
};

struct cptra_lms_ctx {
	/* public key */
	uint32_t pub_key_tree_type;
	uint32_t pub_key_ots_type;
	int pub_key_id_len;
        uint8_t *pub_key_id;
        int pub_key_digest_len;
	uint8_t *pub_key_digest;

	/* signature */
	uint32_t sig_q;
        int sig_ots_len;
	uint8_t *sig_ots;
	uint32_t sig_tree_type;
        int sig_tree_path_len;
	uint8_t *sig_tree_path;
};

int cptra_ipc_enable(void);

#endif /* ZEPHYR_DRIVERS_MISC_ASPEED_CPTRA_IPC_H_ */
