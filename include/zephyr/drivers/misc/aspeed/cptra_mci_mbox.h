/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_CPTRA_MCI_MBOX_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_CPTRA_MCI_MBOX_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

/*
 * The Caliptra-SS MCI (Manageability Controller Interface) mailbox is not
 * mapped directly. It is only reachable through a fixed 64KB local window;
 * writing a 64KB-aligned target address into this SCU1 register remaps the
 * window onto that target for as long as the page stays selected.
 */
#define SCU1_CPTRA_MCI_WIN			0x120

#define CPTRA_MCI_MBOX_WIN_SIZE			0x10000

/* Actual backing size of the mailbox data SRAM (the rest of the 64KB window is unused) */
#define CPTRA_MCI_MBOX_SRAM_SIZE		0x4000

/* MAX_CMB_DATA_SIZE: max variable-length payload per single mailbox call */
#define CPTRA_MCI_MBOX_MAX_INPUT_SIZE		4096

/* Max time to wait for the mailbox LOCK to become available */
#define CPTRA_MCI_MBOX_LOCK_TIMEOUT_MS		1000

/*
 * Page targets for SCU1_CPTRA_MCI_WIN. Different bus masters reaching this
 * same window encode the page target differently -- e.g. some write it as
 * a shifted address>>16 (0x2100), while this MCU-side bus master writes
 * the full, unshifted target address. Do not reuse a page value derived
 * for one bus master's encoding on another.
 */
#define CPTRA_MCI_MBOX_SRAM_PAGE		0x21400000	/* mailbox data */
#define CPTRA_MCI_MBOX_CSR_PAGE			0x21600000	/* mcu_mbox0_csr */
#define CPTRA_MCI_REG_PAGE			0x21000000	/* mci_reg */
#define CPTRA_MCI_SOC_IFC_PAGE			0xa0030000	/* soc_ifc_reg */

/* mcu_mbox0_csr register offsets, valid once CPTRA_MCI_MBOX_CSR_PAGE is selected */
#define CPTRA_MCI_MBOX_LOCK			0x00
#define CPTRA_MCI_MBOX_USER			0x04
#define CPTRA_MCI_MBOX_TARGET_USER		0x08
#define CPTRA_MCI_MBOX_TARGET_USER_VALID	0x0c
#define CPTRA_MCI_MBOX_CMD			0x10
#define CPTRA_MCI_MBOX_DLEN			0x14
#define CPTRA_MCI_MBOX_EXECUTE			0x18
#define CPTRA_MCI_MBOX_TARGET_STATUS		0x1c
#define CPTRA_MCI_MBOX_CMD_STATUS		0x20
#define   CPTRA_MCI_MBOX_CMD_STATUS_PS		GENMASK(3, 0)
#define CPTRA_MCI_MBOX_HW_STATUS		0x24

/*
 * mci_reg register offsets, valid once CPTRA_MCI_REG_PAGE is selected --
 * plain memory-mapped status registers, not part of the mailbox
 * command/lock/execute protocol. Offsets confirmed on real hardware.
 */
#define CPTRA_MCI_REG_MCU_IFU_AXI_USER			0x0020
#define CPTRA_MCI_REG_MCU_LSU_AXI_USER			0x0024
#define CPTRA_MCI_REG_MCU_SRAM_CONFIG_AXI_USER		0x0028
#define CPTRA_MCI_REG_MCI_SOC_CONFIG_AXI_USER		0x002c
#define CPTRA_MCI_REG_RESET_REASON			0x0038
#define   CPTRA_MCI_REG_RESET_REASON_FW_HITLESS_UPD_RESET	BIT(0)
#define   CPTRA_MCI_REG_RESET_REASON_FW_BOOT_UPD_RESET		BIT(1)
#define   CPTRA_MCI_REG_RESET_REASON_WARM_RESET		BIT(2)
#define CPTRA_MCI_REG_SECURITY_STATE			0x0040
#define   CPTRA_MCI_REG_SECURITY_STATE_DEVICE_LIFECYCLE	GENMASK(1, 0)
#define   CPTRA_MCI_REG_SECURITY_STATE_DEBUG_LOCKED		BIT(2)
#define   CPTRA_MCI_REG_SECURITY_STATE_SCAN_MODE		BIT(3)

/* device_lifecycle_e */
enum cptra_mci_device_lifecycle {
	CPTRA_MCI_DEVICE_UNPROVISIONED = 0,
	CPTRA_MCI_DEVICE_MANUFACTURING = 1,
	CPTRA_MCI_DEVICE_PRODUCTION = 3,
};

/* Each MBOXn_*_AXI_USER block is CPTRA_MCI_REG_MBOX_AXI_USER_COUNT 32-bit regs */
#define CPTRA_MCI_REG_MBOX_AXI_USER_COUNT		5
#define CPTRA_MCI_REG_MBOX0_VALID_AXI_USER(n)		(0x0180 + 4 * (n))
#define CPTRA_MCI_REG_MBOX0_AXI_USER_LOCK(n)		(0x01a0 + 4 * (n))
#define CPTRA_MCI_REG_MBOX1_VALID_AXI_USER(n)		(0x01c0 + 4 * (n))
#define CPTRA_MCI_REG_MBOX1_AXI_USER_LOCK(n)		(0x01e0 + 4 * (n))

#define CPTRA_MCI_REG_SS_DEBUG_INTENT			0x0418
#define CPTRA_MCI_REG_SS_CONFIG_DONE_STICKY		0x0440
#define CPTRA_MCI_REG_SS_CONFIG_DONE			0x0444

/*
 * soc_ifc_reg register offsets, valid once CPTRA_MCI_SOC_IFC_PAGE is
 * selected. Bit fields for CPTRA_RESET_REASON/CPTRA_SECURITY_STATE
 * cross-checked against
 * caliptra-mcu-sw/registers/generated-firmware/src/soc.rs
 * (CptraResetReason/CptraSecurityState) -- note CPTRA_RESET_REASON only
 * has 2 bits (no hitless/boot split), unlike mci_reg's own 3-bit
 * RESET_REASON.
 */
#define CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON		0x0040
#define   CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON_FW_UPD_RESET	BIT(0)
#define   CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON_WARM_RESET	BIT(1)
#define CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE		0x0044
#define   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE_DEVICE_LIFECYCLE	GENMASK(1, 0)
#define   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE_DEBUG_LOCKED		BIT(2)
#define   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE_SCAN_MODE		BIT(3)
#define CPTRA_MCI_SOC_IFC_MBOX_AXI_USER_COUNT		5
#define CPTRA_MCI_SOC_IFC_CPTRA_MBOX_VALID_AXI_USER(n)	(0x0048 + 4 * (n))
#define CPTRA_MCI_SOC_IFC_CPTRA_MBOX_AXI_USER_LOCK(n)	(0x005c + 4 * (n))
#define CPTRA_MCI_SOC_IFC_CPTRA_TRNG_VALID_AXI_USER	0x0070
#define CPTRA_MCI_SOC_IFC_CPTRA_TRNG_AXI_USER_LOCK	0x0074
#define CPTRA_MCI_SOC_IFC_CPTRA_FUSE_VALID_AXI_USER	0x0108

union cptra_mci_mbox_lock_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t lock : 1;		/*[0-0]*/
		volatile uint32_t reserved : 31;	/*[1-31]*/
	} fields;
}; /* 0x00 */

union cptra_mci_mbox_cmd_status_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t status : 4;		/*[0-3]*/
		volatile uint32_t reserved : 28;	/*[4-31]*/
	} fields;
}; /* 0x20 */

/* mbox_status_e (mcu_mbox0_csr.mbox_cmd_status) */
enum cptra_mci_mbox_sts {
	CPTRA_MCI_MBSTS_CMD_BUSY = 0,
	CPTRA_MCI_MBSTS_DATA_READY,
	CPTRA_MCI_MBSTS_CMD_COMPLETE,
	CPTRA_MCI_MBSTS_CMD_FAILURE,
};

/* CommandId ("common/mcu-mbox/src/messages.rs", caliptra-mcu-sw) */
enum cptra_mci_mbox_cmd {
	CPTRA_MCI_MBCMD_FIRMWARE_VERSION                = 0x4D465756, /* "MFWV" */
	CPTRA_MCI_MBCMD_DEVICE_CAPABILITIES             = 0x4D434150, /* "MCAP" */
	CPTRA_MCI_MBCMD_GET_LOG                         = 0x4D474C47, /* "MGLG" */
	CPTRA_MCI_MBCMD_CLEAR_LOG                       = 0x4D434C47, /* "MCLG" */
	CPTRA_MCI_MBCMD_FIPS_SELF_TEST_START            = 0x4D465354, /* "MFST" */
	CPTRA_MCI_MBCMD_FIPS_SELF_TEST_GET_RESULTS      = 0x4D464752, /* "MFGR" */
	CPTRA_MCI_MBCMD_FIPS_PERIODIC_ENABLE            = 0x4D465045, /* "MFPE" */
	CPTRA_MCI_MBCMD_FIPS_PERIODIC_STATUS            = 0x4D465053, /* "MFPS" */
	CPTRA_MCI_MBCMD_SHA_INIT                        = 0x4D435349, /* "MCSI" */
	CPTRA_MCI_MBCMD_SHA_UPDATE                      = 0x4D435355, /* "MCSU" */
	CPTRA_MCI_MBCMD_SHA_FINAL                       = 0x4D435346, /* "MCSF" */
	CPTRA_MCI_MBCMD_HMAC                            = 0x4D43484D, /* "MCHM" */
	CPTRA_MCI_MBCMD_HMAC_KDF_COUNTER                = 0x4D434B43, /* "MCKC" */
	CPTRA_MCI_MBCMD_HKDF_EXTRACT                    = 0x4D434B54, /* "MCKT" */
	CPTRA_MCI_MBCMD_HKDF_EXPAND                     = 0x4D434B50, /* "MCKP" */
	CPTRA_MCI_MBCMD_AES_ENCRYPT_INIT                = 0x4D434349, /* "MCCI" */
	CPTRA_MCI_MBCMD_AES_ENCRYPT_UPDATE              = 0x4D434355, /* "MCCU" */
	CPTRA_MCI_MBCMD_AES_DECRYPT_INIT                = 0x4D43414A, /* "MCAJ" */
	CPTRA_MCI_MBCMD_AES_DECRYPT_UPDATE              = 0x4D434155, /* "MCAU" */
	CPTRA_MCI_MBCMD_AES_GCM_ENCRYPT_INIT            = 0x4D434749, /* "MCGI" */
	CPTRA_MCI_MBCMD_AES_GCM_ENCRYPT_UPDATE          = 0x4D434755, /* "MCGU" */
	CPTRA_MCI_MBCMD_AES_GCM_ENCRYPT_FINAL           = 0x4D434746, /* "MCGF" */
	CPTRA_MCI_MBCMD_AES_GCM_DECRYPT_INIT            = 0x4D434449, /* "MCDI" */
	CPTRA_MCI_MBCMD_AES_GCM_DECRYPT_UPDATE          = 0x4D434455, /* "MCDU" */
	CPTRA_MCI_MBCMD_AES_GCM_DECRYPT_FINAL           = 0x4D434446, /* "MCDF" */
	CPTRA_MCI_MBCMD_RANDOM_STIR                     = 0x4D435253, /* "MCRS" */
	CPTRA_MCI_MBCMD_RANDOM_GENERATE                 = 0x4D435247, /* "MCRG" */
	CPTRA_MCI_MBCMD_IMPORT                          = 0x4D43494D, /* "MCIM" */
	CPTRA_MCI_MBCMD_DELETE                          = 0x4D43444C, /* "MCDL" */
	CPTRA_MCI_MBCMD_CM_STATUS                       = 0x4D435354, /* "MCST" */
	CPTRA_MCI_MBCMD_ECDH_GENERATE                   = 0x4D434547, /* "MCEG" */
	CPTRA_MCI_MBCMD_ECDH_FINISH                     = 0x4D434546, /* "MCEF" */
	CPTRA_MCI_MBCMD_ECDSA_CMK_PUBLIC_KEY             = 0x4D434550, /* "MCEP" */
	CPTRA_MCI_MBCMD_ECDSA_CMK_SIGN                   = 0x4D434553, /* "MCES" */
	CPTRA_MCI_MBCMD_ECDSA_CMK_VERIFY                 = 0x4D434556, /* "MCEV" */
	CPTRA_MCI_MBCMD_MLDSA_CMK_PUBLIC_KEY             = 0x4D4D4C50, /* "MMLP" */
	CPTRA_MCI_MBCMD_MLDSA_CMK_SIGN                   = 0x4D4D4C53, /* "MMLS" */
	CPTRA_MCI_MBCMD_MLDSA_CMK_VERIFY                 = 0x4D4D4C56, /* "MMLV" */
	CPTRA_MCI_MBCMD_ECDSA384_SIG_VERIFY               = 0x4D454356, /* "MECV" */
	CPTRA_MCI_MBCMD_LMS_SIG_VERIFY                   = 0x4D4C4D56, /* "MLMV" */
	CPTRA_MCI_MBCMD_MLDSA87_SIG_VERIFY               = 0x4D4D5356, /* "MMSV" */
	CPTRA_MCI_MBCMD_PROD_DEBUG_UNLOCK_REQ            = 0x4D505552, /* "MPUR" */
	CPTRA_MCI_MBCMD_PROD_DEBUG_UNLOCK_TOKEN          = 0x4D505554, /* "MPUT" */
	CPTRA_MCI_MBCMD_FUSE_READ                        = 0x49465052, /* "IFPR" */
	CPTRA_MCI_MBCMD_FUSE_WRITE                       = 0x49465057, /* "IFPW" */
	CPTRA_MCI_MBCMD_FUSE_LOCK_PARTITION               = 0x4946504B, /* "IFPK" */
	CPTRA_MCI_MBCMD_GET_AUTH_CMD_CHALLENGE            = 0x4D414343, /* "MACC" */
	CPTRA_MCI_MBCMD_PROVISION_VENDOR_PK_HASH          = 0x5056504B, /* "PVPK" */
	CPTRA_MCI_MBCMD_FUSE_INCREASE_CALIPTRA_MIN_SVN    = 0x4D434D53, /* "MCMS" */
	CPTRA_MCI_MBCMD_FE_PROG                           = 0x4D434650, /* "MCFP" */
	CPTRA_MCI_MBCMD_FUSE_REVOKE_VENDOR_PUB_KEY        = 0x4D52564B, /* "MRVK" */
	CPTRA_MCI_MBCMD_FUSE_REVOKE_VENDOR_PK_HASH        = 0x52564B48, /* "RVKH" */
	CPTRA_MCI_MBCMD_EXPORT_ATTESTED_CSR               = 0x4D454143, /* "MEAC" */
	CPTRA_MCI_MBCMD_OCP_LOCK_ROTATE_HEK                = 0x4F4C5248, /* "OLRH" */
	CPTRA_MCI_MBCMD_OCP_LOCK_SET_PERMA_HEK             = 0x4F4C5350, /* "OLSP" */
	CPTRA_MCI_MBCMD_GET_OCP_LOCK_ENDORSEMENT_CERT      = 0x4F4C4543, /* "OLEC" */
	CPTRA_MCI_MBCMD_OCP_LOCK_ENUMERATE_HPKE_HANDLES    = 0x4F4C4548, /* "OLEH" */
};

/* Common request/response headers (messages.rs MailboxReqHeader/MailboxRespHeaderVarSize) */
struct cptra_mci_mbox_req_hdr {
	uint32_t chksum;
};

struct cptra_mci_mbox_resp_hdr {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_mci_mbox_resp_hdr_var {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint32_t data_len;
};

/* FwIndex (messages.rs) */
enum cptra_mci_fw_index {
	CPTRA_MCI_FW_INDEX_CALIPTRA_CORE = 0,
	CPTRA_MCI_FW_INDEX_MCU_RUNTIME,
	CPTRA_MCI_FW_INDEX_SOC,
};

#define CPTRA_MCI_MAX_FW_VERSION_STR_LEN	32

struct cptra_mci_fw_version_req {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t index;
};

struct cptra_mci_fw_version_resp {
	struct cptra_mci_mbox_resp_hdr_var hdr;
	uint8_t version[CPTRA_MCI_MAX_FW_VERSION_STR_LEN];
};

/* MC_DEVICE_CAPABILITIES (DeviceCapsReq/DeviceCapsResp) */
#define CPTRA_MCI_DEVICE_CAPS_SIZE		32

struct cptra_mci_device_caps_req {
	struct cptra_mci_mbox_req_hdr hdr;
};

struct cptra_mci_device_caps_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t caps[CPTRA_MCI_DEVICE_CAPS_SIZE];
};

/*
 * MC_GET_LOG / MC_CLEAR_LOG (GetLogReq/GetLogResp, ClearLogReq/ClearLogResp).
 * Neither request currently carries a log-type selector on the wire, despite
 * messages.rs defining a (so far unused) LogType { DebugLog, AttestationLog }.
 */
#define CPTRA_MCI_MAX_RESP_DATA_SIZE		4096	/* MAX_RESP_DATA_SIZE = 4 * 1024 */

struct cptra_mci_get_log_req {
	struct cptra_mci_mbox_req_hdr hdr;
};

struct cptra_mci_get_log_resp {
	struct cptra_mci_mbox_resp_hdr_var hdr;
	uint8_t data[CPTRA_MCI_MAX_RESP_DATA_SIZE];
};

struct cptra_mci_clear_log_req {
	struct cptra_mci_mbox_req_hdr hdr;
};

struct cptra_mci_clear_log_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
};

/* MC_GET_AUTH_CMD_CHALLENGE (GetAuthCmdChallengeReq/GetAuthCmdChallengeResp) */
#define CPTRA_MCI_AUTH_CHALLENGE_SIZE		32

struct cptra_mci_auth_cmd_challenge_req {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t flags;
	uint32_t reserved;
};

struct cptra_mci_auth_cmd_challenge_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint32_t reserved;
	uint8_t challenge[CPTRA_MCI_AUTH_CHALLENGE_SIZE];
};

/* MC_EXPORT_ATTESTED_CSR (ExportAttestedCsrReq/ExportAttestedCsrResp) */
enum cptra_mci_device_key_id {
	CPTRA_MCI_DEVICE_KEY_LDEVID = 1,
	CPTRA_MCI_DEVICE_KEY_FMC_ALIAS = 2,
	CPTRA_MCI_DEVICE_KEY_RT_ALIAS = 3,
};

enum cptra_mci_csr_algo {
	CPTRA_MCI_CSR_ALGO_ECC384 = 1,
	CPTRA_MCI_CSR_ALGO_MLDSA87 = 2,
};

#define CPTRA_MCI_CSR_NONCE_SIZE		32

struct cptra_mci_export_csr_req {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t device_key_id;
	uint32_t algorithm;
	uint8_t nonce[CPTRA_MCI_CSR_NONCE_SIZE];
};

struct cptra_mci_export_csr_resp {
	struct cptra_mci_mbox_resp_hdr_var hdr;
	uint8_t data[CPTRA_MCI_MAX_RESP_DATA_SIZE];
};

/*
 * MC_RANDOM_STIR (CmRandomStirReq, response is a bare MailboxRespHeader with
 * no payload). Only the fixed-size header is modeled; the (up to
 * CPTRA_MCI_MBOX_MAX_INPUT_SIZE byte) input payload is supplied separately
 * to cptra_mci_mbox_execute_sg().
 */
struct cptra_mci_random_stir_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t input_size;
};

struct cptra_mci_random_stir_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
};

/*
 * MC_RANDOM_GENERATE (CmRandomGenerateReq/Resp). Unlike MC_RANDOM_STIR, the
 * request here is fully fixed-size (just the requested byte count) and goes
 * through cptra_mci_mbox_execute(), not the _sg() variant; it is the
 * response that is variable-size, with data_len telling how many of the up
 * to CPTRA_MCI_MBOX_MAX_INPUT_SIZE returned bytes are valid.
 */
struct cptra_mci_random_generate_req {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t size;
};

struct cptra_mci_random_generate_resp {
	struct cptra_mci_mbox_resp_hdr_var hdr;
	uint8_t data[CPTRA_MCI_MBOX_MAX_INPUT_SIZE];
};

/* CmHashAlgorithm (messages.rs / mailbox.rs) */
enum cptra_mci_sha_algo {
	CPTRA_MCI_SHA_ALGO_RESERVED = 0,
	CPTRA_MCI_SHA_ALGO_SHA384 = 1,
	CPTRA_MCI_SHA_ALGO_SHA512 = 2,
};

#define CPTRA_MCI_SHA_CONTEXT_SIZE		200	/* CMB_SHA_CONTEXT_SIZE */
#define CPTRA_MCI_SHA384_DIGEST_SIZE		48
#define CPTRA_MCI_SHA512_DIGEST_SIZE		64

/*
 * MC_SHA_INIT / MC_SHA_UPDATE / MC_SHA_FINAL (CmShaInitReq/CmShaUpdateReq/CmShaFinalReq).
 * Only the fixed-size header is modeled here; the (up to CPTRA_MCI_MBOX_MAX_INPUT_SIZE
 * byte) input payload is supplied separately to cptra_mci_mbox_execute_sg().
 */
struct cptra_mci_sha_init_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t hash_algorithm;
	uint32_t input_size;
};

/* MC_SHA_UPDATE and MC_SHA_FINAL requests share this header shape */
struct cptra_mci_sha_data_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint8_t context[CPTRA_MCI_SHA_CONTEXT_SIZE];
	uint32_t input_size;
};

/* MC_SHA_INIT and MC_SHA_UPDATE responses share this shape */
struct cptra_mci_sha_ctx_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t context[CPTRA_MCI_SHA_CONTEXT_SIZE];
};

struct cptra_mci_sha_final_resp {
	struct cptra_mci_mbox_resp_hdr_var hdr;
	uint8_t hash[CPTRA_MCI_SHA512_DIGEST_SIZE];
};

/*
 * MC_CM_STATUS (bare MailboxReqHeader request / CmStatusResp). Reports how
 * much of Caliptra-SS's Cmk key storage is in use -- a companion query to
 * the whole family of Cmk-producing commands below (MC_IMPORT, MC_HMAC_KDF_*,
 * MC_HKDF_*, MC_ECDSA_CMK_*, MC_MLDSA_CMK_*).
 */
struct cptra_mci_cm_status_req {
	struct cptra_mci_mbox_req_hdr hdr;
};

struct cptra_mci_cm_status_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint32_t used_usage_storage;
	uint32_t total_usage_storage;
};

#define CPTRA_MCI_CMK_SIZE			128	/* CMK_SIZE_BYTES */
#define CPTRA_MCI_ECC384_SCALAR_SIZE		48	/* ECC384_SCALAR_BYTE_SIZE */

/* Cmk: an opaque, encrypted key blob. The raw key material never leaves Caliptra-SS. */
struct cptra_mci_cmk {
	uint8_t value[CPTRA_MCI_CMK_SIZE];
};

/* CmKeyUsage (mailbox.rs) */
enum cptra_mci_key_usage {
	CPTRA_MCI_KEY_USAGE_RESERVED = 0,
	CPTRA_MCI_KEY_USAGE_HMAC = 1,
	CPTRA_MCI_KEY_USAGE_AES = 2,
	CPTRA_MCI_KEY_USAGE_ECDSA = 3,
	CPTRA_MCI_KEY_USAGE_MLDSA = 4,
	CPTRA_MCI_KEY_USAGE_MLKEM = 5,
};

#define CPTRA_MCI_IMPORT_MAX_KEY_SIZE		64	/* CMK_MAX_KEY_SIZE_BITS / 8 */

/*
 * MC_IMPORT (CmImportReq/Resp). Imports raw key material under key_usage and
 * returns an opaque Cmk handle for it; the raw material never round-trips
 * back out. Only the fixed-size header is modeled; the input key payload is
 * supplied separately to cptra_mci_mbox_execute_sg().
 */
struct cptra_mci_import_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t key_usage;
	uint32_t input_size;
};

struct cptra_mci_import_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	struct cptra_mci_cmk cmk;
};

#define CPTRA_MCI_ECDH_CONTEXT_SIZE		76	/* CMB_ECDH_ENCRYPTED_CONTEXT_SIZE */
#define CPTRA_MCI_ECDH_EXCHANGE_DATA_SIZE	96	/* CMB_ECDH_EXCHANGE_DATA_MAX_SIZE */

/*
 * MC_ECDH_GENERATE (bare MailboxReqHeader request / CmEcdhGenerateResp).
 * Generates an ephemeral ECDH key pair: context is opaque/encrypted state to
 * hand back to MC_ECDH_FINISH, exchange_data is the local public key
 * material to send to the peer over some external channel.
 */
struct cptra_mci_ecdh_generate_req {
	struct cptra_mci_mbox_req_hdr hdr;
};

struct cptra_mci_ecdh_generate_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t context[CPTRA_MCI_ECDH_CONTEXT_SIZE];
	uint8_t exchange_data[CPTRA_MCI_ECDH_EXCHANGE_DATA_SIZE];
};

/*
 * MC_ECDH_FINISH (CmEcdhFinishReq/Resp). Takes the context from
 * MC_ECDH_GENERATE plus the peer's exchange_data and derives a new Cmk
 * (output) from the shared secret, tagged for key_usage. Unlike the other
 * CM_* commands with a variable-length payload, incoming_exchange_data is a
 * fixed CMB_ECDH_EXCHANGE_DATA_MAX_SIZE quantity, so this request is fully
 * fixed-size and goes through cptra_mci_mbox_execute(), not the _sg() variant.
 */
struct cptra_mci_ecdh_finish_req {
	struct cptra_mci_mbox_req_hdr hdr;
	uint8_t context[CPTRA_MCI_ECDH_CONTEXT_SIZE];
	uint32_t key_usage;
	uint8_t incoming_exchange_data[CPTRA_MCI_ECDH_EXCHANGE_DATA_SIZE];
};

struct cptra_mci_ecdh_finish_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	struct cptra_mci_cmk output;
};

/* MC_ECDSA_CMK_PUBLIC_KEY (CmEcdsaPublicKeyReq/Resp) */
struct cptra_mci_ecdsa_pubkey_req {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk cmk;
};

struct cptra_mci_ecdsa_pubkey_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t qx[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t qy[CPTRA_MCI_ECC384_SCALAR_SIZE];
};

/*
 * MC_ECDSA_CMK_SIGN (CmEcdsaSignReq/Resp). As with the SHA commands, only the
 * fixed-size header is modeled; the message payload is supplied separately to
 * cptra_mci_mbox_execute_sg(). There is no streaming/context field here, so
 * message_size must fit within a single mailbox call (CPTRA_MCI_MBOX_MAX_INPUT_SIZE)
 * -- callers sign a digest, not arbitrary-length data.
 */
struct cptra_mci_ecdsa_sign_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk cmk;
	uint32_t message_size;
};

struct cptra_mci_ecdsa_sign_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t r[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t s[CPTRA_MCI_ECC384_SCALAR_SIZE];
};

/*
 * MC_ECDSA_CMK_VERIFY (CmEcdsaVerifyReq/Resp). The response carries no explicit
 * pass/fail field -- a signature mismatch surfaces as CMD_FAILURE from the
 * mailbox itself (cptra_mci_mbox_execute() returning -EIO), not a payload flag.
 */
struct cptra_mci_ecdsa_verify_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk cmk;
	uint8_t r[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t s[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint32_t message_size;
};

struct cptra_mci_ecdsa_verify_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
};

#define CPTRA_MCI_HMAC_MAX_SIZE			64	/* CMB_HMAC_MAX_SIZE (SHA-512 digest) */

/*
 * MC_HMAC (CmHmacReq/Resp). Only the fixed-size header is modeled; the (up to
 * CPTRA_MCI_MBOX_MAX_INPUT_SIZE byte) input payload is supplied separately to
 * cptra_mci_mbox_execute_sg(). The response is variable-size: mac_len depends
 * on hash_algorithm (48 bytes for SHA384, 64 for SHA512).
 */
struct cptra_mci_hmac_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk cmk;
	uint32_t hash_algorithm;
	uint32_t data_size;
};

struct cptra_mci_hmac_resp {
	struct cptra_mci_mbox_resp_hdr_var hdr;
	uint8_t mac[CPTRA_MCI_HMAC_MAX_SIZE];
};

/*
 * MC_HMAC_KDF_COUNTER (CmHmacKdfCounterReq/Resp). Derives a new Cmk (kout)
 * from an existing Cmk (kin) via counter-mode HMAC-KDF. As with MC_HMAC, only
 * the fixed-size header is modeled; the label payload is supplied separately
 * to cptra_mci_mbox_execute_sg(). The response is fixed-size (not var-size).
 */
struct cptra_mci_hmac_kdf_counter_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk kin;
	uint32_t hash_algorithm;
	uint32_t key_usage;
	uint32_t key_size;
	uint32_t label_size;
};

struct cptra_mci_hmac_kdf_counter_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	struct cptra_mci_cmk kout;
};

/*
 * MC_HKDF_EXTRACT (CmHkdfExtractReq/Resp). Unlike the other CM_* commands in
 * this batch, both inputs (salt, ikm) are already-opaque Cmk handles rather
 * than raw payload bytes, so the request has no variable-length tail at all
 * -- it is fully fixed-size and goes through cptra_mci_mbox_execute(), not
 * the _sg() variant.
 */
struct cptra_mci_hkdf_extract_req {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t hash_algorithm;
	struct cptra_mci_cmk salt;
	struct cptra_mci_cmk ikm;
};

struct cptra_mci_hkdf_extract_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	struct cptra_mci_cmk prk;
};

/*
 * MC_HKDF_EXPAND (CmHkdfExpandReq/Resp). Derives a new Cmk (okm) from a
 * pseudo-random key Cmk (prk, typically the output of MC_HKDF_EXTRACT). Only
 * the fixed-size header is modeled; the info payload is supplied separately
 * to cptra_mci_mbox_execute_sg().
 */
struct cptra_mci_hkdf_expand_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk prk;
	uint32_t hash_algorithm;
	uint32_t key_usage;
	uint32_t key_size;
	uint32_t info_size;
};

struct cptra_mci_hkdf_expand_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	struct cptra_mci_cmk okm;
};

/* CmAesMode (mailbox.rs) */
enum cptra_mci_aes_mode {
	CPTRA_MCI_AES_MODE_RESERVED = 0,
	CPTRA_MCI_AES_MODE_CBC = 1,
	CPTRA_MCI_AES_MODE_CTR = 2,
};

#define CPTRA_MCI_AES_CONTEXT_SIZE		156	/* CMB_AES_ENCRYPTED_CONTEXT_SIZE */
#define CPTRA_MCI_AES_IV_SIZE			16

/*
 * MC_AES_ENCRYPT_INIT (CmAesEncryptInitReq/Resp). Only the fixed-size header
 * is modeled; the (up to CPTRA_MCI_MBOX_MAX_INPUT_SIZE byte) plaintext
 * payload is supplied separately to cptra_mci_mbox_execute_sg().
 */
struct cptra_mci_aes_encrypt_init_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk cmk;
	uint32_t mode;
	uint32_t plaintext_size;
};

/*
 * MC_AES_ENCRYPT_INIT's response only. caliptra-mcu-sw's wrapper type
 * (McuAesDecryptInitResp(pub CmAesEncryptInitResp), "reuse encrypt init
 * resp") claims MC_AES_DECRYPT_INIT's response is byte-for-byte identical to
 * this, but that does NOT hold on real hardware -- confirmed by a capture
 * where parsing decrypt_init's response against this (16 bytes too wide,
 * from the extra iv field) shifted every field after context by 16 bytes.
 * MC_AES_DECRYPT_INIT actually returns cptra_mci_aes_resp below, matching
 * its own Request::Resp trait declaration (CmAesResp) instead.
 */
struct cptra_mci_aes_init_resp_hdr {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE];
	uint8_t iv[CPTRA_MCI_AES_IV_SIZE];
	uint32_t output_size;
};

struct cptra_mci_aes_init_resp {
	struct cptra_mci_aes_init_resp_hdr hdr;
	uint8_t output[CPTRA_MCI_MBOX_MAX_INPUT_SIZE];
};

/*
 * MC_AES_ENCRYPT_UPDATE / MC_AES_DECRYPT_UPDATE (CmAesEncryptUpdateReq and
 * CmAesDecryptUpdateReq share this exact shape, just as MC_SHA_UPDATE and
 * MC_SHA_FINAL requests do). context is IN/OUT for the caller: read here as
 * input, and cptra_mci_aes_{en,de}crypt_update() overwrite it in place with
 * the value to feed into the next call, mirroring the SHA driver's context
 * threading.
 */
struct cptra_mci_aes_update_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE];
	uint32_t size;
};

/* Response shape shared by MC_AES_ENCRYPT_UPDATE and MC_AES_DECRYPT_UPDATE (CmAesResp). */
struct cptra_mci_aes_resp_hdr {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE];
	uint32_t output_size;
};

struct cptra_mci_aes_resp {
	struct cptra_mci_aes_resp_hdr hdr;
	uint8_t output[CPTRA_MCI_MBOX_MAX_INPUT_SIZE];
};

/*
 * MC_AES_DECRYPT_INIT (CmAesDecryptInitReq, response is cptra_mci_aes_resp --
 * see the comment on cptra_mci_aes_init_resp_hdr above for why NOT the
 * encrypt_init response shape, despite what the wrapper type implies).
 * Unlike encrypt_init, iv is an INPUT here (the value reported by the
 * original MC_AES_ENCRYPT_INIT call), since decryption has no prior context
 * to carry it.
 */
struct cptra_mci_aes_decrypt_init_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk cmk;
	uint32_t mode;
	uint8_t iv[CPTRA_MCI_AES_IV_SIZE];
	uint32_t ciphertext_size;
};

#define CPTRA_MCI_AES_GCM_CONTEXT_SIZE		128	/* CMB_AES_GCM_ENCRYPTED_CONTEXT_SIZE */
#define CPTRA_MCI_AES_GCM_IV_SIZE		12
#define CPTRA_MCI_AES_GCM_TAG_SIZE		16
#define CPTRA_MCI_AES_GCM_MAX_OUTPUT_SIZE	4112	/* MAX_CMB_DATA_SIZE + 16 */

/*
 * MC_AES_GCM_ENCRYPT_INIT (CmAesGcmEncryptInitReq/Resp). Unlike plain AES,
 * GCM's INIT step only sets up AAD (additional authenticated data) and
 * returns context+iv -- no plaintext/ciphertext flows until UPDATE or FINAL.
 * Only the fixed-size header is modeled; the (up to CPTRA_MCI_MBOX_MAX_INPUT_SIZE
 * byte) aad payload is supplied separately to cptra_mci_mbox_execute_sg().
 */
struct cptra_mci_aes_gcm_encrypt_init_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t flags;
	struct cptra_mci_cmk cmk;
	uint32_t aad_size;
};

/*
 * Response shape shared, byte-for-byte, by MC_AES_GCM_ENCRYPT_INIT and
 * MC_AES_GCM_DECRYPT_INIT (CmAesGcmEncryptInitResp / CmAesGcmDecryptInitResp
 * are separate Rust types but identically laid out: hdr+context+iv).
 */
struct cptra_mci_aes_gcm_init_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE];
	uint8_t iv[CPTRA_MCI_AES_GCM_IV_SIZE];
};

/*
 * MC_AES_GCM_ENCRYPT_UPDATE and MC_AES_GCM_ENCRYPT_FINAL requests share this
 * exact shape (hdr+context+size), and MC_AES_GCM_DECRYPT_UPDATE's request is
 * identically shaped too -- only the command ID sent distinguishes them.
 */
struct cptra_mci_aes_gcm_data_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE];
	uint32_t size;
};

/* Response shape shared by MC_AES_GCM_ENCRYPT_UPDATE and MC_AES_GCM_DECRYPT_UPDATE. */
struct cptra_mci_aes_gcm_update_resp_hdr {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE];
	uint32_t output_size;
};

struct cptra_mci_aes_gcm_update_resp {
	struct cptra_mci_aes_gcm_update_resp_hdr hdr;
	uint8_t output[CPTRA_MCI_AES_GCM_MAX_OUTPUT_SIZE];
};

/*
 * MC_AES_GCM_ENCRYPT_FINAL response: no more context (the stream is done);
 * carries the authentication tag instead.
 */
struct cptra_mci_aes_gcm_encrypt_final_resp_hdr {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t tag[CPTRA_MCI_AES_GCM_TAG_SIZE];
	uint32_t output_size;
};

struct cptra_mci_aes_gcm_encrypt_final_resp {
	struct cptra_mci_aes_gcm_encrypt_final_resp_hdr hdr;
	uint8_t output[CPTRA_MCI_AES_GCM_MAX_OUTPUT_SIZE];
};

/*
 * MC_AES_GCM_DECRYPT_INIT (CmAesGcmDecryptInitReq, response is
 * cptra_mci_aes_gcm_init_resp above). Unlike encrypt_init, iv is an INPUT
 * here (the value reported by the original MC_AES_GCM_ENCRYPT_INIT call).
 */
struct cptra_mci_aes_gcm_decrypt_init_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t flags;
	struct cptra_mci_cmk cmk;
	uint8_t iv[CPTRA_MCI_AES_GCM_IV_SIZE];
	uint32_t aad_size;
};

/*
 * MC_AES_GCM_DECRYPT_FINAL (CmAesGcmDecryptFinalReq). The tag to verify is a
 * fixed 16-byte quantity here, not part of the variable payload -- only the
 * trailing ciphertext chunk goes through cptra_mci_mbox_execute_sg().
 */
struct cptra_mci_aes_gcm_decrypt_final_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE];
	uint32_t tag_len;
	uint8_t tag[CPTRA_MCI_AES_GCM_TAG_SIZE];
	uint32_t ciphertext_size;
};

/*
 * MC_AES_GCM_DECRYPT_FINAL response. tag_verified is an explicit pass/fail
 * signal carried on the wire (unlike MC_ECDSA_CMK_VERIFY/MC_MLDSA_CMK_VERIFY,
 * which rely solely on mailbox CMD_FAILURE) -- cptra_mci_aes_gcm_decrypt_final()
 * treats a false tag_verified as -EBADMSG and does not release plaintext.
 */
struct cptra_mci_aes_gcm_decrypt_final_resp_hdr {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint32_t tag_verified;
	uint32_t output_size;
};

struct cptra_mci_aes_gcm_decrypt_final_resp {
	struct cptra_mci_aes_gcm_decrypt_final_resp_hdr hdr;
	uint8_t output[CPTRA_MCI_AES_GCM_MAX_OUTPUT_SIZE];
};

#define CPTRA_MCI_MLDSA87_PUBKEY_SIZE		2592	/* MLDSA87_PUB_KEY_BYTE_SIZE */
#define CPTRA_MCI_MLDSA87_SIGNATURE_SIZE	4628	/* MLDSA87_SIGNATURE_BYTE_SIZE */

/* MC_MLDSA_CMK_PUBLIC_KEY (CmMldsaPublicKeyReq/Resp) */
struct cptra_mci_mldsa_pubkey_req {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk cmk;
};

struct cptra_mci_mldsa_pubkey_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t public_key[CPTRA_MCI_MLDSA87_PUBKEY_SIZE];
};

/*
 * MC_MLDSA_CMK_SIGN (CmMldsaSignReq/Resp). As with the ECDSA CMK commands,
 * only the fixed-size header is modeled; the message payload is supplied
 * separately to cptra_mci_mbox_execute_sg(). The signature is a fixed
 * MLDSA87_SIGNATURE_BYTE_SIZE array, so unlike MC_HMAC there is no separate
 * length field -- the response is not variable-size.
 */
struct cptra_mci_mldsa_sign_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk cmk;
	uint32_t message_size;
};

struct cptra_mci_mldsa_sign_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
	uint8_t signature[CPTRA_MCI_MLDSA87_SIGNATURE_SIZE];
};

/*
 * MC_MLDSA_CMK_VERIFY (CmMldsaVerifyReq/Resp). Unlike ECDSA verify, the
 * signature here is large enough that it is itself part of this fixed-size
 * header rather than data handed to execute_sg() -- only the message is
 * supplied separately. As with ECDSA verify, the response carries no
 * explicit pass/fail field; a mismatch surfaces as CMD_FAILURE (-EIO).
 */
struct cptra_mci_mldsa_verify_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	struct cptra_mci_cmk cmk;
	uint8_t signature[CPTRA_MCI_MLDSA87_SIGNATURE_SIZE];
	uint32_t message_size;
};

struct cptra_mci_mldsa_verify_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
};

/*
 * MC_ECDSA384_SIG_VERIFY (EcdsaVerifyReq/Resp, pure passthrough to
 * Caliptra's ECDSA384_SIGNATURE_VERIFY). Like MC_LMS_SIG_VERIFY/
 * MC_MLDSA87_SIG_VERIFY, the public key is raw bytes (qx/qy), not an opaque
 * Cmk, and the caller must SHA-384-hash the message itself -- this command
 * takes the digest, not the raw message. No explicit pass/fail field; a
 * mismatch surfaces as CMD_FAILURE (-EIO).
 */
struct cptra_mci_ecdsa384_sig_verify_req {
	struct cptra_mci_mbox_req_hdr hdr;
	uint8_t pub_key_x[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t pub_key_y[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t signature_r[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t signature_s[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t hash[CPTRA_MCI_ECC384_SCALAR_SIZE];
};

struct cptra_mci_ecdsa384_sig_verify_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
};

#define CPTRA_MCI_LMS_PUBKEY_ID_SIZE		16	/* LMS "I" identifier */
#define CPTRA_MCI_LMS_PUBKEY_DIGEST_SIZE	24	/* N=6 words, LmsSha256N24H15 */
#define CPTRA_MCI_LMS_OTS_SIGNATURE_SIZE	1252	/* fixed param set, see below */
#define CPTRA_MCI_LMS_TREE_PATH_SIZE		360	/* H=15 levels * 24-byte digest */
#define CPTRA_MCI_LMS_HASH_SIZE			48	/* SHA-384 digest of the signed message */

/*
 * MC_LMS_SIG_VERIFY (LmsVerifyReq/Resp, pure passthrough to Caliptra's
 * LMS_SIGNATURE_VERIFY). Unlike MC_MLDSA_CMK_VERIFY/MC_ECDSA_CMK_VERIFY, the
 * public key here is raw bytes, not an opaque Cmk -- LMS keys are generated
 * offline (e.g. for firmware signing), never held as an on-device Cmk.
 * Caller must SHA-384-hash the message itself; this command takes the
 * digest, not the raw message. *_type fields exist on the wire as if the
 * parameter set were selectable, but Caliptra's runtime hard-codes and
 * rejects anything except tree_type=12 (LmsSha256N24H15) and
 * ots_type=7 -- the fixed-size arrays above are sized for exactly that one
 * parameter set. As with the other *_VERIFY commands, the response carries
 * no explicit pass/fail field; a mismatch surfaces as CMD_FAILURE (-EIO).
 */
#define CPTRA_MCI_LMS_TREE_TYPE_FIXED		12
#define CPTRA_MCI_LMS_OTS_TYPE_FIXED		7

struct cptra_mci_lms_verify_req {
	struct cptra_mci_mbox_req_hdr hdr;
	uint32_t pub_key_tree_type;
	uint32_t pub_key_ots_type;
	uint8_t pub_key_id[CPTRA_MCI_LMS_PUBKEY_ID_SIZE];
	uint8_t pub_key_digest[CPTRA_MCI_LMS_PUBKEY_DIGEST_SIZE];
	uint32_t signature_q;
	uint8_t signature_ots[CPTRA_MCI_LMS_OTS_SIGNATURE_SIZE];
	uint32_t signature_tree_type;
	uint8_t signature_tree_path[CPTRA_MCI_LMS_TREE_PATH_SIZE];
	uint8_t hash[CPTRA_MCI_LMS_HASH_SIZE];
};

struct cptra_mci_lms_verify_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
};

/*
 * MC_MLDSA87_SIG_VERIFY (MldsaVerifyReq/Resp, pure passthrough to Caliptra's
 * MLDSA87_SIGNATURE_VERIFY). Like MC_LMS_SIG_VERIFY, the public key is raw
 * bytes rather than an opaque Cmk. Unlike LMS, Caliptra hashes the message
 * internally, so this command takes the raw message (up to
 * CPTRA_MCI_MBOX_MAX_INPUT_SIZE), not a pre-hashed digest -- the message
 * payload is supplied separately to cptra_mci_mbox_execute_sg(), same as
 * MC_MLDSA_CMK_VERIFY. No explicit pass/fail field; a mismatch surfaces as
 * CMD_FAILURE (-EIO).
 */
struct cptra_mci_mldsa87_sig_verify_hdr {
	struct cptra_mci_mbox_req_hdr hdr;
	uint8_t pub_key[CPTRA_MCI_MLDSA87_PUBKEY_SIZE];
	uint8_t signature[CPTRA_MCI_MLDSA87_SIGNATURE_SIZE];
	uint32_t message_size;
};

struct cptra_mci_mldsa87_sig_verify_resp {
	struct cptra_mci_mbox_resp_hdr hdr;
};

/* cptra_mci_mbox.c */
uint32_t cptra_mci_mbox_checksum(uint32_t cmd, const void *data, uint32_t len);
uint32_t cptra_mci_mbox_checksum_ext(uint32_t checksum, const void *data, uint32_t len);
int cptra_mci_mbox_lock(void);
int cptra_mci_mbox_unlock(void);
uint32_t cptra_mci_mbox_status(void);
int cptra_mci_mbox_execute(uint32_t cmd, const void *req, uint32_t req_len,
			   void *resp, uint32_t resp_buf_len, uint32_t *resp_len);
int cptra_mci_mbox_execute_sg(uint32_t cmd, const void *hdr, uint32_t hdr_len,
			      const void *data, uint32_t data_len,
			      void *resp, uint32_t resp_buf_len, uint32_t *resp_len);
/*
 * For wrappers that keep their request/response in function-local `static`
 * storage: call _begin() before the first write to that storage and _end()
 * only after the last read of it, so a concurrent call to the same wrapper
 * can't interleave with it. Safe to call around cptra_mci_mbox_execute()/
 * execute_sg() -- those take the same underlying lock, and it supports
 * nested locking by the owning thread.
 */
void cptra_mci_mbox_txn_begin(void);
void cptra_mci_mbox_txn_end(void);
int cptra_mci_reg_session_begin(uint32_t page);
uint32_t cptra_mci_reg_session_read(uint32_t offset);
void cptra_mci_reg_session_end(void);

/* cptra_mci_misc.c */
int cptra_mci_get_firmware_version(enum cptra_mci_fw_index index, char *version, size_t len);
int cptra_mci_get_device_capabilities(uint8_t *caps, size_t len);
int cptra_mci_get_log(uint8_t *log, size_t len, size_t *log_len);
int cptra_mci_clear_log(void);
int cptra_mci_get_auth_cmd_challenge(uint32_t flags, uint8_t *challenge, size_t len);
int cptra_mci_export_attested_csr(enum cptra_mci_device_key_id device_key_id,
				  enum cptra_mci_csr_algo algorithm,
				  const uint8_t nonce[CPTRA_MCI_CSR_NONCE_SIZE],
				  uint8_t *csr, size_t len, size_t *csr_len);

/* cptra_mci_cryptographic_mbox.c (the CM_* cryptographic mailbox commands) */
int cptra_mci_cm_status(uint32_t *used_usage_storage, uint32_t *total_usage_storage);
int cptra_mci_random_stir(const uint8_t *input, size_t input_len);
int cptra_mci_random_generate(uint8_t *data, size_t len, size_t *data_len);
int cptra_mci_import_key(enum cptra_mci_key_usage key_usage, const uint8_t *key, size_t key_len,
			uint8_t cmk[CPTRA_MCI_CMK_SIZE]);
int cptra_mci_ecdh_generate(uint8_t context[CPTRA_MCI_ECDH_CONTEXT_SIZE],
			    uint8_t exchange_data[CPTRA_MCI_ECDH_EXCHANGE_DATA_SIZE]);
int cptra_mci_ecdh_finish(const uint8_t context[CPTRA_MCI_ECDH_CONTEXT_SIZE],
			  enum cptra_mci_key_usage key_usage,
			  const uint8_t incoming_exchange_data[CPTRA_MCI_ECDH_EXCHANGE_DATA_SIZE],
			  uint8_t output[CPTRA_MCI_CMK_SIZE]);
int cptra_mci_ecdsa_cmk_public_key(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
				   uint8_t qx[CPTRA_MCI_ECC384_SCALAR_SIZE],
				   uint8_t qy[CPTRA_MCI_ECC384_SCALAR_SIZE]);
int cptra_mci_ecdsa_cmk_sign(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
			    const uint8_t *message, size_t message_len,
			    uint8_t r[CPTRA_MCI_ECC384_SCALAR_SIZE],
			    uint8_t s[CPTRA_MCI_ECC384_SCALAR_SIZE]);
int cptra_mci_ecdsa_cmk_verify(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
			      const uint8_t r[CPTRA_MCI_ECC384_SCALAR_SIZE],
			      const uint8_t s[CPTRA_MCI_ECC384_SCALAR_SIZE],
			      const uint8_t *message, size_t message_len);
int cptra_mci_hmac(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], enum cptra_mci_sha_algo hash_algorithm,
		   const uint8_t *data, size_t data_len,
		   uint8_t *mac, size_t mac_buf_len, size_t *mac_len);
int cptra_mci_hmac_kdf_counter(const uint8_t kin[CPTRA_MCI_CMK_SIZE],
			       enum cptra_mci_sha_algo hash_algorithm,
			       enum cptra_mci_key_usage key_usage, uint32_t key_size,
			       const uint8_t *label, size_t label_len,
			       uint8_t kout[CPTRA_MCI_CMK_SIZE]);
int cptra_mci_hkdf_extract(enum cptra_mci_sha_algo hash_algorithm,
			   const uint8_t salt[CPTRA_MCI_CMK_SIZE],
			   const uint8_t ikm[CPTRA_MCI_CMK_SIZE],
			   uint8_t prk[CPTRA_MCI_CMK_SIZE]);
int cptra_mci_hkdf_expand(const uint8_t prk[CPTRA_MCI_CMK_SIZE],
			  enum cptra_mci_sha_algo hash_algorithm,
			  enum cptra_mci_key_usage key_usage, uint32_t key_size,
			  const uint8_t *info, size_t info_len,
			  uint8_t okm[CPTRA_MCI_CMK_SIZE]);
int cptra_mci_aes_encrypt_init(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], enum cptra_mci_aes_mode mode,
			       const uint8_t *plaintext, size_t plaintext_len,
			       uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE],
			       uint8_t iv[CPTRA_MCI_AES_IV_SIZE],
			       uint8_t *ciphertext, size_t ciphertext_buf_len,
			       size_t *ciphertext_len);
int cptra_mci_aes_encrypt_update(uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE],
				 const uint8_t *plaintext, size_t plaintext_len,
				 uint8_t *ciphertext, size_t ciphertext_buf_len,
				 size_t *ciphertext_len);
int cptra_mci_aes_decrypt_init(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], enum cptra_mci_aes_mode mode,
			       const uint8_t iv[CPTRA_MCI_AES_IV_SIZE],
			       const uint8_t *ciphertext, size_t ciphertext_len,
			       uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE],
			       uint8_t *plaintext, size_t plaintext_buf_len,
			       size_t *plaintext_len);
int cptra_mci_aes_decrypt_update(uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE],
				 const uint8_t *ciphertext, size_t ciphertext_len,
				 uint8_t *plaintext, size_t plaintext_buf_len,
				 size_t *plaintext_len);
int cptra_mci_aes_gcm_encrypt_init(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], uint32_t flags,
				   const uint8_t *aad, size_t aad_len,
				   uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				   uint8_t iv[CPTRA_MCI_AES_GCM_IV_SIZE]);
int cptra_mci_aes_gcm_encrypt_update(uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				    const uint8_t *plaintext, size_t plaintext_len,
				    uint8_t *ciphertext, size_t ciphertext_buf_len,
				    size_t *ciphertext_len);
int cptra_mci_aes_gcm_encrypt_final(uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				   const uint8_t *plaintext, size_t plaintext_len,
				   uint8_t *ciphertext, size_t ciphertext_buf_len,
				   size_t *ciphertext_len,
				   uint8_t tag[CPTRA_MCI_AES_GCM_TAG_SIZE]);
int cptra_mci_aes_gcm_decrypt_init(const uint8_t cmk[CPTRA_MCI_CMK_SIZE], uint32_t flags,
				   const uint8_t iv[CPTRA_MCI_AES_GCM_IV_SIZE],
				   const uint8_t *aad, size_t aad_len,
				   uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE]);
int cptra_mci_aes_gcm_decrypt_update(uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				    const uint8_t *ciphertext, size_t ciphertext_len,
				    uint8_t *plaintext, size_t plaintext_buf_len,
				    size_t *plaintext_len);
int cptra_mci_aes_gcm_decrypt_final(uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE],
				   const uint8_t tag[CPTRA_MCI_AES_GCM_TAG_SIZE],
				   const uint8_t *ciphertext, size_t ciphertext_len,
				   uint8_t *plaintext, size_t plaintext_buf_len,
				   size_t *plaintext_len);
int cptra_mci_mldsa_cmk_public_key(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
				   uint8_t public_key[CPTRA_MCI_MLDSA87_PUBKEY_SIZE]);
int cptra_mci_mldsa_cmk_sign(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
			    const uint8_t *message, size_t message_len,
			    uint8_t signature[CPTRA_MCI_MLDSA87_SIGNATURE_SIZE]);
int cptra_mci_mldsa_cmk_verify(const uint8_t cmk[CPTRA_MCI_CMK_SIZE],
			      const uint8_t signature[CPTRA_MCI_MLDSA87_SIGNATURE_SIZE],
			      const uint8_t *message, size_t message_len);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_CPTRA_MCI_MBOX_H_ */
