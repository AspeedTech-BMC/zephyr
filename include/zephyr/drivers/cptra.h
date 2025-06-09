/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CPTRA_H_
#define ZEPHYR_INCLUDE_DRIVERS_CPTRA_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

/* SCU register offsets */
#define SCU1_CPTRA				0x130
#define   SCU1_CPTRA_RDY_FOR_RT			BIT(18)

/* CPTRA MBOX register offsets */
#define CPTRA_MBOX_LOCK				0x00
#define CPTRA_MBOX_USER				0x04
#define CPTRA_MBOX_CMD				0x08
#define CPTRA_MBOX_DLEN				0x0c
#define CPTRA_MBOX_DATAIN			0x10
#define CPTRA_MBOX_DATAOUT			0x14
#define CPTRA_MBOX_EXEC				0x18
#define CPTRA_MBOX_STS				0x1c
#define   CPTRA_MBOX_STS_SOC_LOCK		BIT(9)
#define   CPTRA_MBOX_STS_FSM_PS			GENMASK(8, 6)
#define   CPTRA_MBOX_STS_PS			GENMASK(3, 0)
#define CPTRA_MBOX_UNLOCK			0x20

#define CPTRA_ECDSA_SIG_LEN			96	/* ECDSA384 */
#define CPTRA_ECDSA_SHA_LEN			48	/* SHA384 */
#define CPTRA_MBOX_SZ				0x20000	/* 128KB */

/* Mailbox commands */
enum cptra_mbox_cmd {
	CPTRA_MBCMD_ECDSA384_SIGNATURE_VERIFY       = 0x53494756, /* "SIGV" */
	CPTRA_MBCMD_LMS_SIGNATURE_VERIFY            = 0x4c4d5356, /* "LMSV" */
	CPTRA_MBCMD_CALIPTRA_FW_LOAD                = 0x46574c44, /* "FWLD" */
	CPTRA_MBCMD_STASH_MEASUREMENT               = 0x4d454153, /* "MEAS" */
	CPTRA_MBCMD_QUOTE_PCRS                      = 0x50435251, /* "PCRQ" */
	CPTRA_MBCMD_GET_IDEV_CERT                   = 0x49444543, /* "IDEC" */
	CPTRA_MBCMD_GET_IDEV_INFO                   = 0x49444549, /* "IDEI" */
	CPTRA_MBCMD_POPULATE_IDEV_CERT              = 0x49444550, /* "IDEP" */
	CPTRA_MBCMD_GET_LDEV_CERT                   = 0x4C444556, /* "LDEV" */
	CPTRA_MBCMD_GET_FMC_ALIAS_CERT              = 0x43455246, /* "CERF" */
	CPTRA_MBCMD_GET_RT_ALIAS_CERT               = 0x43455252, /* "CERR" */
	CPTRA_MBCMD_INVOKE_DPE_COMMAND              = 0x44504543, /* "DPEC" */
	CPTRA_MBCMD_DISABLE_ATTESTATION             = 0x4453424C, /* "DSBL" */
	CPTRA_MBCMD_FW_INFO                         = 0x494E464F, /* "INFO" */
	CPTRA_MBCMD_DPE_TAG_TCI                     = 0x54514754, /* "TAGT" */
	CPTRA_MBCMD_DPE_GET_TAGGED_TCI              = 0x47544744, /* "GTGD" */
	CPTRA_MBCMD_INCREMENT_PCR_RESET_COUNTER     = 0x50435252, /* "PCRR" */
	CPTRA_MBCMD_EXTEND_PCR                      = 0x50435245, /* "PCRE" */
	CPTRA_MBCMD_ADD_SUBJECT_ALT_NAME            = 0x414C544E, /* "ALTN" */
	CPTRA_MBCMD_CERTIFY_KEY_EXTENDED            = 0x434B4558, /* "CKEX" */
	CPTRA_MBCMD_FIPS_VERSION                    = 0x46505652, /* "FPVR" */
	CPTRA_MBCMD_SELF_TEST_START                 = 0x46504C54, /* "FPST" */
	CPTRA_MBCMD_SELF_TEST_GET_RESULTS           = 0x46504C67, /* "FPGR" */
	CPTRA_MBCMD_SHUTDOWN                        = 0x46505344, /* "FPSD" */
	CPTRA_MBCMD_CAPABILITIES                    = 0x43415053, /* "CAPS" */
	CPTRA_MBCMD_SET_AUTH_MANIFEST               = 0x41544D4E, /* "ATMN" */
	CPTRA_MBCMD_AUTHORIZE_AND_STASH             = 0x41545348, /* "ATSH" */
	CPTRA_MBCMD_GET_IDEVID_CSR                  = 0x49444352, /* "IDCR" */
	CPTRA_MBCMD_SIGN_WITH_EXPORTED_ECDSA        = 0x53574545, /* "SWEE" */
	CPTRA_MBCMD_REVOKE_EXPORTED_CDI_HANDLE      = 0x52564348, /* "RVCH" */
};

union cptra_mbox_lock_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t lock : 1;		/*[0-0]*/
		volatile uint32_t reserved : 31;	/*[1-31]*/
	} fields;
}; /* 0x00 */

union cptra_mbox_user_s {
	volatile uint32_t value;
}; /* 0x04 */

union cptra_mbox_cmd_s {
	volatile uint32_t value;
}; /* 0x08 */

union cptra_mbox_dlen_s {
	volatile uint32_t value;
}; /* 0x0C */

union cptra_mbox_datain_s {
	volatile uint32_t value;
}; /* 0x10 */

union cptra_mbox_dataout_s {
	volatile uint32_t value;
}; /* 0x14 */

union cptra_mbox_exec_s {
	volatile uint32_t value;
}; /* 0x18 */

union cptra_mbox_sts_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t status : 4;			/*[0-3]*/
		volatile uint32_t ecc_single_error : 1;		/*[4-4]*/
		volatile uint32_t ecc_double_error : 1;		/*[5-5]*/
		volatile uint32_t mbox_fsm_ps : 3;		/*[6-8]*/
		volatile uint32_t soc_has_lock : 1;		/*[9-9]*/
		volatile uint32_t mbox_rdptr : 15;		/*[10-24]*/
		volatile uint32_t reserved : 7;			/*[25-31]*/
	} fields;
}; /* 0x1C */

union cptra_mbox_unlock_s {
	volatile uint32_t value;
}; /* 0x20 */

struct cptra_mbox_register_s {
	union cptra_mbox_lock_s mbox_lock;			/* 00 */
	union cptra_mbox_user_s mbox_user;			/* 04 */
	union cptra_mbox_cmd_s mbox_cmd;			/* 08 */
	union cptra_mbox_dlen_s mbox_dlen;			/* 0C */
	union cptra_mbox_datain_s mbox_datain;			/* 10 */
	union cptra_mbox_dataout_s mbox_dataout;		/* 14 */
	union cptra_mbox_exec_s mbox_exec;			/* 18 */
	union cptra_mbox_sts_s mbox_sts;			/* 1C */
	union cptra_mbox_unlock_s mbox_unlock;			/* 20 */
};

enum cptra_mbox_sts {
	CPTRA_MBSTS_CMD_BUSY,
	CPTRA_MBSTS_DATA_READY,
	CPTRA_MBSTS_CMD_COMPLETE,
	CPTRA_MBSTS_CMD_FAILURE,
};

enum cptra_mbox_fsm {
	CPTRA_MBFSM_IDLE,
	CPTRA_MBFSM_RDY_FOR_CMD,
	CPTRA_MBFSM_RDY_FOR_DLEN,
	CPTRA_MBFSM_RDY_FOR_DATA,
	CPTRA_MBFSM_EXEC_UC,
	CPTRA_MBFSM_EXEC_SOC,
	CPTRA_MBFSM_ERROR,
};

/* SHA register offsets */
#define CPTRA_SHA_LOCK				0x00
#define CPTRA_SHA_USER				0x04
#define CPTRA_SHA_MODE				0x08
#define   CPTRA_SHA_MODE_ENDIAN			BIT(2)
#define   CPTRA_SHA_MODE_SEL			GENMASK(1, 0)
#define CPTRA_SHA_DLEN				0x10
#define CPTRA_SHA_DATAIN			0x14
#define CPTRA_SHA_EXEC				0x18
#define CPTRA_SHA_STS				0x1c
#define   CPTRA_SHA_STS_SOC_LOCK		BIT(1)
#define   CPTRA_SHA_STS_VLD			BIT(0)
#define CPTRA_SHA_DIGEST(n)			(0x20 + ((n) << 2))
#define CPTRA_SHA_CTRL				0x60
#define   CPTRA_SHA_CTRL_ZEROIZE		BIT(0)

union cptra_sha_lock_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t lock : 1;			/*[0-0]*/
		volatile uint32_t reserved : 31;		/*[1-31]*/
	} fields;
}; /* 0x00 */

union cptra_sha_user_s {
	volatile uint32_t value;
}; /* 0x04 */

union cptra_sha_mode_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t mode : 2;			/*[0-1]*/
		volatile uint32_t endian_toggle : 1;		/*[2-2]*/
		volatile uint32_t reserved : 29;		/*[3-31]*/
	} fields;
}; /* 0x08 */

union cptra_sha_addr_s {
	volatile uint32_t value;
}; /* 0x0C */

union cptra_sha_dlen_s {
	volatile uint32_t value;
}; /* 0x10 */

union cptra_sha_datain_s {
	volatile uint32_t value;
}; /* 0x14 */

union cptra_sha_exec_s {
	volatile uint32_t value;
}; /* 0x18 */

union cptra_sha_sts_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t valid : 1;			/*[0-0]*/
		volatile uint32_t soc_has_lock : 1;		/*[1-1]*/
		volatile uint32_t reserved : 30;		/*[2-31]*/
	} fields;
}; /* 0x1C */

union cptra_sha_digest_s {
	volatile uint32_t value;
}; /* 0x20 */

union cptra_sha_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t zeroize : 1;			/*[0-0]*/
		volatile uint32_t reserved : 31;		/*[1-31]*/
	} fields;
}; /* 0x60 */

struct cptra_sha_register_s {
	union cptra_sha_lock_s sha_lock;			/* 00 */
	union cptra_sha_user_s sha_user;			/* 04 */
	union cptra_sha_mode_s sha_mode;			/* 08 */
	union cptra_sha_addr_s sha_addr;			/* 0C */
	union cptra_sha_dlen_s sha_dlen;			/* 10 */
	union cptra_sha_datain_s sha_datain;			/* 14 */
	union cptra_sha_exec_s sha_exec;			/* 18 */
	union cptra_sha_sts_s sha_sts;				/* 1C */
	union cptra_sha_digest_s sha_digest[16];		/* 20 */
	union cptra_sha_ctrl_s sha_ctrl;			/* 60 */
};

/* SoC ifc register offsets */
#define CPTRA_HW_ERROR_FATAL			0x000
#define CPTRA_HW_ERROR_NONFATAL			0x004
#define CPTRA_FW_ERROR_FATAL			0x008
#define CPTRA_FW_ERROR_NONFATAL			0x00c
#define CPTRA_HW_ERROR_ENC			0x010
#define CPTRA_FW_ERROR_ENC			0x014
#define CPTRA_FW_EXT_ERROR_INFO			0x018
#define CPTRA_BOOT_STS				0x038
#define CPTRA_FLOW_STS				0x03c
#define CPTRA_BOOT_STS				0x038
#define CPTRA_FLOW_STS				0x03c
#define   CPTRA_FLOW_STS_RDY_FOR_FUSES		BIT(30)
#define   CPTRA_FLOW_STS_RDY_FOR_RT		BIT(29)
#define   CPTRA_FLOW_STS_RDY_FOR_FW		BIT(28)
#define CPTRA_RST_REASON			0x040
#define   CPTRA_FW_UPD_RESET			BIT(0)
#define   CPTRA_WARM_RESET			BIT(1)
#define CPTRA_TRNG_DATA(n)			(0x078 + ((n) << 2))
#define CPTRA_TRNG_STS				0x0ac
#define   CPTRA_TRNG_STS_DATA_WR_DONE		BIT(1)
#define   CPTRA_TRNG_STS_DATA_REQ		BIT(0)

#define CPTRA_MAX_TRNG				12

#define CPTRA_UPD_RST_TIMEOUT			1000
#define CPTRA_TRNG_REQ_LOOP_CNT			1000000		/* TODO: real chip exp */

struct cptra_stash_measurement_ia {
	uint8_t metadata[4];
	uint8_t measure[48];
	uint8_t context[48];
	uint32_t svn;
};

struct cptra_stash_measurement_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint32_t dpe_result;
};

struct cptra_quote_pcrs_ia {
	uint8_t nonce[32];
};

typedef uint8_t PcrValue[48];

struct cptra_quote_pcrs_oa {
	uint32_t chksum;
	uint32_t fips_status;
	PcrValue PCRs[32];
	uint8_t nonce[32];
	uint8_t digest[48];
	uint32_t reset_ctrs[32];
	uint8_t signature_r[48];
	uint8_t signature_s[48];
};

struct cptra_extend_pcr_ia {
	uint32_t index;
	uint8_t value[48];
};

struct cptra_extend_pcr_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_increment_pcr_reset_counter_ia {
	uint32_t index;
};

struct cptra_increment_pcr_reset_counter_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_dpe_tag_tci_ia {
	uint8_t handle[16];
	uint32_t tag;
};

struct cptra_dpe_tag_tci_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_dpe_get_tagged_tci_ia {
	uint32_t tag;
};

struct cptra_dpe_get_tagged_tci_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint8_t tci_cumulative[48];
	uint8_t tci_current[48];
};

struct cptra_add_subject_alt_name_ia {
	uint32_t dmtf_device_info_size;
	uint8_t dmtf_device_info[128];
};

struct cptra_add_subject_alt_name_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_certify_key_extended_ia {
	uint8_t certify_key_req[72];
	uint32_t flags;
};

struct cptra_certify_key_extended_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint8_t certify_key_resp[2176];
};

struct cptra_disable_attestation_ia {
};

struct cptra_disable_attestation_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_get_idev_cert_ia {
	uint32_t tbs_size;
	uint8_t signature_r[48];
	uint8_t signature_s[48];
	uint8_t tbs[916];
};

struct cptra_get_idev_cert_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint32_t cert_size;
	uint8_t cert[1024];
};

struct cptra_populate_idev_cert_ia {
	uint32_t cert_size;
	uint8_t cert[1024];
};

struct cptra_populate_idev_cert_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_get_idev_info_ia {
};

struct cptra_get_idev_info_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint8_t idev_pub_x[48];
	uint8_t idev_pub_y[48];
};

struct cptra_get_ldev_cert_ia {
};

struct cptra_get_ldev_cert_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint32_t data_size;
	uint8_t data[1024];
};

struct cptra_get_fmc_alias_cert_ia {
};

struct cptra_get_fmc_alias_cert_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint32_t data_size;
	uint8_t data[1024];
};

struct cptra_get_rt_alias_cert_ia {
};

struct cptra_get_rt_alias_cert_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint32_t data_size;
	uint8_t data[1024];
};

struct cptra_fw_info_ia {
};

struct cptra_fw_info_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint32_t pl0_pauser;
	uint32_t runtime_svn;
	uint32_t min_runtime_svn;
	uint32_t fmc_manifest_svn;
	uint32_t attestation_disabled;
	uint8_t rom_revision[20];
	uint8_t fmc_revision[20];
	uint8_t runtime_revision[20];
	uint32_t rom_sha256_digest[8];
	uint32_t fmc_sha384_digest[12];
	uint32_t runtime_sha384_digest[12];
	uint32_t owner_pub_key_hash[12];
};

struct cptra_capabilities_ia {
};

struct cptra_capabilities_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint8_t capabilities[16];
};

struct cptra_version_ia {
};

struct cptra_version_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint32_t mode;
	uint32_t fips_rev[3];
	uint8_t name[12];
};

struct cptra_self_test_start_ia {
};

struct cptra_self_test_start_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_self_test_get_results_ia {
};

struct cptra_self_test_get_results_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_shutdown_ia {
};

struct cptra_shutdown_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

/* AUTH_MANIFEST_FLAGS */
#define VENDOR_SIGNATURE_REQUIRED		BIT(0)

struct cptra_manifest_preamble {
	uint32_t manifest_marker;
	uint32_t preamble_size;
	uint32_t manifest_version;
	uint32_t manifest_flags;
	uint32_t manifest_vendor_ecc384_key[24];
	uint32_t manifest_vendor_lms_key[12];
	uint32_t manifest_vendor_ecc384_sig[24];
	uint32_t manifest_vendor_LMS_sig[405];
	uint32_t manifest_owner_ecc384_key[24];
	uint32_t manifest_owner_lms_key[12];
	uint32_t manifest_owner_ecc384_sig[24];
	uint32_t manifest_owner_LMS_sig[405];
	uint32_t metadata_vendor_ecc384_sig[24];
	uint32_t metadata_vendor_LMS_sig[405];
	uint32_t metadata_owner_ecc384_sig[24];
	uint32_t metadata_owner_LMS_sig[405];
};

struct cptra_set_auth_manifest_ia {
	uint32_t manifest_size;

	struct cptra_manifest_preamble preamble;
	uint32_t metadata_entry_entry_count;
	struct {
		uint32_t fw_id;
		uint32_t flags;
		uint8_t digest[48];
	} metadata_entries[1];
};

struct cptra_set_auth_manifest_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

struct cptra_authorize_and_stash_ia {
	uint8_t fw_id[4];
	uint8_t measurement[48];
	uint8_t context[48];
	uint32_t svn;
	uint32_t flags;
	uint32_t source;
};

struct cptra_authorize_and_stash_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint32_t auth_req_result;
};

enum image_hash_source {
	Invalid = 0,
	InRequest,
	ShaAcc,
};

#define AUTHORIZE_IMAGE				0xDEADC0DE
#define IMAGE_NOT_AUTHORIZED			0x21523F21
#define IMAGE_HASH_MISMATCH			0x8BFB95CB

#define AUTHORIZE_AND_STASH_FLAGS_SKIP_STASH	BIT(0)

struct cptra_get_idevid_csr_ia {
};

struct cptra_get_idevid_csr_oa {
	uint32_t chksum;
	uint32_t data_size;
	uint8_t data[1024]; /* Maximum size for the DER-encoded CSR */
};

struct cptra_sign_with_exported_ecdsa_ia {
	uint8_t exported_cdi_handle[32];
	uint8_t tbs[48];
};

struct cptra_sign_with_exported_ecdsa_oa {
	uint8_t derived_pubkey_x[48];
	uint8_t derived_pubkey_y[48];
	uint8_t signature_r[48];
	uint8_t signature_s[48];
};

struct cptra_revoke_exported_cdi_handle_ia {
	uint32_t chksum;
	uint8_t exported_cdi_handle[32];
};

struct cptra_revoke_exported_cdi_handle_oa {
	uint32_t chksum;
	uint32_t fips_status;
};

#define DPE_COMMAND_MAGIC			0x44504543	/* DPEC */

enum dpe_command {
	GET_PROFILE		= 0x01,
	INITIALIZE_CONTEXT	= 0x07,
	DERIVE_CONTEXT		= 0x08,
	CERTIFY_KEY		= 0x09,
	SIGN			= 0x0A,
	ROTATE_CONTEXT_HANDLE	= 0x0e,
	DESTROY_CONTEXT		= 0x0f,
	GET_CERTIFICATE_CHAIN	= 0x10,
};

enum dpe_profile {
	P256Sha256 = 1,
	P384Sha384 = 2,
};

/* TODO: DPE command */
struct dpe_cmd_header {
	uint32_t magic;
	uint32_t cmd;
	uint32_t profile;
};

struct dpe_get_profile_i {
	struct dpe_cmd_header cmd_hdr;
};

struct dpe_initialize_context_i {
	struct dpe_cmd_header cmd_hdr;
	uint32_t init_ctx_cmd;
};

struct dpe_derive_context_i {
	struct dpe_cmd_header cmd_hdr;
	uint8_t handle[16];
	uint8_t data[48];
	uint32_t flags;
	uint32_t tci_type;
	uint32_t target_locality;
};

struct dpe_certify_key_i {
	struct dpe_cmd_header cmd_hdr;
	uint8_t handle[16];
	uint32_t flags;
	uint32_t format;
	uint8_t label[48];
};

struct dpe_sign_i {
	struct dpe_cmd_header cmd_hdr;
	uint8_t handle[16];
	uint8_t label[48];
	uint32_t flags;
	uint8_t digest[48];
};

struct dpe_rotate_context_handle_i {
	struct dpe_cmd_header cmd_hdr;
	uint8_t handle[16];
	uint32_t flags;
};

struct dpe_destroy_context_i {
	struct dpe_cmd_header cmd_hdr;
	uint8_t handle[16];
};

struct dpe_get_certificate_chain_i {
	struct dpe_cmd_header cmd_hdr;
	uint32_t offset;
	uint32_t size;
};

struct cptra_invoke_dpe_command_ia {
	uint32_t data_size;
	uint8_t data[256];
};

struct cptra_invoke_dpe_command_oa {
	uint32_t chksum;
	uint32_t fips_status;
	uint32_t data_size;
	uint8_t data[2304];
};

/* The API a cptra driver should implement */
__subsystem struct cptra_driver_api {
	int (*caliptra_fw_upload)(const struct device *dev, uint8_t *buf, int size);
	int (*caliptra_stash_measurement)(const struct device *dev,
					  struct cptra_stash_measurement_ia *input,
					  struct cptra_stash_measurement_oa *output);
	int (*caliptra_quote_pcrs)(const struct device *dev, struct cptra_quote_pcrs_ia *input,
				   struct cptra_quote_pcrs_oa *output);
	int (*caliptra_extend_pcr)(const struct device *dev, struct cptra_extend_pcr_ia *input,
				   struct cptra_extend_pcr_oa *output);
	int (*caliptra_increment_pcr_reset_counter)(const struct device *dev,
				struct cptra_increment_pcr_reset_counter_ia *input,
				struct cptra_increment_pcr_reset_counter_oa *output);
	int (*caliptra_dpe_tag_tci)(const struct device *dev, struct cptra_dpe_tag_tci_ia *input,
				    struct cptra_dpe_tag_tci_oa *output);
	int (*caliptra_dpe_get_tagged_tci)(const struct device *dev,
					   struct cptra_dpe_get_tagged_tci_ia *input,
					   struct cptra_dpe_get_tagged_tci_oa *output);
	int (*caliptra_add_subject_alt_name)(const struct device *dev,
					     struct cptra_add_subject_alt_name_ia *input,
					     struct cptra_add_subject_alt_name_oa *output);
	int (*caliptra_certify_key_extended)(const struct device *dev,
					     struct cptra_certify_key_extended_ia *input,
					     struct cptra_certify_key_extended_oa *output);
	int (*caliptra_disable_attestation)(const struct device *dev,
					    struct cptra_disable_attestation_ia *input,
					    struct cptra_disable_attestation_oa *output);
	int (*caliptra_invoke_dpe_command)(const struct device *dev,
					   struct cptra_invoke_dpe_command_ia *input,
					   struct cptra_invoke_dpe_command_oa *output);
	int (*caliptra_get_idev_cert)(const struct device *dev,
				      struct cptra_get_idev_cert_ia *input,
				      struct cptra_get_idev_cert_oa *output);
	int (*caliptra_populate_idev_cert)(const struct device *dev,
					   struct cptra_populate_idev_cert_ia *input,
					   struct cptra_populate_idev_cert_oa *output);
	int (*caliptra_get_idev_info)(const struct device *dev,
				      struct cptra_get_idev_info_ia *input,
				      struct cptra_get_idev_info_oa *output);
	int (*caliptra_get_ldev_cert)(const struct device *dev,
				      struct cptra_get_ldev_cert_ia *input,
				      struct cptra_get_ldev_cert_oa *output);
	int (*caliptra_get_fmc_alias_cert)(const struct device *dev,
					   struct cptra_get_fmc_alias_cert_ia *input,
					   struct cptra_get_fmc_alias_cert_oa *output);
	int (*caliptra_get_rt_alias_cert)(const struct device *dev,
					  struct cptra_get_rt_alias_cert_ia *input,
					  struct cptra_get_rt_alias_cert_oa *output);
	int (*caliptra_fw_info)(const struct device *dev,
				struct cptra_fw_info_ia *input,
				struct cptra_fw_info_oa *output);
	int (*caliptra_capabilities)(const struct device *dev,
				     struct cptra_capabilities_ia *input,
				     struct cptra_capabilities_oa *output);
	int (*caliptra_version)(const struct device *dev,
				struct cptra_version_ia *input,
				struct cptra_version_oa *output);
	int (*caliptra_self_test_start)(const struct device *dev,
					struct cptra_self_test_start_ia *input,
					struct cptra_self_test_start_oa *output);
	int (*caliptra_self_test_get_results)(const struct device *dev,
					      struct cptra_self_test_get_results_ia *input,
					      struct cptra_self_test_get_results_oa *output);
	int (*caliptra_shutdown)(const struct device *dev,
				 struct cptra_shutdown_ia *input,
				 struct cptra_shutdown_oa *output);
	int (*caliptra_set_auth_manifest)(const struct device *dev,
					  struct cptra_set_auth_manifest_ia *input,
					  struct cptra_set_auth_manifest_oa *output);
	int (*caliptra_authorize_and_stash)(const struct device *dev,
					    struct cptra_authorize_and_stash_ia *input,
					    struct cptra_authorize_and_stash_oa *output);
	int (*caliptra_get_idevid_csr)(const struct device *dev,
				       struct cptra_get_idevid_csr_ia *input,
				       struct cptra_get_idevid_csr_oa *output);
	int (*caliptra_sign_with_exported_ecdsa)(const struct device *dev,
						 struct cptra_sign_with_exported_ecdsa_ia *input,
						 struct cptra_sign_with_exported_ecdsa_oa *output);
	int (*caliptra_revoke_exported_cdi_handle)(const struct device *dev,
						   struct cptra_revoke_exported_cdi_handle_ia *input,
						   struct cptra_revoke_exported_cdi_handle_oa *output);
};

static inline int caliptra_fw_upload(const struct device *dev, uint8_t *buf, int size)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_fw_upload(dev, buf, size);

	return tmp;
}

static inline int caliptra_stash_measurement(const struct device *dev,
					     struct cptra_stash_measurement_ia *input,
					     struct cptra_stash_measurement_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_stash_measurement(dev, input, output);

	return tmp;
}

static inline int caliptra_quote_pcrs(const struct device *dev, struct cptra_quote_pcrs_ia *input,
				      struct cptra_quote_pcrs_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_quote_pcrs(dev, input, output);

	return tmp;
}

static inline int caliptra_extend_pcr(const struct device *dev, struct cptra_extend_pcr_ia *input,
				      struct cptra_extend_pcr_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_extend_pcr(dev, input, output);

	return tmp;
}

static inline int
caliptra_increment_pcr_reset_counter(const struct device *dev,
				     struct cptra_increment_pcr_reset_counter_ia *input,
				     struct cptra_increment_pcr_reset_counter_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_increment_pcr_reset_counter(dev, input, output);

	return tmp;
}

static inline int caliptra_dpe_tag_tci(const struct device *dev, struct cptra_dpe_tag_tci_ia *input,
				       struct cptra_dpe_tag_tci_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_dpe_tag_tci(dev, input, output);

	return tmp;
}

static inline int caliptra_dpe_get_tagged_tci(const struct device *dev,
					      struct cptra_dpe_get_tagged_tci_ia *input,
					      struct cptra_dpe_get_tagged_tci_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_dpe_get_tagged_tci(dev, input, output);

	return tmp;
}

static inline int caliptra_add_subject_alt_name(const struct device *dev,
						struct cptra_add_subject_alt_name_ia *input,
						struct cptra_add_subject_alt_name_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_add_subject_alt_name(dev, input, output);

	return tmp;
}

static inline int caliptra_certify_key_extended(const struct device *dev,
						struct cptra_certify_key_extended_ia *input,
						struct cptra_certify_key_extended_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_certify_key_extended(dev, input, output);

	return tmp;
}

static inline int caliptra_disable_attestation(const struct device *dev,
					       struct cptra_disable_attestation_ia *input,
					       struct cptra_disable_attestation_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_disable_attestation(dev, input, output);

	return tmp;
}

static inline int caliptra_invoke_dpe_command(const struct device *dev,
					      struct cptra_invoke_dpe_command_ia *input,
					      struct cptra_invoke_dpe_command_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_invoke_dpe_command(dev, input, output);

	return tmp;
}

static inline int caliptra_get_idev_cert(const struct device *dev,
					 struct cptra_get_idev_cert_ia *input,
					 struct cptra_get_idev_cert_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_get_idev_cert(dev, input, output);

	return tmp;
}

static inline int caliptra_populate_idev_cert(const struct device *dev,
					      struct cptra_populate_idev_cert_ia *input,
					      struct cptra_populate_idev_cert_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_populate_idev_cert(dev, input, output);

	return tmp;
}

static inline int caliptra_get_idev_info(const struct device *dev,
					 struct cptra_get_idev_info_ia *input,
					 struct cptra_get_idev_info_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_get_idev_info(dev, input, output);

	return tmp;
}

static inline int caliptra_get_ldev_cert(const struct device *dev,
					 struct cptra_get_ldev_cert_ia *input,
					 struct cptra_get_ldev_cert_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_get_ldev_cert(dev, input, output);

	return tmp;
}

static inline int caliptra_get_fmc_alias_cert(const struct device *dev,
					      struct cptra_get_fmc_alias_cert_ia *input,
					      struct cptra_get_fmc_alias_cert_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_get_fmc_alias_cert(dev, input, output);

	return tmp;
}

static inline int caliptra_get_rt_alias_cert(const struct device *dev,
					     struct cptra_get_rt_alias_cert_ia *input,
					     struct cptra_get_rt_alias_cert_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_get_rt_alias_cert(dev, input, output);

	return tmp;
}

static inline int caliptra_fw_info(const struct device *dev, struct cptra_fw_info_ia *input,
				   struct cptra_fw_info_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_fw_info(dev, input, output);

	return tmp;
}

static inline int caliptra_capabilities(const struct device *dev,
					struct cptra_capabilities_ia *input,
					struct cptra_capabilities_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_capabilities(dev, input, output);

	return tmp;
}

static inline int caliptra_version(const struct device *dev, struct cptra_version_ia *input,
				   struct cptra_version_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_version(dev, input, output);

	return tmp;
}

static inline int caliptra_self_test_start(const struct device *dev,
					   struct cptra_self_test_start_ia *input,
					   struct cptra_self_test_start_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_self_test_start(dev, input, output);

	return tmp;
}

static inline int caliptra_self_test_get_results(const struct device *dev,
						 struct cptra_self_test_get_results_ia *input,
						 struct cptra_self_test_get_results_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_self_test_get_results(dev, input, output);

	return tmp;
}

static inline int caliptra_shutdown(const struct device *dev, struct cptra_shutdown_ia *input,
				    struct cptra_shutdown_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_shutdown(dev, input, output);

	return tmp;
}

static inline int caliptra_set_auth_manifest(const struct device *dev,
					     struct cptra_set_auth_manifest_ia *input,
					     struct cptra_set_auth_manifest_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_set_auth_manifest(dev, input, output);

	return tmp;
}

static inline int caliptra_authorize_and_stash(const struct device *dev,
					       struct cptra_authorize_and_stash_ia *input,
					       struct cptra_authorize_and_stash_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_authorize_and_stash(dev, input, output);

	return tmp;
}

static inline int caliptra_get_idevid_csr(const struct device *dev,
					  struct cptra_get_idevid_csr_ia *input,
					  struct cptra_get_idevid_csr_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_get_idevid_csr(dev, input, output);

	return tmp;
}

static inline int
caliptra_sign_with_exported_ecdsa(const struct device *dev,
				  struct cptra_sign_with_exported_ecdsa_ia *input,
				  struct cptra_sign_with_exported_ecdsa_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_sign_with_exported_ecdsa(dev, input, output);

	return tmp;
}

static inline int
caliptra_revoke_exported_cdi_handle(const struct device *dev,
				    struct cptra_revoke_exported_cdi_handle_ia *input,
				    struct cptra_revoke_exported_cdi_handle_oa *output)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_revoke_exported_cdi_handle(dev, input, output);

	return tmp;
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_CPTRA_H_ */
