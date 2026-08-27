/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/ecdsa.h>
#include <zephyr/crypto/hash.h>
#include <zephyr/crypto/lms.h>
#include <zephyr/crypto/mldsa.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>

static int cmd_cptra_mci_fw_version(const struct shell *shell, size_t argc, char **argv)
{
	enum cptra_mci_fw_index index = CPTRA_MCI_FW_INDEX_CALIPTRA_CORE;
	char version[CPTRA_MCI_MAX_FW_VERSION_STR_LEN + 1];
	int ret;

	if (argc > 1)
		index = (enum cptra_mci_fw_index)strtoul(argv[1], NULL, 0);

	ret = cptra_mci_get_firmware_version(index, version, sizeof(version));
	if (ret) {
		shell_error(shell, "MC_FIRMWARE_VERSION failed: %d", ret);
		return ret;
	}

	shell_print(shell, "index %d: %s", index, version);

	return 0;
}

/*
 * Dump of the Caliptra-SS subsystem status registers (mci_reg and
 * soc_ifc_reg blocks) via the cptra_mci_reg_session_* calls -- plain
 * memory-mapped register reads through the paged SCU1 window, not mailbox
 * protocol commands. See the comment on CPTRA_MCI_REG_PAGE/
 * CPTRA_MCI_SOC_IFC_PAGE in cptra_mci_mbox.h for the page-encoding caveat.
 *
 * Each register list below is dumped inside a single
 * cptra_mci_reg_session_begin()/_end() pair per page, so a whole section's
 * worth of registers costs one lock/page-select instead of one per
 * register.
 */

struct cptra_mci_reg_desc {
	const char *label;
	uint32_t offset;
};

struct cptra_mci_reg_flag_desc {
	const char *label;
	uint32_t mask;
};

/* Section header: page is shown once here instead of on every register line. */
static void cptra_mci_reg_section(const struct shell *shell, const char *name, uint32_t page)
{
	shell_print(shell, "\n== %s (page 0x%08x) ==", name, page);
}

/* Bit-field sub-line, indented further than the register line it belongs to. */
static void cptra_mci_reg_field_print(const struct shell *shell, const char *name, uint32_t value)
{
	shell_print(shell, "        %-20s = %u", name, value);
}

static const char *cptra_mci_device_lifecycle_name(uint32_t val)
{
	switch (val) {
	case CPTRA_MCI_DEVICE_UNPROVISIONED:
		return "UNPROVISIONED";
	case CPTRA_MCI_DEVICE_MANUFACTURING:
		return "MANUFACTURING";
	case CPTRA_MCI_DEVICE_PRODUCTION:
		return "PRODUCTION";
	default:
		return "RESERVED";
	}
}

/* Must be called inside an open cptra_mci_reg_session_begin()/_end() pair. */
static void cptra_mci_reg_dump_list(const struct shell *shell,
				    const struct cptra_mci_reg_desc *list, size_t count)
{
	size_t i;

	for (i = 0; i < count; i++) {
		uint32_t value = cptra_mci_reg_session_read(list[i].offset);

		shell_print(shell, "  %-30s 0x%04x = 0x%08x", list[i].label, list[i].offset,
			    value);
	}
}

static void cptra_mci_reg_dump_array(const struct shell *shell, const char *label,
				     uint32_t base_offset, uint32_t count)
{
	char name[40];
	uint32_t i;

	for (i = 0; i < count; i++) {
		uint32_t offset = base_offset + 4 * i;
		uint32_t value = cptra_mci_reg_session_read(offset);

		snprintk(name, sizeof(name), "%s[%u]", label, i);
		shell_print(shell, "  %-30s 0x%04x = 0x%08x", name, offset, value);
	}
}

static void cptra_mci_reg_print_flags(const struct shell *shell, const char *label,
				      uint32_t offset, uint32_t value,
				      const struct cptra_mci_reg_flag_desc *flags, size_t count)
{
	size_t i;

	shell_print(shell, "  %-30s 0x%04x = 0x%08x", label, offset, value);

	for (i = 0; i < count; i++)
		cptra_mci_reg_field_print(shell, flags[i].label, FIELD_GET(flags[i].mask, value));
}

static void cptra_mci_reg_print_security_state(const struct shell *shell, const char *label,
					       uint32_t offset, uint32_t value,
					       uint32_t lifecycle_mask, uint32_t debug_locked_mask,
					       uint32_t scan_mode_mask)
{
	uint32_t lifecycle = FIELD_GET(lifecycle_mask, value);

	shell_print(shell, "  %-30s 0x%04x = 0x%08x", label, offset, value);
	shell_print(shell, "        %-20s = %u (%s)", "device_lifecycle", lifecycle,
		    cptra_mci_device_lifecycle_name(lifecycle));
	cptra_mci_reg_field_print(shell, "debug_locked", FIELD_GET(debug_locked_mask, value));
	cptra_mci_reg_field_print(shell, "scan_mode", FIELD_GET(scan_mode_mask, value));
}

static const struct cptra_mci_reg_desc cptra_mci_reg_axi_user_list[] = {
	{ "MCU_IFU_AXI_USER", CPTRA_MCI_REG_MCU_IFU_AXI_USER },
	{ "MCU_LSU_AXI_USER", CPTRA_MCI_REG_MCU_LSU_AXI_USER },
	{ "MCU_SRAM_CONFIG_AXI_USER", CPTRA_MCI_REG_MCU_SRAM_CONFIG_AXI_USER },
	{ "MCI_SOC_CONFIG_AXI_USER", CPTRA_MCI_REG_MCI_SOC_CONFIG_AXI_USER },
};

static const struct cptra_mci_reg_desc cptra_mci_reg_ss_list[] = {
	{ "SS_DEBUG_INTENT", CPTRA_MCI_REG_SS_DEBUG_INTENT },
	{ "SS_CONFIG_DONE_STICKY", CPTRA_MCI_REG_SS_CONFIG_DONE_STICKY },
	{ "SS_CONFIG_DONE", CPTRA_MCI_REG_SS_CONFIG_DONE },
};

static const struct cptra_mci_reg_flag_desc cptra_mci_reg_reset_reason_flags[] = {
	{ "WARM_RESET", CPTRA_MCI_REG_RESET_REASON_WARM_RESET },
	{ "FW_BOOT_UPD_RESET", CPTRA_MCI_REG_RESET_REASON_FW_BOOT_UPD_RESET },
	{ "FW_HITLESS_UPD_RESET", CPTRA_MCI_REG_RESET_REASON_FW_HITLESS_UPD_RESET },
};

static const struct cptra_mci_reg_desc cptra_mci_soc_ifc_std_list[] = {
	{ "CPTRA_TRNG_VALID_AXI_USER", CPTRA_MCI_SOC_IFC_CPTRA_TRNG_VALID_AXI_USER },
	{ "CPTRA_TRNG_AXI_USER_LOCK", CPTRA_MCI_SOC_IFC_CPTRA_TRNG_AXI_USER_LOCK },
	{ "CPTRA_FUSE_VALID_AXI_USER", CPTRA_MCI_SOC_IFC_CPTRA_FUSE_VALID_AXI_USER },
};

static const struct cptra_mci_reg_flag_desc cptra_mci_soc_ifc_reset_reason_flags[] = {
	{ "WARM_RESET", CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON_WARM_RESET },
	{ "FW_UPD_RESET", CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON_FW_UPD_RESET },
};

static int cmd_cptra_mci_subsystem_info(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t value;
	int ret;

	cptra_mci_reg_section(shell, "mci_reg", CPTRA_MCI_REG_PAGE);

	ret = cptra_mci_reg_session_begin(CPTRA_MCI_REG_PAGE);
	if (ret) {
		shell_error(shell, "failed to select mci_reg page: %d", ret);
		return ret;
	}

	cptra_mci_reg_dump_list(shell, cptra_mci_reg_axi_user_list,
				ARRAY_SIZE(cptra_mci_reg_axi_user_list));

	value = cptra_mci_reg_session_read(CPTRA_MCI_REG_RESET_REASON);
	cptra_mci_reg_print_flags(shell, "RESET_REASON", CPTRA_MCI_REG_RESET_REASON, value,
				  cptra_mci_reg_reset_reason_flags,
				  ARRAY_SIZE(cptra_mci_reg_reset_reason_flags));

	value = cptra_mci_reg_session_read(CPTRA_MCI_REG_SECURITY_STATE);
	cptra_mci_reg_print_security_state(shell, "SECURITY_STATE", CPTRA_MCI_REG_SECURITY_STATE,
					   value, CPTRA_MCI_REG_SECURITY_STATE_DEVICE_LIFECYCLE,
					   CPTRA_MCI_REG_SECURITY_STATE_DEBUG_LOCKED,
					   CPTRA_MCI_REG_SECURITY_STATE_SCAN_MODE);

	cptra_mci_reg_dump_array(shell, "MBOX0_VALID_AXI_USER",
				 CPTRA_MCI_REG_MBOX0_VALID_AXI_USER(0),
				 CPTRA_MCI_REG_MBOX_AXI_USER_COUNT);
	cptra_mci_reg_dump_array(shell, "MBOX0_AXI_USER_LOCK",
				 CPTRA_MCI_REG_MBOX0_AXI_USER_LOCK(0),
				 CPTRA_MCI_REG_MBOX_AXI_USER_COUNT);
	cptra_mci_reg_dump_array(shell, "MBOX1_VALID_AXI_USER",
				 CPTRA_MCI_REG_MBOX1_VALID_AXI_USER(0),
				 CPTRA_MCI_REG_MBOX_AXI_USER_COUNT);
	cptra_mci_reg_dump_array(shell, "MBOX1_AXI_USER_LOCK",
				 CPTRA_MCI_REG_MBOX1_AXI_USER_LOCK(0),
				 CPTRA_MCI_REG_MBOX_AXI_USER_COUNT);

	cptra_mci_reg_dump_list(shell, cptra_mci_reg_ss_list, ARRAY_SIZE(cptra_mci_reg_ss_list));

	cptra_mci_reg_session_end();

	cptra_mci_reg_section(shell, "soc_ifc_reg", CPTRA_MCI_SOC_IFC_PAGE);

	ret = cptra_mci_reg_session_begin(CPTRA_MCI_SOC_IFC_PAGE);
	if (ret) {
		shell_error(shell, "failed to select soc_ifc_reg page: %d", ret);
		return ret;
	}

	value = cptra_mci_reg_session_read(CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON);
	cptra_mci_reg_print_flags(shell, "CPTRA_RESET_REASON", CPTRA_MCI_SOC_IFC_CPTRA_RESET_REASON,
				  value, cptra_mci_soc_ifc_reset_reason_flags,
				  ARRAY_SIZE(cptra_mci_soc_ifc_reset_reason_flags));

	value = cptra_mci_reg_session_read(CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE);
	cptra_mci_reg_print_security_state(shell, "CPTRA_SECURITY_STATE",
					   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE, value,
					   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE_DEVICE_LIFECYCLE,
					   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE_DEBUG_LOCKED,
					   CPTRA_MCI_SOC_IFC_CPTRA_SECURITY_STATE_SCAN_MODE);

	cptra_mci_reg_dump_array(shell, "CPTRA_MBOX_VALID_AXI_USER",
				 CPTRA_MCI_SOC_IFC_CPTRA_MBOX_VALID_AXI_USER(0),
				 CPTRA_MCI_SOC_IFC_MBOX_AXI_USER_COUNT);
	cptra_mci_reg_dump_array(shell, "CPTRA_MBOX_AXI_USER_LOCK",
				 CPTRA_MCI_SOC_IFC_CPTRA_MBOX_AXI_USER_LOCK(0),
				 CPTRA_MCI_SOC_IFC_MBOX_AXI_USER_COUNT);

	cptra_mci_reg_dump_list(shell, cptra_mci_soc_ifc_std_list,
				ARRAY_SIZE(cptra_mci_soc_ifc_std_list));

	cptra_mci_reg_session_end();

	return 0;
}

static int cmd_cptra_mci_device_caps(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t caps[CPTRA_MCI_DEVICE_CAPS_SIZE];
	int ret;

	ret = cptra_mci_get_device_capabilities(caps, sizeof(caps));
	if (ret) {
		shell_error(shell, "MC_DEVICE_CAPABILITIES failed: %d", ret);
		return ret;
	}

	shell_hexdump(shell, caps, sizeof(caps));

	return 0;
}

/*
 * Expected ECDSA public key for the all-zero 48-byte seed imported below
 * (see caliptra-mcu-sw's ecc384_test()) -- captured once from real hardware,
 * used to make the pubkey step a known-answer check instead of an
 * unverifiable printout.
 */
static const uint8_t cptra_mci_ecdsa_test_pubkey_x[CPTRA_MCI_ECC384_SCALAR_SIZE] = {
	0xd7, 0xdd, 0x94, 0xe0, 0xbf, 0xfc, 0x4c, 0xad, 0xe9, 0x90, 0x2b, 0x7f, 0xdb, 0x15, 0x42,
	0x60, 0xd5, 0xec, 0x5d, 0xfd, 0x57, 0x95, 0x0e, 0x83, 0x59, 0x01, 0x5a, 0x30, 0x2c, 0x8b,
	0xf7, 0xbb, 0xa7, 0xe5, 0xf6, 0xdf, 0xfc, 0x16, 0x85, 0x16, 0x2b, 0xdd, 0x35, 0xf9, 0xf5,
	0xc1, 0xb0, 0xff,
};

static const uint8_t cptra_mci_ecdsa_test_pubkey_y[CPTRA_MCI_ECC384_SCALAR_SIZE] = {
	0xbb, 0x9c, 0x3a, 0x2f, 0x06, 0x1e, 0x8d, 0x70, 0x14, 0x27, 0x8d, 0xd5, 0x1e, 0x66, 0xa9,
	0x18, 0xa6, 0xb6, 0xf9, 0xf1, 0xc1, 0x93, 0x73, 0x12, 0xd4, 0xe7, 0xa9, 0x21, 0xb1, 0x8e,
	0xf0, 0xf4, 0x1f, 0xdd, 0x40, 0x1d, 0x9e, 0x77, 0x18, 0x50, 0x9f, 0x87, 0x31, 0xe9, 0xee,
	0xc9, 0xc3, 0x1d,
};

static int cmd_cptra_mci_ecdsa_test(const struct shell *shell, size_t argc, char **argv)
{
	/* All-zero seed: pubkey is then a known, fixed value (see the comment above). */
	static const uint8_t key[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t cmk[CPTRA_MCI_CMK_SIZE];
	uint8_t qx[CPTRA_MCI_ECC384_SCALAR_SIZE], qy[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t digest[CPTRA_MCI_ECC384_SCALAR_SIZE], bad_digest[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t r[CPTRA_MCI_ECC384_SCALAR_SIZE], s[CPTRA_MCI_ECC384_SCALAR_SIZE];
	size_t digest_len;
	int ret;

	ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_ECDSA, key, sizeof(key), cmk);
	if (ret) {
		shell_error(shell, "MC_IMPORT failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_ecdsa_cmk_public_key(cmk, qx, qy);
	if (ret) {
		shell_error(shell, "MC_ECDSA_CMK_PUBLIC_KEY failed: %d", ret);
		return ret;
	}

	shell_print(shell, "qx:");
	shell_hexdump(shell, qx, sizeof(qx));
	shell_print(shell, "qy:");
	shell_hexdump(shell, qy, sizeof(qy));

	if (memcmp(qx, cptra_mci_ecdsa_test_pubkey_x, sizeof(qx)) != 0 ||
	    memcmp(qy, cptra_mci_ecdsa_test_pubkey_y, sizeof(qy)) != 0) {
		shell_error(shell, "public key does not match known-answer vector");
		return -EIO;
	}
	shell_print(shell, "public key matches known-answer vector: PASS");

	if (argc > 1) {
		digest_len = hex2bin(argv[1], strlen(argv[1]), digest, sizeof(digest));
		if (digest_len == 0) {
			shell_error(shell, "bad digest hex string");
			return -EINVAL;
		}
	} else {
		/* No digest given: fill with pseudo-random bytes for a quick smoke test. */
		digest_len = sizeof(digest);

		for (size_t i = 0; i < digest_len; i += sizeof(uint32_t)) {
			uint32_t rnd = k_cycle_get_32() ^ (uint32_t)i;

			memcpy(&digest[i], &rnd, MIN(sizeof(rnd), digest_len - i));
		}

		shell_print(shell, "no digest_hex given, generated random %u-byte digest",
			    (unsigned int)digest_len);
	}

	ret = cptra_mci_ecdsa_cmk_sign(cmk, digest, digest_len, r, s);
	if (ret) {
		shell_error(shell, "MC_ECDSA_CMK_SIGN failed: %d", ret);
		return ret;
	}

	/* Positive case: verify against the digest that was actually signed. */
	ret = cptra_mci_ecdsa_cmk_verify(cmk, r, s, digest, digest_len);
	if (ret) {
		shell_error(shell, "positive case FAIL (expected PASS): %d", ret);
		return ret;
	}
	shell_print(shell, "positive case (correct digest): PASS");

	/* Negative case: same signature, a tampered digest -- must be rejected. */
	memcpy(bad_digest, digest, digest_len);
	bad_digest[0] ^= 0xFF;

	ret = cptra_mci_ecdsa_cmk_verify(cmk, r, s, bad_digest, digest_len);
	if (ret == 0) {
		shell_error(shell, "negative case FAIL (a tampered digest was accepted!)");
		return -EIO;
	}
	shell_print(shell, "negative case (tampered digest): correctly rejected (%d)", ret);

	shell_print(shell, "MC_ECDSA_CMK_PUBLIC_KEY/SIGN/VERIFY: PASS "
		    "(pubkey retrieved, positive and negative verify cases both OK)");

	return 0;
}

/*
 * RFC 4231 section 4.2/4.3 HMAC-SHA test cases 1 and 2. MC_IMPORT for Hmac
 * usage requires the raw key material to be exactly 48 (SHA384) or 64
 * (SHA512) bytes, so the actual imported key is the RFC key zero-padded out
 * to that fixed size -- the expected MACs below were captured against that
 * padded key on real hardware (see caliptra-mcu-sw's cm_hmac_run_case /
 * cm_hmac_hkdf_test), not the RFC's own published value for the raw
 * (unpadded) key.
 */
static const uint8_t cptra_mci_hmac_test_key_1[20] = {
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
};

static const uint8_t cptra_mci_hmac_test_data_1[8] = {
	0x48, 0x69, 0x20, 0x54, 0x68, 0x65, 0x72, 0x65, /* "Hi There" */
};

static const uint8_t cptra_mci_hmac_test_sha384_1[SHA384_DIGEST_SIZE] = {
	0xaf, 0xd0, 0x39, 0x44, 0xd8, 0x48, 0x95, 0x62, 0x6b, 0x08, 0x25, 0xf4, 0xab, 0x46, 0x90,
	0x7f, 0x15, 0xf9, 0xda, 0xdb, 0xe4, 0x10, 0x1e, 0xc6, 0x82, 0xaa, 0x03, 0x4c, 0x7c, 0xeb,
	0xc5, 0x9c, 0xfa, 0xea, 0x9e, 0xa9, 0x07, 0x6e, 0xde, 0x7f, 0x4a, 0xf1, 0x52, 0xe8, 0xb2,
	0xfa, 0x9c, 0xb6,
};

static const uint8_t cptra_mci_hmac_test_sha512_1[SHA512_DIGEST_SIZE] = {
	0x87, 0xaa, 0x7c, 0xde, 0xa5, 0xef, 0x61, 0x9d, 0x4f, 0xf0, 0xb4, 0x24, 0x1a, 0x1d, 0x6c,
	0xb0, 0x23, 0x79, 0xf4, 0xe2, 0xce, 0x4e, 0xc2, 0x78, 0x7a, 0xd0, 0xb3, 0x05, 0x45, 0xe1,
	0x7c, 0xde, 0xda, 0xa8, 0x33, 0xb7, 0xd6, 0xb8, 0xa7, 0x02, 0x03, 0x8b, 0x27, 0x4e, 0xae,
	0xa3, 0xf4, 0xe4, 0xbe, 0x9d, 0x91, 0x4e, 0xeb, 0x61, 0xf1, 0x70, 0x2e, 0x69, 0x6c, 0x20,
	0x3a, 0x12, 0x68, 0x54,
};

static const uint8_t cptra_mci_hmac_test_key_2[4] = {
	0x4a, 0x65, 0x66, 0x65, /* "Jefe" */
};

/* "what do ya want for nothing?" */
static const uint8_t cptra_mci_hmac_test_data_2[28] = {
	0x77, 0x68, 0x61, 0x74, 0x20, 0x64, 0x6f, 0x20, 0x79, 0x61, 0x20, 0x77, 0x61, 0x6e, 0x74,
	0x20, 0x66, 0x6f, 0x72, 0x20, 0x6e, 0x6f, 0x74, 0x68, 0x69, 0x6e, 0x67, 0x3f,
};

static const uint8_t cptra_mci_hmac_test_sha384_2[SHA384_DIGEST_SIZE] = {
	0xaf, 0x45, 0xd2, 0xe3, 0x76, 0x48, 0x40, 0x31, 0x61, 0x7f, 0x78, 0xd2, 0xb5, 0x8a, 0x6b,
	0x1b, 0x9c, 0x7e, 0xf4, 0x64, 0xf5, 0xa0, 0x1b, 0x47, 0xe4, 0x2e, 0xc3, 0x73, 0x63, 0x22,
	0x44, 0x5e, 0x8e, 0x22, 0x40, 0xca, 0x5e, 0x69, 0xe2, 0xc7, 0x8b, 0x32, 0x39, 0xec, 0xfa,
	0xb2, 0x16, 0x49,
};

static const uint8_t cptra_mci_hmac_test_sha512_2[SHA512_DIGEST_SIZE] = {
	0x16, 0x4b, 0x7a, 0x7b, 0xfc, 0xf8, 0x19, 0xe2, 0xe3, 0x95, 0xfb, 0xe7, 0x3b, 0x56, 0xe0,
	0xa3, 0x87, 0xbd, 0x64, 0x22, 0x2e, 0x83, 0x1f, 0xd6, 0x10, 0x27, 0x0c, 0xd7, 0xea, 0x25,
	0x05, 0x54, 0x97, 0x58, 0xbf, 0x75, 0xc0, 0x5a, 0x99, 0x4a, 0x6d, 0x03, 0x4f, 0x65, 0xf8,
	0xf0, 0xe6, 0xfd, 0xca, 0xea, 0xb1, 0xa3, 0x4d, 0x4a, 0x6b, 0x4b, 0x63, 0x6e, 0x07, 0x0a,
	0x38, 0xbc, 0xe7, 0x37,
};

static int cptra_mci_hmac_test_run(const struct shell *shell, const char *label,
				   enum cptra_mci_sha_algo algo, size_t key_len,
				   const uint8_t *key, const uint8_t *data, size_t data_len,
				   const uint8_t *expected_mac, size_t expected_mac_len)
{
	uint8_t padded_key[64] = { 0 };
	uint8_t cmk[CPTRA_MCI_CMK_SIZE];
	uint8_t mac[CPTRA_MCI_HMAC_MAX_SIZE];
	size_t mac_len;
	int ret;

	memcpy(padded_key, key, key_len);

	/*
	 * The fixed Hmac import size happens to equal the digest size (48 for
	 * SHA384, 64 for SHA512), which is exactly expected_mac_len here.
	 */
	ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_HMAC, padded_key, expected_mac_len, cmk);
	if (ret) {
		shell_error(shell, "%s: MC_IMPORT failed: %d", label, ret);
		return ret;
	}

	ret = cptra_mci_hmac(cmk, algo, data, data_len, mac, sizeof(mac), &mac_len);
	if (ret) {
		shell_error(shell, "%s: MC_HMAC failed: %d", label, ret);
		return ret;
	}

	if (mac_len != expected_mac_len || memcmp(mac, expected_mac, expected_mac_len) != 0) {
		shell_error(shell, "%s: FAIL (MAC does not match known-answer vector)", label);
		return -EIO;
	}

	shell_print(shell, "%s: PASS", label);

	return 0;
}

static int cmd_cptra_mci_hmac_test(const struct shell *shell, size_t argc, char **argv)
{
	int ret;

	ret = cptra_mci_hmac_test_run(shell, "RFC4231#1/SHA384", CPTRA_MCI_SHA_ALGO_SHA384,
				      sizeof(cptra_mci_hmac_test_key_1), cptra_mci_hmac_test_key_1,
				      cptra_mci_hmac_test_data_1,
				      sizeof(cptra_mci_hmac_test_data_1),
				      cptra_mci_hmac_test_sha384_1,
				      sizeof(cptra_mci_hmac_test_sha384_1));
	if (ret)
		return ret;

	ret = cptra_mci_hmac_test_run(shell, "RFC4231#1/SHA512", CPTRA_MCI_SHA_ALGO_SHA512,
				      sizeof(cptra_mci_hmac_test_key_1), cptra_mci_hmac_test_key_1,
				      cptra_mci_hmac_test_data_1,
				      sizeof(cptra_mci_hmac_test_data_1),
				      cptra_mci_hmac_test_sha512_1,
				      sizeof(cptra_mci_hmac_test_sha512_1));
	if (ret)
		return ret;

	ret = cptra_mci_hmac_test_run(shell, "RFC4231#2/SHA384", CPTRA_MCI_SHA_ALGO_SHA384,
				      sizeof(cptra_mci_hmac_test_key_2), cptra_mci_hmac_test_key_2,
				      cptra_mci_hmac_test_data_2,
				      sizeof(cptra_mci_hmac_test_data_2),
				      cptra_mci_hmac_test_sha384_2,
				      sizeof(cptra_mci_hmac_test_sha384_2));
	if (ret)
		return ret;

	ret = cptra_mci_hmac_test_run(shell, "RFC4231#2/SHA512", CPTRA_MCI_SHA_ALGO_SHA512,
				      sizeof(cptra_mci_hmac_test_key_2), cptra_mci_hmac_test_key_2,
				      cptra_mci_hmac_test_data_2,
				      sizeof(cptra_mci_hmac_test_data_2),
				      cptra_mci_hmac_test_sha512_2,
				      sizeof(cptra_mci_hmac_test_sha512_2));
	if (ret)
		return ret;

	shell_print(shell, "MC_HMAC: PASS (all 4 RFC 4231 known-answer cases matched)");

	return 0;
}

/*
 * Caliptra-SS's key-derivation validation (validate_hkdf_params() in the RT
 * firmware) accepts exactly one key_size per (key_usage, hash_algorithm)
 * pair and rejects everything else with RUNTIME_MAILBOX_INVALID_PARAMS:
 *   Aes -> 32, Ecdsa -> 48, Mldsa -> 32, Hmac+Sha384 -> 48, Hmac+Sha512 -> 64.
 * Mlkem (and Reserved) have no valid size here at all.
 */
static uint32_t cptra_mci_kdf_key_size(enum cptra_mci_key_usage usage, enum cptra_mci_sha_algo algo)
{
	switch (usage) {
	case CPTRA_MCI_KEY_USAGE_AES:
	case CPTRA_MCI_KEY_USAGE_MLDSA:
		return 32;
	case CPTRA_MCI_KEY_USAGE_ECDSA:
		return 48;
	case CPTRA_MCI_KEY_USAGE_HMAC:
		return (algo == CPTRA_MCI_SHA_ALGO_SHA512) ? 64 : 48;
	default:
		return 0;
	}
}

/*
 * HMAC-KDF-Counter mode known-answer vector (see caliptra-mcu-sw's
 * cm_hkdf_counter_mode_test): derives an ML-DSA cmk from a fixed Hmac384
 * seed key + label. The derived Cmk is an opaque, encrypted blob, so
 * correctness is checked indirectly via the resulting key's ML-DSA-87
 * public key, captured once from real hardware.
 */
static const uint8_t cptra_mci_hkdf_counter_key_in[48] = {
	0xae, 0xb0, 0xea, 0x3a, 0x4e, 0x01, 0x3f, 0xc0, 0x87, 0x0e, 0x51, 0x5a, 0x5a, 0x94, 0x0e,
	0x30, 0xc5, 0xbc, 0xca, 0x4a, 0x2a, 0x24, 0xf0, 0xb1, 0x42, 0x07, 0xa2, 0xb4, 0x37, 0xb0,
	0x32, 0x49, 0xf6, 0xcc, 0x83, 0x31, 0xa1, 0x21, 0x92, 0xf1, 0x72, 0x66, 0x38, 0xc6, 0x01,
	0x0e, 0x82, 0xf6,
};

static const uint8_t cptra_mci_hkdf_counter_label[60] = {
	0x1c, 0xbf, 0x18, 0xdd, 0x26, 0xdd, 0x65, 0xbc, 0x18, 0xa8, 0x89, 0x92, 0xce, 0x41, 0x39,
	0xad, 0x48, 0x19, 0x55, 0xe2, 0x60, 0xa7, 0xa4, 0xe7, 0xed, 0x7d, 0xb3, 0xf0, 0x0a, 0xc0,
	0xdd, 0x72, 0xc5, 0x88, 0x71, 0x37, 0x90, 0xb4, 0xf0, 0x64, 0x45, 0xcc, 0x74, 0xa5, 0x84,
	0x8d, 0xb5, 0x19, 0xea, 0x56, 0xe7, 0xf0, 0x93, 0xb6, 0x99, 0xd2, 0x4c, 0xfa, 0x54, 0x8e,
};

static const uint8_t cptra_mci_hkdf_counter_mldsa_pk[64] = {
	0x05, 0x74, 0xa8, 0xf0, 0x63, 0x71, 0x35, 0x38, 0x59, 0xe5, 0x10, 0x3c, 0x12, 0xba, 0xa4,
	0xa5, 0x62, 0x39, 0x61, 0x37, 0x24, 0x38, 0x84, 0xde, 0xd0, 0x07, 0x4f, 0x64, 0x92, 0x60,
	0xa5, 0xa4, 0x42, 0xf2, 0x21, 0x46, 0x75, 0xe6, 0xad, 0x0e, 0x8a, 0x92, 0x80, 0x7f, 0x0d,
	0x3c, 0x4b, 0x48, 0x9a, 0x47, 0xf5, 0xcf, 0x83, 0x46, 0x98, 0xa6, 0x8d, 0x87, 0x3f, 0xf4,
	0xa6, 0xd8, 0x30, 0xaa,
};

/*
 * RFC 5869 HKDF-SHA384 Extract-and-Expand known-answer vector (see
 * caliptra-mcu-sw's cm_hkdf_extract_and_expand_test), same self-check
 * approach as the counter-mode vector above.
 */
static const uint8_t cptra_mci_hkdf_ee_salt[48] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
	0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
	0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c,
	0x2d, 0x2e, 0x2f,
};

static const uint8_t cptra_mci_hkdf_ee_ikm[48] = {
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e,
	0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
	0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c,
	0x5d, 0x5e, 0x5f,
};

static const uint8_t cptra_mci_hkdf_ee_info[10] = {
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9,
};

static const uint8_t cptra_mci_hkdf_ee_mldsa_pk[64] = {
	0x01, 0x8f, 0xad, 0x8f, 0x72, 0x41, 0x30, 0xf7, 0x7c, 0xdb, 0xd4, 0xfa, 0xe0, 0xc2, 0x21,
	0x1f, 0x18, 0xa6, 0x2c, 0xeb, 0x15, 0x85, 0xd2, 0x35, 0xef, 0x1e, 0x22, 0x8d, 0x8e, 0x68,
	0x18, 0x84,
	0x31, 0xaa, 0xe5, 0xc9, 0x60, 0x1f, 0x42, 0x66, 0x21, 0x8d, 0x13, 0xc3, 0xed, 0xb7, 0x0d,
	0x31, 0xec, 0xb5, 0xc9, 0xf3, 0x16, 0x83, 0x2b, 0x89, 0x1a, 0xc3, 0xe2, 0x39, 0x11, 0xa2,
	0xfa, 0x4b,
};

static int cmd_cptra_mci_hkdf_test(const struct shell *shell, size_t argc, char **argv)
{
	static uint8_t pubkey[CPTRA_MCI_MLDSA87_PUBKEY_SIZE];
	uint8_t kin[CPTRA_MCI_CMK_SIZE], kout[CPTRA_MCI_CMK_SIZE];
	uint8_t salt[CPTRA_MCI_CMK_SIZE], ikm[CPTRA_MCI_CMK_SIZE], prk[CPTRA_MCI_CMK_SIZE];
	uint32_t key_size;
	int ret;

	key_size = cptra_mci_kdf_key_size(CPTRA_MCI_KEY_USAGE_MLDSA, CPTRA_MCI_SHA_ALGO_SHA384);

	/* HMAC-KDF-Counter mode. */
	ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_HMAC, cptra_mci_hkdf_counter_key_in,
				   sizeof(cptra_mci_hkdf_counter_key_in), kin);
	if (ret) {
		shell_error(shell, "MC_IMPORT (kin) failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_hmac_kdf_counter(kin, CPTRA_MCI_SHA_ALGO_SHA384, CPTRA_MCI_KEY_USAGE_MLDSA,
					 key_size, cptra_mci_hkdf_counter_label,
					 sizeof(cptra_mci_hkdf_counter_label), kout);
	if (ret) {
		shell_error(shell, "MC_HMAC_KDF_COUNTER failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_mldsa_cmk_public_key(kout, pubkey);
	if (ret) {
		shell_error(shell, "MC_MLDSA_CMK_PUBLIC_KEY failed: %d", ret);
		return ret;
	}

	if (memcmp(pubkey, cptra_mci_hkdf_counter_mldsa_pk, 64) != 0) {
		shell_error(shell, "HMAC-KDF-Counter: FAIL (derived key does not match "
			    "known-answer vector)");
		return -EIO;
	}
	shell_print(shell, "HMAC-KDF-Counter: PASS");

	/* HKDF-Extract+Expand mode. */
	ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_HMAC, cptra_mci_hkdf_ee_salt,
				   sizeof(cptra_mci_hkdf_ee_salt), salt);
	if (ret) {
		shell_error(shell, "MC_IMPORT (salt) failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_HMAC, cptra_mci_hkdf_ee_ikm,
				   sizeof(cptra_mci_hkdf_ee_ikm), ikm);
	if (ret) {
		shell_error(shell, "MC_IMPORT (ikm) failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_hkdf_extract(CPTRA_MCI_SHA_ALGO_SHA384, salt, ikm, prk);
	if (ret) {
		shell_error(shell, "MC_HKDF_EXTRACT failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_hkdf_expand(prk, CPTRA_MCI_SHA_ALGO_SHA384, CPTRA_MCI_KEY_USAGE_MLDSA,
				    key_size, cptra_mci_hkdf_ee_info,
				    sizeof(cptra_mci_hkdf_ee_info), kout);
	if (ret) {
		shell_error(shell, "MC_HKDF_EXPAND failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_mldsa_cmk_public_key(kout, pubkey);
	if (ret) {
		shell_error(shell, "MC_MLDSA_CMK_PUBLIC_KEY failed: %d", ret);
		return ret;
	}

	if (memcmp(pubkey, cptra_mci_hkdf_ee_mldsa_pk, 64) != 0) {
		shell_error(shell, "HKDF-Extract+Expand: FAIL (derived key does not match "
			    "known-answer vector)");
		return -EIO;
	}
	shell_print(shell, "HKDF-Extract+Expand: PASS");

	shell_print(shell, "MC_HMAC_KDF_COUNTER/MC_HKDF_EXTRACT+EXPAND: PASS "
		    "(both known-answer cases matched)");

	return 0;
}

/*
 * ML-DSA-87 public keys/signatures (2592/4628 bytes) are far too large for
 * the shell's stack, so these stay static like the driver's own
 * request/response buffers for the same command.
 */
static uint8_t cptra_mci_mldsa_test_pubkey[CPTRA_MCI_MLDSA87_PUBKEY_SIZE];
static uint8_t cptra_mci_mldsa_test_sig[CPTRA_MCI_MLDSA87_SIGNATURE_SIZE];

/*
 * Expected ML-DSA-87 public key prefix (p || t1, the first 64 bytes) for the
 * all-zero 32-byte seed imported below (see caliptra-mcu-sw's
 * mldsa87_test()) -- captured once from real hardware, used to make the
 * pubkey step a known-answer check instead of an unverifiable printout.
 */
static const uint8_t cptra_mci_mldsa_test_expected_pubkey[64] = {
	0xe4, 0x5f, 0xfc, 0x8c, 0xc7, 0x3d, 0xb8, 0x85, 0xdc, 0x66, 0x2e, 0x62, 0xa1, 0x8c, 0xd8,
	0xe3, 0x80, 0x32, 0x97, 0x11, 0x7f, 0xa5, 0x65, 0x88, 0x14, 0xa9, 0x85, 0xb5, 0xff, 0x1d,
	0xb7, 0xb4, 0x68, 0xcf, 0xc8, 0x2b, 0xb9, 0x29, 0xf1, 0xd8, 0x6b, 0x77, 0xed, 0x14, 0xf5,
	0xae, 0x16, 0xa6, 0x53, 0x68, 0x77, 0x2c, 0xe5, 0x19, 0x12, 0x41, 0x01, 0x05, 0xe0, 0x45,
	0x69, 0x75, 0xae, 0x91,
};

static int cmd_cptra_mci_mldsa_test(const struct shell *shell, size_t argc, char **argv)
{
	/* All-zero seed: pubkey is then a known, fixed value (see the comment above). */
	static const uint8_t key[32];
	uint8_t cmk[CPTRA_MCI_CMK_SIZE];
	uint8_t message[CPTRA_MCI_ECC384_SCALAR_SIZE], bad_message[CPTRA_MCI_ECC384_SCALAR_SIZE];
	size_t message_len;
	int ret;

	ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_MLDSA, key, sizeof(key), cmk);
	if (ret) {
		shell_error(shell, "MC_IMPORT failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_mldsa_cmk_public_key(cmk, cptra_mci_mldsa_test_pubkey);
	if (ret) {
		shell_error(shell, "MC_MLDSA_CMK_PUBLIC_KEY failed: %d", ret);
		return ret;
	}

	shell_print(shell, "public_key (%u bytes):",
		    (unsigned int)sizeof(cptra_mci_mldsa_test_pubkey));
	shell_hexdump(shell, cptra_mci_mldsa_test_pubkey, sizeof(cptra_mci_mldsa_test_pubkey));

	if (memcmp(cptra_mci_mldsa_test_pubkey, cptra_mci_mldsa_test_expected_pubkey,
		   sizeof(cptra_mci_mldsa_test_expected_pubkey)) != 0) {
		shell_error(shell, "public key does not match known-answer vector");
		return -EIO;
	}
	shell_print(shell, "public key matches known-answer vector: PASS");

	if (argc > 1) {
		message_len = hex2bin(argv[1], strlen(argv[1]), message, sizeof(message));
		if (message_len == 0) {
			shell_error(shell, "bad message hex string");
			return -EINVAL;
		}
	} else {
		/* No message given: fill with pseudo-random bytes for a quick smoke test. */
		message_len = sizeof(message);

		for (size_t i = 0; i < message_len; i += sizeof(uint32_t)) {
			uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

			memcpy(&message[i], &r, MIN(sizeof(r), message_len - i));
		}

		shell_print(shell, "no message_hex given, generated random %u-byte message",
			    (unsigned int)message_len);
	}

	ret = cptra_mci_mldsa_cmk_sign(cmk, message, message_len, cptra_mci_mldsa_test_sig);
	if (ret) {
		shell_error(shell, "MC_MLDSA_CMK_SIGN failed: %d", ret);
		return ret;
	}

	/* Positive case: verify against the message that was actually signed. */
	ret = cptra_mci_mldsa_cmk_verify(cmk, cptra_mci_mldsa_test_sig, message, message_len);
	if (ret) {
		shell_error(shell, "positive case FAIL (expected PASS): %d", ret);
		return ret;
	}
	shell_print(shell, "positive case (correct message): PASS");

	/* Negative case: same signature, a tampered message -- must be rejected. */
	memcpy(bad_message, message, message_len);
	bad_message[0] ^= 0xFF;

	ret = cptra_mci_mldsa_cmk_verify(cmk, cptra_mci_mldsa_test_sig, bad_message, message_len);
	if (ret == 0) {
		shell_error(shell, "negative case FAIL (a tampered message was accepted!)");
		return -EIO;
	}
	shell_print(shell, "negative case (tampered message): correctly rejected (%d)", ret);

	shell_print(shell, "MC_MLDSA_CMK_PUBLIC_KEY/SIGN/VERIFY: PASS "
		    "(pubkey retrieved, positive and negative verify cases both OK)");

	return 0;
}

/*
 * Real LMS_SIGNATURE_VERIFY known-answer vector (message/pubkey/signature
 * pair 2/1 from caliptra-sw's runtime_integration_tests/test_lms.rs). LMS
 * keys are generated offline (e.g. for firmware signing), never on-device,
 * so unlike the Cmk-based tests above there is no way to self-generate a
 * matching (pubkey, message, signature) triple -- this borrows one already
 * known to verify successfully against real Caliptra-SS hardware. The hash
 * field below is SHA-384 of that message, computed once and pinned here
 * (the command takes a pre-hashed digest, not the raw message).
 */
static const uint8_t cptra_mci_lms_test_pubkey_id[CPTRA_MCI_LMS_PUBKEY_ID_SIZE] = {
	0x9e, 0x15, 0xd8, 0x11, 0x5f, 0x82, 0xec, 0x66, 0xc6, 0x4c, 0x9e, 0x7f, 0x10, 0x11, 0xe7,
	0xfd,
};

static const uint8_t cptra_mci_lms_test_pubkey_digest[CPTRA_MCI_LMS_PUBKEY_DIGEST_SIZE] = {
	0x4b, 0x18, 0x29, 0x4d, 0x66, 0x1d, 0xab, 0x4c, 0x40, 0x8d, 0xac, 0x0b, 0xff, 0x33, 0x89,
	0x1b, 0xcc, 0x54, 0x5e, 0xec, 0xa9, 0x53, 0x7a, 0xef,
};

#define CPTRA_MCI_LMS_TEST_SIGNATURE_Q	32153

static const uint8_t cptra_mci_lms_test_signature_ots[CPTRA_MCI_LMS_OTS_SIGNATURE_SIZE] = {
	0x00, 0x00, 0x00, 0x07, 0x59, 0x39, 0xba, 0xdc, 0xb2, 0xf4, 0x4a, 0x6a, 0xba, 0x06, 0x48,
	0x6a, 0x54, 0x26, 0x06, 0x15, 0x53, 0xf5, 0x4a, 0xf4, 0xf6, 0xfe, 0x54, 0xd2, 0xbc, 0xcb,
	0x95, 0x67, 0xc0, 0x50, 0x74, 0xa9, 0x83, 0x3e, 0xfe, 0xcd, 0x7b, 0xf0, 0x36, 0x3f, 0xb4,
	0xbd, 0x21, 0xbb, 0xad, 0x25, 0xb3, 0x2d, 0x7b, 0x23, 0x1d, 0xb9, 0xe6, 0xcc, 0x77, 0x0e,
	0xd0, 0xfc, 0x54, 0xc4, 0x38, 0x7f, 0x5a, 0x07, 0x7d, 0xf8, 0xee, 0x93, 0xf8, 0x60, 0xc7,
	0xaa, 0xfa, 0x02, 0xdd, 0x73, 0x99, 0x93, 0x19, 0x8f, 0x9d, 0xd9, 0x2b, 0xb8, 0xf1, 0x4b,
	0xad, 0x2e, 0x74, 0x18, 0x25, 0x4a, 0x3b, 0xcc, 0x04, 0x37, 0x6a, 0x59, 0x91, 0xa2, 0x55,
	0x58, 0x3d, 0x11, 0xa7, 0x8d, 0xad, 0xca, 0xd0, 0xee, 0x6b, 0x95, 0x34, 0x72, 0xa1, 0xdb,
	0x77, 0x54, 0x84, 0xd0, 0x10, 0xba, 0x31, 0xe3, 0xc4, 0xab, 0x7c, 0x61, 0xb5, 0x07, 0x3f,
	0xe8, 0x82, 0x7d, 0x65, 0xb0, 0x01, 0xd1, 0x3e, 0xc4, 0x27, 0xf2, 0x9a, 0x07, 0x3a, 0xd8,
	0x94, 0x0b, 0x1d, 0x9f, 0x4e, 0xc7, 0xd3, 0xe5, 0x72, 0x56, 0xdc, 0x50, 0x42, 0xeb, 0xa1,
	0x3a, 0x9a, 0x51, 0xa6, 0x90, 0x7e, 0x4b, 0x74, 0xda, 0xc2, 0x9e, 0x5a, 0x93, 0x1b, 0xf4,
	0x80, 0xdd, 0x6c, 0x18, 0x81, 0xac, 0xa1, 0x5e, 0x1e, 0x8b, 0x49, 0x1f, 0x56, 0x09, 0x99,
	0x00, 0xa5, 0x31, 0xcf, 0x1d, 0x6e, 0x1f, 0x79, 0x01, 0x8b, 0xb8, 0x4a, 0xc7, 0x5f, 0x3e,
	0x14, 0x36, 0x68, 0x92, 0x82, 0xf8, 0x30, 0x81, 0xfa, 0x53, 0x6e, 0x34, 0x9b, 0x61, 0xc1,
	0x47, 0x93, 0xef, 0x64, 0xa4, 0xe4, 0xf2, 0xba, 0x56, 0xfc, 0x34, 0x0f, 0xbb, 0x25, 0x84,
	0xe6, 0x08, 0xcc, 0x6a, 0xab, 0x46, 0x8b, 0x14, 0x35, 0xf1, 0x9d, 0x0d, 0xb7, 0x91, 0x49,
	0x7f, 0xfe, 0x1c, 0xc3, 0x42, 0xbf, 0x19, 0x6e, 0x68, 0xc5, 0x1d, 0x83, 0xdf, 0x46, 0xa3,
	0x92, 0x9c, 0xd6, 0xae, 0x1b, 0x55, 0x3a, 0xbf, 0x32, 0xe8, 0x90, 0xd2, 0xa8, 0xcb, 0x92,
	0xf9, 0x63, 0xc7, 0x16, 0x28, 0x52, 0x29, 0x29, 0xb8, 0xd4, 0x15, 0xa3, 0x57, 0xb5, 0x36,
	0xe9, 0xcb, 0xcd, 0x60, 0xe4, 0x14, 0xf8, 0x3e, 0x0b, 0x45, 0x0c, 0x87, 0x71, 0x5a, 0x44,
	0xc0, 0x18, 0x57, 0x32, 0x7c, 0xfb, 0xcf, 0xc5, 0xd4, 0x61, 0x73, 0x73, 0x9f, 0x90, 0x71,
	0x2e, 0x05, 0x12, 0xac, 0x04, 0x46, 0x6c, 0x4d, 0xe8, 0xc8, 0x2a, 0x1f, 0xd1, 0x93, 0x7c,
	0x81, 0x0a, 0x30, 0x4f, 0x8f, 0x9a, 0x85, 0x94, 0x4f, 0xdf, 0xd4, 0xbd, 0xa4, 0xf7, 0xbf,
	0xdc, 0x3f, 0xa9, 0xbd, 0x7b, 0x40, 0x37, 0xb9, 0xd6, 0xe7, 0xa4, 0x29, 0xae, 0x22, 0x4a,
	0x6a, 0x63, 0x15, 0x4b, 0x41, 0x74, 0x8e, 0xb7, 0x77, 0x18, 0xd9, 0x3a, 0x68, 0x89, 0x9e,
	0x14, 0xa7, 0x21, 0xa7, 0x78, 0x6a, 0x98, 0x9e, 0x73, 0x83, 0x62, 0x95, 0x6b, 0x50, 0x11,
	0xcc, 0xad, 0xfe, 0x3b, 0x18, 0xad, 0x45, 0xb5, 0x10, 0x18, 0x29, 0x3b, 0xaf, 0x2e, 0x2d,
	0xf0, 0x2d, 0x9a, 0x8a, 0x8b, 0x2d, 0x7a, 0x12, 0xd4, 0x0e, 0x94, 0xb5, 0x6f, 0x57, 0xeb,
	0xf4, 0xab, 0x96, 0x1d, 0x47, 0xb9, 0x98, 0xe4, 0xc6, 0x4d, 0x92, 0x4d, 0x75, 0x83, 0x99,
	0x98, 0x7d, 0x4b, 0x59, 0x5b, 0x94, 0xbf, 0xf3, 0x02, 0xca, 0xf6, 0xaa, 0x2a, 0x0f, 0x64,
	0x3e, 0x1a, 0x39, 0xc8, 0x3c, 0x5b, 0x50, 0xe2, 0x8f, 0xa2, 0x89, 0xf7, 0x89, 0xe8, 0xe3,
	0xf6, 0x8a, 0x2c, 0x98, 0xe0, 0xf5, 0x2c, 0xf6, 0x45, 0xa7, 0x28, 0x93, 0xe0, 0x94, 0x68,
	0x36, 0xf3, 0xfd, 0x8a, 0xa1, 0x19, 0xc1, 0xf9, 0x7a, 0xd7, 0x0f, 0x62, 0x81, 0xff, 0x65,
	0x26, 0x01, 0x07, 0x76, 0x25, 0xd9, 0x04, 0x87, 0xcf, 0x96, 0xc9, 0xf7, 0x12, 0x1f, 0x06,
	0x63, 0x52, 0xc9, 0xe5, 0x2c, 0x7c, 0xe2, 0xa6, 0xbb, 0x4e, 0x40, 0xf8, 0x23, 0x96, 0x79,
	0xc3, 0xe1, 0x15, 0x54, 0x72, 0xeb, 0xd1, 0x2e, 0xb5, 0xc7, 0x34, 0xf0, 0xa6, 0x43, 0xea,
	0x6a, 0xba, 0x04, 0x91, 0x27, 0x70, 0xae, 0xb5, 0xf2, 0x22, 0xc3, 0x33, 0x6c, 0xac, 0x1d,
	0xa0, 0xf2, 0x78, 0x83, 0x61, 0xde, 0xb1, 0xb9, 0xcc, 0x83, 0x78, 0xbc, 0x24, 0x95, 0x98,
	0x84, 0xf2, 0x54, 0x53, 0x30, 0xd5, 0x55, 0xdd, 0x07, 0xe0, 0x6a, 0x74, 0x46, 0xda, 0xfc,
	0xc7, 0x98, 0x34, 0x14, 0x3a, 0xf3, 0x78, 0xb9, 0xc6, 0xe8, 0x4a, 0xb1, 0x3d, 0xcd, 0x1d,
	0x2b, 0x5c, 0x80, 0x30, 0x74, 0x09, 0xa3, 0x96, 0x51, 0x7e, 0xbe, 0xa6, 0x7b, 0x8b, 0x64,
	0xf7, 0xf8, 0xc2, 0xd2, 0x09, 0x12, 0x6d, 0x71, 0xa7, 0x7c, 0xe9, 0x3a, 0xc9, 0x1d, 0xd9,
	0x01, 0x3d, 0xa4, 0x5b, 0x92, 0x8e, 0xd5, 0x9a, 0x1f, 0x9c, 0xd8, 0x32, 0x46, 0xaf, 0x1a,
	0xe2, 0xc1, 0xba, 0x17, 0x49, 0x21, 0xef, 0xe9, 0x43, 0x22, 0x65, 0xc5, 0x0d, 0x84, 0x5c,
	0x19, 0x4b, 0x59, 0x04, 0xd8, 0x77, 0x42, 0x27, 0xdd, 0x46, 0x6b, 0x09, 0xf7, 0xb7, 0xe5,
	0xfd, 0xe5, 0xac, 0x72, 0x9f, 0x47, 0x8c, 0x89, 0x9c, 0x58, 0x6a, 0x3f, 0xc2, 0xab, 0xa1,
	0x56, 0xc4, 0x05, 0x41, 0x2c, 0xa1, 0x17, 0xb7, 0x42, 0x3e, 0x77, 0x9d, 0x6e, 0x6c, 0xb3,
	0xd3, 0xc5, 0x44, 0xb9, 0x58, 0xdd, 0x33, 0x51, 0xf7, 0x92, 0x9d, 0x81, 0x40, 0x64, 0x4c,
	0x01, 0x53, 0x63, 0x10, 0x92, 0xf2, 0xd0, 0x08, 0x91, 0xf2, 0x84, 0x22, 0x81, 0xf6, 0x96,
	0x63, 0xea, 0xf1, 0xb2, 0x86, 0xf1, 0x96, 0x13, 0x10, 0x10, 0x5b, 0x36, 0x25, 0x46, 0x25,
	0xfc, 0xec, 0xba, 0x68, 0x37, 0x23, 0x2d, 0x71, 0x7b, 0x91, 0xd7, 0x70, 0x54, 0x78, 0x2e,
	0x97, 0x5a, 0xdc, 0x8e, 0xb8, 0xdb, 0xf7, 0x0d, 0xf8, 0x59, 0x12, 0xc0, 0xd9, 0x6f, 0x5c,
	0x2a, 0x16, 0xf4, 0x16, 0xc3, 0x82, 0xcf, 0xaa, 0xff, 0x6f, 0xdc, 0x5a, 0x46, 0x00, 0x5c,
	0x78, 0xd1, 0xfc, 0x72, 0x6f, 0xaa, 0xe9, 0x53, 0xa9, 0xf6, 0xda, 0x5f, 0x22, 0x09, 0x29,
	0x0e, 0x8a, 0xbd, 0x65, 0x16, 0xf3, 0x74, 0xa5, 0xcf, 0xa8, 0xea, 0x9f, 0x93, 0xc0, 0xcc,
	0x60, 0xf3, 0x62, 0x78, 0xb5, 0x0f, 0x2a, 0xf4, 0x6d, 0x20, 0x60, 0x59, 0x80, 0x3c, 0xcd,
	0x7e, 0xb8, 0xe1, 0x39, 0x5a, 0x2a, 0xc7, 0xb5, 0x47, 0x3e, 0xc0, 0xc7, 0xa0, 0x0a, 0x3d,
	0xb5, 0xc0, 0xdb, 0x9d, 0x40, 0x2c, 0x5f, 0xee, 0x2e, 0x34, 0xdd, 0x1b, 0x0a, 0x04, 0xd8,
	0x35, 0xf9, 0xa7, 0xc9, 0xc1, 0x00, 0xfe, 0x0e, 0x78, 0x5d, 0x4a, 0xa9, 0x69, 0x4a, 0xbb,
	0xb5, 0xe5, 0x8a, 0xb6, 0xf9, 0xe9, 0xf1, 0xcc, 0x9c, 0xd5, 0x1b, 0x30, 0x33, 0xfa, 0x4e,
	0xbd, 0xbd, 0x1b, 0x5f, 0xa1, 0x73, 0xea, 0x70, 0xfb, 0x00, 0x59, 0x67, 0x9b, 0xf3, 0x5a,
	0x33, 0x7b, 0x9b, 0xe1, 0xe5, 0xc2, 0xb8, 0xcf, 0xcc, 0xd6, 0x88, 0x5e, 0x18, 0xda, 0x02,
	0x4d, 0x1b, 0x83, 0x04, 0xac, 0x7e, 0x83, 0xfa, 0x9f, 0xde, 0x65, 0xbc, 0x9d, 0x43, 0xb3,
	0x03, 0xba, 0x03, 0xd3, 0x08, 0xad, 0x21, 0x14, 0xeb, 0x0b, 0xc3, 0x36, 0x0d, 0xba, 0x81,
	0xca, 0x29, 0x9d, 0xb5, 0xf4, 0x68, 0xd6, 0x3b, 0xec, 0xfa, 0xc3, 0xdc, 0xf2, 0x60, 0xb6,
	0xd8, 0xb9, 0xb3, 0xc6, 0x90, 0x23, 0xc7, 0x33, 0xc2, 0xa4, 0xd2, 0x04, 0x24, 0x54, 0xb2,
	0x95, 0xf1, 0xd6, 0xc0, 0x62, 0x8d, 0x11, 0x47, 0x76, 0x75, 0x5f, 0x07, 0xeb, 0x23, 0xbe,
	0xa2, 0x08, 0x8d, 0x64, 0x83, 0xca, 0x3b, 0xc2, 0x46, 0xc9, 0xdf, 0x2f, 0x06, 0x44, 0x2a,
	0x8a, 0x8c, 0xa5, 0xd9, 0xde, 0x8f, 0x3a, 0x16, 0xcb, 0x45, 0x9b, 0xfe, 0x4a, 0xb4, 0x6c,
	0xb2, 0x8a, 0x84, 0x09, 0x2b, 0x99, 0xc8, 0x63, 0xac, 0xe7, 0x85, 0xde, 0x6d, 0x01, 0x65,
	0xfe, 0x3b, 0xea, 0x06, 0xc4, 0x4e, 0x8c, 0x71, 0x7f, 0x19, 0x5a, 0x4f, 0x70, 0xb4, 0x04,
	0x22, 0xa0, 0x62, 0xad, 0x4e, 0x7f, 0xf2, 0xff, 0x76, 0x39, 0x3a, 0x6b, 0x5f, 0x56, 0x1a,
	0x41, 0x75, 0x27, 0x3e, 0xb0, 0x76, 0xe8, 0x90, 0xa7, 0xc3, 0x38, 0x79, 0xb6, 0xb0, 0xcf,
	0x88, 0xfe, 0x25, 0xd8, 0xcd, 0xea, 0x16, 0x9c, 0xb5, 0x40, 0xdd, 0x38, 0xc0, 0x21, 0x05,
	0x79, 0x91, 0xfe, 0x8e, 0x80, 0xa8, 0x5d, 0x94, 0x74, 0x5d, 0x6a, 0xe7, 0x31, 0x7b, 0x89,
	0xba, 0xa6, 0x94, 0xd3, 0x2f, 0x11, 0xd4, 0x4a, 0x21, 0x5d, 0xcb, 0x48, 0x7f, 0xab, 0x11,
	0xa1, 0x6f, 0x4b, 0x37, 0x17, 0xf1, 0x74, 0x04, 0x3a, 0x38, 0xef, 0x45, 0xe6, 0x67, 0x3c,
	0xc3, 0x28, 0x90, 0x2d, 0x22, 0x42, 0x25, 0x65, 0xe6, 0xfa, 0x5e, 0xab, 0x3e, 0xf1, 0x24,
	0xdf, 0x75, 0xc7, 0x74, 0xe8, 0x32, 0x9d, 0x28, 0x15, 0x5d, 0x7f, 0x4f, 0xfd, 0xd1, 0xa4,
	0x85, 0xad, 0x5e, 0xec, 0x8b, 0x80, 0xa5, 0x7d, 0x77, 0xd8, 0x76, 0x0a, 0x6b, 0x12, 0xe3,
	0x5e, 0x91, 0xfe, 0xc3, 0x2d, 0xb9, 0x16, 0x8d, 0xbf, 0x9f, 0xa1, 0xaf, 0x41, 0xce, 0xb4,
	0xdf, 0x9e, 0x0e, 0x7d, 0x6c, 0x63, 0x61, 0xa1, 0x63, 0x0d, 0xb8, 0xe8, 0x36, 0x73, 0x27,
	0xda, 0x46, 0x1d, 0x80, 0xe4, 0x08, 0x5c,
};

static const uint8_t cptra_mci_lms_test_signature_tree_path[CPTRA_MCI_LMS_TREE_PATH_SIZE] = {
	0x2c, 0x72, 0xd8, 0x51, 0xbf, 0x2a, 0x1e, 0xdc, 0xda, 0x01, 0xe5, 0xf7, 0x5c, 0x9c, 0xa0,
	0x2e, 0x07, 0x7f, 0xb9, 0x6b, 0x42, 0x1f, 0x9d, 0x87, 0x4a, 0xe2, 0xd3, 0x45, 0xd5, 0xe4,
	0xed, 0x1e, 0xb6, 0xc8, 0xae, 0x7f, 0xd2, 0x39, 0xd5, 0x8c, 0xff, 0xc3, 0x01, 0xb5, 0xd2,
	0x3f, 0x0d, 0x16, 0xf2, 0x0d, 0xb1, 0x13, 0xa5, 0xc9, 0xf8, 0x75, 0xe7, 0xef, 0x43, 0x89,
	0x49, 0x6d, 0x9e, 0x4f, 0xd1, 0x62, 0xca, 0xe5, 0xbf, 0xaf, 0x92, 0x81, 0x60, 0x60, 0xa8,
	0x5a, 0xac, 0xbe, 0xbc, 0xee, 0xf3, 0xbc, 0x6c, 0x87, 0xa3, 0xcd, 0x11, 0x13, 0x75, 0xbd,
	0xa5, 0xdc, 0xc6, 0xaa, 0x92, 0xa0, 0xe4, 0xf9, 0xc4, 0x00, 0xaf, 0x8f, 0x7a, 0xcd, 0xec,
	0xc4, 0x4d, 0xd4, 0xbf, 0xf1, 0x74, 0x68, 0x54, 0xcf, 0x47, 0x82, 0xaa, 0x79, 0x2a, 0xee,
	0x7e, 0x81, 0x43, 0x2b, 0xb2, 0x2e, 0x15, 0x6f, 0x76, 0x9c, 0x71, 0x7b, 0x55, 0xb1, 0x32,
	0xd1, 0xae, 0xdb, 0x90, 0x97, 0xeb, 0x0d, 0x77, 0xd9, 0xdc, 0x55, 0xbf, 0x0a, 0x7f, 0x55,
	0x1f, 0x70, 0x24, 0x1e, 0x92, 0xdc, 0x58, 0xda, 0x38, 0x0c, 0x23, 0x45, 0x8f, 0x06, 0x93,
	0xe6, 0xb5, 0xdb, 0x76, 0x38, 0x0f, 0xeb, 0xda, 0x1c, 0xbd, 0x9d, 0xd1, 0xeb, 0x4b, 0xc2,
	0x78, 0xcc, 0x8c, 0x7c, 0xaa, 0xec, 0x3b, 0x3e, 0xbd, 0x4d, 0x48, 0xcd, 0x13, 0xf3, 0x45,
	0xe8, 0xd1, 0x49, 0x83, 0x6a, 0x96, 0xe0, 0xf5, 0xaa, 0x3f, 0x25, 0xa4, 0x17, 0x93, 0x14,
	0x3f, 0x73, 0xcd, 0xd5, 0x30, 0xa4, 0x1c, 0x0a, 0x59, 0x28, 0x86, 0x06, 0x34, 0xe6, 0x3a,
	0x7e, 0xb1, 0x6c, 0x08, 0x3e, 0x6c, 0x39, 0x71, 0x77, 0x29, 0x5b, 0x13, 0x43, 0x31, 0x90,
	0x54, 0x91, 0x9b, 0x54, 0x18, 0xc2, 0xec, 0x21, 0x9c, 0xa5, 0x19, 0x7b, 0xa3, 0x80, 0x1e,
	0xbb, 0xc6, 0xf9, 0x67, 0x48, 0xfe, 0x90, 0x45, 0x9d, 0x53, 0xba, 0x33, 0x73, 0xb2, 0x29,
	0x25, 0xc6, 0x24, 0x75, 0xf5, 0x3e, 0x77, 0xf8, 0xa7, 0xfd, 0x72, 0x3e, 0x90, 0x54, 0x68,
	0xfd, 0x5c, 0x0a, 0x17, 0xce, 0x37, 0xb3, 0x1f, 0x18, 0x9b, 0x50, 0x3a, 0x6e, 0x5a, 0x2e,
	0xb0, 0xe9, 0x34, 0x0c, 0x19, 0x8d, 0x13, 0xcf, 0xfb, 0x21, 0x1f, 0xd6, 0x38, 0xaf, 0x18,
	0xe3, 0xb9, 0x55, 0x94, 0xaf, 0x5c, 0x34, 0x29, 0x6e, 0x78, 0x47, 0x05, 0x1f, 0x52, 0x32,
	0x3a, 0xb2, 0x85, 0x1e, 0xcb, 0x6f, 0x04, 0x43, 0x4c, 0x9d, 0x2e, 0x48, 0xcc, 0xf8, 0x1f,
	0xbd, 0x69, 0x63, 0xd8, 0x03, 0xdd, 0x0a, 0xd7, 0x20, 0x0c, 0x81, 0x12, 0x12, 0x04, 0x9f,
};

static const uint8_t cptra_mci_lms_test_hash[CPTRA_MCI_LMS_HASH_SIZE] = {
	0xe2, 0x20, 0xcc, 0x1d, 0x61, 0xcf, 0xd3, 0x32, 0xf9, 0xd7, 0xd2, 0x02, 0x60, 0xa5, 0xd6,
	0x39, 0x24, 0xca, 0x3c, 0xc2, 0xe8, 0x22, 0x6f, 0x4c, 0x69, 0x74, 0xad, 0x3e, 0xb4, 0x78,
	0x60, 0x17, 0x1b, 0xd2, 0xe7, 0x63, 0x33, 0x98, 0x2b, 0x0d, 0xbe, 0xe8, 0x42, 0xe1, 0x97,
	0xa4, 0x62, 0x70,
};

/*
 * ECDSA384/LMS/ML-DSA-87 signature verify: all three take a raw public key
 * over the wire, not an opaque Cmk, so unlike the cm_* tests above they are
 * not really part of the Cmk-managed "CM_*" family -- grouped into one
 * command here instead of three separate cm_*-prefixed ones.
 */
static int cmd_cptra_mci_sig_verify_test(const struct shell *shell, size_t argc, char **argv)
{
	int ret;

	/*
	 * ECDSA384: there is no on-device way to generate a matching
	 * (pubkey, digest, signature) triple for a raw-pubkey verify
	 * directly, so this reuses the Cmk-based pubkey/sign (same all-zero
	 * seed as cm_ecdsa_test, so the pubkey is already known-answer-
	 * checked there) and cross-checks that the raw-pubkey passthrough
	 * command agrees with the Cmk-based one on the same signature.
	 */
	{
		/* All-zero seed: same known-answer key as cm_ecdsa_test. */
		static const uint8_t key[CPTRA_MCI_ECC384_SCALAR_SIZE];
		const struct device *sha_dev, *ecdsa_dev;
		struct hash_ctx hash_ctx = { 0 };
		struct hash_pkt hash_pkt = { 0 };
		struct ecdsa_ctx ecdsa_ctx = { 0 };
		struct ecdsa_pkt ecdsa_pkt = { 0 };
		struct ecdsa_key ecdsa_key = { 0 };
		uint8_t cmk[CPTRA_MCI_CMK_SIZE];
		uint8_t qx[CPTRA_MCI_ECC384_SCALAR_SIZE], qy[CPTRA_MCI_ECC384_SCALAR_SIZE];
		uint8_t message[CPTRA_MCI_ECC384_SCALAR_SIZE];
		uint8_t r[CPTRA_MCI_ECC384_SCALAR_SIZE], s[CPTRA_MCI_ECC384_SCALAR_SIZE];
		uint8_t hash[SHA384_DIGEST_SIZE], bad_hash[SHA384_DIGEST_SIZE];

		ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_ECDSA, key, sizeof(key), cmk);
		if (ret) {
			shell_error(shell, "MC_IMPORT failed: %d", ret);
			return ret;
		}

		ret = cptra_mci_ecdsa_cmk_public_key(cmk, qx, qy);
		if (ret) {
			shell_error(shell, "MC_ECDSA_CMK_PUBLIC_KEY failed: %d", ret);
			return ret;
		}

		for (size_t i = 0; i < sizeof(message); i += sizeof(uint32_t)) {
			uint32_t rnd = k_cycle_get_32() ^ (uint32_t)i;

			memcpy(&message[i], &rnd, MIN(sizeof(rnd), sizeof(message) - i));
		}

		ret = cptra_mci_ecdsa_cmk_sign(cmk, message, sizeof(message), r, s);
		if (ret) {
			shell_error(shell, "MC_ECDSA_CMK_SIGN failed: %d", ret);
			return ret;
		}

		/*
		 * MC_ECDSA_CMK_SIGN/VERIFY SHA-384-hash their "message"
		 * argument internally before signing/verifying, but
		 * MC_ECDSA384_SIG_VERIFY takes an already-hashed digest with
		 * no further hashing -- so the same bytes signed above have
		 * to be hashed here before they can be handed to the
		 * raw-pubkey verify below.
		 */
		sha_dev = DEVICE_DT_GET_ANY(aspeed_cptra_mci_sha);
		if (!sha_dev) {
			shell_error(shell, "cptra_mci_sha device not found");
			return -ENODEV;
		}

		hash_ctx.flags = crypto_query_hwcaps(sha_dev);

		ret = hash_begin_session(sha_dev, &hash_ctx, CRYPTO_HASH_ALGO_SHA384);
		if (ret) {
			shell_error(shell, "hash_begin_session failed: %d", ret);
			return ret;
		}

		hash_pkt.in_buf = message;
		hash_pkt.in_len = sizeof(message);
		hash_pkt.out_buf = hash;

		ret = hash_compute(&hash_ctx, &hash_pkt);
		hash_free_session(sha_dev, &hash_ctx);
		if (ret) {
			shell_error(shell, "hash_compute failed: %d", ret);
			return ret;
		}

		/*
		 * Go through the generic crypto/ecdsa.h API (rather than
		 * calling a raw mailbox helper directly) so this test also
		 * exercises the cptra_mci_ecdsa driver itself.
		 */
		ecdsa_dev = DEVICE_DT_GET_ANY(aspeed_cptra_mci_ecdsa);
		if (!ecdsa_dev) {
			shell_error(shell, "cptra_mci_ecdsa device not found");
			return -ENODEV;
		}

		ecdsa_key.curve_id = ECC_CURVE_NIST_P384;
		ecdsa_key.qx = (char *)qx;
		ecdsa_key.qy = (char *)qy;

		ret = ecdsa_begin_session(ecdsa_dev, &ecdsa_ctx, &ecdsa_key);
		if (ret) {
			shell_error(shell, "ecdsa_begin_session failed: %d", ret);
			return ret;
		}

		ecdsa_pkt.m = hash;
		ecdsa_pkt.m_len = sizeof(hash);
		ecdsa_pkt.r = r;
		ecdsa_pkt.r_len = sizeof(r);
		ecdsa_pkt.s = s;
		ecdsa_pkt.s_len = sizeof(s);

		ret = ecdsa_verify(&ecdsa_ctx, &ecdsa_pkt);
		if (ret) {
			ecdsa_free_session(ecdsa_dev, &ecdsa_ctx);
			shell_error(shell, "ECDSA384 positive case FAIL (expected PASS): %d", ret);
			return ret;
		}

		shell_print(shell, "MC_ECDSA384_SIG_VERIFY positive case: PASS "
			    "(agrees with the Cmk-based signature)");

		memcpy(bad_hash, hash, sizeof(hash));
		bad_hash[0] ^= 0xFF;
		ecdsa_pkt.m = bad_hash;

		ret = ecdsa_verify(&ecdsa_ctx, &ecdsa_pkt);
		ecdsa_free_session(ecdsa_dev, &ecdsa_ctx);
		if (ret == 0) {
			shell_error(shell, "ECDSA384 negative case FAIL "
				    "(a tampered digest was accepted!)");
			return -EIO;
		}

		shell_print(shell, "MC_ECDSA384_SIG_VERIFY negative case: PASS "
			    "(tampered digest correctly rejected)");
	}

	/*
	 * ML-DSA-87: same reasoning as ECDSA384 above, reusing the Cmk-based
	 * pubkey/sign (same all-zero seed as cm_mldsa_test).
	 */
	{
		/*
		 * mldsa_pkt.sig (4628 bytes) and mldsa_key.key (2592 bytes)
		 * are kept off the shell thread's stack
		 * (CONFIG_SHELL_STACK_SIZE=2048), same reasoning as the LMS
		 * block below / the driver's own static req buffers.
		 */
		static struct mldsa_pkt mldsa_pkt;
		static struct mldsa_pub_key mldsa_key;
		/* All-zero seed: same known-answer key as cm_mldsa_test. */
		static const uint8_t key[32];
		const struct device *mldsa_dev;
		struct mldsa_ctx mldsa_ctx = { 0 };
		uint8_t cmk[CPTRA_MCI_CMK_SIZE];
		uint8_t message[CPTRA_MCI_ECC384_SCALAR_SIZE];
		uint8_t bad_message[CPTRA_MCI_ECC384_SCALAR_SIZE];

		ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_MLDSA, key, sizeof(key), cmk);
		if (ret) {
			shell_error(shell, "MC_IMPORT failed: %d", ret);
			return ret;
		}

		ret = cptra_mci_mldsa_cmk_public_key(cmk, cptra_mci_mldsa_test_pubkey);
		if (ret) {
			shell_error(shell, "MC_MLDSA_CMK_PUBLIC_KEY failed: %d", ret);
			return ret;
		}

		for (size_t i = 0; i < sizeof(message); i += sizeof(uint32_t)) {
			uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

			memcpy(&message[i], &r, MIN(sizeof(r), sizeof(message) - i));
		}

		ret = cptra_mci_mldsa_cmk_sign(cmk, message, sizeof(message),
					       cptra_mci_mldsa_test_sig);
		if (ret) {
			shell_error(shell, "MC_MLDSA_CMK_SIGN failed: %d", ret);
			return ret;
		}

		/*
		 * Go through the generic crypto/mldsa.h API (rather than
		 * calling a raw mailbox helper directly) so this test also
		 * exercises the cptra_mci_mldsa driver itself.
		 */
		mldsa_dev = DEVICE_DT_GET_ANY(aspeed_cptra_mci_mldsa);
		if (!mldsa_dev) {
			shell_error(shell, "cptra_mci_mldsa device not found");
			return -ENODEV;
		}

		memcpy(mldsa_key.key, cptra_mci_mldsa_test_pubkey, sizeof(mldsa_key.key));

		ret = mldsa_begin_session(mldsa_dev, &mldsa_ctx, &mldsa_key);
		if (ret) {
			shell_error(shell, "mldsa_begin_session failed: %d", ret);
			return ret;
		}

		memcpy(mldsa_pkt.sig, cptra_mci_mldsa_test_sig, sizeof(mldsa_pkt.sig));
		mldsa_pkt.m = message;
		mldsa_pkt.m_len = sizeof(message);

		ret = mldsa_verify(&mldsa_ctx, &mldsa_pkt);
		if (ret) {
			mldsa_free_session(mldsa_dev, &mldsa_ctx);
			shell_error(shell, "MLDSA87 positive case FAIL (expected PASS): %d", ret);
			return ret;
		}

		shell_print(shell, "MC_MLDSA87_SIG_VERIFY positive case: PASS "
			    "(agrees with the Cmk-based signature)");

		memcpy(bad_message, message, sizeof(message));
		bad_message[0] ^= 0xFF;
		mldsa_pkt.m = bad_message;

		ret = mldsa_verify(&mldsa_ctx, &mldsa_pkt);
		mldsa_free_session(mldsa_dev, &mldsa_ctx);
		if (ret == 0) {
			shell_error(shell, "MLDSA87 negative case FAIL "
				    "(a tampered message was accepted!)");
			return -EIO;
		}

		shell_print(shell, "MC_MLDSA87_SIG_VERIFY negative case: PASS "
			    "(tampered message correctly rejected)");
	}

	/*
	 * LMS: keys are generated offline, never on-device, so this borrows
	 * a real known-answer vector instead (see the comment on the
	 * constants above).
	 */
	{
		/*
		 * lms_pkt.sig alone is ots[1252] + tree_path[360] -- kept off
		 * the shell thread's stack (CONFIG_SHELL_STACK_SIZE=2048), same
		 * reasoning as the driver's own static req buffers.
		 */
		static struct lms_pkt lms_pkt;
		const struct device *lms_dev;
		struct lms_ctx lms_ctx = { 0 };
		struct lms_pub_key lms_key = { 0 };
		uint8_t hash[CPTRA_MCI_LMS_HASH_SIZE], bad_hash[CPTRA_MCI_LMS_HASH_SIZE];

		/*
		 * Go through the generic crypto/lms.h API (rather than
		 * calling a raw mailbox helper directly) so this test also
		 * exercises the cptra_mci_lms driver itself.
		 */
		lms_dev = DEVICE_DT_GET_ANY(aspeed_cptra_mci_lms);
		if (!lms_dev) {
			shell_error(shell, "cptra_mci_lms device not found");
			return -ENODEV;
		}

		lms_key.pub_key_tree_type = CPTRA_MCI_LMS_TREE_TYPE_FIXED;
		lms_key.pub_key_ots_type = CPTRA_MCI_LMS_OTS_TYPE_FIXED;
		memcpy(lms_key.pub_key_id, cptra_mci_lms_test_pubkey_id,
		       sizeof(lms_key.pub_key_id));
		memcpy(lms_key.pub_key_digest, cptra_mci_lms_test_pubkey_digest,
		       sizeof(lms_key.pub_key_digest));

		ret = lms_begin_session(lms_dev, &lms_ctx, &lms_key);
		if (ret) {
			shell_error(shell, "lms_begin_session failed: %d", ret);
			return ret;
		}

		lms_pkt.sig.q = CPTRA_MCI_LMS_TEST_SIGNATURE_Q;
		memcpy(lms_pkt.sig.ots, cptra_mci_lms_test_signature_ots,
		       sizeof(lms_pkt.sig.ots));
		lms_pkt.sig.tree_type = CPTRA_MCI_LMS_TREE_TYPE_FIXED;
		memcpy(lms_pkt.sig.tree_path, cptra_mci_lms_test_signature_tree_path,
		       sizeof(lms_pkt.sig.tree_path));

		memcpy(hash, cptra_mci_lms_test_hash, sizeof(hash));
		lms_pkt.m = hash;
		lms_pkt.m_len = sizeof(hash);

		ret = lms_verify(&lms_ctx, &lms_pkt);
		if (ret) {
			lms_free_session(lms_dev, &lms_ctx);
			shell_error(shell, "LMS positive case FAIL (expected PASS): %d", ret);
			return ret;
		}

		shell_print(shell, "MC_LMS_SIG_VERIFY positive case: PASS");

		memcpy(bad_hash, hash, sizeof(bad_hash));
		bad_hash[0] ^= 0xFF;
		lms_pkt.m = bad_hash;

		ret = lms_verify(&lms_ctx, &lms_pkt);
		lms_free_session(lms_dev, &lms_ctx);
		if (ret == 0) {
			shell_error(shell, "LMS negative case FAIL "
				    "(a tampered digest was accepted!)");
			return -EIO;
		}

		shell_print(shell, "MC_LMS_SIG_VERIFY negative case: PASS "
			    "(tampered digest correctly rejected)");
	}

	shell_print(shell, "sig_verify_test: PASS (ECDSA384/LMS/ML-DSA-87 raw-pubkey signature "
		    "verify all OK)");

	return 0;
}

static int cmd_cptra_mci_ecdh_test(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t alice_ctx[CPTRA_MCI_ECDH_CONTEXT_SIZE], bob_ctx[CPTRA_MCI_ECDH_CONTEXT_SIZE];
	uint8_t alice_xchg[CPTRA_MCI_ECDH_EXCHANGE_DATA_SIZE];
	uint8_t bob_xchg[CPTRA_MCI_ECDH_EXCHANGE_DATA_SIZE];
	uint8_t alice_cmk[CPTRA_MCI_CMK_SIZE], bob_cmk[CPTRA_MCI_CMK_SIZE];
	uint8_t data[CPTRA_MCI_ECC384_SCALAR_SIZE];
	uint8_t mac_a[CPTRA_MCI_HMAC_MAX_SIZE], mac_b[CPTRA_MCI_HMAC_MAX_SIZE];
	size_t mac_a_len, mac_b_len;
	int ret;

	/*
	 * There is only one mailbox to talk to, so this simulates both sides
	 * of the exchange locally: two independent MC_ECDH_GENERATE calls
	 * stand in for Alice and Bob, each MC_ECDH_FINISH consumes the
	 * OTHER side's exchange_data, and since the resulting Cmk is an
	 * opaque, encrypted blob (never comparable byte-for-byte even when
	 * it holds the same key), correctness is instead proven by HMACing
	 * the same data with both derived keys and comparing the MACs.
	 */
	ret = cptra_mci_ecdh_generate(alice_ctx, alice_xchg);
	if (ret) {
		shell_error(shell, "MC_ECDH_GENERATE (alice) failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_ecdh_generate(bob_ctx, bob_xchg);
	if (ret) {
		shell_error(shell, "MC_ECDH_GENERATE (bob) failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_ecdh_finish(alice_ctx, CPTRA_MCI_KEY_USAGE_HMAC, bob_xchg, alice_cmk);
	if (ret) {
		shell_error(shell, "MC_ECDH_FINISH (alice) failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_ecdh_finish(bob_ctx, CPTRA_MCI_KEY_USAGE_HMAC, alice_xchg, bob_cmk);
	if (ret) {
		shell_error(shell, "MC_ECDH_FINISH (bob) failed: %d", ret);
		return ret;
	}

	for (size_t i = 0; i < sizeof(data); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

		memcpy(&data[i], &r, MIN(sizeof(r), sizeof(data) - i));
	}

	ret = cptra_mci_hmac(alice_cmk, CPTRA_MCI_SHA_ALGO_SHA384, data, sizeof(data),
			     mac_a, sizeof(mac_a), &mac_a_len);
	if (ret) {
		shell_error(shell, "MC_HMAC (alice) failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_hmac(bob_cmk, CPTRA_MCI_SHA_ALGO_SHA384, data, sizeof(data),
			     mac_b, sizeof(mac_b), &mac_b_len);
	if (ret) {
		shell_error(shell, "MC_HMAC (bob) failed: %d", ret);
		return ret;
	}

	if (mac_a_len != mac_b_len || memcmp(mac_a, mac_b, mac_a_len) != 0) {
		shell_error(shell, "FAIL: alice and bob derived different shared secrets");
		return -EIO;
	}

	shell_print(shell, "MC_ECDH_GENERATE/MC_ECDH_FINISH: PASS "
		    "(alice and bob independently derived the same shared secret)");

	return 0;
}

static int cmd_cptra_mci_cm_status(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t used_usage_storage, total_usage_storage;
	int ret;

	ret = cptra_mci_cm_status(&used_usage_storage, &total_usage_storage);
	if (ret) {
		shell_error(shell, "MC_CM_STATUS failed: %d", ret);
		return ret;
	}

	shell_print(shell, "cmk storage: %u / %u used",
		    used_usage_storage, total_usage_storage);

	return 0;
}

static int cmd_cptra_mci_aes_test(const struct shell *shell, size_t argc, char **argv)
{
	/*
	 * CTR is a stream cipher (no block buffering), so each call's output
	 * length can be trusted to equal its input length -- picked as the
	 * default to keep this round trip unambiguous. CBC may buffer partial
	 * blocks internally, which this simple two-chunk test does not
	 * account for.
	 */
	enum cptra_mci_aes_mode mode = CPTRA_MCI_AES_MODE_CTR;
	uint8_t cmk[CPTRA_MCI_CMK_SIZE];
	uint8_t key[32], plain1[32], plain2[32];
	uint8_t cipher1[64], cipher2[64], decrypted1[64], decrypted2[64];
	uint8_t context[CPTRA_MCI_AES_CONTEXT_SIZE];
	uint8_t iv[CPTRA_MCI_AES_IV_SIZE];
	size_t cipher1_len, cipher2_len, plain1_out_len, plain2_out_len;
	int ret;

	if (argc > 1)
		mode = (enum cptra_mci_aes_mode)strtoul(argv[1], NULL, 0);

	shell_print(shell, "mode=%d (%s)", (int)mode,
		    mode == CPTRA_MCI_AES_MODE_CBC ? "CBC" :
		    mode == CPTRA_MCI_AES_MODE_CTR ? "CTR" : "?");

	for (size_t i = 0; i < sizeof(key); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

		memcpy(&key[i], &r, MIN(sizeof(r), sizeof(key) - i));
	}

	ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_AES, key, sizeof(key), cmk);
	if (ret) {
		shell_error(shell, "MC_IMPORT failed: %d", ret);
		return ret;
	}

	for (size_t i = 0; i < sizeof(plain1); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

		memcpy(&plain1[i], &r, MIN(sizeof(r), sizeof(plain1) - i));
	}
	for (size_t i = 0; i < sizeof(plain2); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)(i + 1);

		memcpy(&plain2[i], &r, MIN(sizeof(r), sizeof(plain2) - i));
	}

	shell_print(shell, "plain1:");
	shell_hexdump(shell, plain1, sizeof(plain1));
	shell_print(shell, "plain2:");
	shell_hexdump(shell, plain2, sizeof(plain2));

	ret = cptra_mci_aes_encrypt_init(cmk, mode, plain1, sizeof(plain1), context, iv,
					 cipher1, sizeof(cipher1), &cipher1_len);
	if (ret) {
		shell_error(shell, "MC_AES_ENCRYPT_INIT failed: %d", ret);
		return ret;
	}

	shell_print(shell, "iv (from MC_AES_ENCRYPT_INIT, %u bytes):",
		    (unsigned int)sizeof(iv));
	shell_hexdump(shell, iv, sizeof(iv));
	shell_print(shell, "cipher1 (cipher1_len=%u):", (unsigned int)cipher1_len);
	shell_hexdump(shell, cipher1, cipher1_len);

	ret = cptra_mci_aes_encrypt_update(context, plain2, sizeof(plain2),
					   cipher2, sizeof(cipher2), &cipher2_len);
	if (ret) {
		shell_error(shell, "MC_AES_ENCRYPT_UPDATE failed: %d", ret);
		return ret;
	}

	shell_print(shell, "cipher2 (cipher2_len=%u):", (unsigned int)cipher2_len);
	shell_hexdump(shell, cipher2, cipher2_len);

	ret = cptra_mci_aes_decrypt_init(cmk, mode, iv, cipher1, cipher1_len, context,
					 decrypted1, sizeof(decrypted1), &plain1_out_len);
	if (ret) {
		shell_error(shell, "MC_AES_DECRYPT_INIT failed: %d", ret);
		return ret;
	}

	shell_print(shell, "decrypted1 (plain1_out_len=%u):", (unsigned int)plain1_out_len);
	shell_hexdump(shell, decrypted1, plain1_out_len);

	ret = cptra_mci_aes_decrypt_update(context, cipher2, cipher2_len,
					   decrypted2, sizeof(decrypted2), &plain2_out_len);
	if (ret) {
		shell_error(shell, "MC_AES_DECRYPT_UPDATE failed: %d", ret);
		return ret;
	}

	shell_print(shell, "decrypted2 (plain2_out_len=%u):", (unsigned int)plain2_out_len);
	shell_hexdump(shell, decrypted2, plain2_out_len);

	if (plain1_out_len != sizeof(plain1) || memcmp(plain1, decrypted1, sizeof(plain1)) != 0) {
		shell_error(shell, "FAIL: decrypted chunk 1 does not match original plaintext");
		return -EIO;
	}
	if (plain2_out_len != sizeof(plain2) || memcmp(plain2, decrypted2, sizeof(plain2)) != 0) {
		shell_error(shell, "FAIL: decrypted chunk 2 does not match original plaintext");
		return -EIO;
	}

	shell_print(shell, "MC_AES_ENCRYPT/DECRYPT (init+update): PASS "
		    "(round-tripped %u bytes across 2 chunks)",
		    (unsigned int)(sizeof(plain1) + sizeof(plain2)));

	return 0;
}

static int cmd_cptra_mci_aes_gcm_test(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t cmk[CPTRA_MCI_CMK_SIZE];
	uint8_t key[32], aad[16], plain1[32], plain2[32];
	uint8_t cipher1[48], cipher2[48], decrypted1[48], decrypted2[48];
	uint8_t context[CPTRA_MCI_AES_GCM_CONTEXT_SIZE];
	uint8_t iv[CPTRA_MCI_AES_GCM_IV_SIZE];
	uint8_t tag[CPTRA_MCI_AES_GCM_TAG_SIZE], bad_tag[CPTRA_MCI_AES_GCM_TAG_SIZE];
	size_t cipher1_len, cipher2_len, plain1_out_len, plain2_out_len;
	int ret;

	for (size_t i = 0; i < sizeof(key); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

		memcpy(&key[i], &r, MIN(sizeof(r), sizeof(key) - i));
	}

	ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_AES, key, sizeof(key), cmk);
	if (ret) {
		shell_error(shell, "MC_IMPORT failed: %d", ret);
		return ret;
	}

	for (size_t i = 0; i < sizeof(aad); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

		memcpy(&aad[i], &r, MIN(sizeof(r), sizeof(aad) - i));
	}
	for (size_t i = 0; i < sizeof(plain1); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

		memcpy(&plain1[i], &r, MIN(sizeof(r), sizeof(plain1) - i));
	}
	for (size_t i = 0; i < sizeof(plain2); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)(i + 1);

		memcpy(&plain2[i], &r, MIN(sizeof(r), sizeof(plain2) - i));
	}

	/* Encrypt: init (AAD only, no data) -> update (chunk 1) -> final (chunk 2 + tag). */
	ret = cptra_mci_aes_gcm_encrypt_init(cmk, 0, aad, sizeof(aad), context, iv);
	if (ret) {
		shell_error(shell, "MC_AES_GCM_ENCRYPT_INIT failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_aes_gcm_encrypt_update(context, plain1, sizeof(plain1),
					       cipher1, sizeof(cipher1), &cipher1_len);
	if (ret) {
		shell_error(shell, "MC_AES_GCM_ENCRYPT_UPDATE failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_aes_gcm_encrypt_final(context, plain2, sizeof(plain2),
					      cipher2, sizeof(cipher2), &cipher2_len, tag);
	if (ret) {
		shell_error(shell, "MC_AES_GCM_ENCRYPT_FINAL failed: %d", ret);
		return ret;
	}

	/* Decrypt, positive case: same cmk/iv/aad/tag -- must succeed and round-trip. */
	ret = cptra_mci_aes_gcm_decrypt_init(cmk, 0, iv, aad, sizeof(aad), context);
	if (ret) {
		shell_error(shell, "MC_AES_GCM_DECRYPT_INIT failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_aes_gcm_decrypt_update(context, cipher1, cipher1_len,
					       decrypted1, sizeof(decrypted1), &plain1_out_len);
	if (ret) {
		shell_error(shell, "MC_AES_GCM_DECRYPT_UPDATE failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_aes_gcm_decrypt_final(context, tag, cipher2, cipher2_len,
					      decrypted2, sizeof(decrypted2), &plain2_out_len);
	if (ret) {
		shell_error(shell, "positive case FAIL (expected PASS): %d", ret);
		return ret;
	}

	if (plain1_out_len != sizeof(plain1) || memcmp(plain1, decrypted1, sizeof(plain1)) != 0) {
		shell_error(shell, "FAIL: decrypted chunk 1 does not match original plaintext");
		return -EIO;
	}
	if (plain2_out_len != sizeof(plain2) || memcmp(plain2, decrypted2, sizeof(plain2)) != 0) {
		shell_error(shell, "FAIL: decrypted chunk 2 does not match original plaintext");
		return -EIO;
	}
	shell_print(shell, "positive case (correct tag): PASS, round-tripped %u bytes",
		    (unsigned int)(sizeof(plain1) + sizeof(plain2)));

	/*
	 * Decrypt, negative case: a fresh session (context was consumed above),
	 * same cmk/iv/aad/ciphertext, but a tampered tag -- must be rejected.
	 */
	ret = cptra_mci_aes_gcm_decrypt_init(cmk, 0, iv, aad, sizeof(aad), context);
	if (ret) {
		shell_error(shell, "MC_AES_GCM_DECRYPT_INIT (negative) failed: %d", ret);
		return ret;
	}

	ret = cptra_mci_aes_gcm_decrypt_update(context, cipher1, cipher1_len,
					       decrypted1, sizeof(decrypted1), &plain1_out_len);
	if (ret) {
		shell_error(shell, "MC_AES_GCM_DECRYPT_UPDATE (negative) failed: %d", ret);
		return ret;
	}

	memcpy(bad_tag, tag, sizeof(tag));
	bad_tag[0] ^= 0xFF;

	ret = cptra_mci_aes_gcm_decrypt_final(context, bad_tag, cipher2, cipher2_len,
					      decrypted2, sizeof(decrypted2), &plain2_out_len);
	if (ret == 0) {
		shell_error(shell, "negative case FAIL (a tampered tag was accepted!)");
		return -EIO;
	}
	shell_print(shell, "negative case (tampered tag): correctly rejected (%d)", ret);

	shell_print(shell,
		    "MC_AES_GCM_ENCRYPT/DECRYPT: PASS (positive and negative cases both OK)");

	return 0;
}

static int cmd_cptra_mci_get_log(const struct shell *shell, size_t argc, char **argv)
{
	static uint8_t log[CPTRA_MCI_MAX_RESP_DATA_SIZE];
	size_t log_len;
	int ret;

	ret = cptra_mci_get_log(log, sizeof(log), &log_len);
	if (ret) {
		shell_error(shell, "MC_GET_LOG failed: %d", ret);
		return ret;
	}

	shell_print(shell, "log_len=%u", (unsigned int)log_len);
	shell_hexdump(shell, log, log_len);

	return 0;
}

static int cmd_cptra_mci_clear_log(const struct shell *shell, size_t argc, char **argv)
{
	int ret;

	ret = cptra_mci_clear_log();
	if (ret) {
		shell_error(shell, "MC_CLEAR_LOG failed: %d", ret);
		return ret;
	}

	shell_print(shell, "log cleared");

	return 0;
}

static int cmd_cptra_mci_auth_challenge(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t flags = 0;
	uint8_t challenge[CPTRA_MCI_AUTH_CHALLENGE_SIZE];
	int ret;

	if (argc > 1)
		flags = strtoul(argv[1], NULL, 0);

	ret = cptra_mci_get_auth_cmd_challenge(flags, challenge, sizeof(challenge));
	if (ret) {
		shell_error(shell, "MC_GET_AUTH_CMD_CHALLENGE failed: %d", ret);
		return ret;
	}

	shell_hexdump(shell, challenge, sizeof(challenge));

	return 0;
}

static int cmd_cptra_mci_export_csr(const struct shell *shell, size_t argc, char **argv)
{
	static uint8_t csr[CPTRA_MCI_MAX_RESP_DATA_SIZE];
	enum cptra_mci_device_key_id device_key_id = CPTRA_MCI_DEVICE_KEY_LDEVID;
	enum cptra_mci_csr_algo algo = CPTRA_MCI_CSR_ALGO_ECC384;
	uint8_t nonce[CPTRA_MCI_CSR_NONCE_SIZE];
	size_t csr_len;
	int ret;

	if (argc > 1)
		device_key_id = (enum cptra_mci_device_key_id)strtoul(argv[1], NULL, 0);
	if (argc > 2)
		algo = (enum cptra_mci_csr_algo)strtoul(argv[2], NULL, 0);

	/*
	 * Freshness for this nonce only matters to a real attestation flow, not
	 * to exercising the command from the shell, so avoid pulling in a full
	 * entropy backend just for this test helper.
	 */
	for (size_t i = 0; i < sizeof(nonce); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

		memcpy(&nonce[i], &r, MIN(sizeof(r), sizeof(nonce) - i));
	}

	ret = cptra_mci_export_attested_csr(device_key_id, algo, nonce, csr, sizeof(csr),
					    &csr_len);
	if (ret) {
		shell_error(shell, "MC_EXPORT_ATTESTED_CSR failed: %d", ret);
		return ret;
	}

	shell_print(shell, "csr_len=%u", (unsigned int)csr_len);
	shell_hexdump(shell, csr, csr_len);

	return 0;
}

static int cmd_cptra_mci_random_stir(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t input[CPTRA_MCI_ECC384_SCALAR_SIZE];
	size_t input_len;
	int ret;

	if (argc > 1) {
		input_len = hex2bin(argv[1], strlen(argv[1]), input, sizeof(input));
		if (input_len == 0) {
			shell_error(shell, "bad input hex string");
			return -EINVAL;
		}
	} else {
		/* No input given: fill with pseudo-random bytes for a quick smoke test. */
		input_len = sizeof(input);

		for (size_t i = 0; i < input_len; i += sizeof(uint32_t)) {
			uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

			memcpy(&input[i], &r, MIN(sizeof(r), input_len - i));
		}

		shell_print(shell, "no input_hex given, generated random %u-byte input",
			    (unsigned int)input_len);
	}

	ret = cptra_mci_random_stir(input, input_len);
	if (ret) {
		shell_error(shell, "MC_RANDOM_STIR failed: %d", ret);
		return ret;
	}

	shell_print(shell, "MC_RANDOM_STIR: OK");

	return 0;
}

static int cmd_cptra_mci_random_generate(const struct shell *shell, size_t argc, char **argv)
{
	static uint8_t data[CPTRA_MCI_MBOX_MAX_INPUT_SIZE];
	unsigned long len = 32;
	size_t data_len;
	int ret;

	if (argc > 1)
		len = strtoul(argv[1], NULL, 0);

	if (len == 0 || len > sizeof(data)) {
		shell_error(shell, "size must be 1..%u", (unsigned int)sizeof(data));
		return -EINVAL;
	}

	ret = cptra_mci_random_generate(data, len, &data_len);
	if (ret) {
		shell_error(shell, "MC_RANDOM_GENERATE failed: %d", ret);
		return ret;
	}

	shell_print(shell, "data_len=%u", (unsigned int)data_len);
	shell_hexdump(shell, data, data_len);

	return 0;
}

/*
 * TRNG statistical sanity test (see caliptra-mcu-sw's cm_trng_test): stirs a
 * fixed seed, then generates a run of fixed-size chunks and checks that no
 * chunk repeats another chunk within a trailing window -- not a full NIST
 * randomness test suite, but enough to catch a stuck/degenerate CSRNG.
 */
#define CPTRA_MCI_TRNG_TEST_CHUNK_SIZE	64
#define CPTRA_MCI_TRNG_TEST_WINDOW	10
#define CPTRA_MCI_TRNG_TEST_ITERATIONS	100

static int cmd_cptra_mci_trng_test(const struct shell *shell, size_t argc, char **argv)
{
	static uint8_t window[CPTRA_MCI_TRNG_TEST_CHUNK_SIZE * CPTRA_MCI_TRNG_TEST_WINDOW];
	uint8_t seed[32];
	size_t out_len;
	int ret;

	for (size_t i = 0; i < sizeof(seed); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

		memcpy(&seed[i], &r, MIN(sizeof(r), sizeof(seed) - i));
	}

	ret = cptra_mci_random_stir(seed, sizeof(seed));
	if (ret) {
		shell_error(shell, "MC_RANDOM_STIR failed: %d", ret);
		return ret;
	}

	for (size_t i = 0; i < CPTRA_MCI_TRNG_TEST_ITERATIONS; i++) {
		size_t offset = (i % CPTRA_MCI_TRNG_TEST_WINDOW) * CPTRA_MCI_TRNG_TEST_CHUNK_SIZE;

		ret = cptra_mci_random_generate(&window[offset], CPTRA_MCI_TRNG_TEST_CHUNK_SIZE,
						&out_len);
		if (ret) {
			shell_error(shell, "MC_RANDOM_GENERATE failed at iteration %u: %d",
				    (unsigned int)i, ret);
			return ret;
		}

		if (i < CPTRA_MCI_TRNG_TEST_WINDOW - 1)
			continue; /* let the TRNG warm up before checking for repeats */

		for (size_t j = 0; j < CPTRA_MCI_TRNG_TEST_WINDOW - 1; j++) {
			size_t cmp_offset = ((i - j - 1) % CPTRA_MCI_TRNG_TEST_WINDOW) *
					    CPTRA_MCI_TRNG_TEST_CHUNK_SIZE;

			if (memcmp(&window[offset], &window[cmp_offset],
				   CPTRA_MCI_TRNG_TEST_CHUNK_SIZE) == 0) {
				shell_error(shell,
					    "FAIL: random output at iteration %u repeats "
					    "output from iteration %u",
					    (unsigned int)i, (unsigned int)(i - j - 1));
				return -EIO;
			}
		}
	}

	shell_print(shell, "MC_RANDOM_STIR/MC_RANDOM_GENERATE: PASS "
		    "(%u chunks of %u bytes, no repeats within a %u-chunk window)",
		    (unsigned int)CPTRA_MCI_TRNG_TEST_ITERATIONS,
		    (unsigned int)CPTRA_MCI_TRNG_TEST_CHUNK_SIZE,
		    (unsigned int)CPTRA_MCI_TRNG_TEST_WINDOW);

	return 0;
}

/*
 * NIST CAVS SHA-384/SHA-512 short and long message known-answer vectors
 * (see caliptra-mcu-sw's sha384_test()/sha512_test()).
 */
static const uint8_t cptra_mci_sha384_msg_1[128] = {
	0x3b, 0xf5, 0x2c, 0xc5, 0xee, 0x86, 0xb9, 0xa0, 0x19, 0x0f, 0x39, 0x0a, 0x5c, 0x03, 0x66,
	0xa5, 0x60, 0xb5, 0x57, 0x00, 0x0d, 0xbe, 0x51, 0x15, 0xfd, 0x9e, 0xe1, 0x16, 0x30, 0xa6,
	0x27, 0x69, 0x01, 0x15, 0x75, 0xf1, 0x58, 0x81, 0x19, 0x8f, 0x22, 0x78, 0x76, 0xe8, 0xfe,
	0x68, 0x5a, 0x69, 0x39, 0xbc, 0x8b, 0x89, 0xfd, 0x48, 0xa3, 0x4e, 0xc5, 0xe7, 0x1e, 0x13,
	0x14, 0x62, 0xb2, 0x88, 0x67, 0x94, 0xdf, 0xfa, 0x68, 0xcc, 0xc6, 0xd5, 0x64, 0x73, 0x3e,
	0x67, 0xff, 0xef, 0x25, 0xe6, 0x27, 0xc6, 0xf4, 0xb5, 0x46, 0x07, 0x96, 0xe3, 0xbc, 0xe6,
	0x7b, 0xf5, 0x8c, 0xa6, 0xe8, 0xe5, 0x55, 0xbc, 0x91, 0x6a, 0x85, 0x31, 0x69, 0x7a, 0xc9,
	0x48, 0xb9, 0x0d, 0xc8, 0x61, 0x6f, 0x25, 0x10, 0x1d, 0xb9, 0x0b, 0x50, 0xc3, 0xd3, 0xdb,
	0xc9, 0xe2, 0x1e, 0x42, 0xff, 0x38, 0x71, 0x87,
};

static const uint8_t cptra_mci_sha384_md_1[SHA384_DIGEST_SIZE] = {
	0x12, 0xb6, 0xcb, 0x35, 0xed, 0xa9, 0x2e, 0xe3, 0x73, 0x56, 0xdd, 0xee, 0x77, 0x78, 0x1a,
	0x17, 0xb3, 0xd9, 0x0e, 0x56, 0x38, 0x24, 0xa9, 0x84, 0xfa, 0xff, 0xc6, 0xfd, 0xd1, 0x69,
	0x3b, 0xd7, 0x62, 0x60, 0x39, 0x63, 0x55, 0x63, 0xcf, 0xc3, 0xb9, 0xa2, 0xb0, 0x0f, 0x9c,
	0x65, 0xee, 0xfd,
};

static const uint8_t cptra_mci_sha384_msg_2[524] = {
	0xb4, 0x1e, 0xf4, 0xa2, 0xb3, 0x1d, 0x3a, 0x47, 0xa5, 0xe8, 0x38, 0x69, 0x25, 0xfb, 0x90,
	0xfb, 0xe9, 0x80, 0xca, 0x2c, 0xf6, 0xba, 0x34, 0xe8, 0x14, 0xa3, 0xec, 0xa2, 0x80, 0xd4,
	0x3b, 0x51, 0xe2, 0xa6, 0x24, 0xb8, 0x7e, 0x97, 0x3d, 0xeb, 0xae, 0xef, 0xbe, 0x0f, 0x90,
	0xa3, 0xd8, 0x61, 0xfc, 0x79, 0x48, 0x7a, 0xb9, 0x6d, 0x2b, 0xd1, 0xf0, 0x53, 0x14, 0x81,
	0xe0, 0xbf, 0x5c, 0x4c, 0xd4, 0x22, 0xb9, 0xe0, 0x52, 0x35, 0x12, 0x5a, 0x96, 0x9b, 0x92,
	0x09, 0x3f, 0xc2, 0xfc, 0x74, 0x47, 0x0a, 0x3e, 0x9e, 0x38, 0x29, 0x48, 0x88, 0x9d, 0xce,
	0x1e, 0xf8, 0xb7, 0x84, 0x2c, 0x79, 0x72, 0xd9, 0xdb, 0xa3, 0x97, 0xc0, 0x61, 0x9c, 0x43,
	0x35, 0xdd, 0xdf, 0x98, 0x1f, 0xa3, 0x3c, 0xb2, 0x7a, 0xd4, 0x20, 0xdd, 0xb5, 0x33, 0x17,
	0x05, 0x59, 0x33, 0x8f, 0x6e, 0xec, 0x55, 0xdf, 0xa5, 0x15, 0xb0, 0x88, 0xb6, 0x01, 0xd1,
	0xba, 0x9f, 0x04, 0xc6, 0x1a, 0x05, 0x84, 0x76, 0xc2, 0x26, 0x13, 0x34, 0x89, 0x0f, 0xdb,
	0x93, 0x85, 0x0e, 0x3f, 0x7b, 0xcf, 0x6b, 0xbf, 0x7c, 0x86, 0x53, 0x1c, 0x63, 0x17, 0x15,
	0xbd, 0x0b, 0xbb, 0x91, 0x52, 0x76, 0x41, 0xec, 0xb7, 0x1c, 0x0d, 0x89, 0x1b, 0x83, 0x61,
	0x35, 0x61, 0x10, 0xdb, 0xa2, 0xc6, 0x1d, 0x3f, 0x17, 0xc5, 0x4c, 0x77, 0x65, 0x38, 0x1a,
	0xd8, 0x1d, 0xbf, 0xb7, 0xa7, 0x00, 0x65, 0x15, 0xdc, 0xb7, 0x81, 0x3e, 0x25, 0xc5, 0xdb,
	0xec, 0x2e, 0xb9, 0x10, 0x53, 0x20, 0xa8, 0x38, 0x42, 0x68, 0xda, 0xc5, 0x6a, 0xb9, 0xc4,
	0xae, 0xfd, 0x54, 0x36, 0xce, 0x7c, 0x87, 0xd6, 0x87, 0xca, 0x73, 0xcd, 0xaa, 0x84, 0x13,
	0xc0, 0x9b, 0x95, 0x7a, 0x02, 0xb0, 0x04, 0x5c, 0x08, 0xdb, 0x11, 0xd0, 0x1d, 0x89, 0x81,
	0x54, 0x56, 0xc9, 0x36, 0x12, 0xc9, 0xb1, 0x02, 0x62, 0xfa, 0xfd, 0xbd, 0x8f, 0x96, 0xfa,
	0x95, 0xde, 0x64, 0xa3, 0xa6, 0xcc, 0x78, 0x09, 0xcb, 0x98, 0xaa, 0xf8, 0xb3, 0x01, 0x46,
	0xd7, 0x37, 0x51, 0x05, 0xd0, 0xea, 0x9a, 0x3c, 0x31, 0xf2, 0x40, 0x54, 0x12, 0x17, 0xb7,
	0x7a, 0x39, 0x1a, 0x8e, 0x97, 0x3a, 0x48, 0xad, 0x83, 0xe2, 0x4e, 0xdb, 0x76, 0xd6, 0x4f,
	0x83, 0xbf, 0x78, 0x34, 0xb7, 0xb8, 0x0a, 0x44, 0x75, 0xf9, 0x16, 0xf4, 0x29, 0xcc, 0x0d,
	0x00, 0x74, 0xe1, 0x2e, 0xba, 0x2c, 0x43, 0xa9, 0xa2, 0xa4, 0x8b, 0x05, 0x38, 0x2d, 0x54,
	0xbf, 0x0e, 0xdc, 0xee, 0x34, 0x21, 0x8e, 0x57, 0xc0, 0x69, 0x2a, 0x66, 0x28, 0xaf, 0x26,
	0x4a, 0x35, 0x9f, 0x4d, 0x33, 0xa2, 0x1e, 0xbf, 0x4f, 0x39, 0xde, 0x39, 0xf8, 0x38, 0x2f,
	0x96, 0x2c, 0xf9, 0x18, 0x96, 0x3d, 0xd2, 0xa6, 0xdf, 0xc9, 0x3b, 0x84, 0x41, 0xed, 0x2e,
	0xcc, 0x3e, 0xb6, 0xd4, 0x1d, 0xca, 0xd8, 0x5d, 0xee, 0x4f, 0xa3, 0x2f, 0x21, 0xf4, 0x32,
	0x44, 0xdb, 0x1f, 0xe6, 0xcd, 0x94, 0x38, 0x13, 0x1c, 0x6a, 0xb1, 0xf8, 0xc0, 0xbb, 0x43,
	0xf5, 0xf2, 0xf2, 0x7a, 0xf8, 0x30, 0xa1, 0x73, 0x39, 0x27, 0xd1, 0xe5, 0x27, 0xd6, 0x96,
	0x5e, 0xa1, 0xf1, 0xe6, 0xcf, 0x6a, 0x83, 0xaa, 0x4b, 0xd7, 0xd8, 0x16, 0x0c, 0xcb, 0x9a,
	0x36, 0xb7, 0xe5, 0x3a, 0xca, 0xe0, 0xf8, 0x91, 0x54, 0xd1, 0xbc, 0xc8, 0x6f, 0x87, 0xc3,
	0x75, 0x62, 0xfe, 0xae, 0x06, 0xf5, 0xaf, 0xa0, 0x63, 0x84, 0x56, 0xf3, 0xcf, 0x51, 0xf8,
	0x2a, 0x0f, 0x1e, 0x25, 0x27, 0xcf, 0xfc, 0x7b, 0x9f, 0x41, 0x5e, 0x6b, 0xca, 0x07, 0x71,
	0x40, 0x5f, 0x53, 0xb0, 0x5c, 0xbf, 0x4d, 0xb3, 0xc5, 0x70, 0xb5, 0x47, 0xe1, 0xce, 0xc9,
	0xc4, 0x7e, 0xfd, 0x69, 0x9b, 0x15, 0x22, 0x79, 0x2f, 0x50, 0xd3, 0x87, 0xe6, 0xc9,
};

static const uint8_t cptra_mci_sha384_md_2[SHA384_DIGEST_SIZE] = {
	0x0f, 0x8f, 0xf0, 0xee, 0xe8, 0xff, 0xde, 0xfb, 0x8b, 0x31, 0x51, 0xb7, 0x66, 0x4c, 0xe9,
	0xdc, 0xaa, 0xaf, 0x3f, 0x7a, 0xf0, 0xb1, 0xe2, 0x90, 0xee, 0x49, 0xba, 0x21, 0x1b, 0x67,
	0x86, 0x3f, 0x63, 0xfa, 0x71, 0x20, 0x2a, 0x53, 0x4c, 0x0c, 0x42, 0x44, 0x4a, 0x52, 0xbe,
	0xbf, 0x6c, 0x62,
};

static const uint8_t cptra_mci_sha512_msg_1[128] = {
	0xfd, 0x22, 0x03, 0xe4, 0x67, 0x57, 0x4e, 0x83, 0x4a, 0xb0, 0x7c, 0x90, 0x97, 0xae, 0x16,
	0x45, 0x32, 0xf2, 0x4b, 0xe1, 0xeb, 0x5d, 0x88, 0xf1, 0xaf, 0x77, 0x48, 0xce, 0xff, 0x0d,
	0x2c, 0x67, 0xa2, 0x1f, 0x4e, 0x40, 0x97, 0xf9, 0xd3, 0xbb, 0x4e, 0x9f, 0xbf, 0x97, 0x18,
	0x6e, 0x0d, 0xb6, 0xdb, 0x01, 0x00, 0x23, 0x0a, 0x52, 0xb4, 0x53, 0xd4, 0x21, 0xf8, 0xab,
	0x9c, 0x9a, 0x60, 0x43, 0xaa, 0x32, 0x95, 0xea, 0x20, 0xd2, 0xf0, 0x6a, 0x2f, 0x37, 0x47,
	0x0d, 0x8a, 0x99, 0x07, 0x5f, 0x1b, 0x8a, 0x83, 0x36, 0xf6, 0x22, 0x8c, 0xf0, 0x8b, 0x59,
	0x42, 0xfc, 0x1f, 0xb4, 0x29, 0x9c, 0x7d, 0x24, 0x80, 0xe8, 0xe8, 0x2b, 0xce, 0x17, 0x55,
	0x40, 0xbd, 0xfa, 0xd7, 0x75, 0x2b, 0xc9, 0x5b, 0x57, 0x7f, 0x22, 0x95, 0x15, 0x39, 0x4f,
	0x3a, 0xe5, 0xce, 0xc8, 0x70, 0xa4, 0xb2, 0xf8,
};

static const uint8_t cptra_mci_sha512_md_1[SHA512_DIGEST_SIZE] = {
	0xa2, 0x1b, 0x10, 0x77, 0xd5, 0x2b, 0x27, 0xac, 0x54, 0x5a, 0xf6, 0x3b, 0x32, 0x74, 0x6c,
	0x6e, 0x3c, 0x51, 0xcb, 0x0c, 0xb9, 0xf2, 0x81, 0xeb, 0x9f, 0x35, 0x80, 0xa6, 0xd4, 0x99,
	0x6d, 0x5c, 0x99, 0x17, 0xd2, 0xa6, 0xe4, 0x84, 0x62, 0x7a, 0x9d, 0x5a, 0x06, 0xfa, 0x1b,
	0x25, 0x32, 0x7a, 0x9d, 0x71, 0x0e, 0x02, 0x73, 0x87, 0xfc, 0x3e, 0x07, 0xd7, 0xc4, 0xd1,
	0x4c, 0x60, 0x86, 0xcc,
};

static const uint8_t cptra_mci_sha512_msg_2[524] = {
	0x5e, 0x0e, 0x84, 0x41, 0x9c, 0x02, 0xdd, 0xda, 0x28, 0x9a, 0xa1, 0x26, 0xbd, 0xb4, 0x0a,
	0x06, 0x04, 0x64, 0xaa, 0x58, 0xb8, 0x8b, 0xad, 0x27, 0x08, 0xab, 0x5f, 0x1e, 0x3d, 0xf9,
	0xee, 0x43, 0x9c, 0xb4, 0x70, 0xe2, 0x8e, 0xb6, 0x27, 0xc6, 0xfe, 0x49, 0x04, 0xaf, 0x03,
	0x3b, 0x6b, 0x01, 0xbf, 0x35, 0x36, 0xba, 0x87, 0x48, 0xfc, 0xa6, 0x43, 0xc9, 0x93, 0xd6,
	0x18, 0x5f, 0xd3, 0x4e, 0x45, 0x5a, 0x9d, 0xfe, 0x4b, 0x46, 0x1c, 0xf4, 0x51, 0xc0, 0x4b,
	0xcf, 0xc6, 0x89, 0xb8, 0x77, 0x48, 0xd9, 0x87, 0x0b, 0xc5, 0xf6, 0xb9, 0x1b, 0xe0, 0x04,
	0xaf, 0x18, 0x96, 0x1f, 0xe9, 0x08, 0x21, 0xa7, 0x14, 0x7e, 0x1c, 0xdb, 0x44, 0xca, 0xe9,
	0xaa, 0x7e, 0x6d, 0x50, 0xc5, 0x79, 0xd0, 0x68, 0xf9, 0xa5, 0x35, 0xbb, 0xbc, 0x6d, 0xed,
	0xa5, 0x06, 0xb9, 0xcb, 0xfd, 0x62, 0xb8, 0xda, 0xf7, 0x44, 0xdc, 0x4b, 0x49, 0x9d, 0x26,
	0xb1, 0x8d, 0xad, 0xa3, 0x71, 0xe7, 0x18, 0x37, 0x73, 0xe7, 0x3d, 0x99, 0x1e, 0xb1, 0xc3,
	0x9f, 0x84, 0x5b, 0x74, 0xbc, 0xd9, 0x96, 0x4f, 0xc7, 0x2a, 0x91, 0xd8, 0xfd, 0x4b, 0x1a,
	0xb3, 0x4a, 0x12, 0x07, 0x71, 0xc4, 0xc2, 0xd4, 0xaa, 0x78, 0xca, 0x8d, 0x4c, 0x6a, 0xb0,
	0xee, 0x32, 0xd7, 0x48, 0xca, 0xf9, 0xbd, 0x29, 0xa9, 0x0f, 0x9e, 0x61, 0xb5, 0x0c, 0x80,
	0x68, 0xd7, 0x46, 0x38, 0x53, 0x1d, 0x9f, 0xe8, 0x4a, 0x5f, 0xa2, 0xc7, 0x3c, 0x22, 0xcf,
	0x20, 0xd1, 0xbc, 0x32, 0x9e, 0xa1, 0xb9, 0x3b, 0xc6, 0xa3, 0x7e, 0xc9, 0xc5, 0xe8, 0x2c,
	0x88, 0x6c, 0x89, 0xc7, 0x7d, 0x79, 0xde, 0x98, 0xdf, 0x18, 0xf0, 0xcf, 0x29, 0xa9, 0x31,
	0x6d, 0x6d, 0xc4, 0x6b, 0x61, 0xeb, 0x7a, 0xf7, 0xf1, 0xe2, 0xde, 0x2f, 0x5c, 0xa6, 0xc5,
	0x25, 0xbe, 0xf3, 0xc9, 0x96, 0x33, 0x81, 0x94, 0x19, 0x3f, 0xd8, 0x5b, 0x9c, 0x6e, 0x66,
	0xa8, 0x11, 0x37, 0xcf, 0x5d, 0x65, 0x26, 0x84, 0xf6, 0xb2, 0x3b, 0x97, 0x0e, 0xb5, 0x8d,
	0xce, 0x24, 0x82, 0x32, 0xf6, 0xa0, 0x76, 0x63, 0x79, 0x11, 0x6e, 0xdc, 0x33, 0xb9, 0x36,
	0x82, 0x82, 0x4b, 0x45, 0x48, 0x9c, 0xf3, 0xa7, 0x53, 0x26, 0x97, 0x3a, 0x5d, 0x02, 0x5d,
	0x1d, 0x57, 0x68, 0x6d, 0x66, 0x52, 0x38, 0xf8, 0x13, 0x9f, 0x8e, 0x79, 0x44, 0x35, 0x97,
	0x3a, 0xa7, 0x54, 0xa4, 0x1a, 0x33, 0x68, 0x7d, 0x8f, 0x19, 0x30, 0xf8, 0xe7, 0xf7, 0x1f,
	0xca, 0xd3, 0x4f, 0x03, 0x9c, 0x25, 0x39, 0xe4, 0x62, 0xd9, 0x54, 0x2f, 0xe8, 0x52, 0x02,
	0x14, 0xad, 0xe7, 0xd5, 0x2c, 0xf5, 0x9e, 0x44, 0x5f, 0x1a, 0x37, 0xa3, 0x05, 0x23, 0x6d,
	0x52, 0x4b, 0x97, 0x78, 0x34, 0xd8, 0x94, 0xd6, 0x62, 0xb1, 0x1b, 0x4b, 0x21, 0x5a, 0xae,
	0x27, 0x12, 0x3f, 0x1d, 0xc7, 0xa9, 0x52, 0x9d, 0x5e, 0xe0, 0xf4, 0x1a, 0xf6, 0x2d, 0x19,
	0xce, 0xb1, 0xb1, 0xa7, 0x13, 0x55, 0xb2, 0x20, 0x75, 0x07, 0x4b, 0x81, 0x0c, 0x57, 0x62,
	0x6a, 0x09, 0x7d, 0xdb, 0x7e, 0x8e, 0xe2, 0x0b, 0x0d, 0xc3, 0xe3, 0x70, 0xf1, 0x26, 0xd1,
	0x9f, 0xb5, 0x22, 0x3c, 0x7d, 0xfd, 0xe4, 0x71, 0xb2, 0x21, 0x6a, 0x41, 0x5b, 0x1c, 0xdc,
	0x04, 0xff, 0xa5, 0x20, 0xde, 0xc5, 0x9b, 0xa9, 0x41, 0xe4, 0xa6, 0x8f, 0xb3, 0x5a, 0x1c,
	0xe5, 0xd2, 0xe1, 0x21, 0x47, 0x95, 0x1f, 0x13, 0xb2, 0xd4, 0x1d, 0x25, 0x98, 0x0b, 0xfa,
	0x49, 0x34, 0x5b, 0xe1, 0x54, 0x1c, 0xd1, 0x38, 0x5c, 0x15, 0xeb, 0x1a, 0x65, 0x2a, 0x58,
	0x08, 0x3d, 0x00, 0x00, 0xdb, 0xa5, 0x64, 0x44, 0xfd, 0xb1, 0x26, 0x87, 0xd9, 0x72, 0x2a,
	0xed, 0x90, 0xd2, 0xbc, 0x62, 0xee, 0xbe, 0x24, 0xdf, 0x9c, 0x8b, 0x7a, 0xe8, 0x97,
};

static const uint8_t cptra_mci_sha512_md_2[SHA512_DIGEST_SIZE] = {
	0xd4, 0xa3, 0xb3, 0x8f, 0x18, 0xb0, 0x48, 0xe6, 0x16, 0x86, 0xc2, 0x15, 0x9c, 0x66, 0x64,
	0x69, 0xce, 0x9a, 0x94, 0x00, 0x63, 0x2f, 0xd4, 0x0a, 0xc1, 0xd6, 0xed, 0x33, 0x16, 0x2b,
	0x92, 0x1b, 0x95, 0xac, 0x26, 0x23, 0x8a, 0xf9, 0xfa, 0x00, 0xa4, 0x31, 0x4a, 0x60, 0x46,
	0xe1, 0x73, 0x3f, 0xd4, 0xfa, 0xdc, 0x60, 0x33, 0xf3, 0xac, 0x33, 0xd2, 0xb8, 0xc2, 0x02,
	0xaf, 0xa5, 0xf0, 0x2f,
};

static int cptra_mci_sha_test_run(const struct shell *shell, const struct device *dev,
				  const char *label, enum hash_algo algo,
				  const uint8_t *msg, size_t msg_len,
				  const uint8_t *expected_md, size_t md_len)
{
	struct hash_ctx ctx = { 0 };
	struct hash_pkt pkt = { 0 };
	uint8_t digest[SHA512_DIGEST_SIZE];
	int ret;

	/*
	 * hash_ctx.flags is documented as "to be populated by the app before
	 * calling hash_begin_session()" -- hash_begin_session() asserts on it
	 * before the driver ever sees ctx, so an uninitialized ctx.flags here
	 * would assert against stack garbage.
	 */
	ctx.flags = crypto_query_hwcaps(dev);

	ret = hash_begin_session(dev, &ctx, algo);
	if (ret) {
		shell_error(shell, "%s: hash_begin_session failed: %d", label, ret);
		return ret;
	}

	pkt.in_buf = (uint8_t *)msg;
	pkt.in_len = msg_len;
	pkt.out_buf = digest;

	ret = hash_compute(&ctx, &pkt);
	hash_free_session(dev, &ctx);
	if (ret) {
		shell_error(shell, "%s: hash_compute failed: %d", label, ret);
		return ret;
	}

	if (memcmp(digest, expected_md, md_len) != 0) {
		shell_error(shell, "%s: FAIL (digest does not match known-answer vector)", label);
		return -EIO;
	}

	shell_print(shell, "%s: PASS", label);

	return 0;
}

static int cmd_cptra_mci_sha_test(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	dev = DEVICE_DT_GET_ANY(aspeed_cptra_mci_sha);
	if (!dev) {
		shell_error(shell, "cptra_mci_sha device not found");
		return -ENODEV;
	}

	ret = cptra_mci_sha_test_run(shell, dev, "SHA384 short KAT", CRYPTO_HASH_ALGO_SHA384,
				     cptra_mci_sha384_msg_1, sizeof(cptra_mci_sha384_msg_1),
				     cptra_mci_sha384_md_1, sizeof(cptra_mci_sha384_md_1));
	if (ret)
		return ret;

	ret = cptra_mci_sha_test_run(shell, dev, "SHA384 long KAT", CRYPTO_HASH_ALGO_SHA384,
				     cptra_mci_sha384_msg_2, sizeof(cptra_mci_sha384_msg_2),
				     cptra_mci_sha384_md_2, sizeof(cptra_mci_sha384_md_2));
	if (ret)
		return ret;

	ret = cptra_mci_sha_test_run(shell, dev, "SHA512 short KAT", CRYPTO_HASH_ALGO_SHA512,
				     cptra_mci_sha512_msg_1, sizeof(cptra_mci_sha512_msg_1),
				     cptra_mci_sha512_md_1, sizeof(cptra_mci_sha512_md_1));
	if (ret)
		return ret;

	ret = cptra_mci_sha_test_run(shell, dev, "SHA512 long KAT", CRYPTO_HASH_ALGO_SHA512,
				     cptra_mci_sha512_msg_2, sizeof(cptra_mci_sha512_msg_2),
				     cptra_mci_sha512_md_2, sizeof(cptra_mci_sha512_md_2));
	if (ret)
		return ret;

	shell_print(shell, "MC_SHA: PASS (all 4 known-answer cases matched)");

	return 0;
}

/*
 * The MCI mailbox protocol caps a single MC_SHA_UPDATE/FINAL call at
 * CPTRA_MCI_MBOX_MAX_INPUT_SIZE bytes regardless of how much data hash_update()
 * is handed at once, so a chunk buffer of that size feeds the mailbox at full
 * efficiency without the driver having to split it further.
 */
static uint8_t cptra_mci_sha_perf_buf[CPTRA_MCI_MBOX_MAX_INPUT_SIZE];

static int cptra_mci_sha_perf_run(const struct shell *shell, const struct device *dev,
				  const char *label, size_t total_size)
{
	struct hash_ctx ctx = { 0 };
	struct hash_pkt pkt = { 0 };
	uint8_t digest[SHA384_DIGEST_SIZE];
	size_t remaining = total_size;
	uint32_t chunk, elapsed_ms;
	uint64_t start, kbps;
	int ret;

	/* See the comment in cmd_cptra_mci_sha_test() -- ctx.flags must be set here. */
	ctx.flags = crypto_query_hwcaps(dev);

	ret = hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA384);
	if (ret) {
		shell_error(shell, "hash_begin_session failed: %d", ret);
		return ret;
	}

	start = k_uptime_get();

	while (remaining > 0) {
		chunk = MIN(remaining, sizeof(cptra_mci_sha_perf_buf));
		remaining -= chunk;

		pkt.in_buf = cptra_mci_sha_perf_buf;
		pkt.in_len = chunk;

		if (remaining == 0) {
			pkt.out_buf = digest;
			ret = hash_compute(&ctx, &pkt);
		} else {
			ret = hash_update(&ctx, &pkt);
		}

		if (ret) {
			shell_error(shell, "%s: hash op failed: %d", label, ret);
			hash_free_session(dev, &ctx);
			return ret;
		}
	}

	hash_free_session(dev, &ctx);

	elapsed_ms = (uint32_t)(k_uptime_get() - start);
	if (elapsed_ms == 0)
		elapsed_ms = 1;

	kbps = ((uint64_t)total_size * 1000ULL) / elapsed_ms / 1024ULL;

	shell_print(shell, "%-6s: %10u bytes in %8u ms -> %u KB/s",
		    label, (unsigned int)total_size, elapsed_ms, (unsigned int)kbps);

	return 0;
}

static int cmd_cptra_mci_sha_perf(const struct shell *shell, size_t argc, char **argv)
{
	static const struct {
		const char *label;
		size_t size;
	} sizes[] = {
		{ "256KB", 256UL * 1024 },
		{ "512KB", 512UL * 1024 },
		{ "1MB",   1024UL * 1024 },
		{ "16MB",   1024UL * 1024 * 16 },
	};
	const struct device *dev;
	int ret;

	dev = DEVICE_DT_GET_ANY(aspeed_cptra_mci_sha);
	if (!dev) {
		shell_error(shell, "cptra_mci_sha device not found");
		return -ENODEV;
	}

	for (size_t i = 0; i < ARRAY_SIZE(sizes); i++) {
		ret = cptra_mci_sha_perf_run(shell, dev, sizes[i].label, sizes[i].size);
		if (ret)
			return ret;
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(cptra_mci_cmds,
	SHELL_CMD_ARG(fw_version, NULL,
		      "get firmware version [index: 0=CaliptraCore 1=McuRuntime 2=SoC]",
		      cmd_cptra_mci_fw_version, 1, 1),
	SHELL_CMD_ARG(subsystem_info, NULL,
		      "dump mci_reg/soc_ifc_reg status registers",
		      cmd_cptra_mci_subsystem_info, 1, 0),
	SHELL_CMD_ARG(device_caps, NULL,
		      "get device capabilities",
		      cmd_cptra_mci_device_caps, 1, 0),
	SHELL_CMD_ARG(cm_ecdsa_test, NULL,
		      "ECDSA pubkey+sign+verify on a cmk imported from an all-zero seed: "
		      "checks the public key against a known-answer vector, then tests "
		      "both that the correct digest passes and a tampered one is "
		      "rejected [digest_hex, default: random]",
		      cmd_cptra_mci_ecdsa_test, 1, 1),
	SHELL_CMD_ARG(cm_hmac_test, NULL,
		      "HMAC known-answer test: RFC 4231 test cases #1/#2 over both "
		      "SHA384 and SHA512",
		      cmd_cptra_mci_hmac_test, 1, 0),
	SHELL_CMD_ARG(cm_hkdf_test, NULL,
		      "HKDF known-answer test: runs both HMAC-KDF-Counter and "
		      "HKDF-Extract+Expand, each deriving an ML-DSA cmk and checking "
		      "its public key against a known vector",
		      cmd_cptra_mci_hkdf_test, 1, 0),
	SHELL_CMD_ARG(cm_mldsa_test, NULL,
		      "ML-DSA-87 pubkey+sign+verify on a cmk imported from an all-zero "
		      "seed: checks the public key against a known-answer vector, then "
		      "tests both that the correct message passes and a tampered one "
		      "is rejected [message_hex, default: random]",
		      cmd_cptra_mci_mldsa_test, 1, 1),
	SHELL_CMD_ARG(cm_ecdh_test, NULL,
		      "ECDH key exchange round trip: runs MC_ECDH_GENERATE/MC_ECDH_FINISH "
		      "for both simulated sides and checks they derive the same secret",
		      cmd_cptra_mci_ecdh_test, 1, 0),
	SHELL_CMD_ARG(sig_verify_test, NULL,
		      "ECDSA384/LMS/ML-DSA-87 raw-pubkey (not cmk) signature verify: "
		      "ECDSA384/ML-DSA-87 cross-check a Cmk-based signature via the "
		      "raw-pubkey passthrough, LMS verifies a real known-answer vector "
		      "(LMS keys are generated offline, not on-device) -- then each "
		      "checks that a tampered digest/message is rejected",
		      cmd_cptra_mci_sig_verify_test, 1, 0),
	SHELL_CMD_ARG(cm_status, NULL,
		      "get cmk key storage usage",
		      cmd_cptra_mci_cm_status, 1, 0),
	SHELL_CMD_ARG(cm_aes_test, NULL,
		      "AES encrypt/decrypt round trip across MC_AES_*_INIT/_UPDATE, "
		      "checks the decrypted output matches the original plaintext "
		      "[mode: 1=CBC 2=CTR, default: 2]",
		      cmd_cptra_mci_aes_test, 1, 1),
	SHELL_CMD_ARG(cm_aes_gcm_test, NULL,
		      "AES-GCM encrypt/decrypt round trip across "
		      "MC_AES_GCM_*_INIT/_UPDATE/_FINAL: tests both that the correct tag "
		      "passes and round-trips the plaintext, and a tampered tag is rejected",
		      cmd_cptra_mci_aes_gcm_test, 1, 0),
	SHELL_CMD_ARG(get_log, NULL,
		      "get log",
		      cmd_cptra_mci_get_log, 1, 0),
	SHELL_CMD_ARG(clear_log, NULL,
		      "clear log",
		      cmd_cptra_mci_clear_log, 1, 0),
	SHELL_CMD_ARG(auth_challenge, NULL,
		      "get auth cmd challenge nonce [flags]",
		      cmd_cptra_mci_auth_challenge, 1, 1),
	SHELL_CMD_ARG(export_csr, NULL,
		      "export attested CSR [device_key_id: 1=LDevID 2=FMCAlias 3=RTAlias] "
		      "[algo: 1=ECC384 2=MLDSA87]",
		      cmd_cptra_mci_export_csr, 1, 2),
	SHELL_CMD_ARG(random_stir, NULL,
		      "stir entropy into the CSRNG [input_hex, default: random]",
		      cmd_cptra_mci_random_stir, 1, 1),
	SHELL_CMD_ARG(random_generate, NULL,
		      "generate random bytes [size, default: 32, max: "
		      STRINGIFY(CPTRA_MCI_MBOX_MAX_INPUT_SIZE) "]",
		      cmd_cptra_mci_random_generate, 1, 1),
	SHELL_CMD_ARG(cm_trng_test, NULL,
		      "TRNG sanity test: stirs a fixed seed, then checks that a run of "
		      "generated chunks has no repeats within a trailing window",
		      cmd_cptra_mci_trng_test, 1, 0),
	SHELL_CMD_ARG(cm_sha_test, NULL,
		      "SHA known-answer test: NIST CAVS short/long message vectors for "
		      "both SHA384 and SHA512",
		      cmd_cptra_mci_sha_test, 1, 0),
	SHELL_CMD_ARG(sha_perf, NULL,
		      "SHA384 throughput over 256KB/512KB/1MB/16MB of data",
		      cmd_cptra_mci_sha_perf, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(cptra_mci, &cptra_mci_cmds, "Caliptra-SS MCI mailbox commands", NULL);
