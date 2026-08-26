/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ports the logic of cmd_cptra_mci_mldsa_test() (drivers/misc/aspeed/
 * cptra_mci_shell.c) into the ast_zassert_* / test_<name>(void) convention
 * used throughout tests/boards/ast1040_evb/, so it can later be folded into
 * the combined runner in tests/boards/ast1040_evb/all/ the same way test_gpio
 * is. Unlike the shell command, this does not print progress/hexdump output
 * -- only ast_zassert_*'s own failure messages, same as every other
 * peripheral test here.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/misc/aspeed/cptra_mci_mbox.h>
#include "ast_test.h"

/*
 * ML-DSA-87 public keys/signatures (2592/4628 bytes) are far too large for
 * a thread's stack -- kept static, same reasoning as the shell test /
 * driver's own request buffers.
 */
static uint8_t mldsa_pubkey[CPTRA_MCI_MLDSA87_PUBKEY_SIZE];
static uint8_t mldsa_sig[CPTRA_MCI_MLDSA87_SIGNATURE_SIZE];

/*
 * Expected ML-DSA-87 public key prefix (p || t1, the first 64 bytes) for the
 * all-zero 32-byte seed imported below -- same known-answer vector as
 * cmd_cptra_mci_mldsa_test().
 */
static const uint8_t mldsa_expected_pubkey[64] = {
	0xe4, 0x5f, 0xfc, 0x8c, 0xc7, 0x3d, 0xb8, 0x85, 0xdc, 0x66, 0x2e, 0x62, 0xa1, 0x8c, 0xd8,
	0xe3, 0x80, 0x32, 0x97, 0x11, 0x7f, 0xa5, 0x65, 0x88, 0x14, 0xa9, 0x85, 0xb5, 0xff, 0x1d,
	0xb7, 0xb4, 0x68, 0xcf, 0xc8, 0x2b, 0xb9, 0x29, 0xf1, 0xd8, 0x6b, 0x77, 0xed, 0x14, 0xf5,
	0xae, 0x16, 0xa6, 0x53, 0x68, 0x77, 0x2c, 0xe5, 0x19, 0x12, 0x41, 0x01, 0x05, 0xe0, 0x45,
	0x69, 0x75, 0xae, 0x91,
};

int test_cptra_mci_mldsa(void)
{
	/* All-zero seed: pubkey is then a known, fixed value (see the comment above). */
	static const uint8_t key[32];
	uint8_t cmk[CPTRA_MCI_CMK_SIZE];
	uint8_t message[CPTRA_MCI_ECC384_SCALAR_SIZE];
	int ret;

	ret = cptra_mci_import_key(CPTRA_MCI_KEY_USAGE_MLDSA, key, sizeof(key), cmk);
	if (!ast_zassert_ok(ret, "MC_IMPORT failed: %d", ret))
		return ast_ztest_result();

	ret = cptra_mci_mldsa_cmk_public_key(cmk, mldsa_pubkey);
	if (!ast_zassert_ok(ret, "MC_MLDSA_CMK_PUBLIC_KEY failed: %d", ret))
		return ast_ztest_result();

	ast_zassert_mem_equal(mldsa_pubkey, mldsa_expected_pubkey, sizeof(mldsa_expected_pubkey),
			      "public key does not match known-answer vector");

	for (size_t i = 0; i < sizeof(message); i += sizeof(uint32_t)) {
		uint32_t r = k_cycle_get_32() ^ (uint32_t)i;

		memcpy(&message[i], &r, MIN(sizeof(r), sizeof(message) - i));
	}

	ret = cptra_mci_mldsa_cmk_sign(cmk, message, sizeof(message), mldsa_sig);
	if (!ast_zassert_ok(ret, "MC_MLDSA_CMK_SIGN failed: %d", ret))
		return ast_ztest_result();

	ret = cptra_mci_mldsa_cmk_verify(cmk, mldsa_sig, message, sizeof(message));
	ast_zassert_ok(ret, "verify FAIL (expected PASS): %d", ret);

	return ast_ztest_result();
}

/*
 * Entry point for callers outside this file (the standalone ZTEST() below,
 * and tests/boards/ast1040_evb/all/) -- currently just ML-DSA-87, but keeps
 * the externally-visible name generic so more cptra_mci_* sub-tests can be
 * folded in here later without changing callers.
 */
int test_cptra_mci(void)
{
	return test_cptra_mci_mldsa();
}

#if !defined(AST1040_CONCURRENT_ALL)
ZTEST(cptra, cptra_mci)
{
	zassert_equal(test_cptra_mci(), AST_TEST_PASS, "mldsa test failed");
}

ZTEST_SUITE(cptra, NULL, NULL, NULL, NULL, NULL);
#endif
