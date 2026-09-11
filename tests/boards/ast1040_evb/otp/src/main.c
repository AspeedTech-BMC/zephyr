/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reads the whole AST1040 OTP array through the AST27XX OTP driver API
 * (drivers/misc/aspeed/otp_ast2700.c) and checks it is blank: every word
 * must read back as 0, with no read error along the way.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/misc/aspeed/otp.h>
#include "ast_test.h"

/* AST1040 OTP array size: 0x1000 32-bit words (16KB). */
#define OTP_TOTAL_DW	0x1000
#define OTP_CHUNK_DW	16

int test_otp(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(otp));
	uint32_t data[OTP_CHUNK_DW];
	int ret;

	if (!ast_zassert_true(device_is_ready(dev), "OTP device not ready"))
		return ast_ztest_result();

	for (uint32_t off = 0; off < OTP_TOTAL_DW; off += OTP_CHUNK_DW) {
		uint32_t chunk = MIN(OTP_CHUNK_DW, OTP_TOTAL_DW - off);

		ret = otp_read_multi(dev, off, data, chunk);
		if (!ast_zassert_ok(ret, "otp_read_multi failed at dw 0x%03x: %d", off, ret))
			return ast_ztest_result();

		for (uint32_t i = 0; i < chunk; i++) {
			ast_zassert_equal(data[i], 0, "otp[dw 0x%03x] = 0x%08x, expected 0",
					   off + i, data[i]);
		}
	}

	return ast_ztest_result();
}

#if !defined(AST1040_CONCURRENT_ALL)
ZTEST(otp, otp_blank)
{
	zassert_equal(test_otp(), AST_TEST_PASS, "OTP blank-check failed");
}

ZTEST_SUITE(otp, NULL, NULL, NULL, NULL, NULL);
#endif
