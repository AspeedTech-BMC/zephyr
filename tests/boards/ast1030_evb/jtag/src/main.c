/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include "ast_test.h"

static const struct device *const jtag_dev = DEVICE_DT_GET(DT_NODELABEL(jtag1));

int test_jtag(void)
{
	ast_zassert_true(device_is_ready(jtag_dev), "JTAG master device is not ready");

	return ast_ztest_result();
}

/*
 * tests/boards/ast1030_evb/all links this file too, calling test_jtag()
 * from its own concurrent thread instead - skip registering this suite
 * there so the same check doesn't also run sequentially through ztest's
 * own runner first.
 */
#if !defined(AST1030_CONCURRENT_ALL)
ZTEST(jtag, test_jtag_device_ready)
{
	zassert_equal(test_jtag(), AST_TEST_PASS, "jtag test failed");
}

ZTEST_SUITE(jtag, NULL, NULL, NULL, NULL, NULL);
#endif
