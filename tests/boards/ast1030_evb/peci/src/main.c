/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include "ast_test.h"

static const struct device *const peci_dev = DEVICE_DT_GET(DT_NODELABEL(peci));

int test_peci(void)
{
	ast_zassert_true(device_is_ready(peci_dev), "PECI device is not ready");

	return ast_ztest_result();
}

/*
 * tests/boards/ast1030_evb/all links this file too, calling test_peci()
 * from its own concurrent thread instead - skip registering this suite
 * there so the same check doesn't also run sequentially through ztest's
 * own runner first.
 */
#if !defined(AST1030_CONCURRENT_ALL)
ZTEST(peci, test_peci_device_ready)
{
	zassert_equal(test_peci(), AST_TEST_PASS, "peci test failed");
}

ZTEST_SUITE(peci, NULL, NULL, NULL, NULL, NULL);
#endif
