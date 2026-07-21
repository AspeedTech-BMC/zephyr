/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include "ast_test.h"

/*
 * The old CI-mode eSPI test (tests/boards/ast1030/src/espi.c on
 * aspeed-dev-v2.6.0) did nothing but return 0 - the register-level bit poke
 * lived only in the FT (production test) path, which is out of scope for
 * this port. Keep the same CI intent: confirm the controller is present.
 */
static const struct device *const espi_dev = DEVICE_DT_GET(DT_NODELABEL(espi));

int test_espi(void)
{
	ast_zassert_true(device_is_ready(espi_dev), "eSPI device is not ready");

	return ast_ztest_result();
}

/*
 * tests/boards/ast1030_evb/all links this file too, calling test_espi()
 * from its own concurrent thread instead - skip registering this suite
 * there so the same check doesn't also run sequentially through ztest's
 * own runner first.
 */
#if !defined(AST1030_CONCURRENT_ALL)
ZTEST(espi, test_espi_device_ready)
{
	zassert_equal(test_espi(), AST_TEST_PASS, "espi test failed");
}

ZTEST_SUITE(espi, NULL, NULL, NULL, NULL, NULL);
#endif
