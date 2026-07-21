/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include "ast_test.h"

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart5));

int test_uart(void)
{
	ast_zassert_true(device_is_ready(uart_dev), "UART device is not ready");

	return ast_ztest_result();
}

/*
 * tests/boards/ast1030_evb/all links this file too, calling test_uart()
 * from its own concurrent thread instead - skip registering this suite
 * there so the same check doesn't also run sequentially through ztest's
 * own runner first.
 */
#if !defined(AST1030_CONCURRENT_ALL)
ZTEST(uart, test_uart_device_ready)
{
	zassert_equal(test_uart(), AST_TEST_PASS, "uart test failed");
}

ZTEST_SUITE(uart, NULL, NULL, NULL, NULL, NULL);
#endif
