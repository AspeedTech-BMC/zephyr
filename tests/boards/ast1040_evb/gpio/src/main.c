/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * AST1040 has no prior test suite to port (checked aspeed-dev-v2.6.0 and
 * every other branch/tag in this repo - nothing exists), so this is new
 * content: a simple self-loopback smoke check, following the same
 * ast_zassert_* / test_<name>(void) convention used throughout
 * tests/boards/ast1030_evb/ so the combined runner in
 * tests/boards/ast1040_evb/all/ can share this same function - see that
 * directory's main.c.
 *
 * Global pins 108 (out) / 109 (in) are loopback-wired on the real AST1040
 * EVB; both fall in the gpio0_96_127 bank (pin-offset 96), so the
 * bank-local pin numbers used below are 12 and 13 - see the overlay in
 * boards/ast1040_evb_ast1040_cm4.overlay for the offset math.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "ast_test.h"

#define GPIO_LOOPBACK_OUT_PIN 12
#define GPIO_LOOPBACK_IN_PIN  13

int test_gpio(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(gpio0_96_127));

	ast_zassert_true(device_is_ready(dev), "gpio0_96_127 device is not ready");

	ast_zassert_ok(gpio_pin_configure(dev, GPIO_LOOPBACK_OUT_PIN, GPIO_OUTPUT),
		       "gpio_pin_configure(out) failed");
	ast_zassert_ok(gpio_pin_configure(dev, GPIO_LOOPBACK_IN_PIN, GPIO_INPUT),
		       "gpio_pin_configure(in) failed");

	ast_zassert_ok(gpio_pin_set(dev, GPIO_LOOPBACK_OUT_PIN, 1), "gpio_pin_set(1) failed");
	ast_zassert_equal(gpio_pin_get(dev, GPIO_LOOPBACK_IN_PIN), 1, "loopback read-back != 1");

	ast_zassert_ok(gpio_pin_set(dev, GPIO_LOOPBACK_OUT_PIN, 0), "gpio_pin_set(0) failed");
	ast_zassert_equal(gpio_pin_get(dev, GPIO_LOOPBACK_IN_PIN), 0, "loopback read-back != 0");

	return ast_ztest_result();
}

#if !defined(AST1040_CONCURRENT_ALL)
ZTEST(gpio, test_gpio_all)
{
	zassert_equal(test_gpio(), AST_TEST_PASS, "gpio test failed");
}

ZTEST_SUITE(gpio, NULL, NULL, NULL, NULL, NULL);
#endif
