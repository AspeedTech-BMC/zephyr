/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include "ast_test.h"

/* 16550-style Modem Control Register, offset 0x10 from the UART's base. */
#define UART_MCR_OFFSET 0x10
#define UART_MCR_LOOP   BIT(4)

/* uart_poll_in() is non-blocking, so retry until the looped-back byte lands. */
#define UART_LOOPBACK_POLL_RETRY_COUNT 1000

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart12));

static const struct device *const uart_devs[] = {
	DEVICE_DT_GET(DT_NODELABEL(uart0)),  DEVICE_DT_GET(DT_NODELABEL(uart1)),
	DEVICE_DT_GET(DT_NODELABEL(uart2)),  DEVICE_DT_GET(DT_NODELABEL(uart3)),
	DEVICE_DT_GET(DT_NODELABEL(uart4)),  DEVICE_DT_GET(DT_NODELABEL(uart5)),
	DEVICE_DT_GET(DT_NODELABEL(uart6)),  DEVICE_DT_GET(DT_NODELABEL(uart7)),
	DEVICE_DT_GET(DT_NODELABEL(uart8)),  DEVICE_DT_GET(DT_NODELABEL(uart9)),
	DEVICE_DT_GET(DT_NODELABEL(uart10)), DEVICE_DT_GET(DT_NODELABEL(uart11)),
};

/* Base addresses, in the same order as uart_devs[], for uart_set_internal_loopback(). */
static const uint32_t uart_bases[] = {
	DT_REG_ADDR(DT_NODELABEL(uart0)),  DT_REG_ADDR(DT_NODELABEL(uart1)),
	DT_REG_ADDR(DT_NODELABEL(uart2)),  DT_REG_ADDR(DT_NODELABEL(uart3)),
	DT_REG_ADDR(DT_NODELABEL(uart4)),  DT_REG_ADDR(DT_NODELABEL(uart5)),
	DT_REG_ADDR(DT_NODELABEL(uart6)),  DT_REG_ADDR(DT_NODELABEL(uart7)),
	DT_REG_ADDR(DT_NODELABEL(uart8)),  DT_REG_ADDR(DT_NODELABEL(uart9)),
	DT_REG_ADDR(DT_NODELABEL(uart10)), DT_REG_ADDR(DT_NODELABEL(uart11)),
};

BUILD_ASSERT(ARRAY_SIZE(uart_bases) == ARRAY_SIZE(uart_devs));

/*
 * Sets the MCR loopback bit, which loops TX straight back to RX inside the
 * UART IP - it never reaches the TX/RX pins, so no external wiring is
 * involved. Pokes the register directly rather than going through the
 * uart_aspeed driver, which does not implement line_ctrl_set() (or touch
 * MCR at all) - see drivers/serial/uart_aspeed.c.
 */
static bool uart_set_internal_loopback(uint32_t base, size_t idx, bool enable)
{
	uint32_t reg = sys_read32(base + UART_MCR_OFFSET);

	if (enable) {
		reg |= UART_MCR_LOOP;
	} else {
		reg &= ~UART_MCR_LOOP;
	}
	sys_write32(reg, base + UART_MCR_OFFSET);

	reg = sys_read32(base + UART_MCR_OFFSET);
	return ast_zassert_equal(!!(reg & UART_MCR_LOOP), enable,
				 "uart%zu MCR loopback bit readback mismatch", idx);
}

/* Discards any stale byte left in the RX FIFO from a previous transfer. */
static void uart_drain_rx(const struct device *dev)
{
	unsigned char discard;

	while (uart_poll_in(dev, &discard) == 0) {
	}
}

static bool uart_internal_loopback_byte(const struct device *dev, size_t idx, uint8_t tx)
{
	unsigned char rx = 0;
	int ret = -1;

	uart_drain_rx(dev);

	uart_poll_out(dev, tx);

	k_busy_wait(100);

	for (int attempt = 0; attempt < UART_LOOPBACK_POLL_RETRY_COUNT; attempt++) {
		ret = uart_poll_in(dev, &rx);
		if (ret == 0) {
			break;
		}
		k_msleep(1);
	}

	if (!ast_zassert_ok(ret, "uart%zu poll_in timed out for tx 0x%02x", idx, tx)) {
		return false;
	}

	return ast_zassert_equal(rx, tx,
				 "uart%zu internal loopback mismatch: sent 0x%02x, got 0x%02x",
				 idx, tx, rx);
}

static void test_uart_internal_loopback(const struct device *dev, size_t idx)
{
	for (int value = 0x00; value <= 0xff; value++) {
		if (!uart_internal_loopback_byte(dev, idx, (uint8_t)value)) {
			return;
		}
	}
}

int test_uart(void)
{
	ast_zassert_true(device_is_ready(uart_dev), "UART device is not ready");

	for (size_t i = 0; i < ARRAY_SIZE(uart_devs); i++) {
		if (!ast_zassert_true(device_is_ready(uart_devs[i]), "uart%zu device is not ready",
				      i)) {
			continue;
		}
		if (uart_set_internal_loopback(uart_bases[i], i, true)) {
			test_uart_internal_loopback(uart_devs[i], i);
		}
		uart_set_internal_loopback(uart_bases[i], i, false);
	}

	return ast_ztest_result();
}

/*
 * tests/boards/ast1040_evb/all links this file too, calling test_uart()
 * from its own concurrent thread instead - skip registering this suite
 * there so the same check doesn't also run sequentially through ztest's
 * own runner first.
 */
#if !defined(AST1040_CONCURRENT_ALL)
ZTEST(uart, test_uart_internal_loopback_all)
{
	zassert_equal(test_uart(), AST_TEST_PASS, "uart test failed");
}

ZTEST_SUITE(uart, NULL, NULL, NULL, NULL, NULL);
#endif
