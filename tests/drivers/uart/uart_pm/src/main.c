/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/uart.h>
#include <device.h>
#include <pm/device.h>
#include <ztest.h>

#if defined(CONFIG_BOARD_NRF52840DK_NRF52840)
#define LABEL uart0
#endif

#define UART_DEVICE_NAME DT_LABEL(DT_NODELABEL(LABEL))
#define HAS_RX DT_NODE_HAS_PROP(DT_NODELABEL(LABEL), rx_pin)

static const struct device *exp_dev;
static uint32_t exp_state;
static void *exp_arg;
static volatile int callback_cnt;

static void polling_verify(const struct device *dev, bool is_async, bool active)
{
	char c;
	char outs[] = "abc";
	int err;

	if (!HAS_RX || is_async) {
		/* If no RX pin just run few poll outs to check that it does
		 * not hang.
		 */
		for (int i = 0; i < ARRAY_SIZE(outs); i++) {
			uart_poll_out(dev, outs[i]);
		}

		return;
	}

	err = uart_poll_in(dev, &c);
	zassert_equal(err, -1, NULL);

	for (int i = 0; i < ARRAY_SIZE(outs); i++) {
		uart_poll_out(dev, outs[i]);
		k_busy_wait(1000);

		if (active) {
			err = uart_poll_in(dev, &c);
			zassert_equal(err, 0, "Unexpected err: %d", err);
			zassert_equal(c, outs[i], NULL);
		}

		err = uart_poll_in(dev, &c);
		zassert_equal(err, -1, NULL);
	}
}

static void async_callback(const struct device *dev, struct uart_event *evt, void *ctx)
{
	bool *done = ctx;

	switch (evt->type) {
	case UART_TX_DONE:
		*done = true;
		break;
	default:
		break;
	}
}

static bool async_verify(const struct device *dev, bool active)
{
	char txbuf[] = "test";
	uint8_t rxbuf[32];
	volatile bool tx_done = false;
	int err;

	err = uart_callback_set(dev, async_callback, (void *)&tx_done);
	if (err == -ENOTSUP) {
		return false;
	}

	if (!active) {
		return true;
	}

	zassert_equal(err, 0, "Unexpected err: %d", err);

	if (HAS_RX) {
		err = uart_rx_enable(dev, rxbuf, sizeof(rxbuf), 1);
		zassert_equal(err, 0, "Unexpected err: %d", err);
	}

	err = uart_tx(dev, txbuf, sizeof(txbuf), 10);
	zassert_equal(err, 0, "Unexpected err: %d", err);

	k_busy_wait(10000);

	if (HAS_RX) {
		err = uart_rx_disable(dev);
		zassert_equal(err, 0, "Unexpected err: %d", err);

		k_busy_wait(10000);

		err = memcmp(txbuf, rxbuf, sizeof(txbuf));
		zassert_equal(err, 0, "Unexpected err: %d", err);
	}

	zassert_true(tx_done, NULL);

	return true;
}

static void communication_verify(const struct device *dev, bool active)
{
	bool is_async = async_verify(dev, active);

	polling_verify(dev, is_async, active);
}

#define state_verify(dev, exp_state) do {\
	uint32_t power_state; \
	int err = pm_device_state_get(dev, &power_state); \
	zassert_equal(err, 0, "Unexpected err: %d", err); \
	zassert_equal(power_state, exp_state, NULL); \
} while (0)

static void pm_callback(const struct device *dev,
			int status, uint32_t *state, void *arg)
{
	zassert_equal(dev, exp_dev, NULL);
	zassert_equal(status, 0, NULL);
	zassert_equal(*state, exp_state, NULL);
	zassert_equal(arg, exp_arg, NULL);
	callback_cnt++;
}

static void state_set(const struct device *dev, uint32_t state, int exp_err, bool cb)
{
	int err;
	uint32_t prev_state;

	err = pm_device_state_get(dev, &prev_state);
	zassert_equal(err, 0, "Unexpected err: %d", err);

	if (cb) {
		callback_cnt = 0;
		exp_dev = dev;
		exp_arg = &state;
		exp_state = state;

		err = pm_device_state_set(dev, state, pm_callback, exp_arg);
		zassert_equal(err, exp_err, "Unexpected err: %d", err);
		zassert_equal(callback_cnt, 1, NULL);
	} else {
		err = pm_device_state_set(dev, state, NULL, NULL);
		zassert_equal(err, exp_err, "Unexpected err: %d", err);
	}

	uint32_t exp_state = err == 0 ? state : prev_state;

	state_verify(dev, exp_state);
}

static void test_uart_pm_in_idle(void)
{
	const struct device *dev;

	dev = device_get_binding(UART_DEVICE_NAME);
	zassert_true(dev != NULL, NULL);

	state_verify(dev, PM_DEVICE_STATE_ACTIVE);
	communication_verify(dev, true);

	state_set(dev, PM_DEVICE_STATE_LOW_POWER, 0, false);
	communication_verify(dev, false);

	state_set(dev, PM_DEVICE_STATE_ACTIVE, 0, false);
	communication_verify(dev, true);

	state_set(dev, PM_DEVICE_STATE_LOW_POWER, 0, true);
	communication_verify(dev, false);

	state_set(dev, PM_DEVICE_STATE_ACTIVE, 0, true);
	communication_verify(dev, true);
}

static void test_uart_pm_poll_tx(void)
{
	const struct device *dev;

	dev = device_get_binding(UART_DEVICE_NAME);
	zassert_true(dev != NULL, NULL);

	communication_verify(dev, true);

	uart_poll_out(dev, 'a');
	state_set(dev, PM_DEVICE_STATE_LOW_POWER, 0, false);

	communication_verify(dev, false);

	state_set(dev, PM_DEVICE_STATE_ACTIVE, 0, false);

	communication_verify(dev, true);

	/* Now same thing but with callback */
	uart_poll_out(dev, 'a');
	state_set(dev, PM_DEVICE_STATE_LOW_POWER, 0, true);

	communication_verify(dev, false);

	state_set(dev, PM_DEVICE_STATE_ACTIVE, 0, true);

	communication_verify(dev, true);
}

static void timeout(struct k_timer *timer)
{
	const struct device *uart = k_timer_user_data_get(timer);

	state_set(uart, PM_DEVICE_STATE_LOW_POWER, 0, false);
}

static K_TIMER_DEFINE(pm_timer, timeout, NULL);

/* Test going into low power state after interrupting poll out. Use various
 * delays to test interruption at multiple places.
 */
static void test_uart_pm_poll_tx_interrupted(void)
{
	const struct device *dev;
	char str[] = "test";

	dev = device_get_binding(UART_DEVICE_NAME);
	zassert_true(dev != NULL, NULL);

	k_timer_user_data_set(&pm_timer, (void *)dev);

	for (int i = 1; i < 100; i++) {
		k_timer_start(&pm_timer, K_USEC(i * 10), K_NO_WAIT);

		for (int j = 0; j < sizeof(str); j++) {
			uart_poll_out(dev, str[j]);
		}

		k_timer_status_sync(&pm_timer);

		state_set(dev, PM_DEVICE_STATE_ACTIVE, 0, false);

		communication_verify(dev, true);
	}
}

void test_main(void)
{
	if (!HAS_RX) {
		PRINT("No RX pin\n");
	}

	ztest_test_suite(uart_pm,
			 ztest_unit_test(test_uart_pm_in_idle),
			 ztest_unit_test(test_uart_pm_poll_tx),
			 ztest_unit_test(test_uart_pm_poll_tx_interrupted)
			);
	ztest_run_test_suite(uart_pm);
}
