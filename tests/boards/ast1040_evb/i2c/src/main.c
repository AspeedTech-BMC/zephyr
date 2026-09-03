/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tests EEPROM and IPMB target loopback on i2c4/i2c5 and i2c10/i2c11.
 * Every pair runs at standard, fast, and fast-plus speed.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c/target/eeprom.h>
#include <zephyr/drivers/i2c/target/ipmb.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>
#include "ast_test.h"

#define TEST_DATA_SIZE 32
#define TRANSFER_RETRY_COUNT 5
#define IPMB_PREFIX_LEN 2
#define IPMB_TARGET_ADDED_LEN 1
#define IPMB_REQUEST_SIZE 6
#define IPMI_NETFN_APP_REQUEST 0x06
#define IPMB_RESPONDER_LUN 0
#define IPMB_REQUESTER_ADDR_7BIT 0x20
#define IPMB_REQUESTER_LUN 0
#define IPMI_GET_DEVICE_ID_CMD 0x01
#define IPMB_FIFO_RELEASE_COUNT 2

#define EEPROM_SIZE 256
#define EEPROM_WRAP_DATA_SIZE 16
#define EEPROM_WRAP_TAIL_SIZE 6
#define EEPROM_WRAP_HEAD_SIZE (EEPROM_WRAP_DATA_SIZE - EEPROM_WRAP_TAIL_SIZE)
#define EEPROM_WRAP_START_OFFSET (EEPROM_SIZE - EEPROM_WRAP_TAIL_SIZE)

struct i2c_test_speed {
	uint32_t value;
	const char *name;
};

static const struct i2c_test_speed i2c_speeds[] = {
	{I2C_SPEED_STANDARD, "standard"},
	{I2C_SPEED_FAST, "fast"},
	{I2C_SPEED_FAST_PLUS, "fast-plus"},
};

struct i2c_target_pair {
	const char *name;
	const struct device *controller;
	const struct device *target;
	uint16_t target_addr;
	size_t queue_capacity;
};

static const struct i2c_target_pair eeprom_pairs[] = {
	{
		.name = "i2c4->eeprom5",
		.controller = DEVICE_DT_GET(DT_NODELABEL(i2c4)),
		.target = DEVICE_DT_GET(DT_NODELABEL(eeprom5)),
		.target_addr = DT_REG_ADDR(DT_NODELABEL(eeprom5)),
	},
	{
		.name = "i2c10->eeprom11",
		.controller = DEVICE_DT_GET(DT_NODELABEL(i2c10)),
		.target = DEVICE_DT_GET(DT_NODELABEL(eeprom11)),
		.target_addr = DT_REG_ADDR(DT_NODELABEL(eeprom11)),
	},
};

static const struct i2c_target_pair ipmb_pairs[] = {
	{
		.name = "i2c5->ipmb4",
		.controller = DEVICE_DT_GET(DT_NODELABEL(i2c5)),
		.target = DEVICE_DT_GET(DT_NODELABEL(ipmb4)),
		.target_addr = DT_REG_ADDR(DT_NODELABEL(ipmb4)),
		.queue_capacity = DT_PROP(DT_NODELABEL(ipmb4), size),
	},
	{
		.name = "i2c11->ipmb10",
		.controller = DEVICE_DT_GET(DT_NODELABEL(i2c11)),
		.target = DEVICE_DT_GET(DT_NODELABEL(ipmb10)),
		.target_addr = DT_REG_ADDR(DT_NODELABEL(ipmb10)),
		.queue_capacity = DT_PROP(DT_NODELABEL(ipmb10), size),
	},
};

typedef void (*i2c_target_test_fn)(const struct i2c_target_pair *pair,
				   const struct i2c_test_speed *speed);

static int retry_i2c_write(const struct i2c_target_pair *pair, const uint8_t *buf, size_t len)
{
	int result;

	for (int attempt = 0; attempt < TRANSFER_RETRY_COUNT; attempt++) {
		result = i2c_write(pair->controller, buf, len, pair->target_addr);
		if (result != -ETIMEDOUT) {
			break;
		}
	}

	return result;
}

static int eeprom_write_bytes(const struct i2c_target_pair *pair, uint8_t offset,
			      const uint8_t *data, size_t len)
{
	int result;

	for (int attempt = 0; attempt < TRANSFER_RETRY_COUNT; attempt++) {
		result = 0;
		for (size_t i = 0; i < len; i++) {
			result = i2c_reg_write_byte(pair->controller, pair->target_addr, offset + i,
						    data[i]);
			if (result != 0) {
				break;
			}
		}
		if (result != -ETIMEDOUT) {
			break;
		}
	}

	return result;
}

static int eeprom_read_bytes(const struct i2c_target_pair *pair, uint8_t offset, uint8_t *data,
			     size_t len)
{
	int result;

	for (int attempt = 0; attempt < TRANSFER_RETRY_COUNT; attempt++) {
		result = 0;
		for (size_t i = 0; i < len; i++) {
			result = i2c_reg_read_byte(pair->controller, pair->target_addr, offset + i,
						   &data[i]);
			if (result != 0) {
				break;
			}
		}
		if (result != -ETIMEDOUT) {
			break;
		}
	}

	return result;
}

static int retry_ipmb_read(const struct device *target, struct ipmb_msg **msg, uint8_t *length)
{
	int result;

	for (int attempt = 0; attempt < TRANSFER_RETRY_COUNT; attempt++) {
		result = ipmb_target_read(target, msg, length);
		if (result == 0) {
			break;
		}
	}

	return result;
}

static uint8_t ipmb_checksum(const uint8_t *data, size_t len)
{
	uint8_t sum = 0;

	for (size_t i = 0; i < len; i++) {
		sum += data[i];
	}

	return (uint8_t)-sum;
}

static void build_ipmb_request(const struct i2c_target_pair *pair, size_t sequence,
			       uint8_t request[IPMB_REQUEST_SIZE])
{
	uint8_t header[] = {
		/* IPMB uses the 8-bit responder address in its header checksum. */
		(uint8_t)(pair->target_addr << 1),
		/* Application request NetFn in bits [7:2], responder LUN 0 in [1:0]. */
		(IPMI_NETFN_APP_REQUEST << 2) | IPMB_RESPONDER_LUN,
	};

	request[0] = header[1];
	request[1] = ipmb_checksum(header, ARRAY_SIZE(header));
	/* rqSA is the requester's 7-bit address shifted into IPMB's 8-bit form. */
	request[2] = IPMB_REQUESTER_ADDR_7BIT << 1;
	/* The upper six bits identify this request; the lower two select requester LUN 0. */
	request[3] = (uint8_t)(((sequence & 0x3f) << 2) | IPMB_REQUESTER_LUN);
	request[4] = IPMI_GET_DEVICE_ID_CMD;
	request[5] = ipmb_checksum(&request[2], 3);
}

static bool write_ipmb_sequence(const struct i2c_target_pair *pair,
				const struct i2c_test_speed *speed, size_t sequence)
{
	uint8_t request[IPMB_REQUEST_SIZE];
	int result;

	build_ipmb_request(pair, sequence, request);
	result = retry_i2c_write(pair, request, sizeof(request));

	return ast_zassert_ok(result, "%s sequence %zu write at %s failed: %d", pair->name,
			      sequence, speed->name, result);
}

static bool read_and_verify_ipmb_sequence(const struct i2c_target_pair *pair,
					  const struct i2c_test_speed *speed,
					  size_t expected_sequence)
{
	uint8_t expected[IPMB_REQUEST_SIZE];
	struct ipmb_msg *msg = NULL;
	uint8_t length = 0;
	uint8_t *buf;
	int result;

	result = retry_ipmb_read(pair->target, &msg, &length);
	if (!ast_zassert_ok(result, "%s sequence %zu read at %s failed: %d", pair->name,
			    expected_sequence, speed->name, result)) {
		return false;
	}
	if (!ast_zassert_not_null(msg, "%s sequence %zu returned NULL at %s", pair->name,
				  expected_sequence, speed->name)) {
		return false;
	}

	build_ipmb_request(pair, expected_sequence, expected);
	if (!ast_zassert_equal(length, sizeof(expected) + IPMB_TARGET_ADDED_LEN,
			       "%s sequence %zu length mismatch at %s: %d", pair->name,
			       expected_sequence, speed->name, length)) {
		return false;
	}

	buf = (uint8_t *)msg;
	if (!ast_zassert_equal(buf[0], pair->target_addr << 1,
			       "%s sequence %zu address mismatch at %s", pair->name,
			       expected_sequence, speed->name)) {
		return false;
	}

	return ast_zassert_mem_equal(expected, &buf[IPMB_TARGET_ADDED_LEN], sizeof(expected),
				     "%s expected sequence %zu at %s", pair->name,
				     expected_sequence, speed->name);
}

static void verify_ipmb_queue_empty(const struct i2c_target_pair *pair,
				    const struct i2c_test_speed *speed)
{
	struct ipmb_msg *msg = NULL;
	uint8_t length = 0;
	int result;

	result = ipmb_target_read(pair->target, &msg, &length);
	ast_zassert_not_equal(result, 0, "%s queue is not empty at %s", pair->name, speed->name);
}

static void test_eeprom(const struct i2c_target_pair *pair,
			const struct i2c_test_speed *speed)
{
	uint8_t data_s[TEST_DATA_SIZE];
	uint8_t data_r[TEST_DATA_SIZE];
	int result;

	sys_rand_get(data_s, sizeof(data_s));

	result = eeprom_write_bytes(pair, 0, data_s, sizeof(data_s));
	if (!ast_zassert_ok(result, "%s write at %s failed: %d", pair->name, speed->name,
			    result)) {
		return;
	}

	result = eeprom_read_bytes(pair, 0, data_r, sizeof(data_r));
	if (!ast_zassert_ok(result, "%s read at %s failed: %d", pair->name, speed->name,
			    result)) {
		return;
	}

	ast_zassert_mem_equal(data_s, data_r, sizeof(data_s), "%s R/W mismatch at %s", pair->name,
			       speed->name);
}

static void test_eeprom_wraparound(const struct i2c_target_pair *pair,
				   const struct i2c_test_speed *speed)
{
	uint8_t data_s[EEPROM_WRAP_DATA_SIZE];
	uint8_t data_r[MAX(EEPROM_WRAP_TAIL_SIZE, EEPROM_WRAP_HEAD_SIZE)];
	uint8_t write_buf[EEPROM_WRAP_DATA_SIZE + 1];
	int result;

	sys_rand_get(data_s, sizeof(data_s));
	/* An EEPROM write starts with the memory offset. Starting at 250 makes
	 * the 16-byte payload cross the 256-byte boundary.
	 */
	write_buf[0] = EEPROM_WRAP_START_OFFSET;
	memcpy(&write_buf[1], data_s, sizeof(data_s));

	/* One transaction is required to exercise target-side address wrapping. */
	result = retry_i2c_write(pair, write_buf, sizeof(write_buf));
	if (!ast_zassert_ok(result, "%s wrap write at %s failed: %d", pair->name, speed->name,
			    result)) {
		return;
	}

	result = eeprom_read_bytes(pair, EEPROM_WRAP_START_OFFSET, data_r,
				    EEPROM_WRAP_TAIL_SIZE);
	if (!ast_zassert_ok(result, "%s wrap tail read at %s failed: %d", pair->name,
			    speed->name, result)) {
		return;
	}
	ast_zassert_mem_equal(data_s, data_r, EEPROM_WRAP_TAIL_SIZE,
			       "%s wrap tail mismatch at %s", pair->name, speed->name);

	result = eeprom_read_bytes(pair, 0, data_r, EEPROM_WRAP_HEAD_SIZE);
	if (!ast_zassert_ok(result, "%s wrap head read at %s failed: %d", pair->name,
			    speed->name, result)) {
		return;
	}
	ast_zassert_mem_equal(&data_s[EEPROM_WRAP_TAIL_SIZE], data_r, EEPROM_WRAP_HEAD_SIZE,
			       "%s wrap head mismatch at %s", pair->name, speed->name);
}

static void test_ipmb(const struct i2c_target_pair *pair, const struct i2c_test_speed *speed)
{
	uint8_t data_s[TEST_DATA_SIZE];
	uint8_t write_buf[TEST_DATA_SIZE + 1];
	struct ipmb_msg *msg = NULL;
	uint8_t length = 0;
	uint8_t *buf;
	int result;

	sys_rand_get(data_s, sizeof(data_s));
	/* The target supplies rsSA at msg[0]. This byte sets netFn_rsLUN at
	 * msg[1] to zero, leaving the random test data to start at msg[2].
	 */
	write_buf[0] = 0;
	memcpy(&write_buf[1], data_s, sizeof(data_s));

	result = retry_i2c_write(pair, write_buf, sizeof(write_buf));
	if (!ast_zassert_ok(result, "%s write at %s failed: %d", pair->name, speed->name,
			    result)) {
		return;
	}

	/* An empty IPMB queue is reported as 1, so retry any nonzero result. */
	result = retry_ipmb_read(pair->target, &msg, &length);
	if (!ast_zassert_ok(result, "%s read at %s failed: %d", pair->name, speed->name,
			    result)) {
		return;
	}
	if (!ast_zassert_not_null(msg, "%s read at %s returned NULL", pair->name, speed->name)) {
		return;
	}

	/* The target adds its address at byte 0, and write_buf[0] becomes byte 1.
	 * The test payload therefore starts at byte 2.
	 */
	if (!ast_zassert_equal(length, TEST_DATA_SIZE + IPMB_PREFIX_LEN,
			       "%s length mismatch at %s: %d", pair->name, speed->name, length)) {
		return;
	}

	buf = (uint8_t *)msg;
	ast_zassert_equal(buf[0], pair->target_addr << 1, "%s device ID mismatch at %s: %d",
			  pair->name, speed->name, buf[0]);
	ast_zassert_mem_equal(data_s, &buf[IPMB_PREFIX_LEN], sizeof(data_s),
			       "%s R/W mismatch at %s", pair->name, speed->name);
}

static void test_ipmb_queue_full(const struct i2c_target_pair *pair,
				 const struct i2c_test_speed *speed)
{
	uint8_t request[IPMB_REQUEST_SIZE];

	for (size_t i = 0; i < pair->queue_capacity; i++) {
		if (!write_ipmb_sequence(pair, speed, i)) {
			return;
		}
	}

	/* ASPEED packet mode has already ACKed the transfer when the target callback
	 * rejects it, so queue contents rather than i2c_write() status prove rejection.
	 */
	build_ipmb_request(pair, pair->queue_capacity, request);
	(void)i2c_write(pair->controller, request, sizeof(request), pair->target_addr);

	for (size_t i = 0; i < pair->queue_capacity; i++) {
		if (!read_and_verify_ipmb_sequence(pair, speed, i)) {
			return;
		}
	}

	verify_ipmb_queue_empty(pair, speed);
}

static void test_ipmb_queue_fifo(const struct i2c_target_pair *pair,
				 const struct i2c_test_speed *speed)
{
	if (!ast_zassert_true(pair->queue_capacity >= IPMB_FIFO_RELEASE_COUNT,
			      "%s queue capacity is too small at %s", pair->name, speed->name)) {
		return;
	}

	/* Fill the queue with sequences 0 through 4 when its DTS size is five. */
	for (size_t sequence = 0; sequence < pair->queue_capacity; sequence++) {
		if (!write_ipmb_sequence(pair, speed, sequence)) {
			return;
		}
	}

	/* Remove the first two entries so their circular-buffer slots can be reused. */
	for (size_t sequence = 0; sequence < IPMB_FIFO_RELEASE_COUNT; sequence++) {
		if (!read_and_verify_ipmb_sequence(pair, speed, sequence)) {
			return;
		}
	}

	/* Append sequences 5 and 6 to the queue when its DTS size is five. */
	for (size_t sequence = pair->queue_capacity;
	     sequence < pair->queue_capacity + IPMB_FIFO_RELEASE_COUNT; sequence++) {
		if (!write_ipmb_sequence(pair, speed, sequence)) {
			return;
		}
	}

	/* Entries 2 through 6 must retain FIFO order across the slot wraparound. */
	for (size_t sequence = IPMB_FIFO_RELEASE_COUNT;
	     sequence < pair->queue_capacity + IPMB_FIFO_RELEASE_COUNT; sequence++) {
		if (!read_and_verify_ipmb_sequence(pair, speed, sequence)) {
			return;
		}
	}

	verify_ipmb_queue_empty(pair, speed);
}

static void run_i2c_target_tests(const char *test_name, const struct i2c_target_pair *pairs,
				 size_t pair_count, i2c_target_test_fn test_fn)
{
	for (size_t pair_idx = 0; pair_idx < pair_count; pair_idx++) {
		const struct i2c_target_pair *pair = &pairs[pair_idx];
		bool controller_ready;
		bool target_ready;

		controller_ready = ast_zassert_true(device_is_ready(pair->controller),
						    "%s controller not ready", pair->name);
		target_ready = ast_zassert_true(device_is_ready(pair->target),
						"%s target not ready", pair->name);
		if (!controller_ready || !target_ready) {
			continue;
		}

		for (size_t speed_idx = 0; speed_idx < ARRAY_SIZE(i2c_speeds); speed_idx++) {
			const struct i2c_test_speed *speed = &i2c_speeds[speed_idx];
			int result;

			result = i2c_configure(pair->controller, I2C_MODE_CONTROLLER |
							 I2C_SPEED_SET(speed->value));
			if (!ast_zassert_ok(result, "%s %s configure at %s failed: %d", test_name,
					    pair->name, speed->name, result)) {
				continue;
			}

			result = i2c_target_driver_register(pair->target);
			if (!ast_zassert_ok(result, "%s %s register at %s failed: %d", test_name,
					    pair->name, speed->name, result)) {
				continue;
			}

			test_fn(pair, speed);

			result = i2c_target_driver_unregister(pair->target);
			ast_zassert_ok(result, "%s %s unregister at %s failed: %d", test_name,
				       pair->name, speed->name, result);
		}
	}
}

int test_i2c(void)
{
	run_i2c_target_tests("eeprom", eeprom_pairs, ARRAY_SIZE(eeprom_pairs), test_eeprom);
	run_i2c_target_tests("eeprom-wrap", eeprom_pairs, ARRAY_SIZE(eeprom_pairs),
			     test_eeprom_wraparound);
	run_i2c_target_tests("ipmb", ipmb_pairs, ARRAY_SIZE(ipmb_pairs), test_ipmb);
	run_i2c_target_tests("ipmb-queue-full", ipmb_pairs, ARRAY_SIZE(ipmb_pairs),
			     test_ipmb_queue_full);
	run_i2c_target_tests("ipmb-queue-fifo", ipmb_pairs, ARRAY_SIZE(ipmb_pairs),
			     test_ipmb_queue_fifo);

	return ast_ztest_result();
}

#if !defined(AST1040_CONCURRENT_ALL)
ZTEST(i2c, test_i2c_all)
{
	zassert_equal(test_i2c(), AST_TEST_PASS, "i2c test failed");
}

ZTEST_SUITE(i2c, NULL, NULL, NULL, NULL, NULL);
#endif
