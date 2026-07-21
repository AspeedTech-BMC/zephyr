/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/target/eeprom.h>
#include <zephyr/drivers/i2c/target/ipmb.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>
#include "ast_test.h"

#define DATA_COUNT 0x20
#define RETRY 0x5

static const uint32_t i2c_speed[] = {
	I2C_SPEED_STANDARD,
	I2C_SPEED_FAST,
	I2C_SPEED_FAST_PLUS,
};

struct i2c_target_pair {
	const struct device *master;
	const struct device *target;
	uint8_t target_addr;
};

static const struct i2c_target_pair eeprom_pairs[] = {
	{DEVICE_DT_GET(DT_NODELABEL(i2c0)), DEVICE_DT_GET(DT_NODELABEL(eeprom1)), 0x41},
	{DEVICE_DT_GET(DT_NODELABEL(i2c2)), DEVICE_DT_GET(DT_NODELABEL(eeprom3)), 0x43},
	{DEVICE_DT_GET(DT_NODELABEL(i2c4)), DEVICE_DT_GET(DT_NODELABEL(eeprom5)), 0x45},
};

static const struct i2c_target_pair ipmb_pairs[] = {
	{DEVICE_DT_GET(DT_NODELABEL(i2c1)), DEVICE_DT_GET(DT_NODELABEL(ipmb0)), 0x50},
	{DEVICE_DT_GET(DT_NODELABEL(i2c3)), DEVICE_DT_GET(DT_NODELABEL(ipmb2)), 0x52},
	{DEVICE_DT_GET(DT_NODELABEL(i2c5)), DEVICE_DT_GET(DT_NODELABEL(ipmb4)), 0x54},
};

static void prepare_test_data(uint8_t *data, int nbytes)
{
	uint32_t value = sys_rand32_get();
	uint32_t shift;

	for (int i = 0; i < nbytes; i++) {
		shift = (i & 0x3) * 8;
		data[i] = (value >> shift) & 0xff;
		if ((i & 0x3) == 0x3) {
			value = sys_rand32_get();
		}
	}
}

static void test_i2c_target_eeprom(void)
{
	for (int i = 0; i < ARRAY_SIZE(eeprom_pairs); i++) {
		const struct device *master = eeprom_pairs[i].master;
		const struct device *target = eeprom_pairs[i].target;
		uint8_t dev_addr = eeprom_pairs[i].target_addr;
		uint8_t data_s[DATA_COUNT];
		uint8_t data_r[DATA_COUNT];
		uint8_t retry_count = RETRY;
		int result;

		ast_zassert_ok(i2c_configure(master, I2C_MODE_CONTROLLER |
					      I2C_SPEED_SET(i2c_speed[i % 3])),
			       "i2c_configure failed for pair %d", i);

		ast_zassert_ok(i2c_target_driver_register(target),
			       "i2c_target_driver_register failed for pair %d", i);

		prepare_test_data(data_s, DATA_COUNT);

		while (retry_count != 0) {
			result = i2c_burst_write(master, dev_addr, 0, data_s, DATA_COUNT);
			if (result == -ETIMEDOUT) {
				retry_count--;
			} else {
				break;
			}
		}
		ast_zassert_ok(result, "eeprom pair %d write failed: %d", i, result);
		if (result != 0) {
			ast_zassert_ok(i2c_target_driver_unregister(target),
				       "i2c_target_driver_unregister failed for pair %d", i);
			continue;
		}

		retry_count = RETRY;
		while (retry_count != 0) {
			result = i2c_burst_read(master, dev_addr, 0, data_r, DATA_COUNT);
			if (result == -ETIMEDOUT) {
				retry_count--;
			} else {
				break;
			}
		}
		ast_zassert_ok(result, "eeprom pair %d read failed: %d", i, result);
		if (result != 0) {
			ast_zassert_ok(i2c_target_driver_unregister(target),
				       "i2c_target_driver_unregister failed for pair %d", i);
			continue;
		}

		ast_zassert_mem_equal(data_s, data_r, DATA_COUNT, "eeprom pair %d R/W mismatch",
				       i);

		ast_zassert_ok(i2c_target_driver_unregister(target),
			       "i2c_target_driver_unregister failed for pair %d", i);
	}
}

static void test_i2c_target_ipmb(void)
{
	for (int i = 0; i < ARRAY_SIZE(ipmb_pairs); i++) {
		const struct device *master = ipmb_pairs[i].master;
		const struct device *target = ipmb_pairs[i].target;
		uint8_t dev_addr = ipmb_pairs[i].target_addr;
		uint8_t data_s[DATA_COUNT];
		uint8_t retry_count = RETRY;
		struct ipmb_msg *msg = NULL;
		uint8_t length = 0;
		uint8_t *buf;
		int result;

		ast_zassert_ok(i2c_configure(master, I2C_MODE_CONTROLLER |
					      I2C_SPEED_SET(i2c_speed[i % 3])),
			       "i2c_configure failed for pair %d", i);

		ast_zassert_ok(i2c_target_driver_register(target),
			       "i2c_target_driver_register failed for pair %d", i);

		prepare_test_data(data_s, DATA_COUNT);

		while (retry_count != 0) {
			result = i2c_burst_write(master, dev_addr, 0, data_s, DATA_COUNT);
			if (result == -ETIMEDOUT) {
				retry_count--;
			} else {
				break;
			}
		}
		ast_zassert_ok(result, "ipmb pair %d write failed: %d", i, result);
		if (result != 0) {
			ast_zassert_ok(i2c_target_driver_unregister(target),
				       "i2c_target_driver_unregister failed for pair %d", i);
			continue;
		}

		retry_count = RETRY;
		while (retry_count != 0) {
			result = ipmb_target_read(target, &msg, &length);
			if (result == -ETIMEDOUT) {
				retry_count--;
			} else {
				break;
			}
		}
		ast_zassert_ok(result, "ipmb pair %d read failed: %d", i, result);
		if (result != 0) {
			ast_zassert_ok(i2c_target_driver_unregister(target),
				       "i2c_target_driver_unregister failed for pair %d", i);
			continue;
		}

		ast_zassert_not_null(msg, "ipmb pair %d returned a NULL message", i);
		if (msg == NULL) {
			ast_zassert_ok(i2c_target_driver_unregister(target),
				       "i2c_target_driver_unregister failed for pair %d", i);
			continue;
		}

		ast_zassert_equal(length, DATA_COUNT + 2, "ipmb pair %d length mismatch: %d", i,
				   length);
		if (length != DATA_COUNT + 2) {
			ast_zassert_ok(i2c_target_driver_unregister(target),
				       "i2c_target_driver_unregister failed for pair %d", i);
			continue;
		}

		buf = (uint8_t *)msg;
		ast_zassert_equal(buf[0], dev_addr << 1, "ipmb pair %d device id mismatch: %d", i,
				   buf[0]);

		for (int j = 2; j < length; j++) {
			ast_zassert_equal(data_s[j - 2], buf[j],
					   "ipmb pair %d R/W mismatch at %d", i, j);
		}

		ast_zassert_ok(i2c_target_driver_unregister(target),
			       "i2c_target_driver_unregister failed for pair %d", i);
	}
}

int test_i2c(void)
{
	for (int i = 0; i < ARRAY_SIZE(eeprom_pairs); i++) {
		ast_zassert_true(device_is_ready(eeprom_pairs[i].master),
				  "eeprom pair %d master not ready", i);
		ast_zassert_true(device_is_ready(eeprom_pairs[i].target),
				  "eeprom pair %d target not ready", i);
	}
	for (int i = 0; i < ARRAY_SIZE(ipmb_pairs); i++) {
		ast_zassert_true(device_is_ready(ipmb_pairs[i].master),
				  "ipmb pair %d master not ready", i);
		ast_zassert_true(device_is_ready(ipmb_pairs[i].target),
				  "ipmb pair %d target not ready", i);
	}

	test_i2c_target_eeprom();
	test_i2c_target_ipmb();

	return ast_ztest_result();
}

#if !defined(AST1030_CONCURRENT_ALL)
ZTEST(i2c, test_i2c_all)
{
	zassert_equal(test_i2c(), AST_TEST_PASS, "i2c test failed");
}

ZTEST_SUITE(i2c, NULL, NULL, NULL, NULL, NULL);
#endif
