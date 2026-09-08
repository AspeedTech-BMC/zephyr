/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>
#include "ast_test.h"

#define NPWMS DT_PROP(DT_NODELABEL(pwm), npwms)

static const struct device *const pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm));

static const struct device *const fan_devs[NPWMS] = {
	DEVICE_DT_GET(DT_NODELABEL(fan0)),  DEVICE_DT_GET(DT_NODELABEL(fan1)),
	DEVICE_DT_GET(DT_NODELABEL(fan2)),  DEVICE_DT_GET(DT_NODELABEL(fan3)),
	DEVICE_DT_GET(DT_NODELABEL(fan4)),  DEVICE_DT_GET(DT_NODELABEL(fan5)),
	DEVICE_DT_GET(DT_NODELABEL(fan6)),  DEVICE_DT_GET(DT_NODELABEL(fan7)),
	DEVICE_DT_GET(DT_NODELABEL(fan8)),  DEVICE_DT_GET(DT_NODELABEL(fan9)),
	DEVICE_DT_GET(DT_NODELABEL(fan10)), DEVICE_DT_GET(DT_NODELABEL(fan11)),
	DEVICE_DT_GET(DT_NODELABEL(fan12)), DEVICE_DT_GET(DT_NODELABEL(fan13)),
	DEVICE_DT_GET(DT_NODELABEL(fan14)), DEVICE_DT_GET(DT_NODELABEL(fan15)),
};

/*
 * Golden RPM values are for the AFB0812SH fan wired to the last PWM/TACH
 * channel on the AST1030 EVB; owned by the test author, do not invent/adjust.
 */
#define AFB0812SH_FAN_ERROR 450
static const uint32_t afb0812sh_fan_rpm[5] = {
	4500, /* 100% */
	3400, /* 75% */
	2250, /* 50% */
	1100, /* 25% */
	700,  /* 0~17% */
};

static const uint32_t afb0812sh_fan_duty[5] = {
	100, 75, 50, 25, 0,
};

/*
 * PWM->TACH self-loopback: drive each PWM channel (all but the last, which
 * is reserved for the physical fan below) at a random period and confirm
 * the paired TACH channel measures the matching RPM.
 */
static void test_pwm_tach_loopback(void)
{
	for (int hw_pwm = 0; hw_pwm < NPWMS - 1; hw_pwm++) {
		uint64_t cycles_per_sec, cycles_per_min;
		uint32_t expected_rpm, rand_cycles;
		struct sensor_value sensor_value;

		ast_zassert_ok(pwm_get_cycles_per_sec(pwm_dev, hw_pwm, &cycles_per_sec),
			       "pwm_get_cycles_per_sec failed for channel %d", hw_pwm);
		cycles_per_min = cycles_per_sec * 60;

#if defined(CONFIG_PWM_ASPEED_ACCURATE_FREQ)
		uint32_t div_l = sys_rand32_get() % 256;
		uint32_t div_h = sys_rand32_get() % 4;

		rand_cycles = BIT(div_h) * (div_l + 1) * 256;
		if (rand_cycles < 2) {
			rand_cycles = 2;
		}
#else
		/*
		 * drivers/pwm/aspeed/pwm_aspeed.c rejects any period below
		 * PWM_ASPEED_FIXED_PERIOD + 1 (256) with -ENOTSUP. The old
		 * aspeed-dev-v2.6.0 test drew from [2, 2001] here, which hits
		 * that floor about 1 in 8 draws per channel - a real, if
		 * rare, driver-vs-test-range mismatch, not something specific
		 * to this port. Floor the range at 256 to stay clear of it.
		 */
		rand_cycles = (sys_rand32_get() % 2000) + 256;
#endif
		ast_zassert_ok(pwm_set_cycles(pwm_dev, hw_pwm, rand_cycles, rand_cycles / 2, 0),
			       "pwm_set_cycles failed for channel %d", hw_pwm);

		/* Needs one cycle time for period transition. */
		k_usleep((uint64_t)rand_cycles * USEC_PER_SEC / cycles_per_sec);

		expected_rpm = cycles_per_min / rand_cycles;

		ast_zassert_ok(sensor_sample_fetch(fan_devs[hw_pwm]),
			       "Failed to read tach channel %d", hw_pwm);
		sensor_channel_get(fan_devs[hw_pwm], SENSOR_CHAN_RPM, &sensor_value);

#if defined(CONFIG_PWM_ASPEED_ACCURATE_FREQ)
		ast_zassert_equal(sensor_value.val1, expected_rpm, "PWM%d(%d == %d)", hw_pwm,
				   sensor_value.val1, expected_rpm);
#else
		ast_zassert_true(sensor_value.val1 >= expected_rpm, "PWM%d(%d < %d)", hw_pwm,
				  sensor_value.val1, expected_rpm);
#endif
	}
}

static void test_pwm_tach_fan(void)
{
	const int hw_pwm = NPWMS - 1;
	const uint32_t period_ns = 40000;

	for (int i = 0; i < ARRAY_SIZE(afb0812sh_fan_rpm); i++) {
		struct sensor_value sensor_value;

		ast_zassert_ok(pwm_set(pwm_dev, hw_pwm, period_ns,
					period_ns / 100 * afb0812sh_fan_duty[i], 0),
			       "pwm_set failed for fan channel");

		/* Wait for the fan to stabilize. */
		k_msleep(3000);

		ast_zassert_ok(sensor_sample_fetch(fan_devs[hw_pwm]), "Failed to read fan tach");
		sensor_channel_get(fan_devs[hw_pwm], SENSOR_CHAN_RPM, &sensor_value);

		ast_zassert_within(sensor_value.val1, afb0812sh_fan_rpm[i], AFB0812SH_FAN_ERROR,
				    "%d != %d+-%d", sensor_value.val1, afb0812sh_fan_rpm[i],
				    AFB0812SH_FAN_ERROR);
	}
}

int test_pwm_tach(void)
{
	ast_zassert_true(device_is_ready(pwm_dev), "PWM device is not ready");
	for (int i = 0; i < NPWMS; i++) {
		ast_zassert_true(device_is_ready(fan_devs[i]), "fan device %d is not ready", i);
	}

	test_pwm_tach_loopback();
	test_pwm_tach_fan();

	return ast_ztest_result();
}

#if !defined(AST1030_CONCURRENT_ALL)
ZTEST(pwm_tach, test_pwm_tach_all)
{
	zassert_equal(test_pwm_tach(), AST_TEST_PASS, "pwm_tach test failed");
}

ZTEST_SUITE(pwm_tach, NULL, NULL, NULL, NULL, NULL);
#endif
