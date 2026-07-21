/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>
#include "ast_test.h"

#define ASPEED_ADC_CH_NUMBER 8

static const struct device *const adc_devs[] = {
	DEVICE_DT_GET(DT_NODELABEL(adc0)),
	DEVICE_DT_GET(DT_NODELABEL(adc1)),
};

/*
 * Golden millivolt values are wired on the AST1030 EVB via fixed voltage
 * dividers per ADC channel; owned by the test author, do not invent/adjust.
 */
static const uint16_t ci_golden_value[ARRAY_SIZE(adc_devs) * ASPEED_ADC_CH_NUMBER] = {
	605, 1190, 1812, 605, 1006, 1812, 1006, 3022,
	605, 1190, 1812, 605, 1006, 1812, 1006, 3022,
};

/* double, not float: %f varargs promote to double anyway. */
#define TOLERANCE_ERROR_RATE 0.03

/*
 * adc_get_ref() has no public wrapper in <zephyr/drivers/adc.h> on this
 * Zephyr version, even though the vtable slot and the aspeed driver's
 * implementation still exist; call through dev->api directly instead of
 * patching the shared subsystem header.
 */
static uint16_t adc_ref_mv(const struct device *dev)
{
	const struct adc_driver_api *api = dev->api;

	return api->get_ref(dev);
}

static void test_adc_normal_mode(void)
{
	/*
	 * Zero-init: this struct has fields (e.g. .differential) this test
	 * never sets. The old v2.6.0 code got zero-init for free from static
	 * storage duration; as a local here it must be explicit, or the
	 * driver reads garbage and rejects the config.
	 */
	struct adc_channel_cfg channel_config = {0};
	uint16_t sample_buffer[ASPEED_ADC_CH_NUMBER];
	const struct adc_sequence sequence = {
		.channels = GENMASK(6, 0),
		.buffer = sample_buffer,
		.buffer_size = sizeof(sample_buffer),
		.resolution = 10,
	};

	for (int i = 0; i < ARRAY_SIZE(adc_devs); i++) {
		for (int ch = 0; ch < ASPEED_ADC_CH_NUMBER; ch++) {
			channel_config.channel_id = ch;
			channel_config.reference = ADC_REF_INTERNAL;
			channel_config.gain = ADC_GAIN_1;
			channel_config.acquisition_time = ADC_ACQ_TIME_DEFAULT;
			ast_zassert_ok(adc_channel_setup(adc_devs[i], &channel_config),
				       "adc_channel_setup failed for dev %d ch %d", i, ch);
		}

		ast_zassert_ok(adc_read(adc_devs[i], &sequence), "adc_read failed for dev %d", i);

		for (int ch = 0; ch < 7; ch++) {
			int32_t val = sample_buffer[ch];

			adc_raw_to_millivolts(adc_ref_mv(adc_devs[i]), ADC_GAIN_1, 10, &val);
			ast_zassert_within(val, ci_golden_value[i * ASPEED_ADC_CH_NUMBER + ch],
					    adc_ref_mv(adc_devs[i]) * TOLERANCE_ERROR_RATE,
					    "dev %d ch %d: %dmv(raw:%d) check failed", i, ch, val,
					    sample_buffer[ch]);
		}
	}
}

static void test_adc_battery_mode(void)
{
	struct adc_channel_cfg channel_config = {0};
	uint16_t sample_buffer[1];
	const struct adc_sequence sequence = {
		.channels = BIT(7),
		.buffer = sample_buffer,
		.buffer_size = sizeof(sample_buffer),
		.resolution = 10,
	};

	for (int i = 0; i < ARRAY_SIZE(adc_devs); i++) {
		int16_t ref = adc_ref_mv(adc_devs[i]);

		channel_config.channel_id = 7;
		channel_config.reference = ADC_REF_INTERNAL;
		channel_config.gain = (ref < 1550) ? ADC_GAIN_1_3 : ADC_GAIN_2_3;
		channel_config.acquisition_time = ADC_ACQ_TIME_DEFAULT;
		ast_zassert_ok(adc_channel_setup(adc_devs[i], &channel_config),
			       "adc_channel_setup failed for dev %d ch 7", i);

		ast_zassert_ok(adc_read(adc_devs[i], &sequence), "adc_read failed for dev %d", i);

		int32_t val = sample_buffer[0];

		adc_raw_to_millivolts(ref, channel_config.gain, 10, &val);
		ast_zassert_within(val, ci_golden_value[i * ASPEED_ADC_CH_NUMBER + 7],
				    ref * TOLERANCE_ERROR_RATE,
				    "dev %d ch 7: %dmv(raw:%d) check failed", i, val,
				    sample_buffer[0]);
	}
}

int test_adc(void)
{
	for (int i = 0; i < ARRAY_SIZE(adc_devs); i++) {
		ast_zassert_true(device_is_ready(adc_devs[i]), "ADC device %d is not ready", i);
	}

	test_adc_normal_mode();
	test_adc_battery_mode();

	return ast_ztest_result();
}

#if !defined(AST1030_CONCURRENT_ALL)
ZTEST(adc, test_adc_all)
{
	zassert_equal(test_adc(), AST_TEST_PASS, "adc test failed");
}

ZTEST_SUITE(adc, NULL, NULL, NULL, NULL, NULL);
#endif
