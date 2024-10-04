/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_tach

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>
#include "tach_aspeed.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tach_aspeed);

static const struct pinctrl_dev_config *pcfg;

struct tach_aspeed_cfg {
struct tach_register_s *base;
	const struct device *clock_dev;
	const clock_control_subsys_t clk_id;
	const struct reset_dt_spec reset;
	uint8_t tach_ch;
	uint8_t pulse_pr;
	uint8_t tach_mode;
	uint8_t tach_div;
	uint32_t min_rpm;
};

struct tach_aspeed_data {
	uint32_t count;
	uint32_t tach_freq;
	uint32_t sample_period;
};

#define DEV_CFG(dev)  ((const struct tach_aspeed_cfg *const)(dev)->config)
#define DEV_DATA(dev) ((struct tach_aspeed_data *)(dev)->data)

static int tach_aspeed_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct tach_register_s *tach_reg = DEV_CFG(dev)->base;
	union tach_general_register_s tach_general;
	union tach_status_register_s tach_status;
	int ret;

	ARG_UNUSED(chan);
	/* Restart the Tach channel to guarantee the value is fresh */
	tach_general.value = tach_reg->tach_general.value;
	tach_general.fields.enable_tach = 0;
	tach_reg->tach_general.value = tach_general.value;
	tach_general.fields.enable_tach = 1;
	tach_reg->tach_general.value = tach_general.value;
	ret = reg_read_poll_timeout(tach_reg, tach_status, tach_status,
				    tach_status.fields.tach_full_measurement, 1,
				    DEV_DATA(dev)->sample_period);
	if (ret) {
		DEV_DATA(dev)->count = 0;
	} else {
		DEV_DATA(dev)->count = (tach_status.fields.tach_value + 1) & 0xfffff;
	}

	return 0;
}

static int tach_aspeed_channel_get(const struct device *dev, enum sensor_channel chan,
				   struct sensor_value *val)
{
	uint32_t tach_count = DEV_DATA(dev)->count;
	uint32_t tach_freq = DEV_DATA(dev)->tach_freq;
	uint8_t pulse_pr = DEV_CFG(dev)->pulse_pr;
	/*
	 * We need the mode to determine if the raw_data is double (from
	 * counting both edges).
	 */
	uint8_t both = (DEV_CFG(dev)->tach_mode == 0x2) ? 1 : 0;

	if (chan != SENSOR_CHAN_RPM) {
		return -ENOTSUP;
	}

	if (tach_count > 0) {
		tach_count <<= both;
		/*
		 * RPM = (f * 60) / (n * TACH)
		 *   n = Pulses per round
		 *   f = Tachometer operation freqency (Hz)
		 *   TACH = Captured counts of tachometer
		 */
		if (tach_freq > (0xffffffff / 60)) {
			val->val1 = ((tach_freq) / (pulse_pr * tach_count)) * 60;
		} else {
			val->val1 = ((tach_freq * 60) / (pulse_pr * tach_count));
		}
	} else {
		val->val1 = 0U;
	}

	LOG_INF("tach_freq = %d %d", tach_freq, val->val1);

	val->val2 = 0U;

	return 0;
}

static int tach_aspeed_get_sample_period(const struct device *dev)
{
	uint32_t sample_period_ms;
	uint32_t min_rpm = DEV_CFG(dev)->min_rpm;
	uint8_t pulse_pr = DEV_CFG(dev)->pulse_pr;

	/*
	 * min(Tach input clock) = (PulsePR * minRPM) / 60
	 * max(Tach input period) = 60 / (PulsePR * minRPM)
	 * Tach sample period > 2 * max(Tach input period) = (2*60) / (PulsePR * minRPM)
	 */
	sample_period_ms = DIV_ROUND_UP(1000 * 2 * 60, (pulse_pr * min_rpm));
	/* Add the margin (about 1.2) of tach sample period to avoid sample miss */
	sample_period_ms = (sample_period_ms * 1200) >> 10;
	LOG_DBG("sample period = %d ms", sample_period_ms);
	DEV_DATA(dev)->sample_period = sample_period_ms;
	return 0;
}

static int tach_aspeed_get_tach_freq(const struct device *dev)
{
	uint32_t clk_src;

	clock_control_get_rate(DEV_CFG(dev)->clock_dev, DEV_CFG(dev)->clk_id, &clk_src);
	/* divide = 2^(tacho_div*2) */
	DEV_DATA(dev)->tach_freq = clk_src / (1 << (DEV_CFG(dev)->tach_div << 1));
	return 0;
}

static int tach_aspeed_init(const struct device *dev)
{
	struct tach_register_s *tach_reg = DEV_CFG(dev)->base;
	union tach_general_register_s tach_general;
	int ret;

	ret = pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}
	reset_line_deassert_dt(&DEV_CFG(dev)->reset);
	tach_aspeed_get_sample_period(dev);
	tach_aspeed_get_tach_freq(dev);
	tach_general.value = tach_reg->tach_general.value;
	tach_general.fields.tach_clock_division = DEV_CFG(dev)->tach_div;
	tach_general.fields.tach_edge = DEV_CFG(dev)->tach_mode;
	tach_general.fields.enable_tach = 0x1;
	tach_reg->tach_general.value = tach_general.value;
	return 0;
}

static const struct sensor_driver_api tach_aspeed_api = {
	.sample_fetch = tach_aspeed_sample_fetch,
	.channel_get = tach_aspeed_channel_get,
};

#define TACH_ENUM(node_id) node_id,

#define TACH_ASPEED_DEV_CFG(node_id)                                                               \
	{                                                                                          \
		.base = (struct tach_register_s *)(DT_REG_ADDR(DT_GPARENT(node_id)) +              \
						   0x10 * DT_REG_ADDR(node_id)),                   \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_PARENT(node_id))),                    \
		.clk_id = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_PARENT(node_id), clk_id),      \
		.reset = RESET_DT_SPEC_GET(DT_PARENT(node_id)),                                    \
		.tach_ch = DT_REG_ADDR(node_id),                                                   \
		.pulse_pr = DT_PROP(node_id, pulse_pr),                                            \
		.min_rpm = DT_PROP(node_id, min_rpm),                                              \
		.tach_mode = DT_PROP(node_id, tach_mode),                                          \
		.tach_div = DT_PROP(node_id, tach_div),                                            \
	},

#define TACH_ASPEED_DEV_DATA(node_id) {},

#define TACH_ASPEED_DT_DEFINE(node_id)                                                             \
	DEVICE_DT_DEFINE(node_id, tach_aspeed_init, NULL, &tach_aspeed_data[node_id],              \
			 &tach_aspeed_cfg[node_id], POST_KERNEL,                                   \
			 CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &tach_aspeed_api);

#define TACH_ASPEED_INIT(n)                                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct pinctrl_dev_config *pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n);          \
	static struct tach_aspeed_data tach_aspeed_data[] = {                                      \
		DT_FOREACH_CHILD(DT_DRV_INST(n), TACH_ASPEED_DEV_DATA)};                           \
	static const struct tach_aspeed_cfg tach_aspeed_cfg[] = {                                  \
		DT_FOREACH_CHILD(DT_DRV_INST(n), TACH_ASPEED_DEV_CFG)};                            \
	enum {                                                                                     \
		DT_FOREACH_CHILD(DT_DRV_INST(n), TACH_ENUM)                                        \
	};                                                                                         \
	DT_FOREACH_CHILD(DT_DRV_INST(n), TACH_ASPEED_DT_DEFINE)

DT_INST_FOREACH_STATUS_OKAY(TACH_ASPEED_INIT)
