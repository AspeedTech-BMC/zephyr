/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast1030_pwm

#define LOG_LEVEL CONFIG_PWM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(pwm_aspeed);

#include <stdlib.h>
#include <sys/sys_io.h>
#include <device.h>
#include <drivers/pwm.h>
#include "soc.h"
#include <drivers/clock_control.h>
#include <drivers/reset_control.h>
#include "pwm_aspeed.h"

#define NUM_OF_CHANNELS DT_INST_PROP(0, npwms)


/* Structure Declarations */

struct pwm_aspeed_data {
	uint32_t clk_src;
	uint32_t freq[NUM_OF_CHANNELS];
};

struct pwm_aspeed_cfg {
	pwm_register_t * base;
	const clock_control_subsys_t clk_id;
	const reset_control_subsys_t rst_id;
};

#define DEV_CFG(dev)					\
	((const struct pwm_aspeed_cfg * const)	\
	 (dev)->config)

#define DEV_DATA(dev)					\
	((struct pwm_aspeed_data *)	\
	 (dev)->data)

/* API Functions */

static int pwm_aspeed_init(const struct device *dev)
{
	const struct device *clock_dev = device_get_binding(ASPEED_CLK_CTRL_NAME);
	const struct device *reset_dev = device_get_binding(ASPEED_RST_CTRL_NAME);
	clock_control_get_rate(clock_dev, DEV_CFG(dev)->clk_id, &DEV_DATA(dev)->clk_src);
	reset_control_deassert(reset_dev, DEV_CFG(dev)->rst_id);
	return 0;
}

static void aspeed_set_pwm_channel_enable(const struct device *dev, uint32_t pwm,
					  bool enable)
{
	volatile pwm_register_t * pwm_reg = DEV_CFG(dev)->base;
	pwm_general_register_t general_reg;
	general_reg.value = pwm_reg->pwm_gather[pwm].pwm_general.value;
	general_reg.fields.enable_pwm_pin = enable;
	general_reg.fields.enable_pwm_clock = enable;
	pwm_reg->pwm_gather[pwm].pwm_general.value = general_reg.value;
}

static void aspeed_set_pwm_freq(const struct device *dev,
				uint32_t pwm, uint32_t period_cycles)
{
	volatile pwm_register_t * pwm_reg = DEV_CFG(dev)->base;
	struct pwm_aspeed_data *priv = DEV_DATA(dev);
	pwm_duty_cycle_register_t duty_reg;
	pwm_general_register_t general_reg;
	uint32_t target_div;
	int diff, min_diff = INT_MAX;
	uint32_t tmp_div_h, tmp_div_l;
	uint32_t div_h = BIT(5) - 1, div_l = BIT(8) - 1;

	target_div = period_cycles >> 8;
	/* calculate for target frequence */
	for (tmp_div_h = 0; tmp_div_h < 0x10; tmp_div_h++) {
		tmp_div_l = target_div / BIT(tmp_div_h) - 1;

		if (tmp_div_l < 0 || tmp_div_l > 255)
			continue;

		diff = target_div - (BIT(tmp_div_h) * (tmp_div_l + 1));
		if (abs(diff) < abs(min_diff)) {
			min_diff = diff;
			div_l = tmp_div_l;
			div_h = tmp_div_h;
			if (diff == 0)
				break;
		}
	}
	/*
	* The PWM frequency = PCLK(200Mhz) / (clock division L bit *
	* clock division H bit * (period bit(0xff) + 1))
	*/
	priv->freq[pwm] = (priv->clk_src >> 8) / (BIT(div_h) * (div_l + 1));
	LOG_DBG("div h %x, l : %x pwm out clk %d\n", div_h, div_l,
		 priv->freq[pwm]);
	duty_reg.value = pwm_reg->pwm_gather[pwm].pwm_duty_cycle.value;
	duty_reg.fields.pwm_period = 0xff;
	pwm_reg->pwm_gather[pwm].pwm_duty_cycle.value = duty_reg.value;

	general_reg.value = pwm_reg->pwm_gather[pwm].pwm_general.value;
	general_reg.fields.pwm_clock_division_h = div_h;
	general_reg.fields.pwm_clock_division_l = div_l;
	pwm_reg->pwm_gather[pwm].pwm_general.value = general_reg.value;
	LOG_DBG("pwm general 0x%08x pwm_duty 0x%08x",
		pwm_reg->pwm_gather[pwm].pwm_general.value,
		pwm_reg->pwm_gather[pwm].pwm_duty_cycle.value);
}

static void aspeed_set_pwm_duty(const struct device *dev,
				uint32_t pwm, uint32_t duty_pt)
{
	volatile pwm_register_t * pwm_reg = DEV_CFG(dev)->base;
	pwm_duty_cycle_register_t duty_reg;
	if (duty_pt == 0) {
		aspeed_set_pwm_channel_enable(dev, pwm, false);
	} else {
		duty_reg.value = pwm_reg->pwm_gather[pwm].pwm_duty_cycle.value;
		duty_reg.fields.pwm_falling_point = duty_pt;
		pwm_reg->pwm_gather[pwm].pwm_duty_cycle.value = duty_reg.value;
		aspeed_set_pwm_channel_enable(dev, pwm, true);
	}
	LOG_DBG("pwm general 0x%08x pwm_duty 0x%08x",
		pwm_reg->pwm_gather[pwm].pwm_general.value,
		pwm_reg->pwm_gather[pwm].pwm_duty_cycle.value);
}

static void aspeed_set_pwm_polarity(const struct device *dev,
				    uint32_t pwm, uint8_t polarity)
{
	volatile pwm_register_t * pwm_reg = DEV_CFG(dev)->base;
	pwm_general_register_t general_reg;
	general_reg.value = pwm_reg->pwm_gather[pwm].pwm_general.value;
	general_reg.fields.inverse_pwm_pin = polarity;
	pwm_reg->pwm_gather[pwm].pwm_general.value = general_reg.value;
}

static int pwm_aspeed_pin_set(const struct device *dev,
			      uint32_t pwm,
			      uint32_t period_cycles,
			      uint32_t pulse_cycles,
			      pwm_flags_t flags)
{
	struct pwm_aspeed_data *priv = DEV_DATA(dev);
	uint32_t cycles_max = priv->clk_src;

	uint32_t duty_pt = DIV_ROUND_UP(pulse_cycles * 256, period_cycles);

	if (period_cycles > cycles_max) {
		LOG_ERR("Requested period cycles is %d but maximum cycles is %d\n",
			period_cycles, cycles_max);
		return -EIO;
	}

	if (pulse_cycles > period_cycles) {
		LOG_ERR("Requested pulse %d is longer than period %d\n",
			pulse_cycles, period_cycles);
		return -EIO;
	}

	LOG_DBG("duty_pt: %d", duty_pt);
	aspeed_set_pwm_freq(dev, pwm, period_cycles);
	aspeed_set_pwm_duty(dev, pwm, duty_pt);
	aspeed_set_pwm_polarity(dev, pwm, flags);
	return 0;
}

static int pwm_aspeed_get_cycles_per_sec(const struct device *dev,
					 uint32_t pwm,
					 uint64_t *cycles)
{
	struct pwm_aspeed_data *priv = DEV_DATA(dev);
	*cycles = priv->freq[pwm];
	return 0;
}

/* Device Instantiation */

static const struct pwm_driver_api pwm_aspeed_api = {
	.pin_set = pwm_aspeed_pin_set,
	.get_cycles_per_sec = pwm_aspeed_get_cycles_per_sec,
};

#define PWM_ASPEED_INIT(n)	\
	static struct pwm_aspeed_data pwm_aspeed_data_##n; \
	static const struct pwm_aspeed_cfg pwm_aspeed_cfg_##n = {	\
			.base = (pwm_register_t *)DT_REG_ADDR(DT_PARENT(DT_DRV_INST(n))),	\
			.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, clk_id), \
			.rst_id = (reset_control_subsys_t)DT_INST_RESETS_CELL(n, rst_id), \
		};	\
	DEVICE_DT_INST_DEFINE(n,	\
			    pwm_aspeed_init,	\
			    device_pm_control_nop,	\
			    &pwm_aspeed_data_##n,	\
			    &pwm_aspeed_cfg_##n,	\
			     PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			    &pwm_aspeed_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_ASPEED_INIT)
