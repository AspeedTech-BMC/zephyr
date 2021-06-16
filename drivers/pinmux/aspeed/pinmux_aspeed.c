/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_pinmux

#include <drivers/pinmux.h>
#include <kernel.h>

#define LOG_LEVEL CONFIG_PINMUX_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(pimux_aspeed);
#include "pinmux_aspeed.h"

#ifdef CONFIG_PINCTRL_STRING_NAME

#define PIN_DEFINE(pin)	\
	[pin] = # pin,
const char *aspeed_pin_name[] =
{
    #include "pin_def_list.h"
};
#undef PIN_DEFINE

#define FUN_DEFINE(fun, ...) \
	[fun] = # fun,
const char *aspeed_fun_name[] =
{
    #include "fun_def_list.h"
};
#undef FUN_DEFINE

#define SIG_DEFINE(sig, ...) \
	[sig] = # sig,
const char *aspeed_sig_name[] =
{
    #include "sig_def_list.h"
};
#undef SIG_DEFINE

#endif

#define SIG_DEFINE(sig, pin, ...) SIG_DECL(sig, pin, __VA_ARGS__);
#include "sig_def_list.h"
#undef SIG_DEFINE

#define FUN_DEFINE(fun, ...) FUN_DECL(fun, __VA_ARGS__);
#include "fun_def_list.h"
#undef FUN_DEFINE

#define PIN_DEFINE(pin)	\
	[pin] = 0xffffffff,
uint32_t aspeed_pin_desc_table[] =
{
    #include "pin_def_list.h"
};
#undef PIN_DEFINE

#define SIG_DEFINE(sig, ...) \
	[sig] = SIG_SYM_PTR(sig),
const struct aspeed_sig_desc *aspeed_sig_desc_table[] =
{
	#include "sig_def_list.h"
};
#undef SIG_DEFINE

#define FUN_DEFINE(fun, ...) \
	[fun] = FUN_SYM_PTR(fun),
const struct aspeed_fun_desc *aspeed_fun_desc_table[] =
{
	#include "fun_def_list.h"
};
#undef FUN_DEFINE

struct pinmux_aspeed_config {
	uintptr_t base;
};

#define DEV_CFG(dev)				    \
	((const struct pinmux_aspeed_config *const) \
	 (dev)->config)

/**
 * @brief Get the signal id of the occupied pin
 * @param pin: Pin number.
 * @param func: Pointer to a variable where signal id will be stored.
 * @retval 0 If successful.
 * @retval -EINVAL  Invalid pin.
 */
static int pinmux_aspeed_get(const struct device *dev, uint32_t pin,
			     uint32_t *func)
{
	ARG_UNUSED(dev);
	if (pin >= MAX_PIN_ID || pin < 0) {
		return -EINVAL;
	}
	*func = aspeed_pin_desc_table[pin];
	return 0;
}

/**
 * @brief Request the signal.
 * @param pin: Pin number.
 * @param func: Signal id which want to apply to the pin
 * @retval 0 If successful.
 * @retval -EINVAL  Invalid pin.
 */
static int pinmux_aspeed_set(const struct device *dev, uint32_t pin,
			     uint32_t func)
{
	uint32_t scu_base = DEV_CFG(dev)->base;
	const struct aspeed_sig_desc *sig_desc;
	const struct aspeed_sig_en *sig_en;
	uint32_t ret_sig_id;
	int sig_en_number;
	int sig_en_idx;

	if (func >=
	    sizeof(aspeed_sig_desc_table) / sizeof(aspeed_sig_desc_table[0])) {
		return -EINVAL;
	}
	sig_desc = aspeed_sig_desc_table[func];
	if (sig_desc == NULL) {
		return -EINVAL;
	}

	if (pin != sig_desc->pin) {
		return -EINVAL;
	}

	pinmux_aspeed_get(dev, pin, &ret_sig_id);
	if (ret_sig_id == 0xffffffff) {
		aspeed_pin_desc_table[pin] = func;
#ifdef CONFIG_PINCTRL_STRING_NAME
		LOG_DBG("The pin %s is being occupied by signal %s\n",
			aspeed_pin_name[pin], aspeed_sig_name[func]);
#else
		LOG_DBG("The pin %d is being occupied by signal %d\n", pin, func);
#endif
		sig_en_number = sig_desc->nsig_en;
		for (sig_en_idx = 0; sig_en_idx < sig_en_number; sig_en_idx++) {
			sig_en = &sig_desc->sig_en[sig_en_idx];
			if (sig_en->op) {
				sys_write32(
					(sys_read32(scu_base + sig_en->offset) &
					 ~BIT(sig_en->bits)),
					scu_base + sig_en->offset);
			} else {
				sys_write32(
					(sys_read32(scu_base + sig_en->offset) |
					 BIT(sig_en->bits)),
					scu_base + sig_en->offset);
			}
		}
		return 0;
	} else {
#ifdef CONFIG_PINCTRL_STRING_NAME
		LOG_ERR("Request %s to pin %s error: already occupied by signal %s\n",
			aspeed_sig_name[func], aspeed_pin_name[pin],
			aspeed_sig_name[ret_sig_id]);
#else
		LOG_ERR("Request %d to pin %d error: already occupied by signal %d\n",
			func, pin, ret_sig_id);
#endif
		return -EBUSY;
	}
}

static int pinmux_aspeed_pullup(const struct device *dev,
				uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_aspeed_input(const struct device *dev,
			       uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

/**
 * @brief Request all of the signal in the function group.
 * @param fun_id: Function group id.
 * @retval 0 If successful.
 * @retval -EINVAL  Invalid fun_id.
 */
static int aspeed_pinctrl_fn_group_request(const struct device *dev, uint32_t fun_id)
{
	const struct aspeed_fun_desc *fun_desc;
	const struct aspeed_sig_desc *sig_desc;
	int sig_number;
	int sig_idx;
	uint16_t sig_id;
	int ret;

	fun_desc = aspeed_fun_desc_table[fun_id];
	if (fun_desc == NULL) {
		LOG_ERR("Invalid argument %d", fun_id);
		return -EINVAL;
	}
	sig_number = fun_desc->nsignal;
	for (sig_idx = 0; sig_idx < sig_number; sig_idx++) {
		sig_id = fun_desc->sig_id_list[sig_idx];
		sig_desc = aspeed_sig_desc_table[sig_id];
		ret |= pinmux_aspeed_set(dev, sig_desc->pin, sig_id);
	}
	return 0;
}

static int pinmux_aspeed_init(const struct device *dev)
{
	uint32_t fun_id;

	for (fun_id = 0; fun_id < MAX_FUN_ID; fun_id++) {
		aspeed_pinctrl_fn_group_request(dev, fun_id);
	}
	return 0;
}

static const struct pinmux_driver_api pinmux_aspeed_driver_api = {
	.set = pinmux_aspeed_set,
	.get = pinmux_aspeed_get,
	.pullup = pinmux_aspeed_pullup,
	.input = pinmux_aspeed_input,
};

static const struct pinmux_aspeed_config pinmux_aspeed_config = {
	.base = DT_REG_ADDR(DT_NODELABEL(syscon)),
};


DEVICE_DT_INST_DEFINE(0, &pinmux_aspeed_init, device_pm_control_nop,
		      NULL, &pinmux_aspeed_config,
		      PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY,
		      &pinmux_aspeed_driver_api);
