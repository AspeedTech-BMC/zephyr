/*
 * Copyright (c) 2025 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "zephyr/sys/util.h"
#define DT_DRV_COMPAT aspeed_g7_soc1_pinctrl

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/dt-bindings/pinctrl/ast27xx-soc1-pinctrl.h>
#include <pinctrl_soc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pinctrl_aspeed_g7_soc1, LOG_LEVEL_INF);

#define PINCTRL_SIG_DESC_FUNC_IDX	GENMASK(31, 28)
#define PINCTRL_SIG_DESC_BIT_OFFSET	GENMASK(27, 20)
#define PINCTRL_SIG_DESC_REG_OFFSET	GENMASK(19, 0)

static const struct device *syscon = DEVICE_DT_GET(DT_INST_PARENT(0));

static int pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t func_index, bit_offset, offset, value;
	uint32_t mask;
	int ret = 0;

#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
	LOG_DBG("name %s, ball %d sig_descs %x", pin->name, pin->ball, pin->sig_descs);
#else
	LOG_DBG("ball %d sig_descs %x", pin->ball, pin->sig_descs);
#endif

	func_index = FIELD_GET(PINCTRL_SIG_DESC_FUNC_IDX, pin->sig_descs);
	bit_offset = FIELD_GET(PINCTRL_SIG_DESC_BIT_OFFSET, pin->sig_descs);
	offset = FIELD_GET(PINCTRL_SIG_DESC_REG_OFFSET, pin->sig_descs);
	mask = GENMASK(2, 0) << bit_offset;

	ret = syscon_read_reg(syscon, offset, &value);
	value = (value & ~(mask)) | (func_index << bit_offset);
	ret = syscon_write_reg(syscon, offset, value);

	return ret;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	int i, ret = 0;

	for (i = 0; i < pin_cnt; i++) {
		ret = pinctrl_configure_pin(&pins[i]);
		if (ret) {
			LOG_ERR("Failed to configure pin %d\n", pins[i].ball);
			break;
		}
	}

	return ret;
}
