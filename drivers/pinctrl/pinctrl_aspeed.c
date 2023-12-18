/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_pinctrl

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/syscon.h>
#ifdef CONFIG_SOC_SERIES_AST10X0
#include <zephyr/dt-bindings/pinctrl/ast10x0-pinctrl.h>
#else
#include <zephyr/dt-bindings/pinctrl/ast26xx-pinctrl.h>
#endif
#include <pinctrl_soc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pinctrl_aspeed, LOG_LEVEL_INF);

#define PINCTRL_SIG_DESC_OP		BIT(31)
#define   PINCTRL_SIG_DESC_SET		0
#define   PINCTRL_SIG_DESC_CLR		1
#define PINCTRL_SIG_DESC_BIT_INDEX	GENMASK(27, 20)
#define PINCTRL_SIG_DESC_REG_OFFSET	GENMASK(19, 0)

static int pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t op, index, offset, value;
	int i, ret = 0;

#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
	LOG_DBG("name %s, ball %d", pin->name, pin->ball);
#else
	LOG_DBG("ball %d", pin->ball);
#endif

	for (i = 0; i < pin->num_of_descs; i++) {
		op = FIELD_GET(PINCTRL_SIG_DESC_OP, pin->sig_descs[i]);
		index = FIELD_GET(PINCTRL_SIG_DESC_BIT_INDEX, pin->sig_descs[i]);
		offset = FIELD_GET(PINCTRL_SIG_DESC_REG_OFFSET, pin->sig_descs[i]);

		ret = syscon_read_reg(dev, offset, &value);
		if (op == PINCTRL_SIG_DESC_CLR) {
			value &= ~BIT(index);
		} else {
			value |= BIT(index);
		}
		ret = syscon_write_reg(dev, offset, value);
	}

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
