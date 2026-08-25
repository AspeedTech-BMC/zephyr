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
#ifdef CONFIG_SOC_SERIES_AST10x0_G2
#include <zephyr/dt-bindings/pinctrl/ast10x0-g2-pinctrl.h>
#define ASPEED_PINCTRL_BALL_NUM AST10X0_G2_BALL_NUM
#else
#include <zephyr/dt-bindings/pinctrl/ast27xx-soc1-pinctrl.h>
#define ASPEED_PINCTRL_BALL_NUM AST27XX_SOC1_BALL_NUM
#endif
#include <pinctrl_soc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pinctrl_aspeed_g7_soc1, LOG_LEVEL_INF);

#define PINCTRL_SIG_DESC_FUNC_IDX	GENMASK(31, 28)
#define PINCTRL_SIG_DESC_BIT_OFFSET	GENMASK(27, 20)
#define PINCTRL_SIG_DESC_REG_OFFSET	GENMASK(19, 0)

#define PINCTRL_PINCFG_DESC_WIDTH	GENMASK(31, 28)
#define PINCTRL_PINCFG_DESC_BIT_OFFSET	GENMASK(27, 20)
#define PINCTRL_PINCFG_DESC_REG_OFFSET	GENMASK(19, 0)

static const struct device *syscon = DEVICE_DT_GET(DT_INST_PARENT(0));

static struct {
	bool requested;
	uint32_t sig_descs;
#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
	const char *name;
#endif
} ball_owner[ASPEED_PINCTRL_BALL_NUM];

static int pinctrl_request_ball(const pinctrl_soc_pin_t *pin)
{
	if (pin->ball < 0 || pin->ball >= ARRAY_SIZE(ball_owner)) {
		LOG_ERR("Invalid ball %d", pin->ball);
		return -EINVAL;
	}

	if (ball_owner[pin->ball].requested &&
	    ball_owner[pin->ball].sig_descs != pin->sig_descs) {
#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
		LOG_ERR("ball %d already requested by pin %s, reject pin %s", pin->ball,
			ball_owner[pin->ball].name, pin->name);
#else
		LOG_ERR("ball %d already requested by sig_descs %08x, reject sig_descs %08x",
			pin->ball, ball_owner[pin->ball].sig_descs, pin->sig_descs);
#endif
		return -EBUSY;
	}

	ball_owner[pin->ball].requested = true;
	ball_owner[pin->ball].sig_descs = pin->sig_descs;
#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
	ball_owner[pin->ball].name = pin->name;
#endif

	return 0;
}

/*
 * Apply one PINCFG_DESC()-located attribute (drive strength, bias/pull
 * disable, ...): read-modify-write `val` into the described bitfield.
 * A zero `desc` means the ball has no register wired up for this
 * attribute yet, so it is silently skipped.
 */
static int pinctrl_apply_pincfg_desc(uint32_t desc, uint32_t val)
{
	uint32_t width, bit_offset, offset, mask, value;
	int ret;

	if (!desc) {
		return 0;
	}

	width = FIELD_GET(PINCTRL_PINCFG_DESC_WIDTH, desc);
	bit_offset = FIELD_GET(PINCTRL_PINCFG_DESC_BIT_OFFSET, desc);
	offset = FIELD_GET(PINCTRL_PINCFG_DESC_REG_OFFSET, desc);
	mask = GENMASK(width - 1, 0) << bit_offset;

	ret = syscon_read_reg(syscon, offset, &value);
	if (ret) {
		return ret;
	}

	value = (value & ~mask) | ((val << bit_offset) & mask);

	return syscon_write_reg(syscon, offset, value);
}

static int pinctrl_configure_pincfg(const pinctrl_soc_pin_t *pin)
{
	int ret;

	if (pin->bias_disable) {
		ret = pinctrl_apply_pincfg_desc(pin->bias_disable_desc, 1);
		if (ret) {
			return ret;
		}
	}

	if (pin->drive_strength_desc && pin->drive_strength_valid) {
		ret = pinctrl_apply_pincfg_desc(pin->drive_strength_desc, pin->drive_strength);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static int pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t func_index, bit_offset, offset, value;
	uint32_t mask;
	int ret = 0;

#ifdef CONFIG_PINCTRL_ASPEED_STRING_NAME
	LOG_DBG("name %s, ball %d sig_descs %x\n", pin->name, pin->ball, pin->sig_descs);
#else
	LOG_DBG("ball %d sig_descs %x", pin->ball, pin->sig_descs);
#endif

	ret = pinctrl_request_ball(pin);
	if (ret) {
		return ret;
	}

	func_index = FIELD_GET(PINCTRL_SIG_DESC_FUNC_IDX, pin->sig_descs);
	bit_offset = FIELD_GET(PINCTRL_SIG_DESC_BIT_OFFSET, pin->sig_descs);
	offset = FIELD_GET(PINCTRL_SIG_DESC_REG_OFFSET, pin->sig_descs);
	mask = GENMASK(2, 0) << bit_offset;

	ret = syscon_read_reg(syscon, offset, &value);
	value = (value & ~(mask)) | (func_index << bit_offset);
	ret = syscon_write_reg(syscon, offset, value);
	if (ret) {
		return ret;
	}

	return pinctrl_configure_pincfg(pin);
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
