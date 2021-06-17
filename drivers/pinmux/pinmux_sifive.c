/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifive_iof

/**
 * @brief PINMUX driver for the SiFive Freedom Processor
 */

#include <errno.h>
#include <device.h>
#include <drivers/pinmux.h>
#include <soc.h>

struct pinmux_sifive_config {
	uintptr_t base;
};

struct pinmux_sifive_regs_t {
	uint32_t iof_en;
	uint32_t iof_sel;
};

#define DEV_CFG(dev)					\
	((const struct pinmux_sifive_config * const)	\
	 (dev)->config)

#define DEV_PINMUX(dev)						\
	((struct pinmux_sifive_regs_t *)(DEV_CFG(dev))->base)

static int pinmux_sifive_set(const struct device *dev, uint32_t pin,
			     uint32_t func)
{
	volatile struct pinmux_sifive_regs_t *pinmux = DEV_PINMUX(dev);

	if (func > SIFIVE_PINMUX_IOF1 ||
	    pin >= SIFIVE_PINMUX_PINS)
		return -EINVAL;

	if (func == SIFIVE_PINMUX_IOF1)
		pinmux->iof_sel |= (SIFIVE_PINMUX_IOF1 << pin);
	else
		pinmux->iof_sel &= ~(SIFIVE_PINMUX_IOF1 << pin);

	/* Enable IO function for this pin */
	pinmux->iof_en |= (1 << pin);

	return 0;
}

static int pinmux_sifive_get(const struct device *dev, uint32_t pin,
			     uint32_t *func)
{
	volatile struct pinmux_sifive_regs_t *pinmux = DEV_PINMUX(dev);

	if (pin >= SIFIVE_PINMUX_PINS ||
	    func == NULL)
		return -EINVAL;

	*func = (pinmux->iof_sel & (SIFIVE_PINMUX_IOF1 << pin)) ?
		SIFIVE_PINMUX_IOF1 : SIFIVE_PINMUX_IOF0;

	return 0;
}

static int pinmux_sifive_pullup(const struct device *dev, uint32_t pin,
				uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_sifive_input(const struct device *dev, uint32_t pin,
			       uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_sifive_init(const struct device *dev)
{
	volatile struct pinmux_sifive_regs_t *pinmux = DEV_PINMUX(dev);

	/* Ensure that all pins are disabled initially */
	pinmux->iof_en = 0x0;

	return 0;
}

static const struct pinmux_driver_api pinmux_sifive_driver_api = {
	.set = pinmux_sifive_set,
	.get = pinmux_sifive_get,
	.pullup = pinmux_sifive_pullup,
	.input = pinmux_sifive_input,
};

static const struct pinmux_sifive_config pinmux_sifive_0_config = {
	.base = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0,
		    &pinmux_sifive_init, NULL, NULL,
		    &pinmux_sifive_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmux_sifive_driver_api);
