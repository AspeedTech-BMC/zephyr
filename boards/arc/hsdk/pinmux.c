/*
 * Copyright (c) 2019 Synopsys
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <init.h>
#include <drivers/pinmux.h>

static int board_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	const struct device *pinmux = DEVICE_DT_GET(DT_NODELABEL(pinctrl));

	__ASSERT_NO_MSG(device_is_ready(pinmux));

	/*
	 * to do configuration for each sel,
	 * please refer the doc for hsdk board.
	 */
	pinmux_pin_set(pinmux, HSDK_PINMUX_SEL0, HSDK_PINMUX_FUN0);
	pinmux_pin_set(pinmux, HSDK_PINMUX_SEL1, HSDK_PINMUX_FUN0);
	pinmux_pin_set(pinmux, HSDK_PINMUX_SEL2, HSDK_PINMUX_FUN0);
	pinmux_pin_set(pinmux, HSDK_PINMUX_SEL3, HSDK_PINMUX_FUN2);
	pinmux_pin_set(pinmux, HSDK_PINMUX_SEL4, HSDK_PINMUX_FUN0);
	pinmux_pin_set(pinmux, HSDK_PINMUX_SEL5, HSDK_PINMUX_FUN0);
	pinmux_pin_set(pinmux, HSDK_PINMUX_SEL6, HSDK_PINMUX_FUN0);
	pinmux_pin_set(pinmux, HSDK_PINMUX_SEL7, HSDK_PINMUX_FUN0);

	return 0;
}


SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
