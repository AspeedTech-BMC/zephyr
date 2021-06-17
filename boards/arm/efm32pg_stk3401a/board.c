/*
 * Copyright (c) 2020 Rafael Dias Menezes <rdmeneze@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include "board.h"
#include <drivers/gpio.h>
#include <sys/printk.h>

static int efm32pg_stk3401a_init(const struct device *dev)
{
	const struct device *bce_dev; /* Board Controller Enable Gpio Device */

	ARG_UNUSED(dev);

	/* Enable the board controller to be able to use the serial port */
	bce_dev = device_get_binding(BC_ENABLE_GPIO_NAME);

	if (!bce_dev) {
		printk("Board controller gpio port was not found!\n");
		return -ENODEV;
	}

	gpio_pin_configure(bce_dev, BC_ENABLE_GPIO_PIN, GPIO_OUTPUT_HIGH);

	return 0;
}

/* needs to be done after GPIO driver init */
SYS_INIT(efm32pg_stk3401a_init, POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
