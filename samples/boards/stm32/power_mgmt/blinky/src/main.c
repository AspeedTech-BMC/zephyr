/*
 * Copyright (c) 2021 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

#define SLEEP_TIME_MS   2000

static const struct gpio_dt_spec led =
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

void main(void)
{
	bool led_is_on = true;

	__ASSERT_NO_MSG(device_is_ready(led.port));

	printk("Device ready\n");

	/* Don't let the system power off / low power this device */
	device_busy_set(led.port);

	while (true) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
		gpio_pin_set(led.port, led.pin, (int)led_is_on);
		if (led_is_on == false) {
			/* Release resource to release device clock */
			gpio_pin_configure(led.port, led.pin, GPIO_DISCONNECTED);
		}
		k_msleep(SLEEP_TIME_MS);
		if (led_is_on == true) {
			/* Release resource to release device clock */
			gpio_pin_configure(led.port, led.pin, GPIO_DISCONNECTED);
		}
		led_is_on = !led_is_on;
	}
}
