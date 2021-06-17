/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2020 Linaro Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr.h>
#include <init.h>
#include <device.h>
#include <drivers/gpio.h>
#include <pm/pm.h>

#include <driverlib/ioc.h>


#define PORT    DT_GPIO_LABEL(DT_ALIAS(sw0), gpios)
#define PIN     DT_GPIO_PIN(DT_ALIAS(sw0), gpios)
#define PULL_UP DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios)

#define BUSY_WAIT_S 5U
#define SLEEP_US 2000U
#define SLEEP_S     3U

extern void CC1352R1_LAUNCHXL_shutDownExtFlash(void);

void main(void)
{
	uint32_t config, status;
	const struct device *gpiob;

	printk("\n%s system off demo\n", CONFIG_BOARD);

	/* Shut off external flash to save power */
	CC1352R1_LAUNCHXL_shutDownExtFlash();

	/* Configure to generate PORT event (wakeup) on button 1 press. */
	gpiob = device_get_binding(PORT);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, PIN, GPIO_INPUT | PULL_UP);

	/* Set wakeup bits for button gpio */
	config = IOCPortConfigureGet(PIN);
	config |= IOC_WAKE_ON_LOW;
	IOCPortConfigureSet(PIN, IOC_PORT_GPIO, config);

	printk("Busy-wait %u s\n", BUSY_WAIT_S);
	k_busy_wait(BUSY_WAIT_S * USEC_PER_SEC);

	printk("Sleep %u us (IDLE)\n", SLEEP_US);
	k_usleep(SLEEP_US);

	printk("Sleep %u s (STANDBY)\n", SLEEP_S);
	k_sleep(K_SECONDS(SLEEP_S));

	printk("Entering system off (SHUTDOWN); press BUTTON1 to restart\n");

	/* Clear GPIO interrupt */
	status = GPIO_getEventMultiDio(GPIO_DIO_ALL_MASK);
	GPIO_clearEventMultiDio(status);

	/*
	 * Force the SOFT_OFF state.
	 */
	pm_power_state_force((struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});

	printk("ERROR: System off failed\n");
	while (true) {
		/* spin to avoid fall-off behavior */
	}
}
