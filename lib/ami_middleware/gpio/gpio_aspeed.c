/*
 * Copyright (c) 2021 AMI 
 * 
 */

#include <drivers/flash.h>
#include <drivers/spi_nor.h>
#include <gpio/gpio_aspeed.h>
#include <kernel.h>
#include <sys/util.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr.h>
#include <drivers/gpio.h>


#define LOG_MODULE_NAME gpio_api

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static char *GPIO_Devices_List[6] = {
	"GPIO0_A_D",
	"GPIO0_E_H",
	"GPIO0_I_L",
	"GPIO0_M_P",
	"GPIO0_Q_T",
	"GPIO0_U_V",
};

static void GPIO_dump_buf(uint8_t *buf, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++) {
		printk("%02x ", buf[i]);
		if (i % 16 == 15)
			printk("\n");
	}
	printk("\n");
}

int BMCBootHold()
{
	const struct device *gpio_dev = NULL;

	/* GPIOM5 */
	gpio_dev = device_get_binding("GPIO0_M_P");

	if (gpio_dev == NULL)
	{
		printk("[%d]Fail to get GPIO0_M_P", __LINE__);
		return -1;
	}

	gpio_pin_configure(gpio_dev, 5, GPIO_OUTPUT);

	k_busy_wait(10000); /* 10ms */

	gpio_pin_set(gpio_dev, 5, 0);

	k_busy_wait(50000); /* 50ms */

	return 0;
}

int PchBootHold()
{
	const struct device *gpio_dev = NULL;

	/* GPIOM5 */
	gpio_dev = device_get_binding("GPIO0_M_P");

	if (gpio_dev == NULL)
	{
		printk("[%d]Fail to get GPIO0_M_P", __LINE__);
		return -1;
	}

	gpio_pin_configure(gpio_dev, 1, GPIO_OUTPUT);

	k_busy_wait(10000); /* 10ms */

	gpio_pin_set(gpio_dev, 1, 0);

	k_busy_wait(50000); /* 50ms */

	return 0;
}

int BmcBootRelease()
{
	const struct device *gpio_dev = NULL;

	/* GPIOM5 */
	gpio_dev = device_get_binding("GPIO0_M_P");

	if (gpio_dev == NULL)
	{
		printk("[%d]Fail to get GPIO0_M_P", __LINE__);
		return -1;
	}

	gpio_pin_configure(gpio_dev, 5, GPIO_OUTPUT);

	k_busy_wait(10000); /* 10ms */

	gpio_pin_set(gpio_dev, 5, 1);

	k_busy_wait(50000); /* 50ms */

	return 0;
}

int PchBootRelease()
{
	const struct device *gpio_dev = NULL;

	/* GPIOM5 */
	gpio_dev = device_get_binding("GPIO0_M_P");

	if (gpio_dev == NULL)
	{
		printk("[%d]Fail to get GPIO0_M_P", __LINE__);
		return -1;
	}

	gpio_pin_configure(gpio_dev, 1, GPIO_OUTPUT);

	k_busy_wait(10000); /* 10ms */

	gpio_pin_set(gpio_dev, 1, 1);

	k_busy_wait(50000); /* 50ms */

	return 0;
}

