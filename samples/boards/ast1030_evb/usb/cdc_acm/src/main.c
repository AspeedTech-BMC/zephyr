/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample data transfer for CDC ACM class
 *
 * Sample app for USB CDC ACM class driver. The received data will be printed.
 */

#include <device.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(cdc_acm, LOG_LEVEL_INF);

#define RX_BUFF_SIZE 1024

static void interrupt_handler(const struct device *dev, void *user_data)
{
	uint8_t rx_buff[RX_BUFF_SIZE];
	int recv_len;
	int i;

	ARG_UNUSED(user_data);

	while (uart_irq_is_pending(dev)) {
		recv_len = uart_fifo_read(dev, rx_buff, RX_BUFF_SIZE);
		if (!recv_len)
			return;

		LOG_INF("recv_len: 0x%x", recv_len);
		for (i = 0; i < recv_len; i++)
			LOG_INF("rx_buff: 0x%x", rx_buff[i]);
	}
}

void main(void)
{
	const struct device *dev;
	int ret;

	dev = device_get_binding("CDC_ACM_0");
	if (!dev) {
		LOG_ERR("CDC ACM device not found");
		return;
	}

	ret = usb_enable(NULL);
	if (ret) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	uart_irq_callback_set(dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);
}
