/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/uart.h>
#include <shell/shell.h>
#include <usb/usb_device.h>
#include <stdlib.h>

#define SHELL_HELP_USB_ENABLE	"Enable USB as CDC_ACM"
#define SHELL_HELP_USB_CLEAR	"Clear USB env"
#define SHELL_HELP_USB_WAIT	"Set/Clear wait during USB transfer"
#define SHELL_HELP_SHELL	"Useful, not Unix-like shell commands."

#define RX_BUFF_SIZE	64
#define RING_BUF_SIZE	256

RING_BUF_DECLARE(ringbuf, RING_BUF_SIZE);

int total_len;
int count;
int transfer_wait;

static void data_handle(void)
{
	uint8_t buff[RX_BUFF_SIZE];
	int total = 0;
	int rb_len;

	while (!ring_buf_is_empty(&ringbuf)) {
		rb_len = ring_buf_get(&ringbuf, buff, sizeof(buff));
		total += rb_len;
	}
}

static void interrupt_handler(const struct device *dev, void *user_data)
{
	uint8_t rx_buff[RX_BUFF_SIZE];
	int recv_len, rb_len;

	ARG_UNUSED(user_data);

	while (uart_irq_is_pending(dev) && uart_irq_rx_ready(dev)) {
		recv_len = uart_fifo_read(dev, rx_buff, sizeof(rx_buff));

		total_len += recv_len;

		if (recv_len) {
			rb_len = ring_buf_put(&ringbuf, rx_buff, recv_len);
			if (rb_len < recv_len) {
				printk("Drop %u bytes\n", recv_len - rb_len);
			}
		}

		data_handle();
		count++;

		printk("total transfer len: 0x%x, count: %d\n", total_len, count);

		if (transfer_wait) {
			printk("wait 1 sec...\n");
			k_busy_wait(USEC_PER_SEC);
		}
	}
}

static int cmd_usb_enable(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	printk("enter %s\n", __func__);
	shell_print(shell, "enter %s...\n", __func__);

	ret = usb_enable(NULL);
	if (ret != 0) {
		shell_print(shell, "Failed to enable USB");
		return -1;
	}

	dev = device_get_binding("CDC_ACM_0");
	if (!dev) {
		shell_print(shell, "CDC ACM device not found");
		return -1;
	}

	uart_irq_callback_set(dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);

	return 0;
}

static int cmd_usb_clear(const struct shell *shell, size_t argc, char **argv)
{
	total_len = 0;
	count = 0;

	return 0;
}

static int cmd_usb_wait(const struct shell *shell, size_t argc, char **argv)
{
	transfer_wait = strtol(argv[1], NULL, 10);
	printk("wait: %d\n", transfer_wait);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(m_sub_usb,
	SHELL_CMD(enable, NULL, SHELL_HELP_USB_ENABLE, cmd_usb_enable),
	SHELL_CMD(clear, NULL, SHELL_HELP_USB_CLEAR, cmd_usb_clear),
	SHELL_CMD_ARG(wait, NULL, SHELL_HELP_USB_WAIT, cmd_usb_wait, 2, 0)
);

SHELL_CMD_REGISTER(usb, &m_sub_usb, SHELL_HELP_SHELL, NULL);
