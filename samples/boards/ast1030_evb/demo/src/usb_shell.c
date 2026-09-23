/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/usb/usb_device.h>

#define SHELL_HELP_USB_ENABLE	"Enable USB as DFU"
#define SHELL_HELP_USB_CLEAR	"Clear USB env"
#define SHELL_HELP_USB_PRINT	"Print USB data"
#define SHELL_HELP_USB_WAIT	"Set/Clear wait during USB transfer"
#define SHELL_HELP_USB_SEND	"Send a string to the host over CDC-ACM"
#define SHELL_HELP_USB_ECHO	"Echo received data back to the host"
#define SHELL_HELP_SHELL	"Useful, not Unix-like shell commands."

#define RX_BUFF_SIZE		64
#define RING_BUF_SIZE		256

RING_BUF_DECLARE(ringbuf, RING_BUF_SIZE);
RING_BUF_DECLARE(txringbuf, RING_BUF_SIZE);

int total_len;
int transfer_wait;
int data_print;
int data_echo;

static bool usb_ready;

static void data_handle(const struct device *dev)
{
	uint8_t buff[RX_BUFF_SIZE];
	int total = 0;
	int rb_len;
	int put;
	int i;

	while (!ring_buf_is_empty(&ringbuf)) {
		rb_len = ring_buf_get(&ringbuf, buff, sizeof(buff));
		if (data_print) {
			printk("Print Data: ");
			for (i = 0; i < rb_len; i++)
				printk("0x%x ", buff[i]);
			printk("\n");
		}

		if (data_echo) {
			put = ring_buf_put(&txringbuf, buff, rb_len);

			if (put < rb_len) {
				printk("Echo drop %d bytes\n", rb_len - put);
			}

			uart_irq_tx_enable(dev);
		}

		total += rb_len;
	}
}

static void interrupt_handler(const struct device *dev, void *user_data)
{
	uint8_t rx_buff[RX_BUFF_SIZE];
	uint8_t tx_buff[RX_BUFF_SIZE];
	int recv_len, rb_len;

	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			recv_len = uart_fifo_read(dev, rx_buff, sizeof(rx_buff));

			total_len += recv_len;

			if (!recv_len) {
				break;
			}

			rb_len = ring_buf_put(&ringbuf, rx_buff, recv_len);
			if (rb_len < recv_len) {
				printk("Drop %u bytes\n", recv_len - rb_len);
			}

			data_handle(dev);

			printk("total transfer len: 0x%x\n", total_len);

			if (transfer_wait) {
				printk("wait 1 sec...\n");
				k_busy_wait(USEC_PER_SEC);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			rb_len = ring_buf_get(&txringbuf, tx_buff, sizeof(tx_buff));
			if (!rb_len) {
				uart_irq_tx_disable(dev);
				continue;
			}

			uart_fifo_fill(dev, tx_buff, rb_len);
		}
	}
}

static int cmd_usb_enable(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	int ret;

	shell_print(shell, "enter %s...\n", __func__);

	ret = usb_enable(NULL);
	if (ret != 0) {
		shell_print(shell, "Failed to enable USB");
		return -1;
	}

	/* Enable  CDC-ACM (composite with DFU) */
	dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (dev) {

		uart_irq_callback_set(dev, interrupt_handler);
		/* Enable rx interrupts */
		uart_irq_rx_enable(dev);
		usb_ready = true;
	}
	return 0;
}

static int cmd_usb_clear(const struct shell *shell, size_t argc, char **argv)
{
	total_len = 0;

	return 0;
}

static int cmd_usb_print(const struct shell *shell, size_t argc, char **argv)
{
	data_print = strtol(argv[1], NULL, 10);
	printk("data print: %s\n", data_print ? "true" : "false");

	return 0;
}

static int cmd_usb_wait(const struct shell *shell, size_t argc, char **argv)
{
	transfer_wait = strtol(argv[1], NULL, 10);
	printk("wait: %s\n", transfer_wait ? "true" : "false");

	return 0;
}

static int cmd_usb_send(const struct shell *shell, size_t argc,
			char **argv)
{
	const struct device *dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	size_t i;

	if (!usb_ready) {
		shell_print(shell, "USB is not enabled, run \"usb enable\" "
			    "first");
		return -1;
	}

	for (i = 1; i < argc; i++) {
		const char *p = argv[i];

		if (i > 1) {
			uart_poll_out(dev, ' ');
		}

		while (*p) {
			uart_poll_out(dev, *p++);
		}
	}

	uart_poll_out(dev, '\r');
	uart_poll_out(dev, '\n');

	return 0;
}

static int cmd_usb_echo(const struct shell *shell, size_t argc,
			char **argv)
{
	data_echo = strtol(argv[1], NULL, 10);
	printk("echo: %s\n", data_echo ? "true" : "false");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(m_sub_usb,
	SHELL_CMD(enable, NULL, SHELL_HELP_USB_ENABLE, cmd_usb_enable),
	SHELL_CMD(clear, NULL, SHELL_HELP_USB_CLEAR, cmd_usb_clear),
	SHELL_CMD_ARG(print, NULL, SHELL_HELP_USB_PRINT, cmd_usb_print, 2, 0),
	SHELL_CMD_ARG(wait, NULL, SHELL_HELP_USB_WAIT, cmd_usb_wait, 2, 0),
	SHELL_CMD_ARG(send, NULL, SHELL_HELP_USB_SEND, cmd_usb_send, 2, 8),
	SHELL_CMD_ARG(echo, NULL, SHELL_HELP_USB_ECHO, cmd_usb_echo, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(usb, &m_sub_usb, SHELL_HELP_SHELL, NULL);
