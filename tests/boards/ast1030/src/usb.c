/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <tc_util.h>

#include <usb/usb_device.h>
#include <usb/usb_common.h>
#include <drivers/uart.h>

/* Max packet size for endpoints */
#define BULK_EP_MPS		64
#define ENDP_BULK_IN		0x81
#define VALID_EP		ENDP_BULK_IN
#define INVALID_EP		0x20

#define RX_BUFF_SIZE		64
#define RING_BUF_SIZE		256

RING_BUF_DECLARE(ringbuf, RING_BUF_SIZE);

K_SEM_DEFINE(usb_sem, 0, 1);
static const char *golden_str = "usb_test";

static int data_handle(void)
{
	uint8_t buff[RX_BUFF_SIZE];
	int data_print = 1;
	int rb_len;
	int i;

	rb_len = ring_buf_get(&ringbuf, buff, sizeof(buff));
	if (data_print) {
		printk("Print Data: ");
		for (i = 0; i < rb_len; i++)
			printk("0x%x ", buff[i]);
		printk("\n");
	}

	/* Check golden pattern */
	rb_len = strlen(golden_str);
	for (i = 0; i < rb_len; i++) {
		if (golden_str[i] != buff[i]) {
			printk("cmp to %d, buff: %s\n", i, buff);
			return -EIO;
		}
	}

	return TC_PASS;
}

static void interrupt_handler(const struct device *dev, void *user_data)
{
	uint8_t rx_buff[RX_BUFF_SIZE];
	int total_len = 0;
	int recv_len;
	int rb_len;

	ARG_UNUSED(user_data);

	while (uart_irq_is_pending(dev) && uart_irq_rx_ready(dev)) {
		recv_len = uart_fifo_read(dev, rx_buff, sizeof(rx_buff));
		total_len += recv_len;

		if (recv_len) {
			rb_len = ring_buf_put(&ringbuf, rx_buff, recv_len);
			if (rb_len < recv_len) {
				printk("Drop %u bytes\n", recv_len - rb_len);
			}
		} else {
			break;
		}

		printk("total transfer len: 0x%x\n", total_len);
		k_sem_give(&usb_sem);
	}
}

void test_usb_comm(void)
{
	const struct device *dev;
	int count = 0;
	int ret;

	dev = device_get_binding("CDC_ACM_0");
	zassert_not_null(dev, "CDC ACM device is not found");

	uart_irq_callback_set(dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);

	printk("Wait for host side data, expect to receive \"%s\"...\n\n", golden_str);
	while (count < 100) {
		ret = k_sem_take(&usb_sem, K_NO_WAIT);
		if (ret) {
			count++;
			k_sleep(K_MSEC(100));
		} else {
			break;
		}
	}

	zassert_equal(data_handle(), TC_PASS, "compare received data failed");
}

/* Test USB Device Cotnroller API */
void test_usb_dc_api(void)
{
	/* Control endpoins are configured */
	zassert_equal(usb_dc_ep_mps(0x0), 64,
		      "usb_dc_ep_mps(0x00) failed");
	zassert_equal(usb_dc_ep_mps(0x80), 64,
		      "usb_dc_ep_mps(0x80) failed");

	/* Bulk EP is not configured yet */
	zassert_equal(usb_dc_ep_mps(ENDP_BULK_IN), 0,
		      "usb_dc_ep_mps(ENDP_BULK_IN) not configured");
}

/* Test USB Device Cotnroller API for invalid parameters */
void test_usb_dc_api_invalid(void)
{
	uint32_t size;
	uint8_t byte;

	/* Set stall to invalid EP */
	zassert_not_equal(usb_dc_ep_set_stall(INVALID_EP), TC_PASS,
			  "usb_dc_ep_set_stall(INVALID_EP)");

	/* Clear stall to invalid EP */
	zassert_not_equal(usb_dc_ep_clear_stall(INVALID_EP), TC_PASS,
			  "usb_dc_ep_clear_stall(INVALID_EP)");

	/* Check if the selected endpoint is stalled */
	zassert_not_equal(usb_dc_ep_is_stalled(INVALID_EP, &byte), TC_PASS,
			  "usb_dc_ep_is_stalled(INVALID_EP, stalled)");
	zassert_not_equal(usb_dc_ep_is_stalled(VALID_EP, NULL), TC_PASS,
			  "usb_dc_ep_is_stalled(VALID_EP, NULL)");

	/* Halt invalid EP */
	zassert_not_equal(usb_dc_ep_halt(INVALID_EP), TC_PASS,
			  "usb_dc_ep_halt(INVALID_EP)");

	/* Enable invalid EP */
	zassert_not_equal(usb_dc_ep_enable(INVALID_EP), TC_PASS,
			  "usb_dc_ep_enable(INVALID_EP)");

	/* Disable invalid EP */
	zassert_not_equal(usb_dc_ep_disable(INVALID_EP), TC_PASS,
			  "usb_dc_ep_disable(INVALID_EP)");

	/* Flush invalid EP */
	zassert_not_equal(usb_dc_ep_flush(INVALID_EP), TC_PASS,
			  "usb_dc_ep_flush(INVALID_EP)");

	/* Set callback to invalid EP */
	zassert_not_equal(usb_dc_ep_set_callback(INVALID_EP, NULL), TC_PASS,
			  "usb_dc_ep_set_callback(INVALID_EP, NULL)");

	/* Write to invalid EP */
	zassert_not_equal(usb_dc_ep_write(INVALID_EP, &byte, sizeof(byte),
					  &size),
			  TC_PASS, "usb_dc_ep_write(INVALID_EP)");

	/* Read invalid EP */
	zassert_not_equal(usb_dc_ep_read(INVALID_EP, &byte, sizeof(byte),
					 &size),
			  TC_PASS, "usb_dc_ep_read(INVALID_EP)");
	zassert_not_equal(usb_dc_ep_read_wait(INVALID_EP, &byte, sizeof(byte),
					      &size),
			  TC_PASS, "usb_dc_ep_read_wait(INVALID_EP)");
	zassert_not_equal(usb_dc_ep_read_continue(INVALID_EP), TC_PASS,
			  "usb_dc_ep_read_continue(INVALID_EP)");

	/* Get endpoint max packet size for invalid EP */
	zassert_not_equal(usb_dc_ep_mps(INVALID_EP), TC_PASS,
			  "usb_dc_ep_mps(INVALID_EP)");
}

void test_usb_dc_api_read_write(void)
{
	uint32_t size;
	uint8_t byte;

	/* Read invalid EP */
	zassert_not_equal(usb_read(INVALID_EP, &byte, sizeof(byte), &size),
			  TC_PASS, "usb_read(INVALID_EP)");

	/* Write to invalid EP */
	zassert_not_equal(usb_write(INVALID_EP, &byte, sizeof(byte), &size),
			  TC_PASS, "usb_write(INVALID_EP)");
}

void test_usb_enable(void)
{
	zassert_equal(usb_enable(NULL), 0, "usb_enable() failed");
}

void test_usb_hw(void)
{
	printk("Do nothing\n");
}
