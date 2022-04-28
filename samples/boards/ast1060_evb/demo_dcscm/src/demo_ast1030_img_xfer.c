/*
 * Copyright (c) 2022 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdlib.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <drivers/flash.h>

#define BIN_BUFFER_SZ 0x2000
#define BIC_IMAGE_OFFSET 0x80000

void ast1030_fwspi_ck_ctrl(uint8_t val);

static const struct uart_config uart_config = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
};

int ast1030_img_xfer(void)
{
	const struct device *uart_dev, *flash_dev;
	uint32_t flash_offset, binary_size, xfer_size;
	uint8_t *binary_buffer = NULL;
	int i;
	int ret = 0;

	/* release  reset of UART1 */
	sys_write32(0xa00, 0x7e789098);

	uart_dev = device_get_binding("UART_1");
	if (!uart_dev) {
		printk("bin_xfer: UART device not found\n");
		ret = -EINVAL;
		goto end;
	}

	flash_dev = device_get_binding("fmc_cs0");
	if (!flash_dev) {
		printk("bin_xfer: flash device not found\n");
		ret = -EINVAL;
		goto end;
	}

	uart_configure(uart_dev, &uart_config);

	flash_offset = BIC_IMAGE_OFFSET;
	binary_buffer = malloc(BIN_BUFFER_SZ);
	if (!binary_buffer) {
		printk("bin_xfer: fail to alloc buffer\n");
		ret = -ENOSR;
		goto end;
	}

	ret = flash_read(flash_dev, flash_offset, binary_buffer, 4);
	if (ret != 0) {
		printk("bin_xfer: fail to read flash\n");
		ret = -ECANCELED;
		goto end;
	}

	binary_size = ((uint32_t *)binary_buffer)[0] + 4;

	printk("bin_xfer: uart boot binary size = %d\n", binary_size);

	printk("release BMC reset...\n");
	ast1030_fwspi_ck_ctrl(1);
	pfr_bmc_srst_enable_ctrl(false);
	pfr_bmc_extrst_enable_ctrl(false);

	if (binary_size > 256 * 1024) {
		printk("bin_xfer: invalid image size %d, skip transfer.\n", binary_size);
		ret = 0;
		goto end;
	}

	printk("bin_xfer: transfer ast1030 image....\n");

	while (binary_size) {
		xfer_size = (binary_size >= BIN_BUFFER_SZ) ? BIN_BUFFER_SZ : binary_size;
		ret = flash_read(flash_dev, flash_offset, binary_buffer, xfer_size);
		if (ret != 0) {
			printk("bin_xfer: fail to read flash (in progress)\n");
			ret = -ECANCELED;
			goto end;
		}

		for (i = 0; i < xfer_size; i++)
			uart_poll_out(uart_dev, binary_buffer[i]);

		binary_size -= xfer_size;
		flash_offset += xfer_size;
	}

end:
	if (binary_buffer)
		free(binary_buffer);

	return ret;
}
