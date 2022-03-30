#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <device.h>
#include <shell/shell.h>
#include <drivers/uart.h>
#include <drivers/flash.h>


static const struct uart_config uart_config = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
};

static int cmd_bin_xfer(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *uart_dev, *flash_dev;
	uint32_t flash_offset, binary_size, xfer_size;
	unsigned char *binary_buffer;
	int i;

	uart_dev = device_get_binding(argv[1]);
	if (!uart_dev) {
		shell_error(shell, "UART device not found");
		return -EINVAL;
	}
	flash_dev = device_get_binding(argv[2]);
	if (!flash_dev) {
		shell_error(shell, "flash device not found");
		return -EINVAL;
	}
	uart_configure(uart_dev, &uart_config);

	flash_offset = strtoul(argv[3], NULL, 16);
	binary_buffer = malloc(4096);
	flash_read(flash_dev, flash_offset, binary_buffer, 4);
	binary_size = ((uint32_t *)binary_buffer)[0] + 4;

	shell_print(shell, "uart boot binary size = %x", binary_size);

	while (binary_size) {
		xfer_size = (binary_size >= 4096) ? 4096 : binary_size;

		flash_read(flash_dev, flash_offset, binary_buffer, xfer_size);
		for (i = 0; i < xfer_size; i++) {
			uart_poll_out(uart_dev, binary_buffer[i]);
		}
		binary_size -= xfer_size;
		flash_offset += xfer_size;
	}
	free(binary_buffer);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	uart_cmds,
	SHELL_CMD_ARG(bin_xfer, NULL, "<uart_device> <flash_device> <flash_offset(hex)>",
		      cmd_bin_xfer, 4, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(uart, &uart_cmds, "Uart bin-xfer commands", NULL);
