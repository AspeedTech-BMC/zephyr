/* ST Microelectronics STMEMS hal i/f
 *
 * Copyright (c) 2021 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * zephyrproject-rtos/modules/hal/st/sensor/stmemsc/
 */

#include "stmemsc.h"

#define SPI_READ		(1 << 7)

/*
 * SPI read
 */
int stmemsc_spi_read(const struct stmemsc_cfg_spi *stmemsc,
		     uint8_t reg_addr, uint8_t *value, uint8_t len)
{
	const struct spi_config *spi_cfg = &stmemsc->spi_cfg;
	uint8_t buffer_tx[2] = { reg_addr | SPI_READ, 0 };

	/*  write 1 byte with reg addr (msb at 1) + 1 dummy byte */
	const struct spi_buf tx_buf = { .buf = buffer_tx, .len = 2, };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	/*
	 *   trensactions #1: dummy read to skip first byte
	 *   trensactions #2: read "len" byte of data
	 */
	const struct spi_buf rx_buf[2] = {
		{ .buf = NULL, .len = 1, },
		{ .buf = value, .len = len, }
	};
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = 2 };

	return spi_transceive(stmemsc->bus, spi_cfg, &tx, &rx);
}

/*
 * SPI write
 */
int stmemsc_spi_write(const struct stmemsc_cfg_spi *stmemsc,
		      uint8_t reg_addr, uint8_t *value, uint8_t len)
{
	const struct spi_config *spi_cfg = &stmemsc->spi_cfg;
	uint8_t buffer_tx[1] = { reg_addr & ~SPI_READ };

	/*
	 *   trensactions #1: write 1 byte with reg addr (msb at 0)
	 *   trensactions #2: write "len" byte of data
	 */
	const struct spi_buf tx_buf[2] = {
		{ .buf = buffer_tx, .len = 1, },
		{ .buf = value, .len = len, }
	};
	const struct spi_buf_set tx = { .buffers = tx_buf, .count = 2 };

	return spi_write(stmemsc->bus, spi_cfg, &tx);
}
