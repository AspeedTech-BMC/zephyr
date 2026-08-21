/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_aspeed.h>
#include <zephyr/sys/byteorder.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(espi_shell, CONFIG_LOG_DEFAULT_LEVEL);

extern const struct device *espi_dev;

#define MAFS_TX_BUF_SIZE 256

static uint8_t mafs_tx_buf[MAFS_TX_BUF_SIZE];
static uint8_t mafs_rx_buf[sizeof(struct espi_flash_rwe) + ESPI_PLD_LEN_MAX];

/* mafs read <addr> <len> */
static int cmd_mafs_read(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "Usage: mafs read <addr> <len>");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	uint32_t addr = (uint32_t)strtoul(argv[1], NULL, 0);
	uint16_t len  = (uint16_t)strtoul(argv[2], NULL, 0);

	if (len > MAFS_TX_BUF_SIZE) {
		shell_error(sh, "len exceeds max %d", MAFS_TX_BUF_SIZE);
		return -EINVAL;
	}

	struct espi_flash_packet pckt = {
		.buf        = mafs_tx_buf,
		.flash_addr = addr,
		.len        = len,
	};

	int ret = espi_read_flash(espi_dev, &pckt);

	if (ret) {
		shell_error(sh, "mafs read failed: %d", ret);
		return ret;
	}

	shell_print(sh, "mafs read addr=0x%08x len=%u:", addr, len);
	for (uint16_t i = 0; i < len; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%02x ", mafs_tx_buf[i]);
		if ((i + 1) % 16 == 0) {
			shell_fprintf(sh, SHELL_NORMAL, "\n");
		}
	}
	shell_fprintf(sh, SHELL_NORMAL, "\n");

	return 0;
}

/* mafs write <addr> <hex_bytes...> */
static int cmd_mafs_write(const struct shell *sh, size_t argc, char **argv)
{
	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	uint32_t addr = (uint32_t)strtoul(argv[1], NULL, 0);
	uint16_t len  = argc - 2;
	int ret;
	struct espi_aspeed_ioc ioc;
	struct espi_flash_rwe *rwe;
	uint32_t rx_addr;
	uint16_t rx_len;

	if (len > MAFS_TX_BUF_SIZE) {
		shell_error(sh, "too many bytes (max %d)", MAFS_TX_BUF_SIZE);
		return -EINVAL;
	}

	for (uint16_t i = 0; i < len; i++) {
		mafs_tx_buf[i] = (uint8_t)strtoul(argv[i + 2], NULL, 16);
	}

	struct espi_flash_packet pckt = {
		.buf        = mafs_tx_buf,
		.flash_addr = addr,
		.len        = len,
	};

	ret = espi_write_flash(espi_dev, &pckt);
	if (ret) {
		shell_error(sh, "mafs write failed: %d", ret);
		return ret;
	}
	shell_print(sh, "mafs write addr=0x%08x len=%u done", addr, len);

	ioc.pkt_len = sizeof(mafs_rx_buf);
	ioc.pkt     = mafs_rx_buf;

	ret = espi_aspeed_flash_get_rx(espi_dev, &ioc, true);
	if (ret) {
		shell_error(sh, "flash_get_rx failed: %d", ret);
		return ret;
	}

	rwe     = (struct espi_flash_rwe *)mafs_rx_buf;
	rx_addr = BSWAP_32(rwe->addr_be);
	rx_len  = (rwe->len_h << 8) | rwe->len_l;

	shell_print(sh, "RX: cyc=0x%02x tag=0x%02x len=0x%04x addr=0x%08x",
		    rwe->cyc, rwe->tag, rx_len, rx_addr);
	return 0;
}

/* mafs erase <addr> <len> */
static int cmd_mafs_erase(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "Usage: mafs erase <addr> <len>");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	uint32_t addr     = (uint32_t)strtoul(argv[1], NULL, 0);
	uint32_t erase_sz = (uint32_t)strtoul(argv[2], NULL, 0);
	uint16_t len;
	int ret;
	struct espi_aspeed_ioc ioc;
	struct espi_flash_rwe *rwe;
	uint32_t rx_addr;
	uint16_t rx_len;

	switch (erase_sz) {
	case 4096:
		len = 1;
		break;
	case 65536:
		len = 2;
		break;
	case 131072:
		len = 4;
		break;
	case 262144:
		len = 5;
		break;
	default:
		shell_error(sh, "invalid erase size %u (valid: 4096/65536/131072/262144)",
			    erase_sz);
		return -EINVAL;
	}

	struct espi_flash_packet pckt = {
		.buf        = NULL,
		.flash_addr = addr,
		.len        = len,
	};

	ret = espi_flash_erase(espi_dev, &pckt);
	if (ret) {
		shell_error(sh, "mafs erase failed: %d", ret);
		return ret;
	}
	shell_print(sh, "mafs erase addr=0x%08x size=%u done", addr, erase_sz);

	ioc.pkt_len = sizeof(mafs_rx_buf);
	ioc.pkt     = mafs_rx_buf;

	ret = espi_aspeed_flash_get_rx(espi_dev, &ioc, true);
	if (ret) {
		shell_error(sh, "flash_get_rx failed: %d", ret);
		return ret;
	}

	rwe     = (struct espi_flash_rwe *)mafs_rx_buf;
	rx_addr = BSWAP_32(rwe->addr_be);
	rx_len  = (rwe->len_h << 8) | rwe->len_l;

	shell_print(sh, "RX: cyc=0x%02x tag=0x%02x len=0x%04x addr=0x%08x",
		    rwe->cyc, rwe->tag, rx_len, rx_addr);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mafs_cmds,
	SHELL_CMD_ARG(read,  NULL, "Read flash: <addr> <len>",          cmd_mafs_read,  3, 0),
	SHELL_CMD_ARG(write, NULL, "Write flash: <addr> <hex_bytes...>", cmd_mafs_write, 2,
		      CONFIG_SHELL_ARGC_MAX - 2),
	SHELL_CMD_ARG(erase, NULL, "Erase flash: <addr> <len>",          cmd_mafs_erase, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(mafs, &mafs_cmds, "eSPI MAFS flash commands", NULL);
