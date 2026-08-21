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
#include <stdlib.h>

LOG_MODULE_DECLARE(espi_shell, CONFIG_LOG_DEFAULT_LEVEL);

extern const struct device *espi_dev;

#define OOB_PKT_LEN_MAX 73  /* 3-byte header + 69-byte payload (max OOB body) */

static uint8_t oob_rx_buf[OOB_PKT_LEN_MAX];

static uint8_t oob_async_buf[OOB_PKT_LEN_MAX];
static uint16_t oob_async_len;

#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
static struct espi_callback oob_rx_cb;

struct oob_rx_work {
	struct k_work work;
	uint8_t len;
};

static struct oob_rx_work oob_work;

static void oob_rx_work_handler(struct k_work *work)
{
	struct oob_rx_work *w = CONTAINER_OF(work, struct oob_rx_work, work);
	struct espi_oob_packet pckt = {
		.buf = oob_async_buf,
		.len = OOB_PKT_LEN_MAX,
	};

	int ret = espi_receive_oob(espi_dev, &pckt);

	if (ret) {
		LOG_ERR("OOB async RX failed: %d", ret);
		return;
	}

	oob_async_len = pckt.len;
	LOG_INF("OOB async RX %u bytes (hint=%u)", pckt.len, w->len);
	for (uint16_t i = 0; i < pckt.len; i++) {
		LOG_INF("[%02u] 0x%02x", i, oob_async_buf[i]);
	}
}

static void oob_rx_handler(const struct device *dev,
			    struct espi_callback *cb,
			    struct espi_event event)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	oob_work.len = (uint8_t)event.evt_details;
	k_work_submit(&oob_work.work);
}

void oob_async_init(void)
{
	k_work_init(&oob_work.work, oob_rx_work_handler);
	espi_init_callback(&oob_rx_cb, oob_rx_handler, ESPI_BUS_EVENT_OOB_RECEIVED);
	espi_add_callback(espi_dev, &oob_rx_cb);
	LOG_INF("OOB async RX enabled");
}
#endif /* CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC */

static int cmd_oob_send(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: espi oob send <hex_bytes...>");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	static uint8_t tx_buf[OOB_PKT_LEN_MAX];
	size_t len = argc - 1;

	if (len > OOB_PKT_LEN_MAX) {
		shell_error(sh, "Packet too long (max %d bytes)", OOB_PKT_LEN_MAX);
		return -EINVAL;
	}

	for (size_t i = 0; i < len; i++) {
		tx_buf[i] = (uint8_t)strtoul(argv[i + 1], NULL, 16);
	}

	struct espi_oob_packet pckt = {
		.buf = tx_buf,
		.len = len,
	};

	int ret = espi_send_oob(espi_dev, &pckt);

	if (ret) {
		shell_error(sh, "OOB TX failed: %d", ret);
		return ret;
	}

	shell_print(sh, "OOB TX sent %zu bytes", len);
	return 0;
}

static int cmd_oob_recv(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	struct espi_oob_packet pckt = {
		.buf = oob_rx_buf,
		.len = OOB_PKT_LEN_MAX,
	};

	int ret = espi_receive_oob(espi_dev, &pckt);

	if (ret) {
		shell_error(sh, "OOB RX failed: %d", ret);
		return ret;
	}

	shell_print(sh, "OOB RX %u bytes:", pckt.len);
	for (uint16_t i = 0; i < pckt.len; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%02x ", oob_rx_buf[i]);
		if ((i + 1) % 16 == 0) {
			shell_fprintf(sh, SHELL_NORMAL, "\n");
		}
	}
	shell_fprintf(sh, SHELL_NORMAL, "\n");

	return 0;
}

static int cmd_oob_dump(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (oob_async_len == 0) {
		shell_print(sh, "No async OOB packet received yet");
		return 0;
	}

	shell_print(sh, "Last async OOB RX %u bytes:", oob_async_len);
	for (uint16_t i = 0; i < oob_async_len; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%02x ", oob_async_buf[i]);
		if ((i + 1) % 16 == 0) {
			shell_fprintf(sh, SHELL_NORMAL, "\n");
		}
	}
	shell_fprintf(sh, SHELL_NORMAL, "\n");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(oob_cmds,
	SHELL_CMD_ARG(send, NULL, "Send OOB packet: <hex_bytes...>", cmd_oob_send, 2,
		      CONFIG_SHELL_ARGC_MAX - 2),
	SHELL_CMD(recv, NULL, "Receive one OOB packet (sync)", cmd_oob_recv),
	SHELL_COND_CMD(CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC, dump, NULL,
		       "Dump last async received OOB packet", cmd_oob_dump),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(oob, &oob_cmds, "eSPI OOB shell commands", NULL);
