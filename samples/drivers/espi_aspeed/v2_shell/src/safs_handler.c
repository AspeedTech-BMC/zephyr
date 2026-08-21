/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_aspeed.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include "safs_handler.h"

LOG_MODULE_DECLARE(espi_shell, CONFIG_LOG_DEFAULT_LEVEL);

#define SAFS_FLASH_SIZE      (8 * 1024 * 1024)

#define SAFS_ERASE_4K        (4 * 1024)
#define SAFS_ERASE_32K       (32 * 1024)
#define SAFS_ERASE_64K       (64 * 1024)
#define SAFS_ERASE_128K      (128 * 1024)

static uint8_t fake_flash[SAFS_FLASH_SIZE];
static uint8_t taf_tx_buf[sizeof(struct espi_flash_cmplt) + ESPI_PLD_LEN_MAX];

static uint16_t taf_payload_len(uint16_t len)
{
	return len ? len : ESPI_PLD_LEN_MAX;
}

static bool taf_range_valid(uint32_t addr, uint32_t len)
{
	return (addr < SAFS_FLASH_SIZE) && (len <= SAFS_FLASH_SIZE - addr);
}

static bool taf_erase_size(uint16_t type, uint32_t *erase_size)
{
	switch (type) {
	case 0:
		*erase_size = SAFS_ERASE_4K;
		return true;
	case 1:
		*erase_size = SAFS_ERASE_32K;
		return true;
	case 2:
		*erase_size = SAFS_ERASE_64K;
		return true;
	case 3:
		*erase_size = SAFS_ERASE_128K;
		return true;
	default:
		return false;
	}
}

static void send_unsuc_cmplt(const struct device *dev, uint8_t tag)
{
	struct espi_flash_cmplt cmplt;
	struct espi_aspeed_ioc ioc;
	int ret;

	cmplt.cyc   = ESPI_FLASH_UNSUC_CMPLT;
	cmplt.tag   = tag;
	cmplt.len_h = 0;
	cmplt.len_l = 0;

	ioc.pkt_len = sizeof(cmplt);
	ioc.pkt     = (uint8_t *)&cmplt;
	ret = espi_aspeed_flash_put_tx(dev, &ioc);
	if (ret) {
		LOG_ERR("UNSUC_CMPLT put_tx failed: %d", ret);
	}
}

static void send_suc_cmplt(const struct device *dev, uint8_t tag, const char *op)
{
	struct espi_flash_cmplt cmplt;
	struct espi_aspeed_ioc ioc;
	int ret;

	cmplt.cyc   = ESPI_FLASH_SUC_CMPLT;
	cmplt.tag   = tag;
	cmplt.len_h = 0;
	cmplt.len_l = 0;

	ioc.pkt_len = sizeof(cmplt);
	ioc.pkt     = (uint8_t *)&cmplt;
	ret = espi_aspeed_flash_put_tx(dev, &ioc);
	if (ret) {
		LOG_ERR("TAF %s completion failed: %d", op, ret);
	}
}

static void espi_taf_handler(const struct device *dev, struct espi_taf_req *req, void *user_data)
{
	struct espi_aspeed_ioc ioc;
	int ret;

	ARG_UNUSED(user_data);

	LOG_INF("TAF request: dev=%s cyc=0x%02x tag=0x%02x len=%u addr=0x%08x data=%p",
		dev->name, req->cyc, req->tag, req->len, req->addr, req->data);

	switch (req->cyc) {
	case ESPI_FLASH_READ: {
		struct espi_flash_cmplt *cmplt = (struct espi_flash_cmplt *)taf_tx_buf;
		uint16_t len = taf_payload_len(req->len);

		LOG_INF("TAF READ: addr=0x%08x len=%u tag=0x%02x",
			req->addr, len, req->tag);

		if (!taf_range_valid(req->addr, len)) {
			LOG_ERR("TAF READ out of range: addr=0x%08x len=%u flash_size=%u",
				req->addr, len, SAFS_FLASH_SIZE);
			send_unsuc_cmplt(dev, req->tag);
			return;
		}

		cmplt->cyc   = ESPI_FLASH_SUC_CMPLT_D_ONLY;
		cmplt->tag   = req->tag;
		cmplt->len_h = len >> 8;
		cmplt->len_l = len & 0xff;
		memcpy(cmplt->data, &fake_flash[req->addr], len);

		ioc.pkt     = taf_tx_buf;
		ioc.pkt_len = sizeof(*cmplt) + len;
		ret = espi_aspeed_flash_put_tx(dev, &ioc);
		if (ret) {
			LOG_ERR("TAF READ completion failed: %d", ret);
		} else {
			LOG_DBG("TAF READ completion sent: len=%u", ioc.pkt_len);
		}
		break;
	}
	case ESPI_FLASH_WRITE: {
		uint16_t len = taf_payload_len(req->len);

		LOG_INF("TAF WRITE: addr=0x%08x len=%u tag=0x%02x data=%p",
			req->addr, len, req->tag, req->data);

		if (!req->data) {
			LOG_ERR("TAF WRITE missing data pointer");
			send_unsuc_cmplt(dev, req->tag);
			return;
		}

		if (!taf_range_valid(req->addr, len)) {
			LOG_ERR("TAF WRITE out of range: addr=0x%08x len=%u flash_size=%u",
				req->addr, len, SAFS_FLASH_SIZE);
			send_unsuc_cmplt(dev, req->tag);
			return;
		}

		memcpy(&fake_flash[req->addr], req->data, len);
		send_suc_cmplt(dev, req->tag, "WRITE");
		break;
	}
	case ESPI_FLASH_ERASE: {
		uint32_t erase_size;

		LOG_INF("TAF ERASE: addr=0x%08x type=%u tag=0x%02x",
			req->addr, req->len, req->tag);

		if (!taf_erase_size(req->len, &erase_size)) {
			LOG_ERR("TAF ERASE invalid type: %u", req->len);
			send_unsuc_cmplt(dev, req->tag);
			return;
		}

		if ((req->addr % erase_size) != 0) {
			LOG_ERR("TAF ERASE unaligned addr: addr=0x%08x size=%u",
				req->addr, erase_size);
			send_unsuc_cmplt(dev, req->tag);
			return;
		}

		if (!taf_range_valid(req->addr, erase_size)) {
			LOG_ERR("TAF ERASE out of range: addr=0x%08x size=%u flash_size=%u",
				req->addr, erase_size, SAFS_FLASH_SIZE);
			send_unsuc_cmplt(dev, req->tag);
			return;
		}

		memset(&fake_flash[req->addr], 0xff, erase_size);
		send_suc_cmplt(dev, req->tag, "ERASE");
		break;
	}
	default:
		LOG_ERR("TAF unknown cycle: cyc=0x%02x tag=0x%02x", req->cyc, req->tag);
		send_unsuc_cmplt(dev, req->tag);
		break;
	}
}

int safs_init(void)
{
	const struct device *espi_dev = DEVICE_DT_GET(DT_NODELABEL(espi0));
	int ret;

	memset(fake_flash, 0xff, sizeof(fake_flash));

	LOG_INF("SAFS init: registering TAF handler on %s", espi_dev->name);

	ret = espi_aspeed_taf_register(espi_dev, espi_taf_handler, NULL);
	if (ret) {
		LOG_ERR("SAFS init: TAF handler register failed: %d", ret);
		return ret;
	}

	LOG_INF("SAFS init: TAF handler registered");
	return 0;
}
