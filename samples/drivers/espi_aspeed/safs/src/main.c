/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/espi_aspeed.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#define ESPI_PLD_LEN_MIN	BIT(6)

#define SIZE_4K             (0x1000)
#define SIZE_32K            (0x8000)
#define SIZE_64K            (0x10000)
#define SIZE_128K           (0x20000)
#define SIZE_256K           (0x40000)

static uint64_t ssp_mem_base;
static uint64_t tgt_mapping_addr;

static void hexdump(uint8_t *buf, uint32_t len)
{
	int i;

	for (i = 0; i < len; ++i) {
		if (i && (i % 16 == 0))
			printk("\n");
		printk("%02x ", buf[i]);
	}
	printk("\n");
}

static int espi_safs_read(const struct device *dev, uint8_t tag, uint32_t addr, uint32_t len)
{
	int i, rc;
	uint32_t cnt = 0;
	struct espi_aspeed_ioc ioc;
	struct espi_flash_cmplt *cmplt_pkt;
	uint32_t cmplt_pkt_len;

	cmplt_pkt_len = sizeof(*cmplt_pkt) + ESPI_PLD_LEN_MAX;
	cmplt_pkt = (struct espi_flash_cmplt *)k_malloc(cmplt_pkt_len);
	if (!cmplt_pkt) {
		printk("cannot allocate completion packet\n");
		return -ENOMEM;
	}

	if (len <= ESPI_PLD_LEN_MIN) {
		cmplt_pkt->cyc = ESPI_FLASH_SUC_CMPLT_D_ONLY;
		cmplt_pkt->tag = tag;
		cmplt_pkt->len_h = len >> 8;
		cmplt_pkt->len_l = len & 0xff;
		for (i = 0; i < len; ++i)
			cmplt_pkt->data[i] = i;

		ioc.pkt_len = len + sizeof(*cmplt_pkt);
		ioc.pkt = (uint8_t *)cmplt_pkt;

		rc = espi_aspeed_flash_put_tx(dev, &ioc);
		if (rc) {
			printk("failed to put the only flash completion, rc=%d\n", rc);
			goto free_n_out;
		}
	} else {
		/* first data */
		cmplt_pkt->cyc = ESPI_FLASH_SUC_CMPLT_D_FIRST;
		cmplt_pkt->tag = tag;
		cmplt_pkt->len_h = ESPI_PLD_LEN_MIN >> 8;
		cmplt_pkt->len_l = ESPI_PLD_LEN_MIN & 0xff;
		for (i = 0; i < len; ++i)
			cmplt_pkt->data[i] = cnt++;

		ioc.pkt_len = ESPI_PLD_LEN_MIN + sizeof(*cmplt_pkt);
		ioc.pkt = (uint8_t *)cmplt_pkt;

		rc = espi_aspeed_flash_put_tx(dev, &ioc);
		if (rc) {
			printk("failed to put the first flash completion, rc=%d\n", rc);
			goto free_n_out;
		}

		len -= ESPI_PLD_LEN_MIN;
		cnt += ESPI_PLD_LEN_MIN;

		/* middle data */
		while (len > ESPI_PLD_LEN_MIN) {
			cmplt_pkt->cyc = ESPI_FLASH_SUC_CMPLT_D_MIDDLE;
			cmplt_pkt->tag = tag;
			cmplt_pkt->len_h = ESPI_PLD_LEN_MIN >> 8;
			cmplt_pkt->len_l = ESPI_PLD_LEN_MIN & 0xff;
			for (i = 0; i < ESPI_PLD_LEN_MIN; ++i)
				cmplt_pkt->data[i] = cnt++;

			ioc.pkt_len = ESPI_PLD_LEN_MIN + sizeof(*cmplt_pkt);
			ioc.pkt = (uint8_t *)cmplt_pkt;

			rc = espi_aspeed_flash_put_tx(dev, &ioc);
			if (rc) {
				printk("failed to put the middle flash completion, rc=%d\n", rc);
				goto free_n_out;
			}

			len -= ESPI_PLD_LEN_MIN;
			cnt += ESPI_PLD_LEN_MIN;
		}

		/* last data */
		cmplt_pkt->cyc = ESPI_FLASH_SUC_CMPLT_D_LAST;
		cmplt_pkt->tag = tag;
		cmplt_pkt->len_h = len >> 8;
		cmplt_pkt->len_l = len & 0xff;
		for (i = 0; i < len; ++i)
			cmplt_pkt->data[i] = cnt++;

		ioc.pkt_len = len + sizeof(*cmplt_pkt);
		ioc.pkt = (uint8_t *)cmplt_pkt;

		rc = espi_aspeed_flash_put_tx(dev, &ioc);
		if (rc) {
			printk("failed to put thelast flash completion, rc=%d\n", rc);
			goto free_n_out;
		}
	}

free_n_out:
	k_free(cmplt_pkt);

	return rc;
}

static int espi_safs_write(const struct device *dev, uint8_t tag, uint32_t addr, uint32_t len,
			   uint8_t *buf)
{
	int rc;
	struct espi_aspeed_ioc ioc;
	struct espi_flash_cmplt cmplt_pkt;
	int i;

	for (i = 0; i < len; i += 4) {
		sys_write32(*(uint32_t *)(buf + i), tgt_mapping_addr - ssp_mem_base + addr + i);
	}

	cmplt_pkt.cyc = ESPI_FLASH_SUC_CMPLT;
	cmplt_pkt.tag = tag;
	cmplt_pkt.len_h = 0;
	cmplt_pkt.len_l = 0;

	ioc.pkt_len = sizeof(cmplt_pkt);
	ioc.pkt = (uint8_t *)&cmplt_pkt;

	rc = espi_aspeed_flash_put_tx(dev, &ioc);
	if (rc) {
		printk("failed to put flash completion, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int espi_safs_erase(const struct device *dev, uint8_t tag, uint32_t addr, uint32_t len)
{
	int rc;
	struct espi_aspeed_ioc ioc;
	struct espi_flash_cmplt cmplt_pkt;
	int i;

	/* decode erase size */
	switch (len) {
	case 0:
		len = SIZE_4K;
		break;
	case 1:
		len = SIZE_32K;
		break;
	case 2:
		len = SIZE_64K;
		break;
	case 3:
		len = SIZE_128K;
		break;
	default:
		printk("unknown erase size: 0x%x\n", len);
		break;
	}
	for (i = 0; i < len; i += 4) {
		sys_write32(0xffffffff, tgt_mapping_addr - ssp_mem_base + addr + i);
	}

	cmplt_pkt.cyc = ESPI_FLASH_SUC_CMPLT;
	cmplt_pkt.tag = tag;
	cmplt_pkt.len_h = 0;
	cmplt_pkt.len_l = 0;

	ioc.pkt_len = sizeof(cmplt_pkt);
	ioc.pkt = (uint8_t *)&cmplt_pkt;

	rc = espi_aspeed_flash_put_tx(dev, &ioc);
	if (rc) {
		printk("failed to put flash completion, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int main(void)
{
	int rc;
	const struct device *espi_dev;
	struct espi_aspeed_ioc ioc;
	struct espi_flash_rwe *rwe_pkt;
	uint32_t rwe_pkt_len;
	uint32_t cyc, tag, len, addr;

	espi_dev = device_get_binding("espi@74c05000");
	if (!espi_dev) {
		printk("no eSPI device found\n");
		return -ENODEV;
	}

	rwe_pkt_len = sizeof(*rwe_pkt) + ESPI_PLD_LEN_MAX;
	rwe_pkt = (struct espi_flash_rwe *)k_malloc(rwe_pkt_len);
	if (!rwe_pkt) {
		printk("failed to allocate flash packet\n");
		return -ENOMEM;
	}

	ssp_mem_base = (sys_read32(0x72c02128) & 0x7fffffffULL) << 4;
	tgt_mapping_addr = ((uint64_t)sys_read32(0x74c05434) << 32) | sys_read32(0x74c05430);
	printk("ssp_mem_base=0x%llx, tgt_mapping_addr=0x%llx\n", ssp_mem_base, tgt_mapping_addr);

	while (1) {
		ioc.pkt_len = rwe_pkt_len;
		ioc.pkt = (uint8_t *)rwe_pkt;

		rc = espi_aspeed_flash_get_rx(espi_dev, &ioc, true);
		if (rc) {
			printk("failed to get flash packet, rc=%d\n", rc);
			continue;
		}

		cyc = rwe_pkt->cyc;
		tag = rwe_pkt->tag;
		len = (rwe_pkt->len_h << 8) | (rwe_pkt->len_l & 0xff);
		addr = BSWAP_32(rwe_pkt->addr_be);

		printk("\n==== Receive RX Packet ====\n");
		printk("cyc=0x%02x, tag=0x%02x, len=0x%04x, addr=0x%08x\n", cyc, tag, len, addr);
		if (cyc == ESPI_FLASH_WRITE)
			hexdump(rwe_pkt->data, (len) ? len : ESPI_PLD_LEN_MAX);

		switch (cyc) {
		case ESPI_FLASH_READ:
			rc = espi_safs_read(espi_dev, tag, addr, len);
			break;
		case ESPI_FLASH_WRITE:
			rc = espi_safs_write(espi_dev, tag, addr, len, rwe_pkt->data);
			break;
		case ESPI_FLASH_ERASE:
			rc = espi_safs_erase(espi_dev, tag, addr, len);
			break;
		default:
			rc = -EFAULT;
			break;
		}

		if (rc) {
			printk("failed to put flash packet, rc=%d\n", rc);
			continue;
		}
	}

	k_free(rwe_pkt);
	return 0;
}
