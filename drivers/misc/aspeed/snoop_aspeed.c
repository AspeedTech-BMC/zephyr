/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_snoop

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/misc/aspeed/snoop_aspeed.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(snoop_aspeed);

/* LPC registers */
#define	HICR5		0x080
#define	  HICR5_ENINT_SNP1W	BIT(3)
#define	  HICR5_EN_SNP1W	BIT(2)
#define	  HICR5_ENINT_SNP0W	BIT(1)
#define	  HICR5_EN_SNP0W	BIT(0)
#define	HICR6		0x084
#define	  HICR6_STR_SNP1W	BIT(1)
#define	  HICR6_STR_SNP0W	BIT(0)
#define	SNPWADR		0x090
#define	  SNPWADR_ADDR1_MASK	GENMASK(31, 16)
#define	  SNPWADR_ADDR1_SHIFT	16
#define	  SNPWADR_ADDR0_MASK	GENMASK(15, 0)
#define	  SNPWADR_ADDR0_SHIFT	0
#define	SNPWDR		0x094
#define	  SNPWDR_DATA1_MASK	GENMASK(15, 8)
#define	  SNPWDR_DATA1_SHIFT	8
#define	  SNPWDR_DATA0_MASK	GENMASK(7, 0)
#define	  SNPWDR_DATA0_SHIFT	0
#define	HICRB		0x100
#define	  HICRB_ENSNP1D		BIT(15)
#define	  HICRB_ENSNP0D		BIT(14)

static uintptr_t lpc_base;
#define LPC_RD(reg)             sys_read32(lpc_base + (reg))
#define LPC_WR(val, reg)        sys_write32(val, lpc_base + (reg))

struct snoop_aspeed_data {
	snoop_aspeed_rx_callback_t *rx_cb;
};

struct snoop_aspeed_config {
	uintptr_t base;
	uint16_t port[SNOOP_CHANNEL_NUM];
};

int snoop_aspeed_register_rx_callback(const struct device *dev, snoop_aspeed_rx_callback_t cb)
{
	struct snoop_aspeed_data *data = (struct snoop_aspeed_data *)dev->data;

	if (data->rx_cb) {
		LOG_ERR("Snoop RX callback is registered\n");
		return -EBUSY;
	}

	data->rx_cb = cb;

	return 0;
}

static void snoop_aspeed_isr(const struct device *dev)
{
	uint32_t hicr6, snpwdr;
	uint8_t snoop[SNOOP_CHANNEL_NUM];
	uint8_t *snoop_ptr[SNOOP_CHANNEL_NUM];
	struct snoop_aspeed_data *data = (struct snoop_aspeed_data *)dev->data;

	hicr6 = LPC_RD(HICR6);
	snpwdr = LPC_RD(SNPWDR);

	memset(snoop_ptr, 0, sizeof(snoop_ptr));

	if (hicr6 & HICR6_STR_SNP0W) {
		snoop[0] = (snpwdr & SNPWDR_DATA0_MASK) >> SNPWDR_DATA0_SHIFT;
		snoop_ptr[0] = &snoop[0];
	}

	if (hicr6 & HICR6_STR_SNP1W) {
		snoop[1] = (snpwdr & SNPWDR_DATA1_MASK) >> SNPWDR_DATA1_SHIFT;
		snoop_ptr[1] = &snoop[1];
	}

	if (data->rx_cb) {
		data->rx_cb(snoop_ptr[0], snoop_ptr[1]);
	}

	LPC_WR(hicr6, HICR6);
}

static void snoop_aspeed_enable(const struct device *dev, uint32_t ch)
{
	uint32_t hicr5, snpwadr, hicrb;
	struct snoop_aspeed_config *cfg = (struct snoop_aspeed_config *)dev->config;

	hicr5 = LPC_RD(HICR5);
	snpwadr = LPC_RD(SNPWADR);
	hicrb = LPC_RD(HICRB);

	switch (ch) {
	case 0:
		hicr5 |= (HICR5_EN_SNP0W | HICR5_ENINT_SNP0W);
		snpwadr &= ~(SNPWADR_ADDR0_MASK);
		snpwadr |= ((cfg->port[0] << SNPWADR_ADDR0_SHIFT) & SNPWADR_ADDR0_MASK);
		hicrb |= HICRB_ENSNP0D;
		break;
	case 1:
		hicr5 |= (HICR5_EN_SNP1W | HICR5_ENINT_SNP1W);
		snpwadr &= ~(SNPWADR_ADDR1_MASK);
		snpwadr |= ((cfg->port[1] << SNPWADR_ADDR1_SHIFT) & SNPWADR_ADDR1_MASK);
		hicrb |= HICRB_ENSNP1D;
		break;
	default:
		LOG_ERR("unsupported snoop channel %d\n", ch);
		return;
	};

	LPC_WR(hicr5, HICR5);
	LPC_WR(snpwadr, SNPWADR);
	LPC_WR(hicrb, HICRB);
}

static int snoop_aspeed_init(const struct device *dev)
{
	int i;
	struct snoop_aspeed_config *cfg = (struct snoop_aspeed_config *)dev->config;

	if (!lpc_base)
		lpc_base = cfg->base;

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    snoop_aspeed_isr,
		    DEVICE_DT_INST_GET(0), 0);

	irq_enable(DT_INST_IRQN(0));

	for (i = 0; i < SNOOP_CHANNEL_NUM; ++i) {
		if (!cfg->port[i])
			continue;

		snoop_aspeed_enable(dev, i);
	}

	return 0;
}

static struct snoop_aspeed_data snoop_aspeed_data;

static struct snoop_aspeed_config snoop_aspeed_config = {
	.base = DT_REG_ADDR(DT_PARENT(DT_DRV_INST(0))),
	.port = { DT_INST_PROP_OR(0, port_IDX_0, 0),
		  DT_INST_PROP_OR(0, port_IDX_1, 0), },
};

DEVICE_DT_INST_DEFINE(0, snoop_aspeed_init, NULL,
		      &snoop_aspeed_data, &snoop_aspeed_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
