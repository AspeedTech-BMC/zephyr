/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_ast10x0_g2_atc

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(atc_aspeed, CONFIG_MISC_ASPEED_LOG_LEVEL);

/* TSENSE (temperature sensor) anti-tamper registers */
#define ATC_TSENSE_CTRL         0x070
#define ATC_TSENSE_CTRL_MODE    BIT(4)
#define ATC_TSENSE_CTRL_START   BIT(3)
#define ATC_TSENSE_CTRL_CORR    BIT(2)
#define ATC_TSENSE_CTRL_RSTN    BIT(1)
#define ATC_TSENSE_CTRL_EN      BIT(0)

#define ATC_TSENSE_CLKDIV       0x074
#define ATC_TSENSE_CLKDIV_MASK  GENMASK(3, 0)

#define ATC_TSENSE_STATUS               0x07c
#define ATC_TSENSE_STATUS_ERROR	        BIT(17)
#define ATC_TSENSE_STATUS_DATA_VLD      BIT(16)
#define ATC_TSENSE_STATUS_DATA_MASK     GENMASK(9, 0)

#define ATC_TSENSE_INT_STS		0x084
#define ATC_TSENSE_INT_STS_PEND	BIT(0)

#define ATC_TSENSE_INT_EN       0x088
#define ATC_TSENSE_INT_EN_EN    BIT(0)

struct atc_aspeed_config {
	mm_reg_t base;
	void (*irq_config_func)(const struct device *dev);
};

#define ATC_RD(cfg, reg)       sys_read32((cfg)->base + (reg))
#define ATC_WR(cfg, val, reg)  sys_write32((val), (cfg)->base + (reg))

static void atc_aspeed_tsense_isr(const struct device *dev)
{
	const struct atc_aspeed_config *cfg = dev->config;
	uint32_t status;

	status = ATC_RD(cfg, ATC_TSENSE_STATUS);

	LOG_INF("ATC TSENSE tamper interrupt: status=0x%08x data=0x%03lx error=%d",
		status,
		(unsigned long)(status & ATC_TSENSE_STATUS_DATA_MASK),
		!!(status & ATC_TSENSE_STATUS_ERROR));

	/* RW1C: writing 1 clears the pending bit / data-valid flag */
	ATC_WR(cfg, ATC_TSENSE_INT_STS_PEND, ATC_TSENSE_INT_STS);
	ATC_WR(cfg, ATC_TSENSE_STATUS_DATA_VLD, ATC_TSENSE_STATUS);
}

static int atc_aspeed_init(const struct device *dev)
{
	const struct atc_aspeed_config *cfg = dev->config;

	/* Bring TSENSE out of reset, enable it */
	ATC_WR(cfg, ATC_TSENSE_CTRL_RSTN | ATC_TSENSE_CTRL_EN, ATC_TSENSE_CTRL);

	ATC_WR(cfg, ATC_TSENSE_INT_EN_EN, ATC_TSENSE_INT_EN);

	cfg->irq_config_func(dev);

	return 0;
}

#define ATC_ASPEED_INIT(n)                                                                    \
	static void atc_aspeed_irq_config_##n(const struct device *dev)                          \
	{                                                                                         \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), atc_aspeed_tsense_isr,    \
			    DEVICE_DT_INST_GET(n), 0);                                            \
		irq_enable(DT_INST_IRQN(n));                                                      \
	}                                                                                          \
	static const struct atc_aspeed_config atc_aspeed_config_##n = {                          \
		.base = DT_INST_REG_ADDR(n),                                                      \
		.irq_config_func = atc_aspeed_irq_config_##n,                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, atc_aspeed_init, NULL, NULL, &atc_aspeed_config_##n,             \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(ATC_ASPEED_INIT)
