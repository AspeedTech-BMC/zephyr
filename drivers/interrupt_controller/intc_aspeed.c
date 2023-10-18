/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Aspeed Interrupt Controller.
 *
 *  Copyright (C) 2023 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_ast2700_intc_ic

#include <zephyr/drivers/interrupt_controller/intc_aspeed.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#define LOG_LEVEL CONFIG_INTC_LOG_LEVEL
LOG_MODULE_REGISTER(intc_ast2700_ic, LOG_LEVEL_ERR);

#define INTC_GIC_BASE 128
#define GIC_IRQ_NUM 9 /* INTC0~INTC8 */
#define MAX_INTC_NUMBER (32 * GIC_IRQ_NUM)

/* Each bit in the register represents an IPC ID */
#define INTC_IER	0x0
#define INTC_RAW	0x4

/* Driver config */
struct intc_ast2700_config {
	/* Aspeed INTC base address. */
	uintptr_t base;
	/* number of Aspeed INTC. */
	uint8_t intc_num;
};

/* Driver data */
struct intc_ast2700_data {
	intc_callback_t callback;
	void *user_data;
};

static struct intc_ast2700_data aspeed_intcd[MAX_INTC_NUMBER];

static int intc_ast2700_isr(const void *dev)
{
	const struct intc_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base;
	int intc_num = config->intc_num;
	unsigned int status;
	unsigned int i;

	if (intc_num < 0 || intc_num > 9) {
		LOG_ERR("%s error idx %d\n", __func__, intc_num);
		return 1;
	}

	status = sys_read32(base + INTC_RAW);
	LOG_DBG("intc base 0x%lx: isr status 0x%x", base, status);
	for (i = 0; i < 32; i++) {
		if (status & BIT(i)) {
			if (!aspeed_intcd[(intc_num * 32) + i].callback) {
				LOG_ERR("No interrupt controller callback.\n");
				return 1;
			}
			aspeed_intcd[(intc_num * 32) + i].callback(dev,
						   aspeed_intcd[(intc_num * 32) + i].user_data);
			sys_write32(BIT(i), base + INTC_RAW);
		}
	}
	return 0;
}

static int intc_ast2700_register_callback(const struct device *dev, intc_callback_t cb,
					  void *user_data, int intc_bit)
{
	const struct intc_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base;
	int intc_num = config->intc_num;
	int irqn = (intc_num * 32) + intc_bit;

	if (irqn >= MAX_INTC_NUMBER) {
		LOG_ERR("invalid interrupt id=%u\n", irqn);
		return 1;
	}

	if (aspeed_intcd[irqn].callback) {
		LOG_ERR("IRQ %u has been requested\n", irqn);
		return -1;
	}

	LOG_DBG("interrupt controller base=%lx", base);
	LOG_DBG("irqn=%d, intc_num=%d, intc_bit=%d", irqn, intc_num, intc_bit);

	aspeed_intcd[irqn].callback = cb;
	aspeed_intcd[irqn].user_data = user_data;
	sys_write32(sys_read32(base + INTC_IER) | BIT(intc_bit),
		    base + INTC_IER);

	return 0;
}

static int intc_ast2700_irq_mask(const struct device *dev, int intc_bit)
{
	const struct intc_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base;
	int intc_num = config->intc_num;
	int irqn = (intc_num * 32) + intc_bit;

	if (irqn >= MAX_INTC_NUMBER) {
		LOG_ERR("invalid interrupt id=%u\n", irqn);
		return 1;
	}

	LOG_DBG("base=0x%lx", base + INTC_IER);
	LOG_DBG("before mask, value=0x%x", sys_read32(base + INTC_IER));
	sys_write32(sys_read32(base + INTC_IER) & ~BIT(intc_bit), base + INTC_IER);
	LOG_DBG("after mask, value=0x%x", sys_read32(base + INTC_IER));

	return 0;
}

static int intc_ast2700_irq_unmask(const struct device *dev, int intc_bit)
{
	const struct intc_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base;
	int intc_num = config->intc_num;
	int irqn = (intc_num * 32) + intc_bit;

	if (irqn >= MAX_INTC_NUMBER) {
		LOG_ERR("invalid interrupt id=%u\n", irqn);
		return 1;
	}

	LOG_DBG("base=0x%lx", base + INTC_IER);
	LOG_DBG("before unmask, value=0x%x", sys_read32(base + INTC_IER));
	sys_write32(sys_read32(base + INTC_IER) | BIT(intc_bit), base + INTC_IER);
	LOG_DBG("after unmask, value=0x%x", sys_read32(base + INTC_IER));

	return 0;
}

static int intc_ast2700_init(const struct device *dev)
{
	const struct intc_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base;
	int intc_num = config->intc_num;
	int ret = 0;

	LOG_DBG("INTC%d init: ", intc_num);
	LOG_DBG("register base=%lx, irqn=%d", base, intc_num + INTC_GIC_BASE);

	/* Check initial value to 0. */
	if (sys_read32(base + INTC_RAW)) {
		LOG_ERR("init isr incorrect INTC%d ", intc_num);
		LOG_ERR("%lx:", base + INTC_RAW);
		LOG_ERR("%x\n", sys_read32(base + INTC_RAW));
		ret = 1;
	}
	return ret;
}

static const struct intc_driver_api intc_ast2700_driver_api = {
	.register_callback	= intc_ast2700_register_callback,
	.set_irq_mask		= intc_ast2700_irq_mask,
	.set_irq_unmask		= intc_ast2700_irq_unmask,
};

#define INTC_AST2700_INIT(n)                                                                       \
	static int intc_ast2700_config_func_##n(const struct device *dev);                         \
	static const struct intc_ast2700_config intc_ast2700_config_##n = {                        \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.intc_num = DT_INST_IRQN(n) - INTC_GIC_BASE,                                       \
	};                                                                                         \
	static struct intc_ast2700_data intc_ast2700_data_##n;                                     \
	DEVICE_DT_INST_DEFINE(n, &intc_ast2700_config_func_##n, NULL, &intc_ast2700_data_##n,      \
			      &intc_ast2700_config_##n, POST_KERNEL,                               \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &intc_ast2700_driver_api);       \
	static int intc_ast2700_config_func_##n(const struct device *dev)                          \
	{                                                                                          \
		intc_ast2700_init(dev);                                                            \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), intc_ast2700_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
		return 0;                                                                          \
	}
DT_INST_FOREACH_STATUS_OKAY(INTC_AST2700_INIT)
