/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <zephyr/sw_isr_table.h>

#define LOG_LEVEL CONFIG_INTC_LOG_LEVEL
LOG_MODULE_REGISTER(intc_ast2700_ic, LOG_LEVEL_ERR);

#define MAX_INTC_NUMBER       (CONFIG_MAX_IRQ_PER_AGGREGATOR * CONFIG_NUM_3RD_LEVEL_AGGREGATORS)
#define INTC_2ND_LEVEL_NUMBER 11

/* Each bit in the register represents an IPC ID */
#define INTC_IER 0x0
#define INTC_RAW 0x4

#define IS_BIT_SET(reg, bit) ((((reg) >> (bit)) & (0x1)) != 0)

/* Driver config */
struct intc_ast2700_config {
	/* Aspeed INTC base address. */
	uintptr_t base;
	/* IRQ number from device tree. */
	uint32_t irqn;
	/* level of Aspeed INTC. */
	uint8_t level;
	/* number IRQ of Aspeed INTC. */
	uint32_t num_irqs;
};

static int intc_ast2700_isr(const void *dev)
{
	const struct intc_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base;
	int irqn = config->irqn;
	int level = config->level;
	unsigned int enable;
	unsigned int status;
	unsigned int i;
	unsigned int intc_num = 0;
	unsigned int irqn_from_intc = 0;
	unsigned int offset = 0;

	enable = sys_read32(base + INTC_IER);
	status = sys_read32(base + INTC_RAW);
	LOG_DBG("level %d interrupt, base 0x%lx", level, base);
	LOG_DBG("enable 0x%x, status 0x%x", enable, status);

	if (level == 1) {
		intc_num = INTC_2ND_LEVEL_NUMBER; /* INTC0_11 in NVIC */
		irqn_from_intc = irqn;            /* IRQN of INTC0_11 is NVIC#160~NVIC#165 */
		offset = CONFIG_2ND_LVL_ISR_TBL_OFFSET;
	} else if (level == 2) {
		/*
		 * INTC1_x in INTC0_11, which shifts 2rd level interrupt bits.
		 * x begin from 0, such as INTC1_0
		 */
		intc_num = (irqn >> CONFIG_2ND_LEVEL_INTERRUPT_BITS) - 1;

		/*
		 * INTC1_x registered ISR in IRQN calculated with INTC number(from INTC0).
		 * Table offset is in CONFIG_3RD_LVL_ISR_TBL_OFFSET +
		 * intc_num * CONFIG_MAX_IRQ_PER_AGGREGATOR.
		 */
		irqn_from_intc = irqn + (intc_num * CONFIG_MAX_IRQ_PER_AGGREGATOR);
		offset = CONFIG_3RD_LVL_ISR_TBL_OFFSET + (intc_num * CONFIG_MAX_IRQ_PER_AGGREGATOR);
	}

	for (i = 0; i < CONFIG_MAX_IRQ_PER_AGGREGATOR; i++) {
		/* Read INTC0_11 or INTC1_x status back to check bit as INTC number. */
		if (IS_BIT_SET(enable, i) && IS_BIT_SET(status, i)) {
			LOG_DBG("INTC %d: irqn %d, bit %lx", intc_num, irqn_from_intc + i, BIT(i));
			LOG_DBG("offset at %d", offset + i);
			if (_sw_isr_table[offset + i].isr) {
				_sw_isr_table[offset + i].isr(_sw_isr_table[offset + i].arg);
			} else {
				LOG_ERR("IRQ %u has not been requested", irqn_from_intc + i);
			}

			/* Clear INTC0_11 or INTC1_x status bit */
			LOG_DBG("base 0x%lx, write 0x%lx", base + INTC_RAW, BIT(i));
			sys_write32(BIT(i), base + INTC_RAW);
		}
	}

	return 0;
}

static int intc_ast2700_irq_mask(const struct device *dev, int intc_bit)
{
	const struct intc_ast2700_config *config = dev->config;
	uintptr_t base = config->base;

	if (intc_bit > CONFIG_MAX_IRQ_PER_AGGREGATOR - 1) {
		LOG_ERR("No such intc bit %d", intc_bit);
		return -EINVAL;
	}

	LOG_DBG("base=0x%lx", base + INTC_IER);
	LOG_DBG("before mask, value=0x%x", sys_read32(base + INTC_IER));
	sys_write32(sys_read32(base + INTC_IER) & ~BIT(intc_bit), base + INTC_IER);
	LOG_DBG("after mask, value=0x%x", sys_read32(base + INTC_IER));

	return 0;
}

static int intc_ast2700_irq_unmask(const struct device *dev, int intc_bit)
{
	const struct intc_ast2700_config *config = dev->config;
	uintptr_t base = config->base;

	if (intc_bit > CONFIG_MAX_IRQ_PER_AGGREGATOR - 1) {
		LOG_ERR("No such intc bit %d", intc_bit);
		return -EINVAL;
	}

	LOG_DBG("base=0x%lx", base + INTC_IER);
	LOG_DBG("before unmask, value=0x%x", sys_read32(base + INTC_IER));
	sys_write32(sys_read32(base + INTC_IER) | BIT(intc_bit), base + INTC_IER);
	LOG_DBG("after unmask, value=0x%x", sys_read32(base + INTC_IER));

	return 0;
}

static int intc_ast2700_irq_is_enabled(const struct device *dev, int intc_bit)
{
	const struct intc_ast2700_config *config = dev->config;
	uintptr_t base = config->base;

	if (intc_bit > CONFIG_MAX_IRQ_PER_AGGREGATOR - 1) {
		LOG_ERR("No such intc bit %d", intc_bit);
		return -EINVAL;
	}

	return (!!(sys_read32(base + INTC_IER) & BIT(intc_bit)));
}

static int intc_ast2700_init(const struct device *dev)
{
	const struct intc_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base;
	int irqn = config->irqn;
	int level = config->level;
	int shift = 0;
	int intc_num = 0;
	int ret = 0;

	LOG_DBG("interrupt level: %d", level);
	if (level == 2) {
		shift = CONFIG_1ST_LEVEL_INTERRUPT_BITS;
	} else if (level == 3) {
		shift = CONFIG_1ST_LEVEL_INTERRUPT_BITS + CONFIG_2ND_LEVEL_INTERRUPT_BITS;
	}
	intc_num = (irqn >> shift) - 1;

	LOG_DBG("INTC%d(0x%x) init: ", intc_num, intc_num);
	LOG_DBG("register base=%lx, irqn=0x%x", base, irqn);

	/* Check initial value to 0. */
	if (sys_read32(base + INTC_RAW)) {
		LOG_ERR("init isr incorrect INTC%d ", intc_num);
		LOG_ERR("%lx:", base + INTC_RAW);
		LOG_ERR("%x", sys_read32(base + INTC_RAW));
		ret = 1;
	}
	return ret;
}

static const struct intc_driver_api intc_ast2700_driver_api = {
	.disable_irq = intc_ast2700_irq_mask,
	.enable_irq = intc_ast2700_irq_unmask,
	.irq_enabled = intc_ast2700_irq_is_enabled,
};

#define INTC_AST2700_IRQ_CONNECT(index, inst)                                                      \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, index),                                      \
			    DT_INST_IRQ_BY_IDX(inst, index, priority), intc_ast2700_isr,           \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN_BY_IDX(inst, index));                                      \
	} while (false);

#define INTC_AST2700_INIT(n)                                                                       \
	static int intc_ast2700_config_func_##n(const struct device *dev);                         \
	static const struct intc_ast2700_config intc_ast2700_config_##n = {                        \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.level = DT_INST_IRQ_LEVEL(n),                                                     \
		.irqn = DT_INST_IRQN(n),                                                           \
		.num_irqs = DT_NUM_IRQS(DT_DRV_INST(n)),                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &intc_ast2700_config_func_##n, NULL, NULL,                        \
			      &intc_ast2700_config_##n, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,   \
			      &intc_ast2700_driver_api);                                           \
	static int intc_ast2700_config_func_##n(const struct device *dev)                          \
	{                                                                                          \
		intc_ast2700_init(dev);                                                            \
		LISTIFY(DT_NUM_IRQS(DT_DRV_INST(n)), INTC_AST2700_IRQ_CONNECT, (), n);             \
		return 0;                                                                          \
	}
DT_INST_FOREACH_STATUS_OKAY(INTC_AST2700_INIT)
