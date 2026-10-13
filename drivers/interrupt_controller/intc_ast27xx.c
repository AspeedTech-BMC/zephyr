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
#include <zephyr/devicetree/interrupt_controller.h>

#define LOG_LEVEL CONFIG_INTC_LOG_LEVEL
LOG_MODULE_REGISTER(intc_ast2700_ic);

#define MAX_INTC_NUMBER       (CONFIG_MAX_IRQ_PER_AGGREGATOR * CONFIG_NUM_3RD_LEVEL_AGGREGATORS)

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
	/* Base offset into _sw_isr_table for this instance's children. */
	uint32_t offset;
	/* level of Aspeed INTC. */
	uint8_t level;
	/* number IRQ of Aspeed INTC. */
	uint32_t num_irqs;
	/* Status bits allowed to already be set at init (see devicetree binding). */
	uint32_t init_ignore_pending_mask;
};

static int intc_ast2700_isr(const void *dev)
{
	const struct intc_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base;
	int level = config->level;
	unsigned int offset = config->offset;
	unsigned int enable;
	unsigned int status;
	unsigned int i;

	enable = sys_read32(base + INTC_IER);
	status = sys_read32(base + INTC_RAW);
	LOG_DBG("level %d interrupt, base 0x%lx", level, base);
	LOG_DBG("enable 0x%x, status 0x%x", enable, status);

	for (i = 0; i < CONFIG_MAX_IRQ_PER_AGGREGATOR; i++) {
		/* Read INTC0_11 or INTC1_x status back to check bit as INTC number. */
		if (IS_BIT_SET(enable, i) && IS_BIT_SET(status, i)) {
			LOG_DBG("sw_isr_table[%u], bit %lx", offset + i, BIT(i));
			if (_sw_isr_table[offset + i].isr) {
				_sw_isr_table[offset + i].isr(_sw_isr_table[offset + i].arg);
			} else {
				LOG_ERR("no ISR registered for sw_isr_table[%u]", offset + i);
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
	uint32_t unexpected_pending;

	LOG_DBG("interrupt level: %d", level);
	if (level == 2) {
		shift = CONFIG_1ST_LEVEL_INTERRUPT_BITS;
	} else if (level == 3) {
		shift = CONFIG_1ST_LEVEL_INTERRUPT_BITS + CONFIG_2ND_LEVEL_INTERRUPT_BITS;
	}
	intc_num = (irqn >> shift) - 1;

	LOG_DBG("INTC%d(0x%x) init: ", intc_num, intc_num);
	LOG_DBG("register base=%lx, irqn=0x%x", base, irqn);

	/*
	 * Check initial value to 0, other than bits the devicetree says are
	 * allowed to already be pending (e.g. intc_sw's level-triggered
	 * reset-line signaling, see aspeed,init-ignore-pending-mask).
	 */
	unexpected_pending = sys_read32(base + INTC_RAW) & ~config->init_ignore_pending_mask;
	if (unexpected_pending) {
		LOG_ERR("init isr incorrect INTC%d (reg 0x%lx): unexpected pending 0x%x", intc_num,
			base + INTC_RAW, unexpected_pending);
		ret = 1;
	}
	return ret;
}

static int intc_ast2700_irq_clear_pending(const struct device *dev, int intc_bit)
{
	const struct intc_ast2700_config *config = dev->config;
	uintptr_t base = config->base;

	if (intc_bit > CONFIG_MAX_IRQ_PER_AGGREGATOR - 1) {
		LOG_ERR("No such intc bit %d", intc_bit);
		return -EINVAL;
	}

	/* Write-1-to-clear the latched status bit, independent of IER. */
	sys_write32(BIT(intc_bit), base + INTC_RAW);

	return 0;
}

static const struct intc_driver_api intc_ast2700_driver_api = {
	.disable_irq = intc_ast2700_irq_mask,
	.enable_irq = intc_ast2700_irq_unmask,
	.irq_enabled = intc_ast2700_irq_is_enabled,
	.clear_pending_irq = intc_ast2700_irq_clear_pending,
};

/**
 * @brief Calculate the base offset in _sw_isr_table for a given INTC instance.
 *
 * In Zephyr's multi-level interrupt architecture:
 * - A 1st level aggregator (DT_INST_IRQ_LEVEL == 1) handles 2nd level IRQs.
 *   Its base offset starts at CONFIG_2ND_LVL_ISR_TBL_OFFSET plus the devicetree
 *   aspeed,l1-aggregator-index value times CONFIG_MAX_IRQ_PER_AGGREGATOR.
 * - A 2nd level aggregator (DT_INST_IRQ_LEVEL == 2) handles 3rd level IRQs.
 *   Its base offset starts at CONFIG_3RD_LVL_ISR_TBL_OFFSET plus its interrupt
 *   bit position in the parent aggregator times CONFIG_MAX_IRQ_PER_AGGREGATOR.
 *
 * Examples:
 * - intc_sw (a Level 1 aggregator node in DTS)
 * - aspeed,l1-aggregator-index = <1>;
 * - INTC_BASE_ISR_TBL_OFFSET = 240 (CONFIG_2ND_LVL_ISR_TBL_OFFSET)
 * - Calculation: 240 + (1 * 32) = 272
 *
 * - intc1_3 (a Level 2 aggregator node in DTS)
 * - parent: intc0_11 (Level 1 aggregator)
 * - interrupts = <3 ...>; (connected to bit 3 of its parent)
 * - DT_INST_IRQ_LEVEL(n) = 2 (indicates it is a 2nd level aggregator)
 * - INTC_BASE_ISR_TBL_OFFSET = 304 (CONFIG_3RD_LVL_ISR_TBL_OFFSET)
 * - Calculation: 304 + (3 * 32) = 400
 *
 * Result: All IRQs managed by each aggregator will have their ISRs stored
 *         starting from its calculated _sw_isr_table[] base offset.
 *
 * Current ast27xx_ssp.dtsi instances (with this SoC's current Kconfig:
 * CONFIG_2ND_LVL_ISR_TBL_OFFSET=240, CONFIG_3RD_LVL_ISR_TBL_OFFSET=304,
 * CONFIG_MAX_IRQ_PER_AGGREGATOR=32):
 *
 *   | node     | level | index source          | offset calc      | sw_isr_table idx |
 *   +----------+-------+-----------------------+------------------+------------------+
 *   | intc0_11 | 1     | l1-aggregator-index=0 | 240 + 0*32 = 240 | 240 .. 271       |
 *   | intc_sw  | 1     | l1-aggregator-index=1 | 240 + 1*32 = 272 | 272 .. 303       |
 *   | intc1_0  | 2     | bit 0 of intc0_11     | 304 + 0*32 = 304 | 304 .. 335       |
 *   | intc1_1  | 2     | bit 1 of intc0_11     | 304 + 1*32 = 336 | 336 .. 367       |
 *   | intc1_2  | 2     | bit 2 of intc0_11     | 304 + 2*32 = 368 | 368 .. 399       |
 *   | intc1_3  | 2     | bit 3 of intc0_11     | 304 + 3*32 = 400 | 400 .. 431       |
 *   | intc1_4  | 2     | bit 4 of intc0_11     | 304 + 4*32 = 432 | 432 .. 463       |
 *   | intc1_5  | 2     | bit 5 of intc0_11     | 304 + 5*32 = 464 | 464 .. 495       |
 *   | (none)   | 2     | bit 6, reserved       | 304 + 6*32 = 496 | 496 .. 527       |
 *   | (none)   | 2     | bit 7, reserved       | 304 + 7*32 = 528 | 528 .. 559       |
 */
#define ASPEED_INTC_OFFSET(n)                                                                      \
	(INTC_BASE_ISR_TBL_OFFSET(DT_DRV_INST(n)) +                                                \
	 (DT_INST_IRQ_LEVEL(n) == 1 ? DT_INST_PROP(n, aspeed_l1_aggregator_index)                  \
				    : DT_INST_IRQ_BY_IDX(n, 0, irq)) *                             \
	 CONFIG_MAX_IRQ_PER_AGGREGATOR)

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
		.offset = ASPEED_INTC_OFFSET(n),                                                   \
		.num_irqs = DT_NUM_IRQS(DT_DRV_INST(n)),                                           \
		.init_ignore_pending_mask =                                                        \
			DT_INST_PROP(n, aspeed_init_ignore_pending_mask),                          \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &intc_ast2700_config_func_##n, NULL, NULL,                        \
			      &intc_ast2700_config_##n, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,   \
			      &intc_ast2700_driver_api);                                           \
	IRQ_PARENT_ENTRY_DEFINE(ast2700_intc_##n, DEVICE_DT_INST_GET(n), DT_INST_IRQN(n),          \
				ASPEED_INTC_OFFSET(n), DT_INST_INTC_GET_AGGREGATOR_LEVEL(n));      \
	static int intc_ast2700_config_func_##n(const struct device *dev)                          \
	{                                                                                          \
		intc_ast2700_init(dev);                                                            \
		LISTIFY(DT_NUM_IRQS(DT_DRV_INST(n)), INTC_AST2700_IRQ_CONNECT, (), n);             \
		return 0;                                                                          \
	}
DT_INST_FOREACH_STATUS_OKAY(INTC_AST2700_INIT)
