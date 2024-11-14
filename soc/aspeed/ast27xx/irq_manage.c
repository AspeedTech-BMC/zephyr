/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM Cortex-M interrupt management
 *
 *
 * Interrupt management: enabling/disabling and dynamic ISR
 * connecting/replacing.  SW_ISR_TABLE_DYNAMIC has to be enabled for
 * connecting ISRs at runtime.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <cmsis_core.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/irq.h>
#include <zephyr/tracing/tracing.h>
#include <zephyr/pm/pm.h>

#define NUM_IRQS_PER_REG 32
#define REG_FROM_IRQ(irq) ((irq) / NUM_IRQS_PER_REG)
#define BIT_FROM_IRQ(irq) ((irq) % NUM_IRQS_PER_REG)

#if defined(CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER)

#include <zephyr/drivers/interrupt_controller/intc_aspeed.h>
#include "../../../arch/common/include/sw_isr_common.h"

#define INTCG_DEV		DT_ALIAS(intc)
#define INTCG_BASE		(DT_REG_ADDR(INTCG_DEV))
#define INTCG_IRQ_ROUTE_SEL	DT_ENUM_IDX(INTCG_DEV, irq_route_select)

#define INTCG_IRQ_ROUTE_REG0	0x200
#define INTCG_IRQ_ROUTE_REG1	0x300
#define NUM_OF_INTC		9
#define NUM_OF_2ND_LVL_IRQS	(32 * NUM_OF_INTC)
/**
 * | irq_raw  | level 1 irqn | level 2 irqn |
 * | 0        | 0            | N/A          |
 * | m        | m            | N/A          |
 * | 127      | 127          | N/A          |
 * | 128 + 0  | 128          | 0            |\
 * | 128 + 1  | 128          | 1            | \
 * | ...      | 128          | ...          |  \
 * | 128 + n  | 128          | n            |  INTC0
 * | ...      | 128          | ...          | /
 * | 128 + 31 | 128          | 31           |/
 * | 160 + 0  | 129          | 0            |\
 * | 160 + 1  | 129          | 1            | \
 * | ...      | 129          | ...          |  \
 * | 160 + k  | 129          | k            |  INTC1
 * | ...      | 129          | ...          |  /
 * | 160 + 31 | 129          | 31           |/
 * | ...      | ...          | ...          |
 * | ...      | ...          | ...          |
 * | ...      | ...          | ...          |
 * | 384 + 0  | 136          | 0            |\
 * | 384 + 1  | 136          | 1            | \
 * | ...      | 136          | ...          |  \
 * | 384 + p  | 136          | p            |  INTC8
 * | ...      | 136          | ...          |  /
 * | 384 + 31 | 136          | 31           |/
 * | 416      | 137          | N/A          |\
 * | 417      | 138          | N/A          | \
 * | ...      | ...          | ...          |  Reserved, no IRQ signals
 * | 421      | 142          | N/A          | /
 * | 422      | 143          | N/A          |/
 * | 423      | 144          | N/A          |\
 * | 424      | 145          | N/A          | \
 * | ...      | ...          | ...          |  Inter-processor interrupts, no interrupt routing
 * | 437      | 158          | N/A          | /
 * | 438      | 159          | N/A          |/
 */
static unsigned int irq_to_raw_irq(unsigned int irq)
{
	unsigned int level = irq_get_level(irq);
	unsigned int irq_raw;

	if (level == 1) {
		if (irq < CONFIG_2ND_LVL_INTR_00_OFFSET + NUM_OF_INTC) {
			irq_raw = irq;
		} else {
			irq_raw = irq + NUM_OF_2ND_LVL_IRQS - NUM_OF_INTC;
		}
	} else {
		unsigned int lvl1_irq = irq_parent_level_2(irq);
		unsigned int lvl2_irq = irq_from_level_2(irq);
		unsigned int offset = (lvl1_irq - CONFIG_2ND_LVL_INTR_00_OFFSET) * 32;

		irq_raw = CONFIG_2ND_LVL_INTR_00_OFFSET + offset + lvl2_irq;
	}

	return irq_raw;
}

static void intcg_set_irq_route(unsigned int irq_raw, int select)
{
	uintptr_t base = INTCG_BASE;
	uint32_t byte_offset = (irq_raw >> 5) * 4;
	uint32_t pos = irq_raw & BIT_MASK(5);
	uint32_t reg;

	reg = sys_read32(base + INTCG_IRQ_ROUTE_REG0 + byte_offset);
	reg &= ~BIT(pos);
	reg |= (select & BIT(0)) << pos;
	sys_write32(reg, base + INTCG_IRQ_ROUTE_REG0 + byte_offset);

	reg = sys_read32(base + INTCG_IRQ_ROUTE_REG1 + byte_offset);
	reg &= ~BIT(pos);
	reg |= ((select & BIT(1)) >> 1) << pos;
	sys_write32(reg, base + INTCG_IRQ_ROUTE_REG1 + byte_offset);
}

static int intc_aspeed_enable_irq(unsigned int irq)
{
	const struct device *dev;
	unsigned int local_irq = irq_from_level_2(irq);

	dev = z_get_sw_isr_device_from_irq(irq);
	if (!dev) {
		return -ENOSYS;
	}

	intc_enable_irq(dev, local_irq);

	return 0;
}

static int intc_aspeed_disable_irq(unsigned int irq)
{
	const struct device *dev;
	unsigned int local_irq = irq_from_level_2(irq);

	dev = z_get_sw_isr_device_from_irq(irq);
	if (!dev) {
		return -ENOSYS;
	}

	intc_disable_irq(dev, local_irq);

	return 0;
}

static int intc_aspeed_irq_is_enabled(unsigned int irq)
{
	const struct device *dev;
	unsigned int local_irq = irq_from_level_2(irq);

	dev = z_get_sw_isr_device_from_irq(irq);
	if (!dev) {
		return -ENOSYS;
	}


	return intc_irq_is_enabled(dev, local_irq);
}

void z_soc_irq_enable(unsigned int irq)
{
	unsigned int irq_raw;

	irq_raw = irq_to_raw_irq(irq);
	if (irq_raw < CONFIG_2ND_LVL_INTR_00_OFFSET + NUM_OF_2ND_LVL_IRQS) {
		intcg_set_irq_route(irq_raw, INTCG_IRQ_ROUTE_SEL);
	}

	if (irq_get_level(irq) == 1) {
		NVIC_EnableIRQ((IRQn_Type)irq);
	} else {
		intc_aspeed_enable_irq(irq);
	}
}

void z_soc_irq_disable(unsigned int irq)
{
	if (irq_get_level(irq) == 1) {
		NVIC_DisableIRQ((IRQn_Type)irq);
	} else {
		intc_aspeed_disable_irq(irq);
	}
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	if (irq_get_level(irq) == 1) {
		return NVIC->ISER[REG_FROM_IRQ(irq)] & BIT(BIT_FROM_IRQ(irq));
	} else {
		return intc_aspeed_irq_is_enabled(irq);
	}
}

/**
 * @internal
 *
 * @brief Set an interrupt's priority
 *
 * The priority is verified if ASSERT_ON is enabled. The maximum number
 * of priority levels is a little complex, as there are some hardware
 * priority levels which are reserved.
 */
void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, uint32_t flags)
{
	/* The kernel may reserve some of the highest priority levels.
	 * So we offset the requested priority level with the number
	 * of priority levels reserved by the kernel.
	 */

	/* If we have zero latency interrupts, those interrupts will
	 * run at a priority level which is not masked by irq_lock().
	 * Our policy is to express priority levels with special properties
	 * via flags
	 */
	if (IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) && (flags & IRQ_ZERO_LATENCY)) {
		if (ZERO_LATENCY_LEVELS == 1) {
			prio = _EXC_ZERO_LATENCY_IRQS_PRIO;
		} else {
			/* Use caller supplied prio level as-is */
		}
	} else {
		prio += _IRQ_PRIO_OFFSET;
	}

	/* The last priority level is also used by PendSV exception, but
	 * allow other interrupts to use the same level, even if it ends up
	 * affecting performance (can still be useful on systems with a
	 * reduced set of priorities, like Cortex-M0/M0+).
	 */
	__ASSERT(prio <= (BIT(NUM_IRQ_PRIO_BITS) - 1),
		 "invalid priority %d for %d irq! values must be less than %lu\n",
		 prio - _IRQ_PRIO_OFFSET, irq,
		 BIT(NUM_IRQ_PRIO_BITS) - (_IRQ_PRIO_OFFSET));

	/* 2nd level INTC does not support IRQ priority */
	if (irq_get_level(irq) == 1) {
		NVIC_SetPriority((IRQn_Type)irq, prio);
	}
}

inline __attribute__((always_inline)) unsigned int z_soc_irq_get_active(void)
{
	return __get_IPSR();
}

void z_soc_irq_eoi(unsigned int irq)
{
	ARG_UNUSED(irq);
}

#endif /* !defined(CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER) */
