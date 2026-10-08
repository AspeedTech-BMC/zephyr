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
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/tracing/tracing.h>
#include <zephyr/pm/pm.h>

#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_REGISTER(soc, LOG_LEVEL_ERR);

#define NUM_IRQS_PER_REG  32
#define REG_FROM_IRQ(irq) ((irq) / NUM_IRQS_PER_REG)
#define BIT_FROM_IRQ(irq) ((irq) % NUM_IRQS_PER_REG)

#if defined(CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER)

#include <zephyr/drivers/interrupt_controller/intc_aspeed.h>
#include "../../../arch/common/include/sw_isr_common.h"

/*
 * sirq_csel: intcg_base + INTCG_IRQ_ROUTE_REG0
 * sirq_csel1: intcg_base + INTCG_IRQ_ROUTE_REG1
 * sirq_csel2: intcg_base + INTCG_IRQ_ROUTE_REG2
 */
#define INTCG_DEV           DT_ALIAS(intc)
#define INTCG_BASE          (DT_REG_ADDR(INTCG_DEV))
#define INTCG_IRQ_ROUTE_SEL DT_ENUM_IDX(INTCG_DEV, irq_route_select)

#define INTCG_IRQ_ROUTE_REG0 0x200
#define INTCG_IRQ_ROUTE_REG1 0x300
#define INTCG_IRQ_ROUTE_REG2 0x400

/*
 * sirqio_csel0: intc1g_base + INTC1G_IRQ_ROUTE_REG0
 * sirqio_csel1: intc1g_base + INTC1G_IRQ_ROUTE_REG1
 * sirqio_csel2: intc1g_base + INTC1G_IRQ_ROUTE_REG2
 */
#define INTC1G_DEV            DT_ALIAS(intc1)
#define INTC1G_BASE           (DT_REG_ADDR(INTC1G_DEV))
#define INTC1G_IRQ_ROUTE_SEL  DT_ENUM_IDX(INTC1G_DEV, irq_route_select)
#define INTC1G_IRQ_ROUTE_REG0 0x80
#define INTC1G_IRQ_ROUTE_REG1 0xa0
#define INTC1G_IRQ_ROUTE_REG2 0xc0

/*
 * Per AST2750 ch.24.3, INTC1's route-select registers address real
 * silicon sources (INTn) starting at 128, independent of software's
 * _sw_isr_table layout (CONFIG_3RD_LVL_ISR_TBL_OFFSET).
 */
#define INTC1G_ROUTE_BASE_IRQ 128

/*
 * Per AST2750 ch.7, raw NVIC irqn >= 138 is no longer an individually
 * routable INTn on this SoC (reserved range, or intc_sw's own
 * software-signaling lines) - this boundary may differ on other SoCs.
 */
#define INTCG_ROUTE_MAX_IRQ 138

/*
 * INTC1 (intc1_global) reset-deassert status register. The INTC1 interrupt
 * registers of a given processor are held in reset-assert until an external
 * master (e.g. the PSP) releases them by clearing the corresponding bit here.
 * While the bit is set the registers cannot be accessed, so poll it before
 * touching the INTC1 path. We never clear it - that is the master's job.
 *
 *   BIT(1): SSP (irq_route_select == 0x2)
 *   BIT(2): TSP (irq_route_select == 0x3)
 */
#define INTC1G_RESET_STATUS_REG 0x14
#define INTC1G_RESET_POLL_MAX   100000
/*
 * Wait for the external master to deassert this processor's INTC1 reset.
 * Until INTC1G_RESET_STATUS_REG's bit is cleared the INTC1 interrupt
 * registers are inaccessible, so this must run before any INTC1 access.
 */
static void intc1g_wait_reset_deassert(void)
{
	uintptr_t reg = INTC1G_BASE + INTC1G_RESET_STATUS_REG;
	uint32_t bit;

	if (INTC1G_IRQ_ROUTE_SEL == 0x2) {
		bit = BIT(1); /* routed to SSP */
	} else if (INTC1G_IRQ_ROUTE_SEL == 0x3) {
		bit = BIT(2); /* routed to TSP */
	} else {
		return;
	}

	for (int i = 0; i < INTC1G_RESET_POLL_MAX; i++) {
		if (!(sys_read32(reg) & bit)) {
			return;
		}
	}

	LOG_ERR("INTC1 still in reset (reg 0x%lx, bit 0x%x); registers inaccessible", reg, bit);
}

/*
 * For INTID m, when DIV and MOD are the integer division and modulo operations:
 * • The corresponding GICD_ISENABLER number, n, is given by n = m DIV 32.
 * • The offset of the required INTC_CSEL is (0x200 + (4*n)).
 * • The offset of the required INTC_CSEL1 is (0x300 + (4*n)).
 * • The offset of the required INTC_CSEL2 is (0x400 + (4*n)).
 * • The bit number of the required group modifier bit in this register is m MOD 32.
 *
 * "Combined {sirq_csel2[n], sirq_csel1[n], sirq_csel[n]}. The 'n' is from 0 to 128
 * 000: Route interrupt INTn(irq_raw) to PSP GICINT0-128
 * 001: Route interrupt INTn(irq_raw) to SSPINT0-128
 * 010: Route interrupt INTn(irq_raw) to TSPINT0-128
 * others: reserved"
 */
static void intcg_set_irq_route(unsigned int irq_raw, int select)
{
	uintptr_t base = INTCG_BASE;
	uint32_t byte_offset = (irq_raw >> 5) * 4;
	uint32_t bit_pos = irq_raw & BIT_MASK(5);

	if (select == 0x2) {
		/* INTC0 interrupt select for ssp. */
		sys_set_bit(base + INTCG_IRQ_ROUTE_REG0 + byte_offset, bit_pos);
		sys_clear_bit(base + INTCG_IRQ_ROUTE_REG1 + byte_offset, bit_pos);
		sys_clear_bit(base + INTCG_IRQ_ROUTE_REG2 + byte_offset, bit_pos);
	} else if (select == 0x3) {
		/* INTC0 interrupt select for tsp. */
		sys_clear_bit(base + INTCG_IRQ_ROUTE_REG0 + byte_offset, bit_pos);
		sys_set_bit(base + INTCG_IRQ_ROUTE_REG1 + byte_offset, bit_pos);
		sys_clear_bit(base + INTCG_IRQ_ROUTE_REG2 + byte_offset, bit_pos);
	} else {
		LOG_ERR("Unknown interrupt route select=0x%x", select);
	}

	LOG_DBG("%s: irq_raw=0x%x, select=0x%x, base=0x%lx, byte_offset=0x%x, "
		"bit_pos=0x%x",
		__func__, irq_raw, select, base, byte_offset, bit_pos);
	LOG_DBG("INTCG_IRQ_ROUTE_REG0=0x%x, "
		"INTCG_IRQ_ROUTE_REG1=0x%x, INTCG_IRQ_ROUTE_REG2=0x%x",
		sys_read32(base + INTCG_IRQ_ROUTE_REG0 + byte_offset),
		sys_read32(base + INTCG_IRQ_ROUTE_REG1 + byte_offset),
		sys_read32(base + INTCG_IRQ_ROUTE_REG2 + byte_offset));
}

/*
 * For INTID m, when DIV and MOD are the integer division and modulo operations:
 * • The corresponding GICD_ISENABLER number, n, is given by n = m DIV 32.
 * • The offset of the required INTCIO_CSEL is (0x80 + (4*n)).
 * • The offset of the required INTCIO_CSEL1 is (0xA0 + (4*n)).
 * • The offset of the required INTCIO_CSEL2 is (0xD0 + (4*n)).
 * • The bit number of the required group modifier bit in this register is m MOD 32.
 *
 * "Combined {sirqio_csel2[n], sirqio_csel1[n], sirqio_csel0[n]}.
 * The 'n' is from 0 to 191 and i=n+128.
 * 000: Route interrupt INTi(irq_raw) to PSP GICINT192 (default)
 * 001: Route interrupt INTi(irq_raw) to INTC controller
 * 010: Route interrupt INTi(irq_raw) to SSPINT160
 * 011: Route interrupt INTi(irq_raw) to TSPINT160
 * 100: Route interrupt INTi(irq_raw) to PSP GICINT208
 * 101: Route interrupt INTi(irq_raw) to PSP GICINT224
 * 110: Route interrupt INTi(irq_raw) to MCU0
 * others: reserved"
 */
static void intc1g_set_irq_route(unsigned int irq_raw, int select)
{
	uintptr_t base = INTC1G_BASE;
	uint32_t byte_offset;
	uint32_t bit_pos = irq_raw & BIT_MASK(5);

	intc1g_wait_reset_deassert();

	irq_raw = irq_raw - INTC1G_ROUTE_BASE_IRQ;
	byte_offset = (irq_raw >> 5) * 4;

	if (sys_test_bit((mem_addr_t)&select, 0)) {
		sys_set_bit(base + INTC1G_IRQ_ROUTE_REG0 + byte_offset, bit_pos);
	} else {
		sys_clear_bit(base + INTC1G_IRQ_ROUTE_REG0 + byte_offset, bit_pos);
	}

	if (sys_test_bit((mem_addr_t)&select, 1)) {
		sys_set_bit(base + INTC1G_IRQ_ROUTE_REG1 + byte_offset, bit_pos);
	} else {
		sys_clear_bit(base + INTC1G_IRQ_ROUTE_REG1 + byte_offset, bit_pos);
	}

	if (sys_test_bit((mem_addr_t)&select, 2)) {
		sys_set_bit(base + INTC1G_IRQ_ROUTE_REG2 + byte_offset, bit_pos);
	} else {
		sys_clear_bit(base + INTC1G_IRQ_ROUTE_REG2 + byte_offset, bit_pos);
	}

	LOG_DBG("%s: irq_raw=0x%x, select=0x%x, base=0x%lx, byte_offset=0x%x, "
		"bit_pos=0x%x",
		__func__, irq_raw, select, base, byte_offset, bit_pos);
	LOG_DBG("INTC1G_IRQ_ROUTE_REG0=0x%x, "
		"INTC1G_IRQ_ROUTE_REG1=0x%x, INTC1G_IRQ_ROUTE_REG2=0x%x",
		sys_read32(base + INTC1G_IRQ_ROUTE_REG0 + byte_offset),
		sys_read32(base + INTC1G_IRQ_ROUTE_REG1 + byte_offset),
		sys_read32(base + INTC1G_IRQ_ROUTE_REG2 + byte_offset));
}

/**
 * @brief Get the aggregator that's responsible for the given irq
 *
 * @param irq IRQ number to query
 *
 * @return Aggregator entry, NULL if irq is level 1 or not found.
 */
static const struct _irq_parent_entry *get_intc_entry_for_irq(unsigned int irq)
{
	const unsigned int level = irq_get_level(irq);

	/* 1st level aggregator is not registered */
	if (level == 1) {
		LOG_ERR("%s, %d: No aggregator for level 1 irq %d", __func__, __LINE__, irq);
		return NULL;
	}

	/*
	 * Get device by its parent level irqn. For example, the interrupt assert sequence
	 * IPC -> INTC1_5(level 3) bit 19 -> INTC0_11(level 2) bit 5 -> NVIC 160(level 1)
	 * After DT_MACRO, irq_enable((IRQ_TO_L3(19) | IRQ_TO_L2(5) | 160));
	 * in intc_table, ._intc_table.static.intc_l3_5_ section only registered by L2 irqn(5)
	 * Get parent device by its irqn of  L2 irqn(5
	 */
	unsigned int intc_irq = irq_get_intc_irq(irq_from_level(irq, level - 1));

	/* Find an aggregator entry that matches the level & intc_irq */
	STRUCT_SECTION_FOREACH_ALTERNATE(intc_table, _irq_parent_entry, intc)
	{
		if (intc->level == level && intc->irq == intc_irq) {
			return intc;
		}
	}

	LOG_ERR("%s, %d: No aggregator found for irq %d, level %d, intc_irq %d", __func__, __LINE__,
		irq, level, intc_irq);
	return NULL;
}

const struct device *aspeed_get_sw_isr_device_from_irq(unsigned int irq)
{
	const struct _irq_parent_entry *intc = get_intc_entry_for_irq(irq);

	__ASSERT(!intc, "can't find an aggregator to handle irq(%X)", irq);

	/* Check intc and intc->dev or not, return NULL. */
	if (!intc || !intc->dev) {
		LOG_ERR("%s, %d: can't find an aggregator to handle irq(%X)", __func__, __LINE__,
			irq);
		return NULL;
	}

	return intc->dev;
}

static int intc_aspeed_enable_irq(unsigned int irq)
{
	const struct device *dev;
	unsigned int level = irq_get_level(irq);
	unsigned int local_irq = irq_from_level(irq, level);

	LOG_DBG("%s, %d: irq=%d, level=%d, local_irq=%d", __func__, __LINE__, irq, level,
		local_irq);
	dev = aspeed_get_sw_isr_device_from_irq(irq);
	if (!dev) {
		LOG_ERR("%s, %d: Failed to get device for irq %d", __func__, __LINE__, irq);
		return -ENOSYS;
	}

	/* Enable local_irq(intc_bit) in device. */
	intc_enable_irq(dev, local_irq);

	return 0;
}

static int intc_aspeed_disable_irq(unsigned int irq)
{
	const struct device *dev;
	unsigned int level = irq_get_level(irq);
	unsigned int local_irq = irq_from_level(irq, level);

	LOG_DBG("%s, %d: irq=%d, level=%d, local_irq=%d", __func__, __LINE__, irq, level,
		local_irq);
	dev = aspeed_get_sw_isr_device_from_irq(irq);
	if (!dev) {
		LOG_ERR("%s, %d: Failed to get device for irq %d", __func__, __LINE__, irq);
		return -ENOSYS;
	}

	/* Disable local_irq(intc_bit) in device. */
	intc_disable_irq(dev, local_irq);

	return 0;
}

static int intc_aspeed_irq_is_enabled(unsigned int irq)
{
	const struct device *dev;
	unsigned int level = irq_get_level(irq);
	unsigned int local_irq = irq_from_level(irq, level);

	LOG_DBG("%s, %d: irq=%d, level=%d, local_irq=%d", __func__, __LINE__, irq, level,
		local_irq);
	dev = aspeed_get_sw_isr_device_from_irq(irq);
	if (!dev) {
		LOG_ERR("%s, %d: Failed to get device for irq %d", __func__, __LINE__, irq);
		return -ENOSYS;
	}

	/* check eanbled or not in local_irq(intc_bit). */
	return intc_irq_is_enabled(dev, local_irq);
}

void z_soc_irq_enable(unsigned int irq)
{
	unsigned int level = irq_get_level(irq);

	LOG_DBG("%s, %d: irq=%d", __func__, __LINE__, irq);
	if (level == 1) {
		/*
		 * irq here is just this core's NVIC line, not necessarily a
		 * real INTn. Only raw < INTCG_ROUTE_MAX_IRQ is individually
		 * routable; above that is reserved or an aggregator's own
		 * internal signaling.
		 */
		if (irq < INTCG_ROUTE_MAX_IRQ) {
			intcg_set_irq_route(irq, INTCG_IRQ_ROUTE_SEL);
		}
	} else if (level == 3) {
		/*
		 * A level-3 leaf is always a real peripheral under some
		 * INTC1_x, rooted at INTC0_11 (ch.24.1/24.3). intc_sw's own
		 * lines are pure software signaling with no further levels,
		 * and the other 1-1 INTC aggregators ch.7 documents at raw
		 * NVIC 128..137 (INTC0_0..INTC0_10) have no devicetree node
		 * on this SoC, so INTC0_11 is the only level-1 aggregator
		 * that can ever produce a level-3 irq here.
		 */
		unsigned int lvl2_irq = irq_from_level_2(irq);
		unsigned int lvl3_irq = irq_from_level_3(irq);
		unsigned int irq_raw = INTC1G_ROUTE_BASE_IRQ +
					NUM_IRQS_PER_REG * lvl2_irq + lvl3_irq;

		intc1g_set_irq_route(irq_raw, INTC1G_IRQ_ROUTE_SEL);
	}

	if (level == 1) {
		NVIC_EnableIRQ((IRQn_Type)irq);
	} else {
		intc_aspeed_enable_irq(irq);
	}
}

void z_soc_irq_disable(unsigned int irq)
{
	LOG_DBG("%s, %d: irq=%d", __func__, __LINE__, irq);
	if (irq_get_level(irq) == 1) {
		NVIC_DisableIRQ((IRQn_Type)irq);
	} else {
		intc_aspeed_disable_irq(irq);
	}
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	LOG_DBG("%s, %d: irq=%d", __func__, __LINE__, irq);
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
		 prio - _IRQ_PRIO_OFFSET, irq, BIT(NUM_IRQ_PRIO_BITS) - (_IRQ_PRIO_OFFSET));

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
