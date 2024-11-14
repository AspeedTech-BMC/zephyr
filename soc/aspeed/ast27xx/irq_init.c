/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 * Copyright (c) 2023 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM Cortex-M interrupt initialization
 *
 */

#include <zephyr/arch/cpu.h>
#include <cmsis_core.h>

/**
 *
 * @brief Initialize interrupts
 *
 * Ensures all interrupts have their priority set to _EXC_IRQ_DEFAULT_PRIO and
 * not 0, which they have it set to when coming out of reset. This ensures that
 * interrupt locking via BASEPRI works as expected.
 *
 */

void z_soc_irq_init(void)
{
	int irq = 0;

	for (; irq < CONFIG_NUM_IRQS; irq++) {
		z_soc_irq_priority_set(irq, _IRQ_PRIO_OFFSET, 0);
	}
}
