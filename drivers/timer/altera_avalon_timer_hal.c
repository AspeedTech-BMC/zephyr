/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <device.h>
#include <drivers/timer/system_timer.h>
#include <altera_common.h>

#include "altera_avalon_timer_regs.h"
#include "altera_avalon_timer.h"

/* The old driver "now" API would return a full uptime value.  The new
 * one only requires the driver to track ticks since the last announce
 * call.  Implement the new call in terms of the old one on legacy
 * drivers by keeping (yet another) uptime value locally.
 */
static uint32_t driver_uptime;

static uint32_t accumulated_cycle_count;

static int32_t _sys_idle_elapsed_ticks = 1;

static void wrapped_announce(int32_t ticks)
{
	driver_uptime += ticks;
	sys_clock_announce(ticks);
}

static void timer_irq_handler(const void *unused)
{
	ARG_UNUSED(unused);

	accumulated_cycle_count += k_ticks_to_cyc_floor32(1);

	/* Clear the interrupt */
	alt_handle_irq((void *)TIMER_0_BASE, TIMER_0_IRQ);

	wrapped_announce(_sys_idle_elapsed_ticks);
}

int sys_clock_driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	IOWR_ALTERA_AVALON_TIMER_PERIODL(TIMER_0_BASE,
			k_ticks_to_cyc_floor32(1) & 0xFFFF);
	IOWR_ALTERA_AVALON_TIMER_PERIODH(TIMER_0_BASE,
			(k_ticks_to_cyc_floor32(1) >> 16) & 0xFFFF);

	IRQ_CONNECT(TIMER_0_IRQ, 0, timer_irq_handler, NULL, 0);
	irq_enable(TIMER_0_IRQ);

	alt_avalon_timer_sc_init((void *)TIMER_0_BASE, 0,
			TIMER_0_IRQ, k_ticks_to_cyc_floor32(1));

	return 0;
}

uint32_t sys_clock_cycle_get_32(void)
{
	/* Per the Altera Embedded IP Peripherals guide, you cannot
	 * use a timer instance for both the system clock and timestamps
	 * at the same time.
	 *
	 * Having this function return accumulated_cycle_count + get_snapshot()
	 * does not work reliably. It's possible for the current countdown
	 * to reset to the next interval before the timer interrupt is
	 * delivered (and accumulated cycle count gets updated). The result
	 * is an unlucky call to this function will appear to jump backward
	 * in time.
	 *
	 * To properly obtain timestamps, the CPU must be configured with
	 * a second timer peripheral instance that is configured to
	 * count down from some large initial 64-bit value. This
	 * is currently unimplemented.
	 */
	return accumulated_cycle_count;
}

uint32_t sys_clock_elapsed(void)
{
	return 0;
}
