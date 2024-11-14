/*
 * Copyright (c) 2024 Aspeed Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast2700_ibex_timer

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <soc.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

#define LOG_LEVEL CONFIG_TIMER_LOG_LEVEL
LOG_MODULE_REGISTER(ast2700_ibex_timer_ic, LOG_LEVEL_ERR);

#define TMR_COUNT_L		0x00
#define TMR_COUNT_H		0x04
#define TMR_ALARM_L		0x08
#define TMR_ALARM_H		0x0c
#define TMR_CTRL		0x10
#define TMR_CTRL_CLR		0x14
#define   TMR_CTRL_INTR_STS	BIT(16)	/* This can only be cleared by writing Reg08 and Reg0C */
#define   TMR_CTRL_COUNT_CLR	BIT(4)
#define   TMR_CTRL_RESET_EN	BIT(3)
#define   TMR_CTRL_EN		BIT(0)

#define TMR_DEV			DT_ALIAS(systick)
#define TMR_BASE		(DT_REG_ADDR(TMR_DEV))

#define CYCLES_PER_TICK		\
	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

static int32_t delta_tick;
static struct k_spinlock lock;

uint64_t sys_clock_cycle_get_64(void)
{
	uint32_t cnt_h, cnt_l;

	cnt_h = sys_read32(TMR_BASE + TMR_COUNT_H);
	cnt_l = sys_read32(TMR_BASE + TMR_COUNT_L);

	return ((uint64_t)(cnt_h) << 32) | cnt_l;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return sys_read32(TMR_BASE + TMR_COUNT_L);
}

uint32_t sys_clock_elapsed(void)
{
	return 0;
}

static void systick_isr(const void *unused)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	/* Disable the timer interrupt */
	sys_write32(TMR_CTRL_EN, TMR_BASE + TMR_CTRL_CLR);

	k_spin_unlock(&lock, key);
	sys_clock_announce(delta_tick);
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	uint64_t next_cycle, curr_cycle;
	uint32_t reg;
	k_spinlock_key_t key = k_spin_lock(&lock);

	/* Disable the timer interrupt */
	sys_write32(TMR_CTRL_EN, TMR_BASE + TMR_CTRL_CLR);

	curr_cycle = sys_clock_cycle_get_64();
	next_cycle = curr_cycle + ticks * CYCLES_PER_TICK;
	delta_tick = ticks;

	/* Configure timeout alarm. This will also clear TMR_CTRL_INTR_STS */
	sys_write32((uint32_t)(next_cycle & GENMASK(31, 0)), TMR_BASE + TMR_ALARM_L);
	sys_write32((uint32_t)(next_cycle >> 32), TMR_BASE + TMR_ALARM_H);

	/* Start the timer and enable the interrupt triggering */
	reg = TMR_CTRL_EN;
	sys_write32(reg, TMR_BASE + TMR_CTRL);

	k_spin_unlock(&lock, key);
}

static int sys_clock_driver_init(void)
{
	/* Disable the timer interrupt */
	sys_write32(TMR_CTRL_EN, TMR_BASE + TMR_CTRL_CLR);

	/* Set the maximum alarm value to prevent the interrupt from triggering */
	sys_write32(0xffffffff, TMR_BASE + TMR_ALARM_L);
	sys_write32(0xffffffff, TMR_BASE + TMR_ALARM_H);

	/* Reset the timer counter */
	sys_write32(TMR_CTRL_COUNT_CLR | TMR_CTRL_RESET_EN, TMR_BASE + TMR_CTRL);

	LOG_DBG("register base=%x, irqn=%d", TMR_BASE, DT_IRQN(DT_NODELABEL(systick)));
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(systick)), 0, systick_isr, 0, 0);
	irq_enable(DT_IRQN(DT_NODELABEL(systick)));

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
