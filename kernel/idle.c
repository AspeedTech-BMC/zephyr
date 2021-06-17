/*
 * Copyright (c) 2016 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <toolchain.h>
#include <linker/sections.h>
#include <drivers/timer/system_timer.h>
#include <wait_q.h>
#include <pm/pm.h>
#include <stdbool.h>
#include <logging/log.h>
#include <ksched.h>
#include <kswap.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

/**
 * @brief Indicate that kernel is idling in tickless mode
 *
 * Sets the kernel data structure idle field to either a positive value or
 * K_FOREVER.
 */
static void pm_save_idle(void)
{
#ifdef CONFIG_PM
	int32_t ticks = z_get_next_timeout_expiry();
	_kernel.idle = ticks;

	/*
	 * Call the suspend hook function of the soc interface to allow
	 * entry into a low power state. The function returns
	 * PM_STATE_ACTIVE if low power state was not entered, in which
	 * case, kernel does normal idle processing.
	 *
	 * This function is entered with interrupts disabled. If a low power
	 * state was entered, then the hook function should enable inerrupts
	 * before exiting. This is because the kernel does not do its own idle
	 * processing in those cases i.e. skips k_cpu_idle(). The kernel's
	 * idle processing re-enables interrupts which is essential for
	 * the kernel's scheduling logic.
	 */
	if (pm_system_suspend(ticks) == PM_STATE_ACTIVE) {
		k_cpu_idle();
	}
#endif
}

void z_pm_save_idle_exit(int32_t ticks)
{
#ifdef CONFIG_PM
	/* Some CPU low power states require notification at the ISR
	 * to allow any operations that needs to be done before kernel
	 * switches task or processes nested interrupts.
	 * This can be simply ignored if not required.
	 */
	pm_system_resume();
#endif	/* CONFIG_PM */
	sys_clock_idle_exit();
}

void idle(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	__ASSERT_NO_MSG(_current->base.prio >= 0);

	while (true) {
		/* SMP systems without a working IPI can't
		 * actual enter an idle state, because they
		 * can't be notified of scheduler changes
		 * (i.e. threads they should run).  They just
		 * spin in a yield loop.  This is intended as
		 * a fallback configuration for new platform
		 * bringup.
		 */
		if (IS_ENABLED(CONFIG_SMP) &&
		    !IS_ENABLED(CONFIG_SCHED_IPI_SUPPORTED)) {
			k_busy_wait(100);
			k_yield();
			continue;
		}

		/* Note weird API: k_cpu_idle() is called with local
		 * CPU interrupts masked, and returns with them
		 * unmasked.  It does not take a spinlock or other
		 * higher level construct.
		 */
		(void) arch_irq_lock();

		if (IS_ENABLED(CONFIG_PM)) {
			pm_save_idle();
		} else {
			k_cpu_idle();
		}

#if !defined(CONFIG_PREEMPT_ENABLED)
# if !defined(CONFIG_USE_SWITCH) || defined(CONFIG_SPARC)
		/* A legacy mess: the idle thread is by definition
		 * preemptible as far as the modern scheduler is
		 * concerned, but older platforms use
		 * CONFIG_PREEMPT_ENABLED=n as an optimization hint
		 * that interrupt exit always returns to the
		 * interrupted context.  So in that setup we need to
		 * explicitly yield in the idle thread otherwise
		 * nothing else will run once it starts.
		 */
		if (_kernel.ready_q.cache != _current) {
			z_swap_unlocked();
		}
# endif
#endif
	}
}
