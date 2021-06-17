/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * This test case verifies the correctness of irq_offload(), an important
 * routine used in many other test cases for running a function in interrupt
 * context, on the IRQ stack.
 *
 */
#include <zephyr.h>
#include <ztest.h>
#include <kernel_structs.h>
#include <irq_offload.h>

volatile uint32_t sentinel;
#define SENTINEL_VALUE 0xDEADBEEF

static void offload_function(const void *param)
{
	uint32_t x = POINTER_TO_INT(param);

	/* Make sure we're in IRQ context */
	zassert_true(k_is_in_isr(), "Not in IRQ context!");

	sentinel = x;
}

/**
 * @brief Verify thread context
 *
 * @ingroup kernel_interrupt_tests
 *
 * @details Check whether offloaded running function is in interrupt
 * context, on the IRQ stack or not.
 */
void test_irq_offload(void)
{
	/* Simple validation of nested locking. */
	unsigned int key1, key2;

	key1 = arch_irq_lock();
	zassert_true(arch_irq_unlocked(key1),
		     "IRQs should have been unlocked, but key is 0x%x\n",
		     key1);
	key2 = arch_irq_lock();
	zassert_false(arch_irq_unlocked(key2),
		      "IRQs should have been locked, but key is 0x%x\n",
		      key2);
	arch_irq_unlock(key2);
	arch_irq_unlock(key1);

	/**TESTPOINT: Offload to IRQ context*/
	irq_offload(offload_function, (const void *)SENTINEL_VALUE);

	zassert_equal(sentinel, SENTINEL_VALUE,
		      "irq_offload() didn't work properly");
}

/**
 * @brief Test the arch_nop() by invoking and measure it.
 *
 * @details This test is mainly for coverage of the code. arch_nop()
 * is a special implementation and it will behave differently on
 * different platforms. By the way, this also measures how many
 * cycles it spends for platforms that support it.
 *
 * FYI: The potential uses of arch_nop() could be:
 * - Code alignment: Although in this case it's much more likely the
 *   compiler doing so (or you're in an assembly file, in which case
 *   you're not calling arch_nop() anyway). And this would require
 *   that arch_nop() be ALWAYS_INLINE.
 * - Giving you a guaranteed place to put a breakpoint / trace trigger
 *   / etc. when debugging. This is on main usage of arch_nop(); it
 *   inherently is generally debugging code removed before actually
 *   pushing.
 * - Giving you a guaranteed place to put a patchpoint. E.g. ARMv7
 *   allows nop (and a few other instructions) to be modified
 *   concurrently with execution, but not most other instructions.
 * - Delaying a few instructions, e.g. for tight timing loops on
 *   M-cores.
 *
 * Our test here mainly aims at the 4th scenario mentioned above but
 * also potentially tests the 1st scenario. So no optimization here to
 * prevent arch_nop() has optimized by the compiler is necessary.
 *
 * @ingroup kernel_common_tests
 *
 * @see arch_nop()
 */
__no_optimization void test_nop(void)
{
	uint32_t t_get_time, t_before, t_after, diff;

	t_before = k_cycle_get_32();
	t_after = k_cycle_get_32();

	/* calculate time spent between two k_cycle_get_32() call */
	t_get_time = t_after - t_before;

	printk("time k_cycle_get_32() takes %d cycles\n", t_get_time);

	/*
	 *  If two k_cycle_get_32() call take zero cycle here, this
	 *  means it cannot comes out a correct result, in these
	 *  case, we skip this test, such as native posix.
	 */
	if (t_get_time == 0) {
		ztest_test_skip();
	}

	t_before = k_cycle_get_32();

	arch_nop();

#if defined(CONFIG_RISCV)
	/* do 2 nop instructions more to cost cycles */
	arch_nop();
	arch_nop();
#elif defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE) || \
	defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	/* do 4 nop instructions more to cost cycles */
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
#elif defined(CONFIG_ARC)
	/* do 7 nop instructions more to cost cycles */
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
#elif defined(CONFIG_SPARC)
	/* do 9 nop instructions more to cost cycles */
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
	arch_nop();
#elif defined(CONFIG_ARMV8_A)
	/* the ARMv8-A ARM states the following:
	 * No Operation does nothing, other than advance the value of
	 * the program counter by 4. This instruction can be used for
	 * instruction alignment purposes.
	 * Note: The timing effects of including a NOP instruction in
	 * a program are not guaranteed. It can increase execution time
	 * ,leave it unchanged, or even reduce it. Therefore, NOP
	 * instructions are not suitable for timing loops.
	 *
	 * So we skip the this test, it will get a negative cycles.
	 */
	ztest_test_skip();
#endif

	t_after = k_cycle_get_32();

	/* Calculate delta time of arch_nop(). */
	diff = t_after - t_before - t_get_time;
	printk("arch_nop() takes %d cycles\n", diff);

	/* An arch_nop() call should spend actual cpu cycles */
	zassert_true(diff > 0,
			"arch_nop() takes %d cpu cycles", diff);
}
