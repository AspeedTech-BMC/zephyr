/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Combined AST1040 EVB board bring-up suite: one flash, one boot, every
 * peripheral test runs in its own kernel thread and all threads are
 * batch-started (k_thread_create with K_FOREVER, then a tight
 * k_thread_start loop) so the RTOS scheduler genuinely interleaves them,
 * instead of running one after another. Harness itself (aspeed_test_cb/
 * create_test/run_test/terminate_test/run_test_suite/show_banner) is
 * board-agnostic and ported verbatim from
 * tests/boards/ast1030_evb/all/src/main.c, which in turn ported it from
 * aspeed-dev-v2.6.0's tests/boards/ast1030/src/main.c.
 *
 * aspeed_testcase[] currently holds "gpio" and "cptra_mci" - AST1040 has no
 * prior test suite to draw the rest of the peripheral list from (unlike
 * AST1030), so other peripherals get added one at a time in later rounds,
 * same as how the AST1030 port progressed tier by tier.
 *
 * Each peripheral's test_<name>() (defined in its own
 * tests/boards/ast1040_evb/<name>/src/main.c, using ast_zassert_* - see
 * common/ast_test.h) is the SAME function called by that peripheral's own
 * standalone ZTEST() wrapper - no separate/duplicate implementation here.
 * ast_zassert_* just sets a flag and prints instead of ztest's real
 * zassert_* longjmp, which is what makes it safe to call from any of
 * these worker threads instead of only ztest's own single runner thread.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>

extern int test_gpio(void);
extern int test_cptra_mci(void);

typedef int (*test_func_t)(void);

struct aspeed_tests {
	const char *name;
	test_func_t test;
	int results;
};

static struct aspeed_tests aspeed_testcase[] = {
	{"gpio", test_gpio, -1},
	{"cptra_mci", test_cptra_mci, -1},
};

#define TEST_MODULE_CNT ARRAY_SIZE(aspeed_testcase)
#define TEST_STACKSIZE  4096
/*
 * ztest's own single-runner-thread default stack (CONFIG_ZTEST_STACK_SIZE)
 * is only 1024, but that's one thread at a time; here TEST_MODULE_CNT
 * threads' stacks are all live simultaneously, so this is sized generously.
 * CONFIG_HW_STACK_PROTECTION (see prj.conf) turns any overflow that does
 * happen into a clean fault instead of silently corrupting whatever memory
 * sits past the stack.
 */
#define TEST_CI_TIMEOUT 40

K_THREAD_STACK_ARRAY_DEFINE(test_thread_stack, TEST_MODULE_CNT, TEST_STACKSIZE);
static struct k_thread ztest_thread[TEST_MODULE_CNT];
static int unit_test_remain;

/*
 * Ported verbatim from aspeed-dev-v2.6.0's aspeed_show_banner() (via
 * tests/boards/ast1030_evb/all/src/main.c) - the same ANSI-colored
 * PASS/FAIL ASCII art, using printk() to match this file's own
 * printk-only convention (no CONFIG_LOG=y dependency for a purely
 * cosmetic banner).
 */
static void aspeed_show_banner(bool is_passed)
{
	if (is_passed) {
		printk("\x1b[;32;1m\n"
		       "########     ###     ######   ######  ######## ########\n"
		       "##     ##   ## ##   ##    ## ##    ## ##       ##     ##\n"
		       "##     ##  ##   ##  ##       ##       ##       ##     ##\n"
		       "########  ##     ##  ######   ######  ######   ##     ##\n"
		       "##        #########       ##       ## ##       ##     ##\n"
		       "##        ##     ## ##    ## ##    ## ##       ##     ##\n"
		       "##        ##     ##  ######   ######  ######## ########\n"
		       "\x1b[0;m\n");
	} else {
		printk("\x1b[;31;1m\n"
		       "########    ###    #### ##       ######## ########\n"
		       "##         ## ##    ##  ##       ##       ##     ##\n"
		       "##        ##   ##   ##  ##       ##       ##     ##\n"
		       "######   ##     ##  ##  ##       ######   ##     ##\n"
		       "##       #########  ##  ##       ##       ##     ##\n"
		       "##       ##     ##  ##  ##       ##       ##     ##\n"
		       "##       ##     ## #### ######## ######## ########\n"
		       "\x1b[0;m\n");
	}
}

static void aspeed_test_cb(void *a, void *b, void *c)
{
	int num = (int)(intptr_t)a;

	ARG_UNUSED(b);
	ARG_UNUSED(c);

	printk("START - %s\n", aspeed_testcase[num].name);
	aspeed_testcase[num].results = aspeed_testcase[num].test();
	unit_test_remain--;
	printk("%s - %s\n", aspeed_testcase[num].results ? "FAIL" : "PASS",
	       aspeed_testcase[num].name);
}

static void aspeed_create_test(int thread_num)
{
	k_thread_create(&ztest_thread[thread_num], test_thread_stack[thread_num],
			 TEST_STACKSIZE, (k_thread_entry_t)aspeed_test_cb,
			 (void *)(intptr_t)thread_num, NULL, NULL, CONFIG_ZTEST_THREAD_PRIORITY,
			 0, K_FOREVER);
	k_thread_name_set(&ztest_thread[thread_num], aspeed_testcase[thread_num].name);
}

static void aspeed_run_test(void)
{
	for (int i = 0; i < TEST_MODULE_CNT; i++) {
		k_thread_start(&ztest_thread[i]);
	}
}

static bool aspeed_terminate_test(void)
{
	bool is_passed = true;

	printk("\nTest Counter: %d.\n", (int)TEST_MODULE_CNT);

	for (int i = 0; i < TEST_MODULE_CNT; i++) {
		k_thread_abort(&ztest_thread[i]);
		if (aspeed_testcase[i].results) {
			is_passed = false;
		}

		printk("Case %d - %s results: %s\n", i + 1, aspeed_testcase[i].name,
		       aspeed_testcase[i].results ? "FAILED" : "PASSED");
	}

	aspeed_show_banner(is_passed);

	return is_passed;
}

static bool aspeed_run_test_suite(void)
{
	int timer = 0;

	for (int i = 0; i < TEST_MODULE_CNT; i++) {
		aspeed_create_test(i);
	}

	unit_test_remain = TEST_MODULE_CNT;

	aspeed_run_test();

	while (unit_test_remain && (timer < TEST_CI_TIMEOUT)) {
		timer++;
		k_sleep(K_SECONDS(1));
	}

	if (unit_test_remain) {
		printk("TIMEOUT! Total test case: %d, remaining: %d\n", (int)TEST_MODULE_CNT,
		       unit_test_remain);
	}

	return aspeed_terminate_test();
}

ZTEST(all, test_ast1040)
{
	zassert_true(aspeed_run_test_suite(), "test ast1040 failed");
}

ZTEST_SUITE(all, NULL, NULL, NULL, NULL, NULL);
