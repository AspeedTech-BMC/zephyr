/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <debug/thread_analyzer.h>

#ifdef CONFIG_BOARD_AST1030_EVB
/* USB */
extern void test_usb_enable(void);
extern void test_usb_dc_api(void);
extern void test_usb_dc_api_read_write(void);
extern void test_usb_dc_api_invalid(void);
extern void test_usb_comm(void);

/* ADC */
extern void test_adc_enable(void);
extern void test_adc_normal_mode(void);
extern void test_adc_battery_mode(void);

/* PWM */
extern void test_pwm_tach_enable(void);
#if CONFIG_PWM_ASPEED_ACCURATE_FREQ
extern void test_pwm_tach_loopback_accurate(void);
#else
extern void test_pwm_tach_loopback_rough(void);
#endif
extern void test_pwm_tach_fan(void);

/* JTAG */
extern void test_jtag_enable(void);

/* I2C */
extern void test_i2c_enable(void);

/* I3C */
extern void test_i3c_enable(void);

/* GPIO */
extern void test_gpio_enable(void);

/* UART */
extern void test_uart_enable(void);

/* SPI */
extern void test_spi_enable(void);

/* ESPI */
extern void test_espi_enable(void);

/* PECI */
extern void test_peci_enable(void);
#endif

#ifdef CONFIG_BOARD_AST1030_SLT
extern void test_usb_enable(void);
#endif

#define run_test_suite(suite) \
	ast_run_test_suite(#suite, _##suite)

#define TEST_THREAD_CNT		20
#define TEST_STACKSIZE		1024

K_THREAD_STACK_ARRAY_DEFINE(test_thread_stack, TEST_THREAD_CNT, TEST_STACKSIZE);

static struct k_thread ztest_thread[TEST_THREAD_CNT];
static int unit_test_count;
static int unit_test_remain;

static void test_cb(void *a, void *dummy2, void *dummy)
{
	struct unit_test *test = (struct unit_test *)a;
	int ret = TC_PASS;

	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy);

	TC_START(test->name);

	printk("Current thread id: %d\n", (int)k_current_get());
	test->test();

	unit_test_remain--;
	Z_TC_END_RESULT(ret, test->name);
}

static int create_test(struct unit_test *test, int thread_num)
{
	int ret = TC_PASS;
	k_tid_t pid;

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		pid = k_thread_create(&ztest_thread[thread_num], test_thread_stack[thread_num],
				TEST_STACKSIZE,
				(k_thread_entry_t) test_cb, (struct unit_test *)test,
				NULL, NULL, CONFIG_ZTEST_THREAD_PRIORITY,
				test->thread_options | K_INHERIT_PERMS,
					K_FOREVER);

		if (test->name != NULL) {
			k_thread_name_set(&ztest_thread[thread_num], test->name);
		}
	} else {
		printk("multithreading is not supported\n");
		ret = TC_FAIL;
	}

	printk("Create thread %s\t to num %d, pid: %d\n", test->name, thread_num, (int)pid);

	return ret;
}

static void run_test(void)
{
	int i;

	for (i = 0; i < unit_test_count; i++) {
		k_thread_start(&ztest_thread[i]);
	}
}

static void terminate_test(void)
{
	int i;

	for (i = 0; i < unit_test_count; i++) {
		k_thread_abort(&ztest_thread[i]);
	}
}

static void ast_run_test_suite(const char *name, struct unit_test *suite)
{
	int count = 0;
	int fail = 0;

	while (suite->test) {
		fail += create_test(suite, unit_test_count++);
		suite++;
		if (unit_test_count >= TEST_THREAD_CNT) {
			printk("unit tests are limited to %d\n", unit_test_count);
			break;
		}
	}

	if (fail)
		goto end;

	unit_test_remain = unit_test_count;
	thread_analyzer_print();

	run_test();

	while (unit_test_remain && (count < 200)) {
		count++;
		k_sleep(K_MSEC(100));
	}

	if (unit_test_remain) {
		printk("Total test case: %d, remaining: %d\n",
			unit_test_count, unit_test_remain);
		fail++;
	}

end:
	terminate_test();
	zassert_equal(fail, 0, "test ast1030 failed");
}

static void test_platform(void)
{
#if CONFIG_BOARD_AST1030_EVB
	ztest_test_suite(test_ast1030,
			 ztest_unit_test(test_usb_enable),
			 ztest_unit_test(test_usb_dc_api),
			 ztest_unit_test(test_usb_dc_api_read_write),
			 ztest_unit_test(test_usb_dc_api_invalid),
			 ztest_unit_test(test_usb_comm),

			 ztest_unit_test(test_adc_enable),
			 ztest_unit_test(test_adc_normal_mode),
			 ztest_unit_test(test_adc_battery_mode),
			 ztest_unit_test(test_pwm_tach_enable),
#if CONFIG_PWM_ASPEED_ACCURATE_FREQ
			 ztest_unit_test(test_pwm_tach_loopback_accurate),
#else
			 ztest_unit_test(test_pwm_tach_loopback_rough),
#endif
			 ztest_unit_test(test_pwm_tach_fan),
			 ztest_unit_test(test_jtag_enable),
			 ztest_unit_test(test_i2c_enable),
			 ztest_unit_test(test_i3c_enable),
			 ztest_unit_test(test_gpio_enable),
			 ztest_unit_test(test_uart_enable),
			 ztest_unit_test(test_spi_enable),
			 ztest_unit_test(test_espi_enable),
			 ztest_unit_test(test_peci_enable)
			 );
#endif
#if CONFIG_BOARD_AST1030_SLT
	ztest_test_suite(test_ast1030,
			 ztest_unit_test(test_usb_hw),
			 );
#endif
	run_test_suite(test_ast1030);
}

/* test case main entry */
void test_main(void)
{
	ztest_test_suite(platform_test,
			 ztest_unit_test(test_platform));

	ztest_run_test_suite(platform_test);
}
