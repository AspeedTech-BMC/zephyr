/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @defgroup kernel_pipe_tests PIPEs
 * @ingroup all_tests
 * @{
 * @}
 */

#include <ztest.h>
extern void test_pipe_thread2thread(void);
extern void test_pipe_put_fail(void);
extern void test_pipe_get_fail(void);
extern void test_pipe_block_put(void);
extern void test_pipe_block_put_sema(void);
extern void test_pipe_get_put(void);
extern void test_pipe_get_large(void);

extern void test_half_pipe_put_get(void);
extern void test_half_pipe_saturating_block_put(void);
extern void test_half_pipe_block_put_sema(void);
extern void test_pipe_alloc(void);
extern void test_pipe_reader_wait(void);
extern void test_pipe_block_writer_wait(void);
extern void test_pipe_cleanup(void);
#ifdef CONFIG_USERSPACE
extern void test_pipe_user_thread2thread(void);
extern void test_pipe_user_put_fail(void);
extern void test_pipe_user_get_fail(void);
extern void test_resource_pool_auto_free(void);
extern void test_pipe_alloc_not_init(void);
extern void test_pipe_get_null(void);
extern void test_pipe_get_unreach_data(void);
extern void test_pipe_get_unreach_size(void);
extern void test_pipe_put_null(void);
extern void test_pipe_put_unreach_data(void);
extern void test_pipe_put_unreach_size(void);
extern void test_pipe_read_avail_null(void);
extern void test_pipe_write_avail_null(void);
#endif

extern void test_pipe_avail_r_lt_w(void);
extern void test_pipe_avail_w_lt_r(void);
extern void test_pipe_avail_r_eq_w_full(void);
extern void test_pipe_avail_r_eq_w_empty(void);
extern void test_pipe_avail_no_buffer(void);

/* k objects */
extern struct k_pipe pipe, kpipe, khalfpipe, put_get_pipe;
extern struct k_sem end_sema;
extern struct k_stack tstack;
extern struct k_thread tdata;
extern struct k_heap test_pool;

#ifndef CONFIG_USERSPACE
#define dummy_test(_name) \
	static void _name(void) \
	{ \
		ztest_test_skip(); \
	}

dummy_test(test_pipe_user_thread2thread);
dummy_test(test_pipe_user_put_fail);
dummy_test(test_pipe_user_get_fail);
dummy_test(test_resource_pool_auto_free);
dummy_test(test_pipe_alloc_not_init);
dummy_test(test_pipe_get_null);
dummy_test(test_pipe_get_unreach_data);
dummy_test(test_pipe_get_unreach_size);
dummy_test(test_pipe_put_null);
dummy_test(test_pipe_put_unreach_data);
dummy_test(test_pipe_put_unreach_size);
dummy_test(test_pipe_read_avail_null);
dummy_test(test_pipe_write_avail_null);
#endif /* !CONFIG_USERSPACE */

/*test case main entry*/
void test_main(void)
{
	k_thread_access_grant(k_current_get(), &pipe,
			      &kpipe, &end_sema, &tdata, &tstack,
			      &khalfpipe, &put_get_pipe);

	k_thread_heap_assign(k_current_get(), &test_pool);

	ztest_test_suite(pipe_api,
			 ztest_1cpu_unit_test(test_pipe_thread2thread),
			 ztest_1cpu_user_unit_test(test_pipe_user_thread2thread),
			 ztest_1cpu_user_unit_test(test_pipe_user_put_fail),
			 ztest_user_unit_test(test_pipe_user_get_fail),
			 ztest_user_unit_test(test_pipe_alloc_not_init),
			 ztest_user_unit_test(test_pipe_get_null),
			 ztest_user_unit_test(test_pipe_get_unreach_data),
			 ztest_user_unit_test(test_pipe_get_unreach_size),
			 ztest_user_unit_test(test_pipe_put_null),
			 ztest_user_unit_test(test_pipe_put_unreach_data),
			 ztest_user_unit_test(test_pipe_put_unreach_size),
			 ztest_user_unit_test(test_pipe_read_avail_null),
			 ztest_user_unit_test(test_pipe_write_avail_null),
			 ztest_unit_test(test_resource_pool_auto_free),
			 ztest_1cpu_unit_test(test_pipe_put_fail),
			 ztest_unit_test(test_pipe_get_fail),
			 ztest_unit_test(test_half_pipe_put_get),
			 ztest_unit_test(test_pipe_get_put),
			 ztest_unit_test(test_pipe_get_large),
			 ztest_1cpu_unit_test(test_pipe_alloc),
			 ztest_unit_test(test_pipe_cleanup),
			 ztest_unit_test(test_pipe_reader_wait),
			 ztest_unit_test(test_pipe_avail_r_lt_w),
			 ztest_unit_test(test_pipe_avail_w_lt_r),
			 ztest_unit_test(test_pipe_avail_r_eq_w_full),
			 ztest_unit_test(test_pipe_avail_r_eq_w_empty),
			 ztest_unit_test(test_pipe_avail_no_buffer));
	ztest_run_test_suite(pipe_api);
}
