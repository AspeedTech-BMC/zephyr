/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
extern void test_mheap_malloc_free(void);
extern void test_mheap_calloc(void);
extern void test_mheap_malloc_align4(void);
extern void test_mheap_threadsafe(void);
extern void test_k_aligned_alloc(void);
extern void test_sys_heap_mem_pool_assign(void);
extern void test_malloc_in_isr(void);
extern void test_malloc_in_thread(void);


/**
 * @brief Heap tests
 *
 * @defgroup kernel_heap_tests Heap Memory Tests
 *
 * @ingroup all_tests
 * @{
 * @}
 */
/*test case main entry*/
void test_main(void)
{
	ztest_test_suite(mheap_api,
			 ztest_unit_test(test_mheap_malloc_free),
			 ztest_unit_test(test_mheap_calloc),
			 ztest_unit_test(test_mheap_malloc_align4),
			 ztest_unit_test(test_mheap_threadsafe),
			 ztest_unit_test(test_k_aligned_alloc),
			 ztest_unit_test(test_sys_heap_mem_pool_assign),
			 ztest_unit_test(test_malloc_in_isr),
			 ztest_unit_test(test_malloc_in_thread));
	ztest_run_test_suite(mheap_api);
}
