/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <sys/mem_manage.h>
#include <toolchain.h>
#include <mmu.h>

/* 32-bit IA32 page tables have no mechanism to restrict execution */
#if defined(CONFIG_X86) && !defined(CONFIG_X86_64) && !defined(CONFIG_X86_PAE)
#define SKIP_EXECUTE_TESTS
#endif

#define BASE_FLAGS	(K_MEM_CACHE_WB)
volatile bool expect_fault;

static uint8_t __aligned(CONFIG_MMU_PAGE_SIZE)
			test_page[2 * CONFIG_MMU_PAGE_SIZE];

void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *pEsf)
{
	printk("Caught system error -- reason %d\n", reason);

	if (expect_fault && reason == 0) {
		expect_fault = false;
		ztest_test_pass();
	} else {
		printk("Unexpected fault during test");
		k_fatal_halt(reason);
	}
}


/* z_phys_map() doesn't have alignment requirements, any oddly-sized buffer
 * can get mapped. This will span two pages.
 */
#define BUF_SIZE	5003
#define BUF_OFFSET	1238

/**
 * Show that mapping an irregular size buffer works and RW flag is respected
 *
 * @ingroup kernel_memprotect_tests
 */
void test_z_phys_map_rw(void)
{
	uint8_t *mapped_rw, *mapped_ro;
	uint8_t *buf = test_page + BUF_OFFSET;

	expect_fault = false;

	/* Map in a page that allows writes */
	z_phys_map(&mapped_rw, z_mem_phys_addr(buf),
		   BUF_SIZE, BASE_FLAGS | K_MEM_PERM_RW);

	/* Initialize buf with some bytes */
	for (int i = 0; i < BUF_SIZE; i++) {
		mapped_rw[i] = (uint8_t)(i % 256);
	}

	/* Map again this time only allowing reads */
	z_phys_map(&mapped_ro, z_mem_phys_addr(buf),
		   BUF_SIZE, BASE_FLAGS);

	/* Check that the mapped area contains the expected data. */
	for (int i = 0; i < BUF_SIZE; i++) {
		zassert_equal(buf[i], mapped_ro[i],
			      "unequal byte at index %d", i);
	}

	/* This should explode since writes are forbidden */
	expect_fault = true;
	mapped_ro[0] = 42;

	printk("shouldn't get here\n");
	ztest_test_fail();
}

#ifndef SKIP_EXECUTE_TESTS
extern char __test_mem_map_start[];
extern char __test_mem_map_end[];

__in_section_unique(test_mem_map) __used
static void transplanted_function(bool *executed)
{
	*executed = true;
}

/**
 * Show that mapping with/withour K_MEM_PERM_EXEC works as expected
 *
 * @ingroup kernel_memprotect_tests
 */
void test_z_phys_map_exec(void)
{
	uint8_t *mapped_exec, *mapped_ro;
	bool executed = false;
	void (*func)(bool *executed);

	expect_fault = false;

	/*
	 * Need to reference the function or else linker would
	 * garbage collected it.
	 */
	func = transplanted_function;

	/* Now map with execution enabled and try to run the copied fn */
	z_phys_map(&mapped_exec, z_mem_phys_addr(__test_mem_map_start),
		   (uintptr_t)(__test_mem_map_end - __test_mem_map_start),
		   BASE_FLAGS | K_MEM_PERM_EXEC);

	func = (void (*)(bool *executed))mapped_exec;
	func(&executed);
	zassert_true(executed, "function did not execute");

	/* Now map without execution and execution should now fail */
	z_phys_map(&mapped_ro, z_mem_phys_addr(__test_mem_map_start),
		   (uintptr_t)(__test_mem_map_end - __test_mem_map_start), BASE_FLAGS);

	func = (void (*)(bool *executed))mapped_ro;
	expect_fault = true;
	func(&executed);

	printk("shouldn't get here\n");
	ztest_test_fail();
}
#else
void test_z_phys_map_exec(void)
{
	ztest_test_skip();
}
#endif /* SKIP_EXECUTE_TESTS */

/**
 * Show that memory mapping doesn't have unintended side effects
 *
 * @ingroup kernel_memprotect_tests
 */
void test_z_phys_map_side_effect(void)
{
	uint8_t *mapped;

	expect_fault = false;

	/* z_phys_map() is supposed to always create fresh mappings.
	 * Show that by mapping test_page to an RO region, we can still
	 * modify test_page.
	 */
	z_phys_map(&mapped, z_mem_phys_addr(test_page),
		   sizeof(test_page), BASE_FLAGS);

	/* Should NOT fault */
	test_page[0] = 42;

	/* Should fault */
	expect_fault = true;
	mapped[0] = 42;
	printk("shouldn't get here\n");
	ztest_test_fail();
}

/**
 * Test that z_phys_unmap() unmaps the memory and it is no longer
 * accessible afterwards.
 *
 * @ingroup kernel_memprotect_tests
 */
void test_z_phys_unmap(void)
{
	uint8_t *mapped;

	expect_fault = false;

	/* Map in a page that allows writes */
	z_phys_map(&mapped, z_mem_phys_addr(test_page),
		   sizeof(test_page), BASE_FLAGS | K_MEM_PERM_RW);

	/* Should NOT fault */
	mapped[0] = 42;

	/* Unmap the memory */
	z_phys_unmap(mapped, sizeof(test_page));

	/* Should fault since test_page is no longer accessible */
	expect_fault = true;
	mapped[0] = 42;
	printk("shouldn't get here\n");
	ztest_test_fail();
}

/**
 * Basic k_mem_map() and k_mem_unmap() functionality
 *
 * Does not exercise K_MEM_MAP_* control flags, just default behavior
 */
void test_k_mem_map_unmap(void)
{
	size_t free_mem, free_mem_after_map, free_mem_after_unmap;
	char *mapped, *last_mapped;
	int i, repeat;

	expect_fault = false;
	last_mapped = NULL;

	free_mem = k_mem_free_get();
	zassert_not_equal(free_mem, 0, "no free memory");
	printk("Free memory: %zu\n", free_mem);

	/* Repeat a couple times to make sure everything still works */
	for (repeat = 1; repeat <= 10; repeat++) {
		mapped = k_mem_map(CONFIG_MMU_PAGE_SIZE, K_MEM_PERM_RW);
		zassert_not_null(mapped, "failed to map memory");
		printk("mapped a page to %p\n", mapped);

		if (last_mapped != NULL) {
			zassert_equal(mapped, last_mapped,
				      "should have mapped at same address");
		}
		last_mapped = mapped;

		/* Page should be zeroed */
		for (i = 0; i < CONFIG_MMU_PAGE_SIZE; i++) {
			zassert_equal(mapped[i], '\x00', "page not zeroed");
		}

		free_mem_after_map = k_mem_free_get();
		printk("Free memory after mapping: %zu\n", free_mem_after_map);
		zassert_equal(free_mem, free_mem_after_map + CONFIG_MMU_PAGE_SIZE,
			"incorrect free memory accounting");

		/* Show we can write to page without exploding */
		(void)memset(mapped, '\xFF', CONFIG_MMU_PAGE_SIZE);
		for (i = 0; i < CONFIG_MMU_PAGE_SIZE; i++) {
			zassert_true(mapped[i] == '\xFF',
				"incorrect value 0x%hhx read at index %d",
				mapped[i], i);
		}

		k_mem_unmap(mapped, CONFIG_MMU_PAGE_SIZE);

		free_mem_after_unmap = k_mem_free_get();
		printk("Free memory after unmapping: %zu\n", free_mem_after_unmap);
		zassert_equal(free_mem, free_mem_after_unmap,
			"k_mem_unmap has not freed physical memory");

		if (repeat == 10) {
			/* Should fault since mapped is no longer accessible */
			expect_fault = true;
			mapped[0] = 42;
			printk("shouldn't get here\n");
			ztest_test_fail();
		}
	}
}

/**
 * Test that the "before" guard page is in place for k_mem_map().
 */
void test_k_mem_map_guard_before(void)
{
	uint8_t *mapped;

	expect_fault = false;

	mapped = k_mem_map(CONFIG_MMU_PAGE_SIZE, K_MEM_PERM_RW);
	zassert_not_null(mapped, "failed to map memory");
	printk("mapped a page: %p - %p\n", mapped,
		mapped + CONFIG_MMU_PAGE_SIZE);

	/* Should NOT fault */
	mapped[0] = 42;

	/* Should fault here in the guard page location */
	expect_fault = true;
	mapped -= sizeof(void *);

	printk("trying to access %p\n", mapped);

	mapped[0] = 42;
	printk("shouldn't get here\n");
	ztest_test_fail();
}

/**
 * Test that the "after" guard page is in place for k_mem_map().
 */
void test_k_mem_map_guard_after(void)
{
	uint8_t *mapped;

	expect_fault = false;

	mapped = k_mem_map(CONFIG_MMU_PAGE_SIZE, K_MEM_PERM_RW);
	zassert_not_null(mapped, "failed to map memory");
	printk("mapped a page: %p - %p\n", mapped,
		mapped + CONFIG_MMU_PAGE_SIZE);

	/* Should NOT fault */
	mapped[0] = 42;

	/* Should fault here in the guard page location */
	expect_fault = true;
	mapped += CONFIG_MMU_PAGE_SIZE + sizeof(void *);

	printk("trying to access %p\n", mapped);

	mapped[0] = 42;
	printk("shouldn't get here\n");
	ztest_test_fail();
}

void test_k_mem_map_exhaustion(void)
{
	/* With demand paging enabled, there is backing store
	 * which extends available memory. However, we don't
	 * have a way to figure out how much extra memory
	 * is available. So skip for now.
	 */
#if !defined(CONFIG_DEMAND_PAGING)
	uint8_t *addr;
	size_t free_mem, free_mem_now, free_mem_expected;
	size_t cnt, expected_cnt;
	uint8_t *last_mapped = NULL;

	free_mem = k_mem_free_get();
	printk("Free memory: %zu\n", free_mem);
	zassert_not_equal(free_mem, 0, "no free memory");

	/* Determine how many times we can map */
	expected_cnt = free_mem / CONFIG_MMU_PAGE_SIZE;

	/* Figure out how many pages we can map within
	 * the remaining virtual address space by:
	 *
	 * 1. Find out the top of available space. This can be
	 *    done by mapping one page, and use the returned
	 *    virtual address (plus itself and guard page)
	 *    to obtain the end address.
	 * 2. Calculate how big this region is from
	 *    Z_FREE_VM_START to end address.
	 * 3. Calculate how many times we can call k_mem_map().
	 *    Remember there are two guard pages for every
	 *    mapping call (hence 1 + 2 == 3).
	 */
	addr = k_mem_map(CONFIG_MMU_PAGE_SIZE, K_MEM_PERM_RW);
	zassert_not_null(addr, "fail to map memory");
	k_mem_unmap(addr, CONFIG_MMU_PAGE_SIZE);

	cnt = POINTER_TO_UINT(addr) + CONFIG_MMU_PAGE_SIZE * 2;
	cnt -= POINTER_TO_UINT(Z_FREE_VM_START);
	cnt /= CONFIG_MMU_PAGE_SIZE * 3;

	/* If we are limited by virtual address space... */
	if (cnt < expected_cnt) {
		expected_cnt = cnt;
	}

	/* Now k_mem_map() until it fails */
	free_mem_expected = free_mem - (expected_cnt * CONFIG_MMU_PAGE_SIZE);
	cnt = 0;
	do {
		addr = k_mem_map(CONFIG_MMU_PAGE_SIZE, K_MEM_PERM_RW);

		if (addr != NULL) {
			*((uintptr_t *)addr) = POINTER_TO_UINT(last_mapped);
			last_mapped = addr;
			cnt++;
		}
	} while (addr != NULL);

	printk("Mapped %zu pages\n", cnt);
	zassert_equal(cnt, expected_cnt,
		      "number of pages mapped: expected %u, got %u",
		      expected_cnt, cnt);

	free_mem_now = k_mem_free_get();
	printk("Free memory now: %zu\n", free_mem_now);
	zassert_equal(free_mem_now, free_mem_expected,
		      "free memory should be %zu", free_mem_expected);

	/* Now free all of them */
	cnt = 0;
	while (last_mapped != NULL) {
		addr = last_mapped;
		last_mapped = UINT_TO_POINTER(*((uintptr_t *)addr));
		k_mem_unmap(addr, CONFIG_MMU_PAGE_SIZE);

		cnt++;
	}

	printk("Unmapped %zu pages\n", cnt);
	zassert_equal(cnt, expected_cnt,
		      "number of pages unmapped: expected %u, got %u",
		      expected_cnt, cnt);

	free_mem_now = k_mem_free_get();
	printk("Free memory now: %zu\n", free_mem_now);
	zassert_equal(free_mem_now, free_mem,
		      "free memory should be %zu", free_mem);
#else
	ztest_test_skip();
#endif /* !CONFIG_DEMAND_PAGING */
}

/* ztest main entry*/
void test_main(void)
{
#ifdef CONFIG_DEMAND_PAGING
	/* This test sets up multiple mappings of RAM pages, which is only
	 * allowed for pinned memory
	 */
	k_mem_pin(test_page, sizeof(test_page));
#endif
	ztest_test_suite(test_mem_map,
			ztest_unit_test(test_z_phys_map_rw),
			ztest_unit_test(test_z_phys_map_exec),
			ztest_unit_test(test_z_phys_map_side_effect),
			ztest_unit_test(test_z_phys_unmap),
			ztest_unit_test(test_k_mem_map_unmap),
			ztest_unit_test(test_k_mem_map_guard_before),
			ztest_unit_test(test_k_mem_map_guard_after),
			ztest_unit_test(test_k_mem_map_exhaustion)
			);
	ztest_run_test_suite(test_mem_map);
}
