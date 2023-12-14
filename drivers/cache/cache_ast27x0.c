/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/arm/aarch32/cortex_m/cmsis.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/sys/barrier.h>

/*
 * cache area control: each bit controls 32KB cache area
 *	1: cacheable
 *	0: no-cache
 *
 *	bit[0]: 1st 32KB from 0x0000_0000 to 0x0000_7fff
 *	bit[1]: 2nd 32KB from 0x0000_8000 to 0x0000_ffff
 *	...
 *	bit[22]: 23th 32KB from 0x000a_8000 to 0x000a_ffff
 *	bit[23]: 24th 32KB from 0x000b_0000 to 0x000b_ffff
 */
#define ICACHE_AREA_CTRL_REG	0xc
#define DCACHE_AREA_CTRL_REG	0x10
#define CACHE_INVALID_REG	0x14
#define CACHE_FUNC_CTRL_REG	0x18
#define CACHE_AREA_SIZE_LOG2	15

#define CACHED_SRAM_ADDR	CONFIG_SRAM_BASE_ADDRESS
#define CACHED_SRAM_SIZE	KB(CONFIG_SRAM_SIZE)
#define CACHED_SRAM_END		(CACHED_SRAM_ADDR + CACHED_SRAM_SIZE - 1)
#define CACHE_AREA_SIZE		BIT(CACHE_AREA_SIZE_LOG2)

#define DCACHE_INVALID(addr)	(BIT(31) | (((addr) & GENMASK(10, 0)) << 16))
#define ICACHE_INVALID(addr)	(BIT(15) | (((addr) & GENMASK(10, 0)) << 0))

#define ICACHE_CLEAN		BIT(3)
#define DCACHE_CLEAN		BIT(2)
#define ICACHE_ENABLE		BIT(1)
#define DCACHE_ENABLE		BIT(0)

/* cache size = 32B * 128 = 4KB */
#define CACHE_LINE_SIZE_LOG2	5
#define CACHE_LINE_SIZE		BIT(CACHE_LINE_SIZE_LOG2)
#define N_CACHE_LINE		128
#define CACHE_ALIGNED_ADDR(addr) (((addr) >> CACHE_LINE_SIZE_LOG2) << CACHE_LINE_SIZE_LOG2)

#define CACHE_DEV		DT_ALIAS(cache)
#define CACHE_BASE		(DT_REG_ADDR(DT_PARENT(CACHE_DEV)) + DT_REG_ADDR(CACHE_DEV))

static uint32_t get_cache_area(void)
{
	uint32_t start_bit, end_bit, max_bit;

	/* calculate how many areas need to be set */
	max_bit = 8 * sizeof(uint32_t) - 1;
	start_bit = MIN(max_bit, CACHED_SRAM_ADDR >> CACHE_AREA_SIZE_LOG2);
	end_bit = MIN(max_bit, CACHED_SRAM_END >> CACHE_AREA_SIZE_LOG2);

	return GENMASK(end_bit, start_bit);
}

/**
 * @brief get aligned address and the number of cachline to be invalied
 * @param [IN] addr - start address to be invalidated
 * @param [IN] size - size in byte
 * @param [OUT] p_aligned_addr - pointer to the cacheline aligned address variable
 * @return number of cacheline to be invalidated
 *
 *  * addr
 *   |--------size-------------|
 * |-----|-----|-----|-----|-----|
 *  \                             \
 *   head                          tail
 *
 * example 1:
 * addr = 0x100 (cacheline aligned), size = 64
 * then head = 0x100, number of cache line to be invalidated = 64 / 32 = 2
 * which means range [0x100, 0x140) will be invalidated
 *
 * example 2:
 * addr = 0x104 (cacheline unaligned), size = 64
 * then head = 0x100, number of cache line to be invalidated = 1 + 64 / 32 = 3
 * which means range [0x100, 0x160) will be invalidated
 */
static uint32_t get_n_cacheline(uint32_t addr, uint32_t size, uint32_t *p_head)
{
	uint32_t n = 0;
	uint32_t tail;

	/* head */
	*p_head = CACHE_ALIGNED_ADDR(addr);

	/* roundup the tail address */
	tail = addr + size + (CACHE_LINE_SIZE - 1);
	tail = CACHE_ALIGNED_ADDR(tail);

	n = (tail - *p_head) >> CACHE_LINE_SIZE_LOG2;

	return n;
}

void cache_data_disable(void)
{
	uintptr_t base = CACHE_BASE;
	uint32_t reg;

	reg = sys_read32(base + CACHE_FUNC_CTRL_REG);
	reg &= ~DCACHE_ENABLE;
	sys_write32(reg, base + CACHE_FUNC_CTRL_REG);
}

void cache_data_enable(void)
{
	uintptr_t base = CACHE_BASE;
	uint32_t reg, area;

	cache_data_disable();

	area = get_cache_area();
	sys_write32(area, base + DCACHE_AREA_CTRL_REG);

	reg = sys_read32(base + CACHE_FUNC_CTRL_REG);
	reg |= DCACHE_ENABLE;
	sys_write32(reg, base + CACHE_FUNC_CTRL_REG);
}

void cache_instr_disable(void)
{
	uintptr_t base = CACHE_BASE;
	uint32_t reg;

	reg = sys_read32(base + CACHE_FUNC_CTRL_REG);
	reg &= ~ICACHE_ENABLE;
	sys_write32(reg, base + CACHE_FUNC_CTRL_REG);
}

void cache_instr_enable(void)
{
	uintptr_t base = CACHE_BASE;
	uint32_t reg, area;

	cache_instr_disable();

	area = get_cache_area();
	sys_write32(area, base + ICACHE_AREA_CTRL_REG);

	reg = sys_read32(base + CACHE_FUNC_CTRL_REG);
	reg |= ICACHE_ENABLE;
	sys_write32(reg, base + CACHE_FUNC_CTRL_REG);
}

int cache_data_invd_all(void)
{
	uintptr_t base = CACHE_BASE;
	uint32_t ctrl;
	unsigned int key = 0;

	ctrl = sys_read32(base + CACHE_FUNC_CTRL_REG);

	/* enter critical section */
	if (!k_is_in_isr()) {
		key = irq_lock();
	}

	ctrl &= ~DCACHE_CLEAN;
	sys_write32(ctrl, base + CACHE_FUNC_CTRL_REG);

	barrier_dsync_fence_full();
	ctrl |= DCACHE_CLEAN;
	sys_write32(ctrl, base + CACHE_FUNC_CTRL_REG);
	barrier_dsync_fence_full();

	/* exit critical section */
	if (!k_is_in_isr()) {
		irq_unlock(key);
	}

	return 0;
}

int cache_data_invd_range(void *addr, size_t size)
{
	uint32_t aligned_addr, i, n;
	uintptr_t base = CACHE_BASE;
	unsigned int key = 0;

	if (((uint32_t)addr < CACHED_SRAM_ADDR) ||
	    ((uint32_t)addr > CACHED_SRAM_END)) {
		return 0;
	}

	/* enter critical section */
	if (!k_is_in_isr()) {
		key = irq_lock();
	}

	n = get_n_cacheline((uint32_t)addr, size, &aligned_addr);

	for (i = 0; i < n; i++) {
		sys_write32(0, base + CACHE_INVALID_REG);
		sys_write32(DCACHE_INVALID(aligned_addr), base + CACHE_INVALID_REG);
		aligned_addr += CACHE_LINE_SIZE;
	}
	barrier_dsync_fence_full();

	/* exit critical section */
	if (!k_is_in_isr()) {
		irq_unlock(key);
	}

	return 0;
}

int cache_instr_invd_all(void)
{
	uintptr_t base = CACHE_BASE;
	uint32_t ctrl;
	unsigned int key = 0;

	ctrl = sys_read32(base + CACHE_FUNC_CTRL_REG);

	/* enter critical section */
	if (!k_is_in_isr()) {
		key = irq_lock();
	}

	ctrl &= ~ICACHE_CLEAN;
	sys_write32(ctrl, base + CACHE_FUNC_CTRL_REG);
	barrier_isync_fence_full();
	ctrl |= ICACHE_CLEAN;
	sys_write32(ctrl, base + CACHE_FUNC_CTRL_REG);
	barrier_isync_fence_full();

	/* exit critical section */
	if (!k_is_in_isr()) {
		irq_unlock(key);
	}

	return 0;
}

int cache_instr_invd_range(void *addr, size_t size)
{
	uint32_t aligned_addr, i, n;
	uintptr_t base = CACHE_BASE;
	unsigned int key = 0;

	if (((uint32_t)addr < CACHED_SRAM_ADDR) ||
	    ((uint32_t)addr > CACHED_SRAM_END)) {
		return 0;
	}

	n = get_n_cacheline((uint32_t)addr, size, &aligned_addr);

	/* enter critical section */
	if (!k_is_in_isr()) {
		key = irq_lock();
	}

	for (i = 0; i < n; i++) {
		sys_write32(0, base + CACHE_INVALID_REG);
		sys_write32(ICACHE_INVALID(aligned_addr), base + CACHE_INVALID_REG);
		aligned_addr += CACHE_LINE_SIZE;
	}
	barrier_dsync_fence_full();

	/* exit critical section */
	if (!k_is_in_isr()) {
		irq_unlock(key);
	}

	return 0;
}

int cache_data_flush_all(void)
{
	return -ENOTSUP;
}

int cache_data_flush_and_invd_all(void)
{
	return -ENOTSUP;
}

int cache_data_flush_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);

	return -ENOTSUP;
}

int cache_data_flush_and_invd_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);

	return -ENOTSUP;
}

int cache_instr_flush_all(void)
{
	return -ENOTSUP;
}

int cache_instr_flush_and_invd_all(void)
{
	return -ENOTSUP;
}

int cache_instr_flush_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);

	return -ENOTSUP;
}

int cache_instr_flush_and_invd_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);

	return -ENOTSUP;
}

#ifdef CONFIG_DCACHE_LINE_SIZE_DETECT
size_t cache_data_line_size_get(void)
{
	uintptr_t base = CACHE_BASE;
	uint32_t ctrl;

	ctrl = sys_read32(base + CACHE_FUNC_CTRL_REG);

	return (ctrl & DCACHE_ENABLE) ? CACHE_LINE_SIZE : 0;
}
#endif /* CONFIG_DCACHE_LINE_SIZE_DETECT */

#ifdef CONFIG_ICACHE_LINE_SIZE_DETECT
size_t cache_instr_line_size_get(void)
{
	uintptr_t base = CACHE_BASE;
	uint32_t ctrl;

	ctrl = sys_read32(base + CACHE_FUNC_CTRL_REG);

	return (ctrl & ICACHE_ENABLE) ? CACHE_LINE_SIZE : 0;
}
#endif /* CONFIG_ICACHE_LINE_SIZE_DETECT */
