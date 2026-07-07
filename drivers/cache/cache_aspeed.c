/*
 * Copyright (c) 2022 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/devicetree.h>
#include <cmsis_core.h>

#ifdef CONFIG_SOC_SERIES_AST26XX
/*
 * cache area control: each bit controls 16MB cache area
 *	1: cacheable
 *	0: no-cache
 *
 *	bit[0]: 1st 16MB from 0x0000_0000 to 0x00ff_ffff
 *	bit[1]: 2nd 16MB from 0x0100_0000 to 0x01ff_ffff
 *	...
 *	bit[30]: 31th 16MB from 0x1e00_0000 to 0x1eff_ffff
 *	bit[31]: 32th 16MB from 0x1f00_0000 to 0x1fff_ffff
 */
#define CACHE_AREA_CTRL_REG	0xa40
#define CACHE_INVALID_REG	0xa44
#define CACHE_FUNC_CTRL_REG	0xa48
#define CACHE_AREA_SIZE_LOG2	24
#define CACHE_AREA_MAX_BIT	31
#elif defined(CONFIG_SOC_SERIES_AST10X0)
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
#define CACHE_AREA_CTRL_REG	0xa50
#define CACHE_INVALID_REG	0xa54
#define CACHE_FUNC_CTRL_REG	0xa58
#define CACHE_AREA_SIZE_LOG2	15
#define CACHE_AREA_MAX_BIT	31
#elif defined(CONFIG_SOC_SERIES_AST10x0_G2)
/*
 * PSP (Cortex-M4) cache controller, in SCU:
 *	SCU90C: I-cacheable area control, one bit per 512KB area
 *	SCU910: D-cacheable area control, one bit per 512KB area
 *	SCU914: cache invalidate command
 *	SCU918: cache function control
 *
 * cacheable area control: the 32 bits map the entire 16MB memory
 * region, each bit controls whether one 512KB area is cacheable
 *	1: cacheable
 *	0: no-cache
 *
 *	bit[0]: 1st 512KB from 0x0000_0000 to 0x0007_ffff
 *	bit[1]: 2nd 512KB from 0x0008_0000 to 0x000f_ffff
 *	...
 *	bit[26]: 27th 512KB from 0x00d0_0000 to 0x00d7_ffff
 *	bit[27]: 28th 512KB from 0x00d8_0000 to 0x00df_ffff
 *	bit[31:28]: last 2MB, unused
 */
#define ICACHE_AREA_CTRL_REG	0x90c
#define DCACHE_AREA_CTRL_REG	0x910
#define CACHE_INVALID_REG	0x914
#define CACHE_FUNC_CTRL_REG	0x918
#define CACHE_AREA_SIZE_LOG2	19
#define CACHE_AREA_MAX_BIT	27
#else
#error "Unsupported SOC series"
#endif

#define CACHE_AREA_SIZE	BIT(CACHE_AREA_SIZE_LOG2)

#if DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_cached_memory), okay)
#define CACHED_SRAM_ADDR	DT_REG_ADDR(DT_CHOSEN(zephyr_cached_memory))
#define CACHED_SRAM_SIZE	DT_REG_SIZE(DT_CHOSEN(zephyr_cached_memory))
#elif CONFIG_XIP
/*
 * XIP: cover both the flash and the SRAM.  The two regions are not
 * necessarily contiguous, so span from the flash base to the SRAM end.
 */
#define CACHED_SRAM_ADDR	CONFIG_FLASH_BASE_ADDRESS
#define CACHED_SRAM_SIZE	(CONFIG_SRAM_BASE_ADDRESS + KB(CONFIG_SRAM_SIZE) - \
				 CONFIG_FLASH_BASE_ADDRESS)
#else
#define CACHED_SRAM_ADDR	CONFIG_SRAM_BASE_ADDRESS
#define CACHED_SRAM_SIZE	KB(CONFIG_SRAM_SIZE)
#endif
#define CONFIGURED_CACHED_SRAM_END	(CACHED_SRAM_ADDR + CACHED_SRAM_SIZE - 1)

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
/*
 * Treat the entire 14MB system memory window as cacheable.  This temporarily
 * includes the area currently exposed to the linker as RAM_NC; that region
 * will be removed from the board devicetrees later.
 */
#define CACHED_SRAM_END	MAX(CONFIGURED_CACHED_SRAM_END, \
			    (CACHE_AREA_MAX_BIT + 1) * CACHE_AREA_SIZE - 1)
#else
#define CACHED_SRAM_END	CONFIGURED_CACHED_SRAM_END
#endif

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
BUILD_ASSERT((CACHED_SRAM_END >> CACHE_AREA_SIZE_LOG2) <= CACHE_AREA_MAX_BIT,
	     "cached memory range exceeds the cacheable area");

/*
 * SCU914: invalidate by address.  The 13-bit address field takes
 * addr[12:0]; the cache is 2-way set-associative so the tag cannot be
 * matched, and the hardware invalidates both ways of the set selected
 * by addr[12:5].
 */
#define DCACHE_INVALID(addr)	(BIT(31) | (((addr) & GENMASK(12, 0)) << 16))
#define ICACHE_INVALID(addr)	(BIT(15) | (((addr) & GENMASK(12, 0)) << 0))

#define ICACHE_PREFETCH_CLR	BIT(5)
#define DCACHE_PREFETCH_CLR	BIT(4)
#define ICACHE_RESET		BIT(3)
#define DCACHE_RESET		BIT(2)
#define ICACHE_ENABLE		BIT(1)
#define DCACHE_ENABLE		BIT(0)
#else
#define DCACHE_INVALID(addr)	(BIT(31) | (((addr) & GENMASK(10, 0)) << 16))
#define ICACHE_INVALID(addr)	(BIT(15) | (((addr) & GENMASK(10, 0)) << 0))

#define ICACHE_CLEAN		BIT(2)
#define DCACHE_CLEAN		BIT(1)
#define CACHE_ENABLE		BIT(0)
#endif

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
/*
 * 16KB I-cache + 16KB D-cache: 32B line, 2-way, 256 sets.
 * Addresses 8KB apart (equal addr[12:5]) share a cache set.
 */
#define CACHE_LINE_SIZE_LOG2	5
#define N_CACHE_LINE		512
#define N_CACHE_SET		256
#else
/* cache size = 32B * 128 = 4KB */
#define CACHE_LINE_SIZE_LOG2	5
#define N_CACHE_LINE		128
#endif

#define CACHE_LINE_SIZE		BIT(CACHE_LINE_SIZE_LOG2)
#define CACHE_ALIGNED_ADDR(addr) \
	((addr >> CACHE_LINE_SIZE_LOG2) << CACHE_LINE_SIZE_LOG2)

/* prefetch buffer */
#define PREFETCH_BUF_SIZE	CACHE_LINE_SIZE

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
/*
 * SCU918 also carries the closely-coupled memory control bits
 * (SCU_PSP_SRAM_MASTER_SEL and SCU_PSP_SRAM_MODE_EN, bits [7:6]), so the
 * cache control bits must be updated with read-modify-write.
 */
static void cache_func_ctrl_update(uint32_t clear, uint32_t set)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t key;
	uint32_t reg;

	key = __get_PRIMASK();
	__disable_irq();

	syscon_read_reg(dev, CACHE_FUNC_CTRL_REG, &reg);
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, (reg & ~clear) | set);

	__set_PRIMASK(key);
}

static void aspeed_cache_enable(uint32_t area_reg, uint32_t reset_bits,
				uint32_t enable_bit)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t start_bit, end_bit;

	/* disable the cache while (re)configuring the cacheable area */
	cache_func_ctrl_update(enable_bit, 0);

	start_bit = MIN(CACHE_AREA_MAX_BIT, CACHED_SRAM_ADDR >> CACHE_AREA_SIZE_LOG2);
	end_bit = MIN(CACHE_AREA_MAX_BIT, CACHED_SRAM_END >> CACHE_AREA_SIZE_LOG2);
	syscon_write_reg(dev, area_reg, GENMASK(end_bit, start_bit));

	/* flush the cache and clear the prefetch buffer before enabling */
	cache_func_ctrl_update(0, reset_bits);
	barrier_dsync_fence_full();
	cache_func_ctrl_update(reset_bits, 0);
	barrier_dsync_fence_full();

	cache_func_ctrl_update(0, enable_bit);
}
#else /* !CONFIG_SOC_SERIES_AST10x0_G2 */
static void aspeed_cache_init(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t start_bit, end_bit;

	/* set all cache areas to no-cache by default */
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, 0);

	/* calculate how many areas need to be set */
	start_bit = MIN(CACHE_AREA_MAX_BIT, CACHED_SRAM_ADDR >> CACHE_AREA_SIZE_LOG2);
	end_bit = MIN(CACHE_AREA_MAX_BIT, CACHED_SRAM_END >> CACHE_AREA_SIZE_LOG2);
	syscon_write_reg(dev, CACHE_AREA_CTRL_REG, GENMASK(end_bit, start_bit));

	/* enable cache */
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, CACHE_ENABLE);
}
#endif /* CONFIG_SOC_SERIES_AST10x0_G2 */

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

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
void cache_data_enable(void)
{
	aspeed_cache_enable(DCACHE_AREA_CTRL_REG,
			    DCACHE_RESET | DCACHE_PREFETCH_CLR, DCACHE_ENABLE);
}

void cache_data_disable(void)
{
	cache_func_ctrl_update(DCACHE_ENABLE, 0);
}

void cache_instr_enable(void)
{
	aspeed_cache_enable(ICACHE_AREA_CTRL_REG,
			    ICACHE_RESET | ICACHE_PREFETCH_CLR, ICACHE_ENABLE);
}

void cache_instr_disable(void)
{
	cache_func_ctrl_update(ICACHE_ENABLE, 0);
}
#else /* !CONFIG_SOC_SERIES_AST10x0_G2 */
void cache_data_enable(void)
{
	aspeed_cache_init();
}

void cache_data_disable(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));

	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, 0);
}

void cache_instr_enable(void)
{
	aspeed_cache_init();
}

void cache_instr_disable(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));

	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, 0);
}
#endif /* CONFIG_SOC_SERIES_AST10x0_G2 */

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
int cache_data_invd_all(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t ctrl;
	uint32_t key;

	key = __get_PRIMASK();
	__disable_irq();

	syscon_read_reg(dev, CACHE_FUNC_CTRL_REG, &ctrl);
	ctrl &= ~(DCACHE_RESET | DCACHE_PREFETCH_CLR);

	/* the cache must be disabled before it is reset */
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl & ~DCACHE_ENABLE);
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG,
			 (ctrl & ~DCACHE_ENABLE) | DCACHE_RESET | DCACHE_PREFETCH_CLR);
	barrier_dsync_fence_full();
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl & ~DCACHE_ENABLE);
	barrier_dsync_fence_full();

	/* restore the previous enable state */
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl);

	__set_PRIMASK(key);

	return 0;
}
#else /* !CONFIG_SOC_SERIES_AST10x0_G2 */
int cache_data_invd_all(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t ctrl;
	uint32_t key;

	key = __get_PRIMASK();
	__disable_irq();

	syscon_read_reg(dev, CACHE_FUNC_CTRL_REG, &ctrl);

	ctrl &= ~DCACHE_CLEAN;
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl);

	barrier_dsync_fence_full();
	ctrl |= DCACHE_CLEAN;
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl);
	barrier_dsync_fence_full();

	__set_PRIMASK(key);

	return 0;
}
#endif /* CONFIG_SOC_SERIES_AST10x0_G2 */

int cache_data_invd_range(void *addr, size_t size)
{
	uint32_t aligned_addr, i, n;
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t key;

	if (((uint32_t)addr < CACHED_SRAM_ADDR) ||
	    ((uint32_t)addr > CACHED_SRAM_END)) {
		return 0;
	}

	n = get_n_cacheline((uint32_t)addr, size, &aligned_addr);

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
	/*
	 * Per-address invalidation hits both ways of the set selected by
	 * addr[12:5].  A range covering all the sets is cheaper to handle
	 * with a whole-cache invalidation.
	 */
	if (n >= N_CACHE_SET) {
		return cache_data_invd_all();
	}
#endif

	key = __get_PRIMASK();
	__disable_irq();

	for (i = 0; i < n; i++) {
		syscon_write_reg(dev, CACHE_INVALID_REG, 0);
		syscon_write_reg(dev, CACHE_INVALID_REG, DCACHE_INVALID(aligned_addr));
		aligned_addr += CACHE_LINE_SIZE;
	}
	barrier_dsync_fence_full();

	__set_PRIMASK(key);

	return 0;
}

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
int cache_instr_invd_all(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t ctrl;
	uint32_t key;

	key = __get_PRIMASK();
	__disable_irq();

	syscon_read_reg(dev, CACHE_FUNC_CTRL_REG, &ctrl);
	ctrl &= ~(ICACHE_RESET | ICACHE_PREFETCH_CLR);

	/* the cache must be disabled before it is reset */
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl & ~ICACHE_ENABLE);
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG,
			 (ctrl & ~ICACHE_ENABLE) | ICACHE_RESET | ICACHE_PREFETCH_CLR);
	barrier_isync_fence_full();
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl & ~ICACHE_ENABLE);
	barrier_isync_fence_full();

	/* restore the previous enable state */
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl);

	__set_PRIMASK(key);

	return 0;
}
#else /* !CONFIG_SOC_SERIES_AST10x0_G2 */
int cache_instr_invd_all(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t ctrl;
	uint32_t key;

	key = __get_PRIMASK();
	__disable_irq();

	syscon_read_reg(dev, CACHE_FUNC_CTRL_REG, &ctrl);

	ctrl &= ~ICACHE_CLEAN;
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl);
	barrier_isync_fence_full();
	ctrl |= ICACHE_CLEAN;
	syscon_write_reg(dev, CACHE_FUNC_CTRL_REG, ctrl);
	barrier_isync_fence_full();

	__set_PRIMASK(key);

	return 0;
}
#endif /* CONFIG_SOC_SERIES_AST10x0_G2 */

int cache_instr_invd_range(void *addr, size_t size)
{
	uint32_t aligned_addr, i, n;
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t key;

	if (((uint32_t)addr < CACHED_SRAM_ADDR) ||
	    ((uint32_t)addr > CACHED_SRAM_END)) {
		return 0;
	}

	n = get_n_cacheline((uint32_t)addr, size, &aligned_addr);

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
	/* see cache_data_invd_range() */
	if (n >= N_CACHE_SET) {
		return cache_instr_invd_all();
	}
#endif

	key = __get_PRIMASK();
	__disable_irq();

	for (i = 0; i < n; i++) {
		syscon_write_reg(dev, CACHE_INVALID_REG, 0);
		syscon_write_reg(dev, CACHE_INVALID_REG, ICACHE_INVALID(aligned_addr));
		aligned_addr += CACHE_LINE_SIZE;
	}
	barrier_dsync_fence_full();

	/*
	 * Flush the CPU pipeline so no stale instruction is executed.  Note
	 * this does not clear the instruction prefetch buffer: flows that
	 * modify code (e.g. firmware load) should use cache_instr_invd_all().
	 */
	barrier_isync_fence_full();

	__set_PRIMASK(key);

	return 0;
}

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
int cache_data_flush_all(void)
{
	/* write-through cache: nothing to flush, only drain prior stores */
	barrier_dsync_fence_full();

	return 0;
}

int cache_data_flush_and_invd_all(void)
{
	barrier_dsync_fence_full();

	return cache_data_invd_all();
}
#else /* !CONFIG_SOC_SERIES_AST10x0_G2 */
int cache_data_flush_all(void)
{
	return -ENOTSUP;
}

int cache_data_flush_and_invd_all(void)
{
	return -ENOTSUP;
}
#endif /* CONFIG_SOC_SERIES_AST10x0_G2 */

int cache_data_flush_range(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);

	return -ENOTSUP;
}

int cache_data_flush_and_invd_range(void *addr, size_t size)
{
	int ret;

	if (size == 0U) {
		return 0;
	}

	ret = cache_data_flush_range(addr, size);
	if (ret != 0) {
		return ret;
	}

	/* The flush read-back can refill the last cache line. */
	return cache_data_invd_range(addr, size);
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
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t ctrl;

	syscon_read_reg(dev, CACHE_FUNC_CTRL_REG, &ctrl);

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
	return (ctrl & DCACHE_ENABLE) ? CACHE_LINE_SIZE : 0;
#else
	return (ctrl & CACHE_ENABLE) ? CACHE_LINE_SIZE : 0;
#endif
}
#endif /* CONFIG_DCACHE_LINE_SIZE_DETECT */

#ifdef CONFIG_ICACHE_LINE_SIZE_DETECT
size_t cache_instr_line_size_get(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t ctrl;

	syscon_read_reg(dev, CACHE_FUNC_CTRL_REG, &ctrl);

#ifdef CONFIG_SOC_SERIES_AST10x0_G2
	return (ctrl & ICACHE_ENABLE) ? CACHE_LINE_SIZE : 0;
#else
	return (ctrl & CACHE_ENABLE) ? CACHE_LINE_SIZE : 0;
#endif
}
#endif /* CONFIG_ICACHE_LINE_SIZE_DETECT */
