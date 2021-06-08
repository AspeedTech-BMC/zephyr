#include <stdint.h>
#include <kernel.h>
/**
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
#define CACHE_AREA_SIZE_LOG2	24
#define CACHE_AREA_SIZE		(1 << CACHE_AREA_SIZE_LOG2)

#define CACHE_INVALID_REG	0xa44
#define DCACHE_INVALID(addr)	(BIT(31) | ((addr & GENMASK(10, 0)) << 16))
#define ICACHE_INVALID(addr)	(BIT(15) | ((addr & GENMASK(10, 0)) << 0))

#define CACHE_FUNC_CTRL_REG	0xa48
#define ICACHE_CLEAN		BIT(2)
#define DCACHE_CLEAN		BIT(1)
#define CACHE_EANABLE		BIT(0)

/* cache size = 32B * 128 = 4KB */
#define CACHE_LINE_SIZE_LOG2    5
#define CACHE_LINE_SIZE         (1 << CACHE_LINE_SIZE_LOG2)
#define N_CACHE_LINE            128
#define CACHE_ALIGNED_ADDR(addr) \
	((addr >> CACHE_LINE_SIZE_LOG2) << CACHE_LINE_SIZE_LOG2)

/* prefetch buffer */
#define PREFETCH_BUF_SIZE       CACHE_LINE_SIZE

#define CACHED_SRAM_ADDR        DT_REG_ADDR_BY_IDX(DT_NODELABEL(sdram0), 0)
#define CACHED_SRAM_SIZE        DT_REG_SIZE_BY_IDX(DT_NODELABEL(sdram0), 0)

void aspeed_cache_init(void)
{
	uint32_t base = DT_REG_ADDR(DT_NODELABEL(syscon));
	uint32_t sram_c_end = CACHED_SRAM_ADDR + CACHED_SRAM_SIZE - 1;
	uint32_t start_bit, end_bit;

	/* set all cache areas to no-cache by default */
	sys_write32(0, base + CACHE_FUNC_CTRL_REG);

	/* calculate how many areas need to be set */
	start_bit = CACHED_SRAM_ADDR >> CACHE_AREA_SIZE_LOG2;
	end_bit = sram_c_end >> CACHE_AREA_SIZE_LOG2;
	sys_write32(GENMASK(end_bit, start_bit), base + CACHE_AREA_CTRL_REG);

	/* enable cache */
	sys_write32(CACHE_EANABLE, base + CACHE_FUNC_CTRL_REG);
}
