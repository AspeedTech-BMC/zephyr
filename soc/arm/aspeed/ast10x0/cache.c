#include <stdint.h>
#include <kernel.h>
/**
 * cache area control: each bit controls 32KB cache area
 * 	1: cacheable
 * 	0: no-cache
 * 
 * 	bit[0]: 1st 32KB from 0x0000_0000 to 0x0000_7fff
 *	bit[1]: 2nd 32KB from 0x0000_8000 to 0x0000_ffff
 *	...
 *	bit[22]: 23th 32KB from 0x000a_8000 to 0x000a_ffff
 *	bit[23]: 24th 32KB from 0x000b_0000 to 0x000b_ffff
 */
#define CACHE_AREA_CTRL_REG		0xa50
#define CACHE_AREA_SIZE_LOG2		15
#define CACHE_AREA_SIZE			(1 << CACHE_AREA_SIZE_LOG2)

#define CACHE_INVALID_REG		0xa54
#define DCACHE_INVALID(addr)		(BIT(31) | ((addr & GENMASK(10, 0)) << 16))
#define ICACHE_INVALID(addr)		(BIT(15) | ((addr & GENMASK(10, 0)) << 0))

#define CACHE_FUNC_CTRL_REG		0xa58
#define ICACHE_CLEAN			BIT(2)
#define DCACHE_CLEAN			BIT(1)
#define CACHE_EANABLE			BIT(0)

/* cache size = 32B * 128 = 4KB */
#define CACHE_LINE_SIZE_LOG2		5
#define CACHE_LINE_SIZE			(1 << CACHE_LINE_SIZE_LOG2)
#define N_CACHE_LINE			128
#define CACHE_ALIGNED_ADDR(addr)	((addr >> CACHE_LINE_SIZE_LOG2) << CACHE_LINE_SIZE_LOG2)

/* prefetch buffer */
#define PREFETCH_BUF_SIZE		CACHE_LINE_SIZE

void aspeed_cache_init(void)
{
	uint32_t base = DT_REG_ADDR(DT_NODELABEL(syscon));
	uint32_t sram_c_start = DT_REG_ADDR_BY_IDX(DT_NODELABEL(sram0), 0);
	uint32_t sram_c_end = sram_c_start + DT_REG_SIZE_BY_IDX(DT_NODELABEL(sram0), 0) - 1;
	uint32_t start_bit, end_bit, config;

	sys_write32(0, base + CACHE_FUNC_CTRL_REG);
	start_bit = sram_c_start >> CACHE_AREA_SIZE_LOG2;
	end_bit = sram_c_end >> CACHE_AREA_SIZE_LOG2;
	config = GENMASK(end_bit, start_bit);
	sys_write32(config, base + CACHE_AREA_CTRL_REG);
	sys_write32(CACHE_EANABLE, base + CACHE_FUNC_CTRL_REG);
}
