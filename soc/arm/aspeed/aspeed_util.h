/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2021 ASPEED Technology Inc.
 */
#ifndef ZEPHYR_SOC_ARM_ASPEED_UTIL_H_
#define ZEPHYR_SOC_ARM_ASPEED_UTIL_H_
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/toolchain/gcc.h>

/* gcc.h doesn't define __section but checkpatch.pl will complain for this. so
 * temporarily add a macro here.
 */
#ifndef __section
#define __section(x)    __attribute__((__section__(x)))
#endif

/* to make checkpatch.pl happy */
#define ALIGNED16_SECTION(name)         (aligned(16), section(name))
#define __section_aligned16(name)       __attribute__(ALIGNED16_SECTION(name))

/* non-cached (DMA) memory */
#define NON_CACHED_BSS                  __section("RAM_NC")
#define NON_CACHED_BSS_ALIGN16          __section_aligned16("RAM_NC")

/* 64-bit Aspeed SOC ID */
#define ASPEED_SOC_ID_AST1030A0		0x8000000080000000
#define ASPEED_SOC_ID_AST1030A1		0x8001000080010000
#define ASPEED_SOC_ID_AST1035A1		0x8001010080010100
#define ASPEED_SOC_ID_AST1060A1		0xA0010000A0010000
#define ASPEED_SOC_ID_AST1060A2		0xA0030000A0030000
#define ASPEED_SOC_ID_AST1060A2_ENG	0x8003000080030000

#define ASPEED_SOC_ID_AST2600A0		0x0500030305000303
#define ASPEED_SOC_ID_AST2600A1		0x0501030305010303
#define ASPEED_SOC_ID_AST2620A1		0x0501020305010203
#define ASPEED_SOC_ID_AST2600A2		0x0502030305010303
#define ASPEED_SOC_ID_AST2620A2		0x0502020305010203
#define ASPEED_SOC_ID_AST2600A3		0x0503030305030303
#define ASPEED_SOC_ID_AST2620A3		0x0503020305030203

#define reg_read_poll_timeout(map, reg, val, cond, sleep_ms, timeout_ms)	    \
	({									    \
		uint32_t __timeout_tick = Z_TIMEOUT_MS(timeout_ms).ticks;	    \
		uint32_t __start = sys_clock_tick_get_32();			    \
		int __ret = 0;							    \
		for (;;) {							    \
			val.value = map->reg.value;				    \
			if (cond) {						    \
				break;						    \
			}							    \
			if ((sys_clock_tick_get_32() - __start) > __timeout_tick) { \
				__ret = -ETIMEDOUT;				    \
				break;						    \
			}							    \
			if (sleep_ms) {						    \
				k_msleep(sleep_ms);				    \
			}							    \
		}								    \
		__ret;								    \
	})

#define	DEBUG_HALT()	{ volatile int halt = 1; while (halt) { __asm__ volatile("nop"); } }
#endif
