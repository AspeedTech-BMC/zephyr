/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2021 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_SOC_ARM_ASPEED_AST26XX_SOC_H_
#define ZEPHYR_SOC_ARM_ASPEED_AST26XX_SOC_H_

#include <sys/util.h>
#include <devicetree.h>

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#define reg_read_poll_timeout(map, reg, val, cond, sleep_tick, timeout_tick) \
	({								     \
		uint32_t __timeout_tick = (timeout_tick);		     \
		uint32_t __start = z_tick_get_32();			     \
		int __ret = 0;						     \
		for (;;) {						     \
			val.value = map->reg.value;			     \
			if (cond)					     \
				break;					     \
			if ((z_tick_get_32() - __start) > __timeout_tick) {  \
				__ret = -ETIMEDOUT;			     \
				break;					     \
			}						     \
			if (sleep_tick)					     \
				k_sleep(K_TICKS(sleep_tick));		     \
		}							     \
		__ret;							     \
	})

/* Common clock control device name for all Ast26xx series */
#define ASPEED_CLK_CTRL_NAME DT_INST_CLOCKS_LABEL(0)
/* Common reset control device name for all Ast26xx series */
#define ASPEED_RST_CTRL_NAME DT_INST_RESETS_LABEL(0)

/* non-cached (DMA) memory */
#if (CONFIG_SRAM_NC_SIZE > 0)
#define SECTION_ALIGNED16(name)		aligned(16), section(name)
#define NON_CACHED_BSS  __section(".nocache.bss")
#define NON_CACHED_BSS_ALIGN16                                                 \
	__attribute__((SECTION_ALIGNED16(".nocache.bss")))
#else
#define NON_CACHED_BSS
#define NON_CACHED_BSS_ALIGN16  __aligned(16)
#endif /* end of "#if (CONFIG_SRAM_NC_SIZE > 0)" */
/* CMSIS required definitions */
typedef enum {
	Reset_IRQn              = -15,
	NonMaskableInt_IRQn     = -14,
	HardFault_IRQn          = -13,
#if defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	MemoryManagement_IRQn   = -12,
	BusFault_IRQn           = -11,
	UsageFault_IRQn         = -10,
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
	SecureFault_IRQn        = -9,
#endif  /* CONFIG_ARM_SECURE_FIRMWARE */
#endif  /* CONFIG_ARMV7_M_ARMV8_M_MAINLINE */
	SVCall_IRQn             =  -5,
	DebugMonitor_IRQn       =  -4,
	PendSV_IRQn             =  -2,
	SysTick_IRQn            =  -1,
/*
 * CMSIS IRQn_Type enum is broken relative to ARM GNU compiler.
 *
 * So add the MAX_IRQn to enlarge the size of IRQn_Type to avoid
 * ARM compiler from sign extending IRQn_Type values higher than 0x80
 * into negative IRQ values, which causes hard-to-debug Hard Faults.
 */
	MAX_IRQn                = CONFIG_NUM_IRQS,
} IRQn_Type;

#define __VTOR_PRESENT          1U
#define __NVIC_PRIO_BITS        NUM_IRQ_PRIO_BITS
#define __Vendor_SysTickConfig  0U
#define __FPU_PRESENT           0U
#define __MPU_PRESENT           0U

#define PHY_SRAM_ADDR           (*(volatile uint32_t *)(0x7e6e2a04))
#define TO_PHY_ADDR(addr)       (PHY_SRAM_ADDR + (uint32_t)(addr))
#define TO_VIR_ADDR(addr)       ((uint32_t)(addr) - PHY_SRAM_ADDR)

#endif /* ZEPHYR_SOC_ARM_ASPEED_AST26XX_SOC_H_*/
