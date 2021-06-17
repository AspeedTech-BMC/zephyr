/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM AArch32 public exception handling
 *
 * ARM AArch32-specific kernel exception handling interface. Included by
 * arm/arch.h.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_EXC_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_EXC_H_

#if defined(CONFIG_CPU_CORTEX_M)
#include <devicetree.h>

#include <arch/arm/aarch32/cortex_m/nvic.h>

/* for assembler, only works with constants */
#define Z_EXC_PRIO(pri) (((pri) << (8 - NUM_IRQ_PRIO_BITS)) & 0xff)

/*
 * In architecture variants with non-programmable fault exceptions
 * (e.g. Cortex-M Baseline variants), hardware ensures processor faults
 * are given the highest interrupt priority level. SVCalls are assigned
 * the highest configurable priority level (level 0); note, however, that
 * this interrupt level may be shared with HW interrupts.
 *
 * In Cortex variants with programmable fault exception priorities we
 * assign the highest interrupt priority level (level 0) to processor faults
 * with configurable priority.
 * The highest priority level may be shared with either Zero-Latency IRQs (if
 * support for the feature is enabled) or with SVCall priority level.
 * Regular HW IRQs are always assigned priority levels lower than the priority
 * levels for SVCalls, Zero-Latency IRQs and processor faults.
 *
 * PendSV IRQ (which is used in Cortex-M variants to implement thread
 * context-switching) is assigned the lowest IRQ priority level.
 */
#if defined(CONFIG_CPU_CORTEX_M_HAS_PROGRAMMABLE_FAULT_PRIOS)
#define _EXCEPTION_RESERVED_PRIO 1
#else
#define _EXCEPTION_RESERVED_PRIO 0
#endif

#define _EXC_FAULT_PRIO 0
#define _EXC_ZERO_LATENCY_IRQS_PRIO 0
#define _EXC_SVC_PRIO COND_CODE_1(CONFIG_ZERO_LATENCY_IRQS, (1), (0))
#define _IRQ_PRIO_OFFSET (_EXCEPTION_RESERVED_PRIO + _EXC_SVC_PRIO)

#define _EXC_IRQ_DEFAULT_PRIO Z_EXC_PRIO(_IRQ_PRIO_OFFSET)

/* Use lowest possible priority level for PendSV */
#define _EXC_PENDSV_PRIO 0xff
#define _EXC_PENDSV_PRIO_MASK Z_EXC_PRIO(_EXC_PENDSV_PRIO)
#endif /* CONFIG_CPU_CORTEX_M */

#ifdef _ASMLANGUAGE
GTEXT(z_arm_exc_exit);
#else
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Additional register state that is not stacked by hardware on exception
 * entry.
 *
 * These fields are ONLY valid in the ESF copy passed into z_arm_fatal_error().
 * When information for a member is unavailable, the field is set to zero.
 */
#if defined(CONFIG_EXTRA_EXCEPTION_INFO)
struct __extra_esf_info {
	_callee_saved_t *callee;
	uint32_t msp;
	uint32_t exc_return;
};
#endif /* CONFIG_EXTRA_EXCEPTION_INFO */

struct __esf {
	struct __basic_sf {
		sys_define_gpr_with_alias(a1, r0);
		sys_define_gpr_with_alias(a2, r1);
		sys_define_gpr_with_alias(a3, r2);
		sys_define_gpr_with_alias(a4, r3);
		sys_define_gpr_with_alias(ip, r12);
		sys_define_gpr_with_alias(lr, r14);
		sys_define_gpr_with_alias(pc, r15);
		uint32_t xpsr;
	} basic;
#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
	float s[16];
	uint32_t fpscr;
	uint32_t undefined;
#endif
#if defined(CONFIG_EXTRA_EXCEPTION_INFO)
	struct __extra_esf_info extra_info;
#endif
};

extern uint32_t z_arm_coredump_fault_sp;

typedef struct __esf z_arch_esf_t;

#ifdef CONFIG_CPU_CORTEX_M
extern void z_arm_exc_exit(void);
#else
extern void z_arm_exc_exit(bool fatal);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_AARCH32_EXC_H_ */
