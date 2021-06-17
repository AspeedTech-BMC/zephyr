/*
 * arch_inlines.h - automatically selects the correct arch_inlines.h file to
 *   include based on the selected architecture.
 */

/*
 * Copyright (c) 2019 Stephanos Ioannidis <root@stephanos.io>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_INLINES_H_
#define ZEPHYR_INCLUDE_ARCH_INLINES_H_

#if defined(CONFIG_X86) || defined(CONFIG_X86_64)
#include <arch/x86/arch_inlines.h>
#elif defined(CONFIG_ARM64)
#include <arch/arm64/arch_inlines.h>
#elif defined(CONFIG_ARC)
#include <arch/arc/arch_inlines.h>
#elif defined(CONFIG_XTENSA)
#include <arch/xtensa/arch_inlines.h>
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_INLINES_H_ */
