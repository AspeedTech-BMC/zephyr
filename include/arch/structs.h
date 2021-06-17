/*
 * Copyright (c) BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The purpose of this file is to provide essential/minimal architecture-
 * specific structure definitions to be included in generic kernel
 * structures.
 *
 * The following rules must be observed:
 *  1. arch/structs.h shall not depend on kernel.h both directly and
 *     indirectly (i.e. it shall not include any header files that include
 *     kernel.h in their dependency chain).
 *  2. kernel.h shall imply arch/structs.h via kernel_structs.h , such that
 *     it shall not be necessary to include arch/structs.h explicitly when
 *     kernel.h is included.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_STRUCTS_H_
#define ZEPHYR_INCLUDE_ARCH_STRUCTS_H_

#if !defined(_ASMLANGUAGE)

#if defined(CONFIG_ARM64)
#include <arch/arm64/structs.h>
#else

/* Default definitions when no architecture specific definitions exist. */

/* Per CPU architecture specifics (empty) */
struct _cpu_arch {
};

#endif

/* typedefs to be used with GEN_OFFSET_SYM(), etc. */
typedef struct _cpu_arch _cpu_arch_t;

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_STRUCTS_H_ */
