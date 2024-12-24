/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_PRICTRL_ASPEED_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_PRICTRL_ASPEED_H_

#include <zephyr/sys/util.h>

/* Privilege control util definition */
#define PRICTRL_NARG(...)  PRICTRL_NARG_(__VA_ARGS__, PRICTRL_RSEQ_N())
#define PRICTRL_NARG_(...) PRICTRL_ARG_N(__VA_ARGS__)
#define PRICTRL_ARG_N(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17,  \
		      _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32,   \
		      _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47,   \
		      _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, _61, _62,   \
		      _63, N, ...)                                                                 \
	N
#define PRICTRL_RSEQ_N()                                                                           \
	63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42,    \
		41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22,    \
		21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0

/* Privilege control register definition */
#define PRICTRL_READ_OFFSET          (0x100)
#define PRICTRL_CLIENT_OFFSET        (0x200)
#define PRICTRL_CPU_SMP_SCRATCH_0    (0x12C02780)
#define PRICTRL_CPU_SMP_EP0          (0x12c02110)
#define PRICTRL_CPU_SMP_EP1          (0x12c02114)
#define PRICTRL_CPU_SMP_EP2          (0x12c02118)
#define PRICTRL_CPU_SMP_EP3          (0x12c0211C)
#define PRICTRL_CPU_CA35_REL         (0x12c0210c)
#define PRICTRL_WDT_BASE             (0x14C37380)
#define PRICTRL_WDT_COUNTER          (PRICTRL_WDT_BASE)
#define PRICTRL_WDT_RELOAD_VALUE     (PRICTRL_WDT_BASE + 0x04)
#define PRICTRL_WDT_RESTART          (PRICTRL_WDT_BASE + 0x08)
#define PRICTRL_WDT_CONTROL          (PRICTRL_WDT_BASE + 0x0C)
#define PRICTRL_WDT_TIMEOUT_STAT     (PRICTRL_WDT_BASE + 0x10)
#define PRICTRL_WDT_CLR_TIMEOUT_STAT (PRICTRL_WDT_BASE + 0x14)
#define PRICTRL_WDT_RESET_MASK_1     (PRICTRL_WDT_BASE + 0x1C)
#define PRICTRL_WDT_RESET_MASK_2     (PRICTRL_WDT_BASE + 0x20)
#define PRICTRL_WDT_RESET_MASK_3     (PRICTRL_WDT_BASE + 0x24)
#define PRICTRL_WDT_RESET_MASK_4     (PRICTRL_WDT_BASE + 0x28)
#define PRICTRL_WDT_RESET_MASK_5     (PRICTRL_WDT_BASE + 0x2C)

/* Privilege control register attribute definition */
#define PRICTRL_FIELD_SIZE_IN_BITS    (8)
#define PRICTRL_REGISTER_SIZE_IN_BITS (32)
#define PRICTRL_FILED_NUM_PER_REG     (PRICTRL_REGISTER_SIZE_IN_BITS / PRICTRL_FIELD_SIZE_IN_BITS)

/* Privilege control group definition */
#define NO_PERM_GROUP         (0)
#define BOOT_MCU_GROUP        BIT(0)
#define SSP_GROUP             BIT(1)
#define TSP_GROUP             BIT(2)
#define S_CA35_GROUP          BIT(3)
#define NS_CA35_GROUP         BIT(4)
#define DP_MCU_GROUP          BIT(5)
#define INVALID_GROUP         (0xFF)
#define PRICTRL_GROUP_MASK    GENMASK(5, 0)
#define PRICTRL_GROUP_DEFAULT GENMASK(5, 0)

/* Privilege control lock definition */
#define PRICTRL_NO_LOCK (0)
#define PRICTRL_LOCK    BIT(7)

/* Privilege control config setting operation */
#define DEFINE_MASTER_DEV(_cpu_io, _group, ...)                                                    \
	{                                                                                          \
		_cpu_io, PRICTRL_MASTER, _group, PRICTRL_NARG(__VA_ARGS__), (uint16_t[])           \
		{                                                                                  \
			__VA_ARGS__                                                                \
		}                                                                                  \
	}
#define DEFINE_CLIENT_DEV(_cpu_io, _group, _prop)                                                  \
	{                                                                                          \
		_cpu_io, PRICTRL_CLIENT, _group, DT_PROP_LEN(DT_DRV_INST(0), _prop),               \
			(uint16_t[])DT_PROP(DT_DRV_INST(0), _prop)                                 \
	}
#define PRICTRL_SET_DEV(_cfg, _cpu_io, _ms, _dev, _grp, _lck)                                      \
	do {                                                                                       \
		__typeof__(_cfg) __cfg = (_cfg);                                                   \
		(__cfg)->cpu_io = (_cpu_io);                                                       \
		(__cfg)->ms = (_ms);                                                               \
		(__cfg)->device = (_dev);                                                          \
		(__cfg)->group = (_grp);                                                           \
		(__cfg)->last_group = INVALID_GROUP;                                               \
		(__cfg)->lock = (_lck);                                                            \
	} while (0)
#define PRICTRL_SHIFT_FIELD(_value, _field) ((_value) << ((_field) * PRICTRL_FIELD_SIZE_IN_BITS))

/* Privilege control structure */
enum prictrl_cpu_io {
	PRICTRL_CPU_DIE = 0,
	PRICTRL_IO_DIE,
	PRICTRL_CPU_IO_DIE,
	PRICTRL_CPU_IO_END,
};

enum prictrl_rw {
	PRICTRL_WRITE = 0,
	PRICTRL_READ = 1,
	PRICTRL_RW_END,
};

enum prictrl_ms {
	PRICTRL_MASTER = 0,
	PRICTRL_CLIENT = 1,
	PRICTRL_MS_END,
};

enum prictrl_cpu_master {
	C_M_CPU_S_USER,
	C_M_CPU_S_PRI,
	C_M_CPU_NS_USER,
	C_M_CPU_NS_PRI,
	C_M_SSP_I_USER,
	C_M_SSP_I_PRI,
	C_M_SSP_D_USER,
	C_M_SSP_D_PRI,
	C_M_SSP_S_USER,
	C_M_SSP_S_PRI,
	C_M_DP_MCU = 15,
	C_M_TSP_S_USER,
	C_M_TSP_S_PRI,
	C_M_LIST_END,
};

enum prictrl_io_master {
	IO_M_MCU0_I,
	IO_M_MCU0_D,
	IO_M_LIST_END,
};

struct prictrl_dev_cfg {
	enum prictrl_cpu_io cpu_io;
	enum prictrl_ms ms;
	uint16_t device;
	uint8_t group;
	uint8_t last_group;
	uint8_t lock;
};

struct prictrl_list_cfg {
	enum prictrl_cpu_io cpu_io;
	enum prictrl_ms ms;
	uint8_t group;
	uint8_t device_num;
	uint16_t *device;
};

struct prictrl_aspeed_config {
	uintptr_t cpu_base;
	uintptr_t io_base;
	struct prictrl_list_cfg *master;
	struct prictrl_list_cfg *client;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_PRICTRL_ASPEED_H_ */
