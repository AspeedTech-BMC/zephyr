/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_PRICTRL_ASPEED_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_PRICTRL_ASPEED_H_

#include <zephyr/sys/util.h>

/* Privilege control dts parser */
#define PRICTRL_DTS(_n)              DT_DRV_INST(_n)
#define PRICTRL_PROT_DTS(_node)      DT_NODELABEL(_node)
#define PRICTRL_DTS_ARRAY(_type, _grp, _node, _prop)                                               \
	{                                                                                          \
		_type, _grp, DT_PROP_LEN(_node, _prop), (uint16_t[])DT_PROP(_node, _prop)          \
	}
#define PRICTRL_DTS_MASTER(_grp, _node, _prop) PRICTRL_DTS_ARRAY(PRICTRL_MASTER, _grp, _node, _prop)
#define PRICTRL_DTS_CLIENT(_grp, _node, _prop) PRICTRL_DTS_ARRAY(PRICTRL_CLIENT, _grp, _node, _prop)
#define PRICTRL_DTS_MPU_ALLOW(_node_id)                                                            \
	COND_CODE_1(DT_NODE_HAS_PROP(_node_id, allow), ((uint16_t[])DT_PROP(_node_id, allow)),     \
		    (NULL))
#define PRICTRL_DTS_MPU(_node_id, _prop, _idx)                                                     \
	{                                                                                          \
		DT_PROP_BY_PHANDLE_IDX(_node_id, _prop, _idx, protect_start),                      \
		DT_PROP_BY_PHANDLE_IDX(_node_id, _prop, _idx, protect_end),                        \
		DT_PROP_LEN_OR(DT_PHANDLE_BY_IDX(_node_id, _prop, _idx), allow, 0),                \
		PRICTRL_DTS_MPU_ALLOW(DT_PHANDLE_BY_IDX(_node_id, _prop, _idx)),                   \
		DT_PROP_LEN_OR(DT_CHILD(DT_PHANDLE_BY_IDX(_node_id, _prop, _idx), _PRICTRL_LEVEL), \
			       allow, 0),                                                          \
		PRICTRL_DTS_MPU_ALLOW(                                                             \
			DT_CHILD(DT_PHANDLE_BY_IDX(_node_id, _prop, _idx), _PRICTRL_LEVEL)),       \
	},

/* Privilege control register definition */
#define PRICTRL_READ_OFFSET          (0x100)
#define PRICTRL_CLIENT_OFFSET        (0x200)
#define PRICTRL_MPU_H2M_OFFSET       (0x400)
#define PRICTRL_MPU_SLI_OFFSET       (0x500)
#define PRICTRL_MPU_S_GRP_OFFSET     (0x000)
#define PRICTRL_MPU_E_GRP_OFFSET     (0x004)
#define PRICTRL_MPU_WPERM_OFFSET     (0x008)
#define PRICTRL_MPU_RPERM_OFFSET     (0x00C)

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

/* Privilege control reset definition */
#define PRICTRL_RST BIT(6)

/* Privilege control lock definition */
#define PRICTRL_NO_LOCK (0)
#define PRICTRL_LOCK    BIT(7)

/* Privilege control config setting operation */
#define PRICTRL_INIT_DEV(_ms, _grp)                                                                \
	({                                                                                         \
		struct prictrl_dev_cfg __dev_cfg = {0};                                            \
		__dev_cfg.ms = (_ms);                                                              \
		__dev_cfg.group = (_grp);                                                          \
		__dev_cfg.last_group = INVALID_GROUP;                                              \
		__dev_cfg;                                                                         \
	})

#define PRICTRL_SET_DEV(_cfg, _dev, _perm)                                                         \
	do {                                                                                       \
		__typeof__(_cfg) __cfg = (_cfg);                                                   \
		(__cfg)->device = (_dev);                                                          \
		(__cfg)->perm = (_perm);                                                           \
	} while (0)

#define PRICTRL_SHIFT_FIELD(_value, _field) ((_value) << ((_field) * PRICTRL_FIELD_SIZE_IN_BITS))

/* Privilege control mpu hardware definition */
#define PRICTRL_MPU_ADDR_ALIGN     (12)
#define PRICTRL_MPU_BANK            (4)
#define PRICTRL_MPU_MAX_NUM         (4)
#define PRICTRL_MPU_SLI_NUM         (1)
#define PRICTRL_MPU_ADDR_ALIGN_MASK (BIT(PRICTRL_MPU_ADDR_ALIGN) - 1)
#define PRICTRL_MPU_DEFAULT_GRP     GENMASK(7, 2)
#define PRICTRL_MPU_ADDR(_addr)     (((_addr) >> PRICTRL_MPU_ADDR_ALIGN) << 8)
#define PRICTRL_MPU_RESET           BIT(1)
#define PRICTRL_MPU_ENABLE          BIT(0)
#define PRICTRL_MPU_LOCK            BIT(0)

/* Privilege control mpu software definition */
#define PRICTRL_MPU_H2M0_MASK BIT(12)
#define PRICTRL_MPU_H2M1_MASK BIT(13)
#define PRICTRL_MPU_SLI_MASK  BIT(14)

/* Privilege control structure */
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

struct prictrl_dev_dts {
	uint16_t dev;
	uint16_t perm;
};

struct prictrl_dev_cfg {
	enum prictrl_ms ms;
	uint16_t device;
	uint16_t perm;
	uint8_t group;
	uint8_t last_group;
};

struct prictrl_dev_list {
	enum prictrl_ms ms;
	uint8_t group;
	uint8_t device_num;
	uint16_t *device;
};

struct prictrl_mpu_cfg {
	/* Region start and end address */
	uintptr_t start;
	uintptr_t end;

	/* Level1 mpu config, dramc protection is level1 mpu. */
	uint8_t l1_num;
	uint16_t *l1_dev;

	/* Level2 mpu config, sli/h2m protection is level2 mpu. */
	uint8_t l2_num;
	uint16_t *l2_dev;
};

struct prictrl_aspeed_config {
	uintptr_t reg;
	uint8_t master_num;
	uint32_t master_max;
	uint8_t client_num;
	uint32_t client_max;
	struct prictrl_dev_list *master;
	struct prictrl_dev_list *client;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_PRICTRL_ASPEED_H_ */
