/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_prictrl

#include <soc.h>
#include <errno.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/misc/ast27xx_prictrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "prictrl_aspeed.h"

LOG_MODULE_REGISTER(prictrl_aspeed, LOG_LEVEL_INF);

/**********************************************************************
 * Get privilege control configuration register
 **********************************************************************/
static uintptr_t prictrl_get_reg_base(const struct prictrl_aspeed_config *cfg, enum prictrl_ms ms,
				      enum prictrl_rw rw)
{
	uintptr_t reg_offset = 0;

	if (!cfg)
		return -EINVAL;

	if (ms != PRICTRL_MASTER && ms != PRICTRL_CLIENT)
		return -EINVAL;

	if (rw != PRICTRL_WRITE && rw != PRICTRL_READ)
		return -EINVAL;

	reg_offset += (ms == PRICTRL_MASTER ? 0 : PRICTRL_CLIENT_OFFSET);
	reg_offset += (rw == PRICTRL_WRITE ? 0 : PRICTRL_READ_OFFSET);

	return cfg->reg + reg_offset;
}

static uintptr_t prictrl_get_reg_offset(const struct prictrl_aspeed_config *cfg, enum prictrl_ms ms,
					int device)
{
	if (!cfg)
		return -EINVAL;

	if (ms != PRICTRL_MASTER && ms != PRICTRL_CLIENT)
		return -EINVAL;

	if (ms == PRICTRL_MASTER && device >= cfg->master_max)
		return -EINVAL;

	if (ms == PRICTRL_CLIENT && device >= cfg->client_max)
		return -EINVAL;

	return ROUND_DOWN(device, PRICTRL_FILED_NUM_PER_REG);
}

static uint8_t prictrl_get_reg_field(uint16_t device)
{
	return (device % PRICTRL_FILED_NUM_PER_REG);
}

static uintptr_t prictrl_get_reg_addr(const struct prictrl_aspeed_config *cfg, enum prictrl_rw rw,
				      struct prictrl_dev_cfg *dev_cfg)
{
	uintptr_t prictrl_base = 0;
	uintptr_t prictrl_offset = 0;

	prictrl_base = prictrl_get_reg_base(cfg, dev_cfg->ms, rw);
	if (prictrl_base == -EINVAL)
		return -EINVAL;

	prictrl_offset = prictrl_get_reg_offset(cfg, dev_cfg->ms, dev_cfg->device);
	if (prictrl_offset == -EINVAL)
		return -EINVAL;

	return prictrl_base + prictrl_offset;
}

/**********************************************************************
 * Operate privilege control configuration register
 **********************************************************************/
static int prictrl_get_perm(const struct prictrl_aspeed_config *cfg, enum prictrl_rw rw,
			    struct prictrl_dev_cfg *dev_cfg)
{
	uint8_t field = 0;
	uintptr_t addr = 0;

	if (!dev_cfg)
		return -EINVAL;

	/* Get the config register address */
	addr = prictrl_get_reg_addr(cfg, rw, dev_cfg);
	if (addr == -EINVAL)
		return -EINVAL;

	/* Get the config register field */
	field = prictrl_get_reg_field(dev_cfg->device);

	/* Get group setting from privilege control register */
	dev_cfg->last_group =
		FIELD_GET(PRICTRL_SHIFT_FIELD(PRICTRL_GROUP_MASK, field), sys_read32(addr));

	return 0;
}

static int prictrl_set_perm(const struct prictrl_aspeed_config *cfg, enum prictrl_rw rw,
			    struct prictrl_dev_cfg *dev_cfg)
{
	uint8_t field = 0;
	uint8_t val = 0;
	uintptr_t addr = 0;

	if (!dev_cfg)
		return -EINVAL;

	/* Get the config register address */
	addr = prictrl_get_reg_addr(cfg, rw, dev_cfg);
	if (addr == -EINVAL)
		return -EINVAL;

	/* Get the config register field */
	field = prictrl_get_reg_field(dev_cfg->device);

	/* Get the configuration from prictrl_dev_cfg */
	val = PRICTRL_GROUP_MASK & dev_cfg->group;

	/* If it is the first time programming, clear the default config */
	prictrl_get_perm(cfg, rw, dev_cfg);
	if (dev_cfg->last_group == PRICTRL_GROUP_DEFAULT)
		sys_clear_bits(addr, PRICTRL_SHIFT_FIELD(PRICTRL_GROUP_MASK & ~val, field));

	/* Set group configuration */
	sys_set_bits(addr, PRICTRL_SHIFT_FIELD(val, field));

	LOG_DBG("(addr:field:value) = (0x%08lx:0x%02x:0x%02x), readback: 0x%08x", addr, field, val,
		sys_read32(addr));

	return 0;
}

static int prictrl_lock_perm(const struct prictrl_aspeed_config *cfg, enum prictrl_rw rw,
			     struct prictrl_dev_cfg *dev_cfg)
{
	uint8_t field = 0;
	uintptr_t addr = 0;

	if (!dev_cfg)
		return -EINVAL;

	/* Get the config register address */
	addr = prictrl_get_reg_addr(cfg, rw, dev_cfg);
	if (addr == -EINVAL)
		return -EINVAL;

	/* Get the config register field */
	field = prictrl_get_reg_field(dev_cfg->device);

	/* Set group lock */
	sys_set_bits(addr, PRICTRL_SHIFT_FIELD(PRICTRL_LOCK, field));

	LOG_DBG("(addr:field:lock) = (0x%08lx:0x%02x:0x01), readback: 0x%08x", addr, field,
		sys_read32(addr));

	return 0;
}

/**********************************************************************
 * Privilege control application programming interface
 **********************************************************************/
static int prictrl_config_group(const struct prictrl_aspeed_config *cfg,
				struct prictrl_dev_cfg *dev_cfg, bool lock)
{
	int ret = 0;

	if (!cfg || !dev_cfg)
		return -EINVAL;

	if (dev_cfg->ms != PRICTRL_MASTER && dev_cfg->ms != PRICTRL_CLIENT)
		return -EINVAL;

	if (dev_cfg->ms == PRICTRL_MASTER && dev_cfg->device >= cfg->master_max)
		return -EINVAL;

	if (dev_cfg->ms == PRICTRL_CLIENT && dev_cfg->device >= cfg->client_max)
		return -EINVAL;

	LOG_DBG("%s (dev:perm:group:lock) = (0x%02x:0x%02x:0x%02x:0x%02x)",
		dev_cfg->ms == PRICTRL_MASTER ? "Master" : "Client", dev_cfg->device, dev_cfg->perm,
		dev_cfg->group, lock);

	/* Setup write protection */
	ret = !lock ? prictrl_set_perm(cfg, PRICTRL_WRITE, dev_cfg)
		    : prictrl_lock_perm(cfg, PRICTRL_WRITE, dev_cfg);

	/* Only set permission if write protection setting ok and dts denote read protection */
	if (!ret && dev_cfg->perm == RW_PROT) {
		ret = !lock ? prictrl_set_perm(cfg, PRICTRL_READ, dev_cfg)
			    : prictrl_lock_perm(cfg, PRICTRL_READ, dev_cfg);
	}

	return ret;
}

static int prictrl_list_config_group(const struct prictrl_aspeed_config *cfg,
				     struct prictrl_dev_list *list, bool lock)
{
	int ret = 0;
	struct prictrl_dev_cfg dev_cfg = PRICTRL_INIT_DEV(list->ms, list->group);
	struct prictrl_dev_dts *dev_dts = NULL;

	if (!list || !cfg)
		return -EINVAL;

	for (dev_dts = (struct prictrl_dev_dts *)list->device;
	     dev_dts < (struct prictrl_dev_dts *)(list->device + list->device_num); dev_dts++) {
		PRICTRL_SET_DEV(&dev_cfg, dev_dts->dev, dev_dts->perm);
		ret = prictrl_config_group(cfg, &dev_cfg, lock);
		if (ret) {
			LOG_ERR("%s %d's %d in %d fail.\n", !lock ? "Set" : "Lock", list->ms,
				dev_dts->dev, list->group);
			return -EINVAL;
		}
	}

	return ret;
}

static int prictrl_setup(const struct prictrl_aspeed_config *cfg, struct prictrl_dev_list *list,
			 int num)
{
	int i = 0;
	int ret = 0;

	if (!cfg)
		return -EINVAL;

	/* Configure the privilege control groups */
	for (i = 0; i < num; i++)
		ret |= prictrl_list_config_group(cfg, &list[i], false);

	/* Lock the privilege control groups */
	for (i = 0; i < num; i++)
		ret |= prictrl_list_config_group(cfg, &list[i], true);

	return 0;
}

static int prictrl_hw_init(const struct device *dev)
{
	int i = 0;
	const uint32_t init_val = 0x7F7F7F7F;
	const uint32_t magic = 0x7F7F7F7E;
	const struct prictrl_aspeed_config *cfg = dev ? dev->config : NULL;

	if (!dev || !cfg)
		return -EINVAL;

	/* Initialize privilege control configuration register */
	for (i = 0; i < 8; i++)
		sys_write32(init_val, (cfg->reg + i * 4));

	for (i = 0; i < 8; i++)
		sys_write32(init_val, (cfg->reg + 0x100 + i * 4));

	for (i = 0; i < 64; i++)
		sys_write32(init_val, (cfg->reg + 0x200 + i * 4));

	for (i = 0; i < 64; i++)
		sys_write32(init_val, (cfg->reg + 0x300 + i * 4));

	/* Check whether privilege control is ready */
	sys_write32(magic, cfg->reg);
	if (sys_read32(cfg->reg) != magic)
		return -EAGAIN;
	sys_write32(init_val, cfg->reg);

	return 0;
}

static int prictrl_aspeed_init(const struct device *dev)
{
	int ret = 0;
	const struct prictrl_aspeed_config *cfg = dev ? dev->config : NULL;

	if (!dev || !cfg)
		return -EINVAL;

	ret = prictrl_hw_init(dev);
	if (ret)
		return ret;

	/* Privilege control master group mapping */
	ret = prictrl_setup(cfg, cfg->master, cfg->master_num);
	if (ret)
		LOG_ERR("Master mapping fail(%d).", ret);

	/* Privilege control client group setting */
	ret = prictrl_setup(cfg, cfg->client, cfg->client_num);
	if (ret)
		LOG_ERR("Client group setting fail(%d).", ret);

	return 0;
}

static struct prictrl_dev_list prictrl_master_0[] = {
	PRICTRL_DTS_MASTER(SSP_GROUP, PRICTRL_DTS(0), mlist1),
	PRICTRL_DTS_MASTER(TSP_GROUP, PRICTRL_DTS(0), mlist2),
	PRICTRL_DTS_MASTER(S_CA35_GROUP, PRICTRL_DTS(0), mlist3),
	PRICTRL_DTS_MASTER(NS_CA35_GROUP, PRICTRL_DTS(0), mlist4),
	PRICTRL_DTS_MASTER(DP_MCU_GROUP, PRICTRL_DTS(0), mlist5),
};

static struct prictrl_dev_list prictrl_master_1[] = {
	PRICTRL_DTS_MASTER(BOOT_MCU_GROUP, PRICTRL_DTS(1), mlist0),
};

static struct prictrl_dev_list prictrl_client[] = {
	PRICTRL_DTS_CLIENT(BOOT_MCU_GROUP, PRICTRL_DTS_PH(0, clist0), protect_dev),
	PRICTRL_DTS_CLIENT(SSP_GROUP, PRICTRL_DTS_PH(0, clist1), protect_dev),
	PRICTRL_DTS_CLIENT(TSP_GROUP, PRICTRL_DTS_PH(0, clist2), protect_dev),
	PRICTRL_DTS_CLIENT(S_CA35_GROUP, PRICTRL_DTS_PH(0, clist3), protect_dev),
	PRICTRL_DTS_CLIENT(NS_CA35_GROUP, PRICTRL_DTS_PH(0, clist4), protect_dev),
	PRICTRL_DTS_CLIENT(DP_MCU_GROUP, PRICTRL_DTS_PH(0, clist5), protect_dev),
};

#define PRICTRL_ASPEED_INIT(_n)                                                                    \
	static struct prictrl_aspeed_config prictrl_aspeed_config_##_n = {                         \
		.reg = DT_INST_REG_ADDR(_n),                                                       \
		.master_num = ARRAY_SIZE(prictrl_master_##_n),                                     \
		.master_max = _n ? IO_M_LIST_END : C_M_LIST_END,                                   \
		.client_num = ARRAY_SIZE(prictrl_client),                                          \
		.client_max = S_LIST_END,                                                          \
		.master = prictrl_master_##_n,                                                     \
		.client = prictrl_client,                                                          \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(_n, prictrl_aspeed_init, NULL, NULL, &prictrl_aspeed_config_##_n,    \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(PRICTRL_ASPEED_INIT)
