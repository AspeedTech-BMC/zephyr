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
#include <zephyr/dt-bindings/memory-controller/ast27xx-mpu.h>
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

static int prictrl_peri_group_init(const struct prictrl_aspeed_config *cfg,
				   struct prictrl_dev_list *list, int num)
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
	bool init = false;
	const uint32_t init_val = 0x7F7F7F7F;
	const uint32_t default_val = 0x3F3F3F3F;
	const uint32_t magic = 0x7F7F7F7E;
	const struct prictrl_aspeed_config *cfg = dev ? dev->config : NULL;

	if (!dev || !cfg)
		return -EINVAL;

	sys_write32(magic, cfg->reg);
	init = sys_read32(cfg->reg) != magic ? true : false;

	/* Initialize privilege control configuration register */
	if (init) {
		for (i = 0; i < 8; i++)
			sys_write32(init_val, (cfg->reg + i * 4));

		for (i = 0; i < 8; i++)
			sys_write32(init_val, (cfg->reg + 0x100 + i * 4));

		for (i = 0; i < 64; i++)
			sys_write32(init_val, (cfg->reg + 0x200 + i * 4));

		for (i = 0; i < 64; i++)
			sys_write32(init_val, (cfg->reg + 0x300 + i * 4));
	} else {
		sys_write32(default_val, cfg->reg);
	}

	return init ? -EAGAIN : 0;
}

/**********************************************************************
 * Privilege control peripheral protection initialization
 **********************************************************************/
static int prictrl_peri_init(const struct device *dev)
{
	int ret = 0;
	const struct prictrl_aspeed_config *cfg = dev ? dev->config : NULL;

	if (!dev || !cfg)
		return -EINVAL;

	ret = prictrl_hw_init(dev);
	if (ret)
		return ret == -EAGAIN ? 0 : ret;

	/* Privilege control master group mapping */
	ret = prictrl_peri_group_init(cfg, cfg->master, cfg->master_num);
	if (ret)
		LOG_ERR("Master mapping fail(%d).", ret);

	/* Privilege control client group setting */
	ret = prictrl_peri_group_init(cfg, cfg->client, cfg->client_num);
	if (ret)
		LOG_ERR("Client group setting fail(%d).", ret);

	return 0;
}

/**********************************************************************
 * Privilege control memory protection region application interface
 **********************************************************************/
static void prictrl_mpu_cfg_dbg_dump(struct prictrl_mpu_cfg *mpu, int num)
{
	int i = 0;
	int j = 0;

	for (i = 0; i < num; i++) {
		LOG_DBG("MPU config: start: 0x%08lx, end: 0x%08lx", mpu[i].start, mpu[i].end);

		for (j = 0; j < mpu[i].l1_num; j++)
			LOG_DBG("  Level1 MPU device[%d]: 0x%08x", j, mpu[i].l1_dev[j]);

		for (j = 0; j < mpu[i].l2_num; j++)
			LOG_DBG("  Level2 MPU device[%d]: 0x%08x", j, mpu[i].l2_dev[j]);
	}
}

static int prictrl_mpu_build_region(struct prictrl_mpu_cfg *dst_mpu,
				    const struct prictrl_mpu_cfg *src_mpu, int mpu_num, int target)
{
	bool hit = false;
	int i = 0;
	int j = 0;
	int rnum = 0;

	if (!dst_mpu || !src_mpu || mpu_num < 0 || mpu_num >= MAX_MPU_COUNT)
		return -EINVAL;

	if (target != MPU_ID_SLIM && target != MPU_ID_H2M)
		return -EINVAL;

	for (i = 0; i < mpu_num; i++) {
		/* No level2 mpu is configured */
		if (src_mpu[i].l2_num == 0)
			continue;

		if (rnum >= PRICTRL_MPU_MAX_NUM)
			return -ENOSR;

		for (j = 0, hit = false; j < src_mpu[i].l1_num && !hit; j++)
			hit = src_mpu[i].l1_dev[j] == target ? true : false;

		if (!hit)
			continue;

		memcpy(&dst_mpu[rnum++], &src_mpu[i], sizeof(struct prictrl_mpu_cfg));
	}

	prictrl_mpu_cfg_dbg_dump(&dst_mpu[0], rnum);

	return rnum;
}

static int prictrl_mpu_build_sli_region(struct prictrl_mpu_cfg *sli_mpu,
					struct prictrl_mpu_cfg *mpu_cfg)
{
	if (!sli_mpu || !mpu_cfg)
		return -EINVAL;

	if ((mpu_cfg->start & PRICTRL_MPU_ADDR_ALIGN_MASK) ||
	    (mpu_cfg->end & PRICTRL_MPU_ADDR_ALIGN_MASK))
		return -EFAULT;

	if (mpu_cfg->end > 0xF0000000 || mpu_cfg->start > mpu_cfg->end)
		return -EFAULT;

	memset(sli_mpu, 0, sizeof(struct prictrl_mpu_cfg) * PRICTRL_MPU_MAX_NUM);

	/* Region 0, keep default setting. */

	/* Region 1, used to ensure other master can pass through */
	sli_mpu[1].start = 0x00000000;
	sli_mpu[1].end = mpu_cfg->start;

	/* Region 2, used to ensure other master can pass through */
	sli_mpu[2].start = mpu_cfg->end;
	sli_mpu[2].end = 0xF0000000;

	/* Region 3, used to configure sli mpu */
	memcpy(&sli_mpu[3], mpu_cfg, sizeof(struct prictrl_mpu_cfg));

	return 0;
}

/**********************************************************************
 * Privilege control memory protection application interface
 **********************************************************************/
static int prictrl_mpu_region_perm(struct prictrl_mpu_cfg *mpu, uint32_t cfg_mask, uint32_t *wperm,
				   uint32_t *rperm)
{
	struct prictrl_dev_dts *dev_dts = NULL;

	if (!mpu || !wperm || !rperm)
		return -EINVAL;

	if (cfg_mask != PRICTRL_MPU_SLI_MASK && cfg_mask != PRICTRL_MPU_H2M0_MASK &&
	    cfg_mask != PRICTRL_MPU_H2M1_MASK)
		return -EINVAL;

	/* Before permission configuration, initialize permissions it */
	if (mpu->l2_num != 0 && cfg_mask == PRICTRL_MPU_SLI_MASK)
		*wperm = *rperm = (1 << (MPU_ID_H2M1 & 0x00000FFF));
	else
		*wperm = *rperm = 0;

	for (dev_dts = (struct prictrl_dev_dts *)mpu->l2_dev;
	     dev_dts < (struct prictrl_dev_dts *)(mpu->l2_dev + mpu->l2_num); dev_dts++) {
		if ((dev_dts->dev & cfg_mask) != cfg_mask)
			continue;

		if ((dev_dts->dev & 0x00000FFF) > 31)
			continue;

		if (dev_dts->perm == S_READWRITE || dev_dts->perm == NS_READWRITE ||
		    dev_dts->perm == S_WRITEONLY || dev_dts->perm == NS_WRITEONLY)
			*wperm |= (1 << (dev_dts->dev & 0x00000FFF));

		if (dev_dts->perm == S_READWRITE || dev_dts->perm == NS_READWRITE ||
		    dev_dts->perm == S_READONLY || dev_dts->perm == NS_READONLY)
			*rperm |= (1 << (dev_dts->dev & 0x00000FFF));
	}

	return 0;
}

static int prictrl_mpu_region_en(const struct prictrl_aspeed_config *cfg,
				 struct prictrl_mpu_cfg *mpu_cfg, int cfg_mask, int idx)
{
	uintptr_t reg = 0;
	uint32_t offset = 0;
	uint32_t s_addr = 0;
	uint32_t e_addr = 0;
	uint32_t w_perm = 0;
	uint32_t r_perm = 0;

	if (!cfg || !mpu_cfg || idx >= PRICTRL_MPU_MAX_NUM)
		return -EINVAL;

	if (mpu_cfg->start > mpu_cfg->end)
		return -EFAULT;

	if ((mpu_cfg->start & PRICTRL_MPU_ADDR_ALIGN_MASK) ||
	    (mpu_cfg->end & PRICTRL_MPU_ADDR_ALIGN_MASK))
		return -EFAULT;

	if (cfg_mask != PRICTRL_MPU_SLI_MASK && cfg_mask != PRICTRL_MPU_H2M0_MASK &&
	    cfg_mask != PRICTRL_MPU_H2M1_MASK)
		return -EINVAL;

	offset = (cfg_mask == PRICTRL_MPU_SLI_MASK) ? PRICTRL_MPU_SLI_OFFSET
						    : PRICTRL_MPU_H2M_OFFSET;
	reg = cfg->reg + offset + (idx * PRICTRL_MPU_BANK);

	s_addr = PRICTRL_MPU_DEFAULT_GRP | PRICTRL_MPU_ADDR(mpu_cfg->start);
	e_addr = PRICTRL_MPU_DEFAULT_GRP | PRICTRL_MPU_ADDR(mpu_cfg->end);
	prictrl_mpu_region_perm(mpu_cfg, cfg_mask, &w_perm, &r_perm);

	/* Configure start and end address */
	sys_write32(s_addr, reg + PRICTRL_MPU_S_GRP_OFFSET);
	sys_write32(e_addr, reg + PRICTRL_MPU_E_GRP_OFFSET);

	/* Configure r/w permission, if l2_num == 0, keep default setting */
	if (mpu_cfg->l2_num != 0)
		sys_write32(w_perm, reg + PRICTRL_MPU_WPERM_OFFSET);
	if (mpu_cfg->l2_num != 0)
		sys_write32(r_perm, reg + PRICTRL_MPU_RPERM_OFFSET);

	/* Enable region and lock setting */
	sys_write32(s_addr | PRICTRL_MPU_ENABLE, reg + PRICTRL_MPU_S_GRP_OFFSET);
	sys_write32(e_addr | PRICTRL_MPU_LOCK, reg + PRICTRL_MPU_E_GRP_OFFSET);

	LOG_DBG("Enable %d region: (s: 0x%08x, e: 0x%08x, w: 0x%08x, r: 0x%08x)", idx, s_addr,
		e_addr, w_perm, r_perm);

	return 0;
}

static int prictrl0_mpu_config_region(const struct prictrl_aspeed_config *cfg,
				      struct prictrl_mpu_cfg *mpu_cfg)
{
	int i = 0;
	int ret = 0;

	if (!cfg || !mpu_cfg)
		return -EINVAL;

	/* Configure h2m mpu regions */
	for (i = 0; i < PRICTRL_MPU_MAX_NUM; i++) {
		ret = prictrl_mpu_region_en(cfg, &mpu_cfg[i], PRICTRL_MPU_H2M0_MASK, i);
		if (ret)
			return ret;
	}

	return 0;
}

static int prictrl1_mpu_config_region(const struct prictrl_aspeed_config *cfg,
				      struct prictrl_mpu_cfg *mpu_cfg)
{
	int i = 0;
	int ret = 0;
	struct prictrl_mpu_cfg sli_mpu_cfg[PRICTRL_MPU_MAX_NUM] = {0};

	if (!cfg || !mpu_cfg)
		return -EINVAL;

	/* SLI uses a default-block policy. Pass-through paths must be explicitly configured */
	ret = prictrl_mpu_build_sli_region(sli_mpu_cfg, &mpu_cfg[0]);
	if (ret)
		return ret;

	/* Configure the first sli mpu region */
	for (i = 0; i < PRICTRL_MPU_MAX_NUM; i++) {
		ret = prictrl_mpu_region_en(cfg, &sli_mpu_cfg[i], PRICTRL_MPU_SLI_MASK, i);
		if (ret)
			return ret;
	}

	/* Configure all h2m mpu regions */
	for (i = 0; i < PRICTRL_MPU_MAX_NUM; i++) {
		ret = prictrl_mpu_region_en(cfg, &mpu_cfg[i], PRICTRL_MPU_H2M1_MASK, i);
		if (ret)
			return ret;
	}

	return 0;
}

/**********************************************************************
 * Privilege control memory protection initialization
 **********************************************************************/
static int prictrl0_mpu_init(const struct device *dev)
{
	int rc = 0;
	const struct prictrl_aspeed_config *cfg = dev ? dev->config : NULL;
	const struct prictrl_mpu_cfg src_mpu0[] = {
#if DT_NODE_HAS_PROP(PRICTRL_PROT_DTS(sdrammc), mpus)
#define _PRICTRL_LEVEL h2m_port
		DT_FOREACH_PROP_ELEM(PRICTRL_PROT_DTS(sdrammc), mpus, PRICTRL_DTS_MPU)
#undef _PRICTRL_LEVEL
#endif
	};
	struct prictrl_mpu_cfg mpu0[PRICTRL_MPU_MAX_NUM] = {0};

	if (!cfg)
		return -EINVAL;

	/* rc < 0, build region error; rc == 0, no region to configure */
	rc = prictrl_mpu_build_region(mpu0, src_mpu0, ARRAY_SIZE(src_mpu0), MPU_ID_H2M);
	if (rc <= 0)
		return rc;

	return prictrl0_mpu_config_region(cfg, mpu0);
}
static int prictrl1_mpu_init(const struct device *dev)
{
	int rc = 0;
	const struct prictrl_aspeed_config *cfg = dev ? dev->config : NULL;
	const struct prictrl_mpu_cfg src_mpu1[] = {
#if DT_NODE_HAS_PROP(PRICTRL_PROT_DTS(sdrammc), mpus)
#define _PRICTRL_LEVEL sli_port
		DT_FOREACH_PROP_ELEM(PRICTRL_PROT_DTS(sdrammc), mpus, PRICTRL_DTS_MPU)
#undef _PRICTRL_LEVEL
#endif
	};
	struct prictrl_mpu_cfg mpu1[PRICTRL_MPU_MAX_NUM] = {0};

	if (!cfg)
		return -EINVAL;

	/* rc < 0, build region error; rc == 0, no region to configure */
	rc = prictrl_mpu_build_region(mpu1, src_mpu1, ARRAY_SIZE(src_mpu1), MPU_ID_SLIM);
	if (rc <= 0)
		return rc;

	/*
	 * prictrl1-h2m supports 4 regions, prictrl-sli supports 1 region.
	 * If dts configure region number between 2 ~ 4, prictrl1-h2m will be used
	 * for all regions; prictrl-sli only configure the first region.
	 */
	if (rc > PRICTRL_MPU_SLI_NUM)
#ifdef PRICTRL_MPU_MULTI_REGION_PARTIAL_PROT
		LOG_WRN("SLI MPU region number %d exceeds the supported number.", rc);
#else
		return rc;
#endif

	return prictrl1_mpu_config_region(cfg, mpu1);
}

/**********************************************************************
 * Privilege control system hook
 **********************************************************************/
static int prictrl0_aspeed_init(const struct device *dev)
{
	int ret = 0;
	const struct prictrl_aspeed_config *cfg = dev ? dev->config : NULL;

	if (!dev || !cfg)
		return -EINVAL;

	ret = prictrl_peri_init(dev);
	if (ret) {
		LOG_ERR("Privilege control 0 peri initialization fail. %d", ret);
		return ret;
	}

	ret = prictrl0_mpu_init(dev);
	if (ret) {
		LOG_ERR("Privilege control 0 MPU initialization fail. %d", ret);
		return ret;
	}

	return 0;
}

static int prictrl1_aspeed_init(const struct device *dev)
{
	int ret = 0;
	const struct prictrl_aspeed_config *cfg = dev ? dev->config : NULL;

	if (!dev || !cfg)
		return -EINVAL;

	ret = prictrl_peri_init(dev);
	if (ret) {
		LOG_ERR("Privilege control 1 peri initialization fail. %d", ret);
		return ret;
	}

	ret = prictrl1_mpu_init(dev);
	if (ret) {
		LOG_ERR("Privilege control 1 MPU initialization fail. %d", ret);
		return ret;
	}

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
	PRICTRL_DTS_CLIENT(BOOT_MCU_GROUP, PRICTRL_PROT_DTS(prictrl_bootmcu), protect_dev),
	PRICTRL_DTS_CLIENT(SSP_GROUP, PRICTRL_PROT_DTS(prictrl_ssp), protect_dev),
	PRICTRL_DTS_CLIENT(TSP_GROUP, PRICTRL_PROT_DTS(prictrl_tsp), protect_dev),
	PRICTRL_DTS_CLIENT(S_CA35_GROUP, PRICTRL_PROT_DTS(prictrl_s_ca35), protect_dev),
	PRICTRL_DTS_CLIENT(NS_CA35_GROUP, PRICTRL_PROT_DTS(prictrl_ns_ca35), protect_dev),
	PRICTRL_DTS_CLIENT(DP_MCU_GROUP, PRICTRL_PROT_DTS(prictrl_dp), protect_dev),
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
	DEVICE_DT_INST_DEFINE(_n, prictrl##_n##_aspeed_init, NULL, NULL,                           \
			      &prictrl_aspeed_config_##_n, POST_KERNEL,                            \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(PRICTRL_ASPEED_INIT)
