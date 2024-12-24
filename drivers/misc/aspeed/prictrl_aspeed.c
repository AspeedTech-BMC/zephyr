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

LOG_MODULE_REGISTER(prictrl_aspeed);

/**********************************************************************
 * Get privilege control configuration register
 **********************************************************************/
static uintptr_t prictrl_get_master_reg_base(const struct prictrl_aspeed_config *cfg,
					     enum prictrl_cpu_io cpu_io, enum prictrl_rw rw)
{
	if (!cfg)
		return -EINVAL;

	if (cpu_io != PRICTRL_CPU_DIE && cpu_io != PRICTRL_IO_DIE)
		return -EINVAL;

	if (rw != PRICTRL_WRITE && rw != PRICTRL_READ)
		return -EINVAL;

	/* Return CPU die base address + read/write offset */
	return ((cpu_io == PRICTRL_CPU_DIE ? cfg->cpu_base : cfg->io_base) +
		(rw == PRICTRL_WRITE ? 0 : PRICTRL_READ_OFFSET));
}

static uintptr_t prictrl_get_client_reg_base(const struct prictrl_aspeed_config *cfg,
					     enum prictrl_cpu_io cpu_io, enum prictrl_rw rw)
{
	if (!cfg)
		return -EINVAL;

	if (cpu_io != PRICTRL_CPU_DIE && cpu_io != PRICTRL_IO_DIE)
		return -EINVAL;

	if (rw != PRICTRL_WRITE && rw != PRICTRL_READ)
		return -EINVAL;

	/* Return CPU die base address + read/write offset + client offset */
	return ((cpu_io == PRICTRL_CPU_DIE ? cfg->cpu_base : cfg->io_base) +
		(rw == PRICTRL_WRITE ? 0 : PRICTRL_READ_OFFSET) + PRICTRL_CLIENT_OFFSET);
}

static uintptr_t prictrl_get_reg_base(const struct prictrl_aspeed_config *cfg,
				      enum prictrl_cpu_io cpu_io, enum prictrl_ms ms,
				      enum prictrl_rw rw)
{
	if (!cfg)
		return -EINVAL;

	if (cpu_io != PRICTRL_CPU_DIE && cpu_io != PRICTRL_IO_DIE)
		return -EINVAL;

	if (ms != PRICTRL_MASTER && ms != PRICTRL_CLIENT)
		return -EINVAL;

	return (ms == PRICTRL_MASTER ? prictrl_get_master_reg_base(cfg, cpu_io, rw)
				     : prictrl_get_client_reg_base(cfg, cpu_io, rw));
}

static uintptr_t prictrl_get_master_reg_offset(enum prictrl_cpu_io cpu_io, int device)
{
	if (cpu_io == PRICTRL_CPU_DIE && (device < C_M_CPU_S_USER || device >= C_M_LIST_END))
		return -EINVAL;

	if (cpu_io == PRICTRL_IO_DIE && (device < IO_M_MCU0_I || device >= IO_M_LIST_END))
		return -EINVAL;

	return ROUND_DOWN(device, PRICTRL_FILED_NUM_PER_REG);
}

static uintptr_t prictrl_get_client_reg_offset(int device)
{
	if (device < IO_S_I_USB_UHCI || device >= S_LIST_END)
		return -EINVAL;

	return ROUND_DOWN(device, PRICTRL_FILED_NUM_PER_REG);
}

static uintptr_t prictrl_get_reg_offset(enum prictrl_cpu_io cpu_io, enum prictrl_ms ms, int device)
{
	if (cpu_io != PRICTRL_CPU_DIE && cpu_io != PRICTRL_IO_DIE)
		return -EINVAL;

	if (ms != PRICTRL_MASTER && ms != PRICTRL_CLIENT)
		return -EINVAL;

	return (ms == PRICTRL_MASTER ? prictrl_get_master_reg_offset(cpu_io, device)
				     : prictrl_get_client_reg_offset(device));
}

static uint8_t prictrl_get_reg_field(uint16_t device)
{
	return (device % PRICTRL_FILED_NUM_PER_REG);
}

static uintptr_t prictrl_get_reg_addr(const struct prictrl_aspeed_config *cfg,
				      enum prictrl_cpu_io cpu_io, enum prictrl_rw rw,
				      struct prictrl_dev_cfg *dev_cfg)
{
	uintptr_t prictrl_base = 0;
	uintptr_t prictrl_offset = 0;

	prictrl_base = prictrl_get_reg_base(cfg, cpu_io, dev_cfg->ms, rw);
	if (prictrl_base == -EINVAL)
		return -EINVAL;

	prictrl_offset = prictrl_get_reg_offset(cpu_io, dev_cfg->ms, dev_cfg->device);
	if (prictrl_offset == -EINVAL)
		return -EINVAL;

	return prictrl_base + prictrl_offset;
}

/**********************************************************************
 * Operate privilege control configuration register
 **********************************************************************/
static int prictrl_get_perm(const struct prictrl_aspeed_config *cfg, enum prictrl_cpu_io cpu_io,
			    enum prictrl_rw rw, struct prictrl_dev_cfg *dev_cfg)
{
	uint8_t field = 0;
	uintptr_t addr = 0;

	/* Get the config register address */
	addr = prictrl_get_reg_addr(cfg, cpu_io, rw, dev_cfg);
	if (addr == -EINVAL)
		return -EINVAL;

	/* Get the config register field */
	field = prictrl_get_reg_field(dev_cfg->device);

	/* Get group setting from privilege control register */
	dev_cfg->last_group =
		FIELD_GET(PRICTRL_SHIFT_FIELD(PRICTRL_GROUP_MASK, field), sys_read32(addr));

	return 0;
}

static int prictrl_set_perm(const struct prictrl_aspeed_config *cfg, enum prictrl_cpu_io cpu_io,
			    enum prictrl_rw rw, struct prictrl_dev_cfg *dev_cfg)
{
	uint8_t field = 0;
	uint8_t val = 0;
	uintptr_t addr = 0;

	/* Get the config register address */
	addr = prictrl_get_reg_addr(cfg, cpu_io, rw, dev_cfg);
	if (addr == -EINVAL)
		return -EINVAL;

	/* Get the config register field */
	field = prictrl_get_reg_field(dev_cfg->device);

	/* Get the configuration from prictrl_dev_cfg */
	val = PRICTRL_GROUP_MASK & dev_cfg->group;

	/* If it is the first time programming, clear the default config */
	prictrl_get_perm(cfg, cpu_io, rw, dev_cfg);
	if (dev_cfg->last_group == PRICTRL_GROUP_DEFAULT)
		sys_clear_bits(addr, PRICTRL_SHIFT_FIELD(PRICTRL_GROUP_MASK & ~val, field));

	/* Set group configuration */
	sys_set_bits(addr, PRICTRL_SHIFT_FIELD(val, field));

	/* Set group lock */
	sys_set_bits(addr, PRICTRL_SHIFT_FIELD(dev_cfg->lock, field));

	LOG_DBG("(addr:field:value:lock) = (0x%08lx:0x%02x:0x%02x:0x%02x), readback: 0x%08x", addr,
		field, val, dev_cfg->lock, sys_read32(addr));

	return 0;
}

/**********************************************************************
 * Privilege control application programming interface
 **********************************************************************/
static int prictrl_set_master_group(const struct prictrl_aspeed_config *cfg,
				    struct prictrl_dev_cfg *dev_cfg)
{
	int ret = 0;

	if (!cfg || !dev_cfg)
		return -EINVAL;

	if (dev_cfg->cpu_io == PRICTRL_CPU_DIE &&
	    (dev_cfg->device < C_M_CPU_S_USER || dev_cfg->device >= C_M_LIST_END))
		return -EINVAL;

	if (dev_cfg->cpu_io == PRICTRL_IO_DIE &&
	    (dev_cfg->device < IO_M_MCU0_I || dev_cfg->device >= IO_M_LIST_END))
		return -EINVAL;

	ret = prictrl_set_perm(cfg, dev_cfg->cpu_io, PRICTRL_WRITE, dev_cfg);
	if (ret)
		return -EINVAL;

	ret = prictrl_set_perm(cfg, dev_cfg->cpu_io, PRICTRL_READ, dev_cfg);
	if (ret)
		return -EINVAL;

	return ret;
}

static int prictrl_set_client_group(const struct prictrl_aspeed_config *cfg,
				    struct prictrl_dev_cfg *dev_cfg)
{
	int ret = 0;

	if (!cfg || !dev_cfg)
		return -EINVAL;

	if (dev_cfg->cpu_io != PRICTRL_CPU_IO_DIE || dev_cfg->device < IO_S_I_USB_UHCI ||
	    dev_cfg->device >= S_LIST_END)
		return -EINVAL;

	ret = prictrl_set_perm(cfg, PRICTRL_CPU_DIE, PRICTRL_WRITE, dev_cfg);
	if (ret)
		return -EINVAL;

	ret = prictrl_set_perm(cfg, PRICTRL_CPU_DIE, PRICTRL_READ, dev_cfg);
	if (ret)
		return -EINVAL;

	ret = prictrl_set_perm(cfg, PRICTRL_IO_DIE, PRICTRL_WRITE, dev_cfg);
	if (ret)
		return -EINVAL;

	ret = prictrl_set_perm(cfg, PRICTRL_IO_DIE, PRICTRL_READ, dev_cfg);
	if (ret)
		return -EINVAL;

	return ret;
}

static int prictrl_get_group(const struct prictrl_aspeed_config *cfg,
			     struct prictrl_dev_cfg *dev_cfg)
{
	int ret = 0;

	if (!cfg || !dev_cfg)
		return -EINVAL;

	/* Before we assign the last_group, check it is in initial value */
	if (dev_cfg->last_group != INVALID_GROUP)
		return -EINVAL;

	if (dev_cfg->ms != PRICTRL_MASTER && dev_cfg->ms != PRICTRL_CLIENT)
		return -EINVAL;

	/* Master r/w permission are in the same group, we only need to get one of them.  */
	if (dev_cfg->ms == PRICTRL_MASTER)
		ret = prictrl_get_perm(cfg, dev_cfg->cpu_io, PRICTRL_WRITE, dev_cfg);

	/* Client r/w/cpu/io permission are in the same group, we only need to get one of them. */
	if (dev_cfg->ms == PRICTRL_CLIENT)
		ret = prictrl_get_perm(cfg, PRICTRL_CPU_DIE, PRICTRL_WRITE, dev_cfg);

	return ret;
}

static int prictrl_set_group(const struct prictrl_aspeed_config *cfg,
			     struct prictrl_dev_cfg *dev_cfg)
{
	if (!cfg || !dev_cfg)
		return -EINVAL;

	if (dev_cfg->ms != PRICTRL_MASTER && dev_cfg->ms != PRICTRL_CLIENT)
		return -EINVAL;

	return (dev_cfg->ms == PRICTRL_MASTER ? prictrl_set_master_group(cfg, dev_cfg)
					      : prictrl_set_client_group(cfg, dev_cfg));
}

static int prictrl_lock_group(const struct prictrl_aspeed_config *cfg,
			      struct prictrl_dev_cfg *dev_cfg)
{
	if (!cfg || !dev_cfg)
		return -EINVAL;

	if (dev_cfg->ms != PRICTRL_MASTER && dev_cfg->ms != PRICTRL_CLIENT)
		return -EINVAL;

	if (prictrl_get_group(cfg, dev_cfg))
		return -EINVAL;

	PRICTRL_SET_DEV(dev_cfg, dev_cfg->cpu_io, dev_cfg->ms, dev_cfg->device, dev_cfg->last_group,
			PRICTRL_LOCK);
	if (prictrl_set_group(cfg, dev_cfg))
		return -EINVAL;

	return 0;
}

static int prictrl_list_set_group(const struct prictrl_aspeed_config *cfg,
				  struct prictrl_list_cfg *list)
{
	int ret = 0;
	int i = 0;
	struct prictrl_dev_cfg dev_cfg = {0};

	if (!list || !cfg)
		return -EINVAL;

	for (i = 0; i < list->device_num; i++) {
		PRICTRL_SET_DEV(&dev_cfg, list->cpu_io, list->ms, list->device[i], list->group,
				PRICTRL_NO_LOCK);
		if (prictrl_set_group(cfg, &dev_cfg)) {
			LOG_ERR("Set %d's %d in %d fail.\n", list->ms, list->device[i],
				list->group);
			ret = -EINVAL;
		}
	}

	return ret;
}

static int prictrl_list_lock_group(const struct prictrl_aspeed_config *cfg,
				   struct prictrl_list_cfg *list)
{
	int ret = 0;
	int i = 0;
	struct prictrl_dev_cfg dev_cfg = {0};

	if (!list || !cfg)
		return -EINVAL;

	for (i = 0; i < list->device_num; i++) {
		PRICTRL_SET_DEV(&dev_cfg, list->cpu_io, list->ms, list->device[i], list->group,
				PRICTRL_LOCK);
		if (prictrl_lock_group(cfg, &dev_cfg)) {
			LOG_ERR("Lock %d's %d in %d fail.\n", list->ms, list->device[i],
				list->group);
			ret = -EINVAL;
		}
	}

	return ret;
}

static int prictrl_master_mapping(const struct device *dev)
{
	int ret = 0;
	const struct prictrl_aspeed_config *cfg = NULL;

	if (!dev || !(dev->config))
		return -EINVAL;

	cfg = dev->config;

	/* CPU die master mapping */
	ret |= prictrl_list_set_group(cfg, &cfg->master[1]);
	ret |= prictrl_list_set_group(cfg, &cfg->master[2]);
	ret |= prictrl_list_set_group(cfg, &cfg->master[3]);
	ret |= prictrl_list_set_group(cfg, &cfg->master[4]);
	ret |= prictrl_list_set_group(cfg, &cfg->master[5]);

	/* IO die master mapping */
	ret |= prictrl_list_set_group(cfg, &cfg->master[0]);

	/* No permission master mappin */
	ret |= prictrl_list_set_group(cfg, &cfg->master[6]);

	/* Lock all setting */
	ret |= prictrl_list_lock_group(cfg, &cfg->master[0]);
	ret |= prictrl_list_lock_group(cfg, &cfg->master[1]);
	ret |= prictrl_list_lock_group(cfg, &cfg->master[2]);
	ret |= prictrl_list_lock_group(cfg, &cfg->master[3]);
	ret |= prictrl_list_lock_group(cfg, &cfg->master[4]);
	ret |= prictrl_list_lock_group(cfg, &cfg->master[5]);
	ret |= prictrl_list_lock_group(cfg, &cfg->master[6]);

	return ret;
}

static int prictrl_client_setting(const struct device *dev)
{
	int ret = 0;
	const struct prictrl_aspeed_config *cfg = NULL;

	if (!dev || !(dev->config))
		return -EINVAL;

	cfg = dev->config;

	ret |= prictrl_list_set_group(cfg, &cfg->client[0]);
	ret |= prictrl_list_set_group(cfg, &cfg->client[1]);
	ret |= prictrl_list_set_group(cfg, &cfg->client[2]);
	ret |= prictrl_list_set_group(cfg, &cfg->client[3]);
	ret |= prictrl_list_set_group(cfg, &cfg->client[4]);
	ret |= prictrl_list_set_group(cfg, &cfg->client[5]);

	/* Lock all setting */
	ret |= prictrl_list_lock_group(cfg, &cfg->client[0]);
	ret |= prictrl_list_lock_group(cfg, &cfg->client[1]);
	ret |= prictrl_list_lock_group(cfg, &cfg->client[2]);
	ret |= prictrl_list_lock_group(cfg, &cfg->client[3]);
	ret |= prictrl_list_lock_group(cfg, &cfg->client[4]);
	ret |= prictrl_list_lock_group(cfg, &cfg->client[5]);

	return ret;
}

static int prictrl_hw_init(void)
{
	/* The polling instruction of CPU core 0 ~ 3 */
	sys_write32(0x14000000, PRICTRL_CPU_SMP_SCRATCH_0);

	/* core 0 ~ 3 jump to polling instruction */
	sys_write32(PRICTRL_CPU_SMP_SCRATCH_0, PRICTRL_CPU_SMP_EP0);
	sys_write32(PRICTRL_CPU_SMP_SCRATCH_0, PRICTRL_CPU_SMP_EP1);
	sys_write32(PRICTRL_CPU_SMP_SCRATCH_0, PRICTRL_CPU_SMP_EP2);
	sys_write32(PRICTRL_CPU_SMP_SCRATCH_0, PRICTRL_CPU_SMP_EP3);

	/* Init core 0 ~ 3 and privilege control */
	sys_write32(0x1, PRICTRL_CPU_CA35_REL);

	return 0;
}

static int prictrl_hw_deinit(void)
{
	uint32_t polling_count = 10000;
	uint32_t wdt_mask[5] = {0};

	/* Backup wdt mask setting */
	wdt_mask[0] = sys_read32(PRICTRL_WDT_RESET_MASK_1);
	wdt_mask[1] = sys_read32(PRICTRL_WDT_RESET_MASK_2);
	wdt_mask[2] = sys_read32(PRICTRL_WDT_RESET_MASK_3);
	wdt_mask[3] = sys_read32(PRICTRL_WDT_RESET_MASK_4);
	wdt_mask[4] = sys_read32(PRICTRL_WDT_RESET_MASK_5);

	/* Config only reset ca35 core 0 ~ 3 */
	sys_write32(0x1, PRICTRL_WDT_RESET_MASK_1);
	sys_write32(0x0, PRICTRL_WDT_RESET_MASK_2);
	sys_write32(0x0, PRICTRL_WDT_RESET_MASK_3);
	sys_write32(0x0, PRICTRL_WDT_RESET_MASK_4);
	sys_write32(0x0, PRICTRL_WDT_RESET_MASK_5);

	/* Re-Init ca35 core 0 ~ 3 */
	sys_write32(0x500, PRICTRL_WDT_RELOAD_VALUE);
	sys_write32(0x4755, PRICTRL_WDT_RESTART);
	sys_write32(0x13, PRICTRL_WDT_CONTROL);

	/* Polling timeout status and clear wdt timeout status */
	while (polling_count-- && sys_test_bit(PRICTRL_WDT_TIMEOUT_STAT, 0) == 0)
		;
	sys_set_bit(PRICTRL_WDT_CLR_TIMEOUT_STAT, 0);

	/* Restore wdt mask setting */
	sys_write32(wdt_mask[0], PRICTRL_WDT_RESET_MASK_1);
	sys_write32(wdt_mask[1], PRICTRL_WDT_RESET_MASK_2);
	sys_write32(wdt_mask[2], PRICTRL_WDT_RESET_MASK_3);
	sys_write32(wdt_mask[3], PRICTRL_WDT_RESET_MASK_4);
	sys_write32(wdt_mask[4], PRICTRL_WDT_RESET_MASK_5);

	return 0;
}

static int prictrl_aspeed_init(const struct device *dev)
{
	int ret = 0;

	if (!dev)
		return -EINVAL;

	prictrl_hw_init();

	ret = prictrl_master_mapping(dev);
	if (ret)
		LOG_ERR("Master mapping fail(%d).", ret);

	ret = prictrl_client_setting(dev);
	if (ret)
		LOG_ERR("Client group setting fail(%d).", ret);

	prictrl_hw_deinit();

	LOG_INF("privilege control init done.");

	return 0;
}

static struct prictrl_list_cfg master_list[] = {
	[0] = DEFINE_MASTER_DEV(PRICTRL_IO_DIE, BOOT_MCU_GROUP, IO_M_MCU0_I, IO_M_MCU0_D),
	[1] = DEFINE_MASTER_DEV(PRICTRL_CPU_DIE, SSP_GROUP, C_M_SSP_I_USER, C_M_SSP_I_PRI,
				C_M_SSP_D_USER, C_M_SSP_D_PRI, C_M_SSP_S_USER, C_M_SSP_S_PRI),
	[2] = DEFINE_MASTER_DEV(PRICTRL_CPU_DIE, TSP_GROUP, C_M_TSP_S_USER, C_M_TSP_S_PRI),
	[3] = DEFINE_MASTER_DEV(PRICTRL_CPU_DIE, S_CA35_GROUP, C_M_CPU_S_PRI),
	[4] = DEFINE_MASTER_DEV(PRICTRL_CPU_DIE, NS_CA35_GROUP, C_M_CPU_NS_PRI),
	[5] = DEFINE_MASTER_DEV(PRICTRL_CPU_DIE, DP_MCU_GROUP, C_M_DP_MCU),
	[6] = DEFINE_MASTER_DEV(PRICTRL_CPU_DIE, NO_PERM_GROUP, C_M_CPU_S_USER, C_M_CPU_NS_USER),
};

static struct prictrl_list_cfg client_list[] = {
	[0] = DEFINE_CLIENT_DEV(PRICTRL_CPU_IO_DIE, BOOT_MCU_GROUP, client_list_1),
	[1] = DEFINE_CLIENT_DEV(PRICTRL_CPU_IO_DIE, SSP_GROUP, client_list_2),
	[2] = DEFINE_CLIENT_DEV(PRICTRL_CPU_IO_DIE, TSP_GROUP, client_list_3),
	[3] = DEFINE_CLIENT_DEV(PRICTRL_CPU_IO_DIE, S_CA35_GROUP, client_list_4),
	[4] = DEFINE_CLIENT_DEV(PRICTRL_CPU_IO_DIE, NS_CA35_GROUP, client_list_5),
	[5] = DEFINE_CLIENT_DEV(PRICTRL_CPU_IO_DIE, DP_MCU_GROUP, client_list_6),
};

static struct prictrl_aspeed_config prictrl_aspeed_config = {
	.cpu_base = DT_INST_REG_ADDR_BY_NAME(0, cpu),
	.io_base = DT_INST_REG_ADDR_BY_NAME(0, io),
	.master = master_list,
	.client = client_list,
};

DEVICE_DT_INST_DEFINE(0, prictrl_aspeed_init, NULL, NULL, &prictrl_aspeed_config, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);
