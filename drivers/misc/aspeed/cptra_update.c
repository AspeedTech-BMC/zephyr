/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_update

#include <soc.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/cptra.h>
#include <zephyr/drivers/misc/aspeed/cptra_mbox.h>

LOG_MODULE_REGISTER(cptra_update, CONFIG_LOG_DEFAULT_LEVEL);

/* Device config */
struct cptra_update_config {
	uintptr_t base;			/* Caliptra mbox base address */
	uintptr_t ifc_base;		/* Caliptra soc ifc base address */
	uintptr_t scu_base;		/* SCU1 base address */
};

struct cptra_update_drv_state {
	bool in_use;
};

#define DEV_CFG(dev)				\
	((struct cptra_update_config *)		\
	(dev)->config)

#define DEV_DATA(dev)				\
	((struct cptra_update_drv_state *)	\
	(dev)->data)

static int aspeed_cptra_fw_upload_init(const struct device *dev)
{
	struct cptra_update_drv_state *state = DEV_DATA(dev);
	struct cptra_update_config *cfg = DEV_CFG(dev);
	uint32_t cmd, csum;
	uint32_t sts;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	state->in_use = true;

	/* Get mbox lock */
	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	/* init mbox parameters */
	cmd = CPTRA_MBCMD_CALIPTRA_FW_LOAD;
	csum = 0;

	sys_write32(CPTRA_MBCMD_CALIPTRA_FW_LOAD, cfg->base + CPTRA_MBOX_CMD);
	sys_write32(CPTRA_MBOX_SZ, cfg->base + CPTRA_MBOX_DLEN);

	return 0;
}

static int aspeed_cptra_fw_upload_update(const struct device *dev, uint8_t *buf, int size)
{
	struct cptra_update_drv_state *state = DEV_DATA(dev);
	struct cptra_update_config *cfg = DEV_CFG(dev);
	uint32_t data;
	int i;

	if (!state->in_use) {
		LOG_ERR("Peripheral in not use, wrong state.");
		return -EBUSY;
	}

	for (i = 0; i < size; i += sizeof(data)) {
		if ((size - i) < sizeof(data))
			break;

		data = sys_read32((mem_addr_t)buf + i);
		sys_write32(data, cfg->base + CPTRA_MBOX_DATAIN);
	}

	return 0;
}

static int aspeed_cptra_fw_upload_final(const struct device *dev, int size)
{
	struct cptra_update_drv_state *state = DEV_DATA(dev);
	struct cptra_update_config *cfg = DEV_CFG(dev);
	uint32_t sts;
	int count = 0;

	if (!state->in_use) {
		LOG_ERR("Peripheral in not use, wrong state.");
		return -EBUSY;
	}

	sys_write32(0x1, cfg->base + CPTRA_MBOX_EXEC);

	/* check update reset occurs */
	while (count++ < CPTRA_UPD_RST_TIMEOUT) {
		sts = sys_read32(cfg->ifc_base + CPTRA_RST_REASON);
		if (sts & CPTRA_FW_UPD_RESET) {
			LOG_INF("FW Update Reset !!!");
			break;
		}
	}

	/* poll for result */
	while (1) {
		sts = FIELD_GET(CPTRA_MBOX_STS_PS, cptra_mbox_status());
		if (sts != CPTRA_MBSTS_CMD_BUSY)
			break;
	}

	while (cptra_mbox_unlock())
		;

	state->in_use = false;

	return (sts == CPTRA_MBSTS_CMD_COMPLETE) ? 0 : -EIO;
}

static int aspeed_cptra_fw_upload(const struct device *dev, uint8_t *buf, int size)
{
	struct cptra_update_drv_state *state = DEV_DATA(dev);
	struct cptra_update_config *cfg = DEV_CFG(dev);
	uint32_t cmd, csum;
	uint32_t sts, data;
	int count = 0;
	int i;

	LOG_INF("Start upload from 0x%x, size:0x%x", (uint32_t)buf, size);

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	/* sanity check */
	if (!buf || size > CPTRA_MBOX_SZ) {
		LOG_ERR("Sanity check failed");
		return -EINVAL;
	}

	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	/* init mbox parameters */
	cmd = CPTRA_MBCMD_CALIPTRA_FW_LOAD;
	csum = 0;

	sys_write32(CPTRA_MBCMD_CALIPTRA_FW_LOAD, cfg->base + CPTRA_MBOX_CMD);
	sys_write32(size, cfg->base + CPTRA_MBOX_DLEN);

	for (i = 0; i < size; i += sizeof(data)) {
		if ((size - i) < sizeof(data))
			break;

		data = sys_read32((mem_addr_t)buf + i);
		sys_write32(data, cfg->base + CPTRA_MBOX_DATAIN);
	}

	if (i != size) {
		for (data = 0; i < size; ++i)
			data <<= sys_read8((mem_addr_t)buf + i);

		sys_write32(data, cfg->base + CPTRA_MBOX_DATAIN);
	}

	sys_write32(0x1, cfg->base + CPTRA_MBOX_EXEC);

	/* check update reset occurs */
	while (count++ < CPTRA_UPD_RST_TIMEOUT) {
		sts = sys_read32(cfg->ifc_base + CPTRA_RST_REASON);
		if (sts & CPTRA_FW_UPD_RESET) {
			LOG_INF("FW Update Reset !!!");
			break;
		}
	}

	/* poll for result */
	while (1) {
		sts = FIELD_GET(CPTRA_MBOX_STS_PS, cptra_mbox_status());
		if (sts != CPTRA_MBSTS_CMD_BUSY)
			break;
	}

	while (cptra_mbox_unlock())
		;

	state->in_use = false;

	return (sts == CPTRA_MBSTS_CMD_COMPLETE) ? 0 : -EIO;
}

int cptra_update_init(const struct device *dev)
{
	struct cptra_update_drv_state *state = DEV_DATA(dev);
	struct cptra_update_config *cfg = DEV_CFG(dev);

	state->in_use = false;

	if (!(sys_read32(cfg->scu_base + SCU1_CPTRA) & SCU1_CPTRA_RDY_FOR_RT)) {
		LOG_WRN("Caliptra is unavailable");
		return -ENODEV;
	}

	LOG_INF("\t0x%x: Caliptra update driver initialized", (uint32_t)cfg->base);

	return 0;
}

static struct cptra_driver_api cptra_funcs = {
	.caliptra_fw_upload = aspeed_cptra_fw_upload,
	.caliptra_fw_upload_init = aspeed_cptra_fw_upload_init,
	.caliptra_fw_upload_update = aspeed_cptra_fw_upload_update,
	.caliptra_fw_upload_final = aspeed_cptra_fw_upload_final,
};

static const struct cptra_update_config cptra_update_config = {
	.base = DT_REG_ADDR_BY_IDX(DT_PARENT(DT_DRV_INST(0)), 0),
	.ifc_base = DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PARENT(DT_DRV_INST(0)), aspeed_cptra_ifc, 0)),
	.scu_base = DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PARENT(DT_DRV_INST(0)), aspeed_scu, 0)),
};

static struct cptra_update_drv_state cptra_update_state;

#define ASPEED_CPTRA_UPD_INIT(inst)						\
	DEVICE_DT_INST_DEFINE(inst, cptra_update_init, NULL,			\
		      &cptra_update_state, &cptra_update_config,		\
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
		      &cptra_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_UPD_INIT)
