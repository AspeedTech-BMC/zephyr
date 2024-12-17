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

LOG_MODULE_REGISTER(cptra_update, CONFIG_LOG_DEFAULT_LEVEL);

/* Device config */
struct cptra_update_config {
	uintptr_t base;			/* Caliptra mbox base address */
	uintptr_t soc_ifc_base;		/* Caliptra soc ifc base address */
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

#define SCU1_RNG_DATA			0x14c020f4

/* TODO: use sys_rand() instead */
static uint32_t rand(void)
{
	return sys_read32(SCU1_RNG_DATA);
}

static void trng_req_service(const struct device *dev)
{
	struct cptra_update_config *cfg = DEV_CFG(dev);
	int count = 0;
	uint32_t ts;

	while (count < CPTRA_TRNG_REQ_LOOP_CNT) {
		ts = sys_read32(cfg->soc_ifc_base + CPTRA_TRNG_STS);

		if ((ts & CPTRA_TRNG_STS_DATA_REQ)) {
			for (int i = 0; i < CPTRA_MAX_TRNG; ++i)
				sys_write32(rand(), cfg->soc_ifc_base + CPTRA_TRNG_DATA(i));
			sys_write32(CPTRA_TRNG_STS_DATA_WR_DONE, cfg->soc_ifc_base +
				    CPTRA_TRNG_STS);
		}
		count++;
	}
}

static int aspeed_cptra_fw_upload(const struct device *dev, uint8_t *buf, int size)
{
	struct cptra_update_drv_state *state = DEV_DATA(dev);
	struct cptra_update_config *cfg = DEV_CFG(dev);
	struct cptra_mbox_register_s *mbox_register;
	union cptra_mbox_lock_s mbox_lock;
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
	mbox_register = (struct cptra_mbox_register_s *)cfg->base;

	/* get CPTRA MBOX lock */
	if (reg_read_poll_timeout(mbox_register, mbox_lock, mbox_lock,
				  mbox_lock.fields.lock == 0, 10, 1000))
		return -EBUSY;

	/* check MBOX is ready for command */
	sts = sys_read32(cfg->base + CPTRA_MBOX_STS);
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
		sts = sys_read32(cfg->soc_ifc_base + CPTRA_RST_REASON);
		if (sts & CPTRA_FW_UPD_RESET) {
			LOG_INF("FW Update Reset !!!");
			break;
		}
	}

	/* service cptra trng request
	 * TODO: enable interrupt mode to service
	 */
	trng_req_service(dev);

	/* poll for result */
	while (1) {
		sts = FIELD_GET(CPTRA_MBOX_STS_PS, sys_read32(cfg->base + CPTRA_MBOX_STS));
		if (sts != CPTRA_MBSTS_CMD_BUSY)
			break;
	}

	sys_write32(0x0, cfg->base + CPTRA_MBOX_EXEC);
	state->in_use = false;

	return (sts == CPTRA_MBSTS_CMD_COMPLETE) ? 0 : -EIO;
}

int cptra_update_init(const struct device *dev)
{
	struct cptra_update_drv_state *state = DEV_DATA(dev);
	struct cptra_update_config *cfg = DEV_CFG(dev);

	state->in_use = false;

	if (!(sys_read32(cfg->scu_base + SCU1_CPTRA) & SCU1_CPTRA_RDY_FOR_RT)) {
		LOG_ERR("Caliptra is unavailable\n");
		return -ENODEV;
	}

	LOG_INF("0x%x: Aspeed Caliptra update service is ready", (uint32_t)cfg->base);

	return 0;
}

static struct cptra_driver_api cptra_funcs = {
	.caliptra_fw_upload = aspeed_cptra_fw_upload,
};

static const struct cptra_update_config cptra_update_config = {
	.base = DT_REG_ADDR_BY_IDX(DT_PARENT(DT_DRV_INST(0)), 0),
	.soc_ifc_base = DT_REG_ADDR_BY_IDX(DT_PARENT(DT_DRV_INST(0)), 1),
	.scu_base = DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PARENT(DT_DRV_INST(0)), aspeed_scu, 0)),
};

static struct cptra_update_drv_state cptra_update_state;

DEVICE_DT_INST_DEFINE(0, cptra_update_init, NULL,
		      &cptra_update_state, &cptra_update_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &cptra_funcs);
