/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_dice

#include <soc.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/cptra.h>
#include <zephyr/drivers/misc/aspeed/cptra_mbox.h>

LOG_MODULE_REGISTER(cptra_dice, CONFIG_LOG_DEFAULT_LEVEL);

/* Device config */
struct cptra_dice_config {
	uintptr_t base;			/* Caliptra mbox base address */
	uintptr_t ifc_base;		/* Caliptra soc ifc base address */
	uintptr_t scu_base;		/* SCU1 base address */
};

struct cptra_dice_drv_state {
	bool in_use;
};

#define DEV_CFG(dev)				\
	((struct cptra_dice_config *)		\
	(dev)->config)

#define DEV_DATA(dev)				\
	((struct cptra_dice_drv_state *)	\
	(dev)->data)

static void aspeed_cptra_ifc_error(const struct device *dev)
{
	struct cptra_dice_config *cfg = DEV_CFG(dev);

	LOG_ERR("CPTRA error: 0x%08x 0x%08x 0x%08x 0x%08x",
		sys_read32(cfg->ifc_base + 0x0), sys_read32(cfg->ifc_base + 0x4),
		sys_read32(cfg->ifc_base + 0x8), sys_read32(cfg->ifc_base + 0xc));
}

static int aspeed_cptra_extend_pcr(const struct device *dev, struct cptra_extend_pcr_ia *input,
				   struct cptra_extend_pcr_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_EXTEND_PCR;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing extend pcr");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_extend_pcr_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_extend_pcr_ia);
	ilen = sizeof(struct cptra_extend_pcr_ia);
	olen = sizeof(struct cptra_extend_pcr_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x\n", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_quote_pcrs(const struct device *dev, struct cptra_quote_pcrs_ia *input,
				   struct cptra_quote_pcrs_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_QUOTE_PCRS;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing quote pcrs");

	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_quote_pcrs_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_quote_pcrs_ia);
	ilen = sizeof(struct cptra_quote_pcrs_ia);
	olen = sizeof(struct cptra_quote_pcrs_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x\n", output->chksum, output->fips_status);
	LOG_HEXDUMP_INF(output->PCRs[31], sizeof(output->PCRs[31]), "PCRs[31]:");
	LOG_HEXDUMP_INF(output->nonce, sizeof(output->nonce), "nonce:");
	LOG_HEXDUMP_INF(output->digest, sizeof(output->digest), "digest:");
	LOG_HEXDUMP_INF(output->reset_ctrs, sizeof(output->reset_ctrs), "reset_ctrs:");
	LOG_HEXDUMP_INF(output->signature_r, sizeof(output->signature_r), "signature_r:");
	LOG_HEXDUMP_INF(output->signature_s, sizeof(output->signature_s), "signature_s:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_stash_measurement(const struct device *dev,
					  struct cptra_stash_measurement_ia *input,
					  struct cptra_stash_measurement_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_STASH_MEASUREMENT;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing stash measurement");

	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_stash_measurement_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_stash_measurement_ia);
	ilen = sizeof(struct cptra_stash_measurement_ia);
	olen = sizeof(struct cptra_stash_measurement_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x, dpe_result: 0x%x",
		output->chksum, output->fips_status, output->dpe_result);

	state->in_use = false;

	return rc;
}

int cptra_dice_init(const struct device *dev)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	struct cptra_dice_config *cfg = DEV_CFG(dev);

	state->in_use = false;

	if (!(sys_read32(cfg->scu_base + SCU1_CPTRA) & SCU1_CPTRA_RDY_FOR_RT)) {
		LOG_ERR("Caliptra is unavailable\n");
		return -ENODEV;
	}

	LOG_INF("0x%x: Aspeed Caliptra dice service is ready", (uint32_t)cfg->base);

	return 0;
}

static struct cptra_driver_api cptra_funcs = {
	.caliptra_stash_measurement = aspeed_cptra_stash_measurement,
	.caliptra_quote_pcrs = aspeed_cptra_quote_pcrs,
	.caliptra_extend_pcr = aspeed_cptra_extend_pcr,
};

static const struct cptra_dice_config cptra_dice_config = {
	.base = DT_REG_ADDR_BY_IDX(DT_PARENT(DT_DRV_INST(0)), 0),
	.ifc_base = DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PARENT(DT_DRV_INST(0)), aspeed_cptra_ifc, 0)),
	.scu_base = DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PARENT(DT_DRV_INST(0)), aspeed_scu, 0)),
};

static struct cptra_dice_drv_state cptra_dice_state;

#define ASPEED_CPTRA_DICE_INIT(inst)						\
	DEVICE_DT_INST_DEFINE(inst, cptra_dice_init, NULL,			\
		      &cptra_dice_state, &cptra_dice_config,			\
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
		      &cptra_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_DICE_INIT)
