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

static int aspeed_cptra_get_fmc_alias_csr(const struct device *dev,
					  struct cptra_get_fmc_alias_csr_ia *input,
					  struct cptra_get_fmc_alias_csr_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_GET_FMC_ALIAS_CSR;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing get_fmc_alias_csr");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD) {
		return -EACCES;
	}

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_get_fmc_alias_csr_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_get_fmc_alias_csr_ia);
	ilen = sizeof(struct cptra_get_fmc_alias_csr_ia);
	olen = sizeof(struct cptra_get_fmc_alias_csr_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc) {
		aspeed_cptra_ifc_error(dev);
	}

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, data_size: 0x%x", output->chksum, output->data_size);
	LOG_HEXDUMP_INF(output->data, output->data_size, "FMC ALIAS CSR:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_get_rt_alias_cert(const struct device *dev,
					  struct cptra_get_rt_alias_cert_ia *input,
					  struct cptra_get_rt_alias_cert_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_GET_RT_ALIAS_CERT;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing get_rt_alias_cert");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_get_rt_alias_cert_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_get_rt_alias_cert_ia);
	ilen = sizeof(struct cptra_get_rt_alias_cert_ia);
	olen = sizeof(struct cptra_get_rt_alias_cert_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_INF("data_size: 0x%x", output->data_size);
	LOG_HEXDUMP_INF(output->data, output->data_size, "RT ALIAS CERT:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_get_fmc_alias_cert(const struct device *dev,
					   struct cptra_get_fmc_alias_cert_ia *input,
					   struct cptra_get_fmc_alias_cert_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_GET_FMC_ALIAS_CERT;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing get_fmc_alias_cert");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_get_fmc_alias_cert_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_get_fmc_alias_cert_ia);
	ilen = sizeof(struct cptra_get_fmc_alias_cert_ia);
	olen = sizeof(struct cptra_get_fmc_alias_cert_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_INF("data_size: 0x%x", output->data_size);
	LOG_HEXDUMP_INF(output->data, output->data_size, "FMC ALIAS CERT:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_get_ldev_cert(const struct device *dev,
				      struct cptra_get_ldev_cert_ia *input,
				      struct cptra_get_ldev_cert_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_GET_LDEV_CERT;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing get_ldev_cert");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_get_ldev_cert_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_get_ldev_cert_ia);
	ilen = sizeof(struct cptra_get_ldev_cert_ia);
	olen = sizeof(struct cptra_get_ldev_cert_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_INF("data_size: 0x%x", output->data_size);
	LOG_HEXDUMP_INF(output->data, output->data_size, "LDEVID CERT:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_get_idev_info(const struct device *dev,
				      struct cptra_get_idev_info_ia *input,
				      struct cptra_get_idev_info_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_GET_IDEV_INFO;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing get_idev_info");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_get_idev_info_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_get_idev_info_ia);
	ilen = sizeof(struct cptra_get_idev_info_ia);
	olen = sizeof(struct cptra_get_idev_info_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_HEXDUMP_INF(output->idev_pub_x, sizeof(output->idev_pub_x), "idev_pub_x:");
	LOG_HEXDUMP_INF(output->idev_pub_y, sizeof(output->idev_pub_y), "idev_pub_y:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_populate_idev_cert(const struct device *dev,
					   struct cptra_populate_idev_cert_ia *input,
					   struct cptra_populate_idev_cert_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_POPULATE_IDEV_CERT;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing populate_idev_cert");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_populate_idev_cert_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_populate_idev_cert_ia);
	ilen = sizeof(struct cptra_populate_idev_cert_ia);
	olen = sizeof(struct cptra_populate_idev_cert_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_get_idev_cert(const struct device *dev,
				      struct cptra_get_idev_cert_ia *input,
				      struct cptra_get_idev_cert_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_GET_IDEV_CERT;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing get_idev_cert");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_get_idev_cert_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_get_idev_cert_ia);
	ilen = sizeof(struct cptra_get_idev_cert_ia);
	olen = sizeof(struct cptra_get_idev_cert_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_INF("cert_size: 0x%x", output->cert_size);
	LOG_HEXDUMP_INF(output->cert, output->cert_size, "IDEVID CERT:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_get_idevid_csr(const struct device *dev,
				       struct cptra_get_idevid_csr_ia *input,
				       struct cptra_get_idevid_csr_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_GET_IDEVID_CSR;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing get_idevid_csr");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD) {
		return -EACCES;
	}

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_get_idevid_csr_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_get_idevid_csr_ia);
	ilen = sizeof(struct cptra_get_idevid_csr_ia);
	olen = sizeof(struct cptra_get_idevid_csr_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc) {
		aspeed_cptra_ifc_error(dev);
	}

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, data_size: 0x%x", output->chksum, output->data_size);
	LOG_HEXDUMP_INF(output->data, output->data_size, "IDEVID CSR:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_invoke_dpe_command(const struct device *dev,
					   struct cptra_invoke_dpe_command_ia *input,
					   struct cptra_invoke_dpe_command_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_INVOKE_DPE_COMMAND;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing invoke_dpe_command");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_invoke_dpe_command_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_invoke_dpe_command_ia);
	ilen = sizeof(struct cptra_invoke_dpe_command_ia);
	olen = sizeof(struct cptra_invoke_dpe_command_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	if (output->data_size > 0x400)
		dlen = 0x400;
	else
		dlen = output->data_size;
	LOG_HEXDUMP_INF(output->data, dlen, "DPE COMMAND RESP:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_disable_attestation(const struct device *dev,
					    struct cptra_disable_attestation_ia *input,
					    struct cptra_disable_attestation_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_DISABLE_ATTESTATION;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing disable_attestation");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_disable_attestation_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_disable_attestation_ia);
	ilen = sizeof(struct cptra_disable_attestation_ia);
	olen = sizeof(struct cptra_disable_attestation_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_certify_key_extended(const struct device *dev,
					     struct cptra_certify_key_extended_ia *input,
					     struct cptra_certify_key_extended_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_CERTIFY_KEY_EXTENDED;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing certify_key_extended");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input,
			       sizeof(struct cptra_certify_key_extended_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_certify_key_extended_ia);
	ilen = sizeof(struct cptra_certify_key_extended_ia);
	olen = sizeof(struct cptra_certify_key_extended_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_HEXDUMP_INF(output->certify_key_resp, sizeof(output->certify_key_resp),
			"certify_key_resp:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_add_subject_alt_name(const struct device *dev,
					     struct cptra_add_subject_alt_name_ia *input,
					     struct cptra_add_subject_alt_name_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_ADD_SUBJECT_ALT_NAME;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing add_subject_alt_name");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input,
			       sizeof(struct cptra_add_subject_alt_name_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_add_subject_alt_name_ia);
	ilen = sizeof(struct cptra_add_subject_alt_name_ia);
	olen = sizeof(struct cptra_add_subject_alt_name_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_dpe_get_tagged_tci(const struct device *dev,
					   struct cptra_dpe_get_tagged_tci_ia *input,
					   struct cptra_dpe_get_tagged_tci_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_DPE_GET_TAGGED_TCI;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing dpe_get_tagged_tci");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_dpe_get_tagged_tci_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_dpe_get_tagged_tci_ia);
	ilen = sizeof(struct cptra_dpe_get_tagged_tci_ia);
	olen = sizeof(struct cptra_dpe_get_tagged_tci_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_HEXDUMP_INF(output->tci_cumulative, sizeof(output->tci_cumulative), "tci_cumulative:");
	LOG_HEXDUMP_INF(output->tci_current, sizeof(output->tci_current), "tci_current:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_dpe_tag_tci(const struct device *dev, struct cptra_dpe_tag_tci_ia *input,
				    struct cptra_dpe_tag_tci_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_DPE_TAG_TCI;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing dpe_tag_tci");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_dpe_tag_tci_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_dpe_tag_tci_ia);
	ilen = sizeof(struct cptra_dpe_tag_tci_ia);
	olen = sizeof(struct cptra_dpe_tag_tci_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
}

static int
aspeed_cptra_increment_pcr_reset_counter(const struct device *dev,
					 struct cptra_increment_pcr_reset_counter_ia *input,
					 struct cptra_increment_pcr_reset_counter_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_INCREMENT_PCR_RESET_COUNTER;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing increment_pcr_reset_counter");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input,
			       sizeof(struct cptra_increment_pcr_reset_counter_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_increment_pcr_reset_counter_ia);
	ilen = sizeof(struct cptra_increment_pcr_reset_counter_ia);
	olen = sizeof(struct cptra_increment_pcr_reset_counter_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
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

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

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

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
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

static int aspeed_cptra_sign_with_exported_ecdsa(const struct device *dev,
						 struct cptra_sign_with_exported_ecdsa_ia *input,
						 struct cptra_sign_with_exported_ecdsa_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_SIGN_WITH_EXPORTED_ECDSA;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing sign_with_exported_ecdsa");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* Check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD) {
		rc = -EACCES;
		goto unlock;
	}

	/* Compute checksum */
	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input,
			       sizeof(struct cptra_sign_with_exported_ecdsa_ia));

	/* Initialize mailbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_sign_with_exported_ecdsa_ia);
	ilen = sizeof(struct cptra_sign_with_exported_ecdsa_ia);
	olen = sizeof(struct cptra_sign_with_exported_ecdsa_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

unlock:
	while (cptra_mbox_unlock())
		;

	LOG_INF("Derived Public Key X:");
	LOG_HEXDUMP_INF(output->derived_pubkey_x, sizeof(output->derived_pubkey_x), "X:");
	LOG_INF("Derived Public Key Y:");
	LOG_HEXDUMP_INF(output->derived_pubkey_y, sizeof(output->derived_pubkey_y), "Y:");
	LOG_INF("Signature R:");
	LOG_HEXDUMP_INF(output->signature_r, sizeof(output->signature_r), "R:");
	LOG_INF("Signature S:");
	LOG_HEXDUMP_INF(output->signature_s, sizeof(output->signature_s), "S:");

	state->in_use = false;

	return rc;
}

static int
aspeed_cptra_revoke_exported_cdi_handle(const struct device *dev,
					struct cptra_revoke_exported_cdi_handle_ia *input,
					struct cptra_revoke_exported_cdi_handle_oa *output)
{
	struct cptra_dice_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_REVOKE_EXPORTED_CDI_HANDLE;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing revoke_exported_cdi_handle");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD) {
		rc = -EACCES;
		goto unlock;
	}

	/* Compute checksum */
	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input,
			       sizeof(struct cptra_revoke_exported_cdi_handle_ia));

	/* Initialize mailbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_revoke_exported_cdi_handle_ia);
	ilen = sizeof(struct cptra_revoke_exported_cdi_handle_ia);
	olen = sizeof(struct cptra_revoke_exported_cdi_handle_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

unlock:
	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

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
	.caliptra_increment_pcr_reset_counter = aspeed_cptra_increment_pcr_reset_counter,
	.caliptra_dpe_tag_tci = aspeed_cptra_dpe_tag_tci,
	.caliptra_dpe_get_tagged_tci = aspeed_cptra_dpe_get_tagged_tci,
	.caliptra_add_subject_alt_name = aspeed_cptra_add_subject_alt_name,
	.caliptra_certify_key_extended = aspeed_cptra_certify_key_extended,
	.caliptra_disable_attestation = aspeed_cptra_disable_attestation,
	.caliptra_invoke_dpe_command = aspeed_cptra_invoke_dpe_command,
	.caliptra_get_idev_cert = aspeed_cptra_get_idev_cert,
	.caliptra_populate_idev_cert = aspeed_cptra_populate_idev_cert,
	.caliptra_get_idev_info = aspeed_cptra_get_idev_info,
	.caliptra_get_ldev_cert = aspeed_cptra_get_ldev_cert,
	.caliptra_get_fmc_alias_cert = aspeed_cptra_get_fmc_alias_cert,
	.caliptra_get_rt_alias_cert = aspeed_cptra_get_rt_alias_cert,
	.caliptra_get_idevid_csr = aspeed_cptra_get_idevid_csr,
	.caliptra_get_fmc_alias_csr = aspeed_cptra_get_fmc_alias_csr,
	.caliptra_sign_with_exported_ecdsa = aspeed_cptra_sign_with_exported_ecdsa,
	.caliptra_revoke_exported_cdi_handle = aspeed_cptra_revoke_exported_cdi_handle,
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
