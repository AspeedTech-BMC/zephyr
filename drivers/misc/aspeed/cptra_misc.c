/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_misc

#include <soc.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/cptra.h>
#include <zephyr/drivers/misc/aspeed/cptra_mbox.h>

LOG_MODULE_REGISTER(cptra_misc, CONFIG_LOG_DEFAULT_LEVEL);

/* Device config */
struct cptra_misc_config {
	uintptr_t base;			/* Caliptra mbox base address */
	uintptr_t ifc_base;		/* Caliptra soc ifc base address */
	uintptr_t scu_base;		/* SCU1 base address */
};

struct cptra_misc_drv_state {
	bool in_use;
};

#define DEV_CFG(dev)				\
	((struct cptra_misc_config *)		\
	(dev)->config)

#define DEV_DATA(dev)				\
	((struct cptra_misc_drv_state *)	\
	(dev)->data)

static void aspeed_cptra_ifc_error(const struct device *dev)
{
	struct cptra_misc_config *cfg = DEV_CFG(dev);

	LOG_ERR("CPTRA error: 0x%08x 0x%08x 0x%08x 0x%08x",
		sys_read32(cfg->ifc_base + 0x0), sys_read32(cfg->ifc_base + 0x4),
		sys_read32(cfg->ifc_base + 0x8), sys_read32(cfg->ifc_base + 0xc));
}

static int aspeed_cptra_set_auth_manifest(const struct device *dev,
					  struct cptra_set_auth_manifest_ia *input,
					  struct cptra_set_auth_manifest_oa *output)
{
	struct cptra_misc_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_SET_AUTH_MANIFEST;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing set_auth_manifest");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input,
			       sizeof(struct cptra_set_auth_manifest_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_set_auth_manifest_ia);
	ilen = sizeof(struct cptra_set_auth_manifest_ia);
	olen = sizeof(struct cptra_set_auth_manifest_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_shutdown(const struct device *dev, struct cptra_shutdown_ia *input,
				 struct cptra_shutdown_oa *output)
{
	struct cptra_misc_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_SHUTDOWN;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing shutdown");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_shutdown_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_shutdown_ia);
	ilen = sizeof(struct cptra_shutdown_ia);
	olen = sizeof(struct cptra_shutdown_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_self_test_get_results(const struct device *dev,
					      struct cptra_self_test_get_results_ia *input,
					      struct cptra_self_test_get_results_oa *output)
{
	struct cptra_misc_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_SELF_TEST_GET_RESULTS;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing self_test_get_results");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input,
			       sizeof(struct cptra_self_test_get_results_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_self_test_get_results_ia);
	ilen = sizeof(struct cptra_self_test_get_results_ia);
	olen = sizeof(struct cptra_self_test_get_results_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_self_test_start(const struct device *dev,
					struct cptra_self_test_start_ia *input,
					struct cptra_self_test_start_oa *output)
{
	struct cptra_misc_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_SELF_TEST_START;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing self_test_start");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_self_test_start_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_self_test_start_ia);
	ilen = sizeof(struct cptra_self_test_start_ia);
	olen = sizeof(struct cptra_self_test_start_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_version(const struct device *dev, struct cptra_version_ia *input,
				struct cptra_version_oa *output)
{
	struct cptra_misc_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_FIPS_VERSION;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing version");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_version_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_version_ia);
	ilen = sizeof(struct cptra_version_ia);
	olen = sizeof(struct cptra_version_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_INF("mode: 0x%x", output->mode);
	LOG_HEXDUMP_INF(output->fips_rev, sizeof(output->fips_rev), "fips_rev:");
	LOG_INF("name: %.12s", output->name);

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_capabilities(const struct device *dev, struct cptra_capabilities_ia *input,
				     struct cptra_capabilities_oa *output)
{
	struct cptra_misc_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_CAPABILITIES;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing capabilities");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_capabilities_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_capabilities_ia);
	ilen = sizeof(struct cptra_capabilities_ia);
	olen = sizeof(struct cptra_capabilities_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_HEXDUMP_INF(output->capabilities, sizeof(output->capabilities), "capabilities:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_fw_info(const struct device *dev, struct cptra_fw_info_ia *input,
				struct cptra_fw_info_oa *output)
{
	struct cptra_misc_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_FW_INFO;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing fw_info");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_fw_info_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_fw_info_ia);
	ilen = sizeof(struct cptra_fw_info_ia);
	olen = sizeof(struct cptra_fw_info_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x", output->chksum, output->fips_status);
	LOG_INF("pl0_pauser: 0x%x", output->pl0_pauser);
	LOG_INF("runtime_svn: 0x%x", output->runtime_svn);
	LOG_INF("min_runtime_svn: 0x%x", output->min_runtime_svn);
	LOG_INF("fmc_manifest_svn: 0x%x", output->fmc_manifest_svn);
	LOG_INF("attestation_disabled: 0x%x", output->attestation_disabled);
	LOG_HEXDUMP_INF(output->rom_revision, sizeof(output->rom_revision), "rom_revision:");
	LOG_HEXDUMP_INF(output->fmc_revision, sizeof(output->fmc_revision), "fmc_revision:");
	LOG_HEXDUMP_INF(output->runtime_revision, sizeof(output->runtime_revision),
			"runtime_revision:");
	LOG_HEXDUMP_INF(output->rom_sha256_digest, sizeof(output->rom_sha256_digest),
			"rom_sha256_digest:");
	LOG_HEXDUMP_INF(output->fmc_sha384_digest, sizeof(output->fmc_sha384_digest),
			"fmc_sha384_digest:");
	LOG_HEXDUMP_INF(output->runtime_sha384_digest, sizeof(output->runtime_sha384_digest),
			"runtime_sha384_digest:");

	state->in_use = false;

	return rc;
}

static int aspeed_cptra_authorize_and_stash(const struct device *dev,
					    struct cptra_authorize_and_stash_ia *input,
					    struct cptra_authorize_and_stash_oa *output)
{
	struct cptra_misc_drv_state *state = DEV_DATA(dev);
	uint32_t cmd = CPTRA_MBCMD_AUTHORIZE_AND_STASH;
	uint32_t csum = 0, sts, dlen, ilen, olen;
	int rc;

	if (state->in_use) {
		LOG_ERR("Peripheral in use");
		return -EBUSY;
	}

	LOG_INF("Start doing authorize_and_stash");
	state->in_use = true;

	while (cptra_mbox_lock())
		;

	/* check MBOX is ready for command */
	sts = cptra_mbox_status();
	if (FIELD_GET(CPTRA_MBOX_STS_FSM_PS, sts) != CPTRA_MBFSM_RDY_FOR_CMD)
		return -EACCES;

	csum = cptra_mbox_csum(csum, (uint8_t *)&cmd, sizeof(cmd));
	csum = cptra_mbox_csum(csum, (uint8_t *)input, sizeof(struct cptra_authorize_and_stash_ia));

	/* init mbox parameters */
	dlen = sizeof(csum) + sizeof(struct cptra_authorize_and_stash_ia);
	ilen = sizeof(struct cptra_authorize_and_stash_ia);
	olen = sizeof(struct cptra_authorize_and_stash_oa);
	rc = cptra_mbox_trigger(cmd, dlen, csum, (uint8_t *)input, ilen, (uint8_t *)output, olen);
	if (rc)
		aspeed_cptra_ifc_error(dev);

	while (cptra_mbox_unlock())
		;

	LOG_INF("chksum: 0x%x, fips_status: 0x%x, auth_req_result: 0x%x",
		output->chksum, output->fips_status, output->auth_req_result);

	state->in_use = false;

	return rc;
}

int cptra_misc_init(const struct device *dev)
{
	struct cptra_misc_drv_state *state = DEV_DATA(dev);
	struct cptra_misc_config *cfg = DEV_CFG(dev);

	state->in_use = false;

	if (!(sys_read32(cfg->scu_base + SCU1_CPTRA) & SCU1_CPTRA_RDY_FOR_RT)) {
		LOG_WRN("Caliptra is unavailable");
		return -ENODEV;
	}

	LOG_INF("\t0x%x: Caliptra misc driver initialized", (uint32_t)cfg->base);

	return 0;
}

static struct cptra_driver_api cptra_funcs = {
	.caliptra_fw_info = aspeed_cptra_fw_info,
	.caliptra_capabilities = aspeed_cptra_capabilities,
	.caliptra_version = aspeed_cptra_version,
	.caliptra_self_test_start = aspeed_cptra_self_test_start,
	.caliptra_self_test_get_results = aspeed_cptra_self_test_get_results,
	.caliptra_shutdown = aspeed_cptra_shutdown,
	.caliptra_set_auth_manifest = aspeed_cptra_set_auth_manifest,
	.caliptra_authorize_and_stash = aspeed_cptra_authorize_and_stash,
};

static const struct cptra_misc_config cptra_misc_config = {
	.base = DT_REG_ADDR_BY_IDX(DT_PARENT(DT_DRV_INST(0)), 0),
	.ifc_base = DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PARENT(DT_DRV_INST(0)), aspeed_cptra_ifc, 0)),
	.scu_base = DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_PARENT(DT_DRV_INST(0)), aspeed_scu, 0)),
};

static struct cptra_misc_drv_state cptra_misc_state;

#define ASPEED_CPTRA_MISC_INIT(inst)						\
	DEVICE_DT_INST_DEFINE(inst, cptra_misc_init, NULL,			\
		      &cptra_misc_state, &cptra_misc_config,			\
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
		      &cptra_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_MISC_INIT)
