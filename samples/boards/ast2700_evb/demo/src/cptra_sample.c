/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#include <zephyr/drivers/cptra.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/misc/aspeed/cptra_ipc.h>
#include <zephyr/shell/shell.h>
#include <zephyr/crypto/hash.h>
#include "cptra_sample.h"
#include <zephyr/sys/byteorder.h>

#if defined(CONFIG_MBEDTLS)
#include <mbedtls/sha512.h>
#endif

LOG_MODULE_REGISTER(cptra_test, CONFIG_SOC_LOG_LEVEL);

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
#define CPTRA_UPDATE_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_update))
#define CPTRA_DICE_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_dice))
#define CPTRA_MISC_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_misc))
#elif CONFIG_CPTRA_SAMPLE_SSP
#define CPTRA_UPDATE_DRV_NAME		NULL
#define CPTRA_DICE_DRV_NAME		NULL
#define CPTRA_MISC_DRV_NAME		NULL
#endif

#define SPI_TO_DRAM_BASE_ADDR		0x2c000000
#define CPTRA_FW_ADDR			0x20000000
#define CPTRA_FW_SIZE			0x20000

#define FMC_DEV_NAME			"fmc@0"

__attribute__((unused)) static void cptra_test_shutdown(void)
{
	struct cptra_shutdown_ia input;
	struct cptra_shutdown_oa output;
	int ret;

	LOG_INF("Test caliptra_shutdown...");

	memset(&input, 0, sizeof(struct cptra_shutdown_ia));
	memset(&output, 0, sizeof(struct cptra_shutdown_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);

	ret = caliptra_shutdown(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_SHUTDOWN,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_shutdown is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_shutdown is successful");

	LOG_DBG("output: chksum=0x%x, fips_status=0x%x",
		output.chksum, output.fips_status);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_version(void)
{
	struct cptra_version_ia input;
	struct cptra_version_oa output;
	int ret;

	LOG_INF("Test caliptra_version...");

	memset(&input, 0, sizeof(struct cptra_version_ia));
	memset(&output, 0, sizeof(struct cptra_version_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);

	ret = caliptra_version(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_FIPS_VERSION,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_version is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_version is successful");

	LOG_DBG("output: chksum=0x%x, fips_status=0x%x",
		output.chksum, output.fips_status);

	LOG_DBG("mode: 0x%x", output.mode);
	LOG_HEXDUMP_DBG(output.fips_rev, sizeof(output.fips_rev), "fips_rev:");
	LOG_HEXDUMP_DBG(output.name, sizeof(output.name), "name:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_capabilities(void)
{
	struct cptra_capabilities_ia input;
	struct cptra_capabilities_oa output;
	int ret;

	LOG_INF("Test caliptra_capabilities...");

	memset(&input, 0, sizeof(struct cptra_capabilities_ia));
	memset(&output, 0, sizeof(struct cptra_capabilities_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);

	ret = caliptra_capabilities(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_CAPABILITIES,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif
	if (ret) {
		LOG_ERR("caliptra_capabilities is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_capabilities is successful");

	LOG_DBG("output: chksum=0x%x, fips_status=0x%x",
		output.chksum, output.fips_status);

	LOG_HEXDUMP_DBG(output.capabilities, sizeof(output.capabilities), "capabilities:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_fw_info(void)
{
	struct cptra_fw_info_ia input;
	struct cptra_fw_info_oa output;
	int ret;

	LOG_INF("Test caliptra_fw_info...");

	memset(&input, 0, sizeof(struct cptra_fw_info_ia));
	memset(&output, 0, sizeof(struct cptra_fw_info_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);

	ret = caliptra_fw_info(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_FW_INFO,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_fw_info is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_fw_info is successful");

	LOG_DBG("output: chksum=0x%x, fips_status=0x%x",
		output.chksum, output.fips_status);
	LOG_DBG("pl0_pauser=0x%x", output.pl0_pauser);
	LOG_DBG("runtime_svn=0x%x", output.runtime_svn);
	LOG_DBG("min_runtime_svn=0x%x", output.min_runtime_svn);
	LOG_DBG("fmc_manifest_svn: 0x%x", output.fmc_manifest_svn);
	LOG_DBG("attestation_disabled: 0x%x", output.attestation_disabled);
	LOG_HEXDUMP_DBG(output.rom_revision, sizeof(output.rom_revision), "rom_revision:");
	LOG_HEXDUMP_DBG(output.fmc_revision, sizeof(output.fmc_revision), "fmc_revision:");
	LOG_HEXDUMP_DBG(output.runtime_revision, sizeof(output.runtime_revision),
			"runtime_revision:");
	LOG_HEXDUMP_DBG(output.rom_sha256_digest, sizeof(output.rom_sha256_digest),
			"rom_sha256_digest:");
	LOG_HEXDUMP_DBG(output.fmc_sha384_digest, sizeof(output.fmc_sha384_digest),
			"fmc_sha384_digest:");
	LOG_HEXDUMP_DBG(output.runtime_sha384_digest, sizeof(output.runtime_sha384_digest),
			"runtime_sha384_digest:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_get_rt_alias_cert(void)
{
	struct cptra_get_rt_alias_cert_ia input;
	struct cptra_get_rt_alias_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_get_rt_alias_cert...");

	memset(&input, 0, sizeof(struct cptra_get_rt_alias_cert_ia));
	memset(&output, 0, sizeof(struct cptra_get_rt_alias_cert_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_get_rt_alias_cert(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_GET_RT_ALIAS_CERT,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_get_rt_alias_cert is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_get_rt_alias_cert is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);
	LOG_HEXDUMP_DBG(output.data, output.data_size, "rt_alias_cert:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_get_fmc_alias_cert(void)
{
	struct cptra_get_fmc_alias_cert_ia input;
	struct cptra_get_fmc_alias_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_get_fmc_alias_cert...");

	memset(&input, 0, sizeof(struct cptra_get_fmc_alias_cert_ia));
	memset(&output, 0, sizeof(struct cptra_get_fmc_alias_cert_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_get_fmc_alias_cert(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_GET_FMC_ALIAS_CERT,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_get_fmc_alias_cert is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_get_fmc_alias_cert is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);
	LOG_HEXDUMP_DBG(output.data, output.data_size, "fmc_alias_cert:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_get_ldev_cert(void)
{
	struct cptra_get_ldev_cert_ia input;
	struct cptra_get_ldev_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_get_ldev_cert...");

	memset(&input, 0, sizeof(struct cptra_get_ldev_cert_ia));
	memset(&output, 0, sizeof(struct cptra_get_ldev_cert_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_get_ldev_cert(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_GET_LDEV_CERT,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_get_ldev_cert is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_get_ldev_cert is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);
	LOG_HEXDUMP_DBG(output.data, output.data_size, "ldev_cert:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_get_idev_info(void)
{
	struct cptra_get_idev_info_ia input;
	struct cptra_get_idev_info_oa output;
	int ret;

	LOG_INF("Test caliptra_get_idev_info...");

	memset(&input, 0, sizeof(struct cptra_get_idev_info_ia));
	memset(&output, 0, sizeof(struct cptra_get_idev_info_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_get_idev_info(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_GET_IDEV_INFO,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_get_idev_info is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_get_idev_info is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);
	LOG_HEXDUMP_DBG(output.idev_pub_x, sizeof(output.idev_pub_x), "idev_pub_x:");
	LOG_HEXDUMP_DBG(output.idev_pub_y, sizeof(output.idev_pub_y), "idev_pub_y:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

__attribute__((unused)) static void cptra_test_populate_idev_cert(void)
{
	struct cptra_populate_idev_cert_ia input;
	struct cptra_populate_idev_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_populate_idev_cert...");

	memset(&input, 0, sizeof(struct cptra_populate_idev_cert_ia));
	memset(&output, 0, sizeof(struct cptra_populate_idev_cert_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_populate_idev_cert(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_POPULATE_IDEV_CERT,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_populate_idev_cert is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_populate_idev_cert is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_get_idev_cert(void)
{
	struct cptra_get_idev_cert_ia input;
	struct cptra_get_idev_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_get_idev_cert...");

	memset(&input, 0, sizeof(struct cptra_get_idev_cert_ia));
	memset(&output, 0, sizeof(struct cptra_get_idev_cert_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_get_idev_cert(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_GET_IDEV_CERT,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_get_idev_cert is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_get_idev_cert is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);
	LOG_HEXDUMP_DBG(output.cert, output.cert_size, "cert:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

/* TODO: Caliptra should be provisioned */
__attribute__((unused)) static void cptra_test_get_idevid_csr(void)
{
	struct cptra_get_idevid_csr_ia input;
	struct cptra_get_idevid_csr_oa output;
	int ret;

	LOG_INF("Test caliptra_get_idevid_csr...");

	memset(&input, 0, sizeof(struct cptra_get_idevid_csr_ia));
	memset(&output, 0, sizeof(struct cptra_get_idevid_csr_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_get_idevid_csr(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_GET_IDEVID_CSR,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_get_idevid_csr is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_get_idevid_csr is successful");

	LOG_DBG("output: chksum:0x%x, data_size:0x%x",
		output.chksum, output.data_size);
	LOG_HEXDUMP_DBG(output.data, output.data_size, "CSR data:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static int cptra_dpe_response_check(struct dpe_rsp_header *header)
{
	if (header->magic != DPE_RESPONSE_MAGIC || header->status != 0)
		return -1;

	return 0;
}

void cptra_test_invoke_dpe_get_profile(void)
{
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
#endif
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	struct dpe_get_profile_i *get_profile_input = NULL;
	struct dpe_get_profile_o *get_profile_output = NULL;
	int ret;

	LOG_INF("Test caliptra_invoke_dpe_command...");

	LOG_INF("\tTest GetProfile...");

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));

	/* Set input */
	get_profile_input = (struct dpe_get_profile_i *)input.data;
	get_profile_input->cmd_hdr.magic = DPE_COMMAND_MAGIC;
	get_profile_input->cmd_hdr.cmd = GET_PROFILE;
	input.data_size = sizeof(struct dpe_get_profile_i);

	get_profile_output = (struct dpe_get_profile_o *)output.data;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	ret = caliptra_invoke_dpe_command(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_INVOKE_DPE_COMMAND,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x", ret);
	else if (cptra_dpe_response_check(&get_profile_output->rsp_hdr)) {
		LOG_ERR("DPE command failed, magic:0x%08x status:0x%08x profile:0x%08x\n",
			get_profile_output->rsp_hdr.magic, get_profile_output->rsp_hdr.status,
			get_profile_output->rsp_hdr.profile);

	} else {
		LOG_DBG("caliptra_invoke_dpe_command is successful");
		LOG_DBG("DPE Profile: %08x\n", get_profile_output->rsp_hdr.profile);
		LOG_DBG("Major Version: %04x Minor Version: %04x\n",
			get_profile_output->major_version, get_profile_output->minor_version);
		LOG_DBG("Vendor ID: %08x Sku ID: %08x\n", get_profile_output->vendor_id,
			get_profile_output->vendor_sku);
		LOG_DBG("Max TCI Nodes: %d Flags: %08x\n", get_profile_output->max_tci_nodes,
			get_profile_output->flags);
	}
}

#define SRAM_BASE_ADDR		0x14b80000

void cptra_test_invoke_dpe_get_certificate_chain(void)
{
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
#endif
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	struct dpe_get_certificate_chain_i *get_certificate_chain_input = NULL;
	struct dpe_get_certificate_chain_o *get_certificate_chain_output = NULL;
	uint8_t *certificate_chain = (uint8_t *)SRAM_BASE_ADDR;
	uint32_t offset = 0;
	int ret;

	LOG_INF("\tTest GetCertificateChain...");

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));

	/* Set input */
	get_certificate_chain_input = (struct dpe_get_certificate_chain_i *)input.data;
	get_certificate_chain_input->cmd_hdr.magic = DPE_COMMAND_MAGIC;
	get_certificate_chain_input->cmd_hdr.cmd = GET_CERTIFICATE_CHAIN;
	get_certificate_chain_input->cmd_hdr.profile = P384Sha384;
	get_certificate_chain_output = (struct dpe_get_certificate_chain_o *)output.data;
	input.data_size = sizeof(struct dpe_get_certificate_chain_i);

	do {
		get_certificate_chain_input->offset = offset;
		get_certificate_chain_input->size =
			sizeof(get_certificate_chain_output->cert_chain);
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
		ret = caliptra_invoke_dpe_command(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
		ret = cptra_ipc_transfer(CPTRA_IPCCMD_INVOKE_DPE_COMMAND,
					 (uint32_t *)&input, sizeof(input),
					 CPTRA_IPC_RX_TYPE_EXTERNAL,
					 (uint32_t *)&output, sizeof(output));
#endif
		if (ret) {
			LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x\n", ret);
			break;

		} else {
			LOG_INF("Successful offset=%u size=%u\n", offset,
				get_certificate_chain_output->size);

			memcpy(&certificate_chain[offset],
			       get_certificate_chain_output->cert_chain,
			       get_certificate_chain_output->size);

			offset += get_certificate_chain_output->size;
			if (get_certificate_chain_output->size <
			    sizeof(get_certificate_chain_output->cert_chain)) {
				break;
			}
		}

	} while (1);

	LOG_HEXDUMP_INF(certificate_chain, offset, "certificate_chain:");
}

void cptra_test_invoke_dpe_initialize_context(void)
{
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
#endif
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	struct dpe_initialize_context_i *initialize_context_input = NULL;
	struct dpe_new_context_o *initialize_context_output = NULL;
	int ret;

	LOG_INF("Test caliptra_invoke_dpe_command...");

	LOG_INF("\tTest InitializeContext...");

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));

	/* Set input */
	initialize_context_input = (struct dpe_initialize_context_i *)input.data;
	initialize_context_input->cmd_hdr.magic = DPE_COMMAND_MAGIC;
	initialize_context_input->cmd_hdr.cmd = INITIALIZE_CONTEXT;
	initialize_context_input->cmd_hdr.profile = P384Sha384;
	initialize_context_input->init_ctx_cmd = BIT(30); /* DEFAULT_FLAG_MASK */
	input.data_size = sizeof(struct dpe_initialize_context_i);

	initialize_context_output = (struct dpe_new_context_o *)output.data;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	ret = caliptra_invoke_dpe_command(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_INVOKE_DPE_COMMAND,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x", ret);

	} else if (cptra_dpe_response_check(&initialize_context_output->rsp_hdr)) {
		LOG_ERR("DPE command failed, magic:0x%08x status:0x%08x profile:0x%08x\n",
			initialize_context_output->rsp_hdr.magic,
			initialize_context_output->rsp_hdr.status,
			initialize_context_output->rsp_hdr.profile);

	} else {
		LOG_DBG("caliptra_invoke_dpe_command is successful");
		LOG_HEXDUMP_DBG(initialize_context_output->context_handle,
				sizeof(initialize_context_output->context_handle),
				"context_handle:");
	}
}

void cptra_test_invoke_dpe_derive_context(uint8_t *derived_context)
{
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
#endif
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	struct dpe_derive_context_i *derive_context_input = NULL;
	struct dpe_derive_context_o *derive_context_output = NULL;
	int ret;

	LOG_INF("Test caliptra_invoke_dpe_command...");

	LOG_INF("\tTest DeriveContext...");

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));

	/* Set input */
	derive_context_input = (struct dpe_derive_context_i *)input.data;
	derive_context_input->cmd_hdr.magic = DPE_COMMAND_MAGIC;
	derive_context_input->cmd_hdr.cmd = DERIVE_CONTEXT;
	derive_context_input->cmd_hdr.profile = P384Sha384;
	derive_context_input->flags = BIT(25) | BIT(26) | BIT(30) | BIT(31); /* INPUT_ALLOW_X509 |
									      * INPUT_ALLOW_CA |
									      * INPUT_DICE |
									      * INPUT_INFO
									      */
	input.data_size = sizeof(struct dpe_derive_context_i);

	derive_context_output = (struct dpe_derive_context_o *)output.data;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	ret = caliptra_invoke_dpe_command(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_INVOKE_DPE_COMMAND,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x", ret);
	else if (cptra_dpe_response_check(&derive_context_output->rsp_hdr)) {
		LOG_ERR("DPE command failed, magic:0x%08x status:0x%08x profile:0x%08x\n",
			derive_context_output->rsp_hdr.magic, derive_context_output->rsp_hdr.status,
			derive_context_output->rsp_hdr.profile);

	} else {
		LOG_DBG("caliptra_invoke_dpe_command is successful");
		LOG_HEXDUMP_DBG(derive_context_output->context_handle,
				sizeof(derive_context_output->context_handle),
				"context_handle:");
		LOG_HEXDUMP_DBG(derive_context_output->parent_context_handle,
				sizeof(derive_context_output->parent_context_handle),
				"parent_context_handle:");
		memcpy(derived_context, derive_context_output->context_handle,
		       sizeof(derive_context_output->context_handle));
	}
}

void cptra_test_invoke_dpe_certify_key(void)
{
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
#endif
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	struct dpe_certify_key_i *certify_key_input = NULL;
	struct dpe_certify_key_o *certify_key_output = NULL;
	int ret;

	LOG_INF("Test caliptra_invoke_dpe_command...");

	LOG_INF("\tTest CertifyKey...");

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));

	/* Set input */
	certify_key_input = (struct dpe_certify_key_i *)input.data;
	certify_key_input->cmd_hdr.magic = DPE_COMMAND_MAGIC;
	certify_key_input->cmd_hdr.cmd = CERTIFY_KEY;
	certify_key_input->cmd_hdr.profile = P384Sha384;
	certify_key_input->format = FORMAT_X509;
	input.data_size = sizeof(struct dpe_certify_key_i);

	certify_key_output = (struct dpe_certify_key_o *)output.data;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	ret = caliptra_invoke_dpe_command(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_INVOKE_DPE_COMMAND,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x", ret);
	else if (cptra_dpe_response_check(&certify_key_output->rsp_hdr)) {
		LOG_ERR("DPE command failed, magic:0x%08x status:0x%08x profile:0x%08x\n",
			certify_key_output->rsp_hdr.magic, certify_key_output->rsp_hdr.status,
			certify_key_output->rsp_hdr.profile);

	} else {
		LOG_DBG("caliptra_invoke_dpe_command is successful");
		LOG_HEXDUMP_DBG(certify_key_output->context_handle,
				sizeof(certify_key_output->context_handle),
				"context_handle:");
		LOG_HEXDUMP_DBG(certify_key_output->public_key_x,
				sizeof(certify_key_output->public_key_x),
				"public_key_x:");
		LOG_HEXDUMP_DBG(certify_key_output->public_key_y,
				sizeof(certify_key_output->public_key_y),
				"public_key_y:");
		LOG_HEXDUMP_DBG(certify_key_output->cert,
				certify_key_output->cert_size, "cert:");
	}
}

void cptra_test_invoke_dpe_sign(void)
{
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
#endif
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	struct dpe_sign_i *sign_input = NULL;
	struct dpe_sign_o *sign_output = NULL;
	int ret;

	LOG_INF("Test caliptra_invoke_dpe_command...");

	LOG_INF("\tTest Sign...");

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));

	/* Set input */
	sign_input = (struct dpe_sign_i *)input.data;
	sign_input->cmd_hdr.magic = DPE_COMMAND_MAGIC;
	sign_input->cmd_hdr.cmd = SIGN;
	sign_input->cmd_hdr.profile = P384Sha384;
	input.data_size = sizeof(struct dpe_sign_i);

	sign_output = (struct dpe_sign_o *)output.data;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	ret = caliptra_invoke_dpe_command(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_INVOKE_DPE_COMMAND,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x", ret);
	else if (cptra_dpe_response_check(&sign_output->rsp_hdr)) {
		LOG_ERR("DPE command failed, magic:0x%08x status:0x%08x profile:0x%08x\n",
			sign_output->rsp_hdr.magic, sign_output->rsp_hdr.status,
			sign_output->rsp_hdr.profile);

	} else {
		LOG_DBG("caliptra_invoke_dpe_command is successful");
		LOG_HEXDUMP_DBG(sign_output->context_handle,
				sizeof(sign_output->context_handle),
				"context_handle:");
		LOG_HEXDUMP_DBG(sign_output->signature_r,
				sizeof(sign_output->signature_r),
				"signature_r:");
		LOG_HEXDUMP_DBG(sign_output->signature_s,
				sizeof(sign_output->signature_s),
				"signature_s:");
	}
}

void cptra_test_invoke_dpe_rotate_context(uint8_t *context_handle, uint8_t *new_context_handle)
{
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
#endif
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	struct dpe_rotate_context_i *rotate_context_input = NULL;
	struct dpe_new_context_o *rotate_context_output = NULL;
	int ret;

	LOG_INF("Test caliptra_invoke_dpe_command...");

	LOG_INF("\tTest RotateContextHandle...");

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));

	/* Set input */
	rotate_context_input = (struct dpe_rotate_context_i *)input.data;
	rotate_context_input->cmd_hdr.magic = DPE_COMMAND_MAGIC;
	rotate_context_input->cmd_hdr.cmd = ROTATE_CONTEXT_HANDLE;
	rotate_context_input->cmd_hdr.profile = P384Sha384;
	memcpy(rotate_context_input->handle, context_handle, sizeof(rotate_context_input->handle));
	input.data_size = sizeof(struct dpe_rotate_context_i);

	rotate_context_output = (struct dpe_new_context_o *)output.data;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	ret = caliptra_invoke_dpe_command(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_INVOKE_DPE_COMMAND,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x", ret);
	else if (cptra_dpe_response_check(&rotate_context_output->rsp_hdr)) {
		LOG_ERR("DPE command failed, magic:0x%08x status:0x%08x profile:0x%08x\n",
			rotate_context_output->rsp_hdr.magic, rotate_context_output->rsp_hdr.status,
			rotate_context_output->rsp_hdr.profile);

	} else {
		LOG_DBG("caliptra_invoke_dpe_command is successful");
		LOG_HEXDUMP_DBG(rotate_context_output->context_handle,
				sizeof(rotate_context_output->context_handle),
				"context_handle:");
		memcpy(new_context_handle, rotate_context_output->context_handle,
		       sizeof(rotate_context_output->context_handle));
	}
}

void cptra_test_invoke_dpe_destroy_context(uint8_t *context_handle)
{
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
#endif
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	struct dpe_destroy_context_i *destroy_context_input = NULL;
	struct dpe_destroy_context_o  *destroy_context_output = NULL;
	int ret;

	LOG_INF("Test caliptra_invoke_dpe_command...");

	LOG_INF("\tTest DestroyContext...");

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));

	/* Set input */
	destroy_context_input = (struct dpe_destroy_context_i *)input.data;
	destroy_context_input->cmd_hdr.magic = DPE_COMMAND_MAGIC;
	destroy_context_input->cmd_hdr.cmd = DESTROY_CONTEXT;
	destroy_context_input->cmd_hdr.profile = P384Sha384;
	memcpy(destroy_context_input->handle, context_handle,
	       sizeof(destroy_context_input->handle));
	input.data_size = sizeof(struct dpe_destroy_context_i);

	destroy_context_output = (struct dpe_destroy_context_o *)output.data;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	ret = caliptra_invoke_dpe_command(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_INVOKE_DPE_COMMAND,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x", ret);
	else if (cptra_dpe_response_check(&destroy_context_output->rsp_hdr)) {
		LOG_ERR("DPE command failed, magic:0x%08x status:0x%08x profile:0x%08x\n",
			destroy_context_output->rsp_hdr.magic,
			destroy_context_output->rsp_hdr.status,
			destroy_context_output->rsp_hdr.profile);

	} else {
		LOG_DBG("caliptra_invoke_dpe_command is successful");
	}
}

static void cptra_test_invoke_dpe_command(void)
{
	uint8_t public_key[97] = {0};
	uint8_t derived_context[16] = {0};
	uint8_t rotated_context[16] = {0};

	/* 0x04 is the prefix for uncompressed public key */
	public_key[0] = 0x04;

	cptra_test_invoke_dpe_get_profile();
	cptra_test_invoke_dpe_get_certificate_chain();

	/* DPE might has initialized */
	/* cptra_test_invoke_dpe_initialize_context(); */

	cptra_test_invoke_dpe_certify_key();
	cptra_test_invoke_dpe_sign();

	cptra_test_invoke_dpe_derive_context(derived_context);
	cptra_test_invoke_dpe_rotate_context(derived_context, rotated_context);
	cptra_test_invoke_dpe_destroy_context(rotated_context);
}

static void cptra_test_disable_attestation(void)
{
	struct cptra_disable_attestation_ia input;
	struct cptra_disable_attestation_oa output;
	int ret;

	LOG_INF("Test caliptra_disable_attestation...");

	memset(&input, 0, sizeof(struct cptra_disable_attestation_ia));
	memset(&output, 0, sizeof(struct cptra_disable_attestation_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_disable_attestation(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_DISABLE_ATTESTATION,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_disable_attestation is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_disable_attestation is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_certify_key_extended(void)
{
	struct cptra_certify_key_extended_ia input;
	struct cptra_certify_key_extended_oa output;
	int ret;

	LOG_INF("Test caliptra_certify_key_extended...");

	memset(&input, 0, sizeof(struct cptra_certify_key_extended_ia));
	memset(&output, 0, sizeof(struct cptra_certify_key_extended_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_certify_key_extended(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_CERTIFY_KEY_EXTENDED,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_certify_key_extended is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_certify_key_extended is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);
	LOG_HEXDUMP_DBG(output.certify_key_resp, sizeof(output.certify_key_resp),
			"certify_key_resp:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_add_subject_alt_name(void)
{
	struct cptra_add_subject_alt_name_ia input;
	struct cptra_add_subject_alt_name_oa output;
	int ret;

	LOG_INF("Test caliptra_add_subject_alt_name...");

	memset(&input, 0, sizeof(struct cptra_add_subject_alt_name_ia));
	memset(&output, 0, sizeof(struct cptra_add_subject_alt_name_oa));

	/* Set input */
	char *dev_info = "abc:def:ghi";

	memcpy(input.dmtf_device_info, dev_info, strlen(dev_info));
	input.dmtf_device_info_size = strlen(dev_info);

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_add_subject_alt_name(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_ADD_SUBJECT_ALT_NAME,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_add_subject_alt_name is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_add_subject_alt_name is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_dpe_get_tagged_tci(void)
{
	struct cptra_dpe_get_tagged_tci_ia input;
	struct cptra_dpe_get_tagged_tci_oa output;
	int ret;

	LOG_INF("Test caliptra_dpe_get_tagged_tci...");

	memset(&input, 0, sizeof(struct cptra_dpe_get_tagged_tci_ia));
	memset(&output, 0, sizeof(struct cptra_dpe_get_tagged_tci_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_dpe_get_tagged_tci(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_DPE_GET_TAGGED_TCI,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_dpe_get_tagged_tci is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_dpe_get_tagged_tci is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);
	LOG_HEXDUMP_DBG(output.tci_cumulative, sizeof(output.tci_cumulative), "tci_cumulative:");
	LOG_HEXDUMP_DBG(output.tci_current, sizeof(output.tci_current), "tci_current:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_dpe_tag_tci(void)
{
	struct cptra_dpe_tag_tci_ia input;
	struct cptra_dpe_tag_tci_oa output;
	int ret;

	LOG_INF("Test caliptra_dpe_tag_tci...");

	memset(&input, 0, sizeof(struct cptra_dpe_tag_tci_ia));
	memset(&output, 0, sizeof(struct cptra_dpe_tag_tci_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_dpe_tag_tci(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_DPE_TAG_TCI, (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL, (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_dpe_tag_tci is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_dpe_tag_tci is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_increment_pcr_reset_counter(void)
{
	struct cptra_increment_pcr_reset_counter_ia input;
	struct cptra_increment_pcr_reset_counter_oa output;
	int ret;

	LOG_INF("Test caliptra_increment_pcr_reset_counter...");

	memset(&input, 0, sizeof(struct cptra_increment_pcr_reset_counter_ia));
	memset(&output, 0, sizeof(struct cptra_increment_pcr_reset_counter_oa));

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_increment_pcr_reset_counter(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_INCREMENT_PCR_RESET_COUNTER, (uint32_t *)&input,
				 sizeof(input), CPTRA_IPC_RX_TYPE_EXTERNAL, (uint32_t *)&output,
				 sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_increment_pcr_reset_counter is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_increment_pcr_reset_counter is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_quote_pcrs(void)
{
	struct cptra_quote_pcrs_ia input;
	struct cptra_quote_pcrs_oa output;
	int ret;

	LOG_INF("Test caliptra_quote_pcrs...");

	memset(&input, 0, sizeof(struct cptra_quote_pcrs_ia));
	memset(&output, 0, sizeof(struct cptra_quote_pcrs_oa));
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_quote_pcrs(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_QUOTE_PCRS, (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL, (uint32_t *)&output, sizeof(output));
#endif
	if (ret) {
		LOG_ERR("caliptra_quote_pcrs is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_quote_pcrs is successful");

	LOG_DBG("output: chksum=0x%x, fips_status=0x%x",
		output.chksum, output.fips_status);
	LOG_HEXDUMP_DBG(output.PCRs[31], sizeof(output.PCRs[31]), "PCRs[31]:");
	LOG_HEXDUMP_DBG(output.nonce, sizeof(output.nonce), "nonce:");
	LOG_HEXDUMP_DBG(output.digest, sizeof(output.digest), "digest:");
	LOG_HEXDUMP_DBG(output.reset_ctrs, sizeof(output.reset_ctrs), "reset_ctrs:");
	LOG_HEXDUMP_DBG(output.signature_r, sizeof(output.signature_r), "signature_r:");
	LOG_HEXDUMP_DBG(output.signature_s, sizeof(output.signature_s), "signature_s:");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_extend_pcr(void)
{
	struct cptra_extend_pcr_ia input;
	struct cptra_extend_pcr_oa output;
	int ret;

	LOG_INF("Test caliptra_extend_pcr...");

	memset(&input, 0, sizeof(struct cptra_extend_pcr_ia));
	memset(&output, 0, sizeof(struct cptra_extend_pcr_oa));

	/* TODO: customize input data by application */
	input.index = 31;
	input.value[0] = 0x28;
	input.value[1] = 0x01;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_extend_pcr(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_EXTEND_PCR, (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL, (uint32_t *)&output, sizeof(output));
#endif
	if (ret) {
		LOG_ERR("caliptra_extend_pcr is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_extend_pcr is successful");

	cptra_test_quote_pcrs();

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static void cptra_test_stash_measurement(void)
{
	struct cptra_stash_measurement_ia input;
	struct cptra_stash_measurement_oa output;
	int ret;

	LOG_INF("Test caliptra_stash_measurement...");

	memset(&input, 0, sizeof(struct cptra_stash_measurement_ia));
	memset(&output, 0, sizeof(struct cptra_stash_measurement_oa));

	/* TODO: customize input data by application */
	memcpy(input.metadata, "META", 4);

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_stash_measurement(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_STASH_MEASUREMENT, (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL, (uint32_t *)&output, sizeof(output));
#endif
	if (ret) {
		LOG_ERR("caliptra_stash_measurement is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_stash_measurement is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x, dpe_result:0x%x",
		output.chksum, output.fips_status, output.dpe_result);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

static int cptra_sha384(uint8_t *msg, int msg_size, uint8_t *output, int output_size)
{
	uint8_t *p8_bmcu_out = (uint8_t *)IPC_CHANNEL_1_BOOTMCU_OUT_ADDR;
	uint8_t *p8_bmcu_in = (uint8_t *)IPC_CHANNEL_1_BOOTMCU_IN_ADDR;
	uint8_t *p8_ssp_in = (uint8_t *)IPC_CHANNEL_1_SSP_IN_ADDR;
	int ipccmd = CPTRA_IPCCMD_SHA384_DIGEST;
	struct cptra_hash_ctx ctx;
	uint32_t data[2];
	int ret;

	LOG_DBG("%s", __func__);

	/* Prepare tx data to bootmcu */
	data[0] = IPC_CHANNEL_1_BOOTMCU_IN_ADDR;
	data[1] = IPC_CHANNEL_1_BOOTMCU_OUT_ADDR;

	ctx.algo = CRYPTO_HASH_ALGO_SHA384;
	ctx.in_len = msg_size;
	ctx.in_buf = p8_bmcu_in + sizeof(struct cptra_hash_ctx);
	ctx.out_len = output_size;
	ctx.out_buf = p8_bmcu_out;

	/* Copy input data structure into shared memory */
	memcpy(p8_ssp_in, &ctx, sizeof(struct cptra_hash_ctx));
	p8_ssp_in += sizeof(struct cptra_hash_ctx);

	/* Copy input data into shared memory */
	memcpy(p8_ssp_in, msg, msg_size);

	ret = cptra_ipc_trigger(ipccmd, data, sizeof(data));
	if (ret) {
		LOG_ERR("cptra_ipc_trigger:0x%x is failure, ret:0x%x", ipccmd, ret);
		goto end;
	} else
		LOG_DBG("cptra_ipc_trigger:0x%x is successful", ipccmd);

	cptra_ipc_receive(CPTRA_IPC_RX_TYPE_EXTERNAL, output, output_size);

	return 0;

end:
	return ret;
}

__attribute__((unused)) static int cptra_test_ecdsa_verify(void)
{
	uint8_t *p8_bmcu_in = (uint8_t *)IPC_CHANNEL_1_BOOTMCU_IN_ADDR;
	uint8_t *p8_ssp_in = (uint8_t *)IPC_CHANNEL_1_SSP_IN_ADDR;
	int ipccmd = CPTRA_IPCCMD_ECDSA384_SIGNATURE_VERIFY;
	const struct ecdsa_testvec *tv = secp384r1_tv;
	int tv_size = ARRAY_SIZE(secp384r1_tv);
	struct cptra_ecdsa_ctx ctx;
	uint8_t digest[64];
	uint32_t data[2];
	int ret;

	LOG_INF("%s: Start...", __func__);
	for (int i = 0; i < tv_size; i++) {
		LOG_DBG("Test vector %d", i);

		/* Doing hash first for Caliptra secure IP case */
		cptra_sha384((uint8_t *)tv[i].raw, tv[i].raw_size, digest, 48);
		if (!memcmp(digest, tv[i].m, tv[i].m_size))
			LOG_DBG("digest compare - PASS");
		else {
			LOG_ERR("digest compare - FAIL");
			return -1;
		}

		/* Prepare tx data to bootmcu */
		p8_bmcu_in = (uint8_t *)IPC_CHANNEL_1_BOOTMCU_IN_ADDR;
		p8_bmcu_in += sizeof(struct cptra_ecdsa_ctx);
		ctx.qx = p8_bmcu_in;
		p8_bmcu_in += 48;
		ctx.qy = p8_bmcu_in;
		p8_bmcu_in += 48;
		ctx.r = p8_bmcu_in;
		p8_bmcu_in += 48;
		ctx.s = p8_bmcu_in;
		p8_bmcu_in += 48;
		ctx.m = p8_bmcu_in;
		p8_bmcu_in += tv[i].m_size;
		ctx.qx_len = 48;
		ctx.qy_len = 48;
		ctx.r_len = 48;
		ctx.s_len = 48;
		ctx.m_len = tv[i].m_size;

		p8_ssp_in = (uint8_t *)IPC_CHANNEL_1_SSP_IN_ADDR;
		memcpy(p8_ssp_in, &ctx, sizeof(struct cptra_ecdsa_ctx));
		p8_ssp_in += sizeof(struct cptra_ecdsa_ctx);
		memcpy(p8_ssp_in, tv[i].qx, 48);
		p8_ssp_in += 48;
		memcpy(p8_ssp_in, tv[i].qy, 48);
		p8_ssp_in += 48;
		memcpy(p8_ssp_in, tv[i].r, 48);
		p8_ssp_in += 48;
		memcpy(p8_ssp_in, tv[i].s, 48);
		p8_ssp_in += 48;
		memcpy(p8_ssp_in, tv[i].m, tv[i].m_size);
		p8_ssp_in += tv[i].m_size;

		data[0] = IPC_CHANNEL_1_BOOTMCU_IN_ADDR;
		data[1] = IPC_CHANNEL_1_BOOTMCU_OUT_ADDR;

		ret = cptra_ipc_trigger(ipccmd, data, sizeof(data));
		if (ret) {
			LOG_ERR("cptra_ipc_trigger:0x%x is failure, ret:0x%x", ipccmd, ret);
			break;
		}

		LOG_DBG("cptra_ipc_trigger:%x is successful", ipccmd);

		cptra_ipc_receive(CPTRA_IPC_RX_TYPE_INTERNAL, &ret, sizeof(ret));
		if (ret && !tv[i].result)
			LOG_DBG(" result expected (failed), Pass");
		else if (ret == 0 && tv[i].result)
			LOG_DBG(" result expected (pass), Pass");
		else {
			LOG_ERR(" result unexpected (ret=%d), Failed", ret);
			return -1;
		}
	}

	LOG_INF("%s: Pass", __func__);

	return 0;
}

__attribute__((unused)) static int cptra_test_lms_verify(void)
{
	uint8_t *p8_bmcu_in = (uint8_t *)IPC_CHANNEL_1_BOOTMCU_IN_ADDR;
	uint8_t *p8_ssp_in = (uint8_t *)IPC_CHANNEL_1_SSP_IN_ADDR;
	int ipccmd = CPTRA_IPCCMD_LMS_SIGNATURE_VERIFY;
	const struct lms_testvec *tv = lms_tv;
	int tv_size = ARRAY_SIZE(lms_tv);
	struct cptra_lms_ctx ctx;
	uint8_t digest[64];
	uint32_t data[2];
	int ret;

	LOG_INF("%s: Start...", __func__);
	for (int i = 0; i < tv_size; i++) {
		LOG_DBG("Test vector %d", i);

		/* Doing hash first for Caliptra secure IP case */
		cptra_sha384((uint8_t *)tv[i].raw, tv[i].raw_size, digest, 48);

		/* Prepare tx data to bootmcu */
		p8_bmcu_in = (uint8_t *)IPC_CHANNEL_1_BOOTMCU_IN_ADDR;
		p8_bmcu_in += sizeof(struct cptra_lms_ctx);
		ctx.pub_key_id = p8_bmcu_in;
		p8_bmcu_in += tv[i].pub_key_id_len;
		ctx.pub_key_digest = p8_bmcu_in;
		p8_bmcu_in += tv[i].pub_key_digest_len;
		ctx.sig_ots = p8_bmcu_in;
		p8_bmcu_in += tv[i].sig_ots_len;
		ctx.sig_tree_path = p8_bmcu_in;
		p8_bmcu_in += tv[i].sig_tree_path_len;

		ctx.pub_key_tree_type = tv[i].pub_key_tree_type;
		ctx.pub_key_ots_type = tv[i].pub_key_ots_type;
		ctx.pub_key_id_len = tv[i].pub_key_id_len;
		ctx.pub_key_digest_len = tv[i].pub_key_digest_len;
		ctx.sig_q = tv[i].sig_q;
		ctx.sig_ots_len = tv[i].sig_ots_len;
		ctx.sig_tree_type = tv[i].sig_tree_type;
		ctx.sig_tree_path_len = tv[i].sig_tree_path_len;

		p8_ssp_in = (uint8_t *)IPC_CHANNEL_1_SSP_IN_ADDR;
		memcpy(p8_ssp_in, &ctx, sizeof(struct cptra_lms_ctx));
		p8_ssp_in += sizeof(struct cptra_lms_ctx);
		memcpy(p8_ssp_in, tv[i].pub_key_id, tv[i].pub_key_id_len);
		p8_ssp_in += tv[i].pub_key_id_len;
		memcpy(p8_ssp_in, tv[i].pub_key_digest, tv[i].pub_key_digest_len);
		p8_ssp_in += tv[i].pub_key_digest_len;
		memcpy(p8_ssp_in, tv[i].sig_ots, tv[i].sig_ots_len);
		p8_ssp_in += tv[i].sig_ots_len;
		memcpy(p8_ssp_in, tv[i].sig_tree_path, tv[i].sig_tree_path_len);
		p8_ssp_in += tv[i].sig_tree_path_len;

		data[0] = IPC_CHANNEL_1_BOOTMCU_IN_ADDR;
		data[1] = IPC_CHANNEL_1_BOOTMCU_OUT_ADDR;

		ret = cptra_ipc_trigger(ipccmd, data, sizeof(data));
		if (ret) {
			LOG_ERR("cptra_ipc_trigger:0x%x is failure, ret:0x%x", ipccmd, ret);
			break;
		}

		LOG_DBG("cptra_ipc_trigger:%x is successful", ipccmd);

		cptra_ipc_receive(CPTRA_IPC_RX_TYPE_INTERNAL, &ret, sizeof(ret));
		if (ret && !tv[i].result)
			LOG_DBG(" result expected (failed), Pass");
		else if (ret == 0 && tv[i].result)
			LOG_DBG(" result expected (pass), Pass");
		else {
			LOG_ERR(" result unexpected (ret=%d), Failed", ret);
			return -1;
		}
	}

	LOG_INF("%s: Pass", __func__);

	return 0;
}

__attribute__((unused)) static int cptra_test_sha384(void)
{
	const struct hash_testvec *tv = sha384_tv_template;
	int tv_size = ARRAY_SIZE(sha384_tv_template);
	uint8_t digest[64];

	LOG_INF("%s: Start...", __func__);
	for (int i = 0; i < tv_size; i++) {
		LOG_DBG("Test vector %d", i);

		cptra_sha384((uint8_t *)tv[i].plaintext, tv[i].psize, digest, 48);
		if (!memcmp(digest, tv[i].digest, 48))
			LOG_DBG("digest compare - PASS");
		else {
			LOG_ERR("digest compare - FAIL");
			return -1;
		}
	}

	LOG_INF("%s: Pass", __func__);

	return 0;
}

__attribute__((unused)) static void cptra_test_fw_upload(void)
{
	int ret;

	LOG_INF("Test caliptra_fw_upload...");

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_UPDATE_DRV_NAME);

	ret = caliptra_fw_upload(dev, (uint8_t *)CPTRA_FW_ADDR, CPTRA_FW_SIZE);
#elif CONFIG_CPTRA_SAMPLE_SSP
	uint8_t *p8_bmcu_in = (uint8_t *)IPC_CHANNEL_1_BOOTMCU_IN_ADDR;
	uint8_t *p8_ssp_in = (uint8_t *)IPC_CHANNEL_1_SSP_IN_ADDR;
	int ipccmd = CPTRA_IPCCMD_CALIPTRA_FW_LOAD;
	const struct device *flash_dev;
	uint32_t data[2];

	/* Prepare tx data to bootmcu */
	data[0] = (uint32_t)p8_bmcu_in;
	data[1] = (uint32_t)CPTRA_FW_SIZE;

	flash_dev = device_get_binding(FMC_DEV_NAME);
	if (!flash_dev) {
		LOG_ERR("Failed to get flash device");
		goto end;
	}

	/* Copy input data into shared memory */
	ret = flash_read(flash_dev, 0x0, p8_ssp_in, CPTRA_FW_SIZE);
	if (ret)
		LOG_ERR("fail to read flash, ret:0x%x", ret);

	ret = cptra_ipc_trigger(ipccmd, data, sizeof(data));
	if (ret) {
		LOG_ERR("cptra_ipc_trigger:0x%x is failure, ret:0x%x", ipccmd, ret);
		goto end;
	} else
		LOG_DBG("cptra_ipc_trigger:0x%x is successful", ipccmd);

	cptra_ipc_receive(CPTRA_IPC_RX_TYPE_INTERNAL, &ret, sizeof(ret));
	if (ret) {
		LOG_ERR("cptra_ipc_receive:0x%x is failure, ret:0x%x", ipccmd, ret);
		goto end;
	} else
		LOG_DBG("cptra_ipc_receive:0x%x is successful", ipccmd);
#endif
	if (ret) {
		LOG_ERR("caliptra_fw_upload is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_fw_upload is successful");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

__attribute__((unused)) static void cptra_test_set_auth_manifest(void)
{
	struct cptra_set_auth_manifest_ia input;
	struct cptra_set_auth_manifest_oa output;
	uint8_t hash[48];
	int range;
	int ret;

	LOG_INF("Test caliptra_set_auth_manifest...");

	memset(&input, 0, sizeof(struct cptra_set_auth_manifest_ia));
	memset(&output, 0, sizeof(struct cptra_set_auth_manifest_oa));

	/* Set input data */
	input.manifest_size = sizeof(struct cptra_manifest_preamble) +
			      sizeof(uint32_t) +
			      sizeof(struct cptra_manifest_ime);
	input.preamble.manifest_marker = CPTRA_MBCMD_SET_AUTH_MANIFEST;
	input.preamble.preamble_size = sizeof(struct cptra_manifest_preamble);
	input.preamble.manifest_version = 1;
	input.preamble.manifest_flags = 0;
	input.metadata_entry_entry_count = 1;

	LOG_DBG("manifest_size:0x%x", input.manifest_size);
	LOG_DBG("manifest_marker:0x%x", input.preamble.manifest_marker);
	LOG_DBG("preamble_size:0x%x", input.preamble.preamble_size);
	LOG_DBG("metadata_entry_entry_count:0x%x", input.metadata_entry_entry_count);

	memcpy(input.preamble.manifest_owner_ecc384_key, own_manifest_tv[0].raw, 48);
	memcpy(input.preamble.manifest_owner_ecc384_key + 12, own_manifest_tv[0].raw + 48, 48);

	/* Hash the manifest_version and manifest_flags */
	LOG_DBG("Hashing manifest_version and manifest_flags");
	memset(hash, 0, sizeof(hash));
	range = sizeof(input.preamble.manifest_version) + sizeof(input.preamble.manifest_flags) +
		sizeof(input.preamble.manifest_vendor_ecc384_key) +
		sizeof(input.preamble.manifest_vendor_lms_key);
#if defined(CONFIG_MBEDTLS)
	ret = mbedtls_sha512((const uint8_t *)&input.preamble.manifest_version, range, hash, 1);
	if (ret) {
		LOG_ERR("Failed to hash message, ret: %d", ret);
		goto end;
	}
#endif
	LOG_HEXDUMP_DBG(hash, sizeof(hash), "Hash:");
	ret = memcmp(hash, vnd_manifest_tv[0].m, 48);
	if (ret) {
		LOG_ERR("Hash compare - FAIL");
		goto end;
	} else
		LOG_DBG("Hash compare - PASS");

	ret = memcmp(&input.preamble.manifest_version, vnd_manifest_tv[0].raw,
		     range);
	if (ret) {
		LOG_ERR("Raw compare - FAIL");
		goto end;
	} else
		LOG_DBG("Raw compare - PASS");

	LOG_DBG("Populate manifest_vendor_ecc384_sig");
	/* Copy r and s into manifest_vendor_ecc384_sig */
	memcpy(input.preamble.manifest_vendor_ecc384_sig, vnd_manifest_tv[0].r, 48);
	memcpy(input.preamble.manifest_vendor_ecc384_sig + 12, vnd_manifest_tv[0].s, 48);

	/* Convert r to big-endian per 4-byte unit */
	for (int i = 0; i < 48; i += 4) {
		uint32_t *data = (uint32_t *)(input.preamble.manifest_vendor_ecc384_sig + i / 4);
		*data = __builtin_bswap32(*data);
	}

	/* Convert s to big-endian per 4-byte unit */
	for (int i = 0; i < 48; i += 4) {
		uint32_t *data =
			(uint32_t *)(input.preamble.manifest_vendor_ecc384_sig + 12 + i / 4);
		*data = __builtin_bswap32(*data);
	}

	LOG_DBG("manifest_vendor_ecc384_sig addr: 0x%x 0x%x",
		(uint32_t)input.preamble.manifest_vendor_ecc384_sig,
		(uint32_t)(input.preamble.manifest_vendor_ecc384_sig + 12));
	LOG_HEXDUMP_DBG(input.preamble.manifest_vendor_ecc384_sig, 48, "r:");
	LOG_HEXDUMP_DBG(input.preamble.manifest_vendor_ecc384_sig + 12, 48, "s:");

	LOG_DBG("Verify & Populate vendor signature successfully");

	LOG_DBG("Hashing owner pub keys");
	memset(hash, 0, sizeof(hash));
	range = sizeof(input.preamble.manifest_owner_ecc384_key) +
		sizeof(input.preamble.manifest_owner_lms_key);
#if defined(CONFIG_MBEDTLS)
	ret = mbedtls_sha512((const uint8_t *)&input.preamble.manifest_owner_ecc384_key, range,
			     hash, 1);
	if (ret) {
		LOG_ERR("Failed to hash message, ret: %d", ret);
		goto end;
	}
#endif
	LOG_HEXDUMP_DBG(hash, sizeof(hash), "Hash:");
	ret = memcmp(hash, own_manifest_tv[0].m, 48);
	if (ret) {
		LOG_ERR("Hash compare - FAIL");
		goto end;
	} else
		LOG_DBG("Hash compare - PASS");

	ret = memcmp(&input.preamble.manifest_owner_ecc384_key, own_manifest_tv[0].raw,
		     range);
	if (ret) {
		LOG_ERR("Raw compare - FAIL");
		goto end;
	} else
		LOG_DBG("Raw compare - PASS");

	LOG_DBG("Populate manifest_owner_ecc384_sig");
	/* Copy r and s into manifest_owner_ecc384_sig */
	memcpy(input.preamble.manifest_owner_ecc384_sig, own_manifest_tv[0].r, 48);
	memcpy(input.preamble.manifest_owner_ecc384_sig + 12, own_manifest_tv[0].s, 48);

	/* Convert r to big-endian per 4-byte unit */
	for (int i = 0; i < 48; i += 4) {
		uint32_t *data = (uint32_t *)(input.preamble.manifest_owner_ecc384_sig + i / 4);
		*data = __builtin_bswap32(*data);
	}

	/* Convert s to big-endian per 4-byte unit */
	for (int i = 0; i < 48; i += 4) {
		uint32_t *data =
			(uint32_t *)(input.preamble.manifest_owner_ecc384_sig + 12 + i / 4);
		*data = __builtin_bswap32(*data);
	}

	LOG_DBG("manifest_owner_ecc384_sig addr: 0x%x 0x%x",
		(uint32_t)input.preamble.manifest_owner_ecc384_sig,
		(uint32_t)(input.preamble.manifest_owner_ecc384_sig + 12));
	LOG_HEXDUMP_DBG(input.preamble.manifest_owner_ecc384_sig, 48, "r:");
	LOG_HEXDUMP_DBG(input.preamble.manifest_owner_ecc384_sig + 12, 48, "s:");

	LOG_DBG("Verify & Populate owner signature successfully");

	LOG_DBG("Hashing metadata entries");
	memset(hash, 0, sizeof(hash));
	range = sizeof(input.metadata_entry_entry_count) +
		sizeof(input.metadata_entries[0]);
#if defined(CONFIG_MBEDTLS)
	ret = mbedtls_sha512((const uint8_t *)&input.metadata_entry_entry_count, range,
			     hash, 1);
	if (ret) {
		LOG_ERR("Failed to hash message, ret: %d", ret);
		goto end;
	}
#endif
	LOG_HEXDUMP_DBG(hash, sizeof(hash), "Hash:");
	ret = memcmp(hash, metadata_manifest_tv[0].m, 48);
	if (ret) {
		LOG_ERR("Hash compare - FAIL");
		goto end;
	} else
		LOG_DBG("Hash compare - PASS");

	ret = memcmp(&input.metadata_entry_entry_count, metadata_manifest_tv[0].raw,
		     range);
	if (ret) {
		LOG_ERR("Raw compare - FAIL");
		goto end;
	} else
		LOG_DBG("Raw compare - PASS");

	LOG_DBG("Populate metadata_owner_ecc384_sig");
	/* Copy r and s into metadata_owner_ecc384_sig */
	memcpy(input.preamble.metadata_owner_ecc384_sig,
	       metadata_manifest_tv[0].r, 48);
	memcpy(input.preamble.metadata_owner_ecc384_sig + 12,
	       metadata_manifest_tv[0].s, 48);

	/* Convert r to big-endian per 4-byte unit */
	for (int i = 0; i < 48; i += 4) {
		uint32_t *data = (uint32_t *)(input.preamble.metadata_owner_ecc384_sig + i / 4);
		*data = __builtin_bswap32(*data);
	}

	/* Convert s to big-endian per 4-byte unit */
	for (int i = 0; i < 48; i += 4) {
		uint32_t *data =
			(uint32_t *)(input.preamble.metadata_owner_ecc384_sig + 12 + i / 4);
		*data = __builtin_bswap32(*data);
	}

	LOG_DBG("metadata_owner_ecc384_sig addr: 0x%x 0x%x",
		(uint32_t)input.preamble.metadata_owner_ecc384_sig,
		(uint32_t)(input.preamble.metadata_owner_ecc384_sig + 12));
	LOG_HEXDUMP_DBG(input.preamble.metadata_owner_ecc384_sig, 48, "r:");
	LOG_HEXDUMP_DBG(input.preamble.metadata_owner_ecc384_sig + 12, 48, "s:");
	LOG_DBG("Verify & Populate metadata signature successfully");

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);

	ret = caliptra_set_auth_manifest(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_SET_AUTH_MANIFEST,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_set_auth_manifest is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_set_auth_manifest is successful");

	LOG_DBG("output: chksum:0x%x, fips_status:0x%x",
		output.chksum, output.fips_status);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

__attribute__((unused)) static void cptra_test_authorize_and_stash(void)
{
	struct cptra_authorize_and_stash_ia input;
	struct cptra_authorize_and_stash_oa output;
	int ret;

	LOG_INF("Test caliptra_authorize_and_stash...");

	memset(&input, 0, sizeof(struct cptra_authorize_and_stash_ia));
	memset(&output, 0, sizeof(struct cptra_authorize_and_stash_oa));

	/* Set input data */
	input.source = InRequest;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);

	ret = caliptra_authorize_and_stash(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_AUTHORIZE_AND_STASH,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_authorize_and_stash is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_authorize_and_stash is successful");

	LOG_DBG("output: chksum=0x%x, fips_status=0x%x",
		output.chksum, output.fips_status);
	LOG_DBG("auth_req_result: 0x%x", output.auth_req_result);

	if (output.auth_req_result != AUTHORIZE_IMAGE) {
		LOG_ERR("authorize image failed");
		goto end;
	}

	/* Set wrong measurement */
	*input.measurement = 0x12;

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	dev = device_get_binding(CPTRA_MISC_DRV_NAME);

	ret = caliptra_authorize_and_stash(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_AUTHORIZE_AND_STASH,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif
	if (ret) {
		LOG_ERR("caliptra_authorize_and_stash is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_authorize_and_stash is successful");

	LOG_DBG("output: chksum=0x%x, fips_status=0x%x",
		output.chksum, output.fips_status);
	LOG_DBG("auth_req_result: 0x%x", output.auth_req_result);

	if (output.auth_req_result != IMAGE_HASH_MISMATCH) {
		LOG_ERR("authorize image failed, image hash should be mismatch");
		goto end;
	}

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

__attribute__((unused)) static void cptra_test_sign_with_exported_ecdsa(void)
{
	struct cptra_sign_with_exported_ecdsa_ia input;
	struct cptra_sign_with_exported_ecdsa_oa output;
	int ret;

	LOG_INF("Test caliptra_sign_with_exported_ecdsa...");

	memset(&input, 0, sizeof(struct cptra_sign_with_exported_ecdsa_ia));
	memset(&output, 0, sizeof(struct cptra_sign_with_exported_ecdsa_oa));

	/* Set input data */
	memcpy(input.exported_cdi_handle, "test", 4);
	memcpy(input.tbs, "test_tbs", 8);

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_sign_with_exported_ecdsa(dev, &input, &output);
	if (ret) {
		LOG_ERR("caliptra_sign_with_exported_ecdsa failed, ret:0x%x", ret);
		return;
	}
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_SIGN_WITH_EXPORTED_ECDSA, (uint32_t *)&input,
				 sizeof(input), CPTRA_IPC_RX_TYPE_EXTERNAL, (uint32_t *)&output,
				 sizeof(output));
#endif
	LOG_INF("Derived Public Key X:");
	LOG_HEXDUMP_INF(output.derived_pubkey_x, sizeof(output.derived_pubkey_x), "X:");
	LOG_INF("Derived Public Key Y:");
	LOG_HEXDUMP_INF(output.derived_pubkey_y, sizeof(output.derived_pubkey_y), "Y:");
	LOG_INF("Signature R:");
	LOG_HEXDUMP_INF(output.signature_r, sizeof(output.signature_r), "R:");
	LOG_INF("Signature S:");
	LOG_HEXDUMP_INF(output.signature_s, sizeof(output.signature_s), "S:");

	if (ret) {
		LOG_ERR("caliptra_sign_with_exported_ecdsa is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_sign_with_exported_ecdsa is successful");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

__attribute__((unused)) static void cptra_test_revoke_exported_cdi_handle(void)
{
	struct cptra_revoke_exported_cdi_handle_ia input;
	struct cptra_revoke_exported_cdi_handle_oa output;
	int ret;

	LOG_INF("Test caliptra_revoke_exported_cdi_handle...");

	memset(&input, 0, sizeof(struct cptra_revoke_exported_cdi_handle_ia));
	memset(&output, 0, sizeof(struct cptra_revoke_exported_cdi_handle_oa));

	/* Set input data */
	memcpy(input.exported_cdi_handle, "test_handle", 11);

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);

	ret = caliptra_revoke_exported_cdi_handle(dev, &input, &output);
#elif CONFIG_CPTRA_SAMPLE_SSP
	ret = cptra_ipc_transfer(CPTRA_IPCCMD_REVOKE_EXPORTED_CDI_HANDLE,
				 (uint32_t *)&input, sizeof(input),
				 CPTRA_IPC_RX_TYPE_EXTERNAL,
				 (uint32_t *)&output, sizeof(output));
#endif

	if (ret) {
		LOG_ERR("caliptra_revoke_exported_cdi_handle is failure, ret:0x%x", ret);
		goto end;
	} else
		LOG_DBG("caliptra_revoke_exported_cdi_handle is successful");

	LOG_DBG("output: chksum=0x%x, fips_status=0x%x",
		output.chksum, output.fips_status);

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
}

#if CONFIG_CPTRA_SAMPLE_BOOTMCU
int cptra_test(void)
{
	cptra_test_fw_upload();
	cptra_test_stash_measurement();
	cptra_test_quote_pcrs();
	cptra_test_extend_pcr();
	cptra_test_increment_pcr_reset_counter();
	cptra_test_dpe_tag_tci();
	cptra_test_dpe_get_tagged_tci();
	cptra_test_add_subject_alt_name();
	cptra_test_certify_key_extended();
	cptra_test_invoke_dpe_command();
	cptra_test_disable_attestation();
	cptra_test_get_idev_cert();
	/* cptra_test_populate_idev_cert(); */
	cptra_test_get_idev_info();
	cptra_test_get_ldev_cert();
	cptra_test_get_fmc_alias_cert();
	cptra_test_get_rt_alias_cert();
	cptra_test_fw_info();
	cptra_test_capabilities();
	cptra_test_version();
	/* cptra_test_shutdown(); */

	/* Caliptra 1.2 new mailbox commands */
	/*
	 * cptra_test_set_auth_manifest();
	 * cptra_test_authorize_and_stash();
	 * cptra_test_get_idevid_csr();
	 * cptra_test_sign_with_exported_ecdsa();
	 * cptra_test_revoke_exported_cdi_handle();
	 */

	return 0;
}
#elif CONFIG_CPTRA_SAMPLE_SSP
static int cmd_cptra(const struct shell *shell, size_t argc, char **argv)
{
	/* cptra: test update */
	cptra_test_fw_upload();

	/* cptra: test crypto */
	cptra_test_sha384();
	cptra_test_ecdsa_verify();
	cptra_test_lms_verify();

	/* cptra: test dice */
	cptra_test_stash_measurement();
	cptra_test_quote_pcrs();
	cptra_test_extend_pcr();
	cptra_test_increment_pcr_reset_counter();
	cptra_test_dpe_tag_tci();
	cptra_test_dpe_get_tagged_tci();
	cptra_test_add_subject_alt_name();
	cptra_test_certify_key_extended();
	cptra_test_invoke_dpe_command();
	cptra_test_disable_attestation();
	cptra_test_get_idev_cert();
	cptra_test_populate_idev_cert();
	cptra_test_get_idev_info();
	cptra_test_get_ldev_cert();
	cptra_test_get_fmc_alias_cert();
	cptra_test_get_rt_alias_cert();

	/* cptra: test misc */
	cptra_test_fw_info();
	cptra_test_capabilities();
	cptra_test_version();

	/* Caliptra 1.2 new mailbox commands */
	/*
	 * cptra_test_set_auth_manifest();
	 * cptra_test_authorize_and_stash();
	 * cptra_test_get_idevid_csr();
	 * cptra_test_sign_with_exported_ecdsa();
	 * cptra_test_revoke_exported_cdi_handle();
	 */

	return 0;
}

SHELL_CMD_REGISTER(cptra, NULL, "Caliptra demo command", cmd_cptra);
#endif
