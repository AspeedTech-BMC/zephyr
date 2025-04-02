/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#include <zephyr/drivers/cptra.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/misc/aspeed/cptra_ipc.h>
#include <zephyr/shell/shell.h>
#include <zephyr/crypto/hash.h>
#include "cptra_sample.h"

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

static void cptra_test_populate_idev_cert(void)
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

static void cptra_test_invoke_dpe_command(void)
{
#if CONFIG_CPTRA_SAMPLE_BOOTMCU
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
#endif
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	int ret;

	LOG_INF("Test caliptra_invoke_dpe_command...");

	LOG_INF("\tTest GetProfile...");

	struct dpe_get_profile_i get_profile_input;

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));
	memset(&get_profile_input, 0, sizeof(struct dpe_get_profile_i));

	/* Set input */
	get_profile_input.cmd_hdr.magic = DPE_COMMAND_MAGIC;
	get_profile_input.cmd_hdr.cmd = GET_PROFILE;
	memcpy(input.data, &get_profile_input, sizeof(struct dpe_get_profile_i));
	input.data_size = sizeof(struct dpe_get_profile_i);

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
	else
		LOG_DBG("caliptra_invoke_dpe_command is successful");

	LOG_INF("\tTest InitializeContext...");

	struct dpe_initialize_context_i initialize_context_input;

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));
	memset(&initialize_context_input, 0, sizeof(struct dpe_initialize_context_i));

	/* Set input */
	initialize_context_input.cmd_hdr.magic = DPE_COMMAND_MAGIC;
	initialize_context_input.cmd_hdr.cmd = INITIALIZE_CONTEXT;
	initialize_context_input.cmd_hdr.profile = P384Sha384;
	memcpy(input.data, &initialize_context_input, sizeof(struct dpe_initialize_context_i));
	input.data_size = sizeof(struct dpe_initialize_context_i);

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
	else
		LOG_DBG("caliptra_invoke_dpe_command is successful");

	LOG_INF("\tTest DeriveContext...");

	struct dpe_derive_context_i derive_context_input;

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));
	memset(&derive_context_input, 0, sizeof(struct dpe_derive_context_i));

	/* Set input */
	derive_context_input.cmd_hdr.magic = DPE_COMMAND_MAGIC;
	derive_context_input.cmd_hdr.cmd = DERIVE_CONTEXT;
	derive_context_input.cmd_hdr.profile = P384Sha384;
	memcpy(input.data, &derive_context_input, sizeof(struct dpe_derive_context_i));
	input.data_size = sizeof(struct dpe_derive_context_i);

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
	else
		LOG_DBG("caliptra_invoke_dpe_command is successful");

	LOG_INF("\tTest CertifyKey...");

	struct dpe_certify_key_i certify_key_input;

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));
	memset(&certify_key_input, 0, sizeof(struct dpe_certify_key_i));

	/* Set input */
	certify_key_input.cmd_hdr.magic = DPE_COMMAND_MAGIC;
	certify_key_input.cmd_hdr.cmd = CERTIFY_KEY;
	certify_key_input.cmd_hdr.profile = P384Sha384;
	memcpy(input.data, &certify_key_input, sizeof(struct dpe_certify_key_i));
	input.data_size = sizeof(struct dpe_certify_key_i);

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
	else
		LOG_DBG("caliptra_invoke_dpe_command is successful");

	LOG_INF("\tTest Sign...");

	struct dpe_sign_i sign_input;

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));
	memset(&sign_input, 0, sizeof(struct dpe_sign_i));

	/* Set input */
	sign_input.cmd_hdr.magic = DPE_COMMAND_MAGIC;
	sign_input.cmd_hdr.cmd = SIGN;
	sign_input.cmd_hdr.profile = P384Sha384;
	memcpy(input.data, &sign_input, sizeof(struct dpe_sign_i));
	input.data_size = sizeof(struct dpe_sign_i);

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
		goto end;
	} else
		LOG_DBG("caliptra_invoke_dpe_command is successful");

	LOG_INF("\tTest RotateContextHandle...");

	struct dpe_rotate_context_handle_i rotate_context_handle_input;

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));
	memset(&rotate_context_handle_input, 0, sizeof(struct dpe_rotate_context_handle_i));

	/* Set input */
	rotate_context_handle_input.cmd_hdr.magic = DPE_COMMAND_MAGIC;
	rotate_context_handle_input.cmd_hdr.cmd = ROTATE_CONTEXT_HANDLE;
	rotate_context_handle_input.cmd_hdr.profile = P384Sha384;
	memcpy(input.data, &rotate_context_handle_input,
	       sizeof(struct dpe_rotate_context_handle_i));
	input.data_size = sizeof(struct dpe_rotate_context_handle_i);

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
		goto end;
	} else
		LOG_DBG("caliptra_invoke_dpe_command is successful");

	LOG_INF("\tTest DestroyContext...");

	struct dpe_destroy_context_i destroy_context_input;

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));
	memset(&destroy_context_input, 0, sizeof(struct dpe_destroy_context_i));

	/* Set input */
	destroy_context_input.cmd_hdr.magic = DPE_COMMAND_MAGIC;
	destroy_context_input.cmd_hdr.cmd = DESTROY_CONTEXT;
	destroy_context_input.cmd_hdr.profile = P384Sha384;
	memcpy(input.data, &destroy_context_input, sizeof(struct dpe_destroy_context_i));
	input.data_size = sizeof(struct dpe_destroy_context_i);

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
		goto end;
	} else
		LOG_DBG("caliptra_invoke_dpe_command is successful");

	LOG_INF("\tTest GetCertificateChain...");

	struct dpe_get_certificate_chain_i get_certificate_chain_input;

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));
	memset(&get_certificate_chain_input, 0, sizeof(struct dpe_get_certificate_chain_i));

	/* Set input */
	get_certificate_chain_input.cmd_hdr.magic = DPE_COMMAND_MAGIC;
	get_certificate_chain_input.cmd_hdr.cmd = GET_CERTIFICATE_CHAIN;
	get_certificate_chain_input.cmd_hdr.profile = P384Sha384;
	memcpy(input.data, &get_certificate_chain_input,
	       sizeof(struct dpe_get_certificate_chain_i));
	input.data_size = sizeof(struct dpe_get_certificate_chain_i);

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
		goto end;
	} else
		LOG_DBG("caliptra_invoke_dpe_command is successful");

	LOG_INF("%s: Pass", __func__);
	return;
end:
	LOG_INF("%s: Failed", __func__);
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
	int ipccmd = CPTRA_IPCCMD_SHA384;
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
	uint32_t data[2];

	/* Prepare tx data to bootmcu */
	data[0] = (uint32_t)p8_bmcu_in;
	data[1] = (uint32_t)CPTRA_FW_SIZE;

	/* Copy input data into shared memory */
	memcpy(SPI_TO_DRAM_BASE_ADDR + p8_ssp_in, (void *)CPTRA_FW_ADDR, CPTRA_FW_SIZE);

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
	cptra_test_populate_idev_cert();
	cptra_test_get_idev_info();
	cptra_test_get_ldev_cert();
	cptra_test_get_fmc_alias_cert();
	cptra_test_get_rt_alias_cert();
	cptra_test_fw_info();
	cptra_test_capabilities();
	cptra_test_version();
	/* cptra_test_shutdown(); */

	return 0;
}
#elif CONFIG_CPTRA_SAMPLE_SSP
static int cmd_cptra(const struct shell *shell, size_t argc, char **argv)
{
	/* cptra: test update */
	/* cptra_test_fw_upload(); */

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

	return 0;
}

SHELL_CMD_REGISTER(cptra, NULL, "Caliptra demo command", cmd_cptra);
#endif
