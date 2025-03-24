/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#include <zephyr/drivers/cptra.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cptra_test, CONFIG_SOC_LOG_LEVEL);

#define CPTRA_UPDATE_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_update))
#define CPTRA_DICE_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_dice))
#define CPTRA_MISC_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_misc))
#define CPTRA_FW_ADDR			0x20000000
#define CPTRA_FW_SIZE			0x20000

__attribute__((unused)) static void cptra_test_shutdown(void)
{
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);
	struct cptra_shutdown_ia input;
	struct cptra_shutdown_oa output;
	int ret;

	LOG_INF("Test caliptra_shutdown...");

	memset(&input, 0, sizeof(struct cptra_shutdown_ia));
	memset(&output, 0, sizeof(struct cptra_shutdown_oa));

	ret = caliptra_shutdown(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_shutdown is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_shutdown is successful\n");
}

__attribute__((unused)) static void cptra_test_self_test_get_results(void)
{
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);
	struct cptra_self_test_get_results_ia input;
	struct cptra_self_test_get_results_oa output;
	int ret;

	LOG_INF("Test caliptra_self_test_get_results...");

	memset(&input, 0, sizeof(struct cptra_self_test_get_results_ia));
	memset(&output, 0, sizeof(struct cptra_self_test_get_results_oa));

	ret = caliptra_self_test_get_results(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_self_test_get_results is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_self_test_get_results is successful\n");
}

__attribute__((unused)) static void cptra_test_self_test_start(void)
{
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);
	struct cptra_self_test_start_ia input;
	struct cptra_self_test_start_oa output;
	int ret;

	LOG_INF("Test caliptra_self_test_start...");

	memset(&input, 0, sizeof(struct cptra_self_test_start_ia));
	memset(&output, 0, sizeof(struct cptra_self_test_start_oa));

	ret = caliptra_self_test_start(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_self_test_start is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_self_test_start is successful\n");
}

static void cptra_test_version(void)
{
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);
	struct cptra_version_ia input;
	struct cptra_version_oa output;
	int ret;

	LOG_INF("Test caliptra_version...");

	memset(&input, 0, sizeof(struct cptra_version_ia));
	memset(&output, 0, sizeof(struct cptra_version_oa));

	ret = caliptra_version(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_version is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_version is successful\n");
}

static void cptra_test_capabilities(void)
{
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);
	struct cptra_capabilities_ia input;
	struct cptra_capabilities_oa output;
	int ret;

	LOG_INF("Test caliptra_capabilities...");

	memset(&input, 0, sizeof(struct cptra_capabilities_ia));
	memset(&output, 0, sizeof(struct cptra_capabilities_oa));

	ret = caliptra_capabilities(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_capabilities is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_capabilities is successful\n");
}

static void cptra_test_fw_info(void)
{
	const struct device *dev = device_get_binding(CPTRA_MISC_DRV_NAME);
	struct cptra_fw_info_ia input;
	struct cptra_fw_info_oa output;
	int ret;

	LOG_INF("Test caliptra_fw_info...");

	memset(&input, 0, sizeof(struct cptra_fw_info_ia));
	memset(&output, 0, sizeof(struct cptra_fw_info_oa));

	ret = caliptra_fw_info(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_fw_info is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_fw_info is successful\n");
}

static void cptra_test_get_rt_alias_cert(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_get_rt_alias_cert_ia input;
	struct cptra_get_rt_alias_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_get_rt_alias_cert...");

	memset(&input, 0, sizeof(struct cptra_get_rt_alias_cert_ia));
	memset(&output, 0, sizeof(struct cptra_get_rt_alias_cert_oa));

	ret = caliptra_get_rt_alias_cert(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_get_rt_alias_cert is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_get_rt_alias_cert is successful\n");
}

static void cptra_test_get_fmc_alias_cert(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_get_fmc_alias_cert_ia input;
	struct cptra_get_fmc_alias_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_get_fmc_alias_cert...");

	memset(&input, 0, sizeof(struct cptra_get_fmc_alias_cert_ia));
	memset(&output, 0, sizeof(struct cptra_get_fmc_alias_cert_oa));

	ret = caliptra_get_fmc_alias_cert(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_get_fmc_alias_cert is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_get_fmc_alias_cert is successful\n");
}

static void cptra_test_get_ldev_cert(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_get_ldev_cert_ia input;
	struct cptra_get_ldev_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_get_ldev_cert...");

	memset(&input, 0, sizeof(struct cptra_get_ldev_cert_ia));
	memset(&output, 0, sizeof(struct cptra_get_ldev_cert_oa));

	ret = caliptra_get_ldev_cert(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_get_ldev_cert is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_get_ldev_cert is successful\n");
}

static void cptra_test_get_idev_info(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_get_idev_info_ia input;
	struct cptra_get_idev_info_oa output;
	int ret;

	LOG_INF("Test caliptra_get_idev_info...");

	memset(&input, 0, sizeof(struct cptra_get_idev_info_ia));
	memset(&output, 0, sizeof(struct cptra_get_idev_info_oa));

	ret = caliptra_get_idev_info(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_get_idev_info is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_get_idev_info is successful\n");
}

static void cptra_test_populate_idev_cert(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_populate_idev_cert_ia input;
	struct cptra_populate_idev_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_populate_idev_cert...");

	memset(&input, 0, sizeof(struct cptra_populate_idev_cert_ia));
	memset(&output, 0, sizeof(struct cptra_populate_idev_cert_oa));

	ret = caliptra_populate_idev_cert(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_populate_idev_cert is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_populate_idev_cert is successful\n");
}

static void cptra_test_get_idev_cert(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_get_idev_cert_ia input;
	struct cptra_get_idev_cert_oa output;
	int ret;

	LOG_INF("Test caliptra_get_idev_cert...");

	memset(&input, 0, sizeof(struct cptra_get_idev_cert_ia));
	memset(&output, 0, sizeof(struct cptra_get_idev_cert_oa));

	ret = caliptra_get_idev_cert(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_get_idev_cert is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_get_idev_cert is successful\n");
}

static void cptra_test_invoke_dpe_command(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_invoke_dpe_command_ia input;
	struct cptra_invoke_dpe_command_oa output;
	int ret;

	LOG_INF("Test caliptra_invoke_dpe_command...");

	LOG_INF("Test GetProfile...");

	struct dpe_get_profile_i get_profile_input;

	memset(&input, 0, sizeof(struct cptra_invoke_dpe_command_ia));
	memset(&output, 0, sizeof(struct cptra_invoke_dpe_command_oa));
	memset(&get_profile_input, 0, sizeof(struct dpe_get_profile_i));

	/* Set input */
	get_profile_input.cmd_hdr.magic = DPE_COMMAND_MAGIC;
	get_profile_input.cmd_hdr.cmd = GET_PROFILE;
	memcpy(input.data, &get_profile_input, sizeof(struct dpe_get_profile_i));
	input.data_size = sizeof(struct dpe_get_profile_i);

	ret = caliptra_invoke_dpe_command(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_invoke_dpe_command is successful\n");

	LOG_INF("Test InitializeContext...");

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

	ret = caliptra_invoke_dpe_command(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_invoke_dpe_command is successful\n");

	LOG_INF("Test DeriveContext...");

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

	ret = caliptra_invoke_dpe_command(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_invoke_dpe_command is successful\n");

	LOG_INF("Test CertifyKey...");

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

	ret = caliptra_invoke_dpe_command(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_invoke_dpe_command is successful\n");

	LOG_INF("Test Sign...");

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

	ret = caliptra_invoke_dpe_command(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_invoke_dpe_command is successful\n");

	LOG_INF("Test RotateContextHandle...");

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

	ret = caliptra_invoke_dpe_command(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_invoke_dpe_command is successful\n");

	LOG_INF("Test DestroyContext...");

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

	ret = caliptra_invoke_dpe_command(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_invoke_dpe_command is successful\n");

	LOG_INF("Test GetCertificateChain...");

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

	ret = caliptra_invoke_dpe_command(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_invoke_dpe_command is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_invoke_dpe_command is successful\n");
}

static void cptra_test_disable_attestation(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_disable_attestation_ia input;
	struct cptra_disable_attestation_oa output;
	int ret;

	LOG_INF("Test caliptra_disable_attestation...");

	memset(&input, 0, sizeof(struct cptra_disable_attestation_ia));
	memset(&output, 0, sizeof(struct cptra_disable_attestation_oa));
	ret = caliptra_disable_attestation(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_disable_attestation is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_disable_attestation is successful\n");
}

static void cptra_test_certify_key_extended(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_certify_key_extended_ia input;
	struct cptra_certify_key_extended_oa output;
	int ret;

	LOG_INF("Test caliptra_certify_key_extended...");

	memset(&input, 0, sizeof(struct cptra_certify_key_extended_ia));
	memset(&output, 0, sizeof(struct cptra_certify_key_extended_oa));
	ret = caliptra_certify_key_extended(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_certify_key_extended is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_certify_key_extended is successful\n");
}

static void cptra_test_add_subject_alt_name(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_add_subject_alt_name_ia input;
	struct cptra_add_subject_alt_name_oa output;
	int ret;

	LOG_INF("Test caliptra_add_subject_alt_name...");

	memset(&input, 0, sizeof(struct cptra_add_subject_alt_name_ia));
	memset(&output, 0, sizeof(struct cptra_add_subject_alt_name_oa));
	ret = caliptra_add_subject_alt_name(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_add_subject_alt_name is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_add_subject_alt_name is successful\n");
}

static void cptra_test_dpe_get_tagged_tci(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_dpe_get_tagged_tci_ia input;
	struct cptra_dpe_get_tagged_tci_oa output;
	int ret;

	LOG_INF("Test caliptra_dpe_get_tagged_tci...");

	memset(&input, 0, sizeof(struct cptra_dpe_get_tagged_tci_ia));
	memset(&output, 0, sizeof(struct cptra_dpe_get_tagged_tci_oa));
	ret = caliptra_dpe_get_tagged_tci(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_dpe_get_tagged_tci is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_dpe_get_tagged_tci is successful\n");
}

static void cptra_test_dpe_tag_tci(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_dpe_tag_tci_ia input;
	struct cptra_dpe_tag_tci_oa output;
	int ret;

	LOG_INF("Test caliptra_dpe_tag_tci...");

	memset(&input, 0, sizeof(struct cptra_dpe_tag_tci_ia));
	memset(&output, 0, sizeof(struct cptra_dpe_tag_tci_oa));
	ret = caliptra_dpe_tag_tci(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_dpe_tag_tci is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_dpe_tag_tci is successful\n");
}

static void cptra_test_increment_pcr_reset_counter(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_increment_pcr_reset_counter_ia input;
	struct cptra_increment_pcr_reset_counter_oa output;
	int ret;

	LOG_INF("Test caliptra_increment_pcr_reset_counter...");

	memset(&input, 0, sizeof(struct cptra_increment_pcr_reset_counter_ia));
	memset(&output, 0, sizeof(struct cptra_increment_pcr_reset_counter_oa));
	ret = caliptra_increment_pcr_reset_counter(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_increment_pcr_reset_counter is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_increment_pcr_reset_counter is successful\n");
}

static void cptra_test_quote_pcrs(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_quote_pcrs_ia input;
	struct cptra_quote_pcrs_oa output;
	int ret;

	LOG_INF("Test caliptra_quote_pcrs...");

	memset(&input, 0, sizeof(struct cptra_quote_pcrs_ia));
	memset(&output, 0, sizeof(struct cptra_quote_pcrs_oa));
	ret = caliptra_quote_pcrs(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_quote_pcrs is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_quote_pcrs is successful\n");
}

static void cptra_test_extend_pcr(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_extend_pcr_ia input;
	struct cptra_extend_pcr_oa output;
	int ret;

	LOG_INF("Test caliptra_extend_pcr...");

	memset(&input, 0, sizeof(struct cptra_extend_pcr_ia));
	memset(&output, 0, sizeof(struct cptra_extend_pcr_oa));

	input.index = 31;
	memcpy(input.value, (uint8_t *)0xbeef, 2);

	ret = caliptra_extend_pcr(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_extend_pcr is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_extend_pcr is successful\n");

	cptra_test_quote_pcrs();
}

static void cptra_test_stash_measurement(void)
{
	const struct device *dev = device_get_binding(CPTRA_DICE_DRV_NAME);
	struct cptra_stash_measurement_ia input;
	struct cptra_stash_measurement_oa output;
	int ret;

	LOG_INF("Test caliptra_stash_measurement...");

	memset(&input, 0, sizeof(struct cptra_stash_measurement_ia));
	memset(&output, 0, sizeof(struct cptra_stash_measurement_oa));
	ret = caliptra_stash_measurement(dev, &input, &output);
	if (ret)
		LOG_ERR("caliptra_stash_measurement is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_stash_measurement is successful\n");
}

static void cptra_test_fw_upload(void)
{
	const struct device *dev = device_get_binding(CPTRA_UPDATE_DRV_NAME);
	int ret;

	LOG_INF("Test caliptra_fw_upload...");
	ret = caliptra_fw_upload(dev, (uint8_t *)CPTRA_FW_ADDR, CPTRA_FW_SIZE);
	if (ret)
		LOG_ERR("caliptra_fw_upload is failure, ret:0x%x\n", ret);
	else
		LOG_INF("caliptra_fw_upload is successful\n");
}

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
