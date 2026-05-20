/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/cptra.h>
#include <zephyr/drivers/misc/aspeed/cptra_ipc.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/ecdsa_structs.h>
#include <zephyr/crypto/ecdsa.h>
#include <zephyr/crypto/lms.h>
#include <zephyr/crypto/lms_structs.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(cptra_ipc, CONFIG_MISC_ASPEED_LOG_LEVEL);

#define CPTRA_ECDSA_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_ecdsa))
#define CPTRA_HASH_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_sha))
#define CPTRA_LMS_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_lms))
#define CPTRA_UPDATE_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_update))
#define CPTRA_DICE_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_dice))
#define CPTRA_MISC_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_cptra_misc))

#define IPC_DEV_SSP_BOOTMCU		"ipc1@400"
#define IPC_DEV_NS_CA35_BOOTMCU		"ipc1@200"
#define IPC_CHANNEL_ID_CPTRA		1

static int cptra_ipc_ecdsa384_verify(const struct device *dev, void *arg1, void *arg2);
static int cptra_ipc_sha384(const struct device *dev, void *arg1, void *arg2);
static int cptra_ipc_sha384_init(const struct device *dev, void *arg1, void *arg2);
static int cptra_ipc_sha384_update(const struct device *dev, void *arg1, void *arg2);
static int cptra_ipc_sha384_final(const struct device *dev, void *arg1, void *arg2);
static int cptra_ipc_lms_verify(const struct device *dev, void *arg1, void *arg2);
static int cptra_ipc_fw_upload(const struct device *dev, void *arg1, void *arg2);

typedef int (*cptra_callback_t)(const struct device *dev, void *arg1, void *arg2);

struct cptra_ipc_callback_tbl {
	const char *dev;
	enum cptra_ipc_cmd cmd;
	cptra_callback_t cptra_cb;
};

static const struct cptra_ipc_callback_tbl cptra_ipc_list[] = {
	{ CPTRA_ECDSA_DRV_NAME, CPTRA_IPCCMD_ECDSA384_SIGNATURE_VERIFY, cptra_ipc_ecdsa384_verify },
	{ CPTRA_HASH_DRV_NAME, CPTRA_IPCCMD_SHA384_DIGEST, (cptra_callback_t)cptra_ipc_sha384 },
	{ CPTRA_HASH_DRV_NAME, CPTRA_IPCCMD_SHA384_INIT, (cptra_callback_t)cptra_ipc_sha384_init },
	{ CPTRA_HASH_DRV_NAME, CPTRA_IPCCMD_SHA384_UPDATE, (cptra_callback_t)cptra_ipc_sha384_update },
	{ CPTRA_HASH_DRV_NAME, CPTRA_IPCCMD_SHA384_FINAL, (cptra_callback_t)cptra_ipc_sha384_final },
	{ CPTRA_LMS_DRV_NAME, CPTRA_IPCCMD_LMS_SIGNATURE_VERIFY, (cptra_callback_t)cptra_ipc_lms_verify },
	{ CPTRA_UPDATE_DRV_NAME, CPTRA_IPCCMD_CALIPTRA_FW_LOAD, (cptra_callback_t)cptra_ipc_fw_upload },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_STASH_MEASUREMENT, (cptra_callback_t)caliptra_stash_measurement },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_QUOTE_PCRS, (cptra_callback_t)caliptra_quote_pcrs },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_GET_IDEV_CERT, (cptra_callback_t)caliptra_get_idev_cert },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_GET_IDEV_INFO, (cptra_callback_t)caliptra_get_idev_info },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_POPULATE_IDEV_CERT, (cptra_callback_t)caliptra_populate_idev_cert },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_GET_LDEV_CERT, (cptra_callback_t)caliptra_get_ldev_cert },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_GET_FMC_ALIAS_CERT, (cptra_callback_t)caliptra_get_fmc_alias_cert },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_GET_RT_ALIAS_CERT, (cptra_callback_t)caliptra_get_rt_alias_cert },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_INVOKE_DPE_COMMAND, (cptra_callback_t)caliptra_invoke_dpe_command },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_DISABLE_ATTESTATION, (cptra_callback_t)caliptra_disable_attestation },
	{ CPTRA_MISC_DRV_NAME, CPTRA_IPCCMD_FW_INFO, (cptra_callback_t)caliptra_fw_info },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_DPE_TAG_TCI, (cptra_callback_t)caliptra_dpe_tag_tci },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_DPE_GET_TAGGED_TCI, (cptra_callback_t)caliptra_dpe_get_tagged_tci },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_INCREMENT_PCR_RESET_COUNTER, (cptra_callback_t)caliptra_increment_pcr_reset_counter },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_EXTEND_PCR, (cptra_callback_t)caliptra_extend_pcr },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_ADD_SUBJECT_ALT_NAME, (cptra_callback_t)caliptra_add_subject_alt_name },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_CERTIFY_KEY_EXTENDED, (cptra_callback_t)caliptra_certify_key_extended },
	{ CPTRA_MISC_DRV_NAME, CPTRA_IPCCMD_FIPS_VERSION, (cptra_callback_t)caliptra_version },
	{ CPTRA_MISC_DRV_NAME, CPTRA_IPCCMD_SHUTDOWN, (cptra_callback_t)caliptra_shutdown },
	{ CPTRA_MISC_DRV_NAME, CPTRA_IPCCMD_CAPABILITIES, (cptra_callback_t)caliptra_capabilities },
	{ CPTRA_MISC_DRV_NAME, CPTRA_IPCCMD_SET_AUTH_MANIFEST, (cptra_callback_t)caliptra_set_auth_manifest },
	{ CPTRA_MISC_DRV_NAME, CPTRA_IPCCMD_AUTHORIZE_AND_STASH, (cptra_callback_t)caliptra_authorize_and_stash },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_GET_FMC_ALIAS_CSR, (cptra_callback_t)caliptra_get_fmc_alias_csr },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_SIGN_WITH_EXPORTED_ECDSA, (cptra_callback_t)caliptra_sign_with_exported_ecdsa },
	{ CPTRA_DICE_DRV_NAME, CPTRA_IPCCMD_REVOKE_EXPORTED_CDI_HANDLE, (cptra_callback_t)caliptra_revoke_exported_cdi_handle },
	{ NULL, 0, NULL }
};

static bool ipc_fw_upload_init;

static int cptra_ipc_fw_upload(const struct device *dev, void *arg1, void *arg2)
{
	uint8_t *buf;
	int size;
	int rc;

	buf = (uint8_t *)arg1;
	size = (int)(uintptr_t)arg2;

	if (size == IPC_SHARE_MEM_SRAM_SIZE) {
		rc = caliptra_fw_upload_init(dev);
		if (rc)
			goto end;

		ipc_fw_upload_init = true;

		rc = caliptra_fw_upload_update(dev, buf, size);

	} else if (size == CPTRA_MBOX_SZ) {
		if (!ipc_fw_upload_init) {
			rc = caliptra_fw_upload_init(dev);
			if (rc)
				goto end;

			rc = caliptra_fw_upload_update(dev, buf, CPTRA_MBOX_SZ);
			if (rc)
				goto end;

			rc = caliptra_fw_upload_final(dev, size);

		} else {
			rc = caliptra_fw_upload_update(dev, buf, IPC_SHARE_MEM_SRAM_SIZE);
			if (rc)
				goto end;

			rc = caliptra_fw_upload_final(dev, size);
			ipc_fw_upload_init = false;
		}

	} else {
		if (!ipc_fw_upload_init) {
			LOG_ERR("Firmware upload not initialized");
			return -EINVAL;
		}

		rc = caliptra_fw_upload_update(dev, buf, IPC_SHARE_MEM_SRAM_SIZE);
	}

end:
	return rc;
}

static int cptra_ipc_lms_verify(const struct device *dev, void *arg1, void *arg2)
{
	struct cptra_lms_ctx *ctx = (struct cptra_lms_ctx *)arg1;
	struct lms_ctx ini;
	struct lms_pkt pkt;
	struct lms_pub_key ek;
	int ret, rc = 0;

	ek.pub_key_tree_type = sys_cpu_to_be32(ctx->pub_key_tree_type);
	ek.pub_key_ots_type = sys_cpu_to_be32(ctx->pub_key_ots_type);
	memcpy(ek.pub_key_id, ctx->pub_key_id, LMS_PUB_KEY_ID_LEN);
	memcpy(ek.pub_key_digest, ctx->pub_key_digest, LMS_PUB_KEY_DGST);

	pkt.sig.q = ctx->sig_q;
	memcpy(pkt.sig.ots, ctx->sig_ots, LMS_SIG_OTS_LEN);
	pkt.sig.tree_type = ctx->sig_tree_type;
	memcpy(pkt.sig.tree_path, ctx->sig_tree_path, LMS_SIG_TREE_PATH);

	ret = lms_begin_session(dev, &ini, &ek);
	if (ret) {
		LOG_ERR("lms_begin_session fail: %d", ret);
		goto end;
	}

	ret = lms_verify(&ini, &pkt);
	if (ret) {
		LOG_ERR("lms verify failed\n");
		rc = -1;
	} else {
		LOG_INF("lms verify pass\n");
		rc = 0;
	}

	lms_free_session(dev, &ini);

end:
	return rc;
}

static int cptra_ipc_ecdsa384_verify(const struct device *dev, void *arg1, void *arg2)
{
	struct cptra_ecdsa_ctx *ctx = (struct cptra_ecdsa_ctx *)arg1;
	struct ecdsa_ctx ini;
	struct ecdsa_pkt pkt;
	struct ecdsa_key ek;
	int ret, rc = 0;

	ek.curve_id = ECC_CURVE_NIST_P384;
	ek.qx = (char *)ctx->qx;
	ek.qy = (char *)ctx->qy;
	pkt.m = (char *)ctx->m;
	pkt.r = (char *)ctx->r;
	pkt.s = (char *)ctx->s;
	pkt.m_len = ctx->m_len;
	pkt.r_len = ctx->r_len;
	pkt.s_len = ctx->s_len;

	ret = ecdsa_begin_session(dev, &ini, &ek);
	if (ret)
		LOG_ERR("ecdsa_begin_session fail: %d", ret);

	ret = ecdsa_verify(&ini, &pkt);
	if (ret) {
		LOG_ERR("ecdsa384 verify failed\n");
		rc = -1;
	} else {
		LOG_DBG("ecdsa384 verify pass\n");
		rc = 0;
	}

	ecdsa_free_session(dev, &ini);

	return rc;
}

static int cptra_ipc_sha384(const struct device *dev, void *arg1, void *arg2)
{
	struct cptra_hash_ctx *ctx = (struct cptra_hash_ctx *)arg1;
	enum hash_algo algo;
	struct hash_ctx ini;
	struct hash_pkt pkt;
	int ret;

	algo = ctx->algo;
	pkt.in_buf = ctx->in_buf;
	pkt.in_len = ctx->in_len;
	pkt.out_buf = ctx->out_buf;

	LOG_DBG("algo: %d", algo);
	LOG_DBG("pkt.in_buf: 0x%x", (uint32_t)pkt.in_buf);
	LOG_DBG("pkt.in_len: %d", pkt.in_len);
	LOG_DBG("pkt.out_buf: 0x%x", (uint32_t)pkt.out_buf);
	LOG_HEXDUMP_DBG(pkt.in_buf, pkt.in_len, "input:");

	/* initial */
	ret = hash_begin_session(dev, &ini, algo);
	if (ret) {
		LOG_ERR("hash_begin_session error");
		return -EINVAL;
	}

	/* update */
	ret = hash_update(&ini, &pkt);
	if (ret) {
		LOG_ERR("hash_update error");
		goto end;
	}

	/* final */
	ret = hash_compute(&ini, &pkt);
	if (ret) {
		LOG_ERR("hash_compute error");
		goto end;
	}

	LOG_HEXDUMP_DBG(pkt.out_buf, 48, "digest:");

end:
	hash_free_session(dev, &ini);

	return ret;
}

struct hash_ctx g_ini;
struct hash_pkt g_pkt;

static int cptra_ipc_sha384_init(const struct device *dev, void *arg1, void *arg2)
{
	struct cptra_hash_ctx *ctx = (struct cptra_hash_ctx *)arg1;
	int ret;

	LOG_DBG("algo: %d", ctx->algo);

	/* initial */
	ret = hash_begin_session(dev, &g_ini, ctx->algo);
	if (ret) {
		LOG_ERR("hash_begin_session error");
		return -EINVAL;
	}

	return ret;
}

static int cptra_ipc_sha384_update(const struct device *dev, void *arg1, void *arg2)
{
	struct cptra_hash_ctx *ctx = (struct cptra_hash_ctx *)arg1;
	int ret;

	g_pkt.in_buf = ctx->in_buf;
	g_pkt.in_len = ctx->in_len;

	LOG_DBG("pkt.in_buf: 0x%x", (uint32_t)g_pkt.in_buf);
	LOG_DBG("pkt.in_len: %d", g_pkt.in_len);
	LOG_DBG("pkt.out_buf: 0x%x", (uint32_t)g_pkt.out_buf);
	LOG_HEXDUMP_DBG(g_pkt.in_buf, g_pkt.in_len, "input:");

	/* update */
	ret = hash_update(&g_ini, &g_pkt);
	if (ret) {
		LOG_ERR("hash_update error");
		goto end;
	}

	return 0;

end:
	hash_free_session(dev, &g_ini);

	return ret;
}

static int cptra_ipc_sha384_final(const struct device *dev, void *arg1, void *arg2)
{
	struct cptra_hash_ctx *ctx = (struct cptra_hash_ctx *)arg1;
	int ret;

	g_pkt.out_buf = ctx->out_buf;

	/* final */
	ret = hash_compute(&g_ini, &g_pkt);
	if (ret) {
		LOG_ERR("hash_compute error");
		goto end;
	}

	LOG_HEXDUMP_DBG(g_pkt.out_buf, 48, "digest:");

end:
	hash_free_session(dev, &g_ini);

	return ret;
}

static void cptra_ipc_cb(const struct device *ipmdev, void *user_data,
			 uint32_t id, volatile void *msg_data)
{
	uint32_t *buf = (uint32_t *)msg_data;
	uint32_t cmd = buf[0];
	uint32_t *input = (uint32_t *)buf[1];
	uint32_t *output = (uint32_t *)buf[2];
	const struct device *dev;
	int ret = -EINVAL;

	LOG_DBG("id: 0x%x", id);
	LOG_HEXDUMP_DBG((uint8_t *)buf, 32, "msg");

	for (int i = 0; i < ARRAY_SIZE(cptra_ipc_list); i++) {
		if (cptra_ipc_list[i].cmd == cmd) {
			LOG_DBG("cmd: 0x%x", cmd);
			LOG_DBG("buf[1]:0x%x", (uint32_t)input);
			LOG_DBG("buf[2]:0x%x", (uint32_t)output);
			dev = device_get_binding(cptra_ipc_list[i].dev);
			if (cptra_ipc_list[i].cptra_cb)
				ret = cptra_ipc_list[i].cptra_cb(dev, (void *)input,
								 (void *)output);
			break;
		}
	}

	LOG_DBG("cptra_cb: ret: 0x%x", ret);
	ipm_send(ipmdev, 0, IPC_CHANNEL_ID_CPTRA, (void *)&ret, sizeof(ret));
}

int cptra_ipc_enable(void)
{
	static const char * const ipc_name[] = {
		IPC_DEV_SSP_BOOTMCU,
		IPC_DEV_NS_CA35_BOOTMCU,
	};
	int device_id = IPC_CHANNEL_ID_CPTRA;
	const struct device *ipmdev;
	int rc = 0;

	LOG_INF("%s", __func__);

	for (int i = 0; i < ARRAY_SIZE(ipc_name); i++) {
		LOG_INF("Register IPC %s channel %d callback", ipc_name[i], device_id);
		ipmdev = device_get_binding(ipc_name[i]);
		if (!ipmdev) {
			printk("%s: device_get_binding failed to find device\n", ipc_name[i]);
			rc = -1;
			return rc;
		}

		ipm_register_id_callback(ipmdev, device_id, cptra_ipc_cb, NULL);
		rc = ipm_set_id_enabled(ipmdev, device_id, 1);
		if (rc) {
			printk("%s: cannot ipm_set_enabled\n", ipc_name[i]);
			return rc;
		}
	}

	return 0;
}
