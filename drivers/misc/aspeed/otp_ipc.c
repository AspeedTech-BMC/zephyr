/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/misc/aspeed/otp.h>
#include <zephyr/drivers/misc/aspeed/otp_ipc.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(otp_ipc, CONFIG_MISC_ASPEED_LOG_LEVEL);

#define IPC_DEV_SSP_BOOTMCU		"ipc1@400"
#define IPC_DEV_NS_CA35_BOOTMCU		"ipc1@200"
#define OTP_DEV				DEVICE_DT_GET(DT_INST(0, aspeed_ast27xx_otp))

static void otp_ipc_cb(const struct device *ipmdev, void *user_data,
		       uint32_t id, volatile void *msg_data)
{
	uint32_t *buf = (uint32_t *)msg_data;
	uint32_t addr = buf[0];
	uint32_t seq = OTP_IPC_REQ_SEQ(buf[1]);
	uint32_t op = OTP_IPC_UNPACK_OP(buf[1]);
	uint32_t count = OTP_IPC_UNPACK_COUNT(buf[1]);
	uint32_t resp[8] = {0};
	const struct device *dev = OTP_DEV;
	int status;

	if (!device_is_ready(dev)) {
		status = -ENODEV;
		goto done;
	}

	switch (op) {
	case OTP_IPC_OP_READ: {
		uint16_t data[OTP_IPC_READ_MAX_WORDS];

		if (count > OTP_IPC_READ_MAX_WORDS) {
			status = -EINVAL;
			break;
		}

		status = otp_read_multi(dev, addr, data, count);
		if (status == 0) {
			for (uint32_t i = 0; i < count; i++)
				((uint16_t *)&resp[1])[i] = data[i];
		}
		break;
	}
	case OTP_IPC_OP_PROGRAM: {
		uint16_t data[OTP_IPC_PROGRAM_MAX_WORDS];

		if (count > OTP_IPC_PROGRAM_MAX_WORDS) {
			status = -EINVAL;
			break;
		}

		for (uint32_t i = 0; i < count; i++)
			data[i] = ((uint16_t *)&buf[2])[i];

		status = otp_program_multi(dev, addr, data, count);
		break;
	}
	case OTP_IPC_OP_ECC_GET_STATUS: {
		int ecc_status = 0;

		status = otp_ecc_get_status(dev, &ecc_status);
		if (status == 0)
			resp[1] = (uint32_t)ecc_status;
		break;
	}
	case OTP_IPC_OP_ECC_SET_ENABLE: {
		status = otp_ecc_set_enable(dev, count != 0);
		break;
	}
	case OTP_IPC_OP_ECC_GET_POLICY: {
		bool ecc_en = false;
		bool ecc_supported = false;

		status = otp_ecc_get_policy(dev, count, &ecc_en, &ecc_supported);
		if (status == 0) {
			resp[1] = (uint32_t)ecc_en;
			resp[2] = (uint32_t)ecc_supported;
		}
		break;
	}
	case OTP_IPC_OP_ECC_SET_POLICY: {
		status = otp_ecc_set_policy(dev, count, buf[2] != 0);
		break;
	}
	case OTP_IPC_OP_GET_REVID: {
		uint32_t revid0 = 0, revid1 = 0;

		status = otp_get_chip_revid(dev, &revid0, &revid1);
		if (status == 0) {
			resp[1] = revid0;
			resp[2] = revid1;
		}
		break;
	}
	case OTP_IPC_OP_SET_READ_MODE: {
		status = otp_set_read_mode(dev, count);
		break;
	}
	default:
		status = -EINVAL;
		break;
	}

done:
	resp[0] = OTP_IPC_PACK_RESP(seq, status);

	LOG_DBG("otp_ipc: dev=%s seq=%u op=%u addr=0x%x count=%u status=%d",
		ipmdev->name, seq, op, addr, count, status);

	ipm_send(ipmdev, 0, IPC_CHANNEL_ID_OTP, resp, sizeof(resp));
}

int otp_ipc_enable(void)
{
	static const char * const ipc_name[] = {
		IPC_DEV_SSP_BOOTMCU,
		IPC_DEV_NS_CA35_BOOTMCU,
	};
	const struct device *ipmdev;
	int rc;

	LOG_INF("%s", __func__);

	for (int i = 0; i < ARRAY_SIZE(ipc_name); i++) {
		ipmdev = device_get_binding(ipc_name[i]);
		if (!ipmdev) {
			LOG_ERR("%s: device_get_binding failed to find device", ipc_name[i]);
			return -1;
		}

		ipm_register_id_callback(ipmdev, IPC_CHANNEL_ID_OTP, otp_ipc_cb, NULL);
		rc = ipm_set_id_enabled(ipmdev, IPC_CHANNEL_ID_OTP, 1);
		if (rc) {
			LOG_ERR("%s: cannot enable OTP channel", ipc_name[i]);
			return rc;
		}
	}

	return 0;
}
