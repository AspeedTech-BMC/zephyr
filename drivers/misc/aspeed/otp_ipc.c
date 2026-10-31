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

/*
 * Which mailbox a request arrived on. Indexes otp_ipc_master_dev[] and the
 * per-op bitmasks below; must stay in the same order otp_ipc_enable()
 * registers the channels in.
 */
enum otp_ipc_master {
	OTP_IPC_MASTER_SSP = 0,
	OTP_IPC_MASTER_NS_CA35,
	OTP_IPC_MASTER_COUNT,
};

/* Resolved once in otp_ipc_enable(); otp_ipc_cb() maps its ipmdev back to one of these. */
static const struct device *otp_ipc_master_dev[OTP_IPC_MASTER_COUNT];

#define OTP_IPC_MASTER_BIT(m)		BIT(m)
#define OTP_IPC_MASTERS_NONE		0
#define OTP_IPC_MASTERS_ALL \
	(BIT(OTP_IPC_MASTER_SSP) | BIT(OTP_IPC_MASTER_NS_CA35))

/*
 * Per-op bitmask of which masters may issue it. OTP_IPC_OP_SET_READ_MODE
 * switches the DAP cell read reference to margin mode for CA35-side margin
 * screening (aspeed_app's "otp screen" tool), so it's routed to NS_CA35 here.
 *
 * SECURITY: margin mode can be abused for fault-injection/side-channel
 * attacks against OTP secrets. Granting it to the non-secure CA35 channel is
 * only appropriate for bring-up/test builds where that tool is actually
 * used - production builds should revisit this (e.g. drop the bit here, or
 * gate it behind a debug Kconfig) so CA35 can't reach it in the field.
 */
static const uint8_t otp_ipc_op_allowed_masters[] = {
	[OTP_IPC_OP_READ] = OTP_IPC_MASTERS_ALL,
	[OTP_IPC_OP_PROGRAM] = OTP_IPC_MASTERS_ALL,
	[OTP_IPC_OP_ECC_GET_STATUS] = OTP_IPC_MASTERS_ALL,
	[OTP_IPC_OP_ECC_SET_ENABLE] = OTP_IPC_MASTERS_ALL,
	[OTP_IPC_OP_ECC_GET_POLICY] = OTP_IPC_MASTERS_ALL,
	[OTP_IPC_OP_ECC_SET_POLICY] = OTP_IPC_MASTERS_ALL,
	[OTP_IPC_OP_GET_REVID] = OTP_IPC_MASTERS_ALL,
	/* Test/bring-up only - see SECURITY note above before shipping this as-is */
	[OTP_IPC_OP_SET_READ_MODE] = OTP_IPC_MASTER_BIT(OTP_IPC_MASTER_NS_CA35),
};

/*
 * Per-region bitmask of which masters may OTP_IPC_OP_READ/OTP_IPC_OP_PROGRAM
 * that region. Starts at OTP_IPC_MASTERS_ALL for every region; tighten
 * individual entries once there's an actual policy to enforce.
 */
static const uint8_t otp_ipc_region_allowed_masters[OTP_REGION_ID_MAX] = {
	[OTP_REGION_ID_ROM] = OTP_IPC_MASTERS_ALL,
	[OTP_REGION_ID_RBP] = OTP_IPC_MASTERS_ALL,
	[OTP_REGION_ID_CFG] = OTP_IPC_MASTERS_ALL,
	[OTP_REGION_ID_STRAP] = OTP_IPC_MASTERS_ALL,
	[OTP_REGION_ID_STRAPEXT] = OTP_IPC_MASTERS_ALL,
	[OTP_REGION_ID_USR] = OTP_IPC_MASTERS_ALL,
	[OTP_REGION_ID_SEC] = OTP_IPC_MASTERS_ALL,
	[OTP_REGION_ID_CAL] = OTP_IPC_MASTERS_ALL,
	[OTP_REGION_ID_PUF] = OTP_IPC_MASTERS_ALL,
};

static int otp_ipc_master_from_dev(const struct device *ipmdev)
{
	for (int master = 0; master < OTP_IPC_MASTER_COUNT; master++) {
		if (otp_ipc_master_dev[master] == ipmdev)
			return master;
	}

	return -1;
}

static bool otp_ipc_op_allowed(int master, uint32_t op)
{
	if (master < 0 || op >= ARRAY_SIZE(otp_ipc_op_allowed_masters))
		return false;

	return otp_ipc_op_allowed_masters[op] & BIT(master);
}

/*
 * otp_addr_to_region() only consults a build-time table, not the hardware,
 * so it's safe to call regardless of dev's ready state (every current
 * caller already passed device_is_ready() before reaching here, though).
 *
 * Targets with no region table at all (otp_addr_to_region() returns
 * -ENOTSUP, e.g. AST1040 or the A1 bootmcu build) have nothing to
 * restrict, so that specific failure defaults open. addr itself outside
 * every known region fails closed instead.
 *
 * A [addr, addr+count) request can straddle two adjacent regions (callers
 * read/program in fixed-size chunks that don't line up with region
 * boundaries) - that's fine as long as this master is allowed in both;
 * only deny if it's missing from either one. This assumes every region is
 * at least OTP_IPC_READ_MAX_WORDS/OTP_IPC_PROGRAM_MAX_WORDS wide, so a
 * request can never touch more than two - true today, but re-check this if
 * those limits ever grow past the smallest region.
 */
static bool otp_ipc_region_allowed(const struct device *dev, int master,
				   uint32_t addr, uint32_t count)
{
	uint32_t region, last_region;
	int ret;

	if (master < 0)
		return false;

	ret = otp_addr_to_region(dev, addr, &region);
	if (ret == -ENOTSUP)
		return true;
	if (ret)
		return false;

	if (region >= ARRAY_SIZE(otp_ipc_region_allowed_masters))
		return false;
	if (!(otp_ipc_region_allowed_masters[region] & BIT(master)))
		return false;

	if (count <= 1)
		return true;

	if (otp_addr_to_region(dev, addr + count - 1, &last_region))
		return false;
	if (last_region == region)
		return true;

	if (last_region >= ARRAY_SIZE(otp_ipc_region_allowed_masters))
		return false;

	return otp_ipc_region_allowed_masters[last_region] & BIT(master);
}

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
	int master = otp_ipc_master_from_dev(ipmdev);
	int status;

	if (!otp_ipc_op_allowed(master, op)) {
		LOG_WRN("otp_ipc: dev=%s op=%u denied", ipmdev->name, op);
		status = -EACCES;
		goto done;
	}

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

		if (!otp_ipc_region_allowed(dev, master, addr, count)) {
			LOG_WRN("otp_ipc: dev=%s op=%u addr=0x%x count=%u region denied",
				ipmdev->name, op, addr, count);
			status = -EACCES;
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

		if (!otp_ipc_region_allowed(dev, master, addr, count)) {
			LOG_WRN("otp_ipc: dev=%s op=%u addr=0x%x count=%u region denied",
				ipmdev->name, op, addr, count);
			status = -EACCES;
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

	BUILD_ASSERT(ARRAY_SIZE(ipc_name) == OTP_IPC_MASTER_COUNT);

	for (int i = 0; i < ARRAY_SIZE(ipc_name); i++) {
		ipmdev = device_get_binding(ipc_name[i]);
		if (!ipmdev) {
			LOG_ERR("%s: device_get_binding failed to find device", ipc_name[i]);
			return -1;
		}

		otp_ipc_master_dev[i] = ipmdev;

		ipm_register_id_callback(ipmdev, IPC_CHANNEL_ID_OTP, otp_ipc_cb, NULL);
		rc = ipm_set_id_enabled(ipmdev, IPC_CHANNEL_ID_OTP, 1);
		if (rc) {
			LOG_ERR("%s: cannot enable OTP channel", ipc_name[i]);
			return rc;
		}
	}

	return 0;
}
