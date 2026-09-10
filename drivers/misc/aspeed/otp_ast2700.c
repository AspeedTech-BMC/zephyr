/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <stdint.h>
#define DT_DRV_COMPAT		aspeed_ast27xx_otp

#include <soc.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/misc/aspeed/otp_ast27xx.h>

#include "otp_info_ast27xx.h"
#include <zephyr/drivers/misc/aspeed/otp.h>

LOG_MODULE_REGISTER(otp_ast2700, CONFIG_LOG_DEFAULT_LEVEL);

/***********************
 *                     *
 * OTP regs definition *
 *                     *
 ***********************/
#define OTP_REG_SIZE			0x200

#define OTP_PASSWD			0x349fe38a
#define OTP_CMD_READ			0x23b1e361
#define OTP_CMD_PROG			0x23b1e364
#define OTP_CMD_PROG_MULTI		0x23b1e365

#define OTP_CMD_OFFSET			0x20

#if defined(CONFIG_SOC_AST2700_SSP)
#define OTP_MASTER			OTP_M2
#elif defined(CONFIG_SOC_AST2700_TSP)
#define OTP_MASTER			OTP_M3
#elif defined(CONFIG_SOC_AST1040_CM4)
/* AST1040 CM4 plays the CA35 role on this OTP macro */
#define OTP_MASTER			OTP_M1
#else
/* bootmcu (AST2700 and AST1040 share OTP_M0 here) */
#define OTP_MASTER			OTP_M0
#endif


#define OTP_KEY				0x0
#define OTP_CMD				(OTP_MASTER * OTP_CMD_OFFSET + 0x4)
#define OTP_WDATA_0			(OTP_MASTER * OTP_CMD_OFFSET + 0x8)
#define OTP_WDATA_1			(OTP_MASTER * OTP_CMD_OFFSET + 0xc)
#define OTP_WDATA_2			(OTP_MASTER * OTP_CMD_OFFSET + 0x10)
#define OTP_WDATA_3			(OTP_MASTER * OTP_CMD_OFFSET + 0x14)
#define OTP_STATUS			(OTP_MASTER * OTP_CMD_OFFSET + 0x18)
#define OTP_ADDR			(OTP_MASTER * OTP_CMD_OFFSET + 0x1c)
#define OTP_RDATA			(OTP_MASTER * OTP_CMD_OFFSET + 0x20)

#define OTP_DBG01                   0x0C8
#define OTP_ECC_EN                  0x0D4
#define OTP_PMC_CQ                  0x0E4

#define OTP_MASTER_ID               0x1B0
#define OTP_MASTER_ID_EXT           0x1B4
#define OTP_R_MASTER_ID             0x1B8
#define OTP_R_MASTER_ID_EXT         0x1BC
#define OTP_MASTER_ID_LOCK          BIT(31)

/* Table 4.3: Manager ID for OTP Security Access Control */
#define OTP_MID_CA35_WRITE          0x00
#define OTP_MID_CA35_READ           0x01
#define OTP_MID_SSP_SBUS            0x04
#define OTP_MID_UARTDBG_CPU_DIE	    0x08
#define OTP_MID_TSP_SBUS            0x0B
#define OTP_MID_BOOTMCU_DCHANNEL    0x21
#define OTP_MID_UARTDBG_IO_DIE      0x30

/* Not in Table 4.3: parks unused master-id ports on an id nothing presents */
#define OTP_MID_NONE                0x3F

#define OTP_DAP_CFG_RQ			0x538

/* OTP status: [0] */
#define OTP_STS_IDLE			0x0
#define OTP_STS_BUSY			0x1

/* OTP cmd status: [7:4] */
#define OTP_GET_CMD_STS(x)		(((x) & 0xF0) >> 4)
#define OTP_STS_PASS			0x0
#define OTP_STS_FAIL			0x1
#define OTP_STS_CMP_FAIL		0x2
#define OTP_STS_REGION_FAIL		0x3
#define OTP_STS_MASTER_FAIL		0x4

/*
 * OTP_DBG01 ECC status: [5] single-bit error (corrected), [4:0] ECC syndrome
 *   S[5]=0, S[4:0]=0    -> no error
 *   S[5]=0, S[4:0]!=0   -> dual-bit error in Data or ECC[4:0] (uncorrectable)
 *   S[5]=1, S[4:0]!=0   -> single-bit error in Data or ECC[4:0] (corrected)
 */
#define OTP_ECC_STS_SINGLE_ERR		BIT(5)
#define OTP_ECC_STS_SYNDROME(x)		((x) & GENMASK(4, 0))

/* OTP ECC EN */
#define ECC_ENABLE			0x1
#define ECC_DISABLE			0x0
#define ECCBRP_EN			BIT(0)

#define ROM_REGION_START_ADDR		0x0
#define ROM_REGION_END_ADDR		0x3e0
#define RBP_REGION_START_ADDR		ROM_REGION_END_ADDR
#define RBP_REGION_END_ADDR		0x400
#define CONF_REGION_START_ADDR		RBP_REGION_END_ADDR
#define CONF_REGION_END_ADDR		0x420
#define STRAP_REGION_START_ADDR		CONF_REGION_END_ADDR
#define STRAP_REGION_END_ADDR		0x430
#define STRAPEXT_REGION_START_ADDR	STRAP_REGION_END_ADDR
#define STRAPEXT_REGION_END_ADDR	0x440
#define USER_REGION_START_ADDR		STRAPEXT_REGION_END_ADDR
#define USER_REGION_END_ADDR		0x1000
#define SEC_REGION_START_ADDR		USER_REGION_END_ADDR
#define SEC_REGION_END_ADDR		0x1c00
#define CAL_REGION_START_ADDR		SEC_REGION_END_ADDR
#define CAL_REGION_END_ADDR		0x1f80
#define   CAL_VENDOR_KEY_HASH_OFFSET	0x12
#define   CAL_VENDOR_KEY_HASH_BYTES	48
#define SW_PUF_REGION_START_ADDR	CAL_REGION_END_ADDR
#define SW_PUF_REGION_END_ADDR		0x1fc0
#define HW_PUF_REGION_START_ADDR	SW_PUF_REGION_END_ADDR
#define HW_PUF_REGION_END_ADDR		0x2000

#define OTP_TIMEOUT_US			10000

/* OTPSTRAP */
#define OTPSTRAP0_ADDR			STRAP_REGION_START_ADDR
#define OTPSTRAP14_ADDR			(OTPSTRAP0_ADDR + 0xe)

#define SCU1_ROM_PATCH_OFFSET		0x180

#define ID0_AST2700A0			0x06000003
#define ID1_AST2700A0			0x06000003
#define ID0_AST2750A1			0x06010003
#define ID1_AST2750A1			0x06010003
#define ID0_AST2700A1			0x06010103
#define ID1_AST2700A1			0x06010103
#define ID0_AST2750A2			0x06020003
#define ID1_AST2750A2			0x06020003
#define ID0_AST2700A2			0x06020103
#define ID1_AST2700A2			0x06020103
#define ID0_AST2720A2			0x06020203
#define ID1_AST2720A2			0x06020203

#if defined(CONFIG_SOC_AST2700_SSP) || defined(CONFIG_SOC_AST2700_TSP)
#define SCU0_REVISION_ID		0x72C02000
#define SCU1_REVISION_ID		0x74C02000
#else
#define SCU0_REVISION_ID		0x12C02000
#define SCU1_REVISION_ID		0x14C02000
#endif

enum otp_error_code {
	OTP_SUCCESS,

	/*
	 * Dedicated error codes for OTP_STATUS[7:4] command results, kept
	 * out of the POSIX errno range so callers can tell them apart from
	 * generic I/O errors.
	 */
	OTP_CMD_ERR_BASE = 200,
	OTP_CMD_ERR_FAIL,           /* prog fail or soak limit exceeded */
	OTP_CMD_ERR_CMP_FAIL,       /* compare mismatch */
	OTP_CMD_ERR_REGION_FAIL,    /* region write/read protected */
	OTP_CMD_ERR_MASTER_FAIL,    /* master protection error */
	OTP_ECC_ERR_DUAL,           /* uncorrectable dual-bit ECC error */
};

enum aspeed_otp_master_id {
	OTP_M0 = 0,
	OTP_M1,
	OTP_M2,
	OTP_M3,
	OTP_M4,
	OTP_M5,
	OTP_MID_MAX,
};

enum rom_patch_version_a1 {
	A1_OTP_ROM_PATCH_NONE =	0x0,
	A1_OTP_ROM_PATCH_V1 =	0x3,
	A1_OTP_ROM_PATCH_V2 =	0x3276,
	A1_OTP_ROM_PATCH_V3 =	0x3376,
};


struct otp_ast27xx_config {
	uintptr_t base;
	uintptr_t scu_base;
	/* OTP macro cell width in bits (16 on AST2700, 32 on AST1040 - same design as AST2705) */
	uint8_t gran_bits;
};

struct otp_ast27xx_drv_state {
	bool in_use;
	int gbl_ecc_en;
};

enum otp_region_id {
	OTP_REGION_ROM = 0,
	OTP_REGION_RBP,
	OTP_REGION_CFG,
	OTP_REGION_STRAP,
	OTP_REGION_STRAPEXT,
	OTP_REGION_USR,
	OTP_REGION_SEC,
	OTP_REGION_CAL,
	OTP_REGION_PUF,
	OTP_REGION_MAX,
};

struct otp_region_ecc {
	uint32_t start;
	uint32_t end;
	bool ecc_supported;
	bool ecc_en;
};

#if !defined(CONFIG_SOC_SERIES_AST10x0_G2)
/*
 * Per-region ECC default, consulted when state->gbl_ecc_en (force ECC) is
 * off. OTPRBP/OTPSTRAP don't support ECC in hardware, so ecc_supported is
 * false and ECC stays off for them even under the global force.
 */
static const struct otp_region_ecc otp_region_ecc_tbl[OTP_REGION_MAX] = {
	[OTP_REGION_ROM] = {
		ROM_REGION_START_ADDR, ROM_REGION_END_ADDR, true, true
	},
	[OTP_REGION_RBP] = {
		RBP_REGION_START_ADDR, RBP_REGION_END_ADDR, false, false
	},
	[OTP_REGION_CFG] = {
		CONF_REGION_START_ADDR, CONF_REGION_END_ADDR, true, false
	},
	[OTP_REGION_STRAP] = {
		STRAP_REGION_START_ADDR, STRAP_REGION_END_ADDR, false, false
	},
	[OTP_REGION_STRAPEXT] = {
		STRAPEXT_REGION_START_ADDR, STRAPEXT_REGION_END_ADDR, true, false
	},
	[OTP_REGION_USR] = {
		USER_REGION_START_ADDR, USER_REGION_END_ADDR, true, false
	},
	[OTP_REGION_SEC] = {
		SEC_REGION_START_ADDR, SEC_REGION_END_ADDR, true, false
	},
	[OTP_REGION_CAL] = {
		CAL_REGION_START_ADDR, CAL_REGION_END_ADDR, true, false
	},
	[OTP_REGION_PUF] = {
		SW_PUF_REGION_START_ADDR, HW_PUF_REGION_END_ADDR, true, true
	},
};
#endif /* !CONFIG_SOC_SERIES_AST10x0_G2 */

static bool otp_region_ecc_active(const struct device *dev, uint32_t offset)
{
	struct otp_ast27xx_drv_state *state = (struct otp_ast27xx_drv_state *)dev->data;

#if defined(CONFIG_SOC_SERIES_AST10x0_G2)
	/* Region layout/ECC table below is AST2700-specific; TBD on AST1040 */
	return state->gbl_ecc_en;
#else
	for (int i = 0; i < OTP_REGION_MAX; i++) {
		const struct otp_region_ecc *region = &otp_region_ecc_tbl[i];

		if (offset < region->start || offset >= region->end)
			continue;

		if (!region->ecc_supported)
			return false;

		return state->gbl_ecc_en || region->ecc_en;
	}

	return false;
#endif
}

static void otp_unlock(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;

	sys_write32(OTP_PASSWD, cfg->base + OTP_KEY);
}

static int wait_complete(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	uint32_t val;
	uint32_t addr;
	uint32_t cmd_sts;
	bool done;

	done = WAIT_FOR(!((val = sys_read32(cfg->base + OTP_STATUS)) & OTP_STS_BUSY),
			OTP_TIMEOUT_US, k_busy_wait(1));
	if (!done) {
		LOG_WRN("%s: timeout. sts:0x%x", __func__, val);
		return -ETIMEDOUT;
	}

	addr = sys_read32(cfg->base + OTP_ADDR);

	cmd_sts = OTP_GET_CMD_STS(val);
	switch (cmd_sts) {
	case OTP_STS_PASS:
		return OTP_SUCCESS;
	case OTP_STS_FAIL:
		LOG_ERR("%s: prog fail or soak limit exceeded at addr 0x%x", __func__, addr);
		return -OTP_CMD_ERR_FAIL;
	case OTP_STS_CMP_FAIL:
		LOG_ERR("%s: compare mismatch at addr 0x%x", __func__, addr);
		return -OTP_CMD_ERR_CMP_FAIL;
	case OTP_STS_REGION_FAIL:
		LOG_ERR("%s: region write/read protected at addr 0x%x", __func__, addr);
		return -OTP_CMD_ERR_REGION_FAIL;
	case OTP_STS_MASTER_FAIL:
		LOG_ERR("%s: master protection error at addr 0x%x", __func__, addr);
		return -OTP_CMD_ERR_MASTER_FAIL;
	default:
		LOG_ERR("%s: unknown cmd sts:0x%x", __func__, cmd_sts);
		return -EIO;
	}
}

static int otp_check_ecc_status(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	uint32_t status, addr, syndrome;

	status = sys_read32(cfg->base + OTP_DBG01);
	syndrome = OTP_ECC_STS_SYNDROME(status);

	if (!syndrome)
		return 0;

	addr = sys_read32(cfg->base + OTP_ADDR);

	if (status & OTP_ECC_STS_SINGLE_ERR) {
		LOG_DBG("%s: single-bit ECC error corrected, addr:0x%x, syndrome:0x%x",
			__func__, addr, syndrome);
		return 0;
	}

	LOG_ERR("%s: uncorrectable dual-bit ECC error, addr:0x%x, syndrome:0x%x",
		__func__, addr, syndrome);
	return -OTP_ECC_ERR_DUAL;
}

static void otp_ecc_cfg(const struct device *dev, bool ecc_en, bool auto_cfg)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	bool self_cfg = ecc_en && !auto_cfg;

	sys_write32(ecc_en, cfg->base + OTP_ECC_EN);

	/* OTP_PMC_CQ/OTP_DAP_CFG_RQ don't exist on AST1040 */
	if (IS_ENABLED(CONFIG_SOC_SERIES_AST10x0_G2))
		return;

	/* Self config or auto config */
	sys_write32(self_cfg ? 0x4 : 0x0, cfg->base + OTP_PMC_CQ);
	/* Clearing OTP_PMC_CQ auto-reverts OTP_DAP_CFG_RQ, no explicit write needed to disable */
	if (self_cfg)
		sys_write32(0x40008, cfg->base + OTP_DAP_CFG_RQ);
}

static int otp_read_data(const struct device *dev, uint32_t offset, uint32_t *data)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int ret;
	bool ecc_en = otp_region_ecc_active(dev, offset);

	sys_write32(offset, cfg->base + OTP_ADDR);
	otp_ecc_cfg(dev, ecc_en, false);

	sys_write32(OTP_CMD_READ, cfg->base + OTP_CMD);
	ret = wait_complete(dev);
	if (!ret)
		data[0] = sys_read32(cfg->base + OTP_RDATA);

	if (!ret && ecc_en)
		ret = otp_check_ecc_status(dev);

	/* Restore ECC config to default */
	otp_ecc_cfg(dev, ecc_en, true);

	return ret;
}

int otp_prog_data(const struct device *dev, uint32_t offset, uint32_t data)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;

	sys_write32(otp_region_ecc_active(dev, offset), cfg->base + OTP_ECC_EN);
	/* OTP_PMC_CQ doesn't exist on AST1040 */
	if (!IS_ENABLED(CONFIG_SOC_SERIES_AST10x0_G2))
		sys_write32(0x0, cfg->base + OTP_PMC_CQ);

	sys_write32(offset, cfg->base + OTP_ADDR);
	sys_write32(data, cfg->base + OTP_WDATA_0);
	sys_write32(OTP_CMD_PROG, cfg->base + OTP_CMD);

	return wait_complete(dev);
}

int otp_prog_multi_data(const struct device *dev, uint32_t offset, uint32_t *data, int count)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;

	/* Only 4 WDATA registers (WDATA_0..WDATA_3) exist per OTP_CMD_PROG_MULTI */
	if (count > 4)
		return -EINVAL;

	sys_write32(otp_region_ecc_active(dev, offset), cfg->base + OTP_ECC_EN);
	/* OTP_PMC_CQ doesn't exist on AST1040 */
	if (!IS_ENABLED(CONFIG_SOC_SERIES_AST10x0_G2))
		sys_write32(0x0, cfg->base + OTP_PMC_CQ);

	sys_write32(offset, cfg->base + OTP_ADDR);
	for (int i = 0; i < count; i++)
		sys_write32(data[i], cfg->base + OTP_WDATA_0 + 4 * i);

	sys_write32(OTP_CMD_PROG_MULTI, cfg->base + OTP_CMD);

	return wait_complete(dev);
}

static int aspeed_otp_read(const struct device *dev, uint32_t offset, void *buf, int size)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int ret = 0;
	uint32_t rdata;

	for (int i = 0; i < size; i++) {
		ret = otp_read_data(dev, offset + i, &rdata);
		if (ret) {
			LOG_ERR("%s: read failed", __func__);
			break;
		}
		if (cfg->gran_bits == 32)
			((uint32_t *)buf)[i] = rdata;
		else
			((uint16_t *)buf)[i] = (uint16_t)rdata;
	}

	return ret;
}

static int aspeed_otp_write(const struct device *dev, uint32_t offset, void *buf, int size)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int ret;

	if (cfg->gran_bits == 32) {
		uint32_t *data = (uint32_t *)buf;

		if (size == 1)
			ret = otp_prog_data(dev, offset, data[0]);
		else
			ret = otp_prog_multi_data(dev, offset, data, size);
	} else {
		uint32_t *data32 = (uint32_t *)buf;
		uint16_t *data = (uint16_t *)buf;

		if (size == 1)
			ret = otp_prog_data(dev, offset, data[0]);
		else
			ret = otp_prog_multi_data(dev, offset, data32, size / 2);
	}

	if (ret)
		LOG_ERR("%s: prog failed", __func__);

	return ret;
}

static int aspeed_otp_ecc_init(const struct device *dev)
{
	struct otp_ast27xx_drv_state *state = (struct otp_ast27xx_drv_state *)dev->data;

#if defined(CONFIG_SOC_SERIES_AST10x0_G2)
	/* OTPSTRAP14 location is AST2700-specific; region layout is TBD on AST1040 */
	state->gbl_ecc_en = 0x0;

	return 0;
#else
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int ret;
	uint32_t val;

	/* Check cfg_ecc_en */
	sys_write32(0, cfg->base + OTP_ECC_EN);
	sys_write32(OTPSTRAP14_ADDR, cfg->base + OTP_ADDR);
	sys_write32(OTP_CMD_READ, cfg->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return ret;

	val = sys_read32(cfg->base + OTP_RDATA);
	if (val & 0x1)
		state->gbl_ecc_en = 0x1;
	else
		state->gbl_ecc_en = 0x0;

	return 0;
#endif
}

static int aspeed_chip_version(const struct device *dev, uint32_t *chip_version)
{
#if defined(CONFIG_SOC_SERIES_AST10x0_G2)
	/*
	 * AST1040 is single-die and its chip-ID revision values aren't known
	 * yet, so don't touch AST2700's dual-die SCU0/SCU1 addresses here.
	 */
	*chip_version = -1;

	return 0;
#else
	uint32_t revid0, revid1;

	revid0 = sys_read32(SCU0_REVISION_ID);
	revid1 = sys_read32(SCU1_REVISION_ID);

	if (revid0 == ID0_AST2700A0 && revid1 == ID1_AST2700A0) {
		/* AST2700-A0 */
		*chip_version = OTP_AST2700_A0;
	} else if ((revid0 == ID0_AST2700A1 && revid1 == ID1_AST2700A1) ||
		   (revid0 == ID0_AST2750A1 && revid1 == ID1_AST2750A1)) {
		/* AST2700-A1 */
		*chip_version = OTP_AST2700_A1;
	}  else if ((revid0 == ID0_AST2700A2 && revid1 == ID1_AST2700A2) ||
		   (revid0 == ID0_AST2750A2 && revid1 == ID1_AST2750A2) ||
		   (revid0 == ID0_AST2720A2 && revid1 == ID1_AST2720A2)) {
		/* AST2700-A2 */
		*chip_version = OTP_AST2700_A2;
	} else {
		*chip_version = -1;
	}

	return 0;
#endif
}

#if !defined(CONFIG_SOC_SERIES_AST10x0_G2)
static void aspeed_otp_rom_info_a2(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	uint32_t rom_patch_ver;

	rom_patch_ver = sys_read32(cfg->scu_base + SCU1_ROM_PATCH_OFFSET);

	if (rom_patch_ver == 0x0) {
		LOG_INF("\tROM patch: None");
	} else if ((rom_patch_ver & 0xFF) == 'v') {
		LOG_INF("\tROM patch: %.4s", (char *)&rom_patch_ver);
	} else {
		LOG_INF("\tROM patch: Unknown (0x%x)", rom_patch_ver);
	}
}

static void aspeed_otp_rom_info_a1(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int rom_patch_ver;
	char *rom_ver_str;

	/* Check ROM patch version */
	rom_patch_ver = sys_read32(cfg->scu_base + SCU1_ROM_PATCH_OFFSET);
	switch (rom_patch_ver) {
	case A1_OTP_ROM_PATCH_NONE:
		rom_ver_str = "None";
		break;
	case A1_OTP_ROM_PATCH_V1:
		rom_ver_str = "v1";
		break;
	case A1_OTP_ROM_PATCH_V2:
		rom_ver_str = "v2";
		break;
	case A1_OTP_ROM_PATCH_V3:
		rom_ver_str = "v3";
		break;
	default:
		rom_ver_str = "Unknown";
		break;
	}

	LOG_INF("\tROM patch: %s", rom_ver_str);
}
#endif /* !CONFIG_SOC_SERIES_AST10x0_G2 */

static void aspeed_otp_dump_info(const struct device *dev)
{
#if defined(CONFIG_SOC_SERIES_AST10x0_G2)
	/*
	 * ROM patch info (SCU1_ROM_PATCH_OFFSET) and CAL_REGION_START_ADDR
	 * both belong to the AST2700 region/SCU layout; region layout is
	 * still TBD on AST1040, so skip the whole dump rather than print
	 * misleading info read from addresses that have no confirmed
	 * meaning there.
	 */
	ARG_UNUSED(dev);
#else
	uint32_t offset = CAL_REGION_START_ADDR + CAL_VENDOR_KEY_HASH_OFFSET;
	uint16_t hash[CAL_VENDOR_KEY_HASH_BYTES / sizeof(uint16_t)];
	uint32_t ver;
	int ret;

	/* Dump ROM patch version */
	aspeed_chip_version(dev, &ver);

	if (ver == OTP_AST2700_A2) {
		aspeed_otp_rom_info_a2(dev);
	} else {
		aspeed_otp_rom_info_a1(dev);
	}

	/* Dump vendor keyhash */
	ret = aspeed_otp_read(dev, offset, hash, ARRAY_SIZE(hash));
	if (ret) {
		LOG_ERR("Failed to read vendor key hash");
		return;
	}

	LOG_INF("\tVendor key hash: %04x%04x...",
		sys_be16_to_cpu(hash[0]), sys_be16_to_cpu(hash[1]));
#endif
}

#if defined(CONFIG_SOC_AST2700_BOOTMCU)
static uint32_t otp_master_id_pack(uint8_t id0, uint8_t id1, uint8_t id2, uint8_t id3)
{
	return id0 | ((uint32_t)id1 << 8) | ((uint32_t)id2 << 16) | ((uint32_t)id3 << 24);
}

static uint32_t otp_master_id_ext_pack(uint8_t id4, uint8_t id5, bool lock)
{
	return id4 | ((uint32_t)id5 << 8) | (lock ? OTP_MASTER_ID_LOCK : 0);
}

static void otp_lock_master_access(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;

	/*
	 * Restrict which masters may access OTP before handing off to
	 * SSP/TSP. REG_MASTER_LOCK/RLOCK covers master ids 0-5, so ports
	 * 0-3 must be written before the _EXT write that also sets the
	 * lock bit, or the lock would latch before ports 0-3 are set.
	 */
	sys_write32(otp_master_id_pack(OTP_MID_BOOTMCU_DCHANNEL, OTP_MID_CA35_WRITE,
				       OTP_MID_SSP_SBUS, OTP_MID_TSP_SBUS),
		    cfg->base + OTP_MASTER_ID);
	sys_write32(otp_master_id_pack(OTP_MID_BOOTMCU_DCHANNEL, OTP_MID_CA35_READ,
				       OTP_MID_SSP_SBUS, OTP_MID_TSP_SBUS),
		    cfg->base + OTP_R_MASTER_ID);
	sys_write32(otp_master_id_ext_pack(OTP_MID_NONE, OTP_MID_NONE, true),
		    cfg->base + OTP_MASTER_ID_EXT);
	sys_write32(otp_master_id_ext_pack(OTP_MID_NONE, OTP_MID_NONE, true),
		    cfg->base + OTP_R_MASTER_ID_EXT);
}
#endif

static int otp_ast27xx_init(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int rc;

	otp_unlock(dev);

	/* OTP ECC init */
	rc = aspeed_otp_ecc_init(dev);
	if (rc) {
		LOG_ERR("OTP ECC init failed, rc:%d", rc);
		return rc;
	}

	aspeed_otp_dump_info(dev);

#if defined(CONFIG_SOC_AST2700_BOOTMCU)
	otp_lock_master_access(dev);
	LOG_INF("\t0x%x: OTP driver initialized, MASTER: 0x%x, master access locked",
		(uint32_t)cfg->base, OTP_MASTER);
#else
	LOG_INF("\t0x%x: OTP driver initialized, MASTER: 0x%x", (uint32_t)cfg->base, OTP_MASTER);
#endif

	return rc;
}

static struct otp_driver_api otp_funcs = {
	.otp_read_multi = aspeed_otp_read,
	.otp_program_multi = aspeed_otp_write,
	.get_chip_version = aspeed_chip_version,
};

static const struct otp_ast27xx_config otp_ast27xx_config = {
	.base = DT_REG_ADDR(DT_DRV_INST(0)),
	.scu_base = DT_REG_ADDR_BY_IDX(DT_INST_PHANDLE_BY_IDX(0, aspeed_scu, 0), 0),
	.gran_bits = DT_INST_PROP_OR(0, aspeed_otp_gran_bits, 16),
};

static struct otp_ast27xx_drv_state otp_ast27xx_state;

#define ASPEED_AST27XX_OTP_INIT(inst)					\
DEVICE_DT_INST_DEFINE(inst, otp_ast27xx_init, NULL,			\
		      &otp_ast27xx_state, &otp_ast27xx_config,		\
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
		      (void *)&otp_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST27XX_OTP_INIT)
