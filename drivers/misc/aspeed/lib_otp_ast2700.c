/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <soc.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/otp_ast27xx.h>

#include "otp_info_ast27xx.h"
#include <zephyr/drivers/misc/aspeed/otp.h>

LOG_MODULE_REGISTER(lib_otp_ast2700, CONFIG_LOG_DEFAULT_LEVEL);

#ifdef DEBUG
#define OTP_INF(fmt, ...) \
	printk(fmt, ##__VA_ARGS__)
#else
#define OTP_INF(fmt, ...)
#endif

enum otp_region {
	OTP_REGION_ROM,
	OTP_REGION_RBP,
	OTP_REGION_CONF,
	OTP_REGION_STRAP,
	OTP_REGION_STRAP_EXT,
	OTP_REGION_STRAP_EXT_VLD,
	OTP_REGION_USER_DATA,
	OTP_REGION_SECURE,
	OTP_REGION_CALIPTRA,
	OTP_REGION_PUF,
};

enum otp_status {
	OTP_FAILURE = -2,
	OTP_USAGE = -1,
	OTP_SUCCESS = 0,
	OTP_PROG_SKIP,
};

#define OTP_VER				"1.1.0"

#define SOC_AST2700A0			8
#define SOC_AST2700A1			9

/* OTP memory address from 0x0~0x2000. (unit: Single Word 16-bits) */
/* ----  0x0  -----
 *       ROM
 * ---- 0x3e0 -----
 *       RBP
 * ---- 0x400 -----
 *      CONF
 * ---- 0x420 -----
 *      STRAP
 * ---- 0x430 -----
 *    STRAP EXT
 * ---- 0x440 -----
 *   User Region
 * ---- 0x1000 ----
 *  Secure Region
 * ---- 0x1c00 ----
 *     Caliptra
 * ---- 0x1f80 ----
 *      SW PUF
 * ---- 0x1fc0 ----
 *      HW PUF
 * ---- 0x2000 ----
 */
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
#define SW_PUF_REGION_START_ADDR	CAL_REGION_END_ADDR
#define SW_PUF_REGION_END_ADDR		0x1fc0
#define HW_PUF_REGION_START_ADDR	SW_PUF_REGION_END_ADDR
#define HW_PUF_REGION_END_ADDR		0x2000

#define OTP_MEM_ADDR_MAX		HW_PUF_REGION_START_ADDR
#define OTP_ROM_REGION_SIZE		(ROM_REGION_END_ADDR - ROM_REGION_START_ADDR)
#define OTP_RBP_REGION_SIZE		(RBP_REGION_END_ADDR - RBP_REGION_START_ADDR)
#define OTP_CONF_REGION_SIZE		(CONF_REGION_END_ADDR - CONF_REGION_START_ADDR)
#define OTP_STRAP_REGION_SIZE		(STRAP_REGION_END_ADDR - STRAP_REGION_START_ADDR - 4)
#define OTP_STRAP_EXT_REGION_SIZE	(STRAPEXT_REGION_END_ADDR - STRAPEXT_REGION_START_ADDR)
#define OTP_USER_REGION_SIZE		(USER_REGION_END_ADDR - USER_REGION_START_ADDR)
#define OTP_SEC_REGION_SIZE		(SEC_REGION_END_ADDR - SEC_REGION_START_ADDR)
#define OTP_CAL_REGION_SIZE		(CAL_REGION_END_ADDR - CAL_REGION_START_ADDR)
#define OTP_PUF_REGION_SIZE		(HW_PUF_REGION_END_ADDR - SW_PUF_REGION_START_ADDR)

/* OTPRBP */
#define OTPRBP0_ADDR			OTPRBP_START_ADDR
#define OTPRBP1_ADDR			0x1
#define OTPRBP2_ADDR			0x2
#define OTPRBP3_ADDR			0x3
#define OTPRBP4_ADDR			0x4
#define OTPRBP8_ADDR			0x8
#define OTPRBP10_ADDR			0xa
#define OTPRBP18_ADDR			0x12

#define SOC_ECC_KEY_RETIRE		OTPRBP0_ADDR
#define SOC_LMS_KEY_RETIRE		OTPRBP1_ADDR
#define CAL_OWN_KEY_RETURE		OTPRBP3_ADDR
#define SOC_HW_SVN_ADDR			OTPRBP4_ADDR
#define CAL_FMC_HW_SVN_ADDR		OTPRBP8_ADDR
#define CAL_RT_HW_SVN_ADDR		OTPRBP10_ADDR
#define CAL_MANU_ECC_KEY_MASK		OTPRBP18_ADDR

#define OTP_MAGIC			"SOCOTP"
#define CHECKSUM_LEN			48
#define OTP_INC_ROM			BIT(31)
#define OTP_INC_RBP			BIT(30)
#define OTP_INC_CONFIG			BIT(29)
#define OTP_INC_STRAP			BIT(28)
#define OTP_INC_STRAP_EXT		BIT(27)
#define OTP_INC_SECURE			BIT(26)
#define OTP_INC_CALIPTRA		BIT(25)
#define OTP_REGION_SIZE(info)		(((info) >> 16) & 0xffff)
#define OTP_REGION_OFFSET(info)		((info) & 0xffff)
#define OTP_IMAGE_SIZE(info)		((info) & 0xffff)

/* OTP key header format */
#define OTP_KH_NUM			80
#define OTP_KH_KEY_ID(kh)		((kh) & 0xf)
#define OTP_KH_KEY_TYPE(kh)		(((kh) >> 4) & 0x7)
#define OTP_KH_LAST(kh)			(((kh) >> 15) & 0x1)
#define OTP_KH_OFFSET(kh)		(((kh) >> 16) & 0xfff)

struct otp_header {
	uint8_t	otp_magic[8];
	uint32_t	soc_ver;
	uint32_t	otptool_ver;
	uint32_t	image_info;
	uint32_t	rom_info;
	uint32_t	rbp_info;
	uint32_t	config_info;
	uint32_t	strap_info;
	uint32_t	strap_ext_info;
	uint32_t	secure_info;
	uint32_t	cptra_info;
	uint32_t	checksum_offset;
} __packed;

struct otpstrap_status {
	int value;
	int option_value[6];
	int remain_times;
	int writeable_option;
	int protected;
};

static struct otp_info_cb info_cb;

struct otp_image_layout {
	int rom_length;
	int rbp_length;
	int conf_length;
	int strap_length;
	int strap_ext_length;
	int secure_length;
	int cptra_length;
	uint8_t *rom;
	uint8_t *rbp;
	uint8_t *conf;
	uint8_t *strap;
	uint8_t *strap_ext;
	uint8_t *secure;
	uint8_t *cptra;
};

enum command_ret_t {
	CMD_RET_SUCCESS,	/* 0 = Success */
	CMD_RET_FAILURE,	/* 1 = Failure */
	CMD_RET_USAGE = -1,	/* Failure, please report 'usage' error */
};

static int otp_ast27xx_init(void);

static void buf_print(uint8_t *buf, int len)
{
	int i;

	OTP_INF("      00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
	for (i = 0; i < len; i++) {
		if (i % 16 == 0)
			OTP_INF("%04X: ", i);
		OTP_INF("%02X ", buf[i]);
		if ((i + 1) % 16 == 0)
			OTP_INF("\n");
	}
	OTP_INF("\n");
}

#define OTP_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_ast27xx_otp))

static const struct device *otp_dev;

static bool is_otp_dev_ready(void)
{
	if (otp_dev && device_is_ready(otp_dev)) {
		return true;
	}

	LOG_DBG("OTP device is not ready or not bound\n");
	return false;
}

static int otp_dev_init(void)
{
	if (!otp_dev)
		otp_dev = device_get_binding(OTP_DRV_NAME);

	if (!otp_dev) {
		LOG_ERR("device get binding %s failed\n", OTP_DRV_NAME);
		return -ENODEV;
	}

	return 0;
}

static int lib_otp_read(uint32_t offset, uint16_t *data)
{
	return otp_read_multi(otp_dev, offset, data, 1);
}

int otp_read_rom(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + ROM_REGION_START_ADDR, data);
}

int otp_read_rbp(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + RBP_REGION_START_ADDR, data);
}

int otp_read_conf(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + CONF_REGION_START_ADDR, data);
}

int otp_read_strap(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + STRAP_REGION_START_ADDR, data);
}

int otp_read_strap_ext(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + STRAPEXT_REGION_START_ADDR, data);
}

int otp_read_strap_ext_vld(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + STRAPEXT_REGION_START_ADDR + 0x8, data);
}

int otp_read_user(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + USER_REGION_START_ADDR, data);
}

int otp_read_secure(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + SEC_REGION_START_ADDR, data);
}

static int otp_read_secure_multi(uint32_t offset, uint16_t *data, int num)
{
	return otp_read_multi(otp_dev, offset + SEC_REGION_START_ADDR, data, num);
}

int otp_read_cptra(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + CAL_REGION_START_ADDR, data);
}

int otp_read_puf(uint32_t offset, uint16_t *data)
{
	otp_ast27xx_init();

	return lib_otp_read(offset + SW_PUF_REGION_START_ADDR, data);
}

int otp_print_rom(uint32_t offset, int w_count)
{
	int range = OTP_ROM_REGION_SIZE;
	uint16_t ret[1];
	int rc;

	rc = otp_ast27xx_init();
	if (rc)
		return OTP_USAGE;

	if (offset + w_count > range)
		return OTP_USAGE;

	OTP_INF("ROM_REGION: 0x%x~0x%x\n", offset, offset + w_count);
	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(ROM_REGION_START_ADDR + i, &ret[0]);
		if (rc)
			return rc;

		if (i % 8 == 0)
			OTP_INF("\n%03X: %04X ", i * 2, ret[0]);
		else
			OTP_INF("%04X ", ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_rbp(uint32_t offset, int w_count)
{
	int range = OTP_RBP_REGION_SIZE;
	uint16_t ret[1];
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(RBP_REGION_START_ADDR + i, ret);
		if (rc)
			return rc;

		OTP_INF("OTPRBP0x%X: 0x%04X\n", i, ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_conf(uint32_t offset, int w_count)
{
	int range = OTP_CONF_REGION_SIZE;
	uint16_t ret[1];
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(CONF_REGION_START_ADDR + i, ret);
		if (rc)
			return rc;

		OTP_INF("OTPCFG0x%X: 0x%04X\n", i, ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_strap(uint32_t offset, int w_count)
{
	int range = 12;	/* 32-bit * 6 / 16 (per word) */
	uint16_t ret[1];
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(STRAP_REGION_START_ADDR + 2 + i, ret);
		if (rc)
			return rc;

		OTP_INF("OTPSTRAP0x%X: 0x%04X\n", i, ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_strap_pro(uint32_t offset, int w_count)
{
	int range = 2;	/* 32-bit / 16 (per word) */
	uint16_t ret[1];
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(STRAP_REGION_START_ADDR + i, ret);
		if (rc)
			return rc;

		OTP_INF("OTPSTRAP_PRO0x%X: 0x%04X\n", i, ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_strap_ext(uint32_t offset, int w_count)
{
	int range = (OTP_STRAP_EXT_REGION_SIZE) / 2;
	uint16_t ret[1];
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(STRAPEXT_REGION_START_ADDR + i, ret);
		if (rc)
			return rc;

		OTP_INF("OTPSTRAPEXT0x%X: 0x%04X\n", i, ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_strap_ext_valid(uint32_t offset, int w_count)
{
	int range = (OTP_STRAP_EXT_REGION_SIZE) / 2;
	uint16_t ret[1];
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(STRAPEXT_REGION_START_ADDR + 0x8 + i, ret);
		if (rc)
			return rc;

		OTP_INF("OTPSTRAPEXT_VLD0x%X: 0x%04X\n", i, ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_user_data(uint32_t offset, int w_count)
{
	int range = OTP_USER_REGION_SIZE;
	uint16_t ret[1];
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	OTP_INF("User Region: 0x%x~0x%x\n", offset, offset + w_count);
	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(USER_REGION_START_ADDR + i, &ret[0]);
		if (rc)
			return rc;

		if (i % 8 == 0)
			OTP_INF("\n%03X: %04X ", i * 2, ret[0]);
		else
			OTP_INF("%04X ", ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_sec_data(uint32_t offset, int w_count)
{
	int range = OTP_SEC_REGION_SIZE;
	uint16_t ret;
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	OTP_INF("Secure Region: 0x%x~0x%x\n", offset, offset + w_count);
	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(SEC_REGION_START_ADDR + i, &ret);
		if (rc)
			return rc;

		if (i % 8 == 0)
			OTP_INF("\n%03X: %04X ", i * 2, ret);
		else
			OTP_INF("%04X ", ret);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_cptra(uint32_t offset, int w_count)
{
	int range = OTP_CAL_REGION_SIZE;
	uint16_t ret[1];
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	OTP_INF("Caliptra Region: 0x%x~0x%x\n", offset, offset + w_count);
	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(CAL_REGION_START_ADDR + i, &ret[0]);
		if (rc)
			return rc;

		if (i % 8 == 0)
			OTP_INF("\n%03X: %04X ", i * 2, ret[0]);
		else
			OTP_INF("%04X ", ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

int otp_print_puf(uint32_t offset, int w_count)
{
	int range = OTP_PUF_REGION_SIZE;
	uint16_t ret[1];
	int rc;

	otp_ast27xx_init();

	if (offset + w_count > range)
		return OTP_USAGE;

	OTP_INF("PUF: 0x%x~0x%x\n", offset, offset + w_count);
	for (int i = offset; i < offset + w_count; i++) {
		rc = lib_otp_read(SW_PUF_REGION_START_ADDR + i, &ret[0]);
		if (rc)
			return rc;

		if (i % 8 == 0)
			OTP_INF("\n%03X: %04X ", i * 2, ret[0]);
		else
			OTP_INF("%04X ", ret[0]);
	}
	OTP_INF("\n");

	return OTP_SUCCESS;
}

static void otp_strap_status(struct otpstrap_status *otpstrap)
{
	int strap_start, strap_end;
	uint16_t data[2];
	int ret;

	/* Initial otpstrap */
	for (int i = 0; i < 32; i++) {
		otpstrap[i].value = 0;
		otpstrap[i].remain_times = 6;
		otpstrap[i].writeable_option = -1;
		otpstrap[i].protected = 0;
	}

	/* Check OTP strap value */
	strap_start = 2;
	strap_end = 2 + 12;

	for (int i = strap_start; i < strap_end; i += 2) {
		int option = (i - strap_start) / 2;

		otp_read_strap(i, &data[0]);
		otp_read_strap(i + 1, &data[1]);

		for (int j = 0; j < 16; j++) {
			char bit_value = ((data[0] >> j) & 0x1);

			if (bit_value == 0 && otpstrap[j].writeable_option == -1)
				otpstrap[j].writeable_option = option;
			if (bit_value == 1)
				otpstrap[j].remain_times--;
			otpstrap[j].value ^= bit_value;
			otpstrap[j].option_value[option] = bit_value;
		}

		for (int j = 16; j < 32; j++) {
			char bit_value = ((data[1] >> (j - 16)) & 0x1);

			if (bit_value == 0 && otpstrap[j].writeable_option == -1)
				otpstrap[j].writeable_option = option;
			if (bit_value == 1)
				otpstrap[j].remain_times--;
			otpstrap[j].value ^= bit_value;
			otpstrap[j].option_value[option] = bit_value;
		}
	}

	/* Check OTP strap write protect */
	ret = otp_read_strap(0, &data[0]);
	ret += otp_read_strap(1, &data[1]);
	if (ret)
		OTP_INF("OTP read strap failed, ret=0x%x\n", ret);

	for (int j = 0; j < 16; j++) {
		if (((data[0] >> j) & 0x1) == 1)
			otpstrap[j].protected = 1;
	}

	for (int j = 16; j < 32; j++) {
		if (((data[1] >> (j - 16)) & 0x1) == 1)
			otpstrap[j].protected = 1;
	}

#ifdef DEBUG
	for (int i = 0; i < 32; i++) {
		OTP_INF("%s[%d]: value:%d, %s:%d, writeable_option:%d, protected:%d\n",
			"otpstrap", i, otpstrap[i].value,
			"remain_times", otpstrap[i].remain_times,
			otpstrap[i].writeable_option, otpstrap[i].protected);

		OTP_INF("option_value: ");
		for (int j = 0; j < 6; j++)
			OTP_INF("%d ", otpstrap[i].option_value[j]);
		OTP_INF("\n");
	}
#endif
}

int otp_print_rbp_info(void)
{
	const struct otprbp_info *rbp_info = info_cb.rbp_info;
	uint16_t OTPRBP[21];
	uint32_t w_offset;
	uint32_t length;

	otp_ast27xx_init();

	for (int i = 0; i < 21; i++)
		otp_read_rbp(i, &OTPRBP[i]);

	OTP_INF("W   bit-length            Description                       Value\n");
	OTP_INF("__________________________________________________________________________\n");
	for (int i = 0; i < info_cb.rbp_info_len; i++) {
		w_offset = rbp_info[i].w_offset;
		length = rbp_info[i].length;

		OTP_INF("0x%-4X", w_offset);
		OTP_INF("0x%-9X", length);
		OTP_INF("%-40s: ", rbp_info[i].information);

		for (int j = 0; j < (length + 15) / 16; j++)
			OTP_INF("0x%04x ", OTPRBP[w_offset + j]);
		OTP_INF("\n");
	}

	return OTP_SUCCESS;
}

int otp_print_conf_info(void)
{
	const struct otpconf_info *conf_info = info_cb.conf_info;
	uint16_t OTPCFG[32];
	uint32_t mask;
	uint32_t w_offset;
	uint32_t bit_offset;
	uint32_t otp_value;

	otp_ast27xx_init();

	for (int i = 0; i < 32; i++)
		otp_read_conf(i, &OTPCFG[i]);

	OTP_INF("W    BIT        Value       Description\n");
	OTP_INF("__________________________________________________________________________\n");
	for (int i = 0; i < info_cb.conf_info_len; i++) {
		w_offset = conf_info[i].w_offset;
		bit_offset = conf_info[i].bit_offset;
		mask = BIT(conf_info[i].length) - 1;
		otp_value = (OTPCFG[w_offset] >> bit_offset) & mask;

		if (otp_value != conf_info[i].value &&
		    conf_info[i].value != OTP_REG_RESERVED &&
		    conf_info[i].value != OTP_REG_VALUE)
			continue;
		OTP_INF("0x%-4X", w_offset);

		if (conf_info[i].length == 1) {
			OTP_INF("0x%-9X", conf_info[i].bit_offset);
		} else {
			OTP_INF("0x%-2X:0x%-4X",
				conf_info[i].bit_offset + conf_info[i].length - 1,
				conf_info[i].bit_offset);
		}
		OTP_INF("0x%-10x", otp_value);

		if (conf_info[i].value == OTP_REG_RESERVED) {
			OTP_INF("Reserved\n");
		} else if (conf_info[i].value == OTP_REG_VALUE) {
			OTP_INF("%s: 0x%x\n", conf_info[i].information, otp_value);
			OTP_INF("\n");
		} else {
			OTP_INF("%s\n", conf_info[i].information);
		}
	}

	return OTP_SUCCESS;
}

void otp_print_strap_info(void)
{
	const struct otpstrap_info *strap_info = info_cb.strap_info;
	struct otpstrap_status strap_status[32];
	uint32_t bit_offset;
	uint32_t length;
	uint32_t otp_value;
	uint32_t otp_protect;

	otp_ast27xx_init();

	otp_strap_status(strap_status);

	OTP_INF("BIT(hex) Value  Remains  Protect   Description\n");
	OTP_INF("_____________________________________________________________________________\n");

	for (int i = 0; i < info_cb.strap_info_len; i++) {
		otp_value = 0;
		otp_protect = 0;
		bit_offset = strap_info[i].bit_offset;
		length = strap_info[i].length;
		for (int j = 0; j < length; j++) {
			otp_value |= strap_status[bit_offset + j].value << j;
			otp_protect |= strap_status[bit_offset + j].protected << j;
		}

		if (otp_value != strap_info[i].value &&
		    strap_info[i].value != OTP_REG_RESERVED)
			continue;

		for (int j = 0; j < length; j++) {
			OTP_INF("0x%-7X", strap_info[i].bit_offset + j);
			OTP_INF("0x%-5X", strap_status[bit_offset + j].value);
			OTP_INF("%-9d", strap_status[bit_offset + j].remain_times);
			OTP_INF("0x%-7X", strap_status[bit_offset + j].protected);
			if (strap_info[i].value == OTP_REG_RESERVED) {
				OTP_INF(" Reserved\n");
				continue;
			}

			if (length == 1) {
				OTP_INF(" %s\n", strap_info[i].information);
				continue;
			}

			if (j == 0)
				OTP_INF("/%s\n", strap_info[i].information);
			else if (j == length - 1)
				OTP_INF("\\ \"\n");
			else
				OTP_INF("| \"\n");
		}
	}
}

void otp_print_strap_ext_info(void)
{
	const struct otpstrap_ext_info *strap_ext_info = info_cb.strap_ext_info;
	uint32_t bit_offset;
	uint32_t otp_value, otp_vld;
	uint32_t length;
	uint16_t data[8];
	uint16_t vld[8];

	otp_ast27xx_init();

	/* Read Flash strap */
	for (int i = 0; i < 8; i++)
		otp_read_strap_ext(i, &data[i]);

	/* Read Flash strap valid */
	for (int i = 0; i < 8; i++)
		otp_read_strap_ext_vld(i, &vld[i]);

	OTP_INF("BIT(hex) Value  Valid   Description\n");
	OTP_INF("_____________________________________________________________________________\n");

	for (int i = 0; i < info_cb.strap_ext_info_len; i++) {
		otp_value = 0;
		otp_vld = 0;
		bit_offset = strap_ext_info[i].bit_offset;
		length = strap_ext_info[i].length;

		int w_offset = bit_offset / 16;
		int b_offset = bit_offset % 16;

		otp_value = (data[w_offset] >> b_offset) &
			    GENMASK(length - 1, 0);
		otp_vld = (vld[w_offset] >> b_offset) &
			  GENMASK(length - 1, 0);

		if (otp_value != strap_ext_info[i].value)
			continue;

		for (int j = 0; j < length; j++) {
			OTP_INF("0x%-7X", strap_ext_info[i].bit_offset + j);
			OTP_INF("0x%-5lX", (otp_value & BIT(j)) >> j);
			OTP_INF("0x%-5lX", (otp_vld & BIT(j)) >> j);

			if (length == 1) {
				OTP_INF(" %s\n", strap_ext_info[i].information);
				continue;
			}

			if (j == 0)
				OTP_INF("/%s\n", strap_ext_info[i].information);
			else if (j == length - 1)
				OTP_INF("\\ \"\n");
			else
				OTP_INF("| \"\n");
		}
	}
}

static int _otp_print_key(uint32_t header, uint32_t offset, uint8_t *data)
{
	const struct otpkey_type *key_info_array = info_cb.key_info;
	struct otpkey_type key_info = { .value = -1 };
	int key_id, key_w_offset, key_offset, key_type;
	int last;
	int i;

	if (!header)
		return -1;

	key_id = OTP_KH_KEY_ID(header);
	key_w_offset = OTP_KH_OFFSET(header);
	key_offset = key_w_offset * 2;
	key_type = OTP_KH_KEY_TYPE(header);
	last = OTP_KH_LAST(header);

	OTP_INF("\nKey[%d]:\n", offset);
	OTP_INF("Header: %x\n", header);

	for (i = 0; i < info_cb.key_info_len; i++) {
		if (key_type == key_info_array[i].value) {
			key_info = key_info_array[i];
			break;
		}
	}

	if (i == info_cb.key_info_len) {
		OTP_INF("Error: Cannot find the key type\n");
		return -1;
	}

	OTP_INF("Key Type: ");
	OTP_INF("%s\n", key_info.information);
	OTP_INF("Key Number ID: %d\n", key_id);
	OTP_INF("Key Word Offset: 0x%x\n", key_w_offset);
	if (last)
		OTP_INF("This is the last key\n");

	if (!data)
		return -1;

	OTP_INF("Key Value:\n");
	if (key_info.key_type == SOC_ECDSA_PUB) {
		OTP_INF("Q.x:\n");
		buf_print(&data[key_offset], 0x30);
		OTP_INF("Q.y:\n");
		buf_print(&data[key_offset + 0x30], 0x30);

	} else if (key_info.key_type == SOC_LMS_PUB) {
		OTP_INF("tree_type:\n");
		buf_print(&data[key_offset], 0x4);
		OTP_INF("otstype:\n");
		buf_print(&data[key_offset + 0x4], 0x4);
		OTP_INF("id:\n");
		buf_print(&data[key_offset + 0x8], 0x10);
		OTP_INF("digest:\n");
		buf_print(&data[key_offset + 0x18], 0x18);

	} else if (key_info.key_type == CAL_MANU_PUB_HASH) {
		buf_print(&data[key_offset], 0x30);
		OTP_INF("Manufacture ECC Key Mask: 0x%x\n", data[key_offset + 0x30]);
		OTP_INF("Manufacture LMS Key Mask: 0x%x\n", data[key_offset + 0x34]);

	} else if (key_info.key_type == CAL_OWN_PUB_HASH) {
		buf_print(&data[key_offset], 0x30);

	} else if (key_info.key_type == SOC_VAULT || key_info.key_type == SOC_VAULT_SEED) {
		buf_print(&data[key_offset], 0x20);
	}

	return 0;
}

static void otp_print_key(uint32_t *data)
{
	uint8_t *byte_buf;
	int empty;
	int ret;

	byte_buf = (uint8_t *)data;
	empty = 1;

	for (int i = 0; i < OTP_KH_NUM; i++) {
		if (data[i] != 0)
			empty = 0;
	}

	if (empty) {
		OTP_INF("OTP data header is empty\n");
		return;
	}

	for (int i = 0; i < OTP_KH_NUM; i++)
		ret = _otp_print_key(data[i], i, byte_buf);
}

void otp_print_key_info(void)
{
	uint16_t buf[OTP_SEC_REGION_SIZE];

	otp_ast27xx_init();

	otp_read_secure_multi(0, buf, OTP_SEC_REGION_SIZE);
	otp_print_key((uint32_t *)buf);
}

int otp_print_ver(void)
{
	OTP_INF("OTP tool version: %s\n", OTP_VER);
	OTP_INF("OTP info version: %s\n", OTP_INFO_VER);

	return CMD_RET_SUCCESS;
}

static int otp_ast27xx_init(void)
{
	union otp_pro_sts *pro_sts;
	uint16_t otp_conf0;
	uint32_t ver;
	int ret;

	if (is_otp_dev_ready())
		return 0;

	ret = otp_dev_init();
	if (ret)
		return ret;

	otp_get_chip_version(otp_dev, &ver);
	switch (ver) {
	case OTP_AST2700_A1:
		OTP_INF("Chip: AST2700-A1\n");
		/* AST2700-A1 OTP tables dropped to reduce code size; unsupported on this build */
#if defined(OTP_AST2700_INFO_UTILITY)
		info_cb.version = OTP_AST2700_A1;
		info_cb.rbp_info = a1_rbp_info;
		info_cb.rbp_info_len = ARRAY_SIZE(a1_rbp_info);
		info_cb.conf_info = a1_conf_info;
		info_cb.conf_info_len = ARRAY_SIZE(a1_conf_info);
		info_cb.strap_info = a1_strap_info;
		info_cb.strap_info_len = ARRAY_SIZE(a1_strap_info);
		info_cb.strap_ext_info = a1_strap_ext_info;
		info_cb.strap_ext_info_len = ARRAY_SIZE(a1_strap_ext_info);
		info_cb.cal_info = a1_cal_info;
		info_cb.cal_info_len = ARRAY_SIZE(a1_cal_info);
		info_cb.key_info = a1_key_type;
		info_cb.key_info_len = ARRAY_SIZE(a1_key_type);
#endif
		break;
	case OTP_AST2700_A2:
		OTP_INF("Chip: AST2700-A2\n");
		/* AST2700-A2 OTP tables dropped to reduce code size; unsupported on this build */
#if defined(OTP_AST2700_INFO_UTILITY)
		info_cb.version = OTP_AST2700_A2;
		info_cb.rbp_info = a2_rbp_info;
		info_cb.rbp_info_len = ARRAY_SIZE(a2_rbp_info);
		info_cb.conf_info = a2_conf_info;
		info_cb.conf_info_len = ARRAY_SIZE(a2_conf_info);
		info_cb.strap_info = a2_strap_info;
		info_cb.strap_info_len = ARRAY_SIZE(a2_strap_info);
		info_cb.strap_ext_info = a2_strap_ext_info;
		info_cb.strap_ext_info_len = ARRAY_SIZE(a2_strap_ext_info);
		info_cb.cal_info = a2_cal_info;
		info_cb.cal_info_len = ARRAY_SIZE(a2_cal_info);
		info_cb.key_info = a2_key_type;
		info_cb.key_info_len = ARRAY_SIZE(a2_key_type);
#endif
		break;
	default:
		OTP_INF("SOC is not supported\n");
		return CMD_RET_FAILURE;
	}

	otp_read_conf(0, &otp_conf0);
	pro_sts = &info_cb.pro_sts;
	pro_sts->value = otp_conf0;

	return ret;
}
