/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_otp

#include <device.h>
#include <soc.h>
#include <stdio.h>
#include <logging/log.h>
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include <drivers/misc/aspeed/otp_aspeed.h>
#include <drivers/misc/aspeed/otp.h>
#include "otp_info_10x0.h"
LOG_MODULE_REGISTER(otp_aspeed, CONFIG_LOG_DEFAULT_LEVEL);

#define OTP_VER				"2.1.0"
#define OTP_PASSWD			0x349fe38a

/****************************
 *                          *
 * OTP register definitions *
 *                          *
 ****************************/
#define OTP_PROTECT_KEY			0x0
#define OTP_COMMAND			0x4
#define OTP_TIMING			0x8
#define OTP_ADDR			0x10
#define OTP_STATUS			0x14
#define OTP_COMPARE_1			0x20
#define OTP_COMPARE_2			0x24
#define OTP_COMPARE_3			0x28
#define OTP_COMPARE_4			0x2c
#define SW_REV_ID0			0x68
#define SW_REV_ID1			0x6c
#define SEC_KEY_NUM			0x78

/******************************************************************************/

#define ASPEED_REVISION_ID0		0x7e6e2004
#define ASPEED_REVISION_ID1		0x7e6e2014

/*
 * Maximum commands number
 * otcfg:    16 (16 * 3)
 * otpstrap: 24 (24 * 2)
 */
#define OTP_DT_OTPCFG_GROUP_NUM		3
#define OTP_DT_OTPSTRAP_GROUP_NUM	2
#define OTP_DT_SETTING_NUM_MAX		48

static uintptr_t otp_base;
#define OTP_RD(reg)             sys_read32(otp_base + (reg))
#define OTP_WR(val, reg)        sys_write32(val, otp_base + (reg))

struct otp_aspeed_data {
	struct otp_info_cb info_cb;
};

struct otp_aspeed_config {
	uintptr_t base;
	uint32_t otpcfg_dt_setting_list[OTP_DT_SETTING_NUM_MAX];
	int otpcfg_dt_setting_num;
	uint32_t otpstrap_dt_setting_list[OTP_DT_SETTING_NUM_MAX];
	int otpstrap_dt_setting_num;
};

static struct otp_aspeed_data otp_aspeed_data;
static struct otpstrap_status strap_status[64];

static const struct otpkey_type ast10xxa0_key_type[] = {
	{
		1, OTP_KEY_TYPE_VAULT, 0,
		"AES-256 as secret vault key"
	},
	{
		2, OTP_KEY_TYPE_AES,   1,
		"AES-256 as OEM platform key for image encryption/decryption in Mode 2 or AES-256 as OEM DSS keys for Mode GCM"
	},
	{
		8, OTP_KEY_TYPE_RSA_PUB,   1,
		"RSA-public as OEM DSS public keys in Mode 2"
	},
	{
		10, OTP_KEY_TYPE_RSA_PUB,  0,
		"RSA-public as AES key decryption key"
	},
	{
		14, OTP_KEY_TYPE_RSA_PRIV,  0,
		"RSA-private as AES key decryption key"
	},
};

static const struct otpkey_type ast10xxa1_key_type[] = {
	{
		1, OTP_KEY_TYPE_VAULT, 0,
		"AES-256 as secret vault key"
	},
	{
		2, OTP_KEY_TYPE_AES,   1,
		"AES-256 as OEM platform key for image encryption/decryption in Mode 2 or AES-256 as OEM DSS keys for Mode GCM"
	},
	{
		8, OTP_KEY_TYPE_RSA_PUB,   1,
		"RSA-public as OEM DSS public keys in Mode 2"
	},
	{
		9, OTP_KEY_TYPE_RSA_PUB,   1,
		"RSA-public as OEM DSS public keys in Mode 2(big endian)"
	},
	{
		10, OTP_KEY_TYPE_RSA_PUB,  0,
		"RSA-public as AES key decryption key"
	},
	{
		11, OTP_KEY_TYPE_RSA_PUB,  0,
		"RSA-public as AES key decryption key(big endian)"
	},
	{
		12, OTP_KEY_TYPE_RSA_PRIV,  0,
		"RSA-private as AES key decryption key"
	},
	{
		13, OTP_KEY_TYPE_RSA_PRIV,  0,
		"RSA-private as AES key decryption key(big endian)"
	},
	{
		5, OTP_KEY_ECDSA384P,  0,
		"ECDSA384 cure parameter"
	},
	{
		7, OTP_KEY_ECDSA384,  0,
		"ECDSA-public as OEM DSS public keys"
	}
};

static int sb_sha256(uint8_t *src, uint32_t len, uint8_t *digest_ret)
{
#ifdef CONFIG_MBEDTLS_MAC_SHA256_ENABLED
	mbedtls_sha256(src, len, digest_ret, 0);
	return 0;
#else
	LOG_ERR("%s: not supported", __func__);
	return -1;
#endif
}

static int sb_sha384(uint8_t *src, uint32_t len, uint8_t *digest_ret)
{
#ifdef CONFIG_MBEDTLS_MAC_SHA512_ENABLED
	mbedtls_sha512(src, len, digest_ret, 1);
	return 0;
#else
	LOG_ERR("%s: not supported", __func__);
	return -1;
#endif
}

static uint32_t chip_version(void)
{
	uint32_t revid0, revid1;

	revid0 = sys_read32(ASPEED_REVISION_ID0);
	revid1 = sys_read32(ASPEED_REVISION_ID1);

	if (revid0 == ID0_AST1030A0 && revid1 == ID1_AST1030A0) {
		/* AST1030-A0 */
		return OTP_AST1030A0;
	} else if (revid0 == ID0_AST1030A1 && revid1 == ID1_AST1030A1) {
		/* AST1030-A1 */
		return OTP_AST1030A1;
	} else if (revid0 == ID0_AST1060A1 && revid1 == ID1_AST1060A1) {
		/* AST1060-A1 */
		return OTP_AST1060A1;
	} else if ((revid0 == ID0_AST1060A2 && revid1 == ID1_AST1060A2) ||
		(revid0 == ID0_AST1060A2_ENG && revid1 == ID1_AST1060A2_ENG)) {
		/* AST1060-A1 */
		return OTP_AST1060A2;
	}
	return OTP_FAILURE;
}

static void wait_complete(void)
{
	int reg;

	k_usleep(100);
	do {
		reg = OTP_RD(OTP_STATUS);
	} while ((reg & 0x6) != 0x6);
}

static void otp_write(uint32_t otp_addr, uint32_t data)
{
	OTP_WR(otp_addr, OTP_ADDR); /* write address */
	OTP_WR(data, OTP_COMPARE_1); /* write data */
	OTP_WR(0x23b1e362, OTP_COMMAND); /* write command */
	wait_complete();
}

static void otp_soak(int soak)
{
	switch (soak) {
	case 0: /* default */
		otp_write(0x3000, 0x0); /* Write MRA */
		otp_write(0x5000, 0x0); /* Write MRB */
		otp_write(0x1000, 0x0); /* Write MR */
		break;
	case 1: /* normal program */
		otp_write(0x3000, 0x1320); /* Write MRA */
		otp_write(0x5000, 0x1008); /* Write MRB */
		otp_write(0x1000, 0x0024); /* Write MR */
		OTP_WR(0x04191388, OTP_TIMING); /* 200us */
		break;
	case 2: /* soak program */
		otp_write(0x3000, 0x1320); /* Write MRA */
		otp_write(0x5000, 0x0007); /* Write MRB */
		otp_write(0x1000, 0x0100); /* Write MR */
		OTP_WR(0x04193a98, OTP_TIMING); /* 600us */
		break;
	}
	wait_complete();
}

static void otp_read_data(uint32_t offset, uint32_t *data)
{
	OTP_WR(offset, OTP_ADDR); /* Read address */
	OTP_WR(0x23b1e361, OTP_COMMAND); /* trigger read */
	wait_complete();
	data[0] = OTP_RD(OTP_COMPARE_1);
	data[1] = OTP_RD(OTP_COMPARE_2);
}

static void otp_read_conf(uint32_t offset, uint32_t *data)
{
	int config_offset;

	config_offset = 0x800;
	config_offset |= (offset / 8) * 0x200;
	config_offset |= (offset % 8) * 0x2;

	OTP_WR(config_offset, OTP_ADDR);  /* Read address */
	OTP_WR(0x23b1e361, OTP_COMMAND); /* trigger read */
	wait_complete();
	data[0] = OTP_RD(OTP_COMPARE_1);
}

static int verify_bit(uint32_t otp_addr, int bit_offset, int value)
{
	uint32_t ret[2];
	uint32_t addr;

	if (otp_addr % 2 == 0)
		addr = otp_addr;
	else
		addr = otp_addr - 1;

	otp_read_data(addr, ret);

	if (otp_addr % 2 == 0) {
		if (((ret[0] >> bit_offset) & 1) == value)
			return OTP_SUCCESS;
		else
			return OTP_FAILURE;
	} else {
		if (((ret[1] >> bit_offset) & 1) == value)
			return OTP_SUCCESS;
		else
			return OTP_FAILURE;
	}
}

static uint32_t verify_dw(uint32_t otp_addr, uint32_t *value, uint32_t *ignore,
			  uint32_t *compare, int size)
{
	uint32_t ret[2];
	uint32_t addr;

	otp_addr &= ~(1 << 15);

	if (otp_addr % 2 == 0)
		addr = otp_addr;
	else
		addr = otp_addr - 1;

	otp_read_data(addr, ret);

	if (size == 1) {
		if (otp_addr % 2 == 0) {
			if ((value[0] & ~ignore[0]) == (ret[0] & ~ignore[0])) {
				compare[0] = 0;
				return OTP_SUCCESS;
			}
			compare[0] = value[0] ^ ret[0];
			return OTP_FAILURE;

		} else {
			if ((value[0] & ~ignore[0]) == (ret[1] & ~ignore[0])) {
				compare[0] = ~0;
				return OTP_SUCCESS;
			}
			compare[0] = ~(value[0] ^ ret[1]);
			return OTP_FAILURE;
		}
	} else if (size == 2) {
		/* otp_addr should be even */
		if ((value[0] & ~ignore[0]) == (ret[0] & ~ignore[0]) &&
			(value[1] & ~ignore[1]) == (ret[1] & ~ignore[1])) {
			compare[0] = 0;
			compare[1] = ~0;
			return OTP_SUCCESS;
		}
		compare[0] = value[0] ^ ret[0];
		compare[1] = ~(value[1] ^ ret[1]);
		return OTP_FAILURE;
	} else {
		return OTP_FAILURE;
	}
}

static void otp_prog(uint32_t otp_addr, uint32_t prog_bit)
{
	otp_write(0x0, prog_bit);
	OTP_WR(otp_addr, OTP_ADDR); /* write address */
	OTP_WR(prog_bit, OTP_COMPARE_1); /* write data */
	OTP_WR(0x23b1e364, OTP_COMMAND); /* write command */
	wait_complete();
}

static void _otp_prog_bit(uint32_t value, uint32_t prog_address, uint32_t bit_offset)
{
	int prog_bit;

	if (prog_address % 2 == 0) {
		if (value)
			prog_bit = ~(0x1 << bit_offset);
		else
			return;
	} else {
		if (!value)
			prog_bit = 0x1 << bit_offset;
		else
			return;
	}
	otp_prog(prog_address, prog_bit);
}

static int otp_prog_dc_b(uint32_t value, uint32_t prog_address, uint32_t bit_offset)
{
	int pass;
	int i;

	otp_soak(1);
	_otp_prog_bit(value, prog_address, bit_offset);
	pass = 0;

	for (i = 0; i < RETRY; i++) {
		if (verify_bit(prog_address, bit_offset, value) != 0) {
			otp_soak(2);
			_otp_prog_bit(value, prog_address, bit_offset);
			if (verify_bit(prog_address, bit_offset, value) != 0) {
				otp_soak(1);
			} else {
				pass = 1;
				break;
			}
		} else {
			pass = 1;
			break;
		}
	}
	if (pass)
		return OTP_SUCCESS;

	return OTP_FAILURE;
}

static void otp_prog_dw(uint32_t value, uint32_t ignore, uint32_t prog_address)
{
	int j, bit_value, prog_bit;

	for (j = 0; j < 32; j++) {
		if ((ignore >> j) & 0x1)
			continue;
		bit_value = (value >> j) & 0x1;
		if (prog_address % 2 == 0) {
			if (bit_value)
				prog_bit = ~(0x1 << j);
			else
				continue;
		} else {
			if (bit_value)
				continue;
			else
				prog_bit = 0x1 << j;
		}
		otp_prog(prog_address, prog_bit);
	}
}

static int otp_prog_verify_2dw(uint32_t *data, uint32_t *buf,
			       uint32_t *ignore_mask, uint32_t prog_address)
{
	uint32_t data0_masked;
	uint32_t data1_masked;
	uint32_t buf0_masked;
	uint32_t buf1_masked;
	uint32_t compare[2];
	int pass;
	int i;

	data0_masked = data[0]  & ~ignore_mask[0];
	buf0_masked  = buf[0] & ~ignore_mask[0];
	data1_masked = data[1]  & ~ignore_mask[1];
	buf1_masked  = buf[1] & ~ignore_mask[1];
	if (data0_masked == buf0_masked && data1_masked == buf1_masked)
		return OTP_SUCCESS;

	for (i = 0; i < 32; i++) {
		if (((data0_masked >> i) & 0x1) == 1 && ((buf0_masked >> i) & 0x1) == 0)
			return OTP_FAILURE;
		if (((data1_masked >> i) & 0x1) == 0 && ((buf1_masked >> i) & 0x1) == 1)
			return OTP_FAILURE;
	}

	otp_soak(1);
	if (data0_masked != buf0_masked)
		otp_prog_dw(buf[0], ignore_mask[0], prog_address);
	if (data1_masked != buf1_masked)
		otp_prog_dw(buf[1], ignore_mask[1], prog_address + 1);

	pass = 0;
	for (i = 0; i < RETRY; i++) {
		if (verify_dw(prog_address, buf, ignore_mask, compare, 2) != 0) {
			otp_soak(2);
			if (compare[0] != 0)
				otp_prog_dw(compare[0], ignore_mask[0], prog_address);
			if (compare[1] != ~0)
				otp_prog_dw(compare[1], ignore_mask[1], prog_address + 1);
			if (verify_dw(prog_address, buf, ignore_mask, compare, 2) != 0) {
				otp_soak(1);
			} else {
				pass = 1;
				break;
			}
		} else {
			pass = 1;
			break;
		}
	}

	if (!pass) {
		otp_soak(0);
		return OTP_FAILURE;
	}
	return OTP_SUCCESS;
}

static void otp_strap_status(struct otpstrap_status *os)
{
	uint32_t OTPSTRAP_RAW[2];
	int strap_end;
	int i, j;

	for (j = 0; j < 64; j++) {
		os[j].value = 0;
		os[j].remain_times = 6;
		os[j].writeable_option = -1;
		os[j].protected = 0;
	}
	strap_end = 28;

	otp_soak(0);
	for (i = 16; i < strap_end; i += 2) {
		int option = (i - 16) / 2;

		otp_read_conf(i, &OTPSTRAP_RAW[0]);
		otp_read_conf(i + 1, &OTPSTRAP_RAW[1]);
		for (j = 0; j < 32; j++) {
			char bit_value = ((OTPSTRAP_RAW[0] >> j) & 0x1);

			if (bit_value == 0 && os[j].writeable_option == -1)
				os[j].writeable_option = option;
			if (bit_value == 1)
				os[j].remain_times--;
			os[j].value ^= bit_value;
			os[j].option_array[option] = bit_value;
		}
		for (j = 32; j < 64; j++) {
			char bit_value = ((OTPSTRAP_RAW[1] >> (j - 32)) & 0x1);

			if (bit_value == 0 && os[j].writeable_option == -1)
				os[j].writeable_option = option;
			if (bit_value == 1)
				os[j].remain_times--;
			os[j].value ^= bit_value;
			os[j].option_array[option] = bit_value;
		}
	}

	otp_read_conf(30, &OTPSTRAP_RAW[0]);
	otp_read_conf(31, &OTPSTRAP_RAW[1]);
	for (j = 0; j < 32; j++) {
		if (((OTPSTRAP_RAW[0] >> j) & 0x1) == 1)
			os[j].protected = 1;
	}
	for (j = 32; j < 64; j++) {
		if (((OTPSTRAP_RAW[1] >> (j - 32)) & 0x1) == 1)
			os[j].protected = 1;
	}
}

static int otp_prog_strap_b(int bit_offset, int value)
{
	uint32_t prog_address;
	int offset;

	otp_strap_status(strap_status);

	prog_address = 0x800;
	if (bit_offset < 32) {
		offset = bit_offset;
		prog_address |= ((strap_status[bit_offset].writeable_option * 2 + 16) / 8) * 0x200;
		prog_address |= ((strap_status[bit_offset].writeable_option * 2 + 16) % 8) * 0x2;

	} else {
		offset = (bit_offset - 32);
		prog_address |= ((strap_status[bit_offset].writeable_option * 2 + 17) / 8) * 0x200;
		prog_address |= ((strap_status[bit_offset].writeable_option * 2 + 17) % 8) * 0x2;
	}

	return otp_prog_dc_b(1, prog_address, offset);
}

static int otp_prog_data(struct otp_image_layout *image_layout, uint32_t *data)
{
	uint32_t *buf_ignore;
	uint32_t *buf;
	int ret;
	int i;

	buf = (uint32_t *)image_layout->data;
	buf_ignore = (uint32_t *)image_layout->data_ignore;
	LOG_INF("Start Programing...\n");

	/* programing ecc region first */
	for (i = 1792; i < 2046; i += 2) {
		ret = otp_prog_verify_2dw(&data[i], &buf[i], &buf_ignore[i], i);
		if (ret != OTP_SUCCESS) {
			LOG_ERR("address: %08x, data: %08x %08x, buffer: %08x %08x, mask: %08x %08x\n",
				i, data[i], data[i + 1], buf[i], buf[i + 1],
				buf_ignore[i], buf_ignore[i + 1]);
			return ret;
		}
	}

	for (i = 0; i < 1792; i += 2) {
		ret = otp_prog_verify_2dw(&data[i], &buf[i], &buf_ignore[i], i);
		if (ret != OTP_SUCCESS) {
			LOG_ERR("address: %08x, data: %08x %08x, buffer: %08x %08x, mask: %08x %08x\n",
				i, data[i], data[i + 1], buf[i], buf[i + 1],
				buf_ignore[i], buf_ignore[i + 1]);
			return ret;
		}
	}
	otp_soak(0);

	return OTP_SUCCESS;
}

static int otp_prog_strap(struct otp_image_layout *image_layout,
			  struct otpstrap_status *os)
{
	uint32_t *strap;
	uint32_t *strap_ignore;
	uint32_t *strap_pro;
	uint32_t prog_address;
	int bit, pbit, ibit, offset;
	int prog_flag = 0;
	int fail = 0;
	int ret;
	int i;

	strap = (uint32_t *)image_layout->strap;
	strap_pro = (uint32_t *)image_layout->strap_pro;
	strap_ignore = (uint32_t *)image_layout->strap_ignore;

	for (i = 0; i < 64; i++) {
		prog_address = 0x800;
		if (i < 32) {
			offset = i;
			bit = (strap[0] >> offset) & 0x1;
			ibit = (strap_ignore[0] >> offset) & 0x1;
			pbit = (strap_pro[0] >> offset) & 0x1;
			prog_address |= ((os[i].writeable_option * 2 + 16) / 8) * 0x200;
			prog_address |= ((os[i].writeable_option * 2 + 16) % 8) * 0x2;

		} else {
			offset = (i - 32);
			bit = (strap[1] >> offset) & 0x1;
			ibit = (strap_ignore[1] >> offset) & 0x1;
			pbit = (strap_pro[1] >> offset) & 0x1;
			prog_address |= ((os[i].writeable_option * 2 + 17) / 8) * 0x200;
			prog_address |= ((os[i].writeable_option * 2 + 17) % 8) * 0x2;
		}

		if (ibit == 1)
			continue;
		if (bit == os[i].value)
			prog_flag = 0;
		else
			prog_flag = 1;

		if (os[i].protected == 1 && prog_flag) {
			fail = 1;
			continue;
		}
		if (os[i].remain_times == 0 && prog_flag) {
			fail = 1;
			continue;
		}

		if (prog_flag) {
			ret = otp_prog_dc_b(1, prog_address, offset);
			if (ret)
				return OTP_FAILURE;
		}

		if (pbit != 0) {
			prog_address = 0x800;
			if (i < 32)
				prog_address |= 0x60c;
			else
				prog_address |= 0x60e;

			ret = otp_prog_dc_b(1, prog_address, offset);
			if (ret)
				return OTP_FAILURE;
		}
	}
	otp_soak(0);
	if (fail == 1)
		return OTP_FAILURE;
	return OTP_SUCCESS;
}

static int otp_prog_conf(struct otp_image_layout *image_layout,
			 uint32_t *otp_conf)
{
	uint32_t *conf_ignore = (uint32_t *)image_layout->conf_ignore;
	uint32_t *conf = (uint32_t *)image_layout->conf;
	uint32_t prog_address;
	uint32_t compare[2];
	uint32_t data_masked;
	uint32_t buf_masked;
	int pass = 0;
	int i, k;

	LOG_INF("Start Programing...\n");

	otp_soak(0);
	for (i = 0; i < 16; i++) {
		data_masked = otp_conf[i]  & ~conf_ignore[i];
		buf_masked  = conf[i] & ~conf_ignore[i];
		prog_address = 0x800;
		prog_address |= (i / 8) * 0x200;
		prog_address |= (i % 8) * 0x2;
		if (data_masked == buf_masked) {
			pass = 1;
			continue;
		}

		otp_soak(1);
		otp_prog_dw(conf[i], conf_ignore[i], prog_address);

		pass = 0;
		for (k = 0; k < RETRY; k++) {
			if (verify_dw(prog_address, &conf[i], &conf_ignore[i], compare, 1) != 0) {
				otp_soak(2);
				otp_prog_dw(compare[0], conf_ignore[i], prog_address);
				if (verify_dw(prog_address, &conf[i], &conf_ignore[i], compare, 1) != 0) {
					otp_soak(1);
				} else {
					pass = 1;
					break;
				}
			} else {
				pass = 1;
				break;
			}
		}
		if (pass == 0) {
			LOG_ERR("address: %08x, otp_conf: %08x, input_conf: %08x, mask: %08x\n",
				i, otp_conf[i], conf[i], conf_ignore[i]);
			break;
		}
	}

	otp_soak(0);
	if (!pass)
		return OTP_FAILURE;

	return OTP_SUCCESS;
}

static int otp_prog_scu_protect(struct otp_image_layout *image_layout,
				uint32_t *scu_pro)
{
	uint32_t *OTPSCU_IGNORE = (uint32_t *)image_layout->scu_pro_ignore;
	uint32_t *OTPSCU = (uint32_t *)image_layout->scu_pro;
	uint32_t prog_address;
	uint32_t compare[2];
	uint32_t data_masked;
	uint32_t buf_masked;
	int pass = 0;
	int i, k;

	LOG_INF("Start Programing...\n");

	otp_soak(0);
	for (i = 0; i < 2; i++) {
		data_masked = scu_pro[i]  & ~OTPSCU_IGNORE[i];
		buf_masked  = OTPSCU[i] & ~OTPSCU_IGNORE[i];
		prog_address = 0xe08 + i * 2;
		if (data_masked == buf_masked) {
			pass = 1;
			continue;
		}

		otp_soak(1);
		otp_prog_dw(OTPSCU[i], OTPSCU_IGNORE[i], prog_address);

		pass = 0;
		for (k = 0; k < RETRY; k++) {
			if (verify_dw(prog_address, &OTPSCU[i], &OTPSCU_IGNORE[i], compare, 1) != 0) {
				otp_soak(2);
				otp_prog_dw(compare[0], OTPSCU_IGNORE[i], prog_address);
				if (verify_dw(prog_address, &OTPSCU[i], &OTPSCU_IGNORE[i], compare, 1) != 0) {
					otp_soak(1);
				} else {
					pass = 1;
					break;
				}
			} else {
				pass = 1;
				break;
			}
		}
		if (pass == 0) {
			LOG_ERR("OTPCFG0x%x: 0x%08x, input: 0x%08x, mask: 0x%08x\n",
				i + 28, scu_pro[i], OTPSCU[i], OTPSCU_IGNORE[i]);
			break;
		}
	}

	otp_soak(0);
	if (!pass)
		return OTP_FAILURE;

	return OTP_SUCCESS;
}

static int otp_verify_image(uint8_t *src_buf, uint32_t length,
			    uint8_t *digest_buf, int version)
{
	uint8_t digest_ret[48];
	int digest_len;
	int ret;

	switch (version) {
	case 1:
		ret = sb_sha256(src_buf, length, digest_ret);
		digest_len = 32;
		break;
	case 2:
		ret = sb_sha384(src_buf, length, digest_ret);
		digest_len = 48;
		break;
	default:
		return OTP_FAILURE;
	}

	if (ret)
		goto end;

	if (!memcmp(digest_buf, digest_ret, digest_len))
		return OTP_SUCCESS;

end:
	return OTP_FAILURE;
}

static int _otp_prog_image(int addr)
{
	struct otp_info_cb *info_cb = &otp_aspeed_data.info_cb;
	struct otp_image_layout image_layout;
	struct otp_header *otp_header;
	uint32_t data[2048];
	uint32_t scu_pro[2];
	uint32_t conf[16];
	uint8_t *checksum;
	uint8_t *buf;
	int image_soc_ver = 0;
	int image_size;
	int ret;
	int i;

	otp_header = (struct otp_header *)addr;
	image_size = OTP_IMAGE_SIZE(otp_header->image_info);
	buf = (uint8_t *)addr;

	if (!buf) {
		return OTP_FAILURE;
	}
	otp_header = (struct otp_header *)buf;
	checksum = buf + otp_header->checksum_offset;

	if (strcmp(OTP_MAGIC, (char *)otp_header->otp_magic) != 0) {
		return OTP_INVALID_HEADER;
	}

	image_layout.data_length = (int)(OTP_REGION_SIZE(otp_header->data_info) / 2);
	image_layout.data = buf + OTP_REGION_OFFSET(otp_header->data_info);
	image_layout.data_ignore = image_layout.data + image_layout.data_length;

	image_layout.conf_length = (int)(OTP_REGION_SIZE(otp_header->config_info) / 2);
	image_layout.conf = buf + OTP_REGION_OFFSET(otp_header->config_info);
	image_layout.conf_ignore = image_layout.conf + image_layout.conf_length;

	image_layout.strap = buf + OTP_REGION_OFFSET(otp_header->strap_info);
	image_layout.strap_length = (int)(OTP_REGION_SIZE(otp_header->strap_info) / 3);
	image_layout.strap_pro = image_layout.strap + image_layout.strap_length;
	image_layout.strap_ignore = image_layout.strap + 2 * image_layout.strap_length;

	image_layout.scu_pro = buf + OTP_REGION_OFFSET(otp_header->scu_protect_info);
	image_layout.scu_pro_length = (int)(OTP_REGION_SIZE(otp_header->scu_protect_info) / 2);
	image_layout.scu_pro_ignore = image_layout.scu_pro + image_layout.scu_pro_length;

	if (otp_header->soc_ver == SOC_AST1030A0) {
		image_soc_ver = OTP_AST1030A0;
	} else if (otp_header->soc_ver == SOC_AST1030A1) {
		image_soc_ver = OTP_AST1030A1;
	} else if (otp_header->soc_ver == SOC_AST1060A1) {
		image_soc_ver = OTP_AST1060A1;
	} else if (otp_header->soc_ver == SOC_AST1060A2) {
		image_soc_ver = OTP_AST1060A2;
	} else {
		return OTP_INVALID_SOC;
	}

	if (image_soc_ver != info_cb->version) {
		return OTP_INVALID_SOC;
	}

	switch (OTPTOOL_VERSION_MAJOR(otp_header->otptool_ver)) {
	case 1:
		/* WARNING: OTP image is not generated by otptool v2.x.x */
		/* Please use the latest version of otptool to generate OTP image */
		ret = otp_verify_image(buf, image_size, checksum, 1);
		break;
	case 2:
		ret = otp_verify_image(buf, image_size, checksum, 2);
		break;
	default:
		return OTP_FAILURE;
	}

	if (ret) {
		return OTP_INVALID_CHECKSUM;
	}

	if (info_cb->pro_sts.mem_lock) {
		return OTP_PROTECTED;
	}
	ret = 0;
	if (otp_header->image_info & OTP_INC_DATA) {
		if (info_cb->pro_sts.pro_data) {
			ret = OTP_PROTECTED;
		}
		if (info_cb->pro_sts.pro_sec) {
			ret = OTP_PROTECTED;
		}
		for (i = 0; i < 2048 ; i += 2)
			otp_read_data(i, &data[i]);
	}
	if (otp_header->image_info & OTP_INC_CONFIG) {
		if (info_cb->pro_sts.pro_conf) {
			ret = OTP_PROTECTED;
		}
		for (i = 0; i < 16 ; i++)
			otp_read_conf(i, &conf[i]);
	}
	if (otp_header->image_info & OTP_INC_STRAP) {
		if (info_cb->pro_sts.pro_strap) {
			ret = OTP_PROTECTED;
		}
		otp_strap_status(strap_status);
	}
	if (otp_header->image_info & OTP_INC_SCU_PRO) {
		if (info_cb->pro_sts.pro_strap) {
			ret = OTP_PROTECTED;
		}
		otp_read_conf(28, &scu_pro[0]);
		otp_read_conf(29, &scu_pro[1]);
	}
	if (ret < 0)
		return ret;

	if (otp_header->image_info & OTP_INC_DATA) {
		ret = otp_prog_data(&image_layout, data);
		if (ret != 0) {
			return OTP_PROG_FAILED;
		}
	}
	if (otp_header->image_info & OTP_INC_STRAP) {
		ret = otp_prog_strap(&image_layout, strap_status);
		if (ret != 0) {
			return OTP_PROG_FAILED;
		}
	}
	if (otp_header->image_info & OTP_INC_SCU_PRO) {
		ret = otp_prog_scu_protect(&image_layout, scu_pro);
		if (ret != 0) {
			return OTP_PROG_FAILED;
		}
	}
	if (otp_header->image_info & OTP_INC_CONFIG) {
		ret = otp_prog_conf(&image_layout, conf);
		if (ret != 0) {
			return OTP_PROG_FAILED;
		}
	}

	return OTP_SUCCESS;
}

static void ast_otp_unlock(void)
{
	OTP_WR(OTP_PASSWD, OTP_PROTECT_KEY); /* password */
}

static void ast_otp_lock(void)
{
	OTP_WR(1, OTP_PROTECT_KEY); /* protect otp controller */
}

static int aspeed_otp_prog(const struct device *dev, uint32_t otp_addr,
			   uint32_t prog_bit)
{
	ARG_UNUSED(dev);

	otp_prog(otp_addr, prog_bit);

	return 0;
}

static int aspeed_otp_read(const struct device *dev, uint32_t otp_addr,
			   uint32_t *data)
{
	ARG_UNUSED(dev);

	if (!data)
		return OTP_INVALID_PARAM;

	otp_read_data(otp_addr, data);

	return 0;
}

static int aspeed_otp_set_soak(const struct device *dev, int soak)
{
	ARG_UNUSED(dev);

	otp_soak(soak);

	return 0;
}

static int aspeed_otp_get_chip_rid(const struct device *dev,
				   uint32_t *revid)
{
	ARG_UNUSED(dev);

	if (!revid)
		return OTP_INVALID_PARAM;

	revid[0] = sys_read32(ASPEED_REVISION_ID0);
	revid[1] = sys_read32(ASPEED_REVISION_ID1);

	return 0;
}

static int aspeed_otp_get_key_num(const struct device *dev,
				  uint32_t *key_num)
{
	ARG_UNUSED(dev);

	if (!key_num)
		return OTP_INVALID_PARAM;

	*key_num = OTP_RD(SEC_KEY_NUM) & 7;

	return 0;
}

static int aspeed_otp_get_sw_rid(const struct device *dev,
				 uint32_t *sw_rid)
{
	ARG_UNUSED(dev);

	if (!sw_rid)
		return OTP_INVALID_PARAM;

	sw_rid[0] = OTP_RD(SW_REV_ID0);
	sw_rid[1] = OTP_RD(SW_REV_ID1);

	return 0;
}

static int aspeed_otp_get_tool_ver(const struct device *dev,
				   char *otp_ver_str)
{
	ARG_UNUSED(dev);

	if (!otp_ver_str)
		return OTP_INVALID_PARAM;

	memcpy(otp_ver_str, OTP_VER, sizeof(OTP_VER));

	return 0;
}

static int aspeed_otp_session_setup(const struct device *dev,
				    struct otp_info_cb *info_cb)
{
	struct otp_aspeed_data *data = (struct otp_aspeed_data *)dev->data;

	if (!info_cb)
		return OTP_INVALID_PARAM;

	ast_otp_unlock();

	*info_cb = data->info_cb;
	memcpy((void *)info_cb, (void *)&data->info_cb, sizeof(struct otp_info_cb));

	return 0;
}

static int aspeed_otp_session_free(const struct device *dev)
{
	ARG_UNUSED(dev);
	ast_otp_lock();

	return 0;
}

/*
 * Export API.
 * @brief:  Read OTP data from "offset" with legnth "len", and write it into
 *          "buf" buffer.
 * @offset: dw unit
 * @buf:    output buffer
 * @len:    dw unit
 */
int aspeed_otp_read_data(uint32_t offset, uint32_t *buf, uint32_t len)
{
	uint32_t ret[2];
	int i;

	if (offset + len > 2048 || offset % 4 != 0)
		return OTP_USAGE;

	ast_otp_unlock();
	otp_soak(0);
	for (i = offset; i < offset + len; i += 2) {
		otp_read_data(i, ret);
		memcpy(buf, ret, 8);
		buf += 2;
	}

	ast_otp_lock();

	return OTP_SUCCESS;
}

/*
 * Export API.
 * @brief:  Read OTP conf from "offset" with legnth "len", and write it into
 *          "buf" buffer.
 * @offset: dw unit
 * @buf:    output buffer
 * @len:    dw unit
 */
int aspeed_otp_read_conf(uint32_t offset, uint32_t *buf, uint32_t len)
{
	uint32_t ret[1];
	int i;

	if (offset + len > 32)
		return OTP_USAGE;

	ast_otp_unlock();
	otp_soak(0);
	for (i = offset; i < offset + len; i++) {
		otp_read_conf(i, ret);
		memcpy(buf, ret, 4);
		buf++;
	}

	ast_otp_lock();

	return OTP_SUCCESS;
}

/*
 * Export API.
 * @brief:  Program OTP data starts from "offset" with "buf" contents
 * @offset: dw unit
 * @buf:    input buffer
 * @len:    dw unit
 */
int aspeed_otp_prog_data(uint32_t offset, uint32_t *buf, uint32_t len)
{
	uint32_t ignore_mask[1] = {0};
	uint32_t *input_buf = buf;
	uint32_t prog_address;
	uint32_t compare[1];
	int pass;

	if (offset + len > 2048 || offset % 4 != 0)
		return OTP_USAGE;

	ast_otp_unlock();

	for (int i = 0; i < len; i++) {
		prog_address = offset + i;
		otp_soak(1);
		otp_prog_dw(input_buf[0], ignore_mask[0], prog_address);

		pass = 0;
		for (int j = 0; j < RETRY; j++) {
			if (verify_dw(prog_address, input_buf, ignore_mask, compare, 1) != 0) {
				otp_soak(2);
				if (verify_dw(prog_address, input_buf, ignore_mask, compare, 1) != 0) {
					otp_soak(1);
				} else {
					pass = 1;
					break;
				}
			} else {
				pass = 1;
				break;
			}
		}

		if (!pass) {
			otp_soak(0);
			return OTP_FAILURE;
		}

		input_buf++;
	}

	ast_otp_lock();

	return OTP_SUCCESS;
}

/*
 * Export API.
 * @brief:  Program OTP conf starts from "offset" with "buf" contents
 * @offset: dw unit
 * @buf:    input buffer
 * @len:    dw unit
 */
int aspeed_otp_prog_conf(uint32_t offset, uint32_t *buf, uint32_t len)
{
	uint32_t ignore_mask[1] = {0};
	uint32_t *input_buf = buf;
	uint32_t prog_address;
	uint32_t compare[1];
	int pass;

	if (offset + len > 32)
		return OTP_USAGE;

	ast_otp_unlock();

	for (int i = 0; i < len; i++) {
		prog_address = 0x800 +
				((offset + i) / 8) * 0x200 +
				((offset + i) % 8) * 0x2;

		otp_soak(1);
		otp_prog_dw(input_buf[0], ignore_mask[0], prog_address);

		pass = 0;
		for (int j = 0; j < RETRY; j++) {
			if (verify_dw(prog_address, input_buf, ignore_mask, compare, 1) != 0) {
				otp_soak(2);
				if (verify_dw(prog_address, input_buf, ignore_mask, compare, 1) != 0) {
					otp_soak(1);
				} else {
					pass = 1;
					break;
				}
			} else {
				pass = 1;
				break;
			}
		}

		if (!pass) {
			otp_soak(0);
			return OTP_FAILURE;
		}

		input_buf++;
	}

	ast_otp_lock();

	return OTP_SUCCESS;
}

/*
 * Export API.
 * @brief:  Program OTP memory with OTP image.
 * @addr:   OTP image location
 */
int aspeed_otp_prog_image(uint32_t addr)
{
	int ret;

	ast_otp_unlock();
	ret = _otp_prog_image(addr);
	ast_otp_lock();

	return ret;
}

/*
 * Export API.
 * @brief:      Program OTP strap by bit field.
 * @bit_offset: bit field
 * @value:      0 or 1
 */
int aspeed_otp_prog_strap_bit(uint32_t bit_offset, int value)
{
	struct otp_info_cb *info_cb = &otp_aspeed_data.info_cb;

	int ret = -EINVAL;

	if (bit_offset >= 64 || (value != 0 && value != 1))
		return -EINVAL;

	if (info_cb->pro_sts.pro_strap)
		goto end;

	ast_otp_unlock();
	ret = otp_prog_strap_b(bit_offset, value);

end:
	ast_otp_lock();
	return ret;
}

/*
 * Read full OTP strap into buffer.
 * @ buf: output OTP strap into buffer, should be at least 8 bytes length.
 */
int aspeed_otp_read_strap(uint32_t *buf)
{
	if (!buf)
		return -EINVAL;

	buf[0] = buf[1] = 0;

	ast_otp_unlock();
	otp_strap_status(strap_status);

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 32; j++)
			buf[i] |= strap_status[i * 32 + j].value << j;
	}

	ast_otp_lock();

	return 0;
}

static int aspeed_otp_dt(const struct device *dev)
{
	struct otp_aspeed_config *cfg = (struct otp_aspeed_config *)dev->config;
	int otpcfg_num = cfg->otpcfg_dt_setting_num/OTP_DT_OTPCFG_GROUP_NUM;
	int otpstrap_num = cfg->otpstrap_dt_setting_num/OTP_DT_OTPSTRAP_GROUP_NUM;
	uint32_t *otpcfg_list = cfg->otpcfg_dt_setting_list;
	uint32_t *otpstrap_list = cfg->otpstrap_dt_setting_list;
	uint32_t otp_conf[16];
	uint32_t prog_address;
	uint32_t compare[2];
	uint32_t value;
	int pass = 0;

	if (!cfg->otpcfg_dt_setting_num && !cfg->otpcfg_dt_setting_num)
		return 0;

	ast_otp_unlock();
	LOG_INF("Start programming OTPCFG... (numbers of setting: %d)", otpcfg_num);

	for (int i = 0; i < 16 ; i++)
		otp_read_conf(i, &otp_conf[i]);

	otp_soak(0);
	for (int i = 0; i < otpcfg_num * OTP_DT_OTPCFG_GROUP_NUM; i += OTP_DT_OTPCFG_GROUP_NUM) {
		LOG_INF("otpcfg[%d]: bit_offset:0x%x, value:0x%x",
			otpcfg_list[i], otpcfg_list[i + 1], otpcfg_list[i + 2]);

		prog_address = 0x800;
		prog_address |= (otpcfg_list[i] / 8) * 0x200;
		prog_address |= (otpcfg_list[i] % 8) * 0x2;

		otp_soak(1);
		value = otpcfg_list[i + 2] << otpcfg_list[i+1];

		otp_prog_dw(value, 0x0, prog_address);

		pass = 0;
		for (int k = 0; k < RETRY; k++) {
			if (verify_dw(prog_address, &value, 0x0, compare, 1) != 0) {
				otp_soak(2);
				otp_prog_dw(compare[0], 0x0, prog_address);
				if (verify_dw(prog_address, &value, 0x0, compare, 1) != 0) {
					otp_soak(1);
				} else {
					pass = 1;
					break;
				}
			} else {
				pass = 1;
				break;
			}
		}
		if (pass == 0) {
			LOG_ERR("address: %08x, otp_conf: %08x, input_conf: %08x\n",
				otpcfg_list[i], otp_conf[otpcfg_list[i]], value);
			break;
		}
	}

	otp_soak(0);
	ast_otp_lock();
	if (!pass)
		return OTP_FAILURE;

	LOG_INF("Done");

	LOG_INF("Start programming OTPSTRAP... (numbers of setting: %d)", otpstrap_num);

	for (int i = 0; i < otpstrap_num * OTP_DT_OTPSTRAP_GROUP_NUM; i += OTP_DT_OTPSTRAP_GROUP_NUM) {
		LOG_INF("otpstrap[%d]: 0x%x", otpstrap_list[i], otpstrap_list[i + 1]);
		pass = aspeed_otp_prog_strap_bit(otpstrap_list[i], otpstrap_list[i + 1]);
	}

	if (!pass)
		return OTP_FAILURE;

	LOG_INF("Done");

	return OTP_SUCCESS;
}

static int aspeed_otp_init(const struct device *dev)
{
	struct otp_aspeed_config *cfg = (struct otp_aspeed_config *)dev->config;
	struct otp_aspeed_data *data = (struct otp_aspeed_data *)dev->data;
	struct otp_info_cb *info_cb = &data->info_cb;
	struct otp_pro_sts *pro_sts;
	uint32_t otp_conf0;
	uint32_t ver;
	int ret;

	if (!otp_base)
		otp_base = cfg->base;

	LOG_DBG("otp_base:0x%x", (uint32_t)otp_base);

	ver = chip_version();
	switch (ver) {
	case OTP_AST1030A0:
		info_cb->version = OTP_AST1030A0;
		info_cb->conf_info = ast1030a0_conf_info;
		info_cb->conf_info_len = ARRAY_SIZE(ast1030a0_conf_info);
		info_cb->strap_info = ast1030a0_strap_info;
		info_cb->strap_info_len = ARRAY_SIZE(ast1030a0_strap_info);
		info_cb->scu_info = ast1030a0_scu_info;
		info_cb->scu_info_len = ARRAY_SIZE(ast1030a0_scu_info);
		info_cb->key_info = ast10xxa0_key_type;
		info_cb->key_info_len = ARRAY_SIZE(ast10xxa0_key_type);
		sprintf(info_cb->ver_name, "AST1030A0");
		break;
	case OTP_AST1030A1:
		info_cb->version = OTP_AST1030A1;
		info_cb->conf_info = ast1030a1_conf_info;
		info_cb->conf_info_len = ARRAY_SIZE(ast1030a1_conf_info);
		info_cb->strap_info = ast1030a0_strap_info;
		info_cb->strap_info_len = ARRAY_SIZE(ast1030a0_strap_info);
		info_cb->scu_info = ast1030a0_scu_info;
		info_cb->scu_info_len = ARRAY_SIZE(ast1030a0_scu_info);
		info_cb->key_info = ast10xxa1_key_type;
		info_cb->key_info_len = ARRAY_SIZE(ast10xxa1_key_type);
		sprintf(info_cb->ver_name, "AST1030A1");
		break;
	case OTP_AST1060A1:
		info_cb->version = OTP_AST1060A1;
		info_cb->conf_info = ast1030a1_conf_info;
		info_cb->conf_info_len = ARRAY_SIZE(ast1030a1_conf_info);
		info_cb->strap_info = ast1030a0_strap_info;
		info_cb->strap_info_len = ARRAY_SIZE(ast1030a0_strap_info);
		info_cb->scu_info = ast1030a0_scu_info;
		info_cb->scu_info_len = ARRAY_SIZE(ast1030a0_scu_info);
		info_cb->key_info = ast10xxa1_key_type;
		info_cb->key_info_len = ARRAY_SIZE(ast10xxa1_key_type);
		sprintf(info_cb->ver_name, "AST1060A1");
		break;
	case OTP_AST1060A2:
		info_cb->version = OTP_AST1060A2;
		info_cb->conf_info = ast1030a1_conf_info;
		info_cb->conf_info_len = ARRAY_SIZE(ast1030a1_conf_info);
		info_cb->strap_info = ast1030a0_strap_info;
		info_cb->strap_info_len = ARRAY_SIZE(ast1030a0_strap_info);
		info_cb->scu_info = ast1030a0_scu_info;
		info_cb->scu_info_len = ARRAY_SIZE(ast1030a0_scu_info);
		info_cb->key_info = ast10xxa1_key_type;
		info_cb->key_info_len = ARRAY_SIZE(ast10xxa1_key_type);
		sprintf(info_cb->ver_name, "AST1060A2");
		break;
	default:
		LOG_ERR("SOC is not supported\n");
		return -EINVAL;
	}

	ast_otp_unlock();
	otp_read_conf(0, &otp_conf0);
	ast_otp_lock();
	pro_sts = &info_cb->pro_sts;

	pro_sts->mem_lock = (otp_conf0 >> 31) & 0x1;
	pro_sts->pro_key_ret = (otp_conf0 >> 29) & 0x1;
	pro_sts->pro_strap = (otp_conf0 >> 25) & 0x1;
	pro_sts->pro_conf = (otp_conf0 >> 24) & 0x1;
	pro_sts->pro_data = (otp_conf0 >> 23) & 0x1;
	pro_sts->pro_sec = (otp_conf0 >> 22) & 0x1;
	pro_sts->sec_size = ((otp_conf0 >> 16) & 0x3f) << 5;

	ret = aspeed_otp_dt(dev);
	if (ret)
		LOG_ERR("Program otpcfg/otpstrap from dt failed");

	return 0;
}

static struct otp_driver_api otp_funcs = {
	.begin_session = aspeed_otp_session_setup,
	.free_session = aspeed_otp_session_free,
	.get_tool_ver = aspeed_otp_get_tool_ver,
	.get_sw_rid = aspeed_otp_get_sw_rid,
	.get_key_num = aspeed_otp_get_key_num,
	.get_chip_rid = aspeed_otp_get_chip_rid,

	.set_soak = aspeed_otp_set_soak,

	.otp_read = aspeed_otp_read,
	.otp_program = aspeed_otp_prog,
};

static struct otp_aspeed_config otp_aspeed_config = {
	.base = DT_REG_ADDR(DT_DRV_INST(0)),
	.otpcfg_dt_setting_list = DT_PROP(DT_DRV_INST(0), otpcfg),
	.otpcfg_dt_setting_num = DT_PROP_LEN(DT_DRV_INST(0), otpcfg),
	.otpstrap_dt_setting_list = DT_PROP(DT_DRV_INST(0), otpstrap),
	.otpstrap_dt_setting_num = DT_PROP_LEN(DT_DRV_INST(0), otpstrap),
};

DEVICE_DEFINE(otp_aspeed, CONFIG_OTP_ASPEED_DRV_NAME, aspeed_otp_init,
		NULL, &otp_aspeed_data, &otp_aspeed_config,
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		(void *)&otp_funcs);
