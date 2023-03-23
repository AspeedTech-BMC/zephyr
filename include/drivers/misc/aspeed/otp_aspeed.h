/*
 * Copyright (c) 2022 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_OTP_ASPEED_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_OTP_ASPEED_H_

#define RETRY				20
#define OTP_REGION_STRAP		BIT(0)
#define OTP_REGION_CONF			BIT(1)
#define OTP_REGION_DATA			BIT(2)

#define OTP_PROG_SKIP			1

#define OTP_MAGIC			"SOCOTP"
#define OTP_INC_DATA			BIT(31)
#define OTP_INC_CONFIG			BIT(30)
#define OTP_INC_STRAP			BIT(29)
#define OTP_INC_SCU_PRO			BIT(25)
#define OTP_REGION_SIZE(info)		(((info) >> 16) & 0xffff)
#define OTP_REGION_OFFSET(info)		((info) & 0xffff)
#define OTP_IMAGE_SIZE(info)		((info) & 0xffff)

#define ID0_AST1030A0		0x80000000
#define ID1_AST1030A0		0x80000000
#define ID0_AST1030A1		0x80010000
#define ID1_AST1030A1		0x80010000
#define ID0_AST1060A1		0xA0010000
#define ID1_AST1060A1		0xA0010000
#define ID0_AST1060A2		0xA0030000
#define ID1_AST1060A2		0xA0030000
#define ID0_AST1060A2_ENG	0x80030000
#define ID1_AST1060A2_ENG	0x80030000

#define OTP_AST1030A0	1
#define OTP_AST1030A1	2
#define OTP_AST1060A1	3
#define OTP_AST1060A2	4

#define SOC_AST1030A0	4
#define SOC_AST1030A1	5
#define SOC_AST1060A1	6
#define SOC_AST1060A2	7

#define OTPTOOL_VERSION(a, b, c) (((a) << 24) + ((b) << 12) + (c))
#define OTPTOOL_VERSION_MAJOR(x) (((x) >> 24) & 0xff)

enum otp_status {
	OTP_SUCCESS		= 0,
	OTP_USAGE		= -1,
	OTP_FAILURE		= -2,
	OTP_INVALID_HEADER	= -3,
	OTP_INVALID_SOC		= -4,
	OTP_INVALID_CHECKSUM	= -5,
	OTP_INVALID_PARAM	= -6,
	OTP_PROTECTED		= -7,
	OTP_PROG_FAILED		= -8,
};

enum otp_key_type {
	OTP_KEY_TYPE_RSA_PUB	= 1,
	OTP_KEY_TYPE_RSA_PRIV,
	OTP_KEY_TYPE_AES,
	OTP_KEY_TYPE_VAULT,
	OTP_KEY_TYPE_HMAC,
	OTP_KEY_ECDSA384,
	OTP_KEY_ECDSA384P,
};

struct otp_header {
	uint8_t		otp_magic[8];
	uint32_t	soc_ver;
	uint32_t	otptool_ver;
	uint32_t	image_info;
	uint32_t	data_info;
	uint32_t	config_info;
	uint32_t	strap_info;
	uint32_t	scu_protect_info;
	uint32_t	checksum_offset;
} __packed;

struct otpstrap_status {
	int value;
	int option_array[7];
	int remain_times;
	int writeable_option;
	int protected;
};

struct otpkey_type {
	int value;
	int key_type;
	int need_id;
	char information[110];
};

struct otp_pro_sts {
	char mem_lock;
	char pro_key_ret;
	char pro_strap;
	char pro_conf;
	char pro_data;
	char pro_sec;
	uint32_t sec_size;
};

struct otp_info_cb {
	int version;
	char ver_name[10];
	const struct otpstrap_info *strap_info;
	int strap_info_len;
	const struct otpconf_info *conf_info;
	int conf_info_len;
	const struct otpkey_type *key_info;
	int key_info_len;
	const struct scu_info *scu_info;
	int scu_info_len;
	struct otp_pro_sts pro_sts;
};

struct otp_image_layout {
	int data_length;
	int conf_length;
	int strap_length;
	int scu_pro_length;
	uint8_t *data;
	uint8_t *data_ignore;
	uint8_t *conf;
	uint8_t *conf_ignore;
	uint8_t *strap;
	uint8_t *strap_pro;
	uint8_t *strap_ignore;
	uint8_t *scu_pro;
	uint8_t *scu_pro_ignore;
};

int aspeed_otp_read_data(uint32_t offset, uint32_t *buf, uint32_t len);
int aspeed_otp_read_conf(uint32_t offset, uint32_t *buf, uint32_t len);
int aspeed_otp_read_strap(uint32_t *buf);

int aspeed_otp_prog_data(uint32_t offset, uint32_t *buf, uint32_t len);
int aspeed_otp_prog_conf(uint32_t offset, uint32_t *buf, uint32_t len);
int aspeed_otp_prog_strap_bit(uint32_t bit_offset, int value);
int aspeed_otp_prog_image(uint32_t addr);

#endif
