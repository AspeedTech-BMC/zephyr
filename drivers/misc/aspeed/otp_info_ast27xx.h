/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_OTP_INFO_AST27XX_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_OTP_INFO_AST27XX_H_

#define OTP_INFO_VER		"1.0.0"

#define OTP_REG_RESERVED        -1
#define OTP_REG_VALUE		-2

struct otprbp_info {
	signed char w_offset;
	unsigned char length;
	const char *information;
};

struct otpconf_info {
	signed char w_offset;
	signed char bit_offset;
	signed char length;
	signed char value;
	const char *information;
};

struct otpstrap_info {
	signed char bit_offset;
	signed char length;
	signed char value;
	const char *information;
};

struct otpstrap_ext_info {
	signed char bit_offset;
	signed char length;
	signed char value;
	const char *information;
};

struct otpcal_info {
	int w_offset;
	signed char bit_offset;
	int length;
	int value;
	const char *information;
};

struct otpkey_type {
	int value;
	int key_type;
	char *information;
};

enum KeyType {
	SOC_ECDSA_PUB = 1,
	SOC_LMS_PUB = 2,
	CAL_MANU_PUB_HASH = 3,
	CAL_OWN_PUB_HASH = 4,
	SOC_VAULT = 5,
	SOC_VAULT_SEED = 6,
};

union otp_pro_sts {
	uint32_t value;
	struct {
		char r_prot_strap_ext : 1;
		char r_prot_rom : 1;
		char r_prot_conf : 1;
		char r_prot_strap : 1;
		char w_prot_strap_ext : 1;
		char w_prot_rom : 1;
		char w_prot_conf : 1;
		char w_prot_strap : 1;
		char retire_option : 1;
		char en_sec_boot : 1;
		char w_prot_rbp : 1;
		char r_prot_cal : 1;
		char w_prot_cal : 1;
		char dis_otp_bist : 1;
		char w_prot_puf : 1;
		char mem_lock : 1;
	} fields;
};

struct otp_info_cb {
	int version;
	char ver_name[3];
	const struct otprbp_info *rbp_info;
	int rbp_info_len;
	const struct otpconf_info *conf_info;
	int conf_info_len;
	const struct otpstrap_info *strap_info;
	int strap_info_len;
	const struct otpstrap_ext_info *strap_ext_info;
	int strap_ext_info_len;
	const struct otpcal_info *cal_info;
	int cal_info_len;
	const struct otpkey_type *key_info;
	int key_info_len;
	union otp_pro_sts pro_sts;
};

static const struct otpkey_type a1_key_type[] = {
	{1, SOC_ECDSA_PUB,     "ECDSA384 as SoC OEM DSS public key"},
	{2, SOC_LMS_PUB,       "LMS as SoC OEM DSS public key"},
	{3, CAL_MANU_PUB_HASH, "Manufacture public key hash for Caliptra"},
	{4, CAL_OWN_PUB_HASH,  "Owner public key hash for Caliptra"},
	{5, SOC_VAULT,         "AES-256 as secret vault key"},
	{6, SOC_VAULT_SEED,    "AES-256 as secret vault key seed"},
};

static const struct otprbp_info a1_rbp_info[] = {
	{ 0,  16,  "SoC FMC ECC Key Retire bits" },
	{ 1,  16,  "SoC FMC LMS Key Retire bits" },
	{ 3,  16,  "Caliptra Owner pk key hash retire" },
	{ 4,  64,  "SoC FMC Hardware SVN" },
	{ 8,  32,  "Caliptra FMC Hardware SVN" },
	{ 10, 128, "Caliptra Runtime Hardware SVN" },
	{ 18, 4,   "Caliptra Manufacture ECC key mask" },
	{ 19, 32,  "Caliptra Manufacture LMS key mask" },
};

static const struct otpconf_info a1_conf_info[] = {
	{ 0, 15, 1, 1, "OTP memory lock enable" },
	{ 0, 15, 1, 0, "OTP memory lock disable" },
	{ 0, 14, 1, 1, "Enable PUF program done" },
	{ 0, 14, 1, 0, "Disable PUF program done" },
	{ 0, 13, 1, 1, "Disable OTP Memory Factory Test Mode" },
	{ 0, 13, 1, 0, "Enable OTP Memory Factory Test Mode" },
	{ 0, 12, 1, 1, "Enable write protect of caliptra region" },
	{ 0, 12, 1, 0, "Disable write protect of caliptra region" },
	{ 0, 11, 1, 1, "Enable read protect of caliptra region" },
	{ 0, 11, 1, 0, "Disable read protect of caliptra region" },
	{ 0, 10, 1, 1, "Enable Write Protect of rbp region" },
	{ 0, 10, 1, 0, "Disable Write Protect of rbp region" },
	{ 0, 9, 1, 1, "Enable Secure Boot" },
	{ 0, 9, 1, 0, "Disable Secure Boot" },
	{ 0, 8, 1, 1, "key retirement mechanism option enable" },
	{ 0, 8, 1, 0, "key retirement mechanism option disable" },
	{ 0, 7, 1, 1, "Enable Write Protectof OTP strap region" },
	{ 0, 7, 1, 0, "Disable Write Protectof OTP strap region" },
	{ 0, 6, 1, 1, "Enable Write Protect of Configure region" },
	{ 0, 6, 1, 0, "Disable Write Protect of Configure region" },
	{ 0, 5, 1, 1, "Enable write protect of rom region" },
	{ 0, 5, 1, 0, "Disable write protect of rom region" },
	{ 0, 4, 1, 1, "Enable Write Protect of strap ext region" },
	{ 0, 4, 1, 0, "Disable Write Protect of strap ext region" },
	{ 0, 3, 1, 1, "Enable read protect of strap region" },
	{ 0, 3, 1, 0, "Disable read protect of strap region" },
	{ 0, 2, 1, 1, "Enable read protect of configure region" },
	{ 0, 2, 1, 0, "Disable read protect of configure region" },
	{ 0, 1, 1, 1, "Enable read protect of rom region" },
	{ 0, 1, 1, 0, "Disable read protect of rom region" },
	{ 0, 0, 1, 1, "Enable read protect of strap ext region" },
	{ 0, 0, 1, 0, "Disable read protect of strap ext region" },
	{ 2, 13, 1, 1, "reserved" },
	{ 2, 13, 1, 0, "reserved" },
	{ 2, 12, 1, 1, "Ignore Secure Boot hardware strap enable" },
	{ 2, 12, 1, 0, "Ignore Secure Boot hardware strap disable" },
	{ 2, 11, 1, 1, "Disable low security key #0" },
	{ 2, 11, 1, 0, "Enable low security key #0" },
	{ 2, 10, 1, 1, "Enable SoC FMC verification with ECDSA and LMS" },
	{ 2, 10, 1, 0, "Disable SoC FMC verification with ECDSA and LMS" },
	{ 2, 8, 2, OTP_REG_VALUE, "The Device Life Cycle State: 0x%x" },
	{ 2, 6, 2, OTP_REG_VALUE, "Boot From UART Port Selection: 0x%x" },
	{ 2, 5, 1, 1, "Enable eMMC software reset" },
	{ 2, 5, 1, 0, "Disable eMMC software reset" },
	{ 2, 4, 1, 1, "Disable Uart Message of Boot ROM" },
	{ 2, 4, 1, 0, "Enable Uart Message of Boot ROM" },
	{ 2, 3, 1, 1, "Disable Boot from Uart/USB" },
	{ 2, 3, 1, 0, "Enable Boot from Uart/USB" },
	{ 2, 2, 1, 1, "Disable Post ROM Patch code" },
	{ 2, 2, 1, 0, "Enable Post ROM Patch code" },
	{ 2, 1, 1, 1, "Disable Pre ROM Patch code" },
	{ 2, 1, 1, 0, "Enable Pre ROM Patch code" },
	{ 2, 0, 1, 1, "Boot From UART Port Selection" },
	{ 2, 0, 1, 0, "Boot From UART Port Selection" },
	{ 4, 1, 10, OTP_REG_VALUE, "The Pre OTP patch code location in OTP ROM Region: 0x%x" },
	{ 4, 0, 1, 1, "Enable pre otp patch" },
	{ 4, 0, 1, 0, "Disable pre otp patch" },
	{ 5, 0, 10, OTP_REG_VALUE, "Pre OTP Patch code size (unit: word): 0x%x" },
	{ 6, 1, 10, OTP_REG_VALUE, "The Post OTP patch code location in OTP ROM Region: 0x%x" },
	{ 6, 0, 1, 1, "Enable post otp patch" },
	{ 6, 0, 1, 0, "Disable post otp patch" },
	{ 7, 0, 10, OTP_REG_VALUE, "Post OTP Patch code size: 0x%x" },
	{ 9, 15, 1, 1, "Enable auto load debug control SCU0C8 setting" },
	{ 9, 15, 1, 0, "Disable auto load debug control SCU0C8 setting" },
	{ 9, 0, 15, OTP_REG_VALUE, "CPU_SCU0C8[14 :0] auto setting value: 0x%x" },
	{ 10, 0, 8, OTP_REG_VALUE, "IO_SCU0C8[7 :0] auto setting value: 0x%x" },
	{ 11, 15, 1, 1, "Enable auto load OTP region user 0 ctrl setting" },
	{ 11, 15, 1, 0, "Disable auto load OTP region user 0 ctrl setting" },
	{ 11, 14, 1, 1, "Enable Read protect for OTP region user 0" },
	{ 11, 14, 1, 0, "Disable Read protect for OTP region user 0" },
	{ 11, 13, 1, 1, "Enable Write protect for OTP region user 0" },
	{ 11, 13, 1, 0, "Disable Write protect for OTP region user 0" },
	{ 11, 0, 12, OTP_REG_VALUE, "User 0 region start offset: 0x%x" },
	{ 12, 0, 16, OTP_REG_VALUE, "User 0 region size: 0x%x" },
	{ 13, 15, 1, 1, "Enable auto load OTP region secure 0 ctrl setting" },
	{ 13, 15, 1, 0, "Disable auto load OTP region secure 0 ctrl setting" },
	{ 13, 14, 1, 1, "Enable Read protect for OTP region secure 0" },
	{ 13, 14, 1, 0, "Disable Read protect for OTP region secure 0" },
	{ 13, 13, 1, 1, "Enable Write protect for OTP region secure 0" },
	{ 13, 13, 1, 0, "Disable Write protect for OTP region secure 0" },
	{ 13, 0, 12, OTP_REG_VALUE, "Secure 0 region start offset: 0x%x" },
	{ 14, 0, 16, OTP_REG_VALUE, "Secure 0 region size: 0x%x" },
	{ 15, 8, 8, OTP_REG_VALUE, "i3c: DCR: 0x%x" },
	{ 15, 5, 3, OTP_REG_VALUE, "recerved: 0x%x" },
	{ 15, 4, 1, 1, "i3c hot-join request enable" },
	{ 15, 4, 1, 0, "i3c hot-join request disable" },
	{ 15, 0, 4, OTP_REG_VALUE, "i3c/i2c channel used for recovery: 0x%x" },
	{ 16, 0, 16, OTP_REG_VALUE, "reserved: 0x%x" },
	{ 17, 15, 1, 1, "UART6 Mode - lsb (bit0)" },
	{ 17, 15, 1, 0, "UART6 Mode - lsb (bit0)" },
	{ 17, 14, 1, 1, "Enable to turn off UART5 Tx Discard" },
	{ 17, 14, 1, 0, "Disable to turn off UART5 Tx Discard" },
	{ 17, 12, 2, OTP_REG_VALUE, "UART5 Mode: 0x%x" },
	{ 17, 11, 1, 1, "Enable to turn off UART3 Tx Discard" },
	{ 17, 11, 1, 0, "Disable to turn off UART3 Tx Discard" },
	{ 17, 9, 2, OTP_REG_VALUE, "UART3 Mode: 0x%x" },
	{ 17, 8, 1, 1, "Enable to turn off UART2 Tx Discard" },
	{ 17, 8, 1, 0, "Disable to turn off UART2 Tx Discard" },
	{ 17, 6, 2, OTP_REG_VALUE, "UART2 Mode: 0x%x" },
	{ 17, 5, 1, 1, "Enable to turn off UART1 Tx Discard" },
	{ 17, 5, 1, 0, "Disable to turn off UART1 Tx Discard" },
	{ 17, 3, 2, OTP_REG_VALUE, "UART1 Mode: 0x%x" },
	{ 17, 2, 1, 1, "Enable to turn off UART0 Tx Discard" },
	{ 17, 2, 1, 0, "Disable to turn off UART0 Tx Discard" },
	{ 17, 0, 2, OTP_REG_VALUE, "UART0 Mode: 0x%x" },
	{ 18, 14, 2, OTP_REG_VALUE, "UART11 Mode: 0x%x" },
	{ 18, 13, 1, 1, "Enable to turn off UART10 Tx Discard" },
	{ 18, 13, 1, 0, "Disable to turn off UART10 Tx Discard" },
	{ 18, 11, 2, OTP_REG_VALUE, "UART10 Mode: 0x%x" },
	{ 18, 10, 1, 1, "Enable to turn off UART9 Tx Discard" },
	{ 18, 10, 1, 0, "Disable to turn off UART9 Tx Discard" },
	{ 18, 8, 2, OTP_REG_VALUE, "UART9 Mode: 0x%x" },
	{ 18, 7, 1, 1, "Enable to turn off UART8 Tx Discard" },
	{ 18, 7, 1, 0, "Disable to turn off UART8 Tx Discard" },
	{ 18, 5, 2, OTP_REG_VALUE, "UART8 Mode: 0x%x" },
	{ 18, 4, 1, 1, "Enable to turn off UART7 Tx Discard" },
	{ 18, 4, 1, 0, "Disable to turn off UART7 Tx Discard" },
	{ 18, 2, 2, OTP_REG_VALUE, "UART7 Mode: 0x%x" },
	{ 18, 1, 1, 1, "Enable to turn off UART6 Tx Discard" },
	{ 18, 1, 1, 0, "Disable to turn off UART6 Tx Discard" },
	{ 18, 0, 1, 1, "UART6 Mode - msb (bit1)" },
	{ 18, 0, 1, 0, "UART6 Mode - msb (bit1)" },
	{ 19, 13, 3, OTP_REG_VALUE, "uart11/13/14 routing (hw effect by scu): 0x%x" },
	{ 19, 9, 1, 1, "Enable UART to put all boot message and wait the completetion" },
	{ 19, 9, 1, 0, "Disable UART to put all boot message and wait the completetion" },
	{ 19, 8, 1, 1, "Enable Mode-3 (USB2UART only, no vHub)" },
	{ 19, 8, 1, 0, "Disable Mode-3 (USB2UART only, no vHub)" },
	{ 19, 7, 1, 1, "Enable Mode-0 hub to support 23 ports" },
	{ 19, 7, 1, 0, "Disable Mode-0 hub to support 23 ports" },
	{ 19, 6, 1, 1, "Enable to turn off UART15 Tx Discard" },
	{ 19, 6, 1, 0, "Disable to turn off UART15 Tx Discard" },
	{ 19, 4, 2, OTP_REG_VALUE, "UART15 Mode: 0x%x" },
	{ 19, 3, 1, 1, "Enable to turn off UART12 Tx Discard" },
	{ 19, 3, 1, 0, "Disable to turn off UART12 Tx Discard" },
	{ 19, 1, 2, OTP_REG_VALUE, "UART12 Mode: 0x%x" },
	{ 19, 0, 1, 1, "Enable to turn off UART11 Tx Discard" },
	{ 19, 0, 1, 0, "Disable to turn off UART11 Tx Discard" },
	{ 20, 15, 1, 1, "Enable auto load OTP region cptra 0 ctrl setting" },
	{ 20, 15, 1, 0, "Disable auto load OTP region cptra 0 ctrl setting" },
	{ 20, 14, 1, 1, "Enable Read protect for OTP region cptra 0" },
	{ 20, 14, 1, 0, "Disable Read protect for OTP region cptra 0" },
	{ 20, 13, 1, 1, "Enable Write protect for OTP region cptra 0" },
	{ 20, 13, 1, 0, "Disable Write protect for OTP region cptra 0" },
	{ 20, 0, 12, OTP_REG_VALUE, "cptra 0 region start offset: 0x%x" },
	{ 21, 0, 16, OTP_REG_VALUE, "cptra 0 region size: 0x%x" },
	{ 23, 9, 1, 1, "0=Don't check LINK_SPEED before AD (follow the LTPI spec.)" },
	{ 23, 9, 1, 0, "0=Don't check LINK_SPEED before AD (follow the LTPI spec.)" },
	{ 23, 8, 1, 1, "0=Don't check LINK_SPEED before AD (follow the LTPI spec.)" },
	{ 23, 8, 1, 0, "0=Don't check LINK_SPEED before AD (follow the LTPI spec.)" },
	{ 23, 6, 2, OTP_REG_VALUE, "bit[1] - RX clock inverse: 0x%x" },
	{ 23, 4, 2, OTP_REG_VALUE, "bit[1] - RX clock inverse: 0x%x" },
	{ 23, 2, 2, OTP_REG_VALUE, "4 levels: 0x%x" },
	{ 23, 1, 1, 1, "Set 1 to enable AST1700 runtime firmware" },
	{ 23, 1, 1, 0, "Set 1 to disable AST1700 runtime firmware" },
	{ 23, 0, 1, 1, "LTPI0 & LTPI1 CRC format" },
	{ 23, 0, 1, 0, "LTPI0 & LTPI1 CRC format" },
	{ 24, 12, 4, OTP_REG_VALUE, "IOD TX (US) clock speed: 0x%x" },
	{ 24, 11, 1, 1, "IOD SLIH TXCLK delay[5]" },
	{ 24, 11, 1, 0, "IOD SLIH TXCLK delay[5]" },
	{ 24, 6, 5, OTP_REG_VALUE, "IOD SLIH TXCLK delay[4:0]: 0x%x" },
	{ 24, 5, 1, 1, "IOD SLIH RXCLK delay[5]" },
	{ 24, 5, 1, 0, "IOD SLIH RXCLK delay[5]" },
	{ 24, 0, 5, OTP_REG_VALUE, "IOD SLIH RXCLK delay[4:0]: 0x%x" },
	{ 25, 12, 4, OTP_REG_VALUE, "CPU TX (DS) clock speed: 0x%x" },
	{ 25, 0, 6, OTP_REG_VALUE, "bit[5] - CPU SLIV RX latch data P/N swap: 0x%x" },
	{ 26, 0, 16, OTP_REG_VALUE, "bit[15:12] - CPU SLIH RXD1[3:0]: 0x%x" },
	{ 27, 0, 16, OTP_REG_VALUE, "bit[15:12] - CPU SLIM RXD3[3:0]: 0x%x" },
	{ 28, 0, 16, OTP_REG_VALUE, "bit[15:12] - IOD SLIM RXD3[3:0]: 0x%x" },
	{ 29, 0, 16, OTP_REG_VALUE, "bit[15:12] - CPU SLIV RXD1[3:0]: 0x%x" },
	{ 30, 15, 1, 1, "Set 1 to disable LTPI0 DDR mode" },
	{ 30, 15, 1, 0, "Set 1 to enable LTPI0 DDR mode" },
	{ 30, 0, 15, OTP_REG_VALUE, "bit[14:13] - LTPI spec. reserved: 0x%x" },
	{ 31, 15, 1, 1, "Set 1 to disable LTPI1 DDR mode" },
	{ 31, 15, 1, 0, "Set 1 to enable LTPI1 DDR mode" },
	{ 31, 0, 15, OTP_REG_VALUE, "bit[14:13] - LTPI spec. reserved: 0x%x" },
};

static const struct otpstrap_info a1_strap_info[] = {
	{ 1, 1, 1, "Boot from CPU die SPI" },
	{ 1, 1, 0, "Boot from CPU die SPI" },
	{ 2, 2, 0, "HPLL default frequency" },
	{ 2, 2, 1, "HPLL default frequency" },
	{ 2, 2, 2, "HPLL default frequency" },
	{ 2, 2, 3, "HPLL default frequency" },
	{ 4, 1, 1, "CPU clock PLL" },
	{ 4, 1, 0, "CPU clock PLL" },
	{ 5, 2, 0, "AXI/AHB clock selection" },
	{ 5, 2, 1, "AXI/AHB clock selection" },
	{ 5, 2, 2, "AXI/AHB clock selection" },
	{ 5, 2, 3, "AXI/AHB clock selection" },
	{ 7, 1, 1, "AXI/AHB clock PLL" },
	{ 7, 1, 0, "AXI/AHB clock PLL" },
	{ 8, 1, 1, "Disable Debug Interfaces for CPU die" },
	{ 8, 1, 0, "Enable Debug Interfaces for CPU die" },
	{ 9, 1, 1, "TSP reset pin selection" },
	{ 9, 1, 0, "TSP reset pin selection" },
	{ 10, 1, 1, "VGA linear memory size" },
	{ 10, 1, 0, "VGA linear memory size" },
	{ 11, 1, 1, "VGA linear memory size" },
	{ 11, 1, 0, "VGA linear memory size" },
	{ 12, 1, 1, "Disable SSP running." },
	{ 12, 1, 0, "Enable SSP running." },
	{ 13, 1, 1, "Disable TSP running." },
	{ 13, 1, 0, "Enable TSP running." },
	{ 14, 1, 1, "Enable UFS secure mode" },
	{ 14, 1, 0, "Disable UFS secure mode" },
	{ 16, 1, 1, "Enable Secure Boot " },
	{ 16, 1, 0, "Disable Secure Boot " },
	{ 17, 1, 1, "Disable Debug Interfaces for IO die" },
	{ 17, 1, 0, "Enable Debug Interfaces for IO die" },
	{ 18, 1, 1, "Disable WDT reset full" },
	{ 18, 1, 0, "Enable WDT reset full" },
	{ 19, 1, 1, "Disable Uart Debug" },
	{ 19, 1, 0, "Enable Uart Debug" },
	{ 20, 1, 1, "Selection of IO for Uart Debug" },
	{ 20, 1, 0, "Selection of IO for Uart Debug" },
	{ 21, 1, 1, "Disable JTAG of Caliptra permanently" },
	{ 21, 1, 0, "Enable JTAG of Caliptra permanently" },
	{ 22, 1, 1, "Disable SoC FMC hash verify" },
	{ 22, 1, 0, "Enable SoC FMC hash verify" },
	{ 23, 1, 1, "Disable BootMCU ROM & jump to I_FMC" },
	{ 23, 1, 0, "Enable BootMCU ROM & jump to I_FMC" },
	{ 24, 1, 1, "Boot from eMMC or UFS selection" },
	{ 24, 1, 0, "Boot from eMMC or UFS selection" },
	{ 25, 1, 1, "Allow BROM to wait SPI flash ready bit." },
	{ 25, 1, 0, "Allow BROM to wait SPI flash ready bit." },
	{ 26, 1, 1, "Use 4-byte command to read flash." },
	{ 26, 1, 0, "Use 4-byte command to read flash." },
	{ 27, 2, 0, "Recovery from Uart or USB selection" },
	{ 27, 2, 1, "Recovery from Uart or USB selection" },
	{ 27, 2, 2, "Recovery from Uart or USB selection" },
	{ 27, 2, 3, "Recovery from Uart or USB selection" },
	{ 29, 2, 0, "Boot from USB port selection" },
	{ 29, 2, 1, "Boot from USB port selection" },
	{ 29, 2, 2, "Boot from USB port selection" },
	{ 29, 2, 3, "Boot from USB port selection" },
	{ 31, 1, 1, "Disable Caliptra booting" },
	{ 31, 1, 0, "Enable Caliptra booting" },
};

static const struct otpstrap_ext_info a1_strap_ext_info[] = {
	{ 16, 1, 1, "Disable ARM ICE" },
	{ 16, 1, 0, "Enable ARM ICE" },
	{ 18, 1, 1, "VGA0/1 Class Code" },
	{ 18, 1, 0, "VGA0/1 Class Code" },
	{ 19, 1, 1, "UFS reference clock selection" },
	{ 19, 1, 0, "UFS reference clock selection" },
	{ 20, 1, 1, "eMMC Boot Speed" },
	{ 20, 1, 0, "eMMC Boot Speed" },
	{ 21, 1, 1, "Disable XHCI0/1" },
	{ 21, 1, 0, "Enable XHCI0/1" },
	{ 22, 1, 1, "Disable ARM ICE in Trust World" },
	{ 22, 1, 0, "Enable ARM ICE in Trust World" },
	{ 25, 1, 1, "Disable WDT reset full" },
	{ 25, 1, 0, "Enable WDT reset full" },
	{ 26, 2, 0, "Uart Debug timeout selection" },
	{ 26, 2, 1, "Uart Debug timeout selection" },
	{ 26, 2, 2, "Uart Debug timeout selection" },
	{ 26, 2, 3, "Uart Debug timeout selection" },
	{ 28, 1, 1, "DAC's source selection" },
	{ 28, 1, 0, "DAC's source selection" },
	{ 29, 1, 1, "DisplayPort's source selection" },
	{ 29, 1, 0, "DisplayPort's source selection" },
	{ 30, 1, 1, "Disable RVAS0/1" },
	{ 30, 1, 0, "Enable RVAS0/1" },
	{ 100, 1, 1, "Node0 : Size of VGA RAM" },
	{ 100, 1, 0, "Node0 : Size of VGA RAM" },
	{ 103, 1, 1, "Enable PCIe VGA0 " },
	{ 103, 1, 0, "Disable PCIe VGA0 " },
	{ 104, 1, 1, "Enable PCIe IPMI0 " },
	{ 104, 1, 0, "Disable PCIe IPMI0 " },
	{ 105, 1, 1, "Enable PCIe EHCI0" },
	{ 105, 1, 0, "Disable PCIe EHCI0" },
	{ 106, 1, 1, "Enable PCIe XHCI0" },
	{ 106, 1, 0, "Disable PCIe XHCI0" },
	{ 107, 1, 1, "Enable PCIe VGA1" },
	{ 107, 1, 0, "Disable PCIe VGA1" },
	{ 108, 1, 1, "Enable PCIe IPMI1" },
	{ 108, 1, 0, "Disable PCIe IPMI1" },
	{ 109, 1, 1, "Enable PCIe EHCI1" },
	{ 109, 1, 0, "Disable PCIe EHCI1" },
	{ 110, 1, 1, "Enable PCIe XHCI1" },
	{ 110, 1, 0, "Disable PCIe XHCI1" },
	{ 32, 1, 1, "boot storage ABR enable" },
	{ 32, 1, 0, "boot storage ABR enable" },
	{ 33, 1, 1, "ROM clear SRAM" },
	{ 33, 1, 0, "ROM clear SRAM" },
	{ 39, 2, 0, "Uart Debug timeout selection" },
	{ 39, 2, 1, "Uart Debug timeout selection" },
	{ 39, 2, 2, "Uart Debug timeout selection" },
	{ 39, 2, 3, "Uart Debug timeout selection" },
	{ 43, 1, 1, "Enable Caliptra debug state" },
	{ 43, 1, 0, "Disable Caliptra debug state" },
	{ 44, 1, 1, "Enable ROM WDT" },
	{ 44, 1, 0, "Disable ROM WDT" },
	{ 45, 3, 0, "Size of FW SPI Flash" },
	{ 45, 3, 1, "Size of FW SPI Flash" },
	{ 45, 3, 2, "Size of FW SPI Flash" },
	{ 45, 3, 3, "Size of FW SPI Flash" },
	{ 45, 3, 4, "Size of FW SPI Flash" },
	{ 45, 3, 5, "Size of FW SPI Flash" },
	{ 45, 3, 6, "Size of FW SPI Flash" },
	{ 45, 3, 7, "Size of FW SPI Flash" },
	{ 48, 1, 1, "Enable FWSPI auxiliary pins" },
	{ 48, 1, 0, "Disable FWSPI auxiliary pins" },
	{ 55, 2, 0, "Boot SPI Frequency Selection" },
	{ 55, 2, 1, "Boot SPI Frequency Selection" },
	{ 55, 2, 2, "Boot SPI Frequency Selection" },
	{ 55, 2, 3, "Boot SPI Frequency Selection" },
	{ 57, 1, 1, "Check FWSPI flash is accessible before booting up" },
	{ 57, 1, 0, "Don't check FWSPI flash is accessible before booting up" },
	{ 59, 1, 1, "Enable FW SPI ABR infinite loop" },
	{ 59, 1, 0, "Disable FW SPI ABR infinite loop" },
	{ 60, 1, 1, "Disable CS swap after dual flash ABR is triggerred." },
	{ 60, 1, 0, "Enable CS swap after dual flash ABR is triggerred." },
	{ 61, 1, 1, "FMC abr mode" },
	{ 61, 1, 0, "FMC abr mode" },
	{ 67, 1, 1, "Enable Host0 SPI auxiliary pins" },
	{ 67, 1, 0, "Disable Host0 SPI auxiliary pins" },
	{ 70, 1, 1, "Selection of SIO decode address" },
	{ 70, 1, 0, "Selection of SIO decode address" },
	{ 71, 1, 1, "Disable SIO Decoding" },
	{ 71, 1, 0, "Enable SIO Decoding" },
	{ 72, 1, 1, "Enable ACPI" },
	{ 72, 1, 0, "Disable ACPI" },
	{ 73, 1, 1, "Enable LPC" },
	{ 73, 1, 0, "Disable LPC" },
	{ 74, 1, 1, "Enable eDAF" },
	{ 74, 1, 0, "Disable eDAF" },
	{ 75, 1, 1, "Enable eSPI RTC" },
	{ 75, 1, 0, "Disable eSPI RTC" },
	{ 83, 1, 1, "Enable Host1 SPI auxiliary pins" },
	{ 83, 1, 0, "Disable Host1 SPI auxiliary pins" },
	{ 86, 1, 1, "Selection of SIO decode address" },
	{ 86, 1, 0, "Selection of SIO decode address" },
	{ 87, 1, 1, "Disable SIO Decoding" },
	{ 87, 1, 0, "Enable SIO Decoding" },
	{ 88, 1, 1, "Enable ACPI" },
	{ 88, 1, 0, "Disable ACPI" },
	{ 89, 1, 1, "Enable LPC" },
	{ 89, 1, 0, "Disable LPC" },
	{ 90, 1, 1, "Enable eDAF" },
	{ 90, 1, 0, "Disable eDAF" },
	{ 91, 1, 1, "Enable eSPI RTC" },
	{ 91, 1, 0, "Disable eSPI RTC" },
};

static const struct otpcal_info a1_cal_info[] = {
	{ 0, 0, 1, 1, "Enable LMS verify" },
	{ 0, 0, 1, 0, "Disable LMS verify" },
	{ 1, 0, 1, 1, "Anti-rollback disable" },
	{ 1, 0, 1, 0, "Anti-rollback enable" },
	{ 2, 0, 256, OTP_REG_VALUE, "Field Entropy: 0x%x" },
	{ 18, 0, 384, OTP_REG_VALUE, "Manufacture Key Hash: 0x%x" },
	{ 42, 0, 768, OTP_REG_VALUE, "IDEVID Cert attr: 0x%x" },
	{ 90, 0, 128, OTP_REG_VALUE, "IDEVID manuf HSM identifier: 0x%x" },
	{ 98, 0, 8192, OTP_REG_VALUE, "IDEVID TBS: 0x%x" },
	{ 610, 0, 768, OTP_REG_VALUE, "IDEVID Cert Signature: 0x%x" },
};

#endif
