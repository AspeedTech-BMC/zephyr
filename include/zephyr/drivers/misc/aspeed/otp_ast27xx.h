/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_OTP_AST27XX_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_OTP_AST27XX_H_

#include <stdint.h>

#define OTP_AST2700_A0			0
#define OTP_AST2700_A1			1
#define OTP_AST2700_A2			2

int otp_read_rom(uint32_t offset, uint16_t *data);
int otp_read_rbp(uint32_t offset, uint16_t *data);
int otp_read_conf(uint32_t offset, uint16_t *data);
int otp_read_strap(uint32_t offset, uint16_t *data);
int otp_read_strap_ext(uint32_t offset, uint16_t *data);
int otp_read_strap_ext_vld(uint32_t offset, uint16_t *data);
int otp_read_user(uint32_t offset, uint16_t *data);
int otp_read_secure(uint32_t offset, uint16_t *data);
int otp_read_cptra(uint32_t offset, uint16_t *data);
int otp_read_puf(uint32_t offset, uint16_t *data);

int otp_print_ver(void);

int otp_print_rom(uint32_t offset, int w_count);
int otp_print_rbp(uint32_t offset, int w_count);
int otp_print_conf(uint32_t offset, int w_count);
int otp_print_strap(uint32_t offset, int w_count);
int otp_print_strap_pro(uint32_t offset, int w_count);
int otp_print_strap_ext(uint32_t offset, int w_count);
int otp_print_strap_ext_valid(uint32_t offset, int w_count);
int otp_print_user_data(uint32_t offset, int w_count);
int otp_print_sec_data(uint32_t offset, int w_count);
int otp_print_cptra(uint32_t offset, int w_count);
int otp_print_puf(uint32_t offset, int w_count);

int otp_print_rbp_info(void);
int otp_print_conf_info(void);
void otp_print_strap_info(void);
void otp_print_strap_ext_info(void);
void otp_print_key_info(void);

#endif
