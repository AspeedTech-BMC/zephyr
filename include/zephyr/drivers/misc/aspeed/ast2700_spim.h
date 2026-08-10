/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_SPIM_AST2700_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_SPIM_AST2700_H_

/*
 * pfr_aspeed.h already defines all the types, macros, and function
 * declarations in this block. Skip it when pfr_aspeed.h has already
 * been included to avoid duplicate-definition errors in translation
 * units that pull in both headers.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_PFR_ASPEED_H_

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/device.h>

/* general command */
#define CMD_RDID            0x9F
#define CMD_WREN            0x06
#define CMD_WRDIS           0x04
#define CMD_RDSR            0x05
#define CMD_RDCR            0x15
#define CMD_RDSR2           0x35
#define CMD_WRSR            0x01
#define CMD_WRSR2           0x31
#define CMD_SFDP            0x5A
#define CMD_EN4B            0xB7
#define CMD_EX4B            0xE9
#define CMD_RDFSR           0x70
#define CMD_VSR_WREN        0x50

/* read commands */
#define CMD_READ_1_1_1_3B   0x03
#define CMD_READ_1_1_1_4B   0x13
#define CMD_FREAD_1_1_1_3B  0x0B
#define CMD_FREAD_1_1_1_4B  0x0C
#define CMD_READ_1_1_2_3B   0x3B
#define CMD_READ_1_1_2_4B   0x3C
#define CMD_READ_1_2_2_3B   0xBB
#define CMD_READ_1_2_2_4B   0xBC
#define CMD_READ_1_1_4_3B   0x6B
#define CMD_READ_1_1_4_4B   0x6C
#define CMD_READ_1_4_4_3B   0xEB
#define CMD_READ_1_4_4_4B   0xEC

/* write command */
#define CMD_PP_1_1_1_3B     0x02
#define CMD_PP_1_1_1_4B     0x12
#define CMD_PP_1_1_4_3B     0x32
#define CMD_PP_1_1_4_4B     0x34
#define CMD_PP_1_4_4_3B     0x38
#define CMD_PP_1_4_4_4B     0x3E

/* sector erase command */
#define CMD_SE_1_1_0_3B     0x20
#define CMD_SE_1_1_0_4B     0x21
#define CMD_SE_1_1_0_64_3B  0xD8
#define CMD_SE_1_1_0_64_4B  0xDC

#define CMD_WREAR           0xC5
#define CMD_WINBOND_DIE_SEL 0xC2

struct cmd_table_info {
	uint8_t cmd;
	uint8_t reserved[3];
	uint32_t cmd_table_val;
};

/* Unused for AST2700 */
enum spim_ext_mux_sel {
	SPIM_EXT_MUX_SEL_0,
	SPIM_EXT_MUX_SEL_1,
};

/* allow command table control */
#define FLAG_CMD_TABLE_VALID         0x00000000
#define FLAG_CMD_TABLE_VALID_ONCE    0x00000001
#define FLAG_CMD_TABLE_LOCK_ALL      0x00000002

void spim_dump_allow_command_table(const struct device *dev);
int spim_get_allow_cmd_slot(const struct device *dev,
			    uint8_t cmd, uint32_t start_off);
int spim_add_allow_command(const struct device *dev, uint8_t cmd, uint32_t flag);
int spim_remove_allow_command(const struct device *dev, uint8_t cmd);
int spim_lock_allow_command_table(const struct device *dev, uint8_t cmd, uint32_t flag);
void spim_lock_common(const struct device *dev);
void spim_dump_addr_priv_table(const struct device *dev);
void spim_addr_priv_remove_all(const struct device *dev);
void spim_monitor_enable(const struct device *dev, bool enable);

struct spim_log_info {
	mem_addr_t log_ram_addr;
	uint32_t log_max_sz;
	uint32_t log_idx_reg;
};

typedef void (*spim_isr_callback_t)(const struct device *dev);
void spim_isr_callback_install(const struct device *dev,
	spim_isr_callback_t isr_callback);
void spim_get_log_info(const struct device *dev, struct spim_log_info *info);
uint32_t spim_get_ctrl_idx(const struct device *dev);
void aspeed_spi_monitor_sw_rst(const struct device *dev);
bool get_wdt_timeout_status(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_PFR_ASPEED_H_ */

/* AST2700-specific — always available */
#define FLAG_ADDR_PRIV_WRITE_DIS     0x00000001
#define FLAG_ADDR_PRIV_READ_DIS      0x00000002
#define FLAG_ADDR_PRIV_TABLE_LOCK    0x00000004

#define FLAG_SPIM_ACCESS_READ_EN     0x00000001
#define FLAG_SPIM_ACCESS_WRITE_EN    0x00000002

int ast2700_address_privilege_config(const struct device *dev,
				     uint32_t addr, uint32_t len,
				     uint32_t attr);
int ast2700_address_privilege_remove(const struct device *dev,
				     uint32_t addr, uint32_t len);
void ast2700_spim_blocked_log_parser(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_SPIM_AST2700_H_ */
