/*
 * Copyright (c) 2021 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_spi_monitor_controller

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>
#include <drivers/clock_control.h>
#include <errno.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(spim_aspeed, CONFIG_SPI_LOG_LEVEL);
#include "spi_context.h"
#include <sys/sys_io.h>
#include <sys/__assert.h>

/* general command */
#define CMD_RDID			0x9F
#define CMD_WREN			0x06
#define CMD_WRDIS			0x04
#define CMD_RDSR			0x05
#define CMD_RDCR			0x15
#define CMD_RDSR2			0x35
#define CMD_WRSR			0x01
#define CMD_WRSR2			0x31
#define CMD_SFDP			0x5A
#define CMD_EN4B			0xB7
#define CMD_EX4B			0xE9

/* read commands */
#define CMD_READ_1_1_1_3B	0x03
#define CMD_READ_1_1_1_4B	0x13
#define CMD_FREAD_1_1_1_3B	0x0B
#define CMD_FREAD_1_1_1_4B	0x0C
#define CMD_READ_1_1_2_3B	0x3B
#define CMD_READ_1_1_2_4B	0x3C
#define CMD_READ_1_1_4_3B	0x6B
#define CMD_READ_1_1_4_4B	0x6C

/* write command */
#define CMD_PP_1_1_1_3B		0x02
#define CMD_PP_1_1_1_4B		0x12
#define CMD_PP_1_1_4_3B		0x32
#define CMD_PP_1_1_4_4B		0x34
#define CMD_PP_1_4_4_3B		0x38
#define CMD_PP_1_4_4_4B		0x3E


/* sector erase command */
#define CMD_SE_1_1_0_3B		0x20
#define CMD_SE_1_1_0_4B		0x21
#define CMD_SE_1_1_0_64_3B	0xD8
#define CMD_SE_1_1_0_64_4B	0xDC

struct valid_cmd_info {
	uint8_t cmd;
	uint8_t reserved[3];
	uint32_t valid_table_val;
};

#define VALID_LIST_VALUE(G, W, R, M, DAT_MODE, DUMMY, PROG_SZ, ADDR_LEN, ADDR_MODE, CMD) \
	(G << 29 | W << 28 | R << 27 | M << 26 | DAT_MODE << 24 | DUMMY << 16 | PROG_SZ << 13 | \
	ADDR_LEN << 10 | ADDR_MODE << 8 | CMD)

struct valid_cmd_info valid_cmds[] = {
	{.cmd = CMD_READ_1_1_1_3B,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 1, 1, 0, 0, 3, 1, CMD_READ_1_1_1_3B)},
	{.cmd = CMD_READ_1_1_1_4B,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 1, 1, 0, 0, 4, 1, CMD_READ_1_1_1_4B)},
	{.cmd = CMD_FREAD_1_1_1_3B,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 1, 1, 8, 0, 3, 1, CMD_FREAD_1_1_1_3B)},
	{.cmd = CMD_FREAD_1_1_1_4B,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 1, 1, 8, 0, 4, 1, CMD_FREAD_1_1_1_4B)},
	{.cmd = CMD_READ_1_1_2_3B,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 1, 2, 8, 0, 3, 1, CMD_READ_1_1_2_3B)},
	{.cmd = CMD_READ_1_1_2_4B,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 1, 2, 8, 0, 4, 1, CMD_READ_1_1_2_4B)},
	{.cmd = CMD_READ_1_1_4_3B,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 1, 3, 8, 0, 3, 1, CMD_READ_1_1_4_3B)},
	{.cmd = CMD_READ_1_1_4_4B,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 1, 3, 8, 0, 4, 1, CMD_READ_1_1_4_4B)},
	{.cmd = CMD_PP_1_1_1_3B,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 1, 1, 0, 1, 3, 1, CMD_PP_1_1_1_3B)},
	{.cmd = CMD_PP_1_1_1_4B,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 1, 1, 0, 1, 4, 1, CMD_PP_1_1_1_4B)},
	{.cmd = CMD_PP_1_1_4_3B,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 1, 3, 0, 1, 3, 1, CMD_PP_1_1_4_3B)},
	{.cmd = CMD_PP_1_1_4_4B,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 1, 3, 0, 1, 4, 1, CMD_PP_1_1_4_4B)},
	{.cmd = CMD_SE_1_1_0_3B,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 1, 0, 0, 1, 3, 1, CMD_SE_1_1_0_3B)},
	{.cmd = CMD_SE_1_1_0_4B,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 1, 0, 0, 1, 4, 1, CMD_SE_1_1_0_4B)},
	{.cmd = CMD_SE_1_1_0_64_3B,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 1, 0, 0, 5, 3, 1, CMD_SE_1_1_0_64_3B)},
	{.cmd = CMD_SE_1_1_0_64_4B,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 1, 0, 0, 5, 4, 1, CMD_SE_1_1_0_64_4B)},
	{.cmd = CMD_WREN,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 0, 0, 0, 0, 0, 0, 0, CMD_WREN)},
	{.cmd = CMD_WRDIS,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 0, 0, 0, 0, 0, 0, 0, CMD_WRDIS)},
	{.cmd = CMD_RDSR,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 0, 1, 0, 0, 0, 0, CMD_RDSR)},
	{.cmd = CMD_RDSR2,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 0, 1, 0, 0, 0, 0, CMD_RDSR2)},
	{.cmd = CMD_WRSR,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 0, 1, 0, 0, 0, 0, CMD_WRSR)},
	{.cmd = CMD_WRSR2,
		.valid_table_val = VALID_LIST_VALUE(1, 1, 0, 0, 1, 0, 0, 0, 0, CMD_WRSR2)},
	{.cmd = CMD_RDCR,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 0, 1, 0, 0, 0, 0, CMD_RDCR)},
	{.cmd = CMD_EN4B,
		.valid_table_val = VALID_LIST_VALUE(0, 0, 0, 0, 0, 0, 0, 0, 0, CMD_EN4B)},
	{.cmd = CMD_EX4B,
		.valid_table_val = VALID_LIST_VALUE(0, 0, 0, 0, 0, 0, 0, 0, 0, CMD_EX4B)},
	{.cmd = CMD_SFDP,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 0, 1, 8, 0, 3, 1, CMD_SFDP)},
	{.cmd = CMD_RDID,
		.valid_table_val = VALID_LIST_VALUE(1, 0, 1, 0, 1, 0, 0, 0, 0, CMD_RDID)},
};

#define SPIM_CTRL				(0x0000)
#define SPIM_STATUS				(0x0004)
#define SPIM_EAR				(0x0008)
#define SPIM_FIFO				(0x000C)
#define SPIM_LOG_BASE			(0x0010)
#define SPIM_LOG_SZ				(0x0014)
#define SPIM_LOG_PTR			(0x0018)
#define SPIM_LOCK_REG			(0x007C)
#define SPIM_VALID_LIST_BASE	(0x0080)
#define SPIM_VALID_ADDR_FTR		(0x0100)

#define SPIM_PRIV_WRITE_SELECT	0x57000000
#define SPIM_PRIV_READ_SELECT	0x52000000


#define SPIM_ADDR_PRIV_REG_NUN	512
#define SPIM_ADDR_PRIV_BIT_NUN	(512 * 32)

#define SPIM_VALID_TABLE_VALID_ONCE_BIT	BIT(31)
#define SPIM_VALID_TABLE_VALID_BIT	BIT(30)


#define SPIM_MODE_SCU_CTRL		(0x00f0)

#define FLAG_VALID_LIST_VALID_ONCE		0x00000002

struct aspeed_spim_data {
	const  uint8_t *valid_cmd_list;
	uint32_t valid_cmd_num;
	const  uint32_t *read_forbidden_regions;
	uint32_t read_forbidden_region_num;
	const  uint32_t *write_forbidden_regions;
	uint32_t write_forbidden_region_num;
};

struct aspeed_spim_config {
	mm_reg_t ctrl_base;
	mm_reg_t scu_base;
	uint32_t irq_num;
	uint32_t irq_priority;
	uint32_t log_ram_addr;
	uint32_t log_max_len;
	uint32_t ctrl_num;
};

void spim_dump_valid_cmd_table(const struct device *dev)
{
	uint32_t i;
	const struct aspeed_spim_config *config = dev->config;

	for (i = 0; i < 32; i++) {
		LOG_INF("[%s]idx %d: 0x%08x", dev->name, i,
			sys_read32(config->ctrl_base + SPIM_VALID_LIST_BASE + i * 4));
	}
}

uint32_t spim_get_valid_cmd_val(uint8_t cmd)
{
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(valid_cmds); i++) {
		if (valid_cmds[i].cmd == cmd)
			return valid_cmds[i].valid_table_val;
	}

	LOG_ERR("Error: Cannot get item in command table cmd(%02x)\n", cmd);
	return 0;
}

void spim_config_passthrough_mode(const struct device *dev, bool passthrough_en)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t scu_reg_val;
	uint32_t ctrl_reg_val;

	scu_reg_val = sys_read32(config->scu_base + SPIM_MODE_SCU_CTRL);
	ctrl_reg_val = sys_read32(config->ctrl_base);

	ctrl_reg_val &= ~0x00000003;
	if (passthrough_en) {
		scu_reg_val |= (BIT(config->ctrl_num - 1) << 4);
		ctrl_reg_val &= ~0x00000003;
		ctrl_reg_val |= 0x00000002;
	} else {
		scu_reg_val &= ~(BIT(config->ctrl_num - 1) << 4);
	}

	sys_write32(scu_reg_val, config->scu_base + SPIM_MODE_SCU_CTRL);
	sys_write32(ctrl_reg_val, config->ctrl_base);
}

void spim_fill_valid_table(const struct device *dev,
	const uint8_t cmd_list[], uint32_t cmd_num, uint32_t flag)
{
	const struct aspeed_spim_config *config = dev->config;

	uint32_t i;
	uint32_t reg_val;
	uint32_t idx = 3;

	for (i = 0; i < cmd_num; i++) {
		reg_val = spim_get_valid_cmd_val(cmd_list[i]);
		LOG_DBG("cmd %02x, val %08x", cmd_list[i], reg_val);
		if (reg_val == 0)
			continue;

		if (flag & FLAG_VALID_LIST_VALID_ONCE)
			reg_val |= SPIM_VALID_TABLE_VALID_ONCE_BIT;
		else
			reg_val |= SPIM_VALID_TABLE_VALID_BIT;

		switch (cmd_list[i]) {
		case CMD_EN4B:
			sys_write32(reg_val, config->ctrl_base + SPIM_VALID_LIST_BASE);
			continue;

		case CMD_EX4B:
			sys_write32(reg_val, config->ctrl_base + SPIM_VALID_LIST_BASE + 4);
			continue;
		default:
			idx++;
		}

		sys_write32(reg_val, config->ctrl_base + SPIM_VALID_LIST_BASE + idx * 4);
	}
}

void spim_rw_perm_init(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;
	uint32_t i;

	/* select write privilege */
	reg_val = sys_read32(config->ctrl_base);
	reg_val = (reg_val & 0x00ffffff) | SPIM_PRIV_WRITE_SELECT;
	sys_write32(reg_val, config->ctrl_base);
	for (i = 0; i < SPIM_ADDR_PRIV_REG_NUN; i++)
		sys_write32(0xffffffff, config->ctrl_base + SPIM_VALID_ADDR_FTR + i * 4);

	/* select read privilege */
	reg_val = sys_read32(config->ctrl_base);
	reg_val = (reg_val & 0x00ffffff) | SPIM_PRIV_READ_SELECT;
	sys_write32(reg_val, config->ctrl_base);
	for (i = 0; i < SPIM_ADDR_PRIV_REG_NUN; i++)
		sys_write32(0xffffffff, config->ctrl_base + SPIM_VALID_ADDR_FTR + i * 4);
}

void spim_scu_monitor_config(const struct device *dev, bool enable)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;

	reg_val = sys_read32(config->scu_base + SPIM_MODE_SCU_CTRL);
	if (enable)
		reg_val |= ((BIT(config->ctrl_num - 1)) << 8);
	else
		reg_val &= ~((BIT(config->ctrl_num - 1)) << 8);

	sys_write32(reg_val, config->scu_base + SPIM_MODE_SCU_CTRL);
}

void spim_ctrl_monitor_config(const struct device *dev, bool enable)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;

	reg_val = sys_read32(config->ctrl_base);
	if (enable)
		reg_val |= BIT(2);
	else
		reg_val &= ~(BIT(2));

	sys_write32(reg_val, config->ctrl_base);
}


static int aspeed_spi_monitor_init(const struct device *dev)
{
	struct aspeed_spim_data *data = dev->data;

	spim_config_passthrough_mode(dev, true);

	spim_fill_valid_table(dev, data->valid_cmd_list, data->valid_cmd_num, 0);

	/* spim_dump_valid_cmd_table(dev); */
	spim_rw_perm_init(dev);

	/* enable filter */
	spim_scu_monitor_config(dev, true);
	spim_ctrl_monitor_config(dev, true);

	return 0;
}

#define ASPEED_SPI_MONITOR_INIT(n)						\
	const uint8_t valid_cmd_list_##n[] = DT_INST_PROP(n, valid_cmds); \
	const uint32_t read_forbidden_regions_##n[] = DT_INST_PROP(n, read_forbidden_regions); \
	const uint32_t write_forbidden_regions_##n[] = DT_INST_PROP(n, write_forbidden_regions); \
	\
	static struct aspeed_spim_config aspeed_spim_config_##n = { \
		.ctrl_base = DT_INST_REG_ADDR(n),	\
		.scu_base = DT_REG_ADDR_BY_IDX(DT_INST_PHANDLE_BY_IDX(n, aspeed_scu, 0), 0), \
		.irq_num = DT_INST_IRQN(n),		\
		.irq_priority = DT_INST_IRQ(n, priority),	\
		.log_ram_addr = DT_INST_PROP_BY_IDX(n, log_ram_info, 0), \
		.log_max_len = DT_INST_PROP_BY_IDX(n, log_ram_info, 1),	\
		.ctrl_num = DT_INST_PROP(n, spi_monitor_num),	\
	};								\
									\
	static struct aspeed_spim_data aspeed_spim_data_##n = {	\
		.valid_cmd_list = &valid_cmd_list_##n[0], \
		.valid_cmd_num = DT_INST_PROP_LEN(n, valid_cmds), \
		.read_forbidden_regions = &read_forbidden_regions_##n[0],	\
		.read_forbidden_region_num = DT_INST_PROP_LEN(n, read_forbidden_regions),	\
		.write_forbidden_regions = &write_forbidden_regions_##n[0],	\
		.write_forbidden_region_num = DT_INST_PROP_LEN(n, write_forbidden_regions),	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, &aspeed_spi_monitor_init,			\
			    NULL,					\
			    &aspeed_spim_data_##n,			\
			    &aspeed_spim_config_##n, POST_KERNEL,	\
			    80,		\
			    NULL);			\

DT_INST_FOREACH_STATUS_OKAY(ASPEED_SPI_MONITOR_INIT)
