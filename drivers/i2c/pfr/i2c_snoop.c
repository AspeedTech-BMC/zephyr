/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_i2c_snoop

#include <sys/util.h>
#include <sys/slist.h>
#include <kernel.h>
#include <errno.h>
#include <soc.h>
#include <device.h>
#include <string.h>
#include <stdlib.h>
#include <drivers/i2c/pfr/i2c_snoop.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_pfr_snoop);

#define ASPEED_I2C_SNW_DUMP

#ifdef ASPEED_I2C_SNW_DUMP
#define I2C_W_R(value, addr) LOG_INF("  dw %x %x", addr, value); sys_write32(value, addr);
#define I2C_LW_R(value, addr) LOG_INF("  dw %x %lx", addr, value); sys_write32(value, addr);
#else
#define I2C_W_R(value, addr) sys_write32(value, addr);
#define I2C_LW_R(value, addr) sys_write32(value, addr);
#endif

/* i2c snoop message queue*/
uint8_t snoop_msg0[AST_I2C_SP_MSG_COUNT] NON_CACHED_BSS_ALIGN16;
uint8_t snoop_msg1[AST_I2C_SP_MSG_COUNT] NON_CACHED_BSS_ALIGN16;

struct ast_i2c_snoop_data {
	uint8_t	snoop_dev_en;	/* i2c snoop device enable */
	uint8_t	*snoop_buf;		/* i2c snoop base */
	uint32_t	snoop_dev_read;	/* i2c snoop device read pos */
	uint32_t	snoop_dev_write;	/* i2c snoop device write pos */
	uint32_t	snoop_dev_max;	/* i2c snoop device max size */
};

struct ast_i2c_snoop_config {
	uint32_t	snoop_dev_base;	/* i2c snoop device base */
};

/* convenience defines */
#define DEV_CFG(dev)				     \
	((const struct ast_i2c_snoop_config *const) \
	(dev)->config)
#define DEV_DATA(dev) \
	((struct ast_i2c_snoop_data *const)(dev)->data)
#define DEV_BASE(dev) \
	((DEV_CFG(dev))->snoop_dev_base)

/* i2c snoop enable */
int ast_i2c_snoop_update(const struct device *dev, uint32_t size)
{
	uint32_t base = DEV_BASE(dev);
	struct ast_i2c_snoop_data *data = DEV_DATA(dev);
	uint32_t still_size = 0, remain_size = 0;
	uint32_t sp_R = sys_read32(base + AST_I2C_SP_DMA_RPT);
	uint32_t sp_W = sys_read32(base + AST_I2C_SP_DMA_WPT);

	if (!data->snoop_dev_en) {
		LOG_ERR("i2c snoop would not be enable");
		return -EINVAL;
	}

	LOG_DBG("sp R: %x", sp_R);
	LOG_DBG("sp W: %x", sp_W);

	if (sp_R != sp_W) {
		if (data->snoop_dev_read + size <= data->snoop_dev_max) {
			/* read from read pos */
			/* normal case */
			I2C_W_R(0, (base+AST_I2C_SP_DMA_RPT));
			LOG_DBG("normal sp R: %x", sys_read32(base + AST_I2C_SP_DMA_RPT));
			data->snoop_dev_read += size;
		} else {
			/* loop case */
			still_size = data->snoop_dev_max - data->snoop_dev_read;
			LOG_DBG("still_size : %x", still_size);
			/* read from read pos */
			remain_size = size - still_size;
			LOG_DBG("remain_size : %x", remain_size);
			/* read from base */
			data->snoop_dev_read = (uint32_t)(data->snoop_buf) + remain_size;
			I2C_W_R(0, (base+AST_I2C_SP_DMA_RPT));
			LOG_DBG("loop sp R: %x", sys_read32(base + AST_I2C_SP_DMA_RPT));
		}
		LOG_DBG("R 0x54: %x", sys_read32(base + 0x54));
		LOG_DBG("R 0x28: %x", sys_read32(base + 0x28));
	}

	return 0;
}

/* i2c snoop enable */
int ast_i2c_snoop_en(const struct device *dev, uint8_t snoop_en, uint8_t idx,
uint8_t filter_idx, uint8_t addr)
{
	uint32_t base = DEV_BASE(dev);
	struct ast_i2c_snoop_data *data = DEV_DATA(dev);
	uint32_t sp_addr;
	uint8_t sp_switch = sys_read8(AST_I2C_SP_SWITCH);

	/* check parameter valid */
	if (idx >= AST_I2C_SP_DEV_COUNT) {
		LOG_ERR("i2c snoop not be support");
		return -EINVAL;
	} else if (filter_idx > AST_I2C_F_COUNT) {
		LOG_ERR("i2c filter not be support");
		return -EINVAL;
	}
	data->snoop_dev_en = snoop_en;

	/* set i2c slave addr */
	sp_addr =  sys_read32(base + AST_I2C_ADDR_CTRL) & ~(AST_I2CS_ADDR_CLEAR);

	if (snoop_en)
		sp_addr |= (AST_I2C_SP_ADDR(addr)|AST_I2C_ADDR_ENABLE);

	I2C_W_R(sp_addr, (base+AST_I2C_ADDR_CTRL));

	/* set i2c snoop */
	if (snoop_en) {
		/* close interrupt */
		I2C_W_R(0, (base+AST_I2CS_IER));

		/* fill snoop buffer and switch control */
		if (idx == 0) {
			data->snoop_buf = &(snoop_msg0[0]);
			sp_switch &= ~(0xF);
			sp_switch |= (filter_idx + 1);
		} else {
			data->snoop_buf = &(snoop_msg1[0]);
			sp_switch &= ~(0xF0);
			sp_switch |= (filter_idx + 1) << 1;
		}

		data->snoop_dev_read = (uint32_t) data->snoop_buf;
		data->snoop_dev_max = data->snoop_dev_read + AST_I2C_SP_MSG_COUNT;

		sys_write8(sp_switch, AST_I2C_SP_SWITCH);

		I2C_LW_R((TO_PHY_ADDR(data->snoop_buf)), (base + AST_I2C_RX_DMA_BUF));

		/* re-set read pos */
		I2C_W_R(0, (base+AST_I2C_SP_DMA_RPT));

		/* set dma size */
		I2C_LW_R(AST_I2CS_SET_RX_DMA_LEN(AST_I2C_SP_MSG_COUNT), (base+AST_I2C_RX_DMA_LEN));

		/* set snoop function */
		I2C_LW_R(AST_I2C_SP_CMD, (base+AST_I2CS_CMD));

		/* set i2c slave support*/
		I2C_LW_R(AST_I2C_S_EN | sys_read32(base + AST_I2C_CTRL), base + AST_I2C_CTRL);
	} else {

		/* clear snoop function */
		I2C_LW_R(~(AST_I2C_SP_CMD) & sys_read32(base + AST_I2CS_CMD), base + AST_I2CS_CMD);

		/* re-set dma size */
		I2C_LW_R(~(AST_I2C_S_EN | AST_I2C_M_EN) &
		sys_read32(base + AST_I2C_CTRL), base + AST_I2C_CTRL);

		/* set master back */
		I2C_LW_R((AST_I2C_M_EN) | sys_read32(base + AST_I2C_CTRL), base + AST_I2C_CTRL);
	}

	return 0;
}

/* i2c snoop initial */
int ast_i2c_snoop_init(const struct device *dev)
{
	struct ast_i2c_snoop_data *data = DEV_DATA(dev);

	data->snoop_dev_en = 0;
	data->snoop_buf = NULL;
	data->snoop_dev_read = 0;
	data->snoop_dev_write = 0;

	return 0;
}

#define I2C_SNOOP_INIT(inst)				 \
	static struct ast_i2c_snoop_data			 \
		ast_i2c_snoop_##inst##_data;	 \
	 \
	static const struct ast_i2c_snoop_config		 \
		ast_i2c_snoop_##inst##_cfg = {		 \
		.snoop_dev_base = DT_INST_REG_ADDR(inst),	 \
	}; \
	 \
	DEVICE_DT_INST_DEFINE(inst, \
			&ast_i2c_snoop_init, \
			NULL, \
			&ast_i2c_snoop_##inst##_data, \
			&ast_i2c_snoop_##inst##_cfg, \
			POST_KERNEL, \
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			NULL); \

DT_INST_FOREACH_STATUS_OKAY(I2C_SNOOP_INIT)
