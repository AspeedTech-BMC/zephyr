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
	uint32_t	snoop_dev_read;	/* i2c snoop device read pos */
	uint32_t	snoop_dev_write;	/* i2c snoop device write pos */
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
	return 0;
}

/* i2c snoop enable */
int ast_i2c_snoop_en(const struct device *dev, uint8_t snoop_en, uint8_t idx,
uint8_t addr)
{
	/* check parameter valid */
	if (idx >= AST_I2C_SP_DEV_COUNT) {
		LOG_ERR("i2c snoop not be support");
		return -EINVAL;
	}

	return 0;
}

/* i2c snoop initial */
int ast_i2c_snoop_init(const struct device *dev)
{
	struct ast_i2c_snoop_data *data = DEV_DATA(dev);

	data->snoop_dev_en = 0;
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
			&ast_i2c_snoop_##inst##_config, \
			POST_KERNEL, \
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			NULL); \
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_SNOOP_INIT)
