/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_i2c_filter

#include <sys/util.h>
#include <sys/slist.h>
#include <kernel.h>
#include <errno.h>
#include <soc.h>
#include <device.h>
#include <string.h>
#include <stdlib.h>
#include <drivers/i2c/pfr/i2c_filter.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_pfr_filter);

/* i2c filter buf */
struct ast_i2c_f_tbl filter_tbl[AST_I2C_F_COUNT] NON_CACHED_BSS_ALIGN16;

struct ast_i2c_filter_data {
	uint32_t	filter_g_base;	/* i2cflt global base*/
	uint8_t	filter_dev_en;	/* i2cflt04 : filter enable */
	uint8_t	filter_en;		/* i2cflt0c : white list */
	uint8_t	filter_idx[AST_I2C_F_REMAP_SIZE];
	uint8_t	*filter_buf;
};

struct ast_i2c_filter_config {
	char		*filter_dev_name;
	uint32_t	filter_dev_base;
	uint8_t	filter_index;
	void (*irq_config_func)(const struct device *dev);
};

/* convenience defines */
#define DEV_CFG(dev)				     \
	((const struct ast_i2c_filter_config *const) \
	 (dev)->config)
#define DEV_DATA(dev) \
	((struct ast_i2c_filter_data *const)(dev)->data)

/* i2c filter interrupt service routine */
static void ast_i2c_filter_isr(const struct device *dev)
{
}
static int ast_i2c_filter_init(const struct device *dev)
{
	struct ast_i2c_filter_data *data = DEV_DATA(dev);
	const struct ast_i2c_filter_config *cfg = DEV_CFG(dev);

	if (!cfg->filter_dev_name) {
		LOG_ERR("i2c filter not found");
		return -EINVAL;
	}
	/* Fill the global base */
	data->filter_g_base = cfg->filter_dev_base & 0xFFFFF000;

	return 0;
}

#define I2C_FILTER_INIT(inst)				 \
	static void ast_i2c_filter_cfg_##inst(const struct device *dev);	 \
			 \
	static struct ast_i2c_filter_data			 \
		ast_i2c_filter_##inst##_dev_data;	 \
			 \
	static const struct ast_i2c_filter_config		 \
		ast_i2c_filter_##inst##_cfg = {		 \
		.filter_dev_name = DT_INST_PROP(inst, label),	 \
		.filter_dev_base = DT_INST_REG_ADDR(inst),	 \
		.filter_index = DT_INST_PROP(inst, index),		 \
		.irq_config_func = ast_i2c_filter_cfg_##inst,	 \
	};		 \
			 \
	DEVICE_DT_INST_DEFINE(inst,		 \
			      &ast_i2c_filter_init,	 \
			      NULL,				 \
			      &ast_i2c_filter_##inst##_dev_data,	 \
			      &ast_i2c_filter_##inst##_cfg,		 \
			      POST_KERNEL,			 \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	 \
			      NULL);								 \
			 \
	static void ast_i2c_filter_cfg_##inst(const struct device *dev) \
	{									 \
		ARG_UNUSED(dev);					 \
										 \
		IRQ_CONNECT(DT_INST_IRQN(inst),	 \
				DT_INST_IRQ(inst, priority),	 \
				ast_i2c_filter_isr, DEVICE_DT_INST_GET(inst), 0); \
			 \
		irq_enable(DT_INST_IRQN(inst));		 \
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_FILTER_INIT)
