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

/* #define ASPEED_I2C_FJ_DUMP */
#define ASPEED_I2C_FW_DUMP

#ifdef ASPEED_I2C_FJ_DUMP
#define I2C_W_R(value, addr) LOG_INF("  dw %x %x", addr, value);
#define I2C_LW_R(value, addr) LOG_INF("  dw %x %lx", addr, value);
#else
#ifdef ASPEED_I2C_FW_DUMP
#define I2C_W_R(value, addr) LOG_INF("  dw %x %x\n", addr, value); sys_write32(value, addr);
#define I2C_LW_R(value, addr) LOG_INF("  dw %x %lx", addr, value); sys_write32(value, addr);
#else
#define I2C_W_R(value, addr) sys_write32(value, addr);
#define I2C_LW_R(value, addr) sys_write32(value, addr);
#endif
#endif

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
	uint8_t	filter_dev_idx;
	void (*irq_config_func)(const struct device *dev);
};

/* convenience defines */
#define DEV_CFG(dev)				     \
	((const struct ast_i2c_filter_config *const) \
	 (dev)->config)
#define DEV_DATA(dev) \
	((struct ast_i2c_filter_data *const)(dev)->data)

/* i2c filter interrupt service routine */
void ast_i2c_filter_isr(const struct device *dev)
{
	struct ast_i2c_filter_data *data = DEV_DATA(dev);

	uint8_t index = 0, i, infowp, inforp;
	uint32_t stsg = sys_read32(data->filter_g_base + AST_I2C_F_G_INT_STS);
	uint32_t filter_dev_base = 0, stsl = 0, sts = 0;
	uint32_t value = 0, count = 0;

	/* find which one local filter interrupt status */
	for (index = 0; index < AST_I2C_F_COUNT; index++) {
		/* local filter */
		if (stsg & (1 << index)) {
			filter_dev_base = (data->filter_g_base + (index * AST_I2C_F_D_OFFSET));
			stsl = sys_read32(filter_dev_base + AST_I2C_F_INT_STS);

			if (stsl) {
				sts = sys_read32(filter_dev_base + AST_I2C_F_STS);

				infowp = (sts & 0xF000) >> 12;
				inforp = (sts & 0x0F00) >> 8;

				/* calculte the information count */
				if (infowp > inforp)
					count = infowp - inforp;
				else
					count = (infowp + 0x10) - inforp;

				/* read back from  */
				for (i = 0; i < count; i++) {
					value = sys_read32(filter_dev_base + AST_I2C_F_INFO);
					LOG_INF(" dr %x", value);
				}

				/* clear status */
				I2C_W_R(stsl, (filter_dev_base + AST_I2C_F_INT_STS));

			}
		}
	}
}

/* i2c filter default */
int ast_i2c_filter_default(const struct device *dev, uint8_t pass)
{
	const struct ast_i2c_filter_config *cfg = DEV_CFG(dev);

	uint8_t i;
	uint32_t value = 0;
	struct ast_i2c_f_tbl *dev_wl_tbl = &(filter_tbl[(cfg->filter_dev_idx)]);
	struct ast_i2c_f_bitmap *bmp_buf = &(dev_wl_tbl->filter_tbl[0]);

	/* check parameter valid */
	if (!cfg->filter_dev_name) {
		LOG_ERR("i2c filter not found");
		return -EINVAL;
	}

	/* transation will be all pass */
	if (pass)
		value = 0xFFFFFFFF;

	/* fill pass or block bitmap table */
	for (i = 0; i < AST_I2C_F_ELEMENT_SIZE; i++) {
		bmp_buf->element[i] = value;
	}

	return 0;
}

/* i2c filter update */
int ast_i2c_filter_update(const struct device *dev, uint8_t idx, uint8_t addr,
struct ast_i2c_f_bitmap *table)
{
	struct ast_i2c_filter_data *data = DEV_DATA(dev);
	const struct ast_i2c_filter_config *cfg = DEV_CFG(dev);

	uint8_t i;
	uint8_t offset = idx >> 2;
	uint32_t *list_index = (uint32_t *)(data->filter_idx);
	struct ast_i2c_f_tbl *dev_wl_tbl = &(filter_tbl[(cfg->filter_dev_idx)]);
	struct ast_i2c_f_bitmap *bmp_buf = &(dev_wl_tbl->filter_tbl[0]);

	/* check parameter valid */
	if (!cfg->filter_dev_name) {
		LOG_ERR("i2c filter not found");
		return -EINVAL;
	} else if (idx > 15) {
		LOG_ERR("i2c filter index invalid");
		return -EINVAL;
	} else if (table == NULL) {
		LOG_ERR("i2c filter bitmap table is NULL");
		return -EINVAL;
	}

	/* fill re-map table and find the value pointer */
	data->filter_idx[idx] = addr;
	list_index += offset;

	switch (offset) {
	case 0:
		I2C_W_R((*list_index), (cfg->filter_dev_base + AST_I2C_F_MAP0));
		break;
	case 1:
		I2C_W_R((*list_index), (cfg->filter_dev_base + AST_I2C_F_MAP1));
		break;
	case 2:
		I2C_W_R((*list_index), (cfg->filter_dev_base + AST_I2C_F_MAP2));
		break;
	case 3:
		I2C_W_R((*list_index), (cfg->filter_dev_base + AST_I2C_F_MAP3));
		break;
	default:
		LOG_ERR("i2c filter invalid re-map index");
		return -EINVAL;
	}

	/* fill pass or block bitmap table */
	bmp_buf += (idx + 1);
	for (i = 0; i < AST_I2C_F_ELEMENT_SIZE; i++) {
		bmp_buf->element[i] = table->element[i];
	}

	return 0;
}


/* i2c filter enable */
int ast_i2c_filter_en(const struct device *dev, uint8_t filter_en, uint8_t wlist_en,
uint8_t clr_idx, uint8_t clr_tbl)
{
	struct ast_i2c_filter_data *data = DEV_DATA(dev);
	const struct ast_i2c_filter_config *cfg = DEV_CFG(dev);

	/* check parameter valid */
	if (!cfg->filter_dev_name) {
		LOG_ERR("i2c filter not found");
		return -EINVAL;
	}

	data->filter_dev_en = filter_en;
	data->filter_en = wlist_en;

	/* set white list buffer into device */
	if ((data->filter_dev_en) && (data->filter_en)) {
		I2C_LW_R(TO_PHY_ADDR(&(filter_tbl[(cfg->filter_dev_idx)])),
		(cfg->filter_dev_base+AST_I2C_F_BUF));
	}

	/* clear re-map index */
	if (clr_idx) {
		I2C_W_R(0x0, (cfg->filter_dev_base+AST_I2C_F_MAP0));
		I2C_W_R(0x0, (cfg->filter_dev_base+AST_I2C_F_MAP1));
		I2C_W_R(0x0, (cfg->filter_dev_base+AST_I2C_F_MAP2));
		I2C_W_R(0x0, (cfg->filter_dev_base+AST_I2C_F_MAP3));
	}

	/* clear white list table */
	if (clr_tbl) {
		I2C_W_R(0, (cfg->filter_dev_base+AST_I2C_F_BUF));
		data->filter_en = 0x0;
	}

	/* apply filter setting */
	I2C_W_R(data->filter_dev_en, (cfg->filter_dev_base+AST_I2C_F_EN));
	I2C_W_R(data->filter_en, (cfg->filter_dev_base+AST_I2C_F_CFG));

	return 0;
}

/* i2c filter initial */
int ast_i2c_filter_init(const struct device *dev)
{
	struct ast_i2c_filter_data *data = DEV_DATA(dev);
	const struct ast_i2c_filter_config *cfg = DEV_CFG(dev);
	uint8_t i = 0;
	uint32_t val = 0;

	/* check parameter valid */
	if (!cfg->filter_dev_name) {
		LOG_ERR("i2c filter not found");
		return -EINVAL;
	}

	/* fill the global base */
	data->filter_g_base = cfg->filter_dev_base & 0xFFFFF000;

	/* close filter first and fill initial timing setting */
	data->filter_dev_en = 0;
	I2C_W_R(0, (cfg->filter_dev_base+AST_I2C_F_EN));
	data->filter_en = 0;
	I2C_W_R(0, (cfg->filter_dev_base+AST_I2C_F_CFG));
	I2C_W_R(AST_I2C_F_TIMING_VAL, (cfg->filter_dev_base+AST_I2C_F_TIMING));

	/* clear and enable global interrupt */
	I2C_W_R(0x1F, (data->filter_g_base+AST_I2C_F_G_INT_STS));
	I2C_W_R(0x1, (data->filter_g_base+AST_I2C_F_G_INT_EN));

	/* clear and enable local interrupt */
	I2C_W_R(0x1, (cfg->filter_dev_base+AST_I2C_F_INT_STS));
	/* val = I2C_R(AST_I2C_F_INT_EN); */
	val |= 1 << (cfg->filter_dev_idx);
	I2C_W_R(val, (cfg->filter_dev_base+AST_I2C_F_INT_EN));

	/* clear filter re-map index table */
	for (i = 0; i < AST_I2C_F_REMAP_SIZE; i++) {
		data->filter_idx[i] = 0x0;
	}

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
		.filter_dev_idx = DT_INST_PROP(inst, index),	 \
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
