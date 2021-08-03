/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_i2c_mbx

#include <sys/util.h>
#include <sys/slist.h>
#include <kernel.h>
#include <errno.h>
#include <soc.h>
#include <device.h>
#include <string.h>
#include <stdlib.h>
#include <drivers/i2c/pfr/i2c_mailbox.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_pfr_mbx);

#define I2C_W_R(value, addr) LOG_INF("  dw %x %x", addr, value);
#define I2C_LW_R(value, addr) LOG_INF("  dw %x %lx", addr, value);
#define I2C_R(addr) LOG_INF("  dr %x", addr);

struct ast_i2c_mbx_data {
	uint32_t	i2c_dev_base;	/* i2c dev base*/
	uint8_t	mail_piroity;		/* mbx : piroity */
	uint8_t	mail_fifo_addr[AST_I2C_M_FIFO_COUNT];
	uint8_t	mail_fifo_rw_disable;
	uint8_t	mail_addr_irq[AST_I2C_M_IRQ_COUNT];
};

struct ast_i2c_mbx_config {
	char		*mail_dev_name;
	uint32_t	mail_g_base;
	uint8_t	mail_dev_idx;
	void (*irq_config_func)(const struct device *dev);
};

/* convenience defines */
#define DEV_CFG(dev)				     \
	((const struct ast_i2c_filter_config *const) \
	 (dev)->config)
#define DEV_DATA(dev) \
	((struct ast_i2c_filter_data *const)(dev)->data)

/* i2c mbx interrupt service routine */
static void ast_i2c_mbx_isr(const struct device *dev)
{
}

/* i2c mbx initial */
static int ast_i2c_mbx_init(const struct device *dev)
{
	return 0;
}

#define I2C_MBX_INIT(inst)				 \
	static void ast_i2c_mbx_cfg_##inst(const struct device *dev);	 \
			 \
	static struct ast_i2c_mbx_data			 \
		ast_i2c_filter_##inst##_dev_data;	 \
			 \
	static const struct ast_i2c_mbx_config		 \
		ast_i2c_mbx_##inst##_cfg = {		 \
		.mail_dev_name = DT_INST_PROP(inst, label),	 \
		.mail_g_base = DT_INST_REG_ADDR(inst),	 \
		.mail_dev_idx = DT_INST_PROP(inst, index),	 \
		.irq_config_func = ast_i2c_mail_cfg_##inst,	 \
	};		 \
			 \
	DEVICE_DT_INST_DEFINE(inst,		 \
			      &ast_i2c_mbx_init,	 \
			      NULL,				 \
			      &ast_i2c_mbx_##inst##_dev_data,	 \
			      &ast_i2c_mbx_##inst##_cfg,		 \
			      POST_KERNEL,			 \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	 \
			      NULL);								 \
			 \
	static void ast_i2c_mbx_cfg_##inst(const struct device *dev) \
	{									 \
		ARG_UNUSED(dev);					 \
										 \
		IRQ_CONNECT(DT_INST_IRQN(inst),	 \
				DT_INST_IRQ(inst, priority),	 \
				ast_i2c_mbx_isr, DEVICE_DT_INST_GET(inst), 0); \
			 \
		irq_enable(DT_INST_IRQN(inst));		 \
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_MBX_INIT)
