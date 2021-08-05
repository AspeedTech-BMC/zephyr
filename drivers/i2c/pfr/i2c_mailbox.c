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
	uint8_t	mail_addr_en;	/* mbx addr enable*/
	uint8_t	mail_en;			/* mbx enable*/
	uint8_t	m_fifo_en;		/* mbx fifo enable*/
	uint8_t	mail_piroity;		/* mbx : piroity */
	uint8_t	mail_fifo_addr;
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
	((const struct ast_i2c_mbx_config *const) \
	 (dev)->config)
#define DEV_DATA(dev) \
	((struct ast_i2c_mbx_data *const)(dev)->data)

/* check common parameter valid */
int check_ast_mbx_valid(const struct ast_i2c_mbx_config *cfg)
{
	if (!cfg->mail_dev_name) {
		LOG_ERR("i2c mailbox not found");
		return -EINVAL;
	} else if ((cfg->mail_dev_idx != 0) &&
			(cfg->mail_dev_idx != 1)) {
		LOG_ERR("invalid i2c mailbox index");
		return -EINVAL;
	}

	return 0;
}

/* i2c mbx interrupt service routine */
static void ast_i2c_mbx_isr(const struct device *dev)
{
}

/* i2c mbx fifo enable */
static int ast_i2c_mbx_fifo_en(const struct device *dev, uint8_t idx,
uint16_t base, uint16_t length)
{
	const struct ast_i2c_mbx_config *cfg = DEV_CFG(dev);

	uint32_t value = (base) | (length << 16);
	uint32_t fifo_int = 0, value1;
	uint32_t fifo_sts = 0;

	/* check common parameter valid */
	if (check_ast_mbx_valid(cfg))
		return -EINVAL;

	value1 = sys_read32(cfg->mail_g_base + AST_I2C_M_FIFO_IRQ);
	fifo_int &= AST_I2C_M_FIFO_INT;
	fifo_sts &= AST_I2C_M_FIFO_STS;

	if (cfg->mail_dev_idx == 0) {

		value1 = (fifo_sts & AST_I2C_M_FIFO0_INT_STS);
		I2C_W_R(value1, cfg->mail_g_base + AST_I2C_M_FIFO_IRQ);

		/* length is used as enable */
		if (length) {
			fifo_int |= AST_I2C_M_FIFO0_INT_EN;
		} else {
			fifo_int &= ~(AST_I2C_M_FIFO0_INT_EN);
		}
		I2C_W_R(fifo_int, cfg->mail_g_base + AST_I2C_M_FIFO_IRQ);

		I2C_W_R(value, cfg->mail_g_base + AST_I2C_M_FIFO_CFG0);

	} else if (cfg->mail_dev_idx == 1) {

		value1 = (fifo_sts & AST_I2C_M_FIFO1_INT_STS);
		I2C_W_R(value1, cfg->mail_g_base + AST_I2C_M_FIFO_IRQ);

		/* length is used as enable */
		if (length) {
			fifo_int |= AST_I2C_M_FIFO1_INT_EN;
		} else {
			fifo_int &= ~(AST_I2C_M_FIFO1_INT_EN);
		}
		I2C_W_R(fifo_int, cfg->mail_g_base + AST_I2C_M_FIFO_IRQ);

		I2C_W_R(value, cfg->mail_g_base + AST_I2C_M_FIFO_CFG1);

	}

	return 0;
}

/* i2c mbx notify enable */
static int ast_i2c_mbx_notify_en(const struct device *dev, uint8_t idx,
uint8_t type, uint8_t enable)
{
	const struct ast_i2c_mbx_config *cfg = DEV_CFG(dev);

	uint8_t reg_offset = AST_I2C_M_IRQ_EN0;
	uint32_t value = 0;

	/* check common parameter valid */
	if (check_ast_mbx_valid(cfg))
		return -EINVAL;

	/* over notify index define */
	if (idx > 0xF)
		return -EINVAL;

	/* invalid type */
	if ((type < AST_I2C_M_R_NOTIFY) ||
	(type > (AST_I2C_M_R_NOTIFY | AST_I2C_M_W_NOTIFY)))
		return -EINVAL;

	if (cfg->mail_dev_idx == 1)
		reg_offset = AST_I2C_M_IRQ_EN1;

	/* calculate the interrupt flag position */
	value = sys_read32(cfg->mail_g_base + reg_offset);

	if (enable) {
		if (type & AST_I2C_M_R_NOTIFY)
			value |= (0x1 << (idx + 0x10));

		if (type & AST_I2C_M_W_NOTIFY)
			value |= (0x1 << idx);
	} else {
		if (type & AST_I2C_M_R_NOTIFY)
			value &= ~((0x1 << (idx + 0x10)));

		if (type & AST_I2C_M_W_NOTIFY)
			value &= ~(0x1 << idx);
	}

	I2C_W_R(value, cfg->mail_g_base + reg_offset);

	return 0;
}


/* i2c mbx notify address */
static int ast_i2c_mbx_notify_addr(const struct device *dev, uint8_t idx,
uint8_t addr)
{
	const struct ast_i2c_mbx_config *cfg = DEV_CFG(dev);

	uint8_t reg_index, reg_offset = AST_I2C_M_ADDR_IRQ0;
	uint32_t value, byteshift;

	/* check common parameter valid */
	if (check_ast_mbx_valid(cfg))
		return -EINVAL;

	/* over notify index define */
	if (idx > 0xF)
		return -EINVAL;

	/* calculte nodify address position */
	reg_index = idx >> 0x2;
	byteshift = ((idx % 0x4) << 3);

	reg_offset += (0x4 << reg_index);

	value = (sys_read32(cfg->mail_g_base + reg_offset)
	& ~(0xFF << byteshift));

	value |= (addr << byteshift);

	I2C_W_R(value, cfg->mail_g_base + reg_offset);

	return 0;
}


/* i2c mbx protect address */
static int ast_i2c_mbx_protect(const struct device *dev, uint8_t addr,
uint8_t enable)
{
	const struct ast_i2c_mbx_config *cfg = DEV_CFG(dev);

	uint8_t index, bit;
	uint32_t value, base = AST_I2C_M_WP_BASE0;

	/* check common parameter valid */
	if (check_ast_mbx_valid(cfg))
		return -EINVAL;

	/* change write protect base */
	if (cfg->mail_dev_idx == 0x1)
		base = AST_I2C_M_WP_BASE1;

	/* calculte bitmap position */
	index = addr / 0x20;
	bit = addr % 0x20;

	base += (index << 2);

	value = sys_read32(cfg->mail_g_base + base);

	if (enable)
		value |= (0x1 << bit);
	else
		value = value & ~(0x1 << bit);

	I2C_W_R(value, cfg->mail_g_base + base);

	return 0;
}

/* i2c mbx enable */
static int ast_i2c_mbx_en(const struct device *dev, uint32_t base,
uint16_t length, uint8_t enable)
{
	struct ast_i2c_mbx_data *data = DEV_DATA(dev);
	const struct ast_i2c_mbx_config *cfg = DEV_CFG(dev);

	uint32_t value = 0;

	/* check common parameter valid */
	if (check_ast_mbx_valid(cfg))
		return -EINVAL;

	/* set mbx slave address before enable it */
	if (enable && (!(data->mail_addr_en)))
		return -EINVAL;

	/* set mbx base and length */
	if (enable) {
		I2C_W_R(base, data->i2c_dev_base + AST_I2C_RX_DMA);
		I2C_W_R(base, data->i2c_dev_base + AST_I2C_TX_DMA);
		I2C_W_R(0x400, data->i2c_dev_base + AST_I2C_MBX_LIM);

		value = AST_I2C_MBX_RX_DMA_LEN(length);
		I2C_W_R(value, data->i2c_dev_base + AST_I2C_DMA_LEN);
		value = AST_I2C_MBX_TX_DMA_LEN(length);
		I2C_W_R(value, data->i2c_dev_base + AST_I2C_DMA_LEN);
		value = (sys_read32(data->i2c_dev_base + AST_I2C_CTL)
			|AST_I2C_MBX_EN);
		I2C_W_R(value, data->i2c_dev_base + AST_I2C_CTL);
	} else {
		value = (sys_read32(data->i2c_dev_base + AST_I2C_CTL)
		& (~AST_I2C_MBX_EN));
		I2C_W_R(value, data->i2c_dev_base + AST_I2C_CTL);

		/* reset dma offset */
		value = (sys_read32(data->i2c_dev_base + AST_I2C_CTL)
		& (~AST_I2C_MASTER_EN));
		I2C_W_R(value, data->i2c_dev_base + AST_I2C_CTL);
		value |= AST_I2C_MASTER_EN;
		I2C_W_R(value, data->i2c_dev_base + AST_I2C_CTL);
	}

	data->mail_en = enable;

	return 0;
}

/* i2c mbx set addr */
static int ast_i2c_mbx_addr(const struct device *dev, uint8_t idx,
uint8_t offset, uint8_t addr, uint8_t enable)
{
	struct ast_i2c_mbx_data *data = DEV_DATA(dev);
	const struct ast_i2c_mbx_config *cfg = DEV_CFG(dev);

	uint32_t mask_addr = 0;

	/* check common parameter valid */
	if (check_ast_mbx_valid(cfg))
		return -EINVAL;

	/* Set slave addr. */
	switch (idx) {
	case 0:
		mask_addr = (sys_read32(data->i2c_dev_base + AST_I2C_ADDR)
			& ~(AST_I2CS_ADDR1_CLEAR));
		if (enable) {
			mask_addr |= (AST_I2CS_ADDR1(addr)|
			AST_I2CS_ADDR1_MBX_TYPE(offset)|AST_I2CS_ADDR1_ENABLE);
		}
		break;
	case 1:
		mask_addr = (sys_read32(data->i2c_dev_base + AST_I2C_ADDR)
			& ~(AST_I2CS_ADDR2_CLEAR));
		if (enable) {
			mask_addr |= (AST_I2CS_ADDR2(addr)|
			AST_I2CS_ADDR2_MBX_TYPE(offset)|AST_I2CS_ADDR2_ENABLE);
		}
		break;
	case 2:
		mask_addr = (sys_read32(data->i2c_dev_base + AST_I2C_ADDR)
			& ~(AST_I2CS_ADDR3_CLEAR));
		if (enable) {
			mask_addr |= (AST_I2CS_ADDR3(addr)|
			AST_I2CS_ADDR3_MBX_TYPE(offset)|AST_I2CS_ADDR3_ENABLE);
		}
		break;
	default:
		return -EINVAL;
	}

	/* fire in register */
	I2C_W_R(mask_addr, data->i2c_dev_base + AST_I2C_ADDR);

	/* find out if any one i2c slave is existed */
	mask_addr = (sys_read32(data->i2c_dev_base + AST_I2C_ADDR)
		& (AST_I2CS_ADDR1_ENABLE|
		AST_I2CS_ADDR2_ENABLE|
		AST_I2CS_ADDR3_ENABLE));

	if (mask_addr) {
		data->mail_addr_en = 1;
	} else {
		data->mail_addr_en = 0;
	}

	return 0;
}

/* i2c mbx initial */
static int ast_i2c_mbx_init(const struct device *dev)
{
	struct ast_i2c_mbx_data *data = DEV_DATA(dev);
	const struct ast_i2c_mbx_config *cfg = DEV_CFG(dev);

	uint32_t sts = 0;

	/* check common parameter valid */
	if (check_ast_mbx_valid(cfg))
		return -EINVAL;

	/* clear mbx addr /fifo irq status */
	I2C_W_R(0xFFFFFFFF, (cfg->mail_g_base+AST_I2C_M_IRQ_STA0));
	I2C_W_R(0xFFFFFFFF, (cfg->mail_g_base+AST_I2C_M_IRQ_STA1));
	sts =  sys_read32(cfg->mail_g_base + AST_I2C_M_FIFO_IRQ);
	I2C_W_R(sts, (cfg->mail_g_base+AST_I2C_M_FIFO_IRQ));

	/* i2c device base for slave setting */
	data->i2c_dev_base = (cfg->mail_g_base & 0xFFFF0000);
	data->i2c_dev_base += (0x80 * (cfg->mail_dev_idx + 1));

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
