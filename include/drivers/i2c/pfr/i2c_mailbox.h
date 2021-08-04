/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_MAILBOX_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_MAILBOX_H_

/* mail box define */
#define AST_I2C_M_DEV_COUNT	2
#define AST_I2C_M_WP_COUNT		8
#define AST_I2C_M_IRQ_COUNT		16

/* mail box fifo define */
#define AST_I2C_M_FIFO_COUNT	2
#define AST_I2C_M_FIFO_REMAP	0x100000
#define AST_I2C_M_FIFO0_MASK	0xFFFCFF00
#define AST_I2C_M_FIFO1_MASK	0xFFF300FF
#define AST_I2C_M_FIFO_R_NAK	0x10000000

/* mail box base define */
#define AST_I2C_M_BASE			0x3000

/* i2c device registers */
#define AST_I2C_CTL		0x00
#define AST_I2C_ADDR	0x40

/* device registers */
#define AST_I2C_M_CFG		0x04
#define AST_I2C_M_FIFO_IRQ	0x08
#define AST_I2C_M_IRQ_EN0	0x10
#define AST_I2C_M_IRQ_STA0	0x14
#define AST_I2C_M_IRQ_EN1	0x18
#define AST_I2C_M_IRQ_STA1	0x1C

/* (0) 0x20 ~ 0x3C /(1)0x40 ~ 0x5C */
#define AST_I2C_M_WP_BASE0	0x20
#define AST_I2C_M_WP_BASE1	0x40

#define AST_I2C_M_FIFO_CFG0	0x60
#define AST_I2C_M_FIFO_STA0	0x64
#define AST_I2C_M_FIFO_CFG1	0x68
#define AST_I2C_M_FIFO_STA1	0x6C
#define AST_I2C_M_ADDR_IRQ0	0x70
#define AST_I2C_M_ADDR_IRQ1	0x74
#define AST_I2C_M_ADDR_IRQ2	0x78
#define AST_I2C_M_ADDR_IRQ3	0x7C

/* i2c slave address control */
#define AST_I2CS_ADDR3_MBX_TYPE(x)	(x << 28)
#define AST_I2CS_ADDR2_MBX_TYPE(x)	(x << 26)
#define AST_I2CS_ADDR1_MBX_TYPE(x)	(x << 24)
#define AST_I2CS_ADDR3_ENABLE		BIT(23)
#define AST_I2CS_ADDR3(x)			((x & 0x7f) << 16)
#define AST_I2CS_ADDR2_ENABLE		BIT(15)
#define AST_I2CS_ADDR2(x)			((x & 0x7f) << 8)
#define AST_I2CS_ADDR1_ENABLE		BIT(7)
#define AST_I2CS_ADDR1(x)			(x & 0x7f)

#define	AST_I2CS_ADDR3_MASK	(0x7f << 16)
#define	AST_I2CS_ADDR2_MASK	(0x7f << 8)
#define	AST_I2CS_ADDR1_MASK	0x7f

#define	AST_I2CS_ADDR3_CLEAR	(AST_I2CS_ADDR3_MASK |\
AST_I2CS_ADDR3_ENABLE|AST_I2CS_ADDR3_MBX_TYPE(0x3))

#define	AST_I2CS_ADDR2_CLEAR	(AST_I2CS_ADDR2_MASK |\
AST_I2CS_ADDR2_ENABLE|AST_I2CS_ADDR2_MBX_TYPE(0x3))

#define	AST_I2CS_ADDR1_CLEAR	(AST_I2CS_ADDR1_MASK |\
AST_I2CS_ADDR1_ENABLE|AST_I2CS_ADDR1_MBX_TYPE(0x3))

/* i2c mailbox write protect element */
struct ast_i2c_m_wp_tbl {
	uint32_t		wp_element[AST_I2C_M_WP_COUNT];
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initial i2c mailbox device
 *
 * @param dev Pointer to the device structure for the driver instance
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or offset
 */
static int ast_i2c_mbx_init(const struct device *dev);

/**
 * @brief Set i2c mailbox device slave address
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param idx Index to the slave address
 * @param offset Offset to the slave address length
 * @param addr Address to the mbx device
 * @param enable Enable flag to the mbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or offset
 */
static int ast_i2c_mbx_addr(const struct device *dev, uint8_t idx,
uint8_t offset, uint8_t addr, uint8_t enable);

/**
 * @}
 */
#ifdef __cplusplus
	}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_MAILBOX_H_ */
