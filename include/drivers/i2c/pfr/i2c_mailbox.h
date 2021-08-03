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
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid data pointer or offset
 */
static int ast_i2c_mbx_init(const struct device *dev);
/**
 * @}
 */
#ifdef __cplusplus
	}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_MAILBOX_H_ */
