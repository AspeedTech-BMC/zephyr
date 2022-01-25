/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_SLAVE_SWMBX_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_SLAVE_SWMBX_H_

/* swmbx define */
#define SWMBX_DEVICE_COUNT	0x10
#define SWMBX_INFO_BASE	0x7e7b0f00
#define SWMBX_PROTECT_COUNT	0x8
#define SWMBX_NOTIFY_COUNT	0x8

/* i2c controller define */
#define AST_I2CS_ADDR_CTRL		0x40
#define AST_I2CS_ADDR1_MASK	0x7f

/* swmbx flags define */
#define SWMBX_REGISTER		BIT(31)

/* enhance behavior flags define */
#define SWMBX_PROTECT		BIT(0)
#define SWMBX_NOTIFY		BIT(1)
#define FLAG_MASK		(BIT(0) | BIT(1))

/**
 * @brief I2C SW Mailbox Slave Driver API
 * @defgroup i2c_swmbx_slave_api i2c SW Mailbox slave driver API
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief Enable / Disable sw mailbox enhance behavior
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param item_flag Enhance behavior of the sw mbx device
 * @param enable Enable /Disable of the enhance behavior in sw mbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer
 */
int swmbx_enable_behavior(const struct device *dev, uint32_t item_flag, uint8_t enable);

/**
 * @brief Apply sw mailbox write protect by bit map
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param *bitmap Bitmap of the write protect in sw mbx device
 * @param start_idx Start index of the write protect bitmap in sw mbx device
 * @param num Number of bitmap index to the write protect in sw mbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or setting
 */
int swmbx_apply_protect(const struct device *dev, uint32_t *bitmap, uint8_t start_idx, uint8_t num);

/**
 * @brief Update sw mailbox write protect by address index
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param addr Address to the write protect in sw mbx device
 * @param enable Enable to the write protect in sw mbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or offset
 */
int swmbx_update_protect(const struct device *dev, uint8_t addr, uint8_t enable);

/**
 * @brief Set sw mailbox notify address
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param sem Pointer to the semaphore that is initiailed and sent when notify is trigger
 * @param idx Index to the notify internal index position (0x0~0x7)
 * @param addr Address to the write notify in mbx device
 * @param enable Enable to the notify in sw mbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or offset
 */
int swmbx_update_notify(const struct device *dev, struct k_sem *sem,
uint8_t idx, uint8_t addr, uint8_t enable);


/**
 * @}
 */


#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_SLAVE_SWMBX_H_ */
