/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SLAVE_SWMBX_H_
#define ZEPHYR_INCLUDE_DRIVERS_SLAVE_SWMBX_H_

/* swmbx define */
#define SWMBX_DEV_COUNT	0x2
#define SWMBX_BUF_BASE		0x7e7b0e00
#define SWMBX_PROTECT_COUNT	0x100
#define SWMBX_NOTIFY_COUNT	0x100
#define SWMBX_FIFO_COUNT	0x4

/* enhance behavior flags define */
#define SWMBX_PROTECT		BIT(0)
#define SWMBX_NOTIFY		BIT(1)
#define SWMBX_FIFO		BIT(2)
#define FLAG_MASK		(BIT(0) | BIT(1)|BIT(2))

/* fifo notify flags */
#define SWMBX_FIFO_NOTIFY_START	BIT(0)
#define SWMBX_FIFO_NOTIFY_STOP	BIT(1)
#define FIFO_NOTIFY_MASK	(BIT(0) | BIT(1))

/**
 * @brief I2C SW Mailbox Slave Driver API
 * @defgroup i2c_swmbx_slave_api i2c SW Mailbox slave driver API
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief write sw mailbox value
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param fifo append value into fifo
 * @param addr Address of swmbx
 * @param val value of swmbx
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer
 */
int swmbx_write(const struct device *dev, uint8_t fifo, uint8_t addr, uint8_t *val);

/**
 * @brief read sw mailbox value
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param fifo peak value from fifo
 * @param addr Address of swmbx
 * @param val value of swmbx
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer
 */
int swmbx_read(const struct device *dev, uint8_t fifo, uint8_t addr, uint8_t *val);

/**
 * @brief Enable / Disable sw mailbox enhance behavior
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param item_flag Enhance behavior of the swmbx device
 * @param enable Enable /Disable of the enhance behavior in swmbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer
 */
int swmbx_enable_behavior(const struct device *dev, uint32_t item_flag, uint8_t enable);

/**
 * @brief Apply sw mailbox write protect by bit map
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param port port index to the mail box device
 * @param *bitmap Bitmap of the write protect in swmbx device
 * @param start_idx Start index of the write protect bitmap in swmbx device
 * @param num Number of bitmap index to the write protect in swmbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or setting
 */
int swmbx_apply_protect(const struct device *dev, uint8_t port,
uint32_t *bitmap, uint8_t start_idx, uint8_t num);

/**
 * @brief Update sw mailbox write protect by address index
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param port port index to the mail box device
 * @param addr Address to the write protect in swmbx device
 * @param enable Enable to the write protect in swmbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or offset
 */
int swmbx_update_protect(const struct device *dev, uint8_t port,
uint8_t addr, uint8_t enable);

/**
 * @brief Set sw mailbox notify address
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param port port index to the mail box device
 * @param sem Pointer to the semaphore that is initialed and sent when notify is trigger
 * @param addr Address to the write notify in mbx device
 * @param enable Enable / Disable to the notify in swmbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or offset
 */
int swmbx_update_notify(const struct device *dev, uint8_t port,
struct k_sem *sem, uint8_t addr, uint8_t enable);

/**
 * @brief Flush sw mailbox fifo
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param idx Index to the fifo internal index position (0x0~0x7)
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or index
 */
int swmbx_flush_fifo(const struct device *dev, uint8_t idx);

/**
 * @brief Update sw mailbox fifo
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param sem Pointer to the semaphore that is initialed and sent when fifo boundary is trigger
 * @param idx Index to the fifo internal index position (0x0~0x7)
 * @param addr Address to the fifo in swmbx device
 * @param depth fifo depth to the fifo in swmbx device
 * @param notify fifo notify behavior flag
 * @param enable Enable / Disable to the fifo in swmbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or offset
 */
int swmbx_update_fifo(const struct device *dev, struct k_sem *sem,
uint8_t idx, uint8_t addr, uint8_t depth, uint8_t notify, uint8_t enable);


/**
 * @}
 */


#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_SLAVE_SWMBX_H_ */
