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
#define SWMBX_INFO_BASE	0x7e7b0f00
#define SWMBX_PROTECT_BITMAP	0x8
#define SWMBX_NODE_COUNT	0x100
#define SWMBX_FIFO_COUNT	0x4

/* enhance behavior flags define */
#define SWMBX_PROTECT		BIT(0)
#define SWMBX_NOTIFY		BIT(1)
#define SWMBX_FIFO		BIT(2)
#define FLAG_MASK		(BIT(0) | BIT(1) | BIT(2))

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
 * @brief send sw mailbox start from device slave
 *
 * @param port port index to the mail box device
 * @param addr address of swmbx
 *
 */
void swmbx_send_start(uint8_t port, uint8_t addr);

/**
 * @brief send sw mailbox message from device slave
 *
 * @param port port index to the mail box device
 * @param addr address of swmbx
 * @param val value of swmbx
 *
 */
void swmbx_send_msg(uint8_t port, uint8_t addr, uint8_t *val);

/**
 * @brief get sw mailbox message into device slave
 *
 * @param port port index to the mail box device
 * @param addr address of swmbx
 * @param val value of swmbx
 *
 */
void swmbx_get_msg(uint8_t port, uint8_t addr, uint8_t *val);

/**
 * @brief send sw mailbox stop from device slave
 *
 * @param port port index to the mail box device
 *
 */
void swmbx_send_stop(uint8_t port);

/**
 * @brief write sw mailbox value
 *
 * @param dev pointer to the device structure for the driver instance
 * @param fifo append value into fifo
 * @param addr address of swmbx
 * @param val value of swmbx
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer
 */
int swmbx_write(const struct device *dev, uint8_t fifo, uint8_t addr, uint8_t *val);

/**
 * @brief read sw mailbox value
 *
 * @param dev pointer to the device structure for the driver instance
 * @param fifo peak value from fifo
 * @param addr address of swmbx
 * @param val value of swmbx
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer
 */
int swmbx_read(const struct device *dev, uint8_t fifo, uint8_t addr, uint8_t *val);

/**
 * @brief Enable / Disable sw mailbox enhance behavior
 *
 * @param dev pointer to the device structure for the driver instance
 * @param item_flag enhance behavior of the swmbx device
 * @param enable enable /disable of the enhance behavior in swmbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer
 */
int swmbx_enable_behavior(const struct device *dev, uint32_t item_flag, uint8_t enable);

/**
 * @brief Apply sw mailbox write protect by bit map
 *
 * @param dev pointer to the device structure for the driver instance
 * @param port port index to the mail box device
 * @param *bitmap bitmap of the write protect in swmbx device
 * @param start_idx start index of the write protect bitmap in swmbx device
 * @param num number of bitmap index to the write protect in swmbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or setting
 */
int swmbx_apply_protect(const struct device *dev, uint8_t port,
uint32_t *bitmap, uint8_t start_idx, uint8_t num);

/**
 * @brief Update sw mailbox write protect by address index
 *
 * @param dev pointer to the device structure for the driver instance
 * @param port port index to the mail box device
 * @param addr address to the write protect in swmbx device
 * @param enable enable to the write protect in swmbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or offset
 */
int swmbx_update_protect(const struct device *dev, uint8_t port,
uint8_t addr, uint8_t enable);

/**
 * @brief Set sw mailbox notify address
 *
 * @param dev pointer to the device structure for the driver instance
 * @param port port index to the mail box device
 * @param sem pointer to the semaphore that is initialed and sent when notify is trigger
 * @param addr address to the write notify in mbx device
 * @param enable enable / disable to the notify in swmbx device
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or offset
 */
int swmbx_update_notify(const struct device *dev, uint8_t port,
struct k_sem *sem, uint8_t addr, uint8_t enable);

/**
 * @brief Flush sw mailbox fifo
 *
 * @param dev pointer to the device structure for the driver instance
 * @param idx Index to the fifo internal index position (0x0~0x3)
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid data pointer or index
 */
int swmbx_flush_fifo(const struct device *dev, uint8_t idx);

/**
 * @brief Update sw mailbox fifo
 *
 * @param dev pointer to the device structure for the driver instance
 * @param sem pointer to the semaphore that is initialed and sent when fifo boundary is trigger
 * @param idx index to the fifo internal index position (0x0~0x3)
 * @param addr address to the fifo in swmbx device
 * @param depth fifo depth to the fifo in swmbx device
 * @param notify fifo notify behavior flag
 * @param enable enable / disable to the fifo in swmbx device
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
