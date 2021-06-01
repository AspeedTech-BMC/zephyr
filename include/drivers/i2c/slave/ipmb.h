/**
 * @file
 *
 * @brief Public APIs for the I2C IPMB Slave driver.
 */

/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_I2C_SLAVE_IPMB_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2C_SLAVE_IPMB_H_

/* ipmb define*/
#define MAX_MSG_LEN		240
#define IPMB_REQUEST_LEN_MIN	7
#define NETFN_RSP_BIT_MASK	0x4
#define REQUEST_QUEUE_MAX_LEN	256

#define IPMB_MSG_LEN_IDX	0
#define RQ_SA_8BIT_IDX		1
#define NETFN_LUN_IDX		2

#define GET_7BIT_ADDR(addr_8bit)	(addr_8bit >> 1)
#define GET_8BIT_ADDR(addr_7bit)	((addr_7bit << 1) & 0xff)

#define IPMB_MSG_PAYLOAD_LEN_MAX (MAX_MSG_LEN - IPMB_REQUEST_LEN_MIN - 1)

#define SMBUS_MSG_HEADER_LENGTH	2
#define SMBUS_MSG_IDX_OFFSET	(SMBUS_MSG_HEADER_LENGTH + 1)

struct ipmb_msg {
	uint8_t len;
	uint8_t rs_sa;
	uint8_t netfn_rs_lun;
	uint8_t checksum1;
	uint8_t rq_sa;
	uint8_t rq_seq_rq_lun;
	uint8_t cmd;
	uint8_t payload[IPMB_MSG_PAYLOAD_LEN_MAX];
	/* checksum2 is included in payload */
} __packed;

/**
 * @brief I2C IPMB Slave Driver API
 * @defgroup i2c_ipmb_slave_api i2c IPMB slave driver API
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief Read single buffer of virtual IPMB memory
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param ipmb_data Pointer of byte where to store the virtual ipmb memory
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid data pointer or offset
 */
int ipmb_slave_read(const struct device *dev, struct ipmb_msg *ipmb_data);

/**
 * @brief Remove single buffer of virtual IPMB memory
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid data pointer or offset
 */
int ipmb_slave_remove(const struct device *dev);

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2C_SLAVE_IPMB_H_ */
