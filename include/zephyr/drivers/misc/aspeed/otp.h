/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

/*
 * @brief OTP driver APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_OTP_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_OTP_H_

#include <zephyr/device.h>
#include <errno.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>

/* The API a otp driver should implement */
__subsystem struct otp_driver_api {
	/* Setup a OTP session */
	int (*begin_session)(const struct device *dev, struct otp_info_cb *info_cb);

	/* Tear down an established session */
	int (*free_session)(const struct device *dev);

	/* Get OTP tool version */
	int (*get_tool_ver)(const struct device *dev, char *otp_ver_str);

	/* Get software revision ID */
	int (*get_sw_rid)(const struct device *dev, uint32_t *sw_rid);

	/* Get successful boot used key number */
	int (*get_key_num)(const struct device *dev, uint32_t *key_num);

	/* Get SoC chip revision ID */
	int (*get_chip_rid)(const struct device *dev, uint32_t *revid);

	/* Set OTP soak */
	int (*set_soak)(const struct device *dev, int soak);

	/* Read OTP memory */
	int (*otp_read)(const struct device *dev, uint32_t otp_addr, uint32_t *data);

	/* Program OTP memory by bit */
	int (*otp_program)(const struct device *dev, uint32_t otp_addr, uint32_t prog_bit);
};

/**
 * @brief Setup a otp session
 *
 * Initializes one time parameters, like the otp info,
 * which may remain constant for all operations in the session. The state
 * may be cached in hardware and/or driver data state variables.
 *
 * @param  dev      Pointer to the device structure for the driver instance.
 * @param  info_cb  Pointer to the otp context structure.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_begin_session(const struct device *dev,
				     struct otp_info_cb *info_cb)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->begin_session(dev, info_cb);
}

/**
 * @brief Cleanup a otp session
 *
 * Clears the hardware and/or driver state of a previous session.
 *
 * @param  dev      Pointer to the device structure for the driver instance.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_free_session(const struct device *dev)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->free_session(dev);
}

/**
 * @brief Perform otp tool version get
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  otp_ver_str   Pointer to char string.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_get_tool_ver(const struct device *dev, char *otp_ver_str)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->get_tool_ver(dev, otp_ver_str);
}

/**
 * @brief Perform software revision get
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  sw_rid        Pointer to sw rid.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_get_sw_rid(const struct device *dev, uint32_t *sw_rid)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->get_sw_rid(dev, sw_rid);
}

/**
 * @brief Perform key number get
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  key_num       Pointer to successful boot key number.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_get_key_num(const struct device *dev, uint32_t *key_num)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->get_key_num(dev, key_num);
}

/**
 * @brief Perform chip revision id get
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  revid         Pointer to SoC chip revision id.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_get_chip_rid(const struct device *dev, uint32_t *revid)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->get_chip_rid(dev, revid);
}

/**
 * @brief Perform otp soak set
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  soak          Level of otp soak.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_set_soak(const struct device *dev, int soak)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->set_soak(dev, soak);
}

/**
 * @brief Perform otp read
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  otp_addr      Address of OTP memory.
 * @param  data          Pointer to the contents of corresponding OTP memory.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_read(const struct device *dev, uint32_t otp_addr,
			   uint32_t *data)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->otp_read(dev, otp_addr, data);
}

/**
 * @brief Perform otp program
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  otp_addr      Address of OTP memory.
 * @param  prog_bit      The bit to be programmed.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_program(const struct device *dev, uint32_t otp_addr,
			      uint32_t prog_bit)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->otp_program(dev, otp_addr, prog_bit);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_OTP_H_ */
