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

struct otp_info_cb;

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
	int (*get_chip_version)(const struct device *dev, uint32_t *chip_version);

	/* Get raw, undecoded SCU0/SCU1 revision ID registers */
	int (*get_chip_revid)(const struct device *dev, uint32_t *revid0, uint32_t *revid1);

	/* Set OTP soak */
	int (*set_soak)(const struct device *dev, int soak);

	/* Read OTP memory */
	int (*otp_read)(const struct device *dev, uint32_t otp_addr, uint32_t *data);

	/* Read multiple OTP memory */
	int (*otp_read_multi)(const struct device *dev, uint32_t otp_addr, void *data, int size);

	/* Program OTP memory by bit */
	int (*otp_program)(const struct device *dev, uint32_t otp_addr, uint32_t prog_bit);

	/* Program multiple OTP memory */
	int (*otp_program_multi)(const struct device *dev, uint32_t otp_addr, void *data, int size);

	/* Get global ECC force-enable status */
	int (*ecc_get_status)(const struct device *dev, int *ecc_en);

	/* Force ECC on/off globally, overriding per-region policy */
	int (*ecc_set_enable)(const struct device *dev, bool enable);

	/* Get per-region ECC policy */
	int (*ecc_get_policy)(const struct device *dev, uint32_t region,
			      bool *ecc_en, bool *ecc_supported);

	/* Set per-region ECC policy */
	int (*ecc_set_policy)(const struct device *dev, uint32_t region, bool ecc_en);

	/*
	 * Switch the OTP cell read reference current (DAP timing register)
	 * between normal and margin mode, for margin-screening diagnostics.
	 * Affects every otp_read()/otp_read_multi() call until switched back.
	 */
	int (*set_read_mode)(const struct device *dev, uint32_t mode);
};

/* Values for set_read_mode()/otp_set_read_mode() */
enum otp_read_mode {
	OTP_READ_MODE_NORMAL = 0,
	OTP_READ_MODE_MARGIN = 1,
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
 * @brief Perform chip version get
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  chip_version  SoC chip version.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_get_chip_version(const struct device *dev, uint32_t *chip_version)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->get_chip_version(dev, chip_version);
}

/**
 * @brief Get raw, undecoded SCU0/SCU1 revision ID registers
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  revid0        SCU0 revision ID register value.
 * @param  revid1        SCU1 revision ID register value.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_get_chip_revid(const struct device *dev, uint32_t *revid0,
				     uint32_t *revid1)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->get_chip_revid(dev, revid0, revid1);
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

/**
 * @brief Perform otp read multi
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  otp_addr      Address of OTP memory.
 * @param  data          Pointer to the contents of corresponding OTP memory.
 * @param  size          Size of data to be read.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_read_multi(const struct device *dev, uint32_t otp_addr,
				 void *data, int size)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->otp_read_multi(dev, otp_addr, data, size);
}

/**
 * @brief Perform otp program multi
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  otp_addr      Address of OTP memory.
 * @param  data          Pointer to the contents of corresponding OTP memory.
 * @param  size          Size of data to be programmed.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_program_multi(const struct device *dev, uint32_t otp_addr,
				    void *data, int size)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->otp_program_multi(dev, otp_addr, data, size);
}

/**
 * @brief Get the global ECC force-enable status
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  ecc_en        Set to 1 if ECC is forced on for every capable region, else 0.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_ecc_get_status(const struct device *dev, int *ecc_en)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->ecc_get_status(dev, ecc_en);
}

/**
 * @brief Force ECC on/off globally, overriding each region's own policy
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  enable        true to force ECC on, false to fall back to per-region policy.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_ecc_set_enable(const struct device *dev, bool enable)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->ecc_set_enable(dev, enable);
}

/**
 * @brief Get a region's ECC policy
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  region        Region id (enum otp_region_id).
 * @param  ecc_en        Set to the region's current ECC enable state.
 * @param  ecc_supported Set to whether the region supports ECC in hardware.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_ecc_get_policy(const struct device *dev, uint32_t region,
				     bool *ecc_en, bool *ecc_supported)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->ecc_get_policy(dev, region, ecc_en, ecc_supported);
}

/**
 * @brief Set a region's ECC policy
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  region        Region id (enum otp_region_id).
 * @param  ecc_en        ECC enable state to set for this region.
 *
 * @return 0 on success, negative errno code on fail (e.g. region doesn't support ECC).
 */
static inline int otp_ecc_set_policy(const struct device *dev, uint32_t region, bool ecc_en)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->ecc_set_policy(dev, region, ecc_en);
}

/**
 * @brief Switch the OTP cell read reference current between normal and
 *        margin mode (enum otp_read_mode), for margin-screening diagnostics
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  mode          enum otp_read_mode.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_set_read_mode(const struct device *dev, uint32_t mode)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->set_read_mode(dev, mode);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_OTP_H_ */
