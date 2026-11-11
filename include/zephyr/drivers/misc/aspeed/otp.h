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
struct otp_region_protect;

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

	/*
	 * Get the raw, undecoded SCU1 ROM patch version register. Decoding
	 * differs between A1 and A2, so the caller decodes it, not this API.
	 */
	int (*get_rom_patch_version)(const struct device *dev, uint32_t *version);

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

	/*
	 * Get global ECC status (enum otp_ecc_global_status): distinguishes
	 * ECC being off from the two ways it can be forced on - temporarily
	 * by software, or latched by a hardware strap that can't be turned
	 * back off for the life of the session.
	 */
	int (*ecc_get_status)(const struct device *dev, int *ecc_status);

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

	/*
	 * Classify an OTP address into its region (enum otp_region_id).
	 * -ENOTSUP on targets with no region table (e.g. AST1040).
	 */
	int (*addr_to_region)(const struct device *dev, uint32_t addr, uint32_t *region);

	/*
	 * Read one OTP word raw (ECC decode forced off, regardless of the
	 * addressed region's own ECC policy) and return the hardware ECC
	 * code (OTP_DBG01[5:0]) captured by that same read. A non-zero code
	 * on a word means it was already burned with ECC parity - used to
	 * detect that without otherwise disturbing global/per-region ECC
	 * state.
	 */
	int (*read_ecc_code)(const struct device *dev, uint32_t offset,
			     uint32_t *data, uint32_t *ecc_code);

	/*
	 * Snapshot every OTPMISCREG region-protect register (lock/wen/ren,
	 * raw, undecoded) in one call. See struct otp_region_protect.
	 */
	int (*get_region_protect)(const struct device *dev, struct otp_region_protect *prot);
};

/*
 * Used with get_region_protect()/otp_get_region_protect(). Each field is a
 * raw OTPMISCREG value - bit31=LOCK, bits15:8=WEN, bits7:0=REN - except
 * .puf, which has its own 4-bit lock-only layout (bit8=sw_puf_rlock,
 * bit4=hw_puf_rlock, bit1=hw_puf_wlock, bit0=sw_puf_wlock). secure/usr/
 * caliptra additionally have a *_range field: bits31:16=size, bits15:0=
 * start offset, both in OTP words.
 *
 * Must stay in lockstep with the identically-shaped struct
 * otp_region_protect in u-boot/CA35's cmd/aspeed/otp_ast2700.c - over IPC,
 * otp_ipc.c slices this into OTP_IPC_OP_GET_REGION_PROTECT groups by
 * field order, and the CA35 client reassembles them in that same order.
 */
struct otp_region_protect {
	uint32_t rom_patch;
	uint32_t otpcfg;
	uint32_t otpstrap;
	uint32_t strap_ext;
	uint32_t secure0;
	uint32_t secure0_range;
	uint32_t secure1;
	uint32_t secure1_range;
	uint32_t secure2;
	uint32_t secure2_range;
	uint32_t secure3;
	uint32_t secure3_range;
	uint32_t usr0;
	uint32_t usr0_range;
	uint32_t usr1;
	uint32_t usr1_range;
	uint32_t usr2;
	uint32_t usr2_range;
	uint32_t usr3;
	uint32_t usr3_range;
	uint32_t caliptra0;
	uint32_t caliptra0_range;
	uint32_t caliptra1;
	uint32_t caliptra1_range;
	uint32_t caliptra2;
	uint32_t caliptra2_range;
	uint32_t caliptra3;
	uint32_t caliptra3_range;
	uint32_t rbp_soc_svn;
	uint32_t rbp_soc_keyretire;
	uint32_t rbp_calip_svn;
	uint32_t rbp_calip_keyretire;
	uint32_t puf;
};

/* Values for set_read_mode()/otp_set_read_mode() */
enum otp_read_mode {
	OTP_READ_MODE_NORMAL = 0,
	OTP_READ_MODE_MARGIN = 1,
};

/* Values for ecc_get_status()/otp_ecc_get_status() */
enum otp_ecc_global_status {
	OTP_ECC_GLOBAL_DISABLED = 0,
	OTP_ECC_GLOBAL_SW_ENABLED,
	OTP_ECC_GLOBAL_HW_ENABLED,
};

/*
 * OTP region ids, shared by ecc_get_policy()/ecc_set_policy()'s region
 * parameter and addr_to_region()'s output. Order is part of the ABI other
 * OTP IPC clients (CA35/SSP) rely on - never reorder or insert. Named
 * OTP_REGION_ID_* (not OTP_REGION_*) to avoid colliding with the unrelated,
 * differently-sized enum otp_region in lib_otp_ast2700.c, which C's flat
 * enumerator namespace would otherwise reject as a redefinition.
 */
enum otp_region_id {
	OTP_REGION_ID_ROM = 0,
	OTP_REGION_ID_RBP,
	OTP_REGION_ID_CFG,
	OTP_REGION_ID_STRAP,
	OTP_REGION_ID_STRAPEXT,
	OTP_REGION_ID_USR,
	OTP_REGION_ID_SEC,
	OTP_REGION_ID_CAL,
	OTP_REGION_ID_PUF,
	OTP_REGION_ID_MAX,
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
 * @brief Get the raw, undecoded SCU1 ROM patch version register
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  version       SCU1 ROM patch version register value.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_get_rom_patch_version(const struct device *dev, uint32_t *version)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->get_rom_patch_version(dev, version);
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
 * @brief Get the global ECC status
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  ecc_status    Set to enum otp_ecc_global_status.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_ecc_get_status(const struct device *dev, int *ecc_status)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->ecc_get_status(dev, ecc_status);
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

/**
 * @brief Classify an OTP address into its region
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  addr          Address of OTP memory.
 * @param  region        Set to the region id (enum otp_region_id) addr falls in.
 *
 * @return 0 on success, negative errno code on fail (e.g. -ENOTSUP if this
 *         target has no region table, or -EINVAL if addr is out of range).
 */
static inline int otp_addr_to_region(const struct device *dev, uint32_t addr, uint32_t *region)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->addr_to_region(dev, addr, region);
}

/**
 * @brief Read one OTP word raw (ECC decode forced off) and its hardware ECC code
 *
 * @param  dev           Pointer to the device structure for the driver instance.
 * @param  offset        Address of OTP memory.
 * @param  data          Set to the raw 16-bit word value.
 * @param  ecc_code      Set to OTP_DBG01[5:0] captured by the same read.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_read_ecc_code(const struct device *dev, uint32_t offset,
				    uint32_t *data, uint32_t *ecc_code)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->read_ecc_code(dev, offset, data, ecc_code);
}

/**
 * @brief Snapshot every OTPMISCREG region-protect register
 *
 * @param  dev   Pointer to the device structure for the driver instance.
 * @param  prot  Filled in with every region's raw lock/wen/ren register.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int otp_get_region_protect(const struct device *dev, struct otp_region_protect *prot)
{
	struct otp_driver_api *api;

	api = (struct otp_driver_api *)dev->api;

	return api->get_region_protect(dev, prot);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_OTP_H_ */
