/*
 * Copyright 2025 Aspeed Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_LTPI_H_
#define ZEPHYR_INCLUDE_DRIVERS_LTPI_H_

#include <stdint.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <errno.h>

/* OCP_DC-SCM_2.0_LTPI_ver_1.0, Table 21 LTPI speed capability encoding */
#define LTPI_SP_CAP_25M				BIT(0)
#define LTPI_SP_CAP_50M				BIT(1)
#define LTPI_SP_CAP_75M				BIT(2)
#define LTPI_SP_CAP_100M			BIT(3)
#define LTPI_SP_CAP_150M			BIT(4)
#define LTPI_SP_CAP_200M			BIT(5)
#define LTPI_SP_CAP_250M			BIT(6)
#define LTPI_SP_CAP_300M			BIT(7)
#define LTPI_SP_CAP_400M			BIT(8)
#define LTPI_SP_CAP_600M			BIT(9)
#define LTPI_SP_CAP_800M			BIT(10)
#define LTPI_SP_CAP_1G				BIT(11)
#define LTPI_SP_CAP_500M			BIT(12)	/* Aspeed only */
/* --gap-- */
#define LTPI_SP_CAP_DDR				BIT(15)

/* LTPI standard registers */
struct ltpi_ctrl_status_regs {
	uint32_t link_status;			/* 0x000 */
	uint32_t link_detect_cap_local;		/* 0x004 */
	uint32_t link_detect_cap_remote;	/* 0x008 */
	uint32_t platform_id_local;		/* 0x00c */
	uint32_t platform_id_remote;		/* 0x010 */
	uint32_t advertise_cap_local_lo;	/* 0x014 */
	uint32_t advertise_cap_local_hi;	/* 0x018 */
	uint32_t advertise_cap_remote_lo;	/* 0x01c */
	uint32_t advertise_cap_remote_hi;	/* 0x020 */
	uint32_t default_config_lo;		/* 0x024 */
	uint32_t default_config_hi;		/* 0x028 */
	uint32_t link_align_err_cnt;		/* 0x02c */
	uint32_t link_lost_err_cnt;		/* 0x030 */
	uint32_t crc_err_cnt;			/* 0x034 */
	uint32_t unknown_comma_err_cnt;		/* 0x038 */
	uint32_t link_speed_to_err_cnt;		/* 0x03c */
	uint32_t config_acc_to_err_cnt;		/* 0x040 */
	uint32_t training_rx_fmt_cnt_lo;	/* 0x044 */
	uint32_t training_rx_fmt_cnt_hi;	/* 0x048 */
	uint32_t training_tx_fmt_cnt_lo;	/* 0x04c */
	uint32_t training_tx_fmt_cnt_hi;	/* 0x050 */
	uint32_t op_rx_fmt_cnt;			/* 0x054 */
	uint32_t op_tx_fmt_cnt;			/* 0x058 */
	uint32_t reserved[9];			/* 0x05c - 0x07f */
	uint32_t link_control;			/* 0x080 */
};

/* 0x000 link_status register */
#define LTPI_STATUS_LINK_STATE_LOCAL		GENMASK(19, 16)
#define LTPI_STATUS_LINK_STATE_REMOTE		GENMASK(15, 12)
#define LTPI_STATUS_LINK_SPEED			GENMASK(11, 8)
#define LTPI_STATUS_DDR_MODE			BIT(7)
#define LTPI_STATUS_CONFIG_ACC_TO_ERR		BIT(5)
#define LTPI_STATUS_LINK_SPEED_TO_ERR		BIT(4)
#define LTPI_STATUS_UNKNOWN_COMMA_ERR		BIT(3)
#define LTPI_STATUS_CRC_ERR			BIT(2)
#define LTPI_STATUS_LINK_LOST			BIT(1)
#define LTPI_STATUS_LINK_ALIGNED		BIT(0)

/* LTPI_STATUS_LINK_STATE_* */
enum ltpi_link_state {
	LINK_STATE_LINK_DETECT = 0,
	LINK_STATE_LINK_SPEED = 1,
	LINK_STATE_ADVERTISE = 2,
	LINK_STATE_CONFIGURATION = 3,
	LINK_STATE_OPERATIONAL = 4,
	LINK_STATE_RESERVED = 0xf,
};

/* LTPI_STATUS_LINK_SPEED */
enum ltpi_link_speed {
	LINK_SPEED_BASE_X1 = 0x0,
	LINK_SPEED_BASE_X2 = 0x1,
	LINK_SPEED_BASE_X3 = 0x2,
	LINK_SPEED_BASE_X4 = 0x3,
	LINK_SPEED_BASE_X6 = 0x4,
	LINK_SPEED_BASE_X8 = 0x5,
	LINK_SPEED_BASE_X10 = 0x6,
	LINK_SPEED_BASE_X12 = 0x7,
	LINK_SPEED_BASE_X16 = 0x8,
	LINK_SPEED_BASE_X24 = 0x9,
	LINK_SPEED_BASE_X32 = 0xa,
	LINK_SPEED_BASE_X40 = 0xb,
	LINK_SPEED_RESERVED = 0xf,
};

/* 0x004 link_detect_cap_local & 0x008 link_detect_cap_remote registers */
#define LTPI_LINK_SPEED_CAP			GENMASK(23, 8)

/* 0x080 link_control register */
#define LTPI_LINK_CTRL_AUTO_CONFIG		BIT(10)
#define LTPI_LINK_CTRL_SW_RESET			BIT(0)

/* LTPI APIs */
typedef int (*ltpi_api_do_link)(const struct device *dev, int timeout_ms);
typedef int (*ltpi_api_get_status)(const struct device *dev, struct ltpi_ctrl_status_regs **status);

/**
 * @brief ADC driver API
 *
 * This is the mandatory API any LTPI driver needs to expose.
 */
__subsystem struct ltpi_driver_api {
	ltpi_api_do_link do_link;
	ltpi_api_get_status get_status;
};

__syscall int ltpi_do_link(const struct device *dev, int timeout_ms);
static inline int z_impl_ltpi_do_link(const struct device *dev, int timeout_ms)
{
	const struct ltpi_driver_api *api = (const struct ltpi_driver_api *)dev->api;

	if (!api->do_link) {
		return -ENOSYS;
	}

	return api->do_link(dev, timeout_ms);
}

__syscall int ltpi_get_status(const struct device *dev, struct ltpi_ctrl_status_regs **status);
static inline int z_impl_ltpi_get_status(const struct device *dev,
					 struct ltpi_ctrl_status_regs **status)
{
	const struct ltpi_driver_api *api = (const struct ltpi_driver_api *)dev->api;

	if (!api->get_status) {
		return -ENOSYS;
	}

	return api->get_status(dev, status);
}

#include <zephyr/syscalls/ltpi.h>

#endif /* end of "#ifndef ZEPHYR_INCLUDE_DRIVERS_LTPI_H_" */
