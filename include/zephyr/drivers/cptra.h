/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CPTRA_H_
#define ZEPHYR_INCLUDE_DRIVERS_CPTRA_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

/* SCU register offsets */
#define SCU1_CPTRA				0x130
#define   SCU1_CPTRA_RDY_FOR_RT			BIT(18)

/* CPTRA MBOX register offsets */
#define CPTRA_MBOX_LOCK				0x00
#define CPTRA_MBOX_USER				0x04
#define CPTRA_MBOX_CMD				0x08
#define CPTRA_MBOX_DLEN				0x0c
#define CPTRA_MBOX_DATAIN			0x10
#define CPTRA_MBOX_DATAOUT			0x14
#define CPTRA_MBOX_EXEC				0x18
#define CPTRA_MBOX_STS				0x1c
#define   CPTRA_MBOX_STS_SOC_LOCK		BIT(9)
#define   CPTRA_MBOX_STS_FSM_PS			GENMASK(8, 6)
#define   CPTRA_MBOX_STS_PS			GENMASK(3, 0)
#define CPTRA_MBOX_UNLOCK			0x20

#define CPTRA_ECDSA_SIG_LEN			96	/* ECDSA384 */
#define CPTRA_ECDSA_SHA_LEN			48	/* SHA384 */
#define CPTRA_MBOX_SZ				0x20000	/* 128KB */

/* Mailbox commands */
#define CPTRA_MBCMD_ECDSA384_SIGNATURE_VERIFY	0x53494756
#define CPTRA_MBCMD_CALIPTRA_FW_LOAD		0x46574C44

union cptra_mbox_lock_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t lock : 1;		/*[0-0]*/
		volatile uint32_t reserved : 31;	/*[1-31]*/
	} fields;
}; /* 0x00 */

union cptra_mbox_user_s {
	volatile uint32_t value;
}; /* 0x04 */

union cptra_mbox_cmd_s {
	volatile uint32_t value;
}; /* 0x08 */

union cptra_mbox_dlen_s {
	volatile uint32_t value;
}; /* 0x0C */

union cptra_mbox_datain_s {
	volatile uint32_t value;
}; /* 0x10 */

union cptra_mbox_dataout_s {
	volatile uint32_t value;
}; /* 0x14 */

union cptra_mbox_exec_s {
	volatile uint32_t value;
}; /* 0x18 */

union cptra_mbox_sts_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t status : 4;			/*[0-3]*/
		volatile uint32_t ecc_single_error : 1;		/*[4-4]*/
		volatile uint32_t ecc_double_error : 1;		/*[5-5]*/
		volatile uint32_t mbox_fsm_ps : 3;		/*[6-8]*/
		volatile uint32_t soc_has_lock : 1;		/*[9-9]*/
		volatile uint32_t mbox_rdptr : 15;		/*[10-24]*/
		volatile uint32_t reserved : 7;			/*[25-31]*/
	} fields;
}; /* 0x1C */

union cptra_mbox_unlock_s {
	volatile uint32_t value;
}; /* 0x20 */

struct cptra_mbox_register_s {
	union cptra_mbox_lock_s mbox_lock;			/* 00 */
	union cptra_mbox_user_s mbox_user;			/* 04 */
	union cptra_mbox_cmd_s mbox_cmd;			/* 08 */
	union cptra_mbox_dlen_s mbox_dlen;			/* 0C */
	union cptra_mbox_datain_s mbox_datain;			/* 10 */
	union cptra_mbox_dataout_s mbox_dataout;		/* 14 */
	union cptra_mbox_exec_s mbox_exec;			/* 18 */
	union cptra_mbox_sts_s mbox_sts;			/* 1C */
	union cptra_mbox_unlock_s mbox_unlock;			/* 20 */
};

enum cptra_mbox_sts {
	CPTRA_MBSTS_CMD_BUSY,
	CPTRA_MBSTS_DATA_READY,
	CPTRA_MBSTS_CMD_COMPLETE,
	CPTRA_MBSTS_CMD_FAILURE,
};

enum cptra_mbox_fsm {
	CPTRA_MBFSM_IDLE,
	CPTRA_MBFSM_RDY_FOR_CMD,
	CPTRA_MBFSM_RDY_FOR_DLEN,
	CPTRA_MBFSM_RDY_FOR_DATA,
	CPTRA_MBFSM_EXEC_UC,
	CPTRA_MBFSM_EXEC_SOC,
	CPTRA_MBFSM_ERROR,
};

/* SHA register offsets */
#define CPTRA_SHA_LOCK				0x00
#define CPTRA_SHA_USER				0x04
#define CPTRA_SHA_MODE				0x08
#define   CPTRA_SHA_MODE_ENDIAN			BIT(2)
#define   CPTRA_SHA_MODE_SEL			GENMASK(1, 0)
#define CPTRA_SHA_DLEN				0x10
#define CPTRA_SHA_DATAIN			0x14
#define CPTRA_SHA_EXEC				0x18
#define CPTRA_SHA_STS				0x1c
#define   CPTRA_SHA_STS_SOC_LOCK		BIT(1)
#define   CPTRA_SHA_STS_VLD			BIT(0)
#define CPTRA_SHA_DIGEST(n)			(0x20 + ((n) << 2))
#define CPTRA_SHA_CTRL				0x60
#define   CPTRA_SHA_CTRL_ZEROIZE		BIT(0)

union cptra_sha_lock_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t lock : 1;			/*[0-0]*/
		volatile uint32_t reserved : 31;		/*[1-31]*/
	} fields;
}; /* 0x00 */

union cptra_sha_user_s {
	volatile uint32_t value;
}; /* 0x04 */

union cptra_sha_mode_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t mode : 2;			/*[0-1]*/
		volatile uint32_t endian_toggle : 1;		/*[2-2]*/
		volatile uint32_t reserved : 29;		/*[3-31]*/
	} fields;
}; /* 0x08 */

union cptra_sha_addr_s {
	volatile uint32_t value;
}; /* 0x0C */

union cptra_sha_dlen_s {
	volatile uint32_t value;
}; /* 0x10 */

union cptra_sha_datain_s {
	volatile uint32_t value;
}; /* 0x14 */

union cptra_sha_exec_s {
	volatile uint32_t value;
}; /* 0x18 */

union cptra_sha_sts_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t valid : 1;			/*[0-0]*/
		volatile uint32_t soc_has_lock : 1;		/*[1-1]*/
		volatile uint32_t reserved : 30;		/*[2-31]*/
	} fields;
}; /* 0x1C */

union cptra_sha_digest_s {
	volatile uint32_t value;
}; /* 0x20 */

union cptra_sha_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t zeroize : 1;			/*[0-0]*/
		volatile uint32_t reserved : 31;		/*[1-31]*/
	} fields;
}; /* 0x60 */

struct cptra_sha_register_s {
	union cptra_sha_lock_s sha_lock;			/* 00 */
	union cptra_sha_user_s sha_user;			/* 04 */
	union cptra_sha_mode_s sha_mode;			/* 08 */
	union cptra_sha_addr_s sha_addr;			/* 0C */
	union cptra_sha_dlen_s sha_dlen;			/* 10 */
	union cptra_sha_datain_s sha_datain;			/* 14 */
	union cptra_sha_exec_s sha_exec;			/* 18 */
	union cptra_sha_sts_s sha_sts;				/* 1C */
	union cptra_sha_digest_s sha_digest[16];		/* 20 */
	union cptra_sha_ctrl_s sha_ctrl;			/* 60 */
};

/* SoC ifc register offsets */
#define CPTRA_HW_ERROR_FATAL			0x000
#define CPTRA_HW_ERROR_NONFATAL			0x004
#define CPTRA_FW_ERROR_FATAL			0x008
#define CPTRA_FW_ERROR_NONFATAL			0x00c
#define CPTRA_HW_ERROR_ENC			0x010
#define CPTRA_FW_ERROR_ENC			0x014
#define CPTRA_FW_EXT_ERROR_INFO			0x018
#define CPTRA_BOOT_STS				0x038
#define CPTRA_FLOW_STS				0x03c
#define CPTRA_BOOT_STS				0x038
#define CPTRA_FLOW_STS				0x03c
#define   CPTRA_FLOW_STS_RDY_FOR_FUSES		BIT(30)
#define   CPTRA_FLOW_STS_RDY_FOR_RT		BIT(29)
#define   CPTRA_FLOW_STS_RDY_FOR_FW		BIT(28)
#define CPTRA_RST_REASON			0x040
#define   CPTRA_FW_UPD_RESET			BIT(0)
#define   CPTRA_WARM_RESET			BIT(1)
#define CPTRA_TRNG_DATA(n)			(0x078 + ((n) << 2))
#define CPTRA_TRNG_STS				0x0ac
#define   CPTRA_TRNG_STS_DATA_WR_DONE		BIT(1)
#define   CPTRA_TRNG_STS_DATA_REQ		BIT(0)

#define CPTRA_MAX_TRNG				12

#define CPTRA_UPD_RST_TIMEOUT			1000
#define CPTRA_TRNG_REQ_LOOP_CNT			1000000		/* TODO: real chip exp */

/* The API a cptra driver should implement */
__subsystem struct cptra_driver_api {
	int (*caliptra_fw_upload)(const struct device *dev, uint8_t *buf, int size);
};

static inline int caliptra_fw_upload(const struct device *dev, uint8_t *buf, int size)
{
	struct cptra_driver_api *api;
	int tmp;

	api = (struct cptra_driver_api *)dev->api;
	tmp = api->caliptra_fw_upload(dev, buf, size);

	return tmp;
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_CPTRA_H_ */
