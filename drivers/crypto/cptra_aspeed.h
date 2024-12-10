/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_DRIVERS_CRYPTO_CPTRA_ASPEED_H_
#define ZEPHYR_DRIVERS_CRYPTO_CPTRA_ASPEED_H_

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

#define CPTRA_MBCMD_ECDSA384_SIGNATURE_VERIFY	0x53494756

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

#endif /* ZEPHYR_DRIVERS_CRYPTO_CPTRA_ASPEED_H_ */
