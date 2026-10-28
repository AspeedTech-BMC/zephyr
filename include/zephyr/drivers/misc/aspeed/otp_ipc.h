/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_OTP_IPC_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_OTP_IPC_H_

#include <stdint.h>

/*
 * Standalone OTP-over-IPC channel: lets SSP/CA35 read and program OTP
 * through bootmcu, independent of the Caliptra IPC channel/dispatch table.
 *
 * No shared memory is used - request and response both fit in the IPM
 * mailbox's own 8x32-bit word message, with OTP's 16-bit words packed
 * two per 32-bit slot. All ops share one channel; word1 packs the op
 * into its upper 16 bits and a small op-specific value into its lower
 * 16 bits (see OTP_IPC_PACK_OP_COUNT()) so no op loses payload capacity
 * to another.
 *
 * Read request:  word0 = addr, word1 = pack(OTP_IPC_OP_READ, count)
 * Read response: word0 = status, word1..word7 = up to OTP_IPC_READ_MAX_WORDS
 *                16-bit OTP words, packed 2 per word
 *
 * Program request:  word0 = addr, word1 = pack(OTP_IPC_OP_PROGRAM, count),
 *                    word2..word7 = up to OTP_IPC_PROGRAM_MAX_WORDS
 *                    16-bit OTP words to burn, packed 2 per word
 * Program response: word0 = status
 *
 * ECC_GET_STATUS request:  word1 = pack(OTP_IPC_OP_ECC_GET_STATUS, 0)
 * ECC_GET_STATUS response: word0 = status, word1 = ecc_en
 *
 * ECC_SET_ENABLE request:  word1 = pack(OTP_IPC_OP_ECC_SET_ENABLE, enable)
 * ECC_SET_ENABLE response: word0 = status
 *
 * ECC_GET_POLICY request:  word1 = pack(OTP_IPC_OP_ECC_GET_POLICY, region)
 * ECC_GET_POLICY response: word0 = status, word1 = ecc_en, word2 = ecc_supported
 *
 * ECC_SET_POLICY request:  word1 = pack(OTP_IPC_OP_ECC_SET_POLICY, region),
 *                          word2 = ecc_en
 * ECC_SET_POLICY response: word0 = status
 *
 * GET_REVID request:  word1 = pack(OTP_IPC_OP_GET_REVID, 0)
 * GET_REVID response: word0 = status, word1 = SCU0 revid, word2 = SCU1 revid
 *
 * SET_READ_MODE request:  word1 = pack(OTP_IPC_OP_SET_READ_MODE, mode)
 *                          mode: enum otp_read_mode (0=normal, 1=margin)
 * SET_READ_MODE response: word0 = status
 * Affects every subsequent READ until switched back - used for OTP cell
 * margin-screening diagnostics, not part of normal read/program traffic.
 */

#define IPC_CHANNEL_ID_OTP		2

enum otp_ipc_op {
	OTP_IPC_OP_READ = 0,
	OTP_IPC_OP_PROGRAM,
	OTP_IPC_OP_ECC_GET_STATUS,
	OTP_IPC_OP_ECC_SET_ENABLE,
	OTP_IPC_OP_ECC_GET_POLICY,
	OTP_IPC_OP_ECC_SET_POLICY,
	OTP_IPC_OP_GET_REVID,
	OTP_IPC_OP_SET_READ_MODE,
};

#define OTP_IPC_OP_SHIFT		16
#define OTP_IPC_COUNT_MASK		0xFFFF

#define OTP_IPC_PACK_OP_COUNT(op, count) \
	(((uint32_t)(op) << OTP_IPC_OP_SHIFT) | ((uint32_t)(count) & OTP_IPC_COUNT_MASK))
#define OTP_IPC_UNPACK_OP(word)		((word) >> OTP_IPC_OP_SHIFT)
#define OTP_IPC_UNPACK_COUNT(word)	((word) & OTP_IPC_COUNT_MASK)

#define OTP_IPC_READ_MAX_WORDS		14	/* 7 msg words x 2 */
/* 6 msg words x 2; HW already caps a single otp_program_multi() call at 8 */
#define OTP_IPC_PROGRAM_MAX_WORDS	12

int otp_ipc_enable(void);
int otp_ipc_read(uint32_t addr, uint32_t count, uint16_t *out);
int otp_ipc_program(uint32_t addr, uint32_t count, const uint16_t *data);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_OTP_IPC_H_ */
