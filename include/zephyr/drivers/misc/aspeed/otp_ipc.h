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
 * two per 32-bit slot. All ops share one channel; word1 packs a per-call
 * sequence number into its top 8 bits, the op into the next 8 bits, and
 * a small op-specific value into its lower 16 bits (see
 * OTP_IPC_PACK_OP_COUNT(), OTP_IPC_REQ_SEQ()) so no op loses payload
 * capacity to another.
 *
 * The response's word0 echoes that same sequence number back in its top
 * 8 bits alongside the status in its lower 16 bits (see
 * OTP_IPC_PACK_RESP()). A caller whose call already timed out may still
 * see a late reply land afterwards; echoing the sequence number lets it
 * tell that stale reply apart from the answer to whatever it asks next,
 * instead of silently accepting someone else's data or status.
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
 * ECC_GET_STATUS response: word0 = status, word1 = enum otp_ecc_global_status
 * (0=disabled, 1=on because software temporarily enabled it, 2=on because
 * the hardware strap forces it - can't be turned off this session)
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
 *
 * GET_ROM_PATCH_VERSION request:  word1 = pack(OTP_IPC_OP_GET_ROM_PATCH_VERSION, 0)
 * GET_ROM_PATCH_VERSION response: word0 = status, word1 = raw SCU1 ROM patch
 * version register value. Decoding differs between A1 and A2, so the raw
 * value is returned as-is - the caller (CA35 side) decodes it.
 *
 * READ_ECC_CODE request:  word0 = addr, word1 = pack(OTP_IPC_OP_READ_ECC_CODE, 0)
 * READ_ECC_CODE response: word0 = status, word1 = raw 16-bit data, word2 =
 * OTP_DBG01[5:0] (ECC code) captured by the same read. ECC decode is forced
 * off for this read regardless of addr's region's own ECC policy, so data
 * and code reflect exactly what's burned there - used to tell whether a word
 * was already programmed with ECC parity without disturbing any ECC state.
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
	OTP_IPC_OP_GET_ROM_PATCH_VERSION,
	OTP_IPC_OP_READ_ECC_CODE,
};

#define OTP_IPC_SEQ_SHIFT		24
#define OTP_IPC_SEQ_MASK		0xFF
#define OTP_IPC_OP_SHIFT		16
#define OTP_IPC_OP_MASK			0xFF
#define OTP_IPC_COUNT_MASK		0xFFFF

#define OTP_IPC_PACK_OP_COUNT(op, count) \
	((((uint32_t)(op) & OTP_IPC_OP_MASK) << OTP_IPC_OP_SHIFT) | \
	 ((uint32_t)(count) & OTP_IPC_COUNT_MASK))
#define OTP_IPC_UNPACK_OP(word)		(((word) >> OTP_IPC_OP_SHIFT) & OTP_IPC_OP_MASK)
#define OTP_IPC_UNPACK_COUNT(word)	((word) & OTP_IPC_COUNT_MASK)
#define OTP_IPC_REQ_SEQ(word)		(((word) >> OTP_IPC_SEQ_SHIFT) & OTP_IPC_SEQ_MASK)

/* Response word0: seq echoed back in the top 8 bits, status (sign-extended,
 * every status value here fits in int16_t) in the lower 16 bits.
 */
#define OTP_IPC_PACK_RESP(seq, status) \
	((((uint32_t)(seq) & OTP_IPC_SEQ_MASK) << OTP_IPC_SEQ_SHIFT) | \
	 ((uint32_t)(status) & 0xFFFF))
#define OTP_IPC_RESP_SEQ(word)		(((word) >> OTP_IPC_SEQ_SHIFT) & OTP_IPC_SEQ_MASK)
#define OTP_IPC_RESP_STATUS(word)	((int16_t)((word) & 0xFFFF))

#define OTP_IPC_READ_MAX_WORDS		14	/* 7 msg words x 2 */
/* 6 msg words x 2; HW already caps a single otp_program_multi() call at 8 */
#define OTP_IPC_PROGRAM_MAX_WORDS	12

int otp_ipc_enable(void);
int otp_ipc_read(uint32_t addr, uint32_t count, uint16_t *out);
int otp_ipc_program(uint32_t addr, uint32_t count, const uint16_t *data);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_OTP_IPC_H_ */
