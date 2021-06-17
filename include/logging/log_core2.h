/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_LOGGING_LOG_CORE2_H_
#define ZEPHYR_INCLUDE_LOGGING_LOG_CORE2_H_

#include <logging/log_msg2.h>

#ifdef __cplusplus
extern "C" {
#endif

#define Z_TRACING_LOG_TRACE(id) do { \
	Z_TRACING_LOG_HDR_INIT(_msg, id); \
	z_log_msg2_put_trace(_msg); \
} while (0)

#define Z_TRACING_LOG_TRACE_PTR(id, ptr) do { \
	Z_TRACING_LOG_HDR_INIT(_msg, id); \
	z_log_msg2_put_trace_ptr(_msg, ptr); \
} while (0)

void z_log_msg2_put_trace(struct log_msg2_trace trace);

void z_log_msg2_put_trace_ptr(struct log_msg2_trace hdr, void *data);


/** @brief Initialize module for handling logging message. */
void z_log_msg2_init(void);

/** @brief Allocate log message.
 *
 * @param wlen Length in 32 bit words.
 *
 * @return allocated space or null if cannot be allocated.
 */
struct log_msg2 *z_log_msg2_alloc(uint32_t wlen);

/** @brief Commit log message.
 *
 * @param msg Message.
 */
void z_log_msg2_commit(struct log_msg2 *msg);

/** @brief Get pending log message.
 *
 * @param[out] len Message length in bytes is written is @p len is not null.
 *
 * @param Message or null if no pending messages.
 */
union log_msg2_generic *z_log_msg2_claim(void);

/** @brief Free message.
 *
 * @param msg Message.
 */
void z_log_msg2_free(union log_msg2_generic *msg);

/** @brief Check if there are any message pending.
 *
 * @retval true if at least one message is pending.
 * @retval false if no message is pending.
 */
bool z_log_msg2_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_LOGGING_LOG_CORE2_H_ */
