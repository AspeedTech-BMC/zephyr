/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stddef.h>
#include <logging/log_backend.h>
#include <logging/log_backend_std.h>
#include <logging/log_core.h>
#include <logging/log_msg.h>
#include <logging/log_output.h>
#include <irq.h>
#include <arch/posix/posix_trace.h>

#define _STDOUT_BUF_SIZE 256
static char stdout_buff[_STDOUT_BUF_SIZE];
static int n_pend; /* Number of pending characters in buffer */

static void preprint_char(int c)
{
	int printnow = 0;

	if (c == '\r') {
		/* Discard carriage returns */
		return;
	}
	if (c != '\n') {
		stdout_buff[n_pend++] = c;
		stdout_buff[n_pend] = 0;
	} else {
		printnow = 1;
	}

	if (n_pend >= _STDOUT_BUF_SIZE - 1) {
		printnow = 1;
	}

	if (printnow) {
		posix_print_trace("%s\n", stdout_buff);
		n_pend = 0;
		stdout_buff[0] = 0;
	}
}

static uint8_t buf[_STDOUT_BUF_SIZE];

static int char_out(uint8_t *data, size_t length, void *ctx)
{
	for (size_t i = 0; i < length; i++) {
		preprint_char(data[i]);
	}

	return length;
}

LOG_OUTPUT_DEFINE(log_output_posix, char_out, buf, sizeof(buf));

static void put(const struct log_backend *const backend,
		struct log_msg *msg)
{
	log_msg_get(msg);

	uint32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_SHOW_COLOR)) {
		if (posix_trace_over_tty(0)) {
			flags |= LOG_OUTPUT_FLAG_COLORS;
		}
	}

	if (IS_ENABLED(CONFIG_LOG_BACKEND_FORMAT_TIMESTAMP)) {
		flags |= LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
	}

	log_output_msg_process(&log_output_posix, msg, flags);

	log_msg_put(msg);

}

static void panic(struct log_backend const *const backend)
{
	log_output_flush(&log_output_posix);
}

static void dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);

	log_output_dropped_process(&log_output_posix, cnt);
}

static void sync_string(const struct log_backend *const backend,
		     struct log_msg_ids src_level, uint32_t timestamp,
		     const char *fmt, va_list ap)
{
	uint32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP;
	uint32_t key;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_SHOW_COLOR)) {
		flags |= LOG_OUTPUT_FLAG_COLORS;
	}

	if (IS_ENABLED(CONFIG_LOG_BACKEND_FORMAT_TIMESTAMP)) {
		flags |= LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
	}

	key = irq_lock();
	log_output_string(&log_output_posix, src_level,
			  timestamp, fmt, ap, flags);
	irq_unlock(key);
}

static void sync_hexdump(const struct log_backend *const backend,
			 struct log_msg_ids src_level, uint32_t timestamp,
			 const char *metadata, const uint8_t *data, uint32_t length)
{
	uint32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP;
	uint32_t key;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_SHOW_COLOR)) {
		flags |= LOG_OUTPUT_FLAG_COLORS;
	}

	if (IS_ENABLED(CONFIG_LOG_BACKEND_FORMAT_TIMESTAMP)) {
		flags |= LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
	}

	key = irq_lock();
	log_output_hexdump(&log_output_posix, src_level, timestamp,
			metadata, data, length, flags);
	irq_unlock(key);
}

static void process(const struct log_backend *const backend,
		union log_msg2_generic *msg)
{
	uint32_t flags = log_backend_std_get_flags();

	log_output_msg2_process(&log_output_posix, &msg->log, flags);
}

const struct log_backend_api log_backend_native_posix_api = {
	.process = IS_ENABLED(CONFIG_LOG2) ? process : NULL,
	.put = IS_ENABLED(CONFIG_LOG_MODE_DEFERRED) ? put : NULL,
	.put_sync_string = IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE) ?
			sync_string : NULL,
	.put_sync_hexdump = IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE) ?
			sync_hexdump : NULL,
	.panic = panic,
	.dropped = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ? NULL : dropped,
};

LOG_BACKEND_DEFINE(log_backend_native_posix,
		   log_backend_native_posix_api,
		   true);
