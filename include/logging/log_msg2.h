/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_LOGGING_LOG_MSG2_H_
#define ZEPHYR_INCLUDE_LOGGING_LOG_MSG2_H_

#include <logging/log_instance.h>
#include <sys/mpsc_packet.h>
#include <sys/cbprintf.h>
#include <sys/atomic.h>
#include <sys/util.h>
#include <string.h>

#ifdef __GNUC__
#ifndef alloca
#define alloca __builtin_alloca
#endif
#else
#include <alloca.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_MSG2_DEBUG 0
#define LOG_MSG2_DBG(...) IF_ENABLED(LOG_MSG2_DEBUG, (printk(__VA_ARGS__)))

#if CONFIG_LOG_TIMESTAMP_64BIT
typedef uint64_t log_timestamp_t;
#else
typedef uint32_t log_timestamp_t;
#endif

/**
 * @brief Log message API
 * @defgroup log_msg2 Log message v2 API
 * @ingroup logger
 * @{
 */

#define Z_LOG_MSG2_LOG 0
#define Z_LOG_MSG2_TRACE 1

#define LOG_MSG2_GENERIC_HDR \
	MPSC_PBUF_HDR;\
	uint32_t type:1

struct log_msg2_desc {
	LOG_MSG2_GENERIC_HDR;
	uint32_t domain:3;
	uint32_t level:3;
	uint32_t package_len:10;
	uint32_t data_len:12;
	uint32_t reserved:1;
};

struct log_msg2_trace_hdr {
	LOG_MSG2_GENERIC_HDR;
	uint32_t evt_id:5;
#if CONFIG_LOG_TRACE_SHORT_TIMESTAMP
	uint32_t timestamp:24;
#else
	log_timestamp_t timestamp;
#endif
};

union log_msg2_source {
	const struct log_source_const_data *fixed;
	struct log_source_dynamic_data *dynamic;
	void *raw;
};

struct log_msg2_hdr {
	struct log_msg2_desc desc;
/* Attempting to keep best alignment. When address is 64 bit and timestamp 32
 * swap the order to have 16 byte header instead of 24 byte.
 */
#if (INTPTR_MAX > INT32_MAX) && !CONFIG_LOG_TIMESTAMP_64BIT
	log_timestamp_t timestamp;
	const void *source;
#else
	const void *source;
	log_timestamp_t timestamp;
#endif
#if defined(__xtensa__) && !defined(CONFIG_LOG_TIMESTAMP_64BIT)
	/* xtensa requires that cbprintf package that follows the header is
	 * aligned to 16 bytes. Adding padding when necessary.
	 */
	uint32_t padding;
#endif
};

struct log_msg2_trace {
	struct log_msg2_trace_hdr hdr;
};

struct log_msg2_trace_ptr {
	struct log_msg2_trace_hdr hdr;
	void *ptr;
};

struct log_msg2 {
	struct log_msg2_hdr hdr;
	uint8_t data[];
};

struct log_msg2_generic_hdr {
	LOG_MSG2_GENERIC_HDR;
};

union log_msg2_generic {
	union mpsc_pbuf_generic buf;
	struct log_msg2_generic_hdr generic;
	struct log_msg2_trace trace;
	struct log_msg2_trace_ptr trace_ptr;
	struct log_msg2 log;
};

/** @brief Method used for creating a log message.
 *
 * It is used for testing purposes to validate that expected mode was used.
 */
enum z_log_msg2_mode {
	/* Runtime mode is least efficient but supports all cases thus it is
	 * threated as a fallback method when others cannot be used.
	 */
	Z_LOG_MSG2_MODE_RUNTIME,
	/* Mode creates statically a string package on stack and calls a
	 * function for creating a message. It takes code size than
	 * Z_LOG_MSG2_MODE_ZERO_COPY but is a bit slower.
	 */
	Z_LOG_MSG2_MODE_FROM_STACK,

	/* Mode calculates size of the message and allocates it and writes
	 * directly to the message space. It is the fastest method but requires
	 * more code size.
	 */
	Z_LOG_MSG2_MODE_ZERO_COPY,

	/* Mode used when synchronous logging is enabled. */
	Z_LOG_MSG2_MODE_SYNC
};

#define Z_LOG_MSG_DESC_INITIALIZER(_domain_id, _level, _plen, _dlen) \
{ \
	.valid = 0, \
	.busy = 0, \
	.type = Z_LOG_MSG2_LOG, \
	.domain = _domain_id, \
	.level = _level, \
	.package_len = _plen, \
	.data_len = _dlen, \
	.reserved = 0, \
}

/* Messages are aligned to alignment required by cbprintf package. */
#define Z_LOG_MSG2_ALIGNMENT CBPRINTF_PACKAGE_ALIGNMENT

#if CONFIG_LOG2_USE_VLA
#define Z_LOG_MSG2_ON_STACK_ALLOC(ptr, len) \
	long long _ll_buf[ceiling_fraction(len, sizeof(long long))]; \
	long double _ld_buf[ceiling_fraction(len, sizeof(long double))]; \
	ptr = (sizeof(long double) == Z_LOG_MSG2_ALIGNMENT) ? \
			(struct log_msg2 *)_ld_buf : (struct log_msg2 *)_ll_buf; \
	if (IS_ENABLED(CONFIG_LOG_TEST_CLEAR_MESSAGE_SPACE)) { \
		/* During test fill with 0's to simplify message comparison */ \
		memset(ptr, 0, len); \
	}
#else /* Z_LOG_MSG2_USE_VLA */
/* When VLA cannot be used we need to trick compiler a bit and create multiple
 * fixed size arrays and take the smallest one that will fit the message.
 * Compiler will remove unused arrays and stack usage will be kept similar
 * to vla case, rounded to the size of the used buffer.
 */
#define Z_LOG_MSG2_ON_STACK_ALLOC(ptr, len) \
	long long _ll_buf32[32 / sizeof(long long)]; \
	long long _ll_buf48[48 / sizeof(long long)]; \
	long long _ll_buf64[64 / sizeof(long long)]; \
	long long _ll_buf128[128 / sizeof(long long)]; \
	long long _ll_buf256[256 / sizeof(long long)]; \
	long double _ld_buf32[32 / sizeof(long double)]; \
	long double _ld_buf48[48 / sizeof(long double)]; \
	long double _ld_buf64[64 / sizeof(long double)]; \
	long double _ld_buf128[128 / sizeof(long double)]; \
	long double _ld_buf256[256 / sizeof(long double)]; \
	if (sizeof(long double) == Z_LOG_MSG2_ALIGNMENT) { \
		ptr = (len > 128) ? (struct log_msg2 *)_ld_buf256 : \
			((len > 64) ? (struct log_msg2 *)_ld_buf128 : \
			((len > 48) ? (struct log_msg2 *)_ld_buf64 : \
			((len > 32) ? (struct log_msg2 *)_ld_buf48 : \
				      (struct log_msg2 *)_ld_buf32)));\
	} else { \
		ptr = (len > 128) ? (struct log_msg2 *)_ll_buf256 : \
			((len > 64) ? (struct log_msg2 *)_ll_buf128 : \
			((len > 48) ? (struct log_msg2 *)_ll_buf64 : \
			((len > 32) ? (struct log_msg2 *)_ll_buf48 : \
				      (struct log_msg2 *)_ll_buf32)));\
	} \
	if (IS_ENABLED(CONFIG_LOG_TEST_CLEAR_MESSAGE_SPACE)) { \
		/* During test fill with 0's to simplify message comparison */ \
		memset(ptr, 0, len); \
	}
#endif /* Z_LOG_MSG2_USE_VLA */

#define Z_LOG_MSG2_ALIGN_OFFSET \
	sizeof(struct log_msg2_hdr)

#define Z_LOG_MSG2_LEN(pkg_len, data_len) \
	(sizeof(struct log_msg2_hdr) + pkg_len + (data_len))

#define Z_LOG_MSG2_ALIGNED_WLEN(pkg_len, data_len) \
	ceiling_fraction(ROUND_UP(Z_LOG_MSG2_LEN(pkg_len, data_len), \
				  Z_LOG_MSG2_ALIGNMENT), \
			 sizeof(uint32_t))

#define Z_LOG_MSG2_SYNC(_domain_id, _source, _level, _data, _dlen, ...) do { \
	int _plen; \
	CBPRINTF_STATIC_PACKAGE(NULL, 0, _plen, Z_LOG_MSG2_ALIGN_OFFSET, \
				__VA_ARGS__); \
	struct log_msg2 *_msg; \
	Z_LOG_MSG2_ON_STACK_ALLOC(_msg, Z_LOG_MSG2_LEN(_plen, _dlen)); \
	if (_plen) {\
		CBPRINTF_STATIC_PACKAGE(_msg->data, _plen, _plen, \
					Z_LOG_MSG2_ALIGN_OFFSET, __VA_ARGS__); \
	} \
	struct log_msg2_desc _desc = \
		     Z_LOG_MSG_DESC_INITIALIZER(_domain_id, _level, \
			   (uint32_t)_plen, _dlen); \
	z_log_msg2_finalize(_msg, _source, _desc, _data); \
} while (0)

#define Z_LOG_MSG2_STACK_CREATE(_domain_id, _source, _level, _data, _dlen, ...)\
do { \
	int _plen; \
	if (GET_ARG_N(1, __VA_ARGS__) == NULL) { \
		_plen = 0; \
	} else { \
		CBPRINTF_STATIC_PACKAGE(NULL, 0, _plen, \
					Z_LOG_MSG2_ALIGN_OFFSET, __VA_ARGS__); \
	} \
	struct log_msg2 *_msg; \
	Z_LOG_MSG2_ON_STACK_ALLOC(_msg, Z_LOG_MSG2_LEN(_plen, 0)); \
	if (_plen) { \
		CBPRINTF_STATIC_PACKAGE(_msg->data, _plen, \
					_plen, Z_LOG_MSG2_ALIGN_OFFSET, \
					__VA_ARGS__);\
	} \
	struct log_msg2_desc _desc = \
		Z_LOG_MSG_DESC_INITIALIZER(_domain_id, _level, \
					   (uint32_t)_plen, _dlen); \
	LOG_MSG2_DBG("creating message on stack: package len: %d, data len: %d\n", \
			_plen, (int)(_dlen)); \
	z_log_msg2_static_create((void *)_source, _desc, _msg->data, _data); \
} while (0)

#if CONFIG_LOG_SPEED
#define Z_LOG_MSG2_SIMPLE_CREATE(_domain_id, _source, _level, ...) do { \
	int _plen; \
	CBPRINTF_STATIC_PACKAGE(NULL, 0, _plen, Z_LOG_MSG2_ALIGN_OFFSET, \
				__VA_ARGS__); \
	size_t _msg_wlen = Z_LOG_MSG2_ALIGNED_WLEN(_plen, 0); \
	struct log_msg2 *_msg = z_log_msg2_alloc(_msg_wlen); \
	struct log_msg2_desc _desc = \
		Z_LOG_MSG_DESC_INITIALIZER(_domain_id, _level, (uint32_t)_plen, 0); \
	LOG_MSG2_DBG("creating message zero copy: package len: %d, msg: %p\n", \
			_plen, _msg); \
	if (_msg) { \
		CBPRINTF_STATIC_PACKAGE(_msg->data, _plen, _plen, \
					Z_LOG_MSG2_ALIGN_OFFSET, __VA_ARGS__); \
	} \
	z_log_msg2_finalize(_msg, (void *)_source, _desc, NULL); \
} while (0)
#else
/* Alternative empty macro created to speed up compilation when LOG_SPEED is
 * disabled (default).
 */
#define Z_LOG_MSG2_SIMPLE_CREATE(...)
#endif

/* Macro handles case when local variable with log message string is created.It
 * replaces origing string literal with that variable.
 */
#define Z_LOG_FMT_ARGS_2(_name, ...) \
	COND_CODE_1(CONFIG_LOG2_FMT_SECTION, \
		(COND_CODE_0(NUM_VA_ARGS_LESS_1(__VA_ARGS__), \
		   (_name), (_name, GET_ARGS_LESS_N(1, __VA_ARGS__)))), \
		(__VA_ARGS__))

/** @brief Wrapper for log message string with arguments.
 *
 * Wrapper is replacing first argument with a variable from a dedicated memory
 * section if option is enabled. Macro handles the case when there is no
 * log message provided.
 *
 * @param _name Name of the variable with log message string. It is optionally used.
 * @param ... Optional log message with arguments (may be empty).
 */
#define Z_LOG_FMT_ARGS(_name, ...) \
	COND_CODE_0(NUM_VA_ARGS_LESS_1(_, ##__VA_ARGS__), \
		(NULL), \
		(Z_LOG_FMT_ARGS_2(_name, ##__VA_ARGS__)))

/* Macro handles case when there is no string provided, in that case variable
 * is not created.
 */
#define Z_LOG_MSG2_STR_VAR_IN_SECTION(_name, ...) \
	COND_CODE_0(NUM_VA_ARGS_LESS_1(_, ##__VA_ARGS__), \
		    (/* No args provided, no variable */), \
		    (static const char _name[] \
			__attribute__((__section__(".log_strings"))) = \
			GET_ARG_N(1, __VA_ARGS__);))

/** @brief Create variable in the dedicated memory section (if enabled).
 *
 * Variable is initialized with a format string from the log message.
 *
 * @param _name Variable name.
 * @param ... Optional log message with arguments (may be empty).
 */
#define Z_LOG_MSG2_STR_VAR(_name, ...) \
	IF_ENABLED(CONFIG_LOG2_FMT_SECTION, \
		   (Z_LOG_MSG2_STR_VAR_IN_SECTION(_name, ##__VA_ARGS__)))

/** @brief Create log message and write it into the logger buffer.
 *
 * Macro handles creation of log message which includes storing log message
 * description, timestamp, arguments, copying string arguments into message and
 * copying user data into the message space. The are 3 modes of message
 * creation:
 * - at compile time message size is determined, message is allocated and
 *   content is written directly to the message. It is the fastest but cannot
 *   be used in user mode. Message size cannot be determined at compile time if
 *   it contains data or string arguments which are string pointers.
 * - at compile time message size is determined, string package is created on
 *   stack, message is created in function call. String package can only be
 *   created on stack if it does not contain unexpected pointers to strings.
 * - string package is created at runtime. This mode has no limitations but
 *   it is significantly slower.
 *
 * @param _try_0cpy If positive then, if possible, message content is written
 * directly to message. If 0 then, if possible, string package is created on
 * the stack and message is created in the function call.
 *
 * @param _mode Used for testing. It is set according to message creation mode
 *		used.
 *
 * @param _cstr_cnt Number of constant strings present in the string. It is
 * used to help detect messages which must be runtime processed, compared to
 * message which can be prebuilt at compile time.
 *
 * @param _domain_id Domain ID.
 *
 * @param _source Pointer to the constant descriptor of the log message source.
 *
 * @param _level Log message level.
 *
 * @param _data Pointer to the data. Can be null.
 *
 * @param _dlen Number of data bytes. 0 if data is not provided.
 *
 * @param ...  Optional string with arguments (fmt, ...). It may be empty.
 */
#if CONFIG_LOG2_ALWAYS_RUNTIME
#define Z_LOG_MSG2_CREATE2(_try_0cpy, _mode,  _cstr_cnt, _domain_id, _source,\
			  _level, _data, _dlen, ...) \
do {\
	Z_LOG_MSG2_STR_VAR(_fmt, ##__VA_ARGS__) \
	z_log_msg2_runtime_create(_domain_id, (void *)_source, \
				  _level, (uint8_t *)_data, _dlen,\
				  Z_LOG_FMT_ARGS(_fmt, ##__VA_ARGS__));\
	_mode = Z_LOG_MSG2_MODE_RUNTIME; \
} while (0)
#elif CONFIG_LOG2_MODE_IMMEDIATE /* CONFIG_LOG2_ALWAYS_RUNTIME */
#define Z_LOG_MSG2_CREATE2(_try_0cpy, _mode,  _cstr_cnt, _domain_id, _source,\
			  _level, _data, _dlen, ...) \
do { \
	Z_LOG_MSG2_STR_VAR(_fmt, ##__VA_ARGS__); \
	if (CBPRINTF_MUST_RUNTIME_PACKAGE(_cstr_cnt, __VA_ARGS__)) { \
		LOG_MSG2_DBG("create runtime message\n");\
		z_log_msg2_runtime_create(_domain_id, (void *)_source, \
					  _level, (uint8_t *)_data, _dlen,\
					  Z_LOG_FMT_ARGS(_fmt, ##__VA_ARGS__));\
		_mode = Z_LOG_MSG2_MODE_RUNTIME; \
	} else {\
		Z_LOG_MSG2_SYNC(_domain_id, _source, _level, \
				_data, _dlen, Z_LOG_FMT_ARGS(_fmt, ##__VA_ARGS__)); \
		_mode = Z_LOG_MSG2_MODE_SYNC; \
	} \
} while (0)
#else /* CONFIG_LOG2_ALWAYS_RUNTIME */
#define Z_LOG_MSG2_CREATE2(_try_0cpy, _mode,  _cstr_cnt, _domain_id, _source,\
			  _level, _data, _dlen, ...) \
do { \
	Z_LOG_MSG2_STR_VAR(_fmt, ##__VA_ARGS__); \
	if (CBPRINTF_MUST_RUNTIME_PACKAGE(_cstr_cnt, __VA_ARGS__)) { \
		LOG_MSG2_DBG("create runtime message\n");\
		z_log_msg2_runtime_create(_domain_id, (void *)_source, \
					  _level, (uint8_t *)_data, _dlen,\
					  Z_LOG_FMT_ARGS(_fmt, ##__VA_ARGS__));\
		_mode = Z_LOG_MSG2_MODE_RUNTIME; \
	} else if (IS_ENABLED(CONFIG_LOG_SPEED) && _try_0cpy && ((_dlen) == 0)) {\
		LOG_MSG2_DBG("create zero-copy message\n");\
		Z_LOG_MSG2_SIMPLE_CREATE(_domain_id, _source, \
					_level, Z_LOG_FMT_ARGS(_fmt, ##__VA_ARGS__)); \
		_mode = Z_LOG_MSG2_MODE_ZERO_COPY; \
	} else { \
		LOG_MSG2_DBG("create on stack message\n");\
		Z_LOG_MSG2_STACK_CREATE(_domain_id, _source, _level, _data, \
					_dlen, Z_LOG_FMT_ARGS(_fmt, ##__VA_ARGS__)); \
		_mode = Z_LOG_MSG2_MODE_FROM_STACK; \
	} \
	(void)_mode; \
} while (0)
#endif /* CONFIG_LOG2_ALWAYS_RUNTIME */

#define Z_LOG_MSG2_CREATE(_try_0cpy, _mode,  _domain_id, _source,\
			  _level, _data, _dlen, ...) \
	Z_LOG_MSG2_CREATE2(_try_0cpy, _mode, UTIL_CAT(Z_LOG_FUNC_PREFIX_, _level), \
			   _domain_id, _source, _level, _data, _dlen, \
			   Z_LOG_STR(_level, __VA_ARGS__))

#define Z_TRACING_LOG_HDR_INIT(name, id) \
	struct log_msg2_trace name = { \
		.hdr = { \
			.type = Z_LOG_MSG2_TRACE, \
			.valid = 1, \
			.busy = 0, \
			.evt_id = id, \
		} \
	}

/** @brief Finalize message.
 *
 * Finalization includes setting source, copying data and timestamp in the
 * message followed by committing the message.
 *
 * @param msg Message.
 *
 * @param source Address of the source descriptor.
 *
 * @param desc Message descriptor.
 *
 * @param data Data.
 */
void z_log_msg2_finalize(struct log_msg2 *msg, const void *source,
			 const struct log_msg2_desc desc, const void *data);

/** @brief Create simple message from message details and string package.
 *
 * @param source Source.
 *
 * @param desc Message descriptor.
 *
 * @param package Package.
 *
 * @oaram data Data.
 */
__syscall void z_log_msg2_static_create(const void *source,
					const struct log_msg2_desc desc,
					uint8_t *package, const void *data);

/** @brief Create message at runtime.
 *
 * Function allows to build any log message based on input data. Processing
 * time is significantly higher than statically message creating.
 *
 * @param domain_id Domain ID.
 *
 * @param source Source.
 *
 * @param level Log level.
 *
 * @param data Data.
 *
 * @param dlen Data length.
 *
 * @param fmt String.
 *
 * @param ap Variable list of string arguments.
 */
__syscall void z_log_msg2_runtime_vcreate(uint8_t domain_id, const void *source,
					  uint8_t level, const void *data,
					  size_t dlen, const char *fmt,
					  va_list ap);

/** @brief Create message at runtime.
 *
 * Function allows to build any log message based on input data. Processing
 * time is significantly higher than statically message creating.
 *
 * @param domain_id Domain ID.
 *
 * @param source Source.
 *
 * @param level Log level.
 *
 * @param data Data.
 *
 * @param dlen Data length.
 *
 * @param fmt String.
 *
 * @param ... String arguments.
 */
static inline void z_log_msg2_runtime_create(uint8_t domain_id,
					     const void *source,
					     uint8_t level, const void *data,
					     size_t dlen, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	z_log_msg2_runtime_vcreate(domain_id, source, level,
				   data, dlen, fmt, ap);
	va_end(ap);
}

static inline bool z_log_item_is_msg(union log_msg2_generic *msg)
{
	return msg->generic.type == Z_LOG_MSG2_LOG;
}

/** @brief Get total length (in 32 bit words) of a log message.
 *
 * @param desc Log message descriptor.
 *
 * @return Length.
 */
static inline uint32_t log_msg2_get_total_wlen(const struct log_msg2_desc desc)
{
	return Z_LOG_MSG2_ALIGNED_WLEN(desc.package_len, desc.data_len);
}

/** @brief Get length of the log item.
 *
 * @param item Item.
 *
 * @return Length in 32 bit words.
 */
static inline uint32_t log_msg2_generic_get_wlen(union mpsc_pbuf_generic *item)
{
	union log_msg2_generic *generic_msg = (union log_msg2_generic *)item;

	if (z_log_item_is_msg(generic_msg)) {
		struct log_msg2 *msg = (struct log_msg2 *)generic_msg;

		return log_msg2_get_total_wlen(msg->hdr.desc);
	}

	/* trace TODO */
	return 0;
}

/** @brief Get log message domain ID.
 *
 * @param msg Log message.
 *
 * @return Domain ID
 */
static inline uint8_t log_msg2_get_domain(struct log_msg2 *msg)
{
	return msg->hdr.desc.domain;
}

/** @brief Get log message level.
 *
 * @param msg Log message.
 *
 * @return Log level.
 */
static inline uint8_t log_msg2_get_level(struct log_msg2 *msg)
{
	return msg->hdr.desc.level;
}

/** @brief Get message source data.
 *
 * @param msg Log message.
 *
 * @return Pointer to the source data.
 */
static inline const void *log_msg2_get_source(struct log_msg2 *msg)
{
	return msg->hdr.source;
}

/** @brief Get timestamp.
 *
 * @param msg Log message.
 *
 * @return Timestamp.
 */
static inline log_timestamp_t log_msg2_get_timestamp(struct log_msg2 *msg)
{
	return msg->hdr.timestamp;
}

/** @brief Get data buffer.
 *
 * @param msg log message.
 *
 * @param len location where data length is written.
 *
 * @return pointer to the data buffer.
 */
static inline uint8_t *log_msg2_get_data(struct log_msg2 *msg, size_t *len)
{
	*len = msg->hdr.desc.data_len;

	return msg->data + msg->hdr.desc.package_len;
}

/** @brief Get string package.
 *
 * @param msg log message.
 *
 * @param len location where string package length is written.
 *
 * @return pointer to the package.
 */
static inline uint8_t *log_msg2_get_package(struct log_msg2 *msg, size_t *len)
{
	*len = msg->hdr.desc.package_len;

	return msg->data;
}

/**
 * @}
 */

#include <syscalls/log_msg2.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_LOGGING_LOG_MSG2_H_ */
