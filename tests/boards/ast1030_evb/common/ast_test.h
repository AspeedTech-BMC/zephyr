/*
 * Copyright (c) 2021 Aspeedtech Inc.
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ported near-verbatim from aspeed-dev-v2.6.0's
 * tests/boards/ast1030/src/ast_test.h. Only the include path changed
 * (<ztest.h> -> <zephyr/ztest.h>); ztest_relative_filename()/vprintk()/PRINT
 * still exist unchanged on this Zephyr version. Kept as the shared
 * assertion style for every peripheral in tests/boards/ast1030_evb/ because,
 * unlike zassert_*, it just sets a flag and prints instead of longjmp'ing -
 * safe to call from any thread, which is required by the concurrent runner
 * in tests/boards/ast1030_evb/all/.
 */

#ifndef ASPEED_TESTSUITE_ZTEST_H_
#define ASPEED_TESTSUITE_ZTEST_H_

#include <zephyr/ztest.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define AST_TEST_PASS 0
#define AST_TEST_FAIL -1

static bool ast_test_fail;

const char *ztest_relative_filename(const char *file);

static inline bool aspeed_zassert(bool cond, const char *default_msg, const char *file, int line,
				   const char *func, const char *msg, ...)
{
	if (!cond) {
		va_list vargs;

		va_start(vargs, msg);
		PRINT("\n    Assertion failed at %s:%d: %s: %s\n",
		      ztest_relative_filename(file), line, func, default_msg);
		vprintk(msg, vargs);
		printk("\n");
		va_end(vargs);
		ast_test_fail = true;
		return false;
	}

	return true;
}

static inline int ast_test_result(void)
{
	return ast_test_fail ? AST_TEST_FAIL : AST_TEST_PASS;
}

#define ast_ztest_result() ast_test_result()

#define ast_zassert(cond, default_msg, msg, ...)                                                 \
	aspeed_zassert(cond, msg ? ("(" default_msg ")") : (default_msg), __FILE__, __LINE__,     \
		       __func__, msg ? msg : "", ##__VA_ARGS__)

#define ast_zassert_unreachable(msg, ...)                                                        \
	ast_zassert(0, "Reached unreachable code", msg, ##__VA_ARGS__)

#define ast_zassert_true(cond, msg, ...) ast_zassert(cond, #cond " is false", msg, ##__VA_ARGS__)

#define ast_zassert_false(cond, msg, ...)                                                         \
	ast_zassert(!(cond), #cond " is true", msg, ##__VA_ARGS__)

#define ast_zassert_ok(cond, msg, ...)                                                            \
	ast_zassert(!(cond), #cond " is non-zero", msg, ##__VA_ARGS__)

#define ast_zassert_is_null(ptr, msg, ...)                                                        \
	ast_zassert((ptr) == NULL, #ptr " is not NULL", msg, ##__VA_ARGS__)

#define ast_zassert_not_null(ptr, msg, ...)                                                        \
	ast_zassert((ptr) != NULL, #ptr " is NULL", msg, ##__VA_ARGS__)

#define ast_zassert_equal(a, b, msg, ...)                                                          \
	ast_zassert((a) == (b), #a " not equal to " #b, msg, ##__VA_ARGS__)

#define ast_zassert_not_equal(a, b, msg, ...)                                                      \
	ast_zassert((a) != (b), #a " equal to " #b, msg, ##__VA_ARGS__)

#define ast_zassert_equal_ptr(a, b, msg, ...)                                                      \
	ast_zassert((void *)(a) == (void *)(b), #a " not equal to " #b, msg, ##__VA_ARGS__)

#define ast_zassert_within(a, b, d, msg, ...)                                                      \
	ast_zassert(((a) >= ((b) - (d))) && ((a) <= ((b) + (d))), #a " not within " #b " +/- " #d, \
		    msg, ##__VA_ARGS__)

#define ast_zassert_mem_equal(...) ast_zassert_mem_equal__(__VA_ARGS__)

#define ast_zassert_mem_equal__(buf, exp, size, msg, ...)                                          \
	ast_zassert(memcmp(buf, exp, size) == 0, #buf " not equal to " #exp, msg, ##__VA_ARGS__)

#endif /* ASPEED_TESTSUITE_ZTEST_H_ */
