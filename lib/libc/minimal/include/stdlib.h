/* stdlib.h */

/*
 * Copyright (c) 2011-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_LIBC_MINIMAL_INCLUDE_STDLIB_H_
#define ZEPHYR_LIB_LIBC_MINIMAL_INCLUDE_STDLIB_H_

#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

unsigned long strtoul(const char *nptr, char **endptr, int base);
long strtol(const char *nptr, char **endptr, int base);
int atoi(const char *s);

void *malloc(size_t size);
void free(void *ptr);
void *calloc(size_t nmemb, size_t size);
void *realloc(void *ptr, size_t size);
void *reallocarray(void *ptr, size_t nmemb, size_t size);

void *bsearch(const void *key, const void *array,
	      size_t count, size_t size,
	      int (*cmp)(const void *key, const void *element));

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1
void _exit(int status);
static inline void exit(int status)
{
	_exit(status);
}
void abort(void);

int rand(void);

static inline int abs(int __n)
{
	return (__n < 0) ? -__n : __n;
}

static inline long labs(long __n)
{
	return (__n < 0L) ? -__n : __n;
}

static inline long long llabs(long long __n)
{
	return (__n < 0LL) ? -__n : __n;
}

#ifdef __cplusplus
}
#endif

#endif  /* ZEPHYR_LIB_LIBC_MINIMAL_INCLUDE_STDLIB_H_ */
