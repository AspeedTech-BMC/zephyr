/*
 * Copyright (c) 2021 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Test log message
 */

#include <sys/mpsc_pbuf.h>

#include <tc_util.h>
#include <stdbool.h>
#include <zephyr.h>
#include <ztest.h>
#include <random/rand32.h>

#define PUT_EXT_LEN \
	((sizeof(union mpsc_pbuf_generic) + sizeof(void *)) / sizeof(uint32_t))

#define LEN_BITS 9

struct test_data {
	MPSC_PBUF_HDR;
	uint32_t len : LEN_BITS;
	uint32_t data : 32 - MPSC_PBUF_HDR_BITS - LEN_BITS;
};

struct test_data_ext {
	struct test_data hdr;
	void *data;
} __packed;

struct test_data_var {
	struct test_data hdr;
	uint32_t data[];
};

union test_item {
	struct test_data data;
	struct test_data_ext data_ext;
	union mpsc_pbuf_generic item;
};

static uint32_t get_wlen(union mpsc_pbuf_generic *item)
{
	union test_item *t_item = (union test_item *)item;

	return t_item->data.len;
}

static uint32_t drop_cnt;
static uintptr_t exp_dropped_data[10];
static uint32_t exp_dropped_len[10];

static void drop(struct mpsc_pbuf_buffer *buffer, union mpsc_pbuf_generic *item)
{
	struct test_data_var *packet = (struct test_data_var *)item;

	zassert_equal(packet->hdr.data, exp_dropped_data[drop_cnt], NULL);
	zassert_equal(packet->hdr.len, exp_dropped_len[drop_cnt], NULL);
	for (int i = 0; i < exp_dropped_len[drop_cnt] - 1; i++) {
		int err = memcmp(packet->data, &exp_dropped_data[drop_cnt],
				 sizeof(uint32_t));

		zassert_equal(err, 0, NULL);
	}

	drop_cnt++;
}

static uint32_t buf32[512];

static struct mpsc_pbuf_buffer_config cfg = {
	.buf = buf32,
	.size = ARRAY_SIZE(buf32),
	.notify_drop = drop,
	.get_wlen = get_wlen
};

static void init(struct mpsc_pbuf_buffer *buffer, bool overwrite, bool pow2)
{
	drop_cnt = 0;
	cfg.flags = overwrite ? MPSC_PBUF_MODE_OVERWRITE : 0;
	cfg.size = ARRAY_SIZE(buf32) - (pow2 ? 0 : 1);
	mpsc_pbuf_init(buffer, &cfg);

#if CONFIG_SOC_SERIES_NRF52X
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	DWT->CYCCNT = 0;
#endif
}

static inline uint32_t get_cyc(void)
{
#if CONFIG_SOC_SERIES_NRF52X
	return DWT->CYCCNT;
#else
	return k_cycle_get_32();
#endif
}

void item_put_no_overwrite(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, false, pow2);

	int repeat = buffer.size*2;
	union test_item test_1word = {.data = {.valid = 1, .len = 1 }};

	for (int i = 0; i < repeat; i++) {
		union test_item *t;

		test_1word.data.data = i;
		mpsc_pbuf_put_word(&buffer, test_1word.item);

		t = (union test_item *)mpsc_pbuf_claim(&buffer);
		zassert_true(t, NULL);
		zassert_equal(t->data.data, i, NULL);
		mpsc_pbuf_free(&buffer, &t->item);

	}

	zassert_equal(mpsc_pbuf_claim(&buffer), NULL, NULL);
}

void test_item_put_no_overwrite(void)
{
	item_put_no_overwrite(true);
	item_put_no_overwrite(false);
}

void item_put_overwrite(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, true, pow2);

	union test_item test_1word = {.data = {.valid = 1, .len = 1 }};

	exp_dropped_data[0] = 0;
	exp_dropped_len[0] = 1;

	for (int i = 0; i < buffer.size; i++) {
		test_1word.data.data = i;
		mpsc_pbuf_put_word(&buffer, test_1word.item);
	}

	zassert_equal(drop_cnt, 1,
			"Unexpected number of dropped messages: %d", drop_cnt);
}

void test_item_put_overwrite(void)
{
	item_put_overwrite(true);
	item_put_overwrite(false);
}

void item_put_saturate(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, false, pow2);

	int repeat = buffer.size;
	union test_item test_1word = {.data = {.valid = 1, .len = 1 }};
	union test_item *t;

	zassert_false(mpsc_pbuf_is_pending(&buffer), NULL);

	for (int i = 0; i < repeat/2; i++) {
		test_1word.data.data = i;
		mpsc_pbuf_put_word(&buffer, test_1word.item);

		zassert_true(mpsc_pbuf_is_pending(&buffer), NULL);

		t = (union test_item *)mpsc_pbuf_claim(&buffer);
		zassert_true(t, NULL);
		zassert_equal(t->data.data, i, NULL);
		mpsc_pbuf_free(&buffer, &t->item);
	}

	for (int i = 0; i < repeat; i++) {
		test_1word.data.data = i;
		mpsc_pbuf_put_word(&buffer, test_1word.item);
	}

	for (int i = 0; i < (repeat-1); i++) {
		t = (union test_item *)mpsc_pbuf_claim(&buffer);
		zassert_true(t, NULL);
		zassert_equal(t->data.data, i, NULL);
		mpsc_pbuf_free(&buffer, &t->item);
	}

	zassert_equal(mpsc_pbuf_claim(&buffer), NULL, NULL);
}

void test_item_put_saturate(void)
{
	item_put_saturate(true);
	item_put_saturate(false);
}

void benchmark_item_put(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, true, pow2);

	int repeat = buffer.size - 1;
	union test_item test_1word = {.data = {.valid = 1, .len = 1 }};
	uint32_t t = get_cyc();

	for (int i = 0; i < repeat; i++) {
		test_1word.data.data = i;
		mpsc_pbuf_put_word(&buffer, test_1word.item);
	}

	t = get_cyc() - t;
	PRINT("%s buffer\n", pow2 ? "pow2" : "non-pow2");
	PRINT("single word put time: %d cycles\n", t/repeat);

	t = get_cyc();
	for (int i = 0; i < repeat; i++) {
		union test_item *t;

		t = (union test_item *)mpsc_pbuf_claim(&buffer);
		zassert_true(t, NULL);
		zassert_equal(t->data.data, i, NULL);
		mpsc_pbuf_free(&buffer, &t->item);
	}

	t = get_cyc() - t;
	PRINT("single word item claim,free: %d cycles\n", t/repeat);

	zassert_equal(mpsc_pbuf_claim(&buffer), NULL, NULL);
}

void test_benchmark_item_put(void)
{
	benchmark_item_put(true);
	benchmark_item_put(false);
}

void item_put_ext_no_overwrite(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, false, pow2);

	int repeat = buffer.size * 2;
	union test_item test_ext_item = {
		.data = {
			.valid = 1,
			.len = PUT_EXT_LEN
		}
	};
	void *data;

	for (uintptr_t i = 0; i < repeat; i++) {
		union test_item *t;

		data = (void *)i;
		test_ext_item.data.data = i;
		mpsc_pbuf_put_word_ext(&buffer, test_ext_item.item, data);

		t = (union test_item *)mpsc_pbuf_claim(&buffer);
		zassert_true(t, NULL);
		zassert_equal(t->data_ext.hdr.data, i, NULL);
		zassert_equal(t->data_ext.data, (void *)i, NULL);
		mpsc_pbuf_free(&buffer, &t->item);
	}

	zassert_equal(mpsc_pbuf_claim(&buffer), NULL, NULL);
}

void test_item_put_ext_no_overwrite(void)
{
	item_put_ext_no_overwrite(true);
	item_put_ext_no_overwrite(false);
}

void item_put_word_ext_overwrite(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, true, pow2);

	size_t w = (sizeof(uint32_t) + sizeof(void *)) / sizeof(uint32_t);
	int repeat = 1 + (buffer.size - 1) / w;
	union test_item test_ext_item = {
		.data = {
			.valid = 1,
			.len = PUT_EXT_LEN
		}
	};

	exp_dropped_data[0] = 0;
	exp_dropped_len[0] = w;
	exp_dropped_data[1] = 1;
	exp_dropped_len[1] = w;

	for (uintptr_t i = 0; i < repeat; i++) {
		test_ext_item.data.data = i;
		mpsc_pbuf_put_word_ext(&buffer, test_ext_item.item, (void *)i);
	}

	uint32_t exp_drop_cnt = (sizeof(void *) == sizeof(uint32_t)) ?
				(pow2 ? 1 : 2) : 2;

	zassert_equal(drop_cnt, exp_drop_cnt,
			"Unexpected number of dropped messages: %d", drop_cnt);
}

void test_item_put_word_ext_overwrite(void)
{
	item_put_word_ext_overwrite(true);
	item_put_word_ext_overwrite(false);
}

void item_put_ext_saturate(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, false, pow2);

	int repeat = buffer.size / PUT_EXT_LEN;
	union test_item test_ext_item = {
		.data = {
			.valid = 1,
			.len = PUT_EXT_LEN
		}
	};
	void *data;
	union test_item *t;

	for (uintptr_t i = 0; i < repeat/2; i++) {
		test_ext_item.data.data = i;
		data = (void *)i;
		mpsc_pbuf_put_word_ext(&buffer, test_ext_item.item, data);

		t = (union test_item *)mpsc_pbuf_claim(&buffer);
		zassert_true(t, NULL);
		zassert_equal(t->data.data, i, NULL);
		mpsc_pbuf_free(&buffer, &t->item);
	}

	for (uintptr_t i = 0; i < repeat; i++) {
		test_ext_item.data.data = i;
		data = (void *)i;
		mpsc_pbuf_put_word_ext(&buffer, test_ext_item.item, data);
	}

	for (uintptr_t i = 0; i < (repeat-1); i++) {
		t = (union test_item *)mpsc_pbuf_claim(&buffer);
		zassert_true(t, NULL);
		zassert_equal(t->data_ext.data, (void *)i, NULL);
		zassert_equal(t->data_ext.hdr.data, i, NULL);
		mpsc_pbuf_free(&buffer, &t->item);
	}

	zassert_equal(mpsc_pbuf_claim(&buffer), NULL, NULL);
}

void test_item_put_ext_saturate(void)
{
	item_put_ext_saturate(true);
	item_put_ext_saturate(false);
}

void benchmark_item_put_ext(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, false, pow2);

	int repeat = (buffer.size - 1) / PUT_EXT_LEN;
	union test_item test_ext_item = {
		.data = {
			.valid = 1,
			.len = PUT_EXT_LEN
		}
	};
	void *data = NULL;
	uint32_t t = get_cyc();

	for (int i = 0; i < repeat; i++) {
		test_ext_item.data.data = i;
		mpsc_pbuf_put_word_ext(&buffer, test_ext_item.item, data);
	}

	t = get_cyc() - t;
	PRINT("%spow2 buffer\n", pow2 ? "" : "non-");
	PRINT("put_ext time: %d cycles\n", t/repeat);

	t = get_cyc();
	for (int i = 0; i < repeat; i++) {
		union test_item *t;

		t = (union test_item *)mpsc_pbuf_claim(&buffer);
		zassert_true(t, NULL);
		zassert_equal(t->data.data, i, NULL);
		mpsc_pbuf_free(&buffer, &t->item);
	}

	t = get_cyc() - t;
	PRINT("ext item claim,free: %d cycles\n", t/repeat);

	zassert_equal(mpsc_pbuf_claim(&buffer), NULL, NULL);
}

void test_benchmark_item_put_ext(void)
{
	benchmark_item_put_ext(true);
	benchmark_item_put_ext(false);
}

void benchmark_item_put_data(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, false, pow2);

	int repeat = (buffer.size - 1) / PUT_EXT_LEN;
	union test_item test_ext_item = {
		.data_ext = {
			.hdr = {
				.valid = 1,
				.len = PUT_EXT_LEN
			},
			.data = NULL
		}
	};
	uint32_t t = get_cyc();

	for (uintptr_t i = 0; i < repeat; i++) {
		test_ext_item.data_ext.hdr.data = i;
		test_ext_item.data_ext.data = (void *)i;
		mpsc_pbuf_put_data(&buffer, (uint32_t *)&test_ext_item,
				    PUT_EXT_LEN);
	}

	t = get_cyc() - t;
	PRINT("%spow2 buffer\n", pow2 ? "" : "non-");
	PRINT("put_ext time: %d cycles\n", t/repeat);

	t = get_cyc();
	for (int i = 0; i < repeat; i++) {
		union test_item *t;

		t = (union test_item *)mpsc_pbuf_claim(&buffer);
		zassert_true(t, NULL);
		zassert_equal(t->data.data, i, NULL);
		mpsc_pbuf_free(&buffer, &t->item);
	}

	t = get_cyc() - t;
	PRINT("ext item claim,free: %d cycles\n", t/repeat);

	zassert_equal(mpsc_pbuf_claim(&buffer), NULL, NULL);
}

void test_benchmark_item_put_data(void)
{
	benchmark_item_put_data(true);
	benchmark_item_put_data(false);
}

void item_put_data_overwrite(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, true, pow2);

	size_t w = (sizeof(uint32_t) + sizeof(void *)) / sizeof(uint32_t);
	int repeat = 1 + (buffer.size - 1) / w;
	static const int len = sizeof(struct test_data_ext) / sizeof(uint32_t);
	struct test_data_ext item = {
		.hdr = {
			.valid = 1,
			.len = len
		}
	};

	exp_dropped_data[0] = 0;
	exp_dropped_len[0] = w;
	exp_dropped_data[1] = 1;
	exp_dropped_len[1] = w;

	for (uintptr_t i = 0; i < repeat; i++) {
		item.data = (void *)i;
		item.hdr.data = i;
		mpsc_pbuf_put_data(&buffer, (uint32_t *)&item, len);
	}

	uint32_t exp_drop_cnt = (sizeof(void *) == sizeof(uint32_t)) ?
				(pow2 ? 1 : 2) : 2;

	zassert_equal(drop_cnt, exp_drop_cnt,
			"Unexpected number of dropped messages: %d", drop_cnt);
}

void test_put_data_overwrite(void)
{
	item_put_data_overwrite(true);
	item_put_data_overwrite(false);
}

void item_alloc_commit(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, false, pow2);

	struct test_data_var *packet;
	uint32_t len = 5;
	int repeat = 1024;

	for (int i = 0; i < repeat; i++) {
		packet = (struct test_data_var *)mpsc_pbuf_alloc(&buffer, len,
								 K_NO_WAIT);
		packet->hdr.len = len;
		for (int j = 0; j < len - 1; j++) {
			packet->data[j] = i + j;
		}

		mpsc_pbuf_commit(&buffer, (union mpsc_pbuf_generic *)packet);

		packet = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
		zassert_true(packet, NULL);
		zassert_equal(packet->hdr.len, len, NULL);

		for (int j = 0; j < len - 1; j++) {
			zassert_equal(packet->data[j], i + j, NULL);
		}

		mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)packet);
	}
}

void test_item_alloc_commit(void)
{
	item_alloc_commit(true);
	item_alloc_commit(false);
}

static uint32_t saturate_buffer_uneven(struct mpsc_pbuf_buffer *buffer,
					uint32_t len)
{
	struct test_data_var *packet;
	uint32_t uneven = 5;
	uint32_t cnt = 0;
	int repeat =
		uneven - 1 + ((buffer->size - (uneven * len)) / len);

	/* Put some data to include wrapping */
	for (int i = 0; i < uneven; i++) {
		packet = (struct test_data_var *)mpsc_pbuf_alloc(buffer, len,
								 K_NO_WAIT);
		packet->hdr.len = len;
		mpsc_pbuf_commit(buffer, (union mpsc_pbuf_generic *)packet);

		packet = (struct test_data_var *)mpsc_pbuf_claim(buffer);
		zassert_true(packet, NULL);
		mpsc_pbuf_free(buffer, (union mpsc_pbuf_generic *)packet);
	}

	for (int i = 0; i < repeat; i++) {
		packet = (struct test_data_var *)mpsc_pbuf_alloc(buffer, len,
								 K_NO_WAIT);
		zassert_true(packet, NULL);
		packet->hdr.len = len;
		packet->hdr.data = i;
		for (int j = 0; j < len - 1; j++) {
			packet->data[j] = i + j;
		}

		mpsc_pbuf_commit(buffer, (union mpsc_pbuf_generic *)packet);
		cnt++;
	}

	return cnt;
}

void item_alloc_commit_saturate(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, false, pow2);

	saturate_buffer_uneven(&buffer, 5);

	struct test_data_var *packet;
	uint32_t len = 5;

	packet = (struct test_data_var *)mpsc_pbuf_alloc(&buffer, len,
							 K_NO_WAIT);
	zassert_equal(packet, NULL, NULL);

	/* Get one packet from the buffer. */
	packet = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_true(packet, NULL);
	mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)packet);

	/* and try to allocate one more time, this time with success. */
	packet = (struct test_data_var *)mpsc_pbuf_alloc(&buffer, len,
							 K_NO_WAIT);
	zassert_true(packet, NULL);
}

void test_item_alloc_commit_saturate(void)
{
	item_alloc_commit_saturate(true);
	item_alloc_commit_saturate(false);
}

void item_alloc_preemption(bool pow2)
{
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, false, pow2);

	struct test_data_var *p0;
	struct test_data_var *p1;
	struct test_data_var *p;

	p0 = (struct test_data_var *)mpsc_pbuf_alloc(&buffer, 10, K_NO_WAIT);
	zassert_true(p0, NULL);
	p0->hdr.len = 10;

	/* Check that no packet is yet available */
	p = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_equal(p, NULL, NULL);

	p1 = (struct test_data_var *)mpsc_pbuf_alloc(&buffer, 20, K_NO_WAIT);
	zassert_true(p1, NULL);
	p1->hdr.len = 20;

	/* Commit p1, p0 is still not committed, there should be no packets
	 * available for reading.
	 */
	mpsc_pbuf_commit(&buffer, (union mpsc_pbuf_generic *)p1);

	/* Check that no packet is yet available */
	p = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_equal(p, NULL, NULL);

	mpsc_pbuf_commit(&buffer, (union mpsc_pbuf_generic *)p0);

	/* Validate that p0 is the first one. */
	p = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_true(p, NULL);
	zassert_equal(p->hdr.len, 10, NULL);
	mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)p);

	/* Validate that p1 is the next one. */
	p = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_true(p, NULL);
	zassert_equal(p->hdr.len, 20, NULL);
	mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)p);

	/* No more packets. */
	p = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_equal(p, NULL, NULL);
}

void test_item_alloc_preemption(void)
{
	item_alloc_preemption(true);
	item_alloc_preemption(false);
}

void overwrite(bool pow2)
{
	struct test_data_var *p;
	uint32_t fill_len = 5;
	uint32_t len0, len1;
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, true, pow2);
	uint32_t packet_cnt = saturate_buffer_uneven(&buffer, fill_len);

	exp_dropped_data[0] = 0;
	exp_dropped_len[0] = fill_len;
	len0 = 6;
	p = (struct test_data_var *)mpsc_pbuf_alloc(&buffer, len0, K_NO_WAIT);

	p->hdr.len = len0;
	mpsc_pbuf_commit(&buffer, (union mpsc_pbuf_generic *)p);
	zassert_equal(drop_cnt, 1, NULL);

	/* Request allocation which will require dropping 2 packets. */
	len1 = 9;
	exp_dropped_data[1] = 1;
	exp_dropped_len[1] = fill_len;
	exp_dropped_data[2] = 2;
	exp_dropped_len[2] = fill_len;

	p = (struct test_data_var *)mpsc_pbuf_alloc(&buffer, len1, K_NO_WAIT);

	p->hdr.len = len1;
	mpsc_pbuf_commit(&buffer, (union mpsc_pbuf_generic *)p);
	zassert_equal(drop_cnt, 3, NULL);

	for (int i = 0; i < (packet_cnt - drop_cnt); i++) {
		p = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
		zassert_true(p, NULL);
		zassert_equal(p->hdr.len, fill_len, NULL);
		zassert_equal(p->hdr.data, i + drop_cnt, NULL);
		for (int j = 0; j < fill_len - 1; j++) {
			zassert_equal(p->data[j], p->hdr.data + j, NULL);
		}

		mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)p);
	}

	p = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_true(p, NULL);
	zassert_equal(p->hdr.len, len0, NULL);
	mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)p);

	p = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_true(p, NULL);
	zassert_equal(p->hdr.len, len1, NULL);
	mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)p);

	p = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_equal(p, NULL, NULL);
}

void test_overwrite(void)
{
	overwrite(true);
	overwrite(false);
}

void overwrite_while_claimed(bool pow2)
{
	struct test_data_var *p0;
	struct test_data_var *p1;
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, true, pow2);

	uint32_t fill_len = 5;
	uint32_t len = 6;
	uint32_t packet_cnt = saturate_buffer_uneven(&buffer, fill_len);

	/* Start by claiming a packet. Buffer is now full. Allocation shall
	 * skip claimed packed and drop the next one.
	 */
	p0 = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_true(p0, NULL);
	zassert_equal(p0->hdr.len, fill_len, NULL);

	exp_dropped_data[0] = p0->hdr.data + 1; /* next packet is dropped */
	exp_dropped_len[0] = fill_len;
	exp_dropped_data[1] = p0->hdr.data + 2; /* next packet is dropped */
	exp_dropped_len[1] = fill_len;
	p1 = (struct test_data_var *)mpsc_pbuf_alloc(&buffer, 6, K_NO_WAIT);

	zassert_equal(drop_cnt, 2, NULL);
	p1->hdr.len = len;
	mpsc_pbuf_commit(&buffer, (union mpsc_pbuf_generic *)p1);

	mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)p0);

	for (int i = 0; i < packet_cnt - drop_cnt - 1; i++) {
		p0 = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
		zassert_true(p0, NULL);
		zassert_equal(p0->hdr.len, fill_len, NULL);
		zassert_equal(p0->hdr.data, i + drop_cnt + 1, NULL);
		mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)p0);
	}

	p0 = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_true(p0, NULL);
	zassert_equal(p0->hdr.len, len, NULL);

	p0 = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_equal(p0, NULL, NULL);
}

void test_overwrite_while_claimed(void)
{
	overwrite_while_claimed(true);
	overwrite_while_claimed(false);
}

void overwrite_while_claimed2(bool pow2)
{
	struct test_data_var *p0;
	struct test_data_var *p1;
	struct mpsc_pbuf_buffer buffer;

	init(&buffer, true, pow2);

	uint32_t fill_len = 1;
	uint32_t len = 3;
	uint32_t packet_cnt = saturate_buffer_uneven(&buffer, fill_len);

	/* Start by claiming a packet. Buffer is now full. Allocation shall
	 * skip claimed packed and drop the next one.
	 */
	p0 = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_true(p0, NULL);
	zassert_equal(p0->hdr.len, fill_len, NULL);

	exp_dropped_data[0] = p0->hdr.data + 1; /* next packet is dropped */
	exp_dropped_len[0] = fill_len;
	exp_dropped_data[1] = p0->hdr.data + 2; /* next packet is dropped */
	exp_dropped_len[1] = fill_len;
	exp_dropped_data[2] = p0->hdr.data + 3; /* next packet is dropped */
	exp_dropped_len[2] = fill_len;
	exp_dropped_data[3] = p0->hdr.data + 4; /* next packet is dropped */
	exp_dropped_len[3] = fill_len;
	p1 = (struct test_data_var *)mpsc_pbuf_alloc(&buffer, len, K_NO_WAIT);

	zassert_equal(drop_cnt, 4, NULL);
	p1->hdr.len = len;
	mpsc_pbuf_commit(&buffer, (union mpsc_pbuf_generic *)p1);

	mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)p0);

	for (int i = 0; i < packet_cnt - drop_cnt - 1; i++) {
		p0 = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
		zassert_true(p0, NULL);
		zassert_equal(p0->hdr.len, fill_len, NULL);
		zassert_equal(p0->hdr.data, i + drop_cnt + 1, NULL);
		mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)p0);
	}

	p0 = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_true(p0, NULL);
	zassert_equal(p0->hdr.len, len, NULL);

	p0 = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
	zassert_equal(p0, NULL, NULL);
}

void test_overwrite_while_claimed2(void)
{
		overwrite_while_claimed2(true);
		overwrite_while_claimed2(false);
}

static uintptr_t current_rd_idx;

static void validate_packet(struct test_data_var *packet)
{
	zassert_equal((uintptr_t)packet->hdr.data, current_rd_idx,
			"Got %d, expected: %d",
			(uintptr_t)packet->hdr.data, current_rd_idx);
	current_rd_idx++;
}

static void consistent_drop(struct mpsc_pbuf_buffer *buffer,
			    union mpsc_pbuf_generic *item)
{
	validate_packet((struct test_data_var *)item);
}

uint32_t rand_get(uint32_t min, uint32_t max)
{
	return min + (sys_rand32_get() % max);
}

void test_overwrite_consistency(void)
{
	struct mpsc_pbuf_buffer buffer;
	static struct mpsc_pbuf_buffer_config cfg = {
		.buf = buf32,
		.size = ARRAY_SIZE(buf32),
		.notify_drop = consistent_drop,
		.get_wlen = get_wlen,
		.flags = MPSC_PBUF_MODE_OVERWRITE
	};

	mpsc_pbuf_init(&buffer, &cfg);
	int repeat = 50000;
	int id = 0;

	while (id < repeat) {
		struct test_data_var *t = NULL;
		bool alloc_during_claim = (rand_get(1, 5) <= 2);

		/* Occasionally claim buffer to simulate that claiming is
		 * interrupted by allocation.
		 */
		if (alloc_during_claim) {
			t = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
			if (t) {
				validate_packet(t);
			}
		}

		uint32_t wr_cnt = rand_get(1, 200);

		for (int i = 0; i < wr_cnt; i++) {
			uint32_t wlen = rand_get(1, 15);
			struct test_data_var *t;

			t = (struct test_data_var *)mpsc_pbuf_alloc(&buffer,
								    wlen,
								    K_NO_WAIT);
			t->hdr.len = wlen;
			t->hdr.data = id++;
			mpsc_pbuf_commit(&buffer, (union mpsc_pbuf_generic *)t);
		}

		/* Put back item claimed before committing new items. */
		if (t) {
			mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)t);
		}

		uint32_t rd_cnt = rand_get(1, 30);

		for (int i = 0; i < rd_cnt; i++) {
			struct test_data_var *t;

			t = (struct test_data_var *)mpsc_pbuf_claim(&buffer);
			if (!t) {
				continue;
			}

			validate_packet(t);
			mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)t);
		}
	}
}

K_THREAD_STACK_DEFINE(t1_stack, 1024);
K_THREAD_STACK_DEFINE(t2_stack, 1024);

static k_thread_stack_t *stacks[2] = {t1_stack, t2_stack};
static struct k_thread threads[2];
static k_tid_t tids[2];

void t_entry(void *p0, void *p1, void *p2)
{
	struct mpsc_pbuf_buffer *buffer = p0;
	uintptr_t wait_ms = (uintptr_t)p1;
	struct test_data_ext *t;

	t = (struct test_data_ext *)mpsc_pbuf_alloc(buffer,
						    sizeof(*t) / sizeof(uint32_t),
						    K_MSEC(1));
	zassert_equal(t, NULL, NULL);

	t = (struct test_data_ext *)mpsc_pbuf_alloc(buffer,
						    sizeof(*t) / sizeof(uint32_t),
						    K_MSEC(wait_ms));
	t->hdr.len = PUT_EXT_LEN;
	t->data = k_current_get();

	mpsc_pbuf_commit(buffer, (union mpsc_pbuf_generic *)t);
	while (1) {
		k_sleep(K_MSEC(10));
	}
}

void start_threads(struct mpsc_pbuf_buffer *buffer)
{
	int prio = 2;
	uintptr_t wait_ms = 1000;

	for (int i = 0; i < ARRAY_SIZE(threads); i++) {
		tids[i] = k_thread_create(&threads[i], stacks[i], 1024, t_entry,
					  buffer, (void *)wait_ms, NULL,
					  prio--,
					  0, K_NO_WAIT);
	}

	k_sleep(K_MSEC(10));

	for (int i = 0; i < ARRAY_SIZE(threads); i++) {
		k_ticks_t t = k_thread_timeout_remaining_ticks(tids[i]);
		k_ticks_t exp_wait = k_ms_to_ticks_ceil32(wait_ms);

		/* Threads shall be blocked, waiting for available space. */
		zassert_within(t, exp_wait, k_ms_to_ticks_ceil32(2), NULL);
	}
}

/* Test creates two threads which pends on the buffer until there is a space
 * available. When engough buffers is released threads are waken up and they
 * allocate packets.
 */
void test_pending_alloc(void)
{
	int prio = k_thread_priority_get(k_current_get());
	struct mpsc_pbuf_buffer buffer;

	k_thread_priority_set(k_current_get(), 3);

	init(&buffer, true, false);

	uint32_t fill_len = 1;
	uint32_t packet_cnt = saturate_buffer_uneven(&buffer, fill_len);

	start_threads(&buffer);

	k_sleep(K_MSEC(1));

	for (int i = 0; i < packet_cnt; i++) {
		union test_item *t = (union test_item *)mpsc_pbuf_claim(&buffer);

		mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)t);
	}


	for (int i = 0; i < 2; i++) {
		struct test_data_ext *t =
			(struct test_data_ext *)mpsc_pbuf_claim(&buffer);

		zassert_true(t, NULL);
		zassert_equal(t->data, tids[ARRAY_SIZE(tids) - 1 - i], NULL);
		mpsc_pbuf_free(&buffer, (union mpsc_pbuf_generic *)t);
	}

	zassert_equal(mpsc_pbuf_claim(&buffer), NULL, "No more packets.");
	k_thread_priority_set(k_current_get(), prio);
}

/*test case main entry*/
void test_main(void)
{
	ztest_test_suite(test_log_buffer,
		ztest_unit_test(test_benchmark_item_put),
		ztest_unit_test(test_item_put_saturate),
		ztest_unit_test(test_item_put_no_overwrite),
		ztest_unit_test(test_item_put_overwrite),
		ztest_unit_test(test_item_put_ext_no_overwrite),
		ztest_unit_test(test_item_put_word_ext_overwrite),
		ztest_unit_test(test_item_put_ext_saturate),
		ztest_unit_test(test_put_data_overwrite),
		ztest_unit_test(test_benchmark_item_put_ext),
		ztest_unit_test(test_benchmark_item_put_data),
		ztest_unit_test(test_item_alloc_commit),
		ztest_unit_test(test_item_alloc_commit_saturate),
		ztest_unit_test(test_item_alloc_preemption),
		ztest_unit_test(test_overwrite),
		ztest_unit_test(test_overwrite_while_claimed),
		ztest_unit_test(test_overwrite_while_claimed2),
		ztest_unit_test(test_overwrite_consistency),
		ztest_unit_test(test_pending_alloc)
		);
	ztest_run_test_suite(test_log_buffer);
}
