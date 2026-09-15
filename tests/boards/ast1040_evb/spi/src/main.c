/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include "ast_test.h"

#define TEST_ARR_SIZE 0x1000
#define UPDATE_TEST_PATTERN_SIZE (TEST_ARR_SIZE - 4)
#define TEST_ROUNDS 10

static uint8_t __aligned(4) test_arr[TEST_ARR_SIZE];

struct flash_test_dev {
	const struct device *dev;
	off_t offset;
};

static const struct flash_test_dev flash_devs[] = {
	/* fmc_cs0 is 4MB; the first 3MB are reserved, so stay in the top 1MB. */
	{DEVICE_DT_GET(DT_NODELABEL(fmc_cs0)), 0x380000},
	/* spi0/spi1 cs0/cs1 are 128MB and fully scratch - reuse the ast1030 offset. */
	{DEVICE_DT_GET(DT_NODELABEL(spi0_cs0)), 0xE0000},
	{DEVICE_DT_GET(DT_NODELABEL(spi0_cs1)), 0xE0000},
	{DEVICE_DT_GET(DT_NODELABEL(spi1_cs0)), 0xE0000},
	{DEVICE_DT_GET(DT_NODELABEL(spi1_cs1)), 0xE0000},
};

static int do_erase_write_verify(const struct device *dev, uint32_t op_addr, uint8_t *write_buf,
				  uint8_t *read_back_buf, uint32_t erase_sz)
{
	int ret;

	ret = flash_erase(dev, op_addr, erase_sz);
	if (ret != 0) {
		printk("%s: erase @0x%x failed: %d\n", dev->name, op_addr, ret);
		return ret;
	}

	ret = flash_write(dev, op_addr, write_buf, erase_sz);
	if (ret != 0) {
		printk("%s: write @0x%x failed: %d\n", dev->name, op_addr, ret);
		return ret;
	}

	ret = flash_read(dev, op_addr, read_back_buf, erase_sz);
	if (ret != 0) {
		printk("%s: read @0x%x failed: %d\n", dev->name, op_addr, ret);
		return ret;
	}

	if (memcmp(write_buf, read_back_buf, erase_sz) != 0) {
		printk("%s: verify @0x%x mismatch (wrote vs read-back differ)\n", dev->name,
		       op_addr);
		return -EINVAL;
	}

	return 0;
}

static int do_update(const struct device *dev, off_t offset, uint8_t *buf, size_t len)
{
	int ret = 0;
	size_t flash_sz = flash_get_flash_size(dev);
	size_t sector_sz = flash_get_write_block_size(dev);
	uint32_t flash_offset = (uint32_t)offset;
	uint32_t remain, op_addr, end_sector_addr;
	uint8_t *update_ptr = buf, *op_buf = NULL, *read_back_buf = NULL;
	bool update_it = false;

	if (flash_sz < flash_offset + len) {
		printk("%s: flash_sz=%zu sector_sz=%zu too small for offset=0x%x len=%zu\n",
		       dev->name, flash_sz, sector_sz, flash_offset, len);
		return -EINVAL;
	}

	op_buf = k_malloc(sector_sz);
	read_back_buf = k_malloc(sector_sz);
	if (op_buf == NULL || read_back_buf == NULL) {
		printk("%s: k_malloc(%zu) failed\n", dev->name, sector_sz);
		ret = -ENOMEM;
		goto end;
	}

	/* initial op_addr */
	op_addr = (flash_offset / sector_sz) * sector_sz;

	/* handle the start part which is not multiple of sector size */
	if (flash_offset % sector_sz != 0) {
		ret = flash_read(dev, op_addr, op_buf, sector_sz);
		if (ret != 0) {
			printk("%s: start-part read @0x%x failed: %d\n", dev->name, op_addr, ret);
			goto end;
		}

		remain = MIN(sector_sz - (flash_offset % sector_sz), len);
		memcpy((uint8_t *)op_buf + (flash_offset % sector_sz), update_ptr, remain);
		ret = do_erase_write_verify(dev, op_addr, op_buf, read_back_buf, sector_sz);
		if (ret != 0) {
			goto end;
		}

		op_addr += sector_sz;
		update_ptr += remain;
	}

	end_sector_addr = (flash_offset + len) / sector_sz * sector_sz;
	/* handle body */
	for (; op_addr < end_sector_addr;) {
		ret = flash_read(dev, op_addr, op_buf, sector_sz);
		if (ret != 0) {
			printk("%s: body read @0x%x failed: %d\n", dev->name, op_addr, ret);
			goto end;
		}

		if (memcmp(op_buf, update_ptr, sector_sz) != 0) {
			update_it = true;
		}

		if (update_it) {
			ret = do_erase_write_verify(dev, op_addr, update_ptr, read_back_buf,
						     sector_sz);
			if (ret != 0) {
				goto end;
			}
		}

		op_addr += sector_sz;
		update_ptr += sector_sz;
	}

	/* handle remain part */
	if (end_sector_addr < flash_offset + len) {
		ret = flash_read(dev, op_addr, op_buf, sector_sz);
		if (ret != 0) {
			printk("%s: remain-part read @0x%x failed: %d\n", dev->name, op_addr,
			       ret);
			goto end;
		}

		remain = flash_offset + len - end_sector_addr;
		memcpy((uint8_t *)op_buf, update_ptr, remain);

		ret = do_erase_write_verify(dev, op_addr, op_buf, read_back_buf, sector_sz);
	}

end:
	k_free(op_buf);
	k_free(read_back_buf);

	return ret;
}

int test_spi(void)
{
	bool test_repeat = true;

	for (int i = 0; i < ARRAY_SIZE(flash_devs); i++) {
		ast_zassert_true(device_is_ready(flash_devs[i].dev),
				  "flash device %d is not ready", i);
	}

	for (int round = 0; round < TEST_ROUNDS; round++) {
		for (int i = 0; i < UPDATE_TEST_PATTERN_SIZE; i++) {
			test_arr[i] = test_repeat ? ('a' + (i % 26)) : ('z' - (i % 26));
		}

		for (int i = 0; i < ARRAY_SIZE(flash_devs); i++) {
			ast_zassert_ok(do_update(flash_devs[i].dev, flash_devs[i].offset,
						  test_arr, UPDATE_TEST_PATTERN_SIZE),
				       "flash update round %d dev %d failed", round, i);
		}

		test_repeat = !test_repeat;
	}

	return ast_ztest_result();
}

#if !defined(AST1040_CONCURRENT_ALL)
ZTEST(spi, test_spi_flash_erase_write_verify)
{
	zassert_equal(test_spi(), AST_TEST_PASS, "spi test failed");
}

ZTEST_SUITE(spi, NULL, NULL, NULL, NULL, NULL);
#endif
