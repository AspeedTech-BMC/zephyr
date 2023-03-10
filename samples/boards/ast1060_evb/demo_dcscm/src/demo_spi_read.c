/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/flash.h>
#include <drivers/spi_nor.h>
#include <kernel.h>
#include <sys/util.h>
#include <soc.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr.h>

#define LOG_MODULE_NAME spi_read_demo

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define SPI_FLASH_TEST_SECTOR_SIZE 0x1000
#define SPI_FLASH_TEST_SIZE SPI_FLASH_TEST_SECTOR_SIZE
#define SPI_FLASH_TEST_ARR_SIZE 0x1100

static uint8_t op_arr[SPI_FLASH_TEST_SIZE] NON_CACHED_BSS_ALIGN16;

#define HOST_SPI_MONITOR_NUM 5

static char *flash_devices[HOST_SPI_MONITOR_NUM] = {
	"fmc_cs0",
	"fmc_cs1",
	"spi1_cs0",
	"spi2_cs0",
	"spi2_cs1",
};

static void spi_flash_dump_buf(uint8_t *buf, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++) {
		printk("%02x ", buf[i]);
		if (i % 16 == 15)
			printk("\n");
	}
	printk("\n");
}

/* Just demo read operation here */
int demo_spi_host_read(void)
{
	int ret = 0;
	const struct device *flash_dev;
	uint8_t *op_buf = NULL;
	uint32_t i;

	op_buf = (uint8_t *)op_arr;

	for (i = 0; i < HOST_SPI_MONITOR_NUM; i++) {
		flash_dev = device_get_binding(flash_devices[i]);
		if (!flash_dev) {
			LOG_ERR("No device named %s.", flash_devices[i]);
			return -ENOEXEC;
		}

		ret = spi_nor_rst_by_cmd(flash_dev);
		if (ret) {
			LOG_ERR("fail to reset flash %s", flash_devices[i]);
			goto end;
		}

		ret = flash_read(flash_dev, 0x0, op_buf, SPI_FLASH_TEST_SIZE);
		if (ret) {
			LOG_ERR("fail to read flash %s", flash_devices[i]);
			goto end;
		}

		printk("[%s]read result:\n", flash_devices[i]);
		spi_flash_dump_buf(op_buf, 4);
		spi_flash_dump_buf(op_buf + SPI_FLASH_TEST_SIZE - 4, 4);

		ret = spi_nor_config_4byte_mode(flash_dev, false);
		if (ret)
			goto end;
	}

	k_busy_wait(20000); /* 20ms */

end:

	return ret;
}
