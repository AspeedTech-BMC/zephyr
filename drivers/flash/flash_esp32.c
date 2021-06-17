/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_flash_controller
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

#include <kernel.h>
#include <device.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <drivers/flash.h>
#include <soc.h>
#include <esp_spi_flash.h>
#include <esp32/rom/spi_flash.h>

#include "stubs.h"
#include <hal/spi_ll.h>
#include <hal/spi_flash_ll.h>
#include <hal/spi_flash_hal.h>
#include <soc/spi_struct.h>
#include <spi_flash_defs.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(flash_esp32, CONFIG_FLASH_LOG_LEVEL);

struct flash_esp32_dev_config {
	spi_dev_t *controller;
	esp_rom_spiflash_chip_t *chip;
};

struct flash_esp32_dev_data {
	struct k_sem sem;
};

static const struct flash_parameters flash_esp32_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

#define DEV_DATA(dev) ((struct flash_esp32_dev_data *const)(dev)->data)
#define DEV_CFG(dev) ((const struct flash_esp32_dev_config *const)(dev)->config)
#define SPI1_EXTRA_DUMMIES (g_rom_spiflash_dummy_len_plus[1])
#define MAX_BUFF_ALLOC_RETRIES 5
#define MAX_READ_CHUNK 16384
#define ADDRESS_MASK_24BIT 0xFFFFFF
#define SPI_TIMEOUT_MSEC 500

static inline void flash_esp32_sem_take(const struct device *dev)
{
	k_sem_take(&DEV_DATA(dev)->sem, K_FOREVER);
}

static inline void flash_esp32_sem_give(const struct device *dev)
{
	k_sem_give(&DEV_DATA(dev)->sem);
}

static inline int flash_esp32_wait_cmd_done(const spi_dev_t *hw)
{
	int64_t timeout = k_uptime_get() + SPI_TIMEOUT_MSEC;

	while (!spi_flash_ll_cmd_is_done(hw)) {
		if (k_uptime_get() > timeout) {
			LOG_ERR("controller has timed out");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static inline bool points_to_dram(const void *ptr)
{
	return ((intptr_t)ptr >= SOC_DRAM_LOW && (intptr_t)ptr < SOC_DRAM_HIGH);
}

int configure_read_mode(spi_dev_t *hw,
			uint32_t cmd,
			uint32_t addr_bitlen,
			int dummy_len,
			bool byte_cmd)
{
	if (dummy_len) {
		spi_flash_ll_set_dummy(hw, dummy_len);
	}

	spi_flash_ll_set_addr_bitlen(hw, addr_bitlen);

	if (!byte_cmd) {
		REG_SET_FIELD(PERIPHS_SPI_FLASH_USRREG2, SPI_USR_COMMAND_VALUE, cmd);
	} else {
		spi_flash_ll_set_command8(hw, (uint8_t) cmd);
	}

	return 0;
}

static int set_read_options(const struct device *dev)
{
	spi_dev_t *hw = DEV_CFG(dev)->controller;
	uint32_t dummy_len = 0;
	uint32_t addr_len;
	uint8_t read_cmd;
	bool byte_cmd = true;
	uint32_t read_mode = READ_PERI_REG(PERIPHS_SPI_FLASH_CTRL);

	if ((read_mode & SPI_FREAD_QIO) && (read_mode & SPI_FASTRD_MODE)) {
		spi_ll_enable_mosi(hw, 0);
		spi_ll_enable_miso(hw, 1);
		dummy_len = 1 + SPI1_R_QIO_DUMMY_CYCLELEN + SPI1_EXTRA_DUMMIES;
		addr_len = SPI1_R_QIO_ADDR_BITSLEN + 1;
		read_cmd = CMD_FASTRD_QIO;
	} else if (read_mode & SPI_FASTRD_MODE) {
		spi_ll_enable_mosi(hw, 0);
		spi_ll_enable_miso(hw, 1);
		if (read_mode & SPI_FREAD_DIO) {
			read_cmd = CMD_FASTRD_DIO;
			if (SPI1_EXTRA_DUMMIES == 0) {
				spi_flash_ll_set_dummy(hw, 0);
				addr_len = SPI1_R_DIO_ADDR_BITSLEN + 1;
			} else {
				byte_cmd = false;
				dummy_len = SPI1_EXTRA_DUMMIES;
				addr_len = SPI1_R_DIO_ADDR_BITSLEN + 1;
			}
		} else {
			if ((read_mode & SPI_FREAD_QUAD)) {
				read_cmd = CMD_FASTRD_QUAD;
			} else if ((read_mode & SPI_FREAD_DUAL)) {
				read_cmd = CMD_FASTRD_DUAL;
			} else {
				read_cmd = CMD_FASTRD;
			}
			dummy_len = 1 + SPI1_R_FAST_DUMMY_CYCLELEN + SPI1_EXTRA_DUMMIES;
			addr_len = SPI1_R_FAST_ADDR_BITSLEN + 1;
		}
	} else {
		spi_ll_enable_mosi(hw, 0);
		if (SPI1_EXTRA_DUMMIES == 0) {
			spi_flash_ll_set_dummy(hw, 0);
		} else {
			dummy_len = SPI1_EXTRA_DUMMIES;
		}
		spi_ll_enable_miso(hw, 1);
		addr_len = SPI1_R_SIO_ADDR_BITSLEN + 1;
		read_cmd = CMD_READ;
	}

	return configure_read_mode(hw, read_cmd, addr_len, dummy_len, byte_cmd);
}

static int read_once(const struct device *dev, void *buffer, uint32_t address, uint32_t read_len)
{
	spi_dev_t *hw = DEV_CFG(dev)->controller;
	int bitlen = spi_flash_ll_get_addr_bitlen(hw);

	spi_flash_ll_set_usr_address(hw, address << (bitlen - 24), bitlen);
	spi_flash_ll_set_miso_bitlen(hw, read_len * 8);
	spi_flash_ll_user_start(hw);

	int rc = flash_esp32_wait_cmd_done(hw);

	if (rc != 0) {
		return rc;
	}

	spi_flash_ll_get_buffer_data(hw, buffer, read_len);
	return 0;
}

static int read_data(const struct device *dev, uint8_t *buffer, uint32_t address, uint32_t length)
{
	int rc = 0;

	rc = set_read_options(dev);

	if (rc == -ENOTSUP) {
		LOG_ERR("configure host io mode failed - unsupported");
		return rc;
	}

	while (rc == 0 && length > 0) {
		uint32_t read_len = MIN(length, SPI_FLASH_HAL_MAX_READ_BYTES);

		rc = read_once(dev, buffer, address, read_len);

		address += read_len;
		length -= read_len;
		buffer += read_len;
	}

	return rc;
}

static int flash_esp32_read(const struct device *dev, off_t address, void *buffer, size_t length)
{
	const struct flash_esp32_dev_config *const cfg = DEV_CFG(dev);
	const spi_flash_guard_funcs_t *guard = spi_flash_guard_get();
	uint32_t chip_size = cfg->chip->chip_size;

	if (length == 0) {
		return 0;
	}

	if (buffer == NULL || address > chip_size || address + length > chip_size) {
		return -EINVAL;
	}

	bool direct_read = points_to_dram(buffer);
	uint8_t *temp_buff = NULL;
	size_t read_chunk = MIN(MAX_READ_CHUNK, length);
	size_t temp_chunk = MAX_READ_CHUNK;
	int rc = 0;

	flash_esp32_sem_take(dev);

	if (!direct_read) {

		unsigned int retries = MAX_BUFF_ALLOC_RETRIES;

		while (temp_buff == NULL && retries--) {
			read_chunk = MIN(read_chunk, temp_chunk);
			temp_chunk >>= 1;
			read_chunk = (read_chunk + 3) & ~3;
			temp_buff = k_malloc(read_chunk);
		}

		LOG_INF("allocate temp buffer: %p (%d)", temp_buff, read_chunk);

		if (temp_buff == NULL) {
			rc = -ENOMEM;
			goto out;
		}
	}

	uint8_t *buff = (uint8_t *)buffer;

	do {
		guard->start();

		uint8_t *read_buff = (temp_buff) ? temp_buff : buffer;
		size_t read_len = MIN(read_chunk, length);

		rc = read_data(dev, read_buff, address, read_len);

		if (rc) {
			guard->end();
			break;
		}

		guard->end();

		if (temp_buff) {
			memcpy(buffer, temp_buff, read_len);
		}

		address += read_len;
		length -= read_len;
		buff += read_len;
		buffer = (void *)buff;
	} while (rc == 0 && length > 0);

	k_free(temp_buff);

out:
	flash_esp32_sem_give(dev);

	return rc;
}

static inline void set_write_options(const struct device *dev)
{
	spi_dev_t *hw = DEV_CFG(dev)->controller;

	spi_flash_ll_set_dummy(hw, 0);
	/* only single line flash write is currently supported */
	spi_flash_ll_set_addr_bitlen(hw, (1 + ESP_ROM_SPIFLASH_W_SIO_ADDR_BITSLEN));
}

static int read_status(const struct device *dev, uint32_t *status)
{
	const struct flash_esp32_dev_config *const cfg = DEV_CFG(dev);
	uint32_t status_value = ESP_ROM_SPIFLASH_BUSY_FLAG;

	if (SPI1_EXTRA_DUMMIES == 0) {
		while (ESP_ROM_SPIFLASH_BUSY_FLAG ==
		       (status_value & ESP_ROM_SPIFLASH_BUSY_FLAG)) {
			WRITE_PERI_REG(PERIPHS_SPI_FLASH_STATUS, 0);
			WRITE_PERI_REG(PERIPHS_SPI_FLASH_CMD, SPI_FLASH_RDSR);

			int rc = flash_esp32_wait_cmd_done(cfg->controller);

			if (rc != 0) {
				return rc;
			}

			status_value = READ_PERI_REG(PERIPHS_SPI_FLASH_STATUS);
			status_value &= cfg->chip->status_mask;
		}
	} else {
		while (ESP_ROM_SPIFLASH_BUSY_FLAG == (status_value & ESP_ROM_SPIFLASH_BUSY_FLAG)) {
			esp_rom_spiflash_read_user_cmd(&status_value, CMD_RDSR);
		}
	}
	*status = status_value;

	return 0;
}

static inline bool host_idle(spi_dev_t *hw)
{
	bool idle = spi_flash_ll_host_idle(hw);

	idle &= spi_flash_ll_host_idle(&SPI0);

	return idle;
}

static int wait_idle(const struct device *dev)
{
	const struct flash_esp32_dev_config *const cfg = DEV_CFG(dev);
	uint32_t status;
	int64_t timeout = k_uptime_get() + SPI_TIMEOUT_MSEC;

	/* wait for spi control ready */
	while (host_idle(cfg->controller)) {
		if (k_uptime_get() > timeout) {
			return -ETIMEDOUT;
		}
	}

	/* wait for flash status ready */
	if (read_status(dev, &status) != 0) {
		return -EINVAL;
	}
	return 0;
}

static int write_protect(const struct device *dev, bool write_protect)
{
	const struct flash_esp32_dev_config *const cfg = DEV_CFG(dev);
	uint32_t flash_status = 0;

	wait_idle(dev);

	/* enable writing */
	spi_flash_ll_set_write_protect(cfg->controller, write_protect);

	int rc = flash_esp32_wait_cmd_done(cfg->controller);

	if (rc != 0) {
		return rc;
	}

	/* make sure the flash is ready for writing */
	while (ESP_ROM_SPIFLASH_WRENABLE_FLAG != (flash_status & ESP_ROM_SPIFLASH_WRENABLE_FLAG)) {
		read_status(dev, &flash_status);
	}

	return 0;
}

static int program_page(const struct device *dev, uint32_t spi_addr,
			uint32_t *addr_source, int32_t byte_length)
{
	const struct flash_esp32_dev_config *const cfg = DEV_CFG(dev);
	spi_dev_t *hw = DEV_CFG(dev)->controller;

	/* check 4byte alignment */
	if ((byte_length & 0x3) != 0) {
		return -EINVAL;
	}

	/* check if write in one page */
	if ((cfg->chip->page_size) < ((spi_addr % (cfg->chip->page_size)) + byte_length)) {
		return -EINVAL;
	}

	wait_idle(dev);

	uint32_t addr;
	uint32_t prog_len;

	while (byte_length > 0) {
		if (write_protect(dev, false) != 0) {
			return -EINVAL;
		}

		addr = spi_addr & ADDRESS_MASK_24BIT;

		if (byte_length >= ESP_ROM_SPIFLASH_BUFF_BYTE_WRITE_NUM) {
			addr |= ESP_ROM_SPIFLASH_BUFF_BYTE_WRITE_NUM << ESP_ROM_SPIFLASH_BYTES_LEN;
			prog_len = (uint32_t)ESP_ROM_SPIFLASH_BUFF_BYTE_WRITE_NUM;
			spi_flash_ll_set_address(hw, addr);
			spi_flash_ll_program_page(hw, addr_source, prog_len);
			byte_length -= ESP_ROM_SPIFLASH_BUFF_BYTE_WRITE_NUM;
			spi_addr += ESP_ROM_SPIFLASH_BUFF_BYTE_WRITE_NUM;
		} else {
			addr |= byte_length << ESP_ROM_SPIFLASH_BYTES_LEN;
			prog_len = (uint32_t)byte_length;
			spi_flash_ll_set_address(hw, addr);
			spi_flash_ll_program_page(hw, addr_source, prog_len);
			byte_length = 0;
		}

		int rc = flash_esp32_wait_cmd_done(hw);

		if (rc != 0) {
			return rc;
		}

		wait_idle(dev);
	}

	return 0;
}

static int flash_esp32_write(const struct device *dev,
			     off_t address,
			     const void *buffer,
			     size_t length)
{
	const struct flash_esp32_dev_config *const cfg = DEV_CFG(dev);

	uint32_t page_size;
	uint32_t prog_len, prog_num;
	int rc = 0;

	flash_esp32_sem_take(dev);

	set_write_options(dev);

	/* check program size */
	if ((address + length) > (cfg->chip->chip_size)) {
		rc = -EINVAL;
		goto out;
	}

	page_size = cfg->chip->page_size;
	prog_len = page_size - (address % page_size);
	if (length < prog_len) {
		rc = program_page(dev, address, (uint32_t *)buffer, length);
		if (rc) {
			goto out;
		}
	} else {
		rc = program_page(dev, address, (uint32_t *)buffer, prog_len);

		if (rc) {
			goto out;
		}

		/* whole page program */
		prog_num = (length - prog_len) / page_size;
		for (uint8_t i = 0; i < prog_num; ++i) {
			rc = program_page(dev,
					  address + prog_len,
					  (uint32_t *)buffer + (prog_len >> 2),
					  page_size);

			if (rc) {
				goto out;
			}

			prog_len += page_size;
		}

		/* remain parts to program */
		rc = program_page(dev,
				  address + prog_len,
				  (uint32_t *)buffer + (prog_len >> 2),
				  length - prog_len);
		if (rc) {
			LOG_ERR("invalid page programming setting");
		}
	}

out:
	flash_esp32_sem_give(dev);

	return rc;
}

static int erase_sector(const struct device *dev, uint32_t start_addr)
{
	spi_dev_t *hw = DEV_CFG(dev)->controller;
	int rc = write_protect(dev, false);

	if (rc == 0) {
		rc = wait_idle(dev);
	}

	if (rc == 0) {
		spi_flash_ll_set_addr_bitlen(hw, 24);
		spi_flash_ll_set_address(hw, start_addr & ADDRESS_MASK_24BIT);
		spi_flash_ll_erase_sector(hw);

		rc = flash_esp32_wait_cmd_done(hw);
		if (rc) {
			return rc;
		}

		rc = wait_idle(dev);
		if (rc) {
			LOG_ERR("waiting for host device idle state has failed");
		}
	}

	return rc;
}

static int flash_esp32_erase(const struct device *dev, off_t start, size_t len)
{
	uint32_t block_erase_size = DEV_CFG(dev)->chip->block_size;
	uint32_t sector_size = DEV_CFG(dev)->chip->sector_size;
	uint32_t chip_size = DEV_CFG(dev)->chip->chip_size;
	const spi_flash_guard_funcs_t *guard = spi_flash_guard_get();

	if (sector_size == 0 || (block_erase_size % sector_size) != 0) {
		return -EIO;
	}
	if (start > chip_size || start + len > chip_size) {
		return -EINVAL;
	}
	if ((start % sector_size) != 0 || (len % sector_size) != 0) {
		return -EINVAL;
	}

	flash_esp32_sem_take(dev);

	set_write_options(dev);

	int rc = 0;

	while (rc == 0 && len >= sector_size) {
		guard->start();

		rc = erase_sector(dev, start);
		if (rc) {
			guard->end();
			goto out;
		}

		start += sector_size;
		len -= sector_size;

		guard->end();
	}

out:
	flash_esp32_sem_give(dev);

	return rc;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_esp32_pages_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / FLASH_ERASE_BLK_SZ,
	.pages_size = DT_PROP(SOC_NV_FLASH_NODE, erase_block_size),
};

void flash_esp32_page_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	*layout = &flash_esp32_pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_esp32_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_esp32_parameters;
}

static int flash_esp32_init(const struct device *dev)
{
	struct flash_esp32_dev_data *const dev_data = DEV_DATA(dev);

	k_sem_init(&dev_data->sem, 1, 1);

	return 0;
}

static const struct flash_driver_api flash_esp32_driver_api = {
	.read = flash_esp32_read,
	.write = flash_esp32_write,
	.erase = flash_esp32_erase,
	.get_parameters = flash_esp32_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_esp32_page_layout,
#endif
};

static struct flash_esp32_dev_data flash_esp32_data;

static const struct flash_esp32_dev_config flash_esp32_config = {
	.controller = (spi_dev_t *) DT_INST_REG_ADDR(0),
	.chip = &g_rom_flashchip
};

DEVICE_DT_INST_DEFINE(0, flash_esp32_init,
		      NULL,
		      &flash_esp32_data, &flash_esp32_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &flash_esp32_driver_api);
