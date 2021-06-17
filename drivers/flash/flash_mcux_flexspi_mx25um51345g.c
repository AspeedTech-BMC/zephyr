/*
 * Copyright 2021 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT	nxp_imx_flexspi_mx25um51345g

#include <drivers/flash.h>
#include <logging/log.h>
#include <sys/util.h>
#include "spi_nor.h"
#include "memc_mcux_flexspi.h"

#ifdef CONFIG_HAS_MCUX_CACHE
#include <fsl_cache.h>
#endif

#define NOR_WRITE_SIZE	1
#define NOR_ERASE_VALUE	0xff

#ifdef CONFIG_FLASH_MCUX_FLEXSPI_NOR_WRITE_BUFFER
static uint8_t nor_write_buf[SPI_NOR_PAGE_SIZE];
#endif

LOG_MODULE_REGISTER(flash_flexspi_nor, CONFIG_FLASH_LOG_LEVEL);

enum {
	/* Instructions matching with XIP layout */
	READ,
	WRITE_ENABLE_OPI,
	WRITE_ENABLE,
	ERASE_SECTOR,
	PAGE_PROGRAM_INPUT,
	PAGE_PROGRAM,
	READ_ID_OPI,
	ENTER_OPI,
	READ_STATUS_REG,
	ERASE_CHIP,
};

struct flash_flexspi_nor_config {
	char *controller_label;
	flexspi_port_t port;
	flexspi_device_config_t config;
	struct flash_pages_layout layout;
	struct flash_parameters flash_parameters;
};

struct flash_flexspi_nor_data {
	const struct device *controller;
};

static const uint32_t flash_flexspi_nor_lut[][4] = {
	[READ_ID_OPI] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,		kFLEXSPI_8PAD, 0x9F,
				kFLEXSPI_Command_DDR,		kFLEXSPI_8PAD, 0x60),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_DDR,	kFLEXSPI_8PAD, 0x20,
				kFLEXSPI_Command_DUMMY_DDR,	kFLEXSPI_8PAD, 0x16),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR,	kFLEXSPI_8PAD, 0x04,
				kFLEXSPI_Command_STOP,		kFLEXSPI_1PAD, 0x0),
	},

	[READ_STATUS_REG] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0x05,
				kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0xFA),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_SDR,	kFLEXSPI_8PAD, 0x20,
				kFLEXSPI_Command_DUMMY_SDR,	kFLEXSPI_8PAD, 0x14),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR,	kFLEXSPI_8PAD, 0x04,
				kFLEXSPI_Command_STOP,		kFLEXSPI_1PAD, 0x0),
	},

	[WRITE_ENABLE] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,		kFLEXSPI_1PAD, 0x06,
				kFLEXSPI_Command_STOP,		kFLEXSPI_1PAD, 0),
	},

	[WRITE_ENABLE_OPI] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0x06,
				kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0xF9),
	},

	[ERASE_SECTOR] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0x21,
				kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0xDE),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_SDR,	kFLEXSPI_8PAD, 0x20,
				kFLEXSPI_Command_STOP,		kFLEXSPI_8PAD, 0),
	},

	[ERASE_CHIP] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0x60,
				kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0x9F),
	},

	[READ] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0xEC,
				kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0x13),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_SDR,	kFLEXSPI_8PAD, 0x20,
				kFLEXSPI_Command_DUMMY_SDR,	kFLEXSPI_8PAD, 0x14),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR,	kFLEXSPI_8PAD, 0x04,
				kFLEXSPI_Command_STOP,		kFLEXSPI_1PAD, 0x0),
	},

	[PAGE_PROGRAM] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0x12,
				kFLEXSPI_Command_SDR,		kFLEXSPI_8PAD, 0xED),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_RADDR_SDR,	kFLEXSPI_8PAD, 0x20,
				kFLEXSPI_Command_WRITE_SDR,	kFLEXSPI_8PAD, 0x04),
	},

	[ENTER_OPI] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,		kFLEXSPI_1PAD, 0x72,
				kFLEXSPI_Command_RADDR_SDR,	kFLEXSPI_1PAD, 0x20),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR,	kFLEXSPI_1PAD, 0x04,
				kFLEXSPI_Command_STOP,		kFLEXSPI_1PAD, 0),
	},

};

static int flash_flexspi_nor_get_vendor_id(const struct device *dev,
		uint8_t *vendor_id)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;
	uint32_t buffer = 0;
	int ret;

	flexspi_transfer_t transfer = {
		.deviceAddress = 0,
		.port = config->port,
		.cmdType = kFLEXSPI_Read,
		.SeqNumber = 1,
		.seqIndex = READ_ID_OPI,
		.data = &buffer,
		.dataSize = 1,
	};

	LOG_DBG("Reading id");

	ret = memc_flexspi_transfer(data->controller, &transfer);
	*vendor_id = buffer;

	return ret;
}

static int flash_flexspi_nor_read_status(const struct device *dev,
		uint32_t *status)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;

	flexspi_transfer_t transfer = {
		.deviceAddress = 0,
		.port = config->port,
		.cmdType = kFLEXSPI_Read,
		.SeqNumber = 1,
		.seqIndex = READ_STATUS_REG,
		.data = status,
		.dataSize = 1,
	};

	LOG_DBG("Reading status register");

	return memc_flexspi_transfer(data->controller, &transfer);
}

static int flash_flexspi_nor_write_status(const struct device *dev,
		uint32_t *status)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;

	flexspi_transfer_t transfer = {
		.deviceAddress = 0,
		.port = config->port,
		.cmdType = kFLEXSPI_Write,
		.SeqNumber = 1,
		.seqIndex = ENTER_OPI,
		.data = status,
		.dataSize = 1,
	};

	LOG_DBG("Writing status register");

	return memc_flexspi_transfer(data->controller, &transfer);
}

static int flash_flexspi_nor_write_enable(const struct device *dev,
		bool enableOctal)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;
	flexspi_transfer_t transfer;

	transfer.deviceAddress = 0;
	transfer.port = config->port;
	transfer.cmdType = kFLEXSPI_Command;
	transfer.SeqNumber = 1;
	if (enableOctal) {
		transfer.seqIndex = WRITE_ENABLE_OPI;
	} else {
		transfer.seqIndex = WRITE_ENABLE;
	}
	transfer.data = NULL;
	transfer.dataSize = 0;

	LOG_DBG("Enabling write");

	return memc_flexspi_transfer(data->controller, &transfer);
}

static int flash_flexspi_nor_erase_sector(const struct device *dev,
		off_t offset)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;

	flexspi_transfer_t transfer = {
		.deviceAddress = offset,
		.port = config->port,
		.cmdType = kFLEXSPI_Command,
		.SeqNumber = 1,
		.seqIndex = ERASE_SECTOR,
		.data = NULL,
		.dataSize = 0,
	};

	LOG_DBG("Erasing sector at 0x%08x", offset);

	return memc_flexspi_transfer(data->controller, &transfer);
}

static int flash_flexspi_nor_erase_chip(const struct device *dev)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;

	flexspi_transfer_t transfer = {
		.deviceAddress = 0,
		.port = config->port,
		.cmdType = kFLEXSPI_Command,
		.SeqNumber = 1,
		.seqIndex = ERASE_CHIP,
		.data = NULL,
		.dataSize = 0,
	};

	LOG_DBG("Erasing chip");

	return memc_flexspi_transfer(data->controller, &transfer);
}

static int flash_flexspi_nor_page_program(const struct device *dev,
		off_t offset, const void *buffer, size_t len)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;

	flexspi_transfer_t transfer = {
		.deviceAddress = offset,
		.port = config->port,
		.cmdType = kFLEXSPI_Write,
		.SeqNumber = 1,
		.seqIndex = PAGE_PROGRAM,
		.data = (uint32_t *) buffer,
		.dataSize = len,
	};

	LOG_DBG("Page programming %d bytes to 0x%08x", len, offset);

	return memc_flexspi_transfer(data->controller, &transfer);
}

static int flash_flexspi_nor_wait_bus_busy(const struct device *dev)
{
	uint32_t status = 0;
	int ret;

	do {
		ret = flash_flexspi_nor_read_status(dev, &status);
		LOG_DBG("status: 0x%x", status);
		if (ret) {
			LOG_ERR("Could not read status");
			return ret;
		}
	} while (status & BIT(0));

	return 0;
}

static int flash_flexspi_enable_octal_mode(const struct device *dev)
{
	struct flash_flexspi_nor_data *data = dev->data;
	/* FLASH_ENABLE_OCTAL_CMD: (01 = STR OPI Enable) */
	uint32_t status = 0x01;

	flash_flexspi_nor_write_enable(dev, false);
	flash_flexspi_nor_write_status(dev, &status);
	flash_flexspi_nor_wait_bus_busy(dev);
	memc_flexspi_reset(data->controller);

	return 0;
}

static int flash_flexspi_nor_read(const struct device *dev, off_t offset,
		void *buffer, size_t len)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;
	uint8_t *src = memc_flexspi_get_ahb_address(data->controller,
						    config->port,
						    offset);

	memcpy(buffer, src, len);

	return 0;
}

static int flash_flexspi_nor_write(const struct device *dev, off_t offset,
		const void *buffer, size_t len)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;
	size_t size = len;
	uint8_t *src = (uint8_t *) buffer;
	int i;
	unsigned int key = 0;

	uint8_t *dst = memc_flexspi_get_ahb_address(data->controller,
						    config->port,
						    offset);

	if (memc_flexspi_is_running_xip(data->controller)) {
		key = irq_lock();
	}

	while (len) {
		i = MIN(SPI_NOR_PAGE_SIZE, len);
#ifdef CONFIG_FLASH_MCUX_FLEXSPI_NOR_WRITE_BUFFER
		memcpy(nor_write_buf, src, i);
#endif
		flash_flexspi_nor_write_enable(dev, true);
#ifdef CONFIG_FLASH_MCUX_FLEXSPI_NOR_WRITE_BUFFER
		flash_flexspi_nor_page_program(dev, offset, nor_write_buf, i);
#else
		flash_flexspi_nor_page_program(dev, offset, src, i);
#endif
		flash_flexspi_nor_wait_bus_busy(dev);
		memc_flexspi_reset(data->controller);
		src += i;
		offset += i;
		len -= i;
	}

	if (memc_flexspi_is_running_xip(data->controller)) {
		irq_unlock(key);
	}

#ifdef CONFIG_HAS_MCUX_CACHE
	DCACHE_InvalidateByRange((uint32_t) dst, size);
#endif

	return 0;
}

static int flash_flexspi_nor_erase(const struct device *dev, off_t offset,
		size_t size)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;
	int num_sectors = size / SPI_NOR_SECTOR_SIZE;
	int i;
	unsigned int key = 0;

	uint8_t *dst = memc_flexspi_get_ahb_address(data->controller,
						    config->port,
						    offset);

	if (offset % SPI_NOR_SECTOR_SIZE) {
		LOG_ERR("Invalid offset");
		return -EINVAL;
	}

	if (size % SPI_NOR_SECTOR_SIZE) {
		LOG_ERR("Invalid size");
		return -EINVAL;
	}

	if (memc_flexspi_is_running_xip(data->controller)) {
		key = irq_lock();
	}

	if ((offset == 0) && (size == config->config.flashSize * KB(1))) {
		flash_flexspi_nor_write_enable(dev, true);
		flash_flexspi_nor_erase_chip(dev);
		flash_flexspi_nor_wait_bus_busy(dev);
		memc_flexspi_reset(data->controller);
	} else {
		for (i = 0; i < num_sectors; i++) {
			flash_flexspi_nor_write_enable(dev, true);
			flash_flexspi_nor_erase_sector(dev, offset);
			flash_flexspi_nor_wait_bus_busy(dev);
			memc_flexspi_reset(data->controller);
			offset += SPI_NOR_SECTOR_SIZE;
		}
	}

	if (memc_flexspi_is_running_xip(data->controller)) {
		irq_unlock(key);
	}

#ifdef CONFIG_HAS_MCUX_CACHE
	DCACHE_InvalidateByRange((uint32_t) dst, size);
#endif

	return 0;
}

static const struct flash_parameters *flash_flexspi_nor_get_parameters(
		const struct device *dev)
{
	const struct flash_flexspi_nor_config *config = dev->config;

	return &config->flash_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_flexspi_nor_pages_layout(const struct device *dev,
		const struct flash_pages_layout **layout, size_t *layout_size)
{
	const struct flash_flexspi_nor_config *config = dev->config;

	*layout = &config->layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int flash_flexspi_nor_init(const struct device *dev)
{
	const struct flash_flexspi_nor_config *config = dev->config;
	struct flash_flexspi_nor_data *data = dev->data;
	uint8_t vendor_id;

	data->controller = device_get_binding(config->controller_label);
	if (data->controller == NULL) {
		LOG_ERR("Could not find controller");
		return -EINVAL;
	}

	if (!memc_flexspi_is_running_xip(data->controller) &&
	    memc_flexspi_set_device_config(data->controller, &config->config,
					   config->port)) {
		LOG_ERR("Could not set device configuration");
		return -EINVAL;
	}

	if (memc_flexspi_update_lut(data->controller, 0,
				   (const uint32_t *) flash_flexspi_nor_lut,
				   sizeof(flash_flexspi_nor_lut) / 4)) {
		LOG_ERR("Could not update lut");
		return -EINVAL;
	}

	memc_flexspi_reset(data->controller);

	if (flash_flexspi_enable_octal_mode(dev)) {
		LOG_ERR("Could not enable octal mode");
		return -EIO;
	}

	if (flash_flexspi_nor_get_vendor_id(dev, &vendor_id)) {
		LOG_ERR("Could not read vendor id");
		return -EIO;
	}
	LOG_DBG("Vendor id: 0x%0x", vendor_id);

	return 0;
}

static const struct flash_driver_api flash_flexspi_nor_api = {
	.erase = flash_flexspi_nor_erase,
	.write = flash_flexspi_nor_write,
	.read = flash_flexspi_nor_read,
	.get_parameters = flash_flexspi_nor_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_flexspi_nor_pages_layout,
#endif
};

#define CONCAT3(x, y, z) x ## y ## z

#define CS_INTERVAL_UNIT(unit)						\
	CONCAT3(kFLEXSPI_CsIntervalUnit, unit, SckCycle)

#define AHB_WRITE_WAIT_UNIT(unit)					\
	CONCAT3(kFLEXSPI_AhbWriteWaitUnit, unit, AhbCycle)

#define FLASH_FLEXSPI_DEVICE_CONFIG(n)					\
	{								\
		.flexspiRootClk = MHZ(120),				\
		.flashSize = DT_INST_PROP(n, size) / 8 / KB(1),		\
		.CSIntervalUnit =					\
			CS_INTERVAL_UNIT(				\
				DT_INST_PROP(n, cs_interval_unit)),	\
		.CSInterval = DT_INST_PROP(n, cs_interval),		\
		.CSHoldTime = DT_INST_PROP(n, cs_hold_time),		\
		.CSSetupTime = DT_INST_PROP(n, cs_setup_time),		\
		.dataValidTime = DT_INST_PROP(n, data_valid_time),	\
		.columnspace = DT_INST_PROP(n, column_space),		\
		.enableWordAddress = DT_INST_PROP(n, word_addressable),	\
		.AWRSeqIndex = 0,					\
		.AWRSeqNumber = 0,					\
		.ARDSeqIndex = READ,					\
		.ARDSeqNumber = 1,					\
		.AHBWriteWaitUnit =					\
			AHB_WRITE_WAIT_UNIT(				\
				DT_INST_PROP(n, ahb_write_wait_unit)),	\
		.AHBWriteWaitInterval =					\
			DT_INST_PROP(n, ahb_write_wait_interval),	\
	}								\

#define FLASH_FLEXSPI_NOR(n)						\
	static const struct flash_flexspi_nor_config			\
		flash_flexspi_nor_config_##n = {			\
		.controller_label = DT_INST_BUS_LABEL(n),		\
		.port = DT_INST_REG_ADDR(n),				\
		.config = FLASH_FLEXSPI_DEVICE_CONFIG(n),		\
		.layout = {						\
			.pages_count = DT_INST_PROP(n, size) / 8	\
				/ SPI_NOR_SECTOR_SIZE,			\
			.pages_size = SPI_NOR_SECTOR_SIZE,		\
		},							\
		.flash_parameters = {					\
			.write_block_size = NOR_WRITE_SIZE,		\
			.erase_value = NOR_ERASE_VALUE,			\
		},							\
	};								\
									\
	static struct flash_flexspi_nor_data				\
		flash_flexspi_nor_data_##n;				\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			      flash_flexspi_nor_init,			\
			      NULL,					\
			      &flash_flexspi_nor_data_##n,		\
			      &flash_flexspi_nor_config_##n,		\
			      POST_KERNEL,				\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &flash_flexspi_nor_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_FLEXSPI_NOR)
