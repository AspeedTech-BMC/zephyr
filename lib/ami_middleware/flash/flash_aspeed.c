/*
 * Copyright (c) 2021 AMI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/flash.h>
#include <drivers/spi_nor.h>
#include <flash/flash_aspeed.h>
#include <kernel.h>
#include <sys/util.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr.h>
//#include <flash_master.h>
//#include "flash/spi_flash.h"

#define LOG_MODULE_NAME spi_api

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static char *Flash_Devices_List[6] = {
	"spi1_cs0",
	"spi1_cs1",
	"spi2_cs0",
	"spi2_cs1",
	"fmc_cs0",
	"fmc_cs1"
};

static void Data_dump_buf(uint8_t *buf, uint32_t len)
{
	uint32_t i;

	for (i = 0; i < len; i++) {
		printk("%02x ", buf[i]);
		if (i % 16 == 15)
			printk("\n");
	}
	printk("\n");
}

int SPI_Command_Xfer(struct pflash_master *spi, struct pflash_xfer * xfer)
{
	struct device *flash_device;
	uint32_t FlashSize=0;
	uint8_t DeviceId=0;
	uint32_t sector_sz=0;
	uint32_t page_sz=0;
	int ret  = 0 , i =0;
	char buf[2048];
	int AdrOffset=0, Datalen=0;
	struct device *dev;


	//spi1_cs0
	DeviceId = 0;

	flash_device = device_get_binding(Flash_Devices_List[DeviceId]);
	AdrOffset = xfer->address;
	Datalen = xfer->length;

	//printk("<---- SPI_Command_Xfer Command = %d Device = %s ---->\n",xfer->cmd,Flash_Devices_List[DeviceId]);

	switch(xfer->cmd)
	{
		case SPI_APP_CMD_GET_FLASH_SIZE:
			FlashSize = flash_get_flash_size(flash_device);
			return FlashSize;
		break;
		case SPI_APP_CMD_GET_FLASH_SECTOR_SIZE:
			page_sz = flash_get_write_block_size(flash_device);
			return page_sz;
		break;
		case SPI_APP_CMD_GET_FLASH_BLOCK_SIZE:
			page_sz = (flash_get_write_block_size(flash_device) << 4);
			return page_sz;
		break;
		case MIDLEY_FLASH_CMD_READ:
			ret = flash_read(flash_device, AdrOffset, buf, Datalen);
			Data_dump_buf(buf,Datalen);
		break;
		case MIDLEY_FLASH_CMD_PP://Flash Write
			memset(buf,0xff,Datalen);
			memcpy(buf,xfer->data,Datalen);	
			ret = flash_write(flash_device, AdrOffset, buf, Datalen);
		break;
		case MIDLEY_FLASH_CMD_4K_ERASE:
			sector_sz = flash_get_write_block_size(flash_device);
			ret = flash_erase(flash_device,AdrOffset,sector_sz);
		break;
		case MIDLEY_FLASH_CMD_64K_ERASE:
			sector_sz =  (flash_get_write_block_size(flash_device) << 4);
			ret = flash_erase(flash_device,AdrOffset,sector_sz);
		break;
		case MIDLEY_FLASH_CMD_CE:
			FlashSize = flash_get_flash_size(flash_device);
			ret = flash_erase(flash_device,0,FlashSize);
		break;
		default:
			printk("%d Command is not supported",xfer->cmd);
		break;
	}

	return ret ;
}

