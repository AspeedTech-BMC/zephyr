/*
 * Copyright (c) 2021 AMI
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_SPI_API_MIDLEYER_H_
#define ZEPHYR_INCLUDE_SPI_API_MIDLEYER_H_

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>

enum 
{
	SPI_APP_CMD_NOOP  = 0x00,				/**< No-op */
	SPI_APP_CMD_READ  = 0x01,
	SPI_APP_CMD_WRITE = 0x02,
	SPI_APP_CMD_ERASE_SECTOR = 0x03,
	SPI_APP_CMD_ERASE_BLOCK = 0x04,
	SPI_APP_CMD_ERASE_CHIP = 0x05,
	SPI_APP_CMD_GET_FLASH_SECTOR_SIZE = 0x06,
	SPI_APP_CMD_GET_FLASH_BLOCK_SIZE = 0x07,
	SPI_APP_CMD_GET_FLASH_PAGE_SIZE = 0x08,
	SPI_APP_CMD_GET_FLASH_SIZE = 0x09,
};

enum {
    MIDLEY_FLASH_CMD_NOOP = 0x00,				/**< No-op */
	//MIDLEY_FLASH_CMD_WRSR = 0x01,				/**< Write status register */
	MIDLEY_FLASH_CMD_PP = 0x02,				    /**< Page program */
	MIDLEY_FLASH_CMD_READ = 0x03,				/**< Normal read */
	//MIDLEY_FLASH_CMD_WRDI = 0x04,				/**< Write disable */
	//MIDLEY_FLASH_CMD_RDSR = 0x05,				/**< Read status register */
	//MIDLEY_FLASH_CMD_WREN = 0x06,				/**< Write enable */
	//MIDLEY_FLASH_CMD_FAST_READ = 0x0b,		/**< Fast read */
	//MIDLEY_FLASH_CMD_4BYTE_FAST_READ = 0x0c,	/**< Fast read with 4 byte address */
	//MIDLEY_FLASH_CMD_4BYTE_PP = 0x12,			/**< Page program with 4 byte address */
	//MIDLEY_FLASH_CMD_4BYTE_READ = 0x13,		/**< Normal read with 4 byte address */
	//MIDLEY_FLASH_CMD_RDSR3 = 0x15,			/**< Read status register 3 (configuration register) */
	MIDLEY_FLASH_CMD_4K_ERASE = 0x20,			/**< Sector erase 4kB */
	//MIDLEY_FLASH_CMD_4BYTE_4K_ERASE = 0x21,	/**< Sector erase 4kB with 4 byte address */
	//MIDLEY_FLASH_CMD_WRSR2 = 0x31,			/**< Write status register 2 */
	//MIDLEY_FLASH_CMD_RDSR2 = 0x35,			/**< Read status register 2 */
	//MIDLEY_FLASH_CMD_DUAL_READ = 0x3b,		/**< Dual output read */
	//MIDLEY_FLASH_CMD_4BYTE_DUAL_READ = 0x3c,	/**< Dual output read with 4 byte address */
	//MIDLEY_FLASH_CMD_ALT_WRSR2 = 0x3e,		/**< Alternate Write status register 2 */
	//MIDLEY_FLASH_CMD_ALT_RDSR2 = 0x3f,		/**< Alternate Read status register 2 */
	//MIDLEY_FLASH_CMD_VOLATILE_WREN = 0x50,	/**< Volatile write enabl efor status register 1 */
	//MIDLEY_FLASH_CMD_SFDP = 0x5a,				/**< Read SFDP registers */
	//MIDLEY_FLASH_CMD_RSTEN = 0x66,			/**< Reset enable */
	//MIDLEY_FLASH_CMD_QUAD_READ = 0x6b,		/**< Quad output read */
	//MIDLEY_FLASH_CMD_4BYTE_QUAD_READ = 0x6c,	/**< Quad output read with 4 byte address */
	//MIDLEY_FLASH_CMD_RDSR_FLAG = 0x70,		/**< Read flag status register */
	//MIDLEY_FLASH_CMD_RST = 0x99,				/**< Reset device */
	//MIDLEY_FLASH_CMD_RDID = 0x9f,				/**< Read identification */
	//MIDLEY_FLASH_CMD_RDP = 0xab,				/**< Release from deep power down */
	//MIDLEY_FLASH_CMD_WR_NV_CFG = 0xb1,		/**< Write non-volatile configuration register */
	//MIDLEY_FLASH_CMD_RD_NV_CFG = 0xb5,		/**< Read non-volatile configuration register */
	//MIDLEY_FLASH_CMD_EN4B = 0xb7,				/**< Enter 4-byte mode */
	//MIDLEY_FLASH_CMD_DP = 0xb9,				/**< Deep power down the device */
	//MIDLEY_FLASH_CMD_DIO_READ = 0xbb,			/**< Dual I/O read */
	//MIDLEY_FLASH_CMD_4BYTE_DIO_READ = 0xbc,	/**< Dual I/O read with 4 byte address */
	MIDLEY_FLASH_CMD_CE = 0xc7,				    /**< Chip erase */
	MIDLEY_FLASH_CMD_64K_ERASE = 0xd8,			/**< Block erase 64kB */
	//MIDLEY_FLASH_CMD_4BYTE_64K_ERASE = 0xdc,	/**< Block erase 64kB with 4 byte address */
	//MIDLEY_FLASH_CMD_EX4B = 0xe9,				/**< Exit 4-byte mode */
	//MIDLEY_FLASH_CMD_QIO_READ = 0xeb,			/**< Quad I/O read */
	//MIDLEY_FLASH_CMD_4BYTE_QIO_READ = 0xec,	/**< Quad I/O read with 4 byte address */
	//MIDLEY_FLASH_CMD_ALT_RST = 0xf0,			/**< Alternate reset command supported by some devices. */
};

#if 1

/**
 * Specifies a transaction to be executed by the SPI master.
 */
struct pflash_xfer {
	uint32_t address;		/**< The address for the command. */
	uint8_t *data;			/**< The buffer for the command data. */
	uint32_t length;		/**< The length of the command data. */
	uint8_t cmd;			/**< The flash command code. */
	uint8_t dummy_bytes;	/**< The number of dummy bytes in the transaction. */
	uint8_t mode_bytes;		/**< The number of mode bytes in the transaction. */
	uint16_t flags;			/**< Transaction flags. */
};


/**
 * Defines the interface to the SPI master connected to a flash device.
 */
struct pflash_master {
	/**
	 * Submit a transfer to be executed by the SPI master.
	 *
	 * @param spi The SPI master to use to execute the transfer.
	 * @param xfer The transfer to execute.
	 *
	 * @return 0 if the transfer was executed successfully or an error code.
	 */
	int (*xfer) (struct flash_master *spi, const struct flash_xfer *xfer);

	/**
	 * Get a set of capabilities supported by the SPI master.
	 *
	 * @param spi The SPI master to query.
	 *
	 * @return A capabilities bitmask for the SPI master.
	 */
	uint32_t (*capabilities) (struct flash_master *spi);
};
#endif

int SPI_Command_Xfer(struct pflash_master *spi, struct pflash_xfer * xfer);


#endif
