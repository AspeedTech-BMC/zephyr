/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_NOR_H__
#define __SPI_NOR_H__

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <zephyr/drivers/jesd216.h>

#define SPI_NOR_MAX_ID_LEN	3

#define SPI_NOR_MFR_ID_WINBOND      0xEF
#define SPI_NOR_MFR_ID_MXIC         0xC2
#define SPI_NOR_MFR_ID_ST           0x20
#define SPI_NOR_MFR_ID_MICRON       0x2C
#define SPI_NOR_MFR_ID_ISSI         0x9D
#define SPI_NOR_MFR_ID_GIGADEVICE   0xC8
#define SPI_NOR_MFR_ID_CYPRESS      0x34

#define SPI_NOR_JEDEC_ID(id) (((id) & 0xff0000) >> 16)

/* Status register bits */
#define SPI_NOR_WIP_BIT         BIT(0)  /* Write in progress */
#define SPI_NOR_WEL_BIT         BIT(1)  /* Write enable latch */

/* Flash opcodes */
#define SPI_NOR_CMD_WRSR        0x01    /* Write status register */
#define SPI_NOR_CMD_RDSR        0x05    /* Read status register */
#define SPI_NOR_CMD_WRSR2       0x31    /* Write status register 2 */
#define SPI_NOR_CMD_RDSR2       0x35    /* Read status register 2 */
#define SPI_NOR_CMD_RDSR3       0x15    /* Read status register 3 */
#define SPI_NOR_CMD_WRSR3       0x11    /* Write status register 3 */
#define SPI_NOR_CMD_READ        0x03    /* Read data */
#define SPI_NOR_CMD_READ_FAST   0x0B    /* Read data */
#define SPI_NOR_CMD_DREAD       0x3B    /* Read data (1-1-2) */
#define SPI_NOR_CMD_2READ       0xBB    /* Read data (1-2-2) */
#define SPI_NOR_CMD_QREAD       0x6B    /* Read data (1-1-4) */
#define SPI_NOR_CMD_4READ       0xEB    /* Read data (1-4-4) */
#define SPI_NOR_CMD_WREN        0x06    /* Write enable */
#define SPI_NOR_CMD_WRDI        0x04    /* Write disable */
#define SPI_NOR_CMD_PP          0x02    /* Page program */
#define SPI_NOR_CMD_PP_1_1_2    0xA2    /* Dual Page program (1-1-2) */
#define SPI_NOR_CMD_PP_1_1_4    0x32    /* Quad Page program (1-1-4) */
#define SPI_NOR_CMD_PP_1_4_4    0x38    /* Quad Page program (1-4-4) */
#define SPI_NOR_CMD_RDCR        0x15    /* Read control register */
#define SPI_NOR_CMD_SE          0x20    /* Sector erase */
#define SPI_NOR_CMD_SE_4B       0x21    /* Sector erase 4 byte address*/
#define SPI_NOR_CMD_BE_32K      0x52    /* Block erase 32KB */
#define SPI_NOR_CMD_BE          0xD8    /* Block erase */
#define SPI_NOR_CMD_SE          0x20    /* Sector erase */
#define SPI_NOR_CMD_BE_32K_4B   0x5C    /* Block erase 32KB */
#define SPI_NOR_CMD_BE_4B       0xDC    /* Block erase */
#define SPI_NOR_CMD_CE          0xC7    /* Chip erase */
#define SPI_NOR_CMD_RDID        0x9F    /* Read JEDEC ID */
#define SPI_NOR_CMD_ULBPR       0x98    /* Global Block Protection Unlock */
#define SPI_NOR_CMD_4BA         0xB7    /* Enter 4-Byte Address Mode */
#define SPI_NOR_CMD_EXIT_4BA    0xE9    /* Exit 4-Byte Address Mode */
#define SPI_NOR_CMD_DPD         0xB9    /* Deep Power Down */
#define SPI_NOR_CMD_RDPD        0xAB    /* Release from Deep Power Down */
#define SPI_NOR_CMD_WR_CFGREG2  0x72    /* Write config register 2 */
#define SPI_NOR_CMD_RD_CFGREG2  0x71    /* Read config register 2 */
#define SPI_NOR_CMD_RESET_EN    0x66    /* Reset Enable */
#define SPI_NOR_CMD_RESET_MEM   0x99    /* Reset Memory */
#define SPI_NOR_CMD_BULKE       0x60    /* Bulk Erase */
#define SPI_NOR_CMD_READ_4B      0x13  /* Read data 4 Byte Address */
#define SPI_NOR_CMD_READ_FAST_4B 0x0C  /* Fast Read 4 Byte Address */
#define SPI_NOR_CMD_DREAD_4B     0x3C  /* Read data (1-1-2) 4 Byte Address */
#define SPI_NOR_CMD_2READ_4B     0xBC  /* Read data (1-2-2) 4 Byte Address */
#define SPI_NOR_CMD_QREAD_4B     0x6C  /* Read data (1-1-4) 4 Byte Address */
#define SPI_NOR_CMD_4READ_4B     0xEC  /* Read data (1-4-4) 4 Byte Address */
#define SPI_NOR_CMD_PP_4B        0x12  /* Page Program 4 Byte Address */
#define SPI_NOR_CMD_PP_1_1_4_4B  0x34  /* Quad Page program (1-1-4) 4 Byte Address */
#define SPI_NOR_CMD_PP_1_4_4_4B  0x3e  /* Quad Page program (1-4-4) 4 Byte Address */

#define SPI_NOR_CMD_RDSFDP          0x5A    /* Read SFDP */

/* Flash octal opcodes */
#define SPI_NOR_OCMD_SE         0x21DE  /* Octal Sector erase */
#define SPI_NOR_OCMD_CE         0xC738  /* Octal Chip erase */
#define SPI_NOR_OCMD_RDSR       0x05FA  /* Octal Read status register */
#define SPI_NOR_OCMD_DTR_RD     0xEE11  /* Octal IO DTR read command */
#define SPI_NOR_OCMD_RD         0xEC13  /* Octal IO read command */
#define SPI_NOR_OCMD_PAGE_PRG   0x12ED  /* Octal Page Prog */
#define SPI_NOR_OCMD_WREN       0x06F9  /* Octal Write enable */
#define SPI_NOR_OCMD_NOP        0x00FF  /* Octal No operation */
#define SPI_NOR_OCMD_RESET_EN   0x6699  /* Octal Reset Enable */
#define SPI_NOR_OCMD_RESET_MEM  0x9966  /* Octal Reset Memory */
#define SPI_NOR_OCMD_WR_CFGREG2 0x728D  /* Octal Write configuration Register2 */
#define SPI_NOR_OCMD_RD_CFGREG2 0x718E  /* Octal Read configuration Register2 */
#define SPI_NOR_OCMD_BULKE      0x609F  /* Octa Bulk Erase */

/* Page, sector, and block size are standard, not configurable. */
#define SPI_NOR_PAGE_SIZE    0x0100U
#define SPI_NOR_SECTOR_SIZE  0x1000U
#define SPI_NOR_BLOCK_SIZE   0x10000U

/* Flash Auto-polling values */
#define SPI_NOR_WREN_MATCH    0x02
#define SPI_NOR_WREN_MASK     0x02

#define SPI_NOR_WEL_MATCH     0x00
#define SPI_NOR_WEL_MASK      0x02

#define SPI_NOR_MEM_RDY_MATCH 0x00
#define SPI_NOR_MEM_RDY_MASK  0x01

#define SPI_NOR_AUTO_POLLING_INTERVAL   0x10

/* Flash Dummy Cycles values */
#define SPI_NOR_DUMMY_RD                8U
#define SPI_NOR_DUMMY_RD_OCTAL          6U
#define SPI_NOR_DUMMY_RD_OCTAL_DTR      6U
#define SPI_NOR_DUMMY_REG_OCTAL         4U
#define SPI_NOR_DUMMY_REG_OCTAL_DTR     5U


/* Memory registers address */
#define SPI_NOR_REG2_ADDR1              0x0000000
#define SPI_NOR_CR2_STR_OPI_EN          0x01
#define SPI_NOR_CR2_DTR_OPI_EN          0x02
#define SPI_NOR_REG2_ADDR3              0x00000300
#define SPI_NOR_CR2_DUMMY_CYCLES_66MHZ  0x07

/* Test whether offset is aligned to a given number of bits. */
#define SPI_NOR_IS_ALIGNED(_ofs, _bits) (((_ofs) & BIT_MASK(_bits)) == 0)
#define SPI_NOR_IS_SECTOR_ALIGNED(_ofs) SPI_NOR_IS_ALIGNED(_ofs, 12)

#define SPI_NOR_CMD_BUS_WIDTH(prot) ((prot) & 0xF)
#define SPI_NOR_ADDR_BUS_WIDTH(prot) (((prot) & 0xF0) >> 4)
#define SPI_NOR_DATA_BUS_WIDTH(prot) (((prot) & 0xF00) >> 8)

#define SPI_NOR_DUAL_CAP_MASK 0x00000F00
#define SPI_NOR_QUAD_CAP_MASK 0x000F0000

enum spi_nor_cap {
	SPI_NOR_MODE_1_1_1_CAP = 0x00000001,
	SPI_NOR_MODE_1_1_1_FAST_CAP = 0x00000002,
	SPI_NOR_MODE_1_1_2_CAP = 0x00000100,
	SPI_NOR_MODE_1_2_2_CAP = 0x00000200,
	SPI_NOR_MODE_2_2_2_CAP = 0x00000400,
	SPI_NOR_MODE_1_1_4_CAP = 0x00010000,
	SPI_NOR_MODE_1_4_4_CAP = 0x00020000,
	SPI_NOR_MODE_4_4_4_CAP = 0x00040000,
};

struct spi_nor_mode_cap {
	enum jesd216_mode_type mode;
	enum spi_nor_cap cap;
};

struct spi_nor_cmd_info {
	enum jesd216_mode_type read_mode;
	uint8_t read_opcode;
	uint8_t read_dummy;

	enum jesd216_mode_type pp_mode;
	uint8_t pp_opcode;

	enum jesd216_mode_type se_mode;
	uint8_t se_opcode;
};

#define SPI_NOR_OP_INFO(_mode_, _opcode_, _addr_, _addr_len_,	\
	_dummy_cycle_, _data_, _data_len_, _data_direct_)	\
	{	\
		.mode = _mode_,	\
		.opcode = _opcode_,	\
		.addr = _addr_,	\
		.addr_len = _addr_len_,	\
		.dummy_cycle = _dummy_cycle_,	\
		.buf = _data_,	\
		.data_len = _data_len_,	\
		.data_direct = _data_direct_,	\
	}

#define SPI_NOR_DATA_DIRECT_IN    0x00000001
#define SPI_NOR_DATA_DIRECT_OUT   0x00000002

struct spi_nor_op_info {
	enum jesd216_mode_type mode;
	uint8_t opcode;
	off_t addr;
	uint8_t addr_len;
	uint8_t dummy_cycle;
	void *buf;
	size_t data_len;
	uint32_t data_direct;
};

int spi_nor_config_4byte_mode(const struct device *dev, bool en4b);
int spi_nor_re_init(const struct device *dev);
int spi_nor_erase_by_cmd(const struct device *dev, off_t addr,
			 size_t size, uint8_t cmd);
int spi_nor_get_erase_sz(const struct device *dev, uint8_t cmd);
int spi_nor_rst_by_cmd(const struct device *dev);

#endif /*__SPI_NOR_H__*/
