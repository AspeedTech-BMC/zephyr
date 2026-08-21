/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_ESPI_ASPEED_H_
#define ZEPHYR_INCLUDE_DRIVERS_ESPI_ASPEED_H_

/*
 * eSPI cycle type encoding
 *
 * Section 5.1 Cycle Types and Packet Format,
 * Intel eSPI Interface Base Specification, Rev 1.0, Jan. 2016.
 */
#define ESPI_PERIF_MEMRD32		0x00
#define ESPI_PERIF_MEMRD64		0x02
#define ESPI_PERIF_MEMWR32		0x01
#define ESPI_PERIF_MEMWR64		0x03
#define ESPI_PERIF_IORD			0x04
#define ESPI_PERIF_IOWR			0x05
#define ESPI_PERIF_MSG			0x10
#define ESPI_PERIF_MSG_D		0x11
#define ESPI_PERIF_SUC_CMPLT		0x06
#define ESPI_PERIF_SUC_CMPLT_D_MIDDLE	0x09
#define ESPI_PERIF_SUC_CMPLT_D_FIRST	0x0b
#define ESPI_PERIF_SUC_CMPLT_D_LAST	0x0d
#define ESPI_PERIF_SUC_CMPLT_D_ONLY	0x0f
#define ESPI_PERIF_UNSUC_CMPLT		0x0c
#define ESPI_OOB_MSG			0x21
#define ESPI_FLASH_READ			0x00
#define ESPI_FLASH_WRITE		0x01
#define ESPI_FLASH_ERASE		0x02
#define ESPI_FLASH_SUC_CMPLT		0x06
#define ESPI_FLASH_SUC_CMPLT_D_MIDDLE	0x09
#define ESPI_FLASH_SUC_CMPLT_D_FIRST	0x0b
#define ESPI_FLASH_SUC_CMPLT_D_LAST	0x0d
#define ESPI_FLASH_SUC_CMPLT_D_ONLY	0x0f
#define ESPI_FLASH_UNSUC_CMPLT		0x0c

/*
 * eSPI packet format structure
 *
 * Section 5.1 Cycle Types and Packet Format,
 * Intel eSPI Interface Base Specification, Rev 1.0, Jan. 2016.
 */
struct espi_comm_hdr {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
} __packed;

struct espi_perif_mem32 {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint32_t addr_be;
	uint8_t data[];
} __packed;

struct espi_perif_mem64 {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint32_t addr_be;
	uint8_t data[];
} __packed;

struct espi_perif_io {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint16_t addr_be;
	uint8_t data[];
} __packed;

struct espi_perif_msg {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint8_t msg_code;
	uint8_t msg_byte[4];
	uint8_t data[];
} __packed;

struct espi_perif_cmplt {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint8_t data[];
} __packed;

struct espi_oob_msg {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint8_t data[];
} __packed;

struct espi_flash_rwe {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint32_t addr_be;
	uint8_t data[];
} __packed;

struct espi_flash_cmplt {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
	uint8_t data[];
} __packed;

struct espi_aspeed_ioc {
	uint32_t pkt_len;
	uint8_t *pkt;
};

#define ESPI_PLD_LEN_MAX	BIT(12)

int espi_aspeed_perif_pc_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc,
				bool blocking);
int espi_aspeed_perif_pc_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc);
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
int espi_aspeed_perif_np_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc,
				bool blocking);
#endif
int espi_aspeed_perif_np_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc);
int espi_aspeed_oob_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc, bool blocking);
int espi_aspeed_oob_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc);
int espi_aspeed_flash_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc, bool blocking);
int espi_aspeed_flash_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc);

#ifdef CONFIG_ESPI_TAF
/*
 * eSPI TAF (Target Attached Flash) request notification
 *
 * The driver parses the host's flash read/write/erase request and hands it
 * to the registered handler. The handler owns the flash backend: it is
 * responsible for performing the access and, for ESPI_FLASH_READ, building
 * and sending the completion packet itself via espi_aspeed_flash_put_tx().
 */
struct espi_taf_req {
	uint8_t cyc;	/* ESPI_FLASH_READ / ESPI_FLASH_WRITE / ESPI_FLASH_ERASE */
	uint8_t tag;
	uint32_t addr;
	uint16_t len;
	uint8_t *data;	/* valid for ESPI_FLASH_WRITE only, NULL otherwise */
};

typedef void (*espi_taf_handler_t)(const struct device *dev,
				       struct espi_taf_req *req,
				       void *user_data);

int espi_aspeed_taf_register(const struct device *dev,
			      espi_taf_handler_t handler,
			      void *user_data);
#endif /* CONFIG_ESPI_TAF */

#endif
