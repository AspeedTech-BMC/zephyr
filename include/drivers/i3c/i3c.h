/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>

#define I3C_HOT_JOIN_ADDR       0x02
#define I3C_BROADCAST_ADDR      0x7e
#define I3C_MAX_ADDR            0x7f

#define I3C_CCC_DIRECT          0x80

/* broadcast / unicast commands */
#define I3C_CCC_RSTDAA          0x06
#define I3C_CCC_SETMWL          0x09
#define I3C_CCC_SETMRL          0x0a

/* broadcast only commands */
#define I3C_CCC_ENTDAA          0x07
#define I3C_CCC_SETAASA         0x29

/* unicast only commands */
#define I3C_CCC_SETDASA         (0x7 | I3C_CCC_DIRECT)
#define I3C_CCC_GETMWL          (0xb | I3C_CCC_DIRECT)
#define I3C_CCC_GETMRL          (0xc | I3C_CCC_DIRECT)
#define I3C_CCC_GETPID          (0xd | I3C_CCC_DIRECT)
#define I3C_CCC_GETBCR          (0xe | I3C_CCC_DIRECT)
#define I3C_CCC_GETDCR          (0xf | I3C_CCC_DIRECT)
#define I3C_CCC_GETSTATUS       (0x10 | I3C_CCC_DIRECT)

/**
 * @brief I3C Common-Command-Codes (CCC) structure
 * @param rnw 1'b0 = write command, 1'b1 = read command
 * @param id command code
 * @param addr address of slave device
 * @param ret return value
 * @param payload length: length of the payload.  data: pointer to the payload
 */
struct i3c_ccc_cmd {
	uint8_t rnw;
	uint8_t id;
	uint8_t addr;
	uint8_t ret;
	struct {
		uint16_t length;
		void *data;
	} payload;
};

/**
 * @brief I3C private transfer structure
 * @param data pointer to the read/write data
 * @param len length of the data
 * @param rnw 1'b0 = write command, 1'b1 = read command
 */
struct i3c_priv_xfer {
	union {
		void *in;
		void *out;
	} data;
	int len;
	int rnw;
};

/* slave device structure */
struct i3c_device_info {
	uint64_t pid;
	uint8_t dcr;
	uint8_t bcr;
	uint8_t static_addr;
	uint8_t assigned_dynamic_addr;
	uint8_t dynamic_addr;
	uint32_t i2c_mode;
};

struct i3c_device {
	const struct device *master_dev;
	void *driver_data;
	struct i3c_device_info info;
};

/* slave driver structure */
struct i3c_slave_payload {
	int size;
	void *buf;
};

struct i3c_slave_callbacks {
	struct i3c_slave_payload* (*write_requested)(const struct device *dev);
	void (*write_done)(const struct device *dev);
};

struct i3c_slave_setup {
	int max_payload_len;
	const struct device *dev;
	const struct i3c_slave_callbacks *callbacks;
};

/* ASPEED API */
int i3c_aspeed_slave_register(const struct device *dev, struct i3c_slave_setup *slave_data);
