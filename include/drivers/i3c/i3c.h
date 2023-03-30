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
#define I3C_CCC_ENEC            0x00
#define I3C_CCC_DISEC           0x01
#define I3C_CCC_RSTDAA          0x06
#define I3C_CCC_SETMWL          0x09
#define I3C_CCC_SETMRL          0x0a

/* broadcast only commands */
#define I3C_CCC_ENTDAA          0x07
#define I3C_CCC_SETAASA         0x29
#define I3C_CCC_SETHID		0x61
#define I3C_CCC_DEVCTRL		0x62

/* unicast only commands */
#define I3C_CCC_SETDASA         (0x7 | I3C_CCC_DIRECT)
#define I3C_CCC_GETMWL          (0xb | I3C_CCC_DIRECT)
#define I3C_CCC_GETMRL          (0xc | I3C_CCC_DIRECT)
#define I3C_CCC_GETPID          (0xd | I3C_CCC_DIRECT)
#define I3C_CCC_GETBCR          (0xe | I3C_CCC_DIRECT)
#define I3C_CCC_GETDCR          (0xf | I3C_CCC_DIRECT)
#define I3C_CCC_GETSTATUS       (0x10 | I3C_CCC_DIRECT)

/* event id */
#define I3C_CCC_EVT_SIR         0x1
#define I3C_CCC_EVT_MR          0x2
#define I3C_CCC_EVT_HJ          0x8

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

#define I3C_PID_VENDOR_ID(x)            ((x) >> 33)
#define I3C_PID_VENDOR_ID_ASPEED        0x03f6

#define I3C_BCR_IBI_PAYLOAD		BIT(2)

/**
 * @brief descriptor of the i3c device attached to the bus
 * @param bus the bus controller which hosts the bus
 * @param priv_data pointer to the low level driver private data
 * @param info the device information
 */
struct i3c_dev_desc {
	const struct device *bus;
	struct i3c_device_info info;
	void *priv_data;
};

/* MIPI I3C MDB definition: see https://www.mipi.org/MIPI_I3C_mandatory_data_byte_values_public */
#define IBI_MDB_ID(grp, id)		((((grp) << 5) & GENMASK(7, 5)) | ((id) & GENMASK(4, 0)))
#define IBI_MDB_GET_GRP(m)		(((m) & GENMASK(7, 5)) >> 5)
#define IBI_MDB_GET_ID(m)		((m) & GENMASK(4, 0))

#define IBI_MDB_GRP_PENDING_READ_NOTIF	0x5
#define IS_MDB_PENDING_READ_NOTIFY(m)	(IBI_MDB_GET_GRP(m) == IBI_MDB_GRP_PENDING_READ_NOTIF)
#define IBI_MDB_MIPI_DBGDATAREADY	IBI_MDB_ID(IBI_MDB_GRP_PENDING_READ_NOTIF, 0xd)
#define IBI_MDB_MCTP			IBI_MDB_ID(IBI_MDB_GRP_PENDING_READ_NOTIF, 0xe)
/* Interrupt ID 0x10 to 0x1F are for vendor specific */
#define IBI_MDB_ASPEED			IBI_MDB_ID(IBI_MDB_GRP_PENDING_READ_NOTIF, 0x1f)

struct i3c_ibi_payload {
	int max_payload_size;
	int size;
	uint8_t *buf;
};

/**
 * @brief IBI callback function structure
 * @param write_requested callback function to return a memory block for receiving IBI data
 * @param write_done callback function to process the received IBI data
 */
struct i3c_ibi_callbacks {
	struct i3c_ibi_payload* (*write_requested)(struct i3c_dev_desc *i3cdev);
	void (*write_done)(struct i3c_dev_desc *i3cdev);
};

/* slave driver structure */
struct i3c_slave_payload {
	int size;
	void *buf;
};

/* slave events */
#define I3C_SLAVE_EVENT_SIR		BIT(0)
#define I3C_SLAVE_EVENT_MR		BIT(1)
#define I3C_SLAVE_EVENT_HJ		BIT(2)

/**
 * @brief slave callback function structure
 * @param write_requested callback function to return a memory block for receiving data sent from
 *                        the master device
 * @param write_done callback function to process the received IBI data
 */
struct i3c_slave_callbacks {
	struct i3c_slave_payload* (*write_requested)(const struct device *dev);
	void (*write_done)(const struct device *dev);
};

struct i3c_slave_setup {
	int max_payload_len;
	const struct device *dev;
	const struct i3c_slave_callbacks *callbacks;
};

/* Aspeed HAL API */
int i3c_aspeed_master_attach_device(const struct device *dev, struct i3c_dev_desc *slave);
int i3c_aspeed_master_detach_device(const struct device *dev, struct i3c_dev_desc *slave);
int i3c_aspeed_master_send_ccc(const struct device *dev, struct i3c_ccc_cmd *ccc);
int i3c_aspeed_master_priv_xfer(struct i3c_dev_desc *i3cdev, struct i3c_priv_xfer *xfers,
				int nxfers);
int i3c_aspeed_master_request_ibi(struct i3c_dev_desc *i3cdev, struct i3c_ibi_callbacks *cb);
int i3c_aspeed_master_enable_ibi(struct i3c_dev_desc *i3cdev);
int i3c_aspeed_slave_register(const struct device *dev, struct i3c_slave_setup *slave_data);

/**
 * @brief get the assigned dynamic address of the i3c controller
 * @param dev the I3C controller in slave mode
 * @param dynamic_addr pointer to the dynamic address variable
 * @return -1 if dynamic address is not assigned
 * @return 0 if dynamic address is assigned.  The value is passed to `dynamic_addr`
 */
int i3c_aspeed_slave_get_dynamic_addr(const struct device *dev, uint8_t *dynamic_addr);

/**
 * @brief get the event enabling status
 * @param dev the I3C controller in slave mode
 * @param event_en pointer to the event enabling mask, see `I3C_SLAVE_EVENT_*`.
 * @return 0 if success
 *
 * This function gets the status of the event enabling from the slave controller.
 * The bits set in `event_en` means the corresponding slave events are enabled.
 */
int i3c_aspeed_slave_get_event_enabling(const struct device *dev, uint32_t *event_en);

/**
 * @brief slave device sends SIR (IBI) with data
 *
 * @param dev the I3C controller in slave mode
 * @param payload pointer to IBI payload structure
 * @return int 0 = success
 */
int i3c_aspeed_slave_send_sir(const struct device *dev, struct i3c_ibi_payload *payload);

/**
 * @brief slave device prepares the data for master private read transfer
 *
 * @param dev the I3C controller in slave mode
 * @param data pointer to the data structure to be read
 * @param ibi_notify pointer to the IBI notification structure (optional)
 * @return int 0 = success
 *
 * This function puts the pending read data to the TX FIFO and waits until the
 * pending read data is consumed.  The API uses osEventFlagsWait and will make
 * the caller thread sleep so do not call it in the ISR.
 * If @ibi_notify is specified, a slave interrupt with the IBI payload will be
 * issued to notify the master device that there is a pending read data.  The
 * master device shall issue a private read transfer to read the data back.
 */
int i3c_aspeed_slave_put_read_data(const struct device *dev, struct i3c_slave_payload *data,
				   struct i3c_ibi_payload *ibi_notify);

/* common API */
int i3c_master_send_enec(const struct device *master, uint8_t addr, uint8_t evt);
int i3c_master_send_disec(const struct device *master, uint8_t addr, uint8_t evt);
int i3c_master_send_rstdaa(const struct device *master);
int i3c_master_send_sethid(const struct device *master);
int i3c_master_send_aasa(const struct device *master);
int i3c_master_send_setmrl(const struct device *master, uint8_t addr, uint16_t mrl,
			   uint8_t ibi_payload_size);
int i3c_master_send_getpid(const struct device *master, uint8_t addr, uint64_t *pid);
int i3c_master_send_getbcr(const struct device *master, uint8_t addr, uint8_t *bcr);

#define i3c_master_attach_device	i3c_aspeed_master_attach_device
#define i3c_master_detach_device	i3c_aspeed_master_detach_device
#define i3c_master_send_ccc		i3c_aspeed_master_send_ccc
#define i3c_master_priv_xfer		i3c_aspeed_master_priv_xfer
#define i3c_master_request_ibi		i3c_aspeed_master_request_ibi
#define i3c_master_enable_ibi		i3c_aspeed_master_enable_ibi
#define i3c_slave_register		i3c_aspeed_slave_register
#define i3c_slave_send_sir		i3c_aspeed_slave_send_sir
#define i3c_slave_put_read_data		i3c_aspeed_slave_put_read_data
#define i3c_slave_get_dynamic_addr	i3c_aspeed_slave_get_dynamic_addr
#define i3c_slave_get_event_enabling	i3c_aspeed_slave_get_event_enabling

int i3c_jesd403_read(struct i3c_dev_desc *slave, uint8_t *addr, int addr_size, uint8_t *data,
		     int data_size);
int i3c_jesd403_write(struct i3c_dev_desc *slave, uint8_t *addr, int addr_size, uint8_t *data,
		      int data_size);
int i3c_i2c_read(struct i3c_dev_desc *slave, uint8_t addr, uint8_t *buf, int length);
int i3c_i2c_write(struct i3c_dev_desc *slave, uint8_t addr, uint8_t *buf, int length);
