/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <string.h>
#include <device.h>
#include <drivers/i3c/i3c.h>

/* external reference */
int i3c_aspeed_master_attach_device(const struct device *dev, struct i3c_device *slave);
int i3c_aspeed_master_deattach_device(const struct device *dev, struct i3c_device *slave);
int i3c_aspeed_master_send_ccc(const struct device *dev, struct i3c_ccc_cmd *ccc);
int i3c_aspeed_master_priv_xfer(struct i3c_device *i3cdev, struct i3c_priv_xfer *xfers, int nxfers);

/* functions for sending CCC */
static int i3c_send_rstdaa(const struct device *master)
{
	struct i3c_ccc_cmd ccc;

	/* RSTDAA CCC */
	ccc.addr = I3C_BROADCAST_ADDR;
	ccc.id = I3C_CCC_RSTDAA;
	ccc.payload.length = 0;
	ccc.payload.data = NULL;
	ccc.rnw = 0;
	ccc.ret = 0;

	return i3c_aspeed_master_send_ccc(master, &ccc);
}

static int i3c_send_setaasa(const struct device *master)
{
	struct i3c_ccc_cmd ccc;

	ccc.addr = I3C_BROADCAST_ADDR;
	ccc.id = I3C_CCC_SETAASA;
	ccc.payload.length = 0;
	ccc.payload.data = NULL;
	ccc.rnw = 0;
	ccc.ret = 0;

	return i3c_aspeed_master_send_ccc(master, &ccc);
}

/**
 * @brief data read for the JESD compliant devices
 * @param slave the JESD compliant device
 * @param add the address to be read
 * @param buf buffer to store the read data
 * @param length length of the read data
 * @return -1 if the slave device is not registered
 */
int i3c_jesd_read(struct i3c_device *slave, uint8_t addr, uint8_t *buf, int length)
{
	struct i3c_priv_xfer xfer[2];
	uint8_t mode_reg = addr;

	if (!slave->master_dev) {
		printk("unregistered device\n");
		return -1;
	}

	if (slave->info.i2c_mode) {
		printk("Not I3C device\n");
		return -2;
	}

	xfer[0].rnw = 0;
	xfer[0].len = 1;
	xfer[0].data.out = &mode_reg;

	xfer[1].rnw = 1;
	xfer[1].len = length;
	xfer[1].data.in = buf;

	return i3c_aspeed_master_priv_xfer(slave, xfer, 2);
}

int i3c_i2c_read(struct i3c_device *slave, uint8_t addr, uint8_t *buf, int length)
{
	struct i3c_priv_xfer xfer;
	uint8_t mode_reg = addr;
	int ret;

	if (!slave->master_dev) {
		printk("unregistered device\n");
		return -1;
	}

	if (!slave->info.i2c_mode) {
		printk("Not I2C device\n");
		return -2;
	}

	xfer.rnw = 0;
	xfer.len = 1;
	xfer.data.out = &mode_reg;
	ret = i3c_aspeed_master_priv_xfer(slave, &xfer, 1);
	if (ret) {
		return ret;
	}

	xfer.rnw = 1;
	xfer.len = length;
	xfer.data.in = buf;
	return i3c_aspeed_master_priv_xfer(slave, &xfer, 1);
}

void main(void)
{
	const struct device *master;
	struct i3c_device slave;
	int ret;
	uint8_t id[2];

	master = device_get_binding(DT_LABEL(DT_NODELABEL(i3c0)));
	if (!master) {
		printk("master device not found\n");
		goto test_fail;
	}

	/* Renesas IMX3102 */
	slave.info.static_addr = 0xf;
	slave.info.assigned_dynamic_addr = 0xf;
	slave.info.i2c_mode = 1;
	ret = i3c_aspeed_master_attach_device(master, &slave);
	if (ret) {
		printk("failed to attach slave\n");
		goto test_fail;
	}

	ret = i3c_send_rstdaa(master);
	if (ret) {
		printk("RSTDAA failed %d\n", ret);
		goto test_fail;
	}

	ret = i3c_i2c_read(&slave, 0, id, 2);
	if (ret) {
		printk("i2c xfer failed\n");
		goto test_fail;
	}
	printk("device ID in I2C mode %02x %02x\n", id[0], id[1]);

	i3c_aspeed_master_deattach_device(master, &slave);
	slave.info.i2c_mode = 0;
	ret = i3c_aspeed_master_attach_device(master, &slave);
	ret = i3c_send_setaasa(master);
	if (ret) {
		printk("SETAASA failed\n");
		goto test_fail;
	}
	/* read device ID */
	ret = i3c_jesd_read(&slave, 0, id, 2);
	if (ret) {
		printk("priv xfer failed\n");
		goto test_fail;
	}
	printk("device ID in I3C mode %02x %02x\n", id[0], id[1]);

test_fail:
	while (1) {
		k_sleep(K_MSEC(1000));
	}
}
