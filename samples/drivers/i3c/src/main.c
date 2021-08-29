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

int i3c_slave_mqueue_read(const struct device *dev, uint8_t *dest, int budget);

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

#ifdef CONFIG_I3C_SAMPLE_IMX3102
void i3c_imx3112_test(void)
{
	const struct device *master;
	struct i3c_device slave;
	int ret;
	uint8_t id[2];

	master = device_get_binding(DT_LABEL(DT_NODELABEL(i3c0)));
	if (!master) {
		printk("master device not found\n");
		return;
	}

	/* Renesas IMX3102 */
	slave.info.static_addr = 0xf;
	slave.info.assigned_dynamic_addr = 0xf;
	slave.info.i2c_mode = 1;
	ret = i3c_aspeed_master_attach_device(master, &slave);
	if (ret) {
		printk("failed to attach i2c slave\n");
		return;
	}

	ret = i3c_send_rstdaa(master);
	if (ret) {
		printk("RSTDAA failed %d\n", ret);
		return;
	}

	ret = i3c_i2c_read(&slave, 0, id, 2);
	if (ret) {
		printk("i2c xfer failed\n");
		return;
	}
	printk("device ID in I2C mode %02x %02x\n", id[0], id[1]);

	i3c_aspeed_master_deattach_device(master, &slave);
	slave.info.i2c_mode = 0;
	ret = i3c_aspeed_master_attach_device(master, &slave);
	if (ret) {
		printk("failed to attach i3c slave\n");
		return;
	}

	ret = i3c_send_setaasa(master);
	if (ret) {
		printk("SETAASA failed\n");
		return;
	}
	/* read device ID */
	ret = i3c_jesd_read(&slave, 0, id, 2);
	if (ret) {
		printk("priv xfer failed %d\n", ret);
		return;
	}
	printk("device ID in I3C mode %02x %02x\n", id[0], id[1]);
}
#endif

#ifdef CONFIG_I3C_SAMPLE_LOOPBACK
/**
 * @brief i3c0-to-i3c1 loopback test
 *
 * i3c0 plays the role as the main master and i3c1 plays the role as the slave device.
 * Use SETAASA to assign the dynamic address.
 *
 */
void i3c_loopback_test(void)
{
	const struct device *master, *slave_mq;
	struct i3c_device slave;
	struct i3c_priv_xfer xfer;
	int ret, i;
	uint8_t data[16], result[16];

	master = device_get_binding(DT_LABEL(DT_NODELABEL(i3c0)));
	if (!master) {
		printk("master device not found\n");
		return;
	}

	slave_mq = device_get_binding(DT_LABEL(DT_NODELABEL(i3c1_smq)));
	if (!slave_mq) {
		printk("slave-mq device not found\n");
		return;
	}

	slave.info.static_addr = DT_PROP(DT_NODELABEL(i3c1), assigned_address);
	slave.info.assigned_dynamic_addr = slave.info.static_addr;
	slave.info.i2c_mode = 0;
	ret = i3c_aspeed_master_attach_device(master, &slave);
	if (ret) {
		printk("failed to attach slave\n");
		return;
	}

	/*
	 * ASPEED IOP fix:
	 * Aspeed slave device needs for a dummy CCC to activate the slave mode.
	 * The slave device will NACK the dummy CCC so just ignore the first response in the master
	 * side.
	 */
	ret = i3c_send_rstdaa(master);
	ret = i3c_send_rstdaa(master);
	if (ret) {
		printk("RSTDAA failed %d\n", ret);
		return;
	}

	ret = i3c_send_setaasa(master);
	if (ret) {
		printk("SETAASA failed\n");
		return;
	}
	printk("bus init done\n");

	for (i = 0; i < 16; i++) {
		data[i] = i;
	}

	xfer.rnw = 0;
	xfer.len = 4;
	for (i = 0; i < 16; i += 4) {
		xfer.data.out = &data[i];
		ret = i3c_aspeed_master_priv_xfer(&slave, &xfer, 1);
		if (ret) {
			printk("priv wr fail\n");
			return;
		}
	}

	for (i = 0; i < 16; i += 4) {
		ret = i3c_slave_mqueue_read(slave_mq, &result[i], 4);
	}

	for (i = 0; i < 16; i++) {
		if (result[i] != data[i]) {
			printk("in: %d out: %d\n", data[i], result[i]);
			return;
		}
	}

	printk("loopback test pass\n");
}
#endif

void main(void)
{
#ifdef CONFIG_I3C_SAMPLE_IMX3102
	i3c_imx3112_test();
#endif

#ifdef CONFIG_I3C_SAMPLE_LOOPBACK
	i3c_loopback_test();
#endif
	while (1) {
		k_sleep(K_MSEC(1000));
	}
}
