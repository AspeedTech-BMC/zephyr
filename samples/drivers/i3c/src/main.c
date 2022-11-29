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
int i3c_slave_mqueue_read(const struct device *dev, uint8_t *dest, int budget);
int i3c_slave_mqueue_write(const struct device *dev, uint8_t *src, int size);

#ifdef CONFIG_I3C_SAMPLE_IMX3112
/*
 * IMX3112: 1-to-2 multiplexier
 *
 *                                        SPD@50  SPD@51
 * +----------------- +                   |       |
 * | SoC              |              /----+-------+---...
 * |                  |   +---------+   child bus 0, select IMX3112 local port 0
 * | I3C controller - | --| IMX3112 |
 * |                  |   +---------+   child bus 1, select IMX3112 local port 1
 * +------------------+              \----+-------+---...
 *                                        |       |
 *                                        SPD@52  SPD@53
 */
void i3c_imx3112_reg_read(struct i3c_dev_desc *imx3112, uint8_t addr, uint8_t *data, int data_size);
void i3c_imx3112_reg_write(struct i3c_dev_desc *imx3112, uint8_t addr, uint8_t *data,
			   int data_size);
void i3c_imx3112_select(struct i3c_dev_desc *imx3112, int chan);
void i3c_spd5118_reg_read(struct i3c_dev_desc *spd5118, uint8_t addr, uint8_t *data, int data_size);

static void i3c_imx3112_test(void)
{
	const struct device *master;
	struct i3c_dev_desc slave[5];
	int ret, i;
	uint8_t data[2];

	master = device_get_binding(DT_LABEL(DT_NODELABEL(i3c2)));
	__ASSERT(master, "master device not found\n");

	slave[0].info.static_addr = 0x70;
	slave[1].info.static_addr = 0x50;
	slave[2].info.static_addr = 0x51;
	slave[3].info.static_addr = 0x52;
	slave[4].info.static_addr = 0x53;

	for (i = 0; i < 5; i++) {
		slave[i].info.i2c_mode = 1;
		slave[i].info.assigned_dynamic_addr = slave[i].info.static_addr;
		ret = i3c_master_attach_device(master, &slave[i]);
		__ASSERT(!ret, "failed to attach i2c slave[%d]\n", i);
	}

	/* errata: must write 1 to MR46[0] before accessing */
	data[0] = 0x1;
	ret = i3c_i2c_write(&slave[0], 0x46, data, 1);
	__ASSERT_NO_MSG(!ret);

	ret = i3c_master_send_rstdaa(master);
	__ASSERT_NO_MSG(!ret);

	/* select child bus 0 */
	i3c_imx3112_select(&slave[0], 0);
	for (i = 0; i < 3; i++) {
		ret = i3c_i2c_read(&slave[i], 0, data, 2);
		__ASSERT_NO_MSG(!ret);
		printk("device%d ID in I2C mode %02x %02x\n", i, data[0], data[1]);
	}

	/* select child bus 1 */
	i3c_imx3112_select(&slave[0], 1);

	for (i = 3; i < 5; i++) {
		ret = i3c_i2c_read(&slave[i], 0, data, 2);
		__ASSERT_NO_MSG(!ret);
		printk("device%d ID in I2C mode %02x %02x\n", i, data[0], data[1]);
	}

	/* select child bus 0 */
	i3c_imx3112_select(&slave[0], 0);

	/* bring the bus to I3C mode */
	for (i = 0; i < 5; i++) {
		i3c_master_detach_device(master, &slave[i]);
		slave[i].info.i2c_mode = 0;
		ret = i3c_master_attach_device(master, &slave[i]);
		if (ret) {
			printk("failed to attach i3c slave[%d]\n", i);
			return;
		}
	}

	ret = i3c_master_send_sethid(master);
	__ASSERT(!ret, "SETHID failed\n");
	ret = i3c_master_send_aasa(master);
	__ASSERT(!ret, "SETAASA failed\n");

	for (i = 0; i < 3; i++) {
		i3c_spd5118_reg_read(&slave[i], 0, data, 2);
		printk("device%d ID in I3C mode %02x %02x\n", i, data[0], data[1]);
	}

	/* select child bus 1 */
	i3c_imx3112_select(&slave[0], 1);

	for (i = 3; i < 5; i++) {
		i3c_spd5118_reg_read(&slave[i], 0, data, 2);
		printk("device%d ID in I3C mode %02x %02x\n", i, data[0], data[1]);
	}
}
#endif

#ifdef CONFIG_I3C_SAMPLE_IMX3102
/*
 * IMX3102: 2-to-1 multiplexier
 *
 * +------------------   +
 * | SoC                 |
 * |                     |
 * | I3C controller #0 - | --+
 * |                     |    \                  SPD@51
 * |                     |     +---------+       |
 * |                     |     | IMX3102 | ---+--+---- i3c bus
 * |                     |     +---------+    |
 * |                     |    /               SPD@50
 * | I3C controller #1 - | --+
 * |                     |
 * +---------------------+
 */
void i3c_imx3102_reg_read(struct i3c_dev_desc *imx3102, uint8_t addr, uint8_t *data, int data_size);
void i3c_imx3102_reg_write(struct i3c_dev_desc *imx3102, uint8_t addr, uint8_t *data,
			   int data_size);
void i3c_imx3102_release_ownership(struct i3c_dev_desc *imx3102);
void i3c_imx3102_init(struct i3c_dev_desc *imx3102);
void i3c_spd5118_reg_read(struct i3c_dev_desc *spd5118, uint8_t addr, uint8_t *data, int data_size);

static void i3c_imx3102_test(void)
{
	const struct device *master[2];
	struct i3c_dev_desc slave[2][3];
	int ret, i, j;
	uint8_t data[2];

	master[0] = device_get_binding(DT_LABEL(DT_NODELABEL(i3c0)));
	__ASSERT(master[0], "master device not found\n");
	master[1] = device_get_binding(DT_LABEL(DT_NODELABEL(i3c1)));
	__ASSERT(master[1], "master device not found\n");

	/* init descriptors to the slave devices */
	for (i = 0; i < 2; i++) {
		slave[i][0].info.static_addr = 0xf;
		slave[i][1].info.static_addr = 0x50;
		slave[i][2].info.static_addr = 0x51;
		for (j = 0; j < 3; j++) {
			slave[i][j].info.i2c_mode = 1;
			slave[i][j].info.assigned_dynamic_addr = slave[i][j].info.static_addr;
			ret = i3c_master_attach_device(master[i], &slave[i][j]);
			__ASSERT(!ret, "failed to attach i2c slave%d to master%d\n", j, i);
		}
	}

	ret = i3c_master_send_rstdaa(master[0]);
	__ASSERT(!ret, "RSTDAA failed %d\n", ret);

	/* init the local port */
	i3c_imx3102_init(&slave[0][0]);

	for (j = 0; j < 3; j++) {
		ret = i3c_i2c_read(&slave[0][j], 0, data, 2);
		__ASSERT(!ret, "i2c xfer failed\n");
		printk("[I3C0][I2C mode] device%d ID %02x %02x\n", j, data[0], data[1]);
	}

	i3c_imx3102_release_ownership(&slave[0][0]);

	for (j = 0; j < 3; j++) {
		ret = i3c_i2c_read(&slave[1][j], 0, data, 2);
		__ASSERT(!ret, "i2c xfer failed\n");
		printk("[I3C1][I2C mode] device%d ID %02x %02x\n", j, data[0], data[1]);
	}

	for (i = 0; i < 2; i++) {
		for (j = 0; j < 3; j++) {
			i3c_master_detach_device(master[i], &slave[i][j]);
			slave[i][j].info.i2c_mode = 0;
			ret = i3c_master_attach_device(master[i], &slave[i][j]);
			__ASSERT(!ret, "failed to attach i3c slave%d to master%d\n", j, i);
		}
	}

	ret = i3c_master_send_sethid(master[1]);
	__ASSERT_NO_MSG(!ret);
	ret = i3c_master_send_aasa(master[1]);
	__ASSERT_NO_MSG(!ret);

	i3c_imx3102_reg_read(&slave[1][0], 0, data, 2);
	printk("[I3C1][I3C mode] device0 ID %02x %02x\n", data[0], data[1]);
	for (j = 1; j < 3; j++) {
		i3c_spd5118_reg_read(&slave[1][j], 0, data, 2);
		printk("[I3C1][I3C mode] device%d ID %02x %02x\n", j, data[0], data[1]);
	}

	i3c_imx3102_release_ownership(&slave[1][0]);

	i3c_imx3102_reg_read(&slave[0][0], 0, data, 2);
	printk("[I3C0][I3C mode] device0 ID %02x %02x\n", data[0], data[1]);
	for (j = 1; j < 3; j++) {
		i3c_spd5118_reg_read(&slave[0][j], 0, data, 2);
		printk("[I3C0][I3C mode] device%d ID %02x %02x\n", j, data[0], data[1]);
	}
}
#endif

void main(void)
{
#ifdef CONFIG_I3C_SAMPLE_IMX3102
	i3c_imx3102_test();
#endif

#ifdef CONFIG_I3C_SAMPLE_IMX3112
	i3c_imx3112_test();
#endif

	while (1) {
		k_sleep(K_MSEC(1000));
	}
}
