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
int i3c_aspeed_master_attach_device(const struct device *dev, struct i3c_dev_desc *slave);
int i3c_aspeed_master_deattach_device(const struct device *dev, struct i3c_dev_desc *slave);


int i3c_slave_mqueue_read(const struct device *dev, uint8_t *dest, int budget);
int i3c_slave_mqueue_write(const struct device *dev, uint8_t *src, int size);


#ifdef CONFIG_I3C_SAMPLE_IMX3102
void i3c_imx3112_test(void)
{
	const struct device *master;
	struct i3c_dev_desc slave;
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

	ret = i3c_master_send_rstdaa(master);
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

	ret = i3c_master_send_aasa(master);
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
static uint8_t ibi_data[CONFIG_I3C_ASPEED_MAX_IBI_PAYLOAD];
static struct i3c_ibi_payload i3c_payload;
static struct k_sem ibi_complete;
static struct i3c_ibi_payload *sample_ibi_write_requested(struct i3c_dev_desc *desc)
{
	i3c_payload.buf = ibi_data;
	i3c_payload.size = 0;

	return &i3c_payload;
}

static void sample_ibi_write_done(struct i3c_dev_desc *desc)
{
	int i;
	uint8_t *buf = (uint8_t *)i3c_payload.buf;

	for (i = 0; i < i3c_payload.size; i++) {
		if ((i & 0xf) == 0) {
			printk("\n");
		}
		printk("%02x ", buf[i]);
	}
	printk("\n");
	k_sem_give(&ibi_complete);
}

static struct i3c_ibi_callbacks i3c_ibi_def_callbacks = {
	.write_requested = sample_ibi_write_requested,
	.write_done = sample_ibi_write_done,
};

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
	struct i3c_dev_desc slave;
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
	ret = i3c_master_send_rstdaa(master);
	ret = i3c_master_send_rstdaa(master);
	if (ret) {
		printk("RSTDAA failed %d\n", ret);
		return;
	}

	ret = i3c_master_send_aasa(master);
	if (ret) {
		printk("SETAASA failed\n");
		return;
	}

	ret = i3c_master_send_getpid(master, slave.info.dynamic_addr, &slave.info.pid);
	printk("slave pid = %llx\n", slave.info.pid);

	printk("bus init done\n");

	for (i = 0; i < 16; i++) {
		data[i] = i;
	}

	xfer.rnw = 0;
	xfer.len = 4;
	for (i = 0; i < 16; i += 4) {
		xfer.data.out = &data[i];
		ret = i3c_master_priv_xfer(&slave, &xfer, 1);
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

	i3c_master_request_ibi(&slave, &i3c_ibi_def_callbacks);
	ret = i3c_master_enable_ibi(&slave);
	if (ret) {
		printk("failed to enable sir: %d\n", ret);
	}

	k_sem_init(&ibi_complete, 0, 1);
	ret = i3c_slave_mqueue_write(slave_mq, data, 16);
	k_sem_take(&ibi_complete, K_FOREVER);

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
