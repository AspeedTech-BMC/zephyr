/*
 * Copyright (c) 2018 Prevas A/S
 * Copyright (c) 2022 ASPEED Technology Inc.
 *
 * Implement common and specified i2c usage for aspeed chips.
 *
 * Common i2c : ast2600 / ast1030 / ast1060
 * PFR i2c : ast1060
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/target/eeprom.h>
#include <zephyr/drivers/i2c/target/ipmb.h>

#include <string.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(iic_shell, CONFIG_LOG_DEFAULT_LEVEL);

#define I2C_DEVICE_PREFIX "I2C_"

/* Maximum bytes we can write or read at once */
#define MAX_I2C_BYTES	16

/*
 * This sends I2C messages without any data (i.e. stop condition after
 * sending just the address). If there is an ACK for the address, it
 * is assumed there is a device present.
 *
 * WARNING: As there is no standard I2C detection command, this code
 * uses arbitrary SMBus commands (namely SMBus quick write and SMBus
 * receive byte) to probe for devices.  This operation can confuse
 * your I2C bus, cause data loss, and is known to corrupt the Atmel
 * AT24RF08 EEPROM found on many IBM Thinkpad laptops.
 *
 * https://manpages.debian.org/buster/i2c-tools/i2cdetect.8.en.html
 */
static int cmd_i2c_scan(const struct shell *shell,
			size_t argc, char **argv)
{
	const struct device *dev;
	uint8_t cnt = 0, first = 0x04, last = 0x77;

	dev = device_get_binding(argv[1]);

	if (!dev) {
		shell_error(shell, "I2C: Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	shell_print(shell,
		    "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
	for (uint8_t i = 0; i <= last; i += 16) {
		shell_fprintf(shell, SHELL_NORMAL, "%02x: ", i);
		for (uint8_t j = 0; j < 16; j++) {
			if (i + j < first || i + j > last) {
				shell_fprintf(shell, SHELL_NORMAL, "   ");
				continue;
			}

			struct i2c_msg msgs[1];
			uint8_t dst;

			/* Send the address to read from */
			msgs[0].buf = &dst;
			msgs[0].len = 0U;
			msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
			if (i2c_transfer(dev, &msgs[0], 1, i + j) == 0) {
				shell_fprintf(shell, SHELL_NORMAL,
					      "%02x ", i + j);
				++cnt;
			} else {
				shell_fprintf(shell, SHELL_NORMAL, "-- ");
			}
		}
		shell_print(shell, "");
	}

	shell_print(shell, "%u devices found on %s",
		    cnt, argv[1]);

	return 0;
}

static int cmd_i2c_recover(const struct shell *shell,
			   size_t argc, char **argv)
{
	const struct device *dev;
	int err;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I2C: Device driver %s not found.", argv[1]);
		return -ENODEV;
	}

	err = i2c_recover_bus(dev);
	if (err) {
		shell_error(shell, "I2C: Bus recovery failed (err %d)", err);
		return err;
	}

	return 0;
}

/* i2c write <device> <dev_addr> [<byte1>, ...] */
static int cmd_i2c_write(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t buf[MAX_I2C_BYTES];
	const struct device *dev;
	int num_bytes;
	int reg_addr;
	int dev_addr;
	int i;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I2C: Device driver not found.");
		return -ENODEV;
	}

	dev_addr = strtol(argv[2], NULL, 16);
	reg_addr = strtol(argv[3], NULL, 16);
	num_bytes = argc - 4;
	if (num_bytes < 0) {
		return 0;
	}
	if (num_bytes > MAX_I2C_BYTES) {
		num_bytes = MAX_I2C_BYTES;
	}
	for (i = 0; i < num_bytes; i++) {
		buf[i] = (uint8_t)strtol(argv[4 + i], NULL, 16);
	}

	if (i2c_burst_write(dev, dev_addr, reg_addr, buf, num_bytes) < 0) {
		shell_error(shell, "Failed to write to device: %s", argv[1]);
		return -EIO;
	}

	return 0;
}

/* i2c write_bytes <device> <dev_addr> [<numbytes>] */
static int cmd_i2c_write_multi_bytes(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *dev;
	int num_bytes;
	int reg_addr;
	int dev_addr;
	int i;
	uint8_t tx_buf[2];

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I2C: Device driver %s not found.", argv[1]);
		return -ENODEV;
	}

	dev_addr = strtol(argv[2], NULL, 16);
	reg_addr = strtol(argv[3], NULL, 16);
	tx_buf[0] = (uint8_t)reg_addr;
	num_bytes = argc - 4;

	if (num_bytes < 0) {
		return 0;
	}

	if (num_bytes > MAX_I2C_BYTES) {
		num_bytes = MAX_I2C_BYTES;
	}

	for (i = 0; i < num_bytes; i++) {
		tx_buf[1] = (uint8_t)strtol(argv[4 + i], NULL, 16);
		if (i2c_write(dev, tx_buf, 2, dev_addr) < 0) {
			shell_error(shell, "Failed to mb write from device: %s", argv[1]);
			return -EIO;
		}
		tx_buf[0]++;
		k_msleep(8);
	}

	return 0;
}

static int cmd_i2c_write_byte(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *dev;
	int reg_addr;
	int dev_addr;
	int out_byte;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I2C: Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	dev_addr = strtol(argv[2], NULL, 16);
	reg_addr = strtol(argv[3], NULL, 16);
	out_byte = strtol(argv[4], NULL, 16);

	if (i2c_reg_write_byte(dev, dev_addr, reg_addr, out_byte) < 0) {
		shell_error(shell, "Failed to write to device: %s", argv[1]);
		return -EIO;
	}

	return 0;
}

static int cmd_i2c_write_bytes(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *dev;
	uint8_t buf[MAX_I2C_BYTES];
	int num_bytes;
	int dev_addr;
	int i;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I2C: Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	dev_addr = strtol(argv[2], NULL, 16);
	num_bytes = argc - 3;
	if (num_bytes < 0) {
		return 0;
	}
	if (num_bytes > MAX_I2C_BYTES) {
		num_bytes = MAX_I2C_BYTES;
	}
	for (i = 0; i < num_bytes; i++) {
		buf[i] = (uint8_t)strtol(argv[3 + i], NULL, 16);
	}

	if (i2c_write(dev, buf, num_bytes, dev_addr) < 0) {
		shell_error(shell, "Failed to write to device: %s", argv[1]);
		return -EIO;
	}

	return 0;
}

static int cmd_i2c_read_byte(const struct shell *shell,
			     size_t argc, char **argv)
{
	const struct device *dev;
	int reg_addr;
	int dev_addr;
	uint8_t out;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I2C: Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	dev_addr = strtol(argv[2], NULL, 16);
	reg_addr = strtol(argv[3], NULL, 16);

	if (i2c_reg_read_byte(dev, dev_addr, reg_addr, &out) < 0) {
		shell_error(shell, "Failed to read from device: %s", argv[1]);
		return -EIO;
	}

	shell_print(shell, "Output: 0x%x", out);

	return 0;
}

/* i2c read_bytes <device> <dev_addr> [<numbytes>] */
static int cmd_i2c_read_multi_bytes(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t buf[MAX_I2C_BYTES];
	const struct device *dev;
	int num_bytes;
	int reg_addr;
	int dev_addr;
	int i;
	uint8_t tx_buf;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I2C: Device driver %s not found.", argv[1]);
		return -ENODEV;
	}

	dev_addr = strtol(argv[2], NULL, 16);
	reg_addr = strtol(argv[3], NULL, 16);
	tx_buf = (uint8_t)reg_addr;

	if (argc > 4) {
		num_bytes = strtol(argv[4], NULL, 16);
		if (num_bytes > MAX_I2C_BYTES)
			num_bytes = MAX_I2C_BYTES;
	} else {
		num_bytes = MAX_I2C_BYTES;
	}

	for (i = 0; i < num_bytes; i++) {
		if (i2c_write(dev, &tx_buf, 1, dev_addr) < 0) {
			shell_error(shell, "Failed to mb write from device: %s", argv[1]);
			return -EIO;
		}

		if (i2c_read(dev, &buf[i], 1, dev_addr) < 0) {
			shell_error(shell, "Failed to mb read from device: %s", argv[1]);
			return -EIO;
		}
		tx_buf++;
	}

	shell_hexdump(shell, buf, num_bytes);

	return 0;
}

/* i2c read <device> <dev_addr> [<numbytes>] */
static int cmd_i2c_read(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t buf[MAX_I2C_BYTES];
	const struct device *dev;
	int num_bytes;
	int reg_addr;
	int dev_addr;

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(shell, "I2C: Device driver %s not found.", argv[1]);
		return -ENODEV;
	}

	dev_addr = strtol(argv[2], NULL, 16);
	reg_addr = strtol(argv[3], NULL, 16);
	if (argc > 4) {
		num_bytes = strtol(argv[4], NULL, 16);
		if (num_bytes > MAX_I2C_BYTES)
			num_bytes = MAX_I2C_BYTES;
	} else {
		num_bytes = MAX_I2C_BYTES;
	}

	if (i2c_burst_read(dev, dev_addr, reg_addr, buf, num_bytes) < 0) {
		shell_error(shell, "Failed to read from device: %s", argv[1]);
		return -EIO;
	}

	shell_hexdump(shell, buf, num_bytes);

	return 0;
}

#ifdef CONFIG_PFR_SW_MAILBOX
#define thread_fifo_size 256
#define THREAD_DELAY 100
static struct k_thread zipmb_n_t;
static struct k_thread zipmb_f_t;
struct k_sem sem_0_0, sem_0_40, sem_0_50;
struct k_sem sem_1_31, sem_1_41, sem_1_FF;
struct k_sem sem_fifo_50, sem_fifo_31;

K_THREAD_STACK_DEFINE(i2c_thread_n_s, thread_fifo_size);
K_THREAD_STACK_DEFINE(i2c_thread_f_s, thread_fifo_size);

/* i2c thread demo for swmbx notify */
void swmbx_notify(void *a, void *b, void *c)
{
	LOG_INF("swmbx: swmbx_notify successful.");

	while (1) {
		if (k_sem_take(&sem_0_0, K_MSEC(50)) == 0) {
			LOG_INF("swmbx: SEM_0_0 is taken!!");
		}

		if (k_sem_take(&sem_0_40, K_MSEC(50)) == 0) {
			LOG_INF("swmbx: SEM_0_40 is taken!!");
		}

		if (k_sem_take(&sem_0_50, K_MSEC(50)) == 0) {
			LOG_INF("swmbx: SEM_0_50 is taken!!");
		}

		if (k_sem_take(&sem_1_31, K_MSEC(50)) == 0) {
			LOG_INF("swmbx: SEM_1_31 is taken!!");
		}

		if (k_sem_take(&sem_1_41, K_MSEC(50)) == 0) {
			LOG_INF("swmbx: SEM_1_41 is taken!!");
		}

		if (k_sem_take(&sem_1_FF, K_MSEC(50)) == 0) {
			LOG_INF("swmbx: SEM_1_FF is taken!!");
		}

		k_sleep(K_MSEC(THREAD_DELAY));
	}
}

/* i2c thread demo for swmbx fifo */
void swmbx_fifo(void *a, void *b, void *c)
{
	LOG_INF("swmbx: swmbx_fifo successful.");

	while (1) {
		if (k_sem_take(&sem_fifo_50, K_MSEC(50)) == 0) {
			LOG_INF("swmbx: SEM_FIFO_50 is taken!!");
		}

		if (k_sem_take(&sem_fifo_31, K_MSEC(50)) == 0) {
			LOG_INF("swmbx: SEM_FIFO_31 is taken!!");
		}

		k_sleep(K_MSEC(THREAD_DELAY));
	}
}

static int cmd_i2c_sw_mbx(const struct shell *shell,
			      size_t argc, char **argv)
{
	int ret = 0;
	const struct device *swmbx_ctrl;
	k_tid_t pidn, pidf;
	uint32_t protect0_bit[] = {0xffff0000, 0x55555555, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0};
	uint32_t protect1_bit[] = {0x0000ffff, 0xaaaaaaaa, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0};

	/* swmbx ctrl for 0 /1 devices with write protect */
	swmbx_ctrl = device_get_binding("SWMBX");
	if (!swmbx_ctrl) {
		shell_error(shell, "xx I2C: SWMBX Controller not be found.");
		return -ENODEV;
	}

	if (swmbx_ctrl) {
		/* apply function enable */
		ret = swmbx_enable_behavior(swmbx_ctrl,
			(SWMBX_PROTECT | SWMBX_NOTIFY | SWMBX_FIFO), true);
		if (ret) {
			shell_error(shell, "xx I2C: Enable SWMBX function failed.");
			return -ENODEV;
		}

		/* swmbx protect usage */
		ret = swmbx_apply_protect(swmbx_ctrl, 0x0, protect0_bit, 0, 7);
		if (ret) {
			shell_error(shell, "xx I2C: Apply SWMBX protect bitmap 0 fail.");
			return -ENODEV;
		}

		ret = swmbx_apply_protect(swmbx_ctrl, 0x1, protect1_bit, 0, 7);
		if (ret) {
			shell_error(shell, "xx I2C: Apply SWMBX protect bitmap 1 fail.");
			return -ENODEV;
		}

		ret = swmbx_update_protect(swmbx_ctrl, 0x0, 0x40, true);
		ret = swmbx_update_protect(swmbx_ctrl, 0x0, 0x41, true);

		ret = swmbx_update_protect(swmbx_ctrl, 0x1, 0x45, true);
		ret = swmbx_update_protect(swmbx_ctrl, 0x1, 0x46, true);

		/* swmbx notify usage */
		k_sem_init(&sem_0_0, 0, 1);
		k_sem_init(&sem_0_40, 0, 1);
		k_sem_init(&sem_0_50, 0, 1);
		k_sem_init(&sem_1_31, 0, 1);
		k_sem_init(&sem_1_41, 0, 1);
		k_sem_init(&sem_1_FF, 0, 1);

		ret = swmbx_update_notify(swmbx_ctrl, 0x0, &sem_0_0,
		0x0, true);
		ret = swmbx_update_notify(swmbx_ctrl, 0x0, &sem_0_40,
		0x40, true);
		ret = swmbx_update_notify(swmbx_ctrl, 0x0, &sem_0_50,
		0x50, true);
		ret = swmbx_update_notify(swmbx_ctrl, 0x1, &sem_1_31,
		0x31, true);
		ret = swmbx_update_notify(swmbx_ctrl, 0x1, &sem_1_41,
		0x41, true);
		ret = swmbx_update_notify(swmbx_ctrl, 0x1, &sem_1_FF,
		0xFF, true);

		if (IS_ENABLED(CONFIG_MULTITHREADING)) {
			pidn = k_thread_create(&zipmb_n_t,
						  i2c_thread_n_s,
						  thread_fifo_size,
						  (k_thread_entry_t)swmbx_notify,
						  NULL, NULL, NULL,
						  -1,
						  0, K_NO_WAIT);
		}

		/* turn on fifo */
		k_sem_init(&sem_fifo_50, 0, 1);
		k_sem_init(&sem_fifo_31, 0, 1);

		ret = swmbx_update_fifo(swmbx_ctrl, &sem_fifo_50,
		0, 0x50, 0x5, SWMBX_FIFO_NOTIFY_STOP, true);
		if (ret) {
			shell_error(shell, "xx I2C: Apply fifo 50 fail.");
			return -ENODEV;
		}

		ret = swmbx_update_fifo(swmbx_ctrl, &sem_fifo_31,
		1, 0x31, 0x5, SWMBX_FIFO_NOTIFY_STOP, true);
		if (ret) {
			shell_error(shell, "xx I2C: Apply fifo 31 fail.");
			return -ENODEV;
		}

		if (IS_ENABLED(CONFIG_MULTITHREADING)) {
			pidf = k_thread_create(&zipmb_f_t,
						  i2c_thread_f_s,
						  thread_fifo_size,
						  (k_thread_entry_t)swmbx_fifo,
						  NULL, NULL, NULL,
						  -1,
						  0, K_NO_WAIT);
		}
	}

	return ret;
}

static int cmd_i2c_sw_mbx_r(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *swmbx_ctrl;
	int ret = 0;
	uint8_t fifo;
	uint8_t addr;
	uint8_t val;

	/* swmbx ctrl for 0 /1 devices with write protect */
	swmbx_ctrl = device_get_binding("SWMBX");
	if (!swmbx_ctrl) {
		shell_error(shell, "xx I2C: SWMBX Controller not be found.");
		return -ENODEV;
	}

	fifo = strtol(argv[1], NULL, 16);
	addr = strtol(argv[2], NULL, 16);

	ret =  swmbx_read(swmbx_ctrl, fifo, addr, &val);
	if (!ret) {
		shell_hexdump(shell, &val, 1);
	} else {
		shell_error(shell, "xx I2C: SWMBX read failed.");
		return -ENODEV;
	}

	return 0;
}

static int cmd_i2c_sw_mbx_w(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *swmbx_ctrl;
	int ret = 0;
	uint8_t fifo;
	uint8_t addr;
	uint8_t val;

	/* swmbx ctrl for 0 /1 devices with write protect */
	swmbx_ctrl = device_get_binding("SWMBX");
	if (!swmbx_ctrl) {
		shell_error(shell, "xx I2C: SWMBX Controller not be found.");
		return -ENODEV;
	}

	fifo = strtol(argv[1], NULL, 16);
	addr = strtol(argv[2], NULL, 16);
	val = strtol(argv[3], NULL, 16);

	ret = swmbx_write(swmbx_ctrl, fifo, addr, &val);
	if (ret) {
		shell_error(shell, "xx I2C: SWMBX write failed.");
		return -ENODEV;
	}

	return 0;
}

static int cmd_i2c_sw_mbx_flush(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *swmbx_ctrl;
	int ret = 0;
	uint8_t index;

	swmbx_ctrl = device_get_binding("SWMBX");
	if (!swmbx_ctrl) {
		shell_error(shell, "xx I2C: SWMBX Controller not be found.");
		return -ENODEV;
	}

	index = strtol(argv[1], NULL, 16);

	ret = swmbx_flush_fifo(swmbx_ctrl, index);
	if (ret) {
		shell_error(shell, "xx I2C: flush address 0x%x is not found.",
				index);
		return -ENODEV;
	}

	return 0;
}

#endif

#ifdef CONFIG_I2C_PFR_SNOOP
static int cmd_sp_en(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_sp_dev = NULL;
	int ret = 0;
	int sp_en;
	int flt_idx;
	int addr;

	sp_en = strtol(argv[2], NULL, 16);
	flt_idx = strtol(argv[3], NULL, 16);
	addr = strtol(argv[4], NULL, 16);

	pfr_sp_dev = device_get_binding(argv[1]);
	if (!pfr_sp_dev) {
		shell_error(shell, "xx I2C: PFR SP Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_sp_dev) {
		ret = ast_i2c_snoop_en(pfr_sp_dev, (uint8_t)sp_en,
		(uint8_t)flt_idx, (uint8_t)addr);
		if (ret) {
			shell_error(shell, "xx I2C: PFR SP Enable / Disable failed.");
			return ret;
		}
	}

	return ret;

}

static int cmd_sp_update(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_sp_dev = NULL;
	int ret = 0;
	int byte_size;

	byte_size = strtol(argv[2], NULL, 16);

	pfr_sp_dev = device_get_binding(argv[1]);
	if (!pfr_sp_dev) {
		shell_error(shell, "xx I2C: PFR SP Device driver %s not found.",
				argv[1]);
		return -ENODEV;
	}

	if (pfr_sp_dev) {
		ret = ast_i2c_snoop_update(pfr_sp_dev, byte_size);
		if (ret) {
			shell_error(shell, "xx I2C: PFR SP Update failed.");
			return ret;
		}
	}

	return ret;
}
#endif

#ifdef CONFIG_I2C_PFR_FILTER

struct ast_i2c_f_bitmap data_flt[] = {
{
{	/* block all (index 0) */
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000
}

},
{
{	/* accept all (index 1) */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
}

},
{
{	/* block every 16 byte (index 2) */
	0xFFFF0000, 0xFFFF0000, 0xFFFF0000, 0xFFFF0000,
	0xFFFF0000, 0xFFFF0000, 0xFFFF0000, 0xFFFF0000
}

},
{
{	/* block first 16 byte (index 3) */
	0xFFFF0000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
}

},
{
{	/* block first 128 byte (index 4) */
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
}

},
{
{	/* block last 128 byte (index 5) */
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
}

},
};

int tbl_size = sizeof(data_flt) / (sizeof(struct ast_i2c_f_bitmap));

static int cmd_flt_init(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_flt_dev = NULL;
	int ret = 0;

	pfr_flt_dev = device_get_binding(argv[1]);
	if (!pfr_flt_dev) {
		shell_error(shell, "xx I2C: PFR FLT Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_flt_dev) {
		ret = ast_i2c_filter_init(pfr_flt_dev);
		if (ret) {
			shell_error(shell, "xx I2C: PFR FLT Initial failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_flt_en(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_flt_dev = NULL;
	int ret = 0;
	int flt_en;
	int wlist_en;
	int clr_idx;
	int clr_tbl;

	flt_en = strtol(argv[2], NULL, 16);
	wlist_en = strtol(argv[3], NULL, 16);
	clr_idx = strtol(argv[4], NULL, 16);
	clr_tbl = strtol(argv[5], NULL, 16);

	pfr_flt_dev = device_get_binding(argv[1]);
	if (!pfr_flt_dev) {
		shell_error(shell, "xx I2C: PFR FLT Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_flt_dev) {
		ret = ast_i2c_filter_en(pfr_flt_dev, (uint8_t)flt_en,
		(uint8_t)wlist_en, (uint8_t)clr_idx, (uint8_t)clr_tbl);
		if (ret) {
			shell_error(shell, "xx I2C: PFR FLT Enable / Disable failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_flt_update(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_flt_dev = NULL;
	int ret = 0;
	int idx;
	int addr;
	int tbl_idx;

	idx = strtol(argv[2], NULL, 16);
	addr = strtol(argv[3], NULL, 16);
	tbl_idx = strtol(argv[4], NULL, 16);

	pfr_flt_dev = device_get_binding(argv[1]);
	if (!pfr_flt_dev) {
		shell_error(shell, "xx I2C: PFR FLT Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	} else if (tbl_idx > tbl_size) {
		shell_error(shell, "xx I2C: PFR FLT Table Index %s not found.",
			    argv[4]);
		shell_error(shell, "xx I2C: PFR FLT Table MAX Index is %d ",
			    tbl_size);
		return -ENODEV;
	}

	if (pfr_flt_dev) {
		ret = ast_i2c_filter_update(pfr_flt_dev, (uint8_t)idx,
		(uint8_t)addr, &data_flt[tbl_idx]);
		if (ret) {
			shell_error(shell, "xx I2C: PFR FLT Update failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_flt_default(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_flt_dev = NULL;
	int ret = 0;
	int pass;

	pass = strtol(argv[2], NULL, 16);

	pfr_flt_dev = device_get_binding(argv[1]);
	if (!pfr_flt_dev) {
		shell_error(shell, "xx I2C: PFR FLT Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_flt_dev) {
		ret = ast_i2c_filter_default(pfr_flt_dev, (uint8_t)pass);
		if (ret) {
			shell_error(shell, "xx I2C: PFR FLT Set Default failed.");
			return ret;
		}
	}

	return ret;
}

#endif

#ifdef CONFIG_I2C_PFR_MAILBOX
static int cmd_mbx_init(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_init(pfr_mbx_dev);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Initial failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_mbx_addr(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int dev_idx;
	int idx;
	int addr;
	int enable;

	dev_idx = strtol(argv[2], NULL, 16);
	idx = strtol(argv[3], NULL, 16);
	addr = strtol(argv[4], NULL, 16);
	enable = strtol(argv[5], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_addr(pfr_mbx_dev, (uint8_t)dev_idx,
		(uint8_t)idx, 0x1, (uint8_t)addr, (uint8_t)enable);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Set address failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_mbx_en(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int dev_idx;
	int enable;

	dev_idx = strtol(argv[2], NULL, 16);
	enable = strtol(argv[3], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_en(pfr_mbx_dev, (uint8_t)dev_idx,
		(uint8_t)enable);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Enable / Disable failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_mbx_n_addr(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int idx;
	int addr;

	idx = strtol(argv[2], NULL, 16);
	addr = strtol(argv[3], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_notify_addr(pfr_mbx_dev, (uint8_t)idx,
		(uint8_t)addr);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Notify address failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_mbx_n_en(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int dev_idx;
	int idx;
	int type;
	int enable;

	dev_idx = strtol(argv[2], NULL, 16);
	idx = strtol(argv[3], NULL, 16);
	type = strtol(argv[4], NULL, 16);
	enable = strtol(argv[5], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_notify_en(pfr_mbx_dev, (uint8_t)dev_idx,
		(uint8_t)idx, (uint8_t)type, (uint8_t)enable);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Notify Enable / Disable failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_mbx_p_en(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int dev_idx;
	int addr;
	int enable;

	dev_idx = strtol(argv[2], NULL, 16);
	addr = strtol(argv[3], NULL, 16);
	enable = strtol(argv[4], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_protect(pfr_mbx_dev, (uint8_t)dev_idx,
		(uint8_t)addr, (uint8_t)enable);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Protect Enable / Disable failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_mbx_f_p(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int priority;

	priority = strtol(argv[2], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_fifo_priority(pfr_mbx_dev, (uint8_t)priority);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX FIFO Priority failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_mbx_f_set(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int idx;
	int addr;
	int type;

	idx = strtol(argv[2], NULL, 16);
	addr = strtol(argv[3], NULL, 16);
	type = strtol(argv[4], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_fifo_apply(pfr_mbx_dev, (uint8_t)idx,
		(uint8_t)addr, (uint8_t)type);
		if (ret) {
			shell_error(shell, "xx I2C: PFR FIFO Set index failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_mbx_f_en(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int idx;
	int base;
	int length;

	idx = strtol(argv[2], NULL, 16);
	base = strtol(argv[3], NULL, 16);
	length = strtol(argv[4], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_fifo_en(pfr_mbx_dev, (uint8_t)idx,
		(uint16_t)base, (uint16_t)length);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX FIFO Enable / Disable failed.");
			return ret;
		}
	}

	return ret;
}

static int cmd_mbx_f_access(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int idx;
	int type;
	int data;

	idx = strtol(argv[2], NULL, 16);
	type = strtol(argv[3], NULL, 16);
	data = strtol(argv[4], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_fifo_access(pfr_mbx_dev, (uint8_t)idx,
		(uint8_t)type, (uint8_t *)&data);

		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX FIFO access failed.");
			return ret;
		}

		if (type == AST_I2C_M_R)
			shell_print(shell, "I2C: PFR MBX FIFO read back is 0x%x", data);
	}

	return ret;
}

#endif

#ifdef CONFIG_I2C_PFR
uint8_t EEPROM_COUNT = 8;
uint8_t EEPROM_PASS_TBL[] = {0, 0, 0, 1, 1, 4, 4, 5};

static int cmd_i2c_pfr_demo(const struct shell *shell,
			      size_t argc, char **argv)
{
	int ret = 0;

#ifdef CONFIG_I2C_PFR_FILTER
	const struct device *pfr_flt_dev = NULL;
	int i = 0;
	/* initial flt */
	pfr_flt_dev = device_get_binding("I2C_FILTER_0");
	if (!pfr_flt_dev) {
		shell_error(shell, "xx I2C: PFR FLT Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_flt_dev) {
		ret = ast_i2c_filter_init(pfr_flt_dev);
		if (ret) {
			shell_error(shell, "xx I2C: PFR FLT Initial failed.");
			return ret;
		}

		ret = ast_i2c_filter_en(pfr_flt_dev, 1, 1, 0, 0);
		if (ret) {
			shell_error(shell, "xx I2C: PFR FLT Enable / Disable failed.");
			return ret;
		}

		ret = ast_i2c_filter_default(pfr_flt_dev, 0);
		if (ret) {
			shell_error(shell, "xx I2C: PFR FLT Set Default failed.");
			return ret;
		}

		for (i = 0x0; i < EEPROM_COUNT; i++) {
			ret = ast_i2c_filter_update(pfr_flt_dev, (uint8_t)(i),
			(uint8_t)(0x50 + i), &data_flt[EEPROM_PASS_TBL[i]]);
			if (ret) {
				shell_error(shell, "xx I2C: PFR FLT Update failed.");
				return ret;
			}
		}
	}
#endif

#ifdef CONFIG_I2C_PFR_MAILBOX
	const struct device *pfr_mbx_dev = NULL;

	pfr_mbx_dev = device_get_binding("I2C_MBX");

	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev) {
		ret = ast_i2c_mbx_init(pfr_mbx_dev);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Initial failed.");
			return ret;
		}

		ret = ast_i2c_mbx_addr(pfr_mbx_dev, 0x0, 0x0, 0x1, 0x29, 0x1);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Set address failed.");
			return ret;
		}

		ret = ast_i2c_mbx_en(pfr_mbx_dev, 0x0, 0x1);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Enable / Disable failed.");
			return ret;
		}

		ret = ast_i2c_mbx_addr(pfr_mbx_dev, 0x1, 0x0, 0x1, 0x39, 0x1);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Set address failed.");
			return ret;
		}

		ret = ast_i2c_mbx_en(pfr_mbx_dev, 0x1, 0x1);
		if (ret) {
			shell_error(shell, "xx I2C: PFR MBX Enable / Disable failed.");
			return ret;
		}
	}
#endif

	return ret;
}
#endif

#ifdef CONFIG_I2C_TARGET
#ifdef CONFIG_I2C_EEPROM_TARGET
#define EEPROM_SLAVE	0
#define TEST_DATA_SIZE	20
static const uint8_t eeprom_0_data[TEST_DATA_SIZE] = "0123456789abcdefghij";
#endif
static int cmd_i2c_target_attach(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *target_dev;

	target_dev = device_get_binding(argv[1]);
	if (!target_dev) {
		shell_error(shell, "xx I2C: Target Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

#ifdef CONFIG_I2C_EEPROM_TARGET
	int cmp = 0;
	/* compare slave type*/
	cmp = strcmp(argv[2], "EE");

	if (!cmp) {
		/* Program differentiable data into the two devices through a back door
		 * that doesn't use I2C.
		 */
		if (eeprom_target_program(target_dev, eeprom_0_data, TEST_DATA_SIZE))
			shell_error(shell, "I2C: Target Device driver %s found.", target_dev->name);
	}
#endif

	/* Attach each EEPROM to its owning bus as a slave device. */
	if (i2c_target_driver_register(target_dev))
		shell_error(shell, "I2C: Target Device driver %s not found.", target_dev->name);

	return 0;
}

static int cmd_i2c_target_detach(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *target_dev;

	target_dev = device_get_binding(argv[1]);
	if (!target_dev) {
		shell_error(shell, "xx I2C: Target Device driver %s not found.",
				argv[1]);
		return -ENODEV;
	}

	/* Attach each EEPROM to its owning bus as a slave device. */
	if (i2c_target_driver_unregister(target_dev))
		shell_error(shell, "I2C: Target Device driver %s not found.", target_dev->name);

	return 0;
}

#ifdef CONFIG_I2C_IPMB_TARGET
static int cmd_i2c_ipmb_read(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *target_dev = NULL;
	int ret = 0;
	struct ipmb_msg *msg = NULL;
	uint8_t length = 0;
	uint8_t *buf = NULL;

	target_dev = device_get_binding(argv[1]);
	if (!target_dev) {
		shell_error(shell, "xx I2C: Target Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (target_dev) {
		ret = ipmb_target_read(target_dev, &msg, &length);

		if (!ret) {
			buf = (uint8_t *)(msg);
			shell_print(shell, "ipmb length : %x", length);

			shell_hexdump(shell, buf, length);
		}
	}

	return ret;
}
#endif
#endif

static void device_name_get(size_t idx, struct shell_static_entry *entry);

SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, I2C_DEVICE_PREFIX);

	entry->syntax = (dev) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_iic_cmds,
			       SHELL_CMD(scan, &dsub_device_name,
					 "Scan I2C devices", cmd_i2c_scan),
			       SHELL_CMD(recover, &dsub_device_name,
					 "Recover I2C bus", cmd_i2c_recover),
			       SHELL_CMD_ARG(read, &dsub_device_name,
					     "Read bytes from an I2C device",
					     cmd_i2c_read, 3, MAX_I2C_BYTES),
				SHELL_CMD_ARG(read_sbyte, &dsub_device_name,
					  "Read bytes from an I2C device with single",
					  cmd_i2c_read_multi_bytes, 3, MAX_I2C_BYTES),
			       SHELL_CMD_ARG(read_byte, &dsub_device_name,
					     "Read a byte from an I2C device",
					     cmd_i2c_read_byte, 3, 1),
			       SHELL_CMD_ARG(write, &dsub_device_name,
					     "Write bytes to an I2C device",
					     cmd_i2c_write, 3, MAX_I2C_BYTES),
				SHELL_CMD_ARG(write_sbyte, &dsub_device_name,
					  "Write bytes to an I2C device with single",
					  cmd_i2c_write_multi_bytes, 3, MAX_I2C_BYTES),
			       SHELL_CMD_ARG(write_byte, &dsub_device_name,
					     "Write a byte to an I2C device",
					     cmd_i2c_write_byte, 4, 1),
				SHELL_CMD_ARG(write_bytes, &dsub_device_name,
					  "Write bytes to an I2C device",
					  cmd_i2c_write_bytes, 3, MAX_I2C_BYTES),
#ifdef CONFIG_I2C_TARGET
			       SHELL_CMD_ARG(target_attach, &dsub_device_name,
					     "Attach target device",
					     cmd_i2c_target_attach, 2, 1),
			       SHELL_CMD_ARG(target_detach, &dsub_device_name,
					     "Detach target device",
					     cmd_i2c_target_detach, 0, 1),
#ifdef CONFIG_I2C_IPMB_TARGET
			       SHELL_CMD_ARG(target_ipmb_read, &dsub_device_name,
					     "Read ipmb buffer from target",
					      cmd_i2c_ipmb_read, 0, 1),
#endif
#endif
#ifdef CONFIG_I2C_PFR
				SHELL_CMD_ARG(pfr_demo, &dsub_device_name,
					  "pfr demo default",
					   cmd_i2c_pfr_demo, 0, 0),
#endif
#ifdef CONFIG_I2C_SWMBX_SLAVE
				SHELL_CMD_ARG(slave_swmbx, &dsub_device_name,
					"Apply sw mbx slave",
					cmd_i2c_sw_mbx, 0, 0),
				SHELL_CMD_ARG(slave_swmbx_r, &dsub_device_name,
					"Read sw mbx slave",
					cmd_i2c_sw_mbx_r, 3, 3),
				SHELL_CMD_ARG(slave_swmbx_w, &dsub_device_name,
					"Write sw mbx slave",
					cmd_i2c_sw_mbx_w, 4, 4),
				SHELL_CMD_ARG(slave_swmbx_flush, &dsub_device_name,
					"Flush sw mbx slave",
					cmd_i2c_sw_mbx_flush, 2, 2),
#endif
#ifdef CONFIG_I2C_PFR_SNOOP
				SHELL_CMD_ARG(sp_en, &dsub_device_name,
				"Enable pfr snoop",
				cmd_sp_en, 0, 4),
				SHELL_CMD_ARG(sp_update, &dsub_device_name,
				"Update pfr snoop",
				cmd_sp_update, 0, 2),
#endif
#ifdef CONFIG_I2C_PFR_FILTER
				SHELL_CMD_ARG(flt_init, &dsub_device_name,
				 "Init pfr filter device",
				 cmd_flt_init, 0, 1),
				SHELL_CMD_ARG(flt_en, &dsub_device_name,
				"Enable pfr flt",
				cmd_flt_en, 0, 5),
				SHELL_CMD_ARG(flt_update, &dsub_device_name,
				"Update pfr flt",
				cmd_flt_update, 0, 4),
				SHELL_CMD_ARG(flt_default, &dsub_device_name,
				"Set pfr flt default",
				cmd_flt_default, 0, 2),

#endif
#ifdef CONFIG_I2C_PFR_MAILBOX
				SHELL_CMD_ARG(mbx_init, &dsub_device_name,
					"Init pfr mbx device",
					cmd_mbx_init, 0, 1),
				SHELL_CMD_ARG(mbx_addr, &dsub_device_name,
					"Set pfr mbx device address",
					cmd_mbx_addr, 0, 6),
				SHELL_CMD_ARG(mbx_en, &dsub_device_name,
					"Enable pfr mbx",
					cmd_mbx_en, 0, 3),
				SHELL_CMD_ARG(mbx_notify_addr, &dsub_device_name,
					"Set pfr mbx notify address",
					cmd_mbx_n_addr, 0, 3),
				SHELL_CMD_ARG(mbx_notify_en, &dsub_device_name,
					"Enable pfr mbx notify",
					cmd_mbx_n_en, 0, 5),
				SHELL_CMD_ARG(mbx_protect_en, &dsub_device_name,
					"Enable pfr mbx protect address",
					cmd_mbx_p_en, 0, 4),
				SHELL_CMD_ARG(mbx_fifo_priority, &dsub_device_name,
					"Set pfr mbx FIFO priority",
					cmd_mbx_f_p, 0, 2),
				SHELL_CMD_ARG(mbx_fifo_set, &dsub_device_name,
					"Set pfr mbx FIFO address",
					cmd_mbx_f_set, 0, 4),
				SHELL_CMD_ARG(mbx_fifo_en, &dsub_device_name,
					"Enable pfr mbx FIFO",
					cmd_mbx_f_en, 0, 4),
				SHELL_CMD_ARG(mbx_fifo_rw, &dsub_device_name,
					"Access pfr mbx FIFO wt CPU",
					cmd_mbx_f_access, 0, 4),
#endif
			       SHELL_SUBCMD_SET_END     /* Array terminated. */
			       );

SHELL_CMD_REGISTER(iic, &sub_iic_cmds, "ASPEED IIC commands", NULL);
