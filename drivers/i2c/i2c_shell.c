/*
 * Copyright (c) 2018 Prevas A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <shell/shell.h>
#include <stdlib.h>
#include <drivers/i2c.h>
#include <drivers/i2c/slave/eeprom.h>
#include <drivers/i2c/slave/ipmb.h>
#include <drivers/i2c/pfr/i2c_filter.h>
#include <drivers/i2c/pfr/i2c_mailbox.h>

#include <string.h>
#include <sys/util.h>
#include <stdlib.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_shell, CONFIG_LOG_DEFAULT_LEVEL);

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
		shell_error(shell, "I2C: Device driver %s not found.", argv[1]);
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

#ifdef CONFIG_I2C_PFR_MAILBOX
static int cmd_pfr_mbx_init(const struct shell *shell,
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

	if (pfr_mbx_dev != NULL) {
		ret = ast_i2c_mbx_init(pfr_mbx_dev);
	}

	return ret;
}

static int cmd_pfr_mbx_addr(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int dev_idx;
	int idx;
	int offset;
	int addr;
	int enable;

	dev_idx = strtol(argv[2], NULL, 16);
	idx = strtol(argv[3], NULL, 16);
	offset = strtol(argv[4], NULL, 16);
	addr = strtol(argv[5], NULL, 16);
	enable = strtol(argv[6], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev != NULL) {
		ret = ast_i2c_mbx_addr(pfr_mbx_dev, (uint8_t)dev_idx,
		(uint8_t)idx, (uint8_t)offset, (uint8_t)addr, (uint8_t)enable);
	}

	return ret;
}

static int cmd_pfr_mbx_en(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *pfr_mbx_dev = NULL;
	int ret = 0;
	int dev_idx;
	int base;
	int length;
	int enable;

	dev_idx = strtol(argv[2], NULL, 16);
	base = strtol(argv[3], NULL, 16);
	length = strtol(argv[4], NULL, 16);
	enable = strtol(argv[5], NULL, 16);

	pfr_mbx_dev = device_get_binding(argv[1]);
	if (!pfr_mbx_dev) {
		shell_error(shell, "xx I2C: PFR MBX Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (pfr_mbx_dev != NULL) {
		ret = ast_i2c_mbx_en(pfr_mbx_dev, (uint8_t)dev_idx,
		(uint32_t) base, (uint16_t)length, (uint8_t)enable);
	}

	return ret;
}


#endif

#ifdef CONFIG_I2C_SLAVE
#ifdef CONFIG_I2C_EEPROM_SLAVE
#define EEPROM_SLAVE	0
#define TEST_DATA_SIZE	20
static const uint8_t eeprom_0_data[TEST_DATA_SIZE] = "0123456789abcdefghij";
#endif
static int cmd_i2c_slave_attach(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *slave_dev;

	slave_dev = device_get_binding(argv[1]);
	if (!slave_dev) {
		shell_error(shell, "xx I2C: Slave Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

#ifdef CONFIG_I2C_EEPROM_SLAVE
	int cmp = 0;
	/* compare slave type*/
	cmp = strcmp(argv[2], "EE");

	if (!cmp) {
		/* Program differentiable data into the two devices through a back door
		 * that doesn't use I2C.
		 */
		if (eeprom_slave_program(slave_dev, eeprom_0_data, TEST_DATA_SIZE))
			shell_error(shell, "I2C: Slave Device driver %s found.", slave_dev->name);
	}
#endif

	/* Attach each EEPROM to its owning bus as a slave device. */
	if(i2c_slave_driver_register(slave_dev))
		shell_error(shell, "I2C: Slave Device driver %s not found.", slave_dev->name);

	return 0;
}

static int cmd_i2c_slave_deattach(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *slave_dev;

	slave_dev = device_get_binding(argv[1]);
	if (!slave_dev) {
		shell_error(shell, "xx I2C: Slave Device driver %s not found.",
				argv[1]);
		return -ENODEV;
	}

	/* Attach each EEPROM to its owning bus as a slave device. */
	if (i2c_slave_driver_unregister(slave_dev))
		shell_error(shell, "I2C: Slave Device driver %s not found.", slave_dev->name);

	return 0;
}

#ifdef CONFIG_I2C_IPMB_SLAVE
static int cmd_i2c_ipmb_read(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *slave_dev = NULL;
	int ret = 0;
	struct ipmb_msg *msg = NULL;
	uint8_t length = 0;
	uint8_t *buf = NULL;

	slave_dev = device_get_binding(argv[1]);
	if (!slave_dev) {
		shell_error(shell, "xx I2C: Slave Device driver %s not found.",
			    argv[1]);
		return -ENODEV;
	}

	if (slave_dev != NULL) {
		ret = ipmb_slave_read(slave_dev, &msg, &length);

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

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_i2c_cmds,
			       SHELL_CMD(scan, &dsub_device_name,
					 "Scan I2C devices", cmd_i2c_scan),
			       SHELL_CMD(recover, &dsub_device_name,
					 "Recover I2C bus", cmd_i2c_recover),
			       SHELL_CMD_ARG(read, &dsub_device_name,
					     "Read bytes from an I2C device",
					     cmd_i2c_read, 3, MAX_I2C_BYTES),
			       SHELL_CMD_ARG(read_byte, &dsub_device_name,
					     "Read a byte from an I2C device",
					     cmd_i2c_read_byte, 3, 1),
			       SHELL_CMD_ARG(write, &dsub_device_name,
					     "Write bytes to an I2C device",
					     cmd_i2c_write, 3, MAX_I2C_BYTES),
			       SHELL_CMD_ARG(write_byte, &dsub_device_name,
					     "Write a byte to an I2C device",
					     cmd_i2c_write_byte, 4, 1),
#ifdef CONFIG_I2C_SLAVE
			       SHELL_CMD_ARG(slave_attach, &dsub_device_name,
					     "Attach slave device",
					     cmd_i2c_slave_attach, 2, 1),
			       SHELL_CMD_ARG(slave_deattach, &dsub_device_name,
					     "Deattach slave device",
					     cmd_i2c_slave_deattach, 0, 1),
#ifdef CONFIG_I2C_IPMB_SLAVE
			       SHELL_CMD_ARG(slave_ipmb_read, &dsub_device_name,
					     "Read ipmb buffer from slave",
					      cmd_i2c_ipmb_read, 0, 1),
#endif
#endif
#ifdef CONFIG_I2C_PFR_MAILBOX
			       SHELL_CMD_ARG(mbx_init, &dsub_device_name,
					"Init pfr mbx device",
					cmd_pfr_mbx_init, 0, 1),
				SHELL_CMD_ARG(mbx_addr, &dsub_device_name,
					"Set pfr mbx device address",
					cmd_pfr_mbx_addr, 0, 6),
				SHELL_CMD_ARG(mbx_enable, &dsub_device_name,
					"Enable pfr mbx",
					cmd_pfr_mbx_en, 0, 5),
#endif
			       SHELL_SUBCMD_SET_END     /* Array terminated. */
			       );

SHELL_CMD_REGISTER(i2c, &sub_i2c_cmds, "I2C commands", NULL);
