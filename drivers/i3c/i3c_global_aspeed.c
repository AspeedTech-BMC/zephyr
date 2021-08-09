/*
 * Copyright (c) ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_i3c_global

#include <soc.h>
#include <device.h>
#include <drivers/reset_control.h>
#include <sys/sys_io.h>
#include <logging/log.h>
#define LOG_LEVEL CONFIG_I3C_LOG_LEVEL
LOG_MODULE_REGISTER(i3c_global);

/* Device config */
struct i3c_global_config {
	uintptr_t base;
	const reset_control_subsys_t rst_id;
	const uint32_t ni3cs;
};
#define DEV_CFG(dev)			((const struct i3c_global_config *const)(dev)->config)

/* Registers */
#define I3C_GLOBAL_REG0(x)		(((x) * 0x10) + 0x10)
#define I3C_GLOBAL_REG1(x)		(((x) * 0x10) + 0x14)
#define DEFAULT_SLAVE_STATIC_ADDR	0x74
#define DEFAULT_SLAVE_INST_ID		0x4

union i3c_global_reg1_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t force_i2c_slave : 1;		/* bit[0] */
		volatile uint32_t slave_test_mode : 1;		/* bit[1] */
		volatile uint32_t act_mode : 2;			/* bit[3:2] */
		volatile uint32_t pending_int : 4;		/* bit[7:4] */
		volatile uint32_t slave_static_addr : 7;	/* bit[14:8] */
		volatile uint32_t slave_static_addr_en : 1;	/* bit[15] */
		volatile uint32_t slave_inst_id : 4;		/* bit[19:16] */
		volatile uint32_t reserved : 12;		/* bit[31:20] */
	} fields;
};


static int i3c_global_init(const struct device *dev)
{
	const struct device *reset_dev = device_get_binding(ASPEED_RST_CTRL_NAME);
	union i3c_global_reg1_s reg;
	uint32_t base = DEV_CFG(dev)->base;
	uint32_t ni3cs = DEV_CFG(dev)->ni3cs;
	int i;

	reset_control_deassert(reset_dev, DEV_CFG(dev)->rst_id);

	reg.value = 0;
	reg.fields.act_mode = 1;
	reg.fields.slave_static_addr = DEFAULT_SLAVE_STATIC_ADDR;
	reg.fields.slave_inst_id = DEFAULT_SLAVE_INST_ID;
	for (i = 0; i < ni3cs; i++) {
		sys_write32(reg.value, base + I3C_GLOBAL_REG1(i));
	}

	return 0;
}

static const struct i3c_global_config i3c_aspeed_config = {
	.base = DT_REG_ADDR(DT_NODELABEL(i3c_gr)),
	.rst_id = (reset_control_subsys_t)DT_INST_RESETS_CELL(0, rst_id),
	.ni3cs = DT_PROP(DT_NODELABEL(i3c_gr), ni3cs),
};


DEVICE_DT_INST_DEFINE(0, &i3c_global_init, NULL,
		      NULL, &i3c_aspeed_config,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
