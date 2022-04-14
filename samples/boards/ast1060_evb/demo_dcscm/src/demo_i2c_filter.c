/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/i2c.h>
#include <drivers/i2c/pfr/i2c_filter.h>

void aspeed_dcscm_i2c_flt_demo(void)
{
#if CONFIG_I2C_PFR_FILTER
	const struct device *pfr_flt_dev[3] = {NULL, NULL, NULL};
	int ret = 0, i = 0;

	/* get for dc-scm i2c flt */
	pfr_flt_dev[0] = device_get_binding("I2C_FILTER_0");
	if (!pfr_flt_dev[0]) {
		printk("I2C PFR : I2C FLT Device 0 not found.");
		return;
	}

	pfr_flt_dev[1] = device_get_binding("I2C_FILTER_1");
	if (!pfr_flt_dev[1]) {
		printk("I2C PFR : I2C FLT Device 1 not found.");
		return;
	}

	pfr_flt_dev[2] = device_get_binding("I2C_FILTER_2");
	if (!pfr_flt_dev[2]) {
		printk("I2C PFR : I2C FLT Device 2 not found.");
		return;
	}

	/* initial i2c filter as all block setting */
	for (i = 0; i < 3; i++) {
		if (pfr_flt_dev[i] != NULL) {
			ret = ast_i2c_filter_init(pfr_flt_dev[i]);
			if (ret) {
				printk("I2C PFR : FLT Device %d Initial failed.", i);
				return;
			}

			ret = ast_i2c_filter_en(pfr_flt_dev[i], 1, 1, 0, 0);
			if (ret) {
				printk("I2C PFR : FLT Device %d Enable / Disable failed.", i);
				return;
			}

			ret = ast_i2c_filter_default(pfr_flt_dev[i], 0);
			if (ret) {
				printk("I2C PFR : FLT Device %d Set Default failed.", i);
				return;
			}
		}
	}
#endif
}
