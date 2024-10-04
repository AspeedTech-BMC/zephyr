# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_I2C_PFR_FILTER	i2c_filter.c)
zephyr_library_sources_ifdef(CONFIG_PFR_SW_MAILBOX	swmbx.c)

