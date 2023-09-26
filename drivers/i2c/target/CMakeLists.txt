# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_I2C_EEPROM_TARGET eeprom_target.c)
zephyr_library_sources_ifdef(CONFIG_I2C_IPMB_TARGET ipmb_target.c)
