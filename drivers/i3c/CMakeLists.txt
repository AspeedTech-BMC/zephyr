# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_I3C_ASPEED	i3c_global_aspeed.c)
zephyr_library_sources_ifdef(CONFIG_I3C_ASPEED	i3c_aspeed.c)
