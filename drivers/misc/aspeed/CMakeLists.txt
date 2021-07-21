# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SNOOP_ASPEED	snoop_aspeed.c)
