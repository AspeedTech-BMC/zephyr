# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# to eliminate CMake warning
zephyr_library_sources(dummy.c)

zephyr_library_sources_ifdef(CONFIG_SNOOP_ASPEED		snoop_aspeed.c)
zephyr_library_sources_ifdef(CONFIG_MAILBOX_ASPEED		mbox_aspeed.c)
zephyr_library_sources_ifdef(CONFIG_PCC_ASPEED			pcc_aspeed.c)
