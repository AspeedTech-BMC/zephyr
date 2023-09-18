# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# to eliminate CMake warning
zephyr_library_sources(dummy.c)

zephyr_library_sources_ifdef(CONFIG_SNOOP_ASPEED		snoop_aspeed.c)
zephyr_library_sources_ifdef(CONFIG_MAILBOX_ASPEED		mbox_aspeed.c)
zephyr_library_sources_ifdef(CONFIG_PCC_ASPEED			pcc_aspeed.c)
zephyr_library_sources_ifdef(CONFIG_OTP_ASPEED			otp_aspeed.c)
zephyr_library_sources_ifdef(CONFIG_SHELL_CMDS_OTP		otp_shell.c)

zephyr_library_link_libraries_ifdef(CONFIG_MBEDTLS mbedTLS)
