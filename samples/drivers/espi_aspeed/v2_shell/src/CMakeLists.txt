# Copyright (c) 2026 ASPEED Technology Inc.
# SPDX-License-Identifier: MIT

target_sources(app PRIVATE
	main.c
	oob_cmd.c
	mafs_cmd.c
	perif_cmd.c
)

if(CONFIG_ESPI_TAF AND CONFIG_ESPI_ASPEED_V2)
	target_sources(app PRIVATE safs_handler.c)
endif()
