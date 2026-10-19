# Copyright (c) 2025 Aspeed Technology
#
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/ltpi.h)

zephyr_library()

zephyr_library_sources_ifdef(
  CONFIG_LTPI_AST27XX
  ltpi_ast27xx.c
)

zephyr_library_sources_ifdef(
  CONFIG_LTPI_AST10X0_G2
  ltpi_ast10x0_g2.c
)
