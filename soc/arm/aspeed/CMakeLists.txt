# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2021 ASPEED Technology Inc.

add_subdirectory(${SOC_SERIES})

zephyr_include_directories(.)

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND sh ${BOARD_DIR}/tools/gen_uart_booting_image.sh
  ${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.bin
  ${PROJECT_BINARY_DIR}/uart_${CONFIG_KERNEL_BIN_NAME}.bin
)