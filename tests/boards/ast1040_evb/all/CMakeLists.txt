# SPDX-License-Identifier: Apache-2.0
#
# Build the peripheral tests into one concurrent test image.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ast1040_evb_all)

add_definitions(-DAST1040_CONCURRENT_ALL)

set(PERIPH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_sources(app PRIVATE
  src/main.c
  ${PERIPH_DIR}/gpio/src/main.c
  ${PERIPH_DIR}/cptra/src/main.c
  ${PERIPH_DIR}/i2c/src/main.c
)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
