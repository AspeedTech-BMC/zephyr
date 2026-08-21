# Copyright (c) 2026 ASPEED Technology Inc.
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(espi_aspeed_v2_shell LANGUAGES C)

add_subdirectory(src)
