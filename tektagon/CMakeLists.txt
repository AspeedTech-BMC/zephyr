# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(ast1060_evb_full_sample)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

target_include_directories(
    app
    PRIVATE
    ${CERBERUS_ROOT}/core
    ${CERBERUS_ROOT}/projects/zephyr
    ${TEKTAGON_ROOT}/src/include
)

target_compile_options(
    app
    PRIVATE
    -fno-builtin
    -fdata-sections
    -Wall
    -Wextra
    # -Werror
    -Wno-unused-parameter
    -g -ggdb3
)