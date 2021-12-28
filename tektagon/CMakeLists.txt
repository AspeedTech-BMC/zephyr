# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(tektagon-oe)

set(TEKTAGON_ROOT ${CMAKE_CURRENT_LIST_DIR})

FILE(GLOB_RECURSE app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
set(CERBERUS_ROOT $ENV{ZEPHYR_BASE}/lib/cerberus)

#message(STATUS "Printing app_sources - ${app_sources}")

target_include_directories(
	app
	PRIVATE
		${CERBERUS_ROOT}/core
		${CERBERUS_ROOT}/projects/zephyr
		${TEKTAGON_ROOT}/src
		${AMI_PORT_ROOT}
)

target_compile_options(
	app
	PRIVATE
 		-fno-builtin
		-fdata-sections
		-Wall
		-Wextra
# 		-Werror
		-Wno-unused-parameter
		-g -ggdb3
)
