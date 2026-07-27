# SPDX-License-Identifier: Apache-2.0
#
# Combined AST1040 EVB board bring-up suite: one flash, one boot, every
# peripheral test runs concurrently in its own kernel thread - see
# src/main.c.
#
# Each peripheral's actual test logic (test_<name>(), using ast_zassert_* -
# see common/ast_test.h) lives once in its own
# tests/boards/ast1040_evb/<peripheral>/src/main.c and is referenced here,
# not duplicated. AST1040_CONCURRENT_ALL only skips that file's *trivial*
# standalone ZTEST()/ZTEST_SUITE() wrapper (a couple of lines calling
# test_<name>() once, sequentially) so the same hardware isn't exercised
# twice when building this combined image.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ast1040_evb_all)

add_definitions(-DAST1040_CONCURRENT_ALL)

set(PERIPH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_sources(app PRIVATE
  src/main.c
  ${PERIPH_DIR}/gpio/src/main.c
)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
