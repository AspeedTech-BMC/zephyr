# Copyright (c) 2023 ASPEED Technology Inc.
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_sources(soc.c)
zephyr_include_directories(.)
zephyr_linker_sources(ROM_START SORT_KEY 0x1sboot sboot.ld)

string(TOUPPER "${SOC_NAME}" soc_name_upper)
#zephyr_include_directories(${ZEPHYR_BASE}/soc/arm/common/cortex_m)
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
