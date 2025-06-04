# Copyright (c) 2023 ASPEED Technology Inc.
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_include_directories(.)
zephyr_sources(soc.c)

if(CONFIG_ARM)
  if(CONFIG_SOC_AST2700)
    zephyr_library_sources_ifdef(CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER irq_init.c irq_manage.c)
  else()
    zephyr_library_sources_ifdef(CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER irq_init.c irq_manage_legacy.c)
  endif()
  zephyr_include_directories(${ZEPHYR_BASE}/soc/arm/common/cortex_m)
  zephyr_linker_sources(ROM_START SORT_KEY 0x1sboot sboot.ld)
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
  # The vector table must be placed at the start of SRAM
  zephyr_linker_sources_ifdef(CONFIG_SRAM_VECTOR_TABLE
  RAM_SECTIONS
  SORT_KEY 0
  sram_vector_table.ld
  )
endif()

if(CONFIG_RISCV)
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
  zephyr_library_link_libraries_ifdef(CONFIG_MBEDTLS mbedTLS)
  zephyr_sources(crypto.c)
endif()
