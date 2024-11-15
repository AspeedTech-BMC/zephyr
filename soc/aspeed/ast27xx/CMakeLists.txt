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
endif()

if(CONFIG_RISCV)
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
endif()
