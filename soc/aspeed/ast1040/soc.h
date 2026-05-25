/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2023 ASPEED Technology Inc.
 */

#ifndef ZEPHYR_SOC_ASPEED_AST27XX_SOC_H_
#define ZEPHYR_SOC_ASPEED_AST27XX_SOC_H_
#include <aspeed_util.h>

#ifdef CONFIG_ARM
#include <cmsis_core_m_defaults.h>
#endif /* ARM */

uintptr_t ast10x0_g2_soc_phy_addr_to_virt_addr(uint64_t addr);
uint64_t ast10x0_g2_soc_virt_addr_to_phy_addr(uintptr_t addr);
#define TO_PHY_ADDR(addr)		ast10x0_g2_soc_virt_addr_to_phy_addr(addr)
#define TO_VIR_ADDR(addr)		ast10x0_g2_soc_phy_addr_to_virt_addr(addr)

#endif /* ZEPHYR_SOC_ASPEED_AST27XX_SOC_H_*/
