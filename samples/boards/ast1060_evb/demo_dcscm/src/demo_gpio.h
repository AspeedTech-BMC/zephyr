/*
 * Copyright (c) 2021 - 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DEMO_GPIO_H__
#define __DEMO_GPIO_H__

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#if DT_NODE_HAS_STATUS(DT_INST(0, demo_gpio_basic_api), okay)
#define RST_IND_NUM 4

#define BMC_RSTIND_GPIO_DEV \
			DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(DT_INST(0, demo_gpio_basic_api), \
			bmc_rst_ind_in_gpios, 0))
#define BMC_RSTIND_GPIO_PIN_IN \
			DT_GPIO_PIN_BY_IDX(DT_INST(0, demo_gpio_basic_api), \
			bmc_rst_ind_in_gpios, 0)

#define BIC_RSTIND_GPIO_DEV \
			DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(DT_INST(0, demo_gpio_basic_api), \
			bic_rst_ind_in_gpios, 0))
#define BIC_RSTIND_GPIO_PIN_IN \
			DT_GPIO_PIN_BY_IDX(DT_INST(0, demo_gpio_basic_api), \
			bic_rst_ind_in_gpios, 0)

#define CPU0_RSTIND_GPIO_DEV \
			DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(DT_INST(0, demo_gpio_basic_api), \
			cpu0_rst_ind_in_gpios, 0))
#define CPU0_RSTIND_GPIO_PIN_IN \
			DT_GPIO_PIN_BY_IDX(DT_INST(0, demo_gpio_basic_api), \
			cpu0_rst_ind_in_gpios, 0)

#define CPU1_RSTIND_GPIO_DEV \
			DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(DT_INST(0, demo_gpio_basic_api), \
			cpu1_rst_ind_in_gpios, 0))
#define CPU1_RSTIND_GPIO_PIN_IN \
			DT_GPIO_PIN_BY_IDX(DT_INST(0, demo_gpio_basic_api), \
			cpu1_rst_ind_in_gpios, 0)
#else
#error Unsupported board
#endif

#endif /* __DEMO_GPIO_H__ */
