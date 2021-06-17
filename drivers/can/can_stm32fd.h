/*
 * Copyright (c) 2020 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_DRIVERS_CAN_STM32FD_H_
#define ZEPHYR_DRIVERS_CAN_STM32FD_H_

#include "can_mcan.h"
#include <pinmux/stm32/pinmux_stm32.h>

#define DEV_DATA(dev) ((struct can_stm32fd_data *)(dev)->data)
#define DEV_CFG(dev) ((const struct can_stm32fd_config *)(dev)->config)

struct can_stm32fd_config {
	struct can_mcan_msg_sram *msg_sram;
	void (*config_irq)(void);
	struct can_mcan_config mcan_cfg;
	/* CAN always has an RX and TX pin. Hence, hardcode it to two*/
	const struct soc_gpio_pinctrl pinctrl[2];
};

struct can_stm32fd_data {
	struct can_mcan_data mcan_data;
};

#endif /*ZEPHYR_DRIVERS_CAN_STM32FD_H_*/
