/*
 * Copyright (c) 2023  ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_ASPEED_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_ASPEED_GPIO_H_

/**
 * @brief Enable GPIO pin debounce.
 *
 * The debounce flag is a Zephyr specific extension of the standard GPIO flags
 * specified by the Linux GPIO binding. Only applicable for ASPEED SoCs.
 */
#define ASPEED_GPIO_DEBOUNCE (1U << 8)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_ATMEL_SAM_GPIO_H_ */
