/*
 * Copyright (c) 2026 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_UART_ROUTING_ASPEED_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_UART_ROUTING_ASPEED_H_

#include <zephyr/device.h>

/**
 * @brief Feed a peer's output into a UART controller or I/O pin's input.
 *
 * Note the direction: @p target is the source. For example,
 * name = "io0", target = "uart1" wires uart1's TX output into io0's RX
 * input, not the other way around.
 *
 * @param dev uart-routing device
 * @param name the selector to change, e.g. "uart0" or "io1"
 * @param target the peer whose output @p name should receive, e.g. "io0" or "uart1"
 *
 * @retval 0 on success
 * @retval -EINVAL if @p name is not a valid selector, or @p target is not
 *         a valid peer for that selector
 */
int uart_routing_aspeed_set(const struct device *dev, const char *name, const char *target);

/**
 * @brief Get which peer's output currently feeds a UART controller or I/O pin.
 *
 * @param dev uart-routing device
 * @param name the selector to query, e.g. "uart0" or "io1"
 * @param target set to a pointer to the current source peer's name on
 *        success, e.g. "io0" (i.e. @p name currently receives io0's output).
 *        This points at static storage; do not free it.
 *
 * @retval 0 on success
 * @retval -EINVAL if @p name is not a valid selector
 * @retval -ERANGE if the register field's current value doesn't match any
 *         known setting for @p name
 */
int uart_routing_aspeed_get(const struct device *dev, const char *name, const char **target);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_UART_ROUTING_ASPEED_H_ */
