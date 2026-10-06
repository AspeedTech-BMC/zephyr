/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_PSP_RESET_NOTIFY_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_PSP_RESET_NOTIFY_H_

#include <zephyr/device.h>

/*
 * callback invoked from ISR context when the PSP reset interrupt fires
 * @dev: this psp_reset_notify device, pass back into
 *       psp_reset_notify_irq_disable()/_enable()
 * @user_data: pointer registered via psp_reset_notify_set_callback()
 */
typedef void (*psp_reset_notify_callback_t)(const struct device *dev, void *user_data);

/*
 * Register the callback invoked when the PSP reset interrupt fires.
 * The callback runs in ISR context. Passing cb == NULL unregisters it.
 */
void psp_reset_notify_set_callback(const struct device *dev, psp_reset_notify_callback_t cb,
				   void *user_data);

/* Mask the PSP reset interrupt. Safe to call from ISR context. */
void psp_reset_notify_irq_disable(const struct device *dev);

/* Unmask the PSP reset interrupt. Safe to call from ISR context. */
void psp_reset_notify_irq_enable(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_ASPEED_PSP_RESET_NOTIFY_H_ */
