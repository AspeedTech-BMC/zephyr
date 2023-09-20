/**
 * @file
 *
 * @brief Generic low-level inter-processor mailbox communication API.
 */

/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTC_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTC_H_

/**
 * @brief IPM Interface
 * @defgroup intc_interface INTC Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @typedef intc_callback_t
 * @brief Callback API for incoming IPM messages
 *
 * These callbacks execute in interrupt context. Therefore, use only
 * interrupt-safe APIS. Registration of callbacks is done via
 * @a intc_register_callback
 *
 * @param intcdev Driver instance
 * @param user_data Pointer to some private data provided at registration
 *        time.
 */
typedef void (*intc_callback_t)(const struct device *intcdev, void *user_data);

/**
 * @typedef intc_register_callback_t
 * @brief Callback API upon registration
 *
 * See @a intc_register_callback() for argument definitions.
 */
typedef int (*intc_register_callback_t)(const struct device *port,
					intc_callback_t cb,
					void *user_data,
					int intc_bit);

/**
 * @typedef intc_set_mask_t
 * @brief Callback API upon mask of interrupts
 *
 * See @a intc_set_mask() for argument definitions.
 */
typedef int (*intc_set_irq_mask_t)(const struct device *intcdev, int index);
typedef int (*intc_set_irq_unmask_t)(const struct device *intcdev, int index);

__subsystem struct intc_driver_api {
	intc_register_callback_t register_callback;
	intc_set_irq_mask_t set_irq_mask;
	intc_set_irq_unmask_t set_irq_unmask;
};

/**
 * @brief Register a callback function for incoming messages.
 *
 * @param intcdev Driver instance pointer.
 * @param cb Callback function to execute on incoming message interrupts.
 * @param user_data Application-specific data pointer which will be passed
 *        to the callback function when executed.
 */
static inline void intc_register_callback(const struct device *intcdev,
					  intc_callback_t cb, void *user_data, int intc_bit)
{
	const struct intc_driver_api *api = (const struct intc_driver_api *)intcdev->api;

	api->register_callback(intcdev, cb, user_data, intc_bit);
}

/**
 * @typedef intc_set_mask_t
 * @brief Callback API upon mask of interrupts
 *
 * See @a intc_set_mask() for argument definitions.
 */
static inline int intc_set_irq_mask(const struct device *intcdev, int intc_bit)
{
	const struct intc_driver_api *api = (const struct intc_driver_api *)intcdev->api;

	return api->set_irq_mask(intcdev, intc_bit);
}

static inline int intc_set_irq_unmask(const struct device *intcdev, int intc_bit)
{
	const struct intc_driver_api *api = (const struct intc_driver_api *)intcdev->api;

	return api->set_irq_unmask(intcdev, intc_bit);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_INTC_H_ */
