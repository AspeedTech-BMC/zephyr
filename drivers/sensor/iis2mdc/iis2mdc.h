/* ST Microelectronics IIS2MDC 3-axis magnetometer sensor
 *
 * Copyright (c) 2020 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/iis2mdc.pdf
 */

#ifndef __MAG_IIS2MDC_H
#define __MAG_IIS2MDC_H

#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <sys/util.h>
#include "iis2mdc_reg.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */


union iis2mdc_bus_cfg {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	uint16_t i2c_slv_addr;
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_config spi_cfg;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
};

struct iis2mdc_dev_config {
	const struct device *bus;
	int (*bus_init)(const struct device *dev);
	const union iis2mdc_bus_cfg bus_cfg;
#ifdef CONFIG_IIS2MDC_TRIGGER
	const struct gpio_dt_spec gpio_drdy;
#endif  /* CONFIG_IIS2MDC_TRIGGER */
};

/* Sensor data */
struct iis2mdc_data {
	const struct device *dev;
	uint16_t i2c_addr;
	int16_t mag[3];
	int32_t temp_sample;

	stmdev_ctx_t *ctx;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	stmdev_ctx_t ctx_i2c;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	stmdev_ctx_t ctx_spi;
#endif

#ifdef CONFIG_IIS2MDC_TRIGGER
	struct gpio_callback gpio_cb;

	sensor_trigger_handler_t handler_drdy;

#if defined(CONFIG_IIS2MDC_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_IIS2MDC_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_IIS2MDC_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif  /* CONFIG_IIS2MDC_TRIGGER_GLOBAL_THREAD */
#endif  /* CONFIG_IIS2MDC_TRIGGER */
};

int iis2mdc_spi_init(const struct device *dev);
int iis2mdc_i2c_init(const struct device *dev);

#ifdef CONFIG_IIS2MDC_TRIGGER
int iis2mdc_init_interrupt(const struct device *dev);
int iis2mdc_trigger_set(const struct device *dev,
			  const struct sensor_trigger *trig,
			  sensor_trigger_handler_t handler);
#endif /* CONFIG_IIS2MDC_TRIGGER */

#endif /* __MAG_IIS2MDC_H */
