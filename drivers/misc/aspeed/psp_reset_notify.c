/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_ast2700_ssp_psp_reset_notify

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/psp_reset_notify.h>

LOG_MODULE_REGISTER(psp_reset_notify, CONFIG_MISC_ASPEED_LOG_LEVEL);

struct psp_reset_notify_config {
	uint32_t irqn;
};

struct psp_reset_notify_data {
	psp_reset_notify_callback_t cb;
	void *user_data;
};

static void psp_reset_notify_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct psp_reset_notify_config *config = dev->config;
	struct psp_reset_notify_data *data = dev->data;

	/*
	 * Mask on entry: the caller re-arms via psp_reset_notify_irq_enable()
	 * once it has decided PSP reset has been released, not before.
	 */
	irq_disable(config->irqn);

	if (data->cb) {
		data->cb(dev, data->user_data);
	} else {
		LOG_WRN("PSP reset interrupt received, no callback registered");
	}
}

void psp_reset_notify_set_callback(const struct device *dev, psp_reset_notify_callback_t cb,
				   void *user_data)
{
	struct psp_reset_notify_data *data = dev->data;
	unsigned int key = irq_lock();

	data->cb = cb;
	data->user_data = user_data;
	irq_unlock(key);
}

void psp_reset_notify_irq_disable(const struct device *dev)
{
	const struct psp_reset_notify_config *config = dev->config;

	irq_disable(config->irqn);
}

void psp_reset_notify_irq_enable(const struct device *dev)
{
	const struct psp_reset_notify_config *config = dev->config;

	irq_enable(config->irqn);
}

/*
 * Left masked: the application arms it via psp_reset_notify_irq_enable()
 * once it has registered a callback and is ready to handle the event.
 */
#define PSP_RESET_NOTIFY_INIT(n)                                                                   \
	static struct psp_reset_notify_data psp_reset_notify_data_##n;                             \
	static const struct psp_reset_notify_config psp_reset_notify_config_##n = {                \
		.irqn = DT_INST_IRQN(n),                                                           \
	};                                                                                         \
	static int psp_reset_notify_init_##n(const struct device *dev)                             \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), psp_reset_notify_isr,       \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(n, psp_reset_notify_init_##n, NULL, &psp_reset_notify_data_##n,      \
			      &psp_reset_notify_config_##n, POST_KERNEL,                           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(PSP_RESET_NOTIFY_INIT)
