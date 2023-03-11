/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_ast2700_ipc

#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_IPM_LOG_LEVEL
LOG_MODULE_REGISTER(ipm_ast2700);

#define IPC_NUM_OF_ID	4
#define IPC_MAX_MSG_SIZE 0x20

#define IPCR_TRIG	0x0
#define IPCR_ENABLE	0x4
#define IPCR_STATUS	0x8

#define IPCR_DATA0	0x10
#define IPCR_DATA1	0x30
#define IPCR_DATA2	0x50
#define IPCR_DATA3	0x70

struct ipm_ast2700_config {
	uintptr_t base;
	uintptr_t reg_tx_offset;
	uintptr_t reg_rx_offset;
};

struct ipm_ast2700_data {
	ipm_callback_t callback;
	void *user_data;
};

static void ipm_ast2700_isr(const void *dev)
{
	const struct ipm_ast2700_data *data = ((const struct device *)dev)->data;
	const struct ipm_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base + config->reg_rx_offset;
	uint32_t status = sys_read32(base + IPCR_STATUS);
	uint32_t msg_base;
	int i;

	LOG_DBG("ipc@%lx: isr status 0x%x\n", config->base, status);

	if (!data->callback)
		goto done;

	for (i = 0; i < IPC_NUM_OF_ID; i++) {
		if (status & BIT(i)) {
			msg_base = base + IPCR_DATA0 + IPC_MAX_MSG_SIZE * i;

			LOG_DBG("msg@%08x:", msg_base);
			LOG_HEXDUMP_DBG((void *)msg_base, IPC_MAX_MSG_SIZE, "msg");

			data->callback(dev, data->user_data, i, (volatile void *)msg_base);
		}
	}

done:
	sys_write32(status, base + IPCR_STATUS);
}

static int ipm_ast2700_send(const struct device *dev, int wait, uint32_t id, const void *data,
			    int size)
{
	const struct ipm_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base  = config->base + config->reg_tx_offset;
	uint32_t reg;

	if (size > IPC_MAX_MSG_SIZE) {
		return -EMSGSIZE;
	}

	if (wait) {
		LOG_WRN("not support wait mode for now\n");
	}

	reg = sys_read32(base + IPCR_TRIG);
	if (reg & BIT(id)) {
		return -EBUSY;
	}

	sys_write32(reg | BIT(id), base + IPCR_TRIG);

	return 0;
}

static void ipm_ast2700_register_callback(const struct device *dev, ipm_callback_t cb,
					  void *user_data)
{
	struct ipm_ast2700_data *data = dev->data;

	data->callback = cb;
	data->user_data = user_data;
}

static int ipm_ast2700_max_data_size_get(const struct device *dev)
{
	return IPC_MAX_MSG_SIZE;
}

static uint32_t ipm_ast2700_max_id_val_get(const struct device *dev)
{
	return IPC_NUM_OF_ID - 1;
}

static int ipm_ast2700_set_enabled(const struct device *dev, int enable)
{
	const struct ipm_ast2700_config *config = dev->config;
	uint32_t reg = 0;

	if (enable) {
		reg = 0xf;
	}
	sys_write32(reg, config->base + config->reg_rx_offset + IPCR_ENABLE);

	return 0;
}

static int ipm_ast2700_init(const struct device *dev)
{
	const struct ipm_ast2700_config *config = dev->config;

	/* Enabled by default just for now.  This should be controlled by api->set_enabled */
	sys_write32(0xf, config->base + config->reg_rx_offset + IPCR_ENABLE);

	/* clear all un-finished interrupts */
	sys_write32(0xf, config->base + config->reg_rx_offset + IPCR_STATUS);

	return 0;
}

static const struct ipm_driver_api ipm_ast2700_driver_api = {
	.send = ipm_ast2700_send,
	.register_callback = ipm_ast2700_register_callback,
	.max_data_size_get = ipm_ast2700_max_data_size_get,
	.max_id_val_get = ipm_ast2700_max_id_val_get,
	.set_enabled = ipm_ast2700_set_enabled,
};

#define IPM_AST2700_INIT(n)                                                                        \
	static int ipm_ast2700_config_func_##n(const struct device *dev);                          \
	static const struct ipm_ast2700_config ipm_ast2700_config_##n = {                          \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.reg_tx_offset = DT_INST_PROP(n, reg_tx_offset),                                   \
		.reg_rx_offset = DT_INST_PROP(n, reg_rx_offset),                                   \
	};                                                                                         \
	static struct ipm_ast2700_data ipm_ast2700_data_##n;                                       \
	DEVICE_DT_INST_DEFINE(n, &ipm_ast2700_config_func_##n, NULL, &ipm_ast2700_data_##n,        \
			      &ipm_ast2700_config_##n, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ipm_ast2700_driver_api);        \
	static int ipm_ast2700_config_func_##n(const struct device *dev)                           \
	{                                                                                          \
		ipm_ast2700_init(dev);                                                             \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), ipm_ast2700_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
		return 0;                                                                          \
	}
DT_INST_FOREACH_STATUS_OKAY(IPM_AST2700_INIT)