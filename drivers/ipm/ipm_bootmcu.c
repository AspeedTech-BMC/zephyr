/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_bootmcu_ipc

#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ipc, CONFIG_LOG_DEFAULT_LEVEL);

#define THREAD_STACK_SIZE		512
#define THREAD_PRIORITY			-1

#define IPC_MAX_MSG_SIZE 0x20
#define IPC_NUM_OF_ID	4
enum {
	IPC_ID_0 = 0,
	IPC_ID_1,
	IPC_ID_2,
	IPC_ID_CALIPTRA,
};

/* Each bit in the register represents an IPC ID */
#define IPCR_TRIG	0x0
#define IPCR_ENABLE	0x4
#define IPCR_STATUS	0x8

#define IPCR_DATA0	0x10
#define IPCR_DATA1	0x30
#define IPCR_DATA2	0x50
#define IPCR_DATA3	0x70

struct bootmcu_ipc_config {
	uintptr_t base;
	uintptr_t reg_tx_offset;
	uintptr_t reg_rx_offset;
};

struct bootmcu_ipc_state {
	bool in_use;
	struct k_thread thread_data;

	K_KERNEL_STACK_MEMBER(thread_stack, THREAD_STACK_SIZE);
	struct k_sem sem;
};

struct bootmcu_ipc_data {
	struct bootmcu_ipc_state state;
	ipm_callback_t callback[IPC_NUM_OF_ID];
	void *user_data[IPC_NUM_OF_ID];
};

static void ipc_thread(const void *dev)
{
	const struct bootmcu_ipc_data *data = ((const struct device *)dev)->data;
	const struct bootmcu_ipc_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base + config->reg_rx_offset;
	uint32_t msg_base, status;
	int i, ret;

	while (1) {
		status = sys_read32(base + IPCR_STATUS);
		for (i = 0; i < IPC_NUM_OF_ID; i++) {
			if ((status & BIT(i)) && data->callback[i]) {
				msg_base = base + IPCR_DATA0 + IPC_MAX_MSG_SIZE * i;
				data->callback[i](dev, data->user_data[i], i, (void *)msg_base);
				sys_write32(BIT(i), base + IPCR_STATUS);
			}
		}

		ret = k_msleep(100);
	}
}

static int ipc_send(const struct device *dev, int wait, uint32_t id, const void *data,
		    int size)
{
	const struct bootmcu_ipc_config *config = ((const struct device *)dev)->config;
	uintptr_t base  = config->base + config->reg_tx_offset;
	uint32_t reg;

	if (size > IPC_MAX_MSG_SIZE) {
		return -EMSGSIZE;
	}

	if (id >= IPC_NUM_OF_ID) {
		return -EINVAL;
	}

	reg = sys_read32(base + IPCR_TRIG);
	if (reg & BIT(id)) {
		return -EBUSY;
	}

	sys_write32(reg | BIT(id), base + IPCR_TRIG);

	if (wait) {
		while (sys_read32(base + IPCR_TRIG) & BIT(id)) {
			/* busy-wait */
		}
	}

	return 0;
}

static void ipc_register_id_callback(const struct device *dev, uint32_t id,
				     ipm_callback_t cb, void *user_data)
{
	struct bootmcu_ipc_data *data = dev->data;

	data->callback[id] = cb;
	data->user_data[id] = user_data;
}

static int ipc_max_data_size_get(const struct device *dev)
{
	return IPC_MAX_MSG_SIZE;
}

static uint32_t ipc_max_id_val_get(const struct device *dev)
{
	return IPC_NUM_OF_ID - 1;
}

static int ipc_set_id_enabled(const struct device *dev, uint32_t id, int enable)
{
	const struct bootmcu_ipc_config *config = dev->config;
	uint32_t reg = 0;

	reg = sys_read32(config->base + config->reg_rx_offset + IPCR_ENABLE);
	if (enable) {
		reg |= BIT(id);
	} else {
		reg &= ~BIT(id);
	}
	sys_write32(reg, config->base + config->reg_rx_offset + IPCR_ENABLE);

	return 0;
}

static int bootmcu_ipc_init(const struct device *dev)
{
	const struct bootmcu_ipc_config *config = ((const struct device *)dev)->config;
	struct bootmcu_ipc_data *data = ((struct device *)dev)->data;
	struct k_thread *thread_data = &data->state.thread_data;
	k_thread_stack_t *thread_stack = data->state.thread_stack;
	k_tid_t tid;

	printk("\n");
	LOG_INF("0x%x: Create threads to service ipc requests", (uint32_t)config->base);

	tid = k_thread_create(thread_data, thread_stack, THREAD_STACK_SIZE,
			      (k_thread_entry_t)ipc_thread, (void *)dev,
			      NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);

	/* Disabled by default */
	sys_write32(0x0, config->base + config->reg_rx_offset + IPCR_ENABLE);

	/* clear all un-finished interrupts */
	sys_write32(0xf, config->base + config->reg_rx_offset + IPCR_STATUS);

	return 0;
}

static const struct ipm_driver_api ipc_driver_api = {
	.send = ipc_send,
	.register_id_callback = ipc_register_id_callback,
	.max_data_size_get = ipc_max_data_size_get,
	.max_id_val_get = ipc_max_id_val_get,
	.set_id_enabled = ipc_set_id_enabled,
};

#define ASPEED_BOOTMCU_IPC_INIT(n)                                                                \
	static int bootmcu_ipc_config_func_##n(const struct device *dev);                          \
	static const struct bootmcu_ipc_config bootmcu_ipc_config_##n = {                          \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.reg_tx_offset = DT_INST_PROP(n, reg_tx_offset),                                   \
		.reg_rx_offset = DT_INST_PROP(n, reg_rx_offset),                                   \
	};                                                                                         \
	static struct bootmcu_ipc_data bootmcu_ipc_data_##n;                                       \
	DEVICE_DT_INST_DEFINE(n, &bootmcu_ipc_config_func_##n, NULL, &bootmcu_ipc_data_##n,        \
			      &bootmcu_ipc_config_##n, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ipc_driver_api);                \
	static int bootmcu_ipc_config_func_##n(const struct device *dev)                           \
	{                                                                                          \
		bootmcu_ipc_init(dev);                                                             \
		return 0;                                                                          \
	}
DT_INST_FOREACH_STATUS_OKAY(ASPEED_BOOTMCU_IPC_INIT)
