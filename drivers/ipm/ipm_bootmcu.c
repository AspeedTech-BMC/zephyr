/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_bootmcu_ipc

#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/ipm_ast.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ipc, CONFIG_LOG_DEFAULT_LEVEL);

#define THREAD_STACK_SIZE		4096
#define THREAD_PRIORITY			0

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

struct bootmcu_ipc_state {
	bool in_use;
	struct k_thread thread_data;

	K_KERNEL_STACK_MEMBER(thread_stack, THREAD_STACK_SIZE);
	struct k_sem sem;
};

struct bootmcu_ipc_config {
	uintptr_t base;
	uintptr_t reg_tx_offset;
	uintptr_t reg_rx_offset;
};

struct bootmcu_ipc_shmem {
	uintptr_t shmem_tx_base;
	uintptr_t shmem_rx_base;
	unsigned int shmem_tx_size;
	unsigned int shmem_rx_size;
};

struct bootmcu_ipc_data {
	struct bootmcu_ipc_state state;
	void *user_data[IPC_NUM_OF_ID];
	ipm_callback_t callback[IPC_NUM_OF_ID];
	struct bootmcu_ipc_shmem shmem_info[IPC_NUM_OF_ID]; /* share memory info */
};

/* list */
void ast_ipm_list(const struct device *dev)
{
	const struct bootmcu_ipc_data *data = ((const struct device *)dev)->data;

	for (int i = 0 ; i < IPC_NUM_OF_ID; i++) {
		if (data->shmem_info[i].shmem_tx_base) {
			LOG_INF("Ch[%d] TX-SHMEM : 0x%08lx\n",
			i, data->shmem_info[i].shmem_tx_base);
			LOG_INF("Ch[%d] TX-MSIZE : 0x%08x\n\n",
			i, data->shmem_info[i].shmem_tx_size);
		}
		if (data->shmem_info[i].shmem_rx_base) {
			LOG_INF("Ch[%d] RX-SHMEM : 0x%08lx\n",
			i, data->shmem_info[i].shmem_rx_base);
			LOG_INF("Ch[%d] RX-MSIZE : 0x%08x\n\n",
			i, data->shmem_info[i].shmem_rx_size);
		}
	}
}

/* tx shmem information */
int ast_ipm_max_tx_shmem_size(const struct device *dev, uint32_t channel)
{
	const struct bootmcu_ipc_data *data = ((const struct device *)dev)->data;

	return data->shmem_info[channel].shmem_tx_size;
}

/* Write the tx shmem */
int ast_ipm_shmem_write(const struct device *dev, uint32_t channel,
uint32_t offset, const void *buf, uint32_t size)
{
	const struct bootmcu_ipc_data *data = ((const struct device *)dev)->data;
	uintptr_t dst = 0;

	if (data->shmem_info[channel].shmem_tx_base) {
		if (data->shmem_info[channel].shmem_tx_size) {
			if (offset + size > data->shmem_info[channel].shmem_tx_size) {
				LOG_DBG("Write out of tx range");
				goto shmem_write_fail;
			} else {
				dst = data->shmem_info[channel].shmem_tx_base;
				memcpy((void *)(dst + offset), buf, size);
			}
		} else
			goto shmem_write_fail;
	} else
		goto shmem_write_fail;

	return size;

shmem_write_fail:
	return -EINVAL;
}

/* rx shmem information */
int ast_ipm_max_rx_shmem_size(const struct device *dev, uint32_t channel)
{
	const struct bootmcu_ipc_data *data = ((const struct device *)dev)->data;

	return data->shmem_info[channel].shmem_rx_size;
}

/* Read the rx shmem */
int ast_ipm_shmem_read(const struct device *dev, uint32_t channel,
uint32_t offset, void *buf, uint32_t size)
{
	const struct bootmcu_ipc_data *data = ((const struct device *)dev)->data;
	uintptr_t src = 0;

	if (data->shmem_info[channel].shmem_rx_base) {
		if (data->shmem_info[channel].shmem_rx_size) {
			if (offset + size > data->shmem_info[channel].shmem_rx_size) {
				LOG_DBG("Read out of rx range");
				goto shmem_read_fail;
			} else {
				src = data->shmem_info[channel].shmem_rx_base;
				memcpy((void *)buf, (void *)(src + offset), size);
			}
		} else
			goto shmem_read_fail;
	} else
		goto shmem_read_fail;

	return size;

shmem_read_fail:
	return -EINVAL;
}

static void ipc_thread(const void *dev)
{
	const struct bootmcu_ipc_data *data = ((const struct device *)dev)->data;
	const struct bootmcu_ipc_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base + config->reg_rx_offset;
	uint32_t status, msg_base;
	int i, ret;

	while (1) {
		status = sys_read32(base + IPCR_STATUS);
		for (i = 0; i < IPC_NUM_OF_ID; i++) {
			if ((status & BIT(i)) && data->callback[i]) {
				msg_base = base + IPCR_DATA0 + IPC_MAX_MSG_SIZE * i;
				data->callback[i](dev, data->user_data[i], i, (void *)msg_base);
			}
			sys_write32(BIT(i), base + IPCR_STATUS);
		}

		ret = k_msleep(1);
	}
}

static int ipc_send(const struct device *dev, int wait, uint32_t id, const void *data,
		    int size)
{
	const struct bootmcu_ipc_config *config = ((const struct device *)dev)->config;
	uintptr_t base  = config->base + config->reg_tx_offset;
	uint32_t status = sys_read32(base + IPCR_STATUS);
	uint32_t ret = 0;
	uint32_t i;
	LOG_DBG("wait=0x%x, id=0x%x, size=0x%x\n", wait, id, size);

	if (status & BIT(id)) {
		ret = -EBUSY;
		goto finish;
	}

	if (id >= IPC_NUM_OF_ID) {
		ret = -EINVAL;
		goto finish;
	}

	if (size > IPC_MAX_MSG_SIZE) {
		ret = -EMSGSIZE;
		goto finish;
	}

	/* Copy message data to IPC Data registers. */
	for (i = 0; i < size / 4; i++) {
		sys_write32(((uint32_t *)data)[i],
			    base + IPCR_DATA0 + IPC_MAX_MSG_SIZE * id + i * 4);
	}

	/* Trigger IPC TX. */
	sys_write32(BIT(id), base + IPCR_TRIG);
	if (wait) {
		do {
			/* busy-wait for the status clean */
			status = sys_read32(base + IPCR_STATUS);
			LOG_DBG("status base:0x%lx, status: 0x%x\n", base + IPCR_STATUS, status);
		} while (status & BIT(id));
	}

finish:
	return ret;
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

	LOG_DBG("0x%x: Create threads to service ipc requests", (uint32_t)config->base);

	tid = k_thread_create(thread_data, thread_stack, THREAD_STACK_SIZE,
			      (k_thread_entry_t)ipc_thread, (void *)dev,
			      NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);

#ifdef CONFIG_THREAD_NAME
	int ret = k_thread_name_set(tid, "ipc");

	if (ret)
		LOG_ERR("set thread name failed, ret:0x%x\n", ret);
#endif

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

#define ASPEED_BOOTMCU_IPC_INIT(n)                                                                 \
	static int bootmcu_ipc_config_func_##n(const struct device *dev);                          \
	static const struct bootmcu_ipc_config bootmcu_ipc_config_##n = {                          \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.reg_tx_offset = DT_INST_PROP(n, reg_tx_offset),                                   \
		.reg_rx_offset = DT_INST_PROP(n, reg_rx_offset),                                   \
	};                                                                                         \
	struct bootmcu_ipc_data bootmcu_ipc_data_##n = {                      \
		.shmem_info[0] = DT_PROP_OR(DT_DRV_INST(n), shmem_ch0, {0}),  \
		.shmem_info[1] = DT_PROP_OR(DT_DRV_INST(n), shmem_ch1, {0}),  \
		.shmem_info[2] = DT_PROP_OR(DT_DRV_INST(n), shmem_ch2, {0}),  \
		.shmem_info[3] = DT_PROP_OR(DT_DRV_INST(n), shmem_ch3, {0}),  \
	};                                                                    \
	DEVICE_DT_INST_DEFINE(n, &bootmcu_ipc_config_func_##n, NULL, &bootmcu_ipc_data_##n,        \
			      &bootmcu_ipc_config_##n, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ipc_driver_api);                \
	static int bootmcu_ipc_config_func_##n(const struct device *dev)                           \
	{                                                                                          \
		bootmcu_ipc_init(dev);                                                             \
		return 0;                                                                          \
	}
DT_INST_FOREACH_STATUS_OKAY(ASPEED_BOOTMCU_IPC_INIT)
