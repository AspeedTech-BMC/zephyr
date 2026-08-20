/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_ast2700_ipc

#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/ipm_ast.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
#include <zephyr/cache.h>

#define LOG_LEVEL CONFIG_IPM_LOG_LEVEL
LOG_MODULE_REGISTER(ipm_ast2700, LOG_LEVEL_ERR);

#define IPC_NUM_OF_ID	4
#define IPC_MAX_MSG_SIZE 0x20

/* Each bit in the register represents an IPC ID */
#define IPCR_TRIG	0x0
#define IPCR_ENABLE	0x4
#define IPCR_STATUS	0x8

#define IPCR_DATA0	0x10
#define IPCR_DATA1	0x30
#define IPCR_DATA2	0x50
#define IPCR_DATA3	0x70

struct ipm_ast2700_config {
	unsigned int irqn;
	uintptr_t base;
	uintptr_t reg_tx_offset;
	uintptr_t reg_rx_offset;
};

struct ipm_ast2700_shmem {
	uintptr_t shmem_tx_base;
	uintptr_t shmem_rx_base;
	unsigned int shmem_tx_size;
	unsigned int shmem_rx_size;
};

struct ipm_ast2700_data {
	void *user_data[IPC_NUM_OF_ID];
	ipm_callback_t callback[IPC_NUM_OF_ID];
	struct ipm_ast2700_shmem shmem_info[IPC_NUM_OF_ID]; /* share memory info */
};

/* list */
void ast_ipm_list(const struct device *dev)
{
	const struct ipm_ast2700_data *data = ((const struct device *)dev)->data;

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
	const struct ipm_ast2700_data *data = ((const struct device *)dev)->data;

	return data->shmem_info[channel].shmem_tx_size;
}

/* Write the tx shmem */
int ast_ipm_shmem_write(const struct device *dev, uint32_t channel,
uint32_t offset, const void *buf, uint32_t size)
{
	const struct ipm_ast2700_data *data = ((const struct device *)dev)->data;
	uintptr_t dst = 0;

	if (data->shmem_info[channel].shmem_tx_base) {
		if (data->shmem_info[channel].shmem_tx_size) {
			if (offset + size > data->shmem_info[channel].shmem_tx_size) {
				LOG_DBG("Write out of tx range");
				goto shmem_write_fail;
			} else {
				dst = data->shmem_info[channel].shmem_tx_base;
				memcpy((void *)(dst + offset), buf, size);
				sys_cache_data_flush_range((void *)(dst + offset), size);
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
	const struct ipm_ast2700_data *data = ((const struct device *)dev)->data;

	return data->shmem_info[channel].shmem_rx_size;
}

/* Read the rx shmem */
int ast_ipm_shmem_read(const struct device *dev, uint32_t channel,
uint32_t offset, void **buf, uint32_t size)
{
	const struct ipm_ast2700_data *data = ((const struct device *)dev)->data;
	uintptr_t src = 0;

	if (data->shmem_info[channel].shmem_rx_base) {
		if (data->shmem_info[channel].shmem_rx_size) {
			if (offset + size > data->shmem_info[channel].shmem_rx_size) {
				LOG_DBG("Read out of rx range");
				goto shmem_read_fail;
			} else {
				src = data->shmem_info[channel].shmem_rx_base + offset;
				*buf = (void *)src;
			}
		} else
			goto shmem_read_fail;
	} else
		goto shmem_read_fail;

	return size;

shmem_read_fail:
	return -EINVAL;
}

static void ipm_ast2700_isr(const void *dev)
{
	const struct ipm_ast2700_data *data = ((const struct device *)dev)->data;
	const struct ipm_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base = config->base + config->reg_rx_offset;
	uint32_t status = sys_read32(base + IPCR_STATUS);
	uint32_t msg_base;
	int i;

	for (i = 0; i < IPC_NUM_OF_ID; i++) {
		msg_base = base + IPCR_DATA0 + IPC_MAX_MSG_SIZE * i;
		if ((status & BIT(i)) && data->callback[i]) {
			if (data->shmem_info[i].shmem_rx_size) {
				/*
				 * The doorbell's first word carries the actual payload
				 * length; invalidate only that much instead of the whole
				 * configured shmem_rx_size on every interrupt.
				 */
				void *rx_base = (void *)data->shmem_info[i].shmem_rx_base;
				uint32_t msg_len = sys_read32(msg_base);
				uint32_t inv_len = MIN(msg_len, data->shmem_info[i].shmem_rx_size);

				if (inv_len)
					sys_cache_data_invd_range(rx_base, inv_len);
			}
			data->callback[i](dev, data->user_data[i], i, (volatile void *)msg_base);
			sys_write32(BIT(i), base + IPCR_STATUS);
		}
	}
}

static int ipm_ast2700_send(const struct device *dev, int wait, uint32_t id, const void *data,
			    int size)
{
	const struct ipm_ast2700_config *config = ((const struct device *)dev)->config;
	uintptr_t base  = config->base + config->reg_tx_offset;
	uint32_t status = sys_read32(base + IPCR_STATUS);
	uint32_t ret = 0;
	uint32_t i;

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
		} while (status & BIT(id));
	}

finish:
	return ret;
}

static void ipm_ast2700_register_id_callback(const struct device *dev, uint32_t id,
					     ipm_callback_t cb, void *user_data)
{
	struct ipm_ast2700_data *data = dev->data;

	data->callback[id] = cb;
	data->user_data[id] = user_data;
}

static int ipm_ast2700_max_data_size_get(const struct device *dev)
{
	return IPC_MAX_MSG_SIZE;
}

static uint32_t ipm_ast2700_max_id_val_get(const struct device *dev)
{
	return IPC_NUM_OF_ID - 1;
}

static int ipm_ast2700_set_id_enabled(const struct device *dev, uint32_t id, int enable)
{
	const struct ipm_ast2700_config *config = dev->config;
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

static int ipm_ast2700_init(const struct device *dev)
{
	const struct ipm_ast2700_config *config = dev->config;

	LOG_DBG("irqn=0x%x base=0x%lx", config->irqn, config->base);

	/* Disabled by default */
	sys_write32(0x0, config->base + config->reg_rx_offset + IPCR_ENABLE);

	/* clear all un-finished interrupts */
	sys_write32(0xf, config->base + config->reg_rx_offset + IPCR_STATUS);

	return 0;
}

static const struct ipm_driver_api ipm_ast2700_driver_api = {
	.send = ipm_ast2700_send,
	.register_id_callback = ipm_ast2700_register_id_callback,
	.max_data_size_get = ipm_ast2700_max_data_size_get,
	.max_id_val_get = ipm_ast2700_max_id_val_get,
	.set_id_enabled = ipm_ast2700_set_id_enabled,
};

#define IPM_AST2700_INIT(n)                                                                        \
	static int ipm_ast2700_config_func_##n(const struct device *dev);                          \
	static const struct ipm_ast2700_config ipm_ast2700_config_##n = {                          \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.irqn = DT_INST_IRQN(n),                                                           \
		.reg_tx_offset = DT_INST_PROP(n, reg_tx_offset),                                   \
		.reg_rx_offset = DT_INST_PROP(n, reg_rx_offset),                                   \
	};                                                                                         \
	struct ipm_ast2700_data ipm_ast2700_data_##n = {                       \
		.shmem_info[0] = DT_PROP_OR(DT_DRV_INST(n), shmem_ch0, {0}),  \
		.shmem_info[1] = DT_PROP_OR(DT_DRV_INST(n), shmem_ch1, {0}),  \
		.shmem_info[2] = DT_PROP_OR(DT_DRV_INST(n), shmem_ch2, {0}),  \
		.shmem_info[3] = DT_PROP_OR(DT_DRV_INST(n), shmem_ch3, {0}),  \
	};                                                                     \
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
