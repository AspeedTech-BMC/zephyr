/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_ast2600_ipc

#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#define LOG_LEVEL CONFIG_IPM_LOG_LEVEL
LOG_MODULE_REGISTER(ipm_ast2600);

#if defined(CONFIG_SOC_AST2600)
/* Secondary service processor ARM Cortex-M3. */
#define IPCR_TRIG		0x18
#define IPCR_STATUS		0x28
#define IPCR_CLEAR		0x2c
#elif defined(CONFIG_SOC_AST2700_SSP) || defined(CONFIG_SOC_AST2700_A1_SSP) || \
	defined(CONFIG_SOC_AST2705_SSP)
/* Secondary service processor ARM Cortex-M4. */
#define IPCR_EN			0x20
#define IPCR_TRIG		0x28
#define IPCR_STATUS		0x24
#define IPCR_CLEAR		0x24
#elif defined(CONFIG_SOC_AST2700_TSP) || defined(CONFIG_SOC_AST2700_A1_TSP) || \
	defined(CONFIG_SOC_AST2705_TSP)
/* Tertiary service processor ARM Cortex-M4. */
#define IPCR_EN			0x30
#define IPCR_TRIG		0x38
#define IPCR_STATUS		0x34
#define IPCR_CLEAR		0x34
#endif

#if defined(CONFIG_IPC_SHM_RX_SIZE) || defined(CONFIG_IPC_SHM_TX_SIZE)
/* shared memory for SSP communicating with PSP. */
uint8_t shm_rx[CONFIG_IPC_SHM_RX_SIZE] NON_CACHED_SHM_RX = {0};
uint8_t shm_tx[CONFIG_IPC_SHM_TX_SIZE] NON_CACHED_SHM_TX = {0};
#endif

struct ipm_ast2600_config {
	uint32_t base;
	uint32_t num_irqs;
#if defined(CONFIG_IPC_SHM_RX_SIZE) || defined(CONFIG_IPC_SHM_TX_SIZE)
	uint32_t shm_rx_size;
	uint32_t shm_tx_size;
#endif
};

struct ipm_ast2600_obj {
	ipm_callback_t callback;
	void *user_data;
};

#define DEV_DATA(dev)		((struct ipm_ast2600_obj *)(dev)->data)
#define DEV_CFG(dev)		((struct ipm_ast2600_config *)(dev)->config)

static void ipc_ast2600_isr(const void *dev)
{
	struct ipm_ast2600_obj *obj = DEV_DATA((const struct device *)dev);
	struct ipm_ast2600_config *config = DEV_CFG((const struct device *)dev);
	uint32_t base = config->base;
	uint32_t status = sys_read32(base + IPCR_STATUS);
	int i;

	LOG_DBG("isr status: %08x\n", status);

	if (!obj->callback) {
		goto finish;
	}

	for (i = 0; i < config->num_irqs; i++) {
		if (status & BIT(i)) {
			obj->callback(dev, obj->user_data, i, NULL);
		}
	}

finish:
	sys_write32(status, base + IPCR_CLEAR);
}

static int ipm_ast2600_send(const struct device *dev, int wait, uint32_t id, const void *data,
			    int size)
{
	struct ipm_ast2600_config *config = DEV_CFG(dev);
	uint32_t base = config->base;
#if defined(CONFIG_IPC_SHM_RX_SIZE) || defined(CONFIG_IPC_SHM_TX_SIZE)
	uint32_t shm_tx_size = config->shm_tx_size;
#endif
	uint32_t status = sys_read32(base + IPCR_STATUS);
	uint32_t ret = 0;

	if (status & BIT(id)) {
		ret = -EBUSY;
		goto finish;
	}

	if (id >= config->num_irqs) {
		ret = -EINVAL;
		goto finish;
	}

#if defined(CONFIG_IPC_SHM_RX_SIZE) || defined(CONFIG_IPC_SHM_TX_SIZE)
	if (size > shm_tx_size) {
		ret = -EMSGSIZE;
		goto finish;
	}

	/* Copy message data to shared memory. */
	memcpy((void *)shm_tx, data, size);
#endif

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

static uint32_t ipm_ast2600_max_id_val_get(const struct device *dev)
{
	struct ipm_ast2600_config *config = DEV_CFG(dev);

	return config->num_irqs;
}

static int ipm_ast2600_max_data_size_get(const struct device *dev)
{
#if defined(CONFIG_IPC_SHM_RX_SIZE) || defined(CONFIG_IPC_SHM_TX_SIZE)
	struct ipm_ast2600_config *config = DEV_CFG(dev);

	/* share memory include TX and RX data. */
	return config->shm_rx_size;
#else
	ARG_UNUSED(dev);
	return 0;
#endif
}

static int ipm_ast2600_set_enabled(const struct device *dev, int enable)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(enable);
	return 0;
}

static void ipm_ast2600_register_callback(const struct device *dev, ipm_callback_t cb,
					  void *user_data)
{
	struct ipm_ast2600_obj *obj = DEV_DATA(dev);

	obj->user_data = user_data;
	obj->callback = cb;
}

static int ipm_ast2600_init(const struct device *dev)
{
#if defined(CONFIG_IPC_SHM_RX_SIZE) || defined(CONFIG_IPC_SHM_TX_SIZE)
	struct ipm_ast2600_config *config = DEV_CFG(dev);

	config->shm_rx_size = CONFIG_IPC_SHM_RX_SIZE;
	config->shm_tx_size = CONFIG_IPC_SHM_TX_SIZE;
	printk("[SSP] Clear shared memory.\n");
	printk("RX:%p, size:0x%x\n", shm_rx, CONFIG_IPC_SHM_RX_SIZE);
	printk("TX:%p, size:0x%x\n", shm_tx, CONFIG_IPC_SHM_TX_SIZE);
	/* clear shared memory for SSP communicating with PSP. */
	memset(shm_rx, 0, CONFIG_IPC_SHM_RX_SIZE);
	memset(shm_tx, 0, CONFIG_IPC_SHM_TX_SIZE);
#endif

#if defined(CONFIG_SOC_AST2700_SSP) || defined(CONFIG_SOC_AST2700_A1_SSP) || \
	defined(CONFIG_SOC_AST2700_TSP) || defined(CONFIG_SOC_AST2700_A1_TSP) || \
	defined(CONFIG_SOC_AST2705_SSP) || defined(CONFIG_SOC_AST2705_TSP)
	sys_write32(0xff, config->base + IPCR_EN);
#endif

	return 0;
}

static const struct ipm_driver_api ipm_ast2600_driver_api = {
	.send = ipm_ast2600_send,
	.register_callback = ipm_ast2600_register_callback,
	.max_data_size_get = ipm_ast2600_max_data_size_get,
	.max_id_val_get = ipm_ast2600_max_id_val_get,
	.set_enabled = ipm_ast2600_set_enabled,
};

#define IPM_AST2600_IRQ_CONNECT(index, inst)                                                       \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, index),                                      \
			    DT_INST_IRQ_BY_IDX(inst, index, priority), ipc_ast2600_isr,            \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN_BY_IDX(inst, index));                                      \
	} while (false);

#if defined(CONFIG_SOC_AST2600)
#define IPM_BASE_ADDR(inst) DT_INST_REG_ADDR(inst)
#elif defined(CONFIG_SOC_AST2700) || defined(CONFIG_SOC_AST2700_A1) || \
	defined(CONFIG_SOC_AST2705)
#define IPM_BASE_ADDR(inst) DT_REG_ADDR(DT_INST_PARENT(inst)) + DT_INST_REG_ADDR(inst)
#endif

#define IPM_AST2600_INIT(n)                                                                        \
	static int ipm_ast2600_config_func_##n(const struct device *dev);                          \
	static const struct ipm_ast2600_config ipm_ast2600_config_##n = {                          \
		.base = IPM_BASE_ADDR(n),                                                          \
		.num_irqs = DT_NUM_IRQS(DT_DRV_INST(n))                                            \
	};                                                                                         \
	static struct ipm_ast2600_obj ipm_ast2600_obj_##n;                                         \
	DEVICE_DT_INST_DEFINE(n, &ipm_ast2600_config_func_##n, NULL, &ipm_ast2600_obj_##n,         \
			      &ipm_ast2600_config_##n, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ipm_ast2600_driver_api);        \
	static int ipm_ast2600_config_func_##n(const struct device *dev)                           \
	{                                                                                          \
		LISTIFY(DT_NUM_IRQS(DT_DRV_INST(n)), IPM_AST2600_IRQ_CONNECT, (), n);              \
		ipm_ast2600_init(dev);                                                             \
		return 0;                                                                          \
	}
DT_INST_FOREACH_STATUS_OKAY(IPM_AST2600_INIT)
