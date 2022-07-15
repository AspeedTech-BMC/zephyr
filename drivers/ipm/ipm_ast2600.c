/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_ast2600_ipc

#include <drivers/ipm.h>
#include <sys/util.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <sys/sys_io.h>
#include <logging/log.h>
#define LOG_LEVEL CONFIG_IPM_LOG_LEVEL
LOG_MODULE_REGISTER(ipm_ast2600);

#ifdef CONFIG_CPU_CORTEX_M3
/* secondary service processor ARM Cortex-M3 */
#define IPCR_TRIG		0x18
#define IPCR_STATUS		0x28
#define IPCR_CLEAR		0x2c
#else
/* Primary service processor ARM Cortex-A7 */
#define IPCR_TRIG		0x28
#define IPCR_STATUS		0x18
#define IPCR_CLEAR		0x1c
#endif

struct ipm_ast2600_config {
	uint32_t base;
	uint32_t num_irqs;
	int *irq_list;
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
	uint32_t status;
	int i;

	status = sys_read32(base + IPCR_STATUS);

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
	ARG_UNUSED(data);
	ARG_UNUSED(size);

	struct ipm_ast2600_config *config = DEV_CFG(dev);
	uint32_t base = config->base;
	uint32_t reg;

	if (id >= config->num_irqs) {
		return -EINVAL;
	}

	reg = sys_read32(base + IPCR_TRIG);
	if (reg & BIT(id)) {
		return -EBUSY;
	}

	sys_write32(BIT(id), base + IPCR_TRIG);
	if (wait) {
		do {
			k_busy_wait(100);
			reg = sys_read32(base + IPCR_TRIG);
		} while (reg & BIT(id));
	}

	return 0;
}

static uint32_t ipm_ast2600_max_id_val_get(const struct device *dev)
{
	struct ipm_ast2600_config *config = DEV_CFG(dev);

	return config->num_irqs;
}

static int ipm_ast2600_max_data_size_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
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
	struct ipm_ast2600_config *config = DEV_CFG(dev);

	int i, irq, priority;

	/*
	 * Assume "#interrupt-cells = <2>;"
	 * irq_list[0] = irq number of irq#0
	 * irq_list[1] = priority of irq#0
	 * irq_list[2] = irq number of irq#1
	 * irq_list[3] = priority of irq#1
	 * ...
	 * irq_list[2*n] = irq number of irq#n
	 * irq_list[2*n + 1] = priority of irq#n
	 */
	for (i = 0; i < 2 * config->num_irqs; i = i + 2) {
		irq = config->irq_list[i];
		priority = config->irq_list[i + 1];
		irq_connect_dynamic(irq, priority, ipc_ast2600_isr, dev, 0);
		irq_enable(irq);
	}

	return 0;
}

static const struct ipm_driver_api ipm_ast2600_driver_api = {
	.send = ipm_ast2600_send,
	.register_callback = ipm_ast2600_register_callback,
	.max_data_size_get = ipm_ast2600_max_data_size_get,
	.max_id_val_get = ipm_ast2600_max_id_val_get,
	.set_enabled = ipm_ast2600_set_enabled,
};

#define IPM_AST2600_IRQ_LIST(node_id, prop, idx) DT_PROP_BY_IDX(node_id, prop, idx),

#define IPM_AST2600_INIT(n)                                                                        \
	static int ipm_ast2600_config_func_##n(const struct device *dev);                          \
	static int ipm_ast2600_config_irq_list_##n[] = { DT_FOREACH_PROP_ELEM(                     \
		DT_DRV_INST(n), interrupts, IPM_AST2600_IRQ_LIST) };                               \
	static const struct ipm_ast2600_config ipm_ast2600_config_##n = {                          \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.num_irqs = DT_NUM_IRQS(DT_DRV_INST(n)),                                           \
		.irq_list = ipm_ast2600_config_irq_list_##n,                                       \
	};                                                                                         \
	static struct ipm_ast2600_obj ipm_ast2600_obj_##n;                                         \
	DEVICE_DT_INST_DEFINE(n, &ipm_ast2600_config_func_##n, NULL, &ipm_ast2600_obj_##n,         \
			      &ipm_ast2600_config_##n, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ipm_ast2600_driver_api);        \
	static int ipm_ast2600_config_func_##n(const struct device *dev)                           \
	{                                                                                          \
		ipm_ast2600_init(dev);                                                             \
		return 0;                                                                          \
	}
DT_INST_FOREACH_STATUS_OKAY(IPM_AST2600_INIT)
