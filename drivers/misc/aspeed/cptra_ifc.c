/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_ifc

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/cptra.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(cptra_ifc, CONFIG_LOG_DEFAULT_LEVEL);

#define THREAD_STACK_SIZE		512
#define THREAD_PRIORITY			-1

/* Device config */
struct cptra_ifc_config {
	uintptr_t base;			/* Caliptra ifc base address */
};

struct cptra_ifc_drv_state {
	bool in_use;
	struct k_thread thread_data;

	K_KERNEL_STACK_MEMBER(thread_stack, THREAD_STACK_SIZE);
	struct k_sem sem;
};

#define DEV_CFG(dev)				\
	((struct cptra_ifc_config *)		\
	(dev)->config)

#define DEV_DATA(dev)				\
	((struct cptra_ifc_drv_state *)		\
	(dev)->data)

static void cptra_trng_thread(const struct device *dev)
{
	struct cptra_ifc_config *cfg = DEV_CFG(dev);
	uint32_t ts;
	int ret;

	while (1) {
		ts = sys_read32(cfg->base + CPTRA_TRNG_STS);

		if ((ts & CPTRA_TRNG_STS_DATA_REQ)) {
			for (int i = 0; i < CPTRA_MAX_TRNG; ++i)
				sys_write32(sys_rand32_get(), cfg->base + CPTRA_TRNG_DATA(i));
			sys_write32(CPTRA_TRNG_STS_DATA_WR_DONE, cfg->base +
				    CPTRA_TRNG_STS);
			continue;
		}

		ret = k_msleep(10);
	}
}

int cptra_ifc_init(const struct device *dev)
{
	struct cptra_ifc_config *cfg = DEV_CFG(dev);
	struct cptra_ifc_drv_state *state = DEV_DATA(dev);
	k_tid_t tid;

	LOG_INF("0x%x: Create threads to service trng requests", (uint32_t)cfg->base);

	tid = k_thread_create(&state->thread_data, state->thread_stack, THREAD_STACK_SIZE,
			      (k_thread_entry_t)cptra_trng_thread, (void *)dev,
			      NULL, NULL, THREAD_PRIORITY,
			      0, K_NO_WAIT);

#ifdef CONFIG_THREAD_NAME
	int ret = k_thread_name_set(tid, "cptra-ifc");

	if (ret)
		LOG_ERR("set thread name failed, ret:0x%x\n", ret);
#endif
	return 0;
}

static const struct cptra_ifc_config cptra_ifc_config = {
	.base = DT_REG_ADDR(DT_DRV_INST(0)),
};

static struct cptra_ifc_drv_state cptra_ifc_state;

#define ASPEED_CPTRA_IFC_INIT(inst)						\
	DEVICE_DT_INST_DEFINE(inst, cptra_ifc_init, NULL,			\
		      &cptra_ifc_state, &cptra_ifc_config,			\
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
		      NULL);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_CPTRA_IFC_INIT)
