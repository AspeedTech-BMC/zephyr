/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT aspeed_cptra_mbox

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/llext/symbol.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/cptra.h>
#include <zephyr/drivers/misc/aspeed/cptra_mbox.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(cptra_mbox, CONFIG_LOG_DEFAULT_LEVEL);

/* Device config */
struct cptra_mbox_config {
	uintptr_t base;			/* Caliptra mbox base address */
};

struct cptra_mbox_drv_state {
	struct k_mutex cptra_mbox_mutex;
};

#define DEV_CFG(dev)				\
	((struct cptra_mbox_config *)		\
	(dev)->config)

#define DEV_DATA(dev)				\
	((struct cptra_mbox_drv_state *)	\
	(dev)->data)

static uint32_t cptra_mbox_base;
static struct k_mutex cptra_mbox_mutex;

static void cptra_mbox_set_base(uint32_t base)
{
	cptra_mbox_base = base;
}

void cptra_mbox_dump(void)
{
	for (int i = 0; i < 9; i++) {
		LOG_INF("0x%x: 0x%x", cptra_mbox_base + (i << 2),
			sys_read32(cptra_mbox_base + (i << 2)));
	}
}

int cptra_mbox_trigger(uint32_t cmd, uint32_t dlen, uint32_t csum,
		       uint8_t *input, uint32_t ilen,
		       uint8_t *output, uint32_t olen)
{
	uint32_t *p32;
	uint32_t sts;
	int rc;

	sys_write32(cmd, cptra_mbox_base + CPTRA_MBOX_CMD);
	sys_write32(sizeof(csum) + ilen, cptra_mbox_base + CPTRA_MBOX_DLEN);
	sys_write32(csum, cptra_mbox_base + CPTRA_MBOX_DATAIN);

	p32 = (uint32_t *)input;
	for (int i = 0; i < (ilen / sizeof(uint32_t)); i++)
		sys_write32(p32[i], cptra_mbox_base + CPTRA_MBOX_DATAIN);

	sys_write32(0x1, cptra_mbox_base + CPTRA_MBOX_EXEC);

	while (1) {
		sts = FIELD_GET(CPTRA_MBOX_STS_PS, cptra_mbox_status());
		if (sts != CPTRA_MBSTS_CMD_BUSY)
			break;
	}

	if (sts == CPTRA_MBSTS_DATA_READY) {
		dlen = sys_read32(cptra_mbox_base + CPTRA_MBOX_DLEN);
		LOG_INF("output dlen:0x%x", dlen);

		p32 = (uint32_t *)output;
		for (int i = 0; i < (dlen / sizeof(uint32_t)); i++)
			*p32++ = sys_read32(cptra_mbox_base + CPTRA_MBOX_DATAOUT);

		rc = 0;
	} else {
		cptra_mbox_dump();
		rc = -1;
	}

	return rc;
}

uint32_t cptra_mbox_csum(uint32_t csum, uint8_t *data, uint32_t dlen)
{
	uint32_t i;

	if (!data)
		return csum;

	for (i = 0; i < dlen; ++i)
		csum -= data[i];

	return csum;
}
EXPORT_SYMBOL(cptra_mbox_csum);

uint32_t cptra_mbox_status(void)
{
	return sys_read32(cptra_mbox_base + CPTRA_MBOX_STS);
}
EXPORT_SYMBOL(cptra_mbox_status);

int cptra_mbox_lock(void)
{
	uint32_t reg, sts;

	k_mutex_lock(&cptra_mbox_mutex, K_FOREVER);

	sts = cptra_mbox_status();
	if (sts & CPTRA_MBOX_STS_SOC_LOCK) {
		k_mutex_unlock(&cptra_mbox_mutex);
		return 0;
	}

	reg = sys_read32(cptra_mbox_base + CPTRA_MBOX_LOCK);

	return (reg) ? -EBUSY : 0;
}
EXPORT_SYMBOL(cptra_mbox_lock);

int cptra_mbox_unlock(void)
{
	uint32_t sts;
	uint32_t mb_sts;
	int ret;

	sts = sys_read32(cptra_mbox_base + CPTRA_MBOX_STS);
	if (!(sts & CPTRA_MBOX_STS_SOC_LOCK)) {
		ret = -EPERM;
		goto end;
	}

	mb_sts = FIELD_GET(CPTRA_MBOX_STS_PS, sts);
	if (mb_sts == CPTRA_MBSTS_CMD_BUSY) {
		ret = -EACCES;
		goto end;
	}

	sys_write32(0x0, cptra_mbox_base + CPTRA_MBOX_EXEC);

end:
	k_mutex_unlock(&cptra_mbox_mutex);
	return 0;
}
EXPORT_SYMBOL(cptra_mbox_unlock);

static int cptra_mbox_init(const struct device *dev)
{
	struct cptra_mbox_config *cfg = DEV_CFG(dev);

	cptra_mbox_set_base((uint32_t)cfg->base);

	k_mutex_init(&cptra_mbox_mutex);

	LOG_INF("0x%x: Initialized", (uint32_t)cfg->base);

	return 0;
}

static const struct cptra_mbox_config cptra_mbox_config = {
	.base = DT_REG_ADDR(DT_DRV_INST(0)),
};

DEVICE_DT_INST_DEFINE(0, cptra_mbox_init, NULL, NULL,
		      &cptra_mbox_config,
		      POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      NULL);
