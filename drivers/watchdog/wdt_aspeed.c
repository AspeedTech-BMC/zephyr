/*
 * Copyright (c) 2021 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast_watchdog

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>
#include <drivers/clock_control.h>
#include <errno.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(wdt_ast, CONFIG_WDT_LOG_LEVEL);
#include <sys/sys_io.h>
#include <sys/__assert.h>
#include <sys/reboot.h>

#define WDT_RELOAD_VAL_REG		0x0004
#define WDT_RESTART_REG			0x0008
#define WDT_CTRL_REG			0x000C

#define WDT_RESET_MASK1_REG		0x001C
#define WDT_RESET_MASK2_REG		0x0020
#define WDT_SW_RESET_MASK1_REG	0x0028
#define WDT_SW_RESET_MASK2_REG	0x002C
#define WDT_SW_RESET_CTRL_REG	0x0024
#define WDT_TRIGGER_KEY			0xAEEDF123

#define WDT_SCU_SRST_ENABLE_REG	0x00D8
#define WDT_SCU_SRST_ENABLE_BIT	BIT(4)


#define WDT_RESTART_MAGIC		0x4755
#define WDT_CTRL_RST_MASK		GENMASK(6, 5)
#define WDT_CTRL_FULL_CHIP_RST	(BIT(5))
#define WDT_CTRL_ENABLE			GENMASK(1, 0)

struct aspeed_wdt_config {
	mm_reg_t ctrl_base;
	mm_reg_t scu_base;
	uint32_t rst_mask1;
	uint32_t rst_mask2;
};

void aspeed_wdt_reboot_sw(const struct device *dev, int type)
{
	const struct aspeed_wdt_config *config = dev->config;

	sys_write32(config->rst_mask1, config->ctrl_base + WDT_SW_RESET_MASK1_REG);
	sys_write32(config->rst_mask2, config->ctrl_base + WDT_SW_RESET_MASK2_REG);
	sys_write32(WDT_TRIGGER_KEY, config->ctrl_base + WDT_SW_RESET_CTRL_REG);
	ARG_UNUSED(type);
}

void aspeed_wdt_reboot_device(const struct device *dev, int type)
{
	const struct aspeed_wdt_config *config = dev->config;
	uint32_t ctrl_val;

	sys_write32(1, config->ctrl_base + WDT_RELOAD_VAL_REG);
	sys_write32(WDT_RESTART_MAGIC, config->ctrl_base + WDT_RESTART_REG);

	ctrl_val = sys_read32(config->ctrl_base + WDT_CTRL_REG);
	ctrl_val &= ~WDT_CTRL_RST_MASK;
	if (type == SYS_REBOOT_COLD) {
		ctrl_val |= WDT_CTRL_FULL_CHIP_RST;
		sys_write32(sys_read32(config->scu_base + WDT_SCU_SRST_ENABLE_REG) &
			~(WDT_SCU_SRST_ENABLE_BIT), config->scu_base + WDT_SCU_SRST_ENABLE_REG);
	}
	ctrl_val |= WDT_CTRL_ENABLE;
	sys_write32(ctrl_val, config->ctrl_base + WDT_CTRL_REG);
}

static int aspeed_wdt_init(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;

	sys_write32(config->rst_mask1, config->ctrl_base + WDT_SW_RESET_MASK1_REG);
	sys_write32(config->rst_mask2, config->ctrl_base + WDT_SW_RESET_MASK2_REG);

	return 0;
}

#define ASPEED_WDT_INIT(n)						\
	static struct aspeed_wdt_config aspeed_wdt_config_##n = { \
		.ctrl_base = DT_INST_REG_ADDR(n),	\
		.scu_base = DT_REG_ADDR_BY_IDX(DT_INST_PHANDLE_BY_IDX(n, aspeed_scu, 0), 0), \
		.rst_mask1 = DT_INST_PROP_BY_IDX(n, reset_mask, 0),	\
		.rst_mask2 = DT_INST_PROP_BY_IDX(n, reset_mask, 1),	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, &aspeed_wdt_init,			\
			    NULL,					\
			    NULL,			\
			    &aspeed_wdt_config_##n, POST_KERNEL,	\
			    80,		\
			    NULL);			\

DT_INST_FOREACH_STATUS_OKAY(ASPEED_WDT_INIT)
