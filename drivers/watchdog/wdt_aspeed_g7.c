/*
 * Copyright (c) 2023 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast_watchdog_g7

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>

#include <zephyr/irq.h>
#include <errno.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_ast, CONFIG_WDT_LOG_LEVEL);
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/watchdog.h>

#define WDT_CTRL_REG_OFF            0x0080
#define WDT_CTRL_NUM                4

#define WDT_RELOAD_VAL_REG          0x0004
#define WDT_RESTART_REG             0x0008
#define WDT_CTRL_REG                0x000C
#define WDT_TIMEOUT_STATUS_REG      0x0010
#define WDT_TIMEOUT_STATUS_CLR_REG  0x0014
#define WDT_TRIGGER_KEY             0xAEEDF123

#define WDT_RESTART_MAGIC           0x4755
#define WDT_CTRL_RST_MASK           GENMASK(6, 5)
#define WDT_CTRL_FULL_CHIP_RST      BIT(5)
#define WDT_CTRL_RST_WDT_BY_SOC     BIT(4)
#define WDT_CTRL_INT_ENABLE         BIT(2)
#define WDT_CTRL_RST_SYS            BIT(1)
#define WDT_CTRL_ENABLE             BIT(0)

#define WDT_TIMEOUT_INDICATOR       BIT(0)
#define WDT_TIMEOUT_COUNTER_CLR     0x3B

/* 32-bit timer with 1MHz clock.
 * The maximum timeout ms is (2^32 - 1) / 10^6 * 1000.
 */
#define WDT_MAX_TIMEOUT_MS          4294900

struct aspeed_wdt_config {
	mm_reg_t ctrl_base;
};

struct aspeed_full_chip_reset {
	uint32_t scu_reg;
	uint32_t bit_mask;
};

struct aspeed_wdt_data {
	struct k_spinlock wdt_spin_lock;
	uint32_t rst_mask[8];
	uint32_t rst_mask_len;
	uint32_t rst_mask_off;
	uint32_t sw_rst_ctrl;
	uint32_t sw_rst_mask_off;
	bool timeout_installed;
	uint32_t timeout_counter;
	struct aspeed_full_chip_reset full_rst[2];
	wdt_callback_t callback;
	uint8_t wdt_timeout_sts;
};

void aspeed_wdt_full_rst_enable(const struct device *dev)
{
	struct aspeed_wdt_data *const data = dev->data;
	uint32_t i;
	uint32_t reg_val;

	for (i = 0; i < 2; i++) {
		if (data->full_rst[i].scu_reg == 0x0)
			continue;
		reg_val = sys_read32(data->full_rst[i].scu_reg);
		reg_val &= ~(data->full_rst[i].bit_mask);
		sys_write32(reg_val, data->full_rst[i].scu_reg);
	}
}

void aspeed_wdt_reset_mask_config(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	uint32_t i;

	for (i = 0; i < data->rst_mask_len; i++) {
		sys_write32(data->rst_mask[i],
			    config->ctrl_base + data->rst_mask_off + i * 4);
	}
}

void aspeed_wdt_sw_reset_mask_config(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	uint32_t i;

	for (i = 0; i < data->rst_mask_len; i++) {
		sys_write32(data->rst_mask[i],
			    config->ctrl_base + data->sw_rst_mask_off + i * 4);
	}
}

void aspeed_wdt_reset_mask_clear(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	uint32_t i;

	for (i = 0; i < data->rst_mask_len; i++)
		sys_write32(0x0, config->ctrl_base + data->rst_mask_off + i * 4);

	for (i = 0; i < data->rst_mask_len; i++)
		sys_write32(0x0, config->ctrl_base + data->sw_rst_mask_off + i * 4);
}

void aspeed_wdt_reboot_sw(const struct device *dev, int type)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->wdt_spin_lock);

	aspeed_wdt_sw_reset_mask_config(dev);

	sys_write32(WDT_TRIGGER_KEY, config->ctrl_base + data->sw_rst_ctrl);
	ARG_UNUSED(type);

	k_spin_unlock(&data->wdt_spin_lock, key);
}

void aspeed_wdt_reboot_device(const struct device *dev, int type)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	uint32_t ctrl_val;
	k_spinlock_key_t key = k_spin_lock(&data->wdt_spin_lock);

	/* disable wdt */
	ctrl_val = sys_read32(config->ctrl_base + WDT_CTRL_REG);
	ctrl_val &= ~WDT_CTRL_ENABLE;
	sys_write32(ctrl_val, config->ctrl_base + WDT_CTRL_REG);

	sys_write32(1, config->ctrl_base + WDT_RELOAD_VAL_REG);
	sys_write32(WDT_RESTART_MAGIC, config->ctrl_base + WDT_RESTART_REG);

	ctrl_val = sys_read32(config->ctrl_base + WDT_CTRL_REG);
	ctrl_val &= ~WDT_CTRL_RST_MASK;
	if (type == SYS_REBOOT_COLD) {
		ctrl_val |= WDT_CTRL_FULL_CHIP_RST;
		aspeed_wdt_full_rst_enable(dev);
	}
	ctrl_val |= (WDT_CTRL_RST_WDT_BY_SOC | WDT_CTRL_RST_SYS | WDT_CTRL_ENABLE);
	sys_write32(ctrl_val, config->ctrl_base + WDT_CTRL_REG);

	k_spin_unlock(&data->wdt_spin_lock, key);
}

static int wdt_aspeed_install_timeout(const struct device *dev,
			const struct wdt_timeout_cfg *cfg)
{
	int ret = 0;
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->wdt_spin_lock);

	if (sys_read32(config->ctrl_base + WDT_CTRL_REG) & WDT_CTRL_ENABLE) {
		ret = -EBUSY;
		goto end;
	}

	/* min lower limit should be 0 */
	if (cfg->window.min != 0 || cfg->window.max == 0 ||
		cfg->window.max > WDT_MAX_TIMEOUT_MS) {
		data->timeout_installed = false;
		ret = -EINVAL;
		goto end;
	}

	/* counter / (10^6) * 1000 = ms
	 * counter = ms * 1000
	 */
	data->timeout_counter = cfg->window.max * 1000;
	data->callback = cfg->callback;
	data->timeout_installed = true;

end:
	k_spin_unlock(&data->wdt_spin_lock, key);

	return ret;
}

static int wdt_aspeed_setup(const struct device *dev, uint8_t options)
{
	int ret = 0;
	struct aspeed_wdt_data *const data = dev->data;
	const struct aspeed_wdt_config *config = dev->config;
	uint32_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->wdt_spin_lock);

	if (!data->timeout_installed) {
		LOG_ERR("wdt, %s, has not been installed", dev->name);
		ret = -EINVAL;
		goto end;
	}

	if (sys_read32(config->ctrl_base + WDT_CTRL_REG) & WDT_CTRL_ENABLE) {
		ret = -EBUSY;
		goto end;
	}

	/* setup reload counter */
	sys_write32(data->timeout_counter, config->ctrl_base + WDT_RELOAD_VAL_REG);
	sys_write32(WDT_RESTART_MAGIC, config->ctrl_base + WDT_RESTART_REG);

	sys_write32(WDT_TIMEOUT_INDICATOR,
		config->ctrl_base + WDT_TIMEOUT_STATUS_CLR_REG);
	data->wdt_timeout_sts = 0;

	reg_val = sys_read32(config->ctrl_base + WDT_CTRL_REG);
	reg_val &= (~WDT_CTRL_RST_MASK & ~WDT_CTRL_RST_WDT_BY_SOC &
			~WDT_CTRL_RST_SYS);
	if (options == WDT_FLAG_RESET_NONE) {
		/* pure watchdog timer */
		reg_val |= WDT_CTRL_INT_ENABLE;
		aspeed_wdt_reset_mask_clear(dev);
	} else if (options == WDT_FLAG_RESET_CPU_CORE) {
		/* soc reset */
		reg_val |= WDT_CTRL_RST_WDT_BY_SOC | WDT_CTRL_RST_SYS;
		aspeed_wdt_reset_mask_config(dev);
	} else if (options == WDT_FLAG_RESET_SOC) {
		/* full chip reset */
		reg_val |= WDT_CTRL_FULL_CHIP_RST | WDT_CTRL_RST_WDT_BY_SOC | WDT_CTRL_RST_SYS;
		aspeed_wdt_reset_mask_config(dev);
		/* enable full chip reset SCU0D8[4] */
		aspeed_wdt_full_rst_enable(dev);
	} else {
		LOG_ERR("unsupported options: 0x%02x", options);
	}

	reg_val |= WDT_CTRL_ENABLE;
	sys_write32(reg_val, config->ctrl_base + WDT_CTRL_REG);

end:
	k_spin_unlock(&data->wdt_spin_lock, key);

	return ret;
}

static int wdt_aspeed_disable(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->wdt_spin_lock);

	/* reset to initial value */
	sys_write32(WDT_CTRL_RST_WDT_BY_SOC,
		config->ctrl_base + WDT_CTRL_REG);
	sys_write32(WDT_TIMEOUT_COUNTER_CLR << 1,
		config->ctrl_base + WDT_TIMEOUT_STATUS_CLR_REG);
	sys_write32(WDT_TIMEOUT_INDICATOR,
		config->ctrl_base + WDT_TIMEOUT_STATUS_CLR_REG);

	data->wdt_timeout_sts = 0;
	data->timeout_installed = false;

	k_spin_unlock(&data->wdt_spin_lock, key);

	return 0;
}

static int wdt_aspeed_feed(const struct device *dev, int channel_id)
{
	int ret = 0;
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->wdt_spin_lock);

	sys_write32(data->timeout_counter, config->ctrl_base + WDT_RELOAD_VAL_REG);
	sys_write32(WDT_RESTART_MAGIC, config->ctrl_base + WDT_RESTART_REG);

	k_spin_unlock(&data->wdt_spin_lock, key);
	ARG_UNUSED(channel_id);

	return ret;
}

bool get_wdt_timeout_status(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	uint32_t reg_val;

	reg_val = sys_read32(config->ctrl_base + WDT_TIMEOUT_STATUS_REG) &
				WDT_TIMEOUT_INDICATOR;

	return (reg_val & WDT_TIMEOUT_INDICATOR) || !!(data->wdt_timeout_sts);
}

void aspeed_wdt_platform_init(const struct device *dev)
{
	struct aspeed_wdt_data *const data = dev->data;

	memset(data->full_rst, 0x0, sizeof(struct aspeed_full_chip_reset) * 2);
	data->rst_mask_off = 0x34;
	data->sw_rst_ctrl = 0x30;
	data->sw_rst_mask_off = 0x50;
	data->timeout_installed = false;
	data->timeout_counter = 0;
	data->wdt_timeout_sts = 0;
	data->full_rst[0].scu_reg = 0x14c02010;
	data->full_rst[0].bit_mask = BIT(25);
	data->full_rst[1].scu_reg = 0x12c020c8;
	data->full_rst[1].bit_mask = BIT(7);
}

static int aspeed_wdt_init(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;
	uint32_t reg_val;

	/* disable WDT by default */
	reg_val = sys_read32(config->ctrl_base + WDT_CTRL_REG);
	reg_val &= ~WDT_CTRL_ENABLE;
	sys_write32(reg_val, config->ctrl_base + WDT_CTRL_REG);

	aspeed_wdt_platform_init(dev);

	return 0;
}

static const struct wdt_driver_api wdt_aspeed_driver_api = {
	.install_timeout = wdt_aspeed_install_timeout,
	.setup = wdt_aspeed_setup,
	.feed = wdt_aspeed_feed,
	.disable = wdt_aspeed_disable,
};

#define ASPEED_WDT_INIT(n)						\
	static struct aspeed_wdt_config aspeed_wdt_config_##n = {	\
		.ctrl_base = DT_REG_ADDR(DT_DRV_INST(n)),		\
	};								\
									\
	static struct aspeed_wdt_data aspeed_wdt_data_##n = {		\
		.rst_mask = DT_PROP(DT_DRV_INST(n), reset_mask),	\
		.rst_mask_len = DT_PROP_LEN(DT_DRV_INST(n), reset_mask), \
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, &aspeed_wdt_init,			\
			    NULL,					\
			    &aspeed_wdt_data_##n,			\
			    &aspeed_wdt_config_##n, POST_KERNEL,	\
			    81,						\
			    &wdt_aspeed_driver_api);			\

DT_INST_FOREACH_STATUS_OKAY(ASPEED_WDT_INIT)
