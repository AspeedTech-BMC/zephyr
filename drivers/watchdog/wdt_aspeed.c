/*
 * Copyright (c) 2023 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
#define WDT_RST_MASK_NUM_MAX        5

#define WDT_RELOAD_VAL_REG          0x0004
#define WDT_RESTART_REG             0x0008
#define WDT_CTRL_REG                0x000C
#define WDT_TIMEOUT_STATUS_REG      0x0010
#define WDT_TIMEOUT_STATUS_CLR_REG  0x0014
#define WDT_TRIGGER_KEY             0xAEEDF123

#define WDT_SCU_FULL_RST_EN_REG     0x00D8
#define WDT_SCU_FULL_RST_EN         BIT(4)

#define WDT_2700_SCU_FULL_RST_EN_REG  0x00C8
#define WDT_2700_SCU0_FULL_RST_EN     BIT(9)
#define WDT_2700_SCU1_FULL_RST_EN        BIT(7)

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

/* AST1030 / AST1060 common (parent) device structs */
struct aspeed_wdt_common_config {
	mm_reg_t ctrl_base;
};

struct aspeed_wdt_common_data {
	struct k_spinlock wdt_irq_lock;
	const struct device *wdt_devs[WDT_CTRL_NUM];
	wdt_callback_t callbacks[WDT_CTRL_NUM];
	uint8_t wdt_timeout_sts[WDT_CTRL_NUM];
	uint32_t pre_wdt_timeout_idx;
};

struct aspeed_wdt_config {
	mm_reg_t ctrl_base;
	mm_reg_t scu0_base;
	mm_reg_t scu1_base;
	const struct device *parent;
	uint32_t ctrl_idx;
	/* common_irq_line: true (AST1030/AST1060): all WDT controllers share one IRQ line,
	 *   callback/timeout_sts are managed by the common parent device.
	 * common_irq_line: false (AST2700): each WDT has its own IRQ line,
	 *   callback/timeout_sts are managed locally in the child device.
	 */
	bool common_irq_line;
	bool has_irq;
	uint8_t rst_mask_num;
	uint32_t rst_mask_base_off;
	uint32_t sw_rst_ctrl_off;
	uint32_t sw_rst_mask_base_off;
};

struct aspeed_wdt_data {
	struct k_spinlock wdt_spin_lock;
	uint32_t rst_mask[WDT_RST_MASK_NUM_MAX];
	bool timeout_installed;
	uint32_t timeout_counter;
	wdt_callback_t callback;
	uint8_t wdt_timeout_sts;
};

static void aspeed_wdt_config_rst_masks(mm_reg_t ctrl_base, uint32_t base_off,
					uint8_t num, const uint32_t *masks)
{
	uint32_t i;

	for (i = 0; i < num; i++) {
		sys_write32(masks[i], ctrl_base + base_off + i * 4);
	}
}

void aspeed_wdt_reboot_sw(const struct device *dev, int type)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->wdt_spin_lock);

	aspeed_wdt_config_rst_masks(config->ctrl_base, config->sw_rst_mask_base_off,
				    config->rst_mask_num, data->rst_mask);
	sys_write32(WDT_TRIGGER_KEY, config->ctrl_base + config->sw_rst_ctrl_off);
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
		if (config->scu0_base) {
			sys_write32(sys_read32(config->scu0_base + WDT_2700_SCU_FULL_RST_EN_REG) &
				    ~(WDT_2700_SCU0_FULL_RST_EN),
				    config->scu0_base + WDT_2700_SCU_FULL_RST_EN_REG);
			sys_write32(sys_read32(config->scu1_base + WDT_2700_SCU_FULL_RST_EN_REG) &
				    ~(WDT_2700_SCU1_FULL_RST_EN),
				    config->scu1_base + WDT_2700_SCU_FULL_RST_EN_REG);
		} else {
			sys_write32(sys_read32(config->scu1_base + WDT_SCU_FULL_RST_EN_REG) &
				    ~(WDT_SCU_FULL_RST_EN),
				    config->scu1_base + WDT_SCU_FULL_RST_EN_REG);
		}
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

	if (config->common_irq_line) {
		struct aspeed_wdt_common_data *pd = config->parent->data;

		pd->callbacks[config->ctrl_idx - 1] = cfg->callback;
		pd->wdt_devs[config->ctrl_idx - 1] = dev;
	} else if (config->has_irq) {
		data->callback = cfg->callback;
	}
	data->timeout_installed = true;

end:
	k_spin_unlock(&data->wdt_spin_lock, key);

	return ret;
}

static int wdt_aspeed_setup(const struct device *dev, uint8_t options)
{
	int ret = 0;
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	uint32_t reg_val;
	uint8_t i;
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

	if (options == WDT_FLAG_RESET_NONE && !config->has_irq) {
		LOG_ERR("%s: WDT_FLAG_RESET_NONE requires interrupt support", dev->name);
		ret = -ENOTSUP;
		goto end;
	}

	/* setup reload counter */
	sys_write32(data->timeout_counter, config->ctrl_base + WDT_RELOAD_VAL_REG);
	sys_write32(WDT_RESTART_MAGIC, config->ctrl_base + WDT_RESTART_REG);
	sys_write32(WDT_TIMEOUT_INDICATOR, config->ctrl_base + WDT_TIMEOUT_STATUS_CLR_REG);

	if (config->common_irq_line) {
		struct aspeed_wdt_common_data *pd = config->parent->data;

		pd->wdt_timeout_sts[config->ctrl_idx - 1] = 0;
	} else {
		data->wdt_timeout_sts = 0;
	}

	reg_val = sys_read32(config->ctrl_base + WDT_CTRL_REG);
	reg_val &= (~WDT_CTRL_RST_MASK & ~WDT_CTRL_RST_WDT_BY_SOC &
			~WDT_CTRL_RST_SYS);
	if (options == WDT_FLAG_RESET_NONE) {
		/* pure watchdog timer */
		reg_val |= WDT_CTRL_INT_ENABLE;
		for (i = 0; i < config->rst_mask_num; i++) {
			sys_write32(0x0, config->ctrl_base + config->rst_mask_base_off + i * 4);
		}
	} else if (options == WDT_FLAG_RESET_CPU_CORE) {
		/* soc reset */
		reg_val |= WDT_CTRL_RST_WDT_BY_SOC | WDT_CTRL_RST_SYS;
		aspeed_wdt_config_rst_masks(config->ctrl_base, config->rst_mask_base_off,
					    config->rst_mask_num, data->rst_mask);
	} else if (options == WDT_FLAG_RESET_SOC) {
		/* full chip reset */
		reg_val |= WDT_CTRL_FULL_CHIP_RST | WDT_CTRL_RST_WDT_BY_SOC | WDT_CTRL_RST_SYS;
		aspeed_wdt_config_rst_masks(config->ctrl_base, config->rst_mask_base_off,
					    config->rst_mask_num, data->rst_mask);
		if (config->scu0_base) {
			sys_write32(sys_read32(config->scu0_base + WDT_2700_SCU_FULL_RST_EN_REG) &
				    ~(WDT_2700_SCU0_FULL_RST_EN),
				    config->scu0_base + WDT_2700_SCU_FULL_RST_EN_REG);
			sys_write32(sys_read32(config->scu1_base + WDT_2700_SCU_FULL_RST_EN_REG) &
				    ~(WDT_2700_SCU1_FULL_RST_EN),
				    config->scu1_base + WDT_2700_SCU_FULL_RST_EN_REG);
		} else {
			sys_write32(sys_read32(config->scu1_base + WDT_SCU_FULL_RST_EN_REG) &
				    ~(WDT_SCU_FULL_RST_EN),
				    config->scu1_base + WDT_SCU_FULL_RST_EN_REG);
		}
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

	if (config->common_irq_line) {
		struct aspeed_wdt_common_data *pd = config->parent->data;

		pd->wdt_timeout_sts[config->ctrl_idx - 1] = 0;
	} else {
		data->wdt_timeout_sts = 0;
	}
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
	const struct aspeed_wdt_data *const data = dev->data;
	uint8_t timeout_sts;
	uint32_t reg_val;

	reg_val = sys_read32(config->ctrl_base + WDT_TIMEOUT_STATUS_REG) &
			     WDT_TIMEOUT_INDICATOR;

	if (config->common_irq_line) {
		const struct aspeed_wdt_common_data *pd = config->parent->data;

		timeout_sts = pd->wdt_timeout_sts[config->ctrl_idx - 1];
	} else {
		timeout_sts = data->wdt_timeout_sts;
	}

	return (reg_val & WDT_TIMEOUT_INDICATOR) || (timeout_sts == 1);
}

static int __unused aspeed_wdt_init(const struct device *dev)
{
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	uint32_t reg_val;

	/* disable WDT by default */
	reg_val = sys_read32(config->ctrl_base + WDT_CTRL_REG);
	reg_val &= ~WDT_CTRL_ENABLE;
	sys_write32(reg_val, config->ctrl_base + WDT_CTRL_REG);

	aspeed_wdt_config_rst_masks(config->ctrl_base, config->rst_mask_base_off,
				    config->rst_mask_num, data->rst_mask);
	aspeed_wdt_config_rst_masks(config->ctrl_base, config->sw_rst_mask_base_off,
				    config->rst_mask_num, data->rst_mask);

	return 0;
}

void wdt_common_isr(const void *param)
{
	const struct device *dev = (const struct device *)param;
	const struct aspeed_wdt_common_config *config = dev->config;
	struct aspeed_wdt_common_data *const data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->wdt_irq_lock);
	uint32_t idx;
	uint32_t reg_val;
	uint32_t count = 0;

	/* round robin policy is used here in order to avoid
	 * interrupt event starvation.
	 */
	idx = data->pre_wdt_timeout_idx;
	while (count < WDT_CTRL_NUM) {
		if (idx >= WDT_CTRL_NUM)
			idx = 0;

		reg_val = sys_read32(config->ctrl_base + WDT_CTRL_REG_OFF * idx +
				     WDT_TIMEOUT_STATUS_REG);
		if ((reg_val & WDT_TIMEOUT_INDICATOR) != 0) {
			if (data->callbacks[idx])
				data->callbacks[idx](data->wdt_devs[idx], 0);
			else
				LOG_ERR("invalid callback function pointer");

			data->pre_wdt_timeout_idx++;
			/* ack */
			sys_write32(WDT_TIMEOUT_INDICATOR,
				    config->ctrl_base + WDT_CTRL_REG_OFF * idx +
				    WDT_TIMEOUT_STATUS_CLR_REG);
			data->wdt_timeout_sts[idx] = 1;
			goto end;
		}

		count++;
		idx++;
	}

	LOG_ERR("abnormal WDT interrupt");

end:
	k_spin_unlock(&data->wdt_irq_lock, key);
}

static void __unused wdt_aspeed_ast2700_isr(const void *param)
{
	const struct device *dev = (const struct device *)param;
	const struct aspeed_wdt_config *config = dev->config;
	struct aspeed_wdt_data *const data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->wdt_spin_lock);
	uint32_t reg_val;

	reg_val = sys_read32(config->ctrl_base + WDT_TIMEOUT_STATUS_REG);
	if ((reg_val & WDT_TIMEOUT_INDICATOR) != 0) {
		if (data->callback)
			data->callback(dev, 0);
		else
			LOG_ERR("invalid callback function pointer");

		sys_write32(WDT_TIMEOUT_INDICATOR,
			    config->ctrl_base + WDT_TIMEOUT_STATUS_CLR_REG);
		data->wdt_timeout_sts = 1;
	} else {
		LOG_ERR("abnormal WDT interrupt");
	}

	k_spin_unlock(&data->wdt_spin_lock, key);
}

static int __unused aspeed_wdt_common_init(const struct device *dev)
{
	struct aspeed_wdt_common_data *const data = dev->data;
	uint32_t i;

	for (i = 0; i < WDT_CTRL_NUM; i++) {
		data->wdt_timeout_sts[i] = 0;
	}

	data->pre_wdt_timeout_idx = 0;

	return 0;
}

static const struct wdt_driver_api wdt_aspeed_driver_api __unused = {
	.install_timeout = wdt_aspeed_install_timeout,
	.setup = wdt_aspeed_setup,
	.feed = wdt_aspeed_feed,
	.disable = wdt_aspeed_disable,
};

#define WDT_ENUM(node_id) node_id,

/* ============================================================
 * AST1030 / AST1060 (aspeed,ast-watchdog)
 * ============================================================
 */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast_watchdog

#define ASPEED_WDT_DEV_CFG(node_id) {	\
	.ctrl_base = DT_REG_ADDR(DT_PARENT(node_id)) + WDT_CTRL_REG_OFF *	\
				(DT_REG_ADDR(node_id) - 1),	\
	.scu0_base = 0,	\
	.scu1_base = DT_REG_ADDR_BY_IDX(DT_PHANDLE_BY_IDX(node_id, aspeed_scu1, 0), 0), \
	.parent = DEVICE_DT_GET(DT_PARENT(node_id)),	\
	.ctrl_idx = DT_REG_ADDR(node_id),	\
	.common_irq_line = true,	\
	.has_irq = true,	\
	.rst_mask_num = 2,	\
	.rst_mask_base_off = 0x001C,	\
	.sw_rst_mask_base_off = 0x0028,	\
	.sw_rst_ctrl_off = 0x0024,	\
},

#define ASPEED_WDT_DEV_DATA(node_id) {	\
	.rst_mask = {	\
		DT_PROP_BY_IDX(node_id, reset_mask, 0),	\
		DT_PROP_BY_IDX(node_id, reset_mask, 1),	\
	},	\
},

/* child node define */
#define ASPEED_WDT_DT_DEFINE(node_id)	\
	DEVICE_DT_DEFINE(node_id, aspeed_wdt_init,	\
					NULL,	\
					&aspeed_wdt_data[node_id],		\
					&aspeed_wdt_config[node_id],	\
					POST_KERNEL, 81, &wdt_aspeed_driver_api);

/* common node define */
#define ASPEED_WDT_IRQ_CONNECT(index, inst)                                                        \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, index),                                      \
			    DT_INST_IRQ_BY_IDX(inst, index, priority), wdt_common_isr,             \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN_BY_IDX(inst, index));                                      \
	} while (false);

#define ASPEED_WDT_COMMON_INIT(n)                                                                  \
	static int aspeed_wdt_config_func_##n(const struct device *dev);                           \
	static struct aspeed_wdt_common_config aspeed_wdt_common_config_##n = {                    \
		.ctrl_base = DT_INST_REG_ADDR(n),                                                  \
	};                                                                                         \
	static struct aspeed_wdt_common_data aspeed_wdt_common_data_##n;                           \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &aspeed_wdt_config_func_##n, NULL, &aspeed_wdt_common_data_##n,   \
			      &aspeed_wdt_common_config_##n, POST_KERNEL, 80, NULL);               \
	static int aspeed_wdt_config_func_##n(const struct device *dev)                            \
	{                                                                                          \
		aspeed_wdt_common_init(dev);                                                       \
		LISTIFY(DT_NUM_IRQS(DT_DRV_INST(n)), ASPEED_WDT_IRQ_CONNECT, (), n);               \
		return 0;                                                                          \
	}                                                                                          \
	/* handle child node */                                                                    \
	static const struct aspeed_wdt_config aspeed_wdt_config[] = {                              \
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n), ASPEED_WDT_DEV_CFG)};                 \
	static struct aspeed_wdt_data aspeed_wdt_data[] = {                                        \
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n), ASPEED_WDT_DEV_DATA)};                \
                                                                                                   \
	enum {                                                                                     \
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n), WDT_ENUM)                             \
	};                                                                                         \
	DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n), ASPEED_WDT_DT_DEFINE)

DT_INST_FOREACH_STATUS_OKAY(ASPEED_WDT_COMMON_INIT)

/* ============================================================
 * AST2700 (aspeed,ast2700-watchdog)
 * Each WDT controller is a standalone device node (no shared parent).
 * ============================================================
 */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast2700_watchdog

#define ASPEED_WDT_AST2700_DT_DEFINE(n)	\
	static int aspeed_wdt2700_init_##n(const struct device *dev)	\
	{	\
		aspeed_wdt_init(dev);	\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(n, interrupts),	\
			(IRQ_CONNECT(DT_INST_IRQN(n),	\
				     DT_INST_IRQ(n, priority),	\
				     wdt_aspeed_ast2700_isr,	\
				     DEVICE_DT_INST_GET(n), 0);	\
			 irq_enable(DT_INST_IRQN(n));),	\
			())	\
		return 0;	\
	}	\
	static const struct aspeed_wdt_config aspeed_wdt2700_config_##n = {	\
		.ctrl_base = DT_INST_REG_ADDR(n),	\
		.scu0_base = DT_REG_ADDR_BY_IDX(	\
				DT_PHANDLE_BY_IDX(DT_DRV_INST(n), aspeed_scu0, 0), 0),	\
		.scu1_base = DT_REG_ADDR_BY_IDX(	\
				DT_PHANDLE_BY_IDX(DT_DRV_INST(n), aspeed_scu1, 0), 0),	\
		.parent = NULL,	\
		.ctrl_idx = 0,	\
		.common_irq_line = false,	\
		.has_irq = DT_INST_NODE_HAS_PROP(n, interrupts),	\
		.rst_mask_num = 5,	\
		.rst_mask_base_off = 0x001C,	\
		.sw_rst_mask_base_off = 0x0034,	\
		.sw_rst_ctrl_off = 0x0030,	\
	};	\
	static struct aspeed_wdt_data aspeed_wdt2700_data_##n = {	\
		.rst_mask = {	\
			DT_INST_PROP_BY_IDX(n, reset_mask, 0),	\
			DT_INST_PROP_BY_IDX(n, reset_mask, 1),	\
			DT_INST_PROP_BY_IDX(n, reset_mask, 2),	\
			DT_INST_PROP_BY_IDX(n, reset_mask, 3),	\
			DT_INST_PROP_BY_IDX(n, reset_mask, 4),	\
		},	\
	};	\
	DEVICE_DT_INST_DEFINE(n, aspeed_wdt2700_init_##n,	\
			      NULL,	\
			      &aspeed_wdt2700_data_##n,	\
			      &aspeed_wdt2700_config_##n,	\
			      POST_KERNEL, 80, &wdt_aspeed_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_WDT_AST2700_DT_DEFINE)
