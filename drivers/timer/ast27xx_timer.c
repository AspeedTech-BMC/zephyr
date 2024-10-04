/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_ast27xx_timer

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/timer/aspeed_timer.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <string.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(timer_aspeed, LOG_LEVEL_DBG);

/* timer registers */
#define TIMER_COUNTER_STATUS		0x0
#define TIMER_RELOAD_VALUE		0x4
#define TIMER_1ST_MATCHING		0x8
#define TIMER_2ND_MATCHING		0xc
#define TIMER_CTRL			0x10
#define   TIMER_CTRL_INTR_STATUS	BIT(16)
#define   TIMER_CTRL_WDT_RESET		BIT(3)
#define   TIMER_CTRL_OVFL_INT		BIT(2)
#define   TIMER_CTRL_CLKSEL		BIT(1)
#define   TIMER_CTRL_EN			BIT(0)
#define TIMER_CTRL_CLR			0x14

struct timer_aspeed_obj {
	uint32_t tick_per_microsec;
	void (*callback)(void *user_data);
	void *user_data;
	bool auto_reload;
};

struct timer_aspeed_config {
	uintptr_t base;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_id;
};

#define DEV_CFG(dev)			((struct timer_aspeed_config *)(dev)->config)
#define DEV_DATA(dev)			((struct timer_aspeed_obj *)(dev)->data)

int timer_aspeed_stop(const struct device *dev)
{
	struct timer_aspeed_config *config = DEV_CFG(dev);

	/* Write CTRL_CLR to clear CTRL bits */
	sys_write32(TIMER_CTRL_EN | TIMER_CTRL_OVFL_INT | TIMER_CTRL_INTR_STATUS,
		    config->base + TIMER_CTRL_CLR);

	/* clear reload value then the HW will clear counter automatically */
	sys_write32(0, config->base + TIMER_RELOAD_VALUE);

	return 0;
}

int timer_aspeed_start(const struct device *dev, struct aspeed_timer_user_config *user_config)
{
	struct timer_aspeed_obj *obj = DEV_DATA(dev);
	struct timer_aspeed_config *config = DEV_CFG(dev);
	uint32_t reload, reg;

	/* suspend timer */
	timer_aspeed_stop(dev);

	/* configure timer, not support 1st & 2nd matching value for now */
	reload = user_config->millisec * obj->tick_per_microsec * 1000;
	sys_write32(reload, config->base + TIMER_RELOAD_VALUE);
	sys_write32(0xffffffff, config->base + TIMER_1ST_MATCHING);
	sys_write32(0xffffffff, config->base + TIMER_2ND_MATCHING);
	LOG_DBG("reload: %d ms, %d tick\n", user_config->millisec, reload);

	obj->callback = user_config->callback;
	obj->user_data = user_config->user_data;
	obj->auto_reload = (user_config->timer_type == ASPEED_TIMER_TYPE_ONE_SHOT) ? 0 : 1;

	/* enable timer */
	reg = TIMER_CTRL_EN | TIMER_CTRL_OVFL_INT;
	sys_write32(reg, config->base + TIMER_CTRL);

	return 0;
}

int timer_aspeed_query(const struct device *dev)
{
	struct timer_aspeed_config *config = DEV_CFG(dev);

	return sys_read32(config->base + TIMER_COUNTER_STATUS);
}

/*
 * if (1st matching && 2nd matching)
 *         overflow interrupt is controlled by TMR30
 * else
 *         overflow interrupt is enabled
 */
static void timer_aspeed_isr(const struct device *dev)
{
	struct timer_aspeed_obj *obj = DEV_DATA(dev);
	struct timer_aspeed_config *config = DEV_CFG(dev);

	/* disable timer if not auto-reload */
	if (!obj->auto_reload) {
		timer_aspeed_stop(dev);
	}

	if (obj->callback) {
		obj->callback(obj->user_data);
	}

	sys_write32(TIMER_CTRL_INTR_STATUS, config->base + TIMER_CTRL_CLR);
}

static int timer_aspeed_init(const struct device *dev)
{
	struct timer_aspeed_config *config = DEV_CFG(dev);
	struct timer_aspeed_obj *obj = DEV_DATA(dev);
	uint32_t clock_freq;

	clock_control_get_rate(config->clock_dev, config->clock_id, &clock_freq);
	obj->tick_per_microsec = clock_freq / 1000000;

	return 0;
}

#define TIMER_ASPEED_INIT(n)                                                                       \
	static int timer_aspeed_config_func_##n(const struct device *dev);                         \
	static const struct timer_aspeed_config timer_aspeed_config_##n = {                        \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, clk_id),                \
	};                                                                                         \
												   \
	static struct timer_aspeed_obj timer_aspeed_obj_##n;                                       \
												   \
	DEVICE_DT_INST_DEFINE(n, &timer_aspeed_config_func_##n, NULL, &timer_aspeed_obj_##n,       \
			      &timer_aspeed_config_##n, POST_KERNEL,                               \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);                           \
												   \
	static int timer_aspeed_config_func_##n(const struct device *dev)                          \
	{                                                                                          \
		timer_aspeed_init(dev);                                                            \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), timer_aspeed_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
		return 0;                                                                          \
	}

DT_INST_FOREACH_STATUS_OKAY(TIMER_ASPEED_INIT)
