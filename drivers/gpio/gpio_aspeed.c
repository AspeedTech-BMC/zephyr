/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast1030_gpio

#include <kernel.h>
#include <device.h>
#include <drivers/gpio.h>
#include <soc.h>

#include "gpio_utils.h"
#include "gpio_aspeed.h"

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_aspeed);

/* Driver config */
struct gpio_aspeed_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* GPIO controller base address */
	gpio_register_t *base;
	uint8_t pin_offset;
};

/* Driver data */
struct gpio_aspeed_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* list of callbacks */
	sys_slist_t cb;
};

uint16_t gpio_offset_data[] = {
	[0] = offsetof(gpio_register_t, group0_data),
	[1] = offsetof(gpio_register_t, group1_data),
	[2] = offsetof(gpio_register_t, group2_data),
	[3] = offsetof(gpio_register_t, group3_data),
	[4] = offsetof(gpio_register_t, group4_data),
	[5] = offsetof(gpio_register_t, group5_data),
	[6] = offsetof(gpio_register_t, group6_data),
};

uint16_t gpio_offset_write_latch[] = {
	[0] = offsetof(gpio_register_t, group0_rd_data),
	[1] = offsetof(gpio_register_t, group1_rd_data),
	[2] = offsetof(gpio_register_t, group2_rd_data),
	[3] = offsetof(gpio_register_t, group3_rd_data),
	[4] = offsetof(gpio_register_t, group4_rd_data),
	[5] = offsetof(gpio_register_t, group5_rd_data),
	[6] = offsetof(gpio_register_t, group6_rd_data),
};

uint16_t gpio_offset_int_status[] = {
	[0] = offsetof(gpio_register_t, group0_int_status),
	[1] = offsetof(gpio_register_t, group1_int_status),
	[2] = offsetof(gpio_register_t, group2_int_status),
	[3] = offsetof(gpio_register_t, group3_int_status),
	[4] = offsetof(gpio_register_t, group4_int_status),
	[5] = offsetof(gpio_register_t, group5_int_status),
	[6] = offsetof(gpio_register_t, group6_int_status),
};

#define ASPEED_GPIO_GROUP_NUM DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)
static int nr_isr_devs;
static const struct device *isr_devs[ASPEED_GPIO_GROUP_NUM];

/* Driver convenience defines */
#define DEV_CFG(dev) ((const struct gpio_aspeed_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_aspeed_data *)(dev)->data)

static void gpio_aspeed_init_cmd_src_sel(const struct device *dev)
{
	volatile gpio_register_t *gpio_reg = DEV_CFG(dev)->base;
	gpio_cmd_src_sel_t cmd_src_sel;

	cmd_src_sel.value = gpio_reg->cmd_src_sel.value;
	cmd_src_sel.fields.mst1 = ASPEED_GPIO_SEL_PRI;
	cmd_src_sel.fields.mst2 = ASPEED_GPIO_SEL_LPC;
	cmd_src_sel.fields.mst3 = ASPEED_GPIO_SEL_SSP;
	cmd_src_sel.fields.mst4 = ASPEED_GPIO_SEL_PRI;
	cmd_src_sel.fields.mst5 = ASPEED_GPIO_SEL_PRI;
	cmd_src_sel.fields.lock = 1;
	gpio_reg->cmd_src_sel.value = cmd_src_sel.value;
}

static int gpio_aspeed_cmd_src_set(const struct device *dev, gpio_pin_t pin, uint8_t cmd_src)
{
	volatile gpio_register_t *gpio_reg = DEV_CFG(dev)->base;
	uint8_t pin_offset = DEV_CFG(dev)->pin_offset;
	gpio_index_register_t index;

	if (pin >= 32) {
		LOG_ERR("Invalid gpio pin #%d", pin);
		return -EINVAL;
	}
	pin += pin_offset;
	index.value = 0;
	index.fields.index_type = ASPEED_GPIO_CMD_SRC;
	index.fields.index_command = ASPEED_GPIO_INDEX_WRITE;
	index.fields.index_number = pin;
	index.fields.index_data = cmd_src;
	gpio_reg->index.value = index.value;
	return 0;
}

static int gpio_aspeed_set_direction(const struct device *dev, gpio_pin_t pin, int direct)
{
	volatile gpio_register_t *gpio_reg = DEV_CFG(dev)->base;
	uint8_t pin_offset = DEV_CFG(dev)->pin_offset;
	gpio_index_register_t index;

	if (pin >= 32) {
		LOG_ERR("Invalid gpio pin #%d", pin);
		return -EINVAL;
	}
	pin += pin_offset;
	index.value = 0;
	index.fields.index_type = ASPEED_GPIO_DIRECTION;
	index.fields.index_command = ASPEED_GPIO_INDEX_WRITE;
	index.fields.index_data = direct;
	index.fields.index_number = pin;
	LOG_DBG("gpio index = 0x%08x\n", index.value);
	gpio_reg->index.value = index.value;
	return 0;
}

static void gpio_aspeed_isr(const void *unused)
{
	ARG_UNUSED(unused);
	const struct device *dev;
	struct gpio_aspeed_data *data;
	uint32_t index, group_idx;
	uint32_t gpio_pin, int_pendding;
	gpio_int_status_register_t *int_reg;
	for (index = 0; index < ASPEED_GPIO_GROUP_NUM; index++) {
		dev = isr_devs[index];
		data = DEV_DATA(dev);
		group_idx = DEV_CFG(dev)->pin_offset >> 5;
		int_reg = (gpio_int_status_register_t *)((uint32_t)DEV_CFG(dev)->base +
							 gpio_offset_int_status[group_idx]);
		int_pendding = int_reg->value;
		gpio_pin = 0;
		while (int_pendding) {
			if (int_pendding & 0x1) {
				gpio_fire_callbacks(&data->cb,
						    dev, BIT(gpio_pin));
				int_reg->value = BIT(gpio_pin);
			}
			gpio_pin++;
			int_pendding >>= 1;
		}
	}
}

/* GPIO api functions */
static int gpio_aspeed_port_get_raw(const struct device *dev,
				    gpio_port_value_t *value)
{
	uint8_t pin_offset = DEV_CFG(dev)->pin_offset;
	uint32_t group_idx = pin_offset >> 5;

	*value = sys_read32((uint32_t)DEV_CFG(dev)->base + gpio_offset_data[group_idx]);
	return 0;
}

static int gpio_aspeed_port_set_masked_raw(const struct device *dev,
					   gpio_port_pins_t mask,
					   gpio_port_value_t value)
{
	uint8_t pin_offset = DEV_CFG(dev)->pin_offset;
	uint32_t group_idx = pin_offset >> 5;
	uint32_t port_val;

	port_val = sys_read32((uint32_t)DEV_CFG(dev)->base + gpio_offset_write_latch[group_idx]);
	port_val = (port_val & ~mask) | (mask & value);
	sys_write32(port_val, (uint32_t)DEV_CFG(dev)->base + gpio_offset_data[group_idx]);
	return 0;
}

static int gpio_aspeed_port_set_bits_raw(const struct device *dev,
					 gpio_port_value_t mask)
{
	return gpio_aspeed_port_set_masked_raw(dev, mask, mask);
}

static int gpio_aspeed_port_clear_bits_raw(const struct device *dev,
					   gpio_port_value_t mask)
{
	return gpio_aspeed_port_set_masked_raw(dev, mask, 0);
}

static int gpio_aspeed_port_toggle_bits(const struct device *dev,
					gpio_port_value_t mask)
{
	uint8_t pin_offset = DEV_CFG(dev)->pin_offset;
	uint32_t group_idx = pin_offset >> 5;
	uint32_t port_val;

	port_val = sys_read32((uint32_t)DEV_CFG(dev)->base + gpio_offset_write_latch[group_idx]);
	port_val ^= mask;
	sys_write32(port_val, (uint32_t)DEV_CFG(dev)->base + gpio_offset_data[group_idx]);
	return 0;
}

static int gpio_aspeed_pin_interrupt_configure(const struct device *dev,
					       gpio_pin_t pin,
					       enum gpio_int_mode mode,
					       enum gpio_int_trig trig)
{
	volatile gpio_register_t *gpio_reg = DEV_CFG(dev)->base;
	uint8_t pin_offset = DEV_CFG(dev)->pin_offset;
	gpio_index_register_t index;
	uint8_t int_type, index_data;

	if (pin >= 32) {
		LOG_ERR("Invalid gpio pin #%d", pin);
		return -EINVAL;
	}
	pin += pin_offset;
	index.value = 0;

	if (mode == GPIO_INT_MODE_DISABLED) {
		index_data = 0;
	} else {
		if (mode == GPIO_INT_MODE_LEVEL) {
			if (trig == GPIO_INT_TRIG_LOW) {
				int_type = ASPEED_GPIO_LEVEL_LOW;
			} else if (trig == GPIO_INT_TRIG_HIGH) {
				int_type = ASPEED_GPIO_LEVEL_HIGH;
			} else {
				return -ENOTSUP;
			}
		} else {
			if (trig == GPIO_INT_TRIG_LOW) {
				int_type = ASPEED_GPIO_FALLING_EDGE;
			} else if (trig == GPIO_INT_TRIG_HIGH) {
				int_type = ASPEED_GPIO_RAISING_EDGE;
			} else {
				int_type = ASPEED_GPIO_DUAL_EDGE;
			}
		}
		index_data = BIT(0) | (int_type << 1);
	}

	index.fields.index_type = ASPEED_GPIO_INTERRUPT;
	index.fields.index_command = ASPEED_GPIO_INDEX_WRITE;
	index.fields.index_data = index_data;
	index.fields.index_number = pin;
	LOG_DBG("gpio index = 0x%08x\n", index.value);
	gpio_reg->index.value = index.value;

	return 0;
}

static int gpio_aspeed_manage_callback(const struct device *dev,
				       struct gpio_callback *callback, bool set)
{
	struct gpio_aspeed_data *data = DEV_DATA(dev);

	return gpio_manage_callback(&data->cb, callback, set);
}

static int gpio_aspeed_config(const struct device *dev,
			      gpio_pin_t pin, gpio_flags_t flags)
{
	int ret;
	uint32_t io_flags;

	/* Does not support disconnected pin, and
	 * not supporting both input/output at same time.
	 */
	io_flags = flags & (GPIO_INPUT | GPIO_OUTPUT);
	if ((io_flags == GPIO_DISCONNECTED)
	    || (io_flags == (GPIO_INPUT | GPIO_OUTPUT))) {
		return -ENOTSUP;
	}

	/* Does not support pull-up/pull-down */
	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0U) {
		return -ENOTSUP;
	}

	if (flags & GPIO_OUTPUT) {

		if (flags & GPIO_SINGLE_ENDED) {

			if (flags & GPIO_LINE_OPEN_DRAIN) {
				/* connect to ground or disconnected */
				ret = gpio_aspeed_port_set_bits_raw(dev, BIT(pin));
			} else {
				/* connect to power supply or disconnected */
				ret = gpio_aspeed_port_clear_bits_raw(dev, BIT(pin));
			}
		} else {

		}
		/* Set output pin initial value */
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			ret = gpio_aspeed_port_set_bits_raw(dev, BIT(pin));
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			ret = gpio_aspeed_port_clear_bits_raw(dev, BIT(pin));
		}
		/* Set pin direction to output */
		ret = gpio_aspeed_set_direction(dev, pin, 1);
	} else { /* Input */
		/* Set pin direction to input */
		ret = gpio_aspeed_set_direction(dev, pin, 0);
	}
	gpio_aspeed_cmd_src_set(dev, pin, ASPEED_GPIO_CMD_SRC_ARM);
	return 0;
}

/* GPIO driver registration */
static const struct gpio_driver_api gpio_aspeed_driver = {
	.pin_configure = gpio_aspeed_config,
	.port_get_raw = gpio_aspeed_port_get_raw,
	.port_set_masked_raw = gpio_aspeed_port_set_masked_raw,
	.port_set_bits_raw = gpio_aspeed_port_set_bits_raw,
	.port_clear_bits_raw = gpio_aspeed_port_clear_bits_raw,
	.port_toggle_bits = gpio_aspeed_port_toggle_bits,
	.pin_interrupt_configure = gpio_aspeed_pin_interrupt_configure,
	.manage_callback = gpio_aspeed_manage_callback,
};

int gpio_aspeed_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	gpio_aspeed_init_cmd_src_sel(dev);

	if (nr_isr_devs == 0) {

		IRQ_CONNECT(DT_IRQN(DT_PARENT(DT_DRV_INST(0))),
			    DT_IRQ(DT_PARENT(DT_DRV_INST(0)), priority),
			    gpio_aspeed_isr,
			    NULL,
			    0);
		irq_enable(DT_IRQN(DT_PARENT(DT_DRV_INST(0))));
	}

	// irq_enable(DT_INST_IRQN(0));
	isr_devs[nr_isr_devs++] = dev;

	return 0;
}
#define ASPEED_GPIO_DEVICE_INIT(inst)									      \
	static const struct gpio_aspeed_config gpio_aspeed_cfg_##inst = {				      \
		.common = {										      \
			.port_pin_mask =								      \
				GPIO_PORT_PIN_MASK_FROM_DT_INST(inst) & ~(DT_INST_PROP(inst, gpio_reserved)), \
		},											      \
		.base = (gpio_register_t *)DT_REG_ADDR(DT_PARENT(DT_DRV_INST(inst))),			      \
		.pin_offset = DT_INST_PROP(inst, pin_offset),						      \
	};												      \
													      \
	static struct gpio_aspeed_data gpio_aspeed_data_##inst;						      \
													      \
	DEVICE_DT_INST_DEFINE(inst, gpio_aspeed_init, device_pm_control_nop,				      \
			      &gpio_aspeed_data_##inst,							      \
			      &gpio_aspeed_cfg_##inst, POST_KERNEL,					      \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,					      \
			      &gpio_aspeed_driver);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_GPIO_DEVICE_INIT)