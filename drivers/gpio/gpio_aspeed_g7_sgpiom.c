/*
 * Copyright (c) 2026 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_g7_sgpiom

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(g7_sgpiom_aspeed);

#define SGPIO_G7_IRQ_STS_BASE		0x40U
#define SGPIO_G7_IRQ_STS_OFFSET(x)	(SGPIO_G7_IRQ_STS_BASE + ((x) * 0x4U))
#define SGPIO_G7_CTRL_REG_BASE		0x80U
#define SGPIO_G7_CTRL_REG_OFFSET(x)	(SGPIO_G7_CTRL_REG_BASE + ((x) * 0x4U))

#define SGPIO_G7_OUT_DATA		BIT(0)
#define SGPIO_G7_IRQ_EN		BIT(2)
#define SGPIO_G7_IRQ_TYPE0		BIT(3)
#define SGPIO_G7_IRQ_TYPE1		BIT(4)
#define SGPIO_G7_IRQ_TYPE2		BIT(5)
#define SGPIO_G7_IRQ_STS		BIT(12)
#define SGPIO_G7_IN_DATA		BIT(13)
#define SGPIO_G7_SERIAL_OUT_SEL	GENMASK(17, 16)
#define SELECT_FROM_CSR		0U

#define ASPEED_SGPIO_G7_CFG_OFFSET	0x0U
#define ASPEED_SGPIO_CLK_DIV_MASK	GENMASK(31, 16)
#define ASPEED_SGPIO_ENABLE		BIT(0)
#define ASPEED_SGPIO_PINS_SHIFT	6U
#define ASPEED_SGPIO_G7_PINS_MASK	GENMASK(11, ASPEED_SGPIO_PINS_SHIFT)

#define ASPEED_G7_SGPIOM_GPIOS_PER_BANK	GPIO_MAX_PINS_PER_PORT
#define ASPEED_G7_SGPIOM_PARENT_REMAINING_GPIOS(node_id)			\
	((DT_PROP(DT_PARENT(node_id), ngpios) > DT_PROP(node_id, pin_offset)) ?	\
	 (DT_PROP(DT_PARENT(node_id), ngpios) - DT_PROP(node_id, pin_offset)) : 0U)
#define ASPEED_G7_SGPIOM_CHILD_NGPIOS(node_id)					\
	MIN(MIN(DT_PROP_OR(node_id, ngpios, ASPEED_G7_SGPIOM_GPIOS_PER_BANK),	\
		ASPEED_G7_SGPIOM_GPIOS_PER_BANK),				\
	    ASPEED_G7_SGPIOM_PARENT_REMAINING_GPIOS(node_id))
#define ASPEED_G7_SGPIOM_CHILD_NGPIOS_MASK(node_id)				\
	GPIO_PORT_PIN_MASK_FROM_NGPIOS(ASPEED_G7_SGPIOM_CHILD_NGPIOS(node_id))
#define ASPEED_G7_SGPIOM_CHILD_GPIO_RESERVED(node_id)				\
	DT_PROP_OR(node_id, gpio_reserved, 0)
#define ASPEED_G7_SGPIOM_CHILD_PORT_PIN_MASK(node_id)				\
	(ASPEED_G7_SGPIOM_CHILD_NGPIOS_MASK(node_id) &				\
	 ~ASPEED_G7_SGPIOM_CHILD_GPIO_RESERVED(node_id))

struct aspeed_g7_sgpiom_device_array {
	const struct device *dev;
};

struct aspeed_g7_sgpiom_parent_config {
	mem_addr_t base;
	const struct device *clk_dev;
	const clock_control_subsys_t clk_id;
	void (*irq_config_func)(void);
	const struct aspeed_g7_sgpiom_device_array *child_dev;
	uint32_t child_num;
	uint32_t bus_freq;
	uint32_t ngpios;
	const struct pinctrl_dev_config *pcfg;
};

struct aspeed_g7_sgpiom_parent_data {
	struct k_spinlock lock;
};

struct aspeed_g7_sgpiom_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	const struct device *parent;
	uint32_t pin_offset;
};

struct aspeed_g7_sgpiom_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t cb;
};

enum aspeed_g7_sgpiom_reg {
	reg_val,
	reg_rdata,
	reg_irq_enable,
	reg_irq_type0,
	reg_irq_type1,
	reg_irq_type2,
	reg_irq_status,
	reg_serial_out_sel,
};

#define DEV_PARENT_CFG(dev) ((const struct aspeed_g7_sgpiom_parent_config *)(dev)->config)
#define DEV_PARENT_DATA(dev) ((struct aspeed_g7_sgpiom_parent_data *)(dev)->data)
#define DEV_CFG(dev) ((const struct aspeed_g7_sgpiom_config *)(dev)->config)
#define DEV_DATA(dev) ((struct aspeed_g7_sgpiom_data *)(dev)->data)

static uint32_t aspeed_g7_sgpiom_reg_mask(enum aspeed_g7_sgpiom_reg reg)
{
	switch (reg) {
	case reg_val:
	case reg_rdata:
		return SGPIO_G7_OUT_DATA;
	case reg_irq_enable:
		return SGPIO_G7_IRQ_EN;
	case reg_irq_type0:
		return SGPIO_G7_IRQ_TYPE0;
	case reg_irq_type1:
		return SGPIO_G7_IRQ_TYPE1;
	case reg_irq_type2:
		return SGPIO_G7_IRQ_TYPE2;
	case reg_irq_status:
		return SGPIO_G7_IRQ_STS;
	case reg_serial_out_sel:
		return SGPIO_G7_SERIAL_OUT_SEL;
	default:
		return 0U;
	}
}

static void aspeed_g7_sgpiom_reg_bank_set_raw(const struct device *parent,
					      uint32_t offset,
					      enum aspeed_g7_sgpiom_reg reg,
					      uint32_t val)
{
	mem_addr_t addr = DEV_PARENT_CFG(parent)->base + SGPIO_G7_CTRL_REG_OFFSET(offset);
	uint32_t mask = aspeed_g7_sgpiom_reg_mask(reg);
	uint32_t tmp;

	if (reg != reg_serial_out_sel || mask == 0U) {
		return;
	}

	tmp = sys_read32(addr);
	tmp = (tmp & ~mask) | FIELD_PREP(mask, val);
	sys_write32(tmp, addr);
}

static void aspeed_g7_sgpiom_reg_bit_set_raw(const struct device *parent,
					     uint32_t offset,
					     enum aspeed_g7_sgpiom_reg reg,
					     bool val)
{
	mem_addr_t addr = DEV_PARENT_CFG(parent)->base + SGPIO_G7_CTRL_REG_OFFSET(offset);
	uint32_t mask = aspeed_g7_sgpiom_reg_mask(reg);
	uint32_t tmp;

	if (reg == reg_val || reg == reg_rdata) {
		aspeed_g7_sgpiom_reg_bank_set_raw(parent, offset, reg_serial_out_sel,
						  SELECT_FROM_CSR);
		mask = SGPIO_G7_OUT_DATA;
	}

	if (mask == 0U) {
		return;
	}

	tmp = sys_read32(addr);
	tmp = (tmp & ~mask) | FIELD_PREP(mask, val ? 1U : 0U);
	sys_write32(tmp, addr);
}

static bool aspeed_g7_sgpiom_reg_bit_get_raw(const struct device *parent,
					     uint32_t offset,
					     enum aspeed_g7_sgpiom_reg reg)
{
	mem_addr_t addr = DEV_PARENT_CFG(parent)->base + SGPIO_G7_CTRL_REG_OFFSET(offset);
	uint32_t mask = aspeed_g7_sgpiom_reg_mask(reg);

	if (reg == reg_val) {
		mask = SGPIO_G7_IN_DATA;
	} else if (reg == reg_rdata) {
		mask = SGPIO_G7_OUT_DATA;
	}

	if (mask == 0U) {
		return false;
	}

	return FIELD_GET(mask, sys_read32(addr)) != 0U;
}

static uint32_t aspeed_g7_sgpiom_reg_bank_get_raw(const struct device *parent,
						  uint32_t bank,
						  enum aspeed_g7_sgpiom_reg reg)
{
	mem_addr_t addr;

	if (reg != reg_irq_status) {
		return 0U;
	}

	addr = DEV_PARENT_CFG(parent)->base + SGPIO_G7_IRQ_STS_OFFSET(bank);

	return sys_read32(addr);
}

static int aspeed_g7_sgpiom_pin_to_offset(const struct device *dev,
					  gpio_pin_t pin,
					  uint32_t *offset)
{
	const struct aspeed_g7_sgpiom_config *cfg = DEV_CFG(dev);
	const struct aspeed_g7_sgpiom_parent_config *parent_cfg = DEV_PARENT_CFG(cfg->parent);
	gpio_port_pins_t bit;

	if (pin >= GPIO_MAX_PINS_PER_PORT) {
		LOG_ERR("Invalid gpio pin #%u", pin);
		return -EINVAL;
	}

	bit = BIT(pin);
	if ((cfg->common.port_pin_mask & bit) == 0U) {
		LOG_ERR("Unsupported gpio pin #%u", pin);
		return -EINVAL;
	}

	*offset = cfg->pin_offset + pin;
	if (*offset >= parent_cfg->ngpios) {
		LOG_ERR("GPIO pin offset %u exceeds parent ngpios %u", *offset,
			parent_cfg->ngpios);
		return -EINVAL;
	}

	return 0;
}

static int aspeed_g7_sgpiom_port_get_raw(const struct device *dev,
					 gpio_port_value_t *value)
{
	const struct aspeed_g7_sgpiom_config *cfg = DEV_CFG(dev);
	const struct device *parent = cfg->parent;
	gpio_port_pins_t pin_mask = cfg->common.port_pin_mask;
	gpio_port_value_t port_value = 0U;

	for (uint32_t pin = 0U; pin < GPIO_MAX_PINS_PER_PORT; pin++) {
		if ((pin_mask & BIT(pin)) == 0U) {
			continue;
		}

		if (aspeed_g7_sgpiom_reg_bit_get_raw(parent, cfg->pin_offset + pin,
						     reg_val)) {
			port_value |= BIT(pin);
		}
	}

	*value = port_value;

	return 0;
}

static int aspeed_g7_sgpiom_port_set_masked_raw(const struct device *dev,
						gpio_port_pins_t mask,
						gpio_port_value_t value)
{
	const struct aspeed_g7_sgpiom_config *cfg = DEV_CFG(dev);
	const struct device *parent = cfg->parent;
	struct aspeed_g7_sgpiom_parent_data *data = DEV_PARENT_DATA(parent);
	k_spinlock_key_t key;

	mask &= cfg->common.port_pin_mask;
	key = k_spin_lock(&data->lock);

	for (uint32_t pin = 0U; pin < GPIO_MAX_PINS_PER_PORT; pin++) {
		if ((mask & BIT(pin)) == 0U) {
			continue;
		}

		aspeed_g7_sgpiom_reg_bit_set_raw(parent, cfg->pin_offset + pin, reg_val,
						 (value & BIT(pin)) != 0U);
	}

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int aspeed_g7_sgpiom_port_set_bits_raw(const struct device *dev,
					      gpio_port_value_t mask)
{
	return aspeed_g7_sgpiom_port_set_masked_raw(dev, mask, mask);
}

static int aspeed_g7_sgpiom_port_clear_bits_raw(const struct device *dev,
						gpio_port_value_t mask)
{
	return aspeed_g7_sgpiom_port_set_masked_raw(dev, mask, 0U);
}

static int aspeed_g7_sgpiom_port_toggle_bits(const struct device *dev,
					     gpio_port_value_t mask)
{
	const struct aspeed_g7_sgpiom_config *cfg = DEV_CFG(dev);
	const struct device *parent = cfg->parent;
	struct aspeed_g7_sgpiom_parent_data *data = DEV_PARENT_DATA(parent);
	k_spinlock_key_t key;

	mask &= cfg->common.port_pin_mask;
	key = k_spin_lock(&data->lock);

	for (uint32_t pin = 0U; pin < GPIO_MAX_PINS_PER_PORT; pin++) {
		bool val;

		if ((mask & BIT(pin)) == 0U) {
			continue;
		}

		val = aspeed_g7_sgpiom_reg_bit_get_raw(parent, cfg->pin_offset + pin,
						       reg_rdata);
		aspeed_g7_sgpiom_reg_bit_set_raw(parent, cfg->pin_offset + pin,
						 reg_val, !val);
	}

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int aspeed_g7_sgpiom_pin_interrupt_configure(const struct device *dev,
						    gpio_pin_t pin,
						    enum gpio_int_mode mode,
						    enum gpio_int_trig trig)
{
	const struct device *parent = DEV_CFG(dev)->parent;
	struct aspeed_g7_sgpiom_parent_data *data = DEV_PARENT_DATA(parent);
	k_spinlock_key_t key;
	uint32_t type0 = 0U;
	uint32_t type1 = 0U;
	uint32_t type2 = 0U;
	uint32_t offset;
	int ret;

	ret = aspeed_g7_sgpiom_pin_to_offset(dev, pin, &offset);
	if (ret) {
		return ret;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		key = k_spin_lock(&data->lock);
		aspeed_g7_sgpiom_reg_bit_set_raw(parent, offset, reg_irq_enable, false);
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	if (mode == GPIO_INT_MODE_LEVEL) {
		if (trig == GPIO_INT_TRIG_LOW) {
			type1 = 1U;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			type0 = 1U;
			type1 = 1U;
		} else {
			return -ENOTSUP;
		}
	} else {
		if (trig == GPIO_INT_TRIG_LOW) {
			/* falling edge */
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			type0 = 1U;
		} else if (trig == GPIO_INT_TRIG_BOTH) {
			type0 = 1U;
			type2 = 1U;
		} else {
			return -ENOTSUP;
		}
	}

	key = k_spin_lock(&data->lock);
	aspeed_g7_sgpiom_reg_bit_set_raw(parent, offset, reg_irq_enable, false);
	aspeed_g7_sgpiom_reg_bit_set_raw(parent, offset, reg_irq_type0, type0);
	aspeed_g7_sgpiom_reg_bit_set_raw(parent, offset, reg_irq_type1, type1);
	aspeed_g7_sgpiom_reg_bit_set_raw(parent, offset, reg_irq_type2, type2);
	aspeed_g7_sgpiom_reg_bit_set_raw(parent, offset, reg_irq_status, true);
	aspeed_g7_sgpiom_reg_bit_set_raw(parent, offset, reg_irq_enable, true);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int aspeed_g7_sgpiom_manage_callback(const struct device *dev,
					    struct gpio_callback *callback,
					    bool set)
{
	struct aspeed_g7_sgpiom_data *data = DEV_DATA(dev);

	return gpio_manage_callback(&data->cb, callback, set);
}

static int aspeed_g7_sgpiom_config(const struct device *dev,
				   gpio_pin_t pin,
				   gpio_flags_t flags)
{
	const struct device *parent = DEV_CFG(dev)->parent;
	struct aspeed_g7_sgpiom_parent_data *data = DEV_PARENT_DATA(parent);
	k_spinlock_key_t key;
	uint32_t offset;
	uint32_t io_flags;
	int ret;

	ret = aspeed_g7_sgpiom_pin_to_offset(dev, pin, &offset);
	if (ret) {
		return ret;
	}

	io_flags = flags & (GPIO_INPUT | GPIO_OUTPUT);
	if (io_flags == GPIO_DISCONNECTED) {
		return -ENOTSUP;
	}

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0U) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_OUTPUT) == 0U) {
		return 0;
	}

	key = k_spin_lock(&data->lock);
	if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
		aspeed_g7_sgpiom_reg_bit_set_raw(parent, offset, reg_val, true);
	} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
		aspeed_g7_sgpiom_reg_bit_set_raw(parent, offset, reg_val, false);
	}
	k_spin_unlock(&data->lock, key);

	return 0;
}

static const struct gpio_driver_api aspeed_g7_sgpiom_driver = {
	.pin_configure = aspeed_g7_sgpiom_config,
	.port_get_raw = aspeed_g7_sgpiom_port_get_raw,
	.port_set_masked_raw = aspeed_g7_sgpiom_port_set_masked_raw,
	.port_set_bits_raw = aspeed_g7_sgpiom_port_set_bits_raw,
	.port_clear_bits_raw = aspeed_g7_sgpiom_port_clear_bits_raw,
	.port_toggle_bits = aspeed_g7_sgpiom_port_toggle_bits,
	.pin_interrupt_configure = aspeed_g7_sgpiom_pin_interrupt_configure,
	.manage_callback = aspeed_g7_sgpiom_manage_callback,
};

static gpio_port_pins_t aspeed_g7_sgpiom_get_pending_pins(const struct device *parent,
							  const struct device *dev)
{
	const struct aspeed_g7_sgpiom_config *cfg = DEV_CFG(dev);
	const struct aspeed_g7_sgpiom_parent_config *parent_cfg = DEV_PARENT_CFG(parent);
	uint32_t bank = cfg->pin_offset / GPIO_MAX_PINS_PER_PORT;
	uint32_t shift = cfg->pin_offset % GPIO_MAX_PINS_PER_PORT;
	uint64_t pending;

	pending = aspeed_g7_sgpiom_reg_bank_get_raw(parent, bank, reg_irq_status);
	pending >>= shift;

	if (shift != 0U && (cfg->pin_offset + GPIO_MAX_PINS_PER_PORT) < parent_cfg->ngpios) {
		pending |= (uint64_t)aspeed_g7_sgpiom_reg_bank_get_raw(parent, bank + 1U,
								       reg_irq_status)
			   << (GPIO_MAX_PINS_PER_PORT - shift);
	}

	return (gpio_port_pins_t)pending & cfg->common.port_pin_mask;
}

static void aspeed_g7_sgpiom_isr(const void *arg)
{
	const struct device *parent = arg;
	const struct aspeed_g7_sgpiom_parent_config *parent_cfg = DEV_PARENT_CFG(parent);
	struct aspeed_g7_sgpiom_parent_data *parent_data = DEV_PARENT_DATA(parent);

	for (uint32_t index = 0U; index < parent_cfg->child_num; index++) {
		const struct device *dev = parent_cfg->child_dev[index].dev;
		const struct aspeed_g7_sgpiom_config *cfg = DEV_CFG(dev);
		struct aspeed_g7_sgpiom_data *data = DEV_DATA(dev);
		gpio_port_pins_t pending = aspeed_g7_sgpiom_get_pending_pins(parent, dev);

		while (pending != 0U) {
			gpio_port_pins_t bit = pending & -pending;
			uint32_t pin = find_lsb_set(bit) - 1U;
			k_spinlock_key_t key;

			gpio_fire_callbacks(&data->cb, dev, bit);

			key = k_spin_lock(&parent_data->lock);
			aspeed_g7_sgpiom_reg_bit_set_raw(parent, cfg->pin_offset + pin,
							 reg_irq_status, true);
			k_spin_unlock(&parent_data->lock, key);

			pending &= ~bit;
		}
	}
}

static int aspeed_g7_sgpiom_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int aspeed_g7_sgpiom_get_clk_div(const struct device *parent, uint32_t *clk_div)
{
	const struct aspeed_g7_sgpiom_parent_config *cfg = DEV_PARENT_CFG(parent);
	uint64_t div;
	uint32_t clk_rate;
	int ret;

	if (!device_is_ready(cfg->clk_dev)) {
		return -ENODEV;
	}

	if (cfg->bus_freq == 0U) {
		return -EINVAL;
	}

	ret = clock_control_get_rate(cfg->clk_dev, cfg->clk_id, &clk_rate);
	if (ret) {
		return ret;
	}

	div = (uint64_t)clk_rate / ((uint64_t)cfg->bus_freq * 2U);
	if (div == 0U || (div - 1U) > UINT16_MAX) {
		return -EINVAL;
	}

	*clk_div = div - 1U;
	LOG_DBG("target rate: %u div: %u", cfg->bus_freq, *clk_div);

	return 0;
}

static int aspeed_g7_sgpiom_configure_bus(const struct device *parent)
{
	const struct aspeed_g7_sgpiom_parent_config *cfg = DEV_PARENT_CFG(parent);
	uint32_t pin_count = cfg->ngpios / 8U;
	uint32_t clk_div;
	uint32_t value;
	int ret;

	if (cfg->ngpios == 0U || (cfg->ngpios % 8U) != 0U) {
		LOG_ERR("ngpios must be a non-zero multiple of 8");
		return -EINVAL;
	}

	if (pin_count > FIELD_GET(ASPEED_SGPIO_G7_PINS_MASK, ASPEED_SGPIO_G7_PINS_MASK)) {
		LOG_ERR("ngpios %u exceeds controller field width", cfg->ngpios);
		return -EINVAL;
	}

	ret = aspeed_g7_sgpiom_get_clk_div(parent, &clk_div);
	if (ret) {
		return ret;
	}

	for (uint32_t pin = 0U; pin < cfg->ngpios; pin++) {
		aspeed_g7_sgpiom_reg_bank_set_raw(parent, pin, reg_serial_out_sel,
						  SELECT_FROM_CSR);
	}

	value = FIELD_PREP(ASPEED_SGPIO_CLK_DIV_MASK, clk_div) |
		FIELD_PREP(ASPEED_SGPIO_G7_PINS_MASK, pin_count) |
		ASPEED_SGPIO_ENABLE;
	sys_write32(value, cfg->base + ASPEED_SGPIO_G7_CFG_OFFSET);

	return 0;
}

static void aspeed_g7_sgpiom_setup_irqs(const struct device *parent)
{
	const struct aspeed_g7_sgpiom_parent_config *cfg = DEV_PARENT_CFG(parent);
	struct aspeed_g7_sgpiom_parent_data *data = DEV_PARENT_DATA(parent);
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	for (uint32_t pin = 0U; pin < cfg->ngpios; pin++) {
		aspeed_g7_sgpiom_reg_bit_set_raw(parent, pin, reg_irq_enable, false);
		aspeed_g7_sgpiom_reg_bit_set_raw(parent, pin, reg_irq_status, true);
		aspeed_g7_sgpiom_reg_bit_set_raw(parent, pin, reg_irq_type0, false);
		aspeed_g7_sgpiom_reg_bit_set_raw(parent, pin, reg_irq_type1, false);
		aspeed_g7_sgpiom_reg_bit_set_raw(parent, pin, reg_irq_type2, false);
	}

	k_spin_unlock(&data->lock, key);
}

static int aspeed_g7_sgpiom_parent_init(const struct device *parent)
{
	const struct aspeed_g7_sgpiom_parent_config *cfg = DEV_PARENT_CFG(parent);
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	ret = aspeed_g7_sgpiom_configure_bus(parent);
	if (ret) {
		return ret;
	}

	aspeed_g7_sgpiom_setup_irqs(parent);
	cfg->irq_config_func();

	return 0;
}

struct aspeed_g7_sgpiom_device_cont {
	const struct aspeed_g7_sgpiom_config *cfg;
	struct aspeed_g7_sgpiom_data *data;
};

#define ASPEED_G7_SGPIOM_ENUM(node_id) node_id,
#define ASPEED_G7_SGPIOM_DEV_DATA(node_id) {},
#define ASPEED_G7_SGPIOM_DEV_CFG(node_id) {						\
	.common = {									\
		.port_pin_mask = ASPEED_G7_SGPIOM_CHILD_PORT_PIN_MASK(node_id),		\
	},										\
	.parent = DEVICE_DT_GET(DT_PARENT(node_id)),					\
	.pin_offset = DT_PROP(node_id, pin_offset),					\
},
#define ASPEED_G7_SGPIOM_DT_DEFINE(node_id)						\
	DEVICE_DT_DEFINE(node_id, aspeed_g7_sgpiom_init, NULL,			\
			 &DT_PARENT(node_id).data[node_id],				\
			 &DT_PARENT(node_id).cfg[node_id], POST_KERNEL,		\
			 CONFIG_GPIO_ASPEED_G7_SGPIOM_INIT_PRIORITY,			\
			 &aspeed_g7_sgpiom_driver);

#define ASPEED_G7_SGPIOM_DEV_DECLARE(node_id) { .dev = DEVICE_DT_GET(node_id) },

#define ASPEED_G7_SGPIOM_IRQ_CONFIG(inst)						\
static void aspeed_g7_sgpiom_irq_config_##inst(void)					\
{											\
	IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),			\
		    aspeed_g7_sgpiom_isr, DEVICE_DT_INST_GET(inst), 0);		\
	irq_enable(DT_INST_IRQN(inst));							\
}

#define ASPEED_G7_SGPIOM_DEVICE_INIT(inst)						\
	BUILD_ASSERT((DT_INST_PROP(inst, ngpios) % 8) == 0,				\
		     "aspeed,g7-sgpiom ngpios must be a multiple of 8");		\
	ASPEED_G7_SGPIOM_IRQ_CONFIG(inst)						\
	PINCTRL_DT_INST_DEFINE(inst);							\
	static const struct aspeed_g7_sgpiom_device_array child_dev_##inst[] = {	\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst),				\
					     ASPEED_G7_SGPIOM_DEV_DECLARE)		\
	};										\
	static const struct aspeed_g7_sgpiom_parent_config				\
		aspeed_g7_sgpiom_parent_cfg_##inst = {					\
		.base = DT_INST_REG_ADDR(inst),						\
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),			\
		.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, clk_id),	\
		.irq_config_func = aspeed_g7_sgpiom_irq_config_##inst,			\
		.child_dev = child_dev_##inst,						\
		.child_num = ARRAY_SIZE(child_dev_##inst),				\
		.ngpios = DT_INST_PROP(inst, ngpios),					\
		.bus_freq = DT_INST_PROP(inst, aspeed_bus_freq),			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),				\
	};										\
	static struct aspeed_g7_sgpiom_parent_data aspeed_g7_sgpiom_parent_data_##inst; \
	DEVICE_DT_INST_DEFINE(inst, aspeed_g7_sgpiom_parent_init, NULL,		\
			      &aspeed_g7_sgpiom_parent_data_##inst,			\
			      &aspeed_g7_sgpiom_parent_cfg_##inst, POST_KERNEL,		\
			      CONFIG_GPIO_ASPEED_G7_SGPIOM_INIT_PRIORITY, NULL);	\
	static const struct aspeed_g7_sgpiom_config aspeed_g7_sgpiom_cfg_##inst[] = {	\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst),				\
					     ASPEED_G7_SGPIOM_DEV_CFG)		\
	};										\
	static struct aspeed_g7_sgpiom_data aspeed_g7_sgpiom_data_##inst[] = {		\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst),				\
					     ASPEED_G7_SGPIOM_DEV_DATA)		\
	};										\
	static const struct aspeed_g7_sgpiom_device_cont DT_DRV_INST(inst) = {		\
		.cfg = aspeed_g7_sgpiom_cfg_##inst,					\
		.data = aspeed_g7_sgpiom_data_##inst,					\
	};										\
	enum {										\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst),				\
					     ASPEED_G7_SGPIOM_ENUM)			\
	};										\
	DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), ASPEED_G7_SGPIOM_DT_DEFINE)

DT_INST_FOREACH_STATUS_OKAY(ASPEED_G7_SGPIOM_DEVICE_INIT)
