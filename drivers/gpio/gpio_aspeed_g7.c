/*
 * Copyright (c) 2026 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_g7_gpio

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/dt-bindings/gpio/aspeed-gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_aspeed_g7);

#define ASPEED_G7_NR_GPIOS			216U
#define ASPEED_G7_GPIOS_PER_BANK		32U
#define ASPEED_G7_BANK(offset)			((offset) / ASPEED_G7_GPIOS_PER_BANK)
#define ASPEED_G7_PIN(offset)			((offset) % ASPEED_G7_GPIOS_PER_BANK)

#define ASPEED_G7_IRQ_STS_BASE			0x100U
#define ASPEED_G7_IRQ_STS_OFFSET(bank)		(ASPEED_G7_IRQ_STS_BASE + ((bank) * 0x4U))
#define ASPEED_G7_CTRL_REG_BASE			0x180U
#define ASPEED_G7_CTRL_REG_OFFSET(offset)	(ASPEED_G7_CTRL_REG_BASE + ((offset) * 0x4U))

#define ASPEED_G7_CTRL_OUT_DATA			BIT(0)
#define ASPEED_G7_CTRL_DIR			BIT(1)
#define ASPEED_G7_CTRL_IRQ_EN			BIT(2)
#define ASPEED_G7_CTRL_IRQ_TYPE0		BIT(3)
#define ASPEED_G7_CTRL_IRQ_TYPE1		BIT(4)
#define ASPEED_G7_CTRL_IRQ_TYPE2		BIT(5)
#define ASPEED_G7_CTRL_DEBOUNCE_SEL2		BIT(7)
#define ASPEED_G7_CTRL_DEBOUNCE_SEL1		BIT(8)
#define ASPEED_G7_CTRL_IRQ_STS			BIT(12)
#define ASPEED_G7_CTRL_IN_DATA			BIT(13)

#define ASPEED_G7_DEBOUNCE_TIMER_NUM		4U

#define ASPEED_G7_INPUT_MASK_FROM_OFFSET(offset)					\
	(((offset) == 32U) ? 0x0fffffffU : (((offset) == 192U) ? 0x00ffffffU :	\
					     0xffffffffU))
#define ASPEED_G7_OUTPUT_MASK_FROM_OFFSET(offset)					\
	(((offset) == 32U) ? 0x0fffffffU : (((offset) == 192U) ? 0x00ff0000U :	\
					     0xffffffffU))
#define ASPEED_G7_VALID_MASK_FROM_OFFSET(offset)					\
	(ASPEED_G7_INPUT_MASK_FROM_OFFSET(offset) |					\
	 ASPEED_G7_OUTPUT_MASK_FROM_OFFSET(offset))

#define ASPEED_G7_PARENT_REMAINING_GPIOS(node_id)				\
	((DT_PROP(DT_PARENT(node_id), ngpios) > DT_PROP(node_id, pin_offset)) ?	\
	 (DT_PROP(DT_PARENT(node_id), ngpios) - DT_PROP(node_id, pin_offset)) : 0U)
#define ASPEED_G7_CHILD_NGPIOS(node_id)						\
	MIN(MIN(DT_PROP_OR(node_id, ngpios, ASPEED_G7_GPIOS_PER_BANK),		\
		ASPEED_G7_GPIOS_PER_BANK), ASPEED_G7_PARENT_REMAINING_GPIOS(node_id))
#define ASPEED_G7_CHILD_NGPIOS_MASK(node_id)					\
	GPIO_PORT_PIN_MASK_FROM_NGPIOS(ASPEED_G7_CHILD_NGPIOS(node_id))
#define ASPEED_G7_CHILD_GPIO_RESERVED(node_id)					\
	DT_PROP_OR(node_id, gpio_reserved, 0)
#define ASPEED_G7_CHILD_PORT_PIN_MASK(node_id)					\
	(ASPEED_G7_CHILD_NGPIOS_MASK(node_id) &					\
	 ASPEED_G7_VALID_MASK_FROM_OFFSET(DT_PROP(node_id, pin_offset)) &		\
	 ~ASPEED_G7_CHILD_GPIO_RESERVED(node_id))

static const uint16_t aspeed_g7_debounce_timer_offsets[ASPEED_G7_DEBOUNCE_TIMER_NUM] = {
	0x00, 0x00, 0x04, 0x08
};

struct gpio_aspeed_g7_device_array {
	const struct device *dev;
};

struct gpio_aspeed_g7_parent_config {
	mem_addr_t base;
	const struct device *clock_dev;
	const clock_control_subsys_t clk_id;
	void (*irq_config_func)(const struct device *dev);
	const struct gpio_aspeed_g7_device_array *child_dev;
	uint32_t child_num;
	uint32_t ngpios;
	uint32_t deb_interval_us;
};

struct gpio_aspeed_g7_parent_data {
	struct k_spinlock lock;
	uint8_t offset_timer[ASPEED_G7_NR_GPIOS];
	uint32_t timer_users[ASPEED_G7_DEBOUNCE_TIMER_NUM];
};

struct gpio_aspeed_g7_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	const struct device *parent;
	uint16_t pin_offset;
	gpio_port_pins_t input_mask;
	gpio_port_pins_t output_mask;
};

struct gpio_aspeed_g7_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t cb;
};

#define DEV_PARENT_CFG(dev) ((const struct gpio_aspeed_g7_parent_config *)(dev)->config)
#define DEV_PARENT_DATA(dev) ((struct gpio_aspeed_g7_parent_data *)(dev)->data)
#define DEV_CFG(dev) ((const struct gpio_aspeed_g7_config *)(dev)->config)
#define DEV_DATA(dev) ((struct gpio_aspeed_g7_data *)(dev)->data)

static mem_addr_t aspeed_g7_ctrl_addr(const struct device *parent, uint32_t offset)
{
	return DEV_PARENT_CFG(parent)->base + ASPEED_G7_CTRL_REG_OFFSET(offset);
}

static mem_addr_t aspeed_g7_irq_sts_addr(const struct device *parent, uint32_t bank)
{
	return DEV_PARENT_CFG(parent)->base + ASPEED_G7_IRQ_STS_OFFSET(bank);
}

static uint32_t aspeed_g7_global_offset(const struct device *dev, gpio_pin_t pin)
{
	return DEV_CFG(dev)->pin_offset + pin;
}

static bool aspeed_g7_have_gpio(const struct device *dev, gpio_pin_t pin)
{
	uint32_t offset = aspeed_g7_global_offset(dev, pin);

	if (pin >= ASPEED_G7_GPIOS_PER_BANK) {
		return false;
	}

	if (offset >= ASPEED_G7_NR_GPIOS ||
	    offset >= DEV_PARENT_CFG(DEV_CFG(dev)->parent)->ngpios) {
		return false;
	}

	return (DEV_CFG(dev)->common.port_pin_mask & BIT(pin)) != 0U;
}

static bool aspeed_g7_have_input(const struct device *dev, gpio_pin_t pin)
{
	if (!aspeed_g7_have_gpio(dev, pin)) {
		return false;
	}

	return (DEV_CFG(dev)->input_mask & BIT(pin)) != 0U;
}

static bool aspeed_g7_have_output(const struct device *dev, gpio_pin_t pin)
{
	if (!aspeed_g7_have_gpio(dev, pin)) {
		return false;
	}

	return (DEV_CFG(dev)->output_mask & BIT(pin)) != 0U;
}

static bool aspeed_g7_reg_bit_get(const struct device *parent, uint32_t offset, uint32_t mask)
{
	return (sys_read32(aspeed_g7_ctrl_addr(parent, offset)) & mask) != 0U;
}

static void aspeed_g7_reg_bit_set(const struct device *parent, uint32_t offset,
				  uint32_t mask, bool val)
{
	mem_addr_t addr = aspeed_g7_ctrl_addr(parent, offset);
	uint32_t reg = sys_read32(addr);

	if (val) {
		reg |= mask;
	} else {
		reg &= ~mask;
	}

	sys_write32(reg, addr);
}

static gpio_port_value_t aspeed_g7_port_read_bits(const struct device *dev,
						  gpio_port_pins_t mask,
						  uint32_t reg_mask)
{
	const struct device *parent = DEV_CFG(dev)->parent;
	gpio_port_value_t value = 0U;

	for (uint32_t pin = 0; pin < ASPEED_G7_GPIOS_PER_BANK; pin++) {
		if ((mask & BIT(pin)) == 0U) {
			continue;
		}

		if (aspeed_g7_reg_bit_get(parent, aspeed_g7_global_offset(dev, pin), reg_mask)) {
			value |= BIT(pin);
		}
	}

	return value;
}

static int aspeed_g7_usecs_to_cycles(const struct device *parent, uint32_t usecs,
				     uint32_t *cycles)
{
	const struct gpio_aspeed_g7_parent_config *cfg = DEV_PARENT_CFG(parent);
	uint64_t cycles_64;
	uint32_t clk_rate;
	int ret;

	ret = clock_control_get_rate(cfg->clock_dev, cfg->clk_id, &clk_rate);
	if (ret) {
		return ret;
	}

	if (clk_rate == 0U) {
		return -ENOTSUP;
	}

	cycles_64 = DIV_ROUND_UP((uint64_t)usecs * clk_rate, USEC_PER_SEC);
	if (cycles_64 > UINT32_MAX) {
		return -ERANGE;
	}

	*cycles = (uint32_t)cycles_64;

	return 0;
}

static void aspeed_g7_configure_debounce_timer(const struct device *parent,
					       uint32_t offset, uint32_t timer)
{
	aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_DEBOUNCE_SEL1,
			      (timer & BIT(1)) != 0U);
	aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_DEBOUNCE_SEL2,
			      (timer & BIT(0)) != 0U);
}

static void aspeed_g7_unregister_debounce_timer(const struct device *parent, uint32_t offset)
{
	struct gpio_aspeed_g7_parent_data *data = DEV_PARENT_DATA(parent);
	uint8_t timer;

	if (offset >= ASPEED_G7_NR_GPIOS) {
		return;
	}

	timer = data->offset_timer[offset];
	if (timer == 0U) {
		return;
	}

	if (data->timer_users[timer] > 0U) {
		data->timer_users[timer]--;
	}

	data->offset_timer[offset] = 0U;
}

static int aspeed_g7_enable_debounce(const struct device *dev, gpio_pin_t pin)
{
	const struct device *parent = DEV_CFG(dev)->parent;
	const struct gpio_aspeed_g7_parent_config *cfg = DEV_PARENT_CFG(parent);
	struct gpio_aspeed_g7_parent_data *data = DEV_PARENT_DATA(parent);
	uint32_t offset = aspeed_g7_global_offset(dev, pin);
	uint32_t requested_cycles;
	k_spinlock_key_t key;
	uint32_t timer;
	int ret;

	if (!aspeed_g7_have_input(dev, pin)) {
		return -ENOTSUP;
	}

	if (cfg->deb_interval_us == 0U) {
		key = k_spin_lock(&data->lock);
		aspeed_g7_unregister_debounce_timer(parent, offset);
		aspeed_g7_configure_debounce_timer(parent, offset, 0U);
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	ret = aspeed_g7_usecs_to_cycles(parent, cfg->deb_interval_us, &requested_cycles);
	if (ret) {
		return ret;
	}

	key = k_spin_lock(&data->lock);

	aspeed_g7_unregister_debounce_timer(parent, offset);

	for (timer = 1U; timer < ASPEED_G7_DEBOUNCE_TIMER_NUM; timer++) {
		mem_addr_t addr = cfg->base + aspeed_g7_debounce_timer_offsets[timer];

		if (sys_read32(addr) == requested_cycles) {
			break;
		}
	}

	if (timer == ASPEED_G7_DEBOUNCE_TIMER_NUM) {
		for (timer = 1U; timer < ASPEED_G7_DEBOUNCE_TIMER_NUM; timer++) {
			if (data->timer_users[timer] == 0U) {
				break;
			}
		}

		if (timer == ASPEED_G7_DEBOUNCE_TIMER_NUM) {
			aspeed_g7_configure_debounce_timer(parent, offset, 0U);
			k_spin_unlock(&data->lock, key);
			return -EPERM;
		}

		sys_write32(requested_cycles, cfg->base + aspeed_g7_debounce_timer_offsets[timer]);
	}

	data->offset_timer[offset] = timer;
	data->timer_users[timer]++;
	aspeed_g7_configure_debounce_timer(parent, offset, timer);

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int aspeed_g7_disable_debounce(const struct device *dev, gpio_pin_t pin)
{
	const struct device *parent = DEV_CFG(dev)->parent;
	struct gpio_aspeed_g7_parent_data *data = DEV_PARENT_DATA(parent);
	uint32_t offset = aspeed_g7_global_offset(dev, pin);
	k_spinlock_key_t key;

	if (!aspeed_g7_have_input(dev, pin)) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);
	aspeed_g7_unregister_debounce_timer(parent, offset);
	aspeed_g7_configure_debounce_timer(parent, offset, 0U);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int gpio_aspeed_g7_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	*value = aspeed_g7_port_read_bits(dev, DEV_CFG(dev)->common.port_pin_mask,
					  ASPEED_G7_CTRL_IN_DATA);

	return 0;
}

static int gpio_aspeed_g7_port_set_masked_raw(const struct device *dev,
					      gpio_port_pins_t mask,
					      gpio_port_value_t value)
{
	const struct device *parent = DEV_CFG(dev)->parent;
	struct gpio_aspeed_g7_parent_data *data = DEV_PARENT_DATA(parent);
	k_spinlock_key_t key;

	if ((mask & ~DEV_CFG(dev)->output_mask) != 0U) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);

	for (uint32_t pin = 0U; pin < ASPEED_G7_GPIOS_PER_BANK; pin++) {
		if ((mask & BIT(pin)) == 0U) {
			continue;
		}

		aspeed_g7_reg_bit_set(parent, aspeed_g7_global_offset(dev, pin),
				      ASPEED_G7_CTRL_OUT_DATA, (value & BIT(pin)) != 0U);
	}

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int gpio_aspeed_g7_port_set_bits_raw(const struct device *dev, gpio_port_pins_t mask)
{
	return gpio_aspeed_g7_port_set_masked_raw(dev, mask, mask);
}

static int gpio_aspeed_g7_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t mask)
{
	return gpio_aspeed_g7_port_set_masked_raw(dev, mask, 0U);
}

static int gpio_aspeed_g7_port_toggle_bits(const struct device *dev, gpio_port_pins_t mask)
{
	gpio_port_value_t value;

	if ((mask & ~DEV_CFG(dev)->output_mask) != 0U) {
		return -ENOTSUP;
	}

	value = aspeed_g7_port_read_bits(dev, mask, ASPEED_G7_CTRL_OUT_DATA);

	return gpio_aspeed_g7_port_set_masked_raw(dev, mask, value ^ mask);
}

static int gpio_aspeed_g7_pin_interrupt_configure(const struct device *dev,
						  gpio_pin_t pin,
						  enum gpio_int_mode mode,
						  enum gpio_int_trig trig)
{
	const struct device *parent = DEV_CFG(dev)->parent;
	struct gpio_aspeed_g7_parent_data *data = DEV_PARENT_DATA(parent);
	uint32_t offset = aspeed_g7_global_offset(dev, pin);
	bool type0 = false;
	bool type1 = false;
	bool type2 = false;
	k_spinlock_key_t key;

	if (!aspeed_g7_have_input(dev, pin)) {
		return -ENOTSUP;
	}

	if ((trig & GPIO_INT_TRIG_WAKE) != 0U) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);

	if (mode == GPIO_INT_MODE_DISABLED) {
		aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_EN, false);
		aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_STS, true);
		k_spin_unlock(&data->lock, key);
		return 0;
	}

#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	if (mode == GPIO_INT_MODE_DISABLE_ONLY) {
		aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_EN, false);
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	if (mode == GPIO_INT_MODE_ENABLE_ONLY) {
		aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_STS, true);
		aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_EN, true);
		k_spin_unlock(&data->lock, key);
		return 0;
	}
#endif

	if (mode == GPIO_INT_MODE_LEVEL) {
		if (trig == GPIO_INT_TRIG_LOW) {
			type1 = true;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			type0 = true;
			type1 = true;
		} else {
			k_spin_unlock(&data->lock, key);
			return -ENOTSUP;
		}
	} else if (mode == GPIO_INT_MODE_EDGE) {
		if (trig == GPIO_INT_TRIG_HIGH) {
			type0 = true;
		} else if (trig == GPIO_INT_TRIG_BOTH) {
			type0 = true;
			type2 = true;
		} else if (trig != GPIO_INT_TRIG_LOW) {
			k_spin_unlock(&data->lock, key);
			return -ENOTSUP;
		}
	} else {
		k_spin_unlock(&data->lock, key);
		return -ENOTSUP;
	}

	aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_EN, false);
	aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_TYPE0, type0);
	aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_TYPE1, type1);
	aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_TYPE2, type2);
	aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_STS, true);
	aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_IRQ_EN, true);

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int gpio_aspeed_g7_manage_callback(const struct device *dev,
					  struct gpio_callback *callback, bool set)
{
	return gpio_manage_callback(&DEV_DATA(dev)->cb, callback, set);
}

static uint32_t gpio_aspeed_g7_get_pending_int(const struct device *dev)
{
	uint32_t bank = ASPEED_G7_BANK(DEV_CFG(dev)->pin_offset);

	return sys_read32(aspeed_g7_irq_sts_addr(DEV_CFG(dev)->parent, bank)) &
	       DEV_CFG(dev)->common.port_pin_mask;
}

static int gpio_aspeed_g7_configure(const struct device *dev, gpio_pin_t pin,
				    gpio_flags_t flags)
{
	const struct device *parent = DEV_CFG(dev)->parent;
	struct gpio_aspeed_g7_parent_data *data = DEV_PARENT_DATA(parent);
	uint32_t io_flags = flags & (GPIO_INPUT | GPIO_OUTPUT);
	uint32_t offset = aspeed_g7_global_offset(dev, pin);
	k_spinlock_key_t key;
	int ret;

	if (!aspeed_g7_have_gpio(dev, pin)) {
		return -EINVAL;
	}

	if (io_flags == GPIO_DISCONNECTED || io_flags == (GPIO_INPUT | GPIO_OUTPUT)) {
		return -ENOTSUP;
	}

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0U) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_SINGLE_ENDED) != 0U) {
		if ((flags & GPIO_LINE_OPEN_DRAIN) != 0U) {
			ret = gpio_aspeed_g7_port_clear_bits_raw(dev, BIT(pin));
		} else {
			ret = gpio_aspeed_g7_port_set_bits_raw(dev, BIT(pin));
		}

		if (ret) {
			return ret;
		}
	}

	if ((flags & GPIO_OUTPUT) != 0U) {
		if (!aspeed_g7_have_output(dev, pin)) {
			return -ENOTSUP;
		}

		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			ret = gpio_aspeed_g7_port_set_bits_raw(dev, BIT(pin));
			if (ret) {
				return ret;
			}
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			ret = gpio_aspeed_g7_port_clear_bits_raw(dev, BIT(pin));
			if (ret) {
				return ret;
			}
		}

		key = k_spin_lock(&data->lock);
		aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_DIR, true);
		k_spin_unlock(&data->lock, key);
	} else if ((flags & GPIO_INPUT) != 0U) {
		if (!aspeed_g7_have_input(dev, pin)) {
			return -ENOTSUP;
		}

		if ((flags & ASPEED_GPIO_DEBOUNCE) != 0U) {
			ret = aspeed_g7_enable_debounce(dev, pin);
		} else {
			ret = aspeed_g7_disable_debounce(dev, pin);
		}

		if (ret) {
			return ret;
		}

		key = k_spin_lock(&data->lock);
		aspeed_g7_reg_bit_set(parent, offset, ASPEED_G7_CTRL_DIR, false);
		k_spin_unlock(&data->lock, key);
	}

	return 0;
}

#ifdef CONFIG_GPIO_GET_DIRECTION
static int gpio_aspeed_g7_port_get_dir(const struct device *dev, gpio_port_pins_t map,
				       gpio_port_pins_t *inputs,
				       gpio_port_pins_t *outputs)
{
	gpio_port_pins_t input_pins = 0U;
	gpio_port_pins_t output_pins = 0U;

	map &= DEV_CFG(dev)->common.port_pin_mask;

	for (uint32_t pin = 0U; pin < ASPEED_G7_GPIOS_PER_BANK; pin++) {
		gpio_port_pins_t bit = BIT(pin);

		if ((map & bit) == 0U) {
			continue;
		}

		if ((DEV_CFG(dev)->input_mask & bit) == 0U) {
			output_pins |= bit;
		} else if ((DEV_CFG(dev)->output_mask & bit) == 0U) {
			input_pins |= bit;
		} else if (aspeed_g7_reg_bit_get(DEV_CFG(dev)->parent,
						 aspeed_g7_global_offset(dev, pin),
						 ASPEED_G7_CTRL_DIR)) {
			output_pins |= bit;
		} else {
			input_pins |= bit;
		}
	}

	if (inputs) {
		*inputs = input_pins;
	}

	if (outputs) {
		*outputs = output_pins;
	}

	return 0;
}
#endif

static void gpio_aspeed_g7_isr(const void *arg)
{
	const struct device *parent = arg;
	const struct gpio_aspeed_g7_parent_config *cfg = DEV_PARENT_CFG(parent);

	for (uint32_t index = 0U; index < cfg->child_num; index++) {
		const struct device *dev = cfg->child_dev[index].dev;
		struct gpio_aspeed_g7_data *data = DEV_DATA(dev);
		uint32_t bank = ASPEED_G7_BANK(DEV_CFG(dev)->pin_offset);
		uint32_t pending;

		pending = sys_read32(aspeed_g7_irq_sts_addr(parent, bank)) &
			  DEV_CFG(dev)->common.port_pin_mask;

		for (uint32_t pin = 0U; pin < ASPEED_G7_GPIOS_PER_BANK; pin++) {
			if ((pending & BIT(pin)) == 0U) {
				continue;
			}

			gpio_fire_callbacks(&data->cb, dev, BIT(pin));
			aspeed_g7_reg_bit_set(parent, aspeed_g7_global_offset(dev, pin),
					      ASPEED_G7_CTRL_IRQ_STS, true);
		}
	}
}

static const struct gpio_driver_api gpio_aspeed_g7_driver = {
	.pin_configure = gpio_aspeed_g7_configure,
	.port_get_raw = gpio_aspeed_g7_port_get_raw,
	.port_set_masked_raw = gpio_aspeed_g7_port_set_masked_raw,
	.port_set_bits_raw = gpio_aspeed_g7_port_set_bits_raw,
	.port_clear_bits_raw = gpio_aspeed_g7_port_clear_bits_raw,
	.port_toggle_bits = gpio_aspeed_g7_port_toggle_bits,
	.pin_interrupt_configure = gpio_aspeed_g7_pin_interrupt_configure,
	.manage_callback = gpio_aspeed_g7_manage_callback,
	.get_pending_int = gpio_aspeed_g7_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_aspeed_g7_port_get_dir,
#endif
};

static int gpio_aspeed_g7_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int gpio_aspeed_g7_parent_init(const struct device *parent)
{
	const struct gpio_aspeed_g7_parent_config *cfg = DEV_PARENT_CFG(parent);

	if (cfg->ngpios > ASPEED_G7_NR_GPIOS) {
		return -EINVAL;
	}

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}

	cfg->irq_config_func(parent);

	return 0;
}

struct gpio_aspeed_g7_device_cont {
	const struct gpio_aspeed_g7_config *cfg;
	struct gpio_aspeed_g7_data *data;
};

#define GPIO_ASPEED_G7_ENUM(node_id) node_id,

#define GPIO_ASPEED_G7_DEV_DATA(node_id) {},

#define GPIO_ASPEED_G7_DEV_CFG(node_id)						\
	{										\
		.common = {								\
			.port_pin_mask = ASPEED_G7_CHILD_PORT_PIN_MASK(node_id),	\
		},									\
		.parent = DEVICE_DT_GET(DT_PARENT(node_id)),				\
		.pin_offset = DT_PROP(node_id, pin_offset),				\
		.input_mask = ASPEED_G7_CHILD_NGPIOS_MASK(node_id) &			\
			      ASPEED_G7_INPUT_MASK_FROM_OFFSET(				\
				      DT_PROP(node_id, pin_offset)) &			\
			      ~ASPEED_G7_CHILD_GPIO_RESERVED(node_id),			\
		.output_mask = ASPEED_G7_CHILD_NGPIOS_MASK(node_id) &			\
			       ASPEED_G7_OUTPUT_MASK_FROM_OFFSET(			\
				       DT_PROP(node_id, pin_offset)) &			\
			       ~ASPEED_G7_CHILD_GPIO_RESERVED(node_id),			\
	},

#define GPIO_ASPEED_G7_DT_DEFINE(node_id)						\
	BUILD_ASSERT((DT_PROP(node_id, pin_offset) % ASPEED_G7_GPIOS_PER_BANK) == 0,	\
		     "ASPEED G7 GPIO child pin-offset must be bank aligned");		\
	DEVICE_DT_DEFINE(node_id, gpio_aspeed_g7_init, NULL,				\
			 &DT_PARENT(node_id).data[node_id],				\
			 &DT_PARENT(node_id).cfg[node_id], POST_KERNEL,			\
			 CONFIG_GPIO_ASPEED_G7_INIT_PRIORITY, &gpio_aspeed_g7_driver);

#define GPIO_ASPEED_G7_DEV_DECLARE(node_id) { .dev = DEVICE_DT_GET(node_id) },

#define ASPEED_G7_GPIO_DEVICE_INIT(inst)							\
	static const struct gpio_aspeed_g7_device_array child_dev_##inst[] = {		\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst),				\
					     GPIO_ASPEED_G7_DEV_DECLARE)};		\
	static void gpio_aspeed_g7_irq_config_func_##inst(const struct device *dev)	\
	{										\
		ARG_UNUSED(dev);								\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),		\
			    gpio_aspeed_g7_isr, DEVICE_DT_INST_GET(inst), 0);		\
		irq_enable(DT_INST_IRQN(inst));						\
	}										\
	static const struct gpio_aspeed_g7_parent_config				\
		gpio_aspeed_g7_parent_cfg_##inst = {					\
			.base = DT_INST_REG_ADDR(inst),					\
			.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),		\
			.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst,	\
									      clk_id),	\
			.irq_config_func = gpio_aspeed_g7_irq_config_func_##inst,	\
			.child_dev = child_dev_##inst,					\
			.child_num = ARRAY_SIZE(child_dev_##inst),			\
			.ngpios = DT_INST_PROP(inst, ngpios),				\
			.deb_interval_us = DT_INST_PROP(inst, aspeed_deb_interval_us),	\
		};									\
	static struct gpio_aspeed_g7_parent_data gpio_aspeed_g7_parent_data_##inst;	\
	DEVICE_DT_INST_DEFINE(inst, gpio_aspeed_g7_parent_init, NULL,			\
			      &gpio_aspeed_g7_parent_data_##inst,			\
			      &gpio_aspeed_g7_parent_cfg_##inst, POST_KERNEL,		\
			      CONFIG_GPIO_ASPEED_G7_INIT_PRIORITY, NULL);		\
	static const struct gpio_aspeed_g7_config gpio_aspeed_g7_cfg_##inst[] = {	\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), GPIO_ASPEED_G7_DEV_CFG)};	\
	static struct gpio_aspeed_g7_data gpio_aspeed_g7_data_##inst[] = {		\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), GPIO_ASPEED_G7_DEV_DATA)};	\
	static const struct gpio_aspeed_g7_device_cont DT_DRV_INST(inst) = {		\
		.cfg = gpio_aspeed_g7_cfg_##inst,					\
		.data = gpio_aspeed_g7_data_##inst,					\
	};										\
	enum {										\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), GPIO_ASPEED_G7_ENUM)	\
	};										\
	DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), GPIO_ASPEED_G7_DT_DEFINE)

DT_INST_FOREACH_STATUS_OKAY(ASPEED_G7_GPIO_DEVICE_INIT)
