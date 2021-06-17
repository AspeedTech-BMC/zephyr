/*
 * Copyright (c) 2020 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc11u6x_gpio

/**
 * @file
 * @brief GPIO driver for NXP LPC11U6X SoCs
 *
 * This driver allows to configure the GPIOs found on the LPC11U6x MCUs.
 *
 * @note See the UM10732 LPC11U6x/E6x user manual for register definitions.
 */

#include <drivers/clock_control.h>
#include <drivers/gpio.h>
#include <drivers/pinmux.h>

#include <dt-bindings/pinctrl/lpc11u6x-pinctrl.h>

#include "gpio_utils.h"

#define DEV_CFG(dev)  ((const struct gpio_lpc11u6x_config *) \
		       ((dev)->config))
#define DEV_DATA(dev) ((struct gpio_lpc11u6x_data *) \
		       ((dev)->data))

/* Offset from syscon base address. */
#define LPC11U6X_PINTSEL_REGS	0x178

/* Offsets from GPIO base address. */
#define LPC11U6X_GPIO_REGS	0x2000
#define LPC11U6X_PINT_REGS	0x4000

/**
 * @brief Structure mapping the GPIO registers.
 *
 * @note The byte and word pin registers are not included because they are
 *       not used by this driver. A 0x2000 offset is applied to skip them.
 */
struct lpc11u6x_gpio_regs {
	volatile uint32_t dir[3];
	volatile uint32_t _unused1[29];
	volatile uint32_t mask[3];
	volatile uint32_t _unused2[29];
	volatile uint32_t pin[3];
	volatile uint32_t _unused3[29];
	volatile uint32_t mpin[3];
	volatile uint32_t _unused4[29];
	volatile uint32_t set[3];
	volatile uint32_t _unused5[29];
	volatile uint32_t clr[3];
	volatile uint32_t _unused6[29];
	volatile uint32_t not[3];
};

/**
 * @brief Structure mapping the PINT registers.
 */
struct lpc11u6x_pint_regs {
	volatile uint32_t isel;
	volatile uint32_t ienr;
	volatile uint32_t sienr;
	volatile uint32_t cienr;
	volatile uint32_t ienf;
	volatile uint32_t sienf;
	volatile uint32_t cienf;
	volatile uint32_t rise;
	volatile uint32_t fall;
	volatile uint32_t ist;
	volatile uint32_t pmctrl;
	volatile uint32_t pmsrc;
	volatile uint32_t pmcfg;
};

/**
 * @brief Structure for resources and information shared between GPIO ports.
 *
 * This structure is included by all the per-port private configuration.
 * It gathers all the resources and information shared between all the GPIO
 * ports: GPIO and SYSCON registers base addresses, clock name and subsystem.
 */
struct gpio_lpc11u6x_shared {
	char *clock_controller_name;
	clock_control_subsys_t clock_subsys;
	uint32_t gpio_base;
	uint32_t syscon_base;
	uint8_t nirqs;
};

struct gpio_lpc11u6x_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	const struct gpio_lpc11u6x_shared *shared;
	char *pinmux_name;
	uint8_t port_num;
	uint8_t ngpios;
};

struct gpio_lpc11u6x_data {
	/* gpio_driver_data needs to be first. */
	struct gpio_driver_data common;
	const struct device *pinmux_dev;
	sys_slist_t cb_list;
};

static int gpio_lpc11u6x_pin_configure(const struct device *port,
				       gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_lpc11u6x_config *config = DEV_CFG(port);
	struct gpio_lpc11u6x_data *data = DEV_DATA(port);
	struct lpc11u6x_gpio_regs *gpio_regs = (struct lpc11u6x_gpio_regs *)
		(config->shared->gpio_base + LPC11U6X_GPIO_REGS);
	uint8_t port_num = config->port_num;
	uint32_t func;
	int ret;

	if (pin >= config->ngpios) {
		return -EINVAL;
	}

	/*
	 * PIO0_4 and PIO0_5 are "true" open drain pins muxed with the I2C port
	 * 0. They still can be configured as GPIOs but only in open drain mode
	 * and with no pull-down or pull-up resitor enabled.
	 */
	if (port_num == 0 && (pin == 4 || pin == 5) &&
		((flags & GPIO_OPEN_DRAIN) == 0 ||
		 (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)))) {
		return -EINVAL;
	}

	/*
	 * For PIO0_0 and PIO0_[10-15] function 1 enables GPIO mode. For all
	 * the other pins, function 0 must be selected.
	 */
	if (port_num == 0 && (pin == 0 || (pin >= 10 && pin <= 15))) {
		func = IOCON_FUNC1;
	} else {
		func = IOCON_FUNC0;
	}

	if (flags & GPIO_SINGLE_ENDED) {
		/* Open source mode is not supported. */
		if (flags & GPIO_LINE_OPEN_DRAIN)
			func |= IOCON_OPENDRAIN_EN;
		else
			return -ENOTSUP;
	}

	if (flags & GPIO_PULL_UP) {
		func |= IOCON_MODE_PULLUP;
	} else if (flags & GPIO_PULL_DOWN) {
		func |= IOCON_MODE_PULLDOWN;
	} else {
		func |= IOCON_MODE_INACT;
	}

	ret = pinmux_pin_set(data->pinmux_dev, pin, func);
	if (ret < 0) {
		return ret;
	}

	/* Initial output value. */
	if (flags & GPIO_OUTPUT_INIT_HIGH) {
		gpio_regs->set[port_num] |= BIT(pin);
	}

	if (flags & GPIO_OUTPUT_INIT_LOW) {
		gpio_regs->clr[port_num] |= BIT(pin);
	}

	/*
	 * TODO: maybe configure the STARTERP0 register to allow wake-up from
	 * deep-sleep or power-down modes.
	 */

	/* Configure GPIO direction. */
	WRITE_BIT(gpio_regs->dir[port_num], pin, flags & GPIO_OUTPUT);

	return 0;
}

static int gpio_lpc11u6x_port_get_raw(const struct device *port,
				      gpio_port_value_t *value)
{
	const struct gpio_lpc11u6x_config *config = DEV_CFG(port);
	struct lpc11u6x_gpio_regs *gpio_regs = (struct lpc11u6x_gpio_regs *)
		(config->shared->gpio_base + LPC11U6X_GPIO_REGS);

	*value = gpio_regs->pin[config->port_num];

	return 0;
}

static int gpio_lpc11u6x_port_set_masked_raw(const struct device *port,
					     gpio_port_pins_t mask,
					     gpio_port_value_t value)
{
	const struct gpio_lpc11u6x_config *config = DEV_CFG(port);
	struct lpc11u6x_gpio_regs *gpio_regs = (struct lpc11u6x_gpio_regs *)
		(config->shared->gpio_base + LPC11U6X_GPIO_REGS);
	uint8_t port_num = config->port_num;
	uint32_t orig_mask;

	orig_mask = gpio_regs->mask[port_num];
	/* Apply inverted mask (bit set to 1 masks the pin). */
	gpio_regs->mask[port_num] = ~mask;
	compiler_barrier();
	/* Update pins values. */
	gpio_regs->mpin[port_num] = value;
	compiler_barrier();
	/* Restore original mask. */
	gpio_regs->mask[port_num] = orig_mask;
	compiler_barrier();

	return 0;
}

static int gpio_lpc11u6x_port_set_bits_raw(const struct device *port,
					   gpio_port_pins_t pins)
{
	const struct gpio_lpc11u6x_config *config = DEV_CFG(port);
	struct lpc11u6x_gpio_regs *gpio_regs = (struct lpc11u6x_gpio_regs *)
		(config->shared->gpio_base + LPC11U6X_GPIO_REGS);

	gpio_regs->set[config->port_num] = pins;

	return 0;
}

static int gpio_lpc11u6x_port_clear_bits_raw(const struct device *port,
					     gpio_port_pins_t pins)
{
	const struct gpio_lpc11u6x_config *config = DEV_CFG(port);
	struct lpc11u6x_gpio_regs *gpio_regs = (struct lpc11u6x_gpio_regs *)
		(config->shared->gpio_base + LPC11U6X_GPIO_REGS);

	gpio_regs->clr[config->port_num] = pins;

	return 0;
}

static int gpio_lpc11u6x_port_toggle_bits(const struct device *port,
					  gpio_port_pins_t pins)
{
	const struct gpio_lpc11u6x_config *config = DEV_CFG(port);
	struct lpc11u6x_gpio_regs *gpio_regs = (struct lpc11u6x_gpio_regs *)
		(config->shared->gpio_base + LPC11U6X_GPIO_REGS);

	gpio_regs->not[config->port_num] = pins;

	return 0;
}

/**
 * @brief Attach a free interrupt line to a GPIO.
 *
 * @param shared   Pointer to a structure shared between all the GPIO ports.
 * @param intpin   GPIO port and pin numbers encoded into a value compatible
 *                 with the INTPIN register (included in the PINTSEL register).
 *
 * @retval >0      Number of the attached interrupt on success.
 * @retval -EBUSY  All the interrupt lines are already attached.
 */
static int
pintsel_attach(const struct gpio_lpc11u6x_shared *shared, uint8_t intpin)
{
	uint8_t irq;
	int ret = -EBUSY;
	uint32_t *pintsel_reg =
		(uint32_t *) (shared->syscon_base + LPC11U6X_PINTSEL_REGS);

	for (irq = 0; irq < shared->nirqs; irq++) {
		/* GPIO already attached. */
		if ((pintsel_reg[irq] & BIT_MASK(5)) == intpin) {
			return irq;
		}

		if (ret < 0 && (pintsel_reg[irq] & BIT_MASK(5)) == 0) {
			ret = irq;
		}
	}
	/* Attach GPIO to the first free interrupt found if any. */
	if (ret >= 0) {
		pintsel_reg[ret] = intpin;
	}

	return ret;
}

/**
 * @brief Detach an interrupt line from a GPIO.
 *
 * @param shared   Pointer to a structure shared between all the GPIO ports.
 * @param intpin   GPIO port and pin numbers encoded into a value compatible
 *                 with the INTPIN register (included in the PINTSEL register).
 *
 * @retval >0      Number of the detached interrupt on success.
 * @retval -EINVAL No attached interrupt found for the requested GPIO.
 */
static int
pintsel_detach(const struct gpio_lpc11u6x_shared *shared, uint8_t intpin)
{
	uint8_t irq;
	uint32_t *pintsel_reg =
		(uint32_t *) (shared->syscon_base + LPC11U6X_PINTSEL_REGS);

	for (irq = 0; irq < shared->nirqs; irq++) {
		if ((pintsel_reg[irq] & BIT_MASK(5)) == intpin) {
			pintsel_reg[irq] = 0;
			return irq;
		}
	}
	return -EINVAL;
}

static int gpio_lpc11u6x_pin_interrupt_configure(const struct device *port,
						 gpio_pin_t pin,
						 enum gpio_int_mode mode,
						 enum gpio_int_trig trig)
{
	const struct gpio_lpc11u6x_config *config = DEV_CFG(port);
	struct lpc11u6x_pint_regs *pint_regs = (struct lpc11u6x_pint_regs *)
		(config->shared->gpio_base + LPC11U6X_PINT_REGS);
	uint8_t intpin;
	int irq;

	if (pin >= config->ngpios) {
		return -EINVAL;
	}

	/*
	 * Because the PINTSEL register only have 6 bits to encode a pin
	 * number, then PIO2_8 to PIO2_23 can't be attacehd to an interrupt
	 * line.
	 */
	if (config->port_num == 2 && pin > 7) {
		return -ENOTSUP;
	}

	/*
	 * Convert the requested GPIO port and pin numbers into a value
	 * compatible with the INTPIN register (included in the PINTSEL
	 * register).
	 */
	intpin = pin;
	if (config->port_num == 1) {
		intpin += 24;
	} else if (config->port_num == 2) {
		intpin += 56;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		irq = pintsel_detach(config->shared, intpin);
	} else {
		irq = pintsel_attach(config->shared, intpin);
	}
	if (irq < 0) {
		return irq;
	}

	switch (mode) {
	case GPIO_INT_MODE_DISABLED:
		pint_regs->isel &= ~BIT(irq);
		pint_regs->cienr |= BIT(irq);
		pint_regs->cienf |= BIT(irq);
		break;
	case GPIO_INT_MODE_EDGE:
		/* Select edge interrupt mode. */
		pint_regs->isel &= ~BIT(irq);
		/* Enable interrupts on falling and/or rising edges. */
		if (trig & GPIO_INT_TRIG_LOW)
			pint_regs->sienf |= BIT(irq);
		else
			pint_regs->cienf |= BIT(irq);
		if (trig & GPIO_INT_TRIG_HIGH)
			pint_regs->sienr |= BIT(irq);
		else
			pint_regs->cienr |= BIT(irq);
		break;
	case GPIO_INT_MODE_LEVEL:
		/* Select level interrupt mode. */
		pint_regs->isel |= BIT(irq);
		/* Set active level. */
		if (trig & GPIO_INT_TRIG_LOW)
			pint_regs->cienf |= BIT(irq);
		else
			pint_regs->sienf |= BIT(irq);
		/* Enable level interrupt. */
		pint_regs->sienr |= BIT(irq);
		break;
	default:
		return -ENOTSUP;
	}

	/* Clear interrupt status. */
	pint_regs->ist |= BIT(irq);

	return 0;
}

static int gpio_lpc11u6x_manage_callback(const struct device *port,
					 struct gpio_callback *cb, bool set)
{
	struct gpio_lpc11u6x_data *data = DEV_DATA(port);

	return gpio_manage_callback(&data->cb_list, cb, set);
}

static uint32_t gpio_lpc11u6x_get_pending_int(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static void gpio_lpc11u6x_isr(const void *arg)
{
	struct gpio_lpc11u6x_shared *shared =
		(struct gpio_lpc11u6x_shared *)arg;
	struct lpc11u6x_pint_regs *pint_regs = (struct lpc11u6x_pint_regs *)
		(shared->gpio_base + LPC11U6X_PINT_REGS);
	uint32_t *pintsel_reg =
		(uint32_t *) (shared->syscon_base + LPC11U6X_PINTSEL_REGS);
	uint8_t irq;
	uint32_t pins[3] = { 0, 0, 0 };
	const struct device *port;
	struct gpio_lpc11u6x_data *data;

	for (irq = 0; irq < shared->nirqs; irq++) {
		uint32_t intpin;

		if ((pint_regs->ist & BIT(irq)) == 0) {
			continue;
		}

		/* Clear interrupt status. */
		pint_regs->ist |= BIT(irq);

		/*
		 * Look in the PINTSEL register to retrieve the "intpin" value
		 * attached with the requested interrupt. Extract the GPIO
		 * port and pin numbers from this "intpin" value and store them
		 * into an "active pins" mask.
		 */
		intpin = pintsel_reg[irq] & BIT_MASK(5);
		if (intpin < 24) {
			pins[0] |= BIT(intpin);
		} else if (intpin < 56) {
			pins[1] |= BIT(intpin - 24);
		} else {
			pins[2] |= BIT(intpin - 56);
		}
	}
	/* For each port with active pins, fire the GPIO interrupt callbacks. */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
	if (pins[0]) {
		port = DEVICE_DT_GET(DT_NODELABEL(gpio0));
		data = port->data;
		gpio_fire_callbacks(&data->cb_list, port, pins[0]);
	}
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio1), okay)
	if (pins[1]) {
		port = DEVICE_DT_GET(DT_NODELABEL(gpio1));
		data = port->data;
		gpio_fire_callbacks(&data->cb_list, port, pins[1]);
	}
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio2), okay)
	if (pins[2]) {
		port = DEVICE_DT_GET(DT_NODELABEL(gpio2));
		data = port->data;
		gpio_fire_callbacks(&data->cb_list, port, pins[2]);
	}
#endif
}

static const struct gpio_driver_api gpio_lpc11u6x_driver_api = {
	.pin_configure = gpio_lpc11u6x_pin_configure,
	.port_get_raw = gpio_lpc11u6x_port_get_raw,
	.port_set_masked_raw = gpio_lpc11u6x_port_set_masked_raw,
	.port_set_bits_raw = gpio_lpc11u6x_port_set_bits_raw,
	.port_clear_bits_raw = gpio_lpc11u6x_port_clear_bits_raw,
	.port_toggle_bits = gpio_lpc11u6x_port_toggle_bits,
	.pin_interrupt_configure = gpio_lpc11u6x_pin_interrupt_configure,
	.manage_callback = gpio_lpc11u6x_manage_callback,
	.get_pending_int = gpio_lpc11u6x_get_pending_int,
};

/*
 * Note that the first DT instance is used to initialize the resources
 * shared between all the ports (IRQ lines, clock).
 */

static const struct gpio_lpc11u6x_shared gpio_lpc11u6x_shared = {
	.clock_controller_name = DT_LABEL(DT_INST_PHANDLE(0, clocks)),
	.clock_subsys = (clock_control_subsys_t) DT_INST_PHA(0, clocks, clkid),
	.gpio_base = DT_INST_REG_ADDR_BY_IDX(0, 0),
	.syscon_base = DT_INST_REG_ADDR_BY_IDX(0, 1),
	.nirqs = DT_NUM_IRQS(DT_DRV_INST(0)),
};

#define IRQ_INIT(n)							\
do {							                \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, n, irq),			\
		    DT_INST_IRQ_BY_IDX(0, n, priority),			\
		    gpio_lpc11u6x_isr, &gpio_lpc11u6x_shared, 0);	\
	irq_enable(DT_INST_IRQ_BY_IDX(0, n, irq));			\
} while (0)

static int gpio_lpc11u6x_init(const struct device *dev)
{
	const struct gpio_lpc11u6x_config *config = DEV_CFG(dev);
	struct gpio_lpc11u6x_data *data = DEV_DATA(dev);
	const struct device *clock_dev;
	int ret;
	static bool gpio_ready;

	/* Retrieve pinmux device. */
	data->pinmux_dev = device_get_binding(config->pinmux_name);
	if (!data->pinmux_dev) {
		return -EINVAL;
	}

	/* Initialize shared resources only once. */
	if (gpio_ready) {
		return 0;
	}

	/* Enable GPIO and PINT clocks. */
	clock_dev = device_get_binding(config->shared->clock_controller_name);
	if (!clock_dev) {
		return -ENODEV;
	}

	ret = clock_control_on(clock_dev, config->shared->clock_subsys);
	if (ret < 0) {
		return ret;
	}

#if DT_INST_IRQ_HAS_IDX(0, 0)
	IRQ_INIT(0);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 1)
	IRQ_INIT(1);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 2)
	IRQ_INIT(2);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 3)
	IRQ_INIT(3);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 4)
	IRQ_INIT(4);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 5)
	IRQ_INIT(5);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 6)
	IRQ_INIT(6);
#endif
#if DT_INST_IRQ_HAS_IDX(0, 7)
	IRQ_INIT(7);
#endif
	gpio_ready = true;

	return 0;
}

#define GPIO_LPC11U6X_INIT(id)						\
static const struct gpio_lpc11u6x_config				\
			gpio_lpc11u6x_config_##id = {			\
	.common = {							\
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(	\
					DT_NODELABEL(gpio##id)),	\
	},								\
	.shared = &gpio_lpc11u6x_shared,				\
	.port_num = id,							\
	.pinmux_name = DT_LABEL(DT_PHANDLE(DT_NODELABEL(gpio##id),	\
				pinmux_port)),				\
	.ngpios = DT_PROP(DT_NODELABEL(gpio##id), ngpios),		\
};									\
									\
static struct gpio_lpc11u6x_data gpio_lpc11u6x_data_##id;		\
									\
DEVICE_DT_DEFINE(DT_NODELABEL(gpio##id),				\
		    &gpio_lpc11u6x_init,				\
		    NULL,						\
		    &gpio_lpc11u6x_data_##id,				\
		    &gpio_lpc11u6x_config_##id,				\
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
		    &gpio_lpc11u6x_driver_api)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
GPIO_LPC11U6X_INIT(0);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio1), okay)
GPIO_LPC11U6X_INIT(1);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio2), okay)
GPIO_LPC11U6X_INIT(2);
#endif
