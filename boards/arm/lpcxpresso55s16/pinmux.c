/*
 * Copyright (c) 2020 Henrik Brix Andersen <henrik@brixandersen.dk>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <fsl_common.h>
#include <fsl_iocon.h>
#include <soc.h>

static int lpcxpresso_55s16_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pio0), okay)
	const struct device *port0 = DEVICE_DT_GET(DT_NODELABEL(pio0));

	__ASSERT_NO_MSG(device_is_ready(port0));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pio1), okay)
	const struct device *port1 = DEVICE_DT_GET(DT_NODELABEL(pio1));

	__ASSERT_NO_MSG(device_is_ready(port1));
#endif

#if DT_PHA_HAS_CELL(DT_ALIAS(sw0), gpios, pin)
	/* Wakeup button */
	uint32_t sw0_config = (
		IOCON_PIO_FUNC0 |
		IOCON_PIO_INV_DI |
		IOCON_PIO_DIGITAL_EN |
		IOCON_PIO_INPFILT_OFF |
		IOCON_PIO_OPENDRAIN_DI
		);
	pinmux_pin_set(port1, DT_GPIO_PIN(DT_ALIAS(sw0), gpios), sw0_config);
#endif

#if DT_PHA_HAS_CELL(DT_ALIAS(sw1), gpios, pin)
	/* USR button */
	uint32_t sw1_config = (
		IOCON_PIO_FUNC0 |
		IOCON_PIO_INV_DI |
		IOCON_PIO_DIGITAL_EN |
		IOCON_PIO_INPFILT_OFF |
		IOCON_PIO_OPENDRAIN_DI
		);
	pinmux_pin_set(port1, DT_GPIO_PIN(DT_ALIAS(sw1), gpios), sw1_config);
#endif

#if DT_PHA_HAS_CELL(DT_ALIAS(sw2), gpios, pin)
	/* ISP button */
	uint32_t sw2_config = (
		IOCON_PIO_FUNC0 |
		IOCON_PIO_INV_DI |
		IOCON_PIO_DIGITAL_EN |
		IOCON_PIO_INPFILT_OFF |
		IOCON_PIO_OPENDRAIN_DI
		);
	pinmux_pin_set(port0, DT_GPIO_PIN(DT_ALIAS(sw2), gpios), sw2_config);
#endif

#if DT_PHA_HAS_CELL(DT_ALIAS(led0), gpios, pin)
	/* Red LED */
	uint32_t led0_config = (
		IOCON_PIO_FUNC0 |
		IOCON_PIO_INV_DI |
		IOCON_PIO_DIGITAL_EN |
		IOCON_PIO_INPFILT_OFF |
		IOCON_PIO_OPENDRAIN_DI
		);
	pinmux_pin_set(port1, DT_GPIO_PIN(DT_ALIAS(led0), gpios), led0_config);
#endif

#if DT_PHA_HAS_CELL(DT_ALIAS(led1), gpios, pin)
	/* Green LED */
	uint32_t led1_config = (
		IOCON_PIO_FUNC0 |
		IOCON_PIO_INV_DI |
		IOCON_PIO_DIGITAL_EN |
		IOCON_PIO_INPFILT_OFF |
		IOCON_PIO_OPENDRAIN_DI
		);
	pinmux_pin_set(port1, DT_GPIO_PIN(DT_ALIAS(led1), gpios), led1_config);
#endif

#if DT_PHA_HAS_CELL(DT_ALIAS(led2), gpios, pin)
	/* Blue LED */
	uint32_t led2_config = (
		IOCON_PIO_FUNC0 |
		IOCON_PIO_INV_DI |
		IOCON_PIO_DIGITAL_EN |
		IOCON_PIO_INPFILT_OFF |
		IOCON_PIO_OPENDRAIN_DI
		);
	pinmux_pin_set(port1, DT_GPIO_PIN(DT_ALIAS(led2), gpios), led2_config);
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm0), nxp_lpc_usart, okay) && CONFIG_SERIAL
	/* USART0 RX, TX */
	uint32_t port0_pin29_config = (
			IOCON_PIO_FUNC1 |
			IOCON_PIO_MODE_INACT |
			IOCON_PIO_INV_DI |
			IOCON_PIO_DIGITAL_EN |
			IOCON_PIO_SLEW_STANDARD |
			IOCON_PIO_OPENDRAIN_DI
			);
	uint32_t port0_pin30_config = (
			IOCON_PIO_FUNC1 |
			IOCON_PIO_MODE_INACT |
			IOCON_PIO_INV_DI |
			IOCON_PIO_DIGITAL_EN |
			IOCON_PIO_SLEW_STANDARD |
			IOCON_PIO_OPENDRAIN_DI
			);
	pinmux_pin_set(port0, 29, port0_pin29_config);
	pinmux_pin_set(port0, 30, port0_pin30_config);
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm4), nxp_lpc_i2c, okay) && CONFIG_I2C
	/* PORT1 PIN20 is configured as FC4_TXD_SCL_MISO_WS */
	pinmux_pin_set(port1, 20, IOCON_PIO_FUNC5  |
				  IOCON_PIO_MODE_INACT |
				  IOCON_PIO_SLEW_STANDARD |
				  IOCON_PIO_INV_DI |
				  IOCON_PIO_DIGITAL_EN |
				  IOCON_PIO_OPENDRAIN_DI);

	/* PORT1 PIN21 is configured as FC4_RXD_SDA_MOSI_DATA */
	pinmux_pin_set(port1, 21, IOCON_PIO_FUNC5  |
				  IOCON_PIO_MODE_INACT |
				  IOCON_PIO_SLEW_STANDARD |
				  IOCON_PIO_INV_DI |
				  IOCON_PIO_DIGITAL_EN |
				  IOCON_PIO_OPENDRAIN_DI);
#endif

#ifdef CONFIG_FXOS8700_TRIGGER
	pinmux_pin_set(port1, 26, IOCON_PIO_FUNC0 |
				  IOCON_PIO_MODE_PULLUP |
				  IOCON_PIO_INV_DI |
				  IOCON_PIO_DIGITAL_EN |
				  IOCON_PIO_INPFILT_OFF |
				  IOCON_PIO_SLEW_STANDARD |
				  IOCON_PIO_OPENDRAIN_DI);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(hs_lspi), okay) && CONFIG_SPI
	/* PORT0 PIN26 is configured as HS_SPI_MOSI */
	pinmux_pin_set(port0, 26, IOCON_PIO_FUNC9 |
				  IOCON_PIO_MODE_PULLUP |
				  IOCON_PIO_INV_DI |
				  IOCON_PIO_DIGITAL_EN |
				  IOCON_PIO_SLEW_STANDARD |
				  IOCON_PIO_OPENDRAIN_DI);

	/* PORT1 PIN1 is configured as HS_SPI_SSEL1 */
	pinmux_pin_set(port1,  1, IOCON_PIO_FUNC5 |
				  IOCON_PIO_MODE_PULLUP |
				  IOCON_PIO_INV_DI |
				  IOCON_PIO_DIGITAL_EN |
				  IOCON_PIO_SLEW_STANDARD |
				  IOCON_PIO_OPENDRAIN_DI);

	/* PORT1 PIN2 is configured as HS_SPI_SCK */
	pinmux_pin_set(port1,  2, IOCON_PIO_FUNC6 |
				  IOCON_PIO_MODE_PULLUP |
				  IOCON_PIO_INV_DI |
				  IOCON_PIO_DIGITAL_EN |
				  IOCON_PIO_SLEW_STANDARD |
				  IOCON_PIO_OPENDRAIN_DI);

	/* PORT1 PIN3 is configured as HS_SPI_MISO */
	pinmux_pin_set(port1,  3, IOCON_PIO_FUNC6 |
				  IOCON_PIO_MODE_PULLUP |
				  IOCON_PIO_INV_DI |
				  IOCON_PIO_DIGITAL_EN |
				  IOCON_PIO_SLEW_STANDARD |
				  IOCON_PIO_OPENDRAIN_DI);
#endif

	return 0;
}

SYS_INIT(lpcxpresso_55s16_pinmux_init, PRE_KERNEL_1,
	 CONFIG_PINMUX_INIT_PRIORITY);
