/*
 * Copyright 2017, 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <fsl_port.h>

static int frdm_kw41z_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(porta), okay)
	__unused const struct device *porta =
		DEVICE_DT_GET(DT_NODELABEL(porta));
	__ASSERT_NO_MSG(device_is_ready(porta));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portb), okay)
	__unused const struct device *portb =
		DEVICE_DT_GET(DT_NODELABEL(portb));
	__ASSERT_NO_MSG(device_is_ready(portb));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(portc), okay)
	__unused const struct device *portc =
		DEVICE_DT_GET(DT_NODELABEL(portc));
	__ASSERT_NO_MSG(device_is_ready(portc));
#endif

	/* Red, green, blue LEDs. Note the red LED and accel INT1 are both
	 * wired to PTC1.
	 */
#if defined(CONFIG_PWM) && DT_NODE_HAS_STATUS(DT_NODELABEL(tpm0), okay)
	pinmux_pin_set(portc,  1, PORT_PCR_MUX(kPORT_MuxAlt5));
#endif

#if defined(CONFIG_PWM) && DT_NODE_HAS_STATUS(DT_NODELABEL(tpm2), okay)
	pinmux_pin_set(porta, 19, PORT_PCR_MUX(kPORT_MuxAlt5));
	pinmux_pin_set(porta, 18, PORT_PCR_MUX(kPORT_MuxAlt5));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) && CONFIG_I2C
	/* I2C1 SCL, SDA */
	pinmux_pin_set(portc,  2, PORT_PCR_MUX(kPORT_MuxAlt3)
					| PORT_PCR_PS_MASK);
	pinmux_pin_set(portc,  3, PORT_PCR_MUX(kPORT_MuxAlt3)
					| PORT_PCR_PS_MASK);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc0), okay) && CONFIG_ADC
	/* ADC0_SE3 */
	pinmux_pin_set(portb,  2, PORT_PCR_MUX(kPORT_PinDisabledOrAnalog));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart0), okay) && CONFIG_SERIAL
	/* UART0 RX, TX */
	pinmux_pin_set(portc,  6, PORT_PCR_MUX(kPORT_MuxAlt4));
	pinmux_pin_set(portc,  7, PORT_PCR_MUX(kPORT_MuxAlt4));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay) && CONFIG_SPI
	/* SPI0 CS0, SCK, SOUT, SIN */
	pinmux_pin_set(portc, 16, PORT_PCR_MUX(kPORT_MuxAlt2));
	pinmux_pin_set(portc, 17, PORT_PCR_MUX(kPORT_MuxAlt2));
	pinmux_pin_set(portc, 18, PORT_PCR_MUX(kPORT_MuxAlt2));
	pinmux_pin_set(portc, 19, PORT_PCR_MUX(kPORT_MuxAlt2));
#endif

	return 0;
}

SYS_INIT(frdm_kw41z_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
