/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <fsl_port.h>

static int hexiwear_kw40z_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);

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

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart0), okay) && CONFIG_SERIAL
	/* UART0 RX, TX */
	pinmux_pin_set(portc,  6, PORT_PCR_MUX(kPORT_MuxAlt4));
	pinmux_pin_set(portc,  7, PORT_PCR_MUX(kPORT_MuxAlt4));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc0), okay) && CONFIG_ADC
	/* ADC0_SE1 */
	pinmux_pin_set(portb,  1, PORT_PCR_MUX(kPORT_PinDisabledOrAnalog));
#endif

	return 0;
}

SYS_INIT(hexiwear_kw40z_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
