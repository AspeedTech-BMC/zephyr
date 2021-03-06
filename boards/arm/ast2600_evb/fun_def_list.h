/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <devicetree.h>

#ifdef CONFIG_ARM_ICE
FUN_DEFINE(DT_NODELABEL(pinctrl_arm_ice_default), ARM_TRST, ARM_TCK, ARM_TDI, ARM_TDO, ARM_TMS)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart11), okay) && (CONFIG_UART_ASPEED || CONFIG_UART_NS16550)
FUN_DEFINE(DT_NODELABEL(pinctrl_uart11_default), TXD11, RXD11)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc0), okay) &&  CONFIG_ADC_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_adc0_default), ADC0)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc1_default), ADC1)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc2_default), ADC2)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc3_default), ADC3)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc4_default), ADC4)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc5_default), ADC5)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc6_default), ADC6)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc7_default), ADC7)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc1), okay) &&  CONFIG_ADC_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_adc8_default), ADC8)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc9_default), ADC9)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc10_default), ADC10)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc11_default), ADC11)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc12_default), ADC12)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc13_default), ADC13)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc14_default), ADC14)
FUN_DEFINE(DT_NODELABEL(pinctrl_adc15_default), ADC15)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm), okay) && CONFIG_PWM_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm0_default), PWM0)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm1_default), PWM1)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm2_default), PWM2)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm3_default), PWM3)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm4_default), PWM4)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm5_default), PWM5)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm6_default), PWM6)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm7_default), PWM7)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm8_default), PWM8)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm9_default), PWM9)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm10_default), PWM10)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm11_default), PWM11)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm12_default), PWM12)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm13_default), PWM13)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm14_default), PWM14)
FUN_DEFINE(DT_NODELABEL(pinctrl_pwm15_default), PWM15)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sgpiom0), okay) && CONFIG_GPIO_ASPEED_SGPIOM
FUN_DEFINE(DT_NODELABEL(pinctrl_sgpiom0_default), SGPM0CLK, SGPM0LD, SGPM0O, SGPM0I)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sgpiom1), okay) && CONFIG_GPIO_ASPEED_SGPIOM
FUN_DEFINE(DT_NODELABEL(pinctrl_sgpiom1_default), SGPM1CLK, SGPM1LD, SGPM1O, SGPM1I)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(tach), okay) && CONFIG_TACH_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_tach0_default), TACH0)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach1_default), TACH1)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach2_default), TACH2)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach3_default), TACH3)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach4_default), TACH4)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach5_default), TACH5)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach6_default), TACH6)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach7_default), TACH7)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach8_default), TACH8)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach9_default), TACH9)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach10_default), TACH10)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach11_default), TACH11)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach12_default), TACH12)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach13_default), TACH13)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach14_default), TACH14)
FUN_DEFINE(DT_NODELABEL(pinctrl_tach15_default), TACH15)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(jtag0), okay) &&  CONFIG_JTAG_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_jtagm0_default), MTRSTN1, MTDI1, MTCK1, MTMS1, MTDO1)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(jtag1), okay) &&  CONFIG_JTAG_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_jtagm1_default), MTRSTN2, MTDI2, MTCK2, MTMS2, MTDO2)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c0_default), SCL1, SDA1)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c1_default), SCL2, SDA2)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c2_default), SCL3, SDA3)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c3_default), SCL4, SDA4)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c4_default), SCL5, SDA5)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c5), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c5_default), SCL6, SDA6)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c6), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c6_default), SCL7, SDA7)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c7), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c7_default), SCL8, SDA8)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c8), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c8_default), SCL9, SDA9)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c9), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c9_default), SCL10, SDA10)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c10), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c10_default), SCL11, SDA11)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c11), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c11_default), SCL12, SDA12)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c12), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c12_default), SCL13, SDA13)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c13), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c13_default), SCL14, SDA14)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c14), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c14_default), SCL15, SDA15)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c15), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i2c15_default), SCL16, SDA16)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c0), okay) && CONFIG_I3C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i3c0_default), I3C1SCL, I3C1SDA)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c1), okay) && CONFIG_I3C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i3c1_default), I3C2SCL, I3C2SDA)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c2), okay) && CONFIG_I3C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i3c2_default), I3C3SCL, I3C3SDA)
FUN_DEFINE(DT_NODELABEL(pinctrl_hvi3c2_default), HVI3C3SCL, HVI3C3SDA)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c3), okay) && CONFIG_I3C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_i3c3_default), I3C4SCL, I3C4SDA)
FUN_DEFINE(DT_NODELABEL(pinctrl_hvi3c3_default), HVI3C4SCL, HVI3C4SDA)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c4), okay) && CONFIG_I3C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_hvi3c4_default), HVI3C5SCL, HVI3C5SDA)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c5), okay) && CONFIG_I3C_ASPEED
FUN_DEFINE(DT_NODELABEL(pinctrl_hvi3c5_default), HVI3C6SCL, HVI3C6SDA)
#endif
