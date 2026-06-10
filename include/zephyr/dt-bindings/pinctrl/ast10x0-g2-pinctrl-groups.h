/*
 * Copyright (c) ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _AST10X0_G2_PINCTRL_GROUPS_H_
#define _AST10X0_G2_PINCTRL_GROUPS_H_

/* I2C / SMBus / SMBus Filter */
#define pinctrl_i2c0_default   &pin_scl0 &pin_sda0
#define pinctrl_i2c1_default   &pin_scl1 &pin_sda1
#define pinctrl_i2c2_default   &pin_scl2 &pin_sda2
#define pinctrl_i2c3_default   &pin_scl3 &pin_sda3
#define pinctrl_i2c4_default   &pin_scl4 &pin_sda4
#define pinctrl_i2c5_default   &pin_scl5 &pin_sda5
#define pinctrl_i2c6_default   &pin_scl6 &pin_sda6
#define pinctrl_i2c7_default   &pin_scl7 &pin_sda7
#define pinctrl_i2c8_default   &pin_scl8 &pin_sda8
#define pinctrl_i2c9_default   &pin_scl9 &pin_sda9
#define pinctrl_i2c10_default  &pin_scl10 &pin_sda10
#define pinctrl_i2c11_default  &pin_scl11 &pin_sda11
#define pinctrl_i2c12_default  &pin_scl12 &pin_sda12
#define pinctrl_i2c13_default  &pin_scl13 &pin_sda13
#define pinctrl_smbflt1_default &pin_smbf1sclin &pin_smbf1sdain &pin_smbf1sclout &pin_smbf1sdaout
#define pinctrl_smbflt2_default &pin_smbf2sclin &pin_smbf2sdain &pin_smbf2sclout &pin_smbf2sdaout

/* I3C */
#define pinctrl_i3c0_default   &pin_i3cscl0 &pin_i3csda0
#define pinctrl_i3c1_default   &pin_i3cscl1 &pin_i3csda1
#define pinctrl_i3c2_default   &pin_i3cscl2 &pin_i3csda2
#define pinctrl_i3c3_default   &pin_i3cscl3 &pin_i3csda3
#define pinctrl_i3c4_default   &pin_i3cscl4 &pin_i3csda4
#define pinctrl_i3c5_default   &pin_i3cscl5 &pin_i3csda5
#define pinctrl_i3c6_default   &pin_i3cscl6 &pin_i3csda6
#define pinctrl_i3c7_default   &pin_i3cscl7 &pin_i3csda7

/* UART */
#define pinctrl_uart0_default  &pin_txd0 &pin_rxd0
#define pinctrl_uart1_default  &pin_txd1 &pin_rxd1
#define pinctrl_uart2_default  &pin_txd2 &pin_rxd2
#define pinctrl_uart3_default  &pin_txd3 &pin_rxd3
#define pinctrl_uart4_default  &pin_txd4 &pin_rxd4
#define pinctrl_uart5_default  &pin_txd5 &pin_rxd5
#define pinctrl_uart6_default  &pin_txd6 &pin_rxd6
#define pinctrl_uart7_default  &pin_txd7 &pin_rxd7
#define pinctrl_uart8_default  &pin_txd8 &pin_rxd8
#define pinctrl_uart9_default  &pin_txd9 &pin_rxd9
#define pinctrl_uart10_default &pin_txd10 &pin_rxd10
#define pinctrl_uart11_default &pin_txd11 &pin_rxd11
#define pinctrl_bmcuart_default &pin_bmctxd &pin_bmcrxd

/* PWM */
#define pinctrl_pwm0_default   &pin_pwm0
#define pinctrl_pwm1_default   &pin_pwm1
#define pinctrl_pwm2_default   &pin_pwm2
#define pinctrl_pwm3_default   &pin_pwm3
#define pinctrl_pwm4_default   &pin_pwm4
#define pinctrl_pwm5_default   &pin_pwm5
#define pinctrl_pwm6_default   &pin_pwm6
#define pinctrl_pwm7_default   &pin_pwm7
#define pinctrl_pwm8_default   &pin_pwm8
#define pinctrl_pwm9_default   &pin_pwm9
#define pinctrl_pwm10_default  &pin_pwm10
#define pinctrl_pwm11_default  &pin_pwm11
#define pinctrl_pwm12_default  &pin_pwm12
#define pinctrl_pwm13_default  &pin_pwm13
#define pinctrl_pwm14_default  &pin_pwm14
#define pinctrl_pwm15_default  &pin_pwm15

/* TACH */
#define pinctrl_tach0_default  &pin_tach0
#define pinctrl_tach1_default  &pin_tach1
#define pinctrl_tach2_default  &pin_tach2
#define pinctrl_tach3_default  &pin_tach3
#define pinctrl_tach4_default  &pin_tach4
#define pinctrl_tach5_default  &pin_tach5
#define pinctrl_tach6_default  &pin_tach6
#define pinctrl_tach7_default  &pin_tach7
#define pinctrl_tach8_default  &pin_tach8
#define pinctrl_tach9_default  &pin_tach9
#define pinctrl_tach10_default &pin_tach10
#define pinctrl_tach11_default &pin_tach11
#define pinctrl_tach12_default &pin_tach12
#define pinctrl_tach13_default &pin_tach13
#define pinctrl_tach14_default &pin_tach14
#define pinctrl_tach15_default &pin_tach15

/* ADC */
#define pinctrl_adc0_default   &pin_adc00
#define pinctrl_adc1_default   &pin_adc01
#define pinctrl_adc2_default   &pin_adc02
#define pinctrl_adc3_default   &pin_adc03
#define pinctrl_adc4_default   &pin_adc04
#define pinctrl_adc5_default   &pin_adc05
#define pinctrl_adc6_default   &pin_adc06
#define pinctrl_adc7_default   &pin_adc07
#define pinctrl_adc8_default   &pin_adc08
#define pinctrl_adc9_default   &pin_adc09
#define pinctrl_adc10_default  &pin_adc10
#define pinctrl_adc11_default  &pin_adc11
#define pinctrl_adc12_default  &pin_adc12
#define pinctrl_adc13_default  &pin_adc13
#define pinctrl_adc14_default  &pin_adc14
#define pinctrl_adc15_default  &pin_adc15

/* Thru bridge */
#define pinctrl_thru0_default  &pin_thruin0 &pin_thruout0
#define pinctrl_thru1_default  &pin_thruin1 &pin_thruout1
#define pinctrl_thru2_default  &pin_thruin2 &pin_thruout2
#define pinctrl_thru3_default  &pin_thruin3 &pin_thruout3

/* SGPM */
#define pinctrl_sgpm0_default  &pin_sgpm0clk &pin_sgpm0ld &pin_sgpm0out &pin_sgpm0in
#define pinctrl_sgpm1_default  &pin_sgpm1clk &pin_sgpm1ld &pin_sgpm1out &pin_sgpm1in

/* SPI master / slave */
#define pinctrl_spim0_default      &pin_spim0ck &pin_spim0dq0 &pin_spim0dq1 &pin_spim0cs0
#define pinctrl_spim0_quad_default &pin_spim0dq2 &pin_spim0dq3
#define pinctrl_spim0_cs1_default  &pin_spim0cs1
#define pinctrl_spim1_default      &pin_spim1ck &pin_spim1dq0 &pin_spim1dq1 &pin_spim1cs0
#define pinctrl_spim1_quad_default &pin_spim1dq2 &pin_spim1dq3
#define pinctrl_spim1_cs1_default  &pin_spim1cs1
#define pinctrl_spis0_default      &pin_spis0ck &pin_spis0dq0 &pin_spis0dq1 &pin_spis0cs0
#define pinctrl_spis0_quad_default &pin_spis0dq2 &pin_spis0dq3
#define pinctrl_spis0_cs1_default  &pin_spis0cs1
#define pinctrl_spis1_default      &pin_spis1ck &pin_spis1dq0 &pin_spis1dq1 &pin_spis1cs0
#define pinctrl_spis1_quad_default &pin_spis1dq2 &pin_spis1dq3

/* FWSPI quad lanes */
#define pinctrl_fwspi_quad_default &pin_fwspidq2 &pin_fwspidq3

/* eSPI */
#define pinctrl_espi_default   &pin_espid0 &pin_espid1 &pin_espid2 &pin_espid3 \
				&pin_espick &pin_espics &pin_espialt &pin_espirst

/* USB Port A (SCU3B0[1:0] SCU_USBA_SEL) */
#define pinctrl_usb2aud_default &pin_usba_usbuart_vhub
#define pinctrl_usb2ad_default  &pin_usba_vhub
#define pinctrl_usb2ah_default  &pin_usba_ehci
#define pinctrl_usb2au_default  &pin_usba_usbuart

/* USB Port B (SCU3B0[3:2] SCU_USBB_SEL) */
#define pinctrl_usb2bh_default  &pin_usbb_ehci
#define pinctrl_usb2bd_default  &pin_usbb_vhub

#endif /* _AST10X0_G2_PINCTRL_GROUPS_H_ */
