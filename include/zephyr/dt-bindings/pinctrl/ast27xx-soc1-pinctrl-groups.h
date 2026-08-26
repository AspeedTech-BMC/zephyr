/*
 * Copyright (c) ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _AST27XX_SOC1_PINCTRL_GROUPS_H_
#define _AST27XX_SOC1_PINCTRL_GROUPS_H_

/* UART */
#define pinctrl_uart0_default            &pin_txd0 &pin_rxd0
#define pinctrl_uart0_modem_default      &pin_ncts0 &pin_ndcd0 &pin_ndsr0 &pin_nri0 &pin_ndtr0 &pin_nrts0 &pin_txd0 &pin_rxd0
#define pinctrl_uart1_default            &pin_txd1 &pin_rxd1
#define pinctrl_uart1_modem_default      &pin_ncts1 &pin_ndcd1 &pin_ndsr1 &pin_nri1 &pin_ndtr1 &pin_nrts1 &pin_txd1 &pin_rxd1
#define pinctrl_uart10_default           &pin_txd10 &pin_rxd10
#define pinctrl_uart11_default           &pin_txd11 &pin_rxd11
#define pinctrl_uart2_default            &pin_txd2 &pin_rxd2
#define pinctrl_uart3_default            &pin_txd3 &pin_rxd3
#define pinctrl_uart5_default            &pin_txd5 &pin_rxd5
#define pinctrl_uart5_modem_default      &pin_ncts5 &pin_ndcd5 &pin_ndsr5 &pin_nri5 &pin_ndtr5 &pin_nrts5 &pin_txd5 &pin_rxd5
#define pinctrl_uart6_default            &pin_txd6 &pin_rxd6
#define pinctrl_uart6_modem_default      &pin_ncts6 &pin_ndcd6 &pin_ndsr6 &pin_nri6 &pin_ndtr6 &pin_nrts6 &pin_txd6 &pin_rxd6
#define pinctrl_uart7_default            &pin_txd7 &pin_rxd7
#define pinctrl_uart8_default            &pin_txd8 &pin_rxd8
#define pinctrl_uart9_default            &pin_txd9 &pin_rxd9

/* SPI */
#define pinctrl_fwspi_default            &pin_fwspics0 &pin_fwspiclk &pin_fwspimosi &pin_fwspimiso
#define pinctrl_fwspi_quad_default       &pin_fwspidq2 &pin_fwspidq3
#define pinctrl_fwspi_cs1_default        &pin_fwspics1
#define pinctrl_fwspi_cs2_default        &pin_fwspics2
#define pinctrl_fwspi_abr_default        &pin_fwspi_abr
#define pinctrl_spi0_default             &pin_spi0ck &pin_spi0mosi &pin_spi0miso
#define pinctrl_spi0_quad_default        &pin_spi0dq2 &pin_spi0dq3
#define pinctrl_spi0_cs1_default         &pin_spi0cs1
#define pinctrl_spi0_abr_default         &pin_spi0abr
#define pinctrl_spi0_wpn_default         &pin_spi0wpn
#define pinctrl_spi1_default             &pin_spi1ck &pin_spi1mosi &pin_spi1miso
#define pinctrl_spi1_quad_default        &pin_spi1dq2 &pin_spi1dq3
#define pinctrl_spi1_cs1_default         &pin_spi1cs1
#define pinctrl_spi1_abr_default         &pin_spi1abr
#define pinctrl_spi1_wpn_default         &pin_spi1wpn
#define pinctrl_spi2_default             &pin_spi2ck &pin_spi2mosi &pin_spi2miso
#define pinctrl_spi2_quad_default        &pin_spi2dq2 &pin_spi2dq3
#define pinctrl_spi2_cs1_default         &pin_spi2cs1
#define pinctrl_spim0_default            &pin_spim0_0 &pin_spim0_1 &pin_spim0_2 &pin_spim0_3 &pin_spim0_4 &pin_spim0_5 &pin_spim0_6
#define pinctrl_spim1_default            &pin_spim1_0 &pin_spim1_1 &pin_spim1_2 &pin_spim1_3 &pin_spim1_4 &pin_spim1_5 &pin_spim1_6 &pin_spim1_7

/* I2C */
#define pinctrl_i2c0_default             &pin_scl0 &pin_sda0
#define pinctrl_di2c0_default            &pin_di2c0scl &pin_di2c0sda
#define pinctrl_i2c1_default             &pin_scl1 &pin_sda1
#define pinctrl_di2c1_default            &pin_di2c1scl &pin_di2c1sda
#define pinctrl_i2c10_default            &pin_scl10 &pin_sda10
#define pinctrl_di2c10_default           &pin_di2c10scl &pin_di2c10sda
#define pinctrl_i2c11_default            &pin_scl11 &pin_sda11
#define pinctrl_di2c11_default           &pin_di2c11scl &pin_di2c11sda
#define pinctrl_i2c12_default            &pin_scl12 &pin_sda12
#define pinctrl_di2c12_default           &pin_di2c12scl &pin_di2c12sda
#define pinctrl_i2c13_default            &pin_scl13 &pin_sda13
#define pinctrl_di2c13_default           &pin_di2c13scl &pin_di2c13sda
#define pinctrl_i2c14_default            &pin_scl14 &pin_sda14
#define pinctrl_di2c14_default           &pin_di2c14scl &pin_di2c14sda
#define pinctrl_i2c15_default            &pin_scl15 &pin_sda15
#define pinctrl_di2c15_default           &pin_di2c15scl &pin_di2c15sda
#define pinctrl_i2c2_default             &pin_scl2 &pin_sda2
#define pinctrl_di2c2_default            &pin_di2c2scl &pin_di2c2sda
#define pinctrl_i2c3_default             &pin_scl3 &pin_sda3
#define pinctrl_di2c3_default            &pin_di2c3scl &pin_di2c3sda
#define pinctrl_i2c4_default             &pin_scl4 &pin_sda4
#define pinctrl_i2c5_default             &pin_scl5 &pin_sda5
#define pinctrl_i2c6_default             &pin_scl6 &pin_sda6
#define pinctrl_i2c7_default             &pin_scl7 &pin_sda7
#define pinctrl_i2c8_default             &pin_scl8 &pin_sda8
#define pinctrl_di2c8_default            &pin_di2c8scl &pin_di2c8sda
#define pinctrl_i2c9_default             &pin_scl9 &pin_sda9
#define pinctrl_di2c9_default            &pin_di2c9scl &pin_di2c9sda
#define pinctrl_ltpi_ps_i2c0_default     &pin_ltpi_ps_i2c0scl &pin_ltpi_ps_i2c0sda
#define pinctrl_ltpi_ps_i2c1_default     &pin_ltpi_ps_i2c1scl &pin_ltpi_ps_i2c1sda
#define pinctrl_ltpi_ps_i2c2_default     &pin_ltpi_ps_i2c2scl &pin_ltpi_ps_i2c2sda
#define pinctrl_ltpi_ps_i2c3_default     &pin_ltpi_ps_i2c3scl &pin_ltpi_ps_i2c3sda

/* I3C */
#define pinctrl_i3c0_default             &pin_hvi3c0scl &pin_hvi3c0sda
#define pinctrl_i3c1_default             &pin_hvi3c1scl &pin_hvi3c1sda
#define pinctrl_i3c10_default            &pin_i3c10scl &pin_i3c10sda
#define pinctrl_i3c11_default            &pin_i3c11scl &pin_i3c11sda
#define pinctrl_i3c12_default            &pin_hvi3c12scl &pin_hvi3c12sda
#define pinctrl_i3c13_default            &pin_hvi3c13scl &pin_hvi3c13sda
#define pinctrl_i3c14_default            &pin_hvi3c14scl &pin_hvi3c14sda
#define pinctrl_i3c15_default            &pin_hvi3c15scl &pin_hvi3c15sda
#define pinctrl_i3c2_default             &pin_hvi3c2scl &pin_hvi3c2sda
#define pinctrl_i3c3_default             &pin_hvi3c3scl &pin_hvi3c3sda
#define pinctrl_i3c4_default             &pin_i3c4scl &pin_i3c4sda
#define pinctrl_i3c5_default             &pin_i3c5scl &pin_i3c5sda
#define pinctrl_i3c6_default             &pin_i3c6scl &pin_i3c6sda
#define pinctrl_i3c7_default             &pin_i3c7scl &pin_i3c7sda
#define pinctrl_i3c8_default             &pin_i3c8scl &pin_i3c8sda
#define pinctrl_i3c9_default             &pin_i3c9scl &pin_i3c9sda

/* PWM */
#define pinctrl_pwm0_default             &pin_pwm0
#define pinctrl_pwm1_default             &pin_pwm1
#define pinctrl_pwm10_default            &pin_pwm10
#define pinctrl_pwm11_default            &pin_pwm11
#define pinctrl_pwm12_default            &pin_pwm12
#define pinctrl_pwm13_default            &pin_pwm13
#define pinctrl_pwm14_default            &pin_pwm14
#define pinctrl_pwm15_default            &pin_pwm15
#define pinctrl_pwm2_default             &pin_pwm2
#define pinctrl_pwm3_default             &pin_pwm3
#define pinctrl_pwm4_default             &pin_pwm4
#define pinctrl_pwm5_default             &pin_pwm5
#define pinctrl_pwm6_default             &pin_pwm6
#define pinctrl_pwm7_default             &pin_pwm7
#define pinctrl_pwm8_default             &pin_pwm8
#define pinctrl_pwm9_default             &pin_pwm9

/* ADC */
#define pinctrl_adc0_default             &pin_adc0
#define pinctrl_adc1_default             &pin_adc1
#define pinctrl_adc10_default            &pin_adc10
#define pinctrl_adc11_default            &pin_adc11
#define pinctrl_adc12_default            &pin_adc12
#define pinctrl_adc13_default            &pin_adc13
#define pinctrl_adc14_default            &pin_adc14
#define pinctrl_adc15_default            &pin_adc15
#define pinctrl_adc2_default             &pin_adc2
#define pinctrl_adc3_default             &pin_adc3
#define pinctrl_adc4_default             &pin_adc4
#define pinctrl_adc5_default             &pin_adc5
#define pinctrl_adc6_default             &pin_adc6
#define pinctrl_adc7_default             &pin_adc7
#define pinctrl_adc8_default             &pin_adc8
#define pinctrl_adc9_default             &pin_adc9

/* TACH */
#define pinctrl_tach0_default            &pin_tach0
#define pinctrl_tach1_default            &pin_tach1
#define pinctrl_tach10_default           &pin_tach10
#define pinctrl_tach11_default           &pin_tach11
#define pinctrl_tach12_default           &pin_tach12
#define pinctrl_tach13_default           &pin_tach13
#define pinctrl_tach14_default           &pin_tach14
#define pinctrl_tach15_default           &pin_tach15
#define pinctrl_tach2_default            &pin_tach2
#define pinctrl_tach3_default            &pin_tach3
#define pinctrl_tach4_default            &pin_tach4
#define pinctrl_tach5_default            &pin_tach5
#define pinctrl_tach6_default            &pin_tach6
#define pinctrl_tach7_default            &pin_tach7
#define pinctrl_tach8_default            &pin_tach8
#define pinctrl_tach9_default            &pin_tach9

/* SALT */
#define pinctrl_salt0_default            &pin_salt0
#define pinctrl_salt1_default            &pin_salt1
#define pinctrl_salt10_default           &pin_salt10
#define pinctrl_salt11_default           &pin_salt11
#define pinctrl_salt12_default           &pin_salt12
#define pinctrl_salt13_default           &pin_salt13
#define pinctrl_salt14_default           &pin_salt14
#define pinctrl_salt15_default           &pin_salt15
#define pinctrl_salt2_default            &pin_salt2
#define pinctrl_salt3_default            &pin_salt3
#define pinctrl_salt4_default            &pin_salt4
#define pinctrl_salt5_default            &pin_salt5
#define pinctrl_salt6_default            &pin_salt6
#define pinctrl_salt7_default            &pin_salt7
#define pinctrl_salt8_default            &pin_salt8
#define pinctrl_salt9_default            &pin_salt9

/* FSI */
#define pinctrl_fsi0_default             &pin_fsi0clk &pin_fsi0dat
#define pinctrl_fsi1_default             &pin_fsi1clk &pin_fsi1dat
#define pinctrl_fsi2_default             &pin_fsi2clk &pin_fsi2dat
#define pinctrl_fsi3_default             &pin_fsi3clk &pin_fsi3dat

/* WDT */
#define pinctrl_wdtrst0n_default         &pin_wdtrst0n
#define pinctrl_wdtrst1n_default         &pin_wdtrst1n
#define pinctrl_wdtrst2n_default         &pin_wdtrst2n
#define pinctrl_wdtrst3n_default         &pin_wdtrst3n
#define pinctrl_wdtrst4n_default         &pin_wdtrst4n
#define pinctrl_wdtrst5n_default         &pin_wdtrst5n
#define pinctrl_wdtrst6n_default         &pin_wdtrst6n
#define pinctrl_wdtrst7n_default         &pin_wdtrst7n

/* SIO */
#define pinctrl_sioonctrln0_default      &pin_sioonctrln0
#define pinctrl_sioonctrln1_default      &pin_sioonctrln1
#define pinctrl_siopbin0_default         &pin_siopbin0
#define pinctrl_siopbin1_default         &pin_siopbin1
#define pinctrl_siopbon0_default         &pin_siopbon0
#define pinctrl_siopbon1_default         &pin_siopbon1
#define pinctrl_siopwreqn0_default       &pin_siopwreqn0
#define pinctrl_siopwreqn1_default       &pin_siopwreqn1
#define pinctrl_siopwrgd1_default        &pin_siopwrgd1
#define pinctrl_sios3n0_default          &pin_sios3n0
#define pinctrl_sios3n1_default          &pin_sios3n1
#define pinctrl_sios5n0_default          &pin_sios5n0
#define pinctrl_sios5n1_default          &pin_sios5n1
#define pinctrl_sioscin0_default         &pin_sioscin0
#define pinctrl_sioscin1_default         &pin_sioscin1

/* HostIF */
#define pinctrl_espi0_default            &pin_espi0_0 &pin_espi0_1 &pin_espi0_2 &pin_espi0_3 &pin_espi0_4 &pin_espi0_5 &pin_espi0_6 &pin_espi0_7
#define pinctrl_espi1_default            &pin_espi1_0 &pin_espi1_1 &pin_espi1_2 &pin_espi1_3 &pin_espi1_4 &pin_espi1_5 &pin_espi1_6 &pin_espi1_7
#define pinctrl_lpc0_default             &pin_lpc0_0 &pin_lpc0_1 &pin_lpc0_2 &pin_lpc0_3 &pin_lpc0_4 &pin_lpc0_5 &pin_lpc0_6 &pin_lpc0_7 &pin_lpc0_8 &pin_lpc0_9
#define pinctrl_lpc1_default             &pin_lpc1_0 &pin_lpc1_1 &pin_lpc1_2 &pin_lpc1_3 &pin_lpc1_4 &pin_lpc1_5 &pin_lpc1_6 &pin_lpc1_7 &pin_lpc1_8 &pin_lpc1_9
#define pinctrl_oscclk_default           &pin_oscclk
#define pinctrl_sd_default               &pin_sd_0 &pin_sd_1 &pin_sd_2 &pin_sd_3 &pin_sd_4 &pin_sd_5 &pin_sd_6 &pin_sd_7
#define pinctrl_vpi_default              &pin_vpi_0 &pin_vpi_1 &pin_vpi_2 &pin_vpi_3 &pin_vpi_4 &pin_vpi_5 &pin_vpi_8 &pin_vpi_9 &pin_vpi_10 &pin_vpi_11 &pin_vpi_12 &pin_vpi_13 &pin_vpi_14 &pin_vpi_15 &pin_vpi_16 &pin_vpi_17 &pin_vpi_18 &pin_vpi_19 &pin_vpi_20 &pin_vpi_21 &pin_vpi_22 &pin_vpi_23 &pin_vpi_24 &pin_vpi_25 &pin_vpi_26 &pin_vpi_27 &pin_vpi_28 &pin_vpi_29

/* Ethernet */
#define pinctrl_mdio0_default            &pin_mdc0 &pin_mdio0
#define pinctrl_mdio1_default            &pin_mdc1 &pin_mdio1
#define pinctrl_mdio2_default            &pin_mdc2 &pin_mdio2
#define pinctrl_rgmii0_default           &pin_rgmii0_0 &pin_rgmii0_1 &pin_rgmii0_2 &pin_rgmii0_3 &pin_rgmii0_4 &pin_rgmii0_5 &pin_rgmii0_6 &pin_rgmii0_7 &pin_rgmii0_8 &pin_rgmii0_9 &pin_rgmii0_10 &pin_rgmii0_11
#define pinctrl_rgmii1_default           &pin_rgmii1_0 &pin_rgmii1_1 &pin_rgmii1_2 &pin_rgmii1_3 &pin_rgmii1_4 &pin_rgmii1_5 &pin_rgmii1_6 &pin_rgmii1_7 &pin_rgmii1_8 &pin_rgmii1_9 &pin_rgmii1_10 &pin_rgmii1_11
#define pinctrl_rmii0_default            &pin_rmii0_0 &pin_rmii0_1 &pin_rmii0_2 &pin_rmii0_3 &pin_rmii0_4 &pin_rmii0_5 &pin_rmii0_6 &pin_rmii0_7
#define pinctrl_rmii0_rclko_default      &pin_rmii0_rclko
#define pinctrl_rmii1_default            &pin_rmii1_0 &pin_rmii1_1 &pin_rmii1_2 &pin_rmii1_3 &pin_rmii1_4 &pin_rmii1_5 &pin_rmii1_6 &pin_rmii1_7
#define pinctrl_rmii1_rclko_default      &pin_rmii1_rclko
#define pinctrl_vga_default              &pin_vga_hs &pin_vga_vs

/* USB */
#define pinctrl_usb2cd_default           &pin_usb2cd
#define pinctrl_usb2ch_default           &pin_usb2ch
#define pinctrl_usb2cu_default           &pin_usb2cu
#define pinctrl_usb2cud_default          &pin_usb2cud
#define pinctrl_usb2dd_default           &pin_usb2dd
#define pinctrl_usb2dh_default           &pin_usb2dh
#define pinctrl_usbuart_default          &pin_usbuart_d_p &pin_usbuart_d_n

/* Net-misc */
#define pinctrl_maclink0_default         &pin_maclink0
#define pinctrl_maclink1_default         &pin_maclink1
#define pinctrl_maclink2_default         &pin_maclink2
#define pinctrl_pe2sgrstn_default        &pin_pe2sgrstn_0 &pin_pe2sgrstn_1
#define pinctrl_sgmii_default            &pin_sgmii

/* SGPM-SMON */
#define pinctrl_dsgpm0_default           &pin_dsgpm0_0 &pin_dsgpm0_1 &pin_dsgpm0_2 &pin_dsgpm0_3
#define pinctrl_sgpm0_default            &pin_sgpm0_0 &pin_sgpm0_1 &pin_sgpm0_2 &pin_sgpm0_3
#define pinctrl_sgpm1_default            &pin_sgpm1_0 &pin_sgpm1_1 &pin_sgpm1_2 &pin_sgpm1_3
#define pinctrl_sgps_default             &pin_sgps_0 &pin_sgps_1 &pin_sgps_2 &pin_sgps_3
#define pinctrl_smon0_default            &pin_smon0_0 &pin_smon0_1 &pin_smon0_2 &pin_smon0_3
#define pinctrl_smon1_default            &pin_smon1_0 &pin_smon1_1 &pin_smon1_2 &pin_smon1_3

/* misc */
#define pinctrl_auxpwrgood0_default      &pin_auxpwrgood0
#define pinctrl_auxpwrgood1_default      &pin_auxpwrgood1
#define pinctrl_canbus_default           &pin_canbus_tx &pin_canbus_rx
#define pinctrl_canbus_stby_default      &pin_canbus_stby
#define pinctrl_hbled_default            &pin_hbled
#define pinctrl_i2cf0_default            &pin_i2cf0_0 &pin_i2cf0_1 &pin_i2cf0_2 &pin_i2cf0_3
#define pinctrl_i2cf1_default            &pin_i2cf1_0 &pin_i2cf1_1 &pin_i2cf1_2 &pin_i2cf1_3
#define pinctrl_i2cf2_default            &pin_i2cf2_0 &pin_i2cf2_1 &pin_i2cf2_2 &pin_i2cf2_3
#define pinctrl_jtagm1_default           &pin_jtagm1_0 &pin_jtagm1_1 &pin_jtagm1_2 &pin_jtagm1_3 &pin_jtagm1_4
#define pinctrl_thru0_default            &pin_thru0_0 &pin_thru0_1
#define pinctrl_thru1_default            &pin_thru1_0 &pin_thru1_1
#define pinctrl_thru2_default            &pin_thru2_0 &pin_thru2_1
#define pinctrl_thru3_default            &pin_thru3_0 &pin_thru3_1

#endif /* end of "#ifndef _AST27XX_SOC1_PINCTRL_GROUPS_H_" */
