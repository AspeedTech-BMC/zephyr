/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2021 ASPEED Technology Inc.
 */
#ifndef _AST10X0_G2_IRQ_H_
#define _AST10X0_G2_IRQ_H_

#define AST10X0_G2_IRQ_DEFAULT_PRIORITY	1

#define INTR_LPC			0
#define INTR_SNOOP			1
#define INTR_BT				2
#define INTR_POSTCODE			3
#define INTR_KCS_1			4
#define INTR_KCS_2			5
#define INTR_KCS_3			6
#define INTR_KCS_4			7
#define INTR_KCS_5			8
#define INTR_SIO			9
#define INTR_ESPI_GLOBAL		10
#define INTR_ESPI_PERIPHERAL		11
#define INTR_ESPI_VW			12
#define INTR_ESPI_OOB			13
#define INTR_ESPI_FLASH			14
#define INTR_ESPI_SAFS			15
#define INTR_MMBI			16
#define INTR_VUART1			17
#define INTR_VUART2			18
#define INTR_LPC_SWC			19
#define INTR_LPC_MAILBOX		20
#define INTR_ERTC			21

#define INTR_I2C0			64
#define INTR_I2C1			65
#define INTR_I2C2			66
#define INTR_I2C3			67
#define INTR_I2C4			68
#define INTR_I2C5			69
#define INTR_I2C6			70
#define INTR_I2C7			71
#define INTR_I2C8			72
#define INTR_I2C9			73
#define INTR_I2C10			74
#define INTR_I2C11			75
#define INTR_I2C12			76
#define INTR_I2C13			77

#define INTR_ADC			80
#define INTR_ADC_SSP			81
#define INTR_GPIO			82
#define INTR_GPIO_SSP			83
#define INTR_GPIO_COP2			84
#define INTR_SGPIO0			85
#define INTR_SGPIO0_SSP			86
#define INTR_SGPIO0_COP2		87
#define INTR_SGPIO1			88
#define INTR_SGPIO1_SSP			89
#define INTR_SGPIO1_COP2		90
#define INTR_RTC			91
#define INTR_TIMER0			92
#define INTR_TIMER1			93
#define INTR_TIMER2			94
#define INTR_TIMER3			95
#define INTR_I3C0			96
#define INTR_I3C1			97
#define INTR_I3C2			98
#define INTR_I3C3			99
#define INTR_I3C4			100
#define INTR_I3C5			101
#define INTR_I3C6			102
#define INTR_I3C7			103

#define INTR_WDT0			112
#define INTR_WDT1			113
#define INTR_WDT2			114
#define INTR_WDT3			115
#define INTR_WDT4			116
#define INTR_WDT5			117
#define INTR_WDT6			118
#define INTR_WDT7			119
#define INTR_WDT_ABR			120
#define INTR_FMC			121
#define INTR_SPI0			122
#define INTR_SPI1			123
#define INTR_HYPERRAM			124
#define INTR_TACH			125
#define INTR_AHBC			126

#define INTR_MAC			128

#define INTR_TIMER4			131
#define INTR_TIMER5			132
#define INTR_TIMER6			133
#define INTR_TIMER7			134
#define INTR_UART0			135
#define INTR_UART1			136
#define INTR_UART2			137
#define INTR_UART3			138
#define INTR_UART4			139
#define INTR_UART5			140
#define INTR_UART6			141
#define INTR_UART7			142
#define INTR_UART8			143
#define INTR_UART9			144
#define INTR_UART10			145
#define INTR_UART11			146
#define INTR_UART12			147
#define INTR_UARTDMA			148

#define INTR_ACE			150
#define INTR_FMC_FILTER			151
#define INTR_SPI0_FILTER		152
#define INTR_SPI1_FILTER		153
#define INTR_USB11H			155
#define INTR_USB2C			156
#define INTR_USB2D			157
#define INTR_SCU			159
#define INTR_JTAG			162
#define INTR_PECI			164
#define INTR_OTP			168
#define INTR_SRAM			169
#define INTR_AACAM			170
#define INTR_AAGLITCH			171
#define INTR_AAIR			172
#define INTR_AATSENSE			173
#define INTR_IPC0			174
#define INTR_IPC1			175
#define INTR_LPCHOST			182
#define INTR_SPI_F0			183
#define INTR_SPI_F1			184
#define INTR_SPI_F2			185
#define INTR_SPI_F3			186
#define INTR_I2C_F0			187
#define INTR_I2C_F1			188
#define INTR_I2C_F2			189
#define INTR_I2C_F3			190

#endif /* #ifndef _AST10X0_G2_IRQ_H_ */
