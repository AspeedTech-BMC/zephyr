/*
 * Copyright (c) 2023 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AST27XX_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AST27XX_H_

/* cpu die clk-gate */
#define AST2700_CPU_CLK_GATE_MCLK	(0)
#define AST2700_CPU_CLK_GATE_ECLK	(1)
#define AST2700_CPU_CLK_GATE_GCLK	(2)
#define AST2700_CPU_CLK_GATE_VCLK	(3)
#define AST2700_CPU_CLK_GATE_BCLK	(4)
#define AST2700_CPU_CLK_GATE_D1CLK	(5)
#define AST2700_CPU_CLK_GATE_REFCLK	(6)
#define AST2700_CPU_CLK_GATE_USB0CLK	(7)
#define AST2700_CPU_CLK_GATE_RSV8	(8)
#define AST2700_CPU_CLK_GATE_USB1CLK	(9)
#define AST2700_CPU_CLK_GATE_D2CLK	(10)
#define AST2700_CPU_CLK_GATE_RSV11	(11)
#define AST2700_CPU_CLK_GATE_RSV12	(12)
#define AST2700_CPU_CLK_GATE_YCLK	(13)
#define AST2700_CPU_CLK_GATE_USB2CLK	(14)
#define AST2700_CPU_CLK_GATE_UART4CLK	(15)
#define AST2700_CPU_CLK_GATE_SLICLK	(16)
#define AST2700_CPU_CLK_GATE_DACCLK	(17)
#define AST2700_CPU_CLK_GATE_RSV18	(18)
#define AST2700_CPU_CLK_GATE_RSV19	(19)
#define AST2700_CPU_CLK_GATE_CRT1CLK	(20)
#define AST2700_CPU_CLK_GATE_CRT2CLK	(21)
#define AST2700_CPU_CLK_GATE_VLCLK	(22)
#define AST2700_CPU_CLK_GATE_ECCCLK	(23)
#define AST2700_CPU_CLK_GATE_RSACLK	(24)
#define AST2700_CPU_CLK_GATE_RVAS0CLK	(25)
#define AST2700_CPU_CLK_GATE_UFSCLK	(26)
#define AST2700_CPU_CLK_GATE_EMMCCLK	(27)
#define AST2700_CPU_CLK_GATE_RVAS1CLK	(28)

/* reserved 29 ~ 31*/

#define AST2700_CPU_CLK_GATE_NUM			29

/* cpu die clk */
#define AST2700_CPU_CLKIN		(AST2700_CPU_CLK_GATE_NUM + 0)
#define AST2700_CPU_CLK_24M		(AST2700_CPU_CLK_GATE_NUM + 1)
#define AST2700_CPU_CLK_192M	(AST2700_CPU_CLK_GATE_NUM + 2)
#define AST2700_CPU_CLK_UART	(AST2700_CPU_CLK_GATE_NUM + 3)
#define AST2700_CPU_CLK_HPLL		(AST2700_CPU_CLK_GATE_NUM + 4)
#define AST2700_CPU_CLK_DPLL		(AST2700_CPU_CLK_GATE_NUM + 5)
#define AST2700_CPU_CLK_MPLL		(AST2700_CPU_CLK_GATE_NUM + 6)
#define AST2700_CPU_CLK_D1CLK           (AST2700_CPU_CLK_GATE_NUM + 7)
#define AST2700_CPU_CLK_D2CLK           (AST2700_CPU_CLK_GATE_NUM + 8)
#define AST2700_CPU_CLK_CRT1		(AST2700_CPU_CLK_GATE_NUM + 9)
#define AST2700_CPU_CLK_CRT2		(AST2700_CPU_CLK_GATE_NUM + 10)
#define AST2700_CPU_CLK_MPHY            (AST2700_CPU_CLK_GATE_NUM + 11)
#define AST2700_CPU_CLK_AXI		(AST2700_CPU_CLK_GATE_NUM + 12)
#define AST2700_CPU_CLK_AHB             (AST2700_CPU_CLK_GATE_NUM + 13)
#define AST2700_CPU_CLK_APB             (AST2700_CPU_CLK_GATE_NUM + 14)
#define AST2700_CPU_CLK_MCLK            (AST2700_CPU_CLK_GATE_NUM + 15)
#define AST2700_CPU_CLK_ECLK            (AST2700_CPU_CLK_GATE_NUM + 16)
#define AST2700_CPU_CLK_GCLK            (AST2700_CPU_CLK_GATE_NUM + 17)
#define AST2700_CPU_CLK_VCLK            (AST2700_CPU_CLK_GATE_NUM + 18)
#define AST2700_CPU_CLK_BCLK            (AST2700_CPU_CLK_GATE_NUM + 19)
#define AST2700_CPU_CLK_REF		(AST2700_CPU_CLK_GATE_NUM + 20)
#define AST2700_CPU_CLK_USB0CLK		(AST2700_CPU_CLK_GATE_NUM + 21)
#define AST2700_CPU_CLK_USB1CLK		(AST2700_CPU_CLK_GATE_NUM + 22)
#define AST2700_CPU_CLK_USB2CLK		(AST2700_CPU_CLK_GATE_NUM + 23)
#define AST2700_CPU_CLK_YCLK		(AST2700_CPU_CLK_GATE_NUM + 24)
#define AST2700_CPU_CLK_UART4		(AST2700_CPU_CLK_GATE_NUM + 25)
#define AST2700_CPU_CLK_SLI		(AST2700_CPU_CLK_GATE_NUM + 26)
#define AST2700_CPU_CLK_ECC		(AST2700_CPU_CLK_GATE_NUM + 27)
#define AST2700_CPU_CLK_RSA		(AST2700_CPU_CLK_GATE_NUM + 28)
#define AST2700_CPU_CLK_RVAS0		(AST2700_CPU_CLK_GATE_NUM + 29)
#define AST2700_CPU_CLK_UFS		(AST2700_CPU_CLK_GATE_NUM + 30)
#define AST2700_CPU_CLK_RVAS1		(AST2700_CPU_CLK_GATE_NUM + 31)

#define AST2700_CPU_NUM_CLKS		(AST2700_CPU_CLK_RVAS1 + 1)

/* io die clk gate */
#define AST2700_IO_CLK_GATE_LCLK0       (0)
#define AST2700_IO_CLK_GATE_LCLK1       (1)
#define AST2700_IO_CLK_GATE_ESPI0CLK    (2)
#define AST2700_IO_CLK_GATE_ESPI1CLK    (3)
#define AST2700_IO_CLK_GATE_SDCLK       (4)
#define AST2700_IO_CLK_GATE_REFCLK      (5)
#define AST2700_IO_CLK_GATE_RSV5CLK     (6)
#define AST2700_IO_CLK_GATE_LPCHCLK     (7)
#define AST2700_IO_CLK_GATE_MAC0CLK     (8)
#define AST2700_IO_CLK_GATE_MAC1CLK     (9)
#define AST2700_IO_CLK_GATE_MAC2CLK     (10)
#define AST2700_IO_CLK_GATE_UART0CLK    (11)
#define AST2700_IO_CLK_GATE_UART1CLK    (12)
#define AST2700_IO_CLK_GATE_UART2CLK    (13)
#define AST2700_IO_CLK_GATE_UART3CLK    (14)
/* reserved bit 15*/
#define AST2700_IO_CLK_GATE_I3C0CLK     (16)
#define AST2700_IO_CLK_GATE_I3C1CLK     (17)
#define AST2700_IO_CLK_GATE_I3C2CLK     (18)
#define AST2700_IO_CLK_GATE_I3C3CLK     (19)
#define AST2700_IO_CLK_GATE_I3C4CLK     (20)
#define AST2700_IO_CLK_GATE_I3C5CLK     (21)
#define AST2700_IO_CLK_GATE_I3C6CLK     (22)
#define AST2700_IO_CLK_GATE_I3C7CLK     (23)
#define AST2700_IO_CLK_GATE_I3C8CLK     (24)
#define AST2700_IO_CLK_GATE_I3C9CLK     (25)
#define AST2700_IO_CLK_GATE_I3C10CLK    (26)
#define AST2700_IO_CLK_GATE_I3C11CLK    (27)
#define AST2700_IO_CLK_GATE_I3C12CLK    (28)
#define AST2700_IO_CLK_GATE_I3C13CLK    (29)
#define AST2700_IO_CLK_GATE_I3C14CLK    (30)
#define AST2700_IO_CLK_GATE_I3C15CLK    (31)

#define AST2700_IO_CLK_GATE_UART5CLK    (32 + 0)
#define AST2700_IO_CLK_GATE_UART6CLK    (32 + 1)
#define AST2700_IO_CLK_GATE_UART7CLK    (32 + 2)
#define AST2700_IO_CLK_GATE_UART8CLK    (32 + 3)
#define AST2700_IO_CLK_GATE_UART9CLK	(32 + 4)
#define AST2700_IO_CLK_GATE_UART10CLK   (32 + 5)
#define AST2700_IO_CLK_GATE_UART11CLK   (32 + 6)
#define AST2700_IO_CLK_GATE_UART12CLK   (32 + 7)
#define AST2700_IO_CLK_GATE_FSICLK      (32 + 8)
#define AST2700_IO_CLK_GATE_LTPIPHYCLK	(32 + 9)
#define AST2700_IO_CLK_GATE_LTPICLK     (32 + 10)
#define AST2700_IO_CLK_GATE_VGALCLK     (32 + 11)
#define AST2700_IO_CLK_GATE_USBUARTCLK  (32 + 12)
#define AST2700_IO_CLK_GATE_CANCLK      (32 + 13)
#define AST2700_IO_CLK_GATE_PCICLK	(32 + 14)
#define AST2700_IO_CLK_GATE_SLICLK      (32 + 15)

#define AST2700_IO_CLK_GATE_NUM			(AST2700_IO_CLK_GATE_SLICLK + 1)

/* io die clk */
#define AST2700_IO_CLKIN		(AST2700_IO_CLK_GATE_NUM + 0)
#define AST2700_IO_CLK_HPLL		(AST2700_IO_CLK_GATE_NUM + 1)
#define AST2700_IO_CLK_APLL		(AST2700_IO_CLK_GATE_NUM + 2)
#define AST2700_IO_CLK_APLL_DIV2	(AST2700_IO_CLK_GATE_NUM + 3)
#define AST2700_IO_CLK_APLL_DIV4	(AST2700_IO_CLK_GATE_NUM + 4)
#define AST2700_IO_CLK_DPLL		(AST2700_IO_CLK_GATE_NUM + 5)
#define AST2700_IO_CLK_UXCLK		(AST2700_IO_CLK_GATE_NUM + 6)
#define AST2700_IO_CLK_HUXCLK		(AST2700_IO_CLK_GATE_NUM + 7)
#define AST2700_IO_CLK_UARTCLK		(AST2700_IO_CLK_GATE_NUM + 8)
#define AST2700_IO_CLK_HUARTCLK		(AST2700_IO_CLK_GATE_NUM + 9)
#define AST2700_IO_CLK_AHB		(AST2700_IO_CLK_GATE_NUM + 10)
#define AST2700_IO_CLK_APB		(AST2700_IO_CLK_GATE_NUM + 11)
#define AST2700_IO_CLK_UART0		(AST2700_IO_CLK_GATE_NUM + 12)
#define AST2700_IO_CLK_UART1		(AST2700_IO_CLK_GATE_NUM + 13)
#define AST2700_IO_CLK_UART2		(AST2700_IO_CLK_GATE_NUM + 14)
#define AST2700_IO_CLK_UART3		(AST2700_IO_CLK_GATE_NUM + 15)
#define AST2700_IO_CLK_UART5		(AST2700_IO_CLK_GATE_NUM + 16)
#define AST2700_IO_CLK_UART6		(AST2700_IO_CLK_GATE_NUM + 17)
#define AST2700_IO_CLK_UART7		(AST2700_IO_CLK_GATE_NUM + 18)
#define AST2700_IO_CLK_UART8		(AST2700_IO_CLK_GATE_NUM + 19)
#define AST2700_IO_CLK_UART9		(AST2700_IO_CLK_GATE_NUM + 20)
#define AST2700_IO_CLK_UART10		(AST2700_IO_CLK_GATE_NUM + 21)
#define AST2700_IO_CLK_UART11		(AST2700_IO_CLK_GATE_NUM + 22)
#define AST2700_IO_CLK_UART12		(AST2700_IO_CLK_GATE_NUM + 23)
#define AST2700_IO_CLK_HPLL_DIVN	(AST2700_IO_CLK_GATE_NUM + 24)
#define AST2700_IO_CLK_APLL_DIVN	(AST2700_IO_CLK_GATE_NUM + 25)
#define AST2700_IO_CLK_SDCLK		(AST2700_IO_CLK_GATE_NUM + 26)
#define AST2700_IO_CLK_RMII		(AST2700_IO_CLK_GATE_NUM + 27)
#define AST2700_IO_CLK_RGMII		(AST2700_IO_CLK_GATE_NUM + 28)
#define AST2700_IO_CLK_MACHCLK		(AST2700_IO_CLK_GATE_NUM + 29)

#define AST2700_IO_NUM_CLKS		(AST2700_IO_CLK_MACHCLK + 1)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AST27XX_H_ */