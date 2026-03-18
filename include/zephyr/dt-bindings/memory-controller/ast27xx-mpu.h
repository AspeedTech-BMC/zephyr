#ifndef _AST2700_MPU_H_
#define _AST2700_MPU_H_

#define MAX_MPU_COUNT			16
#define MPU_PROP_NAME			"mpu-"

/* port 0 */
#define MPU_ID_CA35		(0)

/* port 1 */
#define MPU_ID_VE_HI		(1)
#define MPU_ID_VE_LO		(2)
#define MPU_ID_USB_A1		(3)
#define MPU_ID_USB_A2		(4)
#define MPU_ID_E2M		(5)
#define MPU_ID_MCTP		(6)
#define MPU_ID_H2M		(7)
#define MPU_ID_HMAC		(8)

/* port 2 */
#define MPU_ID_USB_B1		(9)
#define MPU_ID_USB_B2		(10)
#define MPU_ID_VGA1_CR		(11)
#define MPU_ID_VGA1_LE		(12)
#define MPU_ID_TSP_INST		(13)
#define MPU_ID_VE		(14)
#define MPU_ID_MCTP8		(15)
#define MPU_ID_UHCI		(16)

/* port 3 */
#define MPU_ID_USB3_A1		(17)
#define MPU_ID_USB3_A2		(18)
#define MPU_ID_SHA3		(19)
#define MPU_ID_VGA2_CR		(20)
#define MPU_ID_VGA2_LE		(21)
#define MPU_ID_TSP_DATA		(22)
#define MPU_ID_E2M1		(23)
#define MPU_ID_GFX		(24)
#define MPU_ID_RVAS1		(25)
#define MPU_ID_RVAS2		(26)
#define MPU_ID_MHMAC		(27)
#define MPU_ID_M2D		(28)
#define MPU_ID_M2D2		(29)
#define MPU_ID_SSP_INST		(30)
#define MPU_ID_SSP_DATA		(31)
#define MPU_ID_XDMA8		(32)

/* port 4 */
#define MPU_ID_XDMA		(33)
#define MPU_ID_EMMC		(34)
#define MPU_ID_SLIM		(35)

/* port 5 */
#define MPU_ID_USBH_A		(36)
#define MPU_ID_USBH_B		(37)
#define MPU_ID_UFS		(38)

/* port h2m0 */
#define MPU_ID_AHBC_MONITOR	(0x1006)
#define MPU_ID_SBDMA		(0x1007)
#define MPU_ID_UARTDBG0		(0x1008)
#define MPU_ID_PORT80		(0x1009)
#define MPU_ID_DP_MCU		(0x100A)
#define MPU_ID_EMMC_BOOT	(0x100C)
#define MPU_ID_UDMAC		(0x100E)

/* port h2m1 */
#define MPU_ID_BOOTMCU		(0x2001)
#define MPU_ID_FMC		(0x2004)
#define MPU_ID_AHBC1		(0x2006)
#define MPU_ID_SPI0		(0x2007)
#define MPU_ID_SPI1		(0x2008)
#define MPU_ID_SPI2		(0x2009)
#define MPU_ID_ESPI0		(0x200A)
#define MPU_ID_ESPI1		(0x200B)
#define MPU_ID_80H0		(0x200E)
#define MPU_ID_80H1		(0x200F)
#define MPU_ID_UARTDBG1		(0x2010)
#define MPU_ID_H2A_SPI1		(0x2013)
#define MPU_ID_H2A_SPI2		(0x2014)
#define MPU_ID_UARTDMA		(0x2015)
#define MPU_ID_USB2UARTA	(0x201A)
#define MPU_ID_USB2UARTB	(0x201B)

/* port sli */
#define MPU_ID_I2C		(0x4000)
#define MPU_ID_I2C_FILTER	(0x4001)
#define MPU_ID_SDIO		(0x4002)
#define MPU_ID_SPI_FILTER	(0x4003)
#define MPU_ID_ACE		(0x4004)
#define MPU_ID_UHCI_IO		(0x4005)
#define MPU_ID_E2M_IO		(0x4006)
#define MPU_ID_H2M1		(0x4007)
#define MPU_ID_MAC0		(0x4008)
#define MPU_ID_MAC1		(0x4009)
#define MPU_ID_MAC2		(0x400A)
#define MPU_ID_FSI		(0x400B)
#define MPU_ID_MCTP2		(0x400C)
#define MPU_ID_LTPI0		(0x400D)
#define MPU_ID_USB2_PORT_A	(0x400E)
#define MPU_ID_USB2_PORT_B	(0x400F)
#define MPU_ID_LTPI1		(0x4010)

#define S_READWRITE		0
#define S_READONLY		1
#define S_WRITEONLY		2
#define NS_READWRITE		3
#define NS_READONLY		4
#define NS_WRITEONLY		5

#endif
