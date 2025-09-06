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
#define MPU_ID_SDIO		(34)
#define MPU_ID_SLIM		(35)

/* port 5 */
#define MPU_ID_USBH_A		(36)
#define MPU_ID_USBH_B		(37)
#define MPU_ID_UFS		(38)

#define S_READWRITE		0
#define S_READONLY		1
#define S_WRITEONLY		2
#define NS_READWRITE		3
#define NS_READONLY		4
#define NS_WRITEONLY		5

#endif
