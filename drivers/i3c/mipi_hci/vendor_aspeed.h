/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I3C_MIPI_HCI_VENDOR_ASPEED_H_
#define ZEPHYR_DRIVERS_I3C_MIPI_HCI_VENDOR_ASPEED_H_

#include <errno.h>

#include "hci.h"

static inline uint32_t ast_inhouse_read(const struct i3c_hci *hci, uint32_t reg)
{
	return sys_read32((mem_addr_t)(hci->VENDOR_regs + reg));
}

static inline void ast_inhouse_write(const struct i3c_hci *hci, uint32_t reg, uint32_t val)
{
	sys_write32(val, (mem_addr_t)(hci->VENDOR_regs + reg));
}

#define ASPEED_HC_PAYLOAD_LIMIT 128U

#define ASPEED_I3C_CTRL 0x0
#define ASPEED_I3C_CTRL_STOP_QUEUE_PT BIT(31)
#define ASPEED_I3C_CTRL_INIT BIT(4)
#define ASPEED_I3C_CTRL_INIT_MODE GENMASK(1, 0)
#define INIT_MST_MODE 0
#define INIT_SEC_MST_MODE 1
#define INIT_SLV_MODE 2

#define ASPEED_I3C_STS 0x4
#define ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_VALID BIT(23)
#define ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS GENMASK(22, 16)
#define ASPEED_I3C_STS_MODE_PURE_SLV BIT(8)
#define ASPEED_I3C_STS_MODE_SECONDARY_SLV_TO_MST BIT(7)
#define ASPEED_I3C_STS_MODE_SECONDARY_MST_TO_SLV BIT(6)
#define ASPEED_I3C_STS_MODE_SECONDARY_SLV BIT(5)
#define ASPEED_I3C_STS_MODE_SECONDARY_MST BIT(4)
#define ASPEED_I3C_STS_MODE_PRIMARY_SLV_TO_MST BIT(3)
#define ASPEED_I3C_STS_MODE_PRIMARY_MST_TO_SLV BIT(2)
#define ASPEED_I3C_STS_MODE_PRIMARY_SLV BIT(1)
#define ASPEED_I3C_STS_MODE_PRIMARY_MST BIT(0)

#define ASPEED_I3C_MST_MRL 0x8
#define ASPEED_I3C_IBI_TERMINATE_EN BIT(16)
#define ASPEED_I3C_IBI_TERMINATE_LEN GENMASK(15, 0)

#define ASPEED_I3C_DAA_INDEX0 0x10
#define ASPEED_I3C_DAA_INDEX1 0x14
#define ASPEED_I3C_DAA_INDEX2 0x18
#define ASPEED_I3C_DAA_INDEX3 0x1c

#define ASPEED_I3C_AUTOCMD_0 0x20
#define ASPEED_I3C_AUTOCMD_1 0x24
#define ASPEED_I3C_AUTOCMD_2 0x28
#define ASPEED_I3C_AUTOCMD_3 0x2c
#define ASPEED_I3C_AUTOCMD_4 0x30
#define ASPEED_I3C_AUTOCMD_5 0x34
#define ASPEED_I3C_AUTOCMD_6 0x38
#define ASPEED_I3C_AUTOCMD_7 0x3c

#define ASPEED_I3C_AUTOCMD_SEL_0_7 0x40
#define ASPEED_I3C_AUTOCMD_SEL_8_15 0x44
#define ASPEED_I3C_AUTOCMD_SEL_16_23 0x48
#define ASPEED_I3C_AUTOCMD_SEL_24_31 0x4c
#define ASPEED_I3C_AUTOCMD_SEL_32_39 0x50
#define ASPEED_I3C_AUTOCMD_SEL_40_47 0x54
#define ASPEED_I3C_AUTOCMD_SEL_48_55 0x58
#define ASPEED_I3C_AUTOCMD_SEL_56_63 0x5c
#define ASPEED_I3C_AUTOCMD_SEL_64_71 0x60
#define ASPEED_I3C_AUTOCMD_SEL_72_79 0x64
#define ASPEED_I3C_AUTOCMD_SEL_80_87 0x68
#define ASPEED_I3C_AUTOCMD_SEL_88_95 0x6c
#define ASPEED_I3C_AUTOCMD_SEL_96_103 0x70
#define ASPEED_I3C_AUTOCMD_SEL_104_111 0x74
#define ASPEED_I3C_AUTOCMD_SEL_112_119 0x78
#define ASPEED_I3C_AUTOCMD_SEL_120_127 0x7c

#define ASPEED_I3C_WDMA_CTRL 0x80
#define ASPEED_I3C_WDMA_DBG_LO 0x84
#define ASPEED_I3C_WDMA_DBG_HI 0x88
#define ASPEED_I3C_RDMA_CTRL 0x90
#define ASPEED_I3C_RDMA_DBG_LO 0x94
#define ASPEED_I3C_RDMA_DBG_HI 0x98
#define I3C_DMA_DBG_LO_ABORT BIT(0)
#define I3C_DMA_DBG_LO_BUSY BIT(1)
#define I3C_DMA_DBG_LO_DONE BIT(2)
#define I3C_DMA_DBG_LO_START BIT(3)
#define I3C_DMA_DBG_LO_READY BIT(4)
#define I3C_DMA_DBG_LO_VALID BIT(5)
#define I3C_DMA_DBG_LO_EN BIT(10)

#define ASPEED_I3C_RING_STATUS 0x9c
#define I3C_RING_IDLE 0
#define I3C_RING_GET_TRANSFER 1
#define I3C_RING_GET_TX_DATA 2
#define I3C_RING_CMD_PROCESS 3
#define I3C_RING_WRITE_RX_DATA 4
#define I3C_RING_WRITE_RESPONSE 5
#define I3C_RING_UPDATE_CR_PTR 6
#define I3C_RING_WRITE_IBI_STS 7
#define I3C_RING_WRITE_IBI_DAT 8
#define I3C_RING_UPDATE_IBI_PTR 9
#define I3C_RING_ABORT 10
#define I3C_RING_SLV_WRITE_DATA 11
#define I3C_RING_SLV_WRITE_RESPONSE 12
#define I3C_RING_SLV_UPDATE_PT 13

#define ASPEED_I3C_SLV_CHAR_CTRL 0xa0
#define ASPEED_I3C_SLV_CHAR_CTRL_DCR GENMASK(23, 16)
#define ASPEED_I3C_SLV_CHAR_CTRL_BCR GENMASK(15, 8)
#define SLV_BCR_DEVICE_ROLE GENMASK(7, 6)
#define ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR_EN BIT(7)
#define ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR GENMASK(6, 0)
#define SLV_PID_HI(x) (((x) >> 32) & GENMASK(15, 0))
#define SLV_PID_LO(x) ((x) & GENMASK(31, 0))
#define ASPEED_I3C_SLV_PID_LO 0xa4
#define ASPEED_I3C_SLV_PID_HI 0xa8
#define ASPEED_I3C_SLV_FSM 0xac
#define ASPEED_I3C_SLV_CAP_CTRL 0xb0
#define ASPEED_I3C_SLV_CAP_CTRL_PEC_EN BIT(31)
#define ASPEED_I3C_SLV_CAP_CTRL_HAIT_IF_IBI_ERR BIT(30)
#define ASPEED_I3C_SLV_CAP_CTRL_ACCEPT_CR BIT(16)
#define ASPEED_I3C_SLV_CAP_CTRL_HJ_REQ BIT(10)
#define ASPEED_I3C_SLV_CAP_CTRL_MR_REQ BIT(9)
#define ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ BIT(8)
#define ASPEED_I3C_SLV_CAP_CTRL_HJ_WAIT BIT(6)
#define ASPEED_I3C_SLV_CAP_CTRL_MR_WAIT BIT(5)
#define ASPEED_I3C_SLV_CAP_CTRL_IBI_WAIT BIT(4)
#define ASPEED_I3C_SLV_CAP_CTRL_NOTSUP_DEF_BYTE BIT(1)
#define ASPEED_I3C_SLV_CAP_CTRL_I2C_DEV BIT(0)

#define ASPEED_I3C_SLV_STS1 0xb4
#define ASPEED_I3C_SLV_STS1_IBI_PAYLOAD_SIZE GENMASK(31, 24)
#define ASPEED_I3C_SLV_STS1_RSTACT GENMASK(22, 16)
#define ASPEED_I3C_SLV_STS1_ETP_ACK_CAP BIT(15)
#define ASPEED_I3C_SLV_STS1_ETP_W_REQ BIT(14)
#define ASPEED_I3C_SLV_STS1_ETP_CRC GENMASK(13, 12)
#define ASPEED_I3C_SLV_STS1_ENDXFER_CONFIRM BIT(11)
#define ASPEED_I3C_SLV_STS1_ENTER_TEST_MDOE BIT(8)
#define ASPEED_I3C_SLV_STS1_HJ_EN BIT(6)
#define ASPEED_I3C_SLV_STS1_CR_EN BIT(5)
#define ASPEED_I3C_SLV_STS1_IBI_EN BIT(4)
#define ASPEED_I3C_SLV_STS1_HJ_DONE BIT(2)
#define ASPEED_I3C_SLV_STS1_CR_DONE BIT(1)
#define ASPEED_I3C_SLV_STS1_IBI_DONE BIT(0)
#define ASPEED_I3C_SLV_STS2 0xb8
#define ASPEED_I3C_SLV_STS2_MWL GENMASK(31, 16)
#define ASPEED_I3C_SLV_STS2_MRL GENMASK(15, 0)
#define ASPEED_I3C_SLV_STS3_GROUP_ADDR 0xbc
#define ASPEED_I3C_SLV_STS3_GROUP3_VALID BIT(31)
#define ASPEED_I3C_SLV_STS3_GROUP3_ADDR GENMASK(30, 24)
#define ASPEED_I3C_SLV_STS3_GROUP2_VALID BIT(23)
#define ASPEED_I3C_SLV_STS3_GROUP2_ADDR GENMASK(22, 16)
#define ASPEED_I3C_SLV_STS3_GROUP1_VALID BIT(15)
#define ASPEED_I3C_SLV_STS3_GROUP1_ADDR GENMASK(14, 8)
#define ASPEED_I3C_SLV_STS3_GROUP0_VALID BIT(7)
#define ASPEED_I3C_SLV_STS3_GROUP0_ADDR GENMASK(6, 0)
#define ASPEED_I3C_SLV_STS4_RSTACT_TIME 0xc0
#define ASPEED_I3C_SLV_STS4_DBG_NET GENMASK(23, 16)
#define ASPEED_I3C_SLV_STS4_WHOLE_CHIP GENMASK(15, 8)
#define ASPEED_I3C_SLV_STS4_I3C GENMASK(7, 0)
#define ASPEED_I3C_SLV_STS5_GETMXDS_RW 0xc4
#define ASPEED_I3C_SLV_STS5_MAXWR GENMASK(15, 8)
#define ASPEED_I3C_SLV_STS5_MAXRD GENMASK(7, 0)
#define ASPEED_I3C_SLV_STS6_GETMXDS 0xc8
#define ASPEED_I3C_SLV_STS6_FORMAT BIT(24)
#define ASPEED_I3C_SLV_STS6_MAXRD_TURN_H GENMASK(23, 16)
#define ASPEED_I3C_SLV_STS6_MAXRD_TURN_M GENMASK(15, 8)
#define ASPEED_I3C_SLV_STS6_MAXRD_TURN_L GENMASK(7, 0)
#define ASPEED_I3C_SLV_STS7_GETSTATUS 0xcc
#define ASPEED_I3C_SLV_STS7_PRECR GENMASK(31, 16)
#define ASPEED_I3C_SLV_STS7_TGT GENMASK(15, 0)
#define ASPEED_I3C_SLV_STS8_GETCAPS_TGT 0xd0
#define ASPEED_I3C_SLV_STS9_GETCAPS_VT_CR 0xd4
#define ASPEED_I3C_SLV_STS7_VT GENMASK(31, 16)
#define ASPEED_I3C_SLV_STS7_CR GENMASK(15, 0)

#define ASPEED_I3C_QUEUE_PTR0 0xd8
#define QUEUE_PTR0_TX_R(q) FIELD_GET(GENMASK(24, 20), q)
#define QUEUE_PTR0_TX_W(q) FIELD_GET(GENMASK(16, 12), q)
#define QUEUE_PTR0_IBI_R(q) FIELD_GET(GENMASK(11, 10), q)
#define QUEUE_PTR0_IBI_W(q) FIELD_GET(GENMASK(9, 8), q)
#define QUEUE_PTR0_RESP_R(q) FIELD_GET(GENMASK(7, 6), q)
#define QUEUE_PTR0_RESP_W(q) FIELD_GET(GENMASK(5, 4), q)
#define QUEUE_PTR0_CMD_R(q) FIELD_GET(GENMASK(3, 2), q)
#define QUEUE_PTR0_CMD_W(q) FIELD_GET(GENMASK(1, 0), q)

#define ASPEED_I3C_QUEUE_PTR1 0xdc
#define QUEUE_PTR1_IBI_DATA_R(q) FIELD_GET(GENMASK(28, 24), q)
#define QUEUE_PTR1_IBI_DATA_W(q) FIELD_GET(GENMASK(20, 16), q)
#define QUEUE_PTR1_RX_R(q) FIELD_GET(GENMASK(12, 8), q)
#define QUEUE_PTR1_RX_W(q) FIELD_GET(GENMASK(4, 0), q)

#define ASPEED_I3C_INTR_STATUS 0xe0
#define ASPEED_I3C_INTR_STATUS_ENABLE 0xe4
#define ASPEED_I3C_INTR_SIGNAL_ENABLE 0xe8
#define ASPEED_I3C_INTR_FORCE 0xec
#define ASPEED_I3C_INTR_I2C_SDA_STUCK_LOW BIT(14)
#define ASPEED_I3C_INTR_I3C_SDA_STUCK_HIGH BIT(13)
#define ASPEED_I3C_INTR_I3C_SDA_STUCK_LOW BIT(12)
#define ASPEED_I3C_INTR_MST_INTERNAL_DONE BIT(10)
#define ASPEED_I3C_INTR_MST_DDR_READ_DONE BIT(9)
#define ASPEED_I3C_INTR_MST_DDR_WRITE_DONE BIT(8)
#define ASPEED_I3C_INTR_MST_IBI_DONE BIT(7)
#define ASPEED_I3C_INTR_MST_READ_DONE BIT(6)
#define ASPEED_I3C_INTR_MST_WRITE_DONE BIT(5)
#define ASPEED_I3C_INTR_MST_DAA_DONE BIT(4)
#define ASPEED_I3C_INTR_SLV_SCL_STUCK BIT(1)
#define ASPEED_I3C_INTR_TGRST BIT(0)

#define ASPEED_I3C_INTR_SUM_STATUS 0xf0
#define ASPEED_INTR_SUM_INHOUSE BIT(3)
#define ASPEED_INTR_SUM_RHS BIT(2)
#define ASPEED_INTR_SUM_PIO BIT(1)
#define ASPEED_INTR_SUM_CAP BIT(0)

#define ASPEED_I3C_INTR_RENEW 0xf4

static inline uint32_t ast_phy_read(const struct i3c_hci *hci, uint32_t reg)
{
	return sys_read32((mem_addr_t)(hci->PHY_regs + reg));
}

static inline void ast_phy_write(const struct i3c_hci *hci, uint32_t reg, uint32_t val)
{
	sys_write32(val, (mem_addr_t)(hci->PHY_regs + reg));
}

#define PHY_SW_FORCE_CTRL 0x4
#define PHY_SW_FORCE_CTRL_SCL_IN_EN BIT(31)
#define PHY_SW_FORCE_CTRL_SCL_OUT_EN BIT(30)
#define PHY_SW_FORCE_CTRL_SCL_OE_EN BIT(29)
#define PHY_SW_FORCE_CTRL_SCL_PU_EN BIT(28)
#define PHY_SW_FORCE_CTRL_SDA_IN_EN BIT(27)
#define PHY_SW_FORCE_CTRL_SDA_OUT_EN BIT(26)
#define PHY_SW_FORCE_CTRL_SDA_OE_EN BIT(25)
#define PHY_SW_FORCE_CTRL_SDA_PU_EN BIT(24)
#define PHY_SW_FORCE_CTRL_SCL_IN_VAL BIT(13)
#define PHY_SW_FORCE_CTRL_SCL_OUT_VAL BIT(12)
#define PHY_SW_FORCE_CTRL_SCL_OE_VAL BIT(11)
#define PHY_SW_FORCE_CTRL_SCL_PU_VAL GENMASK(10, 8)
#define PHY_SW_FORCE_CTRL_SDA_IN_VAL BIT(5)
#define PHY_SW_FORCE_CTRL_SDA_OUT_VAL BIT(4)
#define PHY_SW_FORCE_CTRL_SDA_OE_VAL BIT(3)
#define PHY_SW_FORCE_CTRL_SDA_PU_VAL GENMASK(2, 0)

#define PHY_I2C_FM_CTRL0 0x8
#define PHY_I2C_FM_CTRL0_CAS GENMASK(26, 16)
#define PHY_I2C_FM_CTRL0_SU_STO GENMASK(10, 0)
#define PHY_I2C_FM_CTRL1 0xc
#define PHY_I2C_FM_CTRL1_SCL_H GENMASK(26, 16)
#define PHY_I2C_FM_CTRL1_SCL_L GENMASK(10, 0)
#define PHY_I2C_FM_CTRL2 0x10
#define PHY_I2C_FM_CTRL2_ACK_H GENMASK(26, 16)
#define PHY_I2C_FM_CTRL2_ACK_L GENMASK(10, 0)
#define PHY_I2C_FM_CTRL3 0x14
#define PHY_I2C_FM_CTRL3_HD_DAT GENMASK(26, 16)
#define PHY_I2C_FM_CTRL3_AHD_DAT GENMASK(10, 0)

#define PHY_I2C_FM_DEFAULT_CAS_NS 1130
#define PHY_I2C_FM_DEFAULT_SU_STO_NS 1370
#define PHY_I2C_FM_DEFAULT_SCL_H_NS 1130
#define PHY_I2C_FM_DEFAULT_SCL_L_NS 1370
#define PHY_I2C_FM_DEFAULT_HD_DAT 10
#define PHY_I2C_FM_DEFAULT_AHD_DAT 10

#define PHY_I2C_FMP_CTRL0 0x18
#define PHY_I2C_FMP_CTRL0_CAS GENMASK(26, 16)
#define PHY_I2C_FMP_CTRL0_SU_STO GENMASK(10, 0)
#define PHY_I2C_FMP_CTRL1 0x1c
#define PHY_I2C_FMP_CTRL1_SCL_H GENMASK(26, 16)
#define PHY_I2C_FMP_CTRL1_SCL_L GENMASK(10, 0)
#define PHY_I2C_FMP_CTRL2 0x20
#define PHY_I2C_FMP_CTRL2_ACK_H GENMASK(26, 16)
#define PHY_I2C_FMP_CTRL2_ACK_L GENMASK(10, 0)
#define PHY_I2C_FMP_CTRL3 0x24
#define PHY_I2C_FMP_CTRL3_HD_DAT GENMASK(26, 16)
#define PHY_I2C_FMP_CTRL3_AHD_DAT GENMASK(10, 0)

#define PHY_I2C_FMP_DEFAULT_CAS_NS 380
#define PHY_I2C_FMP_DEFAULT_SU_STO_NS 620
#define PHY_I2C_FMP_DEFAULT_SCL_H_NS 380
#define PHY_I2C_FMP_DEFAULT_SCL_L_NS 620
#define PHY_I2C_FMP_DEFAULT_HD_DAT 10
#define PHY_I2C_FMP_DEFAULT_AHD_DAT 10

#define PHY_I3C_OD_CTRL0 0x28
#define PHY_I3C_OD_CTRL0_CAS GENMASK(26, 16)
#define PHY_I3C_OD_CTRL0_CBP GENMASK(10, 0)
#define PHY_I3C_OD_CTRL1 0x2c
#define PHY_I3C_OD_CTRL1_SCL_H GENMASK(26, 16)
#define PHY_I3C_OD_CTRL1_SCL_L GENMASK(10, 0)
#define PHY_I3C_OD_CTRL2 0x30
#define PHY_I3C_OD_CTRL2_ACK_H GENMASK(26, 16)
#define PHY_I3C_OD_CTRL2_ACK_L GENMASK(10, 0)
#define PHY_I3C_OD_CTRL3 0x34
#define PHY_I3C_OD_CTRL3_HD_DAT GENMASK(26, 16)
#define PHY_I3C_OD_CTRL3_AHD_DAT GENMASK(10, 0)

#define PHY_I3C_OD_DEFAULT_CAS_NS 40
#define PHY_I3C_OD_DEFAULT_CBP_NS 40
#define PHY_I3C_OD_DEFAULT_SCL_H_NS 380
#define PHY_I3C_OD_DEFAULT_SCL_L_NS 620
#define PHY_I3C_OD_DEFAULT_HD_DAT 10
#define PHY_I3C_OD_DEFAULT_AHD_DAT 10

#define PHY_I3C_SDR0_CTRL0 0x38
#define PHY_I3C_SDR0_CTRL0_SCL_H GENMASK(26, 16)
#define PHY_I3C_SDR0_CTRL0_SCL_L GENMASK(10, 0)
#define PHY_I3C_SDR0_CTRL1 0x3c
#define PHY_I3C_SDR0_CTRL1_TBIT_H GENMASK(26, 16)
#define PHY_I3C_SDR0_CTRL1_TBIT_L GENMASK(10, 0)
#define PHY_I3C_SDR0_CTRL2 0x40
#define PHY_I3C_SDR0_CTRL2_HD_PP GENMASK(26, 16)
#define PHY_I3C_SDR0_CTRL2_TBIT_HD_PP GENMASK(10, 0)

#define PHY_I3C_SDR0_DEFAULT_SCL_H_NS 380
#define PHY_I3C_SDR0_DEFAULT_SCL_L_NS 620
#define PHY_I3C_SDR0_DEFAULT_TBIT_H_NS 380
#define PHY_I3C_SDR0_DEFAULT_TBIT_L_NS 620
#define PHY_I3C_SDR0_DEFAULT_HD_PP_NS 10
#define PHY_I3C_SDR0_DEFAULT_TBIT_HD_PP_NS 10

#define PHY_I3C_CTRL0_OFFSET 0x0
#define PHY_I3C_CTRL1_OFFSET 0x4
#define PHY_I3C_CTRL2_OFFSET 0x8

#define PHY_I3C_SDR1_CTRL0 0x44
#define PHY_I3C_SDR1_CTRL0_SCL_H GENMASK(26, 16)
#define PHY_I3C_SDR1_CTRL0_SCL_L GENMASK(10, 0)
#define PHY_I3C_SDR1_CTRL1 0x48
#define PHY_I3C_SDR1_CTRL1_TBIT_H GENMASK(26, 16)
#define PHY_I3C_SDR1_CTRL1_TBIT_L GENMASK(10, 0)
#define PHY_I3C_SDR1_CTRL2 0x4c
#define PHY_I3C_SDR1_CTRL2_HD_PP GENMASK(26, 16)
#define PHY_I3C_SDR1_CTRL2_TBIT_HD_PP GENMASK(10, 0)

#define PHY_I3C_SDR2_CTRL0 0x50
#define PHY_I3C_SDR2_CTRL0_SCL_H GENMASK(26, 16)
#define PHY_I3C_SDR2_CTRL0_SCL_L GENMASK(10, 0)
#define PHY_I3C_SDR2_CTRL1 0x54
#define PHY_I3C_SDR2_CTRL1_TBIT_H GENMASK(26, 16)
#define PHY_I3C_SDR2_CTRL1_TBIT_L GENMASK(10, 0)
#define PHY_I3C_SDR2_CTRL2 0x58
#define PHY_I3C_SDR2_CTRL2_HD_PP GENMASK(26, 16)
#define PHY_I3C_SDR2_CTRL2_TBIT_HD_PP GENMASK(10, 0)

#define PHY_I3C_SDR3_CTRL0 0x5c
#define PHY_I3C_SDR3_CTRL0_SCL_H GENMASK(26, 16)
#define PHY_I3C_SDR3_CTRL0_SCL_L GENMASK(10, 0)
#define PHY_I3C_SDR3_CTRL1 0x60
#define PHY_I3C_SDR3_CTRL1_TBIT_H GENMASK(26, 16)
#define PHY_I3C_SDR3_CTRL1_TBIT_L GENMASK(10, 0)
#define PHY_I3C_SDR3_CTRL2 0x64
#define PHY_I3C_SDR3_CTRL2_HD_PP GENMASK(26, 16)
#define PHY_I3C_SDR3_CTRL2_TBIT_HD_PP GENMASK(10, 0)

#define PHY_I3C_SDR4_CTRL0 0x68
#define PHY_I3C_SDR4_CTRL0_SCL_H GENMASK(26, 16)
#define PHY_I3C_SDR4_CTRL0_SCL_L GENMASK(10, 0)
#define PHY_I3C_SDR4_CTRL1 0x6c
#define PHY_I3C_SDR4_CTRL1_TBIT_H GENMASK(26, 16)
#define PHY_I3C_SDR4_CTRL1_TBIT_L GENMASK(10, 0)
#define PHY_I3C_SDR4_CTRL2 0x70
#define PHY_I3C_SDR4_CTRL2_HD_PP GENMASK(26, 16)
#define PHY_I3C_SDR4_CTRL2_TBIT_HD_PP GENMASK(10, 0)

#define PHY_I3C_DDR_CTRL0 0x74
#define PHY_I3C_DDR_CTRL0_SCL_H GENMASK(26, 16)
#define PHY_I3C_DDR_CTRL0_SCL_L GENMASK(10, 0)
#define PHY_I3C_DDR_CTRL1 0x78
#define PHY_I3C_DDR_CTRL1_TBIT_H GENMASK(26, 16)
#define PHY_I3C_DDR_CTRL1_TBIT_L GENMASK(10, 0)
#define PHY_I3C_DDR_CTRL2 0x7c
#define PHY_I3C_DDR_CTRL2_HD_PP GENMASK(26, 16)
#define PHY_I3C_DDR_CTRL2_TBIT_HD_PP GENMASK(10, 0)

#define PHY_I3C_DDR_DEFAULT_SCL_H_NS 380
#define PHY_I3C_DDR_DEFAULT_SCL_L_NS 620
#define PHY_I3C_DDR_DEFAULT_TBIT_H_NS 380
#define PHY_I3C_DDR_DEFAULT_TBIT_L_NS 620
#define PHY_I3C_DDR_DEFAULT_HD_PP_NS 10
#define PHY_I3C_DDR_DEFAULT_TBIT_HD_PP_NS 10

#define PHY_I3C_SR_P_PREPARE_CTRL 0x80
#define PHY_I3C_SR_P_PREPARE_CTRL_HD GENMASK(26, 16)
#define PHY_I3C_SR_P_PREPARE_CTRL_SCL_L GENMASK(10, 0)
#define PHY_I3C_SR_P_DEFAULT_HD_NS 16
#define PHY_I3C_SR_P_DEFAULT_SCL_L_NS 40

#define PHY_PULLUP_EN 0x98
#define PHY_PULLUP_EN_SCL GENMASK(14, 12)
#define PHY_PULLUP_EN_SDA GENMASK(10, 8)
#define PHY_PULLUP_EN_DDR_SCL GENMASK(6, 4)
#define PHY_PULLUP_EN_DDR_SDA GENMASK(2, 0)

#define PHY_I3C_OD_CTRL4 0xd8
#define PHY_I3C_OD_CTRL4_DAP GENMASK(26, 16)
#define PHY_I3C_OD_DEFAULT_DAP_NS 12

#define MIPI_I3C_HCI_AUTOCMD_MODE_SDR 0x00U
#define MIPI_I3C_HCI_AUTOCMD_MODE_HDR_DDR 0x01U
#define MIPI_I3C_HCI_AUTOCMD_MODE_I2C 0x08U

#define MIPI_I3C_HCI_AUTOCMD_ROC BIT(0)
#define MIPI_I3C_HCI_AUTOCMD_TOC BIT(1)
#define MIPI_I3C_HCI_AUTOCMD_RNW BIT(2)
#define MIPI_I3C_HCI_AUTOCMD_FLAGS_MASK                                                   \
	(MIPI_I3C_HCI_AUTOCMD_ROC | MIPI_I3C_HCI_AUTOCMD_TOC |                         \
	 MIPI_I3C_HCI_AUTOCMD_RNW)

struct mipi_i3c_hci_autocmd_entry {
	uint8_t slot;
	uint8_t target_addr;
	uint8_t ccc_or_cmd;
	uint8_t mode;
	uint16_t data_len;
	uint32_t flags;
};

#ifdef CONFIG_I3C_MIPI_HCI_ASPEED_VENDOR
extern const struct mipi_i3c_hci_vendor_ops mipi_i3c_hci_aspeed_ops;
int mipi_i3c_hci_aspeed_init(struct i3c_hci *hci);
void mipi_i3c_hci_aspeed_phy_init(struct i3c_hci *hci);
void mipi_i3c_hci_aspeed_populate_bus_timing(struct i3c_hci *hci);
uint32_t mipi_i3c_hci_aspeed_get_sdr_phy_reg(struct i3c_hci *hci);
void mipi_i3c_hci_aspeed_set_ibi_terminate_len(struct i3c_hci *hci, uint16_t max_len);
void mipi_i3c_hci_aspeed_set_slv_pid(struct i3c_hci *hci, uint64_t pid);
void mipi_i3c_hci_aspeed_set_slv_char_ctrl(struct i3c_hci *hci, uint8_t bcr, uint8_t dcr,
					   bool static_addr_en);
bool mipi_i3c_hci_aspeed_payload_too_big(unsigned int data_len);
uint32_t mipi_i3c_hci_aspeed_get_status(struct i3c_hci *hci);
void mipi_i3c_hci_aspeed_dma_start(struct i3c_hci *hci);
void mipi_i3c_hci_aspeed_dma_drain(struct i3c_hci *hci);
uint32_t mipi_i3c_hci_aspeed_ring_status(struct i3c_hci *hci);
void mipi_i3c_hci_aspeed_ccc_handler(struct i3c_hci *hci, uint8_t ccc);
#if defined(CONFIG_I3C_MIPI_HCI_ASPEED_AUTOCMD)
int mipi_i3c_hci_aspeed_autocmd_init(struct i3c_hci *hci);
int mipi_i3c_hci_aspeed_autocmd_install(struct i3c_hci *hci,
					const struct mipi_i3c_hci_autocmd_entry *entry);
int mipi_i3c_hci_aspeed_autocmd_remove(struct i3c_hci *hci, uint8_t slot);
int mipi_i3c_hci_aspeed_autocmd_enable(struct i3c_hci *hci, uint8_t slot, bool enable);
void mipi_i3c_hci_aspeed_autocmd_set_trigger(struct i3c_hci *hci, uint8_t slot,
					     uint8_t ibi_addr);
#else
static inline int mipi_i3c_hci_aspeed_autocmd_init(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
	return 0;
}

static inline int
mipi_i3c_hci_aspeed_autocmd_install(struct i3c_hci *hci,
				    const struct mipi_i3c_hci_autocmd_entry *entry)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(entry);
	return -ENOTSUP;
}

static inline int mipi_i3c_hci_aspeed_autocmd_remove(struct i3c_hci *hci, uint8_t slot)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(slot);
	return -ENOTSUP;
}

static inline int mipi_i3c_hci_aspeed_autocmd_enable(struct i3c_hci *hci, uint8_t slot,
						     bool enable)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(slot);
	ARG_UNUSED(enable);
	return -ENOTSUP;
}

static inline void mipi_i3c_hci_aspeed_autocmd_set_trigger(struct i3c_hci *hci,
							   uint8_t slot, uint8_t ibi_addr)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(slot);
	ARG_UNUSED(ibi_addr);
}
#endif /* CONFIG_I3C_MIPI_HCI_ASPEED_AUTOCMD */
#else
static inline int mipi_i3c_hci_aspeed_init(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
	return 0;
}

static inline void mipi_i3c_hci_aspeed_phy_init(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
}

static inline void mipi_i3c_hci_aspeed_populate_bus_timing(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
}

static inline uint32_t mipi_i3c_hci_aspeed_get_sdr_phy_reg(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
	return PHY_I3C_SDR4_CTRL0;
}

static inline void mipi_i3c_hci_aspeed_set_ibi_terminate_len(struct i3c_hci *hci,
							     uint16_t max_len)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(max_len);
}

static inline void mipi_i3c_hci_aspeed_set_slv_pid(struct i3c_hci *hci, uint64_t pid)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(pid);
}

static inline void mipi_i3c_hci_aspeed_set_slv_char_ctrl(struct i3c_hci *hci, uint8_t bcr,
							 uint8_t dcr, bool static_addr_en)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(bcr);
	ARG_UNUSED(dcr);
	ARG_UNUSED(static_addr_en);
}

static inline bool mipi_i3c_hci_aspeed_payload_too_big(unsigned int data_len)
{
	ARG_UNUSED(data_len);
	return false;
}

static inline uint32_t mipi_i3c_hci_aspeed_get_status(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
	return 0;
}

static inline void mipi_i3c_hci_aspeed_dma_start(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
}

static inline void mipi_i3c_hci_aspeed_dma_drain(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
}

static inline uint32_t mipi_i3c_hci_aspeed_ring_status(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
	return I3C_RING_IDLE;
}

static inline void mipi_i3c_hci_aspeed_ccc_handler(struct i3c_hci *hci, uint8_t ccc)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(ccc);
}

static inline int mipi_i3c_hci_aspeed_autocmd_init(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
	return 0;
}

static inline int
mipi_i3c_hci_aspeed_autocmd_install(struct i3c_hci *hci,
				    const struct mipi_i3c_hci_autocmd_entry *entry)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(entry);
	return -ENOTSUP;
}

static inline int mipi_i3c_hci_aspeed_autocmd_remove(struct i3c_hci *hci, uint8_t slot)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(slot);
	return -ENOTSUP;
}

static inline int mipi_i3c_hci_aspeed_autocmd_enable(struct i3c_hci *hci, uint8_t slot,
						     bool enable)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(slot);
	ARG_UNUSED(enable);
	return -ENOTSUP;
}

static inline void mipi_i3c_hci_aspeed_autocmd_set_trigger(struct i3c_hci *hci,
							   uint8_t slot, uint8_t ibi_addr)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(slot);
	ARG_UNUSED(ibi_addr);
}
#endif /* CONFIG_I3C_MIPI_HCI_ASPEED_VENDOR */

#endif /* ZEPHYR_DRIVERS_I3C_MIPI_HCI_VENDOR_ASPEED_H_ */
