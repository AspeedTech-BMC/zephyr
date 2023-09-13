/*
 * Copyright (c) 2023 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_espi_v2

#include <soc.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_aspeed.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include "espi_utils.h"

LOG_MODULE_REGISTER(espi);

/* global registers */
#define ESPI_CTRL			0x000
#define ESPI_STS			0x004
#define ESPI_INT_STS			0x008
#define   ESPI_INT_STS_RST_DEASSERT	BIT(31)
#define   ESPI_INT_STS_RST_ASSERT	BIT(30)
#define   ESPI_INT_STS_CH3		BIT(3)
#define   ESPI_INT_STS_CH2		BIT(2)
#define   ESPI_INT_STS_CH1		BIT(1)
#define   ESPI_INT_STS_CH0		BIT(0)
#define ESPI_INT_EN			0x00c
#define   ESPI_INT_EN_RST_DEASSERT	BIT(31)
#define   ESPI_INT_EN_RST_ASSERT	BIT(30)
#define ESPI_DEV_ID			0x010
#define ESPI_CAP_GEN			0x014
#define ESPI_CAP_CH0			0x018
#define ESPI_CAP_CH1			0x01c
#define ESPI_CAP_CH2			0x020
#define ESPI_CAP_CH3_0			0x024
#define ESPI_CAP_CH3_1			0x028
#define ESPI_DEV_STS			0x030
#define ESPI_DBG_CTRL			0x034
#define ESPI_DBG_ADDRL			0x038
#define ESPI_DBG_ADDRH			0x03c
#define ESPI_DBG_CMD			0x040
#define ESPI_DBG_RES			0x044
#define ESPI_CH_ACC_CTRL		0x04c
#define ESPI_CH_ACC_OFST1		0x050
#define ESPI_CH_ACC_OFST2		0x054
#define ESPI_WPROT0			0x0f8
#define ESPI_WPROT1			0x0fc

/* peripheral channel (ch0) registers */
#define ESPI_CH0_CTRL			0x100
#define   ESPI_CH0_CTRL_NP_TX_RST	BIT(31)
#define   ESPI_CH0_CTRL_NP_RX_RST	BIT(30)
#define   ESPI_CH0_CTRL_PC_TX_RST	BIT(29)
#define   ESPI_CH0_CTRL_PC_RX_RST	BIT(28)
#define   ESPI_CH0_CTRL_NP_TX_DMA_EN	BIT(19)
#define   ESPI_CH0_CTRL_PC_TX_DMA_EN	BIT(17)
#define   ESPI_CH0_CTRL_PC_RX_DMA_EN	BIT(16)
#define   ESPI_CH0_CTRL_MCYC_RD_DIS	BIT(6)
#define   ESPI_CH0_CTRL_MCYC_WR_DIS	BIT(4)
#define   ESPI_CH0_CTRL_SW_RDY		BIT(1)
#define ESPI_CH0_STS			0x104
#define ESPI_CH0_INT_STS		0x108
#define   ESPI_CH0_INT_STS_PC_RX_CMPLT	BIT(0)
#define ESPI_CH0_INT_EN			0x10c
#define   ESPI_CH0_INT_EN_PC_RX_CMPLT	BIT(0)
#define ESPI_CH0_PC_RX_DMAL		0x110
#define ESPI_CH0_PC_RX_DMAH		0x114
#define ESPI_CH0_PC_RX_CTRL		0x118
#define   ESPI_CH0_PC_RX_CTRL_SERV_PEND	BIT(31)
#define   ESPI_CH0_PC_RX_CTRL_LEN	GENMASK(23, 12)
#define   ESPI_CH0_PC_RX_CTRL_TAG	GENMASK(11, 8)
#define   ESPI_CH0_PC_RX_CTRL_CYC	GENMASK(7, 0)
#define ESPI_CH0_PC_RX_DATA		0x11c
#define ESPI_CH0_PC_TX_DMAL		0x120
#define ESPI_CH0_PC_TX_DMAH		0x124
#define ESPI_CH0_PC_TX_CTRL		0x128
#define   ESPI_CH0_PC_TX_CTRL_TRIG_PEND	BIT(31)
#define   ESPI_CH0_PC_TX_CTRL_LEN	GENMASK(23, 12)
#define   ESPI_CH0_PC_TX_CTRL_TAG	GENMASK(11, 8)
#define   ESPI_CH0_PC_TX_CTRL_CYC	GENMASK(7, 0)
#define ESPI_CH0_PC_TX_DATA		0x12c
#define ESPI_CH0_NP_TX_DMAL		0x130
#define ESPI_CH0_NP_TX_DMAH		0x134
#define ESPI_CH0_NP_TX_CTRL		0x138
#define   ESPI_CH0_NP_TX_CTRL_TRIG_PEND	BIT(31)
#define   ESPI_CH0_NP_TX_CTRL_LEN	GENMASK(23, 12)
#define   ESPI_CH0_NP_TX_CTRL_TAG	GENMASK(11, 8)
#define   ESPI_CH0_NP_TX_CTRL_CYC	GENMASK(7, 0)
#define ESPI_CH0_NP_TX_DATA		0x13c
#define ESPI_CH0_MCYC0_SADDRL		0x140
#define ESPI_CH0_MCYC0_SADDRH		0x144
#define ESPI_CH0_MCYC0_TADDRL		0x148
#define ESPI_CH0_MCYC0_TADDRH		0x14c
#define ESPI_CH0_MCYC0_MASKL		0x150
#define   ESPI_CH0_MCYC0_MASKL_EN	BIT(0)
#define ESPI_CH0_MCYC0_MASKH		0x154
#define ESPI_CH0_MCYC1_SADDRL		0x158
#define ESPI_CH0_MCYC1_SADDRH		0x15c
#define ESPI_CH0_MCYC1_TADDRL		0x160
#define ESPI_CH0_MCYC1_TADDRH		0x164
#define ESPI_CH0_MCYC1_MASKL		0x168
#define   ESPI_CH0_MCYC1_MASKL_EN	BIT(0)
#define ESPI_CH0_MCYC1_MASKH		0x16c
#define ESPI_CH0_WPROT0			0x1f8
#define ESPI_CH0_WPROT1			0x1fc

/* virtual wire channel (ch1) registers */
#define ESPI_CH1_CTRL			0x200
#define   ESPI_CH1_CTRL_GPIO_HW		BIT(9)
#define   ESPI_CH1_CTRL_SW_RDY		BIT(1)
#define ESPI_CH1_STS			0x204
#define ESPI_CH1_INT_STS		0x208
#define   ESPI_CH1_INT_STS_GPIO		BIT(2)
#define ESPI_CH1_INT_EN			0x20c
#define   ESPI_CH1_INT_EN_GPIO		BIT(2)
#define ESPI_CH1_EVT0			0x210
#define ESPI_CH1_EVT0_INT_EN		0x214
#define ESPI_CH1_EVT0_INT_T0		0x218
#define ESPI_CH1_EVT0_INT_T1		0x21c
#define ESPI_CH1_EVT0_INT_T2		0x220
#define ESPI_CH1_EVT0_INT_STS		0x224
#define ESPI_CH1_EVT1			0x230
#define ESPI_CH1_EVT1_INT_EN		0x234
#define ESPI_CH1_EVT1_INT_T0		0x238
#define ESPI_CH1_EVT1_INT_T1		0x23c
#define ESPI_CH1_EVT1_INT_T2		0x240
#define ESPI_CH1_EVT1_INT_STS		0x244
#define ESPI_CH1_GPIO_VAL0		0x250
#define ESPI_CH1_GPIO_VAL1		0x254
#define ESPI_CH1_GPIO_DIR0		0x258
#define ESPI_CH1_GPIO_DIR1		0x258
#define ESPI_CH1_GPIO_RSTSEL0		0x260
#define ESPI_CH1_GPIO_RSTSEL1		0x264
#define ESPI_CH1_GPIO_GRP		0x268
#define ESPI_CH1_GP50_DIR0		0x270
#define ESPI_CH1_GP50_DIR1		0x274
#define ESPI_CH1_GP50_VAL0		0x278
#define ESPI_CH1_GP50_VAL1		0x27c
#define ESPI_CH1_SW_INT			0x280
#define ESPI_CH1_INT_RSTSEL0		0x284
#define ESPI_CH1_INT_RSTSEL1		0x288
#define ESPI_CH1_WPROT0			0x2f8
#define ESPI_CH1_WPROT1			0x2fc

/* out-of-band channel (ch2) registers */
#define ESPI_CH2_CTRL			0x300
#define   ESPI_CH2_CTRL_TX_RST		BIT(31)
#define   ESPI_CH2_CTRL_RX_RST		BIT(30)
#define   ESPI_CH2_CTRL_TX_DMA_EN	BIT(17)
#define   ESPI_CH2_CTRL_RX_DMA_EN	BIT(16)
#define   ESPI_CH2_CTRL_SW_RDY		BIT(4)
#define ESPI_CH2_STS			0x304
#define ESPI_CH2_INT_STS		0x308
#define   ESPI_CH2_INT_STS_RX_CMPLT	BIT(0)
#define ESPI_CH2_INT_EN			0x30c
#define   ESPI_CH2_INT_EN_RX_CMPLT	BIT(0)
#define ESPI_CH2_RX_DMAL		0x310
#define ESPI_CH2_RX_DMAH		0x314
#define ESPI_CH2_RX_CTRL		0x318
#define   ESPI_CH2_RX_CTRL_SERV_PEND	BIT(31)
#define   ESPI_CH2_RX_CTRL_PEC		BIT(24)
#define   ESPI_CH2_RX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_CH2_RX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_CH2_RX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_CH2_RX_DATA		0x31c
#define ESPI_CH2_TX_DMAL		0x320
#define ESPI_CH2_TX_DMAH		0x324
#define ESPI_CH2_TX_CTRL		0x328
#define   ESPI_CH2_TX_CTRL_TRIG_PEND	BIT(31)
#define   ESPI_CH2_TX_CTRL_PEC		BIT(24)
#define   ESPI_CH2_TX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_CH2_TX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_CH2_TX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_CH2_TX_DATA		0x32c
#define ESPI_CH2_RX_DESC_EPTR		0x330
#define ESPI_CH2_RX_DESC_RPTR		0x334
#define ESPI_CH2_RX_DESC_WPTR		0x338
#define   ESPI_CH2_RX_DESC_WPTR_VALID	BIT(31)
#define ESPI_CH2_RX_DESC_TMOUT		0x33c
#define ESPI_CH2_TX_DESC_EPTR		0x340
#define ESPI_CH2_TX_DESC_RPTR		0x344
#define   ESPI_CH2_TX_DESC_RPTR_UPT	BIT(31)
#define ESPI_CH2_TX_DESC_WPTR		0x348
#define   ESPI_CH2_TX_DESC_WPTR_VALID	BIT(31)
#define ESPI_CH2_WPROT0			0x3f8
#define ESPI_CH2_WPROT1			0x3fc

/* flash channel (ch3) registers */
#define ESPI_CH3_CTRL			0x400
#define   ESPI_CH3_CTRL_TX_RST		BIT(31)
#define   ESPI_CH3_CTRL_RX_RST		BIT(30)
#define   ESPI_CH3_CTRL_TX_DMA_EN	BIT(17)
#define   ESPI_CH3_CTRL_RX_DMA_EN	BIT(16)
#define   ESPI_CH3_CTRL_EDAF_MODE	GENMASK(9, 8)
#define   ESPI_CH3_CTRL_SW_RDY		BIT(5)
#define ESPI_CH3_STS			0x404
#define ESPI_CH3_INT_STS		0x408
#define   ESPI_CH3_INT_STS_RX_CMPLT	BIT(0)
#define ESPI_CH3_INT_EN			0x40c
#define   ESPI_CH3_INT_EN_RX_CMPLT	BIT(0)
#define ESPI_CH3_RX_DMAL		0x410
#define ESPI_CH3_RX_DMAH		0x414
#define ESPI_CH3_RX_CTRL		0x418
#define   ESPI_CH3_RX_CTRL_SERV_PEND	BIT(31)
#define   ESPI_CH3_RX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_CH3_RX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_CH3_RX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_CH3_RX_DATA		0x41c
#define ESPI_CH3_TX_DMAL		0x420
#define ESPI_CH3_TX_DMAH		0x424
#define ESPI_CH3_TX_CTRL		0x428
#define   ESPI_CH3_TX_CTRL_TRIG_PEND	BIT(31)
#define   ESPI_CH3_TX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_CH3_TX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_CH3_TX_CTRL_CYC		GENMASK(7, 0)
#define ESPI_CH3_TX_DATA		0x42c
#define ESPI_CH3_EDAF_TADDRL		0x430
#define ESPI_CH3_EDAF_TADDRH		0x434
#define ESPI_CH3_EDAF_MASKL		0x438
#define ESPI_CH3_EDAF_MASKH		0x43c
#define ESPI_CH3_WPROT0			0x4f8
#define ESPI_CH3_WPROT1			0x4fc

/* helper macro */
#define ESPI_RD(reg)            sys_read32(espi_base + (reg))
#define ESPI_WR(val, reg)       sys_write32((uint32_t)val, espi_base + (reg))

/* constant */
#define PERIF_MCYC_ALIGN	0x10000
#define OOB_DMA_RPTR_KEY	0x4f4f4253
#define OOB_DMA_DESC_NUM	2
#define OOB_DMA_DESC_CUSTOM	0x4
#define OOB_DMA_BUF_SIZE	(OOB_DMA_DESC_NUM * ESPI_PLD_LEN_MAX)
#define FLASH_EDAF_ALIGN	0x1000000

/* driver data structure */
struct espi_ast2700_perif {
	struct {
		bool enable;
		uint8_t *virt;
		uint64_t saddr;
		uint64_t taddr;
		uint64_t mcyc_size;
	} mcyc;

	struct {
		bool enable;
		uint8_t *pc_rx_virt;
		uint64_t pc_rx_addr;
		uint8_t *pc_tx_virt;
		uint64_t pc_tx_addr;
		uint8_t *np_tx_virt;
		uint64_t np_tx_addr;
	} dma;

	struct k_sem pc_tx_lock;
	struct k_sem np_tx_lock;
	struct k_sem rx_lock;
	struct k_sem rx_ready;
};

struct espi_ast2700_vw {
	struct {
		bool hw_mode;
		uint32_t grp;
		uint32_t dir0;
		uint32_t dir1;
		uint32_t val0;
		uint32_t val1;
	} gpio;
};

struct espi_ast2700_oob_dma_tx_desc {
	uint32_t data_addrl;
	uint32_t data_addrh;
	uint8_t cyc;
	uint16_t tag : 4;
	uint16_t len : 12;
	uint8_t msg_type : 3;
	uint8_t raz0 : 1;
	uint8_t pec : 1;
	uint8_t int_en : 1;
	uint8_t pause : 1;
	uint8_t raz1 : 1;
	uint32_t raz2;
	uint32_t raz3;
	uint32_t pad[3];
} __packed;

struct espi_ast2700_oob_dma_rx_desc {
	uint32_t data_addrl;
	uint32_t data_addrh;
	uint8_t cyc;
	uint16_t tag : 4;
	uint16_t len : 12;
	uint8_t raz : 7;
	uint8_t dirty : 1;
	uint32_t pad[1];
} __packed;

struct espi_ast2700_oob {
	struct {
		bool enable;
		struct espi_ast2700_oob_dma_tx_desc *txd_virt;
		uint64_t txd_addr;
		struct espi_ast2700_oob_dma_tx_desc *rxd_virt;
		uint64_t rxd_addr;
		uint8_t *tx_virt;
		uint64_t tx_addr;
		uint8_t *rx_virt;
		uintptr_t rx_addr;
	} dma;

	struct k_sem tx_lock;
	struct k_sem rx_lock;
	struct k_sem rx_ready;
};

struct espi_ast2700_flash {
	struct {
		uint32_t mode;
		uint64_t taddr;
		uint64_t size;
	} edaf;

	struct {
		bool enable;
		uint8_t *tx_virt;
		uint64_t tx_addr;
		uint8_t *rx_virt;
		uint64_t rx_addr;
	} dma;

	struct k_sem tx_lock;
	struct k_sem rx_lock;
	struct k_sem rx_ready;
};

struct espi_ast2700_data {
	const struct device *dev;
	struct espi_ast2700_perfi perif;
	struct espi_ast2700_vw vw;
	struct espi_ast2700_oob oob;
	struct espi_ast2700_flash flash;
	sys_slist_t callbacks;
};

static uint32_t espi_base;
static struct espi_ast2700_data espi_ast2700_data;

/* peripheral channel */
#if DT_INST_PROP(0, perif_dma_mode)
static uint8_t perif_pc_rx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t perif_pc_tx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t perif_np_tx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
#else
static uint8_t perif_pc_rx_buf[0];
static uint8_t perif_pc_tx_buf[0];
static uint8_t perif_np_tx_buf[0];
#endif

#if DT_INST_PROP(0, perif_mcyc_enable)
static uint8_t perif_mcyc_buf[DT_INST_PROP(0, perif_mcyc_size)]
	__aligned(DT_INST_PROP(0, perif_mcyc_size)) NON_CACHED_BSS;
#else
static uint8_t perif_mcyc_buf[0];
#endif

static void espi_ast2700_perif_isr(struct espi_ast2700_data *data)
{
	struct espi_ast2700_perif *perif = &data->perif;
	uint32_t sts;

	sts = ESPI_RD(ESPI_CH0_INT_STS);

	if (sts & ESPI_CH0_INT_STS_PC_RX_CMPLT) {
		ESPI_WR(ESPI_CH0_INT_STS_PC_RX_CMPLT, ESPI_CH0_INT_STS);
		k_sem_give(&perif->rx_ready);
	}
}

static void espi_ast2700_perif_reset(struct espi_ast2700_perif *perif)
{
	uint32_t reg;

	ESPI_WR(0x0, ESPI_CH0_INT_EN);
	ESPI_WR(0xffffffff, ESPI_CH0_INT_STS);

	reg = ESPI_RD(ESPI_CH0_MCYC1_MASKL)
	reg &= ~ESPI_CH0_MCYC1_MASKL_EN;
	ESPI_WR(reg, ESPI_CH0_MCYC1_MASKL);

	reg = ESPI_RD(ESPI_CH0_CTRL);
	reg |= (ESPI_CH0_CTRL_MCYC_RD_DIS | ESPI_CH0_CTRL_MCYC_WR_DIS);
	reg &= ~(ESPI_CH0_CTRL_NP_TX_RST
		 | ESPI_CH0_CTRL_NP_RX_RST
		 | ESPI_CH0_CTRL_PC_TX_RST
		 | ESPI_CH0_CTRL_PC_RX_RST
		 | ESPI_CH0_CTRL_NP_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_RX_DMA_EN
		 | ESPI_CH0_CTRL_SW_RDY);
	ESPI_WR(reg, ESPI_CH0_CTRL);

	reg |= (ESPI_CH0_CTRL_NP_TX_RST
		| ESPI_CH0_CTRL_NP_RX_RST
		| ESPI_CH0_CTRL_PC_TX_RST
		| ESPI_CH0_CTRL_PC_RX_RST);
	ESPI_WR(reg, ESPI_CH0_CTRL);

	if (perif->mcyc.enable) {
		mask = ~(perif->mcyc.size - 1);
		ESPI_WR(mask >> 32, ESPI_CH0_MCYC1_MASKH);
		ESPI_WR(mask & 0xffffffff, ESPI_CH0_MCYC1_MASKH);
		ESPI_WR((perif->mcyc.saddr >> 32), ESPI_CH0_MCYC1_SADDRH);
		ESPI_WR((perif->mcyc.saddr & 0xffffffff), ESPI_CH0_MCYC1_SADDRH);
		ESPI_WR((perif->mcyc.taddr >> 32), ESPI_CH0_MCYC1_TADDRH);
		ESPI_WR((perif->mcyc.taddr & 0xffffffff), ESPI_CH0_MCYC1_TADDRH);

		reg = ESPI_RD(RSPI_CH0_MCYC1_MASKL) | ESPI_CH0_MCYC1_MASKL_EN;
		ESPI_WR(reg, ESPI_CH0_MCYC1_MASKL);

		reg = ESPI_RD(ESPI_CH0_CTRL);
		reg &= ~(ESPI_CH0_CTRL_MCYC_RD_DIS | ESPI_CH0_CTRL_MCYC_WR_DIS);
		ESPI_WR(reg, ESPI_CH0_CTRL);
	}

	if (perif->dma.enable) {
		ESPI_WR((perif->dma.np_tx_addr >> 32), ESPI_CH0_NP_TX_DMAH);
		ESPI_WR((perif->dma.np_tx_addr & 0xffffffff), ESPI_CH0_NP_TX_DMAL);
		ESPI_WR((perif->dma.pc_tx_addr >> 32), ESPI_CH0_PC_TX_DMAH);
		ESPI_WR((perif->dma.pc_tx_addr & 0xffffffff), ESPI_CH0_PC_TX_DMAL);
		ESPI_WR((perif->dma.pc_rx_addr >> 32), ESPI_CH0_PC_RX_DMAH);
		ESPI_WR((perif->dma.pc_rx_addr & 0xffffffff), ESPI_CH0_PC_RX_DMAL);

		reg = readl(ESPI_CH0_CTRL)
		      | ESPI_CH0_CTRL_NP_TX_DMA_EN
		      | ESPI_CH0_CTRL_PC_TX_DMA_EN
		      | ESPI_CH0_CTRL_PC_RX_DMA_EN;
		ESPI_WR(reg, ESPI_CH0_CTRL);
	}

	ESPI_WR(ESPI_CH0_INT_EN_PC_RX_CMPLT, ESPI_CH0_INT_EN);

	reg = ESPI_RD(ESPI_CH0_CTRL) | ESPI_CH0_CTRL_SW_RDY;
	ESPI_WR(reg, ESPI_CH0_CTRL);
}

static void espi_ast2700_perif_init(struct espi_ast2700_perif *perif)
{
	perif->dma.enable = DT_INST_PROP(0, perif_dma_mode);
	perif->dma.pc_rx_virt = perif_pc_rx_buf;
	perif->dma.pc_rx_addr = TO_PHY_ADDR(perif->pc_rx_virt);
	perif->dma.pc_tx_virt = perif_pc_tx_buf;
	perif->dma.pc_tx_addr = TO_PHY_ADDR(perif->pc_tx_virt);
	perif->dma.np_tx_virt = perif_np_tx_buf;
	perif->dma.np_tx_addr = TO_PHY_ADDR(perif->np_tx_virt);

	perif->mcyc.enable = DT_INST_PROP(0, perif_mcyc_enable);
	perif->mcyc.virt = perif_mcyc_buf;
	perif->mcyc.size = (uint32_t)(DT_INST_PROP_OR(0, perif_mcyc_size, 0) << 32)
			   | (DT_INST_PROP_OR(1, perif_mcyc_size, 0);
	perif->mcyc.saddr = (uint32_t)(DT_INST_PROP_OR(0, perif_mcyc_src_addr, 0) << 32)
			    | (DT_INST_PROP_OR(1, perif_mcyc_src_addr, 0);
	perif->mcyc.taddr = TO_PHY_ADDR(perif->mcyc_virt);

	k_sem_init(&perif->pc_tx_lock, 1, 1);
	k_sem_init(&perif->np_tx_lock, 1, 1);
	k_sem_init(&perif->rx_lock, 1, 1);
	k_sem_init(&perif->rx_ready, 0, 1);
}

/* virtual wire channel */
static void espi_ast2700_vw_isr(struct espi_ast2700_data *data)
{
	struct espi_event evt_vw = { ESPI_BUS_EVENT_VWIRE_RECEIVED, 0, 0 };
	struct espi_ast2700_vw *vw = &data->vw;
	uint32_t sts, evt;

	espi_send_callbacks(&data->callbacks, data->dev, evt_vw);

	sts = ESPI_RD(ESPI_CH1_INT_STS);

	if (sts & ESPI_CH1_INT_STS_GPIO) {
		vw->gpio.val0 = ESPI_RD(ESPI_CH1_GPIO_VAL0);
		vw->gpio.val1 = ESPI_RD(ESPI_CH1_GPIO_VAL0);
		ESPI_WR(ESPI_CH1_INT_STS_GPIO, ESPI_CH1_INT_STS);
	}
}

static void espi_ast2700_vw_reset(struct espi_ast2700_vw *vw)
{
	uint32_t reg;

	ESPI_WR(0x0, ESPI_CH1_INT_EN);
	ESPI_WR(0xffffffff, ESPI_CH1_INT_STS);

	ESPI_WR(vw->gpio.grp, ESPI_CH1_GPIO_GRP);
	ESPI_WR(vw->gpio.dir0, ESPI_CH1_GPIO_DIR0);
	ESPI_WR(vw->gpio.dir1, ESPI_CH1_GPIO_DIR1);

	vw->gpio.val0 = ESPI_RD(ESPI_CH1_GPIO_VAL0);
	vw->gpio.val1 = ESPI_RD(ESPI_CH1_GPIO_VAL1);

	ESPI_WR(ESPI_CH1_INT_EN_GPIO, ESPI_CH1_INT_EN);

	reg = ESPI_RD(ESPI_CH1_CTRL)
	      | ((vw->gpio.hw_mode) ? ESPI_CH1_CTRL_GPIO_HW : 0)
	      | ESPI_CH1_CTRL_SW_RDY;
	ESPI_WR(reg, ESPI_CH1_CTRL);
}

static void espi_ast2700_vw_init(struct espi_ast2700_vw *vw)
{
	vw->gpio.hw_mode = DT_INST_PROP(0, vw_gpio_hw_mode);
	vw->gpio.grp = DT_INST_PROP_OR(0, vw_gpio_group, 0);
	vw->gpio.dir0 = DT_INST_PROP_OR(0, vw_gpio_direction, 0);
	vw->gpio.dir1 = DT_INST_PROP_OR(1, vw_gpio_direction, 0);
}

/* out of band channel */
#if DT_INST_PROP(0, oob_dma_mode)
static struct oob_tx_dma_desc oob_tx_desc[OOB_DMA_DESC_NUM] NON_CACHED_BSS;
static struct oob_rx_dma_desc oob_rx_desc[OOB_DMA_DESC_NUM] NON_CACHED_BSS;
static uint8_t oob_tx_buf[OOB_DMA_BUF_SIZE] NON_CACHED_BSS;
static uint8_t oob_rx_buf[OOB_DMA_BUF_SIZE] NON_CACHED_BSS;
#else
static struct oob_tx_dma_desc oob_tx_desc[0];
static struct oob_rx_dma_desc oob_rx_desc[0];
static uint8_t oob_tx_buf[0];
static uint8_t oob_rx_buf[0];
#endif

static void espi_ast2700_oob_isr(uint32_t sts, struct espi_ast2700_data *data)
{
	struct espi_ast2700_oob *oob = data->oob;
	uint32_t sts;

	sts = ESPI_RD(ESPI_CH2_INT_STS);

	if (sts & ESPI_CH2_INT_STS_RX_CMPLT) {
		ESPI_WR(ESPI_CH2_INT_STS_RX_CMPLT, ESPI_CH2_INT_STS);
		k_sem_give(&oob->rx_ready);
	}
}

static void espi_ast2700_oob_reset(struct espi_ast2700_oob *oob)
{
	uint64_t tx_addr, rx_addr;
	uint32_t reg;
	int i;

	ESPI_WR(0x0, ESPI_CH2_INT_EN);
	ESPI_WR(0xffffffff, ESPI_CH2_INT_STS);

	reg = ESPI_RD(ESPI_CH2_CTRL);
	reg &= ~(ESPI_CH2_CTRL_TX_RST
		 | ESPI_CH2_CTRL_RX_RST
		 | ESPI_CH2_CTRL_TX_DMA_EN
		 | ESPI_CH2_CTRL_RX_DMA_EN
		 | ESPI_CH2_CTRL_SW_RDY);
	ESPI_WR(reg, ESPI_CH2_CTRL);

	reg |= (ESPI_CH2_CTRL_TX_RST | ESPI_CH2_CTRL_RX_RST);
	ESPI_WR(reg, ESPI_CH2_CTRL);

	if (oob->dma.enable) {
		tx_addr = oob->dma.tx_addr;
		rx_addr = oob->dma.rx_addr;

		for (i = 0; i < OOB_DMA_DESC_NUM; ++i) {
			oob->dma.txd_virt[i].data_addrh = tx_addr >> 32;
			oob->dma.txd_virt[i].data_addrl = tx_addr & 0xffffffff;
			tx_addr += ESPI_PLD_LEN_MAX;

			oob->dma.rxd_virt[i].data_addrh = rx_addr >> 32;
			oob->dma.rxd_virt[i].data_addrl = rx_addr & 0xffffffff;
			oob->dma.rxd_virt[i].dirty = 0;
			rx_addr += ESPI_PLD_LEN_MAX;
		}

		ESPI_WR(oob->dma.txd_addr >> 32, ESPI_CH2_TX_DMAH);
		ESPI_WR(oob->dma.txd_addr & 0xffffffff, ESPI_CH2_TX_DMAL);
		ESPI_WR(OOB_DMA_RPTR_KEY, ESPI_CH2_TX_DESC_RPTR);
		ESPI_WR(0x0, ESPI_CH2_TX_DESC_WPTR);
		ESPI_WR(OOB_DMA_DESC_NUM, ESPI_CH2_TX_DESC_EPTR);

		ESPI_WR(oob->dma.rxd_addr >> 32, ESPI_CH2_RX_DMAH);
		ESPI_WR(oob->dma.rxd_addr & 0xffffffff, ESPI_CH2_RX_DMAL);
		ESPI_WR(OOB_DMA_RPTR_KEY, ESPI_CH2_RX_DESC_RPTR);
		ESPI_WR(0x0, ESPI_CH2_RX_DESC_WPTR);
		ESPI_WR(OOB_DMA_DESC_NUM, ESPI_CH2_RX_DESC_EPTR);

		reg = ESPI_RD(ESPI_CH2_CTRL)
		      | ESPI_CH2_CTRL_TX_DMA_EN
		      | ESPI_CH2_CTRL_RX_DMA_EN;
		ESPI_WR(reg, ESPI_CH2_CTRL);

		/* activate RX DMA to make OOB_FREE */
		reg = ESPI_RD(ESPI_CH2_RX_DESC_WPTR) | ESPI_CH2_RX_DESC_WPTR_VALID;
		ESPI_WR(reg, ESPI_CH2_RX_DESC_WPTR);
	}

	ESPI_WR(ESPI_CH2_INT_EN_RX_CMPLT, ESPI_CH2_INT_EN);

	reg = ESPI_RD(ESPI_CH2_CTRL) | ESPI_CH2_CTRL_SW_RDY;
	ESPI_WR(reg, ESPI_CH2_CTRL);
}

static void espi_ast2700_oob_init(struct espi_ast2700_oob *oob)
{
	oob->dma.enable = DT_INST_PROP(0, oob_dma_mode);
	oob->dma.txd_virt = oob_tx_desc;
	oob->dma.txd_addr = TO_PHY_ADDR(oob->txd_virt);
	oob->dma.rxd_virt = oob_rx_desc;
	oob->dma.rxd_addr = TO_PHY_ADDR(oob->rxd_virt);
	oob->dma.tx_virt = oob_tx_buf;
	oob->dma.tx_addr = TO_PHY_ADDR(oob->tx_virt);
	oob->dma.rx_virt = oob_rx_buf;
	oob->dma.rx_addr = TO_PHY_ADDR(oob->rx_virt);
	k_sem_init(&oob->tx_lock, 1, 1);
	k_sem_init(&oob->rx_lock, 1, 1);
	k_sem_init(&oob->rx_ready, 0, 1);
}

/* flash channel */
#if DT_INST_PROP(0, flash_dma_mode)
static uint8_t flash_tx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t flash_rx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
#else
static uint8_t flash_tx_buf[0];
static uint8_t flash_rx_buf[0];
#endif

static void espi_ast2700_flash_isr(uint32_t sts, struct espi_ast2700_data *data)
{
	struct espi_ast2700_flash *flash = data->flash;

	if (sts & ESPI_CH3_INT_STS_RX_CMPLT) {
		ESPI_WR(ESPI_CH3_INT_STS_RX_CMPLT, ESPI_INT_STS);
		k_sem_give(&flash->rx_ready);
	}
}

static void espi_ast2700_flash_reset(struct espi_ast2700_flash *flash)
{
	uint32_t reg;
	uint64_t mask;

	ESPI_WR(0x0, ESPI_CH3_INT_EN);
	ESPI_WR(0xffffffff, ESPI_CH3_INT_STS);

	reg = ESPI_RD(ESPI_CH3_CTRL);
	reg &= ~(ESPI_CH3_CTRL_TX_RST
		 | ESPI_CH3_CTRL_RX_RST
		 | ESPI_CH3_CTRL_TX_DMA_EN
		 | ESPI_CH3_CTRL_RX_DMA_EN
		 | ESPI_CH3_CTRL_SW_RDY);
	ESPI_WR(reg, ESPI_CH3_CTRL);

	reg |= (ESPI_CH3_CTRL_TX_RST | ESPI_CH3_CTRL_RX_RST);
	ESPI_WR(reg, ESPI_CH3_CTRL);

	if (flash->edaf.mode == EDAF_MODE_MIX) {
		mask = ~(flash->edaf.size - 1);
		ESPI_WR(mask >> 32, ESPI_CH3_EDAF_MASKH);
		ESPI_WR(mask & 0xffffffff, ESPI_CH3_EDAF_MASKL);
		ESPI_WR(flash->edaf.taddr >> 32, ESPI_CH3_EDAF_TADDRH);
		ESPI_WR(flash->edaf.taddr & 0xffffffff, ESPI_CH3_EDAF_TADDRL);
	}

	reg = ESPI_RD(ESPI_CH3_CTRL) & ~ESPI_CH3_CTRL_EDAF_MODE;
	reg |= FIELD_PREP(ESPI_CH3_CTRL_EDAF_MODE, flash->edaf.mode);
	ESPI_WR(reg, ESPI_CH3_CTRL);

	if (flash->dma.enable) {
		ESPI_WR(flash->dma.tx_addr >> 32, ESPI_CH3_TX_DMAH);
		ESPI_WR(flash->dma.tx_addr & 0xffffffff, ESPI_CH3_TX_DMAL);
		ESPI_WR(flash->dma.rx_addr >> 32, ESPI_CH3_RX_DMAH);
		ESPI_WR(flash->dma.rx_addr & 0xffffffff, ESPI_CH3_RX_DMAL);

		reg = ESPI_RD(ESPI_CH3_CTRL)
		      | ESPI_CH3_CTRL_TX_DMA_EN
		      | ESPI_CH3_CTRL_RX_DMA_EN;
		ESPI_WR(reg, ESPI_CH3_CTRL);
	}

	ESPI_WR(ESPI_CH3_INT_EN_RX_CMPLT, ESPI_CH3_INT_EN);

	reg = ESPI_RD(ESPI_CH3_CTRL) | ESPI_CH3_CTRL_SW_RDY;
	ESPI_WR(reg, ESPI_CH3_CTRL);
}

static void espi_ast2700_flash_init(struct espi_ast2700_flash *flash)
{
	flash->dma_mode = DT_INST_PROP(0, flash_dma_mode);
	flash->edaf_mode = DT_INST_PROP_OR(0, flash_edaf_mode, 2);
	flash->tx_virt = flash_tx_buf;
	flash->tx_addr = TO_PHY_ADDR(flash->tx_virt);
	flash->rx_virt = flash_rx_buf;
	flash->rx_addr = TO_PHY_ADDR(flash->rx_virt);
	k_sem_init(&flash->tx_lock, 1, 1);
	k_sem_init(&flash->rx_lock, 1, 1);
	k_sem_init(&flash->rx_ready, 0, 1);
}

/* eSPI controller config. */
struct espi_ast2700_config {
	uintptr_t base;
};

static const struct espi_ast2700_config espi_ast2700_config = {
	.base = DT_INST_REG_ADDR(0),
};

static void espi_ast2700_isr(const struct device *dev)
{
	uint32_t sts;
	uint32_t sysevt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;

	sts = ESPI_RD(ESPI_INT_STS);

	if (sts & ESPI_INT_STS_CH0)
		espi_ast2700_perif_isr(data);

	if (sts & ESPI_INT_STS_CH1)
		espi_ast2700_vw_isr(data);

	if (sts & ESPI_INT_STS_CH2)
		espi_ast2700_oob_isr(data);

	if (sts & ESPI_INT_STS_CH3)
		espi_ast2700_flash_isr(data);

	if (sts & ESPI_INT_STS_HW_RST_DEASSERT) {
		espi_ast2700_perif_reset(data);
		espi_ast2700_vw_reset(data);
		espi_ast2700_oob_reset(data);
		espi_ast2700_flash_reset(data);
		ESPI_WR(ESPI_INT_STS_RST_DEASSERT, ESPI_INT_STS);
	}

	/* dummy read to make sure W1C arrives HW */
	sts = ESPI_RD(ESPI_INT_STS);
}

static int espi_ast2700_init(const struct device *dev)
{
	uint32_t reg, scu_base;
	struct espi_ast2700_config *cfg = (struct espi_ast2700_config *)dev->config;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;

	data->dev = dev;

	espi_base = cfg->base;

	reg = ESPI_RD(ESPI_INT_EN);
	reg &= ~ESPI_INT_EN_RST_DEASSERT;
	ESPI_WR(reg, ESPI_INT_EN);

	espi_ast2700_perif_init(&data->perif);
	espi_ast2700_vw_init(&data->vw);
	espi_ast2700_oob_init(&data->oob);
	espi_ast2700_flash_init(&data->flash);

	espi_ast2700_perif_reset(perif);
	espi_ast2700_vw_reset(vw);
	espi_ast2700_oob_reset(oob);
	espi_ast2700_flash_reset(flash);

	/* install interrupt handler */
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    espi_ast2700_isr,
		    DEVICE_DT_INST_GET(0), 0);

	/* enable eSPI interrupt */
	irq_enable(DT_INST_IRQN(0));

	reg = ESPI_RD(ESPI_INT_EN) | ESPI_INT_EN_RST_DEASSERT;
	ESPI_WR(reg, ESPI_INT_EN);

	return 0;
}

/* eSPI ASPEED proprietary APIs for raw packet TX/RX */
int espi_aspeed_perif_pc_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc,
				bool blocking)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_perif *perif = data->perif;

	rc = k_sem_take(&perif->rx_lock, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		return rc;

	rc = k_sem_take(&perif->rx_ready, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		goto unlock_n_out;

	reg = ESPI_RD(ESPI_CH0_PC_RX_CTRL);
	cyc = FIELD_GET(ESPI_CH0_PC_RX_CTRL_CYC, reg);
	tag = FIELD_GET(ESPI_CH0_PC_RX_CTRL_TAG, reg);
	len = FIELD_GET(ESPI_CH0_PC_RX_CTRL_LEN, reg);

	switch (cyc) {
	case ESPI_PERIF_MSG:
		ioc->pkt_len = len + sizeof(struct espi_perif_msg);
		break;
	case ESPI_PERIF_MSG_D:
		ioc->pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) +
			sizeof(struct espi_perif_msg);
		break;
	case ESPI_PERIF_SUC_CMPLT_D_MIDDLE:
	case ESPI_PERIF_SUC_CMPLT_D_FIRST:
	case ESPI_PERIF_SUC_CMPLT_D_LAST:
	case ESPI_PERIF_SUC_CMPLT_D_ONLY:
		ioc->pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) +
			sizeof(struct espi_perif_cmplt);
		break;
	case ESPI_PERIF_SUC_CMPLT:
	case ESPI_PERIF_UNSUC_CMPLT:
		ioc->pkt_len = len + sizeof(struct espi_perif_cmplt);
		break;
	default:
		__ASSERT(0, "Unrecognized eSPI peripheral packet");

		k_sem_give(&perif->rx_ready);
		rc = -EFAULT;
		goto unlock_n_out;
	}

	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	if (perif->dma.enable)
		memcpy(hdr + 1, perif->pc_rx_virt, ioc->pkt_len - sizeof(*hdr));
	else
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			ioc->pkt[i] = (ESPI_RD(ESPI_CH0_PC_RX_DATA) & 0xff);

	ESPI_WR(reg | ESPI_CH0_PC_RX_CTRL_SERV_PEND, ESPI_CH0_PC_RX_CTRL);

unlock_n_out:
	k_sem_give(&perif->rx_lock);

	return rc;
}

int espi_aspeed_perif_pc_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_perif *perif = data->perif;

	rc = k_sem_take(&perif->pc_tx_lock, K_NO_WAIT);
	if (rc)
		return rc;

	reg = ESPI_RD(ESPI_CH0_PC_TX_CTRL);
	if (reg & ESPI_CH0_PC_TX_CTRL_TRIGGER) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	if (perif->dma_mode)
		memcpy(perif->pc_tx_virt, hdr + 1, ioc->pkt_len - sizeof(*hdr));
	else
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			ESPI_WR(ioc->pkt[i], ESPI_CH0_PC_TX_DATA);

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = FIELD_PREP(ESPI_CH0_PC_TX_CTRL_CYC, cyc)
	      | FIELD_PREP(ESPI_CH0_PC_TX_CTRL_TAG, tag)
	      | FIELD_PREP(ESPI_CH0_PC_TX_CTRL_LEN, len)
	      | ESPI_CH0_PC_TX_CTRL_TRIG_PEND;
	ESPI_WR(reg, ESPI_CH0_PC_TX_CTRL);

	rc = 0;

unlock_n_out:
	k_sem_give(&perif->pc_tx_lock);

	return rc;
}

int espi_aspeed_perif_np_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_perif *perif = data->perif;

	rc = k_sem_take(&perif->np_tx_lock, K_NO_WAIT);
	if (rc)
		return rc;

	reg = ESPI_RD(ESPI_CH0_NP_TX_CTRL);
	if (reg & ESPI_CH0_NP_TX_CTRL_TRIG_PEND) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	if (perif->dma.enable)
		memcpy(perif->np_tx_virt, hdr + 1, ioc->pkt_len - sizeof(*hdr));
	else
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			ESPI_WR(ioc->pkt[i], ESPI_CH0_NP_TX_DATA);

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = FIELD_PREP(ESPI_CH0_NP_TX_CTRL_CYC, cyc)
	      | FIELD_PREP(ESPI_CH0_NP_TX_CTRL_TAG, tag)
	      | FIELD_PREP(ESPI_CH0_NP_TX_CTRL_LEN, len)
	      | ESPI_CH0_NP_TX_CTRL_TRIG_PEND;
	ESPI_WR(reg, ESPI_CH0_NP_TX_CTRL);

	rc = 0;

unlock_n_out:
	k_sem_give(&perif->np_tx_lock);

	return rc;
}

int espi_aspeed_oob_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc, bool blocking)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint32_t wptr;
	struct oob_rx_dma_desc *d;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_oob *oob = data->oob;

	rc = k_sem_take(&oob->rx_lock, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		return rc;

	rc = k_sem_take(&oob->rx_ready, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		goto unlock_n_out;

	if (oob->dma.enable) {
		wptr = ESPI_RD(ESPI_CH2_RX_DESC_WPTR);

		d = &oob->rxd_virt[sptr];

		ioc->pkt_len = (d->len) ? d->len : ESPI_PLD_LEN_MAX;
		ioc->pkt_len += sizeof(struct espi_comm_hdr);

		hdr->cyc = d->cyc;
		hdr->tag = d->tag;
		hdr->len_h = d->len >> 8;
		hdr->len_l = d->len & 0xff;
		memcpy(hdr + 1, oob->dma.rx_virt + (ESPI_PLD_LEN_MAX * wptr),
		       ioc->pkt_len - sizeof(*hdr));

		d->dirty = 0;

		wptr = (wptr + 1) % OOB_RX_DMA_DESC_NUM;
		ESPI_WR(wptr | ESPI_CH2_RX_DESC_WPTR_VALID, ESPI_CH2_RX_DESC_WPTR);

		if (oob->rx_desc[sptr].dirty)
			k_sem_give(&oob->rx_ready);

	} else {
		reg = ESPI_RD(ESPI_OOB_RX_CTRL);
		cyc = FIELD_GET(ESPI_CH2_RX_CTRL_CYC, reg);
		tag = FIELD_GET(ESPI_CH2_RX_CTRL_TAG, reg);
		len = FIELD_GET(ESPI_CH2_RX_CTRL_LEN, reg);

		ioc->pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) + sizeof(struct espi_comm_hdr);

		hdr->cyc = cyc;
		hdr->tag = tag;
		hdr->len_h = len >> 8;
		hdr->len_l = len & 0xff;

		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			ioc->pkt[i] = ESPI_RD(ESPI_CH2_RX_DATA) & 0xff;

		ESPI_WR(reg | ESPI_CH2_RX_CTRL_SERV_PEND, ESPI_CH2_RX_CTRL);
	}

	rc = 0;

unlock_n_out:
	k_sem_give(&oob->rx_lock);

	return rc;
}

int espi_aspeed_oob_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint32_t rptr, wptr;
	struct oob_tx_dma_desc *d;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_oob *oob = data->oob;

	rc = k_sem_take(&oob->tx_lock, K_NO_WAIT);
	if (rc)
		return rc;

	if (oob->dma.enable) {
		ESPI_WR(ESPI_CH2_TX_DESC_RPTR_UPT, ESPI_CH2_TX_DESC_RPTR);

		rptr = ESPI_RD(ESPI_CH2_TX_DESC_RPTR);
		wptr = ESPI_RD(ESPI_CH2_TX_DESC_WPTR);

		if (((wptr + 1) % OOB_DMA_DESC_NUM) == rptr) {
			rc = -EBUSY;
			goto unlock_n_out;
		}

		d = &oob->tx_desc[wptr];
		d->cyc = hdr->cyc;
		d->tag = hdr->tag;
		d->len = (hdr->len_h << 8) | (hdr->len_l & 0xff);
		d->msg_type = OOB_DMA_DESC_CUSTOM;

		memcpy(oob->tx_virt + (ESPI_PLD_LEN_MAX * wptr), hdr + 1,
		       ioc->pkt_len - sizeof(*hdr));

		wptr = (wptr + 1) % OOB_TX_DMA_DESC_NUM;
		ESPI_WR(wptr | ESPI_CH2_TX_DESC_WPTR_VALID, ESPI_CH2_TX_DESC_WPTR);
	} else {
		reg = ESPI_RD(ESPI_CH2_TX_CTRL);
		if (reg & ESPI_CH2_TX_CTRL_TRIG_PEND) {
			rc = -EBUSY;
			goto unlock_n_out;
		}

		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			ESPI_WR(ioc->pkt[i], ESPI_CH2_TX_DATA);

		cyc = hdr->cyc;
		tag = hdr->tag;
		len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

		reg = FIELD_PREP(ESPI_CH2_TX_CTRL_CYC, cyc)
			  | FIELD_PREP(ESPI_CH2_TX_CTRL_TAG, tag)
			  | FIELD_PREP(ESPI_CH2_TX_CTRL_LEN, len)
			  | ESPI_CH2_TX_CTRL_TRIG_PEND;
		ESPI_WR(reg, ESPI_CH2_TX_CTRL);
	}

	rc = 0;

unlock_n_out:
	k_sem_give(&oob->tx_lock);

	return rc;

}

int espi_aspeed_flash_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc, bool blocking)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_flash *flash = data->flash;

	rc = k_sem_take(&flash->rx_lock, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		return rc;

	rc = k_sem_take(&flash->rx_ready, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		goto unlock_n_out;

	reg = ESPI_RD(ESPI_CH3_RX_CTRL);
	cyc = FIELD_GET(ESPI_CH3_RX_CTRL_CYC, reg);
	tag = FIELD_GET(ESPI_CH3_RX_CTRL_TAG, reg);
	len = FIELD_GET(ESPI_CH3_RX_CTRL_LEN, reg);

	switch (cyc) {
	case ESPI_FLASH_READ:
	case ESPI_FLASH_WRITE:
	case ESPI_FLASH_ERASE:
		ioc->pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) + sizeof(struct espi_flash_rwe);
		break;
	case ESPI_FLASH_SUC_CMPLT_D_MIDDLE:
	case ESPI_FLASH_SUC_CMPLT_D_FIRST:
	case ESPI_FLASH_SUC_CMPLT_D_LAST:
	case ESPI_FLASH_SUC_CMPLT_D_ONLY:
		ioc->pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) + sizeof(struct espi_flash_cmplt);
		break;
	case ESPI_FLASH_SUC_CMPLT:
	case ESPI_FLASH_UNSUC_CMPLT:
		ioc->pkt_len = len + sizeof(struct espi_flash_cmplt);
		break;
	default:
		__ASSERT(0, "Unrecognized eSPI flash packet");

		k_sem_give(&flash->rx_ready);
		rc = -EFAULT;
		goto unlock_n_out;
	}

	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	if (flash->dma.enable)
		memcpy(hdr + 1, flash->rx_virt, ioc->pkt_len - sizeof(*hdr));
	else
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			ioc->pkt[i] = ESPI_RD(ESPI_CH3_RX_DATA) & 0xff;

	ESPI_WR(ESPI_CH3_RX_CTRL_SERV_PEND, ESPI_CH3_RX_CTRL);

unlock_n_out:
	k_sem_give(&flash->rx_lock);

	return rc;
}

int espi_aspeed_flash_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_flash *flash = data->flash;

	rc = k_sem_take(&flash->tx_lock, K_NO_WAIT);
	if (rc)
		return rc;

	reg = ESPI_RD(ESPI_CH3_TX_CTRL);
	if (reg & ESPI_CH3_TX_CTRL_TRIG_PEND) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	if (flash->dma.enable)
		memcpy(flash->tx_virt, hdr + 1, ioc->pkt_len - sizeof(*hdr));
	else
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			ESPI_WR(ioc->pkt[i], ESPI_CH3_TX_DATA);

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = FIELD_PREP(ESPI_CH3_TX_CTRL_CYC, cyc)
	      | FIELD_PREP(ESPI_CH3_TX_CTRL_TAG, tag)
	      | FIELD_PREP(ESPI_CH3_TX_CTRL_LEN, len)
	      | ESPI_CH3_TX_CTRL_TRIG_PEND;
	ESPI_WR(reg, ESPI_CH3_TX_CTRL);

	rc = 0;

unlock_n_out:
	k_sem_give(&flash->tx_lock);

	return rc;
}

/* eSPI standard callback */
static bool espi_ast2700_channel_ready(const struct device *dev, enum espi_channel ch)
{
	switch (ch) {
	case ESPI_CHANNEL_PERIPHERAL:
		return ESPI_RD(ESPI_CTRL) & ESPI_CTRL_PERIF_SW_RDY;
	case ESPI_CHANNEL_VWIRE:
		return ESPI_RD(ESPI_CTRL) & ESPI_CTRL_VW_SW_RDY;
	case ESPI_CHANNEL_OOB:
		return ESPI_RD(ESPI_CTRL) & ESPI_CTRL_OOB_SW_RDY;
	case ESPI_CHANNEL_FLASH:
		return ESPI_RD(ESPI_CTRL) & ESPI_CTRL_FLASH_SW_RDY;
	default:
		return false;
	}

	return false;
}

static int espi_ast2700_send_oob(const struct device *dev, struct espi_oob_packet *pckt)
{
	struct espi_oob_msg *oob_msg;
	struct espi_ast2700_ioc ioc;
	uint8_t pkt[sizeof(*oob_msg) + ESPI_PLD_LEN_MAX];

	ioc.pkt = pkt;
	ioc.pkt_len = sizeof(*oob_msg) + pckt->len;

	oob_msg = (struct espi_oob_msg *)pkt;
	oob_msg->cyc = OOB_MSG;
	oob_msg->tag = 0;
	oob_msg->len_h = pckt->len >> 8;
	oob_msg->len_l = pckt->len & 0xff;

	memcpy(oob_msg + 1, pckt->buf, pckt->len);

	return espi_ast2700_oob_put_tx(dev, &ioc);
}

static int espi_ast2700_receive_oob(const struct device *dev, struct espi_oob_packet *pckt)
{
	int rc;
	struct espi_oob_msg *oob_msg;
	struct espi_ast2700_ioc ioc;
	uint8_t pkt[sizeof(*oob_msg) + ESPI_PLD_LEN_MAX];

	ioc.pkt = pkt;
	ioc.pkt_len = sizeof(pkt);

	rc = espi_ast2700_oob_get_rx(dev, &ioc, false);
	if (rc)
		return rc;

	oob_msg = (struct espi_oob_msg *)ioc.pkt;

	pckt->len = (oob_msg->len_h << 8) | (oob_msg->len_l & 0xff);
	memcpy(pckt->buf, oob_msg + 1, pckt->len);

	return 0;
}

static int espi_ast2700_flash_rwe(const struct device *dev, struct espi_flash_packet *pckt,
				 uint32_t flash_op)
{
	struct espi_flash_rwe *flash_rwe;
	struct espi_ast2700_ioc ioc;
	uint8_t pkt[sizeof(*flash_rwe) + ESPI_PLD_LEN_MAX];

	ioc.pkt = pkt;
	ioc.pkt_len = sizeof(*flash_rwe) + pckt->len;

	flash_rwe = (struct espi_flash_rwe *)pkt;
	flash_rwe->cyc = flash_op;
	flash_rwe->tag = FLASH_TAG;
	flash_rwe->len_h = pckt->len >> 8;
	flash_rwe->len_l = pckt->len & 0xff;
	flash_rwe->addr_be = __bswap_32(pckt->flash_addr);

	memcpy(flash_rwe + 1, pckt->buf, pckt->len);

	return espi_ast2700_flash_put_tx(dev, &ioc);
}

static int espi_ast2700_flash_read(const struct device *dev, struct espi_flash_packet *pckt)
{
	return espi_ast2700_flash_rwe(dev, pckt, FLASH_READ);
}

static int espi_ast2700_flash_write(const struct device *dev, struct espi_flash_packet *pckt)
{
	return espi_ast2700_flash_rwe(dev, pckt, FLASH_WRITE);
}

static int espi_ast2700_flash_erase(const struct device *dev, struct espi_flash_packet *pckt)
{
	return espi_ast2700_flash_rwe(dev, pckt, FLASH_ERASE);
}

static int espi_ast2700_manage_callback(const struct device *dev,
				       struct espi_callback *callback,
				       bool set)
{
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;

	return espi_manage_callback(&data->callbacks, callback, set);
}

static const struct espi_driver_api espi_ast2700_driver_api = {
	.get_channel_status = espi_ast2700_channel_ready,
	.send_oob = espi_ast2700_send_oob,
	.receive_oob = espi_ast2700_receive_oob,
	.flash_read = espi_ast2700_flash_read,
	.flash_write = espi_ast2700_flash_write,
	.flash_erase = espi_ast2700_flash_erase,
	.manage_callback = espi_ast2700_manage_callback,
};

DEVICE_DT_INST_DEFINE(0, &espi_ast2700_init, NULL,
		      &espi_ast2700_data, &espi_ast2700_config,
		      PRE_KERNEL_2, CONFIG_ESPI_INIT_PRIORITY,
		      &espi_ast2700_driver_api);
