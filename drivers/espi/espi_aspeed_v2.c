/*
 * Copyright (c) 2023 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_espi_v2

#include <soc.h>
#include <zephyr/cache.h>
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
#define   ESPI_CAP_CH3_0_SHARE_MODE	BIT(11)
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
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
#define   ESPI_CH0_INT_STS_NP_RX_VALID	BIT(2)
#endif
#define   ESPI_CH0_INT_STS_PC_RX_CMPLT	BIT(0)
#define ESPI_CH0_INT_EN			0x10c
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
#define   ESPI_CH0_INT_EN_NP_RX_VALID	BIT(2)
#endif
#define   ESPI_CH0_INT_EN_PC_RX_CMPLT	BIT(0)
#define ESPI_CH0_PC_RX_DMAL		0x110
#define ESPI_CH0_PC_RX_DMAH		0x114
#define ESPI_CH0_PC_RX_CTRL		0x118
#define   ESPI_CH0_PC_RX_CTRL_SERV_PEND	BIT(31)
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
#define   ESPI_CH0_PC_RX_CTRL_FW	BIT(24)
#endif
#define   ESPI_CH0_PC_RX_CTRL_LEN	GENMASK(23, 12)
#define   ESPI_CH0_PC_RX_CTRL_TAG	GENMASK(11, 8)
#define   ESPI_CH0_PC_RX_CTRL_CYC	GENMASK(7, 0)
#define ESPI_CH0_PC_RX_DATA		0x11c
#define ESPI_CH0_PC_TX_DMAL		0x120
#define ESPI_CH0_PC_TX_DMAH		0x124
#define ESPI_CH0_PC_TX_CTRL		0x128
#define   ESPI_CH0_PC_TX_CTRL_TRIG_PEND	BIT(31)
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
#define   ESPI_CH0_PC_TX_CTRL_FW	BIT(24)
#endif
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
#define ESPI_CH0_MCYC0_SADDRL	0x140
#define ESPI_CH0_MCYC0_SADDRH	0x144
#define ESPI_CH0_MCYC0_TADDRL	0x148
#define ESPI_CH0_MCYC0_TADDRH	0x14c
#define ESPI_CH0_MCYC0_MASKL	0x150
#define   ESPI_CH0_MCYC0_MASKL_FW	BIT(1)
#define   ESPI_CH0_MCYC0_MASKL_EN	BIT(0)
#define ESPI_CH0_MCYC0_MASKH	0x154
#define ESPI_CH0_MCYC1_SADDRL	0x158
#define ESPI_CH0_MCYC1_SADDRH	0x15c
#define ESPI_CH0_MCYC1_TADDRL	0x160
#define ESPI_CH0_MCYC1_TADDRH	0x164
#define ESPI_CH0_MCYC1_MASKL	0x168
#define   ESPI_CH0_MCYC1_MASKL_FW	BIT(1)
#define   ESPI_CH0_MCYC1_MASKL_EN	BIT(0)
#define ESPI_CH0_MCYC1_MASKH	0x16c
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
#define ESPI_CH0_MCYC2_SADDRL	0x170
#define ESPI_CH0_MCYC2_SADDRH	0x174
#define ESPI_CH0_MCYC2_TADDRL	0x178
#define ESPI_CH0_MCYC2_TADDRH	0x17c
#define ESPI_CH0_MCYC2_MASKL	0x180
#define   ESPI_CH0_MCYC2_MASKL_FW	BIT(1)
#define   ESPI_CH0_MCYC2_MASKL_EN	BIT(0)
#define ESPI_CH0_MCYC2_MASKH	0x184
#define ESPI_CH0_PC_RX_ADDRL	0x1c4
#define ESPI_CH0_PC_RX_ADDRH	0x1c8
#define ESPI_CH0_NP_RX_CTRL		0x1d0
#define   ESPI_CH0_NP_RX_CTRL_SERV_PEND	BIT(31)
#define   ESPI_CH0_NP_RX_CTRL_LEN		GENMASK(23, 12)
#define   ESPI_CH0_NP_RX_CTRL_TAG		GENMASK(11, 8)
#define   ESPI_CH0_NP_RX_MEM64_RD	BIT(3)
#define   ESPI_CH0_NP_RX_MEM32_RD	BIT(2)
#define   ESPI_CH0_NP_RX_IO_WR		BIT(1)
#define   ESPI_CH0_NP_RX_IO_RD		BIT(0)
#define ESPI_CH0_NP_RX_ADDRL	0x1d4
#define ESPI_CH0_NP_RX_ADDRH	0x1d8
#endif
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
#define   ESPI_CH3_INT_STS_TX_CMPLT	BIT(1)
#define   ESPI_CH3_INT_STS_RX_CMPLT	BIT(0)
#define ESPI_CH3_INT_EN			0x40c
#define   ESPI_CH3_INT_EN_TX_CMPLT	BIT(1)
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

#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
#define SCU_OTP_STRAP_3		0x050
#define   SCU_OTP_STRAP_TAF_EN	BIT(19)
#define SCU_DBG_DIS_CFG		0x0c8
#define   SCU_DIS_ESPI_AHB		BIT(0)
#endif

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
#define FLASH_TAG		0xa

#define EDAF_MODE_MIX	0
#define EDAF_MODE_SW	1
#define EDAF_MODE_HW	2

/* driver data structure */
struct espi_ast2700_perif {
	struct {
		bool enable;
		bool dma_mode;
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
	struct k_sem pc_rx_lock;
	struct k_sem pc_rx_ready;
	struct k_sem np_rx_lock;
	struct k_sem np_rx_ready;
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
} __packed __aligned(CONFIG_DCACHE_LINE_SIZE);

struct espi_ast2700_oob_dma_rx_desc {
	uint32_t data_addrl;
	uint32_t data_addrh;
	uint8_t cyc;
	uint16_t tag : 4;
	uint16_t len : 12;
	uint8_t raz : 7;
	uint8_t dirty : 1;
	uint32_t pad[1];
} __packed __aligned(CONFIG_DCACHE_LINE_SIZE);

struct espi_ast2700_oob {
	struct {
		bool enable;
		struct espi_ast2700_oob_dma_tx_desc *txd_virt;
		uint64_t txd_addr;
		struct espi_ast2700_oob_dma_rx_desc *rxd_virt;
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
	struct espi_ast2700_perif perif;
	struct espi_ast2700_vw vw;
	struct espi_ast2700_oob oob;
	struct espi_ast2700_flash flash;
	sys_slist_t callbacks;
};

static uint32_t espi_base;
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
static uint32_t scu_base;
#endif
static struct espi_ast2700_data espi_ast2700_data;

/* peripheral channel */
#if DT_INST_PROP(0, perif_dma_mode)
static uint8_t perif_pc_rx_buf[ESPI_PLD_LEN_MAX] __aligned(CONFIG_DCACHE_LINE_SIZE);
static uint8_t perif_pc_tx_buf[ESPI_PLD_LEN_MAX] __aligned(CONFIG_DCACHE_LINE_SIZE);
static uint8_t perif_np_tx_buf[ESPI_PLD_LEN_MAX] __aligned(CONFIG_DCACHE_LINE_SIZE);
#else
static uint8_t perif_pc_rx_buf[0];
static uint8_t perif_pc_tx_buf[0];
static uint8_t perif_np_tx_buf[0];
#endif

static void espi_aspeed_dma_cache_range(void *addr, size_t len,
					int (*cache_op)(void *addr, size_t size))
{
	size_t line_size;
	uintptr_t start, end;

	if (!addr || !len)
		return;

	line_size = sys_cache_data_line_size_get();
	if (!line_size) {
		(void)cache_op(addr, len);
		return;
	}

	start = ROUND_DOWN((uintptr_t)addr, line_size);
	end = ROUND_UP((uintptr_t)addr + len, line_size);

	(void)cache_op((void *)start, end - start);
}

#if DT_INST_PROP(0, perif_mcyc_enable) && DT_INST_PROP(0, perif_mcyc_dma_mode)
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
	LOG_INF("perif int sts: 0x%08x", sts);

#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
	if (sts & ESPI_CH0_INT_STS_NP_RX_VALID) {
		ESPI_WR(ESPI_CH0_INT_STS_NP_RX_VALID, ESPI_CH0_INT_STS);
		k_sem_give(&perif->np_rx_ready);
	}
#endif

	if (sts & ESPI_CH0_INT_STS_PC_RX_CMPLT) {
		ESPI_WR(ESPI_CH0_INT_STS_PC_RX_CMPLT, ESPI_CH0_INT_STS);
		k_sem_give(&perif->pc_rx_ready);
	}
}

static void espi_ast2700_perif_reset(struct espi_ast2700_perif *perif)
{
	uint32_t reg;
	uint64_t mask;

	ESPI_WR(0x0, ESPI_CH0_INT_EN);
	ESPI_WR(0xffffffff, ESPI_CH0_INT_STS);

	reg = ESPI_RD(ESPI_CH0_MCYC1_MASKL);
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
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
		reg = sys_read32(scu_base + SCU_DBG_DIS_CFG);
		reg &= ~SCU_DIS_ESPI_AHB;
		sys_write32(reg, scu_base + SCU_DBG_DIS_CFG);
#endif
		mask = ~(perif->mcyc.mcyc_size - 1);
		ESPI_WR(0xffffffff, ESPI_CH0_MCYC1_MASKH);
		ESPI_WR(mask & 0xffffffff, ESPI_CH0_MCYC1_MASKL);
		ESPI_WR((perif->mcyc.saddr >> 32), ESPI_CH0_MCYC1_SADDRH);
		ESPI_WR((perif->mcyc.saddr & 0xffffffff), ESPI_CH0_MCYC1_SADDRL);
		ESPI_WR((perif->mcyc.taddr >> 32), ESPI_CH0_MCYC1_TADDRH);
		ESPI_WR((perif->mcyc.taddr & 0xffffffff), ESPI_CH0_MCYC1_TADDRL);

		reg = ESPI_RD(ESPI_CH0_MCYC1_MASKL) | ESPI_CH0_MCYC1_MASKL_EN;
		ESPI_WR(reg, ESPI_CH0_MCYC1_MASKL);

		reg = ESPI_RD(ESPI_CH0_CTRL);
		reg &= ~(ESPI_CH0_CTRL_MCYC_RD_DIS | ESPI_CH0_CTRL_MCYC_WR_DIS);

		ESPI_WR(reg, ESPI_CH0_CTRL);

#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
		if (!perif->mcyc.dma_mode) {
			reg = ESPI_RD(ESPI_CH0_MCYC1_MASKL) | ESPI_CH0_MCYC1_MASKL_FW;
			ESPI_WR(reg, ESPI_CH0_MCYC1_MASKL);
		}
#endif
	}

	if (perif->dma.enable) {
		ESPI_WR((perif->dma.np_tx_addr >> 32), ESPI_CH0_NP_TX_DMAH);
		ESPI_WR((perif->dma.np_tx_addr & 0xffffffff), ESPI_CH0_NP_TX_DMAL);
		ESPI_WR((perif->dma.pc_tx_addr >> 32), ESPI_CH0_PC_TX_DMAH);
		ESPI_WR((perif->dma.pc_tx_addr & 0xffffffff), ESPI_CH0_PC_TX_DMAL);
		ESPI_WR((perif->dma.pc_rx_addr >> 32), ESPI_CH0_PC_RX_DMAH);
		ESPI_WR((perif->dma.pc_rx_addr & 0xffffffff), ESPI_CH0_PC_RX_DMAL);

		reg = ESPI_RD(ESPI_CH0_CTRL)
		      | ESPI_CH0_CTRL_NP_TX_DMA_EN
		      | ESPI_CH0_CTRL_PC_TX_DMA_EN
		      | ESPI_CH0_CTRL_PC_RX_DMA_EN;
		ESPI_WR(reg, ESPI_CH0_CTRL);
	}

	ESPI_WR(ESPI_CH0_INT_EN_PC_RX_CMPLT, ESPI_CH0_INT_EN);

#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
	if (perif->mcyc.enable && !perif->mcyc.dma_mode) {
		reg = ESPI_RD(ESPI_CH0_INT_EN) | ESPI_CH0_INT_EN_NP_RX_VALID;
		ESPI_WR(reg, ESPI_CH0_INT_EN);
	}
#endif

	reg = ESPI_RD(ESPI_CH0_CTRL) | ESPI_CH0_CTRL_SW_RDY;
	ESPI_WR(reg, ESPI_CH0_CTRL);
}

static void espi_ast2700_perif_init(struct espi_ast2700_perif *perif)
{
	perif->dma.enable = DT_INST_PROP(0, perif_dma_mode);
	perif->dma.pc_rx_virt = perif_pc_rx_buf;
	perif->dma.pc_rx_addr = TO_PHY_ADDR((uintptr_t)perif->dma.pc_rx_virt);
	perif->dma.pc_tx_virt = perif_pc_tx_buf;
	perif->dma.pc_tx_addr = TO_PHY_ADDR((uintptr_t)perif->dma.pc_tx_virt);
	perif->dma.np_tx_virt = perif_np_tx_buf;
	perif->dma.np_tx_addr = TO_PHY_ADDR((uintptr_t)perif->dma.np_tx_virt);

	perif->mcyc.enable = DT_INST_PROP(0, perif_mcyc_enable);
	perif->mcyc.dma_mode = DT_INST_PROP(0, perif_mcyc_dma_mode);
	perif->mcyc.virt = perif_mcyc_buf;
	perif->mcyc.mcyc_size = COND_CODE_1(DT_INST_NODE_HAS_PROP(0, perif_mcyc_size),
		(((uint64_t)DT_INST_PROP_BY_IDX(0, perif_mcyc_size, 0) << 32) |
		 DT_INST_PROP_BY_IDX(0, perif_mcyc_size, 1)), (0));
	perif->mcyc.saddr = COND_CODE_1(DT_INST_NODE_HAS_PROP(0, perif_mcyc_src_addr),
		(((uint64_t)DT_INST_PROP_BY_IDX(0, perif_mcyc_src_addr, 0) << 32) |
		 DT_INST_PROP_BY_IDX(0, perif_mcyc_src_addr, 1)), (0));
	perif->mcyc.taddr = TO_PHY_ADDR((uintptr_t)perif->mcyc.virt);

	k_sem_init(&perif->pc_tx_lock, 1, 1);
	k_sem_init(&perif->np_tx_lock, 1, 1);
	k_sem_init(&perif->pc_rx_lock, 1, 1);
	k_sem_init(&perif->pc_rx_ready, 0, 1);
	k_sem_init(&perif->np_rx_lock, 1, 1);
	k_sem_init(&perif->np_rx_ready, 0, 1);
}

/* virtual wire channel */
static void espi_ast2700_vw_isr(struct espi_ast2700_data *data)
{
	struct espi_event evt_vw = { ESPI_BUS_EVENT_VWIRE_RECEIVED, 0, 0 };
	struct espi_ast2700_vw *vw = &data->vw;
	uint32_t sts;

	espi_send_callbacks(&data->callbacks, data->dev, evt_vw);

	sts = ESPI_RD(ESPI_CH1_INT_STS);

	if (sts & ESPI_CH1_INT_STS_GPIO) {
		vw->gpio.val0 = ESPI_RD(ESPI_CH1_GPIO_VAL0);
		vw->gpio.val1 = ESPI_RD(ESPI_CH1_GPIO_VAL1);
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
static struct espi_ast2700_oob_dma_tx_desc oob_tx_desc[OOB_DMA_DESC_NUM]
	__aligned(CONFIG_DCACHE_LINE_SIZE);
static struct espi_ast2700_oob_dma_rx_desc oob_rx_desc[OOB_DMA_DESC_NUM]
	__aligned(CONFIG_DCACHE_LINE_SIZE);
static uint8_t oob_tx_buf[OOB_DMA_BUF_SIZE] __aligned(CONFIG_DCACHE_LINE_SIZE);
static uint8_t oob_rx_buf[OOB_DMA_BUF_SIZE] __aligned(CONFIG_DCACHE_LINE_SIZE);
#else
static struct espi_ast2700_oob_dma_tx_desc oob_tx_desc[0];
static struct espi_ast2700_oob_dma_rx_desc oob_rx_desc[0];
static uint8_t oob_tx_buf[0];
static uint8_t oob_rx_buf[0];
#endif

static void espi_ast2700_oob_isr(struct espi_ast2700_data *data)
{
	uint32_t sts;

	sts = ESPI_RD(ESPI_CH2_INT_STS);

	if (sts & ESPI_CH2_INT_STS_RX_CMPLT) {
		ESPI_WR(ESPI_CH2_INT_STS_RX_CMPLT, ESPI_CH2_INT_STS);
#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
		struct espi_event evt = {
			.evt_type = ESPI_BUS_EVENT_OOB_RECEIVED,
			.evt_details = 0,
			.evt_data = 0,
		};
		espi_send_callbacks(&data->callbacks, data->dev, evt);
#else
		k_sem_give(&data->oob.rx_ready);
#endif
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

		espi_aspeed_dma_cache_range(oob->dma.txd_virt,
					   sizeof(oob->dma.txd_virt[0]) * OOB_DMA_DESC_NUM,
					   sys_cache_data_flush_range);
		espi_aspeed_dma_cache_range(oob->dma.rxd_virt,
					   sizeof(oob->dma.rxd_virt[0]) * OOB_DMA_DESC_NUM,
					   sys_cache_data_flush_range);

#if ARM64
		ESPI_WR(oob->dma.txd_addr >> 32, ESPI_CH2_TX_DMAH);
#endif
		ESPI_WR(oob->dma.txd_addr & 0xffffffff, ESPI_CH2_TX_DMAL);
		ESPI_WR(OOB_DMA_RPTR_KEY, ESPI_CH2_TX_DESC_RPTR);
		ESPI_WR(0x0, ESPI_CH2_TX_DESC_WPTR);
		ESPI_WR(OOB_DMA_DESC_NUM, ESPI_CH2_TX_DESC_EPTR);

#if ARM64
		ESPI_WR(oob->dma.rxd_addr >> 32, ESPI_CH2_RX_DMAH);
#endif
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
	oob->dma.txd_addr = TO_PHY_ADDR((uintptr_t)oob->dma.txd_virt);
	oob->dma.rxd_virt = oob_rx_desc;
	oob->dma.rxd_addr = TO_PHY_ADDR((uintptr_t)oob->dma.rxd_virt);
	oob->dma.tx_virt = oob_tx_buf;
	oob->dma.tx_addr = TO_PHY_ADDR((uintptr_t)oob->dma.tx_virt);
	oob->dma.rx_virt = oob_rx_buf;
	oob->dma.rx_addr = TO_PHY_ADDR((uintptr_t)oob->dma.rx_virt);
	k_sem_init(&oob->tx_lock, 1, 1);
	k_sem_init(&oob->rx_lock, 1, 1);
	k_sem_init(&oob->rx_ready, 0, 1);
}

/* flash channel */
#if DT_INST_PROP(0, flash_dma_mode)
static uint8_t flash_tx_buf[ESPI_PLD_LEN_MAX] __aligned(CONFIG_DCACHE_LINE_SIZE);
static uint8_t flash_rx_buf[ESPI_PLD_LEN_MAX] __aligned(CONFIG_DCACHE_LINE_SIZE);
#else
static uint8_t flash_tx_buf[0];
static uint8_t flash_rx_buf[0];
#endif

#ifdef CONFIG_ESPI_TAF
struct espi_aspeed_taf_pckt {
	uint32_t pkt_len;
	uint8_t pkt[sizeof(struct espi_flash_rwe) + ESPI_PLD_LEN_MAX];
};

static struct espi_aspeed_taf_pckt espi_aspeed_taf_pckt;
static void espi_taf_dispatch(struct espi_aspeed_taf_pckt *pckt);
#endif

static void espi_ast2700_flash_isr(struct espi_ast2700_data *data)
{
	struct espi_ast2700_flash *flash = &data->flash;
	uint32_t sts;

	sts = ESPI_RD(ESPI_CH3_INT_STS);

	if (sts & ESPI_CH3_INT_STS_TX_CMPLT) {
		ESPI_WR(ESPI_CH3_INT_STS_TX_CMPLT, ESPI_CH3_INT_STS);
	}

	if (sts & ESPI_CH3_INT_STS_RX_CMPLT) {
#ifdef CONFIG_ESPI_TAF
		uint32_t cap = ESPI_RD(ESPI_CAP_CH3_0);

		if (cap & ESPI_CAP_CH3_0_SHARE_MODE) {
			int i;
			uint32_t reg, cyc, tag, len;
			struct espi_flash_rwe *rwe;

			reg = ESPI_RD(ESPI_CH3_RX_CTRL);
			cyc = FIELD_GET(ESPI_CH3_RX_CTRL_CYC, reg);
			tag = FIELD_GET(ESPI_CH3_RX_CTRL_TAG, reg);
			len = FIELD_GET(ESPI_CH3_RX_CTRL_LEN, reg);

			switch (cyc) {
			case ESPI_FLASH_READ:
			case ESPI_FLASH_WRITE:
			case ESPI_FLASH_ERASE:
				espi_aspeed_taf_pckt.pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) +
								sizeof(struct espi_flash_rwe);
				rwe = (struct espi_flash_rwe *)espi_aspeed_taf_pckt.pkt;
				rwe->cyc   = cyc;
				rwe->tag   = tag;
				rwe->len_h = len >> 8;
				rwe->len_l = len & 0xff;

				if (flash->dma.enable) {
					size_t dma_len = espi_aspeed_taf_pckt.pkt_len -
						offsetof(struct espi_flash_rwe, addr_be);

					espi_aspeed_dma_cache_range(flash->dma.rx_virt, dma_len,
								 sys_cache_data_invd_range);

					memcpy(espi_aspeed_taf_pckt.pkt +
						offsetof(struct espi_flash_rwe, addr_be),
						flash->dma.rx_virt, dma_len);
				} else {
					/*
					 * read addr_be + payload from HW FIFO, starting at
					 * offset 3 (after hdr)
					 */
					for (i = offsetof(struct espi_flash_rwe, addr_be);
						i < espi_aspeed_taf_pckt.pkt_len; ++i)
						espi_aspeed_taf_pckt.pkt[i] =
							ESPI_RD(ESPI_CH3_RX_DATA) & 0xff;
				}

				espi_taf_dispatch(&espi_aspeed_taf_pckt);
				ESPI_WR(ESPI_CH3_INT_STS_RX_CMPLT, ESPI_CH3_INT_STS);
				return;
			default:
				LOG_ERR("flash isr: non-taf flash cyc=0x%02x", cyc);
				break;
			}
		}
#endif
		ESPI_WR(ESPI_CH3_INT_STS_RX_CMPLT, ESPI_CH3_INT_STS);
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
		ESPI_WR((mask >> 32), ESPI_CH3_EDAF_MASKH);
		ESPI_WR(mask & 0xffffffff, ESPI_CH3_EDAF_MASKL);
		ESPI_WR((flash->edaf.taddr >> 32), ESPI_CH3_EDAF_TADDRH);
		ESPI_WR(flash->edaf.taddr & 0xffffffff, ESPI_CH3_EDAF_TADDRL);
	}

	reg = ESPI_RD(ESPI_CH3_CTRL) & ~ESPI_CH3_CTRL_EDAF_MODE;
	reg |= FIELD_PREP(ESPI_CH3_CTRL_EDAF_MODE, flash->edaf.mode);
	ESPI_WR(reg, ESPI_CH3_CTRL);

	if (flash->dma.enable) {
		ESPI_WR((flash->dma.tx_addr >> 32), ESPI_CH3_TX_DMAH);
		ESPI_WR(flash->dma.tx_addr & 0xffffffff, ESPI_CH3_TX_DMAL);
		ESPI_WR((flash->dma.rx_addr >> 32), ESPI_CH3_RX_DMAH);
		ESPI_WR(flash->dma.rx_addr & 0xffffffff, ESPI_CH3_RX_DMAL);

		reg = ESPI_RD(ESPI_CH3_CTRL)
		      | ESPI_CH3_CTRL_TX_DMA_EN
		      | ESPI_CH3_CTRL_RX_DMA_EN;
		ESPI_WR(reg, ESPI_CH3_CTRL);
	}

	ESPI_WR((ESPI_CH3_INT_EN_RX_CMPLT | ESPI_CH3_INT_EN_TX_CMPLT), ESPI_CH3_INT_EN);

	reg = ESPI_RD(ESPI_CH3_CTRL) | ESPI_CH3_CTRL_SW_RDY;
	ESPI_WR(reg, ESPI_CH3_CTRL);
}

static void espi_ast2700_flash_init(struct espi_ast2700_flash *flash)
{
	flash->dma.enable = DT_INST_PROP(0, flash_dma_mode);
	flash->edaf.mode = DT_INST_PROP_OR(0, flash_edaf_mode, 2);
	if (flash->edaf.mode == 0) {
		flash->edaf.taddr = COND_CODE_1(DT_INST_NODE_HAS_PROP(0, flash_edaf_tgt_addr),
			((uint64_t)DT_INST_PROP_BY_IDX(0, flash_edaf_tgt_addr, 0) << 32
			| DT_INST_PROP_BY_IDX(0, flash_edaf_tgt_addr, 1)),
			(0));
	}

	flash->edaf.size = FLASH_EDAF_ALIGN;
	flash->dma.tx_virt = flash_tx_buf;
	flash->dma.tx_addr = TO_PHY_ADDR((uintptr_t)flash->dma.tx_virt);
	flash->dma.rx_virt = flash_rx_buf;
	flash->dma.rx_addr = TO_PHY_ADDR((uintptr_t)flash->dma.rx_virt);

#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
#if defined(CONFIG_ESPI_FLASH_CHANNEL) && defined(CONFIG_ESPI_TAF)
	{
		uint32_t reg;

		reg = sys_read32(scu_base + SCU_OTP_STRAP_3);
		reg |= SCU_OTP_STRAP_TAF_EN;
		sys_write32(reg, scu_base + SCU_OTP_STRAP_3);
	}
#endif
#endif

	k_sem_init(&flash->tx_lock, 1, 1);
	k_sem_init(&flash->rx_lock, 1, 1);
	k_sem_init(&flash->rx_ready, 0, 1);
}

#ifdef CONFIG_ESPI_TAF
struct espi_taf_data {
	const struct device *host_dev;
	espi_taf_handler_t handler;
	void *user_data;
	struct k_work work;
	struct espi_aspeed_taf_pckt pckt;
};

static struct espi_taf_data espi_taf_data;

static void taf_send_unsuc_cmplt(const struct device *host_dev, uint8_t tag)
{
	struct espi_flash_cmplt cmplt = {
		.cyc   = ESPI_FLASH_UNSUC_CMPLT,
		.tag   = tag,
		.len_h = 0,
		.len_l = 0,
	};
	struct espi_aspeed_ioc ioc = {
		.pkt     = (uint8_t *)&cmplt,
		.pkt_len = sizeof(cmplt),
	};
	int ret = espi_aspeed_flash_put_tx(host_dev, &ioc);

	if (ret)
		LOG_ERR("UNSUC_CMPLT put_tx failed: %d", ret);
}

static void espi_aspeed_flash_release_rx(void)
{
	if (ESPI_RD(ESPI_CH3_RX_CTRL) & ESPI_CH3_RX_CTRL_SERV_PEND)
		ESPI_WR(ESPI_CH3_RX_CTRL_SERV_PEND, ESPI_CH3_RX_CTRL);
}

static void espi_taf_work(struct k_work *item)
{
	struct espi_taf_data *data =
		CONTAINER_OF(item, struct espi_taf_data, work);
	struct espi_flash_rwe *rwe = (struct espi_flash_rwe *)data->pckt.pkt;
	struct espi_taf_req req = {
		.cyc  = rwe->cyc,
		.tag  = rwe->tag,
		.addr = sys_be32_to_cpu(rwe->addr_be),
		.len  = ((uint16_t)rwe->len_h << 8) | rwe->len_l,
		.data = (rwe->cyc == ESPI_FLASH_WRITE) ? rwe->data : NULL,
	};

	LOG_INF("TAF req: cyc=0x%02x tag=%d addr=0x%08x len=%d",
		req.cyc, req.tag, req.addr, req.len);

	if (data->handler)
		data->handler(data->host_dev, &req, data->user_data);
	else
		taf_send_unsuc_cmplt(data->host_dev, rwe->tag);

	espi_aspeed_flash_release_rx();
}

static void espi_taf_dispatch(struct espi_aspeed_taf_pckt *pckt)
{
	memcpy(&espi_taf_data.pckt, pckt, sizeof(*pckt));
	k_work_submit(&espi_taf_data.work);
}

static void espi_taf_init(const struct device *dev)
{
	espi_taf_data.host_dev = dev;
	k_work_init(&espi_taf_data.work, espi_taf_work);
}

int espi_aspeed_taf_register(const struct device *dev,
			      espi_taf_handler_t handler,
			      void *user_data)
{
	ARG_UNUSED(dev);

	espi_taf_data.handler = handler;
	espi_taf_data.user_data = user_data;

	return 0;
}
#endif

/* eSPI controller config. */
struct espi_ast2700_config {
	uintptr_t base;
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
	uintptr_t scu_base;
#endif
};

static const struct espi_ast2700_config espi_ast2700_config = {
	.base = DT_INST_REG_ADDR(0),
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
	.scu_base = DT_REG_ADDR_BY_IDX(DT_INST_PHANDLE_BY_IDX(0, aspeed_scu, 0), 0),
#endif
};

static void espi_ast2700_isr(const struct device *dev)
{
	uint32_t sts;
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

	if (sts & ESPI_INT_STS_RST_DEASSERT) {
		espi_ast2700_perif_reset(&data->perif);
		espi_ast2700_vw_reset(&data->vw);
		espi_ast2700_oob_reset(&data->oob);
		espi_ast2700_flash_reset(&data->flash);
		ESPI_WR(ESPI_INT_STS_RST_DEASSERT, ESPI_INT_STS);
	}
	ESPI_WR(ESPI_INT_STS_RST_DEASSERT, ESPI_INT_STS);
	/* dummy read to make sure W1C arrives HW */
	sts = ESPI_RD(ESPI_INT_STS);
}

static int espi_ast2700_init(const struct device *dev)
{
	uint32_t reg;
	struct espi_ast2700_config *cfg = (struct espi_ast2700_config *)dev->config;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;

	data->dev = dev;

	espi_base = cfg->base;
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
	scu_base = cfg->scu_base;
#endif

	reg = ESPI_RD(ESPI_INT_EN);
	reg &= ~ESPI_INT_EN_RST_DEASSERT;
	ESPI_WR(reg, ESPI_INT_EN);

	espi_ast2700_perif_init(&data->perif);
	espi_ast2700_vw_init(&data->vw);
	espi_ast2700_oob_init(&data->oob);
	espi_ast2700_flash_init(&data->flash);

	espi_ast2700_perif_reset(&data->perif);
	espi_ast2700_vw_reset(&data->vw);
	espi_ast2700_oob_reset(&data->oob);
	espi_ast2700_flash_reset(&data->flash);

#ifdef CONFIG_ESPI_TAF
	espi_taf_init(dev);
#endif

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
	uint64_t addr;
	uint8_t *data_buf;
	uint32_t data_len;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_perif *perif = &data->perif;
	uint32_t *addr32 = (uint32_t *)(hdr + 1);
	uint64_t *addr64 = (uint64_t *)(hdr + 1);

	rc = k_sem_take(&perif->pc_rx_lock, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		return rc;

	rc = k_sem_take(&perif->pc_rx_ready, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		goto unlock_n_out;

	reg = ESPI_RD(ESPI_CH0_PC_RX_CTRL);
	cyc = FIELD_GET(ESPI_CH0_PC_RX_CTRL_CYC, reg);
	tag = FIELD_GET(ESPI_CH0_PC_RX_CTRL_TAG, reg);
	len = FIELD_GET(ESPI_CH0_PC_RX_CTRL_LEN, reg);

	switch (cyc) {
	case ESPI_PERIF_MEMWR32:
		ioc->pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) +
			sizeof(*hdr) + sizeof(uint32_t);
		*addr32 = sys_cpu_to_be32(ESPI_RD(ESPI_CH0_PC_RX_ADDRL));
		data_buf = (uint8_t *)(addr32 + 1);
		data_len = ioc->pkt_len - sizeof(*hdr) - sizeof(uint32_t);
		break;
	case ESPI_PERIF_MEMWR64:
		ioc->pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) +
			sizeof(*hdr) + sizeof(uint64_t);
		addr = ((uint64_t)ESPI_RD(ESPI_CH0_PC_RX_ADDRH) << 32) |
		       ESPI_RD(ESPI_CH0_PC_RX_ADDRL);
		*addr64 = sys_cpu_to_be64(addr);
		data_buf = (uint8_t *)(addr32 + 2);
		data_len = ioc->pkt_len - sizeof(*hdr) - sizeof(uint64_t);
		break;
	case ESPI_PERIF_MSG:
		ioc->pkt_len = len + sizeof(struct espi_perif_msg);
		data_buf = (uint8_t *)(hdr + 1);
		data_len = ioc->pkt_len - sizeof(*hdr);
		break;
	case ESPI_PERIF_MSG_D:
		ioc->pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) +
			sizeof(struct espi_perif_msg);
		data_buf = (uint8_t *)(hdr + 1);
		data_len = ioc->pkt_len - sizeof(*hdr);
		break;
	case ESPI_PERIF_SUC_CMPLT_D_MIDDLE:
	case ESPI_PERIF_SUC_CMPLT_D_FIRST:
	case ESPI_PERIF_SUC_CMPLT_D_LAST:
	case ESPI_PERIF_SUC_CMPLT_D_ONLY:
		ioc->pkt_len = ((len) ? len : ESPI_PLD_LEN_MAX) +
			sizeof(struct espi_perif_cmplt);
		data_buf = (uint8_t *)(hdr + 1);
		data_len = ioc->pkt_len - sizeof(*hdr);
		break;
	case ESPI_PERIF_SUC_CMPLT:
	case ESPI_PERIF_UNSUC_CMPLT:
		ioc->pkt_len = len + sizeof(struct espi_perif_cmplt);
		data_buf = (uint8_t *)(hdr + 1);
		data_len = ioc->pkt_len - sizeof(*hdr);
		break;
	default:
		__ASSERT(0, "Unrecognized eSPI peripheral packet");

		k_sem_give(&perif->pc_rx_ready);
		rc = -EFAULT;
		goto unlock_n_out;
	}

	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	if (perif->dma.enable) {
		espi_aspeed_dma_cache_range(perif->dma.pc_rx_virt, data_len,
						 sys_cache_data_invd_range);
		memcpy(data_buf, perif->dma.pc_rx_virt, data_len);
	} else
		for (i = 0; i < data_len; ++i)
			data_buf[i] = (ESPI_RD(ESPI_CH0_PC_RX_DATA) & 0xff);

	ESPI_WR(reg | ESPI_CH0_PC_RX_CTRL_SERV_PEND, ESPI_CH0_PC_RX_CTRL);

unlock_n_out:
	k_sem_give(&perif->pc_rx_lock);

	return rc;
}

int espi_aspeed_perif_pc_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_perif *perif = &data->perif;

	rc = k_sem_take(&perif->pc_tx_lock, K_NO_WAIT);
	if (rc)
		return rc;

	reg = ESPI_RD(ESPI_CH0_PC_TX_CTRL);
	if (reg & ESPI_CH0_PC_TX_CTRL_TRIG_PEND) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	if (perif->dma.enable) {
		memcpy(perif->dma.pc_tx_virt, hdr + 1, ioc->pkt_len - sizeof(*hdr));
		espi_aspeed_dma_cache_range(perif->dma.pc_tx_virt,
						 ioc->pkt_len - sizeof(*hdr),
						 sys_cache_data_flush_range);
	} else
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			ESPI_WR(ioc->pkt[i], ESPI_CH0_PC_TX_DATA);

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = FIELD_PREP(ESPI_CH0_PC_TX_CTRL_CYC, cyc)
	      | FIELD_PREP(ESPI_CH0_PC_TX_CTRL_TAG, tag)
	      | FIELD_PREP(ESPI_CH0_PC_TX_CTRL_LEN, len)
	      | ESPI_CH0_PC_TX_CTRL_TRIG_PEND;

#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
	if (cyc == ESPI_PERIF_SUC_CMPLT || cyc == ESPI_PERIF_SUC_CMPLT_D_LAST ||
		cyc == ESPI_PERIF_SUC_CMPLT_D_ONLY || cyc == ESPI_PERIF_UNSUC_CMPLT) {
		reg |= ESPI_CH0_PC_TX_CTRL_FW;
	}
#endif

	ESPI_WR(reg, ESPI_CH0_PC_TX_CTRL);

unlock_n_out:
	k_sem_give(&perif->pc_tx_lock);

	return rc;
}

#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_espi_ast1040)
int espi_aspeed_perif_np_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc,
				bool blocking)
{
	int rc;
	uint32_t reg;
	uint32_t len;
	uint64_t addr;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_perif_mem32 *mem32 = (struct espi_perif_mem32 *)ioc->pkt;
	struct espi_perif_io *io = (struct espi_perif_io *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_perif *perif = &data->perif;

	rc = k_sem_take(&perif->np_rx_lock, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		return rc;

	rc = k_sem_take(&perif->np_rx_ready, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		goto unlock_n_out;

	reg = ESPI_RD(ESPI_CH0_NP_RX_CTRL);
	len = FIELD_GET(ESPI_CH0_NP_RX_CTRL_LEN, reg);

	hdr->tag = FIELD_GET(ESPI_CH0_NP_RX_CTRL_TAG, reg);
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	if (reg & ESPI_CH0_NP_RX_MEM64_RD) {
		hdr->cyc = ESPI_PERIF_MEMRD64;
		ioc->pkt_len = sizeof(struct espi_comm_hdr) + sizeof(uint64_t);
		addr = ((uint64_t)ESPI_RD(ESPI_CH0_NP_RX_ADDRH) << 32) |
		       ESPI_RD(ESPI_CH0_NP_RX_ADDRL);
		sys_put_be64(addr, ioc->pkt + sizeof(struct espi_comm_hdr));
	} else if (reg & ESPI_CH0_NP_RX_MEM32_RD) {
		hdr->cyc = ESPI_PERIF_MEMRD32;
		ioc->pkt_len = sizeof(*mem32);
		mem32->addr_be = sys_cpu_to_be32(ESPI_RD(ESPI_CH0_NP_RX_ADDRL));
	} else if (reg & ESPI_CH0_NP_RX_IO_WR) {
		hdr->cyc = ESPI_PERIF_IOWR;
		ioc->pkt_len = sizeof(*io);
		io->addr_be = sys_cpu_to_be16(ESPI_RD(ESPI_CH0_NP_RX_ADDRL) & 0xffff);
	} else if (reg & ESPI_CH0_NP_RX_IO_RD) {
		hdr->cyc = ESPI_PERIF_IORD;
		ioc->pkt_len = sizeof(*io);
		io->addr_be = sys_cpu_to_be16(ESPI_RD(ESPI_CH0_NP_RX_ADDRL) & 0xffff);
	} else {
		__ASSERT(0, "Unrecognized eSPI peripheral NP packet");

		k_sem_give(&perif->np_rx_ready);
		rc = -EFAULT;
		goto unlock_n_out;
	}

	ESPI_WR(reg | ESPI_CH0_NP_RX_CTRL_SERV_PEND, ESPI_CH0_NP_RX_CTRL);
	rc = 0;

unlock_n_out:
	k_sem_give(&perif->np_rx_lock);

	return rc;
}
#endif

int espi_aspeed_perif_np_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_perif *perif = &data->perif;

	rc = k_sem_take(&perif->np_tx_lock, K_NO_WAIT);
	if (rc)
		return rc;

	reg = ESPI_RD(ESPI_CH0_NP_TX_CTRL);
	if (reg & ESPI_CH0_NP_TX_CTRL_TRIG_PEND) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	if (perif->dma.enable) {
		memcpy(perif->dma.np_tx_virt, hdr + 1, ioc->pkt_len - sizeof(*hdr));
		espi_aspeed_dma_cache_range(perif->dma.np_tx_virt,
						 ioc->pkt_len - sizeof(*hdr),
						 sys_cache_data_flush_range);
	} else
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

int espi_ast2700_oob_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc, bool blocking)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint32_t wptr;
	struct espi_ast2700_oob_dma_rx_desc *d;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_oob *oob = &data->oob;

#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
	/* In async mode, data is already present when this is called from callback.
	 * If the lock is held by another reader, fail immediately rather than block.
	 */
	rc = k_sem_take(&oob->rx_lock, K_NO_WAIT);
#else
	rc = k_sem_take(&oob->rx_lock, (blocking) ? K_FOREVER : K_NO_WAIT);
#endif
	if (rc)
		return rc;

#ifndef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
	rc = k_sem_take(&oob->rx_ready, (blocking) ? K_FOREVER : K_NO_WAIT);
	if (rc)
		goto unlock_n_out;
#endif

	if (oob->dma.enable) {
		wptr = ESPI_RD(ESPI_CH2_RX_DESC_WPTR);

		d = &oob->dma.rxd_virt[wptr];

		espi_aspeed_dma_cache_range(d, sizeof(*d), sys_cache_data_invd_range);

		ioc->pkt_len = (d->len) ? d->len : ESPI_PLD_LEN_MAX;
		ioc->pkt_len += sizeof(struct espi_comm_hdr);

		hdr->cyc = d->cyc;
		hdr->tag = d->tag;
		hdr->len_h = d->len >> 8;
		hdr->len_l = d->len & 0xff;

		espi_aspeed_dma_cache_range(oob->dma.rx_virt + (ESPI_PLD_LEN_MAX * wptr),
					   ioc->pkt_len - sizeof(*hdr),
					   sys_cache_data_invd_range);
		memcpy(hdr + 1, oob->dma.rx_virt + (ESPI_PLD_LEN_MAX * wptr),
		       ioc->pkt_len - sizeof(*hdr));

		d->dirty = 0;
		espi_aspeed_dma_cache_range(d, sizeof(*d), sys_cache_data_flush_range);

		wptr = (wptr + 1) % OOB_DMA_DESC_NUM;
		ESPI_WR(wptr | ESPI_CH2_RX_DESC_WPTR_VALID, ESPI_CH2_RX_DESC_WPTR);

		espi_aspeed_dma_cache_range(&oob->dma.rxd_virt[wptr],
					   sizeof(oob->dma.rxd_virt[0]),
					   sys_cache_data_invd_range);
		if (oob->dma.rxd_virt[wptr].dirty)
			k_sem_give(&oob->rx_ready);

	} else {
		reg = ESPI_RD(ESPI_CH2_RX_CTRL);
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

#ifndef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
unlock_n_out:
#endif
	k_sem_give(&oob->rx_lock);

	return rc;
}

int espi_ast2700_oob_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint32_t rptr, wptr;
	struct espi_ast2700_oob_dma_tx_desc *d;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_oob *oob = &data->oob;

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

		d = &oob->dma.txd_virt[wptr];
		d->cyc = hdr->cyc;
		d->tag = hdr->tag;
		d->len = (hdr->len_h << 8) | (hdr->len_l & 0xff);
		d->msg_type = OOB_DMA_DESC_CUSTOM;
		espi_aspeed_dma_cache_range(d, sizeof(*d), sys_cache_data_flush_range);

		memcpy(oob->dma.tx_virt + (ESPI_PLD_LEN_MAX * wptr), hdr + 1,
		       ioc->pkt_len - sizeof(*hdr));
		espi_aspeed_dma_cache_range(oob->dma.tx_virt + (ESPI_PLD_LEN_MAX * wptr),
					   ioc->pkt_len - sizeof(*hdr),
					   sys_cache_data_flush_range);

		wptr = (wptr + 1) % OOB_DMA_DESC_NUM;
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
	struct espi_ast2700_flash *flash = &data->flash;

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

	if (flash->dma.enable) {
		espi_aspeed_dma_cache_range(flash->dma.rx_virt, ioc->pkt_len - sizeof(*hdr),
					   sys_cache_data_invd_range);
		memcpy(hdr + 1, flash->dma.rx_virt, ioc->pkt_len - sizeof(*hdr));
	} else
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
	size_t payload_len;
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)ioc->pkt;
	struct espi_flash_rwe *flash_rwe = (struct espi_flash_rwe *)ioc->pkt;
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;
	struct espi_ast2700_flash *flash = &data->flash;

	rc = k_sem_take(&flash->tx_lock, K_NO_WAIT);
	if (rc)
		return rc;

	reg = ESPI_RD(ESPI_CH3_TX_CTRL);
	if (reg & ESPI_CH3_TX_CTRL_TRIG_PEND) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	cyc = flash_rwe->cyc;
	tag = flash_rwe->tag;
	len = (flash_rwe->len_h << 8) | (flash_rwe->len_l & 0xff);
	payload_len = ioc->pkt_len - sizeof(*hdr);

	if (flash->dma.enable) {
		if (payload_len) {
			memcpy(flash->dma.tx_virt, hdr + 1, payload_len);
			espi_aspeed_dma_cache_range(flash->dma.tx_virt, payload_len,
						   sys_cache_data_flush_range);
		}
	} else {
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i) {
			ESPI_WR(ioc->pkt[i], ESPI_CH3_TX_DATA);
		}
	}

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
		return ESPI_RD(ESPI_CH0_CTRL) & ESPI_CH0_CTRL_SW_RDY;
	case ESPI_CHANNEL_VWIRE:
		return ESPI_RD(ESPI_CH1_CTRL) & ESPI_CH1_CTRL_SW_RDY;
	case ESPI_CHANNEL_OOB:
		return ESPI_RD(ESPI_CH2_CTRL) & ESPI_CH2_CTRL_SW_RDY;
	case ESPI_CHANNEL_FLASH:
		return ESPI_RD(ESPI_CH3_CTRL) & ESPI_CH3_CTRL_SW_RDY;
	default:
		return false;
	}

	return false;
}

static int espi_ast2700_send_oob(const struct device *dev, struct espi_oob_packet *pckt)
{
	struct espi_oob_msg *oob_msg;
	struct espi_aspeed_ioc ioc;
	uint8_t pkt[sizeof(*oob_msg) + ESPI_PLD_LEN_MAX];

	ioc.pkt = pkt;
	ioc.pkt_len = sizeof(*oob_msg) + pckt->len;

	oob_msg = (struct espi_oob_msg *)pkt;
	oob_msg->cyc = ESPI_OOB_MSG;
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
	struct espi_aspeed_ioc ioc;
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
	struct espi_aspeed_ioc ioc;
	uint8_t pkt[sizeof(*flash_rwe) + ESPI_PLD_LEN_MAX];
	uint32_t len;
	uint32_t payload_len;

	ioc.pkt = pkt;

	if (flash_op == ESPI_FLASH_ERASE || flash_op == ESPI_FLASH_READ) {
		len = pckt->len;
		payload_len = 0;
	} else {
		len = pckt->len;
		payload_len = pckt->len;
	}

	flash_rwe = (struct espi_flash_rwe *)pkt;
	flash_rwe->cyc = flash_op;
	flash_rwe->len_h = (len >> 8) & 0xf;
	flash_rwe->tag = FLASH_TAG;
	flash_rwe->len_l = len & 0xff;
	flash_rwe->addr_be = BSWAP_32(pckt->flash_addr);

	ioc.pkt_len = sizeof(*flash_rwe) + payload_len;

	if (payload_len && pckt->buf) {
		memcpy(flash_rwe + 1, pckt->buf, payload_len);
	}

	return espi_aspeed_flash_put_tx(dev, &ioc);
}

static int espi_ast2700_flash_read(const struct device *dev, struct espi_flash_packet *pckt)
{
	struct espi_aspeed_ioc ioc;
	struct espi_flash_cmplt *cmplt;
	uint8_t pkt[sizeof(*cmplt) + ESPI_PLD_LEN_MAX];
	int rc;

	rc = espi_ast2700_flash_rwe(dev, pckt, ESPI_FLASH_READ);
	if (rc)
		return rc;

	ioc.pkt = pkt;
	ioc.pkt_len = sizeof(pkt);

	rc = espi_aspeed_flash_get_rx(dev, &ioc, true);
	if (rc)
		return rc;

	cmplt = (struct espi_flash_cmplt *)pkt;
	if (pckt->buf)
		memcpy(pckt->buf, cmplt + 1, pckt->len);

	return 0;
}

static int espi_ast2700_flash_write(const struct device *dev, struct espi_flash_packet *pckt)
{
	return espi_ast2700_flash_rwe(dev, pckt, ESPI_FLASH_WRITE);
}

static int espi_ast2700_flash_erase(const struct device *dev, struct espi_flash_packet *pckt)
{
	return espi_ast2700_flash_rwe(dev, pckt, ESPI_FLASH_ERASE);
}

static int espi_ast2700_manage_callback(const struct device *dev,
				       struct espi_callback *callback,
				       bool set)
{
	struct espi_ast2700_data *data = (struct espi_ast2700_data *)dev->data;

	return espi_manage_callback(&data->callbacks, callback, set);
}

/*
 * espi_read_request: Target sends a non-posted memory read request to Controller
 * and waits for the completion with data returned via PC RX.
 *
 * Flow: NP TX (MEMRD32/64) → Controller → PC RX (SUC_CMPLT_D_ONLY)
 * req->data must point to a buffer of at least req->len bytes.
 */
static int espi_ast2700_read_request(const struct device *dev,
				     struct espi_request_packet *req)
{
	int rc;
	/* mem64 needs comm_hdr(3) + addr64(8) = 11 bytes; pad beyond struct size */
	uint8_t tx_buf[sizeof(struct espi_perif_mem64) + sizeof(uint32_t)];
	uint8_t rx_buf[sizeof(struct espi_perif_cmplt) + ESPI_PLD_LEN_MAX];
	struct espi_perif_mem32 *mem32 = (struct espi_perif_mem32 *)tx_buf;
	struct espi_perif_mem64 *mem64 = (struct espi_perif_mem64 *)tx_buf;
	struct espi_perif_cmplt *cmplt = (struct espi_perif_cmplt *)rx_buf;
	struct espi_aspeed_ioc tx_ioc = { .pkt = tx_buf };
	struct espi_aspeed_ioc rx_ioc = { .pkt = rx_buf, .pkt_len = sizeof(rx_buf) };

	switch (req->cycle_type) {
	case ESPI_CYCLE_MEMORY_READ32:
		mem32->cyc = ESPI_PERIF_MEMRD32;
		mem32->tag = req->tag;
		mem32->len_h = (req->len >> 8) & 0xf;
		mem32->len_l = req->len & 0xff;
		sys_put_be32(req->address, (uint8_t *)&mem32->addr_be);
		tx_ioc.pkt_len = sizeof(*mem32);
		break;
	case ESPI_CYCLE_MEMORY_READ64:
		mem64->cyc = ESPI_PERIF_MEMRD64;
		mem64->tag = req->tag;
		mem64->len_h = (req->len >> 8) & 0xf;
		mem64->len_l = req->len & 0xff;
		sys_put_be64((uint64_t)req->address,
			     tx_buf + sizeof(struct espi_comm_hdr));
		tx_ioc.pkt_len = sizeof(*mem64) + sizeof(uint32_t);
		break;
	default:
		return -EINVAL;
	}

	/* Split completion is not supported; limit to a single SUC_CMPLT_D_ONLY */
	if (req->len > ESPI_PLD_LEN_MAX)
		return -EINVAL;

	rc = espi_aspeed_perif_np_put_tx(dev, &tx_ioc);
	if (rc)
		return rc;

	rc = espi_aspeed_perif_pc_get_rx(dev, &rx_ioc, true);
	if (rc)
		return rc;

	if (cmplt->cyc == ESPI_PERIF_UNSUC_CMPLT)
		return -EIO;

	if (cmplt->cyc != ESPI_PERIF_SUC_CMPLT_D_ONLY)
		return -ENOTSUP;

	if (req->data) {
		uint16_t data_len = ((uint16_t)cmplt->len_h << 8) | cmplt->len_l;

		memcpy(req->data, cmplt->data, MIN(data_len, req->len));
	}

	return 0;
}

/*
 * espi_write_request: Target sends a posted memory write request to Controller.
 *
 * Builds a MEMWR32 or MEMWR64 packet from req and transmits it via PC TX.
 * req->cycle_type must be ESPI_CYCLE_MEMORY_WRITE32 or ESPI_CYCLE_MEMORY_WRITE64.
 */
static int espi_ast2700_write_request(const struct device *dev,
				      struct espi_request_packet *req)
{
	uint8_t pkt_buf[sizeof(struct espi_perif_mem64) + ESPI_PLD_LEN_MAX];
	struct espi_perif_mem32 *mem32 = (struct espi_perif_mem32 *)pkt_buf;
	struct espi_perif_mem64 *mem64 = (struct espi_perif_mem64 *)pkt_buf;
	struct espi_aspeed_ioc ioc = { .pkt = pkt_buf };

	if (req->len > ESPI_PLD_LEN_MAX)
		return -EINVAL;

	switch (req->cycle_type) {
	case ESPI_CYCLE_MEMORY_WRITE32:
		mem32->cyc = ESPI_PERIF_MEMWR32;
		mem32->tag = req->tag;
		mem32->len_h = (req->len >> 8) & 0xf;
		mem32->len_l = req->len & 0xff;
		sys_put_be32(req->address, (uint8_t *)&mem32->addr_be);
		memcpy(mem32->data, req->data, req->len);
		ioc.pkt_len = sizeof(*mem32) + req->len;
		break;
	case ESPI_CYCLE_MEMORY_WRITE64:
		mem64->cyc = ESPI_PERIF_MEMWR64;
		mem64->tag = req->tag;
		mem64->len_h = (req->len >> 8) & 0xf;
		mem64->len_l = req->len & 0xff;
		sys_put_be64((uint64_t)req->address,
			     pkt_buf + sizeof(struct espi_comm_hdr));
		/* data follows the full 8-byte address at offset comm_hdr(3)+addr64(8)=11 */
		memcpy(pkt_buf + sizeof(struct espi_comm_hdr) + sizeof(uint64_t),
		       req->data, req->len);
		ioc.pkt_len = sizeof(struct espi_comm_hdr) + sizeof(uint64_t) + req->len;
		break;
	default:
		return -EINVAL;
	}

	return espi_aspeed_perif_pc_put_tx(dev, &ioc);
}

static const struct espi_driver_api espi_ast2700_driver_api = {
	.get_channel_status = espi_ast2700_channel_ready,
	.read_request = espi_ast2700_read_request,
	.write_request = espi_ast2700_write_request,
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
