/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_espi

#include <device.h>
#include <kernel.h>
#include <soc.h>
#include <errno.h>
#include <string.h>
#include <drivers/espi.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(espi, CONFIG_ESPI_LOG_LEVEL);

/* SCU register offset */
#define SCU_HWSTRAP2		0x510
#define SCU_HWSTRAP2_LPC	BIT(6)

/* eSPI register offset */
#define ESPI_CTRL                       0x000
#define   ESPI_CTRL_OOB_RX_SW_RST               BIT(28)
#define   ESPI_CTRL_FLASH_TX_DMA_EN             BIT(23)
#define   ESPI_CTRL_FLASH_RX_DMA_EN             BIT(22)
#define   ESPI_CTRL_OOB_TX_DMA_EN               BIT(21)
#define   ESPI_CTRL_OOB_RX_DMA_EN               BIT(20)
#define   ESPI_CTRL_PERIF_NP_TX_DMA_EN          BIT(19)
#define   ESPI_CTRL_PERIF_PC_TX_DMA_EN          BIT(17)
#define   ESPI_CTRL_PERIF_PC_RX_DMA_EN          BIT(16)
#define   ESPI_CTRL_FLASH_SW_MODE_MASK          GENMASK(11, 10)
#define   ESPI_CTRL_FLASH_SW_MODE_SHIFT         10
#define   ESPI_CTRL_PERIF_PC_RX_DMA_EN          BIT(16)
#define   ESPI_CTRL_FLASH_SW_RDY                BIT(7)
#define   ESPI_CTRL_VW_SW_RDY                   BIT(3)
#define   ESPI_CTRL_OOB_SW_RDY                  BIT(4)
#define   ESPI_CTRL_PERIF_SW_RDY                BIT(1)
#define ESPI_STS                        0x004
#define ESPI_INT_STS                    0x008
#define   ESPI_INT_STS_HW_RST_DEASSERT          BIT(31)
#define   ESPI_INT_STS_HW_RST_ASSERT            BIT(30)
#define   ESPI_INT_STS_OOB_RX_TMOUT             BIT(23)
#define   ESPI_INT_STS_VW_SYSEVT1               BIT(22)
#define   ESPI_INT_STS_FLASH_TX_ERR             BIT(21)
#define   ESPI_INT_STS_OOB_TX_ERR               BIT(20)
#define   ESPI_INT_STS_FLASH_TX_ABT             BIT(19)
#define   ESPI_INT_STS_OOB_TX_ABT               BIT(18)
#define   ESPI_INT_STS_PERIF_NP_TX_ABT          BIT(17)
#define   ESPI_INT_STS_PERIF_PC_TX_ABT          BIT(16)
#define   ESPI_INT_STS_FLASH_RX_ABT             BIT(15)
#define   ESPI_INT_STS_OOB_RX_ABT               BIT(14)
#define   ESPI_INT_STS_PERIF_NP_RX_ABT          BIT(13)
#define   ESPI_INT_STS_PERIF_PC_RX_ABT          BIT(12)
#define   ESPI_INT_STS_PERIF_NP_TX_ERR          BIT(11)
#define   ESPI_INT_STS_PERIF_PC_TX_ERR          BIT(10)
#define   ESPI_INT_STS_VW_GPIOEVT               BIT(9)
#define   ESPI_INT_STS_VW_SYSEVT                BIT(8)
#define   ESPI_INT_STS_FLASH_TX_CMPLT           BIT(7)
#define   ESPI_INT_STS_FLASH_RX_CMPLT           BIT(6)
#define   ESPI_INT_STS_OOB_TX_CMPLT             BIT(5)
#define   ESPI_INT_STS_OOB_RX_CMPLT             BIT(4)
#define   ESPI_INT_STS_PERIF_NP_TX_CMPLT        BIT(3)
#define   ESPI_INT_STS_PERIF_PC_TX_CMPLT        BIT(1)
#define   ESPI_INT_STS_PERIF_PC_RX_CMPLT        BIT(0)
#define ESPI_INT_EN                     0x00c
#define   ESPI_INT_EN_HW_RST_DEASSERT           BIT(31)
#define   ESPI_INT_EN_HW_RST_ASSERT             BIT(30)
#define   ESPI_INT_EN_OOB_RX_TMOUT              BIT(23)
#define   ESPI_INT_EN_VW_SYSEVT1                BIT(22)
#define   ESPI_INT_EN_FLASH_TX_ERR              BIT(21)
#define   ESPI_INT_EN_OOB_TX_ERR                BIT(20)
#define   ESPI_INT_EN_FLASH_TX_ABT              BIT(19)
#define   ESPI_INT_EN_OOB_TX_ABT                BIT(18)
#define   ESPI_INT_EN_PERIF_NP_TX_ABT           BIT(17)
#define   ESPI_INT_EN_PERIF_PC_TX_ABT           BIT(16)
#define   ESPI_INT_EN_FLASH_RX_ABT              BIT(15)
#define   ESPI_INT_EN_OOB_RX_ABT                BIT(14)
#define   ESPI_INT_EN_PERIF_NP_RX_ABT           BIT(13)
#define   ESPI_INT_EN_PERIF_PC_RX_ABT           BIT(12)
#define   ESPI_INT_EN_PERIF_NP_TX_ERR           BIT(11)
#define   ESPI_INT_EN_PERIF_PC_TX_ERR           BIT(10)
#define   ESPI_INT_EN_VW_GPIOEVT                BIT(9)
#define   ESPI_INT_EN_VW_SYSEVT                 BIT(8)
#define   ESPI_INT_EN_FLASH_TX_CMPLT            BIT(7)
#define   ESPI_INT_EN_FLASH_RX_CMPLT            BIT(6)
#define   ESPI_INT_EN_OOB_TX_CMPLT              BIT(5)
#define   ESPI_INT_EN_OOB_RX_CMPLT              BIT(4)
#define   ESPI_INT_EN_PERIF_NP_TX_CMPLT         BIT(3)
#define   ESPI_INT_EN_PERIF_PC_TX_CMPLT         BIT(1)
#define   ESPI_INT_EN_PERIF_PC_RX_CMPLT         BIT(0)
#define ESPI_PERIF_PC_RX_DMA            0x010
#define ESPI_PERIF_PC_RX_CTRL           0x014
#define   ESPI_PERIF_PC_RX_CTRL_PEND_SERV       BIT(31)
#define   ESPI_PERIF_PC_RX_CTRL_LEN_MASK        GENMASK(23, 12)
#define   ESPI_PERIF_PC_RX_CTRL_LEN_SHIFT       12
#define   ESPI_PERIF_PC_RX_CTRL_TAG_MASK        GENMASK(11, 8)
#define   ESPI_PERIF_PC_RX_CTRL_TAG_SHIFT       8
#define   ESPI_PERIF_PC_RX_CTRL_CYC_MASK        GENMASK(7, 0)
#define   ESPI_PERIF_PC_RX_CTRL_CYC_SHIFT       0
#define ESPI_PERIF_PC_RX_PORT           0x018
#define ESPI_PERIF_PC_TX_DMA            0x020
#define ESPI_PERIF_PC_TX_CTRL           0x024
#define   ESPI_PERIF_PC_TX_CTRL_TRIGGER         BIT(31)
#define   ESPI_PERIF_PC_TX_CTRL_LEN_MASK        GENMASK(23, 12)
#define   ESPI_PERIF_PC_TX_CTRL_LEN_SHIFT       12
#define   ESPI_PERIF_PC_TX_CTRL_TAG_MASK        GENMASK(11, 8)
#define   ESPI_PERIF_PC_TX_CTRL_TAG_SHIFT       8
#define   ESPI_PERIF_PC_TX_CTRL_CYC_MASK        GENMASK(7, 0)
#define   ESPI_PERIF_PC_TX_CTRL_CYC_SHIFT       0
#define ESPI_PERIF_PC_TX_PORT           0x028
#define ESPI_PERIF_NP_TX_DMA            0x030
#define ESPI_PERIF_NP_TX_CTRL           0x034
#define   ESPI_PERIF_NP_TX_CTRL_TRIGGER         BIT(31)
#define   ESPI_PERIF_NP_TX_CTRL_LEN_MASK        GENMASK(23, 12)
#define   ESPI_PERIF_NP_TX_CTRL_LEN_SHIFT       12
#define   ESPI_PERIF_NP_TX_CTRL_TAG_MASK        GENMASK(11, 8)
#define   ESPI_PERIF_NP_TX_CTRL_TAG_SHIFT       8
#define   ESPI_PERIF_NP_TX_CTRL_CYC_MASK        GENMASK(7, 0)
#define   ESPI_PERIF_NP_TX_CTRL_CYC_SHIFT       0
#define ESPI_PERIF_NP_TX_PORT           0x038
#define ESPI_OOB_RX_DMA                 0x040
#define ESPI_OOB_RX_CTRL                0x044
#define   ESPI_OOB_RX_CTRL_PEND_SERV            BIT(31)
#define   ESPI_OOB_RX_CTRL_LEN_MASK             GENMASK(23, 12)
#define   ESPI_OOB_RX_CTRL_LEN_SHIFT            12
#define   ESPI_OOB_RX_CTRL_TAG_MASK             GENMASK(11, 8)
#define   ESPI_OOB_RX_CTRL_TAG_SHIFT            8
#define   ESPI_OOB_RX_CTRL_CYC_MASK             GENMASK(7, 0)
#define   ESPI_OOB_RX_CTRL_CYC_SHIFT            0
#define ESPI_OOB_RX_PORT                0x048
#define ESPI_OOB_TX_DMA                 0x050
#define ESPI_OOB_TX_CTRL                0x054
#define   ESPI_OOB_TX_CTRL_TRIGGER              BIT(31)
#define   ESPI_OOB_TX_CTRL_LEN_MASK             GENMASK(23, 12)
#define   ESPI_OOB_TX_CTRL_LEN_SHIFT            12
#define   ESPI_OOB_TX_CTRL_TAG_MASK             GENMASK(11, 8)
#define   ESPI_OOB_TX_CTRL_TAG_SHIFT            8
#define   ESPI_OOB_TX_CTRL_CYC_MASK             GENMASK(7, 0)
#define   ESPI_OOB_TX_CTRL_CYC_SHIFT            0
#define ESPI_OOB_TX_PORT                0x058
#define ESPI_FLASH_RX_DMA               0x060
#define ESPI_FLASH_RX_CTRL              0x064
#define   ESPI_FLASH_RX_CTRL_PEND_SERV          BIT(31)
#define   ESPI_FLASH_RX_CTRL_LEN_MASK           GENMASK(23, 12)
#define   ESPI_FLASH_RX_CTRL_LEN_SHIFT          12
#define   ESPI_FLASH_RX_CTRL_TAG_MASK           GENMASK(11, 8)
#define   ESPI_FLASH_RX_CTRL_TAG_SHIFT          8
#define   ESPI_FLASH_RX_CTRL_CYC_MASK           GENMASK(7, 0)
#define   ESPI_FLASH_RX_CTRL_CYC_SHIFT          0
#define ESPI_FLASH_RX_PORT              0x068
#define ESPI_FLASH_TX_DMA               0x070
#define ESPI_FLASH_TX_CTRL              0x074
#define   ESPI_FLASH_TX_CTRL_TRIGGER            BIT(31)
#define   ESPI_FLASH_TX_CTRL_LEN_MASK           GENMASK(23, 12)
#define   ESPI_FLASH_TX_CTRL_LEN_SHIFT          12
#define   ESPI_FLASH_TX_CTRL_TAG_MASK           GENMASK(11, 8)
#define   ESPI_FLASH_TX_CTRL_TAG_SHIFT          8
#define   ESPI_FLASH_TX_CTRL_CYC_MASK           GENMASK(7, 0)
#define   ESPI_FLASH_TX_CTRL_CYC_SHIFT          0
#define ESPI_FLASH_TX_PORT              0x078
#define ESPI_CTRL2                      0x080
#define   ESPI_CTRL2_MEMCYC_RD_DIS              BIT(6)
#define   ESPI_CTRL2_MEMCYC_WR_DIS              BIT(4)
#define ESPI_PERIF_PC_RX_SADDR          0x084
#define ESPI_PERIF_PC_RX_TADDR          0x088
#define ESPI_PERIF_PC_RX_MASK           0x08c
#define   ESPI_PERIF_PC_RX_MASK_CFG_WP          BIT(0)
#define ESPI_SYSEVT_INT_EN              0x094
#define ESPI_SYSEVT                     0x098
#define   ESPI_SYSEVT_HOST_RST_ACK              BIT(27)
#define   ESPI_SYSEVT_RST_CPU_INIT              BIT(26)
#define   ESPI_SYSEVT_SLV_BOOT_STS              BIT(23)
#define   ESPI_SYSEVT_NON_FATAL_ERR             BIT(22)
#define   ESPI_SYSEVT_FATAL_ERR                 BIT(21)
#define   ESPI_SYSEVT_SLV_BOOT_DONE             BIT(20)
#define   ESPI_SYSEVT_OOB_RST_ACK               BIT(16)
#define   ESPI_SYSEVT_NMI_OUT                   BIT(10)
#define   ESPI_SYSEVT_SMI_OUT                   BIT(9)
#define   ESPI_SYSEVT_HOST_RST_WARN             BIT(8)
#define   ESPI_SYSEVT_OOB_RST_WARN              BIT(6)
#define   ESPI_SYSEVT_PLTRSTN                   BIT(5)
#define   ESPI_SYSEVT_SUSPEND                   BIT(4)
#define   ESPI_SYSEVT_S5_SLEEP                  BIT(2)
#define   ESPI_SYSEVT_S4_SLEEP                  BIT(1)
#define   ESPI_SYSEVT_S3_SLEEP                  BIT(0)
#define ESPI_VW_GPIO_VAL                0x09c
#define ESPI_GEN_CAP_N_CONF             0x0a0
#define ESPI_CH0_CAP_N_CONF             0x0a4
#define ESPI_CH1_CAP_N_CONF             0x0a8
#define ESPI_CH2_CAP_N_CONF             0x0ac
#define ESPI_CH3_CAP_N_CONF             0x0b0
#define ESPI_CH3_CAP_N_CONF2            0x0b4
#define ESPI_SYSEVT1_INT_EN             0x100
#define ESPI_SYSEVT1                    0x104
#define   ESPI_SYSEVT1_SUSPEND_ACK              BIT(20)
#define   ESPI_SYSEVT1_SUSPEND_WARN             BIT(0)
#define ESPI_SYSEVT_INT_T0              0x110
#define ESPI_SYSEVT_INT_T1              0x114
#define ESPI_SYSEVT_INT_T2              0x118
#define   ESPI_SYSEVT_INT_T2_HOST_RST_WARN      ESPI_SYSEVT_HOST_RST_WARN
#define   ESPI_SYSEVT_INT_T2_OOB_RST_WARN       ESPI_SYSEVT_OOB_RST_WARN
#define ESPI_SYSEVT_INT_STS             0x11c
#define   ESPI_SYSEVT_INT_STS_NMI_OUT           ESPI_SYSEVT_NMI_OUT
#define   ESPI_SYSEVT_INT_STS_SMI_OUT           ESPI_SYSEVT_SMI_OUT
#define   ESPI_SYSEVT_INT_STS_HOST_RST_WARN     ESPI_SYSEVT_HOST_RST_WARN
#define   ESPI_SYSEVT_INT_STS_OOB_RST_WARN      ESPI_SYSEVT_OOB_RST_WARN
#define   ESPI_SYSEVT_INT_STS_PLTRSTN           ESPI_SYSEVT_PLTRSTN
#define   ESPI_SYSEVT_INT_STS_SUSPEND           ESPI_SYSEVT_SUSPEND
#define   ESPI_SYSEVT_INT_STS_S5_SLEEP          ESPI_SYSEVT_INT_S5_SLEEP
#define   ESPI_SYSEVT_INT_STS_S4_SLEEP          ESPI_SYSEVT_INT_S4_SLEEP
#define   ESPI_SYSEVT_INT_STS_S3_SLEEP          ESPI_SYSEVT_INT_S3_SLEEP
#define ESPI_SYSEVT1_INT_T0             0x120
#define ESPI_SYSEVT1_INT_T1             0x124
#define ESPI_SYSEVT1_INT_T2             0x128
#define ESPI_SYSEVT1_INT_STS            0x12c
#define   ESPI_SYSEVT1_INT_STS_SUSPEND_WARN     ESPI_SYSEVT1_SUSPEND_WARN
#define ESPI_OOB_RX_DMA_RB_SIZE         0x130
#define ESPI_OOB_RX_DMA_RD_PTR          0x134
#define   ESPI_OOB_RX_DMA_RD_PTR_UPDATE         BIT(31)
#define ESPI_OOB_RX_DMA_WS_PTR          0x138
#define   ESPI_OOB_RX_DMA_WS_PTR_RECV_EN        BIT(31)
#define   ESPI_OOB_RX_DMA_WS_PTR_SP_MASK        GENMASK(25, 16)
#define   ESPI_OOB_RX_DMA_WS_PTR_SP_SHIFT       16
#define   ESPI_OOB_RX_DMA_WS_PTR_WP_MASK        GENMASK(9, 0)
#define   ESPI_OOB_RX_DMA_WS_PTR_WP_SHIFT       0
#define ESPI_OOB_TX_DMA_RB_SIZE         0x140
#define ESPI_OOB_TX_DMA_RD_PTR          0x144
#define   ESPI_OOB_TX_DMA_RD_PTR_UPDATE         BIT(31)
#define ESPI_OOB_TX_DMA_WR_PTR          0x148
#define   ESPI_OOB_TX_DMA_WR_PTR_SEND_EN        BIT(31)

/* collect ESPI_INT_STS bits of eSPI channels for convenience */
#define ESPI_INT_STS_PERIF_BITS		  \
	(ESPI_INT_STS_PERIF_NP_TX_ABT	  \
	 | ESPI_INT_STS_PERIF_PC_TX_ABT	  \
	 | ESPI_INT_STS_PERIF_NP_RX_ABT	  \
	 | ESPI_INT_STS_PERIF_PC_RX_ABT	  \
	 | ESPI_INT_STS_PERIF_NP_TX_ERR	  \
	 | ESPI_INT_STS_PERIF_PC_TX_ERR	  \
	 | ESPI_INT_STS_PERIF_NP_TX_CMPLT \
	 | ESPI_INT_STS_PERIF_PC_TX_CMPLT \
	 | ESPI_INT_STS_PERIF_PC_RX_CMPLT)
#define ESPI_INT_STS_VW_BITS	   \
	(ESPI_INT_STS_VW_SYSEVT1   \
	 | ESPI_INT_STS_VW_GPIOEVT \
	 | ESPI_INT_STS_VW_SYSEVT)
#define ESPI_INT_STS_OOB_BITS	     \
	(ESPI_INT_STS_OOB_RX_TMOUT   \
	 | ESPI_INT_STS_OOB_TX_ERR   \
	 | ESPI_INT_STS_OOB_TX_ABT   \
	 | ESPI_INT_STS_OOB_RX_ABT   \
	 | ESPI_INT_STS_OOB_TX_CMPLT \
	 | ESPI_INT_STS_OOB_RX_CMPLT)
#define ESPI_INT_STS_FLASH_BITS	       \
	(ESPI_INT_STS_FLASH_TX_ERR     \
	 | ESPI_INT_STS_FLASH_TX_ABT   \
	 | ESPI_INT_STS_FLASH_RX_ABT   \
	 | ESPI_INT_STS_FLASH_TX_CMPLT \
	 | ESPI_INT_STS_FLASH_RX_CMPLT)

/* collect ESPI_INT_EN bits of eSPI channels for convenience */
#define ESPI_INT_EN_PERIF_BITS		 \
	(ESPI_INT_EN_PERIF_NP_TX_ABT	 \
	 | ESPI_INT_EN_PERIF_PC_TX_ABT	 \
	 | ESPI_INT_EN_PERIF_NP_RX_ABT	 \
	 | ESPI_INT_EN_PERIF_PC_RX_ABT	 \
	 | ESPI_INT_EN_PERIF_NP_TX_ERR	 \
	 | ESPI_INT_EN_PERIF_PC_TX_ERR	 \
	 | ESPI_INT_EN_PERIF_NP_TX_CMPLT \
	 | ESPI_INT_EN_PERIF_PC_TX_CMPLT \
	 | ESPI_INT_EN_PERIF_PC_RX_CMPLT)
#define ESPI_INT_EN_VW_BITS	  \
	(ESPI_INT_EN_VW_SYSEVT1	  \
	 | ESPI_INT_EN_VW_GPIOEVT \
	 | ESPI_INT_EN_VW_SYSEVT)
#define ESPI_INT_EN_OOB_BITS	    \
	(ESPI_INT_EN_OOB_RX_TMOUT   \
	 | ESPI_INT_EN_OOB_TX_ERR   \
	 | ESPI_INT_EN_OOB_TX_ABT   \
	 | ESPI_INT_EN_OOB_RX_ABT   \
	 | ESPI_INT_EN_OOB_TX_CMPLT \
	 | ESPI_INT_EN_OOB_RX_CMPLT)
#define ESPI_INT_EN_FLASH_BITS	      \
	(ESPI_INT_EN_FLASH_TX_ERR     \
	 | ESPI_INT_EN_FLASH_TX_ABT   \
	 | ESPI_INT_EN_FLASH_RX_ABT   \
	 | ESPI_INT_EN_FLASH_TX_CMPLT \
	 | ESPI_INT_EN_FLASH_RX_CMPLT)

struct espi_comm_hdr {
	uint8_t cyc;
	uint8_t len_h : 4;
	uint8_t tag : 4;
	uint8_t len_l;
};

static uint32_t espi_base;
#define ESPI_RD(reg)            sys_read32(espi_base + reg)
#define ESPI_WR(val, reg)       sys_write32((uint32_t)val, espi_base + reg)
#define ESPI_PLD_LEN_MAX        (1UL << 12)

/* peripheral channel */
static uint8_t perif_pc_rx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t perif_pc_tx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t perif_np_tx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t perif_mcyc_buf[DT_INST_PROP(0, perif_memcyc_size)]  __attribute__((aligned(DT_INST_PROP(0, perif_memcyc_size)))) NON_CACHED_BSS;

struct espi_aspeed_perif {
	uint8_t dma_mode;

	uint8_t *pc_rx_virt;
	uintptr_t pc_rx_addr;

	uint8_t *pc_tx_virt;
	uintptr_t pc_tx_addr;

	uint8_t *np_tx_virt;
	uintptr_t np_tx_addr;

	uint8_t *mcyc_virt;
	uint32_t mcyc_size;
	uintptr_t mcyc_saddr;
	uintptr_t mcyc_taddr;

	struct k_sem tx_lock;
	struct k_sem rx_lock;
};

static void espi_aspeed_perif_isr(uint32_t sts, struct espi_aspeed_perif *perif)
{
	if (sts & ESPI_INT_STS_PERIF_PC_RX_CMPLT) {
		k_sem_give(&perif->rx_lock);
	}

	ESPI_WR(sts & ESPI_INT_STS_PERIF_BITS, ESPI_INT_STS);
}

static void espi_aspeed_perif_init(struct espi_aspeed_perif *perif)
{
	uint32_t reg;

	ESPI_WR(perif->mcyc_saddr, ESPI_PERIF_PC_RX_SADDR);
	ESPI_WR(perif->mcyc_taddr, ESPI_PERIF_PC_RX_TADDR);

	reg = sys_read32(0x7e6e20d8);
	reg &= ~(0x1);
	sys_write32(reg, 0x7e6e20d8);

	reg = ESPI_RD(ESPI_CTRL2);
	reg &= ~(ESPI_CTRL2_MEMCYC_RD_DIS | ESPI_CTRL2_MEMCYC_WR_DIS);
	ESPI_WR(reg, ESPI_CTRL2);

	if (perif->dma_mode) {
		ESPI_WR(perif->pc_rx_addr, ESPI_PERIF_PC_RX_DMA);
		ESPI_WR(perif->pc_tx_addr, ESPI_PERIF_PC_TX_DMA);
		ESPI_WR(perif->np_tx_addr, ESPI_PERIF_NP_TX_DMA);

		reg = ESPI_RD(ESPI_CTRL) |
		      ESPI_CTRL_PERIF_NP_TX_DMA_EN |
		      ESPI_CTRL_PERIF_PC_TX_DMA_EN |
		      ESPI_CTRL_PERIF_PC_RX_DMA_EN;
		ESPI_WR(reg, ESPI_CTRL);
	}

	ESPI_WR(ESPI_INT_STS_PERIF_BITS, ESPI_INT_STS);

	reg = ESPI_RD(ESPI_INT_EN) | ESPI_INT_EN_PERIF_BITS;
	ESPI_WR(reg, ESPI_INT_EN);

	reg = ESPI_RD(ESPI_CTRL) | ESPI_CTRL_PERIF_SW_RDY;
	ESPI_WR(reg, ESPI_CTRL);
}

/* virtual wire channel */
struct espi_aspeed_vw {
	uint32_t gpio_sel;
	uint32_t gpio_val;
};

static void espi_aspeed_vw_isr(uint32_t sts, struct espi_aspeed_vw *vw)
{
	uint32_t evt, evt_int;

	if (sts & ESPI_INT_STS_VW_GPIOEVT) {
		ESPI_WR(ESPI_INT_STS_VW_GPIOEVT, ESPI_INT_STS);
	}

	if (sts & ESPI_INT_STS_VW_SYSEVT) {
		evt_int = ESPI_RD(ESPI_SYSEVT_INT_STS);
		ESPI_WR(evt_int, ESPI_SYSEVT_INT_STS);
		ESPI_WR(ESPI_INT_STS_VW_SYSEVT, ESPI_INT_STS);
	}

	if (sts & ESPI_INT_STS_VW_SYSEVT1) {
		evt = ESPI_RD(ESPI_SYSEVT1);
		evt_int = ESPI_RD(ESPI_SYSEVT1_INT_STS);

		if (evt_int & ESPI_SYSEVT1_INT_STS_SUSPEND_WARN) {
			evt |= ESPI_SYSEVT1_SUSPEND_ACK;
		}

		ESPI_WR(evt, ESPI_SYSEVT1);
		ESPI_WR(evt_int, ESPI_SYSEVT1_INT_STS);
		ESPI_WR(ESPI_INT_STS_VW_SYSEVT1, ESPI_INT_STS);
	}
}

static void espi_aspeed_vw_init(struct espi_aspeed_vw *vw)
{
	uint32_t reg;

	ESPI_WR(ESPI_INT_STS_VW_BITS, ESPI_INT_STS);

	reg = ESPI_RD(ESPI_INT_EN) | ESPI_INT_EN_VW_BITS;
	ESPI_WR(reg, ESPI_INT_EN);

	reg = ESPI_RD(ESPI_CTRL) | ESPI_CTRL_VW_SW_RDY;
	ESPI_WR(reg, ESPI_CTRL);
}

/* out of band channel */
#define OOB_TX_DMA_DESC_NUM     2
#define OOB_TX_DMA_BUF_SIZE     (OOB_TX_DMA_DESC_NUM * ESPI_PLD_LEN_MAX)
#define OOB_RX_DMA_DESC_NUM     4
#define OOB_RX_DMA_BUF_SIZE     (OOB_RX_DMA_DESC_NUM * ESPI_PLD_LEN_MAX)
#define OOB_DMA_UNLOCK          0x45535049
#define OOB_MSG                 0x21
#define OOB_TAG                 0x00

struct oob_tx_dma_desc {
	uint32_t data_addr;
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
} __attribute__((packed));

struct oob_rx_dma_desc {
	uint32_t data_addr;
	uint8_t cyc;
	uint16_t tag : 4;
	uint16_t len : 12;
	uint8_t raz : 7;
	uint8_t dirty : 1;
} __attribute__((packed));

static struct oob_tx_dma_desc oob_tx_desc[OOB_TX_DMA_DESC_NUM] NON_CACHED_BSS;
static struct oob_rx_dma_desc oob_rx_desc[OOB_RX_DMA_DESC_NUM] NON_CACHED_BSS;
static uint8_t oob_tx_buf[OOB_TX_DMA_BUF_SIZE] NON_CACHED_BSS;
static uint8_t oob_rx_buf[OOB_RX_DMA_BUF_SIZE] NON_CACHED_BSS;

struct espi_aspeed_oob {
	uint8_t dma_mode;

	struct oob_tx_dma_desc *tx_desc;
	uintptr_t tx_desc_addr;

	struct oob_rx_dma_desc *rx_desc;
	uintptr_t rx_desc_addr;

	uint8_t *tx_virt;
	uintptr_t tx_addr;

	uint8_t *rx_virt;
	uintptr_t rx_addr;

	struct k_sem tx_lock;
	struct k_sem rx_lock;
};

static void espi_aspeed_oob_isr(uint32_t sts, struct espi_aspeed_oob *oob)
{
	int i;
	uint32_t reg;

	if (sts & ESPI_INT_STS_HW_RST_DEASSERT) {
		if (oob->dma_mode) {
			reg = ESPI_RD(ESPI_CTRL) |
			      ESPI_CTRL_OOB_TX_DMA_EN |
			      ESPI_CTRL_OOB_RX_DMA_EN;
			ESPI_WR(reg, ESPI_CTRL);

			for (i = 0; i < OOB_RX_DMA_DESC_NUM; ++i)
				oob->rx_desc[i].dirty = 0;

			ESPI_WR(oob->tx_desc_addr, ESPI_OOB_TX_DMA);
			ESPI_WR(OOB_TX_DMA_DESC_NUM, ESPI_OOB_TX_DMA_RB_SIZE);
			ESPI_WR(OOB_DMA_UNLOCK, ESPI_OOB_TX_DMA_RD_PTR);
			ESPI_WR(0, ESPI_OOB_TX_DMA_WR_PTR);

			ESPI_WR(oob->rx_desc_addr, ESPI_OOB_RX_DMA);
			ESPI_WR(OOB_RX_DMA_DESC_NUM, ESPI_OOB_RX_DMA_RB_SIZE);
			ESPI_WR(OOB_DMA_UNLOCK, ESPI_OOB_RX_DMA_RD_PTR);
			ESPI_WR(0, ESPI_OOB_RX_DMA_WS_PTR);

			ESPI_WR(ESPI_OOB_RX_DMA_WS_PTR_RECV_EN, ESPI_OOB_RX_DMA_WS_PTR);
		}

		reg = ESPI_RD(ESPI_CTRL) | ESPI_CTRL_OOB_SW_RDY;
		ESPI_WR(reg, ESPI_CTRL);
	}

	if (sts & ESPI_INT_STS_OOB_TX_CMPLT) {
		k_sem_give(&oob->tx_lock);
	}

	if (sts & ESPI_INT_STS_OOB_RX_CMPLT) {
		k_sem_give(&oob->rx_lock);
	}

	ESPI_WR(sts & ESPI_INT_STS_OOB_BITS, ESPI_INT_STS);
}

static void espi_aspeed_oob_init(struct espi_aspeed_oob *oob)
{
	int i;
	uint32_t reg;

	if (oob->dma_mode) {
		reg = ESPI_RD(ESPI_CTRL) |
		      ESPI_CTRL_OOB_TX_DMA_EN |
		      ESPI_CTRL_OOB_RX_DMA_EN;
		ESPI_WR(reg, ESPI_CTRL);

		for (i = 0; i < OOB_TX_DMA_DESC_NUM; ++i)
			oob->tx_desc[i].data_addr = (uint32_t)(oob->tx_addr + (i * ESPI_PLD_LEN_MAX));

		for (i = 0; i < OOB_RX_DMA_DESC_NUM; ++i) {
			oob->rx_desc[i].data_addr = (uint32_t)(oob->rx_addr + (i * ESPI_PLD_LEN_MAX));
			oob->rx_desc[i].dirty = 0;
		}

		ESPI_WR(oob->tx_desc_addr, ESPI_OOB_TX_DMA);
		ESPI_WR(OOB_TX_DMA_DESC_NUM, ESPI_OOB_TX_DMA_RB_SIZE);
		ESPI_WR(OOB_DMA_UNLOCK, ESPI_OOB_TX_DMA_RD_PTR);
		ESPI_WR(0, ESPI_OOB_TX_DMA_WR_PTR);

		ESPI_WR(oob->rx_desc_addr, ESPI_OOB_RX_DMA);
		ESPI_WR(OOB_RX_DMA_DESC_NUM, ESPI_OOB_RX_DMA_RB_SIZE);
		ESPI_WR(OOB_DMA_UNLOCK, ESPI_OOB_RX_DMA_RD_PTR);
		ESPI_WR(0, ESPI_OOB_RX_DMA_WS_PTR);

		ESPI_WR(ESPI_OOB_RX_DMA_WS_PTR_RECV_EN, ESPI_OOB_RX_DMA_WS_PTR);
	}

	ESPI_WR(ESPI_INT_STS_OOB_BITS, ESPI_INT_STS);

	reg = ESPI_RD(ESPI_INT_EN) | ESPI_INT_EN_OOB_BITS;
	ESPI_WR(reg, ESPI_INT_EN);

	reg = ESPI_RD(ESPI_CTRL) | ESPI_CTRL_OOB_SW_RDY;
	ESPI_WR(reg, ESPI_CTRL);
}

/* flash channel */
#define FLASH_READ      0x00
#define FLASH_WRITE     0x01
#define FLASH_ERASE     0x02
#define FLASH_TAG       0x00

static uint8_t flash_tx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;
static uint8_t flash_rx_buf[ESPI_PLD_LEN_MAX] NON_CACHED_BSS;

struct espi_aspeed_flash {
	uint8_t dma_mode;
	uint8_t safs_mode;

	uint8_t *tx_virt;
	uintptr_t tx_addr;

	uint8_t *rx_virt;
	uintptr_t rx_addr;

	struct k_sem tx_lock;
	struct k_sem rx_lock;
};

static void espi_aspeed_flash_isr(uint32_t sts, struct espi_aspeed_flash *flash)
{
	if (sts & ESPI_INT_STS_FLASH_TX_CMPLT) {
		k_sem_give(&flash->tx_lock);
	}

	if (sts & ESPI_INT_STS_FLASH_RX_CMPLT) {
		k_sem_give(&flash->rx_lock);
	}

	ESPI_WR(sts & ESPI_INT_STS_FLASH_BITS, ESPI_INT_STS);
}

static void espi_aspeed_flash_init(struct espi_aspeed_flash *flash)
{
	uint32_t reg;

	reg = ESPI_RD(ESPI_CTRL);
	reg &= ~(ESPI_CTRL_FLASH_SW_MODE_MASK);
	reg |= ((flash->safs_mode << ESPI_CTRL_FLASH_SW_MODE_SHIFT) & ESPI_CTRL_FLASH_SW_MODE_MASK);
	ESPI_WR(reg, ESPI_CTRL);

	if (flash->dma_mode) {
		ESPI_WR(flash->tx_addr, ESPI_FLASH_TX_DMA);
		ESPI_WR(flash->rx_addr, ESPI_FLASH_RX_DMA);
		reg = ESPI_RD(ESPI_CTRL) | ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN;
		ESPI_WR(reg, ESPI_CTRL);
	}

	ESPI_WR(ESPI_INT_STS_FLASH_BITS, ESPI_INT_STS);

	reg = ESPI_RD(ESPI_INT_EN) | ESPI_INT_EN_FLASH_BITS;
	ESPI_WR(reg, ESPI_INT_EN);

	reg = ESPI_RD(ESPI_CTRL) | ESPI_CTRL_FLASH_SW_RDY;
	ESPI_WR(reg, ESPI_CTRL);
}

/* eSPI controller config. */
struct espi_aspeed_data {
	struct espi_aspeed_perif perif;
	struct espi_aspeed_vw vw;
	struct espi_aspeed_oob oob;
	struct espi_aspeed_flash flash;
};

static struct espi_aspeed_data espi_aspeed_data;

struct espi_aspeed_config {
	uintptr_t base;
};

static const struct espi_aspeed_config espi_aspeed_config = {
	.base = DT_INST_REG_ADDR(0),
};

static void espi_aspeed_ctrl_init(struct espi_aspeed_data *data)
{
	uint32_t reg;

	ESPI_WR(0x0, ESPI_SYSEVT_INT_T0);
	ESPI_WR(0x0, ESPI_SYSEVT_INT_T1);

	reg = ESPI_RD(ESPI_INT_EN) |
	      ESPI_INT_EN_HW_RST_DEASSERT |
	      ESPI_INT_EN_HW_RST_ASSERT;
	ESPI_WR(reg, ESPI_INT_EN);

	ESPI_WR(0xffffffff, ESPI_SYSEVT_INT_EN);

	ESPI_WR(0x1, ESPI_SYSEVT1_INT_EN);
	ESPI_WR(0x1, ESPI_SYSEVT1_INT_T0);
}

static void espi_aspeed_reset_isr(const struct device *dev)
{
	uint32_t sts;
	struct espi_aspeed_data *data = dev->data;

	espi_aspeed_ctrl_init(data);
	espi_aspeed_perif_init(&data->perif);
	espi_aspeed_vw_init(&data->vw);
	espi_aspeed_oob_init(&data->oob);
	espi_aspeed_flash_init(&data->flash);

	ESPI_WR(ESPI_INT_STS_HW_RST_ASSERT, ESPI_INT_STS);

	/* dummy read to make sure W1C arrives HW */
	sts = ESPI_RD(ESPI_INT_STS);
}

static void espi_aspeed_isr(const struct device *dev)
{
	uint32_t sts;
	uint32_t sysevt;
	struct espi_aspeed_data *data = (struct espi_aspeed_data *)dev->data;

	sts = ESPI_RD(ESPI_INT_STS);

	if (sts & ESPI_INT_STS_HW_RST_ASSERT) {
		return espi_aspeed_reset_isr(dev);
	}

	if (sts & ESPI_INT_STS_PERIF_BITS) {
		espi_aspeed_perif_isr(sts, &data->perif);
	}

	if (sts & ESPI_INT_STS_VW_BITS) {
		espi_aspeed_vw_isr(sts, &data->vw);
	}

	if (sts & (ESPI_INT_STS_OOB_BITS | ESPI_INT_STS_HW_RST_DEASSERT)) {
		espi_aspeed_oob_isr(sts, &data->oob);
	}

	if (sts & ESPI_INT_STS_FLASH_BITS) {
		espi_aspeed_flash_isr(sts, &data->flash);
	}

	if (sts & ESPI_INT_STS_HW_RST_DEASSERT) {
		sysevt = ESPI_RD(ESPI_SYSEVT) |
			 ESPI_SYSEVT_SLV_BOOT_STS |
			 ESPI_SYSEVT_SLV_BOOT_DONE;
		ESPI_WR(sysevt, ESPI_SYSEVT);
		ESPI_WR(ESPI_INT_STS_HW_RST_DEASSERT, ESPI_INT_STS);
	}

	/* dummy read to make sure W1C arrives HW */
	sts = ESPI_RD(ESPI_INT_STS);
}

static int espi_aspeed_init(const struct device *dev)
{
	uint32_t reg, scu_base;
	struct espi_aspeed_config *cfg = (struct espi_aspeed_config *)dev->config;
	struct espi_aspeed_data *data = (struct espi_aspeed_data *)dev->data;
	struct espi_aspeed_perif *perif = &data->perif;
	struct espi_aspeed_vw *vw = &data->vw;
	struct espi_aspeed_oob *oob = &data->oob;
	struct espi_aspeed_flash *flash = &data->flash;

	espi_base = cfg->base;
	scu_base = DT_REG_ADDR_BY_IDX(DT_INST_PHANDLE_BY_IDX(0, aspeed_scu, 0), 0);

	/* abort initialization if LPC mode is selected */
	reg = sys_read32(scu_base + SCU_HWSTRAP2);
	if (reg & SCU_HWSTRAP2_LPC) {
		return -ENODEV;
	}

	/* init private data */
	perif->dma_mode = DT_INST_PROP(0, perif_dma_mode);
	perif->pc_rx_virt = perif_pc_rx_buf;
	perif->pc_rx_addr = TO_PHY_ADDR(perif->pc_rx_virt);
	perif->pc_tx_virt = perif_pc_tx_buf;
	perif->pc_tx_addr = TO_PHY_ADDR(perif->pc_tx_virt);
	perif->np_tx_virt = perif_np_tx_buf;
	perif->np_tx_addr = TO_PHY_ADDR(perif->np_tx_virt);
	perif->mcyc_virt = perif_mcyc_buf;
	perif->mcyc_size = DT_INST_PROP(0, perif_memcyc_size);
	perif->mcyc_saddr = DT_INST_PROP(0, perif_memcyc_src_addr);
	perif->mcyc_taddr = TO_PHY_ADDR(perif->mcyc_virt);
	k_sem_init(&perif->tx_lock, 1, 1);
	k_sem_init(&perif->rx_lock, 0, 1);

	oob->dma_mode = DT_INST_PROP(0, oob_dma_mode);
	oob->tx_desc = oob_tx_desc;
	oob->tx_desc_addr = TO_PHY_ADDR(oob->tx_desc);
	oob->rx_desc = oob_rx_desc;
	oob->rx_desc_addr = TO_PHY_ADDR(oob->rx_desc);
	oob->tx_virt = oob_tx_buf;
	oob->tx_addr = TO_PHY_ADDR(oob->tx_virt);
	oob->rx_virt = oob_rx_buf;
	oob->rx_addr = TO_PHY_ADDR(oob->rx_virt);
	if (oob->dma_mode) {
		k_sem_init(&oob->tx_lock, OOB_TX_DMA_DESC_NUM, OOB_TX_DMA_DESC_NUM);
		k_sem_init(&oob->rx_lock, 0, OOB_RX_DMA_DESC_NUM);
	} else {
		k_sem_init(&oob->tx_lock, 1, 1);
		k_sem_init(&oob->rx_lock, 0, 1);
	}

	flash->dma_mode = DT_INST_PROP(0, flash_dma_mode);

	if (DT_NODE_HAS_PROP(DT_DRV_INST(0), flash_safs_mode)) {
		flash->safs_mode = DT_INST_PROP(0, flash_safs_mode);
	} else {
		flash->safs_mode = 0;
	}
	flash->tx_virt = flash_tx_buf;
	flash->tx_addr = TO_PHY_ADDR(flash->tx_virt);
	flash->rx_virt = flash_rx_buf;
	flash->rx_addr = TO_PHY_ADDR(flash->rx_virt);
	k_sem_init(&flash->tx_lock, 1, 1);
	k_sem_init(&flash->rx_lock, 0, 1);

	/* init controller */
	espi_aspeed_ctrl_init(data);
	espi_aspeed_perif_init(perif);
	espi_aspeed_vw_init(vw);
	espi_aspeed_oob_init(oob);
	espi_aspeed_flash_init(flash);

	/* install interrupt handler */
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    espi_aspeed_isr,
		    DEVICE_DT_INST_GET(0), 0);

	/* enable eSPI interrupt */
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

/* eSPI standard/specific APIs */
static bool espi_aspeed_channel_ready(const struct device *dev, enum espi_channel ch)
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

static int espi_aspeed_send_oob(const struct device *dev, struct espi_oob_packet *pckt)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	uint32_t rptr, wptr;
	struct oob_tx_dma_desc *d;
	struct espi_aspeed_data *data = (struct espi_aspeed_data *)dev->data;
	struct espi_aspeed_oob *oob = &data->oob;

	rc = k_sem_take(&oob->tx_lock, K_MSEC(100));
	if (rc == -EAGAIN) {
		return -ETIMEDOUT;
	}

	if (oob->dma_mode) {
		/* kick HW to reflect the up-to-date read/write pointer */
		ESPI_WR(ESPI_OOB_TX_DMA_RD_PTR_UPDATE, ESPI_OOB_TX_DMA_RD_PTR);

		rptr = ESPI_RD(ESPI_OOB_TX_DMA_RD_PTR);
		wptr = ESPI_RD(ESPI_OOB_TX_DMA_WR_PTR);

		if (((wptr + 1) % OOB_TX_DMA_DESC_NUM) == rptr) {
			return -EFAULT;
		}

		d = &oob->tx_desc[wptr];
		d->cyc = OOB_MSG;
		d->tag = OOB_TAG;
		d->len = pckt->len;
		d->msg_type = 0x4; /* ASPEED Customized OOB TX DMA descriptor */

		memcpy(oob->tx_virt + (ESPI_PLD_LEN_MAX * wptr), pckt->buf, pckt->len);

		wptr = (wptr + 1) % OOB_TX_DMA_DESC_NUM;
		wptr |= ESPI_OOB_TX_DMA_WR_PTR_SEND_EN;
		ESPI_WR(wptr, ESPI_OOB_TX_DMA_WR_PTR);
	} else {
		reg = ESPI_RD(ESPI_OOB_TX_CTRL);
		if (reg & ESPI_OOB_TX_CTRL_TRIGGER) {
			return -EFAULT;
		}

		for (i = 0; i < pckt->len; ++i)
			ESPI_WR(pckt->buf[i], ESPI_OOB_TX_PORT);

		cyc = OOB_MSG;
		tag = OOB_TAG;
		len = pckt->len;

		reg = ((cyc << ESPI_OOB_TX_CTRL_CYC_SHIFT) & ESPI_OOB_TX_CTRL_CYC_MASK) |
		      ((tag << ESPI_OOB_TX_CTRL_TAG_SHIFT) & ESPI_OOB_TX_CTRL_TAG_MASK) |
		      ((len << ESPI_OOB_TX_CTRL_LEN_SHIFT) & ESPI_OOB_TX_CTRL_LEN_MASK) |
		      ESPI_OOB_TX_CTRL_TRIGGER;

		ESPI_WR(reg, ESPI_OOB_TX_CTRL);
	}

	return 0;
}

static int espi_aspeed_receive_oob(const struct device *dev, struct espi_oob_packet *pckt)
{
	int i, rc;
	uint32_t reg, len;
	uint32_t wptr, sptr;
	struct espi_aspeed_data *data = (struct espi_aspeed_data *)dev->data;
	struct espi_aspeed_oob *oob = &data->oob;
	struct oob_rx_dma_desc *d;

	rc = k_sem_take(&oob->rx_lock, K_NO_WAIT);
	if (rc) {
		return rc;
	}

	if (oob->dma_mode) {
		reg = ESPI_RD(ESPI_OOB_RX_DMA_WS_PTR);
		wptr = (reg & ESPI_OOB_RX_DMA_WS_PTR_WP_MASK) >> ESPI_OOB_RX_DMA_WS_PTR_WP_SHIFT;
		sptr = (reg & ESPI_OOB_RX_DMA_WS_PTR_SP_MASK) >> ESPI_OOB_RX_DMA_WS_PTR_SP_SHIFT;

		d = &oob->rx_desc[sptr];
		if (!d->dirty) {
			return -EFAULT;
		}

		pckt->len = (d->len) ? d->len : ESPI_PLD_LEN_MAX;
		memcpy(pckt->buf, oob->rx_virt + (ESPI_PLD_LEN_MAX * sptr), pckt->len);

		d->dirty = 0;
		sptr = (sptr + 1) % OOB_RX_DMA_DESC_NUM;
		wptr = (wptr + 1) % OOB_RX_DMA_DESC_NUM;

		reg = ((wptr << ESPI_OOB_RX_DMA_WS_PTR_WP_SHIFT) & ESPI_OOB_RX_DMA_WS_PTR_WP_MASK) |
		      ((sptr << ESPI_OOB_RX_DMA_WS_PTR_SP_SHIFT) & ESPI_OOB_RX_DMA_WS_PTR_SP_MASK) |
		      ESPI_OOB_RX_DMA_WS_PTR_RECV_EN;
		ESPI_WR(reg, ESPI_OOB_RX_DMA_WS_PTR);
	} else {
		reg = ESPI_RD(ESPI_OOB_RX_CTRL);
		len = (reg & ESPI_OOB_RX_CTRL_LEN_MASK) >> ESPI_OOB_RX_CTRL_LEN_SHIFT;

		pckt->len = (len) ? len : ESPI_PLD_LEN_MAX;
		for (i = 0; i < pckt->len; ++i)
			pckt->buf[i] = (uint8_t)(ESPI_RD(ESPI_OOB_TX_PORT) & 0xff);

		ESPI_WR(ESPI_OOB_RX_CTRL_PEND_SERV, ESPI_OOB_RX_CTRL);
	}

	return 0;
}

static int espi_aspeed_flash_read(const struct device *dev, struct espi_flash_packet *pckt)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_aspeed_data *data = (struct espi_aspeed_data *)dev->data;
	struct espi_aspeed_flash *flash = &data->flash;

	rc = k_sem_take(&flash->tx_lock, K_MSEC(100));
	if (rc == -EAGAIN) {
		return -ETIMEDOUT;
	}

	if (flash->dma_mode) {
		flash->tx_virt[0] = (pckt->flash_addr >> 24) & 0xff;
		flash->tx_virt[1] = (pckt->flash_addr >> 16) & 0xff;
		flash->tx_virt[2] = (pckt->flash_addr >> 8) & 0xff;
		flash->tx_virt[3] = (pckt->flash_addr >> 0) & 0xff;
		memcpy(flash->tx_virt + sizeof(pckt->flash_addr), pckt->buf, pckt->len);
	} else   {
		ESPI_WR((pckt->flash_addr >> 24) & 0xff, ESPI_FLASH_TX_PORT);
		ESPI_WR((pckt->flash_addr >> 16) & 0xff, ESPI_FLASH_TX_PORT);
		ESPI_WR((pckt->flash_addr >> 8) & 0xff, ESPI_FLASH_TX_PORT);
		ESPI_WR((pckt->flash_addr >> 0) & 0xff, ESPI_FLASH_TX_PORT);
		for (i = 0; i < pckt->len; ++i)
			ESPI_WR(pckt->buf[i], ESPI_FLASH_TX_PORT);
	}

	cyc = FLASH_READ;
	tag = FLASH_TAG;
	len = sizeof(pckt->flash_addr) + pckt->len;

	reg = ((cyc << ESPI_FLASH_TX_CTRL_CYC_SHIFT) & ESPI_FLASH_TX_CTRL_CYC_MASK) |
	      ((tag << ESPI_FLASH_TX_CTRL_TAG_SHIFT) & ESPI_FLASH_TX_CTRL_TAG_MASK) |
	      ((len << ESPI_FLASH_TX_CTRL_LEN_SHIFT) & ESPI_FLASH_TX_CTRL_LEN_MASK) |
	      ESPI_FLASH_TX_CTRL_TRIGGER;
	ESPI_WR(reg, ESPI_FLASH_TX_CTRL);

	return 0;
}

static int espi_aspeed_flash_write(const struct device *dev, struct espi_flash_packet *pckt)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_aspeed_data *data = (struct espi_aspeed_data *)dev->data;
	struct espi_aspeed_flash *flash = &data->flash;

	rc = k_sem_take(&flash->tx_lock, K_MSEC(100));
	if (rc == -EAGAIN) {
		return -ETIMEDOUT;
	}

	if (flash->dma_mode) {
		flash->tx_virt[0] = (pckt->flash_addr >> 24) & 0xff;
		flash->tx_virt[1] = (pckt->flash_addr >> 16) & 0xff;
		flash->tx_virt[2] = (pckt->flash_addr >> 8) & 0xff;
		flash->tx_virt[3] = (pckt->flash_addr >> 0) & 0xff;
		memcpy(flash->tx_virt + sizeof(pckt->flash_addr), pckt->buf, pckt->len);
	} else   {
		ESPI_WR((pckt->flash_addr >> 24) & 0xff, ESPI_FLASH_TX_PORT);
		ESPI_WR((pckt->flash_addr >> 16) & 0xff, ESPI_FLASH_TX_PORT);
		ESPI_WR((pckt->flash_addr >> 8) & 0xff, ESPI_FLASH_TX_PORT);
		ESPI_WR((pckt->flash_addr >> 0) & 0xff, ESPI_FLASH_TX_PORT);
		for (i = 0; i < pckt->len; ++i)
			ESPI_WR(pckt->buf[i], ESPI_FLASH_TX_PORT);
	}

	cyc = FLASH_WRITE;
	tag = FLASH_TAG;
	len = sizeof(pckt->flash_addr) + pckt->len;


	reg = ((cyc << ESPI_FLASH_TX_CTRL_CYC_SHIFT) & ESPI_FLASH_TX_CTRL_CYC_MASK) |
	      ((tag << ESPI_FLASH_TX_CTRL_TAG_SHIFT) & ESPI_FLASH_TX_CTRL_TAG_MASK) |
	      ((len << ESPI_FLASH_TX_CTRL_LEN_SHIFT) & ESPI_FLASH_TX_CTRL_LEN_MASK) |
	      ESPI_FLASH_TX_CTRL_TRIGGER;
	ESPI_WR(reg, ESPI_FLASH_TX_CTRL);

	return 0;
}

static int espi_aspeed_flash_erase(const struct device *dev, struct espi_flash_packet *pckt)
{
	int i, rc;
	uint32_t reg;
	uint32_t cyc, tag, len;
	struct espi_aspeed_data *data = (struct espi_aspeed_data *)dev->data;
	struct espi_aspeed_flash *flash = &data->flash;

	rc = k_sem_take(&flash->tx_lock, K_MSEC(100));
	if (rc == -EAGAIN) {
		return -ETIMEDOUT;
	}

	if (flash->dma_mode) {
		flash->tx_virt[0] = (pckt->flash_addr >> 24) & 0xff;
		flash->tx_virt[1] = (pckt->flash_addr >> 16) & 0xff;
		flash->tx_virt[2] = (pckt->flash_addr >> 8) & 0xff;
		flash->tx_virt[3] = (pckt->flash_addr >> 0) & 0xff;
		memcpy(flash->tx_virt + sizeof(pckt->flash_addr), pckt->buf, pckt->len);
	} else   {
		ESPI_WR((pckt->flash_addr >> 24) & 0xff, ESPI_FLASH_TX_PORT);
		ESPI_WR((pckt->flash_addr >> 16) & 0xff, ESPI_FLASH_TX_PORT);
		ESPI_WR((pckt->flash_addr >> 8) & 0xff, ESPI_FLASH_TX_PORT);
		ESPI_WR((pckt->flash_addr >> 0) & 0xff, ESPI_FLASH_TX_PORT);
		for (i = 0; i < pckt->len; ++i)
			ESPI_WR(pckt->buf[i], ESPI_FLASH_TX_PORT);
	}

	cyc = FLASH_ERASE;
	tag = FLASH_TAG;
	len = sizeof(pckt->flash_addr) + pckt->len;

	reg = ((cyc << ESPI_FLASH_TX_CTRL_CYC_SHIFT) & ESPI_FLASH_TX_CTRL_CYC_MASK) |
	      ((tag << ESPI_FLASH_TX_CTRL_TAG_SHIFT) & ESPI_FLASH_TX_CTRL_TAG_MASK) |
	      ((len << ESPI_FLASH_TX_CTRL_LEN_SHIFT) & ESPI_FLASH_TX_CTRL_LEN_MASK) |
	      ESPI_FLASH_TX_CTRL_TRIGGER;
	ESPI_WR(reg, ESPI_FLASH_TX_CTRL);

	return 0;
}

static const struct espi_driver_api espi_aspeed_driver_api = {
	.get_channel_status = espi_aspeed_channel_ready,
	.send_oob = espi_aspeed_send_oob,
	.receive_oob = espi_aspeed_receive_oob,
	.flash_read = espi_aspeed_flash_read,
	.flash_write = espi_aspeed_flash_write,
	.flash_erase = espi_aspeed_flash_erase,
};

DEVICE_DT_INST_DEFINE(0, &espi_aspeed_init, NULL,
		      &espi_aspeed_data, &espi_aspeed_config,
		      PRE_KERNEL_2, CONFIG_ESPI_INIT_PRIORITY,
		      &espi_aspeed_driver_api);
