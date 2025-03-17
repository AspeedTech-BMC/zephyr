/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_ast27xx_ltpi

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/ltpi.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log.h>
#define LOG_MODULE_NAME			ltpi_ast27xx
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_LTPI_LOG_LEVEL);

#define BOOTSTAGE_LTPI_INIT			"L "
#define ADVERTISE_TIMEOUT_US			105000 /* 105 ms */

struct bootstage_t {
	uint8_t errno;
	uint8_t syndrome;
	union {
		uint16_t boot2fmc;
		uint16_t panic;
	};
};

#define LTPI_SP_CAP_ASPEED_SUPPORTED                              \
	(LTPI_SP_CAP_25M | LTPI_SP_CAP_50M | LTPI_SP_CAP_75M |    \
	 LTPI_SP_CAP_100M | LTPI_SP_CAP_150M | LTPI_SP_CAP_200M | \
	 LTPI_SP_CAP_250M | LTPI_SP_CAP_300M | LTPI_SP_CAP_400M | \
	 LTPI_SP_CAP_600M | LTPI_SP_CAP_500M | LTPI_SP_CAP_DDR)

/* bootstage_t->errno */
#define LTPI_STATUS_EXIT			BIT(7)	/* 1: exit due to errors */
#define LTPI_STATUS_RESTART			BIT(6)	/* 1: restart LTPI initialization */

#define LTPI_STATUS_HAS_CRC_ERR			BIT(4)
#define LTPI_STATUS_MODE			GENMASK(3, 2)	/* PHY mode */
#define LTPI_STATUS_IDX				BIT(1)	/* 1: LTPI1 */
#define LTPI_STATUS_HPM				BIT(0)	/* 1: LTPI controller is on HPM */

/* bootstage_t->syndrome */
#define LTPI_SYND_OK				0
#define LTPI_SYND_OK_ALREADY_INIT		1
#define LTPI_SYND_NO_COMMOM_SPEED		2
#define LTPI_SYND_WAIT_OP_TO			3
#define LTPI_SYND_EXTRST_LINK_TRAINING		4 /* EXTRST deasserted during link training */
#define LTPI_SYND_EXTRST_LINK_CONFIG		5 /* EXTRST deasserted during link configuration */
#define LTPI_SYND_SOC_RECOVERY			6 /* Exit due to SOC recovery */

/* LTPI wait state return code */
#define LTPI_ERR_NONE				0x00
#define LTPI_ERR_TIMEOUT			0x10
#define LTPI_ERR_DISCON				0x20

/* LTPI PHY control registers */
#define LTPI_PHY_CTRL				0x000
#define   REG_LTPI_PHY_MODE			GENMASK(3, 0)
#define     LTPI_PHY_MODE_CDR_HI_SP		0b1000
#define     LTPI_PHY_MODE_CDR_LO_SP		0b0100
#define     LTPI_PHY_MODE_DDR			0b0010
#define     LTPI_PHY_MODE_SDR			0b0001
#define     LTPI_PHY_MODE_OFF			0b0000
#define LTPI_PLL_CTRL				0x004
#define   REG_LTPI_RX_PHY_CLK_INV		BIT(9)
#define   REG_LTPI_TX_PHY_CLK_INV		BIT(8)
#define   REG_LTPI_PLL_SET			BIT(4)
#define   REG_LTPI_RX_PLL_DIV2			BIT(3)
#define   REG_LTPI_PLL_SELECT			GENMASK(2, 0)
#define     REG_LTPI_PLL_25M			0
#define     REG_LTPI_PLL_LPLL			7

#define LTPI_PHY_ALIGN_CTRL			0x008
#define LTPI_DLL_CTRL				0x00C
#define   REG_LTPI_SW_DLL_RST			BIT(26)
#define   REG_LTPI_FORCE_DLL_RST		BIT(25)
#define   REG_LTPI_DLL_PD			BIT(24)
#define   REG_LTPI_DLL_TSTCTRL			GENMASK(23, 16)
#define   REG_LTPI_DLL_RST_TIMEOUT_A0		GENMASK(15, 0)
#define   REG_LTPI_DLL_CLK_2X			BIT(8)
#define   REG_LTPI_DLL_RST_TIMEOUT		GENMASK(7, 0)
#define LTPI_PHY_HI_SP_CDR_CTRL			0x010
#define   REG_LTPI_SW_HI_SP_CDR_EN		BIT(10)
#define   REG_LTPI_FORCE_HI_SP_CDR_EN		BIT(9)
#define   REG_LTPI_HI_SP_CDR_EN_TIMEOUT_EN	BIT(8)
#define	  LTPI_HI_SP_CDR_EN_TIMEOUT		GENMASK(7, 0)

#define LTPI_PROT_KEY				0x34
#define   LTPI_PROT_KEY_UNLOCK			0x1728aacc

/* LVDS TOP registers */
#define LTPI_LVDS_TX_CTRL			0x000
#define   REG_LTPI_LVDS_TX1_DS1			BIT(22)
#define   REG_LTPI_LVDS_TX1_DS0			BIT(21)
#define   REG_LTPI_LVDS_TX1_IPREE		BIT(20)
#define   REG_LTPI_LVDS_TX1_IPREE_EN		BIT(19)
#define   REG_LTPI_LVDS_TX1_PD			BIT(18)
#define   REG_LTPI_LVDS_TX1_PU			BIT(17)
#define   REG_LTPI_LVDS_TX1_OE			BIT(16)
#define   REG_LTPI_LVDS_TX0_DS1			BIT(6)
#define   REG_LTPI_LVDS_TX0_DS0			BIT(5)
#define   REG_LTPI_LVDS_TX0_IPREE		BIT(4)
#define   REG_LTPI_LVDS_TX0_IPREE_EN		BIT(3)
#define   REG_LTPI_LVDS_TX0_PD			BIT(2)
#define   REG_LTPI_LVDS_TX0_PU			BIT(1)
#define   REG_LTPI_LVDS_TX0_OE			BIT(0)
#define LTPI_LVDS_RX_CTRL			0x004
#define   REG_LTPI_LVDS_RX1_BIAS_EN		BIT(18)
#define   REG_LTPI_LVDS_RX1_ST			BIT(17)
#define   REG_LTPI_LVDS_RX1_IE			BIT(16)
#define   REG_LTPI_LVDS_RX0_BIAS_EN		BIT(2)
#define   REG_LTPI_LVDS_RX0_ST			BIT(1)
#define   REG_LTPI_LVDS_RX0_IE			BIT(0)
#define LTPI_SW_RST				0x008
#define   REG_LTPI_ALL_SW_RST			BIT(31)
#define   REG_LTPI1_SW_RST			BIT(17)
#define   REG_LTPI0_SW_RST			BIT(16)
#define   REG_LTPI_DLL_CTRL_SW_RST		BIT(9)
#define   REG_LTPI_REF_SW_RST			BIT(8)
#define   REG_LTPI1_RX_PHY_SW_RST		BIT(7)
#define   REG_LTPI1_TX_PHY_SW_RST		BIT(6)
#define   REG_LTPI1_RX_MAC_SW_RST		BIT(5)
#define   REG_LTPI1_TX_MAC_SW_RST		BIT(4)
#define   REG_LTPI0_RX_PHY_SW_RST		BIT(3)
#define   REG_LTPI0_TX_PHY_SW_RST		BIT(2)
#define   REG_LTPI0_RX_MAC_SW_RST		BIT(1)
#define   REG_LTPI0_TX_MAC_SW_RST		BIT(0)
#define LTPI_STRAP_VAL				0x00c
#define   REG_LTPI_STRAP_2LTPI_EN		BIT(1)
#define   REG_LTPI_STRAP_1700_EN		BIT(0)
#define LTPI_SW_FORCE_EN			0x010
#define LTPI_SW_FORCE_VAL			0x014
#define   REG_LTPI_SW_FORCE_LVDS_TX_DS_EN	BIT(2)
#define   REG_LTPI_SW_FORCE_2LTPI_EN		BIT(1)
#define   REG_LTPI_SW_FORCE_1700_EN		BIT(0)

/* LTPI control registers */
#define LTPI_INTR				0x100
#define   REG_LTPI_LINK_HALT			BIT(24)
#define   REG_LTPI_DATA_CH_DROP_FRAME		BIT(23)
#define   REG_LTPI_DATA_CH_INVALID_ACCESS	BIT(22)
#define   REG_LTPI_I2C5_TO_ERR			BIT(21)
#define   REG_LTPI_I2C4_TO_ERR			BIT(20)
#define   REG_LTPI_I2C3_TO_ERR			BIT(19)
#define   REG_LTPI_I2C2_TO_ERR			BIT(18)
#define   REG_LTPI_I2C1_TO_ERR			BIT(17)
#define   REG_LTPI_I2C0_TO_ERR			BIT(16)
#define   REG_LTPI_AD_ALIGN_TO			BIT(12)
#define   REG_LTPI_HW_RETRY_LINK_DET_STUCK_TO_ERR BIT(11)
#define   REG_LTPI_HW_RETRY_TO_ERR		BIT(10)
#define   REG_LTPI_OP_LINK_LOST_ERR		BIT(4)
#define   REG_LTPI_CON_OR_ACC_LINK_LOST_ERR	BIT(3)
#define   REG_LTPI_AD_LINK_LOST_ERR		BIT(2)
#define   REG_LTPI_CON_READY			BIT(1)
#define   REG_LTPI_AD_READY			BIT(0)
#define LTPI_INTR_EN				0x104
#define   REG_LTPI_LINK_HALT_INTR_EN		BIT(24)
#define   REG_LTPI_DATA_CH_DROP_FRAME_EN	BIT(23)
#define   REG_LTPI_DATA_CH_INVALID_ACCESS_EN	BIT(22)
#define   REG_LTPI_I2C5_TO_ERR_EN		BIT(21)
#define   REG_LTPI_I2C4_TO_ERR_EN		BIT(20)
#define   REG_LTPI_I2C3_TO_ERR_EN		BIT(19)
#define   REG_LTPI_I2C2_TO_ERR_EN		BIT(18)
#define   REG_LTPI_I2C1_TO_ERR_EN		BIT(17)
#define   REG_LTPI_I2C0_TO_ERR_EN		BIT(16)
#define   REG_LTPI_AD_ALIGN_TO_EN		BIT(12)
#define   REG_LTPI_HW_RETRY_LINK_DET_STUCK_TO_ERR_EN BIT(11)
#define   REG_LTPI_HW_RETRY_TO_ERR_EN		BIT(10)
#define   REG_LTPI_CON_ACC_TO_ERR_EN		BIT(9)
#define   REG_LTPI_LINK_SP_TO_ERR_EN		BIT(8)
#define   REG_LTPI_UNKNOWN_COMMA_ERR_EN		BIT(7)
#define   REG_LTPI_FRM_CRC_ERR_EN		BIT(6)
#define   REG_LTPI_LINK_LOST_ERR_EN		BIT(5)
#define   REG_LTPI_OP_LINK_LOST_ERR_EN		BIT(4)
#define   REG_LTPI_CON_OR_ACC_LINK_LOST_ERR_EN	BIT(3)
#define   REG_LTPI_AD_LINK_LOST_ERR_EN		BIT(2)
#define   REG_LTPI_CON_READY_EN			BIT(1)
#define   REG_LTPI_AD_READY_EN			BIT(0)
#define LTPI_LINK_MNG_ST			0x108
#define   REG_LTPI_LINK_PARTNER_FLAG		BIT(24)
#define     REG_LTPI_LINK_PARTNER_FPGA		0b0
#define     REG_LTPI_LINK_PARTNER_1700		0b1
#define   REG_LTPI_SP_INTERSETION		GENMASK(23, 8)
#define   REG_LTPI_LINK_MNG_ST			GENMASK(3, 0)
#define     LTPI_LINK_MNG_ST_DETECT_ALIGN	0
#define     LTPI_LINK_MNG_ST_DETECT		1
#define     LTPI_LINK_MNG_ST_SPEED		2
#define     LTPI_LINK_MNG_ST_WAIT_PLL_SET	3
#define     LTPI_LINK_MNG_ST_ADV_ALIGN		4
#define     LTPI_LINK_MNG_ST_ADV		5
#define     LTPI_LINK_MNG_ST_CONFIG_ACC		6
#define     LTPI_LINK_MNG_ST_OP			7
#define LTPI_LINK_MANAGE_CTRL0			0x10C
#define   REG_LTPI_LINK_PARTNER_CHKNUM		GENMASK(19, 16)
#define   REG_LTPI_CON_FRM_NOCHK		BIT(8)
#define   REG_LTPI_TX_LINK_SP_FRM_NUM		GENMASK(7, 4)
#define   REG_LTPI_RX_LINK_SP_FRM_NUM		GENMASK(3, 0)
#define LTPI_LINK_MANAGE_CTRL1			0x110
#define LTPI_LINK_MANAGE_CTRL2			0x114
#define LTPI_CON_CAP_LOW			0x118
#define LTPI_CON_CAP_HIGH			0x11C
#define LTPI_MAC_CTRL				0x120
#define LTPI_AHB_CTRL0				0x124
#define   REG_LTPI_AHB_ADDR_MAP1		GENMASK(25, 16)
#define   REG_LTPI_AHB_ADDR_MAP0		GENMASK(9, 0)
#define LTPI_AHB_CTRL1				0x128
#define   REG_LTPI_AHB_ADDR_MAP3		GENMASK(25, 16)
#define   REG_LTPI_AHB_ADDR_MAP2		GENMASK(9, 0)
#define LTPI_CRC_OPTION				0x12c
#define   REG_LTPI_SW_CRC_OUT_ML_FIRST		BIT(2)
#define   REG_LTPI_SW_CRC_IN_LSB_FIRST		BIT(1)
#define   REG_LTPI_CRC_SW_FORCE			BIT(0)
#define LTPI_I2C				0x130
#define   REG_LTPI_I2C_SLV_MST_SWITCH_SW_EN	BIT(8)
#define   REG_LTPI_I2C_SLV_EN			GENMASK(5, 0)

#define LTPI_OEM_BUS_SETTING			0x188
#define   REG_LTPI_OEM_RX_START_TRIG		BIT(1)
#define   REG_LTPI_OEM_TX_START_TRIG		BIT(0)

#define LTPI_OEM_DBG0				0x190
#define   REG_LTPI_OEM_RX_INIT_DONE		BIT(9)
#define   REG_LTPI_OEM_TX_INIT_DONE		BIT(8)

static const int16_t ltpi_clk_lookup_sdr[13] = {
	25, 50, 75, 100, 150, 200, 250, 300, 400, 600, -1, -1, 50,
};

/* AST2700 IO-die PLL control */
/* SCU registers */
#define SCU1_L0PLL_1				0x340
#define SCU1_L1PLL_1				0x350
#define   PLL_REG1_RESET			BIT(25)
#define   PLL_REG1_BYPASS			BIT(24)
#define   PLL_REG1_DIS				BIT(23)
#define   PLL_REG1_P				GENMASK(22, 19)
#define   PLL_REG1_N				GENMASK(18, 13)
#define   PLL_REG1_M				GENMASK(12, 0)

#define SCU1_L0PLL_2				0x344
#define SCU1_L1PLL_2				0x354
#define   PLL_REG2_LOCK				BIT(31)
#define   PLL_REG2_BWADJ			GENMASK(11, 0)

#define NUM_PLL_PARAM				13
#define REG_N_M_P(n, m, p)			((((n) - 1) << 13) | ((m) - 1) | (((p) - 1) << 19))
#define REG_BWADJ(bwadj)			((bwadj) - 1)

#define SCU_PLL_ID_IO_L0PLL			0
#define SCU_PLL_ID_IO_L1PLL			1
#define SCU_PLL_ID_MAX				SCU_PLL_ID_IO_L1PLL

struct pll_info {
	mm_reg_t reg_offset0;
	mm_reg_t reg_offset1;
};

struct pll_param {
	int freq;
	uint32_t n_m_p;
	uint16_t bwadj;
};

static const struct pll_info scu_pll_info[SCU_PLL_ID_MAX + 1] = {
	[SCU_PLL_ID_IO_L0PLL] = { .reg_offset0 = SCU1_L0PLL_1, .reg_offset1 = SCU1_L0PLL_2 },
	[SCU_PLL_ID_IO_L1PLL] = { .reg_offset0 = SCU1_L1PLL_1, .reg_offset1 = SCU1_L1PLL_2 },
};

static const struct pll_param pll_param_lookup[NUM_PLL_PARAM] = {
	{ .freq = MHZ(50), .n_m_p = REG_N_M_P(1, 32, 16), .bwadj = REG_BWADJ(16) },
	{ .freq = MHZ(75), .n_m_p = REG_N_M_P(1, 48, 16), .bwadj = REG_BWADJ(24) },
	{ .freq = MHZ(100), .n_m_p = REG_N_M_P(1, 56, 14), .bwadj = REG_BWADJ(28) },
	{ .freq = MHZ(150), .n_m_p = REG_N_M_P(1, 60, 10), .bwadj = REG_BWADJ(30) },
	{ .freq = MHZ(200), .n_m_p = REG_N_M_P(1, 48, 6), .bwadj = REG_BWADJ(24) },
	{ .freq = MHZ(250), .n_m_p = REG_N_M_P(1, 60, 6), .bwadj = REG_BWADJ(30) },
	{ .freq = MHZ(300), .n_m_p = REG_N_M_P(1, 48, 4), .bwadj = REG_BWADJ(24) },
	{ .freq = MHZ(400), .n_m_p = REG_N_M_P(1, 32, 2), .bwadj = REG_BWADJ(16) },
	{ .freq = MHZ(500), .n_m_p = REG_N_M_P(1, 40, 2), .bwadj = REG_BWADJ(20) },
	{ .freq = MHZ(600), .n_m_p = REG_N_M_P(1, 48, 2), .bwadj = REG_BWADJ(24) },
	{ .freq = MHZ(800), .n_m_p = REG_N_M_P(1, 32, 1), .bwadj = REG_BWADJ(16) },
	{ .freq = MHZ(1000), .n_m_p = REG_N_M_P(1, 40, 1), .bwadj = REG_BWADJ(20) },
	{ .freq = MHZ(1200), .n_m_p = REG_N_M_P(1, 48, 1), .bwadj = REG_BWADJ(24) },
};

struct ast27xx_ltpi_config {
	mm_reg_t base;
	mm_reg_t phy_base;
	mm_reg_t top_base;
	mm_reg_t scu_base;
	int index;
	struct reset_dt_spec reset;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_ahb;
	const clock_control_subsys_t clock_phy;
};

struct ast27xx_ltpi_data {
	const struct ast27xx_ltpi_config *config;
	struct ltpi_ctrl_status_regs *ctrl_status_reg;

	/* encoding as LTPI speed capability */
	uint16_t otp_speed_cap;	/* limit the speed via OTP strap */
	uint16_t phy_speed_cap; /* limit the speed with physical line status */
	bool otp_ddr_dis;

	int crc_format;
	int io_driving;

#define RX_CLK_INVERSE		BIT(1)
#define TX_CLK_INVERSE		BIT(0)
	int clk_inverse;

	int link_speed_frm_rx_cnt;
	struct bootstage_t *bootstage;

	/* Advertise timeout in us */
	int ad_timeout;

	/* Start time to listen LINK_DETECT frame */
	uint64_t t_link_detect;

	/* Timeout to get operational state, in tick */
	uint64_t op_timeout;
};

static void setbits_le32(mm_reg_t addr, uint32_t set)
{
	sys_write32(sys_read32(addr) | set, addr);
}

static void clrbits_le32(mm_reg_t addr, uint32_t clr)
{
	sys_write32(sys_read32(addr) & (~clr), addr);
}

static void clrsetbits_le32(mm_reg_t addr, uint32_t clr, uint32_t set)
{
	sys_write32((sys_read32(addr) & (~clr)) | set, addr);
}

/**
 * @brief Count the number of leading zeros of a uint16_t
 * @param [IN] x - the uint16_t to be counted
 * @return the number of leading zeros in x
 */
static int clz16(uint16_t x)
{
	int n = 0;

	if (x == 0) {
		return 16;
	}

	if (x <= 0x00ff) {
		n += 8;
		x <<= 8;
	}
	if (x <= 0x0fff) {
		n += 4;
		x <<= 4;
	}
	if (x <= 0x3fff) {
		n += 2;
		x <<= 2;
	}
	if (x <= 0x7fff) {
		n += 1;
	}

	return n;
}

 /**
  * @brief find the bit index of the max attainable speed from the bitmap
  * @param [IN] cap - bitmap of the speed capability
  * @return the bit index of the max attainable speed
  */
static uint16_t find_max_speed(uint16_t cap)
{
	return 15 - clz16(cap & ~LTPI_SP_CAP_DDR);
}

static void ltpi_phy_unlock(struct ast27xx_ltpi_data *ltpi)
{
	sys_write32(LTPI_PROT_KEY_UNLOCK, ltpi->config->phy_base + LTPI_PROT_KEY);
}

static void ltpi_enable_rx_bias(struct ast27xx_ltpi_data *ltpi)
{
	setbits_le32(ltpi->config->top_base + LTPI_LVDS_RX_CTRL,
		     REG_LTPI_LVDS_RX1_BIAS_EN | REG_LTPI_LVDS_RX0_BIAS_EN);
	k_usleep(1);
}

static int ltpi_phy_get_mode(struct ast27xx_ltpi_data *ltpi)
{
	uint32_t reg = sys_read32(ltpi->config->phy_base + LTPI_PHY_CTRL);

	return FIELD_GET(REG_LTPI_PHY_MODE, reg);
}

static void ltpi_phy_set_mode(struct ast27xx_ltpi_data *ltpi, int mode)
{
	__ASSERT_NO_MSG(mode > 0 && mode <= LTPI_PHY_MODE_DDR);

	clrsetbits_le32(ltpi->config->phy_base + LTPI_PHY_CTRL, REG_LTPI_PHY_MODE,
			FIELD_PREP(REG_LTPI_PHY_MODE, mode));
}

static int ltpi_phy_set_clksel(struct ast27xx_ltpi_data *ltpi, int clksel, bool is_op_clk)
{
	const struct ast27xx_ltpi_config *config = ltpi->config;
	uint32_t reg;

	reg = sys_read32(config->phy_base + LTPI_PLL_CTRL);
	reg &= ~(REG_LTPI_PLL_SELECT | REG_LTPI_PLL_SET | REG_LTPI_RX_PHY_CLK_INV |
		 REG_LTPI_TX_PHY_CLK_INV);
	reg |= FIELD_PREP(REG_LTPI_PLL_SELECT, clksel);

	if (ltpi->clk_inverse & RX_CLK_INVERSE) {
		reg |= REG_LTPI_RX_PHY_CLK_INV;
	}

	if (ltpi->clk_inverse & TX_CLK_INVERSE) {
		reg |= REG_LTPI_TX_PHY_CLK_INV;
	}

	if (is_op_clk) {
		reg |= REG_LTPI_PLL_SET;
	}

	sys_write32(reg, config->phy_base + LTPI_PLL_CTRL);

	return 0;
}

static void ltpi_set_crc_format(struct ast27xx_ltpi_data *ltpi, int crc_fmt)
{
	const struct ast27xx_ltpi_config *config = ltpi->config;
	uint32_t val = sys_read32(config->base + LTPI_CRC_OPTION);

	val &= ~(REG_LTPI_SW_CRC_OUT_ML_FIRST | REG_LTPI_SW_CRC_IN_LSB_FIRST);
	if (crc_fmt) {
		val |= REG_LTPI_SW_CRC_OUT_ML_FIRST | REG_LTPI_SW_CRC_IN_LSB_FIRST;
	}

	sys_write32(val, config->base + LTPI_CRC_OPTION);
}

static int ltpi_reset(struct ast27xx_ltpi_data *ltpi)
{
	int ret;

	ret = reset_line_assert_dt(&ltpi->config->reset);
	__ASSERT_NO_MSG(ret == 0);

	k_usleep(1);

	/* Do not turn off the clock, as the clock is shared between LTPI controllers */
	ret = clock_control_on(ltpi->config->clock_dev, ltpi->config->clock_ahb);
	__ASSERT_NO_MSG(ret == 0);

	ret = clock_control_on(ltpi->config->clock_dev, ltpi->config->clock_phy);
	__ASSERT_NO_MSG(ret == 0);

	ret = reset_line_deassert_dt(&ltpi->config->reset);
	__ASSERT_NO_MSG(ret == 0);

	ltpi_phy_unlock(ltpi);

	return 0;
}

static uint32_t ltpi_get_link_mng_state(struct ast27xx_ltpi_data *ltpi)
{
	return FIELD_GET(REG_LTPI_LINK_MNG_ST,
			 sys_read32(ltpi->config->base + LTPI_LINK_MNG_ST));
}

static int ltpi_poll_link_mng_state(struct ast27xx_ltpi_data *ltpi, uint32_t expected,
				    uint32_t unexpected, int timeout_us)
{
	uint64_t start, timeout_tick;
	uintptr_t addr = ltpi->config->base + LTPI_LINK_MNG_ST;
	uint32_t reg = sys_read32(addr);
	uint32_t state;
	int ret = LTPI_ERR_NONE;

	if (timeout_us) {
		start = sys_clock_tick_get_32();
		timeout_tick = Z_TIMEOUT_US(timeout_us).ticks;
	}

	do {
		reg = sys_read32(addr);
		state = FIELD_GET(REG_LTPI_LINK_MNG_ST, reg);

		if (state == expected)
			break;

		if (state == unexpected) {
			/* link is disconnected, break the loop directly */
			ret = LTPI_ERR_DISCON;
			break;
		}

		if (timeout_us && ((sys_clock_tick_get_32() - start) > timeout_tick)) {
			ret = LTPI_ERR_TIMEOUT;
			break;
		}
	} while (1);

	return ret;
}

static int ltpi_wait_state_pll_set(struct ast27xx_ltpi_data *ltpi, int timeout_us)
{
	return ltpi_poll_link_mng_state(ltpi, LTPI_LINK_MNG_ST_WAIT_PLL_SET, -1,
					timeout_us);
}

static int ltpi_wait_state_op(struct ast27xx_ltpi_data *ltpi)
{
	setbits_le32((mm_reg_t)&ltpi->ctrl_status_reg->link_status,
		     LTPI_STATUS_CONFIG_ACC_TO_ERR | LTPI_STATUS_CRC_ERR | LTPI_STATUS_LINK_LOST);

	return ltpi_poll_link_mng_state(ltpi, LTPI_LINK_MNG_ST_OP, LTPI_LINK_MNG_ST_DETECT_ALIGN,
					ADVERTISE_TIMEOUT_US);
}

static int ltpi_set_lvds_io_driving(struct ast27xx_ltpi_data *ltpi, int driving)
{
	uint32_t val;

	setbits_le32(ltpi->config->top_base + LTPI_SW_FORCE_EN, REG_LTPI_SW_FORCE_LVDS_TX_DS_EN);

	val = sys_read32(ltpi->config->top_base + LTPI_LVDS_TX_CTRL);
	val &= ~(REG_LTPI_LVDS_TX1_DS1 | REG_LTPI_LVDS_TX1_DS0 | REG_LTPI_LVDS_TX0_DS1 |
		 REG_LTPI_LVDS_TX0_DS0);

	/* tx1: clk, tx0: data */
	if (driving & BIT(1)) {
		val |= (REG_LTPI_LVDS_TX1_DS1 | REG_LTPI_LVDS_TX0_DS1);
	}

	if (driving & BIT(0)) {
		val |= (REG_LTPI_LVDS_TX1_DS0 | REG_LTPI_LVDS_TX0_DS0);
	}

	sys_write32(val, ltpi->config->top_base + LTPI_LVDS_TX_CTRL);

	return 0;
}

static int ltpi_set_local_speed_cap(struct ast27xx_ltpi_data *ltpi, uint32_t speed_cap)
{
	mm_reg_t reg_addr = (mm_reg_t)&ltpi->ctrl_status_reg->link_detect_cap_local;
	uint32_t reg;

	/* only set bits that Aspeed SOC supported */
	speed_cap &= LTPI_SP_CAP_ASPEED_SUPPORTED;
	if (ltpi->otp_ddr_dis) {
		speed_cap &= ~LTPI_SP_CAP_DDR;
	}

	reg = sys_read32(reg_addr);
	reg &= ~LTPI_LINK_SPEED_CAP;
	reg |= FIELD_PREP(LTPI_LINK_SPEED_CAP, speed_cap);
	sys_write32(reg, reg_addr);

	return 0;
}

static void bootstage_prologue(const char *mark)
{
	if (!mark)
		return;

	printf("%s", mark);
}

static void bootstage_epilogue(struct bootstage_t sts)
{
	printf(" %02x%02x\n", sts.errno, sts.syndrome);
}

static void ltpi_log_exit(struct ast27xx_ltpi_data *ltpi, int reason)
{
	ltpi->bootstage->errno |= LTPI_STATUS_EXIT;
	ltpi->bootstage->syndrome = reason;
}

static void ltpi_log_restart(struct ast27xx_ltpi_data *ltpi, int reason)
{
	ltpi->bootstage->errno |= LTPI_STATUS_RESTART;
	ltpi->bootstage->syndrome = reason;
	bootstage_epilogue(*ltpi->bootstage);

	/* Restart a boot log */
	bootstage_prologue(BOOTSTAGE_LTPI_INIT);
	ltpi->bootstage->errno &= ~LTPI_STATUS_RESTART;
	ltpi->bootstage->errno &= ~LTPI_STATUS_HAS_CRC_ERR;
	ltpi->bootstage->syndrome = LTPI_SYND_OK;
}

static void ltpi_log_phy_mode(struct ast27xx_ltpi_data *ltpi, int phy_mode)
{
	ltpi->bootstage->errno &= ~LTPI_STATUS_MODE;
	ltpi->bootstage->errno |= FIELD_PREP(LTPI_STATUS_MODE, phy_mode);
}

/*
 * Link training phase:
 * Link lost -> Link detect frame alignment -> Link detect -> Link speed -> Wait PLL set
 */
static void ltpi_do_link_training(struct ast27xx_ltpi_data *ltpi)
{
	/* Reset the PHY to PHY_MODE_OFF */
	ltpi_reset(ltpi);

	ltpi_enable_rx_bias(ltpi);
	ltpi_set_local_speed_cap(ltpi, ltpi->phy_speed_cap);
	ltpi_set_lvds_io_driving(ltpi, ltpi->io_driving);
	ltpi_set_crc_format(ltpi, ltpi->crc_format);

	/*
	 * Configure the LINK_SPEED frame count to be received before
	 * entering AD. This configuraiton only effects the SCM LTPI.
	 */
	clrsetbits_le32(ltpi->config->base + LTPI_LINK_MANAGE_CTRL0,
		    REG_LTPI_RX_LINK_SP_FRM_NUM,
		    FIELD_PREP(REG_LTPI_RX_LINK_SP_FRM_NUM, ltpi->link_speed_frm_rx_cnt));

	/* Set the clock source to the base frequency 25MHz */
	ltpi_phy_set_clksel(ltpi, REG_LTPI_PLL_25M, false);

	/* To make the remote side back to the link lost state */
	k_usleep(ADVERTISE_TIMEOUT_US);

	ltpi_phy_set_mode(ltpi, LTPI_PHY_MODE_SDR);
}

static int scu_get_pll_freq(mm_reg_t scu_base, int pll_id)
{
	const struct pll_info *info;
	uint32_t reg;
	int m, n, p;

	__ASSERT_NO_MSG(pll_id > 0 && pll_id <= SCU_PLL_ID_MAX);

	info = &scu_pll_info[pll_id];
	reg = sys_read32(scu_base + info->reg_offset0);
	m = FIELD_GET(PLL_REG1_M, reg);
	n = FIELD_GET(PLL_REG1_N, reg);
	p = FIELD_GET(PLL_REG1_P, reg);

	return (25000000 * (m + 1) / (n + 1) / (p + 1));
}

static int scu_set_pll_freq(mm_reg_t scu_base, int pll_id, int freq)
{
	const struct pll_info *info;
	const struct pll_param *param;
	int curr_freq, i;
	bool match = false;

	__ASSERT_NO_MSG(pll_id > 0 && pll_id <= SCU_PLL_ID_MAX);

	curr_freq = scu_get_pll_freq(scu_base, pll_id);
	if (curr_freq == freq)
		return 0;

	for (i = 0; i < NUM_PLL_PARAM; i++) {
		if (freq == pll_param_lookup[i].freq) {
			match = true;
			break;
		}
	}

	if (!match) {
		LOG_ERR("Can't find PLL frequency");
		return -EINVAL;
	}

	param = &pll_param_lookup[i];

	info = &scu_pll_info[pll_id];
	setbits_le32(scu_base + info->reg_offset0, PLL_REG1_RESET);
	clrsetbits_le32(scu_base + info->reg_offset0, PLL_REG1_P | PLL_REG1_N | PLL_REG1_M,
			param->n_m_p);

	clrsetbits_le32(scu_base + info->reg_offset1, PLL_REG2_BWADJ, param->bwadj);

	/* Wait 5us to ensure the parameters are set */
	k_busy_wait(5);
	clrbits_le32(scu_base + info->reg_offset0, PLL_REG1_RESET);

	/* PLL should be locked after 20us */
	k_busy_wait(20);

	return 0;
}

static int ltpi_set_operational_clk(struct ast27xx_ltpi_data *ltpi, uint16_t speed_cap)
{
	const struct ast27xx_ltpi_config *config = ltpi->config;
	int target_speed, phy_mode, pll_id;

	if (config->index) {
		pll_id = SCU_PLL_ID_IO_L1PLL;
	} else {
		pll_id = SCU_PLL_ID_IO_L0PLL;
	}

	/* find max attainable speed */
	target_speed = find_max_speed(speed_cap);

	/* set phy mode "OFF" */
	ltpi_phy_set_mode(ltpi, LTPI_PHY_MODE_OFF);

	if (speed_cap & LTPI_SP_CAP_DDR) {
		phy_mode = LTPI_PHY_MODE_DDR;
	} else {
		phy_mode = LTPI_PHY_MODE_SDR;
	}

	if (phy_mode == LTPI_PHY_MODE_SDR && target_speed == LTPI_SP_CAP_25M) {
		ltpi_phy_set_clksel(ltpi, REG_LTPI_PLL_25M, true);
	} else {
		int pll_freq = MHZ(ltpi_clk_lookup_sdr[target_speed]);

		if (phy_mode == LTPI_PHY_MODE_DDR) {
			pll_freq *= 2;
		}

		scu_set_pll_freq(ltpi->config->scu_base, pll_id, pll_freq);
		ltpi_phy_set_clksel(ltpi, REG_LTPI_PLL_LPLL, true);
	}

	/* Start TX with the operational frequency */
	ltpi_phy_set_mode(ltpi, phy_mode);

	ltpi_log_phy_mode(ltpi, phy_mode);

	return target_speed;
}

static int ltpi_optimeout_init(struct ast27xx_ltpi_data *ltpi)
{
	ltpi->t_link_detect = sys_clock_tick_get_32();

	return 0;
}

static int ltpi_optimeout_query(struct ast27xx_ltpi_data *ltpi)
{
	if (ltpi->op_timeout == 0)
		return 0;

	if ((sys_clock_tick_get_32() - ltpi->t_link_detect) > ltpi->op_timeout)
		return 1;

	return 0;
}

static int ast27xx_ltpi_do_link(const struct device *dev)
{
	struct ast27xx_ltpi_data *ltpi = dev->data;
	int ret, target_speed;
	uint32_t reg, state;

	/* Check whether LTPI is initialized */
	state = ltpi_get_link_mng_state(ltpi);
	if (state == LTPI_LINK_MNG_ST_OP) {
		ltpi_log_phy_mode(ltpi, ltpi_phy_get_mode(ltpi));
		ltpi_log_exit(ltpi, LTPI_SYND_OK_ALREADY_INIT);
		return 0;
	}

	ltpi_optimeout_init(ltpi);
	/* LTPI initialization is required, start link training phase */
	do {
		ltpi_do_link_training(ltpi);

		do {
			ret = ltpi_wait_state_pll_set(ltpi, 20000);
			if (ret == LTPI_ERR_NONE) {
				break;
			}

			if (ltpi_optimeout_query(ltpi)) {
				ltpi_log_exit(ltpi, LTPI_SYND_EXTRST_LINK_TRAINING);
				goto ltpi_scm_exit;
			}
		} while (1);

		/* read intersection of the speed capabilities */
		reg = FIELD_GET(REG_LTPI_SP_INTERSETION,
				sys_read32(ltpi->config->base + LTPI_LINK_MNG_ST));
		if (reg == 0) {
			ltpi_log_exit(ltpi, LTPI_SYND_NO_COMMOM_SPEED);
			goto ltpi_scm_exit;
		}

		target_speed = ltpi_set_operational_clk(ltpi, reg);

		/* poll link state 0x7 */
		ret = ltpi_wait_state_op(ltpi);
		if (ret == LTPI_ERR_NONE) {
			/* Start OEM TX & RX if the link partner is AST1700 */
			if (sys_read32(ltpi->config->base + LTPI_LINK_MNG_ST) &
			    REG_LTPI_LINK_PARTNER_FLAG) {

				setbits_le32(ltpi->config->base + LTPI_OEM_BUS_SETTING,
					     REG_LTPI_OEM_RX_START_TRIG |
						     REG_LTPI_OEM_TX_START_TRIG);
			}

			break;
		}

		if (sys_read32((mm_reg_t)ltpi->ctrl_status_reg->link_status) &
		    LTPI_STATUS_CRC_ERR) {
			ltpi->bootstage->errno |= LTPI_STATUS_HAS_CRC_ERR;
		}

		if (ltpi_optimeout_query(ltpi)) {
			ltpi_log_exit(ltpi, LTPI_SYND_EXTRST_LINK_CONFIG);
			goto ltpi_scm_exit;
		}

		/* clear the bit to specify the current speed doesn't work */
		ltpi->phy_speed_cap &= ~BIT(target_speed);

		/* the lowest speed 25M should always be supported */
		if (ltpi->phy_speed_cap == 0) {
			ltpi->phy_speed_cap |= BIT(0);
		}

		ltpi_log_restart(ltpi, LTPI_SYND_WAIT_OP_TO);
	} while (1);

	return 0;

ltpi_scm_exit:
	ltpi_reset(ltpi);

	return 0;
}

static int ast27xx_ltpi_get_status(const struct device *dev, struct ltpi_ctrl_status_regs **status)
{
	const struct ast27xx_ltpi_config *config = dev->config;

	*status = (struct ltpi_ctrl_status_regs *)config->base;

	return 0;
}

static int ast27xx_ltpi_init(const struct device *dev)
{
	const struct ast27xx_ltpi_config *config = dev->config;
	struct ast27xx_ltpi_data *data = dev->data;

	data->config = config;
	data->ctrl_status_reg = (struct ltpi_ctrl_status_regs *)config->base;
	data->clk_inverse = 0;
	data->io_driving = 0x2;
	data->crc_format = 0x0;
	data->link_speed_frm_rx_cnt = 0;
	data->op_timeout = 0;
	data->ad_timeout = ADVERTISE_TIMEOUT_US;
	data->otp_speed_cap = LTPI_SP_CAP_ASPEED_SUPPORTED;
	data->otp_ddr_dis = false;
	data->phy_speed_cap = data->otp_speed_cap;

	return 0;
}

static struct ltpi_driver_api ast27xx_ltpi_api = {
	.do_link = ast27xx_ltpi_do_link,
	.get_status = ast27xx_ltpi_get_status,
};

#define LTPI_AST27XX_INIT(n)                                                                       \
	static const struct ast27xx_ltpi_config ast27xx_ltpi_config_##n = {                        \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.phy_base = DT_INST_REG_ADDR(n) + 0x200,                                           \
		.top_base = DT_INST_REG_ADDR(n) + 0x800,                                           \
		.scu_base = DT_REG_ADDR_BY_IDX(DT_INST_PHANDLE(n, aspeed_scu), 0),                 \
		.index = DT_INST_PROP(n, index),                                                   \
		.reset.dev = DEVICE_DT_GET(DT_INST_RESET_CTLR(n)),                                 \
		.reset.id = DT_INST_RESET_ID(n),                                                   \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_ahb = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, ahb, clk_id),  \
		.clock_phy = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, phy, clk_id),  \
	};                                                                                         \
	static struct ast27xx_ltpi_data ast27xx_ltpi_data##n;                                      \
	DEVICE_DT_INST_DEFINE(n, ast27xx_ltpi_init, NULL, &ast27xx_ltpi_data##n,                   \
			      &ast27xx_ltpi_config_##n, POST_KERNEL,                               \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ast27xx_ltpi_api);

DT_INST_FOREACH_STATUS_OKAY(LTPI_AST27XX_INIT)
