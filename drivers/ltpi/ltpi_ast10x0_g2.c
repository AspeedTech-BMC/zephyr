/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_ast10x0_g2_ltpi

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/drivers/ltpi.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log.h>
#define LOG_MODULE_NAME			ltpi_ast10x0_g2
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_LTPI_LOG_LEVEL);

/*
 * AST10X0-G2, operating as the HPM side of an LTPI link. Unlike the AST27xx
 * SCM-side implementation (ltpi_ast27xx.c) this generation:
 *  - has a single LTPI instance (no dual LTPI0/LTPI1),
 *  - has no OEM frame, so the OEM TX/RX bus is never started,
 *  - uses a different, simpler PLL (SCU+0x310/0x31C, documented in the
 *    AST1040 program guide) instead of the AST27xx IO-die L0PLL/L1PLL.
 * Kept deliberately separate from ltpi_ast27xx.c rather than parameterized,
 * since the two chips do not share a PLL or top-level link-up sequence.
 */

#define BOOTSTAGE_LTPI_INIT			"L "
#define ADVERTISE_TIMEOUT_US			105000 /* 105 ms */

struct bootstage_t {
	uint8_t flags;
	uint8_t syndrome;
};

#define LTPI_SP_CAP_ASPEED_SUPPORTED                              \
	(LTPI_SP_CAP_25M | LTPI_SP_CAP_50M | LTPI_SP_CAP_75M |    \
	 LTPI_SP_CAP_100M | LTPI_SP_CAP_150M | LTPI_SP_CAP_200M | \
	 LTPI_SP_CAP_250M | LTPI_SP_CAP_300M | LTPI_SP_CAP_400M | \
	 LTPI_SP_CAP_500M | LTPI_SP_CAP_DDR)

/* bootstage_t->flags */
#define LTPI_STATUS_EXIT			BIT(7)	/* 1: exit due to errors */
#define LTPI_STATUS_RESTART			BIT(6)	/* 1: restart LTPI initialization */

#define LTPI_STATUS_HAS_CRC_ERR			BIT(4)
#define LTPI_STATUS_MODE			GENMASK(3, 2)	/* PHY mode */
#define LTPI_STATUS_HPM				BIT(0)	/* 1: LTPI controller is on HPM */

/* bootstage_t->syndrome */
#define LTPI_SYND_OK				0
#define LTPI_SYND_OK_ALREADY_INIT		1
#define LTPI_SYND_NO_COMMON_SPEED		2
#define LTPI_SYND_WAIT_OP_TO			3
#define LTPI_SYND_TIMEOUT_LINK_TRAINING		4 /* timeout waiting for WAIT_PLL_SET state */
#define LTPI_SYND_TIMEOUT_LINK_CONFIG		5 /* timeout waiting for OP state */
#define LTPI_SYND_PLL_PROGRAM_FAILED		6 /* no pll_param_lookup entry for target freq */
#define LTPI_SYND_RESET_FAILED			7 /* reset/clock-enable sequence failed */

/* LTPI wait state return code */
#define LTPI_ERR_NONE				0x00
#define LTPI_ERR_TIMEOUT			0x10
#define LTPI_ERR_DISCON				0x20

/* LTPI PHY control registers (offset from phy_base) */
#define LTPI_PHY_CTRL				0x000
#define   REG_LTPI_PHY_MODE			GENMASK(3, 0)
#define     LTPI_PHY_MODE_DDR			0b0010
#define     LTPI_PHY_MODE_SDR			0b0001
#define     LTPI_PHY_MODE_OFF			0b0000
#define LTPI_PLL_CTRL				0x004
#define   REG_LTPI_RX_PHY_CLK_INV		BIT(9)
#define   REG_LTPI_TX_PHY_CLK_INV		BIT(8)
#define   REG_LTPI_PLL_SET			BIT(4)
#define   REG_LTPI_PLL_SELECT			GENMASK(2, 0)
#define     REG_LTPI_PLL_25M			0
#define     REG_LTPI_PLL_LPLL			7

#define LTPI_PROT_KEY				0x34
#define   LTPI_PROT_KEY_UNLOCK			0x1728aacc

/* LVDS TOP registers (offset from top_base) */
#define LTPI_LVDS_TX_CTRL			0x000
#define   REG_LTPI_LVDS_TX1_DS1			BIT(22)
#define   REG_LTPI_LVDS_TX1_DS0			BIT(21)
#define   REG_LTPI_LVDS_TX0_DS1			BIT(6)
#define   REG_LTPI_LVDS_TX0_DS0			BIT(5)
#define LTPI_LVDS_RX_CTRL			0x004
#define   REG_LTPI_LVDS_RX1_BIAS_EN		BIT(18)
#define   REG_LTPI_LVDS_RX0_BIAS_EN		BIT(2)
#define LTPI_SW_FORCE_EN			0x010
#define   REG_LTPI_SW_FORCE_LVDS_TX_DS_EN	BIT(2)
#define   REG_LTPI_SW_FORCE_1700_EN		BIT(0)
#define LTPI_SW_FORCE_VAL			0x014
#define   REG_LTPI_SW_FORCE_VAL_1700_EN		BIT(0)

/* LTPI control registers (offset from base) */
#define LTPI_LINK_MNG_ST			0x108
#define   REG_LTPI_SP_INTERSECTION		GENMASK(23, 8)
#define   REG_LTPI_LINK_MNG_ST			GENMASK(3, 0)
#define     LTPI_LINK_MNG_ST_DETECT_ALIGN	0
#define     LTPI_LINK_MNG_ST_WAIT_PLL_SET	3
#define     LTPI_LINK_MNG_ST_OP			7
/* REG_LTPI_LINK_MNG_ST is 4 bits: no real state value can ever match this */
#define     LTPI_LINK_MNG_ST_NONE		UINT32_MAX
#define LTPI_LINK_MANAGE_CTRL0			0x10C
#define   REG_LTPI_RX_LINK_SP_FRM_NUM		GENMASK(3, 0)
#define LTPI_CRC_OPTION				0x12c
#define   REG_LTPI_SW_CRC_OUT_ML_FIRST		BIT(2)
#define   REG_LTPI_SW_CRC_IN_LSB_FIRST		BIT(1)

/*
 * advertise_cap_local_lo (struct ltpi_ctrl_status_regs, offset 0x014).
 * This silicon's defaults wrongly advertise OEM-frame/data-frame support;
 * it supports neither.
 */
#define   REG_LTPI_AD_CAP_OEM_FRAME_SUPPORT	BIT(4)
#define   REG_LTPI_AD_CAP_DATA_FRAME_SUPPORT	BIT(3)

static const int16_t ltpi_clk_lookup_sdr[13] = {
	25, 50, 75, 100, 150, 200, 250, 300, 400, 600, -1, -1, 500,
};

/*
 * LTPI PLL control (SCU310/SCU31C, documented in the AST1040 program
 * guide). Single PLL feeding the single LTPI instance; this is not the
 * AST27xx IO-die L0PLL/L1PLL layout, do not reuse those offsets here.
 */
#define SCU1_LTPI_PLL_CTRL			0x310
#define   REG_LTPI_PLL_DCO_EN			BIT(1)
#define   REG_LTPI_PLL_EN			BIT(0)
#define   REG_LTPI_PLL_DIV_N			GENMASK(17, 9)
#define SCU1_LTPI_PLL_DIV			0x31C
#define   REG_LTPI_PLL_DIV_POST			GENMASK(5, 1)

#define NUM_PLL_PARAM				13

struct pll_param {
	int freq;
	uint16_t div_n;
	uint16_t div_post;
};

static const struct pll_param pll_param_lookup[NUM_PLL_PARAM] = {
	{ .freq = MHZ(50), .div_n = 40, .div_post = 20 },
	{ .freq = MHZ(75), .div_n = 30, .div_post = 10 },
	{ .freq = MHZ(100), .div_n = 40, .div_post = 10 },
	{ .freq = MHZ(150), .div_n = 48, .div_post = 8 },
	{ .freq = MHZ(200), .div_n = 40, .div_post = 5 },
	{ .freq = MHZ(250), .div_n = 40, .div_post = 4 },
	{ .freq = MHZ(300), .div_n = 48, .div_post = 4 },
	{ .freq = MHZ(400), .div_n = 48, .div_post = 3 },
	{ .freq = MHZ(500), .div_n = 40, .div_post = 2 },
	{ .freq = MHZ(600), .div_n = 48, .div_post = 2 },
	{ .freq = MHZ(800), .div_n = 32, .div_post = 1 },
	{ .freq = MHZ(1000), .div_n = 40, .div_post = 1 },
	{ .freq = MHZ(1200), .div_n = 48, .div_post = 1 },
};

struct ast10x0_g2_ltpi_config {
	mm_reg_t base;
	mm_reg_t phy_base;
	mm_reg_t top_base;
	mm_reg_t scu_base;
	struct reset_dt_spec reset;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_ahb;
	const clock_control_subsys_t clock_phy;
};

struct ast10x0_g2_ltpi_data {
	const struct ast10x0_g2_ltpi_config *config;
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
	struct bootstage_t bootstage;

	/* Advertise timeout in us */
	int ad_timeout;

	/*
	 * Start time to listen LINK_DETECT frame, and the timeout to reach
	 * operational state, both in the 32-bit tick count returned by
	 * sys_clock_tick_get_32(). Keep these the same width as that
	 * counter: mixing widths in `sys_clock_tick_get_32() - t_link_detect`
	 * breaks the wraparound-safe modular arithmetic a same-width
	 * unsigned subtraction gives for free.
	 */
	uint32_t t_link_detect;
	uint32_t op_timeout;

	/* Serializes ast10x0_g2_ltpi_do_link() against concurrent callers. */
	struct k_mutex lock;
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

static void ltpi_phy_unlock(struct ast10x0_g2_ltpi_data *ltpi)
{
	sys_write32(LTPI_PROT_KEY_UNLOCK, ltpi->config->phy_base + LTPI_PROT_KEY);
}

/*
 * Force this LTPI instance into AST1700/HPM mode (AST1040 program guide,
 * "Enable HPM LTPI": write <base>+0x810[0]=1, then <base>+0x814[0]=1).
 * 0x810/0x814 sit in the LVDS-TOP aperture (top_base = base + 0x800), so
 * these are top_base + 0x010 / top_base + 0x014 here.
 */
static void ltpi_enable_hpm_mode(struct ast10x0_g2_ltpi_data *ltpi)
{
	setbits_le32(ltpi->config->top_base + LTPI_SW_FORCE_EN, REG_LTPI_SW_FORCE_1700_EN);
	setbits_le32(ltpi->config->top_base + LTPI_SW_FORCE_VAL, REG_LTPI_SW_FORCE_VAL_1700_EN);
	LOG_DBG("hpm mode: forced via SW_FORCE_EN/VAL (base+0x810[0]=1, base+0x814[0]=1)");
}

static void ltpi_enable_rx_bias(struct ast10x0_g2_ltpi_data *ltpi)
{
	setbits_le32(ltpi->config->top_base + LTPI_LVDS_RX_CTRL,
		     REG_LTPI_LVDS_RX1_BIAS_EN | REG_LTPI_LVDS_RX0_BIAS_EN);
	k_usleep(1);
}

static int ltpi_phy_get_mode(struct ast10x0_g2_ltpi_data *ltpi)
{
	uint32_t reg = sys_read32(ltpi->config->phy_base + LTPI_PHY_CTRL);

	return FIELD_GET(REG_LTPI_PHY_MODE, reg);
}

static void ltpi_phy_set_mode(struct ast10x0_g2_ltpi_data *ltpi, int mode)
{
	__ASSERT_NO_MSG(mode >= LTPI_PHY_MODE_OFF && mode <= LTPI_PHY_MODE_DDR);

	clrsetbits_le32(ltpi->config->phy_base + LTPI_PHY_CTRL, REG_LTPI_PHY_MODE,
			FIELD_PREP(REG_LTPI_PHY_MODE, mode));
}

static int ltpi_phy_set_clksel(struct ast10x0_g2_ltpi_data *ltpi, int clksel, bool is_op_clk)
{
	const struct ast10x0_g2_ltpi_config *config = ltpi->config;
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

static void ltpi_set_crc_format(struct ast10x0_g2_ltpi_data *ltpi, int crc_fmt)
{
	const struct ast10x0_g2_ltpi_config *config = ltpi->config;
	uint32_t val = sys_read32(config->base + LTPI_CRC_OPTION);

	val &= ~(REG_LTPI_SW_CRC_OUT_ML_FIRST | REG_LTPI_SW_CRC_IN_LSB_FIRST);
	if (crc_fmt) {
		val |= REG_LTPI_SW_CRC_OUT_ML_FIRST | REG_LTPI_SW_CRC_IN_LSB_FIRST;
	}

	sys_write32(val, config->base + LTPI_CRC_OPTION);
}

static int ltpi_reset(struct ast10x0_g2_ltpi_data *ltpi)
{
	int ret;

	LOG_DBG("reset: assert");
	ret = reset_line_assert_dt(&ltpi->config->reset);
	if (ret) {
		LOG_ERR("reset: failed to assert reset (%d)", ret);
		return ret;
	}

	k_usleep(1);

	ret = clock_control_on(ltpi->config->clock_dev, ltpi->config->clock_ahb);
	if (ret) {
		LOG_ERR("reset: failed to enable AHB clock (%d)", ret);
		return ret;
	}

	ret = clock_control_on(ltpi->config->clock_dev, ltpi->config->clock_phy);
	if (ret) {
		LOG_ERR("reset: failed to enable PHY/TX clock (%d)", ret);
		return ret;
	}

	ret = reset_line_deassert_dt(&ltpi->config->reset);
	if (ret) {
		LOG_ERR("reset: failed to deassert reset (%d)", ret);
		return ret;
	}

	ltpi_phy_unlock(ltpi);
	ltpi_enable_hpm_mode(ltpi);
	LOG_DBG("reset: deasserted, clocks on, PHY unlocked, HPM mode forced");

	return 0;
}

/*
 * Leave the controller stopped: PLL off, held in reset. Best-effort only,
 * since this runs on a failure-exit path.
 */
static void ltpi_quiesce(struct ast10x0_g2_ltpi_data *ltpi)
{
	clrbits_le32(ltpi->config->scu_base + SCU1_LTPI_PLL_CTRL,
		     REG_LTPI_PLL_EN | REG_LTPI_PLL_DCO_EN);
	reset_line_assert_dt(&ltpi->config->reset);
}

static uint32_t ltpi_get_link_mng_state(struct ast10x0_g2_ltpi_data *ltpi)
{
	return FIELD_GET(REG_LTPI_LINK_MNG_ST,
			 sys_read32(ltpi->config->base + LTPI_LINK_MNG_ST));
}

static int ltpi_poll_link_mng_state(struct ast10x0_g2_ltpi_data *ltpi, uint32_t expected,
				    uint32_t unexpected, int timeout_us)
{
	uint32_t start = 0, timeout_tick = 0;
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

		if (state == expected) {
			break;
		}

		if (state == unexpected) {
			/* link is disconnected, break the loop directly */
			ret = LTPI_ERR_DISCON;
			break;
		}

		if (timeout_us && ((sys_clock_tick_get_32() - start) > timeout_tick)) {
			ret = LTPI_ERR_TIMEOUT;
			break;
		}

		/*
		 * This loop can legitimately run for seconds. Yield instead of
		 * busy-spinning so other threads (e.g. the shell, or whatever
		 * else called ltpi_do_link()) still get scheduled -- otherwise
		 * this starves everything else on a single-core target for the
		 * whole wait.
		 */
		k_usleep(100);
	} while (1);

	return ret;
}

static int ltpi_wait_state_pll_set(struct ast10x0_g2_ltpi_data *ltpi, int timeout_us)
{
	return ltpi_poll_link_mng_state(ltpi, LTPI_LINK_MNG_ST_WAIT_PLL_SET,
					LTPI_LINK_MNG_ST_NONE, timeout_us);
}

static int ltpi_wait_state_op(struct ast10x0_g2_ltpi_data *ltpi)
{
	setbits_le32((mm_reg_t)&ltpi->ctrl_status_reg->link_status,
		     LTPI_STATUS_CONFIG_ACC_TO_ERR | LTPI_STATUS_CRC_ERR | LTPI_STATUS_LINK_LOST);

	return ltpi_poll_link_mng_state(ltpi, LTPI_LINK_MNG_ST_OP, LTPI_LINK_MNG_ST_DETECT_ALIGN,
					ADVERTISE_TIMEOUT_US);
}

static int ltpi_set_lvds_io_driving(struct ast10x0_g2_ltpi_data *ltpi, int driving)
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

static int ltpi_set_local_speed_cap(struct ast10x0_g2_ltpi_data *ltpi, uint32_t speed_cap)
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

/*
 * Clear the OEM-frame and data-frame support bits this silicon wrongly
 * advertises by default in advertise_cap_local_lo. Left set, the SCM side
 * believes it can use OEM/Data frame access and breaks when it tries,
 * since neither is supported. Must run before every advertise phase,
 * since ltpi_reset()'s PHY reset restores the hardware default.
 */
static void ltpi_clear_unsupported_ad_caps(struct ast10x0_g2_ltpi_data *ltpi)
{
	clrbits_le32((mm_reg_t)&ltpi->ctrl_status_reg->advertise_cap_local_lo,
		     REG_LTPI_AD_CAP_OEM_FRAME_SUPPORT | REG_LTPI_AD_CAP_DATA_FRAME_SUPPORT);
	LOG_DBG("ad cap: cleared OEM/data frame support bits (unsupported)");
}

static void bootstage_prologue(const char *mark)
{
	if (!mark) {
		return;
	}

	LOG_DBG("%s", mark);
}

static void bootstage_epilogue(struct bootstage_t sts)
{
	LOG_DBG("bootstage: flags=0x%02x syndrome=0x%02x", sts.flags, sts.syndrome);
}

static const char *ltpi_syndrome_str(int reason)
{
	switch (reason) {
	case LTPI_SYND_OK:
		return "ok";
	case LTPI_SYND_OK_ALREADY_INIT:
		return "already initialized";
	case LTPI_SYND_NO_COMMON_SPEED:
		return "no common speed with link partner";
	case LTPI_SYND_WAIT_OP_TO:
		return "timeout waiting for operational state";
	case LTPI_SYND_TIMEOUT_LINK_TRAINING:
		return "timeout waiting for PLL-set state";
	case LTPI_SYND_TIMEOUT_LINK_CONFIG:
		return "timeout waiting for operational state (link config)";
	case LTPI_SYND_PLL_PROGRAM_FAILED:
		return "no PLL parameters for the negotiated speed";
	case LTPI_SYND_RESET_FAILED:
		return "reset/clock-enable sequence failed";
	default:
		return "unknown";
	}
}

static void ltpi_log_exit(struct ast10x0_g2_ltpi_data *ltpi, int reason)
{
	if (reason == LTPI_SYND_OK || reason == LTPI_SYND_OK_ALREADY_INIT) {
		LOG_INF("exit: %s (syndrome=%d)", ltpi_syndrome_str(reason), reason);
	} else {
		LOG_ERR("exit: %s (syndrome=%d)", ltpi_syndrome_str(reason), reason);
		ltpi->bootstage.flags |= LTPI_STATUS_EXIT;
	}
	ltpi->bootstage.syndrome = reason;
	bootstage_epilogue(ltpi->bootstage);
}

static void ltpi_log_restart(struct ast10x0_g2_ltpi_data *ltpi, int reason)
{
	LOG_WRN("restart: %s (syndrome=%d), phy_speed_cap now 0x%04x", ltpi_syndrome_str(reason),
		reason, ltpi->phy_speed_cap);
	ltpi->bootstage.flags |= LTPI_STATUS_RESTART;
	ltpi->bootstage.syndrome = reason;
	bootstage_epilogue(ltpi->bootstage);

	/* Restart a boot log */
	bootstage_prologue(BOOTSTAGE_LTPI_INIT);
	ltpi->bootstage.flags &= ~LTPI_STATUS_RESTART;
	ltpi->bootstage.flags &= ~LTPI_STATUS_HAS_CRC_ERR;
	ltpi->bootstage.syndrome = LTPI_SYND_OK;
}

static void ltpi_log_phy_mode(struct ast10x0_g2_ltpi_data *ltpi, int phy_mode)
{
	ltpi->bootstage.flags &= ~LTPI_STATUS_MODE;
	ltpi->bootstage.flags |= FIELD_PREP(LTPI_STATUS_MODE, phy_mode);
}

/*
 * Link training phase:
 * Link lost -> Link detect frame alignment -> Link detect -> Link speed -> Wait PLL set
 */
static int ltpi_do_link_training(struct ast10x0_g2_ltpi_data *ltpi)
{
	int ret;

	LOG_INF("link training: speed_cap=0x%04x io_driving=0x%x crc_fmt=%d",
		ltpi->phy_speed_cap, ltpi->io_driving, ltpi->crc_format);

	/* Reset the PHY to PHY_MODE_OFF */
	ret = ltpi_reset(ltpi);
	if (ret) {
		return ret;
	}

	ltpi_enable_rx_bias(ltpi);
	ltpi_clear_unsupported_ad_caps(ltpi);
	ltpi_set_local_speed_cap(ltpi, ltpi->phy_speed_cap);
	ltpi_set_lvds_io_driving(ltpi, ltpi->io_driving);
	ltpi_set_crc_format(ltpi, ltpi->crc_format);

	/*
	 * Configure the LINK_SPEED frame count to be received before
	 * entering AD. This configuration only affects the SCM LTPI.
	 */
	clrsetbits_le32(ltpi->config->base + LTPI_LINK_MANAGE_CTRL0,
			REG_LTPI_RX_LINK_SP_FRM_NUM,
			FIELD_PREP(REG_LTPI_RX_LINK_SP_FRM_NUM, ltpi->link_speed_frm_rx_cnt));

	/* Set the clock source to the base frequency 25MHz */
	ltpi_phy_set_clksel(ltpi, REG_LTPI_PLL_25M, false);

	LOG_DBG("link training: waiting %d us for remote link-lost", ADVERTISE_TIMEOUT_US);
	/* To make the remote side back to the link lost state */
	k_usleep(ADVERTISE_TIMEOUT_US);

	ltpi_phy_set_mode(ltpi, LTPI_PHY_MODE_SDR);
	LOG_DBG("link training: PHY mode set to SDR, advertising begins");

	return 0;
}

/*
 * LTPI PLL programming sequence (per the AST1040 program guide):
 *   1. Clear SCU310[1:0] (dco_en, pll_en)
 *   2. Program div_n (SCU310[17:9]) and div_post (SCU31C[5:1])
 *   3. Set SCU310[1:0]
 *   4. Wait 12us for PLL lock
 * freq = div_n / div_post * 25MHz
 */
/* div_n is a 9-bit field (up to 511): 25000000 * div_n can exceed INT32_MAX. */
static int scu_get_pll_freq(mm_reg_t scu_base)
{
	uint32_t ctrl = sys_read32(scu_base + SCU1_LTPI_PLL_CTRL);
	uint32_t div = sys_read32(scu_base + SCU1_LTPI_PLL_DIV);
	uint32_t div_n = FIELD_GET(REG_LTPI_PLL_DIV_N, ctrl);
	uint32_t div_post = FIELD_GET(REG_LTPI_PLL_DIV_POST, div);

	if (div_post == 0) {
		return 0;
	}

	return (int)((uint64_t)MHZ(25) * div_n / div_post);
}

/* Both PLL_EN and DCO_EN must be set for the PLL to actually be running. */
static bool scu_pll_is_on(mm_reg_t scu_base)
{
	uint32_t ctrl = sys_read32(scu_base + SCU1_LTPI_PLL_CTRL);

	return (ctrl & (REG_LTPI_PLL_EN | REG_LTPI_PLL_DCO_EN)) ==
	       (REG_LTPI_PLL_EN | REG_LTPI_PLL_DCO_EN);
}

static int scu_set_pll_freq(mm_reg_t scu_base, int freq)
{
	const struct pll_param *param = NULL;
	int i;

	if (scu_pll_is_on(scu_base) && scu_get_pll_freq(scu_base) == freq) {
		LOG_DBG("pll: already at %d Hz and running, nothing to do", freq);
		return 0;
	}

	for (i = 0; i < NUM_PLL_PARAM; i++) {
		if (freq == pll_param_lookup[i].freq) {
			param = &pll_param_lookup[i];
			break;
		}
	}

	if (!param) {
		LOG_ERR("pll: can't find PLL parameters for %d Hz", freq);
		return -EINVAL;
	}

	LOG_INF("pll: programming %d Hz (div_n=%d div_post=%d)", freq, param->div_n,
		param->div_post);

	clrbits_le32(scu_base + SCU1_LTPI_PLL_CTRL, REG_LTPI_PLL_EN | REG_LTPI_PLL_DCO_EN);

	clrsetbits_le32(scu_base + SCU1_LTPI_PLL_CTRL, REG_LTPI_PLL_DIV_N,
			FIELD_PREP(REG_LTPI_PLL_DIV_N, param->div_n));
	clrsetbits_le32(scu_base + SCU1_LTPI_PLL_DIV, REG_LTPI_PLL_DIV_POST,
			FIELD_PREP(REG_LTPI_PLL_DIV_POST, param->div_post));

	setbits_le32(scu_base + SCU1_LTPI_PLL_CTRL, REG_LTPI_PLL_EN | REG_LTPI_PLL_DCO_EN);

	/* PLL should be locked after 12us */
	k_busy_wait(12);

	LOG_DBG("pll: ctrl=0x%08x div=0x%08x readback=%d Hz",
		sys_read32(scu_base + SCU1_LTPI_PLL_CTRL),
		sys_read32(scu_base + SCU1_LTPI_PLL_DIV), scu_get_pll_freq(scu_base));

	return 0;
}

static int ltpi_set_operational_clk(struct ast10x0_g2_ltpi_data *ltpi, uint16_t speed_cap)
{
	int target_speed, phy_mode;

	/* find max attainable speed: a bit INDEX into ltpi_clk_lookup_sdr[], not a cap bitmask */
	target_speed = find_max_speed(speed_cap);

	/*
	 * Defense in depth: speed_cap is the intersection reported by the
	 * link partner over the wire. The do_link() caller already rejects a
	 * DDR-only intersection (which is what would otherwise send
	 * target_speed out of range here), but don't trust hardware/a
	 * partner to never report something unexpected.
	 */
	if (target_speed >= ARRAY_SIZE(ltpi_clk_lookup_sdr) ||
	    ltpi_clk_lookup_sdr[target_speed] < 0) {
		LOG_ERR("operational clock: target_speed_idx=%d out of range", target_speed);
		return -EINVAL;
	}

	/* set phy mode "OFF" */
	ltpi_phy_set_mode(ltpi, LTPI_PHY_MODE_OFF);

	if (speed_cap & LTPI_SP_CAP_DDR) {
		phy_mode = LTPI_PHY_MODE_DDR;
	} else {
		phy_mode = LTPI_PHY_MODE_SDR;
	}

	LOG_INF("operational clock: target_speed_idx=%d (%d MHz) phy_mode=%s",
		target_speed, ltpi_clk_lookup_sdr[target_speed],
		(phy_mode == LTPI_PHY_MODE_DDR) ? "DDR" : "SDR");

	/* index 0 is the 25M entry in ltpi_clk_lookup_sdr[]; LTPI_SP_CAP_25M is a bitmask, not 0 */
	if (phy_mode == LTPI_PHY_MODE_SDR && target_speed == 0) {
		LOG_DBG("operational clock: staying on 25MHz reference clock");
		ltpi_phy_set_clksel(ltpi, REG_LTPI_PLL_25M, true);
	} else {
		int pll_freq = MHZ(ltpi_clk_lookup_sdr[target_speed]);
		int ret;

		if (phy_mode == LTPI_PHY_MODE_DDR) {
			pll_freq *= 2;
		}

		ret = scu_set_pll_freq(ltpi->config->scu_base, pll_freq);
		if (ret) {
			return ret;
		}
		ltpi_phy_set_clksel(ltpi, REG_LTPI_PLL_LPLL, true);
	}

	/* Start TX with the operational frequency */
	ltpi_phy_set_mode(ltpi, phy_mode);

	ltpi_log_phy_mode(ltpi, phy_mode);

	return target_speed;
}

static int ltpi_optimeout_init(struct ast10x0_g2_ltpi_data *ltpi, int timeout_ms)
{
	ltpi->op_timeout = Z_TIMEOUT_MS(timeout_ms).ticks;
	ltpi->t_link_detect = sys_clock_tick_get_32();

	return 0;
}

static int ltpi_optimeout_query(struct ast10x0_g2_ltpi_data *ltpi)
{
	if (ltpi->op_timeout == 0) {
		return 0;
	}

	if ((sys_clock_tick_get_32() - ltpi->t_link_detect) > ltpi->op_timeout) {
		return 1;
	}

	return 0;
}

static int ast10x0_g2_ltpi_do_link(const struct device *dev, int timeout_ms)
{
	struct ast10x0_g2_ltpi_data *ltpi = dev->data;
	int ret, target_speed, err = 0;
	uint32_t reg, state;

	if (timeout_ms < 0) {
		LOG_ERR("do_link: invalid timeout_ms=%d", timeout_ms);
		return -EINVAL;
	}

	k_mutex_lock(&ltpi->lock, K_FOREVER);

	bootstage_prologue(BOOTSTAGE_LTPI_INIT);
	LOG_INF("do_link: starting, timeout_ms=%d", timeout_ms);

	ltpi->bootstage.flags |= LTPI_STATUS_HPM;

	/* Check whether LTPI is initialized */
	state = ltpi_get_link_mng_state(ltpi);
	if (state == LTPI_LINK_MNG_ST_OP) {
		LOG_INF("do_link: link already operational (mng_st=%d)", state);
		ltpi_log_phy_mode(ltpi, ltpi_phy_get_mode(ltpi));
		ltpi_log_exit(ltpi, LTPI_SYND_OK_ALREADY_INIT);
		k_mutex_unlock(&ltpi->lock);
		return 0;
	}
	LOG_DBG("do_link: link_mng_st=%d, link not yet operational", state);

	ltpi_optimeout_init(ltpi, timeout_ms);
	/* LTPI initialization is required, start link training phase */
	do {
		ret = ltpi_do_link_training(ltpi);
		if (ret) {
			err = ret;
			ltpi_log_exit(ltpi, LTPI_SYND_RESET_FAILED);
			goto ltpi_hpm_exit;
		}

		LOG_DBG("do_link: waiting for WAIT_PLL_SET state");
		for (int poll_cnt = 0; ; poll_cnt++) {
			ret = ltpi_wait_state_pll_set(ltpi, 20000);
			if (ret == LTPI_ERR_NONE) {
				LOG_INF("do_link: reached WAIT_PLL_SET state after %d poll(s)",
					poll_cnt + 1);
				break;
			}

			/*
			 * Each poll already blocks up to 20ms; this can legitimately
			 * repeat for seconds while waiting on the link partner, so
			 * only heartbeat every ~1s instead of flooding the log.
			 */
			if ((poll_cnt % 50) == 0) {
				LOG_WRN("do_link: still waiting for WAIT_PLL_SET (ret=%d, poll=%d)",
					ret, poll_cnt);
			}

			if (ltpi_optimeout_query(ltpi)) {
				err = -ETIMEDOUT;
				ltpi_log_exit(ltpi, LTPI_SYND_TIMEOUT_LINK_TRAINING);
				goto ltpi_hpm_exit;
			}
		}

		/* read intersection of the speed capabilities */
		reg = FIELD_GET(REG_LTPI_SP_INTERSECTION,
				sys_read32(ltpi->config->base + LTPI_LINK_MNG_ST));
		LOG_INF("do_link: speed capability intersection=0x%04x", reg);
		/*
		 * DDR alone (no actual speed bit) is not a usable intersection:
		 * find_max_speed() masks out LTPI_SP_CAP_DDR before looking for
		 * the highest set bit, so an all-zero remainder would make it
		 * return a huge value (clz16(0) == 16, 15-16 wraps to 65535 as
		 * uint16_t) and index far out of ltpi_clk_lookup_sdr[] below.
		 */
		if ((reg & ~LTPI_SP_CAP_DDR) == 0) {
			err = -ENOTSUP;
			ltpi_log_exit(ltpi, LTPI_SYND_NO_COMMON_SPEED);
			goto ltpi_hpm_exit;
		}

		target_speed = ltpi_set_operational_clk(ltpi, reg);
		if (target_speed < 0) {
			err = target_speed;
			ltpi_log_exit(ltpi, LTPI_SYND_PLL_PROGRAM_FAILED);
			goto ltpi_hpm_exit;
		}

		/* poll link state 0x7 */
		LOG_DBG("do_link: waiting for OP state");
		ret = ltpi_wait_state_op(ltpi);
		if (ret == LTPI_ERR_NONE) {
			LOG_INF("do_link: link operational");
			break;
		}

		LOG_WRN("do_link: failed to reach OP state (ret=%d, link_status=0x%08x)", ret,
			sys_read32((mm_reg_t)&ltpi->ctrl_status_reg->link_status));

		if (sys_read32((mm_reg_t)&ltpi->ctrl_status_reg->link_status) &
		    LTPI_STATUS_CRC_ERR) {
			LOG_WRN("do_link: CRC error flagged on link_status");
			ltpi->bootstage.flags |= LTPI_STATUS_HAS_CRC_ERR;
		}

		if (ltpi_optimeout_query(ltpi)) {
			err = -ETIMEDOUT;
			ltpi_log_exit(ltpi, LTPI_SYND_TIMEOUT_LINK_CONFIG);
			goto ltpi_hpm_exit;
		}

		/* clear the bit to specify the current speed doesn't work */
		ltpi->phy_speed_cap &= ~BIT(target_speed);

		/* the lowest speed 25M should always be supported */
		if ((ltpi->phy_speed_cap & LTPI_SP_CAP_25M) == 0) {
			ltpi->phy_speed_cap |= LTPI_SP_CAP_25M;
		}

		ltpi_log_restart(ltpi, LTPI_SYND_WAIT_OP_TO);
	} while (1);

	bootstage_epilogue(ltpi->bootstage);
	k_mutex_unlock(&ltpi->lock);

	return 0;

ltpi_hpm_exit:
	LOG_ERR("do_link: aborting, quiescing LTPI controller");
	ltpi_quiesce(ltpi);
	k_mutex_unlock(&ltpi->lock);

	return err;
}

static int ast10x0_g2_ltpi_get_status(const struct device *dev,
				      struct ltpi_ctrl_status_regs **status)
{
	const struct ast10x0_g2_ltpi_config *config = dev->config;

	*status = (struct ltpi_ctrl_status_regs *)config->base;

	return 0;
}

static int ast10x0_g2_ltpi_init(const struct device *dev)
{
	const struct ast10x0_g2_ltpi_config *config = dev->config;
	struct ast10x0_g2_ltpi_data *data = dev->data;

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
	k_mutex_init(&data->lock);

	return 0;
}

static const struct ltpi_driver_api ast10x0_g2_ltpi_api = {
	.do_link = ast10x0_g2_ltpi_do_link,
	.get_status = ast10x0_g2_ltpi_get_status,
};

#define LTPI_AST10X0_G2_INIT(n)                                                                   \
	static const struct ast10x0_g2_ltpi_config ast10x0_g2_ltpi_config_##n = {                 \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.phy_base = DT_INST_REG_ADDR(n) + 0x200,                                           \
		.top_base = DT_INST_REG_ADDR(n) + 0x800,                                          \
		.scu_base = DT_REG_ADDR_BY_IDX(DT_INST_PHANDLE(n, aspeed_scu), 0),                \
		.reset.dev = DEVICE_DT_GET(DT_INST_RESET_CTLR(n)),                                \
		.reset.id = DT_INST_RESET_ID(n),                                                  \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                               \
		.clock_ahb = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, ahb, clk_id), \
		.clock_phy = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, phy, clk_id), \
	};                                                                                         \
	static struct ast10x0_g2_ltpi_data ast10x0_g2_ltpi_data##n;                               \
	DEVICE_DT_INST_DEFINE(n, ast10x0_g2_ltpi_init, NULL, &ast10x0_g2_ltpi_data##n,            \
			      &ast10x0_g2_ltpi_config_##n, POST_KERNEL,                           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ast10x0_g2_ltpi_api);

DT_INST_FOREACH_STATUS_OKAY(LTPI_AST10X0_G2_INIT)
