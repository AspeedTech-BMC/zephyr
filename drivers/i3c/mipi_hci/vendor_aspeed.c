/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_I3C_MIPI_HCI_ASPEED_VENDOR

#define DT_DRV_COMPAT aspeed_g7_i3c_hci

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_SOC_SERIES_AST27XX)
#include <soc.h>
#endif

#include "cmd.h"
#include "vendor_aspeed.h"

LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

#define ASPEED_I3C_DEFAULT_CORE_RATE_HZ 200000000U
#define ASPEED_I3C_DEFAULT_SCL_RATE_HZ 12500000U
#define ASPEED_I3C_BUS_THIGH_MAX_NS 41U
#define ASPEED_I3C_MIN_TBIT_NS 60U
#define ASPEED_I3C_MIPI_MFG_ID 0x3f6U
#define ASPEED_I3C_FALLBACK_PART_ID 0x2700U
#define ASPEED_I3C_DMA_IBI_CHUNK_SIZE 4U
#if defined(CONFIG_SOC_SERIES_AST27XX)
/* DRAM bus window the I3C DMA master can reach on G7 secondary cores. */
#define ASPEED_I3C_DMA_DRAM_PHYS_BASE 0x400000000ULL
#define ASPEED_I3C_DMA_DRAM_PHYS_LIMIT 0x500000000ULL
#endif

struct aspeed_i3c_dt_props {
	const struct device *dev;
	uintptr_t base_regs;
	uint32_t pp_scl_hi_period_ns;
	uint32_t pp_scl_lo_period_ns;
	uint32_t od_scl_hi_period_ns;
	uint32_t od_scl_lo_period_ns;
	uint32_t sda_tx_hold_ns;
	uint32_t internal_pullup;
	uint8_t assigned_addr;
};

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define ASPEED_I3C_DT_PROPS(inst)                                                           \
	{                                                                                    \
		.dev = DEVICE_DT_INST_GET(inst),                                             \
		.base_regs = DT_INST_REG_ADDR_BY_IDX(inst, 0),                               \
		.pp_scl_hi_period_ns = DT_INST_PROP_OR(inst, i3c_pp_scl_hi_period_ns, 0),    \
		.pp_scl_lo_period_ns = DT_INST_PROP_OR(inst, i3c_pp_scl_lo_period_ns, 0),    \
		.od_scl_hi_period_ns = DT_INST_PROP_OR(inst, i3c_od_scl_hi_period_ns, 0),    \
		.od_scl_lo_period_ns = DT_INST_PROP_OR(inst, i3c_od_scl_lo_period_ns, 0),    \
		.sda_tx_hold_ns = DT_INST_PROP_OR(inst, sda_tx_hold_ns, 0),                  \
		.internal_pullup = DT_INST_PROP_OR(inst, internal_pullup, 0),                \
		.assigned_addr = DT_INST_PROP_OR(inst, assigned_address, 0),                 \
	},

static const struct aspeed_i3c_dt_props aspeed_i3c_dt_props[] = {
	DT_INST_FOREACH_STATUS_OKAY(ASPEED_I3C_DT_PROPS)
};
#endif

#if defined(CONFIG_I3C_MIPI_HCI_ASPEED_AUTOCMD)
#define ASPEED_I3C_AUTOCMD_DESC_VALID BIT(31)
#define ASPEED_I3C_AUTOCMD_DESC_RNW BIT(30)
#define ASPEED_I3C_AUTOCMD_DESC_ROC BIT(29)
#define ASPEED_I3C_AUTOCMD_DESC_TOC BIT(28)
#define ASPEED_I3C_AUTOCMD_DESC_MODE GENMASK(27, 24)
#define ASPEED_I3C_AUTOCMD_DESC_TARGET_ADDR GENMASK(23, 17)
#define ASPEED_I3C_AUTOCMD_DESC_CCC_CMD GENMASK(16, 9)
#define ASPEED_I3C_AUTOCMD_DESC_DATA_LEN GENMASK(8, 0)
#define ASPEED_I3C_AUTOCMD_DESC_DATA_LEN_MAX 0x1ffU

#define ASPEED_I3C_AUTOCMD_SEL_FIELD_BITS 4U
#define ASPEED_I3C_AUTOCMD_SEL_PER_REG 8U
#define ASPEED_I3C_AUTOCMD_SEL_FIELD_MASK GENMASK(3, 0)
#define ASPEED_I3C_AUTOCMD_SEL_VALID BIT(3)
#define ASPEED_I3C_AUTOCMD_SEL_SLOT GENMASK(2, 0)
#define ASPEED_I3C_AUTOCMD_TRIGGER_UNUSED 0xffU

static inline uint32_t ast_autocmd_read(const struct i3c_hci *hci, uint32_t reg)
{
	return sys_read32((mem_addr_t)(hci->AUTOCMD_regs + reg));
}

static inline void ast_autocmd_write(const struct i3c_hci *hci, uint32_t reg,
				     uint32_t val)
{
	sys_write32(val, (mem_addr_t)(hci->AUTOCMD_regs + reg));
}

static uint32_t aspeed_i3c_autocmd_slot_reg(uint8_t slot)
{
	return ASPEED_I3C_AUTOCMD_0 + (uint32_t)slot * sizeof(uint32_t);
}

static uint32_t aspeed_i3c_autocmd_sel_reg(uint8_t ibi_addr)
{
	return ASPEED_I3C_AUTOCMD_SEL_0_7 +
	       ((uint32_t)ibi_addr / ASPEED_I3C_AUTOCMD_SEL_PER_REG) * sizeof(uint32_t);
}

static uint32_t aspeed_i3c_autocmd_sel_shift(uint8_t ibi_addr)
{
	return ((uint32_t)ibi_addr % ASPEED_I3C_AUTOCMD_SEL_PER_REG) *
	       ASPEED_I3C_AUTOCMD_SEL_FIELD_BITS;
}

static int aspeed_i3c_autocmd_require(struct i3c_hci *hci)
{
	if (!hci) {
		return -EINVAL;
	}

	if (hci->AUTOCMD_regs == 0U) {
		return -ENODEV;
	}

	if ((hci->caps & HC_CAP_AUTO_COMMAND) == 0U) {
		return -ENOTSUP;
	}

	return 0;
}

static uint32_t aspeed_i3c_autocmd_sel_value(uint8_t slot, bool enable)
{
	if (!enable) {
		return 0;
	}

	return ASPEED_I3C_AUTOCMD_SEL_VALID |
	       FIELD_PREP(ASPEED_I3C_AUTOCMD_SEL_SLOT, slot);
}

static void aspeed_i3c_autocmd_write_selector(struct i3c_hci *hci, uint8_t ibi_addr,
					      uint8_t slot, bool enable)
{
	uint32_t reg = aspeed_i3c_autocmd_sel_reg(ibi_addr);
	uint32_t shift = aspeed_i3c_autocmd_sel_shift(ibi_addr);
	uint32_t mask = ASPEED_I3C_AUTOCMD_SEL_FIELD_MASK << shift;
	uint32_t val = ast_autocmd_read(hci, reg);

	val &= ~mask;
	val |= (aspeed_i3c_autocmd_sel_value(slot, enable) <<
		shift) & mask;
	ast_autocmd_write(hci, reg, val);
}

static int aspeed_i3c_autocmd_pack(struct i3c_hci *hci,
				   const struct mipi_i3c_hci_autocmd_entry *entry,
				   uint32_t *desc)
{
	uint8_t mode;

	if (!entry || !desc) {
		return -EINVAL;
	}

	if (entry->slot >= I3C_HCI_AUTOCMD_SLOTS || entry->target_addr > 0x7fU ||
	    ((entry->flags & ~MIPI_I3C_HCI_AUTOCMD_FLAGS_MASK) != 0U)) {
		return -EINVAL;
	}

	if (entry->data_len > MIN(ASPEED_HC_PAYLOAD_LIMIT,
				  ASPEED_I3C_AUTOCMD_DESC_DATA_LEN_MAX)) {
		return -EFBIG;
	}

	switch (entry->mode) {
	case MIPI_I3C_HCI_AUTOCMD_MODE_SDR:
		mode = MIPI_I3C_HCI_AUTOCMD_MODE_SDR;
		break;
	case MIPI_I3C_HCI_AUTOCMD_MODE_HDR_DDR:
		if ((hci->caps & HC_CAP_HDR_DDR_EN) == 0U) {
			return -ENOTSUP;
		}
		mode = MIPI_I3C_HCI_AUTOCMD_MODE_HDR_DDR;
		break;
	case MIPI_I3C_HCI_AUTOCMD_MODE_I2C:
		mode = MIPI_I3C_HCI_AUTOCMD_MODE_I2C;
		break;
	default:
		return -EINVAL;
	}

	*desc = ASPEED_I3C_AUTOCMD_DESC_VALID |
		((entry->flags & MIPI_I3C_HCI_AUTOCMD_RNW) != 0U ?
			 ASPEED_I3C_AUTOCMD_DESC_RNW :
			 0U) |
		((entry->flags & MIPI_I3C_HCI_AUTOCMD_ROC) != 0U ?
			 ASPEED_I3C_AUTOCMD_DESC_ROC :
			 0U) |
		((entry->flags & MIPI_I3C_HCI_AUTOCMD_TOC) != 0U ?
			 ASPEED_I3C_AUTOCMD_DESC_TOC :
			 0U) |
		FIELD_PREP(ASPEED_I3C_AUTOCMD_DESC_MODE, mode) |
		FIELD_PREP(ASPEED_I3C_AUTOCMD_DESC_TARGET_ADDR, entry->target_addr) |
		FIELD_PREP(ASPEED_I3C_AUTOCMD_DESC_CCC_CMD, entry->ccc_or_cmd) |
		FIELD_PREP(ASPEED_I3C_AUTOCMD_DESC_DATA_LEN, entry->data_len);

	return 0;
}
#endif /* CONFIG_I3C_MIPI_HCI_ASPEED_AUTOCMD */

static const struct aspeed_i3c_dt_props *aspeed_i3c_get_dt_props(struct i3c_hci *hci)
{
	if (!hci) {
		return NULL;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
	for (size_t i = 0; i < ARRAY_SIZE(aspeed_i3c_dt_props); i++) {
		if (aspeed_i3c_dt_props[i].dev == hci->dev ||
		    aspeed_i3c_dt_props[i].base_regs == hci->base_regs) {
			return &aspeed_i3c_dt_props[i];
		}
	}
#else
	ARG_UNUSED(hci);
#endif

	return NULL;
}

static uint32_t aspeed_i3c_get_core_rate(struct i3c_hci *hci)
{
	uint32_t rate = 0;

	if (hci && hci->clk && device_is_ready(hci->clk)) {
		if (clock_control_get_rate(hci->clk, NULL, &rate) == 0 && rate != 0U) {
			return rate;
		}

		if (clock_control_get_rate(hci->clk, hci->clock_id, &rate) == 0 && rate != 0U) {
			return rate;
		}
	}

	return ASPEED_I3C_DEFAULT_CORE_RATE_HZ;
}

static uint32_t aspeed_i3c_core_period_ns(struct i3c_hci *hci)
{
	return DIV_ROUND_UP(1000000000U, aspeed_i3c_get_core_rate(hci));
}

static uint32_t aspeed_i3c_round_closest_count(uint32_t ns, uint32_t period_ns)
{
	if (ns == 0U || period_ns == 0U) {
		return 0;
	}

	return DIV_ROUND_CLOSEST(ns, period_ns);
}

static uint32_t aspeed_i3c_round_closest_count_minus_one(uint32_t ns, uint32_t period_ns)
{
	uint32_t count = aspeed_i3c_round_closest_count(ns, period_ns);

	return count > 0U ? count - 1U : 0U;
}

static uint32_t aspeed_i3c_round_up_count_minus_one(uint32_t ns, uint32_t period_ns)
{
	uint32_t count;

	if (ns == 0U || period_ns == 0U) {
		return 0;
	}

	count = DIV_ROUND_UP(ns, period_ns);
	return count > 0U ? count - 1U : 0U;
}

static uint32_t aspeed_i3c_get_scl_rate(struct i3c_hci *hci)
{
	if (hci && hci->common.ctrl_config.scl.i3c != 0U) {
		return hci->common.ctrl_config.scl.i3c;
	}

	return ASPEED_I3C_DEFAULT_SCL_RATE_HZ;
}

static bool aspeed_i3c_bus_is_pure(struct i3c_hci *hci)
{
	if (!hci || !hci->config) {
		return true;
	}

	return hci->config->common.dev_list.num_i2c == 0U;
}

int mipi_i3c_hci_aspeed_init(struct i3c_hci *hci)
{
	uint32_t init_mode;

	if (!hci || hci->VENDOR_regs == 0U) {
		return -EINVAL;
	}

	/* Quirks the generic layers must honour on this silicon. */
	hci->quirks |= HCI_QUIRK_RING_INTR_NO_PREWRITE;

	init_mode = hci->common.ctrl_config.is_secondary ? INIT_SEC_MST_MODE : INIT_MST_MODE;
	ast_inhouse_write(hci, ASPEED_I3C_CTRL,
			  ASPEED_I3C_CTRL_INIT |
				  FIELD_PREP(ASPEED_I3C_CTRL_INIT_MODE, init_mode));

	mipi_i3c_hci_aspeed_phy_init(hci);
	mipi_i3c_hci_aspeed_populate_bus_timing(hci);

	return 0;
}

void mipi_i3c_hci_aspeed_phy_init(struct i3c_hci *hci)
{
	uint32_t hcnt;
	uint32_t lcnt;
	uint32_t core_period;

	if (!hci || hci->PHY_regs == 0U) {
		return;
	}

	core_period = aspeed_i3c_core_period_ns(hci);

	hcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FM_DEFAULT_CAS_NS,
							core_period);
	lcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FM_DEFAULT_SU_STO_NS,
							core_period);
	ast_phy_write(hci, PHY_I2C_FM_CTRL0,
		      FIELD_PREP(PHY_I2C_FM_CTRL0_CAS, hcnt) |
			      FIELD_PREP(PHY_I2C_FM_CTRL0_SU_STO, lcnt));

	hcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FM_DEFAULT_SCL_H_NS,
							core_period);
	lcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FM_DEFAULT_SCL_L_NS,
							core_period);
	ast_phy_write(hci, PHY_I2C_FM_CTRL1,
		      FIELD_PREP(PHY_I2C_FM_CTRL1_SCL_H, hcnt) |
			      FIELD_PREP(PHY_I2C_FM_CTRL1_SCL_L, lcnt));
	ast_phy_write(hci, PHY_I2C_FM_CTRL2,
		      FIELD_PREP(PHY_I2C_FM_CTRL2_ACK_H, hcnt) |
			      FIELD_PREP(PHY_I2C_FM_CTRL2_ACK_L, hcnt));

	hcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FM_DEFAULT_HD_DAT,
							core_period);
	lcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FM_DEFAULT_AHD_DAT,
							core_period);
	ast_phy_write(hci, PHY_I2C_FM_CTRL3,
		      FIELD_PREP(PHY_I2C_FM_CTRL3_HD_DAT, hcnt) |
			      FIELD_PREP(PHY_I2C_FM_CTRL3_AHD_DAT, lcnt));

	hcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FMP_DEFAULT_CAS_NS,
							core_period);
	lcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FMP_DEFAULT_SU_STO_NS,
							core_period);
	ast_phy_write(hci, PHY_I2C_FMP_CTRL0,
		      FIELD_PREP(PHY_I2C_FMP_CTRL0_CAS, hcnt) |
			      FIELD_PREP(PHY_I2C_FMP_CTRL0_SU_STO, lcnt));

	hcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FMP_DEFAULT_SCL_H_NS,
							core_period);
	lcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FMP_DEFAULT_SCL_L_NS,
							core_period);
	ast_phy_write(hci, PHY_I2C_FMP_CTRL1,
		      FIELD_PREP(PHY_I2C_FMP_CTRL1_SCL_H, hcnt) |
			      FIELD_PREP(PHY_I2C_FMP_CTRL1_SCL_L, lcnt));
	ast_phy_write(hci, PHY_I2C_FMP_CTRL2,
		      FIELD_PREP(PHY_I2C_FMP_CTRL2_ACK_H, hcnt) |
			      FIELD_PREP(PHY_I2C_FMP_CTRL2_ACK_L, hcnt));

	hcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FMP_DEFAULT_HD_DAT,
							core_period);
	lcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FMP_DEFAULT_AHD_DAT,
							core_period);
	ast_phy_write(hci, PHY_I2C_FMP_CTRL3,
		      FIELD_PREP(PHY_I2C_FMP_CTRL3_HD_DAT, hcnt) |
			      FIELD_PREP(PHY_I2C_FMP_CTRL3_AHD_DAT, lcnt));

	ast_phy_write(hci, PHY_PULLUP_EN, 0);
}

void mipi_i3c_hci_aspeed_populate_bus_timing(struct i3c_hci *hci)
{
	const struct aspeed_i3c_dt_props *props;
	uint32_t hcnt;
	uint32_t lcnt;
	uint32_t total_cnt;
	uint32_t min_tbit_cnt;
	uint32_t core_rate;
	uint32_t core_period;
	uint32_t i3c_rate;
	uint32_t ctrl0;
	uint32_t ctrl1;
	uint32_t ctrl2;
	uint32_t sdr_ctrl0_reg;
	uint32_t pp_high = 0;
	uint32_t pp_low = 0;
	uint32_t od_high = 0;
	uint32_t od_low = 0;
	uint32_t thd_dat = 0;
	uint32_t internal_pu = 0;

	if (!hci || hci->PHY_regs == 0U) {
		return;
	}

	props = aspeed_i3c_get_dt_props(hci);
	if (props) {
		pp_high = props->pp_scl_hi_period_ns;
		pp_low = props->pp_scl_lo_period_ns;
		od_high = props->od_scl_hi_period_ns;
		od_low = props->od_scl_lo_period_ns;
		thd_dat = props->sda_tx_hold_ns;
		internal_pu = props->internal_pullup;
	}

	core_rate = aspeed_i3c_get_core_rate(hci);
	core_period = DIV_ROUND_UP(1000000000U, core_rate);
	i3c_rate = aspeed_i3c_get_scl_rate(hci);
	min_tbit_cnt = aspeed_i3c_round_up_count_minus_one(ASPEED_I3C_MIN_TBIT_NS, core_period);
	sdr_ctrl0_reg = mipi_i3c_hci_aspeed_get_sdr_phy_reg(hci);

	if (pp_high != 0U && pp_low != 0U) {
		hcnt = aspeed_i3c_round_closest_count_minus_one(pp_high, core_period);
		lcnt = aspeed_i3c_round_closest_count_minus_one(pp_low, core_period);
	} else {
		total_cnt = DIV_ROUND_UP(core_rate, i3c_rate);
		total_cnt = total_cnt > 2U ? total_cnt - 2U : 0U;

		if (aspeed_i3c_bus_is_pure(hci)) {
			hcnt = (uint32_t)(((uint64_t)total_cnt * 2U) / 5U);
		} else {
			hcnt = aspeed_i3c_round_up_count_minus_one(ASPEED_I3C_BUS_THIGH_MAX_NS,
								   core_period);
		}

		lcnt = total_cnt > hcnt ? total_cnt - hcnt : 0U;
	}

	ctrl0 = FIELD_PREP(PHY_I3C_SDR0_CTRL0_SCL_H, hcnt) |
		FIELD_PREP(PHY_I3C_SDR0_CTRL0_SCL_L, lcnt);
	ast_phy_write(hci, sdr_ctrl0_reg + PHY_I3C_CTRL0_OFFSET, ctrl0);
	ast_phy_write(hci, PHY_I3C_SDR0_CTRL0, ctrl0);
	ast_phy_write(hci, PHY_I3C_DDR_CTRL0, ctrl0);

	ctrl1 = FIELD_PREP(PHY_I3C_SDR0_CTRL1_TBIT_H, MAX(hcnt, min_tbit_cnt)) |
		FIELD_PREP(PHY_I3C_SDR0_CTRL1_TBIT_L, MAX(lcnt, min_tbit_cnt));
	ast_phy_write(hci, sdr_ctrl0_reg + PHY_I3C_CTRL1_OFFSET, ctrl1);
	ast_phy_write(hci, PHY_I3C_SDR0_CTRL1, ctrl1);
	ast_phy_write(hci, PHY_I3C_DDR_CTRL1, ctrl1);

	ast_phy_write(hci, PHY_I3C_OD_CTRL0,
		      FIELD_PREP(PHY_I3C_OD_CTRL0_CAS, hcnt) |
			      FIELD_PREP(PHY_I3C_OD_CTRL0_CBP, hcnt));

	hcnt = aspeed_i3c_round_closest_count(PHY_I3C_SR_P_DEFAULT_HD_NS, core_period);
	ast_phy_write(hci, PHY_I3C_SR_P_PREPARE_CTRL,
		      FIELD_PREP(PHY_I3C_SR_P_PREPARE_CTRL_HD, hcnt) |
			      FIELD_PREP(PHY_I3C_SR_P_PREPARE_CTRL_SCL_L, lcnt));

	if (od_high != 0U && od_low != 0U) {
		hcnt = aspeed_i3c_round_closest_count_minus_one(od_high, core_period);
		lcnt = aspeed_i3c_round_closest_count_minus_one(od_low, core_period);
	} else {
		hcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FMP_DEFAULT_SCL_H_NS,
								core_period);
		lcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I2C_FMP_DEFAULT_SCL_L_NS,
								core_period);
	}

	ast_phy_write(hci, PHY_I3C_OD_CTRL1,
		      FIELD_PREP(PHY_I3C_OD_CTRL1_SCL_H, hcnt) |
			      FIELD_PREP(PHY_I3C_OD_CTRL1_SCL_L, lcnt));
	ast_phy_write(hci, PHY_I3C_OD_CTRL2,
		      FIELD_PREP(PHY_I3C_OD_CTRL2_ACK_H, hcnt) |
			      FIELD_PREP(PHY_I3C_OD_CTRL2_ACK_L, hcnt));

	if (thd_dat != 0U) {
		hcnt = aspeed_i3c_round_closest_count(thd_dat, core_period);
		lcnt = hcnt;
	} else {
		hcnt = aspeed_i3c_round_closest_count(PHY_I3C_OD_DEFAULT_HD_DAT, core_period);
		lcnt = aspeed_i3c_round_closest_count(PHY_I3C_OD_DEFAULT_AHD_DAT, core_period);
	}

	ctrl2 = FIELD_PREP(PHY_I3C_SDR0_CTRL2_HD_PP, hcnt) |
		FIELD_PREP(PHY_I3C_SDR0_CTRL2_TBIT_HD_PP, lcnt);
	ast_phy_write(hci, sdr_ctrl0_reg + PHY_I3C_CTRL2_OFFSET, ctrl2);
	ast_phy_write(hci, PHY_I3C_SDR0_CTRL2, ctrl2);
	ast_phy_write(hci, PHY_I3C_DDR_CTRL2, ctrl2);

	ast_phy_write(hci, PHY_I3C_OD_CTRL3,
		      FIELD_PREP(PHY_I3C_OD_CTRL3_HD_DAT, hcnt) |
			      FIELD_PREP(PHY_I3C_OD_CTRL3_AHD_DAT, lcnt));

	hcnt = aspeed_i3c_round_closest_count_minus_one(PHY_I3C_OD_DEFAULT_DAP_NS,
							core_period);
	ast_phy_write(hci, PHY_I3C_OD_CTRL4, FIELD_PREP(PHY_I3C_OD_CTRL4_DAP, hcnt));

	if (internal_pu != 0U) {
		ast_phy_write(hci, PHY_SW_FORCE_CTRL,
			      PHY_SW_FORCE_CTRL_SCL_PU_EN | PHY_SW_FORCE_CTRL_SDA_PU_EN |
				      FIELD_PREP(PHY_SW_FORCE_CTRL_SCL_PU_VAL, internal_pu) |
				      FIELD_PREP(PHY_SW_FORCE_CTRL_SDA_PU_VAL, internal_pu));
	}
}

uint32_t mipi_i3c_hci_aspeed_get_sdr_phy_reg(struct i3c_hci *hci)
{
	uint32_t scl_rate = aspeed_i3c_get_scl_rate(hci);

	if (scl_rate > 8000000U) {
		return PHY_I3C_SDR0_CTRL0;
	}

	if (scl_rate > 6000000U) {
		return PHY_I3C_SDR1_CTRL0;
	}

	if (scl_rate > 4000000U) {
		return PHY_I3C_SDR2_CTRL0;
	}

	if (scl_rate > 2000000U) {
		return PHY_I3C_SDR3_CTRL0;
	}

	return PHY_I3C_SDR4_CTRL0;
}

void mipi_i3c_hci_aspeed_set_ibi_terminate_len(struct i3c_hci *hci, uint16_t max_len)
{
	uint32_t reg;
	uint16_t terminate_len;

	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	reg = ast_inhouse_read(hci, ASPEED_I3C_MST_MRL);
	terminate_len = MAX(max_len, (uint16_t)FIELD_GET(ASPEED_I3C_IBI_TERMINATE_LEN, reg));
	ast_inhouse_write(hci, ASPEED_I3C_MST_MRL,
			  ASPEED_I3C_IBI_TERMINATE_EN |
				  FIELD_PREP(ASPEED_I3C_IBI_TERMINATE_LEN, terminate_len));
}

void mipi_i3c_hci_aspeed_set_slv_pid(struct i3c_hci *hci, uint64_t pid)
{
	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	ast_inhouse_write(hci, ASPEED_I3C_SLV_PID_LO, SLV_PID_LO(pid));
	ast_inhouse_write(hci, ASPEED_I3C_SLV_PID_HI, SLV_PID_HI(pid));
}

void mipi_i3c_hci_aspeed_set_slv_char_ctrl(struct i3c_hci *hci, uint8_t bcr, uint8_t dcr,
					   bool static_addr_en)
{
	const struct aspeed_i3c_dt_props *props;
	uint32_t reg;
	uint8_t static_addr = 0;

	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	props = aspeed_i3c_get_dt_props(hci);
	if (props) {
		static_addr = props->assigned_addr;
	}

	reg = FIELD_PREP(ASPEED_I3C_SLV_CHAR_CTRL_DCR, dcr) |
	      FIELD_PREP(ASPEED_I3C_SLV_CHAR_CTRL_BCR, bcr);
	if (static_addr_en) {
		reg |= ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR_EN |
		       FIELD_PREP(ASPEED_I3C_SLV_CHAR_CTRL_STATIC_ADDR, static_addr);
	}

	ast_inhouse_write(hci, ASPEED_I3C_SLV_CHAR_CTRL, reg);
}

bool mipi_i3c_hci_aspeed_payload_too_big(unsigned int data_len)
{
	return data_len > ASPEED_HC_PAYLOAD_LIMIT;
}

uint32_t mipi_i3c_hci_aspeed_get_status(struct i3c_hci *hci)
{
	if (!hci || hci->VENDOR_regs == 0U) {
		return 0;
	}

	return ast_inhouse_read(hci, ASPEED_I3C_STS);
}

void mipi_i3c_hci_aspeed_dma_start(struct i3c_hci *hci)
{
	/*
	 * The G7 HCI auto-fetches the new ring entry once RING_OPERATION1
	 * is updated. Previously poking I3C_DMA_DBG_LO_START here caused
	 * the engine to "complete" with a zero-TID response without driving
	 * the bus. Keep this as a deliberate no-op so the engine drives the
	 * I3C bus the way Linux's ast2700 driver does.
	 */
	ARG_UNUSED(hci);
}

#define ASPEED_DMA_DRAIN_TIMEOUT_US 100000U

void mipi_i3c_hci_aspeed_dma_drain(struct i3c_hci *hci)
{
	/*
	 * Match Linux upstream aspeed_pio_fifo_reset(): wait for the WDMA
	 * and RDMA engines to drain rather than writing I3C_DMA_DBG_LO_ABORT.
	 * Writing the ABORT bit causes HW to fire an extra INTR_TRANSFER_ABORT,
	 * which the IRQ handler then surfaces as a spurious "DMA ring %u
	 * transfer aborted" warning on every bus-level NACK (e.g. end-of-DAA).
	 * Linux notes that the interrupt is raised before the DMA engine
	 * completes the transfer, so the right action is to poll BUSY.
	 */
	(void)WAIT_FOR(!(sys_read32((mem_addr_t)(hci->base_regs +
					    ASPEED_I3C_WDMA_DBG_LO)) &
		       I3C_DMA_DBG_LO_BUSY),
		      ASPEED_DMA_DRAIN_TIMEOUT_US, k_busy_wait(1));
	(void)WAIT_FOR(!(sys_read32((mem_addr_t)(hci->base_regs +
					    ASPEED_I3C_RDMA_DBG_LO)) &
		       I3C_DMA_DBG_LO_BUSY),
		      ASPEED_DMA_DRAIN_TIMEOUT_US, k_busy_wait(1));
}

uint32_t mipi_i3c_hci_aspeed_ring_status(struct i3c_hci *hci)
{
	if (!hci || hci->base_regs == 0U) {
		return I3C_RING_IDLE;
	}

	/* RING_STATUS is in the HC base region, not the INHOUSE block. */
	return sys_read32((mem_addr_t)(hci->base_regs + ASPEED_I3C_RING_STATUS));
}

void mipi_i3c_hci_aspeed_ccc_handler(struct i3c_hci *hci, uint8_t ccc)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(ccc);
}

static bool mipi_i3c_hci_aspeed_pio_ibi_thld_status_only(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);

	/*
	 * On Aspeed G7 silicon STAT_IBI_STATUS_THLD only reflects the IBI
	 * status FIFO, not the IBI data FIFO. The PIO IBI handler must drain
	 * a segment using the length the status word advertised instead of
	 * polling STATUS_THLD between data reads, and the IBI data threshold
	 * has to track the IBI status threshold so STATUS_THLD still asserts
	 * for small payloads.
	 */
	return true;
}

/*
 * The G7 in-house DAA control register is a bitmap indexed by the
 * I3C dynamic address being assigned, NOT by the DAT slot. Each ENTDAA
 * iteration must set BIT(dynamic_addr) in DAA_INDEX[dynamic_addr / 32]
 * before the command goes on the bus, so the controller knows which
 * address the freshly-discovered target should claim.
 */
static void mipi_i3c_hci_aspeed_prep_daa_step(struct i3c_hci *hci, uint8_t dynamic_addr)
{
	uint32_t reg;

	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	if (dynamic_addr < 32U) {
		reg = ASPEED_I3C_DAA_INDEX0;
	} else if (dynamic_addr < 64U) {
		reg = ASPEED_I3C_DAA_INDEX1;
		dynamic_addr -= 32U;
	} else if (dynamic_addr < 96U) {
		reg = ASPEED_I3C_DAA_INDEX2;
		dynamic_addr -= 64U;
	} else {
		reg = ASPEED_I3C_DAA_INDEX3;
		dynamic_addr -= 96U;
	}

	ast_inhouse_write(hci, reg, BIT(dynamic_addr));
}

/*
 * ASPEED G7 silicon expects the DAT entry's index to equal the
 * dynamic address (the in-house DAA register and the response-side
 * IBI routing both index into DAT by dynamic address). Return the
 * caller-supplied address as the preferred allocator slot; if it is
 * already taken the dat_v1 allocator will fall back to first-free
 * and dat_wants_addr_indexed() will relocate the entry during
 * set_dynamic_addr().
 */
static int mipi_i3c_hci_aspeed_dat_pick_slot(struct i3c_hci *hci, uint8_t address)
{
	ARG_UNUSED(hci);
	return (int)address;
}

static bool mipi_i3c_hci_aspeed_dat_wants_addr_indexed(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
	return true;
}

static uint32_t mipi_i3c_hci_aspeed_read_inhouse_summary(struct i3c_hci *hci)
{
	uint32_t status;
	uint32_t summary = 0U;

	if (!hci || hci->VENDOR_regs == 0U) {
		return 0U;
	}

	status = ast_inhouse_read(hci, ASPEED_I3C_INTR_SUM_STATUS);
	if ((status & ASPEED_INTR_SUM_CAP) != 0U) {
		summary |= MIPI_I3C_HCI_VENDOR_IRQ_CORE;
	}
	if ((status & (ASPEED_INTR_SUM_PIO | ASPEED_INTR_SUM_RHS)) != 0U) {
		summary |= MIPI_I3C_HCI_VENDOR_IRQ_IO;
	}
	if ((status & ASPEED_INTR_SUM_INHOUSE) != 0U) {
		summary |= MIPI_I3C_HCI_VENDOR_IRQ_PRIV;
	}

	return summary;
}

static uint32_t mipi_i3c_hci_aspeed_read_inhouse_status(struct i3c_hci *hci)
{
	if (!hci || hci->VENDOR_regs == 0U) {
		return 0U;
	}

	return ast_inhouse_read(hci, ASPEED_I3C_INTR_STATUS);
}

static void mipi_i3c_hci_aspeed_clear_inhouse_status(struct i3c_hci *hci,
						     uint32_t status)
{
	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	ast_inhouse_write(hci, ASPEED_I3C_INTR_STATUS, status);
}

static void mipi_i3c_hci_aspeed_renew_inhouse_irq(struct i3c_hci *hci)
{
	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	ast_inhouse_write(hci, ASPEED_I3C_INTR_RENEW, 1);
}

static void mipi_i3c_hci_aspeed_disable_inhouse_irq(struct i3c_hci *hci)
{
	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	ast_inhouse_write(hci, ASPEED_I3C_INTR_SIGNAL_ENABLE, 0);
	ast_inhouse_write(hci, ASPEED_I3C_INTR_STATUS_ENABLE, 0xffffffff);
}

static void mipi_i3c_hci_handle_aspeed_inhouse_irq(struct i3c_hci *hci,
						   uint32_t status)
{
	uint32_t stuck_mask = ASPEED_I3C_INTR_I2C_SDA_STUCK_LOW |
			      ASPEED_I3C_INTR_I3C_SDA_STUCK_HIGH |
			      ASPEED_I3C_INTR_I3C_SDA_STUCK_LOW |
			      ASPEED_I3C_INTR_SLV_SCL_STUCK;

	if ((status & ASPEED_I3C_INTR_MST_IBI_DONE) != 0U) {
		mipi_i3c_hci_vendor_event(hci, MIPI_I3C_HCI_EVENT_IBI_DONE);
	}

	if ((status & (ASPEED_I3C_INTR_MST_READ_DONE |
		       ASPEED_I3C_INTR_MST_DDR_READ_DONE)) != 0U) {
		mipi_i3c_hci_vendor_event(hci, MIPI_I3C_HCI_EVENT_PENDING_READ_DONE);
	}

	if ((status & stuck_mask) != 0U) {
		mipi_i3c_hci_vendor_event(hci, MIPI_I3C_HCI_EVENT_BUS_STUCK);
	}
}

static void mipi_i3c_hci_aspeed_dma_init(struct i3c_hci *hci)
{
	/*
	 * The G7 HCI auto-enables DMA once HC_CONTROL.PIO_MODE is cleared.
	 * The Linux ast2700 driver never writes WDMA/RDMA CTRL, and writing
	 * them here previously caused the engine to "complete" with bogus
	 * zero-TID responses while never driving SDA. Keep this a deliberate
	 * no-op.
	 */
	ARG_UNUSED(hci);
}

static void mipi_i3c_hci_aspeed_dma_log_status(struct i3c_hci *hci,
					       const char *reason)
{
	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	LOG_DBG("%s: ring=%#x wdma_dbg=%#x rdma_dbg=%#x", reason,
		mipi_i3c_hci_aspeed_ring_status(hci),
		ast_inhouse_read(hci, ASPEED_I3C_WDMA_DBG_LO),
		ast_inhouse_read(hci, ASPEED_I3C_RDMA_DBG_LO));
}

static bool mipi_i3c_hci_aspeed_dma_recovery_done(struct i3c_hci *hci)
{
	uint32_t status = mipi_i3c_hci_aspeed_ring_status(hci);

	return status == I3C_RING_IDLE || status == I3C_RING_ABORT;
}

static unsigned int mipi_i3c_hci_aspeed_ibi_chunk_size(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);

	return ASPEED_I3C_DMA_IBI_CHUNK_SIZE;
}

static void mipi_i3c_hci_aspeed_pio_log_prog_error(struct i3c_hci *hci,
						   uint32_t status)
{
	uint32_t queue;
	uint32_t data;

	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	queue = ast_inhouse_read(hci, ASPEED_I3C_QUEUE_PTR0);
	data = ast_inhouse_read(hci, ASPEED_I3C_QUEUE_PTR1);

	LOG_ERR("PIO prog error %#x C/R/I=%u:%u/%u:%u/%u:%u "
		"TX/RX/IBI=%u:%u/%u:%u/%u:%u",
		(uint32_t)status,
		(unsigned int)QUEUE_PTR0_CMD_W(queue),
		(unsigned int)QUEUE_PTR0_CMD_R(queue),
		(unsigned int)QUEUE_PTR0_RESP_W(queue),
		(unsigned int)QUEUE_PTR0_RESP_R(queue),
		(unsigned int)QUEUE_PTR0_IBI_W(queue),
		(unsigned int)QUEUE_PTR0_IBI_R(queue),
		(unsigned int)QUEUE_PTR0_TX_W(queue),
		(unsigned int)QUEUE_PTR0_TX_R(queue),
		(unsigned int)QUEUE_PTR1_RX_W(data),
		(unsigned int)QUEUE_PTR1_RX_R(data),
		(unsigned int)QUEUE_PTR1_IBI_DATA_W(data),
		(unsigned int)QUEUE_PTR1_IBI_DATA_R(data));
}

static bool mipi_i3c_hci_aspeed_status_changed_role(struct i3c_hci *hci,
						    uint32_t old_status,
						    uint32_t status)
{
	uint32_t role_mask = ASPEED_I3C_STS_MODE_PURE_SLV |
			     ASPEED_I3C_STS_MODE_SECONDARY_SLV_TO_MST |
			     ASPEED_I3C_STS_MODE_SECONDARY_MST_TO_SLV |
			     ASPEED_I3C_STS_MODE_SECONDARY_SLV |
			     ASPEED_I3C_STS_MODE_SECONDARY_MST |
			     ASPEED_I3C_STS_MODE_PRIMARY_SLV_TO_MST |
			     ASPEED_I3C_STS_MODE_PRIMARY_MST_TO_SLV |
			     ASPEED_I3C_STS_MODE_PRIMARY_SLV |
			     ASPEED_I3C_STS_MODE_PRIMARY_MST;

	ARG_UNUSED(hci);

	return ((status ^ old_status) & role_mask) != 0U;
}

static uint64_t mipi_i3c_hci_aspeed_target_pid(struct i3c_hci *hci,
					       uint16_t extra_info)
{
	uint32_t part_id = hci ? hci->vendor_product_id : 0U;

	if (part_id == 0U) {
		part_id = ASPEED_I3C_FALLBACK_PART_ID;
	}

	return ((uint64_t)ASPEED_I3C_MIPI_MFG_ID << 33) |
	       ((uint64_t)(part_id & GENMASK(15, 0)) << 16) |
	       (extra_info & GENMASK(11, 0));
}

static uint8_t mipi_i3c_hci_aspeed_target_dynamic_addr(struct i3c_hci *hci)
{
	uint32_t status = mipi_i3c_hci_aspeed_get_status(hci);

	if ((status & ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS_VALID) == 0U) {
		return 0U;
	}

	return FIELD_GET(ASPEED_I3C_STS_SLV_DYNAMIC_ADDRESS, status);
}

static bool mipi_i3c_hci_aspeed_target_event_bits(enum mipi_i3c_hci_target_event event,
						  uint32_t *sts1_bit, uint32_t *req_bit)
{
	switch (event) {
	case MIPI_I3C_HCI_TARGET_EVENT_IBI:
		*sts1_bit = ASPEED_I3C_SLV_STS1_IBI_EN;
		*req_bit = ASPEED_I3C_SLV_CAP_CTRL_IBI_REQ;
		return true;
	case MIPI_I3C_HCI_TARGET_EVENT_HOTJOIN:
		*sts1_bit = ASPEED_I3C_SLV_STS1_HJ_EN;
		*req_bit = ASPEED_I3C_SLV_CAP_CTRL_HJ_REQ;
		return true;
	case MIPI_I3C_HCI_TARGET_EVENT_MASTER_REQUEST:
		*sts1_bit = ASPEED_I3C_SLV_STS1_CR_EN;
		*req_bit = ASPEED_I3C_SLV_CAP_CTRL_MR_REQ;
		return true;
	default:
		return false;
	}
}

static bool mipi_i3c_hci_aspeed_target_event_enabled(struct i3c_hci *hci,
						     enum mipi_i3c_hci_target_event event)
{
	uint32_t sts1_bit;
	uint32_t req_bit;

	if (!mipi_i3c_hci_aspeed_target_event_bits(event, &sts1_bit, &req_bit)) {
		return false;
	}

	ARG_UNUSED(req_bit);

	if (!hci || hci->VENDOR_regs == 0U) {
		return true;
	}

	return (ast_inhouse_read(hci, ASPEED_I3C_SLV_STS1) & sts1_bit) != 0U;
}

static int mipi_i3c_hci_aspeed_target_request_event(struct i3c_hci *hci,
						    enum mipi_i3c_hci_target_event event)
{
	uint32_t sts1_bit;
	uint32_t req_bit;
	uint32_t reg;

	if (!mipi_i3c_hci_aspeed_target_event_bits(event, &sts1_bit, &req_bit)) {
		return -EINVAL;
	}

	ARG_UNUSED(sts1_bit);

	if (!hci || hci->VENDOR_regs == 0U) {
		return -ENOTSUP;
	}

	reg = ast_inhouse_read(hci, ASPEED_I3C_SLV_CAP_CTRL);
	ast_inhouse_write(hci, ASPEED_I3C_SLV_CAP_CTRL, reg | req_bit);

	return 0;
}

static bool mipi_i3c_hci_aspeed_target_request_pending(struct i3c_hci *hci,
						       enum mipi_i3c_hci_target_event event)
{
	uint32_t sts1_bit;
	uint32_t req_bit;

	if (!mipi_i3c_hci_aspeed_target_event_bits(event, &sts1_bit, &req_bit)) {
		return false;
	}

	ARG_UNUSED(sts1_bit);

	if (!hci || hci->VENDOR_regs == 0U) {
		return false;
	}

	return (ast_inhouse_read(hci, ASPEED_I3C_SLV_CAP_CTRL) & req_bit) != 0U;
}

static void mipi_i3c_hci_aspeed_target_enable_events(struct i3c_hci *hci,
						     uint8_t hdr_caps)
{
	uint32_t reg;

	if (!hci || hci->VENDOR_regs == 0U) {
		return;
	}

	reg = ast_inhouse_read(hci, ASPEED_I3C_SLV_CAP_CTRL);
	reg |= ASPEED_I3C_SLV_CAP_CTRL_IBI_WAIT |
	       ASPEED_I3C_SLV_CAP_CTRL_HJ_WAIT |
	       ASPEED_I3C_SLV_CAP_CTRL_MR_WAIT |
	       ASPEED_I3C_SLV_CAP_CTRL_ACCEPT_CR;
	ast_inhouse_write(hci, ASPEED_I3C_SLV_CAP_CTRL, reg);
	ast_inhouse_write(hci, ASPEED_I3C_SLV_STS8_GETCAPS_TGT, hdr_caps);
}

static void mipi_i3c_hci_aspeed_target_set_mode(struct i3c_hci *hci)
{
	if (!hci || hci->is_secondary || hci->VENDOR_regs == 0U) {
		return;
	}

	ast_inhouse_write(hci, ASPEED_I3C_CTRL,
			  ASPEED_I3C_CTRL_INIT |
				  FIELD_PREP(ASPEED_I3C_CTRL_INIT_MODE,
					     INIT_SLV_MODE));
}

static bool mipi_i3c_hci_aspeed_target_get_role(struct i3c_hci *hci,
						bool *secondary, bool *target)
{
	uint32_t status;

	if (!hci || hci->VENDOR_regs == 0U || !secondary || !target) {
		return false;
	}

	status = mipi_i3c_hci_aspeed_get_status(hci);
	*secondary = (status & (ASPEED_I3C_STS_MODE_SECONDARY_SLV |
				ASPEED_I3C_STS_MODE_SECONDARY_MST |
				ASPEED_I3C_STS_MODE_SECONDARY_SLV_TO_MST |
				ASPEED_I3C_STS_MODE_SECONDARY_MST_TO_SLV)) != 0U;
	*target = *secondary || (status & ASPEED_I3C_STS_MODE_PURE_SLV) != 0U;

	return true;
}

#if defined(CONFIG_I3C_MIPI_HCI_ASPEED_AUTOCMD)
int mipi_i3c_hci_aspeed_autocmd_init(struct i3c_hci *hci)
{
	k_spinlock_key_t key;

	if (!hci) {
		return -EINVAL;
	}

	key = k_spin_lock(&hci->lock);
	hci->autocmd.installed = 0U;
	hci->autocmd.enabled = 0U;
	(void)memset(hci->autocmd.desc, 0, sizeof(hci->autocmd.desc));
	(void)memset(hci->autocmd.trigger_slot, ASPEED_I3C_AUTOCMD_TRIGGER_UNUSED,
		      sizeof(hci->autocmd.trigger_slot));
	k_spin_unlock(&hci->lock, key);

	if (hci->AUTOCMD_regs == 0U) {
		return 0;
	}

	if ((hci->caps & HC_CAP_AUTO_COMMAND) == 0U) {
		return 0;
	}

	for (uint8_t slot = 0U; slot < I3C_HCI_AUTOCMD_SLOTS; slot++) {
		ast_autocmd_write(hci, aspeed_i3c_autocmd_slot_reg(slot), 0);
	}

	for (uint8_t addr = 0U; addr < I3C_HCI_AUTOCMD_TRIGGERS;
	     addr += ASPEED_I3C_AUTOCMD_SEL_PER_REG) {
		ast_autocmd_write(hci, aspeed_i3c_autocmd_sel_reg(addr), 0);
	}

	return 0;
}

int mipi_i3c_hci_aspeed_autocmd_install(struct i3c_hci *hci,
					const struct mipi_i3c_hci_autocmd_entry *entry)
{
	k_spinlock_key_t key;
	uint32_t desc;
	int ret;

	ret = aspeed_i3c_autocmd_require(hci);
	if (ret != 0) {
		return ret;
	}

	ret = aspeed_i3c_autocmd_pack(hci, entry, &desc);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&hci->lock);
	ast_autocmd_write(hci, aspeed_i3c_autocmd_slot_reg(entry->slot), desc);
	hci->autocmd.desc[entry->slot] = desc;
	hci->autocmd.installed |= (uint8_t)BIT(entry->slot);
	k_spin_unlock(&hci->lock, key);

	return 0;
}

int mipi_i3c_hci_aspeed_autocmd_remove(struct i3c_hci *hci, uint8_t slot)
{
	k_spinlock_key_t key;
	int ret;

	ret = aspeed_i3c_autocmd_require(hci);
	if (ret != 0) {
		return ret;
	}

	if (slot >= I3C_HCI_AUTOCMD_SLOTS) {
		return -EINVAL;
	}

	key = k_spin_lock(&hci->lock);
	ast_autocmd_write(hci, aspeed_i3c_autocmd_slot_reg(slot), 0);
	hci->autocmd.desc[slot] = 0U;
	hci->autocmd.installed &= (uint8_t)~BIT(slot);
	hci->autocmd.enabled &= (uint8_t)~BIT(slot);

	for (uint8_t addr = 0U; addr < I3C_HCI_AUTOCMD_TRIGGERS; addr++) {
		if (hci->autocmd.trigger_slot[addr] != slot) {
			continue;
		}

		hci->autocmd.trigger_slot[addr] = ASPEED_I3C_AUTOCMD_TRIGGER_UNUSED;
		aspeed_i3c_autocmd_write_selector(hci, addr, slot, false);
	}

	k_spin_unlock(&hci->lock, key);

	return 0;
}

int mipi_i3c_hci_aspeed_autocmd_enable(struct i3c_hci *hci, uint8_t slot, bool enable)
{
	k_spinlock_key_t key;
	uint8_t slot_bit;
	int ret;

	ret = aspeed_i3c_autocmd_require(hci);
	if (ret != 0) {
		return ret;
	}

	if (slot >= I3C_HCI_AUTOCMD_SLOTS) {
		return -EINVAL;
	}

	slot_bit = (uint8_t)BIT(slot);

	key = k_spin_lock(&hci->lock);
	if ((hci->autocmd.installed & slot_bit) == 0U) {
		k_spin_unlock(&hci->lock, key);
		return -ENOENT;
	}

	if (enable) {
		hci->autocmd.enabled |= slot_bit;
	} else {
		hci->autocmd.enabled &= (uint8_t)~slot_bit;
	}

	for (uint8_t addr = 0U; addr < I3C_HCI_AUTOCMD_TRIGGERS; addr++) {
		if (hci->autocmd.trigger_slot[addr] == slot) {
			aspeed_i3c_autocmd_write_selector(hci, addr, slot, enable);
		}
	}

	k_spin_unlock(&hci->lock, key);

	return 0;
}

void mipi_i3c_hci_aspeed_autocmd_set_trigger(struct i3c_hci *hci, uint8_t slot,
					     uint8_t ibi_addr)
{
	k_spinlock_key_t key;
	bool enable;

	if ((aspeed_i3c_autocmd_require(hci) != 0) ||
	    slot >= I3C_HCI_AUTOCMD_SLOTS ||
	    ibi_addr >= I3C_HCI_AUTOCMD_TRIGGERS) {
		return;
	}

	key = k_spin_lock(&hci->lock);
	if ((hci->autocmd.installed & BIT(slot)) == 0U) {
		k_spin_unlock(&hci->lock, key);
		return;
	}

	enable = (hci->autocmd.enabled & BIT(slot)) != 0U;
	hci->autocmd.trigger_slot[ibi_addr] = slot;
	aspeed_i3c_autocmd_write_selector(hci, ibi_addr, slot, enable);
	k_spin_unlock(&hci->lock, key);
}
#endif /* CONFIG_I3C_MIPI_HCI_ASPEED_AUTOCMD */

/*
 * The G7 secondary cores (SSP/TSP/BootMCU) see DRAM through a remap
 * window: CPU addresses must be translated to the bus-side physical
 * address before being handed to the I3C DMA master. Without the
 * translation the controller fetches/stores from the wrong DRAM
 * location and silently leaves the response ring untouched even though
 * RING_OPERATION2.CR_DEQ_PTR advances.
 */
static uint64_t mipi_i3c_hci_aspeed_dma_to_phys(struct i3c_hci *hci, uintptr_t addr)
{
	ARG_UNUSED(hci);

#if defined(CONFIG_SOC_SERIES_AST27XX)
	return ast27xx_soc_virt_addr_to_phy_addr(addr);
#else
	return (uint64_t)addr;
#endif
}

static bool mipi_i3c_hci_aspeed_dma_addr_visible(struct i3c_hci *hci, uint64_t phys,
						 size_t len)
{
	ARG_UNUSED(hci);

#if defined(CONFIG_SOC_SERIES_AST27XX)
	/*
	 * AST2700 SSP/TSP SRAM and TCM translate to low AHB-matrix
	 * addresses, while the I3C DMA master can reach DRAM through the
	 * 0x4_00000000 bus window used by the SoC remap logic.
	 */
	return phys >= ASPEED_I3C_DMA_DRAM_PHYS_BASE &&
	       phys < ASPEED_I3C_DMA_DRAM_PHYS_LIMIT &&
	       len <= (ASPEED_I3C_DMA_DRAM_PHYS_LIMIT - phys);
#else
	ARG_UNUSED(phys);
	ARG_UNUSED(len);
	return true;
#endif
}

const struct mipi_i3c_hci_vendor_ops mipi_i3c_hci_aspeed_ops = {
	.init = mipi_i3c_hci_aspeed_init,
	.autocmd_init = mipi_i3c_hci_aspeed_autocmd_init,
	.payload_too_big = mipi_i3c_hci_aspeed_payload_too_big,
	.get_status = mipi_i3c_hci_aspeed_get_status,
	.ccc_handler = mipi_i3c_hci_aspeed_ccc_handler,
	.set_ibi_terminate_len = mipi_i3c_hci_aspeed_set_ibi_terminate_len,
	.set_slv_pid = mipi_i3c_hci_aspeed_set_slv_pid,
	.set_slv_char_ctrl = mipi_i3c_hci_aspeed_set_slv_char_ctrl,
	.dma_drain = mipi_i3c_hci_aspeed_dma_drain,
	.pio_ibi_thld_status_only = mipi_i3c_hci_aspeed_pio_ibi_thld_status_only,
	.prep_daa_step = mipi_i3c_hci_aspeed_prep_daa_step,
	.dat_pick_slot = mipi_i3c_hci_aspeed_dat_pick_slot,
	.dat_wants_addr_indexed = mipi_i3c_hci_aspeed_dat_wants_addr_indexed,
	.ring_status = mipi_i3c_hci_aspeed_ring_status,
	.read_irq_summary = mipi_i3c_hci_aspeed_read_inhouse_summary,
	.read_priv_irq_status = mipi_i3c_hci_aspeed_read_inhouse_status,
	.clear_priv_irq_status = mipi_i3c_hci_aspeed_clear_inhouse_status,
	.renew_irq = mipi_i3c_hci_aspeed_renew_inhouse_irq,
	.disable_priv_irq = mipi_i3c_hci_aspeed_disable_inhouse_irq,
	.handle_priv_irq = mipi_i3c_hci_handle_aspeed_inhouse_irq,
	.dma_start = mipi_i3c_hci_aspeed_dma_start,
	.dma_init = mipi_i3c_hci_aspeed_dma_init,
	.dma_log_status = mipi_i3c_hci_aspeed_dma_log_status,
	.dma_recovery_done = mipi_i3c_hci_aspeed_dma_recovery_done,
	.ibi_chunk_size = mipi_i3c_hci_aspeed_ibi_chunk_size,
	.dma_to_phys = mipi_i3c_hci_aspeed_dma_to_phys,
	.dma_addr_visible = mipi_i3c_hci_aspeed_dma_addr_visible,
	.pio_log_prog_error = mipi_i3c_hci_aspeed_pio_log_prog_error,
	.status_changed_role = mipi_i3c_hci_aspeed_status_changed_role,
	.target_pid = mipi_i3c_hci_aspeed_target_pid,
	.target_dynamic_addr = mipi_i3c_hci_aspeed_target_dynamic_addr,
	.target_event_enabled = mipi_i3c_hci_aspeed_target_event_enabled,
	.target_request_event = mipi_i3c_hci_aspeed_target_request_event,
	.target_request_pending = mipi_i3c_hci_aspeed_target_request_pending,
	.target_enable_events = mipi_i3c_hci_aspeed_target_enable_events,
	.target_set_mode = mipi_i3c_hci_aspeed_target_set_mode,
	.target_get_role = mipi_i3c_hci_aspeed_target_get_role,
};

#endif /* CONFIG_I3C_MIPI_HCI_ASPEED_VENDOR */
