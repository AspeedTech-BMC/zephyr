/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * JTAG opertion priority:
 * HW mode 2 > SW mode > HW mode 1
 */

#define DT_DRV_COMPAT aspeed_jtag

#include <stdlib.h>
#include <errno.h>
#include <zephyr/drivers/jtag.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <soc.h>

#define LOG_LEVEL CONFIG_JTAG_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jtag_aspeed);
#include "jtag_aspeed.h"

#include <zephyr/portability/cmsis_os2.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>

#define DEFAULT_JTAG_FREQ 1000000

struct name_mapping {
	enum tap_state symbol;
	const char *name;
};

/*
 * NOTE:  do not change these state names.  They're documented,
 * and we rely on them to match SVF input (except for "RUN/IDLE").
 */
static const struct name_mapping tap_name_mapping[] = {
	{
		TAP_RESET,
		"RESET",
	},
	{
		TAP_IDLE,
		"RUN/IDLE",
	},
	{
		TAP_DRSELECT,
		"DRSELECT",
	},
	{
		TAP_DRCAPTURE,
		"DRCAPTURE",
	},
	{
		TAP_DRSHIFT,
		"DRSHIFT",
	},
	{
		TAP_DREXIT1,
		"DREXIT1",
	},
	{
		TAP_DRPAUSE,
		"DRPAUSE",
	},
	{
		TAP_DREXIT2,
		"DREXIT2",
	},
	{
		TAP_DRUPDATE,
		"DRUPDATE",
	},
	{
		TAP_IRSELECT,
		"IRSELECT",
	},
	{
		TAP_IRCAPTURE,
		"IRCAPTURE",
	},
	{
		TAP_IRSHIFT,
		"IRSHIFT",
	},
	{
		TAP_IREXIT1,
		"IREXIT1",
	},
	{
		TAP_IRPAUSE,
		"IRPAUSE",
	},
	{
		TAP_IREXIT2,
		"IREXIT2",
	},
	{
		TAP_IRUPDATE,
		"IRUPDATE",
	},
	/* only for input:  accept standard SVF name */
	{
		TAP_IDLE,
		"IDLE",
	},
};

/**
 * struct tms_cycle - This structure represents a tms cycle state.
 *
 * @tmsbits: is the bitwise representation of the needed tms transitions to
 *           move from one state to another.
 * @count:   number of jumps needed to move to the needed state.
 *
 */
struct tms_cycle {
	unsigned char tmsbits;
	unsigned char count;
};

/*
 * This is the complete set TMS cycles for going from any TAP state to any
 * other TAP state, following a "shortest path" rule.
 */
static const struct tms_cycle _tms_cycle_lookup[][16] = {
/* Row  0  Ex2DR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* Ex2DR*/{{0x00, 0}, {0x02, 2}, {0x00, 1}, {0x02, 3}, {0x07, 3}, {0x01, 1},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x03, 3}, {0x03, 2}, {0x57, 7}, {0x17, 5}, {0x07, 5}, {0x17, 6},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x01, 2}, {0x37, 6}, {0x07, 4}, {0x0f, 4} },

/* Row  1  Ex1DR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* Ex1DR*/{{0x02, 2}, {0x00, 0}, {0x02, 3}, {0x00, 1}, {0x07, 3}, {0x01, 1},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x03, 3}, {0x03, 2}, {0x57, 7}, {0x17, 5}, {0x07, 5}, {0x17, 6},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x01, 2}, {0x37, 6}, {0x07, 4}, {0x0f, 4} },

/* Row  2    SDR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/*   SDR*/{{0x05, 3}, {0x01, 1}, {0x00, 0}, {0x01, 2}, {0x0f, 4}, {0x03, 2},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x07, 4}, {0x07, 3}, {0xaf, 8}, {0x2f, 6}, {0x0f, 6}, {0x2f, 7},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x03, 3}, {0x6f, 7}, {0x0f, 5}, {0x1f, 5} },

/* Row  3    PDR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/*   PDR*/{{0x01, 1}, {0x05, 3}, {0x01, 2}, {0x00, 0}, {0x0f, 4}, {0x03, 2},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x07, 4}, {0x07, 3}, {0xaf, 8}, {0x2f, 6}, {0x0f, 6}, {0x2f, 7},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x03, 3}, {0x6f, 7}, {0x0f, 5}, {0x1f, 5} },

/* Row  4  SelIR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* SelIR*/{{0x55, 7}, {0x15, 5}, {0x05, 5}, {0x15, 6}, {0x00, 0}, {0x35, 6},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x05, 4}, {0x05, 3}, {0x0a, 4}, {0x02, 2}, {0x00, 2}, {0x02, 3},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x01, 2}, {0x06, 3}, {0x00, 1}, {0x01, 1} },

/* Row  5  UpdDR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* UpdDR*/{{0x15, 5}, {0x05, 3}, {0x01, 3}, {0x05, 4}, {0x03, 2}, {0x00, 0},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x01, 2}, {0x01, 1}, {0x2b, 6}, {0x0b, 4}, {0x03, 4}, {0x0b, 5},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x00, 1}, {0x1b, 5}, {0x03, 3}, {0x07, 3} },

/* Row  6  CapDR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* CapDR*/{{0x05, 3}, {0x01, 1}, {0x00, 1}, {0x01, 2}, {0x0f, 4}, {0x03, 2},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x00, 0}, {0x07, 3}, {0xaf, 8}, {0x2f, 6}, {0x0f, 6}, {0x2f, 7},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x03, 3}, {0x6f, 7}, {0x0f, 5}, {0x1f, 5} },

/* Row  7  SelDR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* SelDR*/{{0x0a, 4}, {0x02, 2}, {0x00, 2}, {0x02, 3}, {0x01, 1}, {0x06, 3},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x00, 1}, {0x00, 0}, {0x15, 5}, {0x05, 3}, {0x01, 3}, {0x05, 4},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x03, 3}, {0x0d, 4}, {0x01, 2}, {0x03, 2} },

/* Row  8  Ex2IR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* Ex2IR*/{{0x2b, 6}, {0x0b, 4}, {0x03, 4}, {0x0b, 5}, {0x07, 3}, {0x1b, 5},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x03, 3}, {0x03, 2}, {0x00, 0}, {0x02, 2}, {0x00, 1}, {0x02, 3},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x01, 2}, {0x01, 1}, {0x07, 4}, {0x0f, 4} },

/* Row  9  Ex1IR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* Ex1IR*/{{0x2b, 6}, {0x0b, 4}, {0x03, 4}, {0x0b, 5}, {0x07, 3}, {0x1b, 5},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x03, 3}, {0x03, 2}, {0x02, 2}, {0x00, 0}, {0x02, 3}, {0x00, 1},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x01, 2}, {0x01, 1}, {0x07, 4}, {0x0f, 4} },

/* Row 10    SIR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/*   SIR*/{{0x57, 7}, {0x17, 5}, {0x07, 5}, {0x17, 6}, {0x0f, 4}, {0x37, 6},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x07, 4}, {0x07, 3}, {0x05, 3}, {0x01, 1}, {0x00, 0}, {0x01, 2},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x03, 3}, {0x03, 2}, {0x0f, 5}, {0x1f, 5} },

/* Row 11    PIR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/*   PIR*/{{0x57, 7}, {0x17, 5}, {0x07, 5}, {0x17, 6}, {0x0f, 4}, {0x37, 6},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x07, 4}, {0x07, 3}, {0x01, 1}, {0x05, 3}, {0x01, 2}, {0x00, 0},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x03, 3}, {0x03, 2}, {0x0f, 5}, {0x1f, 5} },

/* Row 12    RTI -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/*   RTI*/{{0x15, 5}, {0x05, 3}, {0x01, 3}, {0x05, 4}, {0x03, 2}, {0x0d, 4},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x01, 2}, {0x01, 1}, {0x2b, 6}, {0x0b, 4}, {0x03, 4}, {0x0b, 5},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x00, 0}, {0x1b, 5}, {0x03, 3}, {0x07, 3} },

/* Row 13  UpdIR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* UpdIR*/{{0x15, 5}, {0x05, 3}, {0x01, 3}, {0x05, 4}, {0x03, 2}, {0x0d, 4},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x01, 2}, {0x01, 1}, {0x2b, 6}, {0x0b, 4}, {0x03, 4}, {0x0b, 5},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x00, 1}, {0x00, 0}, {0x03, 3}, {0x07, 3} },

/* Row 14  CapIR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/* CapIR*/{{0x57, 7}, {0x17, 5}, {0x07, 5}, {0x17, 6}, {0x0f, 4}, {0x37, 6},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x07, 4}, {0x07, 3}, {0x05, 3}, {0x01, 1}, {0x00, 1}, {0x01, 2},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x03, 3}, {0x03, 2}, {0x00, 0}, {0x1f, 5} },

/* Row 15    TLR -> */
/*		Ex2DR   Ex1DR     SDR     PDR   SelIR   UpdDR */
/*   TLR*/{{0x2a, 6}, {0x0a, 4}, {0x02, 4}, {0x0a, 5}, {0x06, 3}, {0x1a, 5},
/*		CapDR   SelDR   Ex2IR   Ex1IR     SIR     PIR */
		{0x02, 3}, {0x02, 2}, {0x56, 7}, {0x16, 5}, {0x06, 5}, {0x16, 6},
/*		  RTI   UpdIR   CapIR     TLR */
		{0x00, 1}, {0x36, 6}, {0x06, 4}, {0x00, 0} },
};

struct jtag_aspeed_data {
	uint32_t fifo_length;
	enum tap_state state;
	osEventFlagsId_t evt_id;
	bool sw_tdi;
};

struct jtag_aspeed_cfg {
	struct jtag_register_s *base;
	const struct device *clock_dev;
	const clock_control_subsys_t clk_id;
	const struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

#define DEV_CFG(dev) ((const struct jtag_aspeed_cfg *const)(dev)->config)
#define DEV_DATA(dev) ((struct jtag_aspeed_data *)(dev)->data)

static const char *tap_state_name(enum tap_state state)
{
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(tap_name_mapping); i++) {
		if (tap_name_mapping[i].symbol == state) {
			return tap_name_mapping[i].name;
		}
	}
	return "???";
}

static int jtag_aspeed_tap_state_check(const struct device *dev)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;

	LOG_DBG("%s: state = %s, status = 0x%08x\n", __func__, tap_state_name(priv->state),
		jtag_register->software_mode_and_status.value);

	if ((jtag_register->software_mode_and_status.fields.instr_xfer_pause &&
	     priv->state != TAP_IRPAUSE) ||
	    (jtag_register->software_mode_and_status.fields.data_xfer_pause &&
	     priv->state != TAP_DRPAUSE) ||
	    (jtag_register->software_mode_and_status.fields.engine_idle &&
	     priv->state != TAP_IDLE)) {
		return -EIO;
	}
	return 0;
}

#ifdef CONFIG_JTAG_ASPEED_HW_MODE2
static void jtag_aspeed_wait_shift_complete(struct jtag_aspeed_data *priv)
{
	osEventFlagsWait(priv->evt_id, JTAG_ASPEED_HW2_IRQ_STAT, osFlagsWaitAll,
			 osWaitForever);
}
#else
static void jtag_aspeed_wait_ir_pause_complete(struct jtag_aspeed_data *priv)
{
	osEventFlagsWait(priv->evt_id, JTAG_ASPEED_INST_PAUSE, osFlagsWaitAll,
			 osWaitForever);
}

static void jtag_aspeed_wait_ir_complete(struct jtag_aspeed_data *priv)
{
	osEventFlagsWait(priv->evt_id, JTAG_ASPEED_INST_COMPLETE,
			 osFlagsWaitAll, osWaitForever);
}

static void jtag_aspeed_wait_dr_pause_complete(struct jtag_aspeed_data *priv)
{
	osEventFlagsWait(priv->evt_id, JTAG_ASPEED_DATA_PAUSE, osFlagsWaitAll,
			 osWaitForever);
}

static inline void jtag_aspeed_wait_dr_complete(struct jtag_aspeed_data *priv)
{
	osEventFlagsWait(priv->evt_id, JTAG_ASPEED_DATA_COMPLETE,
			 osFlagsWaitAll, osWaitForever);
}
#endif

static void jtag_aspeed_isr(const struct device *dev)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
#ifdef CONFIG_JTAG_ASPEED_HW_MODE2
	uint32_t int_pending = jtag_register->mode_2_int_ctrl.value &
			       JTAG_ASPEED_HW2_IRQ_STAT;

	osEventFlagsSet(priv->evt_id, int_pending);
	/* W1C interrupt pending */
	jtag_register->mode_2_int_ctrl.value =
		jtag_register->mode_2_int_ctrl.value;
#else
	uint32_t int_pending = jtag_register->mode_1_int_ctrl.value &
			       JTAG_ASPEED_INT_PEND_MASK;

	osEventFlagsSet(priv->evt_id, int_pending);
	/* W1C interrupt pending */
	jtag_register->mode_1_int_ctrl.value =
		jtag_register->mode_1_int_ctrl.value;
#endif
}

static int jtag_aspeed_sw_xfer(const struct device *dev, enum jtag_pin pin,
			       uint8_t value)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	union software_mode_and_status_s software_mode_and_status;

	if (pin == JTAG_TRST) {
		LOG_DBG("JTAG_TRST = %d\t", value);
		jtag_register->engine_control_1.fields.control_of_trstn = value;
		return 0;
	}
	software_mode_and_status.value =
		jtag_register->software_mode_and_status.value;
	software_mode_and_status.fields.software_mode_enable = 1;
	software_mode_and_status.fields.software_tdi_and_tdo = priv->sw_tdi;
	switch (pin) {
	case JTAG_TDI:
		LOG_DBG("JTAG_TDI = %d\t", value);
		priv->sw_tdi = value;
		software_mode_and_status.fields.software_tdi_and_tdo = value;
		break;
	case JTAG_TCK:
		LOG_DBG("JTAG_TCK = %d\t", value);
		software_mode_and_status.fields.software_tck = value;
		break;
	case JTAG_TMS:
		LOG_DBG("JTAG_TMS = %d\t", value);
		software_mode_and_status.fields.software_tms = value;
		break;
	case JTAG_ENABLE:
	default:
		return -EINVAL;
	}
	jtag_register->software_mode_and_status.value =
		software_mode_and_status.value;
	LOG_DBG("Register value 0x%08x\n", software_mode_and_status.value);
	return 0;
}

static int jtag_aspeed_tdo_get(const struct device *dev, uint8_t *value)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	union software_mode_and_status_s software_mode_and_status;

	software_mode_and_status.value =
		jtag_register->software_mode_and_status.value;
	software_mode_and_status.fields.software_mode_enable = 1;
	software_mode_and_status.fields.software_tdi_and_tdo = priv->sw_tdi;
	jtag_register->software_mode_and_status.value =
		software_mode_and_status.value;
	*value = jtag_register->software_mode_and_status.fields.software_tdi_and_tdo;
	LOG_DBG("JTAG_TDO = %d\t", *value);
	LOG_DBG("Register value 0x%08x\n", software_mode_and_status.value);
	return 0;
}

static int jtag_aspeed_tck_cycle(const struct device *dev, uint8_t tms,
				 uint8_t tdi, uint8_t *tdo)
{
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	union software_mode_and_status_s software_mode_and_status;

	software_mode_and_status.value =
		jtag_register->software_mode_and_status.value;
	software_mode_and_status.fields.software_mode_enable = 1;
	/* TCK = 0 */
	software_mode_and_status.fields.software_tck = 0;
	software_mode_and_status.fields.software_tdi_and_tdo = tdi;
	software_mode_and_status.fields.software_tms = tms;
	jtag_register->software_mode_and_status.value =
		software_mode_and_status.value;
	/* TCK = 1 */
	software_mode_and_status.fields.software_tck = 1;
	jtag_register->software_mode_and_status.value =
		software_mode_and_status.value;
	*tdo = jtag_register->software_mode_and_status.fields
	       .software_tdi_and_tdo;
	return 0;
}

int jtag_aspeed_freq_get(const struct device *dev, uint32_t *freq)
{
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	uint32_t src_clk, div;

	clock_control_get_rate(config->clock_dev, config->clk_id, &src_clk);
#ifdef CONFIG_JTAG_ASPEED_HW_MODE2
	div = jtag_register->mode_2_control.fields.clock_divisor;
#else
	div = jtag_register->tck_control.fields.tck_divisor;
#endif
	*freq = src_clk / (div + 1);
	return 0;
}

int jtag_aspeed_freq_set(const struct device *dev, uint32_t freq)
{
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	uint32_t src_clk, div, diff;

	if (freq > JTAG_ASPEED_MAX_FREQUENCY) {
		return -EINVAL;
	}
	clock_control_get_rate(config->clock_dev, config->clk_id, &src_clk);
	div = DIV_ROUND_UP(src_clk, freq);
	diff = abs(src_clk - div * freq);
	if (diff > abs(src_clk - (div - 1) * freq)) {
		div = div - 1;
	}
	/* TCK freq = HCLK / (tck_divisor + 1) */
	if (div >= 1) {
		div = div - 1;
	}
	LOG_DBG("tck divisor = %d, tck freq = %d\n", div, src_clk / (div + 1));
#ifdef CONFIG_JTAG_ASPEED_HW_MODE2
	union mode_2_control_s mode_2_control;

	mode_2_control.value = jtag_register->mode_2_control.value;
	mode_2_control.fields.clock_divisor = div;
	jtag_register->mode_2_control.value = mode_2_control.value;
#else
	union tck_control_s tck_control;

	tck_control.value = jtag_register->tck_control.value;
	tck_control.fields.tck_divisor = div;
	jtag_register->tck_control.value = tck_control.value;
#endif
	return 0;
}

static int jtag_aspeed_set_tap_state_sw(const struct device *dev, enum tap_state from_state,
					enum tap_state end_state)
{
	int i = 0;
	enum tap_state from, to;
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	uint8_t dummy;

	from = from_state;
	to = end_state;

	for (i = 0; i < _tms_cycle_lookup[from][to].count; i++)
		jtag_aspeed_tck_cycle(dev,
				      ((_tms_cycle_lookup[from][to].tmsbits
				      >> i) & 0x1), 0, &dummy);
	priv->state = end_state;
	return 0;
}

#ifdef CONFIG_JTAG_ASPEED_HW_MODE2
int jtag_aspeed_tap_set(const struct device *dev, enum tap_state state)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);

	jtag_aspeed_set_tap_state_sw(dev, priv->state, state);
	LOG_DBG("Move tap state to %s\n", tap_state_name(state));
	return 0;
}

#else
static int jtag_aspeed_tap_idle(const struct device *dev)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	union mode_1_control_s mode_1_control;
	int ret;

	ret = jtag_aspeed_tap_state_check(dev);
	if (ret) {
		return ret;
	}
	if (priv->state == TAP_IDLE) {
		return 0;
	}
	mode_1_control.value = jtag_register->mode_1_control.value;
	mode_1_control.fields.xfer_len = 0;
	mode_1_control.fields.terminating_xfer = 1;
	mode_1_control.fields.last_xfer = 1;
	LOG_DBG("mode_1_ctrl = 0x%08x, status = 0x%08x\n",
		jtag_register->mode_1_control.value,
		jtag_register->software_mode_and_status.value);
	if (priv->state == TAP_IRPAUSE) {
		mode_1_control.fields.ir_xfer_en = 1;
		jtag_register->mode_1_control.value = mode_1_control.value;
		jtag_aspeed_wait_ir_complete(priv);
		mode_1_control.fields.ir_xfer_en = 0;
		jtag_register->mode_1_control.value = mode_1_control.value;
	} else if (priv->state == TAP_DRPAUSE) {
		mode_1_control.fields.dr_xfer_en = 1;
		jtag_register->mode_1_control.value = mode_1_control.value;
		jtag_aspeed_wait_dr_complete(priv);
		mode_1_control.fields.dr_xfer_en = 0;
		jtag_register->mode_1_control.value = mode_1_control.value;
	}
	mode_1_control.fields.terminating_xfer = 0;
	jtag_register->mode_1_control.value = mode_1_control.value;
	priv->state = TAP_IDLE;
	return 0;
}

static int jtag_aspeed_tap_reset(const struct device *dev)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	union mode_1_control_s mode_1_control;

	/* Disable SW mode */
	jtag_register->software_mode_and_status.value = 0;
	mode_1_control.value = jtag_register->mode_1_control.value;
	/* Enable HW mode 1 */
	mode_1_control.fields.engine_enable = 1;
	/* Reset target tap */
	mode_1_control.fields.reset_to_tlr = 1;
	jtag_register->mode_1_control.value = mode_1_control.value;
	while (jtag_register->mode_1_control.fields.reset_to_tlr)
		;
	priv->state = TAP_IDLE;
	return 0;
}

int jtag_aspeed_tap_set(const struct device *dev, enum tap_state state)
{
	int ret;
	struct jtag_aspeed_data *priv = DEV_DATA(dev);

	if (state == TAP_IDLE) {
		ret = jtag_aspeed_tap_idle(dev);
	} else if (state == TAP_RESET) {
		ret = jtag_aspeed_tap_reset(dev);
	} else {
		ret = jtag_aspeed_set_tap_state_sw(dev, priv->state, state);
	}
	if (ret) {
		LOG_ERR("Move tap state to %s fail\n", tap_state_name(state));
	} else {
		LOG_DBG("Move tap state to %s\n", tap_state_name(state));
	}
	return ret;
}
#endif

int jtag_aspeed_tap_get(const struct device *dev, enum tap_state *state)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	int ret;

	ret = jtag_aspeed_tap_state_check(dev);
	if (ret) {
		return ret;
	}
	*state = priv->state;
	return 0;
}

int jtag_aspeed_tck_run(const struct device *dev, uint32_t run_count)
{
#ifdef CONFIG_JTAG_ASPEED_HW_MODE2
	uint32_t execute_tck;
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	union mode_2_control_s mode_2_control;
	union shift_control_s shift_control;

	while (run_count) {
		execute_tck = run_count > GENMASK(9, 0) ? GENMASK(9, 0) : run_count;
		/* Disable sw mode */
		jtag_register->software_mode_and_status.value = 0;
		jtag_register->padding_control_0.value = 0;
		mode_2_control.value = jtag_register->mode_2_control.value;
		mode_2_control.fields.static_shift_value = 0;
		mode_2_control.fields.upper_data_shift_number = execute_tck >> 7;
		jtag_register->mode_2_control.value = mode_2_control.value;

		shift_control.value = 0;
		shift_control.fields.enable_static_shift = 1;
		shift_control.fields.lower_data_shift_number = execute_tck & GENMASK(6, 0);
		jtag_register->shift_control.value = shift_control.value;

		jtag_aspeed_wait_shift_complete(priv);
		run_count -= execute_tck;
	}
#else
	uint32_t i;
	uint8_t dummy;

	for (i = 0; i < run_count; i++)
		jtag_aspeed_tck_cycle(dev, 0, 0, &dummy);
#endif
	return 0;
}

#ifdef CONFIG_JTAG_ASPEED_HW_MODE2

static int jtag_aspeed_shctrl_tms_mask(enum tap_state from,
				       enum tap_state to,
				       enum tap_state there,
				       enum tap_state endstate,
				       bool start_shift, bool end_shift,
				       uint32_t *tms_mask)
{
	uint32_t pre_tms = start_shift ? _tms_cycle_lookup[from][to].count : 0;
	uint32_t post_tms = end_shift ? _tms_cycle_lookup[there][endstate].count : 0;
	uint32_t tms_value = start_shift ? _tms_cycle_lookup[from][to].tmsbits : 0;
	union shift_control_s *shift_control = (union shift_control_s *)tms_mask;

	tms_value |= end_shift ? _tms_cycle_lookup[there][endstate].tmsbits
					 << pre_tms :
				 0;
	if (pre_tms > GENMASK(2, 0) || post_tms > GENMASK(2, 0)) {
		LOG_ERR("pre/port tms count is greater than hw limit");
		return -EINVAL;
	}
	LOG_DBG("from: %s, to: %s, there: %s, endstate: %s, "
		"start_shift: %d, end_shift: %d, pre_tms: %d, post_tms: %d, "
		"tms_value: 0x%08x\n",
		tap_state_name(from), tap_state_name(to),
		tap_state_name(there), tap_state_name(endstate),
		start_shift, end_shift, pre_tms, post_tms, tms_value);

	shift_control->fields.start_of_shift = start_shift;
	shift_control->fields.end_of_shift = end_shift;
	shift_control->fields.pre_tms_shift_number = pre_tms;
	shift_control->fields.post_tms_shift_number = post_tms;
	shift_control->fields.tms_value = tms_value;
	return 0;
}

static int jtag_aspeed_xfer(const struct device *dev, struct scan_command_s *scan)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	uint32_t remain_xfer = scan->fields.num_bits;
	uint32_t xfer_size;
	uint32_t shift_index = 0, curr_out_index = 0, curr_in_index = 0;
	uint8_t end_xfer;
	const uint32_t *out_value = (const uint32_t *)scan->fields.out_value;
	uint32_t *in_value = (uint32_t *)scan->fields.in_value;
	union mode_2_control_s mode_2_control;
	union shift_control_s shift_control;
	enum tap_state shift;
	enum tap_state exit;
	enum tap_state pause;
	enum tap_state endstate;
	bool start_shift;
	bool end_shift;
	uint32_t shift_control_val;
	int ret;

	/* Disable SW mode */
	jtag_register->software_mode_and_status.value = 0;
	/* Clear internal FIFO */
	mode_2_control.value = jtag_register->mode_2_control.value;
	mode_2_control.fields.reset_internal_fifo = 1;
	jtag_register->mode_2_control.value = mode_2_control.value;
	while (jtag_register->mode_2_control.fields.reset_internal_fifo)
		;
	/* Enable HW mode 2 */
	mode_2_control.value = jtag_register->mode_2_control.value;
	mode_2_control.fields.engine_enable = 1;
	mode_2_control.fields.engine_output_enable = 1;
	jtag_register->mode_2_control.value = mode_2_control.value;

	if (scan->ir_scan) {
		shift = TAP_IRSHIFT;
		exit = TAP_IREXIT1;
		pause = TAP_IRPAUSE;
	} else {
		shift = TAP_DRSHIFT;
		exit = TAP_DREXIT1;
		pause = TAP_DRPAUSE;
	}

	LOG_DBG("scan info: %sScan size:%d status from %s to end_state:%s\n",
		scan->ir_scan ? "IR" : "DR", scan->fields.num_bits, tap_state_name(priv->state),
		tap_state_name(scan->end_state));

	start_shift = (priv->state == shift) ? false : true;

	if (scan->end_state == shift) {
		if (scan->fields.num_bits == 1) {
			end_shift = true;
			endstate = pause;
		} else {
			end_shift = false;
			endstate = shift;
		}
	} else {
		endstate = scan->end_state;
		end_shift = true;
	}

	while (remain_xfer) {
		if (remain_xfer > priv->fifo_length) {
			end_xfer = 0;
			xfer_size = priv->fifo_length;
			shift_index += priv->fifo_length >> 5;
		} else {
			end_xfer = 1;
			xfer_size = remain_xfer;
			shift_index += remain_xfer >> 5;
			if (remain_xfer & 0x1f) {
				shift_index++;
			}
		}

		/* Clear internal FIFO */
		mode_2_control.value = jtag_register->mode_2_control.value;
		mode_2_control.fields.reset_internal_fifo = 1;
		jtag_register->mode_2_control.value = mode_2_control.value;
		while (jtag_register->mode_2_control.fields.reset_internal_fifo)
			;

		/* Wait internal FIFO clear to CPU mode */
		while (jtag_register->mode_2_control.fields.internal_fifo_mode)
			;

		/* Write out data to FIFO */
		for (; curr_out_index < shift_index; curr_out_index++) {
			if (xfer_size < 32) {
				jtag_register->data_for_hw_mode_2[0].value =
					out_value[curr_out_index] &
					(uint32_t)GENMASK(xfer_size - 1, 0);
			} else {
				jtag_register->data_for_hw_mode_2[0].value =
					out_value[curr_out_index];
			}
		}

		/*
		 * Set shift length: upper 3 bits in mode_2_control,
		 * lower 7 bits in shift_control
		 */
		mode_2_control.value = jtag_register->mode_2_control.value;
		mode_2_control.fields.upper_data_shift_number = xfer_size >> 7;
		jtag_register->mode_2_control.value = mode_2_control.value;
		shift_control_val = 0;
		if (remain_xfer > priv->fifo_length)
			ret = jtag_aspeed_shctrl_tms_mask(priv->state, shift, exit, endstate,
							  start_shift, 0, &shift_control_val);
		else
			ret = jtag_aspeed_shctrl_tms_mask(priv->state, shift, exit, endstate,
							  start_shift, end_shift,
							  &shift_control_val);
		if (ret)
			return ret;
		shift_control.value = shift_control_val;
		shift_control.fields.lower_data_shift_number = xfer_size & GENMASK(6, 0);
		shift_control.fields.start_of_shift = 1;
		LOG_DBG("Transfer ctrl: 0x%08x, shift ctrl: 0x%08x\n", mode_2_control.value,
			shift_control.value);
		jtag_register->shift_control.value = shift_control.value;

		/* Wait for transfer complete */
		jtag_aspeed_wait_shift_complete(priv);
		if (remain_xfer > priv->fifo_length)
			priv->state = shift;
		else
			priv->state = endstate;
		remain_xfer -= xfer_size;

		/* Read in data from FIFO */
		if (in_value) {
			for (; curr_in_index < shift_index; curr_in_index++) {
				uint32_t temp_in_value = jtag_register->data_for_hw_mode_2[0].value;

				LOG_DBG("in_value[%d] = %08x\n", curr_in_index, temp_in_value);
				if (xfer_size < 32) {
					in_value[curr_in_index] =
						temp_in_value & (uint32_t)GENMASK(xfer_size - 1, 0);
				} else {
					in_value[curr_in_index] = temp_in_value;
				}
				xfer_size -= 32;
			}
		} else {
			for (; curr_in_index < shift_index; curr_in_index++) {
				(void)jtag_register->data_for_hw_mode_2[0].value;
			}
		}
	}
	LOG_DBG("Transfer complete, end state: %s\n", tap_state_name(priv->state));
	return 0;
}
#else
int jtag_aspeed_xfer(const struct device *dev, struct scan_command_s *scan)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	uint32_t remain_xfer = scan->fields.num_bits;
	uint32_t xfer_size;
	uint32_t shift_index = 0, curr_out_index = 0, curr_in_index = 0;
	uint8_t end_xfer;
	const uint32_t *out_value = (const uint32_t *)scan->fields.out_value;
	uint32_t *in_value = (uint32_t *)scan->fields.in_value;
	union mode_1_control_s mode_1_control;

	/* Disable SW mode */
	jtag_register->software_mode_and_status.value = 0;
	/* Clear internal fifo */
	mode_1_control.value = jtag_register->mode_1_control.value;
	mode_1_control.fields.reset_internal_fifo = 1;
	jtag_register->mode_1_control.value = mode_1_control.value;
	while (jtag_register->mode_1_control.fields.reset_internal_fifo)
		;
	/* Enable HW mode 1 */
	mode_1_control.value = jtag_register->mode_1_control.value;
	mode_1_control.fields.engine_enable = 1;
	mode_1_control.fields.msb_first = 0;
	mode_1_control.fields.last_xfer = 0;
	LOG_DBG("scan info: %sScan size:%d end_state:%s\n",
		scan->ir_scan ? "IR" : "DR", scan->fields.num_bits,
		tap_state_name(scan->end_state));
	while (remain_xfer) {
		if (remain_xfer > priv->fifo_length) {
			end_xfer = 0;
			xfer_size = priv->fifo_length;
			shift_index += priv->fifo_length >> 5;
		} else {
			end_xfer = 1;
			xfer_size = remain_xfer;
			shift_index += remain_xfer >> 5;
			if (remain_xfer & 0x1f) {
				shift_index++;
			}
		}
		/* Write out data to FIFO */
		for (; curr_out_index < shift_index; curr_out_index++) {
			if (xfer_size < 32) {
				LOG_DBG("out_value[%d] = %08x\n",
					curr_out_index,
					out_value[curr_out_index] &
					(uint32_t)GENMASK(xfer_size - 1, 0));
				jtag_register->data_for_hw_mode_1[0].value =
					out_value[curr_out_index] &
					(uint32_t)GENMASK(xfer_size - 1, 0);
			} else {
				LOG_DBG("out_value[%d] = %08x\n",
					curr_out_index,
					out_value[curr_out_index]);
				jtag_register->data_for_hw_mode_1[0].value =
					out_value[curr_out_index];
			}
		}
		if (end_xfer && scan->end_state == TAP_IDLE) {
			mode_1_control.fields.last_xfer = 1;
		}
		mode_1_control.fields.xfer_len = xfer_size;
		LOG_DBG("Transfer ctrl: 0x%08x\n", mode_1_control.value);
		/* Enable transfer */
		if (scan->ir_scan) {
			mode_1_control.fields.ir_xfer_en = 1;
			jtag_register->mode_1_control.value =
				mode_1_control.value;
			if (mode_1_control.fields.last_xfer) {
				jtag_aspeed_wait_ir_complete(priv);
			} else {
				jtag_aspeed_wait_ir_pause_complete(priv);
			}
			mode_1_control.fields.ir_xfer_en = 0;
			jtag_register->mode_1_control.value =
				mode_1_control.value;
		} else {
			mode_1_control.fields.dr_xfer_en = 1;
			jtag_register->mode_1_control.value =
				mode_1_control.value;
			if (jtag_register->mode_1_control.fields.last_xfer) {
				jtag_aspeed_wait_dr_complete(priv);
			} else {
				jtag_aspeed_wait_dr_pause_complete(priv);
			}
			mode_1_control.fields.dr_xfer_en = 0;
			jtag_register->mode_1_control.value =
				mode_1_control.value;
		}
		remain_xfer -= xfer_size;
		/* Get in data to fifo */
		if (in_value) {
			for (; curr_in_index < shift_index; curr_in_index++) {
				if (xfer_size < 32) {
					in_value[curr_in_index] =
						jtag_register
						->data_for_hw_mode_1[0]
						.value >>
						(32 - xfer_size);
				} else {
					in_value[curr_in_index] =
						jtag_register
						->data_for_hw_mode_1[0]
						.value;
				}
				LOG_DBG("in_value[%d] = %08x\n", curr_in_index,
					in_value[curr_in_index]);
				xfer_size -= 32;
			}
		} else {
			for (; curr_in_index < shift_index; curr_in_index++) {
				if (xfer_size < 32) {
					LOG_DBG("in_value[%d] = %08x\n",
						curr_in_index,
						jtag_register->data_for_hw_mode_1
						[0]
						.value >>
						(32 - xfer_size));
				} else {
					LOG_DBG("in_value[%d] = %08x\n",
						curr_in_index,
						jtag_register
						->data_for_hw_mode_1[0]
						.value);
				}
			}
		}
	}
	priv->state = scan->end_state;
	return 0;
}
#endif

static int jtag_aspeed_init(const struct device *dev)
{
	struct jtag_aspeed_data *priv = DEV_DATA(dev);
	const struct jtag_aspeed_cfg *config = DEV_CFG(dev);
	struct jtag_register_s *jtag_register = config->base;
	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}
	reset_line_assert_dt(&config->reset);
	reset_line_deassert_dt(&config->reset);
	priv->sw_tdi = 0;
	priv->state = TAP_IDLE;
	priv->evt_id = osEventFlagsNew(NULL);
	config->irq_config_func(dev);
#ifdef CONFIG_JTAG_ASPEED_HW_MODE2
	union mode_2_control_s mode_2_control;
	union mode_2_int_ctrl_s mode_2_int_ctrl;

	/* Enable all of the interrupt */
	mode_2_int_ctrl.value = jtag_register->mode_2_int_ctrl.value;
	mode_2_int_ctrl.fields.shift_complete_interrupt_status = 1;
	mode_2_int_ctrl.fields.shift_complete_interrupt_enable = 1;
	jtag_register->mode_2_int_ctrl.value = mode_2_int_ctrl.value;
	jtag_aspeed_freq_set(dev, DEFAULT_JTAG_FREQ);
	/* Output enable */
	mode_2_control.value = jtag_register->mode_2_control.value;
	mode_2_control.fields.engine_enable = 1;
	mode_2_control.fields.engine_output_enable = 1;
	mode_2_control.fields.trst_value = 1;
	jtag_register->mode_2_control.value = mode_2_control.value;
#else
	union mode_1_int_ctrl_s mode_1_int_ctrl;
	union mode_1_control_s mode_1_control;

	/* Enable all of the interrupt */
	mode_1_int_ctrl.value = jtag_register->mode_1_int_ctrl.value;
	mode_1_int_ctrl.fields.enable_of_data_xfer_completed = 1;
	mode_1_int_ctrl.fields.enable_of_data_xfer_pause = 1;
	mode_1_int_ctrl.fields.enable_of_instr_xfer_completed = 1;
	mode_1_int_ctrl.fields.enable_of_instr_xfer_pause = 1;
	jtag_register->mode_1_int_ctrl.value = mode_1_int_ctrl.value;
	jtag_aspeed_freq_set(dev, DEFAULT_JTAG_FREQ);
	/* Output enable */
	mode_1_control.value = jtag_register->mode_1_control.value;
	mode_1_control.fields.engine_output_enable = 1;
	jtag_register->mode_1_control.value = mode_1_control.value;
#endif
	return 0;
}

static struct jtag_driver_api jtag_aspeed_api = {
	.freq_get = jtag_aspeed_freq_get,
	.freq_set = jtag_aspeed_freq_set,
	.tap_get = jtag_aspeed_tap_get,
	.tap_set = jtag_aspeed_tap_set,
	.tck_run = jtag_aspeed_tck_run,
	.xfer = jtag_aspeed_xfer,
	.sw_xfer = jtag_aspeed_sw_xfer,
	.tdo_get = jtag_aspeed_tdo_get,
};

#define ASPEED_JTAG_INIT(n)                                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct jtag_aspeed_data jtag_aspeed_data_##n = {                                    \
		.fifo_length = 512,                                                                \
	};                                                                                         \
	static void jtag_aspeed_config_func_##n(const struct device *dev);                         \
	static const struct jtag_aspeed_cfg jtag_aspeed_cfg_##n = {                                \
		.base = (struct jtag_register_s *)DT_INST_REG_ADDR(n),                             \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, clk_id),                  \
		.reset = RESET_DT_SPEC_INST_GET(n),                                                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_config_func = jtag_aspeed_config_func_##n,                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, jtag_aspeed_init, NULL, &jtag_aspeed_data_##n,                    \
			      &jtag_aspeed_cfg_##n, POST_KERNEL,                                   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &jtag_aspeed_api);               \
	static void jtag_aspeed_config_func_##n(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), jtag_aspeed_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
                                                                                                   \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(ASPEED_JTAG_INIT)
