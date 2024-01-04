/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_i3c

#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

#include <zephyr/logging/log.h>
#define LOG_MODULE_NAME			i3c_aspeed
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_I3C_ASPEED_LOG_LEVEL);

static int aspeed_i3c_init(const struct device *dev);

/* Aspeed-specific global register set */
#define ASPEED_I3CG_REG0(x)		(((x) * 0x10) + 0x10)
#define   I3CG_REG0_SDA_PULLUP		GENMASK(29, 28)
#define     I3CG_REG0_SDA_PULLUP_2K	0x0
#define     I3CG_REG0_SDA_PULLUP_750	0x2
#define     I3CG_REG0_SDA_PULLUP_545	0x3

#define ASPEED_I3CG_REG1(x)		(((x) * 0x10) + 0x14)
#define   I3CG_REG1_SDA_OUT_SW_MODE_EN	BIT(31)
#define   I3CG_REG1_SCL_OUT_SW_MODE_EN	BIT(30)
#define   I3CG_REG1_SDA_IN_SW_MODE_EN	BIT(29)
#define   I3CG_REG1_SCL_IN_SW_MODE_EN	BIT(28)
#define   I3CG_REG1_SDA_IN_SW_MODE_VAL	BIT(27)
#define   I3CG_REG1_SDA_OUT_SW_MODE_VAL	BIT(25)
#define   I3CG_REG1_SDA_SW_MODE_OE	BIT(24)
#define   I3CG_REG1_SCL_IN_SW_MODE_VAL	BIT(23)
#define   I3CG_REG1_SCL_OUT_SW_MODE_VAL	BIT(21)
#define   I3CG_REG1_SCL_SW_MODE_OE	BIT(20)
#define   I3CG_REG1_INST_ID		GENMASK(19, 16)
#define   I3CG_REG1_SLV_STATIC_ADDR_EN	BIT(15)
#define   I3CG_REG1_SLV_STATIC_ADDR	GENMASK(14, 8)
#define   I3CG_REG1_PENDING_INT		GENMASK(7, 4)
#define   I3CG_REG1_ACT_MODE		GENMASK(3, 2)

#define DEVICE_CTRL			0x0
#define   DEV_CTRL_ENABLE		BIT(31)
#define   DEV_CTRL_RESUME		BIT(30)
#define   DEV_CTRL_ABORT		BIT(29)
#define   DEV_CTRL_AUTO_HJ_DISABLE	BIT(27)
#define   DEV_CTRL_SLAVE_MDB		GENMASK(23, 16)
#define   DEV_CTRL_SLAVE_PEC_EN		BIT(10)
#define   DEV_CTRL_IBI_PAYLOAD_EN	BIT(9)
#define   DEV_CTRL_HOT_JOIN_NACK	BIT(8)
#define   DEV_CTRL_I2C_SLAVE_PRESENT	BIT(7)
#define   DEV_CTRL_IBA_INCLUDE		BIT(0)

#define DEVICE_ADDR			0x4
#define   DEV_ADDR_DYNAMIC_ADDR_VALID	BIT(31)
#define   DEV_ADDR_DYNAMIC_ADDR		GENMASK(22, 16)
#define   DEV_ADDR_STATIC_ADDR_VALID	BIT(15)
#define   DEV_ADDR_STATIC_ADDR		GENMASK(6, 0)

#define HW_CAPABILITY			0x8
#define COMMAND_QUEUE_PORT		0xc
#define   COMMAND_PORT_PEC		BIT(31)
#define   COMMAND_PORT_TOC		BIT(30)
#define   COMMAND_PORT_READ_TRANSFER	BIT(28)
#define   COMMAND_PORT_SDAP		BIT(27)
#define   COMMAND_PORT_ROC		BIT(26)
#define   COMMAND_PORT_DBP		BIT(25)
#define   COMMAND_PORT_SPEED		GENMASK(23, 21)
#define     SPEED_I3C_SDR0		0x0
#define     SPEED_I3C_SDR1		0x1
#define     SPEED_I3C_SDR2		0x2
#define     SPEED_I3C_SDR3		0x3
#define     SPEED_I3C_SDR4		0x4
#define     SPEED_I3C_HDR_TS		0x5
#define     SPEED_I3C_HDR_DDR		0x6
#define     SPEED_I3C_I2C_FM		0x7
#define     SPEED_I2C_FM		0x0
#define     SPEED_I2C_FMP		0x1
#define   COMMAND_PORT_DEV_INDEX	GENMASK(20, 16)
#define   COMMAND_PORT_CP		BIT(15)
#define   COMMAND_PORT_CMD		GENMASK(14, 7)
#define   COMMAND_PORT_TID		GENMASK(6, 3)
#define     TID_TARGET_IBI		0x1
#define     TID_TARGET_RD_DATA		0x2
#define     TID_TARGET_MASTER_WR_DATA	0x8
#define     TID_TARGET_MASTER_DEFSLVS	0xf
#define   COMMAND_PORT_ATTR		GENMASK(2, 0)
#define     COMMAND_PORT_XFER_CMD	0x0
#define     COMMAND_PORT_XFER_ARG	0x1
#define     COMMAND_PORT_SHORT_ARG	0x2
#define     COMMAND_PORT_ADDR_ASSGN_CMD	0x3
#define     COMMAND_PORT_SLAVE_DATA_CMD	0x0

/* for COMMAND_PORT_XFER_ARG */
#define COMMAND_PORT_ARG_DATA_LEN	GENMASK(31, 16)
#define   COMMAND_PORT_ARG_DATA_LEN_MAX	65536
#define COMMAND_PORT_ARG_DB		GENMASK(15, 8)

/* for COMMAND_PORT_ADDR_ASSGN_CMD */
#define COMMAND_PORT_DEV_COUNT		GENMASK(25, 21)

/* for COMMAND_PORT_SLAVE_DATA_CMD */
#define COMMAND_PORT_SLAVE_DATA_LEN	GENMASK(31, 16)
#define COMMAND_PORT_SLAVE_DATA_TID	GENMASK(5, 3)

#define RESPONSE_QUEUE_PORT		0x10
#define   RESPONSE_PORT_ERR_STATUS	GENMASK(31, 28)
#define     RESPONSE_NO_ERROR		0
#define     RESPONSE_ERROR_CRC		1
#define     RESPONSE_ERROR_PARITY	2
#define     RESPONSE_ERROR_FRAME	3
#define     RESPONSE_ERROR_IBA_NACK	4
#define     RESPONSE_ERROR_ADDRESS_NACK	5
#define     RESPONSE_ERROR_OVER_UNDER_FLOW 6
#define     RESPONSE_ERROR_TRANSF_ABORT	8
#define     RESPONSE_ERROR_I2C_W_NACK_ERR 9
#define     RESPONSE_ERROR_EARLY_TERMINATE 10
#define     RESPONSE_ERROR_PEC_ERR	12
#define   RESPONSE_PORT_TID		GENMASK(27, 24)
#define     TID_SLAVE_IBI_DONE		0b0001
#define     TID_MASTER_READ_DATA	0b0010
#define     TID_MASTER_WRITE_DATA	0b1000
#define     TID_CCC_WRITE_DATA		0b1111
#define   RESPONSE_PORT_DATA_LEN	GENMASK(15, 0)

#define RX_TX_DATA_PORT			0x14
#define IBI_QUEUE_STATUS		0x18
#define IBI_QUEUE_STATUS_IBI_ID		GENMASK(15, 8)
#define IBI_QUEUE_STATUS_DATA_LEN	GENMASK(7, 0)
#define   IBI_QUEUE_IBI_ADDR(x)		(FIELD_GET(IBI_QUEUE_STATUS_IBI_ID, x) >> 1)
#define   IBI_QUEUE_IBI_RNW(x)		(FIELD_GET(IBI_QUEUE_STATUS_IBI_ID, x) & BIT(0))
#define   IBI_TYPE_MR(x)		((IBI_QUEUE_IBI_ADDR(x) != 0x2) && !IBI_QUEUE_IBI_RNW(x))
#define   IBI_TYPE_HJ(x)		((IBI_QUEUE_IBI_ADDR(x) == 0x2) && !IBI_QUEUE_IBI_RNW(x))
#define   IBI_TYPE_SIRQ(x)		((IBI_QUEUE_IBI_ADDR(x) != 0x2) && IBI_QUEUE_IBI_RNW(x))

#define QUEUE_THLD_CTRL			0x1c
#define   QUEUE_THLD_CTRL_IBI_STA	GENMASK(31, 24)
#define   QUEUE_THLD_CTRL_IBI_DAT	GENMASK(23, 16)
#define     MAX_IBI_FRAG_SIZE		124
#define   QUEUE_THLD_CTRL_RESP_BUF	GENMASK(15, 8)

#define DATA_BUFFER_THLD_CTRL		0x20
#define   DATA_BUFFER_THLD_CTRL_RX_BUF	GENMASK(11, 8)

#define IBI_QUEUE_CTRL			0x24
#define IBI_MR_REQ_REJECT		0x2c
#define IBI_SIR_REQ_REJECT		0x30
#define IBI_REQ_REJECT_ALL		GENMASK(31, 0)

#define RESET_CTRL			0x34
#define RESET_CTRL_BUS			BIT(31)
#define RESET_CTRL_BUS_RESET_TYPE	GENMASK(30, 29)
#define   BUS_RESET_TYPE_EXIT		0b00
#define   BUS_RESET_TYPE_SCL_LOW	0b11
#define RESET_CTRL_IBI_QUEUE		BIT(5)
#define RESET_CTRL_RX_FIFO		BIT(4)
#define RESET_CTRL_TX_FIFO		BIT(3)
#define RESET_CTRL_RESP_QUEUE		BIT(2)
#define RESET_CTRL_CMD_QUEUE		BIT(1)
#define RESET_CTRL_SOFT			BIT(0)
#define RESET_CTRL_ALL                  (RESET_CTRL_IBI_QUEUE	              |\
					 RESET_CTRL_RX_FIFO	              |\
					 RESET_CTRL_TX_FIFO	              |\
					 RESET_CTRL_RESP_QUEUE	              |\
					 RESET_CTRL_CMD_QUEUE	              |\
					 RESET_CTRL_SOFT)
#define RESET_CTRL_QUEUES		(RESET_CTRL_IBI_QUEUE	              |\
					 RESET_CTRL_RX_FIFO	              |\
					 RESET_CTRL_TX_FIFO	              |\
					 RESET_CTRL_RESP_QUEUE	              |\
					 RESET_CTRL_CMD_QUEUE)
#define RESET_CTRL_XFER_QUEUES		(RESET_CTRL_RX_FIFO	              |\
					 RESET_CTRL_TX_FIFO	              |\
					 RESET_CTRL_RESP_QUEUE	              |\
					 RESET_CTRL_CMD_QUEUE)
#define SLV_EVENT_CTRL			0x38
#define   SLV_EVENT_CTRL_MWL_UPD	BIT(7)
#define   SLV_EVENT_CTRL_MRL_UPD	BIT(6)
#define   SLV_EVENT_CTRL_SIR_EN		BIT(0)
#define INTR_STATUS			0x3c
#define INTR_STATUS_EN			0x40
#define INTR_SIGNAL_EN			0x44
#define INTR_FORCE			0x48
#define INTR_BUSOWNER_UPDATE_STAT	BIT(13)
#define INTR_IBI_UPDATED_STAT		BIT(12)
#define INTR_READ_REQ_RECV_STAT		BIT(11)
#define INTR_DEFSLV_STAT		BIT(10)
#define INTR_TRANSFER_ERR_STAT		BIT(9)
#define INTR_DYN_ADDR_ASSGN_STAT	BIT(8)
#define INTR_CCC_UPDATED_STAT		BIT(6)
#define INTR_TRANSFER_ABORT_STAT	BIT(5)
#define INTR_RESP_READY_STAT		BIT(4)
#define INTR_CMD_QUEUE_READY_STAT	BIT(3)
#define INTR_IBI_THLD_STAT		BIT(2)
#define INTR_RX_THLD_STAT		BIT(1)
#define INTR_TX_THLD_STAT		BIT(0)

#define QUEUE_STATUS_LEVEL		0x4c
#define   QUEUE_STATUS_IBI_STATUS_CNT	GENMASK(28, 24)
#define   QUEUE_STATUS_IBI_BUF_BLR	GENMASK(23, 16)
#define   QUEUE_STATUS_LEVEL_RESP	GENMASK(15, 8)
#define   QUEUE_STATUS_LEVEL_CMD	GENMASK(7, 0)

#define DATA_BUFFER_STATUS_LEVEL	0x50
#define   DATA_BUFFER_STATUS_LEVEL_TX	GENMASK(7, 0)

#define PRESENT_STATE			0x54
#define   CM_TFR_ST_STS			GENMASK(21, 16)
#define     CM_TFR_ST_STS_HALT		0x13
#define   CM_TFR_STS			GENMASK(13, 8)
#define     CM_TFR_STS_MASTER_SERV_IBI	0xe
#define     CM_TFR_STS_MASTER_HALT	0xf
#define     CM_TFR_STS_SLAVE_HALT	0x6

#define CCC_DEVICE_STATUS		0x58
#define DEVICE_ADDR_TABLE_POINTER	0x5c
#define   DEVICE_ADDR_TABLE_DEPTH	GENMASK(31, 16)
#define   DEVICE_ADDR_TABLE_ADDR	GENMASK(15, 0)

#define DEV_CHAR_TABLE_POINTER		0x60
#define VENDOR_SPECIFIC_REG_POINTER	0x6c
#define SLV_PID_HI			0x70
#define   SLV_PID_MFG_ID		GENMASK(15, 1)
#define     MIPI_MFG_ASPEED		0x3f6
#define   SLV_PID_DCR_SELECT		BIT(0)
#define     DCR_SELECT_FIXED		0
#define     DCR_SELECT_RANDOM		1
#define SLV_PID_LO			0x74
#define   SLV_PID_PART_ID		GENMASK(31, 16)
#define   SLV_PID_INST_ID		GENMASK(15, 12)
#define   SLV_PID_EXTRA_INFO		GENMASK(11, 0)
#define SLV_CHAR_CTRL			0x78
#define   SLV_DCR			GENMASK(15, 8)
#define   SLV_BCR			GENMASK(7, 0)
#define     SLV_BCR_DEVICE_ROLE		GENMASK(7, 6)c

#define SLV_MAX_LEN			0x7c
#define   SLV_MAX_RD_LEN		GENMASK(31, 16)
#define   SLV_MAX_WR_LEN		GENMASK(15, 0)

#define MAX_READ_TURNAROUND		0x80
#define MAX_DATA_SPEED			0x84
#define SLV_DEBUG_STATUS		0x88
#define SLV_INTR_REQ			0x8c
#define   SLV_INTR_REQ_IBI_STS		GENMASK(9, 8)
#define     IBI_STS_ACCEPTED		0x01
#define     IBI_STS_NOT_ATTEMPTED	0x11

#define SCL_I3C_OD_TIMING		0xb4
#define   SCL_I3C_OD_TIMING_HCNT	GENMASK(23, 16)
#define   SCL_I3C_OD_TIMING_LCNT	GENMASK(7, 0)
#define SCL_I3C_PP_TIMING		0xb8
#define   SCL_I3C_PP_TIMING_HCNT	GENMASK(23, 16)
#define   SCL_I3C_PP_TIMING_LCNT	GENMASK(7, 0)
#define     SCL_I3C_TIMING_CNT_MIN	5

#define DEVICE_CTRL_EXTENDED		0xb0
#define   DEVICE_CTRL_ROLE_MASK		GENMASK(1, 0)
#define     DEVICE_CTRL_ROLE_MASTER	0
#define     DEVICE_CTRL_ROLE_SLAVE	1

#define SCL_I2C_FM_TIMING		0xbc
#define   SCL_I2C_FM_TIMING_HCNT	GENMASK(31, 16)
#define   SCL_I2C_FM_TIMING_LCNT	GENMASK(15, 0)
#define SCL_I2C_FMP_TIMING		0xc0
#define   SCL_I2C_FMP_TIMING_HCNT	GENMASK(31, 16)
#define   SCL_I2C_FMP_TIMING_LCNT	GENMASK(15, 0)

#define SCL_EXT_TERMN_LCNT_TIMING	0xcc
#define SDA_HOLD_SWITCH_DLY_TIMING	0xd0
#define SDA_TX_HOLD			GENMASK(18, 16)
#define   SDA_TX_HOLD_MIN		0b001
#define   SDA_TX_HOLD_MAX		0b111
#define SDA_PP_OD_SWITCH_DLY		GENMASK(10, 8)
#define SDA_OD_PP_SWITCH_DLY		GENMASK(2, 0)
#define BUS_FREE_TIMING			0xd4
#define   BUS_I3C_AVAILABLE_TIME	GENMASK(31, 16)
#define   BUS_I3C_MST_FREE		GENMASK(15, 0)

#define DEV_ADDR_TABLE_LEGACY_I2C_DEV	BIT(31)
#define DEV_ADDR_TABLE_DYNAMIC_ADDR	GENMASK(23, 16)
#define DEV_ADDR_TABLE_MR_REJECT	BIT(14)
#define DEV_ADDR_TABLE_SIR_REJECT	BIT(13)
#define DEV_ADDR_TABLE_IBI_MDB		BIT(12)
#define DEV_ADDR_TABLE_IBI_PEC		BIT(11)
#define DEV_ADDR_TABLE_STATIC_ADDR	GENMASK(6, 0)

#define I3C_BUS_I2C_STD_TLOW_MIN_NS	4700
#define I3C_BUS_I2C_STD_THIGH_MIN_NS	4000
#define I3C_BUS_I2C_STD_TR_MAX_NS	1000
#define I3C_BUS_I2C_STD_TF_MAX_NS	300
#define I3C_BUS_I2C_FM_TLOW_MIN_NS	1300
#define I3C_BUS_I2C_FM_THIGH_MIN_NS	600
#define I3C_BUS_I2C_FM_TR_MAX_NS	300
#define I3C_BUS_I2C_FM_TF_MAX_NS	300
#define I3C_BUS_I2C_FMP_TLOW_MIN_NS	500
#define I3C_BUS_I2C_FMP_THIGH_MIN_NS	260
#define I3C_BUS_I2C_FMP_TR_MAX_NS	120
#define I3C_BUS_I2C_FMP_TF_MAX_NS	120
#define I3C_BUS_THIGH_MAX_NS		41

#define read32_poll_timeout(addr, val, cond, delay_us, timeout_us)                                 \
	({                                                                                         \
		uint32_t __timeout_tick = Z_TIMEOUT_US(timeout_us).ticks;                          \
		uint32_t __start = sys_clock_tick_get_32();                                        \
		int __ret = 0;                                                                     \
		for (;;) {                                                                         \
			val = sys_read32(addr);                                                    \
			if (cond) {                                                                \
				break;                                                             \
			}                                                                          \
			if ((sys_clock_tick_get_32() - __start) > __timeout_tick) {                \
				__ret = -ETIMEDOUT;                                                \
				break;                                                             \
			}                                                                          \
			if (delay_us) {                                                            \
				k_busy_wait(delay_us);                                             \
			}                                                                          \
		}                                                                                  \
		__ret;                                                                             \
	})

struct aspeed_i3c_cmd {
	uint32_t cmd_lo;
	uint32_t cmd_hi;
	void *tx_buf;
	void *rx_buf;
	int tx_length;
	int rx_length;
	int ret;
};

struct aspeed_i3c_xfer {
	int ret;
	int ncmds;
	struct aspeed_i3c_cmd *cmds;
	struct k_sem sem;
};

struct aspeed_i3c_priv {
	uint8_t pos;
	uint8_t addr;
	struct {
		bool enable;
	} ibi;
};

struct aspeed_i3c_data {
	struct i3c_driver_data common;
	const struct aspeed_i3c_config *config;
	struct i3c_target_config *target_config;
	const struct device *dev;
	struct k_spinlock lock;
	struct aspeed_i3c_xfer *curr_xfer;
	uint32_t core_period;
	uint32_t i3c_pp_scl_hi_period_ns;
	uint32_t i3c_pp_scl_lo_period_ns;
	uint32_t i3c_od_scl_hi_period_ns;
	uint32_t i3c_od_scl_lo_period_ns;
	uint32_t sda_tx_hold_ns;
	uint16_t maxdevs;
	uint16_t datstartaddr;
	uint32_t free_pos;
	uint32_t need_da;
	uint8_t addrs[8];
	struct aspeed_i3c_priv privs[8];
	bool ibi_pec_force_enable;

	/* target mode data */
	bool sir_allowed_by_sw;
	struct k_work target_work;
	struct k_sem target_ibi_sem;
	struct k_sem target_data_sem;
};

struct aspeed_i3c_config {
	struct i3c_driver_config common;

	uintptr_t base;
	uintptr_t global_regs;
	uint32_t global_idx;
	struct reset_dt_spec global_reset;
	struct reset_dt_spec core_reset;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_id;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

static int aspeed_i3c_pullup_to_reg(int ohms, uint32_t *reg)
{
	uint32_t val;

	switch (ohms) {
	case 2000:
		val = I3CG_REG0_SDA_PULLUP_2K;
		break;
	case 750:
		val = I3CG_REG0_SDA_PULLUP_750;
		break;
	case 545:
		val = I3CG_REG0_SDA_PULLUP_545;
		break;
	default:
		return -EINVAL;
	}

	if (reg) {
		*reg = FIELD_PREP(I3CG_REG0_SDA_PULLUP, val);
	}

	return 0;
}

static void aspeed_i3c_exit_sw_mode(struct aspeed_i3c_data *data)
{
	uintptr_t base = data->config->global_regs;
	uint32_t index = data->config->global_idx;
	uint32_t reg;

	reg = sys_read32(base + ASPEED_I3CG_REG1(index));
	reg &= ~(I3CG_REG1_SCL_IN_SW_MODE_EN | I3CG_REG1_SDA_IN_SW_MODE_EN);
	sys_write32(reg, base + ASPEED_I3CG_REG1(index));
}

static void aspeed_i3c_enter_sw_mode(struct aspeed_i3c_data *data)
{
	uintptr_t base = data->config->global_regs;
	uint32_t index = data->config->global_idx;
	uint32_t reg;

	reg = sys_read32(base + ASPEED_I3CG_REG1(index));
	reg |= I3CG_REG1_SCL_IN_SW_MODE_VAL | I3CG_REG1_SDA_IN_SW_MODE_VAL;
	sys_write32(reg, base + ASPEED_I3CG_REG1(index));
	reg |= I3CG_REG1_SCL_IN_SW_MODE_EN | I3CG_REG1_SDA_IN_SW_MODE_EN;
	sys_write32(reg, base + ASPEED_I3CG_REG1(index));
}

static void aspeed_i3c_toggle_scl_in(struct aspeed_i3c_data *data, int count)
{
	uintptr_t base = data->config->global_regs;
	uint32_t index = data->config->global_idx;
	uint32_t reg;

	reg = sys_read32(base + ASPEED_I3CG_REG1(index));

	for (; count; count--) {
		reg &= ~I3CG_REG1_SCL_IN_SW_MODE_VAL;
		sys_write32(reg, base + ASPEED_I3CG_REG1(index));
		reg |= I3CG_REG1_SCL_IN_SW_MODE_VAL;
		sys_write32(reg, base + ASPEED_I3CG_REG1(index));
	}
}

static void aspeed_i3c_gen_internal_stop(struct aspeed_i3c_data *data)
{
	uintptr_t base = data->config->global_regs;
	uint32_t index = data->config->global_idx;
	uint32_t reg;

	reg = sys_read32(base + ASPEED_I3CG_REG1(index));
	reg &= ~I3CG_REG1_SCL_IN_SW_MODE_VAL;
	sys_write32(reg, base + ASPEED_I3CG_REG1(index));
	reg &= ~I3CG_REG1_SDA_IN_SW_MODE_VAL;
	sys_write32(reg, base + ASPEED_I3CG_REG1(index));
	reg |= I3CG_REG1_SCL_IN_SW_MODE_VAL;
	sys_write32(reg, base + ASPEED_I3CG_REG1(index));
	reg |= I3CG_REG1_SDA_IN_SW_MODE_VAL;
	sys_write32(reg, base + ASPEED_I3CG_REG1(index));
}

static int even_parity(uint8_t byte)
{
	int parity = 0;
	uint8_t b = byte;

	while (b) {
		parity = !parity;
		b = b & (b - 1);
	}

	return !parity;
}

static void aspeed_i3c_set_ibi_mdb(struct aspeed_i3c_data *data, uint8_t mdb)
{
	uintptr_t base = data->config->base;
	uint32_t reg;

	reg = sys_read32(base + DEVICE_CTRL);
	reg &= ~DEV_CTRL_SLAVE_MDB;
	reg |= FIELD_PREP(DEV_CTRL_SLAVE_MDB, mdb);
	sys_write32(reg, base + DEVICE_CTRL);
}

static void aspeed_i3c_disable(struct aspeed_i3c_data *data)
{
	const struct aspeed_i3c_config *config = data->config;
	uint32_t reg;

	reg = sys_read32(config->base + DEVICE_CTRL);
	if (!(reg & DEV_CTRL_ENABLE)) {
		return;
	}

	if (data->common.ctrl_config.is_secondary) {
		aspeed_i3c_enter_sw_mode(data);
	}

	reg &= ~DEV_CTRL_ENABLE;
	sys_write32(reg, config->base + DEVICE_CTRL);

	if (data->common.ctrl_config.is_secondary) {
		aspeed_i3c_toggle_scl_in(data, 8);
		aspeed_i3c_gen_internal_stop(data);
		aspeed_i3c_exit_sw_mode(data);
	}
}

static void aspeed_i3c_enable(struct aspeed_i3c_data *data)
{
	const struct aspeed_i3c_config *config = data->config;
	uint32_t reg;

	reg = sys_read32(config->base + DEVICE_CTRL);
	reg |= DEV_CTRL_ENABLE;
	if (data->common.ctrl_config.is_secondary) {
		/* enable hot-join */
		reg &= ~DEV_CTRL_AUTO_HJ_DISABLE;
		reg |= DEV_CTRL_IBI_PAYLOAD_EN;
		aspeed_i3c_enter_sw_mode(data);
	} else {
		reg |= DEV_CTRL_IBA_INCLUDE;
	}
	sys_write32(reg, config->base + DEVICE_CTRL);

	if (data->common.ctrl_config.is_secondary) {
		uint32_t wait_cnt, wait_us;

		reg = sys_read32(config->base + BUS_FREE_TIMING);
		wait_cnt = FIELD_GET(BUS_I3C_AVAILABLE_TIME, reg);
		wait_us = DIV_ROUND_UP(data->core_period * wait_cnt, NSEC_PER_USEC);
		k_busy_wait(wait_us);

		aspeed_i3c_toggle_scl_in(data, 8);

		reg = sys_read32(config->base + DEVICE_CTRL);
		if (!(reg & DEV_CTRL_ENABLE)) {
			goto out;
		}
		aspeed_i3c_gen_internal_stop(data);
out:
		aspeed_i3c_exit_sw_mode(data);
	}
}

static void aspeed_i3c_exit_halt(struct aspeed_i3c_data *data)
{
	uintptr_t base = data->config->base;
	uint32_t status;
	uint32_t halt_state = CM_TFR_STS_MASTER_HALT;
	int ret;

	if (data->common.ctrl_config.is_secondary) {
		halt_state = CM_TFR_STS_SLAVE_HALT;
	}

	status = sys_read32(base + PRESENT_STATE);
	if (FIELD_GET(CM_TFR_STS, status) != halt_state) {
		LOG_DBG("I3C not in halt state, no need for resume");
		return;
	}

	sys_write32(sys_read32(base + DEVICE_CTRL) | DEV_CTRL_RESUME, base + DEVICE_CTRL);

	ret = read32_poll_timeout(base + PRESENT_STATE, status,
				  FIELD_GET(CM_TFR_STS, status) != halt_state, 10, 1000000);
	if (ret) {
		LOG_ERR("Exit halt state failed: %d %08x %08x", ret,
			sys_read32(base + PRESENT_STATE), sys_read32(base + QUEUE_STATUS_LEVEL));
	}
}

static void aspeed_i3c_enter_halt(struct aspeed_i3c_data *data, bool by_sw)
{
	uintptr_t base = data->config->base;
	uint32_t status;
	uint32_t halt_state = CM_TFR_STS_MASTER_HALT;
	int ret;

	if (data->common.ctrl_config.is_secondary) {
		halt_state = CM_TFR_STS_SLAVE_HALT;
	}

	if (by_sw) {
		sys_write32(sys_read32(base + DEVICE_CTRL) | DEV_CTRL_ABORT, base + DEVICE_CTRL);
	}

	ret = read32_poll_timeout(base + PRESENT_STATE, status,
				  FIELD_GET(CM_TFR_STS, status) == halt_state, 10, 1000000);
	if (ret) {
		LOG_ERR("Enter halt state failed: %d %08x %08x", ret,
			sys_read32(base + PRESENT_STATE), sys_read32(base + QUEUE_STATUS_LEVEL));
	}
}

static int aspeed_i3c_reset_ctrl(struct aspeed_i3c_data *data, uint32_t reset_ctrl)
{
	uintptr_t base = data->config->base;
	uint32_t reg = reset_ctrl & RESET_CTRL_ALL;
	uint32_t status;
	int ret;

	if (!reg) {
		return -EINVAL;
	}

	sys_write32(reg, base + RESET_CTRL);
	ret = read32_poll_timeout(base + RESET_CTRL, status, status == 0, 10, 1000000);
	if (ret) {
		LOG_ERR("%s Reset %#x/%#x failed: %d", data->dev->name, status, reg, ret);
	}

	return ret;
}

static void aspeed_i3c_wr_tx_fifo(struct aspeed_i3c_data *data, uint8_t *bytes, int nbytes)
{
	uintptr_t base = data->config->base;
	uint32_t *src = (uint32_t *)bytes;
	int nwords = nbytes >> 2;

	for (int i = 0; i < nwords; i++) {
		LOG_DBG("tx data: %x", *src);
		sys_write32(*src++, base + RX_TX_DATA_PORT);
	}

	if (nbytes & 0x3) {
		uint32_t tmp = 0;

		memcpy(&tmp, bytes + (nbytes & ~0x3), nbytes & 3);
		LOG_DBG("tx data: %x", tmp);
		sys_write32(tmp, base + RX_TX_DATA_PORT);
	}
}

static void aspeed_i3c_rd_fifo(struct aspeed_i3c_data *data, uint32_t reg,
			       uint8_t *bytes, int nbytes)
{
	uintptr_t base = data->config->base;
	uint32_t *dst = (uint32_t *)bytes;
	int nwords = nbytes >> 2;
	int i;

	for (i = 0; i < nwords; i++) {
		*dst++ = sys_read32(base + reg);
	}

	if (nbytes & 0x3) {
		uint32_t tmp;

		tmp = sys_read32(base + reg);
		memcpy(bytes + (nbytes & ~0x3), &tmp, nbytes & 0x3);
	}
}

static void aspeed_i3c_rd_rx_fifo(struct aspeed_i3c_data *data, uint8_t *bytes, int nbytes)
{
	aspeed_i3c_rd_fifo(data, RX_TX_DATA_PORT, bytes, nbytes);
}

static void aspeed_i3c_rd_ibi_fifo(struct aspeed_i3c_data *data, uint8_t *bytes, int nbytes)
{
	aspeed_i3c_rd_fifo(data, IBI_QUEUE_STATUS, bytes, nbytes);
}

static int aspeed_i3c_ibi_enable(const struct device *dev,
			  struct i3c_device_desc *target)
{
	struct aspeed_i3c_data *data = dev->data;
	struct aspeed_i3c_priv *priv = target->controller_priv;
	int pos = priv->pos;
	uintptr_t base = data->config->base;
	uintptr_t dat_addr = base + data->datstartaddr + (pos << 2);
	uint32_t dat, sir_reject, reg;

	dat = sys_read32(dat_addr);
	dat &= ~DEV_ADDR_TABLE_SIR_REJECT;
	if (target->bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE) {
		dat |= DEV_ADDR_TABLE_IBI_MDB | DEV_ADDR_TABLE_IBI_PEC;
	}
	sys_write32(dat, dat_addr);

	sir_reject = sys_read32(base + IBI_SIR_REQ_REJECT);
	sir_reject &= ~BIT(pos);
	sys_write32(sir_reject, base + IBI_SIR_REQ_REJECT);

	reg = sys_read32(base + INTR_STATUS_EN);
	reg |= INTR_IBI_THLD_STAT;
	sys_write32(reg, base + INTR_STATUS_EN);

	reg = sys_read32(base + INTR_SIGNAL_EN);
	reg |= INTR_IBI_THLD_STAT;
	sys_write32(reg, base + INTR_SIGNAL_EN);

	struct i3c_ccc_events i3c_events;

	i3c_events.events = I3C_CCC_EVT_INTR;
	i3c_ccc_do_events_set(target, true, &i3c_events);

	return 0;
}

static int aspeed_i3c_ibi_disable(const struct device *dev,
			  struct i3c_device_desc *target)
{
	return 0;
}

static void aspeed_i3c_start_xfer(struct aspeed_i3c_data *data, struct aspeed_i3c_xfer *xfer)
{
	struct aspeed_i3c_cmd *cmd;
	uintptr_t base = data->config->base;
	uint32_t thld_ctrl;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	int i;

	data->curr_xfer = xfer;

	for (i = 0; i < xfer->ncmds; i++) {
		cmd = &xfer->cmds[i];
		if (cmd->tx_length) {
			aspeed_i3c_wr_tx_fifo(data, cmd->tx_buf, cmd->tx_length);
		}
	}

	thld_ctrl = sys_read32(base + QUEUE_THLD_CTRL);
	thld_ctrl &= ~QUEUE_THLD_CTRL_RESP_BUF;
	thld_ctrl |= FIELD_PREP(QUEUE_THLD_CTRL_RESP_BUF, xfer->ncmds - 1);
	sys_write32(thld_ctrl, base + QUEUE_THLD_CTRL);

	for (i = 0; i < xfer->ncmds; i++) {
		cmd = &xfer->cmds[i];

		sys_write32(cmd->cmd_hi, base + COMMAND_QUEUE_PORT);
		sys_write32(cmd->cmd_lo, base + COMMAND_QUEUE_PORT);
		LOG_DBG("cmd_hi %08x, cmd_lo %08x", cmd->cmd_hi, cmd->cmd_lo);
	}

	k_spin_unlock(&data->lock, key);
}

static void aspeed_i3c_end_xfer(struct aspeed_i3c_data *data)
{
	struct aspeed_i3c_xfer *xfer = data->curr_xfer;
	uintptr_t base = data->config->base;
	uint32_t nresp, i;
	int ret = 0;

	if (!xfer) {
		return;
	}

	nresp = FIELD_GET(QUEUE_STATUS_LEVEL_RESP, sys_read32(base + QUEUE_STATUS_LEVEL));
	for (i = 0; i < nresp; i++) {
		uint32_t resp = sys_read32(base + RESPONSE_QUEUE_PORT);
		struct aspeed_i3c_cmd *cmd;

		LOG_DBG("%s resp = %08x", data->dev->name, resp);

		cmd = &xfer->cmds[FIELD_GET(RESPONSE_PORT_TID, resp)];
		cmd->rx_length = FIELD_GET(RESPONSE_PORT_DATA_LEN, resp);
		cmd->ret = FIELD_GET(RESPONSE_PORT_ERR_STATUS, resp);
		if (cmd->rx_length && !cmd->ret) {
			aspeed_i3c_rd_rx_fifo(data, cmd->rx_buf, cmd->rx_length);
		}
	}

	for (i = 0; i < nresp; i++) {
		if (xfer->cmds[i].ret) {
			ret = xfer->cmds[i].ret;
		}
	}

	if (ret) {
		aspeed_i3c_enter_halt(data, false);
		aspeed_i3c_reset_ctrl(data, RESET_CTRL_XFER_QUEUES);
		aspeed_i3c_exit_halt(data);
	}
	xfer->ret = ret;
	k_sem_give(&xfer->sem);
}

static int aspeed_i3c_get_addr_pos(struct aspeed_i3c_data *data, uint8_t addr)
{
	int pos;

	for (pos = 0; pos < data->maxdevs; pos++) {
		if (addr == data->addrs[pos]) {
			return pos;
		}
	}

	return -1;
}

static struct i3c_device_desc *aspeed_i3c_device_find(const struct device *dev,
						      const struct i3c_device_id *id)
{
	const struct aspeed_i3c_config *config = dev->config;

	return i3c_dev_list_find(&config->common.dev_list, id);
}

static int aspeed_i3c_detach_i3c_device(const struct device *dev, struct i3c_device_desc *target)
{
	struct aspeed_i3c_data *data = dev->data;
	struct aspeed_i3c_priv *priv = target->controller_priv;
	int pos = priv->pos;
	uintptr_t dat_addr = data->config->base + data->datstartaddr;
	uint32_t dat;

	if (pos < 0) {
		return -ENODATA;
	}

	data->free_pos |= BIT(pos);
	data->addrs[pos] = 0;

	dat_addr += (pos << 2);
	dat = DEV_ADDR_TABLE_MR_REJECT | DEV_ADDR_TABLE_SIR_REJECT;
	sys_write32(dat, dat_addr);

	target->controller_priv = NULL;

	return 0;
}

static int aspeed_i3c_attach_i3c_device(const struct device *dev, struct i3c_device_desc *target,
					uint8_t addr)
{
	struct aspeed_i3c_data *data = dev->data;
	struct aspeed_i3c_priv *priv;
	uintptr_t dat_addr = data->config->base + data->datstartaddr;
	uint32_t dat;
	int pos = find_lsb_set(data->free_pos) - 1;

	if (pos < 0 || pos == data->maxdevs) {
		return -ENOSPC;
	}

	/*
	 * No need to check whether the target has been attached since this is
	 * done by the caller.
	 */
	data->free_pos &= ~BIT(pos);
	data->addrs[pos] = addr;

	priv = &data->privs[pos];
	priv->pos = pos;
	priv->addr = addr;
	target->controller_priv = priv;

	dat_addr += (pos << 2);
	addr |= even_parity(addr) << 7;
	dat = FIELD_PREP(DEV_ADDR_TABLE_DYNAMIC_ADDR, addr) | DEV_ADDR_TABLE_MR_REJECT |
	      DEV_ADDR_TABLE_SIR_REJECT;
	sys_write32(dat, dat_addr);

	if (target->dynamic_addr == 0) {
		data->need_da |= BIT(pos);
		LOG_DBG("need_da = %x", data->need_da);
	}

	return 0;
}

static int aspeed_i3c_do_ccc(const struct device *dev, struct i3c_ccc_payload *payload)
{
	struct aspeed_i3c_data *data = dev->data;
	struct aspeed_i3c_xfer xfer;
	struct aspeed_i3c_cmd cmd;
	struct i3c_ccc_target_payload *tgt_pl = &payload->targets.payloads[0];
	int pos = 0;
	uint8_t db = 0;
	bool dbp = false;
	int rnw = 0;

	/* only support broadcast CCC and 1-to-1 direct CCC for now */
	__ASSERT_NO_MSG(payload->targets.num_targets <= 1);

	memset(&cmd, 0, sizeof(cmd));
	memset(&xfer, 0, sizeof(xfer));

	xfer.ncmds = 1;
	xfer.cmds = &cmd;
	xfer.ret = -ETIMEDOUT;

	if (i3c_ccc_is_payload_broadcast(payload)) {
		if (payload->ccc.data_len && payload->ccc.data) {
			/* broadcast CCC with data must be a write command */
			cmd.tx_buf = payload->ccc.data;
			cmd.tx_length = payload->ccc.data_len;
		}
	} else {
		pos = aspeed_i3c_get_addr_pos(data, tgt_pl->addr);
		if (pos < 0) {
			return pos;
		}

		/* direct CCC with ccc.data must be the defining byte */
		if (payload->ccc.data_len) {
			__ASSERT_NO_MSG(payload->ccc.data_len == 1);
			dbp = true;
			db = payload->ccc.data[0];
		}

		if (tgt_pl->rnw) {
			cmd.rx_buf = tgt_pl->data;
			cmd.rx_length = tgt_pl->data_len;
			cmd.tx_length = 0;
			rnw = 1;
		} else {
			cmd.tx_buf = tgt_pl->data;
			cmd.tx_length = tgt_pl->data_len;
			cmd.rx_length = 0;
		}
	}

	cmd.cmd_hi = FIELD_PREP(COMMAND_PORT_ATTR, COMMAND_PORT_XFER_ARG) |
		     FIELD_PREP(COMMAND_PORT_ARG_DB, db);
	if (rnw) {
		cmd.cmd_hi |= FIELD_PREP(COMMAND_PORT_ARG_DATA_LEN, cmd.rx_length);
	} else {
		cmd.cmd_hi |= FIELD_PREP(COMMAND_PORT_ARG_DATA_LEN, cmd.tx_length);
	}

	cmd.cmd_lo = FIELD_PREP(COMMAND_PORT_ATTR, COMMAND_PORT_XFER_CMD) |
		     FIELD_PREP(COMMAND_PORT_DEV_INDEX, pos) |
		     FIELD_PREP(COMMAND_PORT_CMD, payload->ccc.id) |
		     FIELD_PREP(COMMAND_PORT_READ_TRANSFER, rnw) | COMMAND_PORT_CP |
		     COMMAND_PORT_ROC | COMMAND_PORT_TOC;
	if (dbp) {
		cmd.cmd_lo |= COMMAND_PORT_DBP;
	}

	if (payload->ccc.id == I3C_CCC_SETHID || payload->ccc.id == I3C_CCC_DEVCTRL) {
		cmd.cmd_lo |= FIELD_PREP(COMMAND_PORT_SPEED, SPEED_I3C_I2C_FM);
	}

	LOG_DBG("%s do_ccc: cmd_hi = %08x, cmd_hi = %08x", data->dev->name, cmd.cmd_hi,
		cmd.cmd_lo);

	k_sem_init(&xfer.sem, 0, 1);
	aspeed_i3c_start_xfer(data, &xfer);
	if (k_sem_take(&xfer.sem, K_MSEC(1000))) {
		LOG_ERR("%s: Failed to send CCC", dev->name);
		aspeed_i3c_enter_halt(data, true);
		aspeed_i3c_reset_ctrl(data, RESET_CTRL_XFER_QUEUES);
		aspeed_i3c_exit_halt(data);
	}

	if (xfer.ret == RESPONSE_ERROR_IBA_NACK) {
		/*
		 * Here, we should ideally return M2_ERROR. However, the I3C subsystem currently
		 * does not define this error code. For now, returning 0 maintains the I3C
		 * controller's enabled state, allowing us the opportunity to redo CCCs when the
		 * target devices activate later.
		 */
		LOG_INF("do_ccc: target devices not activated for now");
		return 0;
	}

	return xfer.ret;
}

static int aspeed_i3c_do_entdaa(struct aspeed_i3c_data *data, int index)
{
	struct aspeed_i3c_xfer xfer;
	struct aspeed_i3c_cmd cmd;
	int ret;

	cmd.tx_length = 0;
	cmd.rx_length = 0;
	cmd.cmd_hi = FIELD_PREP(COMMAND_PORT_ATTR, COMMAND_PORT_XFER_ARG);

	cmd.cmd_lo = FIELD_PREP(COMMAND_PORT_ATTR, COMMAND_PORT_ADDR_ASSGN_CMD) |
		     FIELD_PREP(COMMAND_PORT_CMD, I3C_CCC_ENTDAA) |
		     FIELD_PREP(COMMAND_PORT_DEV_COUNT, 1) |
		     FIELD_PREP(COMMAND_PORT_DEV_INDEX, index) | COMMAND_PORT_ROC |
		     COMMAND_PORT_TOC;

	k_sem_init(&xfer.sem, 0, 1);
	xfer.ncmds = 1;
	xfer.cmds = &cmd;
	xfer.ret = -ETIMEDOUT;
	aspeed_i3c_start_xfer(data, &xfer);

	ret = k_sem_take(&xfer.sem, K_MSEC(10));
	if (ret) {
		aspeed_i3c_enter_halt(data, true);
		aspeed_i3c_reset_ctrl(data, RESET_CTRL_XFER_QUEUES);
		aspeed_i3c_exit_halt(data);
		return -ETIMEDOUT;
	}

	if (cmd.rx_length) {
		LOG_DBG("ENTDAA: No more new device");
	}

	return xfer.ret;
}

static uint64_t bytes_to_pid(uint8_t *bytes)
{
	uint64_t pid = 0;
	int j, sft;

	for (j = 0; j < 6; j++) {
		sft = (6 - j - 1) * 8;
		pid |= (uint64_t)bytes[j] << sft;
	}

	return pid;
}

static int aspeed_i3c_do_daa(const struct device *dev)
{
	struct aspeed_i3c_data *data = dev->data;
	const struct aspeed_i3c_config *config = dev->config;
	struct i3c_ccc_getpid getpid;
	struct i3c_ccc_payload ccc_payload;
	struct i3c_ccc_target_payload ccc_tgt_payload;
	struct i3c_device_desc *target;
	struct aspeed_i3c_priv *priv;
	uint64_t pid;
	int ret, i, pos;
	uint8_t addr;

	memset(&ccc_payload, 0, sizeof(ccc_payload));
	memset(&ccc_tgt_payload, 0, sizeof(ccc_tgt_payload));
	memset(&getpid.pid, 0, sizeof(getpid.pid));

	for (pos = 0; pos < data->maxdevs; pos++) {
		if ((BIT(pos) & data->need_da) == 0) {
			continue;
		}

		addr = data->addrs[pos];
		ret = aspeed_i3c_do_entdaa(data, pos);
		if (ret) {
			/* TBD: free the address slot? */
			continue;
		}

		/* DA has been assigned. Retrieve the PID */
		ccc_tgt_payload.addr = addr;
		ccc_tgt_payload.rnw = 1;
		ccc_tgt_payload.data = &getpid.pid[0];
		ccc_tgt_payload.data_len = sizeof(getpid.pid);

		ccc_payload.ccc.id = I3C_CCC_GETPID;
		ccc_payload.targets.payloads = &ccc_tgt_payload;
		ccc_payload.targets.num_targets = 1;
		aspeed_i3c_do_ccc(dev, &ccc_payload);

		pid = bytes_to_pid(getpid.pid);
		const struct i3c_device_id i3c_id = I3C_DEVICE_ID(pid);

		target = aspeed_i3c_device_find(dev, &i3c_id);
		if (target) {
			LOG_INF("Device %012llx DA %02x assigned (was %02x)", pid, addr,
				target->dynamic_addr);
			priv = target->controller_priv;
			priv->addr = addr;
			target->dynamic_addr = addr;
			priv->pos = pos;
		} else {
			LOG_INF("Unregistered device with PID = %012llx", pid);
		}
	}

	for (i = 0; i < config->common.dev_list.num_i3c; i++) {
		target = &config->common.dev_list.i3c[i];
		priv = target->controller_priv;
		LOG_INF("dev%d: %012llx, addr %02x, DAT pos %d", i, (uint64_t)target->pid,
			target->dynamic_addr, priv->pos);
	}

	return 0;
}

static int aspeed_i3c_priv_xfer(const struct device *dev, struct i3c_device_desc *target,
				struct i3c_msg *msgs, uint8_t num_msgs)
{
	struct aspeed_i3c_data *data = dev->data;
	struct aspeed_i3c_priv *priv = target->controller_priv;
	struct aspeed_i3c_xfer xfer;
	struct aspeed_i3c_cmd *cmds, *cmd;
	uint32_t cmd_hi, cmd_lo;
	int pos, i, ret;

	if (num_msgs == 0) {
		return 0;
	}

	pos = priv->pos;
	cmds = (struct aspeed_i3c_cmd *)k_calloc(sizeof(struct aspeed_i3c_cmd), num_msgs);

	xfer.ncmds = num_msgs;
	xfer.cmds = cmds;
	xfer.ret = -ETIMEDOUT;

	for (i = 0; i < num_msgs; i++) {
		cmd = &xfer.cmds[i];
		cmd_hi = FIELD_PREP(COMMAND_PORT_ATTR, COMMAND_PORT_XFER_ARG) |
			 FIELD_PREP(COMMAND_PORT_ARG_DATA_LEN, msgs[i].len);
		cmd_lo = 0;
		if (msgs[i].flags & I3C_MSG_READ) {
			cmd->rx_buf = msgs[i].buf;
			cmd->rx_length = msgs[i].len;
			cmd_lo |= COMMAND_PORT_READ_TRANSFER;
		} else {
			cmd->tx_buf = msgs[i].buf;
			cmd->tx_length = msgs[i].len;
		}

		cmd_lo |= FIELD_PREP(COMMAND_PORT_TID, i) |
			  FIELD_PREP(COMMAND_PORT_DEV_INDEX, pos) | COMMAND_PORT_ROC;

		if (i == num_msgs - 1) {
			cmd_lo |= COMMAND_PORT_TOC;
		}

		cmd->cmd_hi = cmd_hi;
		cmd->cmd_lo = cmd_lo;
	}

	k_sem_init(&xfer.sem, 0, 1);
	aspeed_i3c_start_xfer(data, &xfer);
	if (k_sem_take(&xfer.sem, K_MSEC(1000))) {
		LOG_ERR("%s: Failed to send I3C xfer", dev->name);
		aspeed_i3c_enter_halt(data, true);
		aspeed_i3c_reset_ctrl(data, RESET_CTRL_XFER_QUEUES);
		aspeed_i3c_exit_halt(data);
	}

	/* report actual read length */
	for (i = 0; i < num_msgs; i++) {
		cmd = &xfer.cmds[i];
		if (msgs[i].flags & I3C_MSG_READ) {
			msgs[i].len = cmd->rx_length;
		}
	}

	ret = xfer.ret;
	k_free(cmds);

	return ret;
}

static void calc_i2c_clk(int fscl, int *period_hi, int *period_lo)
{
	int hi_min, lo_min;
	int margin;
	int period = DIV_ROUND_UP(1000000000, fscl);

	if (fscl <= 100000) {
		lo_min = DIV_ROUND_UP(I3C_BUS_I2C_STD_TLOW_MIN_NS + I3C_BUS_I2C_STD_TF_MAX_NS,
				      period);
		hi_min = DIV_ROUND_UP(I3C_BUS_I2C_STD_THIGH_MIN_NS + I3C_BUS_I2C_STD_TR_MAX_NS,
				      period);
	} else if (fscl <= 400000) {
		lo_min =
			DIV_ROUND_UP(I3C_BUS_I2C_FM_TLOW_MIN_NS + I3C_BUS_I2C_FM_TF_MAX_NS, period);
		hi_min = DIV_ROUND_UP(I3C_BUS_I2C_FM_THIGH_MIN_NS + I3C_BUS_I2C_FM_TR_MAX_NS,
				      period);
	} else {
		lo_min = DIV_ROUND_UP(I3C_BUS_I2C_FMP_TLOW_MIN_NS + I3C_BUS_I2C_FMP_TF_MAX_NS,
				      period);
		hi_min = DIV_ROUND_UP(I3C_BUS_I2C_FMP_THIGH_MIN_NS + I3C_BUS_I2C_FMP_TR_MAX_NS,
				      period);
	}

	margin = (period - lo_min - hi_min) >> 1;
	*period_lo = lo_min + margin;
	*period_hi = MAX(period - *period_lo, hi_min);
}

static void aspeed_i3c_init_clock(struct aspeed_i3c_data *data)
{
	const struct aspeed_i3c_config *config = data->config;
	uint32_t core_rate, hcnt, lcnt, reg;
	int lo_ns, hi_ns;

	clock_control_get_rate(config->clock_dev, config->clock_id, &core_rate);
	data->core_period = DIV_ROUND_UP(1000000000, core_rate);

	/* I2C FM */
	if (data->common.ctrl_config.scl.i2c) {
		calc_i2c_clk(data->common.ctrl_config.scl.i2c, &hi_ns, &lo_ns);
	} else {
		calc_i2c_clk(KHZ(400), &hi_ns, &lo_ns);
	}
	hcnt = DIV_ROUND_UP(hi_ns, data->core_period);
	lcnt = DIV_ROUND_UP(lo_ns, data->core_period);
	reg = FIELD_PREP(SCL_I2C_FM_TIMING_HCNT, hcnt) |
	      FIELD_PREP(SCL_I2C_FM_TIMING_LCNT, lcnt);
	sys_write32(reg, config->base + SCL_I2C_FM_TIMING);

	/* I2C FM+: 1MHz SCL */
	calc_i2c_clk(MHZ(1), &hi_ns, &lo_ns);
	hcnt = DIV_ROUND_UP(hi_ns, data->core_period);
	lcnt = DIV_ROUND_UP(lo_ns, data->core_period);
	reg = FIELD_PREP(SCL_I2C_FMP_TIMING_HCNT, hcnt) |
	      FIELD_PREP(SCL_I2C_FMP_TIMING_LCNT, lcnt);
	sys_write32(reg, config->base + SCL_I2C_FMP_TIMING);

	/* I3C OD */
	if (data->i3c_od_scl_hi_period_ns && data->i3c_od_scl_lo_period_ns) {
		lcnt = DIV_ROUND_UP(data->i3c_od_scl_lo_period_ns,
				    data->core_period);
		hcnt = DIV_ROUND_UP(data->i3c_od_scl_hi_period_ns,
				    data->core_period);
	} else {
		/* use FM+ timing if OD periods are not specified in DT */
		lcnt = MIN(lcnt, FIELD_GET(SCL_I3C_OD_TIMING_LCNT,
					   SCL_I3C_OD_TIMING_LCNT));
		hcnt = MIN(hcnt, FIELD_GET(SCL_I3C_OD_TIMING_HCNT,
					   SCL_I3C_OD_TIMING_HCNT));
	}
	reg = FIELD_PREP(SCL_I3C_OD_TIMING_HCNT, hcnt) |
	      FIELD_PREP(SCL_I3C_OD_TIMING_LCNT, lcnt);
	sys_write32(reg, config->base + SCL_I3C_OD_TIMING);

	/* I3C PP */
	if (data->i3c_pp_scl_hi_period_ns && data->i3c_pp_scl_lo_period_ns) {
		lcnt = DIV_ROUND_UP(data->i3c_pp_scl_lo_period_ns,
				    data->core_period);
		hcnt = DIV_ROUND_UP(data->i3c_pp_scl_hi_period_ns,
				    data->core_period);
	} else {
		hcnt = DIV_ROUND_UP(I3C_BUS_THIGH_MAX_NS, data->core_period) -
		       1;
		lcnt = DIV_ROUND_UP(core_rate,
				    data->common.ctrl_config.scl.i3c) -
		       hcnt;
	}
	reg = FIELD_PREP(SCL_I3C_PP_TIMING_HCNT, hcnt) |
	      FIELD_PREP(SCL_I3C_PP_TIMING_LCNT, lcnt);
	sys_write32(reg, config->base + SCL_I3C_PP_TIMING);

	/* SDA TX hold time */
	lcnt = CLAMP(DIV_ROUND_UP(data->sda_tx_hold_ns, data->core_period),
		     SDA_TX_HOLD_MIN, SDA_TX_HOLD_MAX);
	reg = sys_read32(config->base + SDA_HOLD_SWITCH_DLY_TIMING);
	reg &= ~SDA_TX_HOLD;
	reg |= lcnt;
	sys_write32(reg, config->base + SDA_HOLD_SWITCH_DLY_TIMING);

	sys_write32(0xffff007c, config->base + BUS_FREE_TIMING);
}

static void aspeed_i3c_set_role(struct aspeed_i3c_data *data, int secondary)
{
	const struct aspeed_i3c_config *config = data->config;
	uint32_t reg, role;

	role = DEVICE_CTRL_ROLE_MASTER;
	if (secondary) {
		LOG_DBG("%s is secondary", data->dev->name);
		role = DEVICE_CTRL_ROLE_SLAVE;
	}

	reg = sys_read32(config->base + DEVICE_CTRL_EXTENDED);
	reg &= ~DEVICE_CTRL_ROLE_MASK;
	reg |= FIELD_PREP(DEVICE_CTRL_ROLE_MASK, role);
	sys_write32(reg, config->base + DEVICE_CTRL_EXTENDED);
}

static void aspeed_i3c_target_worker(struct k_work *work)
{
	struct aspeed_i3c_data *data =
		CONTAINER_OF(work, struct aspeed_i3c_data, target_work);

	/*
	 * The hardware allows SIR for target mode (SLV_EVENT_CTRL_SIR_EN = 1)
	 * by default. This would cause problems if the bus controller doesn't
	 * issue ENTDAA or DISEC to the target. Therefore, we've added a
	 * software flag to postpone the timing of enabling SIR for 1 second.
	 */
	k_msleep(1000);
	data->sir_allowed_by_sw = true;
}

static int aspeed_i3c_init_hw_feature(struct aspeed_i3c_data *data)
{
	uint32_t scu = DT_REG_ADDR(DT_NODELABEL(syscon));
	uint32_t rev_id = FIELD_GET(GENMASK(31, 16), sys_read32(scu + 0x4));

	data->ibi_pec_force_enable = false;

	switch (rev_id) {
	case 0x0503: /* AST26x0A3 */
		data->ibi_pec_force_enable = true;
		__fallthrough;
	case 0x8001: /* AST1030A1 */
	case 0xa001: /* AST1060A1 */
	case 0xa003: /* AST1060A2 */
	case 0x8003: /* AST1060A1-ENG */
		return 0;
	default:
		LOG_ERR("HW revision %04x not supported", rev_id);
		return -EOPNOTSUPP;
	}
}

static void aspeed_i3c_init_pid(struct aspeed_i3c_data *data)
{
	uint32_t scu = DT_REG_ADDR(DT_NODELABEL(syscon));
	uint32_t rev_id = FIELD_GET(GENMASK(31, 16), sys_read32(scu + 0x4));
	uint32_t reg;

	reg = FIELD_PREP(SLV_PID_MFG_ID, MIPI_MFG_ASPEED) |
	      FIELD_PREP(SLV_PID_DCR_SELECT, DCR_SELECT_FIXED);
	sys_write32(reg, data->config->base + SLV_PID_HI);

	reg = FIELD_PREP(SLV_PID_PART_ID, rev_id) |
	      FIELD_PREP(SLV_PID_INST_ID, data->config->global_idx);
	sys_write32(reg, data->config->base + SLV_PID_LO);
}

static int aspeed_i3c_target_tx_write(const struct device *dev, uint8_t *buf,
				      uint16_t len)
{
	struct aspeed_i3c_data *data = dev->data;
	uintptr_t base = data->config->base;
	uint32_t cmd;

	aspeed_i3c_wr_tx_fifo(data, buf, len);
	cmd = FIELD_PREP(COMMAND_PORT_ATTR, COMMAND_PORT_SLAVE_DATA_CMD) |
	      FIELD_PREP(COMMAND_PORT_SLAVE_DATA_LEN, len) |
	      FIELD_PREP(COMMAND_PORT_TID, TID_TARGET_RD_DATA);
	sys_write32(cmd, base + COMMAND_QUEUE_PORT);

	return 0;
}

static int aspeed_i3c_target_pending_read_notify(const struct device *dev,
						 uint8_t *buf, uint16_t len,
						 struct i3c_ibi *notifier)
{
	struct aspeed_i3c_data *data = dev->data;
	uintptr_t base = data->config->base;
	uint32_t reg, cmd;

	reg = sys_read32(base + SLV_EVENT_CTRL);
	if (!(data->sir_allowed_by_sw && reg & SLV_EVENT_CTRL_SIR_EN)) {
		return -EACCES;
	}

	if (notifier->payload_len == 0) {
		return -EPERM;
	}

	aspeed_i3c_set_ibi_mdb(data, notifier->payload[0]);
	aspeed_i3c_wr_tx_fifo(data, notifier->payload, notifier->payload_len);
	cmd = FIELD_PREP(COMMAND_PORT_ATTR, COMMAND_PORT_SLAVE_DATA_CMD) |
	      FIELD_PREP(COMMAND_PORT_SLAVE_DATA_LEN, notifier->payload_len) |
	      FIELD_PREP(COMMAND_PORT_TID, TID_TARGET_IBI);
	sys_write32(cmd, base + COMMAND_QUEUE_PORT);
	k_sem_init(&data->target_ibi_sem, 0, 1);

	reg = sys_read32(base + QUEUE_THLD_CTRL);
	reg &= ~QUEUE_THLD_CTRL_RESP_BUF;
	reg |= FIELD_PREP(QUEUE_THLD_CTRL_RESP_BUF, 1 - 1);
	sys_write32(reg, base + QUEUE_THLD_CTRL);

	aspeed_i3c_target_tx_write(dev, buf, len);
	k_sem_init(&data->target_data_sem, 0, 1);

	sys_write32(1, base + SLV_INTR_REQ);
	if (k_sem_take(&data->target_ibi_sem, K_MSEC(1000))) {
		LOG_WRN("%s SIR timeout! Reset I3C controller", data->dev->name);
		aspeed_i3c_init(data->dev);
		return -EIO;
	}

	if (k_sem_take(&data->target_data_sem, K_MSEC(1000))) {
		LOG_WRN("%s wait master read timeout! Reset queues", data->dev->name);
		aspeed_i3c_disable(data);
		aspeed_i3c_reset_ctrl(data, RESET_CTRL_QUEUES);
		aspeed_i3c_enable(data);
		return -ETIMEDOUT;
	}

	return 0;
}

static int aspeed_i3c_target_register(const struct device *dev, struct i3c_target_config *cfg)
{
	struct aspeed_i3c_data *data = dev->data;
	uintptr_t base = data->config->base;

	if (!data->target_config) {
		data->target_config = cfg;
	}

	data->target_config->address =
		FIELD_GET(DEV_ADDR_DYNAMIC_ADDR, sys_read32(base + DEVICE_ADDR));

	return 0;
}

static int aspeed_i3c_target_unregister(const struct device *dev, struct i3c_target_config *cfg)
{
	struct aspeed_i3c_data *data = dev->data;

	data->target_config = NULL;
	return 0;
}

static int aspeed_i3c_init(const struct device *dev)
{
	const struct aspeed_i3c_config *config = dev->config;
	struct aspeed_i3c_data *data = dev->data;
	uint32_t reg;
	int ret, i;

	data->config = config;
	data->dev = dev;

	LOG_INF("%s init", dev->name);

	ret = aspeed_i3c_init_hw_feature(data);
	if (ret) {
		return ret;
	}

	/* Global reset is shared, so just need to deassert it */
	ret = reset_line_deassert_dt(&config->global_reset);
	__ASSERT_NO_MSG(ret == 0);

	reg = FIELD_PREP(I3CG_REG1_ACT_MODE, 1) |
	      FIELD_PREP(I3CG_REG1_INST_ID, config->global_idx) |
	      FIELD_PREP(I3CG_REG1_SLV_STATIC_ADDR, 0x74);
	sys_write32(reg, config->global_regs + ASPEED_I3CG_REG1(config->global_idx));

	ret = aspeed_i3c_pullup_to_reg(2000, &reg);
	sys_write32(reg, config->global_regs + ASPEED_I3CG_REG0(config->global_idx));

	ret = reset_line_assert_dt(&config->core_reset);
	__ASSERT_NO_MSG(ret == 0);
	ret = clock_control_on(config->clock_dev, config->clock_id);
	__ASSERT_NO_MSG(ret == 0);
	ret = reset_line_deassert_dt(&config->core_reset);
	__ASSERT_NO_MSG(ret == 0);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	__ASSERT_NO_MSG(ret == 0);

	aspeed_i3c_disable(data);

	/* Controller reset */
	sys_write32(RESET_CTRL_ALL, config->base + RESET_CTRL);
	ret = read32_poll_timeout(config->base + RESET_CTRL, reg, reg == 0, 0, 1000);
	if (ret) {
		return ret;
	}

	aspeed_i3c_set_role(data, data->common.ctrl_config.is_secondary);
	aspeed_i3c_init_clock(data);

	/* Init interrupt mask */
	sys_write32(0xffffffff, config->base + INTR_STATUS);
	if (data->common.ctrl_config.is_secondary) {
		reg = INTR_TRANSFER_ERR_STAT | INTR_RESP_READY_STAT | INTR_CCC_UPDATED_STAT |
		      INTR_DYN_ADDR_ASSGN_STAT | INTR_IBI_UPDATED_STAT | INTR_READ_REQ_RECV_STAT;
	} else {
		reg = INTR_TRANSFER_ERR_STAT | INTR_RESP_READY_STAT;
	}
	sys_write32(reg, config->base + INTR_STATUS_EN);
	sys_write32(reg, config->base + INTR_SIGNAL_EN);
	config->irq_config_func(dev);

	/* Disable the SIR for target mode */
	data->sir_allowed_by_sw = false;
	k_work_init(&data->target_work, aspeed_i3c_target_worker);

	/* Init hardware queues */
	reg = FIELD_PREP(QUEUE_THLD_CTRL_IBI_DAT, MAX_IBI_FRAG_SIZE >> 2);
	sys_write32(reg, config->base + QUEUE_THLD_CTRL);

	reg = sys_read32(config->base + DATA_BUFFER_THLD_CTRL);
	reg &= ~DATA_BUFFER_THLD_CTRL_RX_BUF;
	sys_write32(reg, config->base + DATA_BUFFER_THLD_CTRL);

	/* Init PID and DCR for target/secondary mode */
	aspeed_i3c_init_pid(data);
	/* TODO: Add GPIO extra information */

	/* Get max device and DAT start address */
	reg = sys_read32(config->base + DEVICE_ADDR_TABLE_POINTER);
	data->datstartaddr = FIELD_GET(DEVICE_ADDR_TABLE_ADDR, reg);
	data->maxdevs = FIELD_GET(DEVICE_ADDR_TABLE_DEPTH, reg);
	data->free_pos = GENMASK(data->maxdevs - 1, 0);
	data->need_da = 0;

	/* Init DAT */
	reg = DEV_ADDR_TABLE_SIR_REJECT | DEV_ADDR_TABLE_MR_REJECT;
	for (i = 0; i < data->maxdevs; i++) {
		sys_write32(reg, config->base + data->datstartaddr + (i << 2));
	}

	sys_write32(IBI_REQ_REJECT_ALL, config->base + IBI_MR_REQ_REJECT);
	sys_write32(IBI_REQ_REJECT_ALL, config->base + IBI_SIR_REQ_REJECT);
	sys_write32(sys_read32(config->base + DEVICE_CTRL) | DEV_CTRL_HOT_JOIN_NACK,
		    config->base + DEVICE_CTRL);

	ret = i3c_addr_slots_init(dev);
	if (ret) {
		return ret;
	}

	ret = i3c_addr_slots_next_free_find(&data->common.attached_dev.addr_slots);
	if (!data->common.ctrl_config.is_secondary) {
		reg = FIELD_PREP(DEV_ADDR_DYNAMIC_ADDR, ret) | DEV_ADDR_DYNAMIC_ADDR_VALID;
		sys_write32(reg, config->base + DEVICE_ADDR);
	} else {
		reg = FIELD_PREP(DEV_ADDR_STATIC_ADDR, ret) | DEV_ADDR_STATIC_ADDR_VALID;
		sys_write32(reg, config->base + DEVICE_ADDR);
	}
	i3c_addr_slots_mark_i3c(&data->common.attached_dev.addr_slots, ret);

	aspeed_i3c_enable(data);

	/* Perform bus initialization */
	if (!data->common.ctrl_config.is_secondary) {
		ret = i3c_bus_init(dev, &config->common.dev_list);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static void aspeed_i3c_target_handle_ccc_update(struct aspeed_i3c_data *data)
{
	uintptr_t base = data->config->base;
	uint32_t reg;
	uint32_t event = sys_read32(base + SLV_EVENT_CTRL);

	reg = sys_read32(base + SLV_MAX_LEN);
	if (event & SLV_EVENT_CTRL_MRL_UPD) {
		/* TBD */
		LOG_INF("MRL=%ld", FIELD_GET(SLV_MAX_RD_LEN, reg));
	}

	if (event & SLV_EVENT_CTRL_MWL_UPD) {
		/* TBD */
		LOG_INF("MWL=%ld", FIELD_GET(SLV_MAX_WR_LEN, reg));
	}

	sys_write32(event, base + SLV_EVENT_CTRL);

	reg = sys_read32(base + PRESENT_STATE);
	if (FIELD_GET(CM_TFR_STS, reg) == CM_TFR_STS_SLAVE_HALT) {
		aspeed_i3c_enter_halt(data, false);
		aspeed_i3c_exit_halt(data);
	}
}

static void aspeed_i3c_drain_fifo(struct aspeed_i3c_data *data, uint32_t reg, int len)
{
	uintptr_t base = data->config->base;
	int i, nwords;

	nwords = (len + 3) >> 2;
	for (i = 0; i < nwords; i++) {
		sys_read32(base + reg);
	}
}

static void aspeed_i3c_drain_rx_queue(struct aspeed_i3c_data *data, int len)
{
	aspeed_i3c_drain_fifo(data, RX_TX_DATA_PORT, len);
}

static void aspeed_i3c_drain_ibi_queue(struct aspeed_i3c_data *data, int len)
{
	aspeed_i3c_drain_fifo(data, IBI_QUEUE_STATUS, len);
}

static void aspeed_i3c_target_handle_response_ready(struct aspeed_i3c_data *data)
{
	uintptr_t base = data->config->base;
	uint32_t reg = sys_read32(base + QUEUE_STATUS_LEVEL);
	uint32_t nresp = FIELD_GET(QUEUE_STATUS_LEVEL_RESP, reg);
	const struct i3c_target_callbacks *cbs = NULL;
	int ret, i, j;
	bool drop_wr_data = true;

	if (data->target_config) {
		cbs = data->target_config->callbacks;
		if (cbs && cbs->write_requested_cb) {
			drop_wr_data = false;
		}
	}

	for (i = 0; i < nresp; i++) {
		uint32_t resp = sys_read32(base + RESPONSE_QUEUE_PORT);
		uint32_t nbytes = FIELD_GET(RESPONSE_PORT_DATA_LEN, resp);
		uint32_t tid = FIELD_GET(RESPONSE_PORT_TID, resp);
		uint8_t *buf;

		if (nbytes) {
			if (!drop_wr_data) {
				ret = cbs->write_requested_cb(data->target_config);
				if (ret) {
					drop_wr_data = true;
				}
			}

			if (drop_wr_data) {
				LOG_DBG("drop %d bytes received data", nbytes);
				aspeed_i3c_drain_rx_queue(data, nbytes);
			} else {
				buf = k_calloc(1, nbytes);
				aspeed_i3c_rd_rx_fifo(data, buf, nbytes);
				for (j = 0; j < nbytes; j++) {
					cbs->write_received_cb(data->target_config, buf[j]);
				}
				k_free(buf);
			}
		}

		if (tid == TID_SLAVE_IBI_DONE) {
			k_sem_give(&data->target_ibi_sem);
		}

		if (tid == TID_MASTER_READ_DATA) {
			k_sem_give(&data->target_data_sem);
		}
	}
}

static void aspeed_i3c_handle_ibi_sir(struct aspeed_i3c_data *data, uint32_t status)
{
	struct i3c_device_desc *desc;
	uint8_t addr = IBI_QUEUE_IBI_ADDR(status);
	uint8_t len = FIELD_GET(IBI_QUEUE_STATUS_DATA_LEN, status);
	uint8_t buf[2];
	int pos = aspeed_i3c_get_addr_pos(data, addr);

	if (pos < 0) {
		goto drop;
	}

	desc = i3c_dev_list_i3c_addr_find(&data->common.attached_dev, addr);
	aspeed_i3c_rd_ibi_fifo(data, buf, len);
	i3c_ibi_work_enqueue_target_irq(desc, buf, len);
	return;
drop:
	aspeed_i3c_drain_ibi_queue(data, len);
}

static void aspeed_i3c_handle_ibis(struct aspeed_i3c_data *data)
{
	uintptr_t base = data->config->base;
	uint32_t reg, nibis, i, len;

	reg = sys_read32(base + QUEUE_STATUS_LEVEL);
	nibis = FIELD_GET(QUEUE_STATUS_IBI_STATUS_CNT, reg);
	if (nibis == 0) {
		return;
	}

	for (i = 0; i < nibis; i++) {
		reg = sys_read32(base + IBI_QUEUE_STATUS);
		if (IBI_TYPE_SIRQ(reg)) {
			aspeed_i3c_handle_ibi_sir(data, reg);
		} else {
			len = FIELD_GET(IBI_QUEUE_STATUS_DATA_LEN, reg);
			aspeed_i3c_drain_ibi_queue(data, len);
		}
	}
}

static void i3c_aspeed_isr(const struct device *dev)
{
	struct aspeed_i3c_data *data = dev->data;
	uintptr_t base = data->config->base;
	uint32_t status;

	status = sys_read32(base + INTR_STATUS);
	LOG_DBG("%s isr status = %08x", dev->name, status);

	if (data->common.ctrl_config.is_secondary) {
		if (status & INTR_DYN_ADDR_ASSGN_STAT) {
			uint8_t da =
				FIELD_GET(DEV_ADDR_DYNAMIC_ADDR, sys_read32(base + DEVICE_ADDR));

			LOG_DBG("%s DA %02x assigned", data->dev->name, da);
			if (data->target_config) {
				data->target_config->address = da;
			}
			k_work_submit(&data->target_work);
		}

		if (status & INTR_RESP_READY_STAT) {
			aspeed_i3c_target_handle_response_ready(data);
		}

		if (status & INTR_CCC_UPDATED_STAT) {
			aspeed_i3c_target_handle_ccc_update(data);
		}
	} else {

		if (status & INTR_RESP_READY_STAT || status & INTR_TRANSFER_ERR_STAT) {
			aspeed_i3c_end_xfer(data);
		}

		if (status & INTR_IBI_THLD_STAT) {
			aspeed_i3c_handle_ibis(data);
		}
	}

	sys_write32(status, base + INTR_STATUS);
}

static struct i3c_driver_api aspeed_i3c_driver_api = {
	.attach_i3c_device = aspeed_i3c_attach_i3c_device,
	.detach_i3c_device = aspeed_i3c_detach_i3c_device,
	.do_ccc = aspeed_i3c_do_ccc,
	.do_daa = aspeed_i3c_do_daa,
	.i3c_device_find = aspeed_i3c_device_find,
	.i3c_xfers = aspeed_i3c_priv_xfer,
	.ibi_enable = aspeed_i3c_ibi_enable,
	.ibi_disable = aspeed_i3c_ibi_disable,
#ifdef CONFIG_I3C_USE_IBI
	.target_pending_read_notify = aspeed_i3c_target_pending_read_notify,
#endif
	.target_tx_write = aspeed_i3c_target_tx_write,
	.target_register = aspeed_i3c_target_register,
	.target_unregister = aspeed_i3c_target_unregister,
};

#define I3C_ASPEED_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void aspeed_i3c_config_func_##n(const struct device *dev);                          \
	static struct i3c_device_desc aspeed_i3c_dev_array_##n[] = I3C_DEVICE_ARRAY_DT_INST(n);    \
	static struct i3c_i2c_device_desc aspeed_i3c_i2c_dev_array_##n[] =                         \
		I3C_I2C_DEVICE_ARRAY_DT_INST(n);                                                   \
	static const struct aspeed_i3c_config aspeed_i3c_config_##n = {                            \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.global_regs = DT_REG_ADDR(DT_PHANDLE_BY_IDX(DT_DRV_INST(n), global_regs, 0)),     \
		.global_idx = DT_PHA_BY_IDX(DT_DRV_INST(n), global_regs, 0, id),                   \
		.global_reset.dev = DEVICE_DT_GET(DT_INST_RESET_CTLR_BY_NAME(n, global)),          \
		.global_reset.id = DT_RESET_CELL_BY_NAME(DT_DRV_INST(n), global, id),              \
		.core_reset.dev = DEVICE_DT_GET(DT_INST_RESET_CTLR_BY_NAME(n, core)),              \
		.core_reset.id = DT_RESET_CELL_BY_NAME(DT_DRV_INST(n), core, id),                  \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, clk_id),                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.common.dev_list.i3c = aspeed_i3c_dev_array_##n,                                   \
		.common.dev_list.num_i3c = ARRAY_SIZE(aspeed_i3c_dev_array_##n),                   \
		.common.dev_list.i2c = aspeed_i3c_i2c_dev_array_##n,                               \
		.common.dev_list.num_i2c = ARRAY_SIZE(aspeed_i3c_i2c_dev_array_##n),               \
		.irq_config_func = aspeed_i3c_config_func_##n,                                     \
	};                                                                                         \
                                                                                                   \
	static struct aspeed_i3c_data aspeed_i3c_data##n = {                                       \
		.common.ctrl_config.scl.i3c = DT_INST_PROP_OR(n, i3c_scl_hz, 0),                   \
		.common.ctrl_config.scl.i2c = DT_INST_PROP_OR(n, i2c_scl_hz, 0),                   \
		.common.ctrl_config.is_secondary = DT_INST_PROP_OR(n, secondary, 0),               \
		.i3c_pp_scl_hi_period_ns = DT_INST_PROP_OR(n, i3c_pp_scl_hi_period_ns, 0),         \
		.i3c_pp_scl_lo_period_ns = DT_INST_PROP_OR(n, i3c_pp_scl_lo_period_ns, 0),         \
		.i3c_od_scl_hi_period_ns = DT_INST_PROP_OR(n, i3c_od_scl_hi_period_ns, 0),         \
		.i3c_od_scl_lo_period_ns = DT_INST_PROP_OR(n, i3c_od_scl_lo_period_ns, 0),         \
		.sda_tx_hold_ns = DT_INST_PROP_OR(n, sda_tx_hold_ns, 10),                          \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, aspeed_i3c_init, NULL, &aspeed_i3c_data##n,                       \
			      &aspeed_i3c_config_##n, POST_KERNEL,                                 \
			      CONFIG_I3C_CONTROLLER_INIT_PRIORITY, &aspeed_i3c_driver_api);        \
                                                                                                   \
	static void aspeed_i3c_config_func_##n(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i3c_aspeed_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(I3C_ASPEED_INIT)
