/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_i3c

#include <drivers/clock_control.h>
#include <drivers/reset_control.h>
#include <soc.h>
#include <sys/util.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <sys/sys_io.h>
#include <logging/log.h>
#define LOG_LEVEL CONFIG_I3C_LOG_LEVEL
LOG_MODULE_REGISTER(i3c);

union i3c_device_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t boradcast_addr_inc : 1;	/* bit[0] */
		volatile uint32_t reserved0 : 6;		/* bit[6:1] */
		volatile uint32_t i2c_slave_present : 1;	/* bit[7] */
		volatile uint32_t hj_ack_ctrl : 1;		/* bit[8] */
		volatile uint32_t slave_ibi_payload_en : 1;	/* bit[9] */
		volatile uint32_t slave_pec_en : 1;		/* bit[10] */
		volatile uint32_t reserved1 : 5;		/* bit[15:11] */
		volatile uint32_t slave_mdb : 8;		/* bit[23:16] */
		volatile uint32_t reserved2 : 3;		/* bit[26:24] */
		volatile uint32_t slave_auto_mode_adapt : 1;	/* bit[27] */
		volatile uint32_t dma_handshake_en : 1;		/* bit[28] */
		volatile uint32_t abort : 1;			/* bit[29] */
		volatile uint32_t resume : 1;			/* bit[30] */
		volatile uint32_t enable : 1;			/* bit[31] */
	} fields;
}; /* offset 0x00 */

union i3c_device_addr_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t static_addr : 7;		/* bit[6:0] */
		volatile uint32_t reserved0 : 8;		/* bit[14:7] */
		volatile uint32_t static_addr_valid : 1;	/* bit[15] */
		volatile uint32_t dynamic_addr : 7;		/* bit[22:16] */
		volatile uint32_t reserved1 : 8;		/* bit[30:23] */
		volatile uint32_t dynamic_addr_valid : 1;	/* bit[31] */
	} fields;
}; /* offset 0x04 */

union i3c_device_cmd_queue_port_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t static_addr : 7;		/* bit[6:0] */
		volatile uint32_t reserved0 : 8;		/* bit[14:7] */
		volatile uint32_t static_addr_valid : 1;	/* bit[15] */
		volatile uint32_t dynamic_addr : 7;		/* bit[22:16] */
		volatile uint32_t reserved1 : 8;		/* bit[30:23] */
		volatile uint32_t dynamic_addr_valid : 1;	/* bit[31] */
	} fields;
}; /* offset 0x0c */

union i3c_device_resp_queue_port_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t data_length : 16;		/* bit[15:0] */
		volatile uint32_t ccct : 8;			/* bit[23:16] */
		volatile uint32_t tid : 4;			/* bit[27:24] */
		volatile uint32_t err_status : 4;		/* bit[31:28] */
	} fields;
}; /* offset 0x10 */

union i3c_queue_thld_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t cmd_q_empty_thld : 8;		/* bit[7:0] */
		volatile uint32_t resp_q_thld : 8;		/* bit[15:8] */
#define MAX_IBI_CHUNK_IN_BYTE	124
		volatile uint32_t ibi_data_thld : 8;		/* bit[23:16] */
		volatile uint32_t ibi_status_thld : 8;		/* bit[31:24] */
	} fields;
}; /* offset 0x1c */

union i3c_data_buff_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t tx_thld : 3;			/* bit[2:0] */
		volatile uint32_t reserved0 : 5;		/* bit[7:3] */
		volatile uint32_t rx_thld : 3;			/* bit[10:8] */
		volatile uint32_t reserved1 : 5;		/* bit[15:11] */
		volatile uint32_t tx_start_thld : 3;		/* bit[18:16] */
		volatile uint32_t reserved2 : 5;		/* bit[23:19] */
		volatile uint32_t rx_start_thld : 3;		/* bit[26:24] */
		volatile uint32_t reserved3 : 5;		/* bit[31:27] */
	} fields;
}; /* offset 0x20 */
union i3c_reset_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t core_reset : 1;		/* bit[0] */
		volatile uint32_t cmd_queue_reset : 1;		/* bit[1] */
		volatile uint32_t resp_queue_reset : 1;		/* bit[2] */
		volatile uint32_t tx_queue_reset : 1;		/* bit[3] */
		volatile uint32_t rx_queue_reset : 1;		/* bit[4] */
		volatile uint32_t ibi_queue_reset : 1;		/* bit[5] */
		volatile uint32_t reserved : 23;		/* bit[28:6] */
		volatile uint32_t bus_reset_type : 2;		/* bit[30:29] */
		volatile uint32_t bus_reset : 1;		/* bit[31] */
	} fields;
}; /* offset 0x34 */

union i3c_slave_event_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t sir_allowed : 1;		/* bit[0] */
		volatile uint32_t mr_allowed : 1;		/* bit[1] */
		volatile uint32_t reserved0 : 1;		/* bit[2] */
		volatile uint32_t hj_allowed : 1;		/* bit[3] */
		volatile uint32_t act_state : 2;		/* bit[5:4] */
		volatile uint32_t mrl_update : 1;		/* bit[6] */
		volatile uint32_t mwl_update : 1;		/* bit[7] */
		volatile uint32_t reserved1 : 24;		/* bit[31:8] */
	} fields;
}; /* offset 0x38 */

union i3c_intr_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t tx_thld : 1;			/* bit[0] */
		volatile uint32_t rx_thld : 1;			/* bit[1] */
		volatile uint32_t ibi_thld : 1;			/* bit[2] */
		volatile uint32_t cmd_q_ready : 1;		/* bit[3] */
		volatile uint32_t resp_q_ready : 1;		/* bit[4] */
		volatile uint32_t xfr_abort : 1;		/* bit[5] */
		volatile uint32_t ccc_update : 1;		/* bit[6] */
		volatile uint32_t reserved0 : 1;		/* bit[7] */
		volatile uint32_t dyn_addr_assign : 1;		/* bit[8] */
		volatile uint32_t xfr_error : 1;		/* bit[9] */
		volatile uint32_t defslv : 1;			/* bit[10] */
		volatile uint32_t read_q_recv : 1;		/* bit[11] */
		volatile uint32_t ibi_update : 1;		/* bit[12] */
		volatile uint32_t bus_owner_update : 1;		/* bit[13] */
		volatile uint32_t reserved1 : 1;		/* bit[14] */
		volatile uint32_t bus_reset_done : 1;		/* bit[15] */
		volatile uint32_t reserved2 : 16;		/* bit[31:16] */
	} fields;
}; /* offset 0x3c ~ 0x48 */

union i3c_dev_addr_tbl_ptr_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t start_addr : 16;		/* bit[15:0] */
		volatile uint32_t depth : 16;			/* bit[31:16] */
	} fields;
}; /* offset 0x5c */

union i3c_slave_pid_hi_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t dcr_select : 1;		/* bit[0] */
#define DCR_SELECT_VENDOR_FIXED	0
#define DCR_SELECT_RANDOM	1
		volatile uint32_t mipi_mfg_id : 15;		/* bit[15:1] */
#define MIPI_MFG_ASPEED		0x03f6
		volatile uint32_t reserved0 : 16;		/* bit[31:16] */
	} fields;
}; /* offset 0x70 */

union i3c_slave_pid_lo_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t dcr : 12;			/* bit[11:0] */
		volatile uint32_t inst_id : 4;			/* bit[15:12] */
		volatile uint32_t part_id : 16;			/* bit[31:16] */
	} fields;
}; /* offset 0x74 */

union i3c_slave_char_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t bcr : 8;			/* bit[7:0] */
		volatile uint32_t dcr : 8;			/* bit[15:8] */
		volatile uint32_t hdr_cap : 8;			/* bit[23:16] */
		volatile uint32_t reserved0 : 8;		/* bit[31:24] */
	} fields;
}; /* offset 0x78 */

union i3c_device_ctrl_extend_s {
	volatile uint32_t value;
	struct {
#define DEVICE_CTRL_EXT_ROLE_MASTER	0
#define DEVICE_CTRL_EXT_ROLE_SLAVE	1
		volatile uint32_t role : 2;			/* bit[1:0] */
		volatile uint32_t reserved0 : 1;		/* bit[2] */
		volatile uint32_t reqmst_ack : 1;		/* bit[3] */
		volatile uint32_t reserved1 : 28;		/* bit[31:4] */
	} fields;
}; /* offset 0xb0 */

union i3c_scl_timing_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t lcnt : 8;			/* bit[7:0] */
		volatile uint32_t reserved0 : 8;		/* bit[15:8] */
		volatile uint32_t hcnt : 8;			/* bit[23:16] */
		volatile uint32_t reserved1 : 8;		/* bit[31:24] */
	} fields;
}; /* offset 0xb4 and 0xb8 */

union i2c_scl_timing_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t lcnt : 16;			/* bit[15:0] */
		volatile uint32_t hcnt : 16;			/* bit[31:16] */
	} fields;
}; /* offset 0xbc and 0xc0 */

union i3c_ext_termn_timing_s {
	volatile uint32_t value;
	struct {
#define DEFAULT_EXT_TERMN_LCNT	4
		volatile uint32_t lcnt : 4;			/* bit[3:0] */
		volatile uint32_t reserved0 : 12;		/* bit[15:4] */
		volatile uint32_t i3c_ts_skew_cnt : 4;		/* bit[19:16] */
		volatile uint32_t reserved1 : 12;		/* bit[31:20] */
	} fields;
}; /* offset 0xcc */

struct i3c_register_s {
	union i3c_device_ctrl_s device_ctrl;			/* 0x0 */
	union i3c_device_addr_s device_addr;			/* 0x4 */
	uint32_t hw_capability;					/* 0x8 */
	union i3c_device_cmd_queue_port_s cmd_queue_port;	/* 0xc */
	union i3c_device_resp_queue_port_s resp_queue_port;	/* 0x10 */
	uint32_t rx_tx_data_port;				/* 0x14 */
	uint32_t ibi_queue_data;				/* 0x18 */
	union i3c_queue_thld_ctrl_s queue_thld_ctrl;		/* 0x1c */
	union i3c_data_buff_ctrl_s data_buff_ctrl;		/* 0x20 */
	uint32_t reserved0[2];					/* 0x24 ~ 0x28 */
	uint32_t mr_reject;					/* 0x2c */
	uint32_t sir_reject;					/* 0x30 */
	union i3c_reset_ctrl_s reset_ctrl;			/* 0x34 */
	union i3c_slave_event_ctrl_s slave_event_ctrl;		/* 0x38 */
	union i3c_intr_s intr_status;				/* 0x3c */
	union i3c_intr_s intr_status_en;			/* 0x40 */
	union i3c_intr_s intr_signal_en;			/* 0x44 */
	union i3c_intr_s intr_force_en;				/* 0x48 */
	uint32_t reserved1[4];					/* 0x4c ~ 0x58 */
	union i3c_dev_addr_tbl_ptr_s dev_addr_tbl_ptr;		/* 0x5c */
	uint32_t reserved2[4];					/* 0x60 ~ 0x6c */
	union i3c_slave_pid_hi_s slave_pid_hi;			/* 0x70 */
	union i3c_slave_pid_lo_s slave_pid_lo;			/* 0x74 */
	union i3c_slave_char_s slave_char;			/* 0x78 */
	uint32_t reserved3[13];					/* 0x7c ~ 0xac */
	union i3c_device_ctrl_extend_s device_ctrl_ext;		/* 0xb0 */
	union i3c_scl_timing_s op_timing;			/* 0xb4 */
	union i3c_scl_timing_s pp_timing;			/* 0xb8 */
	union i2c_scl_timing_s fm_timing;			/* 0xbc */
	union i2c_scl_timing_s fmp_timing;			/* 0xc0 */
	uint32_t reserved4[2];					/* 0xc4 ~ 0xc8 */
	union i3c_ext_termn_timing_s ext_termn_timing;		/* 0xcc */
};

struct i3c_aspeed_config {
	struct i3c_register_s *base;
	const struct device *clock_dev;
	const reset_control_subsys_t reset_id;
	const clock_control_subsys_t clock_id;
	uint32_t i3c_scl_hz;
	uint32_t i2c_scl_hz;
	int secondary;
	int assigned_addr;
	int inst_id;
};

struct i3c_aspeed_data {
	struct {
		uint32_t ibi_status_correct : 1;
		uint32_t reserved : 31;
	} hw_feature;

	union i3c_dev_addr_tbl_ptr_s hw_dat;
	uint32_t hw_dat_free_pos;
	uint8_t dev_addr_tbl[256];

};

#define DEV_CFG(dev)			((struct i3c_aspeed_config *)(dev)->config)
#define DEV_DATA(dev)			((struct i3c_aspeed_data *)(dev)->data)

static void i3c_aspeed_isr(const struct device *dev)
{
	struct i3c_aspeed_config *config = DEV_CFG(dev);
	union i3c_intr_s status;

	status.value = config->base->intr_status.value;
	if (config->secondary && status.fields.dyn_addr_assign) {
		LOG_DBG("slave received DA assignment\n");
	}
}

static void i3c_aspeed_init_clock(const struct device *dev)
{
	struct i3c_aspeed_config *config = DEV_CFG(dev);
	struct i3c_register_s *i3c_register = config->base;
	union i3c_scl_timing_s reg;
	int i3cclk, tck_ns, period, hcnt, lcnt;

	clock_control_get_rate(config->clock_dev, config->clock_id, &i3cclk);
	LOG_DBG("i3cclk %d hz\n", i3cclk);

	tck_ns = 1000000000 / i3cclk;
	__ASSERT(tck_ns > 0, "i3c clock too fast\n");

	LOG_INF("i2c-scl = %d, i3c-scl = %d\n", config->i2c_scl_hz, config->i3c_scl_hz);

	/* Configure OP mode timing parameters */
	period = (1000000000 / config->i2c_scl_hz) >> 1;
	hcnt = lcnt = DIV_ROUND_UP(period, tck_ns);

	reg.value = 0;
	reg.fields.hcnt = hcnt;
	reg.fields.lcnt = lcnt;
	i3c_register->op_timing.value = reg.value;
	i3c_register->fm_timing.value = reg.value;
	i3c_register->fmp_timing.value = reg.value;

	/* Configure PP mode timing parameters */
	period = (1000000000 / config->i3c_scl_hz) >> 1;
	hcnt = lcnt = DIV_ROUND_UP(period, tck_ns);

	reg.fields.hcnt = hcnt;
	reg.fields.lcnt = lcnt;
	i3c_register->pp_timing.value = reg.value;

	/* Configure extra termination timing */
	i3c_register->ext_termn_timing.fields.lcnt = DEFAULT_EXT_TERMN_LCNT;
}

static void i3c_aspeed_init_hw_feature(struct i3c_aspeed_data *data)
{
	uint32_t scu = DT_REG_ADDR(DT_NODELABEL(syscon));
	uint32_t rev_id = (sys_read32(scu + 0x4) & GENMASK(31, 16)) >> 16;

	/*
	 * if AST26xx-A3 or AST10x0-A1, the IBI status bitfield is correct.
	 * The others are not correct and need for workaround.
	 */
	if ((rev_id == 0x0503) || (rev_id == 0x8001)) {
		data->hw_feature.ibi_status_correct = 1;
	} else {
		data->hw_feature.ibi_status_correct = 0;
	}
}

static void i3c_aspeed_init_pid(const struct device *dev)
{
	struct i3c_aspeed_config *config = DEV_CFG(dev);
	struct i3c_register_s *i3c_register = config->base;
	union i3c_slave_pid_hi_s slave_pid_hi;
	union i3c_slave_pid_lo_s slave_pid_lo;
	union i3c_slave_char_s slave_char;
	uint32_t scu = DT_REG_ADDR(DT_NODELABEL(syscon));
	uint32_t rev_id = (sys_read32(scu + 0x4) & GENMASK(31, 16)) >> 16;

	slave_pid_hi.value = 0;
	slave_pid_hi.fields.mipi_mfg_id = MIPI_MFG_ASPEED;
	i3c_register->slave_pid_hi.value = slave_pid_hi.value;

	slave_pid_lo.value = 0;
	slave_pid_lo.fields.part_id = rev_id >> 16;
	slave_pid_lo.fields.inst_id = config->inst_id;
	i3c_register->slave_pid_lo.value = slave_pid_lo.value;

	slave_char.value = i3c_register->slave_char.value;
	slave_char.fields.bcr = 0x66;
	i3c_register->slave_char.value = slave_char.value;
}

static void i3c_aspeed_set_role(const struct device *dev, int secondary)
{
	struct i3c_aspeed_config *config = DEV_CFG(dev);
	struct i3c_register_s *i3c_register = config->base;
	union i3c_device_ctrl_extend_s device_ctrl_ext;

	device_ctrl_ext.value = i3c_register->device_ctrl_ext.value;
	if (secondary)
		device_ctrl_ext.fields.role = DEVICE_CTRL_EXT_ROLE_SLAVE;
	else
		device_ctrl_ext.fields.role = DEVICE_CTRL_EXT_ROLE_MASTER;

	i3c_register->device_ctrl_ext.value = device_ctrl_ext.value;
}

static void i3c_aspeed_init_queues(const struct device *dev)
{
	struct i3c_aspeed_config *config = DEV_CFG(dev);
	struct i3c_register_s *i3c_register = config->base;
	union i3c_queue_thld_ctrl_s queue_thld_ctrl;
	union i3c_data_buff_ctrl_s data_buff_ctrl;

	queue_thld_ctrl.value = 0;
	/*
	 * The size of the HW IBI queue is 256-byte.  Whereas the max IBI chunk size is 124-byte.
	 * So the max number of the IBI status = ceil(256 / 124) = 3
	 */
	queue_thld_ctrl.fields.ibi_data_thld = MAX_IBI_CHUNK_IN_BYTE >> 2;
	queue_thld_ctrl.fields.ibi_status_thld = 3;
	i3c_register->queue_thld_ctrl.value = queue_thld_ctrl.value;

	data_buff_ctrl.value = 0;
	data_buff_ctrl.fields.tx_thld = 1;
	data_buff_ctrl.fields.rx_thld = 1;
	data_buff_ctrl.fields.tx_start_thld = 1;
	data_buff_ctrl.fields.rx_start_thld = 1;
	i3c_register->data_buff_ctrl.value = data_buff_ctrl.value;
}

static void i3c_aspeed_enable(const struct device *dev)
{
	struct i3c_aspeed_config *config = DEV_CFG(dev);
	struct i3c_register_s *i3c_register = config->base;
	union i3c_device_ctrl_s reg;

	reg.value = i3c_register->device_ctrl.value;
	reg.fields.enable = 1;
	reg.fields.slave_ibi_payload_en = 1;
	if (config->secondary) {
		/*
		 * We don't support hot-join on master mode, so disable auto-mode-adaption for
		 * slave mode accordingly.  Since the only master device we may connect with is
		 * our ourself.
		 */
		reg.fields.slave_auto_mode_adapt = 1;
	}
	i3c_register->device_ctrl.value = reg.value;
}
}

static int i3c_aspeed_init(const struct device *dev)
{
	struct i3c_aspeed_config *config = DEV_CFG(dev);
	struct i3c_aspeed_data *data = DEV_DATA(dev);
	struct i3c_register_s *i3c_register = config->base;
	const struct device *reset_dev = device_get_binding(ASPEED_RST_CTRL_NAME);
	union i3c_reset_ctrl_s reset_ctrl;
	int ret;

	clock_control_on(config->clock_dev, config->clock_id);
	reset_control_deassert(reset_dev, config->reset_id);

	reset_ctrl.value = 0;
	reset_ctrl.fields.core_reset = 1;
	reset_ctrl.fields.tx_queue_reset = 1;
	reset_ctrl.fields.rx_queue_reset = 1;
	reset_ctrl.fields.ibi_queue_reset = 1;
	reset_ctrl.fields.cmd_queue_reset = 1;
	reset_ctrl.fields.resp_queue_reset = 1;
	i3c_register->reset_ctrl.value = reset_ctrl.value;

	ret = reg_read_poll_timeout(i3c_register, reset_ctrl, reset_ctrl, !reset_ctrl.value, 0, 10);
	if (ret) {
		return ret;
	}

	i3c_register->intr_status.value = GENMASK(31, 0);

	i3c_aspeed_init_hw_feature(data);
	i3c_aspeed_set_role(dev, config->secondary);
	i3c_aspeed_init_clock(dev);

	/* setup the dynamic address if playing the role as the main master */
	if (!config->secondary) {
		union i3c_device_addr_s reg;

		reg.value = 0;
		reg.fields.dynamic_addr = config->assigned_addr;
		reg.fields.dynamic_addr_valid = 1;
		i3c_register->device_addr.value = reg.value;
	}

	i3c_aspeed_init_queues(dev);
	i3c_aspeed_init_pid(dev);

	data->hw_dat.value = i3c_register->dev_addr_tbl_ptr.value;
	data->hw_dat_free_pos = GENMASK(data->hw_dat.fields.depth - 1, 0);

	/* Not support MR for now */
	i3c_register->mr_reject = GENMASK(31, 0);
	/* Disable SIR auto-reject */
	i3c_register->sir_reject = 0;

	i3c_aspeed_enable(dev);

	return 0;
}

#define I3C_ASPEED_INIT(n)                                                                         \
	static int i3c_aspeed_config_func_##n(const struct device *dev);                           \
	static const struct i3c_aspeed_config i3c_aspeed_config_##n = {                            \
		.base = (struct i3c_register_s *)DT_INST_REG_ADDR(n),                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, clk_id),                \
		.reset_id = (reset_control_subsys_t)DT_INST_RESETS_CELL(n, rst_id),                \
		.i2c_scl_hz = DT_INST_PROP_OR(n, i2c_scl_hz, 0),                                   \
		.i3c_scl_hz = DT_INST_PROP_OR(n, i3c_scl_hz, 0),                                   \
		.secondary = DT_INST_PROP_OR(n, secondary, 0),                                     \
		.assigned_addr = DT_INST_PROP_OR(n, assigned_addr, 0),                             \
		.inst_id = DT_INST_PROP_OR(n, instance_id, 0),                                     \
	};                                                                                         \
												   \
	static struct i3c_aspeed_data i3c_aspeed_data##n;                                          \
												   \
	DEVICE_DT_INST_DEFINE(n, &i3c_aspeed_config_func_##n, NULL, &i3c_aspeed_data##n,           \
			      &i3c_aspeed_config_##n, POST_KERNEL,                                 \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);                           \
												   \
	static int i3c_aspeed_config_func_##n(const struct device *dev)                            \
	{                                                                                          \
		i3c_aspeed_init(dev);                                                              \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i3c_aspeed_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
		return 0;                                                                          \
	}

DT_INST_FOREACH_STATUS_OKAY(I3C_ASPEED_INIT)
