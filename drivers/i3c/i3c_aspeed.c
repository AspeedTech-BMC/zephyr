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
		volatile uint32_t boradcast_addr_inc : 1; /* bit[0] */
		volatile uint32_t reserved0 : 6; /* bit[6:1] */
		volatile uint32_t i2c_slave_present : 1; /* bit[7] */
		volatile uint32_t hj_ack_ctrl : 1; /* bit[8] */
		volatile uint32_t slave_ibi_payload_en : 1; /* bit[9] */
		volatile uint32_t slave_pec_en : 1; /* bit[10] */
		volatile uint32_t reserved1 : 5; /* bit[15:11] */
		volatile uint32_t slave_mdb : 8; /* bit[23:16] */
		volatile uint32_t reserved2 : 4; /* bit[27:24] */
		volatile uint32_t dma_handshake_en : 1; /* bit[28] */
		volatile uint32_t abort : 1; /* bit[29] */
		volatile uint32_t resume : 1; /* bit[30] */
		volatile uint32_t enable : 1; /* bit[31] */
	} fields;
};

union i3c_device_addr_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t static_addr : 7; /* bit[6:0] */
		volatile uint32_t reserved0 : 8; /* bit[14:7] */
		volatile uint32_t static_addr_valid : 1; /* bit[15] */
		volatile uint32_t dynamic_addr : 7; /* bit[22:16] */
		volatile uint32_t reserved1 : 8; /* bit[30:23] */
		volatile uint32_t dynamic_addr_valid : 1; /* bit[31] */
	} fields;
};

union i3c_device_cmd_queue_port_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t static_addr : 7; /* bit[6:0] */
		volatile uint32_t reserved0 : 8; /* bit[14:7] */
		volatile uint32_t static_addr_valid : 1; /* bit[15] */
		volatile uint32_t dynamic_addr : 7; /* bit[22:16] */
		volatile uint32_t reserved1 : 8; /* bit[30:23] */
		volatile uint32_t dynamic_addr_valid : 1; /* bit[31] */
	} fields;
};

union i3c_device_resp_queue_port_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t data_length : 16;	/* bit[15:0] */
		volatile uint32_t ccct : 8;		/* bit[23:16] */
		volatile uint32_t tid : 4;		/* bit[27:24] */
		volatile uint32_t err_status : 4;	/* bit[31:28] */
	} fields;
};

union i3c_queue_thld_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t cmd_q_empty_thld : 8;	/* bit[7:0] */
		volatile uint32_t resp_q_thld : 8;	/* bit[15:8] */
		volatile uint32_t ibi_data_thld : 8;	/* bit[23:16] */
		volatile uint32_t ibi_status_thld : 8;	/* bit[31:24] */
	} fields;
};

union i3c_reset_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t core_reset : 1;	/* bit[0] */
		volatile uint32_t cmd_queue_reset : 1;	/* bit[1] */
		volatile uint32_t resp_queue_reset : 1;	/* bit[2] */
		volatile uint32_t tx_queue_reset : 1;	/* bit[3] */
		volatile uint32_t rx_queue_reset : 1;	/* bit[4] */
		volatile uint32_t ibi_queue_reset : 1;	/* bit[5] */
		volatile uint32_t reserved : 23;	/* bit[28:6] */
		volatile uint32_t bus_reset_type : 2;	/* bit[30:29] */
		volatile uint32_t bus_reset : 1;	/* bit[31] */
	} fields;
};

union i3c_slave_event_ctrl_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t sir_allowed : 1;	/* bit[0] */
		volatile uint32_t mr_allowed : 1;	/* bit[1] */
		volatile uint32_t reserved0 : 1;	/* bit[2] */
		volatile uint32_t hj_allowed : 1;	/* bit[3] */
		volatile uint32_t act_state : 2;	/* bit[5:4] */
		volatile uint32_t mrl_update : 1;	/* bit[6] */
		volatile uint32_t mwl_update : 1;	/* bit[7] */
		volatile uint32_t reserved1 : 24;	/* bit[31:8] */
	} fields;
};

union i3c_intr_s {
	volatile uint32_t value;
	struct {
		volatile uint32_t tx_thld : 1;		/* bit[0] */
		volatile uint32_t rx_thld : 1;		/* bit[1] */
		volatile uint32_t ibi_thld : 1;		/* bit[2] */
		volatile uint32_t cmd_q_ready : 1;	/* bit[3] */
		volatile uint32_t resp_q_ready : 1;	/* bit[4] */
		volatile uint32_t xfr_abort : 1;	/* bit[5] */
		volatile uint32_t ccc_update : 1;	/* bit[6] */
		volatile uint32_t reserved0 : 1;	/* bit[7] */
		volatile uint32_t dyn_addr_assign : 1;	/* bit[8] */
		volatile uint32_t xfr_error : 1;	/* bit[9] */
		volatile uint32_t defslv : 1;		/* bit[10] */
		volatile uint32_t read_q_recv : 1;	/* bit[11] */
		volatile uint32_t ibi_update : 1;	/* bit[12] */
		volatile uint32_t bus_owner_update : 1;	/* bit[13] */
		volatile uint32_t reserved1 : 1;	/* bit[14] */
		volatile uint32_t bus_reset_done : 1;	/* bit[15] */
		volatile uint32_t reserved2 : 16;	/* bit[31:16] */
	} fields;
};

struct i3c_register_s {
	union i3c_device_ctrl_s device_ctrl;			/* 0x0 */
	union i3c_device_addr_s device_addr;			/* 0x4 */
	uint32_t hw_capability;					/* 0x8 */
	union i3c_device_cmd_queue_port_s cmd_queue_port;	/* 0xc */
	union i3c_device_resp_queue_port_s resp_queue_port;	/* 0x10 */
	uint32_t rx_tx_data_port;				/* 0x14 */
	uint32_t ibi_queue_data;				/* 0x18 */
	union i3c_queue_thld_ctrl_s queue_thld_ctrl;		/* 0x1c */
	uint32_t reserved0[5];					/* 0x20 ~ 0x30 */
	union i3c_reset_ctrl_s reset_ctrl;			/* 0x34 */
	union i3c_slave_event_ctrl_s slave_event_ctrl;		/* 0x38 */
	union i3c_intr_s intr_status;				/* 0x3c */
	union i3c_intr_s intr_status_en;			/* 0x40 */
	union i3c_intr_s intr_signal_en;			/* 0x44 */
	union i3c_intr_s intr_force_en;				/* 0x48 */
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
	uint8_t self_addr;
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

static void i3c_aspeed_master_config_clock(const struct device *dev)
{
	struct i3c_aspeed_config *config = DEV_CFG(dev);
	int i3cclk;

	clock_control_get_rate(config->clock_dev, config->clock_id, &i3cclk);
	LOG_DBG("i3cclk %d hz\n", i3cclk);
}

static int i3c_aspeed_init(const struct device *dev)
{
	struct i3c_aspeed_config *config = DEV_CFG(dev);
	const struct device *reset_dev = device_get_binding(ASPEED_RST_CTRL_NAME);
	union i3c_reset_ctrl_s reset_ctrl;

	clock_control_on(config->clock_dev, config->clock_id);
	reset_control_deassert(reset_dev, config->reset_id);

	reset_ctrl.value = 0;
	reset_ctrl.fields.core_reset = 1;
	reset_ctrl.fields.tx_queue_reset = 1;
	reset_ctrl.fields.rx_queue_reset = 1;
	reset_ctrl.fields.ibi_queue_reset = 1;
	reset_ctrl.fields.cmd_queue_reset = 1;
	reset_ctrl.fields.resp_queue_reset = 1;
	config->base->reset_ctrl.value = reset_ctrl.value;
	while (config->base->reset_ctrl.value)
		;

	config->base->intr_status.value = GENMASK(31, 0);

	i3c_aspeed_master_config_clock(dev);

	LOG_INF("i3c base %08x\n", (uint32_t)config->base);

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
