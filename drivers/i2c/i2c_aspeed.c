/*
 * Copyright (c) ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_i2c
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/clock_control.h>
#include "soc.h"

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

#include <zephyr/sys/util.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
LOG_MODULE_REGISTER(i2c_aspeed);

#include "i2c-priv.h"

#define K_CACHE_INVD	BIT(1)

#define I2C_SLAVE_COUNT			3
#define I2C_SLAVE_BUF_SIZE		256

/* i2c global */
#define ASPEED_I2CG_CLK_DIV		0x10
#define ASPEED_I2CG_CONTROL		0x0C
#define ASPEED_I2CG_NEW_REG_BIT	BIT(2)
#define ASPEED_I2CG_NEW_CLK_BIT	BIT(1)
#define ASPEED_I2C_NEW_MODE	(ASPEED_I2CG_NEW_CLK_BIT |\
			ASPEED_I2CG_NEW_REG_BIT)

/* i2c control reg */
/* 0x00 : I2CC Master/Slave Function Control Register  */
#define AST_I2CC_FUN_CTRL		0x00

#define AST_I2CC_SLAVE_ADDR_RX_EN	BIT(20)
#define AST_I2CC_MASTER_RETRY_MASK	(0x3 << 18)
#define AST_I2CC_MASTER_RETRY(x)	(((x) & 0x3) << 18)
#define AST_I2CC_BUS_AUTO_RELEASE	BIT(17)
#define AST_I2CC_M_SDA_LOCK_EN		BIT(16)
#define AST_I2CC_MULTI_MASTER_DIS	BIT(15)
#define AST_I2CC_M_SCL_DRIVE_EN		BIT(14)
#define AST_I2CC_MSB_STS		BIT(9)
#define AST_I2CC_SDA_DRIVE_1T_EN	BIT(8)
#define AST_I2CC_M_SDA_DRIVE_1T_EN	BIT(7)
#define AST_I2CC_M_HIGH_SPEED_EN	BIT(6)
/* reserver 5 : 2 */
#define AST_I2CC_SLAVE_EN		BIT(1)
#define AST_I2CC_MASTER_EN		BIT(0)

/* 0x04 : I2CC Master/Slave Clock and AC Timing Control Register #1 */
#define AST_I2CC_AC_TIMING		0x04
#define AST_I2CC_tTIMEOUT(x)		(((x) & 0x1f) << 24)	/* 0~7 */
#define AST_I2CC_tCKHIGHMin(x)		(((x) & 0xf) << 20)	/* 0~f */
#define AST_I2CC_tCKHIGH(x)		(((x) & 0xf) << 16)	/* 0~7 */
#define AST_I2CC_tCKLOW(x)		(((x) & 0xf) << 12)	/* 0~7 */
#define AST_I2CC_tHDDAT(x)		(((x) & 0x3) << 10)	/* 0~3 */
#define AST_I2CC_toutBaseCLK(x)		(((x) & 0x3) << 8)	/* 0~3 */
#define AST_I2CC_tBaseCLK(x)		((x) & 0xf)		/* 0~0xf */

/* 0x08 : I2CC Master/Slave Transmit/Receive Byte Buffer Register */
#define AST_I2CC_STS_AND_BUFF		0x08
#define AST_I2CC_TX_DIR_MASK		(0x7 << 29)
#define AST_I2CC_SDA_OE			BIT(28)
#define AST_I2CC_SDA_O			BIT(27)
#define AST_I2CC_SCL_OE			BIT(26)
#define AST_I2CC_SCL_O			BIT(25)

/* Tx State Machine */
#define AST_I2CM_MTXACK			0xf
#define AST_I2CM_MRXD			0xe
#define AST_I2CM_MRXACK			0xd
#define AST_I2CM_MTXD			0xc
#define AST_I2CM_MSTOP			0xb
#define AST_I2CM_MSTARTR		0xa
#define AST_I2CM_MSTART			0x9
#define AST_I2CM_MACTIVE		0x8
#define AST_I2CM_SRXACK			0x7
#define AST_I2CM_STXD			0x6
#define AST_I2CM_STXACK			0x5
#define AST_I2CM_SRXD			0x4
#define AST_I2CM_RECOVER		0x3
#define AST_I2CM_SWAIT			0x1
#define AST_I2CM_IDLE			0x0

#define AST_I2CC_SCL_LINE_STS		BIT(18)
#define AST_I2CC_SDA_LINE_STS		BIT(17)
#define AST_I2CC_BUS_BUSY_STS		BIT(16)

#define AST_I2CC_GET_RX_BUFF(x)		(((x) >> 8) & 0xff)

/* 0x0C : I2CC Master/Slave Pool Buffer Control Register  */
#define AST_I2CC_BUFF_CTRL		0x0C
#define AST_I2CC_GET_RX_BUF_LEN(x)	(((x) >> 24) & 0x3f)
#define AST_I2CC_SET_RX_BUF_LEN(x)	((((x) - 1) & 0x1f) << 16)
#define AST_I2CC_SET_TX_BUF_LEN(x)	((((x) - 1) & 0x1f) << 8)
#define AST_I2CC_GET_TX_BUF_LEN(x)	((((x) >> 8) & 0x1f) + 1)

/* 0x10 : I2CM Master Interrupt Control Register */
#define AST_I2CM_IER			0x10
/* 0x14 : I2CM Master Interrupt Status Register   : WC */
#define AST_I2CM_ISR			0x14

#define AST_I2CM_SW_ISR_MASK		0xfff80000
#define AST_I2CM_PKT_TIMEOUT		BIT(18)
#define AST_I2CM_PKT_ERROR		BIT(17)
#define AST_I2CM_PKT_DONE		BIT(16)
#define AST_I2CM_BUS_RECOVER_FAIL	BIT(15)
#define AST_I2CM_SDA_DL_TO		BIT(14)
#define AST_I2CM_BUS_RECOVER		BIT(13)
#define AST_I2CM_SMBUS_ALT		BIT(12)

#define AST2700_I2CM_ABNORMAL	BIT(8)
#define AST_I2CM_SCL_LOW_TO		BIT(6)
#define AST_I2CM_ABNORMAL		BIT(5)
#define AST_I2CM_NORMAL_STOP		BIT(4)
#define AST_I2CM_ARBIT_LOSS		BIT(3)
#define AST_I2CM_RX_DONE		BIT(2)
#define AST_I2CM_TX_NAK			BIT(1)
#define AST_I2CM_TX_ACK			BIT(0)

/* 0x18 : I2CM Master Command/Status Register   */
#define AST_I2CM_CMD_STS		0x18

#define AST_I2CM_PKT_EN			BIT(16)
#define AST_I2CM_SDA_OE_OUT_DIR		BIT(15)
#define AST_I2CM_SDA_O_OUT_DIR		BIT(14)
#define AST_I2CM_SCL_OE_OUT_DIR		BIT(13)
#define AST_I2CM_SCL_O_OUT_DIR		BIT(12)
#define AST_I2CM_RECOVER_CMD_EN		BIT(11)

#define AST_I2CM_RX_DMA_EN		BIT(9)
#define AST_I2CM_TX_DMA_EN		BIT(8)

/* Command Bit */
#define AST_I2CM_RX_BUFF_EN		BIT(7)
#define AST_I2CM_TX_BUFF_EN		BIT(6)
#define AST_I2CM_STOP_CMD		BIT(5)
#define AST_I2CM_RX_CMD_LAST		BIT(4)
#define AST_I2CM_RX_CMD			BIT(3)
#define AST_I2CM_TX_CMD			BIT(1)
#define AST_I2CM_START_CMD		BIT(0)

#define AST_I2CM_PKT_ADDR(x)		(((x) & 0x7f) << 24)

/* 0x1C : I2CM Master DMA Transfer Length Register   */
#define AST_I2CM_DMA_LEN		0x1C
#define AST_I2CM_SET_RX_DMA_LEN(x)	((((x) & 0xfff) << 16) | BIT(31))	/* 1 ~ 4096 */
#define AST_I2CM_SET_TX_DMA_LEN(x)	(((x) & 0xfff) | BIT(15))		/* 1 ~ 4096 */

/* 0x20 : I2CS Slave Interrupt Control Register */
#define AST_I2CS_IER			0x20
/* 0x24 : I2CS Slave Interrupt Status Register */
#define AST_I2CS_ISR			0x24

#define AST_I2CS_ADDR_INDICATE_MASK      (3 << 30)
#define AST_I2CS_SLAVE_PENDING		BIT(29)
#define AST_I2CS_SADDR_PENDING		BIT(28)

#define AST_I2CS_WAIT_TX_DMA		BIT(25)
#define AST_I2CS_WAIT_RX_DMA		BIT(24)

#define AST_I2CS_ADDR3_NAK		BIT(22)
#define AST_I2CS_ADDR2_NAK		BIT(21)
#define AST_I2CS_ADDR1_NAK		BIT(20)
#define AST_I2CS_ADDR_NAK_MASK (3 << 20)
#define AST_I2CS_ADDR_MASK		(3 << 18)
#define AST_I2CS_GET_SLAVE(x)	(((x) >> 30) & 0x3)
#define AST_I2CS_PKT_ERROR		BIT(17)
#define AST_I2CS_PKT_DONE		BIT(16)
#define AST_I2CS_INACTIVE_TO		BIT(15)
#define AST_I2CS_SLAVE_MATCH		BIT(7)
#define AST_I2CS_ABNOR_STOP		BIT(5)
#define AST_I2CS_STOP			BIT(4)
#define AST_I2CS_RX_DONE_NAK		BIT(3)
#define AST_I2CS_RX_DONE		BIT(2)
#define AST_I2CS_TX_NAK			BIT(1)
#define AST_I2CS_TX_ACK			BIT(0)

/* 0x28 : I2CS Slave CMD/Status Register   */
#define AST_I2CS_CMD_STS		0x28
#define AST_I2CS_ACTIVE_ALL		(0x3 << 17)
#define AST_I2CS_PKT_MODE_EN		BIT(16)
#define AST_I2CS_AUTO_NAK_NOADDR	BIT(15)
#define AST_I2CS_AUTO_NAK_EN		BIT(14)
#define AST_I2CS_RX_DMA_EN			BIT(9)
#define AST_I2CS_TX_DMA_EN			BIT(8)

/* new for i2c snoop */
#define AST_I2CS_SNOOP_LOOP		BIT(12)
#define AST_I2CS_SNOOP_EN		BIT(11)

#define AST_I2CS_ALT_EN			BIT(10)
#define AST_I2CS_RX_DMA_EN		BIT(9)
#define AST_I2CS_TX_DMA_EN		BIT(8)
#define AST_I2CS_RX_BUFF_EN		BIT(7)
#define AST_I2CS_TX_BUFF_EN		BIT(6)
#define AST_I2CS_RX_CMD_LAST		BIT(4)

#define AST_I2CS_TX_CMD			BIT(2)

#define AST_I2CS_DMA_LEN		0x2C
#define AST_I2CS_SET_RX_DMA_LEN(x)	(((((x) - 1) & 0xfff) << 16) | BIT(31))
#define AST_I2CS_RX_DMA_LEN_MASK	(0xfff << 16)

#define AST_I2CS_SET_TX_DMA_LEN(x)	((((x) - 1) & 0xfff) | BIT(15))
#define AST_I2CS_TX_DMA_LEN_MASK	0xfff

/* I2CM Master DMA Tx Buffer Register */
#define AST_I2CM_TX_DMA			0x30
/* I2CM Master DMA Rx Buffer Register */
#define AST_I2CM_RX_DMA			0x34
/* I2CS Slave DMA Tx Buffer Register */
#define AST_I2CS_TX_DMA			0x38
/* I2CS Slave DMA Rx Buffer Register */
#define AST_I2CS_RX_DMA			0x3C

/* I2CM Master DMA Tx Buffer High Register */
#define AST_I2CM_TX_DMA_H		0x60
/* I2CM Master DMA Rx Buffer High Register */
#define AST_I2CM_RX_DMA_H		0x64
/* I2CS Slave DMA Tx Buffer High Register */
#define AST_I2CS_TX_DMA_H		0x68
/* I2CS Slave DMA Rx Buffer High Register */
#define AST_I2CS_RX_DMA_H		0x6C

/* 0x40 : Slave Device Address Register */
#define AST_I2CS_ADDR_CTRL		0x40

#define AST_I2CS_ADDR3_MBX_TYPE(x)	((x) << 28)
#define AST_I2CS_ADDR2_MBX_TYPE(x)	((x) << 26)
#define AST_I2CS_ADDR1_MBX_TYPE(x)	((x) << 24)
#define AST_I2CS_ADDR3_ENABLE		BIT(23)
#define AST_I2CS_ADDR3(x)		(((x) & 0x7f) << 16)
#define AST_I2CS_ADDR2_ENABLE		BIT(15)
#define AST_I2CS_ADDR2(x)		(((x) & 0x7f) << 8)
#define AST_I2CS_ADDR1_ENABLE		BIT(7)
#define AST_I2CS_ADDR1(x)		((x) & 0x7f)

#define AST_I2CS_ADDR3_MASK		(0x7f << 16)
#define AST_I2CS_ADDR2_MASK		(0x7f << 8)
#define AST_I2CS_ADDR1_MASK		0x7f

#define AST_I2CM_DMA_LEN_STS		0x48
#define AST_I2CS_DMA_LEN_STS		0x4C

#define AST_I2C_GET_TX_DMA_LEN(x)	((x) & 0x1fff)
#define AST_I2C_GET_RX_DMA_LEN(x)	(((x) >> 16) & 0x1fff)

/* 0x74 : Slave Device Address Register */
#define MSIC_CONFIG_ACTIMING1		0x74
#define MSIC_I2C_SET_TIMEOUT(s, m)		(((s) << 16) | (m))

/* 0x8c : Slave sirq log */
#define AST2700_I2CC_SIRQ_LOG		0x8c
#define SLAVE_ADDR_SHIFT		8
#define SLAVE_ADDR_MASK		(0xff << 8)
#define SADDR_NACK				BIT(5)
#define SLAVE_PKT_DONE			BIT(4)
#define SADDR_HIT				BIT(3)
#define SRX_DONE				BIT(2)
#define STX_DONE				BIT(1)
#define SLAVE_STOP				BIT(0)

/* 0x94 : Version control */
#define AST2700_I2CC_VER_CTRL		0x94
#define USE_DMA_MODE				BIT(2)

/* i2c timeout counter: use base clk4 1Mhz
 * 1/(1000/4096) = 4.096ms * 8 = 32.768ms
 */
#define I2C_TIMEOUT_CLK			0x2
#define AST2700_I2C_TIMEOUT_CLK	0x3
#define I2C_TIMEOUT_COUNT		0x8 /* i2c timeout setting (wait about 35ms) */

/***************************************************************************/
/* Use platform_data instead of module parameters */
/* Fast Mode = 400 kHz, Standard = 100 kHz */
/* static int clock = 100;  Default: 100 kHz */
/***************************************************************************/
#define AST_LOCKUP_DETECTED		BIT(15)
#define AST_I2C_LOW_TIMEOUT		0x07
/***************************************************************************/
#define ASPEED_I2C_DMA_SIZE		4096
/***************************************************************************/
#define SLAVE_TRIGGER_CMD		(AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN)

#define DEV_CFG(dev) \
	((struct i2c_aspeed_config *)(dev)->config)
#define DEV_DATA(dev) \
	((struct i2c_aspeed_data *)(dev)->data)
#define DEV_BASE(dev) \
	((DEV_CFG(dev))->base)

enum i2c_xfer_mode {
	BYTE_MODE,
	BUFF_MODE,
	DMA_MODE,
};

enum i2c_version {
	AST2600,
	AST2700,
};

struct i2c_aspeed_config {
	uint32_t global_reg;
	uintptr_t base;
	/* Buffer mode */
	uintptr_t buf_base;
	size_t buf_size;
	uint32_t bitrate;
	uint8_t multi_master;
	uint8_t smbus_timeout;
	uint8_t manual_scl_high;
	uint8_t manual_scl_low;
	uint8_t manual_sda_hold;
	int smbus_alert;
	const struct device *clock_dev;
	const clock_control_subsys_t clk_id;
	const struct pinctrl_dev_config *pcfg;
	uint32_t ac_timing;
	void (*irq_config_func)(const struct device *dev);
	uint32_t clk_src;
	enum i2c_xfer_mode mode;
};

struct i2c_aspeed_data {
	struct k_sem sync_sem;
	struct k_mutex trans_mutex;

	int alert_enable;

	uint32_t bus_recover;

	struct i2c_msg          *msgs;  /* cur xfer msgs */
	uint16_t addr;
	int buf_index;                  /*buffer mode idx */
	int msgs_index;                 /* cur xfer msgs index */
	int msgs_count;                 /* total msgs */

	int master_xfer_cnt;            /* total xfer count */

	uint8_t slave_attached;	/* slave attached count */

	int xfer_complete;

	uint32_t bus_frequency;
	/* master configuration */
	int cmd_err;
	uint16_t flags;
	uint8_t         *buf;
	int len;                        /* master xfer count */

	/* slave configuration */
	uint8_t slave_addr;
	uint32_t slave_xfer_len;
	uint32_t slave_xfer_cnt;

	/* byte mode check re-start */
	uint8_t slave_addr_last;

	/* version */
	enum i2c_version version;

	uint8_t smbus_timeout;

	/* function pointers */
	uint32_t (*setup_tx)(uint32_t cmd, const struct device *dev);
	uint32_t (*setup_rx)(uint32_t cmd, const struct device *dev);
	uint32_t (*is_irq_err)(uint32_t cmd);

#ifdef CONFIG_I2C_TARGET
	unsigned char slave_dma_buf[I2C_SLAVE_BUF_SIZE];
	struct i2c_target_config *slave_cfg[I2C_SLAVE_COUNT];
	struct i2c_target_config *slave_get_cfg;
	const struct i2c_target_callbacks *slave_cb;
#endif
};

struct ast_i2c_timing_table {
	uint32_t divisor;
	uint32_t timing;
};

static uint32_t i2c_aspeed_ast2700_select_clock(const struct device *dev)
{
	const struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	unsigned long base_clk;
	int baseclk_idx = 0;
	int divisor = 0;
	uint32_t scl_low;
	uint32_t scl_high;
	uint32_t ac_timing;

	for (int i = 0; i < 0x100; i++) {
		base_clk = (config->clk_src) / (i + 1);
		if ((base_clk / data->bus_frequency) <= 32) {
			baseclk_idx = i;
			divisor = (base_clk / (unsigned long)(data->bus_frequency));
			if ((base_clk / divisor) > (unsigned long)data->bus_frequency)
				divisor++;
			break;
		}
	}

	baseclk_idx = MIN(baseclk_idx, 0xff);
	divisor = MIN(divisor, 32);
	scl_low = MIN(divisor * 9 / 16 - 1, 15);
	scl_high = (divisor - scl_low - 2) & 0x1f;
	ac_timing = (scl_high - 1) << 20 | scl_high << 16 | scl_low << 12 | baseclk_idx;

	/* Set time out timer */
	if (config->smbus_timeout) {
		data->smbus_timeout = MIN(config->smbus_timeout, 255);
		sys_write32(MSIC_I2C_SET_TIMEOUT(data->smbus_timeout, data->smbus_timeout),
		i2c_base + AST_I2CC_FUN_CTRL);

		ac_timing |= AST_I2CC_toutBaseCLK(AST2700_I2C_TIMEOUT_CLK);
	}

	LOG_DBG("ac_timing %x", ac_timing);

	return ac_timing;
}

static uint32_t i2c_aspeed_select_clock(const struct device *dev)
{
	const struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t ac_timing;
	int div = 0;
	int divider_ratio = 0;
	uint32_t clk_div_reg;
	int inc = 0;
	unsigned long base_clk;
	unsigned long base_clk1;
	unsigned long base_clk2;
	unsigned long base_clk3;
	unsigned long base_clk4;
	uint32_t scl_low, scl_high;

	clk_div_reg = sys_read32(config->global_reg + ASPEED_I2CG_CLK_DIV);

	base_clk = config->clk_src;
	base_clk1 = (config->clk_src * 10) /
	((((clk_div_reg & 0xff) + 2) * 10) / 2);
	LOG_DBG("base_clk1 is %lx", base_clk1);
	base_clk2 = (config->clk_src * 10) /
	(((((clk_div_reg >> 8) & 0xff) + 2) * 10) / 2);
	LOG_DBG("base_clk2 is %lx", base_clk2);
	base_clk3 = (config->clk_src * 10) /
	(((((clk_div_reg >> 16) & 0xff) + 2) * 10) / 2);
	LOG_DBG("base_clk3 is %lx", base_clk3);
	base_clk4 = (config->clk_src * 10) /
	(((((clk_div_reg >> 24) & 0xff) + 2) * 10) / 2);
	LOG_DBG("base_clk4 is %lx", base_clk4);

	/* Rounding by ourself */
	if ((config->clk_src / data->bus_frequency) <= 32) {
		div = 0;
		divider_ratio = (base_clk / (unsigned long)(data->bus_frequency));
		if ((base_clk / divider_ratio) > (unsigned long)data->bus_frequency)
			divider_ratio++;
	} else if ((base_clk1 / data->bus_frequency) <= 32) {
		div = 1;
		divider_ratio = (base_clk1 / (unsigned long)(data->bus_frequency));
		if ((base_clk1 / divider_ratio) > (unsigned long)data->bus_frequency)
			divider_ratio++;
	} else if ((base_clk2 / data->bus_frequency) <= 32) {
		div = 2;
		divider_ratio = (base_clk2 / (unsigned long)(data->bus_frequency));
		if ((base_clk2 / divider_ratio) > (unsigned long)data->bus_frequency)
			divider_ratio++;
	} else if ((base_clk3 / data->bus_frequency) <= 32) {
		div = 3;
		divider_ratio = (base_clk3 / (unsigned long)(data->bus_frequency));
		if ((base_clk3 / divider_ratio) > (unsigned long)data->bus_frequency)
			divider_ratio++;
	} else {
		div = 4;
		divider_ratio = (base_clk4 / (unsigned long)(data->bus_frequency));
		inc = 0;
		while ((divider_ratio + inc) > 32) {
			inc |= divider_ratio & 0x1;
			divider_ratio >>= 1;
			div++;
		}
		divider_ratio += inc;
		if ((base_clk4 / divider_ratio) > (unsigned long)data->bus_frequency)
			divider_ratio++;
	}

	LOG_DBG("div %d", div);
	LOG_DBG("divider_ratio %x", divider_ratio);

	divider_ratio = MIN(divider_ratio, 32);
	LOG_DBG("divider_ratio min %x", divider_ratio);
	div &= 0xf;

	/* Set menual scl low length */
	if (config->manual_scl_low && config->manual_scl_high) {
		scl_low = config->manual_scl_low;
		scl_high = config->manual_scl_high;
		LOG_DBG("maual scl_low min %x", scl_low);
		LOG_DBG("maual scl_high min %x", scl_high);
	} else if (config->manual_scl_low || config->manual_scl_high) {
		if (config->manual_scl_low) {
			scl_low = config->manual_scl_low;
			LOG_DBG("maual scl_low min %x", scl_low);
			scl_high = (divider_ratio - scl_low - 2) & 0xf;
		} else {
			scl_high = config->manual_scl_high;
			LOG_DBG("maual scl_high min %x", scl_high);
			scl_low = (divider_ratio - scl_high - 2) & 0xf;
		}
	} else {
		scl_low = ((divider_ratio * 9) / 16) - 1;
		LOG_DBG("default scl_low min%x", scl_low);
		scl_high = (divider_ratio - scl_low - 2) & 0xf;
		LOG_DBG("default scl_high min%x", scl_low);
	}

	scl_low = MIN(scl_low, 0xf);
	scl_high = MIN(scl_high, 0xf);
	LOG_DBG("scl_low min %x", scl_low);
	LOG_DBG("scl_high min %x", scl_high);

	/*Divisor : Base Clock : tCKHighMin : tCK High : tCK Low*/
	ac_timing = ((scl_high - 1) << 20) | (scl_high << 16) | (scl_low << 12) | (div);

	/* Set time out timer */
	if (config->smbus_timeout) {
		ac_timing |= AST_I2CC_toutBaseCLK(I2C_TIMEOUT_CLK);
		ac_timing |= AST_I2CC_tTIMEOUT(I2C_TIMEOUT_COUNT);
		LOG_DBG("smbus_timeout enable");
	}

	/* Manual set the sda hold time */
	if (config->manual_sda_hold) {
		LOG_DBG("manual_sda_hold %x", config->manual_sda_hold);
		if (config->manual_sda_hold < 4)
			ac_timing |= AST_I2CC_tHDDAT(config->manual_sda_hold);
		else
			LOG_DBG("invalid sda hold setting %x", config->manual_sda_hold);
	}

	LOG_DBG("ac_timing %x", ac_timing);

	return ac_timing;
}

/*Default maximum time we allow for an I2C transfer (unit:ms)*/
#define I2C_TRANS_TIMEOUT K_MSEC(100)
#define I2C_ENTRY_TIMEOUT K_MSEC(200)

static int i2c_wait_completion(const struct device *dev)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);

	if (k_sem_take(&data->sync_sem, I2C_TRANS_TIMEOUT) == 0) {
		return 0;
	} else {
		return -ETIMEDOUT;
	}
}

static uint8_t
aspeed_i2c_recover_bus(const struct device *dev)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);

	uint32_t ctrl, state;
	int r;
	int ret = 0;

	ctrl = sys_read32(i2c_base + AST_I2CC_FUN_CTRL);

	sys_write32(ctrl & ~(AST_I2CC_MASTER_EN | AST_I2CC_SLAVE_EN),
			i2c_base + AST_I2CC_FUN_CTRL);

	sys_write32(sys_read32(i2c_base + AST_I2CC_FUN_CTRL) | AST_I2CC_MASTER_EN,
			i2c_base + AST_I2CC_FUN_CTRL);

	/*	Let's retry 10 times	*/
	k_sem_reset(&data->sync_sem);
	data->bus_recover = 1;
	data->cmd_err = 0;

	/*	Check 0x14's SDA and SCL status	*/
	state = sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF);
	if (!(state & AST_I2CC_SDA_LINE_STS) && (state & AST_I2CC_SCL_LINE_STS)) {
		sys_write32(AST_I2CM_RECOVER_CMD_EN, i2c_base + AST_I2CM_CMD_STS);
		r = i2c_wait_completion(dev);
		if (r == 0) {
			LOG_DBG("recovery timed out\n");
			ret = -ETIMEDOUT;
		} else {
			if (data->cmd_err) {
				LOG_DBG("recovery error\n");
				ret = -EPROTO;
			}
		}
	} else {
		LOG_DBG("can't recovery this situation\n");
		ret = -EPROTO;
	}
	LOG_DBG("Recovery done [%x]\n", sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));

	return ret;
}

static int i2c_aspeed_configure(const struct device *dev,
				uint32_t dev_config_raw)
{
	const struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	uint32_t fun_ctrl = AST_I2CC_BUS_AUTO_RELEASE;

	if (I2C_ADDR_10_BITS & dev_config_raw) {
		return -EINVAL;
	}

	if (I2C_MODE_CONTROLLER & dev_config_raw) {
		fun_ctrl |= AST_I2CC_MASTER_EN;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		data->bus_frequency = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		data->bus_frequency = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		data->bus_frequency = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	/*I2C Reset*/
	sys_write32(0, i2c_base + AST_I2CC_FUN_CTRL);

	if (!config->multi_master) {
		fun_ctrl |= AST_I2CC_MULTI_MASTER_DIS;
	}

	/*Enable Master Mode*/
	sys_write32(fun_ctrl, i2c_base + AST_I2CC_FUN_CTRL);

	/*Set AC Timing*/
	if (data->version == AST2700)
		sys_write32(i2c_aspeed_ast2700_select_clock(dev), i2c_base + AST_I2CC_AC_TIMING);
	else
		sys_write32(i2c_aspeed_select_clock(dev), i2c_base + AST_I2CC_AC_TIMING);

	/*Clear Interrupt*/
	sys_write32(0xfffffff, i2c_base + AST_I2CM_ISR);

	/*Set interrupt generation of I2C master controller*/
	if (config->smbus_alert) {
		sys_write32(AST_I2CM_PKT_DONE | AST_I2CM_BUS_RECOVER |
			    AST_I2CM_SMBUS_ALT,
			    i2c_base + AST_I2CM_IER);
	} else {
		sys_write32(AST_I2CM_PKT_DONE | AST_I2CM_BUS_RECOVER,
			    i2c_base + AST_I2CM_IER);
	}

#ifdef CONFIG_I2C_TARGET
	if (config->mode == DMA_MODE) {
		memset(data->slave_dma_buf, 0, I2C_SLAVE_BUF_SIZE);
	}

	sys_write32(0xfffffff, i2c_base + AST_I2CS_ISR);

	if (config->mode == BYTE_MODE) {
		sys_write32(0xffff, i2c_base + AST_I2CS_IER);
	} else {
		/*Set interrupt generation of I2C slave controller*/
		sys_write32((AST_I2CS_PKT_DONE | AST_I2CS_INACTIVE_TO), i2c_base + AST_I2CS_IER);
	}
#endif
	return 0;
}

static int i2c_aspeed_get_configure(const struct device *dev,
				uint32_t *dev_config_raw)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);

	/* NULL pointer */
	if (!dev_config_raw) {
		return -EINVAL;
	}

	switch (data->bus_frequency) {
	case KHZ(100):
		*dev_config_raw = I2C_SPEED_SET(I2C_SPEED_STANDARD);
		break;
	case KHZ(400):
		*dev_config_raw = I2C_SPEED_SET(I2C_SPEED_FAST);
		break;
	case MHZ(1):
		*dev_config_raw = I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* function pointers */
static uint32_t ast2600_i2c_setup_dma_tx(uint32_t cmd, const struct device *dev)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	struct i2c_msg *msg = &data->msgs[data->msgs_index];
	int xfer_len = msg->len - data->master_xfer_cnt;
	uint64_t DMA_Addr = TO_PHY_ADDR((uintptr_t)msg->buf);
	uint32_t DMA_Addr_L = (uint32_t)(DMA_Addr & 0xFFFFFFFF);
	uint32_t DMA_Addr_H = (uint32_t)(((DMA_Addr >> 32) & 0xFFFFFFFF));

	cmd |= AST_I2CM_PKT_EN;

	/*dma mode*/
	if (msg->len > ASPEED_I2C_DMA_SIZE) {
		xfer_len = ASPEED_I2C_DMA_SIZE;
	} else {
		if (data->msgs_index + 1 == data->msgs_count) {
			LOG_DBG("with P\n");
			cmd |= AST_I2CM_STOP_CMD;
		}
		xfer_len = msg->len;
	}

	if (cmd & AST_I2CM_START_CMD)
		cmd |= AST_I2CM_PKT_ADDR(data->addr);

	if (xfer_len) {
		cmd |= AST_I2CM_TX_DMA_EN | AST_I2CM_TX_CMD;
		sys_write32(AST_I2CM_SET_TX_DMA_LEN(xfer_len - 1),
		i2c_base + AST_I2CM_DMA_LEN);
		sys_write32(DMA_Addr_L, i2c_base + AST_I2CM_TX_DMA);
		sys_write32(DMA_Addr_H, i2c_base + AST_I2CM_TX_DMA_H);
	}

	LOG_DBG("len %d , DMA tx_cmd %x\n", xfer_len, cmd);
	sys_write32(cmd, i2c_base + AST_I2CM_CMD_STS);

	return 0;
}

static uint32_t ast2600_i2c_setup_dma_rx(uint32_t cmd, const struct device *dev)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	struct i2c_msg *msg = &data->msgs[data->msgs_index];
	int xfer_len = msg->len - data->master_xfer_cnt;
	uint64_t DMA_Addr = TO_PHY_ADDR((uintptr_t)msg->buf);
	uint32_t DMA_Addr_L = (uint32_t)(DMA_Addr & 0xFFFFFFFF);
	uint32_t DMA_Addr_H = (uint32_t)(((DMA_Addr >> 32) & 0xFFFFFFFF));

	cmd |= AST_I2CM_PKT_EN;

	/*dma mode*/
	if (msg->len > ASPEED_I2C_DMA_SIZE) {
		xfer_len = ASPEED_I2C_DMA_SIZE;
	} else {
		xfer_len = msg->len;
		if (data->msgs_index + 1 == data->msgs_count) {
			LOG_DBG("last stop\n");
			cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
		}
	}

	if (cmd & AST_I2CM_START_CMD)
		cmd |= AST_I2CM_PKT_ADDR(data->addr);

	if (xfer_len) {
		cmd |= AST_I2CM_RX_DMA_EN | AST_I2CM_RX_CMD;
		sys_write32(AST_I2CM_SET_RX_DMA_LEN(xfer_len - 1),
		i2c_base + AST_I2CM_DMA_LEN);
		sys_write32(DMA_Addr_L, i2c_base + AST_I2CM_RX_DMA);
		sys_write32(DMA_Addr_H, i2c_base + AST_I2CM_RX_DMA_H);
	}

	LOG_DBG("len %d , DMA rx_cmd %x\n", xfer_len, cmd);
	sys_write32(cmd, i2c_base + AST_I2CM_CMD_STS);

	return 0;
}

#if defined(CONFIG_SOC_AST2700_SSP) || \
	defined(CONFIG_SOC_AST2700_A1_SSP) || \
	defined(CONFIG_SOC_AST1040_CM4) || \
	defined(CONFIG_SOC_AST1080_CM4)

#else

static uint32_t ast2600_i2c_setup_buff_tx(uint32_t cmd, const struct device *dev)
{
	struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	struct i2c_msg *msg = &data->msgs[data->msgs_index];
	int xfer_len = msg->len - data->master_xfer_cnt;
	uint8_t wbuf[4], i;

	cmd |= AST_I2CM_PKT_EN;

	/*buff mode*/
	if (msg->len > config->buf_size) {
		xfer_len = config->buf_size;
	} else {
		if (data->msgs_index + 1 == data->msgs_count) {
			LOG_DBG("with stop\n");
			cmd |= AST_I2CM_STOP_CMD;
		}
		xfer_len = msg->len;
	}

	if (cmd & AST_I2CM_START_CMD)
		cmd |= AST_I2CM_PKT_ADDR(data->addr);

	if (xfer_len) {
		cmd |= AST_I2CM_TX_BUFF_EN | AST_I2CM_TX_CMD;
		sys_write32(AST_I2CC_SET_TX_BUF_LEN(xfer_len),
		i2c_base + AST_I2CC_BUFF_CTRL);
		for (i = 0; i < xfer_len; i++) {
			wbuf[i % 4] = msg->buf[i];
			if (i % 4 == 3) {
				sys_write32(*(uint32_t *)wbuf,
						config->buf_base + i - 3);
			}
			LOG_DBG("[%02x]\n", msg->buf[i]);
		}
		if (--i % 4 != 3) {
			sys_write32(*(uint32_t *)wbuf,
					config->buf_base + i - (i % 4));
		}
	}

	LOG_DBG("len %d , Buff tx_cmd %x\n", xfer_len, cmd);
	sys_write32(cmd, i2c_base + AST_I2CM_CMD_STS);

	return 0;
}

static uint32_t ast2600_i2c_setup_buff_rx(uint32_t cmd, const struct device *dev)
{
	struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	struct i2c_msg *msg = &data->msgs[data->msgs_index];
	int xfer_len = msg->len - data->master_xfer_cnt;

	cmd |= AST_I2CM_PKT_EN;

	/*buff mode*/
	if (msg->len > config->buf_size) {
		xfer_len = config->buf_size;
	} else {
		xfer_len = msg->len;
		if (data->msgs_index + 1 == data->msgs_count) {
			LOG_DBG("last stop\n");
			cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
		}
	}

	if (cmd & AST_I2CM_START_CMD)
		cmd |= AST_I2CM_PKT_ADDR(data->addr);

	if (xfer_len) {
		cmd |= AST_I2CM_RX_BUFF_EN | AST_I2CM_RX_CMD;
		sys_write32(AST_I2CC_SET_RX_BUF_LEN(xfer_len),
		i2c_base + AST_I2CC_BUFF_CTRL);
	}

	LOG_DBG("len %d , Buff tx_cmd %x\n", xfer_len, cmd);
	sys_write32(cmd, i2c_base + AST_I2CM_CMD_STS);

	return 0;
}

static uint32_t ast2600_i2c_setup_byte_tx(uint32_t cmd, const struct device *dev)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	struct i2c_msg *msg = &data->msgs[data->msgs_index];
	int xfer_len = msg->len - data->master_xfer_cnt;

	/*byte mode*/
	if ((data->msgs_index + 1 == data->msgs_count) && msg->len <= 1) {
		LOG_DBG("with stop\n");
		cmd |= AST_I2CM_STOP_CMD;
	}

	if (msg->len) {
		cmd |= AST_I2CM_TX_CMD;
		xfer_len = 1;
		LOG_DBG("w [0] : %02x\n", msg->buf[0]);
		sys_write32(msg->buf[0], i2c_base + AST_I2CC_STS_AND_BUFF);
	} else {
		xfer_len = 0;
	}

	LOG_DBG("len %d , Byte tx_cmd %x\n", xfer_len, cmd);
	sys_write32(cmd, i2c_base + AST_I2CM_CMD_STS);

	return 0;
}

static uint32_t ast2600_i2c_setup_byte_rx(uint32_t cmd, const struct device *dev)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	struct i2c_msg *msg = &data->msgs[data->msgs_index];
	int xfer_len = msg->len - data->master_xfer_cnt;

	/*byte mode*/
	if ((data->msgs_index + 1 == data->msgs_count) && msg->len == 1) {
		LOG_DBG("last stop\n");
		cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
	}

	if (msg->len) {
		cmd |= AST_I2CM_RX_CMD;
		xfer_len = 1;
	} else {
		xfer_len = 0;
	}

	LOG_DBG("len %d , Byte rx_cmd %x\n", xfer_len, cmd);
	sys_write32(cmd, i2c_base + AST_I2CM_CMD_STS);

	return 0;
}

#endif

static uint32_t aspeed_i2c_do_start(const struct device *dev)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	struct i2c_msg *msg = &data->msgs[data->msgs_index];

	/*send start*/
	LOG_DBG("[%s]: [%d/%d] %sing %d byte%s %s 0x%02x\n",
		dev->name, data->msgs_index, data->msgs_count,
		msg->flags & I2C_MSG_READ ? "read" : "write",
		msg->len, msg->len > 1 ? "s" : "",
		msg->flags & I2C_MSG_READ ? "from" : "to",
		data->addr);

	data->master_xfer_cnt = 0;
	data->buf_index = 0;

	if (msg->flags & I2C_MSG_READ)
		return data->setup_rx(AST_I2CM_START_CMD, dev);

	return data->setup_tx(AST_I2CM_START_CMD, dev);
}

static int i2c_aspeed_transfer(const struct device *dev, struct i2c_msg *msgs,
			       uint8_t num_msgs, uint16_t addr)
{
	struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	uint32_t isr = 0, ctrl = 0, sts = 0;

#ifdef CONFIG_I2C_TARGET
	uint32_t cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;
#endif

	if (!num_msgs) {
		return 0;
	}

	/* mutex lock for api re-entry */
	if (k_mutex_lock(&data->trans_mutex, I2C_ENTRY_TIMEOUT) != 0) {
		return -ETIMEDOUT;
	}

	/*If bus is busy in a single master environment, attempt recovery.*/
	if (!config->multi_master &&
	(sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF) & AST_I2CC_BUS_BUSY_STS)) {
		int ret;

		ret = aspeed_i2c_recover_bus(dev);
		if (ret) {
			k_mutex_unlock(&data->trans_mutex);
			return ret;
		}
	}

	data->addr = addr;
	data->cmd_err = 0;
	data->msgs = msgs;
	data->msgs_index = 0;
	data->msgs_count = num_msgs;
	k_sem_reset(&data->sync_sem);

	aspeed_i2c_do_start(dev);

	if (i2c_wait_completion(dev)) {
		isr = sys_read32(i2c_base + AST_I2CM_ISR);
		sts = sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF);
		LOG_DBG("timeout isr: %x, sts %x\n", isr, sts);

		/* do controller reset */
		if (isr || (sts & AST_I2CC_TX_DIR_MASK)) {
			ctrl = sys_read32(i2c_base + AST_I2CC_FUN_CTRL);
			sys_write32(0, i2c_base + AST_I2CC_FUN_CTRL);
			sys_write32(ctrl, i2c_base + AST_I2CC_FUN_CTRL);
#ifdef CONFIG_I2C_TARGET
			if (ctrl & AST_I2CC_SLAVE_EN) {
				if (config->mode == DMA_MODE) {
					uint64_t DMA_Addr =
					TO_PHY_ADDR((uintptr_t)data->slave_dma_buf);
					uint32_t DMA_Addr_L =
					(uint32_t)(DMA_Addr & 0xFFFFFFFF);
					uint32_t DMA_Addr_H =
					(uint32_t)(((DMA_Addr >> 32) & 0xFFFFFFFF));

					cmd |= AST_I2CS_RX_DMA_EN;
					sys_write32(DMA_Addr_L, i2c_base + AST_I2CS_RX_DMA);
					sys_write32(DMA_Addr_H, i2c_base + AST_I2CS_RX_DMA_H);
					sys_write32(DMA_Addr_L, i2c_base + AST_I2CS_TX_DMA);
					sys_write32(DMA_Addr_H, i2c_base + AST_I2CS_TX_DMA_H);
					sys_write32(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_BUF_SIZE)
					, i2c_base + AST_I2CS_DMA_LEN);
				} else if (config->mode == BUFF_MODE) {
					cmd |= AST_I2CS_RX_BUFF_EN;
					sys_write32(AST_I2CC_SET_RX_BUF_LEN(config->buf_size)
					, i2c_base + AST_I2CC_BUFF_CTRL);
				} else {
					cmd &= ~AST_I2CS_PKT_MODE_EN;
				}
				LOG_DBG("slave trigger: %x\n", cmd);
				sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);
			}
#endif
		}

		k_mutex_unlock(&data->trans_mutex);
		return -ETIMEDOUT;
	}

	/* cache flush for read buffer */
	if (msgs->flags & I2C_MSG_READ) {
		if (config->mode == DMA_MODE) {
			cache_data_invd_range(&msgs->buf,
			msgs->len);
		}
	}

	LOG_DBG(" end %d\n", data->cmd_err);

	/* mutex unlock */
	k_mutex_unlock(&data->trans_mutex);

	return data->cmd_err;
}

uint32_t ast2600_i2c_is_irq_error(uint32_t irq_status)
{
	if (irq_status & AST_I2CM_ARBIT_LOSS) {
		return -EAGAIN;
	}
	if (irq_status & (AST_I2CM_SDA_DL_TO |
			  AST_I2CM_SCL_LOW_TO)) {
		return -EBUSY;
	}
	if (irq_status & (AST_I2CM_ABNORMAL)) {
		return -EPROTO;
	}

	return 0;
}

uint32_t ast2700_i2c_is_irq_error(uint32_t irq_status)
{
	if (irq_status & AST2700_I2CM_ABNORMAL) {
		return -EAGAIN;
	}
	if (irq_status & (AST_I2CM_SDA_DL_TO |
			  AST_I2CM_SCL_LOW_TO)) {
		return -EBUSY;
	}
	if (irq_status & (AST_I2CM_ABNORMAL)) {
		return -EPROTO;
	}

	return 0;
}

void do_i2cm_tx(const struct device *dev)
{
	struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	struct i2c_msg *msg = &data->msgs[data->msgs_index];
	int xfer_len;

	if (config->mode == DMA_MODE) {
		xfer_len =
		AST_I2C_GET_TX_DMA_LEN(sys_read32(i2c_base + AST_I2CM_DMA_LEN_STS));
	} else if (config->mode == BUFF_MODE) {
		xfer_len =
		AST_I2CC_GET_TX_BUF_LEN(sys_read32(i2c_base + AST_I2CC_BUFF_CTRL));
	} else {
		xfer_len = 1;
	}
	data->master_xfer_cnt += xfer_len;

	if (data->master_xfer_cnt == msg->len) {
		data->msgs_index++;
		if (data->msgs_index == data->msgs_count) {
			k_sem_give(&data->sync_sem);
		} else {
			aspeed_i2c_do_start(dev);
		}
	} else {
		/*do next tx*/
		data->setup_tx(0, dev);
	}
}

void do_i2cm_rx(const struct device *dev)
{
	struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	struct i2c_msg *msg = &data->msgs[data->msgs_index];
	int xfer_len, i;

	/*do next rx*/
	if (config->mode == DMA_MODE) {
		xfer_len =
		AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base + AST_I2CM_DMA_LEN_STS));
	} else if (config->mode == BUFF_MODE) {
		xfer_len =
		AST_I2CC_GET_RX_BUF_LEN(sys_read32(i2c_base + AST_I2CC_BUFF_CTRL));
		for (i = 0; i < xfer_len; i++) {
			msg->buf[data->master_xfer_cnt + i] =
			sys_read8(config->buf_base + i);
		}
	} else {
		xfer_len = 1;
		msg->buf[data->master_xfer_cnt] =
		AST_I2CC_GET_RX_BUFF(sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));
	}

	/*update transfer length*/
	data->master_xfer_cnt += xfer_len;
	LOG_DBG("master_xfer_cnt [%d/%d]\n", data->master_xfer_cnt, msg->len);

	if (data->master_xfer_cnt == msg->len) {
		/*TODO dma unmap*/
		/*Assure cache coherency after DMA write operation*/
		cache_data_invd_range(msg->buf, (size_t)(msg->len));

		for (i = 0; i < msg->len; i++) {
			LOG_DBG("M: r %d:[%x]\n", i, msg->buf[i]);
		}
		data->msgs_index++;
		if (data->msgs_index == data->msgs_count) {
			k_sem_give(&data->sync_sem);
		} else {
			aspeed_i2c_do_start(dev);
		}
	} else {
		/*next rx*/
		data->setup_rx(0, dev);
	}
}

int aspeed_i2c_master_irq(const struct device *dev)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	uint32_t sts = sys_read32(i2c_base + AST_I2CM_ISR);

	LOG_DBG("M sts %x\n", sts);

	if (!data->alert_enable) {
		sts &= ~AST_I2CM_SMBUS_ALT;
	}

	if (AST_I2CM_BUS_RECOVER_FAIL & sts) {
		LOG_DBG("AST_I2CM_BUS_RECOVER_FAIL\n");
		LOG_DBG("M clear isr: AST_I2CM_BUS_RECOVER_FAIL= %x\n", sts);
		/*clear other status to avoid endless irq in recovery fail condition*/
		/*if any other irq is existed, it should be clear here*/
		sys_write32(sts, i2c_base + AST_I2CM_ISR);
		if (data->bus_recover) {
			data->cmd_err = -EPROTO;
			data->bus_recover = 0;
			k_sem_give(&data->sync_sem);
		} else {
			LOG_DBG("Error !! Bus revover\n");
		}
		return 1;
	}

	if (AST_I2CM_BUS_RECOVER & sts) {
		LOG_DBG("M clear isr: AST_I2CM_BUS_RECOVER= %x\n", sts);
		sys_write32(AST_I2CM_BUS_RECOVER, i2c_base + AST_I2CM_ISR);
		data->cmd_err = 0;
		if (data->bus_recover) {
			data->bus_recover = 0;
			k_sem_give(&data->sync_sem);
		} else {
			LOG_DBG("Error !! Bus revover\n");
		}
		return 1;
	}

	if (AST_I2CM_SMBUS_ALT & sts) {
		sts &= ~AST_I2CM_SMBUS_ALT;
		if (sys_read32(i2c_base + AST_I2CM_IER) & AST_I2CM_SMBUS_ALT) {
			LOG_DBG("AST_I2CM_SMBUS_ALT 0x%02x\n", sts);
			LOG_DBG("M clear isr: AST_I2CM_SMBUS_ALT= %x\n", sts);
			/*Disable ALT INT*/
			sys_write32(sys_read32(i2c_base + i2c_base + AST_I2CM_IER) &
				    ~AST_I2CM_SMBUS_ALT,
				    AST_I2CM_IER);
			/*i2c_handle_smbus_alert(data->ara);*/
			sys_write32(AST_I2CM_SMBUS_ALT, i2c_base + AST_I2CM_ISR);
			LOG_DBG("TODO aspeed_master_alert_recv bus id Disable Alt, Please Imple\n");
			return 1;
		}
	}

	data->cmd_err = data->is_irq_err(sts);
	if (data->cmd_err) {
		LOG_DBG("received error interrupt: 0x%02x\n",
			sts);
		if (data->version == AST2700) {
			sys_write32(sts, i2c_base + AST_I2CM_ISR);
		} else {
			sys_write32(AST_I2CM_PKT_DONE | AST_I2CM_PKT_ERROR,
				i2c_base + AST_I2CM_ISR);
		}
		k_sem_give(&data->sync_sem);
		return 1;
	}

	if (AST_I2CM_PKT_DONE & sts) {
		if (data->version == AST2700)
			sys_write32(sts, i2c_base + AST_I2CM_ISR);
		else
			sys_write32(AST_I2CM_PKT_DONE, i2c_base + AST_I2CM_ISR);

		sts &= ~(AST_I2CM_PKT_DONE | AST_I2CM_SW_ISR_MASK);

		switch (sts) {
		case AST_I2CM_PKT_ERROR | AST_I2CM_TX_NAK:	/* a0 fix for issue */
		/*LOG_DBG("a0 workaround for M TX NAK [%x]\n",*/
		/*sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));*/
		case AST_I2CM_PKT_ERROR | AST_I2CM_TX_NAK | AST_I2CM_NORMAL_STOP:
			LOG_DBG("M : TX NAK | NORMAL STOP\n");
			data->cmd_err = -ENXIO;
			k_sem_give(&data->sync_sem);
			break;
		case AST_I2CM_NORMAL_STOP:
			/*write 0 byte only have stop isr*/
			LOG_DBG("M clear isr: AST_I2CM_NORMAL_STOP = %x\n", sts);
			data->msgs_index++;
			/* if there is another message need to send, trigger here */
			if (data->msgs_index < data->msgs_count) {
				aspeed_i2c_do_start(dev);
			} else {
				k_sem_give(&data->sync_sem);
			}
			break;
		case AST_I2CM_TX_ACK:
		case AST_I2CM_TX_ACK | AST_I2CM_NORMAL_STOP:
#ifdef CONFIG_I2C_TARGET
			if (sts == AST_I2CM_TX_ACK) {
				/* Workaround for master/slave package mode
				 * enable rx done stuck issue
				 * When master go for first read (RX_DONE),
				 * slave mode will also effect
				 * Then controller will send nack,
				 * not operate anymore.
				 */
				if (sys_read32(i2c_base +
					AST_I2CS_CMD_STS) & AST_I2CS_PKT_MODE_EN) {
					uint32_t slave_cmd =
					sys_read32(i2c_base + AST_I2CS_CMD_STS);

					sys_write32(0, i2c_base + AST_I2CS_CMD_STS);
					sys_write32(slave_cmd, i2c_base + AST_I2CS_CMD_STS);
				}
			}
#endif
			LOG_DBG("M : I2CM_TX_ACK | I2CM_N_S = %x\n", sts);
			do_i2cm_tx(dev);
			break;
		case AST_I2CM_RX_DONE:
		/*LOG_DBG("M : AST_I2CM_RX_DONE = %x\n", sts);*/
		case AST_I2CM_RX_DONE | AST_I2CM_NORMAL_STOP:
			LOG_DBG("M : I2CM_RX_DONE | I2CM_N_S = %x\n", sts);
			do_i2cm_rx(dev);
			break;
		default:
			LOG_DBG("TODO care -- > sts %x\n", sts);
		/*sys_write32(sys_read32(i2c_base + AST_I2CM_ISR), i2c_base + AST_I2CM_ISR);*/
			break;
		}

		return 1;
	}

	if (sys_read32(i2c_base + AST_I2CM_ISR)) {
		LOG_DBG("TODO care -- > sts %x\n", sys_read32(i2c_base + AST_I2CM_ISR));
		sys_write32(sys_read32(i2c_base + AST_I2CM_ISR), i2c_base + AST_I2CM_ISR);
	}

	return 0;

}

#ifdef CONFIG_I2C_TARGET
static inline void aspeed_i2c_trigger_package_cmd(uint32_t i2c_base, uint8_t mode)
{
	uint32_t cmd = SLAVE_TRIGGER_CMD;

	if (mode == DMA_MODE) {
		cmd |= AST_I2CS_RX_DMA_EN;
	} else if (mode == BUFF_MODE) {
		cmd |= AST_I2CS_RX_BUFF_EN;
	} else {
		cmd &= ~AST_I2CS_PKT_MODE_EN;
	}
	sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);
}

static void ast2700_i2c_get_target(struct i2c_aspeed_data *data, uint8_t addr)
{
	uint8_t i = 0;
	bool target_find = false;

	/* find target by address */
	for (i = 0; i < I2C_SLAVE_COUNT; i++) {
		if (data->slave_cfg[i]) {
			if (data->slave_cfg[i]->address == addr) {
				LOG_DBG("address [%x] on %d\n", addr, i);
				data->slave_get_cfg = data->slave_cfg[i];
				data->slave_cb = data->slave_get_cfg->callbacks;
				target_find = true;
			}
		}
	}

	if (!target_find)
		LOG_DBG("address [%x] could not find\n", addr);
}

void ast2700_i2c_slave_packet_irq(const struct device *dev, uint32_t i2c_base, uint32_t sts)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	int slave_rx_len = 0;
	uint32_t cmd = 0;
	int i;
	uint32_t sirq_log;
	uint32_t isr;

	sys_write32(AST_I2CS_SADDR_PENDING | AST_I2CS_WAIT_TX_DMA | AST_I2CS_WAIT_RX_DMA,
	i2c_base + AST_I2CS_ISR);
	isr = sys_read32(i2c_base + AST_I2CS_ISR);

	sts = isr & ~(AST_I2CS_SLAVE_PENDING | AST_I2CS_ADDR_NAK_MASK);

	/* Handle i2c slave timeout condition */
	/* Skip */

	if (AST_I2CS_ABNOR_STOP & sts) {
		LOG_ERR("The target abnomal protocol occurs isr: 0x%08x.\n", isr);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);
		sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);
		/* clear sirq log */
		while ((sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG))) {
			/* assign the target client*/
			if (sirq_log & SADDR_HIT) {
				if (!data->slave_get_cfg)
					ast2700_i2c_get_target(data,
							       sirq_log >> SLAVE_ADDR_SHIFT);
			}
		};
		sys_write32(isr, i2c_base + AST_I2CS_ISR);
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
			data->slave_get_cfg = NULL;
		}
		return;
	}

	sts &= ~(AST_I2CS_PKT_DONE | AST_I2CS_PKT_ERROR);

	switch (sts) {
	case AST_I2CS_SADDR_PENDING | AST_I2CS_WAIT_RX_DMA |
		AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_STOP:
	case AST_I2CS_SADDR_PENDING | AST_I2CS_WAIT_RX_DMA |
		AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE:
		sys_write32(AST_I2CS_SLAVE_MATCH, i2c_base + AST_I2CS_ISR);
		isr = sys_read32(i2c_base + AST_I2CS_ISR);
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
						   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->write_requested(data->slave_get_cfg);
		}
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
							  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_cb->write_received) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		if (sts & AST_I2CS_STOP) {
			if (data->slave_get_cfg) {
				data->slave_cb->stop(data->slave_get_cfg);
			}
			data->slave_get_cfg = NULL;
		}
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
						   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->write_requested(data->slave_get_cfg);
		}
		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		break;
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_STOP:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		/* bug workaround */
		if (sirq_log & SADDR_HIT) {
			if (!data->slave_get_cfg)
				ast2700_i2c_get_target(data,
						   sirq_log >> SLAVE_ADDR_SHIFT);
			if (data->slave_get_cfg) {
				data->slave_cb->write_requested(data->slave_get_cfg);
			}
			sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		}
		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
							  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_get_cfg) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
		}
		data->slave_get_cfg = NULL;
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		break;
	case AST_I2CS_SLAVE_MATCH:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
						   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->write_requested(data->slave_get_cfg);
		}
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
							  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_get_cfg) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		break;
	case AST_I2CS_SADDR_PENDING | AST_I2CS_SLAVE_MATCH |
		AST_I2CS_RX_DONE | AST_I2CS_STOP:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_STOP:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
						   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->write_requested(data->slave_get_cfg);
		}
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
							  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_get_cfg) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
		}
		data->slave_get_cfg = NULL;

		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		break;
	case AST_I2CS_WAIT_RX_DMA | AST_I2CS_SLAVE_MATCH |
		AST_I2CS_RX_DONE | AST_I2CS_STOP:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		/* workaround: false alarm slave match check */
		if (sirq_log & SADDR_HIT) {
			if (!data->slave_get_cfg)
				ast2700_i2c_get_target(data,
						   sirq_log >> SLAVE_ADDR_SHIFT);
			if (data->slave_get_cfg) {
				data->slave_cb->write_requested(data->slave_get_cfg);
			}
			sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		}
		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
									  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_get_cfg) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
		}
		data->slave_get_cfg = NULL;
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->write_requested(data->slave_get_cfg);
		}
		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		break;
	case AST_I2CS_RX_DONE | AST_I2CS_STOP:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		/* workaround new slave match */
		if (sirq_log & SADDR_HIT) {
			if (!data->slave_get_cfg)
				ast2700_i2c_get_target(data,
						   sirq_log >> SLAVE_ADDR_SHIFT);
			if (data->slave_get_cfg) {
				data->slave_cb->write_requested(data->slave_get_cfg);
			}
			sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		}
		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
							  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_get_cfg) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
		}
		data->slave_get_cfg = NULL;
		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		break;
	case AST_I2CS_TX_NAK | AST_I2CS_STOP | AST_I2CS_SLAVE_MATCH:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
		}
		data->slave_get_cfg = NULL;
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (sirq_log & SADDR_HIT) {
			if (!data->slave_get_cfg)
				ast2700_i2c_get_target(data,
						   sirq_log >> SLAVE_ADDR_SHIFT);
		}
		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		/* workaround: not clear slave match due to wait next isr check tx or rx */
		isr &= ~AST_I2CS_SLAVE_MATCH;
		break;
	case AST_I2CS_TX_NAK | AST_I2CS_STOP:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
		}
		data->slave_get_cfg = NULL;
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);

		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);

		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		break;
	case AST_I2CS_WAIT_RX_DMA | AST_I2CS_TX_NAK |
		AST_I2CS_STOP | AST_I2CS_SLAVE_MATCH:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
		}
		data->slave_get_cfg = NULL;
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->write_requested(data->slave_get_cfg);
		}
		sys_write32(AST_I2CS_SET_RX_DMA_LEN(ASPEED_I2C_DMA_SIZE),
		       i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_RX_DMA_EN;
		break;
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_WAIT_TX_DMA:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->read_requested(data->slave_get_cfg
			, &data->slave_dma_buf[0]);
		}
		LOG_DBG("tx [%02x]", data->slave_dma_buf[0]);
		sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
		, i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_TX_DMA_EN;
		break;
	case AST_I2CS_TX_ACK | AST_I2CS_WAIT_TX_DMA:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (data->slave_get_cfg) {
			data->slave_cb->read_processed(data->slave_get_cfg
			, &data->slave_dma_buf[0]);
		}
		LOG_DBG("tx [%02x]", data->slave_dma_buf[0]);
		sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
		, i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_TX_DMA_EN;
		break;
	case AST_I2CS_WAIT_TX_DMA | AST_I2CS_SLAVE_MATCH |
		AST_I2CS_RX_DONE | AST_I2CS_STOP:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
									  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_get_cfg) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
		}
		data->slave_get_cfg = NULL;
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->read_requested(data->slave_get_cfg
			, &data->slave_dma_buf[0]);
		}
		LOG_DBG("tx [%02x]", data->slave_dma_buf[0]);
		sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
		, i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_TX_DMA_EN;
		break;
	case AST_I2CS_WAIT_TX_DMA | AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
									  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_get_cfg) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->read_requested(data->slave_get_cfg
			, &data->slave_dma_buf[0]);
		}
		LOG_DBG("tx [%02x]", data->slave_dma_buf[0]);
		sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
		, i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_TX_DMA_EN;
		break;
	case AST_I2CS_SADDR_PENDING | AST_I2CS_WAIT_TX_DMA |
		AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE:
		sys_write32(AST_I2CS_SLAVE_MATCH, i2c_base + AST_I2CS_ISR);
		isr = sys_read32(i2c_base + AST_I2CS_ISR);
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->write_requested(data->slave_get_cfg);
		}
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
									  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_get_cfg) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->read_requested(data->slave_get_cfg
			, &data->slave_dma_buf[0]);
		}
		LOG_DBG("tx [%02x]", data->slave_dma_buf[0]);
		sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
		, i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_TX_DMA_EN;
		break;
	case AST_I2CS_SADDR_PENDING | AST_I2CS_WAIT_TX_DMA |
			AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_STOP:
		sys_write32(AST_I2CS_SLAVE_MATCH, i2c_base + AST_I2CS_ISR);
		isr = sys_read32(i2c_base + AST_I2CS_ISR);
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->write_requested(data->slave_get_cfg);
		}
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);

		slave_rx_len = AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base +
							  AST_I2CS_DMA_LEN_STS));

		/*aspeed_cache_invalid_data*/
		cache_data_invd_range((&data->slave_dma_buf[0])
		, slave_rx_len);

		if (data->slave_get_cfg) {
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
				data->slave_cb->write_received(data->slave_get_cfg
				, data->slave_dma_buf[i]);
			}
		}
		if (data->slave_get_cfg) {
			data->slave_cb->stop(data->slave_get_cfg);
		}
		data->slave_get_cfg = NULL;
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->read_requested(data->slave_get_cfg
			, &data->slave_dma_buf[0]);
		}
		LOG_DBG("tx [%02x]", data->slave_dma_buf[0]);
		sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
		, i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_TX_DMA_EN;
		break;
	case AST_I2CS_WAIT_TX_DMA:
		sirq_log = sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG);
		if (!data->slave_get_cfg)
			ast2700_i2c_get_target(data,
					   sirq_log >> SLAVE_ADDR_SHIFT);
		if (data->slave_get_cfg) {
			data->slave_cb->read_requested(data->slave_get_cfg
			, &data->slave_dma_buf[0]);
		}
		LOG_DBG("tx [%02x]", data->slave_dma_buf[0]);
		sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
		, i2c_base + AST_I2CS_DMA_LEN);
		cmd = SLAVE_TRIGGER_CMD | AST_I2CS_TX_DMA_EN;
		break;
	default:
		LOG_DBG("unhandled slave isr case %x, sts %x\n", sts,
			sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));

		/* clear sirq log */
		while (sys_read32(i2c_base + AST2700_I2CC_SIRQ_LOG))
			;
		break;
	}

	if (cmd)
		sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);

	sys_write32(isr, i2c_base + AST_I2CS_ISR);
	sys_read32(i2c_base + AST_I2CS_ISR);
}

void ast2600_i2c_slave_packet_irq(const struct device *dev, uint32_t i2c_base, uint32_t sts)
{
	struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	struct i2c_target_config *slave_cfg = data->slave_cfg[AST_I2CS_GET_SLAVE(sts)];
	const struct i2c_target_callbacks *slave_cb = slave_cfg->callbacks;
	uint32_t cmd = 0;
	uint32_t i, slave_rx_len = 0;
	uint8_t byte_data = 0, value = 0;

	/* clear irq first */
	sys_write32(AST_I2CS_PKT_DONE, i2c_base + AST_I2CS_ISR);
	sys_read32(i2c_base + AST_I2CS_ISR);

	sts &= ~(AST_I2CS_PKT_DONE | AST_I2CS_PKT_ERROR | AST_I2CS_ADDR_INDICATE_MASK);

	switch (sts) {
	case AST_I2CS_SLAVE_MATCH:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE:
		if (slave_cb->write_requested) {
			slave_cb->write_requested(slave_cfg);
		}
		break;
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_WAIT_RX_DMA:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_WAIT_RX_DMA: /* re-trigger? */
		if (sys_read32(i2c_base + AST_I2CM_ISR)) {
			LOG_DBG("S : Sw|D - Wait normal\n");
		} else {
			LOG_DBG("S : Sw|D - Issue rx dma\n");
			if (slave_cb->write_requested) {
				slave_cb->write_requested(slave_cfg);
			}

			if (config->mode == DMA_MODE) {
				slave_rx_len =
				AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base + AST_I2CS_DMA_LEN_STS));

				/*aspeed_cache_invalid_data*/
				cache_data_invd_range((&data->slave_dma_buf[0])
				, slave_rx_len);

				if (slave_cb->write_received) {
					for (i = 0; i < slave_rx_len; i++) {
						LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
						slave_cb->write_received(slave_cfg
						, data->slave_dma_buf[i]);
					}
				}
			} else if (config->mode == BUFF_MODE) {
				LOG_DBG("Slave_Buff");
				slave_rx_len =
				AST_I2CC_GET_RX_BUF_LEN(sys_read32(i2c_base + AST_I2CC_BUFF_CTRL));

				if (slave_cb->write_received) {
					for (i = 0; i < slave_rx_len ; i++) {
						slave_cb->write_received(slave_cfg
						, sys_read8(config->buf_base + i));
					}
				}
			} else {
				byte_data =
				AST_I2CC_GET_RX_BUFF(sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));
				LOG_DBG("[%02x]", byte_data);
				if (slave_cb->write_received) {
					slave_cb->write_received(slave_cfg, byte_data);
				}
			}
			aspeed_i2c_trigger_package_cmd(i2c_base, config->mode);
		}
		break;
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_STOP:
		LOG_DBG("S : Sw | P\n");
		if (slave_cb->stop) {
			slave_cb->stop(slave_cfg);
		}
		aspeed_i2c_trigger_package_cmd(i2c_base, config->mode);
		break;
	case AST_I2CS_RX_DONE | AST_I2CS_STOP:
	case AST_I2CS_RX_DONE | AST_I2CS_WAIT_RX_DMA: /* wait for last package received data done */
	case AST_I2CS_RX_DONE | AST_I2CS_WAIT_RX_DMA | AST_I2CS_STOP:
	case AST_I2CS_RX_DONE_NAK | AST_I2CS_RX_DONE | AST_I2CS_STOP:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_STOP:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_WAIT_RX_DMA | AST_I2CS_STOP:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE_NAK | AST_I2CS_RX_DONE | AST_I2CS_STOP:
		if (sts & AST_I2CS_STOP) {
			if (sts & AST_I2CS_SLAVE_MATCH) {
				LOG_DBG("S : Sw|D|P\n");
			} else {
				LOG_DBG("S : D|P\n");
			}
		} else {
			LOG_DBG("S : Sw|D\n");
		}

		if (sts & AST_I2CS_SLAVE_MATCH) {
			if (slave_cb->write_requested) {
				slave_cb->write_requested(slave_cfg);
			}
		}

		if (config->mode == DMA_MODE) {
			slave_rx_len =
			AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base + AST_I2CS_DMA_LEN_STS));

			/*aspeed_cache_invalid_data*/
			cache_data_invd_range((&data->slave_dma_buf[0])
			, slave_rx_len);

			if (slave_cb->write_received) {
				for (i = 0; i < slave_rx_len; i++) {
					LOG_DBG("[%02x] ", data->slave_dma_buf[i]);
					slave_cb->write_received(slave_cfg
					, data->slave_dma_buf[i]);
				}
			}

			sys_write32(0, i2c_base + AST_I2CS_DMA_LEN_STS);
			sys_write32(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_BUF_SIZE)
			, i2c_base + AST_I2CS_DMA_LEN);
		} else if (config->mode == BUFF_MODE) {
			LOG_DBG("Slave_Buff");
			slave_rx_len =
			AST_I2CC_GET_RX_BUF_LEN(sys_read32(i2c_base + AST_I2CC_BUFF_CTRL));

			if (slave_cb->write_received) {
				for (i = 0; i < slave_rx_len ; i++) {
					slave_cb->write_received(slave_cfg
					, sys_read8(config->buf_base + i));
				}
			}
		} else {
			byte_data =
			AST_I2CC_GET_RX_BUFF(sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));
			LOG_DBG("[%02x]", byte_data);
			if (slave_cb->write_received) {
				slave_cb->write_received(slave_cfg, byte_data);
			}
		}
		if (sts & AST_I2CS_STOP) {
			if (slave_cb->stop) {
				slave_cb->stop(slave_cfg);
			}
		}
		aspeed_i2c_trigger_package_cmd(i2c_base, config->mode);
		break;
	/*it is Mw data Mr coming -> it need send tx*/
	case AST_I2CS_RX_DONE | AST_I2CS_WAIT_TX_DMA:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_WAIT_TX_DMA:
		/*it should be repeat start read*/
		if (sts & AST_I2CS_SLAVE_MATCH) {
			LOG_DBG("S: I2CS_W_TX_DMA | I2CS_S_MATCH | I2CS_R_DONE\n");
		} else {
			LOG_DBG("S: I2CS_W_TX_DMA | I2CS_R_DONE\n");
		}

		if (sts & AST_I2CS_SLAVE_MATCH) {
			if (slave_cb->write_requested) {
				slave_cb->write_requested(slave_cfg);
			}
		}

		cmd = SLAVE_TRIGGER_CMD;
		if (config->mode == DMA_MODE) {
			cmd |= AST_I2CS_TX_DMA_EN;
			slave_rx_len =
			AST_I2C_GET_RX_DMA_LEN(sys_read32(i2c_base + AST_I2CS_DMA_LEN_STS));

			for (i = 0; i < slave_rx_len; i++) {
				cache_data_invd_range((&data->slave_dma_buf[i])
				, 1);
				LOG_DBG("rx [%02x]", data->slave_dma_buf[i]);
				if (slave_cb->write_received) {
					slave_cb->write_received(slave_cfg
					, data->slave_dma_buf[i]);
				}
			}

			if (slave_cb->read_requested) {
				slave_cb->read_requested(slave_cfg
				, &data->slave_dma_buf[0]);
			}
			LOG_DBG("tx [%02x]", data->slave_dma_buf[0]);

			sys_write32(0, i2c_base + AST_I2CS_DMA_LEN_STS);
			sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
			, i2c_base + AST_I2CS_DMA_LEN);
		} else if (config->mode == BUFF_MODE) {

			cmd |= AST_I2CS_TX_BUFF_EN;
			slave_rx_len =
			AST_I2CC_GET_RX_BUF_LEN(sys_read32(i2c_base + AST_I2CC_BUFF_CTRL));
			for (i = 0; i < slave_rx_len; i++) {
				LOG_DBG("rx [%02x]", (sys_read32(config->buf_base + i) & 0xFF));
				if (slave_cb->write_received) {
					slave_cb->write_received(slave_cfg
					, (sys_read32(config->buf_base + i) & 0xFF));
				}
			}

			if (slave_cb->read_requested) {
				slave_cb->read_requested(slave_cfg, &value);
			}
			LOG_DBG("tx [%02x]", value);

			sys_write32(value, config->buf_base);
			sys_write32(AST_I2CC_SET_TX_BUF_LEN(1)
			, i2c_base + AST_I2CC_BUFF_CTRL);
		} else {
			cmd &= ~AST_I2CS_PKT_MODE_EN;
			cmd |= AST_I2CS_TX_CMD;
			byte_data = AST_I2CC_GET_RX_BUFF(sys_read32(i2c_base +
			AST_I2CC_STS_AND_BUFF));

			LOG_DBG("rx : [%02x]", byte_data);
			if (slave_cb->write_received) {
				slave_cb->write_received(slave_cfg, byte_data);
			}
			if (slave_cb->read_requested) {
				slave_cb->read_requested(slave_cfg, &byte_data);
			}
			LOG_DBG("tx : [%02x]", byte_data);
			sys_write32(byte_data, i2c_base + AST_I2CC_STS_AND_BUFF);
		}
		LOG_DBG("slave cmd %x\n", cmd);
		sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);
		break;

	case AST_I2CS_SLAVE_MATCH | AST_I2CS_WAIT_TX_DMA:
		/*First Start read*/
		LOG_DBG("S: AST_I2CS_SLAVE_MATCH | AST_I2CS_Wait_TX_DMA\n");
		cmd = SLAVE_TRIGGER_CMD;
		if (config->mode == DMA_MODE) {
			cmd |= AST_I2CS_TX_DMA_EN;
			if (slave_cb->read_requested) {
				slave_cb->read_requested(slave_cfg
				, &data->slave_dma_buf[0]);
			}
			/*currently i2c slave framework only support one byte request.*/
			LOG_DBG("tx: [%x]\n", data->slave_dma_buf[0]);
			sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
			, i2c_base + AST_I2CS_DMA_LEN);
		} else if (config->mode == BUFF_MODE) {
			cmd |= AST_I2CS_TX_BUFF_EN;
			if (slave_cb->read_requested) {
				slave_cb->read_requested(slave_cfg, &byte_data);
			}
			/* currently i2c slave framework only support one byte request. */
			LOG_DBG("tx : [%02x]", byte_data);
			sys_write8(byte_data, config->buf_base);
			sys_write32(AST_I2CC_SET_TX_BUF_LEN(1)
			, i2c_base + AST_I2CC_BUFF_CTRL);
		} else {
			cmd &= ~AST_I2CS_PKT_MODE_EN;
			cmd |= AST_I2CS_TX_CMD;
			if (slave_cb->read_requested) {
				slave_cb->read_requested(slave_cfg, &byte_data);
			}
			sys_write32(byte_data, i2c_base + AST_I2CC_STS_AND_BUFF);
		}
		sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);
		break;

	case AST_I2CS_WAIT_TX_DMA:
		/*it should be next start read*/
		LOG_DBG("S: AST_I2CS_Wait_TX_DMA\n");
		cmd = SLAVE_TRIGGER_CMD;
		if (config->mode == DMA_MODE) {
			cmd |= AST_I2CS_TX_DMA_EN;
			if (slave_cb->read_processed) {
				slave_cb->read_processed(slave_cfg
				, &data->slave_dma_buf[0]);
			}
			LOG_DBG("rx : [%02x]", data->slave_dma_buf[0]);
			sys_write32(0, i2c_base + AST_I2CS_DMA_LEN_STS);
			sys_write32(AST_I2CS_SET_TX_DMA_LEN(1)
			, i2c_base + AST_I2CS_DMA_LEN);
		} else if (config->mode == BUFF_MODE) {
			cmd |= AST_I2CS_TX_BUFF_EN;
			if (slave_cb->read_processed) {
				slave_cb->read_processed(slave_cfg, &value);
			}
			LOG_DBG("tx: [%02x]\n", value);
			sys_write8(value, config->buf_base);
			sys_write32(AST_I2CC_SET_TX_BUF_LEN(1)
			, i2c_base + AST_I2CC_BUFF_CTRL);
		} else {
			cmd &= ~AST_I2CS_PKT_MODE_EN;
			cmd |= AST_I2CS_TX_CMD;
			if (slave_cb->read_processed) {
				slave_cb->read_processed(slave_cfg, &byte_data);
			}
			LOG_DBG("tx: [%02x]\n", byte_data);
			sys_write32(byte_data, i2c_base + AST_I2CC_STS_AND_BUFF);
		}
		sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);
		break;

	case AST_I2CS_TX_NAK | AST_I2CS_STOP:
		LOG_DBG("S: AST_I2CS_TX_NAK\n");
	case AST_I2CS_STOP:
		/*it just tx complete*/
		LOG_DBG("S: AST_I2CS_STOP\n");
		cmd = SLAVE_TRIGGER_CMD;
		if (slave_cb->stop) {
			slave_cb->stop(slave_cfg);
		}
		if (config->mode == DMA_MODE) {
			cmd |= AST_I2CS_RX_DMA_EN;
			sys_write32(0, i2c_base + AST_I2CS_DMA_LEN_STS);
			sys_write32(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_BUF_SIZE)
			, i2c_base + AST_I2CS_DMA_LEN);
		} else if (config->mode == BUFF_MODE) {
			cmd |= AST_I2CS_RX_BUFF_EN;
			sys_write32(AST_I2CC_SET_RX_BUF_LEN(config->buf_size)
			, i2c_base + AST_I2CC_BUFF_CTRL);
		} else {
			cmd &= ~AST_I2CS_PKT_MODE_EN;
		}
		sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);
		break;

	default:
		LOG_DBG("TODO slave sts case %x, now %x\n"
		, sts, sys_read32(i2c_base + AST_I2CS_ISR));
		break;
	}
}

void aspeed_i2c_slave_byte_irq(const struct device *dev, uint32_t i2c_base, uint32_t sts)
{
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	struct i2c_target_config *slave_cfg = data->slave_cfg[AST_I2CS_GET_SLAVE(sts)];
	const struct i2c_target_callbacks *slave_cb = slave_cfg->callbacks;
	uint32_t cmd = AST_I2CS_ACTIVE_ALL;
	uint8_t byte_data = 0;

	sts &= ~(AST_I2CS_ADDR_INDICATE_MASK);

	LOG_DBG("byte mode\n");

	switch (sts) {
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_WAIT_RX_DMA:
		LOG_DBG("S : Sw|D\n");

		/* first address match is address */
		byte_data =
		AST_I2CC_GET_RX_BUFF(sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));
		LOG_DBG("addr [%x]", byte_data);

		/* If the record address is still same, it is re-start case. */
		if (slave_cb->write_requested &&
		byte_data != data->slave_addr_last) {
			slave_cb->write_requested(slave_cfg);
		}

		data->slave_addr_last = byte_data;
		break;

	/*pending stop and start address handle*/
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE |
	AST_I2CS_WAIT_RX_DMA | AST_I2CS_STOP | AST_I2CS_TX_NAK:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE |
	AST_I2CS_WAIT_RX_DMA | AST_I2CS_STOP:
		LOG_DBG("S : Sw|D|P\n");

		if (slave_cb->stop) {
			slave_cb->stop(slave_cfg);
		}

		/* clear record slave address */
		data->slave_addr_last = 0x0;

		/* first address match is address */
		byte_data =
		AST_I2CC_GET_RX_BUFF(sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));
		LOG_DBG("addr [%x]", byte_data);

		/* address set request */
		if (slave_cb->write_requested) {
			slave_cb->write_requested(slave_cfg);
		}

		data->slave_addr_last = byte_data;
		break;

	case AST_I2CS_RX_DONE | AST_I2CS_WAIT_RX_DMA:
		LOG_DBG("S : D\n");
		byte_data =
		AST_I2CC_GET_RX_BUFF(sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));
		LOG_DBG("rx [%x]", byte_data);

		if (slave_cb->write_received) {
			slave_cb->write_received(slave_cfg
			, byte_data);
		}
		break;
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_RX_DONE | AST_I2CS_WAIT_TX_DMA:
		cmd |= AST_I2CS_TX_CMD;
		LOG_DBG("S : Sr|D\n");
		byte_data =
		AST_I2CC_GET_RX_BUFF(sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));
		LOG_DBG("addr : [%02x]", byte_data);

		if (slave_cb->read_requested) {
			slave_cb->read_requested(slave_cfg
			, &byte_data);
		}

		LOG_DBG("tx: [%02x]\n", byte_data);
		sys_write32(byte_data, i2c_base + AST_I2CC_STS_AND_BUFF);
		break;
	case AST_I2CS_TX_ACK | AST_I2CS_WAIT_TX_DMA:
		cmd |= AST_I2CS_TX_CMD;
		LOG_DBG("S : D\n");

		if (slave_cb->read_processed) {
			slave_cb->read_processed(slave_cfg
			, &byte_data);
		}

		LOG_DBG("tx: [%02x]\n", byte_data);
		sys_write32(byte_data, i2c_base + AST_I2CC_STS_AND_BUFF);
		break;
	case AST_I2CS_STOP:
	case AST_I2CS_STOP | AST_I2CS_TX_NAK:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_STOP | AST_I2CS_TX_NAK:
	case AST_I2CS_SLAVE_MATCH | AST_I2CS_WAIT_RX_DMA | AST_I2CS_STOP | AST_I2CS_TX_NAK:
		LOG_DBG("S : P\n");
		if (slave_cb->stop) {
			slave_cb->stop(slave_cfg);
		}

		/* clear record slave address */
		data->slave_addr_last = 0x0;

		if (sts & AST_I2CS_SLAVE_MATCH) {
			/* Don't handle this match for current condition*/
			sts &= ~(AST_I2CS_SLAVE_MATCH);
		}

		if (sts & AST_I2CS_WAIT_RX_DMA) {
			/* Don't handle this waiting for current condition*/
			sts &= ~(AST_I2CS_WAIT_RX_DMA);
		}

		break;
	default:
		LOG_DBG("TODO no pkt_done intr ~~~ ***** sts %x\n", sts);
		break;
	}
	sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);
	sys_write32(sts, i2c_base + AST_I2CS_ISR);
}

int aspeed_i2c_slave_irq(const struct device *dev)
{
	uint32_t i2c_base = DEV_BASE(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t ier = sys_read32(i2c_base + AST_I2CS_IER);
	uint32_t sts = sys_read32(i2c_base + AST_I2CS_ISR);

	/* return without necessary slave interrupt */
	if (!(sts & ier)) {
		return 0;
	}

	LOG_DBG("S irq sts %x, bus %x\n", sts, sys_read32(i2c_base + AST_I2CC_STS_AND_BUFF));

	/* remove unnessary status flags */
	sts &= ~(AST_I2CS_SLAVE_PENDING);

	if (AST_I2CS_ADDR1_NAK & sts) {
		sts &= ~AST_I2CS_ADDR1_NAK;
	}

	if (AST_I2CS_ADDR2_NAK & sts) {
		sts &= ~AST_I2CS_ADDR2_NAK;
	}

	if (AST_I2CS_ADDR3_NAK & sts) {
		sts &= ~AST_I2CS_ADDR3_NAK;
	}

	if (AST_I2CS_ADDR_MASK & sts) {
		sts &= ~AST_I2CS_ADDR_MASK;
	}

	if (AST_I2CS_INACTIVE_TO & sts) {
		struct i2c_aspeed_config *i2c_config = DEV_CFG(dev);
		uint32_t cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;

		/*Turn off slave mode.*/
		sys_write32(~AST_I2CC_SLAVE_EN & sys_read32(i2c_base + AST_I2CC_FUN_CTRL)
		, i2c_base + AST_I2CC_FUN_CTRL);

		/*Set slave mode.*/
		if (i2c_config->mode == DMA_MODE) {
			cmd |= AST_I2CS_RX_DMA_EN;
		} else if (i2c_config->mode == BUFF_MODE) {
			cmd |= AST_I2CS_RX_BUFF_EN;
		} else {
			cmd &= ~AST_I2CS_PKT_MODE_EN;
		}

		/*Turn on slave mode and apply slave type*/
		sys_write32(AST_I2CC_SLAVE_EN | sys_read32(i2c_base + AST_I2CC_FUN_CTRL)
		, i2c_base + AST_I2CC_FUN_CTRL);
		sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);

		return 1;
	}

	if (AST_I2CS_PKT_DONE & sts) {
		if (data->version == AST2700) {
			ast2700_i2c_slave_packet_irq(dev, i2c_base, sts);
		} else {
			ast2600_i2c_slave_packet_irq(dev, i2c_base, sts);
		}
	} else {
		aspeed_i2c_slave_byte_irq(dev, i2c_base, sts);
	}

	return 1;
}
#endif

static void i2c_aspeed_isr(const struct device *dev)
{
#ifdef CONFIG_I2C_TARGET
	uint32_t i2c_base = DEV_BASE(dev);

	if (sys_read32(i2c_base + AST_I2CC_FUN_CTRL) & AST_I2CC_SLAVE_EN) {
		if (aspeed_i2c_slave_irq(dev)) {
			return;
		}
	}
#endif

	aspeed_i2c_master_irq(dev);
}

static int i2c_aspeed_init(const struct device *dev)
{
	struct i2c_aspeed_config *config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = DEV_DATA(dev);
	uint32_t i2c_base = DEV_BASE(dev);
	uint32_t bitrate_cfg;
	int error;
#ifdef CONFIG_I2C_TARGET
	int i;
#endif

	k_sem_init(&data->sync_sem, 0, UINT_MAX);

	config->global_reg = i2c_base & 0xfffff000;

	/* Just support i2c new (package) mode under Zeohyr */
	if (!(sys_read32(config->global_reg + ASPEED_I2CG_CONTROL) & ASPEED_I2C_NEW_MODE)) {
		return -ENOTSUP;
	}

	/* byte mode check re-start */
	data->slave_addr_last = 0xFF;

	/* initial slave attach function pointer */
	data->slave_attached = 0;
#ifdef CONFIG_I2C_TARGET
	for (i = 0; i < I2C_SLAVE_COUNT; i++)
		data->slave_cfg[i] = NULL;
#endif
	clock_control_get_rate(config->clock_dev, config->clk_id, &config->clk_src);
	LOG_INF("clk src %d, multi-master %d, xfer mode %d",
		config->clk_src, config->multi_master, config->mode);

#if defined(CONFIG_SOC_AST2700_SSP) || \
	defined(CONFIG_SOC_AST2700_A1_SSP) || \
	defined(CONFIG_SOC_AST1040_CM4) || \
	defined(CONFIG_SOC_AST1080_CM4)
	uint32_t reg;

	data->version = AST2700;
	reg = sys_read32(i2c_base + AST2700_I2CC_VER_CTRL);
	/* AST2700 need select DMA / Buffer mode in the version register*/
	if (config->mode == DMA_MODE) {
		reg |= USE_DMA_MODE;
		data->setup_tx = ast2600_i2c_setup_dma_tx;
		data->setup_rx = ast2600_i2c_setup_dma_rx;
	} else if (config->mode == BUFF_MODE) {
		sys_write32(0x00, i2c_base + AST_I2CM_TX_DMA);
		sys_write32(0x00, i2c_base + AST_I2CM_TX_DMA_H);
		sys_write32(0x10, i2c_base + AST_I2CM_RX_DMA);
		sys_write32(0x00, i2c_base + AST_I2CM_RX_DMA_H);
		sys_write32(0x30, i2c_base + AST_I2CS_TX_DMA);
		sys_write32(0x00, i2c_base + AST_I2CS_TX_DMA_H);
		sys_write32(0x30, i2c_base + AST_I2CS_RX_DMA);
		sys_write32(0x00, i2c_base + AST_I2CS_RX_DMA_H);
		reg &= ~USE_DMA_MODE;
	} else {
		return -EINVAL;
	}
	sys_write32(reg, i2c_base + AST2700_I2CC_VER_CTRL);

	data->is_irq_err = ast2700_i2c_is_irq_error;
#else
	data->version = AST2600;

	if (config->mode == DMA_MODE) {
		data->setup_tx = ast2600_i2c_setup_dma_tx;
		data->setup_rx = ast2600_i2c_setup_dma_rx;
	} else if (config->mode == BUFF_MODE) {
		data->setup_tx = ast2600_i2c_setup_buff_tx;
		data->setup_rx = ast2600_i2c_setup_buff_rx;
	} else {
		data->setup_tx = ast2600_i2c_setup_byte_tx;
		data->setup_rx = ast2600_i2c_setup_byte_rx;
	}

	data->is_irq_err = ast2600_i2c_is_irq_error;
#endif

	bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);
	error = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	error = i2c_aspeed_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (error) {
		return error;
	}

	config->irq_config_func(dev);

	return 0;
}

#ifdef CONFIG_I2C_TARGET
static int i2c_aspeed_slave_register(const struct device *dev,
				     struct i2c_target_config *config)
{
	struct i2c_aspeed_config *i2c_config = DEV_CFG(dev);
	struct i2c_aspeed_data *data = dev->data;
	uint8_t i = 0;
	uint32_t i2c_base = DEV_BASE(dev);
	uint64_t DMA_Addr = TO_PHY_ADDR((uintptr_t)data->slave_dma_buf);
	uint32_t DMA_Addr_L = (uint32_t)(DMA_Addr & 0xFFFFFFFF);
	uint32_t DMA_Addr_H = (uint32_t)(((DMA_Addr >> 32) & 0xFFFFFFFF));
	uint32_t cmd = AST_I2CS_ACTIVE_ALL | AST_I2CS_PKT_MODE_EN;
	uint32_t slave_addr = sys_read32(i2c_base + AST_I2CS_ADDR_CTRL);

	/* check slave config input */
	if (!config || data->slave_attached == 3) {
		return -EINVAL;
	}

	/* check duplicate address */
	for (i = 0; i < I2C_SLAVE_COUNT; i++) {
		if (data->slave_cfg[i]) {
			if (data->slave_cfg[i]->address == config->address) {
				LOG_DBG("duplicate address [%x] on %d\n", config->address, i);
				return -EINVAL;
			}
		}
	}

	/* assign the slave into slave array */
	for (i = 0; i < I2C_SLAVE_COUNT; i++) {
		if (!data->slave_cfg[i]) {
			data->slave_cfg[i] = config;
			LOG_DBG("reg [%x] into %d\n", config->address, i);
			/* set slave addr by index */
			switch (i) {
			case 0:
				slave_addr &= ~(AST_I2CS_ADDR1_MASK);
				slave_addr |= (AST_I2CS_ADDR1(config->address)
				| AST_I2CS_ADDR1_ENABLE);
				break;
			case 1:
				slave_addr &= ~(AST_I2CS_ADDR2_MASK);
				slave_addr |= (AST_I2CS_ADDR2(config->address)
				| AST_I2CS_ADDR2_ENABLE);
				break;
			case 2:
				slave_addr &= ~(AST_I2CS_ADDR3_MASK);
				slave_addr |= (AST_I2CS_ADDR3(config->address)
				| AST_I2CS_ADDR3_ENABLE);
				break;
			}
			/* set slave addr */
				LOG_DBG("reg slave_addr [%x]\n", slave_addr);
				sys_write32(slave_addr, i2c_base + AST_I2CS_ADDR_CTRL);
				break;
			}
	}

	/* trigger rx buffer */
	if (i2c_config->mode == DMA_MODE) {
		cmd |= AST_I2CS_RX_DMA_EN;
		sys_write32(DMA_Addr_L, i2c_base + AST_I2CS_TX_DMA);
		sys_write32(DMA_Addr_H, i2c_base + AST_I2CS_TX_DMA_H);
		sys_write32(DMA_Addr_L, i2c_base + AST_I2CS_RX_DMA);
		sys_write32(DMA_Addr_H, i2c_base + AST_I2CS_RX_DMA_H);
		sys_write32(AST_I2CS_SET_RX_DMA_LEN(I2C_SLAVE_BUF_SIZE),
		i2c_base + AST_I2CS_DMA_LEN);
	} else if (i2c_config->mode == BUFF_MODE) {
		cmd |= AST_I2CS_RX_BUFF_EN;
		sys_write32(AST_I2CC_SET_RX_BUF_LEN(i2c_config->buf_size),
		i2c_base + AST_I2CC_BUFF_CTRL);
	} else {
		cmd &= ~AST_I2CS_PKT_MODE_EN;
	}

	/* enable slave device */
	sys_write32(AST_I2CC_SLAVE_EN | sys_read32(i2c_base + AST_I2CC_FUN_CTRL)
	, i2c_base + AST_I2CC_FUN_CTRL);

	/* apply slave device setting */
	sys_write32(cmd, i2c_base + AST_I2CS_CMD_STS);

	data->slave_attached++;

	return 0;
}

static int i2c_aspeed_slave_unregister(const struct device *dev,
				       struct i2c_target_config *config)
{
	struct i2c_aspeed_data *data = dev->data;
	uint32_t i2c_base = DEV_BASE(dev);
	uint32_t slave_addr = sys_read32(i2c_base + AST_I2CS_ADDR_CTRL);
	bool slave_found = false;
	uint8_t i;

	/* check slave config input */
	if (!config || data->slave_attached == 0) {
		return -EINVAL;
	}

	/* remove the slave call back from array */
	for (i = 0; i < I2C_SLAVE_COUNT; i++) {
		if (data->slave_cfg[i]) {
			if (data->slave_cfg[i]->address == config->address) {
				slave_found = true;
				data->slave_cfg[i] = NULL;
				LOG_DBG("remove [%x] from %d\n", config->address, i);

				/* remove slave addr by index */
				switch (i) {
				case 0:
					slave_addr &= ~(AST_I2CS_ADDR1_MASK);
					break;
				case 1:
					slave_addr &= ~(AST_I2CS_ADDR2_MASK);
					break;
				case 2:
					slave_addr &= ~(AST_I2CS_ADDR3_MASK);
					break;
				}

				LOG_DBG("un-reg slave_addr [%x]\n", slave_addr);
				sys_write32(slave_addr, i2c_base + AST_I2CS_ADDR_CTRL);
				break;
			}
		}
	}

	/* don't find slave to remove */
	if (!slave_found)
		return -EINVAL;

	data->slave_attached--;
	if (data->slave_attached == 0x0) {
		/*Turn off slave mode.*/
		sys_write32(~AST_I2CC_SLAVE_EN & sys_read32(i2c_base + AST_I2CC_FUN_CTRL)
		, i2c_base + AST_I2CC_FUN_CTRL);
	}

	return 0;
}
#endif

static const struct i2c_driver_api i2c_aspeed_driver_api = {
	.configure = i2c_aspeed_configure,
	.get_config = i2c_aspeed_get_configure,
	.transfer = i2c_aspeed_transfer,
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_aspeed_slave_register,
	.target_unregister = i2c_aspeed_slave_unregister,
#endif

};

#define I2C_ASPEED_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2c_aspeed_config_func_##n(const struct device *dev);                          \
                                                                                                   \
	static const struct i2c_aspeed_config i2c_aspeed_config_##n = {                        \
		.base = DT_INST_REG_ADDR_BY_IDX(n, 0),                                             \
		.buf_base = DT_INST_REG_ADDR_BY_IDX(n, 1),                                         \
		.buf_size = DT_INST_REG_SIZE_BY_IDX(n, 1),                                         \
		.irq_config_func = i2c_aspeed_config_func_##n,                                     \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		.mode = DT_ENUM_IDX(DT_INST(n, DT_DRV_COMPAT), xfer_mode),                         \
		.multi_master = DT_INST_PROP(n, multi_master),                                     \
		.smbus_timeout = DT_INST_PROP(n, smbus_timeout),                                   \
		.manual_scl_high = DT_INST_PROP(n, manual_high_count),                             \
		.manual_scl_low = DT_INST_PROP(n, manual_low_count),                               \
		.manual_sda_hold = DT_INST_PROP(n, manual_sda_delay),                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, clk_id),                  \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
                                                                                                   \
	static struct i2c_aspeed_data i2c_aspeed_data_##n;                                         \
                                                                                                   \
	I2C_DEVICE_DT_INST_DEFINE(n, &i2c_aspeed_init, NULL, &i2c_aspeed_data_##n,                 \
				  &i2c_aspeed_config_##n, POST_KERNEL,                             \
				  CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &i2c_aspeed_driver_api);     \
                                                                                                   \
	static void i2c_aspeed_config_func_##n(const struct device *dev)                           \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2c_aspeed_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
                                                                                                   \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_ASPEED_INIT)
