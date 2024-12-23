/*
 * Copyright (c) 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT aspeed_sdhci

#include <zephyr/kernel.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>

/* Bit map for command Register */
#define SDHCI_HOST_CMD_RESP_TYPE_LOC	0
#define SDHCI_HOST_CMD_CRC_CHECK_EN_LOC 3
#define SDHCI_HOST_CMD_IDX_CHECK_EN_LOC 4
#define SDHCI_HOST_CMD_DATA_PRESENT_LOC 5
#define SDHCI_HOST_CMD_TYPE_LOC		6
#define SDHCI_HOST_CMD_INDEX_LOC	8

/* Bit map for Transfer Mode Register */
#define SDHCI_HOST_XFER_DMA_EN_LOC	    0
#define SDHCI_HOST_XFER_BLOCK_CNT_EN_LOC    1
#define SDHCI_HOST_XFER_AUTO_CMD_EN_LOC     2
#define SDHCI_HOST_XFER_DATA_DIR_LOC	    4
#define SDHCI_HOST_XFER_MULTI_BLOCK_SEL_LOC 5

#define SDHCI_HOST_XFER_DMA_EN_MASK	     0x01
#define SDHCI_HOST_XFER_BLOCK_CNT_EN_MASK    0x01
#define SDHCI_HOST_XFER_AUTO_CMD_EN_MASK     0x03
#define SDHCI_HOST_XFER_DATA_DIR_MASK	     0x01
#define SDHCI_HOST_XFER_MULTI_BLOCK_SEL_MASK 0x01

/* Bit map for Block Size and GAP Register */
#define SDHCI_HOST_BLOCK_SIZE_LOC    0
#define SDHCI_HOST_BLOCK_SIZE_MASK   0xFFF
#define SDHCI_HOST_DMA_BUF_SIZE_LOC  12
#define SDHCI_HOST_DMA_BUF_SIZE_MASK 0x07
#define SDHCI_HOST_BLOCK_GAP_LOC     3
#define SDHCI_HOST_BLOCK_GAP_MASK    0x01

#define SDHCI_HOST_ADMA_BUFF_ADD_LOC   32
#define SDHCI_HOST_ADMA_BUFF_LEN_LOC   16
#define SDHCI_HOST_ADMA_BUFF_LINK_NEXT (0x3 << 4)
#define SDHCI_HOST_ADMA_BUFF_LINK_LAST (0x2 << 4)
#define SDHCI_HOST_ADMA_INTR_EN        BIT(2)
#define SDHCI_HOST_ADMA_BUFF_LAST      BIT(1)
#define SDHCI_HOST_ADMA_BUFF_VALID     BIT(0)

/* Bit Map and length details for Clock Control Register */
#define SDHCI_HOST_CLK_SDCLCK_FREQ_SEL_LOC	 8
#define SDHCI_HOST_CLK_SDCLCK_FREQ_SEL_UPPER_LOC 6

#define SDHCI_HOST_CLK_SDCLCK_FREQ_SEL_MASK	  0xFF
#define SDHCI_HOST_CLK_SDCLCK_FREQ_SEL_UPPER_MASK 0x03

/* Bit Map for Host Control 1 Register */
#define SDHCI_HOST_CTRL1_DAT_WIDTH_LOC	   1
#define SDHCI_HOST_CTRL1_HISPEED	   2
#define SDHCI_HOST_CTRL1_DMA_SEL_LOC	   3
#define SDHCI_HOST_CTRL1_EXT_DAT_WIDTH_LOC 5

#define SDHCI_HOST_CTRL1_DMA_SEL_MASK	    0x03
#define SDHCI_HOST_CTRL1_EXT_DAT_WIDTH_MASK 0x01
#define SDHCI_HOST_CTRL1_DAT_WIDTH_MASK     0x01

/** Constants Software Reset register */
#define SDHCI_HOST_SW_RESET_REG_ALL  BIT(0)
#define SDHCI_HOST_SW_RESET_REG_CMD  BIT(1)
#define SDHCI_HOST_SW_RESET_REG_DATA BIT(2)

#define SDHCI_HOST_RESPONSE_SIZE      4
#define SDHCI_HOST_OCR_BUSY_BIT       BIT(31)
#define SDHCI_HOST_OCR_CAPACITY_MASK  0x40000000U
#define SDHCI_HOST_DUAL_VOLTAGE_RANGE 0x40FF8080U
#define SDHCI_HOST_BLOCK_SIZE	      512

#define SDHCI_HOST_RCA_SHIFT		    16
#define SDHCI_HOST_EXTCSD_SEC_COUNT	    53
#define SDHCI_HOST_EXTCSD_GENERIC_CMD6_TIME 62
#define SDHCI_HOST_EXTCSD_BUS_WIDTH_ADDR    0xB7
#define SDHCI_HOST_EXTCSD_HS_TIMING_ADDR    0xB9
#define SDHCI_HOST_BUS_SPEED_HIGHSPEED	    1

#define SDHCI_HOST_CMD_COMPLETE_RETRY 10000
#define SDHCI_HOST_XFR_COMPLETE_RETRY 2000000

#define SDHCI_HOST_CMD1_RETRY_TIMEOUT 1000
#define SDHCI_HOST_CMD6_TIMEOUT_MULT  10

#define SDHCI_HOST_NORMAL_INTR_MASK	0x3f
#define SDHCI_HOST_ERROR_INTR_MASK	0x13ff
#define SDHCI_HOST_NORMAL_INTR_MASK_CLR 0x60ff

#define SDHCI_HOST_POWER_CTRL_SD_BUS_POWER    0x1
#define SDHCI_HOST_POWER_CTRL_SD_BUS_VOLT_SEL 0x5

#define SDHCI_HOST_UHSMODE_SDR12  0x0
#define SDHCI_HOST_UHSMODE_SDR25  0x1
#define SDHCI_HOST_UHSMODE_SDR50  0x2
#define SDHCI_HOST_UHSMODE_SDR104 0x3
#define SDHCI_HOST_UHSMODE_DDR50  0x4
#define SDHCI_HOST_UHSMODE_HS400  0x5

#define SDHCI_HOST_CTRL2_1P8V_SIG_EN	   1
#define SDHCI_HOST_CTRL2_1P8V_SIG_LOC	   3
#define SDHCI_HOST_CTRL2_UHS_MODE_SEL_LOC  0
#define SDHCI_HOST_CTRL2_UHS_MODE_SEL_MASK 0x07

/* Event/command status */
#define SDHCI_HOST_CMD_COMPLETE   BIT(0)
#define SDHCI_HOST_XFER_COMPLETE  BIT(1)
#define SDHCI_HOST_BLOCK_GAP_INTR BIT(2)
#define SDHCI_HOST_DMA_INTR	  BIT(3)
#define SDHCI_HOST_BUF_WR_READY   BIT(4)
#define SDHCI_HOST_BUF_RD_READY   BIT(5)

#define SDHCI_HOST_CMD_TIMEOUT_ERR  BIT(0)
#define SDHCI_HOST_CMD_CRC_ERR	    BIT(1)
#define SDHCI_HOST_CMD_END_BIT_ERR  BIT(2)
#define SDHCI_HOST_CMD_IDX_ERR	    BIT(3)
#define SDHCI_HOST_DATA_TIMEOUT_ERR BIT(4)
#define SDHCI_HOST_DATA_CRC_ERR     BIT(5)
#define SDHCI_HOST_DATA_END_BIT_ERR BIT(6)
#define SDHCI_HOST_CUR_LMT_ERR	    BIT(7)
#define SDHCI_HOST_DMA_TXFR_ERR     BIT(12)
#define SDHCI_HOST_ERR_STATUS	    0xFFF

/** PState register bits */
#define SDHCI_HOST_PSTATE_CMD_INHIBIT	  BIT(0)
#define SDHCI_HOST_PSTATE_DAT_INHIBIT	  BIT(1)
#define SDHCI_HOST_PSTATE_DAT_LINE_ACTIVE BIT(2)

#define SDHCI_HOST_PSTATE_WR_DMA_ACTIVE BIT(8)
#define SDHCI_HOST_PSTATE_RD_DMA_ACTIVE BIT(9)

#define SDHCI_HOST_PSTATE_BUF_READ_EN  BIT(11)
#define SDHCI_HOST_PSTATE_BUF_WRITE_EN BIT(10)

#define SDHCI_HOST_PSTATE_CARD_INSERTED BIT(16)

#define SDHCI_HOST_MAX_TIMEOUT 0xe
#define SDHCI_HOST_MSEC_DELAY  1000

/** Constants for Clock Control register */
#define SDHCI_HOST_INTERNAL_CLOCK_EN	 BIT(0)
#define SDHCI_HOST_INTERNAL_CLOCK_STABLE BIT(1)
#define SDHCI_HOST_SD_CLOCK_EN		 BIT(2)

/** Clock frequency */
#define SDHCI_HOST_CLK_FREQ_400K 0.4
#define SDHCI_HOST_CLK_FREQ_25M  25
#define SDHCI_HOST_CLK_FREQ_50M  50
#define SDHCI_HOST_CLK_FREQ_100M 100
#define SDHCI_HOST_CLK_FREQ_200M 200

#define SDHCI_HOST_TUNING_SUCCESS BIT(7)
#define SDHCI_HOST_START_TUNING   BIT(6)

#define SDHCI_HOST_VOL_3_3_V_SUPPORT BIT(24)
#define SDHCI_HOST_VOL_3_3_V_SELECT  (7 << 1)
#define SDHCI_HOST_VOL_3_0_V_SUPPORT BIT(25)
#define SDHCI_HOST_VOL_3_0_V_SELECT  (6 << 1)
#define SDHCI_HOST_VOL_1_8_V_SUPPORT BIT(26)
#define SDHCI_HOST_VOL_1_8_V_SELECT  (5 << 1)

#define SDHCI_HOST_CMD_WAIT_TIMEOUT_US	  3000
#define SDHCI_HOST_CMD_CMPLETE_TIMEOUT_US 9000
#define SDHCI_HOST_XFR_CMPLETE_TIMEOUT_US 1000
#define SDHCI_HOST_SDMA_BOUNDARY	  0x0
#define SDHCI_HOST_RCA_ADDRESS		  0x2

#define SDHCI_HOST_RESP_MASK (0xFF000000U)

#define SDHCI_HOST_SET_RESP(resp0, resp1) ((resp0) >> 1) | (((resp1) & 1) << 30)

#define SET_BITS(reg, pos, bit_width, val)	\
	(reg) &= ~((bit_width) << (pos));	\
	(reg) |= (((val) & (bit_width)) << (pos))

/* get value from certain bit
 */
#define GET_BITS(reg_name, start, width) ((reg_name) & (((1 << (width)) - 1) << (start)))

#define ERR_INTR_STATUS_EVENT(reg_bits) ((reg_bits) << 16)

#define ADDRESS_32BIT_MASK 0xFFFFFFFF

enum sdhci_sw_reset {
	SDHCI_HOST_SW_RESET_DATA_LINE = 0,
	SDHCI_HOST_SW_RESET_CMD_LINE,
	SDHCI_HOST_SW_RESET_ALL
};

enum sdhci_cmd_type {
	SDHCI_HOST_CMD_NORMAL = 0,
	SDHCI_HOST_CMD_SUSPEND,
	SDHCI_HOST_CMD_RESUME,
	SDHCI_HOST_CMD_ABORT,
};

enum sdhci_response_type {
	SDHCI_HOST_RESP_NONE = 0,
	SDHCI_HOST_RESP_LEN_136,
	SDHCI_HOST_RESP_LEN_48,
	SDHCI_HOST_RESP_LEN_48B,
	SDHCI_HOST_INVAL_HOST_RESP_LEN,
};

struct sdhci_cmd_config {
	struct sdhc_command *sdhc_cmd;
	uint32_t cmd_idx;
	enum sdhci_cmd_type cmd_type;
	bool data_present;
	bool idx_check_en;
	bool crc_check_en;
};

struct sdhci_data {
	DEVICE_MMIO_RAM;
	uint32_t rca;
	struct sdhc_io host_io;
	struct k_sem lock;
	struct sdhc_host_props props;
	bool card_present;
};

struct sdhci_reg {
	volatile uint32_t sdma_sysaddr;  /**< SDMA System Address */
	volatile uint16_t block_size;	 /**< Block Size */
	volatile uint16_t block_count;	 /**< Block Count */
	volatile uint32_t argument;	 /**< Argument */
	volatile uint16_t transfer_mode; /**< Transfer Mode */
	volatile uint16_t cmd;		 /**< Command */

	volatile uint32_t resp_01;		/**< Response Register 0 & 1 */
	volatile uint16_t resp_2;		/**< Response Register 2*/
	volatile uint16_t resp_3;		/**< Response Register 3 */
	volatile uint16_t resp_4;		/**< Response Register 4 */
	volatile uint16_t resp_5;		/**< Response Register 5 */
	volatile uint16_t resp_6;		/**< Response Register 6 */
	volatile uint16_t resp_7;		/**< Response Register 7 */
	volatile uint32_t data_port;		/**< Buffer Data Port */
	volatile uint32_t present_state;	/**< Present State */
	volatile uint8_t host_ctrl1;		/**< Host Control 1 */
	volatile uint8_t power_ctrl;		/**< Power Control */
	volatile uint8_t block_gap_ctrl;	/**< Block Gap Control */
	volatile uint8_t wake_up_ctrl;		/**< Wakeup Control */
	volatile uint16_t clock_ctrl;		/**< Clock Control */
	volatile uint8_t timeout_ctrl;		/**< Timeout Control */
	volatile uint8_t sw_reset;		/**< Software Reset */
	volatile uint16_t normal_int_stat;	/**< Normal Interrupt Status */
	volatile uint16_t err_int_stat;		/**< Error Interrupt Status */
	volatile uint16_t normal_int_stat_en;	/**< Normal Interrupt Status Enable */
	volatile uint16_t err_int_stat_en;	/**< Error Interrupt Status Enable */
	volatile uint16_t normal_int_signal_en; /**< Normal Interrupt Signal Enable */
	volatile uint16_t err_int_signal_en;	/**< Error Interrupt Signal Enable */
	volatile uint16_t auto_cmd_err_stat;	/**< Auto CMD Error Status */
	volatile uint16_t host_ctrl2;		/**< Host Control 2 */
	volatile uint64_t capabilities;		/**< Capabilities */

	volatile uint64_t max_current_cap;	  /**< Max Current Capabilities */
	volatile uint16_t force_err_autocmd_stat; /**< Force Event for Auto CMD Error Status*/
	volatile uint16_t force_err_int_stat;	  /**< Force Event for Error Interrupt Status */
	volatile uint8_t adma_err_stat;		  /**< ADMA Error Status */
	volatile uint8_t reserved[3];
	volatile uint32_t adma_sys_addr1; /**< ADMA System Address1 */
	volatile uint32_t adma_sys_addr2; /**< ADMA System Address2 */
};

LOG_MODULE_REGISTER(aspeed_sdhci, CONFIG_SDHC_LOG_LEVEL);

struct aspeed_sdhci_config {
	uintptr_t base;
	const struct device *clock_dev;
	const clock_control_subsys_t clk_id;
	const struct reset_dt_spec reset;
};

static int sdhci_set_voltage(const struct device *dev, enum sd_voltage signal_voltage)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	bool power_state = regs->power_ctrl & SDHCI_HOST_POWER_CTRL_SD_BUS_POWER ? true : false;
	int ret = 0;

	if (power_state) {
		/* Turn OFF Bus Power before config clock */
		regs->power_ctrl &= ~SDHCI_HOST_POWER_CTRL_SD_BUS_POWER;
	}

	switch (signal_voltage) {
	case SD_VOL_3_3_V:
		if (regs->capabilities & SDHCI_HOST_VOL_3_3_V_SUPPORT) {
			regs->host_ctrl2 &=
				~(SDHCI_HOST_CTRL2_1P8V_SIG_EN << SDHCI_HOST_CTRL2_1P8V_SIG_LOC);

			/* 3.3v voltage select */
			regs->power_ctrl = SDHCI_HOST_VOL_3_3_V_SELECT;
			LOG_DBG("3.3V Selected for MMC Card");
		} else {
			LOG_ERR("3.3V not supported by MMC Host");
			ret = -ENOTSUP;
		}
		break;

	case SD_VOL_3_0_V:
		if (regs->capabilities & SDHCI_HOST_VOL_3_0_V_SUPPORT) {
			regs->host_ctrl2 &=
				~(SDHCI_HOST_CTRL2_1P8V_SIG_EN << SDHCI_HOST_CTRL2_1P8V_SIG_LOC);

			/* 3.0v voltage select */
			regs->power_ctrl = SDHCI_HOST_VOL_3_0_V_SELECT;
			LOG_DBG("3.0V Selected for MMC Card");
		} else {
			LOG_ERR("3.0V not supported by MMC Host");
			ret = -ENOTSUP;
		}
		break;

	case SD_VOL_1_8_V:
		if (regs->capabilities & SDHCI_HOST_VOL_1_8_V_SUPPORT) {
			regs->host_ctrl2 |= SDHCI_HOST_CTRL2_1P8V_SIG_EN
					    << SDHCI_HOST_CTRL2_1P8V_SIG_LOC;

			/* 1.8v voltage select */
			regs->power_ctrl = SDHCI_HOST_VOL_1_8_V_SELECT;
			LOG_DBG("1.8V Selected for MMC Card");
		} else {
			LOG_ERR("1.8V not supported by MMC Host");
			ret = -ENOTSUP;
		}
		break;

	default:
		ret = -EINVAL;
	}

	if (power_state) {
		/* Turn ON Bus Power */
		regs->power_ctrl |= SDHCI_HOST_POWER_CTRL_SD_BUS_POWER;
	}

	return ret;
}

static int sdhci_set_power(const struct device *dev, enum sdhc_power state)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);

	if (state == SDHC_POWER_ON) {
		/* Turn ON Bus Power */
		regs->power_ctrl |= SDHCI_HOST_POWER_CTRL_SD_BUS_POWER;
	} else {
		/* Turn OFF Bus Power */
		regs->power_ctrl &= ~SDHCI_HOST_POWER_CTRL_SD_BUS_POWER;
	}

	k_busy_wait(10u);

	return 0;
}

static bool sdhci_disable_clock(const struct device *dev)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);

	if (regs->present_state & SDHCI_HOST_PSTATE_CMD_INHIBIT) {
		LOG_ERR("present_state:%x", regs->present_state);
		return false;
	}
	if (regs->present_state & SDHCI_HOST_PSTATE_DAT_INHIBIT) {
		LOG_ERR("present_state:%x", regs->present_state);
		return false;
	}

	regs->clock_ctrl &= ~SDHCI_HOST_INTERNAL_CLOCK_EN;
	regs->clock_ctrl &= ~SDHCI_HOST_SD_CLOCK_EN;

	while ((regs->clock_ctrl & SDHCI_HOST_SD_CLOCK_EN) != 0) {
		;
	}

	return true;
}

static bool sdhci_enable_clock(const struct device *dev)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);

	regs->clock_ctrl |= SDHCI_HOST_INTERNAL_CLOCK_EN;
	/* Wait for the stable Internal Clock */
	while ((regs->clock_ctrl & SDHCI_HOST_INTERNAL_CLOCK_STABLE) == 0) {
		;
	}

	/* Enable SD Clock */
	regs->clock_ctrl |= SDHCI_HOST_SD_CLOCK_EN;
	while ((regs->clock_ctrl & SDHCI_HOST_SD_CLOCK_EN) == 0) {
		;
	}

	return true;
}

static bool sdhci_clock_set(const struct device *dev, enum sdhc_clock_speed speed)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	uint8_t base_freq;
	uint32_t clock_divider;
	float freq;
	bool ret;

	switch (speed) {
	case SDMMC_CLOCK_400KHZ:
		freq = SDHCI_HOST_CLK_FREQ_400K;
		break;

	case SD_CLOCK_25MHZ:
	case MMC_CLOCK_26MHZ:
		freq = SDHCI_HOST_CLK_FREQ_25M;
		break;

	case SD_CLOCK_50MHZ:
	case MMC_CLOCK_52MHZ:
		freq = SDHCI_HOST_CLK_FREQ_50M;
		break;

	case SD_CLOCK_100MHZ:
		freq = SDHCI_HOST_CLK_FREQ_100M;
		break;

	case MMC_CLOCK_HS200:
		freq = SDHCI_HOST_CLK_FREQ_200M;
		break;

	case SD_CLOCK_208MHZ:
	default:
		return false;
	}

	ret = sdhci_disable_clock(dev);
	if (!ret) {
		return false;
	}

	base_freq = 100;
	LOG_DBG("base freq=0x%x", base_freq);

	clock_divider = (int)(base_freq / (freq * 2));

	LOG_DBG("Clock divider for MMC Clk: %d Hz is %d", speed, clock_divider);

	SET_BITS(regs->clock_ctrl, SDHCI_HOST_CLK_SDCLCK_FREQ_SEL_LOC,
		 SDHCI_HOST_CLK_SDCLCK_FREQ_SEL_MASK, clock_divider);
	SET_BITS(regs->clock_ctrl, SDHCI_HOST_CLK_SDCLCK_FREQ_SEL_UPPER_LOC,
		 SDHCI_HOST_CLK_SDCLCK_FREQ_SEL_UPPER_MASK, clock_divider >> 8);

	sdhci_enable_clock(dev);

	return true;
}

static int set_timing(const struct device *dev, enum sdhc_timing_mode timing)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	int ret = 0;
	uint8_t mode = 0;

	LOG_DBG("Timing Mode: %d", timing);

	switch (timing) {
	case SDHC_TIMING_LEGACY:
		break;
	case SDHC_TIMING_HS:
		SET_BITS(regs->host_ctrl1, SDHCI_HOST_CTRL1_HISPEED, 1, 1);
		break;
	case SDHC_TIMING_SDR12:
		mode = SDHCI_HOST_UHSMODE_SDR12;
		break;

	case SDHC_TIMING_SDR25:
		mode = SDHCI_HOST_UHSMODE_SDR25;
		break;

	case SDHC_TIMING_SDR50:
		mode = SDHCI_HOST_UHSMODE_SDR50;
		break;

	case SDHC_TIMING_SDR104:
		mode = SDHCI_HOST_UHSMODE_SDR104;
		break;

	case SDHC_TIMING_DDR50:
	case SDHC_TIMING_DDR52:
		mode = SDHCI_HOST_UHSMODE_DDR50;
		break;

	case SDHC_TIMING_HS400:
	case SDHC_TIMING_HS200:
		mode = SDHCI_HOST_UHSMODE_HS400;
		break;

	default:
		ret = -ENOTSUP;
	}

	if (!ret) {
		if (!sdhci_disable_clock(dev)) {
			LOG_ERR("Disable clk failed");
			return -EIO;
		}

		if (timing > SDHC_TIMING_HS) {
			regs->host_ctrl2 |=
				SDHCI_HOST_CTRL2_1P8V_SIG_EN << SDHCI_HOST_CTRL2_1P8V_SIG_LOC;
			SET_BITS(regs->host_ctrl2, SDHCI_HOST_CTRL2_UHS_MODE_SEL_LOC,
				 SDHCI_HOST_CTRL2_UHS_MODE_SEL_MASK, mode);
		}

		sdhci_enable_clock(dev);
	}

	return ret;
}

static void update_cmd_response(const struct device *dev, struct sdhc_command *sdhc_cmd)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	uint32_t resp0, resp1, resp2, resp3;

	if (sdhc_cmd->response_type == SD_RSP_TYPE_NONE) {
		return;
	}

	resp0 = regs->resp_01;

	if (sdhc_cmd->response_type == SD_RSP_TYPE_R2) {
		resp1 = regs->resp_2 | (regs->resp_3 << 16u);
		resp2 = regs->resp_4 | (regs->resp_5 << 16u);
		resp3 = regs->resp_6 | (regs->resp_7 << 16u);

		LOG_DBG("cmd resp: %x %x %x %x", resp0, resp1, resp2, resp3);

		sdhc_cmd->response[0u] = resp3;
		sdhc_cmd->response[1U] = resp2;
		sdhc_cmd->response[2U] = resp1;
		sdhc_cmd->response[3U] = resp0;
	} else {
		LOG_DBG("cmd resp: %x", resp0);
		sdhc_cmd->response[0u] = resp0;
	}
}

static int poll_cmd_complete(const struct device *dev, uint32_t time_out)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	int ret = -EAGAIN;
	int32_t retry = time_out;

	while (retry > 0) {
		if (regs->normal_int_stat & SDHCI_HOST_CMD_COMPLETE) {
			regs->normal_int_stat = SDHCI_HOST_CMD_COMPLETE;
			ret = 0;
			break;
		}

		k_busy_wait(1000u);
		retry--;
	}

	if (regs->err_int_stat) {
		LOG_ERR("err_int_stat:%x", regs->err_int_stat);
		regs->err_int_stat &= regs->err_int_stat;
		ret = -EIO;
	}

	return ret;
}

static enum sdhci_response_type sdhci_decode_resp_type(enum sd_rsp_type type)
{
	enum sdhci_response_type resp_type;

	switch (type & 0xF) {
	case SD_RSP_TYPE_NONE:
		resp_type = SDHCI_HOST_RESP_NONE;
		break;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
	case SD_RSP_TYPE_R5:
		resp_type = SDHCI_HOST_RESP_LEN_48;
		break;
	case SD_RSP_TYPE_R1b:
		resp_type = SDHCI_HOST_RESP_LEN_48B;
		break;
	case SD_RSP_TYPE_R2:
		resp_type = SDHCI_HOST_RESP_LEN_136;
		break;

	case SD_RSP_TYPE_R5b:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
	default:
		resp_type = SDHCI_HOST_INVAL_HOST_RESP_LEN;
	}

	return resp_type;
}

static int sdhci_send_cmd(const struct device *dev, const struct sdhci_cmd_config *config)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	struct sdhc_command *sdhc_cmd = config->sdhc_cmd;
	enum sdhci_response_type resp_type = sdhci_decode_resp_type(sdhc_cmd->response_type);
	uint16_t cmd_reg;
	int ret;

	LOG_DBG("");

	/* Check if CMD line is available */
	if (regs->present_state & SDHCI_HOST_PSTATE_CMD_INHIBIT) {
		LOG_ERR("CMD line is not available");
		return -EBUSY;
	}

	if (config->data_present && (regs->present_state & SDHCI_HOST_PSTATE_DAT_INHIBIT)) {
		LOG_ERR("Data line is not available");
		return -EBUSY;
	}

	if (resp_type == SDHCI_HOST_INVAL_HOST_RESP_LEN) {
		LOG_ERR("Invalid eMMC resp type:%d", resp_type);
		return -EINVAL;
	}

	regs->argument = sdhc_cmd->arg;

	cmd_reg = config->cmd_idx << SDHCI_HOST_CMD_INDEX_LOC |
		  config->cmd_type << SDHCI_HOST_CMD_TYPE_LOC |
		  config->data_present << SDHCI_HOST_CMD_DATA_PRESENT_LOC |
		  config->idx_check_en << SDHCI_HOST_CMD_IDX_CHECK_EN_LOC |
		  config->crc_check_en << SDHCI_HOST_CMD_CRC_CHECK_EN_LOC |
		  resp_type << SDHCI_HOST_CMD_RESP_TYPE_LOC;
	regs->cmd = cmd_reg;

	LOG_DBG("CMD REG:%x %x", cmd_reg, regs->cmd);

	ret = poll_cmd_complete(dev, sdhc_cmd->timeout_ms);
	if (ret) {
		LOG_ERR("Error on send cmd: %d, status:%d", config->cmd_idx, ret);
		return ret;
	}

	update_cmd_response(dev, sdhc_cmd);

	return 0;
}

static int sdhci_send_cmd_no_data(const struct device *dev, uint32_t cmd_idx,
				     struct sdhc_command *cmd)
{
	struct sdhci_cmd_config sdhci_cmd;

	sdhci_cmd.sdhc_cmd = cmd;
	sdhci_cmd.cmd_idx = cmd_idx;
	sdhci_cmd.cmd_type = SDHCI_HOST_CMD_NORMAL;
	sdhci_cmd.data_present = false;
	sdhci_cmd.idx_check_en = false;
	sdhci_cmd.crc_check_en = false;

	return sdhci_send_cmd(dev, &sdhci_cmd);
}

static int sdhci_init_xfr(const struct device *dev, struct sdhc_data *data, bool read)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	uint16_t multi_block = 0u;

	/* Set Block Size Register */
	SET_BITS(regs->block_size, SDHCI_HOST_DMA_BUF_SIZE_LOC, SDHCI_HOST_DMA_BUF_SIZE_MASK,
		 SDHCI_HOST_SDMA_BOUNDARY);
	SET_BITS(regs->block_size, SDHCI_HOST_BLOCK_SIZE_LOC, SDHCI_HOST_BLOCK_SIZE_MASK,
		 data->block_size);

	if (data->blocks > 1)
		multi_block = 1u;

	/* Disable auto cmd */
	SET_BITS(regs->transfer_mode, SDHCI_HOST_XFER_AUTO_CMD_EN_LOC,
		 SDHCI_HOST_XFER_AUTO_CMD_EN_MASK, 0);

	/* Set block count */
	regs->block_count = (uint16_t)data->blocks;

	/* Enable block count in transfer register */
	SET_BITS(regs->transfer_mode, SDHCI_HOST_XFER_BLOCK_CNT_EN_LOC,
		 SDHCI_HOST_XFER_BLOCK_CNT_EN_MASK, multi_block ? 1 : 0);

	/* Set multi or single block */
	SET_BITS(regs->transfer_mode, SDHCI_HOST_XFER_MULTI_BLOCK_SEL_LOC,
		 SDHCI_HOST_XFER_MULTI_BLOCK_SEL_MASK, multi_block);

	/* Set data transfer direction, Read = 1, Write = 0 */
	SET_BITS(regs->transfer_mode, SDHCI_HOST_XFER_DATA_DIR_LOC, SDHCI_HOST_XFER_DATA_DIR_MASK,
		 read ? 1u : 0u);

	/* Disable DMA */
	SET_BITS(regs->transfer_mode, SDHCI_HOST_XFER_DMA_EN_LOC, SDHCI_HOST_XFER_DMA_EN_MASK,
		 0u);

	SET_BITS(regs->block_gap_ctrl, SDHCI_HOST_BLOCK_GAP_LOC, SDHCI_HOST_BLOCK_GAP_MASK,
		 0u);

	/* Set data timeout time */
	regs->timeout_ctrl = 0xe;

	return 0;
}

static int wait_xfr_poll_complete(const struct device *dev, uint32_t time_out)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	int ret = -EAGAIN;
	int32_t retry = time_out;

	LOG_DBG("");

	while (retry > 0) {
		if (regs->normal_int_stat & SDHCI_HOST_XFER_COMPLETE) {
			regs->normal_int_stat |= SDHCI_HOST_XFER_COMPLETE;
			ret = 0;
			break;
		}

		k_busy_wait(SDHCI_HOST_MSEC_DELAY);
		retry--;
	}

	return ret;
}

static int wait_xfr_complete(const struct device *dev, uint32_t time_out)
{
	int ret;

	ret = wait_xfr_poll_complete(dev, time_out);

	return ret;
}

static int sdhci_stop_transfer(const struct device *dev)
{
	struct sdhci_data *sdhci = dev->data;
	struct sdhc_command hdc_cmd = {0};
	struct sdhci_cmd_config cmd;

	hdc_cmd.arg = sdhci->rca << SDHCI_HOST_RCA_SHIFT;
	hdc_cmd.response_type = SD_RSP_TYPE_R1;
	hdc_cmd.timeout_ms = 1000;

	cmd.sdhc_cmd = &hdc_cmd;
	cmd.cmd_idx = SD_STOP_TRANSMISSION;
	cmd.cmd_type = SDHCI_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = false;
	cmd.crc_check_en = false;

	return sdhci_send_cmd(dev, &cmd);
}

void sdhci_sw_reset(const struct device *dev, enum sdhci_sw_reset reset)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);

	if (reset == SDHCI_HOST_SW_RESET_DATA_LINE) {
		regs->sw_reset = SDHCI_HOST_SW_RESET_REG_DATA;
	} else if (reset == SDHCI_HOST_SW_RESET_CMD_LINE) {
		regs->sw_reset = SDHCI_HOST_SW_RESET_REG_CMD;
	} else if (reset == SDHCI_HOST_SW_RESET_ALL) {
		regs->sw_reset = SDHCI_HOST_SW_RESET_REG_ALL;
	}

	while (regs->sw_reset != 0) {
		;
	}

	k_busy_wait(1000);
}

static void disable_interrupts(const struct device *dev)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);

	/* Keep enable interrupt status register to update */
	regs->normal_int_stat_en = SDHCI_HOST_NORMAL_INTR_MASK;
	regs->err_int_stat_en = SDHCI_HOST_ERROR_INTR_MASK;

	/* Disable only interrupt generation */
	regs->normal_int_signal_en &= 0;
	regs->err_int_signal_en &= 0;
	regs->timeout_ctrl = SDHCI_HOST_MAX_TIMEOUT;
}

static void clear_interrupts(const struct device *dev)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);

	regs->normal_int_stat = SDHCI_HOST_NORMAL_INTR_MASK_CLR;
	regs->err_int_stat = SDHCI_HOST_ERROR_INTR_MASK;
}

static int sdhci_reset(const struct device *dev)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);

	LOG_DBG("");

	if (!(regs->present_state & SDHCI_HOST_PSTATE_CARD_INSERTED)) {
		LOG_ERR("No SDHCI card found");
		return -ENODEV;
	}

	/* Reset device to idle state */
	sdhci_sw_reset(dev, SDHCI_HOST_SW_RESET_ALL);

	clear_interrupts(dev);

	disable_interrupts(dev);

	return 0;
}

static int read_data_port(const struct device *dev, struct sdhc_data *sdhc)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	uint32_t block_size = sdhc->block_size;
	uint32_t i, block_cnt = sdhc->blocks;
	uint32_t *data = (uint32_t *)sdhc->data;
	k_timeout_t wait_time;

	if (sdhc->timeout_ms == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(sdhc->timeout_ms);
	}

	LOG_DBG("");

	while (block_cnt--) {
		while ((regs->present_state & SDHCI_HOST_PSTATE_BUF_READ_EN) == 0) {
			;
		}

		if (regs->present_state & SDHCI_HOST_PSTATE_DAT_INHIBIT) {
			for (i = block_size >> 2u; i != 0u; i--) {
				*data = regs->data_port;
				data++;
			}
		}
	}

	return wait_xfr_complete(dev, sdhc->timeout_ms);
}

static int write_data_port(const struct device *dev, struct sdhc_data *sdhc)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	uint32_t block_size = sdhc->block_size;
	uint32_t i, block_cnt = sdhc->blocks;
	uint32_t *data = (uint32_t *)sdhc->data;
	k_timeout_t wait_time;

	if (sdhc->timeout_ms == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(sdhc->timeout_ms);
	}

	LOG_DBG("");

	while (block_cnt--) {
		LOG_DBG("SDHCI_HOST_BUF_WR_READY\n");

		while ((regs->present_state & SDHCI_HOST_PSTATE_BUF_WRITE_EN) == 0) {
			;
		}

		if (regs->present_state & SDHCI_HOST_PSTATE_DAT_INHIBIT) {
			for (i = block_size >> 2u; i != 0u; i--) {
				regs->data_port = *data;
				data++;
			}
		}
	}

	return wait_xfr_complete(dev, sdhc->timeout_ms);
}

static int sdhci_send_cmd_data(const struct device *dev, uint32_t cmd_idx,
				  struct sdhc_command *cmd, struct sdhc_data *data, bool read)
{
	struct sdhci_cmd_config sdhci_cmd;
	int ret;

	sdhci_cmd.sdhc_cmd = cmd;
	sdhci_cmd.cmd_idx = cmd_idx;
	sdhci_cmd.cmd_type = SDHCI_HOST_CMD_NORMAL;
	sdhci_cmd.data_present = true;
	sdhci_cmd.idx_check_en = true;
	sdhci_cmd.crc_check_en = true;

	ret = sdhci_init_xfr(dev, data, read);
	if (ret) {
		LOG_ERR("Error on init xfr");
		return ret;
	}

	ret = sdhci_send_cmd(dev, &sdhci_cmd);
	if (ret)
		return ret;

	if (read) {
		ret = read_data_port(dev, data);
	} else {
		ret = write_data_port(dev, data);
	}

	return ret;
}

static int sdhci_xfr(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data,
		    bool read)
{
	int ret;
	struct sdhci_cmd_config sdhci_cmd;

	ret = sdhci_init_xfr(dev, data, read);
	if (ret) {
		LOG_ERR("error sdhci init xfr");
		return ret;
	}
	sdhci_cmd.sdhc_cmd = cmd;
	sdhci_cmd.cmd_type = SDHCI_HOST_CMD_NORMAL;
	sdhci_cmd.data_present = true;
	sdhci_cmd.idx_check_en = true;
	sdhci_cmd.crc_check_en = true;

	if (data->blocks > 1)
		sdhci_cmd.cmd_idx = read ? SD_READ_MULTIPLE_BLOCK : SD_WRITE_MULTIPLE_BLOCK;
	else
		sdhci_cmd.cmd_idx = read ? SD_READ_SINGLE_BLOCK : SD_WRITE_SINGLE_BLOCK;

	ret = sdhci_send_cmd(dev, &sdhci_cmd);
	if (ret)
		return ret;

	if (read) {
		ret = read_data_port(dev, data);
	} else {
		ret = write_data_port(dev, data);
	}

	if (data->blocks > 1)
		ret = sdhci_stop_transfer(dev);

	return ret;
}

static int sdhci_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data)
{
	int ret = 0;

	LOG_INF("%s, cmd opcode = %d, type=%x, arg=0x%x\n",
		__func__, cmd->opcode, cmd->response_type & 0xf, cmd->arg);

	if (data) {
		switch (cmd->opcode) {
		case SD_WRITE_SINGLE_BLOCK:
		case SD_WRITE_MULTIPLE_BLOCK:
			LOG_DBG("SD_WRITE_SINGLE_BLOCK");
			ret = sdhci_xfr(dev, cmd, data, false);
			break;

		case SD_READ_SINGLE_BLOCK:
		case SD_READ_MULTIPLE_BLOCK:
			LOG_DBG("SD_READ_SINGLE_BLOCK");
			ret = sdhci_xfr(dev, cmd, data, true);
			break;

		case MMC_SEND_EXT_CSD:
			LOG_DBG("SDHCI_HOST_SEND_EXT_CSD");
			ret = sdhci_send_cmd_data(dev, MMC_SEND_EXT_CSD, cmd, data, true);
			break;

		default:
			ret = sdhci_send_cmd_data(dev, cmd->opcode, cmd, data, true);
		}
	} else {
		ret = sdhci_send_cmd_no_data(dev, cmd->opcode, cmd);
	}

	return ret;
}

static int sdhci_get_card_present(const struct device *dev)
{
	struct sdhci_data *sdhci = dev->data;
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);

	LOG_DBG("");

	sdhci->card_present = (bool)((regs->present_state >> 16u) & 1u);

	if (!sdhci->card_present) {
		LOG_ERR("No MMC device detected");
	}

	return ((int)sdhci->card_present);
}

static int sdhci_card_busy(const struct device *dev)
{
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);

	LOG_DBG("");

	if (regs->present_state & 7u) {
		return 1;
	}

	return 0;
}

static int sdhci_get_host_props(const struct device *dev,
	struct sdhc_host_props *props)
{
	struct sdhci_data *sdhci = dev->data;

	memset(props, 0, sizeof(struct sdhc_host_props));
	props->f_min = SDMMC_CLOCK_400KHZ;
	/*
	 * default max speed is 25MHZ, as per SCR register
	 * it will switch accordingly
	 */
	props->f_max = SD_CLOCK_25MHZ;
	props->power_delay = 0;
	props->host_caps.vol_330_support = true;
	props->is_spi = false;

	sdhci->props = *props;

	return 0;
}

static int aspeed_sdhci_init(const struct device *dev)
{
	const struct aspeed_sdhci_config *config = dev->config;
	int ret;

	LOG_DBG("%s\n", __func__);

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	LOG_DBG("%s, reg_base=0x%x", __func__, (uint32_t)(struct sdhci_reg *)DEVICE_MMIO_GET(dev));

	/* assert reset */
	ret = reset_line_assert_dt(&config->reset);
	__ASSERT_NO_MSG(ret == 0);

	/* enable clock */
	ret = clock_control_on(config->clock_dev, config->clk_id);
	__ASSERT_NO_MSG(ret == 0);

	/* release reset */
	ret = reset_line_deassert_dt(&config->reset);
	__ASSERT_NO_MSG(ret == 0);
	ret = sdhci_reset(dev);

	return ret;
}

static int sdhci_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct sdhci_data *sdhci = dev->data;
	volatile struct sdhci_reg *regs = (struct sdhci_reg *)DEVICE_MMIO_GET(dev);
	struct sdhc_io *host_io = &sdhci->host_io;
	int ret;

	LOG_DBG("sdhci I/O: DW %d, Clk %d Hz, card power state %s, voltage %s", ios->bus_width,
		ios->clock, ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
		ios->signal_voltage == SD_VOL_1_8_V ? "1.8V" : "3.3V");

	if (ios->clock && (ios->clock > sdhci->props.f_max || ios->clock < sdhci->props.f_min)) {
		LOG_ERR("Invalid argument for clock freq: %d Support max:%d and Min:%d", ios->clock,
			sdhci->props.f_max, sdhci->props.f_min);
	}

	/* Set HC clock */
	if (host_io->clock != ios->clock) {
		LOG_DBG("Clock: %d", host_io->clock);
		if (ios->clock != 0) {
			/* Enable clock */
			LOG_DBG("CLOCK: %d", ios->clock);
			if (!sdhci_clock_set(dev, ios->clock)) {
				return -ENOTSUP;
			}
		} else {
			sdhci_disable_clock(dev);
		}
		host_io->clock = ios->clock;
	}

	/* Set data width */
	if (host_io->bus_width != ios->bus_width) {
		LOG_DBG("bus_width: %d", host_io->bus_width);

		if (ios->bus_width == SDHC_BUS_WIDTH4BIT) {
			SET_BITS(regs->host_ctrl1, SDHCI_HOST_CTRL1_EXT_DAT_WIDTH_LOC,
				 SDHCI_HOST_CTRL1_EXT_DAT_WIDTH_MASK,
				 ios->bus_width == SDHC_BUS_WIDTH8BIT ? 1 : 0);
		} else {
			SET_BITS(regs->host_ctrl1, SDHCI_HOST_CTRL1_DAT_WIDTH_LOC,
				 SDHCI_HOST_CTRL1_DAT_WIDTH_MASK,
				 ios->bus_width == SDHC_BUS_WIDTH4BIT ? 1 : 0);
		}
		host_io->bus_width = ios->bus_width;
	}

	/* Set HC signal voltage */
	if (ios->signal_voltage != host_io->signal_voltage) {
		LOG_DBG("signal_voltage: %d", ios->signal_voltage);
		ret = sdhci_set_voltage(dev, ios->signal_voltage);
		if (ret) {
			LOG_ERR("Set signal volatge failed:%d", ret);
			return ret;
		}
		host_io->signal_voltage = ios->signal_voltage;
	}

	/* Set card power */
	if (host_io->power_mode != ios->power_mode) {
		LOG_DBG("power_mode: %d", ios->power_mode);

		ret = sdhci_set_power(dev, ios->power_mode);
		if (ret) {
			LOG_ERR("Set Bus power failed:%d", ret);
			return ret;
		}
		host_io->power_mode = ios->power_mode;
	}

	/* Set I/O timing */
	if (host_io->timing != ios->timing) {
		LOG_DBG("timing: %d", ios->timing);

		ret = set_timing(dev, ios->timing);
		if (ret) {
			LOG_ERR("Set timing failed:%d", ret);
			return ret;
		}
		host_io->timing = ios->timing;
	}

	return 0;
}

static struct sdhc_driver_api aspeed_sdhci_api = {
	.request = sdhci_request,
	.set_io = sdhci_set_io,
	.get_host_props = sdhci_get_host_props,
	.get_card_present = sdhci_get_card_present,
	.reset = sdhci_reset,
	.card_busy = sdhci_card_busy,
};

#define ASPEED_SDHCI_INIT(inst)						\
									\
	static const struct aspeed_sdhci_config aspeed_sdhci_config_##inst = {\
		.base = DT_INST_REG_ADDR(inst) + 0x100,			\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),	 \
		.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, clk_id),	\
		.reset = RESET_DT_SPEC_INST_GET(inst), \
	};								\
	static struct sdhci_data aspeed_sdhci_data_##inst = {		\
	};								\
	DEVICE_DT_INST_DEFINE(inst,					\
			&aspeed_sdhci_init,				\
			NULL,						\
			&aspeed_sdhci_data_##inst,			\
			&aspeed_sdhci_config_##inst,			\
			POST_KERNEL,					\
			CONFIG_SDHC_INIT_PRIORITY,			\
			&aspeed_sdhci_api);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_SDHCI_INIT)
