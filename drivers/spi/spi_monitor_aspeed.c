/*
 * Copyright (c) 2021 - 2023 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <sys/types.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <errno.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spim_aspeed, CONFIG_SPI_LOG_LEVEL);
#include <zephyr/drivers/spi_nor.h>
#include "spi_context.h"
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/misc/aspeed/pfr_aspeed.h>
#include <zephyr/drivers/misc/aspeed/ast2700_spim.h>
#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>

#define CMD_TABLE_VALUE(G, W, R, M, DAT_MODE, DUMMY, PROG_SZ, ADDR_LEN, ADDR_MODE, CMD) \
	((G) << 29 | (W) << 28 | (R) << 27 | (M) << 26 | (DAT_MODE) << 24 |	\
	(DUMMY) << 16 |	(PROG_SZ) << 13 | (ADDR_LEN) << 10 | (ADDR_MODE) << 8 | (CMD))

static struct cmd_table_info cmds_array[] = {
	{.cmd = CMD_READ_1_1_1_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 1, 0, 0, 3, 1, CMD_READ_1_1_1_3B)},
	{.cmd = CMD_READ_1_1_1_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 1, 0, 0, 4, 1, CMD_READ_1_1_1_4B)},
	{.cmd = CMD_FREAD_1_1_1_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 1, 8, 0, 3, 1, CMD_FREAD_1_1_1_3B)},
	{.cmd = CMD_FREAD_1_1_1_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 1, 8, 0, 4, 1, CMD_FREAD_1_1_1_4B)},
	{.cmd = CMD_READ_1_1_2_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 2, 8, 0, 3, 1, CMD_READ_1_1_2_3B)},
	{.cmd = CMD_READ_1_1_2_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 2, 8, 0, 4, 1, CMD_READ_1_1_2_4B)},
	{.cmd = CMD_READ_1_2_2_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 2, 4, 0, 3, 2, CMD_READ_1_2_2_3B)},
	{.cmd = CMD_READ_1_2_2_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 2, 4, 0, 4, 2, CMD_READ_1_2_2_4B)},
	{.cmd = CMD_READ_1_1_4_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 3, 8, 0, 3, 1, CMD_READ_1_1_4_3B)},
	{.cmd = CMD_READ_1_1_4_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 3, 8, 0, 4, 1, CMD_READ_1_1_4_4B)},
	{.cmd = CMD_READ_1_4_4_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 3, 6, 0, 3, 3, CMD_READ_1_4_4_3B)},
	{.cmd = CMD_READ_1_4_4_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 1, 3, 6, 0, 4, 3, CMD_READ_1_4_4_4B)},
	{.cmd = CMD_PP_1_1_1_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 1, 1, 0, 1, 3, 1, CMD_PP_1_1_1_3B)},
	{.cmd = CMD_PP_1_1_1_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 1, 1, 0, 1, 4, 1, CMD_PP_1_1_1_4B)},
	{.cmd = CMD_PP_1_1_4_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 1, 3, 0, 1, 3, 1, CMD_PP_1_1_4_3B)},
	{.cmd = CMD_PP_1_1_4_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 1, 3, 0, 1, 4, 1, CMD_PP_1_1_4_4B)},
	{.cmd = CMD_SE_1_1_0_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 1, 0, 0, 1, 3, 1, CMD_SE_1_1_0_3B)},
	{.cmd = CMD_SE_1_1_0_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 1, 0, 0, 1, 4, 1, CMD_SE_1_1_0_4B)},
	{.cmd = CMD_SE_1_1_0_64_3B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 1, 0, 0, 5, 3, 1, CMD_SE_1_1_0_64_3B)},
	{.cmd = CMD_SE_1_1_0_64_4B,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 1, 0, 0, 5, 4, 1, CMD_SE_1_1_0_64_4B)},
	{.cmd = CMD_WREN,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 0, 0, 0, 0, 0, 0, 0, CMD_WREN)},
	{.cmd = CMD_WRDIS,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 0, 0, 0, 0, 0, 0, 0, CMD_WRDIS)},
	{.cmd = CMD_RDSR,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 0, 1, 0, 0, 0, 0, CMD_RDSR)},
	{.cmd = CMD_RDSR2,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 0, 1, 0, 0, 0, 0, CMD_RDSR2)},
	{.cmd = CMD_WRSR,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 0, 1, 0, 0, 0, 0, CMD_WRSR)},
	{.cmd = CMD_WRSR2,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 0, 1, 0, 0, 0, 0, CMD_WRSR2)},
	{.cmd = CMD_RDCR,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 0, 1, 0, 0, 0, 0, CMD_RDCR)},
	{.cmd = CMD_EN4B,
		.cmd_table_val = CMD_TABLE_VALUE(0, 0, 0, 0, 0, 0, 0, 0, 0, CMD_EN4B)},
	{.cmd = CMD_EX4B,
		.cmd_table_val = CMD_TABLE_VALUE(0, 0, 0, 0, 0, 0, 0, 0, 0, CMD_EX4B)},
	{.cmd = CMD_SFDP,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 0, 1, 8, 0, 3, 1, CMD_SFDP)},
	{.cmd = CMD_RDID,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 0, 1, 0, 0, 0, 0, CMD_RDID)},
	{.cmd = CMD_RDFSR,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 1, 0, 1, 0, 0, 0, 0, CMD_RDFSR)},
	{.cmd = CMD_VSR_WREN,
		.cmd_table_val = CMD_TABLE_VALUE(1, 0, 0, 0, 0, 0, 0, 0, 0, CMD_VSR_WREN)},
	{.cmd = CMD_WREAR,
		.cmd_table_val = CMD_TABLE_VALUE(0, 1, 0, 0, 1, 0, 0, 0, 0, CMD_WREAR)},
	{.cmd = CMD_WINBOND_DIE_SEL,
		.cmd_table_val = CMD_TABLE_VALUE(1, 1, 0, 0, 1, 0, 0, 0, 0, CMD_WINBOND_DIE_SEL)},
};

#if CONFIG_ASPEED_SPIM_LOG_SIZE > 0
static uint8_t spim_log_arr[CONFIG_ASPEED_SPIM_LOG_SIZE] NON_CACHED_BSS_ALIGN16;
#endif

/* control register */
#define SPIM_CTRL                    (0x0000)
#define SPIM_IO_IRQ_CTRL             (0x0004)
#define SPIM_EAR                     (0x0008)
#define SPIM_FIFO                    (0x000C)
#define SPIM_LOG_BASE                (0x0010)
#define SPIM_LOG_SZ                  (0x0014)
#define SPIM_LOG_PTR                 (0x0018)
#define SPIM_LOCK_REG                (0x007C)
#define SPIM_ALLOW_CMD_BASE          (0x0080)
#define SPIM_ADDR_PRIV_TABLE_BASE    (0x0100)

/* allow command table */
#define SPIM_CMD_TABLE_NUM              32
#define SPIM_CMD_TABLE_VALID_MASK       GENMASK(31, 30)
#define SPIM_CMD_TABLE_VALID_ONCE_BIT   BIT(31)
#define SPIM_CMD_TABLE_VALID_BIT        BIT(30)
#define SPIM_CMD_TABLE_IS_GENERIC_CMD   BIT(29)
#define SPIM_CMD_TABLE_IS_WRITE_CMD     BIT(28)
#define SPIM_CMD_TABLE_IS_READ_CMD      BIT(27)
#define SPIM_CMD_TABLE_IS_MEM_CMD       BIT(26)
#define SPIM_CMD_TABLE_DATA_MODE_MASK   GENMASK(25, 24)
#define SPIM_CMD_TABLE_LOCK_BIT         BIT(23)
#define SPIM_CMD_TABLE_DUMMY_MASK       GENMASK(21, 16)
#define SPIM_CMD_TABLE_PROGRAM_SZ_MASK  GENMASK(15, 13)
#define SPIM_CMD_TABLE_ADDR_LEN_MASK    GENMASK(12, 10)
#define SPIM_CMD_TABLE_ADDR_MODE_MASK   GENMASK(9, 8)
#define SPIM_CMD_TABLE_CMD_MASK         GENMASK(7, 0)

/* allow address region configuration */
#define SPIM_PRIV_WRITE_SELECT   0x57000000
#define SPIM_PRIV_READ_SELECT    0x52000000
#define SPIM_ADDR_PRIV_REG_NUN   512
#define SPIM_ADDR_PRIV_BIT_NUN   (SPIM_ADDR_PRIV_REG_NUN * 32)

/* lock register */
/* SPIPF00 */
#define SPIM_CTRL_MULTI_PASSTHROUGH     BIT(1)
#define SPIM_ENABLE                     BIT(2)
#define SPIM_MUX_SEL                    BIT(3)
#define SPIM_RISING_INACTIVATE          BIT(6)
#define SPIM_BLOCK_FIFO_CLR             BIT(8)
#define SPIM_SW_RST                     BIT(15)
#define SPIM_BLOCK_FIFO_CTRL_LOCK       BIT(22)
#define SPIM_SW_RST_CTRL_LOCK           BIT(23)
#define SPIM_BLOCK_FIFO_LEN             GENMASK(26, 24)

/* SPIPF7C */
#define SPIM_CTRL_REG_LOCK              BIT(0)
#define SPIM_INT_STS_REG_LOCK           BIT(1)
#define SPIM_LOG_BASE_ADDR_REG_LOCK     BIT(4)
#define SPIM_LOG_CTRL_REG_LOCK          BIT(5)
#define SPIM_ADDR_PRIV_WRITE_TABLE_LOCK BIT(30)
#define SPIM_ADDR_PRIV_READ_TABLE_LOCK  BIT(31)

/* irq related bit */
#define SPIM_CMD_BLOCK_IRQ              BIT(0)
#define SPIM_WRITE_BLOCK_IRQ            BIT(1)
#define SPIM_READ_BLOCK_IRQ             BIT(2)
#define SPIM_IRQ_STS_MASK               GENMASK(2, 0)
#define SPIM_CMD_BLOCK_IRQ_EN           BIT(16)
#define SPIM_WRITE_BLOCK_IRQ_EN         BIT(17)
#define SPIM_READ_BLOCK_IRQ_EN          BIT(18)

/* push-pulll mode enabled bit */
#define SPIM_PUSH_PULL_ENABLED          BIT(31)

/* spi monitor log control */
#define SPIM_BLOCK_INFO_EN              BIT(31)

/* PFR related control */
#define AST1060_SPIM_MODE_SCU_CTRL      (0x00f0)
#define AST1080_SCU_ANALOG_MUX_MODE     (0x00D0)
#define AST1080_SCU_SPI_MODE            (0x00D4)

/* AST2700 */
/* On AST2700, SPI monitor is concatenated after SPI controller */
#define SPIM_SPIC_CONCAT_OFFSET         0x400
#define SPI_CTRL_WIN                    0x30
#define SPI_CTRL_LOCK_SOC               0x1F8
#define SPI_CTRL_CS0_WIN_LOCK           BIT(20)
#define SPI_CTRL_CS1_WIN_LOCK           BIT(21)
#define SPI_CTRL_CS2_WIN_LOCK           BIT(22)

#define SPIM_CS0_BASE                   (0x0020)

/* AST2700 address filter config */
/* #define SPIM_ADDR_PRIV_TABLE_BASE    (0x0100) */
#define SPIM_ADDR_PRIV_REGION_NUM       64
#define SPIM_ADDR_PRIV_VALID            BIT(0)
#define SPIM_ADDR_PRIV_WRITE_DIS        BIT(1)
#define SPIM_ADDR_PRIV_READ_DIS         BIT(2)
#define SPIM_ADDR_PRIV_START_MASK       GENMASK(31, 12)
/* SPIM_ADDR_CTRL + 0x04 */
#define SPIM_ADDR_PRIV_LEN_MASK         GENMASK(20, 0)
#define SPIM_ADDR_PRIV_LEN_RAW_MASK     GENMASK(31, 12)
#define SPIM_ADDR_PRIV_LEN_SHIFT        12
#define SPIM_ADDR_PRIV_LOCK             BIT(31)

/* access permission control */
#define SPIM_PROT_CTRL_BASE_OFF       0x400
#define SPIM_PROT_MID0                0x10
#define SPIM_PROT_MID1                0x14
#define SPIM_PROT_MID_NUM             8
#define SPIM_BOOTMCU_I_ID             0x20
#define SPIM_BOOTMCU_D_ID             0x21
#define SPIM_SSP_I_ID                 0x02
#define SPIM_SSP_D_ID                 0x03
#define SPIM_SSP_S_ID                 0x04
#define SPIM_DUMMY_M_ID               0x7F
#define SPIM_PROT_RW_CTRL             0x80
#define SPIM_PROT_ADDR_CTRL           0xC0

struct aspeed_spim_data {
	const struct device *dev;
	struct k_sem sem_spim; /* protect most control registers */
	struct k_spinlock irq_ctrl_lock; /* protect ISR content */
	uint8_t allow_cmd_list[SPIM_CMD_TABLE_NUM];
	uint32_t allow_cmd_num;

	/* AST1060 */
	uint32_t read_forbidden_regions[32];
	uint32_t read_forbidden_region_num;
	uint32_t write_forbidden_regions[32];
	uint32_t write_forbidden_region_num;

	/* AST2700 */
	uint32_t addr_priv_config[48];
	uint32_t addr_priv_config_num;

	struct k_work log_work;
	struct spim_log_info log_info;
	spim_isr_callback_t isr_callback;
};

struct aspeed_spim_soc_ops {
	void (*addr_priv_init)(const struct device *dev);
	void (*elec_char_init)(const struct device *dev);
	void (*allow_cmd_table_init)(const struct device *dev,
				     const uint8_t cmd_list[],
				     uint32_t cmd_num, uint32_t flag);
	void (*monitor_enable)(const struct device *dev, bool enable);
	void (*ctrl_sw_rst)(const struct device *dev);
	int  (*blocked_log_init)(const struct device *dev);
	void (*flash_rst_release)(const struct device *dev);
	void (*mux_config)(const struct device *dev, enum spim_ext_mux_sel mux_sel);
	void (*dump_addr_priv)(const struct device *dev);
	void (*addr_priv_lock)(const struct device *dev);
	void (*addr_priv_remove_all)(const struct device *dev);
	void (*misc_lock)(const struct device *dev);
};

struct aspeed_spim_config {
	mm_reg_t ctrl_base;
	uint32_t irq_num;
	uint32_t irq_priority;
	uint32_t ctrl_idx;
	uint32_t ext_mux_sel_default;
	bool force_rel_flash_rst;
	const struct device *parent;
	const struct gpio_dt_spec *ext_mux_sel_gpios;
	uint32_t ext_mux_sel_gpio_num;
	uint32_t ext_mux_sel_delay_us;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
	const struct aspeed_spim_soc_ops *ops;
};

struct aspeed_spim_common_config {
	mm_reg_t scu_base;
	uint32_t mode_ctrl_off;
};

struct aspeed_spim_common_data {
	struct k_spinlock scu_lock; /* protect SCU0F0 */

	struct k_sem sem_log_op; /* protect log info */
	uint32_t cur_log_sz;
};

struct priv_reg_info {
	uint32_t start_reg_off;
	uint32_t start_bit_off;
	uint32_t end_reg_off;
	uint32_t end_bit_off;
};

struct aspeed_spim_cs_pd_info {
	uint32_t off;
	uint32_t bit;
};

struct aspeed_spim_miso_pin_info {
	uint32_t off;
	uint32_t bit;
};

static void acquire_spim_device(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct aspeed_spim_data *const data = dev->data;

		k_sem_take(&data->sem_spim, K_FOREVER);
	}
}

static void release_spim_device(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct aspeed_spim_data *const data = dev->data;

		k_sem_give(&data->sem_spim);
	}
}

static void spim_scu_reg_set(const struct device *dev, uint32_t reg_off,
			      uint32_t mask, uint32_t val)
{
	const struct aspeed_spim_common_config *config = dev->config;
	struct aspeed_spim_common_data *const data = dev->data;
	mm_reg_t spim_scu_reg = config->scu_base + reg_off;
	uint32_t reg_val;
	/* Avoid SCU0F0 being accessed by more than a thread */
	k_spinlock_key_t key = k_spin_lock(&data->scu_lock);

	reg_val = sys_read32(spim_scu_reg);
	reg_val &= ~(mask);
	reg_val |= val;
	sys_write32(reg_val, spim_scu_reg);

	k_spin_unlock(&data->scu_lock, key);
}

static void spim_scu_reg_clear(const struct device *dev, uint32_t reg_off,
				uint32_t clear_bits)
{
	const struct aspeed_spim_common_config *config = dev->config;
	struct aspeed_spim_common_data *const data = dev->data;
	mm_reg_t spim_scu_reg = config->scu_base + reg_off;
	uint32_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->scu_lock);

	reg_val = sys_read32(spim_scu_reg);
	reg_val &= ~(clear_bits);
	sys_write32(reg_val, spim_scu_reg);

	k_spin_unlock(&data->scu_lock, key);
}

void spim_scu_ctrl_set(const struct device *dev, uint32_t mask, uint32_t val)
{
	const struct aspeed_spim_common_config *config = dev->config;

	spim_scu_reg_set(dev, config->mode_ctrl_off, mask, val);
}

void spim_scu_ctrl_clear(const struct device *dev, uint32_t clear_bits)
{
	const struct aspeed_spim_common_config *config = dev->config;

	spim_scu_reg_clear(dev, config->mode_ctrl_off, clear_bits);
}

#if CONFIG_ASPEED_SPIM_LOG_SIZE > 0
static void acquire_log_op(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct aspeed_spim_common_data *const data = dev->data;

		k_sem_take(&data->sem_log_op, K_FOREVER);
	}
}

static void release_log_op(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct aspeed_spim_common_data *const data = dev->data;

		k_sem_give(&data->sem_log_op);
	}
}
#endif /* CONFIG_ASPEED_SPIM_LOG_SIZE > 0 */

static void ast1060_scu_monitor_config(const struct device *dev, bool enable)
{
	const struct aspeed_spim_config *config = dev->config;

	if (enable) {
		spim_scu_ctrl_set(config->parent, BIT(config->ctrl_idx - 1) << 8,
			BIT(config->ctrl_idx - 1) << 8);
	} else {
		spim_scu_ctrl_clear(config->parent, BIT(config->ctrl_idx - 1) << 8);
	}
}

static void ast1060_ctrl_monitor_config(const struct device *dev, bool enable)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;

	acquire_spim_device(dev);

	reg_val = sys_read32(config->ctrl_base);
	if (enable)
		reg_val |= SPIM_ENABLE;
	else
		reg_val &= ~(SPIM_ENABLE);

	sys_write32(reg_val, config->ctrl_base);

	release_spim_device(dev);
}

/*
 * SPI_M1: GPIOA6, SCU610[6]
 * SPI_M2: GPIOC4, SCU610[20]
 * SPI_M3: GPIOE2, SCU614[2] (dummy, cannot be disabled)
 * SPI_M4: GPIOG0, SCU614[16]
 */
static void ast1060_disable_cs_internal_pd(const struct device *dev, uint32_t idx)
{
	const struct aspeed_spim_common_config *config = dev->config;
	struct aspeed_spim_common_data *const data = dev->data;
	mm_reg_t spim_scu_ctrl = config->scu_base;
	uint32_t reg_val;
	struct aspeed_spim_cs_pd_info pd_info[4] = {
		{0x610, BIT(6)},
		{0x610, BIT(20)},
		{0x614, BIT(2)},
		{0x614, BIT(16)},
	};
	/* Avoid SCU being accessed by more than a thread */
	k_spinlock_key_t key = k_spin_lock(&data->scu_lock);

	if (idx >= 4)
		goto end;

	reg_val = sys_read32(spim_scu_ctrl + pd_info[idx].off);
	reg_val |= pd_info[idx].bit;
	sys_write32(reg_val, spim_scu_ctrl + pd_info[idx].off);

end:
	k_spin_unlock(&data->scu_lock, key);
}

static void ast1060_miso_multi_func_adjust(const struct device *dev, bool enable)
{
	const struct aspeed_spim_config *config = dev->config;
	const struct device *parent_dev = config->parent;
	const struct aspeed_spim_common_config *parent_config = parent_dev->config;
	mm_reg_t scu_base = parent_config->scu_base;
	struct aspeed_spim_common_data *const parent_data = parent_dev->data;
	uint32_t reg_val;
	struct aspeed_spim_miso_pin_info miso_info[4] = {
		{0x690, BIT(3)},
		{0x690, BIT(17)},
		{0x690, BIT(31)},
		{0x694, BIT(13)},
	};
	k_spinlock_key_t key;

	if (config->ctrl_idx == 0 || config->ctrl_idx > 4)
		return;

	key = k_spin_lock(&parent_data->scu_lock);

	reg_val = sys_read32(scu_base +
			     miso_info[config->ctrl_idx - 1].off);
	if (enable)
		reg_val |= miso_info[config->ctrl_idx - 1].bit;
	else
		reg_val &= ~(miso_info[config->ctrl_idx - 1].bit);

	sys_write32(reg_val,
		    scu_base + miso_info[config->ctrl_idx - 1].off);

	k_spin_unlock(&parent_data->scu_lock, key);
}

static void ast1060_scu_passthrough_mode(const struct device *dev,
	enum spim_passthrough_mode mode, bool passthrough_en)
{
	const struct aspeed_spim_config *config = dev->config;

	if (passthrough_en) {
		spim_scu_ctrl_set(config->parent, BIT(config->ctrl_idx - 1) << 4,
			BIT(config->ctrl_idx - 1) << 4);
	} else {
		spim_scu_ctrl_clear(config->parent, BIT(config->ctrl_idx - 1) << 4);
	}

	ARG_UNUSED(mode);
}

static void ast1060_passthrough_config(const struct device *dev,
	enum spim_passthrough_mode mode, bool passthrough_en)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t ctrl_reg_val;

	acquire_spim_device(dev);

	ctrl_reg_val = sys_read32(config->ctrl_base);

	ctrl_reg_val &= ~0x00000003;
	if (passthrough_en) {
		if (mode == SPIM_MULTI_PASSTHROUGH)
			ctrl_reg_val |= BIT(1);
		else
			ctrl_reg_val |= BIT(0);
	}

	sys_write32(ctrl_reg_val, config->ctrl_base);

	release_spim_device(dev);
}

/* Don't remove due to existing applications. */
void spim_passthrough_config(const struct device *dev,
	enum spim_passthrough_mode mode, bool passthrough_en)
{
	ast1060_passthrough_config(dev, mode, passthrough_en);
}

static void ast1060_release_flash_rst(const struct device *dev)
{
	uint32_t val;
	const struct aspeed_spim_config *config = dev->config;
	const struct device *parent_dev = config->parent;
	uint32_t bit_off = 1 << (config->ctrl_idx - 1);

	/* Using SCU0F0 to enable flash rst
	 * SCU0F0[23:20]: Reset source selection
	 * SCU0F0[27:24]: Enable reset signal output
	 */
	val = (bit_off << 20) | (bit_off << 24);
	spim_scu_ctrl_set(parent_dev, val, val);

	/* SCU0F0[19:16]: output value */
	val = bit_off << 16;
	/* release reset */
	spim_scu_ctrl_set(parent_dev, val, val);

	k_busy_wait(5000); /* 5ms */
}

static void ast1060_push_pull_mode_config(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;

	acquire_spim_device(dev);

	reg_val = sys_read32(config->ctrl_base + SPIM_IO_IRQ_CTRL);
	reg_val |= SPIM_PUSH_PULL_ENABLED;
	sys_write32(reg_val, config->ctrl_base + SPIM_IO_IRQ_CTRL);

	release_spim_device(dev);

	/*
	 * When AST060 is in unprovision state, except for SPIPF004[31],
	 * SPIPF000[0] and SCU0F0[11:8] should be set for achieving
	 * push-pull mode.
	 */
	ast1060_passthrough_config(dev, SPIM_SINGLE_PASSTHROUGH, true);
	ast1060_scu_monitor_config(dev, true);
}

static void ast1060_elec_char_init(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;

	/* always enable internal passthrough configuration */
	ast1060_scu_passthrough_mode(dev, 0, true);
	/* always keep at master mode during booting up stage */
	spim_ext_mux_config(dev, config->ext_mux_sel_default);
	/* always disable internal pull-down of CS pin */
	ast1060_disable_cs_internal_pd(config->parent, config->ctrl_idx - 1);
	/* use push-pull mode to improve IO signal quality */
	ast1060_push_pull_mode_config(dev);
}

static void ast1060_ext_mux_config(const struct device *dev,
			    enum spim_ext_mux_sel mux_sel)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t i;
	int value;

	if (mux_sel > SPIM_EXT_MUX_SEL_1) {
		LOG_ERR("wrong ext mux selection (%d)", mux_sel);
		return;
	}

	if (config->ext_mux_sel_gpio_num) {
		for (i = 0;  i < config->ext_mux_sel_gpio_num; i++) {
			if (!device_is_ready(config->ext_mux_sel_gpios[i].port)) {
				LOG_ERR("device %s is not ready",
					config->ext_mux_sel_gpios[i].port->name);
				continue;
			}

			if (mux_sel == SPIM_EXT_MUX_SEL_1)
				value = 1;
			else
				value = 0;

			if (gpio_pin_configure_dt(&config->ext_mux_sel_gpios[i], GPIO_OUTPUT)) {
				LOG_ERR("[%s %d]: set output %d failed",
					config->ext_mux_sel_gpios[i].port->name,
					config->ext_mux_sel_gpios[i].pin,
					value);
				continue;
			}
			gpio_pin_set(config->ext_mux_sel_gpios[i].port,
				     config->ext_mux_sel_gpios[i].pin,
				     value);
		}
	} else {
		if (mux_sel == SPIM_EXT_MUX_SEL_1) {
			spim_scu_ctrl_set(config->parent, BIT(config->ctrl_idx - 1) << 12,
					  BIT(config->ctrl_idx - 1) << 12);
		} else {
			spim_scu_ctrl_clear(config->parent, BIT(config->ctrl_idx - 1) << 12);
		}
	}

	k_busy_wait(config->ext_mux_sel_delay_us);
}

/*
 * SCU0D0 is the Analog Mux Mode Register (SCU_AM_MODE_x: 0=external signal,
 * 1=internal signal) and SCU0D4 is the SPI Mode Register (SCU_SPIxO_MODE:
 * 0=Monitor Mode, 1=Master Mode).
 */
struct ast1080_spim_pinmux_bits {
	uint32_t analog_mux_bits;
	uint32_t spi_mode_bit;
};

static struct ast1080_spim_pinmux_bits ast1080_spim_pinmux_bits(uint32_t ctrl_idx)
{
	uint32_t bus = ctrl_idx - 1;
	uint32_t base = bus * 7;

	return (struct ast1080_spim_pinmux_bits) {
		.analog_mux_bits = 0x7F << base,
		.spi_mode_bit = BIT(bus),
	};
}

static void ast1080_ext_mux_config(const struct device *dev,
			    enum spim_ext_mux_sel mux_sel)
{
	const struct aspeed_spim_config *config = dev->config;
	struct ast1080_spim_pinmux_bits bits = ast1080_spim_pinmux_bits(config->ctrl_idx);

	if (mux_sel > SPIM_EXT_MUX_SEL_1) {
		LOG_ERR("wrong ext mux selection (%d)", mux_sel);
		return;
	}

	if (mux_sel == SPIM_EXT_MUX_SEL_1) {
		spim_scu_reg_set(config->parent, AST1080_SCU_ANALOG_MUX_MODE,
				  bits.analog_mux_bits, bits.analog_mux_bits);
		spim_scu_reg_set(config->parent, AST1080_SCU_SPI_MODE,
				  bits.spi_mode_bit, bits.spi_mode_bit);
	} else {
		spim_scu_reg_clear(config->parent, AST1080_SCU_ANALOG_MUX_MODE,
				    bits.analog_mux_bits);
		spim_scu_reg_clear(config->parent, AST1080_SCU_SPI_MODE,
				    bits.spi_mode_bit);
	}

	k_busy_wait(config->ext_mux_sel_delay_us);
}

#define AST1060_ABS_ADDR(reg_off, bit_off) ((reg_off) * 524288 + (bit_off) * 16384)

static void ast1060_addr_priv_enable(const struct device *dev,
					    enum addr_priv_rw_select rw_sel)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;

	reg_val = sys_read32(config->ctrl_base);
	reg_val &= 0x00FFFFFF;

	switch (rw_sel) {
	case FLAG_ADDR_PRIV_READ_SELECT:
		reg_val |= SPIM_PRIV_READ_SELECT;
		break;
	case FLAG_ADDR_PRIV_WRITE_SELECT:
		reg_val |= SPIM_PRIV_WRITE_SELECT;
		break;
	default:
		break;
	};

	sys_write32(reg_val, config->ctrl_base);
}

static void ast1060_fobidden_area_parser(const struct device *dev,
	struct priv_reg_info start, struct priv_reg_info *res,
	uint32_t *num_forbidden_blk)
{
	const struct aspeed_spim_config *config = dev->config;
	mm_reg_t priv_table_base = config->ctrl_base + SPIM_ADDR_PRIV_TABLE_BASE;
	uint32_t reg_off = start.start_reg_off;
	uint32_t bit_off = start.start_bit_off;
	uint32_t reg_val;
	uint32_t i;

	/* init search result */
	*num_forbidden_blk = 0;

	while (reg_off < SPIM_ADDR_PRIV_REG_NUN) {
		reg_val = sys_read32(priv_table_base + reg_off * 4);
		reg_val >>= bit_off;
		for (i = bit_off; i < 32; i++) {
			if ((reg_val & 1) == 0) {
				if (*num_forbidden_blk == 0) {
					/* get the first forbidden block */
					res->start_reg_off = reg_off;
					res->start_bit_off = i;
				}

				(*num_forbidden_blk)++;
			} else if ((reg_val & 1) == 1 && *num_forbidden_blk != 0) {
				res->end_reg_off = reg_off;
				res->end_bit_off = i;
				return;
			}

			reg_val >>= 1;
		}

		bit_off = 0;
		reg_off++;
	}

	res->end_reg_off = SPIM_ADDR_PRIV_REG_NUN - 1;
	res->end_bit_off = 32;
}

static void ast1060_dump_addr_priv_table(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t num_forbidden_blk = 0;
	struct priv_reg_info start;
	struct priv_reg_info res;
	bool protect_en;
	uint32_t rw;
	bool lock;
	uint32_t saved_ctrl;

	acquire_spim_device(dev);

	/* Save SPIM_CTRL[31:24] so the active table selector is not disturbed */
	saved_ctrl = sys_read32(config->ctrl_base);

	for (rw = 0; rw < 2; rw++) {
		if (rw == 0)
			ast1060_addr_priv_enable(dev, FLAG_ADDR_PRIV_READ_SELECT);
		else
			ast1060_addr_priv_enable(dev, FLAG_ADDR_PRIV_WRITE_SELECT);

		lock = false;
		protect_en = false;
		if (sys_read32(config->ctrl_base + SPIM_LOCK_REG) & BIT(31 - rw))
			lock = true;

		memset(&start, 0x0, sizeof(struct priv_reg_info));
		memset(&res, 0x0, sizeof(struct priv_reg_info));
		printk("%s protection regions:\n", rw == 0 ? "read" : "write");
		printk("privilege table is %s\n", lock ? "locked" : "unlocked");
		do {
			ast1060_fobidden_area_parser(dev, start, &res, &num_forbidden_blk);
			if (num_forbidden_blk != 0) {
				protect_en = true;
				printk("[0x%08x - 0x%08x]\n",
					AST1060_ABS_ADDR(res.start_reg_off, res.start_bit_off),
					AST1060_ABS_ADDR(res.end_reg_off, res.end_bit_off));
				start.start_reg_off = res.end_reg_off;
				start.start_bit_off = res.end_bit_off;
			}
		} while (num_forbidden_blk != 0);

		if (!protect_en)
			printk("all regions are %s!\n", rw == 0 ? "readable" : "writable");
		printk("======END======\n\n");
	}

	/* Restore the active table selector to avoid disabling read/write protection */
	sys_write32(saved_ctrl, config->ctrl_base);

	release_spim_device(dev);
}

static uint32_t ast1060_get_cross_block_num(uint32_t addr, uint32_t len)
{
	if (len == 0)
		return 0;

	if (addr % KB(16) != 0)
		len += (addr % KB(16));

	/* 16KB aligned */
	len = (len + KB(16) - 1) / KB(16) * KB(16);

	return len / KB(16);
}

/* Don't modify the function name since it has already been used by customers */
int spim_address_privilege_config(const struct device *dev,
				  enum addr_priv_rw_select rw_sel,
				  enum addr_priv_op priv_op,
				  mm_reg_t addr, uint32_t len)
{
	const struct aspeed_spim_config *config = dev->config;
	mm_reg_t priv_table_base = config->ctrl_base + SPIM_ADDR_PRIV_TABLE_BASE;
	int ret = 0;
	uint32_t reg_off;
	uint32_t bit_off;
	uint32_t total_bit_num;
	uint32_t reg_val;

	if (addr >= MB(256) || len == 0) {
		LOG_WRN("invalid address or zero length!");
		return -EINVAL;
	}

	if (addr + len > MB(256)) {
		LOG_WRN("invalid protected regions, change the protected length...");
		len -= (addr + len - MB(256));
		LOG_WRN("the new length: 0x%08x", len);
	}

	if ((addr % KB(16)) != 0 || (len % KB(16)) != 0) {
		LOG_WRN("protected address(0x%08lx) and length(0x%08x) should be 16KB aligned",
			addr, len);
		LOG_WRN("stricter protection regions will be applied. (force 16KB aligned)");
		/* protect more region in order to align 16KB boundary */
		len = addr + len - (addr / KB(16)) * KB(16);
		addr = (addr / KB(16)) * KB(16);
		len = ((len + KB(16) - 1) / KB(16)) * KB(16);
	}

	reg_off = addr / KB(512); /* 512K per register; */
	bit_off = (addr % KB(512)) / KB(16); /* (512K / 16K); */
	total_bit_num = ast1060_get_cross_block_num(addr, len);
	LOG_DBG("addr: 0x%08lx, len: 0x%08x\n", addr, len);
	LOG_DBG("reg_off: 0x%08x, bit_off: 0x%08x, total_bit_num: 0x%08x\n",
		reg_off, bit_off, total_bit_num);

	acquire_spim_device(dev);

	/* check lock status */
	if (rw_sel == FLAG_ADDR_PRIV_READ_SELECT &&
		(sys_read32(config->ctrl_base + SPIM_LOCK_REG) & BIT(31))) {
		LOG_ERR("read address privilege table is locked!");
		ret = -ECANCELED;
		goto end;
	} else if (rw_sel == FLAG_ADDR_PRIV_WRITE_SELECT &&
		(sys_read32(config->ctrl_base + SPIM_LOCK_REG) & BIT(30))) {
		LOG_ERR("write address privilege table is locked!");
		ret = -ECANCELED;
		goto end;
	}

	/* enable access */
	if (rw_sel == FLAG_ADDR_PRIV_READ_SELECT)
		ast1060_addr_priv_enable(dev, FLAG_ADDR_PRIV_READ_SELECT);
	else
		ast1060_addr_priv_enable(dev, FLAG_ADDR_PRIV_WRITE_SELECT);

	do {
		if (bit_off > 31) {
			bit_off = 0;
			reg_off++;
		}

		if (bit_off == 0 && total_bit_num >= 32) {
			/* speed up for large area configuration */
			if (priv_op == FLAG_ADDR_PRIV_ENABLE)
				sys_write32(0xffffffff, priv_table_base + reg_off * 4);
			else
				sys_write32(0x0, priv_table_base + reg_off * 4);

			reg_off++;
			total_bit_num -= 32;
		} else {
			reg_val = sys_read32(priv_table_base + reg_off * 4);
			if (priv_op == FLAG_ADDR_PRIV_ENABLE) {
				sys_write32(reg_val | BIT(bit_off),
					    priv_table_base + reg_off * 4);
			} else {
				sys_write32(reg_val & (~BIT(bit_off)),
					    priv_table_base + reg_off * 4);
			}

			LOG_DBG("reg: 0x%08lx, val: 0x%08x\n",
				priv_table_base + reg_off * 4,
				sys_read32(priv_table_base + reg_off * 4));

			bit_off++;
			total_bit_num--;
		}
	} while (total_bit_num > 0);

end:
	release_spim_device(dev);

	return ret;
}

static void ast1060_addr_priv_remove_all(const struct device *dev)
{
	int ret;

	ret = spim_address_privilege_config(dev, FLAG_ADDR_PRIV_READ_SELECT,
					     FLAG_ADDR_PRIV_ENABLE, 0x0, MB(256));
	if (ret)
		LOG_WRN("read address privilege table is locked, not cleared.");

	ret = spim_address_privilege_config(dev, FLAG_ADDR_PRIV_WRITE_SELECT,
					     FLAG_ADDR_PRIV_ENABLE, 0x0, MB(256));
	if (ret)
		LOG_WRN("write address privilege table is locked, not cleared.");
}

static void ast1060_addr_priv_init(const struct device *dev)
{
	struct aspeed_spim_data *const data = dev->data;
	int ret;
	uint32_t i;

	spim_address_privilege_config(dev, FLAG_ADDR_PRIV_READ_SELECT,
		FLAG_ADDR_PRIV_ENABLE, 0x0, MB(256));

	spim_address_privilege_config(dev, FLAG_ADDR_PRIV_WRITE_SELECT,
		FLAG_ADDR_PRIV_ENABLE, 0x0, MB(256));

	if (data->read_forbidden_region_num % 2 != 0) {
		LOG_ERR("wrong read-forbidden-regions setting in .dts.");
		return;
	}

	if (data->write_forbidden_region_num % 2 != 0) {
		LOG_ERR("wrong write-forbidden-regions setting in .dts.");
		return;
	}

	if (data->read_forbidden_region_num > ARRAY_SIZE(data->read_forbidden_regions)) {
		LOG_ERR("[%s] read_forbidden_regions exceeds array size %zu",
			dev->name, ARRAY_SIZE(data->read_forbidden_regions));
		return;
	}

	if (data->write_forbidden_region_num > ARRAY_SIZE(data->write_forbidden_regions)) {
		LOG_ERR("[%s] write_forbidden_regions exceeds array size %zu",
			dev->name, ARRAY_SIZE(data->write_forbidden_regions));
		return;
	}

	for (i = 0; i < data->read_forbidden_region_num; i += 2) {
		LOG_DBG("[%s]addr: 0x%08x, len: 0x%08x", dev->name,
				data->read_forbidden_regions[i],
				data->read_forbidden_regions[i + 1]);

		ret = spim_address_privilege_config(dev,
			FLAG_ADDR_PRIV_READ_SELECT,
			FLAG_ADDR_PRIV_DISABLE,
			data->read_forbidden_regions[i],
			data->read_forbidden_regions[i + 1]);
		if (ret != 0)
			LOG_ERR("fail to configure read address privilege table!");
	}

	for (i = 0; i < data->write_forbidden_region_num; i += 2) {
		LOG_DBG("[%s]addr: 0x%08x, len: 0x%08x", dev->name,
				data->write_forbidden_regions[i],
				data->write_forbidden_regions[i + 1]);

		ret = spim_address_privilege_config(dev,
			FLAG_ADDR_PRIV_WRITE_SELECT,
			FLAG_ADDR_PRIV_DISABLE,
			data->write_forbidden_regions[i],
			data->write_forbidden_regions[i + 1]);
		if (ret != 0)
			LOG_ERR("fail to configure write address privilege table!");
	}
}

static void ast1060_addr_priv_table_lock(const struct device *dev,
	enum addr_priv_rw_select rw_sel)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;

	acquire_spim_device(dev);

	reg_val = sys_read32(config->ctrl_base + SPIM_LOCK_REG);

	if (rw_sel == FLAG_ADDR_PRIV_READ_SELECT)
		reg_val |= SPIM_ADDR_PRIV_READ_TABLE_LOCK;

	if (rw_sel == FLAG_ADDR_PRIV_WRITE_SELECT)
		reg_val |= SPIM_ADDR_PRIV_WRITE_TABLE_LOCK;

	sys_write32(reg_val, config->ctrl_base + SPIM_LOCK_REG);

	release_spim_device(dev);
}

static void ast1060_addr_priv_lock(const struct device *dev)
{
	ast1060_addr_priv_table_lock(dev, FLAG_ADDR_PRIV_READ_SELECT);
	ast1060_addr_priv_table_lock(dev, FLAG_ADDR_PRIV_WRITE_SELECT);
}

static void ast1060_misc_lock(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;

	acquire_spim_device(dev);

	reg_val = sys_read32(config->ctrl_base);

	reg_val |= SPIM_BLOCK_FIFO_CTRL_LOCK |
				SPIM_SW_RST_CTRL_LOCK;

	sys_write32(reg_val, config->ctrl_base);

	reg_val = sys_read32(config->ctrl_base + SPIM_LOCK_REG);

	reg_val |= SPIM_CTRL_REG_LOCK |
				SPIM_INT_STS_REG_LOCK |
				SPIM_LOG_BASE_ADDR_REG_LOCK |
				SPIM_LOG_CTRL_REG_LOCK;

	sys_write32(reg_val, config->ctrl_base + SPIM_LOCK_REG);

	release_spim_device(dev);
}

#if CONFIG_ASPEED_SPIM_LOG_SIZE > 0
static int ast1060_ram_log_init(const struct device *dev)
{
	int ret = 0;
	const struct aspeed_spim_config *config = dev->config;
	struct aspeed_spim_data *const data = dev->data;
	const struct device *parent_dev = config->parent;
	struct aspeed_spim_common_data *const parent_data = parent_dev->data;
	uint32_t cur_log_sz;
	uint32_t log_num;

	if (data->log_info.log_max_sz == 0)
		return 0;

	log_num = data->log_info.log_max_sz / 4;

	acquire_log_op(config->parent);
	cur_log_sz = parent_data->cur_log_sz;
	if (cur_log_sz + data->log_info.log_max_sz > CONFIG_ASPEED_SPIM_LOG_SIZE) {
		release_log_op(config->parent);
		LOG_ERR("[%s]invalid log size on ram", dev->name);
		ret = -ENOBUFS;
		goto end;
	}
	data->log_info.log_ram_addr = (mem_addr_t)(&spim_log_arr[0] + cur_log_sz);
	parent_data->cur_log_sz += data->log_info.log_max_sz;
	release_log_op(config->parent);

	acquire_spim_device(dev);
	sys_write32(data->log_info.log_ram_addr, config->ctrl_base + SPIM_LOG_BASE);
	sys_write32(log_num | SPIM_BLOCK_INFO_EN, config->ctrl_base + SPIM_LOG_SZ);
	release_spim_device(dev);

end:
	return ret;
}
#endif /* CONFIG_ASPEED_SPIM_LOG_SIZE > 0 */

static void spim_sw_rst(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;

	acquire_spim_device(dev);

	reg_val = sys_read32(config->ctrl_base + SPIM_CTRL);
	reg_val |= SPIM_SW_RST;
	sys_write32(reg_val, config->ctrl_base + SPIM_CTRL);

	k_usleep(5);

	reg_val &= ~(SPIM_SW_RST);
	sys_write32(reg_val, config->ctrl_base + SPIM_CTRL);

	release_spim_device(dev);
}

static void ast1060_monitor_enable(const struct device *dev, bool enable)
{
	bool pt_en = enable ? false : true;

	ast1060_ctrl_monitor_config(dev, enable);
	ast1060_miso_multi_func_adjust(dev, enable);
	ast1060_passthrough_config(dev, SPIM_SINGLE_PASSTHROUGH, pt_en);
}

/* Try to get the decoding window range for each CS */
static void ast2700_spi_decoding_win_config_common(const struct device *dev, bool lock)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t spic_base = config->ctrl_base - SPIM_SPIC_CONCAT_OFFSET;
	uint32_t spim_base = config->ctrl_base;
	uint32_t spic_win;
	uint32_t spim_win;
	uint32_t val;
	uint32_t i;

	acquire_spim_device(dev);

	for (i = 0; i < 4; i++) {
		spic_win = sys_read32(spic_base + SPI_CTRL_WIN + i * 4);
		spim_win = (spic_win & 0xffff) |
			   (((spic_win & 0xffff0000) - 1) &
			    0xffff0000);

		sys_write32(spim_win, spim_base + SPIM_CS0_BASE + i * 4);
	}

	/*
	 * Write-protected address decoding range registers
	 * to avoid hacker modifing deliberately
	 */
	if (lock) {
		val = sys_read32(spic_base + SPI_CTRL_LOCK_SOC);
		val |= GENMASK(22, 20);
		sys_write32(val, spic_base + SPI_CTRL_LOCK_SOC);
	}

	release_spim_device(dev);
}

static void ast2700_spi_decoding_win_config(const struct device *dev)
{
	ast2700_spi_decoding_win_config_common(dev, true);
}

static void ast10x0_g2_spi_decoding_win_config(const struct device *dev)
{
	ast2700_spi_decoding_win_config_common(dev, false);
}

/*
 * On AST1080, unlike AST2700, the FMC/SPI0/SPI1 controller register
 * blocks are NOT adjacent to the SPI monitor channel's own MMIO
 * window, so "ctrl_base - SPIM_SPIC_CONCAT_OFFSET" does not resolve
 * to the real controller and must not be reused here.
 *
 * Each SPI monitor channel only snoops a 2-CS slice of one real
 * controller (HW-verified mapping):
 *   ctrl_idx 1 (spim1) -> SPI0 CS0/CS1  (spi0 window index 0,1)
 *   ctrl_idx 2 (spim2) -> SPI0 CS2/CS3  (spi0 window index 2,3)
 *   ctrl_idx 3 (spim3) -> SPI1 CS0/CS1  (spi1 window index 0,1)
 */
struct ast1080_spim_bus_map {
	uint32_t ctrl_base; /* real SPI0/SPI1 controller ctrl_reg base */
	uint32_t cs_start;  /* first CS window index owned by this channel */
};

static const struct ast1080_spim_bus_map ast1080_spim_bus_map[] = {
	{ .ctrl_base = 0x74010000, .cs_start = 0 }, /* ctrl_idx 1: spi0 CS0/CS1 */
	{ .ctrl_base = 0x74010000, .cs_start = 2 }, /* ctrl_idx 2: spi0 CS2/CS3 */
	{ .ctrl_base = 0x74020000, .cs_start = 0 }, /* ctrl_idx 3: spi1 CS0/CS1 */
};

static void ast1080_spi_decoding_win_config(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	const struct ast1080_spim_bus_map *map =
		&ast1080_spim_bus_map[config->ctrl_idx - 1];
	uint32_t spic_base = map->ctrl_base;
	uint32_t spim_base = config->ctrl_base;
	uint32_t spic_win;
	uint32_t spim_win;
	uint32_t base_offset = 0;
	uint32_t i;

	acquire_spim_device(dev);

	if (map->cs_start > 0) {
		base_offset = sys_read32(spic_base + SPI_CTRL_WIN +
					  (map->cs_start - 1) * 4) & 0xffff0000;
	}

	for (i = 0; i < 2; i++) {
		spic_win = sys_read32(spic_base + SPI_CTRL_WIN +
				       (map->cs_start + i) * 4);
		spim_win = (((spic_win & 0xffff) - (base_offset >> 16)) & 0xffff) |
			   ((((spic_win & 0xffff0000) - base_offset) - 1) &
			    0xffff0000);

		sys_write32(spim_win, spim_base + SPIM_CS0_BASE + i * 4);
	}
	release_spim_device(dev);
}

struct ast1080_spim_gpio_reg {
	uint32_t addr;
	uint32_t mask;
	uint32_t val;
};

/*
 * These SCU2xx pinmux registers are shared across SPIM0/1/2 (spim1/2/3) —
 * e.g. 0x74c0244c carries SPIM0 in its top byte and SPIM2 in the bottom
 * three bytes. Only the bits that were "2" in the old hardcoded writes
 * belong to that SPIM instance; everything else must be preserved via
 * read-modify-write instead of being zeroed out.
 */
static const struct ast1080_spim_gpio_reg ast1080_spim_gpio_cfg[][2] = {
	/* ctrl_idx 1: SPIM0 */
	{
		{ .addr = 0x74c0244c, .mask = 0xFF000000, .val = 0x22000000 },
		{ .addr = 0x74c02450, .mask = 0x000FFFFF, .val = 0x00022222 },
	},
	/* ctrl_idx 2: SPIM1 */
	{
		{ .addr = 0x74c02418, .mask = 0xFFFFF000, .val = 0x22222000 },
		{ .addr = 0x74c0241c, .mask = 0xFF0000FF, .val = 0x22000022 },
	},
	/* ctrl_idx 3: SPIM2 */
	{
		{ .addr = 0x74c02448, .mask = 0xF0000000, .val = 0x20000000 },
		{ .addr = 0x74c0244c, .mask = 0x00FFFFFF, .val = 0x00222222 },
	},
};

static void ast1080_spim_gpio_mode_config(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	const struct ast1080_spim_gpio_reg *cfg;
	uint32_t reg;
	uint32_t i;

	if (config->ctrl_idx < 1 || config->ctrl_idx > ARRAY_SIZE(ast1080_spim_gpio_cfg)) {
		LOG_ERR("[%s] unexpected ctrl_idx %u for SPIM gpio mode config",
			dev->name, config->ctrl_idx);
		return;
	}
	cfg = ast1080_spim_gpio_cfg[config->ctrl_idx - 1];

	acquire_spim_device(dev);

	for (i = 0; i < 2; i++) {
		reg = sys_read32(cfg[i].addr);
		reg = (reg & ~cfg[i].mask) | cfg[i].val;
		sys_write32(reg, cfg[i].addr);
	}

	release_spim_device(dev);
}

static void ast1080_push_pull_mode_config(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;

	acquire_spim_device(dev);

	switch (config->ctrl_idx) {
	case 1: /* SPIM0 */
		sys_write32(0x02050205, 0x74c025bc);
		sys_write32(0x02050205, 0x74c025c0);
		sys_write32(0x02050205, 0x74c025c4);
		sys_write32(0x02040205, 0x74c025c8);
		break;
	case 2: /* SPIM1 */
		sys_write32(0x02050204, 0x74c024e4);
		sys_write32(0x02050205, 0x74c024e8);
		sys_write32(0x02050205, 0x74c024ec);
		sys_write32(0x02050205, 0x74c024f0);
		break;
	case 3: /* SPIM2 */
		sys_write32(0x02050205, 0x74c025AC);
		sys_write32(0x02050205, 0x74c025B0);
		sys_write32(0x02050205, 0x74c025B4);
		sys_write32(0x02050205, 0x74c025B8);
		break;
	default:
		LOG_ERR("[%s] unexpected ctrl_idx %u for push-pull mode config",
			dev->name, config->ctrl_idx);
		break;
	}

	release_spim_device(dev);
}

static void ast2700_push_pull_mode_config(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg;

	acquire_spim_device(dev);

	reg = sys_read32(config->ctrl_base + SPIM_IO_IRQ_CTRL);
	reg |= SPIM_PUSH_PULL_ENABLED;
	sys_write32(reg, config->ctrl_base + SPIM_IO_IRQ_CTRL);

	release_spim_device(dev);
}

/*
 * When an invalid transmission is detected, CS should be
 * inactivated immediately after clock rising edge and
 * before the next clock falling edge.
 */
static void ast2700_blocked_cs_config(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg;

	acquire_spim_device(dev);

	reg = sys_read32(config->ctrl_base + SPIM_CTRL);
	reg |= SPIM_RISING_INACTIVATE;
	sys_write32(reg, config->ctrl_base + SPIM_CTRL);

	release_spim_device(dev);
}

static void ast2700_blocked_fifo_init(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg;

	acquire_spim_device(dev);

	reg = sys_read32(config->ctrl_base + SPIM_CTRL);
	reg |= SPIM_BLOCK_FIFO_CLR;
	sys_write32(reg, config->ctrl_base + SPIM_CTRL);

	k_busy_wait(100);

	reg &= ~SPIM_BLOCK_FIFO_CLR;
	sys_write32(reg, config->ctrl_base + SPIM_CTRL);

	release_spim_device(dev);
}

/* On AST2700, only SSP and BootMCU can
 * access SPI monitor control registers.
 */
static void ast2700_spim_access_prot_init(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t prot_base = config->ctrl_base + 0x400;
	uint32_t id;
	uint32_t reg;

	/* config ID */
	id = SPIM_BOOTMCU_I_ID | SPIM_BOOTMCU_D_ID << 8 |
	     SPIM_SSP_I_ID << 16 | SPIM_SSP_D_ID << 24;
	sys_write32(id, prot_base + SPIM_PROT_MID0);

	id = SPIM_SSP_S_ID | SPIM_DUMMY_M_ID << 8 |
	     SPIM_DUMMY_M_ID << 16 | SPIM_DUMMY_M_ID << 24;
	sys_write32(id, prot_base + SPIM_PROT_MID1);

	/* config read/write permission */
	reg = 0x1f1f;
	sys_write32(reg, prot_base + SPIM_PROT_RW_CTRL);

	/* prot region */
	reg = (0x400 / 4) | (0x800 / 4) << 16;
	sys_write32(reg, prot_base + SPIM_PROT_ADDR_CTRL);
}

static void ast2700_elec_char_init(const struct device *dev)
{
	spim_sw_rst(dev);
	ast2700_push_pull_mode_config(dev);
	ast2700_spi_decoding_win_config(dev);
	ast2700_blocked_cs_config(dev);
	ast2700_blocked_fifo_init(dev);
	ast2700_spim_access_prot_init(dev);
}

static void ast1080_monitor_elec_char_init(const struct device *dev)
{
	spim_sw_rst(dev);
	ast1080_push_pull_mode_config(dev);
	ast1080_spi_decoding_win_config(dev);
	ast2700_blocked_cs_config(dev);
	ast2700_blocked_fifo_init(dev);
	ast1080_spim_gpio_mode_config(dev);
}

static void ast10x0_g2_filter_elec_char_init(const struct device *dev)
{
	spim_sw_rst(dev);
	ast10x0_g2_spi_decoding_win_config(dev);
	ast2700_blocked_cs_config(dev);
	ast2700_blocked_fifo_init(dev);
}

#define ADDR_CTRL_REG0(base, idx) ((base) + (idx) * 8)
#define ADDR_CTRL_REG1(base, idx) ((base) + (idx) * 8 + 4)

static inline uint32_t ast2700_addr_priv_start(uint32_t base, int idx)
{
	return (sys_read32(base + idx * 8) & SPIM_ADDR_PRIV_START_MASK);
}

static inline uint32_t ast2700_addr_priv_len(uint32_t base, int idx)
{
	return (sys_read32(base + idx * 8 + 4) & SPIM_ADDR_PRIV_LEN_MASK) <<
	       SPIM_ADDR_PRIV_LEN_SHIFT;
}

static inline uint32_t ast2700_addr_priv_end(uint32_t base, int idx)
{
	return ((sys_read32(base + idx * 8) & SPIM_ADDR_PRIV_START_MASK) +
		((sys_read32(base + idx * 8 + 4) & SPIM_ADDR_PRIV_LEN_MASK) <<
		 SPIM_ADDR_PRIV_LEN_SHIFT));
}

static inline bool ast2700_addr_priv_vld(uint32_t base, int idx)
{
	return !!(sys_read32(base + idx * 8) & SPIM_ADDR_PRIV_VALID);
}

static inline uint32_t ast2700_addr_priv_lock(uint32_t base, int idx)
{
	return !!(sys_read32(base + idx * 8 + 4) & SPIM_ADDR_PRIV_LOCK);
}

void ast2700_spim_blocked_log_parser(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t log;
	uint32_t len;

	len = (sys_read32(config->ctrl_base + SPIM_CTRL) &
	       SPIM_BLOCK_FIFO_LEN) >> 24;
	if (len > 4)
		len = 4;

	while (len > 0) {
		log = sys_read32(config->ctrl_base + SPIM_FIFO);
		switch ((log & 0xc0000000) >> 30) {
		case 0x0:
			/* block command */
			printk("[%s][b][cmd] %02xh\n",
			       dev->name, (log & 0xff));
			break;

		case 0x1:
			/* block write command */
			printk("[%s][b][w_addr] 0x%08x\n",
			       dev->name, (log & 0xfffff) << 12);
			break;

		case 0x2:
			/* block read command */
			printk("[%s][b][r_addr] 0x%08x\n",
			       dev->name, (log & 0xfffff) << 12);
			break;

		default:
			printk("[%s]invalid ctx: 0x%08x", dev->name, log);
		}

		len--;
	}
}

static uint32_t ast2700_addr_priv_region_overlay(const struct device *dev,
					  uint32_t idx_off,
					  uint32_t start, uint32_t len)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t addr_priv_base = config->ctrl_base + SPIM_ADDR_PRIV_TABLE_BASE;
	uint32_t end = start + len;
	uint32_t reg_start, reg_end;

	for (; idx_off < SPIM_ADDR_PRIV_REGION_NUM; idx_off++) {
		reg_start = ast2700_addr_priv_start(addr_priv_base, idx_off);
		reg_end = ast2700_addr_priv_end(addr_priv_base, idx_off);
		if (!(end <= reg_start || reg_end <= start)) {
			LOG_WRN("overlay with idx %02d, (0x%08x, 0x%08x), (0x%08x, 0x%08x)",
				 idx_off, start, end, reg_start, reg_end);
			return idx_off;
		}
	}

	return SPIM_ADDR_PRIV_REGION_NUM;
}

static uint32_t ast2700_addr_priv_region_full_overlay(const struct device *dev,
					       uint32_t idx_off,
					       uint32_t start,
					       uint32_t len)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t addr_priv_base = config->ctrl_base + SPIM_ADDR_PRIV_TABLE_BASE;
	uint32_t reg_start, reg_len;

	for (; idx_off < SPIM_ADDR_PRIV_REGION_NUM; idx_off++) {
		reg_start = ast2700_addr_priv_start(addr_priv_base, idx_off);
		reg_len = ast2700_addr_priv_len(addr_priv_base, idx_off);
		if (reg_start == start && reg_len == len)
			return idx_off;
	}

	return SPIM_ADDR_PRIV_REGION_NUM;
}

int ast2700_address_privilege_config(const struct device *dev,
				     uint32_t addr, uint32_t len,
				     uint32_t attr)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t addr_priv_base = config->ctrl_base + SPIM_ADDR_PRIV_TABLE_BASE;
	uint32_t idx;
	uint32_t reg;
	int ret = 0;

	if (!!(addr % KB(4)) || !!(len % KB(4)) || len == 0) {
		LOG_ERR("addr, %x, or len, %x, should be 4KB aligned.",
			 addr, len);
		return -EINVAL;
	}

	if (!(attr & (FLAG_ADDR_PRIV_READ_DIS |
		      FLAG_ADDR_PRIV_WRITE_DIS))) {
		LOG_ERR("Invalid operation.");
		return -EINVAL;
	}

	acquire_spim_device(dev);

	if (ast2700_addr_priv_region_overlay(dev, 0, addr, len) <
	    SPIM_ADDR_PRIV_REGION_NUM) {
		LOG_ERR("region overlay.");
		ret = -EINVAL;
		goto end;
	}

	/* find empty address ctrl slot */
	for (idx = 0; idx < SPIM_ADDR_PRIV_REGION_NUM; idx++) {
		if (ast2700_addr_priv_lock(addr_priv_base, idx) ||
		    ast2700_addr_priv_vld(addr_priv_base, idx))
			continue;

		reg = (addr & SPIM_ADDR_PRIV_START_MASK) | SPIM_ADDR_PRIV_VALID;
		if (attr & FLAG_ADDR_PRIV_READ_DIS)
			reg |= SPIM_ADDR_PRIV_READ_DIS;
		if (attr & FLAG_ADDR_PRIV_WRITE_DIS)
			reg |= SPIM_ADDR_PRIV_WRITE_DIS;

		sys_write32(reg, ADDR_CTRL_REG0(addr_priv_base, idx));

		reg = (len & SPIM_ADDR_PRIV_LEN_RAW_MASK) >>
		      SPIM_ADDR_PRIV_LEN_SHIFT;

		if (attr & FLAG_ADDR_PRIV_TABLE_LOCK)
			reg |= SPIM_ADDR_PRIV_LOCK;

		sys_write32(reg, ADDR_CTRL_REG1(addr_priv_base, idx));

		reg = sys_read32(ADDR_CTRL_REG0(addr_priv_base, idx));
		LOG_INF("[%d] addr 0x%08x with len 0x%08x %s %s.",
			 idx,
			 ast2700_addr_priv_start(addr_priv_base, idx),
			 ast2700_addr_priv_len(addr_priv_base, idx),
			 !!(reg & SPIM_ADDR_PRIV_WRITE_DIS) ?
			 "write_dis" : "write_en",
			 !!(reg & SPIM_ADDR_PRIV_READ_DIS) ?
			 "read_dis" : "read_en");

		break;
	}

	if (idx >= SPIM_ADDR_PRIV_REGION_NUM) {
		LOG_ERR("no more addr ctrl space!");
		ret = -ENOSPC;
		goto end;
	}

end:
	release_spim_device(dev);

	return ret;
}

int ast2700_address_privilege_remove(const struct device *dev,
				     uint32_t addr, uint32_t len)

{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t addr_priv_base = config->ctrl_base + SPIM_ADDR_PRIV_TABLE_BASE;
	uint32_t idx, rm_idx;
	bool found = false;
	bool lock = false;
	int ret = 0;

	if (!!(addr % KB(4)) || !!(len % KB(4)) || len == 0) {
		LOG_ERR("addr %x or len %x must be 4KB aligned and non-zero.",
			addr, len);
		return -EINVAL;
	}

	acquire_spim_device(dev);

	for (idx = 0; idx < SPIM_ADDR_PRIV_REGION_NUM; idx++) {
		rm_idx = ast2700_addr_priv_region_full_overlay(dev, idx,
							       addr, len);
		if (rm_idx >= SPIM_ADDR_PRIV_REGION_NUM)
			break;

		if (ast2700_addr_priv_lock(addr_priv_base, rm_idx)) {
			LOG_ERR("addr ctrl idx %02d is locked, cannot be removed\n",
				rm_idx);
			lock = true;
			break;
		}

		sys_write32(0x0, ADDR_CTRL_REG0(addr_priv_base, rm_idx));
		sys_write32(0x0, ADDR_CTRL_REG1(addr_priv_base, rm_idx));
		found = true;
	}

	if (!found || lock) {
		LOG_ERR("fail to remove addr ctrl, addr: 0x%08x, len: 0x%08x.\n",
			 addr, len);
		ret = -ECANCELED;
		goto end;
	}

end:
	release_spim_device(dev);

	return ret;
}

static void ast2700_addr_priv_remove_all(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t addr_priv_base = config->ctrl_base +
				  SPIM_ADDR_PRIV_TABLE_BASE;
	uint32_t idx;
	uint32_t reg;

	acquire_spim_device(dev);

	for (idx = 0; idx < SPIM_ADDR_PRIV_REGION_NUM; idx++) {
		reg = sys_read32(ADDR_CTRL_REG1(addr_priv_base, idx));
		if (!!(reg & SPIM_ADDR_PRIV_LOCK)) {
			LOG_WRN("idx: %02d, is locked, cannot be cleared.", idx);
			continue;
		}

		sys_write32(0x0, ADDR_CTRL_REG0(addr_priv_base, idx));
		sys_write32(0x0, ADDR_CTRL_REG1(addr_priv_base, idx));
	}

	release_spim_device(dev);
}

static void ast2700_addr_priv_table_lock(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t addr_priv_base = config->ctrl_base +
				  SPIM_ADDR_PRIV_TABLE_BASE;
	uint32_t reg;
	uint32_t idx;

	acquire_spim_device(dev);

	for (idx = 0; idx < SPIM_ADDR_PRIV_REGION_NUM; idx++) {
		reg = sys_read32(ADDR_CTRL_REG1(addr_priv_base, idx));
		reg |= SPIM_ADDR_PRIV_LOCK;
		sys_write32(reg, ADDR_CTRL_REG1(addr_priv_base, idx));
	}

	release_spim_device(dev);
}

static void ast2700_dump_addr_priv_table(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t addr_priv_base = config->ctrl_base +
				  SPIM_ADDR_PRIV_TABLE_BASE;
	uint32_t idx;
	uint32_t reg0, reg1;

	printk("spim addr ctrl dump:\n");
	printk("======================\n");
	for (idx = 0; idx < SPIM_ADDR_PRIV_REGION_NUM; idx++) {
		reg0 = sys_read32(ADDR_CTRL_REG0(addr_priv_base, idx));
		reg1 = sys_read32(ADDR_CTRL_REG1(addr_priv_base, idx));

		if (!reg0 && !(reg1 & ~SPIM_ADDR_PRIV_LOCK))
			continue;

		printk("[%02d]addr: 0x%08x, len: 0x%08x, %s, %s, %s, %s.\n",
		       idx,
		       ast2700_addr_priv_start(addr_priv_base, idx),
		       ast2700_addr_priv_len(addr_priv_base, idx),
		       !!(reg0 & SPIM_ADDR_PRIV_WRITE_DIS) ? "unwritable " : "writable",
		       !!(reg0 & SPIM_ADDR_PRIV_READ_DIS) ? "unreadable" : "readable",
		       !!(reg0 & SPIM_ADDR_PRIV_VALID) ? "valid" : "invalid",
		       !!(reg1 & SPIM_ADDR_PRIV_LOCK) ? "lock" : "unlock");
	}

	printk("======================\n");
}

static void ast2700_addr_priv_init(const struct device *dev)
{
	struct aspeed_spim_data *const data = dev->data;
	int ret = 0;
	uint32_t addr, len, flag;
	uint32_t i;

	ast2700_addr_priv_remove_all(dev);

	if (data->addr_priv_config_num % 3 != 0) {
		LOG_ERR("Wrong read-forbidden-regions setting in .dts.");
		return;
	}

	if (data->addr_priv_config_num > ARRAY_SIZE(data->addr_priv_config)) {
		LOG_ERR("[%s] addr_priv_configs exceeds array size %zu",
			dev->name, ARRAY_SIZE(data->addr_priv_config));
		return;
	}

	for (i = 0; i < data->addr_priv_config_num; i += 3) {
		addr = data->addr_priv_config[i];
		len = data->addr_priv_config[i + 1];
		flag = data->addr_priv_config[i + 2];

		LOG_INF("addr priv: 0x%08x, len: 0x%08x, flag: 0x%02x",
			addr, len, flag);

		ret = ast2700_address_privilege_config(dev, addr,
						       len, flag);
		if (ret)
			LOG_ERR("Fail to config addr priv table!");
	}
}

static void ast2700_monitor_enable(const struct device *dev, bool enable)
{
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg;

	acquire_spim_device(dev);

	reg = sys_read32(config->ctrl_base + SPIM_CTRL);

	if (enable) {
		reg |= SPIM_CTRL_MULTI_PASSTHROUGH |
		       SPIM_ENABLE | SPIM_MUX_SEL;
	} else {
		reg &= ~(SPIM_CTRL_MULTI_PASSTHROUGH |
			 SPIM_ENABLE | SPIM_MUX_SEL);
	}

	sys_write32(reg, config->ctrl_base + SPIM_CTRL);

	release_spim_device(dev);
}

/*
 * On AST1080, elec_char_init() runs at a driver init priority that
 * precedes the flash controller's own probing, so the decoding window
 * it captures from SPI0/SPI1's SPI_CTRL_WIN may still be the
 * power-on-reset default rather than the real per-CS flash size. Redo
 * the decode window snapshot whenever monitoring is (re-)enabled, so a
 * post-boot "spim config enable" picks up the finalized ranges.
 */
static void ast1080_monitor_enable(const struct device *dev, bool enable)
{
	if (enable)
		ast1080_spi_decoding_win_config(dev);

	ast2700_monitor_enable(dev, enable);
}

static void ast10x0_g2_filter_monitor_enable(const struct device *dev, bool enable)
{
	if (enable)
		ast10x0_g2_spi_decoding_win_config(dev);

	ast2700_monitor_enable(dev, enable);
}

void spim_dump_addr_priv_table(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;

	if (config->ops->dump_addr_priv)
		config->ops->dump_addr_priv(dev);
}

void spim_addr_priv_remove_all(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;

	if (config->ops->addr_priv_remove_all)
		config->ops->addr_priv_remove_all(dev);
}

/* dump command information recored in allow command table */
void spim_dump_allow_command_table(const struct device *dev)
{
	uint32_t i;
	const struct aspeed_spim_config *config = dev->config;
	uint32_t reg_val;
	uint32_t addr_len, addr_mode;
	uint32_t dummy_cyc;
	uint32_t data_mode;
	uint8_t cmd;
	uint32_t prog_sz;

	acquire_spim_device(dev);

	for (i = 0; i < SPIM_CMD_TABLE_NUM; i++) {
		reg_val = sys_read32(config->ctrl_base + SPIM_ALLOW_CMD_BASE + i * 4);
		if (reg_val == 0)
			continue;
		printk("[%s]idx %02d: 0x%08x\n", dev->name, i, reg_val);
	}

	printk("\ncmd info:\n");
	for (i = 0; i < SPIM_CMD_TABLE_NUM; i++) {
		reg_val = sys_read32(config->ctrl_base + SPIM_ALLOW_CMD_BASE + i * 4);
		if (reg_val == 0)
			continue;

		cmd = reg_val & SPIM_CMD_TABLE_CMD_MASK;
		addr_mode = (reg_val & SPIM_CMD_TABLE_ADDR_MODE_MASK) >> 8;
		addr_len = (reg_val & SPIM_CMD_TABLE_ADDR_LEN_MASK) >> 10;
		dummy_cyc = (reg_val & SPIM_CMD_TABLE_DUMMY_MASK) >> 16;
		data_mode = (reg_val & SPIM_CMD_TABLE_DATA_MODE_MASK) >> 24;
		prog_sz = (reg_val & SPIM_CMD_TABLE_PROGRAM_SZ_MASK) >> 13;

		printk("cmd: %02x, addr: len(%d)/", cmd, addr_len);
		switch (addr_mode) {
		case 1:
			printk("mode(single),");
			break;
		case 2:
			printk("mode(dual)  ,");
			break;
		case 3:
			printk("mode(quad)  ,");
			break;
		default:
			printk("mode(no)    ,");
		}

		printk(" dummy: %d, data_mode:", dummy_cyc);

		switch (data_mode) {
		case 1:
			printk(" single,");
			break;
		case 2:
			printk(" dual  ,");
			break;
		case 3:
			printk(" quad  ,");
			break;
		default:
			printk(" no    ,");
		}

		printk(" prog_sz: %03ldKB", prog_sz == 0 ? 0 : BIT(prog_sz + 1));
		(reg_val & SPIM_CMD_TABLE_IS_MEM_CMD) != 0 ?
			printk(", mem_op") : printk(",%*c", 7, ' ');
		(reg_val & SPIM_CMD_TABLE_IS_READ_CMD) != 0 ?
			printk(", read") : printk(",%*c", 5, ' ');
		(reg_val & SPIM_CMD_TABLE_IS_WRITE_CMD) != 0 ?
			printk(", write") : printk(",%*c", 6, ' ');
		(reg_val & SPIM_CMD_TABLE_IS_GENERIC_CMD) != 0 ?
			printk(", generic") : printk(",%*c", 8, ' ');
		(reg_val & SPIM_CMD_TABLE_VALID_BIT) != 0 ?
			printk(", valid") : printk(",%*c", 6, ' ');
		(reg_val & SPIM_CMD_TABLE_VALID_ONCE_BIT) != 0 ?
			printk(", valid once") : printk(",%*c", 11, ' ');
		(reg_val & SPIM_CMD_TABLE_LOCK_BIT) != 0 ?
			printk(", locked|") : printk(",%*c|", 7, ' ');
		printk("\n");
	}

	release_spim_device(dev);
}

static uint32_t spim_get_cmd_table_val(uint8_t cmd)
{
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(cmds_array); i++) {
		if (cmds_array[i].cmd == cmd)
			return cmds_array[i].cmd_table_val;
	}

	LOG_ERR("Error: Cannot get item in command table cmd(%02x)\n", cmd);
	return 0;
}

static void spim_allow_cmd_table_init(const struct device *dev,
				  const uint8_t cmd_list[],
				  uint32_t cmd_num, uint32_t flag)
{
	const struct aspeed_spim_config *config = dev->config;
	mm_reg_t table_base = config->ctrl_base + SPIM_ALLOW_CMD_BASE;
	uint32_t i;
	uint32_t reg_val;
	uint32_t idx = 1;

	acquire_spim_device(dev);

	for (i = 0; i < cmd_num; i++) {
		reg_val = spim_get_cmd_table_val(cmd_list[i]);
		LOG_DBG("cmd %02x, val %08x", cmd_list[i], reg_val);
		if (reg_val == 0) {
			LOG_ERR("cmd is not recorded in cmds_array array");
			LOG_ERR("please edit it in spi_monitor_aspeed.c");
			continue;
		}

		if (flag & FLAG_CMD_TABLE_VALID_ONCE)
			reg_val |= SPIM_CMD_TABLE_VALID_ONCE_BIT;
		else
			reg_val |= SPIM_CMD_TABLE_VALID_BIT;

		switch (cmd_list[i]) {
		case CMD_EN4B:
			sys_write32(reg_val, table_base);
			continue;

		case CMD_EX4B:
			sys_write32(reg_val, table_base + 4);
			continue;
		case CMD_WREAR:
			sys_write32(reg_val, table_base + 4 * 31);
			continue;
		default:
			idx++;
		}

		if (idx > 31) {
			LOG_ERR("The allowed command number may exceed the expected.");
			break;
		}

		sys_write32(reg_val, table_base + idx * 4);
	}

	release_spim_device(dev);
}

static int spim_get_empty_allow_cmd_slot(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	int idx;
	uint32_t reg_val;

	for (idx = 2; idx < SPIM_CMD_TABLE_NUM; idx++) {
		reg_val = sys_read32(config->ctrl_base + SPIM_ALLOW_CMD_BASE + idx * 4);
		if (reg_val == 0)
			return idx;
	}

	return -ENOSR;
}

int spim_get_allow_cmd_slot(const struct device *dev,
			    uint8_t cmd, uint32_t start_off)
{
	const struct aspeed_spim_config *config = dev->config;
	int idx;
	uint32_t reg_val;

	for (idx = start_off; idx < SPIM_CMD_TABLE_NUM; idx++) {
		reg_val = sys_read32(config->ctrl_base + SPIM_ALLOW_CMD_BASE + idx * 4);
		if ((reg_val & SPIM_CMD_TABLE_CMD_MASK) == cmd)
			return idx;
	}

	return -ENOSR;
}

/* - If the command already exists in allow command table and
 *   it is disabled, it will be enabled by spim_add_allow_command.
 * - If the command already exists in allow command table and
 *   it is lock, it will not be enabled and an error code will
 *   be returned.
 * - If the command doesn't exist in allow command table, an
 *   empty slot will be found and the command info will be
 *   filled into.
 */

int spim_add_allow_command(const struct device *dev,
	uint8_t cmd, uint32_t flag)
{
	int ret = 0;
	const struct aspeed_spim_config *config = dev->config;
	mm_reg_t table_base = config->ctrl_base + SPIM_ALLOW_CMD_BASE;
	int idx;
	uint32_t off;
	uint32_t reg_val;
	bool found = false;

	acquire_spim_device(dev);

	/* check whether the command is already recorded in allow cmd table */
	for (off = 0; off < SPIM_CMD_TABLE_NUM; off++) {
		idx = spim_get_allow_cmd_slot(dev, cmd, off);
		if (idx >= 0) {
			found = true;
			reg_val = sys_read32(table_base + idx * 4);
			if ((reg_val & SPIM_CMD_TABLE_LOCK_BIT) != 0) {
				LOG_WRN("cmd %02x cannot be enabled in allowed table(%d)",
					cmd, idx);
				off = idx;
				/* search for the next slot with the same command */
				continue;
			} else {
				reg_val &= ~SPIM_CMD_TABLE_VALID_MASK;
				if (flag & FLAG_CMD_TABLE_VALID_ONCE)
					reg_val |= SPIM_CMD_TABLE_VALID_ONCE_BIT;
				else
					reg_val |= SPIM_CMD_TABLE_VALID_BIT;

				sys_write32(reg_val, table_base + idx * 4);
				found = true;
				goto end;
			}
		} else {
			break;
		}
	}

	/* If the cmd already exists in the allow command table and
	 * the register is locked, the same command should not be added again.
	 */
	if (found) {
		LOG_WRN("cmd %02x should not be added in the allow command table again", cmd);
		goto end;
	}

	reg_val = spim_get_cmd_table_val(cmd);
	/* cmd info is not found in allow table array */
	if (reg_val == 0) {
		LOG_ERR("cmd is not recorded in \"cmds_array\" array");
		LOG_ERR("please edit it in spi_monitor_aspeed.c");
		ret = -EINVAL;
		goto end;
	}

	if (flag & FLAG_CMD_TABLE_VALID_ONCE)
		reg_val |= SPIM_CMD_TABLE_VALID_ONCE_BIT;
	else
		reg_val |= SPIM_CMD_TABLE_VALID_BIT;

	switch (cmd) {
	case CMD_EN4B:
		sys_write32(reg_val, table_base);
		goto end;

	case CMD_EX4B:
		sys_write32(reg_val, table_base + 4);
		goto end;

	case CMD_WREAR:
		sys_write32(reg_val, table_base + 4 * 31);
		goto end;

	default:
		break;
	}

	idx = spim_get_empty_allow_cmd_slot(dev);
	if (idx < 0) {
		LOG_ERR("No more space for new command");
		ret = -ENOSR;
		goto end;
	}
	sys_write32(reg_val, table_base + idx * 4);

end:
	release_spim_device(dev);

	return ret;
}

/* All command table slot which command is equal to "cmd"
 * parameter will be removed.
 */
int spim_remove_allow_command(const struct device *dev, uint8_t cmd)
{
	int ret = 0;
	const struct aspeed_spim_config *config = dev->config;
	mm_reg_t table_base = config->ctrl_base + SPIM_ALLOW_CMD_BASE;
	int idx;
	uint32_t off;
	uint32_t reg_val;
	bool found = false;

	acquire_spim_device(dev);

	/* check whether the command is already recorded in allow cmd table */
	for (off = 0; off < SPIM_CMD_TABLE_NUM; off++) {
		idx = spim_get_allow_cmd_slot(dev, cmd, off);
		if (idx >= 0) {
			found = true;
			reg_val = sys_read32(table_base + idx * 4);
			if ((reg_val & SPIM_CMD_TABLE_LOCK_BIT) != 0 &&
				(reg_val & SPIM_CMD_TABLE_VALID_MASK) != 0) {
				LOG_ERR("cmd %02x is locked and cannot be disabled. (%d)",
					cmd, idx);
				ret = -EINVAL;
				goto end;
			} else if ((reg_val & SPIM_CMD_TABLE_LOCK_BIT) == 0) {
				sys_write32(0, table_base + idx * 4);
			} else {
				LOG_INF("cmd %02x is locked and cannot be removed. (%d)",
					cmd, idx);
			}

			off = idx;
			continue;
		} else {
			break;
		}
	}

	if (!found) {
		LOG_ERR("cmd %02x is not found in allow command table", cmd);
		ret = -EINVAL;
		goto end;
	}

end:
	release_spim_device(dev);

	return ret;
}

/* - The overall allow command table will be locked when
 *   flag is FLAG_CMD_TABLE_LOCK_ALL.
 * - All command table slot which command is equal to "cmd"
 *   parameter will be locked.
 */
int spim_lock_allow_command_table(const struct device *dev,
	uint8_t cmd, uint32_t flag)
{
	int ret = 0;
	const struct aspeed_spim_config *config = dev->config;
	mm_reg_t table_base = config->ctrl_base + SPIM_ALLOW_CMD_BASE;
	int idx;
	uint32_t off;
	uint32_t reg_val;
	bool found = false;

	acquire_spim_device(dev);

	if ((flag & FLAG_CMD_TABLE_LOCK_ALL) != 0) {
		for (idx = 0; idx < SPIM_CMD_TABLE_NUM; idx++) {
			reg_val = sys_read32(table_base + idx * 4);
			reg_val |= SPIM_CMD_TABLE_LOCK_BIT;
			sys_write32(reg_val, table_base + idx * 4);
		}
		goto end;
	}

	for (off = 0; off < SPIM_CMD_TABLE_NUM; off++) {
		idx = spim_get_allow_cmd_slot(dev, cmd, off);
		if (idx >= 0) {
			found = true;
			reg_val = sys_read32(table_base + idx * 4);
			if ((reg_val & SPIM_CMD_TABLE_LOCK_BIT) != 0) {
				LOG_INF("cmd %02x is already locked (%d)", cmd, idx);
			} else {
				reg_val |= SPIM_CMD_TABLE_LOCK_BIT;
				sys_write32(reg_val, table_base + idx * 4);
			}

			off = idx;
			continue;
		} else {
			break;
		}
	}

	if (!found) {
		LOG_ERR("cmd %02x is not found in allow command table", cmd);
		ret = -EINVAL;
		goto end;
	}

end:
	release_spim_device(dev);
	return ret;
}

void spim_get_log_info(const struct device *dev, struct spim_log_info *info)
{
	const struct aspeed_spim_config *config = dev->config;
	struct aspeed_spim_data *const data = dev->data;

	if (!info)
		return;

	if (data->log_info.log_max_sz == 0 || data->log_info.log_ram_addr == 0) {
		LOG_ERR("spim_get_log_info: RAM log not supported on this SoC");
		memset(info, 0, sizeof(*info));
		return;
	}

	info->log_ram_addr = data->log_info.log_ram_addr;
	info->log_max_sz = data->log_info.log_max_sz;
	info->log_idx_reg = sys_read32(config->ctrl_base + SPIM_LOG_PTR);
}

uint32_t spim_get_ctrl_idx(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;

	return config->ctrl_idx;
}

void spim_isr_callback_install(const struct device *dev,
	spim_isr_callback_t isr_callback)
{
	struct aspeed_spim_data *const data = dev->data;

	data->isr_callback = isr_callback;
}

static void spim_isr(const void *param)
{
	const struct device *dev = param;
	const struct aspeed_spim_config *config = dev->config;
	struct aspeed_spim_data *const data = dev->data;
	uint32_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->irq_ctrl_lock);

	if (data->isr_callback)
		data->isr_callback(dev);

	/* ack */
	reg_val = sys_read32(config->ctrl_base + SPIM_IO_IRQ_CTRL);
	sys_write32(reg_val | SPIM_IRQ_STS_MASK, config->ctrl_base + SPIM_IO_IRQ_CTRL);

	k_spin_unlock(&data->irq_ctrl_lock, key);
}

static void spim_irq_enable(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	struct aspeed_spim_data *const data = dev->data;
	uint32_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->irq_ctrl_lock);

	reg_val = sys_read32(config->ctrl_base + SPIM_IO_IRQ_CTRL);
	reg_val |= (SPIM_CMD_BLOCK_IRQ_EN | SPIM_WRITE_BLOCK_IRQ_EN |
				SPIM_READ_BLOCK_IRQ_EN);
	sys_write32(reg_val, config->ctrl_base + SPIM_IO_IRQ_CTRL);

	k_spin_unlock(&data->irq_ctrl_lock, key);
}

void spim_ext_mux_config(const struct device *dev, enum spim_ext_mux_sel mux_sel)
{
	const struct aspeed_spim_config *config = dev->config;

	if (config->ops->mux_config)
		config->ops->mux_config(dev, mux_sel);
}

void spim_lock_common(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;

	spim_lock_allow_command_table(dev, 0, FLAG_CMD_TABLE_LOCK_ALL);
	if (config->ops->addr_priv_lock)
		config->ops->addr_priv_lock(dev);
	if (config->ops->misc_lock)
		config->ops->misc_lock(dev);
}

void aspeed_spi_monitor_sw_rst(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;

	if (config->ops->ctrl_sw_rst)
		config->ops->ctrl_sw_rst(dev);
}

void spim_monitor_enable(const struct device *dev, bool enable)
{
	const struct aspeed_spim_config *config = dev->config;

	if (config->ops->monitor_enable)
		config->ops->monitor_enable(dev, enable);
}

static int spi_monitor_init(const struct device *dev)
{
	const struct aspeed_spim_config *config = dev->config;
	struct aspeed_spim_data *const data = dev->data;
	int ret = 0;

	if (IS_ENABLED(CONFIG_MULTITHREADING))
		k_sem_init(&data->sem_spim, 1, 1);

	if (!config->ops || !config->ops->elec_char_init ||
	    !config->ops->allow_cmd_table_init || !config->ops->addr_priv_init ||
	    !config->ops->monitor_enable) {
		LOG_ERR("Incomplete init callback functions");
		return -EINVAL;
	}

	config->ops->elec_char_init(dev);
	config->ops->allow_cmd_table_init(dev, data->allow_cmd_list,
					  data->allow_cmd_num, 0);
	config->ops->addr_priv_init(dev);
	config->ops->monitor_enable(dev, true);

	/* log info init */
	if (config->ops->blocked_log_init) {
		ret = config->ops->blocked_log_init(dev);
		if (ret != 0)
			return ret;
	} else {
		/*
		 * SoCs without the AST1060-style RAM log (AST1080/AST2700)
		 * surface blocked transactions through a small HW FIFO
		 * instead. Install the default FIFO drain/print handler so
		 * blocked commands are visible on the ISR path even if the
		 * application never calls spim_isr_callback_install() itself.
		 * The application can still override this later.
		 */
		data->isr_callback = ast2700_spim_blocked_log_parser;
	}

	/* irq init */
	if (config->irq_config_func) {
		config->irq_config_func(dev);
		spim_irq_enable(dev);
	}

	/* multi-function init */
	if (config->pcfg) {
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			LOG_ERR("[%s] fail to configure multi function pin",
				dev->name);
			return ret;
		}
	}

	if (config->force_rel_flash_rst && config->ops->flash_rst_release)
		config->ops->flash_rst_release(dev);

	return 0;
}

static const __maybe_unused struct aspeed_spim_soc_ops ast1060_spim_ops = {
	.addr_priv_init    = ast1060_addr_priv_init,
	.elec_char_init    = ast1060_elec_char_init,
	.allow_cmd_table_init = spim_allow_cmd_table_init,
	.monitor_enable    = ast1060_monitor_enable,
	.ctrl_sw_rst       = spim_sw_rst,
#if CONFIG_ASPEED_SPIM_LOG_SIZE > 0
	.blocked_log_init  = ast1060_ram_log_init,
#else
	.blocked_log_init  = NULL,
#endif
	.flash_rst_release = ast1060_release_flash_rst,
	.mux_config        = ast1060_ext_mux_config,
	.dump_addr_priv    = ast1060_dump_addr_priv_table,
	.addr_priv_lock    = ast1060_addr_priv_lock,
	.addr_priv_remove_all = ast1060_addr_priv_remove_all,
	.misc_lock         = ast1060_misc_lock,
};

/* For AST1080 SPI Monitor */
static const __maybe_unused struct aspeed_spim_soc_ops ast1080_spim_ops = {
	.addr_priv_init    = ast2700_addr_priv_init,
	.elec_char_init    = ast1080_monitor_elec_char_init,
	.allow_cmd_table_init = spim_allow_cmd_table_init,
	.monitor_enable    = ast1080_monitor_enable,
	.ctrl_sw_rst       = spim_sw_rst,
	.blocked_log_init  = NULL,
	.flash_rst_release = NULL,
	.mux_config        = ast1080_ext_mux_config,
	.dump_addr_priv    = ast2700_dump_addr_priv_table,
	.addr_priv_lock    = ast2700_addr_priv_table_lock,
	.addr_priv_remove_all = ast2700_addr_priv_remove_all,
	.misc_lock         = NULL,
};

/* For AST10X0_G2 SPI Filter */
static const __maybe_unused struct aspeed_spim_soc_ops ast10x0_g2_spif_ops = {
	.addr_priv_init    = ast2700_addr_priv_init,
	.elec_char_init    = ast10x0_g2_filter_elec_char_init,
	.allow_cmd_table_init = spim_allow_cmd_table_init,
	.monitor_enable    = ast10x0_g2_filter_monitor_enable,
	.ctrl_sw_rst       = spim_sw_rst,
	.blocked_log_init  = NULL,
	.flash_rst_release = NULL,
	.mux_config        = NULL,
	.dump_addr_priv    = ast2700_dump_addr_priv_table,
	.addr_priv_lock    = ast2700_addr_priv_table_lock,
	.addr_priv_remove_all = ast2700_addr_priv_remove_all,
	.misc_lock         = NULL,
};

static const __maybe_unused struct aspeed_spim_soc_ops ast2700_spim_ops = {
	.addr_priv_init    = ast2700_addr_priv_init,
	.elec_char_init    = ast2700_elec_char_init,
	.allow_cmd_table_init = spim_allow_cmd_table_init,
	.monitor_enable    = ast2700_monitor_enable,
	.ctrl_sw_rst       = spim_sw_rst,
	.blocked_log_init  = NULL,
	.flash_rst_release = NULL,
	.mux_config        = NULL,
	.dump_addr_priv    = ast2700_dump_addr_priv_table,
	.addr_priv_lock    = ast2700_addr_priv_table_lock,
	.addr_priv_remove_all = ast2700_addr_priv_remove_all,
	.misc_lock         = NULL,
};

static int aspeed_spi_monitor_common_init(const struct device *dev)
{
	struct aspeed_spim_common_data *const data = dev->data;

	data->cur_log_sz = 0;
	if (IS_ENABLED(CONFIG_MULTITHREADING))
		k_sem_init(&data->sem_log_op, 1, 1);

#if CONFIG_ASPEED_SPIM_LOG_SIZE > 0
	memset(spim_log_arr, 0x0, CONFIG_ASPEED_SPIM_LOG_SIZE);
#endif

	return 0;
}

/* ===== Shared leaf helpers (one instance per child node) ===== */
#define SPIM_ENUM(node_id) node_id,

#define ASPEED_SPIM_IRQ_DEFINE(node_id)					\
	static void spim_irq_config_##node_id(const struct device *dev)	\
	{								\
		ARG_UNUSED(dev);					\
		IRQ_CONNECT(DT_IRQN(node_id),				\
			    DT_IRQ(node_id, priority),			\
			    spim_isr,					\
			    DEVICE_DT_GET(node_id),			\
			    0);						\
		irq_enable(DT_IRQN(node_id));				\
	}

/* GPIO array for ext mux select; only defined when property is present */
#define SPIM_EXT_MUX_SEL_GPIOS(node_id)				\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, ext_mux_sel_gpios),	\
		(static const struct gpio_dt_spec			\
		spim_ext_mux_sel_gpios_##node_id[] = {			\
			DT_FOREACH_PROP_ELEM_SEP(node_id,		\
						 ext_mux_sel_gpios,	\
						 GPIO_DT_SPEC_GET_BY_IDX,\
						 (,))			\
		};), ())

/* no-op for nodes without pinctrl-0 */
#define ASPEED_PINCTRL_DT_NODE_DEFINE(node_id)				\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, pinctrl_0),		\
		    (PINCTRL_DT_DEFINE(node_id);), ())

/* ===== Shared base macros ===== */
#define ASPEED_SPIM_DEV_CFG_BASE(node_id, soc_ops) {			\
	.ctrl_base       = DT_REG_ADDR(DT_PARENT(node_id)) +		\
			   0x1000 * (DT_REG_ADDR(node_id) - 1),	\
	.irq_num         = DT_IRQN(node_id),				\
	.irq_priority    = DT_IRQ(node_id, priority),			\
	.irq_config_func = spim_irq_config_##node_id,			\
	.ctrl_idx        = DT_REG_ADDR(node_id),			\
	.parent          = DEVICE_DT_GET(DT_PARENT(node_id)),		\
	.pcfg            = COND_CODE_1(					\
		DT_NODE_HAS_PROP(node_id, pinctrl_0),			\
		(PINCTRL_DT_DEV_CONFIG_GET(node_id)), (NULL)),		\
	.ext_mux_sel_default  = DT_PROP_OR(node_id, ext_mux_sel, 0),	\
	.ext_mux_sel_delay_us =						\
		DT_PROP_OR(node_id, ext_mux_sel_delay_us, 0),		\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, ext_mux_sel_gpios), (	\
		.ext_mux_sel_gpios    = spim_ext_mux_sel_gpios_##node_id,\
		.ext_mux_sel_gpio_num =					\
			DT_PROP_LEN_OR(node_id, ext_mux_sel_gpios, 0),	\
	), ())								\
	.force_rel_flash_rst =						\
		DT_PROP(node_id, force_release_flash_reset),		\
	.ops             = soc_ops,					\
},

/*
 * tag+n together identify which ASPEED_SPIM_COMMON_INIT() expansion this
 * child belongs to: n alone is only unique *within* one DT_DRV_COMPAT (it
 * restarts at 0 for every compatible), so siblings from a different
 * DT_DRV_COMPAT at the same index (e.g. AST1080's single spim_common
 * instance 0 vs AST10X0-G2's fmc_filter/spi0_filter/spi1_filter instances
 * 0/1/2) would otherwise collide on the same aspeed_spim_config_0/
 * aspeed_spim_data_0 names. tag is a fixed per-family literal (ast1060,
 * ast1080, ast2700, ast10x0_g2) supplied by each family's own macros below.
 */
#define ASPEED_SPIM_DT_DEFINE_BASE(node_id, child_prio, tag, n)	\
	DEVICE_DT_DEFINE(node_id, spi_monitor_init, NULL,		\
			 &aspeed_spim_data_##tag##_##n[node_id],	\
			 &aspeed_spim_config_##tag##_##n[node_id],	\
			 POST_KERNEL, child_prio, NULL);

/* ===== Main orchestrator ===== */
#define ASPEED_SPIM_COMMON_INIT(tag, n, cfg_fn, data_fn, define_fn,	\
				 common_prio, mode_ctrl_reg_off)	\
	DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n),			\
				     ASPEED_PINCTRL_DT_NODE_DEFINE)	\
	static struct aspeed_spim_common_config				\
		aspeed_spim_common_config_##tag##_##n = {		\
		.scu_base = DT_REG_ADDR_BY_IDX(			\
			DT_INST_PHANDLE_BY_IDX(n, aspeed_scu, 0), 0),	\
		.mode_ctrl_off = (mode_ctrl_reg_off),			\
	};								\
	static struct aspeed_spim_common_data				\
		aspeed_spim_common_data_##tag##_##n;			\
	DEVICE_DT_INST_DEFINE(n, &aspeed_spi_monitor_common_init, NULL,	\
			      &aspeed_spim_common_data_##tag##_##n,	\
			      &aspeed_spim_common_config_##tag##_##n,	\
			      POST_KERNEL, common_prio, NULL);		\
	DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n),			\
				     SPIM_EXT_MUX_SEL_GPIOS)		\
	DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n),			\
				     ASPEED_SPIM_IRQ_DEFINE)		\
	static const struct aspeed_spim_config aspeed_spim_config_##tag##_##n[] = {	\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n), cfg_fn) };\
	static struct aspeed_spim_data aspeed_spim_data_##tag##_##n[] = {	\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n), data_fn) };\
	enum {								\
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(n), SPIM_ENUM)\
	};								\
	DT_FOREACH_CHILD_STATUS_OKAY_VARGS(DT_DRV_INST(n), define_fn, tag, n)

/* ===== AST1060 ===== */
#define ASPEED_AST1060_SPIM_DEV_CFG(node_id) \
	ASPEED_SPIM_DEV_CFG_BASE(node_id, &ast1060_spim_ops)

#define ASPEED_AST1060_SPIM_DEV_DATA(node_id) {				\
	.allow_cmd_list             = DT_PROP(node_id, allow_cmds),	\
	.allow_cmd_num              = DT_PROP_LEN(node_id, allow_cmds),	\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, read_forbidden_regions), (	\
	.read_forbidden_regions     =					\
		DT_PROP(node_id, read_forbidden_regions),		\
	.read_forbidden_region_num  =					\
		DT_PROP_LEN(node_id, read_forbidden_regions),		\
	), ())								\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, write_forbidden_regions), (	\
	.write_forbidden_regions    =					\
		DT_PROP(node_id, write_forbidden_regions),		\
	.write_forbidden_region_num =					\
		DT_PROP_LEN(node_id, write_forbidden_regions),		\
	), ())								\
	.log_info.log_max_sz        =					\
		DT_PROP_OR(node_id, log_ram_size, 0),		\
	.dev                        = DEVICE_DT_GET(node_id),		\
},

#define ASPEED_AST1060_SPIM_DT_DEFINE(node_id, tag, n) \
	ASPEED_SPIM_DT_DEFINE_BASE(node_id, 71, tag, n)

/*
 * Keep the legacy compatible string for backward compatibility
 * since many customers have adopted this naming.
 */
#define ASPEED_AST1060_SPIM_INIT(n)					\
	ASPEED_SPIM_COMMON_INIT(ast1060, n, ASPEED_AST1060_SPIM_DEV_CFG,	\
				ASPEED_AST1060_SPIM_DEV_DATA,		\
				ASPEED_AST1060_SPIM_DT_DEFINE, 70,	\
				AST1060_SPIM_MODE_SCU_CTRL)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_spi_monitor_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST1060_SPIM_INIT)

/* ===== AST1080 ===== */
#define ASPEED_AST1080_SPIM_DEV_CFG(node_id) \
	ASPEED_SPIM_DEV_CFG_BASE(node_id, &ast1080_spim_ops)

#define ASPEED_AST1080_SPIM_DEV_DATA(node_id) {				\
	.allow_cmd_list       = DT_PROP(node_id, allow_cmds),		\
	.allow_cmd_num        = DT_PROP_LEN(node_id, allow_cmds),	\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, addr_priv_configs), (	\
	.addr_priv_config     = DT_PROP(node_id, addr_priv_configs),	\
	.addr_priv_config_num = DT_PROP_LEN(node_id, addr_priv_configs),\
	), ())								\
	.dev                  = DEVICE_DT_GET(node_id),			\
},

#define ASPEED_AST1080_SPIM_DT_DEFINE(node_id, tag, n) \
	ASPEED_SPIM_DT_DEFINE_BASE(node_id, 79, tag, n)

#define ASPEED_AST1080_SPIM_INIT(n)					\
	ASPEED_SPIM_COMMON_INIT(ast1080, n, ASPEED_AST1080_SPIM_DEV_CFG,	\
				ASPEED_AST1080_SPIM_DEV_DATA,		\
				ASPEED_AST1080_SPIM_DT_DEFINE, 78,	\
				AST1080_SCU_ANALOG_MUX_MODE)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast1080_spi_monitor_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST1080_SPIM_INIT)
#undef DT_DRV_COMPAT

/* ===== AST2700 ===== */
#define ASPEED_AST2700_SPIM_DEV_CFG(node_id) \
	ASPEED_SPIM_DEV_CFG_BASE(node_id, &ast2700_spim_ops)

#define ASPEED_AST2700_SPIM_DEV_DATA(node_id) {				\
	.allow_cmd_list       = DT_PROP(node_id, allow_cmds),		\
	.allow_cmd_num        = DT_PROP_LEN(node_id, allow_cmds),	\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, addr_priv_configs), (	\
	.addr_priv_config     = DT_PROP(node_id, addr_priv_configs),	\
	.addr_priv_config_num = DT_PROP_LEN(node_id, addr_priv_configs),\
	), ())								\
	.dev                  = DEVICE_DT_GET(node_id),			\
},

/* Same spi-monitor-ctrl dependency as AST1080 above; must precede SPI_NOR_INIT_PRIORITY (80). */
#define ASPEED_AST2700_SPIM_DT_DEFINE(node_id, tag, n) \
	ASPEED_SPIM_DT_DEFINE_BASE(node_id, 79, tag, n)

/* AST2700 never calls spim_scu_ctrl_set/clear(); offset unused. */
#define ASPEED_AST2700_SPIM_INIT(n)					\
	ASPEED_SPIM_COMMON_INIT(ast2700, n, ASPEED_AST2700_SPIM_DEV_CFG,	\
				ASPEED_AST2700_SPIM_DEV_DATA,		\
				ASPEED_AST2700_SPIM_DT_DEFINE, 78,	\
				AST1060_SPIM_MODE_SCU_CTRL)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast2700_spi_monitor_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST2700_SPIM_INIT)
#undef DT_DRV_COMPAT

/* ===== AST10X0-G2 SPI Filter ===== */
#define ASPEED_AST10X0_G2_SPIM_DEV_CFG(node_id) \
	ASPEED_SPIM_DEV_CFG_BASE(node_id, &ast10x0_g2_spif_ops)

#define ASPEED_AST10X0_G2_SPIM_DEV_DATA(node_id) {				\
	.allow_cmd_list       = DT_PROP(node_id, allow_cmds),		\
	.allow_cmd_num        = DT_PROP_LEN(node_id, allow_cmds),	\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, addr_priv_configs), (	\
	.addr_priv_config     = DT_PROP(node_id, addr_priv_configs),	\
	.addr_priv_config_num = DT_PROP_LEN(node_id, addr_priv_configs),\
	), ())								\
	.dev                  = DEVICE_DT_GET(node_id),			\
},

/*
 * Same spi-monitor-ctrl dependency as AST1080/AST2700 above; must
 * precede SPI_NOR_INIT_PRIORITY (80).
 */
#define ASPEED_AST10X0_G2_SPIM_DT_DEFINE(node_id, tag, n) \
	ASPEED_SPIM_DT_DEFINE_BASE(node_id, 79, tag, n)

/* AST10X0-G2 never calls spim_scu_ctrl_set/clear(); offset unused. */
#define ASPEED_AST10X0_G2_SPIM_INIT(n)					\
	ASPEED_SPIM_COMMON_INIT(ast10x0_g2, n, ASPEED_AST10X0_G2_SPIM_DEV_CFG,	\
				ASPEED_AST10X0_G2_SPIM_DEV_DATA,	\
				ASPEED_AST10X0_G2_SPIM_DT_DEFINE, 78,	\
				AST1060_SPIM_MODE_SCU_CTRL)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast10x0_g2_spi_filter_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST10X0_G2_SPIM_INIT)
#undef DT_DRV_COMPAT
