/*
 * Copyright (c) 2021 - 2023 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SPI_SPI_ASPEED_H_
#define ZEPHYR_DRIVERS_SPI_SPI_ASPEED_H_

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/spi_nor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/toolchain.h>

#include "spi_context.h"

#define SPI_CALIB_LEN               0x200
#define ASPEED_MAX_CS               5

enum aspeed_ctrl_type {
	BOOT_SPI,
	HOST_SPI,
	NORMAL_SPI
};

struct aspeed_cmd_mode {
	uint32_t normal_read;
	uint32_t normal_write;
	uint32_t user;
};

struct aspeed_spi_decoded_addr {
	mm_reg_t start;
	uint32_t len;
};

struct aspeed_spim_internal_mux_ctrl {
	uint32_t master_idx;
	uint32_t spim_output_base;
	const struct device *spi_monitor_common_ctrl;
};

struct aspeed_spi_config;
struct aspeed_spi_data;
struct pinctrl_dev_config;

struct aspeed_spi_ops {
	void (*init_data)(const struct aspeed_spi_config *config,
			  struct aspeed_spi_data *data);

	int (*pinctrl_init)(const struct device *dev);

	void (*proprietary_config_init)(const struct aspeed_spi_config *config,
					struct aspeed_spi_data *data);

	void (*enable_4byte_mode)(const struct device *dev, uint32_t cs);

	void (*cs_group_select)(const struct device *dev, uint32_t cs);

	void (*safs_read_config)(const struct device *dev,
				 const struct spi_nor_op_info *op_info);
	void (*safs_write_config)(const struct device *dev,
				  const struct spi_nor_op_info *op_info);
#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	bool (*dma_xfer_eligible)(const struct device *dev,
			    const struct spi_nor_op_info *op_info);
	void (*read_dma)(const struct device *dev, const struct spi_config *spi_cfg,
		struct spi_nor_op_info op_info);
	void (*write_dma)(const struct device *dev, const struct spi_config *spi_cfg,
		struct spi_nor_op_info op_info);
	void (*cache_flush)(const void *buf, size_t len);
	void (*cache_invd)(const void *buf, size_t len);
#endif
};

struct aspeed_spi_config {
	mm_reg_t ctrl_base;
	mm_reg_t spi_mmap_base;
	uint32_t max_cs;
	enum aspeed_ctrl_type ctrl_type;
	const struct device *clock_dev;
	const clock_control_subsys_t clk_id;
	void (*irq_config_func)(const struct device *dev);
	const struct aspeed_spi_ops *ops;
	bool timing_calibration_disabled;
	uint32_t timing_calibration_start_off;
	struct aspeed_spim_internal_mux_ctrl mux_ctrl;
	bool aspeed_spim_proprietary_config_enable;
	bool pure_spi_mode_only;
	bool spi_ctrl_fifo_enabled;
	bool cs_group_analog_mux_enable;
	const struct pinctrl_dev_config *pcfg;
};

struct aspeed_spi_data {
	struct spi_context ctx;
	struct aspeed_spi_decoded_addr decode_addr[ASPEED_MAX_CS];
	struct aspeed_cmd_mode cmd_mode[ASPEED_MAX_CS];

	uint32_t hclk;
	uint32_t decode_base;
	uint32_t max_decode_sz;
	uint32_t decode_unit_sz;

	uint32_t (*segment_start)(uint32_t val);
	uint32_t (*segment_end)(uint32_t val);
	uint32_t (*segment_value)(uint32_t start, uint32_t end);
	void (*decode_range_pre_init)(const struct aspeed_spi_config *config,
		struct aspeed_spi_data *data);
	int (*decode_range_reinit)(const struct device *dev, uint32_t flash_sz);

	void (*aspeed_spim_proprietary_pre_config)(void);
	void (*aspeed_spim_proprietary_post_config)(const struct device *dev, uint32_t cs);

#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	/* Buffer pending a post-completion cache invalidate in the DMA ISR. */
	void *dma_pending_buf;
	size_t dma_pending_len;
#endif
};

void ast1060_spi_proprietary_config_init(const struct aspeed_spi_config *config,
					 struct aspeed_spi_data *data);
void ast10x0_g2_spi_proprietary_config_init(const struct aspeed_spi_config *config,
					 struct aspeed_spi_data *data);
void ast10x0_g2_spi_cs_group_select(const struct device *dev, uint32_t cs);

#endif /* ZEPHYR_DRIVERS_SPI_SPI_ASPEED_H_ */
