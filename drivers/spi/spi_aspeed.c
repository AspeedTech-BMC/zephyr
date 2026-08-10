/*
 * Copyright (c) 2021 - 2023 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <errno.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_aspeed, CONFIG_SPI_LOG_LEVEL);
#include "spi_aspeed.h"
#include <zephyr/cache.h>
#include <zephyr/drivers/misc/aspeed/pfr_aspeed.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/__assert.h>
#include <soc.h>

#define SPI00_CE_TYPE_SETTING       (0x0000)
#define SPI04_CE_CTRL               (0x0004)
#define SPI08_INTR_CTRL             (0x0008)
#define SPI0C_CMD_CTRL              (0x000C)

#define SPI10_CE0_CTRL              (0x0010)
#define SPI14_CE1_CTRL              (0x0014)
#define SPI18_CE2_CTRL              (0x0018)
#define DUMMY_REG                   (0x001C)
#define SPI30_CE0_ADDR_DEC          (0x0030)
#define SPI34_CE1_ADDR_DEC          (0x0034)
#define SPI38_CE2_ADDR_DEC          (0x0038)

#define SPI50_SOFT_RST_CTRL         (0x0050)
#define SPI54_MISC_CTRL             (0x0054)

#define SPI60_WDT1                  (0x0060)
#define SPI64_WDT2                  (0x0064)
#define SPI68_WDT2_RELOAD_VAL       (0x0068)
#define SPI6C_WDT2_RESTART          (0x006C)

#define SPI7C_DMA_HI_ADDR_REG       (0x007c)	/* AST2700 */
#define SPI7C_DMA_BUF_LEN           (0x007C)	/* AST1030/AST1060/AST2600 */
#define SPI80_DMA_CTRL              (0x0080)
#define SPI84_DMA_FLASH_ADDR        (0x0084)
#define SPI88_DMA_RAM_ADDR          (0x0088)
#define SPI8C_DMA_LEN               (0x008C)
#define SPI90_CHECKSUM_RESULT       (0x0090)
#define SPI94_CE0_TIMING_CTRL       (0x0094)
#define SPI98_CE1_TIMING_CTRL       (0x0098)
#define SPI9C_CE2_TIMING_CTRL       (0x009C)

#define SPIA0_CMD_FILTER_CTRL       (0x00A0)
#define SPIA4_ADDR_FILTER_CTRL      (0x00A4)
#define SPIA8_REG_LOCK_SRST         (0x00A8)
#define SPIAC_REG_LOCK_WDT          (0x00AC)

#define SPI1F4_SOCRST_LOCK          (0x01F4)
#define SPI200_DATA_FIFO_REG        (0x0200)

/* used for HW SAFS setting */
#define SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4	(0x006c)
#define SPIR74_HOST_DIRECT_ACCESS_CMD_CTRL2	(0x0074)

#define SPI_DMA_IRQ_STS             BIT(19)
#define SPI_DMA_STS                 BIT(11)
#define SPI_DMA_IRQ_EN              BIT(3)
#define SPI_DMA_REQUEST             BIT(31)
#define SPI_DMA_GRANT               BIT(30)
#define SPI_DMA_CALIB_MODE          BIT(3)
#define SPI_DMA_CALC_CKSUM          BIT(2)
#define SPI_DMA_WRITE               BIT(1)
#define SPI_DMA_ENABLE              BIT(0)
#define SPI_DMA_GET_REQ_MAGIC       0xaeed0000
#define SPI_DMA_DISCARD_REQ_MAGIC   0xdeea0000
#define SPI_DMA_TRIGGER_LEN         128
#define SPI_DMA_RAM_MAP_BASE        0x80000000
#define SPI_DMA_FLASH_MAP_BASE      0x60000000

#define SPI_CTRL_FREQ_MASK          0x0F000F00

#define ASPEED_SPI_NORMAL_READ      0x1
#define ASPEED_SPI_NORMAL_WRITE     0x2
#define ASPEED_SPI_USER             0x3
#define ASPEED_SPI_USER_INACTIVE    BIT(2)

#define ASPEED_SPI_SZ_2M            0x200000
#define ASPEED_SPI_SZ_16M           0x1000000
#define ASPEED_SPI_SZ_64M           0x4000000
#define ASPEED_SPI_SZ_256M          0x10000000
#define ASPEED_SPI_SZ_512M          0x20000000
#define ASPEED_SPI_SZ_768M          0x30000000

#define ASPEED_DRAM_PHY_BASE        0x400000000
#define ASPEED_HPRAM_PHY_BASE		0xF0000000
#define ASPEED_IO_SRAM_PHY_BASE     0x14b80000
#define ASPEED_IO_SRAM_SIZE         0x40000


#define ASPEED_SPI_CTRL_VAL(io_mode, opcode, dummy_cycle) \
		((io_mode) | (((opcode) & 0xff) << 16) | (dummy_cycle))

uint32_t ast2600_segment_addr_start(uint32_t reg_val)
{
	return ((reg_val & 0x0ff0) << 16);
}

uint32_t ast2600_segment_addr_end(uint32_t reg_val)
{
	return ((reg_val & 0x0ff00000) | 0x000fffff);
}

uint32_t ast2600_segment_addr_val(uint32_t start, uint32_t end)
{
	return ((((((start) >> 20) << 20) >> 16) & 0xffff) | ((((end) >> 20) << 20) & 0xffff0000));
}

uint32_t ast1030_fmc_segment_addr_start(uint32_t reg_val)
{
	return ((reg_val & 0x0ff8) << 16);
}

uint32_t ast1030_fmc_segment_addr_end(uint32_t reg_val)
{
	return ((reg_val & 0x0ff80000) | 0x0007ffff);
}

uint32_t ast1030_fmc_segment_addr_val(uint32_t start, uint32_t end)
{
	return ((((((start) >> 19) << 19) >> 16) & 0xfff8) | ((((end) >> 19) << 19) & 0xfff80000));
}

uint32_t ast1030_spi_segment_addr_start(uint32_t reg_val)
{
	return ((reg_val & 0x0ff0) << 16);
}

uint32_t ast1030_spi_segment_addr_end(uint32_t reg_val)
{
	return ((reg_val & 0x0ff00000) | 0x000fffff);
}

uint32_t ast1030_spi_segment_addr_val(uint32_t start, uint32_t end)
{
	return ((((((start) >> 20) << 20) >> 16) & 0xffff) | ((((end) >> 20) << 20) & 0xffff0000));
}

uint32_t ast2700_segment_addr_start(uint32_t reg_val)
{
	return ((reg_val & 0x0000ffff) << 16);
}

uint32_t ast2700_segment_addr_end(uint32_t reg_val)
{
	return (reg_val & 0xffff0000) - 1;
}

uint32_t ast2700_segment_addr_val(uint32_t start, uint32_t end)
{
	return (uint32_t)((((start) >> 16) & 0x7fff) |
			 ((end + 1) & 0x7fff0000));
}

uint32_t ast2700_spi_get_fifo_offset(const struct device *dev)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	uint32_t decoding_reg;

	decoding_reg = sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4);

	return (data->segment_start(decoding_reg) / ASPEED_SPI_SZ_16M);
}

static uint32_t aspeed_spi_io_mode(enum jesd216_mode_type mode)
{
	uint32_t reg = 0;

	switch (mode) {
	case JESD216_MODE_111:
	case JESD216_MODE_111_FAST:
		reg = 0x00000000;
		break;
	case JESD216_MODE_112:
		reg = 0x20000000;
		break;
	case JESD216_MODE_122:
		reg = 0x30000000;
		break;
	case JESD216_MODE_114:
		reg = 0x40000000;
		break;
	case JESD216_MODE_144:
		reg = 0x50000000;
		break;
	default:
		LOG_ERR("Unsupported io mode 0x08%x", mode);
		break;
	}

	return reg;
}

static uint32_t aspeed_spi_io_mode_user(uint32_t bus_width)
{
	uint32_t reg;

	switch (bus_width) {
	case 4:
		reg = 0x40000000;
		break;
	case 2:
		reg = 0x20000000;
		break;
	default:
		reg = 0x00000000;
		break;
	}

	return reg;
}

static uint32_t aspeed_spi_cal_dummy_cycle(uint32_t bus_width,
					   uint32_t dummy_cycle)
{
	uint32_t dummy_byte = 0;

	dummy_byte = dummy_cycle / (8 / bus_width);

	return (((dummy_byte & 0x3) << 6) | (((dummy_byte & 0x4) >> 2) << 14));
}

static void aspeed_spi_read_data(uint32_t ahb_addr,
				uint8_t *read_arr,
				uint32_t read_cnt)
{
	int i = 0;
	uint32_t dword;
	uint32_t *read_ptr = (uint32_t *)read_arr;

	if (read_arr) {
		if (((uint32_t)read_ptr & 0xf) == 0) {
			for (i = 0; i < read_cnt; i += 4) {
				if (read_cnt - i < 4)
					break;
				*read_ptr = sys_read32(ahb_addr);
				read_ptr += 1;
			}
		}

		for (; i < read_cnt;) {
			dword = sys_read32(ahb_addr);
			if (i < read_cnt)
				read_arr[i] = dword & 0xff;
			i++;
			if (i < read_cnt)
				read_arr[i] = (dword >> 8) & 0xff;
			i++;
			if (i < read_cnt)
				read_arr[i] = (dword >> 16) & 0xff;
			i++;
			if (i < read_cnt)
				read_arr[i] = (dword >> 24) & 0xff;
			i++;
		}
#if defined(DEBUG)
		LOG_INF("read count: %d", read_cnt);
		for (i = 0; i < read_cnt; i++)
			LOG_INF("[%02x]", read_arr[i]);
#endif
	}
}

static void aspeed_spi_write_data(uint32_t ahb_addr,
				const uint8_t *write_arr,
				uint32_t write_cnt)
{
	int i;
	uint32_t dword;

	if (write_arr) {
#if defined(DEBUG)
		LOG_INF("write count: %d", write_cnt);
		for (i = 0; i < write_cnt; i++)
			LOG_INF("[%02x]", write_arr[i]);
#endif
		for (i = 0; i < write_cnt; i += 4) {
			if ((write_cnt - i) < 4)
				break;
			dword = write_arr[i];
			dword |= write_arr[i + 1] << 8;
			dword |= write_arr[i + 2] << 16;
			dword |= write_arr[i + 3] << 24;
			sys_write32(dword, ahb_addr);
		}

		for (; i < write_cnt; i++)
			sys_write8(write_arr[i], ahb_addr);
	}
}

static void aspeed_spi_start_tx(const struct device *dev)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	mm_reg_t mem_reg = data->decode_addr[cs].start;

	if (!spi_context_tx_buf_on(ctx) && !spi_context_rx_buf_on(ctx)) {
		spi_context_complete(ctx, dev, 0);
		return;
	}

	if (config->spi_ctrl_fifo_enabled) {
		mem_reg = config->ctrl_base + SPI200_DATA_FIFO_REG +
			  ast2700_spi_get_fifo_offset(dev);
	}

	/* active cs */
	sys_write32(data->cmd_mode[cs].user | ASPEED_SPI_USER_INACTIVE,
			config->ctrl_base + SPI10_CE0_CTRL + cs * 4);
	sys_write32(data->cmd_mode[cs].user,
			config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	sys_read32(config->ctrl_base  + SPI10_CE0_CTRL + cs * 4);

	while (ctx->tx_buf && ctx->tx_len > 0) {
		aspeed_spi_write_data(mem_reg, ctx->tx_buf, ctx->tx_len);
		spi_context_update_tx(ctx, 1, ctx->tx_len);
	};

	k_busy_wait(2);

	while (ctx->rx_buf && ctx->rx_len > 0) {
		aspeed_spi_read_data(mem_reg, ctx->rx_buf, ctx->rx_len);
		spi_context_update_rx(ctx, 1, ctx->rx_len);
	};

	sys_write32(data->cmd_mode[cs].user | ASPEED_SPI_USER_INACTIVE,
			config->ctrl_base + SPI10_CE0_CTRL + cs * 4);
	sys_write32(data->cmd_mode[cs].normal_read,
			config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	spi_context_complete(ctx, dev, 0);
}

static void aspeed_spi_nor_transceive_user(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	mm_reg_t mem_reg = data->decode_addr[cs].start;
	uint8_t dummy[12] = {0};
	uint32_t hspi_ctrl;

	ARG_UNUSED(spi_cfg);

	if (config->ops->cs_group_select)
		config->ops->cs_group_select(dev, cs);

#ifdef CONFIG_SPI_MONITOR_ASPEED
	/* change internal MUX */
	if (config->mux_ctrl.master_idx != 0) {
		cs = 0;
		mem_reg = data->decode_addr[cs].start;
		spim_scu_ctrl_set(config->mux_ctrl.spi_monitor_common_ctrl,
				BIT(3), (config->mux_ctrl.master_idx - 1) << 3);
		spim_scu_ctrl_set(config->mux_ctrl.spi_monitor_common_ctrl,
				0x7, config->mux_ctrl.spim_output_base + ctx->config->slave);
	}

	if (data->aspeed_spim_proprietary_pre_config)
		data->aspeed_spim_proprietary_pre_config();
#endif

	if (config->spi_ctrl_fifo_enabled) {
		mem_reg = config->ctrl_base + SPI200_DATA_FIFO_REG +
			  ast2700_spi_get_fifo_offset(dev);
	}

	/* Save current SPI54_MISC_CTRL value and clear SPI54_MISC_CTRL to disable SAFS */
	hspi_ctrl = sys_read32(config->ctrl_base + SPI54_MISC_CTRL);
	sys_write32(0x0, config->ctrl_base + SPI54_MISC_CTRL);

	sys_write32(data->cmd_mode[cs].user | ASPEED_SPI_USER_INACTIVE,
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);
	sys_write32(data->cmd_mode[cs].user, config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	sys_read32(config->ctrl_base  + SPI10_CE0_CTRL + cs * 4);

	/* cmd */
	sys_write32(data->cmd_mode[cs].user |
		aspeed_spi_io_mode_user(JESD216_GET_CMD_BUSWIDTH(op_info.mode)),
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	aspeed_spi_write_data(mem_reg, &op_info.opcode, 1);

	/* addr */
	sys_write32(data->cmd_mode[cs].user |
		aspeed_spi_io_mode_user(JESD216_GET_ADDR_BUSWIDTH(op_info.mode)),
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	if (op_info.addr_len == 3)
		op_info.addr <<= 8;

	op_info.addr = sys_cpu_to_be32(op_info.addr);

	aspeed_spi_write_data(mem_reg, (const uint8_t *)&op_info.addr,
			      op_info.addr_len);

	/* dummy */
	aspeed_spi_write_data(mem_reg, (const uint8_t *)dummy,
		(op_info.dummy_cycle / (8 / JESD216_GET_ADDR_BUSWIDTH(op_info.mode))));

	/* data */
	sys_write32(data->cmd_mode[cs].user |
			aspeed_spi_io_mode_user(JESD216_GET_DATA_BUSWIDTH(op_info.mode)),
			config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	if (op_info.data_direct == SPI_NOR_DATA_DIRECT_IN) {
		/* read data */
		aspeed_spi_read_data(mem_reg, op_info.buf, op_info.data_len);
	} else {
		/* write data */
		aspeed_spi_write_data(mem_reg, op_info.buf, op_info.data_len);
	}

	sys_write32(data->cmd_mode[cs].user | ASPEED_SPI_USER_INACTIVE,
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	sys_write32(data->cmd_mode[cs].normal_read,
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

#ifdef CONFIG_SPI_MONITOR_ASPEED
	if (data->aspeed_spim_proprietary_post_config)
		data->aspeed_spim_proprietary_post_config(dev, cs);

	if (config->mux_ctrl.master_idx != 0)
		spim_scu_ctrl_clear(config->mux_ctrl.spi_monitor_common_ctrl, 0xf);
#endif

	/* Restore the original SPI54_MISC_CTRL setting. */
	sys_write32(hspi_ctrl, config->ctrl_base + SPI54_MISC_CTRL);

	spi_context_complete(ctx, dev, 0);
}

#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
/*
 * DMA buffers are no longer placed in a non-cached memory region, so the
 * driver has to maintain coherency itself: flush CPU-written data out to
 * RAM before the DMA engine reads it, and invalidate stale cache lines
 * after the DMA engine writes to RAM and before the CPU reads it back.
 */
static void aspeed_spi_dma_cache_flush(const void *buf, size_t len)
{
	sys_cache_data_flush_range((void *)buf, len);
}

static void aspeed_spi_dma_cache_invd(const void *buf, size_t len)
{
	size_t align = sys_cache_data_line_size_get();
	uintptr_t start;
	uintptr_t end;

	if (align == 0)
		align = sizeof(uintptr_t);

	start = ROUND_DOWN((uintptr_t)buf, align);
	end = ROUND_UP((uintptr_t)buf + len, align);

	sys_cache_data_invd_range((void *)start, end - start);
}

void aspeed_spi_dma_isr(const void *param)
{
	const struct device *dev = param;
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *const data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	uint32_t reg_val;

#ifdef CONFIG_SPI_MONITOR_ASPEED
	/* change internal MUX */
	if (config->mux_ctrl.master_idx != 0)
		cs = 0;
#endif

	if (!(sys_read32(config->ctrl_base + SPI08_INTR_CTRL) & SPI_DMA_STS))
		LOG_ERR("DMA interrupt status not set");

	/* disable IRQ */
	reg_val = sys_read32(config->ctrl_base + SPI08_INTR_CTRL);
	reg_val &= ~SPI_DMA_IRQ_EN;
	sys_write32(reg_val, config->ctrl_base + SPI08_INTR_CTRL);

	/* disable DMA */
	sys_write32(0x0, config->ctrl_base + SPI80_DMA_CTRL);
	sys_write32(SPI_DMA_DISCARD_REQ_MAGIC, config->ctrl_base + SPI80_DMA_CTRL);

#ifdef CONFIG_SPI_MONITOR_ASPEED
	if (data->aspeed_spim_proprietary_post_config)
		data->aspeed_spim_proprietary_post_config(dev, cs);

	if (config->mux_ctrl.master_idx != 0)
		spim_scu_ctrl_clear(config->mux_ctrl.spi_monitor_common_ctrl, 0xf);
#endif

	sys_write32(data->cmd_mode[cs].normal_read,
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	spi_context_complete(ctx, dev, 0);
}

static void aspeed_dma_irq_enable(const struct device *dev)
{
	const struct aspeed_spi_config *config = dev->config;
	uint32_t reg_val;

	reg_val = sys_read32(config->ctrl_base + SPI08_INTR_CTRL);
	reg_val |= SPI_DMA_IRQ_EN;
	sys_write32(reg_val, config->ctrl_base + SPI08_INTR_CTRL);
}

void aspeed_spi_read_dma(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	uint32_t ctrl_reg;

	ARG_UNUSED(spi_cfg);

#ifdef CONFIG_SPI_MONITOR_ASPEED
	/* change internal MUX */
	if (config->mux_ctrl.master_idx != 0)
		cs = 0;
#endif

	if (op_info.data_len > data->decode_addr[cs].len) {
		LOG_WRN("Invalid read len(0x%08x, 0x%08x)",
			op_info.data_len, data->decode_addr[cs].len);
		spi_context_complete(ctx, dev, 0);
		return;
	}

	if ((op_info.addr % 4) != 0 || ((uint32_t)(op_info.buf) % 4) != 0) {
		LOG_WRN("Address should be 4-byte aligned");
		spi_context_complete(ctx, dev, 0);
		return;
	}

#ifdef CONFIG_SPI_MONITOR_ASPEED
	/* change internal MUX */
	if (config->mux_ctrl.master_idx != 0) {
		spim_scu_ctrl_set(config->mux_ctrl.spi_monitor_common_ctrl,
				BIT(3), (config->mux_ctrl.master_idx - 1) << 3);
		spim_scu_ctrl_set(config->mux_ctrl.spi_monitor_common_ctrl,
				0x7, config->mux_ctrl.spim_output_base + ctx->config->slave);
	}

	if (data->aspeed_spim_proprietary_pre_config)
		data->aspeed_spim_proprietary_pre_config();
#endif

	ctrl_reg = data->cmd_mode[cs].normal_read & SPI_CTRL_FREQ_MASK;
	/* io mode */
	ctrl_reg |= aspeed_spi_io_mode(op_info.mode);
	/* cmd */
	ctrl_reg |= ((uint32_t)op_info.opcode) << 16;
	/* dummy cycle */
	ctrl_reg |= ((uint32_t)(op_info.dummy_cycle /
				(8 / JESD216_GET_ADDR_BUSWIDTH(op_info.mode)))) << 6;
	ctrl_reg |= ASPEED_SPI_NORMAL_READ;
	sys_write32(ctrl_reg, config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	sys_write32(SPI_DMA_GET_REQ_MAGIC, config->ctrl_base + SPI80_DMA_CTRL);
	if (sys_read32(config->ctrl_base + SPI80_DMA_CTRL) & SPI_DMA_REQUEST) {
		while (!(sys_read32(config->ctrl_base + SPI80_DMA_CTRL) & SPI_DMA_GRANT))
			;
	}

	sys_write32(data->decode_addr[cs].start + op_info.addr - SPI_DMA_FLASH_MAP_BASE,
	       config->ctrl_base + SPI84_DMA_FLASH_ADDR);
	sys_write32((uint32_t)(&((uint8_t *)op_info.buf)[0]) + SPI_DMA_RAM_MAP_BASE,
			config->ctrl_base + SPI88_DMA_RAM_ADDR);
	sys_write32(op_info.data_len - 1, config->ctrl_base + SPI8C_DMA_LEN);

	aspeed_dma_irq_enable(dev);

	sys_write32(SPI_DMA_ENABLE, config->ctrl_base + SPI80_DMA_CTRL);
}

void aspeed_spi_write_dma(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	uint32_t ctrl_reg;

	ARG_UNUSED(spi_cfg);

#ifdef CONFIG_SPI_MONITOR_ASPEED
	/* change internal MUX */
	if (config->mux_ctrl.master_idx != 0)
		cs = 0;
#endif

	if (op_info.data_len > data->decode_addr[cs].len) {
		LOG_WRN("Invalid write len(0x%08x, 0x%08x)",
			op_info.data_len, data->decode_addr[cs].len);
		spi_context_complete(ctx, dev, 0);
		return;
	}

	if ((op_info.addr % 4) != 0 || ((uint32_t)(op_info.buf) % 4) != 0) {
		LOG_WRN("Address should be 4-byte aligned");
		spi_context_complete(ctx, dev, 0);
		return;
	}

#ifdef CONFIG_SPI_MONITOR_ASPEED
	/* change internal MUX */
	if (config->mux_ctrl.master_idx != 0) {
		spim_scu_ctrl_set(config->mux_ctrl.spi_monitor_common_ctrl,
				BIT(3), (config->mux_ctrl.master_idx - 1) << 3);
		spim_scu_ctrl_set(config->mux_ctrl.spi_monitor_common_ctrl,
				0x7, config->mux_ctrl.spim_output_base + ctx->config->slave);
	}

	if (data->aspeed_spim_proprietary_pre_config)
		data->aspeed_spim_proprietary_pre_config();
#endif
	ctrl_reg = data->cmd_mode[cs].normal_write;
	/* io mode */
	ctrl_reg |= aspeed_spi_io_mode(op_info.mode);
	/* cmd */
	ctrl_reg |= ((uint32_t)op_info.opcode) << 16;
	/* dummy cycle */
	ctrl_reg |= ((uint32_t)(op_info.dummy_cycle /
				(8 / JESD216_GET_ADDR_BUSWIDTH(op_info.mode)))) << 6;
	ctrl_reg |= ASPEED_SPI_NORMAL_WRITE;
	sys_write32(ctrl_reg, config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	sys_write32(SPI_DMA_GET_REQ_MAGIC, config->ctrl_base + SPI80_DMA_CTRL);
	if (sys_read32(config->ctrl_base + SPI80_DMA_CTRL) & SPI_DMA_REQUEST) {
		while (!(sys_read32(config->ctrl_base + SPI80_DMA_CTRL) & SPI_DMA_GRANT))
			;
	}

	sys_write32(data->decode_addr[cs].start + op_info.addr - SPI_DMA_FLASH_MAP_BASE,
	       config->ctrl_base + SPI84_DMA_FLASH_ADDR);
	sys_write32((uint32_t)(&((uint8_t *)op_info.buf)[0]) + SPI_DMA_RAM_MAP_BASE,
			config->ctrl_base + SPI88_DMA_RAM_ADDR);
	sys_write32(op_info.data_len - 1, config->ctrl_base + SPI8C_DMA_LEN);

	aspeed_dma_irq_enable(dev);

	sys_write32(SPI_DMA_ENABLE | SPI_DMA_WRITE, config->ctrl_base + SPI80_DMA_CTRL);
}

void ast2700_aspeed_spi_dma_isr(const void *param)
{
	const struct device *dev = param;
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *const data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	uint32_t reg_val;

	if (!(sys_read32(config->ctrl_base + SPI08_INTR_CTRL) & SPI_DMA_STS) ||
	    !(sys_read32(config->ctrl_base + SPI08_INTR_CTRL) & SPI_DMA_IRQ_STS)) {
		LOG_ERR("DMA interrupt status not set or DMA completion interrupt missing");
	}

	/* disable IRQ */
	reg_val = sys_read32(config->ctrl_base + SPI08_INTR_CTRL);
	reg_val &= ~SPI_DMA_IRQ_EN;
	sys_write32(reg_val, config->ctrl_base + SPI08_INTR_CTRL);

	/* disable DMA */
	reg_val = sys_read32(config->ctrl_base + SPI08_INTR_CTRL);
	reg_val |= SPI_DMA_IRQ_STS;
	sys_write32(reg_val, config->ctrl_base + SPI08_INTR_CTRL);

	/* A read leaves fresh DMA'd data in RAM that the CPU hasn't seen yet. */
	if (!(sys_read32(config->ctrl_base + SPI80_DMA_CTRL) & SPI_DMA_WRITE) &&
	    data->dma_pending_len != 0) {
		if (config->ops->cache_invd)
			config->ops->cache_invd(data->dma_pending_buf, data->dma_pending_len);
		data->dma_pending_len = 0;
	}

	sys_write32(0x0, config->ctrl_base + SPI80_DMA_CTRL);

	sys_write32(data->cmd_mode[cs].normal_read,
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	spi_context_complete(ctx, dev, 0);
}

void ast2700_aspeed_spi_read_dma(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	uintptr_t dram_virt_addr;
	uint32_t dram_phy_addr;
	uint32_t ctrl_reg;
	uint32_t flash_dma_addr;

	ARG_UNUSED(spi_cfg);

	if (config->ops->cs_group_select)
		config->ops->cs_group_select(dev, cs);

	if (op_info.data_len > data->decode_addr[cs].len) {
		LOG_WRN("Invalid read len(0x%08x, 0x%08x)",
			op_info.data_len, data->decode_addr[cs].len);
		spi_context_complete(ctx, dev, 0);
		return;
	}

	if ((op_info.addr % 4) != 0 || ((uint32_t)(op_info.buf) % 4) != 0) {
		LOG_WRN("Address should be 4-byte aligned");
		spi_context_complete(ctx, dev, 0);
		return;
	}

	ctrl_reg = data->cmd_mode[cs].normal_read & SPI_CTRL_FREQ_MASK;
	/* io mode */
	ctrl_reg |= aspeed_spi_io_mode(op_info.mode);
	/* cmd */
	ctrl_reg |= ((uint32_t)op_info.opcode) << 16;
	/* dummy cycle */
	ctrl_reg |= ((uint32_t)(op_info.dummy_cycle /
				(8 / JESD216_GET_ADDR_BUSWIDTH(op_info.mode)))) << 6;
	ctrl_reg |= ASPEED_SPI_NORMAL_READ;
	sys_write32(ctrl_reg, config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	sys_write32(0x0, config->ctrl_base + SPI80_DMA_CTRL);

	sys_write32(0x0, config->ctrl_base + SPI7C_DMA_HI_ADDR_REG);
	if (TO_PHY_ADDR((uintptr_t)op_info.buf) >= ASPEED_DRAM_PHY_BASE)
		sys_write32(0x4, config->ctrl_base + SPI7C_DMA_HI_ADDR_REG);

	flash_dma_addr = (data->decode_addr[cs].start + op_info.addr -
			  config->spi_mmap_base) & 0x7fffffff;
	sys_write32(flash_dma_addr, config->ctrl_base + SPI84_DMA_FLASH_ADDR);

	dram_virt_addr = (uintptr_t)op_info.buf;
	dram_phy_addr = (uint32_t)TO_PHY_ADDR(dram_virt_addr);

	sys_write32(dram_phy_addr, config->ctrl_base + SPI88_DMA_RAM_ADDR);
	sys_write32(op_info.data_len - 1, config->ctrl_base + SPI8C_DMA_LEN);

	if (config->ops->cache_invd)
		config->ops->cache_invd(op_info.buf, op_info.data_len);

#ifndef CONFIG_SPI_ASPEED_DMA_POLLING_MODE
	data->dma_pending_buf = op_info.buf;
	data->dma_pending_len = op_info.data_len;

	/* enable DMA completion interrupt */
	aspeed_dma_irq_enable(dev);
	sys_write32(SPI_DMA_ENABLE, config->ctrl_base + SPI80_DMA_CTRL);
#else
	sys_write32(SPI_DMA_ENABLE, config->ctrl_base + SPI80_DMA_CTRL);

	/* Polling for DMA completion */
	uint32_t dma_busy;

	do {
		dma_busy = sys_read32(config->ctrl_base + SPI08_INTR_CTRL) &
			   SPI_DMA_STS;
		if (dma_busy == 0)
			k_usleep(1);
	} while (dma_busy == 0);

	sys_write32(0x0, config->ctrl_base + SPI80_DMA_CTRL);
	if (config->ops->cache_invd)
		config->ops->cache_invd(op_info.buf, op_info.data_len);
	spi_context_complete(ctx, dev, 0);
#endif
}

void ast2700_aspeed_spi_write_dma(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	uintptr_t dram_virt_addr;
	uint32_t dram_phy_addr;
	uint32_t ctrl_reg;
	uint32_t flash_dma_addr;

	ARG_UNUSED(spi_cfg);

	if (config->ops->cs_group_select)
		config->ops->cs_group_select(dev, cs);

	if (op_info.data_len > data->decode_addr[cs].len) {
		LOG_WRN("Invalid write len(0x%08x, 0x%08x)",
			op_info.data_len, data->decode_addr[cs].len);
		spi_context_complete(ctx, dev, 0);
		return;
	}

	if ((op_info.addr % 4) != 0 || ((uint32_t)(op_info.buf) % 4) != 0) {
		LOG_WRN("Address should be 4-byte aligned");
		spi_context_complete(ctx, dev, 0);
		return;
	}

	ctrl_reg = data->cmd_mode[cs].normal_write;
	/* io mode */
	ctrl_reg |= aspeed_spi_io_mode(op_info.mode);
	/* cmd */
	ctrl_reg |= ((uint32_t)op_info.opcode) << 16;
	/* dummy cycle */
	ctrl_reg |= ((uint32_t)(op_info.dummy_cycle /
				(8 / JESD216_GET_ADDR_BUSWIDTH(op_info.mode)))) << 6;
	ctrl_reg |= ASPEED_SPI_NORMAL_WRITE;
	sys_write32(ctrl_reg, config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	sys_write32(0x0, config->ctrl_base + SPI80_DMA_CTRL);

	sys_write32(0x0, config->ctrl_base + SPI7C_DMA_HI_ADDR_REG);
	if (TO_PHY_ADDR((uintptr_t)op_info.buf) >= ASPEED_DRAM_PHY_BASE)
		sys_write32(0x4, config->ctrl_base + SPI7C_DMA_HI_ADDR_REG);

	flash_dma_addr = (data->decode_addr[cs].start + op_info.addr -
			  config->spi_mmap_base) & 0x7fffffff;
	sys_write32(flash_dma_addr, config->ctrl_base + SPI84_DMA_FLASH_ADDR);

	dram_virt_addr = (uintptr_t)op_info.buf;
	dram_phy_addr = (uint32_t)TO_PHY_ADDR(dram_virt_addr);

	sys_write32(dram_phy_addr, config->ctrl_base + SPI88_DMA_RAM_ADDR);
	sys_write32(op_info.data_len - 1, config->ctrl_base + SPI8C_DMA_LEN);

	data->dma_pending_len = 0;
	if (config->ops->cache_flush)
		config->ops->cache_flush(op_info.buf, op_info.data_len);

#ifndef CONFIG_SPI_ASPEED_DMA_POLLING_MODE
	/* enable DMA completion interrupt */
	aspeed_dma_irq_enable(dev);
	sys_write32(SPI_DMA_ENABLE | SPI_DMA_WRITE, config->ctrl_base + SPI80_DMA_CTRL);
#else
	sys_write32(SPI_DMA_ENABLE | SPI_DMA_WRITE, config->ctrl_base + SPI80_DMA_CTRL);

	/* Polling for DMA completion */
	uint32_t dma_busy;

	do {
		dma_busy = sys_read32(config->ctrl_base + SPI08_INTR_CTRL) &
			   SPI_DMA_STS;
		if (dma_busy == 0)
			k_usleep(1);
	} while (dma_busy == 0);

	sys_write32(0x0, config->ctrl_base + SPI80_DMA_CTRL);
	spi_context_complete(ctx, dev, 0);
#endif
}

static bool aspeed_spi_dma_xfer_eligible(const struct device *dev,
		const struct spi_nor_op_info *op_info)
{
	const struct aspeed_spi_config *config = dev->config;

	if (config->pure_spi_mode_only)
		return false;

	if (op_info->data_len <= SPI_DMA_TRIGGER_LEN)
		return false;

	if ((op_info->addr % 4) != 0)
		return false;

	if (((uintptr_t)op_info->buf % 4) != 0)
		return false;

	/*
	 * A DMA read invalidates the buffer's cache lines on completion.
	 * That invalidate is destructive, so unless the buffer starts and
	 * ends on cache line boundaries, it would also drop dirty data in
	 * whatever else shares those lines. Buffers that don't satisfy this
	 * fall back to the non-DMA transceive path instead.
	 */
	if (op_info->data_direct == SPI_NOR_DATA_DIRECT_IN) {
		size_t cache_line = sys_cache_data_line_size_get();

		if (cache_line == 0)
			cache_line = sizeof(uintptr_t);

		if (((uintptr_t)op_info->buf % cache_line) != 0)
			return false;

		if ((op_info->data_len % cache_line) != 0)
			return false;
	}

	return true;
}

static bool ast2700_aspeed_spi_dram_region(uintptr_t virt_addr)
{
	uint64_t phy_addr = TO_PHY_ADDR(virt_addr);

	return phy_addr >= ASPEED_DRAM_PHY_BASE;
}

static bool ast2700_spi_dma_xfer_eligible(const struct device *dev,
				const struct spi_nor_op_info *op_info)
{
	if (!aspeed_spi_dma_xfer_eligible(dev, op_info))
		return false;

	return ast2700_aspeed_spi_dram_region((uintptr_t)op_info->buf);
}

static bool ast10x0_g2_aspeed_spi_dram_region(uintptr_t virt_addr)
{
	uint64_t phy_addr = TO_PHY_ADDR(virt_addr);

	return phy_addr >= ASPEED_HPRAM_PHY_BASE;
}

static bool ast10x0_g2_spi_dma_xfer_eligible(const struct device *dev,
				const struct spi_nor_op_info *op_info)
{
	if (!aspeed_spi_dma_xfer_eligible(dev, op_info))
		return false;

	return ast10x0_g2_aspeed_spi_dram_region((uintptr_t)op_info->buf);
}
#endif

static int aspeed_spi_transceive(const struct device *dev,
					    const struct spi_config *spi_cfg,
					    const struct spi_buf_set *tx_bufs,
					    const struct spi_buf_set *rx_bufs)
{
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	spi_context_lock(ctx, false, NULL, NULL, spi_cfg);

	if (!spi_context_configured(ctx, spi_cfg))
		ctx->config = spi_cfg;

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	aspeed_spi_start_tx(dev);

	ret = spi_context_wait_for_completion(ctx);

	spi_context_release(ctx, ret);

	return ret;
}

uint32_t aspeed_get_spi_freq_div(uint32_t bus_clk, uint32_t max_freq)
{
	uint32_t div_arr[16] = {15, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 0};
	uint32_t i, j;
	bool find = false;

	for (i = 0; i < 0xf; i++) {
		for (j = 0; j < 16; j++) {
			if (i == 0 && j == 0)
				continue;

			if (max_freq >= (bus_clk / (j + 1 + (i * 16)))) {
				find = true;
				break;
			}
		}

		if (find)
			break;
	}

	if (i == 0xf && j == 16) {
		LOG_ERR("%s %d cannot get correct frequency division.", __func__, __LINE__);
		return 0;
	}

	return ((i << 24) | (div_arr[j] << 8));
}

/*
 * Check whether the data is not all 0 or 1 in order to
 * avoid calibrate unmount spi-flash.
 */
static bool aspeed_spi_calibration_enable(const uint8_t *buf, uint32_t sz)
{
	const uint32_t *buf_32 = (const uint32_t *)buf;
	uint32_t i;
	uint32_t valid_count = 0;

	for (i = 0; i < (sz / 4); i++) {
		if (buf_32[i] != 0 && buf_32[i] != 0xffffffff)
			valid_count++;
		if (valid_count > 50)
			return true;
	}

	return false;
}

static int aspeed_optimized_timing(uint8_t *buf, uint32_t len)
{
	int i;
	int start = 0, mid_point = 0;
	int max_cnt = 0, cnt = 0;

	for (i = 0; i < len; i++) {
		if (buf[i] == 1) {
			cnt++;
		} else {
			cnt = 0;
			start = i;
		}

		if (max_cnt < cnt) {
			max_cnt = cnt;
			mid_point = start + (cnt / 2);
		}
	}

	/*
	 * In order to get a stable SPI read timing,
	 * abandon the result if the length of longest
	 * consecutive good points is too short.
	 */
	if (max_cnt < 4)
		return -1;

	return mid_point;
}

#define CALIBRATION_RESULT_BUF_LEN	(6 * 17)

static bool aspeed_spi_check_reads(const struct device *dev,
				   struct spi_nor_op_info op_info,
				   uint8_t *buf, uint32_t len)
{
	op_info.buf = (void *)(buf + SPI_CALIB_LEN);
	op_info.data_len = len;
	aspeed_spi_nor_transceive_user(dev, NULL, op_info);

	if (memcmp((void *)buf, (void *)(buf + op_info.data_len),
		   op_info.data_len) != 0)
		return false;

	return true;
}

static K_MUTEX_DEFINE(aspeed_spi_calib_data_buf_lock);

static void aspeed_spi_timing_calibration(const struct device *dev,
				   struct spi_nor_op_info op_info)
{
	struct aspeed_spi_data *data = dev->data;
	const struct aspeed_spi_config *config = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint32_t ctrl_reg = config->ctrl_base;
	uint32_t cs = ctx->config->slave;
	uint32_t max_freq = ctx->config->frequency;
	uint32_t timing_reg = ctrl_reg + SPI94_CE0_TIMING_CTRL + cs * 4;
	/* HCLK/2, ..., HCKL/5 */
	static uint8_t calib_buf[SPI_CALIB_LEN * 2] __aligned(4);
	uint8_t calib_res[CALIBRATION_RESULT_BUF_LEN] = {0};
	uint32_t reg_val;
	uint32_t hdiv = 2, hcycle, delay_ns, timing_val;
	uint32_t hdiv_reg;
	bool pass;
	int calib_point;

	LOG_DBG("device name: %s (%d)", dev->name, cs);

	if (config->timing_calibration_disabled)
		goto no_calib;

	k_mutex_lock(&aspeed_spi_calib_data_buf_lock, K_FOREVER);

	reg_val = sys_read32(timing_reg);
	if (reg_val != 0) {
		LOG_DBG("Already executed calibration.");
		goto unlock_calib;
	}

	if (config->mux_ctrl.master_idx != 0 && cs != 0)
		goto unlock_calib;

#ifdef CONFIG_SPI_MONITOR_ASPEED
	/* change internal MUX */
	if (config->mux_ctrl.master_idx != 0) {
		cs = 0;
		spim_scu_ctrl_set(config->mux_ctrl.spi_monitor_common_ctrl,
				BIT(3), (config->mux_ctrl.master_idx - 1) << 3);
		spim_scu_ctrl_set(config->mux_ctrl.spi_monitor_common_ctrl,
				0x7, config->mux_ctrl.spim_output_base + ctx->config->slave);
	}

	if (data->aspeed_spim_proprietary_pre_config)
		data->aspeed_spim_proprietary_pre_config();
#endif

	LOG_DBG("Calculate timing compensation:");
	/*
	 * use the related low frequency to get check calibration data
	 * and get golden data.
	 */
	data->cmd_mode[cs].user &= ~SPI_CTRL_FREQ_MASK;

	memset(calib_buf, 0x0, SPI_CALIB_LEN * 2);

	op_info.data_direct = SPI_NOR_DATA_DIRECT_IN;
	op_info.addr = config->timing_calibration_start_off;
	op_info.buf = calib_buf;
	op_info.data_len = SPI_CALIB_LEN;
	aspeed_spi_nor_transceive_user(dev, NULL, op_info);

	if (!aspeed_spi_calibration_enable(calib_buf, SPI_CALIB_LEN)) {
		LOG_WRN("Flash data is monotonous, skip calibration.");
		goto unlock_calib;
	}

	/* From HCLK/2 to HCLK/5 */
	for (hdiv = 2; hdiv < 6; hdiv++) {
		if (max_freq < data->hclk / hdiv) {
			LOG_DBG("skipping freq %d", data->hclk / hdiv);
			continue;
		}
		max_freq = data->hclk / hdiv;

		data->cmd_mode[cs].user &= ~SPI_CTRL_FREQ_MASK;
		data->cmd_mode[cs].user |= aspeed_get_spi_freq_div(data->hclk, max_freq);

		sys_write32(0x0, timing_reg);
		pass = aspeed_spi_check_reads(dev, op_info, calib_buf, SPI_CALIB_LEN);
		LOG_DBG("HCLK/%d, no timing compensation: %s", hdiv,
			pass ? "PASS" : "FAIL");

		for (hcycle = 0; hcycle <= 5; hcycle++) {
			/* increase DI delay by the step of 0.5ns */
			LOG_DBG("Delay Enable : hcycle %x", hcycle);
			for (delay_ns = 0; delay_ns <= 0xf; delay_ns++) {
				timing_val = BIT(3) | hcycle | (delay_ns << 4);
				timing_val <<= (hdiv - 2) << 3;
				sys_write32(timing_val, timing_reg);

				pass = aspeed_spi_check_reads(dev, op_info,
							      calib_buf, SPI_CALIB_LEN);
				calib_res[hcycle * 17 + delay_ns] = pass;
				LOG_DBG("HCLK/%d, %d HCLK cycle, %d delay_ns : %s",
					hdiv, hcycle, delay_ns,
					pass ? "PASS" : "FAIL");
			}
		}

		calib_point = aspeed_optimized_timing(calib_res,
						      CALIBRATION_RESULT_BUF_LEN);
		if (calib_point < 0) {
			LOG_INF("cannot get good calibration point.");
			continue;
		}

		hcycle = calib_point / 17;
		delay_ns = calib_point % 17;

		timing_val = (BIT(3) | hcycle | (delay_ns << 4)) << ((hdiv - 2) << 3);
		sys_write32(timing_val, timing_reg);
		LOG_DBG("final hcycle: %d, delay_ns: %d (%08x)",
			hcycle, delay_ns, sys_read32(timing_reg));
		break;
	}

unlock_calib:
	k_mutex_unlock(&aspeed_spi_calib_data_buf_lock);

no_calib:

	if (hdiv == 6)
		max_freq = ctx->config->frequency;

	hdiv_reg = aspeed_get_spi_freq_div(data->hclk, max_freq);

	/* configure SPI clock frequency */
	reg_val = sys_read32(ctrl_reg + SPI10_CE0_CTRL + cs * 4);
	reg_val = (reg_val & (~SPI_CTRL_FREQ_MASK)) | hdiv_reg;
	sys_write32(reg_val, ctrl_reg + SPI10_CE0_CTRL + cs * 4);

	data->cmd_mode[cs].normal_read =
		(data->cmd_mode[cs].normal_read & (~SPI_CTRL_FREQ_MASK)) | hdiv_reg;

	data->cmd_mode[cs].normal_write =
		(data->cmd_mode[cs].normal_write & (~SPI_CTRL_FREQ_MASK)) | hdiv_reg;

	data->cmd_mode[cs].user =
		(data->cmd_mode[cs].user & (~SPI_CTRL_FREQ_MASK)) | hdiv_reg;

	/* add clock setting info for CE ctrl setting */
	LOG_DBG("freq: %dMHz", max_freq / 1000000);

#ifdef CONFIG_SPI_MONITOR_ASPEED
	if (data->aspeed_spim_proprietary_post_config)
		data->aspeed_spim_proprietary_post_config(dev, cs);

	if (config->mux_ctrl.master_idx != 0)
		spim_scu_ctrl_clear(config->mux_ctrl.spi_monitor_common_ctrl, 0xf);
#endif
}

static int aspeed_spi_nor_transceive(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret = 0;

	spi_context_lock(ctx, false, NULL, NULL, spi_cfg);
	if (!spi_context_configured(ctx, spi_cfg))
		ctx->config = spi_cfg;

#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	bool use_dma = false;

	if (config->ops && config->ops->dma_xfer_eligible)
		use_dma = config->ops->dma_xfer_eligible(dev, &op_info);

	if (op_info.data_direct == SPI_NOR_DATA_DIRECT_IN) {
		if (use_dma && config->ops->read_dma)
			config->ops->read_dma(dev, spi_cfg, op_info);
		else
			aspeed_spi_nor_transceive_user(dev, spi_cfg, op_info);
	} else if (op_info.data_direct == SPI_NOR_DATA_DIRECT_OUT) {
#ifdef CONFIG_SPI_DMA_WRITE_SUPPORT_ASPEED
		if (use_dma && config->ops->write_dma)
			config->ops->write_dma(dev, spi_cfg, op_info);
		else
			aspeed_spi_nor_transceive_user(dev, spi_cfg, op_info);
#else
		aspeed_spi_nor_transceive_user(dev, spi_cfg, op_info);
#endif
	}
#else
	ARG_UNUSED(config);
	aspeed_spi_nor_transceive_user(dev, spi_cfg, op_info);
#endif

	ret = spi_context_wait_for_completion(ctx);
	spi_context_release(ctx, ret);

	return ret;
}

static int aspeed_spi_decode_range_reinit_common(const struct device *dev,
						 uint32_t flash_sz)
{
	uint32_t cs, tmp;
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t decode_sz_arr[ASPEED_MAX_CS] = {0};
	uint32_t total_decode_range = 0;
	uint32_t start_addr, end_addr, pre_end_addr = 0;

	/* record original decode range */
	for (cs = 0; cs < config->max_cs; cs++) {
		tmp = sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4);
		if (tmp == 0)
			decode_sz_arr[cs] = 0;
		else
			decode_sz_arr[cs] =
				data->segment_end(tmp) - data->segment_start(tmp) + 1;

		LOG_DBG("ori decode range 0x%08x", decode_sz_arr[cs]);
		total_decode_range += decode_sz_arr[cs];
	}

	/* prepare new decode sz array */
	if (total_decode_range - decode_sz_arr[ctx->config->slave] + flash_sz >
	    data->max_decode_sz) {
		LOG_WRN("decode size out of range 0x%08x", flash_sz);
		return -EINVAL;
	}
	decode_sz_arr[ctx->config->slave] = flash_sz;

	/* modify decode range */
	for (cs = 0; cs < config->max_cs; cs++) {
		if (decode_sz_arr[cs] == 0) {
			sys_write32(0x0,
					config->ctrl_base +
						SPI30_CE0_ADDR_DEC + cs * 4);
			continue;
		}

		start_addr = (cs == 0) ? data->decode_base : pre_end_addr;

		end_addr = start_addr + decode_sz_arr[cs] - 1;

		LOG_DBG("start: 0x%x end: 0x%x (0x%x)",
			start_addr, end_addr, decode_sz_arr[cs]);

		sys_write32(data->segment_value(start_addr, end_addr),
			    config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4);

		if (data->decode_base == 0)
			data->decode_addr[cs].start = start_addr + config->spi_mmap_base;
		else
			data->decode_addr[cs].start = start_addr;

		data->decode_addr[cs].len = decode_sz_arr[cs];

		pre_end_addr = end_addr + 1;
	}

	LOG_DBG("decode reg: <0x%08x, 0x%08x, 0x%08x>",
		sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC),
		sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + 4),
		sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + 8));

	return 0;
}

int aspeed_spi_decode_range_reinit(const struct device *dev,
						uint32_t flash_sz)
{
	return aspeed_spi_decode_range_reinit_common(dev, flash_sz);
}

int ast2700_aspeed_spi_decode_range_reinit(const struct device *dev,
					  uint32_t flash_sz)
{
	return aspeed_spi_decode_range_reinit_common(dev, flash_sz);
}

static int aspeed_spi_nor_read_init(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	int ret = 0;
	struct aspeed_spi_data *data = dev->data;
	const struct aspeed_spi_config *config = dev->config;
	struct spi_context *ctx = &data->ctx;

	spi_context_lock(ctx, false, NULL, NULL, spi_cfg);
	if (!spi_context_configured(ctx, spi_cfg))
		ctx->config = spi_cfg;

	LOG_DBG("[%s] mode %08x, cmd: %x, dummy: %d, frequency: %d",
		__func__, op_info.mode, op_info.opcode, op_info.dummy_cycle,
		ctx->config->frequency);

	/* If internal MUX is used, don't reinit decoded address. */
	if (config->mux_ctrl.master_idx == 0 && !config->pure_spi_mode_only) {
		ret = data->decode_range_reinit(dev, op_info.data_len);
		if (ret != 0)
			goto end;
	}

	data->cmd_mode[ctx->config->slave].normal_read =
		ASPEED_SPI_CTRL_VAL(aspeed_spi_io_mode(op_info.mode),
			op_info.opcode,
			aspeed_spi_cal_dummy_cycle(JESD216_GET_ADDR_BUSWIDTH(op_info.mode),
				op_info.dummy_cycle)) | ASPEED_SPI_NORMAL_READ;
	sys_write32(data->cmd_mode[ctx->config->slave].normal_read,
			config->ctrl_base + SPI10_CE0_CTRL + ctx->config->slave * 4);

	/* set controller to 4-byte mode */
	if (op_info.addr_len == 4 && config->ops->enable_4byte_mode) {
		config->ops->enable_4byte_mode(dev,
								ctx->config->slave);
	}

	if (config->ops->safs_read_config)
		config->ops->safs_read_config(dev, &op_info);

	if (config->ops->pinctrl_post_init) {
		config->ops->pinctrl_post_init(dev,
			JESD216_GET_DATA_BUSWIDTH(op_info.mode));
	}

	aspeed_spi_timing_calibration(dev, op_info);

end:

	spi_context_release(ctx, ret);

	return ret;
}

static int aspeed_spi_nor_write_init(const struct device *dev,
				     const struct spi_config *spi_cfg,
				     struct spi_nor_op_info op_info)
{
	int ret = 0;
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	spi_context_lock(ctx, false, NULL, NULL, spi_cfg);
	if (!spi_context_configured(ctx, spi_cfg))
		ctx->config = spi_cfg;

	data->cmd_mode[ctx->config->slave].normal_write &= SPI_CTRL_FREQ_MASK;
	data->cmd_mode[ctx->config->slave].normal_write |=
		ASPEED_SPI_CTRL_VAL(aspeed_spi_io_mode(op_info.mode),
				     op_info.opcode, 0) |
		ASPEED_SPI_NORMAL_WRITE;

	if (config->ops->safs_write_config)
		config->ops->safs_write_config(dev, &op_info);

	spi_context_release(ctx, ret);

	return ret;
}

static int aspeed_spi_release(const struct device *dev,
				const struct spi_config *spi_cfg)
{
	struct aspeed_spi_data *data = dev->data;

	ARG_UNUSED(spi_cfg);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static void aspeed_spi_decode_range_pre_init_common(const struct aspeed_spi_config *config,
							struct aspeed_spi_data *data)
{
	uint32_t cs;
	uint32_t unit_sz = data->decode_unit_sz;
	uint32_t start_addr, end_addr, pre_end_addr = 0;
	uint32_t max_cs = config->max_cs;
	uint32_t decode_limit;

	/* For PFR device, SPI controller always accesses flash by CS0.
	 * For AST2700, config->mux_ctrl.master_idx is always 0.
	 */
	if (config->mux_ctrl.master_idx != 0) {
		max_cs = 1;
		unit_sz = data->max_decode_sz;
	}

	if (config->pure_spi_mode_only) {
		unit_sz = data->max_decode_sz / config->max_cs;
		unit_sz &= ~(data->decode_unit_sz - 1);
	}

	decode_limit = data->decode_base + data->max_decode_sz;

	for (cs = 0; cs < max_cs; cs++) {
		start_addr = (cs == 0) ? (pre_end_addr + data->decode_base) : pre_end_addr;
		end_addr = start_addr + unit_sz - 1;

		if (decode_limit <= end_addr) {
			LOG_DBG("%s %d smc decode address overflow", __func__, __LINE__);
			sys_write32(0, config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4);
			continue;
		}

		LOG_DBG("cs: %d start: 0x%08x, end: 0x%08x (%08x)",
			cs, start_addr, end_addr, data->segment_value(start_addr, end_addr));

		sys_write32(data->segment_value(start_addr, end_addr),
			config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4);
		LOG_DBG("cs: %d 0x%08x", cs,
			sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4));

		if (data->decode_base == 0)
			data->decode_addr[cs].start = start_addr + config->spi_mmap_base;
		else
			data->decode_addr[cs].start = start_addr;

		data->decode_addr[cs].len = unit_sz;
		pre_end_addr = end_addr + 1;
	}
}

void aspeed_decode_range_pre_init(const struct aspeed_spi_config *config,
				  struct aspeed_spi_data *data)
{
	aspeed_spi_decode_range_pre_init_common(config, data);
}

void ast2700_aspeed_decode_range_pre_init(const struct aspeed_spi_config *config,
				  struct aspeed_spi_data *data)
{
	aspeed_spi_decode_range_pre_init_common(config, data);
}

static void aspeed_spi_enable_4byte_mode(const struct device *dev,
						uint32_t cs)
{
	const struct aspeed_spi_config *config = dev->config;

	sys_write32(sys_read32(config->ctrl_base + SPI04_CE_CTRL) |
				(0x11 << cs),
				config->ctrl_base + SPI04_CE_CTRL);
}

static void ast2700_spi_enable_4byte_mode(const struct device *dev,
						uint32_t cs)
{
	const struct aspeed_spi_config *config = dev->config;

	sys_write32(sys_read32(config->ctrl_base + SPI04_CE_CTRL) |
				(0x11 << cs),
				config->ctrl_base + SPI04_CE_CTRL);

	/*
	 * enable protection for SPI004[0 + ce] and SPI004[4 + ce] from soc reset
	 * to make sure the 4-byte mode setting won't be cleared by soc reset
	 */
	sys_write32(sys_read32(config->ctrl_base + SPI1F4_SOCRST_LOCK) |
		    ((0x11 << 4) << cs),
		    config->ctrl_base + SPI1F4_SOCRST_LOCK);
}

/*
 * Configure SAFS read command for HOST_SPI direct access.
 *
 * The read opcode is programmed into CMD_CTRL4. For 4-byte address mode,
 * the opcode is placed in bits [15:8]; otherwise it is placed in bits [7:0].
 * The read I/O mode field is also updated according to the SPI NOR operation.
 */
static void aspeed_spi_safs_read_config(const struct device *dev,
					const struct spi_nor_op_info *op_info)
{
	const struct aspeed_spi_config *config = dev->config;
	uint32_t reg_val;

	if (config->ctrl_type != HOST_SPI)
		return;

	reg_val = sys_read32(config->ctrl_base +
			     SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4);

	if (op_info->addr_len == 4)
		reg_val = (reg_val & 0xffff00ff) | (op_info->opcode << 8);
	else
		reg_val = (reg_val & 0xffffff00) | op_info->opcode;

	reg_val = (reg_val & 0x0fffffff) | aspeed_spi_io_mode(op_info->mode);

	sys_write32(reg_val, config->ctrl_base +
		    SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4);
}

/*
 * Configure SAFS write command for HOST_SPI direct access.
 *
 * The write I/O mode is programmed into CMD_CTRL4, while the write opcode is
 * programmed into CMD_CTRL2. For 4-byte address mode, the opcode is placed in
 * bits [15:8]; otherwise it is placed in bits [7:0].
 */
static void aspeed_spi_safs_write_config(const struct device *dev,
				const struct spi_nor_op_info *op_info)
{
	const struct aspeed_spi_config *config = dev->config;
	uint32_t reg_val;

	if (config->ctrl_type != HOST_SPI)
		return;

	reg_val = sys_read32(config->ctrl_base +
			     SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4);
	reg_val = (reg_val & 0xf0ffffff) |
		  (aspeed_spi_io_mode(op_info->mode) >> 8);
	sys_write32(reg_val, config->ctrl_base +
		    SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4);

	reg_val = sys_read32(config->ctrl_base +
			     SPIR74_HOST_DIRECT_ACCESS_CMD_CTRL2);

	if (op_info->addr_len == 4)
		reg_val = (reg_val & 0xffff00ff) | (op_info->opcode << 8);
	else
		reg_val = (reg_val & 0xffffff00) | op_info->opcode;

	sys_write32(reg_val, config->ctrl_base +
		    SPIR74_HOST_DIRECT_ACCESS_CMD_CTRL2);
}

static int aspeed_spi_pinctrl_init(const struct device *dev)
{
	const struct aspeed_spi_config *config = dev->config;
	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("[%s] fail to configure multi function pin", dev->name);
		return ret;
	}

	return 0;
}

/* This is for AST1030 and AST1060 initialization */
static void aspeed_spi_init_data(const struct aspeed_spi_config *config,
				  struct aspeed_spi_data *data)
{
	data->decode_base = config->spi_mmap_base;
	data->decode_unit_sz = ASPEED_SPI_SZ_2M;
	data->max_decode_sz = ASPEED_SPI_SZ_256M;
	data->decode_range_pre_init = aspeed_decode_range_pre_init;
	data->decode_range_reinit = aspeed_spi_decode_range_reinit;

	if (config->ctrl_type == BOOT_SPI) {
		data->segment_start = ast1030_fmc_segment_addr_start;
		data->segment_end = ast1030_fmc_segment_addr_end;
		data->segment_value = ast1030_fmc_segment_addr_val;
	} else {
		data->segment_start = ast1030_spi_segment_addr_start;
		data->segment_end = ast1030_spi_segment_addr_end;
		data->segment_value = ast1030_spi_segment_addr_val;
	}
}

static void ast10x0_g2_spi_init_data(const struct aspeed_spi_config *config,
				  struct aspeed_spi_data *data)
{
	ARG_UNUSED(config);

	data->decode_base = 0;
	data->decode_unit_sz = ASPEED_SPI_SZ_64M;
	data->max_decode_sz = ASPEED_SPI_SZ_512M;
	data->decode_range_pre_init = ast2700_aspeed_decode_range_pre_init;
	data->decode_range_reinit = ast2700_aspeed_spi_decode_range_reinit;

	data->segment_start = ast2700_segment_addr_start;
	data->segment_end = ast2700_segment_addr_end;
	data->segment_value = ast2700_segment_addr_val;
}

static void ast2600_spi_init_data(const struct aspeed_spi_config *config,
				  struct aspeed_spi_data *data)
{
	data->decode_base = config->spi_mmap_base;
	data->decode_unit_sz = ASPEED_SPI_SZ_2M;
	data->max_decode_sz = ASPEED_SPI_SZ_256M;
	data->decode_range_pre_init = aspeed_decode_range_pre_init;
	data->decode_range_reinit = aspeed_spi_decode_range_reinit;

	data->segment_start = ast2600_segment_addr_start;
	data->segment_end = ast2600_segment_addr_end;
	data->segment_value = ast2600_segment_addr_val;
}

static void ast2700_spi_init_data(const struct aspeed_spi_config *config,
				  struct aspeed_spi_data *data)
{
	ARG_UNUSED(config);

	data->decode_base = 0;
	data->decode_unit_sz = ASPEED_SPI_SZ_64M;
	data->max_decode_sz = ASPEED_SPI_SZ_768M;
	data->decode_range_pre_init = ast2700_aspeed_decode_range_pre_init;
	data->decode_range_reinit = ast2700_aspeed_spi_decode_range_reinit;

	data->segment_start = ast2700_segment_addr_start;
	data->segment_end = ast2700_segment_addr_end;
	data->segment_value = ast2700_segment_addr_val;
}

static int aspeed_spi_init(const struct device *dev)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	uint32_t cs;
	uint32_t reg_val;
	int ret;

	reg_val = sys_read32(config->ctrl_base + SPI00_CE_TYPE_SETTING);
	for (cs = 0; cs < config->max_cs; cs++) {
		reg_val |= BIT(16 + cs);
		data->cmd_mode[cs].user = ASPEED_SPI_USER;
	}
	sys_write32(reg_val, config->ctrl_base + SPI00_CE_TYPE_SETTING);

	ret = clock_control_get_rate(config->clock_dev, config->clk_id,
			       &data->hclk);
	if (ret != 0)
		return ret;

	if (config->ops && config->ops->init_data)
		config->ops->init_data(config, data);
	data->decode_range_pre_init(config, data);

	spi_context_unlock_unconditionally(&data->ctx);

#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	if (config->irq_config_func)
		config->irq_config_func(dev);
#endif

	if (config->mux_ctrl.master_idx != 0 &&
		(config->mux_ctrl.spim_output_base == 0 ||
		 !config->mux_ctrl.spi_monitor_common_ctrl)) {
		LOG_ERR("[%s]Invalid dts setting for SPI internal mux master",
			dev->name);
		return -EINVAL;
	}

	ret = config->ops->pinctrl_init(dev);
	if (ret != 0)
		return ret;

	if (config->ops->proprietary_config_init)
		config->ops->proprietary_config_init(config, data);

	return 0;
}

static const struct spi_nor_ops aspeed_spi_nor_ops = {
	.transceive = aspeed_spi_nor_transceive,
	.read_init = aspeed_spi_nor_read_init,
	.write_init = aspeed_spi_nor_write_init,
};

static const struct spi_driver_api aspeed_spi_driver_api = {
	.transceive = aspeed_spi_transceive,
	.release = aspeed_spi_release,
	.spi_nor_op = &aspeed_spi_nor_ops,
};

/*
 * Keep a common ops table for backward compatibility with existing
 * AST1030/AST1060 DTBs using the legacy common compatible string.
 *
 * Newer device trees should use SoC-specific compatibles, but existing DTBs
 * may still rely on this path. Preserve the AST1060-specific proprietary
 * setup here to avoid breaking those systems.
 */
static const __maybe_unused struct aspeed_spi_ops aspeed_common_spi_ops = {
	.init_data = aspeed_spi_init_data,
	.pinctrl_init = aspeed_spi_pinctrl_init,
	.pinctrl_post_init = NULL,
	.proprietary_config_init = ast1060_spi_proprietary_config_init,
	.enable_4byte_mode = aspeed_spi_enable_4byte_mode,
	.cs_group_select = NULL,
	.safs_read_config = aspeed_spi_safs_read_config,
	.safs_write_config = aspeed_spi_safs_write_config,
#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	.dma_xfer_eligible = aspeed_spi_dma_xfer_eligible,
	.read_dma = aspeed_spi_read_dma,
	.write_dma = aspeed_spi_write_dma,
#endif
};

static const __maybe_unused struct aspeed_spi_ops ast1030_spi_ops = {
	.init_data = aspeed_spi_init_data,
	.pinctrl_init = aspeed_spi_pinctrl_init,
	.pinctrl_post_init = NULL,
	.proprietary_config_init = NULL,
	.enable_4byte_mode = aspeed_spi_enable_4byte_mode,
	.cs_group_select = NULL,
	.safs_read_config = aspeed_spi_safs_read_config,
	.safs_write_config = aspeed_spi_safs_write_config,
#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	.dma_xfer_eligible = aspeed_spi_dma_xfer_eligible,
	.read_dma = aspeed_spi_read_dma,
	.write_dma = aspeed_spi_write_dma,
#endif
};

static const __maybe_unused struct aspeed_spi_ops ast1060_spi_ops = {
	.init_data = aspeed_spi_init_data,
	.pinctrl_init = aspeed_spi_pinctrl_init,
	.pinctrl_post_init = NULL,
	.proprietary_config_init = ast1060_spi_proprietary_config_init,
	.enable_4byte_mode = aspeed_spi_enable_4byte_mode,
	.cs_group_select = NULL,
	.safs_read_config = NULL,
	.safs_write_config = NULL,
#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	.dma_xfer_eligible = aspeed_spi_dma_xfer_eligible,
	.read_dma = aspeed_spi_read_dma,
	.write_dma = aspeed_spi_write_dma,
#endif
};

static const __maybe_unused struct aspeed_spi_ops ast2600_spi_ops = {
	.init_data = ast2600_spi_init_data,
	.pinctrl_init = aspeed_spi_pinctrl_init,
	.pinctrl_post_init = NULL,
	.proprietary_config_init = NULL,
	.enable_4byte_mode = aspeed_spi_enable_4byte_mode,
	.cs_group_select = NULL,
	.safs_read_config = aspeed_spi_safs_read_config,
	.safs_write_config = aspeed_spi_safs_write_config,
#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	.dma_xfer_eligible = aspeed_spi_dma_xfer_eligible,
	.read_dma = aspeed_spi_read_dma,
	.write_dma = aspeed_spi_write_dma,
#endif
};

static const __maybe_unused struct aspeed_spi_ops ast2700_spi_ops = {
	.init_data = ast2700_spi_init_data,
	.pinctrl_init = aspeed_spi_pinctrl_init,
	.pinctrl_post_init = NULL,
	.proprietary_config_init = NULL,
	.enable_4byte_mode = ast2700_spi_enable_4byte_mode,
	.cs_group_select = NULL,
	.safs_read_config = NULL,
	.safs_write_config = NULL,
#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	.dma_xfer_eligible = ast2700_spi_dma_xfer_eligible,
	.read_dma = ast2700_aspeed_spi_read_dma,
	.write_dma = ast2700_aspeed_spi_write_dma,
	.cache_flush = aspeed_spi_dma_cache_flush,
	.cache_invd = aspeed_spi_dma_cache_invd,
#endif
};

/*
 * Lite SPI describes the ASPEED SPI controller variant without interrupt
 * controller and pinctrl integration. Since it cannot signal completion via
 * interrupts or configure pins through pinctrl, the driver uses a reduced
 * feature set and falls back to polling where needed.
 */
static const __maybe_unused struct aspeed_spi_ops ast2700_spi_lite_ops = {
	.init_data = ast2700_spi_init_data,
	.pinctrl_init = ast2700_spi_lite_pinctrl_init,
	.pinctrl_post_init = ast2700_spi_lite_pinctrl_post_init,
	.proprietary_config_init = NULL,
	.enable_4byte_mode = ast2700_spi_enable_4byte_mode,
	.cs_group_select = NULL,
	.safs_read_config = NULL,
	.safs_write_config = NULL,
#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	.dma_xfer_eligible = ast2700_spi_dma_xfer_eligible,
	.read_dma = ast2700_aspeed_spi_read_dma,
	.write_dma = ast2700_aspeed_spi_write_dma,
#endif
};

/*
 * AST10X0_G2 full SPI controller ops (with interrupt and pinctrl support).
 * Reuses AST2700 data/DMA functions since AST10X0_G2 shares the same
 * SPI controller architecture.
 */
static const __maybe_unused struct aspeed_spi_ops ast10x0_g2_spi_ops = {
	.init_data = ast10x0_g2_spi_init_data,
	.pinctrl_init = aspeed_spi_pinctrl_init,
	.pinctrl_post_init = NULL,
	.proprietary_config_init = ast10x0_g2_spi_proprietary_config_init,
	.enable_4byte_mode = ast2700_spi_enable_4byte_mode,
	.cs_group_select = ast10x0_g2_spi_cs_group_select,
	.safs_read_config = NULL,
	.safs_write_config = NULL,
#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	.dma_xfer_eligible = ast10x0_g2_spi_dma_xfer_eligible,
	.read_dma = ast2700_aspeed_spi_read_dma,
	.write_dma = ast2700_aspeed_spi_write_dma,
	.cache_flush = aspeed_spi_dma_cache_flush,
	.cache_invd = aspeed_spi_dma_cache_invd,
#endif
};

/*
 * AST10X0_G2 lite SPI controller ops (MCU core variant, no interrupt/pinctrl).
 * Falls back to polling and uses lite pinctrl helpers.
 */
static const __maybe_unused struct aspeed_spi_ops ast10x0_g2_spi_lite_ops = {
	.init_data = ast10x0_g2_spi_init_data,
	.pinctrl_init = ast2700_spi_lite_pinctrl_init,
	.pinctrl_post_init = ast2700_spi_lite_pinctrl_post_init,
	.proprietary_config_init = ast10x0_g2_spi_proprietary_config_init,
	.enable_4byte_mode = ast2700_spi_enable_4byte_mode,
	.cs_group_select = ast10x0_g2_spi_cs_group_select,
	.safs_read_config = NULL,
	.safs_write_config = NULL,
#ifdef CONFIG_SPI_DMA_SUPPORT_ASPEED
	.dma_xfer_eligible = ast10x0_g2_spi_dma_xfer_eligible,
	.read_dma = ast2700_aspeed_spi_read_dma,
	.write_dma = ast2700_aspeed_spi_write_dma,
	.cache_flush = aspeed_spi_dma_cache_flush,
	.cache_invd = aspeed_spi_dma_cache_invd,
#endif
};

#if defined(CONFIG_SPI_DMA_SUPPORT_ASPEED)
#define ASPEED_SPI_IRQ_INIT(soc, n, isr)                                 \
	static void aspeed_spi_irq_config_func_##soc##_##n(               \
		const struct device *dev)                                  \
	{                                                                \
		ARG_UNUSED(dev);                                         \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),   \
			    isr, DEVICE_DT_INST_GET(n), 0);              \
		irq_enable(DT_INST_IRQN(n));                             \
	}
#define ASPEED_SPI_IRQ_CONFIG_INIT(soc, n) \
	.irq_config_func = aspeed_spi_irq_config_func_##soc##_##n,
#else
#define ASPEED_SPI_IRQ_INIT(soc, n, isr)
#define ASPEED_SPI_IRQ_CONFIG_INIT(soc, n) \
	.irq_config_func = NULL,
#endif

#define ASPEED_SPI_PINCTRL_DEFINE(n) \
	PINCTRL_DT_INST_DEFINE(n);
#define ASPEED_SPI_PINCTRL_CONFIG_INIT(n) \
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),

#define ASPEED_SPI_CONFIG_INIT(soc, n, soc_ops)                                \
	static struct aspeed_spi_config aspeed_spi_config_##soc##_##n = {   \
		.ctrl_base = DT_INST_REG_ADDR_BY_NAME(n, ctrl_reg),         \
		.spi_mmap_base = DT_INST_REG_ADDR_BY_NAME(n, spi_mmap),     \
		.max_cs = DT_INST_PROP(n, num_cs),                          \
		.ctrl_type = DT_ENUM_IDX(DT_DRV_INST(n), ctrl_type),        \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),         \
		.clk_id = (clock_control_subsys_t)                          \
			  DT_INST_CLOCKS_CELL(n, clk_id),                   \
		.ops = &(soc_ops),                                          \
		.mux_ctrl.master_idx =                                      \
			DT_INST_PROP_OR(n, internal_mux_master, 0),         \
		.mux_ctrl.spim_output_base =                                \
			DT_INST_PROP_OR(n, spi_monitor_output_base, 0),     \
		.mux_ctrl.spi_monitor_common_ctrl =                         \
			COND_CODE_1(DT_NODE_HAS_PROP(DT_DRV_INST(n),        \
						     spi_monitor_common_ctrl),  \
				    DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(   \
					    n, spi_monitor_common_ctrl, 0)), \
				    NULL),                                  \
		.aspeed_spim_proprietary_config_enable =                    \
			DT_PROP(DT_DRV_INST(n),                             \
				spim_proprietary_config_enable),            \
		.timing_calibration_disabled =                              \
			DT_PROP(DT_DRV_INST(n),                             \
				timing_calibration_disabled),                \
		.timing_calibration_start_off =                             \
			DT_INST_PROP_OR(n, timing_calibration_start_offset,  \
					0),                                  \
		.pure_spi_mode_only =                                       \
			DT_PROP(DT_DRV_INST(n), pure_spi_mode_only),        \
		.spi_ctrl_fifo_enabled =                                    \
			DT_PROP(DT_DRV_INST(n), spi_ctrl_fifo_enabled),     \
		.cs_group_analog_mux_enable =                               \
			DT_PROP(DT_DRV_INST(n), cs_group_analog_mux_enable),

#define ASPEED_SPI_DEFINE(soc, n)                                           \
	static struct aspeed_spi_data aspeed_spi_data_##soc##_##n = {       \
		SPI_CONTEXT_INIT_LOCK(aspeed_spi_data_##soc##_##n, ctx),    \
		SPI_CONTEXT_INIT_SYNC(aspeed_spi_data_##soc##_##n, ctx),    \
	};                                                               \
                                                                         \
	DEVICE_DT_INST_DEFINE(n, &aspeed_spi_init, NULL,                 \
			      &aspeed_spi_data_##soc##_##n,              \
			      &aspeed_spi_config_##soc##_##n, POST_KERNEL, \
			      75, &aspeed_spi_driver_api);

#define ASPEED_SPI_INIT(soc, n, ops, isr)                                  \
	ASPEED_SPI_PINCTRL_DEFINE(n)                                        \
	ASPEED_SPI_IRQ_INIT(soc, n, isr)                                  \
	ASPEED_SPI_CONFIG_INIT(soc, n, ops)                               \
		ASPEED_SPI_IRQ_CONFIG_INIT(soc, n)                       \
		ASPEED_SPI_PINCTRL_CONFIG_INIT(n)                         \
	};                                                               \
	ASPEED_SPI_DEFINE(soc, n)

#define ASPEED_SPI_LITE_INIT(soc, n, ops)                               \
	ASPEED_SPI_CONFIG_INIT(soc, n, ops)                               \
		.irq_config_func = NULL,                                  \
	};                                                               \
	ASPEED_SPI_DEFINE(soc, n)

/* For legacy dts support for AST1030 and AST1060 */
#define ASPEED_COMMON_SPI_INIT(n)                                         \
	ASPEED_SPI_INIT(common, n, aspeed_common_spi_ops, aspeed_spi_dma_isr)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_spi_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_COMMON_SPI_INIT)

/* AST1030 */
#define ASPEED_AST1030_SPI_INIT(n)                                         \
	ASPEED_SPI_INIT(ast1030, n, ast1030_spi_ops, aspeed_spi_dma_isr)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast1030_spi_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST1030_SPI_INIT)

/* AST1040/AST1080 */
#define ASPEED_AST10X0_G2_SPI_INIT(n)                                         \
	ASPEED_SPI_INIT(ast10x0_g2, n, ast10x0_g2_spi_ops, ast2700_aspeed_spi_dma_isr)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast10x0_g2_spi_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST10X0_G2_SPI_INIT)

/* AST1060 */
#define ASPEED_AST1060_SPI_INIT(n)                                         \
	ASPEED_SPI_INIT(ast1060, n, ast1060_spi_ops, aspeed_spi_dma_isr)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast1060_spi_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST1060_SPI_INIT)

/* AST2600 */
#define ASPEED_AST2600_SPI_INIT(n)                                         \
	ASPEED_SPI_INIT(ast2600, n, ast2600_spi_ops, aspeed_spi_dma_isr)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast2600_spi_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST2600_SPI_INIT)

/* AST2700 */
#define ASPEED_AST2700_SPI_INIT(n)                                         \
	ASPEED_SPI_INIT(ast2700, n, ast2700_spi_ops,                 \
			ast2700_aspeed_spi_dma_isr)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast2700_spi_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST2700_SPI_INIT)

/* AST2700 Boot MCU */
#define ASPEED_AST2700_SPI_LITE_INIT(n)                                 \
	ASPEED_SPI_LITE_INIT(ast2700_spi_lite, n, ast2700_spi_lite_ops)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast2700_spi_lite_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST2700_SPI_LITE_INIT)

/* AST1040/AST1080 RISC-V MCU */
#define ASPEED_AST10X0_G2_SPI_LITE_INIT(n)                                 \
	ASPEED_SPI_LITE_INIT(ast10x0_g2_spi_lite, n, ast10x0_g2_spi_lite_ops)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT aspeed_ast10x0_g2_spi_lite_controller
DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST10X0_G2_SPI_LITE_INIT)
