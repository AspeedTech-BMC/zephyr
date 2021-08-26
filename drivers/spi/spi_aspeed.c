/*
 * Copyright (c) 2021 Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_spi_controller

#include <device.h>
#include <drivers/spi.h>
#include <drivers/spi_nor.h>
#include <drivers/flash.h>
#include <errno.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_aspeed, CONFIG_SPI_LOG_LEVEL);
#include <sys/sys_io.h>
#include <sys/__assert.h>
#include "spi_context.h"


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

#define SPI60_WDT1                  (0x0060)
#define SPI64_WDT2                  (0x0064)
#define SPI68_WDT2_RELOAD_VAL       (0x0068)
#define SPI6C_WDT2_RESTART          (0x006C)

#define SPI7C_DMA_BUF_LEN           (0x007C)
#define SPI80_DMA_CTRL              (0x0080)
#define SPI84_DMA_FLASH_ADDR        (0x0084)
#define SPI88_DMA_RAM_ADDR          (0x0088)
#define SPI8C_DMA_LEN               (0x008C)
#define SPI90_CHECKSUM              (0x0090)

#define SPI94_CE0_TIMING_CTRL       (0x0094)
#define SPI98_CE1_TIMING_CTRL       (0x0098)
#define SPI9C_CE2_TIMING_CTRL       (0x009C)

#define SPIA0_CMD_FILTER_CTRL       (0x00A0)
#define SPIA4_ADDR_FILTER_CTRL      (0x00A4)
#define SPIA8_REG_LOCK_SRST         (0x00A8)
#define SPIAC_REG_LOCK_WDT          (0x00AC)
/* used for HW SAFS setting */
#define SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4	(0x006c)
#define SPIR74_HOST_DIRECT_ACCESS_CMD_CTRL2	(0x0074)

#define ASPEED_MAX_CS               3

#define ASPEED_SPI_NORMAL_READ      0x1
#define ASPEED_SPI_NORMAL_WRITE     0x2
#define ASPEED_SPI_USER             0x3
#define ASPEED_SPI_USER_INACTIVE    BIT(2)


#define ASPEED_SPI_SZ_2M            0x200000
#define ASPEED_SPI_SZ_256M          0x10000000

#define ASPEED_SPI_CTRL_VAL(io_mode, opcode, dummy_cycle) \
		(io_mode | ((opcode & 0xff) << 16) | dummy_cycle)

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

struct aspeed_spi_data {
	struct spi_context ctx;
	uint32_t (*segment_start)(uint32_t val);
	uint32_t (*segment_end)(uint32_t val);
	uint32_t (*segment_value)(uint32_t start, uint32_t end);
	struct aspeed_spi_decoded_addr decode_addr[ASPEED_MAX_CS];
	struct aspeed_cmd_mode cmd_mode[ASPEED_MAX_CS];
};

struct aspeed_spi_config {
	mm_reg_t ctrl_base;
	mm_reg_t spi_mmap_base;
	uint32_t max_cs;
	uint32_t platform;
	enum aspeed_ctrl_type ctrl_type;
};

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

static uint32_t aspeed_spi_cal_dummy_cycle(
		uint32_t bus_width, uint32_t dummy_cycle)
{
	uint32_t dummy_byte = 0;

	dummy_byte = dummy_cycle / (8 / bus_width);

	return (((dummy_byte & 0x3) << 6) | (((dummy_byte & 0x4) >> 2) << 14));
}

static void aspeed_spi_read_data(uint32_t ahb_addr,
		uint8_t *read_arr, uint32_t read_cnt)
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
		const uint8_t *write_arr, uint32_t write_cnt)
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

	if (!spi_context_tx_buf_on(ctx) && !spi_context_rx_buf_on(ctx)) {
		spi_context_complete(ctx, 0);
		return;
	}

	/* active cs */
	sys_write32(0x7, config->ctrl_base + SPI10_CE0_CTRL + cs * 4);
	sys_write32(0x3, config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	while (ctx->tx_buf && ctx->tx_len > 0) {
		aspeed_spi_write_data(data->decode_addr[cs].start,
			ctx->tx_buf, ctx->tx_len);
		spi_context_update_tx(ctx, 1, ctx->tx_len);
	};

	k_busy_wait(2);

	while (ctx->rx_buf && ctx->rx_len > 0) {
		aspeed_spi_read_data(data->decode_addr[cs].start,
			ctx->rx_buf, ctx->rx_len);
		spi_context_update_rx(ctx, 1, ctx->rx_len);
	};

	sys_write32(0x7, config->ctrl_base + SPI10_CE0_CTRL + cs * 4);
	spi_context_complete(ctx, 0);
}

static void aspeed_spi_nor_transceive_user(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t cs = ctx->config->slave;
	uint8_t dummy[12] = {0};

	sys_write32(ASPEED_SPI_USER_INACTIVE | ASPEED_SPI_USER,
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);
	sys_write32(ASPEED_SPI_USER, config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	/* cmd */
	sys_write32(ASPEED_SPI_USER |
		aspeed_spi_io_mode_user(JESD216_GET_CMD_BUSWIDTH(op_info.mode)),
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	aspeed_spi_write_data(data->decode_addr[cs].start, &op_info.opcode, 1);

	/* addr */
	sys_write32(ASPEED_SPI_USER |
		aspeed_spi_io_mode_user(JESD216_GET_ADDR_BUSWIDTH(op_info.mode)),
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	if (op_info.addr_len == 3)
		op_info.addr <<= 8;

	op_info.addr = sys_cpu_to_be32(op_info.addr);

	aspeed_spi_write_data(data->decode_addr[cs].start,
		(const uint8_t *)&op_info.addr, op_info.addr_len);

	/* dummy */
	aspeed_spi_write_data(data->decode_addr[cs].start,
		(const uint8_t *)&dummy,
		(op_info.dummy_cycle / (8 / JESD216_GET_ADDR_BUSWIDTH(op_info.mode))));

	/* data */
	sys_write32(ASPEED_SPI_USER |
			aspeed_spi_io_mode_user(JESD216_GET_DATA_BUSWIDTH(op_info.mode)),
			config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	if (op_info.data_direct == SPI_NOR_DATA_DIRECT_IN) {
		/* read data */
		aspeed_spi_read_data(data->decode_addr[cs].start,
				op_info.buf, op_info.data_len);
	} else {
		/* write data */
		aspeed_spi_write_data(data->decode_addr[cs].start,
				op_info.buf, op_info.data_len);
	}

	sys_write32(ASPEED_SPI_USER_INACTIVE | ASPEED_SPI_USER,
		config->ctrl_base + SPI10_CE0_CTRL + cs * 4);

	spi_context_complete(ctx, 0);
}

static int aspeed_spi_transceive(const struct device *dev,
					    const struct spi_config *spi_cfg,
					    const struct spi_buf_set *tx_bufs,
					    const struct spi_buf_set *rx_bufs)
{
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	spi_context_lock(ctx, false, NULL, spi_cfg);

	if (!spi_context_configured(ctx, spi_cfg))
		ctx->config = spi_cfg;

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	aspeed_spi_start_tx(dev);

	ret = spi_context_wait_for_completion(ctx);

	spi_context_release(ctx, ret);

	return ret;
}

static int aspeed_spi_nor_transceive(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	struct aspeed_spi_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	spi_context_lock(ctx, false, NULL, spi_cfg);
	if (!spi_context_configured(ctx, spi_cfg))
		ctx->config = spi_cfg;

	aspeed_spi_nor_transceive_user(dev, spi_cfg, op_info);

	ret = spi_context_wait_for_completion(ctx);
	spi_context_release(ctx, ret);

	return ret;
}

static int aspeed_spi_decode_range_reinit(const struct device *dev,
						const struct spi_config *spi_cfg,
						uint32_t flash_sz)
{
	uint32_t cs, tmp;
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	uint32_t decode_sz_arr[ASPEED_MAX_CS] = {0};
	uint32_t total_decode_range = 0;
	uint32_t start_addr, end_addr, pre_end_addr = 0;

	/* record original decode range */
	for (cs = 0; cs < config->max_cs; cs++) {
		tmp = sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4);
		if (tmp == 0)
			decode_sz_arr[cs] = 0;
		else
			decode_sz_arr[cs] = data->segment_end(tmp) - data->segment_start(tmp) + 1;
		LOG_DBG("ori decode range 0x%08x", decode_sz_arr[cs]);
		total_decode_range += decode_sz_arr[cs];
	}

	/* prepare new decode sz array */
	if (total_decode_range - decode_sz_arr[spi_cfg->slave]
		+ flash_sz < ASPEED_SPI_SZ_256M) {
		decode_sz_arr[spi_cfg->slave] = flash_sz;
	} else {
		/* do nothing, otherwise, decode range will be larger than 256MB */
		LOG_WRN("decode size out of range 0x%08x", flash_sz);
		return 0;
	}

	/* modify decode range */
	for (cs = 0; cs < config->max_cs; cs++) {
		if (decode_sz_arr[cs] == 0)
			continue;

		if (cs == 0)
			start_addr = config->spi_mmap_base;
		else
			start_addr = pre_end_addr;

		end_addr = start_addr + decode_sz_arr[cs] - 1;
		LOG_DBG("start: 0x%x end: 0x%x (0x%x)", start_addr, end_addr, decode_sz_arr[cs]);

		sys_write32(data->segment_value(start_addr, end_addr),
			config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4);
		data->decode_addr[cs].start = start_addr;
		if (cs == spi_cfg->slave)
			data->decode_addr[cs].len = flash_sz;

		pre_end_addr = end_addr + 1;
	}

	LOG_DBG("decode reg: <0x%08x, 0x%08x, 0x%08x>",
			sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 0),
			sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4),
			sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 8));

	return 0;
}

static int aspeed_spi_nor_read_init(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	int ret = 0;
	struct aspeed_spi_data *data = dev->data;
	const struct aspeed_spi_config *config = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint32_t reg_val;

	spi_context_lock(ctx, false, NULL, spi_cfg);

	LOG_DBG("[%s] mode %08x, cmd: %x, dummy: %d, frequency: %d",
		__func__, op_info.mode, op_info.opcode, op_info.dummy_cycle,
		spi_cfg->frequency);

	ret = aspeed_spi_decode_range_reinit(dev, spi_cfg, op_info.data_len);
	if (ret != 0)
		goto end;

	data->cmd_mode[spi_cfg->slave].normal_read =
		ASPEED_SPI_CTRL_VAL(aspeed_spi_io_mode(op_info.mode),
			op_info.opcode,
			aspeed_spi_cal_dummy_cycle(JESD216_GET_ADDR_BUSWIDTH(op_info.mode),
				op_info.dummy_cycle)) | ASPEED_SPI_NORMAL_READ;
	sys_write32(data->cmd_mode[spi_cfg->slave].normal_read,
			config->ctrl_base + SPI10_CE0_CTRL + spi_cfg->slave * 4);

	/* set controller to 4-byte mode */
	if (op_info.addr_len == 4) {
		sys_write32(sys_read32(config->ctrl_base + SPI04_CE_CTRL) |
			(0x11 << spi_cfg->slave), config->ctrl_base + SPI04_CE_CTRL);
	}

	/* config for SAFS */
	if (config->ctrl_type == HOST_SPI) {
		reg_val = sys_read32(config->ctrl_base + SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4);
		if (op_info.addr_len == 4)
			reg_val = (reg_val & 0xffff00ff) | (op_info.opcode << 8);
		else
			reg_val = (reg_val & 0xffffff00) | op_info.opcode;

		reg_val = (reg_val & 0x0fffffff) | aspeed_spi_io_mode(op_info.mode);
		sys_write32(reg_val, config->ctrl_base + SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4);
	}

end:

	spi_context_release(ctx, ret);

	return ret;
}

static int aspeed_spi_nor_write_init(const struct device *dev,
						const struct spi_config *spi_cfg,
						struct spi_nor_op_info op_info)
{
	int ret = 0;
	struct aspeed_spi_data *data = dev->data;
	const struct aspeed_spi_config *config = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint32_t reg_val;

	spi_context_lock(ctx, false, NULL, spi_cfg);

	if (config->ctrl_type == HOST_SPI) {
		reg_val = sys_read32(config->ctrl_base + SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4);
		reg_val = (reg_val & 0xf0ffffff) | (aspeed_spi_io_mode(op_info.mode) >> 8);
		sys_write32(reg_val, config->ctrl_base + SPIR6C_HOST_DIRECT_ACCESS_CMD_CTRL4);
		reg_val = sys_read32(config->ctrl_base + SPIR74_HOST_DIRECT_ACCESS_CMD_CTRL2);
		if (op_info.addr_len == 4)
			reg_val = (reg_val & 0xffff00ff) | (op_info.opcode << 8);
		else
			reg_val = (reg_val & 0xffffff00) | op_info.opcode;

		sys_write32(reg_val, config->ctrl_base + SPIR74_HOST_DIRECT_ACCESS_CMD_CTRL2);
	}

	spi_context_release(ctx, ret);

	return ret;

}

static int aspeed_spi_release(const struct device *dev,
				const struct spi_config *spi_cfg)
{
	struct aspeed_spi_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

void aspeed_segment_function_init(
		const struct aspeed_spi_config *config,
		struct aspeed_spi_data *data)

{
	switch (config->platform) {
	case 2600:
	case 2620:
	case 2605:
	case 2625:
		data->segment_start = ast2600_segment_addr_start;
		data->segment_end = ast2600_segment_addr_end;
		data->segment_value = ast2600_segment_addr_val;
		break;

	case 1030:
	case 1060:
		if (config->ctrl_type == BOOT_SPI) {
			data->segment_start = ast1030_fmc_segment_addr_start;
			data->segment_end = ast1030_fmc_segment_addr_end;
			data->segment_value = ast1030_fmc_segment_addr_val;
		} else {
			data->segment_start = ast1030_spi_segment_addr_start;
			data->segment_end = ast1030_spi_segment_addr_end;
			data->segment_value = ast1030_spi_segment_addr_val;
		}

		break;

	default:
		LOG_ERR("undefine ast platform %d\n", config->platform);
		__ASSERT_NO_MSG(0);

	}
}

void aspeed_decode_range_pre_init(
		const struct aspeed_spi_config *config,
		struct aspeed_spi_data *data)
{
	uint32_t cs;
	uint32_t unit_sz = ASPEED_SPI_SZ_2M; /* init 2M for each cs */
	uint32_t start_addr, end_addr, pre_end_addr = 0;

	for (cs = 0; cs < config->max_cs; cs++) {
		if (cs == 0)
			start_addr = config->spi_mmap_base;
		else
			start_addr = pre_end_addr;

		end_addr = start_addr + unit_sz - 1;

		/* the maximum decode size is 256MB */
		if (config->spi_mmap_base + ASPEED_SPI_SZ_256M <= end_addr) {
			LOG_DBG("%s %d smc decode address overflow\n", __func__, __LINE__);
			sys_write32(0, config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4);
			continue;
		}

		LOG_DBG("cs: %d start: 0x%08x, end: 0x%08x (%08x)\n",
			cs, start_addr, end_addr, data->segment_value(start_addr, end_addr));

		sys_write32(data->segment_value(start_addr, end_addr),
			config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4);
		LOG_DBG("cs: %d 0x%08x\n", cs,
			sys_read32(config->ctrl_base + SPI30_CE0_ADDR_DEC + cs * 4));

		data->decode_addr[cs].start = start_addr;
		data->decode_addr[cs].len = ASPEED_SPI_SZ_2M;
		pre_end_addr = end_addr + 1;
	}
}

static int aspeed_spi_init(const struct device *dev)
{
	const struct aspeed_spi_config *config = dev->config;
	struct aspeed_spi_data *data = dev->data;
	uint32_t cs;
	uint32_t reg_val;

	for (cs = 0; cs < config->max_cs; cs++) {
		reg_val = sys_read32(config->ctrl_base + SPI00_CE_TYPE_SETTING);
		sys_write32(reg_val | BIT(16 + cs), config->ctrl_base + SPI00_CE_TYPE_SETTING);

		data->cmd_mode[cs].user = ASPEED_SPI_USER;
	}

	aspeed_segment_function_init(config, data);
	aspeed_decode_range_pre_init(config, data);


	spi_context_unlock_unconditionally(&data->ctx);

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

#define ASPEED_SPI_INIT(n)						\
	static struct aspeed_spi_config aspeed_spi_config_##n = { \
		.ctrl_base = DT_INST_REG_ADDR_BY_NAME(n, ctrl_reg),		\
		.spi_mmap_base = DT_INST_REG_ADDR_BY_NAME(n, spi_mmap),	\
		.max_cs = DT_INST_PROP(n, num_cs),	\
		.platform = DT_INST_PROP(n, ast_platform), \
		.ctrl_type = DT_ENUM_IDX(DT_INST(n, DT_DRV_COMPAT), ctrl_type),	\
	};								\
									\
	static struct aspeed_spi_data aspeed_spi_data_##n = {	\
		SPI_CONTEXT_INIT_LOCK(aspeed_spi_data_##n, ctx),	\
		SPI_CONTEXT_INIT_SYNC(aspeed_spi_data_##n, ctx),	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, &aspeed_spi_init,			\
			    NULL,					\
			    &aspeed_spi_data_##n,			\
			    &aspeed_spi_config_##n, POST_KERNEL,	\
			    79,		\
			    &aspeed_spi_driver_api);			\

DT_INST_FOREACH_STATUS_OKAY(ASPEED_SPI_INIT)
