/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include "zephyr/sys/sys_io.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <soc.h>

#define DT_DRV_COMPAT aspeed_edaf_bridge

LOG_MODULE_REGISTER(edaf_aspeed);

#define UNKNOWN              0
#define EDAF_BDGE_OVER_ESPI0 1
#define EDAF_BDGE_OVER_ESPI1 2

#define SCU1_HOST_CONF_1       0xa00
#define SCU1_HOST_CONF_2       0xa20
#define SCU1_HOST_CONF_EDAF_EN BIT(10)
#define SCU1_HOST_CONF_LPC_EN  BIT(9)

#define ESPI_CHN3_CTL      0x400
#define ESPI_CHN3_SW_READY BIT(5)

#define ESPI_CHN3_FILTER_CTL 0x500
#define ESPI_CHN3_FILTER_SAFS_SIZE GENMASK(24, 16)

#define EDAF_BDGE_CFG           0x0
#define EDAF_BDGE_CFG_CMD_EN    BIT(0)
#define EDAF_BDGE_CBASE         0x20
#define EDAF_BDGE_MBASE         0x24
#define EDAF_BDGE_CMD_READ      0x40
#define EDAF_BDGE_CMD_WRITE_EN  0x44
#define EDAF_BDGE_CMD_WRITE     0x48
#define EDAF_BDGE_CMD_READ_STS  0x4c
#define EDAF_BDGE_CMD_ERASE_4K  0x50
#define EDAF_BDGE_CMD_ERASE_32K 0x54
#define EDAF_BDGE_CMD_ERASE_64K 0x58
#define EDAF_BDGE_MISC          0x70
#define EDAF_BDGE_MISC_MBASE_H  GENMASK(15, 8)
#define EDAF_BDGE_MISC_CBASE_H  GENMASK(7, 0)

#define EDAF_BDGE_GLOBAL_CFG        0x0
#define EDAF_BDGE_GLOBAL_CFG0_ERASE BIT(1)
#define EDAF_BDGE_GLOBAL_CFG1_ERASE BIT(9)

struct aspeed_edaf_bridge_config {
	mm_reg_t base; /* eDAF bridge registers base */

	mm_reg_t scu1_base; /* SCU1 base */
	bool have_scu1;

	mm_reg_t edaf_gcfg_base; /* Global eDAF bridge cfg base (optional) */
	bool have_edaf_gcfg;

	bool edaf_ddr_mode;
	bool edaf_erase_with_1;

	uint64_t mem_base; /* eDAF memory base (physical) */
	bool mem_base_valid;

	uint64_t ctl_base; /* SPI controller base (for CBASE) */
	bool ctl_base_valid;

	uint64_t espi_base; /* eSPI base for CHN3_CTL */
	bool espi_base_valid;

	uint32_t cmd_read;
	uint32_t cmd_write_en;
	uint32_t cmd_write;
	uint32_t cmd_read_sts;
	uint32_t cmd_erase_4k;
	uint32_t cmd_erase_32k;
	uint32_t cmd_erase_64k;

	uint8_t plat; /* EDAF_BDGE_OVER_ESPI0 / 1 */
};

static int aspeed_edaf_bridge_init(const struct device *dev)
{
	const struct aspeed_edaf_bridge_config *cfg = dev->config;
	mm_reg_t edaf_regs = cfg->base;
	uint32_t val;
	uint64_t mbase = cfg->mem_base;

	/* SCU1 checks + enable EDAF on desired host interface */
	if (!cfg->have_scu1) {
		LOG_ERR("SCU1 base not provided");
		return -ENODEV;
	}

	if (cfg->plat == EDAF_BDGE_OVER_ESPI0) {
		val = sys_read32(cfg->scu1_base + SCU1_HOST_CONF_1);
		if (val & SCU1_HOST_CONF_LPC_EN) {
			LOG_ERR("Host Configuration1: eSPI0 eDAF is not valid");
			return -EINVAL;
		}
		val |= SCU1_HOST_CONF_EDAF_EN;
		sys_write32(val, cfg->scu1_base + SCU1_HOST_CONF_1);
	} else if (cfg->plat == EDAF_BDGE_OVER_ESPI1) {
		val = sys_read32(cfg->scu1_base + SCU1_HOST_CONF_2);
		if (val & SCU1_HOST_CONF_LPC_EN) {
			LOG_ERR("Host Configuration2: eSPI1 eDAF is not valid");
			return -EINVAL;
		}
		val |= SCU1_HOST_CONF_EDAF_EN;
		sys_write32(val, cfg->scu1_base + SCU1_HOST_CONF_2);
	} else {
		LOG_ERR("Unknown platform for eDAF bridge");
		return -EINVAL;
	}

	/* Configure command enable depending on DDR mode */
	val = sys_read32(edaf_regs + EDAF_BDGE_CFG);
	if (cfg->edaf_ddr_mode) {
		val &= ~EDAF_BDGE_CFG_CMD_EN;
	} else {
		val |= EDAF_BDGE_CFG_CMD_EN;
	}
	sys_write32(val, edaf_regs + EDAF_BDGE_CFG);

	if (cfg->edaf_ddr_mode) {
		/* DDR mode: mem_base was derived from reserved-memory */
		if (!cfg->mem_base_valid) {
			LOG_ERR("DDR mode but mem_base invalid");
			return -ENODEV;
		}
		mbase = ast27xx_soc_virt_addr_to_phy_addr((uintptr_t)mbase);
		/* Optional: set erase-with-1 behavior via global cfg */
		if (cfg->edaf_erase_with_1) {
			if (!cfg->have_edaf_gcfg) {
				LOG_ERR("erase-with-1 requested but no global cfg base");
				return -ENODEV;
			}

			uint32_t gcfg = sys_read32(cfg->edaf_gcfg_base + EDAF_BDGE_GLOBAL_CFG);

			if (cfg->plat == EDAF_BDGE_OVER_ESPI0) {
				gcfg |= EDAF_BDGE_GLOBAL_CFG0_ERASE;
			} else if (cfg->plat == EDAF_BDGE_OVER_ESPI1) {
				gcfg |= EDAF_BDGE_GLOBAL_CFG1_ERASE;
			} else {
				LOG_ERR("Unknown platform for eDAF bridge (GCFG)");
				return -EINVAL;
			}

			sys_write32(gcfg, cfg->edaf_gcfg_base + EDAF_BDGE_GLOBAL_CFG);
		}
	} else {
		/* SPI mode: program CBASE, command opcodes, etc. */
		if (cfg->ctl_base_valid) {
			uint64_t cbase = cfg->ctl_base;

			sys_write32((uint32_t)cbase, edaf_regs + EDAF_BDGE_CBASE);

			val = sys_read32(edaf_regs + EDAF_BDGE_MISC);
			val &= ~EDAF_BDGE_MISC_CBASE_H;
			val |= FIELD_PREP(EDAF_BDGE_MISC_CBASE_H, cbase >> 32);
			sys_write32(val, edaf_regs + EDAF_BDGE_MISC);
		}

		if (cfg->cmd_read) {
			sys_write32(cfg->cmd_read, edaf_regs + EDAF_BDGE_CMD_READ);
		}
		if (cfg->cmd_write_en) {
			sys_write32(cfg->cmd_write_en, edaf_regs + EDAF_BDGE_CMD_WRITE_EN);
		}
		if (cfg->cmd_write) {
			sys_write32(cfg->cmd_write, edaf_regs + EDAF_BDGE_CMD_WRITE);
		}
		if (cfg->cmd_read_sts) {
			sys_write32(cfg->cmd_read_sts, edaf_regs + EDAF_BDGE_CMD_READ_STS);
		}
		if (cfg->cmd_erase_4k) {
			sys_write32(cfg->cmd_erase_4k, edaf_regs + EDAF_BDGE_CMD_ERASE_4K);
		}
		if (cfg->cmd_erase_32k) {
			sys_write32(cfg->cmd_erase_32k, edaf_regs + EDAF_BDGE_CMD_ERASE_32K);
		}
		if (cfg->cmd_erase_64k) {
			sys_write32(cfg->cmd_erase_64k, edaf_regs + EDAF_BDGE_CMD_ERASE_64K);
		}
	}

	/* Program MBASE (common to both paths) */
	if (!cfg->mem_base_valid) {
		LOG_ERR("mem_base not configured");
		return -ENODEV;
	}
	sys_write32((uint32_t)mbase, edaf_regs + EDAF_BDGE_MBASE);

	val = sys_read32(edaf_regs + EDAF_BDGE_MISC);
	val &= ~EDAF_BDGE_MISC_MBASE_H;
	val |= FIELD_PREP(EDAF_BDGE_MISC_MBASE_H, mbase >> 32);
	sys_write32(val, edaf_regs + EDAF_BDGE_MISC);

	/* Mark eSPI channel 3 as ready if espi-base provided */
	if (cfg->espi_base_valid) {
		mm_reg_t espi_regs = (mm_reg_t)ast27xx_soc_phy_addr_to_virt_addr(cfg->espi_base);
		uint32_t espi = sys_read32(espi_regs + ESPI_CHN3_FILTER_CTL);

		espi &= ~ESPI_CHN3_FILTER_SAFS_SIZE;
		sys_write32(espi, espi_regs + ESPI_CHN3_FILTER_CTL);

		espi = sys_read32(espi_regs + ESPI_CHN3_CTL);
		espi |= ESPI_CHN3_SW_READY;
		sys_write32(espi, espi_regs + ESPI_CHN3_CTL);
	}
	return 0;
}

#define EDAF_CFG_SCU1_BASE(inst) DT_REG_ADDR(DT_PHANDLE(DT_DRV_INST(inst), aspeed_scu))

#define EDAF_CFG_HAVE_SCU1(inst) DT_NODE_HAS_PROP(DT_DRV_INST(inst), aspeed_scu)

#define EDAF_CFG_GCFG_BASE(inst) DT_REG_ADDR(DT_PHANDLE(DT_DRV_INST(inst), aspeed_edaf_bridge_cfg))

#define EDAF_CFG_HAVE_GCFG(inst) DT_NODE_HAS_PROP(DT_DRV_INST(inst), aspeed_edaf_bridge_cfg)

/* mem-base is an array [hi, lo] -> make uint64_t */
#define EDAF_CFG_MEM_BASE(inst)                                                                    \
	(((uint64_t)DT_PROP_BY_IDX(DT_DRV_INST(inst), mem_base, 0) << 32) |                        \
	 (uint64_t)DT_PROP_BY_IDX(DT_DRV_INST(inst), mem_base, 1))

#define EDAF_CFG_HAVE_MEM_BASE(inst) DT_NODE_HAS_PROP(DT_DRV_INST(inst), mem_base)

/* ctl-base and espi-base are direct 64-bit properties */
#define EDAF_CFG_CTL_BASE(inst) DT_PROP_OR(DT_DRV_INST(inst), ctl_base, false)

#define EDAF_CFG_HAVE_CTL_BASE(inst) DT_NODE_HAS_PROP(DT_DRV_INST(inst), ctl_base)

#define EDAF_CFG_ESPI_BASE(inst) DT_PROP(DT_DRV_INST(inst), espi_base)

#define EDAF_CFG_HAVE_ESPI_BASE(inst) DT_NODE_HAS_PROP(DT_DRV_INST(inst), espi_base)

/* Instance → platform mapping (0→ESPI0, 1→ESPI1) */
#define EDAF_CFG_PLAT(inst) ((inst) == 0 ? EDAF_BDGE_OVER_ESPI0 : EDAF_BDGE_OVER_ESPI1)

#define EDAF_CFG_CMD_PROP(inst, prop, default_val)                                                 \
	DT_PROP_OR(DT_DRV_INST(inst), prop, (default_val))

#define EDAF_CFG_DDR_MODE(inst) DT_PROP_OR(DT_DRV_INST(inst), edaf_ddr_mode, false)

#define EDAF_CFG_ERASE_WITH_1(inst) DT_PROP_OR(DT_DRV_INST(inst), edaf_erase_with_1, false)

/* Build config for each instance */
#define EDAF_INST_CONFIG(inst)                                                                     \
	static const struct aspeed_edaf_bridge_config aspeed_edaf_bridge_config_##inst = {         \
		.base = DT_REG_ADDR(DT_DRV_INST(inst)),                                            \
                                                                                                   \
		.scu1_base = EDAF_CFG_HAVE_SCU1(inst) ? EDAF_CFG_SCU1_BASE(inst) : 0,              \
		.have_scu1 = EDAF_CFG_HAVE_SCU1(inst),                                             \
                                                                                                   \
		.edaf_gcfg_base = EDAF_CFG_HAVE_GCFG(inst) ? EDAF_CFG_GCFG_BASE(inst) : 0,         \
		.have_edaf_gcfg = EDAF_CFG_HAVE_GCFG(inst),                                        \
                                                                                                   \
		.edaf_ddr_mode = EDAF_CFG_DDR_MODE(inst),                                          \
		.edaf_erase_with_1 = EDAF_CFG_ERASE_WITH_1(inst),                                  \
                                                                                                   \
		.mem_base = EDAF_CFG_HAVE_MEM_BASE(inst) ? EDAF_CFG_MEM_BASE(inst) : 0,            \
		.mem_base_valid = EDAF_CFG_HAVE_MEM_BASE(inst),                                    \
                                                                                                   \
		.ctl_base = EDAF_CFG_HAVE_CTL_BASE(inst) ? EDAF_CFG_CTL_BASE(inst) : 0,            \
		.ctl_base_valid = EDAF_CFG_HAVE_CTL_BASE(inst),                                    \
                                                                                                   \
		.espi_base = EDAF_CFG_HAVE_ESPI_BASE(inst) ? EDAF_CFG_ESPI_BASE(inst) : 0,         \
		.espi_base_valid = EDAF_CFG_HAVE_ESPI_BASE(inst),                                  \
                                                                                                   \
		.cmd_read = EDAF_CFG_CMD_PROP(inst, cmd_read, 0),                                  \
		.cmd_write_en = EDAF_CFG_CMD_PROP(inst, cmd_write_enable, 0),                      \
		.cmd_write = EDAF_CFG_CMD_PROP(inst, cmd_write, 0),                                \
		.cmd_read_sts = EDAF_CFG_CMD_PROP(inst, cmd_read_status, 0),                       \
		.cmd_erase_4k = EDAF_CFG_CMD_PROP(inst, cmd_erase_4k, 0),                          \
		.cmd_erase_32k = EDAF_CFG_CMD_PROP(inst, cmd_erase_32k, 0),                        \
		.cmd_erase_64k = EDAF_CFG_CMD_PROP(inst, cmd_erase_64k, 0),                        \
                                                                                                   \
		.plat = EDAF_CFG_PLAT(inst),                                                       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, aspeed_edaf_bridge_init, NULL, NULL,                           \
			      &aspeed_edaf_bridge_config_##inst, POST_KERNEL,                      \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL)
/* Generate instances */
DT_INST_FOREACH_STATUS_OKAY(EDAF_INST_CONFIG);
