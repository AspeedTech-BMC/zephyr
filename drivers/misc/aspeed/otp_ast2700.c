/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#define DT_DRV_COMPAT		aspeed_ast27xx_otp

#include <soc.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/aspeed/otp_ast27xx.h>

#include "otp_info_ast27xx.h"
#include <zephyr/drivers/misc/aspeed/otp.h>

LOG_MODULE_REGISTER(otp_ast2700, CONFIG_LOG_DEFAULT_LEVEL);

/***********************
 *                     *
 * OTP regs definition *
 *                     *
 ***********************/
#define OTP_REG_SIZE			0x200

#define OTP_PASSWD			0x349fe38a
#define OTP_CMD_READ			0x23b1e361
#define OTP_CMD_PROG			0x23b1e364
#define OTP_CMD_PROG_MULTI		0x23b1e365

#define OTP_CMD_OFFSET			0x20
#define OTP_MASTER			OTP_M1

#define OTP_KEY				0x0
#define OTP_CMD				(OTP_MASTER * OTP_CMD_OFFSET + 0x4)
#define OTP_WDATA_0			(OTP_MASTER * OTP_CMD_OFFSET + 0x8)
#define OTP_WDATA_1			(OTP_MASTER * OTP_CMD_OFFSET + 0xc)
#define OTP_WDATA_2			(OTP_MASTER * OTP_CMD_OFFSET + 0x10)
#define OTP_WDATA_3			(OTP_MASTER * OTP_CMD_OFFSET + 0x14)
#define OTP_STATUS			(OTP_MASTER * OTP_CMD_OFFSET + 0x18)
#define OTP_ADDR			(OTP_MASTER * OTP_CMD_OFFSET + 0x1c)
#define OTP_RDATA			(OTP_MASTER * OTP_CMD_OFFSET + 0x20)

#define OTP_ECC_EN			0x0D4

/* OTP status: [0] */
#define OTP_STS_IDLE			0x0
#define OTP_STS_BUSY			0x1

/* OTP cmd status: [7:4] */
#define OTP_GET_CMD_STS(x)		(((x) & 0xF0) >> 4)
#define OTP_STS_PASS			0x0
#define OTP_STS_FAIL			0x1
#define OTP_STS_CMP_FAIL		0x2
#define OTP_STS_REGION_FAIL		0x3
#define OTP_STS_MASTER_FAIL		0x4

/* OTP ECC EN */
#define ECC_ENABLE			0x1
#define ECC_DISABLE			0x0
#define ECCBRP_EN			BIT(0)

#define ROM_REGION_START_ADDR		0x0
#define ROM_REGION_END_ADDR		0x3e0
#define RBP_REGION_START_ADDR		ROM_REGION_END_ADDR
#define RBP_REGION_END_ADDR		0x400
#define CONF_REGION_START_ADDR		RBP_REGION_END_ADDR
#define CONF_REGION_END_ADDR		0x420
#define STRAP_REGION_START_ADDR		CONF_REGION_END_ADDR
#define STRAP_REGION_END_ADDR		0x430
#define STRAPEXT_REGION_START_ADDR	STRAP_REGION_END_ADDR
#define STRAPEXT_REGION_END_ADDR	0x440
#define USER_REGION_START_ADDR		STRAPEXT_REGION_END_ADDR
#define USER_REGION_END_ADDR		0x1000
#define SEC_REGION_START_ADDR		USER_REGION_END_ADDR
#define SEC_REGION_END_ADDR		0x1c00
#define CAL_REGION_START_ADDR		SEC_REGION_END_ADDR
#define CAL_REGION_END_ADDR		0x1f80
#define SW_PUF_REGION_START_ADDR	CAL_REGION_END_ADDR
#define SW_PUF_REGION_END_ADDR		0x1fc0
#define HW_PUF_REGION_START_ADDR	SW_PUF_REGION_END_ADDR
#define HW_PUF_REGION_END_ADDR		0x2000

#define OTP_TIMEOUT_US			10000

/* OTPSTRAP */
#define OTPSTRAP0_ADDR			STRAP_REGION_START_ADDR
#define OTPSTRAP14_ADDR			(OTPSTRAP0_ADDR + 0xe)

enum otp_error_code {
	OTP_SUCCESS,
	OTP_READ_FAIL,
	OTP_PROG_FAIL,
	OTP_CMP_FAIL,
};

enum aspeed_otp_master_id {
	OTP_M0 = 0,
	OTP_M1,
	OTP_M2,
	OTP_M3,
	OTP_M4,
	OTP_M5,
	OTP_MID_MAX,
};

struct otp_ast27xx_config {
	uintptr_t base;
	int gbl_ecc_en;
};

static void otp_unlock(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;

	sys_write32(OTP_PASSWD, cfg->base + OTP_KEY);
}

static int wait_complete(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	uint32_t val;

	do {
		val = sys_read32(cfg->base + OTP_STATUS);
		k_usleep(10);
	} while (val != 0x0);

	return 0;
}

static int otp_read_data(const struct device *dev, uint32_t offset, uint16_t *data)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int ret;

	sys_write32(cfg->gbl_ecc_en, cfg->base + OTP_ECC_EN);
	sys_write32(offset, cfg->base + OTP_ADDR);
	sys_write32(OTP_CMD_READ, cfg->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return OTP_READ_FAIL;

	data[0] = sys_read32(cfg->base + OTP_RDATA);

	return 0;
}

int otp_prog_data(const struct device *dev, uint32_t offset, uint16_t data)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int ret;

	sys_write32(cfg->gbl_ecc_en, cfg->base + OTP_ECC_EN);
	sys_write32(offset, cfg->base + OTP_ADDR);
	sys_write32(data, cfg->base + OTP_WDATA_0);
	sys_write32(OTP_CMD_PROG, cfg->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return OTP_PROG_FAIL;

	return 0;
}

int otp_prog_multi_data(const struct device *dev, uint32_t offset, uint32_t *data, int count)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int ret;

	sys_write32(cfg->gbl_ecc_en, cfg->base + OTP_ECC_EN);
	sys_write32(offset, cfg->base + OTP_ADDR);
	for (int i = 0; i < count; i++)
		sys_write32(data[i], cfg->base + OTP_WDATA_0 + 4 * i);

	sys_write32(OTP_CMD_PROG_MULTI, cfg->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return OTP_PROG_FAIL;

	return 0;
}

static int aspeed_otp_read(const struct device *dev, uint32_t offset, void *buf, int size)
{
	int ret;
	uint16_t *data = (uint16_t *)buf;

	for (int i = 0; i < size; i++) {
		ret = otp_read_data(dev, offset + i, data + i);
		if (ret) {
			LOG_ERR("%s: read failed\n", __func__);
			break;
		}
	}

	return ret;
}

static int aspeed_otp_write(const struct device *dev, uint32_t offset, void *buf, int size)
{
	uint32_t *data32 = (uint32_t *)buf;
	uint16_t *data = (uint16_t *)buf;
	int ret;

	if (size == 1)
		ret = otp_prog_data(dev, offset, data[0]);
	else
		ret = otp_prog_multi_data(dev, offset, data32, size / 2);

	if (ret)
		LOG_ERR("%s: prog failed\n", __func__);

	return ret;
}

static int aspeed_otp_ecc_init(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int ret;
	uint32_t val;

	/* Check cfg_ecc_en */
	sys_write32(0, cfg->base + OTP_ECC_EN);
	sys_write32(OTPSTRAP14_ADDR, cfg->base + OTP_ADDR);
	sys_write32(OTP_CMD_READ, cfg->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return OTP_READ_FAIL;

	val = sys_read32(cfg->base + OTP_RDATA);
	if (val & 0x1)
		cfg->gbl_ecc_en = 0x1;
	else
		cfg->gbl_ecc_en = 0x0;

	return 0;
}

static int otp_ast27xx_init(const struct device *dev)
{
	struct otp_ast27xx_config *cfg = (struct otp_ast27xx_config *)dev->config;
	int rc;

	otp_unlock(dev);

	/* OTP ECC init */
	rc = aspeed_otp_ecc_init(dev);
	if (rc) {
		LOG_ERR("OTP ECC init failed, rc:%d\n", rc);
		return rc;
	}

	LOG_INF("0x%x: Aspeed OTP driver Initialized\n", (uint32_t)cfg->base);

	return rc;
}

static struct otp_driver_api otp_funcs = {
	.otp_read_multi = aspeed_otp_read,
	.otp_program_multi = aspeed_otp_write,
};

struct otp_ast27xx_drv_state {
	bool in_use;
};

static const struct otp_ast27xx_config otp_ast27xx_config = {
	.base = DT_REG_ADDR(DT_DRV_INST(0)),
};

static struct otp_ast27xx_drv_state otp_ast27xx_state;

#define ASPEED_AST27XX_OTP_INIT(inst)					\
DEVICE_DT_INST_DEFINE(inst, otp_ast27xx_init, NULL,			\
		      &otp_ast27xx_state, &otp_ast27xx_config,		\
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
		      (void *)&otp_funcs);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_AST27XX_OTP_INIT)
