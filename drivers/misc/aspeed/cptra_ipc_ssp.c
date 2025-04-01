/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/cptra.h>
#include <zephyr/drivers/misc/aspeed/cptra_ipc.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/ecdsa_structs.h>
#include <zephyr/crypto/ecdsa.h>
#include <zephyr/crypto/lms.h>
#include <zephyr/crypto/lms_structs.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(cptra_ipc, CONFIG_MISC_ASPEED_LOG_LEVEL);

#define IPC_DEV_SSP_TX_BOOTMCU_RX		"ipc1@400"
#define IPC_DEV_BOOTMCU_TX_SSP_RX		"ipc1@500"
#define IPC_CHANNEL_ID_CPTRA			1

#define IPC_CHANNEL_1_SSP_OUT_ADDR		(0x1000000 + (3 * 1024 * 1024))

bool cptra_ipc_rx_bootmcu;
uint32_t cptra_ipc_rx_data[8];

int cptra_ipc_transfer(enum cptra_ipc_cmd cmd, void *input, int input_size,
		       enum cptra_ipc_rx_type type, void *output, int output_size)
{
	uint32_t data[2] = {IPC_CHANNEL_1_BOOTMCU_IN_ADDR, IPC_CHANNEL_1_BOOTMCU_OUT_ADDR};
	uint8_t *p8_ssp_in = (uint8_t *)IPC_CHANNEL_1_SSP_IN_ADDR;
	int rc;

	/* Copy input data into shared memory */
	memcpy(p8_ssp_in, &input, input_size);

	rc = cptra_ipc_trigger(cmd, data, sizeof(data));
	if (rc) {
		LOG_ERR("cptra_ipc_trigger failed\n");
		return rc;
	}

	rc = cptra_ipc_receive(type, output, output_size);
	if (rc) {
		LOG_ERR("cptra_ipc_receive failed\n");
		return rc;
	}

	return 0;
}

void cptra_ipc_rx_done(void)
{
	cptra_ipc_rx_bootmcu = true;
}

bool cptra_ipc_rx_is_done(void)
{
	return cptra_ipc_rx_bootmcu;
}

void cptra_ipc_rx_clear(void)
{
	memset(cptra_ipc_rx_data, 0, sizeof(cptra_ipc_rx_data));
	cptra_ipc_rx_bootmcu = false;
}

int cptra_ipc_receive(enum cptra_ipc_rx_type type, void *output, int output_size)
{
	uint8_t *p8 = (uint8_t *)IPC_CHANNEL_1_SSP_OUT_ADDR;
	int size;
	int rc = 0;

	while (cptra_ipc_rx_is_done() == false) {
		k_msleep(1);
	}

	if (type == CPTRA_IPC_RX_TYPE_INTERNAL) {
		if (output_size < 8)
			size = output_size;
		else
			size = 8;
		memcpy(output, cptra_ipc_rx_data, size);

	} else {
		memcpy(output, p8, output_size);
		rc = cptra_ipc_rx_data[0];
	}

	cptra_ipc_rx_clear();

	return rc;
}

int cptra_ipc_trigger(enum cptra_ipc_cmd cmd, void *input, int input_size)
{
	char ipc_name[32] = IPC_DEV_SSP_TX_BOOTMCU_RX;
	const struct device *ipmdev;
	uint32_t *p32 = (uint32_t *)input;
	uint32_t data[8] = {0};
	int rc;

	LOG_DBG("%s\n", __func__);

	ipmdev = device_get_binding(ipc_name);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = -1;
		return rc;
	}

	data[0] = (uint32_t)cmd;
	for (int i = 0; i < input_size / 4 && i < 8; i++) {
		data[i + 1] = p32[i];
	}

	rc = ipm_send(ipmdev, 0, IPC_CHANNEL_ID_CPTRA, (void *)data, sizeof(data));
	if (rc)
		LOG_ERR("ipm_send failed\n");
	else
		LOG_DBG("ipm_send success\n");

	return rc;
}

static void cptra_ipc_cb(const struct device *ipmdev, void *user_data,
			 uint32_t id, volatile void *msg_data)
{
	uint32_t *buf = (uint32_t *)msg_data;

	LOG_DBG("%s: msg ch1 with data at %p\n", __func__, (void *)msg_data);
	for (int i = 0; i < 8; i++) {
		cptra_ipc_rx_data[i] = buf[i];
		LOG_DBG("msg data = 0x%x\n", buf[i]);
	}

	cptra_ipc_rx_done();
}

int cptra_ipc_enable(void)
{
	char ipc_name[32] = IPC_DEV_BOOTMCU_TX_SSP_RX;
	int device_id = IPC_CHANNEL_ID_CPTRA;
	const struct device *ipmdev;
	int rc = 0;

	LOG_INF("%s", __func__);

	ipmdev = device_get_binding(ipc_name);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = -1;
		return rc;
	}

	ipm_register_id_callback(ipmdev, device_id, cptra_ipc_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, 1);
	if (rc) {
		printk("%s: cannot ipm_set_enabled\n", ipc_name);
		return rc;
	}

	return 0;
}
