/*
 * Copyright (c) 2017 comsuisse AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr.h>
#include <ztest.h>
#include <drivers/i2s.h>
#include "i2s_api_test.h"

#define NUM_RX_BLOCKS 4
#define NUM_TX_BLOCKS 4
K_MEM_SLAB_DEFINE(rx_0_mem_slab, BLOCK_SIZE, NUM_RX_BLOCKS, 32);
K_MEM_SLAB_DEFINE(tx_0_mem_slab, BLOCK_SIZE, NUM_TX_BLOCKS, 32);

static int tx_block_write(const struct device *dev_i2s, int att, int err)
{
	return tx_block_write_slab(dev_i2s, att, err, &tx_0_mem_slab);
}
static int rx_block_read(const struct device *dev_i2s, int att)
{
	return rx_block_read_slab(dev_i2s, att, &rx_0_mem_slab);
}

/** Configure I2S TX transfer. */
void test_i2s_tx_transfer_configure_0(void)
{
	const struct device *dev_i2s;
	struct i2s_config i2s_cfg;
	int ret;

	dev_i2s = device_get_binding(I2S_DEV_NAME_TX);
	zassert_not_null(dev_i2s, "device " I2S_DEV_NAME_TX " not found");

	/* Configure */

	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	/* Configure the Transmit port as Master */
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.frame_clk_freq = FRAME_CLK_FREQ;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.mem_slab = &tx_0_mem_slab;
	i2s_cfg.timeout = TIMEOUT;
	i2s_cfg.options |= I2S_OPT_LOOPBACK;

	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);
	zassert_equal(ret, 0, "Failed to configure I2S TX stream");
}

/** Configure I2S RX transfer. */
void test_i2s_rx_transfer_configure_0(void)
{
	const struct device *dev_i2s;
	struct i2s_config i2s_cfg;
	int ret;

	dev_i2s = device_get_binding(I2S_DEV_NAME_RX);
	zassert_not_null(dev_i2s, "device " I2S_DEV_NAME_RX " not found");

	/* Configure */

	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	/* Configure the Receive port as Slave */
	i2s_cfg.options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE;
	i2s_cfg.frame_clk_freq = FRAME_CLK_FREQ;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.mem_slab = &rx_0_mem_slab;
	i2s_cfg.timeout = TIMEOUT;
	i2s_cfg.options |= I2S_OPT_LOOPBACK;

	ret = i2s_configure(dev_i2s, I2S_DIR_RX, &i2s_cfg);
	zassert_equal(ret, 0, "Failed to configure I2S RX stream");
}

/** @brief Short I2S transfer.
 *
 * - TX stream START trigger starts transmission.
 * - RX stream START trigger starts reception.
 * - sending / receiving a short sequence of data returns success.
 * - TX stream DRAIN trigger empties the transmit queue.
 * - RX stream STOP trigger stops reception.
 */
void test_i2s_transfer_short(void)
{
	const struct device *dev_i2s_rx;
	const struct device *dev_i2s_tx;
	int ret;

	dev_i2s_rx = device_get_binding(I2S_DEV_NAME_RX);
	zassert_not_null(dev_i2s_rx, "device " I2S_DEV_NAME_RX " not found");

	dev_i2s_tx = device_get_binding(I2S_DEV_NAME_TX);
	zassert_not_null(dev_i2s_tx, "device " I2S_DEV_NAME_TX " not found");

	/* Prefill TX queue */
	ret = tx_block_write(dev_i2s_tx, 0, 0);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d->OK\n", 1);

	ret = tx_block_write(dev_i2s_tx, 1, 0);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d->OK\n", 2);

	/* Start reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX START trigger failed");

	/* Start transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "TX START trigger failed");

	ret = rx_block_read(dev_i2s_rx, 0);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d<-OK\n", 1);

	ret = tx_block_write(dev_i2s_tx, 2, 0);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d->OK\n", 3);

	/* All data written, drain TX queue and stop the transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	zassert_equal(ret, 0, "TX DRAIN trigger failed");

	ret = rx_block_read(dev_i2s_rx, 1);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d<-OK\n", 2);

	/* All but one data block read, stop reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_STOP);
	zassert_equal(ret, 0, "RX STOP trigger failed");

	ret = rx_block_read(dev_i2s_rx, 2);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d<-OK\n", 3);

	/* TODO: Verify the interface is in READY state when i2s_state_get
	 * function is available.
	 */
}

#define TEST_I2S_TRANSFER_LONG_REPEAT_COUNT  100

/** @brief Long I2S transfer.
 *
 * - TX stream START trigger starts transmission.
 * - RX stream START trigger starts reception.
 * - sending / receiving a long sequence of data returns success.
 * - TX stream DRAIN trigger empties the transmit queue.
 * - RX stream STOP trigger stops reception.
 */
void test_i2s_transfer_long(void)
{
	const struct device *dev_i2s_rx;
	const struct device *dev_i2s_tx;
	int ret;

	dev_i2s_rx = device_get_binding(I2S_DEV_NAME_RX);
	zassert_not_null(dev_i2s_rx, "device " I2S_DEV_NAME_RX " not found");

	dev_i2s_tx = device_get_binding(I2S_DEV_NAME_TX);
	zassert_not_null(dev_i2s_tx, "device " I2S_DEV_NAME_TX " not found");

	/* Prefill TX queue */
	ret = tx_block_write(dev_i2s_tx, 0, 0);
	zassert_equal(ret, TC_PASS, NULL);

	/* Start reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX START trigger failed");

	/* Start transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "TX START trigger failed");

	for (int i = 0; i < TEST_I2S_TRANSFER_LONG_REPEAT_COUNT; i++) {
		ret = tx_block_write(dev_i2s_tx, 0, 0);
		zassert_equal(ret, TC_PASS, NULL);

		ret = rx_block_read(dev_i2s_rx, 0);
		zassert_equal(ret, TC_PASS, NULL);
	}

	/* All data written, flush TX queue and stop the transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	zassert_equal(ret, 0, "TX DRAIN trigger failed");

	/* All but one data block read, stop reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_STOP);
	zassert_equal(ret, 0, "RX STOP trigger failed");

	ret = rx_block_read(dev_i2s_rx, 0);
	zassert_equal(ret, TC_PASS, NULL);

	/* TODO: Verify the interface is in READY state when i2s_state_get
	 * function is available.
	 */
}

/** @brief RX sync start.
 *
 * - TX stream START trigger starts transmission.
 * - sending RX stream START trigger after a delay starts reception on the next
 *   word select sync event at the start of the frame.
 * - TX stream DROP trigger stops transmission and clears the transmit queue.
 * - RX stream DROP trigger stops reception and clears the receive queue.
 */
void test_i2s_rx_sync_start(void)
{
	const struct device *dev_i2s_rx;
	const struct device *dev_i2s_tx;
	size_t rx_size;
	int ret;
	char buf[BLOCK_SIZE];

	dev_i2s_rx = device_get_binding(I2S_DEV_NAME_RX);
	zassert_not_null(dev_i2s_rx, "device " I2S_DEV_NAME_RX " not found");

	dev_i2s_tx = device_get_binding(I2S_DEV_NAME_TX);
	zassert_not_null(dev_i2s_tx, "device " I2S_DEV_NAME_TX " not found");

	/* Prefill TX queue */
	for (int n = 0; n < NUM_TX_BLOCKS; n++) {
		fill_buf_const((uint16_t *)buf, 1, 2);
		ret = i2s_buf_write(dev_i2s_tx, buf, BLOCK_SIZE);
		zassert_equal(ret, TC_PASS, NULL);
		TC_PRINT("%d->OK\n", n);
	}

	/* Start transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "TX START trigger failed");

	k_busy_wait(75);

	/* Start reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX START trigger failed");
	ret = i2s_buf_read(dev_i2s_rx, buf, &rx_size);
	zassert_equal(ret, TC_PASS, NULL);
	ret = verify_buf_const((uint16_t *)buf, 1, 2);

	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d<-OK\n", 1);

	/* All data written, drop TX, RX queue and stop the transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DROP);
	zassert_equal(ret, 0, "TX DROP trigger failed");

	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_DROP);
	zassert_equal(ret, 0, "RX DROP trigger failed");

	/* TODO: Verify the interface is in READY state when i2s_state_get
	 * function is available.
	 */
}

/** @brief Timeout on RX queue empty.
 *
 * - Reading empty RX queue in READY state returns time out error.
 */
void test_i2s_rx_empty_timeout(void)
{
	const struct device *dev_i2s;
	size_t rx_size;
	int ret;
	char buf[BLOCK_SIZE];

	dev_i2s = device_get_binding(I2S_DEV_NAME_RX);
	zassert_not_null(dev_i2s, "device " I2S_DEV_NAME_RX " not found");

	ret = i2s_buf_read(dev_i2s, buf, &rx_size);
	zassert_equal(ret, -EAGAIN, "i2s_read did not timed out");
}

/** @brief Re-start I2S transfer.
 *
 * - STOP trigger stops transfer / reception at the end of the current block,
 *   consecutive START trigger restarts transfer / reception with the next data
 *   block.
 */
void test_i2s_transfer_restart(void)
{
	const struct device *dev_i2s_rx;
	const struct device *dev_i2s_tx;
	int ret;

	dev_i2s_rx = device_get_binding(I2S_DEV_NAME_RX);
	zassert_not_null(dev_i2s_rx, "device " I2S_DEV_NAME_RX " not found");

	dev_i2s_tx = device_get_binding(I2S_DEV_NAME_TX);
	zassert_not_null(dev_i2s_tx, "device " I2S_DEV_NAME_TX " not found");

	/* Prefill TX queue */
	ret = tx_block_write(dev_i2s_tx, 0, 0);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d->OK\n", 1);

	ret = tx_block_write(dev_i2s_tx, 1, 0);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d->OK\n", 2);

	/* Start reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX START trigger failed");

	/* Start transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "TX START trigger failed");

	/* Stop transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_STOP);
	zassert_equal(ret, 0, "TX STOP trigger failed");

	/* Stop reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_STOP);
	zassert_equal(ret, 0, "RX STOP trigger failed");

	ret = rx_block_read(dev_i2s_rx, 0);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d<-OK\n", 1);

	TC_PRINT("Stop transmission\n");

	/* Keep interface inactive */
	k_sleep(K_MSEC(1000));

	TC_PRINT("Start transmission\n");

	/* Prefill TX queue */
	ret = tx_block_write(dev_i2s_tx, 2, 0);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d->OK\n", 3);

	/* Start reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX START trigger failed");

	/* Start transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "TX START trigger failed");

	/* All data written, drain TX queue and stop the transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	zassert_equal(ret, 0, "TX DRAIN trigger failed");

	ret = rx_block_read(dev_i2s_rx, 1);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d<-OK\n", 2);

	/* All but one data block read, stop reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_STOP);
	zassert_equal(ret, 0, "RX STOP trigger failed");

	ret = rx_block_read(dev_i2s_rx, 2);
	zassert_equal(ret, TC_PASS, NULL);
	TC_PRINT("%d<-OK\n", 3);
}

/** @brief RX buffer overrun.
 *
 * - In case of RX buffer overrun it is possible to read out RX data blocks
 *   that are stored in the RX queue.
 * - Reading from an empty RX queue when the RX buffer overrun occurred results
 *   in an error.
 * - Sending PREPARE trigger after the RX buffer overrun occurred changes
 *   the interface state to READY.
 */
void test_i2s_transfer_rx_overrun(void)
{
	const struct device *dev_i2s_rx;
	const struct device *dev_i2s_tx;
	size_t rx_size;
	int ret;
	char rx_buf[BLOCK_SIZE];

	dev_i2s_rx = device_get_binding(I2S_DEV_NAME_RX);
	zassert_not_null(dev_i2s_rx, "device " I2S_DEV_NAME_RX " not found");

	dev_i2s_tx = device_get_binding(I2S_DEV_NAME_TX);
	zassert_not_null(dev_i2s_tx, "device " I2S_DEV_NAME_TX " not found");

	/* Prefill TX queue */
	ret = tx_block_write(dev_i2s_tx, 0, 0);
	zassert_equal(ret, TC_PASS, NULL);

	/* Start reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX START trigger failed");

	/* Start transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "TX START trigger failed");

	for (int i = 0; i < NUM_RX_BLOCKS; i++) {
		ret = tx_block_write(dev_i2s_tx, 0, 0);
		zassert_equal(ret, TC_PASS, NULL);
	}

	/* All data written, flush TX queue and stop the transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	zassert_equal(ret, 0, "TX DRAIN trigger failed");

	/* Wait for transmission to finish */
	k_sleep(K_MSEC(200));

	/* Read one data block, expect success even if RX queue is already in
	 * the error state.
	 */
	ret = rx_block_read(dev_i2s_rx, 0);
	zassert_equal(ret, TC_PASS, NULL);

	/* Attempt to read more data blocks than are available in the RX queue */
	for (int i = 0; i < NUM_RX_BLOCKS; i++) {
		ret = i2s_buf_read(dev_i2s_rx, rx_buf, &rx_size);
		if (ret != 0) {
			break;
		}
	}
	zassert_equal(ret, -EIO, "RX overrun error not detected");

	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_PREPARE);
	zassert_equal(ret, 0, "RX PREPARE trigger failed");

	/* Transmit and receive one more data block */
	ret = tx_block_write(dev_i2s_tx, 0, 0);
	zassert_equal(ret, TC_PASS, NULL);
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX START trigger failed");
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "TX START trigger failed");
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	zassert_equal(ret, 0, "TX DRAIN trigger failed");
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_STOP);
	zassert_equal(ret, 0, "RX STOP trigger failed");
	ret = rx_block_read(dev_i2s_rx, 0);
	zassert_equal(ret, TC_PASS, NULL);

	k_sleep(K_MSEC(200));
}

/** @brief TX buffer underrun.
 *
 * - An attempt to write to the TX queue when TX buffer underrun has occurred
 *   results in an error.
 * - Sending PREPARE trigger after the TX buffer underrun occurred changes
 *   the interface state to READY.
 */
void test_i2s_transfer_tx_underrun(void)
{
	const struct device *dev_i2s_rx;
	const struct device *dev_i2s_tx;
	int ret;

	dev_i2s_rx = device_get_binding(I2S_DEV_NAME_RX);
	zassert_not_null(dev_i2s_rx, "device " I2S_DEV_NAME_RX " not found");

	dev_i2s_tx = device_get_binding(I2S_DEV_NAME_TX);
	zassert_not_null(dev_i2s_tx, "device " I2S_DEV_NAME_TX " not found");

	/* Prefill TX queue */
	ret = tx_block_write(dev_i2s_tx, 0, 0);
	zassert_equal(ret, TC_PASS, NULL);

	/* Start reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX START trigger failed");

	/* Start transmission */
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "TX START trigger failed");

	/* Stop reception */
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_STOP);
	zassert_equal(ret, 0, "RX STOP trigger failed");

	ret = rx_block_read(dev_i2s_rx, 0);
	zassert_equal(ret, TC_PASS, NULL);

	k_sleep(K_MSEC(200));

	/* Write one more TX data block, expect an error */
	ret = tx_block_write(dev_i2s_tx, 2, -EIO);
	zassert_equal(ret, TC_PASS, NULL);

	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
	zassert_equal(ret, 0, "TX PREPARE trigger failed");

	k_sleep(K_MSEC(200));

	/* Transmit and receive two more data blocks */
	ret = tx_block_write(dev_i2s_tx, 1, 0);
	zassert_equal(ret, TC_PASS, NULL);
	ret = tx_block_write(dev_i2s_tx, 1, 0);
	zassert_equal(ret, TC_PASS, NULL);
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "RX START trigger failed");
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_START);
	zassert_equal(ret, 0, "TX START trigger failed");
	ret = rx_block_read(dev_i2s_rx, 1);
	zassert_equal(ret, TC_PASS, NULL);
	ret = i2s_trigger(dev_i2s_tx, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	zassert_equal(ret, 0, "TX DRAIN trigger failed");
	ret = i2s_trigger(dev_i2s_rx, I2S_DIR_RX, I2S_TRIGGER_STOP);
	zassert_equal(ret, 0, "RX STOP trigger failed");
	ret = rx_block_read(dev_i2s_rx, 1);
	zassert_equal(ret, TC_PASS, NULL);

	k_sleep(K_MSEC(200));
}
