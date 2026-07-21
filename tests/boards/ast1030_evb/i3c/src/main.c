/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/target/i3c_target_mqueue.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>
#include "ast_test.h"

#define TEST_PRIV_XFER_SIZE   128
#define TEST_IBI_PAYLOAD_SIZE 128
#define MAX_DATA_SIZE         256

/* Loopback iteration count: the old CI test took this as a `count` argument
 * from the runner; the per-peripheral test model has no such plumbing
 * (see pwm_tach/spi), so a small fixed repeat count is hardcoded instead.
 */
#define TEST_I3C_LOOP_COUNT 3

/*
 * MIPI manufacturer ID that ASPEED I3C controllers report via GETPID when
 * operating in target/secondary mode. Mirrors MIPI_MFG_ASPEED in
 * drivers/i3c/i3c_aspeed.c (aspeed_i3c_init_pid()), which is what i3c1
 * (in target mode) will actually answer with.
 */
#define I3C_PID_MANUF_ID_ASPEED 0x3f6

/*
 * IBI Mandatory Data Byte (MDB) group encoding, mirrors the identical
 * check in drivers/i3c/i3c_dummy_device.c: the top 3 bits of the MDB
 * identify the MIPI-defined "Pending Read Notification" group.
 */
#define IBI_MDB_GROUP                   GENMASK(7, 5)
#define IBI_MDB_GROUP_PENDING_READ_NOTI 5
#define IS_MDB_PENDING_READ_NOTIFY(mdb) \
	(FIELD_GET(IBI_MDB_GROUP, (mdb)) == IBI_MDB_GROUP_PENDING_READ_NOTI)

#define I3C_TARGET_THREAD_STACK_SIZE 512
#define I3C_TARGET_THREAD_PRIO       CONFIG_ZTEST_THREAD_PRIORITY

static const struct device *const i3c0_dev = DEVICE_DT_GET(DT_NODELABEL(i3c0));
static const struct device *const i3c1_smq_dev = DEVICE_DT_GET(DT_NODELABEL(i3c1_smq));

/*
 * i3c1 is not declared as a static DT child of i3c0: it is attached at
 * runtime as a plain I3C device, exactly as the old test did with
 * i3c_master_attach_device(). `bus`/`static_addr` are const members of
 * struct i3c_device_desc, so they must come from the initializer.
 */
static struct i3c_device_desc i3c1_target = {
	.bus = DEVICE_DT_GET(DT_NODELABEL(i3c0)),
	.static_addr = DT_PROP(DT_BUS(DT_NODELABEL(i3c1_smq)), assigned_address),
};

K_THREAD_STACK_DEFINE(i3c_target_thread_stack, I3C_TARGET_THREAD_STACK_SIZE);
static struct k_thread i3c_target_thread;

static uint8_t test_data_tx[MAX_DATA_SIZE];
static uint8_t test_data_rx[MAX_DATA_SIZE];

static struct k_sem ibi_complete;
static uint8_t ibi_mdb;

/*
 * The target-side helper thread must never call ast_zassert_*() either:
 * even though ast_zassert_* doesn't longjmp (that's the whole point), the
 * shared static ast_test_fail flag it writes still isn't safe to touch
 * from two threads at once. Record failures here instead and let
 * test_i3c() (always called from a single thread - either the ztest
 * runner, standalone, or one of the concurrent runner's worker threads,
 * never both at once) check it once per round-trip.
 */
static atomic_t target_side_ok = ATOMIC_INIT(1);

static void prepare_test_data(uint8_t *data, int nbytes)
{
	uint32_t value = sys_rand32_get();
	uint32_t shift;

	for (int i = 0; i < nbytes; i++) {
		shift = (i & 0x3) * 8;
		data[i] = (value >> shift) & 0xff;
		if ((i & 0x3) == 0x3) {
			value = sys_rand32_get();
		}
	}
}

/* Mirrors bytes_to_pid() in drivers/i3c/i3c_aspeed.c: PID is big-endian,
 * byte[0] is the MSB of the 48-bit Provisioned ID.
 */
static uint64_t bytes_to_pid(const uint8_t *bytes)
{
	uint64_t pid = 0;

	for (int i = 0; i < 6; i++) {
		pid |= (uint64_t)bytes[i] << ((6 - i - 1) * 8);
	}

	return pid;
}

static int test_i3c_ibi_cb(struct i3c_device_desc *target, struct i3c_ibi_payload *payload)
{
	ARG_UNUSED(target);

	if (payload->payload_len > 0) {
		ibi_mdb = payload->payload[0];
	}
	k_sem_give(&ibi_complete);

	return 0;
}

/*
 * Target-side responder: services the i3c1_smq (i3c-target-mqueue) device
 * that plays the role of the target on the loopback bus. Mirrors the old
 * test_i3c_slave_task(): wait for the private write from the controller,
 * verify it, then push fresh random data back out via a pending-read-notify
 * IBI for the controller to fetch.
 */
static void test_i3c_target_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *smq = p1;
	int ret;

	for (;;) {
		while (i3c_target_mqueue_read(smq, test_data_rx, TEST_PRIV_XFER_SIZE) == 0) {
			k_sleep(K_USEC(1));
		}

		if (memcmp(test_data_tx, test_data_rx, TEST_PRIV_XFER_SIZE) != 0) {
			atomic_set(&target_side_ok, 0);
		}

		prepare_test_data(test_data_tx, TEST_IBI_PAYLOAD_SIZE);
		ret = i3c_target_mqueue_write(smq, test_data_tx, TEST_IBI_PAYLOAD_SIZE);
		if (ret != 0) {
			atomic_set(&target_side_ok, 0);
		}
	}
}

int test_i3c(void)
{
	struct i3c_ccc_getpid getpid;
	struct i3c_ccc_getbcr getbcr;
	uint64_t pid;

	ast_zassert_true(device_is_ready(i3c0_dev), "i3c0 controller is not ready");
	ast_zassert_true(device_is_ready(i3c1_smq_dev), "i3c1 target mqueue device is not ready");

	k_sem_init(&ibi_complete, 0, 1);
	atomic_set(&target_side_ok, 1);

	k_thread_create(&i3c_target_thread, i3c_target_thread_stack,
			I3C_TARGET_THREAD_STACK_SIZE, test_i3c_target_task,
			(void *)i3c1_smq_dev, NULL, NULL, I3C_TARGET_THREAD_PRIO, 0, K_NO_WAIT);

	ast_zassert_ok(i3c_attach_i3c_device(&i3c1_target),
		       "failed to attach target device onto the bus");

	ast_zassert_ok(i3c_ccc_do_rstdaa_all(i3c0_dev), "failed to send RSTDAA");
	ast_zassert_ok(i3c_ccc_do_setaasa_all(i3c0_dev), "failed to send SETAASA");

	/* SETAASA assigns each statically-addressed target's dynamic address
	 * to equal its static address; the CCC helpers address targets via
	 * i3c_device_desc::dynamic_addr, so update our local copy to match.
	 */
	i3c1_target.dynamic_addr = i3c1_target.static_addr;

	ast_zassert_ok(i3c_ccc_do_getpid(&i3c1_target, &getpid), "failed to send GETPID");
	pid = bytes_to_pid(getpid.pid);
	ast_zassert_equal(I3C_PID_MANUF_ID(pid), I3C_PID_MANUF_ID_ASPEED,
			   "incorrect manufacturer ID %llx", pid);

	ast_zassert_ok(i3c_ccc_do_getbcr(&i3c1_target, &getbcr), "failed to send GETBCR");
	i3c1_target.bcr = getbcr.bcr;
	ast_zassert_equal(i3c1_target.bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE,
			   I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE, "incorrect BCR %x",
			   i3c1_target.bcr);

	i3c1_target.ibi_cb = test_i3c_ibi_cb;
	ast_zassert_ok(i3c_ibi_enable(&i3c1_target), "failed to enable IBI");

	/* Give the target side time to be ready to raise IBI. */
	k_msleep(2000);

	for (int i = 0; i < TEST_I3C_LOOP_COUNT; i++) {
		/*
		 * Test part 1:
		 * controller --- private write transfer ---> target
		 */
		prepare_test_data(test_data_tx, TEST_PRIV_XFER_SIZE);
		ast_zassert_ok(i3c_write(&i3c1_target, test_data_tx, TEST_PRIV_XFER_SIZE),
			       "failed to do private write transfer (iteration %d)", i);

		/*
		 * Test part 2:
		 * target raises an IBI carrying the mandatory-data-byte (MDB);
		 * for this driver, the MDB always falls in the MIPI Pending
		 * Read Notification group, so the controller follows up with
		 * a private read to fetch the data the target queued.
		 */
		ast_zassert_ok(k_sem_take(&ibi_complete, K_SECONDS(2)),
			       "timed out waiting for IBI (iteration %d)", i);

		ast_zassert_equal(ibi_mdb, DT_PROP(DT_NODELABEL(i3c1_smq), mandatory_data_byte),
				   "IBI MDB mismatch: %02x %02x", ibi_mdb,
				   DT_PROP(DT_NODELABEL(i3c1_smq), mandatory_data_byte));

		if (IS_MDB_PENDING_READ_NOTIFY(ibi_mdb)) {
			ast_zassert_ok(i3c_read(&i3c1_target, test_data_rx, TEST_IBI_PAYLOAD_SIZE),
				       "failed to do private read transfer (iteration %d)", i);
			ast_zassert_mem_equal(test_data_tx, test_data_rx, TEST_IBI_PAYLOAD_SIZE,
					       "IBI payload data mismatch (iteration %d)", i);
		} else {
			ast_zassert_mem_equal(test_data_tx, &test_data_rx[1], TEST_IBI_PAYLOAD_SIZE,
					       "IBI payload data mismatch (iteration %d)", i);
		}

		ast_zassert_true(atomic_get(&target_side_ok),
				  "target-side write/compare failed (iteration %d)", i);
	}

	ast_zassert_ok(i3c_detach_i3c_device(&i3c1_target), "failed to detach target device");

	k_thread_abort(&i3c_target_thread);

	return ast_ztest_result();
}

#if !defined(AST1030_CONCURRENT_ALL)
ZTEST(i3c, test_i3c_all)
{
	zassert_equal(test_i3c(), AST_TEST_PASS, "i3c test failed");
}

ZTEST_SUITE(i3c, NULL, NULL, NULL, NULL, NULL);
#endif
