/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Generic I3C controller/target loopback test.
 *
 * Designed to run on any board that wires two I3C controllers
 * together (SDA/SCL of the controller-side instance shorted to the
 * target-side instance externally) and supplies two DT chosen
 * entries:
 *
 *   zephyr,i3c-loopback-controller — the primary controller node
 *   zephyr,i3c-loopback-target     — the secondary target node
 *
 * The board overlay also adds the controller's static-address child
 * (bound to "i3c-dummy-device") so the controller can resolve the
 * loopback target during bus initialisation.
 *
 * Two phases are exercised:
 *   Phase W : controller -> target private write; the target's
 *             write_received_cb captures bytes and compares them
 *             against the expected pattern.
 *   Phase PR: target arms a TX buffer through
 *             i3c_target_pending_read_notify() and raises an IBI;
 *             once the controller's ibi_cb fires, it issues an
 *             i3c_read() to drain the pre-armed bytes.
 *
 * The test prints "I3C LOOPBACK: PASS" or "I3C LOOPBACK: FAIL" so
 * Twister's console harness can summarise the result.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/drivers/i3c/ibi.h>
#include <zephyr/drivers/i3c/target_device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i3c_loopback, LOG_LEVEL_INF);

#define CTRL_NODE	DT_CHOSEN(zephyr_i3c_loopback_controller)
#define TARGET_NODE	DT_CHOSEN(zephyr_i3c_loopback_target)

#if !DT_NODE_EXISTS(CTRL_NODE) || !DT_NODE_EXISTS(TARGET_NODE)
#error \
"This test requires zephyr,i3c-loopback-controller and zephyr,i3c-loopback-target chosens."
#endif

#define LOOPBACK_PATTERN_LEN 16
#define IBI_PAYLOAD_LEN      4
#define PR_PAYLOAD_LEN       8
#define TARGET_WORKER_STACK  2048

/*
 * Park the TX pattern in .data (drop the const) rather than .rodata.
 * On AST2700 SSP the .rodata image lives in dram_ro_region (virt
 * 0x0..0x20000) which maps through a different SCU remap than the .data
 * / .bss in dram_rw_region (virt 0x20000+). Letting the I3C DMA reach
 * for .rodata can return phys bytes that disagree with what the CPU
 * sees through the SSP cache. .data sits next to the Phase PR data
 * pattern that is already known to round-trip correctly.
 */
static uint8_t controller_to_target[LOOPBACK_PATTERN_LEN] = {
	0xA5, 0x5A, 0x01, 0x02, 0x03, 0x04, 0xDE, 0xAD,
	0xBE, 0xEF, 0xCA, 0xFE, 0xF0, 0x0D, 0x00, 0xFF,
};

static uint8_t pr_ibi_pattern[IBI_PAYLOAD_LEN] = {
	0xDE, 0xAD, 0xBE, 0xEF,
};

static uint8_t pr_data_pattern[PR_PAYLOAD_LEN] = {
	0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
};

struct loopback_target {
	struct i3c_target_config cfg;
	uint8_t rx_buf[LOOPBACK_PATTERN_LEN];
	size_t rx_len;
	struct k_sem write_done;
	struct k_sem stop_seen;
};

static struct loopback_target tgt_state;

static int loop_write_requested(struct i3c_target_config *cfg)
{
	struct loopback_target *t = CONTAINER_OF(cfg, struct loopback_target, cfg);

	t->rx_len = 0U;
	return 0;
}

static int loop_write_received(struct i3c_target_config *cfg, uint8_t val)
{
	struct loopback_target *t = CONTAINER_OF(cfg, struct loopback_target, cfg);

	if (t->rx_len >= sizeof(t->rx_buf)) {
		return -ENOSPC;
	}
	t->rx_buf[t->rx_len++] = val;
	return 0;
}

static int loop_stop(struct i3c_target_config *cfg)
{
	struct loopback_target *t = CONTAINER_OF(cfg, struct loopback_target, cfg);

	if (t->rx_len > 0U) {
		k_sem_give(&t->write_done);
	}
	k_sem_give(&t->stop_seen);
	return 0;
}

static const struct i3c_target_callbacks loop_callbacks = {
	.write_requested_cb = loop_write_requested,
	.write_received_cb  = loop_write_received,
	.stop_cb            = loop_stop,
};

static struct k_sem ctrl_ibi_done;
static uint8_t ctrl_ibi_payload[CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE];
static size_t ctrl_ibi_payload_len;

static int ctrl_ibi_cb(struct i3c_device_desc *target,
		       struct i3c_ibi_payload *payload)
{
	ARG_UNUSED(target);

	if (payload) {
		ctrl_ibi_payload_len = MIN(payload->payload_len,
					   sizeof(ctrl_ibi_payload));
		(void)memcpy(ctrl_ibi_payload, payload->payload,
			     ctrl_ibi_payload_len);
	} else {
		ctrl_ibi_payload_len = 0U;
	}
	k_sem_give(&ctrl_ibi_done);
	return 0;
}

static struct i3c_device_desc *find_loopback_desc(const struct device *ctrl)
{
	const struct i3c_device_id id = {
		.pid = ((uint64_t)0x7ec << 32) | 0x06010000ULL,
	};

	return i3c_device_find(ctrl, &id);
}

static int retry_bus_init(const struct device *ctrl)
{
	const struct i3c_driver_config *cfg = ctrl->config;

	if (!cfg) {
		return -ENODEV;
	}
	return i3c_bus_init(ctrl, &cfg->dev_list);
}

K_THREAD_STACK_DEFINE(target_worker_stack, TARGET_WORKER_STACK);
static struct k_thread target_worker_thread;
static struct k_sem target_worker_done;
static volatile int target_worker_result;

static void target_worker(void *p1, void *p2, void *p3)
{
	const struct device *target = p1;
	int ret;
	struct i3c_ibi notifier = {
		.ibi_type = I3C_IBI_TARGET_INTR,
		.payload = pr_ibi_pattern,
		.payload_len = sizeof(pr_ibi_pattern),
	};

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ret = i3c_target_pending_read_notify(target, pr_data_pattern,
					     sizeof(pr_data_pattern),
					     &notifier);
	target_worker_result = ret;
	k_sem_give(&target_worker_done);
}

static int spawn_target_worker(const struct device *target)
{
	target_worker_result = -EBUSY;
	k_sem_reset(&target_worker_done);

	k_thread_create(&target_worker_thread, target_worker_stack,
			K_THREAD_STACK_SIZEOF(target_worker_stack),
			target_worker,
			(void *)target, NULL, NULL,
			K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(&target_worker_thread, "i3c-tgt-worker");
	return 0;
}

static bool wait_target_worker(int timeout_ms)
{
	if (k_sem_take(&target_worker_done, K_MSEC(timeout_ms)) != 0) {
		printk("  target worker did not finish in %d ms\n", timeout_ms);
		return false;
	}
	printk("  target worker returned %d\n", target_worker_result);
	return target_worker_result >= 0;
}

static bool phase_w(struct i3c_device_desc *desc)
{
	int ret;

	printk("Phase W: controller writes %u bytes -> target...\n",
	       LOOPBACK_PATTERN_LEN);
	tgt_state.rx_len = 0U;
	k_sem_reset(&tgt_state.write_done);
	k_sem_reset(&tgt_state.stop_seen);

	ret = i3c_write(desc, controller_to_target,
			sizeof(controller_to_target));
	printk("  i3c_write returned %d\n", ret);
	if (ret < 0) {
		return false;
	}

	if (k_sem_take(&tgt_state.write_done, K_MSEC(500)) != 0) {
		printk("  target write callback did not fire (rx_len=%zu)\n",
		       tgt_state.rx_len);
		return false;
	}

	if (tgt_state.rx_len != sizeof(controller_to_target) ||
	    memcmp(tgt_state.rx_buf, controller_to_target,
		   sizeof(controller_to_target)) != 0) {
		printk("  payload mismatch (rx_len=%zu)\n", tgt_state.rx_len);
		printk("  expected:");
		for (size_t i = 0; i < sizeof(controller_to_target); i++) {
			printk(" %02x", controller_to_target[i]);
		}
		printk("\n  received:");
		for (size_t i = 0; i < tgt_state.rx_len; i++) {
			printk(" %02x", tgt_state.rx_buf[i]);
		}
		printk("\n");
		return false;
	}

	printk("  Phase W OK\n");
	return true;
}

/*
 * Re-arm IBI with a known max_ibi so the controller DAT entry has
 * DAT_0_IBI_PAYLOAD set. The dummy device hooks i3c_ibi_enable() during
 * device init when target->data_length.max_ibi is 0, which leaves the
 * controller side rejecting IBI payloads. Tear that down and re-enable
 * with our desired payload size before the pending-read test.
 */
static int rearm_ibi_with_payload(struct i3c_device_desc *desc,
				  uint16_t max_ibi_payload)
{
	int ret;

	ret = i3c_ibi_disable(desc);
	if (ret != 0 && ret != -ENODEV) {
		printk("  i3c_ibi_disable returned %d\n", ret);
		return ret;
	}

	desc->data_length.max_ibi = max_ibi_payload;
	desc->ibi_cb = ctrl_ibi_cb;

	ret = i3c_ibi_enable(desc);
	if (ret == -EALREADY) {
		ret = 0;
	}
	return ret;
}

static bool phase_pr(const struct device *target,
		     struct i3c_device_desc *desc)
{
	/*
	 * Park the receive buffer in .bss so the controller-side DMA (when
	 * the underlying HCI is in DMA mode) can deposit the bytes — TCM
	 * regions on some SoCs are CPU-only and not DMA-visible.
	 */
	static uint8_t rxbuf[PR_PAYLOAD_LEN] __aligned(4);
	int ret;
	bool ok = true;

	printk("Phase PR: target pending-read-notify (%u IBI + %u data)...\n",
	       IBI_PAYLOAD_LEN, PR_PAYLOAD_LEN);

	ctrl_ibi_payload_len = 0U;
	k_sem_reset(&ctrl_ibi_done);

	ret = rearm_ibi_with_payload(desc, IBI_PAYLOAD_LEN);
	printk("  rearm_ibi_with_payload returned %d (max_ibi=%u)\n",
	       ret, desc->data_length.max_ibi);
	if (ret < 0) {
		return false;
	}

	printk("  spawning target worker (pending-read-notify)\n");
	spawn_target_worker(target);

	printk("  waiting up to 1500 ms for pending-read IBI...\n");
	if (k_sem_take(&ctrl_ibi_done, K_MSEC(1500)) != 0) {
		printk("  controller did not receive pending-read IBI\n");
		ok = false;
	} else {
		printk("  pending-read IBI fired (payload_len=%zu)\n",
		       ctrl_ibi_payload_len);
		if (ctrl_ibi_payload_len != sizeof(pr_ibi_pattern) ||
		    memcmp(ctrl_ibi_payload, pr_ibi_pattern,
			   sizeof(pr_ibi_pattern)) != 0) {
			printk("  pending-read IBI payload mismatch\n");
			ok = false;
		}
	}

	(void)memset(rxbuf, 0, sizeof(rxbuf));
	ret = i3c_read(desc, rxbuf, sizeof(rxbuf));
	printk("  i3c_read returned %d\n", ret);
	if (ret < 0) {
		ok = false;
	}

	if (!wait_target_worker(2500)) {
		ok = false;
	}

	if (ok && memcmp(rxbuf, pr_data_pattern, sizeof(pr_data_pattern)) != 0) {
		printk("  pending-read data mismatch\n");
		printk("  expected:");
		for (size_t i = 0; i < sizeof(pr_data_pattern); i++) {
			printk(" %02x", pr_data_pattern[i]);
		}
		printk("\n  received:");
		for (size_t i = 0; i < sizeof(rxbuf); i++) {
			printk(" %02x", rxbuf[i]);
		}
		printk("\n  rxbuf @%p pr_data_pattern @%p\n",
		       (void *)rxbuf, (void *)pr_data_pattern);
		ok = false;
	}

	if (ok) {
		printk("  Phase PR OK\n");
	}
	return ok;
}

int main(void)
{
	const struct device *ctrl = DEVICE_DT_GET(CTRL_NODE);
	const struct device *target = DEVICE_DT_GET(TARGET_NODE);
	struct i3c_device_desc *desc;
	int ret;
	bool ok = true;

	printk("\n=== I3C loopback test ===\n");
	printk("controller=%s  target=%s\n", ctrl->name, target->name);
	printk("controller ready=%s  target ready=%s\n",
	       device_is_ready(ctrl) ? "yes" : "no",
	       device_is_ready(target) ? "yes" : "no");

	if (!device_is_ready(target)) {
		printk("I3C LOOPBACK: FAIL (target device not ready)\n");
		return 0;
	}

	k_sem_init(&tgt_state.write_done, 0, 1);
	k_sem_init(&tgt_state.stop_seen, 0, 1);
	k_sem_init(&target_worker_done, 0, 1);
	k_sem_init(&ctrl_ibi_done, 0, 1);
	tgt_state.cfg.address = 0x70;
	tgt_state.cfg.callbacks = &loop_callbacks;

	ret = i3c_target_register(target, &tgt_state.cfg);
	if (ret != 0) {
		printk("I3C LOOPBACK: FAIL (target_register=%d)\n", ret);
		return 0;
	}
	printk("target registered at static address 0x70\n");

	/* Allow the target HCI a moment to settle before the controller talks. */
	k_msleep(50);

	if (!device_is_ready(ctrl)) {
		printk("controller not ready (likely SETDASA timeout); retrying...\n");
		printk("  hint: confirm SDA/SCL of controller/target are wired together\n");
		ret = retry_bus_init(ctrl);
		printk("  retry_bus_init=%d\n", ret);
		if (ret != 0) {
			printk("I3C LOOPBACK: FAIL (controller unusable)\n");
			return 0;
		}
	}

	desc = find_loopback_desc(ctrl);
	if (!desc || desc->dynamic_addr == 0U) {
		printk("loopback target not yet on bus; rerunning bus init...\n");
		ret = retry_bus_init(ctrl);
		printk("  retry_bus_init=%d\n", ret);
		desc = find_loopback_desc(ctrl);
	}

	if (!desc || desc->dynamic_addr == 0U) {
		printk("I3C LOOPBACK: FAIL (loopback target not found on bus)\n");
		printk("  hint 1: verify SDA/SCL pull-ups\n");
		printk("  hint 2: verify controller/target SoC pads are muxed to I3C\n");
		return 0;
	}

	printk("controller sees target dynamic_addr=0x%02x\n", desc->dynamic_addr);

	if (!phase_w(desc)) {
		ok = false;
	}

	if (!phase_pr(target, desc)) {
		ok = false;
	}

	if (ok) {
		printk("I3C LOOPBACK: PASS\n");
	} else {
		printk("I3C LOOPBACK: FAIL\n");
	}
	return 0;
}
