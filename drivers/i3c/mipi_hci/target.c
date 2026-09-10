/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/logging/log.h>

#include "cmd.h"
#include "hci.h"

LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

#define MIPI_I3C_HCI_TARGET_TIMEOUT_MS 1000
#define MIPI_I3C_HCI_TARGET_POLL_TIMEOUT_US 1000000
#define MIPI_I3C_HCI_TARGET_RX_FALLBACK_LEN 128U

#define PIO_QUEUE_SIZE 0x18
#define TX_DATA_BUFFER_SIZE GENMASK(31, 24)

#define CMD_0_ATTR_T_R FIELD_PREP(CMD_0_ATTR, 0x0)
#define CMD_0_ATTR_T_I FIELD_PREP(CMD_0_ATTR, 0x1)
#define CMD_T0_DATA_LENGTH(v) FIELD_PREP(W0_MASK(31, 16), v)
#define CMD_T0_MDB(v) FIELD_PREP(W0_MASK(15, 8), v)
#define CMD_T0_MDB_EN W0_BIT_(6)

struct mipi_i3c_hci_target_dt_props {
	const struct device *dev;
	uintptr_t base_regs;
	bool secondary;
	uint8_t assigned_addr;
	uint8_t dcr;
	uint16_t pid_extra_info;
	bool ibi_append_pec;
	bool priv_xfer_pec;
};

/*
 * Enumerate every node bound to either the generic "mipi,i3c-hci"
 * binding or a vendor-specific one, mirroring the instantiation list
 * in core.c. Vendor-only properties read through DT_PROP_OR default
 * to 0 on nodes whose binding does not define them.
 */
#define MIPI_I3C_HCI_TARGET_HAS_NODES                                                    \
	(DT_HAS_COMPAT_STATUS_OKAY(mipi_i3c_hci) ||                                      \
	 DT_HAS_COMPAT_STATUS_OKAY(aspeed_g7_i3c_hci))

#if MIPI_I3C_HCI_TARGET_HAS_NODES
#define MIPI_I3C_HCI_TARGET_DT_PROPS(node_id)                                            \
	{                                                                                 \
		.dev = DEVICE_DT_GET(node_id),                                            \
		.base_regs = DT_REG_ADDR(node_id),                                       \
		.secondary = DT_PROP_OR(node_id, secondary, 0),                          \
		.assigned_addr = DT_PROP_OR(node_id, assigned_address, 0),               \
		.dcr = DT_PROP_OR(node_id, dcr, 0),                                      \
		.pid_extra_info = DT_PROP_OR(node_id, pid_extra_info, 0),                \
		.ibi_append_pec = DT_PROP_OR(node_id, ibi_append_pec, 0),                \
		.priv_xfer_pec = DT_PROP_OR(node_id, priv_xfer_pec, 0),                  \
	},

static const struct mipi_i3c_hci_target_dt_props mipi_i3c_hci_target_dt_props[] = {
	DT_FOREACH_STATUS_OKAY(mipi_i3c_hci, MIPI_I3C_HCI_TARGET_DT_PROPS)
	DT_FOREACH_STATUS_OKAY(aspeed_g7_i3c_hci, MIPI_I3C_HCI_TARGET_DT_PROPS)
};
#endif

static const struct mipi_i3c_hci_target_dt_props *
mipi_i3c_hci_target_get_dt_props(struct i3c_hci *hci)
{
	if (!hci) {
		return NULL;
	}

#if MIPI_I3C_HCI_TARGET_HAS_NODES
	for (size_t i = 0; i < ARRAY_SIZE(mipi_i3c_hci_target_dt_props); i++) {
		if (mipi_i3c_hci_target_dt_props[i].dev == hci->dev ||
		    mipi_i3c_hci_target_dt_props[i].base_regs == hci->base_regs) {
			return &mipi_i3c_hci_target_dt_props[i];
		}
	}
#else
	ARG_UNUSED(hci);
#endif

	return NULL;
}

static bool mipi_i3c_hci_target_enabled(const struct mipi_i3c_hci_target_dt_props *props)
{
	return props && (props->secondary || props->assigned_addr != 0U);
}

static uint64_t mipi_i3c_hci_target_pid(struct i3c_hci *hci,
					const struct mipi_i3c_hci_target_dt_props *props)
{
	uint16_t extra_info = 0;

	if (props) {
		extra_info = props->pid_extra_info & GENMASK(11, 0);
	}

	if (hci->vendor && hci->vendor->target_pid) {
		return hci->vendor->target_pid(hci, extra_info);
	}

	return extra_info;
}

static uint8_t mipi_i3c_hci_target_supported_hdr(struct i3c_hci *hci)
{
	uint8_t supported = 0;

	if (hci->caps & HC_CAP_HDR_DDR_EN) {
		supported |= I3C_MSG_HDR_DDR;
	}

	if (hci->caps & HC_CAP_HDR_TS_EN) {
		if (hci_reg_read(hci, HC_CONTROL) & HC_CONTROL_I2C_TARGET_PRESENT) {
			supported |= I3C_MSG_HDR_TSL;
		} else {
			supported |= I3C_MSG_HDR_TSP;
		}
	}

	if (hci->caps & HC_CAP_HDR_BT_EN) {
		supported |= I3C_MSG_HDR_BT;
	}

	return supported;
}

static uint8_t mipi_i3c_hci_target_bcr(struct i3c_hci *hci)
{
	uint8_t bcr = I3C_BCR_IBI_REQUEST_CAPABLE | I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE;

	if (hci->is_secondary) {
		bcr |= I3C_BCR_DEVICE_ROLE_I3C_CONTROLLER_CAPABLE << I3C_BCR_DEVICE_ROLE_SHIFT;
	}

	if (mipi_i3c_hci_target_supported_hdr(hci) != 0U) {
		bcr |= I3C_BCR_ADV_CAPABILITIES;
	}

	return bcr;
}

static uint8_t mipi_i3c_hci_target_get_dynamic_addr(struct i3c_hci *hci)
{
	if (hci->vendor && hci->vendor->target_dynamic_addr) {
		return hci->vendor->target_dynamic_addr(hci);
	}

	return 0;
}

static void mipi_i3c_hci_target_update_registered_addr(struct i3c_hci *hci)
{
	uint8_t dyn_addr;

	if (!hci->target_cb) {
		return;
	}

	dyn_addr = mipi_i3c_hci_target_get_dynamic_addr(hci);
	if (dyn_addr != 0U) {
		hci->target_cb->address = dyn_addr;
	}
}

static int mipi_i3c_hci_target_alloc_rx_buf(struct i3c_hci *hci)
{
	uint32_t queue_size = 0;
	uint16_t max_len = MIPI_I3C_HCI_TARGET_RX_FALLBACK_LEN;

	if (hci->target_rx.buf) {
		return 0;
	}

	if (hci->PIO_regs != 0U) {
		queue_size = sys_read32((mem_addr_t)(hci->PIO_regs + PIO_QUEUE_SIZE));
		max_len = 4U * (2U << FIELD_GET(TX_DATA_BUFFER_SIZE, queue_size));
		if (max_len == 0U) {
			max_len = MIPI_I3C_HCI_TARGET_RX_FALLBACK_LEN;
		}
	}

	hci->target_rx.buf = k_malloc(max_len);
	if (!hci->target_rx.buf) {
		return -ENOMEM;
	}

	hci->target_rx.max_len = max_len;
	return 0;
}

static void mipi_i3c_hci_target_prep_read_data(struct hci_xfer *xfer, uint8_t *buf,
					       uint16_t len)
{
	xfer->data = buf;
	xfer->data_len = len;
	xfer->rnw = false;
	xfer->cmd_tid = TID_TARGET_RD_DATA;
	xfer->cmd_desc[0] = CMD_0_ATTR_T_R | CMD_T0_TID(TID_TARGET_RD_DATA) |
			    CMD_T0_DATA_LENGTH(len);
}

static int mipi_i3c_hci_target_prep_ibi_data(struct hci_xfer *xfer, struct i3c_ibi *req)
{
	if (!req || !req->payload || req->payload_len == 0U) {
		return -EINVAL;
	}

	xfer->data = &req->payload[1];
	xfer->data_len = req->payload_len - 1U;
	xfer->rnw = false;
	xfer->cmd_tid = TID_TARGET_IBI;
	xfer->cmd_desc[0] = CMD_0_ATTR_T_I | CMD_T0_TID(TID_TARGET_IBI) |
			    CMD_T0_MDB_EN | CMD_T0_MDB(req->payload[0]) |
			    CMD_T0_DATA_LENGTH(xfer->data_len);

	return 0;
}

static int mipi_i3c_hci_target_queue_xfer(struct i3c_hci *hci, struct hci_xfer *xfer)
{
	if (!hci->io || !hci->io->queue_xfer) {
		return -ENOSYS;
	}

	return hci->io->queue_xfer(hci, xfer, 1);
}

static bool mipi_i3c_hci_target_xfer_completed(struct k_sem *sem)
{
	return sem && k_sem_take(sem, K_NO_WAIT) == 0;
}

static void mipi_i3c_hci_target_release_xfer(struct i3c_hci *hci,
					     struct hci_xfer **xfer,
					     bool queued,
					     bool completed,
					     struct k_sem *completion)
{
	if (!xfer || !*xfer) {
		return;
	}

	if (queued && !completed &&
	    !mipi_i3c_hci_target_xfer_completed(completion)) {
		if (!hci->io || !hci->io->dequeue_xfer ||
		    !hci->io->dequeue_xfer(hci, *xfer, 1)) {
			LOG_WRN("%s target transfer TID %u still owned by backend; deferring free",
				hci->dev->name, (*xfer)->cmd_tid);
			*xfer = NULL;
			return;
		}
	}

	hci_free_xfer(*xfer, 1);
	*xfer = NULL;
}

static int mipi_i3c_hci_target_wait(struct k_sem *sem)
{
	int ret = k_sem_take(sem, K_MSEC(MIPI_I3C_HCI_TARGET_TIMEOUT_MS));

	return ret == 0 ? 0 : -ETIMEDOUT;
}

static int mipi_i3c_hci_target_submit_read_data(struct i3c_hci *hci, uint8_t *buf,
						uint16_t len)
{
	struct hci_xfer *xfer;
	bool queued = false;
	bool completed = false;
	int ret;

	xfer = hci_alloc_xfer(1);
	if (!xfer) {
		return -ENOMEM;
	}

	k_sem_reset(&hci->pending_r_comp);
	mipi_i3c_hci_target_prep_read_data(xfer, buf, len);

	ret = mipi_i3c_hci_target_queue_xfer(hci, xfer);
	if (ret == 0) {
		queued = true;
		ret = mipi_i3c_hci_target_wait(&hci->pending_r_comp);
		completed = ret == 0;
	}

	mipi_i3c_hci_target_release_xfer(hci, &xfer, queued, completed,
					 &hci->pending_r_comp);
	return ret;
}

static bool mipi_i3c_hci_target_event_enabled(struct i3c_hci *hci,
					      enum mipi_i3c_hci_target_event event)
{
	if (hci->vendor && hci->vendor->target_event_enabled) {
		return hci->vendor->target_event_enabled(hci, event);
	}

	return true;
}

static int mipi_i3c_hci_target_set_event(struct i3c_hci *hci,
					 enum mipi_i3c_hci_target_event event)
{
	if (hci->vendor && hci->vendor->target_request_event) {
		return hci->vendor->target_request_event(hci, event);
	}

	return -ENOTSUP;
}

static int mipi_i3c_hci_target_request_event_poll(struct i3c_hci *hci,
						  enum mipi_i3c_hci_target_event event)
{
	int ret;

	ret = mipi_i3c_hci_target_set_event(hci, event);
	if (ret != 0) {
		LOG_ERR("%s event %d: set_event failed: %d", hci->dev->name, event, ret);
		return ret;
	}

	if (!hci->vendor || !hci->vendor->target_request_pending) {
		LOG_DBG("%s event %d: no target_request_pending vendor hook, assume done",
			hci->dev->name, event);
		return 0;
	}

	LOG_DBG("%s event %d: request bit set, polling for HW to clear it", hci->dev->name,
		event);

	if (!WAIT_FOR(!hci->vendor->target_request_pending(hci, event),
		      MIPI_I3C_HCI_TARGET_POLL_TIMEOUT_US,
		      k_busy_wait(1))) {
		LOG_ERR("%s event %d: timed out waiting for HW to service request (bus master "
			"never responded?)", hci->dev->name, event);
		return -ETIMEDOUT;
	}

	LOG_DBG("%s event %d: request serviced by HW", hci->dev->name, event);

	return 0;
}

static int mipi_i3c_hci_target_raise_sir(struct i3c_hci *hci, struct i3c_ibi *req)
{
	struct hci_xfer *ibi_xfer = NULL;
	bool ibi_queued = false;
	bool ibi_completed = false;
	int ret;

	if (!mipi_i3c_hci_target_event_enabled(hci,
					       MIPI_I3C_HCI_TARGET_EVENT_IBI)) {
		return -EACCES;
	}

	if (req->payload_len != 0U) {
		if (hci->vendor && hci->vendor->set_ibi_terminate_len) {
			hci->vendor->set_ibi_terminate_len(hci, req->payload_len);
		}

		ibi_xfer = hci_alloc_xfer(1);
		if (!ibi_xfer) {
			return -ENOMEM;
		}

		ret = mipi_i3c_hci_target_prep_ibi_data(ibi_xfer, req);
		if (ret != 0) {
			goto out;
		}

		ret = mipi_i3c_hci_target_queue_xfer(hci, ibi_xfer);
		if (ret != 0) {
			goto out;
		}
		ibi_queued = true;
	}

	k_sem_reset(&hci->ibi_comp);
	ret = mipi_i3c_hci_target_set_event(hci,
					    MIPI_I3C_HCI_TARGET_EVENT_IBI);
	if (ret != 0) {
		goto out;
	}

	ret = mipi_i3c_hci_target_wait(&hci->ibi_comp);
	ibi_completed = ret == 0;

out:
	mipi_i3c_hci_target_release_xfer(hci, &ibi_xfer, ibi_queued, ibi_completed,
					 &hci->ibi_comp);

	return ret;
}

static int mipi_i3c_hci_target_raise_hotjoin(struct i3c_hci *hci)
{
	uint8_t dyn_addr;

	if (!hci->is_secondary) {
		LOG_ERR("%s hotjoin: refused, device is not configured as secondary controller",
			hci->dev->name);
		return -ENOTSUP;
	}

	dyn_addr = mipi_i3c_hci_target_get_dynamic_addr(hci);
	if (dyn_addr != 0U) {
		LOG_ERR("%s hotjoin: refused, already has dynamic address 0x%02x",
			hci->dev->name, dyn_addr);
		return -EALREADY;
	}

	if (!mipi_i3c_hci_target_event_enabled(hci,
					       MIPI_I3C_HCI_TARGET_EVENT_HOTJOIN)) {
		LOG_ERR("%s hotjoin: refused, HJ_EN not set by active controller (ENEC/DISEC?)",
			hci->dev->name);
		return -EACCES;
	}

	LOG_DBG("%s hotjoin: preconditions ok, requesting event", hci->dev->name);

	return mipi_i3c_hci_target_request_event_poll(hci, MIPI_I3C_HCI_TARGET_EVENT_HOTJOIN);
}

static int mipi_i3c_hci_target_raise_mastership(struct i3c_hci *hci)
{
	if (!hci->is_secondary) {
		return -ENOTSUP;
	}

	if (!mipi_i3c_hci_target_event_enabled(hci, MIPI_I3C_HCI_TARGET_EVENT_MASTER_REQUEST)) {
		return -EACCES;
	}

	return mipi_i3c_hci_target_request_event_poll(hci,
						      MIPI_I3C_HCI_TARGET_EVENT_MASTER_REQUEST);
}

int mipi_i3c_hci_target_init(struct i3c_hci *hci)
{
	const struct mipi_i3c_hci_target_dt_props *props;
	uint64_t pid;
	uint8_t bcr;
	uint8_t dcr;
	bool static_addr_en;
	int ret;

	props = mipi_i3c_hci_target_get_dt_props(hci);
	hci->is_secondary = props ? props->secondary : hci->common.ctrl_config.is_secondary;
	hci->common.ctrl_config.is_secondary = hci->is_secondary;
	hci->is_target = mipi_i3c_hci_target_enabled(props);

	if (!hci->is_target) {
		return 0;
	}

	ret = mipi_i3c_hci_target_alloc_rx_buf(hci);
	if (ret != 0) {
		return ret;
	}

	pid = mipi_i3c_hci_target_pid(hci, props);
	bcr = mipi_i3c_hci_target_bcr(hci);
	dcr = props ? props->dcr : 0U;
	static_addr_en = props && props->assigned_addr != 0U;

	if (hci->vendor && hci->vendor->set_slv_pid) {
		hci->vendor->set_slv_pid(hci, pid);
	}
	if (hci->vendor && hci->vendor->set_slv_char_ctrl) {
		hci->vendor->set_slv_char_ctrl(hci, bcr, dcr, static_addr_en);
	}
	if (hci->vendor && hci->vendor->target_enable_events) {
		hci->vendor->target_enable_events(hci,
						  mipi_i3c_hci_target_supported_hdr(hci));
	}

	ret = hci->vendor && hci->vendor->init ? hci->vendor->init(hci) : 0;
	if (ret != 0) {
		return ret;
	}

	if (hci->vendor && hci->vendor->target_set_mode) {
		hci->vendor->target_set_mode(hci);
	}

	LOG_DBG("%s initialized as %s target", hci->dev->name,
		hci->is_secondary ? "secondary-controller" : "slave");

	return 0;
}

int mipi_i3c_hci_target_register(const struct device *dev, struct i3c_target_config *cfg)
{
	struct i3c_hci *hci = dev->data;
	const struct mipi_i3c_hci_target_dt_props *props;
	uint8_t dyn_addr;

	if (!cfg || !cfg->callbacks) {
		return -EINVAL;
	}

	if (!hci->is_target) {
		return -ENOTSUP;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);

	if (hci->target_cb && hci->target_cb != cfg) {
		k_mutex_unlock(&hci->control_mutex);
		return -EBUSY;
	}

	hci->target_cb = cfg;

	dyn_addr = mipi_i3c_hci_target_get_dynamic_addr(hci);
	if (dyn_addr != 0U) {
		cfg->address = dyn_addr;
	} else {
		props = mipi_i3c_hci_target_get_dt_props(hci);
		if (props && props->assigned_addr != 0U) {
			cfg->address = props->assigned_addr;
		}
	}

	k_mutex_unlock(&hci->control_mutex);

	return 0;
}

int mipi_i3c_hci_target_unregister(const struct device *dev, struct i3c_target_config *cfg)
{
	struct i3c_hci *hci = dev->data;

	if (!cfg) {
		return -EINVAL;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);

	if (hci->target_cb == cfg) {
		hci->target_cb = NULL;
	}

	k_mutex_unlock(&hci->control_mutex);

	return 0;
}

int mipi_i3c_hci_target_tx_write(const struct device *dev, uint8_t *buf, uint16_t len)
{
	struct i3c_hci *hci = dev->data;
	int ret;

	if (!buf && len != 0U) {
		return -EINVAL;
	}

	if (!hci->is_target) {
		return -ENOTSUP;
	}

	if (len == 0U) {
		return 0;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);
	ret = mipi_i3c_hci_target_submit_read_data(hci, buf, len);
	k_mutex_unlock(&hci->control_mutex);

	return ret == 0 ? len : ret;
}

int mipi_i3c_hci_target_ibi_raise(const struct device *dev, struct i3c_ibi *request)
{
	struct i3c_hci *hci = dev->data;
	int ret;

	if (!request) {
		LOG_ERR("%s ibi_raise: NULL request", dev->name);
		return -EINVAL;
	}

	LOG_DBG("%s ibi_raise: type=%d is_target=%d is_secondary=%d", dev->name,
		request->ibi_type, hci->is_target, hci->is_secondary);

	if (!hci->is_target) {
		LOG_ERR("%s ibi_raise: not configured as target", dev->name);
		return -ENOTSUP;
	}

	if (request->payload_len != 0U && !request->payload) {
		LOG_ERR("%s ibi_raise: payload_len=%u but payload is NULL", dev->name,
			request->payload_len);
		return -EINVAL;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);

	switch (request->ibi_type) {
	case I3C_IBI_TARGET_INTR:
		ret = mipi_i3c_hci_target_raise_sir(hci, request);
		break;
	case I3C_IBI_HOTJOIN:
		ret = mipi_i3c_hci_target_raise_hotjoin(hci);
		break;
	case I3C_IBI_CONTROLLER_ROLE_REQUEST:
		ret = mipi_i3c_hci_target_raise_mastership(hci);
		break;
	default:
		LOG_ERR("%s ibi_raise: unknown ibi_type %d", dev->name, request->ibi_type);
		ret = -EINVAL;
		break;
	}

	k_mutex_unlock(&hci->control_mutex);

	LOG_DBG("%s ibi_raise: type=%d ret=%d", dev->name, request->ibi_type, ret);

	return ret;
}

int mipi_i3c_hci_target_ibi_enable(const struct device *dev, struct i3c_device_desc *target)
{
	struct i3c_hci *hci = dev->data;
	struct i3c_ibi request = {
		.ibi_type = I3C_IBI_TARGET_INTR,
	};
	struct i3c_ccc_events events = {
		.events = I3C_CCC_EVT_INTR,
	};
	int ret;

	if (!target) {
		return -EINVAL;
	}

	request.payload_len = target->data_length.max_ibi;
	if (request.payload_len != 0U) {
		if (hci->vendor && hci->vendor->set_ibi_terminate_len) {
			hci->vendor->set_ibi_terminate_len(hci, request.payload_len);
		}
	}

	if (!hci->io || !hci->io->request_ibi || !hci->io->free_ibi) {
		return -ENOSYS;
	}

	ret = hci->io->request_ibi(hci, target, &request);
	if (ret != 0) {
		return ret;
	}

	/*
	 * Tell the target via ENEC CCC that it may now raise IBIs. Without
	 * this, the target's SLV_STS1_IBI_EN bit stays clear and the target
	 * HCI silently drops every IBI_REQ instead of driving SDA on the bus.
	 */
	ret = i3c_ccc_do_events_set(target, true, &events);
	if (ret != 0) {
		LOG_ERR("%s ENEC ENINTR to 0x%02x failed: %d",
			dev->name, target->dynamic_addr, ret);
		hci->io->free_ibi(hci, target);
		return ret;
	}

	return 0;
}

int mipi_i3c_hci_target_ibi_disable(const struct device *dev, struct i3c_device_desc *target)
{
	struct i3c_hci *hci = dev->data;
	struct i3c_ccc_events events = {
		.events = I3C_CCC_EVT_INTR,
	};
	int ret;

	if (!target) {
		return -EINVAL;
	}

	if (!hci->io || !hci->io->free_ibi) {
		return -ENOSYS;
	}

	if (target->dynamic_addr != 0U) {
		ret = i3c_ccc_do_events_set(target, false, &events);
		if (ret != 0) {
			LOG_WRN("%s DISEC INTR to 0x%02x failed: %d",
				dev->name, target->dynamic_addr, ret);
		}
	}

	hci->io->free_ibi(hci, target);

	return 0;
}

int mipi_i3c_hci_target_pending_read_notify(const struct device *dev, uint8_t *buf,
					    uint16_t len, struct i3c_ibi *notifier)
{
	struct i3c_hci *hci = dev->data;
	struct hci_xfer *ibi_xfer = NULL;
	struct hci_xfer *read_xfer = NULL;
	bool ibi_queued = false;
	bool read_queued = false;
	bool ibi_completed = false;
	bool read_completed = false;
	int ret;

	if (!notifier || notifier->ibi_type != I3C_IBI_TARGET_INTR) {
		return -EINVAL;
	}

	if (!buf && len != 0U) {
		return -EINVAL;
	}

	if (!hci->is_target) {
		return -ENOTSUP;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);

	if (hci->vendor && hci->vendor->set_ibi_terminate_len) {
		hci->vendor->set_ibi_terminate_len(hci, notifier->payload_len);
	}

	ibi_xfer = hci_alloc_xfer(1);
	if (!ibi_xfer) {
		ret = -ENOMEM;
		goto out;
	}

	ret = mipi_i3c_hci_target_prep_ibi_data(ibi_xfer, notifier);
	if (ret != 0) {
		goto out;
	}

	ret = mipi_i3c_hci_target_queue_xfer(hci, ibi_xfer);
	if (ret != 0) {
		goto out;
	}
	ibi_queued = true;

	if (len != 0U) {
		read_xfer = hci_alloc_xfer(1);
		if (!read_xfer) {
			ret = -ENOMEM;
			goto out;
		}

		k_sem_reset(&hci->pending_r_comp);
		mipi_i3c_hci_target_prep_read_data(read_xfer, buf, len);

		ret = mipi_i3c_hci_target_queue_xfer(hci, read_xfer);
		if (ret != 0) {
			goto out;
		}
		read_queued = true;
	}

	/*
	 * Fire the IBI and wait for completion of both the IBI itself
	 * (tid=TID_TARGET_IBI response → ibi_comp) and the pending read
	 * (tid=TID_TARGET_RD_DATA response → pending_r_comp). Matches the
	 * Linux ast2700 driver, where generate_ibi() internally waits on
	 * ibi_comp before pending_read_notify() waits on pending_r_comp.
	 */
	k_sem_reset(&hci->ibi_comp);
	ret = mipi_i3c_hci_target_set_event(hci,
					    MIPI_I3C_HCI_TARGET_EVENT_IBI);
	if (ret == 0) {
		ret = mipi_i3c_hci_target_wait(&hci->ibi_comp);
		ibi_completed = ret == 0;
	}
	if (ret == 0 && len != 0U) {
		ret = mipi_i3c_hci_target_wait(&hci->pending_r_comp);
		read_completed = ret == 0;
	}

out:
	mipi_i3c_hci_target_release_xfer(hci, &read_xfer, read_queued,
					 read_completed, &hci->pending_r_comp);
	mipi_i3c_hci_target_release_xfer(hci, &ibi_xfer, ibi_queued,
					 ibi_completed, &hci->ibi_comp);
	k_mutex_unlock(&hci->control_mutex);

	return ret;
}

void mipi_i3c_hci_target_rx_data(struct i3c_hci *hci, void *buf, unsigned int len)
{
	const struct i3c_target_callbacks *cbs;
	uint8_t *bytes = buf;
	int ret = 0;

	if (!hci || !hci->target_cb || !bytes || len == 0U) {
		return;
	}

	cbs = hci->target_cb->callbacks;
	if (!cbs) {
		return;
	}

	if (cbs->write_requested_cb) {
		ret = cbs->write_requested_cb(hci->target_cb);
	}

	if (ret == 0 && cbs->write_received_cb) {
		for (unsigned int i = 0; i < len; i++) {
			ret = cbs->write_received_cb(hci->target_cb, bytes[i]);
			if (ret != 0) {
				break;
			}
		}
	}

	if (cbs->stop_cb) {
		(void)cbs->stop_cb(hci->target_cb);
	}
}

void mipi_i3c_hci_target_dyn_addr_updated(struct i3c_hci *hci, uint8_t new_addr)
{
	if (!hci) {
		return;
	}

	if (new_addr == 0U) {
		new_addr = mipi_i3c_hci_target_get_dynamic_addr(hci);
	}

	if (new_addr == 0U) {
		return;
	}

	if (hci->target_cb) {
		hci->target_cb->address = new_addr;
	}

	LOG_DBG("%s target dynamic address updated to 0x%02x", hci->dev->name, new_addr);
}

void mipi_i3c_hci_target_handle_defslvs(struct i3c_hci *hci, const void *payload,
					size_t payload_len)
{
	const uint8_t *bytes = payload;

	if (!hci || !bytes ||
	    payload_len < sizeof(struct i3c_ccc_deftgts_active_controller)) {
		return;
	}

	hci->is_secondary = true;
	hci->common.ctrl_config.is_secondary = true;

	if (bytes[0] != 0U) {
		i3c_addr_slots_mark_i3c(&hci->common.attached_dev.addr_slots, bytes[0]);
	}

	LOG_DBG("%s received DEFTGTS active controller 0x%02x", hci->dev->name, bytes[0]);
}

void mipi_i3c_hci_target_role_updated(struct i3c_hci *hci)
{
	bool secondary;
	bool target;

	if (!hci || !hci->vendor || !hci->vendor->target_get_role) {
		return;
	}

	if (!hci->vendor->target_get_role(hci, &secondary, &target)) {
		return;
	}

	hci->is_secondary = secondary;
	hci->is_target = target;
	hci->common.ctrl_config.is_secondary = secondary;

	mipi_i3c_hci_target_update_registered_addr(hci);

	LOG_DBG("%s role target=%d secondary=%d", hci->dev->name, hci->is_target,
		hci->is_secondary);
}
