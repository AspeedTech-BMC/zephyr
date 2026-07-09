/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Ported to Zephyr from the Linux mipi-i3c-hci driver
 * (drivers/i3c/master/mipi-i3c-hci/).
 */

#define DT_DRV_COMPAT mipi_i3c_hci

#include <errno.h>
#include <limits.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "cmd.h"
#include "dat.h"
#include "ext_caps.h"
#include "hci.h"

LOG_MODULE_REGISTER(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

/*
 * Vendor ops dispatch. The driver registers under both the generic
 * "mipi,i3c-hci" compatible and any vendor-specific compatibles it
 * understands (currently "aspeed,g7-i3c-hci"). Each DT node is bound
 * to the binding YAML of the first compatible in its `compatible`
 * property that has a YAML — vendor-specific compatibles come first
 * so the node picks up the vendor binding's extra properties, and the
 * driver still sees the node through the matching DT_FOREACH_STATUS_OKAY
 * iteration below.
 *
 * Per-node vendor ops are picked at build time from the node's
 * compatible list. Add a `MIPI_I3C_HCI_PICK_*` clause and an extra
 * `DT_FOREACH_STATUS_OKAY` iteration when porting a new silicon
 * family; the dispatch happens entirely at compile time so unselected
 * vendor symbols are never referenced.
 */
#if defined(CONFIG_I3C_MIPI_HCI_ASPEED_VENDOR)
extern const struct mipi_i3c_hci_vendor_ops mipi_i3c_hci_aspeed_ops;
#define MIPI_I3C_HCI_PICK_ASPEED(node_id)                                                     \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, aspeed_g7_i3c_hci),                            \
		    (&mipi_i3c_hci_aspeed_ops), (NULL))
#else
#define MIPI_I3C_HCI_PICK_ASPEED(node_id) NULL
#endif

#define MIPI_I3C_HCI_PICK_VENDOR(node_id) MIPI_I3C_HCI_PICK_ASPEED(node_id)

#define MIPI_I3C_HCI_XFER_TIMEOUT_MS 1000
#define MIPI_I3C_HCI_RESET_TIMEOUT_US 10000
#define MIPI_I3C_HCI_CORE_IRQS \
	(INTR_HC_CMD_SEQ_UFLOW_STAT | INTR_HC_SEQ_CANCEL | INTR_HC_INTERNAL_ERR)

static struct i3c_config_controller *mipi_i3c_hci_master_get_bus(struct i3c_hci *hci)
{
	return &hci->common.ctrl_config;
}

static struct i3c_hci_dev_data *
mipi_i3c_hci_master_get_i3c_dev(struct i3c_device_desc *target)
{
	return target->controller_priv;
}

static struct i3c_hci_dev_data *mipi_i3c_hci_dev_data_alloc(void)
{
	struct i3c_hci_dev_data *dev_data;

	dev_data = k_calloc(1, sizeof(*dev_data));
	if (dev_data) {
		dev_data->dat_idx = -1;
	}

	return dev_data;
}

static void mipi_i3c_hci_dev_data_free(struct i3c_hci_dev_data *dev_data)
{
	k_free(dev_data);
}

static uint8_t mipi_i3c_hci_supported_hdr(struct i3c_hci *hci)
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

static int mipi_i3c_hci_response_to_errno(uint32_t response)
{
	switch (RESP_STATUS(response)) {
	case RESP_SUCCESS:
		return 0;
	case RESP_ERR_NACK:
	case RESP_ERR_ADDR_HEADER:
		return -ENXIO;
	case RESP_ERR_NOT_SUPPORTED:
		return -ENOTSUP;
	case RESP_ERR_HC_TERMINATED:
	case RESP_ERR_BUS_XFER_ABORTED:
		return -ECANCELED;
	default:
		return -EIO;
	}
}

static int mipi_i3c_hci_check_data_len(struct i3c_hci *hci, uint32_t len)
{
	uint32_t size_limit = 1U << (16 + FIELD_GET(HC_CAP_MAX_DATA_LENGTH, hci->caps));

	if (len >= size_limit) {
		return -EFBIG;
	}

	if (hci->vendor && hci->vendor->payload_too_big &&
	    hci->vendor->payload_too_big(len)) {
		return -EFBIG;
	}

	return 0;
}

static int mipi_i3c_hci_queue_xfers(struct i3c_hci *hci, struct hci_xfer *xfer, int n)
{
	struct k_sem done;
	int ret;

	if (!hci->io || !hci->io->queue_xfer || n <= 0) {
		return -EINVAL;
	}

	k_sem_init(&done, 0, 1);
	xfer[n - 1].cmd_desc[0] |= CMD_0_TOC;
	xfer[n - 1].completion = &done;

	ret = hci->io->queue_xfer(hci, xfer, n);
	if (ret != 0) {
		return ret;
	}

	ret = k_sem_take(&done, K_MSEC(MIPI_I3C_HCI_XFER_TIMEOUT_MS));
	if (ret != 0) {
		if (hci->io->dequeue_xfer) {
			(void)hci->io->dequeue_xfer(hci, xfer, n);
		}
		return -ETIMEDOUT;
	}

	return 0;
}

static void mipi_i3c_hci_iba_ctrl(struct i3c_hci *hci, bool enable)
{
	if (enable) {
		hci_reg_set(hci, HC_CONTROL, HC_CONTROL_IBA_INCLUDE);
	} else {
		hci_reg_clear(hci, HC_CONTROL, HC_CONTROL_IBA_INCLUDE);
	}
}

static int mipi_i3c_hci_reset_toggle(const struct reset_dt_spec *rst, const char *name)
{
	int ret;

	if (!rst->dev) {
		return 0;
	}

	if (!device_is_ready(rst->dev)) {
		LOG_ERR("%s reset controller is not ready", name);
		return -ENODEV;
	}

	ret = reset_line_toggle_dt(rst);
	if (ret != 0) {
		LOG_ERR("failed to toggle %s reset: %d", name, ret);
	}

	return ret;
}

static int mipi_i3c_hci_soft_reset(struct i3c_hci *hci)
{
	if (!WAIT_FOR(!(hci_reg_read(hci, RESET_CONTROL) & SOFT_RST),
		      MIPI_I3C_HCI_RESET_TIMEOUT_US, k_busy_wait(1))) {
		return -ENXIO;
	}

	hci_reg_write(hci, RESET_CONTROL, SOFT_RST);

	if (!WAIT_FOR(!(hci_reg_read(hci, RESET_CONTROL) & SOFT_RST),
		      MIPI_I3C_HCI_RESET_TIMEOUT_US, k_busy_wait(1))) {
		return -ENXIO;
	}

	return 0;
}

static int mipi_i3c_hci_detect_version(struct i3c_hci *hci)
{
	uint32_t regval = hci_reg_read(hci, HCI_VERSION);

	hci->version_major = (regval >> 8) & 0xf;
	hci->version_minor = (regval >> 4) & 0xf;
	hci->revision = regval & 0xf;

	LOG_INF("MIPI I3C HCI v%u.%u r%02u", hci->version_major, hci->version_minor,
		hci->revision);

	switch (regval & ~0xfU) {
	case 0x100:
	case 0x110:
	case 0x200:
		return 0;
	default:
		LOG_ERR("unsupported HCI version register %#x", regval);
		return -EPROTONOSUPPORT;
	}
}

static int mipi_i3c_hci_discover_sections(struct i3c_hci *hci)
{
	bool size_in_dwords;
	uint32_t regval;
	uint32_t offset;
	int ret;

	hci->caps = hci_reg_read(hci, HC_CAPABILITIES);
	LOG_DBG("HC_CAPABILITIES %#x", hci->caps);

	size_in_dwords = (hci->version_major < 1) ||
			 ((hci->version_major == 1) && (hci->version_minor < 1));

	regval = hci_reg_read(hci, DAT_SECTION);
	offset = FIELD_GET(DAT_TABLE_OFFSET, regval);
	hci->DAT_regs = offset ? hci->base_regs + offset : 0;
	hci->DAT_entries = FIELD_GET(DAT_TABLE_SIZE, regval);
	hci->DAT_entry_size = FIELD_GET(DAT_ENTRY_SIZE, regval) ? 0 : 8;
	if (hci->DAT_entry_size == 0) {
		return -EINVAL;
	}
	if (size_in_dwords) {
		hci->DAT_entries = 4U * hci->DAT_entries / hci->DAT_entry_size;
	}
	LOG_DBG("DAT: %u %u-byte entries at offset %#x", hci->DAT_entries,
		hci->DAT_entry_size, offset);

	regval = hci_reg_read(hci, DCT_SECTION);
	offset = FIELD_GET(DCT_TABLE_OFFSET, regval);
	hci->DCT_regs = offset ? hci->base_regs + offset : 0;
	hci->DCT_entries = FIELD_GET(DCT_TABLE_SIZE, regval);
	hci->DCT_entry_size = FIELD_GET(DCT_ENTRY_SIZE, regval) ? 0 : 16;
	if (hci->DCT_entry_size == 0) {
		return -EINVAL;
	}
	if (size_in_dwords) {
		hci->DCT_entries = 4U * hci->DCT_entries / hci->DCT_entry_size;
	}
	LOG_DBG("DCT: %u %u-byte entries at offset %#x", hci->DCT_entries,
		hci->DCT_entry_size, offset);

	regval = hci_reg_read(hci, RING_HEADERS_SECTION);
	offset = FIELD_GET(RING_HEADERS_OFFSET, regval);
	hci->RHS_regs = offset ? hci->base_regs + offset : 0;
	LOG_DBG("Ring Headers at offset %#x", offset);

	regval = hci_reg_read(hci, PIO_SECTION);
	offset = FIELD_GET(PIO_REGS_OFFSET, regval);
	hci->PIO_regs = offset ? hci->base_regs + offset : 0;
	LOG_DBG("PIO section at offset %#x", offset);

	regval = hci_reg_read(hci, EXT_CAPS_SECTION);
	offset = FIELD_GET(EXT_CAPS_OFFSET, regval);
	hci->EXTCAPS_regs = offset ? hci->base_regs + offset : 0;
	LOG_DBG("Extended Caps at offset %#x", offset);

	ret = i3c_hci_parse_ext_caps(hci);
	if (ret != 0) {
		return ret;
	}

	mipi_i3c_hci_apply_quirks(hci);

	return 0;
}

static int mipi_i3c_hci_setup_endianness(struct i3c_hci *hci)
{
	uint32_t regval = hci_reg_read(hci, HC_CONTROL);

	if (IS_ENABLED(CONFIG_BIG_ENDIAN)) {
		if ((regval & HC_CONTROL_DATA_BIG_ENDIAN) == 0) {
			hci_reg_write(hci, HC_CONTROL, regval | HC_CONTROL_DATA_BIG_ENDIAN);
			if ((hci_reg_read(hci, HC_CONTROL) & HC_CONTROL_DATA_BIG_ENDIAN) == 0) {
				return -EOPNOTSUPP;
			}
		}
	} else if (regval & HC_CONTROL_DATA_BIG_ENDIAN) {
		hci_reg_write(hci, HC_CONTROL, regval & ~HC_CONTROL_DATA_BIG_ENDIAN);
		if (hci_reg_read(hci, HC_CONTROL) & HC_CONTROL_DATA_BIG_ENDIAN) {
			return -EOPNOTSUPP;
		}
	}

	return 0;
}

static int mipi_i3c_hci_select_cmd_ops(struct i3c_hci *hci)
{
	switch (FIELD_GET(HC_CAP_CMD_SIZE, hci->caps)) {
	case 0:
		hci->cmd = &mipi_i3c_hci_cmd_v1;
		return 0;
	case 1:
		hci->cmd = &mipi_i3c_hci_cmd_v2;
		return 0;
	default:
		LOG_ERR("unsupported CMD_SIZE capability value");
		return -EINVAL;
	}
}

static int mipi_i3c_hci_select_pio(struct i3c_hci *hci, bool mode_selector)
{
#if defined(CONFIG_I3C_MIPI_HCI_PIO)
	if (hci->PIO_regs == 0) {
		return -ENODEV;
	}

	hci_reg_set(hci, HC_CONTROL, HC_CONTROL_PIO_MODE);
	if (mode_selector && ((hci_reg_read(hci, HC_CONTROL) & HC_CONTROL_PIO_MODE) == 0)) {
		return -EIO;
	}

	hci->io = &mipi_i3c_hci_pio;
	LOG_INF("Using HCI PIO transfer mode");
	return 0;
#else
	ARG_UNUSED(hci);
	ARG_UNUSED(mode_selector);
	return -ENOTSUP;
#endif
}

static int mipi_i3c_hci_select_dma(struct i3c_hci *hci, bool mode_selector)
{
#if defined(CONFIG_I3C_MIPI_HCI_DMA)
	int ret;

	if (hci->RHS_regs == 0) {
		return -ENODEV;
	}

	if (!hci->dma_rst.dev) {
		return -ENODEV;
	}

	hci_reg_clear(hci, HC_CONTROL, HC_CONTROL_PIO_MODE);
	if (mode_selector && (hci_reg_read(hci, HC_CONTROL) & HC_CONTROL_PIO_MODE)) {
		return -EIO;
	}

	ret = mipi_i3c_hci_reset_toggle(&hci->dma_rst, "dma");
	if (ret != 0) {
		return ret;
	}

	hci->io = &mipi_i3c_hci_dma;
	LOG_INF("Using HCI DMA transfer mode");
	return 0;
#else
	ARG_UNUSED(hci);
	ARG_UNUSED(mode_selector);
	return -ENOTSUP;
#endif
}

static int mipi_i3c_hci_select_io_ops(struct i3c_hci *hci)
{
	const char *mode = hci->config->io_mode ? hci->config->io_mode : "pio";
	bool mode_selector;

	mode_selector = (hci->version_major > 1) ||
			((hci->version_major == 1) && (hci->version_minor > 0));

	if (hci->quirks & HCI_QUIRK_PIO_MODE) {
		hci->RHS_regs = 0;
		return mipi_i3c_hci_select_pio(hci, mode_selector);
	}

	if (strcmp(mode, "dma") == 0) {
		return mipi_i3c_hci_select_dma(hci, mode_selector);
	}

	if (strcmp(mode, "pio") == 0) {
		return mipi_i3c_hci_select_pio(hci, mode_selector);
	}

	return -EINVAL;
}

static void mipi_i3c_hci_setup_interrupts(struct i3c_hci *hci)
{
	hci_reg_write(hci, INTR_SIGNAL_ENABLE, 0);
	hci_reg_write(hci, INTR_STATUS_ENABLE, GENMASK(31, 10));

	if (hci->vendor && hci->vendor->disable_priv_irq) {
		hci->vendor->disable_priv_irq(hci);
	}
}

static int mipi_i3c_hci_hw_init(struct i3c_hci *hci)
{
	int ret;

	ret = mipi_i3c_hci_detect_version(hci);
	if (ret != 0) {
		return ret;
	}

	ret = mipi_i3c_hci_discover_sections(hci);
	if (ret != 0) {
		return ret;
	}

	ret = mipi_i3c_hci_soft_reset(hci);
	if (ret != 0) {
		return ret;
	}

	mipi_i3c_hci_setup_interrupts(hci);

	ret = mipi_i3c_hci_setup_endianness(hci);
	if (ret != 0) {
		return ret;
	}

	ret = mipi_i3c_hci_select_cmd_ops(hci);
	if (ret != 0) {
		return ret;
	}

	ret = mipi_i3c_hci_select_io_ops(hci);
	if (ret != 0) {
		return ret;
	}

	if (hci->quirks & HCI_QUIRK_OD_PP_TIMING) {
		amd_set_od_pp_timing(hci);
	}

	return 0;
}

static int mipi_i3c_hci_dat_init(struct i3c_hci *hci)
{
	if (hci->cmd == &mipi_i3c_hci_cmd_v1 && hci->dat &&
	    hci->dat->init) {
		return hci->dat->init(hci);
	}

	return 0;
}

static int mipi_i3c_hci_program_master_addr(struct i3c_hci *hci)
{
	uint8_t addr;

	addr = i3c_addr_slots_next_free_find(&hci->common.attached_dev.addr_slots, 0);
	if (addr == 0) {
		return -ENOSPC;
	}

	hci_reg_write(hci, MASTER_DEVICE_ADDR,
		      MASTER_DYNAMIC_ADDR(addr) | MASTER_DYNAMIC_ADDR_VALID);
	i3c_addr_slots_mark_i3c(&hci->common.attached_dev.addr_slots, addr);

	return 0;
}

static int mipi_i3c_hci_configure(const struct device *dev,
				  enum i3c_config_type type, void *config)
{
	struct i3c_hci *hci = dev->data;
	struct i3c_config_controller *bus_config;
	struct i3c_config_controller *new_config = config;

	if (!new_config) {
		return -EINVAL;
	}

	if (type != I3C_CONFIG_CONTROLLER) {
		return -ENOTSUP;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);
	bus_config = mipi_i3c_hci_master_get_bus(hci);
	*bus_config = *new_config;
	bus_config->supported_hdr &= mipi_i3c_hci_supported_hdr(hci);
	k_mutex_unlock(&hci->control_mutex);

	return 0;
}

static int mipi_i3c_hci_config_get(const struct device *dev,
				   enum i3c_config_type type, void *config)
{
	struct i3c_hci *hci = dev->data;

	if (type != I3C_CONFIG_CONTROLLER || !config) {
		return -EINVAL;
	}

	*(struct i3c_config_controller *)config = hci->common.ctrl_config;
	return 0;
}

static int mipi_i3c_hci_abort_reset(struct i3c_hci *hci)
{
	if (hci->io && hci->io->dequeue_xfer) {
		(void)hci->io->dequeue_xfer(hci, NULL, 0);
	}

	hci_reg_set(hci, HC_CONTROL, HC_CONTROL_ABORT);
	if (!WAIT_FOR(!(hci_reg_read(hci, HC_CONTROL) & HC_CONTROL_ABORT),
		      MIPI_I3C_HCI_RESET_TIMEOUT_US, k_busy_wait(1))) {
		mipi_i3c_hci_pio_reset(hci);
		mipi_i3c_hci_resume(hci);
		return -ETIMEDOUT;
	}

	mipi_i3c_hci_pio_reset(hci);
	mipi_i3c_hci_resume(hci);

	return 0;
}

static int mipi_i3c_hci_recover_bus(const struct device *dev)
{
	struct i3c_hci *hci = dev->data;
	int ret;

	k_mutex_lock(&hci->control_mutex, K_FOREVER);
	ret = mipi_i3c_hci_abort_reset(hci);
	k_mutex_unlock(&hci->control_mutex);

	return ret;
}

static int mipi_i3c_hci_attach_i3c_device(const struct device *dev,
					  struct i3c_device_desc *target,
					  uint8_t addr)
{
	struct i3c_hci *hci = dev->data;
	struct i3c_hci_dev_data *dev_data;
	int ret;

	dev_data = mipi_i3c_hci_dev_data_alloc();
	if (!dev_data) {
		return -ENOMEM;
	}

	if (hci->cmd == &mipi_i3c_hci_cmd_v1 && hci->dat &&
	    hci->dat->alloc_entry) {
		int preferred_slot = -1;

		if (hci->vendor && hci->vendor->dat_pick_slot) {
			preferred_slot = hci->vendor->dat_pick_slot(hci, addr);
		}

		ret = hci->dat->alloc_entry(hci, preferred_slot);
		if (ret < 0) {
			mipi_i3c_hci_dev_data_free(dev_data);
			return ret;
		}

		dev_data->dat_idx = ret;
		if (hci->dat->set_dynamic_addr) {
			hci->dat->set_dynamic_addr(hci, dev_data->dat_idx, addr);
		}
	}

	target->controller_priv = dev_data;

	return 0;
}

static int mipi_i3c_hci_reattach_i3c_device(const struct device *dev,
					    struct i3c_device_desc *target,
					    uint8_t old_dyn_addr)
{
	struct i3c_hci *hci = dev->data;
	struct i3c_hci_dev_data *dev_data = mipi_i3c_hci_master_get_i3c_dev(target);

	ARG_UNUSED(old_dyn_addr);

	if (!dev_data) {
		return -EINVAL;
	}

	if (hci->dat && hci->dat->set_dynamic_addr &&
	    dev_data->dat_idx >= 0) {
		hci->dat->set_dynamic_addr(hci, dev_data->dat_idx, target->dynamic_addr);
		/* Sync cached slot index after addr-indexed relocation. */
		if (hci->vendor && hci->vendor->dat_wants_addr_indexed &&
		    hci->vendor->dat_wants_addr_indexed(hci)) {
			dev_data->dat_idx = (int)target->dynamic_addr;
		}
	}

	return 0;
}

static int mipi_i3c_hci_detach_i3c_device(const struct device *dev,
					  struct i3c_device_desc *target)
{
	struct i3c_hci *hci = dev->data;
	struct i3c_hci_dev_data *dev_data = target->controller_priv;

	target->controller_priv = NULL;

	if (!dev_data) {
		return 0;
	}

	if (hci->dat && hci->dat->free_entry &&
	    dev_data->dat_idx >= 0) {
		hci->dat->free_entry(hci, dev_data->dat_idx);
	}

	mipi_i3c_hci_dev_data_free(dev_data);
	return 0;
}

static int mipi_i3c_hci_attach_i2c_device(const struct device *dev,
					  struct i3c_i2c_device_desc *target)
{
	struct i3c_hci *hci = dev->data;
	struct i3c_hci_dev_data *dev_data;
	int ret;

	dev_data = mipi_i3c_hci_dev_data_alloc();
	if (!dev_data) {
		return -ENOMEM;
	}

	if (hci->cmd == &mipi_i3c_hci_cmd_v1 && hci->dat &&
	    hci->dat->alloc_entry) {
		int preferred_slot = -1;

		if (hci->vendor && hci->vendor->dat_pick_slot) {
			preferred_slot = hci->vendor->dat_pick_slot(hci, target->addr);
		}

		ret = hci->dat->alloc_entry(hci, preferred_slot);
		if (ret < 0) {
			mipi_i3c_hci_dev_data_free(dev_data);
			return ret;
		}

		dev_data->dat_idx = ret;
		if (hci->dat->set_static_addr) {
			hci->dat->set_static_addr(hci, dev_data->dat_idx, target->addr);
		}
		if (hci->dat->set_flags) {
			hci->dat->set_flags(hci, dev_data->dat_idx, DAT_0_I2C_DEVICE, 0);
		}
	}

	target->controller_priv = dev_data;

	return 0;
}

static int mipi_i3c_hci_detach_i2c_device(const struct device *dev,
					  struct i3c_i2c_device_desc *target)
{
	struct i3c_hci *hci = dev->data;
	struct i3c_hci_dev_data *dev_data = target->controller_priv;

	target->controller_priv = NULL;

	if (!dev_data) {
		return 0;
	}

	if (hci->dat && hci->dat->free_entry &&
	    dev_data->dat_idx >= 0) {
		hci->dat->free_entry(hci, dev_data->dat_idx);
	}

	mipi_i3c_hci_dev_data_free(dev_data);
	return 0;
}

static int mipi_i3c_hci_do_daa(const struct device *dev)
{
	struct i3c_hci *hci = dev->data;
	int ret;

	if (!hci->cmd || !hci->cmd->perform_daa) {
		return -ENOSYS;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);
	ret = hci->cmd->perform_daa(hci);
	k_mutex_unlock(&hci->control_mutex);

	return ret;
}

static int mipi_i3c_hci_do_ccc(const struct device *dev, struct i3c_ccc_payload *payload)
{
	struct i3c_hci *hci = dev->data;
	struct hci_xfer *xfer;
	bool broadcast;
	bool raw;
	bool prefixed;
	bool dbp = false;
	uint8_t db = 0;
	size_t nxfers;
	size_t xfer_idx = 0;
	int ret = 0;

	if (!payload || !hci->cmd || !hci->cmd->prep_ccc) {
		return -EINVAL;
	}

	broadcast = i3c_ccc_is_payload_broadcast(payload);
	if (payload->ccc.data_len != 0U && !payload->ccc.data) {
		return -EINVAL;
	}

	if (!broadcast) {
		if (!payload->targets.payloads || payload->targets.num_targets == 0U) {
			return -EINVAL;
		}

		if (payload->ccc.data_len > 1U) {
			return -EINVAL;
		}

		if (payload->ccc.data_len == 1U) {
			dbp = true;
			db = payload->ccc.data[0];
		}
	}

	raw = (hci->quirks & HCI_QUIRK_RAW_CCC) != 0U;
	prefixed = raw && !broadcast;
	nxfers = broadcast ? 1U : payload->targets.num_targets + (prefixed ? 1U : 0U);

	if (nxfers > INT_MAX) {
		return -EINVAL;
	}

	xfer = hci_alloc_xfer(nxfers);
	if (!xfer) {
		return -ENOMEM;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);

	if (prefixed) {
		ret = hci->cmd->prep_ccc(hci, &xfer[xfer_idx], I3C_BROADCAST_ADDR,
					 payload->ccc.id, dbp, db, true);
		if (ret != 0) {
			goto out;
		}
		xfer_idx++;
	}

	if (broadcast) {
		ret = mipi_i3c_hci_check_data_len(hci, payload->ccc.data_len);
		if (ret != 0) {
			goto out;
		}

		xfer[0].data = payload->ccc.data;
		xfer[0].data_len = payload->ccc.data_len;
		xfer[0].rnw = false;
		ret = hci->cmd->prep_ccc(hci, &xfer[0], I3C_BROADCAST_ADDR,
					 payload->ccc.id, false, 0, raw);
		if (ret != 0) {
			goto out;
		}
		xfer[0].cmd_desc[0] |= CMD_0_ROC;
	} else {
		for (size_t i = 0; i < payload->targets.num_targets; i++, xfer_idx++) {
			struct i3c_ccc_target_payload *target = &payload->targets.payloads[i];

			if (target->data_len != 0U && !target->data) {
				ret = -EINVAL;
				goto out;
			}

			ret = mipi_i3c_hci_check_data_len(hci, target->data_len);
			if (ret != 0) {
				goto out;
			}

			xfer[xfer_idx].data = target->data;
			xfer[xfer_idx].data_len = target->data_len;
			xfer[xfer_idx].rnw = target->rnw;
			ret = hci->cmd->prep_ccc(hci, &xfer[xfer_idx], target->addr,
						 payload->ccc.id, dbp, db, raw);
			if (ret != 0) {
				goto out;
			}
			xfer[xfer_idx].cmd_desc[0] |= CMD_0_ROC;
		}
	}

	ret = mipi_i3c_hci_queue_xfers(hci, xfer, (int)nxfers);
	if (ret != 0) {
		goto out;
	}

	if (broadcast) {
		ret = mipi_i3c_hci_response_to_errno(xfer[0].response);
		if (ret == 0) {
			payload->ccc.num_xfer = payload->ccc.data_len;
		}
		goto out;
	}

	for (size_t i = 0; i < payload->targets.num_targets; i++) {
		size_t idx = i + (prefixed ? 1U : 0U);
		struct i3c_ccc_target_payload *target = &payload->targets.payloads[i];

		ret = mipi_i3c_hci_response_to_errno(xfer[idx].response);
		if (ret != 0) {
			LOG_ERR("CCC 0x%02x target 0x%02x response %#x", payload->ccc.id,
				target->addr, xfer[idx].response);
			goto out;
		}

		target->num_xfer = target->rnw ? RESP_DATA_LENGTH(xfer[idx].response) :
						 target->data_len;
	}

out:
	k_mutex_unlock(&hci->control_mutex);
	hci_free_xfer(xfer, nxfers);
	return ret;
}

static int mipi_i3c_hci_i3c_xfers(const struct device *dev,
				  struct i3c_device_desc *target,
				  struct i3c_msg *msgs, uint8_t num_msgs)
{
	struct i3c_hci *hci = dev->data;
	struct hci_xfer *xfer;
	int ret = 0;

	if (num_msgs == 0U) {
		return 0;
	}

	if (!target || !msgs || target->dynamic_addr == 0U) {
		return -EINVAL;
	}

	if (!mipi_i3c_hci_master_get_i3c_dev(target)) {
		return -ENODEV;
	}

	xfer = hci_alloc_xfer(num_msgs);
	if (!xfer) {
		return -ENOMEM;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);

	for (uint8_t i = 0; i < num_msgs; i++) {
		bool hdr = (msgs[i].flags & I3C_MSG_HDR) != 0U;

		ret = mipi_i3c_hci_check_data_len(hci, msgs[i].len);
		if (ret != 0) {
			goto out;
		}

		if (hdr) {
			if (msgs[i].hdr_mode == 0U || (POPCOUNT(msgs[i].hdr_mode) != 1)) {
				ret = -EINVAL;
				goto out;
			}

			if ((hci->common.ctrl_config.supported_hdr & msgs[i].hdr_mode) == 0U) {
				ret = -ENOTSUP;
				goto out;
			}

			if (!hci->cmd || !hci->cmd->prep_hdr) {
				ret = -ENOSYS;
				goto out;
			}

			if (msgs[i].hdr_mode == I3C_MSG_HDR_DDR && ((msgs[i].len % 2U) != 0U)) {
				ret = -EINVAL;
				goto out;
			}
		}

		xfer[i].data = msgs[i].buf;
		xfer[i].data_len = msgs[i].len;
		xfer[i].rnw = (msgs[i].flags & I3C_MSG_READ) != 0U;
		if (hdr) {
			ret = hci->cmd->prep_hdr(hci, &xfer[i], target->dynamic_addr,
						 msgs[i].hdr_cmd_code, msgs[i].hdr_mode);
			if (ret != 0) {
				goto out;
			}
		} else {
			hci->cmd->prep_i3c_xfer(hci, target, &xfer[i]);
		}
		xfer[i].cmd_desc[0] |= CMD_0_ROC;
	}

	ret = mipi_i3c_hci_queue_xfers(hci, xfer, num_msgs);
	if (ret != 0) {
		goto out;
	}

	for (uint8_t i = 0; i < num_msgs; i++) {
		ret = mipi_i3c_hci_response_to_errno(xfer[i].response);
		if (ret != 0) {
			LOG_ERR("I3C target 0x%02x response %#x", target->dynamic_addr,
				xfer[i].response);
			goto out;
		}

		msgs[i].num_xfer = xfer[i].rnw ? RESP_DATA_LENGTH(xfer[i].response) :
						 msgs[i].len;
	}

out:
	k_mutex_unlock(&hci->control_mutex);
	hci_free_xfer(xfer, num_msgs);
	return ret;
}

static struct i3c_i2c_device_desc *
mipi_i3c_hci_i2c_device_find(const struct device *dev, uint16_t addr)
{
	struct i3c_hci *hci = dev->data;

	return i3c_dev_list_i2c_addr_find(&hci->common.attached_dev, addr);
}

static int mipi_i3c_hci_i2c_xfers(const struct device *dev,
				  struct i3c_i2c_device_desc *target,
				  struct i2c_msg *msgs, uint8_t num_msgs)
{
	struct i3c_hci *hci = dev->data;
	struct hci_xfer *xfer;
	int ret = 0;

	if (num_msgs == 0U) {
		return 0;
	}

	if (!target || !msgs || !target->controller_priv) {
		return -EINVAL;
	}

	xfer = hci_alloc_xfer(num_msgs);
	if (!xfer) {
		return -ENOMEM;
	}

	k_mutex_lock(&hci->control_mutex, K_FOREVER);

	for (uint8_t i = 0; i < num_msgs; i++) {
		if (msgs[i].flags & I2C_MSG_ADDR_10_BITS) {
			ret = -ENOTSUP;
			goto out;
		}

		ret = mipi_i3c_hci_check_data_len(hci, msgs[i].len);
		if (ret != 0) {
			goto out;
		}

		xfer[i].data = msgs[i].buf;
		xfer[i].data_len = msgs[i].len;
		xfer[i].rnw = (msgs[i].flags & I2C_MSG_READ) != 0U;
		hci->cmd->prep_i2c_xfer(hci, target, &xfer[i]);
		xfer[i].cmd_desc[0] |= CMD_0_ROC;
	}

	ret = mipi_i3c_hci_queue_xfers(hci, xfer, num_msgs);
	if (ret != 0) {
		goto out;
	}

	for (uint8_t i = 0; i < num_msgs; i++) {
		ret = mipi_i3c_hci_response_to_errno(xfer[i].response);
		if (ret != 0) {
			LOG_ERR("I2C target 0x%02x response %#x", target->addr, xfer[i].response);
			goto out;
		}
	}

out:
	k_mutex_unlock(&hci->control_mutex);
	hci_free_xfer(xfer, num_msgs);
	return ret;
}

static int mipi_i3c_hci_i2c_api_configure(const struct device *dev, uint32_t dev_config)
{
	struct i3c_hci *hci = dev->data;
	uint32_t hz;

	if ((dev_config & I2C_MODE_CONTROLLER) == 0U) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		hz = 100000U;
		break;
	case I2C_SPEED_FAST:
		hz = 400000U;
		break;
	case I2C_SPEED_FAST_PLUS:
		hz = 1000000U;
		break;
	case I2C_SPEED_DT:
		hz = hci->common.ctrl_config.scl.i2c;
		break;
	default:
		return -ERANGE;
	}

	hci->i2c_config = dev_config;
	hci->common.ctrl_config.scl.i2c = hz;

	return 0;
}

static int mipi_i3c_hci_i2c_api_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i3c_hci *hci = dev->data;

	if (!dev_config) {
		return -EINVAL;
	}

	*dev_config = hci->i2c_config;
	return 0;
}

static int mipi_i3c_hci_i2c_api_transfer(const struct device *dev,
					 struct i2c_msg *msgs,
					 uint8_t num_msgs,
					 uint16_t addr)
{
	struct i3c_i2c_device_desc *target;

	target = mipi_i3c_hci_i2c_device_find(dev, addr);
	if (!target) {
		return -ENODEV;
	}

	return mipi_i3c_hci_i2c_xfers(dev, target, msgs, num_msgs);
}

static struct i3c_device_desc *mipi_i3c_hci_device_find(const struct device *dev,
							const struct i3c_device_id *id)
{
	const struct i3c_hci_config *config = dev->config;

	return i3c_dev_list_find(&config->common.dev_list, id);
}

static int mipi_i3c_hci_ibi_raise(const struct device *dev, struct i3c_ibi *request)
{
	return mipi_i3c_hci_target_ibi_raise(dev, request);
}

static int mipi_i3c_hci_ibi_enable(const struct device *dev, struct i3c_device_desc *target)
{
	return mipi_i3c_hci_target_ibi_enable(dev, target);
}

static int mipi_i3c_hci_ibi_disable(const struct device *dev, struct i3c_device_desc *target)
{
	return mipi_i3c_hci_target_ibi_disable(dev, target);
}

static int mipi_i3c_hci_target_register_api(const struct device *dev,
					    struct i3c_target_config *cfg)
{
	return mipi_i3c_hci_target_register(dev, cfg);
}

static int mipi_i3c_hci_target_unregister_api(const struct device *dev,
					      struct i3c_target_config *cfg)
{
	return mipi_i3c_hci_target_unregister(dev, cfg);
}

static int mipi_i3c_hci_target_tx_write_api(const struct device *dev, uint8_t *buf,
					    uint16_t len)
{
	return mipi_i3c_hci_target_tx_write(dev, buf, len);
}

static int mipi_i3c_hci_target_pending_read_notify_api(const struct device *dev,
						       uint8_t *buf, uint16_t len,
						       struct i3c_ibi *notifier)
{
	return mipi_i3c_hci_target_pending_read_notify(dev, buf, len, notifier);
}

static void mipi_i3c_hci_hj_work_handler(struct k_work *work)
{
	struct i3c_hci *hci = CONTAINER_OF(work, struct i3c_hci, hj_work);
	int ret;

	ret = mipi_i3c_hci_do_daa(hci->dev);
	if (ret != 0) {
		LOG_DBG("hot-join DAA failed: %d", ret);
	}
}

static void mipi_i3c_hci_halt_rst_work_handler(struct k_work *work)
{
	struct i3c_hci *hci = CONTAINER_OF(work, struct i3c_hci, halt_rst_work);

	if (hci->io && hci->io->dequeue_xfer) {
		(void)hci->io->dequeue_xfer(hci, NULL, 0);
	}

	mipi_i3c_hci_pio_reset(hci);
	mipi_i3c_hci_resume(hci);
	hci_reg_write(hci, INTR_SIGNAL_ENABLE, MIPI_I3C_HCI_CORE_IRQS);
}

static void mipi_i3c_hci_handle_core_irq(struct i3c_hci *hci)
{
	uint32_t status;

	status = hci_reg_read(hci, INTR_STATUS);
	hci_reg_write(hci, INTR_STATUS, status);

	if (status & INTR_HC_SEQ_CANCEL) {
		LOG_DBG("host controller cancelled transaction sequence");
		status &= ~INTR_HC_SEQ_CANCEL;
	}

	if (status & INTR_HC_CMD_SEQ_UFLOW_STAT) {
		LOG_WRN("host controller command sequence underflow");
		status &= ~INTR_HC_CMD_SEQ_UFLOW_STAT;
	}

	if (status & INTR_HC_INTERNAL_ERR) {
		if (hci->is_target) {
			hci_reg_write(hci, INTR_SIGNAL_ENABLE, 0);
			hci_reg_write(hci, INTR_STATUS, INTR_HC_INTERNAL_ERR);
			(void)k_work_submit(&hci->halt_rst_work);
		}
		status &= ~INTR_HC_INTERNAL_ERR;
	}

	if (status != 0U) {
		LOG_WRN("unexpected HCI interrupt status %#x", status);
	}

	if (hci->io && hci->io->irq_handler) {
		(void)hci->io->irq_handler(hci);
	}
}

/*
 * Vendor IRQ hooks report completion events through this helper so the
 * driver-internal synchronization objects stay private to the core.
 */
void mipi_i3c_hci_vendor_event(struct i3c_hci *hci, enum mipi_i3c_hci_vendor_event event)
{
	switch (event) {
	case MIPI_I3C_HCI_EVENT_IBI_DONE:
		k_sem_give(&hci->ibi_comp);
		break;
	case MIPI_I3C_HCI_EVENT_PENDING_READ_DONE:
		k_sem_give(&hci->pending_r_comp);
		break;
	case MIPI_I3C_HCI_EVENT_BUS_STUCK:
		(void)k_work_submit(&hci->halt_rst_work);
		break;
	default:
		break;
	}
}

static void mipi_i3c_hci_isr(const struct device *dev)
{
	struct i3c_hci *hci = dev->data;
	uint32_t summary;

	if (!hci->vendor || !hci->vendor->read_irq_summary) {
		mipi_i3c_hci_handle_core_irq(hci);
		return;
	}

	summary = hci->vendor->read_irq_summary(hci);
	if (summary == 0U) {
		mipi_i3c_hci_handle_core_irq(hci);
		return;
	}

	if ((summary & MIPI_I3C_HCI_VENDOR_IRQ_CORE) != 0U) {
		mipi_i3c_hci_handle_core_irq(hci);
		summary &= ~MIPI_I3C_HCI_VENDOR_IRQ_CORE;
	}

	if ((summary & MIPI_I3C_HCI_VENDOR_IRQ_IO) != 0U) {
		if (hci->io && hci->io->irq_handler) {
			(void)hci->io->irq_handler(hci);
		}
		summary &= ~MIPI_I3C_HCI_VENDOR_IRQ_IO;
	}

	if ((summary & MIPI_I3C_HCI_VENDOR_IRQ_PRIV) != 0U) {
		uint32_t status = 0U;

		if (hci->vendor->read_priv_irq_status) {
			status = hci->vendor->read_priv_irq_status(hci);
		}

		if (hci->vendor->clear_priv_irq_status) {
			hci->vendor->clear_priv_irq_status(hci, status);
		}

		if (hci->vendor->handle_priv_irq) {
			hci->vendor->handle_priv_irq(hci, status);
		}

		summary &= ~MIPI_I3C_HCI_VENDOR_IRQ_PRIV;
	}

	if (summary != 0U) {
		LOG_WRN("unexpected vendor I3C interrupt summary %#x", summary);
	}

	if (hci->vendor->renew_irq) {
		hci->vendor->renew_irq(hci);
	}
}

static int mipi_i3c_hci_init(const struct device *dev)
{
	struct i3c_hci *hci = dev->data;
	const struct i3c_hci_config *config = dev->config;
	struct i3c_config_controller *bus_config = &hci->common.ctrl_config;
	int ret;

	hci->dev = dev;
	hci->config = config;
	hci->base_regs = config->base_regs;
	hci->quirks = config->quirks;
	hci->vendor = config->vendor;

	if (config->pcfg) {
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("pinctrl apply failed: %d", ret);
			return ret;
		}
	}
	hci->dat = &mipi_i3c_hci_dat_v1;
	hci->i2c_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);

	k_mutex_init(&hci->control_mutex);
	k_sem_init(&hci->ibi_comp, 0, 1);
	k_sem_init(&hci->pending_r_comp, 0, 1);
	k_work_init(&hci->hj_work, mipi_i3c_hci_hj_work_handler);
	k_work_init(&hci->halt_rst_work, mipi_i3c_hci_halt_rst_work_handler);
	atomic_set(&hci->next_cmd_tid, 0);

	ret = mipi_i3c_hci_reset_toggle(&hci->rst, "core");
	if (ret != 0) {
		return ret;
	}

	if (!hci->clk || !device_is_ready(hci->clk)) {
		LOG_ERR("clock controller is not ready");
		return -ENODEV;
	}

	ret = clock_control_on(hci->clk, hci->clock_id);
	if (ret != 0) {
		LOG_ERR("failed to enable clock: %d", ret);
		return ret;
	}

	ret = mipi_i3c_hci_hw_init(hci);
	if (ret != 0) {
		return ret;
	}

	ret = mipi_i3c_hci_target_init(hci);
	if (ret != 0) {
		return ret;
	}

	if (!hci->is_target) {
		ret = hci->vendor && hci->vendor->init ? hci->vendor->init(hci) : 0;
		if (ret != 0) {
			return ret;
		}
	}

#if defined(CONFIG_I3C_MIPI_HCI_ASPEED_AUTOCMD)
	if (!hci->is_target) {
		ret = hci->vendor && hci->vendor->autocmd_init ?
		      hci->vendor->autocmd_init(hci) : 0;
		if (ret != 0) {
			return ret;
		}
	}
#endif

	ret = mipi_i3c_hci_dat_init(hci);
	if (ret != 0) {
		return ret;
	}

	if (!hci->io || !hci->io->init) {
		return -ENOSYS;
	}

	ret = hci->io->init(hci);
	if (ret != 0) {
		return ret;
	}

	if (hci->quirks & HCI_QUIRK_RESP_BUF_THLD) {
		amd_set_resp_buf_thld(hci);
	}

	ret = i3c_addr_slots_init(dev);
	if (ret != 0) {
		return ret;
	}

	bus_config->supported_hdr = mipi_i3c_hci_supported_hdr(hci);

	if (!hci->is_target) {
		ret = mipi_i3c_hci_program_master_addr(hci);
		if (ret != 0) {
			return ret;
		}
	}

	config->irq_config_func(dev);
	hci_reg_write(hci, INTR_SIGNAL_ENABLE, MIPI_I3C_HCI_CORE_IRQS);
	hci_reg_set(hci, HC_CONTROL, HC_CONTROL_BUS_ENABLE);

	if (!hci->is_target && !bus_config->is_secondary &&
	    config->common.dev_list.num_i3c > 0U) {
		ret = i3c_bus_init(dev, &config->common.dev_list);
		if (ret != 0) {
			/*
			 * Bus init may fail when targets are not yet wired up
			 * or have not been set up by the application (e.g. an
			 * on-chip loopback target whose target_register() runs
			 * later). Keep the device available so the application
			 * can retry SETDASA / DAA at runtime.
			 */
			LOG_WRN("bus init failed (%d); device available for retry", ret);
			ret = 0;
		}
	}

	mipi_i3c_hci_iba_ctrl(hci, true);
	mipi_i3c_hci_hj_ctrl(hci, true);

	LOG_DBG("%s initialized as a MIPI I3C HCI controller", dev->name);

	return 0;
}

static const struct i3c_driver_api mipi_i3c_hci_driver_api = {
	.i2c_api.configure = mipi_i3c_hci_i2c_api_configure,
	.i2c_api.get_config = mipi_i3c_hci_i2c_api_get_config,
	.i2c_api.transfer = mipi_i3c_hci_i2c_api_transfer,
	.i2c_api.recover_bus = mipi_i3c_hci_recover_bus,

	.configure = mipi_i3c_hci_configure,
	.config_get = mipi_i3c_hci_config_get,
	.recover_bus = mipi_i3c_hci_recover_bus,
	.attach_i3c_device = mipi_i3c_hci_attach_i3c_device,
	.reattach_i3c_device = mipi_i3c_hci_reattach_i3c_device,
	.detach_i3c_device = mipi_i3c_hci_detach_i3c_device,
	.attach_i2c_device = mipi_i3c_hci_attach_i2c_device,
	.detach_i2c_device = mipi_i3c_hci_detach_i2c_device,
	.do_daa = mipi_i3c_hci_do_daa,
	.do_ccc = mipi_i3c_hci_do_ccc,
	.i3c_xfers = mipi_i3c_hci_i3c_xfers,
	.i3c_device_find = mipi_i3c_hci_device_find,
	.ibi_raise = mipi_i3c_hci_ibi_raise,
	.ibi_enable = mipi_i3c_hci_ibi_enable,
	.ibi_disable = mipi_i3c_hci_ibi_disable,
	.target_register = mipi_i3c_hci_target_register_api,
	.target_unregister = mipi_i3c_hci_target_unregister_api,
	.target_tx_write = mipi_i3c_hci_target_tx_write_api,
	.target_pending_read_notify = mipi_i3c_hci_target_pending_read_notify_api,
};

#define MIPI_I3C_HCI_NO_RESET { .dev = NULL, .id = 0 }

#define MIPI_I3C_HCI_RESET_SPEC_BY_IDX_OR(node_id, idx) \
	RESET_DT_SPEC_GET_BY_IDX_OR(node_id, idx, MIPI_I3C_HCI_NO_RESET)

#define MIPI_I3C_HCI_INIT_NODE(node_id)                                                       \
	static void mipi_i3c_hci_config_func_##node_id(const struct device *dev);             \
	static struct i3c_device_desc mipi_i3c_hci_i3c_dev_array_##node_id[] =                \
		I3C_DEVICE_ARRAY_DT(node_id);                                                  \
	static struct i3c_i2c_device_desc mipi_i3c_hci_i2c_dev_array_##node_id[] =            \
		I3C_I2C_DEVICE_ARRAY_DT(node_id);                                              \
	PINCTRL_DT_DEFINE(node_id);                                                            \
	static const struct i3c_hci_config mipi_i3c_hci_config_##node_id = {                  \
		.common.dev_list.i3c = mipi_i3c_hci_i3c_dev_array_##node_id,                  \
		.common.dev_list.num_i3c = ARRAY_SIZE(mipi_i3c_hci_i3c_dev_array_##node_id),  \
		.common.dev_list.i2c = mipi_i3c_hci_i2c_dev_array_##node_id,                  \
		.common.dev_list.num_i2c = ARRAY_SIZE(mipi_i3c_hci_i2c_dev_array_##node_id),  \
		.base_regs = DT_REG_ADDR(node_id),                                             \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(node_id),                                    \
		.io_mode = DT_PROP_OR(node_id, io_mode, "pio"),                               \
		.quirks = 0,                                                                  \
		.irq_config_func = mipi_i3c_hci_config_func_##node_id,                        \
		.vendor = MIPI_I3C_HCI_PICK_VENDOR(node_id),                                  \
	};                                                                                    \
	static struct i3c_hci mipi_i3c_hci_data_##node_id = {                                 \
		.common.ctrl_config.scl.i3c = DT_PROP_OR(node_id, i3c_scl_hz, 0),             \
		.common.ctrl_config.scl.i2c = DT_PROP_OR(node_id, i2c_scl_hz, 0),             \
		.common.ctrl_config.is_secondary = DT_PROP_OR(node_id, secondary, 0),         \
		.rst = MIPI_I3C_HCI_RESET_SPEC_BY_IDX_OR(node_id, 0),                         \
		.dma_rst = MIPI_I3C_HCI_RESET_SPEC_BY_IDX_OR(node_id, 1),                     \
		.clk = DEVICE_DT_GET(DT_CLOCKS_CTLR(node_id)),                                \
		.clock_id = (clock_control_subsys_t)DT_CLOCKS_CELL(node_id, clk_id),          \
	};                                                                                    \
	DEVICE_DT_DEFINE(node_id, mipi_i3c_hci_init, NULL, &mipi_i3c_hci_data_##node_id,      \
			 &mipi_i3c_hci_config_##node_id, POST_KERNEL,                          \
			 CONFIG_I3C_CONTROLLER_INIT_PRIORITY, &mipi_i3c_hci_driver_api);       \
	static void mipi_i3c_hci_config_func_##node_id(const struct device *dev)              \
	{                                                                                     \
		ARG_UNUSED(dev);                                                              \
		IRQ_CONNECT(DT_IRQN(node_id), DT_IRQ(node_id, priority),                      \
			    mipi_i3c_hci_isr, DEVICE_DT_GET(node_id), 0);                     \
		irq_enable(DT_IRQN(node_id));                                                 \
	}

/*
 * Iterate every status-okay node bound to either the generic
 * "mipi,i3c-hci" binding or the vendor-specific "aspeed,g7-i3c-hci"
 * binding. Each node lands in exactly one bucket (Zephyr picks one
 * binding per node), so there is no risk of duplicate device
 * definitions for the same DT node.
 */
DT_FOREACH_STATUS_OKAY(mipi_i3c_hci, MIPI_I3C_HCI_INIT_NODE)
DT_FOREACH_STATUS_OKAY(aspeed_g7_i3c_hci, MIPI_I3C_HCI_INIT_NODE)
