/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Ported to Zephyr from the Linux mipi-i3c-hci driver
 * (drivers/i3c/master/mipi-i3c-hci/).
 */

#include <errno.h>

#include <zephyr/logging/log.h>

#include "ext_caps.h"
#include "xfer_mode_rate.h"

LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

/* Extended Capability Header */
#define CAP_HEADER_LENGTH GENMASK(23, 8)
#define CAP_HEADER_ID GENMASK(7, 0)

#define CAP_ID_HARDWARE_ID 0x01
#define CAP_ID_MASTER_CONFIG 0x02
#define CAP_ID_MULTI_BUS 0x03
#define CAP_ID_XFER_MODES 0x04
#define CAP_ID_AUTO_COMMAND 0x05
#define CAP_ID_XFER_RATES 0x08
#define CAP_ID_DEBUG 0x0c
#define CAP_ID_SCHEDULED_CMD 0x0d
#define CAP_ID_NON_CURRENT_MASTER 0x0e
#define CAP_ID_CCC_RESP_CONF 0x0f
#define CAP_ID_GLOBAL_DAT 0x10
#define CAP_ID_MULTILANE 0x9d
#define CAP_ID_NCM_MULTILANE 0x9e

static inline uint32_t hci_extcap_read(uintptr_t base, uint32_t offset)
{
	return sys_read32((mem_addr_t)(base + offset));
}

static inline void hci_extcap_write(uintptr_t base, uint32_t offset, uint32_t val)
{
	sys_write32(val, (mem_addr_t)(base + offset));
}

static int hci_extcap_hardware_id(struct i3c_hci *hci, uintptr_t base)
{
	hci->vendor_mipi_id = hci_extcap_read(base, 0x04);
	hci->vendor_version_id = hci_extcap_read(base, 0x08);
	hci->vendor_product_id = hci_extcap_read(base, 0x0c) >> 16;

	LOG_INF("%s vendor MIPI ID %#x", hci->dev->name, hci->vendor_mipi_id);
	LOG_INF("%s vendor version ID %#x", hci->dev->name, hci->vendor_version_id);
	LOG_INF("%s vendor product ID %#x", hci->dev->name, hci->vendor_product_id);

	if (hci->vendor_mipi_id == MIPI_VENDOR_NXP) {
		hci->quirks |= HCI_QUIRK_RAW_CCC;
		LOG_DBG("%s raw CCC quirk enabled for NXP HCI", hci->dev->name);
	}

	return 0;
}

static int hci_extcap_master_config(struct i3c_hci *hci, uintptr_t base)
{
	uint32_t master_config = hci_extcap_read(base, 0x04);
	unsigned int operation_mode = FIELD_GET(GENMASK(5, 4), master_config);
	static const char * const functionality[] = {
		"(unknown)",
		"master only",
		"target only",
		"primary/secondary master",
	};

	LOG_INF("%s operation mode: %s", hci->dev->name, functionality[operation_mode]);
	if ((operation_mode & 0x1U) != 0U) {
		return 0;
	}

	LOG_ERR("%s only master mode is currently supported", hci->dev->name);
	return -EOPNOTSUPP;
}

static int hci_extcap_multi_bus(struct i3c_hci *hci, uintptr_t base)
{
	uint32_t bus_instance = hci_extcap_read(base, 0x04);
	unsigned int count = FIELD_GET(GENMASK(3, 0), bus_instance);

	LOG_INF("%s %u bus instances", hci->dev->name, count);
	return 0;
}

static int hci_extcap_xfer_modes(struct i3c_hci *hci, uintptr_t base)
{
	uint32_t header = hci_extcap_read(base, 0);
	uint32_t entries = FIELD_GET(CAP_HEADER_LENGTH, header) - 1U;

	LOG_INF("%s transfer mode table has %u entries", hci->dev->name, entries);
	for (unsigned int index = 0; index < entries; index++) {
		uint32_t mode_entry = hci_extcap_read(base, 4U + index * 4U);

		LOG_DBG("%s mode %u: %#x supported=%u mode=%u", hci->dev->name,
			index, mode_entry, (unsigned int)FIELD_GET(XFERMODE_SUPPORTED, mode_entry),
			(unsigned int)FIELD_GET(XFERMODE_MODE, mode_entry));
	}

	return 0;
}

static int hci_extcap_xfer_rates(struct i3c_hci *hci, uintptr_t base)
{
	uint32_t header = hci_extcap_read(base, 0);
	uint32_t entries = FIELD_GET(CAP_HEADER_LENGTH, header) - 1U;

	LOG_INF("%s available data rates:", hci->dev->name);
	for (unsigned int index = 0; index < entries; index++) {
		uint32_t rate_entry = hci_extcap_read(base, 4U + index * 4U);
		unsigned int rate = FIELD_GET(XFERRATE_ACTUAL_RATE_KHZ, rate_entry);
		unsigned int rate_id = FIELD_GET(XFERRATE_RATE_ID, rate_entry);
		unsigned int mode_id = FIELD_GET(XFERRATE_MODE_ID, rate_entry);
		const char *mode = "unknown mode";

		if (mode_id == XFERRATE_MODE_I3C) {
			mode = "I3C";
		} else if (mode_id == XFERRATE_MODE_I2C) {
			mode = "I2C";
		}

		LOG_INF("%s rate %u for %s = %u kHz", hci->dev->name, rate_id, mode, rate);
		LOG_DBG("%s rate entry %u: %#x", hci->dev->name, index, rate_entry);
	}

	return 0;
}

static int hci_extcap_auto_command(struct i3c_hci *hci, uintptr_t base)
{
	uint32_t autocmd_ext_caps = hci_extcap_read(base, 0x04);
	uint32_t autocmd_ext_config = hci_extcap_read(base, 0x08);
	unsigned int max_count = FIELD_GET(GENMASK(3, 0), autocmd_ext_caps);
	unsigned int count = FIELD_GET(GENMASK(3, 0), autocmd_ext_config);

	LOG_INF("%s %u/%u active auto-command entries", hci->dev->name, count, max_count);
	hci->AUTOCMD_regs = base;
	return 0;
}

static int hci_extcap_debug(struct i3c_hci *hci, uintptr_t base)
{
	LOG_INF("%s debug registers present", hci->dev->name);
	hci->DEBUG_regs = base;
	return 0;
}

static int hci_extcap_scheduled_cmd(struct i3c_hci *hci, uintptr_t base)
{
	ARG_UNUSED(base);

	LOG_INF("%s scheduled commands available", hci->dev->name);
	return 0;
}

static int hci_extcap_non_curr_master(struct i3c_hci *hci, uintptr_t base)
{
	ARG_UNUSED(base);

	LOG_INF("%s non-current master support available", hci->dev->name);
	return 0;
}

static int hci_extcap_ccc_resp_conf(struct i3c_hci *hci, uintptr_t base)
{
	ARG_UNUSED(base);

	LOG_INF("%s CCC response configuration available", hci->dev->name);
	return 0;
}

static int hci_extcap_global_dat(struct i3c_hci *hci, uintptr_t base)
{
	ARG_UNUSED(base);

	LOG_INF("%s global DAT available", hci->dev->name);
	return 0;
}

static int hci_extcap_multilane(struct i3c_hci *hci, uintptr_t base)
{
	ARG_UNUSED(base);

	LOG_INF("%s master multi-lane support available", hci->dev->name);
	return 0;
}

static int hci_extcap_ncm_multilane(struct i3c_hci *hci, uintptr_t base)
{
	ARG_UNUSED(base);

	LOG_INF("%s NCM multi-lane support available", hci->dev->name);
	return 0;
}

struct hci_ext_caps {
	uint8_t id;
	uint16_t min_length;
	int (*parser)(struct i3c_hci *hci, uintptr_t base);
};

#define EXT_CAP(_id, _highest_mandatory_reg_offset, _parser) \
	{ \
		.id = (_id), \
		.parser = (_parser), \
		.min_length = (_highest_mandatory_reg_offset) / 4U + 1U, \
	}

static const struct hci_ext_caps ext_capabilities[] = {
	EXT_CAP(CAP_ID_HARDWARE_ID, 0x0c, hci_extcap_hardware_id),
	EXT_CAP(CAP_ID_MASTER_CONFIG, 0x04, hci_extcap_master_config),
	EXT_CAP(CAP_ID_MULTI_BUS, 0x04, hci_extcap_multi_bus),
	EXT_CAP(CAP_ID_XFER_MODES, 0x24, hci_extcap_xfer_modes),
	EXT_CAP(CAP_ID_AUTO_COMMAND, 0x08, hci_extcap_auto_command),
	EXT_CAP(CAP_ID_XFER_RATES, 0x40, hci_extcap_xfer_rates),
	EXT_CAP(CAP_ID_DEBUG, 0x10, hci_extcap_debug),
	EXT_CAP(CAP_ID_SCHEDULED_CMD, 0x0c, hci_extcap_scheduled_cmd),
	EXT_CAP(CAP_ID_NON_CURRENT_MASTER, 0x80, hci_extcap_non_curr_master),
	EXT_CAP(CAP_ID_CCC_RESP_CONF, 0x04, hci_extcap_ccc_resp_conf),
	EXT_CAP(CAP_ID_GLOBAL_DAT, 0x08, hci_extcap_global_dat),
	EXT_CAP(CAP_ID_MULTILANE, 0x04, hci_extcap_multilane),
	EXT_CAP(CAP_ID_NCM_MULTILANE, 0x04, hci_extcap_ncm_multilane),
};

static int hci_extcap_vendor_nxp(struct i3c_hci *hci, uintptr_t base)
{
	hci->vendor_data = (void *)base;
	LOG_INF("%s NXP build date info %#x", hci->dev->name, hci_extcap_read(base, 0x04));
	hci_extcap_write(base, 0x04, 0xdeadbeef);
	return 0;
}

static int hci_extcap_vendor_aspeed(struct i3c_hci *hci, uintptr_t base)
{
	uint32_t regs_offset;

	regs_offset = hci_extcap_read(base, 0x04);
	LOG_INF("%s INHOUSE control at offset %#x", hci->dev->name, regs_offset);
	hci->VENDOR_regs = regs_offset != 0U ? hci->base_regs + regs_offset : 0U;

	regs_offset = hci_extcap_read(base, 0x08);
	LOG_INF("%s PHY control at offset %#x", hci->dev->name, regs_offset);
	hci->PHY_regs = regs_offset != 0U ? hci->base_regs + regs_offset : 0U;

	return 0;
}

struct hci_ext_cap_vendor_specific {
	uint32_t vendor;
	uint8_t cap;
	uint16_t min_length;
	int (*parser)(struct i3c_hci *hci, uintptr_t base);
};

#define EXT_CAP_VENDOR(_vendor, _cap, _highest_mandatory_reg_offset, _parser) \
	{ \
		.vendor = (_vendor), \
		.cap = (_cap), \
		.parser = (_parser), \
		.min_length = (_highest_mandatory_reg_offset) / 4U + 1U, \
	}

static const struct hci_ext_cap_vendor_specific vendor_ext_caps[] = {
	EXT_CAP_VENDOR(MIPI_VENDOR_NXP, 0xc0, 0x20, hci_extcap_vendor_nxp),
	EXT_CAP_VENDOR(MIPI_VENDOR_ASPEED, 0xc0, 0x08, hci_extcap_vendor_aspeed),
};

static int hci_extcap_vendor_specific(struct i3c_hci *hci, uintptr_t base,
				      uint32_t cap_id, uint32_t cap_length)
{
	const struct hci_ext_cap_vendor_specific *vendor_cap_entry = NULL;

	for (unsigned int i = 0; i < ARRAY_SIZE(vendor_ext_caps); i++) {
		if (vendor_ext_caps[i].vendor == hci->vendor_mipi_id &&
		    vendor_ext_caps[i].cap == cap_id) {
			vendor_cap_entry = &vendor_ext_caps[i];
			break;
		}
	}

	if (!vendor_cap_entry) {
		LOG_WRN("%s unknown ext_cap %#x for vendor %#x", hci->dev->name,
			cap_id, hci->vendor_mipi_id);
		return 0;
	}

	if (cap_length < vendor_cap_entry->min_length) {
		LOG_ERR("%s ext_cap %#x has size %u, expecting >= %u",
			hci->dev->name, cap_id, cap_length, vendor_cap_entry->min_length);
		return -EINVAL;
	}

	return vendor_cap_entry->parser(hci, base);
}

int i3c_hci_parse_ext_caps(struct i3c_hci *hci)
{
	uintptr_t curr_cap = hci->EXTCAPS_regs;
	uintptr_t end = curr_cap + 0x1000U;
	int err = 0;

	if (curr_cap == 0U) {
		return 0;
	}

	while ((err == 0) && (curr_cap < end)) {
		uint32_t cap_header = hci_extcap_read(curr_cap, 0);
		uint32_t cap_id = FIELD_GET(CAP_HEADER_ID, cap_header);
		uint32_t cap_length = FIELD_GET(CAP_HEADER_LENGTH, cap_header);
		const struct hci_ext_caps *cap_entry = NULL;

		LOG_DBG("%s id=%#x length=%u", hci->dev->name, cap_id, cap_length);

		if (cap_id == 0U && cap_length != 0U && cap_length != 1U) {
			LOG_ERR("%s malformed ext-cap header (id=0, length=%u)",
				hci->dev->name, cap_length);
			return -EINVAL;
		}

		if (cap_id == 0U || cap_length == 0U) {
			break;
		}

		if (cap_length > ((end - curr_cap) / sizeof(uint32_t))) {
			LOG_ERR("%s ext_cap %#x has size %u, too big",
				hci->dev->name, cap_id, cap_length);
			return -EINVAL;
		}

		if (cap_id >= 0xc0U && cap_id <= 0xcfU) {
			err = hci_extcap_vendor_specific(hci, curr_cap, cap_id, cap_length);
			curr_cap += cap_length * sizeof(uint32_t);
			continue;
		}

		for (unsigned int i = 0; i < ARRAY_SIZE(ext_capabilities); i++) {
			if (ext_capabilities[i].id == cap_id) {
				cap_entry = &ext_capabilities[i];
				break;
			}
		}

		if (!cap_entry) {
			LOG_WRN("%s unknown ext_cap %#x", hci->dev->name, cap_id);
		} else if (cap_length < cap_entry->min_length) {
			LOG_ERR("%s ext_cap %#x has size %u, expecting >= %u",
				hci->dev->name, cap_id, cap_length, cap_entry->min_length);
			err = -EINVAL;
		} else {
			err = cap_entry->parser(hci, curr_cap);
		}

		curr_cap += cap_length * sizeof(uint32_t);
	}

	return err;
}
