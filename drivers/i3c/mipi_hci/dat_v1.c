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
#include <string.h>

#include <zephyr/logging/log.h>

#include "dat.h"

LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

/*
 * Device Address Table Structure
 */

#define DAT_1_AUTOCMD_HDR_CODE W1_MASK(58, 51)
#define DAT_1_AUTOCMD_MODE W1_MASK(50, 48)
#define DAT_1_AUTOCMD_VALUE W1_MASK(47, 40)
#define DAT_1_AUTOCMD_MASK W1_MASK(39, 32)

#define DAT_0_DEV_NACK_RETRY_CNT W0_MASK(30, 29)
#define DAT_0_RING_ID W0_MASK(28, 26)
#define DAT_0_DYNADDR_PARITY W0_BIT_(23)
#define DAT_0_DYNAMIC_ADDRESS W0_MASK(22, 16)
#define DAT_0_TS W0_BIT_(15)
#define DAT_0_MR_REJECT W0_BIT_(14)
#define DAT_0_STATIC_ADDRESS W0_MASK(6, 0)

#define DAT_V1_ENTRY_SIZE 8U

static inline uintptr_t hci_dat_v1_entry_addr(struct i3c_hci *hci, unsigned int dat_idx)
{
	return hci->DAT_regs + (uintptr_t)dat_idx * DAT_V1_ENTRY_SIZE;
}

static inline uint32_t hci_dat_v1_w0_read(struct i3c_hci *hci, unsigned int dat_idx)
{
	return sys_read32((mem_addr_t)hci_dat_v1_entry_addr(hci, dat_idx));
}

static inline uint32_t hci_dat_v1_w1_read(struct i3c_hci *hci, unsigned int dat_idx)
{
	return sys_read32((mem_addr_t)(hci_dat_v1_entry_addr(hci, dat_idx) + 4U));
}

static inline void hci_dat_v1_w0_write(struct i3c_hci *hci, unsigned int dat_idx,
				       uint32_t val)
{
	sys_write32(val, (mem_addr_t)hci_dat_v1_entry_addr(hci, dat_idx));
}

static inline void hci_dat_v1_w1_write(struct i3c_hci *hci, unsigned int dat_idx,
				       uint32_t val)
{
	sys_write32(val, (mem_addr_t)(hci_dat_v1_entry_addr(hci, dat_idx) + 4U));
}

static inline uint32_t hci_dat_v1_bit(unsigned int bit)
{
	return BIT(bit & 31U);
}

static inline unsigned int hci_dat_v1_word(unsigned int bit)
{
	return bit / 32U;
}

static bool hci_dat_v1_bitmap_test(uint32_t *bitmap, unsigned int bit)
{
	return (bitmap[hci_dat_v1_word(bit)] & hci_dat_v1_bit(bit)) != 0U;
}

static void hci_dat_v1_bitmap_set(uint32_t *bitmap, unsigned int bit)
{
	bitmap[hci_dat_v1_word(bit)] |= hci_dat_v1_bit(bit);
}

static void hci_dat_v1_bitmap_clear(uint32_t *bitmap, unsigned int bit)
{
	bitmap[hci_dat_v1_word(bit)] &= ~hci_dat_v1_bit(bit);
}

static bool hci_dat_v1_addr_parity(uint8_t addr)
{
	return (POPCOUNT(addr) & 1U) != 0U;
}

static bool hci_dat_v1_idx_valid(struct i3c_hci *hci, unsigned int dat_idx)
{
	return (dat_idx < hci->DAT_entries) && (dat_idx < I3C_HCI_DAT_BITMAP_BITS);
}

static int hci_dat_v1_init(struct i3c_hci *hci)
{
	k_spinlock_key_t key;

	if (hci->DAT_regs == 0U) {
		LOG_ERR("%s only DAT in register space is supported", hci->dev->name);
		return -EOPNOTSUPP;
	}

	if (hci->DAT_entry_size != DAT_V1_ENTRY_SIZE) {
		LOG_ERR("%s unsupported DAT entry size %u", hci->dev->name,
			hci->DAT_entry_size);
		return -EOPNOTSUPP;
	}

	if (hci->DAT_entries > I3C_HCI_DAT_BITMAP_BITS) {
		LOG_ERR("%s DAT has %u entries, software bitmap supports %u",
			hci->dev->name, hci->DAT_entries, I3C_HCI_DAT_BITMAP_BITS);
		return -EOPNOTSUPP;
	}

	key = k_spin_lock(&hci->lock);
	(void)memset(hci->DAT_bitmap, 0, sizeof(hci->DAT_bitmap));
	hci->DAT_data = hci->DAT_bitmap;

	for (unsigned int dat_idx = 0; dat_idx < hci->DAT_entries; dat_idx++) {
		hci_dat_v1_w0_write(hci, dat_idx, 0);
		hci_dat_v1_w1_write(hci, dat_idx, 0);
	}

	k_spin_unlock(&hci->lock, key);

	return 0;
}

static void hci_dat_v1_cleanup(struct i3c_hci *hci)
{
	k_spinlock_key_t key = k_spin_lock(&hci->lock);

	(void)memset(hci->DAT_bitmap, 0, sizeof(hci->DAT_bitmap));
	hci->DAT_data = NULL;

	k_spin_unlock(&hci->lock, key);
}

static int hci_dat_v1_alloc_entry(struct i3c_hci *hci, int preferred_slot)
{
	uint32_t *bitmap;
	k_spinlock_key_t key;
	int ret = 0;

	if (!hci->DAT_data) {
		ret = hci_dat_v1_init(hci);
		if (ret != 0) {
			return ret;
		}
	}

	bitmap = hci->DAT_data;
	key = k_spin_lock(&hci->lock);

	/*
	 * Try the vendor-suggested slot first. Address-indexed silicon
	 * (e.g. ASPEED G7) hands back `dynamic_addr` as the preferred
	 * index because its DAT must be indexed by dynamic address.
	 * Callers without a preference pass -1 and fall straight through
	 * to the first-free scan.
	 */
	if (preferred_slot >= 0 &&
	    (unsigned int)preferred_slot < hci->DAT_entries &&
	    !hci_dat_v1_bitmap_test(bitmap, (unsigned int)preferred_slot)) {
		unsigned int slot = (unsigned int)preferred_slot;

		hci_dat_v1_bitmap_set(bitmap, slot);
		hci_dat_v1_w0_write(hci, slot, DAT_0_SIR_REJECT | DAT_0_MR_REJECT);
		hci_dat_v1_w1_write(hci, slot, 0);
		ret = (int)slot;
		goto out;
	}

	for (unsigned int dat_idx = 0U; dat_idx < hci->DAT_entries; dat_idx++) {
		if (hci_dat_v1_bitmap_test(bitmap, dat_idx)) {
			continue;
		}

		hci_dat_v1_bitmap_set(bitmap, dat_idx);
		hci_dat_v1_w0_write(hci, dat_idx, DAT_0_SIR_REJECT | DAT_0_MR_REJECT);
		hci_dat_v1_w1_write(hci, dat_idx, 0);
		ret = (int)dat_idx;
		goto out;
	}

	ret = -ENOSPC;

out:
	k_spin_unlock(&hci->lock, key);
	return ret;
}

static void hci_dat_v1_free_entry(struct i3c_hci *hci, unsigned int dat_idx)
{
	uint32_t *bitmap = hci->DAT_data;
	k_spinlock_key_t key;

	if (!hci_dat_v1_idx_valid(hci, dat_idx)) {
		return;
	}

	key = k_spin_lock(&hci->lock);

	hci_dat_v1_w0_write(hci, dat_idx, 0);
	hci_dat_v1_w1_write(hci, dat_idx, 0);
	if (bitmap) {
		hci_dat_v1_bitmap_clear(bitmap, dat_idx);
	}

	k_spin_unlock(&hci->lock, key);
}

static void hci_dat_v1_set_dynamic_addr(struct i3c_hci *hci, unsigned int dat_idx,
					uint8_t address)
{
	uint32_t *bitmap;
	uint32_t dat_w0;
	uint32_t dat_w1;
	k_spinlock_key_t key;
	bool addr_indexed;

	if (!hci_dat_v1_idx_valid(hci, dat_idx)) {
		LOG_ERR("%s invalid DAT dynamic address update idx %u addr %#x",
			hci->dev->name, dat_idx, address);
		return;
	}

	bitmap = hci->DAT_data;
	addr_indexed = hci->vendor && hci->vendor->dat_wants_addr_indexed &&
		       hci->vendor->dat_wants_addr_indexed(hci);

	key = k_spin_lock(&hci->lock);

	/*
	 * Snapshot the originally-allocated entry. Silicon that tracks
	 * DAT entries by slot index (e.g. ASPEED G7) requires a relocation to
	 * carry the previously-programmed flag bits (SIR_REJECT,
	 * MR_REJECT, etc.) over to the new slot — otherwise the new
	 * slot is "blank" and the HW will not associate it with the
	 * assigned address for private transfers.
	 */
	dat_w0 = hci_dat_v1_w0_read(hci, dat_idx);
	dat_w1 = hci_dat_v1_w1_read(hci, dat_idx);

	if (addr_indexed && dat_idx != address) {
		if (!hci_dat_v1_idx_valid(hci, address)) {
			LOG_ERR("%s DAT relocation rejected: addr %#x out of range",
				hci->dev->name, address);
			goto out;
		}
		if (bitmap && hci_dat_v1_bitmap_test(bitmap, address)) {
			LOG_ERR("%s DAT relocation rejected: slot %u already taken",
				hci->dev->name, address);
			goto out;
		}

		hci_dat_v1_w0_write(hci, dat_idx, 0);
		hci_dat_v1_w1_write(hci, dat_idx, 0);
		if (bitmap) {
			hci_dat_v1_bitmap_clear(bitmap, dat_idx);
			hci_dat_v1_bitmap_set(bitmap, address);
		}
		dat_idx = address;
	}

	dat_w0 &= ~(DAT_0_DYNAMIC_ADDRESS | DAT_0_DYNADDR_PARITY);
	dat_w0 |= FIELD_PREP(DAT_0_DYNAMIC_ADDRESS, address);
	if (!hci_dat_v1_addr_parity(address)) {
		dat_w0 |= DAT_0_DYNADDR_PARITY;
	}
	hci_dat_v1_w0_write(hci, dat_idx, dat_w0);
	hci_dat_v1_w1_write(hci, dat_idx, dat_w1);

out:
	k_spin_unlock(&hci->lock, key);
}

static void hci_dat_v1_set_static_addr(struct i3c_hci *hci, unsigned int dat_idx,
				       uint8_t address)
{
	uint32_t dat_w0;
	k_spinlock_key_t key;

	if (!hci_dat_v1_idx_valid(hci, dat_idx)) {
		return;
	}

	/*
	 * Vendors whose DAT is addressed by slot index (e.g. ASPEED G7) cannot
	 * accept writes to the DAT_0_STATIC_ADDRESS field — the slot index
	 * is the address. The caller is expected to have already placed the
	 * device at DAT[address] via alloc_entry's preferred-slot path.
	 */
	if (hci->vendor && hci->vendor->dat_wants_addr_indexed &&
	    hci->vendor->dat_wants_addr_indexed(hci)) {
		return;
	}

	key = k_spin_lock(&hci->lock);

	dat_w0 = hci_dat_v1_w0_read(hci, dat_idx);
	dat_w0 &= ~DAT_0_STATIC_ADDRESS;
	dat_w0 |= FIELD_PREP(DAT_0_STATIC_ADDRESS, address);
	hci_dat_v1_w0_write(hci, dat_idx, dat_w0);

	k_spin_unlock(&hci->lock, key);
}

static void hci_dat_v1_set_flags(struct i3c_hci *hci, unsigned int dat_idx,
				 uint32_t w0_flags, uint32_t w1_flags)
{
	uint32_t dat_w0;
	uint32_t dat_w1;
	k_spinlock_key_t key;

	if (!hci_dat_v1_idx_valid(hci, dat_idx)) {
		return;
	}

	key = k_spin_lock(&hci->lock);

	dat_w0 = hci_dat_v1_w0_read(hci, dat_idx);
	dat_w1 = hci_dat_v1_w1_read(hci, dat_idx);
	hci_dat_v1_w0_write(hci, dat_idx, dat_w0 | w0_flags);
	hci_dat_v1_w1_write(hci, dat_idx, dat_w1 | w1_flags);

	k_spin_unlock(&hci->lock, key);
}

static void hci_dat_v1_clear_flags(struct i3c_hci *hci, unsigned int dat_idx,
				   uint32_t w0_flags, uint32_t w1_flags)
{
	uint32_t dat_w0;
	uint32_t dat_w1;
	k_spinlock_key_t key;

	if (!hci_dat_v1_idx_valid(hci, dat_idx)) {
		return;
	}

	key = k_spin_lock(&hci->lock);

	dat_w0 = hci_dat_v1_w0_read(hci, dat_idx);
	dat_w1 = hci_dat_v1_w1_read(hci, dat_idx);
	hci_dat_v1_w0_write(hci, dat_idx, dat_w0 & ~w0_flags);
	hci_dat_v1_w1_write(hci, dat_idx, dat_w1 & ~w1_flags);

	k_spin_unlock(&hci->lock, key);
}

static int hci_dat_v1_get_index(struct i3c_hci *hci, uint8_t dev_addr)
{
	uint32_t *bitmap = hci->DAT_data;
	k_spinlock_key_t key;
	int ret = -ENODEV;
	bool addr_indexed;

	if (!bitmap) {
		return -ENODEV;
	}

	addr_indexed = hci->vendor && hci->vendor->dat_wants_addr_indexed &&
		       hci->vendor->dat_wants_addr_indexed(hci);

	key = k_spin_lock(&hci->lock);

	if (addr_indexed) {
		/*
		 * Address-indexed silicon hard-wires the DAT slot index to
		 * the dynamic / static address, so the address itself IS the
		 * index. Just confirm the slot is allocated.
		 */
		if (hci_dat_v1_idx_valid(hci, dev_addr) &&
		    hci_dat_v1_bitmap_test(bitmap, dev_addr)) {
			ret = (int)dev_addr;
		}
		goto out;
	}

	for (unsigned int dat_idx = 0U; dat_idx < hci->DAT_entries; dat_idx++) {
		uint32_t dat_w0;

		if (!hci_dat_v1_bitmap_test(bitmap, dat_idx)) {
			continue;
		}

		dat_w0 = hci_dat_v1_w0_read(hci, dat_idx);
		if (FIELD_GET(DAT_0_DYNAMIC_ADDRESS, dat_w0) == dev_addr ||
		    FIELD_GET(DAT_0_STATIC_ADDRESS, dat_w0) == dev_addr) {
			ret = (int)dat_idx;
			break;
		}
	}

out:
	k_spin_unlock(&hci->lock, key);
	return ret;
}

static void hci_dat_v1_mark_for_sw_retry(struct i3c_hci *hci, unsigned int dat_idx,
					 bool mark)
{
	uint32_t dat_w0;
	k_spinlock_key_t key;

	if (!hci_dat_v1_idx_valid(hci, dat_idx)) {
		return;
	}

	key = k_spin_lock(&hci->lock);

	if (mark) {
		dat_w0 = hci_dat_v1_w0_read(hci, dat_idx);
		dat_w0 &= ~DAT_0_DEV_NACK_RETRY_CNT;
		hci_dat_v1_w0_write(hci, dat_idx, dat_w0);
	}

	k_spin_unlock(&hci->lock, key);
}

const struct hci_dat_ops mipi_i3c_hci_dat_v1 = {
	.init = hci_dat_v1_init,
	.cleanup = hci_dat_v1_cleanup,
	.alloc_entry = hci_dat_v1_alloc_entry,
	.free_entry = hci_dat_v1_free_entry,
	.set_dynamic_addr = hci_dat_v1_set_dynamic_addr,
	.set_static_addr = hci_dat_v1_set_static_addr,
	.set_flags = hci_dat_v1_set_flags,
	.clear_flags = hci_dat_v1_clear_flags,
	.get_index = hci_dat_v1_get_index,
	.mark_for_sw_retry = hci_dat_v1_mark_for_sw_retry,
};
