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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/logging/log.h>

#include "cmd.h"
#include "dat.h"
#include "hci.h"
#include "ibi.h"

LOG_MODULE_DECLARE(mipi_i3c_hci, CONFIG_I3C_MIPI_HCI_LOG_LEVEL);

#define MIPI_I3C_HCI_DMA_MAX_RINGS 8U
#define MIPI_I3C_HCI_DMA_XFER_RING_ENTRIES 16U
#define MIPI_I3C_HCI_DMA_IBI_RINGS 1U
#define MIPI_I3C_HCI_DMA_IBI_STATUS_RING_ENTRIES 32U
#define MIPI_I3C_HCI_DMA_IBI_CHUNK_POOL_SIZE 128U
#define MIPI_I3C_HCI_DMA_DEFAULT_ALIGN 64U
#define MIPI_I3C_HCI_DMA_ABORT_TIMEOUT_US 100000U

#define rhs_reg_read(hci, reg) sys_read32((mem_addr_t)((hci)->RHS_regs + RHS_##reg))
#define rhs_reg_write(hci, reg, val) \
	sys_write32((uint32_t)(val), (mem_addr_t)((hci)->RHS_regs + RHS_##reg))

#define rh_reg_read(rh, reg) sys_read32((mem_addr_t)((rh)->regs + RH_##reg))
#define rh_reg_write(rh, reg, val) \
	sys_write32((uint32_t)(val), (mem_addr_t)((rh)->regs + RH_##reg))

/* Ring Header Section preamble. */
#define RHS_CONTROL 0x00
#define PREAMBLE_SIZE GENMASK(31, 24)
#define HEADER_SIZE GENMASK(23, 16)
#define MAX_HEADER_COUNT_CAP GENMASK(7, 4)
#define MAX_HEADER_COUNT GENMASK(3, 0)
#define RHS_RHN_OFFSET(n) (0x04U + ((n) * 4U))

/* Per-ring header registers. */
#define RH_CR_SETUP 0x00
#define CR_XFER_STRUCT_SIZE GENMASK(31, 24)
#define CR_RESP_STRUCT_SIZE GENMASK(23, 16)
#define CR_RING_SIZE GENMASK(8, 0)

#define RH_IBI_SETUP 0x04
#define IBI_STATUS_STRUCT_SIZE GENMASK(31, 24)
#define IBI_STATUS_RING_SIZE GENMASK(23, 16)
#define IBI_DATA_CHUNK_SIZE GENMASK(12, 10)
#define IBI_DATA_CHUNK_COUNT GENMASK(9, 0)

#define RH_CHUNK_CONTROL 0x08

#define RH_INTR_STATUS 0x10
#define RH_INTR_STATUS_ENABLE 0x14
#define RH_INTR_SIGNAL_ENABLE 0x18
#define RH_INTR_FORCE 0x1c
#define INTR_IBI_READY BIT(12)
#define INTR_TRANSFER_COMPLETION BIT(11)
#define INTR_RING_OP BIT(10)
#define INTR_TRANSFER_ERR BIT(9)
#define INTR_IBI_RING_FULL BIT(6)
#define INTR_TRANSFER_ABORT BIT(5)

#define RH_RING_STATUS 0x20
#define RING_STATUS_LOCKED BIT(3)
#define RING_STATUS_ABORTED BIT(2)
#define RING_STATUS_RUNNING BIT(1)
#define RING_STATUS_ENABLED BIT(0)

#define RH_RING_CONTROL 0x24
#define RING_CTRL_ABORT BIT(2)
#define RING_CTRL_RUN_STOP BIT(1)
#define RING_CTRL_ENABLE BIT(0)

#define RH_RING_OPERATION1 0x28
#define RING_OP1_IBI_DEQ_PTR GENMASK(23, 16)
#define RING_OP1_CR_SW_DEQ_PTR GENMASK(15, 8)
#define RING_OP1_CR_ENQ_PTR GENMASK(7, 0)

#define RH_RING_OPERATION2 0x2c
#define RING_OP2_IBI_ENQ_PTR GENMASK(23, 16)
#define RING_OP2_CR_DEQ_PTR GENMASK(7, 0)

#define RH_CMD_RING_BASE_LO 0x30
#define RH_CMD_RING_BASE_HI 0x34
#define RH_RESP_RING_BASE_LO 0x38
#define RH_RESP_RING_BASE_HI 0x3c
#define RH_IBI_STATUS_RING_BASE_LO 0x40
#define RH_IBI_STATUS_RING_BASE_HI 0x44
#define RH_IBI_DATA_RING_BASE_LO 0x48
#define RH_IBI_DATA_RING_BASE_HI 0x4c

#define RH_CMD_RING_SG 0x50
#define RH_RESP_RING_SG 0x54
#define RH_IBI_STATUS_RING_SG 0x58
#define RH_IBI_DATA_RING_SG 0x5c
#define RING_SG_BLP BIT(31)
#define RING_SG_LIST_SIZE GENMASK(15, 0)

/* Data Buffer Descriptor, appended to the command descriptor in memory. */
#define DATA_BUF_BLP BIT(31)
#define DATA_BUF_IOC BIT(30)
#define DATA_BUF_BLOCK_SIZE GENMASK(15, 0)

#define HCI_DMA_RING_INTRS                                                               \
	(INTR_TRANSFER_COMPLETION | INTR_RING_OP | INTR_TRANSFER_ERR |                  \
	 INTR_TRANSFER_ABORT | INTR_IBI_RING_FULL)
#define HCI_DMA_IBI_INTRS (HCI_DMA_RING_INTRS | INTR_IBI_READY)

enum hci_dma_buf_dir {
	HCI_DMA_TO_DEVICE,
	HCI_DMA_FROM_DEVICE,
};

struct hci_dma_buf {
	void *buf;
	uintptr_t addr;
	size_t len;
	enum hci_dma_buf_dir dir;
	/*
	 * Bounce path: when the caller-supplied buffer lives in a region the
	 * I3C DMA master cannot reach (e.g. SSP TCM / stack), we allocate a
	 * DMA-visible scratch buffer and route the transfer through it.
	 * `orig` keeps the user-side pointer so the read direction can copy
	 * the received bytes back at unmap time; `bounce` owns the lifecycle
	 * so it gets freed alongside the descriptor.
	 */
	void *orig;
	void *bounce;
};

struct i3c_hci_dma_ibi_data {
	unsigned int max_len;
};

struct i3c_hci_dma_ring {
	uintptr_t regs;

	uint8_t *cmd_ring;
	uint8_t *resp_ring;
	uint8_t *ibi_status_ring;
	uint8_t *ibi_data_ring;

	uintptr_t cmd_dma;
	uintptr_t resp_dma;
	uintptr_t ibi_status_dma;
	uintptr_t ibi_data_dma;

	size_t cmd_alloc_size;
	size_t resp_alloc_size;
	size_t ibi_status_alloc_size;
	size_t ibi_data_alloc_size;

	unsigned int xfer_entries;
	unsigned int ibi_status_entries;
	unsigned int ibi_chunks_total;

	unsigned int xfer_struct_sz;
	unsigned int resp_struct_sz;
	unsigned int ibi_status_sz;
	unsigned int ibi_chunk_sz;

	unsigned int done_ptr;
	unsigned int ibi_chunk_ptr;
	bool has_ibi;

	struct hci_xfer **src_xfers;
	struct k_sem op_done;
};

struct i3c_dma {
	unsigned int total;
	unsigned int next_ring;
	bool target_rx_owned;
	struct i3c_hci_dma_ring rings[];
};

/*
 * Some platforms expose memory to the DMA master through a remap
 * window: CPU addresses must be translated to the bus-side physical
 * address before being handed to the controller, and not every CPU
 * address is necessarily reachable by the DMA master at all. Both
 * concerns are delegated to the vendor backend; the defaults assume
 * an identity-mapped, fully DMA-visible address space.
 */
static uint64_t hci_dma_to_phys(struct i3c_hci *hci, uintptr_t addr)
{
	if (addr == 0U) {
		return 0;
	}

	if (hci->vendor && hci->vendor->dma_to_phys) {
		return hci->vendor->dma_to_phys(hci, addr);
	}

	return (uint64_t)addr;
}

static bool hci_dma_phys_addr_visible(struct i3c_hci *hci, uint64_t phys, size_t len)
{
	if (len == 0U) {
		return true;
	}

	if (hci->vendor && hci->vendor->dma_addr_visible) {
		return hci->vendor->dma_addr_visible(hci, phys, len);
	}

	return true;
}

static uint32_t hci_dma_addr_lo(struct i3c_hci *hci, uintptr_t addr)
{
	return (uint32_t)(hci_dma_to_phys(hci, addr) & UINT32_MAX);
}

static uint32_t hci_dma_addr_hi(struct i3c_hci *hci, uintptr_t addr)
{
	return (uint32_t)((hci_dma_to_phys(hci, addr) >> 32) & UINT32_MAX);
}

static size_t hci_dma_cache_align(void)
{
	size_t align = sys_cache_data_line_size_get();

	if (align == 0U) {
		align = MIPI_I3C_HCI_DMA_DEFAULT_ALIGN;
	}

	return MAX(align, sizeof(uintptr_t));
}

static size_t hci_dma_round_up(size_t value, size_t align)
{
	return ROUND_UP(value, align);
}

static void hci_dma_cache_flush(void *addr, size_t len)
{
	if (addr && len != 0U) {
		(void)sys_cache_data_flush_range(addr, len);
	}
}

static void hci_dma_cache_invd(void *addr, size_t len)
{
	if (addr && len != 0U) {
		/*
		 * Invalidate only. We must NOT call
		 * sys_cache_data_flush_and_invd_range here: when the CPU has
		 * never touched the region (HW just wrote a response), the
		 * flush half would copy the stale cache lines (zeroes from
		 * earlier memset) back over the HW-written bytes, and the
		 * subsequent read would always return zero.
		 */
		(void)sys_cache_data_invd_range(addr, len);
	}
}

static void *hci_dma_alloc_buf(struct i3c_hci *hci, size_t size, size_t align,
			       size_t *alloc_size)
{
	void *buf;
	size_t rounded = hci_dma_round_up(size, align);
	uint64_t phys;

	buf = k_aligned_alloc(align, rounded);
	if (!buf) {
		return NULL;
	}

	phys = hci_dma_to_phys(hci, (uintptr_t)buf);
	if (!hci_dma_phys_addr_visible(hci, phys, rounded)) {
		LOG_ERR("%s DMA ring buffer %p phys 0x%llx size %zu "
			"not I3C DMA-visible; refusing DMA mode",
			hci->dev->name, buf, (unsigned long long)phys, rounded);
		k_free(buf);
		return NULL;
	}

	(void)memset(buf, 0, rounded);
	hci_dma_cache_invd(buf, rounded);

	if (alloc_size) {
		*alloc_size = rounded;
	}

	return buf;
}

static uint8_t *hci_dma_ring_entry(uint8_t *ring, unsigned int entry_size,
				   unsigned int index)
{
	return ring + ((size_t)entry_size * index);
}

static unsigned int hci_dma_cmd_words(struct i3c_hci *hci)
{
	if (hci->cmd == &mipi_i3c_hci_cmd_v2) {
		return 4U;
	}

	return hci->is_target ? 1U : 2U;
}

static unsigned int hci_dma_required_xfer_size(struct i3c_hci *hci)
{
	return (hci_dma_cmd_words(hci) + 3U) * sizeof(uint32_t);
}

static unsigned int hci_dma_pow2_ceil(unsigned int value)
{
	if (value <= 1U) {
		return 1U;
	}

	return (unsigned int)NHPOT(value);
}

static unsigned int hci_dma_ibi_chunk_size(struct i3c_hci *hci)
{
	unsigned int chunk_sz;

	if (hci->vendor && hci->vendor->ibi_chunk_size) {
		return hci->vendor->ibi_chunk_size(hci);
	}

	chunk_sz = (unsigned int)hci_dma_cache_align();
	chunk_sz = MAX(chunk_sz, 4U);
	chunk_sz = hci_dma_pow2_ceil(chunk_sz);

	return MIN(chunk_sz, 256U);
}

static unsigned int hci_dma_ring_space(struct i3c_hci_dma_ring *rh,
				       unsigned int enq_ptr)
{
	uint32_t op2_val = rh_reg_read(rh, RING_OPERATION2);
	unsigned int deq_ptr = FIELD_GET(RING_OP2_CR_DEQ_PTR, op2_val);

	if (enq_ptr >= deq_ptr) {
		return rh->xfer_entries - (enq_ptr - deq_ptr) - 1U;
	}

	return deq_ptr - enq_ptr - 1U;
}

static void hci_dma_unmap_one(struct hci_xfer *xfer)
{
	struct hci_dma_buf *dma;

	if (!xfer || !xfer->dma) {
		return;
	}

	dma = xfer->dma;
	if (dma->dir == HCI_DMA_FROM_DEVICE) {
		hci_dma_cache_invd(dma->buf, dma->len);
		if (dma->bounce && dma->orig) {
			(void)memcpy(dma->orig, dma->bounce, dma->len);
		}
	}

	if (dma->bounce) {
		k_free(dma->bounce);
	}
	k_free(dma);
	xfer->dma = NULL;
}

static void hci_dma_unmap_xfers(struct hci_xfer *xfer_list, unsigned int n)
{
	for (unsigned int i = 0; i < n; i++) {
		hci_dma_unmap_one(&xfer_list[i]);
	}
}

static int hci_dma_map_one(struct i3c_hci *hci, struct hci_xfer *xfer)
{
	struct hci_dma_buf *dma;
	uint64_t phys;

	if (!xfer->data || xfer->data_len == 0U) {
		xfer->dma = NULL;
		return 0;
	}

	dma = k_calloc(1, sizeof(*dma));
	if (!dma) {
		return -ENOMEM;
	}

	dma->buf = xfer->data;
	dma->addr = (uintptr_t)xfer->data;
	dma->len = xfer->data_len;
	dma->dir = xfer->rnw ? HCI_DMA_FROM_DEVICE : HCI_DMA_TO_DEVICE;
	phys = hci_dma_to_phys(hci, dma->addr);

	if (!hci_dma_phys_addr_visible(hci, phys, dma->len)) {
		size_t align = hci_dma_cache_align();
		size_t rounded = hci_dma_round_up(dma->len, align);
		void *bounce;
		uint64_t bphys;

		bounce = k_aligned_alloc(align, rounded);
		if (!bounce) {
			LOG_ERR("%s DMA bounce alloc failed for size %zu",
				hci->dev->name, dma->len);
			k_free(dma);
			return -ENOMEM;
		}

		bphys = hci_dma_to_phys(hci, (uintptr_t)bounce);
		if (!hci_dma_phys_addr_visible(hci, bphys, rounded)) {
			LOG_ERR("%s DMA bounce %p phys 0x%llx size %zu still not "
				"I3C DMA-visible",
				hci->dev->name, bounce, (unsigned long long)bphys, rounded);
			k_free(bounce);
			k_free(dma);
			return -ENOMEM;
		}

		if (dma->dir == HCI_DMA_TO_DEVICE) {
			(void)memcpy(bounce, xfer->data, dma->len);
		} else {
			(void)memset(bounce, 0, rounded);
		}

		dma->orig = xfer->data;
		dma->bounce = bounce;
		dma->buf = bounce;
		dma->addr = (uintptr_t)bounce;
	}

	if (dma->dir == HCI_DMA_TO_DEVICE) {
		hci_dma_cache_flush(dma->buf, dma->len);
	} else {
		hci_dma_cache_invd(dma->buf, dma->len);
	}

	xfer->dma = dma;
	return 0;
}

static int hci_dma_map_xfers(struct i3c_hci *hci, struct hci_xfer *xfer_list, int n)
{
	int ret;

	for (int i = 0; i < n; i++) {
		ret = hci_dma_map_one(hci, &xfer_list[i]);
		if (ret != 0) {
			hci_dma_unmap_xfers(xfer_list, (unsigned int)i);
			return ret;
		}
	}

	return 0;
}

static void hci_dma_write_noop(struct i3c_hci *hci, struct i3c_hci_dma_ring *rh,
			       struct hci_xfer *xfer, unsigned int index)
{
	uint8_t *entry = hci_dma_ring_entry(rh->cmd_ring, rh->xfer_struct_sz, index);
	uint32_t *ring_data = (uint32_t *)entry;
	unsigned int words = hci_dma_cmd_words(hci);
	uint32_t tid = xfer ? xfer->cmd_tid : 0U;

	(void)memset(entry, 0, rh->xfer_struct_sz);
	ring_data[0] = FIELD_PREP(CMD_0_ATTR, CMD_0_ATTR_M) | FIELD_PREP(CMD_0_TID, tid);

	for (unsigned int i = 1; i < words; i++) {
		ring_data[i] = 0U;
	}

	hci_dma_cache_flush(entry, rh->xfer_struct_sz);
}

static void hci_dma_write_xfer(struct i3c_hci *hci, struct i3c_hci_dma_ring *rh,
			       struct hci_xfer *xfer, unsigned int index, bool ioc)
{
	uint8_t *entry = hci_dma_ring_entry(rh->cmd_ring, rh->xfer_struct_sz, index);
	uint32_t *ring_data = (uint32_t *)entry;
	struct hci_dma_buf *dma = xfer->dma;
	unsigned int words = hci_dma_cmd_words(hci);
	unsigned int data_len = dma ? xfer->data_len : 0U;
	uintptr_t data_addr = dma ? dma->addr : 0U;

	(void)memset(entry, 0, rh->xfer_struct_sz);

	for (unsigned int i = 0; i < words; i++) {
		ring_data[i] = xfer->cmd_desc[i];
	}

	ring_data[words] = FIELD_PREP(DATA_BUF_BLOCK_SIZE, data_len) |
			   (ioc ? DATA_BUF_IOC : 0U);
	ring_data[words + 1U] = hci_dma_addr_lo(hci, data_addr);
	ring_data[words + 2U] = hci_dma_addr_hi(hci, data_addr);

	rh->src_xfers[index] = xfer;
	xfer->ring_entry = (int)index;
	hci_dma_cache_flush(entry, rh->xfer_struct_sz);
}

static int hci_dma_select_ring(struct i3c_dma *dma, int n, unsigned int *ring_id,
			       unsigned int *enqueue_ptr)
{
	for (unsigned int i = 0; i < dma->total; i++) {
		unsigned int idx = (dma->next_ring + i) % dma->total;
		struct i3c_hci_dma_ring *rh = &dma->rings[idx];
		uint32_t op1_val = rh_reg_read(rh, RING_OPERATION1);
		unsigned int enq = FIELD_GET(RING_OP1_CR_ENQ_PTR, op1_val);

		if (hci_dma_ring_space(rh, enq) >= (unsigned int)n) {
			*ring_id = idx;
			*enqueue_ptr = enq;
			dma->next_ring = (idx + 1U) % dma->total;
			return 0;
		}
	}

	return -EBUSY;
}

static void hci_dma_vendor_init(struct i3c_hci *hci)
{
	if (hci->vendor && hci->vendor->dma_init) {
		hci->vendor->dma_init(hci);
	}
}

static void hci_dma_vendor_log_status(struct i3c_hci *hci, const char *reason)
{
	if (hci->vendor && hci->vendor->dma_log_status) {
		hci->vendor->dma_log_status(hci, reason);
	}
}

static void hci_dma_vendor_recover_fifo(struct i3c_hci *hci)
{
	if (!hci->vendor || !hci->vendor->dma_drain ||
	    !hci->vendor->dma_recovery_done) {
		return;
	}

	hci->vendor->dma_drain(hci);
	(void)WAIT_FOR(hci->vendor->dma_recovery_done(hci),
		       MIPI_I3C_HCI_DMA_ABORT_TIMEOUT_US, k_busy_wait(1));
	mipi_i3c_hci_pio_ibi_reset(hci);
	mipi_i3c_hci_pio_reset(hci);
}

static void hci_dma_ring_disable(struct i3c_hci_dma_ring *rh)
{
	if (rh->regs == 0U) {
		return;
	}

	rh_reg_write(rh, INTR_SIGNAL_ENABLE, 0U);
	rh_reg_write(rh, RING_CONTROL, 0U);
	rh_reg_write(rh, CR_SETUP, 0U);
	rh_reg_write(rh, IBI_SETUP, 0U);
}

static void hci_dma_free_ring(struct i3c_hci_dma_ring *rh)
{
	k_free(rh->cmd_ring);
	k_free(rh->resp_ring);
	k_free(rh->src_xfers);
	k_free(rh->ibi_status_ring);
	k_free(rh->ibi_data_ring);

	(void)memset(rh, 0, sizeof(*rh));
}

static int hci_dma_alloc_target_rx(struct i3c_hci *hci, struct i3c_dma *dma,
				   size_t len)
{
	void *buf;

	if (!hci->is_target) {
		return 0;
	}

	if (hci->target_rx.buf && hci->target_rx.max_len >= len) {
		return 0;
	}

	buf = k_malloc(len);
	if (!buf) {
		return -ENOMEM;
	}

	k_free(hci->target_rx.buf);
	hci->target_rx.buf = buf;
	hci->target_rx.max_len = len;
	dma->target_rx_owned = true;

	return 0;
}

static int hci_dma_init_ring(struct i3c_hci *hci, struct i3c_dma *dma,
			     unsigned int ring_id, uint32_t offset)
{
	struct i3c_hci_dma_ring *rh = &dma->rings[ring_id];
	unsigned int required_xfer_sz = hci_dma_required_xfer_size(hci);
	unsigned int required_resp_sz = sizeof(uint32_t);
	size_t align = hci_dma_cache_align();
	uint32_t regval;
	size_t size;
	int ret;

	if (offset == 0U) {
		return -EINVAL;
	}

	rh->regs = hci->base_regs + offset;
	rh->xfer_entries = MIPI_I3C_HCI_DMA_XFER_RING_ENTRIES;
	rh->has_ibi = ring_id < MIPI_I3C_HCI_DMA_IBI_RINGS;
	k_sem_init(&rh->op_done, 0, 1);

	regval = rh_reg_read(rh, CR_SETUP);
	rh->xfer_struct_sz = FIELD_GET(CR_XFER_STRUCT_SIZE, regval);
	rh->resp_struct_sz = FIELD_GET(CR_RESP_STRUCT_SIZE, regval);
	rh->xfer_struct_sz = MAX(rh->xfer_struct_sz, required_xfer_sz);
	rh->resp_struct_sz = MAX(rh->resp_struct_sz, required_resp_sz);
	rh->xfer_struct_sz = ROUND_UP(rh->xfer_struct_sz, sizeof(uint32_t));
	rh->resp_struct_sz = ROUND_UP(rh->resp_struct_sz, sizeof(uint32_t));

	size = (size_t)rh->xfer_struct_sz * rh->xfer_entries;
	rh->cmd_ring = hci_dma_alloc_buf(hci, size, align, &rh->cmd_alloc_size);
	size = (size_t)rh->resp_struct_sz * rh->xfer_entries;
	rh->resp_ring = hci_dma_alloc_buf(hci, size, align, &rh->resp_alloc_size);
	rh->src_xfers = k_calloc(rh->xfer_entries, sizeof(*rh->src_xfers));
	if (!rh->cmd_ring || !rh->resp_ring || !rh->src_xfers) {
		return -ENOMEM;
	}

	rh->cmd_dma = (uintptr_t)rh->cmd_ring;
	rh->resp_dma = (uintptr_t)rh->resp_ring;
	rh_reg_write(rh, CMD_RING_BASE_LO, hci_dma_addr_lo(hci, rh->cmd_dma));
	rh_reg_write(rh, CMD_RING_BASE_HI, hci_dma_addr_hi(hci, rh->cmd_dma));
	rh_reg_write(rh, RESP_RING_BASE_LO, hci_dma_addr_lo(hci, rh->resp_dma));
	rh_reg_write(rh, RESP_RING_BASE_HI, hci_dma_addr_hi(hci, rh->resp_dma));

	regval = rh_reg_read(rh, CR_SETUP);
	regval &= CR_XFER_STRUCT_SIZE | CR_RESP_STRUCT_SIZE;
	regval |= FIELD_PREP(CR_RING_SIZE, rh->xfer_entries);
	rh_reg_write(rh, CR_SETUP, regval);

	/*
	 * Clear any stale interrupt status before enabling the ring.
	 * Vendors whose HCI latches pending bits on this write (and then
	 * fires a spurious TRANSFER_ERR once the ring is enabled) set
	 * HCI_QUIRK_RING_INTR_NO_PREWRITE to skip it.
	 */
	if ((hci->quirks & HCI_QUIRK_RING_INTR_NO_PREWRITE) == 0U) {
		rh_reg_write(rh, INTR_STATUS, 0xffffffffU);
	}
	rh_reg_write(rh, INTR_STATUS_ENABLE, 0xffffffffU);
	rh_reg_write(rh, INTR_SIGNAL_ENABLE, HCI_DMA_IBI_INTRS);

	if (rh->has_ibi) {
		regval = rh_reg_read(rh, IBI_SETUP);
		rh->ibi_status_sz = FIELD_GET(IBI_STATUS_STRUCT_SIZE, regval);
		rh->ibi_status_sz = MAX(rh->ibi_status_sz, (unsigned int)sizeof(uint32_t));
		rh->ibi_status_sz = ROUND_UP(rh->ibi_status_sz, sizeof(uint32_t));
		rh->ibi_status_entries = MIPI_I3C_HCI_DMA_IBI_STATUS_RING_ENTRIES;
		rh->ibi_chunks_total = MIPI_I3C_HCI_DMA_IBI_CHUNK_POOL_SIZE;
		rh->ibi_chunk_sz = hci_dma_ibi_chunk_size(hci);

		size = (size_t)rh->ibi_status_sz * rh->ibi_status_entries;
		rh->ibi_status_ring =
			hci_dma_alloc_buf(hci, size, align, &rh->ibi_status_alloc_size);
		size = (size_t)rh->ibi_chunk_sz * rh->ibi_chunks_total;
		rh->ibi_data_ring = hci_dma_alloc_buf(hci, size, align,
						      &rh->ibi_data_alloc_size);
		if (!rh->ibi_status_ring || !rh->ibi_data_ring) {
			return -ENOMEM;
		}

		ret = hci_dma_alloc_target_rx(hci, dma, rh->ibi_data_alloc_size);
		if (ret != 0) {
			return ret;
		}

		rh->ibi_status_dma = (uintptr_t)rh->ibi_status_ring;
		rh->ibi_data_dma = (uintptr_t)rh->ibi_data_ring;
		rh_reg_write(rh, IBI_STATUS_RING_BASE_LO,
			     hci_dma_addr_lo(hci, rh->ibi_status_dma));
		rh_reg_write(rh, IBI_STATUS_RING_BASE_HI,
			     hci_dma_addr_hi(hci, rh->ibi_status_dma));
		rh_reg_write(rh, IBI_DATA_RING_BASE_LO,
			     hci_dma_addr_lo(hci, rh->ibi_data_dma));
		rh_reg_write(rh, IBI_DATA_RING_BASE_HI,
			     hci_dma_addr_hi(hci, rh->ibi_data_dma));

		regval = rh_reg_read(rh, IBI_SETUP);
		regval &= IBI_STATUS_STRUCT_SIZE;
		regval |= FIELD_PREP(IBI_STATUS_RING_SIZE, rh->ibi_status_entries) |
			  FIELD_PREP(IBI_DATA_CHUNK_SIZE, LOG2(rh->ibi_chunk_sz) - 2) |
			  FIELD_PREP(IBI_DATA_CHUNK_COUNT, rh->ibi_chunks_total);
		rh_reg_write(rh, IBI_SETUP, regval);
	}

	rh_reg_write(rh, RING_OPERATION1, 0U);
	rh_reg_write(rh, RING_CONTROL, RING_CTRL_ENABLE | RING_CTRL_RUN_STOP);

	LOG_DBG("%s DMA ring %u offset %#x xfer %u-byte resp %u-byte ibi %s",
		hci->dev->name, ring_id, offset, rh->xfer_struct_sz, rh->resp_struct_sz,
		rh->has_ibi ? "enabled" : "disabled");

	return 0;
}

static int hci_dma_init(struct i3c_hci *hci)
{
	struct i3c_dma *dma;
	uint32_t regval;
	unsigned int nr_rings;
	size_t dma_size;
	int ret;

	if (!hci || hci->RHS_regs == 0U) {
		return -ENODEV;
	}

	regval = rhs_reg_read(hci, CONTROL);
	nr_rings = FIELD_GET(MAX_HEADER_COUNT_CAP, regval);
	if (nr_rings == 0U) {
		return -ENODEV;
	}

	nr_rings = MIN(nr_rings, MIPI_I3C_HCI_DMA_MAX_RINGS);
	dma_size = sizeof(*dma) + (nr_rings * sizeof(dma->rings[0]));
	dma = k_calloc(1, dma_size);
	if (!dma) {
		return -ENOMEM;
	}

	hci->io_data = dma;
	dma->total = nr_rings;

	regval &= ~MAX_HEADER_COUNT;
	regval |= FIELD_PREP(MAX_HEADER_COUNT, dma->total);
	rhs_reg_write(hci, CONTROL, regval);

	hci_reg_clear(hci, HC_CONTROL, HC_CONTROL_PIO_MODE);
	hci_dma_vendor_init(hci);

	for (unsigned int i = 0; i < dma->total; i++) {
		uint32_t offset = rhs_reg_read(hci, RHN_OFFSET(i));

		ret = hci_dma_init_ring(hci, dma, i, offset);
		if (ret != 0) {
			hci->io->cleanup(hci);
			return ret;
		}
	}

	return 0;
}

static void hci_dma_cleanup(struct i3c_hci *hci)
{
	struct i3c_dma *dma = hci ? hci->io_data : NULL;

	if (!dma) {
		return;
	}

	if (hci->vendor && hci->vendor->dma_drain) {
		hci->vendor->dma_drain(hci);
	}
	hci_reg_set(hci, HC_CONTROL, HC_CONTROL_PIO_MODE);

	for (unsigned int i = 0; i < dma->total; i++) {
		hci_dma_ring_disable(&dma->rings[i]);
		hci_dma_free_ring(&dma->rings[i]);
	}

	if (dma->target_rx_owned) {
		k_free(hci->target_rx.buf);
		hci->target_rx.buf = NULL;
		hci->target_rx.max_len = 0U;
	}

	rhs_reg_write(hci, CONTROL, 0U);
	k_free(dma);
	hci->io_data = NULL;
}

static int hci_dma_queue_xfer(struct i3c_hci *hci, struct hci_xfer *xfer_list, int n)
{
	struct i3c_dma *dma = hci ? hci->io_data : NULL;
	struct i3c_hci_dma_ring *rh;
	k_spinlock_key_t key;
	unsigned int ring;
	unsigned int enqueue_ptr;
	int ret;

	if (!dma || !xfer_list || n <= 0) {
		return -EINVAL;
	}

	if ((unsigned int)n >= MIPI_I3C_HCI_DMA_XFER_RING_ENTRIES) {
		return -EINVAL;
	}

	ret = hci_dma_map_xfers(hci, xfer_list, n);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&hci->lock);
	ret = hci_dma_select_ring(dma, n, &ring, &enqueue_ptr);
	if (ret != 0) {
		k_spin_unlock(&hci->lock, key);
		hci_dma_unmap_xfers(xfer_list, n);
		return ret;
	}

	rh = &dma->rings[ring];
	for (int i = 0; i < n; i++) {
		struct hci_xfer *xfer = &xfer_list[i];

		xfer->ring_number = (int)ring;
		hci_dma_write_xfer(hci, rh, xfer, enqueue_ptr, i == (n - 1));
		enqueue_ptr = (enqueue_ptr + 1U) % rh->xfer_entries;
	}

	uint32_t op1_val = rh_reg_read(rh, RING_OPERATION1);

	op1_val &= ~RING_OP1_CR_ENQ_PTR;
	op1_val |= FIELD_PREP(RING_OP1_CR_ENQ_PTR, enqueue_ptr);
	rh_reg_write(rh, RING_OPERATION1, op1_val);
	k_spin_unlock(&hci->lock, key);

	if (hci->vendor && hci->vendor->dma_start) {
		hci->vendor->dma_start(hci);
	}
	return 0;
}

static void hci_dma_abort_ring(struct i3c_hci *hci, struct i3c_hci_dma_ring *rh)
{
	uint32_t ring_status = rh_reg_read(rh, RING_STATUS);

	if ((ring_status & RING_STATUS_RUNNING) == 0U) {
		return;
	}

	k_sem_reset(&rh->op_done);
	rh_reg_write(rh, RING_CONTROL, RING_CTRL_ENABLE | RING_CTRL_ABORT);
	if (hci->vendor && hci->vendor->dma_drain) {
		hci->vendor->dma_drain(hci);
	}

	(void)k_sem_take(&rh->op_done, K_MSEC(1000));
	ring_status = rh_reg_read(rh, RING_STATUS);
	if (ring_status & RING_STATUS_RUNNING) {
		LOG_ERR("%s DMA ring at %#lx did not stop, status %#x",
			hci->dev->name, (unsigned long)rh->regs, ring_status);
	}
}

static bool hci_dma_dequeue_ring_xfers(struct i3c_hci *hci,
				       struct i3c_hci_dma_ring *rh,
				       struct hci_xfer *xfer_list, int n)
{
	bool did_unqueue = false;

	for (int i = 0; i < n; i++) {
		struct hci_xfer *xfer = &xfer_list[i];
		int idx = xfer->ring_entry;

		if (idx < 0 || xfer->ring_number < 0) {
			continue;
		}

		if ((unsigned int)idx >= rh->xfer_entries) {
			continue;
		}

		hci_dma_write_noop(hci, rh, xfer, (unsigned int)idx);
		rh->src_xfers[idx] = NULL;
		hci_dma_unmap_one(xfer);
		xfer->ring_entry = -1;
		did_unqueue = true;
	}

	return did_unqueue;
}

static bool hci_dma_dequeue_all_ring_xfers(struct i3c_hci *hci,
					   struct i3c_hci_dma_ring *rh)
{
	bool did_unqueue = false;

	for (unsigned int i = 0; i < rh->xfer_entries; i++) {
		struct hci_xfer *xfer = rh->src_xfers[i];

		if (!xfer) {
			continue;
		}

		hci_dma_write_noop(hci, rh, xfer, i);
		rh->src_xfers[i] = NULL;
		hci_dma_unmap_one(xfer);
		xfer->ring_entry = -1;
		did_unqueue = true;
	}

	return did_unqueue;
}

static void hci_dma_restart_ring(struct i3c_hci *hci, struct i3c_hci_dma_ring *rh)
{
	uint32_t op1_val = rh_reg_read(rh, RING_OPERATION1);
	unsigned int done_ptr = FIELD_GET(RING_OP1_CR_SW_DEQ_PTR, op1_val);

	op1_val &= ~RING_OP1_CR_ENQ_PTR;
	op1_val |= FIELD_PREP(RING_OP1_CR_ENQ_PTR, done_ptr);
	rh_reg_write(rh, RING_OPERATION1, op1_val);

	mipi_i3c_hci_resume(hci);
	rh_reg_write(rh, RING_CONTROL, RING_CTRL_ENABLE);
	rh_reg_write(rh, RING_CONTROL, RING_CTRL_ENABLE | RING_CTRL_RUN_STOP);
}

static bool hci_dma_dequeue_xfer(struct i3c_hci *hci, struct hci_xfer *xfer_list, int n)
{
	struct i3c_dma *dma = hci ? hci->io_data : NULL;
	bool did_unqueue = false;

	if (!dma) {
		return false;
	}

	if (!xfer_list || n <= 0) {
		for (unsigned int i = 0; i < dma->total; i++) {
			struct i3c_hci_dma_ring *rh = &dma->rings[i];

			hci_dma_abort_ring(hci, rh);
			did_unqueue |= hci_dma_dequeue_all_ring_xfers(hci, rh);
			hci_dma_restart_ring(hci, rh);
		}
		return did_unqueue;
	}

	if (xfer_list[0].ring_number < 0 ||
	    (unsigned int)xfer_list[0].ring_number >= dma->total) {
		return false;
	}

	struct i3c_hci_dma_ring *rh = &dma->rings[xfer_list[0].ring_number];

	hci_dma_abort_ring(hci, rh);
	did_unqueue = hci_dma_dequeue_ring_xfers(hci, rh, xfer_list, n);
	hci_dma_restart_ring(hci, rh);

	return did_unqueue;
}

static void hci_dma_complete_target_xfer(struct i3c_hci *hci, uint32_t resp)
{
	if (TARGET_RESP_STATUS(resp) == TARGET_RESP_SUCCESS &&
	    !TARGET_RESP_CCC_INDICATE(resp)) {
		if (TARGET_RESP_TID(resp) == TID_TARGET_IBI) {
			k_sem_give(&hci->ibi_comp);
		} else if (TARGET_RESP_TID(resp) == TID_TARGET_RD_DATA) {
			k_sem_give(&hci->pending_r_comp);
		}
	}

	if (TARGET_RESP_STATUS(resp) >= TARGET_RESP_ERR_CRC &&
	    TARGET_RESP_STATUS(resp) <= TARGET_RESP_ERR_I2C_READ_TOO_MUCH) {
		LOG_ERR("%s target DMA transfer error status %#x",
			hci->dev->name, (uint32_t)TARGET_RESP_STATUS(resp));
		mipi_i3c_hci_resume(hci);
	}
}

static void hci_dma_xfer_done(struct i3c_hci *hci, struct i3c_hci_dma_ring *rh)
{
	struct hci_xfer *done_xfers[MIPI_I3C_HCI_DMA_XFER_RING_ENTRIES];
	uint32_t done_resps[MIPI_I3C_HCI_DMA_XFER_RING_ENTRIES];
	unsigned int done_count = 0;
	unsigned int done_ptr;
	k_spinlock_key_t key;

	key = k_spin_lock(&hci->lock);
	done_ptr = rh->done_ptr;

	for (;;) {
		uint32_t op2_val = rh_reg_read(rh, RING_OPERATION2);
		unsigned int hw_deq = FIELD_GET(RING_OP2_CR_DEQ_PTR, op2_val);
		uint8_t *entry;
		uint32_t resp;
		struct hci_xfer *xfer;

		if (done_ptr == hw_deq) {
			break;
		}

		entry = hci_dma_ring_entry(rh->resp_ring, rh->resp_struct_sz, done_ptr);
		hci_dma_cache_invd(entry, rh->resp_struct_sz);
		resp = *(uint32_t *)entry;
		xfer = rh->src_xfers[done_ptr];
		rh->src_xfers[done_ptr] = NULL;

		if (xfer && done_count < ARRAY_SIZE(done_xfers)) {
			xfer->ring_entry = -1;
			xfer->response = resp;
			done_xfers[done_count] = xfer;
			done_resps[done_count] = resp;
			done_count++;
		}

		done_ptr = (done_ptr + 1U) % rh->xfer_entries;
		rh->done_ptr = done_ptr;
	}

	uint32_t op1_val = rh_reg_read(rh, RING_OPERATION1);

	op1_val &= ~RING_OP1_CR_SW_DEQ_PTR;
	op1_val |= FIELD_PREP(RING_OP1_CR_SW_DEQ_PTR, done_ptr);
	rh_reg_write(rh, RING_OPERATION1, op1_val);
	k_spin_unlock(&hci->lock, key);

	for (unsigned int i = 0; i < done_count; i++) {
		struct hci_xfer *xfer = done_xfers[i];
		uint32_t resp = done_resps[i];
		unsigned int tid = RESP_TID(resp);

		hci_dma_unmap_one(xfer);

		if (!hci->is_target && tid != xfer->cmd_tid) {
			LOG_ERR("%s DMA response TID %u expected %u",
				hci->dev->name, tid, xfer->cmd_tid);
		}

		if (hci->is_target) {
			hci_dma_complete_target_xfer(hci, resp);
		}

		if (xfer->completion) {
			k_sem_give(xfer->completion);
		}
	}
}

static int hci_dma_copy_ibi_data(struct i3c_hci_dma_ring *rh, unsigned int start_chunk,
				 void *dst, size_t len)
{
	size_t offset;
	size_t first_part;
	uint8_t *src;

	if (len == 0U) {
		return 0;
	}

	if (!dst || !rh->ibi_data_ring || rh->ibi_chunks_total == 0U) {
		return -EINVAL;
	}

	offset = (size_t)start_chunk * rh->ibi_chunk_sz;
	first_part = (rh->ibi_chunks_total - start_chunk) * rh->ibi_chunk_sz;
	first_part = MIN(first_part, len);

	src = rh->ibi_data_ring + offset;
	hci_dma_cache_invd(src, first_part);
	(void)memcpy(dst, src, first_part);

	if (len > first_part) {
		src = rh->ibi_data_ring;
		hci_dma_cache_invd(src, len - first_part);
		(void)memcpy((uint8_t *)dst + first_part, src, len - first_part);
	}

	return 0;
}

static int hci_dma_deliver_sir(struct i3c_hci *hci, struct i3c_hci_dma_ring *rh,
			       unsigned int start_chunk, int ibi_addr,
			       unsigned int ibi_size)
{
#if defined(CONFIG_I3C_USE_IBI)
	struct i3c_device_desc *target;
	struct i3c_hci_dev_data *dev_data;
	struct i3c_hci_dma_ibi_data *dev_ibi;
	uint8_t payload[MAX(CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE, 1)];
	int ret;

	target = i3c_dev_list_i3c_addr_find(&hci->common.attached_dev, (uint8_t)ibi_addr);
	if (!target) {
		LOG_ERR("%s IBI for unknown device %#x", hci->dev->name, ibi_addr);
		return -ENODEV;
	}

	dev_data = target->controller_priv;
	dev_ibi = dev_data ? dev_data->ibi_data : NULL;
	if (!dev_ibi) {
		LOG_ERR("%s IBI from %#x without IBI setup", hci->dev->name, ibi_addr);
		return -ENODEV;
	}

	if (ibi_size > dev_ibi->max_len ||
	    ibi_size > (unsigned int)CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE) {
		LOG_ERR("%s IBI payload too large: %u", hci->dev->name, ibi_size);
		return -EMSGSIZE;
	}

	ret = hci_dma_copy_ibi_data(rh, start_chunk, payload, ibi_size);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_ibi_work_enqueue_target_irq(target, payload, ibi_size);
	if (ret != 0) {
		LOG_ERR("%s failed to enqueue IBI from %#x: %d",
			hci->dev->name, ibi_addr, ret);
	}
	return ret;
#else
	ARG_UNUSED(hci);
	ARG_UNUSED(rh);
	ARG_UNUSED(start_chunk);
	ARG_UNUSED(ibi_addr);
	ARG_UNUSED(ibi_size);
	return -ENOTSUP;
#endif
}

static void hci_dma_deliver_controller_role_request(struct i3c_hci *hci, int ibi_addr)
{
	struct i3c_device_desc *target;

	target = i3c_dev_list_i3c_addr_find(&hci->common.attached_dev, (uint8_t)ibi_addr);
	if (!target) {
		LOG_WRN("%s controller-role request from unknown device %#x",
			hci->dev->name, ibi_addr);
		return;
	}

#if defined(CONFIG_I3C_IBI_WORKQUEUE)
	struct i3c_ibi_work work = {
		.type = I3C_IBI_CONTROLLER_ROLE_REQUEST,
		.target = target,
	};

	(void)i3c_ibi_work_enqueue(&work);
#else
	LOG_WRN("%s dropping controller-role request from %#x", hci->dev->name, ibi_addr);
#endif
}

static bool hci_dma_submit_hotjoin(struct i3c_hci *hci)
{
#if defined(CONFIG_I3C_IBI_WORKQUEUE)
	return i3c_ibi_work_enqueue_hotjoin(hci->dev) == 0;
#else
	return k_work_submit(&hci->hj_work) >= 0;
#endif
}

static void hci_dma_process_target_rx(struct i3c_hci *hci, struct i3c_hci_dma_ring *rh,
				      unsigned int start_chunk, uint32_t status,
				      unsigned int ibi_size)
{
	int ret;

	if (ibi_size > hci->target_rx.max_len) {
		LOG_ERR("%s target private write too large: %u", hci->dev->name, ibi_size);
		return;
	}

	ret = hci_dma_copy_ibi_data(rh, start_chunk, hci->target_rx.buf, ibi_size);
	if (ret != 0) {
		LOG_ERR("%s failed to copy target private write: %d", hci->dev->name, ret);
		return;
	}

	if (TARGET_RESP_CCC_INDICATE(status)) {
		if (hci->vendor && hci->vendor->ccc_handler) {
			hci->vendor->ccc_handler(hci, TARGET_RESP_CCC_HDR(status));
		}
	} else if (ibi_size != 0U) {
		mipi_i3c_hci_target_rx_data(hci, hci->target_rx.buf, ibi_size);
	}
}

static void hci_dma_process_ibi(struct i3c_hci *hci, struct i3c_hci_dma_ring *rh)
{
	uint32_t op1_val;
	uint32_t op2_val;
	uint32_t last_status = 0U;
	uint32_t ibi_status_error = 0U;
	unsigned int start_chunk;
	unsigned int deq_ptr;
	unsigned int enq_ptr;
	unsigned int ptr;
	unsigned int ibi_chunks = 0U;
	unsigned int ibi_size = 0U;
	int ibi_addr = -1;
	int last_ptr = -1;
	bool ibi_rnw = false;
	k_spinlock_key_t key;

	if (!rh->has_ibi) {
		return;
	}

	key = k_spin_lock(&hci->lock);
	op1_val = rh_reg_read(rh, RING_OPERATION1);
	deq_ptr = FIELD_GET(RING_OP1_IBI_DEQ_PTR, op1_val);
	op2_val = rh_reg_read(rh, RING_OPERATION2);
	enq_ptr = FIELD_GET(RING_OP2_IBI_ENQ_PTR, op2_val);
	start_chunk = rh->ibi_chunk_ptr;

	for (ptr = deq_ptr; ptr != enq_ptr; ptr = (ptr + 1U) % rh->ibi_status_entries) {
		uint8_t *entry = hci_dma_ring_entry(rh->ibi_status_ring,
						    rh->ibi_status_sz, ptr);
		uint32_t status;
		unsigned int chunks;

		hci_dma_cache_invd(entry, rh->ibi_status_sz);
		status = *(uint32_t *)entry;
		last_status = status;

		if (hci->is_target) {
			unsigned int nbytes = TARGET_RESP_DATA_LENGTH(status);

			/*
			 * TARGET_RESP_XFER_TYPE is target-centric: TYPE_R means
			 * the target received data from the controller (so we
			 * need to account for the incoming RX chunks).
			 */
			if (TARGET_RESP_XFER_TYPE(status) == TARGET_RESP_XFER_TYPE_R) {
				chunks = DIV_ROUND_UP(nbytes, rh->ibi_chunk_sz);
				ibi_chunks += chunks;
				ibi_size += nbytes;
			}
			last_ptr = (int)ptr;
			break;
		}

		if (ibi_status_error != 0U) {
			/* Keep consuming descriptors until LAST_STATUS. */
		} else if (status & IBI_ERROR) {
			ibi_status_error = status;
		} else if (ibi_addr == -1) {
			ibi_addr = FIELD_GET(IBI_TARGET_ADDR, status);
		} else if (ibi_addr != (int)FIELD_GET(IBI_TARGET_ADDR, status)) {
			ibi_status_error = status;
		}

		ibi_rnw = FIELD_GET(IBI_TARGET_RNW, status) != 0U;
		chunks = FIELD_GET(IBI_CHUNKS, status);
		ibi_chunks += chunks;

		if ((status & IBI_LAST_STATUS) == 0U) {
			ibi_size += chunks * rh->ibi_chunk_sz;
			continue;
		}

		if (chunks != 0U) {
			ibi_size += (chunks - 1U) * rh->ibi_chunk_sz;
			ibi_size += FIELD_GET(IBI_DATA_LENGTH, status);
		}
		last_ptr = (int)ptr;
		break;
	}

	if (last_ptr < 0) {
		k_spin_unlock(&hci->lock, key);
		return;
	}

	deq_ptr = ((unsigned int)last_ptr + 1U) % rh->ibi_status_entries;
	op1_val = rh_reg_read(rh, RING_OPERATION1);
	op1_val &= ~RING_OP1_IBI_DEQ_PTR;
	op1_val |= FIELD_PREP(RING_OP1_IBI_DEQ_PTR, deq_ptr);
	rh_reg_write(rh, RING_OPERATION1, op1_val);
	rh->ibi_chunk_ptr = (rh->ibi_chunk_ptr + ibi_chunks) % rh->ibi_chunks_total;
	k_spin_unlock(&hci->lock, key);

	if (hci->is_target) {
		hci_dma_process_target_rx(hci, rh, start_chunk, last_status, ibi_size);
	} else if (ibi_status_error != 0U) {
		LOG_ERR("%s IBI error from %#x: %#x", hci->dev->name,
			ibi_addr, ibi_status_error);
	} else if (IBI_TYPE_HJ(ibi_addr, ibi_rnw)) {
		if (!hci_dma_submit_hotjoin(hci)) {
			LOG_ERR("%s failed to enqueue hot-join work", hci->dev->name);
		}
	} else if (IBI_TYPE_CR(ibi_addr, ibi_rnw)) {
		hci_dma_deliver_controller_role_request(hci, ibi_addr);
	} else {
		(void)hci_dma_deliver_sir(hci, rh, start_chunk, ibi_addr, ibi_size);
	}

	if (ibi_chunks != 0U) {
		rh_reg_write(rh, CHUNK_CONTROL,
			     rh_reg_read(rh, CHUNK_CONTROL) + ibi_chunks);
	}
}

static int hci_dma_request_ibi(struct i3c_hci *hci, struct i3c_device_desc *target,
			       const struct i3c_ibi *request)
{
	struct i3c_hci_dev_data *dev_data;
	struct i3c_hci_dma_ibi_data *dev_ibi;
	int dat_idx = -1;

	if (!target || !request) {
		return -EINVAL;
	}

	dev_data = target->controller_priv;
	if (!dev_data) {
		return -ENODEV;
	}

	dev_ibi = k_calloc(1, sizeof(*dev_ibi));
	if (!dev_ibi) {
		return -ENOMEM;
	}

	dev_ibi->max_len = request->payload_len;
	if (dev_ibi->max_len == 0U) {
		dev_ibi->max_len = target->data_length.max_ibi;
	}
	dev_ibi->max_len = MIN(dev_ibi->max_len,
			       (unsigned int)CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE);
	dev_data->ibi_data = dev_ibi;

	if (hci->dat && hci->dat->get_index) {
		dat_idx = hci->dat->get_index(hci, target->dynamic_addr);
	}
	/*
	 * Set DAT_0_IBI_PAYLOAD whenever either the host explicitly asked
	 * for an IBI payload (dev_ibi->max_len > 0) or the target advertised
	 * IBI_PAYLOAD_HAS_DATA_BYTE in its BCR. See the equivalent comment
	 * in pio.c for why this also honours target->bcr.
	 */
	bool ibi_has_payload = (dev_ibi->max_len != 0U) ||
			       (target->bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE);
	if (dat_idx >= 0 && hci->dat) {
		if (ibi_has_payload && hci->dat->set_flags) {
			hci->dat->set_flags(hci, (unsigned int)dat_idx,
					    DAT_0_IBI_PAYLOAD, 0U);
		} else if (hci->dat->clear_flags) {
			hci->dat->clear_flags(hci, (unsigned int)dat_idx,
					      DAT_0_IBI_PAYLOAD, 0U);
		}
		if (hci->dat->clear_flags) {
			hci->dat->clear_flags(hci, (unsigned int)dat_idx,
					      DAT_0_SIR_REJECT, 0U);
		}
	}

	if (hci->vendor && hci->vendor->set_ibi_terminate_len) {
		hci->vendor->set_ibi_terminate_len(hci, dev_ibi->max_len);
	}

	return 0;
}

static void hci_dma_free_ibi(struct i3c_hci *hci, struct i3c_device_desc *target)
{
	struct i3c_hci_dev_data *dev_data;
	int dat_idx = -1;

	if (!target) {
		return;
	}

	dev_data = target->controller_priv;
	if (!dev_data) {
		return;
	}

	if (hci->dat && hci->dat->get_index) {
		dat_idx = hci->dat->get_index(hci, target->dynamic_addr);
	}
	if (dat_idx >= 0 && hci->dat) {
		if (hci->dat->set_flags) {
			hci->dat->set_flags(hci, (unsigned int)dat_idx,
					    DAT_0_SIR_REJECT, 0U);
		}
		if (hci->dat->clear_flags) {
			hci->dat->clear_flags(hci, (unsigned int)dat_idx,
					      DAT_0_IBI_PAYLOAD, 0U);
		}
	}

	k_free(dev_data->ibi_data);
	dev_data->ibi_data = NULL;
}

static int hci_dma_request_hj(struct i3c_hci *hci)
{
	if (!hci) {
		return -EINVAL;
	}

	mipi_i3c_hci_hj_ctrl(hci, true);
	return 0;
}

static void hci_dma_free_hj(struct i3c_hci *hci)
{
	if (hci) {
		mipi_i3c_hci_hj_ctrl(hci, false);
	}
}

static void hci_dma_recycle_ibi_slot(struct i3c_hci *hci,
				     struct i3c_device_desc *target,
				     struct i3c_ibi_payload *payload)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(target);
	ARG_UNUSED(payload);
}

static bool hci_dma_irq_handler(struct i3c_hci *hci)
{
	struct i3c_dma *dma = hci ? hci->io_data : NULL;
	bool handled = false;

	if (!dma) {
		return false;
	}

	for (unsigned int i = 0; i < dma->total; i++) {
		struct i3c_hci_dma_ring *rh = &dma->rings[i];
		uint32_t status = rh_reg_read(rh, INTR_STATUS);

		if (status == 0U) {
			continue;
		}

		rh_reg_write(rh, INTR_STATUS, status);

		if (status & INTR_IBI_READY) {
			hci_dma_process_ibi(hci, rh);
		}

		if (status & (INTR_TRANSFER_COMPLETION | INTR_TRANSFER_ERR)) {
			hci_dma_xfer_done(hci, rh);
			if (status & INTR_TRANSFER_ERR) {
				LOG_WRN("%s DMA ring %u transfer error", hci->dev->name, i);
				hci_dma_vendor_log_status(hci, "transfer error");
				hci_dma_vendor_recover_fifo(hci);
				mipi_i3c_hci_resume(hci);
			}
		}

		if (status & INTR_RING_OP) {
			k_sem_give(&rh->op_done);
		}

		if (status & INTR_TRANSFER_ABORT) {
			uint32_t ring_status = rh_reg_read(rh, RING_STATUS);

			LOG_WRN("%s DMA ring %u transfer aborted", hci->dev->name, i);
			hci_dma_vendor_log_status(hci, "transfer abort");
			hci_dma_vendor_recover_fifo(hci);
			mipi_i3c_hci_resume(hci);

			if ((ring_status & RING_STATUS_RUNNING) == 0U &&
			    (status & INTR_TRANSFER_COMPLETION) != 0U &&
			    (status & INTR_TRANSFER_ERR) != 0U) {
				rh_reg_write(rh, RING_CONTROL, RING_CTRL_ENABLE);
				rh_reg_write(rh, RING_CONTROL,
					     RING_CTRL_ENABLE | RING_CTRL_RUN_STOP);
			}
		}

		if (status & INTR_IBI_RING_FULL) {
			LOG_ERR("%s DMA ring %u IBI ring full", hci->dev->name, i);
		}

		handled = true;
	}

	return handled;
}

const struct hci_io_ops mipi_i3c_hci_dma = {
	.irq_handler = hci_dma_irq_handler,
	.queue_xfer = hci_dma_queue_xfer,
	.dequeue_xfer = hci_dma_dequeue_xfer,
	.request_ibi = hci_dma_request_ibi,
	.free_ibi = hci_dma_free_ibi,
	.request_hj = hci_dma_request_hj,
	.free_hj = hci_dma_free_hj,
	.recycle_ibi_slot = hci_dma_recycle_ibi_slot,
	.init = hci_dma_init,
	.cleanup = hci_dma_cleanup,
};
