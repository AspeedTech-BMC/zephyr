/*
 * Copyright (c) 2020, MIPI Alliance, Inc.
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Ported to Zephyr from the Linux mipi-i3c-hci driver
 * (drivers/i3c/master/mipi-i3c-hci/).
 */

#ifndef ZEPHYR_DRIVERS_I3C_MIPI_HCI_HCI_H_
#define ZEPHYR_DRIVERS_I3C_MIPI_HCI_HCI_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/* 32-bit word aware bit and mask macros from the Linux HCI driver. */
#define W0_MASK(h, l) GENMASK((h) - 0, (l) - 0)
#define W1_MASK(h, l) GENMASK((h) - 32, (l) - 32)
#define W2_MASK(h, l) GENMASK((h) - 64, (l) - 64)
#define W3_MASK(h, l) GENMASK((h) - 96, (l) - 96)

#define W0_BIT_(x) BIT((x) - 0)
#define W1_BIT_(x) BIT((x) - 32)
#define W2_BIT_(x) BIT((x) - 64)
#define W3_BIT_(x) BIT((x) - 96)

#define hci_reg_read(hci, reg) sys_read32((mem_addr_t)((hci)->base_regs + (reg)))
#define hci_reg_write(hci, reg, val) \
	sys_write32((uint32_t)(val), (mem_addr_t)((hci)->base_regs + (reg)))
#define hci_reg_set(hci, reg, val) hci_reg_write(hci, reg, hci_reg_read(hci, reg) | (val))
#define hci_reg_clear(hci, reg, val) hci_reg_write(hci, reg, hci_reg_read(hci, reg) & ~(val))

/*
 * Host Controller Capabilities and Operation Registers.
 *
 * These offsets are shared by the core and quirk helpers.
 */
#define HCI_VERSION 0x00

#define HC_CONTROL 0x04
#define HC_CONTROL_BUS_ENABLE BIT(31)
#define HC_CONTROL_RESUME BIT(30)
#define HC_CONTROL_ABORT BIT(29)
#define HC_CONTROL_HALT_ON_CMD_TIMEOUT BIT(12)
#define HC_CONTROL_HOT_JOIN_CTRL BIT(8)
#define HC_CONTROL_I2C_TARGET_PRESENT BIT(7)
#define HC_CONTROL_PIO_MODE BIT(6)
#define HC_CONTROL_DATA_BIG_ENDIAN BIT(4)
#define HC_CONTROL_IBA_INCLUDE BIT(0)

#define MASTER_DEVICE_ADDR 0x08
#define MASTER_DYNAMIC_ADDR_VALID BIT(31)
#define MASTER_DYNAMIC_ADDR(v) FIELD_PREP(GENMASK(22, 16), v)

#define HC_CAPABILITIES 0x0c
#define HC_CAP_SG_DC_EN BIT(30)
#define HC_CAP_SG_IBI_EN BIT(29)
#define HC_CAP_SG_CR_EN BIT(28)
#define HC_CAP_MAX_DATA_LENGTH GENMASK(24, 22)
#define HC_CAP_CMD_SIZE GENMASK(21, 20)
#define HC_CAP_DIRECT_COMMANDS_EN BIT(18)
#define HC_CAP_MULTI_LANE_EN BIT(15)
#define HC_CAP_CMD_CCC_DEFBYTE BIT(10)
#define HC_CAP_HDR_BT_EN BIT(8)
#define HC_CAP_HDR_TS_EN BIT(7)
#define HC_CAP_HDR_DDR_EN BIT(6)
#define HC_CAP_NON_CURRENT_MASTER_CAP BIT(5)
#define HC_CAP_DATA_BYTE_CFG_EN BIT(4)
#define HC_CAP_AUTO_COMMAND BIT(3)
#define HC_CAP_COMBO_COMMAND BIT(2)

#define I3C_HCI_DAT_BITMAP_BITS 128U
#define I3C_HCI_DAT_BITMAP_WORDS DIV_ROUND_UP(I3C_HCI_DAT_BITMAP_BITS, 32U)
#define I3C_HCI_AUTOCMD_SLOTS 8U
#define I3C_HCI_AUTOCMD_TRIGGERS 128U

#define RESET_CONTROL 0x10
#define BUS_RESET BIT(31)
#define BUS_RESET_TYPE GENMASK(30, 29)
#define IBI_QUEUE_RST BIT(5)
#define RX_FIFO_RST BIT(4)
#define TX_FIFO_RST BIT(3)
#define RESP_QUEUE_RST BIT(2)
#define CMD_QUEUE_RST BIT(1)
#define SOFT_RST BIT(0)

#define PRESENT_STATE 0x14
#define STATE_CURRENT_MASTER BIT(2)

#define INTR_STATUS 0x20
#define INTR_STATUS_ENABLE 0x24
#define INTR_SIGNAL_ENABLE 0x28
#define INTR_FORCE 0x2c
#define INTR_HC_CMD_SEQ_UFLOW_STAT BIT(12)
#define INTR_HC_SEQ_CANCEL BIT(11)
#define INTR_HC_INTERNAL_ERR BIT(10)

#define DAT_SECTION 0x30
#define DAT_ENTRY_SIZE GENMASK(31, 28)
#define DAT_TABLE_SIZE GENMASK(18, 12)
#define DAT_TABLE_OFFSET GENMASK(11, 0)

#define DCT_SECTION 0x34
#define DCT_ENTRY_SIZE GENMASK(31, 28)
#define DCT_TABLE_INDEX GENMASK(23, 19)
#define DCT_TABLE_SIZE GENMASK(18, 12)
#define DCT_TABLE_OFFSET GENMASK(11, 0)

#define RING_HEADERS_SECTION 0x38
#define RING_HEADERS_OFFSET GENMASK(15, 0)

#define PIO_SECTION 0x3c
#define PIO_REGS_OFFSET GENMASK(15, 0)

#define EXT_CAPS_SECTION 0x40
#define EXT_CAPS_OFFSET GENMASK(15, 0)

#define IBI_NOTIFY_CTRL 0x58
#define IBI_NOTIFY_SIR_REJECTED BIT(3)
#define IBI_NOTIFY_MR_REJECTED BIT(1)
#define IBI_NOTIFY_HJ_REJECTED BIT(0)

#define DEV_CTX_BASE_LO 0x60
#define DEV_CTX_BASE_HI 0x64

struct hci_cmd_ops;
struct hci_dat_ops;
struct hci_xfer;
struct hci_io_ops;
struct i3c_hci;
struct i3c_ibi;
struct i3c_ibi_payload;

#define MIPI_I3C_HCI_VENDOR_IRQ_CORE BIT(0)
#define MIPI_I3C_HCI_VENDOR_IRQ_IO BIT(1)
#define MIPI_I3C_HCI_VENDOR_IRQ_PRIV BIT(2)

enum mipi_i3c_hci_target_event {
	MIPI_I3C_HCI_TARGET_EVENT_IBI,
	MIPI_I3C_HCI_TARGET_EVENT_HOTJOIN,
	MIPI_I3C_HCI_TARGET_EVENT_MASTER_REQUEST,
};

struct mipi_i3c_hci_vendor_ops {
	int (*init)(struct i3c_hci *hci);
	int (*autocmd_init)(struct i3c_hci *hci);
	uint32_t (*get_status)(struct i3c_hci *hci);
	void (*ccc_handler)(struct i3c_hci *hci, uint8_t ccc);
	void (*set_ibi_terminate_len)(struct i3c_hci *hci, uint16_t max_len);
	void (*set_slv_pid)(struct i3c_hci *hci, uint64_t pid);
	void (*set_slv_char_ctrl)(struct i3c_hci *hci, uint8_t bcr,
				  uint8_t dcr, bool static_addr_en);
	void (*dma_drain)(struct i3c_hci *hci);
	/*
	 * Optional. Returns true if STAT_IBI_STATUS_THLD on this silicon
	 * reflects only the IBI status FIFO -- i.e. after prep_new_ibi pops
	 * the status word the bit clears even though the IBI data FIFO
	 * still has the segment payload. When true, hci_pio_get_ibi_segment()
	 * reads data words directly using the segment length advertised by
	 * the status word instead of waiting for STATUS_THLD between reads,
	 * and hci_pio_set_ibi_thresh() lowers the IBI data threshold to the
	 * same value as the status threshold so STATUS_THLD still asserts
	 * for small payloads. Leave NULL on silicon whose STATUS_THLD tracks
	 * the IBI data FIFO too.
	 */
	bool (*pio_ibi_thld_status_only)(struct i3c_hci *hci);
	/*
	 * Called once per DAA iteration with the dynamic address that the
	 * controller is about to assign. Some silicon (e.g. ASPEED G7)
	 * needs to announce the address to a vendor-private register
	 * before the ENTDAA command goes on the bus.
	 */
	void (*prep_daa_step)(struct i3c_hci *hci, uint8_t dynamic_addr);

	/*
	 * Optional preferred DAT slot for the given dynamic address.
	 * Return -1 to fall back to the generic first-free allocator.
	 * Silicon whose DAT is indexed by dynamic address (e.g. ASPEED
	 * G7 requires dat_idx == dynamic address) returns `address` here.
	 */
	int (*dat_pick_slot)(struct i3c_hci *hci, uint8_t address);

	/*
	 * Returns true if the DAT entry at the slot returned by
	 * alloc_entry should be relocated to `dat_idx == address` before
	 * the dynamic-address field is written. Return true on silicon
	 * that indexes the DAT table by dynamic address (e.g. ASPEED G7).
	 */
	bool (*dat_wants_addr_indexed)(struct i3c_hci *hci);
	uint32_t (*ring_status)(struct i3c_hci *hci);
	uint32_t (*read_irq_summary)(struct i3c_hci *hci);
	uint32_t (*read_priv_irq_status)(struct i3c_hci *hci);
	void (*clear_priv_irq_status)(struct i3c_hci *hci, uint32_t status);
	void (*renew_irq)(struct i3c_hci *hci);
	void (*disable_priv_irq)(struct i3c_hci *hci);
	void (*handle_priv_irq)(struct i3c_hci *hci, uint32_t status);

	void (*dma_start)(struct i3c_hci *hci);
	void (*dma_init)(struct i3c_hci *hci);
	void (*dma_log_status)(struct i3c_hci *hci, const char *reason);
	bool (*dma_recovery_done)(struct i3c_hci *hci);
	unsigned int (*ibi_chunk_size)(struct i3c_hci *hci);
	/*
	 * Optional CPU-to-bus address translation for DMA descriptors and
	 * ring buffers. Implement when the DMA master sees memory through
	 * a remap window so CPU addresses differ from bus addresses.
	 * Leave NULL for identity-mapped systems.
	 */
	uint64_t (*dma_to_phys)(struct i3c_hci *hci, uintptr_t addr);
	/*
	 * Optional reachability check for a bus address range. Return
	 * false when the DMA master cannot access [phys, phys + len);
	 * the DMA layer then refuses the ring allocation or bounces the
	 * transfer buffer. Leave NULL when all memory is DMA-visible.
	 */
	bool (*dma_addr_visible)(struct i3c_hci *hci, uint64_t phys, size_t len);
	void (*pio_log_prog_error)(struct i3c_hci *hci, uint32_t status);
	bool (*status_changed_role)(struct i3c_hci *hci, uint32_t old_status,
				    uint32_t status);
	uint64_t (*target_pid)(struct i3c_hci *hci, uint16_t extra_info);
	uint8_t (*target_dynamic_addr)(struct i3c_hci *hci);
	bool (*target_event_enabled)(struct i3c_hci *hci,
				     enum mipi_i3c_hci_target_event event);
	int (*target_request_event)(struct i3c_hci *hci,
				    enum mipi_i3c_hci_target_event event);
	bool (*target_request_pending)(struct i3c_hci *hci,
				       enum mipi_i3c_hci_target_event event);
	void (*target_enable_events)(struct i3c_hci *hci, uint8_t hdr_caps);
	void (*target_set_mode)(struct i3c_hci *hci);
	bool (*target_get_role)(struct i3c_hci *hci, bool *secondary, bool *target);
};

struct i3c_hci_config {
	struct i3c_driver_config common;
	uintptr_t base_regs;
	const struct pinctrl_dev_config *pcfg;
	const char *io_mode;
	unsigned int quirks;
	void (*irq_config_func)(const struct device *dev);
	/*
	 * Per-instance vendor ops chosen at build time from the DT
	 * compatible list (see MIPI_I3C_HCI_PICK_VENDOR in core.c). NULL
	 * for plain MIPI I3C HCI silicon with no vendor extensions.
	 */
	const struct mipi_i3c_hci_vendor_ops *vendor;
};

/*
 * Main HCI runtime object. This is Zephyr driver data, so common must remain
 * the first field for the I3C subsystem helpers.
 */
struct i3c_hci {
	struct i3c_driver_data common;
	const struct device *dev;
	const struct i3c_hci_config *config;

	struct reset_dt_spec rst;
	struct reset_dt_spec dma_rst;
	const struct device *clk;
	clock_control_subsys_t clock_id;

	uintptr_t base_regs;
	uintptr_t DAT_regs;
	uintptr_t DCT_regs;
	uintptr_t RHS_regs;
	uintptr_t PIO_regs;
	uintptr_t EXTCAPS_regs;
	/*
	 * Vendor-private register windows discovered from the
	 * extended-capability list. Only the matching vendor backend
	 * knows their layout; common code never dereferences them and
	 * only tests for presence.
	 */
	uintptr_t VENDOR_regs;
	uintptr_t PHY_regs;
	uintptr_t AUTOCMD_regs;
	uintptr_t DEBUG_regs;

	const struct hci_io_ops *io;
	void *io_data;
	const struct hci_cmd_ops *cmd;
	const struct hci_dat_ops *dat;
	const struct mipi_i3c_hci_vendor_ops *vendor;

	struct k_spinlock lock;
	struct k_mutex control_mutex;
	atomic_t next_cmd_tid;
	uint32_t i2c_config;
	uint32_t caps;
	unsigned int quirks;
	unsigned int DAT_entries;
	unsigned int DAT_entry_size;
	void *DAT_data;
	uint32_t DAT_bitmap[I3C_HCI_DAT_BITMAP_WORDS];
	unsigned int DCT_entries;
	unsigned int DCT_entry_size;
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t revision;
	uint32_t vendor_mipi_id;
	uint32_t vendor_version_id;
	uint32_t vendor_product_id;
	void *vendor_data;
	struct k_sem ibi_comp;
	struct k_sem pending_r_comp;
	struct k_work hj_work;
	struct k_work halt_rst_work;

	/* Used for later target private-write handling. */
	struct {
		void *buf;
		uint16_t max_len;
	} target_rx;
	struct i3c_target_config *target_cb;
	bool is_target;
	bool is_secondary;

#if defined(CONFIG_I3C_MIPI_HCI_ASPEED_AUTOCMD)
	/*
	 * Auto-command slot bookkeeping. Owned and managed entirely by
	 * the ASPEED vendor backend; common code never touches it.
	 */
	struct {
		uint8_t installed;
		uint8_t enabled;
		uint32_t desc[I3C_HCI_AUTOCMD_SLOTS];
		uint8_t trigger_slot[I3C_HCI_AUTOCMD_TRIGGERS];
	} autocmd;
#endif
};

struct hci_xfer {
	uint32_t cmd_desc[4];
	uint32_t response;
	bool rnw;
	void *data;
	unsigned int data_len;
	unsigned int cmd_tid;
	struct k_sem *completion;
	union {
		struct {
			struct hci_xfer *next_xfer;
			struct hci_xfer *next_data;
			struct hci_xfer *next_resp;
			unsigned int data_left;
			uint32_t data_word_before_partial;
		};
		struct {
			void *dma;
			int ring_number;
			int ring_entry;
		};
	};
};

static inline struct hci_xfer *hci_alloc_xfer(unsigned int n)
{
	return k_calloc(n, sizeof(struct hci_xfer));
}

static inline void hci_free_xfer(struct hci_xfer *xfer, unsigned int n)
{
	ARG_UNUSED(n);
	k_free(xfer);
}

/* PIO vs DMA abstraction. The operations are stubs in this skeleton. */
struct hci_io_ops {
	bool (*irq_handler)(struct i3c_hci *hci);
	int (*queue_xfer)(struct i3c_hci *hci, struct hci_xfer *xfer, int n);
	bool (*dequeue_xfer)(struct i3c_hci *hci, struct hci_xfer *xfer, int n);
	int (*request_ibi)(struct i3c_hci *hci, struct i3c_device_desc *target,
			   const struct i3c_ibi *request);
	void (*free_ibi)(struct i3c_hci *hci, struct i3c_device_desc *target);
	int (*request_hj)(struct i3c_hci *hci);
	void (*free_hj)(struct i3c_hci *hci);
	void (*recycle_ibi_slot)(struct i3c_hci *hci, struct i3c_device_desc *target,
				 struct i3c_ibi_payload *payload);
	int (*init)(struct i3c_hci *hci);
	void (*cleanup)(struct i3c_hci *hci);
};

extern const struct hci_io_ops mipi_i3c_hci_pio;
extern const struct hci_io_ops mipi_i3c_hci_dma;

struct i3c_hci_dev_data {
	int dat_idx;
	void *ibi_data;
};

#define HCI_QUIRK_RAW_CCC BIT(1)
#define HCI_QUIRK_PIO_MODE BIT(2)
#define HCI_QUIRK_OD_PP_TIMING BIT(3)
#define HCI_QUIRK_RESP_BUF_THLD BIT(4)
/*
 * Skip the INTR_STATUS pre-write during DMA ring init. On some HCIs
 * (e.g. ASPEED G7) the write forces pending bits and fires a spurious
 * TRANSFER_ERR right after the ring is enabled.
 */
#define HCI_QUIRK_RING_INTR_NO_PREWRITE BIT(5)

/*
 * Normalized completion events a vendor IRQ hook may report back to
 * the core instead of touching driver-internal synchronization state.
 */
enum mipi_i3c_hci_vendor_event {
	MIPI_I3C_HCI_EVENT_IBI_DONE,
	MIPI_I3C_HCI_EVENT_PENDING_READ_DONE,
	MIPI_I3C_HCI_EVENT_BUS_STUCK,
};

void mipi_i3c_hci_vendor_event(struct i3c_hci *hci, enum mipi_i3c_hci_vendor_event event);

void mipi_i3c_hci_resume(struct i3c_hci *hci);
void mipi_i3c_hci_pio_reset(struct i3c_hci *hci);
void mipi_i3c_hci_pio_ibi_reset(struct i3c_hci *hci);
void mipi_i3c_hci_dct_index_reset(struct i3c_hci *hci);
void amd_set_od_pp_timing(struct i3c_hci *hci);
void amd_set_resp_buf_thld(struct i3c_hci *hci);
void mipi_i3c_hci_hj_ctrl(struct i3c_hci *hci, bool ack_nack);
void mipi_i3c_hci_apply_quirks(struct i3c_hci *hci);

#if defined(CONFIG_I3C_MIPI_HCI_TARGET)
int mipi_i3c_hci_target_init(struct i3c_hci *hci);
int mipi_i3c_hci_target_register(const struct device *dev, struct i3c_target_config *cfg);
int mipi_i3c_hci_target_unregister(const struct device *dev, struct i3c_target_config *cfg);
int mipi_i3c_hci_target_tx_write(const struct device *dev, uint8_t *buf, uint16_t len);
int mipi_i3c_hci_target_ibi_raise(const struct device *dev, struct i3c_ibi *request);
int mipi_i3c_hci_target_ibi_enable(const struct device *dev,
				   struct i3c_device_desc *target);
int mipi_i3c_hci_target_ibi_disable(const struct device *dev,
				    struct i3c_device_desc *target);
int mipi_i3c_hci_target_pending_read_notify(const struct device *dev, uint8_t *buf,
					    uint16_t len, struct i3c_ibi *notifier);
void mipi_i3c_hci_target_rx_data(struct i3c_hci *hci, void *buf, unsigned int len);
void mipi_i3c_hci_target_dyn_addr_updated(struct i3c_hci *hci, uint8_t new_addr);
void mipi_i3c_hci_target_handle_defslvs(struct i3c_hci *hci, const void *payload,
					size_t payload_len);
void mipi_i3c_hci_target_role_updated(struct i3c_hci *hci);
#else
static inline int mipi_i3c_hci_target_init(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
	return 0;
}

static inline int mipi_i3c_hci_target_register(const struct device *dev,
					       struct i3c_target_config *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);
	return -ENOSYS;
}

static inline int mipi_i3c_hci_target_unregister(const struct device *dev,
						 struct i3c_target_config *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);
	return -ENOSYS;
}

static inline int mipi_i3c_hci_target_tx_write(const struct device *dev, uint8_t *buf,
					       uint16_t len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	return -ENOSYS;
}

static inline int mipi_i3c_hci_target_ibi_raise(const struct device *dev,
						struct i3c_ibi *request)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(request);
	return -ENOSYS;
}

static inline int mipi_i3c_hci_target_ibi_enable(const struct device *dev,
						 struct i3c_device_desc *target)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(target);
	return -ENOSYS;
}

static inline int mipi_i3c_hci_target_ibi_disable(const struct device *dev,
						  struct i3c_device_desc *target)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(target);
	return -ENOSYS;
}

static inline int mipi_i3c_hci_target_pending_read_notify(const struct device *dev,
							  uint8_t *buf, uint16_t len,
							  struct i3c_ibi *notifier)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	ARG_UNUSED(notifier);
	return -ENOSYS;
}

static inline void mipi_i3c_hci_target_rx_data(struct i3c_hci *hci, void *buf,
					       unsigned int len)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
}

static inline void mipi_i3c_hci_target_dyn_addr_updated(struct i3c_hci *hci,
							uint8_t new_addr)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(new_addr);
}

static inline void mipi_i3c_hci_target_handle_defslvs(struct i3c_hci *hci,
						      const void *payload,
						      size_t payload_len)
{
	ARG_UNUSED(hci);
	ARG_UNUSED(payload);
	ARG_UNUSED(payload_len);
}

static inline void mipi_i3c_hci_target_role_updated(struct i3c_hci *hci)
{
	ARG_UNUSED(hci);
}
#endif /* CONFIG_I3C_MIPI_HCI_TARGET */

#endif /* ZEPHYR_DRIVERS_I3C_MIPI_HCI_HCI_H_ */
