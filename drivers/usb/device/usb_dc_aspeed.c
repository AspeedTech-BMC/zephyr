/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_ast1030_udc

#include <drivers/usb/usb_dc.h>
#include <drivers/reset_control.h>
#include <stdio.h>
#include <usb/usb_device.h>
#include <cache.h>

#include "soc.h"
#define LOG_LEVEL	CONFIG_USB_DRIVER_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(usb_dc_aspeed);

/*************************************************************************************/
#define ASPEED_USB_CTRL 		0x00
#define ASPEED_USB_CONF 		0x04
#define ASPEED_USB_IER 			0x08
#define ASPEED_USB_ISR 			0x0C
#define ASPEED_USB_EP_ACK_IER 		0x10
#define ASPEED_USB_EP_NAK_IER 		0x14
#define ASPEED_USB_EP_ACK_ISR 		0x18
#define ASPEED_USB_EP_NAK_ISR 		0x1C
#define ASPEED_USB_DEV_RESET 		0x20
#define ASPEED_USB_USB_STS 		0x24
#define ASPEED_USB_EP_DATA 		0x28
#define ASPEED_USB_ISO_TX_FAIL		0x2C
#define ASPEED_USB_EP0_CTRL		0x30
#define ASPEED_USB_EP0_DATA_BUFF	0x34

#define ASPEED_USB_SETUP_DATA0		0x80
/*************************************************************************************/
#define ASPEED_EP_OFFSET		0x200

#define ASPEED_EP_CONFIG		0x00
#define ASPEED_EP_DMA_CTRL		0x04
#define ASPEED_EP_DMA_BUFF		0x08
#define ASPEED_EP_DMA_STS		0x0C

/*************************************************************************************/
/* ASPEED_USB_CTRL 			0x00 */
#define ROOT_PHY_CLK_EN			BIT(31)
#define ROOT_PHY_SELF_TEST_EN		BIT(25)
#define ROOT_DN_15K_EN			BIT(24)
#define ROOT_DP_15K_EN			BIT(23)
#define ROOT_FIFO_DYNP_EN		BIT(19)
#define ROOT_EP_LONG_DESC		BIT(18)
#define ROOT_ISO_IN_NULL_RESP		BIT(17)
#define ROOT_SPLIT_IN			BIT(16)
#define ROOT_LOOP_TEST_PASS		BIT(15)
#define ROOT_LOOP_TEST_FINISH		BIT(14)
#define ROOT_BIST_TEST_PASS		BIT(13)
#define ROOT_BIST_ON			BIT(12)
#define ROOT_PHY_RESET_DIS		BIT(11)
#define ROOT_TEST_MODE(x)		((x) << 8)
#define ROOT_FORCE_TIMER_HS		BIT(7)
#define ROOT_FORCE_HS			BIT(6)
#define ROOT_REMOTE_WAKEUP_12MS		BIT(5)
#define ROOT_REMOTE_WAKEUP_EN		BIT(4)
#define ROOT_AUTO_REMOTE_WAKEUP_EN	BIT(3)
#define ROOT_STOP_CLK_IN_SUPEND		BIT(2)
#define ROOT_UPSTREAM_FS		BIT(1)
#define ROOT_UPSTREAM_EN		BIT(0)

/*************************************************************************************/
/* ASPEED_USB_IER			0x08 */
/* ASPEED_USB_ISR			0x0C */
#define ISR_EP_NAK			BIT(17)
#define ISR_EP_ACK_STALL		BIT(16)
#define ISR_SUSPEND_RESUME		BIT(8)
#define ISR_BUS_SUSPEND 		BIT(7)
#define ISR_BUS_RESET 			BIT(6)
#define ISR_EP0_IN_DATA_NAK		BIT(4)
#define ISR_EP0_IN_ACK_STALL		BIT(3)
#define ISR_EP0_OUT_NAK			BIT(2)
#define ISR_EP0_OUT_ACK_STALL		BIT(1)
#define ISR_EP0_SETUP			BIT(0)
#define IRQ_ACK_ALL			0x301ff

/* ASPEED_USB_DEV_RESET 		0x20 */
#define EP_POOL_RESET			BIT(9)
#define DMA_CTRL_RESET			BIT(8)
#define ROOT_UBD_RESET			BIT(0)

/*************************************************************************************/
/* ASPEED_USB_EP0_CTRL			0x30 */
#define EP0_GET_RX_LEN(x)		((x >> 16) & 0x7f)
#define EP0_TX_LEN(x)			((x & 0x7f) << 8)
#define EP0_RX_BUFF_RDY			BIT(2)
#define EP0_TX_BUFF_RDY			BIT(1)
#define EP0_STALL			BIT(0)

/*************************************************************************************/
/* ASPEED_EP_CONFIG			0x00 */
#define EP_SET_MAX_PKT(x)		((x & 0x3ff) << 16)
#define EP_AUTO_DATA_DISABLE		(0x1 << 13)
#define EP_SET_EP_STALL			(0x1 << 12)
#define EP_SET_EP_NUM(x)		((x & 0xf) << 8)
#define EP_SET_TYPE_MASK(x)		((x) << 4)
#define EP_TYPE_BULK_IN			(0x2 << 4)
#define EP_TYPE_BULK_OUT		(0x3 << 4)
#define EP_TYPE_INT_IN			(0x4 << 4)
#define EP_TYPE_INT_OUT			(0x5 << 4)
#define EP_TYPE_ISO_IN			(0x6 << 4)
#define EP_TYPE_ISO_OUT			(0x7 << 4)
#define EP_ALLOCATED_MASK		(0x7 << 1)
#define EP_ENABLE			BIT(0)

/*************************************************************************************/
/* ASPEED_EP_DMA_CTRL			0x04 */
#define EP_DMA_DESC_OP_RESET		BIT(2)
#define EP_DMA_SINGLE_DESC		BIT(1)

/*************************************************************************************/
/* ASPEED_EP_DMA_BUFF			0x08 */

/*************************************************************************************/
/* define ASPEED_EP_DMA_STS		0x0C */
#define EP_TX_LEN(x)			((x & 0x7f) << 16)

/*************************************************************************************/
#define ASPEED_USB_PHY_CTRL0		0x800

#define PHY_CTRL0_8_BITS_UTMI		BIT(8)

/*************************************************************************************/
#define RX_DMA_BUFF_SIZE		1024

enum ep_state {
	ep_state_token = 0,
	ep_state_data,
	ep_state_status,
	ep_state_stall,
};

struct usb_device_ep_data {
	int is_out;
	int tx_len;
	int tx_last;
	int rx_len;
	int rx_last;
	int mps;
	uint8_t *tx_dma;
	uint8_t *rx_data;
	uint8_t rx_dma[RX_DMA_BUFF_SIZE];
	enum ep_state state;

	usb_dc_ep_callback cb_in;
	usb_dc_ep_callback cb_out;
};

struct usb_device_data {
	bool init;
	bool attached;
	usb_dc_status_callback status_cb;
	struct usb_device_ep_data *ep_data;

	/* k_work for bottom half handler */
	struct k_work usb_work;

	/* register base */
	uint32_t base;
	uint32_t max_epns;
};

static struct usb_device_data dev_data;

struct usb_aspeed_cfg {
	const uint32_t base;
	const reset_control_subsys_t rst_id;
	const uint32_t max_epns;
};

#define DEV_CFG(dev) ((const struct usb_aspeed_cfg *const)(dev)->config)

struct usbd_mem_block {
	void *data;
};

/**
 * @brief USBD peripheral event types.
 */
enum usbd_event_type {
	USBD_EVT_RESET,
	USBD_EVT_SETUP,
	USBD_EVT_EP,
	USBD_EVT_REINIT
};

/**
 * @brief Endpoint USB event
 *	  Used by ISR to send events to work handler
 *
 * @param node		Used by the kernel for FIFO management
 * @param block		Mempool block pointer for freeing up after use
 * @param evt		Event data field
 * @param evt_type	Type of event that has occurred from the USBD peripheral
 */
struct usbd_event {
	sys_snode_t node;
	struct usbd_mem_block block;
	enum usbd_event_type evt_type;
};

/* FIFO used for queuing up events from ISR. */
K_FIFO_DEFINE(usbd_evt_fifo);

#define ASPEED_USB_WORK_QUEUE_STACK_SIZE	1024

/* Work queue used for handling the ISR events (i.e. for notifying the USB
 * device stack, for executing the endpoints callbacks, etc.) out of the ISR
 * context.
 * The system work queue cannot be used for this purpose as it might be used in
 * applications for scheduling USB transfers and this could lead to a deadlock
 * when the USB device stack would not be notified about certain event because
 * of a system work queue item waiting for a USB transfer to be finished.
 */
static struct k_work_q usbd_work_queue;
static K_KERNEL_STACK_DEFINE(usbd_work_queue_stack,
			     ASPEED_USB_WORK_QUEUE_STACK_SIZE);

/**
 * @brief Fifo element slab
 *	Used for allocating fifo elements to pass from ISR to work handler
 * TODO: The number of FIFO elements is an arbitrary number now but it should
 * be derived from the theoretical number of backlog events possible depending
 * on the number of endpoints configured.
 */
#define FIFO_ELEM_SZ			sizeof(struct usbd_event)
#define FIFO_ELEM_ALIGN			sizeof(unsigned int)
#define ASPEED_USB_EVT_QUEUE_SIZE	64

K_MEM_SLAB_DEFINE(fifo_elem_slab, FIFO_ELEM_SZ,
		  ASPEED_USB_EVT_QUEUE_SIZE, FIFO_ELEM_ALIGN);

/**
 * @brief Schedule USBD event processing.
 *
 * Should be called after usbd_evt_put().
 */
static inline void usbd_work_schedule(void)
{
	k_work_submit_to_queue(&usbd_work_queue, &dev_data.usb_work);
}

/**
 * @brief Free previously allocated USBD event.
 *
 * Should be called after usbd_evt_get().
 *
 * @param Pointer to the USBD event structure.
 */
static inline void usbd_evt_free(struct usbd_event *ev)
{
	k_mem_slab_free(&fifo_elem_slab, (void **)&ev->block.data);
}

/**
 * @brief Enqueue USBD event.
 *
 * @param Pointer to the previously allocated and filled event structure.
 */
static inline void usbd_evt_put(struct usbd_event *ev)
{
	k_fifo_put(&usbd_evt_fifo, ev);
}

/**
 * @brief Get next enqueued USBD event if present.
 */
static inline struct usbd_event *usbd_evt_get(void)
{
	return k_fifo_get(&usbd_evt_fifo, K_NO_WAIT);
}

/**
 * @brief Drop all enqueued events.
 */
static inline void usbd_evt_flush(void)
{
	struct usbd_event *ev;

	do {
		ev = usbd_evt_get();
		if (ev) {
			usbd_evt_free(ev);
		}
	} while (ev != NULL);
}

/**
 * @brief Allocate USBD event.
 *
 * This function should be called prior to usbd_evt_put().
 *
 * @returns Pointer to the allocated event or NULL if there was no space left.
 */
static inline struct usbd_event *usbd_evt_alloc(void)
{
	struct usbd_event *ev;
	struct usbd_mem_block block;

	if (k_mem_slab_alloc(&fifo_elem_slab,
			     (void **)&block.data, K_NO_WAIT)) {
		LOG_ERR("USBD event allocation failed!");

		/*
		 * Allocation may fail if workqueue thread is starved or event
		 * queue size is too small (ASPEED_USB_EVT_QUEUE_SIZE).
		 * Wipe all events, free the space and schedule
		 * reinitialization.
		 */
		usbd_evt_flush();

		if (k_mem_slab_alloc(&fifo_elem_slab, (void **)&block.data,
				     K_NO_WAIT)) {
			LOG_ERR("USBD event memory corrupted");
			__ASSERT_NO_MSG(0);
			return NULL;
		}

		ev = (struct usbd_event *)block.data;
		ev->block = block;
		ev->evt_type = USBD_EVT_REINIT;
		usbd_evt_put(ev);
		usbd_work_schedule();

		return NULL;
	}

	ev = (struct usbd_event *)block.data;
	ev->block = block;

	return ev;
}

static void aspeed_udc_ep_handle(int ep_num)
{
	LOG_DBG("ep[%d] %s handle", ep_num,
		dev_data.ep_data[ep_num].is_out ? "OUT" : "IN");

	if (dev_data.ep_data[ep_num].is_out &&
	    dev_data.ep_data[ep_num].cb_out)
		dev_data.ep_data[ep_num].cb_out(USB_EP_DIR_OUT | ep_num,
						USB_DC_EP_DATA_OUT);
	else if (!dev_data.ep_data[ep_num].is_out &&
		 dev_data.ep_data[ep_num].cb_in)
		dev_data.ep_data[ep_num].cb_in(USB_EP_DIR_IN | ep_num,
					       USB_DC_EP_DATA_IN);
}

static void aspeed_udc_ep0_rx(uint32_t offset)
{
	uint32_t dma_addr;

	dma_addr = (uint32_t)dev_data.ep_data[0].rx_dma + offset;
	sys_write32(dma_addr, dev_data.base + ASPEED_USB_EP0_DATA_BUFF);
	sys_write32(EP0_RX_BUFF_RDY, dev_data.base + ASPEED_USB_EP0_CTRL);
}

static void aspeed_udc_ep0_tx(uint32_t tx_len)
{
	sys_write32(EP0_TX_LEN(tx_len),
			dev_data.base + ASPEED_USB_EP0_CTRL);
	sys_write32(EP0_TX_LEN(tx_len) | EP0_TX_BUFF_RDY,
			dev_data.base + ASPEED_USB_EP0_CTRL);
}

static void aspeed_udc_ep0_handle_ack(bool in_ack)
{
	int rx_len;

	LOG_DBG("DIR:%s, ep_state:0x%x", in_ack ? "IN" : "OUT",
			dev_data.ep_data[0].state);

	if (dev_data.ep_data[0].state == ep_state_token) {
		LOG_DBG("ACK, wrong ep state: 0x%x",
			dev_data.ep_data[0].state);
		return;

	} else if (dev_data.ep_data[0].state == ep_state_status) {
		dev_data.ep_data[0].state = ep_state_token;
		return;
	}

	/* Data/status stage */
	if (in_ack) {
		LOG_DBG("ACK tx len: [%d/%d]",
			dev_data.ep_data[0].tx_last,
			dev_data.ep_data[0].tx_len);

		if (dev_data.ep_data[0].tx_last != dev_data.ep_data[0].tx_len) {
			struct usbd_event *ev = usbd_evt_alloc();

			if (!ev) {
				return;
			}

			ev->evt_type = USBD_EVT_EP;
			usbd_evt_put(ev);
			usbd_work_schedule();

		} else {
			LOG_DBG("tx done, ready to rx");
			dev_data.ep_data[0].state = ep_state_status;
			aspeed_udc_ep0_rx(0);
		}

	} else {
		/* RX */
		rx_len = EP0_GET_RX_LEN(sys_read32(dev_data.base +
				ASPEED_USB_EP0_CTRL));
		dev_data.ep_data[0].rx_last += rx_len;

		LOG_DBG("ACK rx len: [%d/%d]",
			dev_data.ep_data[0].rx_last,
			dev_data.ep_data[0].rx_len);

		/* rx done or rx buffer full */
		if ((dev_data.ep_data[0].rx_last == dev_data.ep_data[0].rx_len) ||
			(dev_data.ep_data[0].rx_last % RX_DMA_BUFF_SIZE == 0)) {
			struct usbd_event *ev = usbd_evt_alloc();

			if (!ev) {
				return;
			}

			ev->evt_type = USBD_EVT_EP;
			usbd_evt_put(ev);
			usbd_work_schedule();
		} else {
			aspeed_udc_ep0_rx(dev_data.ep_data[0].rx_last %
					  RX_DMA_BUFF_SIZE);
		}
	}
}

static void aspeed_udc_ep0_handle_setup(void)
{
	struct usb_setup_packet *setup;

	setup = (void *)(dev_data.base + ASPEED_USB_SETUP_DATA0);

	if (dev_data.ep_data[0].state != ep_state_token) {
		LOG_DBG("Setup: ep state: 0x%x",
			dev_data.ep_data[0].state);
	}

	dev_data.ep_data[0].state = ep_state_token;

	LOG_DBG("--> Setup ---");
	if (setup->bmRequestType & USB_EP_DIR_IN) {
		dev_data.ep_data[0].is_out = 0;
		dev_data.ep_data[0].tx_len = 0;
		dev_data.ep_data[0].tx_last = 0;
		LOG_DBG("Device -> Host");
	} else {
		dev_data.ep_data[0].is_out = 1;
		dev_data.ep_data[0].rx_len = 0;
		dev_data.ep_data[0].rx_last = 0;
		LOG_DBG("Host -> Device");
	}

	struct usbd_event *ev = usbd_evt_alloc();

	if (!ev) {
		return;
	}

	ev->evt_type = USBD_EVT_SETUP;
	usbd_evt_put(ev);
	usbd_work_schedule();

	LOG_DBG(" --- Setup <----\n");
}

static void usb_aspeed_isr(void)
{
	uint32_t isr_reg = dev_data.base + ASPEED_USB_ISR;
	uint32_t isr = sys_read32(isr_reg);
	uint32_t ep_isr;
	int i;

	if (!(isr & IRQ_ACK_ALL))
		return;

	if (isr & ISR_BUS_RESET) {
		LOG_DBG("ISR_BUS_RESET");
		sys_write32(ISR_BUS_RESET, isr_reg);
		dev_data.ep_data[0].state = ep_state_token;
		dev_data.status_cb(USB_DC_RESET, NULL);
	}

	if (isr & ISR_BUS_SUSPEND) {
		LOG_DBG("ISR_BUS_SUSPEND");
		sys_write32(ISR_BUS_SUSPEND, isr_reg);
		dev_data.status_cb(USB_DC_SUSPEND, NULL);
	}

	if (isr & ISR_SUSPEND_RESUME) {
		LOG_DBG("ISR_SUSPEND_RESUME");
		sys_write32(ISR_SUSPEND_RESUME, isr_reg);
		dev_data.status_cb(USB_DC_RESUME, NULL);
	}

	if (isr & ISR_EP0_IN_ACK_STALL) {
		LOG_DBG("EP0_IN_ACK_STALL");
		sys_write32(ISR_EP0_IN_ACK_STALL, isr_reg);
		aspeed_udc_ep0_handle_ack(true);
	}

	if (isr & ISR_EP0_OUT_NAK) {
		sys_write32(ISR_EP0_OUT_NAK, isr_reg);
	}

	if (isr & ISR_EP0_OUT_ACK_STALL) {
		LOG_DBG("ISR_EP0_OUT_ACK_STALL");
		sys_write32(ISR_EP0_OUT_ACK_STALL, isr_reg);
		aspeed_udc_ep0_handle_ack(false);
	}

	if (isr & ISR_EP0_IN_DATA_NAK) {
		sys_write32(ISR_EP0_IN_DATA_NAK, isr_reg);
	}

	if (isr & ISR_EP_ACK_STALL) {
		LOG_DBG("ISR_EP_ACK_STALL");
		ep_isr = sys_read32(dev_data.base + ASPEED_USB_EP_ACK_ISR);
		for (i = 0; i < dev_data.max_epns; i++) {
			if (ep_isr & (0x1 << i)) {
				sys_write32(0x1 << i,
					dev_data.base + ASPEED_USB_EP_ACK_ISR);
				aspeed_udc_ep_handle(i + 1);
			}
		}
	}

	if (isr & ISR_EP0_SETUP) {
		LOG_DBG("ISR_EP0_SETUP");
		sys_write32(ISR_EP0_SETUP, isr_reg);
		aspeed_udc_ep0_handle_setup();
	}

	if (isr & ISR_EP_NAK) {
		LOG_DBG("ISR_EP_NAK");
		sys_write32(ISR_EP_NAK, isr_reg);
	}
}

/* Work handler */
static void usbd_work_handler(struct k_work *item)
{
	struct usbd_event *ev;

	while ((ev = usbd_evt_get()) != NULL) {
		switch (ev->evt_type) {
		case USBD_EVT_RESET:
			LOG_DBG("USBD reset event");
			break;

		case USBD_EVT_SETUP:
			LOG_DBG("USBD setup event");
			dev_data.ep_data[0].state = ep_state_data;

			if (dev_data.ep_data[0].cb_out)
				dev_data.ep_data[0].cb_out(USB_EP_DIR_OUT,
							   USB_DC_EP_SETUP);

			break;

		case USBD_EVT_EP:
			LOG_DBG("USBD ep event");
			aspeed_udc_ep_handle(0);
			break;

		case USBD_EVT_REINIT:
			LOG_ERR("USBD event queue full!");
			break;

		default:
			LOG_ERR("Unknown USBD event: %"PRId16, ev->evt_type);
			break;
		}
		usbd_evt_free(ev);
	}
}

static int usb_aspeed_init(const struct device *dev)
{
	const struct usb_aspeed_cfg *config = DEV_CFG(dev);
	const struct device *reset_dev;
	int i;

	LOG_DBG("init");

	dev_data.base = config->base;
	dev_data.max_epns = config->max_epns;

	reset_dev = device_get_binding(ASPEED_RST_CTRL_NAME);
	reset_control_deassert(reset_dev, config->rst_id);

	/* wait 1 ms */
	k_busy_wait(1000);
	sys_write32(ROOT_PHY_CLK_EN | ROOT_PHY_RESET_DIS,
		dev_data.base + ASPEED_USB_CTRL);

	/* wait 1 ms */
	k_busy_wait(1000);
	sys_write32(0, dev_data.base + ASPEED_USB_DEV_RESET);

	sys_write32(0x0, dev_data.base + ASPEED_USB_IER);
	sys_write32(IRQ_ACK_ALL, dev_data.base + ASPEED_USB_ISR);

	sys_write32(0x0, dev_data.base + ASPEED_USB_EP_ACK_IER);
	sys_write32(GENMASK(dev_data.max_epns, 0),
		    dev_data.base + ASPEED_USB_EP_ACK_ISR);

	sys_write32(0, dev_data.base + ASPEED_USB_EP0_CTRL);

	/* Enable 8 bit UTMI */
	sys_write32(sys_read32(dev_data.base + ASPEED_USB_PHY_CTRL0) |
		PHY_CTRL0_8_BITS_UTMI,
		dev_data.base + ASPEED_USB_PHY_CTRL0);

	/* Connect and enable USB interrupt */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
		    usb_aspeed_isr, 0, 0);

	irq_enable(DT_INST_IRQN(0));

	if (!dev_data.init) {
		/* initialize dev_data */
		dev_data.ep_data = k_malloc(sizeof(struct usb_device_ep_data) *
					dev_data.max_epns);
		for (i = 0; i < dev_data.max_epns; i++)
			dev_data.ep_data[i].is_out = -1;

		dev_data.init = true;
	}

	k_work_queue_start(&usbd_work_queue,
			usbd_work_queue_stack,
			K_KERNEL_STACK_SIZEOF(usbd_work_queue_stack),
			CONFIG_SYSTEM_WORKQUEUE_PRIORITY, NULL);

	k_thread_name_set(&usbd_work_queue.thread, "usbdworkq");
	k_work_init(&dev_data.usb_work, usbd_work_handler);

	sys_write32(ISR_EP_ACK_STALL |
		    ISR_SUSPEND_RESUME |
		    ISR_BUS_SUSPEND |
		    ISR_BUS_RESET |
		    ISR_EP0_IN_ACK_STALL |
		    ISR_EP0_OUT_ACK_STALL |
		    ISR_EP0_SETUP,
		    dev_data.base + ASPEED_USB_IER);

	return 0;
}

/**
 * @brief Attach USB for device connection
 *
 * Function to attach USB for device connection. Upon success, the USB PLL
 * is enabled, and the USB device is now capable of transmitting and receiving
 * on the USB bus and of generating interrupts.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_attach(void)
{
	LOG_DBG("attach");

	if (dev_data.attached) {
		LOG_WRN("already attached");
		return 0;
	}

	dev_data.attached = true;
	LOG_DBG("attached");

	return 0;
}

/**
 * @brief Detach the USB device
 *
 * Function to detach the USB device. Upon success, the USB hardware PLL
 * is powered down and USB communication is disabled.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_detach(void)
{
	LOG_DBG("detach");

	if (!dev_data.attached) {
		LOG_WRN("already detached");
		return 0;
	}

	dev_data.attached = false;
	usbd_evt_flush();
	LOG_DBG("detached");

	return 0;
}

/**
 * @brief Reset the USB device
 *
 * This function returns the USB device and firmware back to it's initial state.
 * N.B. the USB PLL is handled by the usb_detach function
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_reset(void)
{
	uint32_t setting = 0;

	LOG_DBG("device reset");

	setting = EP_POOL_RESET | DMA_CTRL_RESET | ROOT_UBD_RESET;
	sys_write32(setting, dev_data.base + ASPEED_USB_DEV_RESET);

	return 0;
}

/**
 * @brief Set USB device address
 *
 * @param[in] addr Device address
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_set_address(const uint8_t addr)
{
	LOG_DBG("addr: 0x%x", addr);

	sys_write32(addr & 0x7f, dev_data.base + ASPEED_USB_CONF);
	dev_data.ep_data[0].state = ep_state_status;
	dev_data.ep_data[0].is_out = 0;

	return 0;
}

/**
 * @brief Set USB device controller status callback
 *
 * Function to set USB device controller status callback. The registered
 * callback is used to report changes in the status of the device controller.
 * The status code are described by the usb_dc_status_code enumeration.
 *
 * @param[in] cb Callback function
 */
void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	LOG_DBG("%s...", __func__);

	dev_data.status_cb = cb;
}

/**
 * @brief check endpoint capabilities
 *
 * Function to check capabilities of an endpoint. usb_dc_ep_cfg_data structure
 * provides the endpoint configuration parameters: endpoint address,
 * endpoint maximum packet size and endpoint type.
 * The driver should check endpoint capabilities and return 0 if the
 * endpoint configuration is possible.
 *
 * @param[in] cfg Endpoint config
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data * const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);
	int i;

	LOG_DBG("check ep cap:0x%x", cfg->ep_addr);

	if (!dev_data.init) {
		/* initialize dev_data */
		for (i = 0; i < dev_data.max_epns; i++)
			dev_data.ep_data[i].is_out = -1;

		dev_data.init = true;
	}

	if (ep_idx > dev_data.max_epns) {
		LOG_ERR("ep_idx %s(%d), addr:0x%x, mps:0x%x, type:0x%x",
			"OUT of Range",
			dev_data.max_epns, cfg->ep_addr,
			cfg->ep_mps, cfg->ep_type);
		return -1;
	}

	if (ep_idx == 0U) {
		if (cfg->ep_type != USB_DC_EP_CONTROL) {
			LOG_ERR("ep[0x%x] pre-selected as control endpoint",
				cfg->ep_addr);
			return -1;
		}

		return 0;
	}

	if (dev_data.ep_data[ep_idx].is_out < 0) {
		if (USB_EP_GET_DIR(cfg->ep_addr) == USB_EP_DIR_IN) {
			LOG_INF("select ep[0x%x] as IN endpoint",
				cfg->ep_addr);
			dev_data.ep_data[ep_idx].is_out = 0;
		} else {
			LOG_INF("select ep[0x%x] as OUT endpoint",
				cfg->ep_addr);
			dev_data.ep_data[ep_idx].is_out = 1;
		}

	} else if (dev_data.ep_data[ep_idx].is_out == 0) {
		if (USB_EP_GET_DIR(cfg->ep_addr) != USB_EP_DIR_IN) {
			LOG_WRN("pre-selected ep[0x%x] as IN endpoint",
				cfg->ep_addr);
			return -EINVAL;
		}

	} else if (dev_data.ep_data[ep_idx].is_out == 1) {
		if (USB_EP_GET_DIR(cfg->ep_addr) != USB_EP_DIR_OUT) {
			LOG_WRN("pre-selected ep[0x%x] as OUT endpoint",
				cfg->ep_addr);
			return -EINVAL;
		}
	}

	if (cfg->ep_mps < 1 || cfg->ep_mps > 1024 ||
	    (cfg->ep_type == USB_DC_EP_CONTROL && cfg->ep_mps > 64)) {
		LOG_ERR("invalid endpoint size");
		return -1;
	}

	return 0;
}

/**
 * @brief Configure endpoint
 *
 * Function to configure an endpoint. usb_dc_ep_cfg_data structure provides
 * the endpoint configuration parameters: endpoint address, endpoint maximum
 * packet size and endpoint type.
 *
 * @param[in] cfg Endpoint config
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data * const cfg)
{
	uint32_t ep_conf;
	uint32_t ep_reg;
	uint8_t dir_in;
	int ep_num;

	if (!dev_data.attached)
		return -ENODEV;

	LOG_DBG("ep config: addr:0x%x, mps:0x%x, type:0x%x",
		cfg->ep_addr, cfg->ep_mps, cfg->ep_type);

	ep_conf = EP_SET_MAX_PKT(cfg->ep_mps);
	ep_num = USB_EP_GET_IDX(cfg->ep_addr);
	ep_conf |= EP_SET_EP_NUM(ep_num);
	dir_in = USB_EP_GET_DIR(cfg->ep_addr);

	dev_data.ep_data[ep_num].mps = cfg->ep_mps;

	if (!(dev_data.ep_data[ep_num].is_out ^ dir_in)) {
		LOG_ERR("ep[0x%x] pre-selected dir is mismatch", cfg->ep_addr);
		return -EINVAL;
	}

	switch (cfg->ep_type) {
	case USB_DC_EP_CONTROL:
		LOG_DBG("configure control endpoint, nothing to do");
		return 0;
	case USB_DC_EP_BULK:
		LOG_DBG("configure bulk endpoint");
		if (dir_in)
			ep_conf |= EP_TYPE_BULK_IN;
		else
			ep_conf |= EP_TYPE_BULK_OUT;
		break;
	case USB_DC_EP_INTERRUPT:
		LOG_DBG("configure interrupt endpoint");
		if (dir_in)
			ep_conf |= EP_TYPE_INT_IN;
		else
			ep_conf |= EP_TYPE_INT_OUT;
		break;
	case USB_DC_EP_ISOCHRONOUS:
		LOG_DBG("configure isochronous endpoint");
		if (dir_in)
			ep_conf |= EP_TYPE_ISO_IN;
		else
			ep_conf |= EP_TYPE_ISO_OUT;
		break;
	default:
		return -EINVAL;
	}

	ep_reg = dev_data.base + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));

	sys_write32(EP_DMA_DESC_OP_RESET, ep_reg + ASPEED_EP_DMA_CTRL);
	sys_write32(EP_DMA_SINGLE_DESC, ep_reg + ASPEED_EP_DMA_CTRL);
	sys_write32(0x0, ep_reg + ASPEED_EP_DMA_STS);
	sys_write32(ep_conf, ep_reg + ASPEED_EP_CONFIG);

	LOG_DBG("ep[%d] config:%x", ep_num,
		sys_read32(ep_reg + ASPEED_EP_CONFIG));

	return 0;
}

/**
 * @brief Set stall condition for the selected endpoint
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_set_stall(const uint8_t ep)
{
	uint8_t ep_num = USB_EP_GET_IDX(ep);
	uint32_t ep_reg;

	LOG_DBG("set ep[0x%x] stall", ep);

	if (ep_num >= dev_data.max_epns)
		return -EINVAL;

	if (ep_num == 0)
		sys_write32(EP0_STALL, dev_data.base + ASPEED_USB_EP0_CTRL);
	else {
		ep_reg = dev_data.base + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));
		sys_write32(EP_SET_EP_STALL, ep_reg);
	}

	return 0;
}

/**
 * @brief Clear stall condition for the selected endpoint
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_clear_stall(const uint8_t ep)
{
	uint8_t ep_num = USB_EP_GET_IDX(ep);
	uint32_t ep_reg;

	LOG_DBG("clear stall ep[0x%x]", ep);

	if (ep_num >= dev_data.max_epns)
		return -EINVAL;

	if (ep_num == 0) {
		ep_reg = dev_data.base + ASPEED_USB_EP0_CTRL;
		sys_write32(sys_read32(ep_reg) & (~EP0_STALL), ep_reg);
	} else {
		ep_reg = dev_data.base + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));
		sys_write32(sys_read32(ep_reg) & (~EP_SET_EP_STALL), ep_reg);
	}

	return 0;
}

/**
 * @brief Check if the selected endpoint is stalled
 *
 * @param[in]  ep       Endpoint address corresponding to the one
 *                      listed in the device configuration table
 * @param[out] stalled  Endpoint stall status
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	uint32_t ep_num = USB_EP_GET_IDX(ep);
	uint32_t ep_reg;

	LOG_DBG("check ep[0x%x] is stalled", ep_num);

	if (ep_num >= dev_data.max_epns || !stalled)
		return -EINVAL;

	*stalled = 0;

	if (ep_num == 0) {
		ep_reg = dev_data.base + ASPEED_USB_EP0_CTRL;
		if (sys_read32(ep_reg) & EP0_STALL)
			*stalled = 1;
	} else {
		ep_reg = dev_data.base + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));
		if (sys_read32(ep_reg) & EP_SET_EP_STALL)
			*stalled = 1;
	}

	return 0;
}

/**
 * @brief Halt the selected endpoint
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_halt(const uint8_t ep)
{
	return usb_dc_ep_set_stall(ep);
}

/**
 * @brief Enable the selected endpoint
 *
 * Function to enable the selected endpoint. Upon success interrupts are
 * enabled for the corresponding endpoint and the endpoint is ready for
 * transmitting/receiving data.
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_enable(const uint8_t ep)
{
	uint8_t ep_num = USB_EP_GET_IDX(ep);
	uint32_t ep_reg, val;

	LOG_DBG("enable ep[0x%x]", ep);

	if (ep_num >= dev_data.max_epns)
		return -EINVAL;

	if (ep_num == 0) {
		dev_data.ep_data[0].state = ep_state_token;
		sys_write32(sys_read32(dev_data.base + ASPEED_USB_CTRL) |
			    ROOT_UPSTREAM_EN,
			    dev_data.base + ASPEED_USB_CTRL);

	} else {
		ep_reg = dev_data.base + ASPEED_EP_OFFSET +
			 (0x10 * (ep_num - 1));
		sys_write32(sys_read32(ep_reg + ASPEED_EP_CONFIG) |
			    EP_ENABLE,
			    ep_reg + ASPEED_EP_CONFIG);

		if (dev_data.ep_data[ep_num].is_out) {
			sys_write32(TO_PHY_ADDR(dev_data.ep_data[ep_num].rx_dma),
				    ep_reg + ASPEED_EP_DMA_BUFF);
			sys_write32(0x1, ep_reg + ASPEED_EP_DMA_STS);
		}
	}

	/* enable interrupts */
	val = sys_read32(dev_data.base + ASPEED_USB_EP_ACK_IER);
	val |= BIT(ep_num - 1);
	sys_write32(val, dev_data.base + ASPEED_USB_EP_ACK_IER);

	return 0;
}

/**
 * @brief Disable the selected endpoint
 *
 * Function to disable the selected endpoint. Upon success interrupts are
 * disabled for the corresponding endpoint and the endpoint is no longer able
 * for transmitting/receiving data.
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_disable(const uint8_t ep)
{
	uint8_t ep_num = USB_EP_GET_IDX(ep);
	uint32_t val;

	LOG_DBG("disable ep[0x%x]", ep);

	if (ep_num >= dev_data.max_epns)
		return -EINVAL;

	/* disable interrupts */
	val = sys_read32(dev_data.base + ASPEED_USB_EP_ACK_IER);
	val &= ~ep_num;
	sys_write32(val, dev_data.base + ASPEED_USB_EP_ACK_IER);

	return 0;
}

/**
 * @brief Flush the selected endpoint
 *
 * This function flushes the FIFOs for the selected endpoint.
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_flush(const uint8_t ep)
{
	uint32_t ep_num = USB_EP_GET_IDX(ep);

	LOG_DBG("flush ep[0x%x]", ep_num);

	if (ep_num >= dev_data.max_epns)
		return -EINVAL;

	return 0;
}

/**
 * @brief Write data to the specified endpoint
 *
 * This function is called to write data to the specified endpoint. The
 * supplied usb_ep_callback function will be called when data is transmitted
 * out.
 *
 * @param[in]  ep        Endpoint address corresponding to the one
 *                       listed in the device configuration table
 * @param[in]  data      Pointer to data to write
 * @param[in]  data_len  Length of the data requested to write. This may
 *                       be zero for a zero length status packet.
 * @param[out] ret_bytes Bytes scheduled for transmission. This value
 *                       may be NULL if the application expects all
 *                       bytes to be written
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data,
		    const uint32_t data_len, uint32_t * const ret_bytes)
{
	uint32_t ep_reg;
	int ep_num;
	int tx_len;

	if (!data_len) {
		LOG_DBG("Send ZLP");
		dev_data.ep_data[0].tx_len = 0;
		dev_data.ep_data[0].tx_last = 0;
		aspeed_udc_ep0_tx(0);

		return 0;
	}

	LOG_DBG("[Write] ep:0x%x, data:0x%x, data_len:0x%x, ret_bytes:0x%x",
		ep, (uint32_t)data, data_len, *ret_bytes);

	ep_num = USB_EP_GET_IDX(ep);

	if (ep_num >= dev_data.max_epns)
		return -EINVAL;

	LOG_DBG("trigger ep%d tx", ep_num);

	if (ep_num == 0) {
		dev_data.ep_data[0].tx_dma = (uint8_t *)data;
		dev_data.ep_data[0].tx_len = data_len;
		if (data_len > USB_MAX_CTRL_MPS)
			tx_len = USB_MAX_CTRL_MPS;
		else
			tx_len = data_len;

		LOG_DBG("trigger tx len: [%d/%d]", tx_len, data_len);
		dev_data.ep_data[0].tx_last = tx_len;

		cache_data_range((void *)data, data_len, K_CACHE_INVD);

		sys_write32(TO_PHY_ADDR(dev_data.ep_data[0].tx_dma),
			    dev_data.base + ASPEED_USB_EP0_DATA_BUFF);
		aspeed_udc_ep0_tx(tx_len);

		if (ret_bytes)
			*ret_bytes = tx_len;

	} else {
		LOG_DBG("trigger ep tx");
		LOG_DBG("trigger tx len: [%d]", data_len);

		ep_reg = dev_data.base + ASPEED_EP_OFFSET +
			 (0x10 * (ep_num - 1));

		if (ret_bytes)
			*ret_bytes = data_len;

		sys_write32(TO_PHY_ADDR(data), ep_reg + ASPEED_EP_DMA_BUFF);
		sys_write32(EP_TX_LEN(data_len), ep_reg + ASPEED_EP_DMA_STS);
		sys_write32(EP_TX_LEN(data_len) | 0x1,
			ep_reg + ASPEED_EP_DMA_STS);
	}

	return 0;
}

/**
 * @brief Read data from the specified endpoint
 *
 * This function is called by the endpoint handler function, after an OUT
 * interrupt has been received for that EP. The application must only call this
 * function through the supplied usb_ep_callback function. This function clears
 * the ENDPOINT NAK, if all data in the endpoint FIFO has been read,
 * so as to accept more data from host.
 *
 * @param[in]  ep           Endpoint address corresponding to the one
 *                          listed in the device configuration table
 * @param[in]  data         Pointer to data buffer to write to
 * @param[in]  max_data_len Max length of data to read
 * @param[out] read_bytes   Number of bytes read. If data is NULL and
 *                          max_data_len is 0 the number of bytes
 *                          available for read should be returned.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_read(const uint8_t ep, uint8_t *const data,
		   const uint32_t max_data_len, uint32_t *const read_bytes)
{
	if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
		return -EINVAL;
	}

	if (!data && !max_data_len) {
		/* When both buffer and max data to read are zero the above
		 * call would fetch the data len and we simply return.
		 */
		return 0;
	}

	if (usb_dc_ep_read_continue(ep) != 0) {
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Set callback function for the specified endpoint
 *
 * Function to set callback function for notification of data received and
 * available to application or transmit done on the selected endpoint,
 * NULL if callback not required by application code. The callback status
 * code is described by usb_dc_ep_cb_status_code.
 *
 * @param[in] ep Endpoint address corresponding to the one
 *               listed in the device configuration table
 * @param[in] cb Callback function
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	uint8_t ep_num = USB_EP_GET_IDX(ep);

	LOG_DBG("ep[0x%x] set callback", ep);

	if (ep_num >= dev_data.max_epns) {
		LOG_ERR("Wrong endpoint addr:0x%x", ep_num);
		return -EINVAL;
	}

	if (!dev_data.attached)
		return -ENODEV;

	if (ep & USB_EP_DIR_IN)
		dev_data.ep_data[ep_num].cb_in = cb;
	else
		dev_data.ep_data[ep_num].cb_out = cb;

	return 0;
}

/**
 * @brief Read data from the specified endpoint
 *
 * This is similar to usb_dc_ep_read, the difference being that, it doesn't
 * clear the endpoint NAKs so that the consumer is not bogged down by further
 * upcalls till he is done with the processing of the data. The caller should
 * reactivate ep by invoking usb_dc_ep_read_continue() do so.
 *
 * @param[in]  ep           Endpoint address corresponding to the one
 *                          listed in the device configuration table
 * @param[in]  data         Pointer to data buffer to write to
 * @param[in]  max_data_len Max length of data to read
 * @param[out] read_bytes   Number of bytes read. If data is NULL and
 *                          max_data_len is 0 the number of bytes
 *                          available for read should be returned.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len,
			uint32_t *read_bytes)
{
	struct usb_setup_packet *setup;
	uint32_t ep_dma_sts;
	uint32_t ep_reg;
	int byte_to_copy;
	int data_len;
	int ep_num;

	LOG_DBG("[Read Wait] %s:0x%x, %s:0x%x, %s:0x%x, %s:0x%x",
		"ep", (uint32_t)ep, "data", (uint32_t)data,
		"max_data_len", max_data_len,
		"read_bytes", *read_bytes);

	if (!data && max_data_len)
		return -EINVAL;

	if (!max_data_len) {
		LOG_DBG("rx done, ready to tx");
		dev_data.ep_data[0].state = ep_state_status;
		aspeed_udc_ep0_tx(0);
	}

	setup = (void *)(dev_data.base + ASPEED_USB_SETUP_DATA0);
	ep_num = USB_EP_GET_IDX(ep);

	if (ep_num >= dev_data.max_epns)
		return -EINVAL;

	/* Read Setup packet */
	if (ep_num == 0 && !read_bytes) {
		LOG_DBG("setup: %s:0x%x, %s:0x%x, %s:0x%x, %s:0x%x, %s:0x%x",
			"bmRequestType", setup->bmRequestType,
			"bRequest", setup->bRequest,
			"wValue", setup->wValue,
			"wIndex", setup->wIndex,
			"wLength", setup->wLength);

		LOG_DBG("copy setup data");
		memcpy(data, setup, max_data_len);

		if (dev_data.ep_data[0].is_out && setup->wLength) {
			LOG_DBG("ep0 setup rx buffer PA");
			dev_data.ep_data[0].rx_len = setup->wLength;
			sys_write32((uint32_t)dev_data.ep_data[0].rx_dma,
				dev_data.base + ASPEED_USB_EP0_DATA_BUFF);
		}

		return 0;
	}

	if (ep_num == 0) {
		if (dev_data.ep_data[0].is_out) {

			/* the maximum rx is RX_DMA_BUFF_SIZE */
			if (dev_data.ep_data[0].rx_last % RX_DMA_BUFF_SIZE == 0)
				data_len = RX_DMA_BUFF_SIZE;
			else
				data_len = dev_data.ep_data[0].rx_last %
						RX_DMA_BUFF_SIZE;

			LOG_DBG("Copy data from rx_dma, %s:0x%x",
				"data_len", data_len);
			cache_data_range(dev_data.ep_data[0].rx_dma, data_len,
					 K_CACHE_INVD);
			memcpy(data, dev_data.ep_data[0].rx_dma, data_len);
			memset(dev_data.ep_data[0].rx_dma, 0, RX_DMA_BUFF_SIZE);
			*read_bytes = data_len;

		} else {
			LOG_DBG("ep0 send IN status with no data");
			aspeed_udc_ep0_tx(0);
			dev_data.ep_data[0].state = ep_state_status;
		}

	} else {
		ep_reg = dev_data.base + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));
		ep_dma_sts = sys_read32(ep_reg + ASPEED_EP_DMA_STS);

		data_len = (ep_dma_sts >> 16) & 0x7ff;

		if (data_len > max_data_len) {
			byte_to_copy = max_data_len;
		} else {
			byte_to_copy = data_len;
		}

		cache_data_range(dev_data.ep_data[ep_num].rx_dma, data_len,
				 K_CACHE_INVD);
		if (byte_to_copy <= RX_DMA_BUFF_SIZE) {
			memcpy(data, dev_data.ep_data[ep_num].rx_dma,
				byte_to_copy);
			memset(dev_data.ep_data[ep_num].rx_dma, 0,
				RX_DMA_BUFF_SIZE);
		} else {
			LOG_ERR("%s:0x%x is out-of-bound",
				"byte_to_copy", byte_to_copy);
			return -EINVAL;
		}

		if (read_bytes) {
			*read_bytes = byte_to_copy;
			LOG_DBG("ep[%d] read 0x%x bytes",
				ep_num, (uint32_t)*read_bytes);
		}
	}

	return 0;
}

/**
 * @brief Continue reading data from the endpoint
 *
 * Clear the endpoint NAK and enable the endpoint to accept more data
 * from the host. Usually called after usb_dc_ep_read_wait() when the consumer
 * is fine to accept more data. Thus these calls together act as a flow control
 * mechanism.
 *
 * @param[in]  ep           Endpoint address corresponding to the one
 *                          listed in the device configuration table
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_ep_read_continue(uint8_t ep)
{
	uint32_t ep_num = USB_EP_GET_IDX(ep);
	uint32_t ep_reg;

	LOG_DBG("ep_read_continue, ep:0x%x", ep);

	if (ep_num >= dev_data.max_epns) {
		return -EINVAL;
	}

	if (ep_num == 0) {
		aspeed_udc_ep0_rx(0);
	} else {
		ep_reg = dev_data.base + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));
		sys_write32(0x1, ep_reg + ASPEED_EP_DMA_STS);
	}

	return 0;
}

/**
 * @brief Get endpoint max packet size
 *
 * @param[in]  ep           Endpoint address corresponding to the one
 *                          listed in the device configuration table
 *
 * @return Enpoint max packet size (mps)
 */
int usb_dc_ep_mps(uint8_t ep)
{
	uint32_t ep_num = USB_EP_GET_IDX(ep);

	if (ep_num >= dev_data.max_epns)
		return 0;

	LOG_DBG("ep[%d] mps: 0x%x", ep, dev_data.ep_data[ep_num].mps);

	return dev_data.ep_data[ep_num].mps;
}

#define ASPEED_UDC_INIT(n)						       \
	static const struct usb_aspeed_cfg usb_aspeed_cfg_##n = {	       \
		.base = (uint32_t)DT_INST_REG_ADDR(n),	                       \
		.rst_id = (reset_control_subsys_t)DT_INST_RESETS_CELL(n,       \
								      rst_id), \
		.max_epns = DT_INST_PROP(n, num_bidir_endpoints),              \
	};								       \
	DEVICE_DT_INST_DEFINE(n, usb_aspeed_init, NULL, NULL,		       \
			      &usb_aspeed_cfg_##n,                             \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_UDC_INIT)
