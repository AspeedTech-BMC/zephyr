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

#include "soc.h"

#define LOG_LEVEL	CONFIG_USB_DRIVER_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(usb_dc_aspeed);

#define REG_BASE		DT_REG_ADDR(DT_NODELABEL(udc))
#define NUM_OF_EP_MAX		DT_INST_PROP(0, num_bidir_endpoints)

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
/* ASPEED_USB_ISR 			0x0C */
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

struct usb_device_ep_data {
	int is_out;
	int tx_len;
	int tx_last;
	int rx_len;
	int mps;
	uint8_t *tx_dma;
	uint8_t rx_dma[1024];
	usb_dc_ep_callback cb_in;
	usb_dc_ep_callback cb_out;
};

struct usb_device_data {
	bool init;
	bool attached;
	usb_dc_status_callback status_cb;
	struct usb_device_ep_data ep_data[NUM_OF_EP_MAX];
};

static struct usb_device_data dev_data;

static void aspeed_usb_ep_handle(int ep_num)
{
	LOG_DBG("ep[%d] handle", ep_num);

	if (dev_data.ep_data[ep_num].is_out && dev_data.ep_data[ep_num].cb_out)
		dev_data.ep_data[ep_num].cb_out(USB_EP_DIR_OUT | ep_num, USB_DC_EP_DATA_OUT);
	else if (!dev_data.ep_data[ep_num].is_out && dev_data.ep_data[ep_num].cb_in)
		dev_data.ep_data[ep_num].cb_in(USB_EP_DIR_IN | ep_num, USB_DC_EP_DATA_IN);
}

static void aspeed_usb_setup_handle(void)
{
	struct usb_setup_packet *setup = (void *)(REG_BASE + ASPEED_USB_SETUP_DATA0);

	LOG_DBG(" --> Setup ---");

	if (setup->bmRequestType & USB_EP_DIR_IN) {
		dev_data.ep_data[0].is_out = 0;
		dev_data.ep_data[0].tx_len = 0;
		dev_data.ep_data[0].tx_last = 0;
		LOG_DBG("Device -> Host");
	} else {
		dev_data.ep_data[0].is_out = 1;
		LOG_DBG("Host -> Device");
	}

	if (dev_data.ep_data[0].cb_out)
		dev_data.ep_data[0].cb_out(USB_EP_DIR_OUT, USB_DC_EP_SETUP);

	LOG_DBG(" --- Setup <----");
}

static void usb_aspeed_isr(void)
{
	uint32_t isr_reg = REG_BASE + ASPEED_USB_ISR;
	uint32_t isr = sys_read32(isr_reg);
	uint32_t ep_isr;
	int i;

	if (!(isr & 0x1ffff))
		return;

	LOG_DBG("irq status:0x%x, ep_acks:0x%x, ep_naks:0x%x", isr,
		sys_read32(REG_BASE + ASPEED_USB_EP_ACK_ISR),
		sys_read32(REG_BASE + ASPEED_USB_EP_NAK_ISR));

	if (isr & ISR_BUS_RESET) {
		LOG_DBG("ISR_BUS_RESET");
		sys_write32(ISR_BUS_RESET, isr_reg);
	}

	if (isr & ISR_BUS_SUSPEND) {
		LOG_DBG("ISR_BUS_SUSPEND");
		sys_write32(ISR_BUS_SUSPEND, isr_reg);
	}

	if (isr & ISR_SUSPEND_RESUME) {
		LOG_DBG("ISR_SUSPEND_RESUME");
		sys_write32(ISR_SUSPEND_RESUME, isr_reg);
	}

	if (isr & ISR_EP0_IN_ACK_STALL) {
		LOG_DBG("EP0_IN_ACK_STALL");
		sys_write32(ISR_EP0_IN_ACK_STALL, isr_reg);

		LOG_DBG("tx_last:0x%x, tx_len:0x%x", dev_data.ep_data[0].tx_last, dev_data.ep_data[0].tx_len);
		if (dev_data.ep_data[0].tx_last != dev_data.ep_data[0].tx_len) {
			int tx_len = dev_data.ep_data[0].tx_len - dev_data.ep_data[0].tx_last;
			if (tx_len > USB_MAX_CTRL_MPS)
				tx_len = USB_MAX_CTRL_MPS;

			sys_write32(TO_PHY_ADDR(dev_data.ep_data[0].tx_dma + dev_data.ep_data[0].tx_last),
				    REG_BASE + ASPEED_USB_EP0_DATA_BUFF);
			sys_write32(EP0_TX_LEN(tx_len), REG_BASE + ASPEED_USB_EP0_CTRL);
			sys_write32(EP0_TX_LEN(tx_len) | EP0_TX_BUFF_RDY, REG_BASE + ASPEED_USB_EP0_CTRL);

			LOG_DBG("next tx trigger: [%d/%d]", tx_len, dev_data.ep_data[0].tx_len);
			dev_data.ep_data[0].tx_last += tx_len;

		} else if (!dev_data.ep_data[0].is_out) {
			// tx ready
			sys_write32(EP0_TX_BUFF_RDY, REG_BASE + ASPEED_USB_EP0_CTRL);

			// rx ready
			sys_write32(TO_PHY_ADDR(dev_data.ep_data[0].rx_dma), REG_BASE + ASPEED_USB_EP0_DATA_BUFF);
			sys_write32(EP0_RX_BUFF_RDY, REG_BASE + ASPEED_USB_EP0_CTRL);
		}

	}

	if (isr & ISR_EP0_OUT_NAK) {
		LOG_DBG("ISR_EP0_OUT_NAK");
		sys_write32(ISR_EP0_OUT_NAK, isr_reg);
	}

	if (isr & ISR_EP0_OUT_ACK_STALL) {
		LOG_DBG("ISR_EP0_OUT_ACK_STALL");
		sys_write32(ISR_EP0_OUT_ACK_STALL, isr_reg);
		sys_write32(EP0_TX_BUFF_RDY, REG_BASE + ASPEED_USB_EP0_CTRL);

		if (dev_data.ep_data[0].cb_out)
			dev_data.ep_data[0].cb_out(USB_EP_DIR_OUT, USB_DC_EP_DATA_OUT);
	}

	if (isr & ISR_EP0_IN_DATA_NAK) {
		LOG_DBG("ISR_EP0_IN_DATA_NAK");
		sys_write32(ISR_EP0_IN_DATA_NAK, isr_reg);
	}

	if (isr & ISR_EP0_SETUP) {
		LOG_DBG("ISR_EP0_SETUP");
		sys_write32(ISR_EP0_SETUP, isr_reg);
		aspeed_usb_setup_handle();
	}

	if (isr & ISR_EP_ACK_STALL) {
		LOG_DBG("ISR_EP_ACK_STALL");
		ep_isr = sys_read32(REG_BASE + ASPEED_USB_EP_ACK_ISR);
		for(i = 0; i < NUM_OF_EP_MAX; i++) {
			if (ep_isr & (0x1 << i)) {
				sys_write32(0x1 <<  i, REG_BASE + ASPEED_USB_EP_ACK_ISR);
				aspeed_usb_ep_handle(i + 1);
			}
		}
	}

	if (isr & ISR_EP_NAK) {
		LOG_DBG("ISR_EP_NAK");
		sys_write32(ISR_EP_NAK, isr_reg);
	}
}

static void usb_aspeed_init(void)
{
	const struct device *reset_dev;
	reset_control_subsys_t rst_id;

	LOG_INF("init");

	reset_dev = device_get_binding(ASPEED_RST_CTRL_NAME);
	rst_id = (reset_control_subsys_t)DT_RESETS_CELL(DT_NODELABEL(udc), rst_id);

	reset_control_deassert(reset_dev, rst_id);

	LOG_DBG("udc base addr:0x%x\n", REG_BASE);

	sys_write32(ROOT_PHY_CLK_EN | ROOT_PHY_RESET_DIS, REG_BASE + ASPEED_USB_CTRL);

	k_busy_wait(1000);
	sys_write32(0, REG_BASE + ASPEED_USB_DEV_RESET);

	sys_write32(0x1ffff, REG_BASE + ASPEED_USB_IER);
	sys_write32(0x7ffff, REG_BASE + ASPEED_USB_ISR);

	sys_write32(0x7ffff, REG_BASE + ASPEED_USB_EP_ACK_ISR);
	sys_write32(0x7ffff, REG_BASE + ASPEED_USB_EP_ACK_IER);

	sys_write32(0, REG_BASE + ASPEED_USB_EP0_CTRL);

	/* Connect and enable USB interrupt */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
		    usb_aspeed_isr, 0, 0);

	irq_enable(DT_INST_IRQN(0));
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

	usb_aspeed_init();

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
int usb_dc_detach(void);

/**
 * @brief Reset the USB device
 *
 * This function returns the USB device and firmware back to it's initial state.
 * N.B. the USB PLL is handled by the usb_detach function
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_reset(void);

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

	sys_write32(addr & 0x7f, REG_BASE + ASPEED_USB_CONF);

	/* IN data packet, status phase with zero byte data returned */
	sys_write32(EP0_TX_BUFF_RDY, REG_BASE + ASPEED_USB_EP0_CTRL);

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
		for (i = 0; i < NUM_OF_EP_MAX; i++)
			dev_data.ep_data[i].is_out = -1;

		dev_data.init = true;
	}

	if (ep_idx > NUM_OF_EP_MAX) {
		LOG_ERR("ep idx/addr out of range(%d), addr:0x%x, mps:0x%x, type:0x%x",
		NUM_OF_EP_MAX, cfg->ep_addr, cfg->ep_mps, cfg->ep_type);
		return -1;
	}

	if (ep_idx == 0U) {
		if (cfg->ep_type != USB_DC_EP_CONTROL) {
			LOG_ERR("ep[0x%x] pre-selected as control endpoint", cfg->ep_addr);
			return -1;
		}

		return 0;
	}

	if (dev_data.ep_data[ep_idx].is_out < 0) {
		if (USB_EP_GET_DIR(cfg->ep_addr) == USB_EP_DIR_IN) {
			LOG_INF("select ep[0x%x] as IN endpoint", cfg->ep_addr);
			dev_data.ep_data[ep_idx].is_out = 0;
		} else {
			LOG_INF("select ep[0x%x] as OUT endpoint", cfg->ep_addr);
			dev_data.ep_data[ep_idx].is_out = 1;
		}

	} else if (dev_data.ep_data[ep_idx].is_out == 0) {
		if (USB_EP_GET_DIR(cfg->ep_addr) != USB_EP_DIR_IN) {
			LOG_ERR("pre-selected ep[0x%x] as IN endpoint", cfg->ep_addr);
			return -EINVAL;
		}

	} else if (dev_data.ep_data[ep_idx].is_out == 1) {
		if (USB_EP_GET_DIR(cfg->ep_addr) != USB_EP_DIR_OUT) {
			LOG_ERR("pre-selected ep[0x%x] as OUT endpoint", cfg->ep_addr);
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

	LOG_INF("ep config: addr:0x%x, mps:0x%x, type:0x%x", cfg->ep_addr, cfg->ep_mps, cfg->ep_type);

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

	ep_reg = REG_BASE + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));

	sys_write32(EP_DMA_DESC_OP_RESET, ep_reg + ASPEED_EP_DMA_CTRL);
	sys_write32(EP_DMA_SINGLE_DESC, ep_reg + ASPEED_EP_DMA_CTRL);
	sys_write32(0x0, ep_reg + ASPEED_EP_DMA_STS);
	sys_write32(ep_conf, ep_reg + ASPEED_EP_CONFIG);

	LOG_DBG("ep[%d] config:%x", ep_num, sys_read32(ep_reg + ASPEED_EP_CONFIG));

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
	LOG_DBG("%s...", __func__);

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
	LOG_DBG("%s...", __func__);

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
	LOG_DBG("%s...", __func__);

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
int usb_dc_ep_halt(const uint8_t ep);

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
	uint32_t ep_reg;

	LOG_DBG("enable ep[0x%x]", ep);

	if (ep_num == 0)
		sys_write32(sys_read32(REG_BASE + ASPEED_USB_CTRL) | ROOT_UPSTREAM_EN, REG_BASE + ASPEED_USB_CTRL);
	else {
		ep_reg = REG_BASE + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));
		sys_write32(sys_read32(ep_reg + ASPEED_EP_CONFIG) | EP_ENABLE, ep_reg + ASPEED_EP_CONFIG);

		if (dev_data.ep_data[ep_num].is_out) {
			sys_write32(TO_PHY_ADDR(dev_data.ep_data[ep_num].rx_dma), ep_reg + ASPEED_EP_DMA_BUFF);
			sys_write32(0x1, ep_reg + ASPEED_EP_DMA_STS);
		}
	}

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
	LOG_DBG("%s...", __func__);

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
int usb_dc_ep_flush(const uint8_t ep);

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

	LOG_INF("[Write] ep:0x%x, data:0x%x, data_len:0x%x, ret_bytes:0x%x",
		(uint32_t)ep, (uint32_t)data, data_len, (uint32_t)ret_bytes);

	if (!data)
		return -EINVAL;

	ep_num = USB_EP_GET_IDX(ep);

	if (ep_num == 0) {
		LOG_DBG("trigger setup tx");

		dev_data.ep_data[0].tx_len = data_len;
		dev_data.ep_data[0].tx_dma = (uint8_t *)data;
		if (data_len > USB_MAX_CTRL_MPS)
			tx_len = USB_MAX_CTRL_MPS;
		else
			tx_len = data_len;

		LOG_DBG("trigger tx len: [%d/%d]", tx_len, data_len);
		dev_data.ep_data[0].tx_last = tx_len;

		sys_write32(TO_PHY_ADDR(data), REG_BASE + ASPEED_USB_EP0_DATA_BUFF);
		sys_write32(EP0_TX_LEN(tx_len), REG_BASE + ASPEED_USB_EP0_CTRL);
		sys_write32(EP0_TX_LEN(tx_len) | EP0_TX_BUFF_RDY, REG_BASE + ASPEED_USB_EP0_CTRL);

	} else {
		LOG_DBG("trigger ep tx");
		LOG_DBG("trigger tx len: [%d]", data_len);

		ep_reg = REG_BASE + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));

		sys_write32(TO_PHY_ADDR(data), ep_reg + ASPEED_EP_DMA_BUFF);
		sys_write32(EP_TX_LEN(data_len), ep_reg + ASPEED_EP_DMA_STS);
		sys_write32(EP_TX_LEN(data_len) | 0x1, ep_reg + ASPEED_EP_DMA_STS);

		*ret_bytes = data_len;
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
	struct usb_setup_packet *setup = (void *)(REG_BASE + ASPEED_USB_SETUP_DATA0);
	uint32_t ep_dma_sts;
	uint32_t ep_reg;
	int ep_num;
	int i;

	LOG_INF("[Read] ep:0x%x, data:0x%x, max_data_len:0x%x, read_bytes:0x%x",
		(uint32_t)ep, (uint32_t)data, max_data_len, (uint32_t)read_bytes);

	if (!data)
		return -EINVAL;

	ep_num = USB_EP_GET_IDX(ep);

	if (ep_num == 0) {
		LOG_DBG("trigger setup rx");
		LOG_DBG("setup: bmRequestType:0x%x, bRequest:0x%x, wValue:0x%x, wIndex:0x%x, wLength:0x%x",
			setup->bmRequestType, setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);
		memcpy(data, setup, max_data_len);

		if (dev_data.ep_data[0].is_out && setup->wLength) {
			sys_write32(TO_PHY_ADDR(data), REG_BASE + ASPEED_USB_EP0_DATA_BUFF);
			sys_write32(EP0_RX_BUFF_RDY, REG_BASE + ASPEED_USB_EP0_CTRL);
		}

	} else {
		LOG_DBG("trigger ep rx");

		ep_reg = REG_BASE + ASPEED_EP_OFFSET + (0x10 * (ep_num - 1));
		ep_dma_sts = sys_read32(ep_reg + ASPEED_EP_DMA_STS);

		*read_bytes = (ep_dma_sts >> 16) & 0x7ff;
		LOG_INF("ep[%d] read 0x%x bytes", ep_num, (uint32_t)*read_bytes);

		for (i = 0; i < *read_bytes; i++)
			LOG_INF("char: %c", dev_data.ep_data[ep_num].rx_dma[i]);

		if (*read_bytes < 1024)
			memcpy(data, dev_data.ep_data[ep_num].rx_dma, *read_bytes);
		else
			return -EINVAL;

		sys_write32(0x1, ep_reg + ASPEED_EP_DMA_STS);
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
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	LOG_DBG("ep[0x%x] set callback", ep);

	if (ep_idx >= NUM_OF_EP_MAX) {
		LOG_ERR("Wrong endpoint addr:0x%x", ep_idx);
		return -EINVAL;
	}

	if (!dev_data.attached)
		return -ENODEV;

	if (ep & USB_EP_DIR_IN)
		dev_data.ep_data[ep_idx].cb_in = cb;
	else
		dev_data.ep_data[ep_idx].cb_out = cb;

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
	return usb_dc_ep_read(ep, data, max_data_len, read_bytes);
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
	LOG_DBG("do nothing");

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
	LOG_DBG("ep[%d] mps: 0x%x", ep, dev_data.ep_data[ep].mps);

	return dev_data.ep_data[ep].mps;
}

/**
 * @brief Start the host wake up procedure.
 *
 * Function to wake up the host if it's currently in sleep mode.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usb_dc_wakeup_request(void);

