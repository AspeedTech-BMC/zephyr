/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_udc

#include "udc_common.h"

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/net/buf.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>

#include "soc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_aspeed, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define ASPEED_USB_CTRL			0x00
#define ASPEED_USB_CONF			0x04
#define ASPEED_USB_IER			0x08
#define ASPEED_USB_ISR			0x0C
#define ASPEED_USB_EP_ACK_IER		0x10
#define ASPEED_USB_EP_ACK_ISR		0x18
#define ASPEED_USB_DEV_RESET		0x20
#define ASPEED_USB_EP0_CTRL		0x30
#define ASPEED_USB_EP0_DATA_BUFF	0x34
#define ASPEED_USB_SETUP_DATA0		0x80
#define ASPEED_USB_PHY_CTRL0		0x800

#define ASPEED_EP_OFFSET		0x200
#define ASPEED_EP_CONFIG		0x00
#define ASPEED_EP_DMA_CTRL		0x04
#define ASPEED_EP_DMA_BUFF		0x08
#define ASPEED_EP_DMA_STS		0x0C

#define ROOT_PHY_CLK_EN			BIT(31)
#define ROOT_PHY_RESET_DIS		BIT(11)
#define ROOT_UPSTREAM_EN		BIT(0)

#define ISR_EP_NAK			BIT(17)
#define ISR_EP_ACK_STALL		BIT(16)
#define ISR_SUSPEND_RESUME		BIT(8)
#define ISR_BUS_SUSPEND			BIT(7)
#define ISR_BUS_RESET			BIT(6)
#define ISR_EP0_IN_DATA_NAK		BIT(4)
#define ISR_EP0_IN_ACK_STALL		BIT(3)
#define ISR_EP0_OUT_NAK			BIT(2)
#define ISR_EP0_OUT_ACK_STALL		BIT(1)
#define ISR_EP0_SETUP			BIT(0)
#define IRQ_ACK_ALL			0x301ff

#define EP_POOL_RESET			BIT(9)
#define DMA_CTRL_RESET			BIT(8)
#define ROOT_UBD_RESET			BIT(0)

#define EP0_GET_RX_LEN(x)		(((x) >> 16) & 0x7f)
#define EP0_TX_LEN(x)			(((x) & 0x7f) << 8)
#define EP0_RX_BUFF_RDY			BIT(2)
#define EP0_TX_BUFF_RDY			BIT(1)
#define EP0_STALL			BIT(0)

#define EP_SET_MAX_PKT(x)		(((x) & 0x3ff) << 16)
#define EP_SET_EP_STALL			BIT(12)
#define EP_SET_EP_NUM(x)		(((x) & 0xf) << 8)
#define EP_TYPE_BULK_IN			(0x2 << 4)
#define EP_TYPE_BULK_OUT		(0x3 << 4)
#define EP_TYPE_INT_IN			(0x4 << 4)
#define EP_TYPE_INT_OUT			(0x5 << 4)
#define EP_TYPE_ISO_IN			(0x6 << 4)
#define EP_TYPE_ISO_OUT			(0x7 << 4)
#define EP_ENABLE			BIT(0)

#define EP_DMA_DESC_OP_RESET		BIT(2)
#define EP_DMA_SINGLE_DESC		BIT(1)
#define EP_TX_LEN(x)			(((x) & 0x7ff) << 16)

#define PHY_CTRL0_8_BITS_UTMI		BIT(8)

#define RX_DMA_BUFF_SIZE		1024
#define ASPEED_UDC_MAX_HW_EP		22
#define ASPEED_UDC_MAX_USB_EP		16
#define ASPEED_EP_UNMAPPED		0xff

#define ASPEED_EP_LUT_IDX(ep) \
	(USB_EP_DIR_IS_IN(ep) ? (USB_EP_GET_IDX(ep) + 16) : USB_EP_GET_IDX(ep))
#define ASPEED_UDC_LOGICAL_EP_COUNT(n) \
	MIN(DT_INST_PROP(n, num_bidir_endpoints), ASPEED_UDC_MAX_USB_EP)

enum aspeed_udc_event_type {
	ASPEED_UDC_EVT_SETUP,
	ASPEED_UDC_EVT_EP0_IN,
	ASPEED_UDC_EVT_EP0_OUT,
	ASPEED_UDC_EVT_EP,
};

struct aspeed_udc_event {
	enum aspeed_udc_event_type type;
	uint8_t hw_ep;
};

struct aspeed_udc_ep_data {
	uint8_t addr;
	uint8_t hw_ep;
	bool is_out;
	uint16_t mps;
	uint16_t tx_last;
	uint8_t *rx_dma;
};

struct aspeed_udc_data {
	const struct device *dev;
	struct aspeed_udc_ep_data *ep_data;
	uint8_t (*rx_dma)[RX_DMA_BUFF_SIZE];
	uint8_t *ep_map;
	uint8_t *event_buffer;
	uint8_t free_ep_idx;
	uint8_t max_epns;
	bool attached;
	struct k_work work;
	struct k_msgq event_msgq;
};

struct aspeed_udc_config {
	uintptr_t base;
	const struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	clock_control_subsys_t clk_id;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	uint8_t max_epns;
	uint8_t num_usb_eps;
	uint32_t irq;
	void (*irq_config_func)(void);
};

static inline const struct aspeed_udc_config *aspeed_udc_get_config(const struct device *dev)
{
	return dev->config;
}

static inline struct aspeed_udc_data *aspeed_udc_get_data(const struct device *dev)
{
	return udc_get_private(dev);
}

static inline uintptr_t aspeed_udc_base(const struct device *dev)
{
	const struct aspeed_udc_config *config = aspeed_udc_get_config(dev);

	return config->base;
}

static inline uintptr_t aspeed_udc_ep_reg(const struct device *dev, uint8_t hw_ep)
{
	return aspeed_udc_base(dev) + ASPEED_EP_OFFSET + (0x10 * (hw_ep - 1));
}

static uint32_t aspeed_udc_ep_ack_mask(const struct aspeed_udc_data *priv)
{
	if (priv->max_epns <= 1) {
		return 0;
	}

	return BIT_MASK(priv->max_epns - 1);
}

static void aspeed_udc_ep0_rx(const struct device *dev, uint32_t offset)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	uintptr_t base = aspeed_udc_base(dev);
	uint32_t dma_addr;

	dma_addr = (uint32_t)priv->ep_data[0].rx_dma + offset;
	sys_write32(dma_addr, base + ASPEED_USB_EP0_DATA_BUFF);
	sys_write32(EP0_RX_BUFF_RDY, base + ASPEED_USB_EP0_CTRL);
}

static void aspeed_udc_ep0_tx(const struct device *dev, uint32_t tx_len)
{
	uintptr_t base = aspeed_udc_base(dev);

	sys_write32(EP0_TX_LEN(tx_len), base + ASPEED_USB_EP0_CTRL);
	sys_write32(EP0_TX_LEN(tx_len) | EP0_TX_BUFF_RDY,
		    base + ASPEED_USB_EP0_CTRL);
}

static uint8_t aspeed_udc_hw_ep_get(struct aspeed_udc_data *priv, uint8_t ep)
{
	return priv->ep_map[ASPEED_EP_LUT_IDX(ep)];
}

static int aspeed_udc_hw_ep_alloc(struct aspeed_udc_data *priv, uint8_t ep)
{
	uint8_t lut_idx = ASPEED_EP_LUT_IDX(ep);
	uint8_t hw_ep = priv->ep_map[lut_idx];

	if (hw_ep != ASPEED_EP_UNMAPPED) {
		return hw_ep;
	}

	if (priv->free_ep_idx >= priv->max_epns) {
		LOG_ERR("no hardware endpoint left for 0x%02x", ep);
		return -ENODEV;
	}

	hw_ep = priv->free_ep_idx++;
	priv->ep_map[lut_idx] = hw_ep;

	return hw_ep;
}

static int aspeed_udc_event_submit(const struct device *dev,
				   enum aspeed_udc_event_type type,
				   uint8_t hw_ep)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	struct aspeed_udc_event event = {
		.type = type,
		.hw_ep = hw_ep,
	};
	int ret;

	ret = k_msgq_put(&priv->event_msgq, &event, K_NO_WAIT);
	if (ret != 0) {
		udc_submit_event(dev, UDC_EVT_ERROR, ret);
		return ret;
	}

	k_work_submit_to_queue(udc_get_work_q(), &priv->work);

	return 0;
}

static int aspeed_udc_start_in(const struct device *dev,
			       struct aspeed_udc_ep_data *ep_data,
			       struct net_buf *buf)
{
	uint16_t tx_len;

	tx_len = MIN(buf->len, ep_data->mps);
	ep_data->tx_last = tx_len;

	if (ep_data->hw_ep == 0) {
		sys_write32(TO_PHY_ADDR((uintptr_t)buf->data),
			    aspeed_udc_base(dev) + ASPEED_USB_EP0_DATA_BUFF);
		aspeed_udc_ep0_tx(dev, tx_len);
	} else {
		uintptr_t ep_reg = aspeed_udc_ep_reg(dev, ep_data->hw_ep);

		sys_write32(TO_PHY_ADDR((uintptr_t)buf->data),
			    ep_reg + ASPEED_EP_DMA_BUFF);
		sys_write32(EP_TX_LEN(tx_len), ep_reg + ASPEED_EP_DMA_STS);
		sys_write32(EP_TX_LEN(tx_len) | 0x1,
			    ep_reg + ASPEED_EP_DMA_STS);
	}

	return 0;
}

static int aspeed_udc_start_out(const struct device *dev,
				struct aspeed_udc_ep_data *ep_data,
				uint32_t offset)
{
	if (ep_data->hw_ep == 0) {
		aspeed_udc_ep0_rx(dev, offset);
	} else {
		uintptr_t ep_reg = aspeed_udc_ep_reg(dev, ep_data->hw_ep);

		sys_write32(TO_PHY_ADDR((uintptr_t)ep_data->rx_dma),
			    ep_reg + ASPEED_EP_DMA_BUFF);
		sys_write32(0x1, ep_reg + ASPEED_EP_DMA_STS);
	}

	return 0;
}

static int aspeed_udc_start_next(const struct device *dev, uint8_t ep)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, ep);
	struct aspeed_udc_ep_data *ep_data;
	struct net_buf *buf;
	uint8_t hw_ep;

	if (!cfg || cfg->stat.halted || udc_ep_is_busy(dev, ep)) {
		return 0;
	}

	buf = udc_buf_peek(dev, ep);
	if (!buf) {
		return -ENODATA;
	}

	hw_ep = aspeed_udc_hw_ep_get(priv, ep);
	if (hw_ep == ASPEED_EP_UNMAPPED || hw_ep >= priv->max_epns) {
		return -ENODEV;
	}

	ep_data = &priv->ep_data[hw_ep];
	if (USB_EP_DIR_IS_IN(ep)) {
		aspeed_udc_start_in(dev, ep_data, buf);
	} else {
		aspeed_udc_start_out(dev, ep_data, 0);
	}

	udc_ep_set_busy(dev, ep, true);

	return 0;
}

static int aspeed_udc_ctrl_feed_dout(const struct device *dev, size_t length)
{
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT,
			     length == 0 ? USB_CONTROL_EP_MPS : length);
	if (!buf) {
		return -ENOMEM;
	}

	udc_buf_put(cfg, buf);
	aspeed_udc_start_out(dev, &aspeed_udc_get_data(dev)->ep_data[0], 0);
	udc_ep_set_busy(dev, USB_CONTROL_EP_OUT, true);

	return 0;
}

static int aspeed_udc_handle_setup(const struct device *dev)
{
	struct usb_setup_packet *setup;
	struct net_buf *buf;
	int ret = 0;

	setup = (void *)(aspeed_udc_base(dev) + ASPEED_USB_SETUP_DATA0);

	if (udc_ctrl_stage_is_status_out(dev)) {
		buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
		if (buf) {
			udc_ep_set_busy(dev, USB_CONTROL_EP_OUT, false);
			net_buf_unref(buf);
		}
	}

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT,
			     sizeof(struct usb_setup_packet));
	if (!buf) {
		return -ENOMEM;
	}

	net_buf_add_mem(buf, setup, sizeof(struct usb_setup_packet));
	udc_ep_buf_set_setup(buf);

	LOG_HEXDUMP_DBG(buf->data, buf->len, "setup");

	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_data_out(dev)) {
		ret = aspeed_udc_ctrl_feed_dout(dev, udc_data_stage_length(buf));
		if (ret == -ENOMEM) {
			ret = udc_submit_ep_event(dev, buf, ret);
		}
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		ret = udc_ctrl_submit_s_in_status(dev);
	} else {
		ret = udc_ctrl_submit_s_status(dev);
	}

	return ret;
}

static int aspeed_udc_handle_in(const struct device *dev, uint8_t ep)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	struct aspeed_udc_ep_data *ep_data;
	struct net_buf *buf;
	uint8_t hw_ep;

	hw_ep = aspeed_udc_hw_ep_get(priv, ep);
	if (hw_ep == ASPEED_EP_UNMAPPED || hw_ep >= priv->max_epns) {
		return -ENODEV;
	}

	ep_data = &priv->ep_data[hw_ep];
	buf = udc_buf_peek(dev, ep);
	if (!buf) {
		return -ENODATA;
	}

	if (ep_data->tx_last != 0) {
		net_buf_pull(buf, MIN(ep_data->tx_last, buf->len));
	}

	if (buf->len != 0) {
		return aspeed_udc_start_in(dev, ep_data, buf);
	}

	if (udc_ep_buf_has_zlp(buf)) {
		udc_ep_buf_clear_zlp(buf);
		return aspeed_udc_start_in(dev, ep_data, buf);
	}

	buf = udc_buf_get(dev, ep);
	udc_ep_set_busy(dev, ep, false);

	if (ep == USB_CONTROL_EP_IN) {
		if (udc_ctrl_stage_is_status_in(dev) ||
		    udc_ctrl_stage_is_no_data(dev)) {
			udc_ctrl_submit_status(dev, buf);
		}

		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_out(dev)) {
			net_buf_unref(buf);
			return aspeed_udc_ctrl_feed_dout(dev, USB_CONTROL_EP_MPS);
		}

		return 0;
	}

	udc_submit_ep_event(dev, buf, 0);
	aspeed_udc_start_next(dev, ep);

	return 0;
}

static int aspeed_udc_handle_out(const struct device *dev, uint8_t ep)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	struct aspeed_udc_ep_data *ep_data;
	struct udc_ep_config *cfg;
	struct net_buf *buf;
	uint32_t data_len;
	uint8_t hw_ep;

	hw_ep = aspeed_udc_hw_ep_get(priv, ep);
	if (hw_ep == ASPEED_EP_UNMAPPED || hw_ep >= priv->max_epns) {
		return -ENODEV;
	}

	ep_data = &priv->ep_data[hw_ep];
	buf = udc_buf_peek(dev, ep);
	if (!buf) {
		return -ENODATA;
	}

	if (hw_ep == 0) {
		data_len = EP0_GET_RX_LEN(sys_read32(aspeed_udc_base(dev) +
						     ASPEED_USB_EP0_CTRL));
	} else {
		uintptr_t ep_reg = aspeed_udc_ep_reg(dev, hw_ep);
		uint32_t ep_dma_sts = sys_read32(ep_reg + ASPEED_EP_DMA_STS);

		data_len = (ep_dma_sts >> 16) & 0x7ff;
	}

	if (data_len > net_buf_tailroom(buf)) {
		data_len = net_buf_tailroom(buf);
	}

	if (data_len != 0) {
		net_buf_add_mem(buf, ep_data->rx_dma, data_len);
	}

	cfg = udc_get_ep_cfg(dev, ep);
	if (!cfg) {
		return -ENODEV;
	}

	if (net_buf_tailroom(buf) != 0 && data_len == cfg->mps) {
		uint32_t offset = buf->len % RX_DMA_BUFF_SIZE;

		return aspeed_udc_start_out(dev, ep_data, offset);
	}

	buf = udc_buf_get(dev, ep);
	udc_ep_set_busy(dev, ep, false);

	if (ep == USB_CONTROL_EP_OUT) {
		if (udc_ctrl_stage_is_status_out(dev)) {
			udc_ctrl_submit_status(dev, buf);
			udc_ctrl_update_stage(dev, buf);
			return 0;
		}

		udc_ctrl_update_stage(dev, buf);
		if (udc_ctrl_stage_is_status_in(dev)) {
			return udc_ctrl_submit_s_out_status(dev, buf);
		}

		return 0;
	}

	udc_submit_ep_event(dev, buf, 0);
	aspeed_udc_start_next(dev, ep);

	return 0;
}

static void aspeed_udc_work_handler(struct k_work *work)
{
	struct aspeed_udc_data *priv = CONTAINER_OF(work, struct aspeed_udc_data, work);
	const struct device *dev = priv->dev;
	struct aspeed_udc_event event;
	int ret;

	while (k_msgq_get(&priv->event_msgq, &event, K_NO_WAIT) == 0) {
		ret = 0;

		switch (event.type) {
		case ASPEED_UDC_EVT_SETUP:
			ret = aspeed_udc_handle_setup(dev);
			break;
		case ASPEED_UDC_EVT_EP0_IN:
			ret = aspeed_udc_handle_in(dev, USB_CONTROL_EP_IN);
			break;
		case ASPEED_UDC_EVT_EP0_OUT:
			ret = aspeed_udc_handle_out(dev, USB_CONTROL_EP_OUT);
			break;
		case ASPEED_UDC_EVT_EP:
			if (event.hw_ep >= priv->max_epns) {
				ret = -EINVAL;
				break;
			}

			if (priv->ep_data[event.hw_ep].is_out) {
				ret = aspeed_udc_handle_out(dev,
						priv->ep_data[event.hw_ep].addr);
			} else {
				ret = aspeed_udc_handle_in(dev,
						priv->ep_data[event.hw_ep].addr);
			}
			break;
		default:
			ret = -EINVAL;
			break;
		}

		if (ret != 0 && ret != -ENODATA) {
			udc_submit_event(dev, UDC_EVT_ERROR, ret);
		}
	}
}

static void aspeed_udc_bus_reset(const struct device *dev)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);

	k_msgq_purge(&priv->event_msgq);

	for (uint8_t i = 0; i < priv->max_epns; i++) {
		priv->ep_data[i].tx_last = 0;
	}

	for (uint8_t i = 0; i < ASPEED_UDC_MAX_USB_EP; i++) {
		struct udc_ep_config *cfg;

		cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | i);
		if (cfg) {
			cfg->stat.busy = false;
		}

		cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | i);
		if (cfg) {
			cfg->stat.busy = false;
		}
	}

	sys_write32(0, aspeed_udc_base(dev) + ASPEED_USB_EP0_CTRL);
	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static void aspeed_udc_isr(const void *arg)
{
	const struct device *dev = arg;
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	uintptr_t base = aspeed_udc_base(dev);
	uint32_t isr_reg = base + ASPEED_USB_ISR;
	uint32_t isr = sys_read32(isr_reg);
	uint32_t ep_isr;

	if ((isr & IRQ_ACK_ALL) == 0) {
		return;
	}

	if (isr & ISR_BUS_RESET) {
		sys_write32(ISR_BUS_RESET, isr_reg);
		aspeed_udc_bus_reset(dev);
	}

	if (isr & ISR_BUS_SUSPEND) {
		sys_write32(ISR_BUS_SUSPEND, isr_reg);
		udc_set_suspended(dev, true);
		udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
	}

	if (isr & ISR_SUSPEND_RESUME) {
		sys_write32(ISR_SUSPEND_RESUME, isr_reg);
		udc_set_suspended(dev, false);
		udc_submit_event(dev, UDC_EVT_RESUME, 0);
	}

	if (isr & ISR_EP0_IN_ACK_STALL) {
		sys_write32(ISR_EP0_IN_ACK_STALL, isr_reg);
		aspeed_udc_event_submit(dev, ASPEED_UDC_EVT_EP0_IN, 0);
	}

	if (isr & ISR_EP0_OUT_NAK) {
		sys_write32(ISR_EP0_OUT_NAK, isr_reg);
	}

	if (isr & ISR_EP0_OUT_ACK_STALL) {
		sys_write32(ISR_EP0_OUT_ACK_STALL, isr_reg);
		aspeed_udc_event_submit(dev, ASPEED_UDC_EVT_EP0_OUT, 0);
	}

	if (isr & ISR_EP0_IN_DATA_NAK) {
		sys_write32(ISR_EP0_IN_DATA_NAK, isr_reg);
	}

	if (isr & ISR_EP_ACK_STALL) {
		ep_isr = sys_read32(base + ASPEED_USB_EP_ACK_ISR);
		for (uint8_t i = 1; i < priv->max_epns; i++) {
			if (ep_isr & BIT(i - 1)) {
				sys_write32(BIT(i - 1),
					    base + ASPEED_USB_EP_ACK_ISR);
				aspeed_udc_event_submit(dev, ASPEED_UDC_EVT_EP, i);
			}
		}
	}

	if (isr & ISR_EP0_SETUP) {
		sys_write32(ISR_EP0_SETUP, isr_reg);
		aspeed_udc_event_submit(dev, ASPEED_UDC_EVT_SETUP, 0);
	}

	if (isr & ISR_EP_NAK) {
		sys_write32(ISR_EP_NAK, isr_reg);
	}
}

static int aspeed_udc_set_address(const struct device *dev, const uint8_t addr)
{
	sys_write32(addr & 0x7f, aspeed_udc_base(dev) + ASPEED_USB_CONF);

	return 0;
}

static int aspeed_udc_ep_enqueue(const struct device *dev,
				 struct udc_ep_config *const cfg,
				 struct net_buf *const buf)
{
	udc_buf_put(cfg, buf);

	if (!cfg->stat.halted) {
		aspeed_udc_start_next(dev, cfg->addr);
	}

	return 0;
}

static int aspeed_udc_ep_dequeue(const struct device *dev,
				 struct udc_ep_config *const cfg)
{
	struct net_buf *buf;

	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	udc_ep_set_busy(dev, cfg->addr, false);

	return 0;
}

static int aspeed_udc_ep_set_halt(const struct device *dev,
				  struct udc_ep_config *const cfg)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	uint8_t hw_ep = aspeed_udc_hw_ep_get(priv, cfg->addr);

	if (hw_ep == ASPEED_EP_UNMAPPED || hw_ep >= priv->max_epns) {
		return -ENODEV;
	}

	if (hw_ep == 0) {
		sys_write32(EP0_STALL, aspeed_udc_base(dev) + ASPEED_USB_EP0_CTRL);
	} else {
		uintptr_t ep_reg = aspeed_udc_ep_reg(dev, hw_ep);

		sys_write32(sys_read32(ep_reg + ASPEED_EP_CONFIG) | EP_SET_EP_STALL,
			    ep_reg + ASPEED_EP_CONFIG);
	}

	cfg->stat.halted = true;

	return 0;
}

static int aspeed_udc_ep_clear_halt(const struct device *dev,
				    struct udc_ep_config *const cfg)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	uint8_t hw_ep = aspeed_udc_hw_ep_get(priv, cfg->addr);

	if (hw_ep == ASPEED_EP_UNMAPPED || hw_ep >= priv->max_epns) {
		return -ENODEV;
	}

	if (hw_ep == 0) {
		uintptr_t ep_reg = aspeed_udc_base(dev) + ASPEED_USB_EP0_CTRL;

		sys_write32(sys_read32(ep_reg) & ~EP0_STALL, ep_reg);
	} else {
		uintptr_t ep_reg = aspeed_udc_ep_reg(dev, hw_ep);

		sys_write32(sys_read32(ep_reg + ASPEED_EP_CONFIG) & ~EP_SET_EP_STALL,
			    ep_reg + ASPEED_EP_CONFIG);
	}

	cfg->stat.halted = false;

	return 0;
}

static uint32_t aspeed_udc_ep_type(const struct udc_ep_config *cfg)
{
	bool in = USB_EP_DIR_IS_IN(cfg->addr);

	switch (cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_BULK:
		return in ? EP_TYPE_BULK_IN : EP_TYPE_BULK_OUT;
	case USB_EP_TYPE_INTERRUPT:
		return in ? EP_TYPE_INT_IN : EP_TYPE_INT_OUT;
	case USB_EP_TYPE_ISO:
		return in ? EP_TYPE_ISO_IN : EP_TYPE_ISO_OUT;
	default:
		return 0;
	}
}

static int aspeed_udc_ep_enable(const struct device *dev,
				struct udc_ep_config *const cfg)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	struct aspeed_udc_ep_data *ep_data;
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	uint32_t ep_conf;
	uint8_t hw_ep;
	int ret;

	if (ep_idx == 0) {
		priv->ep_map[ASPEED_EP_LUT_IDX(cfg->addr)] = 0;
		ep_data = &priv->ep_data[0];
		ep_data->addr = cfg->addr;
		ep_data->hw_ep = 0;
		ep_data->is_out = USB_EP_DIR_IS_OUT(cfg->addr);
		ep_data->mps = cfg->mps;
		return 0;
	}

	ret = aspeed_udc_hw_ep_alloc(priv, cfg->addr);
	if (ret < 0) {
		return ret;
	}

	hw_ep = ret;
	ep_data = &priv->ep_data[hw_ep];
	ep_data->addr = cfg->addr;
	ep_data->hw_ep = hw_ep;
	ep_data->is_out = USB_EP_DIR_IS_OUT(cfg->addr);
	ep_data->mps = cfg->mps;

	ep_conf = EP_SET_MAX_PKT(cfg->mps) | EP_SET_EP_NUM(ep_idx) |
		  aspeed_udc_ep_type(cfg);
	if (ep_conf == (EP_SET_MAX_PKT(cfg->mps) | EP_SET_EP_NUM(ep_idx))) {
		return -ENOTSUP;
	}

	uintptr_t ep_reg = aspeed_udc_ep_reg(dev, hw_ep);

	sys_write32(EP_DMA_DESC_OP_RESET, ep_reg + ASPEED_EP_DMA_CTRL);
	sys_write32(EP_DMA_SINGLE_DESC, ep_reg + ASPEED_EP_DMA_CTRL);
	sys_write32(0x0, ep_reg + ASPEED_EP_DMA_STS);
	sys_write32(ep_conf | EP_ENABLE, ep_reg + ASPEED_EP_CONFIG);

	sys_write32(sys_read32(aspeed_udc_base(dev) + ASPEED_USB_EP_ACK_IER) |
		    BIT(hw_ep - 1),
		    aspeed_udc_base(dev) + ASPEED_USB_EP_ACK_IER);

	return 0;
}

static int aspeed_udc_ep_disable(const struct device *dev,
				 struct udc_ep_config *const cfg)
{
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	uint8_t hw_ep = aspeed_udc_hw_ep_get(priv, cfg->addr);

	if (hw_ep == ASPEED_EP_UNMAPPED || hw_ep >= priv->max_epns) {
		return -ENODEV;
	}

	udc_ep_set_busy(dev, cfg->addr, false);

	if (hw_ep > 0) {
		uintptr_t ep_reg = aspeed_udc_ep_reg(dev, hw_ep);
		uintptr_t base = aspeed_udc_base(dev);

		sys_write32(sys_read32(base + ASPEED_USB_EP_ACK_IER) &
			    ~BIT(hw_ep - 1), base + ASPEED_USB_EP_ACK_IER);
		sys_write32(sys_read32(ep_reg + ASPEED_EP_CONFIG) & ~EP_ENABLE,
			    ep_reg + ASPEED_EP_CONFIG);
	}

	return 0;
}

static int aspeed_udc_enable(const struct device *dev)
{
	const struct aspeed_udc_config *config = aspeed_udc_get_config(dev);
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	uintptr_t base = aspeed_udc_base(dev);

	if (priv->attached) {
		return 0;
	}

	irq_enable(config->irq);
	sys_write32(sys_read32(base + ASPEED_USB_CTRL) | ROOT_UPSTREAM_EN,
		    base + ASPEED_USB_CTRL);

	priv->attached = true;

	return 0;
}

static int aspeed_udc_disable(const struct device *dev)
{
	const struct aspeed_udc_config *config = aspeed_udc_get_config(dev);
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	uintptr_t base = aspeed_udc_base(dev);

	if (!priv->attached) {
		return 0;
	}

	sys_write32(sys_read32(base + ASPEED_USB_CTRL) & ~ROOT_UPSTREAM_EN,
		    base + ASPEED_USB_CTRL);
	irq_disable(config->irq);
	k_msgq_purge(&priv->event_msgq);

	priv->attached = false;

	return 0;
}

static int aspeed_udc_init(const struct device *dev)
{
	const struct aspeed_udc_config *config = aspeed_udc_get_config(dev);
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	uintptr_t base = aspeed_udc_base(dev);
	int ret;

	if (config->clock_dev) {
		if (!device_is_ready(config->clock_dev)) {
			return -ENODEV;
		}

		ret = clock_control_on(config->clock_dev, config->clk_id);
		if (ret != 0) {
			return ret;
		}
	}

	if (!device_is_ready(config->reset.dev)) {
		return -ENODEV;
	}

	ret = reset_line_deassert_dt(&config->reset);
	if (ret != 0) {
		return ret;
	}

	k_busy_wait(1000);
	sys_write32(ROOT_PHY_CLK_EN | ROOT_PHY_RESET_DIS, base + ASPEED_USB_CTRL);

	k_busy_wait(1000);
	sys_write32(0, base + ASPEED_USB_DEV_RESET);

	sys_write32(0x0, base + ASPEED_USB_IER);
	sys_write32(IRQ_ACK_ALL, base + ASPEED_USB_ISR);
	sys_write32(0x0, base + ASPEED_USB_EP_ACK_IER);
	sys_write32(aspeed_udc_ep_ack_mask(priv), base + ASPEED_USB_EP_ACK_ISR);
	sys_write32(0, base + ASPEED_USB_EP0_CTRL);

	sys_write32(sys_read32(base + ASPEED_USB_PHY_CTRL0) | PHY_CTRL0_8_BITS_UTMI,
		    base + ASPEED_USB_PHY_CTRL0);

	sys_write32(ISR_EP_ACK_STALL |
		    ISR_SUSPEND_RESUME |
		    ISR_BUS_SUSPEND |
		    ISR_BUS_RESET |
		    ISR_EP0_IN_ACK_STALL |
		    ISR_EP0_OUT_ACK_STALL |
		    ISR_EP0_SETUP,
		    base + ASPEED_USB_IER);

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL,
				     USB_CONTROL_EP_MPS, 0);
	if (ret != 0) {
		return ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
				     USB_CONTROL_EP_MPS, 0);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int aspeed_udc_shutdown(const struct device *dev)
{
	(void)udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
	(void)udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);

	return 0;
}

static int aspeed_udc_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int aspeed_udc_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

static const struct udc_api aspeed_udc_api = {
	.ep_enqueue = aspeed_udc_ep_enqueue,
	.ep_dequeue = aspeed_udc_ep_dequeue,
	.ep_set_halt = aspeed_udc_ep_set_halt,
	.ep_clear_halt = aspeed_udc_ep_clear_halt,
	.ep_enable = aspeed_udc_ep_enable,
	.ep_disable = aspeed_udc_ep_disable,
	.set_address = aspeed_udc_set_address,
	.enable = aspeed_udc_enable,
	.disable = aspeed_udc_disable,
	.init = aspeed_udc_init,
	.shutdown = aspeed_udc_shutdown,
	.lock = aspeed_udc_lock,
	.unlock = aspeed_udc_unlock,
};

static int aspeed_udc_preinit(const struct device *dev)
{
	const struct aspeed_udc_config *config = aspeed_udc_get_config(dev);
	struct udc_data *data = dev->data;
	struct aspeed_udc_data *priv = aspeed_udc_get_data(dev);
	int ret;

	k_mutex_init(&data->mutex);
	k_work_init(&priv->work, aspeed_udc_work_handler);
	k_msgq_init(&priv->event_msgq, priv->event_buffer,
		    sizeof(struct aspeed_udc_event), CONFIG_UDC_ASPEED_EVENT_COUNT);

	priv->dev = dev;
	priv->max_epns = config->max_epns;
	priv->free_ep_idx = 1;
	priv->attached = false;

	for (uint8_t i = 0; i < 32; i++) {
		priv->ep_map[i] = ASPEED_EP_UNMAPPED;
	}

	priv->ep_map[ASPEED_EP_LUT_IDX(USB_CONTROL_EP_OUT)] = 0;
	priv->ep_map[ASPEED_EP_LUT_IDX(USB_CONTROL_EP_IN)] = 0;

	for (uint8_t i = 0; i < priv->max_epns; i++) {
		priv->ep_data[i].hw_ep = i;
		priv->ep_data[i].rx_dma = priv->rx_dma[i];
	}

	if (config->pcfg) {
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("Failed to apply default pinctrl state (%d)", ret);
			return ret;
		}
	}

	for (uint8_t i = 0; i < config->num_usb_eps; i++) {
		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		config->ep_cfg_out[i].caps.out = 1;
		config->ep_cfg_out[i].caps.mps = (i == 0) ? USB_CONTROL_EP_MPS : 1024;
		config->ep_cfg_out[i].caps.control = (i == 0);
		config->ep_cfg_out[i].caps.bulk = (i != 0);
		config->ep_cfg_out[i].caps.interrupt = (i != 0);
		config->ep_cfg_out[i].caps.iso = (i != 0);

		ret = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (ret != 0) {
			return ret;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		config->ep_cfg_in[i].caps.in = 1;
		config->ep_cfg_in[i].caps.mps = (i == 0) ? USB_CONTROL_EP_MPS : 1024;
		config->ep_cfg_in[i].caps.control = (i == 0);
		config->ep_cfg_in[i].caps.bulk = (i != 0);
		config->ep_cfg_in[i].caps.interrupt = (i != 0);
		config->ep_cfg_in[i].caps.iso = (i != 0);

		ret = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (ret != 0) {
			return ret;
		}
	}

	data->caps.hs = true;
	data->caps.rwup = true;
	data->caps.addr_before_status = true;
	data->caps.mps0 = UDC_MPS0_64;

	config->irq_config_func();

	return 0;
}

#define ASPEED_UDC_CLOCK_INIT(n)					       \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, clocks),			       \
		    (.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),       \
		     .clk_id = (clock_control_subsys_t)			       \
			       DT_INST_CLOCKS_CELL(n, clk_id),),		       \
		    (.clock_dev = NULL,				       \
		     .clk_id = CLOCK_CONTROL_SUBSYS_ALL,))

#define ASPEED_UDC_PINCTRL_DT_INST_DEFINE(n)				       \
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, default),		       \
		    (PINCTRL_DT_INST_DEFINE(n)), ())

#define ASPEED_UDC_PINCTRL_DT_INST_DEV_CONFIG_GET(n)			       \
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, default),		       \
		    (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL))

#define ASPEED_UDC_INIT(n)						       \
	BUILD_ASSERT(DT_INST_PROP(n, num_bidir_endpoints) <=		       \
		     ASPEED_UDC_MAX_HW_EP,				       \
		     "ASPEED UDC supports at most 22 hardware endpoints");      \
	static void aspeed_udc_irq_config_##n(void)			       \
	{								       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),	       \
			    aspeed_udc_isr, DEVICE_DT_INST_GET(n), 0);	       \
	}								       \
	ASPEED_UDC_PINCTRL_DT_INST_DEFINE(n);				       \
	static uint8_t aspeed_udc_rx_dma_##n			       \
		[DT_INST_PROP(n, num_bidir_endpoints)][RX_DMA_BUFF_SIZE]       \
		NON_CACHED_BSS_ALIGN16;					       \
	static struct aspeed_udc_ep_data aspeed_udc_ep_data_##n		       \
		[DT_INST_PROP(n, num_bidir_endpoints)];			       \
	static uint8_t aspeed_udc_ep_map_##n[32];			       \
	static uint8_t aspeed_udc_event_buffer_##n			       \
		[CONFIG_UDC_ASPEED_EVENT_COUNT *			       \
		 sizeof(struct aspeed_udc_event)];			       \
	static struct udc_ep_config aspeed_udc_ep_cfg_out_##n		       \
		[ASPEED_UDC_LOGICAL_EP_COUNT(n)];			       \
	static struct udc_ep_config aspeed_udc_ep_cfg_in_##n		       \
		[ASPEED_UDC_LOGICAL_EP_COUNT(n)];			       \
	static struct aspeed_udc_data aspeed_udc_priv_##n = {		       \
		.ep_data = aspeed_udc_ep_data_##n,			       \
		.rx_dma = aspeed_udc_rx_dma_##n,			       \
		.ep_map = aspeed_udc_ep_map_##n,			       \
		.event_buffer = aspeed_udc_event_buffer_##n,		       \
	};								       \
	static struct udc_data aspeed_udc_data_##n = {			       \
		.priv = &aspeed_udc_priv_##n,				       \
	};								       \
	static const struct aspeed_udc_config aspeed_udc_config_##n = {	       \
		.base = DT_INST_REG_ADDR(n),				       \
		.reset = RESET_DT_SPEC_INST_GET(n),			       \
		.pcfg = ASPEED_UDC_PINCTRL_DT_INST_DEV_CONFIG_GET(n),	       \
		.ep_cfg_in = aspeed_udc_ep_cfg_in_##n,			       \
		.ep_cfg_out = aspeed_udc_ep_cfg_out_##n,		       \
		.max_epns = DT_INST_PROP(n, num_bidir_endpoints),	       \
		.num_usb_eps = ASPEED_UDC_LOGICAL_EP_COUNT(n),		       \
		.irq = DT_INST_IRQN(n),					       \
		.irq_config_func = aspeed_udc_irq_config_##n,		       \
		ASPEED_UDC_CLOCK_INIT(n)				       \
	};								       \
	DEVICE_DT_INST_DEFINE(n, aspeed_udc_preinit, NULL,		       \
			      &aspeed_udc_data_##n, &aspeed_udc_config_##n,     \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,  \
			      &aspeed_udc_api);

DT_INST_FOREACH_STATUS_OKAY(ASPEED_UDC_INIT)
