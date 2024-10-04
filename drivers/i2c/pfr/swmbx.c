/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_swmbx_ctrl

#include <zephyr/sys/util.h>
#include <zephyr/sys/slist.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/drivers/i2c/pfr/swmbx.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(swmbx_ctrl);

struct swmbx_fifo {
	sys_snode_t list;
	uint8_t value;
};

struct swmbx_node {
	struct k_sem *sem_notify;
	uint8_t enable;
};

struct swmbx_fifo_data {
	struct k_sem *sem_fifo;
	struct swmbx_fifo *buffer;
	struct swmbx_fifo *current;
	sys_slist_t list_head;
	uint8_t fifo_offset;
	uint8_t enable;
	uint8_t notify_flag;
	uint8_t notify_start;
	uint8_t fifo_write;
	uint32_t msg_index;
	uint32_t max_msg_count;

	uint32_t volatile cur_msg_count;
};

struct swmbx_ctrl_data {
	uint32_t buffer_size;
	uint8_t *buffer;
	uint8_t mbx_en;
	struct swmbx_node node[SWMBX_DEV_COUNT][SWMBX_NODE_COUNT];
	struct swmbx_fifo_data fifo[SWMBX_FIFO_COUNT];
	bool mbx_fifo_execute[SWMBX_DEV_COUNT];
	uint8_t mbx_fifo_addr[SWMBX_DEV_COUNT];
	uint8_t mbx_fifo_idx[SWMBX_DEV_COUNT];
};

struct swmbx_ctrl_config {
	char *controller_dev_name;
	uint32_t buffer_size;
};

/* convenience defines */
#define DEV_CFG(dev)		\
	((const struct swmbx_ctrl_config *const)	\
	(dev)->config)
#define DEV_DATA(dev)	\
	((struct swmbx_ctrl_data *const)(dev)->data)

uint32_t *swmbx_info = (uint32_t *)(SWMBX_INFO_BASE);

/* internal api for pfr swmbx control */
int check_swmbx_fifo(struct swmbx_ctrl_data *data, uint8_t addr, uint8_t *index)
{
	for (uint8_t i = 0; i < SWMBX_FIFO_COUNT; i++) {
		if (data->fifo[i].fifo_offset == addr && data->fifo[i].enable &&
		data->fifo[i].sem_fifo) {
			*index = i;
			LOG_DBG("fifo: fifo inx %x and address %x", i, addr);
			return 1;
		}
	}

	return 0;
}

int append_fifo_write(struct swmbx_ctrl_data *data, uint8_t fifo_idx, uint8_t val)
{
	struct swmbx_fifo_data *fifo = NULL;

	/* find out the fifo item */
	fifo = &data->fifo[fifo_idx];

	/* check the max msg length */
	if (fifo->cur_msg_count < fifo->max_msg_count) {
		LOG_DBG("fifo: cur_msg_index %x", (uint32_t)(fifo->msg_index));

		fifo->current = &fifo->buffer[fifo->cur_msg_count];
		fifo->current->value = val;
		sys_slist_append(&fifo->list_head, &fifo->current->list);
		fifo->cur_msg_count++;

		/* bondary condition */
		if (fifo->msg_index == (fifo->max_msg_count - 1)) {
			fifo->msg_index = 0;
		} else {
			fifo->msg_index++;
		}

		LOG_DBG("fifo: slave write data->current %x", (uint32_t)(fifo->current));
	} else {
		return 1;
	}

	return 0;
}

int peak_fifo_read(struct swmbx_ctrl_data *data, uint8_t fifo_idx, uint8_t *val)
{
	struct swmbx_fifo_data *fifo = NULL;
	sys_snode_t *list_node = NULL;

	/* find out the fifo item */
	fifo = &data->fifo[fifo_idx];
	list_node = sys_slist_peek_head(&fifo->list_head);
	LOG_DBG("fifo: slave read %x", (uint32_t)list_node);

	if (list_node) {
		fifo->current = (struct swmbx_fifo *)(list_node);
		*val = fifo->current->value;
		LOG_DBG("fifo: slave read %x ", *val);

		/* remove this item from list */
		sys_slist_find_and_remove(&fifo->list_head, list_node);

		fifo->cur_msg_count--;
		LOG_DBG("fifo: fifo msg %x", fifo->cur_msg_count);
	} else {
		*val = 0;
		return 1;
	}

	return 0;
}

/* internal api define end */

/* external API for swmbx slave message exchange */
void swmbx_send_start(uint8_t port, uint8_t addr)
{
	/* check invlaid condition */
	if (port > SWMBX_DEV_COUNT) {
		LOG_DBG("swmbx: send start: invlaid port %d", port);
		return;
	}

	struct swmbx_ctrl_data *data = (struct swmbx_ctrl_data *)(*swmbx_info);
	uint8_t fifo_index = 0x0;

	LOG_DBG("swmbx: send start: swmbx data 0x%x", (uint32_t)data);
	LOG_DBG("swmbx: send start: addr 0x%x @ port 0x%x", addr, port);
	LOG_DBG("swmbx: mbx_en 0x%x", data->mbx_en);

	/* check swmbx fifo status */
	data->mbx_fifo_execute[port] = check_swmbx_fifo(data, addr, &fifo_index);

	/* fill the fifo executed index */
	if (data->mbx_fifo_execute[port]) {
		data->mbx_fifo_addr[port] = addr;
		data->mbx_fifo_idx[port] = fifo_index;
	}
}

void swmbx_send_msg(uint8_t port, uint8_t addr, uint8_t *val)
{
	/* check invlaid condition */
	if (port > SWMBX_DEV_COUNT) {
		LOG_DBG("swmbx: send msg: invlaid port %d", port);
		return;
	}

	struct swmbx_ctrl_data *data = (struct swmbx_ctrl_data *)(*swmbx_info);
	bool mbx_write_data = false;

	LOG_DBG("swmbx: send msg: addr 0x%x val 0x%x", addr, *val);

	/* check the FIFO is executed or not */
	if (data->mbx_fifo_execute[port] && (data->mbx_en & SWMBX_FIFO)) {
		uint8_t fifo_addr = data->mbx_fifo_addr[port];
		uint8_t fifo_index = data->mbx_fifo_idx[port];

		/* append value into fifo */
		if (append_fifo_write(data, fifo_index, *val)) {
			/* issue semaphore when fifo full */
			k_sem_give(data->fifo[fifo_index].sem_fifo);
			LOG_DBG("fifo: fifo full at %d group", fifo_index);
			return;
		}

		/* check fifo notify start*/
		if (data->mbx_en & SWMBX_NOTIFY &&
		data->fifo[fifo_index].notify_flag & SWMBX_FIFO_NOTIFY_START &&
		!data->fifo[fifo_index].notify_start) {
			if ((data->node[port][fifo_addr].enable & SWMBX_NOTIFY)) {
				k_sem_give(data->node[port][fifo_addr].sem_notify);
				data->fifo[fifo_index].notify_start = true;
			}
		}

		if (!data->fifo[fifo_index].fifo_write)
			data->fifo[fifo_index].fifo_write = true;
	} else {
		/* check write protect behavior */
		if (!(data->node[port][addr].enable & SWMBX_PROTECT) ||
		!(data->mbx_en & SWMBX_PROTECT)) {
			mbx_write_data = true;
		}

		/* check notify behavior */
		if (data->mbx_en & SWMBX_NOTIFY) {
			if ((data->node[port][addr].enable & SWMBX_NOTIFY)) {
				k_sem_give(data->node[port][addr].sem_notify);
			}
		}

		/* update value into common mbx buffer */
		if (mbx_write_data)
			data->buffer[addr] = *val;
	}
}

void swmbx_get_msg(uint8_t port, uint8_t addr, uint8_t *val)
{
	/* check invlaid condition */
	if (port > SWMBX_DEV_COUNT) {
		LOG_DBG("swmbx: send msg: invlaid port %d", port);
		return;
	}

	struct swmbx_ctrl_data *data = (struct swmbx_ctrl_data *)(*swmbx_info);

	/* check the FIFO is executed or not */
	if (data->mbx_fifo_execute[port] && (data->mbx_en & SWMBX_FIFO)) {
		uint8_t fifo_index = data->mbx_fifo_idx[port];

		/* peak value from fifo */
		if (peak_fifo_read(data, fifo_index, val)) {
			/* issue semaphore when fifo empty */
			k_sem_give(data->fifo[fifo_index].sem_fifo);
			LOG_DBG("fifo: fifo empty at %d group", fifo_index);
			return;
		}
	} else {
		*val = data->buffer[addr];
	}
	LOG_DBG("swmbx: get msg: addr 0x%x val 0x%x", addr, *val);
}

void swmbx_send_stop(uint8_t port)
{
	/* check invlaid condition */
	if (port > SWMBX_DEV_COUNT) {
		LOG_DBG("swmbx: send start: invlaid port %d", port);
		return;
	}

	struct swmbx_ctrl_data *data = (struct swmbx_ctrl_data *)(*swmbx_info);

	LOG_DBG("swmbx: send stop: @ port 0x%x", port);

	/* check fifo notify end*/
	if (data->mbx_fifo_execute[port]) {
		uint8_t fifo_addr = data->mbx_fifo_addr[port];
		uint8_t fifo_index = data->mbx_fifo_idx[port];

		if (data->mbx_en & SWMBX_NOTIFY &&
		data->fifo[fifo_index].notify_flag & SWMBX_FIFO_NOTIFY_STOP &&
		data->fifo[fifo_index].fifo_write) {
			if ((data->node[port][fifo_addr].enable & SWMBX_NOTIFY)) {
				k_sem_give(data->node[port][fifo_addr].sem_notify);
			}
		}

		data->fifo[fifo_index].notify_start = false;
		data->fifo[fifo_index].fifo_write = false;
		data->mbx_fifo_execute[port] = false;
		data->mbx_fifo_addr[port] = 0x0;
		data->mbx_fifo_idx[port] = 0x0;
	}
}

/* end external API for swmbx slave message exchange */

/* external API for swmbx access */
int swmbx_write(const struct device *dev, uint8_t fifo, uint8_t addr, uint8_t *val)
{
	if (!dev)
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;
	unsigned int key = 0;
	bool mbx_fifo_execute = false;
	uint8_t fifo_idx = 0;

	/* enter critical section swmbx access */
	if (!k_is_in_isr())
		key = irq_lock();

	if (fifo) {
		/*check fifo enable and find out fifo index*/
		mbx_fifo_execute = check_swmbx_fifo(data, addr, &fifo_idx);
		if (mbx_fifo_execute) {
			/* append value into fifo */
			if (append_fifo_write(data, fifo_idx, *val)) {
				LOG_DBG("swmbx_write: fifo full at %d group", fifo_idx);
				return 1;
			}
		} else {
			LOG_DBG("swmbx_write: could not find address %d fifo", addr);
			return 1;
		}
	} else {
		data->buffer[addr] = *val;
	}

	/* exit critical section */
	if (!k_is_in_isr())
		irq_unlock(key);

	return 0;
}

int swmbx_read(const struct device *dev, uint8_t fifo, uint8_t addr, uint8_t *val)
{
	if (!dev)
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;
	unsigned int key = 0;
	bool mbx_fifo_execute = false;
	uint8_t fifo_idx = 0;

	/* enter critical section swmbx access */
	if (!k_is_in_isr())
		key = irq_lock();

	if (fifo) {
		/*check fifo enable and find out fifo index*/
		mbx_fifo_execute = check_swmbx_fifo(data, addr, &fifo_idx);
		if (mbx_fifo_execute) {
			/* peak value from fifo */
			if (peak_fifo_read(data, fifo_idx, val)) {
				LOG_DBG("swmbx_read: fifo empty at %d group", fifo_idx);
				return 1;
			}
		} else {
			LOG_DBG("swmbx_read: could not find fifo %d group", addr);
			return 1;
		}
	} else {
		*val = data->buffer[addr];
	}

	/* exit critical section */
	if (!k_is_in_isr())
		irq_unlock(key);

	return 0;
}

/* general control */
int swmbx_enable_behavior(const struct device *dev, uint32_t item_flag, uint8_t enable)
{
	if (!dev || ((item_flag & FLAG_MASK) == 0))
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;

	if (enable) {
		data->mbx_en |= item_flag;
	} else {
		data->mbx_en &= ~(item_flag);
	}

	return 0;
}

/* apply swmbx write protect with bitmap and port index*/
int swmbx_apply_protect(const struct device *dev, uint8_t port,
uint32_t *bitmap, uint8_t start_idx, uint8_t num)
{
	/* check invlaid condition */
	if (!dev || !bitmap ||
	start_idx + num > (SWMBX_PROTECT_BITMAP) ||
	port > SWMBX_DEV_COUNT)
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;
	uint8_t i, j, value;
	uint32_t bitmap_val;

	for (i = start_idx; i < (start_idx + num); i++) {
		bitmap_val = bitmap[i];
		value = i << 0x5;

		for (j = 0; j < 0x20; j++) {
			if (bitmap_val & 0x1) {
				data->node[port][value + j].enable |= SWMBX_PROTECT;
			} else {
				data->node[port][value + j].enable &= ~SWMBX_PROTECT;
			}
			bitmap_val = bitmap_val >> 1;
		}
	}

	return 0;
}

/* update swmbx write protect with single address */
int swmbx_update_protect(const struct device *dev, uint8_t port,
uint8_t addr, uint8_t enable)
{
	/* check invlaid condition */
	if (!dev || port > SWMBX_DEV_COUNT)
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;

	if (enable)
		data->node[port][addr].enable |= SWMBX_PROTECT;
	else
		data->node[port][addr].enable &= ~SWMBX_PROTECT;

	return 0;
}

/* update swmbx notify with single address and index */
int swmbx_update_notify(const struct device *dev, uint8_t port,
struct k_sem *sem, uint8_t addr, uint8_t enable)
{
	if (!dev || port > SWMBX_DEV_COUNT)
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;

	/* check enable or not */
	if (enable) {
		if (!sem)
			return -EINVAL;

		data->node[port][addr].sem_notify = sem;
	} else {
		data->node[port][addr].sem_notify = NULL;
	}

	if (enable)
		data->node[port][addr].enable |= SWMBX_NOTIFY;
	else
		data->node[port][addr].enable &= ~SWMBX_NOTIFY;

	return 0;
}

int swmbx_flush_fifo(const struct device *dev, uint8_t index)
{
	if (!dev)
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;
	sys_snode_t *list_node = NULL;
	bool mbx_fifo_execute = false;
	uint8_t fifo_idx;

	mbx_fifo_execute = check_swmbx_fifo(data, index, &fifo_idx);
	if (mbx_fifo_execute) {
		if (data->fifo[fifo_idx].enable) {
			/* free link list */
			do {
				list_node = sys_slist_peek_head(&data->fifo[fifo_idx].list_head);
				if (list_node) {
					LOG_DBG("swmbx: slave drop fifo %x", (uint32_t)list_node);
					/* remove this item from list */
					sys_slist_find_and_remove(&data->fifo[fifo_idx].list_head,
					list_node);
				}
			} while (list_node);

			data->fifo[fifo_idx].msg_index = 0;
			data->fifo[fifo_idx].cur_msg_count = 0;
		} else {
			LOG_DBG("swmbx: fifo %d would not be enable", fifo_idx);
		}
	} else {
		LOG_DBG("swmbx_flush: could not find address %d fifo", index);
		return -EINVAL;
	}

	return 0;
}

int swmbx_update_fifo(const struct device *dev, struct k_sem *sem,
uint8_t idx, uint8_t addr, uint8_t depth, uint8_t notify, uint8_t enable)
{
	if (!dev || idx > (SWMBX_FIFO_COUNT - 1))
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;
	sys_snode_t *list_node = NULL;

	/* check enable or not */
	if (enable) {
		if (data->fifo[idx].enable || depth == 0 || !sem)
			return -EINVAL;

		/* malloc the message buffer */
		data->fifo[idx].buffer = k_malloc(sizeof(struct swmbx_fifo) * depth);
		if (!data->fifo[idx].buffer) {
			data->fifo[idx].enable = 0x0;
			LOG_ERR("fifo could not alloc enougth fifo buffer");
			return -EINVAL;
		}

		/* initial single list structure*/
		sys_slist_init(&data->fifo[idx].list_head);

		data->fifo[idx].sem_fifo = sem;
		data->fifo[idx].max_msg_count = depth;
		data->fifo[idx].fifo_offset = addr;
		data->fifo[idx].notify_flag = notify;
		data->fifo[idx].msg_index = 0;
		data->fifo[idx].cur_msg_count = 0;
	} else {
		if (data->fifo[idx].buffer) {
			/* free link list */
			do {
				list_node = sys_slist_peek_head(&data->fifo[idx].list_head);
				if (list_node) {
					LOG_DBG("swmbx: slave drop fifo %x", (uint32_t)list_node);
					/* remove this item from list */
					sys_slist_find_and_remove(&data->fifo[idx].list_head,
					list_node);
				}
			} while (list_node);

			/* free memory */
			k_free(data->fifo[idx].buffer);
			data->fifo[idx].buffer = NULL;
		}

		data->fifo[idx].sem_fifo = NULL;
	}

	data->fifo[idx].enable = enable;

	return 0;
}

/* end external API for swmbx */
static int swmbx_ctrl_init(const struct device *dev)
{
	struct swmbx_ctrl_data *data = DEV_DATA(dev);
	const struct swmbx_ctrl_config *cfg = DEV_CFG(dev);
	uint32_t i, j;

	data->buffer = (uint8_t *)(SWMBX_BUF_BASE);
	data->buffer_size = cfg->buffer_size;

	/* clear data structure */
	for (i = 0; i < SWMBX_DEV_COUNT; i++) {
		for (j = 0; j < SWMBX_NODE_COUNT; j++) {
			data->node[i][j].enable = false;
			data->node[i][j].sem_notify = NULL;
		}
	}

	for (i = 0; i < SWMBX_FIFO_COUNT; i++) {
		data->fifo[i].buffer = NULL;
		data->fifo[i].sem_fifo = NULL;
	}

	for (i = 0; i < SWMBX_DEV_COUNT; i++) {
		data->mbx_fifo_execute[i] = false;
		data->mbx_fifo_addr[i] = 0x0;
		data->mbx_fifo_idx[i] = 0x0;
	}

	/* keep the data info */
	*swmbx_info = (uint32_t)(data);

	return 0;
}

#define SWMBX_INIT(inst)						\
	static struct swmbx_ctrl_data				\
		swmbx_ctrl_##inst##_dev_data;			\
									\
	static const struct swmbx_ctrl_config			\
		swmbx_ctrl_##inst##_cfg = {			\
		.buffer_size = DT_INST_PROP(inst, size),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &swmbx_ctrl_init,			\
			    NULL,			\
			    &swmbx_ctrl_##inst##_dev_data,	\
			    &swmbx_ctrl_##inst##_cfg,		\
			    POST_KERNEL,				\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			    NULL);

DT_INST_FOREACH_STATUS_OKAY(SWMBX_INIT)
