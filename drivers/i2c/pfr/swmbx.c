/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_swmbx-ctrl

#include <sys/util.h>
#include <sys/slist.h>
#include <kernel.h>
#include <errno.h>
#include <device.h>
#include <string.h>
#include <drivers/i2c/slave/swmbx.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(swmbx_ctrl);

struct swmbx_fifo {
	sys_snode_t list;
	uint8_t value;
};

struct swmbx_notify {
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
	const struct device *swmbx_controller;
	uint32_t buffer_size;
	uint8_t *buffer;
	uint8_t mbx_en;
	uint8_t mbx_protect[SWMBX_DEV_COUNT][SWMBX_PROTECT_COUNT];
	struct swmbx_notify notify[SWMBX_DEV_COUNT][SWMBX_NOTIFY_COUNT];
	struct swmbx_fifo_data fifo[SWMBX_FIFO_COUNT];
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

/* internal api for pfr swmbx control */
int check_swmbx_fifo(struct swmbx_ctrl_data *data, uint8_t addr, uint8_t *index)
{
	for (uint8_t i = 0; i < SWMBX_FIFO_COUNT; i++) {
		if ((data->fifo[i].fifo_offset == addr) && (data->fifo[i].enable)
		&& (data->fifo[i].sem_fifo)) {
			*index = i;
			LOG_DBG("fifo: fifo inx %x and address %x", i, addr);
			return 1;
		}
	}

	return 0;
}
/* internal api define end */

/* external API for swmbx access */
/* general control */
int swmbx_enable_behavior(const struct device *dev, uint32_t item_flag, uint8_t enable)
{
	if ((dev == NULL) || ((item_flag & FLAG_MASK) == 0))
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
	if ((dev == NULL) || (bitmap == NULL) ||
	(start_idx + num) > (SWMBX_PROTECT_COUNT) ||
	(port > SWMBX_DEV_COUNT))
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;

	for (uint8_t i = start_idx; i < (start_idx + num); i++)
		data->mbx_protect[port][i] = bitmap[i];

	return 0;
}

/* update swmbx write protect with single address */
int swmbx_update_protect(const struct device *dev, uint8_t port,
uint8_t addr, uint8_t enable)
{
	/* check invlaid condition */
	if ((dev == NULL) || (port > SWMBX_DEV_COUNT))
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;
	uint8_t index, bit;
	uint32_t value;

	/* calculte bitmap position */
	index = addr / 0x20;
	bit = addr % 0x20;

	value = data->mbx_protect[port][index];

	if (enable)
		value |= (0x1 << bit);
	else
		value = value & ~(0x1 << bit);

	data->mbx_protect[port][index] = value;

	return 0;
}

/* update swmbx notify with single address and index */
int swmbx_update_notify(const struct device *dev, uint8_t port,
struct k_sem *sem, uint8_t addr, uint8_t enable)
{
	if ((dev == NULL) || (port > SWMBX_DEV_COUNT))
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;

	/* check enable or not */
	if (enable) {
		if (sem == NULL)
			return -EINVAL;

		data->notify[port][addr].sem_notify = sem;
	} else {
		data->notify[port][addr].sem_notify = NULL;
	}

	data->notify[port][addr].enable = enable;

	return 0;
}

int swmbx_flush_fifo(const struct device *dev, uint8_t index)
{
	if (dev == NULL)
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
				list_node = sys_slist_peek_head(&(data->fifo[fifo_idx].list_head));
				if (list_node != NULL) {
					LOG_DBG("swmbx: slave drop fifo %x", (uint32_t)list_node);
					/* remove this item from list */
					sys_slist_find_and_remove(&(data->fifo[fifo_idx].list_head), list_node);
				}
			} while (list_node != NULL);

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
	if ((dev == NULL) || idx > (SWMBX_FIFO_COUNT-1))
		return -EINVAL;

	struct swmbx_ctrl_data *data = dev->data;
	sys_snode_t *list_node = NULL;

	/* check enable or not */
	if (enable) {
		if ((data->fifo[idx].enable) || depth == 0 || sem == NULL)
			return -EINVAL;

		/* malloc the message buffer */
		data->fifo[idx].buffer = k_malloc(sizeof(struct swmbx_fifo) * depth);
		if (!data->fifo[idx].buffer) {
			data->fifo[idx].enable = 0x0;
			LOG_ERR("fifo could not alloc enougth fifo buffer");
			return -EINVAL;
		}

		/* initial single list structure*/
		sys_slist_init(&(data->fifo[idx].list_head));

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
				list_node = sys_slist_peek_head(&(data->fifo[idx].list_head));
				if (list_node != NULL) {
					LOG_DBG("swmbx: slave drop fifo %x", (uint32_t)list_node);
					/* remove this item from list */
					sys_slist_find_and_remove(&(data->fifo[idx].list_head), list_node);
				}
			} while (list_node != NULL);

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

	data->swmbx_controller =
		device_get_binding(cfg->controller_dev_name);
	if (!data->swmbx_controller) {
		LOG_ERR("swmbx controller not found: %s",
			    cfg->controller_dev_name);
		return -EINVAL;
	}

	data->buffer = (uint8_t *)(SWMBX_BUF_BASE);
	data->buffer_size = cfg->buffer_size;

	/* clear data structure */
	for (j = 0; j < SWMBX_PROTECT_COUNT; j++) {
		for (i = 0; i < SWMBX_DEV_COUNT; i++) {
			data->mbx_protect[i][j] = 0;
			data->notify[i][j].enable = 0;
			data->notify[i][j].sem_notify = NULL;
		}
	}

	for (i = 0; i < SWMBX_FIFO_COUNT; i++) {
		data->fifo[i].enable = 0;
		data->fifo[i].buffer = NULL;
		data->fifo[i].sem_fifo = NULL;
	}

	return 0;
}

#define SWMBX_INIT(inst)						\
	static struct swmbx_ctrl_data				\
		swmbx_ctrl_##inst##_dev_data;			\
									\
	static const struct swmbx_ctrl_config			\
		swmbx_ctrl_##inst##_cfg = {			\
		.controller_dev_name = DT_INST_BUS_LABEL(inst),		\
		.buffer_size = DT_INST_PROP(inst, size),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &swmbx_ctrl_init,			\
			    NULL,			\
			    &i2c_swmbx_slave_##inst##_dev_data,	\
			    &i2c_swmbx_slave_##inst##_cfg,		\
			    POST_KERNEL,				\
			    CONFIG_I2C_SLAVE_INIT_PRIORITY,		\
			    NULL);

DT_INST_FOREACH_STATUS_OKAY(SWMBX_INIT)
