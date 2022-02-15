/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_swmbx

#include <sys/util.h>
#include <sys/slist.h>
#include <kernel.h>
#include <errno.h>
#include <drivers/i2c.h>
#include <string.h>
#include <drivers/i2c/slave/swmbx.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_slave_swmbx);

struct i2c_swmbx_fifo {
	sys_snode_t list;
	uint8_t value;
};

struct i2c_swmbx_notify {
	struct k_sem *sem_notify;
	uint8_t address;
	uint8_t enable;
};

struct i2c_swmbx_fifo_data {
	struct k_sem *sem_fifo;
	struct i2c_swmbx_fifo *buffer;
	struct i2c_swmbx_fifo *current;
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

struct i2c_swmbx_slave_data {
	const struct device *i2c_controller;
	struct i2c_slave_config config;
	uint32_t bus_base;
	uint8_t bus_count;
	uint32_t *mbx_info;
	uint32_t buffer_size;
	uint8_t *buffer;
	uint32_t buffer_idx;
	bool first_write;
	bool mbx_write_data;
	bool mbx_fifo_execute;
	uint8_t mbx_en;
	uint8_t mbx_notify_idx;
	uint8_t mbx_fifo_idx;
	uint32_t mbx_protect[SWMBX_PROTECT_COUNT];
	struct i2c_swmbx_notify notify[SWMBX_NOTIFY_COUNT];
	struct i2c_swmbx_fifo_data fifo[SWMBX_FIFO_COUNT];
};

struct i2c_swmbx_slave_config {
	char *controller_dev_name;
	uint32_t bus_base;
	uint8_t address;
	uint32_t buffer_size;
	uintptr_t buffer;
};

/* convenience defines */
#define DEV_CFG(dev)		\
	((const struct i2c_swmbx_slave_config *const)	\
	(dev)->config)
#define DEV_DATA(dev)	\
	((struct i2c_swmbx_slave_data *const)(dev)->data)

/* internal api for pfr sw mbx control */
int check_swmbx_fifo(struct i2c_swmbx_slave_data *data, uint8_t addr, uint8_t *index)
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

int check_swmbx_notify(struct i2c_swmbx_slave_data *data, uint8_t addr)
{
	for (uint8_t i = 0; i < SWMBX_NOTIFY_COUNT; i++) {
		if ((data->notify[i].address == addr) && (data->notify[i].enable)
		&& (data->notify[i].sem_notify)) {
			data->mbx_notify_idx = i;
			return 1;
		}
	}

	return 0;
}

int check_swmbx_protect(struct i2c_swmbx_slave_data *data, uint8_t addr)
{
	uint8_t index, bit;
	uint32_t value;

	/* calculte bitmap position */
	index = addr / 0x20;
	bit = addr % 0x20;

	value = data->mbx_protect[index];

	if (value & (0x1 << bit))
		return 1;

	return 0;
}

int append_fifo_write(struct i2c_swmbx_slave_data *data, uint8_t fifo_idx, uint8_t val)
{
	struct i2c_swmbx_fifo_data *fifo = NULL;

	/* find out the fifo item */
	fifo = &data->fifo[fifo_idx];

	/* check the max msg length */
	if (fifo->cur_msg_count < fifo->max_msg_count) {
		LOG_DBG("fifo: cur_msg_index %x", (uint32_t)(fifo->msg_index));

		fifo->current = &(fifo->buffer[fifo->cur_msg_count]);
		fifo->current->value = val;
		sys_slist_append(&(fifo->list_head), &(fifo->current->list));
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

int peak_fifo_read(struct i2c_swmbx_slave_data *data, uint8_t fifo_idx, uint8_t *val)
{
	struct i2c_swmbx_fifo_data *fifo = NULL;
	sys_snode_t *list_node = NULL;

	/* find out the fifo item */
	fifo = &data->fifo[fifo_idx];
	list_node = sys_slist_peek_head(&(fifo->list_head));
	LOG_DBG("fifo: slave read %x", (uint32_t)list_node);

	if (list_node != NULL) {
		fifo->current = (struct i2c_swmbx_fifo *)(list_node);
		*val = fifo->current->value;
		LOG_DBG("fifo: slave read %x ", *val);

		/* remove this item from list */
		sys_slist_find_and_remove(&(fifo->list_head), list_node);

		fifo->cur_msg_count--;
		LOG_DBG("fifo: fifo msg %x", fifo->cur_msg_count);
	} else {
		*val = 0;
		return 1;
	}

	return 0;
}
/* internal api define end */

/* external API for swmbx access */
int swmbx_write(const struct device *dev, uint8_t fifo, uint8_t index, uint8_t *val)
{
	if (dev == NULL)
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;
	unsigned int key = 0;
	bool mbx_fifo_execute = false;
	uint8_t fifo_idx = 0;

	/* enter critical section sw mbx access */
	if (!k_is_in_isr())
		key = irq_lock();

	if (fifo) {
		/*check fifo enable and find out fifo index*/
		mbx_fifo_execute = check_swmbx_fifo(data, index, &fifo_idx);
		if (mbx_fifo_execute) {
			/* append value into fifo */
			if (append_fifo_write(data, fifo_idx, *val)) {
				LOG_DBG("swmbx_write: fifo full at %d group", fifo_idx);
				return 1;
			}
		} else {
			LOG_DBG("swmbx_write: could not find address %d fifo", index);
			return 1;
		}
	} else {
		data->buffer[index] = *val;
	}

	/* exit critical section */
	if (!k_is_in_isr())
		irq_unlock(key);

	return 0;
}

int swmbx_read(const struct device *dev, uint8_t fifo, uint8_t index, uint8_t *val)
{
	if (dev == NULL)
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;
	unsigned int key = 0;
	bool mbx_fifo_execute = false;
	uint8_t fifo_idx = 0;

	/* enter critical section sw mbx access */
	if (!k_is_in_isr())
		key = irq_lock();

	if (fifo) {
		/*check fifo enable and find out fifo index*/
		mbx_fifo_execute = check_swmbx_fifo(data, index, &fifo_idx);
		if (mbx_fifo_execute) {
			/* peak value from fifo */
			if (peak_fifo_read(data, fifo_idx, val)) {
				LOG_DBG("swmbx_read: fifo empty at %d group", fifo_idx);
				return 1;
			}
		} else {
			LOG_DBG("swmbx_read: could not find fifo %d group", index);
			return 1;
		}
	} else {
		*val = data->buffer[index];
	}

	/* exit critical section */
	if (!k_is_in_isr())
		irq_unlock(key);

	return 0;
}

/* general control */
int swmbx_enable_behavior(const struct device *dev, uint32_t item_flag, uint8_t enable)
{
	if ((dev == NULL) || ((item_flag & FLAG_MASK) == 0))
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;

	data->mbx_en = item_flag;

	return 0;
}

/* apply swmbx write protect with bitmap*/
int swmbx_apply_protect(const struct device *dev, uint32_t *bitmap, uint8_t start_idx, uint8_t num)
{
	if ((dev == NULL) || (bitmap == NULL) ||
	(start_idx + num) > (SWMBX_PROTECT_COUNT))
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;

	for (uint8_t i = start_idx; i < (start_idx + num); i++)
		data->mbx_protect[i] = bitmap[i];

	return 0;
}

/* update swmbx write protect with single address */
int swmbx_update_protect(const struct device *dev, uint8_t addr, uint8_t enable)
{
	if (dev == NULL)
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;
	uint8_t index, bit;
	uint32_t value;

	/* calculte bitmap position */
	index = addr / 0x20;
	bit = addr % 0x20;

	value = data->mbx_protect[index];

	if (enable)
		value |= (0x1 << bit);
	else
		value = value & ~(0x1 << bit);

	data->mbx_protect[index] = value;

	return 0;
}

/* update swmbx notify with single address and index */
int swmbx_update_notify(const struct device *dev, struct k_sem *sem,
uint8_t idx, uint8_t addr, uint8_t enable)
{
	if ((dev == NULL) || idx > (SWMBX_NOTIFY_COUNT-1))
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;

	/* check enable or not */
	if (enable) {
		if ((data->notify[idx].enable) || sem == NULL)
			return -EINVAL;

		data->notify[idx].sem_notify = sem;
		data->notify[idx].address = addr;
	} else {
		data->notify[idx].sem_notify = NULL;
	}

	data->notify[idx].enable = enable;

	return 0;
}

int swmbx_flush_fifo(const struct device *dev, uint8_t index)
{
	if (dev == NULL)
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;
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

	struct i2c_swmbx_slave_data *data = dev->data;
	sys_snode_t *list_node = NULL;

	/* check enable or not */
	if (enable) {
		if ((data->fifo[idx].enable) || depth == 0 || sem == NULL)
			return -EINVAL;

		/* malloc the message buffer */
		data->fifo[idx].buffer = k_malloc(sizeof(struct i2c_swmbx_fifo) * depth);
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

/* turn on / off other swmbx support */
void turn_swmbx_slave(struct i2c_swmbx_slave_data *data, uint8_t on)
{
	uint8_t i = 0, address = 0;
	uint32_t i2c_base = SWMBX_INFO_BASE & 0xFFFF0000;
	uint32_t *swmbx_g_info = (uint32_t *)(SWMBX_INFO_BASE);

	for (i = 0; i < SWMBX_DEVICE_COUNT; i++) {
		/* affect the other device */
		if (*(swmbx_g_info + i) & SWMBX_REGISTER) {
			i2c_base += (i + 1) * 0x80;

			if (on)
				address = *(swmbx_g_info + i) & 0xFF;
			else
				address = 0;

			if (i2c_base != data->bus_base) {
				/*Set slave addr.*/
				sys_write32(address |
				(sys_read32(i2c_base + AST_I2CS_ADDR_CTRL)
				& ~AST_I2CS_ADDR1_MASK), i2c_base + AST_I2CS_ADDR_CTRL);
			}
		}
	}
}
/* end external API for swmbx */

/* i2c virtual slave functions */
static int swmbx_slave_write_requested(struct i2c_slave_config *config)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	LOG_DBG("swmbx: write req");

	/* turn off other swmbx */
	turn_swmbx_slave(data, 0x0);

	data->first_write = true;

	return 0;
}

static int swmbx_slave_write_received(struct i2c_slave_config *config,
				       uint8_t val)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);
	uint8_t index;

	LOG_DBG("swmbx: write received, val=0x%x", val);

	if (data->first_write) {
		/* obtain fifo index */
		data->mbx_fifo_execute = check_swmbx_fifo(data, val, &index);
		if (data->mbx_fifo_execute)
			data->mbx_fifo_idx = index;

		data->buffer_idx = val;
		data->first_write = false;
	} else {
		/* check the FIFO is executed or not */
		if ((data->mbx_fifo_execute) && (data->mbx_en & SWMBX_FIFO)) {
			/* append value into fifo */
			if (append_fifo_write(data, data->mbx_fifo_idx, val)) {
				/* issue semaphore when fifo full */
				k_sem_give(data->fifo[data->mbx_fifo_idx].sem_fifo);
				LOG_DBG("fifo: fifo full at %d group", data->mbx_fifo_idx);
				return 1;
			}

			/* check fifo notify start*/
			if ((data->mbx_en & SWMBX_NOTIFY) &&
			(data->fifo[data->mbx_fifo_idx].notify_flag & SWMBX_FIFO_NOTIFY_START) &&
			(!data->fifo[data->mbx_fifo_idx].notify_start)) {
				if (check_swmbx_notify(data, data->buffer_idx)) {
					k_sem_give(data->notify[data->mbx_notify_idx].sem_notify);
					data->fifo[data->mbx_fifo_idx].notify_start = true;
				}
			}

			if (!data->fifo[data->mbx_fifo_idx].fifo_write)
				data->fifo[data->mbx_fifo_idx].fifo_write = true;
		} else {
			/* check write protect behavior */
			if (!check_swmbx_protect(data, data->buffer_idx) ||
			!(data->mbx_en & SWMBX_PROTECT)) {
				data->mbx_write_data = true;
			} else {
				data->mbx_write_data = false;
			}

			/* check notify behavior */
			if (data->mbx_en & SWMBX_NOTIFY) {
				if (check_swmbx_notify(data, data->buffer_idx)) {
					k_sem_give(data->notify[data->mbx_notify_idx].sem_notify);
				}
			}

			if (data->mbx_write_data)
				data->buffer[data->buffer_idx] = val;

			data->buffer_idx++;
		}
	}

	data->buffer_idx = data->buffer_idx % data->buffer_size;

	return 0;
}

static int swmbx_slave_read_processed(struct i2c_slave_config *config,
				       uint8_t *val)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	/* Increment here */
	data->buffer_idx = (data->buffer_idx + 1) % data->buffer_size;

	/* check the FIFO is executed or not */
	if ((data->mbx_fifo_execute) && (data->mbx_en & SWMBX_FIFO)) {
		/* peak value from fifo */
		if (peak_fifo_read(data, data->mbx_fifo_idx, val)) {
			/* issue semaphore when fifo empty */
			k_sem_give(data->fifo[data->mbx_fifo_idx].sem_fifo);
			LOG_DBG("fifo: fifo empty at %d group", data->mbx_fifo_idx);
			return 1;
		}
	} else {
		*val = data->buffer[data->buffer_idx];
	}

	LOG_DBG("swmbx: read proc, val=0x%x", *val);

	/* Increment will be done in the next read_processed callback
	 * In case of STOP, the byte won't be taken in account
	 */

	return 0;
}

static int swmbx_slave_read_requested(struct i2c_slave_config *config,
				       uint8_t *val)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	/* turn off other swmbx */
	turn_swmbx_slave(data, 0x0);

	/* check the FIFO is executed or not */
	if ((data->mbx_fifo_execute) && (data->mbx_en & SWMBX_FIFO)) {
		/* read fifo will be handled in the read process */
		swmbx_slave_read_processed(config, val);
	} else {
		*val = data->buffer[data->buffer_idx];
	}

	LOG_DBG("swmbx: read req, val=0x%x", *val);

	/* Increment will be done in the read_processed callback */

	return 0;
}

static int swmbx_slave_stop(struct i2c_slave_config *config)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	LOG_DBG("swmbx: stop");

	/* check fifo notify end*/
	if (data->mbx_fifo_execute) {
		if ((data->mbx_en & SWMBX_NOTIFY) &&
		(data->fifo[data->mbx_fifo_idx].notify_flag & SWMBX_FIFO_NOTIFY_STOP) &&
		(data->fifo[data->mbx_fifo_idx].fifo_write)) {
			if (check_swmbx_notify(data, data->buffer_idx)) {
				k_sem_give(data->notify[data->mbx_notify_idx].sem_notify);
			}
		}
		data->fifo[data->mbx_fifo_idx].notify_start = false;
		data->fifo[data->mbx_fifo_idx].fifo_write = false;
		data->mbx_fifo_execute = false;
	}

	/* turn on other swmbx */
	turn_swmbx_slave(data, 0x1);

	data->first_write = true;

	return 0;
}

/* i2c virtual slave register functions */
static int swmbx_slave_register(const struct device *dev)
{
	struct i2c_swmbx_slave_data *data = dev->data;
	uint8_t i;

	*(data->mbx_info) |= SWMBX_REGISTER;

	/* initial data structure value */
	for (i = 0; i < SWMBX_PROTECT_COUNT; i++) {
		data->mbx_protect[i] = 0;
		data->notify[i].enable = 0;
		data->notify[i].sem_notify = NULL;
		data->fifo[i].enable = 0;
		data->fifo[i].buffer = NULL;
		data->fifo[i].sem_fifo = NULL;
	}

	return i2c_slave_register(data->i2c_controller, &data->config);
}

static int swmbx_slave_unregister(const struct device *dev)
{
	struct i2c_swmbx_slave_data *data = dev->data;
	uint8_t i;

	*(data->mbx_info) &= ~(SWMBX_REGISTER);

	/* release fifo memory */
	for (i = 0; i < SWMBX_FIFO_COUNT; i++) {
		swmbx_update_fifo(dev, NULL, i, 0x0, 0x0, 0x0, 0x0);
	}

	return i2c_slave_unregister(data->i2c_controller, &data->config);
}

static const struct i2c_slave_driver_api swmbx_api_funcs = {
	.driver_register = swmbx_slave_register,
	.driver_unregister = swmbx_slave_unregister,
};

static const struct i2c_slave_callbacks swmbx_callbacks = {
	.write_requested = swmbx_slave_write_requested,
	.read_requested = swmbx_slave_read_requested,
	.write_received = swmbx_slave_write_received,
	.read_processed = swmbx_slave_read_processed,
	.stop = swmbx_slave_stop,
};

static int i2c_swmbx_slave_init(const struct device *dev)
{
	struct i2c_swmbx_slave_data *data = DEV_DATA(dev);
	const struct i2c_swmbx_slave_config *cfg = DEV_CFG(dev);
	uint32_t *swmbx_g_info = (uint32_t *)(SWMBX_INFO_BASE);

	data->i2c_controller =
		device_get_binding(cfg->controller_dev_name);
	if (!data->i2c_controller) {
		LOG_ERR("i2c controller not found: %s",
			    cfg->controller_dev_name);
		return -EINVAL;
	}

	data->buffer_size = cfg->buffer_size;
	data->buffer = (uint8_t *)(cfg->buffer);
	data->bus_base = cfg->bus_base;
	data->config.address = cfg->address;
	data->config.callbacks = &swmbx_callbacks;

	data->bus_count = ((cfg->bus_base & 0xFFFF) / 0x80) - 1;
	data->mbx_info = swmbx_g_info + data->bus_count;
	*(data->mbx_info) = cfg->address;

	return 0;
}

#define I2C_SWMBX_INIT(inst)						\
	static struct i2c_swmbx_slave_data				\
		i2c_swmbx_slave_##inst##_dev_data;			\
									\
	static const struct i2c_swmbx_slave_config			\
		i2c_swmbx_slave_##inst##_cfg = {			\
		.controller_dev_name = DT_INST_BUS_LABEL(inst),		\
		.bus_base = DT_REG_ADDR(DT_INST_BUS(inst)),	\
		.address = DT_INST_REG_ADDR(inst),			\
		.buffer_size = DT_INST_PROP(inst, size),		\
		.buffer = DT_INST_PROP(inst, base),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &i2c_swmbx_slave_init,			\
			    NULL,			\
			    &i2c_swmbx_slave_##inst##_dev_data,	\
			    &i2c_swmbx_slave_##inst##_cfg,		\
			    POST_KERNEL,				\
			    CONFIG_I2C_SLAVE_INIT_PRIORITY,		\
			    &swmbx_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(I2C_SWMBX_INIT)

