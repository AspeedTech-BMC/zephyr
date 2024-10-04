/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/ipm.h>

#include <soc.h>

#include <openamp/open_amp.h>
#include <metal/device.h>
#include <resource_table.h>

LOG_MODULE_REGISTER(openamp_ast26xx, LOG_LEVEL_INF);

#define RPMSG_CHAN_NAME	"rpmsg-client-sample"

#define SSP_MEM_RGN_SIZE	0x2000000
#define SSP_SHM_DEV_NAME	"vdev.shm"

#define APP_TASK_STACK_SIZE (512)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

static const struct device *ipm_handle;

struct vdev_shm {
	uint32_t va;
	uint32_t pa;
	size_t sz;
};

static struct vdev_shm rsc_tbl;
static struct vdev_shm vdev0vring0;
static struct vdev_shm vdev0vring1;
static struct vdev_shm vdev0buffer;

static struct metal_io_region *vdev0buffer_io;
static struct metal_io_region *rsc_io;
struct rpmsg_virtio_device rvdev;

static char rcv_msg[20];  /* should receive "Hello world!" */
static unsigned int rcv_len;
static struct rpmsg_endpoint rcv_ept;

static struct metal_device vdev_shm_dev = {
	.name = SSP_SHM_DEV_NAME,
	.num_regions = 2,
	.regions = {
		{ .virt = NULL }, /* resource IO */
		{ .virt = NULL }, /* vdev0buffer IO */
	},
	.node = { NULL },
	.irq_num = 0,
	.irq_info = NULL,
};

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_rx_sem, 0, 1);

static void platform_ipm_callback(const struct device *dev, void *ctx,
				  uint32_t id, volatile void *data)
{
	LOG_DBG("%d: mb received from %d", __LINE__, id);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data,
			       size_t len, uint32_t src, void *priv)
{
	LOG_DBG("%d: len=%d", __LINE__, len);
	memcpy(rcv_msg, data, len);
	rcv_len = len;
	k_sem_give(&data_rx_sem);

	return RPMSG_SUCCESS;
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	while (k_sem_take(&data_rx_sem, K_NO_WAIT) != 0) {
		int status = k_sem_take(&data_sem, K_FOREVER);

		if (status == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		}
	}
	*len = rcv_len;
	*msg = rcv_msg;
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	LOG_INF("%d: unexpected ns service receive for name %s", __LINE__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	LOG_INF("msg received from %d", id);
	ipm_send(ipm_handle, 0, id, NULL, 0);

	return 0;
}

static void platform_shm_init(void)
{
	uint32_t ssp_mem_base;
	size_t ssp_mem_size;

	/* FIXME: hardcoded buffer size, make it consistent with that of Linux */
	ssp_mem_base = PHY_SRAM_ADDR;
	ssp_mem_size = PHY_SRAM_DMEM_LIMIT - PHY_SRAM_ADDR;
	LOG_DBG("%d: ssp_mem: base=%x, sz=%x", __LINE__, ssp_mem_base, ssp_mem_size);

	rsc_tbl.pa = ssp_mem_base + ssp_mem_size - SSP_MEM_RGN_SIZE;
	rsc_tbl.va = TO_VIR_ADDR(rsc_tbl.pa);
	rsc_tbl.sz = 0x1000;
	LOG_DBG("%d: rsc_tbl: pa=%x, va=%x, sz=%x",
		__LINE__, rsc_tbl.pa, rsc_tbl.va, rsc_tbl.sz);

	vdev0vring0.pa = rsc_tbl.pa + rsc_tbl.sz;
	vdev0vring0.va = rsc_tbl.va + rsc_tbl.sz;
	vdev0vring0.sz = 0x1000;
	LOG_DBG("%d: vdev0vring0: pa=%x, va=%x, sz=%x",
		__LINE__, vdev0vring0.pa, vdev0vring0.va, vdev0vring0.sz);

	vdev0vring1.pa = vdev0vring0.pa + vdev0vring0.sz;
	vdev0vring1.va = vdev0vring0.va + vdev0vring0.sz;
	vdev0vring1.sz = 0x1000;
	LOG_DBG("%d: vdev0vring1: pa=%x, va=%x, sz=%x",
		__LINE__, vdev0vring1.pa, vdev0vring1.va, vdev0vring1.sz);

	vdev0buffer.pa = vdev0vring1.pa + vdev0vring1.sz;
	vdev0buffer.va = vdev0vring1.va + vdev0vring1.sz;
	vdev0buffer.sz = 0x1ffd000;
	LOG_DBG("%d: vdev0buffer: pa=%x, va=%x, sz=%x",
		__LINE__, vdev0buffer.pa, vdev0buffer.va, vdev0buffer.sz);
}

int platform_init(void)
{
	int rc;
	struct metal_device *dev;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;

	platform_shm_init();

	rc = metal_init(&metal_params);
	if (rc) {
		LOG_ERR("cannot initialize metal");
		return rc;
	}

	rc = metal_register_generic_device(&vdev_shm_dev);
	if (rc) {
		LOG_ERR("cannot register shared memory");
		return rc;
	}

	rc = metal_device_open("generic", SSP_SHM_DEV_NAME, &dev);
	if (rc) {
		LOG_ERR("cannot open shared memory");
		return rc;
	};

	/* declare resource table region */
	metal_io_init(&dev->regions[0],	(void *)rsc_tbl.va,
		      (metal_phys_addr_t *)&rsc_tbl.pa, rsc_tbl.sz, -1, 0, NULL);

	rsc_io = metal_device_io_region(dev, 0);
	if (!rsc_io) {
		LOG_ERR("cannot get rsc_io region");
		return -1;
	}

	/* declare vdev0buffer region */
	metal_io_init(&dev->regions[1], (void *)vdev0buffer.va,
		      (metal_phys_addr_t *)&vdev0buffer.pa, vdev0buffer.sz, -1, 0, NULL);

	vdev0buffer_io = metal_device_io_region(dev, 1);
	if (!vdev0buffer_io) {
		LOG_ERR("cannot get vdev0buffer_io region");
		return -1;
	}

	/* setup IPM */
	ipm_handle = device_get_binding(CONFIG_OPENAMP_IPC_DEV_NAME);
	if (!ipm_handle) {
		LOG_ERR("cannot find ipm device");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	rc = ipm_set_enabled(ipm_handle, 1);
	if (rc) {
		LOG_ERR("cannot ipm_set_enabled");
		return -1;
	}

	return 0;
}

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct rpmsg_device *platform_create_rpmsg_vdev(unsigned int vdev_index,
						unsigned int role,
						void (*rst_cb)(struct virtio_device *vdev),
						rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID,
					rsc_table_to_vdev((void *)rsc_tbl.va),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		LOG_ERR("failed to create vdev");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0((void *)rsc_tbl.va);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vdev0vring0.va, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 0");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1((void *)rsc_tbl.va);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vdev0vring1.va, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 1");
		goto failed;
	}

	ret = rpmsg_init_vdev(&rvdev, vdev, ns_cb, vdev0buffer_io, NULL);
	if (ret) {
		LOG_ERR("failed rpmsg_init_vdev");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int rc;
	struct rpmsg_device *rpdev;
	unsigned char *msg;
	unsigned int len, msg_cnt = 0;

	LOG_INF("OpenAMP AST26xx remote demo started");

	rc = platform_init();
	if (rc) {
		LOG_ERR("cannot initialize platform");
		rc = -1;
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_DRIVER, NULL, new_service_cb);
	if (!rpdev) {
		LOG_ERR("cannot create rpmsg virtio device");
		rc = -1;
		goto task_end;
	}

	rc = rpmsg_create_ept(&rcv_ept, rpdev, RPMSG_CHAN_NAME,
			      RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			      rpmsg_recv_callback, NULL);
	if (rc != 0)
		LOG_ERR("error while creating endpoint(%d)", rc);

	while (msg_cnt < 100) {
		receive_message(&msg, &len);
		msg_cnt++;
		rpmsg_send(&rcv_ept, msg, len);
	}
	rpmsg_destroy_ept(&rcv_ept);

task_end:
	cleanup_system();

	LOG_INF("OpenAMP demo ended");
}

int main(void)
{
	LOG_INF("Starting application thread!");
	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	return 0;
}
