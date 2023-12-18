#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/ipm.h>

static const struct device *ipm_handle;
extern uint8_t *shm_rx, *shm_tx;

static void platform_ipm_callback(const struct device *dev, void *ctx,
				  uint32_t id, volatile void *data)
{
	int i;
	uint32_t shared_memory_rx = (uint32_t)&shm_rx;
	uint32_t shared_memory_tx = (uint32_t)&shm_tx;

	printk("[CM3] rx addr:%08x, tx addr:%08x\n", shared_memory_rx, shared_memory_tx);
	/* Check the CM3 RX data from CA7 RX data. */
	printk("CM3 RX data from CA7 TX data:\n");
	for (i = 0; i < CONFIG_IPC_SHM_SIZE / 2; i += 4) {
		if (sys_read32(shared_memory_rx + i) != 0) {
			if (((i >> 2) & 0x3) == 0x0)
				printk("[%08x] ", shared_memory_rx + i);
			printk("%08x ", sys_read32(shared_memory_rx + i));
			if (((i >> 2) & 0x3) == 0x3)
				printk("\n");
		} else
			break;
	}

	/* Write back to CM3 TX from CM3 RX data. */
	ipm_send(dev, 1, 0, (void *)shared_memory_rx, CONFIG_IPC_SHM_SIZE / 2);
}

void main(void)
{
	int rc;

	/* setup IPM */
	ipm_handle = device_get_binding(CONFIG_IPC_DEV_NAME);
	if (!ipm_handle) {
		printk("device_get_binding failed to find device\n");
		return;
	}

	/* setup callback function with received data processing. */
	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	/* enable ipm. */
	rc = ipm_set_enabled(ipm_handle, 1);
	if (rc) {
		printk("cannot ipm_set_enabled\n");
		return;
	}

	printk("IPM initialized\n");
}
