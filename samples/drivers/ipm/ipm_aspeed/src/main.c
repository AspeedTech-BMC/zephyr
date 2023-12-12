#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/ipm.h>
#include <soc.h>

static const struct device *ipm_handle;
static uint8_t shm_rx[CONFIG_IPC_SHM_SIZE / 2] NON_CACHED_SHM_RX = {0};
static uint8_t shm_tx[CONFIG_IPC_SHM_SIZE / 2] NON_CACHED_SHM_TX = {0};

static void platform_ipm_callback(const struct device *dev, void *ctx,
				  uint32_t id, volatile void *data)
{
	int i;

	for (i = 0; i < CONFIG_IPC_SHM_SIZE / 2; i++) {
		if (sys_read32(shm_rx + (i << 2)) != 0) {
			if ((i & 0x3) == 0x0) {
				printf("[%08x] ", shm_rx + (i << 2));
			}
			printf("%08x ", sys_read32(shm_rx + (i << 2)));
			if ((i & 0x3) == 0x3)
				printf("\n");
		} else
			break;
	}

	ipm_send(dev, 0, 0, 0, 0);
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
