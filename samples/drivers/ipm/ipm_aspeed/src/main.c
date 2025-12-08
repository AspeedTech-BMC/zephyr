#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/ipm.h>
#include <string.h>

#if defined(CONFIG_IPM_AST2600)
extern uint8_t *shm_rx, *shm_tx;
#elif defined(CONFIG_IPM_AST2700)
#define DEFAULT_LINE_LENGTH_BYTES (16)
#endif
static void test_ipm_cb(const struct device *ipmdev, void *user_data,
			       uint32_t id, volatile void *data)
{
#if defined(CONFIG_IPM_AST2600)
	int i;
	uint32_t shared_memory_rx = (uint32_t)&shm_rx;
	uint32_t shared_memory_tx = (uint32_t)&shm_tx;

	printk("[SSP] rx addr:%08x, tx addr:%08x\n", shared_memory_rx, shared_memory_tx);
	/* Check the SSP RX data from PSP RX data. */
	printk("PSP RX data from PSP TX data:\n");
	for (i = 0; i < CONFIG_IPC_SHM_RX_SIZE; i += 4) {
		if (sys_read32(shared_memory_rx + i) != 0) {
			if (((i >> 2) & 0x3) == 0x0)
				printk("[%08x] ", shared_memory_rx + i);
			printk("%08x ", sys_read32(shared_memory_rx + i));
			if (((i >> 2) & 0x3) == 0x3)
				printk("\n");
		} else
			break;
	}

	/* Write back to SSP TX from SSP RX data. */
	ipm_send(ipmdev, 1, 0, (void *)shared_memory_rx, CONFIG_IPC_SHM_TX_SIZE);
#elif defined(CONFIG_IPM_AST2700)
	int i;
	int check = 0;
	int width = 4;
	int linelen = DEFAULT_LINE_LENGTH_BYTES/width;
	int max_data_size = ipm_max_data_size_get(ipmdev);
	uint32_t *buf = (uint32_t *)data;

	printk("%s:msg id %x, msg data at %p\n", __func__, id, data);

	for (i = 0; i < max_data_size/sizeof(buf) ; i++) {
		/* FAIL when buf is not golden value */
		if (buf[i] != 0x1688a8a8)
			check = 1;
	}

	/* Check golden fail to print the msg data */
	if (check)
		while (max_data_size) {
			printk("%p:", buf);

			for (i = 0; i < linelen; i++)
				printk(" %08x ", buf[i]);
			printk("\n");
			buf += linelen;
			max_data_size -= linelen * width;
		}
	else
		printk("Check msg data: pass.\n");
#endif
}

int main(void)
{
	int rc = 0;
	const struct device *ipmdev;

#if defined(CONFIG_IPM_AST2600)
	/* setup IPM */
	ipmdev = device_get_binding(CONFIG_OPENAMP_IPC_DEV_NAME);
	if (!ipmdev) {
		printk("device_get_binding failed to find device\n");
		rc = 1;
		goto fail;
	}

	/* setup callback function with received data processing. */
	ipm_register_callback(ipmdev, test_ipm_cb, NULL);

	/* enable ipm. */
	rc = ipm_set_enabled(ipmdev, 1);
	if (rc) {
		printk("cannot ipm_set_enabled\n");
		goto fail;
	}
#elif defined(CONFIG_IPM_AST2700)
	char ipc_name[16];
	int device_id, enable;
#if defined(CONFIG_BOARD_AST2700_EVB_AST2700_SSP) ||                                               \
	defined(CONFIG_BOARD_AST2700_EVB_AST2700_A1_SSP)
	printk("SSP alive.\n");
	strcpy(ipc_name, "ipc0@0");
	ipmdev = device_get_binding(&ipc_name[0]);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = 1;
		goto fail;
	}

	device_id = 0;
	enable = 1;
	ipm_register_id_callback(ipmdev, device_id, test_ipm_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, enable);
	if (rc) {
		printk("%s: cannot ipm_set_enabled\n", ipc_name);
		goto fail;
	}

	strcpy(ipc_name, "ipc0@200");
	ipmdev = device_get_binding(&ipc_name[0]);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = 1;
		goto fail;
	}

	device_id = 0;
	enable = 1;
	ipm_register_id_callback(ipmdev, device_id, test_ipm_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, enable);
	if (rc) {
		printk("%s: cannot ipm_set_enabled\n", ipc_name);
		goto fail;
	}
#endif /* defined(CONFIG_BOARD_AST2700_EVB_AST2700_SSP) */
#if defined(CONFIG_BOARD_AST2700_EVB_AST2700_TSP) ||                                               \
	defined(CONFIG_BOARD_AST2700_EVB_AST2700_A1_TSP)
	printk("TSP alive.\n");
	strcpy(ipc_name, "ipc0@400");
	ipmdev = device_get_binding(&ipc_name[0]);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = 1;
		goto fail;
	}

	device_id = 0;
	enable = 1;
	ipm_register_id_callback(ipmdev, device_id, test_ipm_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, enable);
	if (rc) {
		printk("%s: cannot ipm_set_enabled\n", ipc_name);
		goto fail;
	}

	strcpy(ipc_name, "ipc0@600");
	ipmdev = device_get_binding(&ipc_name[0]);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = 1;
		goto fail;
	}

	device_id = 0;
	enable = 1;
	ipm_register_id_callback(ipmdev, device_id, test_ipm_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, enable);
	if (rc) {
		printk("%s: cannot ipm_set_enabled\n", ipc_name);
		goto fail;
	}
#endif /* defined(CONFIG_BOARD_AST2700_EVB_AST2700_TSP) */

	strcpy(ipc_name, "ipc0@800");
	ipmdev = device_get_binding(&ipc_name[0]);
	if (!ipmdev) {
		printk("%s: device_get_binding failed to find device\n", ipc_name);
		rc = 1;
		goto fail;
	}

	device_id = 0;
	enable = 1;
	ipm_register_id_callback(ipmdev, device_id, test_ipm_cb, NULL);
	rc = ipm_set_id_enabled(ipmdev, device_id, enable);
	if (rc) {
		printk("%s: cannot ipm_set_enabled\n", ipc_name);
		goto fail;
	}
#endif /* defined(CONFIG_IPM_AST2700) */
	printk("All IPMs initialized\n");

	while (1) {
		k_msleep(1000);
	}

fail:
	return rc;
}
