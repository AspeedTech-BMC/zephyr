#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/ipm.h>

#if defined(CONFIG_SOC_AST2600)
extern uint8_t *shm_rx, *shm_tx;
#else
#define DEFAULT_LINE_LENGTH_BYTES (16)
#endif
static void test_ipm_cb(const struct device *ipmdev, void *user_data,
			       uint32_t id, volatile void *data)
{
#if defined(CONFIG_SOC_AST2600)
	int i;
	uint32_t shared_memory_rx = (uint32_t)&shm_rx;
	uint32_t shared_memory_tx = (uint32_t)&shm_tx;

	printk("[CM3] rx addr:%08x, tx addr:%08x\n", shared_memory_rx, shared_memory_tx);
	/* Check the CM3 RX data from CA7 RX data. */
	printk("CM3 RX data from CA7 TX data:\n");
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

	/* Write back to CM3 TX from CM3 RX data. */
	ipm_send(ipmdev, 1, 0, (void *)shared_memory_rx, CONFIG_IPC_SHM_TX_SIZE);
#else
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

void main(void)
{
	int rc;
	const struct device *ipmdev;

#if defined(CONFIG_SOC_AST2600)
	/* setup IPM */
	ipmdev = device_get_binding("ipc@7e6c0000");
	if (!ipmdev) {
		printk("device_get_binding failed to find device\n");
		return;
	}
#elif defined(CONFIG_SOC_AST2700)
#if defined(CONFIG_BOARD_AST2700_SSP_FPGA)
	printk("SSP alive.\n");
	ipmdev = device_get_binding("ipc@0");
	if (!device_is_ready(ipmdev)) {
		while (1) {
		}
	}
	ipm_register_callback(ipmdev, test_ipm_cb, NULL);
	ipm_set_enabled(ipmdev, 1);

	ipmdev = device_get_binding("ipc@200");
	if (!device_is_ready(ipmdev)) {
		while (1) {
		}
	}
	ipm_register_callback(ipmdev, test_ipm_cb, NULL);
	ipm_set_enabled(ipmdev, 1);
#elif defined(CONFIG_BOARD_AST2700_TSP_FPGA)
	printk("TSP alive.\n");
	ipmdev = device_get_binding("ipc@400");
	if (!device_is_ready(ipmdev)) {
		while (1) {
		}
	}
	ipm_register_callback(ipmdev, test_ipm_cb, NULL);
	ipm_set_enabled(ipmdev, 1);

	ipmdev = device_get_binding("ipc@600");
	if (!device_is_ready(ipmdev)) {
		while (1) {
		}
	}
	ipm_register_callback(ipmdev, test_ipm_cb, NULL);
	ipm_set_enabled(ipmdev, 1);
#endif
	ipmdev = device_get_binding("ipc@800");
	if (!device_is_ready(ipmdev)) {
		while (1) {
		}
	}
#endif
	/* setup callback function with received data processing. */
	ipm_register_callback(ipmdev, test_ipm_cb, NULL);

	/* enable ipm. */
	rc = ipm_set_enabled(ipmdev, 1);
	if (rc) {
		printk("cannot ipm_set_enabled\n");
		return;
	}

	printk("IPM initialized\n");

	while (1) {
		k_msleep(1000);
	}
}
