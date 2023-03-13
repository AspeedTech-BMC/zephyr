#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/ipm.h>

#define DEFAULT_LINE_LENGTH_BYTES (16)
static void test_ipm_cb(const struct device *ipmdev, void *user_data,
			       uint32_t id, volatile void *data)
{
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
}

void main(void)
{
	const struct device *ipmdev;

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
	ipm_register_callback(ipmdev, test_ipm_cb, NULL);
	ipm_set_enabled(ipmdev, 1);
	printk("IPM initialized\n");

	while (1) {
		k_msleep(1000);
	}
}