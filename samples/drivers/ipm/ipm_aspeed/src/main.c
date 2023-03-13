#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/ipm.h>

static void test_ipm_cb(const struct device *ipmdev, void *user_data,
			       uint32_t id, volatile void *data)
{
	int i;
	int max_data_size;
	uint8_t *buf = (uint8_t *)data;

	printk("%s:id %x, msg at %p\n", __func__, id, data);

	max_data_size = ipm_max_data_size_get(ipmdev);

	for (i = 0; i < max_data_size; i++) {
		printk("%02x ", *buf++);
		if ((i & 0xf) == 0xf) {
			printk("\n");
		}
	}
}

void main(void)
{
	const struct device *ipmdev;

	ipmdev = device_get_binding("ipc@0");
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