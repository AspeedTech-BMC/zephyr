#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/misc/aspeed/pfr_aspeed.h>
#include <drivers/gpio.h>
#include <crypto/hash_structs.h>
#include <crypto/hash.h>
#include <shell/shell.h>
#include <kernel.h>
#include <syscall.h>
#include <syscall_list.h>

#define ASPEED_HASH_TEST_SUPPORT 1
#define ASPEED_GPIO_TEST_SUPPORT 0
#define  GPIO_PIN_TST 0

uint32_t TestCount = 0;

#if ASPEED_HASH_TEST_SUPPORT

struct hash_testvec {
	const char *plaintext;
	const char *digest;
	unsigned int psize;
};
static const struct hash_testvec sha256_tv_template[] = {
	{
		.plaintext = "",
		.psize	= 0,
		.digest	= "\xe3\xb0\xc4\x42\x98\xfc\x1c\x14"
		"\x9a\xfb\xf4\xc8\x99\x6f\xb9\x24"
		"\x27\xae\x41\xe4\x64\x9b\x93\x4c"
		"\xa4\x95\x99\x1b\x78\x52\xb8\x55",
	}, {
		.plaintext = "abc",
		.psize	= 3,
		.digest	= "\xba\x78\x16\xbf\x8f\x01\xcf\xea"
		"\x41\x41\x40\xde\x5d\xae\x22\x23"
		"\xb0\x03\x61\xa3\x96\x17\x7a\x9c"
		"\xb4\x10\xff\x61\xf2\x00\x15\xad",
	}, {
		.plaintext = "Im Superman",
		.psize	= sizeof("Im Superman") - 1,
		.digest	= \
		"\xb2\x25\xcc\x6e\xd4\x58\x28\x06"
		"\x1b\x24\xa2\xd9\x52\x6e\xca\x9a"
		"\xed\x38\x23\xe0\x8e\x6b\xbf\x21"
		"\x06\xb8\xcb\xe7\xd1\x16\x59\x6d",
	},
};

static const struct hash_testvec sha384_tv_template[] = {
	{
		.plaintext = "Im Nero",
		.psize	= sizeof("Im Nero") - 1,
		.digest	= \
		"\x5f\x25\x96\xd4\x76\x54\x8c\xe3"
		"\x4c\x2c\xfe\x4d\x19\xf7\x11\xcd"
		"\x27\x29\x99\x80\x24\x35\x7b\x43"
		"\x12\xf4\x0a\x3b\x71\xce\x6c\x03"
		"\xc3\x09\x68\x62\x93\x84\x6e\xc8"
		"\x1f\xfa\x4a\x87\x23\xd7\xb9\x21",
	},
};

static uint8_t AspeedHashTest(void)
{
	const struct device *dev = device_get_binding(CONFIG_CRYPTO_ASPEED_HASH_DRV_NAME);
	struct hash_ctx ini;
	struct hash_pkt pkt;
	uint8_t digest[64];

	const struct hash_testvec *tv = sha256_tv_template;	//Test Vectors
	enum hash_algo algo = HASH_SHA256;
	uint8_t i = 2, ret = 0, index = 0;

	printk("%20s : 0x%X\n", "HeapCount", TestCount);

	printk("tv[%d]:", i);

	pkt.in_buf = (uint8_t *)tv[i].plaintext;
	pkt.in_len = tv[i].psize;
	pkt.out_buf = digest;
	pkt.out_buf_max = sizeof(digest);

	if(dev != NULL)
	{
		const char *digestPtr;
		digestPtr = (uint8_t *)tv[i].digest;

		for(index = 0; index < 33; index++)
			printk("%x", (uint8_t)(*(digestPtr + index)));

		printk("%s\n", dev->name);
	}
	else
	{
		printk("[%d]Fail in %s\n", __LINE__, __FILE__);

		return 0xFF;
	}

	printk("-------------------------------hash_begin_session\n");
	
	ret = hash_begin_session(dev, &ini, algo);

	if (ret) {
		printk("hash_begin_session error\n");
		return ret;
	}

	printk("-------------------------------hash_update\n");

	ret = hash_update(&ini, &pkt);

	if (ret) {
		printk("hash_update error\n");
		goto out;
	}

	printk("-------------------------------hash_final\n");

	ret = hash_final(&ini, &pkt);

	if (ret) {
		printk("hash_final error\n");
		goto out;
	}

	printk("-------------------------------hash_free_session\n");
	
	hash_free_session(dev, &ini);

	printk("Tv Size : %d\n", tv->psize);

	for(index = 0; index < ini.digest_size; index++)
		printk("%x", (uint8_t)(*(digest + index)));

	printk("\n");

	for(index = 0; index < ini.digest_size; index++)
		printk("%x", (uint8_t)(*(tv[i].digest + index)));

	printk("\n======SHA256=====\n");
	
	if (!memcmp(digest, tv[i].digest, ini.digest_size))
		printk("PASS\n");
	else
		printk("FAIL\n");

	printk("=================\n");

	tv = sha384_tv_template;	//Test Vectors
	algo = HASH_SHA384;

	i = 0;
	pkt.in_buf = (uint8_t *)tv[i].plaintext;
	pkt.in_len = tv[i].psize;
	pkt.out_buf = digest;
	pkt.out_buf_max = sizeof(digest);

	if(dev != NULL)
	{
		const char *digestPtr;
		digestPtr = (uint8_t *)tv[i].digest;

		for(index = 0; index < 33; index++)
			printk("%x", (uint8_t)(*(digestPtr + index)));

		printk("%s\n", dev->name);
	}
	else
	{
		printk("[%d]Fail in %s\n", __LINE__, __FILE__);

		return 0xFF;
	}

	printk("-------------------------------hash_begin_session\n");
	
	ret = hash_begin_session(dev, &ini, algo);

	if (ret) {
		printk("hash_begin_session error\n");
		return ret;
	}

	printk("-------------------------------hash_update\n");

	ret = hash_update(&ini, &pkt);

	if (ret) {
		printk("hash_update error\n");
		goto out;
	}

	printk("-------------------------------hash_final\n");

	ret = hash_final(&ini, &pkt);

	if (ret) {
		printk("hash_final error\n");
		goto out;
	}

	printk("-------------------------------hash_free_session\n");
	
	hash_free_session(dev, &ini);

	printk("Tv Size : %d\n", tv->psize);

	for(index = 0; index < ini.digest_size; index++)
		printk("%x", (uint8_t)(*(digest + index)));

	printk("\n");

	for(index = 0; index < ini.digest_size; index++)
		printk("%x", (uint8_t)(*(tv[i].digest + index)));

	printk("\n======SHA384=====\n");
	
	if (!memcmp(digest, tv[i].digest, ini.digest_size))
		printk("PASS\n");
	else
		printk("FAIL\n");

	printk("=================\n");

	return 0;

out:
	hash_free_session(dev, &ini);
	return ret;
}
#endif

#if ASPEED_GPIO_TEST_SUPPORT
void AspeedGpioTest(void)
{
	uint8_t enable = 0;
	uint8_t count = 0;

	printk("GPIO Base Address : %X\n", Gpiotest());

	/* GPIOL0 */
	const struct device *gpio_dev = NULL;

	gpio_dev = device_get_binding("GPIO0_I_L");

	if (gpio_dev == NULL)
	{
		printk("[%d]Fail to get GPIO0_I_L", __LINE__);
		return;
	}
	else
	{
		printk("Device Name : %s\n", gpio_dev->name);
	}

	for(count = 0; count < 32; count++)
		gpio_pin_configure(gpio_dev, GPIO_PIN_TST, GPIO_OUTPUT);

	k_busy_wait(10000); /* 10ms */

	for(;;) 
	{
		if (enable)
			for(count = 0; count < 32; count++)
				gpio_pin_set(gpio_dev, count, 0);
		else
			for(count = 0; count < 32; count++)
				gpio_pin_set(gpio_dev, count, 1);

		k_busy_wait(500000); /* 500ms */

		enable = (enable) ? (0) : (1);

		for(count = 31; count != 255; count--)
			printk("%X ", gpio_pin_get(gpio_dev, count));

		printk("\nEnable : %X\n", enable);
	}
}
#endif
