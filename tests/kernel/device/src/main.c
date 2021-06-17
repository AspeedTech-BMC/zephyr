/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <init.h>
#include <ztest.h>
#include <sys/printk.h>
#include <pm/device_runtime.h>
#include "abstract_driver.h"


#define DUMMY_PORT_1    "dummy"
#define DUMMY_PORT_2    "dummy_driver"
#define BAD_DRIVER	"bad_driver"

#define MY_DRIVER_A     "my_driver_A"
#define MY_DRIVER_B     "my_driver_B"

extern void test_mmio_multiple(void);
extern void test_mmio_toplevel(void);
extern void test_mmio_single(void);
extern void test_mmio_device_map(void);

/**
 * @brief Test cases to verify device objects
 *
 * Verify zephyr device driver apis with different device types
 *
 * @defgroup kernel_device_tests Device
 *
 * @ingroup all_tests
 *
 * @{
 */

/**
 * @brief Test device object binding
 *
 * Validates device binding for an existing and a non-existing device object.
 * It creates a dummy_driver device object with basic init and configuration
 * information and validates its binding.
 *
 * Validates three kinds situations of driver object:
 * 1. A non-existing device object.
 * 2. An existing device object with basic init and configuration information.
 * 3. A failed init device object.
 *
 * @ingroup kernel_device_tests
 *
 * @see device_get_binding(), device_busy_set(), device_busy_clear(),
 * DEVICE_DEFINE()
 */
void test_dummy_device(void)
{
	const struct device *dev;

	/* Validates device binding for a non-existing device object */
	dev = device_get_binding(DUMMY_PORT_1);
	zassert_equal(dev, NULL, NULL);

	/* Validates device binding for an existing device object */
	dev = device_get_binding(DUMMY_PORT_2);
	zassert_false((dev == NULL), NULL);

	device_busy_set(dev);
	device_busy_clear(dev);

	/* device_get_binding() returns false for device object
	 * with failed init.
	 */
	dev = device_get_binding(BAD_DRIVER);
	zassert_true((dev == NULL), NULL);
}

/**
 * @brief Test device binding for existing device
 *
 * Validates device binding for an existing device object.
 *
 * @see device_get_binding(), DEVICE_DEFINE()
 */
static void test_dynamic_name(void)
{
	const struct device *mux;
	char name[sizeof(DUMMY_PORT_2)];

	snprintk(name, sizeof(name), "%s", DUMMY_PORT_2);
	mux = device_get_binding(name);
	zassert_true(mux != NULL, NULL);
}

/**
 * @brief Test device binding for non-existing device
 *
 * Validates binding of a random device driver(non-defined driver) named
 * "ANOTHER_BOGUS_NAME".
 *
 * @see device_get_binding(), DEVICE_DEFINE()
 */
static void test_bogus_dynamic_name(void)
{
	const struct device *mux;
	char name[64];

	snprintk(name, sizeof(name), "ANOTHER_BOGUS_NAME");
	mux = device_get_binding(name);
	zassert_true(mux == NULL, NULL);
}

/**
 * @brief Test device binding for passing null name
 *
 * Validates device binding for device object when given dynamic name is null.
 *
 * @see device_get_binding(), DEVICE_DEFINE()
 */
static void test_null_dynamic_name(void)
{
	/* Supplying a NULL dynamic name may trigger a SecureFault and
	 * lead to system crash in TrustZone enabled Non-Secure builds.
	 */
#if defined(CONFIG_USERSPACE) && !defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
	const struct device *mux;
	char *drv_name = NULL;

	mux = device_get_binding(drv_name);
	zassert_equal(mux, 0,  NULL);
#else
	ztest_test_skip();
#endif
}

static struct init_record {
	bool pre_kernel;
	bool is_in_isr;
	bool is_pre_kernel;
} init_records[4];

static struct init_record *rp = init_records;

static int add_init_record(bool pre_kernel)
{
	rp->pre_kernel = pre_kernel;
	rp->is_pre_kernel = k_is_pre_kernel();
	rp->is_in_isr = k_is_in_isr();
	++rp;
	return 0;
}

static int pre1_fn(const struct device *dev)
{
	return add_init_record(true);
}

static int pre2_fn(const struct device *dev)
{
	return add_init_record(true);
}

static int post_fn(const struct device *dev)
{
	return add_init_record(false);
}

static int app_fn(const struct device *dev)
{
	return add_init_record(false);
}

SYS_INIT(pre1_fn, PRE_KERNEL_1, 0);
SYS_INIT(pre2_fn, PRE_KERNEL_2, 0);
SYS_INIT(post_fn, POST_KERNEL, 0);
SYS_INIT(app_fn, APPLICATION, 0);

/* This is an error case which driver initializes failed in SYS_INIT .*/
static int null_driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -EINVAL;
}

SYS_INIT(null_driver_init, POST_KERNEL, 0);

/**
 * @brief Test detection of initialization before kernel services available.
 *
 * Confirms check is correct.
 *
 * @see k_is_pre_kernel()
 */
void test_pre_kernel_detection(void)
{
	struct init_record *rpe = rp;

	zassert_equal(rp - init_records, 4U,
		      "bad record count");
	rp = init_records;
	while ((rp < rpe) && rp->pre_kernel) {
		zassert_equal(rp->is_in_isr, false,
			      "rec %zu isr", rp - init_records);
		zassert_equal(rp->is_pre_kernel, true,
			      "rec %zu pre-kernel", rp - init_records);
		++rp;
	}
	zassert_equal(rp - init_records, 2U,
		      "bad pre-kernel count");

	while (rp < rpe) {
		zassert_equal(rp->is_in_isr, false,
			      "rec %zu isr", rp - init_records);
		zassert_equal(rp->is_pre_kernel, false,
			      "rec %zu post-kernel", rp - init_records);
		++rp;
	}
}

#ifdef CONFIG_PM_DEVICE
/**
 * @brief Test system device list query API with PM enabled.
 *
 * It queries the list of devices in the system, used to suspend or
 * resume the devices in PM applications.
 *
 * @see z_device_get_all_static()
 */
static void test_build_suspend_device_list(void)
{
	struct device const *devices;
	size_t devcount = z_device_get_all_static(&devices);

	zassert_false((devcount == 0), NULL);
}

/**
 * @brief Test APIs to enable and disable automatic runtime power management
 *
 * @details Test the API enable and disable, cause we do not implement our PM
 * API here, it will use the default function to handle power status. So when
 * we try to get power state by pm_device_state_get(), it will default
 * return power state zero. And we check it.
 *
 * @ingroup kernel_device_tests
 */
static void test_enable_and_disable_automatic_runtime_pm(void)
{
	const struct device *dev;
	int ret;
	unsigned int device_power_state = 0;

	dev = device_get_binding(DUMMY_PORT_2);
	zassert_false((dev == NULL), NULL);

	/* check its status at first */
	/* for cases that cannot run runtime PM, we skip it now */
	ret = pm_device_state_get(dev, &device_power_state);
	if (ret == -ENOSYS) {
		TC_PRINT("Power management not supported on device");
		ztest_test_skip();
		return;
	}

	zassert_true((ret == 0),
		"Unable to get active state to device");

	/* enable automatic runtime PM and check its status */
	pm_device_enable(dev);
	zassert_not_null((dev->pm), "No device pm");
	zassert_true((dev->pm->enable), "Pm is not enable");

	/* disable automatic runtime PM and check its status */
	pm_device_disable(dev);
	zassert_false((dev->pm->enable), "Pm shall not be enable");
}

/**
 * @brief Test device binding for existing device with PM enabled.
 *
 * Validates device binding for an existing device object with Power management
 * enabled. It also checks if the device is in the middle of a transaction,
 * sets/clears busy status and validates status again.
 *
 * @see device_get_binding(), device_busy_set(), device_busy_clear(),
 * device_busy_check(), device_any_busy_check(),
 * pm_device_state_set()
 */
void test_dummy_device_pm(void)
{
	const struct device *dev;
	int busy, ret;
	unsigned int device_power_state = 0;

	dev = device_get_binding(DUMMY_PORT_2);
	zassert_false((dev == NULL), NULL);

	busy = device_any_busy_check();
	zassert_true((busy == 0), NULL);

	/* Set device state to BUSY*/
	device_busy_set(dev);

	busy = device_any_busy_check();
	zassert_false((busy == 0), NULL);

	busy = device_busy_check(dev);
	zassert_false((busy == 0), NULL);

	/* Clear device BUSY state*/
	device_busy_clear(dev);

	busy = device_busy_check(dev);
	zassert_true((busy == 0), NULL);

	test_build_suspend_device_list();

	/* Set device state to PM_DEVICE_STATE_ACTIVE */
	ret = pm_device_state_set(dev, PM_DEVICE_STATE_ACTIVE, NULL, NULL);
	if (ret == -ENOSYS) {
		TC_PRINT("Power management not supported on device");
		ztest_test_skip();
		return;
	}

	zassert_true((ret == 0),
			"Unable to set active state to device");

	ret = pm_device_state_get(dev, &device_power_state);
	zassert_true((ret == 0),
			"Unable to get active state to device");
	zassert_true((device_power_state == PM_DEVICE_STATE_ACTIVE),
			"Error power status");

	/* Set device state to PM_DEVICE_STATE_FORCE_SUSPEND */
	ret = pm_device_state_set(dev,
		PM_DEVICE_STATE_FORCE_SUSPEND, NULL, NULL);

	zassert_true((ret == 0), "Unable to force suspend device");

	ret = pm_device_state_get(dev, &device_power_state);
	zassert_true((ret == 0),
			"Unable to get suspend state to device");
	zassert_true((device_power_state == PM_DEVICE_STATE_ACTIVE),
			"Error power status");
}
#else
static void test_enable_and_disable_automatic_runtime_pm(void)
{
	ztest_test_skip();
}

static void test_build_suspend_device_list(void)
{
	ztest_test_skip();
}

void test_dummy_device_pm(void)
{
	ztest_test_skip();
}
#endif

/* this is for storing sequence during initializtion */
extern int init_level_sequence[4];
extern int init_priority_sequence[4];
extern unsigned int seq_level_cnt;
extern unsigned int seq_priority_cnt;

/**
 * @brief Test initialization level for device driver instances
 *
 * @details After the defined device instances have initialized, we check the
 * sequence number that each driver stored during initialization. If the
 * sequence of initial level stored is corresponding with our expectation, it
 * means assigning the level for driver instance works.
 *
 * @ingroup kernel_device_tests
 */
void test_device_init_level(void)
{
	bool seq_correct = true;

	/* we check if the stored executing sequence for different level is
	 * correct, and it should be 1, 2, 3, 4
	 */
	for (int i = 0; i < 4; i++) {
		if (init_level_sequence[i] != (i+1))
			seq_correct = false;
	}

	zassert_true((seq_correct == true),
			"init sequence is not correct");
}

/**
 * @brief Test initialization priorities for device driver instances
 *
 * details After the defined device instances have initialized, we check the
 * sequence number that each driver stored during initialization. If the
 * sequence of initial priority stored is corresponding with our expectation, it
 * means assigning the priority for driver instance works.
 *
 * @ingroup kernel_device_tests
 */
void test_device_init_priority(void)
{
	bool sequence_correct = true;

	/* we check if the stored pexecuting sequence for priority is correct,
	 * and it should be 1, 2, 3, 4
	 */
	for (int i = 0; i < 4; i++) {
		if (init_priority_sequence[i] != (i+1))
			sequence_correct = false;
	}

	zassert_true((sequence_correct == true),
			"init sequence is not correct");
}


/**
 * @brief Test abstraction of device drivers with common functionalities
 *
 * @details Abstraction of device drivers with common functionalities
 * shall be provided as an intermediate interface between applications
 * and device drivers, where such interface is implemented by individual
 * device drivers. We verify this by following step:

 * 1. Define a subsystem api for drivers.
 * 2. Define and create two driver instances.
 * 3. Two drivers call the same subsystem API, and we verify that each
 * driver instance will call their own implementations.
 *
 * @ingroup kernel_device_tests
 */
void test_abstraction_driver_common(void)
{
	const struct device *dev;
	int ret;
	int foo = 2;
	int bar = 1;
	unsigned int baz = 0;

	/* verify driver A API has called */
	dev = device_get_binding(MY_DRIVER_A);
	zassert_false((dev == NULL), NULL);

	ret = subsystem_do_this(dev, foo, bar);
	zassert_true(ret == (foo + bar), "common API do_this fail");

	subsystem_do_that(dev, &baz);
	zassert_true(baz == 1, "common API do_that fail");

	/* verify driver B API has called */
	dev = device_get_binding(MY_DRIVER_B);
	zassert_false((dev == NULL), NULL);

	ret = subsystem_do_this(dev, foo, bar);
	zassert_true(ret == (foo - bar), "common API do_this fail");

	subsystem_do_that(dev, &baz);
	zassert_true(baz == 2, "common API do_that fail");
}


/**
 * @}
 */

void test_main(void)
{
	ztest_test_suite(device,
			 ztest_unit_test(test_dummy_device_pm),
			 ztest_unit_test(test_build_suspend_device_list),
			 ztest_unit_test(test_dummy_device),
			 ztest_unit_test(test_enable_and_disable_automatic_runtime_pm),
			 ztest_unit_test(test_pre_kernel_detection),
			 ztest_user_unit_test(test_bogus_dynamic_name),
			 ztest_user_unit_test(test_null_dynamic_name),
			 ztest_user_unit_test(test_dynamic_name),
			 ztest_unit_test(test_device_init_level),
			 ztest_unit_test(test_device_init_priority),
			 ztest_unit_test(test_abstraction_driver_common),
			 ztest_unit_test(test_mmio_single),
			 ztest_unit_test(test_mmio_multiple),
			 ztest_unit_test(test_mmio_toplevel),
			 ztest_unit_test(test_mmio_device_map));
	ztest_run_test_suite(device);
}
