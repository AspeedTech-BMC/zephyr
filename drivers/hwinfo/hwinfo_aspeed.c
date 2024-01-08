/*
 * Copyright (c) 2021 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/syscon.h>
#include <string.h>

#define ASPEED_REVISION_ID0		0x04
#define ASPEED_REVISION_ID1		0x14

#define SYS_RESET_LOG1			0x74
#define  SYS_RESET_LOG1_WDT4		GENMASK(31, 28)
#define  SYS_RESET_LOG1_WDT3		GENMASK(27, 24)
#define  SYS_RESET_LOG1_WDT2		GENMASK(23, 20)
#define  SYS_RESET_LOG1_WDT1		GENMASK(19, 16)
#define    SYS_WDT_RESET_SW		BIT(3)
#define    SYS_WDT_RESET_ARM		BIT(2)
#define    SYS_WDT_RESET_FULL		BIT(1)
#define    SYS_WDT_RESET_SOC		BIT(0)
#define  SYS_RESET_LOG1_MMC_ECC		BIT(3)
#define  SYS_RESET_LOG1_ABR		BIT(2)
#define  SYS_RESET_LOG1_EXTRST		BIT(1)
#define  SYS_RESET_LOG1_POR_SRST	BIT(0)


ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint8_t rev_id[8];

	if (length < sizeof(rev_id))
		return -1;

	syscon_read_reg(dev, ASPEED_REVISION_ID0, (uint32_t *)&rev_id[0]);
	syscon_read_reg(dev, ASPEED_REVISION_ID1, (uint32_t *)&rev_id[4]);
	memcpy(buffer, rev_id, sizeof(rev_id));

	return sizeof(rev_id);
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t event_log, flags = 0;

	syscon_read_reg(dev, SYS_RESET_LOG1, &event_log);

	if (event_log & SYS_RESET_LOG1_POR_SRST) {
		flags |= RESET_POR;
	} else {
		if (event_log & SYS_RESET_LOG1_EXTRST) {
			flags |= RESET_PIN;
		}

		if (event_log & SYS_RESET_LOG1_WDT1 || event_log & SYS_RESET_LOG1_WDT2 ||
		    event_log & SYS_RESET_LOG1_WDT3 || event_log & SYS_RESET_LOG1_WDT4) {
			flags |= RESET_WATCHDOG | RESET_SOFTWARE;
		}

		if (event_log & SYS_RESET_LOG1_ABR) {
			flags |= RESET_WATCHDOG | RESET_HARDWARE;
		}
	}

	*cause = flags;

	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(syscon));
	uint32_t event_log;

	syscon_read_reg(dev, SYS_RESET_LOG1, &event_log);
	syscon_write_reg(dev, SYS_RESET_LOG1, event_log);

	return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = RESET_PIN | RESET_WATCHDOG | RESET_POR | RESET_HARDWARE | RESET_SOFTWARE;

	return 0;
}
