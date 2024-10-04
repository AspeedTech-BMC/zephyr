/*
 * Copyright (c) 2022 - 2023 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_ABR_ASPEED_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_ABR_ASPEED_H_

enum boot_indicator {
	BOOT_FROM_PRIMARY_PART,
	BOOT_FROM_ALTERNATE_PART,
};

void disable_abr_wdt(void);
void clear_abr_event_count(void);
void clear_abr_indicator(void);

#endif
