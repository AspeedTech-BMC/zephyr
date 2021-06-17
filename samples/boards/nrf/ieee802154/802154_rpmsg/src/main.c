/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/__assert.h>
#include <zephyr.h>

#include <nrf_802154_serialization_error.h>

void nrf_802154_serialization_error(const nrf_802154_ser_err_data_t *err)
{
	(void)err;
	__ASSERT(false, "802.15.4 serialization error");
}

void nrf_802154_sl_fault_handler(uint32_t id, int32_t line, const char *err)
{
	__ASSERT(false, "module_id: %u, line: %d, %s", id, line, err);
}
