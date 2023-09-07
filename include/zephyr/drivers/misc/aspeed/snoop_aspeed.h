/*
 * Copyright (c) 2021 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_SNOOP_ASPEED_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_SNOOP_ASPEED_H_

#define SNOOP_CHANNEL_NUM	2

/*
 * callback to handle snoop RX data
 * @snoop0: pointer to the byte snooped by channel 0, NULL if no data
 * @snoop1: pointer to the byte snooped by channel 1, NULL if no data
 */
typedef void snoop_aspeed_rx_callback_t(const uint8_t *snoop0, const uint8_t *snoop1);

int snoop_aspeed_register_rx_callback(const struct device *dev, snoop_aspeed_rx_callback_t cb);

#endif
