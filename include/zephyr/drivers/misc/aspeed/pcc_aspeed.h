/*
 * Copyright (c) 2022 ASPEED
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_PCC_ASPEED_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_PCC_ASPEED_H_

/*
 * callback to handle PCC RX data
 * @ringbuf: ring buffer holding the received post code
 * @ringbuf_sz: ring buffer size to wrap around
 * @st_idx: index of the first received post code
 * @ed_idx: index of the last received post code + 1
 *
 * i.e. the post code bytes available:
 *   ringbuf[(st_idx % ringbuf_sz) ... ((ed_idx - 1) % ringbuf_sz)]
 */
typedef void pcc_aspeed_rx_callback_t(const uint8_t *ringbuf, uint32_t ringbuf_sz,
				      uint32_t st_idx, uint32_t ed_idx);

int pcc_aspeed_register_rx_callback(const struct device *dev, pcc_aspeed_rx_callback_t *cb);

#endif
