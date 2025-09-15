/**
 * @file
 *
 * @brief Specified low-level inter-processor mailbox communication API for Aspeed chip.
 */

/*
 * Copyright (c) 2025 Aspeed Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_IPM_AST_H_
#define ZEPHYR_INCLUDE_DRIVERS_IPM_AST_H_

/* Show share memory information */
void ast_ipm_list(const struct device *dev);

/* Get tx share memory size */
int ast_ipm_max_tx_shmem_size(const struct device *dev, uint32_t channel);

/* Write the tx shmem */
int ast_ipm_shmem_write(const struct device *dev, uint32_t channel,
uint32_t offset, const void *buf, uint32_t size);

/* Get rx share memory size */
int ast_ipm_max_rx_shmem_size(const struct device *dev, uint32_t channel);

/* Read the rx shmem */
int ast_ipm_shmem_read(const struct device *dev, uint32_t channel,
uint32_t offset, void **buf, uint32_t size);

#endif /* ZEPHYR_INCLUDE_DRIVERS_IPM_H_ */
