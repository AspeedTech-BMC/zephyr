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

struct ipm_shell_shmem {
	uintptr_t shmem_tx_base;
	uintptr_t shmem_rx_base;
	unsigned int shmem_tx_size;
	unsigned int shmem_rx_size;
};

/* Show share memory information */
void ast_ipm_list(const struct device *dev);

/* Get share memory information */
void ast_ipm_shmem_info(const struct device *dev, uint32_t channel, struct ipm_shell_shmem *info);

#endif /* ZEPHYR_INCLUDE_DRIVERS_IPM_H_ */
