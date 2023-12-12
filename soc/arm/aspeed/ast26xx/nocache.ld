/*
 * Copyright (c) ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
.bss_nc (NOLOAD) :
{
	__bss_nc_start__ = .;
	*(.nocache.bss*)
	. = ALIGN(4);
	__bss_nc_end__ = .;
} > SRAM_NC

.bss_shm_rx (NOLOAD) :
{
	__bss_shm_rx_start__ = .;
	*(.nocache.shm.rx*)
	. = ALIGN(4);
	__bss_shm_rx_end__ = .;
} > SRAM_NC_SHM_RX

.bss_shm_tx (NOLOAD) :
{
	__bss_shm_tx_start__ = .;
	*(.nocache.shm.tx*)
	. = ALIGN(4);
	__bss_shm_tx_end__ = .;
} > SRAM_NC_SHM_TX
