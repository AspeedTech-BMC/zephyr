# Copyright (c) 2026 ASPEED Technology Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  core.c
  cmd_v1.c
  cmd_v2.c
  dat_v1.c
  dct_v1.c
  ext_caps.c
  hci_quirks.c
)

zephyr_library_sources_ifdef(CONFIG_I3C_MIPI_HCI_PIO pio.c)
zephyr_library_sources_ifdef(CONFIG_I3C_MIPI_HCI_DMA dma.c)
zephyr_library_sources_ifdef(CONFIG_I3C_MIPI_HCI_TARGET target.c)
