# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2020 Linumiz
# Author: Saravanan Sekar <saravanan@linumiz.com>

zephyr_sources(soc.c)
zephyr_linker_sources(ROM_START SORT_KEY 0x1sboot sboot.ld)
