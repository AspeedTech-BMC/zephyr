# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(
  CONFIG_I3C_TARGET_MQUEUE
  i3c_target_mqueue.c
)
