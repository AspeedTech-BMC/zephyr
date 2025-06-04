/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

SECTION_PROLOGUE(.ram_vtor,(NOLOAD),)
{
_ram_vector_start = .;
. += _vector_end - _vector_start;
_ram_vector_end = .;
} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
