/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include "arm_mpu_mem_cfg.h"

/* Some helper defines for aspeed regions */
#define AST_REGION_RAM_ATTR(size)						   \
	{									   \
		(NORMAL_OUTER_INNER_WRITE_BACK_WRITE_READ_ALLOCATE_NON_SHAREABLE | \
		 size | P_RW_U_NA_Msk)						   \
	}

static const struct arm_mpu_region mpu_regions[] = {
	/* Region 0 */
	MPU_REGION_ENTRY("FLASH_0",
			 CONFIG_FLASH_BASE_ADDRESS,
			 REGION_FLASH_ATTR(REGION_FLASH_SIZE)),
	/* Region 1 */
	MPU_REGION_ENTRY("SRAM_0",
			 CONFIG_SRAM_BASE_ADDRESS,
			 AST_REGION_RAM_ATTR(REGION_SRAM_SIZE)),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
