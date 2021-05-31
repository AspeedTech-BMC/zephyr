/*
 * Copyright (c) 2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <sys/slist.h>
#include <arch/arm/aarch32/cortex_m/mpu/arm_mpu.h>

/* Some helper defines for aspeed regions */
#define AST_REGION_RAM_ATTR(size) \
{ \
	(NORMAL_OUTER_INNER_WRITE_BACK_WRITE_READ_ALLOCATE_NON_SHAREABLE | \
	 size | P_RW_U_NA_Msk) \
}

static const struct arm_mpu_region mpu_regions[] = {
	/* Region 0 */
	MPU_REGION_ENTRY("FLASH_0",
			 DT_REG_ADDR(DT_INST(0, flash_aspeed)),
			 REGION_FLASH_ATTR(DT_REG_SIZE(DT_INST(0, flash_aspeed)))
			),
	/* Region 1 */
	MPU_REGION_ENTRY("SRAM_0",
			 DT_REG_ADDR(DT_INST(0, mmio_sram)),
			 AST_REGION_RAM_ATTR(DT_REG_SIZE(DT_INST(0, mmio_sram)))
			),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
