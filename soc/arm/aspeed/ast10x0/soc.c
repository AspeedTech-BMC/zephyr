/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2021 ASPEED Technology Inc.
 */

#include <init.h>
#include <kernel.h>
#include <stdint.h>
#include <linker/linker-defs.h>

struct sb_header {
        uint32_t key_location;
        uint32_t enc_img_addr;
        uint32_t img_size;
        uint32_t sign_location;
        uint32_t header_rev[2];
        uint32_t patch_location;        /* address of the rom patch */
        uint32_t checksum;
};

struct sb_header sbh __attribute((used, section(".sboot"))) = {
        .img_size = (uint32_t)&_image_rom_end,
};

#define JTAG_PINMUX_REG	0x41c

void z_platform_init(void)
{
	uint32_t jtag_pinmux;
	uint32_t base = DT_REG_ADDR(DT_NODELABEL(syscon));

	/* enable JTAG pins */
	jtag_pinmux = sys_read32(base + JTAG_PINMUX_REG);
	jtag_pinmux |= (0x1f << 25);
	sys_write32(jtag_pinmux, base + JTAG_PINMUX_REG);
}
