/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_aspeed.h>
#include <zephyr/sys/sys_io.h>

#ifdef CONFIG_ESPI_TAF
#include "safs_handler.h"
#endif

#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
extern void oob_async_init(void);
#endif

LOG_MODULE_REGISTER(espi_shell, CONFIG_LOG_DEFAULT_LEVEL);

#define ESPI_GEN_CAP			0x014
#define ESPI_CH0_CAP			0x018
#define ESPI_CH1_CAP			0x01c
#define ESPI_CH2_CAP			0x020
#define ESPI_CH3_CAP			0x024
#define ESPI_CH3_CAP_2			0x028

#define ESPI_DEV_STS			0x030
#define ESPI_CH0_STS			0x104
#define ESPI_CH1_STS			0x204
#define ESPI_CH2_STS			0x304
#define ESPI_CH3_STS			0x404

const struct device *espi_dev = DEVICE_DT_GET(DT_NODELABEL(espi0));

static int cmd_espi_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!espi_dev) {
		shell_error(sh, "eSPI device not found");
		return -ENODEV;
	}

	shell_print(sh, "eSPI device: %s", espi_dev->name);
	shell_print(sh, "eSPI device ready: %d", device_is_ready(espi_dev));

	return 0;
}

static void print_espi_status(const struct shell *sh, const char *label, uint16_t sts)
{
	shell_print(sh, "%s: 0x%04x", label, sts);
	shell_print(sh, "  [0]  PC_FREE       : %u", (sts >> 0)  & 1);
	shell_print(sh, "  [1]  NP_FREE       : %u", (sts >> 1)  & 1);
	shell_print(sh, "  [2]  VWIRE_FREE    : %u", (sts >> 2)  & 1);
	shell_print(sh, "  [3]  OOB_FREE      : %u", (sts >> 3)  & 1);
	shell_print(sh, "  [4]  PC_AVAIL      : %u", (sts >> 4)  & 1);
	shell_print(sh, "  [5]  NP_AVAIL      : %u", (sts >> 5)  & 1);
	shell_print(sh, "  [6]  VWIRE_AVAIL   : %u", (sts >> 6)  & 1);
	shell_print(sh, "  [7]  OOB_AVAIL     : %u", (sts >> 7)  & 1);
	shell_print(sh, "  [8]  FLASH_C_FREE  : %u", (sts >> 8)  & 1);
	shell_print(sh, "  [10] FLASH_NP_FREE : %u", (sts >> 10) & 1);
	shell_print(sh, "  [12] FLASH_C_AVAIL : %u", (sts >> 12) & 1);
	shell_print(sh, "  [14] FLASH_NP_AVAIL: %u", (sts >> 14) & 1);
}

static int cmd_espi_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uintptr_t espi_base = DT_REG_ADDR(DT_NODELABEL(espi0));
	uint32_t val = sys_read32(espi_base + ESPI_DEV_STS);

	shell_print(sh, "ESPI030(ESPI_DEV_STS): 0x%08x", val);
	shell_print(sh, "");
	print_espi_status(sh, "Previous status [15:0] ", (uint16_t)(val & 0xffff));
	shell_print(sh, "");
	print_espi_status(sh, "Current  status [31:16]", (uint16_t)(val >> 16));

	return 0;
}

static int cmd_espi_ch_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uintptr_t espi_base = DT_REG_ADDR(DT_NODELABEL(espi0));
	static const struct {
		const char *name;
		uint32_t offset;
	} channels[] = {
		{ "CH0 (Peripheral)", ESPI_CH0_STS },
		{ "CH1 (VW)",         ESPI_CH1_STS },
		{ "CH2 (OOB)",        ESPI_CH2_STS },
		{ "CH3 (Flash)",      ESPI_CH3_STS },
	};

	for (int i = 0; i < ARRAY_SIZE(channels); i++) {
		uint32_t val = sys_read32(espi_base + channels[i].offset);

		shell_print(sh, "%s (0x%03x): 0x%08x", channels[i].name, channels[i].offset, val);
	}

	return 0;
}

static int cmd_espi_ch_cap(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uintptr_t base = DT_REG_ADDR(DT_NODELABEL(espi0));
	uint32_t v;

	/* General Capabilities (0x014) */
	v = sys_read32(base + ESPI_GEN_CAP);
	shell_print(sh, "GEN_CAP (0x014): 0x%08x", v);
	shell_print(sh, "  [2:0]  OP freq supported : %u (0=20M 1=25M 2=33M 3=50M 4=66M)", v & 0x7);
	shell_print(sh, "  [4:3]  OP freq selected  : %u", (v >> 3) & 0x3);
	shell_print(sh, "  [6:5]  IO mode supported : %u (0=x1 1=x1/x2 2=x1/x4 3=x1/x2/x4)",
		    (v >> 5) & 0x3);
	shell_print(sh, "  [8:7]  IO mode selected  : %u (0=x1 1=x2 2=x4)", (v >> 7) & 0x3);
	shell_print(sh, "  [9]    Alert mode        : %u (0=IO1 1=pin)", (v >> 9) & 0x1);
	shell_print(sh, "  [10]   Alert pin mode    : %u (0=OD 1=PP)", (v >> 10) & 0x1);
	shell_print(sh, "  [12]   CRC checking      : %u", (v >> 12) & 0x1);
	shell_print(sh, "  [20]   eSPI Reset        : %u", (v >> 20) & 0x1);
	shell_print(sh, "");

	/* CH0 Peripheral Channel Capabilities (0x018) */
	v = sys_read32(base + ESPI_CH0_CAP);
	shell_print(sh, "CH0_CAP (0x018): 0x%08x  [Peripheral]", v);
	shell_print(sh, "  [0]    CH supported      : %u", v & 0x1);
	shell_print(sh, "  [1]    CH enabled        : %u", (v >> 1) & 0x1);
	shell_print(sh, "  [2]    CH ready          : %u", (v >> 2) & 0x1);
	shell_print(sh, "  [11:8] Max payload (supp): %u (0=64 1=128 2=256)", (v >> 8) & 0xf);
	shell_print(sh, "  [15:12]Max payload (sel) : %u", (v >> 12) & 0xf);
	shell_print(sh, "  [19:16]Max rd req size   : %u", (v >> 16) & 0xf);
	shell_print(sh, "");

	/* CH1 Virtual Wire Channel Capabilities (0x01c) */
	v = sys_read32(base + ESPI_CH1_CAP);
	shell_print(sh, "CH1_CAP (0x01c): 0x%08x  [Virtual Wire]", v);
	shell_print(sh, "  [0]    CH supported      : %u", v & 0x1);
	shell_print(sh, "  [1]    CH enabled        : %u", (v >> 1) & 0x1);
	shell_print(sh, "  [2]    CH ready          : %u", (v >> 2) & 0x1);
	shell_print(sh, "  [13:8] Max VW cnt (supp) : %u", (v >> 8) & 0x3f);
	shell_print(sh, "  [21:16]Max VW cnt (sel)  : %u", (v >> 16) & 0x3f);
	shell_print(sh, "");

	/* CH2 OOB Channel Capabilities (0x020) */
	v = sys_read32(base + ESPI_CH2_CAP);
	shell_print(sh, "CH2_CAP (0x020): 0x%08x  [OOB]", v);
	shell_print(sh, "  [0]    CH supported      : %u", v & 0x1);
	shell_print(sh, "  [1]    CH enabled        : %u", (v >> 1) & 0x1);
	shell_print(sh, "  [2]    CH ready          : %u", (v >> 2) & 0x1);
	shell_print(sh, "  [11:8] Max payload (supp): %u (0=64 1=128 2=256)", (v >> 8) & 0xf);
	shell_print(sh, "  [15:12]Max payload (sel) : %u", (v >> 12) & 0xf);
	shell_print(sh, "");

	/* CH3 Flash Channel Capabilities (0x024) */
	v = sys_read32(base + ESPI_CH3_CAP);
	shell_print(sh, "CH3_CAP (0x024): 0x%08x  [Flash]", v);
	shell_print(sh, "  [0]    CH enable         : %u", (v >> 0) & 0x1);
	shell_print(sh, "  [1]    CH ready          : %u", (v >> 1) & 0x1);
	shell_print(sh, "  [4:2]  Erase size        : %u (1=4K 2=64K 3=4K+64K 4=128K 5=256K)",
		    (v >> 2) & 0x7);
	shell_print(sh, "  [7:5]  Max payload (supp): %u (1=64 2=128 3=256)", (v >> 5) & 0x7);
	shell_print(sh, "  [10:8] Max payload (sel) : %u (1=64 2=128 3=256)", (v >> 8) & 0x7);
	shell_print(sh, "  [11]   Flash share mode  : %u (0=ctrl-attached 1=tgt-attached)",
		    (v >> 11) & 0x1);
	shell_print(sh, "  [14:12]Max rd req (sel)  : %u (1=64 2=128 3=256 4=512 5=1024 6=2048 "
		    "7=4096)", (v >> 12) & 0x7);
	shell_print(sh, "  [17:16]Flash share cap   : %u (0=ctrl-only 1=tgt-only 2=ctrl-only "
		    "3=both)", (v >> 16) & 0x3);
	shell_print(sh, "  [23:20]RPMC counter (1st): %u (0-based)", (v >> 20) & 0xf);
	shell_print(sh, "  [31:24]RPMC OP1 opcode   : 0x%02x", (v >> 24) & 0xff);
	shell_print(sh, "");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(espi_cmds,
	SHELL_CMD(info, NULL, "Show eSPI device info", cmd_espi_info),
	SHELL_CMD(status, NULL, "Read ESPI030 (ESPI_DEV_STS)", cmd_espi_status),
	SHELL_CMD(ch_status, NULL, "Print eSPI channels status", cmd_espi_ch_status),
	SHELL_CMD(ch_cap, NULL, "Print eSPI channel capabilities", cmd_espi_ch_cap),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(espi, &espi_cmds, "eSPI shell commands", NULL);

int main(void)
{
	LOG_INF("ASPEED eSPI shell application");

	if (!device_is_ready(espi_dev)) {
		LOG_ERR("eSPI device not ready");
		return -ENODEV;
	}

#ifdef CONFIG_ESPI_OOB_CHANNEL_RX_ASYNC
	oob_async_init();
#endif

#ifdef CONFIG_ESPI_TAF
	safs_init();
#endif

	return 0;
}
