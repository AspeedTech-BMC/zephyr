/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 ASPEED Technology Inc.
 */

#include <stdint.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/misc/aspeed/otp.h>

#define OTP_VER			"1.0.0"
#define OTP_DRV_NAME		DEVICE_DT_NAME(DT_INST(0, aspeed_ast27xx_otp))

#define shell_printf(_sh, _ft, ...) \
	shell_fprintf(_sh, SHELL_NORMAL, _ft, ##__VA_ARGS__)

static const struct device *otp_dev;

/*
 * AST1040 OTP layout (unit: DW, 32-bit words). Region boundaries come from
 * the AST1040 OTP map, not from the AST2700 tables in otp_info_ast27xx.h.
 */
struct otp_region {
	const char *name;
	uint32_t start;
	uint32_t size;
};

static const struct otp_region otp_regions[] = {
	{ "rom",    0x000, 0x900 },
	{ "rbp",    0x900, 0x010 },
	{ "conf",   0x910, 0x040 },
	{ "u-data", 0x950, 0x600 },
	{ "s-data", 0xf50, 0x090 },
	{ "strap",  0xfe0, 0x010 },
	{ "ctrl",   0xff0, 0x010 },
};

#define OTP_DUMP_CHUNK_DW	16

static int otp_dev_init(const struct shell *shell)
{
	if (!otp_dev)
		otp_dev = device_get_binding(OTP_DRV_NAME);

	if (!otp_dev || !device_is_ready(otp_dev)) {
		shell_printf(shell, "OTP device %s not ready\n", OTP_DRV_NAME);
		return -ENODEV;
	}

	return 0;
}

static const struct otp_region *otp_region_find(const char *name)
{
	for (int i = 0; i < ARRAY_SIZE(otp_regions); i++) {
		if (!strcmp(name, otp_regions[i].name))
			return &otp_regions[i];
	}

	return NULL;
}

static int do_otp_version(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t chip_ver;
	int ret;

	shell_printf(shell, "OTP tool version: %s\n", OTP_VER);

	ret = otp_dev_init(shell);
	if (ret)
		return ret;

	ret = otp_get_chip_version(otp_dev, &chip_ver);
	if (ret) {
		shell_printf(shell, "Failed to get chip version: %d\n", ret);
		return ret;
	}

	if (chip_ver == (uint32_t)-1)
		shell_printf(shell, "Chip version: unknown\n");
	else
		shell_printf(shell, "Chip version: 0x%x\n", chip_ver);

	return 0;
}

static int do_otp_read(const struct shell *shell, size_t argc, char **argv)
{
	const struct otp_region *region;
	uint32_t offset, count;
	uint32_t data;
	int ret;

	region = otp_region_find(argv[1]);
	if (!region) {
		shell_printf(shell, "Unknown region: %s\n", argv[1]);
		return -EINVAL;
	}

	offset = (argc > 2) ? strtoul(argv[2], NULL, 16) : 0;
	count = (argc > 3) ? strtoul(argv[3], NULL, 16) : 1;

	if (offset + count > region->size) {
		shell_printf(shell, "%s is 0x%x DW, offset/count out of range\n",
			     region->name, region->size);
		return -EINVAL;
	}

	ret = otp_dev_init(shell);
	if (ret)
		return ret;

	for (uint32_t i = 0; i < count; i++) {
		ret = otp_read_multi(otp_dev, region->start + offset + i, &data, 1);
		if (ret) {
			shell_printf(shell, "Read failed at %s[0x%03x]: %d\n",
				     region->name, offset + i, ret);
			return ret;
		}

		shell_printf(shell, "%s[0x%03x]: 0x%08x\n", region->name, offset + i, data);
	}

	return 0;
}

/*
 * Pulls a "-o" (no-confirm) flag out of argv, wherever it appears, and
 * copies the remaining arguments into out_argv (relative order preserved).
 * Remaining arguments still have to be given positionally.
 */
static bool otp_parse_o_flag(size_t argc, char **argv, char **out_argv, size_t *out_argc)
{
	bool nconfirm = false;
	size_t n = 0;

	for (size_t i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "-o"))
			nconfirm = true;
		else
			out_argv[n++] = argv[i];
	}

	*out_argc = n;
	return nconfirm;
}

static bool otp_confirm_yesno(const struct shell *shell)
{
	size_t cnt;
	char c;

	while (true) {
		shell->iface->api->read(shell->iface, &c, sizeof(c), &cnt);
		if (cnt == 0) {
			k_busy_wait(100);
			continue;
		}

		shell->iface->api->write(shell->iface, &c, sizeof(c), &cnt);

		if (c == 'y' || c == 'Y')
			return true;
		if (c == 'n' || c == 'N' || c == '\r' || c == '\n')
			return false;
	}
}

static int do_otp_pb(const struct shell *shell, size_t argc, char **argv)
{
	const struct otp_region *region;
	uint32_t offset, bit, value, mask, data;
	char *args[5];
	size_t nargs;
	bool nconfirm;
	int ret;

	nconfirm = otp_parse_o_flag(argc - 1, &argv[1], args, &nargs);
	if (nargs != 4) {
		shell_printf(shell, "Usage: otp pb <region> <otp_dw_offset> <bit_offset> "
				    "<value> [-o]\n");
		return -EINVAL;
	}

	region = otp_region_find(args[0]);
	if (!region) {
		shell_printf(shell, "Unknown region: %s\n", args[0]);
		return -EINVAL;
	}

	offset = strtoul(args[1], NULL, 16);
	bit = strtoul(args[2], NULL, 16);
	value = strtoul(args[3], NULL, 16);

	if (offset >= region->size) {
		shell_printf(shell, "%s is 0x%x DW, offset out of range\n",
			     region->name, region->size);
		return -EINVAL;
	}

	if (bit >= 32) {
		shell_printf(shell, "bit_offset must be 0~31\n");
		return -EINVAL;
	}

	if (value > (UINT32_MAX >> bit)) {
		shell_printf(shell, "value 0x%x overflows past bit 31 at bit_offset %u\n",
			     value, bit);
		return -EINVAL;
	}

	/* result = current | (value << bit_offset), e.g. bit=1 value=3 sets bits 1,2 (0b110) */
	mask = value << bit;

	ret = otp_dev_init(shell);
	if (ret)
		return ret;

	ret = otp_read_multi(otp_dev, region->start + offset, &data, 1);
	if (ret) {
		shell_printf(shell, "Read failed at %s[0x%03x]: %d\n",
			     region->name, offset, ret);
		return ret;
	}

	if ((data & mask) == mask) {
		shell_printf(shell, "%s[0x%03x] bits 0x%x are already set\n",
			     region->name, offset, mask);
		return 0;
	}

	if (!nconfirm) {
		shell_printf(shell, "%s[0x%03x]: 0x%08x -> 0x%08x. Confirm? (y/n) ",
			     region->name, offset, data, data | mask);
		if (!otp_confirm_yesno(shell)) {
			shell_printf(shell, "\nAborted\n");
			return 0;
		}
		shell_printf(shell, "\n");
	}

	data |= mask;

	ret = otp_program_multi(otp_dev, region->start + offset, &data, 1);
	if (ret) {
		shell_printf(shell, "Program failed at %s[0x%03x]: %d\n",
			     region->name, offset, ret);
		return ret;
	}

	shell_printf(shell, "%s[0x%03x] |= 0x%x programmed\n", region->name, offset, mask);

	return 0;
}

static int do_otp_dump(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t data[OTP_DUMP_CHUNK_DW];
	const struct otp_region *filter = NULL;
	int ret;

	if (argc > 1) {
		filter = otp_region_find(argv[1]);
		if (!filter) {
			shell_printf(shell, "Unknown region: %s\n", argv[1]);
			return -EINVAL;
		}
	}

	ret = otp_dev_init(shell);
	if (ret)
		return ret;

	for (int r = 0; r < ARRAY_SIZE(otp_regions); r++) {
		const struct otp_region *region = &otp_regions[r];

		if (filter && region != filter)
			continue;

		shell_printf(shell, "\n%s: 0x%03x ~ 0x%03x\n", region->name,
			     region->start, region->start + region->size - 1);
		shell_printf(shell, "%-7s %-9s %s\n", "dw", "byte", "data");

		for (uint32_t off = 0; off < region->size; off += OTP_DUMP_CHUNK_DW) {
			uint32_t chunk = MIN(OTP_DUMP_CHUNK_DW, region->size - off);
			uint32_t addr = region->start + off;

			ret = otp_read_multi(otp_dev, addr, data, chunk);
			if (ret) {
				shell_printf(shell, "Read failed at 0x%03x: %d\n", addr, ret);
				return ret;
			}

			for (uint32_t i = 0; i < chunk; i++) {
				if (i % 4 == 0)
					shell_printf(shell, "0x%-5x 0x%-7x", addr + i,
						     (addr + i) * 4);
				shell_printf(shell, " %08x", data[i]);
				if (i % 4 == 3 || i == chunk - 1)
					shell_printf(shell, "\n");
			}
		}
	}

	return 0;
}

#define SHELL_HELP_OTP_READ \
	"Read OTP memory\n" \
	"- otp read rom|rbp|conf|u-data|s-data|strap|ctrl [otp_dw_offset] [dw_count]"

#define SHELL_HELP_OTP_PB \
	"Program OTP bits: result = current | (value << bit_offset)\n" \
	"Without -o, asks for a y/n confirmation before programming\n" \
	"-o may appear anywhere in the argument list; the rest stay positional\n" \
	"- otp pb rom|rbp|conf|u-data|s-data|strap|ctrl <otp_dw_offset> <bit_offset> <value> [-o]"

#define SHELL_HELP_OTP_DUMP \
	"Dump the whole OTP array, or a single region\n" \
	"- otp dump [rom|rbp|conf|u-data|s-data|strap|ctrl]"

SHELL_STATIC_SUBCMD_SET_CREATE(otp_cmds,
		       SHELL_CMD_ARG(version, NULL, "otp version", do_otp_version, 1, 0),
		       SHELL_CMD_ARG(read, NULL, SHELL_HELP_OTP_READ, do_otp_read, 2, 2),
		       SHELL_CMD_ARG(pb, NULL, SHELL_HELP_OTP_PB, do_otp_pb, 5, 1),
		       SHELL_CMD_ARG(dump, NULL, SHELL_HELP_OTP_DUMP, do_otp_dump, 1, 1),
		       SHELL_SUBCMD_SET_END);

SHELL_CMD_ARG_REGISTER(otp, &otp_cmds,
		       "ASPEED AST1040 One-Time-Programmable sub-system\n"
		       "otp version\n"
		       "otp read rom|rbp|conf|u-data|s-data|strap|ctrl "
		       "[otp_dw_offset] [dw_count]\n"
		       "otp pb rom|rbp|conf|u-data|s-data|strap|ctrl "
		       "<otp_dw_offset> <bit_offset> <value> [-o]\n"
		       "otp dump [rom|rbp|conf|u-data|s-data|strap|ctrl]\n",
		       NULL, 2, 2);
