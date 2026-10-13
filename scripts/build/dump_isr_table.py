#!/usr/bin/env python3
#
# Copyright (c) 2026 ASPEED Technology Inc.
#
# SPDX-License-Identifier: Apache-2.0
"""Dump every sw_isr_table slot together with its ISR handler's symbol name.

IRQ_CONNECT() never registers anything at runtime: on ARM it expands to
Z_ISR_DECLARE(), which only drops a {irq, flags, func, param} record into the
.intList section. gen_isr_tables.py is what turns that into the real
_sw_isr_table[] contents, at link time, by decoding each entry's multi-level
irq number into a table index. Its own --debug output (west build -v) shows
that decode, but only ever prints the raw ISR function pointer, never its
name, because .intList stores a bare address with no symbol information
attached.

This tool re-parses the same .intList section with the same gen_isr_tables.py
code (so the table index it reports can never disagree with what the actual
build produced) and resolves each entry's function pointer back to a symbol
name using the ELF symbol table.

Only zephyr_pre0.elf (the pre-link image) has an .intList section at all:
the final zephyr.elf discards it (see include/zephyr/linker/intlist.ld's
LINKER_ZEPHYR_FINAL branch), so there is no fallback to it.

Limitations:
- Only the "carrays" .intList layout is supported, i.e. a build with
  CONFIG_ISR_TABLES_LOCAL_DECLARATION unset. A build with that option set
  is rejected rather than silently misparsed.
- Direct interrupts (ISR_FLAG_DIRECT, e.g. via IRQ_DIRECT_CONNECT()) are
  skipped: they are placed in the vector table, never in _sw_isr_table, so
  they have no sw_isr_table index to report here.

Usage:
    scripts/build/dump_isr_table.py -d build
    scripts/build/dump_isr_table.py -k build/zephyr/zephyr_pre0.elf
    scripts/build/dump_isr_table.py -d build --all
"""
import argparse
import os
import struct
import sys

from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_isr_tables as git  # noqa: E402

DEFAULT_INTLIST_SECTIONS = [".intList", "intList"]

# Symbol types/section kinds worth resolving an address back to. This
# deliberately excludes gen_isr_tables.py's own absolute CONFIG_* symbols
# (STT_OBJECT with st_shndx == SHN_ABS) and non-data/code symbols (sections,
# files, ARM mapping symbols like $t/$d), which would otherwise alias a NULL
# or small-integer arg/param to an unrelated name.
_RESOLVABLE_TYPES = ("STT_FUNC", "STT_OBJECT")
_UNRESOLVABLE_SHNDX = ("SHN_ABS", "SHN_UNDEF", "SHN_COMMON")


def find_kernel_elf(build_dir):
    path = os.path.join(build_dir, "zephyr", "zephyr_pre0.elf")
    if os.path.exists(path):
        return path
    git.log.error(
        "no zephyr_pre0.elf found under {}/zephyr (only the pre-link image "
        "has an .intList section; the final zephyr.elf discards it)".format(build_dir))


def build_addr_to_name(kernel):
    """Reverse address->symbol-name map, restricted to real code/data symbols.

    Unlike gen_isr_tables.get_symbols()/get_sym_from_addr(), which index
    every symbol (including absolute CONFIG_* values gen_isr_tables.py itself
    relies on), this only keeps STT_FUNC/STT_OBJECT symbols that live in a
    real section, so a NULL or small-integer arg/param can't alias one of
    those by coincidence.
    """
    addr_to_name = {}
    for section in kernel.iter_sections():
        if not isinstance(section, SymbolTableSection):
            continue
        for sym in section.iter_symbols():
            if not sym.name:
                continue
            if sym["st_info"]["type"] not in _RESOLVABLE_TYPES:
                continue
            if sym["st_shndx"] in _UNRESOLVABLE_SHNDX:
                continue
            addr_to_name.setdefault(sym["st_value"], sym.name)
    return addr_to_name


def resolve(addr_to_name, addr):
    """Return (display string, whether it's a resolved name vs. a raw address/NULL)."""
    if addr == 0:
        return "NULL", False
    name = addr_to_name.get(addr)
    if name is not None:
        return name, True
    return "0x{:x}".format(addr), False


def decode_level(config, irq):
    """Mirror gen_isr_config.get_swt_table_index()'s own level decode.

    That method only logs the level/local-irq it computes internally, it
    never returns them and doesn't expose the pieces (e.g. max-irq-per-
    aggregator) needed to recompute its result independently, so this
    duplicates its bit arithmetic rather than sharing it. If
    get_swt_table_index() is ever changed, keep this in sync with it.
    """
    if not config.check_multi_level_interrupts():
        return 1, irq

    bits = config.int_bits
    masks = config.int_lvl_masks
    irq3 = (irq & masks[2]) >> (bits[0] + bits[1])
    irq2 = (irq & masks[1]) >> bits[0]
    irq1 = irq & masks[0]

    if irq3:
        return 3, irq3 - 1
    if irq2:
        return 2, irq2 - 1
    return 1, irq1


def read_intlist_entries(config, intlist_data):
    prefix = config.endian_prefix()
    header_fmt = prefix + "II"
    header_sz = struct.calcsize(header_fmt)
    num_vectors, offset = struct.unpack_from(header_fmt, intlist_data, 0)

    if num_vectors > pow(2, 15):
        git.log.error(
            "num_vectors ({}) is implausibly large, check -e/--big-endian".format(num_vectors))

    entry_fmt = prefix + ("iiQQ" if config.check_64b() else "iiII")
    entries = list(struct.iter_unpack(entry_fmt, intlist_data[header_sz:]))

    return offset, entries


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("-k", "--kernel", help="path to zephyr_pre0.elf")
    group.add_argument("-d", "--build-dir",
                        help="build directory; auto-finds zephyr/zephyr_pre0.elf")

    parser.add_argument("-e", "--big-endian", action="store_true",
                         help="target encodes data in big-endian format")
    parser.add_argument("-i", "--intlist-section", action="append",
                         help="section name to search for .intList data "
                              "(repeatable; default: {})".format(DEFAULT_INTLIST_SECTIONS))
    parser.add_argument("--sw-isr-table", action="store_true", default=True,
                         help=argparse.SUPPRESS)
    parser.add_argument("--all", action="store_true",
                         help="also print entries whose handler couldn't be resolved to a "
                              "symbol name (shown as a raw address instead); default: skip them")

    args = parser.parse_args()
    if args.intlist_section is None:
        args.intlist_section = list(DEFAULT_INTLIST_SECTIONS)
    return args


def main():
    args = parse_args()
    kernel_path = args.kernel or find_kernel_elf(args.build_dir)

    if os.path.basename(kernel_path) != "zephyr_pre0.elf":
        print(
            "warning: {} is not zephyr_pre0.elf; the final zephyr.elf has no "
            ".intList section, so this will fail unless it's some other "
            "pre-link image".format(kernel_path), file=sys.stderr)

    with open(kernel_path, "rb") as f:
        kernel = ELFFile(f)
        syms = git.get_symbols(kernel)
        config = git.gen_isr_config(args, syms, git.log)

        if config.check_sym("CONFIG_ISR_TABLES_LOCAL_DECLARATION"):
            git.log.error(
                "CONFIG_ISR_TABLES_LOCAL_DECLARATION builds use a different "
                ".intList record layout (see gen_isr_tables_parser_local.py), "
                "which this tool does not parse")

        intlist_data = git.read_intList_sect(kernel, config.get_intlist_snames())
        addr_to_name = build_addr_to_name(kernel)

    offset, entries = read_intlist_entries(config, intlist_data)

    rows = []
    for irq, flags, func, param in entries:
        if config.test_isr_direct(flags):
            # Direct ISRs live in the vector table, not _sw_isr_table.
            continue
        index = config.get_swt_table_index(offset, irq)
        level, local_irq = decode_level(config, irq)
        rows.append((index, irq, level, local_irq, func, param))

    rows.sort(key=lambda r: r[0])

    print("{:<6} {:<10} {:<6} {:<10} {:<30} {}".format(
        "index", "irq", "level", "local", "handler", "arg"))
    print("-" * 90)
    for index, irq, level, local_irq, func, param in rows:
        handler_str, handler_resolved = resolve(addr_to_name, func)
        if not handler_resolved and not args.all:
            continue
        arg_str, _ = resolve(addr_to_name, param)
        print("{:<6} 0x{:<8x} {:<6} {:<10} {:<30} {}".format(
            index, irq, level, local_irq, handler_str, arg_str))


if __name__ == "__main__":
    main()
