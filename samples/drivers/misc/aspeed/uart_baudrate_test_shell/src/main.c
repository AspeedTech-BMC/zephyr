/*
 * Copyright (c) 2026 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/shell/shell.h>

/* The divisor latch is 16 bits wide (DLL + DLH). */
#define MAX_DIVISOR 0xFFFFU
#define FIXED_BYTE 0x55U
#define DEFAULT_LEN 256U
/* Largest accepted gap between the requested and the real baud rate: 3%. */
#define MAX_RATE_ERR_PERMILLE 30U

enum pattern {
	PAT_INC,
	PAT_FIXED,
};

/* 16550 register offsets, for the debug snapshot taken on a timeout. */
#define REG_IIR 0x08
#define REG_LCR 0x0c
#define REG_LSR 0x14

struct uart_entry {
	const char *name;
	uintptr_t base;
	bool dma;
	bool shell;
	uint32_t clk_freq;
	const struct device *dev;
	const struct device *clk_dev;
	clock_control_subsys_t clk_id;
};

#define UART_ENTRY(n)                                                                              \
	{                                                                                          \
		.name = "uart" #n,                                                                 \
		.base = DT_REG_ADDR(DT_NODELABEL(uart##n)),                                        \
		.dma = DT_PROP_OR(DT_NODELABEL(uart##n), dma, 0),                                  \
		.shell = DT_SAME_NODE(DT_CHOSEN(zephyr_shell_uart), DT_NODELABEL(uart##n)),        \
		.clk_freq = DT_PROP_OR(DT_NODELABEL(uart##n), clock_frequency, 0),                 \
		.dev = DEVICE_DT_GET(DT_NODELABEL(uart##n)),                                       \
		.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_NODELABEL(uart##n))),                   \
		.clk_id = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_NODELABEL(uart##n), clk_id),   \
	}

/* The UART that carries the shell is in the table but rejected at run time. */
static const struct uart_entry uarts[] = {
	UART_ENTRY(0), UART_ENTRY(1), UART_ENTRY(2), UART_ENTRY(3), UART_ENTRY(4),  UART_ENTRY(5),
	UART_ENTRY(6), UART_ENTRY(7), UART_ENTRY(8), UART_ENTRY(9), UART_ENTRY(10), UART_ENTRY(11),
	UART_ENTRY(12),
};

struct dir_result {
	uint32_t received;
	uint32_t mismatches;
	uint32_t extra;
	uint32_t first_off;
	uint8_t first_exp;
	uint8_t first_act;
	bool timeout;
	/* Raw registers read at the moment of the timeout (debug). */
	uint32_t rx_lsr, rx_iir, rx_lcr, tx_lsr;
};

static const struct uart_entry *find_uart(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(uarts); i++) {
		if (strcmp(uarts[i].name, name) == 0) {
			return &uarts[i];
		}
	}
	return NULL;
}

static inline uint8_t expected_byte(enum pattern pat, uint32_t off)
{
	return (pat == PAT_FIXED) ? FIXED_BYTE : (uint8_t)off;
}

static void drain(const struct device *dev)
{
	unsigned char c;

	while (uart_poll_in(dev, &c) == 0) {
	}
}

/* Send len bytes from tx and compare them at rx. Single thread, interleaved,
 * so the RX FIFO never has more than a byte or two pending.
 */
static void run_direction(const struct uart_entry *txu, const struct uart_entry *rxu,
			  uint32_t eff_baud, enum pattern pat, uint32_t len,
			  struct dir_result *res)
{
	const struct device *tx = txu->dev;
	const struct device *rx = rxu->dev;
	const uint32_t frame_us = (10ULL * 1000000ULL) / eff_baud;
	const int64_t deadline =
		k_uptime_get() + 2 * (int64_t)((uint64_t)len * 10ULL * 1000ULL / eff_baud) + 100;
	uint32_t sent = 0;
	unsigned char c;

	memset(res, 0, sizeof(*res));

	while (res->received < len) {
		if (sent < len) {
			uart_poll_out(tx, expected_byte(pat, sent));
			sent++;
		} else if (k_uptime_get() > deadline) {
			res->timeout = true;
			res->rx_lsr = sys_read32(rxu->base + REG_LSR);
			res->rx_iir = sys_read32(rxu->base + REG_IIR);
			res->rx_lcr = sys_read32(rxu->base + REG_LCR);
			res->tx_lsr = sys_read32(txu->base + REG_LSR);
			break;
		}

		while (res->received < len && uart_poll_in(rx, &c) == 0) {
			uint8_t exp = expected_byte(pat, res->received);

			if (c != exp) {
				if (res->mismatches == 0) {
					res->first_off = res->received;
					res->first_exp = exp;
					res->first_act = c;
				}
				res->mismatches++;
			}
			res->received++;
		}
	}

	/* Anything arriving after the last expected byte is an error. */
	k_busy_wait(2 * frame_us);
	while (uart_poll_in(rx, &c) == 0) {
		res->extra++;
	}
}

static bool report(const struct shell *sh, const struct uart_entry *tx,
		   const struct uart_entry *rx, uint32_t baud, enum pattern pat, uint32_t len,
		   const struct dir_result *r)
{
	bool pass = !r->timeout && r->mismatches == 0 && r->extra == 0 && r->received == len;

	shell_print(sh, "%s -> %s baud=%u %s len=%u %s", tx->name, rx->name, baud,
		    pat == PAT_FIXED ? "fixed" : "inc", len, pass ? "PASS" : "FAIL");
	if (pass) {
		return true;
	}

	if (r->timeout) {
		shell_error(sh, "FAIL baud=%u timeout, received %u of %u bytes (%s -> %s)", baud,
			    r->received, len, tx->name, rx->name);
		shell_error(sh,
			    "debug: %s (rx) LSR=0x%02x IIR=0x%02x LCR=0x%02x, "
			    "%s (tx) LSR=0x%02x",
			    rx->name, r->rx_lsr, r->rx_iir, r->rx_lcr, tx->name, r->tx_lsr);
		if (r->received == 0) {
			shell_error(sh, "no data received, check the wiring");
		}
	}
	if (r->mismatches) {
		shell_error(sh, "FAIL baud=%u offset=%u expected=0x%02x actual=0x%02x (%s -> %s)",
			    baud, r->first_off, r->first_exp, r->first_act, tx->name, rx->name);
	}
	shell_error(sh, "mismatches=%u extra_bytes=%u received=%u/%u", r->mismatches, r->extra,
		    r->received, len);
	return false;
}

static int get_divisor(const struct uart_entry *u, uint32_t baud, uint32_t *clk, uint32_t *div)
{
	if (u->clk_freq) {
		/* e.g. uart12 (ns16550) takes its clock from "clock-frequency". */
		*clk = u->clk_freq;
	} else {
		int ret = clock_control_get_rate(u->clk_dev, u->clk_id, clk);

		if (ret) {
			return ret;
		}
	}
	/* 64-bit: 16 * baud wraps in 32 bits for baud >= 0x10000000. */
	*div = (uint32_t)(*clk / (16ULL * baud));
	return 0;
}

enum test_result {
	RES_PASS,
	RES_FAIL,
	/* The baud rate cannot be produced (divisor 0 or too large). */
	RES_SKIP,
	/* Setup error, not a test result. */
	RES_ERR,
};

/* Test one baud rate between a and b, both directions. */
static enum test_result test_baud(const struct shell *sh, const struct uart_entry *a,
				  const struct uart_entry *b, uint32_t baud, enum pattern pat,
				  uint32_t len, bool strict)
{
	uint32_t clk_a, clk_b, div_a, div_b, eff, err_permille;
	struct uart_config saved_a, saved_b, cfg;
	struct dir_result res;
	enum test_result result;
	bool pass = true;
	int ret;

	if (get_divisor(a, baud, &clk_a, &div_a) || get_divisor(b, baud, &clk_b, &div_b)) {
		shell_error(sh, "cannot read the UART clock rate");
		return RES_ERR;
	}
	if (div_a == 0 || div_a > MAX_DIVISOR || div_b == 0 || div_b > MAX_DIVISOR) {
		shell_error(sh,
			    "baud %u gives divisor %u (%s, clk %u) / %u (%s, clk %u); "
			    "the divisor must be 1..%u",
			    baud, div_a, a->name, clk_a, div_b, b->name, clk_b, MAX_DIVISOR);
		return RES_SKIP;
	}
	/* Both ends get the same baud; a different clock or divisor would make UART B run at
	 * another rate, and the failure would look like a wiring problem.
	 */
	if (clk_a != clk_b || div_a != div_b) {
		shell_error(sh,
			    "%s (clk %u, div %u) and %s (clk %u, div %u) do not run at the same "
			    "rate; put both on the same clock source",
			    a->name, clk_a, div_a, b->name, clk_b, div_b);
		return RES_ERR;
	}
	eff = clk_a / (16U * div_a);
	/* The divisor is an integer, so the real rate can differ from the request. Both ends
	 * agree on the divisor, so a large error would still PASS; flag it.
	 */
	err_permille =
		(uint32_t)(((uint64_t)(eff > baud ? eff - baud : baud - eff) * 1000ULL) / baud);
	if (err_permille > MAX_RATE_ERR_PERMILLE) {
		if (strict) {
			shell_print(sh, "SKIP baud %u: closest rate is %u (error %u.%u%%)", baud,
				    eff, err_permille / 10, err_permille % 10);
			return RES_SKIP;
		}
		shell_warn(sh, "warning: effective baud %u differs from %u by %u.%u%%", eff, baud,
			   err_permille / 10, err_permille % 10);
	}
	shell_print(sh,
		    "%s: clk=%u div=%u %s, %s: clk=%u div=%u %s, effective baud %u (requested %u)",
		    a->name, clk_a, div_a, a->dma ? "udma" : "fifo", b->name, clk_b, div_b,
		    b->dma ? "udma" : "fifo", eff, baud);
	shell_print(sh, "estimated time: about %u ms (both directions)",
		    (uint32_t)(2ULL * len * 10ULL * 1000ULL / eff));

	ret = uart_config_get(a->dev, &saved_a);
	if (ret == 0) {
		ret = uart_config_get(b->dev, &saved_b);
	}
	if (ret) {
		shell_error(sh, "cannot read the current UART config (%d)", ret);
		return RES_ERR;
	}

	/* Change only the baud rate; keep 8N1 and no flow control. */
	cfg = saved_a;
	cfg.baudrate = baud;
	cfg.data_bits = UART_CFG_DATA_BITS_8;
	cfg.parity = UART_CFG_PARITY_NONE;
	cfg.stop_bits = UART_CFG_STOP_BITS_1;
	cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	ret = uart_configure(a->dev, &cfg);
	if (ret == 0) {
		ret = uart_configure(b->dev, &cfg);
	}
	if (ret) {
		shell_error(sh, "uart_configure failed (%d)", ret);
		result = RES_ERR;
		goto restore;
	}

	/* Let the lines settle, then clear anything left over. */
	k_busy_wait(MAX(2000U, 10U * (10U * 1000000U / eff)));
	drain(a->dev);
	drain(b->dev);

	run_direction(a, b, eff, pat, len, &res);
	pass &= report(sh, a, b, baud, pat, len, &res);
	drain(a->dev);
	drain(b->dev);

	run_direction(b, a, eff, pat, len, &res);
	pass &= report(sh, b, a, baud, pat, len, &res);
	drain(a->dev);
	drain(b->dev);

	shell_print(sh, "%s <-> %s baud=%u: %s", a->name, b->name, baud, pass ? "PASS" : "FAIL");
	result = pass ? RES_PASS : RES_FAIL;

restore:
	if (uart_configure(a->dev, &saved_a) || uart_configure(b->dev, &saved_b)) {
		shell_warn(sh, "warning: cannot restore the UART config, %s/%s may stay at baud %u",
			   a->name, b->name, baud);
	}
	drain(a->dev);
	drain(b->dev);
	return result;
}

static const uint32_t auto_bauds[] = {1200, 9600, 19200, 38400, 57600, 115200};
static const uint32_t auto_hs_bauds[] = {230400, 460800, 921600, 1152000, 3686400};

/* Run every baud rate of a list and print a summary. */
static int run_auto(const struct shell *sh, const struct uart_entry *a,
		    const struct uart_entry *b, const uint32_t *bauds, size_t n,
		    enum pattern pat, uint32_t len)
{
	unsigned int n_pass = 0, n_fail = 0, n_skip = 0, n_err = 0;

	for (size_t i = 0; i < n; i++) {
		shell_print(sh, "--- %s <-> %s baud %u ---", a->name, b->name, bauds[i]);
		switch (test_baud(sh, a, b, bauds[i], pat, len, true)) {
		case RES_PASS:
			n_pass++;
			break;
		case RES_FAIL:
			n_fail++;
			break;
		case RES_SKIP:
			n_skip++;
			break;
		default:
			n_err++;
			break;
		}
	}
	shell_print(sh, "auto %s <-> %s: %u PASS, %u FAIL, %u SKIP, %u ERROR (of %u baud rates)",
		    a->name, b->name, n_pass, n_fail, n_skip, n_err, (unsigned int)n);

	return (n_fail || n_err) ? -EIO : 0;
}

static int cmd_uart_baud(const struct shell *sh, size_t argc, char **argv)
{
	const struct uart_entry *a = find_uart(argv[1]);
	const struct uart_entry *b = find_uart(argv[2]);
	enum pattern pat = PAT_INC;
	uint32_t len = DEFAULT_LEN;
	bool auto_hs = strcmp(argv[3], "auto_hs") == 0;
	bool auto_mode = auto_hs || strcmp(argv[3], "auto") == 0;
	unsigned long baud = 0;
	char *end;

	if (!a || !b) {
		shell_error(sh, "unknown UART \"%s\"; use uart0..uart12", a ? argv[2] : argv[1]);
		return -EINVAL;
	}
	if (a == b) {
		shell_error(sh, "UART A and UART B must differ");
		return -EINVAL;
	}
	if (a->shell || b->shell) {
		shell_error(sh, "%s carries the shell and cannot be tested",
			    a->shell ? a->name : b->name);
		return -EINVAL;
	}
	if (!device_is_ready(a->dev) || !device_is_ready(b->dev)) {
		shell_error(sh, "%s or %s not ready", a->name, b->name);
		return -ENODEV;
	}

	if (!auto_mode) {
		baud = strtoul(argv[3], &end, 0);
		if (*end != '\0' || baud == 0 || baud > UINT32_MAX) {
			shell_error(sh, "invalid baud \"%s\" (a number, \"auto\" or \"auto_hs\")",
				    argv[3]);
			return -EINVAL;
		}
	}

	if (argc > 4) {
		if (strcmp(argv[4], "inc") == 0) {
			pat = PAT_INC;
		} else if (strcmp(argv[4], "fixed") == 0) {
			pat = PAT_FIXED;
		} else {
			shell_error(sh, "pattern must be \"inc\" or \"fixed\"");
			return -EINVAL;
		}
	}
	if (argc > 5) {
		unsigned long l = strtoul(argv[5], &end, 0);

		if (*end != '\0' || l == 0 || l > CONFIG_UART_BAUDRATE_TEST_MAX_LEN) {
			shell_error(sh, "length must be 1..%d", CONFIG_UART_BAUDRATE_TEST_MAX_LEN);
			return -EINVAL;
		}
		len = l;
	}

	if (!auto_mode) {
		switch (test_baud(sh, a, b, baud, pat, len, false)) {
		case RES_PASS:
			return 0;
		case RES_FAIL:
			return -EIO;
		case RES_SKIP:
			return -EINVAL;
		default:
			return -EIO;
		}
	}

	if (auto_hs) {
		return run_auto(sh, a, b, auto_hs_bauds, ARRAY_SIZE(auto_hs_bauds), pat, len);
	}
	return run_auto(sh, a, b, auto_bauds, ARRAY_SIZE(auto_bauds), pat, len);
}

SHELL_CMD_ARG_REGISTER(uart_baud, NULL,
		       "Loopback test between two wired UARTs, both directions\n"
		       "uart_baud <uartA> <uartB> <baud|auto|auto_hs> [inc|fixed] [len]\n"
		       "  baud: a number; auto = 1200 9600 19200 38400 57600 115200;\n"
		       "        auto_hs = 230400 460800 921600 1152000 3686400\n"
		       "  uartA/uartB: uart0..uart12 except the shell UART (wire TX<->RX by hand)\n"
		       "  pattern: inc (default, byte = offset) or fixed (0x55)\n"
		       "  len: bytes per direction (default 256)",
		       cmd_uart_baud, 4, 2);

int main(void)
{
	printk("uart_baud ready, e.g. \"uart_baud uart0 uart1 115200 inc 256\"\n");
	return 0;
}
