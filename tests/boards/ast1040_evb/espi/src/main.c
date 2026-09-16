/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_aspeed.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>
#include "ast_test.h"

/*
 * Offsets from the eSPI controller's own base register, mirroring the
 * ESPI_CAP_CHn defines in drivers/espi/espi_aspeed_v2.c - not exposed via
 * the Zephyr eSPI API, so read directly like the uart test pokes MCR.
 */
#define ESPI_CAP_CH0	0x018
#define ESPI_CAP_CH1	0x01c
#define ESPI_CAP_CH2	0x020
#define ESPI_CAP_CH_EN	BIT(0)

static const struct device *const espi_dev = DEVICE_DT_GET(DT_NODELABEL(espi0));
static const uint32_t espi_base = DT_REG_ADDR(DT_NODELABEL(espi0));

/* The eSPI host may take a while to bring the link up after power-on. */
#define ESPI_LINK_TIMEOUT   K_MINUTES(1)
#define ESPI_LINK_POLL_SLEEP K_MSEC(100)

static const enum espi_channel espi_channels[] = {
	ESPI_CHANNEL_PERIPHERAL,
	ESPI_CHANNEL_VWIRE,
	ESPI_CHANNEL_OOB,
	ESPI_CHANNEL_FLASH,
};

static const uint32_t espi_cap_ch_offsets[] = {
	ESPI_CAP_CH0,
	ESPI_CAP_CH1,
	ESPI_CAP_CH2,
};

static bool espi_cap_ch_enabled(uint32_t offset)
{
	return sys_read32(espi_base + offset) & ESPI_CAP_CH_EN;
}

static bool espi_all_channels_ready(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(espi_channels); i++) {
		if (!espi_get_channel_status(espi_dev, espi_channels[i])) {
			return false;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(espi_cap_ch_offsets); i++) {
		if (!espi_cap_ch_enabled(espi_cap_ch_offsets[i])) {
			return false;
		}
	}

	return true;
}

int test_espi_link(void)
{
	if (!ast_zassert_true(device_is_ready(espi_dev), "eSPI device is not ready")) {
		return ast_ztest_result();
	}

	k_timepoint_t deadline = sys_timepoint_calc(ESPI_LINK_TIMEOUT);

	while (!espi_all_channels_ready() && !sys_timepoint_expired(deadline)) {
		k_sleep(ESPI_LINK_POLL_SLEEP);
	}

	ast_zassert_true(espi_get_channel_status(espi_dev, ESPI_CHANNEL_PERIPHERAL),
			 "eSPI peripheral channel not ready within timeout");
	ast_zassert_true(espi_get_channel_status(espi_dev, ESPI_CHANNEL_VWIRE),
			 "eSPI virtual wire channel not ready within timeout");
	ast_zassert_true(espi_get_channel_status(espi_dev, ESPI_CHANNEL_OOB),
			 "eSPI OOB channel not ready within timeout");
	ast_zassert_true(espi_get_channel_status(espi_dev, ESPI_CHANNEL_FLASH),
			 "eSPI flash channel not ready within timeout");

	ast_zassert_true(espi_cap_ch_enabled(ESPI_CAP_CH0),
			 "eSPI CAP_CH0 bit0 not set within timeout");
	ast_zassert_true(espi_cap_ch_enabled(ESPI_CAP_CH1),
			 "eSPI CAP_CH1 bit0 not set within timeout");
	ast_zassert_true(espi_cap_ch_enabled(ESPI_CAP_CH2),
			 "eSPI CAP_CH2 bit0 not set within timeout");

	return ast_ztest_result();
}

int test_espi(void)
{
	ast_zassert_true(device_is_ready(espi_dev), "eSPI device is not ready");

	return ast_ztest_result();
}

/*
 * Services the host's memory cycle accesses into the peripheral channel's
 * mcyc window (see the perif-mcyc-* properties on &espi0): posted writes are
 * stashed into perif_shadow_buf, and non-posted reads are completed from
 * whatever was last written at that offset - a plain read/write loopback.
 *
 * The mcyc window (perif-mcyc-size) can be much larger than what ast1040's
 * SRAM can spare, so perif_shadow_buf only backs the first 4K of it; an
 * access past that wraps back to the start of the buffer.
 */
#define PERIF_SHADOW_BUF_SIZE 4096u

BUILD_ASSERT(DT_PROP_BY_IDX(DT_NODELABEL(espi0), perif_mcyc_size, 0) == 0,
	     "perif mcyc window larger than 4G is not supported by this test");
BUILD_ASSERT(DT_PROP_BY_IDX(DT_NODELABEL(espi0), perif_mcyc_size, 1) >= PERIF_SHADOW_BUF_SIZE,
	     "perif mcyc window is smaller than perif_shadow_buf");
BUILD_ASSERT(IS_POWER_OF_TWO(PERIF_SHADOW_BUF_SIZE),
	     "perif_shadow_buf size must be a power of two");

static uint8_t perif_shadow_buf[PERIF_SHADOW_BUF_SIZE];

static K_THREAD_STACK_DEFINE(perif_thread_stack, 2048);
static struct k_thread perif_thread_data;

static void perif_service_pc_rx(void)
{
	/*
	 * static: ESPI_PLD_LEN_MAX-sized buffers don't fit perif_thread_stack.
	 * mem64 needs comm_hdr(3) + addr64(8) = 11 bytes before the payload,
	 * but struct espi_perif_mem64's addr_be is only a uint32_t (7-byte
	 * struct) - pad by sizeof(uint32_t) beyond it, same as
	 * perif_service_np_rx(), or a full-size MEMWR64 payload overruns
	 * this buffer by 4 bytes.
	 */
	static uint8_t rx_buf[sizeof(struct espi_perif_mem64) + sizeof(uint32_t) +
			       ESPI_PLD_LEN_MAX];
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)rx_buf;
	struct espi_aspeed_ioc ioc = { .pkt = rx_buf, .pkt_len = sizeof(rx_buf) };
	uint8_t *data;
	uint32_t offset, len;

	if (espi_aspeed_perif_pc_get_rx(espi_dev, &ioc, false) != 0) {
		return;
	}

	if (hdr->cyc == ESPI_PERIF_MEMWR32) {
		offset = sys_be32_to_cpu(*(uint32_t *)(hdr + 1)) & (PERIF_SHADOW_BUF_SIZE - 1);
		data = rx_buf + sizeof(*hdr) + sizeof(uint32_t);
		len = ioc.pkt_len - sizeof(*hdr) - sizeof(uint32_t);
	} else if (hdr->cyc == ESPI_PERIF_MEMWR64) {
		offset = (uint32_t)sys_be64_to_cpu(*(uint64_t *)(hdr + 1)) &
			 (PERIF_SHADOW_BUF_SIZE - 1);
		data = rx_buf + sizeof(*hdr) + sizeof(uint64_t);
		len = ioc.pkt_len - sizeof(*hdr) - sizeof(uint64_t);
	} else {
		return;
	}

	len = MIN(len, PERIF_SHADOW_BUF_SIZE - offset);
	memcpy(&perif_shadow_buf[offset], data, len);
}

static void perif_service_np_rx(void)
{
	/* mem64 needs comm_hdr(3) + addr64(8) = 11 bytes; pad beyond struct size */
	static uint8_t rx_buf[sizeof(struct espi_perif_mem64) + sizeof(uint32_t)];
	/* static: ESPI_PLD_LEN_MAX-sized buffers don't fit perif_thread_stack */
	static uint8_t tx_buf[sizeof(struct espi_perif_cmplt) + ESPI_PLD_LEN_MAX];
	struct espi_comm_hdr *rx_hdr = (struct espi_comm_hdr *)rx_buf;
	struct espi_perif_cmplt *cmplt = (struct espi_perif_cmplt *)tx_buf;
	struct espi_aspeed_ioc rx_ioc = { .pkt = rx_buf, .pkt_len = sizeof(rx_buf) };
	struct espi_aspeed_ioc tx_ioc = { .pkt = tx_buf };
	uint32_t offset, len;

	if (espi_aspeed_perif_np_get_rx(espi_dev, &rx_ioc, false) != 0) {
		return;
	}

	if (rx_hdr->cyc == ESPI_PERIF_MEMRD32) {
		struct espi_perif_mem32 *mem32 = (struct espi_perif_mem32 *)rx_buf;

		offset = sys_be32_to_cpu(mem32->addr_be) & (PERIF_SHADOW_BUF_SIZE - 1);
	} else if (rx_hdr->cyc == ESPI_PERIF_MEMRD64) {
		offset = (uint32_t)sys_be64_to_cpu(*(uint64_t *)(rx_hdr + 1)) &
			 (PERIF_SHADOW_BUF_SIZE - 1);
	} else {
		return;
	}

	len = ((rx_hdr->len_h << 8) | rx_hdr->len_l);
	len = len ? len : ESPI_PLD_LEN_MAX;
	len = MIN(len, MIN(PERIF_SHADOW_BUF_SIZE - offset, ESPI_PLD_LEN_MAX));

	cmplt->cyc = ESPI_PERIF_SUC_CMPLT_D_ONLY;
	cmplt->tag = rx_hdr->tag;
	cmplt->len_h = len >> 8;
	cmplt->len_l = len & 0xff;
	memcpy(cmplt->data, &perif_shadow_buf[offset], len);
	tx_ioc.pkt_len = sizeof(*cmplt) + len;

	(void)espi_aspeed_perif_pc_put_tx(espi_dev, &tx_ioc);
}

static void perif_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		perif_service_pc_rx();
		perif_service_np_rx();
		k_usleep(100);
	}
}

/*
 * Services the host's OOB channel messages: whatever the host sends is
 * echoed straight back, cyc/tag/len untouched - a plain message loopback.
 *
 * This can't go through the standard espi_send_oob()/espi_receive_oob()
 * API: espi_ast2700_send_oob() (drivers/espi/espi_aspeed_v2.c) hardcodes
 * tag=0 on every send, which breaks a tester that tags its OOB messages and
 * expects the same tag echoed back (e.g. SAFS_AUTO's oob loopback test).
 * Call the driver's own get_rx/put_tx instead - non-static, but not
 * declared in any public header, so declared extern here - and forward the
 * received espi_comm_hdr + payload unchanged.
 */
extern int espi_ast2700_oob_get_rx(const struct device *dev, struct espi_aspeed_ioc *ioc,
				   bool blocking);
extern int espi_ast2700_oob_put_tx(const struct device *dev, struct espi_aspeed_ioc *ioc);

static K_THREAD_STACK_DEFINE(oob_thread_stack, 2048);
static struct k_thread oob_thread_data;

static void oob_service_rx(void)
{
	/* static: ESPI_PLD_LEN_MAX-sized buffer doesn't fit oob_thread_stack */
	static uint8_t buf[sizeof(struct espi_comm_hdr) + ESPI_PLD_LEN_MAX];
	struct espi_aspeed_ioc ioc = { .pkt = buf, .pkt_len = sizeof(buf) };

	if (espi_ast2700_oob_get_rx(espi_dev, &ioc, false) != 0) {
		return;
	}

	(void)espi_ast2700_oob_put_tx(espi_dev, &ioc);
}

static void oob_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		oob_service_rx();
		k_usleep(100);
	}
}

/*
 * Services the host's SAFS (flash channel, eDAF software mode) requests via
 * the TAF handler callback registered through espi_aspeed_taf_register() -
 * matching samples/drivers/espi_aspeed/v2_shell/src/safs_handler.c - rather
 * than polling espi_aspeed_flash_get_rx()/put_tx() directly. Reads and
 * writes are loopback'd through safs_shadow_buf, and erase fills the
 * requested region with 0xff, like a real flash. Same 4K SRAM cap as
 * perif_shadow_buf, for the same reason, so an access past that wraps back
 * to the start of the buffer instead of being rejected as out-of-range.
 */
#define SAFS_SHADOW_BUF_SIZE 4096u

BUILD_ASSERT(IS_POWER_OF_TWO(SAFS_SHADOW_BUF_SIZE), "safs_shadow_buf size must be a power of two");

static uint8_t safs_shadow_buf[SAFS_SHADOW_BUF_SIZE];
static uint8_t safs_taf_tx_buf[sizeof(struct espi_flash_cmplt) + ESPI_PLD_LEN_MAX];

static uint16_t safs_taf_payload_len(uint16_t len)
{
	return len ? len : ESPI_PLD_LEN_MAX;
}

static bool safs_taf_erase_size(uint16_t type, uint32_t *erase_size)
{
	switch (type) {
	case 0:
		*erase_size = 4096;
		return true;
	case 1:
		*erase_size = 32768;
		return true;
	case 2:
		*erase_size = 65536;
		return true;
	case 3:
		*erase_size = 131072;
		return true;
	default:
		return false;
	}
}

static void safs_send_cmplt(const struct device *dev, uint8_t cyc, uint8_t tag)
{
	struct espi_flash_cmplt cmplt = { .cyc = cyc, .tag = tag };
	struct espi_aspeed_ioc ioc = { .pkt = (uint8_t *)&cmplt, .pkt_len = sizeof(cmplt) };

	(void)espi_aspeed_flash_put_tx(dev, &ioc);
}

static void safs_taf_handler(const struct device *dev, struct espi_taf_req *req, void *user_data)
{
	uint32_t offset = req->addr & (SAFS_SHADOW_BUF_SIZE - 1);

	ARG_UNUSED(user_data);

	switch (req->cyc) {
	case ESPI_FLASH_READ: {
		struct espi_flash_cmplt *cmplt = (struct espi_flash_cmplt *)safs_taf_tx_buf;
		struct espi_aspeed_ioc ioc = { .pkt = safs_taf_tx_buf };
		uint16_t len = safs_taf_payload_len(req->len);

		len = MIN(len, MIN(SAFS_SHADOW_BUF_SIZE - offset, ESPI_PLD_LEN_MAX));

		cmplt->cyc = ESPI_FLASH_SUC_CMPLT_D_ONLY;
		cmplt->tag = req->tag;
		cmplt->len_h = len >> 8;
		cmplt->len_l = len & 0xff;
		memcpy(cmplt->data, &safs_shadow_buf[offset], len);
		ioc.pkt_len = sizeof(*cmplt) + len;
		(void)espi_aspeed_flash_put_tx(dev, &ioc);
		break;
	}
	case ESPI_FLASH_WRITE: {
		uint16_t len = safs_taf_payload_len(req->len);

		if (!req->data) {
			safs_send_cmplt(dev, ESPI_FLASH_UNSUC_CMPLT, req->tag);
			break;
		}

		len = MIN(len, SAFS_SHADOW_BUF_SIZE - offset);
		memcpy(&safs_shadow_buf[offset], req->data, len);
		safs_send_cmplt(dev, ESPI_FLASH_SUC_CMPLT, req->tag);
		break;
	}
	case ESPI_FLASH_ERASE: {
		uint32_t erase_size;

		if (!safs_taf_erase_size(req->len, &erase_size)) {
			safs_send_cmplt(dev, ESPI_FLASH_UNSUC_CMPLT, req->tag);
			break;
		}

		erase_size = MIN(erase_size, SAFS_SHADOW_BUF_SIZE - offset);
		memset(&safs_shadow_buf[offset], 0xff, erase_size);
		safs_send_cmplt(dev, ESPI_FLASH_SUC_CMPLT, req->tag);
		break;
	}
	default:
		safs_send_cmplt(dev, ESPI_FLASH_UNSUC_CMPLT, req->tag);
		break;
	}
}

/*
 * Reads tester control commands off vuart0, one byte at a time via
 * uart_poll_in(), and assembles them into '\n'/'\r'-terminated lines. What
 * to do with a received line (dispatch, ack, ...) isn't wired up yet.
 */
static const struct device *const vuart_dev = DEVICE_DT_GET(DT_NODELABEL(vuart0));

#define VUART_LINE_BUF_SIZE 128

static K_THREAD_STACK_DEFINE(vuart_thread_stack, 1024);
static struct k_thread vuart_thread_data;

/*
 * One queue per channel, rather than a single shared one: which testcase
 * actually runs when is up to ztest's own (alphabetical, not source-order)
 * test ordering, so e.g. "perif start" can arrive well before
 * test_espi_perif_mcyc gets to run. A shared queue would have that message
 * sitting at the head and silently discarded by whichever unrelated
 * testcase happens to be waiting at the time; a queue per channel keeps it
 * until the right testcase asks for it.
 */
K_MSGQ_DEFINE(perif_cmdq, VUART_LINE_BUF_SIZE, 4, 4);
K_MSGQ_DEFINE(oob_cmdq, VUART_LINE_BUF_SIZE, 4, 4);
K_MSGQ_DEFINE(safs_cmdq, VUART_LINE_BUF_SIZE, 4, 4);

static void vuart_handle_line(const char *line)
{
	printk("vuart cmd: %s\n", line);

	if (strncmp(line, "perif ", 6) == 0) {
		(void)k_msgq_put(&perif_cmdq, line, K_NO_WAIT);
	} else if (strncmp(line, "oob ", 4) == 0) {
		(void)k_msgq_put(&oob_cmdq, line, K_NO_WAIT);
	} else if (strncmp(line, "safs ", 5) == 0) {
		(void)k_msgq_put(&safs_cmdq, line, K_NO_WAIT);
	}
}

/*
 * Waits on the given channel queue for one of the given command strings,
 * discarding any other line received meanwhile. Returns the matched index,
 * or -1 on timeout.
 */
static int vuart_wait_for_cmds(struct k_msgq *cmdq, const char *const *cmds, size_t num_cmds,
			       k_timeout_t timeout)
{
	char msg[VUART_LINE_BUF_SIZE];
	k_timepoint_t deadline = sys_timepoint_calc(timeout);

	while (!sys_timepoint_expired(deadline)) {
		if (k_msgq_get(cmdq, msg, sys_timepoint_timeout(deadline)) != 0) {
			break;
		}

		for (size_t i = 0; i < num_cmds; i++) {
			if (strcmp(msg, cmds[i]) == 0) {
				return (int)i;
			}
		}
	}

	return -1;
}

static void vuart_thread_entry(void *p1, void *p2, void *p3)
{
	static char line[VUART_LINE_BUF_SIZE];
	size_t len = 0;
	unsigned char c;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (uart_poll_in(vuart_dev, &c) != 0) {
			k_usleep(100);
			continue;
		}

		if (c == '\n' || c == '\r') {
			if (len > 0) {
				line[len] = '\0';
				vuart_handle_line(line);
				len = 0;
			}
			continue;
		}

		if (len < VUART_LINE_BUF_SIZE - 1) {
			line[len++] = (char)c;
		}
	}
}

/*
 * perif_thread, oob_thread and the safs TAF handler are attached up front,
 * in the suite setup below, rather than from inside whichever ZTEST used to
 * kick them off - waiting for test_espi_link_up to pass first meant packets
 * the tester sent right as the link came up could already be missed by the
 * time the listener got created.
 *
 * Each of the perif/oob/safs testcases below is tester-driven over vuart0
 * and shares one 30-minute budget, counted from that same setup point: if
 * the tester's "<name> start" or its "<name> pass"/"<name> fail" doesn't
 * arrive before the budget runs out, the testcase fails.
 */
#define ESPI_TESTCASE_TIMEOUT K_MINUTES(30)

static k_timepoint_t espi_testcase_deadline;

static void *espi_suite_setup(void)
{
	k_thread_create(&perif_thread_data, perif_thread_stack,
			K_THREAD_STACK_SIZEOF(perif_thread_stack), perif_thread_entry,
			NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(&perif_thread_data, "perif_thread");

	k_thread_create(&oob_thread_data, oob_thread_stack, K_THREAD_STACK_SIZEOF(oob_thread_stack),
			oob_thread_entry, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(&oob_thread_data, "oob_thread");

	memset(safs_shadow_buf, 0xff, sizeof(safs_shadow_buf));
	(void)espi_aspeed_taf_register(espi_dev, safs_taf_handler, NULL);

	k_thread_create(&vuart_thread_data, vuart_thread_stack,
			K_THREAD_STACK_SIZEOF(vuart_thread_stack), vuart_thread_entry, NULL,
			NULL, NULL, K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
	k_thread_name_set(&vuart_thread_data, "vuart_thread");

	espi_testcase_deadline = sys_timepoint_calc(ESPI_TESTCASE_TIMEOUT);

	return NULL;
}

/*
 * Waits for "<name> start" and then "<name> pass"/"<name> fail" on cmdq,
 * each bounded by whatever remains of espi_testcase_deadline. The tester is
 * not guaranteed to actually send "<name> start" before its result - if
 * "<name> pass"/"<name> fail" shows up first, that's accepted directly
 * instead of waiting forever for a start that never comes. Returns 0 on
 * pass, -1 on anything else (fail, timeout on either wait).
 */
static int espi_wait_testcase(struct k_msgq *cmdq, const char *name)
{
	char start_cmd[VUART_LINE_BUF_SIZE];
	char pass_cmd[VUART_LINE_BUF_SIZE];
	char fail_cmd[VUART_LINE_BUF_SIZE];
	const char *cmds[3];
	int rc;

	snprintk(start_cmd, sizeof(start_cmd), "%s start", name);
	snprintk(pass_cmd, sizeof(pass_cmd), "%s pass", name);
	snprintk(fail_cmd, sizeof(fail_cmd), "%s fail", name);
	cmds[0] = start_cmd;
	cmds[1] = pass_cmd;
	cmds[2] = fail_cmd;

	rc = vuart_wait_for_cmds(cmdq, cmds, ARRAY_SIZE(cmds),
				 sys_timepoint_timeout(espi_testcase_deadline));
	if (rc < 0) {
		printk("[%s] did not receive start/pass/fail within the testcase budget\n", name);
		return -1;
	}

	if (rc != 0) {
		/* index 1 = pass_cmd, index 2 = fail_cmd - a result with no prior start */
		return (rc == 1) ? 0 : -1;
	}

	const char *result_cmds[2] = { pass_cmd, fail_cmd };

	rc = vuart_wait_for_cmds(cmdq, result_cmds, ARRAY_SIZE(result_cmds),
				 sys_timepoint_timeout(espi_testcase_deadline));
	if (rc < 0) {
		printk("[%s] did not receive pass/fail within the testcase budget\n", name);
		return -1;
	}

	return (rc == 0) ? 0 : -1;
}

/*
 * Numbered so ztest's (alphabetical, not source-order) default test
 * ordering still runs these in the intended sequence.
 */
#if !defined(AST1040_CONCURRENT_ALL)
ZTEST(espi, test_espi_0_ready)
{
	zassert_equal(test_espi(), AST_TEST_PASS, "espi test failed");
}

ZTEST(espi, test_espi_1_link_up)
{
	zassert_equal(test_espi_link(), AST_TEST_PASS, "espi link test failed");
}

ZTEST(espi, test_espi_2_perif_mcyc)
{
	zassert_equal(espi_wait_testcase(&perif_cmdq, "perif"), 0,
		     "perif memory cycle r/w test did not start/finish within 30 minutes");
}

ZTEST(espi, test_espi_3_oob)
{
	zassert_equal(espi_wait_testcase(&oob_cmdq, "oob"), 0,
		     "oob loopback test did not start/finish within 30 minutes");
}

ZTEST(espi, test_espi_4_safs)
{
	zassert_equal(espi_wait_testcase(&safs_cmdq, "safs"), 0,
		     "safs loopback test did not start/finish within 30 minutes");
}

ZTEST_SUITE(espi, NULL, espi_suite_setup, NULL, NULL, NULL);
#endif
