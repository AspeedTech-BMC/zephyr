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
#include <zephyr/sys/byteorder.h>
#include <stdlib.h>

LOG_MODULE_DECLARE(espi_shell, CONFIG_LOG_DEFAULT_LEVEL);

extern const struct device *espi_dev;

/*
 * espi_aspeed_perif_pc_get_rx() writes up to ESPI_PLD_LEN_MAX payload bytes
 * plus a header (MEMWR64 is the worst case: espi_comm_hdr + a 64-bit addr).
 */
static uint8_t pc_rx_pkt[ESPI_PLD_LEN_MAX + sizeof(struct espi_comm_hdr) + sizeof(uint64_t)];
static uint8_t pc_tx_pkt[ESPI_PLD_LEN_MAX];

static void print_hex(const struct shell *sh, const uint8_t *buf, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%02x ", buf[i]);
		if ((i + 1) % 16 == 0) {
			shell_fprintf(sh, SHELL_NORMAL, "\n");
		}
	}
	shell_fprintf(sh, SHELL_NORMAL, "\n");
}

static int cmd_perif_pc_get(const struct shell *sh, size_t argc, char **argv)
{
	int ret;
	uint8_t *after_hdr;
	uint8_t *data_ptr;
	uint32_t addr32;
	uint64_t addr64;
	uint16_t data_len;
	struct espi_comm_hdr *hdr;
	struct espi_perif_msg *msg;
	struct espi_aspeed_ioc ioc = {
		.pkt_len = sizeof(pc_rx_pkt),
		.pkt = pc_rx_pkt,
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	ret = espi_aspeed_perif_pc_get_rx(espi_dev, &ioc, true);
	if (ret) {
		shell_error(sh, "PC get_rx failed: %d", ret);
		return ret;
	}

	hdr = (struct espi_comm_hdr *)pc_rx_pkt;
	after_hdr = pc_rx_pkt + sizeof(struct espi_comm_hdr);
	data_len = ((uint16_t)hdr->len_h << 8) | hdr->len_l;

	shell_print(sh, "cyc=0x%02x tag=%u len=%u", hdr->cyc, hdr->tag, data_len);

	switch (hdr->cyc) {
	case ESPI_PERIF_MEMWR32:
		addr32 = sys_get_be32(after_hdr);
		data_ptr = after_hdr + sizeof(uint32_t);

		shell_print(sh, "type: mem32 write addr <0x%08x>", addr32);
		if (data_len) {
			shell_print(sh, "data (%u bytes):", data_len);
			print_hex(sh, data_ptr, data_len);
		}
		break;
	case ESPI_PERIF_MEMWR64:
		addr64 = sys_get_be64(after_hdr);
		data_ptr = after_hdr + sizeof(uint64_t);

		shell_print(sh, "type: mem64 write addr <0x%016llx>", addr64);
		if (data_len) {
			shell_print(sh, "data (%u bytes):", data_len);
			print_hex(sh, data_ptr, data_len);
		}
		break;
	case ESPI_PERIF_MSG:
	case ESPI_PERIF_MSG_D:
		msg = (struct espi_perif_msg *)pc_rx_pkt;
		shell_print(sh, "type: msg%s code=0x%02x msg_bytes=%02x %02x %02x %02x",
			    hdr->cyc == ESPI_PERIF_MSG_D ? "_d" : "",
			    msg->msg_code,
			    msg->msg_byte[0], msg->msg_byte[1],
			    msg->msg_byte[2], msg->msg_byte[3]);
		if (data_len) {
			shell_print(sh, "data (%u bytes):", data_len);
			print_hex(sh, msg->data, data_len);
		}
		break;
	default:
		shell_print(sh, "type: unknown (0x%02x), raw %u bytes:", hdr->cyc, ioc.pkt_len);
		print_hex(sh, pc_rx_pkt, ioc.pkt_len);
		break;
	}

	return 0;
}

static uint8_t np_rx_pkt[ESPI_PLD_LEN_MAX];

static int cmd_perif_np_get(const struct shell *sh, size_t argc, char **argv)
{
	int ret;
	uint16_t req_len;
	uint64_t addr64;
	struct espi_comm_hdr *hdr;
	struct espi_perif_mem32 *m32;
	struct espi_perif_io *io;
	struct espi_aspeed_ioc ioc = {
		.pkt_len = ESPI_PLD_LEN_MAX,
		.pkt = np_rx_pkt,
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	ret = espi_aspeed_perif_np_get_rx(espi_dev, &ioc, true);
	if (ret) {
		shell_error(sh, "NP get_rx failed: %d", ret);
		return ret;
	}

	hdr = (struct espi_comm_hdr *)np_rx_pkt;
	req_len = ((uint16_t)hdr->len_h << 8) | hdr->len_l;

	shell_print(sh, "cyc=0x%02x tag=%u len=%u", hdr->cyc, hdr->tag, req_len);

	switch (hdr->cyc) {
	case ESPI_PERIF_MEMRD32:
		m32 = (struct espi_perif_mem32 *)np_rx_pkt;
		shell_print(sh, "type: memrd32 addr=0x%08x req_len=%u",
			    sys_be32_to_cpu(m32->addr_be), req_len);
		break;
	case ESPI_PERIF_MEMRD64:
		addr64 = sys_get_be64(np_rx_pkt + sizeof(struct espi_comm_hdr));
		shell_print(sh, "type: memrd64 addr=0x%016llx req_len=%u",
			    (unsigned long long)addr64, req_len);
		break;
	case ESPI_PERIF_IORD:
		io = (struct espi_perif_io *)np_rx_pkt;
		shell_print(sh, "type: io_read addr=0x%04x req_len=%u",
			    sys_be16_to_cpu(io->addr_be), req_len);
		break;
	case ESPI_PERIF_IOWR:
		io = (struct espi_perif_io *)np_rx_pkt;
		shell_print(sh, "type: io_write addr=0x%04x data_len=%u",
			    sys_be16_to_cpu(io->addr_be), req_len);
		if (req_len) {
			shell_print(sh, "data (%u bytes):", req_len);
			print_hex(sh, io->data, req_len);
		}
		break;
	default:
		shell_print(sh, "type: unknown (0x%02x), raw %u bytes:", hdr->cyc, ioc.pkt_len);
		print_hex(sh, np_rx_pkt, ioc.pkt_len);
		break;
	}

	return 0;
}

/* hdr_len: value encoded in espi_comm_hdr.len (may differ from payload_len,
 * e.g. MSG packets count only data bytes, not msg_code/msg_byte fields).
 */
static int pc_put_tx(const struct shell *sh, uint8_t cyc, uint8_t tag,
		     const uint8_t *payload, size_t payload_len, size_t hdr_len)
{
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)pc_tx_pkt;
	struct espi_aspeed_ioc ioc;
	int ret;

	if (sizeof(struct espi_comm_hdr) + payload_len > ESPI_PLD_LEN_MAX) {
		shell_error(sh, "Packet too long (max payload %u bytes)",
			    (unsigned int)(ESPI_PLD_LEN_MAX - sizeof(struct espi_comm_hdr)));
		return -EINVAL;
	}

	hdr->cyc   = cyc;
	hdr->tag   = tag;
	hdr->len_h = (hdr_len >> 8) & 0xf;
	hdr->len_l = hdr_len & 0xff;

	if (payload_len)
		memcpy(pc_tx_pkt + sizeof(struct espi_comm_hdr), payload, payload_len);

	ioc.pkt     = pc_tx_pkt;
	ioc.pkt_len = sizeof(struct espi_comm_hdr) + payload_len;

	ret = espi_aspeed_perif_pc_put_tx(espi_dev, &ioc);
	if (ret) {
		shell_error(sh, "PC put_tx failed: %d", ret);
		return ret;
	}

	shell_print(sh, "PC TX sent cyc=0x%02x tag=%u payload=%zu bytes",
		    cyc, tag, payload_len);
	return 0;
}

/* perif pc_put raw <cyc_hex> <tag> [hex_bytes...]  — raw packet */
static int cmd_perif_pc_put_raw(const struct shell *sh, size_t argc, char **argv)
{
	size_t i;
	size_t payload_len;
	static uint8_t payload[ESPI_PLD_LEN_MAX];
	uint8_t cyc;
	uint8_t tag;

	if (argc < 3) {
		shell_error(sh, "Usage: perif pc_put raw <cyc_hex> <tag> [hex_bytes...]");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	cyc         = (uint8_t)strtoul(argv[1], NULL, 16);
	tag         = (uint8_t)strtoul(argv[2], NULL, 0);
	payload_len = argc - 3;

	for (i = 0; i < payload_len; i++)
		payload[i] = (uint8_t)strtoul(argv[3 + i], NULL, 16);

	return pc_put_tx(sh, cyc, tag, payload, payload_len, payload_len);
}

/* perif pc_put msg <tag> <msg_code_hex> <mb0> <mb1> <mb2> <mb3> [hex_bytes...]
 * Builds an ESPI_PERIF_MSG_D packet with a 5-byte message header followed by
 * optional data bytes.
 */
static int cmd_perif_pc_put_msg(const struct shell *sh, size_t argc, char **argv)
{
	size_t i;
	size_t data_len;
	size_t payload_len;
	static uint8_t payload[ESPI_PLD_LEN_MAX];
	uint8_t tag;

	/* argv: [0]=msg [1]=tag [2]=msg_code [3..6]=msg_bytes [7+]=data */
	if (argc < 7) {
		shell_error(sh, "Usage: perif pc_put msg <tag> <msg_code_hex>"
			    " <mb0> <mb1> <mb2> <mb3> [hex_bytes...]");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	tag      = (uint8_t)strtoul(argv[1], NULL, 0);
	data_len = argc - 7;

	/* payload = everything after espi_comm_hdr:
	 * [0]      msg_code
	 * [1..4]   msg_byte[0..3]
	 * [5+]     data
	 */
	payload_len = 1 + 4 + data_len;

	payload[0] = (uint8_t)strtoul(argv[2], NULL, 16); /* msg_code */
	payload[1] = (uint8_t)strtoul(argv[3], NULL, 16); /* msg_byte[0] */
	payload[2] = (uint8_t)strtoul(argv[4], NULL, 16); /* msg_byte[1] */
	payload[3] = (uint8_t)strtoul(argv[5], NULL, 16); /* msg_byte[2] */
	payload[4] = (uint8_t)strtoul(argv[6], NULL, 16); /* msg_byte[3] */

	for (i = 0; i < data_len; i++)
		payload[5 + i] = (uint8_t)strtoul(argv[7 + i], NULL, 16);

	/* len field counts only data bytes, not msg_code + msg_byte[4] */
	return pc_put_tx(sh,
			 data_len ? ESPI_PERIF_MSG_D : ESPI_PERIF_MSG,
			 tag, payload, payload_len, data_len);
}

/* perif pc_put mw32 <tag> <addr_hex_32bit> [hex_bytes...]
 * Builds a MEMWR32 packet: 4-byte BE address followed by data bytes.
 * len field = number of data bytes.
 */
static int cmd_perif_pc_put_mw32(const struct shell *sh, size_t argc, char **argv)
{
	size_t i;
	size_t data_len;
	size_t payload_len;
	static uint8_t payload[ESPI_PLD_LEN_MAX];
	uint8_t tag;
	uint32_t addr;

	/* argv: [0]=mw32 [1]=tag [2]=addr [3+]=data */
	if (argc < 3) {
		shell_error(sh, "Usage: perif pc_put mw32 <tag> <addr_hex> [hex_bytes...]");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	tag      = (uint8_t)strtoul(argv[1], NULL, 0);
	addr     = (uint32_t)strtoul(argv[2], NULL, 16);
	data_len = argc - 3;

	/* payload: 4-byte BE addr + data */
	payload_len = sizeof(uint32_t) + data_len;
	sys_put_be32(addr, payload);

	for (i = 0; i < data_len; i++)
		payload[sizeof(uint32_t) + i] = (uint8_t)strtoul(argv[3 + i], NULL, 16);

	return pc_put_tx(sh, ESPI_PERIF_MEMWR32, tag, payload, payload_len, data_len);
}

/* perif pc_put mw64 <tag> <addr_hex_64bit> [hex_bytes...]
 * Builds a MEMWR64 packet: 8-byte BE address followed by data bytes.
 * len field = number of data bytes.
 */
static int cmd_perif_pc_put_mw64(const struct shell *sh, size_t argc, char **argv)
{
	size_t i;
	size_t data_len;
	size_t payload_len;
	static uint8_t payload[ESPI_PLD_LEN_MAX];
	uint8_t tag;
	uint64_t addr;

	/* argv: [0]=mw64 [1]=tag [2]=addr [3+]=data */
	if (argc < 3) {
		shell_error(sh, "Usage: perif pc_put mw64 <tag> <addr_hex> [hex_bytes...]");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	tag      = (uint8_t)strtoul(argv[1], NULL, 0);
	addr     = (uint64_t)strtoull(argv[2], NULL, 16);
	data_len = argc - 3;

	/* payload: 8-byte BE addr + data */
	payload_len = sizeof(uint64_t) + data_len;
	sys_put_be64(addr, payload);

	for (i = 0; i < data_len; i++)
		payload[sizeof(uint64_t) + i] = (uint8_t)strtoul(argv[3 + i], NULL, 16);

	return pc_put_tx(sh, ESPI_PERIF_MEMWR64, tag, payload, payload_len, data_len);
}

/* perif pc_put suc_cmplt <tag>
 * Successful Completion Without Data (cyc=0x06), no payload.
 */
static int cmd_perif_pc_put_suc_cmplt(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t tag;

	if (argc < 2) {
		shell_error(sh, "Usage: perif pc_put suc_cmplt <tag>");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	tag = (uint8_t)strtoul(argv[1], NULL, 0);

	return pc_put_tx(sh, ESPI_PERIF_SUC_CMPLT, tag, NULL, 0, 0);
}

/* perif pc_put suc_cmplt_d <tag> [hex_bytes...]
 * Successful Completion With Data (cyc=0x0f, D_ONLY), data bytes as payload.
 * len = number of data bytes.
 */
static int cmd_perif_pc_put_suc_cmplt_d(const struct shell *sh, size_t argc, char **argv)
{
	size_t i;
	size_t data_len;
	static uint8_t payload[ESPI_PLD_LEN_MAX];
	uint8_t tag;

	if (argc < 2) {
		shell_error(sh, "Usage: perif pc_put suc_cmplt_d <tag> [hex_bytes...]");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	tag      = (uint8_t)strtoul(argv[1], NULL, 0);
	data_len = argc - 2;

	for (i = 0; i < data_len; i++)
		payload[i] = (uint8_t)strtoul(argv[2 + i], NULL, 16);

	return pc_put_tx(sh, ESPI_PERIF_SUC_CMPLT_D_ONLY, tag, payload, data_len, data_len);
}

/* perif pc_put unsuc_cmplt <tag>
 * Unsuccessful Completion Without Data (cyc=0x0c), no payload.
 */
static int cmd_perif_pc_put_unsuc_cmplt(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t tag;

	if (argc < 2) {
		shell_error(sh, "Usage: perif pc_put unsuc_cmplt <tag>");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	tag = (uint8_t)strtoul(argv[1], NULL, 0);

	return pc_put_tx(sh, ESPI_PERIF_UNSUC_CMPLT, tag, NULL, 0, 0);
}

SHELL_STATIC_SUBCMD_SET_CREATE(pc_put_cmds,
	SHELL_CMD_ARG(raw, NULL,
		      "Send raw PC packet: <cyc_hex> <tag> [hex_bytes...]",
		      cmd_perif_pc_put_raw, 3, CONFIG_SHELL_ARGC_MAX - 3),
	SHELL_CMD_ARG(msg, NULL,
		      "Send MSG/MSG_D packet: <tag> <msg_code_hex> <mb0> <mb1> <mb2> <mb3> "
		      "[data...]",
		      cmd_perif_pc_put_msg, 7, CONFIG_SHELL_ARGC_MAX - 7),
	SHELL_CMD_ARG(mw32, NULL,
		      "Send MEMWR32 packet: <tag> <addr_hex> [hex_bytes...]",
		      cmd_perif_pc_put_mw32, 3, CONFIG_SHELL_ARGC_MAX - 3),
	SHELL_CMD_ARG(mw64, NULL,
		      "Send MEMWR64 packet: <tag> <addr_hex> [hex_bytes...]",
		      cmd_perif_pc_put_mw64, 3, CONFIG_SHELL_ARGC_MAX - 3),
	SHELL_CMD_ARG(suc_cmplt, NULL,
		      "Successful Completion Without Data (cyc=0x06): <tag>",
		      cmd_perif_pc_put_suc_cmplt, 2, 0),
	SHELL_CMD_ARG(suc_cmplt_d, NULL,
		      "Successful Completion With Data (cyc=0x0f): <tag> [hex_bytes...]",
		      cmd_perif_pc_put_suc_cmplt_d, 2, CONFIG_SHELL_ARGC_MAX - 2),
	SHELL_CMD_ARG(unsuc_cmplt, NULL,
		      "Unsuccessful Completion Without Data (cyc=0x0c): <tag>",
		      cmd_perif_pc_put_unsuc_cmplt, 2, 0),
	SHELL_SUBCMD_SET_END
);

static int np_put_tx(const struct shell *sh, uint8_t cyc, uint8_t tag,
		     const uint8_t *payload, size_t payload_len, size_t hdr_len)
{
	struct espi_comm_hdr *hdr = (struct espi_comm_hdr *)pc_tx_pkt;
	struct espi_aspeed_ioc ioc;
	int ret;

	if (sizeof(struct espi_comm_hdr) + payload_len > ESPI_PLD_LEN_MAX) {
		shell_error(sh, "Packet too long (max payload %u bytes)",
			    (unsigned int)(ESPI_PLD_LEN_MAX - sizeof(struct espi_comm_hdr)));
		return -EINVAL;
	}

	hdr->cyc   = cyc;
	hdr->tag   = tag;
	hdr->len_h = (hdr_len >> 8) & 0xf;
	hdr->len_l = hdr_len & 0xff;

	if (payload_len)
		memcpy(pc_tx_pkt + sizeof(struct espi_comm_hdr), payload, payload_len);

	ioc.pkt     = pc_tx_pkt;
	ioc.pkt_len = sizeof(struct espi_comm_hdr) + payload_len;

	ret = espi_aspeed_perif_np_put_tx(espi_dev, &ioc);
	if (ret) {
		shell_error(sh, "NP put_tx failed: %d", ret);
		return ret;
	}

	shell_print(sh, "NP TX sent cyc=0x%02x tag=%u len=%zu payload=%zu bytes",
		    cyc, tag, hdr_len, payload_len);
	return 0;
}

/* perif np_put md32 <tag> <len_hex> <addr_hex_32bit>
 * Builds a MEMRD32 (non-posted read request): len = bytes to read, payload = BE addr.
 */
static int cmd_perif_np_put_md32(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t payload[sizeof(uint32_t)];
	uint8_t tag;
	size_t  req_len;
	uint32_t addr;

	if (argc < 4) {
		shell_error(sh, "Usage: perif np_put md32 <tag> <len_hex> <addr_hex>");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	tag     = (uint8_t)strtoul(argv[1], NULL, 0);
	req_len = (size_t)strtoul(argv[2], NULL, 16);
	addr    = (uint32_t)strtoul(argv[3], NULL, 16);

	sys_put_be32(addr, payload);

	return np_put_tx(sh, ESPI_PERIF_MEMRD32, tag, payload, sizeof(payload), req_len);
}

/* perif np_put md64 <tag> <len_hex> <addr_hex_64bit>
 * Builds a MEMRD64 (non-posted read request): len = bytes to read, payload = BE addr.
 */
static int cmd_perif_np_put_md64(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t payload[sizeof(uint64_t)];
	uint8_t tag;
	size_t  req_len;
	uint64_t addr;

	if (argc < 4) {
		shell_error(sh, "Usage: perif np_put md64 <tag> <len_hex> <addr_hex>");
		return -EINVAL;
	}

	if (!device_is_ready(espi_dev)) {
		shell_error(sh, "eSPI device not ready");
		return -ENODEV;
	}

	tag     = (uint8_t)strtoul(argv[1], NULL, 0);
	req_len = (size_t)strtoul(argv[2], NULL, 16);
	addr    = (uint64_t)strtoull(argv[3], NULL, 16);

	sys_put_be64(addr, payload);

	return np_put_tx(sh, ESPI_PERIF_MEMRD64, tag, payload, sizeof(payload), req_len);
}

/* perif wr_req mem32 <tag> <addr_hex> [hex_bytes...] */
static int cmd_perif_wr_req_mem32(const struct shell *sh, size_t argc, char **argv)
{
	struct espi_request_packet req;
	static uint8_t data[ESPI_PLD_LEN_MAX];
	int ret;
	int i;

	req.cycle_type = ESPI_CYCLE_MEMORY_WRITE32;
	req.tag        = (uint8_t)strtoul(argv[1], NULL, 0);
	req.address    = (uint32_t)strtoul(argv[2], NULL, 16);
	req.len        = argc - 3;
	req.data       = data;

	for (i = 0; i < req.len; i++) {
		data[i] = (uint8_t)strtoul(argv[3 + i], NULL, 16);
	}

	ret = espi_write_request(espi_dev, &req);
	if (ret) {
		shell_error(sh, "write_request failed: %d", ret);
		return ret;
	}

	shell_print(sh, "wr_req mem32 sent: tag=%u addr=0x%08x len=%u",
		    req.tag, req.address, req.len);
	return 0;
}

/* perif wr_req mem64 <tag> <addr_hex> [hex_bytes...] */
static int cmd_perif_wr_req_mem64(const struct shell *sh, size_t argc, char **argv)
{
	struct espi_request_packet req;
	static uint8_t data[ESPI_PLD_LEN_MAX];
	int ret;
	int i;

	req.cycle_type = ESPI_CYCLE_MEMORY_WRITE64;
	req.tag        = (uint8_t)strtoul(argv[1], NULL, 0);
	req.address    = (uint32_t)strtoull(argv[2], NULL, 16);
	req.len        = argc - 3;
	req.data       = data;

	for (i = 0; i < req.len; i++) {
		data[i] = (uint8_t)strtoul(argv[3 + i], NULL, 16);
	}

	ret = espi_write_request(espi_dev, &req);
	if (ret) {
		shell_error(sh, "write_request failed: %d", ret);
		return ret;
	}

	shell_print(sh, "wr_req mem64 sent: tag=%u addr=0x%08x len=%u",
		    req.tag, req.address, req.len);
	return 0;
}

static int do_rd_req(const struct shell *sh, enum espi_cycle_type cyc,
		     uint8_t tag, uint32_t addr, uint16_t len)
{
	struct espi_request_packet req;
	static uint8_t data[ESPI_PLD_LEN_MAX];
	int ret;
	int i;

	req.cycle_type = cyc;
	req.tag        = tag;
	req.address    = addr;
	req.len        = len;
	req.data       = data;

	ret = espi_read_request(espi_dev, &req);
	if (ret) {
		shell_error(sh, "read_request failed: %d", ret);
		return ret;
	}

	shell_print(sh, "rd_req response: cyc=%u tag=%u addr=0x%08x len=%u",
		    req.cycle_type, req.tag, req.address, req.len);
	for (i = 0; i < req.len; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%02x ", data[i]);
		if ((i + 1) % 16 == 0) {
			shell_fprintf(sh, SHELL_NORMAL, "\n");
		}
	}
	if (req.len % 16 != 0) {
		shell_fprintf(sh, SHELL_NORMAL, "\n");
	}
	return 0;
}

/* perif rd_req mem32 <tag> <addr_hex> <len> */
static int cmd_perif_rd_req_mem32(const struct shell *sh, size_t argc, char **argv)
{
	return do_rd_req(sh, ESPI_CYCLE_MEMORY_READ32,
			 (uint8_t)strtoul(argv[1], NULL, 0),
			 (uint32_t)strtoul(argv[2], NULL, 16),
			 (uint16_t)strtoul(argv[3], NULL, 0));
}

/* perif rd_req mem64 <tag> <addr_hex> <len> */
static int cmd_perif_rd_req_mem64(const struct shell *sh, size_t argc, char **argv)
{
	return do_rd_req(sh, ESPI_CYCLE_MEMORY_READ64,
			 (uint8_t)strtoul(argv[1], NULL, 0),
			 (uint32_t)strtoull(argv[2], NULL, 16),
			 (uint16_t)strtoul(argv[3], NULL, 0));
}

SHELL_STATIC_SUBCMD_SET_CREATE(wr_req_cmds,
	SHELL_CMD_ARG(mem32, NULL,
		      "Send MEMWR32 request: <tag> <addr_hex> [hex_bytes...]",
		      cmd_perif_wr_req_mem32, 3, CONFIG_SHELL_ARGC_MAX - 3),
	SHELL_CMD_ARG(mem64, NULL,
		      "Send MEMWR64 request: <tag> <addr_hex> [hex_bytes...]",
		      cmd_perif_wr_req_mem64, 3, CONFIG_SHELL_ARGC_MAX - 3),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(rd_req_cmds,
	SHELL_CMD_ARG(mem32, NULL,
		      "Send MEMRD32 request: <tag> <addr_hex> <len>",
		      cmd_perif_rd_req_mem32, 4, 0),
	SHELL_CMD_ARG(mem64, NULL,
		      "Send MEMRD64 request: <tag> <addr_hex> <len>",
		      cmd_perif_rd_req_mem64, 4, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(np_put_cmds,
	SHELL_CMD_ARG(md32, NULL,
		      "Send MEMRD32 request: <tag> <len_hex> <addr_hex>",
		      cmd_perif_np_put_md32, 4, 0),
	SHELL_CMD_ARG(md64, NULL,
		      "Send MEMRD64 request: <tag> <len_hex> <addr_hex>",
		      cmd_perif_np_put_md64, 4, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(perif_cmds,
	SHELL_CMD(pc_get, NULL, "Receive one PC packet (blocking) and decode it", cmd_perif_pc_get),
	SHELL_CMD(pc_put, &pc_put_cmds, "Send PC packet", NULL),
	SHELL_CMD(np_get, NULL, "Receive one NP packet (blocking) and decode it", cmd_perif_np_get),
	SHELL_CMD(np_put, &np_put_cmds, "Send NP packet", NULL),
	SHELL_CMD(wr_req, &wr_req_cmds, "Send write request", NULL),
	SHELL_CMD(rd_req, &rd_req_cmds, "Send read request", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(perif, &perif_cmds, "eSPI peripheral channel shell commands", NULL);
