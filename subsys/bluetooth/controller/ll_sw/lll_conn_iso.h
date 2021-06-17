/*
 * Copyright (c) 2021 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

struct node_tx_iso {
	union {
		void        *next;
		memq_link_t *link;
	};

	uint64_t payload_number : 39; /* cisPayloadNumber */
	uint8_t  pdu[];
};

struct lll_conn_iso_stream_rxtx {
	uint8_t phy;            /* PHY */
	uint8_t burst_number;   /* Burst number (BN) */
	uint8_t flush_timeout;  /* Flush timeout (FT) */
	uint8_t max_octets;     /* Maximum PDU size */
};

struct lll_conn_iso_stream {
	uint16_t acl_handle;        /* ACL connection handle (for encryption,
				     * channel map, crc init)
				     */
	uint16_t handle;            /* CIS handle */

	/* Connection parameters */
	uint8_t  access_addr[4];    /* Access address */
	uint32_t offset;            /* Offset of CIS from start of CIG in us */
	uint32_t sub_interval;      /* Interval between subevents in us */
	uint8_t  num_subevents;     /* Number of subevents */
	struct lll_conn_iso_stream_rxtx rx; /* RX parameters */
	struct lll_conn_iso_stream_rxtx tx; /* TX parameters */

	/* Event and payload counters */
	uint64_t event_count : 39;       /* cisEventCount */
	uint64_t rx_payload_number : 39; /* cisPayloadNumber */

	/* Acknowledgment and flow control */
	uint8_t sn:1;               /* Sequence number */
	uint8_t nesn:1;             /* Next expected sequence number */
	uint8_t cie:1;              /* Close isochronous event */

	/* Resumption information */
	uint8_t next_subevent;      /* Next subevent to schedule */

	/* Transmission queue */
	MEMQ_DECLARE(tx);
	memq_link_t link_tx;
	memq_link_t *link_tx_free;
};

struct lll_conn_iso_group {
	struct lll_hdr hdr;

	uint16_t handle;        /* CIG handle (internal) */
	uint8_t  num_cis : 5;   /* Number of CISes in this CIG */
	uint8_t  role : 1;      /* 0: CENTRAL, 1: PERIPHERAL*/
	uint8_t  paused : 1;    /* 1: CIG is paused */
#if defined(CONFIG_BT_CTLR_CONN_ISO_STREAMS_PER_GROUP)
	uint16_t cis_handles[CONFIG_BT_CTLR_CONN_ISO_STREAMS_PER_GROUP];
#endif /* CONFIG_BT_CTLR_CONN_ISO_STREAMS */

	/* Resumption information */
	uint8_t  resume_cis;    /* CIS index to schedule at resume */
};

int lll_conn_iso_init(void);
int lll_conn_iso_reset(void);
void lll_conn_iso_done(struct lll_conn_iso_group *cig, uint8_t trx_cnt,
		       uint16_t prog_to_anchor_us, uint8_t mic_state);
void lll_conn_iso_flush(uint16_t handle, struct lll_conn_iso_stream *lll);
