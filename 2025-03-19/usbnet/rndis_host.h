/*
 * Host Side support for RNDIS Networking Links
 * Copyright (C) 2005 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef	__LINUX_USB_RNDIS_HOST_H
#define	__LINUX_USB_RNDIS_HOST_H

/*
 * CONTROL uses CDC "encapsulated commands" with funky notifications.
 *  - control-out:  SEND_ENCAPSULATED
 *  - interrupt-in:  RESPONSE_AVAILABLE
 *  - control-in:  GET_ENCAPSULATED
 *
 * We'll try to ignore the RESPONSE_AVAILABLE notifications.
 *
 * REVISIT some RNDIS implementations seem to have curious issues still
 * to be resolved.
 */
struct rndis_msg_hdr {
	__le32	msg_type;			/* RNDIS_MSG_* */
	__le32	msg_len;
	/* followed by data that varies between messages */
	__le32	request_id;
	__le32	status;
	/* ... and more */
} __attribute__ ((packed));

/* MS-Windows uses this strange size, but RNDIS spec says 1024 minimum */
#define	CONTROL_BUFFER_SIZE		1025

/* RNDIS defines an (absurdly huge) 10 second control timeout,
 * but ActiveSync seems to use a more usual 5 second timeout
 * (which matches the USB 2.0 spec).
 */
#define	RNDIS_CONTROL_TIMEOUT_MS	(5 * 1000)

#define RNDIS_MSG_COMPLETION	cpu_to_le32(0x80000000)

/* codes for "msg_type" field of rndis messages;
 * only the data channel uses packet messages (maybe batched);
 * everything else goes on the control channel.
 */
#define RNDIS_MSG_PACKET	cpu_to_le32(0x00000001)	/* 1-N packets */
#define RNDIS_MSG_INIT		cpu_to_le32(0x00000002)
#define RNDIS_MSG_INIT_C	(RNDIS_MSG_INIT|RNDIS_MSG_COMPLETION)
#define RNDIS_MSG_HALT		cpu_to_le32(0x00000003)
#define RNDIS_MSG_QUERY		cpu_to_le32(0x00000004)
#define RNDIS_MSG_QUERY_C	(RNDIS_MSG_QUERY|RNDIS_MSG_COMPLETION)
#define RNDIS_MSG_SET		cpu_to_le32(0x00000005)
#define RNDIS_MSG_SET_C		(RNDIS_MSG_SET|RNDIS_MSG_COMPLETION)
#define RNDIS_MSG_RESET		cpu_to_le32(0x00000006)
#define RNDIS_MSG_RESET_C	(RNDIS_MSG_RESET|RNDIS_MSG_COMPLETION)
#define RNDIS_MSG_INDICATE	cpu_to_le32(0x00000007)
#define RNDIS_MSG_KEEPALIVE	cpu_to_le32(0x00000008)
#define RNDIS_MSG_KEEPALIVE_C	(RNDIS_MSG_KEEPALIVE|RNDIS_MSG_COMPLETION)

/* codes for "status" field of completion messages */
#define	RNDIS_STATUS_SUCCESS			cpu_to_le32(0x00000000)
#define	RNDIS_STATUS_FAILURE			cpu_to_le32(0xc0000001)
#define	RNDIS_STATUS_INVALID_DATA		cpu_to_le32(0xc0010015)
#define	RNDIS_STATUS_NOT_SUPPORTED		cpu_to_le32(0xc00000bb)
#define	RNDIS_STATUS_MEDIA_CONNECT		cpu_to_le32(0x4001000b)
#define	RNDIS_STATUS_MEDIA_DISCONNECT		cpu_to_le32(0x4001000c)
#define	RNDIS_STATUS_MEDIA_SPECIFIC_INDICATION	cpu_to_le32(0x40010012)

/* codes for OID_GEN_PHYSICAL_MEDIUM */
#define	RNDIS_PHYSICAL_MEDIUM_UNSPECIFIED	cpu_to_le32(0x00000000)
#define	RNDIS_PHYSICAL_MEDIUM_WIRELESS_LAN	cpu_to_le32(0x00000001)
#define	RNDIS_PHYSICAL_MEDIUM_CABLE_MODEM	cpu_to_le32(0x00000002)
#define	RNDIS_PHYSICAL_MEDIUM_PHONE_LINE	cpu_to_le32(0x00000003)
#define	RNDIS_PHYSICAL_MEDIUM_POWER_LINE	cpu_to_le32(0x00000004)
#define	RNDIS_PHYSICAL_MEDIUM_DSL		cpu_to_le32(0x00000005)
#define	RNDIS_PHYSICAL_MEDIUM_FIBRE_CHANNEL	cpu_to_le32(0x00000006)
#define	RNDIS_PHYSICAL_MEDIUM_1394		cpu_to_le32(0x00000007)
#define	RNDIS_PHYSICAL_MEDIUM_WIRELESS_WAN	cpu_to_le32(0x00000008)
#define	RNDIS_PHYSICAL_MEDIUM_MAX		cpu_to_le32(0x00000009)

struct rndis_data_hdr {
	__le32	msg_type;		/* RNDIS_MSG_PACKET */
	__le32	msg_len;		/* rndis_data_hdr + data_len + pad */
	__le32	data_offset;		/* 36 -- right after header */
	__le32	data_len;		/* ... real packet size */

	__le32	oob_data_offset;	/* zero */
	__le32	oob_data_len;		/* zero */
	__le32	num_oob;		/* zero */
	__le32	packet_data_offset;	/* zero */

	__le32	packet_data_len;	/* zero */
	__le32	vc_handle;		/* zero */
	__le32	reserved;		/* zero */
} __attribute__ ((packed));

struct rndis_init {		/* OUT */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_INIT */
	__le32	msg_len;			/* 24 */
	__le32	request_id;
	__le32	major_version;			/* of rndis (1.0) */
	__le32	minor_version;
	__le32	max_transfer_size;
} __attribute__ ((packed));

struct rndis_init_c {		/* IN */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_INIT_C */
	__le32	msg_len;
	__le32	request_id;
	__le32	status;
	__le32	major_version;			/* of rndis (1.0) */
	__le32	minor_version;
	__le32	device_flags;
	__le32	medium;				/* zero == 802.3 */
	__le32	max_packets_per_message;
	__le32	max_transfer_size;
	__le32	packet_alignment;		/* max 7; (1<<n) bytes */
	__le32	af_list_offset;			/* zero */
	__le32	af_list_size;			/* zero */
} __attribute__ ((packed));

struct rndis_halt {		/* OUT (no reply) */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_HALT */
	__le32	msg_len;
	__le32	request_id;
} __attribute__ ((packed));

struct rndis_query {		/* OUT */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_QUERY */
	__le32	msg_len;
	__le32	request_id;
	__le32	oid;
	__le32	len;
	__le32	offset;
/*?*/	__le32	handle;				/* zero */
} __attribute__ ((packed));

struct rndis_query_c {		/* IN */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_QUERY_C */
	__le32	msg_len;
	__le32	request_id;
	__le32	status;
	__le32	len;
	__le32	offset;
} __attribute__ ((packed));

struct rndis_set {		/* OUT */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_SET */
	__le32	msg_len;
	__le32	request_id;
	__le32	oid;
	__le32	len;
	__le32	offset;
/*?*/	__le32	handle;				/* zero */
} __attribute__ ((packed));

struct rndis_set_c {		/* IN */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_SET_C */
	__le32	msg_len;
	__le32	request_id;
	__le32	status;
} __attribute__ ((packed));

struct rndis_reset {		/* IN */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_RESET */
	__le32	msg_len;
	__le32	reserved;
} __attribute__ ((packed));

struct rndis_reset_c {		/* OUT */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_RESET_C */
	__le32	msg_len;
	__le32	status;
	__le32	addressing_lost;
} __attribute__ ((packed));

struct rndis_indicate {		/* IN (unrequested) */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_INDICATE */
	__le32	msg_len;
	__le32	status;
	__le32	length;
	__le32	offset;
/**/	__le32	diag_status;
	__le32	error_offset;
/**/	__le32	message;
} __attribute__ ((packed));

struct rndis_keepalive {	/* OUT (optionally IN) */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_KEEPALIVE */
	__le32	msg_len;
	__le32	request_id;
} __attribute__ ((packed));

struct rndis_keepalive_c {	/* IN (optionally OUT) */
	/* header and: */
	__le32	msg_type;			/* RNDIS_MSG_KEEPALIVE_C */
	__le32	msg_len;
	__le32	request_id;
	__le32	status;
} __attribute__ ((packed));

/* NOTE:  about 30 OIDs are "mandatory" for peripherals to support ... and
 * there are gobs more that may optionally be supported.  We'll avoid as much
 * of that mess as possible.
 */
#define OID_802_3_PERMANENT_ADDRESS	cpu_to_le32(0x01010101)
#define OID_GEN_MAXIMUM_FRAME_SIZE	cpu_to_le32(0x00010106)
#define OID_GEN_CURRENT_PACKET_FILTER	cpu_to_le32(0x0001010e)
#define OID_GEN_PHYSICAL_MEDIUM		cpu_to_le32(0x00010202)

/* packet filter bits used by OID_GEN_CURRENT_PACKET_FILTER */
#define RNDIS_PACKET_TYPE_DIRECTED		cpu_to_le32(0x00000001)
#define RNDIS_PACKET_TYPE_MULTICAST		cpu_to_le32(0x00000002)
#define RNDIS_PACKET_TYPE_ALL_MULTICAST		cpu_to_le32(0x00000004)
#define RNDIS_PACKET_TYPE_BROADCAST		cpu_to_le32(0x00000008)
#define RNDIS_PACKET_TYPE_SOURCE_ROUTING	cpu_to_le32(0x00000010)
#define RNDIS_PACKET_TYPE_PROMISCUOUS		cpu_to_le32(0x00000020)
#define RNDIS_PACKET_TYPE_SMT			cpu_to_le32(0x00000040)
#define RNDIS_PACKET_TYPE_ALL_LOCAL		cpu_to_le32(0x00000080)
#define RNDIS_PACKET_TYPE_GROUP			cpu_to_le32(0x00001000)
#define RNDIS_PACKET_TYPE_ALL_FUNCTIONAL	cpu_to_le32(0x00002000)
#define RNDIS_PACKET_TYPE_FUNCTIONAL		cpu_to_le32(0x00004000)
#define RNDIS_PACKET_TYPE_MAC_FRAME		cpu_to_le32(0x00008000)

/* default filter used with RNDIS devices */
#define RNDIS_DEFAULT_FILTER ( \
	RNDIS_PACKET_TYPE_DIRECTED | \
	RNDIS_PACKET_TYPE_BROADCAST | \
	RNDIS_PACKET_TYPE_ALL_MULTICAST | \
	RNDIS_PACKET_TYPE_PROMISCUOUS)

/* Flags to require specific physical medium type for generic_rndis_bind() */
#define FLAG_RNDIS_PHYM_NOT_WIRELESS	0x0001
#define FLAG_RNDIS_PHYM_WIRELESS	0x0002

/* Flags for driver_info::data */
#define RNDIS_DRIVER_DATA_POLL_STATUS	1	/* poll status before control */

extern void rndis_status(struct usbnet *dev, struct urb *urb);
extern int
rndis_command(struct usbnet *dev, struct rndis_msg_hdr *buf, int buflen);
extern int
generic_rndis_bind(struct usbnet *dev, struct usb_interface *intf, int flags);
extern void rndis_unbind(struct usbnet *dev, struct usb_interface *intf);
extern int rndis_rx_fixup(struct usbnet *dev, struct fp_buf *buf);
extern int rndis_tx_fixup(struct usbnet *dev, struct fp_buf *fpb, gfp_t flags);

#endif	/* __LINUX_USB_RNDIS_HOST_H */
