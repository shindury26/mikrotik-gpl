/*
 * Copyright (c) 1990, 1991, 1993, 1994, 1995, 1996, 1997
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that: (1) source code distributions
 * retain the above copyright notice and this paragraph in its entirety, (2)
 * distributions including binary code include the above copyright notice and
 * this paragraph in its entirety in the documentation or other materials
 * provided with the distribution, and (3) all advertising materials mentioning
 * features or use of this software display the following acknowledgement:
 * ``This product includes software developed by the University of California,
 * Lawrence Berkeley Laboratory and its contributors.'' Neither the name of
 * the University nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior
 * written permission.
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-vjc.c,v 1.11.2.3 2003/11/19 01:09:12 guy Exp $ (LBL)";
#endif

#include <tcpdump-stdinc.h>

#include <pcap.h>
#include <stdio.h>

#include "interface.h"
#include "addrtoname.h"

#include "slcompress.h"
#include "ppp.h"

/*
 * XXX - for BSD/OS PPP, what packets get supplied with a PPP header type
 * of PPP_VJC and what packets get supplied with a PPP header type of
 * PPP_VJNC?  PPP_VJNC is for "UNCOMPRESSED_TCP" packets, and PPP_VJC
 * is for COMPRESSED_TCP packets (PPP_IP is used for TYPE_IP packets).
 *
 * RFC 1144 implies that, on the wire, the packet type is *not* needed
 * for PPP, as different PPP protocol types can be used; it only needs
 * to be put on the wire for SLIP.
 *
 * It also indicates that, for compressed SLIP:
 *
 *	If the COMPRESSED_TCP bit is set in the first byte, it's
 *	a COMPRESSED_TCP packet; that byte is the change byte, and
 *	the COMPRESSED_TCP bit, 0x80, isn't used in the change byte.
 *
 *	If the upper 4 bits of the first byte are 7, it's an
 *	UNCOMPRESSED_TCP packet; that byte is the first byte of
 *	the UNCOMPRESSED_TCP modified IP header, with a connection
 *	number in the protocol field, and with the version field
 *	being 7, not 4.
 *
 *	Otherwise, the packet is an IPv4 packet (where the upper 4 bits
 *	of the packet are 4).
 *
 * So this routine looks as if it's sort-of intended to handle
 * compressed SLIP, although it doesn't handle UNCOMPRESSED_TCP
 * correctly for that (it doesn't fix the version number and doesn't
 * do anything to the protocol field), and doesn't check for COMPRESSED_TCP
 * packets correctly for that (you only check the first bit - see
 * B.1 in RFC 1144).
 *
 * But it's called for BSD/OS PPP, not SLIP - perhaps BSD/OS does weird
 * things with the headers?
 *
 * Without a BSD/OS VJC-compressed PPP trace, or knowledge of what the
 * BSD/OS VJC code does, we can't say what's the case.
 *
 * We therefore leave "proto" - which is the PPP protocol type - in place,
 * *not* marked as unused, for now, so that GCC warnings about the
 * unused argument remind us that we should fix this some day.
 */
int
vjc_print(register const char *bp, u_short proto)
{
	int i;

	switch (bp[0] & 0xf0) {
	case TYPE_IP:
		if (eflag)
			printf("(vjc type=IP) ");
		return PPP_IP;
	case TYPE_UNCOMPRESSED_TCP:
		if (eflag)
			printf("(vjc type=raw TCP) ");
		return PPP_IP;
	case TYPE_COMPRESSED_TCP:
		if (eflag)
			printf("(vjc type=compressed TCP) ");
		for (i = 0; i < 8; i++) {
			if (bp[1] & (0x80 >> i))
				printf("%c", "?CI?SAWU"[i]);
		}
		if (bp[1])
			printf(" ");
		printf("C=0x%02x ", bp[2]);
		printf("sum=0x%04x ", *(u_short *)&bp[3]);
		return -1;
	case TYPE_ERROR:
		if (eflag)
			printf("(vjc type=error) ");
		return -1;
	default:
		if (eflag)
			printf("(vjc type=0x%02x) ", bp[0] & 0xf0);
		return -1;
	}
}
