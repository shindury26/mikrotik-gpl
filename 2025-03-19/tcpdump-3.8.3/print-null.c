/*
 * Copyright (c) 1991, 1993, 1994, 1995, 1996, 1997
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

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-null.c,v 1.49.2.2 2003/11/16 08:51:36 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <pcap.h>
#include <stdio.h>
#include <string.h>

#include "interface.h"
#include "addrtoname.h"

#include "ip.h"
#ifdef INET6
#include "ip6.h"
#endif

/*
 * The DLT_NULL packet header is 4 bytes long. It contains a host-byte-order
 * 32-bit integer that specifies the family, e.g. AF_INET.
 *
 * Note here that "host" refers to the host on which the packets were
 * captured; that isn't necessarily *this* host.
 *
 * The OpenBSD DLT_LOOP packet header is the same, except that the integer
 * is in network byte order.
 */
#define	NULL_HDRLEN 4

/*
 * BSD AF_ values.
 *
 * Unfortunately, the BSDs don't all use the same value for AF_INET6,
 * so, because we want to be able to read captures from all of the BSDs,
 * we check for all of them.
 */
#define BSD_AF_INET		2
#define BSD_AF_NS		6		/* XEROX NS protocols */
#define BSD_AF_ISO		7
#define BSD_AF_APPLETALK	16
#define BSD_AF_IPX		23
#define BSD_AF_INET6_BSD	24	/* OpenBSD (and probably NetBSD), BSD/OS */
#define BSD_AF_INET6_FREEBSD	28
#define BSD_AF_INET6_DARWIN	30

static void
null_print(u_int family, u_int length)
{
	if (nflag)
		printf("AF %u ", family);
	else {
		switch (family) {

		case BSD_AF_INET:
			printf("ip ");
			break;

#ifdef INET6
		case BSD_AF_INET6_BSD:
		case BSD_AF_INET6_FREEBSD:
		case BSD_AF_INET6_DARWIN:
			printf("ip6 ");
			break;
#endif

		case BSD_AF_NS:
			printf("ns ");
			break;

		case BSD_AF_ISO:
			printf("osi ");
			break;

		case BSD_AF_APPLETALK:
			printf("atalk ");
			break;

		case BSD_AF_IPX:
			printf("ipx ");
			break;

		default:
			printf("AF %u ", family);
			break;
		}
	}
	printf("%d: ", length);
}

/*
 * Byte-swap a 32-bit number.
 * ("htonl()" or "ntohl()" won't work - we want to byte-swap even on
 * big-endian platforms.)
 */
#define	SWAPLONG(y) \
((((y)&0xff)<<24) | (((y)&0xff00)<<8) | (((y)&0xff0000)>>8) | (((y)>>24)&0xff))

/*
 * This is the top level routine of the printer.  'p' points
 * to the ether header of the packet, 'h->ts' is the timestamp,
 * 'h->length' is the length of the packet off the wire, and 'h->caplen'
 * is the number of bytes actually captured.
 */
u_int
null_if_print(const struct pcap_pkthdr *h, const u_char *p)
{
	u_int length = h->len;
	u_int caplen = h->caplen;
	u_int family;

	if (caplen < NULL_HDRLEN) {
		printf("[|null]");
		return (NULL_HDRLEN);
	}

	memcpy((char *)&family, (char *)p, sizeof(family));

	/*
	 * This isn't necessarily in our host byte order; if this is
	 * a DLT_LOOP capture, it's in network byte order, and if
	 * this is a DLT_NULL capture from a machine with the opposite
	 * byte-order, it's in the opposite byte order from ours.
	 *
	 * If the upper 16 bits aren't all zero, assume it's byte-swapped.
	 */
	if ((family & 0xFFFF0000) != 0)
		family = SWAPLONG(family);

	length -= NULL_HDRLEN;
	caplen -= NULL_HDRLEN;
	p += NULL_HDRLEN;

	if (eflag)
		null_print(family, length);

	switch (family) {

	case BSD_AF_INET:
		ip_print(p, length);
		break;

#ifdef INET6
	case BSD_AF_INET6_BSD:
	case BSD_AF_INET6_FREEBSD:
	case BSD_AF_INET6_DARWIN:
		ip6_print(p, length);
		break;
#endif

	case BSD_AF_ISO:
		isoclns_print(p, length, caplen);
		break;

	case BSD_AF_APPLETALK:
		atalk_print(p, length);
		break;

	case BSD_AF_IPX:
		ipx_print(p, length);
		break;

	default:
		/* unknown AF_ value */
		if (!eflag)
			null_print(family, length + NULL_HDRLEN);
		if (!xflag && !qflag)
			default_print(p, caplen);
	}

	return (NULL_HDRLEN);
}

