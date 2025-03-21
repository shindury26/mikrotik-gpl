/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that: (1) source code
 * distributions retain the above copyright notice and this paragraph
 * in its entirety, and (2) distributions including binary code include
 * the above copyright notice and this paragraph in its entirety in
 * the documentation or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND
 * WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, WITHOUT
 * LIMITATION, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE.
 *
 * Original code by Hannes Gredler (hannes@juniper.net)
 */

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-bfd.c,v 1.3.2.2 2003/11/16 08:51:12 guy Exp $";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <stdio.h>
#include <stdlib.h>

#include "interface.h"
#include "extract.h"
#include "addrtoname.h"

#include "udp.h"

/*
 * Control packet, draft-katz-ward-bfd-01.txt
 *
 *     0                   1                   2                   3
 *     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |Vers |  Diag   |H|D|P|F| Rsvd  |  Detect Mult  |    Length     |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                       My Discriminator                        |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                      Your Discriminator                       |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                    Desired Min TX Interval                    |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                   Required Min RX Interval                    |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                 Required Min Echo RX Interval                 |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */

struct bfd_header_t {
    u_int8_t version_diag;
    u_int8_t flags;
    u_int8_t detect_time_multiplier;
    u_int8_t length;
    u_int8_t my_discriminator[4];
    u_int8_t your_discriminator[4];
    u_int8_t desired_min_tx_interval[4];
    u_int8_t required_min_rx_interval[4];
    u_int8_t required_min_echo_interval[4];
};

#define	BFD_EXTRACT_VERSION(x) (((x)&0xe0)>>5)
#define	BFD_EXTRACT_DIAG(x)     ((x)&0x1f)

static const struct tok bfd_port_values[] = {
    { BFD_CONTROL_PORT, "Control" },
    { BFD_ECHO_PORT,    "Echo" },
    { 0, NULL }
};


static const struct tok bfd_diag_values[] = {
    { 0, "No Diagnostic" },
    { 1, "Control Detection Time Expired" },
    { 2, "Echo Function Failed" },
    { 3, "Neighbor Signaled Session Down" },
    { 4, "Forwarding Plane Reset" },
    { 5, "Path Down" },
    { 6, "Concatenated Path Down" },
    { 7, "Administratively Down" },
    { 0, NULL }
};

static const struct tok bfd_flag_values[] = {
    { 0x80,	"I Hear You" },
    { 0x40,	"Demand" },
    { 0x20,	"Poll" },
    { 0x10,	"Final" },
    { 0x08,	"Reserved" },
    { 0x04,	"Reserved" },
    { 0x02,	"Reserved" },
    { 0x01,	"Reserved" },
    { 0, NULL }
};

void
bfd_print(register const u_char *pptr, register u_int len, register u_int port)
{
        const struct bfd_header_t *bfd_header;

        bfd_header = (const struct bfd_header_t *)pptr;
        TCHECK(*bfd_header);

        switch (port) {

        case BFD_CONTROL_PORT:
            if (vflag < 1 )
            {
                printf("BFDv%u, %s, Flags: [%s], length: %u",
                       BFD_EXTRACT_VERSION(bfd_header->version_diag),
                       tok2str(bfd_port_values, "unknown (%u)", port),
                       bittok2str(bfd_flag_values, "none", bfd_header->flags),
                       len);
                return;
            }
            
            printf("BFDv%u, length: %u\n\t%s, Flags: [%s], Diagnostic: %s (0x%02x)",
                   BFD_EXTRACT_VERSION(bfd_header->version_diag),
                   len,
                   tok2str(bfd_port_values, "unknown (%u)", port),
                   bittok2str(bfd_flag_values, "none", bfd_header->flags),
                   tok2str(bfd_diag_values,"unknown",BFD_EXTRACT_DIAG(bfd_header->version_diag)),
               BFD_EXTRACT_DIAG(bfd_header->version_diag));
            
            printf("\n\tDetection Timer Multiplier: %u (%u ms Detection time), BFD Length: %u",
                   bfd_header->detect_time_multiplier,
                   bfd_header->detect_time_multiplier * EXTRACT_32BITS(bfd_header->desired_min_tx_interval)/1000,
                   bfd_header->length);


            printf("\n\tMy Discriminator: 0x%08x", EXTRACT_32BITS(bfd_header->my_discriminator));
            printf(", Your Discriminator: 0x%08x", EXTRACT_32BITS(bfd_header->your_discriminator));
            printf("\n\t  Desired min Tx Interval:    %4u ms", EXTRACT_32BITS(bfd_header->desired_min_tx_interval)/1000);
            printf("\n\t  Required min Rx Interval:   %4u ms", EXTRACT_32BITS(bfd_header->required_min_rx_interval)/1000);
            printf("\n\t  Required min Echo Interval: %4u ms", EXTRACT_32BITS(bfd_header->required_min_echo_interval)/1000);
            break;

        case BFD_ECHO_PORT: /* not yet supported - fall through */

        default:
            printf("BFD, %s, length: %u",
                   tok2str(bfd_port_values, "unknown (%u)", port),
                   len);
            if (vflag >= 1) {
                if(!print_unknown_data(pptr,"\n\t",len))
                    return;
            }
            break;
        }
        return;

trunc:
        printf("[|BFD]");
}
