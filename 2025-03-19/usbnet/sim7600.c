/*
 * Copyright (c) 2016 Xiaobin Wang <xiaobin.wang@sim.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include "usbnet.h"

/* SIMCOM devices combine the "control" and "data" functions into a
 * single interface with all three endpoints: interrupt + bulk in and
 * out
 */
static int simcom_wwan_bind(struct usbnet *dev, struct usb_interface *intf)
{
    int rv = -EINVAL;

    /* ignore any interface with additional descriptors */
    if (intf->cur_altsetting->extralen)
        goto err;

    /* Some makes devices where the interface descriptors and endpoint
     * configurations of two or more interfaces are identical, even
     * though the functions are completely different.  If set, then
     * driver_info->data is a bitmap of acceptable interface numbers
     * allowing us to bind to one such interface without binding to
     * all of them
    if (dev->driver_info->data &&
            intf->cur_altsetting->desc.bInterfaceNumber != dev->driver_info->data) {
        rv = -ENODEV;
        goto err;
    }
    */

    /* collect all three endpoints */
    rv = usbnet_get_endpoints(dev, intf);
    if (rv < 0)
        goto err;

    /* require interrupt endpoint for subdriver */
    if (!dev->status) {
        rv = -EINVAL;
        goto err;
    }

    /* can't let usbnet use the interrupt endpoint */
    dev->status = NULL;

    printk("simcom usbnet bind\n");

    /*
     * SIMCOM SIM7600 only support the RAW_IP mode, so the host net driver would
     * remove the arp so the packets can transmit to the modem
     */
    dev->net->flags |= IFF_NOARP;

    /* make MAC addr easily distinguishable from an IP header */
    if ((dev->net->dev_addr[0] & 0xd0) == 0x40) {
        dev->net->dev_addr[0] |= 0x02;	/* set local assignment bit */
        dev->net->dev_addr[0] &= 0xbf;	/* clear "IP" bit */
    }

    /*
     * SIMCOM SIM7600 need set line state
     */
    usb_control_msg(
            interface_to_usbdev(intf),
            usb_sndctrlpipe(interface_to_usbdev(intf), 0),
            0x22, //USB_CDC_REQ_SET_CONTROL_LINE_STATE
            0x21, //USB_DIR_OUT | USB_TYPE_CLASS| USB_RECIP_INTERFACE
            1, //line state 1
            intf->cur_altsetting->desc.bInterfaceNumber,
            NULL,0,100);

err:
    return rv;
}

static int simcom_wwan_tx_fixup(struct usbnet *dev, struct fp_buf *fpb, gfp_t flags)
{
    // skip ethernet header
    fpb_pull(fpb, ETH_HLEN);
    return 1;
}

static int simcom_wwan_rx_fixup(struct usbnet *dev, struct fp_buf *buf)
{
    __be16 proto;

    /* This check is no longer done by usbnet */
    if (fpb_len(buf) < dev->net->hard_header_len) return 0;

    switch (fpb_data(buf)[0] & 0xf0) {
        case 0x40:
            proto = htons(ETH_P_IP);
            break;
        case 0x60:
            proto = htons(ETH_P_IPV6);
            break;
        case 0x00:
            if (is_multicast_ether_addr(fpb_data(buf)))
                return 1;
            /* possibly bogus destination - rewrite mac just in case */
            memset(fpb_data(buf), 0, ETH_ALEN);
            goto fix_dest;
        default:
            /* pass along other packets without modifications */
            return 1;
    }

    if (fpb_headroom(buf) < ETH_HLEN) return 0;

    fpb_push(buf, ETH_HLEN);
    memset(fpb_data(buf), 0, ETH_ALEN);
    ((struct ethhdr*) fpb_data(buf))->h_proto = proto;
    memset(((struct ethhdr*) fpb_data(buf))->h_source, 0, ETH_ALEN);

fix_dest:
    memcpy(((struct ethhdr*) fpb_data(buf))->h_dest, dev->net->dev_addr, ETH_ALEN);
    return 1;
}

const struct driver_info	simcom_info = {
    .description	= "SIMCOM device",
    .flags		= FLAG_WWAN,
    .bind_init		= simcom_wwan_bind,
    .rx_fixup           = simcom_wwan_rx_fixup,
    .tx_fixup           = simcom_wwan_tx_fixup,
};
