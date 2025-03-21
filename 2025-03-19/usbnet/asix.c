/*
 * ASIX AX8817X based USB 2.0 Ethernet Devices
 * Copyright (C) 2003-2006 David Hollis <dhollis@davehollis.com>
 * Copyright (C) 2005 Phil Chang <pchang23@sbcglobal.net>
 * Copyright (C) 2006 James Painter <jamie.painter@iname.com>
 * Copyright (c) 2002-2003 TiVo Inc.
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

// #define	DEBUG			// error path messages, extra info
// #define	VERBOSE			// more; success messages

#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/packet_hook.h>
#include "usbnet.h"
#include <linux/slab.h>

#define DRIVER_VERSION "22-Dec-2011"
#define DRIVER_NAME "asix"

/* ASIX AX8817X based USB 2.0 Ethernet Devices */

#define AX_CMD_SET_SW_MII		0x06
#define AX_CMD_READ_MII_REG		0x07
#define AX_CMD_WRITE_MII_REG		0x08
#define AX_CMD_SET_HW_MII		0x0a
#define AX_CMD_READ_EEPROM		0x0b
#define AX_CMD_WRITE_EEPROM		0x0c
#define AX_CMD_WRITE_ENABLE		0x0d
#define AX_CMD_WRITE_DISABLE		0x0e
#define AX_CMD_READ_RX_CTL		0x0f
#define AX_CMD_WRITE_RX_CTL		0x10
#define AX_CMD_READ_IPG012		0x11
#define AX_CMD_WRITE_IPG0		0x12
#define AX_CMD_WRITE_IPG1		0x13
#define AX_CMD_READ_NODE_ID		0x13
#define AX_CMD_WRITE_NODE_ID		0x14
#define AX_CMD_WRITE_IPG2		0x14
#define AX_CMD_WRITE_MULTI_FILTER	0x16
#define AX88172_CMD_READ_NODE_ID	0x17
#define AX_CMD_READ_PHY_ID		0x19
#define AX_CMD_READ_MEDIUM_STATUS	0x1a
#define AX_CMD_WRITE_MEDIUM_MODE	0x1b
#define AX_CMD_READ_MONITOR_MODE	0x1c
#define AX_CMD_WRITE_MONITOR_MODE	0x1d
#define AX_CMD_READ_GPIOS		0x1e
#define AX_CMD_WRITE_GPIOS		0x1f
#define AX_CMD_SW_RESET			0x20
#define AX_CMD_SW_PHY_STATUS		0x21
#define AX_CMD_SW_PHY_SELECT		0x22

#define AX_MONITOR_MODE			0x01
#define AX_MONITOR_LINK			0x02
#define AX_MONITOR_MAGIC		0x04
#define AX_MONITOR_HSFS			0x10

/* AX88172 Medium Status Register values */
#define AX88172_MEDIUM_FD		0x02
#define AX88172_MEDIUM_TX		0x04
#define AX88172_MEDIUM_FC		0x10
#define AX88172_MEDIUM_DEFAULT \
		( AX88172_MEDIUM_FD | AX88172_MEDIUM_TX | AX88172_MEDIUM_FC )

#define AX_MCAST_FILTER_SIZE		8
#define AX_MAX_MCAST			64

#define AX_SWRESET_CLEAR		0x00
#define AX_SWRESET_RR			0x01
#define AX_SWRESET_RT			0x02
#define AX_SWRESET_PRTE			0x04
#define AX_SWRESET_PRL			0x08
#define AX_SWRESET_BZ			0x10
#define AX_SWRESET_IPRL			0x20
#define AX_SWRESET_IPPD			0x40

#define AX88772_IPG0_DEFAULT		0x15
#define AX88772_IPG1_DEFAULT		0x0c
#define AX88772_IPG2_DEFAULT		0x12

/* AX88772 & AX88178 Medium Mode Register */
#define AX_MEDIUM_PF		0x0080
#define AX_MEDIUM_JFE		0x0040
#define AX_MEDIUM_TFC		0x0020
#define AX_MEDIUM_RFC		0x0010
#define AX_MEDIUM_ENCK		0x0008
#define AX_MEDIUM_AC		0x0004
#define AX_MEDIUM_FD		0x0002
#define AX_MEDIUM_GM		0x0001
#define AX_MEDIUM_SM		0x1000
#define AX_MEDIUM_SBP		0x0800
#define AX_MEDIUM_PS		0x0200
#define AX_MEDIUM_RE		0x0100

#define AX88178_MEDIUM_DEFAULT	\
	(AX_MEDIUM_PS | AX_MEDIUM_FD | AX_MEDIUM_AC | \
	 AX_MEDIUM_RFC | AX_MEDIUM_TFC | AX_MEDIUM_JFE | \
	 AX_MEDIUM_RE)

#define AX88772_MEDIUM_DEFAULT	\
	(AX_MEDIUM_FD | AX_MEDIUM_RFC | \
	 AX_MEDIUM_TFC | AX_MEDIUM_PS | \
	 AX_MEDIUM_AC | AX_MEDIUM_RE)

/* AX88772 & AX88178 RX_CTL values */
#define AX_RX_CTL_SO		0x0080
#define AX_RX_CTL_AP		0x0020
#define AX_RX_CTL_AM		0x0010
#define AX_RX_CTL_AB		0x0008
#define AX_RX_CTL_SEP		0x0004
#define AX_RX_CTL_AMALL		0x0002
#define AX_RX_CTL_PRO		0x0001
#define AX_RX_CTL_MFB_2048	0x0000
#define AX_RX_CTL_MFB_4096	0x0100
#define AX_RX_CTL_MFB_8192	0x0200
#define AX_RX_CTL_MFB_16384	0x0300

#define AX_DEFAULT_RX_CTL	(AX_RX_CTL_SO | AX_RX_CTL_AB)

/* GPIO 0 .. 2 toggles */
#define AX_GPIO_GPO0EN		0x01	/* GPIO0 Output enable */
#define AX_GPIO_GPO_0		0x02	/* GPIO0 Output value */
#define AX_GPIO_GPO1EN		0x04	/* GPIO1 Output enable */
#define AX_GPIO_GPO_1		0x08	/* GPIO1 Output value */
#define AX_GPIO_GPO2EN		0x10	/* GPIO2 Output enable */
#define AX_GPIO_GPO_2		0x20	/* GPIO2 Output value */
#define AX_GPIO_RESERVED	0x40	/* Reserved */
#define AX_GPIO_RSE		0x80	/* Reload serial EEPROM */

#define AX_EEPROM_MAGIC		0xdeadbeef
#define AX88172_EEPROM_LEN	0x40
#define AX88772_EEPROM_LEN	0xff

#define PHY_MODE_MARVELL	0x0000
#define MII_MARVELL_LED_CTRL	0x0018
#define MII_MARVELL_STATUS	0x001b
#define MII_MARVELL_CTRL	0x0014

#define MARVELL_LED_MANUAL	0x0019

#define MARVELL_STATUS_HWCFG	0x0004

#define MARVELL_CTRL_TXDELAY	0x0002
#define MARVELL_CTRL_RXDELAY	0x0080

#define	PHY_MODE_RTL8211CL	0x000C

#ifdef DEBUG
#define dbg(format, arg...)						\
	printk(KERN_DEBUG "%s: " format "\n", __FILE__, ##arg)
#else
#define dbg(format, arg...)						\
do {									\
	if (0)								\
		printk(KERN_DEBUG "%s: " format "\n", __FILE__, ##arg); \
} while (0)
#endif

/* This structure cannot exceed sizeof(unsigned long [5]) AKA 20 bytes */
struct asix_data {
	u8 multi_filter[AX_MCAST_FILTER_SIZE];
	u8 mac_addr[ETH_ALEN];
	u8 phymode;
	u8 ledmode;
	u8 eeprom_len;
};

struct ax88172_int_data {
	__le16 res1;
	u8 link;
	__le16 res2;
	u8 status;
	__le16 res3;
} __packed;

static int asix_read_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			    u16 size, void *data)
{
	void *buf;
	int err = -ENOMEM;

	netdev_dbg(dev->net, "asix_read_cmd() cmd=0x%02x value=0x%04x index=0x%04x size=%d\n",
		   cmd, value, index, size);

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		goto out;

	err = usb_control_msg(
		dev->udev,
		usb_rcvctrlpipe(dev->udev, 0),
		cmd,
		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value,
		index,
		buf,
		size,
		USB_CTRL_GET_TIMEOUT);
	if (err == size)
		memcpy(data, buf, size);
	else if (err >= 0)
		err = -EINVAL;
	kfree(buf);

out:
	return err;
}

static int asix_write_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			     u16 size, void *data)
{
	void *buf = NULL;
	int err = -ENOMEM;

	netdev_dbg(dev->net, "asix_write_cmd() cmd=0x%02x value=0x%04x index=0x%04x size=%d\n",
		   cmd, value, index, size);

	if (data) {
		buf = kmemdup(data, size, GFP_KERNEL);
		if (!buf)
			goto out;
	}

	err = usb_control_msg(
		dev->udev,
		usb_sndctrlpipe(dev->udev, 0),
		cmd,
		USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value,
		index,
		buf,
		size,
		USB_CTRL_SET_TIMEOUT);
	kfree(buf);

out:
	return err;
}

static void asix_async_cmd_callback(struct urb *urb)
{
	struct usb_ctrlrequest *req = (struct usb_ctrlrequest *)urb->context;
	int status = urb->status;

	if (status < 0)
		printk(KERN_DEBUG "asix_async_cmd_callback() failed with %d",
			status);

	kfree(req);
	usb_free_urb(urb);
}

static void
asix_write_cmd_async(struct usbnet *dev, u8 cmd, u16 value, u16 index,
				    u16 size, void *data)
{
	struct usb_ctrlrequest *req;
	int status;
	struct urb *urb;

	netdev_dbg(dev->net, "asix_write_cmd_async() cmd=0x%02x value=0x%04x index=0x%04x size=%d\n",
		   cmd, value, index, size);

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		netdev_err(dev->net, "Error allocating URB in write_cmd_async!\n");
		return;
	}

	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC);
	if (!req) {
		netdev_err(dev->net, "Failed to allocate memory for control request\n");
		usb_free_urb(urb);
		return;
	}

	req->bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	req->bRequest = cmd;
	req->wValue = cpu_to_le16(value);
	req->wIndex = cpu_to_le16(index);
	req->wLength = cpu_to_le16(size);

	usb_fill_control_urb(urb, dev->udev,
			     usb_sndctrlpipe(dev->udev, 0),
			     (void *)req, data, size,
			     asix_async_cmd_callback, req);

	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status < 0) {
		netdev_err(dev->net, "Error submitting the control message: status=%d\n",
			   status);
		kfree(req);
		usb_free_urb(urb);
	}
}

// from v7
#if 1
static int asix_rx_fixup(struct usbnet *dev, struct fp_buf *buf)
{
	int offset = 0;
	u16 size;
        struct asix_rx_fixup_info *rx = &dev->driver_info->fixup;
        bool alloc = false;

	/* When an Ethernet frame spans multiple URB socket buffers,
	 * do a sanity test for the Data header synchronisation.
	 * Attempt to detect the situation of the previous socket buffer having
	 * been truncated or a socket buffer was missing. These situations
	 * cause a discontinuity in the data stream and therefore need to avoid
	 * appending bad data to the end of the current netdev socket buffer.
	 * Also avoid unnecessarily discarding a good current netdev socket
	 * buffer.
	 */
	if (rx->remaining && (rx->remaining + sizeof(u32) <= fpb_len(buf))) {
		offset = ((rx->remaining + 1) & 0xfffe);
		rx->header = get_unaligned_le32(fpb_data(buf) + offset);
		offset = 0;

		size = (u16)(rx->header & 0x7ff);
		if (size != ((~rx->header >> 16) & 0x7ff)) {
			netdev_err(dev->net, "asix_rx_fixup() Data Header synchronisation was lost, remaining %d\n",
				   rx->remaining);
			if (rx->skb_bin) {
                                __skb_bin_put_buffer_noinvalidate(rx->skb_bin);
				rx->skb_bin = NULL;
                                rx->skb_bin_len = 0;
				/* Discard the incomplete netdev Ethernet frame
				 * and assume the Data header is at the start of
				 * the current URB socket buffer.
				 */
			}
			rx->remaining = 0;
		}
	}

	while (offset + sizeof(u16) <= fpb_len(buf)) {
		u16 copy_length;

		if (!rx->remaining) {
			if (fpb_len(buf) - offset == sizeof(u16)) {
				rx->header = get_unaligned_le16(
						fpb_data(buf) + offset);
				rx->split_head = true;
				offset += sizeof(u16);
				break;
			}

			if (rx->split_head == true) {
				rx->header |= (get_unaligned_le16(
						fpb_data(buf) + offset) << 16);
				rx->split_head = false;
				offset += sizeof(u16);
			} else {
				rx->header = get_unaligned_le32(fpb_data(buf) +
								offset);
				offset += sizeof(u32);
			}

			/* take frame length from Data header 32-bit word */
			size = (u16)(rx->header & 0x7ff);
			if (size != ((~rx->header >> 16) & 0x7ff)) {
				netdev_err(dev->net, "asix_rx_fixup() Bad Header Length 0x%x, offset %d\n",
					   rx->header, offset);
				return 0;
			}
			if (size > dev->net->mtu + ETH_HLEN + VLAN_HLEN) {
				netdev_dbg(dev->net, "asix_rx_fixup() Bad RX Length %d\n",
					   size);
				return 0;
			}

			/* Sometimes may fail to get a netdev socket buffer but
			 * continue to process the URB socket buffer so that
			 * synchronisation of the Ethernet frame Data header
			 * word is maintained.
			 */
			rx->remaining = size;
                        alloc = true;

		} else {
                        alloc = false;
                }

		if (rx->remaining > fpb_len(buf) - offset) {
			copy_length = fpb_len(buf) - offset;
			rx->remaining -= copy_length;
		} else {
			copy_length = rx->remaining;
			rx->remaining = 0;
		}

                if (likely(alloc)) {
                    if (fpb_len(buf) == (offset + copy_length) && !rx->remaining) {
                        // got only one whole packet
                        fpb_pull(buf, offset);
                        return 1;
                    }

                    rx->skb_bin = __skb_bin_get_buffer();
                }

		if (rx->skb_bin) {
			memcpy((char*) rx->skb_bin + NET_SKB_PAD + rx->skb_bin_len,
                                fpb_data(buf) + offset, copy_length);
                        rx->skb_bin_len += copy_length;
			if (!rx->remaining) {
				usbnet_skb_return(dev,
                                    fpb_build((char*) rx->skb_bin,
                                            NET_SKB_PAD, rx->skb_bin_len));
                                rx->skb_bin = NULL;
                                rx->skb_bin_len = 0;
                        }
		}

		offset += (copy_length + 1) & 0xfffe;
	}

	if (fpb_len(buf) != offset) {
		netdev_err(dev->net, "asix_rx_fixup() Bad SKB Length %d, %d\n",
			   fpb_len(buf), offset);
		return 0;
	}

        fpb_pull(buf, offset);
	return 1;
}

#else
static int asix_rx_fixup(struct usbnet *dev, struct fp_buf *buf)
{
	u8  *head;
	u32  header;
	unsigned char *packet;
	struct fp_buf *ax_skb;
	u16 size;

	head = (u8 *) fpb_data(buf);
	memcpy(&header, head, sizeof(header));
	le32_to_cpus(&header);

	packet = head + sizeof(header);
        fpb_pull(buf, 4);

	while (fpb_len(buf) > 0) {
		/* get the packet length */
		size = (u16) (header & 0x000007ff);

		if ((header & 0x07ff) != ((~header >> 16) & 0x07ff)) {
			netdev_err(dev->net, "asix_rx_fixup() Bad Header Length\n");
                        return 0;
                }

                // last packet
		if (fpb_len(buf) - ((size + 1) & 0xfffe) == 0) {

			u8 alignment = (unsigned long) packet & 0x3;
			if (alignment != 0x2) {
				/*
				 * not 16bit aligned so use the room provided by
				 * the 32 bit header to align the data
				 *
				 * note we want 16bit alignment as MAC header is
				 * 14bytes thus ip header will be aligned on
				 * 32bit boundary so accessing ipheader elements
				 * using a cast to struct ip header wont cause
				 * an unaligned accesses.
				 */
				u8 realignment = (alignment + 2) & 0x3;
				memmove(fpb_data(buf) - realignment,
					fpb_data(buf),
					size);
				fpb_push(buf, realignment);
                                fpb_len_set(buf, fpb_len(buf) - realignment);
			}
			return 2;
		}

		if (size > dev->net->mtu + ETH_HLEN) {
			netdev_err(dev->net, "asix_rx_fixup() Bad RX Length %d\n",
				   size);
			return 0;
		}
                ax_skb = __fpb_alloc(size);
		if (ax_skb) {
                        u8 realignment = 0;
			u8 alignment = (unsigned long)packet & 0x3;
			if (alignment != 0x2) {
				/*
				 * not 16bit aligned use the room provided by
				 * the 32 bit header to align the data
				 */
				realignment = (alignment + 2) & 0x3;
			}
                        memcpy(fpb_data(ax_skb), packet - realignment, size);
			usbnet_skb_return(dev, ax_skb);
		} else {
			return 0;
		}

                fpb_pull(buf, (size + 1) & 0xfffe);
		if (fpb_len(buf) < sizeof(header))
			break;

		head = (u8 *) fpb_data(buf);
		memcpy(&header, head, sizeof(header));
		le32_to_cpus(&header);

		packet = head + sizeof(header);
                fpb_pull(buf, sizeof(header));
	}

	if (!fpb_len(buf)) {
		netdev_err(dev->net, "asix_rx_fixup() Bad SKB Length %d\n",
			   fpb_len(buf));
		return 0;
	}
	return 1;
}
#endif

static int asix_tx_fixup(struct usbnet *dev, struct fp_buf *fpb,
					gfp_t flags)
{
	u32 packet_len;
	u32 padbytes = 0xffff0000;
	int padlen = ((fpb_len(fpb) + 4) % 512) ? 0 : 4;

	if (fpb_headroom(fpb) < 4 || fpb_tailroom(fpb) < padlen) return 0;

	fpb_push(fpb, 4);
	packet_len = (((fpb_len(fpb) - 4) ^ 0x0000ffff) << 16) + (fpb_len(fpb) - 4);
	cpu_to_le32s(&packet_len);
        memcpy(fpb_data(fpb), &packet_len, sizeof(packet_len));

	if ((fpb_len(fpb) % 512) == 0) {
		cpu_to_le32s(&padbytes);
		memcpy(fpb_data(fpb) + fpb_len(fpb), &padbytes, sizeof(padbytes));
		fpb_put(fpb, sizeof(padbytes));
	}
	return 1;
}

static void asix_status(struct usbnet *dev, struct urb *urb)
{
	struct ax88172_int_data *event;
	int link;

	if (urb->actual_length < 8)
		return;

	event = urb->transfer_buffer;
	link = event->link & 0x01;
	if (netif_carrier_ok(dev->net) != link) {
		if (link) {
			netif_carrier_on(dev->net);
			usbnet_defer_kevent (dev, EVENT_LINK_RESET );
		} else
			netif_carrier_off(dev->net);
		netdev_dbg(dev->net, "Link Status is: %d\n", link);
	}
}

static inline int asix_set_sw_mii(struct usbnet *dev)
{
	int ret;
	ret = asix_write_cmd(dev, AX_CMD_SET_SW_MII, 0x0000, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to enable software MII access\n");
	return ret;
}

static inline int asix_set_hw_mii(struct usbnet *dev)
{
	int ret;
	ret = asix_write_cmd(dev, AX_CMD_SET_HW_MII, 0x0000, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to enable hardware MII access\n");
	return ret;
}

static inline int asix_get_phy_addr(struct usbnet *dev)
{
	u8 buf[2];
	int ret = asix_read_cmd(dev, AX_CMD_READ_PHY_ID, 0, 0, 2, buf);

	netdev_dbg(dev->net, "asix_get_phy_addr()\n");

	if (ret < 0) {
		netdev_err(dev->net, "Error reading PHYID register: %02x\n", ret);
		goto out;
	}
	netdev_dbg(dev->net, "asix_get_phy_addr() returning 0x%04x\n",
		   *((__le16 *)buf));
	ret = buf[1];

out:
	return ret;
}

static int asix_sw_reset(struct usbnet *dev, u8 flags)
{
	int ret;

        ret = asix_write_cmd(dev, AX_CMD_SW_RESET, flags, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to send software reset: %02x\n", ret);

	return ret;
}

static u16 asix_read_rx_ctl(struct usbnet *dev)
{
	__le16 v;
	int ret = asix_read_cmd(dev, AX_CMD_READ_RX_CTL, 0, 0, 2, &v);

	if (ret < 0) {
		netdev_err(dev->net, "Error reading RX_CTL register: %02x\n", ret);
		goto out;
	}
	ret = le16_to_cpu(v);
out:
	return ret;
}

static int asix_write_rx_ctl(struct usbnet *dev, u16 mode)
{
	int ret;

	netdev_dbg(dev->net, "asix_write_rx_ctl() - mode = 0x%04x\n", mode);
	ret = asix_write_cmd(dev, AX_CMD_WRITE_RX_CTL, mode, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to write RX_CTL mode to 0x%04x: %02x\n",
			   mode, ret);

	return ret;
}

static u16 asix_read_medium_status(struct usbnet *dev)
{
	__le16 v;
	int ret = asix_read_cmd(dev, AX_CMD_READ_MEDIUM_STATUS, 0, 0, 2, &v);

	if (ret < 0) {
		netdev_err(dev->net, "Error reading Medium Status register: %02x\n",
			   ret);
		return ret;	/* TODO: callers not checking for error ret */
	}

	return le16_to_cpu(v);

}

static int asix_write_medium_mode(struct usbnet *dev, u16 mode)
{
	int ret;

	netdev_dbg(dev->net, "asix_write_medium_mode() - mode = 0x%04x\n", mode);
	ret = asix_write_cmd(dev, AX_CMD_WRITE_MEDIUM_MODE, mode, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to write Medium Mode mode to 0x%04x: %02x\n",
			   mode, ret);

	return ret;
}

static int asix_write_gpio(struct usbnet *dev, u16 value, int sleep)
{
	int ret;

	netdev_dbg(dev->net, "asix_write_gpio() - value = 0x%04x\n", value);
	ret = asix_write_cmd(dev, AX_CMD_WRITE_GPIOS, value, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to write GPIO value 0x%04x: %02x\n",
			   value, ret);

	if (sleep)
		msleep(sleep);

	return ret;
}

/*
 * AX88772 & AX88178 have a 16-bit RX_CTL value
 */
static void asix_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;
	u16 rx_ctl = AX_DEFAULT_RX_CTL;

	if (net->flags & IFF_PROMISC) {
		rx_ctl |= AX_RX_CTL_PRO;
	} else if (net->flags & IFF_ALLMULTI ||
		   netdev_mc_count(net) > AX_MAX_MCAST) {
		rx_ctl |= AX_RX_CTL_AMALL;
	} else if (netdev_mc_empty(net)) {
		/* just broadcast and directed */
	} else {
		/* We use the 20 byte dev->data
		 * for our 8 byte filter buffer
		 * to avoid allocating memory that
		 * is tricky to free later */
		struct netdev_hw_addr *ha;
		u32 crc_bits;

		memset(data->multi_filter, 0, AX_MCAST_FILTER_SIZE);

		/* Build the multicast hash filter. */
		netdev_for_each_mc_addr(ha, net) {
			crc_bits = ether_crc(ETH_ALEN, ha->addr) >> 26;
			data->multi_filter[crc_bits >> 3] |=
			    1 << (crc_bits & 7);
		}

		asix_write_cmd_async(dev, AX_CMD_WRITE_MULTI_FILTER, 0, 0,
				   AX_MCAST_FILTER_SIZE, data->multi_filter);

		rx_ctl |= AX_RX_CTL_AM;
	}

	asix_write_cmd_async(dev, AX_CMD_WRITE_RX_CTL, rx_ctl, 0, 0, NULL);
}

static int asix_mdio_read(struct net_device *netdev, int phy_id, int loc)
{
	struct usbnet *dev = netdev_priv(netdev);
	__le16 res;

	mutex_lock(&dev->phy_mutex);
	asix_set_sw_mii(dev);
	asix_read_cmd(dev, AX_CMD_READ_MII_REG, phy_id,
				(__u16)loc, 2, &res);
	asix_set_hw_mii(dev);
	mutex_unlock(&dev->phy_mutex);

	netdev_dbg(dev->net, "asix_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x\n",
		   phy_id, loc, le16_to_cpu(res));

	return le16_to_cpu(res);
}

static void
asix_mdio_write(struct net_device *netdev, int phy_id, int loc, int val)
{
	struct usbnet *dev = netdev_priv(netdev);
	__le16 res = cpu_to_le16(val);

	netdev_dbg(dev->net, "asix_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x\n",
		   phy_id, loc, val);
	mutex_lock(&dev->phy_mutex);
	asix_set_sw_mii(dev);
	asix_write_cmd(dev, AX_CMD_WRITE_MII_REG, phy_id, (__u16)loc, 2, &res);
	asix_set_hw_mii(dev);
	mutex_unlock(&dev->phy_mutex);
}

/* Get the PHY Identifier from the PHYSID1 & PHYSID2 MII registers */
static u32 asix_get_phyid(struct usbnet *dev)
{
	int phy_reg;
	u32 phy_id;
	int i;

	/* Poll for the rare case the FW or phy isn't ready yet.  */
	for (i = 0; i < 100; i++) {
		phy_reg = asix_mdio_read(dev->net, dev->mii.phy_id, MII_PHYSID1);
		if (phy_reg != 0 && phy_reg != 0xFFFF)
			break;
		mdelay(1);
	}

	if (phy_reg <= 0 || phy_reg == 0xFFFF)
		return 0;

	phy_id = (phy_reg & 0xffff) << 16;

	phy_reg = asix_mdio_read(dev->net, dev->mii.phy_id, MII_PHYSID2);
	if (phy_reg < 0)
		return 0;

	phy_id |= (phy_reg & 0xffff);

	return phy_id;
}

static void
asix_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 opt;

	if (asix_read_cmd(dev, AX_CMD_READ_MONITOR_MODE, 0, 0, 1, &opt) < 0) {
		wolinfo->supported = 0;
		wolinfo->wolopts = 0;
		return;
	}
	wolinfo->supported = WAKE_PHY | WAKE_MAGIC;
	wolinfo->wolopts = 0;
	if (opt & AX_MONITOR_LINK)
		wolinfo->wolopts |= WAKE_PHY;
	if (opt & AX_MONITOR_MAGIC)
		wolinfo->wolopts |= WAKE_MAGIC;
}

static int
asix_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 opt = 0;

	if (wolinfo->wolopts & WAKE_PHY)
		opt |= AX_MONITOR_LINK;
	if (wolinfo->wolopts & WAKE_MAGIC)
		opt |= AX_MONITOR_MAGIC;

	if (asix_write_cmd(dev, AX_CMD_WRITE_MONITOR_MODE,
			      opt, 0, 0, NULL) < 0)
		return -EINVAL;

	return 0;
}

static int asix_get_eeprom_len(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;

	return data->eeprom_len;
}

static int asix_get_eeprom(struct net_device *net,
			      struct ethtool_eeprom *eeprom, u8 *data)
{
	struct usbnet *dev = netdev_priv(net);
	__le16 *ebuf = (__le16 *)data;
	int i;

	/* Crude hack to ensure that we don't overwrite memory
	 * if an odd length is supplied
	 */
	if (eeprom->len % 2)
		return -EINVAL;

	eeprom->magic = AX_EEPROM_MAGIC;

	/* ax8817x returns 2 bytes from eeprom on read */
	for (i=0; i < eeprom->len / 2; i++) {
		if (asix_read_cmd(dev, AX_CMD_READ_EEPROM,
			eeprom->offset + i, 0, 2, &ebuf[i]) < 0)
			return -EINVAL;
	}
	return 0;
}

static void asix_get_drvinfo (struct net_device *net,
				 struct ethtool_drvinfo *info)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;

	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
	strncpy (info->driver, DRIVER_NAME, sizeof info->driver);
	strncpy (info->version, DRIVER_VERSION, sizeof info->version);
	info->eedump_len = data->eeprom_len;
}

static u32 asix_get_link(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return mii_link_ok(&dev->mii);
}

static int asix_ioctl (struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

static int asix_set_mac_address(struct net_device *net, void *p)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;
	struct sockaddr *addr = p;

	if (netif_running(net))
		return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(net->dev_addr, addr->sa_data, ETH_ALEN);

	/* We use the 20 byte dev->data
	 * for our 6 byte mac buffer
	 * to avoid allocating memory that
	 * is tricky to free later */
	memcpy(data->mac_addr, addr->sa_data, ETH_ALEN);
	asix_write_cmd_async(dev, AX_CMD_WRITE_NODE_ID, 0, 0, ETH_ALEN,
							data->mac_addr);

	return 0;
}

/* We need to override some ethtool_ops so we require our
   own structure so we don't interfere with other usbnet
   devices that may be connected at the same time. */
static const struct ethtool_ops ax88172_ethtool_ops = {
	.get_drvinfo		= asix_get_drvinfo,
	.get_link		= asix_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= asix_get_wol,
	.set_wol		= asix_set_wol,
	.get_eeprom_len		= asix_get_eeprom_len,
	.get_eeprom		= asix_get_eeprom,
	.nway_reset		= usbnet_nway_reset,
	.get_link_ksettings	= usbnet_get_link_ksettings,
	.set_link_ksettings	= usbnet_set_link_ksettings,
};

static void ax88172_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;
	u8 rx_ctl = 0x8c;

	if (net->flags & IFF_PROMISC) {
		rx_ctl |= 0x01;
	} else if (net->flags & IFF_ALLMULTI ||
		   netdev_mc_count(net) > AX_MAX_MCAST) {
		rx_ctl |= 0x02;
	} else if (netdev_mc_empty(net)) {
		/* just broadcast and directed */
	} else {
		/* We use the 20 byte dev->data
		 * for our 8 byte filter buffer
		 * to avoid allocating memory that
		 * is tricky to free later */
		struct netdev_hw_addr *ha;
		u32 crc_bits;

		memset(data->multi_filter, 0, AX_MCAST_FILTER_SIZE);

		/* Build the multicast hash filter. */
		netdev_for_each_mc_addr(ha, net) {
			crc_bits = ether_crc(ETH_ALEN, ha->addr) >> 26;
			data->multi_filter[crc_bits >> 3] |=
			    1 << (crc_bits & 7);
		}

		asix_write_cmd_async(dev, AX_CMD_WRITE_MULTI_FILTER, 0, 0,
				   AX_MCAST_FILTER_SIZE, data->multi_filter);

		rx_ctl |= 0x10;
	}

	asix_write_cmd_async(dev, AX_CMD_WRITE_RX_CTL, rx_ctl, 0, 0, NULL);
}

static int ax88172_link_reset(struct usbnet *dev)
{
	u8 mode;
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);
	mode = AX88172_MEDIUM_DEFAULT;

	if (ecmd.duplex != DUPLEX_FULL)
		mode |= ~AX88172_MEDIUM_FD;

	netdev_dbg(dev->net, "ax88172_link_reset() speed: %u duplex: %d setting mode to 0x%04x\n",
		   ethtool_cmd_speed(&ecmd), ecmd.duplex, mode);

	asix_write_medium_mode(dev, mode);

	return 0;
}

static const struct net_device_ops ax88172_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= fast_path_start_xmit,
	.ndo_fast_path_xmit	= usbnet_fast_path_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_get_stats64	= usbnet_get_stats64,
	.ndo_change_mtu		= usbnet_change_mtu,
	.ndo_set_mac_address 	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl		= asix_ioctl,
	.ndo_set_rx_mode	= ax88172_set_multicast,
};

static int ax88172_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret = 0;
	u8 buf[ETH_ALEN];
	int i;
	unsigned long gpio_bits = dev->driver_info->data;
	struct asix_data *data = (struct asix_data *)&dev->data;

	data->eeprom_len = AX88172_EEPROM_LEN;

	usbnet_get_endpoints(dev,intf);

	/* Toggle the GPIOs in a manufacturer/model specific way */
	for (i = 2; i >= 0; i--) {
		ret = asix_write_cmd(dev, AX_CMD_WRITE_GPIOS,
				(gpio_bits >> (i * 8)) & 0xff, 0, 0, NULL);
		if (ret < 0)
			goto out;
		msleep(5);
	}

	ret = asix_write_rx_ctl(dev, 0x80);
	if (ret < 0)
		goto out;

	/* Get the MAC address */
	ret = asix_read_cmd(dev, AX88172_CMD_READ_NODE_ID, 0, 0, ETH_ALEN, buf);
	if (ret < 0) {
		pr_debug("read AX_CMD_READ_NODE_ID failed: %d", ret);
		goto out;
	}
	memcpy(dev->net->dev_addr, buf, ETH_ALEN);

	/* Initialize MII structure */
	dev->mii.dev = dev->net;
	dev->mii.mdio_read = asix_mdio_read;
	dev->mii.mdio_write = asix_mdio_write;
	dev->mii.phy_id_mask = 0x3f;
	dev->mii.reg_num_mask = 0x1f;
	dev->mii.phy_id = asix_get_phy_addr(dev);

	dev->net->netdev_ops = &ax88172_netdev_ops;
	dev->net->ethtool_ops = &ax88172_ethtool_ops;

	asix_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR, BMCR_RESET);
	asix_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE,
		ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	mii_nway_restart(&dev->mii);

        dev->net->hard_header_len += 8; // len, pad
	return 0;

out:
	return ret;
}

static const struct ethtool_ops ax88772_ethtool_ops = {
	.get_drvinfo		= asix_get_drvinfo,
	.get_link		= asix_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= asix_get_wol,
	.set_wol		= asix_set_wol,
	.get_eeprom_len		= asix_get_eeprom_len,
	.get_eeprom		= asix_get_eeprom,
	.nway_reset		= usbnet_nway_reset,
	.get_link_ksettings	= usbnet_get_link_ksettings,
	.set_link_ksettings	= usbnet_set_link_ksettings,
};

static int ax88772_link_reset(struct usbnet *dev)
{
	u16 mode;
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);
	mode = AX88772_MEDIUM_DEFAULT;

	if (ethtool_cmd_speed(&ecmd) != SPEED_100)
		mode &= ~AX_MEDIUM_PS;

	if (ecmd.duplex != DUPLEX_FULL)
		mode &= ~AX_MEDIUM_FD;

	netdev_dbg(dev->net, "ax88772_link_reset() speed: %u duplex: %d setting mode to 0x%04x\n",
		   ethtool_cmd_speed(&ecmd), ecmd.duplex, mode);

	asix_write_medium_mode(dev, mode);

	return 0;
}

static int ax88772_reset(struct usbnet *dev)
{
	struct asix_data *data = (struct asix_data *)&dev->data;
	int ret, embd_phy;
	u16 rx_ctl;

	ret = asix_write_gpio(dev,
			AX_GPIO_RSE | AX_GPIO_GPO_2 | AX_GPIO_GPO2EN, 5);
	if (ret < 0)
		goto out;

	embd_phy = ((asix_get_phy_addr(dev) & 0x1f) == 0x10 ? 1 : 0);

	ret = asix_write_cmd(dev, AX_CMD_SW_PHY_SELECT, embd_phy, 0, 0, NULL);
	if (ret < 0) {
	    netdev_dbg(dev->net, "Select PHY #1 failed: %d", ret);
		goto out;
	}

	ret = asix_sw_reset(dev, AX_SWRESET_IPPD | AX_SWRESET_PRL);
	if (ret < 0)
		goto out;

	msleep(150);

	ret = asix_sw_reset(dev, AX_SWRESET_CLEAR);
	if (ret < 0)
		goto out;

	msleep(150);

	if (embd_phy) {
		ret = asix_sw_reset(dev, AX_SWRESET_IPRL);
		if (ret < 0)
			goto out;
	} else {
		ret = asix_sw_reset(dev, AX_SWRESET_PRTE);
		if (ret < 0)
			goto out;
	}

	msleep(150);
	rx_ctl = asix_read_rx_ctl(dev);
	netdev_dbg(dev->net, "RX_CTL is 0x%04x after software reset", rx_ctl);
	ret = asix_write_rx_ctl(dev, 0x0000);
	if (ret < 0)
		goto out;

	rx_ctl = asix_read_rx_ctl(dev);
	netdev_dbg(dev->net, "RX_CTL is 0x%04x setting to 0x0000", rx_ctl);

	ret = asix_sw_reset(dev, AX_SWRESET_PRL);
	if (ret < 0)
		goto out;

	msleep(150);

	ret = asix_sw_reset(dev, AX_SWRESET_IPRL | AX_SWRESET_PRL);
	if (ret < 0)
		goto out;

	msleep(150);

	asix_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR, BMCR_RESET);
	asix_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE,
			ADVERTISE_ALL | ADVERTISE_CSMA);
	mii_nway_restart(&dev->mii);

	ret = asix_write_medium_mode(dev, AX88772_MEDIUM_DEFAULT);
	if (ret < 0)
		goto out;

	ret = asix_write_cmd(dev, AX_CMD_WRITE_IPG0,
				AX88772_IPG0_DEFAULT | AX88772_IPG1_DEFAULT,
				AX88772_IPG2_DEFAULT, 0, NULL);
	if (ret < 0) {
		netdev_dbg(dev->net, "Write IPG,IPG1,IPG2 failed: %d", ret);
		goto out;
	}

	/* Rewrite MAC address */
	memcpy(data->mac_addr, dev->net->dev_addr, ETH_ALEN);
	ret = asix_write_cmd(dev, AX_CMD_WRITE_NODE_ID, 0, 0, ETH_ALEN,
							data->mac_addr);
	if (ret < 0)
		goto out;

	/* Set RX_CTL to default values with 2k buffer, and enable cactus */
	ret = asix_write_rx_ctl(dev, AX_DEFAULT_RX_CTL);
	if (ret < 0)
		goto out;

	rx_ctl = asix_read_rx_ctl(dev);
	netdev_dbg(dev->net, "RX_CTL is 0x%04x after all initializations", rx_ctl);

	rx_ctl = asix_read_medium_status(dev);
	netdev_dbg(dev->net, "Medium Status is 0x%04x after all initializations", rx_ctl);

	return 0;

out:
	return ret;

}

static const struct net_device_ops ax88772_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= fast_path_start_xmit,
	.ndo_fast_path_xmit	= usbnet_fast_path_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_get_stats64	= usbnet_get_stats64,
	.ndo_change_mtu		= usbnet_change_mtu,
	.ndo_set_mac_address 	= asix_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl		= asix_ioctl,
	.ndo_set_rx_mode        = asix_set_multicast,
};

static int ax88772_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret, embd_phy;
	struct asix_data *data = (struct asix_data *)&dev->data;
	u8 buf[ETH_ALEN];
	u32 phyid;

	data->eeprom_len = AX88772_EEPROM_LEN;

	usbnet_get_endpoints(dev,intf);

	/* Get the MAC address */
	ret = asix_read_cmd(dev, AX_CMD_READ_NODE_ID, 0, 0, ETH_ALEN, buf);
	if (ret < 0) {
		pr_debug("Failed to read MAC address: %d", ret);
		return ret;
	}
	memcpy(dev->net->dev_addr, buf, ETH_ALEN);

	/* Initialize MII structure */
	dev->mii.dev = dev->net;
	dev->mii.mdio_read = asix_mdio_read;
	dev->mii.mdio_write = asix_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;
	dev->mii.phy_id = asix_get_phy_addr(dev);

	dev->net->netdev_ops = &ax88772_netdev_ops;
	dev->net->ethtool_ops = &ax88772_ethtool_ops;

	embd_phy = ((dev->mii.phy_id & 0x1f) == 0x10 ? 1 : 0);

	/* Reset the PHY to normal operation mode */
	ret = asix_write_cmd(dev, AX_CMD_SW_PHY_SELECT, embd_phy, 0, 0, NULL);
	if (ret < 0) {
		pr_debug("Select PHY #1 failed: %d", ret);
		return ret;
	}

	ret = asix_sw_reset(dev, AX_SWRESET_IPPD | AX_SWRESET_PRL);
	if (ret < 0)
		return ret;

	msleep(150);

	ret = asix_sw_reset(dev, AX_SWRESET_CLEAR);
	if (ret < 0)
		return ret;

	msleep(150);

	ret = asix_sw_reset(dev, embd_phy ? AX_SWRESET_IPRL : AX_SWRESET_PRTE);

	/* Read PHYID register *AFTER* the PHY was reset properly */
	phyid = asix_get_phyid(dev);
	pr_debug("PHYID=0x%08x", phyid);

	/* Asix framing packs multiple eth frames into a 2K usb bulk transfer */
	if (dev->driver_info->flags & FLAG_FRAMING_AX) {
		/* hard_mtu  is still the default - the device does not support
		   jumbo eth frames */
		dev->rx_urb_size = 2048;
	}

        dev->net->hard_header_len += 8; // len, pad
	return 0;
}

static const struct ethtool_ops ax88178_ethtool_ops = {
	.get_drvinfo		= asix_get_drvinfo,
	.get_link		= asix_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= asix_get_wol,
	.set_wol		= asix_set_wol,
	.get_eeprom_len		= asix_get_eeprom_len,
	.get_eeprom		= asix_get_eeprom,
	.nway_reset		= usbnet_nway_reset,
	.get_link_ksettings	= usbnet_get_link_ksettings,
	.set_link_ksettings	= usbnet_set_link_ksettings,
};

static int marvell_phy_init(struct usbnet *dev)
{
	struct asix_data *data = (struct asix_data *)&dev->data;
	u16 reg;

	netdev_dbg(dev->net, "marvell_phy_init()\n");

	reg = asix_mdio_read(dev->net, dev->mii.phy_id, MII_MARVELL_STATUS);
	netdev_dbg(dev->net, "MII_MARVELL_STATUS = 0x%04x\n", reg);

	asix_mdio_write(dev->net, dev->mii.phy_id, MII_MARVELL_CTRL,
			MARVELL_CTRL_RXDELAY | MARVELL_CTRL_TXDELAY);

	if (data->ledmode) {
		reg = asix_mdio_read(dev->net, dev->mii.phy_id,
			MII_MARVELL_LED_CTRL);
		netdev_dbg(dev->net, "MII_MARVELL_LED_CTRL (1) = 0x%04x\n", reg);

		reg &= 0xf8ff;
		reg |= (1 + 0x0100);
		asix_mdio_write(dev->net, dev->mii.phy_id,
			MII_MARVELL_LED_CTRL, reg);

		reg = asix_mdio_read(dev->net, dev->mii.phy_id,
			MII_MARVELL_LED_CTRL);
		netdev_dbg(dev->net, "MII_MARVELL_LED_CTRL (2) = 0x%04x\n", reg);
		reg &= 0xfc0f;
	}

	return 0;
}

static int rtl8211cl_phy_init(struct usbnet *dev)
{
	struct asix_data *data = (struct asix_data *)&dev->data;

	netdev_dbg(dev->net, "rtl8211cl_phy_init()\n");

	asix_mdio_write (dev->net, dev->mii.phy_id, 0x1f, 0x0005);
	asix_mdio_write (dev->net, dev->mii.phy_id, 0x0c, 0);
	asix_mdio_write (dev->net, dev->mii.phy_id, 0x01,
		asix_mdio_read (dev->net, dev->mii.phy_id, 0x01) | 0x0080);
	asix_mdio_write (dev->net, dev->mii.phy_id, 0x1f, 0);

	if (data->ledmode == 12) {
		asix_mdio_write (dev->net, dev->mii.phy_id, 0x1f, 0x0002);
		asix_mdio_write (dev->net, dev->mii.phy_id, 0x1a, 0x00cb);
		asix_mdio_write (dev->net, dev->mii.phy_id, 0x1f, 0);
	}

	return 0;
}

static int marvell_led_status(struct usbnet *dev, u16 speed)
{
	u16 reg = asix_mdio_read(dev->net, dev->mii.phy_id, MARVELL_LED_MANUAL);

	netdev_dbg(dev->net, "marvell_led_status() read 0x%04x\n", reg);

	/* Clear out the center LED bits - 0x03F0 */
	reg &= 0xfc0f;

	switch (speed) {
		case SPEED_1000:
			reg |= 0x03e0;
			break;
		case SPEED_100:
			reg |= 0x03b0;
			break;
		default:
			reg |= 0x02f0;
	}

	netdev_dbg(dev->net, "marvell_led_status() writing 0x%04x\n", reg);
	asix_mdio_write(dev->net, dev->mii.phy_id, MARVELL_LED_MANUAL, reg);

	return 0;
}

static int ax88178_reset(struct usbnet *dev)
{
	struct asix_data *data = (struct asix_data *)&dev->data;
	int ret;
	__le16 eeprom;
	u8 status;
	int gpio0 = 0;
	u32 phyid;

	asix_read_cmd(dev, AX_CMD_READ_GPIOS, 0, 0, 1, &status);
	pr_debug("GPIO Status: 0x%04x", status);

	asix_write_cmd(dev, AX_CMD_WRITE_ENABLE, 0, 0, 0, NULL);
	asix_read_cmd(dev, AX_CMD_READ_EEPROM, 0x0017, 0, 2, &eeprom);
	asix_write_cmd(dev, AX_CMD_WRITE_DISABLE, 0, 0, 0, NULL);

	pr_debug("EEPROM index 0x17 is 0x%04x", eeprom);

	if (eeprom == cpu_to_le16(0xffff)) {
		data->phymode = PHY_MODE_MARVELL;
		data->ledmode = 0;
		gpio0 = 1;
	} else {
		data->phymode = le16_to_cpu(eeprom) & 0x7F;
		data->ledmode = le16_to_cpu(eeprom) >> 8;
		gpio0 = (le16_to_cpu(eeprom) & 0x80) ? 0 : 1;
	}
	pr_debug("GPIO0: %d, PhyMode: %d", gpio0, data->phymode);

	/* Power up external GigaPHY through AX88178 GPIO pin */
	asix_write_gpio(dev, AX_GPIO_RSE | AX_GPIO_GPO_1 | AX_GPIO_GPO1EN, 40);
	if ((le16_to_cpu(eeprom) >> 8) != 1) {
		asix_write_gpio(dev, 0x003c, 30);
		asix_write_gpio(dev, 0x001c, 300);
		asix_write_gpio(dev, 0x003c, 30);
	} else {
		pr_debug("gpio phymode == 1 path");
		asix_write_gpio(dev, AX_GPIO_GPO1EN, 30);
		asix_write_gpio(dev, AX_GPIO_GPO1EN | AX_GPIO_GPO_1, 30);
	}

	/* Read PHYID register *AFTER* powering up PHY */
	phyid = asix_get_phyid(dev);
	pr_debug("PHYID=0x%08x", phyid);

	/* Set AX88178 to enable MII/GMII/RGMII interface for external PHY */
	asix_write_cmd(dev, AX_CMD_SW_PHY_SELECT, 0, 0, 0, NULL);

	asix_sw_reset(dev, 0);
	msleep(150);

	asix_sw_reset(dev, AX_SWRESET_PRL | AX_SWRESET_IPPD);
	msleep(150);

	asix_write_rx_ctl(dev, 0);

	if (data->phymode == PHY_MODE_MARVELL) {
		marvell_phy_init(dev);
		msleep(60);
	} else if (data->phymode == PHY_MODE_RTL8211CL)
		rtl8211cl_phy_init(dev);

	asix_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR,
			BMCR_RESET | BMCR_ANENABLE);
	asix_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE,
			ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	asix_mdio_write(dev->net, dev->mii.phy_id, MII_CTRL1000,
			ADVERTISE_1000FULL);

	mii_nway_restart(&dev->mii);

	ret = asix_write_medium_mode(dev, AX88178_MEDIUM_DEFAULT);
	if (ret < 0)
		return ret;

	/* Rewrite MAC address */
	memcpy(data->mac_addr, dev->net->dev_addr, ETH_ALEN);
	ret = asix_write_cmd(dev, AX_CMD_WRITE_NODE_ID, 0, 0, ETH_ALEN,
							data->mac_addr);
	if (ret < 0)
		return ret;

	ret = asix_write_rx_ctl(dev, AX_DEFAULT_RX_CTL);
	if (ret < 0)
		return ret;

	return 0;
}

static int ax88178_link_reset(struct usbnet *dev)
{
	u16 mode;
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };
	struct asix_data *data = (struct asix_data *)&dev->data;
	u32 speed;

	netdev_dbg(dev->net, "ax88178_link_reset()\n");

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);
	mode = AX88178_MEDIUM_DEFAULT;
	speed = ethtool_cmd_speed(&ecmd);

	if (speed == SPEED_1000)
		mode |= AX_MEDIUM_GM;
	else if (speed == SPEED_100)
		mode |= AX_MEDIUM_PS;
	else
		mode &= ~(AX_MEDIUM_PS | AX_MEDIUM_GM);

	mode |= AX_MEDIUM_ENCK;

	if (ecmd.duplex == DUPLEX_FULL)
		mode |= AX_MEDIUM_FD;
	else
		mode &= ~AX_MEDIUM_FD;

	netdev_dbg(dev->net, "ax88178_link_reset() speed: %u duplex: %d setting mode to 0x%04x\n",
		   speed, ecmd.duplex, mode);

	asix_write_medium_mode(dev, mode);

	if (data->phymode == PHY_MODE_MARVELL && data->ledmode)
		marvell_led_status(dev, speed);

	return 0;
}

static void ax88178_set_mfb(struct usbnet *dev)
{
	u16 mfb = AX_RX_CTL_MFB_16384;
	u16 rxctl;
	u16 medium;
	int old_rx_urb_size = dev->rx_urb_size;

	if (dev->hard_mtu < 2048) {
		dev->rx_urb_size = 2048;
		mfb = AX_RX_CTL_MFB_2048;
	} else if (dev->hard_mtu < 4096) {
		dev->rx_urb_size = 4096;
		mfb = AX_RX_CTL_MFB_4096;
	} else if (dev->hard_mtu < 8192) {
		dev->rx_urb_size = 8192;
		mfb = AX_RX_CTL_MFB_8192;
	} else if (dev->hard_mtu < 16384) {
		dev->rx_urb_size = 16384;
		mfb = AX_RX_CTL_MFB_16384;
	}

	rxctl = asix_read_rx_ctl(dev);
	asix_write_rx_ctl(dev, (rxctl & ~AX_RX_CTL_MFB_16384) | mfb);

	medium = asix_read_medium_status(dev);
	if (dev->net->mtu > 1500)
		medium |= AX_MEDIUM_JFE;
	else
		medium &= ~AX_MEDIUM_JFE;
	asix_write_medium_mode(dev, medium);

	if (dev->rx_urb_size > old_rx_urb_size)
		usbnet_unlink_rx_urbs(dev);
}

static int ax88178_change_mtu(struct net_device *net, int new_mtu)
{
	struct usbnet *dev = netdev_priv(net);
	int ll_mtu = new_mtu + net->hard_header_len + 4;

	netdev_dbg(dev->net, "ax88178_change_mtu() new_mtu=%d\n", new_mtu);

	if (new_mtu <= 0 || ll_mtu > 16384)
		return -EINVAL;

	if ((ll_mtu % dev->maxpacket) == 0)
		return -EDOM;

	net->mtu = new_mtu;
	dev->hard_mtu = net->mtu + net->hard_header_len;
	ax88178_set_mfb(dev);

	return 0;
}

static const struct net_device_ops ax88178_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= fast_path_start_xmit,
	.ndo_fast_path_xmit	= usbnet_fast_path_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_get_stats64	= usbnet_get_stats64,
	.ndo_set_mac_address 	= asix_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_rx_mode	= asix_set_multicast,
	.ndo_do_ioctl 		= asix_ioctl,
	.ndo_change_mtu 	= ax88178_change_mtu,
};

static int ax88178_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret;
	u8 buf[ETH_ALEN];
	struct asix_data *data = (struct asix_data *)&dev->data;

	data->eeprom_len = AX88772_EEPROM_LEN;

	usbnet_get_endpoints(dev,intf);

	/* Get the MAC address */
	ret = asix_read_cmd(dev, AX_CMD_READ_NODE_ID, 0, 0, ETH_ALEN, buf);
	if (ret < 0) {
		pr_debug("Failed to read MAC address: %d", ret);
		return ret;
	}
	memcpy(dev->net->dev_addr, buf, ETH_ALEN);

	/* Initialize MII structure */
	dev->mii.dev = dev->net;
	dev->mii.mdio_read = asix_mdio_read;
	dev->mii.mdio_write = asix_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0xff;
	dev->mii.supports_gmii = 1;
	dev->mii.phy_id = asix_get_phy_addr(dev);

	dev->net->netdev_ops = &ax88178_netdev_ops;
	dev->net->ethtool_ops = &ax88178_ethtool_ops;

	/* Blink LEDS so users know driver saw dongle */
	asix_sw_reset(dev, 0);
	msleep(150);

	asix_sw_reset(dev, AX_SWRESET_PRL | AX_SWRESET_IPPD);
	msleep(150);

	/* Asix framing packs multiple eth frames into a 2K usb bulk transfer */
	if (dev->driver_info->flags & FLAG_FRAMING_AX) {
		/* hard_mtu  is still the default - the device does not support
		   jumbo eth frames */
		dev->rx_urb_size = 2048;
	}

        dev->net->hard_header_len += 8; // len, pad
	return 0;
}

const struct driver_info ax8817x_info = {
	.description = "ASIX AX8817x USB 2.0 Ethernet",
	.bind = ax88172_bind,
	.status = asix_status,
	.link_reset = ax88172_link_reset,
	.reset = ax88172_link_reset,
	.flags =  FLAG_ETHER | FLAG_LINK_INTR,
	.data = 0x00130103,
};

const struct driver_info dlink_dub_e100_info = {
	.description = "DLink DUB-E100 USB Ethernet",
	.bind = ax88172_bind,
	.status = asix_status,
	.link_reset = ax88172_link_reset,
	.reset = ax88172_link_reset,
	.flags =  FLAG_ETHER | FLAG_LINK_INTR,
	.data = 0x009f9d9f,
};

const struct driver_info netgear_fa120_info = {
	.description = "Netgear FA-120 USB Ethernet",
	.bind = ax88172_bind,
	.status = asix_status,
	.link_reset = ax88172_link_reset,
	.reset = ax88172_link_reset,
	.flags =  FLAG_ETHER | FLAG_LINK_INTR,
	.data = 0x00130103,
};

const struct driver_info hawking_uf200_info = {
	.description = "Hawking UF200 USB Ethernet",
	.bind = ax88172_bind,
	.status = asix_status,
	.link_reset = ax88172_link_reset,
	.reset = ax88172_link_reset,
	.flags =  FLAG_ETHER | FLAG_LINK_INTR,
	.data = 0x001f1d1f,
};

const struct driver_info ax88772_info = {
	.description = "ASIX AX88772 USB 2.0 Ethernet",
	.bind = ax88772_bind,
	.status = asix_status,
	.link_reset = ax88772_link_reset,
	.reset = ax88772_reset,
	.flags = FLAG_ETHER | FLAG_FRAMING_AX | FLAG_LINK_INTR,
	.rx_fixup = asix_rx_fixup,
	.tx_fixup = asix_tx_fixup,
};

const struct driver_info ax88178_info = {
	.description = "ASIX AX88178 USB 2.0 Ethernet",
	.bind = ax88178_bind,
	.status = asix_status,
	.link_reset = ax88178_link_reset,
	.reset = ax88178_reset,
	.flags = FLAG_ETHER | FLAG_FRAMING_AX | FLAG_LINK_INTR,
	.rx_fixup = asix_rx_fixup,
	.tx_fixup = asix_tx_fixup,
};

static const struct usb_device_id	products [] = {
{
	// Linksys USB200M
	USB_DEVICE (0x077b, 0x2226),
	.driver_info =	(unsigned long) &ax8817x_info,
}, {
	// Netgear FA120
	USB_DEVICE (0x0846, 0x1040),
	.driver_info =  (unsigned long) &netgear_fa120_info,
}, {
	// DLink DUB-E100
	USB_DEVICE (0x2001, 0x1a00),
	.driver_info =  (unsigned long) &dlink_dub_e100_info,
}, {
	// Intellinet, ST Lab USB Ethernet
	USB_DEVICE (0x0b95, 0x1720),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// Hawking UF200, TrendNet TU2-ET100
	USB_DEVICE (0x07b8, 0x420a),
	.driver_info =  (unsigned long) &hawking_uf200_info,
}, {
	// Billionton Systems, USB2AR
	USB_DEVICE (0x08dd, 0x90ff),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// ATEN UC210T
	USB_DEVICE (0x0557, 0x2009),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// Buffalo LUA-U2-KTX
	USB_DEVICE (0x0411, 0x003d),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// Buffalo LUA-U2-GT 10/100/1000
	USB_DEVICE (0x0411, 0x006e),
	.driver_info =  (unsigned long) &ax88178_info,
}, {
	// Sitecom LN-029 "USB 2.0 10/100 Ethernet adapter"
	USB_DEVICE (0x6189, 0x182d),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// Sitecom LN-031 "USB 2.0 10/100/1000 Ethernet adapter"
	USB_DEVICE (0x0df6, 0x0056),
	.driver_info =  (unsigned long) &ax88178_info,
}, {
	// corega FEther USB2-TX
	USB_DEVICE (0x07aa, 0x0017),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// Surecom EP-1427X-2
	USB_DEVICE (0x1189, 0x0893),
	.driver_info = (unsigned long) &ax8817x_info,
}, {
	// goodway corp usb gwusb2e
	USB_DEVICE (0x1631, 0x6200),
	.driver_info = (unsigned long) &ax8817x_info,
}, {
	// JVC MP-PRX1 Port Replicator
	USB_DEVICE (0x04f1, 0x3008),
	.driver_info = (unsigned long) &ax8817x_info,
}, {
	// ASIX AX88772B 10/100
	USB_DEVICE (0x0b95, 0x772b),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// ASIX AX88772 10/100
	USB_DEVICE (0x0b95, 0x7720),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// ASIX AX88178 10/100/1000
	USB_DEVICE (0x0b95, 0x1780),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// Logitec LAN-GTJ/U2A
	USB_DEVICE (0x0789, 0x0160),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// Linksys USB200M Rev 2
	USB_DEVICE (0x13b1, 0x0018),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// 0Q0 cable ethernet
	USB_DEVICE (0x1557, 0x7720),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// DLink DUB-E100 H/W Ver B1
	USB_DEVICE (0x07d1, 0x3c05),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// DLink DUB-E100 H/W Ver B1 Alternate
	USB_DEVICE (0x2001, 0x3c05),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// Linksys USB1000
	USB_DEVICE (0x1737, 0x0039),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// IO-DATA ETG-US2
	USB_DEVICE (0x04bb, 0x0930),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// Belkin F5D5055
	USB_DEVICE(0x050d, 0x5055),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// Apple USB Ethernet Adapter
	USB_DEVICE(0x05ac, 0x1402),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// Cables-to-Go USB Ethernet Adapter
	USB_DEVICE(0x0b95, 0x772a),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// ABOCOM for pci
	USB_DEVICE(0x14ea, 0xab11),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// ASIX 88772a
	USB_DEVICE(0x0db0, 0xa877),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// Asus USB Ethernet Adapter
	USB_DEVICE (0x0b95, 0x7e2b),
	.driver_info = (unsigned long) &ax88772_info,
},
	{ },		// END
};
//MODULE_DEVICE_TABLE(usb, products);

struct usb_driver asix_driver = {
	.name =		DRIVER_NAME,
	.id_table =	products,
	.probe =	usbnet_probe,
	.disconnect =	usbnet_disconnect,
#if 0
	.suspend =	usbnet_suspend,
	.resume =	usbnet_resume,
	.supports_autosuspend = 1,
#endif
};

//module_usb_driver(asix_driver);

MODULE_AUTHOR("David Hollis");
MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION("ASIX AX8817X based USB 2.0 Ethernet Devices");
MODULE_LICENSE("GPL");
