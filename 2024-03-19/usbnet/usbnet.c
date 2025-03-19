/*
 * USB Network driver infrastructure
 * Copyright (C) 2000-2005 by David Brownell
 * Copyright (C) 2003-2005 David Hollis <dhollis@davehollis.com>
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

/*
 * This is a generic "USB networking" framework that works with several
 * kinds of full and high speed networking devices:  host-to-host cables,
 * smart usb peripherals, and actual Ethernet adapters.
 *
 * These devices usually differ in terms of control protocols (if they
 * even have one!) and sometimes they define new framing to wrap or batch
 * Ethernet packets.  Otherwise, they talk to USB pretty much the same,
 * so interface (un)binding, endpoint I/O queues, fault handling, and other
 * issues can usefully be addressed by this framework.
 */

// #define	DEBUG			// error path messages, extra info
// #define	VERBOSE			// more; success messages
#define DEBUG_PRINTK if (0)

#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ctype.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/if_arp.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/memmove_helper.h>
#include "usbnet.h"
#include "io.h"

#if defined(ARM)
#include <linux/of.h>
#endif

#define DRIVER_VERSION		"22-Aug-2005"

#undef EXPORT_SYMBOL_GPL
#define EXPORT_SYMBOL_GPL(a)

static unsigned FP_QUEUE_SIZE = 0;

static struct net_device *passthrough_fwd_handler(struct net_device *in_dev, struct fp_buf *fpb);
static struct fast_path fp_rx_handler = {
    .handler = passthrough_fwd_handler,
    .priority = FP_PRIO_MODEM_PASSTHROUGH,
};

LIST_HEAD(br_list);
struct br_entry {
    struct list_head list;
    struct net_device *modem;
    struct net_device *slave;
    unsigned short vlan_id;
    u8 dst_mac[ETH_ALEN];
    u32 dst_ip;
    unsigned char single_apn;
    unsigned char mac_update;
    u8 modem_mac[ETH_ALEN];
};

/*-------------------------------------------------------------------------*/

/*
 * Nineteen USB 1.1 max size bulk transactions per frame (ms), max.
 * Several dozen bytes of IPv4 data can fit in two such transactions.
 * One maximum size Ethernet packet takes twenty four of them.
 * For high speed, each frame comfortably fits almost 36 max size
 * Ethernet packets (so queues should be bigger).
 *
 * REVISIT qlens should be members of 'struct usbnet'; the goal is to
 * let the USB host controller be busy for 5msec or more before an irq
 * is required, under load.  Jumbograms change the equation.
 */
#if 0
#define RX_MAX_QUEUE_MEMORY (60 * 1518)
#define	RX_QLEN(dev) (((dev)->udev->speed == USB_SPEED_HIGH) ? \
			(RX_MAX_QUEUE_MEMORY/(dev)->rx_urb_size) : 4)
#define	TX_QLEN(dev) (((dev)->udev->speed == USB_SPEED_HIGH) ? \
			(RX_MAX_QUEUE_MEMORY/(dev)->hard_mtu) : 4)
#else
unsigned inline	RX_QLEN(struct usbnet *dev) { return FP_QUEUE_SIZE; }
unsigned inline	TX_QLEN(struct usbnet *dev) { return FP_QUEUE_SIZE; }
#endif

// reawaken network queue this soon after stopping; else watchdog barks
#define TX_TIMEOUT_JIFFIES	(5*HZ)

// throttle rx/tx briefly after some faults, so khubd might disconnect()
// us (it polls at HZ/4 usually) before we report too many false errors.
#define THROTTLE_JIFFIES	(HZ/8)

// between wakeups
#define UNLINK_TIMEOUT_MS	3

/*-------------------------------------------------------------------------*/

// randomly generated ethernet address
static u8	node_id [ETH_ALEN];

/* use ethtool to change the level for any given device */
static int msg_level = -1;
module_param (msg_level, int, 0);
MODULE_PARM_DESC (msg_level, "Override default message level");

/*-------------------------------------------------------------------------*/

/* handles CDC Ethernet and many other network "bulk data" interfaces */
int usbnet_get_endpoints(struct usbnet *dev, struct usb_interface *intf)
{
	int				tmp;
	struct usb_host_interface	*alt = NULL;
	struct usb_host_endpoint	*in = NULL, *out = NULL;
	struct usb_host_endpoint	*status = NULL;

	for (tmp = 0; tmp < intf->num_altsetting; tmp++) {
		unsigned	ep;

		in = out = status = NULL;
		alt = intf->altsetting + tmp;

		/* take the first altsetting with in-bulk + out-bulk;
		 * remember any status endpoint, just in case;
		 * ignore other endpoints and altsettings.
		 */
		for (ep = 0; ep < alt->desc.bNumEndpoints; ep++) {
			struct usb_host_endpoint	*e;
			int				intr = 0;

			e = alt->endpoint + ep;
			switch (e->desc.bmAttributes) {
			case USB_ENDPOINT_XFER_INT:
				if (!usb_endpoint_dir_in(&e->desc))
					continue;
				intr = 1;
				/* FALLTHROUGH */
			case USB_ENDPOINT_XFER_BULK:
				break;
			default:
				continue;
			}
			if (usb_endpoint_dir_in(&e->desc)) {
				if (!intr && !in)
					in = e;
				else if (intr && !status)
					status = e;
			} else {
				if (!out)
					out = e;
			}
		}
		if (in && out)
			break;
	}
	if (!alt || !in || !out) {
                dev_err(&intf->dev, "required endpoint missing: %p %p %p\n",
                    alt, in, out);
		return -EINVAL;
        }

	if (alt->desc.bAlternateSetting != 0 ||
	    !(dev->driver_info->flags & FLAG_NO_SETINT)) {
		tmp = usb_set_interface (dev->udev, alt->desc.bInterfaceNumber,
				alt->desc.bAlternateSetting);
		if (tmp < 0)
			return tmp;
	}

	dev->in = usb_rcvbulkpipe (dev->udev,
			in->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->out = usb_sndbulkpipe (dev->udev,
			out->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	dev->status = status;
	return 0;
}
EXPORT_SYMBOL_GPL(usbnet_get_endpoints);

int usbnet_get_ethernet_addr(struct usbnet *dev, int iMACAddress)
{
	int 		tmp, i;
	char	buf[13];

	tmp = usb_string(dev->udev, iMACAddress, buf, sizeof buf);
	if (tmp != 12) {
		dev_err(&dev->udev->dev,
			"bad MAC string %d fetch, %d\n", iMACAddress, tmp);
		if (tmp >= 0)
			tmp = -EINVAL;
		return tmp;
	}
	for (i = tmp = 0; i < 6; i++, tmp += 2)
		dev->net->dev_addr [i] =
			(hex_to_bin(buf[tmp]) << 4) + hex_to_bin(buf[tmp + 1]);
	return 0;
}
EXPORT_SYMBOL_GPL(usbnet_get_ethernet_addr);

static void intr_complete (struct urb *urb)
{
	struct usbnet	*dev = urb->context;
	int		status = urb->status;

	switch (status) {
	/* success */
	case 0:
		dev->driver_info->status(dev, urb);
		break;

	/* software-driven interface shutdown */
	case -ENOENT:		/* urb killed */
	case -ESHUTDOWN:	/* hardware gone */
		netif_dbg(dev, ifdown, dev->net,
			  "intr shutdown, code %d\n", status);
		return;

	/* NOTE:  not throttling like RX/TX, since this endpoint
	 * already polls infrequently
	 */
	default:
		netdev_dbg(dev->net, "intr status %d\n", status);
		break;
	}

	if (!netif_running (dev->net))
		return;

	memset(urb->transfer_buffer, 0, urb->transfer_buffer_length);
	status = usb_submit_urb (urb, GFP_ATOMIC);
	if (status != 0)
		netif_err(dev, timer, dev->net,
			  "intr resubmit --> %d\n", status);
}

static int init_status (struct usbnet *dev, struct usb_interface *intf)
{
	char		*buf = NULL;
	unsigned	pipe = 0;
	unsigned	maxp;
	unsigned	period;

	if (!dev->driver_info->status)
		return 0;

	pipe = usb_rcvintpipe (dev->udev,
			dev->status->desc.bEndpointAddress
				& USB_ENDPOINT_NUMBER_MASK);
	maxp = usb_maxpacket (dev->udev, pipe, 0);

	/* avoid 1 msec chatter:  min 8 msec poll rate */
	period = max ((int) dev->status->desc.bInterval,
		(dev->udev->speed == USB_SPEED_HIGH) ? 7 : 3);

	buf = kmalloc (maxp, GFP_KERNEL);
	if (buf) {
		dev->interrupt = usb_alloc_urb (0, GFP_KERNEL);
		if (!dev->interrupt) {
			kfree (buf);
			return -ENOMEM;
		} else {
			usb_fill_int_urb(dev->interrupt, dev->udev, pipe,
				buf, maxp, intr_complete, dev, period);
			dev->interrupt->transfer_flags |= URB_FREE_BUFFER;
			dev_dbg(&intf->dev,
				"status ep%din, %d bytes period %d\n",
				usb_pipeendpoint(pipe), maxp, period);
		}
	}
	return 0;
}

/* Submit the interrupt URB if not previously submitted, increasing refcount */
int usbnet_status_start(struct usbnet *dev, gfp_t mem_flags)
{
	int ret = 0;

	WARN_ON_ONCE(dev->interrupt == NULL);
	if (dev->interrupt) {
		mutex_lock(&dev->interrupt_mutex);

		if (++dev->interrupt_count == 1)
			ret = usb_submit_urb(dev->interrupt, mem_flags);

		dev_dbg(&dev->udev->dev, "incremented interrupt URB count to %d\n",
			dev->interrupt_count);
		mutex_unlock(&dev->interrupt_mutex);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(usbnet_status_start);

#if 0
/* For resume; submit interrupt URB if previously submitted */
static int __usbnet_status_start_force(struct usbnet *dev, gfp_t mem_flags)
{
	int ret = 0;

	mutex_lock(&dev->interrupt_mutex);
	if (dev->interrupt_count) {
		ret = usb_submit_urb(dev->interrupt, mem_flags);
		dev_dbg(&dev->udev->dev,
			"submitted interrupt URB for resume\n");
	}
	mutex_unlock(&dev->interrupt_mutex);
	return ret;
}
#endif

/* Kill the interrupt URB if all submitters want it killed */
void usbnet_status_stop(struct usbnet *dev)
{
	if (dev->interrupt) {
		mutex_lock(&dev->interrupt_mutex);
		WARN_ON(dev->interrupt_count == 0);

		if (dev->interrupt_count && --dev->interrupt_count == 0)
			usb_kill_urb(dev->interrupt);

		dev_dbg(&dev->udev->dev,
			"decremented interrupt URB count to %d\n",
			dev->interrupt_count);
		mutex_unlock(&dev->interrupt_mutex);
	}
}
EXPORT_SYMBOL_GPL(usbnet_status_stop);

#if 0
/* For suspend; always kill interrupt URB */
static void __usbnet_status_stop_force(struct usbnet *dev)
{
	if (dev->interrupt) {
		mutex_lock(&dev->interrupt_mutex);
		usb_kill_urb(dev->interrupt);
		dev_dbg(&dev->udev->dev, "killed interrupt URB for suspend\n");
		mutex_unlock(&dev->interrupt_mutex);
	}
}
#endif

/* Passes this packet up the stack, updating its accounting.
 * Some link protocols batch packets, so their rx_fixup paths
 * can return clones as well as just modify the original skb.
 */
void usbnet_skb_return (struct usbnet *dev, struct fp_buf *fpb)
{
	struct pcpu_sw_netstats *stats64 = this_cpu_ptr(dev->stats64);
        struct br_entry *br;
        struct net_device *dst_dev = NULL;
        struct ethhdr *eth;
        int drop = 1;
        unsigned short vlan_id = 0;

	u64_stats_update_begin(&stats64->syncp);
	stats64->rx_packets++;
	stats64->rx_bytes += fpb_len(fpb);
	u64_stats_update_end(&stats64->syncp);

        if (list_empty(&br_list)) {
            fast_path_rx_noinvalidate(dev->ndev, fpb);
            return;
        }

        eth = (struct ethhdr *) fpb_data(fpb);
        rcu_read_lock();
        list_for_each_entry_rcu(br, &br_list, list) {
            if (br->modem != dev->ndev) continue;

            if (br->vlan_id) {
                if (!vlan_id) {
                    struct vlan_hdr *vhdr = (struct vlan_hdr *)(eth + 1);
                    if (fpb_len(fpb) < ETH_HLEN + sizeof(struct vlan_hdr)
                            || !ANY_VLAN_PROTO_N(eth->h_proto)) {
                        continue;
                    }

                    eth->h_proto = vhdr->h_vlan_encapsulated_proto;
                    vlan_id = ntohs(vhdr->h_vlan_TCI);
                    memmove12plus4(fpb_data(fpb));
                    fpb_pull(fpb, sizeof(struct vlan_hdr));

                    eth = (struct ethhdr *) fpb_data(fpb);

                    dev->net->stats.rx_bytes -= sizeof(struct vlan_hdr);
                }

                if (vlan_id != br->vlan_id) continue;

                DEBUG_PRINTK printk("rx modem vlan %d\n", vlan_id);
            }

            dst_dev = br->slave;

            // forward all
            if (!br->dst_ip) {
                memcpy(eth->h_dest, br->dst_mac, ETH_ALEN);
                drop = 0;
                break;
            }

            if (unlikely(!br->modem_mac[0]) && br->mac_update) {
                memcpy(br->modem_mac, eth->h_source, ETH_ALEN);
            }

            // ip forward
            if (likely(eth->h_proto == __constant_htons(ETH_P_IP))) {
                struct iphdr *ip = (struct iphdr*) (eth + 1);
                if (likely(br->single_apn || br->dst_ip == ip->daddr)) {
                    if (dst_dev) memcpy(eth->h_dest, br->dst_mac, ETH_ALEN);
                    drop = 0;
                    break;
                }

            // local arp rx
            } else if (eth->h_proto == __constant_htons(ETH_P_ARP)) {
                struct arphdr *arp = (struct arphdr*) (eth + 1);
                if (arp->ar_op == htons(ARPOP_REPLY)){
                    if (!memcmp(&br->dst_ip, (char*) arp + sizeof(*arp) + 16, 4)) {
                        drop = 0;
                        break;
                    }
                }
            }

            // forward the rest
            if (br->single_apn && drop) {
                drop = 0;
                break;
            }
        }
        rcu_read_unlock();

        if (unlikely(drop)) {
            __skb_bin_put_buffer_noinvalidate(fpb_buf(fpb));
        } else {
            if (!dst_dev) {
                fast_path_rx_noinvalidate(dev->ndev, fpb);
            } else {
                fast_path_fp_rx_inc(dev->ndev, fpb);
                fast_path_tx(dst_dev, fpb);
            }
        }
}
EXPORT_SYMBOL_GPL(usbnet_skb_return);


/*-------------------------------------------------------------------------
 *
 * Network Device Driver (peer link to "Host Device", from USB host)
 *
 *-------------------------------------------------------------------------*/

int usbnet_change_mtu (struct net_device *net, int new_mtu)
{
	struct usbnet	*dev = netdev_priv(net);
	int		ll_mtu = new_mtu + net->hard_header_len;
	int		old_hard_mtu = dev->hard_mtu;
	int		old_rx_urb_size = dev->rx_urb_size;

	if (new_mtu <= 0)
		return -EINVAL;
	// no second zero-length packet read wanted after mtu-sized packets
	if ((ll_mtu % dev->maxpacket) == 0)
		return -EDOM;

        // limit to skb_bin size
        if (skb_bin_data_size() < net->mtu + net->hard_header_len)
            return -EINVAL;

	net->mtu = new_mtu;

	dev->hard_mtu = net->mtu + net->hard_header_len;

	return 0;
}
EXPORT_SYMBOL_GPL(usbnet_change_mtu);

/*-------------------------------------------------------------------------*/

/* some LK 2.4 HCDs oopsed if we freed or resubmitted urbs from
 * completion callbacks.  2.5 should have fixed those bugs...
 */

static void defer_bh(struct usbnet *dev, struct fp_data *fpd, struct fp_queue *list)
{
	unsigned long		flags;

	spin_lock_irqsave(&list->lock, flags);
        list->qlen--;
	list_del(&fpd->list);
	spin_unlock(&list->lock);

	spin_lock(&dev->done.lock);
        dev->done.qlen++;
	list_add_tail(&fpd->list, &dev->done.queue);
	spin_unlock_irqrestore(&dev->done.lock, flags);

	tasklet_schedule(&dev->bh);
}

/* some work can't be done in tasklets, so we use keventd
 *
 * NOTE:  annoying asymmetry:  if it's active, schedule_work() fails,
 * but tasklet_schedule() doesn't.  hope the failure is rare.
 */
void usbnet_defer_kevent (struct usbnet *dev, int work)
{
	set_bit (work, &dev->flags);
	if (!schedule_work (&dev->kevent)) {
                if (net_ratelimit())
                    netdev_err(dev->net, "kevent %d may have been dropped\n", work);
	} else
		netdev_dbg(dev->net, "kevent %d scheduled\n", work);
}
EXPORT_SYMBOL_GPL(usbnet_defer_kevent);

/*-------------------------------------------------------------------------*/

static void rx_complete (struct urb *urb);

static int rx_submit (struct usbnet *dev, struct urb *urb, gfp_t flags)
{
        void                    *buf;
	struct fp_data		*entry;
	int			retval = 0;
	unsigned long		lockflags;

        if (likely(flags == GFP_ATOMIC)) buf = __skb_bin_get_buffer();
        else {
            local_bh_disable();
            buf = __skb_bin_get_buffer();
            if (unlikely(!buf)) buf = skb_bin_alloc_buffer(GFP_KERNEL);
            local_bh_enable();
        }

	spin_lock_irqsave(&dev->rxq.lock, lockflags);
	if (!buf || !dev->rxq.psize) {
                spin_unlock_irqrestore(&dev->rxq.lock, lockflags);
		if (!buf) usbnet_defer_kevent (dev, EVENT_RX_MEMORY);
                else {
                    if (likely(flags == GFP_ATOMIC)) __skb_bin_put_buffer_noinvalidate(buf);
                    else {
                        local_bh_disable();
                        __skb_bin_put_buffer_noinvalidate(buf);
                        local_bh_enable();
                    }
                }
		netif_dbg(dev, rx_err, dev->net, "no rx skb\n");
		usb_free_urb (urb);
		return -ENOMEM;
	}

        entry = dev->rxq.pool[--dev->rxq.psize];
        dev->rxq.pool[dev->rxq.psize] = NULL;

        entry->buf = buf + NET_SKB_PAD;
	entry->urb = urb;
	entry->dev = dev;
	entry->state = rx_start;

	usb_fill_bulk_urb (urb, dev->udev, dev->in,
                    entry->buf, dev->rx_urb_size, rx_complete, entry);

	if (netif_running (dev->net) &&
	    netif_device_present (dev->net) &&
	    !test_bit (EVENT_RX_HALT, &dev->flags) &&
	    !test_bit (EVENT_DEV_ASLEEP, &dev->flags)) {
		switch (retval = usb_submit_urb (urb, GFP_ATOMIC)) {
		case -EPIPE:
			usbnet_defer_kevent (dev, EVENT_RX_HALT);
			break;
		case -ENOMEM:
			usbnet_defer_kevent (dev, EVENT_RX_MEMORY);
			break;
		case -ENODEV:
			netif_dbg(dev, ifdown, dev->net, "device gone\n");
			netif_device_detach (dev->net);
			break;
		case -EHOSTUNREACH:
			retval = -ENOLINK;
			break;
		default:
			netif_dbg(dev, rx_err, dev->net,
				  "rx submit, %d\n", retval);
			tasklet_schedule (&dev->bh);
			break;
		case 0:
			list_add_tail (&entry->list, &dev->rxq.queue);
                        ++dev->rxq.qlen;
		}

	} else {
		netif_dbg(dev, ifdown, dev->net, "rx: stopped\n");
		retval = -ENOLINK;
	}

	if (retval) {
            entry->buf = NULL;
            entry->urb = NULL;
            dev->rxq.pool[dev->rxq.psize++] = entry;
        }
        spin_unlock_irqrestore(&dev->rxq.lock, lockflags);

	if (retval) {
                if (likely(flags == GFP_ATOMIC)) __skb_bin_put_buffer_noinvalidate(buf);
                else {
                    local_bh_disable();
                    __skb_bin_put_buffer_noinvalidate(buf);
                    local_bh_enable();
                }
		usb_free_urb (urb);
	}
	return retval;
}


/*-------------------------------------------------------------------------*/

static inline void rx_process (struct usbnet *dev, struct fp_data *fpd)
{
	if (dev->driver_info->rx_fixup) {
            struct fp_buf local; // only for fixup passing
            fpb_fill(&local, fpd->buf - NET_SKB_PAD, NET_SKB_PAD, fpd->len);
	    if (!(dev->driver_info->rx_fixup (dev, &local))) {
#if 0
		/* With RX_ASSEMBLE, rx_fixup() must update counters */
		if (!(dev->driver_info->flags & FLAG_RX_ASSEMBLE))
			dev->net->stats.rx_errors++;
#endif
		//goto done;
            } else if (fpb_len(&local)) {
                // TODO ncm
#if 0 // NCM
                    if (dev->driver_info->flags & FLAG_MULTI_PACKET) {
                        __skb_bin_put_buffer_noinvalidate(fpd->buf);
                    } else {
#endif
                struct fp_buf *buf = &raw_cpu_ptr(&per_cpu_fp_state)->fpb;
                *buf = local;
                usbnet_skb_return(dev, buf);
                dev->rxq.pool[dev->rxq.psize++] = fpd;
		return;
            }
	} else {
            // else network stack removes extra byte if we forced a short packet
            if (fpd->len) {
                struct fp_buf *buf = fpb_build(fpd->buf - NET_SKB_PAD, NET_SKB_PAD, fpd->len);
                dev->rxq.pool[dev->rxq.psize++] = fpd;
                usbnet_skb_return(dev, buf);
                return;

            } else {
                netif_dbg(dev, rx_err, dev->net, "drop\n");
                dev->net->stats.rx_errors++;
            }
        }

//done:
        __skb_bin_put_buffer_noinvalidate(fpd->buf - NET_SKB_PAD);
        dev->rxq.pool[dev->rxq.psize++] = fpd;
}

/*-------------------------------------------------------------------------*/

static void rx_complete (struct urb *urb)
{
	struct fp_data		*entry = (struct fp_data *) urb->context;
	struct usbnet		*dev = entry->dev;
	int			urb_status = urb->status;

	entry->state = rx_done;
        entry->len = urb->actual_length;

	switch (urb_status) {
	/* success */
	case 0:
		if (urb->actual_length < dev->net->hard_header_len) {
			entry->state = rx_cleanup;
			dev->net->stats.rx_errors++;
			dev->net->stats.rx_length_errors++;
			netif_dbg(dev, rx_err, dev->net,
				  "rx length %d\n", urb->actual_length);
		}
		break;

	/* stalls need manual reset. this is rare ... except that
	 * when going through USB 2.0 TTs, unplug appears this way.
	 * we avoid the highspeed version of the ETIMEDOUT/EILSEQ
	 * storm, recovering as needed.
	 */
	case -EPIPE:
		dev->net->stats.rx_errors++;
		usbnet_defer_kevent (dev, EVENT_RX_HALT);
		// FALLTHROUGH

	/* software-driven interface shutdown */
	case -ECONNRESET:		/* async unlink */
	case -ESHUTDOWN:		/* hardware gone */
		netif_dbg(dev, ifdown, dev->net,
			  "rx shutdown, code %d\n", urb_status);
		goto block;

	/* we get controller i/o faults during khubd disconnect() delays.
	 * throttle down resubmits, to avoid log floods; just temporarily,
	 * so we still recover when the fault isn't a khubd delay.
	 */
	case -EPROTO:
	case -ETIME:
	case -EILSEQ:
		dev->net->stats.rx_errors++;
		if (!timer_pending (&dev->delay)) {
			mod_timer (&dev->delay, jiffies + THROTTLE_JIFFIES);
			netif_dbg(dev, link, dev->net,
				  "rx throttle %d\n", urb_status);
		}
block:
		entry->state = rx_cleanup;
#if 0 // resubmit
                entry->urb = NULL;
#endif
		break;

	/* data overrun ... flush fifo? */
	case -EOVERFLOW:
		dev->net->stats.rx_over_errors++;
		// FALLTHROUGH

	default:
		entry->state = rx_cleanup;
#if 0 // resubmit
                entry->urb = NULL;
#endif
		dev->net->stats.rx_errors++;
		netif_dbg(dev, rx_err, dev->net, "rx status %d\n", urb_status);
		break;
	}

        if (unlikely(entry->state == rx_cleanup)) entry->buf -= NET_SKB_PAD;

	defer_bh(dev, entry, &dev->rxq);

#if 0 // resubmit from irq, fp unsafe
	if (urb) {
		if (netif_running (dev->net) &&
		    !test_bit (EVENT_RX_HALT, &dev->flags)) {
			rx_submit (dev, urb, GFP_ATOMIC);
			return;
		}
		usb_free_urb (urb);
	}
#endif
	netif_dbg(dev, rx_err, dev->net, "no read resubmitted\n");
}

#if 0
/*-------------------------------------------------------------------------*/
void usbnet_pause_rx(struct usbnet *dev)
{
	set_bit(EVENT_RX_PAUSED, &dev->flags);

	netif_dbg(dev, rx_status, dev->net, "paused rx queue enabled\n");
}
EXPORT_SYMBOL_GPL(usbnet_pause_rx);

void usbnet_resume_rx(struct usbnet *dev)
{
	struct sk_buff *skb;
	int num = 0;

	clear_bit(EVENT_RX_PAUSED, &dev->flags);

	while ((skb = skb_dequeue(&dev->rxq_pause)) != NULL) {
		usbnet_skb_return(dev, skb);
		num++;
	}

	tasklet_schedule(&dev->bh);

	netif_dbg(dev, rx_status, dev->net,
		  "paused rx queue disabled, %d skbs requeued\n", num);
}
EXPORT_SYMBOL_GPL(usbnet_resume_rx);

void usbnet_purge_paused_rxq(struct usbnet *dev)
{
	skb_queue_purge(&dev->rxq_pause);
}
EXPORT_SYMBOL_GPL(usbnet_purge_paused_rxq);
#endif

/*-------------------------------------------------------------------------*/

// unlink pending rx/tx; completion handlers do all other cleanup

static int unlink_urbs (struct usbnet *dev, struct fp_queue *q)
{
	unsigned long		flags;
	int			count = q->qlen;
        struct list_head        *tmp, *pos;

	spin_lock_irqsave (&q->lock, flags);
        list_for_each_safe(pos, tmp, &q->queue) {
		int			retval;
                struct fp_data *entry = (struct fp_data*) pos;
		struct urb *urb = entry->urb;

		/*
		 * Get reference count of the URB to avoid it to be
		 * freed during usb_unlink_urb, which may trigger
		 * use-after-free problem inside usb_unlink_urb since
		 * usb_unlink_urb is always racing with .complete
		 * handler(include defer_bh).
		 */
		usb_get_urb(urb);
		// during some PM-driven resume scenarios,
		// these (async) unlinks complete immediately
		retval = usb_unlink_urb (urb);
		if (retval != -EINPROGRESS && retval != 0)
			netdev_dbg(dev->net, "unlink urb err, %d\n", retval);
		usb_put_urb(urb);
	}
	spin_unlock_irqrestore (&q->lock, flags);
	return count;
}

// Flush all pending rx urbs
// minidrivers may need to do this when the MTU changes

void usbnet_unlink_rx_urbs(struct usbnet *dev)
{
	if (netif_running(dev->net)) {
		(void) unlink_urbs (dev, &dev->rxq);
		tasklet_schedule(&dev->bh);
	}
}
EXPORT_SYMBOL_GPL(usbnet_unlink_rx_urbs);

/*-------------------------------------------------------------------------*/

// precondition: never called in_interrupt
static void usbnet_terminate_urbs(struct usbnet *dev)
{
	DECLARE_WAITQUEUE(wait, current);
	int temp;

	/* ensure there are no more active urbs */
	add_wait_queue(&dev->wait, &wait);
	set_current_state(TASK_UNINTERRUPTIBLE);
	temp = unlink_urbs(dev, &dev->txq) +
		unlink_urbs(dev, &dev->rxq);

	/* maybe wait for deletions to finish. */
	while (dev->rxq.qlen
		&& dev->txq.qlen
		&& dev->done.qlen) {
			schedule_timeout(msecs_to_jiffies(UNLINK_TIMEOUT_MS));
			set_current_state(TASK_UNINTERRUPTIBLE);
			netif_dbg(dev, ifdown, dev->net,
				  "waited for %d urb completions\n", temp);
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&dev->wait, &wait);
}

int usbnet_stop (struct net_device *net)
{
	struct usbnet		*dev = netdev_priv(net);
	struct driver_info	*info = dev->driver_info;
	int			retval, pm, mpn;

	clear_bit(EVENT_DEV_OPEN, &dev->flags);
	netif_stop_queue (net);

	netif_info(dev, ifdown, dev->net,
		   "stop stats: rx/tx %lu/%lu, errs %lu/%lu\n",
		   net->stats.rx_packets, net->stats.tx_packets,
		   net->stats.rx_errors, net->stats.tx_errors);

	/* to not race resume */
	pm = usb_autopm_get_interface(dev->intf);
	/* allow minidriver to stop correctly (wireless devices to turn off
	 * radio etc) */
	if (info->stop) {
		retval = info->stop(dev);
		if (retval < 0)
			netif_info(dev, ifdown, dev->net,
				   "stop fail (%d) usbnet usb-%s-%s, %s\n",
				   retval,
				   dev->udev->bus->bus_name, dev->udev->devpath,
				   info->description);
	}

	if (!(info->flags & FLAG_AVOID_UNLINK_URBS))
		usbnet_terminate_urbs(dev);

	usbnet_status_stop(dev);

	// usbnet_purge_paused_rxq(dev);

	mpn = !test_and_clear_bit(EVENT_NO_RUNTIME_PM, &dev->flags);

	/* deferred work (task, timer, softirq) must also stop.
	 * can't flush_scheduled_work() until we drop rtnl (later),
	 * else workers could deadlock; so make workers a NOP.
	 */
	dev->flags = 0;
	del_timer_sync (&dev->delay);
	tasklet_kill (&dev->bh);
	if (info->manage_power && mpn)
		info->manage_power(dev, 0);
	else
		usb_autopm_put_interface(dev->intf);

        skb_bin_release(net);

	return 0;
}
EXPORT_SYMBOL_GPL(usbnet_stop);

/*-------------------------------------------------------------------------*/

// posts reads, and enables write queuing

// precondition: never called in_interrupt

int usbnet_open (struct net_device *net)
{
	struct usbnet		*dev = netdev_priv(net);
	int			retval;
	struct driver_info	*info = dev->driver_info;

        skb_bin_request(net, dev->rx_urb_size);

	if ((retval = usb_autopm_get_interface(dev->intf)) < 0) {
		netif_info(dev, ifup, dev->net,
			   "resumption fail (%d) usbnet usb-%s-%s, %s\n",
			   retval,
			   dev->udev->bus->bus_name,
			   dev->udev->devpath,
			   info->description);
		goto done_nopm;
	}

	// put into "known safe" state
	if (info->reset && (retval = info->reset (dev)) < 0) {
		netif_info(dev, ifup, dev->net,
			   "open reset fail (%d) usbnet usb-%s-%s, %s\n",
			   retval,
			   dev->udev->bus->bus_name,
			   dev->udev->devpath,
			   info->description);
		goto done;
	}

	// insist peer be connected
	if (info->check_connect && (retval = info->check_connect (dev)) < 0) {
		netif_dbg(dev, ifup, dev->net, "can't open; %d\n", retval);
		goto done;
	}

	/* start any status interrupt transfer */
	if (dev->interrupt) {
		retval = usbnet_status_start(dev, GFP_KERNEL);
		if (retval < 0) {
			netif_err(dev, ifup, dev->net,
				  "intr submit %d\n", retval);
			goto done;
		}
	}

	set_bit(EVENT_DEV_OPEN, &dev->flags);
	netif_start_queue (net);
	netif_info(dev, ifup, dev->net,
		   "open: enable queueing (rx %d, tx %d) mtu %d %s framing\n",
		   (int)RX_QLEN(dev), (int)TX_QLEN(dev),
		   dev->net->mtu,
		   (dev->driver_info->flags & FLAG_FRAMING_NC) ? "NetChip" :
		   (dev->driver_info->flags & FLAG_FRAMING_GL) ? "GeneSys" :
		   (dev->driver_info->flags & FLAG_FRAMING_Z) ? "Zaurus" :
		   (dev->driver_info->flags & FLAG_FRAMING_RN) ? "RNDIS" :
		   (dev->driver_info->flags & FLAG_FRAMING_AX) ? "ASIX" :
		   "simple");

	// delay posting reads until we're fully open
	tasklet_schedule (&dev->bh);
	if (info->manage_power) {
		retval = info->manage_power(dev, 1);
		if (retval < 0) {
			retval = 0;
			set_bit(EVENT_NO_RUNTIME_PM, &dev->flags);
		} else {
			usb_autopm_put_interface(dev->intf);
		}
	}
	return retval;

done:
	usb_autopm_put_interface(dev->intf);
done_nopm:
	return retval;
}
EXPORT_SYMBOL_GPL(usbnet_open);

/*-------------------------------------------------------------------------*/

/* ethtool methods; minidrivers may need to add some more, but
 * they'll probably want to use this base set.
 */

int usbnet_get_link_ksettings(struct net_device *net,
			      struct ethtool_link_ksettings *cmd)
{
	struct usbnet *dev = netdev_priv(net);

	if (!dev->mii.mdio_read)
		return -EOPNOTSUPP;

	mii_ethtool_get_link_ksettings(&dev->mii, cmd);

	return 0;
}
EXPORT_SYMBOL_GPL(usbnet_get_link_ksettings);

int usbnet_set_link_ksettings(struct net_device *net,
			      const struct ethtool_link_ksettings *cmd)
{
	struct usbnet *dev = netdev_priv(net);
	int retval;

	if (!dev->mii.mdio_write)
		return -EOPNOTSUPP;

	retval = mii_ethtool_set_link_ksettings(&dev->mii, cmd);

	/* link speed/duplex might have changed */
	if (dev->driver_info->link_reset)
		dev->driver_info->link_reset(dev);

	/* hard_mtu or rx_urb_size may change in link_reset() */
	//usbnet_update_max_qlen(dev);

	return retval;
}
EXPORT_SYMBOL_GPL(usbnet_set_link_ksettings);

void usbnet_get_stats64(struct net_device *net, struct rtnl_link_stats64 *stats)
{
	struct usbnet *dev = netdev_priv(net);
	unsigned int start;
	int cpu;

	netdev_stats_to_stats64(stats, &net->stats);

	for_each_possible_cpu(cpu) {
		struct pcpu_sw_netstats *stats64;
		u64 rx_packets, rx_bytes;
		u64 tx_packets, tx_bytes;

		stats64 = per_cpu_ptr(dev->stats64, cpu);

		do {
			start = u64_stats_fetch_begin_irq(&stats64->syncp);
			rx_packets = stats64->rx_packets;
			rx_bytes = stats64->rx_bytes;
			tx_packets = stats64->tx_packets;
			tx_bytes = stats64->tx_bytes;
		} while (u64_stats_fetch_retry_irq(&stats64->syncp, start));

		stats->rx_packets += rx_packets;
		stats->rx_bytes += rx_bytes;
		stats->tx_packets += tx_packets;
		stats->tx_bytes += tx_bytes;
	}
}
EXPORT_SYMBOL_GPL(usbnet_get_stats64);

u32 usbnet_get_link (struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	/* If a check_connect is defined, return its result */
	if (dev->driver_info->check_connect)
		return dev->driver_info->check_connect (dev) == 0;

	/* if the device has mii operations, use those */
	if (dev->mii.mdio_read)
		return mii_link_ok(&dev->mii);

	/* Otherwise, dtrt for drivers calling netif_carrier_{on,off} */
	return ethtool_op_get_link(net);
}
EXPORT_SYMBOL_GPL(usbnet_get_link);

int usbnet_nway_reset(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	if (!dev->mii.mdio_write)
		return -EOPNOTSUPP;

	return mii_nway_restart(&dev->mii);
}
EXPORT_SYMBOL_GPL(usbnet_nway_reset);

void usbnet_get_drvinfo (struct net_device *net, struct ethtool_drvinfo *info)
{
	struct usbnet *dev = netdev_priv(net);

	strlcpy (info->driver, dev->driver_name, sizeof info->driver);
	strlcpy (info->version, DRIVER_VERSION, sizeof info->version);
	strlcpy (info->fw_version, dev->driver_info->description,
		sizeof info->fw_version);
	usb_make_path (dev->udev, info->bus_info, sizeof info->bus_info);
}
EXPORT_SYMBOL_GPL(usbnet_get_drvinfo);

u32 usbnet_get_msglevel (struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return dev->msg_enable;
}
EXPORT_SYMBOL_GPL(usbnet_get_msglevel);

void usbnet_set_msglevel (struct net_device *net, u32 level)
{
	struct usbnet *dev = netdev_priv(net);

	dev->msg_enable = level;
}
EXPORT_SYMBOL_GPL(usbnet_set_msglevel);

/* drivers may override default ethtool_ops in their bind() routine */
static const struct ethtool_ops usbnet_ethtool_ops = {
	.get_link		= usbnet_get_link,
	.nway_reset		= usbnet_nway_reset,
	.get_drvinfo		= usbnet_get_drvinfo,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_ts_info		= ethtool_op_get_ts_info,
	.get_link_ksettings	= usbnet_get_link_ksettings,
	.set_link_ksettings	= usbnet_set_link_ksettings,
};

/*-------------------------------------------------------------------------*/

static void __handle_link_change(struct usbnet *dev)
{
	if (!test_bit(EVENT_DEV_OPEN, &dev->flags))
		return;

	if (!netif_carrier_ok(dev->net)) {
		/* kill URBs for reading packets to save bus bandwidth */
		unlink_urbs(dev, &dev->rxq);

		/*
		 * tx_timeout will unlink URBs for sending packets and
		 * tx queue is stopped by netcore after link becomes off
		 */
	} else {
		/* submitting URBs for reading packets */
		tasklet_schedule(&dev->bh);
	}

	/* hard_mtu or rx_urb_size may change during link change */
	//usbnet_update_max_qlen(dev);

	clear_bit(EVENT_LINK_CHANGE, &dev->flags);
}

static void usbnet_set_rx_mode(struct net_device *net)
{
	struct usbnet		*dev = netdev_priv(net);

	usbnet_defer_kevent(dev, EVENT_SET_RX_MODE);
}

static void __handle_set_rx_mode(struct usbnet *dev)
{
	if (dev->driver_info->set_rx_mode)
		(dev->driver_info->set_rx_mode)(dev);

	clear_bit(EVENT_SET_RX_MODE, &dev->flags);
}

/* work that cannot be done in interrupt context uses keventd.
 *
 * NOTE:  with 2.5 we could do more of this using completion callbacks,
 * especially now that control transfers can be queued.
 */
static void
usbnet_deferred_kevent (struct work_struct *work)
{
	struct usbnet		*dev =
		container_of(work, struct usbnet, kevent);
	int			status;

        if (!dev) return;

	/* usb_clear_halt() needs a thread context */
	if (test_bit (EVENT_TX_HALT, &dev->flags)) {
		unlink_urbs (dev, &dev->txq);
		status = usb_autopm_get_interface(dev->intf);
		if (status < 0)
			goto fail_pipe;
		status = usb_clear_halt (dev->udev, dev->out);
		usb_autopm_put_interface(dev->intf);
		if (status < 0 &&
		    status != -EPIPE &&
		    status != -ESHUTDOWN) {
			if (netif_msg_tx_err (dev))
fail_pipe:
				netdev_err(dev->net, "can't clear tx halt, status %d\n",
					   status);
		} else {
			clear_bit (EVENT_TX_HALT, &dev->flags);
			if (status != -ESHUTDOWN)
				netif_wake_queue (dev->net);
		}
	}
	if (test_bit (EVENT_RX_HALT, &dev->flags)) {
		unlink_urbs (dev, &dev->rxq);
		status = usb_autopm_get_interface(dev->intf);
		if (status < 0)
			goto fail_halt;
		status = usb_clear_halt (dev->udev, dev->in);
		usb_autopm_put_interface(dev->intf);
		if (status < 0 &&
		    status != -EPIPE &&
		    status != -ESHUTDOWN) {
			if (netif_msg_rx_err (dev))
fail_halt:
				netdev_err(dev->net, "can't clear rx halt, status %d\n",
					   status);
		} else {
			clear_bit (EVENT_RX_HALT, &dev->flags);
			tasklet_schedule (&dev->bh);
		}
	}

	/* tasklet could resubmit itself forever if memory is tight */
	if (test_bit (EVENT_RX_MEMORY, &dev->flags)) {
		struct urb	*urb = NULL;
		int resched = 1;

		if (netif_running (dev->net))
			urb = usb_alloc_urb (0, GFP_KERNEL);
		else
			clear_bit (EVENT_RX_MEMORY, &dev->flags);
		if (urb != NULL) {
			clear_bit (EVENT_RX_MEMORY, &dev->flags);
			status = usb_autopm_get_interface(dev->intf);
			if (status < 0) {
				usb_free_urb(urb);
				goto fail_lowmem;
			}
			if (rx_submit (dev, urb, GFP_KERNEL) == -ENOLINK)
				resched = 0;
			usb_autopm_put_interface(dev->intf);
fail_lowmem:
			if (resched)
				tasklet_schedule (&dev->bh);
		}
	}

	if (test_bit (EVENT_LINK_RESET, &dev->flags)) {
		struct driver_info	*info = dev->driver_info;
		int			retval = 0;

		clear_bit (EVENT_LINK_RESET, &dev->flags);
		status = usb_autopm_get_interface(dev->intf);
		if (status < 0)
			goto skip_reset;
		if(info->link_reset && (retval = info->link_reset(dev)) < 0) {
			usb_autopm_put_interface(dev->intf);
skip_reset:
			netdev_info(dev->net, "link reset failed (%d) usbnet usb-%s-%s, %s\n",
				    retval,
				    dev->udev->bus->bus_name,
				    dev->udev->devpath,
				    info->description);
		} else {
			usb_autopm_put_interface(dev->intf);
		}

		/* handle link change from link resetting */
		__handle_link_change(dev);
	}

	if (test_bit (EVENT_LINK_CHANGE, &dev->flags))
		__handle_link_change(dev);

	if (test_bit (EVENT_SET_RX_MODE, &dev->flags))
		__handle_set_rx_mode(dev);


	if (dev->flags)
		netdev_dbg(dev->net, "kevent done, flags = 0x%lx\n", dev->flags);
}

/*-------------------------------------------------------------------------*/

static void tx_complete (struct urb *urb)
{
	struct fp_data		*entry = (struct fp_data *) urb->context;
	struct usbnet		*dev = entry->dev;

	if (urb->status == 0) {
		struct pcpu_sw_netstats *stats64 = this_cpu_ptr(dev->stats64);

		u64_stats_update_begin(&stats64->syncp);
		stats64->tx_packets += entry->packets;
		stats64->tx_bytes += entry->reallen;
		u64_stats_update_end(&stats64->syncp);
	} else {
		dev->net->stats.tx_errors++;

		switch (urb->status) {
		case -EPIPE:
			usbnet_defer_kevent (dev, EVENT_TX_HALT);
			break;

		/* software-driven interface shutdown */
		case -ECONNRESET:		// async unlink
		case -ESHUTDOWN:		// hardware gone
			break;

		// like rx, tx gets controller i/o faults during khubd delays
		// and so it uses the same throttling mechanism.
		case -EPROTO:
		case -ETIME:
		case -EILSEQ:
			usb_mark_last_busy(dev->udev);
			if (!timer_pending (&dev->delay)) {
				mod_timer (&dev->delay,
					jiffies + THROTTLE_JIFFIES);
				netif_dbg(dev, link, dev->net,
					  "tx throttle %d\n", urb->status);
			}
			netif_stop_queue (dev->net);
			break;
		default:
			netif_dbg(dev, tx_err, dev->net,
				  "tx err %d\n", entry->urb->status);
			break;
		}
	}

	usb_autopm_put_interface_async(dev->intf);
	entry->state = tx_done;

	defer_bh(dev, entry, &dev->txq);
}

/*-------------------------------------------------------------------------*/

void usbnet_tx_timeout (struct net_device *net, unsigned txqueue)
{
	struct usbnet		*dev = netdev_priv(net);

	unlink_urbs (dev, &dev->txq);
	tasklet_schedule (&dev->bh);
	/* this needs to be handled individually because the generic layer
	 * doesn't know what is sufficient and could not restore private
	 * information if a remedy of an unconditional reset were used.
	 */
	if (dev->driver_info->recover)
		(dev->driver_info->recover)(dev);
}
EXPORT_SYMBOL_GPL(usbnet_tx_timeout);

/*-------------------------------------------------------------------------*/

int usbnet_fast_path_xmit (struct net_device *net, struct fp_buf *fpb)
{
	struct usbnet		*dev = netdev_priv(net);
	struct urb		*urb = NULL;
	struct driver_info	*info = dev->driver_info;
	unsigned long           flags;
        struct fp_data          *entry = NULL;
	int                     retval = 0;
	int                     reallen = 0;

        if (!list_empty(&br_list)) {
            struct ethhdr *eth = (struct ethhdr *) fpb_data(fpb);
            struct br_entry *br;
            int drop = 1;
            int vlan_id;

            rcu_read_lock();
            list_for_each_entry_rcu(br, &br_list, list) {
                if (unlikely(br->modem != net)) continue;
                vlan_id = br->vlan_id;

                // forwarded tx
                if (br->slave) {
                    if (ether_addr_equal_64bits(eth->h_source, br->dst_mac)) {
                        DEBUG_PRINTK {
                            if (likely(eth->h_proto == __constant_htons(ETH_P_IP))) {
                                struct iphdr *ip = (struct iphdr*) (eth + 1);
                                if (!br->single_apn && br->dst_ip != ip->saddr) continue;
                            }

                            printk("tx modem (fw) %pI4, %pM\n", &br->dst_ip, eth->h_source);
                        }
                        drop = 0;
                        break;
                    }

                // local tx
                } else {
                    if (likely(eth->h_proto == __constant_htons(ETH_P_IP))) {
                        struct iphdr *ip = (struct iphdr*) (eth + 1);
                        if (br->dst_ip == ip->saddr) {
                            DEBUG_PRINTK printk("tx modem (local ip) %pI4\n", &br->dst_ip);
                            drop = 0;
                            break;
                        }
                    } else if (eth->h_proto == __constant_htons(ETH_P_ARP)) {
                        DEBUG_PRINTK printk("tx modem (local arp)\n");
                        drop = 0;
                        break;
                    }
                }
            }
            rcu_read_unlock();

            if (drop) goto drop_no_stat;

            if (vlan_id) {
                struct vlan_ethhdr *hdr;
                if (fpb_headroom(fpb) < sizeof(struct vlan_hdr)) {
                    goto drop;
                }

                fpb_push(fpb, sizeof(struct vlan_hdr));
                memmove12minus4(fpb_data(fpb));

                hdr = (struct vlan_ethhdr *)fpb_data(fpb);
                hdr->h_vlan_proto = __constant_htons(ETH_P_8021Q);
                hdr->h_vlan_TCI = htons(vlan_id);
            }
        }

        reallen = fpb_len(fpb);
	// some devices want funky USB-level framing, for
	// win32 driver (usually) and/or hardware quirks
	if (likely(info->tx_fixup)) {
                if (!dev->txq.psize) {
                        netif_dbg(dev, tx_err, dev->net, "no urb\n");
                        goto drop;
                }
		if (!info->tx_fixup (dev, fpb, GFP_ATOMIC)) {
			/* packet collected; minidriver waiting for more */
			if (info->flags & FLAG_MULTI_PACKET)
				goto drop_no_stat;
                        netif_dbg(dev, tx_err, dev->net, "can't tx_fixup skb\n");
                        goto drop;
		}
	}

	spin_lock_irqsave(&dev->txq.lock, flags);
	if (dev->txq.psize) {
            entry = dev->txq.pool[--dev->txq.psize];
            dev->txq.pool[dev->txq.psize] = NULL;
        }
	spin_unlock_irqrestore(&dev->txq.lock, flags);

        if (!entry) goto drop;

	if (!(urb = usb_alloc_urb (0, GFP_ATOMIC))) {
		netif_dbg(dev, tx_err, dev->net, "no urb\n");
		goto drop;
	}


	entry->buf = fpb_buf(fpb);
        entry->len = fpb_len(fpb);
        entry->reallen = reallen;
	entry->urb = urb;
	entry->dev = dev;
	entry->state = tx_start;

	usb_fill_bulk_urb (urb, dev->udev, dev->out,
			fpb_data(fpb), fpb_len(fpb), tx_complete, entry);

	/* don't assume the hardware handles USB_ZERO_PACKET
	 * NOTE:  strictly conforming cdc-ether devices should expect
	 * the ZLP here, but ignore the one-byte packet.
	 * NOTE2: CDC NCM specification is different from CDC ECM when
	 * handling ZLP/short packets, so cdc_ncm driver will make short
	 * packet itself if needed.
	 */
	if (fpb_len(fpb) % dev->maxpacket == 0) {
		if (!(info->flags & FLAG_SEND_ZLP)) {
			if (!(info->flags & FLAG_MULTI_PACKET)) {
				urb->transfer_buffer_length++;
                                if (fpb_tailroom(fpb) >= 1) {
					fpb_data(fpb)[fpb_len(fpb)] = 0;
					fpb_len_set(fpb, fpb_len(fpb) + 1);
				} else {
                                    netif_err(dev, tx_err, dev->net, "no zlp tailroom\n");
                                }
			}
		} else
			urb->transfer_flags |= URB_ZERO_PACKET;
	}

	if (info->flags & FLAG_MULTI_PACKET) {
		/* Driver has set number of packets and a length delta.
		 * Calculate the complete length and ensure that it's
		 * positive.
		 */
// TODO  NCM
#if 0
		entry->length += urb->transfer_buffer_length;
		if (WARN_ON_ONCE(entry->length <= 0))
			entry->length = length;
#endif
	} else {
		usbnet_set_skb_tx_stats(entry, 1, reallen);
	}

        // XXX xmit commit
        spin_lock_irqsave(&dev->txq.lock, flags);
	retval = usb_autopm_get_interface_async(dev->intf);
	if (retval < 0) {
                spin_unlock_irqrestore(&dev->txq.lock, flags);
		goto drop;
	}

	switch ((retval = usb_submit_urb (urb, GFP_ATOMIC))) {
	case -EPIPE:
		netif_stop_queue (net);
		usbnet_defer_kevent (dev, EVENT_TX_HALT);
		usb_autopm_put_interface_async(dev->intf);
		break;
	default:
		usb_autopm_put_interface_async(dev->intf);
		netif_dbg(dev, tx_err, dev->net,
			  "tx: submit urb err %d\n", retval);
		break;
	case 0:
		netif_trans_update(net);
                list_add_tail(&entry->list, &dev->txq.queue);
		if (++dev->txq.qlen >= TX_QLEN (dev))
			netif_stop_queue (net);
	}
        spin_unlock_irqrestore(&dev->txq.lock, flags);

	if (retval) {
		netif_dbg(dev, tx_err, dev->net, "drop, code %d\n", retval);
drop:
		dev->net->stats.tx_dropped++;
drop_no_stat:
                __skb_bin_put_buffer_noinvalidate(fpb_buf(fpb));
		if (urb) usb_free_urb (urb);
                if (entry) {
                    spin_lock_irqsave(&dev->txq.lock, flags);
                    dev->txq.pool[dev->txq.psize++] = entry;
                    spin_unlock_irqrestore(&dev->txq.lock, flags);
                }
        }

	return 1;
}
EXPORT_SYMBOL_GPL(usbnet_fast_path_xmit);

static int rx_alloc_submit(struct usbnet *dev, gfp_t flags)
{
	struct urb	*urb;
	int		i;
	int		ret = 0;

	/* don't refill the queue all at once */
        for (i = 0; i < 16 && (dev->rxq.qlen + i) < RX_QLEN(dev); i++) {
		urb = usb_alloc_urb(0, flags);
		if (urb != NULL) {
			ret = rx_submit(dev, urb, flags);
			if (ret)
				goto err;
		} else {
			ret = -ENOMEM;
			goto err;
		}
	}
err:
	return ret;
}

/*-------------------------------------------------------------------------*/

static void cleanup_done (struct usbnet *dev, bool rx) {
        struct list_head        *head = &dev->done.queue;
        unsigned long           flags;

        spin_lock_irqsave(&dev->done.lock, flags);
        while (dev->done.qlen && head->next != head) {
                struct list_head        *pos = head->next;
                struct fp_data *entry = (struct fp_data*) pos;
                list_del(&entry->list);
                --dev->done.qlen;
                spin_unlock_irqrestore(&dev->done.lock, flags);

		switch (entry->state) {
		case rx_done:
                        if (entry->urb) usb_free_urb (entry->urb);
			if (rx) rx_process (dev, entry);
                        else {
                            __skb_bin_put_buffer_noinvalidate(entry->buf - NET_SKB_PAD);
                            dev->rxq.pool[dev->rxq.psize++] = entry;
                            dev->net->stats.rx_dropped++;
                        }
			break;

		case tx_done:
		case rx_cleanup:
                        if (entry->urb) usb_free_urb (entry->urb);
                        if (entry->buf) __skb_bin_put_buffer_noinvalidate(entry->buf);

                        if (entry->state == tx_done) {
                            spin_lock_irqsave(&dev->txq.lock, flags);
                            dev->txq.pool[dev->txq.psize++] = entry;
                            spin_unlock_irqrestore(&dev->txq.lock, flags);
                        } else {
                            spin_lock_irqsave(&dev->rxq.lock, flags);
                            dev->rxq.pool[dev->rxq.psize++] = entry;
                            spin_unlock_irqrestore(&dev->rxq.lock, flags);
                        }
			break;
		default:
			netdev_err(dev->net, "bogus skb state %d\n", entry->state);
			break;
		}

                spin_lock_irqsave(&dev->done.lock, flags);
        }
        spin_unlock_irqrestore(&dev->done.lock, flags);
}

// tasklet/timer
static void usbnet_bh (unsigned long param)
{
	struct usbnet		*dev = (struct usbnet *) param;

        if (!dev) return;

        cleanup_done(dev, true);

	/* restart RX again after disabling due to high error rate */
	clear_bit(EVENT_RX_KILL, &dev->flags);

	/* waiting for all pending urbs to complete?
	 * only then can we forgo submitting anew
	 */
	if (waitqueue_active(&dev->wait)) {
		if (dev->txq.qlen + dev->rxq.qlen + dev->done.qlen == 0)
			wake_up_all(&dev->wait);

	// or are we maybe short a few urbs?
	} else if (netif_running (dev->net) &&
		   netif_device_present (dev->net) &&
		   netif_carrier_ok(dev->net) &&
		   !timer_pending (&dev->delay) &&
		   !test_bit (EVENT_RX_HALT, &dev->flags)) {
		int	temp = dev->rxq.qlen;

		if (temp < RX_QLEN(dev)) {
			if (rx_alloc_submit(dev, GFP_ATOMIC) == -ENOLINK)
				return;
			if (temp != dev->rxq.qlen)
				netif_dbg(dev, link, dev->net,
					  "rxqlen %d --> %d\n",
					  temp, dev->rxq.qlen);
			if (dev->rxq.qlen < RX_QLEN(dev))
				tasklet_schedule (&dev->bh);
		}
		if (dev->txq.qlen < TX_QLEN (dev))
			netif_wake_queue (dev->net);
	}

        call_xmit_commits();
}

static void usbnet_delay(struct timer_list *t) {
	struct usbnet *dev = from_timer(dev, t, delay);
	usbnet_bh((unsigned long) dev);
}

static int unregister_slave(struct net_device *slave, struct net_device *modem) {
    int ret = -ENOENT;
    struct br_entry *br, *tmp;

    list_for_each_entry_safe(br, tmp, &br_list, list) {
        if (slave) {
            if (br->slave != slave) continue;
        } else {
            if (br->modem != modem) continue;
        }

        if (slave) printk("usbnet: slave %s unreg %x\n", slave->name, br->dst_ip);
        else printk("usbnet: master %s unreg %x\n", modem->name, br->dst_ip);

        unregister_fast_path(&fp_rx_handler, br->slave);

        list_del_rcu(&br->list);
        synchronize_rcu();

        if (br->slave) dev_put(br->slave);
        kfree(br);

        ret = 0;

        if (slave) break;
    }

    return ret;
}

int usbnet_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd) {
    struct usbnet_params p;
    struct br_entry *br;

    if (cmd != USBNET_BR_IFACE) return -EINVAL;

    if (copy_from_user(&p, ifr->ifr_ifru.ifru_data, sizeof(p))) {
        printk(KERN_ERR "usbnet: copy from user failed\n");
        return -EINVAL;
    }

    if (!p.add) {
        return unregister_slave(NULL, dev);

    } else {
        struct net_device *slave = NULL;
        if (p.slave_id) {
            slave = dev_get_by_index(&init_net, p.slave_id);
            if (!slave) return -ENODEV;
        } else { // filter
            if (!p.dst_ip) return -EINVAL;
        }

        list_for_each_entry(br, &br_list, list) {
            if (br->modem == dev) {
                if (br->dst_ip == p.dst_ip) return -EEXIST;
                if (slave && br->slave == slave) {
                    dev_put(slave);
                    return -EEXIST;
                }
            }
        }

        if (slave) register_fast_path(&fp_rx_handler, slave);

        br = kzalloc(sizeof(struct br_entry), GFP_KERNEL);
        br->slave = slave;
        br->modem = dev;
        br->dst_ip = p.dst_ip;
        br->vlan_id = p.vlan_id;
        br->single_apn = p.single_apn;
        br->mac_update = p.mac_update;
        memcpy(br->dst_mac, p.dst_mac, ETH_ALEN);

        list_add_tail_rcu(&br->list, &br_list);

        if (slave) printk("usbnet: passthrough %s -> %s %d %pM\n", dev->name, slave->name, br->vlan_id, br->dst_mac);
        else printk("usbnet: passthrough %s -> local %x\n", dev->name, br->dst_ip);

        return 0;
    }

    return -EINVAL;
}

static struct net_device *passthrough_fwd_handler(struct net_device *in_dev, struct fp_buf *fpb) {
    struct ethhdr *eth = (struct ethhdr *) fpb_data(fpb);
    const unsigned proto = eth->h_proto;
    struct br_entry *br;

    rcu_read_lock();
    list_for_each_entry_rcu(br, &br_list, list) {
        if (br->slave != in_dev) continue;
        if (!ether_addr_equal_64bits(eth->h_source, br->dst_mac)) continue;

        rcu_read_unlock();

        if (ANY_VLAN_PROTO_N(proto) && !(in_dev->priv_flags & IFF_802_1Q_VLAN)) return NULL;

        // rx all
        if (likely(!br->dst_ip)) return br->modem;

        if (likely(proto == __constant_htons(ETH_P_IP))) {
            struct iphdr *ip = (struct iphdr*) (eth + 1);
            if (ip->protocol == IPPROTO_UDP) {
                struct udphdr *udp = (struct udphdr*) (ip + 1);
                // rx DHCP localy if not relayed
                if (unlikely(udp->source == __constant_htons(68))) {
                    return NULL;
                }
            }

            DEBUG_PRINTK printk("rx slave (ip) %pI4, %pM\n", &br->dst_ip, eth->h_source);

            // rx IP only
            fast_path_fp_tx_inc(br->modem, fpb);
            if (br->mac_update) memcpy(eth->h_dest, br->modem_mac, ETH_ALEN);
            return br->modem;

        // rx ARP localy
        } else if (proto == __constant_htons(ETH_P_ARP)) {
            struct arphdr *arp = (struct arphdr*) (eth + 1);
            if (arp->ar_op == htons(ARPOP_REQUEST)){
                DEBUG_PRINTK printk("rx slave (arp) %pI4\n", &br->dst_ip);
                return NULL;
            }
        }

        if (br->single_apn) {
            // pass the rest
            fast_path_fp_tx_inc(br->modem, fpb);
            if (br->mac_update) memcpy(eth->h_dest, br->modem_mac, ETH_ALEN);
            return br->modem;

        } else {
            // drop the rest
            DEBUG_PRINTK printk("rx slave (drop) %pI4\n", &br->dst_ip);
            __fpb_free(fpb);
            return FAST_PATH_CONSUMED;
        }
    }
    rcu_read_unlock();

    return NULL;
}

static int usbnet_device_changed(struct notifier_block *self,
			       unsigned long event, void *d) {
    if (event == NETDEV_UNREGISTER) {
        unregister_slave(netdev_notifier_info_to_dev(d), NULL);
    }
    return 0;
}

static struct notifier_block dev_notify = {
    usbnet_device_changed,
    NULL,
    1100
};
/*-------------------------------------------------------------------------
 *
 * USB Device Driver support
 *
 *-------------------------------------------------------------------------*/

// precondition: never called in_interrupt

void usbnet_disconnect (struct usb_interface *intf)
{
	struct usbnet		*dev;
	struct usb_device	*xdev;
	struct net_device	*net;

	dev = usb_get_intfdata(intf);
	if (!dev) return;

        dev->bh.data = 0;
        cancel_work_sync(&dev->probe);
        cancel_work_sync(&dev->kevent);

	usb_set_intfdata(intf, NULL);

        net = dev->net;
        if (!net || net->reg_state != NETREG_REGISTERED) return;

	xdev = interface_to_usbdev (intf);

	netif_info(dev, probe, dev->net, "unregister '%s' usb-%s-%s, %s\n",
		   intf->dev.driver->name,
		   xdev->bus->bus_name, xdev->devpath,
		   dev->driver_info->description);

        unregister_slave(NULL, net);
	unregister_netdev (net);

	if (dev->driver_info->unbind)
		dev->driver_info->unbind (dev, intf);

	if (dev->interrupt) {
            usb_kill_urb(dev->interrupt);
            usb_free_urb(dev->interrupt);
        }

        cleanup_done(dev, false);
        if (dev->rxq.mem) kfree(dev->rxq.mem);
        if (dev->txq.mem) kfree(dev->txq.mem);

	free_percpu(dev->stats64);
	usb_put_dev (xdev);
}
EXPORT_SYMBOL_GPL(usbnet_disconnect);

static const struct net_device_ops usbnet_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= fast_path_start_xmit,
	.ndo_fast_path_xmit	= usbnet_fast_path_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_set_rx_mode	= usbnet_set_rx_mode,
	.ndo_change_mtu		= usbnet_change_mtu,
	.ndo_get_stats64	= usbnet_get_stats64,
	.ndo_set_mac_address 	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl	        = usbnet_ioctl,
};

/*-------------------------------------------------------------------------*/

// precondition: never called in_interrupt

static struct device_type wlan_type = {
	.name	= "wlan",
};

static struct device_type wwan_type = {
	.name	= "wwan",
};

static void usbnet_probe_finalize (struct work_struct *work);
int
usbnet_probe (struct usb_interface *udev, const struct usb_device_id *prod)
{
	struct usbnet			*dev;
	struct net_device		*net;
	struct driver_info		*info;
	struct usb_device		*xdev;
	int				status;
	const char			*name;
	struct usb_driver 	*driver = to_usb_driver(udev->dev.driver);

	/* usbnet already took usb runtime pm, so have to enable the feature
	 * for usb interface, otherwise usb_autopm_get_interface may return
	 * failure if USB_SUSPEND(RUNTIME_PM) is enabled.
	 */
	if (!driver->supports_autosuspend) {
		driver->supports_autosuspend = 1;
		pm_runtime_enable(&udev->dev);
	}

	name = udev->dev.driver->name;
	info = (struct driver_info *) prod->driver_info;
	if (!info) {
		dev_err (&udev->dev, "blacklisted by %s\n", name);
		return -ENODEV;
	}
	xdev = interface_to_usbdev (udev);

	usb_get_dev (xdev);

	status = -ENOMEM;

	// set up our own records
	net = alloc_etherdev(sizeof(*dev));
	if (!net) {
		dev_dbg (&udev->dev, "can't kmalloc dev");
		goto out;
	}

	/* netdev_printk() needs this so do it as early as possible */
	SET_NETDEV_DEV(net, &udev->dev);

	dev = netdev_priv(net);
        dev->ndev = net;
	dev->udev = xdev;
	dev->intf = udev;
	dev->driver_info = info;
	dev->driver_name = name;

	dev->stats64 = netdev_alloc_pcpu_stats(struct pcpu_sw_netstats);
	if (!dev->stats64)
		goto out;

	dev->msg_enable = netif_msg_init (msg_level, NETIF_MSG_DRV
				| NETIF_MSG_PROBE | NETIF_MSG_LINK);
	init_waitqueue_head(&dev->wait);

	INIT_LIST_HEAD (&dev->rxq.queue);
        dev->rxq.psize = FP_QUEUE_SIZE;
	INIT_LIST_HEAD (&dev->txq.queue);
        dev->txq.psize = FP_QUEUE_SIZE;
	INIT_LIST_HEAD (&dev->done.queue);
	
	dev->bh.func = usbnet_bh;
	dev->bh.data = (unsigned long) dev;
	INIT_WORK (&dev->kevent, usbnet_deferred_kevent);
	INIT_WORK (&dev->probe, usbnet_probe_finalize);
//	init_usb_anchor(&dev->deferred);
	timer_setup(&dev->delay, usbnet_delay, 0);
	mutex_init (&dev->phy_mutex);
	mutex_init (&dev->interrupt_mutex);
	dev->interrupt_count = 0;

	dev->net = net;
	strcpy (net->name, "usb%d");
	memcpy (net->dev_addr, node_id, sizeof node_id);

	/* rx and tx sides can use different message sizes;
	 * bind() should set rx_urb_size in that case.
	 */
        net->min_mtu = 0;
        net->max_mtu = ETH_MAX_MTU;
        net->hard_header_len += 1; // ZLP
	dev->hard_mtu = net->mtu + net->hard_header_len;
#if 0
// dma_supported() is deeply broken on almost all architectures
	// possible with some EHCI controllers
	if (dma_supported (&udev->dev, DMA_BIT_MASK(64)))
		net->features |= NETIF_F_HIGHDMA;
#endif

        net->needs_free_netdev = true;
	net->netdev_ops = &usbnet_netdev_ops;
	net->watchdog_timeo = TX_TIMEOUT_JIFFIES;
	net->ethtool_ops = &usbnet_ethtool_ops;

        if (info->bind_init) {
            status = info->bind_init(dev, udev);
            if (status < 0) {
                goto out;
            }
        }

	usb_set_intfdata (udev, dev);

	schedule_work (&dev->probe);

	return 0;

out:
	usb_put_dev(xdev);
	return status;
}

static void usbnet_probe_finalize (struct work_struct *work) {
        struct usbnet		        *dev = container_of(work, struct usbnet, probe);
        struct usb_interface            *udev = dev->intf;
        struct usb_host_interface	*interface = udev->cur_altsetting;
        struct usb_device	        *xdev = interface_to_usbdev (udev);
        struct net_device		*net = dev->net;
        struct driver_info		*info = dev->driver_info;
        int				status, i;

	if (!dev->rx_urb_size) {
            // set to initial packet_hook buffer size to avoid interface flap
            // good enough for rndis too
#define SKB_SHARED_INFO_SIZE SKB_DATA_ALIGN(sizeof(struct skb_shared_info))
            dev->rx_urb_size = 2048 - SKB_SHARED_INFO_SIZE - NET_SKB_PAD;
        }

	// allow device-specific bind/init procedures
	// NOTE net->name still not usable ...
	if (info->bind || info->bind_init) {
                if (info->bind) {
                    status = info->bind (dev, udev);
                    if (status < 0)
                            goto out1;
                } else status = 0;

		// heuristic:  "usb%d" for links we know are two-host,
		// else "eth%d" when there's reasonable doubt.  userspace
		// can rename the link if it knows better.
		if ((dev->driver_info->flags & FLAG_ETHER) != 0 &&
		    ((dev->driver_info->flags & FLAG_POINTTOPOINT) == 0 ||
		     (net->dev_addr [0] & 0x02) == 0))
			strcpy (net->name, "eth%d");
		/* WLAN devices should always be named "wlan%d" */
		if ((dev->driver_info->flags & FLAG_WLAN) != 0)
			strcpy(net->name, "wlan%d");
		/* WWAN devices should always be named "wwan%d" */
		if ((dev->driver_info->flags & FLAG_WWAN) != 0)
			strcpy(net->name, "wwan%d");

		/* maybe the remote can't receive an Ethernet MTU */
		if (net->mtu > (dev->hard_mtu - net->hard_header_len))
			net->mtu = dev->hard_mtu - net->hard_header_len;
	} else if (!info->in || !info->out)
		status = usbnet_get_endpoints (dev, udev);
	else {
		dev->in = usb_rcvbulkpipe (xdev, info->in);
		dev->out = usb_sndbulkpipe (xdev, info->out);
		if (!(info->flags & FLAG_NO_SETINT))
			status = usb_set_interface (xdev,
				interface->desc.bInterfaceNumber,
				interface->desc.bAlternateSetting);
		else
			status = 0;

	}
	if (status >= 0 && dev->status)
		status = init_status (dev, udev);
	if (status < 0)
		goto out3;
	dev->maxpacket = usb_maxpacket (dev->udev, dev->out, 1);

	if ((dev->driver_info->flags & FLAG_WLAN) != 0)
		SET_NETDEV_DEVTYPE(net, &wlan_type);
	if ((dev->driver_info->flags & FLAG_WWAN) != 0)
		SET_NETDEV_DEVTYPE(net, &wwan_type);

        dev->rxq.mem = kzalloc(dev->rxq.psize * sizeof(struct fp_data), GFP_KERNEL);
        for (i = 0; i < dev->rxq.psize; ++i) dev->rxq.pool[i] = dev->rxq.mem + i;
        dev->txq.mem = kzalloc(dev->txq.psize * sizeof(struct fp_data), GFP_KERNEL);
        for (i = 0; i < dev->txq.psize; ++i) dev->txq.pool[i] = dev->txq.mem + i;

	status = register_netdev (net);
	if (status)
		goto out3;
	netif_info(dev, probe, dev->net,
		   "register '%s' at usb-%s-%s, %s, %pM\n",
		   udev->dev.driver->name,
		   xdev->bus->bus_name, xdev->devpath,
		   dev->driver_info->description,
		   net->dev_addr);

	// ok, it's ready to go.
	netif_device_attach (net);

	if (dev->driver_info->flags & FLAG_LINK_INTR)
		usbnet_link_change(dev, 0, 0);

	return;

out3:
	if (info->unbind)
		info->unbind (dev, udev);
out1:
	usb_put_dev(xdev);
}
EXPORT_SYMBOL_GPL(usbnet_probe);

#if 0
/*-------------------------------------------------------------------------*/

/*
 * suspend the whole driver as soon as the first interface is suspended
 * resume only when the last interface is resumed
 */

int usbnet_suspend (struct usb_interface *intf, pm_message_t message)
{
	struct usbnet		*dev = usb_get_intfdata(intf);

	if (!dev->suspend_count++) {
		spin_lock_irq(&dev->txq.lock);
		/* don't autosuspend while transmitting */
		if (dev->txq.qlen && PMSG_IS_AUTO(message)) {
			spin_unlock_irq(&dev->txq.lock);
			return -EBUSY;
		} else {
			set_bit(EVENT_DEV_ASLEEP, &dev->flags);
			spin_unlock_irq(&dev->txq.lock);
		}
		/*
		 * accelerate emptying of the rx and queues, to avoid
		 * having everything error out.
		 */
		netif_device_detach (dev->net);
		usbnet_terminate_urbs(dev);
		usb_kill_urb(dev->interrupt);

		/*
		 * reattach so runtime management can use and
		 * wake the device
		 */
		netif_device_attach (dev->net);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(usbnet_suspend);

int usbnet_resume (struct usb_interface *intf)
{
	struct usbnet		*dev = usb_get_intfdata(intf);
	struct sk_buff          *skb;
	struct urb              *res;
	int                     retval;

	if (!--dev->suspend_count) {
		/* resume interrupt URBs */
		if (dev->interrupt && test_bit(EVENT_DEV_OPEN, &dev->flags))
			usb_submit_urb(dev->interrupt, GFP_NOIO);

		spin_lock_irq(&dev->txq.lock);
		while ((res = usb_get_from_anchor(&dev->deferred))) {

			skb = (struct sk_buff *)res->context;
			retval = usb_submit_urb(res, GFP_ATOMIC);
			if (retval < 0) {
				dev_kfree_skb_any(skb);
				usb_free_urb(res);
				usb_autopm_put_interface_async(dev->intf);
			} else {
				dev->net->trans_start = jiffies;
				__skb_queue_tail(&dev->txq, skb);
			}
		}

		smp_mb();
		clear_bit(EVENT_DEV_ASLEEP, &dev->flags);
		spin_unlock_irq(&dev->txq.lock);

		if (test_bit(EVENT_DEV_OPEN, &dev->flags)) {
			if (!(dev->txq.qlen >= TX_QLEN(dev)))
				netif_start_queue(dev->net);
			tasklet_schedule (&dev->bh);
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(usbnet_resume);

#endif

/*-------------------------------------------------------------------------*/

/*
 * Either a subdriver implements manage_power, then it is assumed to always
 * be ready to be suspended or it reports the readiness to be suspended
 * explicitly
 */
void usbnet_device_suggests_idle(struct usbnet *dev)
{
	if (!test_and_set_bit(EVENT_DEVICE_REPORT_IDLE, &dev->flags)) {
		dev->intf->needs_remote_wakeup = 1;
		usb_autopm_put_interface_async(dev->intf);
	}
}

/*
 * For devices that can do without special commands
 */
int usbnet_manage_power(struct usbnet *dev, int on)
{
	dev->intf->needs_remote_wakeup = on;
	return 0;
}

void usbnet_link_change(struct usbnet *dev, bool link, bool need_reset)
{
	/* update link after link is reseted */
	if (link && !need_reset)
		netif_carrier_on(dev->net);
	else
		netif_carrier_off(dev->net);

	if (need_reset && link)
		usbnet_defer_kevent(dev, EVENT_LINK_RESET);
	else
		usbnet_defer_kevent(dev, EVENT_LINK_CHANGE);
}

/*-------------------------------------------------------------------------*/
static int __usbnet_read_cmd(struct usbnet *dev, u8 cmd, u8 reqtype,
			     u16 value, u16 index, void *data, u16 size)
{
	void *buf = NULL;
	int err = -ENOMEM;

	netdev_dbg(dev->net, "usbnet_read_cmd cmd=0x%02x reqtype=%02x"
		   " value=0x%04x index=0x%04x size=%d\n",
		   cmd, reqtype, value, index, size);

	if (size) {
		buf = kmalloc(size, GFP_KERNEL);
		if (!buf)
			goto out;
	}

	err = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
			      cmd, reqtype, value, index, buf, size,
			      USB_CTRL_GET_TIMEOUT);
	if (err > 0 && err <= size) {
        if (data)
            memcpy(data, buf, err);
        else
            netdev_dbg(dev->net,
                "Huh? Data requested but thrown away.\n");
    }
	kfree(buf);
out:
	return err;
}

static int __usbnet_write_cmd(struct usbnet *dev, u8 cmd, u8 reqtype,
			      u16 value, u16 index, const void *data,
			      u16 size)
{
	void *buf = NULL;
	int err = -ENOMEM;

	netdev_dbg(dev->net, "usbnet_write_cmd cmd=0x%02x reqtype=%02x"
		   " value=0x%04x index=0x%04x size=%d\n",
		   cmd, reqtype, value, index, size);

	if (data) {
		buf = kmemdup(data, size, GFP_KERNEL);
		if (!buf)
			goto out;
	} else {
        if (size) {
            WARN_ON_ONCE(1);
            err = -EINVAL;
            goto out;
        }
    }

	err = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
			      cmd, reqtype, value, index, buf, size,
			      USB_CTRL_SET_TIMEOUT);
	kfree(buf);

out:
	return err;
}

/*
 * The function can't be called inside suspend/resume callback,
 * otherwise deadlock will be caused.
 */
int usbnet_read_cmd(struct usbnet *dev, u8 cmd, u8 reqtype,
		    u16 value, u16 index, void *data, u16 size)
{
	int ret;

	if (usb_autopm_get_interface(dev->intf) < 0)
		return -ENODEV;
	ret = __usbnet_read_cmd(dev, cmd, reqtype, value, index,
				data, size);
	usb_autopm_put_interface(dev->intf);
	return ret;
}
EXPORT_SYMBOL_GPL(usbnet_read_cmd);

/*
 * The function can't be called inside suspend/resume callback,
 * otherwise deadlock will be caused.
 */
int usbnet_write_cmd(struct usbnet *dev, u8 cmd, u8 reqtype,
		     u16 value, u16 index, const void *data, u16 size)
{
	int ret;

	if (usb_autopm_get_interface(dev->intf) < 0)
		return -ENODEV;
	ret = __usbnet_write_cmd(dev, cmd, reqtype, value, index,
				 data, size);
	usb_autopm_put_interface(dev->intf);
	return ret;
}
EXPORT_SYMBOL_GPL(usbnet_write_cmd);

/*
 * The function can be called inside suspend/resume callback safely
 * and should only be called by suspend/resume callback generally.
 */
int usbnet_read_cmd_nopm(struct usbnet *dev, u8 cmd, u8 reqtype,
			  u16 value, u16 index, void *data, u16 size)
{
	return __usbnet_read_cmd(dev, cmd, reqtype, value, index,
				 data, size);
}
EXPORT_SYMBOL_GPL(usbnet_read_cmd_nopm);

/*
 * The function can be called inside suspend/resume callback safely
 * and should only be called by suspend/resume callback generally.
 */
int usbnet_write_cmd_nopm(struct usbnet *dev, u8 cmd, u8 reqtype,
			  u16 value, u16 index, const void *data,
			  u16 size)
{
	return __usbnet_write_cmd(dev, cmd, reqtype, value, index,
				  data, size);
}
EXPORT_SYMBOL_GPL(usbnet_write_cmd_nopm);

static void usbnet_async_cmd_cb(struct urb *urb)
{
	struct usb_ctrlrequest *req = (struct usb_ctrlrequest *)urb->context;
	int status = urb->status;

	if (status < 0)
		dev_dbg(&urb->dev->dev, "%s failed with %d",
			__func__, status);

	kfree(req);
	usb_free_urb(urb);
}

/*
 * The caller must make sure that device can't be put into suspend
 * state until the control URB completes.
 */
int usbnet_write_cmd_async(struct usbnet *dev, u8 cmd, u8 reqtype,
			   u16 value, u16 index, const void *data, u16 size)
{
	struct usb_ctrlrequest *req = NULL;
	struct urb *urb;
	int err = -ENOMEM;
	void *buf = NULL;

	netdev_dbg(dev->net, "usbnet_write_cmd cmd=0x%02x reqtype=%02x"
		   " value=0x%04x index=0x%04x size=%d\n",
		   cmd, reqtype, value, index, size);

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto fail;

	if (data) {
		buf = kmemdup(data, size, GFP_ATOMIC);
		if (!buf) {
			netdev_err(dev->net, "Error allocating buffer"
				   " in %s!\n", __func__);
			goto fail_free;
		}
	}

	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC);
	if (!req)
		goto fail_free_buf;

	req->bRequestType = reqtype;
	req->bRequest = cmd;
	req->wValue = cpu_to_le16(value);
	req->wIndex = cpu_to_le16(index);
	req->wLength = cpu_to_le16(size);

	usb_fill_control_urb(urb, dev->udev,
			     usb_sndctrlpipe(dev->udev, 0),
			     (void *)req, buf, size,
			     usbnet_async_cmd_cb, req);
	urb->transfer_flags |= URB_FREE_BUFFER;

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		netdev_err(dev->net, "Error submitting the control"
			   " message: status=%d\n", err);
		goto fail_free;
	}
	return 0;

fail_free_buf:
	kfree(buf);
fail_free:
	kfree(req);
	usb_free_urb(urb);
fail:
	return err;

}
EXPORT_SYMBOL_GPL(usbnet_write_cmd_async);
/*-------------------------------------------------------------------------*/

extern struct usb_driver cdc_driver;
extern struct usb_driver rndis_driver;
extern struct usb_driver mcs7830_driver;
extern struct usb_driver asix_driver;

static int __init usbnet_init(void)
{
        FP_QUEUE_SIZE = FP_QUEUE_MAX;

#if defined(ARM)
        // should not exceed ep ring size
	if (of_machine_is_compatible("qcom,ipq8074")) FP_QUEUE_SIZE = 30;
#endif
        BUG_ON(FP_QUEUE_SIZE > FP_QUEUE_MAX);

	eth_random_addr(node_id);
        if (register_netdevice_notifier(&dev_notify)) return -1;

        usb_register(&cdc_driver);
        usb_register(&rndis_driver);
        usb_register(&mcs7830_driver);
        usb_register(&asix_driver);

        return 0;
}
module_init(usbnet_init);

static void __exit usbnet_exit(void)
{
        unregister_netdevice_notifier(&dev_notify);

        usb_deregister(&cdc_driver);
        usb_deregister(&rndis_driver);
        usb_deregister(&mcs7830_driver);
        usb_deregister(&asix_driver);
}
module_exit(usbnet_exit);

MODULE_AUTHOR("David Brownell");
MODULE_DESCRIPTION("USB network driver framework");
MODULE_LICENSE("GPL");
