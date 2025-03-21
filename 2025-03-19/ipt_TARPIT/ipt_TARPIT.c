/*
 * Kernel module to capture and hold incoming TCP connections using
 * no local per-connection resources.
 *
 * Based on ipt_REJECT.c and offering functionality similar to
 * LaBrea <http://www.hackbusters.net/LaBrea/>.
 *
 * Copyright (c) 2002 Aaron Hopkins <tools@die.net>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Goal:
 * - Allow incoming TCP connections to be established.
 * - Passing data should result in the connection being switched to the
 *   persist state (0 byte window), in which the remote side stops sending
 *   data and asks to continue every 60 seconds.
 * - Attempts to shut down the connection should be ignored completely, so
 *   the remote side ends up having to time it out.
 *
 * This means:
 * - Reply to TCP SYN,!ACK,!RST,!FIN with SYN-ACK, window 5 bytes
 * - Reply to TCP SYN,ACK,!RST,!FIN with RST to prevent spoofing
 * - Reply to TCP !SYN,!RST,!FIN with ACK, window 0 bytes, rate-limited
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <net/icmp.h>
#include <net/arp.h>
#include <net/secure_seq.h>
struct in_device;
#include <net/route.h>
#include <linux/random.h>
#include <linux/netfilter_ipv4/ip_tables.h>

#if 0
#define DEBUGP printk
#else
#define DEBUGP(format, args...)
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aaron Hopkins <tools@die.net>");

/* Stolen from ip_finish_output2 */
static int ip_direct_send(struct sk_buff *skb)
{
	struct dst_entry *dst = skb_dst(skb);
	struct rtable *rt = (struct rtable *) dst;
	struct net_device *dev = dst->dev;
	struct neighbour *neigh;
	u32 nexthop;

	rcu_read_lock_bh();
	nexthop = (__force u32) rt_nexthop(rt, ip_hdr(skb)->daddr);
	neigh = __ipv4_neigh_lookup_noref(dev, nexthop);
	if (unlikely(!neigh))
		neigh = __neigh_create(&arp_tbl, &nexthop, dev, false);
	if (!IS_ERR(neigh)) {
		int res;

		sock_confirm_neigh(skb, neigh);
		res = neigh_output(neigh, skb, false);

		rcu_read_unlock_bh();
		return res;
	}
	rcu_read_unlock_bh();

	if (net_ratelimit())
		printk(KERN_DEBUG "TARPIT ip_direct_send: no header cache and no neighbor!\n");
	kfree_skb(skb);
	return -EINVAL;
}


/* Send reply */
static void tarpit_tcp(struct sk_buff *oskb,struct rtable *ort,int local)
{
	struct sk_buff *nskb;
	struct rtable *nrt;
	struct tcphdr *otcph, *ntcph;
	struct flowi4 fl = {};
	unsigned int otcplen;
	unsigned tmp;
	struct inet_peer *peer;

	/* A truncated TCP header isn't going to be useful */
	if (oskb->len < (ip_hdr(oskb)->ihl*4) + sizeof(struct tcphdr))
		return;

	otcph = (struct tcphdr *)((u_int32_t*) ip_hdr(oskb)
				  + ip_hdr(oskb)->ihl);
	otcplen = oskb->len - ip_hdr(oskb)->ihl*4;

	/* No replies for RST or FIN */
	if (otcph->rst || otcph->fin)
		return;

	/* No reply to !SYN,!ACK.  Rate-limit replies to !SYN,ACKs */
	peer = inet_getpeer_v4(init_net.ipv4.peers, ip_hdr(oskb)->daddr,
			       l3mdev_master_ifindex(oskb->dev), 1);
	if (!otcph->syn && (!otcph->ack || !inet_peer_xrlim_allow(peer, 1*HZ))) {
		if (peer)
			inet_putpeer(peer);
		return;
	}
	if (peer)
		inet_putpeer(peer);

	/* Check checksum. */
	if (tcp_v4_check(otcplen, ip_hdr(oskb)->saddr,
			 ip_hdr(oskb)->daddr,
			 csum_partial((char *)otcph, otcplen, 0)) != 0)
		return;

	/* Copy skb (even if skb is about to be dropped, we can't just
           clone it because there may be other things, such as tcpdump,
           interested in it) */
	nskb = skb_copy(oskb, GFP_ATOMIC);
	if (!nskb)
		return;

	/* This packet will not be the same as the other: clear nf fields */
	nf_conntrack_put(skb_nfct(nskb));
	nskb->_nfct = 0;

	ntcph = (struct tcphdr *)((u_int32_t*) ip_hdr(nskb) + ip_hdr(nskb)->ihl);

	/* Truncate to length (no data) */
	ntcph->doff = sizeof(struct tcphdr)/4;
	skb_trim(nskb, ip_hdr(nskb)->ihl*4 + sizeof(struct tcphdr));
	ip_hdr(nskb)->tot_len = htons(nskb->len);

	/* Swap source and dest */
	
	tmp = ip_hdr(nskb)->daddr;
	ip_hdr(nskb)->daddr = ip_hdr(nskb)->saddr;
	ip_hdr(nskb)->saddr = tmp;
	tmp = ntcph->source;
	ntcph->source = ntcph->dest;
	ntcph->dest = tmp;

	/* Use supplied sequence number or make a new one */
	ntcph->seq = otcph->ack ? otcph->ack_seq
		: htonl(secure_tcp_seq(ip_hdr(nskb)->saddr, ip_hdr(nskb)->daddr,
				       ntcph->source, ntcph->dest));

	/* Our SYN-ACKs must have a >0 window */
	ntcph->window = (otcph->syn && !otcph->ack) ? htons(5) : 0;

	ntcph->urg_ptr = 0;

	/* Reset flags */
	((u_int8_t *)ntcph)[13] = 0;

	if (otcph->syn && otcph->ack) {
		ntcph->rst = 1;
		ntcph->ack_seq = 0;
	} else {
		ntcph->syn = otcph->syn;
		ntcph->ack = 1;
		ntcph->ack_seq = htonl(ntohl(otcph->seq) + otcph->syn);
	}

	/* Adjust TCP checksum */
	ntcph->check = 0;
	ntcph->check = tcp_v4_check(sizeof(struct tcphdr),
				   ip_hdr(nskb)->saddr,
				   ip_hdr(nskb)->daddr,
				   csum_partial((char *)ntcph,
						sizeof(struct tcphdr), 0));

	/* Adjust IP TTL */
#ifdef CONFIG_SYSCTL
	ip_hdr(nskb)->ttl = init_net.ipv4.sysctl_ip_default_ttl;
#else
	ip_hdr(nskb)->ttl = IPDEFTTL;
#endif

	/* Set DF, id = 0 */
	ip_hdr(nskb)->frag_off = htons(IP_DF);
	ip_hdr(nskb)->id = 0;

	/* Adjust IP checksum */
	ip_hdr(nskb)->check = 0;
	ip_hdr(nskb)->check = ip_fast_csum((unsigned char *)ip_hdr(nskb),
					   ip_hdr(nskb)->ihl);

	fl.daddr = ip_hdr(nskb)->daddr;
	fl.saddr = local ? ip_hdr(nskb)->saddr : 0;
	fl.flowi4_tos = RT_TOS(ip_hdr(nskb)->tos);
	fl.flowi4_oif = 0;

	nrt = ip_route_output_key(&init_net, &fl);
	if (IS_ERR(nrt))
		goto free_nskb;

	skb_dst_drop(nskb);
	skb_dst_set(nskb, &nrt->dst);

	/* "Never happens" */
	if (nskb->len > dst_mtu(skb_dst(nskb)))
		goto free_nskb;

	ip_direct_send (nskb);

	return;

 free_nskb:
	kfree_skb(nskb);
}


static unsigned int tarpit(struct sk_buff *skb,
			   const struct xt_action_param *par)
{
	struct rtable *rt = skb_rtable(skb);

	/* Do we have an input route cache entry? */
	if (!rt)
		return NF_DROP;

	/* No replies to physical multicast/broadcast */
	if (skb->pkt_type != PACKET_HOST && skb->pkt_type != PACKET_OTHERHOST)
		return NF_DROP;

	/* Now check at the protocol level */
	if (rt->rt_flags&(RTCF_BROADCAST|RTCF_MULTICAST))
		return NF_DROP;

	/* Our naive response construction doesn't deal with IP
           options, and probably shouldn't try. */
	if (ip_hdr(skb)->ihl*4 != sizeof(struct iphdr))
		return NF_DROP;

	/* We aren't interested in fragments */
	if (ip_hdr(skb)->frag_off & htons(IP_OFFSET))
		return NF_DROP;

	tarpit_tcp(skb, rt, xt_hooknum(par) == NF_INET_LOCAL_IN);

	return NF_DROP;
}


static int check(const struct xt_tgchk_param *par)
{
	const struct ipt_entry *e = (const struct ipt_entry *) par->entryinfo;

	/* Only allow these for input/forward packet filtering. */
	if (strcmp(par->table, "filter") != 0) {
		DEBUGP("TARPIT: bad table %s'.\n", tablename);
		return -EINVAL;
	}
	if ((par->hook_mask & ~((1 << NF_INET_LOCAL_IN)
				| (1 << NF_INET_FORWARD))) != 0) {
		DEBUGP("TARPIT: bad hook mask %X\n", par->hook_mask);
		return -EINVAL;
	}

	/* Must specify that it's a TCP packet */
	if (e->ip.proto != IPPROTO_TCP || (e->ip.invflags & IPT_INV_PROTO)) {
		DEBUGP("TARPIT: not valid for non-tcp\n");
		return -EINVAL;
	}

	return 0;
}

static struct xt_target ipt_tarpit_reg = {
	.name = "TARPIT",
	.family = AF_INET,
	.target = tarpit,
	.checkentry = check,
	.me = THIS_MODULE
};

static int __init init(void)
{
	return xt_register_target(&ipt_tarpit_reg);
}

static void __exit fini(void)
{
	xt_unregister_target(&ipt_tarpit_reg);
}

module_init(init);
module_exit(fini);
