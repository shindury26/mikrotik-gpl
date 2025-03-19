// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *	Forwarding decision
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/netpoll.h>
#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/netfilter_bridge.h>
#include <linux/switch.h>
#include "br_private.h"
#include "../mesh/mesh.h"

DECLARE_PER_CPU(unsigned, per_cpu_xmit_switched);

static inline bool is_hw_pkt(const struct sk_buff *skb,
			     const struct net_bridge_port *p)
{
	return !br_multicast_igmp_type(skb) &&
		!BR_INPUT_SKB_CB(skb)->dhcp_req &&
		!BR_INPUT_SKB_CB(skb)->dhcp_rep &&
		!(BR_INPUT_SKB_CB(skb)->ll_mc && hlist_unhashed(&p->rlist));
}

static inline int check_hw(const struct sk_buff *skb,
			   const struct net_bridge_port *p,
			   unsigned from_switch_group,
			   unsigned *switches)
{
	if (p->switch_group == BR_NOSWITCH)
		return true;
	if (!is_hw_pkt(skb, p))
		return true;
	if (!switches && p->switch_group != from_switch_group)
		return true;
	if (switches && !(*switches & BIT(p->switch_group)) &&
	    p->switch_group != from_switch_group) {
		*switches |= BIT(p->switch_group);
		return true;
	}
	return false;
}

/* Don't forward packets to originating port or forwarding disabled */
static inline int should_deliver(const struct net_bridge_port *p,
				 const struct sk_buff *skb, unsigned horizon)
{
	struct net_bridge_vlan_group *vg;

	vg = nbp_vlan_group_rcu(p);
	return ((p->flags & BR_HAIRPIN_MODE) || skb->dev != p->dev) &&
		p->state == BR_STATE_FORWARDING && br_allowed_egress(vg, skb) &&
		nbp_switchdev_allowed_egress(p, skb) &&
		!br_skb_isolated(p, skb) && (horizon == BR_NOHORIZON || horizon != p->horizon)
	        && (BR_INPUT_SKB_CB(skb)->src_port_peer_link
	            ? !(p->flags & BR_MLAG_DUAL_LINK) : true);
}

static inline unsigned packet_length(const struct sk_buff *skb)
{
	if (skb->protocol == htons(MESH_ENCAP_PROTO)) return 0;

	return skb->len - (ANY_VLAN_PROTO_N(skb->protocol) ? VLAN_HLEN : 0);
}

static inline void set_priority(struct sk_buff *skb)
{
	if (ANY_VLAN_PROTO_N(skb->protocol)) {
		struct vlan_ethhdr *v = (struct vlan_ethhdr *)(skb->data - ETH_HLEN);
		unsigned tci = ntohs(v->h_vlan_TCI);

		tci = (tci & ~(7 << 13)) | ((skb->priority & 0x7) << 13);
		v->h_vlan_TCI = htons(tci);
	}
}

int br_dev_queue_push_xmit(struct net *net, struct sock *sk, struct sk_buff *skb)
{
	struct net_bridge_port *port = br_port_get_rcu(skb->dev);

#if 0 // is_skb_forwardable is handled below - IFF_UP check and our l2mtu, mtu checks
	if (!is_skb_forwardable(skb->dev, skb))
		goto drop;
#endif
	if (!(skb->dev->flags & IFF_UP))
	    goto drop;

	/* drop mtu oversized packets except gso */
	if (skb->dev->l2mtu) {
		unsigned len = skb->len + (skb_vlan_tag_present(skb) ? VLAN_HLEN : 0);
		if (skb->dev->l2mtu < len && !skb_is_gso(skb))
			goto drop;
	}
	else if (packet_length(skb) > skb->dev->mtu && !skb_is_gso(skb)) {
		goto drop;
	}

	set_priority(skb);
	skb_push(skb, ETH_HLEN);
	br_drop_fake_rtable(skb);

	if (skb->ip_summed == CHECKSUM_PARTIAL &&
	    (skb->protocol == htons(ETH_P_8021Q) ||
	     skb->protocol == htons(ETH_P_8021AD))) {
		int depth;

		if (!__vlan_get_protocol(skb, skb->protocol, &depth))
			goto drop;

		skb_set_network_header(skb, depth);
	}

	if (port && port->switch_group != BR_NOSWITCH && is_hw_pkt(skb, port)) {
		skb = br_handle_vlan_switch(port->br, skb);
		if (!skb)
			return 0;

		// This should get reset in the switch port xmit
		*raw_cpu_ptr(&per_cpu_xmit_switched) = 1;
	}

	if (dev_queue_xmit(skb)) {
		*raw_cpu_ptr(&per_cpu_xmit_switched) = 0;
	}

	return 0;
drop:
	kfree_skb(skb);
	return 0;
}
EXPORT_SYMBOL_GPL(br_dev_queue_push_xmit);

int br_forward_finish(struct net *net, struct sock *sk, struct sk_buff *skb)
{
	skb->tstamp = 0;
	return NF_HOOK(NFPROTO_BRIDGE, NF_BR_POST_ROUTING,
		       net, sk, skb, NULL, skb->dev,
		       br_dev_queue_push_xmit);

}
EXPORT_SYMBOL_GPL(br_forward_finish);

static void __br_forward(const struct net_bridge_port *to,
			 struct sk_buff *skb, bool local_orig)
{
	struct net_bridge_vlan_group *vg;
	struct net_device *indev;
	struct net *net;
	int br_hook;

	if (BR_INPUT_SKB_CB(skb)->dhcp_req && !(to->flags & BR_TRUSTED_PORT)) {
		kfree_skb(skb);
		return;
	}

	if (!is_hw_pkt(skb, to) || to->switch_group == BR_NOSWITCH) {
		vg = nbp_vlan_group_rcu(to);
		skb = br_handle_vlan(to->br, to, vg, skb);
		if (!skb)
			return;
	}

	if (to->br->add_info_option && !(to->flags & BR_TRUSTED_PORT) &&
	    BR_INPUT_SKB_CB(skb)->dhcp_rep) {
		skb = br_dhcp_remove_agent_info(skb);
		if (!skb)
			return;
	}

	indev = skb->dev;
	skb->dev = to->dev;
	if (!local_orig) {
		if (skb_warn_if_lro(skb)) {
			kfree_skb(skb);
			return;
		}
		br_hook = NF_BR_FORWARD;
		skb_forward_csum(skb);
		net = dev_net(indev);
	} else {
#if 0
		if (unlikely(netpoll_tx_running(to->br->dev))) {
			if (!is_skb_forwardable(skb->dev, skb)) {
				kfree_skb(skb);
			} else {
				skb_push(skb, ETH_HLEN);
				br_netpoll_send_skb(to, skb);
			}
			return;
		}
#endif
		br_hook = NF_BR_LOCAL_OUT;
		net = dev_net(skb->dev);
		indev = NULL;
	}

	NF_HOOK(NFPROTO_BRIDGE, br_hook,
		net, NULL, skb, indev, skb->dev,
		br_forward_finish);
}

static int deliver_clone(const struct net_bridge_port *prev,
			 struct sk_buff *skb, bool local_orig)
{
	struct net_device *dev = BR_INPUT_SKB_CB(skb)->brdev;

	skb = skb_clone(skb, GFP_ATOMIC);
	if (!skb) {
		dev->stats.tx_dropped++;
		return -ENOMEM;
	}

	__br_forward(prev, skb, local_orig);
	return 0;
}

/**
 * br_forward - forward a packet to a specific port
 * @to: destination port
 * @skb: packet being forwarded
 * @local_rcv: packet will be received locally after forwarding
 * @local_orig: packet is locally originated
 *
 * Should be called with rcu_read_lock.
 */
void br_forward(const struct net_bridge_port *to,
		struct sk_buff *skb, bool local_rcv, bool local_orig,
		unsigned horizon, unsigned switch_group)
{
	if (unlikely(!to))
		goto out;

	/* redirect to backup link if the destination port is down */
	if (rcu_access_pointer(to->backup_port) && !netif_carrier_ok(to->dev)) {
		struct net_bridge_port *backup_port;

		backup_port = rcu_dereference(to->backup_port);
		if (unlikely(!backup_port))
			goto out;
		to = backup_port;
	}

	if (should_deliver(to, skb, horizon) &&
	    check_hw(skb, to, switch_group, NULL)) {
		if (local_rcv)
			deliver_clone(to, skb, local_orig);
		else
			__br_forward(to, skb, local_orig);
		return;
	}

out:
	if (!local_rcv)
		kfree_skb(skb);
}
EXPORT_SYMBOL(br_forward);

static struct net_bridge_port *maybe_deliver(
	struct net_bridge_port *prev, struct net_bridge_port *p,
	struct sk_buff *skb, bool local_orig,
	unsigned horizon, unsigned from_switch_group, unsigned *switches)
{
	u8 igmp_type = br_multicast_igmp_type(skb);
	int err;

	if (!should_deliver(p, skb, horizon))
		return prev;

	if (BR_INPUT_SKB_CB(skb)->dhcp_req && !(p->flags & BR_TRUSTED_PORT))
		return prev;

	if (!check_hw(skb, p, from_switch_group, switches))
		return prev;

	if (!prev)
		goto out;

	err = deliver_clone(prev, skb, local_orig);
	if (err)
		return ERR_PTR(err);
out:
	br_multicast_count(p->br, p, skb, igmp_type, BR_MCAST_DIR_TX);

	return p;
}

/* called under rcu_read_lock */
void br_flood(struct net_bridge *br, struct sk_buff *skb,
	      enum br_pkt_type pkt_type, bool local_rcv, bool local_orig,
	      unsigned horizon, unsigned switch_group)
{
	struct net_bridge_port *prev = NULL;
	struct net_bridge_port *p;
	unsigned switches = 0;

	list_for_each_entry_rcu(p, &br->port_list, list) {
		/* Do not flood unicast traffic to ports that turn it off, nor
		 * other traffic if flood off, except for traffic we originate
		 */
		switch (pkt_type) {
		case BR_PKT_UNICAST:
			if (!(p->flags & BR_FLOOD))
				continue;
			break;
		case BR_PKT_MULTICAST:
			if (!(p->flags & BR_MCAST_FLOOD) && skb->dev != br->dev)
				continue;
			break;
		case BR_PKT_BROADCAST:
			if (!(p->flags & BR_BCAST_FLOOD) && skb->dev != br->dev)
				continue;
			break;
		}

//		/* Do not flood to ports that enable proxy ARP */
//		if (p->flags & BR_PROXYARP)
//			continue;
//		if ((p->flags & (BR_PROXYARP_WIFI | BR_NEIGH_SUPPRESS)) &&
//		    BR_INPUT_SKB_CB(skb)->proxyarp_replied)
//			continue;

		prev = maybe_deliver(prev, p, skb, local_orig, horizon, switch_group, &switches);
		if (IS_ERR(prev))
			goto out;
	}

	if (!prev)
		goto out;

	if (local_rcv)
		deliver_clone(prev, skb, local_orig);
	else
		__br_forward(prev, skb, local_orig);
	return;

out:
	if (!local_rcv)
		kfree_skb(skb);
}

#ifdef CONFIG_BRIDGE_IGMP_SNOOPING
static void maybe_deliver_addr(struct net_bridge_port *p, struct sk_buff *skb,
			       const unsigned char *addr, bool local_orig,
			       unsigned horizon, unsigned from_switch_group,
			       unsigned *switches)
{
	struct net_device *dev = BR_INPUT_SKB_CB(skb)->brdev;
	const unsigned char *src = eth_hdr(skb)->h_source;

	if (!should_deliver(p, skb, horizon))
		return;

	if (!check_hw(skb, p, from_switch_group, switches))
		return;

	/* Even with hairpin, no soliloquies - prevent breaking IPv6 DAD */
	if (skb->dev == p->dev && ether_addr_equal(src, addr))
		return;

	skb = skb_copy(skb, GFP_ATOMIC);
	if (!skb) {
		dev->stats.tx_dropped++;
		return;
	}

	if (!is_broadcast_ether_addr(addr))
		memcpy(eth_hdr(skb)->h_dest, addr, ETH_ALEN);

	__br_forward(p, skb, local_orig);
}

/* called with rcu_read_lock */
void br_multicast_flood(struct net_bridge_mdb_entry *mdst,
			struct sk_buff *skb,
			bool local_rcv, bool local_orig,
			unsigned horizon, unsigned switch_group)
{
	struct net_device *dev = BR_INPUT_SKB_CB(skb)->brdev;
	struct net_bridge *br = netdev_priv(dev);
	struct net_bridge_port *prev = NULL;
	struct net_bridge_port_group *p;
	struct hlist_node *rp;
	unsigned switches = 0;

	rp = rcu_dereference(hlist_first_rcu(&br->router_list));
	p = mdst ? rcu_dereference(mdst->ports) : NULL;
	while (p || rp) {
		struct net_bridge_port *port, *lport, *rport;

		lport = p ? p->port : NULL;
		rport = hlist_entry_safe(rp, struct net_bridge_port, rlist);

		if ((unsigned long)lport > (unsigned long)rport) {
			port = lport;

			if (port->flags & BR_MULTICAST_TO_UNICAST) {
				maybe_deliver_addr(lport, skb, p->eth_addr,
						   local_orig,
						   horizon, switch_group, &switches);
				goto delivered;
			}
		} else {
			port = rport;
		}

		prev = maybe_deliver(prev, port, skb, local_orig,
				     horizon, switch_group, &switches);
		if (IS_ERR(prev))
			goto out;
delivered:
		if ((unsigned long)lport >= (unsigned long)port)
			p = rcu_dereference(p->next);
		if ((unsigned long)rport >= (unsigned long)port)
			rp = rcu_dereference(hlist_next_rcu(rp));
	}

	if (!prev)
		goto out;

	if (local_rcv)
		deliver_clone(prev, skb, local_orig);
	else
		__br_forward(prev, skb, local_orig);
	return;

out:
	if (!local_rcv)
		kfree_skb(skb);
}
#endif
