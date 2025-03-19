#include "chan.h"
#include "dev.h"
#include "l2tp_ioctl.h"
#include "l2tp.h"

#include <linux/interrupt.h>
#include <linux/jhash.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/jiffies.h>
#include <linux/netdevice.h>
#include <linux/netfilter_ipv4.h>
#include <linux/rwlock.h>
#include <linux/spinlock.h>
#include <linux/udp.h>
#include <net/ip6_route.h>
#include <net/ip6_checksum.h>
#include <net/route.h>
#include <net/xfrm.h>
#include <net/arp.h>
#include <linux/xfrm.h>
#include <uapi/linux/ppp-ioctl.h>

static DEFINE_RWLOCK(chan_lock);

struct hlist_head ses_table[L2TP_HASHSIZE];

extern struct net_device *ppp_get_device(struct ppp_channel *);
extern void l2tp_chan_attached(struct ppp_channel *);

inline int l2tp_ch_isip4(const struct l2tp_channel* ch) {
    return AF_INET6 != ch->info.address_family;
}

static inline int l2tp_ch_ipproto(const struct l2tp_channel* ch) {
    return L2TPv3_IP == ch->info.l2tp_ver ? L2TPV3_PROTO : IPPROTO_UDP;
}

static inline void print_hw(const char *pfx, unsigned char *p) {
    printk("%s %02x:%02x:%02x:%02x:%02x:%02x\n", pfx,
                p[0], p[1], p[2], p[3], p[4], p[5]);
}

static inline void print_cookie(const char * const pfx,
                                const unsigned char *cookie,
                                const unsigned sz) {
    if (!sz) return;
    if (sz > 8) {
        printk("print_cookie: invalid cookie size %u\n", sz);
        return;
    }

    char buf[64];
    memset(buf, 0, sizeof(buf));

    const unsigned char *p = cookie;
    char *pbuf = buf;
    do {
        snprintf(pbuf, 4, "%02X ", *p);
        pbuf += 3;
    } while(++p < (cookie + sz));
    printk("%s %s\n", pfx, buf);
}

static void free_rt_rcu(struct rcu_head *rcu_head) {
    struct rt_cache *rt = container_of(rcu_head, struct rt_cache, rcu_head);
    if (rt->rt) ip_rt_put(rt->rt);
    if (rt->dst) dst_release(rt->dst); // ipv6
    kfree(rt);
}

static void free_rt(struct rt_cache *rt) {
    if (rt) {
	call_rcu(&rt->rcu_head, free_rt_rcu);
    }
}

struct rt_cache *lookup_rt(struct l2tp_channel *ch, struct net_device *dev) {
    struct rt_cache *rt_cache = rcu_dereference(ch->rt_cache);
    struct rt_cache *old;
    struct rtable *rt;
    struct neighbour *neigh;
    struct flowi4 fl;
    int ip_mode;
    struct socket *current_sok;

    while (rt_cache) {
	struct dst_entry *dst = &rt_cache->rt->dst;
	if (time_before_eq(jiffies, rt_cache->last_lookup + HZ) &&
	        (!dst->obsolete || dst->ops->check(dst, 0))) {
	    return rt_cache;
	}
	old = xchg(&ch->rt_cache, NULL);
	if (old == rt_cache) {
	    free_rt(rt_cache);
	    break;
	}
	rt_cache = old;
    }

    ip_mode = ch->info.l2tp_ver == L2TPv3_IP;
    current_sok = ip_mode ? l2tp_ip4_socket : l2tp_udp4_socket;
    memset(&fl, 0, sizeof(fl));
    fl.daddr = ch->info.remote_addr;
    fl.saddr = ch->info.src_addr;
    fl.flowi4_proto = l2tp_ch_ipproto(ch);
    if (!ip_mode) {
        fl.fl4_sport = inet_sk(current_sok->sk)->inet_sport;
        fl.fl4_dport = ch->info.remote_port;
    }

    security_sk_classify_flow(current_sok->sk, flowi4_to_flowi(&fl));

    rt = ip_route_output_flow(&init_net, &fl, current_sok->sk);
    if (IS_ERR(rt))
	return NULL;

    if (rt->dst.dev == dev) {
        printk("l2tp: routed back to self\n");
        ip_rt_put(rt);
	return NULL;
    }

    rt_cache = kzalloc(sizeof(struct rt_cache), GFP_ATOMIC);
    rt_cache->rt = rt;
    rt_cache->saddr = fl.saddr;
    rt_cache->daddr = fl.daddr;
    rt_cache->last_lookup = jiffies;
    rt_cache->fast_path = false;

    if (unlikely(rt->rt_flags & RTCF_DOREDIRECT))
	goto slow_path;
    if (unlikely(rt->dst.xfrm))
	goto slow_path;

    rcu_read_lock_bh();
    neigh = __ipv4_neigh_lookup_noref(rt->dst.dev, rt_cache->daddr);

    if (!neigh) goto slow_path_rcu;

    if (unlikely(!(neigh->nud_state & (NUD_CONNECTED | NUD_DELAY | NUD_PROBE))))
	goto slow_path_rcu;

    neigh->used = jiffies;

    rt_cache->fast_path = true;
    memcpy(rt_cache->neigh_addr, neigh->ha, 6);

  slow_path_rcu:
    rcu_read_unlock_bh();

  slow_path:
    old = xchg(&ch->rt_cache, rt_cache);
    free_rt(old);

    return rt_cache;
}

struct rt_cache *lookup_rt6(struct l2tp_channel *ch,
                            const struct net_device *dev) {
    struct sock *sk;
    struct inet_sock *inet;
    struct rt_cache *old;
    struct socket *current_sok;
    struct dst_entry *dst;
    int ip_mode;
    struct rt_cache *rt_cache = rcu_dereference(ch->rt_cache);

    while (rt_cache) {
        struct dst_entry *dst = rt_cache->dst;
        if (dst && time_before_eq(jiffies, rt_cache->last_lookup + HZ) &&
                (!dst->obsolete || dst->ops->check(dst, 0))) {
            return rt_cache;
        }
        old = xchg(&ch->rt_cache, NULL);
        if (old == rt_cache) {
            free_rt(rt_cache);
            break;
        }
        rt_cache = old;
    }

    ip_mode = L2TPv3_IP == ch->info.l2tp_ver;
    current_sok = ip_mode ? l2tp_ip6_socket : l2tp_udp6_socket;
    sk = current_sok->sk;
    inet = inet_sk(current_sok->sk);

    struct flowi6 *fl6 = &ch->fl6;
    if (!ip_mode) fl6->fl6_sport = inet->inet_sport;
    security_sk_classify_flow(sk, flowi6_to_flowi(fl6));

    dst = ip6_dst_lookup_flow(sock_net(sk), sk, fl6, NULL);
    if (IS_ERR(dst)) {
        printk("lookup_rt6: xfrm_lookup error\n");
        return NULL;
    }

    if (dst->dev == dev) {
        printk("lookup_rt6: routed back to self (ipv6)\n");
        dst_release(dst);
        return NULL;
    }

    rt_cache = kzalloc(sizeof(struct rt_cache), GFP_ATOMIC);
    rt_cache->dst = dst;
    rt_cache->fl6 = *fl6;
    rt_cache->last_lookup = jiffies;
    rt_cache->fast_path = false;

    old = xchg(&ch->rt_cache, rt_cache);
    free_rt(old);

    return ch->rt_cache;
}

void l2tp_xmit_ip_header_prep(struct sk_buff *skb,
                               struct l2tp_channel *ch, void *iphdr) {
    uint32_t *hdr;

    if (l2tp_ch_isip4(ch)) {
        struct iphdr *iph = iphdr;
        hdr = (uint32_t *) (iph + 1);
    }
    else {
        struct ipv6hdr *iph = iphdr;
        hdr = (uint32_t *) (iph + 1);
    }

    put_unaligned(ch->info.remote_sesid, hdr);
    hdr++;
    if (ch->info.rcookie_sz) {
        memcpy(hdr, ch->info.remote_cookie, ch->info.rcookie_sz);
        hdr += (ch->info.rcookie_sz / sizeof(hdr[0]));
    }

    put_unaligned(__constant_htons(0xff03), (unsigned short *)hdr);
}

inline struct sock *udp_sock(const struct l2tp_channel *ch) {
    return l2tp_ch_isip4(ch) ? l2tp_udp6_socket->sk : l2tp_udp4_socket->sk;
}

void l2tp_xmit_udp_header_prep(struct sk_buff *skb,
                              struct l2tp_channel *ch, void *iphdr) {
    unsigned udp_len;
    struct udphdr *udph;
    unsigned short *hdr;

    udp_len = skb->len - (l2tp_ch_isip4(ch) ? sizeof(struct iphdr) : sizeof(struct ipv6hdr));
    udph = udp_hdr(skb);
    udph->source = inet_sk(udp_sock(ch))->inet_sport;
    udph->dest = ch->info.remote_port;
    udph->len = htons(udp_len);
    udph->check = 0;

    hdr = (unsigned short *) (udph + 1);
    put_unaligned(__constant_htons(L2TP_VERSION2), hdr);
    put_unaligned(ch->info.remote_tunid, &hdr[1]);
    put_unaligned(ch->info.remote_sesid, &hdr[2]);
    put_unaligned(ntohs(0xff03), &hdr[3]);

    if (l2tp_ch_isip4(ch)) {
        struct iphdr *iph = iphdr;
        udph->check = csum_tcpudp_magic(iph->saddr, iph->daddr, udp_len,
                                        IPPROTO_UDP, csum_partial(udph, udp_len, 0));
    }
    else { // ipv6
        struct ipv6hdr *iph = iphdr;
        udph->check = csum_ipv6_magic(&iph->saddr, &iph->daddr, udp_len,
                                      IPPROTO_UDP, csum_partial(udph, udp_len, 0));
    }
}

void l2tp_xmit_udp3_header_prep(struct sk_buff *skb, struct l2tp_channel *ch,
                                void *iphdr) {
    unsigned udp_len;
    struct udphdr *udph;
    unsigned short *hdr;

    udp_len = skb->len - (l2tp_ch_isip4(ch) ? sizeof(struct iphdr) : sizeof(struct ipv6hdr));
    udph = udp_hdr(skb);
    udph->source = inet_sk(udp_sock(ch))->inet_sport;
    udph->dest = ch->info.remote_port;
    udph->len = htons(udp_len);
    udph->check = 0;

    hdr = (unsigned short *) (udph + 1);
    put_unaligned(__constant_htons(L2TP_VERSION3), hdr);
    hdr++;
    put_unaligned(0, hdr); // *reserved* v3 udp header field
    hdr++;
    memcpy(hdr, &ch->info.remote_sesid, sizeof(ch->info.remote_sesid));
    hdr += 2;
    if (ch->info.rcookie_sz) {
        memcpy(hdr, ch->info.remote_cookie, ch->info.rcookie_sz);
        hdr += (ch->info.rcookie_sz / sizeof(hdr[0]));
    }
    put_unaligned(ntohs(0xff03), hdr);

    if (l2tp_ch_isip4(ch)) {
        struct iphdr *iph = iphdr;
        udph->check = csum_tcpudp_magic(iph->saddr, iph->daddr, udp_len,
                                        IPPROTO_UDP, csum_partial(udph, udp_len, 0));
    }
    else {
        struct ipv6hdr *iph = iphdr;
        udph->check = csum_ipv6_magic(&iph->saddr, &iph->daddr, udp_len,
                                      IPPROTO_UDP, csum_partial(udph, udp_len, 0));
    }
}

int l2tp_xmit(struct ppp_channel *chan, struct sk_buff *skb) {
    struct l2tp_channel *ch = chan->private;
    struct rt_cache *rt;
    struct iphdr *iph;
    unsigned head_len;

    if (!atomic_read(&ch->linked)) {
        printk("l2tp_xmit: channel not linked\n");
        kfree_skb(skb);
        return 1;
    }
    rt = lookup_rt(ch, ppp_get_device(chan));
    if (!rt) {
        dst_link_failure(skb); // this is supposed to generate ICMP error
	kfree_skb(skb);
	return 1;
    }

    head_len = ch->xmit_hdr_size;
    if (head_len > skb_headroom(skb) || skb_cloned(skb) || skb_shared(skb)) {
	struct sk_buff *nskb = skb_realloc_headroom(skb, head_len);
	if (!nskb) {
	    printk("l2tp: failed to expand buffer\n");
	    kfree_skb(skb);
	    return 1;
	}

	if (skb->sk) skb_set_owner_w(nskb, skb->sk);
	kfree_skb(skb);
	skb = nskb;
    }

    skb_push(skb, head_len);
    skb_reset_network_header(skb);
    skb_set_transport_header(skb, sizeof(struct iphdr));
    memset(&(IPCB(skb)->opt), 0, sizeof(IPCB(skb)->opt));
    skb_dst_drop(skb);
    skb_dst_set_noref(skb, &rt->rt->dst);
    skb->protocol = __constant_htons(ETH_P_IP);

    nf_reset_ct(skb);

    iph = ip_hdr(skb);
    iph->version = 4;
    iph->ihl = 5;
    iph->tos = 0;
    iph->tot_len = htons(skb->len);
    iph->frag_off = 0;
    iph->ttl = 0x40;
    iph->protocol = l2tp_ch_ipproto(ch);
    iph->saddr = rt->saddr;
    iph->daddr = rt->daddr;
    iph->id = ch->peer_handle ? gre_peer_handle_next_identity(ch->peer_handle) : 0;

    skb->ip_summed = CHECKSUM_NONE;
    ip_send_check(iph);

    ch->hdr_prep_fn(skb, ch, iph);

    NF_HOOK(PF_INET, NF_INET_LOCAL_OUT, dev_net(rt->rt->dst.dev), NULL,
	    skb, NULL, rt->rt->dst.dev, dst_output);

    return 1;
}

int l2tp_xmit6(struct ppp_channel *chan, struct sk_buff *skb) {
    struct ipv6hdr *iph6;
    struct l2tp_channel *ch = chan->private;
    struct rt_cache *rt_cache;
    unsigned head_len;

    if (!atomic_read(&ch->linked)) {
        printk("l2tp_xmit: channel not linked\n");
        kfree_skb(skb);
        return 1;
    }
    rt_cache = lookup_rt6(ch, ppp_get_device(chan));
    if (!rt_cache) {
        dst_link_failure(skb); // this is supposed to generate ICMP error
        kfree_skb(skb);
        return 1;
    }

    head_len = ch->xmit_hdr_size;
    if ((head_len + LL_MAX_HEADER) > skb_headroom(skb) || skb_cloned(skb) || skb_shared(skb)) {
        struct sk_buff *nskb = skb_realloc_headroom(skb, head_len + LL_MAX_HEADER);
        if (!nskb) {
            printk("l2tp: failed to expand buffer\n");
            kfree_skb(skb);
            return 1;
        }

        if (skb->sk) skb_set_owner_w(nskb, skb->sk);
        kfree_skb(skb);
        skb = nskb;
    }

    skb_push(skb, head_len);
    skb_reset_network_header(skb);
    skb_set_transport_header(skb, sizeof(struct ipv6hdr));
    memset(&(IPCB(skb)->opt), 0, sizeof(IPCB(skb)->opt));
    skb_dst_drop(skb);
    skb_dst_set_noref(skb, rt_cache->dst);
    skb->protocol = __constant_htons(ETH_P_IPV6);

    nf_reset_ct(skb);

    iph6 = ipv6_hdr(skb);
    iph6->version = 6;
    iph6->priority = 0;
    put_unaligned(ch->info.local_sesid, (u16 *)&iph6->flow_lbl[1]);
    iph6->hop_limit = 0x40;
    iph6->nexthdr = l2tp_ch_ipproto(ch);
    iph6->saddr = rt_cache->fl6.saddr;
    iph6->daddr = rt_cache->fl6.daddr;
    iph6->payload_len = htons(skb->len - sizeof(struct ipv6hdr));

    skb->ip_summed = CHECKSUM_NONE;

    ch->hdr_prep_fn(skb, ch, iph6);

    NF_HOOK(PF_INET6, NF_INET_LOCAL_OUT, dev_net(rt_cache->dst->dev), NULL,
            skb, NULL, rt_cache->dst->dev, dst_output);

    return 1;
}

static struct ppp_channel_ops l2tp_chanops = {
    .start_xmit = l2tp_xmit,
    .attached = l2tp_chan_attached,
    .lockless = 1,
};

static struct ppp_channel_ops l2tp_chanops6 = {
    .start_xmit = l2tp_xmit6,
    .lockless = 1,
};

static inline unsigned l2tp_get_head_len(const struct l2tp_channel *ch) {
    unsigned head_len;
    const int ver = ch->info.l2tp_ver;

    head_len = l2tp_ch_isip4(ch) ? sizeof(struct iphdr) : sizeof(struct ipv6hdr);
    if (L2TPv3_IP == ver)
        head_len += (ch->info.rcookie_sz + 6); /*sessid + 0xff03*/
    else // udp
        head_len += sizeof(struct udphdr) + 8; /*ver + tun + sess + 0xff03*/

    if (L2TPv3_UDP == ver)
        head_len += ch->info.rcookie_sz + 2; /* UDP v3 *reserved* field */

    return head_len;
}

static inline void l2tp_ch_initifl6(struct l2tp_channel *ch) {
    memset(&ch->fl6, 0, sizeof(ch->fl6));
    ch->fl6.flowi6_proto = l2tp_ch_ipproto(ch);
    memcpy(ch->fl6.saddr.in6_u.u6_addr8, ch->info.src_addr6, sizeof(ch->fl6.saddr.in6_u.u6_addr8));
    memcpy(ch->fl6.daddr.in6_u.u6_addr8, ch->info.remote_addr6, sizeof(ch->fl6.daddr.in6_u.u6_addr8));
    ch->fl6.flowlabel = ch->info.local_sesid & 0xFFFF;
    if (IPPROTO_UDP == l2tp_ch_ipproto(ch)) ch->fl6.fl6_dport = ch->info.remote_port;
}

static int l2tp_validate_ipv6_addr(const struct in6_addr *addr) {
    const int type = ipv6_addr_type(addr);
    if (IPV6_ADDR_MAPPED == type) {
        printk("l2tp_add_channel: IPv6 mapped IPv4 addresses are not supported\n");
        return -EADDRNOTAVAIL;
    }
    if (type & IPV6_ADDR_MULTICAST) {
        printk("l2tp_add_channel: multicast IPv6 addresses are not supported\n");
        return -EADDRNOTAVAIL;
    }
    return 0;
}

static int l2tp_validate_ipv6(const struct flowi6 *fl6) {
    int err;

    err = l2tp_validate_ipv6_addr(&fl6->daddr);
    if (err) return err;
    err = l2tp_validate_ipv6_addr(&fl6->saddr);
    if (err) return err;

    return 0;
}

int l2tp_add_channel(struct l2tp_add *a) {
    struct l2tp_channel *ch = NULL;
    struct rt_cache *rt;
    unsigned h;
    int channel, ipv4, err;

    ch = kmalloc(sizeof(*ch), GFP_ATOMIC);
    if (!ch) {
        printk("l2tp_add_channel: failed to alloc\n");
        return -ENOMEM;
    }
    memset(ch, 0, sizeof(*ch));

    atomic_set(&ch->linked, 1);
    memcpy(&ch->info, &a->data.in, sizeof(ch->info));

    if (ch->info.rcookie_sz) {
        if (ch->info.rcookie_sz != 8 && ch->info.rcookie_sz != 4) {
            printk("l2tp_add_channel: invalid remote cookie size %d\n",
                    ch->info.rcookie_sz);
            kfree(ch);
            return -EINVAL;
        }
    }
    if (ch->info.lcookie_sz) {
        if (ch->info.lcookie_sz != 8 && ch->info.lcookie_sz != 4) {
            printk("l2tp_add_channel: invalid local cookie size %d\n",
                    ch->info.lcookie_sz);
            kfree(ch);
            return -EINVAL;
        }
    }
    ipv4 = l2tp_ch_isip4(ch);

    ch->chan.hdrlen = LL_MAX_HEADER + (ipv4 ? sizeof(struct iphdr) : sizeof(struct ipv6hdr)) +
                      sizeof(struct udphdr) + 8 + 2;
    ch->chan.private = ch;
    ch->chan.ops = ipv4 ? &l2tp_chanops : &l2tp_chanops6;

    if (ipv4) {
        rt = lookup_rt(ch, NULL);
        if (rt) {
            ch->peer_handle = gre_peer_handle_get(rt->saddr, rt->daddr);
        }
    }
    else {
        ch->info.allow_fast_path = 0;
        l2tp_ch_initifl6(ch);
        err = l2tp_validate_ipv6(&ch->fl6);
        if (err) {
            kfree(ch);
            return err;
        }
    }

    if (ppp_register_channel(&ch->chan) != 0) {
        printk("l2tp_add_channel: failed to register\n");
        kfree(ch);
        return -EIO;
    }
    channel = ppp_channel_index(&ch->chan);

    switch (ch->info.l2tp_ver) {
    case L2TPv3_UDP:
        ch->hdr_prep_fn = l2tp_xmit_udp3_header_prep;
        break;
    case L2TPv3_IP:
        ch->hdr_prep_fn = l2tp_xmit_ip_header_prep;
        break;
    default:
        ch->hdr_prep_fn = l2tp_xmit_udp_header_prep;
    }
    ch->xmit_hdr_size = l2tp_get_head_len(ch);
    ch->receive_fn = l2tp_receive_ppp;

    h = ses_hash(ch->info.local_sesid, ch->info.local_tunid);

    write_lock_bh(&chan_lock);
    hlist_add_head_rcu(&ch->next, &ses_table[h]);
    write_unlock_bh(&chan_lock);

    printk("l2tp_add_channel: added ppp channel %u:%u -> %d\n",
	   ntohl(ch->info.local_sesid), ntohl(ch->info.local_tunid),
	   channel);

    return channel;
}

static inline void free_chan_data(struct l2tp_channel *ch) {
    if (ch->peer_handle) gre_peer_handle_put(ch->peer_handle);
    if (ch->rt_cache) free_rt(ch->rt_cache);
    if (ch->stats) kfree(ch->stats);
    kfree_rcu(ch, rcu);
}

int l2tp_del_channel(int index) {
    int i;

    write_lock_bh(&chan_lock);

    for (i = 0; i < L2TP_HASHSIZE; ++i) {
        struct l2tp_channel *ch;

        hlist_for_each_entry_rcu(ch, &ses_table[i], next) {
            if (PW_ETH == ch->info.wire_type && index == ntohl(ch->info.local_sesid)) {
                atomic_set(&ch->linked, 0);
                hlist_del_rcu(&ch->next);
                write_unlock_bh(&chan_lock);

                if (net_ratelimit()) {
                    printk("l2tp_del_channel: unreg net dev %s (l2tpv3 sessid %u -> %u) \n",
                           ch->dev->name,
                           ntohl(ch->info.local_sesid),
                           ntohl(ch->info.remote_sesid));
                    print_hw("    hw addr: ", ch->dev->dev_addr);
                }
                unregister_netdev(ch->dev);

                free_chan_data(ch);
                if (net_ratelimit())
                    printk("l2tp_del_channel: unregistered ether channel %d\n", index);
                return 0;
            }
            else if (ppp_channel_index(&ch->chan) == index) {
                atomic_set(&ch->linked, 0);
                hlist_del_rcu(&ch->next);
                write_unlock_bh(&chan_lock);

                ppp_unregister_channel(&ch->chan);

                free_chan_data(ch);

                if (net_ratelimit())
                    printk("l2tp_del_channel: unregistered ppp channel %d\n", index);
                return 0;
            }
        }
    }

    write_unlock_bh(&chan_lock);

    printk("l2tp_del_channel: could not find channel %d\n", index);
    return -ENODEV;
}

void l2tp_del_all(void) {
    while (1) {
        struct l2tp_channel *ch = NULL;
        int i;

	write_lock_bh(&chan_lock);

        for (i = 0; i < L2TP_HASHSIZE; ++i) {
	    hlist_for_each_entry_rcu(ch, &ses_table[i], next) {
		atomic_set(&ch->linked, 0);
		hlist_del_rcu(&ch->next);
                goto out;
            }
        }
      out:
	write_unlock_bh(&chan_lock);

        if (!ch) break;

        if (PW_ETH == ch->info.wire_type) {
            if (net_ratelimit())
                printk("l2tp_del_ethchannel: unreg net dev %s \n", ch->dev->name);
            unregister_netdev(ch->dev);
        }
        else {
            ppp_unregister_channel(&ch->chan);

            if (net_ratelimit()) printk("l2tp_del_all: unregistered channel\n");
        }

        if (ch->peer_handle)
            gre_peer_handle_put(ch->peer_handle);
        free_rt(ch->rt_cache);
        if (ch->stats) kfree(ch->stats);
        kfree_rcu(ch, rcu);
    }
}

static int handle_l2tpv3_cookie(struct l2tp_channel *ch, struct sk_buff *skb) {
    int v2 = L2TPv2 == ch->info.l2tp_ver;
    if (v2 || !ch->info.lcookie_sz) return 0;

    int size = ch->info.lcookie_sz;
    if (skb->len < size) {
        if (net_ratelimit()) {
            printk("handle_l2tpv3_cookie: invalid session packet "
                   "received (too small), ses %u -> %u\n",
                   ntohl(ch->info.local_sesid), ntohl(ch->info.remote_sesid));
        }
        return -1;
    }

    if (memcmp(skb->data, ch->info.local_cookie, size)) {
        if (net_ratelimit()) {
            printk("handle_l2tpv3_cookie: invalid cookie for ses %u -> %u, "
                   "dropping\n", ntohl(ch->info.local_sesid),
                   ntohl(ch->info.remote_sesid));
        }
        return -1;
    }

    skb_pull(skb, size);

    return 0;
}

inline void l2tp_receive_ppp(struct l2tp_channel *ch, struct sk_buff *skb) {
    if (skb->len > 2 && skb->data[0] == 0xff && skb->data[1] == 0x03)
        skb_pull(skb, 2);

    ppp_input(&ch->chan, skb);
}

inline void l2tp_receive_ether(struct l2tp_channel *ch, struct sk_buff *skb) {
    int err;

    skb->dev = ch->dev;

    if (ch->info.have_l2specific_layer && skb->len > 4) skb_pull(skb, 4); // L2-Specific Sublayer

    skb->protocol = eth_type_trans(skb, skb->dev);
    ch->stats->last_recv = jiffies;
    __skb_tunnel_rx(skb, skb->dev, dev_net(skb->dev));
    err = netif_rx_ni(skb);
    if (err) {
        atomic_long_inc(&ch->stats->rx_errors);
    }
    else {
        atomic_long_add(skb->len, &ch->stats->rx_bytes);
        atomic_long_inc(&ch->stats->rx_packets);
    }
}

void l2tp_receive(unsigned sesid, unsigned tunid, struct sk_buff *skb) {
    int is_v3, err;
    struct l2tp_channel *ch;
    unsigned h = ses_hash(sesid, tunid);

    if (skb_has_frag_list(skb)) {
	struct sk_buff *oskb = skb;
	skb = skb_copy(oskb, GFP_ATOMIC);

	kfree_skb(oskb);
	if (!skb) {
	    return;
	}
    }

    hlist_for_each_entry_rcu(ch, &ses_table[h], next) {
        if (ch->info.local_sesid == sesid && ch->info.local_tunid == tunid) {
            err = handle_l2tpv3_cookie(ch, skb);
            if (err) goto drop;

            ch->receive_fn(ch, skb);
	    return;
	}
    }

    if (net_ratelimit()) {
        is_v3 = 0 == tunid; /*l2tpv3 identifies data messages only by ses ID*/
        if (is_v3) {
            printk("l2tp_receive: no channel for session %u (L2TPv3)\n",
                    ntohl(sesid));
        }
        else {
            printk("l2tp_receive: no channel for %u:%u (L2TPv2)\n",
                   ntohs(sesid), ntohs(tunid));
        }
    }

drop:
    kfree_skb(skb);
    return;
}

static void l2tpeth_xmit_ip_header_prep(struct sk_buff *skb,
                                        struct l2tp_channel *ch,
                                        void *iphdr) {
    u32 *hdr = l2tp_ch_isip4(ch) ? (u32 *)((struct iphdr *)iphdr + 1)
                                   : (u32 *)((struct ipv6hdr *)iphdr + 1);

    put_unaligned(ch->info.remote_sesid, hdr);
    hdr++;
    if (ch->info.rcookie_sz) {
        memcpy(hdr, ch->info.remote_cookie, ch->info.rcookie_sz);
        hdr += (ch->info.rcookie_sz / sizeof(hdr[0]));
    }
    if (ch->info.have_l2specific_layer) {
        put_unaligned(0, hdr);
    }
}

static void l2tpeth_xmit_udp3_header_prep(struct sk_buff *skb,
                                          struct l2tp_channel *ch,
                                          void *iphdr) {
    unsigned udp_len;
    struct udphdr *udph;
    u16 *hdr;
    int ipv4 = l2tp_ch_isip4(ch);

    udp_len = skb->len - (ipv4 ? sizeof(struct iphdr) : sizeof(struct ipv6hdr));
    udph = udp_hdr(skb);
    udph->source = inet_sk(udp_sock(ch))->inet_sport;
    udph->dest = ch->info.remote_port;
    udph->len = htons(udp_len);
    udph->check = 0;

    hdr = (u16 *)(udph + 1);
    put_unaligned(__constant_htons(L2TP_VERSION3), hdr);
    hdr++;
    put_unaligned(0, hdr); // *reserved* v3 udp header field
    hdr++;
    memcpy(hdr, &ch->info.remote_sesid, sizeof(ch->info.remote_sesid));
    hdr += 2;
    if (ch->info.rcookie_sz) {
        memcpy(hdr, ch->info.remote_cookie, ch->info.rcookie_sz);
        hdr += (ch->info.rcookie_sz / sizeof(hdr[0]));
    }
    if (ch->info.have_l2specific_layer) {
        put_unaligned(0, (u32 *)hdr); // L2-Specific Sublayer
    }

    if (ipv4) {
        struct iphdr *iph = iphdr;
        udph->check = csum_tcpudp_magic(iph->saddr, iph->daddr, udp_len,
                                        IPPROTO_UDP, csum_partial(udph, udp_len, 0));
    }
    else {
        struct ipv6hdr *iph = iphdr;
        udph->check = csum_ipv6_magic(&iph->saddr, &iph->daddr, udp_len,
                                      IPPROTO_UDP, csum_partial(udph, udp_len, 0));
    }
}

static int l2tpeth_dev_xmit_skb(struct l2tp_channel *ch, struct sk_buff *skb) {
    struct iphdr *iph;
    struct rt_cache *rt;
    unsigned head_len;

    if (!atomic_read(&ch->linked)) {
        printk("l2tpeth_dev_xmit_skb: channel not linked\n");
        kfree_skb(skb);
        return 1;
    }

    rt = lookup_rt(ch, ch->dev);
    if (!rt) {
        dst_link_failure(skb); // this is supposed to generate ICMP error
        kfree_skb(skb);
        return 1;
    }

    head_len = ch->xmit_hdr_size;
    if ((LL_MAX_HEADER + head_len) > skb_headroom(skb) || skb_cloned(skb) || skb_shared(skb)) {
        struct sk_buff *nskb = skb_realloc_headroom(skb, head_len + LL_MAX_HEADER);
        if (!nskb) {
            printk("l2tpeth_dev_xmit_skb: failed to expand buffer\n");
            kfree_skb(skb);
            return 1;
        }

        if (skb->sk) skb_set_owner_w(nskb, skb->sk);
        kfree_skb(skb);
        skb = nskb;
    }

    skb_push(skb, head_len);
    skb_reset_network_header(skb);
    skb_set_transport_header(skb, sizeof(struct iphdr));
    memset(&(IPCB(skb)->opt), 0, sizeof(IPCB(skb)->opt));
    skb_dst_drop(skb);
    skb_dst_set_noref(skb, &rt->rt->dst);
    skb->protocol = __constant_htons(ETH_P_IP);

    nf_reset_ct(skb);

    iph = ip_hdr(skb);
    iph->version = 4;
    iph->ihl = 5;
    iph->tos = 0;
    iph->tot_len = htons(skb->len);
    iph->frag_off = 0;
    iph->ttl = 0x40;
    iph->protocol = l2tp_ch_ipproto(ch);
    iph->saddr = rt->saddr;
    iph->daddr = rt->daddr;
    iph->id = ch->peer_handle ? gre_peer_handle_next_identity(ch->peer_handle) : 0;

    skb->ip_summed = CHECKSUM_NONE;
    ip_send_check(iph);

    ch->hdr_prep_fn(skb, ch, iph);

    ch->stats->last_xmit = jiffies;
    NF_HOOK(PF_INET, NF_INET_LOCAL_OUT, dev_net(rt->rt->dst.dev), NULL,
            skb, NULL, rt->rt->dst.dev, dst_output);
    return 0;
}

static int l2tpeth_dev_xmit_skb_ipv6(struct l2tp_channel *ch, struct sk_buff *skb) {
    struct ipv6hdr *iph6;
    struct rt_cache *rt_cache;
    unsigned head_len;

    if (!atomic_read(&ch->linked)) {
        printk("l2tpeth_dev_xmit_skb: channel not linked\n");
        kfree_skb(skb);
        return NET_XMIT_DROP;
    }

    rt_cache = lookup_rt6(ch, ch->dev);
    if (!rt_cache) {
        dst_link_failure(skb);
        kfree_skb(skb);
        return NET_XMIT_DROP;
    }

    head_len = ch->xmit_hdr_size;
    if ((LL_MAX_HEADER + head_len) > skb_headroom(skb) || skb_cloned(skb) || skb_shared(skb)) {
        struct sk_buff *nskb = skb_realloc_headroom(skb, head_len + LL_MAX_HEADER);
        if (!nskb) {
            printk("l2tpeth_dev_xmit_skb: failed to expand buffer\n");
            kfree_skb(skb);
            return NET_XMIT_DROP;
        }

        if (skb->sk) skb_set_owner_w(nskb, skb->sk);
        kfree_skb(skb);
        skb = nskb;
    }

    skb_push(skb, head_len);
    skb_reset_network_header(skb);
    skb_set_transport_header(skb, sizeof(struct ipv6hdr));
    memset(&(IPCB(skb)->opt), 0, sizeof(IPCB(skb)->opt));
    skb_dst_drop(skb);
    skb_dst_set_noref(skb, rt_cache->dst);
    skb->protocol = __constant_htons(ETH_P_IPV6);

    nf_reset_ct(skb);

    iph6 = ipv6_hdr(skb);
    iph6->version = 6;
    iph6->priority = 0;
    put_unaligned(ch->info.local_sesid, (u16 *)&iph6->flow_lbl[1]);
    iph6->hop_limit = 0x40;
    iph6->nexthdr = l2tp_ch_ipproto(ch);
    iph6->saddr = rt_cache->fl6.saddr;
    iph6->daddr = rt_cache->fl6.daddr;
    iph6->payload_len = htons(skb->len - sizeof(struct ipv6hdr));

    skb->ip_summed = CHECKSUM_NONE;

    ch->hdr_prep_fn(skb, ch, iph6);

    ch->stats->last_xmit = jiffies;

    NF_HOOK(PF_INET6, NF_INET_LOCAL_OUT, dev_net(rt_cache->dst->dev), NULL,
            skb, NULL, rt_cache->dst->dev, dst_output);

    return NET_XMIT_SUCCESS;
}

inline struct l2tp_channel *l2tpeth_priv(struct net_device *dev) {
    return *(struct l2tp_channel **)netdev_priv(dev);
}

static inline void l2tpeth_dev_update_stats(struct sk_buff *skb,
                                       struct l2tp_channel *ch, int xmit_result) {
    if (likely(0 == xmit_result)) {
        atomic_long_add(skb->len, &ch->stats->tx_bytes);
        atomic_long_inc(&ch->stats->tx_packets);
    }
    else {
        atomic_long_inc(&ch->stats->tx_dropped);
    }
}

int l2tpeth_dev_xmit(struct sk_buff *skb, struct net_device *dev) {
    int ret;
    struct l2tp_channel *ch = l2tpeth_priv(dev);

    ret = l2tpeth_dev_xmit_skb(ch, skb);
    l2tpeth_dev_update_stats(skb, ch, ret);
    return NETDEV_TX_OK;
}

int l2tpeth_dev_xmit6(struct sk_buff *skb, struct net_device *dev) {
    int ret;
    struct l2tp_channel *ch = l2tpeth_priv(dev);

    ret = l2tpeth_dev_xmit_skb_ipv6(ch, skb);
    l2tpeth_dev_update_stats(skb, ch, ret);
    return NETDEV_TX_OK;
}

static int l2tpeth_dev_open(struct net_device *dev) {
    netif_start_queue(dev);
    return 0;
}

static int l2tpeth_dev_close(struct net_device *dev) {
    netif_stop_queue(dev);
    return 0;
}

static void l2tpeth_get_stats64(struct net_device *dev,
                                struct rtnl_link_stats64 *stats) {
    struct l2tp_channel *ch = l2tpeth_priv(dev);
    stats->tx_bytes   = (unsigned long) atomic_long_read(&ch->stats->tx_bytes);
    stats->tx_packets = (unsigned long) atomic_long_read(&ch->stats->tx_packets);
    stats->tx_dropped = (unsigned long) atomic_long_read(&ch->stats->tx_dropped);
    stats->rx_bytes   = (unsigned long) atomic_long_read(&ch->stats->rx_bytes);
    stats->rx_packets = (unsigned long) atomic_long_read(&ch->stats->rx_packets);
    stats->rx_errors  = (unsigned long) atomic_long_read(&ch->stats->rx_errors);
}

static int l2tpeth_dev_changel2mtu(struct net_device *dev, int l2mtu) {
    dev->l2mtu = 65535;
    return 0;
}

static int l2tpeth_dev_changemtu(struct net_device *dev, int mtu) {
    if (mtu < 68 || mtu > 4096) return -EINVAL;
    dev->mtu = mtu;
    return 0;
}

static int l2tpeth_set_mac_address(struct net_device *dev, unsigned char *addr) {
    unsigned char tmp[ETH_ALEN];
    int empty;

    eth_broadcast_addr(dev->broadcast);

    if (is_valid_ether_addr(addr)) {
        memcpy(dev->dev_addr, addr, ETH_ALEN);
        return 0;
    }

    memset(tmp, 0, ETH_ALEN);
    empty = 0 == memcmp(addr, tmp, ETH_ALEN);
    printk("l2tpeth_set_mac_address: %s mac address, "
            "setting random hwaddr\n", empty ? "empty" : "invalid");
    random_ether_addr(dev->dev_addr);
    return 0;
}

static struct net_device_ops l2tpeth_netdev_ops = {
    .ndo_start_xmit = &l2tpeth_dev_xmit,
    .ndo_open = &l2tpeth_dev_open,
    .ndo_stop = &l2tpeth_dev_close,
    .ndo_get_stats64 = &l2tpeth_get_stats64,
    .ndo_change_mtu = &l2tpeth_dev_changemtu,
    .ndo_change_l2mtu = &l2tpeth_dev_changel2mtu,
    .ndo_set_mac_address = &eth_mac_addr,
};

static struct net_device_ops l2tpeth_netdev_ops6 = {
    .ndo_start_xmit = &l2tpeth_dev_xmit6,
    .ndo_open = &l2tpeth_dev_open,
    .ndo_stop = &l2tpeth_dev_close,
    .ndo_get_stats64 = &l2tpeth_get_stats64,
    .ndo_change_mtu = &l2tpeth_dev_changemtu,
    .ndo_set_mac_address = &eth_mac_addr,
};

static void l2tpeth_dev_setup(struct net_device *dev) {
    ether_setup(dev);

    dev->netdev_ops = &l2tpeth_netdev_ops;

    dev->features |= NETIF_F_LLTX;
    dev->needs_free_netdev = 1;
}

static void l2tpeth_dev_setup6(struct net_device *dev) {
    ether_setup(dev);

    dev->netdev_ops = &l2tpeth_netdev_ops6;

    dev->features |= NETIF_F_LLTX;
    dev->needs_free_netdev = 1;
}

static int l2tpeth_iface_exists(const char* ifname) {
    int i;

    write_lock_bh(&chan_lock);
    for (i = 0; i < L2TP_HASHSIZE; ++i) {
        struct l2tp_channel *tmp;
        hlist_for_each_entry_rcu(tmp, &ses_table[i], next) {
            if (PW_ETH == tmp->info.wire_type) {
                if (0 == strncmp(ifname, tmp->dev->name, IFNAMSIZ)) {
                    printk("l2tp_add_ethchan: device with name '%s' already exists (ses %u)\n",
                            ifname,
                            ntohl(tmp->info.local_sesid));
                    write_unlock_bh(&chan_lock);
                    return 1;
                }
            }
        }
    }
    write_unlock_bh(&chan_lock);

    return 0;
}

static unsigned l2tpeth_get_head_len(struct l2tp_channel *ch) {
    const int sessid_sz = 4; // 32bit for L2TPv3
    const int l2_sublayer_sz = ch->info.have_l2specific_layer ? 4 : 0; // 32bit L2TPv3 L2-specific sublayer
    const int udp_mode = ch->info.l2tp_ver != L2TPv3_IP;
    unsigned head_len = l2tp_ch_isip4(ch) ? sizeof(struct iphdr)
                                           : sizeof(struct ipv6hdr);

    if (udp_mode) {
        head_len += sizeof(struct udphdr);
        head_len += 4; // flags & ver + reserved field
    }
    head_len += (sessid_sz + ch->info.rcookie_sz + l2_sublayer_sz);

    return head_len;
}

static inline void l2tpeth_dev_adjust_mtu(struct net_device *dev,
                                          const struct l2tp_channel *ch,
                                          const struct dst_entry *dst,
                                          unsigned min_mtu) {
    unsigned dstmtu = dst_mtu(dst);

    if (dstmtu < min_mtu) return;

    dev->mtu = dstmtu - ch->xmit_hdr_size - ETH_HLEN;
    if (ch->info.rcookie_sz < ch->info.lcookie_sz)
        dev->mtu -= (ch->info.lcookie_sz - ch->info.rcookie_sz);
}

int l2tp_add_ethchan(struct l2tp_add *a) {
    struct net_device *dev = NULL;
    struct l2tp_channel *ch = NULL;
    struct rt_cache *rt;
    int channel = 0;
    int err, udp_mode, ipv4;
    unsigned h;
    const char* ifname = a->data.in.ifname;

    if (0 == ifname[0]) {
        printk("l2tp_add_ethchan: ifname not set\n");
        return -EINVAL;
    }

    if (l2tpeth_iface_exists(ifname)) return -EEXIST;

    ch = kmalloc(sizeof(*ch), GFP_ATOMIC);
    if (!ch) {
        printk("l2tp_add_ethchan: failed to alloc\n");
        return -ENOMEM;
    }
    memset(ch, 0, sizeof(*ch));
    memcpy(&ch->info, &a->data.in, sizeof(ch->info));

    ipv4 = l2tp_ch_isip4(ch);
    dev = alloc_netdev(sizeof(ch), ifname, NET_NAME_USER,
                       ipv4 ? l2tpeth_dev_setup : l2tpeth_dev_setup6);

    if (!dev) {
        printk("l2tp_add_ethchan: failed to allocate netdev\n");
        goto out;
    }

    if (ch->info.rcookie_sz) {
        if (ch->info.rcookie_sz != 8 && ch->info.rcookie_sz != 4) {
            printk("l2tp_add_ethchan: invalid remote cookie size %d\n",
                    ch->info.rcookie_sz);
            goto out;
        }
    }
    if (ch->info.lcookie_sz) {
        if (ch->info.lcookie_sz != 8 && ch->info.lcookie_sz != 4) {
            printk("l2tp_add_ethchan: invalid local cookie size %d\n",
                    ch->info.lcookie_sz);
            goto out;
        }
    }
    ch->stats = kmalloc(sizeof(*ch->stats), GFP_ATOMIC);
    if (!ch->stats) {
        printk("l2tp_add_ethchan: failed to alloc stats\n");
        goto out;
    }
    memset(ch->stats, 0, sizeof(*ch->stats));
    ch->stats->last_recv = jiffies;
    ch->stats->last_xmit = jiffies;
    udp_mode = ch->info.l2tp_ver != L2TPv3_IP;
    ch->dev = dev;
    ch->hdr_prep_fn = udp_mode ? l2tpeth_xmit_udp3_header_prep : l2tpeth_xmit_ip_header_prep;
    ch->receive_fn  = l2tp_receive_ether;
    ch->xmit_hdr_size = l2tpeth_get_head_len(ch);
    /*convert to net order for faster lookup when receiving data messages*/
    ch->info.local_sesid = htonl(ch->info.local_sesid);
    ch->info.local_tunid = htonl(ch->info.local_tunid);
    if (!ipv4) ch->info.allow_fast_path = 0;

    if (!ipv4) {
        l2tp_ch_initifl6(ch);
        err = l2tp_validate_ipv6(&ch->fl6);
        if (err) goto out;
    }


    memcpy(netdev_priv(dev), &ch, sizeof(ch));
    l2tpeth_set_mac_address(dev, a->data.in.hwaddr);
    dev->needed_headroom += ETH_HLEN + ch->xmit_hdr_size + LL_MAX_HEADER;

    dev_net_set(dev, &init_net);
    dev->min_mtu = 0;
    dev->l2mtu = 65535;
    dev->max_mtu = ETH_MAX_MTU;
    err = register_netdev(dev);
    if (err < 0) {
        printk("l2tp_add_ethchan: failed to register netdev '%s' ses %u (%d)\n",
                a->data.in.ifname, a->data.in.local_sesid, err);
        goto out;
    }

    atomic_set(&ch->linked, 1);

    if (ipv4) {
        rt = lookup_rt(ch, NULL);
        if (rt) {
            ch->peer_handle = gre_peer_handle_get(rt->saddr, rt->daddr);

            if (rt->rt)
                l2tpeth_dev_adjust_mtu(dev, ch, &rt->rt->dst, IPV4_MIN_MTU);
        }
    }
    else {
        rt = lookup_rt6(ch, NULL);
        if (rt && rt->dst)
            l2tpeth_dev_adjust_mtu(dev, ch, rt->dst, IPV6_MIN_MTU);
    }

    a->data.out.actual_mtu = dev->mtu;

    h = ses_hash(ch->info.local_sesid, ch->info.local_tunid);

    write_lock_bh(&chan_lock);
    hlist_add_head_rcu(&ch->next, &ses_table[h]);
    write_unlock_bh(&chan_lock);

    channel = ntohl(ch->info.local_sesid); // L2TPv3 session ID is unique per host
    if (net_ratelimit()) {
        printk("l2tp_add_ethchan: added %s channel l2tpv3 sessid %u -> %u, chan %d, %s\n",
               udp_mode ? "UDP" : "IP",
               ntohl(ch->info.local_sesid),
               ntohl(ch->info.remote_sesid),
               channel, a->data.in.ifname);
        print_cookie("    local cookie", ch->info.local_cookie, ch->info.lcookie_sz);
        print_cookie("    remote cookie", ch->info.remote_cookie, ch->info.rcookie_sz);
    }

    return channel;
out:
    if (dev) free_netdev(dev);
    if (ch) {
        if (ch->stats) kfree(ch->stats);
        kfree(ch);
    }

    return -EIO;
}

int l2tp_eth_getidle(struct l2tp_idle_info *info) {
    int i;

    read_lock_bh(&chan_lock);
    for (i = 0; i < L2TP_HASHSIZE; ++i) {
        struct l2tp_channel *ch;

        hlist_for_each_entry_rcu(ch, &ses_table[i], next) {
            if (PW_ETH == ch->info.wire_type && info->channel == ch->info.local_sesid) {
                info->recv_idle = jiffies - ch->stats->last_recv;
                info->xmit_idle = jiffies - ch->stats->last_xmit;
                read_unlock_bh(&chan_lock);
                return 0;
            }
        }
    }
    read_unlock_bh(&chan_lock);

    printk("l2tp_eth_getidle: could not find channel %d\n", info->channel);
    return -ENODEV;
}
