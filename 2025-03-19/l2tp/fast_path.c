#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/ppp_channel.h>
#include <linux/ppp_defs.h>
#include <linux/if_ppp.h>
#include <linux/packet_hook.h>
#include <linux/types.h>
#include <net/netns/generic.h>

#include "l2tp.h"
#include "chan.h"
#include "dev.h"

extern void ppp_get_stats(struct ppp *ppp, struct ppp_stats *st);
extern netdev_tx_t ppp_start_xmit(struct sk_buff *skb, struct net_device *dev);
extern int ppp_net_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);
extern int l2tp_xmit(struct ppp_channel *chan, struct sk_buff *skb);

int l2tpeth_dev_xmit(struct sk_buff *skb, struct net_device *dev);
struct l2tp_channel *l2tpeth_priv(struct net_device *dev);

static int is_tcp_syn(const unsigned char *buf, unsigned len)
{
	const struct iphdr *iph = (const struct iphdr *) buf;
	const struct tcphdr *tcph;
	unsigned hlen;

	if (len < sizeof(struct iphdr))
		return 0;

	if (iph->protocol != IPPROTO_TCP)
		return 0;
	if (iph->ihl < 5 || iph->version != 4)
		return 0;
	if (iph->frag_off & __constant_htons(IP_OFFSET))
		return 0;

	hlen = iph->ihl * 4;
	if (len < hlen + sizeof(struct tcphdr))
		return 0;

	tcph = (struct tcphdr *) ((unsigned char *) iph + hlen);
	return tcph->syn;
}

static inline int ethertype_to_npindex(unsigned ethertype, unsigned *proto)
{
	switch (ethertype) {
	case __constant_htons(ETH_P_IP):
		*proto = __constant_htons(PPP_IP);
		return NP_IP;
	case __constant_htons(ETH_P_IPV6):
		*proto = __constant_htons(PPP_IPV6);
		return NP_IPV6;
	case __constant_htons(ETH_P_MPLS_UC):
		*proto = __constant_htons(PPP_MPLS_UC);
		return NP_MPLS_UC;
	case __constant_htons(ETH_P_MPLS_MC):
		*proto = __constant_htons(PPP_MPLS_MC);
		return NP_MPLS_MC;
	}
	return -1;
}

static inline int proto_to_npindex(unsigned proto, unsigned *ethertype) {
	switch (proto) {
	case __constant_htons(PPP_IP):
		*ethertype = __constant_htons(ETH_P_IP);
		return NP_IP;
	case __constant_htons(PPP_IPV6):
		*ethertype = __constant_htons(ETH_P_IPV6);
		return NP_IPV6;
	case __constant_htons(PPP_MPLS_UC):
		*ethertype = __constant_htons(ETH_P_MPLS_UC);
		return NP_MPLS_UC;
	case __constant_htons(PPP_MPLS_MC):
		*ethertype = __constant_htons(ETH_P_MPLS_MC);
		return NP_MPLS_MC;
	}
	return -1;
}

static struct l2tp_channel *find_session(unsigned tunid, unsigned sesid) {
    struct l2tp_channel *ch;
    hlist_for_each_entry_rcu(ch, &ses_table[ses_hash(tunid, sesid)], next) {
        if (ch->info.local_sesid == sesid
                && ch->info.local_tunid == tunid) {
	    return ch;
	}
    }
    return NULL;
}

static struct net_device * l2tp_fp_rx_eth(struct fp_buf *fpb,
                                          struct l2tp_channel *ch,
                                          unsigned head_len, unsigned len) {
    struct iphdr *iph;
    struct fp_dev_pcpu_stats *stats;
    int is_ip;

    if (!(ch->dev->flags & IFF_UP)) goto slow_path_unlock;

    if (ch->info.have_l2specific_layer) head_len += 4; // 32 bits L2 default sublayer
    if (len < head_len) goto slow_path_unlock;

    iph = (struct iphdr *)(fpb_data(fpb) + head_len + ETH_HLEN);
    is_ip = 4 == iph->version;
    if (is_ip && is_tcp_syn((u8 *)iph, len - head_len)) goto slow_path_unlock;

    iph = (struct iphdr *) (fpb_data(fpb) + ETH_HLEN);
    if (ch->info.remote_addr != iph->saddr) goto slow_path_unlock;

    fpb_pull(fpb, head_len);

    stats = this_cpu_ptr(ch->dev->fp.stats);
    ++stats->fp_rx_packet;
    stats->fp_rx_byte += fpb_len(fpb) - ETH_HLEN;

    ch->stats->last_recv = jiffies;

    fast_path_rx_noinvalidate_nostats(ch->dev, fpb);
    return FAST_PATH_CONSUMED;

slow_path_unlock:
    rcu_read_unlock();
    return NULL;
}

static unsigned short *l2tp_fp_parse_udp_hdr(struct fp_buf *fpb, unsigned short *hdr,
                                             unsigned *head_len, unsigned *tunid,
                                             unsigned *sesid) {
    unsigned ctrl, ver;
    unsigned len = fpb_len(fpb);

    ctrl = ntohs(*hdr++);

    if (ctrl & L2TP_TYPE) {
        goto slow_path;
    }

    ver = ctrl & L2TP_VERSION_MASK;
    if (ver != L2TP_VERSION2 && ver != L2TP_VERSION3) {
        goto slow_path;
    }

    if (L2TP_VERSION2 == ver) {
        if (ctrl & L2TP_LENGTH_PRESENT) {
            unsigned l = *hdr++;
            *head_len += 2;
            if (len < *head_len) {
                goto slow_path;
            }
            if (len < ETH_HLEN + sizeof(struct iphdr) + sizeof(struct udphdr) + l) {
                goto slow_path;
            }
        }

        *tunid = *hdr++;
        *sesid = *hdr++;

        if (ctrl & L2TP_SEQ_PRESENT) {
            hdr += 2;
            *head_len += 4;

            if (len < *head_len)
                goto slow_path;
        }
        if (ctrl & L2TP_OFFSET_PRESENT) {
            *head_len += 2 + ntohs(*hdr++);

            if (len < *head_len)
                goto slow_path;
        }
    }
    else {
        hdr++; // *reserved* field
        *head_len += 2;

        memcpy(sesid, hdr, sizeof(u32));
        hdr += 2;

        *tunid = 0;
    }

    hdr = (unsigned short *) (fpb_data(fpb) + *head_len - 4);

    return hdr;

slow_path:
    return NULL;
}

static struct net_device *l2tp_fp_rx(struct net_device *dev, struct fp_buf *fpb) {
	struct fp_dev_pcpu_stats *stats;
	unsigned len = fpb_len(fpb);
	unsigned head_len;
	struct ethhdr *eth;
	struct ethhdr *old_eth;
	struct iphdr *iph;
	struct udphdr *udph;
	unsigned short *hdr;
	unsigned tunid;
	unsigned sesid;
	unsigned proto;
	unsigned ethertype = 0;
	struct l2tp_channel *ch = NULL;
	struct channel *pch;
	struct ppp *ppp;
	int npi, udp_mode;
	const int ppp_sz = 4; // 0xff03 + ppp protocol ID

	head_len = ETH_HLEN + sizeof(struct iphdr);
	if (len < head_len) goto slow_path;

	iph = (struct iphdr *) (fpb_data(fpb) + ETH_HLEN);

	udp_mode = L2TPV3_PROTO != iph->protocol;
	if (udp_mode) {
	    head_len = ETH_HLEN + sizeof(struct iphdr) + sizeof(struct udphdr) + L2TP_MIN_FRAME_LEN + 4;
	    if (len < head_len) goto slow_path;

	    udph = (struct udphdr *) &iph[1];
	    if (udph->dest != inet_sk(l2tp_udp4_socket->sk)->inet_sport)
	        goto slow_path;
	}

	hdr = udp_mode ? (unsigned short *) &udph[1] : (unsigned short *) &iph[1];

	if (udp_mode) {
	    hdr = l2tp_fp_parse_udp_hdr(fpb, hdr, &head_len, &tunid, &sesid);
	    if (!hdr) goto slow_path;
	}
	else { // l2tpv3_ip mode
	    head_len = ETH_HLEN + sizeof(struct iphdr) + sizeof(sesid) + ppp_sz;
	    if (len < head_len) goto slow_path;

	    sesid = ((unsigned*)hdr)[0];
	    if (0 == sesid) goto slow_path; // this is control channel message
	    hdr += 2;
	    tunid = 0; // in v3 data sessions are identified only by sess ID
	}

        rcu_read_lock();
        ch = find_session(tunid, sesid);
        if (!ch || !ch->info.allow_fast_path) goto slow_path_unlock;

        const int isv3 = 0 == tunid;
        if (isv3 && ch->info.lcookie_sz) {
            const unsigned int cookie_sz = ch->info.lcookie_sz;
            head_len += cookie_sz;
            if (len < head_len) goto slow_path_unlock;
            if (memcmp(hdr, ch->info.local_cookie, cookie_sz)) {
                goto slow_path_unlock;
            }
            hdr += cookie_sz / sizeof(hdr[0]);
        }

        if (PW_ETH == ch->info.wire_type)
            return l2tp_fp_rx_eth(fpb, ch, head_len - ppp_sz, len);

        if (get_unaligned(hdr) == __constant_htons(0xff03)) {
            ++hdr;
        } else {
            head_len -= 2;
        }

	proto = get_unaligned(hdr);

	if (proto == __constant_htons(PPP_MP)) {
		head_len += 6;
		if (len < head_len)
			goto slow_path_unlock;

		/* is it a framgment */
		if (*(unsigned char *) &hdr[1] != 0xc0)
			goto slow_path_unlock;

		proto = get_unaligned(&hdr[3]);
		/* has it protocol compression */
		if (proto & __constant_htons(0x100))
			goto slow_path_unlock;
	}

	npi = proto_to_npindex(proto, &ethertype);
	if (npi == -1) goto slow_path_unlock;

	if (ethertype == __constant_htons(ETH_P_IP) &&
	    is_tcp_syn(fpb_data(fpb) + head_len, len - head_len))
		goto slow_path_unlock;

	if (ch->info.remote_addr != iph->saddr)
		goto slow_path_unlock;

	if (udp_mode && (ch->info.remote_port != udph->source))
		goto slow_path_unlock;

	pch = rcu_dereference(ch->chan.ppp);
	if (!pch) goto slow_path_unlock;

	ppp = rcu_dereference(pch->ppp);

	if (!ppp || ppp->closing)
		goto slow_path_unlock;

	if (ppp->rstate & (SC_DECOMP_RUN | SC_MP_XSHORTSEQ))
		goto slow_path_unlock;

	if (!(ppp->dev->flags & IFF_UP) || ppp->npmode[npi] != NPMODE_PASS)
		goto slow_path_unlock;

	old_eth = (struct ethhdr *) fpb_data(fpb);
	fpb_pull(fpb, head_len - ETH_HLEN);
	eth = (struct ethhdr *) fpb_data(fpb);
	memcpy(eth->h_source, old_eth->h_source, ETH_ALEN);
	memcpy(eth->h_dest, ppp->dev->dev_addr, ETH_ALEN);
	eth->h_proto = ethertype;

	stats = this_cpu_ptr(ppp->dev->fp.stats);
	++stats->fp_rx_packet;
	stats->fp_rx_byte += fpb_len(fpb) - ETH_HLEN;
	ppp->last_recv = jiffies;

	fast_path_rx_noinvalidate_nostats(ppp->dev, fpb);
	return FAST_PATH_CONSUMED;

  slow_path_unlock:
	rcu_read_unlock();
  slow_path:
	return NULL;
}

static int l2tp_fp_fast_eth_xmit(struct net_device *dev, struct fp_buf *fpb) {
    struct l2tp_channel *ch;
    struct rt_cache *rt;
    struct fp_dev_pcpu_stats *stats;
    struct iphdr *iph;
    u8 *hdr;
    u16 *hdr16;
    struct udphdr *udph;
    unsigned udp_len;
    int udp_mode;
    unsigned dlen = fpb_len(fpb) - ETH_HLEN;
    struct ethhdr *eth = (struct ethhdr *) fpb_data(fpb);
    unsigned head_len = sizeof(struct ethhdr) + sizeof(struct iphdr) + 16; // 4 (sessid) + 8 (cookie len) + 4 (L2 sublayer)

    if (eth->h_proto == __constant_htons(ETH_P_IP)
            && is_tcp_syn((unsigned char*) &eth[1], dlen)) goto slow_path;

    if (unlikely(fpb_headroom(fpb) < head_len)) goto slow_path;

    ch = l2tpeth_priv(dev);
    if (!ch || !atomic_read(&ch->linked)) goto slow_path;

    if (!ch->info.allow_fast_path) goto slow_path;

    rt = lookup_rt(ch, dev);
    if (!rt || !rt->fast_path) goto slow_path;

    if (!ch->info.have_l2specific_layer) head_len -= 4;
    head_len -= (8 - ch->info.rcookie_sz);

    udp_mode = L2TPv3_UDP == ch->info.l2tp_ver;
    if (udp_mode) {
        head_len += sizeof(struct udphdr) + 4; // version & reserved fields
        if (unlikely(fpb_headroom(fpb) < head_len)) goto slow_path;
    }

    stats = this_cpu_ptr(dev->fp.stats);
    ++stats->fp_tx_packet;
    stats->fp_tx_byte += dlen;
    ch->stats->last_xmit = jiffies;

    fpb_push(fpb, head_len);

    eth = (struct ethhdr *) fpb_data(fpb);
    eth->h_proto = __constant_htons(ETH_P_IP);
    memcpy(eth->h_source, rt->rt->dst.dev->dev_addr, ETH_ALEN);
    memcpy(eth->h_dest, rt->neigh_addr, ETH_ALEN);

    iph = (struct iphdr *) &eth[1];
    iph->version = 4;
    iph->ihl = 5;
    iph->tos = 0;
    iph->tot_len = htons(fpb_len(fpb) - ETH_HLEN);
    iph->frag_off = 0;
    iph->ttl = 0x40;
    iph->protocol = udp_mode ? IPPROTO_UDP : L2TPV3_PROTO;
    iph->saddr = rt->saddr;
    iph->daddr = rt->daddr;
    iph->id = ch->peer_handle ? gre_peer_handle_next_identity(ch->peer_handle) : 0;
    ip_send_check(iph);

    hdr = (u8 *) &iph[1];

    if (udp_mode) {
        udph = (struct udphdr *) &iph[1];
        udp_len = fpb_len(fpb) - sizeof(struct iphdr) - ETH_HLEN;
        udph->source = inet_sk(udp_sock(ch))->inet_sport;
        udph->dest = ch->info.remote_port;
        udph->len = htons(udp_len);
        udph->check = 0;

        hdr16 = (u16 *) (udph + 1);
        put_unaligned(__constant_htons(L2TP_VERSION3), &hdr16[0]);
        put_unaligned(0, &hdr16[1]); // *reserved* field
        hdr16 += 2;
        put_unaligned(ch->info.remote_sesid, (u32 *)hdr16);
        hdr16 += 2;
        hdr = (u8 *)hdr16;
    }
    else {
        put_unaligned(ch->info.remote_sesid, (unsigned *)hdr);
        hdr += sizeof(ch->info.remote_sesid);
    }

    if (ch->info.rcookie_sz) {
        memcpy(hdr, ch->info.remote_cookie, ch->info.rcookie_sz);
        hdr += ch->info.rcookie_sz;
    }

    if (ch->info.have_l2specific_layer) {
        put_unaligned(0, (unsigned *)hdr);
    }

    return fast_path_tx(rt->rt->dst.dev, fpb);

slow_path:
    return fast_path_tx_slow(dev, fpb);
}

static void l2tp_fp_adj_head_len(unsigned *head_len, const struct l2tp_channel *ch) {
    if (L2TPv3_IP == ch->info.l2tp_ver) {
        *head_len -= sizeof(struct udphdr);
        *head_len -= 2;
        *head_len += ch->info.rcookie_sz;
    }
    else if (L2TPv3_UDP == ch->info.l2tp_ver) {
        *head_len += 2; // v3 udp header *reserved* field
        *head_len += ch->info.rcookie_sz;
    }
}

static unsigned short *
l2tp_fp_build_l2tp_udp_hdr(unsigned short *hdr, const struct l2tp_channel *ch) {
    if (L2TPv2 == ch->info.l2tp_ver) {
        put_unaligned(__constant_htons(L2TP_VERSION2), &hdr[0]);
        put_unaligned(ch->info.remote_tunid, &hdr[1]);
        put_unaligned(ch->info.remote_sesid, &hdr[2]);
        hdr += 3;
        return hdr;
    }

    // udp v3
    put_unaligned(__constant_htons(L2TP_VERSION3), &hdr[0]);
    put_unaligned(0, &hdr[1]); // *reserved* field
    hdr += 2;
    put_unaligned(ch->info.remote_sesid, (unsigned *)hdr);
    hdr += 2;
    if (ch->info.rcookie_sz) {
        memcpy(hdr, ch->info.remote_cookie, ch->info.rcookie_sz);
        hdr += (ch->info.rcookie_sz / sizeof(hdr[0]));
    }
    return hdr;
}

static int l2tp_fp_fast_ppp_xmit(struct net_device *pppdev, struct fp_buf *fpb) {
	struct fp_dev_pcpu_stats *stats;
	struct ppp *ppp = netdev_priv(pppdev);
	struct channel *pch;
	struct ppp_channel *chan = NULL;
	struct l2tp_channel *ch;
	struct rt_cache *rt;
	struct ethhdr *eth = (struct ethhdr *) fpb_data(fpb);
	struct iphdr *iph;
	struct udphdr *udph;
	unsigned short *hdr;
	u8 *hdr_v3;
	unsigned dlen = fpb_len(fpb) - ETH_HLEN;
	unsigned head_len = sizeof(struct iphdr) + sizeof(struct udphdr) + 10;
	unsigned udp_len;
	unsigned nxchan = 0;
	int npi;
	unsigned proto;
	u8 iphproto;

	if (unlikely(ppp->flags & SC_LOOP_TRAFFIC))
		goto slow_path;
	if (unlikely(ppp->xstate & SC_COMP_RUN))
		goto slow_path;

	if (eth->h_proto == __constant_htons(ETH_P_IP) &&
	    is_tcp_syn((unsigned char *) &eth[1], dlen))
		goto slow_path;

	if (ppp->flags & SC_MULTILINK) {
               if (ppp->flags & SC_MP_XSHORTSEQ)
                       goto slow_path;

               if (fpb_len(fpb) - ETH_HLEN > ppp->chan_min_mtu - 6)
                       goto slow_path;

	       if (ppp->n_channels == 0)
			goto slow_path;

               nxchan = atomic_add_return(1, (atomic_t *) &ppp->nxchan) - 1;
               nxchan %= ppp->n_channels;

               head_len += 6;
       }
	
	if (unlikely(fpb_headroom(fpb) < head_len))
		goto slow_path;

	npi = ethertype_to_npindex(eth->h_proto, &proto);
	if (npi == -1 || ppp->npmode[npi] != NPMODE_PASS)
		goto slow_path;

	rcu_read_lock();
	list_for_each_entry_rcu(pch, &ppp->channels, clist) {
		if (nxchan > 0) {
			--nxchan;
			continue;
		}
		chan = rcu_dereference(pch->chan);
		break;
	}
	rcu_read_unlock();

	if (!chan)
		goto slow_path;

	ch = chan->private;
	if (!atomic_read(&ch->linked))
		goto slow_path;

	if (!ch->info.allow_fast_path)
		goto slow_path;

	rt = lookup_rt(ch, pppdev);
	if (!rt || !rt->fast_path)
		goto slow_path;

	stats = this_cpu_ptr(pppdev->fp.stats);
	++stats->fp_tx_packet;
	stats->fp_tx_byte += dlen;
	ppp->last_xmit = jiffies;

	l2tp_fp_adj_head_len(&head_len, ch);
	fpb_push(fpb, head_len);

	eth = (struct ethhdr *) fpb_data(fpb);
	eth->h_proto = __constant_htons(ETH_P_IP);
	memcpy(eth->h_source, rt->rt->dst.dev->dev_addr, ETH_ALEN);
	memcpy(eth->h_dest, rt->neigh_addr, ETH_ALEN);

	iphproto = L2TPv3_IP == ch->info.l2tp_ver ? L2TPV3_PROTO : IPPROTO_UDP;
	iph = (struct iphdr *) &eth[1];
	iph->version = 4;
	iph->ihl = 5;
	iph->tos = 0;
	iph->tot_len = htons(fpb_len(fpb) - ETH_HLEN);
	iph->frag_off = 0;
	iph->ttl = 0x40;
	iph->protocol = iphproto;
	iph->saddr = rt->saddr;
	iph->daddr = rt->daddr;
	iph->id = ch->peer_handle ?
	    gre_peer_handle_next_identity(ch->peer_handle) : 0;
	ip_send_check(iph);
	
	if (IPPROTO_UDP == iphproto) {
	    udph = (struct udphdr *) &iph[1];
	    udp_len = fpb_len(fpb) - sizeof(struct iphdr) - ETH_HLEN;
	    udph->source = inet_sk(udp_sock(ch))->inet_sport;
	    udph->dest = ch->info.remote_port;
	    udph->len = htons(udp_len);
	    udph->check = 0;

            hdr = (unsigned short *) (udph + 1);
            hdr = l2tp_fp_build_l2tp_udp_hdr(hdr, ch);
	}
	else { // L2TPv3 over IP
	    hdr_v3 = (u8 *) &iph[1];

	    put_unaligned(ch->info.remote_sesid, (u32 *)hdr_v3);
	    hdr_v3 += sizeof(ch->info.remote_sesid);

	    if (ch->info.rcookie_sz) {
	        memcpy(hdr_v3, ch->info.remote_cookie, ch->info.rcookie_sz);
	        hdr_v3 += ch->info.rcookie_sz;
	    }
	    hdr = (unsigned short *)hdr_v3;
	}

        put_unaligned(__constant_htons(0xff03), hdr);
        hdr++;

	if (ppp->flags & SC_MULTILINK) {
		unsigned seq =  atomic_add_return(1, (atomic_t *) &ppp->nxseq) - 1;
		put_unaligned(__constant_htons(PPP_MP), &hdr[0]);
		put_unaligned(htons(((seq & 0xff0000) >> 16) | 0xc000), &hdr[1]);
		put_unaligned(htons((seq & 0xffff)), &hdr[2]);
		hdr += 3;
	}	
	put_unaligned(proto, &hdr[0]);

	if (IPPROTO_UDP == iphproto) {
            udph->check = csum_tcpudp_magic(
                iph->saddr, iph->daddr, udp_len, IPPROTO_UDP,
                csum_partial(udph, udp_len, 0));
	}

	return fast_path_tx(rt->rt->dst.dev, fpb);

  slow_path:
	return fast_path_tx_slow(pppdev, fpb);
}

void l2tp_fp_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *s) {
	struct ppp *ppp = netdev_priv(dev);

	fast_path_get_stats64(dev, s);

	s->rx_packets += ppp->stats64.rx_packets;
	s->tx_packets += ppp->stats64.tx_packets;
	s->rx_bytes += ppp->stats64.rx_bytes;
	s->tx_bytes += ppp->stats64.tx_bytes;
	s->tx_errors = dev->stats.tx_errors;
	s->rx_errors = dev->stats.rx_errors;
	s->tx_dropped  = dev->stats.tx_dropped;
	s->rx_dropped = dev->stats.rx_dropped;
	s->rx_length_errors = dev->stats.rx_length_errors;
}

static int l2tp_fp_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd) {
	if (cmd == SIOCGPPPSTATS) {
		struct ppp *ppp = netdev_priv(dev);
		void __user *addr = (void __user *) ifr->ifr_ifru.ifru_data;
		struct ppp_stats stats;
		
		fast_path_update_stats(dev);
		ppp_get_stats(ppp, &stats);
		stats.p.ppp_ipackets += dev->fp.fp_rx_packet;
		stats.p.ppp_ibytes += dev->fp.fp_rx_byte;
		stats.p.ppp_opackets += dev->fp.fp_tx_packet;
		stats.p.ppp_obytes += dev->fp.fp_tx_byte;
		
		if (copy_to_user(addr, &stats, sizeof(stats)))
			return -EFAULT;
		return 0;
	}
	return ppp_net_ioctl(dev, ifr, cmd);
}

static atomic_t l2tp_unit_count = ATOMIC_INIT(0);

static void l2tp_ppp_newdev(struct net_device *dev) {
	static struct net_device_ops ops;

	if (dev->netdev_ops->ndo_fast_path_xmit)
		return;
	
	if (!ops.ndo_fast_path_xmit) {
		memcpy(&ops, dev->netdev_ops, sizeof(ops));
		ops.ndo_fast_path_xmit = l2tp_fp_fast_ppp_xmit;
		ops.ndo_do_ioctl = l2tp_fp_ioctl;
		ops.ndo_get_stats64 = l2tp_fp_get_stats64;
	}
	dev->netdev_ops = &ops;

	if (atomic_inc_return(&l2tp_unit_count) == 1) {
		printk("registering l2tp fp\n");
		fast_path_register_ipproto(IPPROTO_UDP, l2tp_fp_rx);
		fast_path_register_ipproto(L2TPV3_PROTO, l2tp_fp_rx);
	}
}

static void l2tp_eth_newdev(struct net_device *dev) {
    static struct net_device_ops ops;

    if (dev->netdev_ops->ndo_fast_path_xmit) return;

    if (!ops.ndo_fast_path_xmit) {
        memcpy(&ops, dev->netdev_ops, sizeof(ops));
        ops.ndo_fast_path_xmit = l2tp_fp_fast_eth_xmit;
    }
    dev->netdev_ops = &ops;

    if (atomic_inc_return(&l2tp_unit_count) == 1) {
        printk("registering l2tp fp\n");
        fast_path_register_ipproto(IPPROTO_UDP, l2tp_fp_rx);
        fast_path_register_ipproto(L2TPV3_PROTO, l2tp_fp_rx);
    }
}

void l2tp_chan_attached(struct ppp_channel *chan) {
	struct channel *pch = chan->ppp;
	struct ppp *ppp;
	struct net_device *dev;
	
	if (!pch) return; /* channel is dying */
	ppp = pch->ppp;
	dev = ppp->dev;

	if (dev) l2tp_ppp_newdev(dev);
}

int l2tp_fp_device_event(struct notifier_block *this,
			 unsigned long event, void *ptr) {
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct ppp *ppp;
	struct channel *pch;
	struct ppp_channel *chan = NULL;

	if (event == NETDEV_DOWN &&
	    dev->netdev_ops->ndo_fast_path_xmit == l2tp_fp_fast_ppp_xmit) {
		if (atomic_dec_and_test(&l2tp_unit_count)) {
			printk("unregistering l2tp fp\n");
			fast_path_unregister_ipproto(IPPROTO_UDP);
			fast_path_unregister_ipproto(L2TPV3_PROTO);
		}
		return NOTIFY_DONE;
	}
	
	if (event != NETDEV_PRE_UP)
		return NOTIFY_DONE;

	if (dev->netdev_ops->ndo_start_xmit == l2tpeth_dev_xmit) {
	    l2tp_eth_newdev(dev);
	    return NOTIFY_DONE;
	}

	if (dev->netdev_ops->ndo_start_xmit != ppp_start_xmit)
		return NOTIFY_DONE;

	ppp = netdev_priv(dev);

	rcu_read_lock();
	list_for_each_entry_rcu(pch, &ppp->channels, clist) {
		chan = rcu_dereference(pch->chan);
		break;
	}
	rcu_read_unlock();
	
	if (!chan || chan->ops->start_xmit != l2tp_xmit)
		return NOTIFY_DONE;

	l2tp_ppp_newdev(dev);
	return NOTIFY_DONE;
}

static struct notifier_block l2tp_fp_notifier = {
	.notifier_call = l2tp_fp_device_event,
	.priority = 1100,
};

int l2tp_fp_init(void) {
	register_netdevice_notifier(&l2tp_fp_notifier);
	return 0;
}

void l2tp_fp_exit(void) {
	unregister_netdevice_notifier(&l2tp_fp_notifier);
}
