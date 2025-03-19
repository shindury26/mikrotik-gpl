#ifndef _L2TP_CHAN_H_
#define _L2TP_CHAN_H_

#include <linux/ppp_channel.h>
#include <linux/list.h>
#include <linux/skbuff.h>
#include <linux/gre.h>
#include <linux/ip.h>
#include <linux/in6.h>

#include "l2tp_ioctl.h"

#define L2TP_HASHSIZE 4096
extern struct hlist_head ses_table[L2TP_HASHSIZE];

#define DEFAULT_L2_SUBLAYER_SZ 4

struct rt_cache {
    struct rcu_head rcu_head;
    struct rtable *rt;
    unsigned long last_lookup;
    unsigned saddr;
    unsigned daddr;
    unsigned char neigh_addr[6]; 
    bool fast_path;

    struct dst_entry *dst; // ipv6
    struct flowi6 fl6; // ipv6
};

struct l2tp_dev_stats {
    atomic_long_t tx_bytes;
    atomic_long_t tx_packets;
    atomic_long_t tx_dropped;
    atomic_long_t rx_bytes;
    atomic_long_t rx_packets;
    atomic_long_t rx_errors;

    unsigned last_xmit;
    unsigned last_recv;
};

struct l2tp_channel;

/*L2TP control message header handlers*/
typedef void (*l2tp_xmit_hdr_prep)(struct sk_buff *,
                                   struct l2tp_channel *, void *iph);
void l2tp_xmit_ip_header_prep(struct sk_buff *skb,
                              struct l2tp_channel *ch, void *iph);
void l2tp_xmit_udp_header_prep(struct sk_buff *skb,
                               struct l2tp_channel *ch, void *iph);
void l2tp_xmit_udp3_header_prep(struct sk_buff *skb,
                                struct l2tp_channel *ch, void *iph);

/*L2TP incoming data handlers*/
typedef void (*l2tp_receive_handler)(struct l2tp_channel *, struct sk_buff *);

void l2tp_receive_ppp(struct l2tp_channel *ch, struct sk_buff *skb);

void l2tp_receive_ether(struct l2tp_channel *ch, struct sk_buff *skb);

struct l2tp_channel {
    atomic_t linked;
    struct ppp_channel chan;
    struct hlist_node next;
    struct l2tp_info info;
    struct rcu_head rcu;

    struct gre_peer_handle *peer_handle;
    struct rt_cache *rt_cache;
    struct net_device *dev;
    struct l2tp_dev_stats *stats;

    unsigned xmit_hdr_size;
    l2tp_xmit_hdr_prep    hdr_prep_fn;
    l2tp_receive_handler  receive_fn;

    struct flowi6 fl6; // ipv6
};

int l2tp_add_channel(struct l2tp_add *);
int l2tp_del_channel(int);
void l2tp_del_all(void);
int l2tp_add_ethchan(struct l2tp_add *a);
int l2tp_eth_getidle(struct l2tp_idle_info *info);

struct rt_cache *lookup_rt(struct l2tp_channel *ch, struct net_device *dev);

void l2tp_receive(unsigned sesid, unsigned tunid, struct sk_buff *);

static inline unsigned ses_hash(unsigned sesid, unsigned tunid) {
    return (sesid + tunid) % L2TP_HASHSIZE;
}

struct sock *udp_sock(const struct l2tp_channel *ch);
inline int l2tp_ch_isip4(const struct l2tp_channel* ch);
#endif
