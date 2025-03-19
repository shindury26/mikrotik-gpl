#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/udp.h>
#include <linux/file.h>
#include <linux/random.h>
#include <linux/net.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <net/sock.h>
#include <net/udp.h>
#include <net/ip.h>
#include <net/route.h>
#include <net/ip6_route.h>
#include <net/ip6_checksum.h>
#include <linux/netfilter_ipv4.h>
#include <linux/in_route.h>

#include <asm/unaligned.h>
#include <asm/div64.h>
#include "btest.h"

#define BTEST_MASK_LEN 4
#define BTEST_BIT_COUNT 32

MODULE_LICENSE("GPL");


#define STAT_WORK 0
#define STAT_TIMER 1
#define STAT_XMIT 2
#define STAT_XMIT_FAIL 3
#define STAT_MAX 4
//#define STATS
#include <linux/stats.h>
#ifdef STATS

static struct stats_descr stats_btest = {
    .name = "btest",
    .interval = HZ,
    .elements = {
	{
	    .name = "work",
	},
	{
	    .name = "timer",
//	    .no_print = true,
//	    .no_reset = true,
	},
	{
	    .name = "xmit",
	},
	{
	    .name = "xmit_fail",
	},
    }
};
#endif

static unsigned btest_cpu;
struct btest_state {
    bool started;
    struct btest_params params;
    struct btest_info info;
    unsigned cpu;

    void (*old_data_ready)(struct sock *sk);

    // lost packet stats accounting
    unsigned first;
    unsigned unable;
    unsigned maxseq;
    unsigned lastseq;
    unsigned mask[BTEST_MASK_LEN];

    // transmit
    unsigned packet_nsecs;
    unsigned seq;
    u64 next_tx;

    struct socket *socket;
    struct sock *sk;
    struct work_struct work;
    struct hrtimer timer;
    spinlock_t lock;

#ifdef STATS
    struct stats_descr stats;
#endif
};
static atomic_t num_started;

static struct dst_entry *(*btest_ip6_route_output_flags)(struct net *net,
	const struct sock *sk, struct flowi6 *fl6, int flags);

static void btest_account(struct btest_state *state, unsigned seq) {
    unsigned bit;
    if (state->first) {
	state->first = 0;
	if (seq > 10000) state->unable = 1;
	state->maxseq = seq;
	return;
    }
    if (state->unable) return;
//    printk("got seq: %u\n", seq);
    if (seq > state->maxseq) {
	unsigned c, i, j;
	state->lastseq = seq;
	c = (seq - state->maxseq) / BTEST_BIT_COUNT;
	if ((seq - state->maxseq) % BTEST_BIT_COUNT) ++c;
	state->maxseq += c * BTEST_BIT_COUNT;
//	printk("shift by: %u\n", c * BTEST_BIT_COUNT);
	if (c > BTEST_MASK_LEN) {
	    state->info.lost += (c - BTEST_MASK_LEN) * BTEST_BIT_COUNT;
	    c = BTEST_MASK_LEN;
	}

	for (i = 0; i < c; ++i) {
	    for (j = 0; j < BTEST_BIT_COUNT; ++j) {
		if (!(state->mask[i] & (1 << j))) {
		    ++state->info.lost;
		}
	    }
	}
	for (i = 0; i < BTEST_MASK_LEN; ++i) {
	    if (i < BTEST_MASK_LEN - c) state->mask[i] = state->mask[i + c];
	    else state->mask[i] = 0;
	}
    }
    bit = state->maxseq - seq;
//    printk("in range: %u\n", bit);
    if (bit < BTEST_MASK_LEN * BTEST_BIT_COUNT) {
	unsigned c = BTEST_MASK_LEN - 1 - bit / BTEST_BIT_COUNT;
	bit %= BTEST_BIT_COUNT;

	if (state->mask[c] & (1 << bit)) {
	    ++state->info.duplicate;
	}
	else {
	    state->mask[c] |= (1 << bit);
	    if (seq < state->lastseq) {
		++state->info.outoforder;
	    }
	}
    }
}

static void btest_data_ready(struct sock *sk) {
    struct btest_state *state = (struct btest_state *) sk->sk_user_data;
    int err;
    struct sk_buff *skb;
    unsigned seq;
    int off;
    if (!state->started) {
	return;
    }
//    printk("btest_data_ready: %u\n", bytes);

    while ((skb = __skb_recv_udp(sk, 0, 1, &off, &err)) == NULL) {
	if (err == -EAGAIN) {
	    printk("btest: no data available?!");
	    return;
	}
	printk("btest: recvfrom() error %d\n", -err);
    }

    seq = ntohl(get_unaligned((unsigned *)(udp_hdr(skb) + 1)));
//    seq = ntohl(get_unaligned((unsigned *)(skb->data + sizeof(udp_hdr))));

    spin_lock_bh(&state->lock);
    btest_account(state, seq);
    state->info.received += skb->len;
    spin_unlock_bh(&state->lock);

    skb_free_datagram(sk, skb);
}

static unsigned x;
static void btest_random(void *buf, int len) {
    unsigned char *data = buf;
    int i;
    for (i = 0; i < len; ++i) {
	x += (x >> 26) | (x << 14);
	data[i] = x;
    }
}

static inline int do_ip_send(struct net *net, struct sock *sock, struct sk_buff *skb) {
    return dst_output(net, sock, skb);
//    return ip_send(skb);
}

static int btest_send_ipv4(struct sock *sk, unsigned seq, bool random, unsigned size) {
    struct inet_sock *inet = inet_sk(sk);
    int err = 0;
    struct rtable *rt = NULL;
    u8 tos = RT_TOS(inet->tos);
    unsigned headroom;
    struct sk_buff *skb;
    struct iphdr *iph;
    struct udphdr *uh;
    struct flowi4 fl4;

    rt = (struct rtable*)sk_dst_check(sk, 0);

    if (rt == NULL) {
	rt = ip_route_output_ports(&init_net, &fl4, NULL,
				   inet->inet_daddr, inet->inet_saddr,
				   inet->inet_dport, inet->inet_sport,
				   sk->sk_protocol,
				   RT_CONN_FLAGS(sk),
				   sk->sk_bound_dev_if);
	if (IS_ERR(rt)) return PTR_ERR(rt);

	err = -EACCES;
	if (rt->rt_flags & RTCF_BROADCAST && !sock_flag(sk, SOCK_BROADCAST)) {
	    goto out;
	}
	sk_dst_set(sk, dst_clone(&rt->dst));
    }

    headroom = LL_RESERVED_SPACE(rt->dst.dev);

    skb = alloc_skb(headroom + size + 31, GFP_KERNEL);
    if (!skb) {
	return -ENOMEM;
    }
    skb_reserve(skb, headroom);

    skb->priority = sk->sk_priority;
    skb_dst_set(skb, dst_clone(&rt->dst));

    skb_reset_network_header(skb);
    iph = (struct iphdr *)skb_put(skb, size);
    iph->version = 4;
    iph->ihl = sizeof(struct iphdr)>>2;
    iph->frag_off = 0;
    iph->ttl = 64;
    iph->protocol = sk->sk_protocol;
    iph->tos = tos;
    iph->daddr = inet->inet_daddr;
    iph->saddr = inet->inet_saddr;
    iph->check = 0;
    iph->tot_len = htons(size);
    ip_select_ident(&init_net, skb, NULL);
    ip_send_check(iph);

    skb_set_transport_header(skb, sizeof(struct iphdr));
    uh = udp_hdr(skb);
    uh->source = inet->inet_sport;
    uh->dest = inet->inet_dport;
    uh->len = htons(size - sizeof(struct iphdr));
    uh->check = 0;

    if (size >= 32) {
	put_unaligned(htonl(++seq), (unsigned *)(skb->data + 28));
    }
    if (random) {
	btest_random(skb->data + 32, (int)size - 32);
    }

    err = NF_HOOK(PF_INET, NF_INET_LOCAL_OUT, &init_net,
		  NULL, skb, NULL, rt->dst.dev, do_ip_send);

out:
    ip_rt_put(rt);
//    if (err < 0) printk("err: %d\n", err);
    return err;
}

static int btest_send_ipv6(struct sock *sk, unsigned seq, bool random, unsigned size) {
    struct inet_sock *inet = inet_sk(sk);
    int err = 0;
    unsigned headroom;
    struct sk_buff *skb;
    struct ipv6hdr *iph;
    struct udphdr *uh;
    struct dst_entry *dst = sk_dst_check(sk, 0);
    unsigned udpsize = size - sizeof(struct ipv6hdr);

    if (!dst) {
	struct flowi6 fl = {};
	fl.flowi6_oif = sk->sk_bound_dev_if;
	fl.daddr = sk->sk_v6_daddr;
	fl.saddr = inet->pinet6->saddr;
	fl.flowlabel = inet->pinet6->flow_label;
	fl.flowi6_proto = sk->sk_protocol;
	fl.fl6_sport = inet->inet_sport;
	fl.fl6_dport = inet->inet_dport;

	dst = btest_ip6_route_output_flags(&init_net, NULL, &fl, 0);
	if (dst->error) {
	    dst_release(dst);
	    return -EACCES;
	}
	dst = xfrm_lookup(&init_net, dst, flowi6_to_flowi(&fl), NULL, 0);
	if (IS_ERR(dst)) {
	    return -EACCES;
	}

	sk_dst_set(sk, dst_clone(dst));
    }

    headroom = (((dst->dev->hard_header_len + 31) & ~31));

    skb = alloc_skb(headroom + size + 31, GFP_KERNEL);
    if (!skb) {
	return -ENOMEM;
    }
    skb_reserve(skb, headroom);

    skb->priority = sk->sk_priority;
    skb_dst_set(skb, dst_clone(dst));
    skb->protocol = htons(ETH_P_IPV6);
    skb->dev = dst->dev;

    skb_reset_network_header(skb);
    iph = (struct ipv6hdr *)skb_put(skb, size);
    iph->version = 6;
    iph->priority = 0;
    iph->flow_lbl[0] = htonl(inet->pinet6->flow_label) >> 16;
    iph->flow_lbl[1] = htonl(inet->pinet6->flow_label) >> 8;
    iph->flow_lbl[2] = htonl(inet->pinet6->flow_label);
    iph->payload_len = htons(size - sizeof(struct ipv6hdr));
    iph->nexthdr = sk->sk_protocol;
    iph->hop_limit = 64;
    iph->saddr = inet->pinet6->saddr;
    iph->daddr = sk->sk_v6_daddr;

    skb_set_transport_header(skb, sizeof(struct ipv6hdr));
    uh = udp_hdr(skb);

    if (size >= 52) {
	put_unaligned(htonl(++seq), (unsigned *)(skb->data + 48));
    }
    if (random) {
	btest_random(skb->data + 52, (int)size - 52);
    }

    uh->source = inet->inet_sport;
    uh->dest = inet->inet_dport;
    uh->len = htons(udpsize);
    uh->check = 0;
    uh->check = csum_ipv6_magic(&iph->saddr, &iph->daddr,
	    udpsize, IPPROTO_UDP,
	    csum_partial(uh, udpsize, 0));

    err = NF_HOOK(PF_INET6, NF_INET_LOCAL_OUT, &init_net,
		  NULL, skb, NULL, dst->dev, do_ip_send);
    dst_release(dst);
//    if (err < 0) printk("err: %d\n", err);
    return err;
}


static int btest_send_packet(struct sock *sk, unsigned seq, bool random, unsigned size) {
    struct inet_sock *inet = inet_sk(sk);

    if (inet->pinet6) {
	if (inet->pinet6->saddr.in6_u.u6_addr32[0] ||
		inet->pinet6->saddr.in6_u.u6_addr32[1] ||
		inet->pinet6->saddr.in6_u.u6_addr32[2] != ntohl(0xffff)) {
	    return btest_send_ipv6(sk, seq, random, size);
	}
    }
    return btest_send_ipv4(sk, seq, random, size);
}

static inline unsigned btest_get_size(struct btest_state *state) {
    if (state->params.send_size_from != state->params.send_size_to) {
	unsigned rnd;
	get_random_bytes(&rnd, sizeof(rnd));
	return state->params.send_size_from +
	    (rnd % (state->params.send_size_to -
		    state->params.send_size_from));
    }
    else {
	return state->params.send_size_from;
   }
}

static void btest_adjust(struct btest_state *state) {
    unsigned size =
	(state->params.send_size_from + state->params.send_size_to) / 2;
    unsigned long long x = state->params.send_rate;
    do_div(x, size);
    if (!x) {
	x = 1;
    }
    state->packet_nsecs = 1000000 / (unsigned)x * 1000;
    state->next_tx = ktime_get_ns();
}

static void btest_tx_task(struct work_struct *w) {
    struct btest_state *state = container_of(w, struct btest_state, work);
    u64 start;

    stats_inc(&state->stats, STAT_WORK);
    start = ktime_get_ns();

    spin_lock_bh(&state->lock);
    if (!state->started) {
	spin_unlock_bh(&state->lock);
	return;
    }
    u64 curr;
    u64 dur;
    unsigned c = 0;

    struct sock *sk = state->sk;
    bool random = state->params.send_random;
    while (start > state->next_tx) {
	unsigned size = btest_get_size(state);
	++state->seq;
	unsigned seq = state->seq;
	stats_inc(&state->stats, STAT_XMIT);
	spin_unlock_bh(&state->lock);
	if (btest_send_packet(sk, seq, random, size)) {
	    stats_inc(&state->stats, STAT_XMIT_FAIL);
	    spin_lock_bh(&state->lock);
	    break;
	}

	++c;
	if (c == 10) {
	    c = 0;
	    curr = ktime_get_ns();
	    dur = curr - start;
	    if (dur > 2000000) {
		spin_lock_bh(&state->lock);
		break;
	    }
	}

	spin_lock_bh(&state->lock);
	state->next_tx += state->packet_nsecs;
    }
    unsigned sl = state->packet_nsecs;

    hrtimer_start(&state->timer, ktime_set(0, sl), HRTIMER_MODE_REL);
    spin_unlock_bh(&state->lock);
}

static enum hrtimer_restart btest_hrtimer(struct hrtimer *t) {
    struct btest_state *state = container_of(t, struct btest_state, timer);
    stats_inc(&state->stats, STAT_TIMER);
    schedule_work_on(state->cpu, &state->work);
    return HRTIMER_NORESTART;
}

static int btest_start(struct file *file) {
    struct btest_state *state = (struct btest_state *)file->private_data;
    struct socket *socket;
    int err;

    spin_lock_bh(&state->lock);
#ifdef STATS
    memcpy(&state->stats, &stats_btest, sizeof(stats_btest));
    sprintf(state->stats.name, "btest%u", state->params.fd);
#endif

    if (state->started) {
	spin_unlock_bh(&state->lock);
	return -EBUSY;
    }
//    printk("btest: start file %p\n", file);

    socket = sockfd_lookup(state->params.fd, &err);
    if (!socket) {
	spin_unlock_bh(&state->lock);
	return err;
    }
    state->socket = socket;
    state->sk = socket->sk;
    sock_hold(state->sk);
    atomic_inc(&num_started);

    if (state->params.flags & BTEST_RECEIVE) {
	state->old_data_ready = socket->sk->sk_data_ready;
	socket->sk->sk_user_data = state;
	mb();
	socket->sk->sk_data_ready = btest_data_ready;
//	socket->file->private_data = state;
//	printk("btest: recv file %p\n", socket->file);
    }

    if (state->params.flags & BTEST_SEND) {
	btest_adjust(state);
//	printk("btest: size:%u-%u, bytes/sec:%llu, random:%u, packet_nsecs:%u\n",
//		state->params.send_size_from, state->params.send_size_to,
//		state->params.send_rate,
//		state->params.send_random,
//		state->packet_nsecs);

	state->cpu = btest_cpu;
	btest_cpu = (btest_cpu + 1) % num_online_cpus();
	schedule_work_on(state->cpu, &state->work);
    }
    state->started = true;
    spin_unlock_bh(&state->lock);
    return 0;
}

static int btest_open(struct inode *inode, struct file *file) {
    struct btest_state *state;
//    printk("btest: open\n");

    if (!capable(CAP_SYS_ADMIN)) return -EBUSY;

    state = kmalloc(sizeof(struct btest_state), GFP_KERNEL);
    if (!state) return -ENOMEM;
    memset(state, 0, sizeof(struct btest_state));

    state->first = 1;
    memset(state->mask, 0xFF, sizeof(unsigned) * BTEST_MASK_LEN);

    INIT_WORK(&state->work, btest_tx_task);

    hrtimer_init(&state->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    state->timer.function = btest_hrtimer;
    spin_lock_init(&state->lock);

    file->private_data = state;
    return 0;
}

static int btest_release(struct inode *inode, struct file *file) {
    struct btest_state *state = (struct btest_state *)file->private_data;
//    printk("btest: release: %p\n", state);

    if (state->started) {
	state->started = false;
	cancel_work_sync(&state->work);
	spin_lock_bh(&state->lock);
	hrtimer_cancel(&state->timer);

	if (state->old_data_ready) {
	    state->sk->sk_data_ready = state->old_data_ready;
	}

	sockfd_put(state->socket);
	sock_put(state->sk);
	spin_unlock_bh(&state->lock);
	atomic_dec(&num_started);
    }
    kfree(state);
    if (!atomic_read(&num_started)) {
	synchronize_net();
    }
    return 0;
}

static long btest_ioctl(struct file *file,
			unsigned int cmd, unsigned long data) {
    struct btest_state *state = (struct btest_state *)file->private_data;

//    printk("btest_ioctl: %u %lu\n", cmd, data);
    switch (cmd) {
    case BTEST_IOCTL_START:
	if (copy_from_user(&state->params, (void *)data,
			   sizeof(struct btest_params))) {
	    return -EINVAL;
	}
	return btest_start(file);
    case BTEST_IOCTL_GET:
//	printk("state->info.lost: %u\n", state->info.lost);
	spin_lock_bh(&state->lock);
	if (copy_to_user((void *)data, &state->info,
			 sizeof(struct btest_info))) {
	    spin_unlock_bh(&state->lock);
	    return -EINVAL;
	}
	memset(&state->info, 0, sizeof(struct btest_info));
	spin_unlock_bh(&state->lock);
	return 0;
    case BTEST_IOCTL_ADJUST:
	if (copy_from_user(&state->params.send_rate, (void *)data,
			   sizeof(unsigned long long))) {
	    return -EINVAL;
	}
	spin_lock_bh(&state->lock);
	btest_adjust(state);
	spin_unlock_bh(&state->lock);
//	printk("btest: adjust rate: %llu packet_nsecs:%u\n",
//		state->params.send_rate, state->packet_nsecs);
	return 0;
    }
    return -ENOTTY;
}

static struct file_operations btest_fops = {
    owner: THIS_MODULE,
    open: btest_open,
    release: btest_release,
    unlocked_ioctl: btest_ioctl,
    compat_ioctl: btest_ioctl,
};

static struct miscdevice btest_miscdev = {
    minor: MISC_DYNAMIC_MINOR,
    name: "btest",
    fops: &btest_fops,
};

int init_module(void) {
    int err;
    btest_ip6_route_output_flags = symbol_get(ip6_route_output_flags);

    err = misc_register(&btest_miscdev);
    if (err) {
	printk("btest: failed to register char device\n");
	return err;
    }

    get_random_bytes(&x, sizeof(x));
    atomic_set(&num_started, 0);
    return 0;
}

void cleanup_module(void) {
    misc_deregister(&btest_miscdev);

    if (btest_ip6_route_output_flags) {
	symbol_put(ip6_route_output_flags);
    }
}
