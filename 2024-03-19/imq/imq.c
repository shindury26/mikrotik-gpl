#include <linux/module.h>
#include <linux/if_arp.h>
#include <linux/netfilter_ros.h>
#include <linux/imq.h>
#include <linux/ipv6.h>
#include <linux/skb_bin.h>
#include <net/codel.h>
#include <net/codel_qdisc.h>
#include <net/dst.h>
#include <net/pkt_sched.h>
#include <net/netfilter/nf_queue.h>
//#define STATS
#include <linux/stats.h>
#ifdef STATS
atomic_t stats_enqueue;
atomic_t stats_enqueue_classified;
atomic_t stats_dequeue_immediate;

//void stats_more(void) {
//    printk("sub=%d", stats_rx_submitted - stats_rx_cb);
//}
struct stats_descr stats_imq = {
    .name = "imq",
    .interval = HZ,
//    .more = stats_more,
    .elements = {
	{
	    .name = "enq_reg",
	    .counter = &stats_enqueue,
	},
	{
	    .name = "enq_classified",
	    .counter = &stats_enqueue_classified,
	},
	{
	    .name = "deq_imm",
	    .counter = &stats_dequeue_immediate,
	},
	{
	}
    }
};
#endif

//#define DEBUG
#ifdef DEBUG
static int debug_stats_on;
static int debug_stats_ifindex_dst;
static atomic_t debug_stats_counter0;
static atomic_t debug_stats_counter1;
static atomic_t debug_stats_counter2;
static atomic_t debug_stats_counter3;
static atomic_t debug_stats_counter4;
static atomic_t debug_stats_counter5;
static atomic_t debug_stats_counter6;
static atomic_t debug_stats_counter7;

static atomic_t debug_stats_counter0_bytes;
static atomic_t debug_stats_counter1_bytes;
static atomic_t debug_stats_counter2_bytes;
static atomic_t debug_stats_counter3_bytes;
static atomic_t debug_stats_counter4_bytes;
static atomic_t debug_stats_counter5_bytes;
static atomic_t debug_stats_counter6_bytes;
static atomic_t debug_stats_counter7_bytes;
#endif

static unsigned imq_dev_count;
static struct net_device **imq_devs[IMQ_TYPE_MAX];
static unsigned open_devs;
/*
  Notes on skb->cb:
  sch_codel/sch_cake uses skb->cb in qdisc layer (at least from parts of qdisc layer that are used in RouterOS).
  In general other protocol layers expect skb->cb to be initialized to zero.
  At least TCP is known to require this, it can crash in very obscure ways if not.
  I cannot just memset all skb->cb to zero.
  LOCAL_IN for ipv6 expects inet6_skb_parm to survive from ip6_input to ip6_input_finish.
  I am also stuffing original skb->destructor in skb->cb.
  if sizeof(biggest sch cb user) + sizeof(inet6_skb_parm) + sizeof(ptr) <= sizeof(skb->cb) I can put everything in there.

  Ipv4 also uses skb->cb on LOCAL_IN - iif is stored in first 4 bytes of cb that gets passed on to IP_PTKINFO struct in_pktinfo->ipi_ifindex on recvmsg - this is known to be used by resolver.
*/

static_assert(sizeof(struct codel_skb_cb) <= sizeof(struct inet6_skb_parm));
struct cobalt_skb_cb {
	ktime_t enqueue_time;
	u32     adjusted_len;
};
static_assert(sizeof(struct cobalt_skb_cb) <= sizeof(struct inet6_skb_parm));

struct imq_skb_cb {
    struct inet6_skb_parm orig_inet6_skb_parm; // 16
    struct inet6_skb_parm copy_inet6_skb_parm; // 16
    void (*orig_destructor)(struct sk_buff *skb); // 8
};
static_assert(sizeof(struct imq_skb_cb) <= sizeof_field(struct sk_buff, cb));

struct imq_priv {
    unsigned type;
    unsigned num;
    unsigned clstreeid;
};


static void imq_destructor(struct sk_buff *skb) {
    struct imq_skb_cb *cb = (struct imq_skb_cb *)skb->cb;
    dev_put(skb->dev);
    if (cb->orig_destructor) {
	cb->orig_destructor(skb);
	cb->orig_inet6_skb_parm = cb->copy_inet6_skb_parm;
	memset(&cb->copy_inet6_skb_parm, 0, sizeof(cb->copy_inet6_skb_parm));
	cb->orig_destructor = NULL;
    }
}

static inline bool imq_universal_client_hook(struct sk_buff *skb);
static inline void imq_reinject(struct sk_buff *skb) {
    struct imq_skb_cb *cb = (struct imq_skb_cb *)skb->cb;

    if (skb->destructor == imq_destructor) {
	dev_put(skb->dev);
	skb->destructor = cb->orig_destructor;
	cb->orig_inet6_skb_parm = cb->copy_inet6_skb_parm;
	memset(&cb->copy_inet6_skb_parm, 0, sizeof(cb->copy_inet6_skb_parm));
	cb->orig_destructor = NULL;
    }

//    printk("imq_reinject: okfn:%pS orig_destr%pS\n",
//	    skb->okfn, cb->orig_destructor);
//    skb_dump("imq_reinject", skb);
    if (!imq_universal_client_hook(skb)) {
	return;
    }
    skb->okfn(&init_net, NULL, skb);
}

DEFINE_PER_CPU(unsigned, per_cpu_imq_depth);

/* NOTE:
   Don't do immediate dequeue for locally generated packets by checking skb_iif to avoid deadlocks like this:

kernel/linux6/arch/tile/include/asm/spinlock_64.h:41  arch_spin_trylock
            kernel/linux6/lib/spinlock_debug.c:114  do_raw_spin_lock
               kernel/linux6/kernel/spinlock.c:349  _raw_spin_lock_nested
            kernel/linux6/net/ipv4/tcp_ipv4.c:1726  tcp_v4_rcv
             kernel/linux6/net/ipv4/ip_input.c:227  ip_local_deliver_finish
                      system/drivers/imq/imq.c:241  imq_xmit
                      system/drivers/imq/imq.c:189  imq_hook
            kernel/linux6/net/netfilter/core.c:156  nf_iterate
            kernel/linux6/net/netfilter/core.c:192  nf_hook_slow
       kernel/linux6/include/linux/netfilter.h:247  ip_output
               kernel/linux6/include/net/dst.h:433  ip_local_out
            kernel/linux6/net/ipv4/ip_output.c:177  ip_build_and_send_pkt
             kernel/linux6/net/ipv4/tcp_ipv4.c:798  tcp_v4_send_synack
            kernel/linux6/net/ipv4/tcp_ipv4.c:1409  tcp_v4_conn_request
           kernel/linux6/net/ipv4/tcp_input.c:5829  tcp_rcv_state_process
            kernel/linux6/net/ipv4/tcp_ipv4.c:1634  tcp_v4_do_rcv
            kernel/linux6/net/ipv4/tcp_ipv4.c:1737  tcp_v4_rcv
             kernel/linux6/net/ipv4/ip_input.c:227  ip_local_deliver_finish
                      system/drivers/imq/imq.c:241  imq_xmit
                 kernel/linux6/net/core/dev.c:2283  dev_hard_start_xmit
         kernel/linux6/net/sched/sch_generic.c:126  sch_direct_xmit
         kernel/linux6/net/sched/sch_generic.c:196  __qdisc_run
        kernel/linux6/include/linux/spinlock.h:325  net_tx_action
                kernel/linux6/kernel/softirq.c:243  __do_softirq
                kernel/linux6/kernel/softirq.c:768  run_ksoftirqd
                kernel/linux6/kernel/kthread.c:124  kthread
      kernel/linux6/arch/tile/kernel/process.c:728  start_kernel_thread

   Checking skb_iif also prevents reentering same tunnels xmit function on same core as skb_iff is reset to 0 by skb_reset_mark
 */

#define MAX_DEQUEUED_SKBS 4
static inline bool imq_enqueue(struct sk_buff *skb, struct net_device *imq,
	struct Qdisc *q, u32 classid) {
    bool ret = true;
    unsigned *imq_depth = raw_cpu_ptr(&per_cpu_imq_depth);
    struct sk_buff *dequeued_skbs[MAX_DEQUEUED_SKBS];
    unsigned dequeued_count = 0;
    unsigned i;
    int skb_iif = skb->skb_iif;
    struct sk_buff *to_free = NULL;

/*
    spin_lock(qdisc_lock(q));
    spin_unlock(qdisc_lock(q));
    imq->netdev_ops->ndo_start_xmit(skb, imq);
*/


    spin_lock(qdisc_lock(q));

    if (classid) {
	if (likely(q->ops->enqueue_classified)) {
	    stats_inc(&stats_imq, &stats_enqueue_classified);
	    ret = q->ops->enqueue_classified(skb, q, &to_free, classid);
//	    skb_dump("after enqueue_classfied", skb);
	}
	else {
	    ret = false;
	}
    }
    else {
	stats_inc(&stats_imq, &stats_enqueue);
	q->enqueue(skb, q, &to_free);
    }

    if (skb_iif && !*imq_depth) {
	for (i = 0; i < MAX_DEQUEUED_SKBS; ++i) {
	    dequeued_skbs[i] = q->ops->dequeue(q);
	    if (!dequeued_skbs[i]) {
		break;
	    }
	    ++dequeued_count;
	}
    }

    spin_unlock(qdisc_lock(q));

    if (unlikely(to_free))
	kfree_skb_list(to_free);

    if (dequeued_count) {
	stats_inc(&stats_imq, &stats_dequeue_immediate);
	*imq_depth += 1;
	for (i = 0; i < dequeued_count; ++i) {
	    imq->netdev_ops->ndo_start_xmit(dequeued_skbs[i], imq);
	}
	*imq_depth -= 1;
    }
    else {
	__netif_schedule(q);
    }

    return ret;
}

static netdev_tx_t imq_xmit(struct sk_buff *skb,
	struct net_device *imq) {
    struct imq_priv *priv = netdev_priv(imq);
    unsigned next = priv->type + 1;
    struct ll_classify_result res = { 0 };

//    printk("%s: xmit next:%u\n", imq->name, next);
//    skb_dump("imq_xmit", skb);
    if (next >= IMQ_TYPE_MAX) {
	imq_reinject(skb);
	return NETDEV_TX_OK;
    }
    imq = imq_devs[next][0];
    priv = netdev_priv(imq);

#ifdef DEBUG
    if (debug_stats_on && priv->clstreeid == CLS_IN_OUT) {
	atomic_inc(&debug_stats_counter5);
	atomic_add(skb->len, &debug_stats_counter5_bytes);

	if (skb->layer7seen) {
	    atomic_inc(&debug_stats_counter6);
	    atomic_add(skb->len, &debug_stats_counter6_bytes);

	    if (skb->dev && skb->dev->ifindex == debug_stats_ifindex_dst) {
		atomic_inc(&debug_stats_counter7);
		atomic_add(skb->len, &debug_stats_counter7_bytes);
	    }
	}
    }
#endif

    linear_lockless_classify(skb, priv->clstreeid, &res);
    if (!res.classid) {
	imq_reinject(skb);
	return NETDEV_TX_OK;
    }

    if (!imq_enqueue(skb, imq, res.qdisc, res.classid)) {
	imq_reinject(skb);
	return NETDEV_TX_OK;
    }

//    printk("xmit to next: done\n");
    return NETDEV_TX_OK;
}

static inline unsigned imq_try_queue(
	struct sk_buff *skb,
	int (*okfn)(struct net *net, struct sock *, struct sk_buff *),
	int type) {
    struct ll_classify_result res = { 0 };
    struct net_device *imq = imq_devs[type][0];
    struct imq_priv *priv = netdev_priv(imq);
    struct imq_skb_cb *cb = (struct imq_skb_cb *)skb->cb;

    if (type != IMQ_TYPE_TREE) {
	linear_lockless_classify(skb, priv->clstreeid, &res);
	if (!res.classid) {
	    return NF_ACCEPT;
	}
    }
    else {
	fw_lockless_classify(skb, NULL, &res);
	if (!res.classid) {
	    return NF_ACCEPT;
	}
    }


//    printk("imq_try_queue%u okfn:%pS res.classid:%x\n", type, okfn, res.classid);
//    skb_dump("imq_try_queue", skb);


    skb_dst_force(skb);
    skb->okfn = okfn;

    cb->copy_inet6_skb_parm = cb->orig_inet6_skb_parm;
    memset(&cb->orig_inet6_skb_parm, 0, sizeof(cb->orig_inet6_skb_parm));
    cb->orig_destructor = skb->destructor;
    skb->destructor = imq_destructor;
    dev_hold(skb->dev);
//    printk("imq: enqueue skb:%p orig_destr:%pS destr:%pS sk:%p\n",
//	    skb, cb->orig_destructor, skb->destructor, skb->sk);

    if (!imq_enqueue(skb, imq, res.qdisc, res.classid)) {
//	printk("imq: enqueue false\n");
	dev_put(skb->dev);
	skb->destructor = cb->orig_destructor;
	cb->orig_inet6_skb_parm = cb->copy_inet6_skb_parm;
	memset(&cb->copy_inet6_skb_parm, 0, sizeof(cb->copy_inet6_skb_parm));
	cb->orig_destructor = NULL;
	return NF_ACCEPT;
    }
    return NF_STOLEN;
}

static unsigned imq_hook(void *priv,
			 struct sk_buff *skb,
			 const struct nf_hook_state *st) {
    unsigned i;
    unsigned ret;


//    printk("imq_hook:%d in:%s out:%s okfn:%pS\n", st->hook,
//	    st->in ? st->in->name : "", st->out ? st->out->name : "", st->okfn);
//    skb_dump("imq_hook", skb);

    if (!skb->dev) {
	// XXX: occasionaly this can happen, f. e. ipv6 icmp responses,
	// probably something else. No skb->dev crashes in af_packet
	struct dst_entry *dst = skb_dst(skb);
	if (dst && dst->dev) {
	    skb->dev = dst->dev;
	}
	else if (!skb->dev) {
	    skb->dev = (struct net_device *)st->out;
	    if (!skb->dev) {
		printk("imq: eventually this will crash in af_packet\n");
		WARN_ON(1);
	    }
	}
    }
    // XXX: reuse layer7seen to pass hook information to cls_linear
    skb->layer7seen = st->hook == NF_INET_POST_ROUTING;
#ifdef DEBUG
    if (debug_stats_on) {
	atomic_inc(&debug_stats_counter0);
	atomic_add(skb->len, &debug_stats_counter0_bytes);

	if (skb->layer7seen) {
	    atomic_inc(&debug_stats_counter1);
	    atomic_add(skb->len, &debug_stats_counter1_bytes);

	    if (skb->dev && skb->dev->ifindex == debug_stats_ifindex_dst) {
		atomic_inc(&debug_stats_counter2);
		atomic_add(skb->len, &debug_stats_counter2_bytes);
	    }
	}
    }
#endif

    local_bh_disable();

    for (i = 0; i < IMQ_TYPE_MAX; ++i) {
	ret = imq_try_queue(skb, st->okfn, i);
	if (ret != NF_ACCEPT) {
#ifdef DEBUG
//	    WARN_ON(1);
	    if (debug_stats_on) {
		if (skb->layer7seen) {
		    if (skb->dev && skb->dev->ifindex == debug_stats_ifindex_dst) {
			if (i == IMQ_TYPE_TREE) {
			    atomic_inc(&debug_stats_counter3);
			    atomic_add(skb->len, &debug_stats_counter3_bytes);
			}
			if (i == IMQ_TYPE_SIMPLE_IN_OUT) {
			    atomic_inc(&debug_stats_counter4);
			    atomic_add(skb->len, &debug_stats_counter4_bytes);
			}
		    }
		}
	    }
#endif
	    local_bh_enable();
	    return ret;
	}
    }
    local_bh_enable();
    return NF_ACCEPT;
}

static unsigned imq_hook6(void *priv, struct sk_buff *skb,
			  const struct nf_hook_state *st) {
    // XXX: it is known that for some locally origniated packets skb->protocol
    // gets initialized in ip6_finish_output, which is too late for me,
    // this is kind of a quick hack to improve simple queue accuracy, please fix it properly
    skb->protocol = __constant_htons(ETH_P_IPV6);
    return imq_hook(priv, skb, st);
}

static struct nf_hook_ops imq_hooks[] = {
    {
	.hook = imq_hook,
	.pf = PF_INET,
	.hooknum = NF_INET_LOCAL_IN,
	.priority = NF_IP_PRI_IMQ
    },
    {
	.hook = imq_hook,
	.pf = PF_INET,
	.hooknum = NF_INET_POST_ROUTING,
	.priority = NF_IP_PRI_IMQ
    },
    {
	.hook = imq_hook6,
	.pf = PF_INET6,
	.hooknum = NF_INET_LOCAL_IN,
	.priority = NF_IP6_PRI_IMQ
    },
    {
	.hook = imq_hook6,
	.pf = PF_INET6,
	.hooknum = NF_INET_POST_ROUTING,
	.priority = NF_IP6_PRI_IMQ
    }
};

static inline bool imq_universal_client_hook(struct sk_buff *skb) {
    const struct nf_hook_entries *e;
    struct nf_hook_ops **ops;
    unsigned i = 0;
    int ret;
    
    if (skb->protocol != __constant_htons(ETH_P_IP) ||
	    !skb->layer7seen) {
	return true;
    }
    
    e = rcu_dereference(init_net.nf.hooks_ipv4[NF_INET_POST_ROUTING]);
    ops = nf_hook_entries_get_hook_ops(e);

    for (; i < e->num_hook_entries && ops[i] != &imq_hooks[1]; ++i);
    ++i;
    
    for (; i < e->num_hook_entries; ++i) {
	struct nf_hook_state st = {
	    .in = NULL,
	    .out = skb->dev,
	    .okfn = skb->okfn,
	};
	ret = ops[i]->hook(ops[i], skb, &st);
	if (ret == NF_DROP) {
	    kfree_skb(skb);
	    return false;
	}
	if (ret != NF_ACCEPT) {
	    BUG_ON(1);
	}
    }
    return true;
}

static int imq_open(struct net_device *dev) {
    if (!open_devs) {
	if (nf_register_net_hooks(&init_net, imq_hooks, ARRAY_SIZE(imq_hooks))) {
	    return -EINVAL;
	}
    }
    ++open_devs;
    netif_start_queue(dev);
    return 0;
}

static int imq_close(struct net_device *dev) {
    netif_stop_queue(dev);
    --open_devs;
    if (!open_devs) {
	nf_unregister_net_hooks(&init_net, imq_hooks, ARRAY_SIZE(imq_hooks));
    }
    return 0;
}

static const struct net_device_ops imq_netdev_ops = {
    .ndo_open = imq_open,
    .ndo_stop = imq_close,
    .ndo_start_xmit = imq_xmit,
};

static void imq_setup(struct net_device *dev) {
    dev->type = ARPHRD_VOID;
    dev->mtu = 16000;
    dev->tx_queue_len = 0;
    dev->flags = IFF_NOARP;
    dev->features = NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_GSO |
	NETIF_F_HW_CSUM | NETIF_F_HIGHDMA | NETIF_F_LLTX |
	NETIF_F_TSO | NETIF_F_GSO_ROBUST | NETIF_F_TSO_ECN |
	NETIF_F_TSO6 | NETIF_F_FSO;

    dev->priv_flags &= ~IFF_XMIT_DST_RELEASE;
    dev->priv_flags |= IFF_NO_QUEUE;
}

static int __init imq_init_dev(int type, int num) {
    struct net_device *dev;
    struct imq_priv *priv;

    dev = alloc_netdev(sizeof(struct imq_priv), "imq%d", NET_NAME_UNKNOWN,
		       imq_setup);
    if (!dev) {
	return -1;
    }
    priv = netdev_priv(dev);
    priv->type = type;
    priv->num = num;
    if (type == IMQ_TYPE_SIMPLE_IN_OUT) {
	priv->clstreeid = CLS_IN_OUT;
    }
    else if (type == IMQ_TYPE_SIMPLE_TOTAL) {
	priv->clstreeid = CLS_TOTAL;
    }

    dev->netdev_ops = &imq_netdev_ops;
    dev->needs_free_netdev = true;

    if (register_netdevice(dev) < 0) {
	free_netdev(dev);
	return -1;
    }
    imq_devs[type][num] = dev;
    return 0;
}

static void imq_cleanup_devs(void) {
    int i;
    int j;
    rtnl_lock();
    for (i = 0; i < IMQ_TYPE_MAX; i++) {
	if (!imq_devs[i]) {
	    continue;
	}
	for (j = 0; j < imq_dev_count; ++j) {
	    struct net_device *dev = imq_devs[i][j];
	    if (dev) {
		unregister_netdevice(dev);
	    }
	}
	kfree(imq_devs[i]);
    }
    rtnl_unlock();
}



#ifdef DEBUG
static ssize_t imq_sysfs_debug_stats_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf) {
    return sprintf(buf, "%d %d\n%u %u\n%u %u\n%u %u\n%u %u\n%u %u\n%u %u\n%u %u\n%u %u\n",
	    debug_stats_on,
	    debug_stats_ifindex_dst,
	    atomic_read(&debug_stats_counter0),
	    atomic_read(&debug_stats_counter0_bytes),
	    atomic_read(&debug_stats_counter1),
	    atomic_read(&debug_stats_counter1_bytes),
	    atomic_read(&debug_stats_counter2),
	    atomic_read(&debug_stats_counter2_bytes),
	    atomic_read(&debug_stats_counter3),
	    atomic_read(&debug_stats_counter3_bytes),
	    atomic_read(&debug_stats_counter4),
	    atomic_read(&debug_stats_counter4_bytes),
	    atomic_read(&debug_stats_counter5),
	    atomic_read(&debug_stats_counter5_bytes),
	    atomic_read(&debug_stats_counter6),
	    atomic_read(&debug_stats_counter6_bytes),
	    atomic_read(&debug_stats_counter7),
	    atomic_read(&debug_stats_counter7_bytes)
	);
}

static ssize_t imq_sysfs_debug_stats_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count) {
    sscanf(buf, "%d %d", &debug_stats_on,
	    &debug_stats_ifindex_dst);
    atomic_set(&debug_stats_counter0, 0);
    atomic_set(&debug_stats_counter0_bytes, 0);
    atomic_set(&debug_stats_counter1, 0);
    atomic_set(&debug_stats_counter1_bytes, 0);
    atomic_set(&debug_stats_counter2, 0);
    atomic_set(&debug_stats_counter2_bytes, 0);
    atomic_set(&debug_stats_counter3, 0);
    atomic_set(&debug_stats_counter3_bytes, 0);
    atomic_set(&debug_stats_counter4, 0);
    atomic_set(&debug_stats_counter4_bytes, 0);
    atomic_set(&debug_stats_counter5, 0);
    atomic_set(&debug_stats_counter5_bytes, 0);
    atomic_set(&debug_stats_counter6, 0);
    atomic_set(&debug_stats_counter6_bytes, 0);
    atomic_set(&debug_stats_counter7, 0);
    atomic_set(&debug_stats_counter7_bytes, 0);
    return count;
}

static struct kobj_attribute imq_debug_stats_attribute =
    __ATTR(imq_debug_stats, 0664, imq_sysfs_debug_stats_show, imq_sysfs_debug_stats_store);
static struct attribute *attrs[] = {
    &imq_debug_stats_attribute.attr,
    NULL,
};
static struct attribute_group attr_group = {
    .attrs = attrs,
};
#endif

static int __init imq_init_module(void) {
    int i;
    int j;

    if (sizeof(struct imq_skb_cb) > sizeof(((struct sk_buff *)0)->cb)) {
	printk("imq: skb->cb too small: sizeof(imq_skb_cb) %u > sizeof(sk_buff->cb) %u\n",
		(unsigned)sizeof(struct imq_skb_cb),
		(unsigned)sizeof(((struct sk_buff *)0)->cb));
    }

#ifdef DEBUG
    {
	int ret = sysfs_create_group(kernel_kobj, &attr_group);
	if (ret) {
	    return ret;
	}
    }
#endif

    rtnl_lock();
    imq_dev_count = num_online_cpus();
    if (imq_dev_count > 1 && imq_dev_count < 8) {
	imq_dev_count = 8;
    }
    for (i = 0; i < IMQ_TYPE_MAX; i++) {
	imq_devs[i] = kzalloc(sizeof(struct net_device *) * imq_dev_count,
		GFP_KERNEL);
	if (!imq_devs[i]) {
	    rtnl_unlock();
	    imq_cleanup_devs();
	    return -EINVAL;
	}
	for (j = 0; j < imq_dev_count; ++j) {
	    if (imq_init_dev(i, j)) {
		rtnl_unlock();
		imq_cleanup_devs();
		return -EINVAL;
	    }
	}
    }
    rtnl_unlock();
    return 0;
}

static void __exit imq_exit_module(void) {
    printk("imq: exit module\n");
    imq_cleanup_devs();
#ifdef DEBUG
    sysfs_remove_group(kernel_kobj, &attr_group);
#endif
}

module_init(imq_init_module);
module_exit(imq_exit_module);
MODULE_LICENSE("GPL");
