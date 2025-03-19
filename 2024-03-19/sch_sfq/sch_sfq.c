#include <linux/module.h>
#include <linux/jhash.h>
#include <net/pkt_sched.h>

#define SFQ_MAX_QLEN 127 /* max number of packets per flow */
#define SFQ_LIMIT 127 /* max total number of packets */
#define SFQ_FLOWS 128
#define SFQ_NO_FLOW 0xff
#define SFQ_HASH_SIZE 1024

// We dont use pointers to save space.
// Small indexes [0 ... SFQ_FLOWS - 1] are 'pointers' to flows[] array
// while following values [SFQ_FLOWS ... SFQ_FLOWS + SFQ_MAX_QLEN]
// are 'pointers' to dep[] array
struct idx_list_head {
    u8 next;
    u8 prev;
};

struct sfq_flow {
    struct sk_buff_head skblist;
    u8 next;
    u16 hash;
    struct idx_list_head dep;
    int allot;
};

struct sfq_data {
    struct Qdisc *sch;
    u32 perturbation;
    u8 max_qlen;
    u8 flags;
    u8 ht[SFQ_HASH_SIZE];
    struct sfq_flow flows[SFQ_FLOWS];

    struct sfq_flow *tail; // current flow in round

    struct idx_list_head dep[SFQ_MAX_QLEN + 1]; // linked lists of flows, indexed by qlen

    int perturb_period;
    unsigned quantum;
    struct timer_list perturb_timer;
};

// idx_list_head are either in a sfq_flow or in dep[] array
static inline struct idx_list_head *sfq_dep_head(struct sfq_data *q, u16 val) {
    if (val < SFQ_FLOWS) {
	return &q->flows[val].dep;
    }
    return &q->dep[val - SFQ_FLOWS];
}

static unsigned sfq_classify(struct sk_buff *skb, struct sfq_data *q) {
    struct flow_keys keys;
    skb_flow_dissect_flow_keys(skb, &keys, 0);
    return jhash_3words((__force u32)keys.addrs.v4addrs.dst,
	    (__force u32)keys.addrs.v4addrs.src ^ keys.basic.ip_proto,
	    (__force u32)keys.ports.ports, q->perturbation) & (SFQ_HASH_SIZE - 1);
}

// x : flow number [0 .. SFQ_FLOWS - 1]
static inline void sfq_link(struct sfq_data *q, u16 x) {
    u16 p, n;
    struct sfq_flow *flow = &q->flows[x];
    int qlen = skb_queue_len(&flow->skblist);

    p = qlen + SFQ_FLOWS;
    n = q->dep[qlen].next;

    flow->dep.next = n;
    flow->dep.prev = p;

    q->dep[qlen].next = x; // sfq_dep_head(q, p)->next = x
    sfq_dep_head(q, n)->prev = x;
}

#define sfq_unlink(q, x, n, p)			\
    n = q->flows[x].dep.next;			\
    p = q->flows[x].dep.prev;			\
    sfq_dep_head(q, p)->next = n;		\
    sfq_dep_head(q, n)->prev = p


static inline void sfq_dec(struct sfq_data *q, u16 x) {
    u16 p, n;
    sfq_unlink(q, x, n, p);

    if (n == p && q->max_qlen == (skb_queue_len(&q->flows[x].skblist) + 1)) {
	q->max_qlen--;
    }
    sfq_link(q, x);
}

static inline void sfq_inc(struct sfq_data *q, u16 x) {
    u16 p, n;
    sfq_unlink(q, x, n, p);

    if (q->max_qlen < skb_queue_len(&q->flows[x].skblist)) {
	q->max_qlen++;
    }
    sfq_link(q, x);
}

static unsigned sfq_drop(struct Qdisc *sch) {
    struct sfq_data *q = qdisc_priv(sch);
    u16 x;
    struct sk_buff *skb;
    unsigned len;
    struct sfq_flow *flow;

    // get the longest flow and drop tail packet from it
    if (q->max_qlen > 1) {
	x = q->dep[q->max_qlen].next;
	flow = &q->flows[x];
    drop:
	skb = __skb_dequeue(&flow->skblist);
	len = qdisc_pkt_len(skb);
	sfq_dec(q, x);
	kfree_skb(skb);
	sch->q.qlen--;
	sch->qstats.drops++;
	sch->qstats.backlog -= len;
	return len;
    }

    if (q->max_qlen == 1) {
	x = q->tail->next;
	flow = &q->flows[x];
	q->tail->next = flow->next;
	q->ht[flow->hash] = SFQ_NO_FLOW;
	goto drop;
    }
    return 0;
}

static int sfq_enqueue(struct sk_buff *skb, struct Qdisc *sch,
		       struct sk_buff **to_free) {
    struct sfq_data *q = qdisc_priv(sch);
    unsigned hash = sfq_classify(skb, q);
    int dropped;
    u16 x, qlen;
    struct sfq_flow *flow;

    x = q->ht[hash];
    flow = &q->flows[x];
    if (x == SFQ_NO_FLOW) {
	x = q->dep[0].next; // get a empty flow
	q->ht[hash] = x;
	flow = &q->flows[x];
	flow->hash = hash;
	goto enqueue;
    }

    if (skb_queue_len(&flow->skblist) >= SFQ_MAX_QLEN) {
	struct sk_buff *head = __skb_dequeue(&flow->skblist);
	sch->qstats.backlog -= qdisc_pkt_len(head) - qdisc_pkt_len(skb);
	qdisc_drop(head, sch, to_free);

	__skb_queue_tail(&flow->skblist, skb);
	return NET_XMIT_CN;
    }

enqueue:
    sch->qstats.backlog += qdisc_pkt_len(skb);
    __skb_queue_tail(&flow->skblist, skb);
    sfq_inc(q, x);
    if (skb_queue_len(&flow->skblist) == 1) {
	if (!q->tail) {
	    flow->next = x;
	}
	else {
	    flow->next = q->tail->next;
	    q->tail->next = x;
	}
	// We put this flow at the end of our flow list.
	// This might sound unfair for a new flow to wait after old ones,
	// but we could endup servicing new flows only, and freeze old ones.
	q->tail = flow;
	flow->allot = q->quantum;
    }
    if (++sch->q.qlen <= SFQ_LIMIT) {
	return NET_XMIT_SUCCESS;
    }

    qlen = skb_queue_len(&flow->skblist);
    dropped = sfq_drop(sch);
    // Return Congestion Notification only if we dropped a packet from this flow
    if (qlen != skb_queue_len(&flow->skblist)) {
	return NET_XMIT_CN;
    }

    qdisc_tree_reduce_backlog(sch, 1, dropped);
    return NET_XMIT_SUCCESS;
}

static struct sk_buff *sfq_dequeue(struct Qdisc *sch) {
    struct sfq_data *q = qdisc_priv(sch);
    struct sk_buff *skb;
    u16 a, next_a;
    struct sfq_flow *flow;

    if (q->tail == NULL)
	return NULL;

next_flow:
    a = q->tail->next;
    flow = &q->flows[a];
    if (flow->allot <= 0) {
	q->tail = flow;
	flow->allot += q->quantum;
	goto next_flow;
    }
    skb = __skb_dequeue(&flow->skblist);
    sfq_dec(q, a);
    qdisc_bstats_update(sch, skb);
    sch->q.qlen--;
    sch->qstats.backlog -= qdisc_pkt_len(skb);
    if (skb_queue_empty(&flow->skblist)) {
	q->ht[flow->hash] = SFQ_NO_FLOW;
	next_a = flow->next;
	if (a == next_a) {
	    q->tail = NULL;
	    return skb;
	}
	q->tail->next = next_a;
    } else {
	flow->allot -= qdisc_pkt_len(skb);
    }
    return skb;
}

static void sfq_reset(struct Qdisc *sch) {
    struct sk_buff *skb;

    while ((skb = sfq_dequeue(sch)) != NULL) {
	kfree_skb(skb);
    }
}

// When q->perturbation is changed, we rehash all queued skbs
// to avoid OOO (Out Of Order) effects.
// We dont use sfq_dequeue()/sfq_enqueue() because we dont want to change
// counters.
static void sfq_rehash(struct Qdisc *sch) {
    struct sfq_data *q = qdisc_priv(sch);
    struct sk_buff *skb;
    int i;
    struct sfq_flow *flow;
    struct sk_buff_head list;
    int dropped = 0;
    unsigned int drop_len = 0;

    __skb_queue_head_init(&list);

    for (i = 0; i < SFQ_FLOWS; i++) {
	flow = &q->flows[i];
	while (skb_queue_len(&flow->skblist)) {
	    skb = __skb_dequeue(&flow->skblist);
	    sfq_dec(q, i);
	    __skb_queue_tail(&list, skb);
	}
	q->ht[flow->hash] = SFQ_NO_FLOW;
    }
    q->tail = NULL;

    while ((skb = __skb_dequeue(&list)) != NULL) {
	unsigned hash = sfq_classify(skb, q);
	u16 x = q->ht[hash];

	flow = &q->flows[x];
	if (x == SFQ_NO_FLOW) {
	    x = q->dep[0].next;
	    if (x >= SFQ_FLOWS) {
	    drop:
		qdisc_qstats_backlog_dec(sch, skb);
		drop_len += qdisc_pkt_len(skb);
		kfree_skb(skb);
		dropped++;
		continue;
	    }
	    q->ht[hash] = x;
	    flow = &q->flows[x];
	    flow->hash = hash;
	}
	if (skb_queue_len(&flow->skblist) >= SFQ_MAX_QLEN) {
	    goto drop;
	}
	__skb_queue_tail(&flow->skblist, skb);
	sfq_inc(q, x);
	if (skb_queue_len(&flow->skblist) == 1) {
	    if (q->tail == NULL) {
		flow->next = x;
	    } else {
		flow->next = q->tail->next;
		q->tail->next = x;
	    }
	    q->tail = flow;
	    flow->allot = q->quantum;
	}
    }
    sch->q.qlen -= dropped;
    qdisc_tree_reduce_backlog(sch, dropped, drop_len);
}

static void sfq_perturbation(struct timer_list *t) {
    struct sfq_data *q = from_timer(q, t, perturb_timer);
    struct Qdisc *sch = q->sch;
    spinlock_t *root_lock = qdisc_lock(qdisc_root_sleeping(sch));

    spin_lock(root_lock);
    q->perturbation = prandom_u32();
    if (q->tail) {
	sfq_rehash(sch);
    }
    spin_unlock(root_lock);

    if (q->perturb_period)
	mod_timer(&q->perturb_timer, jiffies + q->perturb_period);
}

static int sfq_change(struct Qdisc *sch, struct nlattr *opt) {
    struct sfq_data *q = qdisc_priv(sch);
    struct tc_sfq_qopt *ctl = nla_data(opt);
    unsigned qlen;
    unsigned dropped = 0;

    if (opt->nla_len < nla_attr_size(sizeof(*ctl)))
	return -EINVAL;
    sch_tree_lock(sch);
    if (ctl->quantum) {
	q->quantum = ctl->quantum;
    }
    q->perturb_period = ctl->perturb_period * HZ;

    qlen = sch->q.qlen;
    while (sch->q.qlen > SFQ_LIMIT) {
	dropped += sfq_drop(sch);
    }
    qdisc_tree_reduce_backlog(sch, qlen - sch->q.qlen, dropped);

    del_timer(&q->perturb_timer);
    if (q->perturb_period) {
	mod_timer(&q->perturb_timer, jiffies + q->perturb_period);
	q->perturbation = prandom_u32();
    }
    sch_tree_unlock(sch);
    return 0;
}

static void sfq_destroy(struct Qdisc *sch) {
    struct sfq_data *q = qdisc_priv(sch);
    q->perturb_period = 0;
    del_timer_sync(&q->perturb_timer);
}

static int sfq_init(struct Qdisc *sch, struct nlattr *opt,
		    struct netlink_ext_ack *extack) {
    struct sfq_data *q = qdisc_priv(sch);
    int i;

    q->sch = sch;
    timer_setup(&q->perturb_timer, sfq_perturbation, TIMER_DEFERRABLE);

    for (i = 0; i < SFQ_MAX_QLEN + 1; i++) {
	q->dep[i].next = i + SFQ_FLOWS;
	q->dep[i].prev = i + SFQ_FLOWS;
    }

    q->max_qlen = 0;
    q->tail = NULL;
    q->quantum = psched_mtu(qdisc_dev(sch));
    q->perturb_period = 0;
    q->perturbation = prandom_u32();

    if (opt) {
	int err = sfq_change(sch, opt);
	if (err)
	    return err;
    }

    for (i = 0; i < SFQ_HASH_SIZE; i++) {
	q->ht[i] = SFQ_NO_FLOW;
    }

    for (i = 0; i < SFQ_FLOWS; i++) {
	struct sfq_flow *flow = &q->flows[i];
	memset(flow, 0, sizeof(*flow));
	__skb_queue_head_init(&flow->skblist);
	sfq_link(q, i);
    }
    sch->flags |= TCQ_F_CAN_BYPASS;
    return 0;
}

static int sfq_dump(struct Qdisc *sch, struct sk_buff *skb) {
    struct sfq_data *q = qdisc_priv(sch);
    unsigned char *b = skb_tail_pointer(skb);
    struct tc_sfq_qopt_v1 opt;

    memset(&opt, 0, sizeof(opt));
    opt.v0.quantum = q->quantum;
    opt.v0.perturb_period = q->perturb_period / HZ;
    opt.flags = q->flags;

    if (nla_put(skb, TCA_OPTIONS, sizeof(opt), &opt))
	goto nla_put_failure;
    
    return skb->len;
nla_put_failure:
    nlmsg_trim(skb, b);
    return -1;
}

static struct Qdisc *sfq_leaf(struct Qdisc *sch, unsigned long arg) {
    return NULL;
}

static unsigned long sfq_find(struct Qdisc *sch, u32 classid) {
    return 0;
}

static unsigned long sfq_bind(struct Qdisc *sch, unsigned long parent,
	u32 classid) {
    return 0;
}

static void sfq_put(struct Qdisc *q, unsigned long cl) {
}

static struct tcf_block *sfq_find_tcf(struct Qdisc *sch, unsigned long cl,
				      struct netlink_ext_ack *extack) {
    return NULL;
}

static int sfq_dump_class(struct Qdisc *sch, unsigned long cl,
	struct sk_buff *skb, struct tcmsg *tcm) {
    return 0;
}

static int sfq_dump_class_stats(struct Qdisc *sch, unsigned long cl,
	struct gnet_dump *d) {
    struct sfq_data *q = qdisc_priv(sch);
    u16 idx = q->ht[cl - 1];
    struct gnet_stats_queue qs = { 0 };
    struct tc_sfq_xstats xstats = { 0 };

    if (idx != SFQ_NO_FLOW) {
	const struct sfq_flow *flow = &q->flows[idx];

	xstats.allot = flow->allot;
	qs.qlen = skb_queue_len(&flow->skblist);
    }
    if (gnet_stats_copy_queue(d, NULL, &qs, qs.qlen) < 0)
	return -1;
    return gnet_stats_copy_app(d, &xstats, sizeof(xstats));
}

static void sfq_walk(struct Qdisc *sch, struct qdisc_walker *arg) {
    struct sfq_data *q = qdisc_priv(sch);
    unsigned i;

    if (arg->stop)
	return;

    for (i = 0; i < SFQ_HASH_SIZE; i++) {
	if (q->ht[i] == SFQ_NO_FLOW || arg->count < arg->skip) {
	    arg->count++;
	    continue;
	}
	if (arg->fn(sch, i + 1, arg) < 0) {
	    arg->stop = 1;
	    break;
	}
	arg->count++;
    }
}

static const struct Qdisc_class_ops sfq_class_ops = {
    .leaf = sfq_leaf,
    .find = sfq_find,
    .tcf_block = sfq_find_tcf,
    .bind_tcf = sfq_bind,
    .unbind_tcf = sfq_put,
    .dump = sfq_dump_class,
    .dump_stats = sfq_dump_class_stats,
    .walk = sfq_walk,
};

static struct Qdisc_ops sfq_qdisc_ops __read_mostly = {
    .cl_ops = &sfq_class_ops,
    .id = "sfq",
    .priv_size = sizeof(struct sfq_data),
    .enqueue = sfq_enqueue,
    .dequeue = sfq_dequeue,
    .peek = qdisc_peek_dequeued,
    .init = sfq_init,
    .reset = sfq_reset,
    .destroy = sfq_destroy,
    .dump = sfq_dump,
    .owner = THIS_MODULE,
};

static int __init sfq_module_init(void) {
    printk("sizeof(struct sfq_data): %u\n", (unsigned)sizeof(struct sfq_data));
    return register_qdisc(&sfq_qdisc_ops);
}
static void __exit sfq_module_exit(void) {
    unregister_qdisc(&sfq_qdisc_ops);
}
module_init(sfq_module_init)
module_exit(sfq_module_exit)
MODULE_LICENSE("GPL");
