#include <linux/module.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_connlimit.h>
#include <linux/netfilter_bridge/ebtables.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_acct.h>
#include <net/dsfield.h>
#include <net/ip.h>
#include <net/ipv6.h>

#include <linux/ipt_fastpath_options.h>
#include <linux/ipt_ipv4options.h>
#include <linux/ipt_IPV4OPTSSTRIP.h>
#include <linux/ipt_iprange.h>
#include <linux/ipt_CLEARDF.h>
#include <linux/ipt_PASSTHR.h>
#include <linux/xt_misc.h>
#include <linux/addrlist.h>
#include <linux/packet_hook.h>
#include "../bridge2/br_private.h"


/***   route   ***************************************/

static unsigned route_target(struct sk_buff *skb,
	const struct xt_action_param *par) {
    const struct xt_route_info *info = par->targinfo;
    const struct iphdr *iph = ip_hdr(skb);

    ip_route_input_noref(skb, info->dst, iph->saddr, iph->tos, skb->dev);
    if (info->passthrough) {
	return XT_CONTINUE;
    }
    return NF_ACCEPT;
}

static struct xt_target xt_route_target = {
    .name = "ROUTE",
    .family = AF_INET,
    .target = route_target,
    .targetsize = sizeof(struct xt_route_info),
    .me = THIS_MODULE
};


/***   fastpathconn   ***************************************/

static unsigned int fastpathconn_target(struct sk_buff *skb,
	const struct xt_action_param *par) {
    enum ip_conntrack_info ctinfo;
    struct nf_conn *ct = nf_ct_get(skb, &ctinfo);
    const struct ipt_fastpath_target_info *info = par->targinfo;

    if (ct) {
	if (!(ct->status & IPS_FASTPATH)) {
	    set_bit(IPS_FASTPATH_BIT, &ct->status);
	    if (info->hw_offload) {
		clear_bit(IPS_HW_DISABLE_BIT, &ct->status);
	    }
	    else {
		set_bit(IPS_HW_DISABLE_BIT, &ct->status);
	    }
	}
    }
    return XT_CONTINUE;
}

static atomic_t fp_ipv4_ct_enable_count;
static int fastpathconn_checkentry(const struct xt_tgchk_param *par) {
    if (atomic_inc_return(&fp_ipv4_ct_enable_count) == 1) {
	fasttrack_changed_callback(1);
    }
    return 0;
}

static void fastpathconn_destroy(const struct xt_tgdtor_param *par) {
    if (atomic_dec_return(&fp_ipv4_ct_enable_count) == 0) {
	fasttrack_changed_callback(-1);
    }
}

static struct xt_target xt_fastpathconn_target = {
    .name = "FASTPATHCONN",
    .family = AF_INET,
    .target = fastpathconn_target,
    .targetsize = sizeof(struct ipt_fastpath_target_info),
    .checkentry = fastpathconn_checkentry,
    .destroy = fastpathconn_destroy,
    .me = THIS_MODULE
};

/***   ipv4options   ***************************************/

static bool ipv4options_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    const struct ipt_ipv4options_info *info = par->matchinfo;
    const struct iphdr *iph = ip_hdr(skb);
    const struct ip_options *opt;

    if (iph->ihl * 4 == sizeof(struct iphdr)) {
	/* No options, so we match only the "DONTs" and the "IGNOREs" */

	if (((info->options & IPT_IPV4OPTION_MATCH_ANY_OPT) == IPT_IPV4OPTION_MATCH_ANY_OPT) ||
		((info->options & IPT_IPV4OPTION_MATCH_SSRR) == IPT_IPV4OPTION_MATCH_SSRR) ||
		((info->options & IPT_IPV4OPTION_MATCH_LSRR) == IPT_IPV4OPTION_MATCH_LSRR) ||
		((info->options & IPT_IPV4OPTION_MATCH_RR) == IPT_IPV4OPTION_MATCH_RR) ||
		((info->options & IPT_IPV4OPTION_MATCH_TIMESTAMP) == IPT_IPV4OPTION_MATCH_TIMESTAMP) ||
		((info->options & IPT_IPV4OPTION_MATCH_ROUTER_ALERT) == IPT_IPV4OPTION_MATCH_ROUTER_ALERT))
	    return 0;
	return 1;
    }
    else {
	if ((info->options & IPT_IPV4OPTION_MATCH_ANY_OPT) == IPT_IPV4OPTION_MATCH_ANY_OPT)
	    /* there are options, and we don't need to care which one */
	    return 1;
	else {
	    if ((info->options & IPT_IPV4OPTION_DONT_MATCH_ANY_OPT) == IPT_IPV4OPTION_DONT_MATCH_ANY_OPT)
		/* there are options but we don't want any ! */
		return 0;
	}
    }

    opt = &(IPCB(skb)->opt);

    /* source routing */
    if ((info->options & IPT_IPV4OPTION_MATCH_SSRR) == IPT_IPV4OPTION_MATCH_SSRR) {
	if (!((opt->srr) & (opt->is_strictroute)))
	    return 0;
    }
    else if ((info->options & IPT_IPV4OPTION_MATCH_LSRR) == IPT_IPV4OPTION_MATCH_LSRR) {
	if (!((opt->srr) & (!opt->is_strictroute)))
	    return 0;
    }
    else if ((info->options & IPT_IPV4OPTION_DONT_MATCH_SRR) == IPT_IPV4OPTION_DONT_MATCH_SRR) {
	if (opt->srr)
	    return 0;
    }
    /* record route */
    if ((info->options & IPT_IPV4OPTION_MATCH_RR) == IPT_IPV4OPTION_MATCH_RR) {
	if (!opt->rr)
	    return 0;
    }
    else if ((info->options & IPT_IPV4OPTION_DONT_MATCH_RR) == IPT_IPV4OPTION_DONT_MATCH_RR) {
	if (opt->rr)
	    return 0;
    }
    /* timestamp */
    if ((info->options & IPT_IPV4OPTION_MATCH_TIMESTAMP) == IPT_IPV4OPTION_MATCH_TIMESTAMP) {
	if (!opt->ts)
	    return 0;
    }
    else if ((info->options & IPT_IPV4OPTION_DONT_MATCH_TIMESTAMP) == IPT_IPV4OPTION_DONT_MATCH_TIMESTAMP) {
	if (opt->ts)
	    return 0;
    }
    /* router-alert option  */
    if ((info->options & IPT_IPV4OPTION_MATCH_ROUTER_ALERT) == IPT_IPV4OPTION_MATCH_ROUTER_ALERT) {
	if (!opt->router_alert)
	    return 0;
    }
    else if ((info->options & IPT_IPV4OPTION_DONT_MATCH_ROUTER_ALERT) == IPT_IPV4OPTION_DONT_MATCH_ROUTER_ALERT) {
	if (opt->router_alert)
	    return 0;
    }

    /* we match ! */
    return 1;
}

static int ipv4options_checkentry(const struct xt_mtchk_param *par) {
    const struct ipt_ipv4options_info *info = par->matchinfo;
    /* Now check the coherence of the data ... */
    if (((info->options & IPT_IPV4OPTION_MATCH_ANY_OPT) == IPT_IPV4OPTION_MATCH_ANY_OPT) &&
	    (((info->options & IPT_IPV4OPTION_DONT_MATCH_SRR) == IPT_IPV4OPTION_DONT_MATCH_SRR) ||
		    ((info->options & IPT_IPV4OPTION_DONT_MATCH_RR) == IPT_IPV4OPTION_DONT_MATCH_RR) ||
		    ((info->options & IPT_IPV4OPTION_DONT_MATCH_TIMESTAMP) == IPT_IPV4OPTION_DONT_MATCH_TIMESTAMP) ||
		    ((info->options & IPT_IPV4OPTION_DONT_MATCH_ROUTER_ALERT) == IPT_IPV4OPTION_DONT_MATCH_ROUTER_ALERT) ||
		    ((info->options & IPT_IPV4OPTION_DONT_MATCH_ANY_OPT) == IPT_IPV4OPTION_DONT_MATCH_ANY_OPT)))
	return -EINVAL; /* opposites */
    if (((info->options & IPT_IPV4OPTION_DONT_MATCH_ANY_OPT) == IPT_IPV4OPTION_DONT_MATCH_ANY_OPT) &&
	    (((info->options & IPT_IPV4OPTION_MATCH_LSRR) == IPT_IPV4OPTION_MATCH_LSRR) ||
		    ((info->options & IPT_IPV4OPTION_MATCH_SSRR) == IPT_IPV4OPTION_MATCH_SSRR) ||
		    ((info->options & IPT_IPV4OPTION_MATCH_RR) == IPT_IPV4OPTION_MATCH_RR) ||
		    ((info->options & IPT_IPV4OPTION_MATCH_TIMESTAMP) == IPT_IPV4OPTION_MATCH_TIMESTAMP) ||
		    ((info->options & IPT_IPV4OPTION_MATCH_ROUTER_ALERT) == IPT_IPV4OPTION_MATCH_ROUTER_ALERT) ||
		    ((info->options & IPT_IPV4OPTION_MATCH_ANY_OPT) == IPT_IPV4OPTION_MATCH_ANY_OPT)))
	return -EINVAL; /* opposites */
    if (((info->options & IPT_IPV4OPTION_MATCH_SSRR) == IPT_IPV4OPTION_MATCH_SSRR) &&
	    ((info->options & IPT_IPV4OPTION_MATCH_LSRR) == IPT_IPV4OPTION_MATCH_LSRR))
	return -EINVAL; /* cannot match in the same time loose and strict source routing */
    if ((((info->options & IPT_IPV4OPTION_MATCH_SSRR) == IPT_IPV4OPTION_MATCH_SSRR) ||
		    ((info->options & IPT_IPV4OPTION_MATCH_LSRR) == IPT_IPV4OPTION_MATCH_LSRR)) &&
	    ((info->options & IPT_IPV4OPTION_DONT_MATCH_SRR) == IPT_IPV4OPTION_DONT_MATCH_SRR))
	return -EINVAL; /* opposites */
    if (((info->options & IPT_IPV4OPTION_MATCH_RR) == IPT_IPV4OPTION_MATCH_RR) &&
	    ((info->options & IPT_IPV4OPTION_DONT_MATCH_RR) == IPT_IPV4OPTION_DONT_MATCH_RR))
	return -EINVAL; /* opposites */
    if (((info->options & IPT_IPV4OPTION_MATCH_TIMESTAMP) == IPT_IPV4OPTION_MATCH_TIMESTAMP) &&
	    ((info->options & IPT_IPV4OPTION_DONT_MATCH_TIMESTAMP) == IPT_IPV4OPTION_DONT_MATCH_TIMESTAMP))
	return -EINVAL; /* opposites */
    if (((info->options & IPT_IPV4OPTION_MATCH_ROUTER_ALERT) == IPT_IPV4OPTION_MATCH_ROUTER_ALERT) &&
	    ((info->options & IPT_IPV4OPTION_DONT_MATCH_ROUTER_ALERT) == IPT_IPV4OPTION_DONT_MATCH_ROUTER_ALERT))
	return -EINVAL; /* opposites */

    /* everything looks ok. */
    return 0;
}

static struct xt_match xt_ipv4options_match = {
    .name = "ipv4options",
    .family = AF_INET,
    .match = ipv4options_match,
    .matchsize = sizeof(struct ipt_ipv4options_info),
    .checkentry = ipv4options_checkentry,
    .me = THIS_MODULE
};


/***   ipv4optstrip   ***************************************/

static unsigned int ipv4optstrip_target(struct sk_buff *skb,
	const struct xt_action_param *par) {
    struct iphdr *iph;
    struct ip_options *opt;
    unsigned char *optiph;
    int l;
    const struct ipt_ipv4optsstrip_target_info *info = par->targinfo;

    if (skb_ensure_writable(skb, skb->len))
	return NF_DROP;

    iph = ip_hdr(skb);
    optiph = skb_network_header(skb);
    l = ((struct ip_options *)(&(IPCB(skb)->opt)))->optlen;

    /* if no options in packet then nothing to clear. */
    if (iph->ihl * 4 == sizeof(struct iphdr))
	return XT_CONTINUE;

    /* else clear all options */
    memset(&(IPCB(skb)->opt), 0, sizeof(struct ip_options));
    memset(optiph+sizeof(struct iphdr), IPOPT_NOOP, l);
    iph->check = 0;
    iph->check = ip_fast_csum((unsigned char *)iph, iph->ihl);
    opt = &(IPCB(skb)->opt);
    opt->optlen = l;

    return info->passthrough ? XT_CONTINUE : NF_ACCEPT;
}

static int ipv4optstrip_checkentry(const struct xt_tgchk_param *par) {
    if (strcmp(par->table, "mangle")) {
	printk(KERN_WARNING "IPV4OPTSSTRIP: can only be called from \"mangle\" table, not \"%s\"\n", par->table);
	return -EINVAL;
    }
    /* nothing else to check because no parameters */
    return 0;
}

static struct xt_target xt_ipv4optsstrip_target = {
    .name = "IPV4OPTSSTRIP",
    .family = AF_INET,
    .target = ipv4optstrip_target,
    .targetsize = sizeof(struct ipt_ipv4optsstrip_target_info),
    .checkentry = ipv4optstrip_checkentry,
    .me = THIS_MODULE
};


/***   iprange   ***************************************/

static bool iprange_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    const struct ipt_iprange_info *info = par->matchinfo;
    const struct iphdr *iph = ip_hdr(skb);

    if (info->flags & IPRANGE_SRC) {
	if ((ntohl(iph->saddr) < ntohl(info->src.min_ip)
			|| ntohl(iph->saddr) > ntohl(info->src.max_ip))
		^ !!(info->flags & IPRANGE_SRC_INV)) {
	    pr_debug("src IP %pI4 NOT in range %s"
		    "%pI4-%pI4\n",
		    &iph->saddr,
		    info->flags & IPRANGE_SRC_INV ? "(INV) " : "",
		    &info->src.min_ip,
		    &info->src.max_ip);
	    return false;
	}
    }
    if (info->flags & IPRANGE_DST) {
	if ((ntohl(iph->daddr) < ntohl(info->dst.min_ip)
			|| ntohl(iph->daddr) > ntohl(info->dst.max_ip))
		^ !!(info->flags & IPRANGE_DST_INV)) {
	    pr_debug("dst IP %pI4 NOT in range %s"
		    "%pI4-%pI4\n",
		    &iph->daddr,
		    info->flags & IPRANGE_DST_INV ? "(INV) " : "",
		    &info->dst.min_ip,
		    &info->dst.max_ip);
	    return false;
	}
    }
    return true;
}

static struct xt_match xt_iprange_match __read_mostly = {
    .name = "iprange",
    .family = AF_INET,
    .match = iprange_match,
    .matchsize = sizeof(struct ipt_iprange_info),
    .me = THIS_MODULE
};


/***   cleardf   ***************************************/

static unsigned int cleardf_target(struct sk_buff *skb,
	const struct xt_action_param *par) {
    struct iphdr *iph;
    const struct ipt_cleardf_target_info *info = par->targinfo;
    int df;

    if (skb_ensure_writable(skb, skb->len))
	return NF_DROP;

    iph = ip_hdr(skb);

    df = ntohs(iph->frag_off) & IP_DF;

    if (df) {
	csum_replace2(&iph->check,
		      iph->frag_off, iph->frag_off & ~htons(IP_DF));
	iph->frag_off &= ~htons(IP_DF);
    }

    return info->passthrough ? XT_CONTINUE : NF_ACCEPT;
}

static struct xt_target xt_cleardf_target __read_mostly = {
    .name = "CLEARDF",
    .family = AF_INET,
    .target = cleardf_target,
    .targetsize = sizeof(struct ipt_cleardf_target_info),
    .me = THIS_MODULE,
};

/***   passthrough   ***************************************/

static unsigned passthrough_target(struct sk_buff *skb,
	const struct xt_action_param *par) {
    return XT_CONTINUE;
}

static struct xt_target xt_passthrough_target = {
    .name = "PASSTHR",
    .target = passthrough_target,
    .targetsize = sizeof(struct ipt_passthr_info),
    .me = THIS_MODULE,
};


/***   pcc   ***************************************/

static unsigned initval;

static bool pcc_match(const struct sk_buff *skb,
		  struct xt_action_param *par) {
    const struct xt_pcc_info *info = par->matchinfo;
    u8 nexthdr;
    unsigned char key[40];
    unsigned key_len = 0;
    unsigned result;
    unsigned protoff = par->thoff;

    switch (xt_family(par)) {
    case AF_INET: {
	const struct iphdr *iph = ip_hdr(skb);
	if (!iph) {
	    return info->invert;
	}
	if (info->values_to_hash & XT_PPC_HASH_SRC_ADDR) {
	    memcpy(key + key_len, &iph->saddr, sizeof(iph->saddr));
	    key_len += sizeof(iph->saddr);
//	    printk("add to hash ipv4 src addr %pI4\n", &iph->saddr);
	}
	if (info->values_to_hash & XT_PPC_HASH_DST_ADDR) {
	    memcpy(key + key_len, &iph->daddr, sizeof(iph->daddr));
	    key_len += sizeof(iph->daddr);
//	    printk("add to hash ipv4 dst addr %pI4\n", &iph->daddr);
	}
	nexthdr = iph->protocol;
	break;
    }
    case AF_INET6: {
	const struct ipv6hdr *iph = ipv6_hdr(skb);
	__be16 frag_off;

	if (!iph) {
	    return info->invert;
	}
	if (info->values_to_hash & XT_PPC_HASH_SRC_ADDR) {
	    memcpy(key + key_len, &iph->saddr, sizeof(iph->saddr));
	    key_len += sizeof(iph->saddr);
	}
	if (info->values_to_hash & XT_PPC_HASH_DST_ADDR) {
	    memcpy(key + key_len, &iph->daddr, sizeof(iph->daddr));
	    key_len += sizeof(iph->daddr);
	}
	nexthdr = ipv6_hdr(skb)->nexthdr;
	protoff = ipv6_skip_exthdr(skb, sizeof(struct ipv6hdr), &nexthdr, &frag_off);
	if ((int)protoff < 0) {
	    par->hotdrop = true;
	    return false;
	}
	break;
    }
    default:
	return info->invert;
    }

    switch (nexthdr) {
    case IPPROTO_TCP:
    case IPPROTO_UDP:
    case IPPROTO_UDPLITE:
    case IPPROTO_SCTP:
    case IPPROTO_DCCP: {
	__be16 _ports[2];
	__be16 *ports;

	ports = skb_header_pointer(skb, protoff, sizeof(_ports), _ports);
	if (ports == NULL) {
	    par->hotdrop = true;
	    return false;
	}
	if (info->values_to_hash & XT_PPC_HASH_SRC_PORT) {
	    memcpy(key + key_len, &ports[0], sizeof(__be16));
	    key_len += sizeof(__be16);
//	    printk("add to hash %d src port %d\n", nexthdr, ntohs(ports[0]));
	}
	if (info->values_to_hash & XT_PPC_HASH_DST_PORT) {
	    memcpy(key + key_len, &ports[1], sizeof(__be16));
	    key_len += sizeof(__be16);
//	    printk("add to hash %d dst port %d\n", nexthdr, ntohs(ports[1]));
	}
	break;
    }
    default:
	break;
    }

    result = jhash(key, key_len, initval) % info->denominator;
//    printk("key_len %d, hash %d, rem %d\n", key_len, result, info->remainder);
    return (result == info->remainder) ^ info->invert;
}

static struct xt_match xt_pcc_match = {
    .name		= "per_connection_classifier",
    .match		= pcc_match,
    .matchsize	= sizeof(struct xt_pcc_info),
    .me		= THIS_MODULE
};


/***   priority   ***************************************/

#define DSCP_SHIFT 2

static bool priority_ingress_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    const struct xt_priority_info *info = par->matchinfo;
    return (skb->ingress_priority == info->prio) ^ info->invert;
}

static bool priority_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    const struct xt_priority_info *info = par->matchinfo;
    return (skb->priority == info->prio) ^ info->invert;
}

static struct xt_match xt_priority_match[] = {
    {
	.name		= "ingress_priority",
	.match		= priority_ingress_match,
	.matchsize	= sizeof(struct xt_priority_info),
	.me		= THIS_MODULE
    },
    {
	.name		= "priority",
	.match		= priority_match,
	.matchsize	= sizeof(struct xt_priority_info),
	.me		= THIS_MODULE,
    },
};


static unsigned int priority_target(struct sk_buff *skb,
			   const struct xt_action_param *par) {
    const struct xt_setprio_target_info *info = par->targinfo;

    switch (info->prio) {
    case PRIO_DSCP:
	skb->priority = ipv4_get_dsfield(ip_hdr(skb)) >> DSCP_SHIFT;
	break;
    case PRIO_INGRESS:
	skb->priority = skb->ingress_priority;
	break;
    case PRIO_DSCP_HIGH_3_BITS:
	skb->priority = ipv4_get_dsfield(ip_hdr(skb)) >> (DSCP_SHIFT + 3);
	break;
    default:
	skb->priority = info->prio;
	break;
    }
    if (xt_family(par) == NFPROTO_BRIDGE) {
	return info->passthrough ? EBT_CONTINUE : EBT_ACCEPT;
    }
    return info->passthrough ? XT_CONTINUE : NF_ACCEPT;
}

static struct xt_target xt_priority_targets[] = {
    {
	.name = "SETPRIO",
	.target = priority_target,
	.targetsize = sizeof(struct xt_setprio_target_info),
	.me = THIS_MODULE,
    },
    {
	.name = "ebt_SETPRIO",
	.family = NFPROTO_BRIDGE,
	.target = priority_target,
	.targetsize = sizeof(struct xt_setprio_target_info),
	.me = THIS_MODULE,
    },
};


/***   connrate   ***************************************/

static unsigned read_rate(const struct nf_conn_counter *acct) {
    unsigned ret = 0;
    if (time_before(jiffies, acct->second_end_jiffies + HZ)) {
	ret = atomic_read(&acct->bytes_prev_second);
    }
    return ret;
}

static bool connrate_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    const struct xt_connrate_info *info = par->matchinfo;
    const struct nf_conn *ct;
    enum ip_conntrack_info ctinfo;
    unsigned rate = 0;
    const struct nf_conn_acct *acct;

    ct = nf_ct_get(skb, &ctinfo);
    if (!ct) {
	return false;
    }

    acct = nf_conn_acct_find(ct);
    if (!acct) {
	return false;
    }

    switch (info->direction) {
    case XT_CONNRATE_DIR_ORIGINAL:
	rate = read_rate(&acct->counter[IP_CT_DIR_ORIGINAL]);
	break;
    case XT_CONNRATE_DIR_REPLY:
	rate = read_rate(&acct->counter[IP_CT_DIR_REPLY]);
	break;
    case XT_CONNRATE_DIR_BOTH:
	rate = read_rate(&acct->counter[IP_CT_DIR_ORIGINAL]);
	rate += read_rate(&acct->counter[IP_CT_DIR_REPLY]);
	break;
    }

/*
    printk("rate1 %d %d %lld\n",
	   acct->counter[IP_CT_DIR_ORIGINAL].bytes_prev_second,
	   acct->counter[IP_CT_DIR_ORIGINAL].bytes_this_second,
	   acct->counter[IP_CT_DIR_ORIGINAL].bytes);
    printk("rate2 %d %d %lld\n",
	   acct->counter[IP_CT_DIR_REPLY].bytes_prev_second,
	   acct->counter[IP_CT_DIR_REPLY].bytes_this_second,
	   acct->counter[IP_CT_DIR_REPLY].bytes);
    printk("rate %d %d %d\n", rate, info->from, info->to);
*/
    if (info->to) {
	return (info->from <= rate && rate <= info->to) ^ info->invert;
    }
    else {
	return (info->from <= rate) ^ info->invert;
    }
}

static int connrate_check(const struct xt_mtchk_param *par) {
    const struct xt_connrate_info *info = par->matchinfo;

    if (info->direction != XT_CONNRATE_DIR_ORIGINAL &&
	info->direction != XT_CONNRATE_DIR_REPLY &&
	info->direction != XT_CONNRATE_DIR_BOTH)
	return -EINVAL;

    return 0;
}

static struct xt_match xt_connrate_match __read_mostly = {
    .name = "connrate",
    .checkentry = connrate_check,
    .match = connrate_match,
    .matchsize = sizeof(struct xt_connrate_info),
    .me = THIS_MODULE
};

/***   limit   ***************************************/

struct xt_limit_priv {
    atomic_t current_tokens;
    spinlock_t replenish_lock;
    unsigned last_replenish_jiffies;
    unsigned long long ftokens_per_jiffy;
    unsigned long long ftokens;
    unsigned jiffies_in_burst;
};

#define FTOKENS_SHIFT 32

static void limit_replenish(const struct xt_limit_info *i, struct xt_limit_priv *priv) {
    unsigned now = jiffies;
    unsigned diff;
    unsigned add_tokens;
    if ((int)(now - priv->last_replenish_jiffies) <= 0) {
	return;
    }
    spin_lock_bh(&priv->replenish_lock);
    if ((int)(now - priv->last_replenish_jiffies) <= 0) {
	spin_unlock_bh(&priv->replenish_lock);
	return;
    }

    diff = now - priv->last_replenish_jiffies;
    if (diff > priv->jiffies_in_burst) {
	// this is to prevent ftokens overflow
	diff = priv->jiffies_in_burst;
    }
    priv->last_replenish_jiffies = now;
    priv->ftokens += (unsigned long long)diff * priv->ftokens_per_jiffy;

    add_tokens = priv->ftokens >> FTOKENS_SHIFT;
    if (add_tokens) {
//	printk("add tok: %u %u %llu %u\n", diff, add_tokens, priv->ftokens, atomic_read(&priv->current_tokens));
	int ct = atomic_add_return(add_tokens, &priv->current_tokens);
	if (ct > (int)i->burst) {
	    // remove anything over burst
	    atomic_sub(ct - i->burst, &priv->current_tokens);
	}
	priv->ftokens &= (1llu << FTOKENS_SHIFT) - 1;
    }
    spin_unlock_bh(&priv->replenish_lock);
}

static bool limit_match(const struct sk_buff *skb, struct xt_action_param *par) {
    const struct xt_limit_info *i = par->matchinfo;
    struct xt_limit_priv *priv = i->priv;
    unsigned x = 1;
    int new_tokens;

    limit_replenish(i, priv);

    if (i->mode == XT_LIMIT_BYTE) {
	x = skb->len;
    }

    new_tokens = atomic_sub_return(x, &priv->current_tokens);
    if (new_tokens < 0) {
	atomic_add(x, &priv->current_tokens);
	return i->invert;
    }
    return !i->invert;
}

static int limit_check(const struct xt_mtchk_param *par) {
    struct xt_limit_info *i = par->matchinfo;
    struct xt_limit_priv *priv =
	kmalloc(sizeof(struct xt_limit_priv), GFP_KERNEL);
    unsigned temp;
    if (!priv) {
	return -ENOMEM;
    }
    i->priv = priv;
    if (i->mode == XT_LIMIT_BYTE) {
	do_div(i->count, 8);
	i->burst /= 8;
    }
    atomic_set(&priv->current_tokens, i->burst);
    spin_lock_init(&priv->replenish_lock);
    priv->last_replenish_jiffies = jiffies;
    priv->ftokens_per_jiffy = i->count << FTOKENS_SHIFT;
    if (!i->time) {
	i->time = 1;
    }
    do_div(priv->ftokens_per_jiffy, i->time * HZ);
    priv->ftokens = 0;

    temp = priv->ftokens_per_jiffy >> FTOKENS_SHIFT;
    if (!temp) {
	temp = 1;
    }
    priv->jiffies_in_burst = 2000000000 / temp;
    if (!priv->jiffies_in_burst) {
	priv->jiffies_in_burst = 1;
    }

//    printk("limit match: count:%llu time:%u burst:%u mode:%u ftokens_per_jiffy:%llu jiffies_in_burst:%u\n",
//	    i->count, i->time, i->burst, i->mode, priv->ftokens_per_jiffy, priv->jiffies_in_burst);
    return 0;
}

static void limit_destroy(const struct xt_mtdtor_param *par) {
    struct xt_limit_info *i = par->matchinfo;
    kfree(i->priv);
}

static struct xt_match xt_limit_match __read_mostly = {
    .name             = "limit",
    .revision         = 0,
    .match            = limit_match,
    .checkentry       = limit_check,
    .destroy          = limit_destroy,
    .matchsize        = sizeof(struct xt_limit_info),
    .me               = THIS_MODULE,
};


/***   devlist   ***************************************/

static bool devlist_match(const struct sk_buff *skb, struct xt_action_param *par) {
    const struct xt_devlist_info *i = par->matchinfo;
    const struct net_device *dev;

    if (i->flags & XT_DEVLIST_PHYS) {
	const struct nf_bridge_info *nf_bridge = nf_bridge_info_get(skb);
//	printk(" nfbr:%p\n", nf_bridge);
	if (!nf_bridge) {
	    return !!(i->flags & XT_DEVLIST_INVERT);
	}
//	printk(" physoutdev:%s\n", nf_bridge->physoutdev ? nf_bridge->physoutdev->name : "NULL");
	if (i->flags & XT_DEVLIST_OUT) {
	    dev = nf_bridge->physoutdev;
	}
	else {
	    dev = nf_bridge->physindev;
	}
    }
    else {
	if (i->flags & XT_DEVLIST_OUT) {
	    dev = xt_out(par);
	}
	else {
	    dev = xt_in(par);
	}
    }
//    printk("devlist match %p %s\n", i, dev ? dev->name : "NULL");
    if (!dev) {
	return !!(i->flags & XT_DEVLIST_INVERT);
    }

    return netdev_list_match(dev->list_bitmap, i->list_bitmap) ^ !!(i->flags & XT_DEVLIST_INVERT);
}

static bool ebt_devlist_match(const struct sk_buff *skb, struct xt_action_param *par) {
    const struct xt_devlist_info *i = par->matchinfo;
    const struct net_device *dev;

    if (i->flags & XT_DEVLIST_OUT) {
	dev = xt_out(par);
    }
    else {
	dev = xt_in(par);
    }
    if (!dev) {
	return !!(i->flags & XT_DEVLIST_INVERT);
    }
    if (!(i->flags & XT_DEVLIST_PHYS)) {
	const struct net_bridge_port *p = br_port_get_rcu(dev);
	if (!p) {
	    return !!(i->flags & XT_DEVLIST_INVERT);
	}
	dev = p->br->dev;
    }

    return netdev_list_match(dev->list_bitmap, i->list_bitmap) ^ !!(i->flags & XT_DEVLIST_INVERT);
}

static int devlist_check(const struct xt_mtchk_param *par) {
/*
    struct xt_devlist_info *i = par->matchinfo;
    int k;
    printk("devlist check match %p %x ", i, i->flags);
    for (k = 0; k < DEV_LIST_BITMAP_WORDS; ++k) {
	printk("%08x ", i->list_bitmap[k]);
    }
    printk("\n");
*/
    return 0;
}

static struct xt_match xt_devlist_match[] __read_mostly = {
    {
	.name             = "devlist",
	.match            = devlist_match,
	.checkentry       = devlist_check,
	.matchsize        = sizeof(struct xt_devlist_info),
	.me               = THIS_MODULE,
    },
    {
	.name             = "ebt_devlist",
	.match            = ebt_devlist_match,
	.checkentry       = devlist_check,
	.matchsize        = sizeof(struct xt_devlist_info),
	.me               = THIS_MODULE,
    },
};


/***   addrlist   ***************************************/

static unsigned addrlist_target(struct sk_buff *skb,
	const struct xt_action_param *par) {
    const struct ipt_addrlist_target_info *info = par->targinfo;
    struct addrlist_entry *entry;
    unsigned ip[4] = { info->put_what == ADDRLIST_SRC ?
	ip_hdr(skb)->saddr : ip_hdr(skb)->daddr, 0, 0, 0 };

    write_lock_bh(&info->list->lock);
    entry = addrlist_lookup(info->list, ip);
    if (!entry) {
	entry = addrlist_put_in_list(info->list, ip, ip, info->timeout, true);
	if (!entry) {
//	    printk("addrlist: target failure\n");
	    write_unlock_bh(&info->list->lock);
	    return XT_CONTINUE;
	}
    }
    else {
	addrlist_update(entry, info->timeout);
    }
    write_unlock_bh(&info->list->lock);

    return XT_CONTINUE;
}

static unsigned addrlist6_target(struct sk_buff *skb,
	const struct xt_action_param *par) {
    const struct ipt_addrlist_target_info *info = par->targinfo;
    struct addrlist_entry *entry;
    unsigned ip[4];
    memcpy(ip, info->put_what == ADDRLIST_SRC ?
	    &ipv6_hdr(skb)->saddr : &ipv6_hdr(skb)->daddr, sizeof(ip));

    write_lock_bh(&info->list->lock);
    entry = addrlist_lookup(info->list, ip);
    if (!entry) {
	entry = addrlist_put_in_list(info->list, ip, ip, info->timeout, true);
	if (!entry) {
//	    printk("addrlist: target failure\n");
	    write_unlock_bh(&info->list->lock);
	    return XT_CONTINUE;
	}
    }
    else {
	addrlist_update(entry, info->timeout);
    }
    write_unlock_bh(&info->list->lock);

    return XT_CONTINUE;
}

static int addrlist_target_checkentry(const struct xt_tgchk_param *par) {
    struct ipt_addrlist_target_info *info = par->targinfo;
    info->list = addrlist_get_list(info->list_id, info->type);
    if (!info->list) {
	printk("addrlist: target cannot get addrlist with type:%u id:%u\n",
		info->type, info->list_id);
	return -EINVAL;
    }
    return 0;
}

static void addrlist_target_destroy(const struct xt_tgdtor_param *par) {
    const struct ipt_addrlist_target_info *info = par->targinfo;
    if (info->list_id) addrlist_put_list(info->list);
}


static bool addrlist_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    const struct ipt_addrlist_info *info = par->matchinfo;
    int r1 = 1, r2 = 1;

    if (info->list_src) {
	unsigned ip[4] = { ip_hdr(skb)->saddr, 0, 0, 0 };
	read_lock_bh(&info->list_src->lock);
	r1 = !!addrlist_lookup(info->list_src, ip) ^
	    !!(info->invert & (1 << ADDRLIST_SRC));
	read_unlock_bh(&info->list_src->lock);
    }

    if (info->list_dst) {
	unsigned ip[4] = { ip_hdr(skb)->daddr, 0, 0, 0 };
	read_lock_bh(&info->list_dst->lock);
	r2 = !!addrlist_lookup(info->list_dst, ip) ^
	    !!(info->invert & (1 << ADDRLIST_DST));
	read_unlock_bh(&info->list_dst->lock);
    }
//    printk("r1=%d r2=%d\n", r1, r2);
    return r1 && r2;
}

static bool addrlist6_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    const struct ipt_addrlist_info *info = par->matchinfo;
    int r1 = 1, r2 = 1;
    unsigned ip[4];

    if (info->list_src) {
	memcpy(ip, &ipv6_hdr(skb)->saddr, sizeof(ip));
	read_lock_bh(&info->list_src->lock);
	r1 = !!addrlist_lookup(info->list_src, ip) ^
	    !!(info->invert & (1 << ADDRLIST_SRC));
	read_unlock_bh(&info->list_src->lock);
    }

    if (info->list_dst) {
	memcpy(ip, &ipv6_hdr(skb)->daddr, sizeof(ip));
	read_lock_bh(&info->list_dst->lock);
	r2 = !!addrlist_lookup(info->list_dst, ip) ^
	    !!(info->invert & (1 << ADDRLIST_DST));
	read_unlock_bh(&info->list_dst->lock);
    }
//    printk("addrlist6: match r1=%d r2=%d\n", r1, r2);
    return r1 && r2;
}

static int addrlist_match_checkentry(const struct xt_mtchk_param *par) {
    struct ipt_addrlist_info *info = par->matchinfo;
    if (info->list_src_id) {
//	printk("list_src_id: %u\n", info->list_src_id);
	info->list_src = addrlist_get_list(info->list_src_id, info->type);
	if (!info->list_src) {
	    printk("addrlist: match cannot get src addrlist with id %u\n",
		    info->list_src_id);
	    return -EINVAL;
	}
    }
    else {
//	printk("list_src_id is NULL\n");
	info->list_src = NULL;
    }

    if (info->list_dst_id) {
	info->list_dst = addrlist_get_list(info->list_dst_id, info->type);
	if (!info->list_dst) {
	    printk("addrlist: match cannot get dst addrlist with id %u\n",
		    info->list_dst_id);
	    return -EINVAL;
	}
    }
    else {
	info->list_dst = NULL;
    }

    return 0;
}

static void addrlist_match_destroy(const struct xt_mtdtor_param *par) {
    const struct ipt_addrlist_info *info = par->matchinfo;
    if (info->list_src) addrlist_put_list(info->list_src);
    if (info->list_dst) addrlist_put_list(info->list_dst);
}


static struct xt_target addrlist_targets[] = {
    {
	.name = "ADDRLIST",
	.family = AF_INET,
	.target = addrlist_target,
	.targetsize = sizeof(struct ipt_addrlist_target_info),
	.checkentry = addrlist_target_checkentry,
	.destroy = addrlist_target_destroy,
	.me = THIS_MODULE,
    },
    {
	.name = "ADDRLIST",
	.family = AF_INET6,
	.target = addrlist6_target,
	.targetsize = sizeof(struct ipt_addrlist_target_info),
	.checkentry = addrlist_target_checkentry,
	.destroy = addrlist_target_destroy,
	.me = THIS_MODULE,
    },
};

static struct xt_match addrlist_matches[] = {
    {
	.name = "addrlist",
	.family = AF_INET,
	.match = addrlist_match,
	.matchsize = sizeof(struct ipt_addrlist_info),
	.checkentry = addrlist_match_checkentry,
	.destroy = addrlist_match_destroy,
	.me = THIS_MODULE,
    },
    {
	.name = "addrlist",
	.family = AF_INET6,
	.match = addrlist6_match,
	.matchsize = sizeof(struct ipt_addrlist_info),
	.checkentry = addrlist_match_checkentry,
	.destroy = addrlist_match_destroy,
	.me = THIS_MODULE,
    },
};


/***   connlimit   ***************************************/

struct connlimit_net {
    struct rb_node node;
    union nf_inet_addr addr;
    unsigned count;
};

struct connlimit_conn {
    struct rb_node node;
    struct nf_conn *conn;
    struct connlimit_net *net;
};

struct connlimit_state {
    struct list_head node;
    spinlock_t lock;
    struct rb_root conn_root;
    struct rb_root net_root;
    struct rcu_head rcu_head;
};

static LIST_HEAD(connlimit_states);

extern void (*connlimit_destroy_conntrack)(struct nf_conn *);
static void connlimit_conn_destr(struct nf_conn *conn) {
    struct connlimit_state *state;
    struct connlimit_net *n;
    struct connlimit_conn *c;
    struct rb_node *node;
    rcu_read_lock_bh();
    list_for_each_entry_rcu(state, &connlimit_states, node) {
	spin_lock(&state->lock);

	node = state->conn_root.rb_node;
	while (node) {
	    c = container_of(node, struct connlimit_conn, node);
	    if (conn < c->conn) {
		node = node->rb_left;
		continue;
	    }
	    else if (conn > c->conn) {
		node = node->rb_right;
		continue;
	    }
	    n = c->net;
	    rb_erase(&c->node, &state->conn_root);
	    kfree(c);

	    --n->count;
	    if (!n->count) {
		rb_erase(&n->node, &state->net_root);
		kfree(n);
	    }
	    break;
	}
	spin_unlock(&state->lock);
    }
    rcu_read_unlock_bh();
}

static bool connlimit_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    const struct xt_connlimit_info *info = par->matchinfo;
    struct connlimit_state *state = (struct connlimit_state *)info->data;
    struct connlimit_net *n = NULL;
    struct connlimit_conn *c;
    struct rb_node **node;
    struct rb_node *parent = NULL;
    long diff;
    int connections;

    union nf_inet_addr addr = {};
    enum ip_conntrack_info ctinfo;
    struct nf_conn *ct = nf_ct_get(skb, &ctinfo);
    if (!ct) {
	return false;
    }

    if (xt_family(par) == NFPROTO_IPV6) {
	unsigned i;
	const struct ipv6hdr *iph = ipv6_hdr(skb);
	memcpy(&addr.ip6, (info->flags & XT_CONNLIMIT_DADDR) ?
		&iph->daddr : &iph->saddr, sizeof(addr.ip6));

	for (i = 0; i < ARRAY_SIZE(addr.ip6); ++i) {
	    addr.ip6[i] &= info->mask.ip6[i];
	}
    } else {
	const struct iphdr *iph = ip_hdr(skb);
	addr.ip = (info->flags & XT_CONNLIMIT_DADDR) ?
	    iph->daddr : iph->saddr;
	addr.ip &= info->mask.ip;
    }

    spin_lock_bh(&state->lock);

    node = &state->net_root.rb_node;
    while (*node) {
	n = container_of(*node, struct connlimit_net, node);
	parent = *node;
	diff = memcmp(&addr, &n->addr, sizeof(addr));
	if (diff < 0) {
	    node = &((*node)->rb_left);
	    continue;
	}
	else if (diff > 0) {
	    node = &((*node)->rb_right);
	    continue;
	}
	break;
    }
    if (!*node) {
	n = kzalloc(sizeof(struct connlimit_net), GFP_ATOMIC);
	if (!n) {
	    spin_unlock_bh(&state->lock);
	    return !!(info->flags & XT_CONNLIMIT_INVERT);
	}
	n->addr = addr;
	rb_link_node(&n->node, parent, node);
	rb_insert_color(&n->node, &state->net_root);
    }

    parent = NULL;
    node = &state->conn_root.rb_node;
    while (*node) {
	c = container_of(*node, struct connlimit_conn, node);
	parent = *node;
	if (ct < c->conn) {
	    node = &((*node)->rb_left);
	    continue;
	}
	else if (ct > c->conn) {
	    node = &((*node)->rb_right);
	    continue;
	}
	break;
    }
    if (!*node) {
	c = kzalloc(sizeof(struct connlimit_conn), GFP_ATOMIC);
	if (c) {
	    c->conn = ct;
	    c->net = n;
	    rb_link_node(&c->node, parent, node);
	    rb_insert_color(&c->node, &state->conn_root);
	    ++n->count;
	}
    }
    connections = n->count;
    spin_unlock_bh(&state->lock);

    return (connections > info->limit) ^ !!(info->flags & XT_CONNLIMIT_INVERT);
}

static int connlimit_check(const struct xt_mtchk_param *par) {
    struct xt_connlimit_info *info = par->matchinfo;
    struct connlimit_state *state =
	kzalloc(sizeof(struct connlimit_state), GFP_KERNEL);
    if (!state) {
	return -ENOMEM;
    }
    spin_lock_init(&state->lock);
    state->conn_root = RB_ROOT;
    state->net_root = RB_ROOT;
    if (list_empty(&connlimit_states)) {
	connlimit_destroy_conntrack = connlimit_conn_destr;
    }
    list_add_rcu(&state->node, &connlimit_states);

    info->data = (void *)state;
    return 0;
}

static void connlimit_destroy(const struct xt_mtdtor_param *par) {
    const struct xt_connlimit_info *info = par->matchinfo;
    struct connlimit_state *state = (struct connlimit_state *)info->data;
    struct connlimit_net *n;
    struct connlimit_conn *c;

    list_del_rcu(&state->node);
    if (list_empty(&connlimit_states)) {
	connlimit_destroy_conntrack = NULL;
    }
    spin_lock_bh(&state->lock);

    while (state->conn_root.rb_node) {
	c = container_of(state->conn_root.rb_node, struct connlimit_conn, node);
	rb_erase(&c->node, &state->conn_root);
	kfree(c);
    }
    while (state->net_root.rb_node) {
	n = container_of(state->net_root.rb_node, struct connlimit_net, node);
	rb_erase(&n->node, &state->net_root);
	kfree(n);
    }

    spin_unlock_bh(&state->lock);
    kfree_rcu(state, rcu_head);
}

static struct xt_match xt_connlimit_match __read_mostly = {
    .name = "connlimit",
    .checkentry = connlimit_check,
    .match = connlimit_match,
    .matchsize = sizeof(struct xt_connlimit_info),
    .destroy = connlimit_destroy,
    .me = THIS_MODULE,
};


static int __init xt_misc_init(void) {
    get_random_bytes(&initval, sizeof(initval));
    xt_register_target(&xt_route_target);
    xt_register_target(&xt_fastpathconn_target);
    xt_register_match(&xt_ipv4options_match);
    xt_register_target(&xt_ipv4optsstrip_target);
    xt_register_match(&xt_iprange_match);
    xt_register_target(&xt_cleardf_target);
    xt_register_target(&xt_passthrough_target);
    xt_register_match(&xt_pcc_match);
    xt_register_matches(xt_priority_match, ARRAY_SIZE(xt_priority_match));
    xt_register_targets(xt_priority_targets, ARRAY_SIZE(xt_priority_targets));
    xt_register_match(&xt_connrate_match);
    xt_register_match(&xt_limit_match);
    xt_register_matches(xt_devlist_match, ARRAY_SIZE(xt_devlist_match));
    xt_register_targets(addrlist_targets, ARRAY_SIZE(addrlist_targets));
    xt_register_matches(addrlist_matches, ARRAY_SIZE(addrlist_matches));
    xt_register_match(&xt_connlimit_match);
    return 0;
}

static void __exit xt_misc_exit(void) {
    xt_unregister_target(&xt_route_target);
    xt_unregister_target(&xt_fastpathconn_target);
    xt_unregister_match(&xt_ipv4options_match);
    xt_unregister_target(&xt_ipv4optsstrip_target);
    xt_unregister_match(&xt_iprange_match);
    xt_unregister_target(&xt_cleardf_target);
    xt_unregister_target(&xt_passthrough_target);
    xt_unregister_match(&xt_pcc_match);
    xt_unregister_matches(xt_priority_match, ARRAY_SIZE(xt_priority_match));
    xt_unregister_targets(xt_priority_targets, ARRAY_SIZE(xt_priority_targets));
    xt_unregister_match(&xt_connrate_match);
    xt_unregister_match(&xt_limit_match);
    xt_unregister_matches(xt_devlist_match, ARRAY_SIZE(xt_devlist_match));
    xt_unregister_matches(addrlist_matches, ARRAY_SIZE(addrlist_matches));
    xt_unregister_targets(addrlist_targets, ARRAY_SIZE(addrlist_targets));
    xt_unregister_match(&xt_connlimit_match);
}

module_init(xt_misc_init);
module_exit(xt_misc_exit);
