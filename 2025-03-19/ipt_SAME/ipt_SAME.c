/* Same.  Just like SNAT, only try to make the connections
 * 	  between client A and server B always have the same source ip.
 *
 * (C) 2000 Paul `Rusty' Russell
 * (C) 2001 Martin Josefsson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/ip.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/netfilter.h>
#include <linux/netdevice.h>
#include <linux/if.h>
#include <linux/inetdevice.h>
#include <net/protocol.h>
#include <net/checksum.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter/x_tables.h>
#include <net/netfilter/nf_nat.h>
#include "ipt_SAME.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin Josefsson <gandalf@wlug.westbo.se>");
MODULE_DESCRIPTION("iptables special SNAT module for consistent sourceip");

static int same_check(const struct xt_tgchk_param *par)
{
	unsigned int count, countess, rangeip, index = 0;
	struct ipt_same_info *mr = par->targinfo;

	mr->ipnum = 0;

	if (mr->rangesize < 1) {
		pr_debug("same_check: need at least one dest range.\n");
		return -EINVAL;
	}
	if (mr->rangesize > IPT_SAME_MAX_RANGE) {
		pr_debug("same_check: too many ranges specified, maximum "
			 "is %u ranges\n", IPT_SAME_MAX_RANGE);
		return -EINVAL;
	}
	for (count = 0; count < mr->rangesize; count++) {
		if (ntohl(mr->range[count].min_ip) >
				ntohl(mr->range[count].max_ip)) {
			pr_debug("same_check: min_ip is larger than max_ip in "
				 "range `%pI4-%pI4'.\n",
				 &mr->range[count].min_ip,
				 &mr->range[count].max_ip);
			return -EINVAL;
		}
		if (!(mr->range[count].flags & NF_NAT_RANGE_MAP_IPS)) {
			pr_debug("same_check: bad MAP_IPS.\n");
			return -EINVAL;
		}
		rangeip = (ntohl(mr->range[count].max_ip) -
					ntohl(mr->range[count].min_ip) + 1);
		mr->ipnum += rangeip;

		pr_debug("same_check: range %u, ipnum = %u\n", count, rangeip);
	}
	pr_debug("same_check: total ipaddresses = %u\n", mr->ipnum);

	mr->iparray = kmalloc((sizeof(u_int32_t) * mr->ipnum), GFP_KERNEL);
	if (!mr->iparray) {
		pr_debug("same_check: Couldn't allocate %Zu bytes "
			 "for %u ipaddresses!\n",
			 (sizeof(u_int32_t) * mr->ipnum), mr->ipnum);
		return -EINVAL;
	}
	pr_debug("same_check: Allocated %Zu bytes for %u ipaddresses.\n",
		 (sizeof(u_int32_t) * mr->ipnum), mr->ipnum);

	for (count = 0; count < mr->rangesize; count++) {
		for (countess = ntohl(mr->range[count].min_ip);
				countess <= ntohl(mr->range[count].max_ip);
					countess++) {
			mr->iparray[index] = countess;
#if 0
			pr_debug("same_check: Added ipaddress `%u.%u.%u.%u' "
				 "in index %u.\n", HIPQUAD(countess), index);
#endif
			index++;
		}
	}
	return 0;
}

static void same_destroy(const struct xt_tgdtor_param *par)
{
	struct ipt_same_info *mr = par->targinfo;

	kfree(mr->iparray);

	pr_debug("same_destroy: Deallocated %Zu bytes for %u ipaddresses.\n",
		 (sizeof(u_int32_t) * mr->ipnum), mr->ipnum);
}

static unsigned int same_target(struct sk_buff *skb,
				const struct xt_action_param *par)
{
	struct nf_conn *ct;
	enum ip_conntrack_info ctinfo;
	u_int32_t tmpip, aindex;
	__be32 new_ip;
	const struct ipt_same_info *same = par->targinfo;
	struct nf_nat_range2 newrange = {};
	const struct nf_conntrack_tuple *t;

	WARN_ON(xt_hooknum(par) != NF_INET_PRE_ROUTING &&
		xt_hooknum(par) != NF_INET_POST_ROUTING);
	ct = nf_ct_get(skb, &ctinfo);

	t = &ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple;

	/* Base new source on real src ip and optionally dst ip,
	   giving some hope for consistency across reboots.
	   Here we calculate the index in same->iparray which
	   holds the ipaddress we should use */

	tmpip = ntohl(t->src.u3.ip);

	if (!(same->info & IPT_SAME_NODST))
		tmpip += ntohl(t->dst.u3.ip);
	aindex = tmpip % same->ipnum;

	new_ip = htonl(same->iparray[aindex]);

	pr_debug("ipt_SAME: src=%pI4 dst=%pI4, new src=%pI4\n",
		 &t->src.u3.ip, &t->dst.u3.ip, &new_ip);

	/* Transfer from original range. */
	newrange.flags = same->range[0].flags;
	newrange.min_addr.ip = newrange.max_addr.ip = new_ip;
	newrange.min_proto = same->range[0].min;
	newrange.max_proto = same->range[0].max;

	/* Hand modified range to generic setup. */
	return nf_nat_setup_info(ct, &newrange, HOOK2MANIP(xt_hooknum(par)));
}

static struct xt_target same_reg __read_mostly = {
	.name		= "SAME",
	.family		= AF_INET,
	.target		= same_target,
	.targetsize	= sizeof(struct ipt_same_info),
	.table		= "nat",
	.hooks		= (1 << NF_INET_PRE_ROUTING | 1 << NF_INET_POST_ROUTING),
	.checkentry	= same_check,
	.destroy	= same_destroy,
	.me		= THIS_MODULE,
};

static int __init ipt_same_init(void)
{
	return xt_register_target(&same_reg);
}

static void __exit ipt_same_fini(void)
{
	xt_unregister_target(&same_reg);
}

module_init(ipt_same_init);
module_exit(ipt_same_fini);

