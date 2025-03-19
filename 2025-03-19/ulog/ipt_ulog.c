#include <linux/netfilter_ipv4/ip_tables.h>
#include <linux/module.h>

#include "ulog.h"

static unsigned int ipt_ulog_target(struct sk_buff *skb,
				    const struct xt_action_param *par) {
    ulog_target(skb, xt_in(par), xt_out(par), (struct ulog_info *) par->targinfo, 1);
    return XT_CONTINUE;
}

static unsigned int ip6t_ulog_target(struct sk_buff *skb,
				     const struct xt_action_param *par) {
    ulog_target(skb, xt_in(par), xt_out(par), (struct ulog_info *) par->targinfo, 2);
    return XT_CONTINUE;
}

static bool ipt_ulog_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    ulog_target(skb, xt_in(par), xt_out(par), (struct ulog_info *) par->matchinfo, 1);
    return true;
}

static bool ip6t_ulog_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    ulog_target(skb, xt_in(par), xt_out(par), (struct ulog_info *) par->matchinfo, 2);
    return true;
}

static struct xt_target xt_ulog[] = {
    {
	.name = "ULOG",
	.family = AF_INET,
	.target = ipt_ulog_target,
	.targetsize = sizeof(struct ulog_info),
	me: THIS_MODULE,
    },
    {
	.name = "ULOG",
	.family = AF_INET6,
	.target = ip6t_ulog_target,
	.targetsize = sizeof(struct ulog_info),
	me: THIS_MODULE,
    },
};

static struct xt_match xt_ulog_match[] = {
    {
	.name		= "ulog",
	.family		= AF_INET,
	.match		= ipt_ulog_match,
	.matchsize	= sizeof(struct ulog_info),
	.me		= THIS_MODULE
    },
    {
	.name		= "ulog",
	.family		= AF_INET6,
	.match		= ip6t_ulog_match,
	.matchsize	= sizeof(struct ulog_info),
	.me		= THIS_MODULE
    },
};

static int __init init(void)
{
    int ret;
    ret = xt_register_targets(xt_ulog, ARRAY_SIZE(xt_ulog));
    if (ret) {
	return ret;
    }
    ret = xt_register_matches(xt_ulog_match, ARRAY_SIZE(xt_ulog_match));
    if (ret) {
	xt_unregister_targets(xt_ulog, ARRAY_SIZE(xt_ulog));
    }
    return ret;
}

static void __exit fini(void)
{
    xt_unregister_matches(xt_ulog_match, ARRAY_SIZE(xt_ulog_match));
    xt_unregister_targets(xt_ulog, ARRAY_SIZE(xt_ulog));
}

module_init(init);
module_exit(fini);
