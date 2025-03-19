#include <linux/netfilter/x_tables.h>
#include <linux/netfilter_bridge/ebtables.h>
#include <linux/module.h>

#include "ulog.h"


static unsigned ebt_ulog_target(struct sk_buff *skb,
	const struct xt_action_param *par) {
    ulog_target(skb, xt_in(par), xt_out(par), (struct ulog_info *) par->targinfo, 0);
    return EBT_CONTINUE;
}

static bool ebt_ulog_match(const struct sk_buff *skb,
	struct xt_action_param *par) {
    ulog_target(skb, xt_in(par), xt_out(par), (struct ulog_info *) par->matchinfo, 0);
    return true;
}

static struct xt_target ebt_ulog_t = {
    .name = "ULOG",
    .revision = 0,
    .family = NFPROTO_BRIDGE,
    .target = ebt_ulog_target,
    .targetsize = EBT_ALIGN(sizeof(struct ulog_info)),
    .me	= THIS_MODULE,
};

static struct xt_match ebt_ulog_m = {
    .name = "ulog",
    .revision = 0,
    .family = NFPROTO_BRIDGE,
    .match = ebt_ulog_match,
    .matchsize = EBT_ALIGN(sizeof(struct ulog_info)),
    .me	= THIS_MODULE,
};

static int __init init(void) {
    int ret;
    ret = xt_register_target(&ebt_ulog_t);
    if (ret) {
	return ret;
    }
    ret = xt_register_match(&ebt_ulog_m);
    if (ret) {
	xt_unregister_target(&ebt_ulog_t);
    }
    return ret;
}

static void __exit fini(void) {
    xt_unregister_match(&ebt_ulog_m);
    xt_unregister_target(&ebt_ulog_t);
}

module_init(init);
module_exit(fini);
