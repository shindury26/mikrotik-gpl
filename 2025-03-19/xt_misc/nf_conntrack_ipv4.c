#include <linux/module.h>
#include <net/netfilter/nf_conntrack.h>

static int __init mod_init(void) {
    return nf_ct_netns_get(&init_net, NFPROTO_IPV4);
}

static void __exit mod_exit(void) {
    nf_ct_netns_put(&init_net, NFPROTO_IPV4);
}

module_init(mod_init);
module_exit(mod_exit);
