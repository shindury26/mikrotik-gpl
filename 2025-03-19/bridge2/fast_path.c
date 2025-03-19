#include <linux/module.h>
#include <linux/packet_hook.h>
#include <linux/counter.h>
#include <net/sch_generic.h>
#include "br_private.h"

static inline int check_hw(const struct net_bridge_port *p,
		unsigned from_switch_group)
{
	return p->switch_group == BR_NOSWITCH
		|| p->switch_group != from_switch_group;
}

static inline int should_deliver(const struct net_bridge_port *p, 
	const struct net_device *src, unsigned horizon) {
    return (src != p->dev
	    && p->state == BR_STATE_FORWARDING
	    && (horizon == BR_NOHORIZON || horizon != p->horizon));
}

static int br_dev_fast_path_xmit(struct net_device *dev, struct fp_buf *fpb) {
    struct net_bridge *br = netdev_priv(dev);
    struct net_bridge_fdb_entry *fdb;
    struct net_device *dst;

    if (is_multicast_ether_addr(fpb_data(fpb))) {
	goto slow_path;
    }
    if (br_opt_get(br, BROPT_VLAN_ENABLED)) {
	goto slow_path;
    }
    fdb = br_fdb_find_rcu(br, fpb_data(fpb), 0);
    if (!fdb || !fdb->dst) {
	goto slow_path;
    }

    if (fdb->dst->state != BR_STATE_FORWARDING) {
	goto slow_path;
    }

    dst = fdb->dst->dev;

    if (dst->l2mtu + ETH_HLEN < fpb_len(fpb)) {
	goto slow_path;
    }

    fast_path_fp_tx_inc(dev, fpb);
    return fast_path_tx(dst, fpb);
slow_path:
    return fast_path_tx_slow(dev, fpb);
}


static struct net_device *fast_forward_handler(struct net_device *dev, struct fp_buf *fpb) {
    unsigned char *data = fpb_data(fpb);
    unsigned x;
    struct net_device *dst = dev->fp.forward_dev;
    unsigned a1 = get_unaligned((unsigned *)data);
    unsigned short a2 = get_unaligned((unsigned short *)&data[4]);
    if (unlikely(is_multicast_ether_addr(data))) {
	return NULL;
    }
    x = *(unsigned *)dev->fp.dev_addr - a1;
    x |= *(unsigned short *)&dev->fp.dev_addr[4] - a2;
    if (unlikely(!x)) {
	return NULL;
    }
    x = *(unsigned *)dst->fp.dev_addr - a1;
    x |= *(unsigned short *)&dst->fp.dev_addr[4] - a2;
    if (unlikely(!x)) {
	return NULL;
    }

    if (unlikely(dst->l2mtu + ETH_HLEN < fpb_len(fpb))) {
	return NULL;
    }

    {
	struct fast_path_state *fps = &get_cpu_var(per_cpu_fp_state);
	fps->forward_packets++;
	fps->forward_bytes += fpb_len(fpb);
    }
    return dst;
}

static inline void fast_path_bridge_inc(struct fp_buf *fpb) {
    struct fast_path_state *fps = &get_cpu_var(per_cpu_fp_state);
    fps->bridge_packets++;
    fps->bridge_bytes += fpb_len(fpb);
}

DECLARE_PER_CPU(struct net_device *, per_cpu_port);
static struct net_device *fast_path_handler(struct net_device *src,
	struct fp_buf *fpb) {
    struct ethhdr *eth = (struct ethhdr *)fpb_data(fpb);
    struct net_device *dst;
    struct net_bridge_port *src_port;
    struct net_bridge_port *dst_port;
    struct net_bridge_fdb_entry *fdb;
    struct net_bridge *br;
//    printk("bridge fast path rx %s %u\n", src->name, fpb_len(fpb));
//    fpb_dump("in", fpb);

    src_port = br_port_get_check_rcu(src);
    if (!src_port || src_port->state != BR_STATE_FORWARDING) {
	return NULL;
    }

    br = src_port->br;
    if (br_opt_get(br, BROPT_VLAN_ENABLED) || br->dhcp_snooping) {
	return NULL;
    }

    if (fpb_len(fpb) < ETH_HLEN) {
	return NULL;
    }

    if (is_multicast_ether_addr(eth->h_dest)) {
	return NULL;
    }

    // update src
    rcu_read_lock();
    fdb = br_fdb_find_rcu(br, eth->h_source, 0);
    rcu_read_unlock();
    if (!fdb) {
	return NULL;
    }

    if (test_bit(BR_FDB_LOCAL, &fdb->flags)) {
	return NULL;
    }

    if (fdb->dst != src_port) {
	fdb->dst = src_port;
	// Can skip since, fdb is dumped periodically
	//fdb_notify(br, fdb, RTM_NEWNEIGH);
    }
    {
	unsigned long now = jiffies;
	if (fdb->updated != now) {
	    fdb->updated = now;
	}
    }

//    printk("br rx %s %pM->%pM %pM\n", src->name, eth->h_source, eth->h_dest, br->dev->fp.dev_addr);
    if (ether_addr_equal(eth->h_dest, br->dev->fp.dev_addr)) {
	goto rx;
    }

    rcu_read_lock();
    fdb = br_fdb_find_rcu(br, eth->h_dest, 0);
    rcu_read_unlock();
    if (!fdb) {
	return NULL;
    }

    if (test_bit(BR_FDB_LOCAL, &fdb->flags)) {
    rx:
	if (likely(!*this_cpu_ptr(&per_cpu_port))) {
	    *this_cpu_ptr(&per_cpu_port) = src;
	}
	fast_path_rx_noinvalidate(br->dev, fpb);
	*this_cpu_ptr(&per_cpu_port) = NULL;
	fast_path_bridge_inc(fpb);
	return FAST_PATH_CONSUMED;
    }
    if (!fdb->dst) {
	return NULL;
    }
    dst_port = fdb->dst;
    dst = dst_port->dev;

    // final stuff
    if (!should_deliver(dst_port, src, src_port->horizon)
		    || !check_hw(dst_port, src_port->switch_group)) {
//	printk("should not deliver\n");
	return NULL;
    }

    if (dst->l2mtu + ETH_HLEN < fpb_len(fpb)) {
	return NULL;
    }

    fast_path_bridge_inc(fpb);
//    printk("ok %s\n", dst->name);
//   fpb_dump("ou", fpb);
    return dst;
}

struct fast_path bridge_fast_path = {
    .handler = fast_path_handler,
    .priority = FP_PRIO_BRIDGE,
};

struct fast_path bridge_fast_forward = {
    .handler = fast_forward_handler,
    .priority = FP_PRIO_FORWARD,
};

extern struct net_device_ops br_netdev_ops;

static ssize_t bridge_active_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf) {
    return sprintf(buf, "%d\n", !bridge_fast_path.suspended);
}

static ssize_t bridge_counter_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf) {
    return sprintf(buf, "%llu %llu %llu %llu\n",
	    fp_global.bridge_packets, fp_global.bridge_bytes,
	    fp_global.forward_packets, fp_global.forward_bytes);
}

static struct kobj_attribute bridge_active_attribute =
    __ATTR(fp_bridge_active, 0664, bridge_active_show, NULL);
static struct kobj_attribute bridge_counter_attribute =
    __ATTR(fp_bridge_counter, 0664, bridge_counter_show, NULL);

static struct attribute *attrs[] = {
    &bridge_active_attribute.attr,
    &bridge_counter_attribute.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

int __init bridge_fast_path_init(void) {
    int ret = sysfs_create_group(kernel_kobj, &attr_group);
    if (ret) {
	return ret;
    }
    rcu_assign_pointer(br_netdev_ops.ndo_fast_path_xmit, br_dev_fast_path_xmit);
    return 0;
}

void __exit bridge_fast_path_exit(void) {
    sysfs_remove_group(kernel_kobj, &attr_group);
}
