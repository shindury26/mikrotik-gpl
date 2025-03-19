// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *	Userspace interface
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/netpoll.h>
#include <linux/ethtool.h>
#include <linux/if_arp.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/rtnetlink.h>
#include <linux/if_ether.h>
#include <linux/slab.h>
#include <net/dsa.h>
#include <net/sock.h>
#include <linux/if_vlan.h>
#include <net/switchdev.h>
#include <net/net_namespace.h>
#include <linux/packet_hook.h>

#include "br_private.h"

extern struct fast_path bridge_fast_path;
extern struct fast_path bridge_fast_forward;

///*
// * Determine initial path cost based on speed.
// * using recommendations from 802.1d standard
// *
// * Since driver might sleep need to not be holding any locks.
// */
//static int port_cost(struct net_device *dev)
//{
//	struct ethtool_link_ksettings ecmd;

//	if (!__ethtool_get_link_ksettings(dev, &ecmd)) {
//		switch (ecmd.base.speed) {
//		case SPEED_10000:
//			return 2;
//		case SPEED_1000:
//			return 4;
//		case SPEED_100:
//			return 19;
//		case SPEED_10:
//			return 100;
//		}
//	}

//	/* Old silly heuristics based on name */
//	if (!strncmp(dev->name, "lec", 3))
//		return 7;

//	if (!strncmp(dev->name, "plip", 4))
//		return 2500;

//	return 100;	/* assume old 10Mbps */
//}


///* Check for port carrier transitions. */
//void br_port_carrier_check(struct net_bridge_port *p, bool *notified)
//{
//	struct net_device *dev = p->dev;
//	struct net_bridge *br = p->br;

//	if (!(p->flags & BR_ADMIN_COST) &&
//	    netif_running(dev) && netif_oper_up(dev))
//		p->path_cost = port_cost(dev);

//	*notified = false;
//	if (!netif_running(br->dev))
//		return;

//	spin_lock_bh(&br->lock);
//	if (netif_running(dev) && netif_oper_up(dev)) {
//		if (p->state == BR_STATE_DISABLED) {
//			br_stp_enable_port(p);
//			*notified = true;
//		}
//	} else {
//		if (p->state != BR_STATE_DISABLED) {
//			br_stp_disable_port(p);
//			*notified = true;
//		}
//	}
//	spin_unlock_bh(&br->lock);
//}

static void br_port_set_promisc(struct net_bridge_port *p)
{
	int err = 0;

	if (br_promisc_port(p))
		return;

	err = dev_set_promiscuity(p->dev, 1);
	if (err)
		return;

	br_fdb_unsync_static(p->br, p);
	p->flags |= BR_PROMISC;
}

static void br_port_clear_promisc(struct net_bridge_port *p)
{
	int err;

	/* Check if the port is already non-promisc or if it doesn't
	 * support UNICAST filtering.  Without unicast filtering support
	 * we'll end up re-enabling promisc mode anyway, so just check for
	 * it here.
	 */
	if (!br_promisc_port(p) || !(p->dev->priv_flags & IFF_UNICAST_FLT))
		return;

	/* Since we'll be clearing the promisc mode, program the port
	 * first so that we don't have interruption in traffic.
	 */
	err = br_fdb_sync_static(p->br, p);
	if (err)
		return;

	dev_set_promiscuity(p->dev, -1);
	p->flags &= ~BR_PROMISC;
}

/* When a port is added or removed or when certain port flags
 * change, this function is called to automatically manage
 * promiscuity setting of all the bridge ports.  We are always called
 * under RTNL so can skip using rcu primitives.
 */
void br_manage_promisc(struct net_bridge *br)
{
	struct net_bridge_port *p;
	bool set_all = false;

	/* If vlan filtering is disabled or bridge interface is placed
	 * into promiscuous mode, place all ports in promiscuous mode.
	 */
	if ((br->dev->flags & IFF_PROMISC) || !br_vlan_enabled(br->dev))
		set_all = true;

	list_for_each_entry(p, &br->port_list, list) {
		if (set_all) {
			br_port_set_promisc(p);
		} else {
			/* If the number of auto-ports is <= 1, then all other
			 * ports will have their output configuration
			 * statically specified through fdbs.  Since ingress
			 * on the auto-port becomes forwarding/egress to other
			 * ports and egress configuration is statically known,
			 * we can say that ingress configuration of the
			 * auto-port is also statically known.
			 * This lets us disable promiscuous mode and write
			 * this config to hw.
			 */
			if (br->auto_cnt == 0 ||
			    (br->auto_cnt == 1 && br_auto_port(p)))
				br_port_clear_promisc(p);
			else
				br_port_set_promisc(p);
		}
	}
}

int nbp_backup_change(struct net_bridge_port *p,
		      struct net_device *backup_dev)
{
	struct net_bridge_port *old_backup = rtnl_dereference(p->backup_port);
	struct net_bridge_port *backup_p = NULL;

	ASSERT_RTNL();

	if (backup_dev) {
		if (!netif_is_bridge_port(backup_dev))
			return -ENOENT;

		backup_p = br_port_get_rtnl(backup_dev);
		if (backup_p->br != p->br)
			return -EINVAL;
	}

	if (p == backup_p)
		return -EINVAL;

	if (old_backup == backup_p)
		return 0;

	/* if the backup link is already set, clear it */
	if (old_backup)
		old_backup->backup_redirected_cnt--;

	if (backup_p)
		backup_p->backup_redirected_cnt++;
	rcu_assign_pointer(p->backup_port, backup_p);

	return 0;
}

static void nbp_backup_clear(struct net_bridge_port *p)
{
	nbp_backup_change(p, NULL);
	if (p->backup_redirected_cnt) {
		struct net_bridge_port *cur_p;

		list_for_each_entry(cur_p, &p->br->port_list, list) {
			struct net_bridge_port *backup_p;

			backup_p = rtnl_dereference(cur_p->backup_port);
			if (backup_p == p)
				nbp_backup_change(cur_p, NULL);
		}
	}

	WARN_ON(rcu_access_pointer(p->backup_port) || p->backup_redirected_cnt);
}

static void nbp_update_port_count(struct net_bridge *br)
{
	struct net_bridge_port *p;
	u32 cnt = 0;

	list_for_each_entry(p, &br->port_list, list) {
		if (br_auto_port(p))
			cnt++;
	}
	if (br->auto_cnt != cnt) {
		br->auto_cnt = cnt;
		br_manage_promisc(br);
	}
}

static void nbp_delete_promisc(struct net_bridge_port *p)
{
	/* If port is currently promiscuous, unset promiscuity.
	 * Otherwise, it is a static port so remove all addresses
	 * from it.
	 */
	dev_set_allmulti(p->dev, -1);
	if (br_promisc_port(p))
		dev_set_promiscuity(p->dev, -1);
	else
		br_fdb_unsync_static(p->br, p);
}

//static void release_nbp(struct kobject *kobj)
//{
//	struct net_bridge_port *p
//		= container_of(kobj, struct net_bridge_port, kobj);
//	kfree(p);
//}

//static void brport_get_ownership(struct kobject *kobj, kuid_t *uid, kgid_t *gid)
//{
//	struct net_bridge_port *p = kobj_to_brport(kobj);

//	net_ns_get_ownership(dev_net(p->dev), uid, gid);
//}

//static struct kobj_type brport_ktype = {
//#ifdef CONFIG_SYSFS
//	.sysfs_ops = &brport_sysfs_ops,
//#endif
//	.release = release_nbp,
//	.get_ownership = brport_get_ownership,
//};

static void destroy_nbp(struct net_bridge_port *p)
{
	struct net_device *dev = p->dev;

	p->br = NULL;
	p->dev = NULL;
	dev_put(dev);

//	kobject_put(&p->kobj);
}

static void destroy_nbp_rcu(struct rcu_head *head)
{
	struct net_bridge_port *p =
			container_of(head, struct net_bridge_port, rcu);
	destroy_nbp(p);
}

static unsigned get_max_headroom(struct net_bridge *br)
{
	unsigned max_headroom = 0;
	struct net_bridge_port *p;

	list_for_each_entry(p, &br->port_list, list) {
		unsigned dev_headroom = netdev_get_fwd_headroom(p->dev);

		if (dev_headroom > max_headroom)
			max_headroom = dev_headroom;
	}

	return max_headroom;
}

static void update_headroom(struct net_bridge *br, int new_hr)
{
	struct net_bridge_port *p;

	list_for_each_entry(p, &br->port_list, list)
		netdev_set_rx_headroom(p->dev, new_hr);

	br->dev->needed_headroom = new_hr;
}

/* Delete port(interface) from bridge is done in two steps.
 * via RCU. First step, marks device as down. That deletes
 * all the timers and stops new packets from flowing through.
 *
 * Final cleanup doesn't occur until after all CPU's finished
 * processing packets.
 *
 * Protected from multiple admin operations by RTNL mutex
 */
static void del_nbp(struct net_bridge_port *p)
{
	struct net_bridge *br = p->br;
	struct net_device *dev = p->dev;
	struct net_dhcp_info_opt *opt;

//	sysfs_remove_link(br->ifobj, p->dev->name);

	--br->port_count;
	if (br->fast_forward) {
	    unregister_fast_path(&bridge_fast_forward, dev);
	}
	unregister_fast_path(&bridge_fast_path, dev);

	nbp_delete_promisc(p);

	spin_lock_bh(&br->lock);
//	br_stp_disable_port(p);
	{
		p->state = BR_STATE_DISABLED;
		p->vlan_def_state = BR_STATE_DISABLED;
		br_multicast_disable_port(p);
	}
	spin_unlock_bh(&br->lock);

	br_ifinfo_notify(RTM_DELLINK, NULL, p);

	list_del_rcu(&p->list);
	if (netdev_get_fwd_headroom(dev) == br->dev->needed_headroom)
		update_headroom(br, get_max_headroom(br));
	netdev_reset_rx_headroom(dev);

	nbp_vlan_flush(p);
	br_fdb_delete_by_port(br, p, 0, 1);
	switchdev_deferred_process();
	nbp_backup_clear(p);

	nbp_update_port_count(br);

	netdev_upper_dev_unlink(dev, br->dev);

	dev->priv_flags &= ~IFF_BRIDGE_PORT;

	rcu_assign_pointer(dev->master_dev, NULL);
	netdev_rx_handler_unregister(dev);

	br_multicast_del_port(p);

//	kobject_uevent(&p->kobj, KOBJ_REMOVE);
//	kobject_del(&p->kobj);

//	br_netpoll_disable(p);

	opt = rcu_dereference(p->info_option);
	if (opt) {
		call_rcu(&opt->rcu, destroy_info_opt);
		rcu_assign_pointer(p->info_option, NULL);
	}

	call_rcu(&p->rcu, destroy_nbp_rcu);

	br_update_fast_forward(br);
}

/* Delete bridge device */
void br_dev_delete(struct net_device *dev, struct list_head *head)
{
	struct net_bridge *br = netdev_priv(dev);
	struct net_bridge_port *p, *n;

	list_for_each_entry_safe(p, n, &br->port_list, list) {
		del_nbp(p);
	}

//	br_recalculate_neigh_suppress_enabled(br);

	br_fdb_delete_by_port(br, NULL, 0, 1);

	cancel_delayed_work_sync(&br->gc_work);

//	br_sysfs_delbr(br->dev);
	unregister_netdevice_queue(br->dev, head);
}

/* find an available port number */
static int find_portno(struct net_bridge *br)
{
	int index;
	struct net_bridge_port *p;
	unsigned long *inuse;

	inuse = bitmap_zalloc(BR_MAX_PORTS, GFP_KERNEL);
	if (!inuse)
		return -ENOMEM;

	set_bit(0, inuse);	/* zero is reserved */
	list_for_each_entry(p, &br->port_list, list) {
		set_bit(p->port_no, inuse);
	}
	index = find_first_zero_bit(inuse, BR_MAX_PORTS);
	bitmap_free(inuse);

	return (index >= BR_MAX_PORTS) ? -EXFULL : index;
}

/* called with RTNL but without bridge lock */
static struct net_bridge_port *new_nbp(struct net_bridge *br,
				       struct net_device *dev)
{
	struct net_bridge_port *p;
	int index, err;

	index = find_portno(br);
	if (index < 0)
		return ERR_PTR(index);

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (p == NULL)
		return ERR_PTR(-ENOMEM);

	p->br = br;
	dev_hold(dev);
	p->dev = dev;
//	p->path_cost = port_cost(dev);
//	p->priority = 0x8000 >> BR_PORT_BITS;
	p->port_no = index;
	p->flags = BR_LEARNING | BR_FLOOD | BR_MCAST_FLOOD | BR_BCAST_FLOOD;
//	br_init_port(p);
//	br_set_state(p, BR_STATE_DISABLED);
//	br_stp_port_timer_init(p);
	p->state = BR_STATE_DISABLED;
	p->vlan_def_state = BR_STATE_DISABLED;
	p->horizon = BR_NOHORIZON;
	p->switch_group = BR_NOSWITCH;
	err = br_multicast_add_port(p);
	if (err) {
		dev_put(dev);
		kfree(p);
		p = ERR_PTR(err);
	}

	return p;
}

int br_add_bridge(struct net *net, const char *name)
{
	struct net_device *dev;
	int res;

	dev = alloc_netdev(sizeof(struct net_bridge), name, NET_NAME_UNKNOWN,
			   br_dev_setup);

	if (!dev)
		return -ENOMEM;

	dev_net_set(dev, net);
	dev->rtnl_link_ops = &br_link_ops;

	res = register_netdev(dev);
	if (res)
		free_netdev(dev);
	return res;
}

int br_del_bridge(struct net *net, const char *name)
{
	struct net_device *dev;
	int ret = 0;

	rtnl_lock();
	dev = __dev_get_by_name(net, name);
	if (dev == NULL)
		ret =  -ENXIO; 	/* Could not find device */

	else if (!(dev->priv_flags & IFF_EBRIDGE)) {
		/* Attempt to delete non bridge device! */
		ret = -EPERM;
	}

	else if (dev->flags & IFF_UP) {
		/* Not shutdown yet. */
		ret = -EBUSY;
	}

	else
		br_dev_delete(dev, NULL);

	rtnl_unlock();
	return ret;
}

#if 0
/* MTU of the bridge pseudo-device: ETH_DATA_LEN or the minimum of the ports */
static int br_mtu_min(const struct net_bridge *br)
{
	const struct net_bridge_port *p;
	int ret_mtu = 0;

	list_for_each_entry(p, &br->port_list, list) {
		int dm = p->dev->l2mtu;
		if (!dm) dm = p->dev->mtu;
		if (!ret_mtu || ret_mtu > dm)
			ret_mtu = dm;
	}

	return ret_mtu ? ret_mtu : ETH_DATA_LEN;
}

void br_mtu_auto_adjust(struct net_bridge *br)
{
	ASSERT_RTNL();

	/* if the bridge MTU was manually configured don't mess with it */
	if (br_opt_get(br, BROPT_MTU_SET_BY_USER))
		return;

	/* change to the minimum MTU and clear the flag which was set by
	 * the bridge ndo_change_mtu callback
	 */
	dev_set_mtu(br->dev, br_mtu_min(br));
	br_opt_toggle(br, BROPT_MTU_SET_BY_USER, false);
}
#endif

static void br_set_gso_limits(struct net_bridge *br)
{
	unsigned int gso_max_size = GSO_MAX_SIZE;
	u16 gso_max_segs = GSO_MAX_SEGS;
	const struct net_bridge_port *p;

	list_for_each_entry(p, &br->port_list, list) {
		gso_max_size = min(gso_max_size, p->dev->gso_max_size);
		gso_max_segs = min(gso_max_segs, p->dev->gso_max_segs);
	}
	br->dev->gso_max_size = gso_max_size;
	br->dev->gso_max_segs = gso_max_segs;
}

/*
 * Recomputes features using slave's features
 */
netdev_features_t br_features_recompute(struct net_bridge *br,
	netdev_features_t features)
{
	struct net_bridge_port *p;
	netdev_features_t mask;

	if (list_empty(&br->port_list))
		return features;

	mask = features;
	features &= ~NETIF_F_ONE_FOR_ALL;

	list_for_each_entry(p, &br->port_list, list) {
		features = netdev_increment_features(features,
						     p->dev->features, mask);
	}
	features = netdev_add_tso_features(features, mask);

	return features;
}

/* called with RTNL */
int br_add_if(struct net_bridge *br, struct net_device *dev,
	      struct netlink_ext_ack *extack)
{
	struct net_bridge_port *p;
	int err = 0;
	unsigned br_hr, dev_hr;
//	bool changed_addr;

	/* Don't allow bridging non-ethernet like devices, or DSA-enabled
	 * master network devices since the bridge layer rx_handler prevents
	 * the DSA fake ethertype handler to be invoked, so we do not strip off
	 * the DSA switch tag protocol header and the bridge layer just return
	 * RX_HANDLER_CONSUMED, stopping RX processing for these frames.
	 */
	if ((dev->flags & IFF_LOOPBACK) ||
	    (dev->type != ARPHRD_ETHER && dev->type != ARPHRD_PPP) ||
	    netdev_uses_dsa(dev))
		return -EINVAL;

	/* No bridging of bridges */
	if (dev->netdev_ops->ndo_start_xmit == br_dev_xmit) {
		NL_SET_ERR_MSG(extack,
			       "Can not enslave a bridge to a bridge");
		return -ELOOP;
	}

	/* Device has master upper dev */
	if (netdev_master_upper_dev_get(dev))
		return -EBUSY;

	/* No bridging devices that dislike that (e.g. wireless) */
	if (dev->priv_flags & IFF_DONT_BRIDGE) {
		NL_SET_ERR_MSG(extack,
			       "Device does not allow enslaving to a bridge");
		return -EOPNOTSUPP;
	}

	p = new_nbp(br, dev);
	if (IS_ERR(p))
		return PTR_ERR(p);

	call_netdevice_notifiers(NETDEV_JOIN, dev);

	err = dev_set_allmulti(dev, 1);
	if (err) {
		kfree(p);	/* kobject not yet init'd, manually free */
		goto err1;
	}

//	err = kobject_init_and_add(&p->kobj, &brport_ktype, &(dev->dev.kobj),
//				   SYSFS_BRIDGE_PORT_ATTR);
//	if (err)
//		goto err2;

//	err = br_sysfs_addif(p);
//	if (err)
//		goto err2;

//	err = br_netpoll_enable(p);
//	if (err)
//		goto err3;

	err = netdev_rx_handler_register(dev, br_handle_frame, p);
	if (err)
		goto err4;

	dev->priv_flags |= IFF_BRIDGE_PORT;

	err = netdev_master_upper_dev_link(dev, br->dev, NULL, NULL, extack);
	if (err)
		goto err5;

	err = nbp_switchdev_mark_set(p);
	if (err)
		goto err6;

	dev_disable_lro(dev);

	list_add_rcu(&p->list, &br->port_list);

	nbp_update_port_count(br);

	netdev_update_features(br->dev);

	br_hr = br->dev->needed_headroom;
	dev_hr = netdev_get_fwd_headroom(dev);
	if (br_hr < dev_hr)
		update_headroom(br, dev_hr);
	else
		netdev_set_rx_headroom(dev, br_hr);

	if (dev->type == ARPHRD_ETHER && br_fdb_insert(br, p, dev->dev_addr, 0))
		netdev_err(dev, "failed insert local address bridge forwarding table\n");

	if (br->dev->addr_assign_type != NET_ADDR_SET) {
		/* Ask for permission to use this MAC address now, even if we
		 * don't end up choosing it below.
		 */
		err = dev_pre_changeaddr_notify(br->dev, dev->dev_addr, extack);
		if (err)
			goto err7;
	}

	err = nbp_vlan_init(p, extack);
	if (err) {
		netdev_err(dev, "failed to initialize vlan filtering on this port\n");
		goto err7;
	}

//	spin_lock_bh(&br->lock);
//	changed_addr = br_stp_recalculate_bridge_id(br);

//	if (netif_running(dev) && netif_oper_up(dev) &&
//	    (br->dev->flags & IFF_UP))
//		br_stp_enable_port(p);
//	spin_unlock_bh(&br->lock);

	br_ifinfo_notify(RTM_NEWLINK, NULL, p);

//	if (changed_addr)
//		call_netdevice_notifiers(NETDEV_CHANGEADDR, br->dev);

#if 0 // do not change bridge MTU under the hood
	br_mtu_auto_adjust(br);
#endif
	br_set_gso_limits(br);

//	kobject_uevent(&p->kobj, KOBJ_ADD);

	rcu_assign_pointer(dev->master_dev, br->dev);

	++br->port_count;
	register_fast_path(&bridge_fast_path, dev);
	br_update_fast_forward(br);

	return 0;

err7:
	list_del_rcu(&p->list);
	br_fdb_delete_by_port(br, p, 0, 1);
	nbp_update_port_count(br);
err6:
	netdev_upper_dev_unlink(dev, br->dev);
err5:
	dev->priv_flags &= ~IFF_BRIDGE_PORT;
	netdev_rx_handler_unregister(dev);
err4:
//	br_netpoll_disable(p);
//err3:
//	sysfs_remove_link(br->ifobj, p->dev->name);
//err2:
//	kobject_put(&p->kobj);
	dev_set_allmulti(dev, -1);
err1:
	dev_put(dev);
	return err;
}

/* called with RTNL */
int br_del_if(struct net_bridge *br, struct net_device *dev)
{
	struct net_bridge_port *p;
//	bool changed_addr;

	p = br_port_get_rtnl(dev);
	if (!p || p->br != br)
		return -EINVAL;

	/* Since more than one interface can be attached to a bridge,
	 * there still maybe an alternate path for netconsole to use;
	 * therefore there is no reason for a NETDEV_RELEASE event.
	 */
	del_nbp(p);

#if 0 // do not change bridge MTU under the hood
	br_mtu_auto_adjust(br);
#endif
	br_set_gso_limits(br);

//	spin_lock_bh(&br->lock);
//	changed_addr = br_stp_recalculate_bridge_id(br);
//	spin_unlock_bh(&br->lock);

//	if (changed_addr)
//		call_netdevice_notifiers(NETDEV_CHANGEADDR, br->dev);

	netdev_update_features(br->dev);

	return 0;
}

void br_port_flags_change(struct net_bridge_port *p, unsigned long mask)
{
	struct net_bridge *br = p->br;

	if (mask & BR_AUTO_MASK)
		nbp_update_port_count(br);

//	if (mask & BR_NEIGH_SUPPRESS)
//		br_recalculate_neigh_suppress_enabled(br);
}

bool br_port_flag_is_set(const struct net_device *dev, unsigned long flag)
{
	struct net_bridge_port *p;

	p = br_port_get_rtnl_rcu(dev);
	if (!p)
		return false;

	return p->flags & flag;
}
EXPORT_SYMBOL_GPL(br_port_flag_is_set);

void br_update_fast_forward(struct net_bridge *br) {
	struct net_bridge_port *p;
	bool ff = br->allow_fast_forward && br->port_count == 2
		&& !(br->dev->flags & IFF_PROMISC) && !br_opt_get(br, BROPT_VLAN_ENABLED)
		&& !br->dhcp_snooping;
	if (ff) {
		struct net_bridge_port *p1 = list_entry(br->port_list.next, struct net_bridge_port, list);
		struct net_bridge_port *p2 = list_entry(br->port_list.prev, struct net_bridge_port, list);
		if (!ether_addr_equal(p1->dev->dev_addr, br->dev->dev_addr)
				&& !ether_addr_equal(p2->dev->dev_addr, br->dev->dev_addr)) {
			ff = false;
		}
		else if (p1->state != BR_STATE_FORWARDING || p2->state != BR_STATE_FORWARDING) {
			ff = false;
		}
		else if (p1->switch_group != BR_NOSWITCH
				|| p2->switch_group != BR_NOSWITCH) {
			ff = false;
		}
		else if (!(p1->flags & BR_FLOOD) || !(p2->flags & BR_FLOOD)) {
			ff = false;
		}
		else if (p1->horizon != BR_NOHORIZON
				|| p2->horizon != BR_NOHORIZON) {
			ff = false;
		}
	}

	if (ff == br->fast_forward) {
		return;
	}

	if (ff) {
		struct net_bridge_port *p1 = list_entry(br->port_list.next, struct net_bridge_port, list);
		struct net_bridge_port *p2 = list_entry(br->port_list.prev, struct net_bridge_port, list);
		p1->dev->fp.forward_dev = p2->dev;
		p2->dev->fp.forward_dev = p1->dev;
		register_fast_path(&bridge_fast_forward, p1->dev);
		register_fast_path(&bridge_fast_forward, p2->dev);
	}
	else {
		list_for_each_entry(p, &br->port_list, list) {
			unregister_fast_path(&bridge_fast_forward, p->dev);
		}
	}
	br->fast_forward = ff;
}
