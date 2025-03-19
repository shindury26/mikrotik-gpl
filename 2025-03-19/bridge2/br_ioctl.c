// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *	Ioctl handler
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 */

#include <linux/capability.h>
#include <linux/kernel.h>
#include <linux/if_bridge.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/times.h>
#include <net/net_namespace.h>
#include <linux/uaccess.h>
#include "br_private.h"

/* called with RTNL */
static int get_bridge_ifindices(struct net *net, int *indices, int num)
{
	struct net_device *dev;
	int i = 0;

	for_each_netdev(net, dev) {
		if (i >= num)
			break;
		if (dev->priv_flags & IFF_EBRIDGE)
			indices[i++] = dev->ifindex;
	}

	return i;
}

/* called with RTNL */
static void get_port_ifindices(struct net_bridge *br, int *ifindices, int num)
{
	struct net_bridge_port *p;
        int i = 0;

	list_for_each_entry(p, &br->port_list, list) {
            ifindices[i++] = p->dev->ifindex;
	}
}

/*
 * Format up to a page worth of forwarding table entries
 * userbuf -- where to copy result
 * maxnum  -- maximum number of entries desired
 *            (limited to a page for sanity)
 * offset  -- number of records to skip
 */
static int get_fdb_entries(struct net_bridge *br, void __user *userbuf,
			   unsigned long maxnum, unsigned long offset)
{
	int num;
	void *buf;
	size_t size;

	/* Clamp size to PAGE_SIZE, test maxnum to avoid overflow */
	if (maxnum > PAGE_SIZE/sizeof(struct abrctl_fdb_entry))
		maxnum = PAGE_SIZE/sizeof(struct abrctl_fdb_entry);

	size = maxnum * sizeof(struct abrctl_fdb_entry);

	buf = kmalloc(size, GFP_USER);
	if (!buf)
		return -ENOMEM;

	num = br_fdb_fillbuf(br, buf, maxnum, offset);
	if (num > 0) {
		if (copy_to_user(userbuf, buf, num*sizeof(struct abrctl_fdb_entry)))
			num = -EFAULT;
	}
	kfree(buf);

	return num;
}

static int add_del_if(struct net_bridge *br, int ifindex, int isadd)
{
	struct net *net = dev_net(br->dev);
	struct net_device *dev;
	int ret;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	dev = __dev_get_by_index(net, ifindex);
	if (dev == NULL)
		return -EINVAL;

	if (isadd)
		ret = br_add_if(br, dev, NULL);
	else
		ret = br_del_if(br, dev);

	return ret;
}

static struct net_bridge_port *find_port(struct net_bridge *br, int ifindex)
{
	struct net_bridge_port *p = NULL;
	list_for_each_entry(p, &br->port_list, list) {
		if (p->dev->ifindex == ifindex) {
			return p;
		}
	}
	return NULL;
}

/*
 * Legacy ioctl's through SIOCDEVPRIVATE
 * This interface is deprecated because it was too difficult to
 * to do the translation for 32/64bit ioctl compatability.
 */
static int old_dev_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct net_bridge *br = netdev_priv(dev);
#ifdef CONFIG_64BIT
	// XXX: 32bit compatibility for tile. 64bit userspace not supported
	unsigned args_small[4];
	unsigned long args[4];
	if (copy_from_user(args_small, rq->ifr_data, sizeof(args_small)))
		return -EFAULT;
	args[0] = args_small[0];
	args[1] = args_small[1];
	args[2] = args_small[2];
	args[3] = args_small[3];
	switch (args[0]) {
	case BRCTL_GET_BRIDGE_INFO:
	case BRCTL_GET_PORT_LIST:
	case BRCTL_GET_FDB_ENTRIES:
	case ABRCTL_SET_MC_OPTS:
	    args[1] = (unsigned long)compat_ptr(args_small[1]);
	    break;
	case ABRCTL_SET_INFO_OPT:
	    args[1] = (unsigned long)compat_ptr(args_small[1]);
	    args[2] = (unsigned long)compat_ptr(args_small[2]);
	    break;
	}
#else
	unsigned long args[4];
	if (copy_from_user(args, rq->ifr_data, sizeof(args)))
		return -EFAULT;
#endif

	switch (args[0]) {
	case BRCTL_ADD_IF:
	case BRCTL_DEL_IF:
		return add_del_if(br, args[1], args[0] == BRCTL_ADD_IF);

	case BRCTL_GET_BRIDGE_INFO:
	{
		struct __bridge_info b;

		memset(&b, 0, sizeof(struct __bridge_info));
		rcu_read_lock();
/*
		memcpy(&b.designated_root, &br->designated_root, 8);
		memcpy(&b.bridge_id, &br->bridge_id, 8);
		b.root_path_cost = br->root_path_cost;
		b.max_age = jiffies_to_clock_t(br->max_age);
		b.hello_time = jiffies_to_clock_t(br->hello_time);
		b.forward_delay = br->forward_delay;
		b.bridge_max_age = br->bridge_max_age;
		b.bridge_hello_time = br->bridge_hello_time;
		b.bridge_forward_delay = jiffies_to_clock_t(br->bridge_forward_delay);
		b.topology_change = br->topology_change;
		b.topology_change_detected = br->topology_change_detected;
		b.root_port = br->root_port;
		b.stp_enabled = br->stp_enabled;
*/
		b.ageing_time = jiffies_to_clock_t(br->ageing_time);
/*
		b.hello_timer_value = br_timer_value(&br->hello_timer);
		b.tcn_timer_value = br_timer_value(&br->tcn_timer);
		b.topology_change_timer_value = br_timer_value(&br->topology_change_timer);
*/
	        rcu_read_unlock();

		if (copy_to_user((void __user *)args[1], &b, sizeof(b)))
			return -EFAULT;

		return 0;
	}

	case BRCTL_GET_PORT_LIST:
	{
		int num, *indices;

		num = args[2];
		if (num < 0)
			return -EINVAL;
		if (num == 0)
			num = 256;
		if (num > BR_MAX_PORTS)
			num = BR_MAX_PORTS;

		indices = kcalloc(num, sizeof(int), GFP_KERNEL);
		if (indices == NULL)
			return -ENOMEM;

		get_port_ifindices(br, indices, num);
		if (copy_to_user((void __user *)args[1], indices, num*sizeof(int)))
			num =  -EFAULT;
		kfree(indices);
		return num;
	}

	case BRCTL_SET_AGEING_TIME:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;

		br->ageing_time = args[1];
		return 0;

	case BRCTL_SET_BRIDGE_STP_STATE:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;

		br->stp_enabled = args[1] & 3 ? BR_USER_STP : BR_NO_STP;
		if (br->stp_enabled == BR_NO_STP) {
			br->group_fwd_mask = 0xffff;
		}
		br->allow_fast_forward = !!(args[1] & 4);
		br_multicast_toggle(br, !!(args[1] & 8));
		br_vlan_filter_toggle(br, !!(args[1] & 0x10));
		br_vlan_set_proto(br, args[2]);
		br->dhcp_snooping = !!(args[1] & 0x20);
		br->add_info_option = !!(args[1] & 0x40);

		return 0;

	case BRCTL_GET_FDB_ENTRIES:
		return get_fdb_entries(br, (void __user *)args[1], 
				       args[2], args[3]);

	case ABRCTL_GET_MC_ROUTER: {
		struct net_bridge_port *p;
		int ret = 0;

		spin_lock_bh(&br->multicast_lock);
		if (args[1] == 0) {
			ret = br_multicast_is_router(br);
		}
		else {
			hlist_for_each_entry(p, &br->router_list, rlist) {
				if (p->dev->ifindex == args[1]) {
					ret = 1;
					break;
				}
			}
		}
		spin_unlock_bh(&br->multicast_lock);

		return ret;
	}

	case ABRCTL_FDB_FLUSH: {
		struct net_bridge_port *p = NULL;
		unsigned portidx = args[1];
		unsigned vid = args[2];
//		unsigned external = args[3];
		int ret = 0;
		spin_lock_bh(&br->lock);

		if (portidx != 0) {
			p = find_port(br, portidx);
			if (!p) {
				ret = -ENODEV;
				goto out_delete;
			}
		}
//		if (external) {
//			br_fdb_flush_ext(br);
//		} else {
			br_fdb_delete_by_port(br, p, vid, 0);
//		}
out_delete:
		spin_unlock_bh(&br->lock);
		return ret;
	}

	case ABRCTL_SET_PORTHORIZON: {
            int ret = -ENODEV;
            struct net_bridge_port *p;

            spin_lock_bh(&br->lock);

            list_for_each_entry(p, &br->port_list, list) {
                if (p->dev->ifindex == args[1]) {
                    br_debug(br, "update %s horizon from 0x%x to 0x%x\n",
                             p->dev->name, p->horizon, (unsigned)args[2]);
                    p->horizon = args[2];
                    ret = 0;
                    break;
                }
            }

            spin_unlock_bh(&br->lock);
	    br_update_fast_forward(br);
            return ret;
	}

	case ABRCTL_SET_SWITCH_GROUP: {
            int ret = -ENODEV;
            struct net_bridge_port *p;
	    unsigned old = BR_NOSWITCH;

            spin_lock_bh(&br->lock);
            list_for_each_entry(p, &br->port_list, list) {
                if (p->dev->ifindex == args[1]) {
			old = p->switch_group;
			p->switch_group = args[2];
			ret = 0;
			br_debug(p->br, "set group %s %u\n",
				 p->dev->name, p->switch_group);
			break;
		}
	    }
            spin_unlock_bh(&br->lock);

	    if (ret)
		    return ret;

	    if (old != p->switch_group) {
		int type;
		if (p->switch_group == BR_NOSWITCH) {
		    br_multicast_offload(p, false);
		    type = RTM_DELNEIGH;
		}
		else {
		    br_multicast_offload(p, true);
		    type = RTM_NEWNEIGH;
		}

		/* If port removed from hw then notify to remove all user entries. Otherwise
		 * install them. */
		struct net_bridge_fdb_entry *f;
		rcu_read_lock();
		hlist_for_each_entry_rcu(f, &br->fdb_list, fdb_node) {
		    if (test_bit(BR_FDB_ADDED_BY_USER, &f->flags) && f->dst == p) {
			br_switchdev_fdb_notify(f, type);
		    }
		}
		rcu_read_unlock();
	    }
	    br_update_fast_forward(br);
	    return 0;
	}

	case ABRCTL_ADD_VLAN: {
		unsigned ret = -ENODEV;
		unsigned portidx = args[1];
		unsigned vid = args[2];
		unsigned flags = args[3];
		struct net_bridge_port *p = NULL;

		if (vid >= VLAN_N_VID)
			return -EINVAL;
		if (!br_opt_get(br, BROPT_VLAN_ENABLED))
			return -EFAULT;

		br_debug(br, "add vlan %d on %d flags %08x\n",
				vid, portidx, flags);

		bool changed;
		if (portidx) {
			list_for_each_entry(p, &br->port_list, list) {
				if (p->dev->ifindex == portidx) {
					ret = nbp_vlan_add(p, vid, flags, &changed, NULL);
					break;
				}
			}
		} else {
			flags |= BRIDGE_VLAN_INFO_BRENTRY;
			ret = br_vlan_add(br, vid, flags, &changed, NULL);
		}
		return ret;
	}

	case ABRCTL_DEL_VLAN: {
		unsigned ret = -ENODEV;
		unsigned portidx = args[1];
		unsigned vid = args[2];
		struct net_bridge_port *p = NULL;

		if (vid >= VLAN_N_VID)
			return -EINVAL;
		if (!br_opt_get(br, BROPT_VLAN_ENABLED))
			return -EFAULT;

		br_debug(br, "del vlan %d on %d\n", vid, portidx);

		if (portidx) {
			list_for_each_entry(p, &br->port_list, list) {
				if (p->dev->ifindex == portidx) {
					ret = nbp_vlan_delete(p, vid);
					break;
				}
			}
		} else {
			ret = br_vlan_delete(br, vid);
		}
		return ret;
	}

        case ABRCTL_SET_VLAN_OPTS: {
		unsigned ret = -ENODEV;
		unsigned portidx = args[1];
		unsigned vlan_ingress = args[2];
		struct net_bridge_port *p = NULL;

		if (!br_opt_get(br, BROPT_VLAN_ENABLED))
			return -EFAULT;

		spin_lock_bh(&br->lock);

		br_debug(br, "set vlan opts on %u opts %u %lu %lu\n",
				portidx,
				vlan_ingress & BR_IN_FRAME_TYPES,
				vlan_ingress & BR_IN_FILTERING,
				vlan_ingress & BR_IN_TAG_STACKING);

		if (portidx) {
			list_for_each_entry(p, &br->port_list, list) {
				if (p->dev->ifindex == portidx) {
					p->vlan_ingress = vlan_ingress;
					ret = 0;
					break;
				}
			}
		} else {
			br->vlan_ingress = vlan_ingress;
		}

		spin_unlock_bh(&br->lock);
		return ret;
	}

        case ABRCTL_SET_PORT_OPTS: {
		unsigned ret = -ENODEV;
		unsigned portidx = args[1];
		unsigned flags = args[2];
		struct net_bridge_port *p = NULL;

		spin_lock_bh(&br->lock);
		list_for_each_entry(p, &br->port_list, list) {
			if (p->dev->ifindex == portidx) {
				br_debug(br, "set port %s flag %08x\n",
					 p->dev->name, flags);
				p->flags = flags;
				ret = 0;
				break;
			}
		}
		spin_unlock_bh(&br->lock);
		br_update_fast_forward(br);
		return ret;
	}

        case ABRCTL_SET_INFO_OPT: {
		unsigned ret = 0;
		unsigned portidx = args[1];
		struct bridge_info_opt req;
		struct net_bridge_port *p = NULL;
		struct net_dhcp_info_opt *old_info, *new_info;

		if (copy_from_user(&req, (void __user *)args[2], sizeof(req)))
			return -EFAULT;

		spin_lock_bh(&br->lock);

		p = find_port(br, portidx);
		if (!p) {
			ret = -ENODEV;
			goto out_info_opt1;
		}

		new_info = kzalloc(sizeof(*new_info), GFP_KERNEL);
		if (!new_info) {
			ret = -ENOMEM;
			goto out_info_opt1;
		}

		new_info->circuit_id = kzalloc(req.circuit_id_size, GFP_KERNEL);
		if (!new_info->circuit_id) {
			ret = -ENOMEM;
			goto out_info_opt2;
		}

		new_info->remote_id = kzalloc(req.remote_id_size, GFP_KERNEL);
		if (!new_info->remote_id) {
			ret = -ENOMEM;
			goto out_info_opt3;
		}

		if (copy_from_user(new_info->circuit_id,
				   compat_ptr(req.circuit_id),
				   req.circuit_id_size)) {
			ret = -EFAULT;
			goto out_info_opt4;
		}
		if (copy_from_user(new_info->remote_id,
				   compat_ptr(req.remote_id),
				   req.remote_id_size)) {
			ret = -EFAULT;
			goto out_info_opt4;
		}

		old_info = rcu_dereference(p->info_option);
		rcu_assign_pointer(p->info_option, new_info);

		spin_unlock_bh(&br->lock);

		if (old_info) {
			call_rcu(&old_info->rcu, destroy_info_opt);
		}

		return 0;

out_info_opt4:
		kfree(new_info->remote_id);
out_info_opt3:
		kfree(new_info->circuit_id);
out_info_opt2:
		kfree(new_info);
out_info_opt1:
		spin_unlock_bh(&br->lock);
		return ret;
	}

        case ABRCTL_SET_MC_ROUTER: {
		unsigned ret = -ENODEV;
		unsigned portidx = args[1];
		unsigned val = args[2];
		struct net_bridge_port *p = NULL;

		spin_lock_bh(&br->lock);
		if (portidx) {
			list_for_each_entry(p, &br->port_list, list) {
				if (p->dev->ifindex == portidx) {
					local_bh_disable();
					br_multicast_set_port_router(p, val);
					local_bh_enable();
					ret = 0;
					break;
				}
			}
		} else {
			br_multicast_set_router(br, val);
			ret = 0;
		}
		spin_unlock_bh(&br->lock);
		return ret;
	}

        case ABRCTL_SET_MC_OPTS: {
            struct abrctl_mc_opts x;
            if (copy_from_user(&x, (void __user *)args[1], sizeof(x)) != 0)
		    return -EFAULT;

	    spin_lock_bh(&br->multicast_lock);
	    br->multicast_last_member_count = x.last_member_count;
	    br->multicast_startup_query_count = x.startup_query_count;
	    br->multicast_last_member_interval = x.last_member_interval;
	    br->multicast_membership_interval = x.membership_interval;
	    br->multicast_querier_interval = x.querier_interval;
	    br->multicast_query_interval = x.query_interval;
	    br->multicast_query_response_interval = x.query_response_interval;
	    br->multicast_startup_query_interval = x.startup_query_interval;
	    br->multicast_igmp_version = x.igmp_version;
	    br->multicast_mld_version = x.mld_version;
	    spin_unlock_bh(&br->multicast_lock);
	    br_multicast_set_querier(br, x.querier);

	    return 0;
	}

        case ABRCTL_GET_FAST_FORWARD: {
		return br->fast_forward;
	}

	case ABRCTL_GET_MC_QUERIER: {
		struct abrctl_mc_querier queriers[2] = {};
		struct net_bridge_port *port;

		spin_lock_bh(&br->multicast_lock);

		port = rcu_dereference(br->ip4_querier.port);
		queriers[0].portidx = port ? port->dev->ifindex : 0;
		queriers[0].addr.u.ip4 = br->ip4_querier.addr.u.ip4;
		port = rcu_dereference(br->ip6_querier.port);
		queriers[1].portidx = port ? port->dev->ifindex : 0;
		queriers[1].addr.u.ip6 = br->ip6_querier.addr.u.ip6;

		if (copy_to_user((void __user *)args[1], &queriers, sizeof(queriers)))
			return -EFAULT;

		spin_unlock_bh(&br->multicast_lock);

		return 0;
	}

        default:
            break;
	}

	return -EOPNOTSUPP;
}

static int old_deviceless(struct net *net, void __user *uarg)
{
	unsigned long args[3];

	if (copy_from_user(args, uarg, sizeof(args)))
		return -EFAULT;

	switch (args[0]) {
	case BRCTL_GET_VERSION:
		return BRCTL_VERSION;

	case BRCTL_GET_BRIDGES:
	{
		int *indices;
		int ret = 0;

		if (args[2] >= 2048)
			return -ENOMEM;
		indices = kcalloc(args[2], sizeof(int), GFP_KERNEL);
		if (indices == NULL)
			return -ENOMEM;

		args[2] = get_bridge_ifindices(net, indices, args[2]);

		ret = copy_to_user((void __user *)args[1], indices, args[2]*sizeof(int))
			? -EFAULT : args[2];

		kfree(indices);
		return ret;
	}

	case BRCTL_ADD_BRIDGE:
	case BRCTL_DEL_BRIDGE:
	{
		char buf[IFNAMSIZ];

		if (!ns_capable(net->user_ns, CAP_NET_ADMIN))
			return -EPERM;

		if (copy_from_user(buf, (void __user *)args[1], IFNAMSIZ))
			return -EFAULT;

		buf[IFNAMSIZ-1] = 0;

		if (args[0] == BRCTL_ADD_BRIDGE)
			return br_add_bridge(net, buf);

		return br_del_bridge(net, buf);
	}
	}

	return -EOPNOTSUPP;
}

int br_ioctl_deviceless_stub(struct net *net, unsigned int cmd, void __user *uarg)
{
	switch (cmd) {
	case SIOCGIFBR:
	case SIOCSIFBR:
		return old_deviceless(net, uarg);

	case SIOCBRADDBR:
	case SIOCBRDELBR:
	{
		char buf[IFNAMSIZ];

		if (!ns_capable(net->user_ns, CAP_NET_ADMIN))
			return -EPERM;

		if (copy_from_user(buf, uarg, IFNAMSIZ))
			return -EFAULT;

		buf[IFNAMSIZ-1] = 0;
		if (cmd == SIOCBRADDBR)
			return br_add_bridge(net, buf);

		return br_del_bridge(net, buf);
	}
	}
	return -EOPNOTSUPP;
}

int br_dev_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct net_bridge *br = netdev_priv(dev);

	switch (cmd) {
	case SIOCDEVPRIVATE:
		return old_dev_ioctl(dev, rq, cmd);

	case SIOCBRADDIF:
	case SIOCBRDELIF:
		return add_del_if(br, rq->ifr_ifindex, cmd == SIOCBRADDIF);

	}

	br_debug(br, "Bridge does not support ioctl 0x%x\n", cmd);
	return -EOPNOTSUPP;
}
