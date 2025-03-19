#ifndef ADDITIONAL_BRCTL_H
#define ADDITIONAL_BRCTL_H

#include <linux/types.h>

#ifndef __KERNEL__
#define BIT(x) (1 << (x))
#endif

//#define BR_HAIRPIN_MODE		BIT(0)
//#define BR_BPDU_GUARD		BIT(1)
//#define BR_ROOT_BLOCK		BIT(2)
#define BR_MULTICAST_FAST_LEAVE	BIT(3)
//#define BR_ADMIN_COST		BIT(4)
#define BR_LEARNING		BIT(5)
#define BR_FLOOD		BIT(6)
#define BR_AUTO_MASK		(BR_FLOOD | BR_LEARNING)
//#define BR_PROMISC		BIT(7)
//#define BR_PROXYARP		BIT(8)
//#define BR_LEARNING_SYNC	BIT(9)
//#define BR_PROXYARP_WIFI	BIT(10)
#define BR_MCAST_FLOOD		BIT(11)
//#define BR_MULTICAST_TO_UNICAST	BIT(12)
//#define BR_VLAN_TUNNEL		BIT(13)
#define BR_BCAST_FLOOD		BIT(14)
#define BR_MLAG_PEER_LINK	BIT(29)
#define BR_MLAG_DUAL_LINK	BIT(30)
#define BR_TRUSTED_PORT		BIT(31)

#define BR_DEFAULT		(BR_LEARNING | BR_FLOOD | BR_MCAST_FLOOD | BR_BCAST_FLOOD)

#define BR_NOHORIZON 0
#define BR_NOSWITCH -1u

#define BRFDB_LOCAL 0
#define BRFDB_STATIC 10
#define BRFDB_LEARNED 20
#define BRFDB_ALL 0xffff

#define BR_IN_FRAME_TYPES_OFF 0
#define BR_IN_FRAME_TYPES 0x3
#define BR_PORT_FRAME_ADMIT_ALL 0
#define BR_PORT_FRAME_ADMIT_TAGGED 1
#define BR_PORT_FRAME_ADMIT_UNTAGGED 2
#define BR_IN_FILTERING_OFF 2
#define BR_IN_FILTERING BIT(BR_IN_FILTERING_OFF)
#define BR_IN_TAG_STACKING_OFF 3
#define BR_IN_TAG_STACKING BIT(BR_IN_TAG_STACKING_OFF)

#define ABRCTL_FDB_INSERT 100
#define ABRCTL_FDB_DELETE 101
#define ABRCTL_SET_STP_STATE 102 // port in arg0, mstid in arg1, state in arg2
#define ABRCTL_SET_PORTHORIZON 103 // port in arg0, horizon in arg1
#define ABRCTL_SET_SWITCH_GROUP 104 // port in arg0, switch_group in arg1
#define ABRCTL_GET_MDB 105
#define ABRCTL_GET_MC_ROUTER 106
#define ABRCTL_FDB_FLUSH 107 // port in arg0, vid in arg1, dynamic/static in arg1
#define ABRCTL_ADD_VLAN 108 // port in arg1, vid in arg2, flags in arg3
#define ABRCTL_DEL_VLAN 109 // port in arg1, vid in arg2
#define ABRCTL_SET_INSTANCE 110 // abrctl_vlan_range
#define ABRCTL_DEL_INSTANCE 111 // mstid in arg1
#define ABRCTL_SET_VLAN_OPTS 112 // port in arg1, frameType in arg2, ingFilter in arg3
#define ABRCTL_SET_PORT_OPTS 113 // port in arg1, flags in arg2
#define ABRCTL_SET_INFO_OPT 114 // port in arg1, bridge_info_opt in arg2
#define ABRCTL_SET_MC_ROUTER 115
#define ABRCTL_SET_MC_OPTS 116
#define ABRCTL_GET_FAST_FORWARD 117
#define ABRCTL_GET_MC_QUERIER 118

struct abrctl_fdb_insert {
    unsigned char addr[6];
    unsigned type;
    unsigned portidx;
};

struct abrctl_fdb_entry {
	__u8 mac_addr[6];
	__u8 port_no;
	__u8 is_local;
	__u32 ageing_timer_value;
	__u8 port_hi;
	__u8 is_static;
	__u16 vlan_id;
};

// include/uapi/linux/if_bridge.h
struct abrctl_mdb_entry {
	__u32 ifindex;
#define MDB_TEMPORARY 0
#define MDB_PERMANENT 1
	//__u8 state;
	__u16 vid;
	struct {
		union {
			__be32 ip4;
			struct in6_addr ip6;
		} u;
		__be16		proto;
	} addr;
};

struct abrctl_vlan_range {
	unsigned	arg;
	unsigned	range_count;
	const unsigned	*vlan_ranges;
};

struct bridge_vlan_range {
	unsigned	range_count;
	__u16		*vlan_ranges;
};

struct bridge_info_opt {
	unsigned circuit_id_size;
	unsigned circuit_id;
	unsigned remote_id_size;
	unsigned remote_id;
};

struct abrctl_mc_opts {
	unsigned last_member_count;
	unsigned startup_query_count;
	unsigned last_member_interval;
	unsigned membership_interval;
	unsigned querier_interval;
	unsigned query_interval;
	unsigned query_response_interval;
	unsigned startup_query_interval;
	unsigned char querier;
	unsigned char igmp_version;
	unsigned char mld_version;
};

struct abrctl_mc_querier {
	struct {
		union {
			__be32 ip4;
			struct in6_addr ip6;
		} u;
	} addr;
	unsigned portidx;
};

#endif
