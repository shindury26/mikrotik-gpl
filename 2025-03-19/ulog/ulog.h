/* Header file for IP tables userspace logging, Version 1.8
 *
 * (C) 2000-2002 by Harald Welte <laforge@gnumonks.org>
 * 
 * Distributed under the terms of GNU GPL */

#ifndef _ULOG_H
#define _ULOG_H

#define ULOG_PREFIX_LEN	32

#define ULOG_MAX_QLEN	50
/* Why 50? Well... there is a limit imposed by the slab cache 131000
 * bytes. So the multipart netlink-message has to be < 131000 bytes.
 * Assuming a standard ethernet-mtu of 1500, we could define this up
 * to 80... but even 50 seems to be big enough. */

/* private data structure for each rule with a ULOG target */
struct ulog_info {
	unsigned int nl_group;
	unsigned copy_range;
	unsigned qthreshold;
	char prefix[ULOG_PREFIX_LEN];
	unsigned pad; // pad to 8 bytes to work on CHR
};

/* Format of the ULOG packets passed through netlink */
typedef struct ulog_packet_msg {
	int indev_idx;
	int outdev_idx;
	unsigned short ingress_prio;
	unsigned short egress_prio;
	char prefix[ULOG_PREFIX_LEN];
	char type; // 0 - ebt, 1 - ipt, 2 - ip6t
	unsigned char mac[6];
	int inphysdev_idx;
	int outphysdev_idx;

	bool ct;
	unsigned ct_src1[4];
	unsigned ct_src2[4];
	unsigned ct_dst1[4];
	unsigned ct_dst2[4];
	unsigned short ct_src1_port;
	unsigned short ct_src2_port;
	unsigned short ct_dst1_port;
	unsigned short ct_dst2_port;

	unsigned char payload[0];
} ulog_packet_msg_t;


#ifdef __KERNEL__

void ulog_target(const struct sk_buff *skb, const struct net_device *in,
		 const struct net_device *out,
		 struct ulog_info *loginfo, char type);

#endif


#endif /*_ULOG_H*/
