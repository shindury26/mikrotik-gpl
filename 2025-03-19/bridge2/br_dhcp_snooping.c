#include <net/ip.h>
#include <net/udp.h>
#include <linux/ratelimit.h>
#include <linux/log.h>

#include "br_private.h"

struct dhcphdr {
    unsigned char op;
    unsigned char htype;
    unsigned char hlen;
    unsigned char hops;
    unsigned xid;
    unsigned short secs;
    unsigned short flags;
    unsigned ciaddr;
    unsigned yiaddr;
    unsigned siaddr;
    unsigned giaddr;
    unsigned char chaddr[16];
    unsigned char sname[64];
    unsigned char file[128];
    unsigned char options[0];
};

struct opt {
    unsigned char type;
    unsigned char len;
    unsigned char value[0];
};

enum {
    BOOTREQUEST = 1,
    BOOTREPLY = 2,
};

enum {
    DHCP_HOST_NAME = 12,
    DHCP_REQ_ADDR = 50,
    DHCP_LEASE_TIME = 51,
    DHCP_MSG_TYPE = 53,
    DHCP_SERVER_ID = 54,
    DHCP_CLIENT_IDENTIFIER = 61,
    DHCP_RELAY_AGENT = 82,
};

enum {
    DHCP_DISCOVER = 1,
    DHCP_OFFER = 2,
    DHCP_REQUEST = 3,
    DHCP_ACK = 5,
    DHCP_RELEASE = 7,
};

void destroy_info_opt(struct rcu_head *head)
{
	struct net_dhcp_info_opt *info_opt =
			container_of(head, struct net_dhcp_info_opt, rcu);
	kfree(info_opt->circuit_id);
	kfree(info_opt->remote_id);
	kfree(info_opt);
}

static struct opt *find_opt(unsigned char *opts, unsigned char *end,
			    unsigned type) {
	struct opt *c = (struct opt *) opts;
	while (true) {
		if ((unsigned char *) c + sizeof(struct opt) > end) {
			if ((unsigned char *) c + 1 > end) break;
			if (type == 255 && c->type == 255) return c;
			break;
		}
		if (c->type == 255) {
			if (type == 255) return c;
			else break;
		}
		if (c->value + c->len > end) break;
		if (c->type == type) return c;
		c = (struct opt *) (c->value + c->len);
	}
	return 0;
}

static unsigned put_relay_agent_info(unsigned char *buf,
				     struct net_dhcp_info_opt *info,
				     unsigned circuit_id_len,
				     unsigned remote_id_len,
				     char *vid_str,
				     unsigned vid_len) {
	struct opt *opt = (struct opt *) buf;
	opt->type = 1;
	opt->len = circuit_id_len + vid_len;
	memcpy(opt->value, info->circuit_id, circuit_id_len);
	if (vid_len) {
		memcpy(opt->value + circuit_id_len, vid_str, vid_len);
	}
	opt = (struct opt *) (opt->value + opt->len);
	opt->type = 2;
	opt->len = remote_id_len;
	memcpy(opt->value, info->remote_id, remote_id_len);
	return opt->value + opt->len - buf;
}

static int br_add_info_opt(struct sk_buff *skb, struct net_dhcp_info_opt *info,
			   struct dhcphdr *dhcph, struct opt *r, u16 vid)
{
	unsigned circuit_id_len = strlen(info->circuit_id);
	unsigned remote_id_len = strlen(info->remote_id);
	int opt_len = sizeof(struct opt) * 3 +
		circuit_id_len + remote_id_len + 1;
	unsigned vid_len = 0;
	char vid_str[6] = {0, 0, 0, 0, 0, 0};
	if (vid) {
		vid_len = snprintf(vid_str, 6, ":%d", vid);
		opt_len += vid_len;
	}

	int free = skb_tail_pointer(skb) - (unsigned char *) r;
	if (free < opt_len) {
		opt_len -= free;
		if (!pskb_may_pull(skb, opt_len)) {
			return -EINVAL;
		}
		__skb_put(skb, opt_len);
	}
	else {
		opt_len = 0;
	}

	r->type = DHCP_RELAY_AGENT;
	r->len = put_relay_agent_info(r->value, info, circuit_id_len,
			remote_id_len, vid_str, vid_len);
	unsigned char *e = r->value + r->len;
	*e = 255;

	return 0;
}

static void br_checksum_udp(struct sk_buff *skb, unsigned len)
{
	struct iphdr *iph = ip_hdr(skb);
	struct udphdr *udph = udp_hdr(skb);

	udph->len = htons(len);
	udph->check = 0;

	__wsum csum = csum_partial(udph, len, 0);
	udph->check = csum_tcpudp_magic(iph->saddr, iph->daddr, len,
			iph->protocol, csum);
	if (udph->check == 0)
		udph->check = CSUM_MANGLED_0;

	len += ip_hdrlen(skb);
	iph->tot_len = htons(len);
	iph->check = 0;
	iph->check = ip_fast_csum((u8 *)iph, iph->ihl);
}


static int br_check_udp(struct net_bridge *br, struct net_bridge_port *port,
			struct sk_buff *skb, u16 vid)
{
	struct udphdr *udph;
	struct dhcphdr *dhcph;
	unsigned int len;
	int ret = -EINVAL;

	if (!pskb_may_pull(skb, sizeof(struct udphdr)))
		return -EINVAL;

	udph = udp_hdr(skb);

	if (udph->source != htons(67) && udph->dest != htons(67))
		return 0;

	len = ntohs(udph->len);
	if (skb->len < len || len < sizeof(struct dhcphdr))
		return -EINVAL;

	if (!pskb_may_pull(skb, sizeof(*dhcph)))
		return -EINVAL;

	dhcph = (struct dhcphdr	*) (skb_transport_header(skb) + sizeof(*udph));
	if (dhcph->op == BOOTREPLY && !(port->flags & BR_TRUSTED_PORT)) {
		if (__ratelimit(&br_warn_ratelimit_state)) {
			log_message(bridge_warning_topics,
				    "*%08x: received DHCP server message "
				    "on untrusted port from source IP %pI4, "
				    "MAC %pM",
				    port->dev->ifindex,
				    &ip_hdr(skb)->saddr,
				    eth_hdr(skb)->h_source);
		}
		return -EINVAL;
	}

	unsigned char *end = skb_tail_pointer(skb);
	unsigned char *opts = dhcph->options;
	if (opts + 4 > end || get_unaligned((u32 *) opts) != htonl(0x63825363))
		return 0;
	opts += 4;

	struct opt *type = find_opt(opts, end, DHCP_MSG_TYPE);
	if (!type) {
		if (dhcph->op == BOOTREQUEST) {
			BR_INPUT_SKB_CB(skb)->dhcp_req = 1;
		}
		else if (dhcph->op == BOOTREPLY) {
			BR_INPUT_SKB_CB(skb)->dhcp_rep = 1;
		}
		else {
			return -EINVAL;
		}
		return 0;
	}
	if (!type || type->len != 1)
		return -EINVAL;

	struct opt *eopt = find_opt(opts, end, 255);
	if (!eopt)
		return -EINVAL;

	if (type->value[0] == DHCP_DISCOVER || type->value[0] == DHCP_REQUEST ||
	    type->value[0] == DHCP_RELEASE) {
		struct opt *agent = find_opt(opts, end, DHCP_RELAY_AGENT);
		if (!(port->flags & BR_TRUSTED_PORT) && agent)
			return -EINVAL;

		BR_INPUT_SKB_CB(skb)->dhcp_req = 1;

		if (br->add_info_option && !agent) {
			struct net_dhcp_info_opt *info;
			info = rcu_dereference(port->info_option);
			if (info) {
				ret = br_add_info_opt(skb, info, dhcph, eopt, vid);
				if (ret < 0)
					return ret;
				br_checksum_udp(skb, skb->len);
			}
		}
		ret = 0;
	} else if (type->value[0] == DHCP_OFFER || type->value[0] == DHCP_ACK) {
		BR_INPUT_SKB_CB(skb)->dhcp_rep = 1;
		ret = 0;
	}

	return ret;
}

struct sk_buff *br_dhcp_remove_agent_info(struct sk_buff *skb)
{
	struct sk_buff *skb2 = skb_copy(skb, GFP_ATOMIC);
	if (!skb2) {
		kfree_skb(skb);
		return NULL;
	}

	kfree_skb(skb);
	skb = skb2;

	struct dhcphdr *dhcph = (struct dhcphdr	*)
		(skb_transport_header(skb) + sizeof(struct udphdr));
	unsigned char *end = skb_tail_pointer(skb);
	unsigned char *opts = dhcph->options;
	opts += 4;

	struct opt *agent = find_opt(opts, end, DHCP_RELAY_AGENT);
	if (!agent)
		return skb;

	unsigned char *ptr = (unsigned char *) agent;
	unsigned agent_len = sizeof(struct opt) + agent->len;
	unsigned pkt_len = end - (unsigned char *) dhcph;
	memmove(ptr, ptr + agent_len, end - ptr - agent_len);
	struct opt *eopt = find_opt(opts, end, 255);
	if (eopt) {
		ptr = (unsigned char *) eopt;
		++ptr;
		if (ptr >= end) {
			goto out;
		}
		if (pkt_len > 300 && pkt_len - agent_len < 300) {
			/* trim to minimum dhcp packet size */
			skb_trim(skb, skb->len - (pkt_len - 300));
			end = skb_tail_pointer(skb);
		}
		if (ptr < end) {
			memset(ptr, 0, end - ptr);
		}
	}
out:
	br_checksum_udp(skb2, end - skb_transport_header(skb));
	return skb2;
}

int br_dhcp_rcv(struct net_bridge *br, struct net_bridge_port *port,
		     struct sk_buff *skb, u16 vid)
{
	struct sk_buff *skb2 = skb;
	struct iphdr *iph;
	int len;
	unsigned offset;
	int ret;

	BR_INPUT_SKB_CB(skb)->dhcp_req = 0;
	BR_INPUT_SKB_CB(skb)->dhcp_rep = 0;

	if (!br->dhcp_snooping)
		return 0;

	if (skb->protocol != htons(ETH_P_IP))
		return 0;

	/* We treat OOM as packet loss for now. */
	if (!pskb_may_pull(skb, sizeof(*iph)))
		return -EINVAL;

	iph = ip_hdr(skb);

	if (iph->ihl < 5 || iph->version != 4)
		return -EINVAL;

	if (!pskb_may_pull(skb, ip_hdrlen(skb)))
		return -EINVAL;

	iph = ip_hdr(skb);

	if (unlikely(ip_fast_csum(iph, iph->ihl)))
		return -EINVAL;

	if (iph->protocol != IPPROTO_UDP)
		return 0;

	len = ntohs(iph->tot_len);
	if (skb->len < len || len < ip_hdrlen(skb))
		return -EINVAL;

	if (skb->len > len) {
		skb2 = skb_clone(skb, GFP_ATOMIC);
		if (!skb2)
			return -ENOMEM;

		ret = pskb_trim_rcsum(skb2, len);
		if (ret)
			goto err_out;
	}

	len -= ip_hdrlen(skb2);
	offset = skb_network_offset(skb2) + ip_hdrlen(skb2);
	__skb_pull(skb2, offset);
	skb_reset_transport_header(skb2);

	ret = br_check_udp(br, port, skb2, vid);

	__skb_push(skb2, offset);
err_out:
	if (skb2 != skb)
		kfree_skb(skb2);
	return ret;
}
