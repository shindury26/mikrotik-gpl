#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/miscdevice.h>

#include <linux/kernel.h>
#include <linux/socket.h>
#include <linux/if_ether.h>
#include <linux/in.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/udp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <net/sock.h>
#include <net/route.h>
#include <net/ip.h>
#include <net/udp.h>
#include <net/ip6_route.h>
#include <net/xfrm.h>

#include "l2tp.h"
#include "dev.h"
#include "chan.h"
#include "fast_path.h"
#include "l2tp_ioctl.h"

struct socket *l2tp_udp4_socket = NULL;
struct socket *l2tp_udp6_socket = NULL;
struct socket *l2tp_ip6_socket = NULL;
struct socket *l2tp_ip4_socket = NULL;
static struct sk_buff_head l2tp_rxqueue;
static unsigned long l2tp_state = 0;
#define L2TP_ENABLED 0
static wait_queue_head_t l2tp_waitqueue;

typedef int(*l2tp_lockless_rcv_fn)(struct sock *sk, struct sk_buff *skb);
typedef int(*l2tp_llrcv_hdr_fn)(struct sock *sk, struct sk_buff *skb, unsigned short *phdr,
                                uint32_t *fsesid, unsigned short *ftunid, unsigned *data_len);

static int l2tp_init_sockets(void);
static long l2tp_ioctl(struct file *f, unsigned cmd, unsigned long arg);

static void l2tp_queue_local(struct sock *sk, struct sk_buff *skb) {
    if (!sk->sk_socket) {
        printk("l2tp_queue_local: dropping, because no sk_socket\n");
        kfree_skb(skb);
        return;
    }
    memcpy(skb->cb, &sk->sk_socket, sizeof(sk->sk_socket));

    spin_lock_bh(&l2tp_rxqueue.lock);
    if (test_bit(L2TP_ENABLED, &l2tp_state)) {
        __skb_queue_tail(&l2tp_rxqueue, skb);
        skb = NULL;
    }
    spin_unlock_bh(&l2tp_rxqueue.lock);

    if (skb) {
        printk("l2tp_queue_local: dropping, because no listener\n");
        kfree_skb(skb);
    }
    else wake_up_interruptible(&l2tp_waitqueue);
}

static int l2tp_llrcv_udp_hdr(struct sock *sk, struct sk_buff *skb, unsigned short *phdr,
                              uint32_t *fsesid, unsigned short *ftunid, unsigned *data_len) {
    unsigned short ctrl;
    unsigned short flen = 0, foffsize = 0;
    int len = *data_len, ver, isV2;

    ctrl = ntohs(*phdr);
    // pass control frames to userspace without further checking
    if ((ctrl & L2TP_TYPE) != 0) {
        skb_pull(skb, (unsigned char *)phdr - skb->data);
        l2tp_queue_local(sk, skb);
        return 0;
    }
    phdr++;
    len -= 2;

    // check version
    ver = ctrl & L2TP_VERSION_MASK;
    if (ver != L2TP_VERSION2 && ver != L2TP_VERSION3) {
        printk("l2tp_llrcv_udp_hdr: bad version %d\n", ver);
        return -1;
    }

    isV2 = L2TP_VERSION2 == ver;

    // parse the rest of header
    if (isV2 && ((ctrl & L2TP_LENGTH_PRESENT) != 0)) {
        if (len < 2) {
            printk("l2tp_llrcv_udp_hdr: no length field\n");
            return -1;
        }
        flen = ntohs(*phdr++);
        len -= 2;
        if (flen > (skb->len - sizeof(struct udphdr))) {
            printk("l2tp_llrcv_udp_hdr: truncated message?\n");
            return -1;
        }
    }
    else if (L2TP_VERSION3 == ver) { // skip *reserved* v3 udp header field
        phdr++;
        len -= 2;
    }

    if (len < 4) {
        printk("l2tp_llrcv_udp_hdr: no ids\n");
        return -1;
    }

    if (isV2) {
        *ftunid = *phdr++;
        *fsesid = *((u16 *)phdr++);
    }
    else {
        *ftunid = 0;
        *fsesid = *((u32 *)phdr);
        phdr += 2;
    }
    len -= 4;

    if (isV2) {
        if ((ctrl & L2TP_SEQ_PRESENT) != 0) {
            if (len < 4) {
                printk("l2tp_llrcv_udp_hdr: no seq fields\n");
                return -1;
            }
            phdr += 2;
            len -= 4;
        }

        if ((ctrl & L2TP_OFFSET_PRESENT) != 0) {
            if (len < 2) {
                printk("l2tp_llrcv_udp_hdr: no offset size\n");
                return -1;
            }
            foffsize = ntohs(*phdr++);
            len -= 2;

            if (foffsize > len) {
                printk("l2tp_llrcv_udp_hdr: padding too big\n");
                return -1;
            }

            len -= foffsize;
        }
    }

    *data_len = len;

    return 1;
}

static int l2tp_llrcv_ip_hdr(struct sock *sk, struct sk_buff *skb, unsigned short *phdr,
                             uint32_t *fsesid, unsigned short *ftunid,
                             unsigned *len) {
    memcpy(fsesid, phdr, sizeof(*fsesid));
    if (0 == *fsesid) { // 0 session id = ctl message
        skb_pull(skb, (unsigned char *)phdr - skb->data);
        l2tp_queue_local(sk, skb);
        return 0;
    }

    *ftunid = 0;
    *len -= sizeof(uint32_t);

    return 1;
}

static int l2tp_lockless_rcv(struct sock *sk, struct sk_buff *skb,
                             unsigned len_hdr, l2tp_llrcv_hdr_fn parse_header_fn) {
    unsigned short *phdr;
    unsigned len;
    int is_data_msg;

    unsigned short ftunid = 0;
    uint32_t fsesid = 0;

    if (skb_linearize(skb)) goto drop;

    if (skb->len < len_hdr) {
        printk("l2tp_lockless_rcv: short frame, dropping\n");
        goto drop;
    }

    phdr = (unsigned short *)(skb->data + len_hdr);
    len = skb->len - len_hdr;

    // check for minimum len
    if (len < L2TP_MIN_FRAME_LEN) {
        printk("l2tp_lockless_rcv: short frame!\n");
        goto drop;
    }
    is_data_msg = parse_header_fn(sk, skb, phdr, &fsesid, &ftunid, &len);
    if (is_data_msg < 0)
        goto drop;
    else if (!is_data_msg) { // control message received
        return 0;
    }

    // strip off header, and frame should be ready for PPP processing
    skb_pull(skb, skb->len - len);

    // detach skb from socket, so more data can be received and buffer can
    // be freed by kfree_skb (it was possible anyway...)
    skb_orphan(skb);

    skb_dst_drop(skb);
    nf_reset_ct(skb);
    secpath_reset(skb);

    l2tp_receive(fsesid, ftunid, skb);
    return 0;

  drop:
    kfree_skb(skb);
    return 0;
}

static inline int l2tp_lockless_rcv_udp(struct sock *sk, struct sk_buff *skb) {
    return l2tp_lockless_rcv(sk, skb, sizeof(struct udphdr), l2tp_llrcv_udp_hdr);
}

static inline int l2tp_rcv_no_udphdr(struct sock *sk, struct sk_buff *skb) {
    return l2tp_lockless_rcv(sk, skb, 0, l2tp_llrcv_udp_hdr);
}

static inline int l2tp_lockless_rcv_ip(struct sock *sk, struct sk_buff *skb) {
    return l2tp_lockless_rcv(sk, skb, sizeof(struct iphdr), l2tp_llrcv_ip_hdr);
}

static inline int l2tp_lockless_rcv_ip6(struct sock *sk, struct sk_buff *skb) {
    return l2tp_lockless_rcv(sk, skb, 0, l2tp_llrcv_ip_hdr);
}

static void l2tp_dgram_data_ready(struct sock *sk, l2tp_lockless_rcv_fn recv_fn) {
    int err;
    struct sk_buff *skb;

    while ((skb = skb_recv_datagram(sk, 0, 1, &err)) == NULL) {
        if (err == -EAGAIN) {
            printk("l2tp_data_ready: EAGAIN\n");
            return;
        }
        printk("l2tp_data_ready: skb_recv_datagram error %d\n", err);
    }

    recv_fn(sk, skb);
}

static void l2tp_udp_data_ready(struct sock *sk, l2tp_lockless_rcv_fn recv_fn) {
    int err, off;
    struct sk_buff *skb;

    while ((skb = __skb_recv_udp(sk, 0, 1, &off, &err)) == NULL) {
        if (err == -EAGAIN) {
            printk("l2tp_data_ready: EAGAIN\n");
            return;
        }
        printk("l2tp_udp_data_ready: __skb_recv_udp error %d\n", err);
    }

    recv_fn(sk, skb);
}

static inline void l2tp_data_ready_udp4(struct sock *sk) {
    l2tp_udp_data_ready(sk, l2tp_rcv_no_udphdr);
}

static inline void l2tp_data_ready_udp6(struct sock *sk) {
    l2tp_udp_data_ready(sk, l2tp_rcv_no_udphdr);
}

static inline void l2tp_data_ready_ip(struct sock *sk) {
    l2tp_dgram_data_ready(sk, l2tp_lockless_rcv_ip);
}

static inline void l2tp_data_ready_ip6(struct sock *sk) {
    l2tp_dgram_data_ready(sk, l2tp_lockless_rcv_ip6);
}

static inline struct socket* l2tp_skb_cb_socket(struct sk_buff *skb) {
    struct socket *s = *((struct socket **)skb->cb);
    if (s != l2tp_ip4_socket && s != l2tp_udp4_socket
            && s != l2tp_ip6_socket && s != l2tp_udp6_socket) {
        return NULL;
    }

    return s;
}

static ssize_t l2tp_write(struct file *f, const char *b, size_t s, loff_t *l) {
    int err, ip_mode, ipv4, sesid;
    u8 cmsgbuf[256];
    struct cmsghdr *cm;

    memset(cmsgbuf, 0, sizeof(cmsgbuf));

    struct {
        struct sockaddr_in sin[2];
        uint32_t l2tpv3_sesid;
    } in4;
    struct {
        struct sockaddr_in6 sin[2];
        uint32_t l2tpv3_sesid;
    } in6;

    struct msghdr msg;
    struct iovec iov;

//    printk("l2tp_write\n");

    // see if bastard provided at least sockaddr_in
    if (s < (sizeof(in4) + 1)) {
        printk("l2tp_write: no sockaddr_in or no packet data\n");
        return -EIO;
    }

    u8 proto_fam;
    if (copy_from_user(&proto_fam, b, sizeof(proto_fam)) != 0) {
        printk("l2tp_write: failed to read proto family\n");
    }
    ipv4 = AF_INET6 != proto_fam;
    b++;

    if (ipv4) {
        if (copy_from_user(&in4, b, sizeof(in4)) != 0) {
            printk("l2tp_write: failed to read sockaddr_in\n");
            return -EFAULT;
        }
        b +=  sizeof(in4.sin);
    }
    else {
        if (s < (sizeof(in6) + 1)) {
            printk("l2tp_write: no sockaddr_in or no packet data (ipv6)\n");
            return -EIO;
        }

        if (copy_from_user(&in6, b, sizeof(in6)) != 0) {
            printk("l2tp_write: failed to read sockaddr_in6\n");
            return -EFAULT;
        }
        b += sizeof(in6.sin);
    }

    // prepare message and send frame
    memset(&msg, 0, sizeof(msg));
    memset(&iov, 0, sizeof(iov));

    iov.iov_base = (char *)b;
    iov.iov_len = s - (ipv4 ? sizeof(in4.sin) : sizeof(in6.sin)) - sizeof(proto_fam);

    msg.msg_name = ipv4 ? (void *)&in4.sin[0] : (void *)&in6.sin[0];
    msg.msg_namelen = ipv4 ? sizeof(in4.sin[0]) : sizeof(in6.sin[0]);
    msg.msg_iter.iov = &iov;
    msg.msg_iter.count = iov.iov_len;
    msg.msg_iter.nr_segs = 1;
    msg.msg_flags = MSG_DONTWAIT;

    cm = (struct cmsghdr *)cmsgbuf;
    if (ipv4) {
        if (!in4.sin[1].sin_addr.s_addr) {
            printk("l2tp_write: missing source IPv4 address\n");
            return -EFAULT;
        }
	cm->cmsg_level = IPPROTO_IP;
	cm->cmsg_type = IP_PKTINFO;
	cm->cmsg_len = CMSG_LEN(sizeof(struct in_pktinfo));
	struct in_pktinfo *pi = (struct in_pktinfo *)(CMSG_DATA(cm));
	pi->ipi_ifindex = 0;
	pi->ipi_spec_dst.s_addr = in4.sin[1].sin_addr.s_addr;
	pi->ipi_addr.s_addr = in4.sin[1].sin_addr.s_addr;

	msg.msg_control = cm;
	msg.msg_controllen = sizeof(struct in_pktinfo) + sizeof(struct cmsghdr);
    }
    else {
        cm->cmsg_level = IPPROTO_IPV6;
        cm->cmsg_type = IPV6_PKTINFO;
        cm->cmsg_len = CMSG_LEN(sizeof(struct in6_pktinfo));

        struct in6_pktinfo *pi = (struct in6_pktinfo *)(CMSG_DATA(cm));
        pi->ipi6_ifindex = 0;
        pi->ipi6_addr = in6.sin[1].sin6_addr;

        msg.msg_control = cm;
        msg.msg_controllen = sizeof(struct in6_pktinfo) + sizeof(struct cmsghdr);
    }

    sesid = ipv4 ? in4.l2tpv3_sesid : in6.l2tpv3_sesid;
    ip_mode = 0 == sesid; // in l2tpv3 IP packet starts with 0

    if (ipv4)
        err = sock_sendmsg(ip_mode ? l2tp_ip4_socket : l2tp_udp4_socket, &msg);
    else
        err = sock_sendmsg(ip_mode ? l2tp_ip6_socket : l2tp_udp6_socket, &msg);

    if (err < 0) {
        if (net_ratelimit()) {
            printk("l2tp_write: sock_sendmsg failed with %d (%s)\n", err,
                            ip_mode ? "IP" : "UDP");
        }
    }

    // even if sendmsg failed, do not bother caller with this
    // as there is nothing he can do about UDP failing anyway

    return s;
}

static int l2tp_deliver_local(char *b, size_t s, struct sk_buff *skb) {
    const struct socket *skb_sock;
    const struct ethhdr * eth;
    int err, len, udp_mode, ipv4;

    struct {
        union {
            struct sockaddr_in sin[2];
            struct sockaddr_in6 sin6[2];
        };
    } addr;

    skb_sock = l2tp_skb_cb_socket(skb);
    if (!skb_sock) {
        printk("l2tp_deliver_local: could not to detect socket type\n");
        goto err;
    }
    udp_mode = (skb_sock == l2tp_udp4_socket) || (skb_sock == l2tp_udp6_socket);

    ipv4 = 4 == ip_hdr(skb)->version;
    if (!ipv4 && ipv6_hdr(skb)->version != 6) {
        if (net_ratelimit())
            printk("l2tp_deliver_local: unknown protocol version (ipv4 or ipv6 expected)\n");
        goto err;
    }

    memset(&addr, 0, sizeof(addr));
    if (ipv4) {
        addr.sin[0].sin_family = AF_INET;
        if (udp_mode) addr.sin[0].sin_port = udp_hdr(skb)->source;
        addr.sin[0].sin_addr.s_addr = ip_hdr(skb)->saddr;
        addr.sin[1].sin_family = AF_INET;
        if (udp_mode) addr.sin[1].sin_port = udp_hdr(skb)->dest;
        addr.sin[1].sin_addr.s_addr = ip_hdr(skb)->daddr;
    }
    else { // ipv6
        addr.sin6[0].sin6_family = AF_INET6;
        if (udp_mode) addr.sin6[0].sin6_port = udp_hdr(skb)->source;
        addr.sin6[0].sin6_addr = ipv6_hdr(skb)->saddr;
        addr.sin6[1].sin6_family = AF_INET6;
        if (udp_mode) addr.sin6[1].sin6_port = udp_hdr(skb)->dest;
        addr.sin6[1].sin6_addr = ipv6_hdr(skb)->daddr;
    }

    err = copy_to_user(b, &addr, sizeof(addr));
    if (err != 0) {
        printk("l2tp_deliver_local: failed to copy sockaddr_in\n");
        goto err;
    }

    len = s - sizeof(addr);

    err = copy_to_user(b + sizeof(addr), skb->data, len);
    if (err != 0) {
        printk("l2tp_deliver_local: failed to copy data (%d)\n", len);
        goto err;
    }

    kfree_skb(skb);

    return len + sizeof(addr);

err:
    kfree_skb(skb);
    return -EFAULT;
}

static inline int l2tp_has_local(void) {
    int ret;
    spin_lock_bh(&l2tp_rxqueue.lock);
    ret = skb_queue_len(&l2tp_rxqueue);
    spin_unlock_bh(&l2tp_rxqueue.lock);
    return ret;
}

static unsigned l2tp_poll(struct file *f, poll_table *w) {
    unsigned m = POLLOUT | POLLWRNORM;

//    printk("l2tp_poll\n");

    poll_wait(f, &l2tp_waitqueue, w);

    if (l2tp_has_local()) {
//        printk("l2tp_poll: can read immediately\n");
        m |= POLLIN | POLLRDNORM;
    }

    return m;
}

static ssize_t l2tp_read(struct file *f, char *b, size_t s, loff_t *l) {
    int err;
    struct sk_buff *skb;

//    printk("l2tp_read\n");

    // see if bastard has provided enough space for sockaddr_in
    if (s < sizeof(struct sockaddr_in)) {
        printk("l2tp_read: too short buffer (%d)\n", (int) s);
        return EIO;
    }

    if (f->f_flags & O_NONBLOCK) goto deliver;

    err = wait_event_interruptible(l2tp_waitqueue, l2tp_has_local());
    if (err != 0) {
        printk("l2tp_read: wait_event_interruptible failed (%d)\n", err);
        return -EINTR;
    }

  deliver:
    spin_lock_bh(&l2tp_rxqueue.lock);
    skb = __skb_dequeue(&l2tp_rxqueue);
    spin_unlock_bh(&l2tp_rxqueue.lock);

    if (!skb) {
        printk("l2tp_read: no data to deliver\n");
    }

    return skb ? l2tp_deliver_local(b, s, skb) : -EAGAIN;
}

static int l2tp_open(struct inode *inode, struct file *file) {
    if (!capable(CAP_SYS_ADMIN)) {
        printk("l2tp_open: not superuser?\n");
        return -EPERM;
    }
    if (test_and_set_bit(L2TP_ENABLED, &l2tp_state)) {
        printk("l2tp_open: already open\n");
        return -EBUSY;
    }
    return 0;
}

static int l2tp_release(struct inode *inode, struct file *file) {
    struct sk_buff *skb;

    printk("l2tp_release\n");

    // delete all channels as controlling program does not want to talk
    // to us any more
    l2tp_del_all();

    spin_lock_bh(&l2tp_rxqueue.lock);

    // purge packet queue
    clear_bit(L2TP_ENABLED, &l2tp_state);
    while ((skb = __skb_dequeue(&l2tp_rxqueue)) != 0) {
        printk("l2tp_release: dropping from queue\n");
        kfree_skb(skb);
    }

    spin_unlock_bh(&l2tp_rxqueue.lock);

    return 0;
}

static void l2t_lookup_xfrm6(struct l2tp_add *a) {
    struct flowi6 fl6;
    struct dst_entry *dst = NULL;
    struct inet_sock* inet;
    struct sock *sk;
    int udp_mode;

    udp_mode = a->data.in.l2tp_ver != L2TPv3_IP;
    sk = udp_mode ? l2tp_udp6_socket->sk : l2tp_ip6_socket->sk;
    inet = inet_sk(sk);

    memset(&fl6, 0, sizeof(fl6));
    fl6.flowi6_proto = udp_mode ? IPPROTO_UDP : L2TPV3_PROTO;
    memcpy(fl6.saddr.in6_u.u6_addr8, a->data.in.src_addr6, sizeof(fl6.saddr.in6_u.u6_addr8));
    memcpy(fl6.daddr.in6_u.u6_addr8, a->data.in.remote_addr6, sizeof(fl6.daddr.in6_u.u6_addr8));
    if (udp_mode) {
        fl6.fl6_sport = inet->inet_sport;
        fl6.fl6_dport = a->data.in.remote_port;
    }

    dst = ip6_dst_lookup_flow(sock_net(sk), sk, &fl6, NULL);
    if (!dst) return;

    if (dst->xfrm) {
        struct xfrm_state *xfrm = dst->xfrm;
        char *name = a->data.out.crypto;

        if (xfrm->ealg) {
            strlcat(name, xfrm->ealg->alg_name, CRYPTOLEN);
        }
        if (xfrm->aalg) {
            if (name[0]) strlcat(name, " + ", CRYPTOLEN);
            strlcat(name, xfrm->aalg->alg_name, CRYPTOLEN);
        }
        if (xfrm->calg) {
            if (name[0]) strlcat(name, " + ", CRYPTOLEN);
            strlcat(name, xfrm->calg->alg_name, CRYPTOLEN);
        }
        if (xfrm->aead) {
            if (name[0]) strlcat(name, " + ", CRYPTOLEN);
            strlcat(name, xfrm->aead->alg_name, CRYPTOLEN);
        }
    }
    a->data.out.gw_iface = dst->dev->devid;

    dst_release(dst);
}

static void l2tp_lookup_xfrm(struct l2tp_add *a) {
    struct inet_sock *inet;
    struct flowi4 fl4;
    struct rtable *rt;
    struct sock *sk;
    int udp_mode;
    unsigned raddr;

    if (AF_INET6 == a->data.in.address_family) {
        l2t_lookup_xfrm6(a);
        return;
    }

    raddr = a->data.in.remote_addr;

    udp_mode = a->data.in.l2tp_ver != L2TPv3_IP;
    sk = udp_mode ? l2tp_udp4_socket->sk : l2tp_ip4_socket->sk;
    inet = inet_sk(sk);

    __u8 protocol = udp_mode ? IPPROTO_UDP : L2TPV3_PROTO;
    rt = ip_route_output_ports(&init_net, &fl4, sk,
			       a->data.in.remote_addr, a->data.in.src_addr,
			       a->data.in.remote_port, inet->inet_sport,
			       protocol, 0, 0);
    memset(a->data.out.crypto, 0, sizeof(a->data.out.crypto));

    a->data.out.gateway = 0;
    a->data.out.gw_iface = 0;
    if (!IS_ERR(rt)) {
	if (rt->dst.xfrm) {
	    struct xfrm_state *xfrm = rt->dst.xfrm;
	    char *name = a->data.out.crypto;

	    if (xfrm->ealg) {
		strlcat(name, xfrm->ealg->alg_name, CRYPTOLEN);
	    }
	    if (xfrm->aalg) {
		if (name[0]) strlcat(name, " + ", CRYPTOLEN);
		strlcat(name, xfrm->aalg->alg_name, CRYPTOLEN);
	    }
	    if (xfrm->calg) {
		if (name[0]) strlcat(name, " + ", CRYPTOLEN);
		strlcat(name, xfrm->calg->alg_name, CRYPTOLEN);
	    }
	    if (xfrm->aead) {
		if (name[0]) strlcat(name, " + ", CRYPTOLEN);
		strlcat(name, xfrm->aead->alg_name, CRYPTOLEN);
	    }
	}
	if (rt->rt_gw4 != raddr) {
	    a->data.out.gateway = rt->rt_gw4;
	} else {
	    a->data.out.gw_iface = rt->dst.dev->devid;
	}
	ip_rt_put(rt);
    }
}

static struct file_operations l2tp_fops = {
    owner: THIS_MODULE,
    read: l2tp_read,
    write: l2tp_write,
    poll: l2tp_poll,
    unlocked_ioctl: l2tp_ioctl,
    compat_ioctl: l2tp_ioctl,
    open: l2tp_open,
    release: l2tp_release,
};

static struct miscdevice l2tp_miscdev = {
    minor: MISC_DYNAMIC_MINOR,
    name: "l2tp",
    fops: &l2tp_fops,
};

static long l2tp_ioctl(struct file *f,
		       unsigned cmd, unsigned long arg) {
    int err = 0;

    switch (cmd) {
    case L2TP_IOCADD: {
        struct l2tp_add a;
	int channel;

        if (copy_from_user(&a, (void *)arg, sizeof(a)) != 0) {
            printk("l2tp_ioctl: add failed to copy arg\n");
            err = -EFAULT;
            break;
        }

        channel = l2tp_add_channel(&a);

        if (channel < 0) {
	    err = channel;
	    break;
	}

	l2tp_lookup_xfrm(&a);
	a.data.out.channel = channel;

	if (copy_to_user((void *)arg, &a, sizeof(a)) != 0) {
	    printk("l2tp_ioctl: add failed to write result\n");
	    err = -EFAULT;
        }

        break;
    }

    case L2TP_IOCDEL: {
        struct l2tp_del a;
        if (copy_from_user(&a, (void *)arg, sizeof(a)) != 0) {
            printk("l2tp_ioctl: del failed to copy arg\n");
            err = -EFAULT;
            break;
        }

        err = l2tp_del_channel(a.index);

        break;
    }

    case L2TP_IOCADD_ETH: {
        struct l2tp_add a;
        int channel;

        if (copy_from_user(&a, (void *)arg, sizeof(a)) != 0) {
            printk("l2tp_ioctl: add (eth) failed to copy arg\n");
            err = -EFAULT;
            break;
        }

        channel = l2tp_add_ethchan(&a);

        if (channel < 0) {
            err = channel;
            break;
        }

        l2tp_lookup_xfrm(&a);
        a.data.out.channel = channel;

        if (copy_to_user((void *)arg, &a, sizeof(a)) != 0) {
            printk("l2tp_ioctl: add (eth) failed to write result\n");
            l2tp_del_channel(channel);
            err = -EFAULT;
        }

        break;
    }

    case L2TP_IOCIDLE_ETH: {
            struct l2tp_idle_info a;
            int err;

            if (copy_from_user(&a, (void *)arg, sizeof(a)) != 0) {
                printk("l2tp_ioctl: get idle (eth) failed to copy arg\n");
                err = -EFAULT;
                break;
            }

            err = l2tp_eth_getidle(&a);

            if (err) {
                err = a.channel;
                break;
            }

            if (copy_to_user((void *)arg, &a, sizeof(a)) != 0) {
                printk("l2tp_ioctl: add (eth) failed to write result\n");
                err = -EFAULT;
            }

            break;
        }

    default:
        printk("l2tp_ioctl: unknown ioctl %d\n", cmd);
        err = -EIO;
        break;
    }

    return err;
}

static int l2tp_init_sockets(void) {
    int err = EIO;
    struct sockaddr_in sinip4, sinudp4;
    struct sockaddr_in6 sinudp6, sinip6;
    l2tp_udp4_socket = NULL;
    l2tp_udp6_socket = NULL;
    l2tp_ip4_socket = NULL;
    l2tp_ip6_socket = NULL;

    err = sock_create(PF_INET, SOCK_RAW, L2TPV3_PROTO, &l2tp_ip4_socket);
    if (err != 0) {
        printk("l2tp_init_socket: failed to create IP (over IPv4) socket (%d)\n", err);
        goto out;
    }

    err = sock_create(PF_INET6, SOCK_RAW, L2TPV3_PROTO, &l2tp_ip6_socket);
    if (err != 0) {
        printk("l2tp_init_socket: failed to create IP (over IPv6) socket (%d)\n", err);
        goto out;
    }

    err = sock_create(PF_INET, SOCK_DGRAM, 0, &l2tp_udp4_socket);
    if (err != 0) {
        printk("l2tp_init_socket: failed to create IPv4 udp socket (%d)\n", err);
        goto out;
    }

    err = sock_create(PF_INET6, SOCK_DGRAM, 0, &l2tp_udp6_socket);
    if (err != 0) {
        printk("l2tp_init_socket: failed to create IPv6 udp socket (%d)\n", err);
        goto out;
    }

    printk("l2tp_init_socket: socket created\n");

    l2tp_udp4_socket->sk->sk_reuse = 1;
    l2tp_udp4_socket->sk->sk_allocation = GFP_ATOMIC;

    l2tp_udp6_socket->sk->sk_reuse = 1;
    l2tp_udp6_socket->sk->sk_allocation = GFP_ATOMIC;
    l2tp_udp6_socket->sk->sk_ipv6only = 1;

    l2tp_ip4_socket->sk->sk_reuse = 1;
    l2tp_ip4_socket->sk->sk_allocation = GFP_ATOMIC;
    l2tp_ip6_socket->sk->sk_reuse = 1;
    l2tp_ip6_socket->sk->sk_allocation = GFP_ATOMIC;

    memset(&sinip4, 0, sizeof(sinip4));
    sinip4.sin_family = AF_INET;

    memset(&sinip6, 0, sizeof(sinip6));
    sinip6.sin6_family = AF_INET6;
    sinip6.sin6_addr = in6addr_any;

    memset(&sinudp4, 0, sizeof(sinudp4));
    sinudp4.sin_family = AF_INET;
    sinudp4.sin_port = htons(1701);

    memset(&sinudp6, 0, sizeof(sinudp6));
    sinudp6.sin6_family = AF_INET6;
    sinudp6.sin6_port = htons(1701);
    sinudp6.sin6_addr = in6addr_any;

    err = l2tp_ip4_socket->ops->bind(l2tp_ip4_socket, (struct sockaddr *)&sinip4,
                                    sizeof(sinip4));
    err |= l2tp_ip6_socket->ops->bind(l2tp_ip6_socket, (struct sockaddr *)&sinip6,
                                    sizeof(sinip6));
    err |= l2tp_udp4_socket->ops->bind(l2tp_udp4_socket, (struct sockaddr *)&sinudp4,
                                       sizeof(sinudp4));
    err |= l2tp_udp6_socket->ops->bind(l2tp_udp6_socket, (struct sockaddr *)&sinudp6,
                                       sizeof(sinudp6));
    if (err != 0) {
        printk("l2tp_init_socket: bind failed (%d)\n", err);
        goto out;
    }

    l2tp_udp4_socket->sk->sk_data_ready = &l2tp_data_ready_udp4;
    l2tp_udp4_socket->sk->sk_lockless_rcv = &l2tp_lockless_rcv_udp;

    l2tp_udp6_socket->sk->sk_data_ready = &l2tp_data_ready_udp6;
    l2tp_udp6_socket->sk->sk_lockless_rcv = &l2tp_lockless_rcv_udp;

    l2tp_ip4_socket->sk->sk_data_ready = &l2tp_data_ready_ip;
    l2tp_ip4_socket->sk->sk_lockless_rcv = &l2tp_lockless_rcv_ip;

    l2tp_ip6_socket->sk->sk_data_ready = &l2tp_data_ready_ip6;
    l2tp_ip6_socket->sk->sk_lockless_rcv = &l2tp_lockless_rcv_ip6;

    l2tp_fp_init();

    return 0;
out:
    if(l2tp_udp4_socket) sock_release(l2tp_udp4_socket);
    if(l2tp_udp6_socket) sock_release(l2tp_udp6_socket);
    if(l2tp_ip4_socket) sock_release(l2tp_ip4_socket);
    if(l2tp_ip6_socket) sock_release(l2tp_ip6_socket);

    l2tp_udp4_socket = NULL;
    l2tp_udp6_socket = NULL;
    l2tp_ip4_socket = NULL;
    l2tp_ip6_socket = NULL;
    return err;
}

static int __init l2tp_init(void) {
    int err;

    printk("l2tp_init\n");

    skb_queue_head_init(&l2tp_rxqueue);
    init_waitqueue_head(&l2tp_waitqueue);

    // register character driver
    err = misc_register(&l2tp_miscdev);
    if (err) {
        printk("l2tp: failed to register char device\n");
        goto err_char;
    }

    // create sockets
    err = l2tp_init_sockets();
    if (0 == err) {
        return 0;
    }

    misc_deregister(&l2tp_miscdev);
  err_char:
    return EIO;
}

static void l2tp_exit(void) {
    printk("l2tp_exit\n");
    l2tp_fp_exit();
    if(l2tp_udp4_socket) sock_release(l2tp_udp4_socket);
    if(l2tp_udp6_socket) sock_release(l2tp_udp6_socket);
    if(l2tp_ip4_socket) sock_release(l2tp_ip4_socket);
    if(l2tp_ip6_socket) sock_release(l2tp_ip6_socket);
    misc_deregister(&l2tp_miscdev);
}

module_init(l2tp_init);
module_exit(l2tp_exit);

MODULE_LICENSE("GPL");
