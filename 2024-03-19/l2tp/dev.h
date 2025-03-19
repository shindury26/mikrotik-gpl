#ifndef _L2TP_DEV_H_
#define _L2TP_DEV_H_

#include <linux/net.h>

extern struct socket *l2tp_udp4_socket;
extern struct socket *l2tp_udp6_socket;
extern struct socket *l2tp_ip4_socket;
extern struct socket *l2tp_ip6_socket;

#endif
