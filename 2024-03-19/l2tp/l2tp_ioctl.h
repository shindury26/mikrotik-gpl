#ifndef _L2TP_IOCTL_H_
#define _L2TP_IOCTL_H_
#include <linux/if.h>
#include <linux/if_ether.h>

#define CRYPTOLEN	64
#define L2TPV3_PROTO    115

enum L2TP_VER {
    L2TPv2      = 0,
    L2TPv3_IP   = 1,
    L2TPv3_UDP  = 2,
};

enum PSEUDOWIRE_TYPE {
    PW_ALL = 0,
    PW_ETH = 5,
    PW_PPP = 7,
};

struct l2tp_info {
    int address_family;
    unsigned src_addr;
    unsigned char src_addr6[16];

    unsigned remote_addr;        // network byte order
    unsigned char remote_addr6[16];       // network byte order
    unsigned short remote_port;  // network byte order

    unsigned remote_tunid;       // network byte order
    unsigned remote_sesid;       // network byte order

    unsigned local_tunid;        // host byte order
    unsigned local_sesid;        // host byte order

    int allow_fast_path;

    int l2tp_ver;
    char remote_cookie[8];
    unsigned rcookie_sz;
    char local_cookie[8];
    unsigned lcookie_sz;

    int wire_type; // PPP, ethernet
    int have_l2specific_layer;
    char ifname[IFNAMSIZ];
    unsigned char hwaddr[ETH_ALEN];
};

struct l2tp_del {
    int index;
};

struct l2tp_add {
    union {
        struct l2tp_info in; // config
	struct {
	    int channel;     // allocated channel index
	    unsigned gateway;
	    unsigned gw_iface;
	    char crypto[CRYPTOLEN];
	    unsigned actual_mtu;
	} out;
    } data;
};

struct l2tp_idle_info {
    int channel;
    unsigned xmit_idle;
    unsigned recv_idle;
};


#define L2TP_IOC_MAGIC '7'

// arg for IOCADD has to be 

#define L2TP_IOCADD _IOW(L2TP_IOC_MAGIC, 1, struct l2tp_add)
#define L2TP_IOCDEL _IOW(L2TP_IOC_MAGIC, 2, struct l2tp_del)
#define L2TP_IOCADD_ETH _IOW(L2TP_IOC_MAGIC, 3, struct l2tp_add)
#define L2TP_IOCIDLE_ETH _IOW(L2TP_IOC_MAGIC, 4, struct l2tp_add)

#endif
