#ifndef _USBNET_IO_H_
#define _USBNET_IO_H_

#define USBNET_BR_IFACE (SIOCDEVPRIVATE + 5)

struct usbnet_params {
    unsigned char add;                  // add/remove
    unsigned char single_apn;           // single APN passthrough
    unsigned char mac_update;           // update dst MAC on tx to modem
    unsigned short vlan_id;             // multiAPN vlan-id
    int slave_id;                       // slave interface id or 0 for IP drop
    unsigned dst_ip;                    // peer IP filter for multiple APNs
    unsigned char dst_mac[ETH_ALEN];    // peer MAC
} __attribute__ ((__packed__));

#endif
