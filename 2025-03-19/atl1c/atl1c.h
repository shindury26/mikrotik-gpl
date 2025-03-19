/*
 * Copyright(c) 2008 - 2009 Atheros Corporation. All rights reserved.
 *
 * Derived from Intel e1000 driver
 * Copyright(c) 1999 - 2005 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef _ATL1C_H_
#define _ATL1C_H_

#include <linux/version.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/udp.h>
#include <linux/mii.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/tcp.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/workqueue.h>
#include <net/checksum.h>
#include <net/ip6_checksum.h>
#include <linux/if_switch.h>
#include <linux/eth_stats.h>
#include <linux/packet_hook.h>
#include <linux/switch.h>

#define FIELD_GETX(_x, _name)   ((_x) >> (_name##_SHIFT) & (_name##_MASK))
#define FIELD_SETX(_x, _name, _v) \
(((_x) & ~((_name##_MASK) << (_name##_SHIFT))) |\
(((_v) & (_name##_MASK)) << (_name##_SHIFT)))
#define FIELDX(_name, _v) (((_v) & (_name##_MASK)) << (_name##_SHIFT))

/* hw-ids */
#define PCI_DEVICE_ID_ATTANSIC_L2C      0x1062
#define PCI_DEVICE_ID_ATTANSIC_L1C      0x1063
#define PCI_DEVICE_ID_ATHEROS_L2C_B	0x2060 /* AR8152 v1.1 Fast 10/100 */
#define PCI_DEVICE_ID_ATHEROS_L2C_B2	0x2062 /* AR8152 v2.0 Fast 10/100 */
#define PCI_DEVICE_ID_ATHEROS_L1D	0x1073 /* AR8151 v1.0 Gigabit 1000 */
#define PCI_DEVICE_ID_ATHEROS_L1D_2_0	0x1083 /* AR8151 v2.0 Gigabit 1000 */
#define L2CB_V10			0xc0
#define L2CB_V11			0xc1
#define L2CB_V20			0xc0
#define L2CB_V21			0xc1

/* Wake Up Filter Control */
#define AT_WUFC_LNKC 0x00000001 /* Link Status Change Wakeup Enable */
#define AT_WUFC_MAG  0x00000002 /* Magic Packet Wakeup Enable */
#define AT_WUFC_EX   0x00000004 /* Directed Exact Wakeup Enable */
#define AT_WUFC_MC   0x00000008 /* Multicast Wakeup Enable */
#define AT_WUFC_BC   0x00000010 /* Broadcast Wakeup Enable */

#define SPEED_0		   0xffff
#define HALF_DUPLEX        1
#define FULL_DUPLEX        2

#define AT_RX_BUF_SIZE		(ETH_FRAME_LEN + VLAN_HLEN + ETH_FCS_LEN)
#define MAX_JUMBO_FRAME_SIZE 	(9*1024)
#define MAX_TSO_FRAME_SIZE      (7*1024)
#define MAX_TX_OFFLOAD_THRESH	(9*1024)

#define AT_MAX_RECEIVE_QUEUE    4
#define AT_DEF_RECEIVE_QUEUE	1
#define AT_MAX_TRANSMIT_QUEUE	4

#define AT_DMA_HI_ADDR_MASK     0xffffffff00000000ULL
#define AT_DMA_LO_ADDR_MASK     0x00000000ffffffffULL

#define AT_TWSI_EEPROM_TIMEOUT 	100
#define AT_HW_MAX_IDLE_DELAY 	10
#define AT_SUSPEND_LINK_TIMEOUT 28

#define AT_ASPM_L0S_TIMER	6
#define AT_ASPM_L1_TIMER	12

#define ATL1C_PCIE_L0S_L1_DISABLE 	0x01
#define ATL1C_PCIE_PHY_RESET		0x02

#define ATL1C_ASPM_L0s_ENABLE		0x0001
#define ATL1C_ASPM_L1_ENABLE		0x0002

#define AT_REGS_LEN	(75 * sizeof(u32))
#define AT_EEPROM_LEN 	512

/* tpd word 1 bit 0:7 General Checksum task offload */
#define TPD_L4HDR_OFFSET_MASK	0x00FF
#define TPD_L4HDR_OFFSET_SHIFT	0

/* tpd word 1 bit 0:7 Large Send task offload (IPv4/IPV6) */
#define TPD_TCPHDR_OFFSET_MASK	0x00FF
#define TPD_TCPHDR_OFFSET_SHIFT	0

/* tpd word 1 bit 0:7 Custom Checksum task offload */
#define TPD_PLOADOFFSET_MASK	0x00FF
#define TPD_PLOADOFFSET_SHIFT	0

/* tpd word 1 bit 8:17 */
#define TPD_CCSUM_EN_MASK	0x0001
#define TPD_CCSUM_EN_SHIFT	8
#define TPD_IP_CSUM_MASK	0x0001
#define TPD_IP_CSUM_SHIFT	9
#define TPD_TCP_CSUM_MASK	0x0001
#define TPD_TCP_CSUM_SHIFT	10
#define TPD_UDP_CSUM_MASK	0x0001
#define TPD_UDP_CSUM_SHIFT	11
#define TPD_LSO_EN_MASK		0x0001	/* TCP Large Send Offload */
#define TPD_LSO_EN_SHIFT	12
#define TPD_LSO_VER_MASK	0x0001
#define TPD_LSO_VER_SHIFT	13 	/* 0 : ipv4; 1 : ipv4/ipv6 */
#define TPD_CON_VTAG_MASK	0x0001
#define TPD_CON_VTAG_SHIFT	14
#define TPD_INS_VTAG_MASK	0x0001
#define TPD_INS_VTAG_SHIFT	15
#define TPD_IPV4_PACKET_MASK	0x0001  /* valid when LSO VER  is 1 */
#define TPD_IPV4_PACKET_SHIFT	16
#define TPD_ETH_TYPE_MASK	0x0001
#define TPD_ETH_TYPE_SHIFT	17	/* 0 : 802.3 frame; 1 : Ethernet */

/* tpd word 18:25 Custom Checksum task offload */
#define TPD_CCSUM_OFFSET_MASK	0x00FF
#define TPD_CCSUM_OFFSET_SHIFT	18
#define TPD_CCSUM_EPAD_MASK	0x0001
#define TPD_CCSUM_EPAD_SHIFT	30

/* tpd word 18:30 Large Send task offload (IPv4/IPV6) */
#define TPD_MSS_MASK            0x1FFF
#define TPD_MSS_SHIFT		18

#define TPD_EOP_MASK		0x0001
#define TPD_EOP_SHIFT		31

struct atl1c_tpd_desc {
	union {
		struct {
			__le16	buffer_len;
			__le16	vlan_tag;
			__le32	word1;
		};
		__le64 qword0;
	};
	__le64	buffer_addr;
};

struct atl1c_tpd_ext_desc {
	u32 reservd_0;
	__le32 word1;
	__le32 pkt_len;
	u32 reservd_1;
};
/* rrs word 0 bit 0:31 */
#define RRS_RX_CSUM_MASK	0xFFFF
#define RRS_RX_CSUM_SHIFT	0
#define RRS_RX_RFD_CNT_MASK	0x000F
#define RRS_RX_RFD_CNT_SHIFT	16
#define RRS_RX_RFD_INDEX_MASK	0x0FFF
#define RRS_RX_RFD_INDEX_SHIFT	20

/* rrs flag bit 0:16 */
#define RRS_HEAD_LEN_MASK	0x00FF
#define RRS_HEAD_LEN_SHIFT	0
#define RRS_HDS_TYPE_MASK	0x0003
#define RRS_HDS_TYPE_SHIFT	8
#define RRS_CPU_NUM_MASK	0x0003
#define	RRS_CPU_NUM_SHIFT	10
#define RRS_HASH_FLG_MASK	0x000F
#define RRS_HASH_FLG_SHIFT	12

/* rrs word 3 bit 0:31 */
#define RRS_PKT_SIZE_MASK	0x3FFF
#define RRS_PKT_SIZE_SHIFT	0
#define RRS_ERR_L4_CSUM_MASK	0x0001
#define RRS_ERR_L4_CSUM_SHIFT	14
#define RRS_ERR_IP_CSUM_MASK	0x0001
#define RRS_ERR_IP_CSUM_SHIFT	15
#define RRS_VLAN_INS_MASK	0x0001
#define RRS_VLAN_INS_SHIFT	16
#define RRS_PROT_ID_MASK	0x0007
#define RRS_PROT_ID_SHIFT	17
#define RRS_RX_ERR_SUM_MASK	0x0001
#define RRS_RX_ERR_SUM_SHIFT	20
#define RRS_RX_ERR_CRC_MASK	0x0001
#define RRS_RX_ERR_CRC_SHIFT	21
#define RRS_RX_ERR_FAE_MASK	0x0001
#define RRS_RX_ERR_FAE_SHIFT	22
#define RRS_RX_ERR_TRUNC_MASK	0x0001
#define RRS_RX_ERR_TRUNC_SHIFT	23
#define RRS_RX_ERR_RUNC_MASK	0x0001
#define RRS_RX_ERR_RUNC_SHIFT	24
#define RRS_RX_ERR_ICMP_MASK	0x0001
#define RRS_RX_ERR_ICMP_SHIFT	25
#define RRS_PACKET_BCAST_MASK	0x0001
#define RRS_PACKET_BCAST_SHIFT	26
#define RRS_PACKET_MCAST_MASK	0x0001
#define RRS_PACKET_MCAST_SHIFT	27
#define RRS_PACKET_TYPE_MASK	0x0001
#define RRS_PACKET_TYPE_SHIFT	28
#define RRS_FIFO_FULL_MASK	0x0001
#define RRS_FIFO_FULL_SHIFT	29
#define RRS_802_3_LEN_ERR_MASK 	0x0001
#define RRS_802_3_LEN_ERR_SHIFT 30

#define RRS_ERR_L4_CSUM         0x00004000
#define RRS_ERR_IP_CSUM         0x00008000
#define RRS_VLAN_INS            0x00010000
#define RRS_RX_ERR_SUM          0x00100000
#define RRS_RX_ERR_CRC          0x00200000
#define RRS_802_3_LEN_ERR	0x40000000
#define RRS_RXD_UPDATED		0x80000000

#define RRS_MT_PROT_ID_TCPUDP	BIT(19)

struct atl1c_recv_ret_status {
	__le32  word0;
	__le32	rss_hash;
	__le16	vlan_tag;
	__le16	flag;
	__le32	word3;
};

/* RFD desciptor */
struct atl1c_rx_free_desc {
	__le64	buffer_addr;
};

/* DMA Order Settings */
enum atl1c_dma_order {
	atl1c_dma_ord_in = 1,
	atl1c_dma_ord_enh = 2,
	atl1c_dma_ord_out = 4
};

enum atl1c_mac_speed {
	atl1c_mac_speed_0 = 0,
	atl1c_mac_speed_10_100 = 1,
	atl1c_mac_speed_1000 = 2
};

enum atl1c_dma_req_block {
	atl1c_dma_req_128 = 0,
	atl1c_dma_req_256 = 1,
	atl1c_dma_req_512 = 2,
	atl1c_dma_req_1024 = 3,
	atl1c_dma_req_2048 = 4,
	atl1c_dma_req_4096 = 5
};

enum atl1c_nic_type {
	athr_l1c = 0,
	athr_l2c = 1,
	athr_l2c_b,
	athr_l2c_b2,
	athr_l1d,
	athr_l1d_2,
	athr_mt,
};

struct atl1c_hw_stats {
	/* rx */
	unsigned rx_ok;		/* The number of good packet received. */
	unsigned rx_bcast;		/* The number of good broadcast packet received. */
	unsigned rx_mcast;		/* The number of good multicast packet received. */
	unsigned rx_pause;		/* The number of Pause packet received. */
	unsigned rx_ctrl;		/* The number of Control packet received other than Pause frame. */
	unsigned rx_fcs_err;	/* The number of packets with bad FCS. */
	unsigned rx_len_err;	/* The number of packets with mismatch of length field and actual size. */
	unsigned rx_byte_cnt;	/* The number of bytes of good packet received. FCS is NOT included. */
	unsigned rx_runt;		/* The number of packets received that are less than 64 byte long and with good FCS. */
	unsigned rx_frag;		/* The number of packets received that are less than 64 byte long and with bad FCS. */
	unsigned rx_sz_64;		/* The number of good and bad packets received that are 64 byte long. */
	unsigned rx_sz_65_127;	/* The number of good and bad packets received that are between 65 and 127-byte long. */
	unsigned rx_sz_128_255;	/* The number of good and bad packets received that are between 128 and 255-byte long. */
	unsigned rx_sz_256_511;	/* The number of good and bad packets received that are between 256 and 511-byte long. */
	unsigned rx_sz_512_1023;	/* The number of good and bad packets received that are between 512 and 1023-byte long. */
	unsigned rx_sz_1024_1518;	/* The number of good and bad packets received that are between 1024 and 1518-byte long. */
	unsigned rx_sz_1519_max;	/* The number of good and bad packets received that are between 1519-byte and MTU. */
	unsigned rx_sz_ov;		/* The number of good and bad packets received that are more than MTU size truncated by Selene. */
	unsigned rx_rxf_ov;	/* The number of frame dropped due to occurrence of RX FIFO overflow. */
	unsigned rx_rrd_ov;	/* The number of frame dropped due to occurrence of RRD overflow. */
	unsigned rx_align_err;	/* Alignment Error */
	unsigned rx_bcast_byte_cnt; /* The byte count of broadcast packet received, excluding FCS. */
	unsigned rx_mcast_byte_cnt; /* The byte count of multicast packet received, excluding FCS. */
	unsigned rx_err_addr;	/* The number of packets dropped due to address filtering. */

	/* tx */
	unsigned tx_ok;		/* The number of good packet transmitted. */
	unsigned tx_bcast;		/* The number of good broadcast packet transmitted. */
	unsigned tx_mcast;		/* The number of good multicast packet transmitted. */
	unsigned tx_pause;		/* The number of Pause packet transmitted. */
	unsigned tx_exc_defer;	/* The number of packets transmitted with excessive deferral. */
	unsigned tx_ctrl;		/* The number of packets transmitted is a control frame, excluding Pause frame. */
	unsigned tx_defer;		/* The number of packets transmitted that is deferred. */
	unsigned tx_byte_cnt;	/* The number of bytes of data transmitted. FCS is NOT included. */
	unsigned tx_sz_64;		/* The number of good and bad packets transmitted that are 64 byte long. */
	unsigned tx_sz_65_127;	/* The number of good and bad packets transmitted that are between 65 and 127-byte long. */
	unsigned tx_sz_128_255;	/* The number of good and bad packets transmitted that are between 128 and 255-byte long. */
	unsigned tx_sz_256_511;	/* The number of good and bad packets transmitted that are between 256 and 511-byte long. */
	unsigned tx_sz_512_1023;	/* The number of good and bad packets transmitted that are between 512 and 1023-byte long. */
	unsigned tx_sz_1024_1518;	/* The number of good and bad packets transmitted that are between 1024 and 1518-byte long. */
	unsigned tx_sz_1519_max;	/* The number of good and bad packets transmitted that are between 1519-byte and MTU. */
	unsigned tx_1_col;		/* The number of packets subsequently transmitted successfully with a single prior collision. */
	unsigned tx_2_col;		/* The number of packets subsequently transmitted successfully with multiple prior collisions. */
	unsigned tx_late_col;	/* The number of packets transmitted with late collisions. */
	unsigned tx_abort_col;	/* The number of transmit packets aborted due to excessive collisions. */
	unsigned tx_underrun;	/* The number of transmit packets aborted due to transmit FIFO underrun, or TRD FIFO underrun */
	unsigned tx_rd_eop;	/* The number of times that read beyond the EOP into the next frame area when TRD was not written timely */
	unsigned tx_len_err;	/* The number of transmit packets with length field does NOT match the actual frame size. */
	unsigned tx_trunc;		/* The number of transmit packets truncated due to size exceeding MTU. */
	unsigned tx_bcast_byte;	/* The byte count of broadcast packet transmitted, excluding FCS. */
	unsigned tx_mcast_byte;	/* The byte count of multicast packet transmitted, excluding FCS. */
};

static const struct eth_stat_item eth_stat_items_atl1c[] = {
    { 0, ETH_STATS_RX_PACKET, NULL },
    { 0, ETH_STATS_RX_BROADCAST, NULL },
    { 0, ETH_STATS_RX_MULTICAST, NULL },
    { 1, ETH_STATS_RX_PAUSE, NULL },
    { 1, ETH_STATS_RX_CONTROL, NULL },
    { 1, ETH_STATS_RX_FCS_ERR, NULL },
    { 1, ETH_STATS_RX_LENGTH_ERR, NULL },
    { 0, ETH_STATS_RX_BYTE, NULL },
    { 1, ETH_STATS_RX_TOO_SHORT, NULL },
    { 1, ETH_STATS_RX_FRAGMENT, NULL },
    { 0, ETH_STATS_RX_64, NULL },
    { 0, ETH_STATS_RX_65_127, NULL },
    { 0, ETH_STATS_RX_128_255, NULL },
    { 0, ETH_STATS_RX_256_511, NULL },
    { 0, ETH_STATS_RX_512_1023, NULL },
    { 0, ETH_STATS_RX_1024_1518, NULL },
    { 0, ETH_STATS_RX_1519_MAX, NULL },
    { 1, ETH_STATS_RX_TOO_LONG, NULL },
    { 1, ETH_STATS_RX_OVERRUN, NULL },
    { 1, ETH_STATS_RX_DROP, NULL },
    { 1, ETH_STATS_RX_ALIGN_ERR, NULL },
    { 0, -1, "rx broadcast byte" },
    { 0, -1, "rx multicast byte" },
    { 0, -1, "rx address filtered" },

    { 0, ETH_STATS_TX_PACKET, NULL },
    { 0, ETH_STATS_TX_BROADCAST, NULL },
    { 0, ETH_STATS_TX_MULTICAST, NULL },
    { 0, ETH_STATS_TX_PAUSE, NULL },
    { 1, ETH_STATS_TX_EXCESSIVE_DEFERRED, NULL },
    { 1, ETH_STATS_TX_CONTROL, NULL },
    { 1, ETH_STATS_TX_DEFERRED, NULL },
    { 0, ETH_STATS_TX_BYTE, NULL },
    { 0, ETH_STATS_TX_64, NULL },
    { 0, ETH_STATS_TX_65_127, NULL },
    { 0, ETH_STATS_TX_128_255, NULL },
    { 0, ETH_STATS_TX_256_511, NULL },
    { 0, ETH_STATS_TX_512_1023, NULL },
    { 0, ETH_STATS_TX_1024_1518, NULL },
    { 0, ETH_STATS_TX_1519_MAX, NULL },
    { 1, ETH_STATS_TX_SINGLE_COL, NULL },
    { 1, ETH_STATS_TX_MULTI_COL, NULL },
    { 1, ETH_STATS_TX_LATE_COL, NULL },
    { 1, ETH_STATS_TX_EXCESSIVE_COL, NULL },
    { 1, ETH_STATS_TX_UNDERRUN, NULL },
    { 0, -1, "tx rd eop" },
    { 0, -1, "tx len err" },
    { 1, ETH_STATS_TX_TOO_LONG, NULL },
    { 0, -1, "tx broadcast byte" },
    { 0, -1, "tx multicast byte" },

    { -1, -1, NULL }
};

struct atl1c_hw {
	u8 __iomem      *hw_addr;            /* inner register address */
	struct atl1c_adapter *adapter;
	enum atl1c_nic_type  nic_type;
	enum atl1c_dma_order dma_order;
	enum atl1c_dma_req_block dmar_block;
	enum atl1c_dma_req_block dmaw_block;

	u16 device_id;
	u16 vendor_id;
	u16 subsystem_id;
	u16 subsystem_vendor_id;
	u8 revision_id;

	spinlock_t intr_mask_lock;
	u32 intr_mask;
	u8 dmaw_dly_cnt;
	u8 dmar_dly_cnt;

	u8 preamble_len;
	u16 max_frame_size;
	u16 min_frame_size;

	enum atl1c_mac_speed mac_speed;
	bool mac_duplex;
	bool hibernate;
	u16 media_type;
#define MEDIA_TYPE_AUTO_SENSOR  0
#define MEDIA_TYPE_100M_FULL    1
#define MEDIA_TYPE_100M_HALF    2
#define MEDIA_TYPE_10M_FULL     3
#define MEDIA_TYPE_10M_HALF     4

	u16 autoneg_advertised;
	u16 mii_autoneg_adv_reg;
	u16 mii_1000t_ctrl_reg;

	u16 tx_imt;	/* TX Interrupt Moderator timer ( 2us resolution) */
	u16 rx_imt;	/* RX Interrupt Moderator timer ( 2us resolution) */
	u16 ict;        /* Interrupt Clear timer (2us resolution) */
	u16 ctrl_flags;
#define ATL1C_INTR_CLEAR_ON_READ	0x0001
#define ATL1C_INTR_MODRT_ENABLE	 	0x0002
#define ATL1C_CMB_ENABLE		0x0004
#define ATL1C_SMB_ENABLE		0x0010
#define ATL1C_TXQ_MODE_ENHANCE		0x0020
#define ATL1C_RX_IPV6_CHKSUM		0x0040
#define ATL1C_ASPM_L0S_SUPPORT		0x0080
#define ATL1C_ASPM_L1_SUPPORT		0x0100
#define ATL1C_ASPM_CTRL_MON		0x0200
#define ATL1C_HIB_DISABLE		0x0400
#define ATL1C_APS_MODE_ENABLE           0x0800
#define ATL1C_LINK_EXT_SYNC             0x1000
#define ATL1C_CLK_GATING_EN             0x2000
	u16 link_cap_flags;
#define ATL1C_LINK_CAP_1000M		0x0001
	u16 cmb_tpd;
	u16 cmb_rrd;
	u16 cmb_rx_timer; /* 2us resolution */
	u16 cmb_tx_timer;
	u32 smb_timer;

	u16 rrd_thresh; /* Threshold of number of RRD produced to trigger
			  interrupt request */
	u16 tpd_thresh;
	u8 tpd_burst;   /* Number of TPD to prefetch in cache-aligned burst. */
	u8 rfd_burst;
	u32 base_cpu;
	u32 indirect_tab;
	u8 mac_addr[ETH_ALEN];
	u8 perm_mac_addr[ETH_ALEN];

	bool phy_configured;
	bool re_autoneg;
	bool emi_ca;
	bool msi_lnkpatch;	/* link patch for specific platforms */
};

/*
 * atl1c_ring_header represents a single, contiguous block of DMA space
 * mapped for the three descriptor rings (tpd, rfd, rrd) and the two
 * message blocks (cmb, smb) described below
 */
struct atl1c_ring_header {
	void *desc;
	dma_addr_t dma;
	unsigned int size;
};

struct atl1c_buffer {
	void *buf;
	dma_addr_t dma;
	u16 length;
	bool fpb;
};

struct atl1c_pcpu {
	unsigned xmit_count;
	struct fp_buf fpbs[FP_NAPI_BUDGET];
	u64 dma_addrs[FP_NAPI_BUDGET];
} ____cacheline_aligned_in_smp;

// transimit packet descriptor (tpd) ring
struct atl1c_tpd_ring {
	struct atl1c_adapter *adapter;
	u16 num;
	struct atl1c_tpd_desc *desc;
	dma_addr_t dma;
	u32 size;
	u16 count;
	u16 mask;
	u16 head;
	u16 tail;
	u16 stop_threshold;
	struct atl1c_buffer *buffer_info;
	struct napi_struct  napi;
	u32 target_cpu;
	call_single_data_t csd;
	spinlock_t tx_lock;
	struct atl1c_pcpu pcpu[NR_CPUS];
//#define STATS
#ifdef STATS
	unsigned stats_intr;
	unsigned stats_poll;
	unsigned stats_poll_done;
	unsigned stats_xmit;
	unsigned stats_xmit_commit;
	unsigned stats_tx;
	unsigned stats_tx_queue_stop;
#endif
};

// receive free descriptor (rfd) ring
struct atl1c_rfd_ring {
	struct atl1c_rx_free_desc *desc;
	dma_addr_t dma;
	u32 size;
	u16 count;
	u16 mask;
	u16 head;
	struct atl1c_buffer *buffer_info;
};

// receive return desciptor (rrd) ring
struct atl1c_rrd_ring {
	struct atl1c_adapter *adapter;
	u16 num;
	struct atl1c_recv_ret_status *desc;
	dma_addr_t dma;
	u32 size;
	u16 count;
	u16 mask;
	u16 tail;
	struct napi_struct  napi;
	u32 target_cpu;
	call_single_data_t csd;
#ifdef STATS
	unsigned stats_intr;
	unsigned stats_poll;
	unsigned stats_poll_done;
	unsigned stats_rx;
#endif
};

struct atl1c_cmb {
	void *cmb;
	dma_addr_t dma;
};

struct atl1c_smb {
	void *smb;
	dma_addr_t dma;
};

struct atl1c_adapter {
	struct net_device   *netdev;
	struct pci_dev      *pdev;
	struct atl1c_hw        hw;
	struct atl1c_hw_stats  hw_stats;
	struct net_device_stats net_stats;
	struct mii_if_info  mii;
	u16 rx_buffer_len;
	u32 tx_queue_count;
	u32 rx_queue_count;

	u8 work_event;
#define ATL1C_WORK_EVENT_RESET 		0x01
#define ATL1C_WORK_EVENT_LINK_CHANGE	0x02
	u32 msg_enable;

	bool have_msi;
	u16 link_speed;
	u16 link_duplex;

	spinlock_t mdio_lock;
	unsigned xmit_commit_count;

	struct work_struct common_task;

	struct atl1c_ring_header ring_header;
	struct atl1c_tpd_ring tpd_ring[AT_MAX_TRANSMIT_QUEUE];
	struct atl1c_rfd_ring rfd_ring[AT_MAX_RECEIVE_QUEUE];
	struct atl1c_rrd_ring rrd_ring[AT_MAX_RECEIVE_QUEUE];
	struct atl1c_cmb cmb;
	struct atl1c_smb smb;
	u32 base_cpu;
	struct eth_stats xstats;
#ifdef STATS
	unsigned long stats_last_jiffies;
	unsigned long long stats_last_clock;
	unsigned stats_intr;
#endif
};

#define AT_WRITE_REG(a, reg, value) ( \
		writel((value), ((a)->hw_addr + reg)))

#define AT_WRITE_FLUSH(a) (\
		readl((a)->hw_addr))

#define AT_READ_REG(a, reg, pdata) do {					\
		if (unlikely((a)->hibernate)) {				\
			readl((a)->hw_addr + reg);			\
			*(u32 *)pdata = readl((a)->hw_addr + reg);	\
		} else {						\
			*(u32 *)pdata = readl((a)->hw_addr + reg);	\
		}							\
	} while (0)

#define AT_WRITE_REGW(a, reg, value) (\
		writew((value), ((a)->hw_addr + reg)))

#define AT_READ_REGW(a, reg) (\
		readw((a)->hw_addr + reg))


/* register definition */
#define REG_DEVICE_CAP              	0x5C
#define DEVICE_CAP_MAX_PAYLOAD_MASK     0x7
#define DEVICE_CAP_MAX_PAYLOAD_SHIFT    0

#define REG_DEVICE_CTRL			0x60
#define DEVICE_CTRL_MAX_PAYLOAD_MASK    0x7
#define DEVICE_CTRL_MAX_PAYLOAD_SHIFT   5
#define DEVICE_CTRL_MAX_RREQ_SZ_MASK    0x7
#define DEVICE_CTRL_MAX_RREQ_SZ_SHIFT   12

#define REG_LINK_CTRL			0x68
#define LINK_CTRL_L0S_EN		0x01
#define LINK_CTRL_L1_EN			0x02
#define LINK_CTRL_EXT_SYNC		0x80

#define REG_VPD_CAP			0x6C
#define VPD_CAP_ID_MASK                 0xff
#define VPD_CAP_ID_SHIFT                0
#define VPD_CAP_NEXT_PTR_MASK           0xFF
#define VPD_CAP_NEXT_PTR_SHIFT          8
#define VPD_CAP_VPD_ADDR_MASK           0x7FFF
#define VPD_CAP_VPD_ADDR_SHIFT          16
#define VPD_CAP_VPD_FLAG                0x80000000

#define REG_LINK_CTRL			0x68
#define LINK_CTRL_L0S_EN		0x01
#define LINK_CTRL_L1_EN			0x02
#define LINK_CTRL_EXT_SYNC		0x80

#define REG_PCIE_IND_ACC_ADDR		0x80
#define REG_PCIE_IND_ACC_DATA		0x84

#define REG_PCIE_UC_SEVERITY		0x10C
#define PCIE_UC_SERVRITY_TRN		0x00000001
#define PCIE_UC_SERVRITY_DLP		0x00000010
#define PCIE_UC_SERVRITY_PSN_TLP	0x00001000
#define PCIE_UC_SERVRITY_FCP		0x00002000
#define PCIE_UC_SERVRITY_CPL_TO		0x00004000
#define PCIE_UC_SERVRITY_CA		0x00008000
#define PCIE_UC_SERVRITY_UC		0x00010000
#define PCIE_UC_SERVRITY_ROV		0x00020000
#define PCIE_UC_SERVRITY_MLFP		0x00040000
#define PCIE_UC_SERVRITY_ECRC		0x00080000
#define PCIE_UC_SERVRITY_UR		0x00100000

#define REG_DEV_SERIALNUM_CTRL		0x200
#define REG_DEV_MAC_SEL_MASK		0x0 /* 0:EUI; 1:MAC */
#define REG_DEV_MAC_SEL_SHIFT		0
#define REG_DEV_SERIAL_NUM_EN_MASK	0x1
#define REG_DEV_SERIAL_NUM_EN_SHIFT	1

#define REG_TWSI_CTRL               	0x218
#define TWSI_CTRL_LD_OFFSET_MASK        0xFF
#define TWSI_CTRL_LD_OFFSET_SHIFT       0
#define TWSI_CTRL_LD_SLV_ADDR_MASK      0x7
#define TWSI_CTRL_LD_SLV_ADDR_SHIFT     8
#define TWSI_CTRL_SW_LDSTART            0x800
#define TWSI_CTRL_HW_LDSTART            0x1000
#define TWSI_CTRL_SMB_SLV_ADDR_MASK     0x7F
#define TWSI_CTRL_SMB_SLV_ADDR_SHIFT    15
#define TWSI_CTRL_LD_EXIST              0x400000
#define TWSI_CTRL_READ_FREQ_SEL_MASK    0x3
#define TWSI_CTRL_READ_FREQ_SEL_SHIFT   23
#define TWSI_CTRL_FREQ_SEL_100K         0
#define TWSI_CTRL_FREQ_SEL_200K         1
#define TWSI_CTRL_FREQ_SEL_300K         2
#define TWSI_CTRL_FREQ_SEL_400K         3
#define TWSI_CTRL_SMB_SLV_ADDR
#define TWSI_CTRL_WRITE_FREQ_SEL_MASK   0x3
#define TWSI_CTRL_WRITE_FREQ_SEL_SHIFT  24


#define REG_PCIE_DEV_MISC_CTRL      	0x21C
#define PCIE_DEV_MISC_EXT_PIPE     	0x2
#define PCIE_DEV_MISC_RETRY_BUFDIS 	0x1
#define PCIE_DEV_MISC_SPIROM_EXIST 	0x4
#define PCIE_DEV_MISC_SERDES_ENDIAN    	0x8
#define PCIE_DEV_MISC_SERDES_SEL_DIN   	0x10

#define REG_PCIE_PHYMISC	    	0x1000
#define PCIE_PHYMISC_FORCE_RCV_DET	BIT(2)
#define PCIE_PHYMISC_NFTS_MASK		0xFFUL
#define PCIE_PHYMISC_NFTS_SHIFT		16

#define REG_PCIE_PHYMISC2		0x1004
#define PCIE_PHYMISC2_L0S_TH_MASK	0x3UL
#define PCIE_PHYMISC2_L0S_TH_SHIFT	18
#define L2CB1_PCIE_PHYMISC2_L0S_TH	3
#define PCIE_PHYMISC2_CDR_BW_MASK	0x3UL
#define PCIE_PHYMISC2_CDR_BW_SHIFT	16
#define L2CB1_PCIE_PHYMISC2_CDR_BW	3

#define REG_TWSI_DEBUG			0x1108
#define TWSI_DEBUG_DEV_EXIST		BIT(29)

#define REG_DMA_DBG			0x1114
#define DMA_DBG_VENDOR_MSG		BIT(0)

#define REG_EEPROM_CTRL			0x12C0
#define EEPROM_CTRL_DATA_HI_MASK	0xFFFF
#define EEPROM_CTRL_DATA_HI_SHIFT	0
#define EEPROM_CTRL_ADDR_MASK		0x3FF
#define EEPROM_CTRL_ADDR_SHIFT		16
#define EEPROM_CTRL_ACK			0x40000000
#define EEPROM_CTRL_RW			0x80000000

#define REG_EEPROM_DATA_LO		0x12C4

#define REG_OTP_CTRL			0x12F0
#define OTP_CTRL_CLK_EN			0x0002

#define REG_PM_CTRL			0x12F8
#define PM_CTRL_HOTRST			BIT(31)
#define PM_CTRL_MAC_ASPM_CHK		BIT(30)	/* L0s/L1 dis by MAC based on
						 * thrghput(setting in 15A0) */
#define PM_CTRL_SA_DLY_EN		BIT(29)
#define PM_CTRL_L0S_BUFSRX_EN		BIT(28)
#define PM_CTRL_LCKDET_TIMER_MASK	0xFUL
#define PM_CTRL_LCKDET_TIMER_SHIFT	24
#define PM_CTRL_LCKDET_TIMER_DEF	0xC
#define PM_CTRL_PM_REQ_TIMER_MASK	0xFUL
#define PM_CTRL_PM_REQ_TIMER_SHIFT	20	/* pm_request_l1 time > @
						 * ->L0s not L1 */
#define PM_CTRL_PM_REQ_TO_DEF		0xF
#define PMCTRL_TXL1_AFTER_L0S		BIT(19)	/* l1dv2.0+ */
#define L1D_PMCTRL_L1_ENTRY_TM_MASK	7UL	/* l1dv2.0+, 3bits */
#define L1D_PMCTRL_L1_ENTRY_TM_SHIFT	16
#define L1D_PMCTRL_L1_ENTRY_TM_DIS	0
#define L1D_PMCTRL_L1_ENTRY_TM_2US	1
#define L1D_PMCTRL_L1_ENTRY_TM_4US	2
#define L1D_PMCTRL_L1_ENTRY_TM_8US	3
#define L1D_PMCTRL_L1_ENTRY_TM_16US	4
#define L1D_PMCTRL_L1_ENTRY_TM_24US	5
#define L1D_PMCTRL_L1_ENTRY_TM_32US	6
#define L1D_PMCTRL_L1_ENTRY_TM_63US	7
#define PM_CTRL_L1_ENTRY_TIMER_MASK	0xFUL  /* l1C 4bits */
#define PM_CTRL_L1_ENTRY_TIMER_SHIFT	16
#define L2CB1_PM_CTRL_L1_ENTRY_TM	7
#define L1C_PM_CTRL_L1_ENTRY_TM		0xF
#define PM_CTRL_RCVR_WT_TIMER		BIT(15)	/* 1:1us, 0:2ms */
#define PM_CTRL_CLK_PWM_VER1_1		BIT(14)	/* 0:1.0a,1:1.1 */
#define PM_CTRL_CLK_SWH_L1		BIT(13)	/* en pcie clk sw in L1 */
#define PM_CTRL_ASPM_L0S_EN		BIT(12)
#define PM_CTRL_RXL1_AFTER_L0S		BIT(11)	/* l1dv2.0+ */
#define L1D_PMCTRL_L0S_TIMER_MASK	7UL	/* l1d2.0+, 3bits*/
#define L1D_PMCTRL_L0S_TIMER_SHIFT	8
#define PM_CTRL_L0S_ENTRY_TIMER_MASK	0xFUL	/* l1c, 4bits */
#define PM_CTRL_L0S_ENTRY_TIMER_SHIFT	8
#define PM_CTRL_SERDES_BUFS_RX_L1_EN	BIT(7)
#define PM_CTRL_SERDES_PD_EX_L1		BIT(6)	/* power down serdes rx */
#define PM_CTRL_SERDES_PLL_L1_EN	BIT(5)
#define PM_CTRL_SERDES_L1_EN		BIT(4)
#define PM_CTRL_ASPM_L1_EN		BIT(3)
#define PM_CTRL_CLK_REQ_EN		BIT(2)
#define PM_CTRL_RBER_EN			BIT(1)
#define PM_CTRL_SPRSDWER_EN		BIT(0)

#define REG_LTSSM_ID_CTRL		0x12FC
#define LTSSM_ID_EN_WRO			0x1000


/* Selene Master Control Register */
#define REG_MASTER_CTRL			0x1400
#define MASTER_CTRL_OTP_SEL		BIT(31)
#define MASTER_DEV_NUM_MASK		0x7FUL
#define MASTER_DEV_NUM_SHIFT		24
#define MASTER_REV_NUM_MASK		0xFFUL
#define MASTER_REV_NUM_SHIFT		16
#define MASTER_CTRL_INT_RDCLR		BIT(14)
#define MASTER_CTRL_CLK_SEL_DIS		BIT(12)	/* 1:alwys sel pclk from
						 * serdes, not sw to 25M */
#define MASTER_CTRL_RX_ITIMER_EN	BIT(11)	/* IRQ MODURATION FOR RX */
#define MASTER_CTRL_TX_ITIMER_EN	BIT(10)	/* MODURATION FOR TX/RX */
#define MASTER_CTRL_MANU_INT		BIT(9)	/* SOFT MANUAL INT */
#define MASTER_CTRL_MANUTIMER_EN	BIT(8)
#define MASTER_CTRL_SA_TIMER_EN		BIT(7)	/* SYS ALIVE TIMER EN */
#define MASTER_CTRL_OOB_DIS		BIT(6)	/* OUT OF BOX DIS */
#define MASTER_CTRL_WAKEN_25M		BIT(5)	/* WAKE WO. PCIE CLK */
#define MASTER_CTRL_BERT_START		BIT(4)
#define MASTER_PCIE_TSTMOD_MASK		3UL
#define MASTER_PCIE_TSTMOD_SHIFT	2
#define MASTER_PCIE_RST			BIT(1)
#define MASTER_CTRL_SOFT_RST		BIT(0)	/* RST MAC & DMA */
#define DMA_MAC_RST_TO			50

/* Timer Initial Value Register */
#define REG_MANUAL_TIMER_INIT       	0x1404

/* IRQ ModeratorTimer Initial Value Register */
#define REG_IRQ_MODRT_TIMER_INIT     	0x1408
#define IRQ_MODRT_TIMER_MASK		0xffff
#define IRQ_MODRT_TX_TIMER_SHIFT    	0
#define IRQ_MODRT_RX_TIMER_SHIFT	16

#define REG_GPHY_CTRL               	0x140C
#define GPHY_CTRL_EXT_RESET         	0x1
#define GPHY_CTRL_RTL_MODE		0x2
#define GPHY_CTRL_LED_MODE		0x4
#define GPHY_CTRL_ANEG_NOW		0x8
#define GPHY_CTRL_REV_ANEG		0x10
#define GPHY_CTRL_GATE_25M_EN       	0x20
#define GPHY_CTRL_LPW_EXIT          	0x40
#define GPHY_CTRL_PHY_IDDQ          	0x80
#define GPHY_CTRL_PHY_IDDQ_DIS      	0x100
#define GPHY_CTRL_GIGA_DIS		0x200
#define GPHY_CTRL_HIB_EN            	0x400
#define GPHY_CTRL_HIB_PULSE         	0x800
#define GPHY_CTRL_SEL_ANA_RST       	0x1000
#define GPHY_CTRL_PHY_PLL_ON        	0x2000
#define GPHY_CTRL_PWDOWN_HW		0x4000
#define GPHY_CTRL_PHY_PLL_BYPASS	0x8000

#define GPHY_CTRL_DEFAULT (		 \
		GPHY_CTRL_SEL_ANA_RST	|\
		GPHY_CTRL_HIB_PULSE	|\
		GPHY_CTRL_HIB_EN)

#define GPHY_CTRL_PW_WOL_DIS (		 \
		GPHY_CTRL_SEL_ANA_RST	|\
		GPHY_CTRL_HIB_PULSE	|\
		GPHY_CTRL_HIB_EN	|\
		GPHY_CTRL_PWDOWN_HW	|\
		GPHY_CTRL_PHY_IDDQ)

/* Block IDLE Status Register */
#define REG_IDLE_STATUS  		0x1410
#define IDLE_STATUS_MASK		0x00FF
#define IDLE_STATUS_RXMAC_NO_IDLE      	0x1
#define IDLE_STATUS_TXMAC_NO_IDLE      	0x2
#define IDLE_STATUS_RXQ_NO_IDLE        	0x4
#define IDLE_STATUS_TXQ_NO_IDLE        	0x8
#define IDLE_STATUS_DMAR_NO_IDLE       	0x10
#define IDLE_STATUS_DMAW_NO_IDLE       	0x20
#define IDLE_STATUS_SMB_NO_IDLE        	0x40
#define IDLE_STATUS_CMB_NO_IDLE        	0x80

/* MDIO Control Register */
#define REG_MDIO_CTRL           	0x1414
#define MDIO_DATA_MASK          	0xffff  /* On MDIO write, the 16-bit
						 * control data to write to PHY
						 * MII management register */
#define MDIO_DATA_SHIFT         	0       /* On MDIO read, the 16-bit
						 * status data that was read
						 * from the PHY MII management register */
#define MDIO_REG_ADDR_MASK      	0x1f    /* MDIO register address */
#define MDIO_REG_ADDR_SHIFT     	16
#define MDIO_RW                 	0x200000  /* 1: read, 0: write */
#define MDIO_SUP_PREAMBLE       	0x400000  /* Suppress preamble */
#define MDIO_START              	0x800000  /* Write 1 to initiate the MDIO
						   * master. And this bit is self
						   * cleared after one cycle */
#define MDIO_CLK_SEL_SHIFT      	24
#define MDIO_CLK_25_4           	0
#define MDIO_CLK_25_6           	2
#define MDIO_CLK_25_8           	3
#define MDIO_CLK_25_10          	4
#define MDIO_CLK_25_14          	5
#define MDIO_CLK_25_20          	6
#define MDIO_CLK_25_28          	7
#define MDIO_BUSY               	0x8000000
#define MDIO_AP_EN              	0x10000000
#define MDIO_WAIT_TIMES         	1000

/* MII PHY Status Register */
#define REG_PHY_STATUS           	0x1418
#define PHY_GENERAL_STATUS_MASK		0xFFFF
#define PHY_STATUS_RECV_ENABLE		0x0001
#define PHY_OE_PWSP_STATUS_MASK		0x07FF
#define PHY_OE_PWSP_STATUS_SHIFT	16
#define PHY_STATUS_LPW_STATE		0x80000000
/* BIST Control and Status Register0 (for the Packet Memory) */
#define REG_BIST0_CTRL              	0x141c
#define BIST0_NOW                   	0x1
#define BIST0_SRAM_FAIL             	0x2 /* 1: The SRAM failure is
					     * un-repairable  because
					     * it has address decoder
					     * failure or more than 1 cell
					     * stuck-to-x failure */
#define BIST0_FUSE_FLAG             	0x4

/* BIST Control and Status Register1(for the retry buffer of PCI Express) */
#define REG_BIST1_CTRL			0x1420
#define BIST1_NOW                   	0x1
#define BIST1_SRAM_FAIL             	0x2
#define BIST1_FUSE_FLAG             	0x4

/* SerDes Lock Detect Control and Status Register */
#define REG_SERDES			0x1424
#define SERDES_PHY_CLK_SLOWDOWN		BIT(18)
#define SERDES_MAC_CLK_SLOWDOWN		BIT(17)
#define SERDES_SELFB_PLL_MASK		0x3UL
#define SERDES_SELFB_PLL_SHIFT		14
#define SERDES_PHYCLK_SEL_GTX		BIT(13)	/* 1:gtx_clk, 0:25M */
#define SERDES_PCIECLK_SEL_SRDS		BIT(12)	/* 1:serdes,0:25M */
#define SERDES_BUFS_RX_EN		BIT(11)
#define SERDES_PD_RX			BIT(10)
#define SERDES_PLL_EN			BIT(9)
#define SERDES_EN			BIT(8)
#define SERDES_SELFB_PLL_SEL_CSR	BIT(6)	/* 0:state-machine,1:csr */
#define SERDES_SELFB_PLL_CSR_MASK	0x3UL
#define SERDES_SELFB_PLL_CSR_SHIFT	4
#define SERDES_SELFB_PLL_CSR_4		3	/* 4-12% OV-CLK */
#define SERDES_SELFB_PLL_CSR_0		2	/* 0-4% OV-CLK */
#define SERDES_SELFB_PLL_CSR_12		1	/* 12-18% OV-CLK */
#define SERDES_SELFB_PLL_CSR_18		0	/* 18-25% OV-CLK */
#define SERDES_VCO_SLOW			BIT(3)
#define SERDES_VCO_FAST			BIT(2)
#define SERDES_LOCK_DETECT_EN		BIT(1)
#define SERDES_LOCK_DETECT		BIT(0)

#define REG_LPI_DECISN_TIMER            0x143C
#define L2CB_LPI_DESISN_TIMER		0x7D00

#define REG_LPI_CTRL                    0x1440
#define LPI_CTRL_CHK_DA			BIT(31)
#define LPI_CTRL_ENH_TO_MASK		0x1FFFUL
#define LPI_CTRL_ENH_TO_SHIFT		12
#define LPI_CTRL_ENH_TH_MASK		0x1FUL
#define LPI_CTRL_ENH_TH_SHIFT		6
#define LPI_CTRL_ENH_EN			BIT(5)
#define LPI_CTRL_CHK_RX			BIT(4)
#define LPI_CTRL_CHK_STATE		BIT(3)
#define LPI_CTRL_GMII			BIT(2)
#define LPI_CTRL_TO_PHY			BIT(1)
#define LPI_CTRL_EN			BIT(0)

#define REG_LPI_WAIT			0x1444
#define LPI_WAIT_TIMER_MASK		0xFFFFUL
#define LPI_WAIT_TIMER_SHIFT		0

#define REG_LED_CONFIG            	0x142c

/* MAC Control Register  */
#define REG_MAC_CTRL         		0x1480
#define MAC_CTRL_TX_EN			0x1
#define MAC_CTRL_RX_EN			0x2
#define MAC_CTRL_TX_FLOW		0x4
#define MAC_CTRL_RX_FLOW            	0x8
#define MAC_CTRL_LOOPBACK          	0x10
#define MAC_CTRL_DUPLX              	0x20
#define MAC_CTRL_ADD_CRC            	0x40
#define MAC_CTRL_PAD                	0x80
#define MAC_CTRL_LENCHK             	0x100
#define MAC_CTRL_HUGE_EN            	0x200
#define MAC_CTRL_PRMLEN_SHIFT       	10
#define MAC_CTRL_PRMLEN_MASK        	0xf
#define MAC_CTRL_RMV_VLAN           	0x4000
#define MAC_CTRL_PROMIS_EN          	0x8000
#define MAC_CTRL_TX_PAUSE           	0x10000
#define MAC_CTRL_SCNT               	0x20000
#define MAC_CTRL_SRST_TX            	0x40000
#define MAC_CTRL_TX_SIMURST         	0x80000
#define MAC_CTRL_SPEED_SHIFT        	20
#define MAC_CTRL_SPEED_MASK         	0x3
#define MAC_CTRL_DBG_TX_BKPRESURE   	0x400000
#define MAC_CTRL_TX_HUGE            	0x800000
#define MAC_CTRL_RX_CHKSUM_EN       	0x1000000
#define MAC_CTRL_MC_ALL_EN          	0x2000000
#define MAC_CTRL_BC_EN              	0x4000000
#define MAC_CTRL_DBG                	0x8000000
#define MAC_CTRL_SINGLE_PAUSE_EN	0x10000000
#define MAC_CTRL_HASH_ALG_CRC32		0x20000000
#define MAC_CTRL_SPEED_MODE_SW		0x40000000

/* MAC IPG/IFG Control Register  */
#define REG_MAC_IPG_IFG             	0x1484
#define MAC_IPG_IFG_IPGT_SHIFT      	0 	/* Desired back to back
						 * inter-packet gap. The
						 * default is 96-bit time */
#define MAC_IPG_IFG_IPGT_MASK       	0x7f
#define MAC_IPG_IFG_MIFG_SHIFT      	8       /* Minimum number of IFG to
						 * enforce in between RX frames */
#define MAC_IPG_IFG_MIFG_MASK       	0xff  	/* Frame gap below such IFP is dropped */
#define MAC_IPG_IFG_IPGR1_SHIFT     	16   	/* 64bit Carrier-Sense window */
#define MAC_IPG_IFG_IPGR1_MASK      	0x7f
#define MAC_IPG_IFG_IPGR2_SHIFT     	24    	/* 96-bit IPG window */
#define MAC_IPG_IFG_IPGR2_MASK      	0x7f

/* MAC STATION ADDRESS  */
#define REG_MAC_STA_ADDR		0x1488

/* Hash table for multicast address */
#define REG_RX_HASH_TABLE		0x1490

/* MAC Half-Duplex Control Register */
#define REG_MAC_HALF_DUPLX_CTRL     	0x1498
#define MAC_HALF_DUPLX_CTRL_LCOL_SHIFT  0      /* Collision Window */
#define MAC_HALF_DUPLX_CTRL_LCOL_MASK   0x3ff
#define MAC_HALF_DUPLX_CTRL_RETRY_SHIFT 12
#define MAC_HALF_DUPLX_CTRL_RETRY_MASK  0xf
#define MAC_HALF_DUPLX_CTRL_EXC_DEF_EN  0x10000
#define MAC_HALF_DUPLX_CTRL_NO_BACK_C   0x20000
#define MAC_HALF_DUPLX_CTRL_NO_BACK_P   0x40000 /* No back-off on backpressure,
						 * immediately start the
						 * transmission after back pressure */
#define MAC_HALF_DUPLX_CTRL_ABEBE        0x80000 /* 1: Alternative Binary Exponential Back-off Enabled */
#define MAC_HALF_DUPLX_CTRL_ABEBT_SHIFT  20      /* Maximum binary exponential number */
#define MAC_HALF_DUPLX_CTRL_ABEBT_MASK   0xf
#define MAC_HALF_DUPLX_CTRL_JAMIPG_SHIFT 24      /* IPG to start JAM for collision based flow control in half-duplex */
#define MAC_HALF_DUPLX_CTRL_JAMIPG_MASK  0xf     /* mode. In unit of 8-bit time */

/* Maximum Frame Length Control Register   */
#define REG_MTU                     	0x149c

/* Wake-On-Lan control register */
#define REG_WOL_CTRL                	0x14a0
#define WOL_PATTERN_EN              	0x00000001
#define WOL_PATTERN_PME_EN              0x00000002
#define WOL_MAGIC_EN                    0x00000004
#define WOL_MAGIC_PME_EN                0x00000008
#define WOL_LINK_CHG_EN                 0x00000010
#define WOL_LINK_CHG_PME_EN             0x00000020
#define WOL_PATTERN_ST                  0x00000100
#define WOL_MAGIC_ST                    0x00000200
#define WOL_LINKCHG_ST                  0x00000400
#define WOL_CLK_SWITCH_EN               0x00008000
#define WOL_PT0_EN                      0x00010000
#define WOL_PT1_EN                      0x00020000
#define WOL_PT2_EN                      0x00040000
#define WOL_PT3_EN                      0x00080000
#define WOL_PT4_EN                      0x00100000
#define WOL_PT5_EN                      0x00200000
#define WOL_PT6_EN                      0x00400000

/* WOL Length ( 2 DWORD ) */
#define REG_WOL_PATTERN_LEN         	0x14a4
#define WOL_PT_LEN_MASK                 0x7f
#define WOL_PT0_LEN_SHIFT               0
#define WOL_PT1_LEN_SHIFT               8
#define WOL_PT2_LEN_SHIFT               16
#define WOL_PT3_LEN_SHIFT               24
#define WOL_PT4_LEN_SHIFT               0
#define WOL_PT5_LEN_SHIFT               8
#define WOL_PT6_LEN_SHIFT               16

/* Internal SRAM Partition Register */
#define RFDX_HEAD_ADDR_MASK		0x03FF
#define RFDX_HARD_ADDR_SHIFT		0
#define RFDX_TAIL_ADDR_MASK		0x03FF
#define RFDX_TAIL_ADDR_SHIFT            16

#define REG_SRAM_RFD0_INFO		0x1500
#define REG_SRAM_RFD1_INFO		0x1504
#define REG_SRAM_RFD2_INFO		0x1508
#define	REG_SRAM_RFD3_INFO		0x150C

#define REG_RFD_NIC_LEN			0x1510 /* In 8-bytes */
#define RFD_NIC_LEN_MASK		0x03FF

#define REG_SRAM_TRD_ADDR           	0x1518
#define TPD_HEAD_ADDR_MASK		0x03FF
#define TPD_HEAD_ADDR_SHIFT		0
#define TPD_TAIL_ADDR_MASK		0x03FF
#define TPD_TAIL_ADDR_SHIFT		16

#define REG_SRAM_TRD_LEN            	0x151C /* In 8-bytes */
#define TPD_NIC_LEN_MASK		0x03FF

#define REG_SRAM_RXF_ADDR          	0x1520
#define REG_SRAM_RXF_LEN            	0x1524
#define REG_SRAM_TXF_ADDR           	0x1528
#define REG_SRAM_TXF_LEN            	0x152C
#define REG_SRAM_TCPH_ADDR          	0x1530
#define REG_SRAM_PKTH_ADDR          	0x1532

/*
 * Load Ptr Register
 * Software sets this bit after the initialization of the head and tail */
#define REG_LOAD_PTR                	0x1534

/*
 * addresses of all descriptors, as well as the following descriptor
 * control register, which triggers each function block to load the head
 * pointer to prepare for the operation. This bit is then self-cleared
 * after one cycle.
 */
#define REG_RX_BASE_ADDR_HI		0x1540
#define REG_TX_BASE_ADDR_HI		0x1544
#define REG_SMB_BASE_ADDR_HI		0x1548
#define REG_SMB_BASE_ADDR_LO		0x154C
#define REG_RFD0_HEAD_ADDR_LO		0x1550
#define REG_RFD1_HEAD_ADDR_LO		0x1554
#define REG_RFD2_HEAD_ADDR_LO		0x1558
#define REG_RFD3_HEAD_ADDR_LO		0x155C
#define REG_RFD_RING_SIZE		0x1560
#define RFD_RING_SIZE_MASK		0x0FFF
#define REG_RX_BUF_SIZE			0x1564
#define RX_BUF_SIZE_MASK		0xFFFF
#define REG_RRD0_HEAD_ADDR_LO		0x1568
#define REG_RRD1_HEAD_ADDR_LO		0x156C
#define REG_RRD2_HEAD_ADDR_LO		0x1570
#define REG_RRD3_HEAD_ADDR_LO		0x1574
#define REG_RRD_RING_SIZE		0x1578
#define RRD_RING_SIZE_MASK		0x0FFF
#define REG_HTPD_HEAD_ADDR_LO		0x157C
#define REG_NTPD_HEAD_ADDR_LO		0x1580
#define REG_TPD_RING_SIZE		0x1584
#define TPD_RING_SIZE_MASK		0xFFFF
#define REG_CMB_BASE_ADDR_LO		0x1588

/* RSS about */
#define REG_RSS_KEY0                    0x14B0
#define REG_RSS_KEY1                    0x14B4
#define REG_RSS_KEY2                    0x14B8
#define REG_RSS_KEY3                    0x14BC
#define REG_RSS_KEY4                    0x14C0
#define REG_RSS_KEY5                    0x14C4
#define REG_RSS_KEY6                    0x14C8
#define REG_RSS_KEY7                    0x14CC
#define REG_RSS_KEY8                    0x14D0
#define REG_RSS_KEY9                    0x14D4
#define REG_IDT_TABLE0                	0x14E0
#define REG_IDT_TABLE1                  0x14E4
#define REG_IDT_TABLE2                  0x14E8
#define REG_IDT_TABLE3                  0x14EC
#define REG_IDT_TABLE4                  0x14F0
#define REG_IDT_TABLE5                  0x14F4
#define REG_IDT_TABLE6                  0x14F8
#define REG_IDT_TABLE7                  0x14FC
#define REG_IDT_TABLE                   REG_IDT_TABLE0
#define REG_RSS_HASH_VALUE              0x15B0
#define REG_RSS_HASH_FLAG               0x15B4
#define REG_BASE_CPU_NUMBER             0x15B8

/* TXQ Control Register */
#define REG_TXQ_CTRL                	0x1590
#define	TXQ_NUM_TPD_BURST_MASK     	0xF
#define TXQ_NUM_TPD_BURST_SHIFT    	0
#define TXQ_CTRL_IP_OPTION_EN		0x10
#define TXQ_CTRL_EN                     0x20
#define TXQ_CTRL_ENH_MODE               0x40
#define TXQ_CTRL_LS_8023_EN		0x80
#define TXQ_TXF_BURST_NUM_SHIFT    	16
#define TXQ_TXF_BURST_NUM_MASK     	0xFFFF

/* Jumbo packet Threshold for task offload */
#define REG_TX_TSO_OFFLOAD_THRESH	0x1594 /* In 8-bytes */
#define TX_TSO_OFFLOAD_THRESH_MASK	0x07FF

#define	REG_TXF_WATER_MARK		0x1598 /* In 8-bytes */
#define TXF_WATER_MARK_MASK		0x0FFF
#define TXF_LOW_WATER_MARK_SHIFT	0
#define TXF_HIGH_WATER_MARK_SHIFT 	16
#define TXQ_CTRL_BURST_MODE_EN		0x80000000

#define REG_THRUPUT_MON_CTRL		0x159C
#define THRUPUT_MON_RATE_MASK		0x3
#define THRUPUT_MON_RATE_SHIFT		0
#define THRUPUT_MON_EN			0x80

/* RXQ Control Register */
#define REG_RXQ_CTRL                	0x15A0
#define ASPM_THRUPUT_LIMIT_MASK		0x3
#define ASPM_THRUPUT_LIMIT_SHIFT	0
#define ASPM_THRUPUT_LIMIT_NO		0x00
#define ASPM_THRUPUT_LIMIT_1M		0x01
#define ASPM_THRUPUT_LIMIT_10M		0x02
#define ASPM_THRUPUT_LIMIT_100M		0x04
#define RXQ1_CTRL_EN			0x10
#define RXQ2_CTRL_EN			0x20
#define RXQ3_CTRL_EN			0x40
#define IPV6_CHKSUM_CTRL_EN		0x80
#define RSS_HASH_BITS_MASK		0x00FF
#define RSS_HASH_BITS_SHIFT		8
#define RSS_HASH_IPV4			0x10000
#define RSS_HASH_IPV4_TCP		0x20000
#define RSS_HASH_IPV6			0x40000
#define RSS_HASH_IPV6_TCP		0x80000
#define RXQ_RFD_BURST_NUM_MASK		0x003F
#define RXQ_RFD_BURST_NUM_SHIFT		20
#define RSS_MODE_MASK			0x0003
#define RSS_MODE_SHIFT			26
#define RSS_NIP_QUEUE_SEL_MASK		0x1
#define RSS_NIP_QUEUE_SEL_SHIFT		28
#define RRS_HASH_CTRL_EN		0x20000000
#define RX_CUT_THRU_EN			0x40000000
#define RXQ_CTRL_EN			0x80000000

#define REG_RFD_FREE_THRESH		0x15A4
#define RFD_FREE_THRESH_MASK		0x003F
#define RFD_FREE_HI_THRESH_SHIFT	0
#define RFD_FREE_LO_THRESH_SHIFT	6

/* RXF flow control register */
#define REG_RXQ_RXF_PAUSE_THRESH    	0x15A8
#define RXQ_RXF_PAUSE_TH_HI_SHIFT       0
#define RXQ_RXF_PAUSE_TH_HI_MASK        0x0FFF
#define RXQ_RXF_PAUSE_TH_LO_SHIFT       16
#define RXQ_RXF_PAUSE_TH_LO_MASK        0x0FFF

#define REG_RXD_DMA_CTRL		0x15AC
#define RXD_DMA_THRESH_MASK		0x0FFF	/* In 8-bytes */
#define RXD_DMA_THRESH_SHIFT		0
#define RXD_DMA_DOWN_TIMER_MASK		0xFFFF
#define RXD_DMA_DOWN_TIMER_SHIFT	16

/* DMA Engine Control Register */
#define REG_DMA_CTRL                	0x15C0
#define DMA_CTRL_DMAR_IN_ORDER          0x1
#define DMA_CTRL_DMAR_ENH_ORDER         0x2
#define DMA_CTRL_DMAR_OUT_ORDER         0x4
#define DMA_CTRL_RCB_VALUE              0x8
#define DMA_CTRL_DMAR_BURST_LEN_MASK    0x0007
#define DMA_CTRL_DMAR_BURST_LEN_SHIFT   4
#define DMA_CTRL_DMAW_BURST_LEN_MASK    0x0007
#define DMA_CTRL_DMAW_BURST_LEN_SHIFT   7
#define DMA_CTRL_DMAR_REQ_PRI           0x400
#define DMA_CTRL_DMAR_DLY_CNT_MASK      0x001F
#define DMA_CTRL_DMAR_DLY_CNT_SHIFT     11
#define DMA_CTRL_DMAW_DLY_CNT_MASK      0x000F
#define DMA_CTRL_DMAW_DLY_CNT_SHIFT     16
#define DMA_CTRL_CMB_EN               	0x100000
#define DMA_CTRL_SMB_EN			0x200000
#define DMA_CTRL_CMB_NOW		0x400000
#define MAC_CTRL_SMB_DIS		0x1000000
#define DMA_CTRL_SMB_NOW		0x80000000

/* CMB/SMB Control Register */
#define REG_SMB_STAT_TIMER		0x15C4	/* 2us resolution */
#define SMB_STAT_TIMER_MASK		0xFFFFFF
#define REG_CMB_TPD_THRESH		0x15C8
#define CMB_TPD_THRESH_MASK		0xFFFF
#define REG_CMB_TX_TIMER		0x15CC	/* 2us resolution */
#define CMB_TX_TIMER_MASK		0xFFFF

/* Mail box */
#define MB_RFDX_PROD_IDX_MASK		0xFFFF
#define REG_MB_RFD0_PROD_IDX		0x15E0
#define REG_MB_RFD1_PROD_IDX		0x15E4
#define REG_MB_RFD2_PROD_IDX		0x15E8
#define REG_MB_RFD3_PROD_IDX		0x15EC

#define MB_PRIO_PROD_IDX_MASK		0xFFFF
#define REG_MB_PRIO_PROD_IDX		0x15F0
#define MB_HTPD_PROD_IDX_SHIFT		0
#define MB_NTPD_PROD_IDX_SHIFT		16

#define MB_PRIO_CONS_IDX_MASK		0xFFFF
#define REG_MB_PRIO_CONS_IDX		0x15F4
#define MB_HTPD_CONS_IDX_SHIFT		0
#define MB_NTPD_CONS_IDX_SHIFT		16

#define REG_MB_RFD01_CONS_IDX		0x15F8
#define MB_RFD0_CONS_IDX_MASK		0x0000FFFF
#define MB_RFD1_CONS_IDX_MASK		0xFFFF0000
#define REG_MB_RFD23_CONS_IDX		0x15FC
#define MB_RFD2_CONS_IDX_MASK		0x0000FFFF
#define MB_RFD3_CONS_IDX_MASK		0xFFFF0000

/* Interrupt Status Register */
#define REG_ISR    			0x1600
#define ISR_SMB				0x00000001
#define ISR_TIMER			0x00000002
/*
 * Software manual interrupt, for debug. Set when SW_MAN_INT_EN is set
 * in Table 51 Selene Master Control Register (Offset 0x1400).
 */
#define ISR_MANUAL         		0x00000004
#define ISR_HW_RXF_OV          		0x00000008 /* RXF overflow interrupt */
#define ISR_RFD0_UR			0x00000010 /* RFD0 under run */
#define ISR_RFD1_UR			0x00000020
#define ISR_RFD2_UR			0x00000040
#define ISR_RFD3_UR			0x00000080
#define ISR_TXF_UR			0x00000100
#define ISR_DMAR_TO_RST			0x00000200
#define ISR_DMAW_TO_RST			0x00000400
#define ISR_TX_CREDIT			0x00000800
#define ISR_GPHY			0x00001000
/* GPHY low power state interrupt */
#define ISR_GPHY_LPW           		0x00002000
#define ISR_TXQ_TO_RST			0x00004000
#define ISR_TX_PKT_0			0x00008000
#define ISR_RX_PKT_0			0x00010000
#define ISR_RX_PKT_1			0x00020000
#define ISR_RX_PKT_2			0x00040000
#define ISR_RX_PKT_3			0x00080000
#define ISR_MAC_RX			0x00100000
#define ISR_MAC_TX			0x00200000
#define ISR_UR_DETECTED			0x00400000
#define ISR_FERR_DETECTED		0x00800000
#define ISR_NFERR_DETECTED		0x01000000
#define ISR_CERR_DETECTED		0x02000000
#define ISR_PHY_LINKDOWN		0x04000000
#define ISR_TX_PKT_1			0x10000000
#define ISR_TX_PKT_2			0x20000000
#define ISR_TX_PKT_3			0x40000000
#define ISR_DIS_INT			0x80000000

/* Interrupt Mask Register */
#define REG_IMR				0x1604

#define IMR_NORMAL_MASK		(\
		ISR_MANUAL	|\
		ISR_DMAR_TO_RST	|\
		ISR_TXQ_TO_RST  |\
		ISR_DMAW_TO_RST	|\
		ISR_GPHY	|\
		ISR_GPHY_LPW    |\
		ISR_PHY_LINKDOWN)

#define ISR_TX_PKT 	(\
	ISR_TX_PKT_0    |\
	ISR_TX_PKT_1    |\
	ISR_TX_PKT_2    |\
	ISR_TX_PKT_3)

#define ISR_RX_PKT 	(\
	ISR_RX_PKT_0    |\
	ISR_RX_PKT_1    |\
	ISR_RX_PKT_2    |\
	ISR_RX_PKT_3)

#define ISR_OVER	(\
	ISR_RFD0_UR 	|\
	ISR_RFD1_UR	|\
	ISR_RFD2_UR	|\
	ISR_RFD3_UR	|\
	ISR_HW_RXF_OV	|\
	ISR_TXF_UR)

#define ISR_ERROR	(\
	ISR_DMAR_TO_RST	|\
	ISR_TXQ_TO_RST  |\
	ISR_DMAW_TO_RST	|\
	ISR_PHY_LINKDOWN)

#define REG_INT_RETRIG_TIMER		0x1608
#define INT_RETRIG_TIMER_MASK		0xFFFF

#define REG_HDS_CTRL			0x160C
#define HDS_CTRL_EN			0x0001
#define HDS_CTRL_BACKFILLSIZE_SHIFT	8
#define HDS_CTRL_BACKFILLSIZE_MASK	0x0FFF
#define HDS_CTRL_MAX_HDRSIZE_SHIFT	20
#define HDS_CTRL_MAC_HDRSIZE_MASK	0x0FFF

#define REG_MAC_RX_STATUS_BIN 		0x1700
#define REG_MAC_RX_STATUS_END 		0x175c
#define REG_MAC_TX_STATUS_BIN 		0x1760
#define REG_MAC_TX_STATUS_END 		0x17c0

/* DEBUG ADDR */
#define REG_DEBUG_DATA0 		0x1900
#define REG_DEBUG_DATA1 		0x1904

#define REG_MT_MAGIC	 		0x1F00
#define REG_MT_MODE			0x1F04
#define REG_MT_SPEED			0x1F08
#define REG_MT_VERSION	 		0x1F0C

#define REG_TPD_PRI1_ADDR_LO		0x157C
#define REG_TPD_PRI0_ADDR_LO		0x1580
#define REG_TPD_PRI2_ADDR_LO		0x1F10
#define REG_TPD_PRI3_ADDR_LO		0x1F14

#define REG_TPD_PRI1_PIDX		0x15F0
#define REG_TPD_PRI0_PIDX		0x15F2
#define REG_TPD_PRI1_CIDX		0x15F4
#define REG_TPD_PRI0_CIDX		0x15F6
#define REG_TPD_PRI3_PIDX		0x1F18
#define REG_TPD_PRI2_PIDX		0x1F1A
#define REG_TPD_PRI3_CIDX		0x1F1C
#define REG_TPD_PRI2_CIDX		0x1F1E

#define MT_MAGIC			0xaabb1234
#define MT_MODE_4Q			BIT(0)

/* PHY Control Register */
#define MII_BMCR			0x00
#define BMCR_SPEED_SELECT_MSB		0x0040  /* bits 6,13: 10=1000, 01=100, 00=10 */
#define BMCR_COLL_TEST_ENABLE		0x0080  /* Collision test enable */
#define BMCR_FULL_DUPLEX		0x0100  /* FDX =1, half duplex =0 */
#define BMCR_RESTART_AUTO_NEG		0x0200  /* Restart auto negotiation */
#define BMCR_ISOLATE			0x0400  /* Isolate PHY from MII */
#define BMCR_POWER_DOWN			0x0800  /* Power down */
#define BMCR_AUTO_NEG_EN		0x1000  /* Auto Neg Enable */
#define BMCR_SPEED_SELECT_LSB		0x2000  /* bits 6,13: 10=1000, 01=100, 00=10 */
#define BMCR_LOOPBACK			0x4000  /* 0 = normal, 1 = loopback */
#define BMCR_RESET			0x8000  /* 0 = normal, 1 = PHY reset */
#define BMCR_SPEED_MASK			0x2040
#define BMCR_SPEED_1000			0x0040
#define BMCR_SPEED_100			0x2000
#define BMCR_SPEED_10			0x0000

/* PHY Status Register */
#define MII_BMSR			0x01
#define BMMSR_EXTENDED_CAPS		0x0001  /* Extended register capabilities */
#define BMSR_JABBER_DETECT		0x0002  /* Jabber Detected */
#define BMSR_LINK_STATUS		0x0004  /* Link Status 1 = link */
#define BMSR_AUTONEG_CAPS		0x0008  /* Auto Neg Capable */
#define BMSR_REMOTE_FAULT		0x0010  /* Remote Fault Detect */
#define BMSR_AUTONEG_COMPLETE		0x0020  /* Auto Neg Complete */
#define BMSR_PREAMBLE_SUPPRESS		0x0040  /* Preamble may be suppressed */
#define BMSR_EXTENDED_STATUS		0x0100  /* Ext. status info in Reg 0x0F */
#define BMSR_100T2_HD_CAPS		0x0200  /* 100T2 Half Duplex Capable */
#define BMSR_100T2_FD_CAPS		0x0400  /* 100T2 Full Duplex Capable */
#define BMSR_10T_HD_CAPS		0x0800  /* 10T   Half Duplex Capable */
#define BMSR_10T_FD_CAPS		0x1000  /* 10T   Full Duplex Capable */
#define BMSR_100X_HD_CAPS		0x2000  /* 100X  Half Duplex Capable */
#define BMMII_SR_100X_FD_CAPS		0x4000  /* 100X  Full Duplex Capable */
#define BMMII_SR_100T4_CAPS		0x8000  /* 100T4 Capable */

#define MII_PHYSID1			0x02
#define MII_PHYSID2			0x03

/* Autoneg Advertisement Register */
#define MII_ADVERTISE			0x04
#define ADVERTISE_SPEED_MASK		0x01E0
#define ADVERTISE_DEFAULT_CAP		0x0DE0

/* 1000BASE-T Control Register */
#define MII_GIGA_CR			0x09
#define GIGA_CR_1000T_REPEATER_DTE	0x0400  /* 1=Repeater/switch device port 0=DTE device */

#define GIGA_CR_1000T_MS_VALUE		0x0800  /* 1=Configure PHY as Master 0=Configure PHY as Slave */
#define GIGA_CR_1000T_MS_ENABLE		0x1000  /* 1=Master/Slave manual config value 0=Automatic Master/Slave config */
#define GIGA_CR_1000T_TEST_MODE_NORMAL	0x0000  /* Normal Operation */
#define GIGA_CR_1000T_TEST_MODE_1	0x2000  /* Transmit Waveform test */
#define GIGA_CR_1000T_TEST_MODE_2	0x4000  /* Master Transmit Jitter test */
#define GIGA_CR_1000T_TEST_MODE_3	0x6000  /* Slave Transmit Jitter test */
#define GIGA_CR_1000T_TEST_MODE_4	0x8000	/* Transmitter Distortion test */
#define GIGA_CR_1000T_SPEED_MASK	0x0300
#define GIGA_CR_1000T_DEFAULT_CAP	0x0300

/* PHY Specific Status Register */
#define MII_GIGA_PSSR			0x11
#define GIGA_PSSR_SPD_DPLX_RESOLVED	0x0800  /* 1=Speed & Duplex resolved */
#define GIGA_PSSR_DPLX			0x2000  /* 1=Duplex 0=Half Duplex */
#define GIGA_PSSR_SPEED			0xC000  /* Speed, bits 14:15 */
#define GIGA_PSSR_10MBS			0x0000  /* 00=10Mbs */
#define GIGA_PSSR_100MBS		0x4000  /* 01=100Mbs */
#define GIGA_PSSR_1000MBS		0x8000  /* 10=1000Mbs */

/* PHY Interrupt Enable Register */
#define MII_IER				0x12
#define IER_LINK_UP			0x0400
#define IER_LINK_DOWN			0x0800

/* PHY Interrupt Status Register */
#define MII_ISR				0x13
#define ISR_LINK_UP			0x0400
#define ISR_LINK_DOWN			0x0800

/* Cable-Detect-Test Control Register */
#define MII_CDTC			0x16
#define CDTC_EN_OFF			0   /* sc */
#define CDTC_EN_BITS			1
#define CDTC_PAIR_OFF			8
#define CDTC_PAIR_BIT			2

/* Cable-Detect-Test Status Register */
#define MII_CDTS			0x1C
#define CDTS_STATUS_OFF			8
#define CDTS_STATUS_BITS		2
#define CDTS_STATUS_NORMAL		0
#define CDTS_STATUS_SHORT		1
#define CDTS_STATUS_OPEN		2
#define CDTS_STATUS_INVALID		3

#define MII_DBG_ADDR			0x1D
#define MII_DBG_DATA			0x1E

#define MII_ANA_CTRL_0			0x0
#define ANA_RESTART_CAL			0x0001
#define ANA_MANUL_SWICH_ON_SHIFT	0x1
#define ANA_MANUL_SWICH_ON_MASK		0xF
#define ANA_MAN_ENABLE			0x0020
#define ANA_SEL_HSP			0x0040
#define ANA_EN_HB			0x0080
#define ANA_EN_HBIAS			0x0100
#define ANA_OEN_125M			0x0200
#define ANA_EN_LCKDT			0x0400
#define ANA_LCKDT_PHY			0x0800
#define ANA_AFE_MODE			0x1000
#define ANA_VCO_SLOW			0x2000
#define ANA_VCO_FAST			0x4000
#define ANA_SEL_CLK125M_DSP		0x8000

#define MII_ANA_CTRL_4			0x4
#define ANA_IECHO_ADJ_MASK		0xF
#define ANA_IECHO_ADJ_3_SHIFT		0
#define ANA_IECHO_ADJ_2_SHIFT		4
#define ANA_IECHO_ADJ_1_SHIFT		8
#define ANA_IECHO_ADJ_0_SHIFT		12

#define MII_ANA_CTRL_5			0x5
#define ANA_SERDES_CDR_BW_SHIFT		0
#define ANA_SERDES_CDR_BW_MASK		0x3
#define ANA_MS_PAD_DBG			0x0004
#define ANA_SPEEDUP_DBG			0x0008
#define ANA_SERDES_TH_LOS_SHIFT		4
#define ANA_SERDES_TH_LOS_MASK		0x3
#define ANA_SERDES_EN_DEEM		0x0040
#define ANA_SERDES_TXELECIDLE		0x0080
#define ANA_SERDES_BEACON		0x0100
#define ANA_SERDES_HALFTXDR		0x0200
#define ANA_SERDES_SEL_HSP		0x0400
#define ANA_SERDES_EN_PLL		0x0800
#define ANA_SERDES_EN			0x1000
#define ANA_SERDES_EN_LCKDT		0x2000

#define MII_ANA_CTRL_11			0xB
#define ANA_PS_HIB_EN			0x8000

#define MII_ANA_CTRL_18			0x12
#define ANA_TEST_MODE_10BT_01SHIFT	0
#define ANA_TEST_MODE_10BT_01MASK	0x3
#define ANA_LOOP_SEL_10BT		0x0004
#define ANA_RGMII_MODE_SW		0x0008
#define ANA_EN_LONGECABLE		0x0010
#define ANA_TEST_MODE_10BT_2		0x0020
#define ANA_EN_10BT_IDLE		0x0400
#define ANA_EN_MASK_TB			0x0800
#define ANA_TRIGGER_SEL_TIMER_SHIFT	12
#define ANA_TRIGGER_SEL_TIMER_MASK	0x3
#define ANA_INTERVAL_SEL_TIMER_SHIFT	14
#define ANA_INTERVAL_SEL_TIMER_MASK	0x3

#define MII_ANA_CTRL_41			0x29
#define ANA_TOP_PS_EN			0x8000

#define MII_ANA_CTRL_54			0x36
#define ANA_LONG_CABLE_TH_100_SHIFT	0
#define ANA_LONG_CABLE_TH_100_MASK	0x3F
#define ANA_DESERVED			0x0040
#define ANA_EN_LIT_CH			0x0080
#define ANA_SHORT_CABLE_TH_100_SHIFT	8
#define ANA_SHORT_CABLE_TH_100_MASK	0x3F
#define ANA_BP_BAD_LINK_ACCUM		0x4000
#define ANA_BP_SMALL_BW			0x8000


#endif /* _ATL1C_H_ */
