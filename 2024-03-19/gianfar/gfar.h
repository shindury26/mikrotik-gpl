#ifndef GIANFAR_H
#define GIANFAR_H

#include <linux/sram.h>

#define DEFAULT_RX_BUFFER_SIZE  1536
#define JUMBO_FRAME_SIZE 9500
#define INCREMENTAL_BUFFER_SIZE 128

#define FSL_GIANFAR_DEV_HAS_GIGABIT		0x00000001
#define FSL_GIANFAR_DEV_HAS_COALESCE		0x00000002
#define FSL_GIANFAR_DEV_HAS_RMON		0x00000004
#define FSL_GIANFAR_DEV_HAS_MULTI_INTR		0x00000008
#define FSL_GIANFAR_DEV_HAS_CSUM		0x00000010
#define FSL_GIANFAR_DEV_HAS_VLAN		0x00000020
#define FSL_GIANFAR_DEV_HAS_EXTENDED_HASH	0x00000040
#define FSL_GIANFAR_DEV_HAS_PADDING		0x00000080
#define FSL_GIANFAR_DEV_HAS_MAGIC_PACKET	0x00000100

#define MII_READ_COMMAND       0x00000001
#define MIIMCFG_INIT_VALUE	0x00000007
#define MIIMCFG_RESET           0x80000000
#define MIIMIND_BUSY            0x00000001
#define MIIMIND_NOTVALID        0x00000004

#define MACCFG1_SOFT_RESET	0x80000000
#define MACCFG1_RESET_RX_MC	0x00080000
#define MACCFG1_RESET_TX_MC	0x00040000
#define MACCFG1_RESET_RX_FUN	0x00020000
#define	MACCFG1_RESET_TX_FUN	0x00010000
#define MACCFG1_LOOPBACK	0x00000100
#define MACCFG1_RX_FLOW		0x00000020
#define MACCFG1_TX_FLOW		0x00000010
#define MACCFG1_SYNCD_RX_EN	0x00000008
#define MACCFG1_RX_EN		0x00000004
#define MACCFG1_SYNCD_TX_EN	0x00000002
#define MACCFG1_TX_EN		0x00000001

#define MACCFG2_INIT_SETTINGS	0x00007205
#define MACCFG2_FULL_DUPLEX	0x00000001
#define MACCFG2_IF              0x00000300
#define MACCFG2_MII             0x00000100
#define MACCFG2_GMII            0x00000200
#define MACCFG2_HUGEFRAME	0x00000020
#define MACCFG2_LENGTHCHECK	0x00000010
#define MACCFG2_MPEN		0x00000008

#define ECNTRL_INIT_SETTINGS	0x00001000
#define ECNTRL_TBI_MODE         0x00000020
#define ECNTRL_REDUCED_MODE	0x00000010
#define ECNTRL_R100		0x00000008
#define ECNTRL_REDUCED_MII_MODE	0x00000004
#define ECNTRL_SGMII_MODE	0x00000002

#define TQUEUE_EN0		0x00008000
#define TQUEUE_EN1		0x00004000
#define TQUEUE_EN2		0x00002000
#define TQUEUE_EN3		0x00001000
#define TQUEUE_EN4		0x00000800
#define TQUEUE_EN5		0x00000400
#define TQUEUE_EN6		0x00000200
#define TQUEUE_EN7		0x00000100
#define TQUEUE_EN_ALL		0x0000FF00

#define TR03WT_WT0_MASK		0xFF000000
#define TR03WT_WT1_MASK		0x00FF0000
#define TR03WT_WT2_MASK		0x0000FF00
#define TR03WT_WT3_MASK		0x000000FF

#define TR47WT_WT4_MASK		0xFF000000
#define TR47WT_WT5_MASK		0x00FF0000
#define TR47WT_WT6_MASK		0x0000FF00
#define TR47WT_WT7_MASK		0x000000FF

#define RQUEUE_EX0		0x00800000
#define RQUEUE_EX1		0x00400000
#define RQUEUE_EX2		0x00200000
#define RQUEUE_EX3		0x00100000
#define RQUEUE_EX4		0x00080000
#define RQUEUE_EX5		0x00040000
#define RQUEUE_EX6		0x00020000
#define RQUEUE_EX7		0x00010000
#define RQUEUE_EX_ALL		0x00FF0000

#define RQUEUE_EN0		0x00000080
#define RQUEUE_EN1		0x00000040
#define RQUEUE_EN2		0x00000020
#define RQUEUE_EN3		0x00000010
#define RQUEUE_EN4		0x00000008
#define RQUEUE_EN5		0x00000004
#define RQUEUE_EN6		0x00000002
#define RQUEUE_EN7		0x00000001
#define RQUEUE_EN_ALL		0x000000FF

#define DMACTRL_INIT_SETTINGS   0x000000c3
#define DMACTRL_GRS             0x00000010
#define DMACTRL_GTS             0x00000008

#define TSTAT_CLEAR_THALT       0x80000000

#define RCTRL_PAL_MASK		0x001f0000
#define RCTRL_VLEX		0x00002000
#define RCTRL_FILREN		0x00001000
#define RCTRL_FSQEN		0x00000800
#define RCTRL_GHTX		0x00000400
#define RCTRL_IPCSEN		0x00000200
#define RCTRL_TUCSEN		0x00000100
#define RCTRL_PRSDEP_MASK	0x000000c0
#define RCTRL_PRSDEP_INIT	0x000000c0
#define RCTRL_PRSFM		0x00000020
#define RCTRL_PROM		0x00000008
#define RCTRL_EMEN		0x00000002
#define RCTRL_CHECKSUMMING	(RCTRL_IPCSEN \
		| RCTRL_TUCSEN | RCTRL_PRSDEP_INIT)
#define RCTRL_EXTHASH		(RCTRL_GHTX)
#define RCTRL_VLAN		(RCTRL_PRSDEP_INIT)
#define RCTRL_PADDING(x)	((x << 16) & RCTRL_PAL_MASK)


#define RSTAT_CLEAR_RHALT       0x00ff0000

#define TCTRL_IPCSEN		0x00004000
#define TCTRL_TUCSEN		0x00002000
#define TCTRL_VLINS		0x00001000
#define TCTRL_TXSCHED_MASK	0x00000006
#define TCTRL_TXSCHED_INIT	0x00000000
#define TCTRL_TXSCHED_PRIO	0x00000002
#define TCTRL_TXSCHED_WRRS	0x00000004
#define TCTRL_INIT_CSUM		(TCTRL_TUCSEN | TCTRL_IPCSEN)

#define IEVENT_INIT_CLEAR	0xffffffff
#define IEVENT_BABR		0x80000000
#define IEVENT_RXC		0x40000000
#define IEVENT_BSY		0x20000000
#define IEVENT_EBERR		0x10000000
#define IEVENT_MSRO		0x04000000
#define IEVENT_GTSC		0x02000000
#define IEVENT_BABT		0x01000000
#define IEVENT_TXC		0x00800000
#define IEVENT_TXE		0x00400000
#define IEVENT_TXB		0x00200000
#define IEVENT_TXF		0x00100000
#define IEVENT_LC		0x00040000
#define IEVENT_CRL		0x00020000
#define IEVENT_XFUN		0x00010000
#define IEVENT_RXB		0x00008000
#define IEVENT_MAG		0x00000800
#define IEVENT_GRSC		0x00000100
#define IEVENT_RXF		0x00000080
#define IEVENT_FIR		0x00000008
#define IEVENT_FIQ		0x00000004
#define IEVENT_DPE		0x00000002
#define IEVENT_PERR		0x00000001
#define IEVENT_RX_MASK          (IEVENT_RXB | IEVENT_RXF | IEVENT_BSY)
#define IEVENT_TX_MASK          (IEVENT_TXB | IEVENT_TXF)
#define IEVENT_ERR_MASK						\
    (IEVENT_BABR | IEVENT_RXC | IEVENT_EBERR | IEVENT_MSRO |	\
     IEVENT_BABT | IEVENT_TXC | IEVENT_TXE | IEVENT_LC		\
     | IEVENT_CRL | IEVENT_XFUN | IEVENT_DPE | IEVENT_PERR	\
     | IEVENT_MAG)

#define IMASK_INIT_CLEAR	0x00000000
#define IMASK_BABR              0x80000000
#define IMASK_RXC               0x40000000
#define IMASK_BSY               0x20000000
#define IMASK_EBERR             0x10000000
#define IMASK_MSRO		0x04000000
#define IMASK_GRSC              0x02000000
#define IMASK_BABT		0x01000000
#define IMASK_TXC               0x00800000
#define IMASK_TXEEN		0x00400000
#define IMASK_TXBEN		0x00200000
#define IMASK_TXFEN             0x00100000
#define IMASK_LC		0x00040000
#define IMASK_CRL		0x00020000
#define IMASK_XFUN		0x00010000
#define IMASK_RXB              0x00008000
#define IMASK_MAG		0x00000800
#define IMASK_GTSC              0x00000100
#define IMASK_RXFEN		0x00000080
#define IMASK_FIR		0x00000008
#define IMASK_FIQ		0x00000004
#define IMASK_DPE		0x00000002
#define IMASK_PERR		0x00000001
#define IMASK_DEFAULT  (IMASK_TXEEN | IMASK_TXFEN | IMASK_TXBEN | \
			IMASK_RXFEN | IMASK_EBERR | IMASK_BABR | \
			IMASK_XFUN | IMASK_RXC | IMASK_BABT | IMASK_DPE \
			| IMASK_PERR)
#define IMASK_RX_MASK (IMASK_RXFEN | IMASK_RXB | IMASK_BSY)
#define IMASK_TX_MASK (IMASK_TXBEN | IMASK_TXFEN)

#define ATTR_BDSTASH		0x00000800
#define ATTR_BUFSTASH		0x00004000
#define ATTR_SNOOPING		0x000000c0



#define FPR_FILER_MASK	0xFFFFFFFF
#define MAX_FILER_IDX	0xFF

// This default RIR value directly corresponds
// to the 3-bit hash value generated
#define DEFAULT_RIR0	0x05397700

// RQFCR register bits
#define RQFCR_GPI		0x80000000
#define RQFCR_HASHTBL_Q		0x00000000
#define RQFCR_HASHTBL_0		0x00020000
#define RQFCR_HASHTBL_1		0x00040000
#define RQFCR_HASHTBL_2		0x00060000
#define RQFCR_HASHTBL_3		0x00080000
#define RQFCR_HASH		0x00010000
#define RQFCR_CLE		0x00000200
#define RQFCR_RJE		0x00000100
#define RQFCR_AND		0x00000080
#define RQFCR_CMP_EXACT		0x00000000
#define RQFCR_CMP_MATCH		0x00000020
#define RQFCR_CMP_NOEXACT	0x00000040
#define RQFCR_CMP_NOMATCH	0x00000060

// RQFCR PID values
#define	RQFCR_PID_MASK		0x00000000
#define	RQFCR_PID_PARSE		0x00000001
#define	RQFCR_PID_ARB		0x00000002
#define	RQFCR_PID_DAH		0x00000003
#define	RQFCR_PID_DAL		0x00000004
#define	RQFCR_PID_SAH		0x00000005
#define	RQFCR_PID_SAL		0x00000006
#define	RQFCR_PID_ETY		0x00000007
#define	RQFCR_PID_VID		0x00000008
#define	RQFCR_PID_PRI		0x00000009
#define	RQFCR_PID_TOS		0x0000000A
#define	RQFCR_PID_L4P		0x0000000B
#define	RQFCR_PID_DIA		0x0000000C
#define	RQFCR_PID_SIA		0x0000000D
#define	RQFCR_PID_DPT		0x0000000E
#define	RQFCR_PID_SPT		0x0000000F

// RQFPR when PID is 0x0001
#define RQFPR_HDR_GE_512	0x00200000
#define RQFPR_LERR		0x00100000
#define RQFPR_RAR		0x00080000
#define RQFPR_RARQ		0x00040000
#define RQFPR_AR		0x00020000
#define RQFPR_ARQ		0x00010000
#define RQFPR_EBC		0x00008000
#define RQFPR_VLN		0x00004000
#define RQFPR_CFI		0x00002000
#define RQFPR_JUM		0x00001000
#define RQFPR_IPF		0x00000800
#define RQFPR_FIF		0x00000400
#define RQFPR_IPV4		0x00000200
#define RQFPR_IPV6		0x00000100
#define RQFPR_ICC		0x00000080
#define RQFPR_ICV		0x00000040
#define RQFPR_TCP		0x00000020
#define RQFPR_UDP		0x00000010
#define RQFPR_TUC		0x00000008
#define RQFPR_TUV		0x00000004
#define RQFPR_PER		0x00000002
#define RQFPR_EER		0x00000001

#define TXBD_READY		0x80000000
#define TXBD_PADCRC		0x40000000
#define TXBD_WRAP		0x20000000
#define TXBD_INTERRUPT		0x10000000
#define TXBD_LAST		0x08000000
#define TXBD_CRC		0x04000000
#define TXBD_DEF		0x02000000
#define TXBD_HUGEFRAME		0x00800000
#define TXBD_LATECOLLISION	0x00800000
#define TXBD_RETRYLIMIT		0x00400000
#define	TXBD_RETRYCOUNTMASK	0x003c0000
#define TXBD_UNDERRUN		0x00020000
#define TXBD_TOE		0x00020000

#define TXFCB_VLN		0x80
#define TXFCB_IP		0x40
#define TXFCB_IP6		0x20
#define TXFCB_TUP		0x10
#define TXFCB_UDP		0x08
#define TXFCB_CIP		0x04
#define TXFCB_CTU		0x02
#define TXFCB_NPH		0x01
#define TXFCB_DEFAULT 		(TXFCB_IP|TXFCB_TUP|TXFCB_CTU|TXFCB_NPH)

#define RXBD_EMPTY		0x80000000
#define RXBD_RO1		0x40000000
#define RXBD_WRAP		0x20000000
#define RXBD_INTERRUPT		0x10000000
#define RXBD_LAST		0x08000000
#define RXBD_FIRST		0x04000000
#define RXBD_MISS		0x01000000
#define RXBD_BROADCAST		0x00800000
#define RXBD_MULTICAST		0x00400000
#define RXBD_LARGE		0x00200000
#define RXBD_NONOCTET		0x00100000
#define RXBD_SHORT		0x00080000
#define RXBD_CRCERR		0x00040000
#define RXBD_OVERRUN		0x00020000
#define RXBD_TRUNCATED		0x00010000
#define RXBD_STATS		0x01ff0000
#define RXBD_ERR		(RXBD_LARGE | RXBD_SHORT | RXBD_NONOCTET \
				 | RXBD_CRCERR | RXBD_OVERRUN		\
				 | RXBD_TRUNCATED)

struct desc {
    u32 status_len;
    u32 buf;
};

struct rmon_mib {
    u32	tr64;	// 0x.680 - Transmit and Receive 64-byte Frame
    u32	tr127;	// 0x.684 - Transmit and Receive 65-127 byte Frame
    u32	tr255;	// 0x.688 - Transmit and Receive 128-255 byte Frame
    u32	tr511;	// 0x.68c - Transmit and Receive 256-511 byte Frame
    u32	tr1k;	// 0x.690 - Transmit and Receive 512-1023 byte Frame
    u32	trmax;	// 0x.694 - Transmit and Receive 1024-1518 byte Frame
    u32	trmgv;	// 0x.698 - Transmit and Receive 1519-1522 byte Good VLAN Frame
    u32	rbyt;	// 0x.69c - Receive Byte
    u32	rpkt;	// 0x.6a0 - Receive Packet
    u32	rfcs;	// 0x.6a4 - Receive FCS Error
    u32	rmca;	// 0x.6a8 - Receive Multicast Packet
    u32	rbca;	// 0x.6ac - Receive Broadcast Packet
    u32	rxcf;	// 0x.6b0 - Receive Control Frame Packet
    u32	rxpf;	// 0x.6b4 - Receive Pause Frame Packet
    u32	rxuo;	// 0x.6b8 - Receive Unknown OP Code
    u32	raln;	// 0x.6bc - Receive Alignment Error
    u32	rflr;	// 0x.6c0 - Receive Frame Length Error
    u32	rcde;	// 0x.6c4 - Receive Code Error
    u32	rcse;	// 0x.6c8 - Receive Carrier Sense Error
    u32	rund;	// 0x.6cc - Receive Undersize Packet
    u32	rovr;	// 0x.6d0 - Receive Oversize Packet
    u32	rfrg;	// 0x.6d4 - Receive Fragments
    u32	rjbr;	// 0x.6d8 - Receive Jabber
    u32	rdrp;	// 0x.6dc - Receive Drop
    u32	tbyt;	// 0x.6e0 - Transmit Byte Counter
    u32	tpkt;	// 0x.6e4 - Transmit Packet
    u32	tmca;	// 0x.6e8 - Transmit Multicast Packet
    u32	tbca;	// 0x.6ec - Transmit Broadcast Packet
    u32	txpf;	// 0x.6f0 - Transmit Pause Control Frame
    u32	tdfr;	// 0x.6f4 - Transmit Deferral Packet
    u32	tedf;	// 0x.6f8 - Transmit Excessive Deferral Packet
    u32	tscl;	// 0x.6fc - Transmit Single Collision Packet
    u32	tmcl;	// 0x.700 - Transmit Multiple Collision Packet
    u32	tlcl;	// 0x.704 - Transmit Late Collision Packet
    u32	txcl;	// 0x.708 - Transmit Excessive Collision Packet
    u32	tncl;	// 0x.70c - Transmit Total Collision
    u8	res1[4];
    u32	tdrp;	// 0x.714 - Transmit Drop Frame
    u32	tjbr;	// 0x.718 - Transmit Jabber Frame
    u32	tfcs;	// 0x.71c - Transmit FCS Error
    u32	txcf;	// 0x.720 - Transmit Control Frame
    u32	tovr;	// 0x.724 - Transmit Oversize Frame
    u32	tund;	// 0x.728 - Transmit Undersize Frame
    u32	tfrg;	// 0x.72c - Transmit Fragments Frame
    u32	car1;	// 0x.730 - Carry Register One
    u32	car2;	// 0x.734 - Carry Register Two
    u32	cam1;	// 0x.738 - Carry Mask Register One
    u32	cam2;	// 0x.73c - Carry Mask Register Two
};

struct gfar_extra_stats {
    u64 rx_large;
    u64 rx_short;
    u64 rx_nonoctet;
    u64 rx_crcerr;
    u64 rx_overrun;
    u64 rx_babr;
    u64 rx_trunc;
    u64 eberr;
    u64 tx_babt;
    u64 tx_underrun;
    u64 rx_skbmissing;
};

#define GFAR_RMON_LEN ((sizeof(struct rmon_mib) - 16)/sizeof(u32))
#define GFAR_EXTRA_STATS_LEN (sizeof(struct gfar_extra_stats)/sizeof(u64))
#define GFAR_STATS_LEN (GFAR_RMON_LEN + GFAR_EXTRA_STATS_LEN)

struct gfar_stats {
    u64 extra[GFAR_EXTRA_STATS_LEN];
    u64 rmon[GFAR_RMON_LEN];
};

struct gfar {
    u32	tsec_id;	// 0x.000 - Controller ID
    u32	tsec_id2;	// 0x.000 - Controller ID
    u8	res1[8];
    u32	ievent;		// 0x.010 - Interrupt Event
    u32	imask;		// 0x.014 - Interrupt Mask
    u32	edis;		// 0x.018 - Error Disabled
    u8	res2[4];
    u32	ecntrl;		// 0x.020 - Ethernet Control
    u32	minflr;		// 0x.024 - Minimum Frame Length
    u32	ptv;		// 0x.028 - Pause Time Value
    u32	dmactrl;	// 0x.02c - DMA Control
    u32	tbipa;		// 0x.030 - TBI PHY Address
    u8	res3[28];
    u32	rxfifoalarm;
    u32	rxfifoalarmshutoff;
    u32	rxfifopanic;
    u32	rxfifopanicshutoff;
    u8	res33[44];
    u32	fifo_tx_thr;	// 0x.08c - FIFO transmit threshold
    u8	res4[8];
    u32	fifo_tx_starve;	// 0x.098 - FIFO transmit starve
    u32	fifo_tx_starve_shutoff;	// 0x.09c - FIFO transmit starve shutoff
    u8	res5[4];
    u32	fifo_rx_pause;	// 0x.0a4 - FIFO receive pause threshold
    u32	fifo_rx_alarm;	// 0x.0a8 - FIFO receive alarm threshold
    u8	res6[84];
    u32	tctrl;		// 0x.100 - Transmit Control
    u32	tstat;		// 0x.104 - Transmit Status
    u32	dfvlan;		// 0x.108 - Default VLAN Control word
    u32	tbdlen;		// 0x.10c - Transmit Buffer Descriptor Data Length
    u32	txic;		// 0x.110 - Transmit Interrupt Coalescing Configuration
    u32	tqueue;		// 0x.114 - Transmit queue control
    u8	res7[40];
    u32	tr03wt;		// 0x.140 - TxBD Rings 0-3 round-robin weightings
    u32	tr47wt;		// 0x.144 - TxBD Rings 4-7 round-robin weightings
    u8	res8[52];
    u32	tbdbph;		// 0x.17c - Tx data buffer pointer high
    u8	res9a[4];
    u32	tbptr0;		// 0x.184 - TxBD Pointer for ring 0
    u8	res9b[4];
    u32	tbptr1;
    u8	res9c[4];
    u32	tbptr2;
    u8	res9d[4];
    u32	tbptr3;
    u8	res9e[4];
    u32	tbptr4;
    u8	res9f[4];
    u32	tbptr5;
    u8	res9g[4];
    u32	tbptr6;
    u8	res9h[4];
    u32	tbptr7;
    u8	res9[64];
    u32	tbaseh;		// 0x.200 - TxBD base address high
    struct {
	u32	tbase;		// 0x.204 - TxBD Base Address of ring 0
	u8	res[4];
    } tx[8];
    u8	res10[188];
    u32	rctrl;		// 0x.300 - Receive Control
    u32	rstat;		// 0x.304 - Receive Status
    u8	res12[8];
    u32	rxic;		// 0x.310 - Receive Interrupt Coalescing Configuration
    u32	rqueue;		// 0x.314 - Receive queue control
    u32	rir0;		/* 0x.318 - Ring mapping register 0 */
    u32	rir1;		/* 0x.31c - Ring mapping register 1 */
    u32	rir2;		/* 0x.320 - Ring mapping register 2 */
    u32	rir3;		/* 0x.324 - Ring mapping register 3 */
    u8	res13[8];
    u32	rbifx;		// 0x.330 - Receive bit field extract control
    u32	rqfar;		// 0x.334 - Receive queue filing table address
    u32	rqfcr;		// 0x.338 - Receive queue filing table control
    u32	rqfpr;		// 0x.33c - Receive queue filing table property
    u32	mrblr;		// 0x.340 - Maximum Receive Buffer Length
    u8	res14[56];
    u32	rbdbph;		// 0x.37c - Rx data buffer pointer high
    u8	res15a[4];
    u32	rbptr0;		// 0x.384 - RxBD pointer for ring 0
    u8	res15b[4];
    u32	rbptr1;
    u8	res15c[4];
    u32	rbptr2;
    u8	res15d[4];
    u32	rbptr3;
    u8	res15e[4];
    u32	rbptr4;
    u8	res15f[4];
    u32	rbptr5;
    u8	res15g[4];
    u32	rbptr6;
    u8	res15h[4];
    u32	rbptr7;
    u8	res16[64];
    u32	rbaseh;		// 0x.400 - RxBD base address high
    struct {
	u32	rbase;		// 0x.404 - RxBD base address of ring 0
	u8	res[4];
    } rx[8];
    u8	res17[188];
    u32	maccfg1;	// 0x.500 - MAC Configuration 1
    u32	maccfg2;	// 0x.504 - MAC Configuration 2
    u32	ipgifg;		// 0x.508 - Inter Packet Gap/Inter Frame Gap
    u32	hafdup;		// 0x.50c - Half Duplex
    u32	maxfrm;		// 0x.510 - Maximum Frame Length
    u8	res18[12];
    u8	gfar_mii_regs[24];
    u8	res19[4];
    u32	ifstat;		// 0x.53c - Interface Status
    u32	macstnaddr1;	// 0x.540 - Station Address Part 1
    u32	macstnaddr2;	// 0x.544 - Station Address Part 2
    u32	mac01addr1;	// 0x.548 - MAC exact match address 1, part 1
    u32	mac01addr2;	// 0x.54c - MAC exact match address 1, part 2
    u32	mac02addr1;
    u32	mac02addr2;
    u32	mac03addr1;
    u32	mac03addr2;
    u32	mac04addr1;
    u32	mac04addr2;
    u32	mac05addr1;
    u32	mac05addr2;
    u32	mac06addr1;
    u32	mac06addr2;
    u32	mac07addr1;
    u32	mac07addr2;
    u32	mac08addr1;
    u32	mac08addr2;
    u32	mac09addr1;
    u32	mac09addr2;
    u32	mac10addr1;
    u32	mac10addr2;
    u32	mac11addr1;
    u32	mac11addr2;
    u32	mac12addr1;
    u32	mac12addr2;
    u32	mac13addr1;
    u32	mac13addr2;
    u32	mac14addr1;
    u32	mac14addr2;
    u32	mac15addr1;
    u32	mac15addr2;
    u8	res20[192];
    struct rmon_mib rmon; // 0x.680-0x.73c
    u32	rrej;		// 0x.740 - Receive filer rejected packet counter
    u8	res21[188];
    u32	igaddr0;	// 0x.800 - Indivdual/Group address 0
    u32	igaddr1;
    u32	igaddr2;
    u32	igaddr3;
    u32	igaddr4;
    u32	igaddr5;
    u32	igaddr6;
    u32	igaddr7;
    u8	res22[96];
    u32	gaddr0;		// 0x.880 - Group address 0
    u32	gaddr1;
    u32	gaddr2;
    u32	gaddr3;
    u32	gaddr4;
    u32	gaddr5;
    u32	gaddr6;
    u32	gaddr7;
    u8	res23a[352];
    u32	fifocfg;	// 0x.a00 - FIFO interface config
    u8	res23b[252];
    u8	res23c[248];
    u32	attr;		// 0x.bf8 - Attributes
    u32	attreli;	// 0x.bfc - Attributes Extract Length and Extract Index
    u8	res24[1024];
};

struct gfar_tx_queue {
    unsigned num;
    unsigned count;
    void **buf_ptr;
    struct desc *desc;
    unsigned cur;
    unsigned dirty;
    unsigned first;
    int xmit_commit;

//#define STATS
#ifdef STATS
    unsigned stats_intr;
    unsigned stats_poll;
    unsigned stats_rx;
    unsigned stats_xmit_pack;
    unsigned stats_tx_pack;
    unsigned stats_tx_desc;
    unsigned stats_tx_recycle;
    unsigned stats_tx_queue_stop;
#endif
} ____cacheline_aligned_in_smp;

struct gfar_rx_queue {
    unsigned num;
    unsigned count;
    void **buf;
    struct desc *desc;
    unsigned cur;
    unsigned dirty;
    struct napi_struct napi;
    call_single_data_t csd;
} ____cacheline_aligned_in_smp;
#define MAX_TX_QUEUE_COUNT 2
#define MAX_RX_QUEUE_COUNT 2

struct gfar_mac {
    struct switch_mac sw;
    struct net_device *dev;
    struct platform_device *ofdev;
    struct gfar __iomem *regs;
    spinlock_t imask_lock;
    int irq_cpu_num;

    unsigned device_flags;
    unsigned phy_id;

    unsigned int irq_tx;
    unsigned int irq_rx;
    unsigned int irq_err;
    char irq_names[3][20];

    unsigned rx_desc_per_queue_count;
    unsigned rx_queue_count;
    unsigned tx_desc_per_queue_count;
    unsigned tx_queue_count;
    unsigned rx_buffer_size;
    unsigned rx_padding;

    struct desc *desc;
    dma_addr_t desc_addr;
    struct gfar_tx_queue tx_queue[MAX_TX_QUEUE_COUNT];
    struct gfar_rx_queue rx_queue[MAX_RX_QUEUE_COUNT];

    struct timer_list phy_timer;
    int speed;
    int duplex;
    int link;

    struct work_struct reset_task;
    struct gfar_extra_stats extra_stats;
    unsigned last_stats[GFAR_RMON_LEN];

#ifdef STATS
    unsigned long stats_last_jiffies;
//    unsigned long long stats_last_clock;
    unsigned stats_restart;
#endif
};

extern int tx_descriptor_count;
extern int rx_descriptor_count;


#endif
