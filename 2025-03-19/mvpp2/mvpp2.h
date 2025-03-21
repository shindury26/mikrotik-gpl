/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Definitions for Marvell PPv2 network controller for Armada 375 SoC.
 *
 * Copyright (C) 2014 Marvell
 *
 * Marcin Wojtas <mw@semihalf.com>
 */
#ifndef _MVPP2_H_
#define _MVPP2_H_

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/packet_hook.h>
#include <linux/switch.h>

#ifndef CACHE_LINE_MASK
#define CACHE_LINE_MASK            (~(L1_CACHE_BYTES - 1))
#endif

/* Fifo Registers */
#define MVPP2_RX_DATA_FIFO_SIZE_REG(port)	(0x00 + 4 * (port))
#define MVPP2_RX_ATTR_FIFO_SIZE_REG(port)	(0x20 + 4 * (port))
#define MVPP2_RX_MIN_PKT_SIZE_REG		0x60
#define MVPP2_RX_FIFO_INIT_REG			0x64
#define MVPP22_TX_FIFO_THRESH_REG(port)		(0x8840 + 4 * (port))
#define MVPP22_TX_FIFO_SIZE_REG(port)		(0x8860 + 4 * (port))

/* RX DMA Top Registers */
#define MVPP2_RX_CTRL_REG(port)			(0x140 + 4 * (port))
#define     MVPP2_RX_LOW_LATENCY_PKT_SIZE(s)	(((s) & 0xfff) << 16)
#define     MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK	BIT(31)
#define MVPP2_POOL_BUF_SIZE_REG(pool)		(0x180 + 4 * (pool))
#define     MVPP2_POOL_BUF_SIZE_OFFSET		5
#define MVPP2_RXQ_CONFIG_REG(rxq)		(0x800 + 4 * (rxq))
#define     MVPP2_SNOOP_PKT_SIZE_MASK		0x1ff
#define     MVPP2_SNOOP_BUF_HDR_MASK		BIT(9)
#define     MVPP2_RXQ_POOL_SHORT_OFFS		20
#define     MVPP22_RXQ_POOL_SHORT_MASK		0xf00000
#define     MVPP2_RXQ_POOL_LONG_OFFS		24
#define     MVPP22_RXQ_POOL_LONG_MASK		0xf000000
#define     MVPP2_RXQ_PACKET_OFFSET_OFFS	28
#define     MVPP2_RXQ_PACKET_OFFSET_MASK	0x70000000
#define     MVPP2_RXQ_DISABLE_MASK		BIT(31)
/* Total max number of hw RX queues */
#define MVPP2_RXQ_MAX_NUM			128

/* Top Registers */
#define MVPP2_MH_REG(port)			(0x5040 + 4 * (port))
#define MVPP2_DSA_NON_EXTENDED			BIT(4)
#define MVPP2_DSA_EXTENDED			BIT(5)
#define MVPP2_VER_ID_REG			0x50b0
#define MVPP2_VER_PP22				0x10
#define MVPP2_VER_PP23				0x11

/* Parser Registers */
#define MVPP2_PRS_INIT_LOOKUP_REG		0x1000
#define     MVPP2_PRS_PORT_LU_MAX		0xf
#define     MVPP2_PRS_PORT_LU_MASK(port)	(0xff << ((port) * 4))
#define     MVPP2_PRS_PORT_LU_VAL(port, val)	((val) << ((port) * 4))
#define MVPP2_PRS_INIT_OFFS_REG(port)		(0x1004 + ((port) & 4))
#define     MVPP2_PRS_INIT_OFF_MASK(port)	(0x3f << (((port) % 4) * 8))
#define     MVPP2_PRS_INIT_OFF_VAL(port, val)	((val) << (((port) % 4) * 8))
#define MVPP2_PRS_MAX_LOOP_REG(port)		(0x100c + ((port) & 4))
#define     MVPP2_PRS_MAX_LOOP_MASK(port)	(0xff << (((port) % 4) * 8))
#define     MVPP2_PRS_MAX_LOOP_VAL(port, val)	((val) << (((port) % 4) * 8))
#define MVPP2_PRS_TCAM_IDX_REG			0x1100
#define MVPP2_PRS_TCAM_DATA_REG(idx)		(0x1104 + (idx) * 4)
#define     MVPP2_PRS_TCAM_INV_MASK		BIT(31)
#define MVPP2_PRS_SRAM_IDX_REG			0x1200
#define MVPP2_PRS_SRAM_DATA_REG(idx)		(0x1204 + (idx) * 4)
#define MVPP2_PRS_TCAM_CTRL_REG			0x1230
#define     MVPP2_PRS_TCAM_EN_MASK		BIT(0)
#define MVPP2_PRS_TCAM_HIT_IDX_REG		0x1240
#define MVPP2_PRS_TCAM_HIT_CNT_REG		0x1244
#define     MVPP2_PRS_TCAM_HIT_CNT_MASK		GENMASK(15, 0)

/* RSS Registers */
#define MVPP22_RSS_INDEX			0x1500
#define     MVPP22_RSS_INDEX_TABLE_ENTRY(idx)	(idx)
#define     MVPP22_RSS_INDEX_TABLE(idx)		((idx) << 8)
#define     MVPP22_RSS_INDEX_QUEUE(idx)		((idx) << 16)
#define MVPP22_RXQ2RSS_TABLE			0x1504
#define     MVPP22_RSS_TABLE_POINTER(p)		(p)
#define MVPP22_RSS_TABLE_ENTRY			0x1508
#define MVPP22_RSS_WIDTH			0x150c

/* Classifier Registers */
#define MVPP2_CLS_MODE_REG			0x1800
#define     MVPP2_CLS_MODE_ACTIVE_MASK		BIT(0)
#define MVPP2_CLS_PORT_WAY_REG			0x1810
#define     MVPP2_CLS_PORT_WAY_MASK(port)	(1 << (port))
#define MVPP2_CLS_LKP_INDEX_REG			0x1814
#define     MVPP2_CLS_LKP_INDEX_WAY_OFFS	6
#define MVPP2_CLS_LKP_TBL_REG			0x1818
#define     MVPP2_CLS_LKP_TBL_RXQ_MASK		0xff
#define     MVPP2_CLS_LKP_FLOW_PTR(flow)	((flow) << 16)
#define     MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK	BIT(25)
#define MVPP2_CLS_FLOW_INDEX_REG		0x1820
#define MVPP2_CLS_FLOW_TBL0_REG			0x1824
#define     MVPP2_CLS_FLOW_TBL0_LAST		BIT(0)
#define     MVPP2_CLS_FLOW_TBL0_ENG_MASK	0x7
#define     MVPP2_CLS_FLOW_TBL0_OFFS		1
#define     MVPP2_CLS_FLOW_TBL0_ENG(x)		((x) << 1)
#define     MVPP2_CLS_FLOW_TBL0_PORT_ID_MASK	0xff
#define     MVPP2_CLS_FLOW_TBL0_PORT_ID(port)	((port) << 4)
#define     MVPP2_CLS_FLOW_TBL0_PORT_ID_SEL	BIT(23)
#define MVPP2_CLS_FLOW_TBL1_REG			0x1828
#define     MVPP2_CLS_FLOW_TBL1_N_FIELDS_MASK	0x7
#define     MVPP2_CLS_FLOW_TBL1_N_FIELDS(x)	(x)
#define     MVPP2_CLS_FLOW_TBL1_LKP_TYPE_MASK	0x3f
#define     MVPP2_CLS_FLOW_TBL1_LKP_TYPE(x)	((x) << 3)
#define     MVPP2_CLS_FLOW_TBL1_PRIO_MASK	0x3f
#define     MVPP2_CLS_FLOW_TBL1_PRIO(x)		((x) << 9)
#define     MVPP2_CLS_FLOW_TBL1_SEQ_MASK	0x7
#define     MVPP2_CLS_FLOW_TBL1_SEQ(x)		((x) << 15)
#define MVPP2_CLS_FLOW_TBL2_REG			0x182c
#define     MVPP2_CLS_FLOW_TBL2_FLD_MASK	0x3f
#define     MVPP2_CLS_FLOW_TBL2_FLD_OFFS(n)	((n) * 6)
#define     MVPP2_CLS_FLOW_TBL2_FLD(n, x)	((x) << ((n) * 6))
#define MVPP2_CLS_OVERSIZE_RXQ_LOW_REG(port)	(0x1980 + ((port) * 4))
#define     MVPP2_CLS_OVERSIZE_RXQ_LOW_BITS	3
#define     MVPP2_CLS_OVERSIZE_RXQ_LOW_MASK	0x7
#define MVPP2_CLS_SWFWD_P2HQ_REG(port)		(0x19b0 + ((port) * 4))
#define MVPP2_CLS_SWFWD_PCTRL_REG		0x19d0
#define     MVPP2_CLS_SWFWD_PCTRL_MASK(port)	(1 << (port))

/* Classifier C2 engine Registers */
#define MVPP22_CLS_C2_TCAM_IDX			0x1b00
#define MVPP22_CLS_C2_TCAM_DATA0		0x1b10
#define MVPP22_CLS_C2_TCAM_DATA1		0x1b14
#define MVPP22_CLS_C2_TCAM_DATA2		0x1b18
#define MVPP22_CLS_C2_TCAM_DATA3		0x1b1c
#define MVPP22_CLS_C2_TCAM_DATA4		0x1b20
#define     MVPP22_CLS_C2_PORT_ID(port)		((port) << 8)
#define MVPP2_CLS2_TCAM_INV_REG			0x1b24
#define     MVPP2_CLS2_TCAM_INV_INVALID		31
#define     MVPP22_CLS_C2_LKP_TYPE(type)	(type)
#define     MVPP22_CLS_C2_LKP_TYPE_MASK		(0x3f)
#define MVPP22_CLS_C2_HIT_CTR			0x1b50
#define MVPP22_CLS_C2_ACT			0x1b60
#define     MVPP22_CLS_C2_ACT_RSS_EN(act)	(((act) & 0x3) << 19)
#define     MVPP22_CLS_C2_ACT_FWD(act)		(((act) & 0x7) << 13)
#define     MVPP22_CLS_C2_ACT_QHIGH(act)	(((act) & 0x3) << 11)
#define     MVPP22_CLS_C2_ACT_QLOW(act)		(((act) & 0x3) << 9)
#define MVPP22_CLS_C2_ATTR0			0x1b64
#define     MVPP22_CLS_C2_ATTR0_QHIGH(qh)	(((qh) & 0x1f) << 24)
#define     MVPP22_CLS_C2_ATTR0_QHIGH_MASK	0x1f
#define     MVPP22_CLS_C2_ATTR0_QHIGH_OFFS	24
#define     MVPP22_CLS_C2_ATTR0_QLOW(ql)	(((ql) & 0x7) << 21)
#define     MVPP22_CLS_C2_ATTR0_QLOW_MASK	0x7
#define     MVPP22_CLS_C2_ATTR0_QLOW_OFFS	21
#define MVPP22_CLS_C2_ATTR1			0x1b68
#define MVPP22_CLS_C2_ATTR2			0x1b6c
#define     MVPP22_CLS_C2_ATTR2_RSS_EN		BIT(30)
#define MVPP22_CLS_C2_ATTR3			0x1b70
#define MVPP2_CLS2_TCAM_CTRL_REG		0x1b90
#define     MVPP2_CLS2_TCAM_CTRL_BYPASS_FIFO_STAGES	BIT(0)

/* Descriptor Manager Top Registers */
#define MVPP2_RXQ_NUM_REG			0x2040
#define MVPP2_RXQ_DESC_ADDR_REG			0x2044
#define     MVPP22_DESC_ADDR_OFFS		8
#define MVPP2_RXQ_DESC_SIZE_REG			0x2048
#define     MVPP2_RXQ_DESC_SIZE_MASK		0x3ff0
#define MVPP2_RXQ_STATUS_UPDATE_REG(rxq)	(0x3000 + 4 * (rxq))
#define     MVPP2_RXQ_NUM_PROCESSED_OFFSET	0
#define     MVPP2_RXQ_NUM_NEW_OFFSET		16
#define MVPP2_RXQ_STATUS_REG(rxq)		(0x3400 + 4 * (rxq))
#define     MVPP2_RXQ_OCCUPIED_MASK		0x3fff
#define     MVPP2_RXQ_NON_OCCUPIED_OFFSET	16
#define     MVPP2_RXQ_NON_OCCUPIED_MASK		0x3fff0000
#define MVPP2_RXQ_THRESH_REG			0x204c
#define     MVPP2_OCCUPIED_THRESH_OFFSET	0
#define     MVPP2_OCCUPIED_THRESH_MASK		0x3fff
#define MVPP2_RXQ_INDEX_REG			0x2050
#define MVPP2_TXQ_NUM_REG			0x2080
#define MVPP2_TXQ_DESC_ADDR_REG			0x2084
#define MVPP2_TXQ_DESC_SIZE_REG			0x2088
#define     MVPP2_TXQ_DESC_SIZE_MASK		0x3ff0
#define MVPP2_TXQ_THRESH_REG			0x2094
#define	    MVPP2_TXQ_THRESH_OFFSET		16
#define	    MVPP2_TXQ_THRESH_MASK		0x3fff
#define MVPP2_AGGR_TXQ_UPDATE_REG		0x2090
#define MVPP2_TXQ_INDEX_REG			0x2098
#define MVPP2_TXQ_PREF_BUF_REG			0x209c
#define     MVPP2_PREF_BUF_PTR(desc)		((desc) & 0xfff)
#define     MVPP2_PREF_BUF_SIZE_4		(BIT(12) | BIT(13))
#define     MVPP2_PREF_BUF_SIZE_16		(BIT(12) | BIT(14))
#define     MVPP2_PREF_BUF_THRESH(val)		((val) << 17)
#define     MVPP2_TXQ_DRAIN_EN_MASK		BIT(31)
#define MVPP2_TXQ_PENDING_REG			0x20a0
#define     MVPP2_TXQ_PENDING_MASK		0x3fff
#define MVPP2_TXQ_INT_STATUS_REG		0x20a4
#define MVPP2_TXQ_SENT_REG(txq)			(0x3c00 + 4 * (txq))
#define     MVPP2_TRANSMITTED_COUNT_OFFSET	16
#define     MVPP2_TRANSMITTED_COUNT_MASK	0x3fff0000
#define MVPP2_TXQ_RSVD_REQ_REG			0x20b0
#define     MVPP2_TXQ_RSVD_REQ_Q_OFFSET		16
#define MVPP2_TXQ_RSVD_RSLT_REG			0x20b4
#define     MVPP2_TXQ_RSVD_RSLT_MASK		0x3fff
#define MVPP2_TXQ_RSVD_CLR_REG			0x20b8
#define     MVPP2_TXQ_RSVD_CLR_OFFSET		16
#define MVPP2_AGGR_TXQ_DESC_ADDR_REG(cpu)	(0x2100 + 4 * (cpu))
#define     MVPP22_AGGR_TXQ_DESC_ADDR_OFFS	8
#define MVPP2_AGGR_TXQ_DESC_SIZE_REG(cpu)	(0x2140 + 4 * (cpu))
#define     MVPP2_AGGR_TXQ_DESC_SIZE_MASK	0x3ff0
#define MVPP2_AGGR_TXQ_STATUS_REG(cpu)		(0x2180 + 4 * (cpu))
#define     MVPP2_AGGR_TXQ_PENDING_MASK		0x3fff
#define MVPP2_AGGR_TXQ_INDEX_REG(cpu)		(0x21c0 + 4 * (cpu))

/* MBUS bridge registers */
#define MVPP2_WIN_BASE(w)			(0x4000 + ((w) << 2))
#define MVPP2_WIN_SIZE(w)			(0x4020 + ((w) << 2))
#define MVPP2_WIN_REMAP(w)			(0x4040 + ((w) << 2))
#define MVPP2_BASE_ADDR_ENABLE			0x4060

/* AXI Bridge Registers */
#define MVPP22_AXI_BM_WR_ATTR_REG		0x4100
#define MVPP22_AXI_BM_RD_ATTR_REG		0x4104
#define MVPP22_AXI_AGGRQ_DESCR_RD_ATTR_REG	0x4110
#define MVPP22_AXI_TXQ_DESCR_WR_ATTR_REG	0x4114
#define MVPP22_AXI_TXQ_DESCR_RD_ATTR_REG	0x4118
#define MVPP22_AXI_RXQ_DESCR_WR_ATTR_REG	0x411c
#define MVPP22_AXI_RX_DATA_WR_ATTR_REG		0x4120
#define MVPP22_AXI_TX_DATA_RD_ATTR_REG		0x4130
#define     MVPP22_AXI_TX_DATA_RD_QOS_ATTRIBUTE (0x3 << 4)
#define MVPP22_AXI_RD_NORMAL_CODE_REG		0x4150
#define MVPP22_AXI_RD_SNOOP_CODE_REG		0x4154
#define MVPP22_AXI_WR_NORMAL_CODE_REG		0x4160
#define MVPP22_AXI_WR_SNOOP_CODE_REG		0x4164

/* Values for AXI Bridge registers */
#define MVPP22_AXI_ATTR_CACHE_OFFS		0
#define MVPP22_AXI_ATTR_DOMAIN_OFFS		12

#define MVPP22_AXI_CODE_CACHE_OFFS		0
#define MVPP22_AXI_CODE_DOMAIN_OFFS		4

#define MVPP22_AXI_CODE_CACHE_NON_CACHE		0x3
#define MVPP22_AXI_CODE_CACHE_WR_CACHE		0x7
#define MVPP22_AXI_CODE_CACHE_RD_CACHE		0xb

#define MVPP22_AXI_CODE_DOMAIN_OUTER_DOM	2
#define MVPP22_AXI_CODE_DOMAIN_SYSTEM		3

/* Interrupt Cause and Mask registers */
#define MVPP2_ISR_TX_THRESHOLD_REG(port)	(0x5140 + 4 * (port))
#define     MVPP2_MAX_ISR_TX_THRESHOLD		0xfffff0

#define MVPP2_ISR_RX_THRESHOLD_REG(rxq)		(0x5200 + 4 * (rxq))
#define     MVPP2_MAX_ISR_RX_THRESHOLD		0xfffff0

#define MVPP22_ISR_RXQ_GROUP_INDEX_REG		0x5400
#define MVPP22_ISR_RXQ_GROUP_INDEX_SUBGROUP_MASK 0xf
#define MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_MASK	0x380
#define MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_OFFSET	7

#define MVPP22_ISR_RXQ_GROUP_INDEX_SUBGROUP_MASK 0xf
#define MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_MASK	0x380

#define MVPP22_ISR_RXQ_SUB_GROUP_CONFIG_REG	0x5404
#define MVPP22_ISR_RXQ_SUB_GROUP_STARTQ_MASK	0x1f
#define MVPP22_ISR_RXQ_SUB_GROUP_SIZE_MASK	0xf00
#define MVPP22_ISR_RXQ_SUB_GROUP_SIZE_OFFSET	8

#define MVPP2_ISR_ENABLE_REG(port)		(0x5420 + 4 * (port))
#define     MVPP2_ISR_ENABLE_INTERRUPT(mask)	((mask) & 0xffff)
#define     MVPP2_ISR_DISABLE_INTERRUPT(mask)	(((mask) << 16) & 0xffff0000)
#define MVPP2_ISR_RX_TX_CAUSE_REG(port)		(0x5480 + 4 * (port))
#define     MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK(variant) \
			(static_branch_unlikely(&variant) ? 0xffff : 0xff)
#define     MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK	0xff0000
#define     MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_OFFSET	16
#define     MVPP2_CAUSE_RX_FIFO_OVERRUN_MASK	BIT(24)
#define     MVPP2_CAUSE_FCS_ERR_MASK		BIT(25)
#define     MVPP2_CAUSE_TX_FIFO_UNDERRUN_MASK	BIT(26)
#define     MVPP2_CAUSE_TX_EXCEPTION_SUM_MASK	BIT(29)
#define     MVPP2_CAUSE_RX_EXCEPTION_SUM_MASK	BIT(30)
#define     MVPP2_CAUSE_MISC_SUM_MASK		BIT(31)
#define MVPP2_ISR_RX_TX_MASK_REG(port)		(0x54a0 + 4 * (port))
#define MVPP2_ISR_PON_RX_TX_MASK_REG		0x54bc
#define     MVPP2_PON_CAUSE_RXQ_OCCUP_DESC_ALL_MASK	0xffff
#define     MVPP2_PON_CAUSE_TXP_OCCUP_DESC_ALL_MASK	0x3fc00000
#define     MVPP2_PON_CAUSE_MISC_SUM_MASK		BIT(31)
#define MVPP2_ISR_MISC_CAUSE_REG		0x55b0
#define MVPP2_ISR_RX_ERR_CAUSE_REG(port)	(0x5520 + 4 * (port))
#define	    MVPP2_ISR_RX_ERR_CAUSE_NONOCC_MASK	0x00ff

/* Buffer Manager registers */
#define MVPP2_BM_POOL_BASE_REG(pool)		(0x6000 + ((pool) * 4))
#define     MVPP2_BM_POOL_BASE_ADDR_MASK	0xfffff80
#define MVPP2_BM_POOL_SIZE_REG(pool)		(0x6040 + ((pool) * 4))
#define     MVPP2_BM_POOL_SIZE_MASK		0xfff0
#define MVPP2_BM_POOL_READ_PTR_REG(pool)	(0x6080 + ((pool) * 4))
#define     MVPP2_BM_POOL_GET_READ_PTR_MASK	0xfff0
#define MVPP2_BM_POOL_PTRS_NUM_REG(pool)	(0x60c0 + ((pool) * 4))
#define     MVPP2_BM_POOL_PTRS_NUM_MASK		0xfff0
#define MVPP2_BM_BPPI_READ_PTR_REG(pool)	(0x6100 + ((pool) * 4))
#define MVPP2_BM_BPPI_PTRS_NUM_REG(pool)	(0x6140 + ((pool) * 4))
#define     MVPP2_BM_BPPI_PTR_NUM_MASK		0x7ff
#define MVPP22_BM_POOL_PTRS_NUM_MASK		0xfff8
#define     MVPP2_BM_BPPI_PREFETCH_FULL_MASK	BIT(16)
#define MVPP2_BM_POOL_CTRL_REG(pool)		(0x6200 + ((pool) * 4))
#define     MVPP2_BM_START_MASK			BIT(0)
#define     MVPP2_BM_STOP_MASK			BIT(1)
#define     MVPP2_BM_STATE_MASK			BIT(4)
#define     MVPP2_BM_LOW_THRESH_OFFS		8
#define     MVPP2_BM_LOW_THRESH_MASK		0x7f00
#define     MVPP2_BM_LOW_THRESH_VALUE(val)	((val) << \
						MVPP2_BM_LOW_THRESH_OFFS)
#define     MVPP2_BM_HIGH_THRESH_OFFS		16
#define     MVPP2_BM_HIGH_THRESH_MASK		0x7f0000
#define     MVPP2_BM_HIGH_THRESH_VALUE(val)	((val) << \
						MVPP2_BM_HIGH_THRESH_OFFS)
#define     MVPP2_BM_BPPI_HIGH_THRESH		0x1E
#define     MVPP2_BM_BPPI_LOW_THRESH		0x1C
#define MVPP2_BM_INTR_CAUSE_REG(pool)		(0x6240 + ((pool) * 4))
#define     MVPP2_BM_RELEASED_DELAY_MASK	BIT(0)
#define     MVPP2_BM_ALLOC_FAILED_MASK		BIT(1)
#define     MVPP2_BM_BPPE_EMPTY_MASK		BIT(2)
#define     MVPP2_BM_BPPE_FULL_MASK		BIT(3)
#define     MVPP2_BM_AVAILABLE_BP_LOW_MASK	BIT(4)
#define MVPP2_BM_INTR_MASK_REG(pool)		(0x6280 + ((pool) * 4))
#define MVPP2_BM_PHY_ALLOC_REG(pool)		(0x6400 + ((pool) * 4))
#define     MVPP2_BM_PHY_ALLOC_GRNTD_MASK	BIT(0)
#define MVPP2_BM_VIRT_ALLOC_REG			0x6440
#define MVPP22_BM_ADDR_HIGH_ALLOC		0x6444
#define     MVPP22_BM_ADDR_HIGH_PHYS_MASK	0xff
#define     MVPP22_BM_ADDR_HIGH_VIRT_MASK	0xff00
#define     MVPP22_BM_ADDR_HIGH_VIRT_SHIFT	8
#define MVPP2_BM_PHY_RLS_REG(pool)		(0x6480 + ((pool) * 4))
#define     MVPP2_BM_PHY_RLS_MC_BUFF_MASK	BIT(0)
#define     MVPP2_BM_PHY_RLS_PRIO_EN_MASK	BIT(1)
#define     MVPP2_BM_PHY_RLS_GRNTD_MASK		BIT(2)
#define MVPP2_BM_VIRT_RLS_REG			0x64c0
#define MVPP22_BM_ADDR_HIGH_RLS_REG		0x64c4
#define     MVPP22_BM_ADDR_HIGH_PHYS_RLS_MASK	0xff
#define     MVPP22_BM_ADDR_HIGH_VIRT_RLS_MASK	0xff00
#define     MVPP22_BM_ADDR_HIGH_VIRT_RLS_SHIFT	8

#define MVPP22_BM_POOL_BASE_ADDR_HIGH_REG	0x6310
#define     MVPP22_BM_POOL_BASE_ADDR_HIGH_MASK	0xff

/* Hit counters registers */
#define MVPP2_CTRS_IDX				0x7040
#define MVPP2_CLS_DEC_TBL_HIT_CTR		0x7700
#define MVPP2_CLS_FLOW_TBL_HIT_CTR		0x7704

/* TX Scheduler registers */
#define MVPP2_TXP_SCHED_PORT_INDEX_REG		0x8000
#define MVPP2_TXP_SCHED_Q_CMD_REG		0x8004
#define     MVPP2_TXP_SCHED_ENQ_MASK		0xff
#define     MVPP2_TXP_SCHED_DISQ_OFFSET		8
#define MVPP2_TXP_SCHED_CMD_1_REG		0x8010
#define MVPP2_TXP_SCHED_FIXED_PRIO_REG		0x8014
#define MVPP2_TXP_SCHED_PERIOD_REG		0x8018
#define MVPP2_TXP_SCHED_MTU_REG			0x801c
#define     MVPP2_TXP_MTU_MAX			0x7FFFF
#define MVPP2_TXP_SCHED_REFILL_REG		0x8020
#define     MVPP2_TXP_REFILL_TOKENS_ALL_MASK	0x7ffff
#define     MVPP2_TXP_REFILL_PERIOD_ALL_MASK	0x3ff00000
#define     MVPP2_TXP_REFILL_PERIOD_MASK(v)	((v) << 20)
#define MVPP2_TXP_SCHED_TOKEN_SIZE_REG		0x8024
#define     MVPP2_TXP_TOKEN_SIZE_MAX		0xffffffff
#define MVPP2_TXQ_SCHED_REFILL_REG(q)		(0x8040 + ((q) << 2))
#define     MVPP2_TXQ_REFILL_TOKENS_ALL_MASK	0x7ffff
#define     MVPP2_TXQ_REFILL_PERIOD_ALL_MASK	0x3ff00000
#define     MVPP2_TXQ_REFILL_PERIOD_MASK(v)	((v) << 20)
#define MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(q)	(0x8060 + ((q) << 2))
#define     MVPP2_TXQ_TOKEN_SIZE_MAX		0x7fffffff
#define MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(q)	(0x8080 + ((q) << 2))
#define     MVPP2_TXQ_TOKEN_CNTR_MAX		0xffffffff

/* TX general registers */
#define MVPP2_TX_SNOOP_REG			0x8800
#define MVPP2_TX_PORT_FLUSH_REG			0x8810
#define     MVPP2_TX_PORT_FLUSH_MASK(port)	(1 << (port))

/* LMS registers */
#define MVPP2_SRC_ADDR_MIDDLE			0x24
#define MVPP2_SRC_ADDR_HIGH			0x28
#define MVPP2_PHY_AN_CFG0_REG			0x34
#define     MVPP2_PHY_AN_STOP_SMI0_MASK		BIT(7)
#define MVPP2_MNG_EXTENDED_GLOBAL_CTRL_REG	0x305c
#define     MVPP2_EXT_GLOBAL_CTRL_DEFAULT	0x27

/* Per-port registers */
#define MVPP2_GMAC_CTRL_0_REG			0x0
#define     MVPP2_GMAC_PORT_EN_MASK		BIT(0)
#define     MVPP2_GMAC_PORT_TYPE_MASK		BIT(1)
#define     MVPP2_GMAC_MAX_RX_SIZE_OFFS		2
#define     MVPP2_GMAC_MAX_RX_SIZE_MASK		0x7ffc
#define     MVPP2_GMAC_MIB_CNTR_EN_MASK		BIT(15)
#define MVPP2_GMAC_CTRL_1_REG			0x4
#define     MVPP2_GMAC_PERIODIC_XON_EN_MASK	BIT(1)
#define     MVPP2_GMAC_GMII_LB_EN_MASK		BIT(5)
#define     MVPP2_GMAC_PCS_LB_EN_BIT		6
#define     MVPP2_GMAC_PCS_LB_EN_MASK		BIT(6)
#define     MVPP2_GMAC_SA_LOW_OFFS		7
#define MVPP2_GMAC_CTRL_2_REG			0x8
#define     MVPP2_GMAC_INBAND_AN_MASK		BIT(0)
#define     MVPP2_GMAC_FLOW_CTRL_MASK		GENMASK(2, 1)
#define     MVPP2_GMAC_PCS_ENABLE_MASK		BIT(3)
#define     MVPP2_GMAC_INTERNAL_CLK_MASK	BIT(4)
#define     MVPP2_GMAC_DISABLE_PADDING		BIT(5)
#define     MVPP2_GMAC_PORT_RESET_MASK		BIT(6)
#define MVPP2_GMAC_AUTONEG_CONFIG		0xc
#define     MVPP2_GMAC_FORCE_LINK_DOWN		BIT(0)
#define     MVPP2_GMAC_FORCE_LINK_PASS		BIT(1)
#define     MVPP2_GMAC_IN_BAND_AUTONEG		BIT(2)
#define     MVPP2_GMAC_IN_BAND_AUTONEG_BYPASS	BIT(3)
#define     MVPP2_GMAC_IN_BAND_RESTART_AN	BIT(4)
#define     MVPP2_GMAC_CONFIG_MII_SPEED		BIT(5)
#define     MVPP2_GMAC_CONFIG_GMII_SPEED	BIT(6)
#define     MVPP2_GMAC_AN_SPEED_EN		BIT(7)
#define     MVPP2_GMAC_FC_ADV_EN		BIT(9)
#define     MVPP2_GMAC_FC_ADV_ASM_EN		BIT(10)
#define     MVPP2_GMAC_FLOW_CTRL_AUTONEG	BIT(11)
#define     MVPP2_GMAC_CONFIG_FULL_DUPLEX	BIT(12)
#define     MVPP2_GMAC_AN_DUPLEX_EN		BIT(13)
#define MVPP2_GMAC_STATUS0			0x10
#define     MVPP2_GMAC_STATUS0_LINK_UP		BIT(0)
#define     MVPP2_GMAC_STATUS0_GMII_SPEED	BIT(1)
#define     MVPP2_GMAC_STATUS0_MII_SPEED	BIT(2)
#define     MVPP2_GMAC_STATUS0_FULL_DUPLEX	BIT(3)
#define     MVPP2_GMAC_STATUS0_RX_PAUSE		BIT(6)
#define     MVPP2_GMAC_STATUS0_TX_PAUSE		BIT(7)
#define     MVPP2_GMAC_STATUS0_AN_COMPLETE	BIT(11)
#define MVPP2_GMAC_PORT_FIFO_CFG_0_REG		0x18
#define     MVPP2_GMAC_TX_FIFO_WM_MASK		0xffff
#define     MVPP2_GMAC_TX_FIFO_WM_LOW_OFFSET	8
#define MVPP2_GMAC_PORT_FIFO_CFG_1_REG		0x1c
#define     MVPP2_GMAC_TX_FIFO_MIN_TH_OFFS	6
#define     MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK	0x3fc0
#define     MVPP2_GMAC_TX_FIFO_MIN_TH_MASK(v)	(((v) << 6) & \
					MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK)
#define MVPP22_GMAC_INT_STAT			0x20
#define     MVPP22_GMAC_INT_STAT_LINK		BIT(1)
#define MVPP22_GMAC_INT_MASK			0x24
#define     MVPP22_GMAC_INT_MASK_LINK_STAT	BIT(1)
#define MVPP22_GMAC_CTRL_4_REG			0x90
#define     MVPP22_CTRL4_EXT_PIN_GMII_SEL	BIT(0)
#define     MVPP22_CTRL4_RX_FC_EN		BIT(3)
#define     MVPP22_CTRL4_TX_FC_EN		BIT(4)
#define     MVPP22_CTRL4_DP_CLK_SEL		BIT(5)
#define     MVPP22_CTRL4_SYNC_BYPASS_DIS	BIT(6)
#define     MVPP22_CTRL4_QSGMII_BYPASS_ACTIVE	BIT(7)
#define MVPP22_GMAC_INT_SUM_MASK		0xa4
#define     MVPP22_GMAC_INT_SUM_MASK_LINK_STAT	BIT(1)

/* Per-port XGMAC registers. PPv2.2 and PPv2.3, only for GOP port 0,
 * relative to port->base.
 */
#define MVPP22_XLG_CTRL0_REG			0x100
#define     MVPP22_XLG_CTRL0_PORT_EN		BIT(0)
#define     MVPP22_XLG_CTRL0_MAC_RESET_DIS	BIT(1)
#define     MVPP22_XLG_CTRL0_RX_FLOW_CTRL_EN	BIT(7)
#define     MVPP22_XLG_CTRL0_TX_FLOW_CTRL_EN	BIT(8)
#define     MVPP22_XLG_CTRL0_MIB_CNT_DIS	BIT(14)
#define MVPP22_XLG_CTRL1_REG			0x104
#define     MVPP22_XLG_CTRL1_FRAMESIZELIMIT_OFFS	0
#define     MVPP22_XLG_CTRL1_FRAMESIZELIMIT_MASK	0x1fff
#define MVPP22_XLG_STATUS			0x10c
#define     MVPP22_XLG_STATUS_LINK_UP		BIT(0)
#define MVPP22_XLG_INT_STAT			0x114
#define     MVPP22_XLG_INT_STAT_LINK		BIT(1)
#define MVPP22_XLG_INT_MASK			0x118
#define     MVPP22_XLG_INT_MASK_LINK		BIT(1)
#define MVPP22_XLG_CTRL3_REG			0x11c
#define     MVPP22_XLG_CTRL3_MACMODESELECT_MASK	(7 << 13)
#define     MVPP22_XLG_CTRL3_MACMODESELECT_GMAC	(0 << 13)
#define     MVPP22_XLG_CTRL3_MACMODESELECT_10G	(1 << 13)
#define MVPP22_XLG_EXT_INT_MASK			0x15c
#define     MVPP22_XLG_EXT_INT_MASK_XLG		BIT(1)
#define     MVPP22_XLG_EXT_INT_MASK_GIG		BIT(2)
#define MVPP22_XLG_CTRL4_REG			0x184
#define     MVPP22_XLG_CTRL4_FWD_FC		BIT(5)
#define     MVPP22_XLG_CTRL4_FWD_PFC		BIT(6)
#define     MVPP22_XLG_CTRL4_USE_XPCS		BIT(8)
#define     MVPP22_XLG_CTRL4_MACMODSELECT_GMAC	BIT(12)
#define     MVPP22_XLG_CTRL4_EN_IDLE_CHECK	BIT(14)

/* SMI registers. PPv2.2 and PPv2.3, relative to priv->iface_base. */
#define MVPP22_SMI_MISC_CFG_REG			0x1204
#define     MVPP22_SMI_POLLING_EN		BIT(10)

#define MVPP22_GMAC_BASE(port)		(0x7000 + (port) * 0x1000 + 0xe00)

#define MVPP2_CAUSE_TXQ_SENT_DESC_ALL_MASK	0xff

/* XPCS registers.PPv2.2 and PPv2.3 */
#define MVPP22_MPCS_BASE(port)			(0x7000 + (port) * 0x1000)
#define MVPP22_MPCS_CTRL			0x14
#define     MVPP22_MPCS_CTRL_FWD_ERR_CONN	BIT(10)
#define MVPP22_MPCS_CLK_RESET			0x14c
#define     MAC_CLK_RESET_SD_TX			BIT(0)
#define     MAC_CLK_RESET_SD_RX			BIT(1)
#define     MAC_CLK_RESET_MAC			BIT(2)
#define     MVPP22_MPCS_CLK_RESET_DIV_RATIO(n)	((n) << 4)
#define     MVPP22_MPCS_CLK_RESET_DIV_SET	BIT(11)

/* FCA registers. PPv2.2 and PPv2.3 */
#define MVPP22_FCA_BASE(port)			(0x7600 + (port) * 0x1000)
#define MVPP22_FCA_REG_SIZE			16
#define MVPP22_FCA_REG_MASK			0xFFFF
#define MVPP22_FCA_CONTROL_REG			0x0
#define MVPP22_FCA_ENABLE_PERIODIC		BIT(11)
#define MVPP22_PERIODIC_COUNTER_LSB_REG		(0x110)
#define MVPP22_PERIODIC_COUNTER_MSB_REG		(0x114)

/* XPCS registers. PPv2.2 and PPv2.3 */
#define MVPP22_XPCS_BASE(port)			(0x7400 + (port) * 0x1000)
#define MVPP22_XPCS_CFG0			0x0
#define     MVPP22_XPCS_CFG0_RESET_DIS		BIT(0)
#define     MVPP22_XPCS_CFG0_PCS_MODE(n)	((n) << 3)
#define     MVPP22_XPCS_CFG0_ACTIVE_LANE(n)	((n) << 5)

/* System controller registers. Accessed through a regmap. */
#define GENCONF_SOFT_RESET1				0x1108
#define     GENCONF_SOFT_RESET1_GOP			BIT(6)
#define GENCONF_PORT_CTRL0				0x1110
#define     GENCONF_PORT_CTRL0_BUS_WIDTH_SELECT		BIT(1)
#define     GENCONF_PORT_CTRL0_RX_DATA_SAMPLE		BIT(29)
#define     GENCONF_PORT_CTRL0_CLK_DIV_PHASE_CLR	BIT(31)
#define GENCONF_PORT_CTRL1				0x1114
#define     GENCONF_PORT_CTRL1_EN(p)			BIT(p)
#define     GENCONF_PORT_CTRL1_RESET(p)			(BIT(p) << 28)
#define GENCONF_CTRL0					0x1120
#define     GENCONF_CTRL0_PORT0_RGMII			BIT(0)
#define     GENCONF_CTRL0_PORT1_RGMII_MII		BIT(1)
#define     GENCONF_CTRL0_PORT1_RGMII			BIT(2)

/* Various constants */

/* Coalescing */
#define MVPP2_TXDONE_COAL_PKTS_THRESH	32
#define MVPP2_TXDONE_HRTIMER_PERIOD_NS	1000000UL
#define MVPP2_GUARD_TXDONE_HRTIMER_NS	(10 * NSEC_PER_MSEC)
#define MVPP2_TXDONE_COAL_USEC		1000
#define MVPP2_RX_COAL_PKTS		32
#define MVPP2_RX_COAL_USEC		64

/* The two bytes Marvell header. Either contains a special value used
 * by Marvell switches when a specific hardware mode is enabled (not
 * supported by this driver) or is filled automatically by zeroes on
 * the RX side. Those two bytes being at the front of the Ethernet
 * header, they allow to have the IP header aligned on a 4 bytes
 * boundary automatically: the hardware skips those two bytes on its
 * own.
 */
#define MVPP2_MH_SIZE			2
#define MVPP2_ETH_TYPE_LEN		2
#define MVPP2_PPPOE_HDR_SIZE		8
#define MVPP2_VLAN_TAG_LEN		4
#define MVPP2_VLAN_TAG_EDSA_LEN		8

#define MVPP2_IP_LBDT_TYPE		0xfffa

#define MVPP2_TX_CSUM_MAX_SIZE		9800

/* Timeout constants */
#define MVPP2_TX_DISABLE_TIMEOUT_MSEC	1000
#define MVPP2_TX_PENDING_TIMEOUT_MSEC	1000

#define MVPP2_TX_MTU_MAX		0x7ffff

/* Maximum number of T-CONTs of PON port */
#define MVPP2_MAX_TCONT			16

/* Maximum number of supported ports */
#define MVPP2_MAX_PORTS			4

/* Loopback port index */
#define MVPP2_LOOPBACK_PORT_INDEX	3

/* Maximum number of TXQs used by single port */
#define MVPP2_MAX_TXQ			8

#define MVPP2_PORT_MAX_RXQ		32

/* Amount of Tx descriptors that can be reserved at once by CPU */
#define MVPP2_CPU_DESC_CHUNK		64

/* Max number of Tx descriptors in each aggregated queue */
#define MVPP2_AGGR_TXQ_SIZE		512

/* Descriptor aligned size */
#define MVPP2_DESC_ALIGNED_SIZE		32

/* Descriptor alignment mask */
#define MVPP2_DESC_ALIGN		(MVPP2_DESC_ALIGNED_SIZE - 1)

/* RX FIFO constants */
#define MVPP2_RX_FIFO_PORT_DATA_SIZE_44KB	0xb000
#define MVPP2_RX_FIFO_PORT_DATA_SIZE_32KB	0x8000
#define MVPP2_RX_FIFO_PORT_DATA_SIZE_8KB	0x2000
#define MVPP2_RX_FIFO_PORT_DATA_SIZE_4KB	0x1000
#define MVPP2_RX_FIFO_PORT_ATTR_SIZE(data_size)	(data_size >> 6)
#define MVPP2_RX_FIFO_PORT_ATTR_SIZE_4KB	0x40
#define MVPP2_RX_FIFO_PORT_MIN_PKT		0x80

/* TX FIFO constants */
#define MVPP22_TX_FIFO_DATA_SIZE_18KB		18
#define MVPP22_TX_FIFO_DATA_SIZE_10KB		10
#define MVPP22_TX_FIFO_DATA_SIZE_1KB		1
#define MVPP22_TX_FIFO_DATA_SIZE_MIN		3
#define MVPP22_TX_FIFO_DATA_SIZE_MAX		15
#define MVPP2_TX_FIFO_THRESHOLD_MIN		256 /* Bytes */
#define MVPP2_TX_FIFO_THRESHOLD(kb)		\
		(kb * 1024 - MVPP2_TX_FIFO_THRESHOLD_MIN)
#define MVPP22_TX_FIFO_EXTRA_PARAM_MASK		0xFF
#define MVPP22_TX_FIFO_EXTRA_PARAM_OFFS(port)	(8 * (port))
#define MVPP22_TX_FIFO_EXTRA_PARAM_SIZE(port, val)		\
	(((val) >> MVPP22_TX_FIFO_EXTRA_PARAM_OFFS(port)) &	\
	 MVPP22_TX_FIFO_EXTRA_PARAM_MASK)

/* RX Flow Control Registers */
#define MVPP2_RX_FC_REG(port)		(0x150 + 4 * (port))
#define     MVPP2_RX_FC_EN		BIT(24)
#define     MVPP2_RX_FC_TRSH_OFFS	16
#define     MVPP2_RX_FC_TRSH_MASK	(0xFF << MVPP2_RX_FC_TRSH_OFFS)
#define     MVPP2_RX_FC_TRSH_UNIT	256

/* GMAC TX FIFO configuration */
#define MVPP2_GMAC_TX_FIFO_MIN_TH	\
	MVPP2_GMAC_TX_FIFO_MIN_TH_MASK(50)
#define MVPP2_GMAC_TX_FIFO_LOW_WM		75
#define MVPP2_GMAC_TX_FIFO_HI_WM		77

#define MVPP2_BIT_TO_BYTE(bit)		((bit) / 8)
#define MVPP2_BIT_TO_WORD(bit)		((bit) / 32)
#define MVPP2_BIT_IN_WORD(bit)		((bit) % 32)

#define MVPP22_RSS_TABLE_ENTRIES	32

#define MVPP2_F_IF_TX_ON		BIT(3)

enum mvpp2_tag_type {
	MVPP2_TAG_TYPE_NONE = 0,
	MVPP2_TAG_TYPE_MH   = 1,
	MVPP2_TAG_TYPE_DSA  = 2,
	MVPP2_TAG_TYPE_EDSA = 3,
	MVPP2_TAG_TYPE_VLAN = 4,
	MVPP2_TAG_TYPE_LAST = 5
};

enum mvpp2_prs_l2_cast {
	MVPP2_PRS_L2_UNI_CAST,
	MVPP2_PRS_L2_MULTI_CAST,
};

enum mvpp2_prs_l3_cast {
	MVPP2_PRS_L3_UNI_CAST,
	MVPP2_PRS_L3_MULTI_CAST,
	MVPP2_PRS_L3_BROAD_CAST
};

#define MVPP2_MAX_THREADS		9

/* GMAC MIB Counters register definitions */
#define MVPP22_MIB_COUNTERS_OFFSET		0x0
#define MVPP22_MIB_COUNTERS_PORT_SZ		0x100

#define MVPP2_MIB_GOOD_OCTETS_RCVD		0x0
#define MVPP2_MIB_BAD_OCTETS_RCVD		0x8
#define MVPP2_MIB_CRC_ERRORS_SENT		0xc
#define MVPP2_MIB_UNICAST_FRAMES_RCVD		0x10
#define MVPP2_MIB_BROADCAST_FRAMES_RCVD		0x18
#define MVPP2_MIB_MULTICAST_FRAMES_RCVD		0x1c
#define MVPP2_MIB_FRAMES_64_OCTETS		0x20
#define MVPP2_MIB_FRAMES_65_TO_127_OCTETS	0x24
#define MVPP2_MIB_FRAMES_128_TO_255_OCTETS	0x28
#define MVPP2_MIB_FRAMES_256_TO_511_OCTETS	0x2c
#define MVPP2_MIB_FRAMES_512_TO_1023_OCTETS	0x30
#define MVPP2_MIB_FRAMES_1024_TO_MAX_OCTETS	0x34
#define MVPP2_MIB_GOOD_OCTETS_SENT		0x38
#define MVPP2_MIB_UNICAST_FRAMES_SENT		0x40
#define MVPP2_MIB_MULTICAST_FRAMES_SENT		0x48
#define MVPP2_MIB_BROADCAST_FRAMES_SENT		0x4c
#define MVPP2_MIB_FC_SENT			0x54
#define MVPP2_MIB_FC_RCVD			0x58
#define MVPP2_MIB_RX_FIFO_OVERRUN		0x5c
#define MVPP2_MIB_UNDERSIZE_RCVD		0x60
#define MVPP2_MIB_FRAGMENTS_ERR_RCVD		0x64
#define MVPP2_MIB_OVERSIZE_RCVD			0x68
#define MVPP2_MIB_JABBER_RCVD			0x6c
#define MVPP2_MIB_MAC_RCV_ERROR			0x70
#define MVPP2_MIB_BAD_CRC_EVENT			0x74
#define MVPP2_MIB_COLLISION			0x78
#define MVPP2_MIB_LATE_COLLISION		0x7c

#define MVPP2_MIB_COUNTERS_STATS_DELAY		(1 * HZ)

/* Other counters */
#define MVPP2_OVERRUN_DROP_REG(port)		(0x7000 + 4 * (port))
#define MVPP2_CLS_DROP_REG(port)		(0x7020 + 4 * (port))
#define MVPP2_CNT_IDX_REG			0x7040
#define MVPP2_TX_PKT_FULLQ_DROP_REG		0x7200
#define MVPP2_TX_PKT_EARLY_DROP_REG		0x7204
#define MVPP2_TX_PKT_BM_DROP_REG		0x7208
#define MVPP2_TX_PKT_BM_MC_DROP_REG		0x720c
#define MVPP2_RX_PKT_FULLQ_DROP_REG		0x7220
#define MVPP2_RX_PKT_EARLY_DROP_REG		0x7224
#define MVPP2_RX_PKT_BM_DROP_REG		0x7228

#define MVPP2_DESC_DMA_MASK	DMA_BIT_MASK(40)

/* MSS Flow control */
#define MSS_SRAM_SIZE			0x800
#define MSS_FC_COM_REG			0
#define FLOW_CONTROL_ENABLE_BIT		BIT(0)
#define FLOW_CONTROL_UPDATE_COMMAND_BIT	BIT(31)
#define FC_QUANTA			0xFFFF
#define FC_CLK_DIVIDER			0x140

#define MSS_BUF_POOL_BASE		0x40
#define MSS_BUF_POOL_OFFS		4
#define MSS_BUF_POOL_REG(id)		(MSS_BUF_POOL_BASE		\
					+ (id) * MSS_BUF_POOL_OFFS)

#define MSS_BUF_POOL_STOP_MASK		0xFFF
#define MSS_BUF_POOL_START_MASK		(0xFFF << MSS_BUF_POOL_START_OFFS)
#define MSS_BUF_POOL_START_OFFS		12
#define MSS_BUF_POOL_PORTS_MASK		(0xF << MSS_BUF_POOL_PORTS_OFFS)
#define MSS_BUF_POOL_PORTS_OFFS		24
#define MSS_BUF_POOL_PORT_OFFS(id)	(0x1 <<				\
					((id) + MSS_BUF_POOL_PORTS_OFFS))

#define MSS_RXQ_TRESH_BASE		0x200
#define MSS_RXQ_TRESH_OFFS		4
#define MSS_RXQ_TRESH_REG(q, fq)	(MSS_RXQ_TRESH_BASE + (((q) + (fq)) \
					* MSS_RXQ_TRESH_OFFS))

#define MSS_RXQ_TRESH_START_MASK	0xFFFF
#define MSS_RXQ_TRESH_STOP_MASK		(0xFFFF << MSS_RXQ_TRESH_STOP_OFFS)
#define MSS_RXQ_TRESH_STOP_OFFS		16

#define MSS_RXQ_ASS_BASE	0x80
#define MSS_RXQ_ASS_OFFS	4
#define MSS_RXQ_ASS_PER_REG	4
#define MSS_RXQ_ASS_PER_OFFS	8
#define MSS_RXQ_ASS_PORTID_OFFS	0
#define MSS_RXQ_ASS_PORTID_MASK	0x3
#define MSS_RXQ_ASS_HOSTID_OFFS	2
#define MSS_RXQ_ASS_HOSTID_MASK	0x3F

#define MSS_RXQ_ASS_Q_BASE(q, fq) ((((q) + (fq)) % MSS_RXQ_ASS_PER_REG)	 \
				  * MSS_RXQ_ASS_PER_OFFS)
#define MSS_RXQ_ASS_PQ_BASE(q, fq) ((((q) + (fq)) / MSS_RXQ_ASS_PER_REG) \
				   * MSS_RXQ_ASS_OFFS)
#define MSS_RXQ_ASS_REG(q, fq) (MSS_RXQ_ASS_BASE + MSS_RXQ_ASS_PQ_BASE(q, fq))

#define MSS_THRESHOLD_STOP	768
#define MSS_THRESHOLD_START	1024
#define MSS_FC_MAX_TIMEOUT	5000

#define MVPP2_BM_POOLS_NUM	1
#define MVPP2_BM_POOLS_NUM_MAX	8

struct mvpp2_bm_pool {
	int id;
	int size;
	int buf_num;

	u32 *virt_addr;
	dma_addr_t dma_addr;
};

struct mvpp2 {
	void __iomem *lms_base;
	void __iomem *iface_base;
	void __iomem *cm3_base;
	void __iomem *swth_base[MVPP2_MAX_THREADS];
	struct regmap *sysctrl_base;

	struct clk *pp_clk;
	struct clk *gop_clk;
	struct clk *mg_clk;
	struct clk *mg_core_clk;
	struct clk *axi_clk;

	int port_count;
	struct mvpp2_port *port_list[MVPP2_MAX_PORTS];
	unsigned long port_map;

	unsigned int nthreads;

	struct mvpp2_queue *aggr_txqs;

	struct mvpp2_bm_pool bm_pool[MVPP2_BM_POOLS_NUM];

	struct mvpp2_prs_shadow *prs_shadow;
	bool *prs_double_vlans;

	u32 tclk;

	struct gen_pool *sram_pool;

	bool global_tx_fc;
	bool custom_dma_mask;
};

struct mvpp2_queue_vector {
	int id;
	int irq;
	struct napi_struct napi;
	struct mvpp2_port *port;
};

#define MVPP2_TXD_L3_OFF_SHIFT		0
#define MVPP2_TXD_IP_HLEN_SHIFT		8
#define MVPP2_TXD_L4_CSUM_FRAG		BIT(13)
#define MVPP2_TXD_L4_CSUM_NOT		BIT(14)
#define MVPP2_TXD_IP_CSUM_DISABLE	BIT(15)
#define MVPP2_TXD_PADDING_DISABLE	BIT(23)
#define MVPP2_TXD_L4_UDP		BIT(24)
#define MVPP2_TXD_L3_IP6		BIT(26)
#define MVPP2_TXD_L_DESC		BIT(28)
#define MVPP2_TXD_F_DESC		BIT(29)

#define MVPP2_RXD_ERR_SUMMARY		BIT(15)
#define MVPP2_RXD_ERR_CODE_MASK		(BIT(13) | BIT(14))
#define MVPP2_RXD_ERR_CRC		0x0
#define MVPP2_RXD_ERR_OVERRUN		BIT(13)
#define MVPP2_RXD_ERR_RESOURCE		(BIT(13) | BIT(14))
#define MVPP2_RXD_BM_POOL_ID_OFFS	16
#define MVPP2_RXD_BM_POOL_ID_MASK	(BIT(16) | BIT(17) | BIT(18))
#define MVPP2_RXD_HWF_SYNC		BIT(21)
#define MVPP2_RXD_L4_CSUM_OK		BIT(22)
#define MVPP2_RXD_IP4_HEADER_ERR	BIT(24)
#define MVPP2_RXD_L4_TCP		BIT(25)
#define MVPP2_RXD_L4_UDP		BIT(26)
#define MVPP2_RXD_L3_IP4		BIT(28)
#define MVPP2_RXD_L3_IP6		BIT(30)
#define MVPP2_RXD_BUF_HDR		BIT(31)

struct mvpp2_desc {
	__le32 command_status;
	u8  packet_offset;
	u8  phys_txq;
	__le16 data_size;
	__le64 reserved;
	__le64 buf_dma_addr;
	__le64 buf_cookie_misc;
};

struct mvpp2_queue {
	u8 id;
	u8 log_id;
	u16 pending;

	unsigned size;
	unsigned mask;
	unsigned count;

	struct mvpp2_desc *descs;
	dma_addr_t descs_dma;

	void **bufs;
	unsigned put_index;
	unsigned get_index;
	unsigned xmit_commit_count;
} __aligned(L1_CACHE_BYTES);

struct mvpp2_port_config {
	int sfp;
	int sw_num;
	unsigned *port_map;
	struct phy_ops *ops;
};

struct mvpp2_port {
	struct switch_mac sw;
	u8 id;
	int gop_id;
	int link_irq;

	struct mvpp2 *priv;
	struct fwnode_handle *fwnode;

	bool has_phy;

	void __iomem *base;
	void __iomem *stats_base;

	struct mvpp2_queue rxqs[NR_CPUS];
	struct mvpp2_queue txqs[NR_CPUS];

	u16 tx_ring_size;
	u16 rx_ring_size;

	phy_interface_t phy_interface;

	u8 first_rxq;

	struct mvpp2_queue_vector qvecs[MVPP2_MAX_THREADS];

	u32 tx_time_coal;

	u32 indir[MVPP22_RSS_TABLE_ENTRIES];

	bool tx_fc;
	bool has_xlg_mac;

	struct i2c_sfp i2c_sfp;
	struct mvpp2_port_config *cfg;
};

#define MVPP2_DRIVER_NAME "mvpp2"
#define MVPP2_DRIVER_VERSION "1.0"

static inline
void mvpp2_write(struct mvpp2 *priv, u32 offset, u32 data)
{
	writel(data, priv->swth_base[0] + offset);
}

static inline
u32 mvpp2_read(struct mvpp2 *priv, u32 offset)
{
	return readl(priv->swth_base[0] + offset);
}

static inline
u32 mvpp2_read_relaxed(struct mvpp2 *priv, u32 offset)
{
	return readl_relaxed(priv->swth_base[0] + offset);
}

static inline
void mvpp2_cm3_write(struct mvpp2 *priv, u32 offset, u32 data)
{
	writel(data, priv->cm3_base + offset);
}

static inline
u32 mvpp2_cm3_read(struct mvpp2 *priv, u32 offset)
{
	return readl(priv->cm3_base + offset);
}

/* These accessors should be used to access:
 *
 * - per-thread registers, where each thread has its own copy of the
 *   register.
 *
 *   MVPP2_BM_VIRT_ALLOC_REG
 *   MVPP2_BM_ADDR_HIGH_ALLOC
 *   MVPP22_BM_ADDR_HIGH_RLS_REG
 *   MVPP2_BM_VIRT_RLS_REG
 *   MVPP2_ISR_RX_TX_CAUSE_REG
 *   MVPP2_ISR_RX_TX_MASK_REG
 *   MVPP2_TXQ_NUM_REG
 *   MVPP2_AGGR_TXQ_UPDATE_REG
 *   MVPP2_TXQ_RSVD_REQ_REG
 *   MVPP2_TXQ_RSVD_RSLT_REG
 *   MVPP2_TXQ_SENT_REG
 *   MVPP2_RXQ_NUM_REG
 *
 * - global registers that must be accessed through a specific thread
 *   window, because they are related to an access to a per-thread
 *   register
 *
 *   MVPP2_BM_PHY_ALLOC_REG    (related to MVPP2_BM_VIRT_ALLOC_REG)
 *   MVPP2_BM_PHY_RLS_REG      (related to MVPP2_BM_VIRT_RLS_REG)
 *   MVPP2_RXQ_THRESH_REG      (related to MVPP2_RXQ_NUM_REG)
 *   MVPP2_RXQ_DESC_ADDR_REG   (related to MVPP2_RXQ_NUM_REG)
 *   MVPP2_RXQ_DESC_SIZE_REG   (related to MVPP2_RXQ_NUM_REG)
 *   MVPP2_RXQ_INDEX_REG       (related to MVPP2_RXQ_NUM_REG)
 *   MVPP2_TXQ_PENDING_REG     (related to MVPP2_TXQ_NUM_REG)
 *   MVPP2_TXQ_DESC_ADDR_REG   (related to MVPP2_TXQ_NUM_REG)
 *   MVPP2_TXQ_DESC_SIZE_REG   (related to MVPP2_TXQ_NUM_REG)
 *   MVPP2_TXQ_INDEX_REG       (related to MVPP2_TXQ_NUM_REG)
 *   MVPP2_TXQ_PENDING_REG     (related to MVPP2_TXQ_NUM_REG)
 *   MVPP2_TXQ_PREF_BUF_REG    (related to MVPP2_TXQ_NUM_REG)
 *   MVPP2_TXQ_PREF_BUF_REG    (related to MVPP2_TXQ_NUM_REG)
 */
static inline
void mvpp2_thread_write(struct mvpp2 *priv, unsigned int thread,
			u32 offset, u32 data)
{
	writel(data, priv->swth_base[thread] + offset);
}

static inline
u32 mvpp2_thread_read(struct mvpp2 *priv, unsigned int thread, u32 offset)
{
	return readl(priv->swth_base[thread] + offset);
}

static inline
void mvpp2_thread_write_relaxed(struct mvpp2 *priv, unsigned int thread,
				u32 offset, u32 data)
{
	writel_relaxed(data, priv->swth_base[thread] + offset);
}

static inline
u32 mvpp2_thread_read_relaxed(struct mvpp2 *priv, unsigned int thread,
			      u32 offset)
{
	return readl_relaxed(priv->swth_base[thread] + offset);
}

#endif
