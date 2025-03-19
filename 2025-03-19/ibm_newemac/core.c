/*
 * drivers/net/ibm_newemac/core.c
 *
 * Driver for PowerPC 4xx on-chip ethernet controller.
 *
 * Copyright 2007 Benjamin Herrenschmidt, IBM Corp.
 *                <benh@kernel.crashing.org>
 *
 * Based on the arch/ppc version of the driver:
 *
 * Copyright (c) 2004, 2005 Zultys Technologies.
 * Eugene Surovegin <eugene.surovegin@zultys.com> or <ebs@ebshome.net>
 *
 * Based on original work by
 * 	Matt Porter <mporter@kernel.crashing.org>
 *	(c) 2003 Benjamin Herrenschmidt <benh@kernel.crashing.org>
 *      Armin Kuster <akuster@mvista.com>
 * 	Johnnie Peters <jpeters@mvista.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/crc32.h>
#include <linux/ethtool_extension.h>
#include <linux/mii.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/skb_bin.h>

#include <asm/processor.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/uaccess.h>
#include <asm/dcr.h>
#include <asm/dcr-regs.h>

#include "core.h"

/*
 * Lack of dma_unmap_???? calls is intentional.
 *
 * API-correct usage requires additional support state information to be
 * maintained for every RX and TX buffer descriptor (BD). Unfortunately, due to
 * EMAC design (e.g. TX buffer passed from network stack can be split into
 * several BDs, dma_map_single/dma_map_page can be used to map particular BD),
 * maintaining such information will add additional overhead.
 * Current DMA API implementation for 4xx processors only ensures cache coherency
 * and dma_unmap_???? routines are empty and are likely to stay this way.
 * I decided to omit dma_unmap_??? calls because I don't want to add additional
 * complexity just for the sake of following some abstract API, when it doesn't
 * add any real benefit to the driver. I understand that this decision maybe
 * controversial, but I really tried to make code API-correct and efficient
 * at the same time and didn't come up with code I liked :(.                --ebs
 */

#define DRV_NAME        "emac"
#define DRV_VERSION     "3.54"
#define DRV_DESC        "PPC 4xx OCP EMAC driver"

MODULE_DESCRIPTION(DRV_DESC);
MODULE_AUTHOR
    ("Eugene Surovegin <eugene.surovegin@zultys.com> or <ebs@ebshome.net>");
MODULE_LICENSE("GPL");

/*
 * PPC64 doesn't (yet) have a cacheable_memcpy
 */
#ifdef CONFIG_PPC64
#define cacheable_memcpy(d,s,n) memcpy((d),(s),(n))
#endif

/* minimum number of free TX descriptors required to wake up TX process */
#define EMAC_TX_WAKEUP_THRESH		(NUM_TX_BUFF / 4)

/* If packet size is less than this number, we allocate small skb and copy packet
 * contents into it instead of just sending original big skb up
 */
#define EMAC_RX_COPY_THRESH		CONFIG_IBM_NEW_EMAC_RX_COPY_THRESHOLD

/* Since multiple EMACs share MDIO lines in various ways, we need
 * to avoid re-using the same PHY ID in cases where the arch didn't
 * setup precise phy_map entries
 *
 * XXX This is something that needs to be reworked as we can have multiple
 * EMAC "sets" (multiple ASICs containing several EMACs) though we can
 * probably require in that case to have explicit PHY IDs in the device-tree
 */
static u32 busy_phy_map;
static DEFINE_MUTEX(emac_phy_map_lock);

/* This is the wait queue used to wait on any event related to probe, that
 * is discovery of MALs, other EMACs, ZMII/RGMIIs, etc...
 */
static DECLARE_WAIT_QUEUE_HEAD(emac_probe_wait);

/* Having stable interface names is a doomed idea. However, it would be nice
 * if we didn't have completely random interface names at boot too :-) It's
 * just a matter of making everybody's life easier. Since we are doing
 * threaded probing, it's a bit harder though. The base idea here is that
 * we make up a list of all emacs in the device-tree before we register the
 * driver. Every emac will then wait for the previous one in the list to
 * initialize before itself. We should also keep that list ordered by
 * cell_index.
 * That list is only 4 entries long, meaning that additional EMACs don't
 * get ordering guarantees unless EMAC_BOOT_LIST_SIZE is increased.
 */

#define EMAC_BOOT_LIST_SIZE	4
static struct device_node *emac_boot_list[EMAC_BOOT_LIST_SIZE];

/* How long should I wait for dependent devices ? */
#define EMAC_PROBE_DEP_TIMEOUT	(HZ * 5)

/* I don't want to litter system log with timeout errors
 * when we have brain-damaged PHY.
 */
static inline void emac_report_timeout_error(struct emac_instance *dev,
					     const char *error)
{
	if (emac_has_feature(dev, EMAC_FTR_440GX_PHY_CLK_FIX |
				  EMAC_FTR_460EX_PHY_CLK_FIX |
				  EMAC_FTR_440EP_PHY_CLK_FIX))
		DBG(dev, "%s" NL, error);
	else if (net_ratelimit())
		printk(KERN_ERR "%s: %s\n", dev->ofdev->dev.of_node->full_name,
			error);
}

/* EMAC PHY clock workaround:
 * 440EP/440GR has more sane SDR0_MFR register implementation than 440GX,
 * which allows controlling each EMAC clock
 */
static inline void emac_rx_clk_tx(struct emac_instance *dev)
{
#ifdef CONFIG_PPC_DCR_NATIVE
	if (emac_has_feature(dev, EMAC_FTR_440EP_PHY_CLK_FIX))
		dcri_clrset(SDR0, SDR0_MFR,
			    0, SDR0_MFR_ECS >> dev->cell_index);
#endif
}

static inline void emac_rx_clk_default(struct emac_instance *dev)
{
#ifdef CONFIG_PPC_DCR_NATIVE
	if (emac_has_feature(dev, EMAC_FTR_440EP_PHY_CLK_FIX))
		dcri_clrset(SDR0, SDR0_MFR,
			    SDR0_MFR_ECS >> dev->cell_index, 0);
#endif
}

/* PHY polling intervals */
#define PHY_POLL_LINK_ON	HZ
#define PHY_POLL_LINK_OFF	(HZ / 5)

/* Graceful stop timeouts in us.
 * We should allow up to 1 frame time (full-duplex, ignoring collisions)
 */
#define STOP_TIMEOUT_10		1230
#define STOP_TIMEOUT_100	124
#define STOP_TIMEOUT_1000	13
#define STOP_TIMEOUT_1000_JUMBO	73

static unsigned char default_mcast_addr[] = {
	0x01, 0x80, 0xC2, 0x00, 0x00, 0x01
};

static irqreturn_t emac_irq(int irq, void *dev_instance);
static void emac_clean_tx_ring(struct emac_instance *dev);
static void __emac_set_multicast_list(struct emac_instance *dev);

static inline int emac_phy_supports_gige(int phy_mode)
{
	return  phy_mode == PHY_MODE_GMII ||
		phy_mode == PHY_MODE_RGMII ||
		phy_mode == PHY_MODE_SGMII ||
		phy_mode == PHY_MODE_TBI ||
		phy_mode == PHY_MODE_RTBI;
}

static inline int emac_phy_gpcs(int phy_mode)
{
	return  phy_mode == PHY_MODE_SGMII ||
		phy_mode == PHY_MODE_TBI ||
		phy_mode == PHY_MODE_RTBI;
}

static inline void emac_tx_enable(struct emac_instance *dev)
{
	struct emac_regs __iomem *p = dev->emacp;
	u32 r;

	DBG(dev, "tx_enable" NL);

	r = in_be32(&p->mr0);
	if (!(r & EMAC_MR0_TXE))
		out_be32(&p->mr0, r | EMAC_MR0_TXE);
}

static void emac_tx_disable(struct emac_instance *dev)
{
	struct emac_regs __iomem *p = dev->emacp;
	u32 r;

	DBG(dev, "tx_disable" NL);

	r = in_be32(&p->mr0);
	if (r & EMAC_MR0_TXE) {
		int n = dev->stop_timeout;
		out_be32(&p->mr0, r & ~EMAC_MR0_TXE);
		while (!(in_be32(&p->mr0) & EMAC_MR0_TXI) && n) {
			udelay(1);
			--n;
		}
		if (unlikely(!n))
			emac_report_timeout_error(dev, "TX disable timeout");
	}
}

static void emac_rx_enable(struct emac_instance *dev)
{
	struct emac_regs __iomem *p = dev->emacp;
	u32 r;

	if (unlikely(test_bit(MAL_COMMAC_RX_STOPPED, &dev->commac.flags)))
		goto out;

	DBG(dev, "rx_enable" NL);

	r = in_be32(&p->mr0);
	if (!(r & EMAC_MR0_RXE)) {
		if (unlikely(!(r & EMAC_MR0_RXI))) {
			/* Wait if previous async disable is still in progress */
			int n = dev->stop_timeout;
			while (!(r = in_be32(&p->mr0) & EMAC_MR0_RXI) && n) {
				udelay(1);
				--n;
			}
			if (unlikely(!n))
				emac_report_timeout_error(dev,
							  "RX disable timeout");
		}
		out_be32(&p->mr0, r | EMAC_MR0_RXE);
	}
 out:
	;
}

static void emac_rx_disable(struct emac_instance *dev)
{
	struct emac_regs __iomem *p = dev->emacp;
	u32 r;

	DBG(dev, "rx_disable" NL);

	r = in_be32(&p->mr0);
	if (r & EMAC_MR0_RXE) {
		int n = dev->stop_timeout;
		out_be32(&p->mr0, r & ~EMAC_MR0_RXE);
		while (!(in_be32(&p->mr0) & EMAC_MR0_RXI) && n) {
			udelay(1);
			--n;
		}
		if (unlikely(!n))
			emac_report_timeout_error(dev, "RX disable timeout");
	}
}

static inline void emac_netif_stop(struct emac_instance *dev)
{
	netif_tx_lock_bh(dev->ndev);
	netif_addr_lock(dev->ndev);
	dev->no_mcast = 1;
	netif_addr_unlock(dev->ndev);
	netif_tx_unlock_bh(dev->ndev);
	netif_trans_update(dev->ndev);
	mal_poll_disable(dev->mal, &dev->commac);
	netif_tx_disable(dev->ndev);
}

static inline void emac_netif_start(struct emac_instance *dev)
{
	netif_tx_lock_bh(dev->ndev);
	netif_addr_lock(dev->ndev);
	dev->no_mcast = 0;
	if (dev->mcast_pending && netif_running(dev->ndev))
		__emac_set_multicast_list(dev);
	netif_addr_unlock(dev->ndev);
	netif_tx_unlock_bh(dev->ndev);

	netif_wake_queue(dev->ndev);

	/* NOTE: unconditional netif_wake_queue is only appropriate
	 * so long as all callers are assured to have free tx slots
	 * (taken from tg3... though the case where that is wrong is
	 *  not terribly harmful)
	 */
	mal_poll_enable(dev->mal, &dev->commac);
}

static inline void emac_rx_disable_async(struct emac_instance *dev)
{
	struct emac_regs __iomem *p = dev->emacp;
	u32 r;

	DBG(dev, "rx_disable_async" NL);

	r = in_be32(&p->mr0);
	if (r & EMAC_MR0_RXE)
		out_be32(&p->mr0, r & ~EMAC_MR0_RXE);
}

static int emac_reset(struct emac_instance *dev)
{
	struct emac_regs __iomem *p = dev->emacp;
	int n = 20;

	DBG(dev, "reset" NL);

	if (!dev->reset_failed) {
		/* 40x erratum suggests stopping RX channel before reset,
		 * we stop TX as well
		 */
		emac_rx_disable(dev);
		emac_tx_disable(dev);
	}

#ifdef CONFIG_PPC_DCR_NATIVE
	/* Enable internal clock source */
	if (emac_has_feature(dev, EMAC_FTR_460EX_PHY_CLK_FIX))
		dcri_clrset(SDR0, SDR0_ETH_CFG,
			    0, SDR0_ETH_CFG_ECS << dev->cell_index);
#endif

	out_be32(&p->mr0, EMAC_MR0_SRST);
	while ((in_be32(&p->mr0) & EMAC_MR0_SRST) && n)
		--n;

#ifdef CONFIG_PPC_DCR_NATIVE
	 /* Enable external clock source */
	if (emac_has_feature(dev, EMAC_FTR_460EX_PHY_CLK_FIX))
		dcri_clrset(SDR0, SDR0_ETH_CFG,
			    SDR0_ETH_CFG_ECS << dev->cell_index, 0);
#endif

	// enable TAHOE0 and TAHOE1 bypass as per errata entry CHIP_19
	dcri_clrset(SDR0, SDR0_ETH_CFG, 0, 0x3000);

	if (n) {
		dev->reset_failed = 0;
		return 0;
	} else {
		emac_report_timeout_error(dev, "reset timeout");
		dev->reset_failed = 1;
		return -ETIMEDOUT;
	}
}

static void emac_hash_mc(struct emac_instance *dev)
{
	const int regs = EMAC_XAHT_REGS(dev);
	u32 *gaht_base = emac_gaht_base(dev);
	u32 gaht_temp[regs];
	struct netdev_hw_addr *ha;
	int i;

	DBG(dev, "hash_mc %d" NL, netdev_mc_count(dev->ndev));

	memset(gaht_temp, 0, sizeof (gaht_temp));

	netdev_for_each_mc_addr(ha, dev->ndev) {
		int slot, reg, mask;
		DBG2(dev, "mc %pM" NL, ha->addr);

		slot = EMAC_XAHT_CRC_TO_SLOT(dev,
					     ether_crc(ETH_ALEN, ha->addr));
		reg = EMAC_XAHT_SLOT_TO_REG(dev, slot);
		mask = EMAC_XAHT_SLOT_TO_MASK(dev, slot);

		gaht_temp[reg] |= mask;
	}

	for (i = 0; i < regs; i++)
		out_be32(gaht_base + i, gaht_temp[i]);
}

static inline u32 emac_iff2rmr(struct net_device *ndev)
{
	struct emac_instance *dev = netdev_priv(ndev);
	u32 r;

	r = EMAC_RMR_SP | EMAC_RMR_SFCS | EMAC_RMR_MAE | EMAC_RMR_BAE |
		EMAC_RMR_PME | EMAC_RMR_PMME;

	if (emac_has_feature(dev, EMAC_FTR_EMAC4))
	    r |= EMAC4_RMR_BASE;
	else
	    r |= EMAC_RMR_BASE;

	return r;
}

static u32 __emac_calc_base_mr1(struct emac_instance *dev, int tx_size, int rx_size)
{
	u32 ret = EMAC_MR1_IST | EMAC_MR1_TR0_MULT;

	DBG2(dev, "__emac_calc_base_mr1" NL);

	switch(tx_size) {
	case 2048:
		ret |= EMAC_MR1_TFS_2K;
		break;
	default:
		printk(KERN_WARNING "%s: Unknown Tx FIFO size %d\n",
		       dev->ndev->name, tx_size);
	}

	switch(rx_size) {
	case 16384:
		ret |= EMAC_MR1_RFS_16K;
		break;
	case 4096:
		ret |= EMAC_MR1_RFS_4K;
		break;
	default:
		printk(KERN_WARNING "%s: Unknown Rx FIFO size %d\n",
		       dev->ndev->name, rx_size);
	}

	return ret;
}

static u32 __emac4_calc_base_mr1(struct emac_instance *dev, int tx_size, int rx_size)
{
	u32 ret = EMAC_MR1_IST | EMAC4_MR1_TR |
		EMAC4_MR1_OBCI(dev->opb_bus_freq / 1000000);

	DBG2(dev, "__emac4_calc_base_mr1" NL);

	switch(tx_size) {
	case 16384:
		ret |= EMAC4_MR1_TFS_16K;
		break;
	case 4096:
		ret |= EMAC4_MR1_TFS_4K;
		break;
	case 2048:
		ret |= EMAC4_MR1_TFS_2K;
		break;
	default:
		printk(KERN_WARNING "%s: Unknown Tx FIFO size %d\n",
		       dev->ndev->name, tx_size);
	}

	switch(rx_size) {
	case 16384:
		ret |= EMAC4_MR1_RFS_16K;
		break;
	case 4096:
		ret |= EMAC4_MR1_RFS_4K;
		break;
	case 2048:
		ret |= EMAC4_MR1_RFS_2K;
		break;
	default:
		printk(KERN_WARNING "%s: Unknown Rx FIFO size %d\n",
		       dev->ndev->name, rx_size);
	}

	return ret;
}

static u32 emac_calc_base_mr1(struct emac_instance *dev, int tx_size, int rx_size)
{
	return emac_has_feature(dev, EMAC_FTR_EMAC4) ?
		__emac4_calc_base_mr1(dev, tx_size, rx_size) :
		__emac_calc_base_mr1(dev, tx_size, rx_size);
}

static inline u32 emac_calc_trtr(struct emac_instance *dev, unsigned int size)
{
	if (emac_has_feature(dev, EMAC_FTR_EMAC4))
		return ((size >> 6) - 1) << EMAC_TRTR_SHIFT_EMAC4;
	else
		return ((size >> 6) - 1) << EMAC_TRTR_SHIFT;
}

static inline u32 emac_calc_rwmr(struct emac_instance *dev,
				 unsigned int low, unsigned int high)
{
	if (emac_has_feature(dev, EMAC_FTR_EMAC4))
		return (low << 22) | ( (high & 0x3ff) << 6);
	else
		return (low << 23) | ( (high & 0x1ff) << 7);
}

static int emac_configure(struct emac_instance *dev)
{
	struct emac_regs __iomem *p = dev->emacp;
	struct net_device *ndev = dev->ndev;
	int tx_size, rx_size, link = netif_carrier_ok(dev->ndev);
	u32 r, mr1 = 0;

	DBG(dev, "configure" NL);

	if (!link) {
		out_be32(&p->mr1, in_be32(&p->mr1)
			 | EMAC_MR1_FDE | EMAC_MR1_ILE);
		udelay(100);
	} else if (emac_reset(dev) < 0)
		return -ETIMEDOUT;

	if (emac_has_feature(dev, EMAC_FTR_HAS_TAH))
		tah_reset(dev->tah_dev);

	DBG(dev, " link = %d duplex = %d, pause = %d, asym_pause = %d\n",
	    link, dev->phy.duplex, dev->phy.pause, dev->phy.asym_pause);

	/* Default fifo sizes */
	tx_size = dev->tx_fifo_size;
	rx_size = dev->rx_fifo_size;

	/* No link, force loopback */
	if (!link)
		mr1 = EMAC_MR1_FDE | EMAC_MR1_ILE;

	/* Check for full duplex */
	else if (dev->phy.duplex == DUPLEX_FULL)
		mr1 |= EMAC_MR1_FDE | EMAC4_MR1_MWSW_001;
	mr1 |= EMAC4_MR1_OBCI_100P;

	/* Adjust fifo sizes, mr1 and timeouts based on link speed */
	dev->stop_timeout = STOP_TIMEOUT_10;
	switch (dev->phy.speed) {
	case SPEED_1000:
		if (emac_phy_gpcs(dev->phy.mode)) {
			mr1 |= EMAC_MR1_MF_1000GPCS | EMAC_MR1_MF_IPPA(
				(dev->phy.gpcs_address != 0xffffffff) ?
				 dev->phy.gpcs_address : dev->phy.address);

			/* Put some arbitrary OUI, Manuf & Rev IDs so we can
			 * identify this GPCS PHY later.
			 */
			out_be32(&p->u1.emac4.ipcr, 0xdeadbeef);
		} else
			mr1 |= EMAC_MR1_MF_1000;

		/* Extended fifo sizes */
		tx_size = dev->tx_fifo_size_gige;
		rx_size = dev->rx_fifo_size_gige;
		break;
	case SPEED_100:
		mr1 |= EMAC_MR1_MF_100;
		dev->stop_timeout = STOP_TIMEOUT_100;
		break;
	default: /* make gcc happy */
		break;
	}

	if (dev->ndev->l2mtu > ETH_DATA_LEN) {
		if (emac_has_feature(dev, EMAC_FTR_EMAC4))
			mr1 |= EMAC4_MR1_JPSM;
		else
			mr1 |= EMAC_MR1_JPSM;
		dev->stop_timeout = STOP_TIMEOUT_1000_JUMBO;
	} else
		dev->stop_timeout = STOP_TIMEOUT_1000;

	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII))
		rgmii_set_speed(dev->rgmii_dev, dev->rgmii_port,
				dev->phy.speed);
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII))
		zmii_set_speed(dev->zmii_dev, dev->zmii_port, dev->phy.speed);

	/* on 40x erratum forces us to NOT use integrated flow control,
	 * let's hope it works on 44x ;)
	 */
	if (!emac_has_feature(dev, EMAC_FTR_NO_FLOW_CONTROL_40x) &&
	    dev->phy.duplex == DUPLEX_FULL) {
		if (dev->phy.pause)
			mr1 |= EMAC_MR1_EIFC | EMAC_MR1_APP;
		else if (dev->phy.asym_pause)
			mr1 |= EMAC_MR1_APP;
	}

	/* Add base settings & fifo sizes & program MR1 */
	mr1 |= emac_calc_base_mr1(dev, tx_size, rx_size);
	out_be32(&p->mr1, mr1);

	/* Set individual MAC address */
	out_be32(&p->iahr, (ndev->dev_addr[0] << 8) | ndev->dev_addr[1]);
	out_be32(&p->ialr, (ndev->dev_addr[2] << 24) |
		 (ndev->dev_addr[3] << 16) | (ndev->dev_addr[4] << 8) |
		 ndev->dev_addr[5]);

	/* VLAN Tag Protocol ID */
	out_be32(&p->vtpid, 0);

	/* Receive mode register */
	r = emac_iff2rmr(ndev);
	if (r & EMAC_RMR_MAE)
		emac_hash_mc(dev);
	out_be32(&p->rmr, r);

	/* FIFOs thresholds */
	if (emac_has_feature(dev, EMAC_FTR_EMAC4))
		r = EMAC4_TMR1((dev->mal_burst_size / dev->fifo_entry_size) + 1,
			       tx_size / 2 / dev->fifo_entry_size);
	else
		r = EMAC_TMR1((dev->mal_burst_size / dev->fifo_entry_size) + 1,
			      tx_size / 2 / dev->fifo_entry_size);
	out_be32(&p->tmr1, r);
	out_be32(&p->trtr, emac_calc_trtr(dev, tx_size / 2));

	/* PAUSE frame is sent when RX FIFO reaches its high-water mark,
	   there should be still enough space in FIFO to allow the our link
	   partner time to process this frame and also time to send PAUSE
	   frame itself.

	   Here is the worst case scenario for the RX FIFO "headroom"
	   (from "The Switch Book") (100Mbps, without preamble, inter-frame gap):

	   1) One maximum-length frame on TX                    1522 bytes
	   2) One PAUSE frame time                                64 bytes
	   3) PAUSE frame decode time allowance                   64 bytes
	   4) One maximum-length frame on RX                    1522 bytes
	   5) Round-trip propagation delay of the link (100Mb)    15 bytes
	   ----------
	   3187 bytes

	   I chose to set high-water mark to RX_FIFO_SIZE / 4 (1024 bytes)
	   low-water mark  to RX_FIFO_SIZE / 8 (512 bytes)
	 */
	r = emac_calc_rwmr(dev, rx_size / 8 / dev->fifo_entry_size,
			   rx_size / 4 / dev->fifo_entry_size);
	out_be32(&p->rwmr, r);

	/* Set PAUSE timer to the maximum */
	out_be32(&p->ptr, 0xffff);

	/* IRQ sources */
	r = EMAC_ISR_OVR | EMAC_ISR_BP | EMAC_ISR_SE |
		EMAC_ISR_ALE | EMAC_ISR_BFCS | EMAC_ISR_PTLE | EMAC_ISR_ORE |
		EMAC_ISR_IRE | EMAC_ISR_TE;
	if (emac_has_feature(dev, EMAC_FTR_EMAC4))
	    r |= EMAC4_ISR_TXPE | EMAC4_ISR_RXPE /* | EMAC4_ISR_TXUE |
						  EMAC4_ISR_RXOE | */;
	out_be32(&p->iser,  r);

	/* We need to take GPCS PHY out of isolate mode after EMAC reset */
	if (emac_phy_gpcs(dev->phy.mode)) {
		if (dev->phy.gpcs_address != 0xffffffff)
			emac_mii_reset_gpcs(&dev->phy);
		else
			emac_mii_reset_phy(&dev->phy);
	}

	return 0;
}

static void emac_reinitialize(struct emac_instance *dev)
{
	DBG(dev, "reinitialize" NL);

	emac_netif_stop(dev);
	if (!emac_configure(dev)) {
		emac_tx_enable(dev);
		emac_rx_enable(dev);
	}
	emac_netif_start(dev);
}

static void emac_full_tx_reset(struct emac_instance *dev)
{
	DBG(dev, "full_tx_reset" NL);

	emac_tx_disable(dev);
	mal_disable_tx_channel(dev->mal, dev->mal_tx_chan);
	emac_clean_tx_ring(dev);
	dev->tx_cnt = dev->tx_slot = dev->ack_slot = 0;
	dev->tx_first_slot = -1;
	dev->tx_desc_per_pack = 0;

	emac_configure(dev);

	mal_enable_tx_channel(dev->mal, dev->mal_tx_chan);
	emac_tx_enable(dev);
	emac_rx_enable(dev);
}

static inline int emac_phy_done(struct emac_instance *dev, u32 stacr)
{
	int done = !!(stacr & EMAC_STACR_OC);

	if (emac_has_feature(dev, EMAC_FTR_STACR_OC_INVERT))
		done = !done;

	return done;
};

static int __emac_mdio_read(struct emac_instance *dev, u8 id, u8 reg)
{
	struct emac_regs __iomem *p = dev->emacp;
	u32 r = 0;
	int n, err = -ETIMEDOUT;
	unsigned long flags;

	spin_lock_irqsave(&dev->mdio_lock, flags);

	DBG2(dev, "mdio_read(%02x,%02x)" NL, id, reg);

	/* Enable proper MDIO port */
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII))
		zmii_get_mdio(dev->zmii_dev, dev->zmii_port);
	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII))
		rgmii_get_mdio(dev->rgmii_dev, dev->rgmii_port);

	/* Wait for management interface to become idle */
	n = 20;
	while (!emac_phy_done(dev, in_be32(&p->stacr))) {
		udelay(1);
		if (!--n) {
			DBG2(dev, " -> timeout wait idle\n");
			goto bail;
		}
	}

	/* Issue read command */
	if (emac_has_feature(dev, EMAC_FTR_EMAC4))
		r = EMAC4_STACR_BASE(dev->opb_bus_freq);
	else
		r = EMAC_STACR_BASE(dev->opb_bus_freq);
	if (emac_has_feature(dev, EMAC_FTR_STACR_OC_INVERT))
		r |= EMAC_STACR_OC;
	if (emac_has_feature(dev, EMAC_FTR_HAS_NEW_STACR))
		r |= EMACX_STACR_STAC_READ;
	else
		r |= EMAC_STACR_STAC_READ;
	r |= (reg & EMAC_STACR_PRA_MASK)
		| ((id & EMAC_STACR_PCDA_MASK) << EMAC_STACR_PCDA_SHIFT);
	out_be32(&p->stacr, r);

	/* Wait for read to complete */
	n = 200;
	while (!emac_phy_done(dev, (r = in_be32(&p->stacr)))) {
		udelay(1);
		if (!--n) {
			DBG2(dev, " -> timeout wait complete\n");
			goto bail;
		}
	}

	if (unlikely(r & EMAC_STACR_PHYE)) {
//		DBG(dev, "mdio_read(%02x, %02x) failed" NL, id, reg);
		err = -EREMOTEIO;
		goto bail;
	}

	r = ((r >> EMAC_STACR_PHYD_SHIFT) & EMAC_STACR_PHYD_MASK);

	DBG2(dev, "mdio_read -> %04x" NL, r);
	err = 0;
 bail:
	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII))
		rgmii_put_mdio(dev->rgmii_dev, dev->rgmii_port);
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII))
		zmii_put_mdio(dev->zmii_dev, dev->zmii_port);
	spin_unlock_irqrestore(&dev->mdio_lock, flags);

	return err == 0 ? r : err;
}

static void __emac_mdio_write(struct emac_instance *dev, u8 id, u8 reg,
			      u16 val)
{
	struct emac_regs __iomem *p = dev->emacp;
	u32 r = 0;
	int n, err = -ETIMEDOUT;
	unsigned long flags;

	spin_lock_irqsave(&dev->mdio_lock, flags);

	DBG2(dev, "mdio_write(%02x,%02x,%04x)" NL, id, reg, val);

	/* Enable proper MDIO port */
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII))
		zmii_get_mdio(dev->zmii_dev, dev->zmii_port);
	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII))
		rgmii_get_mdio(dev->rgmii_dev, dev->rgmii_port);

	/* Wait for management interface to be idle */
	n = 20;
	while (!emac_phy_done(dev, in_be32(&p->stacr))) {
		udelay(1);
		if (!--n) {
			DBG2(dev, " -> timeout wait idle\n");
			goto bail;
		}
	}

	/* Issue write command */
	if (emac_has_feature(dev, EMAC_FTR_EMAC4))
		r = EMAC4_STACR_BASE(dev->opb_bus_freq);
	else
		r = EMAC_STACR_BASE(dev->opb_bus_freq);
	if (emac_has_feature(dev, EMAC_FTR_STACR_OC_INVERT))
		r |= EMAC_STACR_OC;
	if (emac_has_feature(dev, EMAC_FTR_HAS_NEW_STACR))
		r |= EMACX_STACR_STAC_WRITE;
	else
		r |= EMAC_STACR_STAC_WRITE;
	r |= (reg & EMAC_STACR_PRA_MASK) |
		((id & EMAC_STACR_PCDA_MASK) << EMAC_STACR_PCDA_SHIFT) |
		(val << EMAC_STACR_PHYD_SHIFT);
	out_be32(&p->stacr, r);

	/* Wait for write to complete */
	n = 200;
	while (!emac_phy_done(dev, in_be32(&p->stacr))) {
		udelay(1);
		if (!--n) {
			DBG2(dev, " -> timeout wait complete\n");
			goto bail;
		}
	}
	err = 0;
 bail:
	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII))
		rgmii_put_mdio(dev->rgmii_dev, dev->rgmii_port);
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII))
		zmii_put_mdio(dev->zmii_dev, dev->zmii_port);
	spin_unlock_irqrestore(&dev->mdio_lock, flags);
}

static int emac_mdio_read(struct net_device *ndev, int id, int reg)
{
	struct emac_instance *dev = netdev_priv(ndev);
	int res;
	int gpcs = dev->phy.gpcs_address;
	if (dev->sw.ops) {
		gpcs = 0xffffffff;
	}

	res = __emac_mdio_read((dev->mdio_instance && gpcs != id) ?
			       dev->mdio_instance : dev,
			       (u8) id, (u8) reg);
	return res;
}

static void emac_mdio_write(struct net_device *ndev, int id, int reg, int val)
{
	struct emac_instance *dev = netdev_priv(ndev);
	int gpcs = dev->phy.gpcs_address;
	if (dev->sw.ops) {
		gpcs = 0xffffffff;
	}

	__emac_mdio_write((dev->mdio_instance && gpcs != id) ?
			  dev->mdio_instance : dev,
			  (u8) id, (u8) reg, (u16) val);
}

static void emac_dump_phy(struct emac_instance *mac, unsigned phy,
			  unsigned reg_from, unsigned reg_to) {
	unsigned i;
	printk("phy %02d: ", phy);
	for (i = 0; i < reg_from; ++i) {
		printk("     ");
	}
	for (i = reg_from; i < reg_to; ++i) {
		int r = __emac_mdio_read(
			mac->mdio_instance ? mac->mdio_instance : mac, phy, i);
		if (r < 0) {
			printk("---- ");
		}
		else {
			printk("%04x ", r);
		}
	}
	printk("\n");
}

static inline void emac_mii_dump(struct emac_instance *mac) {
	unsigned i;

	printk("regs:   ");
	for (i = 0; i < 32; ++i) {
		printk("r%02d  ", i);
	}
	printk("\n");


	for (i = 0; i < 10; ++i) {
		emac_dump_phy(mac, i, 0, 32);
	}
}

static void emac_dump_descriptors(struct emac_instance *dev, unsigned flags) {
	int i;
	printk("TX %d tx_cnt:%d tx_slot:%d ack_slot:%d\n",
	       NUM_TX_BUFF, dev->tx_cnt, dev->tx_slot, dev->ack_slot);
	if (flags & 1) {
		for (i = 0; i < NUM_TX_BUFF; ++i) {
			printk("%d: %04x %d %p\n", i,
			       dev->tx_desc[i].ctrl,
			       dev->tx_desc[i].data_len,
			       (void *)dev->tx_desc[i].data_ptr);
		}
	}
}

static void emac_dump_all(struct emac_instance *dev, unsigned flags) {
	emac_mii_dump(dev);
	emac_dump_descriptors(dev, flags);
}

static int emac_open(struct net_device *ndev);
static int emac_close(struct net_device *ndev);
static void emac_reset_work(struct work_struct *work)
{
	struct emac_instance *dev = container_of(work, struct emac_instance, reset_work);

	DBG(dev, "reset_work" NL);
	printk("%s: reset\n", dev->ndev->name);
#ifdef STATS
	++dev->stats_restart;
#endif

	if (dev->ndev->flags & IFF_UP) {
#ifdef STATS
		emac_dump_descriptors(dev, 1);
#endif
		emac_close(dev->ndev);
		emac_open(dev->ndev);
	}
}

static void emac_tx_timeout(struct net_device *ndev, unsigned txqueue)
{
	struct emac_instance *dev = netdev_priv(ndev);

	DBG(dev, "tx_timeout" NL);

	schedule_work(&dev->reset_work);
}


#ifdef STATS
struct emac_instance *instances[4];
static void emac_do_stats_instance(struct emac_instance *mac) {
	if (!mac->stats_last_jiffies) mac->stats_last_jiffies = jiffies;
	if ((long)(mac->stats_last_jiffies + (HZ) - jiffies) > 0) return;
//	int i;
	mac->stats_last_jiffies = jiffies;


//	if (mac->ndev->name[3] == '0' || mac->ndev->name[3] == '1') {

	printk("%s %lu: int %u, poll %u, rx %u, tx %u:%u(%u), tx stops %u, rst %u\n",
	       mac->ndev->name,
	       mac->stats_last_jiffies,
	       mac->stats_intr, mac->stats_poll, mac->stats_rx,
	       mac->stats_xmit, mac->stats_tx, mac->stats_tx_desc,
	       mac->stats_tx_queue_stop, mac->stats_restart);

	mac->stats_intr = 0;
	mac->stats_poll = 0;
	mac->stats_rx = 0;
	mac->stats_xmit = 0;
	mac->stats_tx = 0;
	mac->stats_tx_desc = 0;
	mac->stats_tx_queue_stop = 0;

/*
	if (!strcmp(mac->ndev->name, "eth4")) {
	    emac_dump_descriptors(mac, 1);
	}
*/
#ifdef SWITCH_STATS
	if (mac->sw.ops) {
	    switch_print_stats(&mac->sw);
	}
#endif
}
static void emac_do_stats(void) {
	unsigned i;
	for (i = 0; i < 4; ++i) {
		if (!instances[i]) continue;
		emac_do_stats_instance(instances[i]);
	}
}
#endif

/* Tx lock BH */
static void __emac_set_multicast_list(struct emac_instance *dev)
{
	struct emac_regs __iomem *p = dev->emacp;
	u32 rmr = emac_iff2rmr(dev->ndev);
	return;

	DBG(dev, "__multicast %08x" NL, rmr);

	/* I decided to relax register access rules here to avoid
	 * full EMAC reset.
	 *
	 * There is a real problem with EMAC4 core if we use MWSW_001 bit
	 * in MR1 register and do a full EMAC reset.
	 * One TX BD status update is delayed and, after EMAC reset, it
	 * never happens, resulting in TX hung (it'll be recovered by TX
	 * timeout handler eventually, but this is just gross).
	 * So we either have to do full TX reset or try to cheat here :)
	 *
	 * The only required change is to RX mode register, so I *think* all
	 * we need is just to stop RX channel. This seems to work on all
	 * tested SoCs.                                                --ebs
	 *
	 * If we need the full reset, we might just trigger the workqueue
	 * and do it async... a bit nasty but should work --BenH
	 */
	dev->mcast_pending = 0;
	emac_rx_disable(dev);
	if (rmr & EMAC_RMR_MAE)
		emac_hash_mc(dev);
	out_be32(&p->rmr, rmr);
	emac_rx_enable(dev);
}

/* Tx lock BH */
static void emac_set_multicast_list(struct net_device *ndev)
{
	struct emac_instance *dev = netdev_priv(ndev);

	DBG(dev, "multicast" NL);

	BUG_ON(!netif_running(dev->ndev));

	if (dev->no_mcast) {
		dev->mcast_pending = 1;
		return;
	}
	__emac_set_multicast_list(dev);
}

static int emac_change_mtu(struct net_device *dev, int new_mtu) {
    if (new_mtu < 68 || new_mtu > dev->l2mtu) return -EINVAL;
    dev->mtu = new_mtu;
    return 0;
}


/* Size of RX skb for the given MTU */
static inline void emac_rx_skb_size(struct emac_instance *dev) {
	dev->rx_skb_size = emac_rx_size(dev->ndev->l2mtu);

	if (!dev->sw.ops) {
		// reserve 2 bytes to get proper ip header alignment
		dev->rx_skb_size += 2;
	}

}

/* Process ctx, rtnl_lock semaphore */
static int emac_change_l2mtu(struct net_device *ndev, int new_mtu)
{
	struct emac_instance *dev = netdev_priv(ndev);

	if (new_mtu < EMAC_MIN_MTU || new_mtu > dev->max_mtu)
		return -EINVAL;
	if (new_mtu == ndev->l2mtu) {
		return 0;
	}

	ndev->l2mtu = new_mtu;
	emac_rx_skb_size(dev);
	if (netif_running(ndev)) {
		emac_close(ndev);
		emac_open(ndev);
	}

	return 0;
}

static void emac_clean_tx_ring(struct emac_instance *dev)
{
	int i;

	for (i = 0; i < NUM_TX_BUFF; ++i) {
		if (dev->tx_skb[i]) {
			dev_kfree_skb(dev->tx_skb[i]);
			dev->tx_skb[i] = NULL;
			if (dev->tx_desc[i].ctrl & MAL_TX_CTRL_READY)
				++dev->stats.tx_dropped;
		}
		dev->tx_desc[i].ctrl = 0;
		dev->tx_desc[i].data_ptr = 0;
	}
}

static void emac_clean_rx_ring(struct emac_instance *dev)
{
	int i;

	for (i = 0; i < NUM_RX_BUFF; ++i)
		if (dev->rx_skb[i]) {
			dev->rx_desc[i].ctrl = 0;
			dev_kfree_skb(dev->rx_skb[i]);
			dev->rx_skb[i] = NULL;
			dev->rx_desc[i].data_ptr = 0;
		}

	if (dev->rx_sg_skb) {
		dev_kfree_skb(dev->rx_sg_skb);
		dev->rx_sg_skb = NULL;
	}
}

static inline int emac_alloc_rx_skb(struct sk_buff *skb,
	struct emac_instance *dev, int slot, gfp_t flags)
{
	if (unlikely(!skb))
		return -ENOMEM;

	dev->rx_skb[slot] = skb;
	dev->rx_desc[slot].data_len = 0;

	if (!dev->sw.ops) {
	    skb_reserve(skb, 2);
	}
	dev->rx_desc[slot].data_ptr =
	    dma_map_single(&dev->ofdev->dev, skb->data, dev->rx_skb_size,
			   DMA_FROM_DEVICE);
	wmb();
	dev->rx_desc[slot].ctrl = MAL_RX_CTRL_EMPTY |
	    (slot == (NUM_RX_BUFF - 1) ? MAL_RX_CTRL_WRAP : 0);

	return 0;
}

static void emac_print_link_status(struct emac_instance *dev)
{
	if (netif_carrier_ok(dev->ndev))
		printk(KERN_INFO "%s: link is up, %d %s%s\n",
		       dev->ndev->name, dev->phy.speed,
		       dev->phy.duplex == DUPLEX_FULL ? "FDX" : "HDX",
		       dev->phy.pause ? ", pause enabled" :
		       dev->phy.asym_pause ? ", asymmetric pause enabled" : "");
	else
		printk(KERN_INFO "%s: link is down\n", dev->ndev->name);
}

static void phy_ath_mmd_write(struct mii_if_info *mii,
	unsigned mmd, unsigned reg, unsigned val) {
    unsigned phy = mii->phy_id;
    mii->mdio_write(mii->dev, phy, 0xd, mmd);
    mii->mdio_write(mii->dev, phy, 0xe, reg);
    mii->mdio_write(mii->dev, phy, 0xd, 0x4000 | mmd);
    mii->mdio_read(mii->dev, phy, 0xe);
    mii->mdio_write(mii->dev, phy, 0xe, val);
}

static void emac_phy_reset(struct mii_if_info *mii) {
	unsigned phy = mii->phy_id;
	unsigned phy_id;
	unsigned i = 0;
	mii->mdio_write(mii->dev, phy, MII_BMCR, BMCR_RESET);
	while (1) {
		if (mii->mdio_read(mii->dev, phy, MII_BMCR) & BMCR_RESET) {
			++i;
			mdelay(1);
		}
		else {
			break;
		}
	}
	mii->mdio_write(mii->dev, mii->phy_id, MII_BMCR,
			BMCR_ANENABLE | BMCR_ANRESTART);

	phy_id = (mii->mdio_read(mii->dev, phy, 2) << 16) |
		mii->mdio_read(mii->dev, phy, 3);

	// setup magic in ar8021/ar8035 phy
	if (phy_id == 0x004DD04E || phy_id == 0x004DD071 || phy_id == 0x004DD072) {
		printk("phy fix %d\n", phy);
		mii->mdio_write(mii->dev, phy, 0x1d, 0);
		mii->mdio_write(mii->dev, phy, 0x1e,
				mii->mdio_read(mii->dev, phy, 0x1e) | 0x100);
		mii->mdio_write(mii->dev, phy, 0x1d, 5);
		mii->mdio_write(mii->dev, phy, 0x1e,
				mii->mdio_read(mii->dev, phy, 0x1e) | 0x100);

		// leds
		mii->mdio_write(mii->dev, phy, 0x18, 0x5200);
	}
	if (phy_id == 0x004DD072 || phy_id == 0x004DD074) {
		// disable SmartEEE
		phy_ath_mmd_write(mii, 3, 0x805d, 0x1000);
		phy_ath_mmd_write(mii, 7, 0x3c, 0);
		printk("SmartEEE disabled\n");
	}
}

/* Process ctx, rtnl_lock semaphore */
static int emac_open(struct net_device *ndev)
{
	struct emac_instance *dev = netdev_priv(ndev);
	int err, i;
	struct list_head *l;
//	printk("%s: open\n", ndev->name);

	list_for_each(l, &dev->mal->list) {
		struct mal_commac *mc = list_entry(l, struct mal_commac, list);
		struct emac_instance *x = container_of(mc, struct emac_instance, commac);
		if (netif_running(x->ndev)) {
			continue;
		}
//		printk("%s: should reset phy\n", x->ndev->name);
		emac_phy_reset(&x->sw.mii_phy);
		emac_mdio_write(x->ndev, x->sw.mii_phy.phy_id, MII_BMCR, BMCR_PDOWN);
	}


	DBG(dev, "open" NL);
	skb_bin_request(ndev, dev->rx_skb_size);

	/* Setup error IRQ handler */
	err = request_irq(dev->emac_irq, emac_irq, 0, "EMAC", dev);
	if (err) {
		printk(KERN_ERR "%s: failed to request IRQ %d\n",
		       ndev->name, dev->emac_irq);
		return err;
	}

	/* Allocate RX ring */
	for (i = 0; i < NUM_RX_BUFF; ++i)
		if (emac_alloc_rx_skb(skb_bin_get(), dev, i, GFP_KERNEL)) {
			printk(KERN_ERR "%s: failed to allocate RX ring\n",
			       ndev->name);
			goto oom;
		}

	dev->tx_cnt = dev->tx_slot = dev->ack_slot = dev->rx_slot = 0;
	dev->tx_first_slot = -1;
	dev->tx_desc_per_pack = 0;
	clear_bit(MAL_COMMAC_RX_STOPPED, &dev->commac.flags);
	dev->rx_sg_skb = NULL;

	mutex_lock(&dev->link_lock);
	dev->opened = 1;

	if (!dev->sw.ops) {
		emac_phy_reset(&dev->sw.mii_phy);

		/* Start PHY polling now.
		 */
		if (dev->phy.address >= 0) {
			int link_poll_interval;
			if (dev->phy.def->ops->poll_link(&dev->phy)) {
				dev->phy.def->ops->read_link(&dev->phy);
				emac_rx_clk_default(dev);
				netif_carrier_on(dev->ndev);
				link_poll_interval = PHY_POLL_LINK_ON;
			} else {
				emac_rx_clk_tx(dev);
				netif_carrier_off(dev->ndev);
				link_poll_interval = PHY_POLL_LINK_OFF;
			}
			dev->link_polling = 1;
			wmb();
			schedule_delayed_work(&dev->link_work, link_poll_interval);
			emac_print_link_status(dev);
		} else
			netif_carrier_on(dev->ndev);
	}
	else {
		dev->phy.speed = SPEED_1000;
		dev->phy.duplex = DUPLEX_FULL;
		netif_carrier_on(dev->ndev);
	}

	/* Required for Pause packet support in EMAC */
	dev_mc_add_global(ndev, default_mcast_addr);

	emac_configure(dev);
	mal_poll_add(dev->mal, &dev->commac);
	mal_enable_tx_channel(dev->mal, dev->mal_tx_chan);
	mal_set_rcbs(dev->mal, dev->mal_rx_chan, emac_rx_size(ndev->l2mtu));
	mal_enable_rx_channel(dev->mal, dev->mal_rx_chan);
	emac_tx_enable(dev);
	emac_rx_enable(dev);
	if (dev->sw.ops) {
	    switch_start(&dev->sw);
	}
	emac_netif_start(dev);

	mutex_unlock(&dev->link_lock);


	return 0;
 oom:
	emac_clean_rx_ring(dev);
	free_irq(dev->emac_irq, dev);

	return -ENOMEM;
}

/* BHs disabled */
#if 0
static int emac_link_differs(struct emac_instance *dev)
{
	u32 r = in_be32(&dev->emacp->mr1);

	int duplex = r & EMAC_MR1_FDE ? DUPLEX_FULL : DUPLEX_HALF;
	int speed, pause, asym_pause;

	if (r & EMAC_MR1_MF_1000)
		speed = SPEED_1000;
	else if (r & EMAC_MR1_MF_100)
		speed = SPEED_100;
	else
		speed = SPEED_10;

	switch (r & (EMAC_MR1_EIFC | EMAC_MR1_APP)) {
	case (EMAC_MR1_EIFC | EMAC_MR1_APP):
		pause = 1;
		asym_pause = 0;
		break;
	case EMAC_MR1_APP:
		pause = 0;
		asym_pause = 1;
		break;
	default:
		pause = asym_pause = 0;
	}
	return speed != dev->phy.speed || duplex != dev->phy.duplex ||
	    pause != dev->phy.pause || asym_pause != dev->phy.asym_pause;
}
#endif

static void emac_link_timer(struct work_struct *work)
{
	struct emac_instance *dev =
		container_of(to_delayed_work(work),
			     struct emac_instance, link_work);
	int link_poll_interval;

#ifdef STATS
	emac_do_stats();
#endif
	mutex_lock(&dev->link_lock);
	DBG2(dev, "link timer" NL);

	if (!dev->opened)
		goto bail;

	if (dev->phy.def->ops->poll_link(&dev->phy)) {
		if (!netif_carrier_ok(dev->ndev)) {
			emac_rx_clk_default(dev);
			/* Get new link parameters */
			dev->phy.def->ops->read_link(&dev->phy);

			netif_carrier_on(dev->ndev);
			emac_netif_stop(dev);
			emac_full_tx_reset(dev);
			emac_netif_start(dev);
			emac_print_link_status(dev);
		}
		link_poll_interval = PHY_POLL_LINK_ON;
	} else {
		if (netif_carrier_ok(dev->ndev)) {
			emac_rx_clk_tx(dev);
			netif_carrier_off(dev->ndev);
			netif_tx_disable(dev->ndev);
			emac_reinitialize(dev);
			emac_print_link_status(dev);
		}
		link_poll_interval = PHY_POLL_LINK_OFF;
	}
	schedule_delayed_work(&dev->link_work, link_poll_interval);
 bail:
	mutex_unlock(&dev->link_lock);
}

static void emac_force_link_update(struct emac_instance *dev)
{
	netif_carrier_off(dev->ndev);
	smp_rmb();
	if (dev->link_polling) {
		cancel_delayed_work_sync(&dev->link_work);
		if (dev->link_polling)
			schedule_delayed_work(&dev->link_work,  PHY_POLL_LINK_OFF);
	}
}

/* Process ctx, rtnl_lock semaphore */
static int emac_close(struct net_device *ndev)
{
	struct emac_instance *dev = netdev_priv(ndev);
	if (!(ndev->flags & IFF_UP)) {
		WARN_ON(1);
		return 0;
	}

//	printk("%s: close\n", ndev->name);
	DBG(dev, "close" NL);

	if (dev->sw.ops) {
		switch_stop(&dev->sw);
	}

	if (dev->phy.address >= 0) {
		dev->link_polling = 0;
		cancel_delayed_work_sync(&dev->link_work);
	}
	mutex_lock(&dev->link_lock);
	emac_netif_stop(dev);
	dev->opened = 0;
	mutex_unlock(&dev->link_lock);

	emac_rx_disable(dev);
	emac_tx_disable(dev);
	mal_disable_rx_channel(dev->mal, dev->mal_rx_chan);
	mal_disable_tx_channel(dev->mal, dev->mal_tx_chan);
	mal_poll_del(dev->mal, &dev->commac);

	emac_clean_tx_ring(dev);
	emac_clean_rx_ring(dev);

	free_irq(dev->emac_irq, dev);

	netif_carrier_off(ndev);

	if (!dev->sw.ops) {
	    emac_mdio_write(ndev, dev->phy.address, MII_BMCR, BMCR_PDOWN);
	}
	skb_bin_release(ndev);

	return 0;
}

static inline u16 emac_tx_csum(struct emac_instance *dev,
			       struct sk_buff *skb)
{
	if (emac_has_feature(dev, EMAC_FTR_HAS_TAH) &&
		(skb->ip_summed == CHECKSUM_PARTIAL)) {
		++dev->stats.tx_packets_csum;
		return EMAC_TX_CTRL_TAH_CSUM;
	}
	return 0;
}

static inline int emac_xmit_finish(struct emac_instance *dev, int len)
{
	struct emac_regs __iomem *p = dev->emacp;
	struct net_device *ndev = dev->ndev;

	/* Send the packet out. If the if makes a significant perf
	 * difference, then we can store the TMR0 value in "dev"
	 * instead
	 */
	if (emac_has_feature(dev, EMAC_FTR_EMAC4))
		out_be32(&p->tmr0, EMAC4_TMR0_XMIT);
	else
		out_be32(&p->tmr0, EMAC_TMR0_XMIT);

	if (unlikely(++dev->tx_cnt >= NUM_TX_BUFF - 5)) {
		netif_stop_queue(ndev);
		DBG2(dev, "stopped TX queue" NL);
#ifdef STATS
		++dev->stats_tx_queue_stop;
#endif
	}

	netif_trans_update(ndev);
	++dev->stats.tx_packets;
	dev->stats.tx_bytes += len;

#ifdef STATS
	++dev->stats_xmit;
#endif
	return NETDEV_TX_OK;
}

/* Tx lock BH */
static int emac_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct emac_instance *dev = netdev_priv(ndev);
	unsigned int len = skb->len;
	int slot;

	u16 ctrl = EMAC_TX_CTRL_GFCS | EMAC_TX_CTRL_GP | MAL_TX_CTRL_READY |
	    MAL_TX_CTRL_LAST | emac_tx_csum(dev, skb);

	slot = dev->tx_slot++;
	if (dev->tx_slot == NUM_TX_BUFF) {
		dev->tx_slot = 0;
		ctrl |= MAL_TX_CTRL_WRAP;
	}

	DBG2(dev, "xmit(%u) %d" NL, len, slot);

	dev->tx_skb[slot] = skb;
	dev->tx_desc[slot].data_ptr = dma_map_single(&dev->ofdev->dev,
						     skb->data, len,
						     DMA_TO_DEVICE);
	dev->tx_desc[slot].data_len = (u16) len;
	wmb();
	dev->tx_desc[slot].ctrl = ctrl;

	return emac_xmit_finish(dev, len);
}

static int emac_xmit(struct net_device *ndev, void *buf, unsigned size,
		     struct sk_buff *skb, unsigned queue_num) {
	struct emac_instance *dev = netdev_priv(ndev);
	u16 ctrl = EMAC_TX_CTRL_GFCS | EMAC_TX_CTRL_GP;
	int slot = dev->tx_slot;

//	printk("xmit part %d %p\n", size, skb);
	if (++dev->tx_slot == NUM_TX_BUFF) {
		dev->tx_slot = 0;
		ctrl |= MAL_TX_CTRL_WRAP;
	}

	dev->tx_desc[slot].data_ptr = dma_map_single(
		&dev->ofdev->dev, buf, size, DMA_TO_DEVICE);
	dev->tx_desc[slot].data_len = (u16) size;

	dev->tx_skb[slot] = skb;
	ctrl |= MAL_TX_CTRL_LAST | MAL_TX_CTRL_READY;
	wmb();
	dev->tx_desc[slot].ctrl = ctrl;

	if (dev->tx_first_slot != -1) {
	    dev->tx_desc[dev->tx_first_slot].ctrl |= MAL_TX_CTRL_READY;
	    dev->tx_first_slot = -1;
	}

	emac_xmit_finish(dev, skb->len);
	return 1;
}

static inline int emac_xmit_split(struct emac_instance *dev, int slot,
				  u32 pd, int len, int last, u16 base_ctrl)
{
	while (1) {
		u16 ctrl = base_ctrl;
		int chunk = min(len, MAL_MAX_TX_SIZE);
		len -= chunk;

		slot = (slot + 1) % NUM_TX_BUFF;

		if (last && !len)
			ctrl |= MAL_TX_CTRL_LAST;
		if (slot == NUM_TX_BUFF - 1)
			ctrl |= MAL_TX_CTRL_WRAP;

		dev->tx_skb[slot] = NULL;
		dev->tx_desc[slot].data_ptr = pd;
		dev->tx_desc[slot].data_len = (u16) chunk;
		dev->tx_desc[slot].ctrl = ctrl;
		++dev->tx_cnt;

		if (!len)
			break;

		pd += chunk;
	}
	return slot;
}

/* Tx lock BH disabled (SG version for TAH equipped EMACs) */
static int emac_start_xmit_sg(struct sk_buff *skb, struct net_device *ndev)
{
	struct emac_instance *dev = netdev_priv(ndev);
	int nr_frags = skb_shinfo(skb)->nr_frags;
	int len = skb->len, chunk;
	int slot, i;
	u16 ctrl;
	u32 pd;

	/* This is common "fast" path */
	if (likely(!nr_frags && len <= MAL_MAX_TX_SIZE))
		return emac_start_xmit(skb, ndev);

	len -= skb->data_len;

	/* Note, this is only an *estimation*, we can still run out of empty
	 * slots because of the additional fragmentation into
	 * MAL_MAX_TX_SIZE-sized chunks
	 */
	if (unlikely(dev->tx_cnt + nr_frags + mal_tx_chunks(len) >= NUM_TX_BUFF - 5))
		goto stop_queue;

	ctrl = EMAC_TX_CTRL_GFCS | EMAC_TX_CTRL_GP | MAL_TX_CTRL_READY |
	    emac_tx_csum(dev, skb);
	slot = dev->tx_slot;

	/* skb data */
	dev->tx_skb[slot] = NULL;
	chunk = min(len, MAL_MAX_TX_SIZE);
	dev->tx_desc[slot].data_ptr = pd =
	    dma_map_single(&dev->ofdev->dev, skb->data, len, DMA_TO_DEVICE);
	dev->tx_desc[slot].data_len = (u16) chunk;
	len -= chunk;
	if (unlikely(len))
		slot = emac_xmit_split(dev, slot, pd + chunk, len, !nr_frags,
				       ctrl);
	/* skb fragments */
	for (i = 0; i < nr_frags; ++i) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		len = skb_frag_size(frag);

		if (unlikely(dev->tx_cnt + mal_tx_chunks(len) >= NUM_TX_BUFF - 5))
			goto undo_frame;

		pd = skb_frag_dma_map(&dev->ofdev->dev, frag, 0,
				      len, DMA_TO_DEVICE);

		slot = emac_xmit_split(dev, slot, pd, len, i == nr_frags - 1,
				       ctrl);
	}

	DBG2(dev, "xmit_sg(%u) %d - %d" NL, skb->len, dev->tx_slot, slot);

	/* Attach skb to the last slot so we don't release it too early */
	dev->tx_skb[slot] = skb;

	/* Send the packet out */
	if (dev->tx_slot == NUM_TX_BUFF - 1)
		ctrl |= MAL_TX_CTRL_WRAP;
	wmb();
	dev->tx_desc[dev->tx_slot].ctrl = ctrl;
	dev->tx_slot = (slot + 1) % NUM_TX_BUFF;

	return emac_xmit_finish(dev, skb->len);

 undo_frame:
	/* Well, too bad. Our previous estimation was overly optimistic.
	 * Undo everything.
	 */
	while (slot != dev->tx_slot) {
		dev->tx_desc[slot].ctrl = 0;
		--dev->tx_cnt;
		if (--slot < 0)
			slot = NUM_TX_BUFF - 1;
	}
	++dev->stats.tx_undo;

 stop_queue:
#ifdef STATS
	++dev->stats_tx_queue_stop;
#endif
	netif_stop_queue(ndev);
	DBG2(dev, "stopped TX queue" NL);
	return NETDEV_TX_BUSY;
}

/* Tx lock BHs */
static void emac_parse_tx_error(struct emac_instance *dev, u16 ctrl)
{
	struct emac_stats *st = &dev->stats;

	DBG(dev, "BD TX error %04x" NL, ctrl);

	++st->tx_bd_errors;
	if (ctrl & EMAC_TX_ST_BFCS)
		++st->tx_bd_bad_fcs;
	if (ctrl & EMAC_TX_ST_LCS)
		++st->tx_bd_carrier_loss;
	if (ctrl & EMAC_TX_ST_ED)
		++st->tx_bd_excessive_deferral;
	if (ctrl & EMAC_TX_ST_EC)
		++st->tx_bd_excessive_collisions;
	if (ctrl & EMAC_TX_ST_LC)
		++st->tx_bd_late_collision;
	if (ctrl & EMAC_TX_ST_MC)
		++st->tx_bd_multple_collisions;
	if (ctrl & EMAC_TX_ST_SC)
		++st->tx_bd_single_collision;
	if (ctrl & EMAC_TX_ST_UR)
		++st->tx_bd_underrun;
	if (ctrl & EMAC_TX_ST_SQE)
		++st->tx_bd_sqe;
}

static inline void emac_recycle(struct emac_instance *dev,
				struct sk_buff *skb) {
	__skb_bin_recycle(skb);
}

static void emac_poll_tx(void *param)
{
	struct emac_instance *dev = param;
	u32 bad_mask;

#ifdef STATS
	++dev->stats_poll;
	emac_do_stats();
#endif
	DBG2(dev, "poll_tx, %d %d" NL, dev->tx_cnt, dev->ack_slot);

	if (emac_has_feature(dev, EMAC_FTR_HAS_TAH))
		bad_mask = EMAC_IS_BAD_TX_TAH;
	else
		bad_mask = EMAC_IS_BAD_TX;

	netif_tx_lock_bh(dev->ndev);
	if (dev->tx_cnt) {
		u16 ctrl;
		int slot = dev->ack_slot, n = 0;
	again:
		ctrl = dev->tx_desc[slot].ctrl;
		if (!(ctrl & MAL_TX_CTRL_READY)) {
			struct sk_buff *skb = dev->tx_skb[slot];
			++n;

#ifdef STATS
			++dev->stats_tx_desc;
#endif
			++dev->tx_desc_per_pack;
			if (skb) {
#ifdef STATS
				++dev->stats_tx;
#endif
				if (dev->sw.ops) {
					switch_tx(&dev->sw, 0, dev->tx_desc_per_pack);
				}
				dev->tx_desc_per_pack = 0;
				emac_recycle(dev, skb);
				dev->tx_skb[slot] = NULL;
			}
			slot = (slot + 1) % NUM_TX_BUFF;

			if (unlikely(ctrl & bad_mask))
				emac_parse_tx_error(dev, ctrl);

			if (--dev->tx_cnt)
				goto again;
		}
		if (n) {
			dev->ack_slot = slot;
			if (netif_queue_stopped(dev->ndev))
				netif_wake_queue(dev->ndev);

			DBG2(dev, "tx %d pkts" NL, n);
		}
	}
	netif_tx_unlock_bh(dev->ndev);
}

static inline void emac_recycle_rx_skb(struct emac_instance *dev, int slot,
				       int len)
{
	struct sk_buff *skb = dev->rx_skb[slot];

	DBG2(dev, "recycle %d %d" NL, slot, len);

	if (len)
		dma_map_single(&dev->ofdev->dev, skb->data - 2,
			       EMAC_DMA_ALIGN(len + 2), DMA_FROM_DEVICE);

	dev->rx_desc[slot].data_len = 0;
	wmb();
	dev->rx_desc[slot].ctrl = MAL_RX_CTRL_EMPTY |
	    (slot == (NUM_RX_BUFF - 1) ? MAL_RX_CTRL_WRAP : 0);
}

static void emac_parse_rx_error(struct emac_instance *dev, u16 ctrl)
{
	struct emac_stats *st = &dev->stats;

	DBG(dev, "BD RX error %04x" NL, ctrl);

	++st->rx_bd_errors;
	if (ctrl & EMAC_RX_ST_OE)
		++st->rx_bd_overrun;
	if (ctrl & EMAC_RX_ST_BP)
		++st->rx_bd_bad_packet;
	if (ctrl & EMAC_RX_ST_RP)
		++st->rx_bd_runt_packet;
	if (ctrl & EMAC_RX_ST_SE)
		++st->rx_bd_short_event;
	if (ctrl & EMAC_RX_ST_AE)
		++st->rx_bd_alignment_error;
	if (ctrl & EMAC_RX_ST_BFCS)
		++st->rx_bd_bad_fcs;
	if (ctrl & EMAC_RX_ST_PTL)
		++st->rx_bd_packet_too_long;
	if (ctrl & EMAC_RX_ST_ORE)
		++st->rx_bd_out_of_range;
	if (ctrl & EMAC_RX_ST_IRE)
		++st->rx_bd_in_range;
}

static inline void emac_rx_csum(struct emac_instance *dev,
				struct sk_buff *skb, u16 ctrl)
{
#ifdef CONFIG_IBM_NEW_EMAC_TAH
	if (!ctrl && dev->tah_dev) {
//		skb->ip_summed = CHECKSUM_UNNECESSARY;
//		++dev->stats.rx_packets_csum;
	}
#endif
}

static inline int emac_rx_sg_append(struct emac_instance *dev, int slot)
{
	if (likely(dev->rx_sg_skb != NULL)) {
		int len = dev->rx_desc[slot].data_len;
		int tot_len = dev->rx_sg_skb->len + len;

		if (unlikely(tot_len + 2 > dev->rx_skb_size)) {
			++dev->stats.rx_dropped_mtu;
			dev_kfree_skb(dev->rx_sg_skb);
			dev->rx_sg_skb = NULL;
		} else {
			memcpy(skb_tail_pointer(dev->rx_sg_skb),
					 dev->rx_skb[slot]->data, len);
			skb_put(dev->rx_sg_skb, len);
			emac_recycle_rx_skb(dev, slot, len);
			return 0;
		}
	}
	emac_recycle_rx_skb(dev, slot, 0);
	return -1;
}

/* NAPI poll context */
static int emac_poll_rx(void *param, int budget)
{
	struct emac_instance *dev = param;
	int slot = dev->rx_slot, received = 0;

	DBG2(dev, "poll_rx(%d)" NL, budget);

 again:
	while (budget > 0) {
		int len;
		struct sk_buff *skb;
		u16 ctrl = dev->rx_desc[slot].ctrl;

		if (ctrl & MAL_RX_CTRL_EMPTY)
			break;

		skb = dev->rx_skb[slot];
		mb();
		len = dev->rx_desc[slot].data_len;

		if (unlikely(!MAL_IS_SINGLE_RX(ctrl))) {
			emac_parse_rx_error(dev, ctrl);
			++dev->stats.rx_dropped_error;
			emac_recycle_rx_skb(dev, slot, 0);
			len = 0;
			goto next;
//			goto sg;
		}

		ctrl &= EMAC_BAD_RX_MASK;
		if (unlikely(ctrl && ctrl != EMAC_RX_TAH_BAD_CSUM)) {
			emac_parse_rx_error(dev, ctrl);
			++dev->stats.rx_dropped_error;
			emac_recycle_rx_skb(dev, slot, 0);
			len = 0;
			goto next;
		}

		if (len < ETH_HLEN) {
			++dev->stats.rx_dropped_stack;
			emac_recycle_rx_skb(dev, slot, len);
			goto next;
		}
		if (len > dev->rx_skb_size) {
			++dev->stats.rx_dropped_stack;
			printk("%s: rx inappropriate size skb:%p %d data:%p tail:%p l2mtu:%d rx_skb_size:%d\n",
			       dev->ndev->name, skb, len, skb->data,
			       skb_tail_pointer(skb),
			       dev->ndev->l2mtu, dev->rx_skb_size);
			BUG();
			emac_recycle_rx_skb(dev, slot, len);
			goto next;
		}

		if (unlikely(emac_alloc_rx_skb(__skb_bin_get(), dev, slot, GFP_ATOMIC)))
			goto oom;

		skb_put(skb, len);
	push_packet:
		if (dev->sw.ops) {
			switch_rx(&dev->sw, skb);
		}
		else {
			skb->protocol = eth_type_trans(skb, dev->ndev);
			emac_rx_csum(dev, skb, ctrl);

			if (unlikely(netif_receive_skb(skb) == NET_RX_DROP))
				++dev->stats.rx_dropped_stack;
		}

	next:
#ifdef STATS
		++dev->stats_rx;
#endif
		++dev->stats.rx_packets;
	skip:
		dev->stats.rx_bytes += len;
		slot = (slot + 1) % NUM_RX_BUFF;
		--budget;
		++received;
		continue;
	sg:
		if (ctrl & MAL_RX_CTRL_FIRST) {
			BUG_ON(dev->rx_sg_skb);
			if (unlikely(emac_alloc_rx_skb(__skb_bin_get(), dev, slot, GFP_ATOMIC))) {
				DBG(dev, "rx OOM %d" NL, slot);
				++dev->stats.rx_dropped_oom;
				emac_recycle_rx_skb(dev, slot, 0);
			} else {
				dev->rx_sg_skb = skb;
				skb_put(skb, len);
			}
		} else if (!emac_rx_sg_append(dev, slot) &&
			   (ctrl & MAL_RX_CTRL_LAST)) {

			skb = dev->rx_sg_skb;
			dev->rx_sg_skb = NULL;

			ctrl &= EMAC_BAD_RX_MASK;
			if (unlikely(ctrl && ctrl != EMAC_RX_TAH_BAD_CSUM)) {
				emac_parse_rx_error(dev, ctrl);
				++dev->stats.rx_dropped_error;
				dev_kfree_skb(skb);
				len = 0;
			} else
				goto push_packet;
		}
		goto skip;
	oom:
		DBG(dev, "rx OOM %d" NL, slot);
		/* Drop the packet and recycle skb */
		++dev->stats.rx_dropped_oom;
		emac_recycle_rx_skb(dev, slot, 0);
		goto next;
	}

	if (received) {
		DBG2(dev, "rx %d BDs" NL, received);
		dev->rx_slot = slot;
	}

	if (unlikely(budget && test_bit(MAL_COMMAC_RX_STOPPED, &dev->commac.flags))) {
		mb();
		if (!(dev->rx_desc[slot].ctrl & MAL_RX_CTRL_EMPTY)) {
			DBG2(dev, "rx restart" NL);
			received = 0;
			goto again;
		}

		if (dev->rx_sg_skb) {
			DBG2(dev, "dropping partial rx packet" NL);
			++dev->stats.rx_dropped_error;
			dev_kfree_skb(dev->rx_sg_skb);
			dev->rx_sg_skb = NULL;
		}

		clear_bit(MAL_COMMAC_RX_STOPPED, &dev->commac.flags);
		mal_enable_rx_channel(dev->mal, dev->mal_rx_chan);
		emac_rx_enable(dev);
		dev->rx_slot = 0;
	}
	return received;
}

/* NAPI poll context */
static int emac_peek_rx(void *param)
{
	struct emac_instance *dev = param;

	if (dev->tx_cnt &&
	    !(dev->tx_desc[dev->ack_slot].ctrl & MAL_TX_CTRL_READY)) {
		return 1;
	}

	return !(dev->rx_desc[dev->rx_slot].ctrl & MAL_RX_CTRL_EMPTY);
}

/* Hard IRQ */
static void emac_rxde(void *param)
{
	struct emac_instance *dev = param;

	++dev->stats.rx_stopped;
	emac_rx_disable_async(dev);
}

/* Hard IRQ */
static irqreturn_t emac_irq(int irq, void *dev_instance)
{
	struct emac_instance *dev = dev_instance;
	struct emac_regs __iomem *p = dev->emacp;
	struct emac_stats *st = &dev->stats;
	u32 isr;

	spin_lock(&dev->lock);

	isr = in_be32(&p->isr);
	out_be32(&p->isr, isr);

	DBG(dev, "isr = %08x" NL, isr);

	if (isr & EMAC4_ISR_TXPE)
		++st->tx_parity;
	if (isr & EMAC4_ISR_RXPE)
		++st->rx_parity;
	if (isr & EMAC4_ISR_TXUE)
		++st->tx_underrun;
	if (isr & EMAC4_ISR_RXOE)
		++st->rx_fifo_overrun;
	if (isr & EMAC_ISR_OVR)
		++st->rx_overrun;
	if (isr & EMAC_ISR_BP)
		++st->rx_bad_packet;
	if (isr & EMAC_ISR_RP)
		++st->rx_runt_packet;
	if (isr & EMAC_ISR_SE)
		++st->rx_short_event;
	if (isr & EMAC_ISR_ALE)
		++st->rx_alignment_error;
	if (isr & EMAC_ISR_BFCS)
		++st->rx_bad_fcs;
	if (isr & EMAC_ISR_PTLE)
		++st->rx_packet_too_long;
	if (isr & EMAC_ISR_ORE)
		++st->rx_out_of_range;
	if (isr & EMAC_ISR_IRE)
		++st->rx_in_range;
	if (isr & EMAC_ISR_SQE)
		++st->tx_sqe;
	if (isr & EMAC_ISR_TE)
		++st->tx_errors;

	spin_unlock(&dev->lock);

	return IRQ_HANDLED;
}

static struct net_device_stats *emac_stats(struct net_device *ndev)
{
	struct emac_instance *dev = netdev_priv(ndev);
	struct emac_stats *st = &dev->stats;
	struct net_device_stats *nst = &ndev->stats;
	unsigned long flags;

	DBG2(dev, "stats" NL);

	/* Compute "legacy" statistics */
	spin_lock_irqsave(&dev->lock, flags);
	nst->rx_packets = (unsigned long)st->rx_packets;
	nst->rx_bytes = (unsigned long)st->rx_bytes;
	nst->tx_packets = (unsigned long)st->tx_packets;
	nst->tx_bytes = (unsigned long)st->tx_bytes;
	nst->rx_dropped = (unsigned long)(st->rx_dropped_oom +
					  st->rx_dropped_error +
					  st->rx_dropped_resize +
					  st->rx_dropped_mtu);
	nst->tx_dropped = (unsigned long)st->tx_dropped;

	nst->rx_errors = (unsigned long)st->rx_bd_errors;
	nst->rx_fifo_errors = (unsigned long)(st->rx_bd_overrun +
					      st->rx_fifo_overrun +
					      st->rx_overrun);
	nst->rx_frame_errors = (unsigned long)(st->rx_bd_alignment_error +
					       st->rx_alignment_error);
	nst->rx_crc_errors = (unsigned long)(st->rx_bd_bad_fcs +
					     st->rx_bad_fcs);
	nst->rx_length_errors = (unsigned long)(st->rx_bd_runt_packet +
						st->rx_bd_short_event +
						st->rx_bd_packet_too_long +
						st->rx_bd_out_of_range +
						st->rx_bd_in_range +
						st->rx_runt_packet +
						st->rx_short_event +
						st->rx_packet_too_long +
						st->rx_out_of_range +
						st->rx_in_range);

	nst->tx_errors = (unsigned long)(st->tx_bd_errors + st->tx_errors);
	nst->tx_fifo_errors = (unsigned long)(st->tx_bd_underrun +
					      st->tx_underrun);
	nst->tx_carrier_errors = (unsigned long)st->tx_bd_carrier_loss;
	nst->collisions = (unsigned long)(st->tx_bd_excessive_deferral +
					  st->tx_bd_excessive_collisions +
					  st->tx_bd_late_collision +
					  st->tx_bd_multple_collisions);
	spin_unlock_irqrestore(&dev->lock, flags);
	return nst;
}

static struct mal_commac_ops emac_commac_ops = {
	.poll_tx = &emac_poll_tx,
	.poll_rx = &emac_poll_rx,
	.peek_rx = &emac_peek_rx,
	.rxde = &emac_rxde,
};

/* Ethtool support */
static int emac_ethtool_get_settings(struct net_device *ndev,
				     struct ethtool_link_ksettings *cmd)
{
	struct emac_instance *dev = netdev_priv(ndev);
	u32 supported, advertising;

	supported = dev->phy.features;
	cmd->base.port = PORT_MII;
	cmd->base.phy_address = dev->phy.address;

	mutex_lock(&dev->link_lock);
	advertising = dev->phy.advertising;
	cmd->base.autoneg = dev->phy.autoneg;
	cmd->base.speed = dev->phy.speed;
	cmd->base.duplex = dev->phy.duplex;
	mutex_unlock(&dev->link_lock);

	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
						supported);
	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising,
						advertising);
	
	return 0;
}

static int emac_ethtool_set_settings(struct net_device *ndev,
				     const struct ethtool_link_ksettings *cmd)
{
	struct emac_instance *dev = netdev_priv(ndev);
	u32 f = dev->phy.features;
	u32 advertising;

	ethtool_convert_link_mode_to_legacy_u32(&advertising,
						cmd->link_modes.advertising);
	
	DBG(dev, "set_settings(%d, %d, %d, 0x%08x)" NL,
	    cmd->autoneg, cmd->speed, cmd->duplex, advertising);

	/* Basic sanity checks */
	if (dev->phy.address < 0)
		return -EOPNOTSUPP;
	if (cmd->base.autoneg != AUTONEG_ENABLE && cmd->base.autoneg != AUTONEG_DISABLE)
		return -EINVAL;
	if (cmd->base.autoneg == AUTONEG_ENABLE && advertising == 0)
		return -EINVAL;
	if (cmd->base.duplex != DUPLEX_HALF && cmd->base.duplex != DUPLEX_FULL)
		return -EINVAL;

	if (cmd->base.autoneg == AUTONEG_DISABLE) {
		switch (cmd->base.speed) {
		case SPEED_10:
			if (cmd->base.duplex == DUPLEX_HALF &&
			    !(f & SUPPORTED_10baseT_Half))
				return -EINVAL;
			if (cmd->base.duplex == DUPLEX_FULL &&
			    !(f & SUPPORTED_10baseT_Full))
				return -EINVAL;
			break;
		case SPEED_100:
			if (cmd->base.duplex == DUPLEX_HALF &&
			    !(f & SUPPORTED_100baseT_Half))
				return -EINVAL;
			if (cmd->base.duplex == DUPLEX_FULL &&
			    !(f & SUPPORTED_100baseT_Full))
				return -EINVAL;
			break;
		case SPEED_1000:
			if (cmd->base.duplex == DUPLEX_HALF &&
			    !(f & SUPPORTED_1000baseT_Half))
				return -EINVAL;
			if (cmd->base.duplex == DUPLEX_FULL &&
			    !(f & SUPPORTED_1000baseT_Full))
				return -EINVAL;
			break;
		default:
			return -EINVAL;
		}

		mutex_lock(&dev->link_lock);
		dev->phy.def->ops->setup_forced(&dev->phy, cmd->base.speed,
						cmd->base.duplex);
		mutex_unlock(&dev->link_lock);

	} else {
		if (!(f & SUPPORTED_Autoneg))
			return -EINVAL;

		mutex_lock(&dev->link_lock);
		dev->phy.def->ops->setup_aneg(&dev->phy,
					      (advertising & f) |
					      (dev->phy.advertising &
					       (ADVERTISED_Pause |
						ADVERTISED_Asym_Pause)));
		mutex_unlock(&dev->link_lock);
	}
	emac_force_link_update(dev);

	return 0;
}

static void emac_ethtool_get_ringparam(struct net_device *ndev,
				       struct ethtool_ringparam *rp)
{
	rp->rx_max_pending = rp->rx_pending = NUM_RX_BUFF;
	rp->tx_max_pending = rp->tx_pending = NUM_TX_BUFF;
}

static void emac_ethtool_get_pauseparam(struct net_device *ndev,
					struct ethtool_pauseparam *pp)
{
	struct emac_instance *dev = netdev_priv(ndev);

	mutex_lock(&dev->link_lock);
	if ((dev->phy.features & SUPPORTED_Autoneg) &&
	    (dev->phy.advertising & (ADVERTISED_Pause | ADVERTISED_Asym_Pause)))
		pp->autoneg = 1;

	if (dev->phy.duplex == DUPLEX_FULL) {
		if (dev->phy.pause)
			pp->rx_pause = pp->tx_pause = 1;
		else if (dev->phy.asym_pause)
			pp->tx_pause = 1;
	}
	mutex_unlock(&dev->link_lock);
}

static int emac_get_regs_len(struct emac_instance *dev)
{
	if (emac_has_feature(dev, EMAC_FTR_EMAC4))
		return sizeof(struct emac_ethtool_regs_subhdr) +
			EMAC4_ETHTOOL_REGS_SIZE(dev);
	else
		return sizeof(struct emac_ethtool_regs_subhdr) +
			EMAC_ETHTOOL_REGS_SIZE(dev);
}

static int emac_ethtool_get_regs_len(struct net_device *ndev)
{
	struct emac_instance *dev = netdev_priv(ndev);
	int size;

	size = sizeof(struct emac_ethtool_regs_hdr) +
		emac_get_regs_len(dev) + mal_get_regs_len(dev->mal);
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII))
		size += zmii_get_regs_len(dev->zmii_dev);
	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII))
		size += rgmii_get_regs_len(dev->rgmii_dev);
	if (emac_has_feature(dev, EMAC_FTR_HAS_TAH))
		size += tah_get_regs_len(dev->tah_dev);

	return size;
}

static void *emac_dump_regs(struct emac_instance *dev, void *buf)
{
	struct emac_ethtool_regs_subhdr *hdr = buf;

	hdr->index = dev->cell_index;
	if (emac_has_feature(dev, EMAC_FTR_EMAC4)) {
		hdr->version = EMAC4_ETHTOOL_REGS_VER;
		memcpy_fromio(hdr + 1, dev->emacp, EMAC4_ETHTOOL_REGS_SIZE(dev));
		return ((void *)(hdr + 1) + EMAC4_ETHTOOL_REGS_SIZE(dev));
	} else {
		hdr->version = EMAC_ETHTOOL_REGS_VER;
		memcpy_fromio(hdr + 1, dev->emacp, EMAC_ETHTOOL_REGS_SIZE(dev));
		return ((void *)(hdr + 1) + EMAC_ETHTOOL_REGS_SIZE(dev));
	}
}

static void emac_ethtool_get_regs(struct net_device *ndev,
				  struct ethtool_regs *regs, void *buf)
{
	struct emac_instance *dev = netdev_priv(ndev);
	struct emac_ethtool_regs_hdr *hdr = buf;

	hdr->components = 0;
	buf = hdr + 1;

	buf = mal_dump_regs(dev->mal, buf);
	buf = emac_dump_regs(dev, buf);
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII)) {
		hdr->components |= EMAC_ETHTOOL_REGS_ZMII;
		buf = zmii_dump_regs(dev->zmii_dev, buf);
	}
	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII)) {
		hdr->components |= EMAC_ETHTOOL_REGS_RGMII;
		buf = rgmii_dump_regs(dev->rgmii_dev, buf);
	}
	if (emac_has_feature(dev, EMAC_FTR_HAS_TAH)) {
		hdr->components |= EMAC_ETHTOOL_REGS_TAH;
		buf = tah_dump_regs(dev->tah_dev, buf);
	}
}

static int emac_ethtool_nway_reset(struct net_device *ndev)
{
	struct emac_instance *dev = netdev_priv(ndev);
	int res = 0;

	DBG(dev, "nway_reset" NL);

	if (dev->phy.address < 0)
		return -EOPNOTSUPP;

	mutex_lock(&dev->link_lock);
	if (!dev->phy.autoneg) {
		res = -EINVAL;
		goto out;
	}

	dev->phy.def->ops->setup_aneg(&dev->phy, dev->phy.advertising);
 out:
	mutex_unlock(&dev->link_lock);
	emac_force_link_update(dev);
	return res;
}

static void emac_ethtool_get_ethtool_stats(struct net_device *ndev,
					   struct ethtool_stats *estats, u64 *stats) {
	struct emac_instance *dev = netdev_priv(ndev);
	eth_stats_copy_stats_long_long(
		&dev->xstats,
		eth_stat_items_ibm_newemac,
		(unsigned long long *)&dev->stats);
	switch_get_ethtool_stats(ndev, estats, stats);
}

static void emac_ethtool_self_test(struct net_device *ndev,
				   struct ethtool_test *test, u64 *data) {
    struct emac_instance *dev = netdev_priv(ndev);
    emac_dump_all(dev, test->flags);
    if (dev->sw.ops) {
	switch_self_test(&dev->sw, test, data);
    }
}

static void emac_ethtool_get_drvinfo(struct net_device *ndev,
				     struct ethtool_drvinfo *info)
{
	struct emac_instance *dev = netdev_priv(ndev);

	strcpy(info->driver, "ibm_emac");
	strcpy(info->version, DRV_VERSION);
	info->fw_version[0] = '\0';
	sprintf(info->bus_info, "PPC 4xx EMAC-%d", dev->cell_index);
	info->regdump_len = emac_ethtool_get_regs_len(ndev);
	ethtool_drvinfo_ext_fill(info, 0, dev->max_mtu);
}

static const struct ethtool_ops emac_ethtool_ops = {
	.get_link_ksettings = emac_ethtool_get_settings,
	.set_link_ksettings = emac_ethtool_set_settings,
	.get_drvinfo = emac_ethtool_get_drvinfo,

	.get_regs_len = emac_ethtool_get_regs_len,
	.get_regs = emac_ethtool_get_regs,

	.nway_reset = emac_ethtool_nway_reset,

	.get_ringparam = emac_ethtool_get_ringparam,
	.get_pauseparam = emac_ethtool_get_pauseparam,

	.get_strings = switch_get_strings,
	.get_sset_count = switch_get_sset_count,
	.get_ethtool_stats = emac_ethtool_get_ethtool_stats,
	.self_test = emac_ethtool_self_test,

	.get_link = ethtool_op_get_link,
};

static int emac_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	struct emac_instance *dev = netdev_priv(ndev);
	struct mii_ioctl_data *data = if_mii(rq);

	DBG(dev, "ioctl %08x" NL, cmd);

	if (!switch_ioctl(ndev, rq, cmd)) {
		return 0;
	}

	if (dev->phy.address < 0)
		return -EOPNOTSUPP;

	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = dev->phy.address;
		/* Fall through */
	case SIOCGMIIREG:
		data->val_out = emac_mdio_read(ndev, dev->phy.address,
					       data->reg_num);
		return 0;

	case SIOCSMIIREG:
		emac_mdio_write(ndev, dev->phy.address, data->reg_num,
				data->val_in);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

struct emac_depentry {
	u32			phandle;
	struct device_node	*node;
	struct platform_device	*ofdev;
	void			*drvdata;
};

#define	EMAC_DEP_MAL_IDX	0
#define	EMAC_DEP_ZMII_IDX	1
#define	EMAC_DEP_RGMII_IDX	2
#define	EMAC_DEP_TAH_IDX	3
#define	EMAC_DEP_MDIO_IDX	4
#define	EMAC_DEP_PREV_IDX	5
#define	EMAC_DEP_COUNT		6

static int emac_check_deps(struct emac_instance *dev,
			   struct emac_depentry *deps)
{
	int i, there = 0;
	struct device_node *np;

	for (i = 0; i < EMAC_DEP_COUNT; i++) {
		/* no dependency on that item, allright */
		if (deps[i].phandle == 0) {
			there++;
			continue;
		}
		/* special case for blist as the dependency might go away */
		if (i == EMAC_DEP_PREV_IDX) {
			np = *(dev->blist - 1);
			if (np == NULL) {
				deps[i].phandle = 0;
				there++;
				continue;
			}
			if (deps[i].node == NULL)
				deps[i].node = of_node_get(np);
		}
		if (deps[i].node == NULL)
			deps[i].node = of_find_node_by_phandle(deps[i].phandle);
		if (deps[i].node == NULL)
			continue;
		if (deps[i].ofdev == NULL)
			deps[i].ofdev = of_find_device_by_node(deps[i].node);
		if (deps[i].ofdev == NULL)
			continue;
		if (deps[i].drvdata == NULL)
			deps[i].drvdata = dev_get_drvdata(&deps[i].ofdev->dev);
		if (deps[i].drvdata != NULL)
			there++;
	}
	return (there == EMAC_DEP_COUNT);
}

static void emac_put_deps(struct emac_instance *dev)
{
	if (dev->mal_dev)
		of_dev_put(dev->mal_dev);
	if (dev->zmii_dev)
		of_dev_put(dev->zmii_dev);
	if (dev->rgmii_dev)
		of_dev_put(dev->rgmii_dev);
	if (dev->mdio_dev)
		of_dev_put(dev->mdio_dev);
	if (dev->tah_dev)
		of_dev_put(dev->tah_dev);
}

static int emac_of_bus_notify(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	/* We are only intereted in device addition */
	if (action == BUS_NOTIFY_BOUND_DRIVER)
		wake_up_all(&emac_probe_wait);
	return 0;
}

static struct notifier_block emac_of_bus_notifier = {
	.notifier_call = emac_of_bus_notify
};

static int emac_wait_deps(struct emac_instance *dev)
{
	struct emac_depentry deps[EMAC_DEP_COUNT];
	int i, err;

	memset(&deps, 0, sizeof(deps));

	deps[EMAC_DEP_MAL_IDX].phandle = dev->mal_ph;
	deps[EMAC_DEP_ZMII_IDX].phandle = dev->zmii_ph;
	deps[EMAC_DEP_RGMII_IDX].phandle = dev->rgmii_ph;
	if (dev->tah_ph)
		deps[EMAC_DEP_TAH_IDX].phandle = dev->tah_ph;
	if (dev->mdio_ph)
		deps[EMAC_DEP_MDIO_IDX].phandle = dev->mdio_ph;
	if (dev->blist && dev->blist > emac_boot_list)
		deps[EMAC_DEP_PREV_IDX].phandle = 0xffffffffu;
	bus_register_notifier(&platform_bus_type, &emac_of_bus_notifier);
	wait_event_timeout(emac_probe_wait,
			   emac_check_deps(dev, deps),
			   EMAC_PROBE_DEP_TIMEOUT);
	bus_unregister_notifier(&platform_bus_type, &emac_of_bus_notifier);
	err = emac_check_deps(dev, deps) ? 0 : -ENODEV;
	for (i = 0; i < EMAC_DEP_COUNT; i++) {
		if (deps[i].node)
			of_node_put(deps[i].node);
		if (err && deps[i].ofdev)
			of_dev_put(deps[i].ofdev);
	}
	if (err == 0) {
		dev->mal_dev = deps[EMAC_DEP_MAL_IDX].ofdev;
		dev->zmii_dev = deps[EMAC_DEP_ZMII_IDX].ofdev;
		dev->rgmii_dev = deps[EMAC_DEP_RGMII_IDX].ofdev;
		dev->tah_dev = deps[EMAC_DEP_TAH_IDX].ofdev;
		dev->mdio_dev = deps[EMAC_DEP_MDIO_IDX].ofdev;
	}
	if (deps[EMAC_DEP_PREV_IDX].ofdev)
		of_dev_put(deps[EMAC_DEP_PREV_IDX].ofdev);
	return err;
}

static int emac_read_uint_prop(struct device_node *np, const char *name,
			       u32 *val, int fatal)
{
	int len;
	const u32 *prop = of_get_property(np, name, &len);
	if (prop == NULL || len < sizeof(u32)) {
		if (fatal)
			printk(KERN_ERR "%s: missing %s property\n",
			       np->full_name, name);
		return -ENODEV;
	}
	*val = *prop;
	return 0;
}

static int emac_init_phy(struct emac_instance *dev)
{
	struct device_node *np = dev->ofdev->dev.of_node;
	struct net_device *ndev = dev->ndev;
	u32 phy_map, adv;
	int i;

	dev->phy.dev = ndev;
	dev->phy.mode = dev->phy_mode;

	/* PHY-less configuration.
	 * XXX I probably should move these settings to the dev tree
	 */
	if (dev->phy_address == 0xffffffff && dev->phy_map == 0xffffffff) {
		emac_reset(dev);

		/* PHY-less configuration.
		 * XXX I probably should move these settings to the dev tree
		 */
		dev->phy.address = -1;
		dev->phy.features = SUPPORTED_MII;
		if (emac_phy_supports_gige(dev->phy_mode))
			dev->phy.features |= SUPPORTED_1000baseT_Full;
		else
			dev->phy.features |= SUPPORTED_100baseT_Full;
		dev->phy.pause = 1;

		return 0;
	}

	mutex_lock(&emac_phy_map_lock);
	phy_map = dev->phy_map | busy_phy_map;

	DBG(dev, "PHY maps %08x %08x" NL, dev->phy_map, busy_phy_map);

	dev->phy.mdio_read = emac_mdio_read;
	dev->phy.mdio_write = emac_mdio_write;

	/* Enable internal clock source */
#ifdef CONFIG_PPC_DCR_NATIVE
	if (emac_has_feature(dev, EMAC_FTR_440GX_PHY_CLK_FIX))
		dcri_clrset(SDR0, SDR0_MFR, 0, SDR0_MFR_ECS);
#endif
	/* PHY clock workaround */
	emac_rx_clk_tx(dev);

	/* Enable internal clock source on 440GX*/
#ifdef CONFIG_PPC_DCR_NATIVE
	if (emac_has_feature(dev, EMAC_FTR_440GX_PHY_CLK_FIX))
		dcri_clrset(SDR0, SDR0_MFR, 0, SDR0_MFR_ECS);
#endif
	/* Configure EMAC with defaults so we can at least use MDIO
	 * This is needed mostly for 440GX
	 */
	if (emac_phy_gpcs(dev->phy.mode)) {
		/* XXX
		 * Make GPCS PHY address equal to EMAC index.
		 * We probably should take into account busy_phy_map
		 * and/or phy_map here.
		 *
		 * Note that the busy_phy_map is currently global
		 * while it should probably be per-ASIC...
		 */
		dev->phy.gpcs_address = dev->gpcs_address;
		if (dev->phy.gpcs_address == 0xffffffff)
			dev->phy.address = dev->cell_index;
	}

	emac_configure(dev);

	if (dev->phy_address != 0xffffffff)
		phy_map = ~(1 << dev->phy_address);

	for (i = 0; i < 0x20; phy_map >>= 1, ++i)
		if (!(phy_map & 1)) {
			int r;
			busy_phy_map |= 1 << i;

			/* Quick check if there is a PHY at the address */
			r = emac_mdio_read(dev->ndev, i, MII_BMCR);
			if (r == 0xffff || r < 0)
				continue;
			if (!emac_mii_phy_probe(&dev->phy, i))
				break;
		}

	/* Enable external clock source */
#ifdef CONFIG_PPC_DCR_NATIVE
	if (emac_has_feature(dev, EMAC_FTR_440GX_PHY_CLK_FIX))
		dcri_clrset(SDR0, SDR0_MFR, SDR0_MFR_ECS, 0);
#endif
	mutex_unlock(&emac_phy_map_lock);
	if (i == 0x20) {
		printk(KERN_WARNING "%s: can't find PHY!\n", np->full_name);
		return -ENXIO;
	}

	/* Init PHY */
	if (dev->phy.def->ops->init)
		dev->phy.def->ops->init(&dev->phy);

	/* Disable any PHY features not supported by the platform */
	dev->phy.def->features &= ~dev->phy_feat_exc;

	/* Setup initial link parameters */
	if (dev->phy.features & SUPPORTED_Autoneg) {
		adv = dev->phy.features;
		if (!emac_has_feature(dev, EMAC_FTR_NO_FLOW_CONTROL_40x))
			adv |= ADVERTISED_Pause | ADVERTISED_Asym_Pause;
		/* Restart autonegotiation */
		dev->phy.def->ops->setup_aneg(&dev->phy, adv);
	} else {
		u32 f = dev->phy.def->features;
		int speed = SPEED_10, fd = DUPLEX_HALF;

		/* Select highest supported speed/duplex */
		if (f & SUPPORTED_1000baseT_Full) {
			speed = SPEED_1000;
			fd = DUPLEX_FULL;
		} else if (f & SUPPORTED_1000baseT_Half)
			speed = SPEED_1000;
		else if (f & SUPPORTED_100baseT_Full) {
			speed = SPEED_100;
			fd = DUPLEX_FULL;
		} else if (f & SUPPORTED_100baseT_Half)
			speed = SPEED_100;
		else if (f & SUPPORTED_10baseT_Full)
			fd = DUPLEX_FULL;

		/* Force link parameters */
		dev->phy.def->ops->setup_forced(&dev->phy, speed, fd);
	}
	return 0;
}

static int emac_init_config(struct emac_instance *dev)
{
	struct device_node *np = dev->ofdev->dev.of_node;
	const void *p;
	int plen;
	const char *pm, *phy_modes[] = {
		[PHY_MODE_NA] = "",
		[PHY_MODE_MII] = "mii",
		[PHY_MODE_RMII] = "rmii",
		[PHY_MODE_SMII] = "smii",
		[PHY_MODE_RGMII] = "rgmii",
		[PHY_MODE_TBI] = "tbi",
		[PHY_MODE_GMII] = "gmii",
		[PHY_MODE_RTBI] = "rtbi",
		[PHY_MODE_SGMII] = "sgmii",
	};

	/* Read config from device-tree */
	if (emac_read_uint_prop(np, "mal-device", &dev->mal_ph, 1))
		return -ENXIO;
	if (emac_read_uint_prop(np, "mal-tx-channel", &dev->mal_tx_chan, 1))
		return -ENXIO;
	if (emac_read_uint_prop(np, "mal-rx-channel", &dev->mal_rx_chan, 1))
		return -ENXIO;
	if (emac_read_uint_prop(np, "cell-index", &dev->cell_index, 1))
		return -ENXIO;
	if (emac_read_uint_prop(np, "max-frame-size", &dev->max_mtu, 0))
		dev->max_mtu = 1500;
	if (dev->max_mtu > MAL_MAX_RX_SIZE) {
	    dev->max_mtu = MAL_MAX_RX_SIZE;
	}
	dev->max_mtu -= EMAC_MTU_OVERHEAD;
	if (emac_read_uint_prop(np, "rx-fifo-size", &dev->rx_fifo_size, 0))
		dev->rx_fifo_size = 2048;
	if (emac_read_uint_prop(np, "tx-fifo-size", &dev->tx_fifo_size, 0))
		dev->tx_fifo_size = 2048;
	if (emac_read_uint_prop(np, "rx-fifo-size-gige", &dev->rx_fifo_size_gige, 0))
		dev->rx_fifo_size_gige = dev->rx_fifo_size;
	if (emac_read_uint_prop(np, "tx-fifo-size-gige", &dev->tx_fifo_size_gige, 0))
		dev->tx_fifo_size_gige = dev->tx_fifo_size;
	if (emac_read_uint_prop(np, "phy-address", &dev->phy_address, 0))
		dev->phy_address = 0xffffffff;
	if (emac_read_uint_prop(np, "phy-map", &dev->phy_map, 0))
		dev->phy_map = 0xffffffff;
	if (emac_read_uint_prop(np, "gpcs-address", &dev->gpcs_address, 0))
		dev->gpcs_address = 0xffffffff;
	if (emac_read_uint_prop(np->parent, "clock-frequency", &dev->opb_bus_freq, 1))
		return -ENXIO;
	if (emac_read_uint_prop(np, "tah-device", &dev->tah_ph, 0))
		dev->tah_ph = 0;
	if (emac_read_uint_prop(np, "tah-channel", &dev->tah_port, 0))
		dev->tah_port = 0;
	if (emac_read_uint_prop(np, "mdio-device", &dev->mdio_ph, 0))
		dev->mdio_ph = 0;
	if (emac_read_uint_prop(np, "zmii-device", &dev->zmii_ph, 0))
		dev->zmii_ph = 0;
	if (emac_read_uint_prop(np, "zmii-channel", &dev->zmii_port, 0))
		dev->zmii_port = 0xffffffff;
	if (emac_read_uint_prop(np, "rgmii-device", &dev->rgmii_ph, 0))
		dev->rgmii_ph = 0;
	if (emac_read_uint_prop(np, "rgmii-channel", &dev->rgmii_port, 0))
		dev->rgmii_port = 0xffffffff;
	if (emac_read_uint_prop(np, "fifo-entry-size", &dev->fifo_entry_size, 0))
		dev->fifo_entry_size = 16;
	if (emac_read_uint_prop(np, "mal-burst-size", &dev->mal_burst_size, 0))
		dev->mal_burst_size = 256;

	/* PHY mode needs some decoding */
	dev->phy_mode = PHY_MODE_NA;
	pm = of_get_property(np, "phy-mode", &plen);
	if (pm != NULL) {
		int i;
		for (i = 0; i < ARRAY_SIZE(phy_modes); i++)
			if (!strcasecmp(pm, phy_modes[i])) {
				dev->phy_mode = i;
				break;
			}
	}

	/* Backward compat with non-final DT */
	if (dev->phy_mode == PHY_MODE_NA && pm != NULL && plen == 4) {
		u32 nmode = *(const u32 *)pm;
		if (nmode > PHY_MODE_NA && nmode <= PHY_MODE_SGMII)
			dev->phy_mode = nmode;
	}

	/* Check EMAC version */
	if (of_device_is_compatible(np, "ibm,emac4sync")) {
		dev->features |= (EMAC_FTR_EMAC4 | EMAC_FTR_EMAC4SYNC);
		if (of_device_is_compatible(np, "ibm,emac-460ex") ||
		    of_device_is_compatible(np, "ibm,emac-460gt"))
			dev->features |= EMAC_FTR_460EX_PHY_CLK_FIX;
		if (of_device_is_compatible(np, "ibm,emac-405ex") ||
		    of_device_is_compatible(np, "ibm,emac-405exr"))
			dev->features |= EMAC_FTR_440EP_PHY_CLK_FIX;
	} else if (of_device_is_compatible(np, "ibm,emac4")) {
		dev->features |= EMAC_FTR_EMAC4;
		if (of_device_is_compatible(np, "ibm,emac-440gx"))
			dev->features |= EMAC_FTR_440GX_PHY_CLK_FIX;
	} else {
		if (of_device_is_compatible(np, "ibm,emac-440ep") ||
		    of_device_is_compatible(np, "ibm,emac-440gr"))
			dev->features |= EMAC_FTR_440EP_PHY_CLK_FIX;
		if (of_device_is_compatible(np, "ibm,emac-405ez")) {
#ifdef CONFIG_IBM_NEW_EMAC_NO_FLOW_CTRL
			dev->features |= EMAC_FTR_NO_FLOW_CONTROL_40x;
#else
			printk(KERN_ERR "%s: Flow control not disabled!\n",
					np->full_name);
			return -ENXIO;
#endif
		}

	}

	/* Fixup some feature bits based on the device tree */
	if (of_get_property(np, "has-inverted-stacr-oc", NULL))
		dev->features |= EMAC_FTR_STACR_OC_INVERT;
	if (of_get_property(np, "has-new-stacr-staopc", NULL))
		dev->features |= EMAC_FTR_HAS_NEW_STACR;

	/* CAB lacks the appropriate properties */
	if (of_device_is_compatible(np, "ibm,emac-axon"))
		dev->features |= EMAC_FTR_HAS_NEW_STACR |
			EMAC_FTR_STACR_OC_INVERT;

	/* Enable TAH/ZMII/RGMII features as found */
	if (dev->tah_ph != 0) {
#ifdef CONFIG_IBM_NEW_EMAC_TAH
		dev->features |= EMAC_FTR_HAS_TAH;
#else
		printk(KERN_ERR "%s: TAH support not enabled !\n",
		       np->full_name);
		return -ENXIO;
#endif
	}

	if (dev->zmii_ph != 0) {
#ifdef CONFIG_IBM_NEW_EMAC_ZMII
		dev->features |= EMAC_FTR_HAS_ZMII;
#else
		printk(KERN_ERR "%s: ZMII support not enabled !\n",
		       np->full_name);
		return -ENXIO;
#endif
	}

	if (dev->rgmii_ph != 0) {
#ifdef CONFIG_IBM_NEW_EMAC_RGMII
		dev->features |= EMAC_FTR_HAS_RGMII;
#else
		printk(KERN_ERR "%s: RGMII support not enabled !\n",
		       np->full_name);
		return -ENXIO;
#endif
	}

	/* Read MAC-address */
	p = of_get_property(np, "local-mac-address", NULL);
	if (p == NULL) {
		printk(KERN_ERR "%s: Can't find local-mac-address property\n",
		       np->full_name);
		return -ENXIO;
	}
	memcpy(dev->ndev->dev_addr, p, 6);

	/* IAHT and GAHT filter parameterization */
	if (emac_has_feature(dev, EMAC_FTR_EMAC4SYNC)) {
		dev->xaht_slots_shift = EMAC4SYNC_XAHT_SLOTS_SHIFT;
		dev->xaht_width_shift = EMAC4SYNC_XAHT_WIDTH_SHIFT;
	} else {
		dev->xaht_slots_shift = EMAC4_XAHT_SLOTS_SHIFT;
		dev->xaht_width_shift = EMAC4_XAHT_WIDTH_SHIFT;
	}

	DBG(dev, "features     : 0x%08x / 0x%08x\n", dev->features, EMAC_FTRS_POSSIBLE);
	DBG(dev, "tx_fifo_size : %d (%d gige)\n", dev->tx_fifo_size, dev->tx_fifo_size_gige);
	DBG(dev, "rx_fifo_size : %d (%d gige)\n", dev->rx_fifo_size, dev->rx_fifo_size_gige);
	DBG(dev, "max_mtu      : %d\n", dev->max_mtu);
	DBG(dev, "OPB freq     : %d\n", dev->opb_bus_freq);

	return 0;
}

static const struct net_device_ops emac_netdev_ops = {
	.ndo_open		= emac_open,
	.ndo_stop		= emac_close,
	.ndo_get_stats		= emac_stats,
	.ndo_set_rx_mode	= emac_set_multicast_list,
	.ndo_do_ioctl		= emac_ioctl,
	.ndo_tx_timeout		= emac_tx_timeout,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_start_xmit		= emac_start_xmit,
	.ndo_change_mtu		= emac_change_mtu,
	.ndo_change_l2mtu	= emac_change_l2mtu,
};

static const struct net_device_ops emac_gige_netdev_ops = {
	.ndo_open		= emac_open,
	.ndo_stop		= emac_close,
	.ndo_get_stats		= emac_stats,
	.ndo_set_rx_mode	= emac_set_multicast_list,
	.ndo_do_ioctl		= emac_ioctl,
	.ndo_tx_timeout		= emac_tx_timeout,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_start_xmit		= emac_start_xmit_sg,
	.ndo_change_mtu		= emac_change_mtu,
	.ndo_change_l2mtu	= emac_change_l2mtu,
};

static int emac_probe(struct platform_device *ofdev)
{
	struct net_device *ndev;
	struct emac_instance *dev;
	struct device_node *np = ofdev->dev.of_node;
	struct device_node **blist = NULL;
	int err, i;
	static unsigned emac_num = 0;
	++emac_num;

	/* Skip unused/unwired EMACS.  We leave the check for an unused
	 * property here for now, but new flat device trees should set a
	 * status property to "disabled" instead.
	 */
	if (of_get_property(np, "unused", NULL) || !of_device_is_available(np))
		return -ENODEV;

	/* Find ourselves in the bootlist if we are there */
	for (i = 0; i < EMAC_BOOT_LIST_SIZE; i++)
		if (emac_boot_list[i] == np)
			blist = &emac_boot_list[i];

	/* Allocate our net_device structure */
	err = -ENOMEM;
	if (emac_num == 4) {
		ndev = alloc_switchdev(0, sizeof(struct emac_instance), 1);
	}
	else {
		ndev = alloc_etherdev(sizeof(struct emac_instance));
	}
	if (!ndev) {
		printk(KERN_ERR "%s: could not allocate ethernet device!\n",
		       np->full_name);
		goto err_gone;
	}
	dev = netdev_priv(ndev);
#ifdef STATS
	instances[emac_num - 1] = dev;
#endif
	dev->ndev = ndev;
	dev->ofdev = ofdev;
	dev->blist = blist;
	SET_NETDEV_DEV(ndev, &ofdev->dev);

	/* Initialize some embedded data structures */
	spin_lock_init(&dev->mdio_lock);
	mutex_init(&dev->link_lock);
	spin_lock_init(&dev->lock);
	INIT_WORK(&dev->reset_work, emac_reset_work);

	/* Init various config data based on device-tree */
	err = emac_init_config(dev);
	if (err != 0)
		goto err_free;

	/* Get interrupts. EMAC irq is mandatory, WOL irq is optional */
	dev->emac_irq = irq_of_parse_and_map(np, 0);
	dev->wol_irq = irq_of_parse_and_map(np, 1);
	if (dev->emac_irq == NO_IRQ) {
		printk(KERN_ERR "%s: Can't map main interrupt\n", np->full_name);
		goto err_free;
	}
	ndev->irq = dev->emac_irq;

	/* Map EMAC regs */
	if (of_address_to_resource(np, 0, &dev->rsrc_regs)) {
		printk(KERN_ERR "%s: Can't get registers address\n",
		       np->full_name);
		goto err_irq_unmap;
	}
	// TODO : request_mem_region
	dev->emacp = ioremap(dev->rsrc_regs.start,
			     dev->rsrc_regs.end - dev->rsrc_regs.start + 1);
	if (dev->emacp == NULL) {
		printk(KERN_ERR "%s: Can't map device registers!\n",
		       np->full_name);
		err = -ENOMEM;
		goto err_irq_unmap;
	}

	/* Wait for dependent devices */
	err = emac_wait_deps(dev);
	if (err) {
		printk(KERN_ERR
		       "%s: Timeout waiting for dependent devices\n",
		       np->full_name);
		/*  display more info about what's missing ? */
		goto err_reg_unmap;
	}
	dev->mal = dev_get_drvdata(&dev->mal_dev->dev);
	if (dev->mdio_dev != NULL)
		dev->mdio_instance = dev_get_drvdata(&dev->mdio_dev->dev);
//	while (1) {
//		emac_mii_dump(dev);
//	}

	/* Register with MAL */
	dev->commac.ops = &emac_commac_ops;
	dev->commac.dev = dev;
	dev->commac.tx_chan_mask = MAL_CHAN_MASK(dev->mal_tx_chan);
	dev->commac.rx_chan_mask = MAL_CHAN_MASK(dev->mal_rx_chan);
	err = mal_register_commac(dev->mal, &dev->commac);
	if (err) {
		printk(KERN_ERR "%s: failed to register with mal %s!\n",
		       np->full_name, dev->mal_dev->dev.of_node->full_name);
		goto err_rel_deps;
	}
	ndev->l2mtu = 1600;
	emac_rx_skb_size(dev);

	/* Get pointers to BD rings */
	dev->tx_desc =
	    dev->mal->bd_virt + mal_tx_bd_offset(dev->mal, dev->mal_tx_chan);
	dev->rx_desc =
	    dev->mal->bd_virt + mal_rx_bd_offset(dev->mal, dev->mal_rx_chan);

	DBG(dev, "tx_desc %p" NL, dev->tx_desc);
	DBG(dev, "rx_desc %p" NL, dev->rx_desc);

	/* Clean rings */
	memset(dev->tx_desc, 0, NUM_TX_BUFF * sizeof(struct mal_descriptor));
	memset(dev->rx_desc, 0, NUM_RX_BUFF * sizeof(struct mal_descriptor));
	memset(dev->tx_skb, 0, NUM_TX_BUFF * sizeof(struct sk_buff *));
	memset(dev->rx_skb, 0, NUM_RX_BUFF * sizeof(struct sk_buff *));

	/* Attach to ZMII, if needed */
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII) &&
	    (err = zmii_attach(dev->zmii_dev, dev->zmii_port, &dev->phy_mode)) != 0)
		goto err_unreg_commac;

	/* Attach to RGMII, if needed */
	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII) &&
	    (err = rgmii_attach(dev->rgmii_dev, dev->rgmii_port, dev->phy_mode)) != 0)
		goto err_detach_zmii;

	/* Attach to TAH, if needed */
	if (emac_has_feature(dev, EMAC_FTR_HAS_TAH) &&
	    (err = tah_attach(dev->tah_dev, dev->tah_port)) != 0)
		goto err_detach_rgmii;

	/* Set some link defaults before we can find out real parameters */
	dev->phy.speed = SPEED_100;
	dev->phy.duplex = DUPLEX_FULL;
	dev->phy.autoneg = AUTONEG_DISABLE;
	dev->phy.pause = dev->phy.asym_pause = 0;
	dev->stop_timeout = STOP_TIMEOUT_100;
	INIT_DELAYED_WORK(&dev->link_work, emac_link_timer);

	/* Find PHY if any */
	err = emac_init_phy(dev);
	if (err != 0)
		goto err_detach_tah;

	ndev->watchdog_timeo = 5 * HZ;
	ndev->netdev_ops = &emac_netdev_ops;
/*
emac_gige_netdev_ops has emac_start_xmit_sg, which can possibly return NETDEV_TX_BUSY, which can screw up things when no-queue qdisc is in use.
RB1200 ethernets do not have this, but comment it out to future proof it.
	if (dev->tah_dev)
		ndev->features |= NETIF_F_IP_CSUM | NETIF_F_SG;
	if (emac_phy_supports_gige(dev->phy_mode)) {
		ndev->netdev_ops = &emac_gige_netdev_ops;
		dev->commac.ops = &emac_commac_sg_ops;
	}
*/
	ndev->ethtool_ops = &emac_ethtool_ops;

	netif_carrier_off(ndev);
	netif_stop_queue(ndev);

	eth_stats_calc_flags(&dev->xstats, eth_stat_items_ibm_newemac);

	rtnl_lock();
	err = register_netdevice(ndev);
	if (err) {
		printk(KERN_ERR "%s: failed to register net device (%d)!\n",
		       np->full_name, err);
		goto err_detach_tah;
	}


	dev->sw.dev = ndev;
	dev->sw.mii_phy.dev = ndev;
	dev->sw.mii_phy.mdio_read = emac_mdio_read;
	dev->sw.mii_phy.mdio_write = emac_mdio_write;
	dev->sw.mii_phy.phy_id = dev->phy.address;
	dev->sw.mii_phy.phy_id_mask = 0x1f;
	dev->sw.mii_phy.reg_num_mask = 0x1f;
	dev->sw.mii_phy.supports_gmii = 1;
	dev->sw.mii_switch.mdio_read = emac_mdio_read;
	dev->sw.mii_switch.mdio_write = emac_mdio_write;


	if (emac_num == 4) {
		int port_map[6] = { 0, 4, 3, 2, 1, -1 };
		unsigned char addr[ETH_ALEN];
		memcpy(addr, ndev->dev_addr, ETH_ALEN);

		// have switch
		dev->sw.type = SWITCH_ATHEROS8316;
		dev->sw.num = 0;
		dev->sw.tx_buffer_count = NUM_TX_BUFF;
		dev->sw.supported_l2mtu = dev->max_mtu;
		dev->sw.xmit = emac_xmit;

		err = switch_init(&dev->sw, port_map, addr);
		if (err) {
			printk("switch init failed %d\n", err);
			goto switch_fail;
		}
	}
	else {
		emac_mdio_write(ndev, dev->phy.address, MII_BMCR, BMCR_PDOWN);
	}


	/* Set our drvdata last as we don't want them visible until we are
	 * fully initialized
	 */
	wmb();
	dev_set_drvdata(&ofdev->dev, dev);

	/* There's a new kid in town ! Let's tell everybody */
	wake_up_all(&emac_probe_wait);


	printk(KERN_INFO "%s: EMAC-%d %s, MAC %pM\n",
	       ndev->name, dev->cell_index, np->full_name, ndev->dev_addr);

	if (dev->phy_mode == PHY_MODE_SGMII)
		printk(KERN_NOTICE "%s: in SGMII mode\n", ndev->name);

	if (dev->phy.address >= 0)
		printk("%s: found %s PHY (0x%02x)\n", ndev->name,
		       dev->phy.def->name, dev->phy.address);

	emac_dbg_register(dev);
	rtnl_unlock();

	/* Life is good */
	return 0;

	/* I have a bad feeling about this ... */

switch_fail:
	unregister_netdevice(ndev);
 err_detach_tah:
	if (emac_has_feature(dev, EMAC_FTR_HAS_TAH))
		tah_detach(dev->tah_dev, dev->tah_port);
	rtnl_unlock();
 err_detach_rgmii:
	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII))
		rgmii_detach(dev->rgmii_dev, dev->rgmii_port);
 err_detach_zmii:
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII))
		zmii_detach(dev->zmii_dev, dev->zmii_port);
 err_unreg_commac:
	mal_unregister_commac(dev->mal, &dev->commac);
 err_rel_deps:
	emac_put_deps(dev);
 err_reg_unmap:
	iounmap(dev->emacp);
 err_irq_unmap:
	if (dev->wol_irq != NO_IRQ)
		irq_dispose_mapping(dev->wol_irq);
	if (dev->emac_irq != NO_IRQ)
		irq_dispose_mapping(dev->emac_irq);
 err_free:
	free_netdev(ndev);
 err_gone:
	/* if we were on the bootlist, remove us as we won't show up and
	 * wake up all waiters to notify them in case they were waiting
	 * on us
	 */
	if (blist) {
		*blist = NULL;
		wake_up_all(&emac_probe_wait);
	}
	return err;
}

static int emac_remove(struct platform_device *ofdev)
{
	struct emac_instance *dev = dev_get_drvdata(&ofdev->dev);

	DBG(dev, "remove" NL);

	rtnl_lock();
	if (dev->sw.ops) {
		switch_cleanup(&dev->sw);
	}

	dev_set_drvdata(&ofdev->dev, NULL);

	unregister_netdevice(dev->ndev);
	rtnl_unlock();

	flush_scheduled_work();

	if (emac_has_feature(dev, EMAC_FTR_HAS_TAH))
		tah_detach(dev->tah_dev, dev->tah_port);
	if (emac_has_feature(dev, EMAC_FTR_HAS_RGMII))
		rgmii_detach(dev->rgmii_dev, dev->rgmii_port);
	if (emac_has_feature(dev, EMAC_FTR_HAS_ZMII))
		zmii_detach(dev->zmii_dev, dev->zmii_port);

	mal_unregister_commac(dev->mal, &dev->commac);
	emac_put_deps(dev);

	emac_dbg_unregister(dev);
	iounmap(dev->emacp);

	if (dev->wol_irq != NO_IRQ)
		irq_dispose_mapping(dev->wol_irq);
	if (dev->emac_irq != NO_IRQ)
		irq_dispose_mapping(dev->emac_irq);

	free_netdev(dev->ndev);

	return 0;
}

/* XXX Features in here should be replaced by properties... */
static struct of_device_id emac_match[] =
{
	{
		.type		= "network",
		.compatible	= "ibm,emac",
	},
	{
		.type		= "network",
		.compatible	= "ibm,emac4",
	},
	{
		.type		= "network",
		.compatible	= "ibm,emac4sync",
	},
	{},
};
MODULE_DEVICE_TABLE(of, emac_match);

static struct platform_driver emac_driver = {
	.driver = {
		.name = "emac",
		.owner = THIS_MODULE,
		.of_match_table = emac_match,
	},
	.probe = emac_probe,
	.remove = emac_remove,
};

static void emac_make_bootlist(void)
{
	struct device_node *np = NULL;
	int j, max, i = 0, k;
	int cell_indices[EMAC_BOOT_LIST_SIZE];

	/* Collect EMACs */
	while((np = of_find_all_nodes(np)) != NULL) {
		const u32 *idx;

		if (of_match_node(emac_match, np) == NULL)
			continue;
		if (of_get_property(np, "unused", NULL))
			continue;
		idx = of_get_property(np, "cell-index", NULL);
		if (idx == NULL)
			continue;
		cell_indices[i] = *idx;
		emac_boot_list[i++] = of_node_get(np);
		if (i >= EMAC_BOOT_LIST_SIZE) {
			of_node_put(np);
			break;
		}
	}
	max = i;

	/* Bubble sort them (doh, what a creative algorithm :-) */
	for (i = 0; max > 1 && (i < (max - 1)); i++)
		for (j = i; j < max; j++) {
			if (cell_indices[i] > cell_indices[j]) {
				np = emac_boot_list[i];
				emac_boot_list[i] = emac_boot_list[j];
				emac_boot_list[j] = np;
				k = cell_indices[i];
				cell_indices[i] = cell_indices[j];
				cell_indices[j] = k;
			}
		}
}

static __init int emac_init(void)
{
	int rc;

	printk(KERN_INFO DRV_DESC ", version " DRV_VERSION "\n");

	/* Init debug stuff */
	emac_init_debug();

	/* Build EMAC boot list */
	emac_make_bootlist();

	/* Init submodules */
	rc = mal_init();
	if (rc)
		goto err;
	rc = zmii_init();
	if (rc)
		goto err_mal;
	rc = rgmii_init();
	if (rc)
		goto err_zmii;
	rc = tah_init();
	if (rc)
		goto err_rgmii;
	rc = platform_driver_register(&emac_driver);
	if (rc)
		goto err_tah;

	return 0;

 err_tah:
	tah_exit();
 err_rgmii:
	rgmii_exit();
 err_zmii:
	zmii_exit();
 err_mal:
	mal_exit();
 err:
	return rc;
}

static void emac_exit(void)
{
	int i;

	platform_driver_unregister(&emac_driver);

	tah_exit();
	rgmii_exit();
	zmii_exit();
	mal_exit();
	emac_fini_debug();

	/* Destroy EMAC boot list */
	for (i = 0; i < EMAC_BOOT_LIST_SIZE; i++)
		if (emac_boot_list[i])
			of_node_put(emac_boot_list[i]);
}

module_init(emac_init);
module_exit(emac_exit);
