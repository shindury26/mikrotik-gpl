/*
 * drivers/net/gianfar.c
 *
 * Gianfar Ethernet Driver
 * This driver is designed for the non-CPM ethernet controllers
 * on the 85xx and 83xx family of integrated processors
 * Based on 8260_io/fcc_enet.c
 *
 * Author: Andy Fleming
 * Maintainer: Kumar Gala
 *
 * Copyright (c) 2002-2006 Freescale Semiconductor, Inc.
 * Copyright (c) 2007 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/in.h>
#include <linux/ethtool_extension.h>
#include <linux/fsl_devices.h>
#include <linux/switch.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/gpio.h>
#include <linux/skb_bin.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <asm/rb_aux.h>
#include <asm/time.h>

#include "gfar.h"
#ifdef STATS
#include "perf_core.h"
#include "perf_cpu.h"
#endif


static inline void prefetch_l1(const void *x) {
    __asm__ __volatile__ ("dcbt 0,%0" : : "r" (x));
}

static inline void prefetch_l1_write(const void *x) {
    __asm__ __volatile__ ("dcbtst 0,%0" : : "r" (x));
}

static inline unsigned read_reg(unsigned addr) {
    unsigned *regs = ioremap(0xe0000000 + addr, 4);
    unsigned ret;
    if (!regs) {
	return -1;
    }
    ret = in_be32(regs);
    iounmap(regs);
    return ret;
}

int tx_descriptor_count = -1;
module_param(tx_descriptor_count, int, 0644);
int rx_descriptor_count = -1;
module_param(rx_descriptor_count, int, 0644);
static int alternative_function = 0;
module_param_named(alt, alternative_function, int, 0644);

MODULE_LICENSE("GPL");

static struct gfar_mac *macs[3];


struct gfar_mii {
    u32	miimcfg;
    u32	miimcom;
    u32	miimadd;
    u32	miimcon;
    u32	miimstat;
    u32	miimind;
};

struct mdio_bus {
    unsigned refs;
    struct gfar_mii __iomem *regs;
    spinlock_t lock;
};
static struct mdio_bus mdio_bus = {
    .lock = __SPIN_LOCK_UNLOCKED(mdio_bus.lock),
};

static const char *get_board_model(void) {
    static const char *board_name = NULL;
    struct device_node *root;
    if (board_name) {
	return board_name;
    }
    root = of_find_node_by_path("/");
    if (root) {
	int size;
	board_name = (char *)of_get_property(root, "model", &size);
	of_node_put(root);
    }
    else {
	board_name = "";
    }
    return board_name;
}

static int is_rb1100(void) {
    static int ret = -1;
    if (ret != -1) {
	return ret;
    }
    ret = !strncmp(get_board_model(), "RB1100", 6);
    return ret;
}

static int is_rb1120(void) {
    static int ret = -1;
    if (ret != -1) {
	return ret;
    }
    ret = !strncmp(get_board_model(), "RB1120", 6);
    return ret;
}


static void gfar_select_mdio_bus_rb1100(int bus) {
    switch (bus) {
    case 0:
	change_latch(0x80, 0x00);    	
	break;
    case 1:
	change_latch(0x00, 0x80);
	break;
    default:
	BUG();
    }
}

static void gfar_select_mdio_bus_rb1120(int bus) {
    static unsigned switch_mdio_gpio7 = 0;
    static unsigned switch_mdio_gpio2 = 0;
    if (switch_mdio_gpio7 == 0) {
	switch_mdio_gpio7 = get_gpio_def("switch_mdio");
	switch_mdio_gpio2 = get_gpio_def("switch_mdio2");
//	printk("rb1120 mdio select gpios: %u %u\n",
//		switch_mdio_gpio7, switch_mdio_gpio2);
    }
    if (!switch_mdio_gpio7 || !switch_mdio_gpio2) return;
    switch (bus) {
    case 0:
	gpio_set_value(switch_mdio_gpio7, 1);
	gpio_set_value(switch_mdio_gpio2, 1);
	break;
    case 1:
	gpio_set_value(switch_mdio_gpio7, 1);
	gpio_set_value(switch_mdio_gpio2, 0);
	break;
    case 2:
	gpio_set_value(switch_mdio_gpio7, 0);
	break;
    default:
	break;
    }
}

static void gfar_select_mdio_bus(struct gfar_mac *mac) {
    if (is_rb1100()) {
	if (mac->sw.ops) {
	    gfar_select_mdio_bus_rb1100(mac->sw.num);
	}
    }
    if (is_rb1120()) {
	if (mac->sw.ops) {
	    gfar_select_mdio_bus_rb1120(mac->sw.num);
	}
	else {
	    gfar_select_mdio_bus_rb1120(2);
	}
    }
}

static int mii_read(struct gfar_mac *mac, int phy, int reg) {
    struct gfar_mii __iomem *regs = mdio_bus.regs;
    unsigned i;
    int ret;
    spin_lock_bh(&mdio_bus.lock);

    gfar_select_mdio_bus(mac);
    out_be32(&regs->miimadd, (phy << 8) | reg);
    out_be32(&regs->miimcom, 0);
    out_be32(&regs->miimcom, MII_READ_COMMAND);

    for (i = 0; i < 100000; ++i) {
	if (in_be32(&regs->miimind) & (MIIMIND_NOTVALID | MIIMIND_BUSY)) {
	}
	else {
	    break;
	}
    }

    ret = in_be32(&regs->miimstat);
    spin_unlock_bh(&mdio_bus.lock);
    return ret;
}

static int gfar_mdio_read(struct net_device *dev, int phy, int reg) {
    struct gfar_mac *mac = netdev_priv(dev);
    return mii_read(mac, phy, reg);
}

static void mii_write(struct gfar_mac *mac, int phy, int reg, u16 value) {
    struct gfar_mii __iomem *regs = mdio_bus.regs;
    unsigned i;
    spin_lock_bh(&mdio_bus.lock);

    gfar_select_mdio_bus(mac);
    out_be32(&regs->miimadd, (phy << 8) | reg);
    out_be32(&regs->miimcon, value);

    for (i = 0; i < 100000; ++i) {
	if (in_be32(&regs->miimind) & MIIMIND_BUSY) {
	}
	else {
	    break;
	}
    }
    spin_unlock_bh(&mdio_bus.lock);
}

static void gfar_mdio_write(struct net_device *dev, int phy, int reg,
	int data) {
    struct gfar_mac *mac = netdev_priv(dev);
    mii_write(mac, phy, reg, data);
}

static void gfar_mdio_reset(void) {
    struct gfar_mii __iomem *regs = mdio_bus.regs;
    unsigned int timeout = 100000;
    spin_lock_bh(&mdio_bus.lock);

    out_be32(&regs->miimcfg, MIIMCFG_RESET);
    out_be32(&regs->miimcfg, MIIMCFG_INIT_VALUE);

    while ((in_be32(&regs->miimind) & MIIMIND_BUSY) && --timeout) {
	cpu_relax();
    }

    spin_unlock_bh(&mdio_bus.lock);

    if (!timeout) {
	printk("The MII Bus is stuck!\n");
    }
}

static int gfar_mdio_init(struct device_node *np) {
    u64 addr, size;

    addr = of_translate_address(np, of_get_address(np, 0, &size, NULL));
    mdio_bus.regs = ioremap(addr, size);
    if (!mdio_bus.regs) {
	return -ENOMEM;
    }

    if (!mdio_bus.refs) {
	gfar_mdio_reset();
    }
    ++mdio_bus.refs;
    return 0;
}

static void gfar_mdio_stop(void) {
    --mdio_bus.refs;
    if (!mdio_bus.refs) {
	iounmap(mdio_bus.regs);
    }
}

static void gfar_dump_mii_phy(struct gfar_mac *mac, unsigned phy,
	unsigned reg_from, unsigned reg_to) {
    unsigned i;
    printk("phy %02d: ", phy);
    for (i = 0; i < reg_from; ++i) {
	printk("     ");
    }
    for (i = reg_from; i < reg_to; ++i) {
	printk("%04x ", mii_read(mac, phy, i));
    }
    printk("\n");
}

static void gfar_dump_mii(struct gfar_mac *mac) {
    unsigned i;
    printk("regs:   ");
    for (i = 0; i < 32; ++i) {
	printk("r%02d  ", i);
    }
    printk("\n");

    for (i = 0; i < 10; ++i) {
	gfar_dump_mii_phy(mac, i, 0, 32);
    }
}

static void gfar_dump(struct gfar_mac *mac, unsigned flags) {
    char __iomem *regs = (char *)mac->regs;
    unsigned i;
    unsigned j;
    printk("%s: dump\n", mac->dev->name);

    printk("ievent:  %08x\n", in_be32(&mac->regs->ievent));
    printk("imask:   %08x\n", in_be32(&mac->regs->imask));
    printk("edis:    %08x\n", in_be32(&mac->regs->edis));
    printk("ecntrl:  %08x\n", in_be32(&mac->regs->ecntrl));
    printk("ptv:     %08x\n", in_be32(&mac->regs->ptv));
    printk("dmactrl: %08x\n", in_be32(&mac->regs->dmactrl));
    printk("tctrl:   %08x\n", in_be32(&mac->regs->tctrl));
    printk("tstat:   %08x\n", in_be32(&mac->regs->tstat));
    printk("txic:    %08x\n", in_be32(&mac->regs->txic));
    printk("tqueue:  %08x\n", in_be32(&mac->regs->tqueue));
    printk("tr03wt:  %08x\n", in_be32(&mac->regs->tr03wt));
    printk("tr47wt:  %08x\n", in_be32(&mac->regs->tr47wt));
    printk("rctrl:   %08x\n", in_be32(&mac->regs->rctrl));
    printk("rstat:   %08x\n", in_be32(&mac->regs->rstat));
    printk("rxic:    %08x\n", in_be32(&mac->regs->rxic));
    printk("rqueue:  %08x\n", in_be32(&mac->regs->rqueue));
    printk("maccfg1: %08x\n", in_be32(&mac->regs->maccfg1));
    printk("maccfg2: %08x\n", in_be32(&mac->regs->maccfg2));
    printk("maxfrm:  %08x\n", in_be32(&mac->regs->maxfrm));
    printk("ifstat:  %08x\n", in_be32(&mac->regs->ifstat));

    if (flags & 1) {
	for (i = 0; i < sizeof(struct gfar); i += 16) {
	    printk("%04x: %08x %08x %08x %08x\n", i,
		    in_be32((unsigned *)(regs + i)),
		    in_be32((unsigned *)(regs + i + 4)),
		    in_be32((unsigned *)(regs + i + 8)),
		    in_be32((unsigned *)(regs + i + 12))
		);
	}
    }

    for (j = 0; j < mac->rx_queue_count; ++j) {
	printk("RX %d %d\n",
		mac->rx_queue[j].dirty,
		mac->rx_queue[j].cur);
	if (flags & 1) {
	    for (i = 0; i < mac->rx_desc_per_queue_count; ++i) {
		printk("%03d: %08x %08x\n", i,
			mac->rx_queue[j].desc[i].status_len,
			mac->rx_queue[j].desc[i].buf
		    );
	    }
	}
    }
}

static inline void gfar_dump_filer(struct gfar_mac *mac) {
    struct gfar __iomem *regs = mac->regs;
    unsigned i;

    printk("%s: filer %08x\n", mac->dev->name, in_be32(&regs->rbifx));
    for (i = 0; i < 256; ++i) {
	out_be32(&regs->rqfar, i);
	printk("%02x: %08x %08x\n",
		i, in_be32(&regs->rqfcr), in_be32(&regs->rqfpr));
    }
}

struct gfar_filer {
    struct gfar_mac *mac;
    unsigned idx;
};

static inline void gfar_write_filer(struct gfar_filer *f,
	unsigned int fcr, unsigned int fpr) {
    struct gfar __iomem *regs = f->mac->regs;

    out_be32(&regs->rqfar, f->idx);
    out_be32(&regs->rqfcr, fcr);
    out_be32(&regs->rqfpr, fpr);
    ++f->idx;
}

static void gfar_two_rx_queue_filer(struct gfar_mac *mac) {
    struct gfar_filer f = { mac, 0 };

    unsigned i;
    for (i = 0; i < 64; ++i) {
	unsigned src_addr = i & 0x3;
	unsigned dst_addr = (i >> 2) & 0x3;
	unsigned src_port = (i >> 4) & 0x1;
	unsigned dst_port = (i >> 5) & 0x1;

	unsigned queue_num = i ^ (i >> 1) ^ (i >> 2) ^ (i >> 3) ^ (i >> 4) ^ (i >> 5);
	queue_num &= 1;
	if (queue_num == 0) {
	    continue;
	}

	gfar_write_filer(&f, RQFCR_AND | RQFCR_PID_MASK, 0x3);
	gfar_write_filer(&f, RQFCR_AND | RQFCR_PID_SIA, src_addr);
	gfar_write_filer(&f, RQFCR_AND | RQFCR_PID_DIA, dst_addr);
	gfar_write_filer(&f, RQFCR_AND | RQFCR_PID_MASK, 0x1);
	gfar_write_filer(&f, RQFCR_AND | RQFCR_PID_SPT, src_port);
	gfar_write_filer(&f, (0x1 << 10) | RQFCR_PID_DPT, dst_port);
    }

    gfar_write_filer(&f, (0x0 << 10) | RQFCR_PID_MASK, 0);


//    gfar_write_filer(mac, 0, (0x1 << 10) | 0, BIT(22));
//    gfar_write_filer(mac, 1, (0x1 << 10) | 1, BIT(22));

//    gfar_write_filer(&f, (0x1 << 10) | RQFCR_CMP_NOEXACT | RQFCR_PID_MASK, 0xffffffff);

//    gfar_write_filer(&f, (0x2 << 10) | RQFCR_CMP_EXACT | RQFCR_PID_ETY, 0x800);


//    gfar_write_filer(&f, (0x0 << 10) | RQFCR_CMP_EXACT | RQFCR_PID_PARSE, 0);

//    gfar_write_filer(&f, (0x1 << 10) | RQFCR_AND | RQFCR_PID_MASK, 0x200);
//    gfar_write_filer(&f, (0x2 << 10) | RQFCR_CMP_EXACT | RQFCR_PID_PARSE, 0x200);
//    gfar_write_filer(&f, (0x2 << 10) | RQFCR_CMP_EXACT | RQFCR_PID_DAL, 0x00111111);
//    gfar_write_filer(&f, (0x2 << 10) | RQFCR_CMP_EXACT | RQFCR_PID_DIA, 0x03040404);
//    gfar_write_filer(&f, (0x2 << 10) | RQFCR_CMP_EXACT | RQFCR_PID_DIA, 0);

//    gfar_write_filer(&f, (0x2 << 10) | RQFCR_CMP_EXACT | RQFCR_PID_L4P, 17);

//    gfar_write_filer(&f, (0x1 << 10) | RQFCR_AND | RQFCR_PID_MASK, 0x1);
//    gfar_write_filer(&f, (0x1 << 10) | RQFCR_CMP_EXACT | RQFCR_PID_SPT, 0x0001);
//    gfar_write_filer(&f, (0x0 << 10) | RQFCR_PID_MASK, 0);
}

static void gfar_setup_filer(struct gfar_mac *mac) {

    gfar_two_rx_queue_filer(mac);

//    gfar_dump_filer(mac);
}

static void gfar_get_drvinfo(struct net_device *dev,
			     struct ethtool_drvinfo *drvinfo) {
    struct gfar_mac *mac = netdev_priv(dev);
    strcpy(drvinfo->driver, "gianfar");
    strcpy(drvinfo->version, "1.4");
    sprintf(drvinfo->bus_info, "%u", mac->phy_id);
    ethtool_drvinfo_ext_fill(drvinfo, 0, JUMBO_FRAME_SIZE);
}

static int gfar_set_settings(struct net_device *dev,
			     const struct ethtool_link_ksettings *cmd) {
    struct gfar_mac *mac = netdev_priv(dev);
    if (!netif_running(dev)) return -EINVAL;
    return mii_ethtool_set_link_ksettings(&mac->sw.mii_phy, cmd);
}

static int gfar_get_settings(struct net_device *dev,
			     struct ethtool_link_ksettings *cmd) {
    struct gfar_mac *mac = netdev_priv(dev);
    if (!netif_running(dev)) return -EINVAL;
    mii_ethtool_set_link_ksettings(&mac->sw.mii_phy, cmd);
    return 0;
}

static u32 gfar_get_link(struct net_device *dev) {
    struct gfar_mac *mac = netdev_priv(dev);
    if (!netif_running(dev)) return 0;
    return mii_check_link(&mac->sw.mii_phy);
}

static void gfar_update_stats(struct gfar_mac *mac) {
    struct eth_stats *x = &mac->sw.xstats[0];
    struct gfar_extra_stats *e = &mac->extra_stats;
    unsigned i;

    fast_path_get_eth_stats(mac->sw.dev, &mac->sw.xstats[0]);

    if (mac->device_flags & FSL_GIANFAR_DEV_HAS_RMON) {
	unsigned *r = (unsigned *)&mac->regs->rmon;
	unsigned current_stats[RMON_LEN];

	for (i = 0; i < GFAR_RMON_LEN; ++i) {
	    current_stats[i] = in_be32(&r[i]);
	}

	eth_stats_update_type_ath_or_gfar_mac(
	    &mac->sw.xstats[0], current_stats, mac->last_stats);
    }

    // override some counters from rmon with ones that get counted in driver
    x->rx_align_err = e->rx_nonoctet;
    x->rx_fcs_err = e->rx_crcerr;

    // some additional counters that are not in rmon
    x->rx_overrun = e->rx_overrun;
    x->tx_underrun = e->tx_underrun;
}

static void gfar_get_ethtool_stats(struct net_device *dev,
	struct ethtool_stats *estats, u64 *stats) {
    struct gfar_mac *mac = netdev_priv(dev);
    gfar_update_stats(mac);
    switch_get_ethtool_stats(dev, estats, stats);
}

static void gfar_self_test(struct net_device *dev, struct ethtool_test *test,
	u64 *data) {
    struct gfar_mac *mac = netdev_priv(dev);
    gfar_dump(mac, test->flags);
    gfar_dump_mii(mac);
    if (mac->sw.ops) {
	switch_self_test(&mac->sw, test, data);
    }
}

static const struct ethtool_ops gfar_ethtool_ops = {
    .get_drvinfo = gfar_get_drvinfo,
    .get_link_ksettings = gfar_get_settings,
    .set_link_ksettings = gfar_set_settings,
    .get_link = gfar_get_link,
    .get_strings = switch_get_strings,
    .get_sset_count = switch_get_sset_count,
    .get_ethtool_stats = gfar_get_ethtool_stats,
    .self_test = gfar_self_test,
};


static void gfar_phy_get_status(struct gfar_mac *mac,
				int *link, int *speed, int *duplex) {
    int phy = mac->sw.mii_phy.phy_id;
    int bmcr;
    int bmsr;

    bmcr = mii_read(mac, phy, MII_BMCR);
    bmsr = mii_read(mac, phy, MII_BMSR);

    if (!(bmsr & BMSR_LSTATUS)) {
	*link = 0;
	*speed = 0;
	*duplex = -1;
	return;
    }
    *link = 1;

    if (bmcr & BMCR_ANENABLE) {
	int advert = mii_read(mac, phy, MII_ADVERTISE);
	int lpa = mii_read(mac, phy, MII_LPA);
	int bmcr2 = 0;
	int lpa2 = 0;
	int nego = mii_nway_result(advert & lpa);

	if (mac->sw.mii_phy.supports_gmii) {
	    bmcr2 = mii_read(mac, phy, MII_CTRL1000);
	    lpa2 = mii_read(mac, phy, MII_STAT1000);
	}

	if ((bmcr2 & (ADVERTISE_1000HALF | ADVERTISE_1000FULL)) &
	    (lpa2 >> 2))
	    *speed = 1000;
	else if (nego == LPA_100FULL || nego == LPA_100HALF) 
	    *speed = 100;
	else
	    *speed = 10;
	if ((lpa2 & LPA_1000FULL) || nego == LPA_100FULL ||
	    nego == LPA_10FULL) {
	    *duplex = 1;
	} else {
	    *duplex = 0;
	}
    }
    else {
	*speed = ((bmcr & BMCR_SPEED1000 &&
		   (bmcr & BMCR_SPEED100) == 0) ? 1000 :
		  (bmcr & BMCR_SPEED100) ? 100 : 10);
	*duplex = bmcr & BMCR_FULLDPLX;
    }
}

#ifdef STATS
static void gfar_perf_stop(void *data) {
    perf_core_disable();
}

static void gfar_perf_show(void *data) {
    perf_core_show();
    perf_core_cycles();
}

static void gfar_perf_start(void *data) {
//    perf_core_l1_cache();

    perf_core_start(

	PERF_DATA_L1_CACHE_RELOADS,
	PERF_BRANCHES_FINISHED,
	PERF_BRANCHES_MISPREDICTED,
	PERF_INSNS_COMPLETED

/*
	PERF_SNOOP_REQUESTS,
	PERF_SNOOP_HITS,
	PERF_SNOOP_PUSHES,
	PERF_SNOOP_RETRIES
*/
/*
	PERF_TOTAL_TRANSLATED,
	PERF_LOADS_TRANSLATED,
	PERF_STORES_TRANSLATED,
	PERF_TOUCHES_TRANSLATED
*/
/*
	PERF_TOTAL_ALLOC,
	PERF_LOADS_TRANS_AND_ALLOC,
	PERF_STORES_TRANS_AND_ALLOC,
	PERF_TOUCHES_TRANS_AND_ALLOC
*/
/*
	PERF_LOAD_MISS_DLBF_FULL_CYCLES,
	PERF_LOAD_MISS_QUEUE_FULL_CYCLES,
	PERF_LOAD_GUARDED_MISS_CYCLES,
	PERF_TRANS_STORE_QUEUE_FULL_CYCLES
*/
/*
	PERF_LOAD_MISS_DLBF_FULL,
	PERF_LOAD_MISS_QUEUE_FULL,
	PERF_LOAD_GUARDED_MISS,
	PERF_TRANS_STORE_QUEUE_FULL
*/
/*
	PERF_DECODE_STALLED_CYCLES,
	PERF_MU_SCHED_STALLED_CYCLES,
	PERF_LSU_SCHED_STALLED_CYCLES,
	PERF_BU_SCHED_STALLED_CYCLES
*/
/*
	PERF_ISSUE_STALLED_CYCLES,
	PERF_BRANCH_STALLED_CYCLES,
	PERF_SU1_SCHED_STALLED_CYCLES,
	PERF_SU2_SCHED_STALLED_CYCLES
*/
	);
    perf_core_enable();
}

static void gfar_perf(int output) {
    perf_cpu_disable();
    on_each_cpu(gfar_perf_stop, NULL, 1);
    if (output) {
#ifdef CONFIG_SMP
	smp_call_function_single(0, gfar_perf_show, NULL, 1);
	smp_call_function_single(1, gfar_perf_show, NULL, 1);
#else
	gfar_perf_show(NULL);
#endif
	perf_cpu_show();
    }
    perf_cpu_reset();

    perf_cpu_gen(PERF_DDR_ROW_OPEN_MISS);
    perf_cpu_gen(PERF_DDR_ROW_OPEN_HIT);
    perf_cpu_gen(PERF_DATA_L2_HIT);
    perf_cpu_gen(PERF_DATA_L2_MISS);
    perf_cpu_gen(PERF_L2_ALLOCATES);
//    perf_cpu_gen(PERF_L2_RETIRES_FULL_WRITE_QUEUE);
//    perf_cpu_gen(PERF_L2_RETIRES_ADDRESS_COLLISION);
    perf_cpu_gen(PERF_L2_VICTIMIZATIONS);
    perf_cpu_gen(PERF_L2_INVALIDATIONS);
    perf_cpu_gen(PERF_RW_CORES);

    on_each_cpu(gfar_perf_start, NULL, 1);
    perf_cpu_enable();
}

/*
#ifdef SRAM
extern unsigned sram_skb_stats_alloc;
extern unsigned sram_skb_stats_free;
extern unsigned sram_skb_stats_used;
static void sram_skb_stats(void) {
    printk("sram skb alloc: %u, free: %u, used: %u\n",
	    sram_skb_stats_alloc,
	    sram_skb_stats_free,
	    sram_skb_stats_used);
    sram_skb_stats_alloc = 0;
    sram_skb_stats_free = 0;
}
#endif
*/
void perf_test(void);
static void gfar_stats(struct gfar_mac *mac) {
    unsigned i;
    if (!mac->stats_last_jiffies) {
	mac->stats_last_jiffies = jiffies;
    }

    if ((long)(mac->stats_last_jiffies + (HZ) - jiffies) > 0) {
	return;
    }

    mac->stats_last_jiffies = jiffies;
/*
    if (mac->dev->name[6] == '0' || mac->dev->name[3] == '0') {
	static int t = 0;
	++t;
	if (t == 3) {
	    perf_test();
	}
	gfar_perf(1);
	sram_skb_stats();
    }
*/

//    if (mac->dev->name[3] == '0' ||mac->dev->name[3] == '1') {


    for (i = 0; i < mac->tx_queue_count; ++i) {
	struct gfar_tx_queue *q = &mac->tx_queue[i];
	printk("%s@%u jf:%ld int %u, ",
		mac->dev->name,
		i,
		mac->stats_last_jiffies,
		q->stats_intr);
	q->stats_intr = 0;

	printk("poll %u, rx %u, xmit %u, tx %u(%u), recycle %u, tx stops %u\n",
		q->stats_poll, q->stats_rx,
		q->stats_xmit_pack,
		q->stats_tx_pack, q->stats_tx_desc,
		q->stats_tx_recycle,
		q->stats_tx_queue_stop);
	q->stats_poll = 0;
	q->stats_rx = 0;
	q->stats_xmit_pack = 0;
	q->stats_tx_pack = 0;
	q->stats_tx_desc = 0;
	q->stats_tx_recycle = 0;
	q->stats_tx_queue_stop = 0;
    }


#ifdef SWITCH_STATS
    if (mac->sw.ops) {
	switch_print_stats(&mac->sw);
    }
#endif
}
#endif

static void gfar_phy_timer(struct timer_list *t) {
    struct gfar_mac *mac = from_timer(mac, t, phy_timer);
    struct gfar __iomem *regs = mac->regs;
    int link;
    int speed;
    int duplex;

    mod_timer(&mac->phy_timer,
	    jiffies + (alternative_function ? (HZ / 10) : HZ));

    gfar_update_stats(mac);
    if (mac->sw.ops) {
	return;
    }

    mii_check_link(&mac->sw.mii_phy);
    gfar_phy_get_status(mac, &link, &speed, &duplex);
    if (link != mac->link || speed != mac->speed || duplex != mac->duplex) {
	printk("%s: link: %d, speed: %d, fdx: %d\n",
	       mac->dev->name, link, speed, duplex);
    }

    if (link && (speed != mac->speed || duplex != mac->duplex)) {
	u32 maccfg2 = in_be32(&regs->maccfg2);
	u32 ecntrl = in_be32(&regs->ecntrl);

	maccfg2 &= ~(MACCFG2_FULL_DUPLEX | MACCFG2_IF);
	if (duplex) {
	    maccfg2 |= MACCFG2_FULL_DUPLEX;
	}

	switch (speed) {
	case 1000:
	    maccfg2 |= MACCFG2_GMII;
	    break;
	case 100:
	    maccfg2 |= MACCFG2_MII;
	    ecntrl |= ECNTRL_R100;
	    break;
	case 10:
	    maccfg2 |= MACCFG2_MII;
	    ecntrl &= ~ECNTRL_R100;
	    break;
	}

	out_be32(&regs->maccfg2, maccfg2);
	out_be32(&regs->ecntrl, ecntrl);

    }
    mac->link = link;
    mac->speed = speed;
    mac->duplex = duplex;
}

static void gfar_free_tx_queue(struct gfar_mac *mac,
	struct gfar_tx_queue *txq) {
    struct desc *txbdp;
    int i;

    if (!txq->buf_ptr) {
	return;
    }
    txbdp = txq->desc;
    for (i = 0; i < txq->count; i++) {
	if (txq->buf_ptr[i]) {
	    skb_bin_put_buffer(txq->buf_ptr[i]);
	}
	txbdp++;
    }
    kfree(txq->buf_ptr);
    txq->buf_ptr = NULL;
}

static void gfar_free_rx_queue(struct gfar_mac *mac,
	struct gfar_rx_queue *rxq) {
    struct desc *d;
    int i;

    if (!rxq->buf) {
	return;
    }
    d = rxq->desc;
    for (i = 0; i < rxq->count; i++) {
	if (rxq->buf[i]) {
	    skb_bin_put_buffer(rxq->buf[i]);
	    rxq->buf[i] = NULL;
	}
	d->status_len = 0;
	d->buf = 0;
	d++;
    }
    kfree(rxq->buf);
    rxq->buf = NULL;
}

static void gfar_free_buffers(struct gfar_mac *mac) {
    int i;

    for (i = 0; i < mac->tx_queue_count; ++i) {
	gfar_free_tx_queue(mac, &mac->tx_queue[i]);
    }

    for (i = 0; i < mac->rx_queue_count; ++i) {
	gfar_free_rx_queue(mac, &mac->rx_queue[i]);
    }

    if (mac->desc) {
	dma_free_coherent(&mac->ofdev->dev,
		sizeof(struct desc) * mac->tx_desc_per_queue_count * mac->tx_queue_count
		+ sizeof(struct desc) * mac->rx_desc_per_queue_count * mac->rx_queue_count,
		mac->desc, mac->desc_addr);
	mac->desc = NULL;
    }
}

static void gfar_stop(struct net_device *dev) {
    struct gfar_mac *mac = netdev_priv(dev);
    struct gfar __iomem *regs = mac->regs;
    unsigned long flags;
    u32 tempval;
    unsigned ret;

    if (!(dev->flags & IFF_UP)) {
	WARN_ON(1);
	return;
    }
    local_irq_save(flags);

    out_be32(&regs->imask, IMASK_INIT_CLEAR);
    out_be32(&regs->ievent, IEVENT_INIT_CLEAR);

    tempval = in_be32(&mac->regs->dmactrl);
    if ((tempval & (DMACTRL_GRS | DMACTRL_GTS))
	!= (DMACTRL_GRS | DMACTRL_GTS)) {
	tempval |= (DMACTRL_GRS | DMACTRL_GTS);
	out_be32(&mac->regs->dmactrl, tempval);

	while (1) {
	    ret = in_be32(&mac->regs->ievent) & (IEVENT_GRSC | IEVENT_GTSC);
	    if (ret == (IEVENT_GRSC | IEVENT_GTSC)) break;
	}
    }

    tempval = in_be32(&regs->maccfg1);
    tempval &= ~(MACCFG1_RX_EN | MACCFG1_TX_EN);
    out_be32(&regs->maccfg1, tempval);

    local_irq_restore(flags);

    free_irq(mac->irq_err, dev);
    free_irq(mac->irq_tx, dev);
    free_irq(mac->irq_rx, dev);

    gfar_free_buffers(mac);
}

static inline void gfar_new_desc(struct gfar_mac *mac,
	struct gfar_rx_queue *rxq, struct desc *d, void *buf) {
    u32 flags = RXBD_EMPTY | RXBD_INTERRUPT;

    d->buf = virt_to_phys(buf + NET_SKB_PAD);

    if (d == rxq->desc + rxq->count - 1) {
	flags |= RXBD_WRAP;
    }

    eieio();

    d->status_len = flags;
}

static void gfar_error(struct gfar_mac *mac) {
    struct net_device *dev = mac->dev;
    u32 events = in_be32(&mac->regs->ievent);
    out_be32(&mac->regs->ievent, events & IEVENT_ERR_MASK);

    if ((mac->device_flags & FSL_GIANFAR_DEV_HAS_MAGIC_PACKET) &&
	(events & IEVENT_MAG)) {
	events &= ~IEVENT_MAG;
    }

//    printk("%s: error interrupt (ievent=0x%08x imask=0x%08x)\n",
//	    dev->name, events, in_be32(&mac->regs->imask));

    if (events & IEVENT_TXE) {
	dev->stats.tx_errors++;

	if (events & IEVENT_LC)
	    dev->stats.tx_window_errors++;
	if (events & IEVENT_CRL)
	    dev->stats.tx_aborted_errors++;
	if (events & IEVENT_XFUN) {
	    dev->stats.tx_dropped++;
	    dev->stats.tx_fifo_errors++;
	    mac->extra_stats.tx_underrun++;

	    out_be32(&mac->regs->tstat, TSTAT_CLEAR_THALT);
	}
    }
    if (events & IEVENT_BABR) {
	dev->stats.rx_errors++;
	mac->extra_stats.rx_babr++;
    }
    if (events & IEVENT_EBERR) {
	mac->extra_stats.eberr++;
    }
    if (events & IEVENT_BABT) {
	mac->extra_stats.tx_babt++;
    }
}

static inline void gfar_imask_modify(struct gfar_mac *mac,
	unsigned clear, unsigned set) {
    u32 tempval;
    tempval = in_be32(&mac->regs->imask);
    tempval &= ~(clear);
    tempval |= set;
    out_be32(&mac->regs->imask, tempval);
}

static inline unsigned gfar_get_queue_num(struct gfar_mac *mac) {
//    unsigned queue_num = 0;
    unsigned queue_num = smp_processor_id();
//    printk("sel q%u\n", queue_num);
    return queue_num;
}

static irqreturn_t gfar_interrupt(int irq, void *dev_id) {
    struct net_device *dev = dev_id;
    struct gfar_mac *mac = netdev_priv(dev);
    unsigned queue_num = gfar_get_queue_num(mac);

    u32 events = in_be32(&mac->regs->ievent);
#ifdef STATS
    ++mac->tx_queue[queue_num].stats_intr;
#endif

    if (events & (IEVENT_TX_MASK | IEVENT_RX_MASK)) {
	struct gfar_rx_queue *rxq = &mac->rx_queue[queue_num];
	out_be32(&mac->regs->ievent, IEVENT_TX_MASK | IEVENT_RX_MASK);
//	printk("%s: intr@%u\n", dev->name, queue_num);
	spin_lock(&mac->imask_lock);
#ifdef CONFIG_SMP
	mac->irq_cpu_num = queue_num;
#endif
	gfar_imask_modify(mac, IMASK_TX_MASK | IMASK_RX_MASK, 0);
	spin_unlock(&mac->imask_lock);
	napi_schedule(&rxq->napi);
    }
    if (events & IEVENT_ERR_MASK) {
	gfar_error(mac);
    }
    return IRQ_HANDLED;
}

static void gfar_set_mac_for_addr(struct net_device *dev, int num, u8 *addr) {
    struct gfar_mac *mac = netdev_priv(dev);
    int idx;
    char tmpbuf[ETH_ALEN];
    u32 tempval;
    u32 __iomem *macptr = &mac->regs->macstnaddr1;

    macptr += num*2;

    for (idx = 0; idx < ETH_ALEN; idx++)
	tmpbuf[ETH_ALEN - 1 - idx] = addr[idx];

    out_be32(macptr, *((u32 *) (tmpbuf)));

    tempval = *((u32 *) (tmpbuf + 4));

    out_be32(macptr+1, tempval);
}

static void gfar_clear_exact_match(struct net_device *dev) {
    int idx;
    u8 zero_arr[ETH_ALEN] = {0,0,0,0,0,0};

    for(idx = 1;idx < 15 + 1;idx++)
	gfar_set_mac_for_addr(dev, idx, (u8 *)zero_arr);
}

static int gfar_alloc_tx_queue(struct gfar_mac *mac,
	struct gfar_tx_queue *txq) {
    struct gfar __iomem *regs = mac->regs;
    struct desc *d;
    int i;
    dma_addr_t addr;

#ifdef SRAM
    txq->buf_ptr = sram_alloc(sizeof(void *) * txq->count);
#else
    txq->buf_ptr = kzalloc(sizeof(void *) * txq->count, GFP_ATOMIC);
#endif
    if (!txq->buf_ptr) {
	return -ENOMEM;
    }

    txq->desc = mac->desc + mac->rx_desc_per_queue_count * mac->rx_queue_count +
	(mac->tx_desc_per_queue_count * txq->num);
    addr = mac->desc_addr +
	(mac->rx_desc_per_queue_count * mac->rx_queue_count + mac->tx_desc_per_queue_count * txq->num) * sizeof(struct desc);
    out_be32(&regs->tx[txq->num].tbase, addr);


    txq->cur = 0;
    txq->dirty = 0;
    txq->first = -1u;


    d = txq->desc;
    for (i = 0; i < txq->count; i++) {
	d->status_len = 0;
	d->buf = 0;
	d++;
    }

    d--;
    d->status_len |= TXBD_WRAP;

    return 0;
}

static int gfar_alloc_rx_queue(struct gfar_mac *mac,
	struct gfar_rx_queue *rxq) {
    struct gfar __iomem *regs = mac->regs;
    struct desc *d;
    int i;
    dma_addr_t addr;

#ifdef SRAM
    rxq->buf = sram_alloc(sizeof(void *) * rxq->count);
#else
    rxq->buf = kzalloc(sizeof(void *) * rxq->count, GFP_ATOMIC);
#endif
    if (!rxq->buf) {
	return -ENOMEM;
    }

    rxq->desc = mac->desc + mac->rx_desc_per_queue_count * rxq->num;
    addr = mac->desc_addr +
	(mac->rx_desc_per_queue_count * rxq->num) * sizeof(struct desc);
    out_be32(&regs->rx[rxq->num].rbase, addr);


    rxq->cur = rxq->count;
    rxq->dirty = rxq->count;

    d = rxq->desc;
    for (i = 0; i < rxq->count; i++) {
	void *buf = skb_bin_alloc_buffer(GFP_KERNEL);
	if (!buf) {
	    printk(KERN_ERR "%s: Can't allocate RX buffers\n", mac->dev->name);
	    return -ENOMEM;
	}
	rxq->buf[i] = buf;
	gfar_new_desc(mac, rxq, d, buf);
	d++;
    }

    d--;
    d->status_len |= TXBD_WRAP;

    return 0;
}

static int gfar_startup(struct net_device *dev) {
    dma_addr_t addr = 0;
    unsigned long vaddr;
    int i;
    struct gfar_mac *mac = netdev_priv(dev);
    struct gfar __iomem *regs = mac->regs;
    int err = 0;
    u32 rctrl = 0;
    u32 tctrl = 0;
    u32 tqueue = 0;
    u32 rqueue = 0;
    u32 tempval;

    tempval = in_be32(&mac->regs->dmactrl);
    tempval &= ~(DMACTRL_GRS | DMACTRL_GTS);
    out_be32(&mac->regs->dmactrl, tempval);

    tempval = in_be32(&mac->regs->dmactrl);
    tempval |= (DMACTRL_GRS | DMACTRL_GTS);
    out_be32(&mac->regs->dmactrl, tempval);

    while (!(in_be32(&mac->regs->ievent) & (IEVENT_GRSC | IEVENT_GTSC))) {
	cpu_relax();
    }

    out_be32(&mac->regs->maccfg1, MACCFG1_SOFT_RESET);
    out_be32(&mac->regs->maccfg1, 0);
    out_be32(&mac->regs->maccfg2, MACCFG2_INIT_SETTINGS);
    out_be32(&mac->regs->ecntrl, ECNTRL_INIT_SETTINGS);

    out_be32(&mac->regs->ievent, IEVENT_INIT_CLEAR);
    out_be32(&mac->regs->imask, IMASK_INIT_CLEAR);
    out_be32(&mac->regs->igaddr0, 0);
    out_be32(&mac->regs->igaddr1, 0);
    out_be32(&mac->regs->igaddr2, 0);
    out_be32(&mac->regs->igaddr3, 0);
    out_be32(&mac->regs->igaddr4, 0);
    out_be32(&mac->regs->igaddr5, 0);
    out_be32(&mac->regs->igaddr6, 0);
    out_be32(&mac->regs->igaddr7, 0);
    out_be32(&mac->regs->gaddr0, 0);
    out_be32(&mac->regs->gaddr1, 0);
    out_be32(&mac->regs->gaddr2, 0);
    out_be32(&mac->regs->gaddr3, 0);
    out_be32(&mac->regs->gaddr4, 0);
    out_be32(&mac->regs->gaddr5, 0);
    out_be32(&mac->regs->gaddr6, 0);
    out_be32(&mac->regs->gaddr7, 0);
    if (mac->device_flags & FSL_GIANFAR_DEV_HAS_RMON) {
	memset_io(&(mac->regs->rmon), 0, sizeof (struct rmon_mib));
	out_be32(&mac->regs->rmon.cam1, 0xffffffff);
	out_be32(&mac->regs->rmon.cam2, 0xffffffff);
    }

    mac->rx_buffer_size = dev->l2mtu + ETH_HLEN;
    mac->rx_padding = 0;
    if (mac->device_flags & FSL_GIANFAR_DEV_HAS_PADDING) {
	mac->rx_buffer_size += 2;
	mac->rx_padding += 2;
    }
    if (mac->rx_queue_count > 1) {
	mac->rx_buffer_size += 8;
	mac->rx_padding += 8;
    }

    mac->rx_buffer_size =
	(mac->rx_buffer_size & ~(INCREMENTAL_BUFFER_SIZE - 1)) +
	INCREMENTAL_BUFFER_SIZE;

    skb_bin_request(dev, mac->rx_buffer_size);
    out_be32(&mac->regs->mrblr, mac->rx_buffer_size);
    out_be32(&mac->regs->maxfrm, mac->rx_buffer_size);
    out_be32(&mac->regs->minflr, 0x40);

    tempval = in_be32(&mac->regs->maccfg2);
    if (mac->rx_buffer_size > DEFAULT_RX_BUFFER_SIZE) {
	tempval |= (MACCFG2_HUGEFRAME | MACCFG2_LENGTHCHECK);
    }
    else {
	tempval &= ~(MACCFG2_HUGEFRAME | MACCFG2_LENGTHCHECK);
    }
    out_be32(&mac->regs->maccfg2, tempval);

    out_be32(&regs->imask, IMASK_INIT_CLEAR);

#ifdef SRAM
    vaddr = (unsigned long)sram_alloc(
	sizeof(struct desc) *
	(mac->tx_desc_per_queue_count + mac->rx_desc_count));
    addr = (unsigned)(sram_paddr + (vaddr - (unsigned long)sram_vaddr));
#else
    vaddr = (unsigned long) dma_alloc_coherent(
	&mac->ofdev->dev,
	sizeof(struct desc) *
	(mac->tx_desc_per_queue_count * mac->tx_queue_count + mac->rx_desc_per_queue_count * mac->rx_queue_count),
	&addr, GFP_ATOMIC);
#endif
    if (!vaddr) {
	printk(KERN_ERR "%s: Could not allocate buffer descriptors!\n",
	       dev->name);
	return -ENOMEM;
    }
    mac->desc = (struct desc *)vaddr;
    mac->desc_addr = addr;

    printk("%s: tx_queue_count %d\n", dev->name, mac->tx_queue_count);
    for (i = 0; i < mac->tx_queue_count; ++i) {
	struct gfar_tx_queue *txq = &mac->tx_queue[i];
	txq->num = i;
	txq->count = mac->tx_desc_per_queue_count;
	if (gfar_alloc_tx_queue(mac, txq)) {
	    goto err_alloc_fail;
	}
	tqueue |= (TQUEUE_EN0 >> i);
    }
    out_be32(&regs->tqueue, tqueue);
//    printk("wighting reg: %08x %08x\n", in_be32(&regs->tr03wt), in_be32(&regs->tr47wt));
    out_be32(&regs->tr03wt, 0xffffffff);
    out_be32(&regs->tr47wt, 0xffffffff);

    for (i = 0; i < mac->rx_queue_count; ++i) {
	struct gfar_rx_queue *rxq = &mac->rx_queue[i];
	rxq->num = i;
	rxq->count = mac->rx_desc_per_queue_count;
	if (gfar_alloc_rx_queue(mac, rxq)) {
	    goto err_alloc_fail;
	}
	rqueue |= (RQUEUE_EN0 >> i);
	rqueue |= (RQUEUE_EX0 >> i);
    }
    out_be32(&regs->rqueue, rqueue);
    printk("%s: rx_queue_count %d %08x\n",
	    dev->name, mac->rx_queue_count, in_be32(&regs->rqueue));
//    printk("%s: tsec_id2 %08x\n", dev->name, in_be32(&regs->tsec_id2));


    sprintf(mac->irq_names[0], "%s error", dev->name);
    if (request_irq(mac->irq_err, gfar_interrupt, 0, mac->irq_names[0], dev) < 0) {
	printk(KERN_ERR "%s: Can't get IRQ %d\n",
		dev->name, mac->irq_err);
	err = -1;
	goto err_irq_fail;
    }
	
    sprintf(mac->irq_names[1], "%s tx", dev->name);
    if (request_irq(mac->irq_tx, gfar_interrupt, 0, mac->irq_names[1], dev) < 0) {
	printk(KERN_ERR "%s: Can't get IRQ %d\n",
		dev->name, mac->irq_tx);
	err = -1;
	goto tx_irq_fail;
    }
	
    sprintf(mac->irq_names[2], "%s rx", dev->name);
    if (request_irq(mac->irq_rx, gfar_interrupt, 0, mac->irq_names[2], dev) < 0) {
	printk(KERN_ERR "%s: Can't get IRQ %d (receive0)\n",
		dev->name, mac->irq_rx);
	err = -1;
	goto rx_irq_fail;
    }

    out_be32(&regs->txic, 0x82000080);
    out_be32(&regs->rxic, 0);

    rctrl |= RCTRL_PROM;
    if (mac->rx_queue_count > 1) {
	rctrl |= RCTRL_FILREN | RCTRL_PRSDEP_INIT;
	gfar_setup_filer(mac);
	if (mac->sw.ops) {
	    rctrl |= (1 << 25);
	}
    }

    if (mac->device_flags & FSL_GIANFAR_DEV_HAS_EXTENDED_HASH) {
	rctrl |= RCTRL_EXTHASH | RCTRL_EMEN;
	gfar_clear_exact_match(dev);
    }

    if (mac->device_flags & FSL_GIANFAR_DEV_HAS_PADDING) {
	rctrl &= ~RCTRL_PAL_MASK;
	rctrl |= RCTRL_PADDING(2);
    }
    out_be32(&mac->regs->rctrl, rctrl);
//    printk("%s: rctrl %08x\n", dev->name, in_be32(&regs->rctrl));

    tctrl |= TCTRL_TXSCHED_WRRS;
//    tctrl |= TCTRL_TXSCHED_PRIO;
    out_be32(&regs->tctrl, tctrl);
    out_be32(&regs->tr03wt, 0x01010101);
    out_be32(&regs->tr47wt, 0x01010101);

    out_be32(&mac->regs->attreli, 0x80 << 16);
    out_be32(&mac->regs->attr, 0x000048c0);
//    printk("%s: attr: %08x, attreli: %08x\n", dev->name,
//	    in_be32(&regs->attr), in_be32(&regs->attreli));

    tempval = in_be32(&mac->regs->dmactrl);
    tempval |= DMACTRL_INIT_SETTINGS;
    out_be32(&mac->regs->dmactrl, tempval);

    tempval = in_be32(&mac->regs->dmactrl);
    tempval &= ~(DMACTRL_GRS | DMACTRL_GTS);
    out_be32(&mac->regs->dmactrl, tempval);

    out_be32(&regs->tstat, TSTAT_CLEAR_THALT);
    out_be32(&regs->rstat, RSTAT_CLEAR_RHALT);

    out_be32(&regs->imask, IMASK_DEFAULT);

    tempval = in_be32(&regs->maccfg1);
    tempval |= (MACCFG1_RX_EN | MACCFG1_TX_EN);
    out_be32(&regs->maccfg1, tempval);
    return 0;

rx_irq_fail:
    free_irq(mac->irq_tx, dev);
tx_irq_fail:
    free_irq(mac->irq_err, dev);
err_irq_fail:
err_alloc_fail:
    gfar_free_buffers(mac);
    return err;
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

static void gfar_phy_reset(struct mii_if_info *mii) {
    unsigned phy = mii->phy_id;
    unsigned phy_id;
    unsigned i = 0;
    mii->mdio_write(mii->dev, phy, MII_BMCR, BMCR_RESET);
    for (i = 0; i < 1000; ++i) {
	if (mii->mdio_read(mii->dev, phy, MII_BMCR) & BMCR_RESET) {
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
	mii->mdio_write(mii->dev, phy, 0x1d, 0); // RX delay
	mii->mdio_write(mii->dev, phy, 0x1e,
		mii->mdio_read(mii->dev, phy, 0x1e) | 0x100);
	mii->mdio_write(mii->dev, phy, 0x1d, 5); // TX delay
	mii->mdio_write(mii->dev, phy, 0x1e,
		mii->mdio_read(mii->dev, phy, 0x1e) | 0x100);
    }
    if (phy_id == 0x004DD072 || phy_id == 0x004DD074) {
	// disable SmartEEE
	phy_ath_mmd_write(mii, 3, 0x805d, 0x1000);
	phy_ath_mmd_write(mii, 7, 0x3c, 0);
	printk("SmartEEE disabled\n");
    }
}


static int gfar_open(struct net_device *dev) {
    struct gfar_mac *mac = netdev_priv(dev);
    unsigned i;
    int err;

//    printk("%s: gfar_open\n", dev->name);
    err = gfar_startup(dev);
    if (err) {
	return err;
    }

    timer_setup(&mac->phy_timer, gfar_phy_timer, 0);
    gfar_phy_timer(&mac->phy_timer);

//    gfar_dump(mac);
    netif_tx_start_all_queues(dev);
    for (i = 0; i < mac->rx_queue_count; ++i) {
	napi_enable(&mac->rx_queue[i].napi);
    }

    if (!mac->sw.ops) {
	if (!alternative_function) {
	    gfar_phy_reset(&mac->sw.mii_phy);
	}
    }
    else {
	switch_start(&mac->sw);
    }
    return 0;
}

static int gfar_close(struct net_device *dev) {
    struct gfar_mac *mac = netdev_priv(dev);
    unsigned i;

//    printk("%s: gfar_close\n", dev->name);
    for (i = 0; i < mac->rx_queue_count; ++i) {
	napi_disable(&mac->rx_queue[i].napi);
    }
    if (mac->sw.ops) {
	switch_stop(&mac->sw);
    }
    netif_tx_stop_all_queues(dev);
    del_timer_sync(&mac->phy_timer);

    gfar_stop(dev);
    skb_bin_release(dev);

    if (!mac->sw.ops) {
	if (!alternative_function) {
	    mii_write(mac, mac->sw.mii_phy.phy_id, MII_BMCR, BMCR_PDOWN);
	}
    }
    return 0;
}

static u16 gfar_select_queue(struct net_device *dev, struct sk_buff *skb,
			     struct net_device *sb_dev) {
    return gfar_get_queue_num(netdev_priv(dev));
}

static int gfar_fast_path_xmit(struct net_device *dev, struct fp_buf *fpb) {
    struct gfar_mac *mac = netdev_priv(dev);
    unsigned queue_num = smp_processor_id();
    struct gfar_tx_queue *txq = &mac->tx_queue[queue_num];
    struct netdev_queue *netdev_queue = netdev_get_tx_queue(dev, queue_num);
    struct desc *bd;

    fast_path_fp_tx_inc(dev, fpb);
    bd = txq->desc + txq->cur;
    bd->buf = virt_to_phys(fpb_data(fpb));

    eieio();
    bd->status_len = ((txq->cur == txq->count - 1) ? TXBD_WRAP : 0) |
	fpb_len(fpb) | TXBD_READY | TXBD_LAST | TXBD_CRC | TXBD_INTERRUPT;

    txq->buf_ptr[txq->cur] = fpb_buf(fpb);
#ifdef STATS
    ++txq->stats_xmit_pack;
#endif

    txq->cur = (txq->cur + 1) & (txq->count - 1);
    if (txq->cur == txq->dirty) {
	netif_tx_stop_queue(netdev_queue);
#ifdef STATS
	++txq->stats_tx_queue_stop;
#endif
    }
    if (!txq->xmit_commit) {
	schedule_xmit_commit(dev);
    }
    ++txq->xmit_commit;
    if (txq->xmit_commit >= 16) {
	call_xmit_commits_raw();
    }
    return 1;
}

static int gfar_xmit_commit(struct net_device *dev) {
    struct gfar_mac *mac = netdev_priv(dev);
    unsigned queue_num = smp_processor_id();
    struct gfar_tx_queue *txq = &mac->tx_queue[queue_num];
//    out_be32(&mac->regs->tstat, 0xc0000000);
    out_be32(&mac->regs->tstat, TSTAT_CLEAR_THALT >> txq->num);
    txq->xmit_commit = 0;
    return 0;
}

static int gfar_change_mtu(struct net_device *dev, int new_mtu) {
    if (new_mtu < 68 || new_mtu > dev->l2mtu) return -EINVAL;
    dev->mtu = new_mtu;
    return 0;
}

static int gfar_change_l2mtu(struct net_device *dev, int new_mtu) {
    bool running = netif_running(dev);
    if (new_mtu < 1500 || new_mtu > JUMBO_FRAME_SIZE)
	return -EINVAL;
    if (dev->l2mtu == new_mtu) {
	return 0;
    }
    if (running) {
	dev_close(dev);
    }
    dev->l2mtu = new_mtu;
    if (running) {
	dev_open(dev, NULL);
    }
    return 0;
}


static int gfar_change_mac_address(struct net_device *dev, void *p) {
    struct sockaddr *addr = p;
    if (!is_valid_ether_addr((const u8 *)addr->sa_data)) return -EADDRNOTAVAIL;
    memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);
    return 0;
}

static void gfar_reset_task(struct work_struct *work) {
    struct gfar_mac *mac = container_of(work, struct gfar_mac, reset_task);
    struct net_device *dev = mac->dev;

    printk("%s: reset_task\n", dev->name);
    rtnl_lock();
    if (netif_running(dev)) {
	dev_close(dev);
	dev_open(dev, NULL);
    }
    rtnl_unlock();

    netif_tx_schedule_all(dev);

#ifdef STATS
    ++mac->stats_restart;
#endif
}

static void gfar_tx_timeout(struct net_device *dev, unsigned txqueue) {
    struct gfar_mac *mac = netdev_priv(dev);
    schedule_work(&mac->reset_task);
}

static int gfar_tx(struct gfar_mac *mac, struct gfar_tx_queue *txq) {
    struct net_device *dev = mac->dev;
    struct desc *bd;
    int count = 0;
    struct netdev_queue *tx_queue = netdev_get_tx_queue(dev, txq->num);
    bool stopped = netif_tx_queue_stopped(tx_queue);

    while (1) {
	bd = txq->desc + txq->dirty;
	if (bd->status_len & TXBD_READY) {
	    break;
	}
	if (txq->cur == txq->dirty && !stopped) {
	    break;
	}

#ifdef STATS
	++txq->stats_tx_desc;
#endif
	__skb_bin_put_buffer(txq->buf_ptr[txq->dirty]);
	txq->buf_ptr[txq->dirty] = NULL;

#ifdef STATS
	++txq->stats_tx_pack;
#endif
	++count;
	txq->dirty = (txq->dirty + 1) & (txq->count - 1);

	if (stopped) {
	    netif_tx_wake_queue(tx_queue);
	    stopped = false;
	}
    }
    if (mac->sw.ops && count) {
	switch_tx(&mac->sw, txq->num, count);
    }
//    printk("%s: gfar tx %d: %d\n", dev->name, txq->num, count);
    return count;
}


static inline void count_errors(unsigned status, struct gfar_mac *mac) {
    struct net_device_stats *stats = &mac->dev->stats;
    struct gfar_extra_stats *estats = &mac->extra_stats;

    if (status & RXBD_TRUNCATED) {
	stats->rx_length_errors++;
	estats->rx_trunc++;
	return;
    }

    if (status & (RXBD_LARGE | RXBD_SHORT)) {
	stats->rx_length_errors++;

	if (status & RXBD_LARGE)
	    estats->rx_large++;
	else
	    estats->rx_short++;
    }
    if (status & RXBD_NONOCTET) {
	stats->rx_frame_errors++;
	estats->rx_nonoctet++;
    }
    if (status & RXBD_CRCERR) {
	estats->rx_crcerr++;
	stats->rx_crc_errors++;
    }
    if (status & RXBD_OVERRUN) {
	estats->rx_overrun++;
	stats->rx_crc_errors++;
    }
}

static int gfar_rx(struct gfar_mac *mac, struct gfar_rx_queue *rxq,
	int rx_work_limit) {
    struct net_device *dev = mac->dev;
    struct desc *d;
    void *buf;
    struct fp_buf *fpb;
    int howmany = 0;
    unsigned rmask = rxq->count - 1;
    unsigned idx = rxq->cur & rmask;
    u32 status;
    unsigned len;

    prefetch_l1_write(rxq->buf[idx] + NET_SKB_PAD);
    while (--rx_work_limit >= 0 && rxq->buf[idx]) {
	d = rxq->desc + idx;
	status = d->status_len;
	len = status & 0xffff;
	if (status & RXBD_EMPTY) {
	    break;
	}
//	rmb();
	buf = rxq->buf[idx];
	prefetch_l1_write(rxq->buf[(idx + 1) & rmask] + NET_SKB_PAD);
	rxq->buf[idx] = NULL;

	if (unlikely(!(status & RXBD_LAST) || status & RXBD_ERR)) {
	    count_errors(status, mac);

	    if (buf) {
		__skb_bin_put_buffer(buf);
	    }
	} else {
	    howmany++;

	    if (!len) {
		printk("gfar: received zero size frame\n");
		__skb_bin_put_buffer(buf);
	    }
	    else {
		fpb = fpb_build(buf, NET_SKB_PAD + mac->rx_padding,
			len - 4 - mac->rx_padding);
//		printk("dat: %p\n", fpb_data(fpb));

		if (mac->sw.ops) {
		    switch_rx_fast(&mac->sw, fpb);
		}
		else {
		    fast_path_rx(dev, fpb);
		}
#ifdef STATS
		++mac->tx_queue[rxq->num].stats_rx;
#endif
	    }
	}

	rxq->cur++;
	idx = rxq->cur & rmask;
    }

    idx = rxq->dirty & rmask;
    while (rxq->dirty != rxq->cur) {
	buf = __skb_bin_get_buffer();
	if (!buf) break;
	rxq->buf[idx] = buf;
	gfar_new_desc(mac, rxq, &rxq->desc[idx], buf);

	rxq->dirty++;
	idx = rxq->dirty & rmask;
    }
    out_be32(&mac->regs->rstat, RSTAT_CLEAR_RHALT);
    return howmany;
}

static void gfar_schedule_napi(void *data) {
    struct gfar_mac *mac = data;
    unsigned queue_num = gfar_get_queue_num(mac);
    struct gfar_rx_queue *rxq = &mac->rx_queue[queue_num];
//    printk("sched@%u %u\n", queue_num, napi_disable_pending(&rxq->napi));
    __napi_schedule(&rxq->napi);
}

static int gfar_poll(struct napi_struct *napi, int budget) {
    struct gfar_mac *mac = netdev_priv(napi->dev);
    int howmany;
    unsigned queue_num = gfar_get_queue_num(mac);
    struct gfar_rx_queue *rxq = &mac->rx_queue[queue_num];

#ifdef CONFIG_SMP
    unsigned long flags;
    spin_lock_irqsave(&mac->imask_lock, flags);
    if (mac->irq_cpu_num != -1) {
	int i;
	mac->irq_cpu_num = -1u;
	spin_unlock_irqrestore(&mac->imask_lock, flags);

	for (i = 0; i < mac->rx_queue_count; ++i) {
	    struct gfar_rx_queue *rxq = &mac->rx_queue[i];
	    if (i == queue_num) {
		continue;
	    }
	    if (napi_schedule_prep(&rxq->napi)) {
		smp_call_function_single_async(i, &rxq->csd);
	    }
	}
    }
    else {
	spin_unlock_irqrestore(&mac->imask_lock, flags);
    }
#endif

#ifdef STATS
    ++mac->tx_queue[queue_num].stats_poll;
    gfar_stats(mac);
#endif
//    printk("%s: poll@%u\n", mac->dev->name, queue_num);

    howmany = gfar_tx(mac, &mac->tx_queue[queue_num]);
    howmany += gfar_rx(mac, rxq, budget);
    if (howmany >= budget) {
	return budget;
    }

#ifdef CONFIG_SMP
    spin_lock_irqsave(&mac->imask_lock, flags);
    if (mac->irq_cpu_num != -1) {
	spin_unlock_irqrestore(&mac->imask_lock, flags);
	return budget;
    }
    napi_complete(napi);
    gfar_imask_modify(mac, 0, IMASK_TX_MASK | IMASK_RX_MASK);
    spin_unlock_irqrestore(&mac->imask_lock, flags);
#else
    napi_complete(napi);
    gfar_imask_modify(mac, 0, IMASK_TX_MASK | IMASK_RX_MASK);
#endif
    call_xmit_commits();
    return howmany;
}

static struct net_device_ops gfar_netdev_base_ops = {
    .ndo_open = gfar_open,
    .ndo_start_xmit = fast_path_start_xmit,
    .ndo_tx_timeout = gfar_tx_timeout,
    .ndo_stop = gfar_close,
    .ndo_change_mtu = gfar_change_mtu,
    .ndo_change_l2mtu = gfar_change_l2mtu,
    .ndo_set_mac_address = gfar_change_mac_address,
    .ndo_do_ioctl = switch_ioctl,
    .ndo_get_stats64 = fast_path_get_stats64,
    .ndo_fast_path_xmit = gfar_fast_path_xmit,
    .ndo_xmit_commit = gfar_xmit_commit,
};

static struct net_device_ops *gfar_get_ops(bool multi_queue) {
    static struct net_device_ops ops_arr[2];
    struct net_device_ops *ops = &ops_arr[multi_queue];
    if (ops->ndo_open) {
	return ops;
    }
    memcpy(ops, &gfar_netdev_base_ops, sizeof(*ops));
    if (multi_queue) {
	ops->ndo_select_queue = gfar_select_queue;
    }
    return ops;
}


static void gfar_set_descriptor_counts(int tx, int rx) {
    if (tx_descriptor_count == -1) {
	tx_descriptor_count = tx;
    }
    if (rx_descriptor_count == -1) {
	rx_descriptor_count = rx;
    }
}

void gfar_init_alternative(struct gfar_mac *mac);
void gfar_init_regular(struct net_device *dev);
void gfar_uninit_alternative(struct gfar_mac *mac);
static int gfar_probe(struct platform_device *ofdev) {
    struct device_node *np = ofdev->dev.of_node;
    struct device_node *phy;
    struct device_node *mdio;
    struct net_device *dev = NULL;
    struct gfar_mac *mac = NULL;
    const u32 *phy_id;
    const char *model;
    const void *mac_addr;
    u64 addr, size;
    int err = 0;
    static int num = 0;
    static int switch_num = -1;
    unsigned tx_queue_count = num_online_cpus();
    unsigned rx_queue_count = num_online_cpus();
//    unsigned tx_queue_count = 1;
//    unsigned rx_queue_count = 1;
    unsigned i;

#ifdef STATS
    perf_cpu_init();
#endif
    if (is_rb1120()) {
	gfar_set_descriptor_counts(512, 128);
    }
    else {
	gfar_set_descriptor_counts(1024, 128);
    }

    printk("gfar: rb1100: %u, rb1120: %u, txd: %u, rxd: %u, pvr: %08x, svr: %08x\n",
	    is_rb1100(), is_rb1120(),
	    tx_descriptor_count, rx_descriptor_count,
	    read_reg(0xe00a0), read_reg(0xe00a4));


    if (!np || !of_device_is_available(np)) return -ENODEV;

    phy = of_parse_phandle(np, "phy-handle", 0);
    if (!phy) {
	return -ENODEV;
    }

    mdio = of_get_parent(phy);
    if (!mdio) {
	return -ENODEV;
    }
    gfar_mdio_init(mdio);

    phy_id = of_get_property(phy, "reg", NULL);
    if (!phy_id) {
	return -ENODEV;
    }

    if (*phy_id == 0x8316 || *phy_id == 0x8327) {
	++switch_num;
	dev = alloc_switchdev(switch_num, sizeof (*mac), tx_queue_count);
    }
    else {
#ifdef SRAM
	dev = alloc_netdev_mq_sram(
	    sizeof(*mac), "eth%d", ether_setup, tx_queue_count);
#else
	dev = alloc_etherdev_mq(sizeof(*mac), tx_queue_count);
	dev->tx_queue_len = 0;
	dev->priv_flags |= IFF_NO_QUEUE;
#endif
    }
    if (!dev) {
	return -ENOMEM;
    }

    mac = netdev_priv(dev);
    mac->dev = dev;
    mac->ofdev = ofdev;
    mac->phy_id = *phy_id;

    mac->irq_tx = irq_of_parse_and_map(np, 0);
    mac->irq_rx = irq_of_parse_and_map(np, 1);
    mac->irq_err = irq_of_parse_and_map(np, 2);

    mac->rx_queue_count = rx_queue_count;
    mac->rx_desc_per_queue_count = rx_descriptor_count;
    mac->tx_queue_count = tx_queue_count;
    mac->tx_desc_per_queue_count = tx_descriptor_count;

    if (mac->irq_tx < 0 || mac->irq_rx < 0 || mac->irq_err < 0) {
	err = -EINVAL;
	goto regs_fail;
    }

    addr = of_translate_address(np, of_get_address(np, 0, &size, NULL));
    mac->regs = ioremap(addr, size);
    if (!mac->regs) {
	err = -ENOMEM;
	goto regs_fail;
    }

    model = of_get_property(np, "model", NULL);

    if (model && !strcasecmp(model, "TSEC"))
	mac->device_flags =
	    FSL_GIANFAR_DEV_HAS_GIGABIT |
	    FSL_GIANFAR_DEV_HAS_COALESCE |
	    FSL_GIANFAR_DEV_HAS_RMON |
	    FSL_GIANFAR_DEV_HAS_MULTI_INTR;
    if (model && !strcasecmp(model, "eTSEC"))
	mac->device_flags =
	    FSL_GIANFAR_DEV_HAS_GIGABIT |
	    FSL_GIANFAR_DEV_HAS_COALESCE |
	    FSL_GIANFAR_DEV_HAS_RMON |
	    FSL_GIANFAR_DEV_HAS_MULTI_INTR |
	    FSL_GIANFAR_DEV_HAS_CSUM |
	    FSL_GIANFAR_DEV_HAS_VLAN |
	    FSL_GIANFAR_DEV_HAS_MAGIC_PACKET |
	    FSL_GIANFAR_DEV_HAS_EXTENDED_HASH;
//	    FSL_GIANFAR_DEV_HAS_PADDING |

    INIT_WORK(&mac->reset_task, gfar_reset_task);

    dev_set_drvdata(&ofdev->dev, mac);

    mac_addr = of_get_mac_address(np);
    if (mac_addr)
	memcpy(dev->dev_addr, mac_addr, ETH_ALEN);

    dev->base_addr = (unsigned long) (mac->regs);

    SET_NETDEV_DEV(dev, &ofdev->dev);

    dev->netdev_ops = gfar_get_ops(mac->tx_queue_count > 1);
    dev->watchdog_timeo = 1 * HZ;
    dev->mtu = 1500;
    dev->l2mtu = 1600;
    dev->ethtool_ops = &gfar_ethtool_ops;
    dev->features |= NETIF_F_LLTX;
    spin_lock_init(&mac->imask_lock);
    for (i = 0; i < mac->rx_queue_count; ++i) {
	struct gfar_rx_queue *rxq = &mac->rx_queue[i];
	netif_napi_add(dev, &rxq->napi, gfar_poll, 64);
	rxq->csd.func = gfar_schedule_napi;
	rxq->csd.info = mac;
	rxq->csd.flags = 0;
    }

    if (mac->device_flags & FSL_GIANFAR_DEV_HAS_RMON) {
	eth_stats_calc_flags(&mac->sw.xstats[0], eth_stat_items_ath_or_gfar_mac);
    }
    eth_stats_set_flag(&mac->sw.xstats[0], ETH_STATS_RX_OVERRUN);
    eth_stats_set_flag(&mac->sw.xstats[0], ETH_STATS_TX_UNDERRUN);
    eth_stats_set_driver_basic_flags(&mac->sw.xstats[0]);

    macs[num] = mac;
    if (alternative_function && num >= 4 - alternative_function) {
//	gfar_init_alternative(mac);
    }

    rtnl_lock();
    netif_carrier_off(dev);
    err = register_netdevice(dev);
    if (err) {
	printk(KERN_ERR "%s: Cannot register net device\n", dev->name);
	goto register_fail;
    }
    if (alternative_function && num < 4 - alternative_function) {
//	gfar_init_regular(dev);
    }
//    printk("gfar mac%d is %s\n", num, dev->name);
    ++num;


    mac->duplex = -1;
    mac->sw.dev = dev;
    mac->sw.mii_phy.dev = dev;
    mac->sw.mii_phy.mdio_read = gfar_mdio_read;
    mac->sw.mii_phy.mdio_write = gfar_mdio_write;
    mac->sw.mii_phy.phy_id = mac->phy_id;
    mac->sw.mii_phy.phy_id_mask = 0x1f;
    mac->sw.mii_phy.reg_num_mask = 0x1f;
    mac->sw.mii_phy.supports_gmii = 1;
    mac->sw.mii_switch.mdio_read = gfar_mdio_read;
    mac->sw.mii_switch.mdio_write = gfar_mdio_write;

/*
    while (1) {
	printk("bus0\n");
	gfar_select_mdio_bus_rb1120(0);
	gfar_dump_mii(mac);
	printk("bus1\n");
	gfar_select_mdio_bus_rb1120(1);
	gfar_dump_mii(mac);
	printk("bus2\n");
	gfar_select_mdio_bus_rb1120(2);
	gfar_dump_mii(mac);
    }
*/

    if (mac->phy_id == 0x8316 || mac->phy_id == 0x8327) {
	int port_map_rb1100[2][6] = {
	    { 1, 2, 3, 4, 0, -1 },
	    { 1, 2, 3, 4, 0, -1 },
	};
	int port_map_rb1120[2][6] = {
	    { 0, 1, 2, 3, 4, -1 },
	    { 0, 1, 2, 3, 4, -1 },
	};
	int *port_map =
	    is_rb1100() ?
	    port_map_rb1100[switch_num] :
	    port_map_rb1120[switch_num];
	unsigned char addr[ETH_ALEN];
	memcpy(addr, mac_addr, ETH_ALEN);

	// have switch
	switch (mac->phy_id) {
	case 0x8316:
	    mac->sw.type = SWITCH_ATHEROS8316;
	    break;
	case 0x8327:
	    mac->sw.type = SWITCH_ATHEROS8327;

	    // set appropriate clock delays
	    switch (switch_num) {
	    case 0: mac->sw.specific_config = 2; break;
	    case 1: mac->sw.specific_config = 2; break;
	    }
	    break;
	}
	mac->sw.num = switch_num;
	mac->sw.tx_buffer_count = mac->tx_desc_per_queue_count;
	mac->sw.supported_l2mtu = JUMBO_FRAME_SIZE;

	err = switch_init(&mac->sw, port_map, addr);
	if (err) {
	    printk("switch init failed %d\n", err);
	    goto switch_fail;
	}
    }
    else {
	if (!alternative_function) {
	    mii_write(mac, mac->sw.mii_phy.phy_id, MII_BMCR, BMCR_PDOWN);
	}
	else {
	    gfar_phy_reset(&mac->sw.mii_phy);
	}
    }

    rtnl_unlock();
    return 0;
switch_fail:
    unregister_netdevice(dev);
register_fail:
    iounmap(mac->regs);
    rtnl_unlock();
regs_fail:
    free_netdev(dev);
    return err;
}

static int gfar_remove(struct platform_device *ofdev) {
    struct gfar_mac *mac = dev_get_drvdata(&ofdev->dev);
    unsigned i;

    rtnl_lock();
    if (mac->sw.ops) {
	switch_cleanup(&mac->sw);
    }

    if (alternative_function) {
//	gfar_uninit_alternative(mac);
    }

    cancel_work_sync(&mac->reset_task);
    dev_set_drvdata(&ofdev->dev, NULL);

    for (i = 0; i < mac->rx_queue_count; ++i) {
	netif_napi_del(&mac->rx_queue[i].napi);
    }

    unregister_netdevice(mac->dev);
    iounmap(mac->regs);
    rtnl_unlock();
    free_netdev(mac->dev);

    gfar_mdio_stop();
    return 0;
}

static struct of_device_id gfar_match[] = {
    {
	.type = "network",
	.compatible = "gianfar",
    },
    {},
};
MODULE_DEVICE_TABLE(of, gfar_match);

static struct platform_driver gfar_driver = {
    .driver = {
	.name = "fsl-gianfar",
	.owner = THIS_MODULE,
	.of_match_table = gfar_match,
    },
    .probe = gfar_probe,
    .remove = gfar_remove,
};

static int __init gfar_init(void) {
    return platform_driver_register(&gfar_driver);
}

static void __exit gfar_exit(void) {
    platform_driver_unregister(&gfar_driver);
}

module_init(gfar_init);
module_exit(gfar_exit);

