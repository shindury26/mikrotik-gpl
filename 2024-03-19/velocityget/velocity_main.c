/*
 * Copyright (c) 1996, 2003 VIA Networking Technologies, Inc.
 * All rights reserved.
 *
 * This software may be redistributed and/or modified under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 *
 * File: velocity_main.c
 *
 * Purpose: Functions for Linux drver interfaces.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include "velocity.h"
#include <linux/ethtool_extension.h>

MODULE_AUTHOR("VIA Networking Technologies, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("VIA Networking Velocity Family Gigabit Ethernet Adapter Driver");

static void enable_mii_auto_poll(struct velocity_info *hw) {
    int i;

    WRITE1(hw, 0, MAC_REG_MIICR);
    WRITE1(hw, MIIADR_SWMPL, MAC_REG_MIIADR);

    for (i = 0; i < W_MAX_TIMEOUT; i++) {
        if (READ1(hw, MAC_REG_MIISR) & MIISR_MIDLE) break;
        udelay(1);
    }

    WRITE1(hw, MIICR_MAUTO, MAC_REG_MIICR);

    for (i = 0; i < W_MAX_TIMEOUT; i++) {
        if (!(READ1(hw, MAC_REG_MIISR) & MIISR_MIDLE)) break;
        udelay(1);
    }
}

static void disable_mii_auto_poll(struct velocity_info *hw) {
    unsigned i;

    WRITE1(hw, 0, MAC_REG_MIICR);
    for (i = 0; i < W_MAX_TIMEOUT; i++) {
        if (READ1(hw, MAC_REG_MIISR) & MIISR_MIDLE) break;
        udelay(1);
    }
}

static void velocity_shutdown(struct velocity_info *hw) {
    WRITE4(hw, CR0_GINTMSK1, MAC_REG_CR0_CLR);
    WRITE4(hw, CR0_STOP, MAC_REG_CR0_SET);
    WRITE2(hw, 0xFFFF, MAC_REG_TDCSR_CLR);
    WRITE1(hw, 0xFF, MAC_REG_RDCSR_CLR);
    disable_mii_auto_poll(hw);
    WRITE4(hw, 0xffffffff, MAC_REG_ISR);
}

static void enable_flow_control_ability(struct velocity_info *hw) {
    switch (FLOW_CNTL) {
    case FLOW_CNTL_DEFAULT:
	if (READ1(hw, MAC_REG_PHYSR0) & PHYSR0_RXFLC)
	    WRITE4(hw, CR0_FDXRFCEN, MAC_REG_CR0_SET);
	else
	    WRITE4(hw, CR0_FDXRFCEN, MAC_REG_CR0_CLR);

	if (READ1(hw, MAC_REG_PHYSR0) & PHYSR0_TXFLC)
	    WRITE4(hw, CR0_FDXTFCEN, MAC_REG_CR0_SET);
	else
	    WRITE4(hw, CR0_FDXTFCEN, MAC_REG_CR0_CLR);
	break;
    case FLOW_CNTL_TX:
	WRITE4(hw, CR0_FDXTFCEN, MAC_REG_CR0_SET);
	WRITE4(hw, CR0_FDXRFCEN, MAC_REG_CR0_CLR);
	break;
    case FLOW_CNTL_RX:
	WRITE4(hw, CR0_FDXRFCEN, MAC_REG_CR0_SET);
	WRITE4(hw, CR0_FDXTFCEN, MAC_REG_CR0_CLR);
	break;
    case FLOW_CNTL_TX_RX:
	WRITE4(hw, CR0_FDXTFCEN, MAC_REG_CR0_SET);
	WRITE4(hw, CR0_FDXRFCEN, MAC_REG_CR0_SET);
	break;
    case FLOW_CNTL_DISABLE:
	WRITE4(hw, CR0_FDXRFCEN, MAC_REG_CR0_CLR);
	WRITE4(hw, CR0_FDXTFCEN, MAC_REG_CR0_CLR);
	break;
    }
}

static u32 check_connectiontype(struct velocity_info *hw) {
    u32 status = 0;
    u8 sr0 = READ1(hw, MAC_REG_PHYSR0);

    if(!(sr0 & PHYSR0_LINKGD)) {
        status |= VELOCITY_LINK_FAIL;
	return status;
    }

    status |= VELOCITY_AUTONEG_ENABLE;
    if (sr0 & PHYSR0_FDPX) {
	status |= VELOCITY_DUPLEX_FULL;
    }

    if (sr0 & PHYSR0_SPDG) {
	status |= VELOCITY_SPEED_1000;
    }

    if (sr0 & PHYSR0_SPD10) {
	status |= VELOCITY_SPEED_10;
    }
    else {
	status |= VELOCITY_SPEED_100;
    }
    return status;
}

static int velocity_mii_read(struct velocity_info *hw, unsigned reg,
			     u16 *pdata) {
    unsigned i;
    disable_mii_auto_poll(hw);

    WRITE1(hw, reg, MAC_REG_MIIADR);

    ON1(hw, MIICR_RCMD, MAC_REG_MIICR);

    for (i = 0; i < W_MAX_TIMEOUT; i++) {
        if (!(READ1(hw, MAC_REG_MIICR) & MIICR_RCMD)) break;
        udelay(5);
    }
    *pdata = READ2(hw, MAC_REG_MIIDATA);

    enable_mii_auto_poll(hw);

    if (i == W_MAX_TIMEOUT) return 0;
    return 1;
}

static int velocity_mii_write(struct velocity_info *hw, unsigned addr,
			      unsigned data) {
    unsigned i;
    disable_mii_auto_poll(hw);

    WRITE1(hw, addr, MAC_REG_MIIADR);
    WRITE2(hw, data, MAC_REG_MIIDATA);

    ON1(hw, MIICR_WCMD, MAC_REG_MIICR);

    for (i = 0; i < W_MAX_TIMEOUT; i++) {
        if (!(READ1(hw, MAC_REG_MIICR) & MIICR_WCMD)) break;
        udelay(5);
    }

    enable_mii_auto_poll(hw);

    if (i == W_MAX_TIMEOUT) return 0;
    return 1;
}

static void mac_set_cam_mask(struct velocity_info *hw, u8 *mask,
			     enum VELOCITY_CAM_TYPE cam_type) {
    int i;

    SET1(hw, CAMCR_PS_CAM_MASK, CAMCR_PS1 | CAMCR_PS0, MAC_REG_CAMCR);

    if (cam_type == VELOCITY_VLAN_ID_CAM)
        WRITE1(hw, CAMADDR_CAMEN | CAMADDR_VCAMSL, MAC_REG_CAMADDR);
    else
        WRITE1(hw, CAMADDR_CAMEN, MAC_REG_CAMADDR);

    for (i = 0; i < 8; i++) {
        WRITE1(hw, *mask++, MAC_REG_MAR + i);
    }
    WRITE1(hw, 0, MAC_REG_CAMADDR);
    SET1(hw, CAMCR_PS_MAR, CAMCR_PS1 | CAMCR_PS0, MAC_REG_CAMCR);
}

static void mac_set_cam(struct velocity_info *hw, int idx, u8 *addr,
			enum VELOCITY_CAM_TYPE cam_type) {
    int i;

    SET1(hw, CAMCR_PS_CAM_DATA, CAMCR_PS1 | CAMCR_PS0, MAC_REG_CAMCR);
    idx &= 64 - 1;

    if (cam_type == VELOCITY_VLAN_ID_CAM)
        WRITE1(hw, CAMADDR_CAMEN | CAMADDR_VCAMSL | idx, MAC_REG_CAMADDR);
    else
        WRITE1(hw, CAMADDR_CAMEN | idx, MAC_REG_CAMADDR);

    if (cam_type == VELOCITY_VLAN_ID_CAM)
        WRITE2(hw, *((u16 *)addr), MAC_REG_MAR);
    else {
        for (i = 0; i < 6; i++) {
            WRITE1(hw, *addr++, MAC_REG_MAR + i);
        }
    }
    ON1(hw, CAMCR_CAMWR, MAC_REG_CAMCR);

    udelay(10);

    WRITE1(hw, 0, MAC_REG_CAMADDR);
    SET1(hw, CAMCR_PS_MAR,CAMCR_PS1 | CAMCR_PS0, MAC_REG_CAMCR);
}

static void velocity_print_link_status(struct velocity_info *hw) {
    if (hw->mii_status & VELOCITY_LINK_FAIL) {
        printk("failed to detect cable link.\n");
    }
    else {
	printk("Link autonegation");

	if (hw->mii_status & VELOCITY_SPEED_1000)
	    printk(" speed 1000M bps");
	else if (hw->mii_status & VELOCITY_SPEED_100)
	    printk(" speed 100M bps");
	else
	    printk(" speed 10M bps");

	if (hw->mii_status & VELOCITY_DUPLEX_FULL)
	    printk(" full duplex\n");
	else
	    printk(" half duplex\n");
    }
}

static unsigned set_mii_flow_control(struct velocity_info *hw) {
    switch(FLOW_CNTL) {
    case FLOW_CNTL_TX:
        return ADVERTISE_PAUSE_ASYM;
    case FLOW_CNTL_RX:
        return ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;
    case FLOW_CNTL_TX_RX:
        return ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;
    case FLOW_CNTL_DISABLE:
        MII_OFF(ADVERTISE_PAUSE_CAP, MII_ADVERTISE, hw);
        MII_OFF(ADVERTISE_PAUSE_ASYM, MII_ADVERTISE, hw);
        return 0;
    default:
        return 0;
    }
}

static int velocity_set_media_mode(struct velocity_info *hw) {
    u16 mask;
    u16 orig, orig1000;
    u16 advert = 0, advert1000 = 0;
    u32 status = VELOCITY_LINK_UNCHANGE;

    velocity_mii_read(hw, MII_ADVERTISE, &orig);
    velocity_mii_read(hw, MII_CTRL1000, &orig1000);
    mask = orig;
    mask &= ~(ADVERTISE_PAUSE_ASYM | ADVERTISE_PAUSE_CAP |
	      ADVERTISE_100FULL | ADVERTISE_100HALF |
	      ADVERTISE_10FULL | ADVERTISE_10HALF);
    orig &=
	ADVERTISE_PAUSE_ASYM | ADVERTISE_PAUSE_CAP |
	ADVERTISE_100FULL | ADVERTISE_100HALF |
	ADVERTISE_10FULL | ADVERTISE_10HALF;
    orig1000 &= ADVERTISE_1000FULL | ADVERTISE_1000HALF;

    advert = set_mii_flow_control(hw);
    advert |=
	ADVERTISE_100FULL | ADVERTISE_100HALF |
	ADVERTISE_10FULL | ADVERTISE_10HALF;
    advert1000 |= ADVERTISE_1000FULL | ADVERTISE_1000HALF;

    OFF1(hw, CHIPGCR_FCMODE, MAC_REG_CHIPGCR);
    MII_ON(BMCR_SPEED1000, MII_BMCR, hw);
    if (orig != advert || orig1000 != advert1000) {
	advert |= mask;
	velocity_mii_write(hw, MII_ADVERTISE, advert);
	MII_ON(ADVERTISE_1000FULL | ADVERTISE_1000HALF, MII_CTRL1000, hw);
	MII_ON(BMCR_ANENABLE | BMCR_ANRESTART, MII_BMCR, hw);

	status = VELOCITY_LINK_CHANGE;
    }
    return status;
}


static void velocity_phy_reset(struct mii_if_info *mii) {
    unsigned i = 0;
    mii->mdio_write(mii->dev, mii->phy_id, MII_BMCR, BMCR_RESET);
    while (i < 100) {
	if (mii->mdio_read(mii->dev, mii->phy_id, MII_BMCR) & BMCR_RESET) {
	    ++i;
	    mdelay(1);
	}
	else {
	    break;
	}
    }
    printk("%s: reset phy %d\n", mii->dev->name, i);
}

static void velocity_init_register_cold(struct velocity_info *hw) {
    u16	temp = 0;

    SET1(hw, RX_THRESH << 4, MCFG_RFT0 | MCFG_RFT1, MAC_REG_MCFG0);
    SET1(hw, DMA_LENGTH, 0x07, MAC_REG_DCFG0);
    WRITE1(hw, WOLCFG_SAM | WOLCFG_SAB, MAC_REG_WOLCFG_SET);

    // back off algorithm use original IEEE standard
    SET1(hw, CFGB_OFSET, CFGB_CRANDOM | CFGB_CAP | CFGB_MBA | CFGB_BAKOPT,
	 MAC_REG_CFGB);

    velocity_phy_reset(&hw->mii_info);

    // enable MII auto-polling
    enable_mii_auto_poll(hw);

    // [1.18] Adaptive Interrupt
    // Set Tx Interrupt Suppression Threshold: 31 (0x001F)
    WRITE1(hw, CAMCR_PS0, MAC_REG_CAMCR);
    WRITE2(hw, 0x001F, MAC_REG_ISR_CTL);

    // Set Rx Interrupt Suppression Threshold: 31 (0x001F)
    WRITE1(hw, CAMCR_PS1, MAC_REG_CAMCR);
    WRITE2(hw, 0x001F, MAC_REG_ISR_CTL);

    // Select page to interrupt hold timer
    WRITE1(hw, 0x00, MAC_REG_CAMCR);

    // Enable Memory-Read-Line, both VT3119 and VT3216
    OFF1(hw, DCFG1_XMRL, MAC_REG_DCFG1);

    WRITE2(hw, RX_DESCS, MAC_REG_RBRDU);

    WRITE4(hw, hw->rd_pool_dma, MAC_REG_RDBASE_LO);
    WRITE2(hw, RX_DESCS - 1, MAC_REG_RDCSIZE);
    WRITE1(hw, TRDCSR_RUN, MAC_REG_RDCSR_SET);
    WRITE1(hw, TRDCSR_WAK, MAC_REG_RDCSR_SET);
    WRITE2(hw, TX_DESCS - 1, MAC_REG_TDCSIZE);

    WRITE4(hw, hw->td_pool_dma, MAC_REG_TDBASE_LO);
    WRITE2(hw, TRDCSR_RUN, MAC_REG_TDCSR_SET);

    // turn on MCFG_PQEN, turn off MCFG_RTGOPT
    SET1(hw, MCFG_PQEN, MCFG_RTGOPT, MAC_REG_MCFG0);
    ON1(hw, MCFG_VIDFR, MAC_REG_MCFG0);

    // Disable all CAM
    memset(hw->vcamMask, 0, sizeof(u8) * 8);
    memset(hw->mcamMask, 0, sizeof(u8) * 8);
    mac_set_cam_mask(hw,hw->vcamMask, VELOCITY_VLAN_ID_CAM);
    mac_set_cam_mask(hw,hw->mcamMask, VELOCITY_MULTICAST_CAM);

    mac_set_cam(hw, 0, (u8 *)&temp, VELOCITY_VLAN_ID_CAM);
    temp = 1;
    mac_set_cam_mask(hw, (u8 *)&temp, VELOCITY_VLAN_ID_CAM);

    WRITE4(hw, CR0_XONEN | CR0_XHITH1 | CR0_XLTH1 | CR0_XLTH0, MAC_REG_CR0_SET);
    WRITE4(hw, CR0_FDXTFCEN | CR0_FDXRFCEN | CR0_HDXFCEN | CR0_XHITH0,
	   MAC_REG_CR0_CLR);
    WRITE2(hw, 0xFFFF, MAC_REG_PAUSE_TIMER);

    WRITE4(hw, CR0_STOP, MAC_REG_CR0_CLR);
    WRITE4(hw, CR0_DPOLL | CR0_TXON | CR0_RXON | CR0_STRT, MAC_REG_CR0_SET);

    hw->mii_status = VELOCITY_AUTONEG_ENABLE;
    WRITE4(hw, 0xffffffffL, MAC_REG_ISR);

    velocity_mii_read(hw, MII_BMCR, &temp);
    if (temp & BMCR_ISOLATE) {
        temp &= ~BMCR_ISOLATE;
        velocity_mii_write(hw, MII_BMCR, temp);
    }

    if (velocity_set_media_mode(hw) != VELOCITY_LINK_CHANGE) {
        hw->mii_status = check_connectiontype(hw);
        velocity_print_link_status(hw);
    }

    enable_flow_control_ability(hw);

    ON1(hw, MIBCR_MIBFRZ, MAC_REG_MIBCR);
    ON1(hw, MIBCR_MIBCLR, MAC_REG_MIBCR);
    while (READ1(hw, MAC_REG_MIBCR) & MIBCR_MIBCLR);
    OFF1(hw, MIBCR_MIBFRZ, MAC_REG_MIBCR);

    WRITE4(hw, INT_MASK_DEF, MAC_REG_IMR);
}


static void velocity_dump(struct velocity_info *info) {
    unsigned i;
    struct rx_desc *rd;
    struct tx_desc *td;

    printk("DUMP %s\n", info->dev->name);
    printk("RCR %02x\n", READ1(info, MAC_REG_RCR));
    printk("TCR %02x\n", READ1(info, MAC_REG_TCR));
    printk("CR %02x %02x %02x %02x\n",
	   READ1(info, MAC_REG_CR0_SET),
	   READ1(info, MAC_REG_CR1_SET),
	   READ1(info, MAC_REG_CR2_SET),
	   READ1(info, MAC_REG_CR3_SET)
	);
    printk("ICR %04x\n", READ2(info, MAC_REG_ISR_CTL));
    printk("TXE %02x\n", READ1(info, MAC_REG_TXE_SR));
    printk("RXE %02x\n", READ1(info, MAC_REG_RXE_SR));
    printk("IS %02x %02x %02x %02x\n",
	   READ1(info, MAC_REG_ISR0),
	   READ1(info, MAC_REG_ISR1),
	   READ1(info, MAC_REG_ISR2),
	   READ1(info, MAC_REG_ISR3)
	);
    printk("IM %02x %02x %02x %02x\n",
	   READ1(info, MAC_REG_IMR0),
	   READ1(info, MAC_REG_IMR1),
	   READ1(info, MAC_REG_IMR2),
	   READ1(info, MAC_REG_IMR3)
	);
    printk("RDINDX %d %d %04x cs: %04x %04x host error: %02x\n",
	   READ2(info, MAC_REG_RDINDX),
	   READ2(info, MAC_REG_RDCSIZE),
	   READ2(info, MAC_REG_RBRDU),
	   READ2(info, MAC_REG_RDCSR_SET),
	   READ2(info, MAC_REG_RDCSR_CLR),
	   READ1(info, MAC_REG_RXE_SR)
	);
    printk("TDINDX %d %d host error: %02x\n",
	   READ2(info, MAC_REG_TDINDX),
	   READ2(info, MAC_REG_TDCSIZE),
	   READ1(info, MAC_REG_TXE_SR)
	);

    printk("    ");
    for (i = 0; i < 32; ++i) {
	printk("%02x ", i);
    }
    for (i = 0; i < 256; ++i) {
	if (!(i % 32)) printk("\n%02x: ", i);
	printk("%02x ", READ1(info, i));
    }
    printk("\n");

    printk("RX rx_curr=%d\n", info->rx_curr);
    for (i = 0; i < RX_DESCS; ++i) {
        rd = &info->rdring[i];
	printk("%d: %08x %08x %08x %08x\n", i,
	       le32_to_cpu(rd->desc0),
	       le32_to_cpu(rd->desc1),
	       le32_to_cpu(rd->addrLo),
	       le32_to_cpu(rd->desc3));
    }

    for (i = 0; i < TX_DESCS; ++i) {
        td = &info->tdring[i];
	printk("%d: %08x %08x %08x %08x\n", i,
	       le32_to_cpu(td->desc0), le32_to_cpu(td->desc1),
	       le32_to_cpu(td->bufs[0].addrLo),
	       le32_to_cpu(td->bufs[0].addrHi));
    }
}

static void velocity_init_adapter(struct velocity_info *info) {
    struct net_device *dev = info->dev;
    int i;
    unsigned rx_mode = RCR_AB | RCR_PROM | RCR_AM;

    for (i = 0; i < 6; i++)
	WRITE1(info, dev->dev_addr[i], MAC_REG_PAR + i);

    OFF1(info, CFGA_PACPI, MAC_REG_CFGA);

    netif_stop_queue(dev);

    velocity_init_register_cold(info);

    WRITE4(info, 0xffffffff, MAC_REG_MAR);
    WRITE4(info, 0xffffffff, MAC_REG_MAR + 4);

    if (dev->l2mtu > 1500) rx_mode |= RCR_AL;

    WRITE1(info, rx_mode, MAC_REG_RCR);
    OFF1(info, 0x7, MAC_REG_MCFG0);

    if (!(info->mii_status & VELOCITY_LINK_FAIL))
	netif_wake_queue(dev);
}

static int velocity_mdio_read(struct net_device *dev, int phy, int reg) {
    struct velocity_info *info = netdev_priv(dev);

    u16 data;
    velocity_mii_read(info, reg, &data);
    return data;
}

static void velocity_mdio_write(struct net_device *dev, int phy, int reg,
				int val) {
    struct velocity_info *info = netdev_priv(dev);

    velocity_mii_write(info, reg, val);
}

static void velocity_get_drvinfo(struct net_device *dev,
				 struct ethtool_drvinfo *drv) {
    struct velocity_info *info = netdev_priv(dev);
    strcpy(drv->driver, "velocity");
    strcpy(drv->version, "1.22");
    strcpy(drv->bus_info, pci_name(info->pcid));
    ethtool_drvinfo_ext_fill(drv, 0, VELOCITY_MAX_MTU);
}

static int velocity_get_link_ksettings(struct net_device *dev,
				       struct ethtool_link_ksettings *cmd) {
    struct velocity_info *priv = netdev_priv(dev);
    if (!netif_running(dev)) return -EINVAL;
    mii_ethtool_get_link_ksettings(&priv->mii_info, cmd);
    return 0;
}

static int velocity_set_link_ksettings(struct net_device *dev,
				       const struct ethtool_link_ksettings *cmd) {
    struct velocity_info *priv = netdev_priv(dev);
    if (!netif_running(dev)) return -EINVAL;
    mii_ethtool_set_link_ksettings(&priv->mii_info, cmd);
    return 0;
}

static u32 velocity_get_link(struct net_device *dev) {
    struct velocity_info *priv = netdev_priv(dev);
    if (!netif_running(dev)) return 0;
    return mii_link_ok(&priv->mii_info);
}

static struct ethtool_ops velocity_ethtool_ops = {
    .get_drvinfo  = velocity_get_drvinfo,
    .get_link_ksettings = velocity_get_link_ksettings,
    .set_link_ksettings = velocity_set_link_ksettings,
    .get_link     = velocity_get_link,
};

static int velocity_alloc_rx_buf(struct velocity_info *info, int idx) {
    struct rx_desc *rd = &info->rdring[idx];
    struct velocity_desc_info *rdinfo = &info->rd[idx];

    rdinfo->skb = dev_alloc_skb(info->rx_buf_size + DMA_ALIGN);
    if (!rdinfo->skb) return 0;

    skb_reserve(rdinfo->skb,
		DMA_ALIGN - ((unsigned)rdinfo->skb->tail & (DMA_ALIGN - 1)));
    rdinfo->skb_dma =
        pci_map_single(info->pcid, skb_tail_pointer(rdinfo->skb),
		info->rx_buf_size, PCI_DMA_FROMDEVICE);
    rd->addrLo = cpu_to_le32(rdinfo->skb_dma);

    return 1;
}

static int velocity_init_rings(struct velocity_info *info) {
    struct net_device* dev = info->dev;
    int i;
    struct rx_desc *rd;

    info->pool = pci_alloc_consistent(info->pcid,
                    RX_DESCS * sizeof(struct rx_desc) + 64 +
                    TX_DESCS * sizeof(struct tx_desc),
                    &info->pool_dma);
    if (!info->pool) {
        printk("%s : allocate dma memory failed\n", dev->name);
        return 0;
    }
    memset(info->pool, 0,
	   RX_DESCS * sizeof(struct rx_desc) + 64 +
	   TX_DESCS * sizeof(struct tx_desc));

    info->rdring =
	(struct rx_desc *)(((unsigned long)(((u8 *) info->pool) + 63)) & ~63);
    info->rd_pool_dma = info->pool_dma;
    info->tdring = (struct tx_desc *) ((u8 *)info->rdring +
				       RX_DESCS * sizeof(struct rx_desc));
    info->td_pool_dma = info->rd_pool_dma + RX_DESCS * sizeof(struct rx_desc);

    info->tx_zero_buf = pci_alloc_consistent(
	info->pcid, ETH_ZLEN, &info->tx_zero_buf_dma);
    if (!info->tx_zero_buf) {
        printk("%s: allocate dma memory failed\n", dev->name);
	goto free_pool;
    }
    memset(info->tx_zero_buf, 0, ETH_ZLEN);


    info->rd = kmalloc(
	sizeof(struct velocity_desc_info) * RX_DESCS, GFP_KERNEL);
    if (!info->rd) {
	goto free_zero;
    }
    memset(info->rd, 0, sizeof(struct velocity_desc_info) * RX_DESCS);

    for (i = 0; i < RX_DESCS; i++) {
        rd = &info->rdring[i];
        if (!velocity_alloc_rx_buf(info, i)) {
            printk("%s: can not alloc rx bufs\n", dev->name);
        }
	else {
	    rd->desc1 = 0;
	    rd->desc3 = cpu_to_le32((info->rx_buf_size << 16) | RDESC3_INTCTL);
	    rd->desc0 = cpu_to_le32(RDESC0_OWN);
	}
    }
    info->rx_curr = 0;

    info->td =
	kmalloc(sizeof(struct velocity_desc_info) * TX_DESCS, GFP_KERNEL);
    if (!info->td) {
	goto free_rd;
    }
    memset(info->td, 0, sizeof(struct velocity_desc_info) * TX_DESCS);
    info->tx_tail = 0;
    info->tx_curr = 0;
    info->tx_used = 0;

    return 1;
free_rd:
    kfree(info->rd);
free_zero:
    pci_free_consistent(
	info->pcid, ETH_ZLEN, info->tx_zero_buf, info->tx_zero_buf_dma);

free_pool:
    pci_free_consistent(info->pcid,
			RX_DESCS * sizeof(struct rx_desc) + 64 +
			TX_DESCS * sizeof(struct tx_desc),
			info->pool, info->pool_dma);
    return 0;
}

static void velocity_free_rings(struct velocity_info *info) {
    int i;
    struct velocity_desc_info *d;

    if (info->td) {
	for (i = 0; i < TX_DESCS; i++) {
	    d = &info->td[i];
	    if (d->skb) {
		if (d->skb_dma) {
		    pci_unmap_single(info->pcid, d->skb_dma,
				     d->skb->len, PCI_DMA_TODEVICE);
		    d->skb_dma = 0;
		}
		dev_kfree_skb_any(d->skb);
		d->skb = NULL;
	    }
	}
	kfree(info->td);
	info->td = NULL;
    }

    if (info->rd) {
	for (i = 0; i < RX_DESCS; i++) {
	    d = &info->rd[i];
	    if (d->skb) {
		if (d->skb_dma) {
		    pci_unmap_single(info->pcid, d->skb_dma,
				     info->rx_buf_size, PCI_DMA_FROMDEVICE);
		    d->skb_dma = 0;
		}
		dev_kfree_skb_any(d->skb);
		d->skb = NULL;
	    }
	}
	kfree(info->rd);
	info->rd = NULL;
    }

    if (info->pool) {
	pci_free_consistent(info->pcid,
			    RX_DESCS * sizeof(struct rx_desc) + 64 +
			    TX_DESCS * sizeof(struct tx_desc),
			    info->pool, info->pool_dma);
    }
    if (info->tx_zero_buf) {
        pci_free_consistent(info->pcid, ETH_ZLEN,
			    info->tx_zero_buf, info->tx_zero_buf_dma);
    }
}

static void velocity_receive_frame(struct velocity_info *info, int idx) {
    struct velocity_desc_info *rdinfo = &info->rd[idx];
    struct rx_desc *rd = &info->rdring[idx];
    struct sk_buff *skb;
    unsigned len;
    unsigned csm = cpu_to_le32(rd->desc1) >> 16;

    skb = rdinfo->skb;
    if (!skb) return;

    skb->dev = info->dev;
    if (rdinfo->skb_dma) {
	pci_unmap_single(info->pcid, rdinfo->skb_dma, info->rx_buf_size,
			 PCI_DMA_FROMDEVICE);
	rdinfo->skb_dma = 0;
    }
    rdinfo->skb = NULL;

    len = (cpu_to_le32(rd->desc0) >> 16) & 0x00003fffL;
    if (len < 4 || len > info->rx_buf_size) {
	info->dev->stats.rx_length_errors++;
	kfree_skb(skb);
	return;
    }
//    printk("%s: recv %d\n", info->dev->name, len - 4);
    skb_put(skb, len - 4);
    skb->protocol = eth_type_trans(skb, skb->dev);

    skb->ip_summed = CHECKSUM_NONE;
    if ((csm & CSM_IPKT) && (csm & CSM_IPOK)) {
        if ((csm & CSM_TCPKT) || (csm & CSM_UDPKT)) {
            if (csm & CSM_TUPOK) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	    }
        }
    }

    info->dev->stats.rx_bytes += skb->len;
    info->dev->stats.rx_packets++;
    netif_receive_skb(skb);
}

static int velocity_rx_srv(struct velocity_info *info, int budget) {
    struct rx_desc *rd;
    struct net_device_stats *stats = &info->dev->stats;
    unsigned idx = info->rx_curr;
    int works = 0;
    unsigned rsr;

    while (budget--) {
        rd = &info->rdring[idx];

        if (rd->desc0 & cpu_to_le32(RDESC0_OWN)) {
	    break;
	}
//	++info->stat_rx;

        rsr = cpu_to_le32(rd->desc0);

	if (rsr & RSR_MAR) stats->multicast++;
	if (rsr & (RSR_STP | RSR_EDP)) {
	    printk("%s: the rx frame span multple RDs\n", info->dev->name);
	    stats->rx_length_errors++;
	}

        if ((rsr & RSR_RXOK) ||
	    (!(rsr & RSR_RXOK) && (rsr & (RSR_CE | RSR_RL | RSR_VIDM)))) {

            velocity_receive_frame(info, idx);
	    if (!velocity_alloc_rx_buf(info, idx)) {
		printk("%s: can not allocate rx buf2 %d\n",
		       info->dev->name, idx);
	    }
        }
        else {
            if (rsr & RSR_CRC) stats->rx_crc_errors++;
            if (rsr & RSR_FAE) stats->rx_frame_errors++;
            stats->rx_dropped++;
        }

	rd->desc1 = 0;
	rd->desc3 = cpu_to_le32((info->rx_buf_size << 16) | RDESC3_INTCTL);

	// XXX MAGIC: you HAVE to set OWN bit for 4 descs at a time
        if ((idx % 4) == 3) {
            int i, previdx = idx;
            for (i = 0; i < 4; i++) {
                rd = &info->rdring[previdx];
                rd->desc0 |= cpu_to_le32(RDESC0_OWN);
		previdx = (previdx + RX_DESCS - 1) % RX_DESCS;
            }
            WRITE2(info, 4, MAC_REG_RBRDU);
        }

	idx = (idx + 1) % RX_DESCS;
	++works;
    }

    info->rx_curr = idx;
    return works;
}

static int velocity_tx_srv(struct velocity_info *info) {
    struct tx_desc *td;
    int idx;
    int works = 0;
    struct velocity_desc_info *tdinfo;
    struct net_device_stats *stats = &info->dev->stats;
    unsigned tsr;

    for (idx = info->tx_tail; info->tx_used > 0; idx = (idx + 1) % TX_DESCS) {
	td = &info->tdring[idx];
	tdinfo = &info->td[idx];

	if (td->desc0 & cpu_to_le32(TDESC0_OWN)) {
	    break;
	}

	tsr = cpu_to_le32(td->desc0);
	if (tsr & TSR0_TERR) {
	    stats->tx_errors++;
	    stats->tx_dropped++;
	    if (tsr & TSR0_CDH) stats->tx_heartbeat_errors++;
	    if (tsr & TSR0_CRS) stats->tx_carrier_errors++;
	    if (tsr & TSR0_ABT) stats->tx_aborted_errors++;
	    if (tsr & TSR0_OWC) stats->tx_window_errors++;
	}
	else {
	    stats->tx_packets++;
	    stats->tx_bytes += tdinfo->skb->len;
	}
//	++info->stat_tx;

	if (tdinfo->skb) {
	    if (tdinfo->skb_dma)  {
		pci_unmap_single(info->pcid, tdinfo->skb_dma, tdinfo->skb->len,
				 PCI_DMA_TODEVICE);
		tdinfo->skb_dma = 0;
	    }
	    dev_kfree_skb_any(tdinfo->skb);
	    tdinfo->skb = NULL;
	}
	info->tx_used--;
    }
    info->tx_tail = idx;

    if (netif_queue_stopped(info->dev) && info->tx_used < TX_DESCS
        && (!(info->mii_status & VELOCITY_LINK_FAIL))) {
        netif_wake_queue(info->dev);
    }
    return works;
}

static void velocity_error(struct velocity_info *info) {
    struct net_device *dev = info->dev;
    unsigned s = info->isr_status;

    if (s & ISR_TXSTLI) {
        printk("TD structure error %08x TDindex=%X\n",
	       s, READ2(info, MAC_REG_TDIDX0));
	velocity_dump(info);

//        ON1(info, TXESR_TDSTR, MAC_REG_TXE_SR);
//        WRITE2(info, TRDCSR_RUN, MAC_REG_TDCSR_CLR);
//        netif_stop_queue(dev);
    }

    if (s & ISR_SRCI) {
	info->mii_status = check_connectiontype(info);

	// only enable CD heart beat counter in 10HD mode
	if (!(info->mii_status & VELOCITY_DUPLEX_FULL) &&
	    (info->mii_status & VELOCITY_SPEED_10)) {
	    OFF1(info, TESTCFG_HBDIS, MAC_REG_TESTCFG);
	}
	else {
	    ON1(info, TESTCFG_HBDIS, MAC_REG_TESTCFG);
	}

	if ((info->mii_status & VELOCITY_SPEED_1000) ||
	    (info->mii_status & VELOCITY_SPEED_100)) {
	    WRITE1(info, 0x59, MAC_REG_TQETMR); // 100us
	    WRITE1(info, 0x14, MAC_REG_RQETMR); // 20us
	}
	else {
	    WRITE1(info, 0x00, MAC_REG_TQETMR);
	    WRITE1(info, 0x00, MAC_REG_TQETMR);
	}

        info->mii_status = check_connectiontype(info);

        velocity_print_link_status(info);
	enable_flow_control_ability(info);

	// re-enable auto-polling because SRCI will disable auto-polling
	enable_mii_auto_poll(info);

        if (info->mii_status & VELOCITY_LINK_FAIL) {
            netif_carrier_off(dev);
            netif_stop_queue(dev);
        }
        else {
            netif_carrier_on(dev);
            netif_wake_queue(dev);
        }
    }

    if (s & (ISR_LSTPEI | /*ISR_LSTEI | */ISR_OVFI | ISR_FLONI | ISR_RACEI |
		  ISR_TXSTLI | ISR_RXSTLI)) {
	if (net_ratelimit()) {
	    printk("%s: interrupt ", dev->name);
	    if (s & ISR_LSTPEI) printk("LSTPEI ");
//	    if (s & ISR_LSTEI) printk("LSTEI ");
	    if (s & ISR_OVFI) printk("OVFI ");
	    if (s & ISR_FLONI) printk("FLONI ");
	    if (s & ISR_RACEI) printk("RACEI ");
	    if (s & ISR_TXSTLI) printk("TXSTLI ");
	    if (s & ISR_RXSTLI) printk("RXSTLI ");
	    printk("\n");
	}
    }
    if (s & ISR_LSTEI) {
//	printk("ISR_LSTEI\n");
	WRITE1(info, TRDCSR_WAK, MAC_REG_RDCSR_SET);
    }
}

static int velocity_change_mtu(struct net_device *dev, int new_mtu) {
    if (new_mtu < 68 || new_mtu > dev->l2mtu) {
        printk("%s: invalid mtu %d, l2mtu %d\n", dev->name, new_mtu,
               dev->l2mtu);
        return -EINVAL;
    }

    dev->mtu = new_mtu;
    printk("%s: change mtu to %d, l2mtu %d\n", dev->name, new_mtu,
           dev->l2mtu);
    return 0;
}

static int velocity_change_l2mtu(struct net_device *dev, int new_mtu) {
    struct velocity_info *info = netdev_priv(dev);
    unsigned long flags;
    int oldmtu = dev->l2mtu;

    printk("%s: change l2mtu %d\n", dev->name, new_mtu);
    if (new_mtu == oldmtu) return 0;
    if (new_mtu < VELOCITY_MIN_MTU || new_mtu > VELOCITY_MAX_MTU
        || new_mtu < dev->mtu) {
        printk("%s: Invaild l2MTU\n", info->dev->name);
        return -EINVAL;
    }

    spin_lock_irqsave(&info->lock, flags);

    if (dev->flags & IFF_UP) {
	netif_stop_queue(dev);
	velocity_shutdown(info);
	velocity_free_rings(info);
    }

    dev->l2mtu = new_mtu;
    info->rx_buf_size = (dev->l2mtu < 1500 ? 1500 : dev->l2mtu) + 32;

    if (dev->flags & IFF_UP) {
	if (!velocity_init_rings(info)) return -ENOMEM;
	velocity_init_adapter(info);
	WRITE4(info, CR0_GINTMSK1, MAC_REG_CR0_SET);
	netif_start_queue(dev);
    }
    spin_unlock_irqrestore(&info->lock, flags);
    return 0;
}

static int velocity_set_mac_address(struct net_device *dev, void *p) {
    struct sockaddr *addr = p;

    if (!is_valid_ether_addr((const u8 *) addr->sa_data))
	return -EADDRNOTAVAIL;
    memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);
    return 0;
}

static int velocity_xmit(struct sk_buff *skb, struct net_device *dev) {
    struct velocity_info *info = netdev_priv(dev);
    int idx = info->tx_curr;
    struct tx_desc *td = &info->tdring[idx];
    struct velocity_desc_info *tdinfo = &info->td[idx];
    unsigned long flags;

    spin_lock_irqsave(&info->lock, flags);

    tdinfo->skb = skb;
    tdinfo->skb_dma =
	pci_map_single(info->pcid, skb->data, skb->len, PCI_DMA_TODEVICE);
    td->desc0 = cpu_to_le32(skb->len << 16);
    td->desc1 = cpu_to_le32((TCPLS_NORMAL << 24) | TCR_TIC);
    td->bufs[0].addrLo = cpu_to_le32(tdinfo->skb_dma);
    td->bufs[0].addrHi = cpu_to_le32(skb->len << 16);
    if (skb->len < ETH_ZLEN) {
	td->desc0 |= cpu_to_le32(ETH_ZLEN << 16);
        td->bufs[1].addrLo = cpu_to_le32(info->tx_zero_buf_dma);
	td->bufs[1].addrHi = cpu_to_le32((ETH_ZLEN - skb->len) << 16);
	td->desc1 |= cpu_to_le32(3 << 28);
    }
    else {
	td->desc0 |= cpu_to_le32(skb->len << 16);
        td->bufs[1].addrLo = cpu_to_le32(0);
	td->bufs[1].addrHi = cpu_to_le32(0);
	td->desc1 |= cpu_to_le32(2 << 28);
    }
    td->desc0 |= cpu_to_le32(TDESC0_OWN);

    info->tx_used++;
    info->tx_curr = (idx + 1) % TX_DESCS;

    if (info->tx_used >= TX_DESCS)
        netif_stop_queue(dev);

    idx = (idx + TX_DESCS - 1) % TX_DESCS;
    td = &info->tdring[idx];
    td->bufs[0].addrHi |= cpu_to_le32(TDTXBUF_QUE);

    WRITE2(info, TRDCSR_WAK, MAC_REG_TDCSR_SET);

    netif_trans_update(dev);
    spin_unlock_irqrestore(&info->lock, flags);
    return 0;
}

static int velocity_poll(struct napi_struct *napi, int budget) {
    struct velocity_info *info = container_of(napi, struct velocity_info, napi);
    int work_done;

//    ++info->stat_poll;
    spin_lock_irq(&info->lock);

    velocity_error(info);
    velocity_tx_srv(info);

    spin_unlock_irq(&info->lock);

    work_done = velocity_rx_srv(info, budget);

/*
    if (!info->last_jiffies) info->last_jiffies = jiffies;
    if (jiffies - info->last_jiffies > HZ) {
	printk("%s: stats %u %u %u %u\n", info->dev->name,
	       info->stat_intrs,
	       info->stat_poll,
	       info->stat_rx,
	       info->stat_tx
	    );

	if (info->stat_intrs && !info->stat_rx && !info->stat_tx) {
//	    velocity_dump(info);
	}

	info->last_jiffies = jiffies;
	info->stat_intrs = 0;
	info->stat_poll = 0;
	info->stat_rx = 0;
	info->stat_tx = 0;
    }
*/
    if (work_done >= budget) {
        return work_done;
    }

    info->isr_status = READ4(info, MAC_REG_ISR);
    if (info->isr_status == 0xffffffff) {
	printk("heijaa!!\n");
	info->isr_status = READ4(info, MAC_REG_ISR);
    }
    if (info->isr_status == 0xffffffff) {
	printk("%s: bad interrupt status in poll\n", info->dev->name);
	velocity_dump(info);
    }
    if (info->isr_status) {
	WRITE4(info, info->isr_status, MAC_REG_ISR);
        return budget;
    }

    napi_complete(napi);
    WRITE4(info, CR0_GINTMSK1, MAC_REG_CR0_SET);
    return work_done;
}


static irqreturn_t velocity_intr(int irq, void *dev_instance) {
    struct net_device * dev = dev_instance;
    struct velocity_info *info = netdev_priv(dev);
    unsigned isr_status;

    spin_lock(&info->lock);

    isr_status = READ4(info, MAC_REG_ISR);
    if (isr_status == 0) {
        spin_unlock(&info->lock);
        return IRQ_RETVAL(0);
    }
    if (isr_status == 0xffffffff) {
	printk("%s: bad interrupt status\n", info->dev->name);
	velocity_dump(info);
        spin_unlock(&info->lock);
	return IRQ_RETVAL(0);
    }
//    ++info->stat_intrs;

    WRITE4(info, isr_status, MAC_REG_ISR);

    if (napi_schedule_prep(&info->napi)) {
	WRITE4(info, CR0_GINTMSK1, MAC_REG_CR0_CLR);
	info->isr_status = isr_status;
	__napi_schedule(&info->napi);
    } else {
//	printk(KERN_ERR "%s: Error, poll already scheduled\n",
//	       dev->name);
    }

    spin_unlock(&info->lock);
    return IRQ_RETVAL(1);
}

static int velocity_open(struct net_device *dev) {
    struct velocity_info *info = netdev_priv(dev);
    int err;

    if (!velocity_init_rings(info)) return -ENOMEM;

    // turn this on to avoid retry forever
    PCI_ON1(MODE2_PCEROPT, PCI_REG_MODE2, info->pcid);
    // for some legacy BIOS and OS don't open BusM
    // bit in PCI configuration space. So, turn it on.
    PCI_ON1(COMMAND_BUSM, PCI_REG_COMMAND, info->pcid);
    // turn this on to detect MII coding error
    PCI_ON1(MODE3_MIION, PCI_REG_MODE3, info->pcid);

    velocity_init_adapter(info);

    err = request_irq(
	info->pcid->irq, &velocity_intr, IRQF_SHARED, dev->name, dev);

    if (err)
        return err;

    WRITE4(info, CR0_GINTMSK1, MAC_REG_CR0_SET);

//    velocity_dump(info);
    netif_start_queue(dev);
    return 0;
}

static int velocity_close(struct net_device *dev) {
    struct velocity_info *info = netdev_priv(dev);

//    velocity_dump(info);
    netif_stop_queue(dev);

    velocity_shutdown(info);

    if (dev->irq != 0)
        free_irq(dev->irq, dev);

    velocity_free_rings(info);

    velocity_mii_write(info, MII_BMCR, BMCR_PDOWN);
    return 0;
}

static int velocity_ioctl(struct net_device *dev, struct ifreq *rq, int cmd) {
    struct velocity_info *info = netdev_priv(dev);
    struct mii_ioctl_data *data = if_mii(rq);
    return generic_mii_ioctl(&info->mii_info, data, cmd, NULL);
}

struct net_device_ops velocity_netdev_ops = {
    .ndo_open           = velocity_open,
    .ndo_stop           = velocity_close,
    .ndo_start_xmit	= velocity_xmit,
    .ndo_do_ioctl       = velocity_ioctl,
    .ndo_change_mtu     = velocity_change_mtu,
    .ndo_change_l2mtu	= velocity_change_l2mtu,
    .ndo_set_mac_address = velocity_set_mac_address,
};

static int velocity_found(struct pci_dev *pcid,
			  const struct pci_device_id *ent) {
    struct net_device *dev;
    int i = 0;
    int rc;
    struct velocity_info *info;
    long ioaddr, memaddr;

    rc = pci_enable_device(pcid);
    if (rc) {
	goto err_out;
    }

    rc = pci_set_dma_mask(pcid, 0xffffffff);
    if (rc) {
        printk("velocity: PCI DMA not supported!\n");
        goto err_out;
    }

    ioaddr = pci_resource_start(pcid, 0);
    memaddr = pci_resource_start(pcid, 1);

    pci_set_master(pcid);

    pci_write_config_byte(pcid, PCI_CACHE_LINE_SIZE, 0x20);

    dev = alloc_etherdev(sizeof(struct velocity_info));
    if (!dev) {
        rc = -ENOMEM;
        printk("velocity: allocate net device failed!\n");
        goto err_out;
    }

    SET_NETDEV_DEV(dev, &pcid->dev);
    info = netdev_priv(dev);
    info->pcid = pcid;
    info->io_size = 256;
    info->ioaddr = ioaddr;
    info->memaddr = memaddr;
    info->dev = dev;
    spin_lock_init(&(info->lock));

    rc = pci_request_regions(pcid, "velocity");
    if (rc) {
        printk("velocity: Failed to find PCI device\n");
        goto err_out_free_dev;
    }

    info->hw_addr =
	ioremap(info->memaddr & PCI_BASE_ADDRESS_MEM_MASK, info->io_size);
    if (!info->hw_addr) {
        rc = -EIO;
        printk("velocity: ioremap failed for region 0x%lx\n", info->memaddr);
        goto err_out_free_res;
    }

    OFF1(info, STICKHW_SWPTAG, MAC_REG_STICKHW);
    OFF1(info, STICKHW_DS1 | STICKHW_DS0, MAC_REG_STICKHW);
    OFF1(info, CHIPGCR_FCGMII, MAC_REG_CHIPGCR);
    OFF1(info, CHIPGCR_FCMODE, MAC_REG_CHIPGCR);
    WRITE1(info, WOLCFG_PMEOVR, MAC_REG_WOLCFG_CLR);
    WRITE2(info, 0xFFFF, MAC_REG_WOLCR0_CLR);
    WRITE2(info, 0xFFFF, MAC_REG_WOLSR0_CLR);

    // software reset
    WRITE4(info, CR0_SFRST, MAC_REG_CR0_SET);
    for (i = 0; i < W_MAX_TIMEOUT; i++) {
        if (!(READ4(info, MAC_REG_CR0_SET) & CR0_SFRST)) break;
        udelay(5);
    }
    if (i == W_MAX_TIMEOUT) {
        WRITE4(info, CR0_FORSRST, MAC_REG_CR0_SET);
        mdelay(2);
    }
    mdelay(5);

    ON1(info, EECSR_RELOAD, MAC_REG_EECSR);
    for (i = 0; i < 0x1000; i++) {
	if (!(READ1(info, MAC_REG_EECSR) & EECSR_RELOAD)) break;
	udelay(10);
    }

    // set net_device related stuffs
    dev->base_addr = info->ioaddr;
    for (i = 0; i < 6; i++)
        dev->dev_addr[i] = READ1(info, MAC_REG_PAR+i);
    dev->irq                = pcid->irq;
    dev->netdev_ops = &velocity_netdev_ops;
    dev->ethtool_ops = &velocity_ethtool_ops;

    dev->l2mtu = 1600;

    netif_napi_add(dev, &info->napi, velocity_poll, 64);
    napi_enable(&info->napi);

    info->mii_info.dev = dev;
    info->mii_info.mdio_read = &velocity_mdio_read;
    info->mii_info.mdio_write = &velocity_mdio_write;
    info->mii_info.phy_id = 1;
    info->mii_info.phy_id_mask = 0x1f;
    info->mii_info.reg_num_mask = 0x1f;
    info->mii_info.supports_gmii = 1;
    velocity_mii_write(info, MII_BMCR, BMCR_PDOWN);

    info->rx_buf_size = (dev->l2mtu < 1500 ? 1500 : dev->l2mtu) + 32;

    rc = register_netdev(dev);
    if (rc) {
        printk("velocity: Failed to register netdev\n");
        goto err_out_unmap;
    }

    pci_set_drvdata(pcid, info);
    return 0;
err_out_unmap:
    iounmap(info->hw_addr);
err_out_free_res:
    pci_release_regions(pcid);
err_out_free_dev:
    free_netdev(dev);
err_out:
    return rc;
}

static void velocity_remove(struct pci_dev *pcid) {
    struct velocity_info *info = pci_get_drvdata(pcid);

    pci_set_drvdata(pcid, NULL);
    unregister_netdev(info->dev);
    iounmap(info->hw_addr);
    pci_release_regions(pcid);
    pci_disable_device(pcid);
    free_netdev(info->dev);
}

static struct pci_device_id velocity_id_table[] = {
    { 0x1106, 0x3119, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { 0 }
};
MODULE_DEVICE_TABLE(pci, velocity_id_table);

static struct pci_driver velocity_driver = {
        name:       "velocity",
        id_table:   velocity_id_table,
        probe:      velocity_found,
        remove:     velocity_remove,
};

static int __init velocity_init_module(void) {
    return pci_register_driver(&velocity_driver);
}

static void __exit velocity_cleanup_module(void) {
    pci_unregister_driver(&velocity_driver);
}

module_init(velocity_init_module);
module_exit(velocity_cleanup_module);

