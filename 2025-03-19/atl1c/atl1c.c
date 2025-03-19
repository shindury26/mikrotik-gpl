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

#ifdef __powerpc
#include <asm/rb_aux.h>
#include <asm/machdep.h>
#include <linux/sram.h>
#endif
#include <linux/skb_bin.h>
#include <linux/interrupt.h>
#include <linux/crc32.h>
#include <linux/ethtool_extension.h>
#include "atl1c.h"

static int atl1c_up(struct atl1c_adapter *adapter);
static void atl1c_down(struct atl1c_adapter *adapter);
static char atl1c_driver_name[] = "atl1c";
#define PCI_DEVICE_ID_ATTANSIC_L2C      0x1062
#define PCI_DEVICE_ID_ATTANSIC_L1C      0x1063
#define PCI_DEVICE_ID_ATHEROS_L2C_B	0x2060 /* AR8152 v1.1 Fast 10/100 */
#define PCI_DEVICE_ID_ATHEROS_L2C_B2	0x2062 /* AR8152 v2.0 Fast 10/100 */
#define PCI_DEVICE_ID_ATHEROS_L1D	0x1073 /* AR8151 v1.0 Gigabit 1000 */
#define PCI_DEVICE_ID_ATHEROS_L1D_2_0	0x1083 /* AR8151 v2.0 Gigabit 1000 */

#define L2CB_V10			0xc0
#define L2CB_V11			0xc1

static const struct pci_device_id atl1c_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_ATTANSIC, PCI_DEVICE_ID_ATTANSIC_L1C)},
	{PCI_DEVICE(PCI_VENDOR_ID_ATTANSIC, PCI_DEVICE_ID_ATTANSIC_L2C)},
	{PCI_DEVICE(PCI_VENDOR_ID_ATTANSIC, PCI_DEVICE_ID_ATHEROS_L2C_B)},
	{PCI_DEVICE(PCI_VENDOR_ID_ATTANSIC, PCI_DEVICE_ID_ATHEROS_L2C_B2)},
	{PCI_DEVICE(PCI_VENDOR_ID_ATTANSIC, PCI_DEVICE_ID_ATHEROS_L1D)},
	{PCI_DEVICE(PCI_VENDOR_ID_ATTANSIC, PCI_DEVICE_ID_ATHEROS_L1D_2_0)},
	{ }
};

MODULE_AUTHOR("Jie Yang <jie.yang@atheros.com>");
MODULE_DESCRIPTION("Atheros 1000M Ethernet Network Driver");
MODULE_LICENSE("GPL");

struct atl1c_qregs {
	u16 tpd_addr_lo;
	u16 tpd_prod;
	u16 tpd_cons;
	u16 rfd_addr_lo;
	u16 rrd_addr_lo;
	u16 rfd_prod;
	u32 tx_isr;
	u32 rx_isr;
};

static struct atl1c_qregs atl1c_qregs[AT_MAX_TRANSMIT_QUEUE] = {
	{
		REG_TPD_PRI0_ADDR_LO, REG_TPD_PRI0_PIDX, REG_TPD_PRI0_CIDX,
		REG_RFD0_HEAD_ADDR_LO, REG_RRD0_HEAD_ADDR_LO,
		REG_MB_RFD0_PROD_IDX, ISR_TX_PKT_0, ISR_RX_PKT_0
	},
	{
		REG_TPD_PRI1_ADDR_LO, REG_TPD_PRI1_PIDX, REG_TPD_PRI1_CIDX,
		REG_RFD1_HEAD_ADDR_LO, REG_RRD1_HEAD_ADDR_LO,
		REG_MB_RFD1_PROD_IDX, ISR_TX_PKT_1, ISR_RX_PKT_1
	},
	{
		REG_TPD_PRI2_ADDR_LO, REG_TPD_PRI2_PIDX, REG_TPD_PRI2_CIDX,
		REG_RFD2_HEAD_ADDR_LO, REG_RRD2_HEAD_ADDR_LO,
		REG_MB_RFD2_PROD_IDX, ISR_TX_PKT_2, ISR_RX_PKT_2
	},
	{
		REG_TPD_PRI3_ADDR_LO, REG_TPD_PRI3_PIDX, REG_TPD_PRI3_CIDX,
		REG_RFD3_HEAD_ADDR_LO, REG_RRD3_HEAD_ADDR_LO,
		REG_MB_RFD3_PROD_IDX, ISR_TX_PKT_3, ISR_RX_PKT_3
	},
};


static bool is_close(u32 n1, u32 n2, u32 size) {
    int diff = n1 - n2;
    if (diff >= -8 && diff <= 8) {
	return true;
    }
    if ((diff & (size - 1)) <= 8) {
	return true;
    }
    return false;
}

static inline u16 atl1c_tpd_avail(struct atl1c_adapter *adapter, u32 queue);
static inline void atl1c_dump(struct atl1c_adapter *adapter, u32 flags) {
	struct atl1c_hw *hw = &adapter->hw;
	unsigned i, j;

	printk("%s: atl1c_dump:\n", adapter->netdev->name);
	if (flags & ETHTOOL_FLAG_MAC_DUMP_REG) {
#define ROW_SIZE 16
		char prefix[16];
		u32 zero[ROW_SIZE / sizeof(u32)] = {};
		u32 regs[ROW_SIZE / sizeof(u32)] = {};
		unsigned i;
		for (i = 0; i < 8192; i += ROW_SIZE) {
			u32 j;
			for (j = 0; j < ROW_SIZE / sizeof(u32); ++j) {
				AT_READ_REG(hw, i + j * sizeof(u32), &regs[j]);
			}
			if (memcmp(zero, regs, ROW_SIZE)) {
				sprintf(prefix, "reg%04x ", i);
				print_hex_dump(KERN_DEBUG, prefix, DUMP_PREFIX_NONE, 32, 4, (void *)regs, ROW_SIZE, false);
			}
		}
	}

	for (j = 0; j < adapter->rx_queue_count; ++j) {
		printk("rrd%u count=%d, size=%d, tail=%d\n", j,
		       adapter->rrd_ring[j].count,
		       adapter->rrd_ring[j].size,
		       adapter->rrd_ring[j].tail);
		printk("rfd%u count=%d, size=%d, head=%d\n", j,
		       adapter->rfd_ring[j].count,
		       adapter->rfd_ring[j].size,
		       adapter->rfd_ring[j].head);
		for (i = 0; i < adapter->rrd_ring[j].count; ++i) {
			struct atl1c_recv_ret_status *rrs = &adapter->rrd_ring[j].desc[i];
			struct atl1c_rx_free_desc *rfd = &adapter->rfd_ring[j].desc[i];
			struct atl1c_buffer *bi = &adapter->rfd_ring[j].buffer_info[i];
			if ((flags & ETHTOOL_FLAG_MAC_DUMP_DESC)
			    || is_close(i, adapter->rfd_ring[j].head, adapter->rfd_ring[j].count)
			    || is_close(i, adapter->rrd_ring[j].tail, adapter->rrd_ring[j].count)) {
				printk("%u: w0=%08x rss=%08x vlan=%04x flag=%04x w3=%08x rfd:%llx bi:buf:%lx dma:%llx\n", i,
				       le32_to_cpu(rrs->word0),
				       le32_to_cpu(rrs->rss_hash),
				       le16_to_cpu(rrs->vlan_tag),
				       le16_to_cpu(rrs->flag),
				       le32_to_cpu(rrs->word3),
				       le64_to_cpu(rfd->buffer_addr),
				       (long)bi->buf, (u64)bi->dma);
			}
		}
	}

	u16 prod_idx, cons_idx;
	for (j = 0; j < adapter->tx_queue_count; ++j) {
		prod_idx = AT_READ_REGW(&adapter->hw, atl1c_qregs[j].tpd_prod);
		cons_idx = AT_READ_REGW(&adapter->hw, atl1c_qregs[j].tpd_cons);
		printk("tx ring%u count=%d, size=%d, head=%d, tail=%d stopped:%u state:%lx avail:%u prod_idx:%u cons_idx:%u\n",
		       j,
		       adapter->tpd_ring[j].count,
		       adapter->tpd_ring[j].size,
		       adapter->tpd_ring[j].head,
		       adapter->tpd_ring[j].tail,
		       __netif_subqueue_stopped(adapter->netdev, j),
		       netdev_get_tx_queue(adapter->netdev, j)->state,
		       atl1c_tpd_avail(adapter, j), prod_idx, cons_idx);
		for (i = 0; i < adapter->tpd_ring[j].count; ++i) {
			struct atl1c_tpd_desc *tpd = &adapter->tpd_ring[j].desc[i];
			struct atl1c_buffer *bi = &adapter->tpd_ring[j].buffer_info[i];
			if ((flags & ETHTOOL_FLAG_MAC_DUMP_DESC)
			    || is_close(i, adapter->tpd_ring[j].head, adapter->tpd_ring[j].count)
			    || is_close(i, adapter->tpd_ring[j].tail, adapter->tpd_ring[j].count)) {
				printk("%u: len=%u vlan=%04x word1=%08x addr=%08llx bi:buf:%lx dma:%llx fpb:%u\n", i,
				       le16_to_cpu(tpd->buffer_len),
				       le16_to_cpu(tpd->vlan_tag),
				       le32_to_cpu(tpd->word1),
				       le64_to_cpu(tpd->buffer_addr),
				       (long)bi->buf, (u64)bi->dma, bi->fpb);
			}
		}
	}
}

static int atl1c_check_eeprom_exist(struct atl1c_hw *hw) {
	u32 data;

	AT_READ_REG(hw, REG_TWSI_DEBUG, &data);
	if (data & TWSI_DEBUG_DEV_EXIST)
		return 1;

	return 0;
}

static int atl1c_read_phy_reg(struct atl1c_hw *hw, u16 reg_addr, u16 *phy_data) {
	u32 val;
	int i;

	val = ((u32)(reg_addr & MDIO_REG_ADDR_MASK)) << MDIO_REG_ADDR_SHIFT |
		MDIO_START | MDIO_RW |
		MDIO_CLK_25_28 << MDIO_CLK_SEL_SHIFT;

	AT_WRITE_REG(hw, REG_MDIO_CTRL, val);

	for (i = 0; i < MDIO_WAIT_TIMES; i++) {
		udelay(2);
		AT_READ_REG(hw, REG_MDIO_CTRL, &val);
		if (!(val & (MDIO_START | MDIO_BUSY)))
			break;
	}
	*phy_data = 0;
	if (!(val & (MDIO_START | MDIO_BUSY))) {
		*phy_data = (u16)val;
		return 0;
	}

	return -1;
}

static int atl1c_write_phy_reg(struct atl1c_hw *hw, u32 reg_addr, u16 phy_data) {
	int i;
	u32 val;

	val = ((u32)(phy_data & MDIO_DATA_MASK)) << MDIO_DATA_SHIFT   |
	       (reg_addr & MDIO_REG_ADDR_MASK) << MDIO_REG_ADDR_SHIFT |
	       MDIO_START |
	       MDIO_CLK_25_28 << MDIO_CLK_SEL_SHIFT;

	AT_WRITE_REG(hw, REG_MDIO_CTRL, val);

	for (i = 0; i < MDIO_WAIT_TIMES; i++) {
		udelay(2);
		AT_READ_REG(hw, REG_MDIO_CTRL, &val);
		if (!(val & (MDIO_START | MDIO_BUSY)))
			break;
	}

	if (!(val & (MDIO_START | MDIO_BUSY)))
		return 0;

	return -1;
}

static int atl1c_phy_setup_adv(struct atl1c_hw *hw) {
	u16 mii_adv_data = ADVERTISE_DEFAULT_CAP & ~ADVERTISE_SPEED_MASK;
	u16 mii_giga_ctrl_data = GIGA_CR_1000T_DEFAULT_CAP &
				~GIGA_CR_1000T_SPEED_MASK;

	if (hw->autoneg_advertised & ADVERTISED_10baseT_Half)
		mii_adv_data |= ADVERTISE_10HALF;
	if (hw->autoneg_advertised & ADVERTISED_10baseT_Full)
		mii_adv_data |= ADVERTISE_10FULL;
	if (hw->autoneg_advertised & ADVERTISED_100baseT_Half)
		mii_adv_data |= ADVERTISE_100HALF;
	if (hw->autoneg_advertised & ADVERTISED_100baseT_Full)
		mii_adv_data |= ADVERTISE_100FULL;

	if (hw->autoneg_advertised & ADVERTISED_Autoneg)
		mii_adv_data |= ADVERTISE_10HALF  | ADVERTISE_10FULL |
				ADVERTISE_100HALF | ADVERTISE_100FULL;

	if (hw->link_cap_flags & ATL1C_LINK_CAP_1000M) {
		if (hw->autoneg_advertised & ADVERTISED_1000baseT_Half)
			mii_giga_ctrl_data |= ADVERTISE_1000HALF;
		if (hw->autoneg_advertised & ADVERTISED_1000baseT_Full)
			mii_giga_ctrl_data |= ADVERTISE_1000FULL;
		if (hw->autoneg_advertised & ADVERTISED_Autoneg)
			mii_giga_ctrl_data |= ADVERTISE_1000HALF |
					ADVERTISE_1000FULL;
	}

	if (atl1c_write_phy_reg(hw, MII_ADVERTISE, mii_adv_data) != 0 ||
	    atl1c_write_phy_reg(hw, MII_GIGA_CR, mii_giga_ctrl_data) != 0)
		return -1;
	return 0;
}

static void atl1c_phy_disable(struct atl1c_hw *hw) {
	AT_WRITE_REGW(hw, REG_GPHY_CTRL,
			GPHY_CTRL_PW_WOL_DIS | GPHY_CTRL_EXT_RESET);
}

static void atl1c_phy_magic_data(struct atl1c_hw *hw) {
	u16 data;

	data = ANA_LOOP_SEL_10BT | ANA_EN_MASK_TB | ANA_EN_10BT_IDLE |
		((1 & ANA_INTERVAL_SEL_TIMER_MASK) <<
		ANA_INTERVAL_SEL_TIMER_SHIFT);

	atl1c_write_phy_reg(hw, MII_DBG_ADDR, MII_ANA_CTRL_18);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, data);

	data = (2 & ANA_SERDES_CDR_BW_MASK) | ANA_MS_PAD_DBG |
		ANA_SERDES_EN_DEEM | ANA_SERDES_SEL_HSP | ANA_SERDES_EN_PLL |
		ANA_SERDES_EN_LCKDT;

	atl1c_write_phy_reg(hw, MII_DBG_ADDR, MII_ANA_CTRL_5);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, data);

	data = (44 & ANA_LONG_CABLE_TH_100_MASK) |
		((33 & ANA_SHORT_CABLE_TH_100_MASK) <<
		ANA_SHORT_CABLE_TH_100_SHIFT) | ANA_BP_BAD_LINK_ACCUM |
		ANA_BP_SMALL_BW;

	atl1c_write_phy_reg(hw, MII_DBG_ADDR, MII_ANA_CTRL_54);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, data);

	data = (11 & ANA_IECHO_ADJ_MASK) | ((11 & ANA_IECHO_ADJ_MASK) <<
		ANA_IECHO_ADJ_2_SHIFT) | ((8 & ANA_IECHO_ADJ_MASK) <<
		ANA_IECHO_ADJ_1_SHIFT) | ((8 & ANA_IECHO_ADJ_MASK) <<
		ANA_IECHO_ADJ_0_SHIFT);

	atl1c_write_phy_reg(hw, MII_DBG_ADDR, MII_ANA_CTRL_4);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, data);

	data = ANA_RESTART_CAL | ((7 & ANA_MANUL_SWICH_ON_MASK) <<
		ANA_MANUL_SWICH_ON_SHIFT) | ANA_MAN_ENABLE |
		ANA_SEL_HSP | ANA_EN_HB | ANA_OEN_125M;

	atl1c_write_phy_reg(hw, MII_DBG_ADDR, MII_ANA_CTRL_0);
	atl1c_write_phy_reg(hw, MII_DBG_DATA, data);

	if (hw->ctrl_flags & ATL1C_HIB_DISABLE) {
		atl1c_write_phy_reg(hw, MII_DBG_ADDR, MII_ANA_CTRL_41);
		if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &data) != 0)
			return;
		data &= ~ANA_TOP_PS_EN;
		atl1c_write_phy_reg(hw, MII_DBG_DATA, data);

		atl1c_write_phy_reg(hw, MII_DBG_ADDR, MII_ANA_CTRL_11);
		if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &data) != 0)
			return;
		data &= ~ANA_PS_HIB_EN;
		atl1c_write_phy_reg(hw, MII_DBG_DATA, data);
	}
}


static void atl1c_phy_mmd_write(struct atl1c_hw *hw,
	unsigned mmd, unsigned reg, unsigned val) {
	u16 phy_data;
	atl1c_write_phy_reg(hw, 0xd, mmd);
	atl1c_write_phy_reg(hw, 0xe, reg);
	atl1c_write_phy_reg(hw, 0xd, 0x4000 | mmd);
	atl1c_read_phy_reg(hw, 0xe, &phy_data);
	atl1c_write_phy_reg(hw, 0xe, val);
}

static int atl1c_phy_reset(struct atl1c_hw *hw) {
	struct atl1c_adapter *adapter = hw->adapter;
	struct pci_dev *pdev = adapter->pdev;
	u16 phy_data;
	u16 phy_data2;
	unsigned phy_id;
	u32 phy_ctrl_data = GPHY_CTRL_DEFAULT;
	u32 mii_ier_data = IER_LINK_UP | IER_LINK_DOWN;
	int err;

	if (hw->ctrl_flags & ATL1C_HIB_DISABLE)
		phy_ctrl_data &= ~GPHY_CTRL_HIB_EN;

	AT_WRITE_REG(hw, REG_GPHY_CTRL, phy_ctrl_data);
	AT_WRITE_FLUSH(hw);
	msleep(40);
	phy_ctrl_data |= GPHY_CTRL_EXT_RESET;
	AT_WRITE_REG(hw, REG_GPHY_CTRL, phy_ctrl_data);
	AT_WRITE_FLUSH(hw);
	msleep(10);

	if (hw->nic_type == athr_l2c_b) {
		atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x0A);
		atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data);
		atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data & 0xDFFF);
	}

	if (hw->nic_type == athr_l2c_b ||
	    hw->nic_type == athr_l2c_b2 ||
	    hw->nic_type == athr_l1d) {
		atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x3B);
		atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data);
		atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data & 0xFFF7);
		msleep(20);
	}

	atl1c_read_phy_reg(hw, 2, &phy_data);
	atl1c_read_phy_reg(hw, 3, &phy_data2);
	phy_id = ((unsigned)phy_data << 16) | (unsigned)phy_data2;
	if (phy_id == 0x004DD070 || phy_id == 0x004DD072 ||
	    phy_id == 0x004DD074 || phy_id == 0x004DD082) {
		// disable SmartEEE
		atl1c_phy_mmd_write(hw, 3, 0x805d, 0x1000);
		atl1c_phy_mmd_write(hw, 7, 0x3c, 0);
	}

	/*Enable PHY LinkChange Interrupt */
	err = atl1c_write_phy_reg(hw, MII_IER, mii_ier_data);
	if (err) {
		if (netif_msg_hw(adapter))
			dev_err(&pdev->dev,
				"Error enable PHY linkChange Interrupt\n");
		return err;
	}
	atl1c_phy_magic_data(hw);
	return 0;
}

static int atl1c_phy_init(struct atl1c_hw *hw) {
	struct atl1c_adapter *adapter = (struct atl1c_adapter *)hw->adapter;
	struct pci_dev *pdev = adapter->pdev;
	int ret_val;
	u16 mii_bmcr_data = BMCR_RESET;
	u16 phy_id1, phy_id2;

	if ((atl1c_read_phy_reg(hw, MII_PHYSID1, &phy_id1) != 0) ||
		(atl1c_read_phy_reg(hw, MII_PHYSID2, &phy_id2) != 0)) {
			if (netif_msg_link(adapter))
				dev_err(&pdev->dev, "Error get phy ID\n");
		return -1;
	}
	switch (hw->media_type) {
	case MEDIA_TYPE_AUTO_SENSOR:
		ret_val = atl1c_phy_setup_adv(hw);
		if (ret_val) {
			if (netif_msg_link(adapter))
				dev_err(&pdev->dev,
					"Error Setting up Auto-Negotiation\n");
			return ret_val;
		}
		mii_bmcr_data |= BMCR_AUTO_NEG_EN | BMCR_RESTART_AUTO_NEG;
		break;
	case MEDIA_TYPE_100M_FULL:
		mii_bmcr_data |= BMCR_SPEED_100 | BMCR_FULL_DUPLEX;
		break;
	case MEDIA_TYPE_100M_HALF:
		mii_bmcr_data |= BMCR_SPEED_100;
		break;
	case MEDIA_TYPE_10M_FULL:
		mii_bmcr_data |= BMCR_SPEED_10 | BMCR_FULL_DUPLEX;
		break;
	case MEDIA_TYPE_10M_HALF:
		mii_bmcr_data |= BMCR_SPEED_10;
		break;
	default:
		if (netif_msg_link(adapter))
			dev_err(&pdev->dev, "Wrong Media type %d\n",
				hw->media_type);
		return -1;
		break;
	}

	ret_val = atl1c_write_phy_reg(hw, MII_BMCR, mii_bmcr_data);
	if (ret_val)
		return ret_val;
	hw->phy_configured = true;

	return 0;
}

static int atl1c_get_speed_and_duplex(struct atl1c_hw *hw, u16 *speed, u16 *duplex) {
	int err;
	u16 phy_data;

	if (hw->nic_type == athr_mt) {
		u32 spd;

		AT_READ_REG(hw, REG_MT_SPEED, &spd);
		*speed = spd;
		*duplex = FULL_DUPLEX;
		return 0;
	}

	/* Read   PHY Specific Status Register (17) */
	err = atl1c_read_phy_reg(hw, MII_GIGA_PSSR, &phy_data);
	if (err)
		return err;

	if (!(phy_data & GIGA_PSSR_SPD_DPLX_RESOLVED))
		return -1;

	switch (phy_data & GIGA_PSSR_SPEED) {
	case GIGA_PSSR_1000MBS:
		*speed = SPEED_1000;
		break;
	case GIGA_PSSR_100MBS:
		*speed = SPEED_100;
		break;
	case  GIGA_PSSR_10MBS:
		*speed = SPEED_10;
		break;
	default:
		return -1;
		break;
	}

	if (phy_data & GIGA_PSSR_DPLX)
		*duplex = FULL_DUPLEX;
	else
		*duplex = HALF_DUPLEX;

	return 0;
}

static int atl1c_get_permanent_address(struct atl1c_hw *hw) {
	u32 addr[2];
	u32 i;
	u32 otp_ctrl_data;
	u32 twsi_ctrl_data;
	u8  eth_addr[ETH_ALEN];
	u16 phy_data;
	bool raise_vol = false;

	/* init */
	addr[0] = addr[1] = 0;
	AT_READ_REG(hw, REG_OTP_CTRL, &otp_ctrl_data);
	if (atl1c_check_eeprom_exist(hw)) {
		if (hw->nic_type == athr_l1c || hw->nic_type == athr_l2c_b) {
			/* Enable OTP CLK */
			if (!(otp_ctrl_data & OTP_CTRL_CLK_EN)) {
				otp_ctrl_data |= OTP_CTRL_CLK_EN;
				AT_WRITE_REG(hw, REG_OTP_CTRL, otp_ctrl_data);
				AT_WRITE_FLUSH(hw);
				msleep(1);
			}
		}

		if (hw->nic_type == athr_l2c_b ||
		    hw->nic_type == athr_l2c_b2 ||
		    hw->nic_type == athr_l1d) {
			atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x00);
			if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data))
				goto out;
			phy_data &= 0xFF7F;
			atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data);

			atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x3B);
			if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data))
				goto out;
			phy_data |= 0x8;
			atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data);
			udelay(20);
			raise_vol = true;
		}

		AT_READ_REG(hw, REG_TWSI_CTRL, &twsi_ctrl_data);
		twsi_ctrl_data |= TWSI_CTRL_SW_LDSTART;
		AT_WRITE_REG(hw, REG_TWSI_CTRL, twsi_ctrl_data);
		for (i = 0; i < AT_TWSI_EEPROM_TIMEOUT; i++) {
			msleep(10);
			AT_READ_REG(hw, REG_TWSI_CTRL, &twsi_ctrl_data);
			if ((twsi_ctrl_data & TWSI_CTRL_SW_LDSTART) == 0)
				break;
		}
		if (i >= AT_TWSI_EEPROM_TIMEOUT)
			return -1;
	}
	/* Disable OTP_CLK */
	if ((hw->nic_type == athr_l1c || hw->nic_type == athr_l2c)) {
		if (otp_ctrl_data & OTP_CTRL_CLK_EN) {
			otp_ctrl_data &= ~OTP_CTRL_CLK_EN;
			AT_WRITE_REG(hw, REG_OTP_CTRL, otp_ctrl_data);
			AT_WRITE_FLUSH(hw);
			msleep(1);
		}
	}
	if (raise_vol) {
		if (hw->nic_type == athr_l2c_b ||
		    hw->nic_type == athr_l2c_b2 ||
		    hw->nic_type == athr_l1d) {
			atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x00);
			if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data))
				goto out;
			phy_data |= 0x80;
			atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data);

			atl1c_write_phy_reg(hw, MII_DBG_ADDR, 0x3B);
			if (atl1c_read_phy_reg(hw, MII_DBG_DATA, &phy_data))
				goto out;
			phy_data &= 0xFFF7;
			atl1c_write_phy_reg(hw, MII_DBG_DATA, phy_data);
			udelay(20);
		}
	}

	/* maybe MAC-address is from BIOS */
	AT_READ_REG(hw, REG_MAC_STA_ADDR, &addr[0]);
	AT_READ_REG(hw, REG_MAC_STA_ADDR + 4, &addr[1]);

	eth_addr[2] = addr[0] >> 24;
	eth_addr[3] = addr[0] >> 16;
	eth_addr[4] = addr[0] >> 8;
	eth_addr[5] = addr[0];

	eth_addr[0] = addr[1] >> 8;
	eth_addr[1] = addr[1];

	if (is_valid_ether_addr(eth_addr)) {
		memcpy(hw->perm_mac_addr, eth_addr, ETH_ALEN);
		return 0;
	}

out:
	return -1;
}

static int atl1c_read_mac_addr(struct atl1c_hw *hw) {
	int err = 0;

	err = atl1c_get_permanent_address(hw);
	if (err)
		random_ether_addr(hw->perm_mac_addr);

	memcpy(hw->mac_addr, hw->perm_mac_addr, sizeof(hw->perm_mac_addr));
	return 0;
}

static const u16 atl1c_pay_load_size[] = {
	128, 256, 512, 1024, 2048, 4096,
};

static const u32 atl1c_default_msg = NETIF_MSG_DRV | NETIF_MSG_PROBE |
	NETIF_MSG_LINK | NETIF_MSG_TIMER | NETIF_MSG_IFDOWN | NETIF_MSG_IFUP;


/*
 * Set ASPM state.
 * Enable/disable L0s/L1 depend on link state.
 */
static void atl1c_set_aspm(struct atl1c_hw *hw, u16 link_speed)
{
	u32 pm_ctrl_data;
	u32 link_l1_timer;

	AT_READ_REG(hw, REG_PM_CTRL, &pm_ctrl_data);
	pm_ctrl_data &= ~(PM_CTRL_ASPM_L1_EN |
			  PM_CTRL_ASPM_L0S_EN |
			  PM_CTRL_MAC_ASPM_CHK);
	/* L1 timer */
	if (hw->nic_type == athr_l2c_b2 || hw->nic_type == athr_l1d_2) {
		pm_ctrl_data &= ~PMCTRL_TXL1_AFTER_L0S;
		link_l1_timer =
			link_speed == SPEED_1000 || link_speed == SPEED_100 ?
			L1D_PMCTRL_L1_ENTRY_TM_16US : 1;
		pm_ctrl_data = FIELD_SETX(pm_ctrl_data,
			L1D_PMCTRL_L1_ENTRY_TM, link_l1_timer);
	} else {
		link_l1_timer = hw->nic_type == athr_l2c_b ?
			L2CB1_PM_CTRL_L1_ENTRY_TM : L1C_PM_CTRL_L1_ENTRY_TM;
		if (link_speed != SPEED_1000 && link_speed != SPEED_100)
			link_l1_timer = 1;
		pm_ctrl_data = FIELD_SETX(pm_ctrl_data,
			PM_CTRL_L1_ENTRY_TIMER, link_l1_timer);
	}

	/* L0S/L1 enable */
	if ((hw->ctrl_flags & ATL1C_ASPM_L0S_SUPPORT) && link_speed != SPEED_0)
		pm_ctrl_data |= PM_CTRL_ASPM_L0S_EN | PM_CTRL_MAC_ASPM_CHK;
	if (hw->ctrl_flags & ATL1C_ASPM_L1_SUPPORT)
		pm_ctrl_data |= PM_CTRL_ASPM_L1_EN | PM_CTRL_MAC_ASPM_CHK;

	/* l2cb & l1d & l2cb2 & l1d2 */
	if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l1d ||
	    hw->nic_type == athr_l2c_b2 || hw->nic_type == athr_l1d_2) {
		pm_ctrl_data = FIELD_SETX(pm_ctrl_data,
			PM_CTRL_PM_REQ_TIMER, PM_CTRL_PM_REQ_TO_DEF);
		pm_ctrl_data |= PM_CTRL_RCVR_WT_TIMER |
				PM_CTRL_SERDES_PD_EX_L1 |
				PM_CTRL_CLK_SWH_L1;
		pm_ctrl_data &= ~(PM_CTRL_SERDES_L1_EN |
				  PM_CTRL_SERDES_PLL_L1_EN |
				  PM_CTRL_SERDES_BUFS_RX_L1_EN |
				  PM_CTRL_SA_DLY_EN |
				  PM_CTRL_HOTRST);
		/* disable l0s if link down or l2cb */
		if (link_speed == SPEED_0 || hw->nic_type == athr_l2c_b)
			pm_ctrl_data &= ~PM_CTRL_ASPM_L0S_EN;
	} else { /* l1c */
		pm_ctrl_data =
			FIELD_SETX(pm_ctrl_data, PM_CTRL_L1_ENTRY_TIMER, 0);
		if (link_speed != SPEED_0) {
			pm_ctrl_data |= PM_CTRL_SERDES_L1_EN |
					PM_CTRL_SERDES_PLL_L1_EN |
					PM_CTRL_SERDES_BUFS_RX_L1_EN;
			pm_ctrl_data &= ~(PM_CTRL_SERDES_PD_EX_L1 |
					  PM_CTRL_CLK_SWH_L1 |
					  PM_CTRL_ASPM_L0S_EN |
					  PM_CTRL_ASPM_L1_EN);
		} else { /* link down */
			pm_ctrl_data |= PM_CTRL_CLK_SWH_L1;
			pm_ctrl_data &= ~(PM_CTRL_SERDES_L1_EN |
					  PM_CTRL_SERDES_PLL_L1_EN |
					  PM_CTRL_SERDES_BUFS_RX_L1_EN |
					  PM_CTRL_ASPM_L0S_EN);
		}
	}
	AT_WRITE_REG(hw, REG_PM_CTRL, pm_ctrl_data);

	return;
}

static void atl1c_disable_l0s_l1(struct atl1c_hw *hw)
{
	u16 ctrl_flags = hw->ctrl_flags;

	hw->ctrl_flags &= ~(ATL1C_ASPM_L0S_SUPPORT | ATL1C_ASPM_L1_SUPPORT);
	atl1c_set_aspm(hw, SPEED_0);
	hw->ctrl_flags = ctrl_flags;
}

static void atl1c_pcie_patch(struct atl1c_hw *hw)
{
	u32 mst_data, data;

	/* pclk sel could switch to 25M */
	AT_READ_REG(hw, REG_MASTER_CTRL, &mst_data);
	mst_data &= ~MASTER_CTRL_CLK_SEL_DIS;
	AT_WRITE_REG(hw, REG_MASTER_CTRL, mst_data);

	/* WoL/PCIE related settings */
	if (hw->nic_type == athr_l1c || hw->nic_type == athr_l2c) {
		AT_READ_REG(hw, REG_PCIE_PHYMISC, &data);
		data |= PCIE_PHYMISC_FORCE_RCV_DET;
		AT_WRITE_REG(hw, REG_PCIE_PHYMISC, data);
	} else { /* new dev set bit5 of MASTER */
		if (!(mst_data & MASTER_CTRL_WAKEN_25M))
			AT_WRITE_REG(hw, REG_MASTER_CTRL,
				mst_data | MASTER_CTRL_WAKEN_25M);
	}
	/* aspm/PCIE setting only for l2cb 1.0 */
	if (hw->nic_type == athr_l2c_b && hw->revision_id == L2CB_V10) {
		AT_READ_REG(hw, REG_PCIE_PHYMISC2, &data);
		data = FIELD_SETX(data, PCIE_PHYMISC2_CDR_BW,
			L2CB1_PCIE_PHYMISC2_CDR_BW);
		data = FIELD_SETX(data, PCIE_PHYMISC2_L0S_TH,
			L2CB1_PCIE_PHYMISC2_L0S_TH);
		AT_WRITE_REG(hw, REG_PCIE_PHYMISC2, data);
		/* extend L1 sync timer */
		AT_READ_REG(hw, REG_LINK_CTRL, &data);
		data |= LINK_CTRL_EXT_SYNC;
		AT_WRITE_REG(hw, REG_LINK_CTRL, data);
	}
	/* l2cb 1.x & l1d 1.x */
	if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l1d) {
		AT_READ_REG(hw, REG_PM_CTRL, &data);
		data |= PM_CTRL_L0S_BUFSRX_EN;
		AT_WRITE_REG(hw, REG_PM_CTRL, data);
		/* clear vendor msg */
		AT_READ_REG(hw, REG_DMA_DBG, &data);
		AT_WRITE_REG(hw, REG_DMA_DBG, data & ~DMA_DBG_VENDOR_MSG);
	}
}

/* FIXME: no need any more ? */
/*
 * atl1c_init_pcie - init PCIE module
 */
static void atl1c_reset_pcie(struct atl1c_hw *hw, u32 flag)
{
	u32 data;
	u32 pci_cmd;
	struct pci_dev *pdev = hw->adapter->pdev;
	int pos;

	AT_READ_REG(hw, PCI_COMMAND, &pci_cmd);
	pci_cmd &= ~PCI_COMMAND_INTX_DISABLE;
	pci_cmd |= (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER |
		PCI_COMMAND_IO);
	AT_WRITE_REG(hw, PCI_COMMAND, pci_cmd);

	/*
	 * Clear any PowerSaveing Settings
	 */
	pci_enable_wake(pdev, PCI_D3hot, 0);
	pci_enable_wake(pdev, PCI_D3cold, 0);
	/* wol sts read-clear */
	AT_READ_REG(hw, REG_WOL_CTRL, &data);
	AT_WRITE_REG(hw, REG_WOL_CTRL, 0);

	/*
	 * Mask some pcie error bits
	 */
	pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_ERR);
	if (pos) {
		pci_read_config_dword(pdev, pos + PCI_ERR_UNCOR_SEVER, &data);
		data &= ~(PCI_ERR_UNC_DLP | PCI_ERR_UNC_FCP);
		pci_write_config_dword(pdev, pos + PCI_ERR_UNCOR_SEVER, data);
	}
	/* clear error status */
	pcie_capability_write_word(pdev, PCI_EXP_DEVSTA,
			PCI_EXP_DEVSTA_NFED |
			PCI_EXP_DEVSTA_FED |
			PCI_EXP_DEVSTA_CED |
			PCI_EXP_DEVSTA_URD);

	AT_READ_REG(hw, REG_LTSSM_ID_CTRL, &data);
	data &= ~LTSSM_ID_EN_WRO;
	AT_WRITE_REG(hw, REG_LTSSM_ID_CTRL, data);

	atl1c_pcie_patch(hw);
	if (flag & ATL1C_PCIE_L0S_L1_DISABLE)
		atl1c_disable_l0s_l1(hw);

	msleep(5);
}

static u32 atl1c_wait_until_idle(struct atl1c_hw *hw) {
	int timeout;
	u32 data;

	for (timeout = 0; timeout < AT_HW_MAX_IDLE_DELAY; timeout++) {
		AT_READ_REG(hw, REG_IDLE_STATUS, &data);
		if ((data & IDLE_STATUS_MASK) == 0)
			return 0;
		msleep(1);
	}
	return data;
}

static int atl1c_stop_mac(struct atl1c_hw *hw) {
	u32 data;

	AT_READ_REG(hw, REG_RXQ_CTRL, &data);
	data &= ~(RXQ1_CTRL_EN | RXQ2_CTRL_EN |
		  RXQ3_CTRL_EN | RXQ_CTRL_EN);
	AT_WRITE_REG(hw, REG_RXQ_CTRL, data);

	AT_READ_REG(hw, REG_TXQ_CTRL, &data);
	data &= ~TXQ_CTRL_EN;
	AT_WRITE_REG(hw, REG_TWSI_CTRL, data);

	atl1c_wait_until_idle(hw);

	AT_READ_REG(hw, REG_MAC_CTRL, &data);
	data &= ~(MAC_CTRL_TX_EN | MAC_CTRL_RX_EN);
	AT_WRITE_REG(hw, REG_MAC_CTRL, data);

	return (int)atl1c_wait_until_idle(hw);
}

static void atl1c_enable_rx_ctrl(struct atl1c_hw *hw) {
	u32 data;

	AT_READ_REG(hw, REG_RXQ_CTRL, &data);
	data |= RXQ1_CTRL_EN;
	data |= RXQ_CTRL_EN;
	AT_WRITE_REG(hw, REG_RXQ_CTRL, data);
}

static void atl1c_enable_tx_ctrl(struct atl1c_hw *hw) {
	u32 data;

	AT_READ_REG(hw, REG_TXQ_CTRL, &data);
	data |= TXQ_CTRL_EN;
	AT_WRITE_REG(hw, REG_TXQ_CTRL, data);
}

static void atl1c_setup_mac_ctrl(struct atl1c_adapter *adapter) {
	struct atl1c_hw *hw = &adapter->hw;
	u32 mac_ctrl_data;

	mac_ctrl_data = MAC_CTRL_TX_EN | MAC_CTRL_RX_EN | MAC_CTRL_PROMIS_EN;
//	mac_ctrl_data |= (MAC_CTRL_TX_FLOW | MAC_CTRL_RX_FLOW);

	if (adapter->link_duplex == FULL_DUPLEX) {
		hw->mac_duplex = true;
		mac_ctrl_data |= MAC_CTRL_DUPLX;
	}

	if (adapter->link_speed == SPEED_1000)
		hw->mac_speed = atl1c_mac_speed_1000;
	else
		hw->mac_speed = atl1c_mac_speed_10_100;

	mac_ctrl_data |= (hw->mac_speed & MAC_CTRL_SPEED_MASK) <<
			MAC_CTRL_SPEED_SHIFT;

	mac_ctrl_data |= (MAC_CTRL_ADD_CRC | MAC_CTRL_PAD);
	mac_ctrl_data |= ((hw->preamble_len & MAC_CTRL_PRMLEN_MASK) <<
			MAC_CTRL_PRMLEN_SHIFT);

	/* disable VLAN tag insert/strip */
	mac_ctrl_data &= ~MAC_CTRL_RMV_VLAN;

	mac_ctrl_data |= MAC_CTRL_BC_EN;
	mac_ctrl_data |= MAC_CTRL_SINGLE_PAUSE_EN;
	if (hw->nic_type == athr_l1d || hw->nic_type == athr_l2c_b2) {
		mac_ctrl_data |= MAC_CTRL_SPEED_MODE_SW;
		mac_ctrl_data |= MAC_CTRL_HASH_ALG_CRC32;
	}
	AT_WRITE_REG(hw, REG_MAC_CTRL, mac_ctrl_data);
}

static void atl1c_check_link_status(struct atl1c_adapter *adapter) {
	struct atl1c_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	struct pci_dev    *pdev   = adapter->pdev;
	int err;
	unsigned long flags;
	u16 speed, duplex, phy_data;

	spin_lock_irqsave(&adapter->mdio_lock, flags);
	/* MII_BMSR must read twise */
	atl1c_read_phy_reg(hw, MII_BMSR, &phy_data);
	atl1c_read_phy_reg(hw, MII_BMSR, &phy_data);
	spin_unlock_irqrestore(&adapter->mdio_lock, flags);

	if ((phy_data & BMSR_LSTATUS) == 0) {
		/* link down */
		if (netif_carrier_ok(netdev)) {
			hw->hibernate = true;
			if (atl1c_stop_mac(hw) != 0)
				if (netif_msg_hw(adapter))
					dev_warn(&pdev->dev,
						"stop mac failed\n");
			atl1c_set_aspm(hw, SPEED_0);
		}
		netif_carrier_off(netdev);
	} else {
		/* Link Up */
		hw->hibernate = false;
		spin_lock_irqsave(&adapter->mdio_lock, flags);
		err = atl1c_get_speed_and_duplex(hw, &speed, &duplex);
		spin_unlock_irqrestore(&adapter->mdio_lock, flags);
		if (unlikely(err))
			return;
		/* link result is our setting */
		if (adapter->link_speed != speed ||
		    adapter->link_duplex != duplex) {
			adapter->link_speed  = speed;
			adapter->link_duplex = duplex;
			atl1c_set_aspm(hw, speed);
			atl1c_enable_tx_ctrl(hw);
			atl1c_enable_rx_ctrl(hw);
			atl1c_setup_mac_ctrl(adapter);
			if (netif_msg_link(adapter))
				dev_info(&pdev->dev,
					"%s: %s NIC Link is Up<%d Mbps %s>\n",
					atl1c_driver_name, netdev->name,
					adapter->link_speed,
					adapter->link_duplex == FULL_DUPLEX ?
					"Full Duplex" : "Half Duplex");
		}
		if (!netif_carrier_ok(netdev))
			netif_carrier_on(netdev);
	}
}

static void atl1c_link_chg_event(struct atl1c_adapter *adapter) {
	struct net_device *netdev = adapter->netdev;
	struct pci_dev    *pdev   = adapter->pdev;
	u16 phy_data;
	u16 link_up;

	spin_lock(&adapter->mdio_lock);
	atl1c_read_phy_reg(&adapter->hw, MII_BMSR, &phy_data);
	atl1c_read_phy_reg(&adapter->hw, MII_BMSR, &phy_data);
	spin_unlock(&adapter->mdio_lock);
	link_up = phy_data & BMSR_LSTATUS;
	/* notify upper layer link down ASAP */
	if (!link_up) {
		if (netif_carrier_ok(netdev)) {
			/* old link state: Up */
			netif_carrier_off(netdev);
			if (netif_msg_link(adapter))
				dev_info(&pdev->dev,
					"%s: %s NIC Link is Down\n",
					atl1c_driver_name, netdev->name);
			adapter->link_speed = SPEED_0;
		}
	}

	adapter->work_event |= ATL1C_WORK_EVENT_LINK_CHANGE;
	schedule_work(&adapter->common_task);
}

static void atl1c_free_ring_resources(struct atl1c_adapter *adapter);
static int atl1c_close(struct net_device *);
static int atl1c_open(struct net_device *);

static void atl1c_common_task(struct work_struct *work) {
	struct atl1c_adapter *adapter = container_of(work, struct atl1c_adapter, common_task);
	struct net_device *netdev = adapter->netdev;

	if (adapter->work_event & ATL1C_WORK_EVENT_RESET) {
		rtnl_lock();
		atl1c_close(netdev);
		atl1c_open(netdev);
		rtnl_unlock();
		return;
	}

	if (adapter->work_event & ATL1C_WORK_EVENT_LINK_CHANGE)
		atl1c_check_link_status(adapter);
}

static void atl1c_tx_timeout(struct net_device *netdev, unsigned txqueue) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);

	printk("%s: tx timeout q%u\n", netdev->name, txqueue);
//	atl1c_dump(adapter, 0);
	/* Do the reset outside of interrupt context */
	adapter->work_event |= ATL1C_WORK_EVENT_RESET;
	schedule_work(&adapter->common_task);
}

static int atl1c_set_mac_addr(struct net_device *netdev, void *p) {
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr((const u8 *)addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	return 0;
}

static void atl1c_set_rxbufsize(struct atl1c_adapter *adapter,
				struct net_device *dev) {
	int mtu = dev->l2mtu;

	adapter->rx_buffer_len = mtu > AT_RX_BUF_SIZE ?
		roundup(mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN, 8) : AT_RX_BUF_SIZE;
}

static int atl1c_change_mtu(struct net_device *netdev, int new_mtu) {
	if (new_mtu < 68 || new_mtu > netdev->l2mtu) return -EINVAL;
	netdev->mtu = new_mtu;
	return 0;
}

static int atl1c_change_l2mtu(struct net_device *netdev, int new_mtu)
{
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	bool running = netif_running(netdev);

	if (netdev->l2mtu == new_mtu) {
	    return 0;
	}
	// limit l2mtu to safe value (jumbo - 100 should be ok)
	if (new_mtu < 1500 || new_mtu > (MAX_JUMBO_FRAME_SIZE - 100))
		return -EINVAL;

	if (running) {
		dev_close(netdev);
	}
	netdev->l2mtu = new_mtu;
	adapter->hw.max_frame_size = new_mtu;
	if (running) {
		dev_open(netdev, NULL);
	}
	return 0;
}

static int atl1c_mdio_read(struct net_device *netdev, int phy_id, int reg_num) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	u16 result;
	unsigned long flags;

	spin_lock_irqsave(&adapter->mdio_lock, flags);
	atl1c_read_phy_reg(&adapter->hw, reg_num & MDIO_REG_ADDR_MASK, &result);
	atl1c_read_phy_reg(&adapter->hw, reg_num & MDIO_REG_ADDR_MASK, &result);
	spin_unlock_irqrestore(&adapter->mdio_lock, flags);
	return result;
}

static void atl1c_mdio_write(struct net_device *netdev, int phy_id,
			     int reg_num, int val) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	unsigned long flags;

	spin_lock_irqsave(&adapter->mdio_lock, flags);
	atl1c_write_phy_reg(&adapter->hw, reg_num & MDIO_REG_ADDR_MASK, val);
	atl1c_write_phy_reg(&adapter->hw, reg_num & MDIO_REG_ADDR_MASK, val);
	spin_unlock_irqrestore(&adapter->mdio_lock, flags);
}

static int atl1c_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	return generic_mii_ioctl(&adapter->mii, if_mii(ifr), cmd, NULL);
}

static enum atl1c_nic_type atl1c_get_mac_type(struct pci_dev *pdev,
					      u8 __iomem *hw_addr)
{
	switch (pdev->device) {
	case PCI_DEVICE_ID_ATTANSIC_L2C:
		return athr_l2c;
	case PCI_DEVICE_ID_ATTANSIC_L1C:
		return athr_l1c;
	case PCI_DEVICE_ID_ATHEROS_L2C_B:
		return athr_l2c_b;
	case PCI_DEVICE_ID_ATHEROS_L2C_B2:
		return athr_l2c_b2;
	case PCI_DEVICE_ID_ATHEROS_L1D:
		return athr_l1d;
	case PCI_DEVICE_ID_ATHEROS_L1D_2_0:
		if (readl(hw_addr + REG_MT_MAGIC) == MT_MAGIC)
			return athr_mt;
		return athr_l1d_2;
	default:
		return athr_l1c;
	}
}

static int atl1c_setup_mac_funcs(struct atl1c_hw *hw) {
	u32 link_ctrl_data;

	AT_READ_REG(hw, REG_LINK_CTRL, &link_ctrl_data);

	hw->ctrl_flags = ATL1C_INTR_MODRT_ENABLE  |
			 ATL1C_TXQ_MODE_ENHANCE;
	hw->ctrl_flags |= ATL1C_ASPM_L0S_SUPPORT |
			  ATL1C_ASPM_L1_SUPPORT;
	hw->ctrl_flags |= ATL1C_ASPM_CTRL_MON;

	if (hw->nic_type == athr_l1c ||
	    hw->nic_type == athr_l1d ||
	    hw->nic_type == athr_l1d_2)
		hw->link_cap_flags |= ATL1C_LINK_CAP_1000M;
	return 0;
}

struct atl1c_platform_patch {
	u16 pci_did;
	u8  pci_revid;
	u16 subsystem_vid;
	u16 subsystem_did;
	u32 patch_flag;
#define ATL1C_LINK_PATCH	0x1
};
static const struct atl1c_platform_patch plats[] = {
{0x2060, 0xC1, 0x1019, 0x8152, 0x1},
{0x2060, 0xC1, 0x1019, 0x2060, 0x1},
{0x2060, 0xC1, 0x1019, 0xE000, 0x1},
{0x2062, 0xC0, 0x1019, 0x8152, 0x1},
{0x2062, 0xC0, 0x1019, 0x2062, 0x1},
{0x2062, 0xC0, 0x1458, 0xE000, 0x1},
{0x2062, 0xC1, 0x1019, 0x8152, 0x1},
{0x2062, 0xC1, 0x1019, 0x2062, 0x1},
{0x2062, 0xC1, 0x1458, 0xE000, 0x1},
{0x2062, 0xC1, 0x1565, 0x2802, 0x1},
{0x2062, 0xC1, 0x1565, 0x2801, 0x1},
{0x1073, 0xC0, 0x1019, 0x8151, 0x1},
{0x1073, 0xC0, 0x1019, 0x1073, 0x1},
{0x1073, 0xC0, 0x1458, 0xE000, 0x1},
{0x1083, 0xC0, 0x1458, 0xE000, 0x1},
{0x1083, 0xC0, 0x1019, 0x8151, 0x1},
{0x1083, 0xC0, 0x1019, 0x1083, 0x1},
{0x1083, 0xC0, 0x1462, 0x7680, 0x1},
{0x1083, 0xC0, 0x1565, 0x2803, 0x1},
{0},
};

static void atl1c_patch_assign(struct atl1c_hw *hw)
{
	struct pci_dev	*pdev = hw->adapter->pdev;
	u32 misc_ctrl;
	int i = 0;

	hw->msi_lnkpatch = false;

	while (plats[i].pci_did != 0) {
		if (plats[i].pci_did == hw->device_id &&
		    plats[i].pci_revid == hw->revision_id &&
		    plats[i].subsystem_vid == hw->subsystem_vendor_id &&
		    plats[i].subsystem_did == hw->subsystem_id) {
			if (plats[i].patch_flag & ATL1C_LINK_PATCH)
				hw->msi_lnkpatch = true;
		}
		i++;
	}

	if (hw->device_id == PCI_DEVICE_ID_ATHEROS_L2C_B2 &&
	    hw->revision_id == L2CB_V21) {
		/* config access mode */
		pci_write_config_dword(pdev, REG_PCIE_IND_ACC_ADDR,
				       REG_PCIE_DEV_MISC_CTRL);
		pci_read_config_dword(pdev, REG_PCIE_IND_ACC_DATA, &misc_ctrl);
		misc_ctrl &= ~0x100;
		pci_write_config_dword(pdev, REG_PCIE_IND_ACC_ADDR,
				       REG_PCIE_DEV_MISC_CTRL);
		pci_write_config_dword(pdev, REG_PCIE_IND_ACC_DATA, misc_ctrl);
	}
}

static int atl1c_sw_init(struct atl1c_adapter *adapter) {
	struct atl1c_hw *hw   = &adapter->hw;
	struct pci_dev	*pdev = adapter->pdev;
	u32 revision;

	adapter->link_speed = SPEED_0;
	adapter->link_duplex = FULL_DUPLEX;
	adapter->tpd_ring[0].count = 1024;
	adapter->tpd_ring[0].stop_threshold = FP_NAPI_BUDGET + MAX_SKB_FRAGS + 5;
	adapter->rfd_ring[0].count = 512;

	hw->vendor_id = pdev->vendor;
	hw->device_id = pdev->device;
	hw->subsystem_vendor_id = pdev->subsystem_vendor;
	hw->subsystem_id = pdev->subsystem_device;
	pci_read_config_dword(pdev, PCI_CLASS_REVISION, &revision);
	hw->revision_id = revision & 0xFF;

	/* before link up, we assume hibernate is true */
	hw->hibernate = true;
	hw->media_type = MEDIA_TYPE_AUTO_SENSOR;
	if (atl1c_setup_mac_funcs(hw) != 0) {
		dev_err(&pdev->dev, "set mac function pointers failed\n");
		return -1;
	}
	atl1c_patch_assign(hw);

	hw->phy_configured = false;
	hw->preamble_len = 7;
	hw->max_frame_size = adapter->netdev->l2mtu;
	hw->autoneg_advertised = ADVERTISED_Autoneg;
	hw->indirect_tab = 0xE4E4E4E4;
	hw->base_cpu = 0;

	hw->ict = 50000;		/* 100ms */
	hw->smb_timer = 200000;	  	/* 400ms */
	hw->cmb_tpd = 4;
	hw->cmb_tx_timer = 1;		/* 2 us  */
#ifdef __powerpc__
#if defined(CONFIG_RB1200)
	hw->rx_imt = 10000;
	hw->tx_imt = 10000;
#else
	hw->rx_imt = 1000;
	hw->tx_imt = 5000;
#endif
#else
	hw->rx_imt = 10;
	hw->tx_imt = 10;
#endif

	hw->tpd_burst = 5;
	hw->rfd_burst = 8;
	hw->dma_order = atl1c_dma_ord_out;
	hw->dmar_block = atl1c_dma_req_1024;
	hw->dmaw_block = atl1c_dma_req_1024;
	hw->dmar_dly_cnt = 15;
	hw->dmaw_dly_cnt = 4;

	atl1c_set_rxbufsize(adapter, adapter->netdev);
	spin_lock_init(&adapter->mdio_lock);
	spin_lock_init(&adapter->hw.intr_mask_lock);
	return 0;
}

static inline void atl1c_clean_buffer(struct atl1c_adapter *adapter,
				      struct atl1c_buffer *buffer_info, bool tx) {

	if (buffer_info->dma) {
		pci_unmap_page(adapter->pdev, buffer_info->dma,
			       buffer_info->length, tx ? PCI_DMA_TODEVICE : PCI_DMA_FROMDEVICE);
		buffer_info->dma = 0;

		if (buffer_info->buf) {
			if (buffer_info->fpb) {
				__skb_bin_put_buffer_noinvalidate(buffer_info->buf);
			}
			else {
				skb_bin_recycle(buffer_info->buf);
			}
			buffer_info->buf = NULL;
		}
	}
	buffer_info->fpb = false;
}

static void atl1c_clean_tx_ring(struct atl1c_adapter *adapter, u32 queue) {
	struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[queue];
	u16 index;

	local_bh_disable();
	for (index = 0; index < tpd_ring->count; index++) {
		atl1c_clean_buffer(adapter, &tpd_ring->buffer_info[index], true);
	}
	local_bh_enable();

	memset(tpd_ring->desc, 0, sizeof(struct atl1c_tpd_desc) * tpd_ring->count);
	tpd_ring->tail = 0;
	tpd_ring->head = 0;
}

static void atl1c_clean_rx_ring(struct atl1c_adapter *adapter) {
	struct atl1c_rfd_ring *rfd_ring = adapter->rfd_ring;
	struct atl1c_rrd_ring *rrd_ring = adapter->rrd_ring;
	struct atl1c_buffer *buffer_info;
	int i, j;

	local_bh_disable();
	for (i = 0; i < adapter->rx_queue_count; i++) {
		for (j = 0; j < rfd_ring[i].count; j++) {
			buffer_info = &rfd_ring[i].buffer_info[j];
			atl1c_clean_buffer(adapter, buffer_info, false);
		}
		memset(rfd_ring[i].desc, 0, rfd_ring[i].size);
		rfd_ring[i].head = 0;
		rrd_ring[i].tail = 0;
	}
	local_bh_enable();
}

static void atl1c_init_ring_ptrs(struct atl1c_adapter *adapter) {
	struct atl1c_tpd_ring *tpd_ring = adapter->tpd_ring;
	struct atl1c_rfd_ring *rfd_ring = adapter->rfd_ring;
	struct atl1c_rrd_ring *rrd_ring = adapter->rrd_ring;
	int i;
	for (i = 0; i < adapter->tx_queue_count; i++) {
		tpd_ring[i].mask = tpd_ring[i].count - 1;
		tpd_ring[i].head = 0;
		tpd_ring[i].tail = 0;
	}
	for (i = 0; i < adapter->rx_queue_count; i++) {
		rfd_ring[i].mask = rfd_ring[i].count - 1;
		rrd_ring[i].mask = rrd_ring[i].count - 1;
		rfd_ring[i].head = 0;
		rrd_ring[i].tail = 0;
	}
}

static void atl1c_free_ring_resources(struct atl1c_adapter *adapter) {
	struct pci_dev *pdev = adapter->pdev;

	pci_free_consistent(pdev, adapter->ring_header.size,
					adapter->ring_header.desc,
					adapter->ring_header.dma);
	adapter->ring_header.desc = NULL;

	if (adapter->tpd_ring[0].buffer_info) {
		kfree(adapter->tpd_ring[0].buffer_info);
		adapter->tpd_ring[0].buffer_info = NULL;
	}
}

static int atl1c_setup_ring_resources(struct atl1c_adapter *adapter) {
	struct pci_dev *pdev = adapter->pdev;
	struct atl1c_tpd_ring *tpd_ring = adapter->tpd_ring;
	struct atl1c_rfd_ring *rfd_ring = adapter->rfd_ring;
	struct atl1c_rrd_ring *rrd_ring = adapter->rrd_ring;
	struct atl1c_ring_header *ring_header = &adapter->ring_header;
	int size;
	int i;
	int count = 0;
	u32 offset = 0;
	int tqc = adapter->tx_queue_count;
	int rqc = adapter->rx_queue_count;

	/* even tough only one tpd queue is actually used, the "high"
	 * priority tpd queue also gets initialized */
	if (tqc == 1)
		tqc = 2;

	rrd_ring[0].count = rfd_ring[0].count;

	size = sizeof(struct atl1c_buffer) * (tpd_ring->count * tqc +
		rfd_ring->count * rqc);
	tpd_ring->buffer_info = kzalloc(size, GFP_KERNEL);
	if (unlikely(!tpd_ring->buffer_info)) {
		dev_err(&pdev->dev, "kzalloc failed, size = %d\n",
			size);
		goto err_nomem;
	}
	for (i = 0; i < tqc; i++) {
		tpd_ring[i].adapter = adapter;
		tpd_ring[i].num = i;
		tpd_ring[i].count = tpd_ring[0].count;
		tpd_ring[i].stop_threshold = tpd_ring[0].stop_threshold;
		tpd_ring[i].buffer_info =
			(struct atl1c_buffer *) (tpd_ring->buffer_info + count);
		count += tpd_ring[i].count;
	}

	for (i = 0; i < rqc; i++) {
		rrd_ring[i].adapter = adapter;
		rrd_ring[i].num = i;
		rrd_ring[i].count = rfd_ring[0].count;
		rfd_ring[i].count = rfd_ring[0].count;
		rfd_ring[i].buffer_info =
			(struct atl1c_buffer *) (tpd_ring->buffer_info + count);
		count += rfd_ring[i].count;
	}
	/*
	 * real ring DMA buffer
	 * each ring/block may need up to 8 bytes for alignment, hence the
	 * additional bytes tacked onto the end.
	 */
	ring_header->size = size =
		sizeof(struct atl1c_tpd_desc) * tpd_ring->count * tqc +
		sizeof(struct atl1c_rx_free_desc) * rfd_ring->count * rqc +
		sizeof(struct atl1c_recv_ret_status) * rfd_ring->count * rqc +
		sizeof(struct atl1c_hw_stats) +
		8 * 4;

	ring_header->desc = pci_alloc_consistent(pdev, ring_header->size,
				&ring_header->dma);
	if (unlikely(!ring_header->desc)) {
		dev_err(&pdev->dev, "pci_alloc_consistend failed\n");
		goto err_nomem;
	}
	memset(ring_header->desc, 0, ring_header->size);

	tpd_ring[0].dma = roundup(ring_header->dma, 8);
	offset = tpd_ring[0].dma - ring_header->dma;
	for (i = 0; i < tqc; i++) {
		tpd_ring[i].dma = ring_header->dma + offset;
		tpd_ring[i].desc = ring_header->desc + offset;
		tpd_ring[i].size =
			sizeof(struct atl1c_tpd_desc) * tpd_ring[i].count;
		offset += roundup(tpd_ring[i].size, 8);
	}

	for (i = 0; i < rqc; i++) {
		rfd_ring[i].dma = ring_header->dma + offset;
		rfd_ring[i].desc = ring_header->desc + offset;
		rfd_ring[i].size = sizeof(struct atl1c_rx_free_desc) *
				rfd_ring[i].count;
		offset += roundup(rfd_ring[i].size, 8);
	}

	for (i = 0; i < rqc; i++) {
		rrd_ring[i].dma = ring_header->dma + offset;
		rrd_ring[i].desc = ring_header->desc + offset;
		rrd_ring[i].size = sizeof(struct atl1c_recv_ret_status) *
				rrd_ring[i].count;
		offset += roundup(rrd_ring[i].size, 8);
	}

	adapter->smb.dma = ring_header->dma + offset;
	adapter->smb.smb = (u8 *)ring_header->desc + offset;
	return 0;

err_nomem:
	kfree(tpd_ring->buffer_info);
	return -ENOMEM;
}

static void atl1c_configure_des_ring(struct atl1c_adapter *adapter) {
	struct atl1c_hw *hw = &adapter->hw;
	struct atl1c_rfd_ring *rfd_ring = (struct atl1c_rfd_ring *)
				adapter->rfd_ring;
	struct atl1c_rrd_ring *rrd_ring = (struct atl1c_rrd_ring *)
				adapter->rrd_ring;
	struct atl1c_tpd_ring *tpd_ring = (struct atl1c_tpd_ring *)
				adapter->tpd_ring;
	struct atl1c_cmb *cmb = (struct atl1c_cmb *) &adapter->cmb;
	struct atl1c_smb *smb = (struct atl1c_smb *) &adapter->smb;
	int i;
	int tx_queue_count = adapter->tx_queue_count;

	if (tx_queue_count == 1)
		tx_queue_count = 2;

	/* TPD */
	AT_WRITE_REG(hw, REG_TX_BASE_ADDR_HI,
			(u32)((tpd_ring[0].dma &
				AT_DMA_HI_ADDR_MASK) >> 32));
	for (i = 0; i < tx_queue_count; i++) {
		AT_WRITE_REG(hw, atl1c_qregs[i].tpd_addr_lo,
			     (u32)(tpd_ring[i].dma & AT_DMA_LO_ADDR_MASK));
	}
	AT_WRITE_REG(hw, REG_TPD_RING_SIZE,
			(u32)(tpd_ring[0].count & TPD_RING_SIZE_MASK));


	/* RFD */
	AT_WRITE_REG(hw, REG_RX_BASE_ADDR_HI,
			(u32)((rfd_ring[0].dma & AT_DMA_HI_ADDR_MASK) >> 32));
	for (i = 0; i < adapter->rx_queue_count; i++) {
		AT_WRITE_REG(hw, atl1c_qregs[i].rfd_addr_lo,
			     (u32)(rfd_ring[i].dma & AT_DMA_LO_ADDR_MASK));
	}

	AT_WRITE_REG(hw, REG_RFD_RING_SIZE,
			rfd_ring[0].count & RFD_RING_SIZE_MASK);
	AT_WRITE_REG(hw, REG_RX_BUF_SIZE,
			adapter->rx_buffer_len & RX_BUF_SIZE_MASK);

	/* RRD */
	for (i = 0; i < adapter->rx_queue_count; i++) {
		AT_WRITE_REG(hw, atl1c_qregs[i].rrd_addr_lo,
			     (u32)(rrd_ring[i].dma & AT_DMA_LO_ADDR_MASK));
	}
	AT_WRITE_REG(hw, REG_RRD_RING_SIZE,
			(rrd_ring[0].count & RRD_RING_SIZE_MASK));

	/* CMB */
	AT_WRITE_REG(hw, REG_CMB_BASE_ADDR_LO, cmb->dma & AT_DMA_LO_ADDR_MASK);

	/* SMB */
	AT_WRITE_REG(hw, REG_SMB_BASE_ADDR_HI,
			(u32)((smb->dma & AT_DMA_HI_ADDR_MASK) >> 32));
	AT_WRITE_REG(hw, REG_SMB_BASE_ADDR_LO,
			(u32)(smb->dma & AT_DMA_LO_ADDR_MASK));
	/* Load all of base address above */
	AT_WRITE_REG(hw, REG_LOAD_PTR, 1);
}

static void atl1c_configure_tx(struct atl1c_adapter *adapter) {
	struct atl1c_hw *hw = &adapter->hw;
	u32 dev_ctrl_data;
	u32 max_pay_load;
	u16 tx_offload_thresh;
	u32 txq_ctrl_data;

	tx_offload_thresh = MAX_TX_OFFLOAD_THRESH;
	AT_WRITE_REG(hw, REG_TX_TSO_OFFLOAD_THRESH,
		(tx_offload_thresh >> 3) & TX_TSO_OFFLOAD_THRESH_MASK);
	AT_READ_REG(hw, REG_DEVICE_CTRL, &dev_ctrl_data);
	max_pay_load  = (dev_ctrl_data >> DEVICE_CTRL_MAX_PAYLOAD_SHIFT) &
			DEVICE_CTRL_MAX_PAYLOAD_MASK;
	hw->dmaw_block = min(max_pay_load, hw->dmaw_block);
	max_pay_load  = (dev_ctrl_data >> DEVICE_CTRL_MAX_RREQ_SZ_SHIFT) &
			DEVICE_CTRL_MAX_RREQ_SZ_MASK;
	hw->dmar_block = min(max_pay_load, hw->dmar_block);

	txq_ctrl_data = (hw->tpd_burst & TXQ_NUM_TPD_BURST_MASK) <<
			TXQ_NUM_TPD_BURST_SHIFT;
	if (hw->ctrl_flags & ATL1C_TXQ_MODE_ENHANCE)
		txq_ctrl_data |= TXQ_CTRL_ENH_MODE;
	txq_ctrl_data |= (atl1c_pay_load_size[hw->dmar_block] &
			TXQ_TXF_BURST_NUM_MASK) << TXQ_TXF_BURST_NUM_SHIFT;

	AT_WRITE_REG(hw, REG_TXQ_CTRL, txq_ctrl_data);
}

static void atl1c_configure_rx(struct atl1c_adapter *adapter) {
	struct atl1c_hw *hw = &adapter->hw;
	u32 rxq_ctrl_data;

	rxq_ctrl_data = (hw->rfd_burst & RXQ_RFD_BURST_NUM_MASK) <<
			RXQ_RFD_BURST_NUM_SHIFT;

	if (hw->ctrl_flags & ATL1C_RX_IPV6_CHKSUM)
		rxq_ctrl_data |= IPV6_CHKSUM_CTRL_EN;

	/* aspm for gigabit */
	if (hw->nic_type != athr_l1d_2 && (hw->device_id & 1) != 0)
		rxq_ctrl_data = FIELD_SETX(rxq_ctrl_data, ASPM_THRUPUT_LIMIT,
			ASPM_THRUPUT_LIMIT_100M);

	AT_WRITE_REG(hw, REG_RXQ_CTRL, rxq_ctrl_data);
}

static void atl1c_configure_dma(struct atl1c_adapter *adapter) {
	struct atl1c_hw *hw = &adapter->hw;
	u32 dma_ctrl_data;

	dma_ctrl_data = DMA_CTRL_DMAR_REQ_PRI;
	if (hw->ctrl_flags & ATL1C_CMB_ENABLE)
		dma_ctrl_data |= DMA_CTRL_CMB_EN;

	switch (hw->dma_order) {
	case atl1c_dma_ord_in:
		dma_ctrl_data |= DMA_CTRL_DMAR_IN_ORDER;
		break;
	case atl1c_dma_ord_enh:
		dma_ctrl_data |= DMA_CTRL_DMAR_ENH_ORDER;
		break;
	case atl1c_dma_ord_out:
		dma_ctrl_data |= DMA_CTRL_DMAR_OUT_ORDER;
		break;
	default:
		break;
	}

	dma_ctrl_data |= (((u32)hw->dmar_block) & DMA_CTRL_DMAR_BURST_LEN_MASK)
		<< DMA_CTRL_DMAR_BURST_LEN_SHIFT;
	dma_ctrl_data |= (((u32)hw->dmaw_block) & DMA_CTRL_DMAW_BURST_LEN_MASK)
		<< DMA_CTRL_DMAW_BURST_LEN_SHIFT;
	dma_ctrl_data |= (((u32)hw->dmar_dly_cnt) & DMA_CTRL_DMAR_DLY_CNT_MASK)
		<< DMA_CTRL_DMAR_DLY_CNT_SHIFT;
	dma_ctrl_data |= (((u32)hw->dmaw_dly_cnt) & DMA_CTRL_DMAW_DLY_CNT_MASK)
		<< DMA_CTRL_DMAW_DLY_CNT_SHIFT;

	AT_WRITE_REG(hw, REG_DMA_CTRL, dma_ctrl_data);
}

static int atl1c_reset_mac(struct atl1c_hw *hw) {
	struct atl1c_adapter *adapter = (struct atl1c_adapter *)hw->adapter;
	struct pci_dev *pdev = adapter->pdev;
	int ret;
	u32 ctrl_data = 0;

	AT_WRITE_REG(hw, REG_IMR, 0);
	AT_WRITE_REG(hw, REG_ISR, ISR_DIS_INT);

	ret = atl1c_stop_mac(hw);
	if (ret)
		return ret;
	/*
	 * Issue Soft Reset to the MAC.  This will reset the chip's
	 * transmit, receive, DMA.  It will not effect
	 * the current PCI configuration.  The global reset bit is self-
	 * clearing, and should clear within a microsecond.
	 */
	AT_READ_REG(hw, REG_MASTER_CTRL, &ctrl_data);
	ctrl_data |= MASTER_CTRL_OOB_DIS;
	AT_WRITE_REG(hw, REG_MASTER_CTRL, ctrl_data | MASTER_CTRL_SOFT_RST);

	AT_WRITE_FLUSH(hw);
	msleep(10);
	/* Wait at least 10ms for All module to be Idle */

	if (atl1c_wait_until_idle(hw)) {
		dev_err(&pdev->dev,
			"MAC state machine can't be idle since"
			" disabled for 10ms second\n");
		return -1;
	}
	AT_WRITE_REG(hw, REG_MASTER_CTRL, ctrl_data);

	/* driver control speed/duplex */
	AT_READ_REG(hw, REG_MAC_CTRL, &ctrl_data);
	AT_WRITE_REG(hw, REG_MAC_CTRL, ctrl_data | MAC_CTRL_SPEED_MODE_SW);

	/* clk switch setting */
	AT_READ_REG(hw, REG_SERDES, &ctrl_data);
	switch (hw->nic_type) {
	case athr_l2c_b:
		ctrl_data &= ~(SERDES_PHY_CLK_SLOWDOWN |
				SERDES_MAC_CLK_SLOWDOWN);
		AT_WRITE_REG(hw, REG_SERDES, ctrl_data);
		break;
	case athr_l2c_b2:
	case athr_l1d_2:
		ctrl_data |= SERDES_PHY_CLK_SLOWDOWN | SERDES_MAC_CLK_SLOWDOWN;
		AT_WRITE_REG(hw, REG_SERDES, ctrl_data);
		break;
	default:
		break;
	}

	return 0;
}


static int atl1c_configure(struct atl1c_adapter *adapter) {
	struct atl1c_hw *hw = &adapter->hw;
	u32 master_ctrl_data = 0;
	u32 intr_modrt_data;

	if (adapter->hw.nic_type == athr_mt) {
		u32 mode;
		AT_READ_REG(&adapter->hw, REG_MT_MODE, &mode);
		if (adapter->rx_queue_count == 4) {
			mode |= MT_MODE_4Q;
		}
		else {
			mode &= ~MT_MODE_4Q;
		}
		AT_WRITE_REG(&adapter->hw, REG_MT_MODE, mode);
	}

	/* clear interrupt status */
	AT_WRITE_REG(hw, REG_ISR, 0xFFFFFFFF);
	/*  Clear any WOL status */
	AT_WRITE_REG(hw, REG_WOL_CTRL, 0);
	/* set Interrupt Clear Timer
	 * HW will enable self to assert interrupt event to system after
	 * waiting x-time for software to notify it accept interrupt.
	 */
	AT_WRITE_REG(hw, REG_INT_RETRIG_TIMER, hw->ict & INT_RETRIG_TIMER_MASK);

	atl1c_configure_des_ring(adapter);

	AT_READ_REG(hw, REG_MASTER_CTRL, &master_ctrl_data);
	master_ctrl_data &= ~(MASTER_CTRL_TX_ITIMER_EN |
			      MASTER_CTRL_RX_ITIMER_EN |
			      MASTER_CTRL_INT_RDCLR);

	if (hw->ctrl_flags & ATL1C_INTR_MODRT_ENABLE) {
		intr_modrt_data = (hw->tx_imt & IRQ_MODRT_TIMER_MASK) <<
					IRQ_MODRT_TX_TIMER_SHIFT;
		intr_modrt_data |= (hw->rx_imt & IRQ_MODRT_TIMER_MASK) <<
					IRQ_MODRT_RX_TIMER_SHIFT;
		AT_WRITE_REG(hw, REG_IRQ_MODRT_TIMER_INIT, intr_modrt_data);
		master_ctrl_data |=
			MASTER_CTRL_TX_ITIMER_EN | MASTER_CTRL_RX_ITIMER_EN;
	}

//	if (hw->ctrl_flags & ATL1C_INTR_CLEAR_ON_READ)
		master_ctrl_data |= MASTER_CTRL_INT_RDCLR;

	AT_WRITE_REG(hw, REG_MASTER_CTRL, master_ctrl_data);

	if (hw->ctrl_flags & ATL1C_CMB_ENABLE) {
		AT_WRITE_REG(hw, REG_CMB_TPD_THRESH,
			hw->cmb_tpd & CMB_TPD_THRESH_MASK);
		AT_WRITE_REG(hw, REG_CMB_TX_TIMER,
			hw->cmb_tx_timer & CMB_TX_TIMER_MASK);
	}

	AT_WRITE_REG(hw, REG_SMB_STAT_TIMER,
		hw->smb_timer & SMB_STAT_TIMER_MASK);
	/* set MTU */
	AT_WRITE_REG(hw, REG_MTU, hw->max_frame_size + ETH_HLEN +
			VLAN_HLEN + ETH_FCS_LEN);
	/* HDS, disable */
	AT_WRITE_REG(hw, REG_HDS_CTRL, 0);

	atl1c_configure_tx(adapter);
	atl1c_configure_rx(adapter);
	AT_WRITE_REG(hw, REG_IDT_TABLE, hw->indirect_tab);
	AT_WRITE_REG(hw, REG_BASE_CPU_NUMBER, hw->base_cpu);
	atl1c_configure_dma(adapter);

	AT_WRITE_REG(hw, REG_LED_CONFIG, 0xcf01cf31);

	return 0;
}

static void atl1c_update_hw_stats(struct atl1c_adapter *adapter) {
	u16 hw_reg_addr = 0;
	unsigned *stats_item = NULL;
	u32 data;

	/* update rx status */
	hw_reg_addr = REG_MAC_RX_STATUS_BIN;
	stats_item  = &adapter->hw_stats.rx_ok;
	while (hw_reg_addr <= REG_MAC_RX_STATUS_END) {
		AT_READ_REG(&adapter->hw, hw_reg_addr, &data);
		*stats_item = data;
		stats_item++;
		hw_reg_addr += 4;
	}
	/* update tx status */
	hw_reg_addr = REG_MAC_TX_STATUS_BIN;
	stats_item  = &adapter->hw_stats.tx_ok;
	while (hw_reg_addr <= REG_MAC_TX_STATUS_END) {
		AT_READ_REG(&adapter->hw, hw_reg_addr, &data);
		*stats_item = data;
		stats_item++;
		hw_reg_addr += 4;
	}
}

static void atl1c_update_stats(struct atl1c_adapter *adapter) {
	if (netif_carrier_ok(adapter->netdev)) {
		atl1c_update_hw_stats(adapter);
	}
	eth_stats_add_stats(
		&adapter->xstats,
		eth_stat_items_atl1c,
		(unsigned *)&adapter->hw_stats);
}

static int atl1c_clean_tx(struct atl1c_adapter *adapter, u32 queue) {
	struct atl1c_tpd_ring *ring = &adapter->tpd_ring[queue];
	struct netdev_queue *txq = netdev_get_tx_queue(adapter->netdev, queue);
	int work_done = 0;
	u16 hw_tail = AT_READ_REGW(&adapter->hw, atl1c_qregs[queue].tpd_cons);
	u16 tail = ring->tail;
	while (tail != hw_tail) {
		struct atl1c_buffer *buffer_info = &ring->buffer_info[tail];
		atl1c_clean_buffer(adapter, buffer_info, true);
		tail = (tail + 1) & ring->mask;
		++work_done;
	}

	if (work_done) {
		spin_lock(&ring->tx_lock);
		ring->tail = tail;
		if (atl1c_tpd_avail(adapter, queue) > ring->stop_threshold && netif_tx_queue_stopped(txq)) {
			netif_tx_wake_queue(txq);
		}
		spin_unlock(&ring->tx_lock);
	}
	return work_done;
}

#ifdef STATS
static void atl1c_stats(struct atl1c_adapter *mac) {
	if (!mac->stats_last_jiffies) {
		mac->stats_last_jiffies = jiffies;
	}

	if ((long)(mac->stats_last_jiffies + HZ - jiffies) > 0) {
		return;
	}
	mac->stats_last_jiffies = jiffies;

	printk("%s@%u %lx jf:%d int:%u\n",
	       mac->netdev->name,
	       smp_processor_id(),
	       in_interrupt(),
	       (int)mac->stats_last_jiffies,
	       mac->stats_intr);
	mac->stats_intr = 0;
	int i;
	for (i = 0; i < mac->tx_queue_count; ++i) {
		struct atl1c_rrd_ring *rr = &mac->rrd_ring[i];
		struct atl1c_tpd_ring *tr = &mac->tpd_ring[i];
		printk("%s q%u tx int:%u poll:%u:%u xmit:%u commit:%u tx:%u stop:%u	rx int:%u poll:%u:%u rx:%u\n",
		       mac->netdev->name,
		       i, tr->stats_intr, tr->stats_poll, tr->stats_poll_done,
		       tr->stats_xmit, tr->stats_xmit_commit, tr->stats_tx,
		       tr->stats_tx_queue_stop,
		       rr->stats_intr, rr->stats_poll, rr->stats_poll_done, rr->stats_rx);

		tr->stats_intr = 0;
		tr->stats_poll = 0;
		tr->stats_poll_done = 0;
		tr->stats_xmit = 0;
		tr->stats_xmit_commit = 0;
		tr->stats_tx = 0;
		tr->stats_tx_queue_stop = 0;

		rr->stats_intr = 0;
		rr->stats_poll = 0;
		rr->stats_poll_done = 0;
		rr->stats_rx = 0;
	}

}
#endif


static void atl1c_intr_rx_tx(struct atl1c_adapter *adapter, u32 status)
{
	struct atl1c_hw *hw = &adapter->hw;
	int i;
	u32 intr_mask;

	spin_lock(&hw->intr_mask_lock);
	intr_mask = hw->intr_mask;
	for (i = 0; i < adapter->rx_queue_count; ++i) {
		struct atl1c_rrd_ring *r = &adapter->rrd_ring[i];
		if (!(status & atl1c_qregs[i].rx_isr))
			continue;
		if (napi_schedule_prep(&r->napi)) {
#ifdef STATS
		++r->stats_intr;
#endif
			intr_mask &= ~atl1c_qregs[i].rx_isr;
			smp_call_function_single_async(r->target_cpu, &r->csd);
//			__napi_schedule(&adapter->rrd_ring[i].napi);
		}
	}
	for (i = 0; i < adapter->tx_queue_count; ++i) {
		struct atl1c_tpd_ring *r = &adapter->tpd_ring[i];
		if (!(status & atl1c_qregs[i].tx_isr))
			continue;
		if (napi_schedule_prep(&r->napi)) {
#ifdef STATS
		++r->stats_intr;
#endif
			intr_mask &= ~atl1c_qregs[i].tx_isr;
			smp_call_function_single_async(r->target_cpu, &r->csd);
//			__napi_schedule(&adapter->tpd_ring[i].napi);
		}
	}

	if (hw->intr_mask != intr_mask) {
		hw->intr_mask = intr_mask;
		AT_WRITE_REG(hw, REG_IMR, hw->intr_mask);
	}
	spin_unlock(&hw->intr_mask_lock);
}

static irqreturn_t atl1c_intr(int irq, void *data) {
	struct net_device *netdev  = data;
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	struct atl1c_hw *hw = &adapter->hw;
	u32 status;
	AT_READ_REG(hw, REG_ISR, &status);
#ifdef STATS
	++adapter->stats_intr;
#endif
	if (status & (ISR_RX_PKT | ISR_TX_PKT)) {
		atl1c_intr_rx_tx(adapter, status);
	}

	if (status & ISR_ERROR) {
		printk("%s: hardware error (status = 0x%x)\n",
		       netdev->name, status);
		spin_lock(&hw->intr_mask_lock);
		hw->intr_mask = 0;
		AT_WRITE_REG(hw, REG_IMR, 0);
		spin_unlock(&hw->intr_mask_lock);

		adapter->work_event |= ATL1C_WORK_EVENT_RESET;
		schedule_work(&adapter->common_task);
	}

	if (status & (ISR_GPHY | ISR_MANUAL)) {
		if (status & ISR_GPHY) {
			u16 phy_data;
			spin_lock(&adapter->mdio_lock);
			atl1c_read_phy_reg(&adapter->hw, MII_ISR, &phy_data);
			spin_unlock(&adapter->mdio_lock);
		}
		adapter->net_stats.tx_carrier_errors++;
		atl1c_link_chg_event(adapter);
	}

	AT_WRITE_REG(&adapter->hw, REG_ISR, status);
	return IRQ_HANDLED;
}

static int atl1c_alloc_rx_buffer(struct atl1c_adapter *adapter, u32 queue) {
	struct atl1c_rfd_ring *ring = &adapter->rfd_ring[queue];
	u16 num_alloc = 0;
	u16 head = ring->head;

	while (!ring->buffer_info[(head + 1) & ring->mask].dma) {
		void *buf = __skb_bin_get_buffer();
		if (unlikely(!buf)) {
			if (netif_msg_rx_err(adapter))
				dev_warn(&adapter->pdev->dev, "alloc rx buffer failed\n");
			break;
		}

		struct atl1c_buffer *buffer_info = &ring->buffer_info[head];
		buffer_info->buf = buf;
		buffer_info->length = adapter->rx_buffer_len;
		buffer_info->dma = pci_map_single(adapter->pdev, buf + NET_SKB_PAD,
						  buffer_info->length, PCI_DMA_FROMDEVICE);
		buffer_info->fpb = true;
		ring->desc[head].buffer_addr = cpu_to_le64(buffer_info->dma);
		head = (head + 1) & ring->mask;
		num_alloc++;
	}

	if (num_alloc) {
		wmb();
		ring->head = head;
		AT_WRITE_REG(&adapter->hw, atl1c_qregs[queue].rfd_prod,
			     head & MB_RFDX_PROD_IDX_MASK);
	}
	return num_alloc;
}

static int atl1c_clean_rx(struct atl1c_adapter *adapter, u32 queue, int budget) {
	int work_done = 0;
	struct pci_dev *pdev = adapter->pdev;
	struct atl1c_rfd_ring *rfd_ring = &adapter->rfd_ring[queue];
	struct atl1c_rrd_ring *rrd_ring = &adapter->rrd_ring[queue];
	struct fast_path_state *fps = raw_cpu_ptr(&per_cpu_fp_state);

	while (work_done < budget) {
		struct atl1c_recv_ret_status *rrs = &rrd_ring->desc[rrd_ring->tail];
		u32 word3 = le32_to_cpu(rrs->word3);
		if (!(word3 & RRS_RXD_UPDATED)) {
			break;
		}
		rrs->word3 &= cpu_to_le32(~RRS_RXD_UPDATED);

		struct atl1c_buffer *buffer_info = &rfd_ring->buffer_info[rrd_ring->tail];
		pci_unmap_single(pdev, buffer_info->dma, buffer_info->length, PCI_DMA_FROMDEVICE);
		u16 length = (word3 >> RRS_PKT_SIZE_SHIFT) & RRS_PKT_SIZE_MASK;
		struct fp_buf *fpb = fpb_build(buffer_info->buf, NET_SKB_PAD, length - ETH_FCS_LEN);
		buffer_info->buf = NULL;
		buffer_info->dma = 0;

		if (++rrd_ring->tail == rrd_ring->count) {
			rrd_ring->tail = 0;
		}
		if (word3 & (RRS_RX_ERR_SUM | RRS_802_3_LEN_ERR)) {
			printk("%s: wrong packet! rrs word3 is %x\n", adapter->netdev->name, word3);
			continue;
		}
		if (adapter->hw.nic_type == athr_mt && word3 & RRS_MT_PROT_ID_TCPUDP) {
			fps->ip_summed = CHECKSUM_UNNECESSARY;
		}
		else {
			fps->ip_summed = CHECKSUM_NONE;
		}

		fast_path_rx(adapter->netdev, fpb);
#ifdef STATS
		++rrd_ring->stats_rx;
#endif
		work_done++;
	}
	fps->ip_summed = CHECKSUM_NONE;
	if (work_done)
		atl1c_alloc_rx_buffer(adapter, queue);
	return work_done;
}

static void atl1c_schedule_tx_napi(void *data) {
	struct atl1c_tpd_ring *ring = data;
	__napi_schedule(&ring->napi);
}

static void atl1c_schedule_rx_napi(void *data) {
	struct atl1c_rrd_ring *ring = data;
	__napi_schedule(&ring->napi);
}

static int atl1c_tx_poll(struct napi_struct *napi, int budget) {
	struct atl1c_tpd_ring *tpd_ring =
		container_of(napi, struct atl1c_tpd_ring, napi);
	struct atl1c_adapter *adapter = tpd_ring->adapter;
	unsigned long flags;
//	if (net_ratelimit()) {
//		printk("%s: tx_poll @%u\n", adapter->netdev->name, smp_processor_id());
//	}
#ifdef STATS
	++tpd_ring->stats_poll;
	atl1c_stats(adapter);
#endif
	int work_done = atl1c_clean_tx(adapter, tpd_ring->num);
	if (work_done >= budget) {
		return budget;
	}

#ifdef STATS
	++tpd_ring->stats_poll_done;
#endif
	napi_complete(napi);
	spin_lock_irqsave(&adapter->hw.intr_mask_lock, flags);
	adapter->hw.intr_mask |= atl1c_qregs[tpd_ring->num].tx_isr;
	AT_WRITE_REG(&adapter->hw, REG_IMR, adapter->hw.intr_mask);
	spin_unlock_irqrestore(&adapter->hw.intr_mask_lock, flags);
	return work_done;
}

static int atl1c_poll(struct napi_struct *napi, int budget) {
	struct atl1c_rrd_ring *rrd_ring =
		container_of(napi, struct atl1c_rrd_ring, napi);
	struct atl1c_adapter *adapter = rrd_ring->adapter;
	unsigned long flags;

#ifdef STATS
	++rrd_ring->stats_poll;
	atl1c_stats(adapter);
#endif
	int work_done = atl1c_clean_rx(adapter, rrd_ring->num, budget);

	call_xmit_commits();
	if (work_done >= budget) {
		return budget;
	}
#ifdef STATS
	++rrd_ring->stats_poll_done;
#endif

	napi_complete(napi);
	spin_lock_irqsave(&adapter->hw.intr_mask_lock, flags);
	adapter->hw.intr_mask |= atl1c_qregs[rrd_ring->num].rx_isr;
	AT_WRITE_REG(&adapter->hw, REG_IMR, adapter->hw.intr_mask);
	spin_unlock_irqrestore(&adapter->hw.intr_mask_lock, flags);
	return work_done;
}

static inline u16 atl1c_tpd_avail(struct atl1c_adapter *adapter, u32 queue) {
	struct atl1c_tpd_ring *ring = &adapter->tpd_ring[queue];
	return (ring->tail - ring->head - 1) & ring->mask;
}

static int atl1c_tso_csum(struct atl1c_adapter *adapter,
			  struct sk_buff *skb,
			  struct atl1c_tpd_desc **tpd,
			  u32 queue) {
	struct pci_dev *pdev = adapter->pdev;
	u8 hdr_len;
	u32 real_len;
	unsigned short offload_type;
	int err;
	struct atl1c_tpd_ring *ring = &adapter->tpd_ring[queue];

	if (skb_is_gso(skb)) {
		if (skb_header_cloned(skb)) {
			err = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
			if (unlikely(err))
				return -1;
		}
		offload_type = skb_shinfo(skb)->gso_type;

		if (offload_type & SKB_GSO_TCPV4) {
			real_len = (((unsigned char *)ip_hdr(skb) - skb->data)
					+ ntohs(ip_hdr(skb)->tot_len));

			if (real_len < skb->len)
				pskb_trim(skb, real_len);

			hdr_len = (skb_transport_offset(skb) + tcp_hdrlen(skb));
			if (unlikely(skb->len == hdr_len)) {
				/* only xsum need */
				if (netif_msg_tx_queued(adapter))
					dev_warn(&pdev->dev,
						"IPV4 tso with zero data??\n");
				goto check_sum;
			} else {
				ip_hdr(skb)->check = 0;
				tcp_hdr(skb)->check = ~csum_tcpudp_magic(
							ip_hdr(skb)->saddr,
							ip_hdr(skb)->daddr,
							0, IPPROTO_TCP, 0);
				(*tpd)->word1 |=
					cpu_to_le32(1 << TPD_IPV4_PACKET_SHIFT);
			}
		}

		if (offload_type & SKB_GSO_TCPV6) {
			struct atl1c_tpd_ext_desc *etpd =
				*(struct atl1c_tpd_ext_desc **)(tpd);

			memset(etpd, 0, sizeof(struct atl1c_tpd_ext_desc));

			*tpd = &ring->desc[ring->head];
			ring->head = (ring->head + 1) & ring->mask;
			memset(*tpd, 0, sizeof(struct atl1c_tpd_desc));

			ipv6_hdr(skb)->payload_len = 0;
			/* check payload == 0 byte ? */
			hdr_len = (skb_transport_offset(skb) + tcp_hdrlen(skb));
			if (unlikely(skb->len == hdr_len)) {
				/* only xsum need */
				if (netif_msg_tx_queued(adapter))
					dev_warn(&pdev->dev,
						"IPV6 tso with zero data??\n");
				goto check_sum;
			} else
				tcp_hdr(skb)->check = ~csum_ipv6_magic(
						&ipv6_hdr(skb)->saddr,
						&ipv6_hdr(skb)->daddr,
						0, IPPROTO_TCP, 0);
			etpd->word1 |= cpu_to_le32(1 << TPD_LSO_EN_SHIFT);
			etpd->word1 |= cpu_to_le32(1 << TPD_LSO_VER_SHIFT);
			etpd->pkt_len = cpu_to_le32(skb->len);
			(*tpd)->word1 |= cpu_to_le32(1 << TPD_LSO_VER_SHIFT);
		}

		(*tpd)->word1 |= cpu_to_le32(1 << TPD_LSO_EN_SHIFT);
		(*tpd)->word1 |= cpu_to_le32(
			(skb_transport_offset(skb) & TPD_TCPHDR_OFFSET_MASK) <<
			TPD_TCPHDR_OFFSET_SHIFT);
		(*tpd)->word1 |= cpu_to_le32(
			(skb_shinfo(skb)->gso_size & TPD_MSS_MASK) <<
			TPD_MSS_SHIFT);
		return 0;
	}

check_sum:
	if (likely(skb->ip_summed == CHECKSUM_PARTIAL)) {
		u8 css, cso;
		cso = skb_transport_offset(skb);

		if (unlikely(cso & 0x1)) {
			if (netif_msg_tx_err(adapter))
				dev_err(&adapter->pdev->dev,
					"payload offset should not an event number\n");
			return -1;
		} else {
			css = cso + skb->csum_offset;

			(*tpd)->word1 |= cpu_to_le32(
				((cso >> 1) & TPD_PLOADOFFSET_MASK) <<
				TPD_PLOADOFFSET_SHIFT);
			(*tpd)->word1 |= cpu_to_le32(
				((css >> 1) & TPD_CCSUM_OFFSET_MASK) <<
				TPD_CCSUM_OFFSET_SHIFT);
			(*tpd)->word1 |= cpu_to_le32(1 << TPD_CCSUM_EN_SHIFT);
		}
	}
	return 0;
}

static void atl1c_tx_map(struct atl1c_adapter *adapter,
			 struct sk_buff *skb, struct atl1c_tpd_desc *tpd,
			 u32 queue) {
	struct atl1c_tpd_desc *use_tpd;
	struct atl1c_buffer *buffer_info;
	u16 buf_len = skb_headlen(skb);
	u16 map_len = 0;
	u16 f;
	struct atl1c_tpd_ring *ring = &adapter->tpd_ring[queue];

	int tso = (le32_to_cpu(tpd->word1) >> TPD_LSO_EN_SHIFT) & TPD_LSO_EN_MASK;
	if (tso) {
		map_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		use_tpd = tpd;

		buffer_info = &ring->buffer_info[use_tpd - ring->desc];
		buffer_info->length = map_len;
		buffer_info->dma = pci_map_single(adapter->pdev,
					skb->data, map_len, PCI_DMA_TODEVICE);
		use_tpd->buffer_addr = cpu_to_le64(buffer_info->dma);
		use_tpd->buffer_len = cpu_to_le16(buffer_info->length);
	}

	if (map_len < buf_len) {
		/* map_len == 0, means we should use the first tpd,
		   which is given by caller  */
		if (map_len == 0)
			use_tpd = tpd;
		else {
			use_tpd = &ring->desc[ring->head];
			ring->head = (ring->head + 1) & ring->mask;
			memcpy(use_tpd, tpd, sizeof(struct atl1c_tpd_desc));
		}
		buffer_info = &ring->buffer_info[use_tpd - ring->desc];
		buffer_info->length = buf_len - map_len;
		buffer_info->dma =
			pci_map_single(adapter->pdev, skb->data + map_len,
					buffer_info->length, PCI_DMA_TODEVICE);
		use_tpd->buffer_addr = cpu_to_le64(buffer_info->dma);
		use_tpd->buffer_len  = cpu_to_le16(buffer_info->length);
	}

	for (f = 0; f < skb_shinfo(skb)->nr_frags; f++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[f];

		use_tpd = &ring->desc[ring->head];
		ring->head = (ring->head + 1) & ring->mask;
		memcpy(use_tpd, tpd, sizeof(struct atl1c_tpd_desc));

		buffer_info = &ring->buffer_info[use_tpd - ring->desc];
		buffer_info->length = skb_frag_size(frag);
		buffer_info->dma = skb_frag_dma_map(&adapter->pdev->dev,
						    frag, 0,
						    buffer_info->length,
						    DMA_TO_DEVICE);
		use_tpd->buffer_addr = cpu_to_le64(buffer_info->dma);
		use_tpd->buffer_len  = cpu_to_le16(buffer_info->length);
	}

	use_tpd->word1 |= cpu_to_le32(1 << TPD_EOP_SHIFT);
	buffer_info->buf = skb;
}

static netdev_tx_t atl1c_xmit_frame(struct sk_buff *skb,
				    struct net_device *netdev) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	struct atl1c_tpd_desc *tpd;
	u32 queue = smp_processor_id() % adapter->tx_queue_count;
	struct atl1c_tpd_ring *ring = &adapter->tpd_ring[queue];
	struct netdev_queue *txq = netdev_get_tx_queue(netdev, queue);
	spin_lock(&ring->tx_lock);

	if (atl1c_tpd_avail(adapter, queue) <= adapter->tpd_ring[queue].stop_threshold) {
		if (!netif_tx_queue_stopped(txq)) {
#ifdef STATS
			++ring->stats_tx_queue_stop;
#endif
			netif_tx_stop_queue(txq);
		}
		spin_unlock(&ring->tx_lock);
		return NETDEV_TX_BUSY;
	}

	tpd = &ring->desc[ring->head];
	ring->head = (ring->head + 1) & ring->mask;
	memset(tpd, 0, sizeof(struct atl1c_tpd_desc));

	fast_path_sp_tx_inc(netdev, skb->len);

	if (atl1c_tso_csum(adapter, skb, &tpd, queue) != 0) {
		spin_unlock(&ring->tx_lock);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (unlikely(skb_vlan_tag_present(skb))) {
		u16 vlan = cpu_to_le16(skb_vlan_tag_get(skb));
		tpd->word1 |= cpu_to_le32(1 << TPD_INS_VTAG_SHIFT);
		tpd->vlan_tag = (vlan >> 8) | (vlan << 8);
	}

	if (skb_network_offset(skb) != ETH_HLEN)
		tpd->word1 |= cpu_to_le32(1 << TPD_ETH_TYPE_SHIFT); /* Ethernet frame */

	atl1c_tx_map(adapter, skb, tpd, queue);
	wmb();
	AT_WRITE_REGW(&adapter->hw, atl1c_qregs[queue].tpd_prod, ring->head);
#ifdef STATS
	++ring->stats_xmit;
#endif

	spin_unlock(&ring->tx_lock);
	return NETDEV_TX_OK;
}

static int atl1c_fast_path_xmit(struct net_device *dev, struct fp_buf *fpb) {
	struct atl1c_adapter *adapter = netdev_priv(dev);
	unsigned cpu = smp_processor_id();
	u32 queue = cpu % adapter->tx_queue_count;
	struct atl1c_tpd_ring *ring = &adapter->tpd_ring[queue];
	struct atl1c_pcpu *pcpu = &ring->pcpu[cpu];

	pcpu->fpbs[pcpu->xmit_count] = *fpb;
	pcpu->dma_addrs[pcpu->xmit_count] = pci_map_single(adapter->pdev, fpb_data(fpb), fpb_len(fpb), PCI_DMA_TODEVICE);
#ifdef STATS
	++ring->stats_xmit;
#endif

	if (!pcpu->xmit_count) {
		schedule_xmit_commit(dev);
	}
	pcpu->xmit_count += 1;
	return 0;
}

static int atl1c_xmit_commit(struct net_device *dev) {
	struct atl1c_adapter *adapter = netdev_priv(dev);
	unsigned cpu = smp_processor_id();
	u32 queue = cpu % adapter->tx_queue_count;
	struct atl1c_tpd_ring *ring = &adapter->tpd_ring[queue];
	struct atl1c_pcpu *pcpu = &ring->pcpu[cpu];
	struct netdev_queue *txq = netdev_get_tx_queue(dev, queue);
	unsigned i;
	struct fp_buf *fpb;

	spin_lock(&ring->tx_lock);

	if (atl1c_tpd_avail(adapter, queue) <= pcpu->xmit_count) {
		if (!netif_tx_queue_stopped(txq)) {
			netif_tx_stop_queue(txq);
		}
		spin_unlock(&ring->tx_lock);
		for (i = 0; i < pcpu->xmit_count; ++i) {
			fpb = &pcpu->fpbs[i];
			pci_unmap_single(adapter->pdev, pcpu->dma_addrs[i], fpb_len(fpb), PCI_DMA_TODEVICE);
			__skb_bin_put_buffer_noinvalidate(fpb_buf(fpb));
		}
#ifdef STATS
		++ring->stats_tx_queue_stop;
#endif
		pcpu->xmit_count = 0;
		return 0;
	}

	u16 head = ring->head;
	for (i = 0; i < pcpu->xmit_count; ++i) {
		fpb = &pcpu->fpbs[i];
		fast_path_fp_tx_inc(dev, fpb);

		struct atl1c_tpd_desc *tpd = &ring->desc[head];
		struct atl1c_buffer *buffer_info = &ring->buffer_info[head];
		buffer_info->length = fpb_len(fpb);
		buffer_info->dma = pcpu->dma_addrs[i];
		buffer_info->fpb = true;
		buffer_info->buf = fpb_buf(fpb);
		tpd->buffer_addr = cpu_to_le64(buffer_info->dma);
		tpd->buffer_len  = cpu_to_le16(buffer_info->length);
		tpd->word1 = cpu_to_le32(1 << TPD_EOP_SHIFT);
		head = (head + 1) & ring->mask;
	}
	ring->head = head;
	wmb();
	AT_WRITE_REGW(&adapter->hw, atl1c_qregs[queue].tpd_prod, ring->head);
	spin_unlock(&ring->tx_lock);

#ifdef STATS
	++ring->stats_xmit_commit;
#endif
	int ret = pcpu->xmit_count;
	pcpu->xmit_count = 0;
	return ret;
}

static void atl1c_free_irq(struct atl1c_adapter *adapter) {
	struct net_device *netdev = adapter->netdev;

	free_irq(adapter->pdev->irq, netdev);

	if (adapter->have_msi)
		pci_disable_msi(adapter->pdev);
}

static int atl1c_request_irq(struct atl1c_adapter *adapter) {
	struct pci_dev    *pdev   = adapter->pdev;
	struct net_device *netdev = adapter->netdev;
	int flags = 0;
	int err = 0;

	adapter->have_msi = true;
	err = pci_enable_msi(adapter->pdev);
	if (err) {
		if (netif_msg_ifup(adapter))
			dev_err(&pdev->dev,
				"Unable to allocate MSI interrupt Error: %d\n",
				err);
		adapter->have_msi = false;
	} else
		netdev->irq = pdev->irq;

	if (!adapter->have_msi)
		flags |= IRQF_SHARED;
	err = request_irq(adapter->pdev->irq, atl1c_intr, flags,
			  "atl1c", netdev);
	if (err) {
		if (netif_msg_ifup(adapter))
			dev_err(&pdev->dev,
				"Unable to allocate interrupt Error: %d\n",
				err);
		if (adapter->have_msi)
			pci_disable_msi(adapter->pdev);
		return err;
	}
	if (netif_msg_ifup(adapter))
		dev_dbg(&pdev->dev, "atl1c_request_irq OK\n");

	err = irq_set_affinity(adapter->pdev->irq, cpumask_of(adapter->base_cpu));
	if (err) {
	    dev_err(&pdev->dev, "irq_set_affinity failed!\n");
	    err = 0;
	}
	return err;
}

static int atl1c_up(struct atl1c_adapter *adapter) {
	struct net_device *netdev = adapter->netdev;
	int num;
	int err;
	int i;

	atl1c_set_rxbufsize(adapter, netdev);
	skb_bin_request(netdev, adapter->rx_buffer_len);
	netif_carrier_off(netdev);
	atl1c_init_ring_ptrs(adapter);

	for (i = 0; i < adapter->rx_queue_count; i++) {
		num = atl1c_alloc_rx_buffer(adapter, i);
		if (unlikely(num == 0)) {
			err = -ENOMEM;
			goto err_alloc_rx;
		}
	}

	if (atl1c_configure(adapter)) {
		err = -EIO;
		goto err_up;
	}

	adapter->hw.intr_mask = IMR_NORMAL_MASK;
	for (i = 0; i < adapter->tx_queue_count; ++i)
		adapter->hw.intr_mask |= atl1c_qregs[i].tx_isr;
	for (i = 0; i < adapter->rx_queue_count; ++i)
		adapter->hw.intr_mask |= atl1c_qregs[i].rx_isr;
	AT_WRITE_REG(&adapter->hw, REG_ISR, 0x7FFFFFFF);
	AT_WRITE_REG(&adapter->hw, REG_IMR, adapter->hw.intr_mask);
	AT_WRITE_FLUSH(&adapter->hw);

	err = atl1c_request_irq(adapter);
	if (unlikely(err))
		goto err_up;

	for (i = 0; i < adapter->tx_queue_count; ++i) {
		napi_enable(&adapter->tpd_ring[i].napi);
	}
	for (i = 0; i < adapter->rx_queue_count; ++i) {
		napi_enable(&adapter->rrd_ring[i].napi);
	}
	atl1c_check_link_status(adapter);
	netif_tx_start_all_queues(netdev);
	return err;

err_up:
err_alloc_rx:
	atl1c_clean_rx_ring(adapter);
	return err;
}

static void atl1c_down(struct atl1c_adapter *adapter) {
	struct net_device *netdev = adapter->netdev;
	int i;

//	atl1c_dump(adapter, ETHTOOL_FLAG_MAC_DUMP_DESC);
	adapter->work_event = 0;
	netif_tx_stop_all_queues(netdev);
	for (i = 0; i < adapter->tx_queue_count; ++i) {
		napi_disable(&adapter->tpd_ring[i].napi);
	}
	for (i = 0; i < adapter->rx_queue_count; ++i) {
		napi_disable(&adapter->rrd_ring[i].napi);
	}
	netif_carrier_off(netdev);

	AT_WRITE_REG(&adapter->hw, REG_IMR, 0);
	AT_WRITE_FLUSH(&adapter->hw);
	synchronize_irq(adapter->pdev->irq);
	atl1c_free_irq(adapter);
	atl1c_reset_mac(&adapter->hw);
	msleep(1);

	adapter->link_speed = SPEED_0;
	adapter->link_duplex = -1;
	for (i = 0; i < adapter->tx_queue_count; ++i) {
		atl1c_clean_tx_ring(adapter, i);
	}
	atl1c_clean_rx_ring(adapter);
	skb_bin_release(netdev);
}

#if defined(CONFIG_RB1100) || defined(CONFIG_RB_PPC)
static int bypass_disabled = 0;
#endif

static int atl1c_open(struct net_device *netdev) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	int err;

//	amd_iommu_page_fault = 0;
	printk("%s: open %x\n", netdev->name, adapter->pdev->bus->number);
#if defined(CONFIG_RB1100) || defined(CONFIG_RB_PPC)
	if (machine_is(rb1100)) {
		if (adapter->pdev->bus->number == 2 ||
		    adapter->pdev->bus->number == 6) {
			if (!bypass_disabled) {
				change_latch(0x40, 0);
			}
			++bypass_disabled;
		}
	}
#endif

	atl1c_reset_pcie(&adapter->hw, ATL1C_PCIE_L0S_L1_DISABLE |
			ATL1C_PCIE_PHY_RESET);
	atl1c_reset_mac(&adapter->hw);
	device_init_wakeup(&adapter->pdev->dev, 1);

	atl1c_phy_reset(&adapter->hw);
	atl1c_phy_init(&adapter->hw);

	err = atl1c_setup_ring_resources(adapter);
	if (unlikely(err))
		return err;

	err = atl1c_up(adapter);
	if (unlikely(err))
		goto err_up;
	return 0;

err_up:
	atl1c_free_irq(adapter);
	atl1c_free_ring_resources(adapter);
	atl1c_reset_mac(&adapter->hw);
	return err;
}

static int atl1c_close(struct net_device *netdev) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	printk("%s: close %x\n", netdev->name, adapter->pdev->bus->number);

	atl1c_down(adapter);
	atl1c_free_ring_resources(adapter);
	atl1c_phy_disable(&adapter->hw);

#if defined(CONFIG_RB1100) || defined(CONFIG_RB_PPC)
	if (machine_is(rb1100)) {
		if (adapter->pdev->bus->number == 2 ||
		    adapter->pdev->bus->number == 6) {
			--bypass_disabled;
			if (!bypass_disabled) {
				change_latch(0, 0x40);
			}
		}
	}
#endif

	return 0;
}

static u16 atl1c_select_queue(struct net_device *dev, struct sk_buff *skb,
			      struct net_device *sb_dev) {
	struct atl1c_adapter *adapter = netdev_priv(dev);
	return smp_processor_id() % adapter->tx_queue_count;
}

static const struct net_device_ops atl1c_netdev_ops = {
	.ndo_open		= atl1c_open,
	.ndo_stop		= atl1c_close,
	.ndo_start_xmit		= atl1c_xmit_frame,
	.ndo_fast_path_xmit = atl1c_fast_path_xmit,
	.ndo_xmit_commit = atl1c_xmit_commit,
	.ndo_set_mac_address 	= atl1c_set_mac_addr,
	.ndo_change_mtu		= atl1c_change_mtu,
	.ndo_change_l2mtu	= atl1c_change_l2mtu,
	.ndo_do_ioctl		= atl1c_ioctl,
	.ndo_tx_timeout		= atl1c_tx_timeout,
	.ndo_get_stats64 = fast_path_get_stats64,
	.ndo_select_queue = atl1c_select_queue,
};

static int atl1c_get_settings(struct net_device *netdev,
			      struct ethtool_link_ksettings *ecmd) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	if (!(netdev->flags & IFF_UP)) return -EINVAL;
	if (adapter->hw.nic_type == athr_mt) {
	    ethtool_link_ksettings_add_link_mode(ecmd, supported, Autoneg);
	    ethtool_link_ksettings_add_link_mode(ecmd, supported, FIBRE);
	    ethtool_link_ksettings_add_link_mode(ecmd, supported,
		    10000baseSR_Full);
	    ethtool_link_ksettings_add_link_mode(ecmd, supported,
		    10000baseCR_Full);
	    ethtool_link_ksettings_add_link_mode(ecmd, supported,
		    25000baseSR_Full);
	    ethtool_link_ksettings_add_link_mode(ecmd, supported,
		    25000baseCR_Full);
	    if (adapter->link_speed != SPEED_0) {
		ecmd->base.speed = adapter->link_speed;
		ecmd->base.duplex = DUPLEX_FULL;
	    }
	    else {
		ecmd->base.speed = SPEED_UNKNOWN;
		ecmd->base.duplex = DUPLEX_UNKNOWN;
	    }
	    ecmd->base.port = PORT_FIBRE;
	    ecmd->base.autoneg = AUTONEG_ENABLE;
	    return 0;
	}
	mii_ethtool_get_link_ksettings(&adapter->mii, ecmd);
	return 0;
}

static int atl1c_set_settings(struct net_device *netdev,
			      const struct ethtool_link_ksettings *ecmd) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);
	if (!(netdev->flags & IFF_UP)) return -EINVAL;
	return mii_ethtool_set_link_ksettings(&adapter->mii, ecmd);
}

static void atl1c_get_drvinfo(struct net_device *netdev,
			      struct ethtool_drvinfo *drvinfo) {
	struct atl1c_adapter *adapter = netdev_priv(netdev);

	strlcpy(drvinfo->driver,  atl1c_driver_name, sizeof(drvinfo->driver));
	strcpy(drvinfo->version, "ros");
	strlcpy(drvinfo->fw_version, "N/A", sizeof(drvinfo->fw_version));
	strlcpy(drvinfo->bus_info, pci_name(adapter->pdev),
		sizeof(drvinfo->bus_info));
	ethtool_drvinfo_ext_fill(drvinfo, 0, MAX_JUMBO_FRAME_SIZE - 100);
}

static void atl1c_ethtool_get_strings(struct net_device *dev, u32 stringset,
				      u8 * buf) {
	struct atl1c_adapter *adapter = netdev_priv(dev);
	eth_stats_ethtool_get_strings(&adapter->xstats, stringset, buf);
}

static int atl1c_get_sset_count(struct net_device *dev, int sset) {
    if (sset == ETH_SS_TEST) {
	return 1; // not zero to make net/core/ethtool.c not WARN_ON
    }
    return eth_stats_ethtool_get_sset_count();
}

static void atl1c_ethtool_get_ethtool_stats(struct net_device *dev,
					    struct ethtool_stats *estats, u64 *stats) {
	struct atl1c_adapter *adapter = netdev_priv(dev);
	fast_path_get_eth_stats(dev, &adapter->xstats);
	atl1c_update_stats(adapter);
	memcpy(stats, &adapter->xstats, sizeof(adapter->xstats));
}

static void atl1c_self_test(struct net_device *dev, struct ethtool_test *test,
			    u64 *data) {
	struct atl1c_adapter *adapter = netdev_priv(dev);
	atl1c_dump(adapter, test->flags);
}

static const struct ethtool_ops atl1c_ethtool_ops = {
	.get_link_ksettings     = atl1c_get_settings,
	.set_link_ksettings     = atl1c_set_settings,
	.get_drvinfo            = atl1c_get_drvinfo,
	.get_link               = ethtool_op_get_link,

	.get_strings = atl1c_ethtool_get_strings,
	.get_sset_count = atl1c_get_sset_count,
	.get_ethtool_stats = atl1c_ethtool_get_ethtool_stats,
	.self_test = atl1c_self_test,
};

static int atl1c_probe(struct pci_dev *pdev,
		       const struct pci_device_id *ent) {
	struct net_device *netdev;
	struct atl1c_adapter *adapter;
	static int base_cpu = 0;
	int i;
	int err = 0;

	/* enable device (incl. PCI PM wakeup and hotplug setup) */
	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "cannot enable PCI device\n");
		return err;
	}

	/*
	 * The atl1c chip can DMA to 64-bit addresses, but it uses a single
	 * shared register for the high 32 bits, so only a single, aligned,
	 * 4 GB physical address range can be used at a time.
	 *
	 * Supporting 64-bit DMA on this hardware is more trouble than it's
	 * worth.  It is far easier to limit to 32-bit DMA than update
	 * various kernel subsystems to support the mechanics required by a
	 * fixed-high-32-bit system.
	 */
	if ((pci_set_dma_mask(pdev, DMA_BIT_MASK(32)) != 0) ||
	    (pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32)) != 0)) {
		dev_err(&pdev->dev, "No usable DMA configuration,aborting\n");
		goto err_dma;
	}

	err = pci_request_regions(pdev, atl1c_driver_name);
	if (err) {
		dev_err(&pdev->dev, "cannot obtain PCI resources\n");
		goto err_pci_reg;
	}

	pci_set_master(pdev);
/*
	int cap = pci_pcie_cap(pdev);
	if (cap) {
		printk("wil: cap offset:%x\n", cap);
		u32 devcap;
		pci_read_config_dword(pdev, cap + PCI_EXP_DEVCAP, &devcap);
		unsigned payload_supported = devcap & PCI_EXP_DEVCAP_PAYLOAD;
		printk("wil: PCI_EXP_DEVCAP:%08x, max payload supported:%u\n",
		       devcap, payload_supported);
		u16 ctl;
		pci_read_config_word(pdev, cap + PCI_EXP_DEVCTL, &ctl);
		unsigned payload = (ctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5;
		unsigned readrq = (ctl & PCI_EXP_DEVCTL_READRQ) >> 12;
		printk("wil: PCI_EXP_DEVCTL: %04x, payload:%u readrq:%u\n", ctl, payload, readrq);
//	if (payload < payload_supported) {
//	    payload = payload_supported;
//	}
		payload = 1;
//	if (readrq > 1) {
//	    readrq = 1;
//	}

//		ctl &= ~(PCI_EXP_DEVCTL_PAYLOAD | PCI_EXP_DEVCTL_READRQ);
		ctl &= ~(PCI_EXP_DEVCTL_PAYLOAD);
		ctl |= payload << 5;
//		ctl |= readrq << 12;
		printk("wil: PCI_EXP_DEVCTL: %04x, payload:%u readrq:%u\n", ctl, payload, readrq);
		pci_write_config_word(pdev, cap + PCI_EXP_DEVCTL, ctl);
	}
*/


	u8 __iomem *hw_addr = pci_ioremap_bar(pdev, 0);
	if (!hw_addr) {
		err = -EIO;
		dev_err(&pdev->dev, "cannot map device registers\n");
		goto err_ioremap;
	}
	u32 queue_count = 1;
	enum atl1c_nic_type nic_type = atl1c_get_mac_type(pdev, hw_addr);
	if (nic_type == athr_mt) {
		queue_count = 4;
	}

	netdev = alloc_etherdev_mq(sizeof(struct atl1c_adapter), queue_count);
	if (netdev == NULL) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "etherdev alloc failed\n");
		goto err_alloc_etherdev;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);

	netdev->irq  = pdev->irq;
	netdev->netdev_ops = &atl1c_netdev_ops;
	netdev->watchdog_timeo = HZ * 5;
	netdev->l2mtu = 1600;
	netdev->ethtool_ops = &atl1c_ethtool_ops;
	netdev->priv_flags |= IFF_NO_QUEUE;
	netdev->hw_features = NETIF_F_SG | NETIF_F_HW_CSUM | NETIF_F_TSO | NETIF_F_TSO6 | NETIF_F_LLTX;
	netdev->features = netdev->hw_features;

	adapter = netdev_priv(netdev);
	adapter->base_cpu = base_cpu;
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	adapter->hw.adapter = adapter;
	adapter->msg_enable = netif_msg_init(-1, atl1c_default_msg);
	adapter->hw.hw_addr = hw_addr;
	adapter->tx_queue_count = queue_count;
	adapter->rx_queue_count = queue_count;
	adapter->hw.nic_type = nic_type;
	netdev->base_addr = (unsigned long)adapter->hw.hw_addr;

	/* init mii data */
	adapter->mii.dev = netdev;
	adapter->mii.mdio_read  = atl1c_mdio_read;
	adapter->mii.mdio_write = atl1c_mdio_write;
	adapter->mii.phy_id_mask = 0x1f;
	adapter->mii.reg_num_mask = MDIO_REG_ADDR_MASK;
	adapter->mii.supports_gmii = 1;
//	dev_set_threaded(netdev, true);
	for (i = 0; i < adapter->rx_queue_count; ++i) {
		struct atl1c_rrd_ring *r = &adapter->rrd_ring[i];
		netif_napi_add(netdev, &r->napi, atl1c_poll, FP_NAPI_BUDGET);
		r->target_cpu = base_cpu;
		r->csd.func = atl1c_schedule_rx_napi;
		r->csd.info = r;
		r->csd.flags = 0;
		base_cpu = (base_cpu + 1) % num_online_cpus();
	}
	for (i = 0; i < adapter->tx_queue_count; ++i) {
		struct atl1c_tpd_ring *r = &adapter->tpd_ring[i];
		netif_napi_add(netdev, &r->napi, atl1c_tx_poll, FP_NAPI_BUDGET);
		r->target_cpu = base_cpu;
		r->csd.func = atl1c_schedule_tx_napi;
		r->csd.info = r;
		r->csd.flags = 0;
		spin_lock_init(&r->tx_lock);
		base_cpu = (base_cpu + 1) % num_online_cpus();
	}
	/* setup the private structure */
	err = atl1c_sw_init(adapter);
	if (err) {
		dev_err(&pdev->dev, "net device private data init failed\n");
		goto err_sw_init;
	}
	atl1c_reset_pcie(&adapter->hw, ATL1C_PCIE_L0S_L1_DISABLE |
			ATL1C_PCIE_PHY_RESET);

	eth_stats_calc_flags(&adapter->xstats, eth_stat_items_atl1c);
	eth_stats_set_driver_basic_flags(&adapter->xstats);

	err = atl1c_reset_mac(&adapter->hw);
	if (err) {
		err = -EIO;
		goto err_reset;
	}

	device_init_wakeup(&pdev->dev, 1);

	if (atl1c_read_mac_addr(&adapter->hw) != 0) {
		err = -EIO;
		dev_err(&pdev->dev, "get mac address failed\n");
		goto err_eeprom;
	}
	memcpy(netdev->dev_addr, adapter->hw.mac_addr, netdev->addr_len);
	memcpy(netdev->perm_addr, adapter->hw.mac_addr, netdev->addr_len);
	if (netif_msg_probe(adapter))
		dev_dbg(&pdev->dev, "mac address : %pM\n",
			adapter->hw.mac_addr);

	INIT_WORK(&adapter->common_task, atl1c_common_task);
	adapter->work_event = 0;
	err = register_netdev(netdev);
	if (err) {
		dev_err(&pdev->dev, "register netdevice failed\n");
		goto err_register;
	}

	return 0;

err_reset:
err_register:
err_sw_init:
err_eeprom:
	free_netdev(netdev);
err_alloc_etherdev:
	iounmap(adapter->hw.hw_addr);
err_ioremap:
	pci_release_regions(pdev);
err_pci_reg:
err_dma:
	pci_disable_device(pdev);
	return err;
}

static void atl1c_remove(struct pci_dev *pdev) {
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct atl1c_adapter *adapter = netdev_priv(netdev);

	unregister_netdev(netdev);

	iounmap(adapter->hw.hw_addr);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	free_netdev(netdev);
}

static struct pci_driver atl1c_driver = {
	.name     = atl1c_driver_name,
	.id_table = atl1c_pci_tbl,
	.probe    = atl1c_probe,
	.remove   = atl1c_remove,
};

static int __init atl1c_init_module(void) {
	return pci_register_driver(&atl1c_driver);
}

static void atl1c_exit_module(void) {
	pci_unregister_driver(&atl1c_driver);
}

module_init(atl1c_init_module);
module_exit(atl1c_exit_module);
