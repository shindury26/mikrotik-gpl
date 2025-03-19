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
 * File: velocity.h
 *
 * Purpose: Header file to define driver's private structures.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */

#include <linux/pci.h>
#include <linux/etherdevice.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/ethtool.h>
#include <linux/mii.h>

//#define USE_PIO
#undef USE_PIO

#ifdef USE_PIO
#define WRITE4(hw, val, reg) writel(val, (hw)->ioaddr + (reg))
#define WRITE2(hw, val, reg) writew(val, (hw)->ioaddr + (reg))
#define WRITE1(hw, val, reg) writeb(val, (hw)->ioaddr + (reg))

#define READ4(hw, reg)   readl((hw)->ioaddr + (reg))
#define READ2(hw, reg)   readw((hw)->ioaddr + (reg))
#define READ1(hw, reg)   readb((hw)->ioaddr + (reg))

#else
#define WRITE4(hw, val, reg) writel(val, (hw)->hw_addr + (reg))
#define WRITE2(hw, val, reg) writew(val, (hw)->hw_addr + (reg))
#define WRITE1(hw, val, reg) writeb(val, (hw)->hw_addr + (reg))

#define READ4(hw, reg)   readl((hw)->hw_addr + (reg))
#define READ2(hw, reg)   readw((hw)->hw_addr + (reg))
#define READ1(hw, reg)   readb((hw)->hw_addr + (reg))
#endif

#define ON1(hw, x, p) do { WRITE1(hw, (READ1(hw, p) |(x)),(p)); } while (0)
#define OFF1(hw, x, p) do { WRITE1(hw,(READ1(hw, p) & (~(x))), (p)); } while (0)
#define SET1(hw, x, m, p) do { WRITE1(hw,(READ1(hw,(p)) & (~(m))) |(x), (p)); } while (0)

// Registers in the MAC
#define MAC_REG_PAR         0x00
#define MAC_REG_RCR         0x06 
#define MAC_REG_TCR         0x07 
#define MAC_REG_CR0_SET     0x08 
#define MAC_REG_CR1_SET     0x09 
#define MAC_REG_CR2_SET     0x0A 
#define MAC_REG_CR3_SET     0x0B 
#define MAC_REG_CR0_CLR     0x0C 
#define MAC_REG_CR1_CLR     0x0D 
#define MAC_REG_CR2_CLR     0x0E 
#define MAC_REG_CR3_CLR     0x0F 
#define MAC_REG_MAR         0x10 
#define MAC_REG_CAM         0x10 
#define MAC_REG_DEC_BASE_HI 0x18 
#define MAC_REG_DBF_BASE_HI 0x1C 
#define MAC_REG_ISR_CTL     0x20 
#define MAC_REG_ISR_HOTMR   0x20 
#define MAC_REG_ISR_TSUPTHR 0x20 
#define MAC_REG_ISR_RSUPTHR 0x20 
#define MAC_REG_ISR_CTL1    0x21 
#define MAC_REG_TXE_SR      0x22 
#define MAC_REG_RXE_SR      0x23 
#define MAC_REG_ISR         0x24 
#define MAC_REG_ISR0        0x24 
#define MAC_REG_ISR1        0x25 
#define MAC_REG_ISR2        0x26 
#define MAC_REG_ISR3        0x27 
#define MAC_REG_IMR         0x28 
#define MAC_REG_IMR0        0x28 
#define MAC_REG_IMR1        0x29 
#define MAC_REG_IMR2        0x2A 
#define MAC_REG_IMR3        0x2B 
#define MAC_REG_TDCSR_SET   0x30 
#define MAC_REG_RDCSR_SET   0x32 
#define MAC_REG_TDCSR_CLR   0x34 
#define MAC_REG_RDCSR_CLR   0x36 
#define MAC_REG_RDBASE_LO   0x38 
#define MAC_REG_RDINDX      0x3C 
#define MAC_REG_TQETMR      0x3E // VT3216 only
#define MAC_REG_RQETMR      0x3F // VT3216 only
#define MAC_REG_TDBASE_LO   0x40 
#define MAC_REG_RDCSIZE     0x50 
#define MAC_REG_TDCSIZE     0x52 
#define MAC_REG_TDINDX      0x54 
#define MAC_REG_TDIDX0      0x54 
#define MAC_REG_TDIDX1      0x56 
#define MAC_REG_TDIDX2      0x58 
#define MAC_REG_TDIDX3      0x5A 
#define MAC_REG_PAUSE_TIMER 0x5C 
#define MAC_REG_RBRDU       0x5E 
#define MAC_REG_FIFO_TEST0  0x60 
#define MAC_REG_FIFO_TEST1  0x64 
#define MAC_REG_CAMADDR     0x68 
#define MAC_REG_CAMCR       0x69 
#define MAC_REG_GFTEST      0x6A 
#define MAC_REG_FTSTCMD     0x6B 
#define MAC_REG_MIICFG      0x6C 
#define MAC_REG_MIISR       0x6D 
#define MAC_REG_PHYSR0      0x6E 
#define MAC_REG_PHYSR1      0x6F 
#define MAC_REG_MIICR       0x70 
#define MAC_REG_MIIADR      0x71 
#define MAC_REG_MIIDATA     0x72 
#define MAC_REG_SOFT_TIMER0 0x74 
#define MAC_REG_SOFT_TIMER1 0x76 
#define MAC_REG_CFGA        0x78 
#define MAC_REG_CFGB        0x79 
#define MAC_REG_CFGC        0x7A 
#define MAC_REG_CFGD        0x7B 
#define MAC_REG_DCFG0       0x7C 
#define MAC_REG_DCFG1       0x7D 
#define MAC_REG_MCFG0       0x7E 
#define MAC_REG_MCFG1       0x7F 

#define MAC_REG_TBIST       0x80 
#define MAC_REG_RBIST       0x81 
#define MAC_REG_PMCC        0x82 
#define MAC_REG_STICKHW     0x83 
#define MAC_REG_MIBCR       0x84 
#define MAC_REG_EERSV       0x85 
#define MAC_REG_REVID       0x86 
#define MAC_REG_MIBREAD     0x88 
#define MAC_REG_MIBDATA     0x88
#define MAC_REG_BPMA        0x8C 
#define MAC_REG_EEWR_DATA   0x8C 
#define MAC_REG_BPMD_WR     0x8F 
#define MAC_REG_BPCMD       0x90 
#define MAC_REG_BPMD_RD     0x91 
#define MAC_REG_EECHKSUM    0x92 
#define MAC_REG_EECSR       0x93 
#define MAC_REG_EERD_DATA   0x94 
#define MAC_REG_EADDR       0x96 
#define MAC_REG_EMBCMD      0x97 
#define MAC_REG_JMPSR0      0x98 
#define MAC_REG_JMPSR1      0x99 
#define MAC_REG_JMPSR2      0x9A 
#define MAC_REG_JMPSR3      0x9B 
#define MAC_REG_CHIPGSR     0x9C 
#define MAC_REG_TESTCFG     0x9D 
#define MAC_REG_DEBUG       0x9E 
#define MAC_REG_CHIPGCR     0x9F 
#define MAC_REG_WOLCR0_SET  0xA0 
#define MAC_REG_WOLCR1_SET  0xA1 
#define MAC_REG_PWCFG_SET   0xA2 
#define MAC_REG_WOLCFG_SET  0xA3 
#define MAC_REG_WOLCR0_CLR  0xA4 
#define MAC_REG_WOLCR1_CLR  0xA5 
#define MAC_REG_PWCFG_CLR   0xA6 
#define MAC_REG_WOLCFG_CLR  0xA7 
#define MAC_REG_WOLSR0_SET  0xA8 
#define MAC_REG_WOLSR1_SET  0xA9 
#define MAC_REG_WOLSR0_CLR  0xAC 
#define MAC_REG_WOLSR1_CLR  0xAD 
#define MAC_REG_PATRN_CRC0  0xB0 
#define MAC_REG_PATRN_CRC1  0xB2 
#define MAC_REG_PATRN_CRC2  0xB4 
#define MAC_REG_PATRN_CRC3  0xB6 
#define MAC_REG_PATRN_CRC4  0xB8 
#define MAC_REG_PATRN_CRC5  0xBA 
#define MAC_REG_PATRN_CRC6  0xBC 
#define MAC_REG_PATRN_CRC7  0xBE 
#define MAC_REG_BYTEMSK0_0  0xC0 
#define MAC_REG_BYTEMSK0_1  0xC4 
#define MAC_REG_BYTEMSK0_2  0xC8 
#define MAC_REG_BYTEMSK0_3  0xCC 
#define MAC_REG_BYTEMSK1_0  0xD0 
#define MAC_REG_BYTEMSK1_1  0xD4 
#define MAC_REG_BYTEMSK1_2  0xD8 
#define MAC_REG_BYTEMSK1_3  0xDC 
#define MAC_REG_BYTEMSK2_0  0xE0 
#define MAC_REG_BYTEMSK2_1  0xE4 
#define MAC_REG_BYTEMSK2_2  0xE8 
#define MAC_REG_BYTEMSK2_3  0xEC 
#define MAC_REG_BYTEMSK3_0  0xF0 
#define MAC_REG_BYTEMSK3_1  0xF4 
#define MAC_REG_BYTEMSK3_2  0xF8 
#define MAC_REG_BYTEMSK3_3  0xFC 


// Bits in the RSR0 register
#define RSR_DETAG 0x0080
#define RSR_SNTAG 0x0040
#define RSR_RXER  0x0020
#define RSR_RL    0x0010
#define RSR_CE    0x0008
#define RSR_FAE   0x0004
#define RSR_CRC   0x0002
#define RSR_VIDM  0x0001
// Bits in the RSR1 register
#define RSR_RXOK  0x8000 // rx OK
#define RSR_PFT   0x4000 // Perfect filtering address match
#define RSR_MAR   0x2000 // MAC accept multicast address packet
#define RSR_BAR   0x1000 // MAC accept broadcast address packet
#define RSR_PHY   0x0800 // MAC accept physical address packet
#define RSR_VTAG  0x0400 // 802.1p/1Q tagging packet indicator
#define RSR_STP   0x0200 // start of packet
#define RSR_EDP   0x0100 // end of packet

// Bits in the CSM register
#define CSM_IPOK  0x40 // IP Checkusm validatiaon ok
#define CSM_TUPOK 0x20 // TCP/UDP Checkusm validatiaon ok
#define CSM_FRAG  0x10 // Fragment IP datagram
#define CSM_IPKT  0x04 // Received an IP packet
#define CSM_TCPKT 0x02 // Received a TCP packet
#define CSM_UDPKT 0x01 // Received a UDP packet

// Bits in the TSR0 register
#define TSR0_ABT  0x0080 // Tx abort because of excessive collision
#define TSR0_OWT  0x0040 // Jumbo frame Tx abort
#define TSR0_OWC  0x0020 // Out of window collision
#define TSR0_COLS 0x0010 // experience collision in this transmit even
#define TSR0_NCR3 0x0008 // collision retry counter[3]
#define TSR0_NCR2 0x0004 // collision retry counter[2]
#define TSR0_NCR1 0x0002 // collision retry counter[1]
#define TSR0_NCR0 0x0001 // collision retry counter[0]
#define TSR0_TERR 0x8000
#define TSR0_FDX  0x4000 // current transaction is serviced by fulldup
#define TSR0_GMII 0x2000 // current transaction is serviced by GMII
#define TSR0_LNKFL 0x1000 // packet serviced during link down
#define TSR0_SHDN 0x0400 // shutdown case
#define TSR0_CRS  0x0200 // carrier sense lost
#define TSR0_CDH  0x0100 // AQE test fail (CD heartbeat)

// Bits in the TCR0 register
#define TCR0_TIC   0x80 // assert interrupt immediately
#define TCR0_PIC   0x40 // priority interrupt request
#define TCR0_VETAG 0x20 // enable VLAN tag
#define TCR0_IPCK  0x10 // request IP  checksum calculation
#define TCR0_UDPCK 0x08 // request UDP checksum calculation
#define TCR0_TCPCK 0x04 // request TCP checksum calculation
#define TCR0_JMBO  0x02 // indicate a jumbo packet in GMAC side
#define TCR0_CRC   0x01 // disable CRC generation

#define TCR_TCPLS_SOF 0x02000000
#define TCR_TCPLS_EOF 0x01000000
#define TCR_TIC       0x00800000
#define TCR_PIC       0x00400000
#define TCR_VETAG     0x00200000 // enable VLAN tag
#define TCR_IPCK      0x00100000 // request IP  checksum calculation
#define TCR_UDPCK     0x00080000 // request UDP checksum calculation
#define TCR_TCPCK     0x00040000 // request TCP checksum calculation

#define TCPLS_NORMAL 3
#define TCPLS_START  2
#define TCPLS_END    1
#define TCPLS_MED    0

//   Bits in the RCR register
#define RCR_AS   0x80 
#define RCR_AP   0x40 
#define RCR_AL   0x20 
#define RCR_PROM 0x10 
#define RCR_AB   0x08 
#define RCR_AM   0x04 
#define RCR_AR   0x02 
#define RCR_SEP  0x01 

//   Bits in the TCR register
#define TCR_TB2BDIS 0x80 
#define TCR_COLTMC1 0x08 
#define TCR_COLTMC0 0x04 
#define TCR_LB1     0x02
#define TCR_LB0     0x01

//   Bits in the CR0 register
#define CR0_TXON     0x00000008
#define CR0_RXON     0x00000004
#define CR0_STOP     0x00000002 // stop MAC, default = 1
#define CR0_STRT     0x00000001 // start MAC
#define CR0_SFRST    0x00008000 // software reset
#define CR0_TM1EN    0x00004000
#define CR0_TM0EN    0x00002000
#define CR0_DPOLL    0x00000800 // disable rx/tx auto polling
#define CR0_DISAU    0x00000100
#define CR0_XONEN    0x00800000
#define CR0_FDXTFCEN 0x00400000 // full-duplex TX flow control enable
#define CR0_FDXRFCEN 0x00200000 // full-duplex RX flow control enable
#define CR0_HDXFCEN  0x00100000 // half-duplex flow control enable
#define CR0_XHITH1   0x00080000 // TX XON high threshold 1
#define CR0_XHITH0   0x00040000 // TX XON high threshold 0
#define CR0_XLTH1    0x00020000 // TX pause frame low threshold 1
#define CR0_XLTH0    0x00010000 // TX pause frame low threshold 0
#define CR0_GSPRST   0x80000000
#define CR0_FORSRST  0x40000000
#define CR0_FPHYRST  0x20000000
#define CR0_DIAG     0x10000000
#define CR0_INTPCTL  0x04000000
#define CR0_GINTMSK1 0x02000000
#define CR0_GINTMSK0 0x01000000

#define ISRCTL_UDPINT  0x8000 
#define ISRCTL_TSUPDIS 0x4000 
#define ISRCTL_RSUPDIS 0x2000 
#define ISRCTL_PMSK1   0x1000 
#define ISRCTL_PMSK0   0x0800 
#define ISRCTL_INTPD   0x0400 
#define ISRCTL_HCRLD   0x0200 
#define ISRCTL_SCRLD   0x0100 

//   Bits in the ISR_CTL1 register
#define ISRCTL1_UDPINT  0x80 
#define ISRCTL1_TSUPDIS 0x40 
#define ISRCTL1_RSUPDIS 0x20 
#define ISRCTL1_PMSK1   0x10 
#define ISRCTL1_PMSK0   0x08 
#define ISRCTL1_INTPD   0x04 
#define ISRCTL1_HCRLD   0x02 
#define ISRCTL1_SCRLD   0x01 

//   Bits in the TXE_SR register
#define TXESR_TFDBS 0x08 
#define TXESR_TDWBS 0x04 
#define TXESR_TDRBS 0x02 
#define TXESR_TDSTR 0x01 

//   Bits in the RXE_SR register
#define RXESR_RFDBS 0x08 
#define RXESR_RDWBS 0x04 
#define RXESR_RDRBS 0x02 
#define RXESR_RDSTR 0x01 

//   Bits in the ISR register
#define ISR_ISR3   0x80000000
#define ISR_ISR2   0x40000000
#define ISR_ISR1   0x20000000
#define ISR_ISR0   0x10000000
#define ISR_TXSTLI 0x02000000
#define ISR_RXSTLI 0x01000000
#define ISR_HFLD   0x00800000
#define ISR_UDPI   0x00400000
#define ISR_MIBFI  0x00200000
#define ISR_SHDNI  0x00100000
#define ISR_PHYI   0x00080000
#define ISR_PWEI   0x00040000
#define ISR_TMR1I  0x00020000
#define ISR_TMR0I  0x00010000
#define ISR_SRCI   0x00008000
#define ISR_LSTPEI 0x00004000
#define ISR_LSTEI  0x00002000
#define ISR_OVFI   0x00001000
#define ISR_FLONI  0x00000800
#define ISR_RACEI  0x00000400
#define ISR_TXWB1I 0x00000200
#define ISR_TXWB0I 0x00000100
#define ISR_PTX3I  0x00000080
#define ISR_PTX2I  0x00000040
#define ISR_PTX1I  0x00000020
#define ISR_PTX0I  0x00000010
#define ISR_PTXI   0x00000008
#define ISR_PRXI   0x00000004
#define ISR_PPTXI  0x00000002
#define ISR_PPRXI  0x00000001

#define INT_MASK_DEF 0x037BFFFF

//   Bits in the TDCSR0/1, RDCSR0 register
#define TRDCSR_DEAD 0x0008 
#define TRDCSR_WAK  0x0004 
#define TRDCSR_ACT  0x0002 
#define TRDCSR_RUN  0x0001 

//   Bits in the CAMADDR register
#define CAMADDR_CAMEN  0x80
#define CAMADDR_VCAMSL 0x40

//   Bits in the CAMCR register
#define CAMCR_PS1         0x80
#define CAMCR_PS0         0x40
#define CAMCR_AITRPKT     0x20
#define CAMCR_AITR16      0x10
#define CAMCR_CAMRD       0x08
#define CAMCR_CAMWR       0x04
#define CAMCR_PS_CAM_MASK 0x40
#define CAMCR_PS_CAM_DATA 0x80
#define CAMCR_PS_MAR      0x00

//   Bits in the MIICFG register
#define MIICFG_MPO1 0x80 
#define MIICFG_MPO0 0x40 
#define MIICFG_MFDC 0x20 

//   Bits in the MIISR register
#define MIISR_MIDLE 0x80 

//   Bits in the PHYSR0 register
#define PHYSR0_PHYRST 0x80 
#define PHYSR0_LINKGD 0x40 
#define PHYSR0_FDPX   0x10 
#define PHYSR0_SPDG   0x08 
#define PHYSR0_SPD10  0x04 
#define PHYSR0_RXFLC  0x02 
#define PHYSR0_TXFLC  0x01 

//   Bits in the PHYSR1 register
#define PHYSR1_PHYTBI       0x01 

//   Bits in the MIICR register
#define MIICR_MAUTO 0x80 
#define MIICR_RCMD  0x40 
#define MIICR_WCMD  0x20 
#define MIICR_MDPM  0x10 
#define MIICR_MOUT  0x08 
#define MIICR_MDO   0x04 
#define MIICR_MDI   0x02 
#define MIICR_MDC   0x01 

//   Bits in the MIIADR register
#define MIIADR_SWMPL        0x80 

//   Bits in the CFGA register
#define CFGA_PMHCTG  0x08 
#define CFGA_GPIO1PD 0x04 
#define CFGA_ABSHDN  0x02 
#define CFGA_PACPI   0x01 

//   Bits in the CFGB register
#define CFGB_GTCKOPT 0x80
#define CFGB_MIIOPT  0x40
#define CFGB_CRSEOPT 0x20
#define CFGB_OFSET   0x10
#define CFGB_CRANDOM 0x08
#define CFGB_CAP     0x04
#define CFGB_MBA     0x02
#define CFGB_BAKOPT  0x01

//   Bits in the CFGC register
#define CFGC_EELOAD 0x80 
#define CFGC_BROPT  0x40 
#define CFGC_DLYEN  0x20 
#define CFGC_DTSEL  0x10 
#define CFGC_BTSEL  0x08 
#define CFGC_BPS2   0x04
#define CFGC_BPS1   0x02
#define CFGC_BPS0   0x01

//   Bits in the CFGD register
#define CFGD_IODIS     0x80
#define CFGD_MSLVDACEN 0x40
#define CFGD_CFGDACEN  0x20
#define CFGD_PCI64EN   0x10
#define CFGD_HTMRL4    0x08

//   Bits in the DCFG1 register (0x7D)
#define DCFG1_XMWI   0x80 
#define DCFG1_XMRM   0x40 
#define DCFG1_XMRL   0x20 
#define DCFG1_PERDIS 0x10 
#define DCFG1_MRWAIT 0x04 
#define DCFG1_MWWAIT 0x02 
#define DCFG1_LATMEN 0x01 

//   Bits in the MCFG0 register
#define MCFG_RXARB    0x0080 
#define MCFG_RFT1     0x0020 
#define MCFG_RFT0     0x0010 
#define MCFG_LOWTHOPT 0x0008 
#define MCFG_PQEN     0x0004 
#define MCFG_RTGOPT   0x0002 
#define MCFG_VIDFR    0x0001 

//   Bits in the MCFG1 register
#define MCFG_TXARB   0x8000 
#define MCFG_TXQBK1  0x0800 
#define MCFG_TXQBK0  0x0400 
#define MCFG_TXQNOBK 0x0200 
#define MCFG_SNAPOPT 0x0100 

//   Bits in the PMCC  register
#define PMCC_DSI    0x80
#define PMCC_D2_DIS 0x40
#define PMCC_D1_DIS 0x20
#define PMCC_D3C_EN 0x10
#define PMCC_D3H_EN 0x08
#define PMCC_D2_EN  0x04
#define PMCC_D1_EN  0x02
#define PMCC_D0_EN  0x01

//   Bits in STICKHW
#define STICKHW_SWPTAG 0x10
#define STICKHW_WOLSR  0x08
#define STICKHW_WOLEN  0x04
#define STICKHW_DS1    0x02 // R/W by software/cfg cycle
#define STICKHW_DS0    0x01 // suspend well DS write port

//   Bits in the MIBCR register
#define MIBCR_MIBISTOK 0x80
#define MIBCR_MIBISTGO 0x40
#define MIBCR_MIBINC   0x20
#define MIBCR_MIBHI    0x10
#define MIBCR_MIBFRZ   0x08
#define MIBCR_MIBFLSH  0x04
#define MIBCR_MPTRINI  0x02
#define MIBCR_MIBCLR   0x01

//   Bits in the EERSV register
#define EERSV_BOOT_RPL   0x01 // Boot method selection for VT6110
#define EERSV_BOOT_MASK  0x06
#define EERSV_BOOT_INT19 0x00
#define EERSV_BOOT_INT18 0x02
#define EERSV_BOOT_LOCAL 0x04
#define EERSV_BOOT_BEV   0x06

//   Bits in BPCMD
#define BPCMD_BPDNE 0x80
#define BPCMD_EBPWR 0x02
#define BPCMD_EBPRD 0x01

//   Bits in the EECSR register
#define EECSR_EMBP   0x40 // eeprom embeded programming
#define EECSR_RELOAD 0x20 // eeprom content reload
#define EECSR_DPM    0x10 // eeprom direct programming
#define EECSR_ECS    0x08 // eeprom CS pin
#define EECSR_ECK    0x04 // eeprom CK pin
#define EECSR_EDI    0x02 // eeprom DI pin
#define EECSR_EDO    0x01 // eeprom DO pin

//   Bits in the EMBCMD register
#define EMBCMD_EDONE 0x80
#define EMBCMD_EWDIS 0x08
#define EMBCMD_EWEN  0x04
#define EMBCMD_EWR   0x02
#define EMBCMD_ERD   0x01

//   Bits in TESTCFG register
#define TESTCFG_HBDIS 0x80

//   Bits in CHIPGCR register
#define CHIPGCR_FCGMII   0x80
#define CHIPGCR_FCFDX    0x40
#define CHIPGCR_FCRESV   0x20
#define CHIPGCR_FCMODE   0x10
#define CHIPGCR_LPSOPT   0x08
#define CHIPGCR_TM1US    0x04
#define CHIPGCR_TM0US    0x02
#define CHIPGCR_PHYINTEN 0x01

//   Bits in WOLCR0
#define WOLCR_MSWOLEN7 0x0080 // enable pattern match filtering
#define WOLCR_MSWOLEN6 0x0040
#define WOLCR_MSWOLEN5 0x0020
#define WOLCR_MSWOLEN4 0x0010
#define WOLCR_MSWOLEN3 0x0008
#define WOLCR_MSWOLEN2 0x0004
#define WOLCR_MSWOLEN1 0x0002
#define WOLCR_MSWOLEN0 0x0001
#define WOLCR_ARP_EN   0x0001

//   Bits in WOLCR1
#define WOLCR_LINKOFF_EN 0x0800 // link off detected enable
#define WOLCR_LINKON_EN  0x0400 // link on detected enable
#define WOLCR_MAGIC_EN   0x0200 // magic packet filter enable
#define WOLCR_UNICAST_EN 0x0100 // unicast filter enable

//  Bits in PWCFG
#define PWCFG_PHYPWOPT     0x80 // internal MII I/F timing
#define PWCFG_PCISTICK     0x40 // PCI sticky R/W enable
#define PWCFG_WOLTYPE      0x20 // pulse(1) or button (0)
#define PWCFG_LEGCY_WOL    0x10
#define PWCFG_PMCSR_PME_SR 0x08
#define PWCFG_PMCSR_PME_EN 0x04 // control by PCISTICK
#define PWCFG_LEGACY_WOLSR 0x02 // Legacy WOL_SR shadow
#define PWCFG_LEGACY_WOLEN 0x01 // Legacy WOL_EN shadow

//   Bits in WOLCFG
#define WOLCFG_PMEOVR   0x80 // for legacy use, force PMEEN always
#define WOLCFG_SAM      0x20 // accept multicast case reset, default=0
#define WOLCFG_SAB      0x10 // accept broadcast case reset, default=0
#define WOLCFG_SMIIACC  0x08
#define WOLCFG_SGENWH   0x02 
#define WOLCFG_PHYINTEN 0x01 // 0:PHYINT trigger enable, 1:use internal MII

//   Bits in WOLSR1
#define WOLSR_LINKOFF_INT 0x0800
#define WOLSR_LINKON_INT  0x0400
#define WOLSR_MAGIC_INT   0x0200
#define WOLSR_UNICAST_INT 0x0100

#define W_MAX_TIMEOUT 0x0FFF // max time out delay time


#define MCAM_SIZE           64
#define VCAM_SIZE           64

#define MAX_HW_MIB_COUNTER  32
#define VELOCITY_MIN_MTU    1500
#define VELOCITY_MAX_MTU    9000

#define DMA_ALIGN  64

enum VELOCITY_CAM_TYPE {
    VELOCITY_VLAN_ID_CAM = 0,
    VELOCITY_MULTICAST_CAM
};

#define RX_DESCS 256
#define TX_DESCS 256

#define RX_THRESH 0
// 0: 128 bytes
// 1: 512 bytes
// 2: 1024 bytes
// 3: store & forward

#define DMA_LENGTH 6
// 0: 8 DWORDs
// 1: 16 DWORDs
// 2: 32 DWORDs
// 3: 64 DWORDs
// 4: 128 DWORDs
// 5: 256 DWORDs
// 6: SF(flush till emply)
// 7: SF(flush till emply)


#define VELOCITY_LINK_FAIL          0x00000001
#define VELOCITY_SPEED_10           0x00000002
#define VELOCITY_SPEED_100          0x00000004
#define VELOCITY_SPEED_1000         0x00000008
#define VELOCITY_DUPLEX_FULL        0x00000010
#define VELOCITY_AUTONEG_ENABLE     0x00000020
#define VELOCITY_FORCED_BY_EEPROM   0x00000040
#define VELOCITY_LINK_CHANGE        0x00000001
#define VELOCITY_LINK_UNCHANGE      0x00000002

enum velocity_flow_cntl_type {
    FLOW_CNTL_DEFAULT = 1,  // 1
    FLOW_CNTL_TX,           // 2
    FLOW_CNTL_RX,           // 3
    FLOW_CNTL_TX_RX,        // 4
    FLOW_CNTL_DISABLE       // 5
};
#define FLOW_CNTL FLOW_CNTL_DEFAULT

#define MII_ON(x, i, hw) do {\
    u16 w;\
    velocity_mii_read((hw), (i), &w);\
    w |= (x);\
    velocity_mii_write((hw), (i), w);\
} while (0)

#define MII_OFF(x, i, hw) do {\
    u16 w;\
    velocity_mii_read((hw), (i), &w);\
    w &= (~(x));\
    velocity_mii_write((hw), (i), w);\
} while (0)


// Registers in the PCI configuration space
#define PCI_REG_COMMAND     0x04
#define PCI_REG_MODE0       0x60
#define PCI_REG_MODE1       0x61
#define PCI_REG_MODE2       0x62
#define PCI_REG_MODE3       0x63
#define PCI_REG_DELAY_TIMER 0x64

#define COMMAND_BUSM        0x04
#define COMMAND_WAIT        0x80

#define MODE0_QPKTDIS       0x80 // DISable transmit PacKeT Queuing

#define MODE2_PCEROPT       0x80 // take PCI bus ERror as a fatal and shutdown
#define MODE2_TXQ16         0x40 // TX write-back Queue control
#define MODE2_TXPOST        0x08 // (Not support in VT3119)
#define MODE2_AUTOOPT       0x04 // (VT3119 GHCI without such behavior)
#define MODE2_MODE10T       0x02 // used to control tx Threshold for 10M case
#define MODE2_TCPLSOPT      0x01 // TCP large send field update disable

#define MODE3_MIION         0x04 // MII symbol codine error detect enable ??



// receive descriptor
#define RDESC0_OWN    0x80000000
#define RDESC3_INTCTL 0x80000000

struct rx_desc {
    u32 desc0; // [31]: Owner, [29:16]: RMBC(receive packet length
               // [15:0]: RCR
    u32 desc1; // [31:24]: IPKT, [23:16]: CSM, [15:0]: PQTAG
    u32 addrLo;
    u32 desc3; // [31]: Intr control enable, [29:16]: buffer size
} __attribute__ ((__packed__));


// transmit descriptor
#define TDESC0_OWN  0x80000000
#define TDTXBUF_QUE 0x80000000
#define TDESC1_CMDZ 0xf0000000

struct td_buf {
    u32 addrLo;
    u32 addrHi; // [31]: Queue bit, [29:16]: buffer size
} __attribute__ ((__packed__));

struct tx_desc {
    u32 desc0; // [31]: Owner, [29:16]: tx packet size
               // [15:0]: TSR
    u32 desc1; // [31:28]: CMDZ, [27:26]: reserved, [25:24]: TCPLS
               // [23:16]: TCR, [15:13]: 802.1p priority bits,
               // [12]: CFI(reserved control bit), [11:0]: 802.1Q
    struct td_buf bufs[7];
} __attribute__ ((__packed__));

struct velocity_desc_info {
    struct sk_buff *skb;
    dma_addr_t skb_dma;
};

struct velocity_info {
    struct pci_dev *pcid;
    struct net_device *dev;
    struct napi_struct napi;

    void *pool;
    dma_addr_t pool_dma;
    dma_addr_t rd_pool_dma;
    dma_addr_t td_pool_dma;

    u8 *tx_zero_buf;
    dma_addr_t tx_zero_buf_dma;

    struct velocity_desc_info *td;
    struct velocity_desc_info *rd;

    long memaddr;
    long ioaddr;
    u32 io_size;
    u8 *hw_addr;

    u32 mii_status;

    unsigned tx_used;
    unsigned tx_curr;
    unsigned tx_tail;
    struct tx_desc *tdring;
    unsigned rx_curr;
    struct rx_desc *rdring;

    unsigned rx_buf_size;

    u8 vcamMask[VCAM_SIZE / 8];
    u8 mcamMask[MCAM_SIZE / 8];

    spinlock_t lock;

    unsigned isr_status;

    struct mii_if_info mii_info;
/*
    unsigned stat_intrs;
    unsigned stat_poll;
    unsigned stat_rx;
    unsigned stat_tx;
    unsigned long last_jiffies;
*/
};


#define PCI_ON1(x,i,p) do{\
    u8 byReg;\
    pci_read_config_byte((p), (i), &(byReg));\
    (byReg) |= (x);\
    pci_write_config_byte((p), (i), (byReg));\
} while (0)

