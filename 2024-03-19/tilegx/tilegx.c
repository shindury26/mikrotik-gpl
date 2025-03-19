/*
 * Copyright 2011 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 */

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skb_bin.h>
#include <linux/eth_stats.h>
#include <linux/phy_helper.h>
#include <linux/i2c_helper.h>
#include <linux/ethtool_extension.h>
#include <linux/interrupt.h>
#include <linux/packet_hook.h>
#include <linux/switch.h>
#include <linux/cycles.h>
#include <linux/mdio.h>
#include <linux/kmemleak.h>
#include <asm/homecache.h>
#include <asm/rb.h>
#include <gxio/mpipe.h>
#include <gxio/gpio.h>
#include <gxio/iorpc_mpipe.h>
#include <arch/mpipe_gbe.h>
#include <arch/mpipe_xaui.h>

MODULE_AUTHOR("Tilera");
MODULE_LICENSE("GPL");


#define MPIPE_SERDES_PRBS_CTRL 0x0064
#define MPIPE_SERDES_PCS_LPBK_CTRL 0x0074



#define TILE_CHANNELS_MAX (MPIPE_NUM_SGMII_MACS + MPIPE_NUM_LOOPBACK_CHANNELS)
#define TILE_DEVS_MAX 16
#define TILE_IQUEUE_LOG2 7
#define TILE_IQUEUE_SIZE (1 << TILE_IQUEUE_LOG2)
#define TILE_IQUEUE_MASK (TILE_IQUEUE_SIZE - 1)
#define TILE_JUMBO (10240 - ETH_HLEN - ETH_FCS_LEN) // limited by GMAC
//#define TILE_JUMBO (6144 - ETH_HLEN) // limited by DMA (1536 128-byte blocks for 16 channels)

#define MPIPE_IQUEUES_PER_CPU_MAX 2
#define BUCKETS_PER_DEV 256

#ifdef CONFIG_SLUB_DEBUG_ON
#define BUFFER_CACHE_OVERHEAD 128lu
#define BUFFER_CACHE_OFFSET 0
#define MASK_BUFFER(x) (void *)((unsigned long)x & ~(BUFFER_CACHE_OVERHEAD - 1))
#else
// buffer offset helps to improve cache hits/utilization (0x780 mask gives 16 positions with 128 byte increments - mpipe requires 128 byte aligned buffers) - this assumes kmalloc returns buffers that are aligned to its size
#define BUFFER_CACHE_OVERHEAD 2048lu
#define SPR_PSEUDO_RANDOM_NUMBER 0x270a
#define BUFFER_CACHE_OFFSET (__insn_mfspr(SPR_PSEUDO_RANDOM_NUMBER) & 0x780)
#define MASK_BUFFER(x) (void *)((unsigned long)x & ~skb_bin_state.off_mask)
#endif
#define MASK_BUFFER_CACHE_OVERHEAD(x) (void *)((unsigned long)x & ~(BUFFER_CACHE_OVERHEAD - 1))

#define TX_TIMER_INTERVAL_MIN 1000
#define TX_TIMER_INTERVAL_MAX 100000000
#define MAX_TX_QUEUE_SIZE_IN_MSEC 5
#define MIN_TX_QUEUE_SIZE_IN_BITS 1000000 // 64k ping is ~512kbits

#define MAX_PER_CPU_TX_QUEUE_SIZE 256
#define RECOMMENDED_PER_CPU_TX_QUEUE_SIZE 128
#define FAST_PATH_TX_QUEUE_LOG2 11
#define FAST_PATH_TX_QUEUE_SIZE (1 << FAST_PATH_TX_QUEUE_LOG2)
#define FAST_PATH_TX_QUEUE_MASK (FAST_PATH_TX_QUEUE_SIZE - 1)
#define XMIT_BATCH 16
#define MAX_FAST_PATH_TX_QUEUE_FILL_10G (1400)
#define MAX_FAST_PATH_TX_QUEUE_FILL_1G (500)
#define ADJUST_BUFFERS_BATCH 64

#define ROUND_UP(n, align) (((n) + (align) - 1) & -(align))


#define I2C_GPIO_BIT BIT(7)
#define I2C_GPIO(i2c_addr, gpio) (I2C_GPIO_BIT | ((i2c_addr) << 8) | gpio)

#define LATCH_GPIO_BIT BIT(8)
#define LATCH_GPIO(num) (LATCH_GPIO_BIT | (num))

#define IS_NATIVE_GPIO(x) ((x) && !((x) & (I2C_GPIO_BIT | LATCH_GPIO_BIT)))
#define IS_I2C_GPIO(x) ((x) && ((x) & I2C_GPIO_BIT))
#define IS_LATCH_GPIO(x) ((x) && ((x) & LATCH_GPIO_BIT) && !((x) & I2C_GPIO_BIT))


struct tile_port_config {
    const char *hv_name;
    const char *hv_name_xaui;
    struct phy_ops *phy_ops;
    int mdio;

    bool phy_1g;
    u16 gpio_phy_1g_int;

    bool sfp;
    bool sfpplus;
    bool combo;
    u16 gpio_sfp_abs;
    u16 gpio_sfp_rx_lose;
    u16 gpio_sfp_tx_fault;
    u16 gpio_sfp_tx_disable;
    u16 gpio_sfp_rate_select;
    u16 gpio_sfp_rate_select2;
    u16 gpio_sfp_led;
    u16 gpio_sfp_led2;
    bool gpio_sfp_led_invert;
    bool gpio_sfp_led2_invert;
    int sfp_i2c_adapter_nr;

    unsigned switch_type;
    int switch_port_map[6];
};

static struct phy_ops tile_sfp_ops;
static struct phy_ops tile_combo_ops;
static struct phy_ops tile_qt2025_ops;
static struct phy_ops tile_ti_ops;

static struct tile_port_config ccr1036_ports[] = {
    {
	.hv_name = "gbe0",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 63,
	.gpio_sfp_rx_lose = 34,
	.gpio_sfp_tx_fault = 38,
	.gpio_sfp_tx_disable = 42,
	.gpio_sfp_rate_select = 52,
	.gpio_sfp_led = 56,
	.sfp_i2c_adapter_nr = 3,
    },
    {
	.hv_name = "gbe1",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 62,
	.gpio_sfp_rx_lose = 35,
	.gpio_sfp_tx_fault = 39,
	.gpio_sfp_tx_disable = 43,
	.gpio_sfp_rate_select = 53,
	.gpio_sfp_led = 57,
	.sfp_i2c_adapter_nr = 4,
    },
    {
	.hv_name = "gbe2",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 61,
	.gpio_sfp_rx_lose = 36,
	.gpio_sfp_tx_fault = 40,
	.gpio_sfp_tx_disable = 44,
	.gpio_sfp_rate_select = 54,
	.gpio_sfp_led = 58,
	.sfp_i2c_adapter_nr = 5,
    },
    {
	.hv_name = "gbe3",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 60,
	.gpio_sfp_rx_lose = 37,
	.gpio_sfp_tx_fault = 41,
	.gpio_sfp_tx_disable = 45,
	.gpio_sfp_rate_select = 55,
	.gpio_sfp_led = 59,
	.sfp_i2c_adapter_nr = 6,
    },
    { .hv_name = "gbe4", .mdio = 0x23, .phy_1g = true, .gpio_phy_1g_int = 51 },
    { .hv_name = "gbe5", .mdio = 0x22, .phy_1g = true, .gpio_phy_1g_int = 51 },
    { .hv_name = "gbe6", .mdio = 0x21, .phy_1g = true, .gpio_phy_1g_int = 51 },
    { .hv_name = "gbe7", .mdio = 0x20, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe8", .mdio = 0x7, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe9", .mdio = 0x6, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe10", .mdio = 0x5, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe11", .mdio = 0x4, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe12", .mdio = 0x3, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe13", .mdio = 0x2, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe14", .mdio = 0x1, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe15", .mdio = 0x0, .phy_1g = true, .gpio_phy_1g_int = 48 },
    {}
};

static struct tile_port_config ccr1036r2_ports[] = {
    {
	.hv_name = "gbe0",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 60,
	.gpio_sfp_rx_lose = 37,
	.gpio_sfp_tx_fault = 41,
	.gpio_sfp_tx_disable = 45,
	.gpio_sfp_rate_select = 55,
	.gpio_sfp_led = 59,
	.sfp_i2c_adapter_nr = 4,
    },
    {
	.hv_name = "gbe1",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 61,
	.gpio_sfp_rx_lose = 36,
	.gpio_sfp_tx_fault = 40,
	.gpio_sfp_tx_disable = 44,
	.gpio_sfp_rate_select = 54,
	.gpio_sfp_led = 58,
	.sfp_i2c_adapter_nr = 3,
    },
    {
	.hv_name = "gbe2",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 62,
	.gpio_sfp_rx_lose = 34,
	.gpio_sfp_tx_fault = 39,
	.gpio_sfp_tx_disable = 43,
	.gpio_sfp_rate_select = 53,
	.gpio_sfp_led = 57,
	.sfp_i2c_adapter_nr = 6,
    },
    {
	.hv_name = "gbe3",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 63,
	.gpio_sfp_rx_lose = 35,
	.gpio_sfp_tx_fault = 38,
	.gpio_sfp_tx_disable = 42,
	.gpio_sfp_rate_select = 52,
	.gpio_sfp_led = 56,
	.sfp_i2c_adapter_nr = 5,
    },
    { .hv_name = "gbe4", .mdio = 0x03, .phy_1g = true,.gpio_phy_1g_int = 51, },
    { .hv_name = "gbe5", .mdio = 0x02, .phy_1g = true,.gpio_phy_1g_int = 51, },
    { .hv_name = "gbe6", .mdio = 0x01, .phy_1g = true,.gpio_phy_1g_int = 51, },
    { .hv_name = "gbe7", .mdio = 0x00, .phy_1g = true,.gpio_phy_1g_int = 50, },
    { .hv_name = "gbe8", .mdio = 0x27, .phy_1g = true,.gpio_phy_1g_int = 50, },
    { .hv_name = "gbe9", .mdio = 0x26, .phy_1g = true,.gpio_phy_1g_int = 50, },
    { .hv_name = "gbe10", .mdio = 0x25, .phy_1g = true,.gpio_phy_1g_int = 49, },
    { .hv_name = "gbe11", .mdio = 0x24, .phy_1g = true,.gpio_phy_1g_int = 49, },
    { .hv_name = "gbe12", .mdio = 0x23, .phy_1g = true,.gpio_phy_1g_int = 49, },
    { .hv_name = "gbe13", .mdio = 0x22, .phy_1g = true,.gpio_phy_1g_int = 48, },
    { .hv_name = "gbe14", .mdio = 0x21, .phy_1g = true,.gpio_phy_1g_int = 48, },
    { .hv_name = "gbe15", .mdio = 0x20, .phy_1g = true,.gpio_phy_1g_int = 48, },
    {}
};

static struct tile_port_config ccr1016r2_ports[] = {
    { .hv_name = "gbe4", .mdio = 0x03, .phy_1g = true,.gpio_phy_1g_int = 51, },
    { .hv_name = "gbe5", .mdio = 0x02, .phy_1g = true,.gpio_phy_1g_int = 51, },
    { .hv_name = "gbe6", .mdio = 0x01, .phy_1g = true,.gpio_phy_1g_int = 51, },
    { .hv_name = "gbe7", .mdio = 0x00, .phy_1g = true,.gpio_phy_1g_int = 50, },
    { .hv_name = "gbe8", .mdio = 0x27, .phy_1g = true,.gpio_phy_1g_int = 50, },
    { .hv_name = "gbe9", .mdio = 0x26, .phy_1g = true,.gpio_phy_1g_int = 50, },
    { .hv_name = "gbe10", .mdio = 0x25, .phy_1g = true,.gpio_phy_1g_int = 49, },
    { .hv_name = "gbe11", .mdio = 0x24, .phy_1g = true,.gpio_phy_1g_int = 49, },
    { .hv_name = "gbe12", .mdio = 0x23, .phy_1g = true,.gpio_phy_1g_int = 49, },
    { .hv_name = "gbe13", .mdio = 0x22, .phy_1g = true,.gpio_phy_1g_int = 48, },
    { .hv_name = "gbe14", .mdio = 0x21, .phy_1g = true,.gpio_phy_1g_int = 48, },
    { .hv_name = "gbe15", .mdio = 0x20, .phy_1g = true,.gpio_phy_1g_int = 48, },
    {}
};

static struct tile_port_config ccr1016_ports[] = {
    { .hv_name = "gbe4", .mdio = 0x23, .phy_1g = true, .gpio_phy_1g_int = 51 },
    { .hv_name = "gbe5", .mdio = 0x22, .phy_1g = true, .gpio_phy_1g_int = 51 },
    { .hv_name = "gbe6", .mdio = 0x21, .phy_1g = true, .gpio_phy_1g_int = 51 },
    { .hv_name = "gbe7", .mdio = 0x20, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe8", .mdio = 0x7, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe9", .mdio = 0x6, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe10", .mdio = 0x5, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe11", .mdio = 0x4, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe12", .mdio = 0x3, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe13", .mdio = 0x2, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe14", .mdio = 0x1, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe15", .mdio = 0x0, .phy_1g = true, .gpio_phy_1g_int = 48 },
    {}
};

static struct tile_port_config ccr1036_8g_2splus_ports[] = {
    { .hv_name = "gbe16", .hv_name_xaui = "xgbe16",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 0,
      .sfpplus = true,
    },
    { .hv_name = "gbe17", .hv_name_xaui = "xgbe17",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 1,
      .sfpplus = true,
    },
    { .hv_name = "gbe8", .mdio = 0x27, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe9", .mdio = 0x26, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe10", .mdio = 0x25, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe11", .mdio = 0x24, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe12", .mdio = 0x3, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe13", .mdio = 0x2, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe14", .mdio = 0x1, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe15", .mdio = 0x0, .phy_1g = true, .gpio_phy_1g_int = 48 },
    {}
};

static struct tile_port_config ccr1036r2_8g_2splus_ports[] = {
    { .hv_name = "gbe16", .hv_name_xaui = "xgbe16",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 0,
      .sfpplus = true,
    },
    { .hv_name = "gbe17", .hv_name_xaui = "xgbe17",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 1,
      .sfpplus = true,
    },
    { .hv_name = "gbe8", .mdio = 0x7, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe9", .mdio = 0x6, .phy_1g = true, .gpio_phy_1g_int = 50 },
    { .hv_name = "gbe10", .mdio = 0x5, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe11", .mdio = 0x4, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe12", .mdio = 0x26, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe13", .mdio = 0x22, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe14", .mdio = 0x21, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe15", .mdio = 0x24, .phy_1g = true, .gpio_phy_1g_int = 48 },
    {}
};

static struct tile_port_config ccr1072_1g_8splus_ports[] = {
    { .hv_name = "gbe43", .hv_name_xaui = "xgbe43",
      .phy_ops = &tile_ti_ops,
      .mdio = 17,
      .sfpplus = true,
      .gpio_sfp_abs = LATCH_GPIO(26),
      .gpio_sfp_rx_lose = LATCH_GPIO(27),
      .gpio_sfp_tx_fault = LATCH_GPIO(25),
      .gpio_sfp_tx_disable = LATCH_GPIO(39),
      .gpio_sfp_rate_select = LATCH_GPIO(38),
      .gpio_sfp_rate_select2 = LATCH_GPIO(37),
      .gpio_sfp_led = LATCH_GPIO(36),
      .gpio_sfp_led2 = LATCH_GPIO(7),
      .gpio_sfp_led_invert = true,
      .gpio_sfp_led2_invert = true,
      .sfp_i2c_adapter_nr = 3,
    },
    { .hv_name = "gbe42", .hv_name_xaui = "xgbe42",
      .phy_ops = &tile_ti_ops,
      .mdio = 16,
      .sfpplus = true,
      .gpio_sfp_abs = LATCH_GPIO(30),
      .gpio_sfp_rx_lose = LATCH_GPIO(31),
      .gpio_sfp_tx_fault = LATCH_GPIO(29),
      .gpio_sfp_tx_disable = LATCH_GPIO(35),
      .gpio_sfp_rate_select = LATCH_GPIO(34),
      .gpio_sfp_rate_select2 = LATCH_GPIO(33),
      .gpio_sfp_led = LATCH_GPIO(32),
      .gpio_sfp_led2 = LATCH_GPIO(6),
      .gpio_sfp_led_invert = true,
      .gpio_sfp_led2_invert = true,
      .sfp_i2c_adapter_nr = 4,
    },
    { .hv_name = "gbe41", .hv_name_xaui = "xgbe41",
      .phy_ops = &tile_ti_ops,
      .mdio = 1,
      .sfpplus = true,
      .gpio_sfp_abs = LATCH_GPIO(18),
      .gpio_sfp_rx_lose = LATCH_GPIO(19),
      .gpio_sfp_tx_fault = LATCH_GPIO(17),
      .gpio_sfp_tx_disable = LATCH_GPIO(31),
      .gpio_sfp_rate_select = LATCH_GPIO(30),
      .gpio_sfp_rate_select2 = LATCH_GPIO(29),
      .gpio_sfp_led = LATCH_GPIO(28),
      .gpio_sfp_led2 = LATCH_GPIO(5),
      .gpio_sfp_led_invert = true,
      .gpio_sfp_led2_invert = true,
      .sfp_i2c_adapter_nr = 5,
    },
    { .hv_name = "gbe40", .hv_name_xaui = "xgbe40",
      .phy_ops = &tile_ti_ops,
      .mdio = 0,
      .sfpplus = true,
      .gpio_sfp_abs = LATCH_GPIO(22),
      .gpio_sfp_rx_lose = LATCH_GPIO(23),
      .gpio_sfp_tx_fault = LATCH_GPIO(21),
      .gpio_sfp_tx_disable = LATCH_GPIO(27),
      .gpio_sfp_rate_select = LATCH_GPIO(26),
      .gpio_sfp_rate_select2 = LATCH_GPIO(25),
      .gpio_sfp_led = LATCH_GPIO(24),
      .gpio_sfp_led2 = LATCH_GPIO(4),
      .gpio_sfp_led_invert = true,
      .gpio_sfp_led2_invert = true,
      .sfp_i2c_adapter_nr = 6,
    },

    { .hv_name = "gbe16", .hv_name_xaui = "xgbe16",
      .phy_ops = &tile_ti_ops,
      .mdio = 1,
      .sfpplus = true,
      .gpio_sfp_abs = LATCH_GPIO(10),
      .gpio_sfp_rx_lose = LATCH_GPIO(11),
      .gpio_sfp_tx_fault = LATCH_GPIO(9),
      .gpio_sfp_tx_disable = LATCH_GPIO(23),
      .gpio_sfp_rate_select = LATCH_GPIO(22),
      .gpio_sfp_rate_select2 = LATCH_GPIO(21),
      .gpio_sfp_led = LATCH_GPIO(20),
      .gpio_sfp_led2 = LATCH_GPIO(3),
      .gpio_sfp_led_invert = true,
      .gpio_sfp_led2_invert = true,
      .sfp_i2c_adapter_nr = 7,
    },
    { .hv_name = "gbe17", .hv_name_xaui = "xgbe17",
      .phy_ops = &tile_ti_ops,
      .mdio = 0,
      .sfpplus = true,
      .gpio_sfp_abs = LATCH_GPIO(14),
      .gpio_sfp_rx_lose = LATCH_GPIO(15),
      .gpio_sfp_tx_fault = LATCH_GPIO(13),
      .gpio_sfp_tx_disable = LATCH_GPIO(19),
      .gpio_sfp_rate_select = LATCH_GPIO(18),
      .gpio_sfp_rate_select2 = LATCH_GPIO(17),
      .gpio_sfp_led = LATCH_GPIO(16),
      .gpio_sfp_led2 = LATCH_GPIO(2),
      .gpio_sfp_led_invert = true,
      .gpio_sfp_led2_invert = true,
      .sfp_i2c_adapter_nr = 8,
    },
    { .hv_name = "gbe18", .hv_name_xaui = "xgbe18",
      .phy_ops = &tile_ti_ops,
      .mdio = 17,
      .sfpplus = true,
      .gpio_sfp_abs = LATCH_GPIO(2),
      .gpio_sfp_rx_lose = LATCH_GPIO(3),
      .gpio_sfp_tx_fault = LATCH_GPIO(1),
      .gpio_sfp_tx_disable = LATCH_GPIO(15),
      .gpio_sfp_rate_select = LATCH_GPIO(14),
      .gpio_sfp_rate_select2 = LATCH_GPIO(13),
      .gpio_sfp_led = LATCH_GPIO(12),
      .gpio_sfp_led2 = LATCH_GPIO(1),
      .gpio_sfp_led_invert = true,
      .gpio_sfp_led2_invert = true,
      .sfp_i2c_adapter_nr = 9,
    },
    { .hv_name = "gbe19", .hv_name_xaui = "xgbe19",
      .phy_ops = &tile_ti_ops,
      .mdio = 16,
      .sfpplus = true,
      .gpio_sfp_abs = LATCH_GPIO(6),
      .gpio_sfp_rx_lose = LATCH_GPIO(7),
      .gpio_sfp_tx_fault = LATCH_GPIO(5),
      .gpio_sfp_tx_disable = LATCH_GPIO(11),
      .gpio_sfp_rate_select = LATCH_GPIO(10),
      .gpio_sfp_rate_select2 = LATCH_GPIO(9),
      .gpio_sfp_led = LATCH_GPIO(8),
      .gpio_sfp_led2 = LATCH_GPIO(0),
      .gpio_sfp_led_invert = true,
      .gpio_sfp_led2_invert = true,
      .sfp_i2c_adapter_nr = 10,
    },

    {}
};

static struct tile_port_config ccr1072_1g_8splus_r3_ports[] = {
    { .hv_name = "gbe43", .hv_name_xaui = "xgbe43",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 0,
      .sfpplus = true,
    },
    { .hv_name = "gbe42", .hv_name_xaui = "xgbe42",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 1,
      .sfpplus = true,
    },
    { .hv_name = "gbe41", .hv_name_xaui = "xgbe41",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 16,
      .sfpplus = true,
    },
    { .hv_name = "gbe40", .hv_name_xaui = "xgbe40",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 17,
      .sfpplus = true,
    },

    { .hv_name = "gbe16", .hv_name_xaui = "xgbe16",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 0,
      .sfpplus = true,
    },
    { .hv_name = "gbe17", .hv_name_xaui = "xgbe17",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 1,
      .sfpplus = true,
    },
    { .hv_name = "gbe18", .hv_name_xaui = "xgbe18",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 16,
      .sfpplus = true,
    },
    { .hv_name = "gbe19", .hv_name_xaui = "xgbe19",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 17,
      .sfpplus = true,
    },

    {}
};

#define GPIO_EXP_48 0
#define GPIO_EXP_4A 1
#define GPIO_EXP_4C 2
#define GPIO_EXP_4E 3
#define GPIO_EXP_COUNT 4
static struct tile_port_config ccr1036_12s_1splus_ports[] = {
    { .hv_name = "gbe16", .hv_name_xaui = "xgbe16",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 0,
      .sfpplus = true,
    },
    {
	.hv_name = "gbe4",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_4C, 12),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_4C, 8),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 12),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_4C, 0),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_4C, 4),
	.gpio_sfp_led = 33,
	.sfp_i2c_adapter_nr = 11,
    },
    {
	.hv_name = "gbe5",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_4C, 13),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_4C, 9),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 13),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_4C, 1),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_4C, 5),
	.gpio_sfp_led = 34,
	.sfp_i2c_adapter_nr = 12,
    },
    {
	.hv_name = "gbe6",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_4C, 14),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_4C, 10),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 14),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_4C, 2),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_4C, 6),
	.gpio_sfp_led = 35,
	.sfp_i2c_adapter_nr = 13,
    },
    {
	.hv_name = "gbe7",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_4C, 15),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_4C, 11),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 15),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_4C, 3),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_4C, 7),
	.gpio_sfp_led = 36,
	.sfp_i2c_adapter_nr = 14,
    },
    {
	.hv_name = "gbe8",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_4A, 12),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_4A, 8),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 0),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_4A, 0),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_4A, 4),
	.gpio_sfp_led = 38,
	.sfp_i2c_adapter_nr = 7,
    },
    {
	.hv_name = "gbe9",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_4A, 13),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_4A, 9),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 1),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_4A, 1),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_4A, 5),
	.gpio_sfp_led = 39,
	.sfp_i2c_adapter_nr = 8,
    },
    {
	.hv_name = "gbe10",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_4A, 14),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_4A, 10),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 2),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_4A, 2),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_4A, 6),
	.gpio_sfp_led = 40,
	.sfp_i2c_adapter_nr = 9,
    },
    {
	.hv_name = "gbe11",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_4A, 15),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_4A, 11),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 3),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_4A, 3),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_4A, 7),
	.gpio_sfp_led = 41,
	.sfp_i2c_adapter_nr = 10,
    },
    {
	.hv_name = "gbe12",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_48, 12),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_48, 8),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 4),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_48, 0),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_48, 4),
	.gpio_sfp_led = 42,
	.sfp_i2c_adapter_nr = 3,
    },
    {
	.hv_name = "gbe13",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_48, 13),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_48, 9),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 5),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_48, 1),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_48, 5),
	.gpio_sfp_led = 43,
	.sfp_i2c_adapter_nr = 4,
    },
    {
	.hv_name = "gbe14",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_48, 14),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_48, 10),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 6),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_48, 2),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_48, 6),
	.gpio_sfp_led = 44,
	.sfp_i2c_adapter_nr = 5,
    },
    {
	.hv_name = "gbe15",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = I2C_GPIO(GPIO_EXP_48, 15),
	.gpio_sfp_rx_lose = I2C_GPIO(GPIO_EXP_48, 11),
	.gpio_sfp_tx_fault = I2C_GPIO(GPIO_EXP_4E, 7),
	.gpio_sfp_tx_disable = I2C_GPIO(GPIO_EXP_48, 3),
	.gpio_sfp_rate_select = I2C_GPIO(GPIO_EXP_48, 7),
	.gpio_sfp_led = 45,
	.sfp_i2c_adapter_nr = 6,
    },
    {}
};

static struct tile_port_config ccr1009_10g[] = {
    { .hv_name = "gbe16", .hv_name_xaui = "xgbe16",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 0,
      .sfpplus = true,
    },
    {
	.hv_name = "gbe4",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 62,
	.gpio_sfp_rx_lose = 61,
	.gpio_sfp_tx_fault = 60,
	.gpio_sfp_tx_disable = 45,
	.gpio_sfp_rate_select = 43,
	.gpio_sfp_led = 44,
	.sfp_i2c_adapter_nr = 1,
    },
    {
	.hv_name = "gbe5",
	.mdio = 0x20,
	.switch_type = SWITCH_ATHEROS8327,
	.switch_port_map = { 0, 1, 2, 3, -1 },
    },
    { .hv_name = "gbe6", .mdio = 0x3, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe7", .mdio = 0x2, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe8", .mdio = 0x1, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe9", .mdio = 0x0, .phy_1g = true, .gpio_phy_1g_int = 48 },
    {}
};

static struct tile_port_config ccr1009_8g_1s[] = {
    {
	.hv_name = "gbe4",
	.phy_ops = &tile_sfp_ops,
	.mdio = -1,
	.sfp = true,
	.gpio_sfp_abs = 62,
	.gpio_sfp_rx_lose = 61,
	.gpio_sfp_tx_fault = 60,
	.gpio_sfp_tx_disable = 45,
	.gpio_sfp_rate_select = 43,
	.gpio_sfp_led = 44,
	.sfp_i2c_adapter_nr = 1,
    },
    {
	.hv_name = "gbe5",
	.mdio = 0x20,
	.switch_type = SWITCH_ATHEROS8327,
	.switch_port_map = { 0, 1, 2, 3, -1 },
    },
    { .hv_name = "gbe6", .mdio = 0x3, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe7", .mdio = 0x2, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe8", .mdio = 0x1, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe9", .mdio = 0x0, .phy_1g = true, .gpio_phy_1g_int = 48 },
    {}
};

static struct tile_port_config ccr1009_8g_1sc_1splus[] = {
    { .hv_name = "gbe16", .hv_name_xaui = "xgbe16",
      .phy_ops = &tile_qt2025_ops,
      .mdio = 0,
      .sfpplus = true,
    },
    {
	.hv_name = "gbe4",
	.phy_ops = &tile_combo_ops,
	.mdio = 0x20,
	.phy_1g = true,
	.sfp = true,
	.combo = true,
	.gpio_sfp_abs = 62,
	.gpio_sfp_rx_lose = 61,
	.gpio_sfp_tx_fault = 60,
	.gpio_sfp_tx_disable = 45,
	.gpio_sfp_rate_select = 43,
	.gpio_sfp_led = 44,
	.sfp_i2c_adapter_nr = 1,
    },
    { .hv_name = "gbe5", .mdio = 0x6, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe6", .mdio = 0x5, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe7", .mdio = 0x4, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe8", .mdio = 0x3, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe9", .mdio = 0x2, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe10", .mdio = 0x1, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe11", .mdio = 0x0, .phy_1g = true, .gpio_phy_1g_int = 48 },
    {}
};

static struct tile_port_config ccr1009_7g_1sc[] = {
    {
	.hv_name = "gbe4",
	.phy_ops = &tile_combo_ops,
	.mdio = 0x20,
	.phy_1g = true,
	.sfp = true,
	.combo = true,
	.gpio_sfp_abs = 62,
	.gpio_sfp_rx_lose = 61,
	.gpio_sfp_tx_fault = 60,
	.gpio_sfp_tx_disable = 45,
	.gpio_sfp_rate_select = 43,
	.gpio_sfp_led = 44,
	.sfp_i2c_adapter_nr = 1,
    },
    { .hv_name = "gbe5", .mdio = 0x6, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe6", .mdio = 0x5, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe7", .mdio = 0x4, .phy_1g = true, .gpio_phy_1g_int = 49 },
    { .hv_name = "gbe8", .mdio = 0x3, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe9", .mdio = 0x2, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe10", .mdio = 0x1, .phy_1g = true, .gpio_phy_1g_int = 48 },
    { .hv_name = "gbe11", .mdio = 0x0, .phy_1g = true, .gpio_phy_1g_int = 48 },
    {}
};

static struct tile_port_config *ports_config;


struct tile_stats {
//#define STATS
#ifdef STATS
    unsigned stats_xmit;
    unsigned stats_xmit_fast;
    unsigned stats_xmit_fast_drop;
    unsigned stats_tx_queue_stop;
    unsigned stats_tx_work;
    unsigned stats_tx;
    unsigned stats_tx_recycle;
    unsigned stats_rx_buffer_error;
    unsigned stats_tx_hw;
    unsigned stats_rx;
#endif
};


#define SLOT_BITS 59

struct tile_tx_entry {
    u64 slot;
    struct sk_buff *buf;
    unsigned bits;
};

struct tile_tx_queue {
    unsigned max_queued_bits;
    unsigned max_cycles_between_tx;
    unsigned edma_ring_size;
    int edma;
    unsigned equeue_order;
    struct page *equeue_pages;
    gxio_mpipe_equeue_t equeue;

    atomic64_t slot ____cacheline_aligned_in_smp;
    unsigned queued_bits;
};

struct tile_cpu_dev_info {
    struct net_device *dev;
    struct tile_stats stats;
#ifdef STATS
    struct tile_stats stats2;
#endif
    unsigned cpu;
    unsigned short tx_tail;
    unsigned short tx_head;

    struct tile_tx_entry *tx_entries;

    struct work_struct tx_work;
    struct hrtimer tx_timer;
    unsigned tx_timer_interval;
    unsigned long long last_tx_time;
    u64 completed;
    unsigned xmit_buf_count;
    bool xmit_commit_scheduled;
    struct fp_buf xmit_bufs[XMIT_BATCH];
};

struct tile_cpu_stats {
    unsigned stats_intr;
    unsigned stats_poll;
    unsigned stats_push;
    unsigned stats_pop;
};

struct tile_rx_queue {
    struct tile_mpipe *mpipe;
    gxio_mpipe_iqueue_t iqueue;
    struct napi_struct napi;
};

struct tile_cpu_mpipe_info {
    unsigned iqueue_order;
    struct page *iqueue_pages;
    int adjust_buffers_counters;
};

struct tile_cpu_info {
    struct tile_cpu_mpipe_info mpipe_info[NR_MPIPE_MAX];
    struct tile_rx_queue rxq[NR_MPIPE_MAX * MPIPE_IQUEUES_PER_CPU_MAX];
    struct tile_cpu_dev_info dev_info[TILE_DEVS_MAX];

#ifdef STATS
    struct tile_cpu_stats stats;
    struct tile_cpu_stats stats2;
#endif
//#define DEBUG
#ifdef DEBUG
    atomic64_t xmit_count;
#endif
};

struct tile_priv {
    struct switch_mac sw;
    struct tile_mpipe *mpipe;
    struct tile_port_config *config;
    struct i2c_sfp i2c_sfp;
    struct phy_led led;
    struct phy_led led2;
    int devno;
    unsigned long last_open_jiffies;
    unsigned long rx_lose_down_jiffies;
    unsigned long autoneg_start_jiffies;
    bool closing;
    bool xaui;
    bool sgmii;
    gxio_mpipe_link_t link;

    unsigned equeue_size;
    unsigned per_cpu_tx_queue_size;
    unsigned max_fast_queue_fill;
    struct tile_tx_queue tx_queue;
    struct tile_tx_queue tx_queue_fast;
    struct tile_tx_entry __percpu *tx_entries;
};


struct tile_mpipe {
    unsigned instance;
    struct cpumask rx_cpus_map;
    unsigned rx_cpus_count;
    unsigned tx_cpus_count;
    unsigned iqueues_per_cpu;
    unsigned buffers_capacity;
    unsigned buffers_min;
    unsigned buffers_max;
    unsigned hw_tx_queue_count;
    unsigned notif_ring_count;
    unsigned notif_group_count;
    unsigned bucket_count;
    unsigned dev_1g_count;
    unsigned dev_10g_count;
    gxio_mpipe_context_t ctx;

    struct page *buffer_stack_pages;
    unsigned buffer_stack_bytes;

    int open_devs;
    int first_bucket;
    int rr_bucket;
    int ingress_irq[MPIPE_IQUEUES_PER_CPU_MAX];
    struct net_device *devs_for_channel[TILE_CHANNELS_MAX];
    struct net_device *active_devs_for_channel[TILE_CHANNELS_MAX];

    spinlock_t adjust_buffers_lock ____cacheline_aligned_in_smp;
    int buffers_current_approx;
};

struct tile_state {
    struct net_device *dummy_netdev;

    int open_mpipes;
    unsigned l2mtu;
    int mpipe_buffer_size;
    gxio_mpipe_buffer_size_enum_t mpipe_buffer_size_enum;

    struct tile_mpipe mpipe[NR_MPIPE_MAX];
    struct net_device *devs[TILE_DEVS_MAX];
    struct tile_cpu_info *cpu_info[NR_CPUS];
    int irq_to_rxq[NR_IRQS];

    spinlock_t mdio_lock;
    gxio_gpio_context_t gpio_ctx;
    struct i2c_gpio_expander gpio_expander[GPIO_EXP_COUNT];
    int phy_1g_irq;
    struct work_struct phy_1g_irq_work;

#ifdef STATS
    unsigned long stats_last_jiffies;
    spinlock_t stats_lock;
#endif
};
static DEFINE_PER_CPU(struct tile_cpu_info, per_cpu_info);

static struct tile_state state;



//#define STATS_TIMESTAMPS
#ifdef STATS_TIMESTAMPS
struct tile_timestamp {
    u8 cpu;
    u16 sec;
    u32 nsec;
    u32 flow;
};
#define MAX_TIME_STAMPS 200 * 1000
static struct tile_timestamp stamps[MAX_TIME_STAMPS];
static atomic_t stamps_read;
static atomic_t stamps_filled;
#endif


#ifdef STATS
#define STATS_INTERVAL (1 * HZ)

static void tile_add_stats(struct tile_stats *res, struct tile_stats *add) {
    res->stats_xmit += add->stats_xmit;
    res->stats_xmit_fast += add->stats_xmit_fast;
    res->stats_xmit_fast_drop += add->stats_xmit_fast_drop;
    res->stats_tx_queue_stop += add->stats_tx_queue_stop;
    res->stats_tx_work += add->stats_tx_work;
    res->stats_tx += add->stats_tx;
    res->stats_tx_recycle += add->stats_tx_recycle;
    res->stats_rx_buffer_error += add->stats_rx_buffer_error;
    res->stats_tx_hw += add->stats_tx_hw;
    res->stats_rx += add->stats_rx;
}

static void tile_add_cpu_stats(struct tile_cpu_stats *res,
	struct tile_cpu_stats *add) {
    res->stats_intr += add->stats_intr;
    res->stats_poll += add->stats_poll;
    res->stats_push += add->stats_push;
    res->stats_pop += add->stats_pop;
}

static void tile_print_stats_header(struct tile_stats *stats_tot,
	struct tile_cpu_stats *cpu_stats_tot) {
    printk("      ");
    if (stats_tot->stats_rx_buffer_error) {
	printk(" rx_buf_e");
    }
    if (stats_tot->stats_xmit) {
	printk("     xmit");
    }
    if (stats_tot->stats_xmit_fast) {
	printk(" xmit_fst");
    }
    if (stats_tot->stats_xmit_fast_drop) {
	printk(" xm_f_drp");
    }
    if (stats_tot->stats_tx_queue_stop) {
	printk(" tx_q_stp");
    }
    if (stats_tot->stats_tx_work) {
	printk("  tx_work");
    }
    if (stats_tot->stats_tx) {
	printk("       tx");
    }
    if (stats_tot->stats_tx_recycle) {
	printk(" tx_recyc");
    }
    if (stats_tot->stats_rx) {
	printk("       rx");
    }
    if (stats_tot->stats_tx_hw) {
	printk("    tx_hw");
    }
    if (cpu_stats_tot) {
	if (cpu_stats_tot->stats_intr) {
	    printk("      irq");
	}
	if (cpu_stats_tot->stats_poll) {
	    printk("     poll");
	}
	if (cpu_stats_tot->stats_push) {
	    printk("     push");
	}
	if (cpu_stats_tot->stats_pop) {
	    printk("      pop");
	}
    }
    printk("\n");
}

static void tile_print_stats(const char *name,
	struct tile_stats *stats_tot,
	struct tile_stats *stats,
	struct tile_cpu_stats *cpu_stats_tot,
	struct tile_cpu_stats *cpu_stats) {
    printk("% 5s:", name);
    if (stats_tot->stats_rx_buffer_error) {
	printk(" % 8u", stats->stats_rx_buffer_error);
    }
    if (stats_tot->stats_xmit) {
	printk(" % 8u", stats->stats_xmit);
    }
    if (stats_tot->stats_xmit_fast) {
	printk(" % 8u", stats->stats_xmit_fast);
    }
    if (stats_tot->stats_xmit_fast_drop) {
	printk(" % 8u", stats->stats_xmit_fast_drop);
    }
    if (stats_tot->stats_tx_queue_stop) {
	printk(" % 8u", stats->stats_tx_queue_stop);
    }
    if (stats_tot->stats_tx_work) {
	printk(" % 8u", stats->stats_tx_work);
    }
    if (stats_tot->stats_tx) {
	printk(" % 8u", stats->stats_tx);
    }
    if (stats_tot->stats_tx_recycle) {
	printk(" % 8u", stats->stats_tx_recycle);
    }
    if (stats_tot->stats_rx) {
	printk(" % 8u", stats->stats_rx);
    }
    if (stats_tot->stats_tx_hw) {
	printk(" % 8u", stats->stats_tx_hw);
    }
    if (cpu_stats) {
	if (cpu_stats_tot->stats_intr) {
	    printk(" % 8u", cpu_stats->stats_intr);
	}
	if (cpu_stats_tot->stats_poll) {
	    printk(" % 8u", cpu_stats->stats_poll);
	}
	if (cpu_stats_tot->stats_push) {
	    printk(" % 8u", cpu_stats->stats_push);
	}
	if (cpu_stats_tot->stats_pop) {
	    printk(" % 8u", cpu_stats->stats_pop);
	}
    }
    printk("\n");
}

static bool tile_stats_empty(struct tile_stats *stats,
	struct tile_cpu_stats *cpu_stats) {
    struct tile_stats se = { 0 };
    struct tile_cpu_stats cse = { 0 };
    if (!memcmp(stats, &se, sizeof(se)) &&
	    (!cpu_stats || !memcmp(cpu_stats, &cse, sizeof(cse)))) {
	return true;
    }
    return false;
}

static void tile_reset_stats(struct tile_stats *stats) {
    unsigned off = offsetof(struct tile_stats, stats_xmit);
    memset((unsigned char *)stats + off, 0, sizeof(struct tile_stats) - off);
}

static void tile_reset_cpu_stats(struct tile_cpu_stats *cpu_stats) {
    memset(cpu_stats, 0, sizeof(struct tile_cpu_stats));
}

static void tile_stats(void) {
    unsigned i;
    unsigned j;
    struct tile_stats total;
    struct tile_stats x;
    struct tile_cpu_stats cpu_total;
    tile_reset_stats(&total);
    tile_reset_cpu_stats(&cpu_total);

//    printk("tile_stats\n");
    if (!state.stats_last_jiffies) {
	state.stats_last_jiffies = jiffies;
    }

    spin_lock(&state.stats_lock);
    if ((long)(state.stats_last_jiffies + STATS_INTERVAL - jiffies) > 0) {
	spin_unlock(&state.stats_lock);
	return;
    }
    state.stats_last_jiffies = jiffies;
    spin_unlock(&state.stats_lock);

    for (j = 0; j < NR_CPUS; ++j) {
	if (!state.cpu_info[j]) {
	    continue;
	}
	for (i = 0; i < TILE_DEVS_MAX; ++i) {
	    struct net_device *dev = state.devs[i];
	    if (!dev) {
		continue;
	    }
	    memcpy(&state.cpu_info[j]->dev_info[i].stats2,
		    &state.cpu_info[j]->dev_info[i].stats,
		    sizeof(struct tile_stats));
	    tile_reset_stats(&state.cpu_info[j]->dev_info[i].stats);
	}
	memcpy(&state.cpu_info[j]->stats2, &state.cpu_info[j]->stats,
		sizeof(struct tile_cpu_stats));
	tile_reset_cpu_stats(&state.cpu_info[j]->stats);
    }

    printk("tilegx stats @%d on:%u\n", (int)jiffies, smp_processor_id());

    // print totals
    for (j = 0; j < NR_CPUS; ++j) {
	if (!state.cpu_info[j]) {
	    continue;
	}
	for (i = 0; i < TILE_DEVS_MAX; ++i) {
	    struct net_device *dev = state.devs[i];
	    if (!dev) {
		continue;
	    }
	    tile_add_stats(&total, &state.cpu_info[j]->dev_info[i].stats2);
	}
	tile_add_cpu_stats(&cpu_total, &state.cpu_info[j]->stats2);
    }

    tile_print_stats_header(&total, &cpu_total);
    tile_print_stats("TOT", &total, &total, &cpu_total, &cpu_total);

    // print device summary
    for (i = 0; i < TILE_DEVS_MAX; ++i) {
	struct net_device *dev = state.devs[i];
	if (!dev) {
	    continue;
	}
	memset(&x, 0, sizeof(x));
	for (j = 0; j < NR_CPUS; ++j) {
	    if (!state.cpu_info[j]) {
		continue;
	    }
	    tile_add_stats(&x, &state.cpu_info[j]->dev_info[i].stats2);
	}
	if (tile_stats_empty(&x, NULL)) {
	    continue;
	}
	tile_print_stats(dev->name, &total, &x, NULL, NULL);
    }

    // print core summary
    for (j = 0; j < NR_CPUS; ++j) {
	char core[10];
	if (!state.cpu_info[j]) {
	    continue;
	}
	memset(&x, 0, sizeof(x));
	for (i = 0; i < TILE_DEVS_MAX; ++i) {
	    tile_add_stats(&x, &state.cpu_info[j]->dev_info[i].stats2);
	}
	struct tile_cpu_stats *cs = &state.cpu_info[j]->stats2;
	if (tile_stats_empty(&x, cs)) {
	    continue;
	}
	sprintf(core, "cpu%u", j);
	tile_print_stats(core, &total, &x, &cpu_total, cs);
    }

/*
    // print individual
    for (i = 0; i < TILE_DEVS_MAX; ++i) {
	struct net_device *dev = state.devs[i];
	if (!dev) {
	    continue;
	}
	for (j = 0; j < NR_CPUS; ++j) {
	    char devcore[20];
	    if (!state.cpu_info[j]) {
		continue;
	    }
	    memset(&x, 0, sizeof(x));
	    tile_add_stats(&x, &state.cpu_info[j]->dev_info[i].stats2);

	    if (!x.stats_rx_buffer_error &&
		    !x.stats_xmit && !x.stats_xmit_fast &&
		    !x.stats_tx_queue_stop && !x.stats_tx_work) {
		continue;
	    }
	    sprintf(devcore, "%s cpu% 2u", dev->name, j);
	    tile_print_stats(devcore, &total, &x, NULL, NULL);
	}
    }
*/
}
#endif



static u64 tile_get_port_config_input_gpios(struct tile_port_config *config) {
    u64 gpios = 0;
    if (IS_NATIVE_GPIO(config->gpio_phy_1g_int)) {
	gpios |= BIT(config->gpio_phy_1g_int);
    }
    if (IS_NATIVE_GPIO(config->gpio_sfp_abs)) {
	gpios |= BIT(config->gpio_sfp_abs);
    }
    if (IS_NATIVE_GPIO(config->gpio_sfp_rx_lose)) {
	gpios |= BIT(config->gpio_sfp_rx_lose);
    }
    if (IS_NATIVE_GPIO(config->gpio_sfp_tx_fault)) {
	gpios |= BIT(config->gpio_sfp_tx_fault);
    }
    return gpios;
}

static u64 tile_get_port_config_output_gpios(struct tile_port_config *config) {
    u64 gpios = 0;
    if (IS_NATIVE_GPIO(config->gpio_sfp_tx_disable)) {
	gpios |= BIT(config->gpio_sfp_tx_disable);
    }
    if (IS_NATIVE_GPIO(config->gpio_sfp_rate_select)) {
	gpios |= BIT(config->gpio_sfp_rate_select);
    }
    if (IS_NATIVE_GPIO(config->gpio_sfp_rate_select2)) {
	gpios |= BIT(config->gpio_sfp_rate_select2);
    }
    if (IS_NATIVE_GPIO(config->gpio_sfp_led)) {
	gpios |= BIT(config->gpio_sfp_led);
    }
    if (IS_NATIVE_GPIO(config->gpio_sfp_led2)) {
	gpios |= BIT(config->gpio_sfp_led2);
    }
    return gpios;
}

static u64 tile_get_port_config_phy_1g_interrupt_gpios(
    struct tile_port_config *config) {
    u64 gpios = 0;
    if (config->phy_1g) {
	if (IS_NATIVE_GPIO(config->gpio_phy_1g_int)) {
	    gpios |= BIT(config->gpio_phy_1g_int);
	}
    }
    return gpios;
}

static void tile_config_i2c_gpio(unsigned gpio, bool input) {
    if (IS_I2C_GPIO(gpio)) {
	i2c_gpio_config(&state.gpio_expander[gpio >> 8], gpio & 0xf, input, false);
    }
}

static void tile_config_i2c_gpios(struct tile_port_config *config) {
    tile_config_i2c_gpio(config->gpio_phy_1g_int, true);
    tile_config_i2c_gpio(config->gpio_sfp_abs, true);
    tile_config_i2c_gpio(config->gpio_sfp_rx_lose, true);
    tile_config_i2c_gpio(config->gpio_sfp_tx_fault, true);

    tile_config_i2c_gpio(config->gpio_sfp_tx_disable, false);
    tile_config_i2c_gpio(config->gpio_sfp_rate_select, false);
    tile_config_i2c_gpio(config->gpio_sfp_rate_select2, false);
    tile_config_i2c_gpio(config->gpio_sfp_led, false);
    tile_config_i2c_gpio(config->gpio_sfp_led2, false);
}


static void tile_gpio_set(u16 gpio, bool value) {
    if (IS_NATIVE_GPIO(gpio)) {
	u64 bm = BIT(gpio);
	gxio_gpio_set(&state.gpio_ctx, value ? bm : 0, bm);
    }
    else if (IS_I2C_GPIO(gpio)) {
	i2c_gpio_set(&state.gpio_expander[gpio >> 8], gpio & 0xf, value);
    }
    else if (IS_LATCH_GPIO(gpio)) {
	access_latch(1, (unsigned long)value << (gpio & 0x7f),
		(unsigned long)1 << (gpio & 0x7f));
    }
}

static bool tile_gpio_get(u16 gpio) {
    if (IS_NATIVE_GPIO(gpio)) {
	return gxio_gpio_get(&state.gpio_ctx) & BIT(gpio);
    }
    if (IS_I2C_GPIO(gpio)) {
	return i2c_gpio_get(&state.gpio_expander[gpio >> 8], gpio & 0xf);
    }
    if (IS_LATCH_GPIO(gpio)) {
	unsigned long x = access_latch(1, 0, 0);
//	printk("latch: %08lx\n", x);
	return x & (1 << (gpio & 0x7f));
    }
    return false;
}

static int tile_gbe_mdio_read(struct net_device *dev, int phy, int reg) {
    struct tile_priv *priv = netdev_priv(dev);
    int ret;
    if (priv->config->mdio < 0) {
	return 0xffff;
    }
    if (priv->sw.ops) {
	phy |= priv->config->mdio;
    }
    spin_lock_bh(&state.mdio_lock);
    ret = gxio_mpipe_link_mdio_rd_aux(&priv->mpipe->ctx, phy, phy, -1, reg);
    spin_unlock_bh(&state.mdio_lock);
//    printk("%s: gbe_mdio_read  %02x,%02x: %04x\n", dev->name, phy, reg, ret);
    return ret;
}

static void tile_gbe_mdio_write(struct net_device *dev, int phy, int reg,
	int data) {
    struct tile_priv *priv = netdev_priv(dev);
    if (priv->config->mdio < 0) {
	return;
    }
    if (priv->sw.ops) {
	phy |= priv->config->mdio;
    }
//    printk("%s: gbe_mdio_write %02x,%02x: %04x\n", dev->name, phy, reg, data);
    spin_lock_bh(&state.mdio_lock);
    gxio_mpipe_link_mdio_wr_aux(&priv->mpipe->ctx, phy, phy, -1, reg, data);
    spin_unlock_bh(&state.mdio_lock);
}

static int tile_xgbe_mdio_read(struct net_device *dev, int phy, int reg) {
    struct tile_priv *priv = netdev_priv(dev);
    int ret;
    if (phy > 0xff) {
	reg |= (phy & 0xff00) << 8;
    }
    spin_lock_bh(&state.mdio_lock);
    ret = gxio_mpipe_link_mdio_rd_aux(
	&priv->mpipe->ctx, phy, phy, reg >> 16, reg & 0xffff);
//    printk("xgbe_mdio_read dev:%x reg:%04x = %04x\n",
//	    reg >> 16, reg & 0xffff, ret);
    spin_unlock_bh(&state.mdio_lock);
    return ret;
}

static void tile_xgbe_mdio_write(struct net_device *dev, int phy, int reg,
	int data) {
    struct tile_priv *priv = netdev_priv(dev);
    unsigned r = reg;
    if (phy > 0xff) {
	r |= (phy & 0xff00) << 8;
	printk("%s %05x: %04x\n",
		dev->name, r, tile_xgbe_mdio_read(dev, phy, reg));
    }
    spin_lock_bh(&state.mdio_lock);
    gxio_mpipe_link_mdio_wr_aux(
	&priv->mpipe->ctx, phy, phy, r >> 16, r & 0xffff, data);
    spin_unlock_bh(&state.mdio_lock);
    if (phy > 0xff) {
	printk("%s %05x: %04x\n",
		dev->name, r, tile_xgbe_mdio_read(dev, phy, reg));
    }
}

static int64_t tile_rd(struct tile_priv *priv, int addr) {
    int64_t ret = 0;
    if (!priv->link.context) {
	printk("%s: mac read unavailable\n", priv->sw.dev->name);
	WARN_ON(1);
	return 0;
    }
    gxio_mpipe_link_mac_rd_aux(
	&priv->mpipe->ctx, (priv->link.mac << 24) | addr, &ret);
    return ret;
}

static void tile_wr(struct tile_priv *priv, int addr, int64_t val) {
    if (!priv->link.context) {
	printk("%s: mac write unavailable\n", priv->sw.dev->name);
	return;
    }
    gxio_mpipe_link_mac_wr_aux(&priv->mpipe->ctx, priv->link.mac, addr, val);
}

static unsigned tile_serdes_read(struct tile_priv *priv,
	unsigned serdes_reg, unsigned lane) {
    MPIPE_XAUI_SERDES_CONFIG_t c = {
	.reg_addr = serdes_reg,
	.lane_sel = BIT(lane),
	.read = 1,
	.send = 1,
    };
    tile_wr(priv, MPIPE_XAUI_SERDES_CONFIG, c.word);
    unsigned i;
    for (i = 0; i < 1000; ++i) {
	c.word = tile_rd(priv, MPIPE_XAUI_SERDES_CONFIG);
	if (!c.send) {
	    break;
	}
    }
    if (i == 1000) {
	printk("tile: serdes read timeout\n");
	return 0;
    }
    return c.reg_data;
}

static unsigned tile_serdes_read_all_lanes(struct tile_priv *priv,
	unsigned serdes_reg) {
    return
	(tile_serdes_read(priv, serdes_reg, 3) << 24) |
	(tile_serdes_read(priv, serdes_reg, 2) << 16) |
	(tile_serdes_read(priv, serdes_reg, 1) << 8) |
	(tile_serdes_read(priv, serdes_reg, 0) << 0);
}

static inline void tile_serdes_write_all_lanes(struct tile_priv *priv,
	unsigned serdes_reg, unsigned val) {
    MPIPE_XAUI_SERDES_CONFIG_t c = {
	.reg_data = val,
	.reg_addr = serdes_reg,
	.lane_sel = 0xf,
	.read = 0,
	.send = 1,
    };
    tile_wr(priv, MPIPE_XAUI_SERDES_CONFIG, c.word);
    unsigned i;
    for (i = 0; i < 1000; ++i) {
	c.word = tile_rd(priv, MPIPE_XAUI_SERDES_CONFIG);
	if (!c.send) {
	    break;
	}
    }
    if (i == 1000) {
	printk("tile: serdes write timeout\n");
    }
}

static inline uint64_t mpipe_rd(struct tile_mpipe *mpipe, int addr) {
    return __gxio_mmio_read(mpipe->ctx.mmio_cfg_base + addr);
}

static inline void mpipe_wr(struct tile_mpipe *mpipe, int addr, uint64_t val) {
    __gxio_mmio_write(mpipe->ctx.mmio_cfg_base + addr, val);
}


static void tile_mpipe_dump(struct tile_mpipe *mpipe) {
    unsigned i;
    printk("MPIPE_DEV_INFO: %llx\n", mpipe_rd(mpipe, MPIPE_DEV_INFO));

    MPIPE_BSM_CTL_t bsm_ctl;
    bsm_ctl.word = mpipe_rd(mpipe, MPIPE_BSM_CTL);
    printk("buffer stack int_lwm:%u max_req:%u lwm:%u\n",
	    bsm_ctl.int_lwm, bsm_ctl.max_req, bsm_ctl.lwm);
    spin_lock_bh(&mpipe->adjust_buffers_lock);
    MPIPE_BSM_INIT_DAT_0_t bsm_dat0;
    MPIPE_BSM_INIT_DAT_1_t bsm_dat1;
    mpipe_wr(mpipe, MPIPE_BSM_INIT_CTL, 0);
    for (i = 0; i < 32; ++i) {
	bsm_dat0.word = mpipe_rd(mpipe, MPIPE_BSM_INIT_DAT);
	bsm_dat1.word = mpipe_rd(mpipe, MPIPE_BSM_INIT_DAT);
	if (!bsm_dat1.enable) {
	    continue;
	}
	printk("buffer stack %u tos_idx:%x lim:%x drain:%u enable:%u size:%u tile_id:%u pin:%u nt_hint:%u hfh:%u base:%x\n",
		i, bsm_dat0.tos_idx, bsm_dat0.lim,
		bsm_dat1.drain,
		bsm_dat1.enable,
		bsm_dat1.size,
		bsm_dat1.tile_id,
		bsm_dat1.pin,
		bsm_dat1.nt_hint,
		bsm_dat1.hfh,
		bsm_dat1.base);
    }
    spin_unlock_bh(&mpipe->adjust_buffers_lock);

    MPIPE_LBL_INFO_t lbl_info;
    lbl_info.word = mpipe_rd(mpipe, MPIPE_LBL_INFO);
    printk("lbl_info direct_switch_support:%u num_buckets:%u num_groups:%u num_nr:%u\n",
	    lbl_info.direct_switch_support,
	    lbl_info.num_buckets, lbl_info.num_groups, lbl_info.num_nr);

    MPIPE_EDMA_CTL_t edma_ctl;
    edma_ctl.word = mpipe_rd(mpipe, MPIPE_EDMA_CTL);
    printk("edma_ctl max_req:%u ud_blocks:%u hunt_cycles:%u desc_read_pace:%u data_read_pace:%u max_dm_req:%u\n",
	    edma_ctl.max_req, edma_ctl.ud_blocks, edma_ctl.hunt_cycles,
	    edma_ctl.desc_read_pace, edma_ctl.data_read_pace,
	    edma_ctl.max_dm_req);

    MPIPE_EDMA_STS_t edma_sts;
    edma_sts.word = mpipe_rd(mpipe, MPIPE_EDMA_STS);
    printk("edma_sts last_desc_disc:%u epkt_blocks(curr:%u min:%u max:%u num:%u) last_invalid_post:%u\n",
	    edma_sts.last_desc_disc,
	    edma_sts.curr_epkt_blocks,
	    edma_sts.min_epkt_blocks,
	    edma_sts.max_epkt_blocks,
	    edma_sts.num_epkt_blocks,
	    edma_sts.last_invalid_post);

    MPIPE_EDMA_INFO_t edma_info;
    edma_info.word = mpipe_rd(mpipe, MPIPE_EDMA_INFO);
    printk("edma_info reorder_support:%u remote_buff_rtn_support:%u epkt_bw_arb_support:%u epkt_burst_support:%u num_rings:%u\n",
	    edma_info.reorder_support,
	    edma_info.remote_buff_rtn_support,
	    edma_info.epkt_bw_arb_support,
	    edma_info.epkt_burst_support,
	    edma_info.num_rings);

    MPIPE_EDMA_DATA_LAT_t data_lat;
    data_lat.word = mpipe_rd(mpipe, MPIPE_EDMA_DATA_LAT);
    printk("edma_data_lat curr:%u max:%u min:%u\n",
	    data_lat.curr_lat, data_lat.max_lat, data_lat.min_lat);
    data_lat.clear = 1;
    mpipe_wr(mpipe, MPIPE_EDMA_DATA_LAT, data_lat.word);

    MPIPE_EDMA_DESC_LAT_t desc_lat;
    desc_lat.word = mpipe_rd(mpipe, MPIPE_EDMA_DESC_LAT);
    printk("edma_desc_lat curr:%u max:%u min:%u\n",
	    desc_lat.curr_lat, desc_lat.max_lat, desc_lat.min_lat);
    desc_lat.clear = 1;
    mpipe_wr(mpipe, MPIPE_EDMA_DESC_LAT, desc_lat.word);

    MPIPE_EDMA_BW_CTL_t edma_bw_ctl;
    edma_bw_ctl.word = mpipe_rd(mpipe, MPIPE_EDMA_BW_CTL);
    printk("edma_bw_ctl line_rate:%u burst_length:%u prior2_rate:%u prior1_rate:%u prior0_rate:%u\n",
	    edma_bw_ctl.line_rate,
	    edma_bw_ctl.burst_length,
	    edma_bw_ctl.prior2_rate,
	    edma_bw_ctl.prior1_rate,
	    edma_bw_ctl.prior0_rate);

    MPIPE_EDMA_DIAG_CTL_t edma_diag_ctl;
    edma_diag_ctl.word = mpipe_rd(mpipe, MPIPE_EDMA_DIAG_CTL);
    printk("edma_diag_ctl disable_final_buf_rtn:%u\n",
	    edma_diag_ctl.disable_final_buf_rtn);

    printk("INT_VEC: %016llx %016llx %016llx %016llx %016llx %016llx %016llx %016llx\n",
	    mpipe_rd(mpipe, MPIPE_INT_VEC0_W1TC),
	    mpipe_rd(mpipe, MPIPE_INT_VEC1_W1TC),
	    mpipe_rd(mpipe, MPIPE_INT_VEC2_W1TC),
	    mpipe_rd(mpipe, MPIPE_INT_VEC3_W1TC),
	    mpipe_rd(mpipe, MPIPE_INT_VEC4_W1TC),
	    mpipe_rd(mpipe, MPIPE_INT_VEC5_W1TC),
	    mpipe_rd(mpipe, MPIPE_INT_VEC6_W1TC),
	    mpipe_rd(mpipe, MPIPE_INT_VEC7_W1TC));
    mpipe_wr(mpipe, MPIPE_INT_VEC0_W1TC, -1ull);
    mpipe_wr(mpipe, MPIPE_INT_VEC1_W1TC, -1ull);
    mpipe_wr(mpipe, MPIPE_INT_VEC2_W1TC, -1ull);
    mpipe_wr(mpipe, MPIPE_INT_VEC3_W1TC, -1ull);
    mpipe_wr(mpipe, MPIPE_INT_VEC4_W1TC, -1ull);
    mpipe_wr(mpipe, MPIPE_INT_VEC5_W1TC, -1ull);
    mpipe_wr(mpipe, MPIPE_INT_VEC6_W1TC, -1ull);
    mpipe_wr(mpipe, MPIPE_INT_VEC7_W1TC, -1ull);

    unsigned cpu;
    for_each_cpu(cpu, &mpipe->rx_cpus_map) {
	struct tile_cpu_info *cpu_info = state.cpu_info[cpu];
//	struct tile_cpu_mpipe_info *mpipe_info =
//	    &cpu_info->mpipe_info[mpipe->instance];
	unsigned i;
	for (i = 0; i < NR_MPIPE_MAX * MPIPE_IQUEUES_PER_CPU_MAX; ++i) {
	    struct tile_rx_queue *rxq = &cpu_info->rxq[i];
	    if (rxq->mpipe != mpipe) {
		continue;
	    }
	    unsigned ring = rxq->iqueue.ring;
	    uint64_t head = rxq->iqueue.head;
	    uint64_t tail = __gxio_mmio_read(rxq->iqueue.idescs);
	    if (head != tail) {
		printk("cpu%u rxq%u ring:%u: head:%llu tail:%llu\n",
			cpu, i, ring, head, tail);
	    }

/*
	    MPIPE_INT_BIND_t data = {{
		    .nw = 1,
		    .bind_sel = ring % 64,
		    .vec_sel = 3 + ring / 64,
		}};
	    mpipe_wr(mpipe, MPIPE_INT_BIND, data.word);
	    data.word = mpipe_rd(mpipe, MPIPE_INT_BIND);
	    printk("cpu%u rxq%u ring:%u:  evt_num:%u int_num:%u tile_id:%u,%u mode:%u enable:%u\n",
		    cpu, i, rxq->iqueue.ring,
		    data.evt_num, data.int_num,
		    data.tileid >> 4, data.tileid & 0xf,
		    data.mode, data.enable);
*/
	}
    }

    printk("ingress_drop: %llu, ingress_pkt:%llu, egress_pkt:%llu, ingress_byte:%llu, egress_byte:%llu\n",
	    mpipe_rd(mpipe, MPIPE_INGRESS_DROP_COUNT_RC),
	    mpipe_rd(mpipe, MPIPE_INGRESS_PKT_COUNT_RC),
	    mpipe_rd(mpipe, MPIPE_EGRESS_PKT_COUNT_RC),
	    mpipe_rd(mpipe, MPIPE_INGRESS_BYTE_COUNT_RC),
	    mpipe_rd(mpipe, MPIPE_EGRESS_BYTE_COUNT_RC));


    printk("NOTIF GROUP TBL:\n");
    MPIPE_LBL_INIT_CTL_t lbl_init_ctl = {};
    lbl_init_ctl.struct_sel = 0;
    lbl_init_ctl.idx = 0;
    mpipe_wr(mpipe, MPIPE_LBL_INIT_CTL, lbl_init_ctl.word);
    for (i = 0; i < lbl_info.num_groups; ++i) {
	printk("group %03u: %016llx %016llx %016llx %016llx\n", i,
		mpipe_rd(mpipe, MPIPE_LBL_INIT_DAT),
		mpipe_rd(mpipe, MPIPE_LBL_INIT_DAT),
		mpipe_rd(mpipe, MPIPE_LBL_INIT_DAT),
		mpipe_rd(mpipe, MPIPE_LBL_INIT_DAT));
    }
    printk(" NR size tileid pin nt_hint hfh base_pa tail count in_flight\n");
    for (i = 0; i < lbl_info.num_nr; ++i) {
	lbl_init_ctl.struct_sel = 2;
	lbl_init_ctl.idx = i * 2;
	mpipe_wr(mpipe, MPIPE_LBL_INIT_CTL, lbl_init_ctl.word);
	MPIPE_LBL_INIT_DAT_NR_TBL_0_t nr_tbl0;
	MPIPE_LBL_INIT_DAT_NR_TBL_1_t nr_tbl1;
	nr_tbl0.word = mpipe_rd(mpipe, MPIPE_LBL_INIT_DAT);
	nr_tbl1.word = mpipe_rd(mpipe, MPIPE_LBL_INIT_DAT);

	lbl_init_ctl.struct_sel = 3;
	lbl_init_ctl.idx = i;
	mpipe_wr(mpipe, MPIPE_LBL_INIT_CTL, lbl_init_ctl.word);
	MPIPE_LBL_INIT_DAT_INFL_CNT_t infl_cnt;
	infl_cnt.word = mpipe_rd(mpipe, MPIPE_LBL_INIT_DAT);

	if (nr_tbl0.base_pa) {
	    printk("% 3u    %u    % 3u   %u       %u   %u %07x % 4u % 5u     % 5u\n", i,
		    nr_tbl0.size, nr_tbl0.tileid, nr_tbl0.pin, nr_tbl0.nt_hint,
		    nr_tbl0.hfh, nr_tbl0.base_pa, nr_tbl0.tail,
		    nr_tbl1.count, infl_cnt.count);
	}
    }

/*
    printk("BUCKETS:\n    ");
    for (i = 0; i < 8; ++i) {
	printk(" m gr count  nr ");
    }
    lbl_init_ctl.struct_sel = 1;
    lbl_init_ctl.idx = 0;
    mpipe_wr(mpipe, MPIPE_LBL_INIT_CTL, lbl_init_ctl.word);
    for (i = 0; i < lbl_info.num_buckets; ++i) {
	if (!(i % 8)) {
	    printk("\n% 4u", i);
	}
	MPIPE_LBL_INIT_DAT_BSTS_TBL_t bsts;
	bsts.word = mpipe_rd(mpipe, MPIPE_LBL_INIT_DAT);
	printk(" %u % 2u % 5u % 3u ", bsts.mode, bsts.group, bsts.count, bsts.notifring);
    }
    printk("\n");
*/

    printk("LBL_NR_STATE:\n");
    MPIPE_LBL_NR_STATE_t lbl_nr_state;
    for (i = 0; i < 26; ++i) {
	printk("%02u0       ", i);
    }
    printk("\n");
    for (i = 0; i < 16; ++i) {
	int j;
	lbl_nr_state.word = mpipe_rd(mpipe, MPIPE_LBL_NR_STATE__FIRST_WORD + i * 8);
	for (j = 0; j < 16; ++j) {
	    printk("%u", (unsigned)lbl_nr_state.word & 0x7);
	    lbl_nr_state.word >>= 4;
	}
    }
    printk("\n");

    static char *lbl_evts[] = { "drop", "drop lbl only", "drop bkt", "drop nr", "sticky pick", "pkts" };
    static char *ipkt_evts[] = { "drop comb", "trunc", "drop", "pkts" };
    static char *idma_evts[] = { "be drops", "bsm stall", "tlb stall", "pkts", "bufs", "retries", "sdn pkts", "sdn stall", "trk stall", "ntf stall", "bsm spill", "bsm fill", "bsm edma" };

    MPIPE_LBL_CTL_t lbl_ctl;
    lbl_ctl.word = mpipe_rd(mpipe, MPIPE_LBL_CTL);
    printk("lbl_ctl srand_thresh:%u freeze:%u\n",
	    lbl_ctl.srand_thresh, lbl_ctl.freeze);
    printk("lbl %s: %llu\n", lbl_evts[lbl_ctl.ctr_sel],
	    mpipe_rd(mpipe, MPIPE_LBL_STAT_CTR));

    MPIPE_IDMA_CTL_t idma_ctl;
    idma_ctl.word = mpipe_rd(mpipe, MPIPE_IDMA_CTL);
    printk("ipkt %s: %llu\n", ipkt_evts[idma_ctl.ipkt_evt_ctr_sel],
	    mpipe_rd(mpipe, MPIPE_IPKT_STAT_CTR));
    printk("idma %s: %llu\n", idma_evts[idma_ctl.idma_evt_ctr_sel], mpipe_rd(mpipe, MPIPE_IDMA_STAT_CTR));

    MPIPE_IDMA_NTF_LAT_t idma_ntf_lat;
    idma_ntf_lat.word = mpipe_rd(mpipe, MPIPE_IDMA_NTF_LAT);
    printk("idma desc latency curr:%u max:%u min:%u\n",
	    idma_ntf_lat.curr_lat,
	    idma_ntf_lat.max_lat,
	    idma_ntf_lat.min_lat);

    MPIPE_IDMA_DAT_LAT_t idma_dat_lat;
    idma_dat_lat.word = mpipe_rd(mpipe, MPIPE_IDMA_DAT_LAT);
    printk("idma data latency curr:%u max:%u min:%u\n",
	    idma_dat_lat.curr_lat,
	    idma_dat_lat.max_lat,
	    idma_dat_lat.min_lat);
}

static void tile_gbe_dump(struct net_device *dev) {
    struct tile_priv *priv = netdev_priv(dev);
    MPIPE_GBE_MAC_INTFC_CTL_t ic = {
	.word = tile_rd(priv, MPIPE_GBE_MAC_INTFC_CTL)
    };
    printk("MAC_INTFC_CTL: %llx tx_prq_ena:%x rx_rst_mode:%d prq_ovd:%d prq_ovd_val:%d pfc_negot:%d pause_mode:%d ins_rx_sts:%d no_tx_crc:%d\n",
	    ic.word,
	    ic.tx_prq_ena, ic.rx_rst_mode, ic.prq_ovd, ic.prq_ovd_val,
	    ic.pfc_negot, ic.pause_mode, ic.ins_rx_sts, ic.no_tx_crc);

    printk("MAC_INTFC_TX_CTL: %llx\n",
	    tile_rd(priv, MPIPE_GBE_MAC_INTFC_TX_CTL));
    printk("MAC_INTFC_FIFO_CTL: %llx\n",
	    tile_rd(priv, MPIPE_GBE_MAC_INTFC_FIFO_CTL));
    MPIPE_GBE_NETWORK_CONTROL_t nt = {
	.word = tile_rd(priv, MPIPE_GBE_NETWORK_CONTROL)
    };
    printk("NETWORK_CONTROL: %llx tx_ena:%d rx_ena:%d loop_local:%d loop:%d\n",
	    nt.word, nt.tx_ena, nt.rx_ena, nt.loop_local, nt.loop);

    MPIPE_GBE_NETWORK_CONFIGURATION_t nc = {
	.word = tile_rd(priv, MPIPE_GBE_NETWORK_CONFIGURATION)
    };
    printk("NETWORK_CONFIGURATION: %llx uni_dir:%d sgmii_mode:%u ignore_fcs:%d ena_half_duplex:%d rcv_csum_ena:%d dis_pause_cpy:%d div:%d fcs_remove:%d len_err_discard:%d rcv_offset:%d rx_pause_ena:%d pcs_sel:%d gige_mode:%d ext_match_ena:%d rcv_1536:%d uni_hash_ena:%d multi_hash_ena:%d no_bcst:%d copy_all:%d jumbo_ena:%d disc_non_vlan:%d full_duplex:%u speed:%u\n",
	    nc.word,
	    nc.uni_dir, nc.sgmii_mode, nc.ignore_fcs, nc.ena_half_duplex,
	    nc.rcv_csum_ena, nc.dis_pause_cpy, nc.div, nc.fcs_remove,
	    nc.len_err_discard, nc.rcv_offset, nc.rx_pause_ena, nc.pcs_sel,
	    nc.gige_mode,
	    nc.ext_match_ena, nc.rcv_1536, nc.uni_hash_ena,
	    nc.multi_hash_ena, nc.no_bcst, nc.copy_all, nc.jumbo_ena,
	    nc.disc_non_vlan, nc.full_duplex, nc.speed);

    MPIPE_GBE_NETWORK_STATUS_t ns = {
	.word = tile_rd(priv, MPIPE_GBE_NETWORK_STATUS)
    };
    printk("NETWORK_STATUS: %llx lpi:%d pfc_negot:%d pause_tx_resol:%d pause_rx_resol:%d full_duplex:%d phy_management_idle:%d mdio_state:%d pcs_state:%d\n",
	    ns.word,
	    ns.lpi, ns.pfc_negot, ns.pause_tx_resol, ns.pause_rx_resol,
	    ns.full_duplex, ns.phy_management_idle, ns.mdio_state,
	    ns.pcs_state);

    printk("TRANSMIT_STATUS: %llx\n", tile_rd(priv, MPIPE_GBE_TRANSMIT_STATUS));
    printk("RECEIVE_STATUS: %llx\n", tile_rd(priv, MPIPE_GBE_RECEIVE_STATUS));
    printk("INTERRUPT_STATUS: %llx\n",
	    tile_rd(priv, MPIPE_GBE_INTERRUPT_STATUS));
    printk("INTERRUPT_MASK: %llx\n", tile_rd(priv, MPIPE_GBE_INTERRUPT_MASK));
    printk("MODULE_ID: %llx\n", tile_rd(priv, MPIPE_GBE_MODULE_ID));


    MPIPE_GBE_PCS_CTL_t pcs_ctl = {
	.word = tile_rd(priv, MPIPE_GBE_PCS_CTL)
    };
    printk("PCS_CTL: %llx speed_sel:%d%d auto_neg:%d duplex_sts:%d\n",
	    pcs_ctl.word, pcs_ctl.speed_sel_0, pcs_ctl.speed_sel_1, pcs_ctl.auto_neg, pcs_ctl.duplex_sts);

    MPIPE_GBE_PCS_STS_t pcs_sts = {
	.word = tile_rd(priv, MPIPE_GBE_PCS_STS)
    };
    printk("PCS_STS: %llx ext_sts:%u auto_neg_comp:%u rem_fault:%u auto_neg_cap:%u link_sts:%u ext_reg_cap:%u\n", pcs_sts.word, pcs_sts.ext_sts, pcs_sts.auto_neg_comp, pcs_sts.rem_fault, pcs_sts.auto_neg_cap, pcs_sts.link_sts, pcs_sts.ext_reg_cap);
//    printk("PCS_PHY_UPPER_ID: %llx\n", tile_rd(priv, MPIPE_GBE_PCS_PHY_UPPER_ID));
//    printk("PCS_PHY_LOWER_ID: %llx\n", tile_rd(priv, MPIPE_GBE_PCS_PHY_LOWER_ID));
    MPIPE_GBE_PCS_AUTO_NEG_t pcs_auto_neg = {
	.word = tile_rd(priv, MPIPE_GBE_PCS_AUTO_NEG)
    };
    printk("PCS_AUTO_NEG: %llx sgmii:%u\n",
	    pcs_auto_neg.word, pcs_auto_neg.sgmii);

    MPIPE_GBE_PCS_AUTO_NEG_PARTNER_t pcs_partner = {
	.word = tile_rd(priv, MPIPE_GBE_PCS_AUTO_NEG_PARTNER)
    };
    printk("PCS_AUTO_NEG_PARTNER: %llx link_sts:%d link_partner_ack:%u duplex:%d speed:%d\n",
	    pcs_partner.word, pcs_partner.link_sts,
	    pcs_partner.link_partner_ack, pcs_partner.duplex,
	    pcs_partner.speed);
    printk("PCS_AUTO_NEG_EXP: %llx\n",
	    tile_rd(priv, MPIPE_GBE_PCS_AUTO_NEG_EXP));
    printk("PCS_AUTO_NEG_NXT_PG: %llx\n",
	    tile_rd(priv, MPIPE_GBE_PCS_AUTO_NEG_NXT_PG));
    printk("PCS_AUTO_NEG_LP_NP: %llx\n",
	    tile_rd(priv, MPIPE_GBE_PCS_AUTO_NEG_LP_NP));
    printk("PCS_AUTO_NEX_EXT_STS: %llx\n",
	    tile_rd(priv, MPIPE_GBE_PCS_AUTO_NEX_EXT_STS));
}

static void tile_xgbe_dump(struct net_device *dev) {
    struct tile_priv *priv = netdev_priv(dev);
    MPIPE_XAUI_MAC_INTFC_CTL_t intfc_ctl = {
	.word = tile_rd(priv, MPIPE_XAUI_MAC_INTFC_CTL)
    };
    printk("XAUI_MAC_INTFC_CTL: %llx tx_prq_ena:%04x prq_ovd:%u prq_ovd_val:%u pfc_negot:%u pause_mode:%u ins_rx_sts:%u no_tx_pre:%u no_tx_crc:%u\n",
	    intfc_ctl.word,
	    intfc_ctl.tx_prq_ena, intfc_ctl.prq_ovd, intfc_ctl.prq_ovd_val,
	    intfc_ctl.pfc_negot, intfc_ctl.pause_mode, intfc_ctl.ins_rx_sts,
	    intfc_ctl.no_tx_pre, intfc_ctl.no_tx_crc);

    MPIPE_XAUI_MAC_INTFC_TX_CTL_t tx_ctl = {
	.word = tile_rd(priv, MPIPE_XAUI_MAC_INTFC_TX_CTL)
    };
    printk("XAUI_MAC_INTFC_TX_CTL: %llx tx_pfc_pause_val:%04x tx_pause_ovd:%u tx_drop:%u tx_pause_hyst:%u\n",
	    tx_ctl.word, tx_ctl.tx_pfc_pause_val, tx_ctl.tx_pause_ovd,
	    tx_ctl.tx_drop, tx_ctl.tx_pause_hyst);

    MPIPE_XAUI_PCS_CTL_t pcs_ctl = {
	.word = tile_rd(priv, MPIPE_XAUI_PCS_CTL)
    };
    printk("XAUI_PCS_CTL: %llx cdr_mode:%u led_ovd_val:%u led_mode:%u test_ena:%u test_sel:%u inv_tx_pol:%x inv_rx_pol:%x reset_mode:%u double_rate:%u\n",
	    pcs_ctl.word, pcs_ctl.cdr_mode, pcs_ctl.led_ovd_val,
	    pcs_ctl.led_mode, pcs_ctl.test_ena, pcs_ctl.test_sel,
	    pcs_ctl.inv_tx_pol, pcs_ctl.inv_rx_pol,
	    pcs_ctl.reset_mode, pcs_ctl.double_rate);

    MPIPE_XAUI_PCS_CTL1_t pcs_ctl1 = {
	.word = tile_rd(priv, MPIPE_XAUI_PCS_CTL1)
    };
    printk("XAUI_PCS_CTL1: %llx pma_ena_mode:%u txvld_val:%u txvld_ovd:%u xaui_rx_rst_mode:%u pwrdn_ovd:%u pwrdn_val:%u\n",
	    pcs_ctl1.word, pcs_ctl1.pma_ena_mode,
	    pcs_ctl1.txvld_val, pcs_ctl1.txvld_ovd, pcs_ctl1.xaui_rx_rst_mode,
	    pcs_ctl1.pwrdn_ovd, pcs_ctl1.pwrdn_val);

    MPIPE_XAUI_PCS_STS_t pcs_sts = {
	.word = tile_rd(priv, MPIPE_XAUI_PCS_STS)
    };
    printk("XAUI_PCS_STS: %llx cdr_locked:%x pcs_ready:%x pcs_dec_err:%x pcs_unaligned:%u pcs_fifo_err:%x pcs_ctc_err:%x pcs_sync_sts:%x serdes_sig_det:%x clock_ready:%u\n",
	    pcs_sts.word, pcs_sts.cdr_locked, pcs_sts.pcs_ready,
	    pcs_sts.pcs_dec_err, pcs_sts.pcs_unaligned,
	    pcs_sts.pcs_fifo_err, pcs_sts.pcs_ctc_err,
	    pcs_sts.pcs_sync_sts, pcs_sts.serdes_sig_det, pcs_sts.clock_ready);
    // reset write-1-to-clear bits
    tile_wr(priv, MPIPE_XAUI_PCS_STS, pcs_sts.word);

    MPIPE_XAUI_INTERRUPT_MASK_t int_mask = {
	.word = tile_rd(priv, MPIPE_XAUI_INTERRUPT_MASK)
    };
    printk("XAUI_INTERRUPT_MASK: %llx\n", int_mask.word);

    MPIPE_XAUI_INTERRUPT_STATUS_t int_status = {
	.word = tile_rd(priv, MPIPE_XAUI_INTERRUPT_STATUS)
    };
    printk("XAUI_INTERRUPT_STATUS: %llx pcs_alignment_changed:%u phy_int:%u tx_ptp_sync:%u tx_ptp_delay:%u tx_ptp_pdelay_req:%u tx_ptp_pdelay_resp:%u rx_ptp_sync:%u rx_ptp_delay:%u rx_ptp_pdelay_req:%u rx_ptp_pdelay_resp:%u lpi:%u link_sts_change:%u mdio_complete:%u pause_tx:%u pause_to:%u pause_rx:%u stat_ovfl:%u tx_err:%u tx_underflow:%u frame_tx:%u frame_rx:%u\n",
	    int_status.word,
	    int_status.pcs_alignment_changed,
	    int_status.phy_int,
	    int_status.tx_ptp_sync, int_status.tx_ptp_delay,
	    int_status.tx_ptp_pdelay_req, int_status.tx_ptp_pdelay_resp,
	    int_status.rx_ptp_sync, int_status.rx_ptp_delay,
	    int_status.rx_ptp_pdelay_req, int_status.rx_ptp_pdelay_resp,
	    int_status.lpi,
	    int_status.link_sts_change,
	    int_status.mdio_complete,
	    int_status.pause_tx, int_status.pause_to, int_status.pause_rx,
	    int_status.stat_ovfl, int_status.tx_err, int_status.tx_underflow,
	    int_status.frame_tx, int_status.frame_rx
	);
    tile_wr(priv, MPIPE_XAUI_INTERRUPT_STATUS, int_status.word);

    MPIPE_XAUI_TRANSMIT_CONTROL_t tx_control = {
	.word = tile_rd(priv, MPIPE_XAUI_TRANSMIT_CONTROL)
    };
    printk("XAUI_TRANSMIT_CONTROL: %llx ena_lpi:%u pfc_zero:%u pfc_pev:%u enable_tx:%u\n",
	    tx_control.word, tx_control.ena_lpi,
	    tx_control.pfc_zero, tx_control.pfc_pev, tx_control.enable_tx);

    MPIPE_XAUI_TRANSMIT_CONFIGURATION_t tx_config = {
	.word = tile_rd(priv, MPIPE_XAUI_TRANSMIT_CONFIGURATION)
    };
    printk("XAUI_TRANSMIT_CONFIGURATION: %llx preamble_crc:%u disable_dic:%u decrease_ipg:%u uni_mode:%u cfg_speed:%u stretch:%u pause_det:%u\n",
	    tx_config.word, tx_config.preamble_crc,
	    tx_config.disable_dic, tx_config.decrease_ipg,
	    tx_config.uni_mode, tx_config.cfg_speed,
	    tx_config.stretch, tx_config.pause_det);

    MPIPE_XAUI_RECEIVE_CONTROL_t rx_control = {
	.word = tile_rd(priv, MPIPE_XAUI_RECEIVE_CONTROL)
    };
    printk("XAUI_RECEIVE_CONTROL: %llx enable_rx:%u\n",
	    tx_control.word, rx_control.enable_rx);

    MPIPE_XAUI_RECEIVE_CONFIGURATION_t rx_config = {
	.word = tile_rd(priv, MPIPE_XAUI_RECEIVE_CONFIGURATION)
    };
    printk("XAUI_RECEIVE_CONFIGURATION: %llx lpi:%u pfc_ena:%u preamble_crc:%u loc_fault:%u rem_fault:%u len_disc:%u preamble_convert:%u accept_nsp:%u pass_preamble:%u discard_pause:%u accept_1536:%u accept_jumbo:%u fcs_remove:%u discard_non_vlan:%u ena_match:%u ena_hash_uni:%u ena_hash_multi:%u dis_bcst:%u copy_all:%u\n",
	    rx_config.word, rx_config.lpi,
	    rx_config.pfc_ena, rx_config.preamble_crc,
	    rx_config.loc_fault, rx_config.rem_fault,
	    rx_config.len_disc, rx_config.preamble_convert,
	    rx_config.accept_nsp, rx_config.pass_preamble,
	    rx_config.discard_pause, rx_config.accept_1536,
	    rx_config.accept_jumbo, rx_config.fcs_remove,
	    rx_config.discard_non_vlan, rx_config.ena_match,
	    rx_config.ena_hash_uni, rx_config.ena_hash_multi,
	    rx_config.dis_bcst, rx_config.copy_all
	);
    rx_config.loc_fault = 0;
    rx_config.rem_fault = 0;
    tile_wr(priv, MPIPE_XAUI_RECEIVE_CONFIGURATION, rx_config.word);

    MPIPE_XAUI_STATISTICS_CONTROL_t stat_ctrl = {
	.word = tile_rd(priv, MPIPE_XAUI_STATISTICS_CONTROL)
    };
    printk("XAUI_STATISTICS_CONTROL: %llx\n", stat_ctrl.word);

    unsigned x = tile_serdes_read_all_lanes(priv, MPIPE_SERDES_PCS_LPBK_CTRL);
    printk("SERDES_PCS_LPBK_CTRL: %08x\n", x);

    x = tile_serdes_read_all_lanes(priv, MPIPE_SERDES_PRBS_CTRL);
    printk("SERDES_PRBS_CTRL: %08x\n", x);
}

static u64 tile_update_tx_completed(struct tile_tx_queue *tx_queue) {
    MPIPE_EDMA_POST_REGION_VAL_t val;
    u64 tail;
    u64 slot = atomic64_read(&tx_queue->slot);

    val.word = __gxio_mmio_read(tx_queue->equeue.dma_queue.post_region_addr);
    tail = val.count;

    u64 gen = slot & ~0xffffllu;
    if (tail > (slot & 0xffff)) {
	gen -= 0x10000;
    }
//    printk("tx_completed: %lu, slot:%llu tail:%u head:%u\n",
//	    gen | tail, slot, val.count, val.ring_idx);
    return gen | tail;
}


static void tile_tx_descs_dump(struct tile_tx_queue *tx_queue, unsigned around) {
    unsigned i;

    printk("hw descs %lu-%lu\n",
	    (around - 10) & tx_queue->equeue.mask_num_entries,
	    (around + 9) & tx_queue->equeue.mask_num_entries);
    for (i = 0; i < 20; ++i) {
	unsigned d = (around - 10 + i) & tx_queue->equeue.mask_num_entries;
	gxio_mpipe_edesc_t edesc;
	gxio_mpipe_edesc_t* edesc_p =
	    &tx_queue->equeue.edescs[d];
	edesc.words[1] = __gxio_mmio_read(&edesc_p->words[1]);
	edesc.words[0] = __gxio_mmio_read(&edesc_p->words[0]);
	printk("%u: gen:%u r0:%u ns:%u notif:%u bound:%u r1:%u size:%u r2:%u csum:%u %u %u %x va:%010llx stack:%u inst:%u hwb:%u s:%u c:%u\n",
		d, edesc.gen, edesc.r0, edesc.ns, edesc.notif, edesc.bound,
		edesc.r1, edesc.xfer_size, edesc.r2,
		edesc.csum, edesc.csum_dest, edesc.csum_start, edesc.csum_seed,
		(unsigned long long)edesc.va, edesc.stack_idx, edesc.inst,
		edesc.hwb, edesc.size, edesc.c);
    }
}

static void tile_tx_queue_dump(struct net_device *dev,
	struct tile_tx_queue *tx_queue, int descs_for_cpu, bool dump_descs) {
    struct tile_priv *priv = netdev_priv(dev);
    unsigned i;

    u64 slot = atomic64_read(&tx_queue->slot);
    u64 completed = tile_update_tx_completed(tx_queue);
    printk("tx_queue:%u slot: %llu(%llu) completed: %llu(%llu)\n",
	    tx_queue->edma,
	    slot, slot & tx_queue->equeue.mask_num_entries,
	    completed, completed & tx_queue->equeue.mask_num_entries);

    MPIPE_EDMA_POST_REGION_VAL_t val;
    val.word = __gxio_mmio_read(tx_queue->equeue.dma_queue.post_region_addr);
    printk("EDMA_POST_REGION_VAL gen: %u, count:%u, ring_idx:%u\n",
	    val.gen, val.count, val.ring_idx);

    MPIPE_EDMA_DIAG_CTL_t ctl = {{ 0 }};
    ctl.word = mpipe_rd(priv->mpipe, MPIPE_EDMA_DIAG_CTL);
    unsigned stat_ctr[5];
    MPIPE_EDMA_DIAG_STS_t diag_sts[5];
    for (i = 0; i < 5; ++i) {
	ctl.diag_ctr_idx = tx_queue->edma;
	ctl.evt_ctr_sel = i;
	ctl.diag_ctr_sel = 0;
	mpipe_wr(priv->mpipe, MPIPE_EDMA_DIAG_CTL, ctl.word);
	stat_ctr[i] = mpipe_rd(priv->mpipe, MPIPE_EDMA_STAT_CTR);
    }
    for (i = 0; i < 5; ++i) {
	ctl.diag_ctr_idx = tx_queue->edma;
	ctl.evt_ctr_sel = 0;
	ctl.diag_ctr_sel = i + 1;
	mpipe_wr(priv->mpipe, MPIPE_EDMA_DIAG_CTL, ctl.word);
	diag_sts[i].word = mpipe_rd(priv->mpipe, MPIPE_EDMA_DIAG_STS);
    }
    printk("diag_ctr pkt:%u rsvd:%u blk:%u fl:%u desc:%u  (mgr:%u rsp:%u post:%u fetch:%u rg:%u)\n",
	    diag_sts[0].diag_ctr_val,
	    diag_sts[1].diag_ctr_val,
	    diag_sts[2].diag_ctr_val,
	    diag_sts[3].diag_ctr_val,
	    diag_sts[4].diag_ctr_val,
	    diag_sts[0].desc_mgr,
	    diag_sts[0].desc_rsp,
	    diag_sts[0].desc_post,
	    diag_sts[0].desc_fetch,
	    diag_sts[0].rg);
    printk("stat_ctr pkts:%u desc:%u req:%u rt_af:%u adn_af:%u\n",
	    stat_ctr[0],
	    stat_ctr[1],
	    stat_ctr[2],
	    stat_ctr[3],
	    stat_ctr[4]);

    slot &= tx_queue->equeue.mask_num_entries;
    completed &= tx_queue->equeue.mask_num_entries;

    if (!dump_descs && descs_for_cpu < 0) {
	return;
    }

    tile_tx_descs_dump(tx_queue, slot);
    if (slot != completed) {
	tile_tx_descs_dump(tx_queue, completed);
    }
    if (slot != val.ring_idx) {
	tile_tx_descs_dump(tx_queue, val.ring_idx);
    }

    for_each_online_cpu(i) {
	unsigned k;
	struct tile_cpu_info *cpu_info = state.cpu_info[i];
	struct tile_cpu_dev_info *cpu_dev_info;
	if (!cpu_info) {
	    continue;
	}
	cpu_dev_info = &cpu_info->dev_info[priv->devno];
	printk("cpu%u head:%u tail:%u stopped:%u trans_start:%d work_busy:%u\n", i,
		cpu_dev_info->tx_head, cpu_dev_info->tx_tail,
		__netif_subqueue_stopped(dev, i),
		(int)netdev_get_tx_queue(dev, i)->trans_start,
		work_busy(&cpu_dev_info->tx_work));
	if (descs_for_cpu >= 0 && descs_for_cpu != i) {
	    continue;
	}
	for (k = 0; k < priv->per_cpu_tx_queue_size; ++k) {
	    struct tile_tx_entry *entry = &cpu_dev_info->tx_entries[k];
	    if (entry->slot || entry->buf) {
		printk("%u: #%lu(%lu) %p\n", k,
			(unsigned long)entry->slot,
			(unsigned long)entry->slot & tx_queue->equeue.mask_num_entries,
			per_cpu_ptr(priv->tx_entries, i)[k].buf);
	    }
	}
    }
}

static void tile_mac_dump(struct net_device *dev) {
    struct tile_priv *priv = netdev_priv(dev);
    MPIPE_GBE_MAC_INFO_t mac_info = {
	.word = tile_rd(priv, MPIPE_GBE_MAC_INFO)
    };
    printk("MAC_INFO: %llx\n", mac_info.word);
    if (priv->xaui) {
	tile_xgbe_dump(dev);
    }
    else {
	tile_gbe_dump(dev);
    }
}

static void tile_dump(struct net_device *dev, unsigned flags) {
    struct tile_priv *priv = netdev_priv(dev);
    printk("%s:\n", dev->name);

    tile_mac_dump(dev);
    phy_dump(&priv->sw.phy);

    if (state.gpio_ctx.mmio_base) {
	printk("GPIO: %016llx\n", gxio_gpio_get(&state.gpio_ctx));
    }
    printk("%s: carrier:%u\n", dev->name, netif_carrier_ok(dev));
    if (!priv->mpipe->ctx.mmio_cfg_base) {
	return;
    }

    tile_mpipe_dump(priv->mpipe);
    tile_tx_queue_dump(dev, &priv->tx_queue, -1, flags & ETHTOOL_FLAG_MAC_DUMP_DESC);
    if (flags & ETHTOOL_FLAG_TX_TIMEOUT) {
	switch_tx_timeout(dev, 0);
    }
}


static struct net_device_ops tile_ops;
static struct net_device_ops tile_ops_fast;
static void tile_set_tx_props(struct tile_priv *priv, unsigned speed) {
    struct tile_tx_queue *q = &priv->tx_queue;
    if (!speed) {
	return;
    }
    q->max_queued_bits = speed * 1000 * MAX_TX_QUEUE_SIZE_IN_MSEC;
    if (q->max_queued_bits < MIN_TX_QUEUE_SIZE_IN_BITS) {
	q->max_queued_bits = MIN_TX_QUEUE_SIZE_IN_BITS;
    }

    // this amounts 100Kbits worth of cycles - do tile_tx() at least every
    // 150th 64-byte packet, or every 8th 1514-byte packet -
    // this is needed to avoid tx queue stops as much as possible
    q->max_cycles_between_tx = CYCLES_PER_SEC / 10 / speed;
//	printk("%s queue%u max_queued_bits:%u max_cycles_between_tx:%u\n",
//		priv->sw.dev->name, i,
//		q->max_queued_bits, q->max_cycles_between_tx);

    if (priv->tx_queue_fast.edma >= 0) {
	switch (speed) {
	case SPEED_10000:
	    priv->sw.dev->netdev_ops = &tile_ops_fast;
	    priv->max_fast_queue_fill = MAX_FAST_PATH_TX_QUEUE_FILL_10G;
	    break;
	case SPEED_1000:
	    priv->sw.dev->netdev_ops = &tile_ops_fast;
	    priv->max_fast_queue_fill = MAX_FAST_PATH_TX_QUEUE_FILL_1G;
	    break;
	default:
	    priv->sw.dev->netdev_ops = &tile_ops;
	    break;
	}
    }
}

static void tile_set_network_configuration(struct tile_priv *priv,
	unsigned speed, unsigned duplex, bool rx_pause, bool tx_pause) {
//    printk("%s(%s): set network configuration %p speed:%d duplex:%d pause:%u/%u\n",
//	    priv->sw.dev->name, priv->config->hv_name, priv->link.context,
//	    speed, duplex, rx_pause, tx_pause);
    tile_set_tx_props(priv, speed);

    if (priv->xaui) {
	MPIPE_XAUI_TRANSMIT_CONFIGURATION_t txconf = {
	    .word = tile_rd(priv, MPIPE_XAUI_TRANSMIT_CONFIGURATION)
	};
	unsigned pause_det = rx_pause;
	if (pause_det != txconf.pause_det) {
	    txconf.pause_det = pause_det;
	    tile_wr(priv, MPIPE_XAUI_TRANSMIT_CONFIGURATION, txconf.word);
	}

	unsigned pause_mode = tx_pause ? 2 : 0;
	MPIPE_XAUI_MAC_INTFC_CTL_t intfcctl = {
	    .word = tile_rd(priv, MPIPE_XAUI_MAC_INTFC_CTL)
	};
	if (pause_mode != intfcctl.pause_mode) {
	    intfcctl.pause_mode = pause_mode;
	    tile_wr(priv, MPIPE_XAUI_MAC_INTFC_CTL, intfcctl.word);
	}
	return;
    }

    MPIPE_GBE_NETWORK_CONFIGURATION_t config = {
	.word = tile_rd(priv, MPIPE_GBE_NETWORK_CONFIGURATION)
    };
    unsigned old_config = config.word;

    switch (speed) {
    case SPEED_10:
	config.gige_mode = 0;
	config.speed = 0;
	break;
    case SPEED_100:
	config.gige_mode = 0;
	config.speed = 1;
	break;
    case SPEED_1000:
	config.gige_mode = 1;
	config.speed = 0;
	break;
    }

    if (duplex == DUPLEX_FULL) {
	config.full_duplex = duplex;
    }
    else {
	config.full_duplex = 0;
    }

    config.rx_pause_ena = rx_pause;

    if (old_config != config.word) {
//	printk("%s: set network configuration speed:%d duplex:%d\n",
//		priv->sw.dev->name, speed, duplex);
	tile_wr(priv, MPIPE_GBE_NETWORK_CONFIGURATION, config.word);
    }

    unsigned pause_mode = tx_pause ? 2 : 0;
    MPIPE_GBE_MAC_INTFC_CTL_t intfcctl = {
	.word = tile_rd(priv, MPIPE_GBE_MAC_INTFC_CTL)
    };
    if (pause_mode != intfcctl.pause_mode) {
	intfcctl.pause_mode = pause_mode;
	tile_wr(priv, MPIPE_GBE_MAC_INTFC_CTL, intfcctl.word);
    }
}

static void tile_sfp_init(struct phy *p) {
    struct ethtool_link_ksettings *cmd = &p->curr.cmd;
    ethtool_link_ksettings_add_link_mode(cmd, supported, Autoneg);
    cmd->base.port = PORT_FIBRE;
    cmd->base.mdio_support = MDIO_SUPPORTS_C45 | MDIO_SUPPORTS_C22;
}

static void tile_sfp_set_settings(struct phy *p, struct phy_settings *ps) {
    struct tile_priv *priv = netdev_priv(p->mii_info.dev);
    struct ethtool_link_ksettings *cmd = &ps->cmd;
//    printk("%s: sfp set settings xaui:%u autoneg:%u speed:%u duplex:%u sfp_rate_sel:%u\n",
//	    priv->sw.dev->name, priv->xaui,
//	    cmd->autoneg, cmd->speed, cmd->duplex, phy_get_sfp_rate_sel(ps));
    priv->rx_lose_down_jiffies = 0;
    priv->autoneg_start_jiffies = 0;

    if (!priv->xaui) {
	MPIPE_GBE_PCS_CTL_t pcs_ctl = {
	    .word = tile_rd(priv, MPIPE_GBE_PCS_CTL)
	};

	pcs_ctl.pcs_soft_reset = 0;
	if (cmd->base.autoneg == AUTONEG_ENABLE) {
	    pcs_ctl.auto_neg = 1;
	    pcs_ctl.restart_neg = 1;
	    tile_wr(priv, MPIPE_GBE_PCS_CTL, pcs_ctl.word);
	}
	else {
	    pcs_ctl.auto_neg = 0;
	    pcs_ctl.restart_neg = 0;

	    MPIPE_GBE_PCS_AUTO_NEG_PARTNER_t pcs_auto_neg_partner = {
		.word = tile_rd(priv, MPIPE_GBE_PCS_AUTO_NEG_PARTNER)
	    };

	    switch(cmd->base.speed) {
	    case SPEED_10:
		pcs_ctl.speed_sel_1 = 0;
		pcs_ctl.speed_sel_0 = 0;
		pcs_auto_neg_partner.speed = 0;
		break;
	    case SPEED_100:
		pcs_ctl.speed_sel_1 = 0;
		pcs_ctl.speed_sel_0 = 1;
		pcs_auto_neg_partner.speed = 1;
		break;
	    case SPEED_1000:
		pcs_ctl.speed_sel_1 = 1;
		pcs_ctl.speed_sel_0 = 0;
		pcs_auto_neg_partner.speed = 2;
		break;
	    }
	    if (cmd->base.duplex == DUPLEX_FULL) {
		pcs_ctl.duplex_sts = 1;
		pcs_auto_neg_partner.duplex = 1;
	    }
	    else {
		pcs_ctl.duplex_sts = 0;
		pcs_auto_neg_partner.duplex = 0;
	    }
	    tile_wr(priv, MPIPE_GBE_PCS_CTL, pcs_ctl.word);

	    tile_wr(priv, MPIPE_GBE_PCS_AUTO_NEG_PARTNER,
		    pcs_auto_neg_partner.word);
	    tile_set_network_configuration(priv, cmd->base.speed, cmd->base.duplex,
		    ps->pause.rx_pause, ps->pause.tx_pause);
	}
    }

    tile_gpio_set(priv->config->gpio_sfp_rate_select, phy_get_sfp_rate_sel(ps));
    tile_gpio_set(priv->config->gpio_sfp_rate_select2, phy_get_sfp_rate_sel(ps));
}

static void tile_set_sgmii(struct tile_priv *priv) {
    MPIPE_GBE_NETWORK_CONFIGURATION_t config, config_new;
    if (!phy_sfp_has_module(&priv->sw.phy.curr)) {
	return;
    }
    if (!priv->sw.phy.eeprom_valid) {
	phy_read_eeprom(&priv->sw.phy, NULL, NULL);
    }
    if (!priv->sw.phy.eeprom_valid) {
	printk("%s: set sgmii bad eeprom %02x\n",
		priv->sw.dev->name, priv->sw.phy.eeprom[0]);
	return;
    }

    priv->sgmii = i2c_eeprom_need_sgmii(priv->sw.phy.eeprom) && priv->sw.phy.req.cmd.base.autoneg == AUTONEG_ENABLE;
    if (!priv->config->combo) {
	config.word = tile_rd(priv, MPIPE_GBE_NETWORK_CONFIGURATION);
	config_new.word = config.word;
	config_new.sgmii_mode = priv->sgmii;
	if (config.word != config_new.word) {
	    printk("%s: sgmii mode:%u\n", priv->sw.dev->name, priv->sgmii);
	    tile_wr(priv, MPIPE_GBE_NETWORK_CONFIGURATION, config_new.word);
	}
    }
}

static bool tile_sfp_allow_autodisable_aneg(struct tile_priv *priv) {
    unsigned connector_type;
    if (priv->config->combo) {
	return false;
    }
    if (!phy_sfp_has_module(&priv->sw.phy.curr)) {
	return false;
    }
    if (!priv->sw.phy.eeprom_valid) {
	phy_read_eeprom(&priv->sw.phy, NULL, NULL);
    }
    if (!priv->sw.phy.eeprom_valid) {
	return false;
    }

    if (priv->sgmii) {
	return false;
    }
    connector_type = i2c_eeprom_connector_type(priv->sw.phy.eeprom, false);
//    printk("%s: connector type %u\n", priv->sw.dev->name, connector_type);
    if (connector_type == I2C_SFP_CONNECTOR_SC ||
	    connector_type == I2C_SFP_CONNECTOR_LC) {
	return true;
    }
    return false;
}

static void tile_sfp_get_settings(struct phy *p, struct phy_settings *ps) {
    struct tile_priv *priv = netdev_priv(p->mii_info.dev);
    struct ethtool_link_ksettings *cmd = &ps->cmd;
    bool rx_lose = false;

    ps->link = 0;
    if (priv->config->gpio_sfp_rx_lose) {
	rx_lose = tile_gpio_get(priv->config->gpio_sfp_rx_lose);
	if (rx_lose) {
	    priv->rx_lose_down_jiffies = 0;
	    priv->autoneg_start_jiffies = 0;
	}
	else {
	    if (!priv->rx_lose_down_jiffies) {
		priv->rx_lose_down_jiffies = jiffies;
		priv->autoneg_start_jiffies = jiffies;
	    }
	}
	ps->link = !rx_lose;
    }

    if (!priv->xaui) {
	MPIPE_GBE_PCS_CTL_t pcs_ctl = {
	    .word = tile_rd(priv, MPIPE_GBE_PCS_CTL)
	};
	MPIPE_GBE_PCS_STS_t pcs_sts = {
	    .word = tile_rd(priv, MPIPE_GBE_PCS_STS)
	};
	MPIPE_GBE_PCS_AUTO_NEG_PARTNER_t pcs_auto_neg_partner = {
	    .word = tile_rd(priv, MPIPE_GBE_PCS_AUTO_NEG_PARTNER)
	};

	if (!priv->config->gpio_sfp_rx_lose) {
	    ps->link = pcs_sts.link_sts;
	}

	if (pcs_ctl.auto_neg) {
	    if (priv->sgmii) {
		if (ps->link) {
		    ps->link = pcs_auto_neg_partner.link_sts;
		}
	    }
	    else {
		if (ps->link) {
		    ps->link = pcs_sts.link_sts;
		}
	    }
	}
	else {
	    priv->autoneg_start_jiffies = 0;
	}
	if (pcs_sts.link_sts) {
	    priv->autoneg_start_jiffies = 0;
	}

	if (tile_sfp_allow_autodisable_aneg(priv)) {
	    if (pcs_ctl.auto_neg && !pcs_sts.link_sts &&
		    priv->autoneg_start_jiffies &&
		    time_after(jiffies, priv->autoneg_start_jiffies + 3 * HZ)) {
		printk("%s: 1000base-x auto neg not complete after 3s rx_lose went down - disabling autoneg\n",
			priv->sw.dev->name);
		pcs_ctl.auto_neg = 0;
		tile_wr(priv, MPIPE_GBE_PCS_CTL, pcs_ctl.word);
	    }
	    if (!pcs_ctl.auto_neg && rx_lose &&
		    priv->sw.phy.req.cmd.base.autoneg == AUTONEG_ENABLE) {
		pcs_ctl.auto_neg = 1;
		tile_wr(priv, MPIPE_GBE_PCS_CTL, pcs_ctl.word);
	    }
	}
    }
//    printk("%s: sfp get settings link:%u\n", priv->sw.dev->name, ps->link);

    cmd->base.reserved[0] = 0;
    if (priv->config->gpio_sfp_abs) {
	if (!tile_gpio_get(priv->config->gpio_sfp_abs)) {
	    cmd->base.reserved[0] |= SFP_MODULE_PRESENT_FLAG;
	    if (tile_gpio_get(priv->config->gpio_sfp_rx_lose)) {
		cmd->base.reserved[0] |= SFP_RX_LOSE_FLAG;
	    }
	    if (tile_gpio_get(priv->config->gpio_sfp_tx_fault)) {
		cmd->base.reserved[0] |= SFP_TX_FAULT_FLAG;
	    }
	}
	else {
	    i2c_sfp_reset(&priv->i2c_sfp);
	}
    }

    if (!netif_running(p->mii_info.dev)) {
	return;
    }

    if (!priv->xaui) {
	MPIPE_GBE_PCS_CTL_t pcs_ctl = {
	    .word = tile_rd(priv, MPIPE_GBE_PCS_CTL)
	};
	MPIPE_GBE_PCS_AUTO_NEG_PARTNER_t pcs_auto_neg_partner = {
	    .word = tile_rd(priv, MPIPE_GBE_PCS_AUTO_NEG_PARTNER)
	};
	MPIPE_GBE_NETWORK_CONFIGURATION_t config = {
	    .word = tile_rd(priv, MPIPE_GBE_NETWORK_CONFIGURATION)
	};

//	printk("sfp get settings %s pcs_ctl:%llx pcs_auto_neg_partner:%llx\n",
//		priv->sw.dev->name, pcs_ctl.word, pcs_auto_neg_partner.word);
	unsigned speed_sel;

	cmd->base.autoneg = pcs_ctl.auto_neg ? AUTONEG_ENABLE : AUTONEG_DISABLE;
	if (!pcs_ctl.auto_neg) {
	    speed_sel = ((unsigned)config.gige_mode << 1) | config.speed;
	    cmd->base.duplex = config.full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
	}
	else {
	    if (pcs_auto_neg_partner.link_sts) {
		speed_sel = pcs_auto_neg_partner.speed;
		cmd->base.duplex = pcs_auto_neg_partner.duplex ? DUPLEX_FULL : DUPLEX_HALF;
	    }
	    else {
		speed_sel = ((unsigned)pcs_ctl.speed_sel_0 << 1) | pcs_ctl.speed_sel_1;
		cmd->base.duplex = pcs_ctl.duplex_sts ? DUPLEX_FULL : DUPLEX_HALF;
	    }
	    if (!config.sgmii_mode) {
		speed_sel = 2;
		cmd->base.duplex = DUPLEX_FULL;
	    }
	}

	switch (speed_sel) {
	case 0:
	    cmd->base.speed = SPEED_10;
	    break;
	case 1:
	    cmd->base.speed = SPEED_100;
	    break;
	case 2:
	    cmd->base.speed = SPEED_1000;
	    break;
	}
    }
    else {
	cmd->base.autoneg = p->req.cmd.base.autoneg;
	cmd->base.speed = SPEED_10000;
	cmd->base.duplex = DUPLEX_FULL;
    }
}

static void tile_sfp_read_eeprom(struct phy *p, u8 *data,
	unsigned off, unsigned len) {
    struct tile_priv *priv = netdev_priv(p->mii_info.dev);
//    printk("%s: tile_sfp_read_eeprom\n", priv->sw.dev->name);
    if (tile_gpio_get(priv->config->gpio_sfp_abs)) {
	return;
    }

    i2c_sfp_try_get(&priv->i2c_sfp, data, off, len, 10);
}

static void tile_sfp_write_eeprom(struct phy *p, u8 *data,
	unsigned off, unsigned len) {
    struct tile_priv *priv = netdev_priv(p->mii_info.dev);
    if (tile_gpio_get(priv->config->gpio_sfp_abs)) {
	return;
    }

    i2c_sfp_set(&priv->i2c_sfp, data, off, len);
}

static struct phy_ops tile_sfp_ops = {
    .init = tile_sfp_init,
    .set_settings = tile_sfp_set_settings,
    .get_settings = tile_sfp_get_settings,
    .read_eeprom = tile_sfp_read_eeprom,
    .write_eeprom = tile_sfp_write_eeprom,
};

static struct phy_ops tile_combo_ops;

static void tile_get_drvinfo(struct net_device *dev,
			     struct ethtool_drvinfo *drvinfo) {
    struct tile_priv *priv = netdev_priv(dev);
    unsigned caps = 0;
    strcpy(drvinfo->driver, "tilegx");
    strcpy(drvinfo->version, "1.0");
    if (priv->config->hv_name_xaui) {
	sprintf(drvinfo->bus_info, "%s", priv->config->hv_name_xaui);
    }
    else {
	sprintf(drvinfo->bus_info, "%s", priv->config->hv_name);
    }

    if (priv->config->sfp) {
	caps |= ETHTOOL_CAP_SFP;
    }
    if (priv->config->sfpplus) {
	caps |= ETHTOOL_CAP_SFP_PLUS;
    }
    if (priv->config->combo) {
	caps |= ETHTOOL_CAP_COMBO;
    }
    if (priv->config->gpio_sfp_abs) {
	caps |= ETHTOOL_CAP_SFP_DETECT_MODULE;
    }
    if (priv->config->gpio_sfp_rx_lose) {
	caps |= ETHTOOL_CAP_SFP_RX_LOSE;
    }
    if (priv->config->gpio_sfp_tx_fault) {
	caps |= ETHTOOL_CAP_SFP_TX_FAULT;
    }
    if (priv->config->gpio_sfp_rate_select || priv->config->gpio_sfp_rate_select2) {
	caps |= ETHTOOL_CAP_SFP_RATE_SELECT;
    }
    if (priv->config->sfp_i2c_adapter_nr) {
	caps |= ETHTOOL_CAP_SFP_I2C_EEPROM;
    }

    if (priv->config->phy_ops == &tile_qt2025_ops) {
	caps |= ETHTOOL_CAP_SFP_DETECT_MODULE;
	caps |= ETHTOOL_CAP_SFP_RX_LOSE;
	caps |= ETHTOOL_CAP_SFP_TX_FAULT;
	caps |= ETHTOOL_CAP_SFP_RATE_SELECT;
	caps |= ETHTOOL_CAP_SFP_I2C_EEPROM;
    }
    ethtool_drvinfo_ext_fill(drvinfo, caps, TILE_JUMBO);
}

static void tile_gbe_update_stats(struct tile_priv *priv) {
    struct eth_stats *x = &priv->sw.xstats[0];

    x->tx_byte += tile_rd(priv, MPIPE_GBE_OCTETS_TX_LO);
    x->tx_byte += tile_rd(priv, MPIPE_GBE_OCTETS_TX_HI) << 32;
    x->tx_packet += tile_rd(priv, MPIPE_GBE_FRAMES_TX);
    x->tx_broadcast += tile_rd(priv, MPIPE_GBE_BCST_FRAMES_TX);
    x->tx_multicast += tile_rd(priv, MPIPE_GBE_MCST_FRAMES_TX);
    x->tx_pause += tile_rd(priv, MPIPE_GBE_PAUSE_FRAMES_TX);
    x->tx_64 += tile_rd(priv, MPIPE_GBE_64_BYTE_FRAMES_TX);
    x->tx_65_127 += tile_rd(priv, MPIPE_GBE_65_TO_127_BYTE_FRAMES_TX);
    x->tx_128_255 += tile_rd(priv, MPIPE_GBE_128_TO_255_BYTE_FRAMES_TX);
    x->tx_256_511 += tile_rd(priv, MPIPE_GBE_256_TO_511_BYTE_FRAMES_TX);
    x->tx_512_1023 += tile_rd(priv, MPIPE_GBE_512_TO_1023_BYTE_FRAMES_TX);
    x->tx_1024_1518 += tile_rd(priv, MPIPE_GBE_1024_TO_1518_BYTE_FRAMES_TX);
    x->tx_1519_max += tile_rd(priv, MPIPE_GBE_GREATER_THAN_1518_BYTE_FRAMES_TX);
    x->tx_underrun += tile_rd(priv, MPIPE_GBE_TX_UNDER_RUNS);
    x->tx_single_col += tile_rd(priv, MPIPE_GBE_SINGLE_COLLISION_FRAMES);
    x->tx_multi_col += tile_rd(priv, MPIPE_GBE_MULTIPLE_COLLISION_FRAMES);
    x->tx_excessive_col += tile_rd(priv, MPIPE_GBE_EXCESSIVE_COLLISIONS);
    x->tx_late_col += tile_rd(priv, MPIPE_GBE_LATE_COLLISIONS);
    x->tx_defered += tile_rd(priv, MPIPE_GBE_DEFERRED_TRANSMISSION_FRAMES);
    x->tx_carrier_sense_err += tile_rd(priv, MPIPE_GBE_CARRIER_SENSE_ERRORS);

    x->rx_byte += tile_rd(priv, MPIPE_GBE_OCTETS_RX_31_0);
    x->rx_byte += tile_rd(priv, MPIPE_GBE_OCTETS_RX_47_32) << 32;
    x->rx_packet += tile_rd(priv, MPIPE_GBE_FRAMES_RX);
    x->rx_broadcast += tile_rd(priv, MPIPE_GBE_BROADCAST_FRAMES_RX);
    x->rx_multicast += tile_rd(priv, MPIPE_GBE_MULTICAST_FRAMES_RX);
    x->rx_pause += tile_rd(priv, MPIPE_GBE_PAUSE_FRAMES_RX);
    x->rx_64 += tile_rd(priv, MPIPE_GBE_64_BYTE_FRAMES_RX);
    x->rx_65_127 += tile_rd(priv, MPIPE_GBE_65_TO_127_BYTE_FRAMES_RX);
    x->rx_128_255 += tile_rd(priv, MPIPE_GBE_128_TO_255_BYTE_FRAMES_RX);
    x->rx_256_511 += tile_rd(priv, MPIPE_GBE_256_TO_511_BYTE_FRAMES_RX);
    x->rx_512_1023 += tile_rd(priv, MPIPE_GBE_512_TO_1023_BYTE_FRAMES_RX);
    x->rx_1024_1518 += tile_rd(priv, MPIPE_GBE_1024_TO_1518_BYTE_FRAMES_RX);
    x->rx_1519_max += tile_rd(priv, MPIPE_GBE_1519_TO_MAXIMUM_BYTE_FRAMES_RX);
    x->rx_too_short += tile_rd(priv, MPIPE_GBE_UNDERSIZED_FRAMES_RX);
    x->rx_too_long += tile_rd(priv, MPIPE_GBE_OVERSIZE_FRAMES_RX);
    x->rx_jabber += tile_rd(priv, MPIPE_GBE_JABBERS_RX);
    x->rx_fcs_err += tile_rd(priv, MPIPE_GBE_FRAME_CHECK_SEQUENCE_ERRORS);
    x->rx_length_err += tile_rd(priv, MPIPE_GBE_LENGTH_FIELD_FRAME_ERRORS);
    x->rx_code_err += tile_rd(priv, MPIPE_GBE_RECEIVE_SYMBOL_ERRORS);
    x->rx_align_err += tile_rd(priv, MPIPE_GBE_ALIGNMENT_ERRORS);
    x->rx_overrun += tile_rd(priv, MPIPE_GBE_RECEIVE_OVERRUNS);
    x->rx_ip_hdr_csum_err += tile_rd(priv, MPIPE_GBE_IP_HEADER_CHECKSUM_ERRORS);
    x->rx_tcp_csum_err += tile_rd(priv, MPIPE_GBE_UDP_CHECKSUM_ERRORS);
    x->rx_udp_csum_err += tile_rd(priv, MPIPE_GBE_TCP_CHECKSUM_ERRORS);
}

static void tile_xgbe_update_stats(struct tile_priv *priv) {
    struct eth_stats *x = &priv->sw.xstats[0];

/*
    MPIPE_XAUI_STATISTICS_CONTROL_t stat_ctrl = {
	.read = 1
    };
    tile_wr(priv, MPIPE_XAUI_STATISTICS_CONTROL, stat_ctrl.word);
*/

    x->tx_byte += tile_rd(priv, MPIPE_XAUI_TRANSMITTED_OCTETS_LO);
    x->tx_byte += tile_rd(priv, MPIPE_XAUI_TRANSMITTED_OCTETS_HI) << 32;
    x->tx_packet += tile_rd(priv, MPIPE_XAUI_TRANSMITTED_FRAMES_LO);
    x->tx_packet += tile_rd(priv, MPIPE_XAUI_TRANSMITTED_FRAMES_HI) << 32;
    x->tx_broadcast += tile_rd(priv, MPIPE_XAUI_TRANSMITTED_BROADCAST_FRAMES);
    x->tx_multicast += tile_rd(priv, MPIPE_XAUI_TRANSMITTED_MULTICAST_FRAMES);
    x->tx_pause += tile_rd(priv, MPIPE_XAUI_TRANSMITTED_PAUSE_FRAMES);
    x->tx_64 += tile_rd(priv, MPIPE_XAUI_64_BYTE_FRAMES_TRANSMITTED);
    x->tx_65_127 += tile_rd(priv, MPIPE_XAUI_65_127_BYTE_FRAMES_TRANSMITTED);
    x->tx_128_255 += tile_rd(priv, MPIPE_XAUI_128_255_BYTE_FRAMES_TRANSMITTED);
    x->tx_256_511 += tile_rd(priv, MPIPE_XAUI_256_511_BYTE_FRAMES_TRANSMITTED);
    x->tx_512_1023 += tile_rd(priv, MPIPE_XAUI_512_1023_BYTE_FRAMES_TRANSMITTED);
    x->tx_1024_1518 += tile_rd(priv, MPIPE_XAUI_1024_1518_BYTE_FRAMES_TRANSMITTED);
    x->tx_1519_max += tile_rd(priv, MPIPE_XAUI_1519_MAX_BYTE_FRAMES_TRANSMITTED);
    x->tx_fcs_err += tile_rd(priv, MPIPE_XAUI_TRANSMITTED_ERROR_FRAMES);

    x->rx_byte += tile_rd(priv, MPIPE_XAUI_RECEIVED_OCTETS_LO);
    x->rx_byte += tile_rd(priv, MPIPE_XAUI_RECEIVED_OCTETS_HI) << 32;
    x->rx_packet += tile_rd(priv, MPIPE_XAUI_RECEIVED_FRAMES_LO);
    x->rx_packet += tile_rd(priv, MPIPE_XAUI_RECEIVED_FRAMES_HI) << 32;
    x->rx_broadcast += tile_rd(priv, MPIPE_XAUI_RECEIVED_BROADCAST_FRAMES);
    x->rx_multicast += tile_rd(priv, MPIPE_XAUI_RECEIVED_MULTICAST_FRAMES);
    x->rx_pause += tile_rd(priv, MPIPE_XAUI_RECEIVED_PAUSE_FRAMES);
    x->rx_64 += tile_rd(priv, MPIPE_XAUI_64_BYTE_FRAMES_RECEIVED);
    x->rx_65_127 += tile_rd(priv, MPIPE_XAUI_65_127_BYTE_FRAMES_RECEIVED);
    x->rx_128_255 += tile_rd(priv, MPIPE_XAUI_128_255_BYTE_FRAMES_RECEIVED);
    x->rx_256_511 += tile_rd(priv, MPIPE_XAUI_256_511_BYTE_FRAMES_RECEIVED);
    x->rx_512_1023 += tile_rd(priv, MPIPE_XAUI_512_1023_BYTE_FRAMES_RECEIVED);
    x->rx_1024_1518 += tile_rd(priv, MPIPE_XAUI_1024_1518_BYTE_FRAMES_RECEIVED);
    x->rx_1519_max += tile_rd(priv, MPIPE_XAUI_1519_MAX_BYTE_FRAMES_RECEIVED);
    x->rx_too_short += tile_rd(priv, MPIPE_XAUI_RECEIVED_SHORT_FRAMES);
    x->rx_too_long += tile_rd(priv, MPIPE_XAUI_RECEIVED_OVERSIZE_FRAMES);
    x->rx_jabber += tile_rd(priv, MPIPE_XAUI_RECEIVED_JABBER_FRAMES);
    x->rx_fcs_err += tile_rd(priv, MPIPE_XAUI_RECEIVED_CRC_ERROR_FRAMES);
    x->rx_length_err += tile_rd(priv, MPIPE_XAUI_RECEIVED_LENGTH_FIELD_ERROR_FRAMES);
    x->rx_code_err += tile_rd(priv, MPIPE_XAUI_RECEIVED_SYMBOL_CODE_ERROR_FRAMES);
}

static void tile_update_stats(struct tile_priv *priv) {
    if (!netif_running(priv->sw.dev)) {
	return;
    }
    if (priv->xaui) {
	tile_xgbe_update_stats(priv);
    }
    else {
	tile_gbe_update_stats(priv);
    }
}

static void tile_get_ethtool_stats(struct net_device *dev,
	struct ethtool_stats *estats, u64 *stats) {
    struct tile_priv *priv = netdev_priv(dev);
    fast_path_get_eth_stats(dev, &priv->sw.xstats[0]);
    tile_update_stats(priv);
    switch_get_ethtool_stats(dev, estats, stats);
}

static void tile_self_test(struct net_device *dev, struct ethtool_test *test,
	u64 *data) {
    struct tile_priv *priv = netdev_priv(dev);
    unsigned flags = test->flags;
    unsigned reg_addr = test->reserved;
    unsigned reg_val = test->reserved2;
    printk("tile: self test\n");
    if (flags & ETHTOOL_FLAG_MAC_REG && reg_addr != -1u) {
//	unsigned long long x = tile_rd(priv, reg_addr);
	unsigned long long x = mpipe_rd(priv->mpipe, reg_addr);
	printk("%04x: %016llx\n", reg_addr, x);
//	tile_wr(priv, reg_addr, reg_val);
	mpipe_wr(priv->mpipe, reg_addr, reg_val);
//	x = tile_rd(priv, reg_addr);
	x = mpipe_rd(priv->mpipe, reg_addr);
	printk("%04x: %016llx\n", reg_addr, x);
    }
    else {
	tile_dump(dev, test->flags);
    }
    if (priv->sw.ops) {
	switch_self_test(&priv->sw, test, data);
    }
}

static const struct ethtool_ops tile_ethtool_ops = {
    .get_drvinfo = tile_get_drvinfo,
    .get_link_ksettings = switch_get_settings,
    .set_link_ksettings = switch_set_settings,
    .get_pauseparam = switch_get_pauseparam,
    .set_pauseparam = switch_set_pauseparam,
    .get_link = switch_get_link,
    .get_strings = switch_get_strings,
    .get_sset_count = switch_get_sset_count,
    .get_ethtool_stats = tile_get_ethtool_stats,
    .get_eeprom_len = switch_get_eeprom_len,
    .get_eeprom = switch_get_eeprom,
    .self_test = tile_self_test,
};


static inline void __tile_push_buffer(struct tile_mpipe *mpipe, void *data) {
    MPIPE_BSM_REGION_ADDR_t offset = {{ 0 }};
    MPIPE_BSM_REGION_VAL_t val = {{ 0 }};
    offset.region =
	MPIPE_MMIO_ADDR__REGION_VAL_BSM - MPIPE_MMIO_ADDR__REGION_VAL_IDMA;
    val.va = __pa(data + BUFFER_CACHE_OFFSET) >> MPIPE_BSM_REGION_VAL__VA_SHIFT;
    kmemleak_ignore(data);

    __insn_mf();
    __gxio_mmio_write(mpipe->ctx.mmio_fast_base + offset.word, val.word);
}

static void *__tile_pop_buffer(struct tile_mpipe *mpipe) {
    MPIPE_BSM_REGION_ADDR_t offset = {{ 0 }};
    MPIPE_BSM_REGION_VAL_t val = {{ 0 }};
    offset.region =
	MPIPE_MMIO_ADDR__REGION_VAL_BSM - MPIPE_MMIO_ADDR__REGION_VAL_IDMA;

    val.word = __gxio_mmio_read(mpipe->ctx.mmio_fast_base + offset.word);
    if (val.c == 3 || !val.va) {
	return NULL;
    }

    return MASK_BUFFER(__va((unsigned long)val.va << MPIPE_BSM_REGION_VAL__VA_SHIFT));
}

static void tile_pop_buffers(struct tile_mpipe *mpipe) {
    unsigned i = 0;

    while (1) {
	void *buf = __tile_pop_buffer(mpipe);
	if (!buf) {
	    break;
	}
	++i;
	kfree(buf);
    }
    printk("tile mpipe%u: popped %u buffers\n", mpipe->instance, i);
}


static void inline tile_mpipe_iqueue_consume(gxio_mpipe_context_t *context,
	gxio_mpipe_iqueue_t* iqueue, unsigned cnt) {
    MPIPE_IDMA_RELEASE_REGION_ADDR_t offset = {{
	    .ring = iqueue->ring,
	    .ring_enable = 1,
	}};
    MPIPE_IDMA_RELEASE_REGION_VAL_t val = {{ .count = cnt }};
    __gxio_mmio_write(context->mmio_fast_base + offset.word, val.word);
}

//#define TEST_LOOPBACK
#ifdef TEST_LOOPBACK
static int tile_fast_path_xmit(struct net_device *dev, struct fp_buf *fpb);
static struct net_device *xmit_map[1000];
#endif
static void tile_handle_packet(struct tile_cpu_info *cpu_info,
	struct tile_mpipe *mpipe, gxio_mpipe_idesc_t *idesc) {
    struct net_device *dev = mpipe->active_devs_for_channel[idesc->channel];
    struct tile_priv *priv = netdev_priv(dev);
#ifdef STATS
    struct tile_stats *stats = &cpu_info->dev_info[priv->devno].stats;
#endif
    unsigned len = idesc->l2_size;
    void *data = __va(idesc->va);
    void *buf = MASK_BUFFER(data);

    if (idesc->be) {
#ifdef STATS
	stats->stats_rx_buffer_error++;
#endif
	return;
    }
    __insn_prefetch_l1_fault(data);

    struct tile_cpu_mpipe_info *mpipe_info =
	&cpu_info->mpipe_info[mpipe->instance];
    --mpipe_info->adjust_buffers_counters;

#ifdef STATS_TIMESTAMPS
    unsigned stamp_idx = atomic_inc_return(&stamps_filled) - 1;
    if (stamp_idx < MAX_TIME_STAMPS) {
	stamps[stamp_idx].cpu = smp_processor_id();
	stamps[stamp_idx].sec = idesc->time_stamp_sec;
	stamps[stamp_idx].nsec = idesc->time_stamp_ns;
	stamps[stamp_idx].flow = idesc->custom0;
    }
#endif

/*
    if (*((unsigned char *)idesc + 0x18) == IPPROTO_GRE) {
	printk("flow_hash:%08x len:%u ethertype:%04x l2_offset:%u l3_offset:%u l4_offset:%u status:%02x protocol:%02x frag:%04x x:%04x\n",
		*(unsigned *)((char *)idesc + 0x0c), len,
		*(unsigned short *)((char *)idesc + 0x12),
		*((unsigned char *)idesc + 0x14),
		*((unsigned char *)idesc + 0x15),
		*((unsigned char *)idesc + 0x16),
		*((unsigned char *)idesc + 0x17),
		*((unsigned char *)idesc + 0x18),
		*(unsigned short *)((char *)idesc + 0x1a),
		*(unsigned short *)((char *)idesc + 0x1c)
	    );
    }
*/
/*
    {
	int i;
	printk("%s: rx cpu:%u data:%p len:%d\n",
		dev->name, smp_processor_id(), data, len);
	for (i = 0; i < 50; ++i) {
	    printk("%02x ", data[i]);
	}
	printk("\n");
    }
*/

#ifdef STATS
    stats->stats_rx++;
#endif

    struct fp_buf *fpb = fpb_build(buf, data - buf, len);
#ifdef TEST_LOOPBACK
    struct net_device *dst = xmit_map[dev->ifindex];
    if (dst) {
	tile_fast_path_xmit(dst, fpb);
	return;
    }
#endif
    if (priv->sw.ops) {
	switch_rx_fast(&priv->sw, fpb);
    }
    else {
	fast_path_rx(dev, fpb);
    }
}

static void tile_current_buffer_fix(struct tile_mpipe *mpipe, int fix) {
    spin_lock(&mpipe->adjust_buffers_lock);
    mpipe->buffers_current_approx += fix;
    spin_unlock(&mpipe->adjust_buffers_lock);
}

static int tile_current_buffer_adj(struct tile_mpipe *mpipe, int adj) {
    int need = 0;
    int bufs;
    spin_lock(&mpipe->adjust_buffers_lock);

    bufs = mpipe->buffers_current_approx;
    bufs += adj;
    if (bufs < mpipe->buffers_min) {
	need = mpipe->buffers_min - bufs;
	if (need > ADJUST_BUFFERS_BATCH * 2) {
	    need = ADJUST_BUFFERS_BATCH * 2;
	}
    }
    if (bufs > mpipe->buffers_max) {
	need = mpipe->buffers_max - bufs;
	if (need < -ADJUST_BUFFERS_BATCH * 2) {
	    need = -ADJUST_BUFFERS_BATCH * 2;
	}
    }
    bufs += need;
    mpipe->buffers_current_approx = bufs;
//    printk("buffers current approx:%d adj:%d need:%d\n", bufs, adj, need);

    spin_unlock(&mpipe->adjust_buffers_lock);
    return need;
}

static void tile_mpipe_adjust_buffers(struct tile_mpipe *mpipe) {
    void *buf;
    struct tile_cpu_info *cpu_info = raw_cpu_ptr(&per_cpu_info);
    struct tile_cpu_mpipe_info *mpipe_info =
	&cpu_info->mpipe_info[mpipe->instance];
    int adj_curr = 0;
    int need = 0;

    if (mpipe_info->adjust_buffers_counters < -ADJUST_BUFFERS_BATCH) {
	mpipe_info->adjust_buffers_counters += ADJUST_BUFFERS_BATCH;
	adj_curr = -ADJUST_BUFFERS_BATCH;
    }
    else if (mpipe_info->adjust_buffers_counters > ADJUST_BUFFERS_BATCH) {
	mpipe_info->adjust_buffers_counters -= ADJUST_BUFFERS_BATCH;
	adj_curr = ADJUST_BUFFERS_BATCH;
    }
    else {
	return;
    }

    need = tile_current_buffer_adj(mpipe, adj_curr);
    if (need > 0) {
	for (; need; --need) {
	    buf = __skb_bin_get_buffer();
	    if (!buf) {
		if (net_ratelimit()) {
		    printk("tilegx: could not alloc buffer\n");
		}
		tile_current_buffer_fix(mpipe, -need);
		break;
	    }
	    __tile_push_buffer(mpipe, buf);
#ifdef STATS
	    ++cpu_info->stats.stats_push;
#endif
	}
    }
    else if (need < 0) {
	for (; need; ++need) {
	    buf = __tile_pop_buffer(mpipe);
	    if (!buf) {
		if (net_ratelimit()) {
		    printk("tilegx: could not pop buffer\n");
		}
		tile_current_buffer_fix(mpipe, -need);
		break;
	    }
	    __skb_bin_put_buffer(buf);
#ifdef STATS
	    ++cpu_info->stats.stats_pop;
#endif
	}
    }
}

static void tile_adjust_buffers(void) {
    unsigned i;
    for (i = 0; i < NR_MPIPE_MAX; ++i) {
	if (state.mpipe[i].open_devs) {
	    tile_mpipe_adjust_buffers(&state.mpipe[i]);
	}
    }
}

static inline void tile_enable_notif_ring_interrupt(struct tile_mpipe *mpipe,
	unsigned ring) {
    mpipe_wr(mpipe, MPIPE_INT_VEC3_W1TC + (ring / 64) * 8, 1ull << (ring % 64));
}

static int tile_poll(struct napi_struct *napi, int budget) {
    struct tile_cpu_info *cpu_info = raw_cpu_ptr(&per_cpu_info);
    struct tile_rx_queue *rxq = container_of(napi, struct tile_rx_queue, napi);
    struct tile_mpipe *mpipe = rxq->mpipe;
    gxio_mpipe_iqueue_t *iqueue = &rxq->iqueue;
    unsigned work = 0;
    unsigned i;

#ifdef STATS
    ++cpu_info->stats.stats_poll;
    tile_stats();
#endif

    uint64_t head = iqueue->head;
    while (1) {
	uint64_t tail = __gxio_mmio_read(iqueue->idescs);
	if (head == tail) {
	    break;
	}
	i = 0;
	do {
	    tile_handle_packet(cpu_info, mpipe, iqueue->idescs + head);
	    ++head;
	    head = (head & TILE_IQUEUE_MASK) + (head >> TILE_IQUEUE_LOG2);
	    ++work;
	    ++i;
	    if (work >= budget) {
		tile_mpipe_iqueue_consume(&mpipe->ctx, &rxq->iqueue, i);
		goto done;
	    }
	} while (head != tail);
	tile_mpipe_iqueue_consume(&mpipe->ctx, &rxq->iqueue, i);
    }

    napi_complete(&rxq->napi);

    tile_enable_notif_ring_interrupt(mpipe, rxq->iqueue.ring);
    uint64_t tail = __gxio_mmio_read(iqueue->idescs);
    if (tail != head) {
	napi_schedule(&rxq->napi);
    }
done:

    call_xmit_commits();

    iqueue->head = head;
    tile_adjust_buffers();
    return work;
}


static irqreturn_t tile_ingress_irq(int irq, void *unused) {
    struct tile_cpu_info *cpu_info = raw_cpu_ptr(&per_cpu_info);
//    printk("tile net irq %d\n", smp_processor_id());
#ifdef STATS
    ++cpu_info->stats.stats_intr;
#endif
    napi_schedule(&cpu_info->rxq[state.irq_to_rxq[irq]].napi);
    return IRQ_HANDLED;
}

static irqreturn_t tile_phy_1g_irq(int irq, void *unused) {
    schedule_work(&state.phy_1g_irq_work);
    return IRQ_HANDLED;
}

static void tile_phy_1g_irq_work(struct work_struct *work) {
    struct tile_state *state =
	container_of(work, struct tile_state, phy_1g_irq_work);
    u64 deasserted;
    unsigned i;
    struct tile_priv *priv;
//    unsigned x;

    gxio_gpio_report_reset_interrupt(&state->gpio_ctx, NULL, &deasserted);
//    printk("gpio interrupt deasserted:%016llx\n", deasserted);

    for (i = 0; i < TILE_DEVS_MAX; ++i) {
	if (!state->devs[i]) {
	    continue;
	}
	priv = netdev_priv(state->devs[i]);
	if (!netif_running(priv->sw.dev)) {
	    continue;
	}
	if (!priv->config->phy_1g) {
	    continue;
	}
	if (!priv->config->gpio_phy_1g_int) {
	    continue;
	}
	if (!(BIT(priv->config->gpio_phy_1g_int) & deasserted)) {
	    continue;
	}
	phy_cl22_rd(&priv->sw.phy, 0x13);
//	printk("%s: phy interrupt: %04x\n", priv->sw.dev->name, x);
	phy_manual_update(&priv->sw.phy);
    }
}

static gxio_mpipe_buffer_size_enum_t tile_buffer_size_enum(int buffer_size) {
    if (buffer_size <= 128) {
	return GXIO_MPIPE_BUFFER_SIZE_128;
    }
    if (buffer_size <= 256) {
	return GXIO_MPIPE_BUFFER_SIZE_256;
    }
    if (buffer_size <= 512) {
	return GXIO_MPIPE_BUFFER_SIZE_512;
    }
    if (buffer_size <= 1024) {
	return GXIO_MPIPE_BUFFER_SIZE_1024;
    }
    if (buffer_size <= 1664) {
	return GXIO_MPIPE_BUFFER_SIZE_1664;
    }
    if (buffer_size <= 4096) {
	return GXIO_MPIPE_BUFFER_SIZE_4096;
    }
    if (buffer_size <= 10368) {
	// XXX: avoid using buffer stack of this size as there is problem with releasing buffers to such stack when xmitting packet bigger than 6144 (0x1800)
	return GXIO_MPIPE_BUFFER_SIZE_16384;
//	return GXIO_MPIPE_BUFFER_SIZE_10368;
    }
    if (buffer_size <= 16384) {
	return GXIO_MPIPE_BUFFER_SIZE_16384;
    }
    BUG();
}

static int tile_buffer_size(gxio_mpipe_buffer_size_enum_t bs) {
    switch (bs) {
    case GXIO_MPIPE_BUFFER_SIZE_128:
	return 128;
    case GXIO_MPIPE_BUFFER_SIZE_256:
	return 256;
    case GXIO_MPIPE_BUFFER_SIZE_512:
	return 512;
    case GXIO_MPIPE_BUFFER_SIZE_1024:
	return 1024;
    case GXIO_MPIPE_BUFFER_SIZE_1664:
	return 1664;
    case GXIO_MPIPE_BUFFER_SIZE_4096:
	return 4096;
    case GXIO_MPIPE_BUFFER_SIZE_10368:
	return 10368;
    case GXIO_MPIPE_BUFFER_SIZE_16384:
	return 16384;
    default:
	BUG();
    }
}

static int tile_mpipe_get_l2mtu(void) {
    int l2mtu = 1500;
    int i;
    for (i = 0; i < TILE_DEVS_MAX; ++i) {
	if (!state.devs[i]) {
	    continue;
	}
	if (state.devs[i]->l2mtu > l2mtu) {
	    l2mtu = state.devs[i]->l2mtu;
	}
    }
    return l2mtu;
}

static int tile_mpipe_get_buffer_size(int l2mtu) {
    // packets get received at NET_IP_ALIGN + NET_SKB_PAD in mpipe buffer,
    // and this padding reduces the maximum packet to be received
    int bs = l2mtu + ETH_HLEN + NET_IP_ALIGN + NET_SKB_PAD;

    // normalize to smallest buffer size that fits requested l2mtu
    bs = tile_buffer_size(tile_buffer_size_enum(bs));

    return bs;
}

static inline unsigned tile_next_tx_slot(struct tile_priv *priv, unsigned slot) {
    unsigned next_slot = slot + 1;
    if (next_slot == priv->per_cpu_tx_queue_size) {
	next_slot = 0;
    }
    return next_slot;
}

static unsigned tile_tx(struct tile_priv *priv,
	struct tile_cpu_dev_info *info) {
    struct tile_tx_queue *tx_queue = &priv->tx_queue;
    u64 tx_completed = tile_update_tx_completed(tx_queue);
    unsigned queued_bits = -1u;

//    printk("%s: tile_tx cpu:%u head:%u tail:%u compl:%llu\n",
//	    priv->sw.dev->name, smp_processor_id(),
//	    info->tx_head, info->tx_tail, tx_completed);
    while (info->tx_tail != info->tx_head) {
	struct tile_tx_entry *entry = &info->tx_entries[info->tx_tail];
	if (tx_completed <= entry->slot) {
	    break;
	}
//	printk("cpu%02u tail:%u head:%u slot:%lu tx_comp:%llu\n",
//		smp_processor_id(),
//		info->tx_tail, info->tx_head, entry->slot, tx_completed);
#ifdef STATS
	info->stats.stats_tx++;
#endif
	if (entry->buf) {
	    dev_kfree_skb(entry->buf);
	    entry->buf = NULL;
	}
	if (priv->sw.ops) {
	    switch_tx(&priv->sw, info->cpu, 1);
	}
	info->tx_tail = tile_next_tx_slot(priv, info->tx_tail);
	queued_bits = __insn_fetchadd4(&tx_queue->queued_bits, -entry->bits) - entry->bits;
    }
    info->last_tx_time = cycles_get_cycles();
    return queued_bits;
}

/*
static inline unsigned tile_in_transmit(struct tile_priv *priv,
	struct tile_cpu_dev_info *info) {
    unsigned head = info->tx_head;
    unsigned tail = info->tx_tail;
    if (head >= tail) {
	return head - tail;
    }
    return head + priv->per_cpu_tx_queue_size - tail;
}
*/

static void tile_tx_work(struct work_struct *w) {
    struct tile_cpu_dev_info *cpu_dev_info =
	container_of(w, struct tile_cpu_dev_info, tx_work);
    struct tile_priv *priv = netdev_priv(cpu_dev_info->dev);
    struct tile_tx_queue *tx_queue = &priv->tx_queue;
    unsigned prev_tx_tail = cpu_dev_info->tx_tail;

    local_bh_disable();
#ifdef DEBUG
    u64 x = atomic64_inc_return(raw_cpu_ptr(&per_cpu_info.xmit_count));
    if (x > 1) {
	printk("%s: %u %u tx work recursion: %llu\n",
		priv->sw.dev->name, cpu_dev_info->cpu, smp_processor_id(), x);
	WARN_ON(1);
    }
    if (smp_processor_id() != cpu_dev_info->cpu) {
	printk("%s: tx_work actual cpu:%u required cpu:%u",
		cpu_dev_info->dev->name, smp_processor_id(), cpu_dev_info->cpu);
	WARN_ON(1);
    }
#endif
#ifdef STATS
    cpu_dev_info->stats.stats_tx_work++;
#endif
    unsigned queued_bits = tile_tx(priv, cpu_dev_info);
//    printk("%s: tx_work cpu:%u old:%u tail:%u head:%u\n",
//	    cpu_dev_info->dev->name, smp_processor_id(),
//	    old_tail, cpu_dev_info->tx_tail, cpu_dev_info->tx_head);
    if ((tile_next_tx_slot(priv, cpu_dev_info->tx_head) != cpu_dev_info->tx_tail &&
		    queued_bits != -1u && queued_bits < tx_queue->max_queued_bits) || cpu_dev_info->tx_tail == cpu_dev_info->tx_head) {
	netif_wake_subqueue(cpu_dev_info->dev, cpu_dev_info->cpu);
	if (priv->sw.ops) {
	    switch_wake_all_port_queue(&priv->sw, cpu_dev_info->cpu);
	}
    }
    if (cpu_dev_info->tx_tail != cpu_dev_info->tx_head) {
	hrtimer_start(
	    &cpu_dev_info->tx_timer,
	    ktime_set(0, cpu_dev_info->tx_timer_interval),
	    HRTIMER_MODE_REL_PINNED);
	if (prev_tx_tail == cpu_dev_info->tx_tail) {
	    cpu_dev_info->tx_timer_interval *= 2;
	}
	else {
	    cpu_dev_info->tx_timer_interval /= 2;
	}
	if (cpu_dev_info->tx_timer_interval < TX_TIMER_INTERVAL_MIN) {
	    cpu_dev_info->tx_timer_interval = TX_TIMER_INTERVAL_MIN;
	}
	if (cpu_dev_info->tx_timer_interval > TX_TIMER_INTERVAL_MAX) {
	    cpu_dev_info->tx_timer_interval = TX_TIMER_INTERVAL_MAX;
	}
    }
    else {
	cpu_dev_info->tx_timer_interval = TX_TIMER_INTERVAL_MIN;
    }
#ifdef DEBUG
    atomic64_dec(raw_cpu_ptr(&per_cpu_info.xmit_count));
#endif
    local_bh_enable();
}

static enum hrtimer_restart tile_tx_timer(struct hrtimer *t) {
    struct tile_cpu_dev_info *cpu_dev_info =
	container_of(t, struct tile_cpu_dev_info, tx_timer);
    struct tile_priv *priv = netdev_priv(cpu_dev_info->dev);

//    printk("%s: tx_timer: %u\n", smp_processor_id());
    if (!priv->closing) {
	schedule_work_on(cpu_dev_info->cpu, &cpu_dev_info->tx_work);
    }

    return HRTIMER_NORESTART;
}

static void tile_init_stacks(struct tile_mpipe *mpipe) {
    int err;
    unsigned stack_bytes;
    pte_t pte;
    pte = pte_set_home(pte, PAGE_HOME_HASH);

    stack_bytes = gxio_mpipe_calc_buffer_stack_bytes(mpipe->buffers_capacity);
    stack_bytes = ROUND_UP(stack_bytes, PAGE_SIZE);

    printk("tile mpipe%u: pte %llx\n", mpipe->instance, pte.val);
    printk("tile mpipe%u: stack_bytes: %u\n", mpipe->instance, stack_bytes);

    if (stack_bytes > HPAGE_SIZE) {
	panic("Cannot fit buffers in one huge page.");
    }

    unsigned x = gxio_mpipe_alloc_buffer_stacks(&mpipe->ctx, 1, 0, 0);
    if (x) {
	panic("Failure in gxio_mpipe_alloc_buffer_stacks()");
    }
    printk("tile mpipe%u: using buffer stack %u\n", mpipe->instance, x);

    mpipe->buffer_stack_bytes = stack_bytes;
    mpipe->buffer_stack_pages =
	alloc_pages(GFP_KERNEL, HUGETLB_PAGE_ORDER);
    if (!mpipe->buffer_stack_pages) {
	panic("Could not allocate buffer stack memory!");
    }

/*
    unsigned *mem = pfn_to_kaddr(page_to_pfn(mpipe->buffer_stack_pages));
    printk("mem: %p\n", mem);
    *mem = 0;
    hv_halt();
*/

    err = gxio_mpipe_register_client_memory(
	&mpipe->ctx, 0, pte, 0);
//	&mpipe->ctx, 0, pte, GXIO_MPIPE_MEM_FLAG_NT_HINT);
    if (err) {
	panic("Error %d in gxio_mpipe_register_buffer_memory()", err);
    }
}


static int tile_mpipe_init_notif_group_and_buckets(struct tile_mpipe *mpipe,
	unsigned int group, unsigned int first_ring, unsigned int bucket,
	gxio_mpipe_bucket_mode_t mode) {
    int i;
    int result;

    MPIPE_LBL_INIT_DAT_BSTS_TBL_t bucket_info = {{
	    .group = group,
	    .mode = mode,
	}};

    gxio_mpipe_notif_group_bits_t bits = {{ 0 }};
    unsigned rings[NR_CPUS];

    for (i = 0; i < mpipe->rx_cpus_count; i++) {
#define SPR_PSEUDO_RANDOM_NUMBER 0x270a
	unsigned rxq = __insn_mfspr(SPR_PSEUDO_RANDOM_NUMBER)
	    % mpipe->iqueues_per_cpu;
	rings[i] = first_ring + rxq * mpipe->rx_cpus_count + i;
//	printk("rx cpu%u rxq:%u\n", i, rxq);
	gxio_mpipe_notif_group_add_ring(&bits, rings[i]);
    }

    result = gxio_mpipe_init_notif_group(&mpipe->ctx, group, bits);
    if (result != 0)
	return result;

    for (i = 0; i < BUCKETS_PER_DEV; i++) {
	bucket_info.notifring = rings[i % mpipe->rx_cpus_count];

	result = gxio_mpipe_init_bucket(&mpipe->ctx, bucket + i, bucket_info);
	if (result != 0)
	    return result;
    }

    return 0;
}

static void tile_mpipe_init_round_robin_bucket(struct tile_mpipe *mpipe,
	unsigned int group) {
    int i;
    int result;

    MPIPE_LBL_INIT_DAT_BSTS_TBL_t bucket_info = {{
	    .group = group,
	    .mode = GXIO_MPIPE_BUCKET_ROUND_ROBIN,
	}};

    gxio_mpipe_notif_group_bits_t bits = {{ 0 }};
    unsigned rings[NR_CPUS];

    for (i = 0; i < mpipe->rx_cpus_count; i++) {
	unsigned rxq = __insn_mfspr(SPR_PSEUDO_RANDOM_NUMBER)
	    % mpipe->iqueues_per_cpu;
	rings[i] = rxq * mpipe->rx_cpus_count + i;
	gxio_mpipe_notif_group_add_ring(&bits, rings[i]);
    }

    result = gxio_mpipe_init_notif_group(&mpipe->ctx, group, bits);
    if (result != 0) {
	printk("tilegx: init_round_robin_bucket gxio_mpipe_init_notif_group: %d\n", result);
	return;
    }

    bucket_info.notifring = rings[0];

    result = gxio_mpipe_init_bucket(&mpipe->ctx, mpipe->rr_bucket, bucket_info);
    if (result != 0) {
	printk("tilegx: init_round_robin_bucket gxio_mpipe_init_bucket: %d\n", result);
	return;
    }
}

static void tile_mpipe_init(struct tile_mpipe *mpipe) {
    int first_ring;
    int first_group;
    int cpu;
    gxio_mpipe_bucket_mode_t mode = GXIO_MPIPE_BUCKET_STICKY_FLOW_LOCALITY;
//    gxio_mpipe_bucket_mode_t mode = GXIO_MPIPE_BUCKET_STATIC_FLOW_AFFINITY;
    unsigned i;
    unsigned k;

    if (!hash_default) {
	printk("WARNING: Networking requires hash_default!\n");
    }

    if (mpipe->ctx.mmio_cfg_base) {
	// already initialized
	return;
    }

    if (gxio_mpipe_init(&mpipe->ctx, mpipe->instance)) {
	printk("tile mpipe%u: failed to initialize\n", mpipe->instance);
	return;
    }

    mpipe->rx_cpus_map = *(cpumask_t *) cpu_online_mask;

/*
    for_each_online_cpu(i) {
//	printk("i: %u - %u,%u\n", i, cpu_x(i), cpu_y(i));
	if ((i % 2) == mpipe->instance) {
//	if ((i / 36) != mpipe->instance) {
//	if ((cpu_x(i) / 4)  == mpipe->instance) {
	    cpumask_clear_cpu(i, &mpipe->rx_cpus_map);
	}
    }
    printk("tile mpipe%u: rx cpus ", mpipe->instance);
    for_each_cpu(i, &mpipe->rx_cpus_map) {
	printk("%u ", i);
    }
    printk("\n");
*/
/*
    cpumask_clear(&mpipe->rx_cpus_map);
    cpumask_set_cpu(0, &mpipe->rx_cpus_map);
    cpumask_set_cpu(1, &mpipe->rx_cpus_map);
    cpumask_set_cpu(2, &mpipe->rx_cpus_map);
    cpumask_set_cpu(3, &mpipe->rx_cpus_map);
    cpumask_set_cpu(4, &mpipe->rx_cpus_map);
*/
    mpipe->rx_cpus_count = cpumask_weight(&mpipe->rx_cpus_map);
    mpipe->tx_cpus_count = num_online_cpus();
    printk("tile mpipe%u: rx_cpus_count: %u\n",
	    mpipe->instance, mpipe->rx_cpus_count);
    printk("tile mpipe%u: tx_cpus_count: %u\n",
	    mpipe->instance, mpipe->tx_cpus_count);

    mpipe->iqueues_per_cpu = 2;
    if (num_online_cpus() >= 72) {
	mpipe->iqueues_per_cpu = 1;
    }
    unsigned base = TILE_IQUEUE_SIZE * mpipe->iqueues_per_cpu * mpipe->rx_cpus_count;
    mpipe->buffers_min = base * 2;
    mpipe->buffers_max = base * 4;
    mpipe->buffers_capacity = base * 8;
    printk("tile mpipe%u: iqueues_per_cpu: %u\n",
	    mpipe->instance, mpipe->iqueues_per_cpu);
    printk("tile mpipe%u: buffers min:%u max:%u capacity:%u\n",
	    mpipe->instance, mpipe->buffers_min, mpipe->buffers_max,
	    mpipe->buffers_capacity);


    tile_init_stacks(mpipe);

    first_ring = gxio_mpipe_alloc_notif_rings(
	&mpipe->ctx, mpipe->rx_cpus_count * mpipe->iqueues_per_cpu, 0, 0);
    if (first_ring < 0) {
	panic("Failure in 'gxio_mpipe_alloc_notif_rings()'.");
    }
    printk("tile mpipe%u: using notif rings %u-%u\n", mpipe->instance,
	    first_ring,
	    first_ring + mpipe->rx_cpus_count * mpipe->iqueues_per_cpu - 1);

    k = 0;
    for_each_cpu(cpu, &mpipe->rx_cpus_map) {
	struct tile_cpu_info *cpu_info = state.cpu_info[cpu];
	struct tile_cpu_mpipe_info *mpipe_info =
	    &cpu_info->mpipe_info[mpipe->instance];
	size_t notif_ring_size = TILE_IQUEUE_SIZE * sizeof(gxio_mpipe_idesc_t);
	size_t notif_ring_mem = ALIGN(notif_ring_size, 4096);
	void *addr;

	memset(mpipe_info, 0, sizeof(*mpipe_info));
	mpipe_info->iqueue_order =
	    get_order(notif_ring_mem * mpipe->iqueues_per_cpu);
	mpipe_info->iqueue_pages = homecache_alloc_pages(
	    GFP_KERNEL, mpipe_info->iqueue_order, cpu);
	if (!mpipe_info->iqueue_pages) {
	    panic("Failed to allocate iqueue memory.");
	}
	addr = pfn_to_kaddr(page_to_pfn(mpipe_info->iqueue_pages));
/*
	if (cpu == 1) {
	    printk("%u iqueue_pages addr:%p\n", cpu, addr);
	    *(unsigned *)addr = 0;
	    hv_halt();
	}
*/
	memset(addr, 0, notif_ring_mem * mpipe->iqueues_per_cpu);

	for (i = 0; i < mpipe->iqueues_per_cpu; ++i) {
	    unsigned q = mpipe->instance * MPIPE_IQUEUES_PER_CPU_MAX + i;
	    cpu_info->rxq[q].mpipe = mpipe;
	    if (gxio_mpipe_iqueue_init(&cpu_info->rxq[q].iqueue, &mpipe->ctx,
			    i * mpipe->rx_cpus_count + k,
			    addr + notif_ring_mem * i,
			    notif_ring_size, 0) != 0) {
		panic("Failure in 'gxio_mpipe_iqueue_init()'.");
	    }
	}
	++k;
    }

    first_group = gxio_mpipe_alloc_notif_groups(
	&mpipe->ctx, TILE_DEVS_MAX + 1, 0, 0);
    if (first_group < 0) {
	panic("Failure in 'gxio_mpipe_alloc_notif_groups()'.");
    }
    printk("tile mpipe%u: using notif groups %u-%u\n", mpipe->instance,
	    first_group, first_group + TILE_DEVS_MAX + 1 - 1);

    mpipe->first_bucket = gxio_mpipe_alloc_buckets(
	&mpipe->ctx, TILE_DEVS_MAX * BUCKETS_PER_DEV, 0, 0);
    if (mpipe->first_bucket < 0) {
	panic("Failure in 'gxio_mpipe_alloc_buckets()'.");
    }
    mpipe->rr_bucket =
	gxio_mpipe_alloc_buckets(&mpipe->ctx, 1, 0, 0);
    if (mpipe->rr_bucket < 0) {
	panic("Failure in 'gxio_mpipe_alloc_buckets()'.");
    }
    printk("tile mpipe%u: rr_bucket:%u\n", mpipe->instance, mpipe->rr_bucket);

    for (i = 0; i < TILE_DEVS_MAX; ++i) {
	if (tile_mpipe_init_notif_group_and_buckets(
		    mpipe, first_group + i, first_ring,
		    mpipe->first_bucket + i * BUCKETS_PER_DEV, mode) != 0) {
	    panic("Fail in 'gxio_mpipe_init_notif_group_and_buckets().");
	}
    }

    tile_mpipe_init_round_robin_bucket(mpipe, first_group + TILE_DEVS_MAX);

    for (i = 0; i < mpipe->iqueues_per_cpu; ++i) {
	unsigned q = mpipe->instance * MPIPE_IQUEUES_PER_CPU_MAX + i;
	mpipe->ingress_irq[i] = irq_alloc_hwirq(-1);
	if (mpipe->ingress_irq[i] < 0) {
	    panic("Failed to create irq for ingress.");
	}
	state.irq_to_rxq[mpipe->ingress_irq[i]] = q;
	tile_irq_activate(mpipe->ingress_irq[i], TILE_IRQ_PERCPU);
	BUG_ON(request_irq(mpipe->ingress_irq[i], tile_ingress_irq, 0, "eth", NULL));
    }

    for_each_cpu(cpu, &mpipe->rx_cpus_map) {
	struct tile_cpu_info *cpu_info = state.cpu_info[cpu];

	for (i = 0; i < mpipe->iqueues_per_cpu; ++i) {
	    unsigned q = mpipe->instance * MPIPE_IQUEUES_PER_CPU_MAX + i;
	    int ring = cpu_info->rxq[q].iqueue.ring;
	    MPIPE_INT_BIND_t data = {{
		    .enable = 1,
		    .mode = 0,
		    .tileid = (cpu_x(cpu) << 4) | cpu_y(cpu),
		    .int_num = KERNEL_PL,
		    .evt_num = mpipe->ingress_irq[i],
		    .vec_sel = 3 + ring / 64,
		    .bind_sel = ring % 64,
		}};
	    mpipe_wr(mpipe, MPIPE_INT_BIND, data.word);

	    tile_enable_notif_ring_interrupt(mpipe, ring);
	}
    }

    MPIPE_CLS_CTL_t cls_ctl;
    cls_ctl.word = mpipe_rd(mpipe, MPIPE_CLS_CTL);
    cls_ctl.default_stack = 0;
    cls_ctl.default_nr = first_ring;
    cls_ctl.default_dest = 1;
    mpipe_wr(mpipe, MPIPE_CLS_CTL, cls_ctl.word);
    printk("tile mpipe%u: budget(min:%u adj:%u ovhd:%u mult:%u) default(stack:%u nr:%u dest:%u) rand_mode:%u\n",
	    mpipe->instance,
	    cls_ctl.budget_min,
	    cls_ctl.budget_adj,
	    cls_ctl.budget_ovhd,
	    cls_ctl.budget_mult,
	    cls_ctl.default_stack,
	    cls_ctl.default_nr,
	    cls_ctl.default_dest,
	    cls_ctl.rand_mode);

/*
    MPIPE_EDMA_CTL_t edma_ctl;
    edma_ctl.word = mpipe_rd(mpipe, MPIPE_EDMA_CTL);
//    edma_ctl.ud_blocks = 0;
//    edma_ctl.hunt_cycles = 0xfff;
//    edma_ctl.max_req = 2;
    mpipe_wr(mpipe, MPIPE_EDMA_CTL, edma_ctl.word);
*/

    MPIPE_EDMA_BW_CTL_t edma_bw_ctl;
    edma_bw_ctl.word = mpipe_rd(mpipe, MPIPE_EDMA_BW_CTL);
    edma_bw_ctl.prior1_rate = 5;
    edma_bw_ctl.prior0_rate = 1;
    mpipe_wr(mpipe, MPIPE_EDMA_BW_CTL, edma_bw_ctl.word);

/*
    MPIPE_EDMA_DIAG_CTL_t edma_diag_ctl;
    edma_diag_ctl.word = mpipe_rd(mpipe, MPIPE_EDMA_DIAG_CTL);
//    edma_diag_ctl.disable_final_buf_rtn = 1;
    mpipe_wr(mpipe, MPIPE_EDMA_DIAG_CTL, edma_diag_ctl.word);
*/

    // select mpipe perf counters
    MPIPE_LBL_CTL_t lbl_ctl;
    lbl_ctl.word = mpipe_rd(mpipe, MPIPE_LBL_CTL);
    lbl_ctl.ctr_sel = 3;
    mpipe_wr(mpipe, MPIPE_LBL_CTL, lbl_ctl.word);

    MPIPE_IDMA_CTL_t idma_ctl;
    idma_ctl.word = mpipe_rd(mpipe, MPIPE_IDMA_CTL);
    idma_ctl.idma_evt_ctr_sel = 10;
    idma_ctl.ipkt_evt_ctr_sel = 2;
    mpipe_wr(mpipe, MPIPE_IDMA_CTL, idma_ctl.word);

    spin_lock_init(&mpipe->adjust_buffers_lock);
}

static void tile_mpipe_open_cpu(struct tile_mpipe *mpipe,
	struct tile_cpu_info *cpu_info, unsigned cpu) {
    unsigned k;
    unsigned i;
//    printk("open_cpu %u\n", cpu);
    for (i = 0; i < mpipe->iqueues_per_cpu; ++i) {
	unsigned q = mpipe->instance * MPIPE_IQUEUES_PER_CPU_MAX + i;

	netif_napi_add(state.dummy_netdev, &cpu_info->rxq[q].napi,
		tile_poll, 64);
	napi_enable(&cpu_info->rxq[q].napi);
    }

    for (k = 0; k < TILE_DEVS_MAX; ++k) {
	if (!state.devs[k]) {
	    continue;
	}
	cpu_info->dev_info[k].dev = state.devs[k];
	cpu_info->dev_info[k].cpu = cpu;

	INIT_WORK(&cpu_info->dev_info[k].tx_work, tile_tx_work);

	hrtimer_init(&cpu_info->dev_info[k].tx_timer,
		CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	cpu_info->dev_info[k].tx_timer.function = tile_tx_timer;
	cpu_info->dev_info[k].tx_timer_interval = TX_TIMER_INTERVAL_MIN;
    }
}

static struct net_device_ops mpipe_manag_ops = {
};

static void tile_mpipe_dummy_init(struct net_device *dev) {
    dev->netdev_ops = &mpipe_manag_ops;
}

static int tile_mpipe_open(struct tile_mpipe *mpipe) {
    unsigned i;
    int err;
    void *mem;

    if (!state.open_mpipes) {
	state.l2mtu = tile_mpipe_get_l2mtu();
	printk("tile: l2mtu: %u\n", state.l2mtu);
	state.mpipe_buffer_size = tile_mpipe_get_buffer_size(state.l2mtu);

	state.dummy_netdev = alloc_netdev(0, "mpipe", NET_NAME_UNKNOWN, tile_mpipe_dummy_init);
	state.dummy_netdev->needs_free_netdev = true;
	err = register_netdevice(state.dummy_netdev);
	if (err) {
	    printk("tile: register_netdev: %d\n", err);
	}
	err = dev_open(state.dummy_netdev, NULL);
	if (err) {
	    printk("tile: dev_open: %d\n", err);
	}

	state.mpipe_buffer_size_enum =
	    tile_buffer_size_enum(state.mpipe_buffer_size);
	printk("tile: mpipe buffer size enum: %d\n", state.mpipe_buffer_size_enum);
	printk("tile: mpipe buffer size: %d\n", state.mpipe_buffer_size);
	printk("tile: sizeof(struct tile_cpu_info): %lu\n", sizeof(struct tile_cpu_info));
	printk("tile: sizeof(struct tile_cpu_dev_info): %lu\n", sizeof(struct tile_cpu_dev_info));
	printk("tile: sizeof(struct tile_tx_entry): %lu\n", sizeof(struct tile_tx_entry));
	printk("tile: TILE_IQUEUE_SIZE: %u\n", TILE_IQUEUE_SIZE);

	if (ports_config == ccr1072_1g_8splus_ports) {
	    printk("tile: take TI phys out of reset\n");
	    access_latch(0, BIT(3) | BIT(5) | BIT(7) | BIT(15),
		    BIT(3) | BIT(5) | BIT(7) | BIT(15));
	}
	if (ports_config == ccr1072_1g_8splus_r3_ports) {
	    printk("tile: take QT2025 phys out of reset\n");
	    access_latch(0,
		    BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(14) | BIT(15),
		    BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(14) | BIT(15));
	}
    }

    tile_mpipe_init(mpipe);
//    printk("tile_mpipe_open\n");

    mem = pfn_to_kaddr(page_to_pfn(mpipe->buffer_stack_pages));
    memset(mem, 0, mpipe->buffer_stack_bytes);

    err = gxio_mpipe_init_buffer_stack_aux(&mpipe->ctx,
	    mem + mpipe->buffer_stack_bytes, mpipe->buffer_stack_bytes,
	    0, 0, state.mpipe_buffer_size_enum);
    if (err) {
	panic("init buffer stack %d", err);
    }

    MPIPE_IPKT_THRESH_t ipkt_thr;
    ipkt_thr.word = mpipe_rd(mpipe, MPIPE_IPKT_THRESH);
    ipkt_thr.cutthrough = 80; // 80 * 128 = 10240 - store and forward even for maximum size packets - needed for classifier to know packet_size and make correct decision about which buffer stack to choose
    mpipe_wr(mpipe, MPIPE_IPKT_THRESH, ipkt_thr.word);
    ipkt_thr.word = mpipe_rd(mpipe, MPIPE_IPKT_THRESH);
    printk("tile mpipe%u: clsq_hwm:%u num_blocks:%u cutthrough:%u\n",
	    mpipe->instance, ipkt_thr.clsq_hwm,
	    ipkt_thr.num_blocks, ipkt_thr.cutthrough);

    MPIPE_EDMA_INFO_t edma_info;
    edma_info.word = mpipe_rd(mpipe, MPIPE_EDMA_INFO);
    mpipe->hw_tx_queue_count = edma_info.num_rings;

    MPIPE_LBL_INFO_t lbl_info;
    lbl_info.word = mpipe_rd(mpipe, MPIPE_LBL_INFO);
    mpipe->notif_ring_count = lbl_info.num_nr;
    mpipe->notif_group_count = lbl_info.num_groups;
    mpipe->bucket_count = lbl_info.num_buckets;
    printk("tile mpipe%u: hw_tx_queue_count:%u\n", mpipe->instance,
	mpipe->hw_tx_queue_count);
    printk("tile mpipe%u: notif_ring_count:%u\n", mpipe->instance,
	mpipe->notif_ring_count);
    printk("tile mpipe%u: notif_group_count:%u\n", mpipe->instance,
	mpipe->notif_group_count);
    printk("tile mpipe%u: bucket_count:%u\n", mpipe->instance,
	mpipe->bucket_count);
    printk("tile mpipe%u: 1g devs: %u, 10g devs: %u\n", mpipe->instance,
	    mpipe->dev_1g_count, mpipe->dev_10g_count);

    mpipe->buffers_current_approx = 0;
    for (i = 0; i < mpipe->buffers_min; ++i) {
	void *buf = skb_bin_alloc_buffer(GFP_KERNEL);
	if (!buf) {
	    break;
	}
	++mpipe->buffers_current_approx;
	__tile_push_buffer(mpipe, buf);
    }

    for (i = 0; i < NR_CPUS; ++i) {
	if (!state.cpu_info[i]) {
	    continue;
	}
	tile_mpipe_open_cpu(mpipe, state.cpu_info[i], i);
    }

    ++state.open_mpipes;
    return 0;
}

static void tile_tx_queue_cleanup(struct tile_tx_queue *q) {
    if (q->equeue_pages) {
	__free_pages(q->equeue_pages, q->equeue_order);
//	__homecache_free_pages(q->equeue_pages, q->equeue_order);
	q->equeue_pages = NULL;
    }
    q->edma = -1;
}

static void tile_mpipe_cleanup(struct tile_mpipe *mpipe) {
    unsigned i;

    if (!mpipe->ctx.mmio_cfg_base) {
	// already cleaned up
	return;
    }
    printk("tile mpipe%u: cleanup\n", mpipe->instance);

    for (i = 0; i < mpipe->iqueues_per_cpu; ++i) {
	free_irq(mpipe->ingress_irq[i], NULL);
	irq_free_hwirq(mpipe->ingress_irq[i]);
    }

    for (i = 0; i < NR_CPUS; ++i) {
	struct tile_cpu_info *cpu_info = state.cpu_info[i];
	if (!cpu_info) {
	    continue;
	}
	struct tile_cpu_mpipe_info *mpipe_info =
	    &cpu_info->mpipe_info[mpipe->instance];

	if (mpipe_info->iqueue_pages) {
	    __homecache_free_pages(
		mpipe_info->iqueue_pages, mpipe_info->iqueue_order);
	    mpipe_info->iqueue_pages = NULL;
	    mpipe_info->iqueue_order = 0;
	}

    }

    for (i = 0; i < TILE_DEVS_MAX; ++i) {
	if (!state.devs[i]) {
	    continue;
	}
	struct tile_priv *priv = netdev_priv(state.devs[i]);
	if (priv->mpipe != mpipe) {
	    continue;
	}
	tile_tx_queue_cleanup(&priv->tx_queue);
	tile_tx_queue_cleanup(&priv->tx_queue_fast);
    }

    iounmap((void __force __iomem *)(mpipe->ctx.mmio_fast_base));
    iounmap((void __force __iomem *)(mpipe->ctx.mmio_cfg_base));
    mpipe->ctx.mmio_fast_base = NULL;
    mpipe->ctx.mmio_cfg_base = NULL;

    hv_dev_close(mpipe->ctx.fd);

    if (mpipe->buffer_stack_pages) {
	__free_pages(mpipe->buffer_stack_pages, HUGETLB_PAGE_ORDER);
	mpipe->buffer_stack_pages = NULL;
    }
}

static void tile_mpipe_close_rx_cpu(struct tile_mpipe *mpipe,
	struct tile_cpu_info *cpu_info) {
    unsigned i;
    for (i = 0; i < mpipe->iqueues_per_cpu; ++i) {
	unsigned q = mpipe->instance * MPIPE_IQUEUES_PER_CPU_MAX + i;
	napi_disable(&cpu_info->rxq[q].napi);
	netif_napi_del(&cpu_info->rxq[q].napi);
    }
}

static void tile_mpipe_close(struct tile_mpipe *mpipe) {
    unsigned i;
    unsigned cpu;
    printk("tile mpipe%u: close\n", mpipe->instance);

    // disable interrupts
    for_each_cpu(cpu, &mpipe->rx_cpus_map) {
	struct tile_cpu_info *cpu_info = state.cpu_info[cpu];
	for (i = 0; i < mpipe->iqueues_per_cpu; ++i) {
	    int ring = cpu_info->rxq[i].iqueue.ring;
	    MPIPE_INT_BIND_t data = {{
		    .vec_sel = 3 + ring / 64,
		    .bind_sel = ring % 64,
		}};
	    mpipe_wr(mpipe, MPIPE_INT_BIND, data.word);
	}
    }

    tile_pop_buffers(mpipe);

    for_each_cpu(i, &mpipe->rx_cpus_map) {
	tile_mpipe_close_rx_cpu(mpipe, state.cpu_info[i]);
    }

    tile_mpipe_cleanup(mpipe);

    if (!--state.open_mpipes) {
	if (ports_config == ccr1072_1g_8splus_ports) {
	    printk("tile: put TI phys in reset\n");
	    access_latch(0, 0, BIT(3) | BIT(5) | BIT(7) | BIT(15));
	}
	if (ports_config == ccr1072_1g_8splus_r3_ports) {
	    printk("tile: put QT2025 phys in reset\n");
	    access_latch(0, 0,
		    BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(14) | BIT(15));
	}
	unregister_netdevice(state.dummy_netdev);
	state.dummy_netdev = NULL;
    }
}


int gxio_mpipe_rules_set_capacity(gxio_mpipe_rules_t* rules, uint16_t capacity) {
    gxio_mpipe_rules_list_t* list = &rules->list;

    gxio_mpipe_rules_rule_t* rule =
	(gxio_mpipe_rules_rule_t*)(list->rules + list->head);

    // Verify begun.
    if (list->tail == 0)
	return GXIO_MPIPE_ERR_RULES_EMPTY;

    rule->capacity = capacity;

    return 0;
}

static void tile_mpipe_update(struct tile_mpipe *mpipe) {
    int channel;

    static gxio_mpipe_rules_t rules;
    gxio_mpipe_rules_stacks_t stacks = {
	.stacks = {
	    0, 0, 0, 0, 0, 0, 0, 0,
	},
    };
    gxio_mpipe_rules_init(&rules, &mpipe->ctx);

    for (channel = 0; channel < TILE_CHANNELS_MAX; channel++) {
	struct net_device *dev = mpipe->active_devs_for_channel[channel];
	if (!dev || dev == state.dummy_netdev) {
	    continue;
	}
	struct tile_priv *priv = netdev_priv(dev);
	if (priv->mpipe != mpipe) {
	    continue;
	}

	gxio_mpipe_rules_begin(&rules,
		mpipe->first_bucket + BUCKETS_PER_DEV * channel,
		BUCKETS_PER_DEV, &stacks);
	gxio_mpipe_rules_set_headroom(&rules, NET_SKB_PAD + NET_IP_ALIGN);
	gxio_mpipe_rules_set_capacity(&rules, NET_SKB_PAD + NET_IP_ALIGN + ETH_HLEN + dev->l2mtu);
	gxio_mpipe_rules_add_channel(&rules, channel);
    }

    int ret  = gxio_mpipe_rules_commit(&rules);
    if (ret) {
	printk("tilegx: gxio_mpipe_rules_commit failed: %d\n", ret);
    }
}

static int tile_create_equeue(struct tile_priv *priv, struct tile_tx_queue *q,
	unsigned size, unsigned prio) {
    struct tile_mpipe *mpipe = priv->mpipe;
    unsigned i;
    int err;

    q->edma_ring_size = size * sizeof(gxio_mpipe_edesc_t);
    q->equeue_order = get_order(q->edma_ring_size);

    if (!q->equeue_pages) {
	q->equeue_pages = alloc_pages(GFP_KERNEL, q->equeue_order);
	if (!q->equeue_pages) {
	    printk("could not allocate edma ring\n");
	    return -1;
	}
    }

    void *mem = pfn_to_kaddr(page_to_pfn(q->equeue_pages));
    memset(mem, 0, q->edma_ring_size);

    for_each_online_cpu(i) {
	struct tile_cpu_dev_info *info;
	if (!state.cpu_info[i]) {
	    continue;
	}
	info = &state.cpu_info[i]->dev_info[priv->devno];
	info->tx_tail = 0;
	info->tx_head = 0;
	info->tx_timer_interval = TX_TIMER_INTERVAL_MIN;
	info->completed = 0;
    }

    if (q->edma < 0) {
	q->edma = gxio_mpipe_alloc_edma_rings(&mpipe->ctx, 1, 0, 0);
    }
    if (q->edma < 0) {
	return -1;
    }

    err = gxio_mpipe_equeue_init(&q->equeue, &mpipe->ctx,
	    q->edma, priv->link.channel, mem, q->edma_ring_size, 0);
    if (err) {
	printk("gxio_mpipe_equeue_init failed:%d\n", err);
	return -1;
    }
    atomic64_set(&q->slot, 0);
    q->queued_bits = 0;

    MPIPE_EDMA_RG_INIT_CTL_t edma_ctl = {{ 0 }};
    MPIPE_EDMA_RG_INIT_DAT_REQ_THR_t edma_req_thr = {{ 0 }};
    MPIPE_EDMA_RG_INIT_DAT_THRESH_t edma_thr = {{ 0 }};
    MPIPE_EDMA_RG_INIT_DAT_MAP_t edma_map = {{ 0 }};
//    unsigned long efifo_prot = 0;

    edma_ctl.idx = q->edma;

    edma_ctl.struct_sel = 1;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
    edma_req_thr.word = mpipe_rd(mpipe, MPIPE_EDMA_RG_INIT_DAT);
//    edma_req_thr.req_thr = 15;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_DAT, edma_req_thr.word);

    edma_ctl.struct_sel = 2;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
    edma_thr.word = mpipe_rd(mpipe, MPIPE_EDMA_RG_INIT_DAT);
//    edma_thr.high_bw = 1;
//    edma_thr.nt_ovd = 3;
//    edma_thr.db = 0;
//    edma_thr.min_snf_blks = 4;
//    edma_thr.max_blks = 65;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_DAT, edma_thr.word);

    edma_ctl.struct_sel = 3;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
    edma_map.word = mpipe_rd(mpipe, MPIPE_EDMA_RG_INIT_DAT);
//    edma_map.priority_queues = 0;
    edma_map.priority_lvl = prio;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_DAT, edma_map.word);


    edma_ctl.struct_sel = 1;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
    edma_req_thr.word = mpipe_rd(mpipe, MPIPE_EDMA_RG_INIT_DAT);

    edma_ctl.struct_sel = 2;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
    edma_thr.word = mpipe_rd(mpipe, MPIPE_EDMA_RG_INIT_DAT);

    edma_ctl.struct_sel = 3;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
    edma_map.word = mpipe_rd(mpipe, MPIPE_EDMA_RG_INIT_DAT);

    edma_ctl.struct_sel = 4;
    mpipe_wr(mpipe, MPIPE_EDMA_RG_INIT_CTL, edma_ctl.word);
//    efifo_prot = mpipe_rd(mpipe, MPIPE_EDMA_RG_INIT_DAT);

/*
    printk("%s: edma:%d size:%u order:%u req_thr:%u high_bw:%u nt_ovd:%u db:%u min_snf_blks:%u max_blks:%u priority_queues:%x priority_lvl:%u channel:%u efifo_prot:%lx\n",
	    priv->sw.dev->name,
	    q->edma, size,
	    q->equeue_order,
	    edma_req_thr.req_thr,
	    edma_thr.high_bw, edma_thr.nt_ovd, edma_thr.db,
	    edma_thr.min_snf_blks, edma_thr.max_blks,
	    edma_map.priority_queues, edma_map.priority_lvl, edma_map.channel,
	    efifo_prot);
*/
    return 0;
}

static void tile_mac_open(struct tile_priv *priv) {
    if (!priv->xaui) {
	MPIPE_GBE_MAC_INTFC_CTL_t intfcctl = {
	    .word = tile_rd(priv, MPIPE_GBE_MAC_INTFC_CTL)
	};
	intfcctl.tx_prq_ena = 0xffff;
	intfcctl.rx_rst_mode = 0;
	intfcctl.prq_ovd = 1;
	intfcctl.prq_ovd_val = 0;
	intfcctl.speed_val = 0;
	intfcctl.speed_sel = 1;
	intfcctl.pause_mode = 0;
	intfcctl.ins_rx_sts = 0;
	intfcctl.no_tx_crc = 0;
	tile_wr(priv, MPIPE_GBE_MAC_INTFC_CTL, intfcctl.word);

	MPIPE_GBE_NETWORK_CONFIGURATION_t config = {
	    .word = 0
	};
	MPIPE_GBE_PCS_CTL_t pcs_ctl = {
	    .word = 0
	};
	if (priv->config->sfpplus) {
	    config.sgmii_mode = 0;
	}
	else if (priv->config->sfp) {
	    config.sgmii_mode = 0;
	    if (priv->config->combo) {
		config.sgmii_mode = 1;
	    }
	    pcs_ctl.auto_neg = 1;
	    pcs_ctl.restart_neg = 1;
	}
	else {
	    config.sgmii_mode = 1;
	    pcs_ctl.auto_neg = 1;
	    pcs_ctl.restart_neg = 1;
	}
	config.jumbo_ena = 1;
	config.rcv_1536 = 1;
//	config.div = 4; // mdio speed 125Mhz / 64 = 1.95Mhz - useless to do this as there is only one mac that is used for mdio acces and that mac might not be used as etherent interface at all
	config.div = 3;
	config.uni_dir = 1;
	config.pcs_sel = 1;
	config.copy_all = 1;
	config.fcs_remove = 1;
	config.dis_pause_cpy = 1;
	config.gige_mode = 1;
	config.full_duplex = 1;
	tile_wr(priv, MPIPE_GBE_NETWORK_CONFIGURATION, config.word);
	priv->sgmii = config.sgmii_mode;

	tile_wr(priv, MPIPE_GBE_PCS_CTL, pcs_ctl.word);

	if (priv->sw.ops) {
	    tile_set_network_configuration(
		priv, SPEED_1000, DUPLEX_FULL, true, true);
	}
    }
    else {
	tile_set_tx_props(priv, SPEED_10000);

	MPIPE_XAUI_MAC_INTFC_TX_CTL_t tx_ctl = {
	    .word = tile_rd(priv, MPIPE_XAUI_MAC_INTFC_TX_CTL)
	};
	tx_ctl.tx_drop = 0;
	tile_wr(priv, MPIPE_XAUI_MAC_INTFC_TX_CTL, tx_ctl.word);

	MPIPE_XAUI_RECEIVE_CONFIGURATION_t config = {
	    .word = tile_rd(priv, MPIPE_XAUI_RECEIVE_CONFIGURATION)
	};
	config.loc_fault = 0;
	config.accept_jumbo = 1;
	config.accept_1536 = 1;
	tile_wr(priv, MPIPE_XAUI_RECEIVE_CONFIGURATION, config.word);

	MPIPE_XAUI_TRANSMIT_CONFIGURATION_t tx_config = {
	    .word = tile_rd(priv, MPIPE_XAUI_TRANSMIT_CONFIGURATION)
	};
	tx_config.uni_mode = 0; // enable local-remote fault forwarding
//	tx_config.cfg_speed = 2;
	tile_wr(priv, MPIPE_XAUI_TRANSMIT_CONFIGURATION, tx_config.word);

	// local mac serdes loopback
//	tile_serdes_write_all_lanes(priv, MPIPE_SERDES_PRBS_CTRL, 0x2);
    }
    if (priv->config->gpio_sfp_tx_disable) {
	tile_gpio_set(priv->config->gpio_sfp_tx_disable, false);
    }
}

static void tile_hv_open(struct tile_priv *priv) {
    int err;
    unsigned i;
    struct tile_mpipe *mpipe = priv->mpipe;

    if (priv->xaui) {
	err = gxio_mpipe_link_open(&priv->link, &priv->mpipe->ctx,
		priv->config->hv_name_xaui, GXIO_MPIPE_LINK_AUTO_UPDOWN);
    }
    else {
	err = gxio_mpipe_link_open(&priv->link, &priv->mpipe->ctx,
		priv->config->hv_name, GXIO_MPIPE_LINK_AUTO_UPDOWN);
    }
    if (err) {
	printk("%s: open err:%d channel:%u mac:%u context:%p\n",
		priv->sw.dev->name, err, priv->link.channel, priv->link.mac,
		priv->link.context);
    }

    priv->equeue_size = RECOMMENDED_PER_CPU_TX_QUEUE_SIZE * num_online_cpus();
    if (priv->equeue_size > 65536) {
	BUG();
    }
    else if (priv->equeue_size > 8192) {
	priv->equeue_size = 65536;
    }
    else if (priv->equeue_size > 2048) {
	priv->equeue_size = 8192;
    }
    else if (priv->equeue_size > 512) {
	priv->equeue_size = 2048;
    }
    else {
	priv->equeue_size = 512;
    }

    priv->per_cpu_tx_queue_size = priv->equeue_size / num_online_cpus();
    if (priv->per_cpu_tx_queue_size > MAX_PER_CPU_TX_QUEUE_SIZE) {
	priv->per_cpu_tx_queue_size = MAX_PER_CPU_TX_QUEUE_SIZE;
    }

    printk("%s: equeue_size:%u per_cpu_tx_queue_size:%u (max:%u)\n",
	    priv->sw.dev->name, priv->equeue_size, priv->per_cpu_tx_queue_size,
	    num_online_cpus() * priv->per_cpu_tx_queue_size);
    priv->tx_entries = __alloc_percpu(
	sizeof(struct tile_tx_entry) * priv->per_cpu_tx_queue_size,
	__alignof__(struct tile_tx_entry));
    if (!priv->tx_entries) {
	printk("%s: failed to alloc_percpu tx_entries\n", priv->sw.dev->name);
	return;
    }

    for_each_online_cpu(i) {
	state.cpu_info[i]->dev_info[priv->devno].tx_entries = per_cpu_ptr(priv->tx_entries, i);
    }

    if (tile_create_equeue(priv, &priv->tx_queue, priv->equeue_size, 1) < 0) {
	netdev_err(priv->sw.dev, "Failed to create egress queue");
	return;
    }
    if ((mpipe->dev_10g_count + mpipe->dev_1g_count) * 2 <= mpipe->hw_tx_queue_count &&
	    !priv->sw.ops) {
	if (tile_create_equeue(priv, &priv->tx_queue_fast, FAST_PATH_TX_QUEUE_SIZE, 0) < 0) {
	    netdev_err(priv->sw.dev, "Failed to create egress fast path queue");
	    return;
	}
    }

    priv->mpipe->devs_for_channel[priv->link.channel] = priv->sw.dev;
    priv->mpipe->active_devs_for_channel[priv->link.channel] = priv->sw.dev;
    tile_mpipe_update(priv->mpipe);
}

static void tile_tx_stop_all_queues(struct net_device *dev) {
    unsigned i;
    struct tile_priv *priv = netdev_priv(dev);

    if (priv->sw.ops) {
	switch_stop(&priv->sw);
    }
    for (i = 0; i < dev->num_tx_queues; i++) {
	struct netdev_queue *txq = netdev_get_tx_queue(dev, i);
	set_bit(__QUEUE_STATE_STACK_XOFF, &txq->state);
	txq->trans_start = jiffies + HZ;
    }
}

static void tile_tx_start_all_queues(struct net_device *dev) {
    unsigned i;
    struct tile_priv *priv = netdev_priv(dev);

    for (i = 0; i < dev->num_tx_queues; i++) {
	struct netdev_queue *txq = netdev_get_tx_queue(dev, i);
	clear_bit(__QUEUE_STATE_STACK_XOFF, &txq->state);
    }
    netif_tx_start_all_queues(dev);
    if (priv->sw.ops) {
	switch_start(&priv->sw);
    }
}

static void tile_hv_close(struct tile_priv *priv) {
    unsigned i;

    priv->mpipe->active_devs_for_channel[priv->link.channel] = state.dummy_netdev;
    tile_mpipe_update(priv->mpipe);
    gxio_mpipe_link_close(&priv->link);
    memset(&priv->link, 0, sizeof(priv->link));

    free_percpu(priv->tx_entries);
    priv->tx_entries = NULL;
    for_each_online_cpu(i) {
	state.cpu_info[i]->dev_info[priv->devno].tx_entries = NULL;
    }
}

static void tile_tx_close_queue(struct tile_priv *priv,
	struct tile_tx_queue *q) {
    if (q->edma < 0) {
	return;
    }
    int res = gxio_mpipe_init_edma_ring_aux(
	&priv->mpipe->ctx, pfn_to_kaddr(page_to_pfn(q->equeue_pages)),
	q->edma_ring_size, 0, q->edma, -1u);
    if (res < 0) {
	printk("%s: cleanup ering failed: %d\n", priv->sw.dev->name, res);
    }
}

static void tile_tx_close(struct tile_priv *priv) {
    unsigned i;
    for (i = 0; i < NR_CPUS; ++i) {
	if (!state.cpu_info[i]) {
	    continue;
	}
	cancel_work_sync(&state.cpu_info[i]->dev_info[priv->devno].tx_work);
	hrtimer_cancel(&state.cpu_info[i]->dev_info[priv->devno].tx_timer);
    }

    tile_tx_close_queue(priv, &priv->tx_queue);
    tile_tx_close_queue(priv, &priv->tx_queue_fast);

    for (i = 0; i < NR_CPUS; ++i) {
	struct tile_tx_entry *tx_entries = per_cpu_ptr(priv->tx_entries, i);
	struct tile_cpu_dev_info *cpu_dev_info;
	unsigned k;
	if (!state.cpu_info[i]) {
	    continue;
	}
	cpu_dev_info = &state.cpu_info[i]->dev_info[priv->devno];

	for (k = 0; k < priv->per_cpu_tx_queue_size; ++k) {
	    struct tile_tx_entry *entry = &cpu_dev_info->tx_entries[k];
	    if (entry->buf) {
		dev_kfree_skb(entry->buf);
	    }
	    memset(entry, 0, sizeof(struct tile_tx_entry));
	    tx_entries[k].buf = NULL;
	}
    }
}

static void tile_do_close(struct tile_priv *priv, bool close_phy) {
    printk("%s: do close phy:%u\n", priv->sw.dev->name, close_phy);
    priv->closing = true;
    __insn_mf();
    tile_tx_stop_all_queues(priv->sw.dev);
    tile_tx_close(priv);
    if (priv->config->gpio_sfp_tx_disable) {
	tile_gpio_set(priv->config->gpio_sfp_tx_disable, true);
    }
    if (close_phy) {
	if (priv->sw.phy.inited) {
	    phy_disable(&priv->sw.phy);
	}
    }
    tile_update_stats(priv);
    tile_hv_close(priv);
    priv->closing = false;
}

static void tile_do_open(struct tile_priv *priv, bool open_phy) {
    printk("%s: do open phy:%u\n", priv->sw.dev->name, open_phy);
    tile_hv_open(priv);
    tile_mac_open(priv);
    if (open_phy) {
	priv->rx_lose_down_jiffies = 0;
	priv->autoneg_start_jiffies = 0;
	if (priv->sw.phy.inited) {
	    phy_enable(&priv->sw.phy);
	}
    }
    tile_tx_start_all_queues(priv->sw.dev);
}

static int tile_open(struct net_device *dev) {
    struct tile_priv *priv = netdev_priv(dev);
    struct tile_mpipe *mpipe = priv->mpipe;

    skb_bin_request(dev, NET_IP_ALIGN + dev->l2mtu + BUFFER_CACHE_OVERHEAD);

    priv->last_open_jiffies = jiffies;
    if (!mpipe->open_devs) {
	if (tile_mpipe_open(mpipe)) {
	    printk("tile_mpipe_open failed\n");
	    return -EIO;
	}
    }
    ++mpipe->open_devs;

    priv->xaui = false;
    if (priv->config->sfpplus) {
	priv->xaui = true;
	if (priv->config->sfp &&
		priv->sw.phy.req.cmd.base.cmd &&
		priv->sw.phy.req.cmd.base.autoneg == AUTONEG_DISABLE &&
		priv->sw.phy.req.cmd.base.speed != SPEED_10000) {
	    priv->xaui = false;
	}
    }
    printk("%s: open %s\n", dev->name, priv->xaui ? "xgbe" : "gbe");
    tile_do_open(priv, true);
    return 0;
}

static int tile_close(struct net_device *dev) {
    struct tile_priv *priv = netdev_priv(dev);
//    printk("%s: close\n", dev->name);
    tile_do_close(priv, true);

    if (!--priv->mpipe->open_devs) {
	tile_mpipe_close(priv->mpipe);
    }
//    printk("%s: close out\n", dev->name);

    skb_bin_release(dev);
    return 0;
}

static void tile_switch_xaui(struct tile_priv *priv, bool need_xaui) {
    if (!priv->config->hv_name) {
	return;
    }
    if (need_xaui != priv->xaui) {
	printk("%s: switch hv dev to xaui:%u\n", priv->sw.dev->name, need_xaui);
	tile_do_close(priv, true);
	priv->xaui = need_xaui;
	tile_do_open(priv, true);
    }
    if (priv->sw.phy.curr.link) {
	tile_set_network_configuration(priv,
		priv->sw.phy.curr.cmd.base.speed,
		priv->sw.phy.curr.cmd.base.duplex,
		priv->sw.phy.curr.pause.rx_pause,
		priv->sw.phy.curr.pause.tx_pause);
    }
}

static void tile_link_changed(struct phy *p) {
    struct tile_priv *priv = netdev_priv(p->mii_info.dev);
//    printk("%s: on link changed has_module:%d link:%d, speed:%d, duplex:%d (requested an:%u %u/%u)\n",
//	    priv->sw.dev->name, phy_sfp_has_module(&p->curr), p->curr.link, p->curr.cmd.speed, p->curr.cmd.duplex,
//	    p->req.cmd.autoneg, p->req.cmd.speed,
//	    p->req.cmd.duplex);

    if (p->ops == &tile_qt2025_ops && p->curr.link) {
	tile_switch_xaui(priv, p->curr.cmd.base.speed == SPEED_10000);
    }
    if (p->ops == &tile_ti_ops && phy_sfp_has_module(&p->curr) &&
	    p->req.cmd.base.autoneg == AUTONEG_ENABLE) {
	if (!p->eeprom_valid) {
	    phy_read_eeprom(p, NULL, NULL);
	}
//	printk("eeprom valid:%u bitrate:%u\n",
//		p->eeprom_valid, p->eeprom[12]);
	if (p->eeprom_valid) {
	    unsigned bitrate_nominal = p->eeprom[12];
	    tile_switch_xaui(priv, bitrate_nominal >= 100);
	}
    }
    if (priv->sw.phy.curr.link) {
	tile_set_network_configuration(priv,
		priv->sw.phy.curr.cmd.base.speed,
		priv->sw.phy.curr.cmd.base.duplex,
		priv->sw.phy.curr.pause.rx_pause,
		priv->sw.phy.curr.pause.tx_pause);
    }

    if (!priv->xaui) {
	tile_set_sgmii(priv);
    }
}

static void tile_hw_xmit_prepare(struct tile_mpipe *mpipe,
	struct tile_cpu_mpipe_info *mpipe_info,
	struct tile_cpu_dev_info *cpu_dev_info,
	gxio_mpipe_edesc_t *edesc) {
    ++mpipe_info->adjust_buffers_counters;
    if (mpipe_info->adjust_buffers_counters > ADJUST_BUFFERS_BATCH) {
	tile_mpipe_adjust_buffers(mpipe);
    }
    edesc->hwb = 1;
    edesc->size = state.mpipe_buffer_size_enum;
#ifdef STATS
    ++cpu_dev_info->stats.stats_tx_hw;
#endif
}

static void tile_xmit_finish(struct net_device *dev,
	struct tile_cpu_dev_info *cpu_dev_info,
	gxio_mpipe_edesc_t edesc, struct tile_tx_entry ent) {
    struct tile_priv *priv = netdev_priv(dev);
    struct tile_tx_queue *tx_queue = &priv->tx_queue;

#ifdef DEBUG
    u64 x = atomic64_inc_return(raw_cpu_ptr(&per_cpu_info.xmit_count));
    if (x > 1) {
	printk("%s: %u %u xmit recursion: %llu\n",
		dev->name, cpu_dev_info->cpu, smp_processor_id(), x);
	WARN_ON(1);
    }
    if (smp_processor_id() != cpu_dev_info->cpu) {
	printk("%s: xmit_finish actual cpu:%u required cpu:%u",
		dev->name, smp_processor_id(), cpu_dev_info->cpu);
	WARN_ON(1);
    }
    if (__netif_subqueue_stopped(dev, cpu_dev_info->cpu)) {
	printk("%s: xmit when queue stopped cpu:%u",
		dev->name, cpu_dev_info->cpu);
	WARN_ON(1);
    }
#endif

    ent.bits = (edesc.xfer_size + 24) * 8;

    // mf could be somewhere else as its purpose is to be sure that writes to
    // packet buffer land before actual transmit
    __insn_mf();
    u64 slot = __insn_fetchadd(&tx_queue->slot, 1);
//    u64 slot = atomic64_add_return(1, &tx_queue->slot) - 1;
    unsigned queued_bits = __insn_fetchadd4(&tx_queue->queued_bits, ent.bits) + ent.bits;
    edesc.gen = !((slot >> tx_queue->equeue.log2_num_entries) & 1);

    ent.slot = slot;
    cpu_dev_info->tx_entries[cpu_dev_info->tx_head] = ent;

    cpu_dev_info->tx_head = tile_next_tx_slot(priv, cpu_dev_info->tx_head);
    unsigned next_tx_head = tile_next_tx_slot(priv, cpu_dev_info->tx_head);
    if (priv->sw.ops || next_tx_head == cpu_dev_info->tx_tail ||
	    queued_bits >= tx_queue->max_queued_bits ||
	    (cycles_get_cycles() - cpu_dev_info->last_tx_time) > tx_queue->max_cycles_between_tx) {
	unsigned qb = tile_tx(priv, cpu_dev_info);
	if (qb != -1u) {
	    queued_bits = qb;
	}
    }

    gxio_mpipe_edesc_t* edesc_p =
	&tx_queue->equeue.edescs[slot & tx_queue->equeue.mask_num_entries];
    __gxio_mmio_write64(&edesc_p->words[1], edesc.words[1]);
    __gxio_mmio_write64(&edesc_p->words[0], edesc.words[0]);

    bool stop = next_tx_head == cpu_dev_info->tx_tail ||
	queued_bits >= tx_queue->max_queued_bits;
    if (stop) {
#ifdef STATS
	cpu_dev_info->stats.stats_tx_queue_stop++;
#endif
	if (priv->sw.ops) {
	    switch_stop_all_port_queue(&priv->sw, cpu_dev_info->cpu);
	}
	netif_stop_subqueue(dev, cpu_dev_info->cpu);
    }
    if (priv->sw.ops || stop) {
	schedule_work_on(cpu_dev_info->cpu, &cpu_dev_info->tx_work);
    }
#ifdef DEBUG
    atomic64_dec(raw_cpu_ptr(&per_cpu_info.xmit_count));
    if (priv->sw.ops) {
	printk("%s: priv->sw.ops:%p\n", dev->name, priv->sw.ops);
    }
#endif
}

static int tile_fast_path_xmit_old(struct net_device *dev, struct fp_buf *fpb) {
    struct tile_priv *priv = netdev_priv(dev);
    struct tile_mpipe *mpipe = priv->mpipe;
    struct tile_cpu_info *cpu_info = raw_cpu_ptr(&per_cpu_info);
    struct tile_cpu_dev_info *info = &cpu_info->dev_info[priv->devno];
    struct tile_cpu_mpipe_info *mpipe_info =
	&cpu_info->mpipe_info[priv->mpipe->instance];
    gxio_mpipe_edesc_t edesc = { { 0 } };

    if (fpb_headroom(fpb) >= BUFFER_CACHE_OVERHEAD) {
	if (!__fpb_realloc(fpb)) {
	    return 0;
	}
    }

    fast_path_fp_tx_inc(dev, fpb);
    edesc.bound = 1;
    edesc.xfer_size = fpb_len(fpb);
    edesc.va = (ulong)__pa(fpb_data(fpb));

/*
    printk("%s: fp xmit in_irq:%d irqs_disabled:%d cpu:%u buf:%p len:%d\n",
	    dev->name, (int)in_irq(), irqs_disabled(),
	    smp_processor_id(), fpb_buf(fpb), fpb_len(fpb));
*/
    kmemleak_ignore(fpb_buf(fpb));
    tile_hw_xmit_prepare(mpipe, mpipe_info, info, &edesc);

    struct tile_tx_entry entry = { 0 };

    tile_xmit_finish(dev, info, edesc, entry);
#ifdef STATS
    info->stats.stats_xmit_fast++;
#endif
    return 1;
}

static int tile_xmit_commit(struct net_device *dev) {
    struct tile_priv *priv = netdev_priv(dev);
    struct tile_cpu_info *cpu_info = raw_cpu_ptr(&per_cpu_info);
    struct tile_cpu_dev_info *info = &cpu_info->dev_info[priv->devno];
    unsigned i;
    struct tile_mpipe *mpipe = priv->mpipe;
    struct tile_cpu_mpipe_info *mpipe_info =
	&cpu_info->mpipe_info[mpipe->instance];
    struct tile_tx_queue *tx_queue = &priv->tx_queue_fast;

    info->xmit_commit_scheduled = false;
    int count = info->xmit_buf_count;
    if (!count) {
	return 0;
    }
    info->xmit_buf_count = 0;
    u64 cslot = atomic64_read(&tx_queue->slot);
    if (cslot - info->completed >= priv->max_fast_queue_fill) {
	info->completed = tile_update_tx_completed(tx_queue);
	if (cslot - info->completed >= priv->max_fast_queue_fill) {
	    for (i = 0; i < count; ++i) {
		struct fp_buf *fpb = &info->xmit_bufs[i];
		__tile_push_buffer(mpipe, fpb_buf(fpb));
	    }
	    mpipe_info->adjust_buffers_counters += count;
#ifdef STATS
	    info->stats.stats_xmit_fast_drop += count;
#endif
	    tile_mpipe_adjust_buffers(mpipe);
	    return -1;
	}
    }

    __insn_mf();
    u64 slot = __insn_fetchadd(&tx_queue->slot, count);

    struct fp_dev_pcpu_stats *s = this_cpu_ptr(dev->fp.stats);
    gxio_mpipe_edesc_t edesc = { { 0 } };
    edesc.bound = 1;
    edesc.hwb = 1;
    edesc.size = state.mpipe_buffer_size_enum;
    for (i = 0; i < count; ++i, ++slot) {
	struct fp_buf *fpb = &info->xmit_bufs[i];
	s->fp_tx_byte += fpb_len(fpb);
	edesc.xfer_size = fpb_len(fpb);
	edesc.va = (ulong)__pa(fpb_data(fpb));
	edesc.gen = !((slot >> FAST_PATH_TX_QUEUE_LOG2) & 1);
#ifdef STATS
	++info->stats.stats_tx_hw;
#endif

	gxio_mpipe_edesc_t* edesc_p =
	    &tx_queue->equeue.edescs[slot & FAST_PATH_TX_QUEUE_MASK];
	__gxio_mmio_write64(&edesc_p->words[1], edesc.words[1]);
	__gxio_mmio_write64(&edesc_p->words[0], edesc.words[0]);
    }
    mpipe_info->adjust_buffers_counters += count;
#ifdef STATS
    info->stats.stats_xmit_fast += count;
#endif
    s->fp_tx_packet += count;
    tile_mpipe_adjust_buffers(mpipe);
    return count;
}

static int tile_fast_path_xmit(struct net_device *dev, struct fp_buf *fpb) {
    struct tile_priv *priv = netdev_priv(dev);
    struct tile_cpu_info *cpu_info = raw_cpu_ptr(&per_cpu_info);
    struct tile_cpu_dev_info *info = &cpu_info->dev_info[priv->devno];

    if (fpb_headroom(fpb) >= BUFFER_CACHE_OVERHEAD) {
	if (!__fpb_realloc(fpb)) {
	    return 0;
	}
    }

    if (!info->xmit_commit_scheduled) {
	schedule_xmit_commit(dev);
	info->xmit_commit_scheduled = true;
    }
    info->xmit_bufs[info->xmit_buf_count] = *fpb;
    ++info->xmit_buf_count;
    if (info->xmit_buf_count != XMIT_BATCH) {
	return 0;
    }
    int ret = tile_xmit_commit(dev);
    info->xmit_commit_scheduled = true;
    return ret;
}

static void tile_reopen(void) {
    u64 open_devs_mask = 0;
    int i;
    printk("tile: complete reopen\n");
    for (i = 0; i < TILE_DEVS_MAX; ++i) {
	if (!state.devs[i]) {
	    continue;
	}
	if (!netif_running(state.devs[i])) {
	    continue;
	}
	open_devs_mask |= BIT(i);
	dev_close(state.devs[i]);
    }
    for (i = 0; i < NR_MPIPE_MAX; ++i) {
	tile_mpipe_cleanup(&state.mpipe[i]);
    }
    for (i = 0; i < TILE_DEVS_MAX; ++i) {
	if (!(open_devs_mask & BIT(i))) {
	    continue;
	}
	dev_open(state.devs[i], NULL);
    }
}

#define TOO_FREQUENT_RESET_INTERVAL (10 * HZ)
static unsigned tile_get_timeouted_queue(struct net_device *dev) {
    unsigned i;
    for (i = 0; i < dev->num_tx_queues; i++) {
	struct netdev_queue *txq = netdev_get_tx_queue(dev, i);
	if (netif_xmit_stopped(txq) &&
		time_after(jiffies, (txq->trans_start + dev->watchdog_timeo))) {
	    return i;
	}
    }
    return 0;
}

static void tile_reset_task(struct work_struct *work) {
    struct tile_priv *priv = container_of(work, struct tile_priv, sw.restart_task);
    struct net_device *dev = priv->sw.dev;

    printk("%s: reset_task (last open @%d)\n",
	    dev->name, (int)priv->last_open_jiffies);
    rtnl_lock();

    if (netif_running(dev)) {
	local_bh_disable();
	tile_mpipe_dump(priv->mpipe);
	tile_mac_dump(dev);
	phy_dump(&priv->sw.phy);
	tile_tx_queue_dump(
	    dev, &priv->tx_queue, tile_get_timeouted_queue(dev), 0);
	local_bh_enable();
    }

//    if (time_after(priv->last_open_jiffies + TOO_FREQUENT_RESET_INTERVAL, jiffies)) {
    if (1) {
	tile_reopen();
    }
    else {
	if (netif_running(dev)) {
	    printk("%s: fast reopen\n", dev->name);
	    // do close/open without phy reset

	    tile_do_close(priv, false);
	    priv->last_open_jiffies = jiffies;
	    tile_do_open(priv, false);
	}
    }
    rtnl_unlock();

    netif_tx_schedule_all(dev);
}

static int tile_ioctl(struct net_device *dev, struct ifreq *rq, int cmd) {
    struct tile_priv *priv = netdev_priv(dev);
    if (priv->sw.phy.inited && priv->sw.phy.mii_info.mdio_read) {
	return generic_mii_ioctl(&priv->sw.phy.mii_info, if_mii(rq), cmd, NULL);
    }
    if (priv->sw.ops && !switch_ioctl(dev, rq, cmd)) {
	return 0;
    }
    return -EINVAL;
}

static void tile_prepare_cpu(void *unused) {
    state.cpu_info[smp_processor_id()] = raw_cpu_ptr(&per_cpu_info);
#ifdef DEBUG
    atomic64_set(raw_cpu_ptr(&per_cpu_info.xmit_count), 0);
#endif
}

static void tile_led_set(struct led_classdev *classdev,
	enum led_brightness brightness) {
    struct phy_led *led = container_of(classdev, struct phy_led, classdev);
    struct tile_priv *priv = container_of(led, struct tile_priv, led);
//    printk("%s: set brightness %u\n", led_act->name, brightness);
    led->brightness = brightness;

    if (brightness != -1) {
	tile_gpio_set(priv->config->gpio_sfp_led,
		!!brightness ^ priv->config->gpio_sfp_led_invert);
    }
}

static void tile_led2_set(struct led_classdev *classdev,
	enum led_brightness brightness) {
    struct phy_led *led = container_of(classdev, struct phy_led, classdev);
    struct tile_priv *priv = container_of(led, struct tile_priv, led2);
//    printk("%s: set brightness %u\n", led_act->name, brightness);
    led->brightness = brightness;

    if (brightness != -1) {
	tile_gpio_set(priv->config->gpio_sfp_led2,
		!!brightness ^ priv->config->gpio_sfp_led2_invert);
    }
}

static struct net_device_ops tile_ops = {
    .ndo_open = tile_open,
    .ndo_stop = tile_close,
    .ndo_select_queue = switch_select_queue,
    .ndo_start_xmit = fast_path_start_xmit,
    .ndo_tx_timeout = switch_tx_timeout,
    .ndo_fast_path_xmit = tile_fast_path_xmit_old,
    .ndo_xmit_commit = tile_xmit_commit,
    .ndo_do_ioctl = tile_ioctl,
    .ndo_get_stats64 = fast_path_get_stats64,
    .ndo_change_mtu = switch_change_mtu,
    .ndo_change_l2mtu = switch_change_l2mtu,
    .ndo_set_mac_address = switch_set_mac_address,
};

static struct net_device_ops tile_ops_fast = {
    .ndo_open = tile_open,
    .ndo_stop = tile_close,
    .ndo_select_queue = switch_select_queue,
    .ndo_start_xmit = fast_path_start_xmit,
    .ndo_tx_timeout = switch_tx_timeout,
    .ndo_fast_path_xmit = tile_fast_path_xmit,
    .ndo_xmit_commit = tile_xmit_commit,
    .ndo_do_ioctl = tile_ioctl,
    .ndo_get_stats64 = fast_path_get_stats64,
    .ndo_change_mtu = switch_change_mtu,
    .ndo_change_l2mtu = switch_change_l2mtu,
    .ndo_set_mac_address = switch_set_mac_address,
};

static void tile_qt2025_set_settings(struct phy *p, struct phy_settings *ps) {
    struct tile_priv *priv = netdev_priv(p->mii_info.dev);
//    printk("%s tile qt2025: set settings autoneg:%u speed:%u duplex:%u\n",
//	    priv->sw.dev->name, ps->cmd.autoneg, ps->cmd.speed, ps->cmd.duplex);
    if (ps->cmd.base.autoneg == AUTONEG_DISABLE) {
	tile_switch_xaui(priv, ps->cmd.base.speed == SPEED_10000);
    }

    tile_sfp_set_settings(p, ps);

    CALL_PHY_OP(&phy_amcc_qt2025_ops, set_settings, p, ps);
}

static void tile_qt2025_get_settings(struct phy *p, struct phy_settings *ps) {
    struct tile_priv *priv = netdev_priv(p->mii_info.dev);

    CALL_PHY_OP(&phy_amcc_qt2025_ops, get_settings, p, ps);

    if (!priv->xaui && priv->sgmii) {
	struct phy_settings x;
	memset(&x, 0, sizeof(x));
	tile_sfp_get_settings(p, &x);
	ps->link &= x.link;
	ps->cmd.base.speed = x.cmd.base.speed;
	ps->cmd.base.duplex = DUPLEX_FULL;
    }
}

static struct phy_ops tile_qt2025_ops = {
    .default_ops = &phy_amcc_qt2025_ops,
    .set_settings = tile_qt2025_set_settings,
    .get_settings = tile_qt2025_get_settings,
};

static void tile_tlk10232_set_settings(struct phy *p, struct phy_settings *ps) {
    struct tile_priv *priv = netdev_priv(p->mii_info.dev);
//    printk("%s tile tlk10232: set settings autoneg:%u speed:%u duplex:%u\n",
//	    priv->sw.dev->name, ps->cmd.autoneg, ps->cmd.speed, ps->cmd.duplex);
    if (ps->cmd.base.autoneg == AUTONEG_DISABLE) {
	tile_switch_xaui(priv, ps->cmd.base.speed == SPEED_10000);
    }

    tile_sfp_set_settings(p, ps);

    ps->cmd.base.autoneg = AUTONEG_DISABLE;
    ps->cmd.base.speed = priv->xaui ? SPEED_10000 : SPEED_1000;
    ps->cmd.base.duplex = DUPLEX_FULL;

    CALL_PHY_OP(&phy_ti_tlk10232_ops, set_settings, p, ps);
}

static void tile_tlk10232_get_settings(struct phy *p, struct phy_settings *ps) {
    struct phy_settings x;
    tile_sfp_get_settings(p, ps);

    CALL_PHY_OP(&phy_ti_tlk10232_ops, get_settings, p, &x);

    ps->link &= x.link;
}

static struct phy_ops tile_ti_ops = {
    .default_ops = &phy_ti_tlk10232_ops,
    .set_settings = tile_tlk10232_set_settings,
    .get_settings = tile_tlk10232_get_settings,
    .read_eeprom = tile_sfp_read_eeprom,
    .write_eeprom = tile_sfp_write_eeprom,
};

static void tile_dev_init(struct tile_mpipe *mpipe,
	struct tile_port_config *config, uint8_t* mac) {
    int ret;
    int devno = 0;
    struct net_device *dev;
    struct tile_priv *priv;
    struct eth_stats *xs;

    while (state.devs[devno] != NULL) {
	devno++;
    }

//    dev = alloc_netdev(sizeof(*priv), "eth%d", ether_setup);
    if (!config->switch_type) {
	dev = alloc_switchdev(
	    -1, sizeof(struct tile_priv), mpipe->tx_cpus_count);
    }
    else {
	dev = alloc_switchdev(
	    0, sizeof(struct tile_priv), mpipe->tx_cpus_count);
    }
    if (!dev) {
	printk("alloc_netdev failed\n");
	return;
    }

    dev->netdev_ops = &tile_ops;
    dev->watchdog_timeo = 5 * HZ;
    dev->features |= NETIF_F_LLTX;
    dev->mtu = 1500;
    dev->l2mtu = 1580;
    dev->ethtool_ops = &tile_ethtool_ops;
    dev->needs_free_netdev = true;

    if (config->hv_name_xaui) {
	++mpipe->dev_10g_count;
    }
    else {
	++mpipe->dev_1g_count;
    }
    priv = netdev_priv(dev);
    priv->mpipe = mpipe;
    priv->sw.dev = dev;
    priv->config = config;
    priv->devno = devno;
    INIT_WORK(&priv->sw.restart_task, tile_reset_task);

    priv->sw.phy.ops = config->phy_ops;
    if (!priv->sw.phy.ops) {
	priv->sw.phy.ops = &phy_atheros_led_ops;
    }
    priv->sw.phy.mii_info.dev = dev;
    priv->sw.phy.mii_info.phy_id = config->mdio;
    priv->sw.phy.mii_info.supports_gmii = 1;
    priv->sw.phy.mii_info.phy_id_mask = 0x3f;
    priv->sw.phy.mii_info.reg_num_mask = 0x1f;
    priv->sw.phy.mii_info.mdio_read = tile_gbe_mdio_read;
    priv->sw.phy.mii_info.mdio_write = tile_gbe_mdio_write;
    priv->sw.phy.change_callback = tile_link_changed;

    xs = &priv->sw.xstats[0];
    eth_stats_set_driver_basic_flags(xs);
    if (config->phy_1g) {
	if (priv->config->gpio_phy_1g_int) {
	    priv->sw.phy.manually_updated = true;
	}
    }
    if (config->sfpplus) {
	priv->sw.phy.mii_info.phy_id_mask = 0xffff;
	priv->sw.phy.mii_info.reg_num_mask = 0xffff;
	priv->sw.phy.mii_info.mdio_read = tile_xgbe_mdio_read;
	priv->sw.phy.mii_info.mdio_write = tile_xgbe_mdio_write;
    }
    eth_stats_set_flag(xs, ETH_STATS_TX_BYTE);
    eth_stats_set_flag(xs, ETH_STATS_TX_PACKET);
    eth_stats_set_flag(xs, ETH_STATS_TX_BROADCAST);
    eth_stats_set_flag(xs, ETH_STATS_TX_MULTICAST);
    eth_stats_set_flag(xs, ETH_STATS_TX_PAUSE);
    eth_stats_set_flag(xs, ETH_STATS_TX_64);
    eth_stats_set_flag(xs, ETH_STATS_TX_65_127);
    eth_stats_set_flag(xs, ETH_STATS_TX_128_255);
    eth_stats_set_flag(xs, ETH_STATS_TX_256_511);
    eth_stats_set_flag(xs, ETH_STATS_TX_512_1023);
    eth_stats_set_flag(xs, ETH_STATS_TX_1024_1518);
    eth_stats_set_flag(xs, ETH_STATS_TX_1519_MAX);
    eth_stats_set_flag(xs, ETH_STATS_TX_UNDERRUN);
    eth_stats_set_flag(xs, ETH_STATS_TX_SINGLE_COL);
    eth_stats_set_flag(xs, ETH_STATS_TX_MULTI_COL);
    eth_stats_set_flag(xs, ETH_STATS_TX_EXCESSIVE_COL);
    eth_stats_set_flag(xs, ETH_STATS_TX_LATE_COL);
    eth_stats_set_flag(xs, ETH_STATS_TX_DEFERRED);
    eth_stats_set_flag(xs, ETH_STATS_TX_CARRIER_SENSE_ERR);
    eth_stats_set_flag(xs, ETH_STATS_TX_FCS_ERR);
    eth_stats_set_flag(xs, ETH_STATS_RX_BYTE);
    eth_stats_set_flag(xs, ETH_STATS_RX_PACKET);
    eth_stats_set_flag(xs, ETH_STATS_RX_BROADCAST);
    eth_stats_set_flag(xs, ETH_STATS_RX_MULTICAST);
    eth_stats_set_flag(xs, ETH_STATS_RX_PAUSE);
    eth_stats_set_flag(xs, ETH_STATS_RX_64);
    eth_stats_set_flag(xs, ETH_STATS_RX_65_127);
    eth_stats_set_flag(xs, ETH_STATS_RX_128_255);
    eth_stats_set_flag(xs, ETH_STATS_RX_256_511);
    eth_stats_set_flag(xs, ETH_STATS_RX_512_1023);
    eth_stats_set_flag(xs, ETH_STATS_RX_1024_1518);
    eth_stats_set_flag(xs, ETH_STATS_RX_1519_MAX);
    eth_stats_set_flag(xs, ETH_STATS_RX_TOO_SHORT);
    eth_stats_set_flag(xs, ETH_STATS_RX_TOO_LONG);
    eth_stats_set_flag(xs, ETH_STATS_RX_JABBER);
    eth_stats_set_flag(xs, ETH_STATS_RX_FCS_ERR);
    eth_stats_set_flag(xs, ETH_STATS_RX_LENGTH_ERR);
    eth_stats_set_flag(xs, ETH_STATS_RX_CODE_ERR);
    eth_stats_set_flag(xs, ETH_STATS_RX_ALIGN_ERR);
    eth_stats_set_flag(xs, ETH_STATS_RX_OVERRUN);
    eth_stats_set_flag(xs, ETH_STATS_RX_IP_HDR_CSUM_ERR);
    eth_stats_set_flag(xs, ETH_STATS_RX_TCP_CSUM_ERR);
    eth_stats_set_flag(xs, ETH_STATS_RX_UDP_CSUM_ERR);
    if (config->sfp_i2c_adapter_nr) {
	i2c_sfp_reset(&priv->i2c_sfp);
	priv->i2c_sfp.adapter = config->sfp_i2c_adapter_nr;
    }

    if (!config->switch_type) {
	memcpy(dev->dev_addr, mac, 6);
	dev->addr_len = 6;

	++mac[5];
	if (!mac[5]) ++mac[4];
    }
    ret = register_netdevice(dev);
    if (ret) {
	netdev_err(dev, "register_netdev failed %d\n", ret);
	free_percpu(priv->tx_entries);
	free_netdev(dev);
	return;
    }
    netif_carrier_off(dev);
//    printk("tilegx: register %s/%s -> %s priv_flags:%08x %pM\n",
//	    config->hv_name, config->hv_name_xaui, dev->name, dev->priv_flags,
//	    dev->dev_addr);

    priv->sw.supported_l2mtu = TILE_JUMBO;
    if (config->switch_type) {
	priv->sw.type = config->switch_type;
	priv->sw.num = 0;
	priv->sw.tx_buffer_count = 2048;
	priv->sw.specific_config = 2 | BIT(8) | BIT(17); // tx delay 2, cpu port is sgmii, sgmii in phy mode

	priv->sw.mii_phy.mdio_read = tile_gbe_mdio_read;
	priv->sw.mii_phy.mdio_write = tile_gbe_mdio_write;
	priv->sw.mii_phy.phy_id_mask = 0x1f;
	priv->sw.mii_phy.reg_num_mask = 0x1f;
	priv->sw.mii_phy.supports_gmii = 1;
	priv->sw.mii_switch.mdio_read = tile_gbe_mdio_read;
	priv->sw.mii_switch.mdio_write = tile_gbe_mdio_write;

	ret = switch_init(&priv->sw, config->switch_port_map, mac);
	if (ret) {
	    printk("switch init failed %d\n", ret);
	    unregister_netdevice(dev);
	    return;
	}
    }
    else {
	if (priv->sw.phy.ops) {
	    phy_setup(&priv->sw.phy);
	}
    }

    if (config->gpio_sfp_led) {
	sprintf(priv->led.name, "%s-led", dev->name);
	priv->led.classdev.name = priv->led.name;
	priv->led.classdev.brightness_set = tile_led_set;
	led_classdev_register(NULL, &priv->led.classdev);
	tile_led_set(&priv->led.classdev, 0);
    }
    if (config->gpio_sfp_led2) {
	sprintf(priv->led2.name, "%s-led2", dev->name);
	priv->led2.classdev.name = priv->led2.name;
	priv->led2.classdev.brightness_set = tile_led2_set;
	led_classdev_register(NULL, &priv->led2.classdev);
	tile_led2_set(&priv->led2.classdev, 0);
    }

    priv->tx_queue.edma = -1;
    priv->tx_queue_fast.edma = -1;
    state.devs[devno] = dev;

#ifdef TEST_LOOPBACK
    static struct net_device *pp;
    if (!pp) {
//	if (!strcmp(dev->name, "eth5")) {
	    pp = dev;
//	}
    }
    else {
//	if (!strcmp(dev->name, "eth6")) {
	    xmit_map[pp->ifindex] = dev;
	    xmit_map[dev->ifindex] = pp;
	    printk("xmit_map %s <-> %s\n", pp->name, dev->name);
	    pp = NULL;
//	}
    }
#endif
}


#ifdef STATS_TIMESTAMPS
static ssize_t tile_stamps_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf) {
    unsigned i;
    for (i = 0; i < 1000; ++i) {
	unsigned idx = atomic_inc_return(&stamps_read) - 1;
	if (idx >= atomic_read(&stamps_filled) || idx >= MAX_TIME_STAMPS) {
	    break;
	}
	sprintf(buf + i * 26, "%02x %04x %08x %08x\n",
		stamps[idx].cpu, stamps[idx].sec, stamps[idx].nsec,
		stamps[idx].flow);
    }
    return i * 26;
}

static ssize_t tile_stamps_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count) {
    atomic_set(&stamps_read, 0);
    atomic_set(&stamps_filled, 0);
    return count;
}

static struct kobj_attribute tile_stamps_attribute =
    __ATTR(tile_stamps, 0664, tile_stamps_show, tile_stamps_store);
#endif

static struct attribute *attrs[] = {
#ifdef STATS_TIMESTAMPS
    &tile_stamps_attribute.attr,
#endif
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

static int __init tile_init(void) {
    char board_name[64];
    char type_name[64];
    char name[GXIO_MPIPE_LINK_NAME_LEN];
    uint8_t mac[6];
    uint8_t mac_temp[6];
    int ret;
    int i, k;

    ret = sysfs_create_group(kernel_kobj, &attr_group);
    if (ret) {
	return ret;
    }
    tile_combo_ops = phy_marvel_ops;
    tile_combo_ops.default_ops = &tile_sfp_ops;

    memset(board_name, 0, sizeof(board_name));
    ret = read_booter_cfg(ID_BOARD_NAME, board_name, sizeof(board_name));

    memset(type_name, 0, sizeof(type_name));
    ret = read_booter_cfg(ID_BOARD_TYPE_NAME, type_name, sizeof(type_name));
#ifdef DEBUG
    printk("tile: debug\n");
#endif
    printk("tile: name: %s\n", board_name);
    printk("tile: type: %s\n", type_name);
    ports_config = ccr1036_ports;
    if (!strcmp(board_name, "CCR1036-12G-4S")) {
	ports_config = ccr1036_ports;
    }
    if (!strcmp(type_name, "ccr1036r2")) {
	ports_config = ccr1036r2_ports;
    }
    if (!strcmp(board_name, "CCR1016-12G")) {
	ports_config = ccr1016_ports;
    }
    if (!strcmp(type_name, "ccr1016r2")) {
	ports_config = ccr1016r2_ports;
    }
    if (!strcmp(board_name, "CCR1036-8G-2S+")) {
	if (!strcmp(type_name, "ccr1036r2")) {
	    ports_config = ccr1036r2_8g_2splus_ports;
	}
	else {
	    ports_config = ccr1036_8g_2splus_ports;
	}
    }
    if (!strcmp(board_name, "CCR1009-10G") ||
	    !strcmp(board_name, "CCR1009-8G-1S-1S+") ||
	    !strcmp(board_name, "CCR1016-8G") ||
	    !strcmp(board_name, "CCR1016-8G-1S-1S+")) {
	ports_config = ccr1009_10g;
    }
    if (!strcmp(board_name, "CCR1009-8G-1Sc-1S+") ||	// obsolete name
	    !strcmp(board_name, "CCR1009-7G-1C-1S+")) {
	ports_config = ccr1009_8g_1sc_1splus;
	i2c_gpio_init(&state.gpio_expander[GPIO_EXP_48], 2, 0x48);
    }
    if (!strcmp(board_name, "CCR1009-7G-1C")) {
	ports_config = ccr1009_7g_1sc;
    }
    if (!strcmp(board_name, "CCR1016-12S-1S+") ||
	    !strcmp(board_name, "CCR1036-12S-1S+")) {
	ports_config = ccr1036_12s_1splus_ports;
	i2c_gpio_init(&state.gpio_expander[GPIO_EXP_48], 1, 0x48);
	i2c_gpio_init(&state.gpio_expander[GPIO_EXP_4A], 1, 0x4A);
	i2c_gpio_init(&state.gpio_expander[GPIO_EXP_4C], 1, 0x4C);
	i2c_gpio_init(&state.gpio_expander[GPIO_EXP_4E], 1, 0x4E);
    }
    if (!strcmp(board_name, "CCR1009-8G-1S")) {
	ports_config = ccr1009_8g_1s;
    }
    if (!strcmp(board_name, "CCR1072-1G-8S+")) {
	if (is_CCR1072_r3()) {
	    printk("tile: CCR1072r3\n");
	    ports_config = ccr1072_1g_8splus_r3_ports;
	}
	else {
	    ports_config = ccr1072_1g_8splus_ports;
	}
    }

    state.mdio_lock = __SPIN_LOCK_UNLOCKED(mdio_lock);
    INIT_WORK(&state.phy_1g_irq_work, tile_phy_1g_irq_work);
    u64 gpio_in = 0;
    u64 gpio_out = 0;
    u64 gpio_phy_1g_int = 0;
    struct tile_port_config *config = ports_config;
    while (config->hv_name || config->hv_name_xaui) {
	gpio_in |= tile_get_port_config_input_gpios(config);
	gpio_out |= tile_get_port_config_output_gpios(config);
	gpio_phy_1g_int |= tile_get_port_config_phy_1g_interrupt_gpios(config);
	tile_config_i2c_gpios(config);
	++config;
    }
/*
    i2c_gpio_dump(&state.gpio_expander[GPIO_EXP_48]);
    i2c_gpio_dump(&state.gpio_expander[GPIO_EXP_4A]);
    i2c_gpio_dump(&state.gpio_expander[GPIO_EXP_4C]);
    i2c_gpio_dump(&state.gpio_expander[GPIO_EXP_4E]);
*/
    printk("tile: gpio in:%llx out:%llx, phy_int:%llx\n",
	    gpio_in, gpio_out, gpio_phy_1g_int);

    if (gpio_in | gpio_out | gpio_phy_1g_int) {
	ret = gxio_gpio_init(&state.gpio_ctx, 0);
	if (ret) {
	    printk("gxio_gpio_init: %d\n", ret);
	    return ret;
	}
	ret = gxio_gpio_attach(&state.gpio_ctx, gpio_in | gpio_out);
	if (ret) {
	    printk("gxio_gpio_attach: %d %016llx %016llx\n", ret, gpio_in, gpio_out);
	    return ret;
	}
	ret = gxio_gpio_set_dir(&state.gpio_ctx, 0, gpio_in, gpio_out, 0);
	if (ret) {
	    printk("gxio_gpio_set_dir: %d\n", ret);
	    return ret;
	}

	state.phy_1g_irq = irq_alloc_hwirq(-1);
	tile_irq_activate(state.phy_1g_irq, TILE_IRQ_PERCPU);
	BUG_ON(request_irq(
		    state.phy_1g_irq, tile_phy_1g_irq, 0, "eth phy", NULL));

	ret = gxio_gpio_cfg_interrupt(
	    &state.gpio_ctx,
	    cpu_x(smp_processor_id()), cpu_y(smp_processor_id()),
	    KERNEL_PL, state.phy_1g_irq, 0, gpio_phy_1g_int);
	if (ret) {
	    printk("gxio_gpio_cfg_interrupt: %d\n", ret);
	    return ret;
	}
	u64 deasserted;
	gxio_gpio_report_reset_interrupt(&state.gpio_ctx, NULL, &deasserted);
    }

    on_each_cpu(tile_prepare_cpu, NULL, 1);

    for (i = 0; i < NR_MPIPE_MAX; ++i) {
	state.mpipe[i].instance = i;
	tile_mpipe_init(&state.mpipe[i]);
    }

    gxio_mpipe_link_enumerate_mac(0, name, mac);
/*
    for (i = 0; gxio_mpipe_link_enumerate_mac(i, name, mac_temp) >= 0; i++) {
	printk("mpipe dev: %s\n", name);
    }
*/
    rtnl_lock();
    for (k = 0; ports_config[k].hv_name || ports_config[k].hv_name_xaui; ++k) {
	const char *hv_name = ports_config[k].hv_name;
	const char *hv_name_xaui = ports_config[k].hv_name_xaui;
	bool gbe = false;
	bool xgbe = false;
	int inst = -1;
	if (hv_name) {
	    inst = gxio_mpipe_link_instance(hv_name);
	}
	if (inst < 0 && hv_name_xaui) {
	    inst = gxio_mpipe_link_instance(hv_name_xaui);
	}
//	printk("find %s/%s inst:%d\n", hv_name, hv_name_xaui, inst);
	if (inst < 0) {
	    continue;
	}
	for (i = 0; gxio_mpipe_link_enumerate_mac(i, name, mac_temp) >= 0; i++) {
//	    printk(" check %s\n", name);
	    if (hv_name && !strcmp(name, hv_name)) {
		gbe = true;
	    }
	    if (hv_name_xaui && !strcmp(name, hv_name_xaui)) {
		xgbe = true;
	    }
	}
	if (gbe || xgbe) {
	    if (ports_config[k].sfpplus) {
		ports_config[k].sfp = true;
	    }
	    if (!gbe) {
		ports_config[k].sfp = false;
		ports_config[k].hv_name = NULL;
	    }
	    if (!xgbe) {
		ports_config[k].sfpplus = false;
		ports_config[k].hv_name_xaui = NULL;
	    }
	    tile_dev_init(&state.mpipe[inst],
		    &ports_config[k], mac);
	}
    }
    rtnl_unlock();

    return 0;
}

static void __exit tile_cleanup(void) {
    int i;

    sysfs_remove_group(kernel_kobj, &attr_group);

    if (state.phy_1g_irq) {
	free_irq(state.phy_1g_irq, NULL);
	irq_free_hwirq(state.phy_1g_irq);
	cancel_work_sync(&state.phy_1g_irq_work);
    }

    rtnl_lock();
    for (i = 0; i < TILE_DEVS_MAX; i++) {
	struct net_device *dev = state.devs[i];
	if (dev) {
	    struct tile_priv *priv = netdev_priv(dev);
	    if (netif_running(dev)) {
		dev_close(dev);
	    }
	    cancel_work_sync(&priv->sw.restart_task);
	    if (priv->sw.ops) {
		switch_cleanup(&priv->sw);
	    }
	    if (priv->sw.phy.inited) {
		phy_cleanup(&priv->sw.phy);
	    }
	    if (priv->config->gpio_sfp_led) {
		led_classdev_unregister(&priv->led.classdev);
	    }
	    if (priv->config->gpio_sfp_led2) {
		led_classdev_unregister(&priv->led2.classdev);
	    }
	    state.devs[i] = NULL;
	    unregister_netdevice(dev);
	}
    }
    rtnl_unlock();
    for (i = 0; i < NR_MPIPE_MAX; ++i) {
	tile_mpipe_cleanup(&state.mpipe[i]);
    }

    if (state.gpio_ctx.mmio_base) {
	gxio_gpio_destroy(&state.gpio_ctx);
    }
}

module_init(tile_init);
module_exit(tile_cleanup);
