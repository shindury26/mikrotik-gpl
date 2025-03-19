// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Marvell PPv2 network controller for Armada 375 SoC.
 *
 * Copyright (C) 2014 Marvell
 *
 * Marcin Wojtas <mw@semihalf.com>
 */

#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/mbus.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/genalloc.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/regmap.h>
#include <linux/cycles.h>
#include <linux/flash.h>
#include <linux/gpio.h>
#include <linux/sfp_helper.h>
#include <net/busy_poll.h>

#include "mvpp2.h"
#include "mvpp2_prs.h"
#include "mvpp2_cls.h"



#define MVMDIO_SMI_PHY_ADDR_SHIFT	16
#define MVMDIO_SMI_PHY_REG_SHIFT	21
#define MVMDIO_SMI_READ_OPERATION	BIT(26)
#define MVMDIO_SMI_WRITE_OPERATION	0
#define MVMDIO_SMI_READ_VALID		BIT(27)
#define MVMDIO_SMI_BUSY			BIT(28)
#define MVMDIO_ERR_INT_CAUSE		0x007C
#define  MVMDIO_ERR_INT_SMI_DONE	0x00000010
#define MVMDIO_ERR_INT_MASK		0x0080

#define MVMDIO_XSMI_MGNT_REG		0x0
#define  MVMDIO_XSMI_PHYADDR_SHIFT	16
#define  MVMDIO_XSMI_DEVADDR_SHIFT	21
#define  MVMDIO_XSMI_WRITE_OPERATION	(0x5 << 26)
#define  MVMDIO_XSMI_READ_OPERATION	(0x7 << 26)
#define  MVMDIO_XSMI_READ_VALID		BIT(29)
#define  MVMDIO_XSMI_BUSY		BIT(30)
#define MVMDIO_XSMI_ADDR_REG		0x8

static void __iomem *mdio_regs;
static struct clk *mdio_clk[5];
static struct mutex mdio_mutex;
static struct hrtimer mdio_timer;
static wait_queue_head_t mdio_wait;
static struct mdio_op_descr *mdio_op;
static int mdio_retries;
//static unsigned long mdio_start_cycles;
#define MDIO_OP_NSEC (5 * 1000)

static int is_type(const char *type) {
	return 0 == strcmp(type, get_hcfg(ID_BOARD_TYPE_NAME, NULL));
}

static void mvpp2_mdio_start_op(void) {
//	mdio_start_cycles = cycles_get_cycles();
	u32 data = (mdio_op->phy << MVMDIO_SMI_PHY_ADDR_SHIFT) |
		(mdio_op->reg << MVMDIO_SMI_PHY_REG_SHIFT)  |
		(mdio_op->read ? MVMDIO_SMI_READ_OPERATION : MVMDIO_SMI_WRITE_OPERATION) |
		mdio_op->data;
	writel(data, mdio_regs);
	mdio_retries = 0;
}

static enum hrtimer_restart mvpp2_mdio_timer(struct hrtimer *t) {
	unsigned status = readl(mdio_regs);
	if (status & MVMDIO_SMI_BUSY) {
		++mdio_retries;
		if (mdio_retries < 1000) {
			hrtimer_forward_now(&mdio_timer, ktime_set(0, MDIO_OP_NSEC));
			return HRTIMER_RESTART;
		}
		if (net_ratelimit()) {
			printk("mvpp2: mdio timeout\n");
		}
	}
//	else {
//		if (net_ratelimit()) {
//			printk("mvpp2: mdio done %u %lu %04x\n", mdio_retries, cycles_get_cycles() - mdio_start_cycles, status & 0xffff);
//		}
//	}
	if (mdio_op->read) {
		mdio_op->data = status & 0xffff;
	}

	++mdio_op;
	if (mdio_op->end) {
		wake_up(&mdio_wait);
		return HRTIMER_NORESTART;
	}
	mvpp2_mdio_start_op();
	hrtimer_forward_now(&mdio_timer, ktime_set(0, MDIO_OP_NSEC));
	return HRTIMER_RESTART;
}

static void mvpp2_mdio_bulk(struct net_device *dev, struct mdio_op_descr *ops) {
    mutex_lock(&mdio_mutex);
    mdio_op = ops;
    mvpp2_mdio_start_op();

    unsigned long flags;
    wait_queue_entry_t wait;
    init_waitqueue_entry(&wait, current);
    __set_current_state(TASK_UNINTERRUPTIBLE);
    spin_lock_irqsave(&mdio_wait.lock, flags);
    __add_wait_queue(&mdio_wait, &wait);

    hrtimer_start(&mdio_timer, ktime_set(0, MDIO_OP_NSEC), HRTIMER_MODE_REL);

    spin_unlock(&mdio_wait.lock);
    schedule_timeout(MAX_SCHEDULE_TIMEOUT);
    remove_wait_queue(&mdio_wait, &wait);

//    if (net_ratelimit()) {
//	printk("%s: mdio bulk done\n", mdio_adapter->name);
//    }
    mutex_unlock(&mdio_mutex);
}

static int orion_mdio_smi_read(struct net_device *dev, int phy, int regnum) {
	struct mdio_op_descr op[2];
	mdio_op_fill(&op[0], 1, phy, regnum, 0xffff);
	mdio_op_end(&op[1]);
	mvpp2_mdio_bulk(NULL, op);
	return op[0].data;
}

static void orion_mdio_smi_write(struct net_device *dev, int phy,
				 int regnum, int value) {
	struct mdio_op_descr op[2];
	mdio_op_fill(&op[0], 0, phy, regnum, value);
	mdio_op_end(&op[1]);
	mvpp2_mdio_bulk(NULL, op);
}

/*
static bool orion_mdio_xsmi_is_done(void) {
	return !(readl(mdio_regs + MVMDIO_XSMI_MGNT_REG) & MVMDIO_XSMI_BUSY);
}

static int orion_mdio_xsmi_read(struct net_device *dev, int phy, int regnum) {
	u16 dev_addr = (regnum >> 16) & GENMASK(4, 0);

	writel(regnum & GENMASK(15, 0), mdio_regs + MVMDIO_XSMI_ADDR_REG);
	writel((phy << MVMDIO_XSMI_PHYADDR_SHIFT) |
	       (dev_addr << MVMDIO_XSMI_DEVADDR_SHIFT) |
	       MVMDIO_XSMI_READ_OPERATION,
	       mdio_regs + MVMDIO_XSMI_MGNT_REG);

	orion_mdio_wait_ready(orion_mdio_xsmi_is_done);

	if (!(readl(mdio_regs + MVMDIO_XSMI_MGNT_REG) &
	      MVMDIO_XSMI_READ_VALID)) {
		return -1;
	}

	return readl(mdio_regs + MVMDIO_XSMI_MGNT_REG) & GENMASK(15, 0);
}

static void orion_mdio_xsmi_write(struct net_device *dev, int phy, int regnum, u16 value) {
	u16 dev_addr = (regnum >> 16) & GENMASK(4, 0);

	writel(regnum & GENMASK(15, 0), mdio_regs + MVMDIO_XSMI_ADDR_REG);
	writel((phy << MVMDIO_XSMI_PHYADDR_SHIFT) |
	       (dev_addr << MVMDIO_XSMI_DEVADDR_SHIFT) |
	       MVMDIO_XSMI_WRITE_OPERATION | value,
	       mdio_regs + MVMDIO_XSMI_MGNT_REG);

	orion_mdio_wait_ready(orion_mdio_xsmi_is_done);
}
*/

static int mvpp2_bm_pool_create(struct platform_device *pdev,
				struct mvpp2 *priv,
				struct mvpp2_bm_pool *bm_pool, int size)
{
	u32 val;
	bm_pool->size = size;
	bm_pool->buf_num = 0;
	bm_pool->virt_addr = dma_alloc_coherent(&pdev->dev, 16 * bm_pool->size,
						&bm_pool->dma_addr,
						GFP_KERNEL);
	if (!bm_pool->virt_addr)
		return -ENOMEM;

	mvpp2_write(priv, MVPP2_BM_POOL_BASE_REG(bm_pool->id),
		    lower_32_bits(bm_pool->dma_addr));
	mvpp2_write(priv, MVPP2_BM_POOL_SIZE_REG(bm_pool->id), size);

	val = mvpp2_read(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id));
	val |= MVPP2_BM_START_MASK;
	val &= ~MVPP2_BM_LOW_THRESH_MASK;
	val &= ~MVPP2_BM_HIGH_THRESH_MASK;
	val |= MVPP2_BM_LOW_THRESH_VALUE(MVPP2_BM_BPPI_LOW_THRESH);
	val |= MVPP2_BM_HIGH_THRESH_VALUE(MVPP2_BM_BPPI_HIGH_THRESH);
	mvpp2_write(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id), val);
	return 0;
}

static void mvpp2_bm_pool_drain(struct mvpp2 *priv, struct mvpp2_bm_pool *bm_pool) {
	int i;
	int buf_num = mvpp2_read(priv, MVPP2_BM_POOL_PTRS_NUM_REG(bm_pool->id)) &
		MVPP22_BM_POOL_PTRS_NUM_MASK;
	buf_num += mvpp2_read(priv, MVPP2_BM_BPPI_PTRS_NUM_REG(bm_pool->id)) &
		MVPP2_BM_BPPI_PTR_NUM_MASK;
	if (buf_num)
		buf_num += 1;

	if (buf_num > bm_pool->buf_num) {
		printk("hw bufs:%u sw bufs:%u\n", buf_num, bm_pool->buf_num);
		buf_num = bm_pool->buf_num;
	}

	for (i = 0; i < buf_num; i++) {
		unsigned cpu = smp_processor_id();
		dma_addr_t dma_addr = mvpp2_thread_read(priv, cpu, MVPP2_BM_PHY_ALLOC_REG(bm_pool->id));
		if (sizeof(dma_addr) == 8) {
			dma_addr |= (u64)(mvpp2_thread_read(priv, cpu, MVPP22_BM_ADDR_HIGH_ALLOC) & MVPP22_BM_ADDR_HIGH_PHYS_MASK) << 32;
		}

		void *data = (void *)phys_to_virt(dma_addr);
//		printk("%i bm pool destr %llx %px\n", i, phys_addr, data);
		if (!data)
			break;
		skb_bin_put_buffer(data);
	}
	bm_pool->buf_num -= i;
}

static void mvpp2_bm_pool_destroy(struct platform_device *pdev,
				 struct mvpp2 *priv,
				 struct mvpp2_bm_pool *bm_pool)
{
	u32 val;
	val = mvpp2_read(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id));
	val |= MVPP2_BM_STOP_MASK;
	mvpp2_write(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id), val);

	dma_free_coherent(&pdev->dev, 16 * bm_pool->size,
			  bm_pool->virt_addr,
			  bm_pool->dma_addr);
}

#define BM_POOL_SIZE 1024
static int mvpp2_bm_init(struct platform_device *pdev, struct mvpp2 *priv)
{
	int i, err, cpu;
	struct mvpp2_bm_pool *bm_pool;

	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		mvpp2_write(priv, MVPP2_BM_INTR_MASK_REG(i), 0);
		mvpp2_write(priv, MVPP2_BM_INTR_CAUSE_REG(i), 0);
	}

	for_each_present_cpu(cpu)
		mvpp2_thread_write(priv, cpu, MVPP2_BM_VIRT_RLS_REG, 0x0);

	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		bm_pool = &priv->bm_pool[i];
		bm_pool->id = i;
		err = mvpp2_bm_pool_create(pdev, priv, bm_pool, BM_POOL_SIZE);
		if (err)
			goto err_unroll_pools;
	}
	return 0;

err_unroll_pools:
	for (i = i - 1; i >= 0; i--)
		mvpp2_bm_pool_destroy(pdev, priv, &priv->bm_pool[i]);
	return err;
}

static void mvpp2_rxq_pool_set(struct mvpp2_port *port, int lrxq, int pool)
{
	u32 val, mask;
	int prxq;

	/* Get queue physical ID */
	prxq = port->rxqs[lrxq].id;

	mask = MVPP22_RXQ_POOL_SHORT_MASK;

	val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~mask;
	val |= (pool << MVPP2_RXQ_POOL_SHORT_OFFS) & mask;
	mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

/* Routine enable flow control for RXQs conditon */
void mvpp2_rxq_enable_fc(struct mvpp2_port *port)
{
	int val, cm3_state, host_id, q;
	int fq = port->first_rxq;

	/* Remove Flow control enable bit to prevent race between FW and Kernel
	 * If Flow control were enabled, it would be re-enabled.
	 */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	cm3_state = (val & FLOW_CONTROL_ENABLE_BIT);
	val &= ~FLOW_CONTROL_ENABLE_BIT;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);

	/* Set same Flow control for all RXQs */
	for (q = 0; q < port->priv->nthreads; q++) {
		/* Set stop and start Flow control RXQ thresholds */
		val = MSS_THRESHOLD_START;
		val |= (MSS_THRESHOLD_STOP << MSS_RXQ_TRESH_STOP_OFFS);
		mvpp2_cm3_write(port->priv, MSS_RXQ_TRESH_REG(q, fq), val);

		val = mvpp2_cm3_read(port->priv, MSS_RXQ_ASS_REG(q, fq));
		/* Set RXQ port ID */
		val &= ~(MSS_RXQ_ASS_PORTID_MASK << MSS_RXQ_ASS_Q_BASE(q, fq));
		val |= (port->id << MSS_RXQ_ASS_Q_BASE(q, fq));
		val &= ~(MSS_RXQ_ASS_HOSTID_MASK << (MSS_RXQ_ASS_Q_BASE(q, fq)
			+ MSS_RXQ_ASS_HOSTID_OFFS));

		host_id = q;

		/* Set RXQ host ID */
		val |= (host_id << (MSS_RXQ_ASS_Q_BASE(q, fq)
			+ MSS_RXQ_ASS_HOSTID_OFFS));

		mvpp2_cm3_write(port->priv, MSS_RXQ_ASS_REG(q, fq), val);
	}

	/* Notify Firmware that Flow control config space ready for update */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	val |= FLOW_CONTROL_UPDATE_COMMAND_BIT;
	val |= cm3_state;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);
}

/* Routine disable flow control for RXQs conditon */
void mvpp2_rxq_disable_fc(struct mvpp2_port *port)
{
	int val, cm3_state, q;
	int fq = port->first_rxq;

	/* Remove Flow control enable bit to prevent race between FW and Kernel
	 * If Flow control were enabled, it would be re-enabled.
	 */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	cm3_state = (val & FLOW_CONTROL_ENABLE_BIT);
	val &= ~FLOW_CONTROL_ENABLE_BIT;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);

	/* Disable Flow control for all RXQs */
	for (q = 0; q < port->priv->nthreads; q++) {
		/* Set threshold 0 to disable Flow control */
		val = 0;
		val |= (0 << MSS_RXQ_TRESH_STOP_OFFS);
		mvpp2_cm3_write(port->priv, MSS_RXQ_TRESH_REG(q, fq), val);

		val = mvpp2_cm3_read(port->priv, MSS_RXQ_ASS_REG(q, fq));

		val &= ~(MSS_RXQ_ASS_PORTID_MASK << MSS_RXQ_ASS_Q_BASE(q, fq));

		val &= ~(MSS_RXQ_ASS_HOSTID_MASK << (MSS_RXQ_ASS_Q_BASE(q, fq)
			+ MSS_RXQ_ASS_HOSTID_OFFS));

		mvpp2_cm3_write(port->priv, MSS_RXQ_ASS_REG(q, fq), val);
	}

	/* Notify Firmware that Flow control config space ready for update */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	val |= FLOW_CONTROL_UPDATE_COMMAND_BIT;
	val |= cm3_state;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);
}

/* Routine disable/enable flow control for BM pool conditon */
void mvpp2_bm_pool_update_fc(struct mvpp2_port *port,
			     struct mvpp2_bm_pool *pool,
			     bool en)
{
	int val, cm3_state;

	/* Remove Flow control enable bit to prevent race between FW and Kernel
	 * If Flow control were enabled, it would be re-enabled.
	 */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	cm3_state = (val & FLOW_CONTROL_ENABLE_BIT);
	val &= ~FLOW_CONTROL_ENABLE_BIT;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);

	/* Check if BM pool should be enabled/disable */
	if (en) {
		/* Set BM pool start and stop thresholds per port */
		val = mvpp2_cm3_read(port->priv, MSS_BUF_POOL_REG(pool->id));
		val |= MSS_BUF_POOL_PORT_OFFS(port->id);
		val &= ~MSS_BUF_POOL_START_MASK;
		val |= (MSS_THRESHOLD_START << MSS_BUF_POOL_START_OFFS);
		val &= ~MSS_BUF_POOL_STOP_MASK;
		val |= MSS_THRESHOLD_STOP;
		mvpp2_cm3_write(port->priv, MSS_BUF_POOL_REG(pool->id), val);
	} else {
		/* Remove BM pool from the port */
		val = mvpp2_cm3_read(port->priv, MSS_BUF_POOL_REG(pool->id));
		val &= ~MSS_BUF_POOL_PORT_OFFS(port->id);

		/* Zero BM pool start and stop thresholds to disable pool
		 * flow control if pool empty (not used by any port)
		 */
		if (!pool->buf_num) {
			val &= ~MSS_BUF_POOL_START_MASK;
			val &= ~MSS_BUF_POOL_STOP_MASK;
		}

		mvpp2_cm3_write(port->priv, MSS_BUF_POOL_REG(pool->id), val);
	}

	/* Notify Firmware that Flow control config space ready for update */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	val |= FLOW_CONTROL_UPDATE_COMMAND_BIT;
	val |= cm3_state;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);
}

static int mvpp2_enable_global_fc(struct mvpp2 *priv)
{
	int val, timeout = 0;

	/* Enable global flow control. In this stage global
	 * flow control enabled, but still disabled per port.
	 */
	val = mvpp2_cm3_read(priv, MSS_FC_COM_REG);
	val |= FLOW_CONTROL_ENABLE_BIT;
	mvpp2_cm3_write(priv, MSS_FC_COM_REG, val);

	/* Check if Firmware running and disable FC if not*/
	val |= FLOW_CONTROL_UPDATE_COMMAND_BIT;
	mvpp2_cm3_write(priv, MSS_FC_COM_REG, val);

	while (timeout < MSS_FC_MAX_TIMEOUT) {
		val = mvpp2_cm3_read(priv, MSS_FC_COM_REG);

		if (!(val & FLOW_CONTROL_UPDATE_COMMAND_BIT))
			return 0;
		usleep_range(10, 20);
		timeout++;
	}

	priv->global_tx_fc = false;
	return -ENOTSUPP;
}

static inline void mvpp2_bm_pool_put(struct mvpp2_port *port, int pool,
				     dma_addr_t buf_dma_addr)
{
	unsigned int cpu = smp_processor_id();
//	printk("bm_put %llx\n", buf_dma_addr);
#if defined(CONFIG_ARCH_DMA_ADDR_T_64BIT) && defined(CONFIG_PHYS_ADDR_T_64BIT)
//	mvpp2_thread_write_relaxed(port->priv, cpu, MVPP22_BM_ADDR_HIGH_RLS_REG,
//				   upper_32_bits(buf_dma_addr) & MVPP22_BM_ADDR_HIGH_PHYS_RLS_MASK);
#endif

	mvpp2_thread_write_relaxed(port->priv, cpu,
				   MVPP2_BM_PHY_RLS_REG(pool), buf_dma_addr);
}

static int mvpp2_swf_bm_pool_init(struct mvpp2_port *port)
{
	int rxq;
	struct mvpp2_bm_pool *pool = &port->priv->bm_pool[0];
	int i;

	for (i = 0; i < pool->size; i++) {
		void *buf = skb_bin_alloc_buffer(GFP_KERNEL);
		if (!buf) {
			break;
		}
		mvpp2_bm_pool_put(port, pool->id, virt_to_phys(buf));
	}
	pool->buf_num += i;

	mvpp2_write(port->priv, MVPP2_POOL_BUF_SIZE_REG(pool->id),
		    ALIGN(skb_bin_data_size(), 1 << MVPP2_POOL_BUF_SIZE_OFFSET));

	for (rxq = 0; rxq < port->priv->nthreads; rxq++)
		mvpp2_rxq_pool_set(port, rxq, pool->id);
	return 0;
}

static inline void mvpp2_interrupts_enable(struct mvpp2_port *port)
{
	int i, sw_thread_mask = 0;

	for (i = 0; i < port->priv->nthreads; i++)
		sw_thread_mask |= BIT(port->qvecs[i].id);

	mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_ENABLE_INTERRUPT(sw_thread_mask));
}

static inline void mvpp2_interrupts_disable(struct mvpp2_port *port)
{
	int i, sw_thread_mask = 0;

	for (i = 0; i < port->priv->nthreads; i++)
		sw_thread_mask |= BIT(port->qvecs[i].id);

	mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_DISABLE_INTERRUPT(sw_thread_mask));
}

static void mvpp2_interrupts_mask(void *arg)
{
	struct mvpp2_port *port = arg;

	mvpp2_thread_write(port->priv, smp_processor_id(),
			   MVPP2_ISR_RX_TX_MASK_REG(port->id), 0);
	mvpp2_thread_write(port->priv, smp_processor_id(),
			   MVPP2_ISR_RX_ERR_CAUSE_REG(port->id), 0);
}

static void mvpp2_interrupts_unmask(void *arg)
{
	struct mvpp2_port *port = arg;
	u32 val = 0xff | MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;

	mvpp2_thread_write(port->priv, smp_processor_id(),
			   MVPP2_ISR_RX_TX_MASK_REG(port->id), val);
	mvpp2_thread_write(port->priv, smp_processor_id(),
			   MVPP2_ISR_RX_ERR_CAUSE_REG(port->id),
			   MVPP2_ISR_RX_ERR_CAUSE_NONOCC_MASK);
}

static void mvpp22_gop_init_rgmii(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	u32 val;

	regmap_read(priv->sysctrl_base, GENCONF_PORT_CTRL0, &val);
	val |= GENCONF_PORT_CTRL0_BUS_WIDTH_SELECT;
	regmap_write(priv->sysctrl_base, GENCONF_PORT_CTRL0, val);

	regmap_read(priv->sysctrl_base, GENCONF_CTRL0, &val);
	if (port->gop_id == 2)
		val |= GENCONF_CTRL0_PORT0_RGMII;
	else if (port->gop_id == 3)
		val |= GENCONF_CTRL0_PORT1_RGMII_MII;
	regmap_write(priv->sysctrl_base, GENCONF_CTRL0, val);
}

static void mvpp22_gop_init_sgmii(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	u32 val;

	regmap_read(priv->sysctrl_base, GENCONF_PORT_CTRL0, &val);
	val |= GENCONF_PORT_CTRL0_BUS_WIDTH_SELECT |
	       GENCONF_PORT_CTRL0_RX_DATA_SAMPLE;
	regmap_write(priv->sysctrl_base, GENCONF_PORT_CTRL0, val);

	if (port->gop_id > 1) {
		regmap_read(priv->sysctrl_base, GENCONF_CTRL0, &val);
		if (port->gop_id == 2)
			val &= ~GENCONF_CTRL0_PORT0_RGMII;
		else if (port->gop_id == 3)
			val &= ~GENCONF_CTRL0_PORT1_RGMII_MII;
		regmap_write(priv->sysctrl_base, GENCONF_CTRL0, val);
	}
}

static void mvpp22_gop_init_xpcs(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	void __iomem *xpcs = priv->iface_base + MVPP22_XPCS_BASE(port->gop_id);
	u32 val;

	/* Reset the XPCS when reconfiguring the lanes */
	val = readl(xpcs + MVPP22_XPCS_CFG0);
	writel(val & ~MVPP22_XPCS_CFG0_RESET_DIS, xpcs + MVPP22_XPCS_CFG0);

	/* XPCS */
	val = readl(xpcs + MVPP22_XPCS_CFG0);
	val &= ~(MVPP22_XPCS_CFG0_PCS_MODE(0x3) |
		 MVPP22_XPCS_CFG0_ACTIVE_LANE(0x3));
	val |= MVPP22_XPCS_CFG0_ACTIVE_LANE(2);
	writel(val, xpcs + MVPP22_XPCS_CFG0);

	/* Release lanes from reset */
	val = readl(xpcs + MVPP22_XPCS_CFG0);
	writel(val | MVPP22_XPCS_CFG0_RESET_DIS, xpcs + MVPP22_XPCS_CFG0);

}

static void mvpp22_gop_init_mpcs(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	void __iomem *mpcs = priv->iface_base + MVPP22_MPCS_BASE(port->gop_id);
	u32 val;

	/* MPCS */
	val = readl(mpcs + MVPP22_MPCS_CTRL);
	val &= ~MVPP22_MPCS_CTRL_FWD_ERR_CONN;
	writel(val, mpcs + MVPP22_MPCS_CTRL);

	val = readl(mpcs + MVPP22_MPCS_CLK_RESET);
	val &= ~(MVPP22_MPCS_CLK_RESET_DIV_RATIO(0x7) | MAC_CLK_RESET_MAC |
		 MAC_CLK_RESET_SD_RX | MAC_CLK_RESET_SD_TX);
	val |= MVPP22_MPCS_CLK_RESET_DIV_RATIO(1);
	writel(val, mpcs + MVPP22_MPCS_CLK_RESET);

	val &= ~MVPP22_MPCS_CLK_RESET_DIV_SET;
	val |= MAC_CLK_RESET_MAC | MAC_CLK_RESET_SD_RX | MAC_CLK_RESET_SD_TX;
	writel(val, mpcs + MVPP22_MPCS_CLK_RESET);
}

static void mvpp22_gop_fca_enable_periodic(struct mvpp2_port *port, bool en)
{
	struct mvpp2 *priv = port->priv;
	void __iomem *fca = priv->iface_base + MVPP22_FCA_BASE(port->gop_id);
	u32 val;

	val = readl(fca + MVPP22_FCA_CONTROL_REG);
	val &= ~MVPP22_FCA_ENABLE_PERIODIC;
	if (en)
		val |= MVPP22_FCA_ENABLE_PERIODIC;
	writel(val, fca + MVPP22_FCA_CONTROL_REG);
}

static void mvpp22_gop_fca_set_timer(struct mvpp2_port *port, u32 timer)
{
	struct mvpp2 *priv = port->priv;
	void __iomem *fca = priv->iface_base + MVPP22_FCA_BASE(port->gop_id);
	u32 lsb, msb;

	lsb = timer & MVPP22_FCA_REG_MASK;
	msb = timer >> MVPP22_FCA_REG_SIZE;

	writel(lsb, fca + MVPP22_PERIODIC_COUNTER_LSB_REG);
	writel(msb, fca + MVPP22_PERIODIC_COUNTER_MSB_REG);
}

/* Set Flow Control timer x140 faster than pause quanta to ensure that link
 * partner won't send taffic if port in XOFF mode.
 */
static void mvpp22_gop_fca_set_periodic_timer(struct mvpp2_port *port)
{
	u32 timer;

	timer = (port->priv->tclk / (USEC_PER_SEC * FC_CLK_DIVIDER))
		* FC_QUANTA;

	mvpp22_gop_fca_enable_periodic(port, false);

	mvpp22_gop_fca_set_timer(port, timer);

	mvpp22_gop_fca_enable_periodic(port, true);
}

static int mvpp22_gop_init(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	u32 val;

	if (!priv->sysctrl_base)
		return 0;

	switch (port->phy_interface) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		if (port->gop_id == 0)
			goto invalid_conf;
		mvpp22_gop_init_rgmii(port);
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_2500BASEX:
	case PHY_INTERFACE_MODE_2500BASET:
		mvpp22_gop_init_sgmii(port);
		break;
	case PHY_INTERFACE_MODE_RXAUI:
		if (port->gop_id != 0)
			goto invalid_conf;
		mvpp22_gop_init_xpcs(port);
		break;
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_5GKR:
		if (!port->has_xlg_mac)
			goto invalid_conf;
		mvpp22_gop_init_mpcs(port);
		break;
	case PHY_INTERFACE_MODE_INTERNAL:
		return 0;
	default:
		goto unsupported_conf;
	}

	regmap_read(priv->sysctrl_base, GENCONF_PORT_CTRL1, &val);
	val |= GENCONF_PORT_CTRL1_RESET(port->gop_id) |
	       GENCONF_PORT_CTRL1_EN(port->gop_id);
	regmap_write(priv->sysctrl_base, GENCONF_PORT_CTRL1, val);

	regmap_read(priv->sysctrl_base, GENCONF_PORT_CTRL0, &val);
	val |= GENCONF_PORT_CTRL0_CLK_DIV_PHASE_CLR;
	regmap_write(priv->sysctrl_base, GENCONF_PORT_CTRL0, val);

	regmap_read(priv->sysctrl_base, GENCONF_SOFT_RESET1, &val);
	val |= GENCONF_SOFT_RESET1_GOP;
	regmap_write(priv->sysctrl_base, GENCONF_SOFT_RESET1, val);

	mvpp22_gop_fca_set_periodic_timer(port);

unsupported_conf:
	return 0;

invalid_conf:
	netdev_err(port->sw.dev, "Invalid port configuration\n");
	return -EINVAL;
}

static void mvpp22_gop_unmask_irq(struct mvpp2_port *port)
{
	u32 val;

	if (phy_interface_mode_is_rgmii(port->phy_interface) ||
	    phy_interface_mode_is_8023z(port->phy_interface) ||
	    port->phy_interface == PHY_INTERFACE_MODE_SGMII ||
	    port->phy_interface == PHY_INTERFACE_MODE_2500BASET) {
		/* Enable the GMAC link status irq for this port */
		val = readl(port->base + MVPP22_GMAC_INT_SUM_MASK);
		val |= MVPP22_GMAC_INT_SUM_MASK_LINK_STAT;
		writel(val, port->base + MVPP22_GMAC_INT_SUM_MASK);
	}

	if (port->has_xlg_mac) {
		/* Enable the XLG/GIG irqs for this port */
		val = readl(port->base + MVPP22_XLG_EXT_INT_MASK);
		if (port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
		    port->phy_interface == PHY_INTERFACE_MODE_5GKR ||
		    port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)
			val |= MVPP22_XLG_EXT_INT_MASK_XLG;
		else
			val |= MVPP22_XLG_EXT_INT_MASK_GIG;
		writel(val, port->base + MVPP22_XLG_EXT_INT_MASK);
	}
}

static void mvpp22_gop_mask_irq(struct mvpp2_port *port)
{
	u32 val;

	if (port->has_xlg_mac) {
		val = readl(port->base + MVPP22_XLG_EXT_INT_MASK);
		val &= ~(MVPP22_XLG_EXT_INT_MASK_XLG |
			 MVPP22_XLG_EXT_INT_MASK_GIG);
		writel(val, port->base + MVPP22_XLG_EXT_INT_MASK);
	}

	if (phy_interface_mode_is_rgmii(port->phy_interface) ||
	    phy_interface_mode_is_8023z(port->phy_interface) ||
	    port->phy_interface == PHY_INTERFACE_MODE_SGMII ||
	    port->phy_interface == PHY_INTERFACE_MODE_2500BASET) {
		val = readl(port->base + MVPP22_GMAC_INT_SUM_MASK);
		val &= ~MVPP22_GMAC_INT_SUM_MASK_LINK_STAT;
		writel(val, port->base + MVPP22_GMAC_INT_SUM_MASK);
	}
}

static void mvpp22_gop_setup_irq(struct mvpp2_port *port)
{
	u32 val;

	if (port->has_xlg_mac) {
		val = readl(port->base + MVPP22_XLG_INT_MASK);
		val |= MVPP22_XLG_INT_MASK_LINK;
		writel(val, port->base + MVPP22_XLG_INT_MASK);
	}

	mvpp22_gop_unmask_irq(port);
}

static void mvpp2_port_enable(struct mvpp2_port *port)
{
	u32 val;

	if (port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)
		return;

	if (port->has_xlg_mac &&
	    (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
	     port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_5GKR)) {
		val = readl(port->base + MVPP22_XLG_CTRL0_REG);
		val |= MVPP22_XLG_CTRL0_PORT_EN |
		       MVPP22_XLG_CTRL0_MAC_RESET_DIS;
		val &= ~MVPP22_XLG_CTRL0_MIB_CNT_DIS;
		writel(val, port->base + MVPP22_XLG_CTRL0_REG);
	} else {
		val = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
		val |= MVPP2_GMAC_PORT_EN_MASK;
		val |= MVPP2_GMAC_MIB_CNTR_EN_MASK;
		writel(val, port->base + MVPP2_GMAC_CTRL_0_REG);
	}
}

static void mvpp2_port_disable(struct mvpp2_port *port)
{
	u32 val;

	if (port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)
		return;

	if (port->has_xlg_mac &&
	    (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
	     port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_5GKR)) {
		val = readl(port->base + MVPP22_XLG_CTRL0_REG);
		val &= ~MVPP22_XLG_CTRL0_PORT_EN;
		writel(val, port->base + MVPP22_XLG_CTRL0_REG);
	} else {
		val = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
		val &= ~(MVPP2_GMAC_PORT_EN_MASK);
		writel(val, port->base + MVPP2_GMAC_CTRL_0_REG);
	}
}

/* Set IEEE 802.3x Flow Control Xon Packet Transmission Mode */
static void mvpp2_port_periodic_xon_disable(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_1_REG) &
		    ~MVPP2_GMAC_PERIODIC_XON_EN_MASK;
	writel(val, port->base + MVPP2_GMAC_CTRL_1_REG);
}

static u64 mvpp2_read_count(struct mvpp2_port *port, unsigned offset, bool is64) {
	u64 val = readl(port->stats_base + offset);
	if (is64) {
		val += (u64)readl(port->stats_base + offset + 4) << 32;
	}
	return val;
}

static void mvpp2_hw_get_stats(struct mvpp2_port *port, struct eth_stats *s) {
	s->rx_byte += mvpp2_read_count(port, MVPP2_MIB_GOOD_OCTETS_RCVD, true);
	s->rx_fcs_err += mvpp2_read_count(port, MVPP2_MIB_BAD_CRC_EVENT, false);
	s->tx_underrun += mvpp2_read_count(port, MVPP2_MIB_CRC_ERRORS_SENT, false);
	s->rx_unicast += mvpp2_read_count(port, MVPP2_MIB_UNICAST_FRAMES_RCVD, false);
	s->rx_broadcast += mvpp2_read_count(port, MVPP2_MIB_BROADCAST_FRAMES_RCVD, false);
	s->rx_multicast += mvpp2_read_count(port, MVPP2_MIB_MULTICAST_FRAMES_RCVD, false);
	s->tx_rx_64 += mvpp2_read_count(port, MVPP2_MIB_FRAMES_64_OCTETS, false);
	s->tx_rx_65_127 += mvpp2_read_count(port, MVPP2_MIB_FRAMES_65_TO_127_OCTETS, false);
	s->tx_rx_128_255 += mvpp2_read_count(port, MVPP2_MIB_FRAMES_128_TO_255_OCTETS, false);
	s->tx_rx_256_511 += mvpp2_read_count(port, MVPP2_MIB_FRAMES_256_TO_511_OCTETS, false);
	s->tx_rx_512_1023 += mvpp2_read_count(port, MVPP2_MIB_FRAMES_512_TO_1023_OCTETS, false);
	s->tx_rx_1024_max += mvpp2_read_count(port, MVPP2_MIB_FRAMES_1024_TO_MAX_OCTETS, false);
	s->tx_byte += mvpp2_read_count(port, MVPP2_MIB_GOOD_OCTETS_SENT, true);
	s->tx_unicast += mvpp2_read_count(port, MVPP2_MIB_UNICAST_FRAMES_SENT, false);
	s->tx_broadcast += mvpp2_read_count(port, MVPP2_MIB_BROADCAST_FRAMES_SENT, false);
	s->tx_multicast += mvpp2_read_count(port, MVPP2_MIB_MULTICAST_FRAMES_SENT, false);
	s->tx_pause += mvpp2_read_count(port, MVPP2_MIB_FC_SENT, false);
	s->rx_pause += mvpp2_read_count(port, MVPP2_MIB_FC_RCVD, false);
	s->rx_overrun += mvpp2_read_count(port, MVPP2_MIB_RX_FIFO_OVERRUN, false);
	s->rx_too_short += mvpp2_read_count(port, MVPP2_MIB_UNDERSIZE_RCVD, false);
	s->rx_fragment += mvpp2_read_count(port, MVPP2_MIB_FRAGMENTS_ERR_RCVD, false);
	s->rx_too_long += mvpp2_read_count(port, MVPP2_MIB_OVERSIZE_RCVD, false);
	s->rx_jabber += mvpp2_read_count(port, MVPP2_MIB_JABBER_RCVD, false);
	s->tx_collision += mvpp2_read_count(port, MVPP2_MIB_COLLISION, false);
	s->tx_late_col += mvpp2_read_count(port, MVPP2_MIB_LATE_COLLISION, false);

	s->rx_drop += mvpp2_read(port->priv, MVPP2_OVERRUN_DROP_REG(port->id));
	s->rx_drop += mvpp2_read(port->priv, MVPP2_CLS_DROP_REG(port->id));

	int queue = port->first_rxq;
	while (queue < (port->first_rxq + port->priv->nthreads)) {
		mvpp2_write(port->priv, MVPP2_CNT_IDX_REG, queue++);

		s->rx_drop += mvpp2_read(port->priv, MVPP2_RX_PKT_FULLQ_DROP_REG);
		s->rx_drop += mvpp2_read(port->priv, MVPP2_RX_PKT_EARLY_DROP_REG);
		s->rx_drop += mvpp2_read(port->priv, MVPP2_RX_PKT_BM_DROP_REG);
	}
}

static void mvpp2_ethtool_get_stats(struct net_device *dev,
				    struct ethtool_stats *estats, u64 *stats) {
	struct mvpp2_port *port = netdev_priv(dev);
	fast_path_get_eth_stats(dev, &port->sw.xstats[0]);
	mvpp2_hw_get_stats(port, &port->sw.xstats[0]);
	switch_get_ethtool_stats(dev, estats, stats);
}

static void mvpp2_port_reset(struct mvpp2_port *port)
{
	u32 val;

	/* Read the GOP statistics to reset the hardware counters */
	struct eth_stats x;
	mvpp2_hw_get_stats(port, &x);

	val = readl(port->base + MVPP2_GMAC_CTRL_2_REG) |
	      MVPP2_GMAC_PORT_RESET_MASK;
	writel(val, port->base + MVPP2_GMAC_CTRL_2_REG);

	if (port->has_xlg_mac) {
		/* Set the XLG MAC in reset */
		val = readl(port->base + MVPP22_XLG_CTRL0_REG) &
		      ~MVPP22_XLG_CTRL0_MAC_RESET_DIS;
		writel(val, port->base + MVPP22_XLG_CTRL0_REG);

		while (readl(port->base + MVPP22_XLG_CTRL0_REG) &
		       MVPP22_XLG_CTRL0_MAC_RESET_DIS)
			continue;
	}
}

/* Change maximum receive size of the port */
static inline void mvpp2_gmac_max_rx_size_set(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
	val &= ~MVPP2_GMAC_MAX_RX_SIZE_MASK;
	val |= ((port->sw.dev->l2mtu + ETH_HLEN + ETH_FCS_LEN) / 2) << MVPP2_GMAC_MAX_RX_SIZE_OFFS;
	writel(val, port->base + MVPP2_GMAC_CTRL_0_REG);
}

/* Change maximum receive size of the port */
static inline void mvpp2_xlg_max_rx_size_set(struct mvpp2_port *port)
{
	u32 val;

	val =  readl(port->base + MVPP22_XLG_CTRL1_REG);
	val &= ~MVPP22_XLG_CTRL1_FRAMESIZELIMIT_MASK;
	val |= ((port->sw.dev->l2mtu + ETH_HLEN + ETH_FCS_LEN) / 2) << MVPP22_XLG_CTRL1_FRAMESIZELIMIT_OFFS;
	writel(val, port->base + MVPP22_XLG_CTRL1_REG);
}

static void mvpp2_gmac_tx_fifo_configure(struct mvpp2_port *port)
{
	u32 val, tx_fifo_min_th;
	u8 low_wm, hi_wm;

	tx_fifo_min_th = MVPP2_GMAC_TX_FIFO_MIN_TH;
	low_wm = MVPP2_GMAC_TX_FIFO_LOW_WM;
	hi_wm = MVPP2_GMAC_TX_FIFO_HI_WM;

	/* Update TX FIFO MIN Threshold */
	val = readl(port->base + MVPP2_GMAC_PORT_FIFO_CFG_1_REG);
	val &= ~MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK;
	val |= tx_fifo_min_th;
	writel(val, port->base + MVPP2_GMAC_PORT_FIFO_CFG_1_REG);

	/* Update TX FIFO levels of assertion/deassertion
	 * of p2mem_ready_signal, which indicates readiness
	 * for fetching the data from DRAM.
	 */
	val = readl(port->base + MVPP2_GMAC_PORT_FIFO_CFG_0_REG);
	val &= ~MVPP2_GMAC_TX_FIFO_WM_MASK;
	val |= (low_wm << MVPP2_GMAC_TX_FIFO_WM_LOW_OFFSET) | hi_wm;
	writel(val, port->base + MVPP2_GMAC_PORT_FIFO_CFG_0_REG);
}

static void mvpp2_defaults_set(struct mvpp2_port *port)
{
	int tx_port_num, val, queue, ptxq, lrxq;

	if (phy_interface_mode_is_rgmii(port->phy_interface) ||
	    port->phy_interface == PHY_INTERFACE_MODE_SGMII ||
	    port->phy_interface == PHY_INTERFACE_MODE_1000BASEX ||
	    port->phy_interface == PHY_INTERFACE_MODE_2500BASEX)
		mvpp2_gmac_tx_fifo_configure(port);

	/* Disable Legacy WRR, Disable EJP, Release from reset */
	tx_port_num = MVPP2_MAX_TCONT + port->id;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG,
		    tx_port_num);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_CMD_1_REG, 0);

	/* Set TXQ scheduling to Round-Robin */
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_FIXED_PRIO_REG, 0);

	/* Close bandwidth for all queues */
	for (queue = 0; queue < MVPP2_MAX_TXQ; queue++) {
		ptxq = (MVPP2_MAX_TCONT + port->id) * MVPP2_MAX_TXQ + queue;
		mvpp2_write(port->priv,
			    MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(ptxq), 0);
	}

	/* Set refill period to 1 usec, refill tokens
	 * and bucket size to maximum
	 */
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PERIOD_REG,
		    port->priv->tclk / USEC_PER_SEC);
	val = mvpp2_read(port->priv, MVPP2_TXP_SCHED_REFILL_REG);
	val &= ~MVPP2_TXP_REFILL_PERIOD_ALL_MASK;
	val |= MVPP2_TXP_REFILL_PERIOD_MASK(1);
	val |= MVPP2_TXP_REFILL_TOKENS_ALL_MASK;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_REFILL_REG, val);
	val = MVPP2_TXP_TOKEN_SIZE_MAX;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);

	/* Set MaximumLowLatencyPacketSize value to 256 */
	mvpp2_write(port->priv, MVPP2_RX_CTRL_REG(port->id),
		    MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK |
		    MVPP2_RX_LOW_LATENCY_PKT_SIZE(256));

/*
	val = mvpp2_read(port->priv, MVPP2_MH_REG(port->id));
	printk("MVPP2_MH_REG(%u): %08x\n", port->id, val);
	val &= ~(BIT(4) | BIT(5));
	if (port->sw.ops && port->sw.ops->overhead_bytes == 4) {
	    val |= MVPP2_DSA_NON_EXTENDED;
	}
	printk("MVPP2_MH_REG(%u): %08x\n", port->id, val);
	mvpp2_write(port->priv, MVPP2_MH_REG(port->id), val);
*/

	/* Enable Rx cache snoop */
	for (lrxq = 0; lrxq < port->priv->nthreads; lrxq++) {
		queue = port->rxqs[lrxq].id;
		val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(queue));
//		printk("rxq%u config %08x\n", lrxq, val);
		val |= BIT(16) | (3 << 12);
//		val |= MVPP2_SNOOP_PKT_SIZE_MASK |
//			   MVPP2_SNOOP_BUF_HDR_MASK;
//		printk("rxq%u config %08x\n", lrxq, val);
		mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(queue), val);
	}

	mvpp2_interrupts_disable(port);
}

/* Enable/disable receiving packets */
static void mvpp2_ingress_enable(struct mvpp2_port *port)
{
	u32 val;
	int lrxq, queue;

	for (lrxq = 0; lrxq < port->priv->nthreads; lrxq++) {
		queue = port->rxqs[lrxq].id;
		val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(queue));
		val &= ~MVPP2_RXQ_DISABLE_MASK;
		mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(queue), val);
	}
}

static void mvpp2_ingress_disable(struct mvpp2_port *port)
{
	u32 val;
	int lrxq, queue;

	for (lrxq = 0; lrxq < port->priv->nthreads; lrxq++) {
		queue = port->rxqs[lrxq].id;
		val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(queue));
		val |= MVPP2_RXQ_DISABLE_MASK;
		mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(queue), val);
	}
}

static void mvpp2_egress_enable(struct mvpp2_port *port)
{
	u32 qmap;
	int queue;
	int tx_port_num = MVPP2_MAX_TCONT + port->id;

	qmap = 0;
	for (queue = 0; queue < port->priv->nthreads; queue++) {
		struct mvpp2_queue *txq = &port->txqs[queue];

		if (txq->descs)
			qmap |= (1 << queue);
	}

	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG, qmap);
}

static void mvpp2_egress_disable(struct mvpp2_port *port)
{
	u32 reg_data;
	int delay;
	int tx_port_num = MVPP2_MAX_TCONT + port->id;

	/* Issue stop command for active channels only */
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	reg_data = (mvpp2_read(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG)) &
		    MVPP2_TXP_SCHED_ENQ_MASK;
	if (reg_data != 0)
		mvpp2_write(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG,
			    (reg_data << MVPP2_TXP_SCHED_DISQ_OFFSET));

	/* Wait for all Tx activity to terminate. */
	delay = 0;
	do {
		if (delay >= MVPP2_TX_DISABLE_TIMEOUT_MSEC) {
			netdev_warn(port->sw.dev,
				    "Tx stop timed out, status=0x%08x\n",
				    reg_data);
			break;
		}
		mdelay(1);
		delay++;

		/* Check port TX Command register that all
		 * Tx queues are stopped
		 */
		reg_data = mvpp2_read(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG);
	} while (reg_data & MVPP2_TXP_SCHED_ENQ_MASK);
}

static void mvpp2_rxq_offset_set(struct mvpp2_port *port,
				 int prxq, int offset)
{
	u32 val;

	/* Convert offset from bytes to units of 32 bytes */
	offset = offset >> 5;

	val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~MVPP2_RXQ_PACKET_OFFSET_MASK;

	/* Offset is in */
	val |= ((offset << MVPP2_RXQ_PACKET_OFFSET_OFFS) &
		    MVPP2_RXQ_PACKET_OFFSET_MASK);

	mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

/* Called through on_each_cpu(), so runs on all CPUs, with migration
 * disabled, therefore using smp_processor_id() is OK.
 */
static void mvpp2_txq_sent_counter_clear(void *arg)
{
	struct mvpp2_port *port = arg;
	int queue;

	for (queue = 0; queue < port->priv->nthreads; queue++) {
		int id = port->txqs[queue].id;

		mvpp2_thread_read(port->priv, smp_processor_id(),
				  MVPP2_TXQ_SENT_REG(id));
	}
}

/* Set max sizes for Tx queues */
static void mvpp2_txp_max_tx_size_set(struct mvpp2_port *port)
{
	u32	val, size, mtu;
	int	txq, tx_port_num;

	mtu = port->sw.dev->l2mtu * 8;
	if (mtu > MVPP2_TXP_MTU_MAX)
		mtu = MVPP2_TXP_MTU_MAX;

	/* WA for wrong Token bucket update: Set MTU value = 3*real MTU value */
	mtu = 3 * mtu;

	/* Indirect access to registers */
	tx_port_num = MVPP2_MAX_TCONT + port->id;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

	/* Set MTU */
	val = mvpp2_read(port->priv, MVPP2_TXP_SCHED_MTU_REG);
	val &= ~MVPP2_TXP_MTU_MAX;
	val |= mtu;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_MTU_REG, val);

	/* TXP token size and all TXQs token size must be larger that MTU */
	val = mvpp2_read(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG);
	size = val & MVPP2_TXP_TOKEN_SIZE_MAX;
	if (size < mtu) {
		size = mtu;
		val &= ~MVPP2_TXP_TOKEN_SIZE_MAX;
		val |= size;
		mvpp2_write(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);
	}

	for (txq = 0; txq < port->priv->nthreads; txq++) {
		val = mvpp2_read(port->priv,
				 MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq));
		size = val & MVPP2_TXQ_TOKEN_SIZE_MAX;

		if (size < mtu) {
			size = mtu;
			val &= ~MVPP2_TXQ_TOKEN_SIZE_MAX;
			val |= size;
			mvpp2_write(port->priv,
				    MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq),
				    val);
		}
	}
}

/* Routine set the number of non-occupied descriptors threshold that change
 * interrupt error cause polled by FW Flow Control
 */
void mvpp2_set_rxq_free_tresh(struct mvpp2_port *port,
			      struct mvpp2_queue *rxq)
{
	u32 val;

	mvpp2_write(port->priv, MVPP2_RXQ_NUM_REG, rxq->id);

	val = mvpp2_read(port->priv, MVPP2_RXQ_THRESH_REG);
	val &= ~MVPP2_RXQ_NON_OCCUPIED_MASK;
	val |= MSS_THRESHOLD_STOP << MVPP2_RXQ_NON_OCCUPIED_OFFSET;
	mvpp2_write(port->priv, MVPP2_RXQ_THRESH_REG, val);
}

static inline void mvpp2_tx_pkts_coal_set_txqs(struct mvpp2_port *port,
					       int cpu, u32 val)
{
	int queue;
	val <<= MVPP2_TXQ_THRESH_OFFSET;

	for (queue = 0; queue < port->priv->nthreads; queue++) {
		struct mvpp2_queue *txq = &port->txqs[queue];
		mvpp2_thread_write(port->priv, cpu, MVPP2_TXQ_NUM_REG, txq->id);
		mvpp2_thread_write(port->priv, cpu, MVPP2_TXQ_THRESH_REG, val);
	}
}

static u32 mvpp2_usec_to_cycles(u32 usec, unsigned long clk_hz)
{
	u64 tmp = (u64)clk_hz * usec;

	do_div(tmp, USEC_PER_SEC);

	return tmp > U32_MAX ? U32_MAX : tmp;
}

static u32 mvpp2_cycles_to_usec(u32 cycles, unsigned long clk_hz)
{
	u64 tmp = (u64)cycles * USEC_PER_SEC;

	do_div(tmp, clk_hz);

	return tmp > U32_MAX ? U32_MAX : tmp;
}

/* Set the time delay in usec before Rx interrupt */
static void mvpp2_rx_time_coal_set(struct mvpp2_port *port,
				   struct mvpp2_queue *rxq)
{
	unsigned long freq = port->priv->tclk;
	u32 val = mvpp2_usec_to_cycles(MVPP2_RX_COAL_USEC, freq);

	if (val > MVPP2_MAX_ISR_RX_THRESHOLD) {
		val = MVPP2_MAX_ISR_RX_THRESHOLD;
	}

	mvpp2_write(port->priv, MVPP2_ISR_RX_THRESHOLD_REG(rxq->id), val);
}

static void mvpp2_tx_time_coal_set(struct mvpp2_port *port)
{
	unsigned long freq = port->priv->tclk;
	u32 val = mvpp2_usec_to_cycles(port->tx_time_coal, freq);

	if (val > MVPP2_MAX_ISR_TX_THRESHOLD) {
		port->tx_time_coal =
			mvpp2_cycles_to_usec(MVPP2_MAX_ISR_TX_THRESHOLD, freq);

		/* re-evaluate to get actual register value */
		val = mvpp2_usec_to_cycles(port->tx_time_coal, freq);
	}

	mvpp2_write(port->priv, MVPP2_ISR_TX_THRESHOLD_REG(port->id), val);
}

static void mvpp2_txq_done(struct mvpp2_port *port, struct mvpp2_queue *txq)
{
	unsigned tx_done = mvpp2_thread_read_relaxed(port->priv, smp_processor_id(), MVPP2_TXQ_SENT_REG(txq->id));
	tx_done = (tx_done & MVPP2_TRANSMITTED_COUNT_MASK) >> MVPP2_TRANSMITTED_COUNT_OFFSET;
	if (!tx_done)
		return;
	unsigned i;
	tx_done &= txq->mask;
//	BUG_ON(tx_done > txq->count);
	for (i = 0; i < tx_done; i++) {
		__skb_bin_put_buffer(txq->bufs[txq->get_index]);
		txq->get_index = (txq->get_index + 1) & txq->mask;
	}
	txq->count -= tx_done;
	if (port->sw.ops) {
	    switch_tx(&port->sw, txq->log_id, tx_done);
	}

//	struct netdev_queue *nq = netdev_get_tx_queue(port->sw.dev, txq->log_id);
//	if (unlikely(netif_tx_queue_stopped(nq))) {
//		if (txq->count < txq->size - FP_NAPI_BUDGET) {
//			netif_tx_wake_queue(nq);
//		}
//	}
}

static int mvpp2_aggr_txq_init(struct platform_device *pdev,
			       struct mvpp2_queue *aggr_txq,
			       unsigned int thread, struct mvpp2 *priv)
{
	aggr_txq->descs = dma_alloc_coherent(&pdev->dev,
				MVPP2_AGGR_TXQ_SIZE * MVPP2_DESC_ALIGNED_SIZE,
				&aggr_txq->descs_dma, GFP_KERNEL);
	if (!aggr_txq->descs)
		return -ENOMEM;

	int i;
	for (i = 0; i < aggr_txq->size; ++i) {
		struct mvpp2_desc *tx_desc = &aggr_txq->descs[i];
		tx_desc->phys_txq = (MVPP2_MAX_TCONT + 0) * MVPP2_MAX_TXQ + aggr_txq->id;
		tx_desc->command_status = cpu_to_le32(MVPP2_TXD_L4_CSUM_NOT | MVPP2_TXD_IP_CSUM_DISABLE | MVPP2_TXD_F_DESC | MVPP2_TXD_L_DESC);
	}
	aggr_txq->put_index = mvpp2_read(priv, MVPP2_AGGR_TXQ_INDEX_REG(thread));

	mvpp2_write(priv, MVPP2_AGGR_TXQ_DESC_ADDR_REG(thread),
		    aggr_txq->descs_dma >> MVPP22_AGGR_TXQ_DESC_ADDR_OFFS);
	mvpp2_write(priv, MVPP2_AGGR_TXQ_DESC_SIZE_REG(thread),
		    MVPP2_AGGR_TXQ_SIZE);
	return 0;
}

static int mvpp2_rxq_init(struct mvpp2_port *port,
			  struct mvpp2_queue *rxq)

{
	unsigned int thread;
	u32 rxq_dma;

	rxq->size = port->rx_ring_size;
	rxq->mask = rxq->size - 1;

	rxq->descs = dma_alloc_coherent(port->sw.dev->dev.parent,
					rxq->size * MVPP2_DESC_ALIGNED_SIZE,
					&rxq->descs_dma, GFP_KERNEL);
	if (!rxq->descs)
		return -ENOMEM;

	rxq->pending = 0;

	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id), 0);

	thread = smp_processor_id();
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_NUM_REG, rxq->id);
	rxq_dma = rxq->descs_dma >> MVPP22_DESC_ADDR_OFFS;
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_DESC_ADDR_REG, rxq_dma);
//	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_DESC_SIZE_REG, rxq->size);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_DESC_SIZE_REG, rxq->size | BIT(16));
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_INDEX_REG, 0);

	mvpp2_rxq_offset_set(port, rxq->id, NET_SKB_PAD);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_NUM_REG, rxq->id);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_THRESH_REG, MVPP2_RX_COAL_PKTS);
	mvpp2_rx_time_coal_set(port, rxq);
	mvpp2_set_rxq_free_tresh(port, rxq);
	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_UPDATE_REG(rxq->id),
		    rxq->size << MVPP2_RXQ_NUM_NEW_OFFSET);
	return 0;
}

static void mvpp2_rxq_deinit(struct mvpp2_port *port,
			     struct mvpp2_queue *rxq)
{
	unsigned int thread;
	int rx_received, i;

	rxq->pending = 0;
	rx_received = mvpp2_read(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id)) & MVPP2_RXQ_OCCUPIED_MASK;
	for (i = 0; i < rx_received; i++) {
		struct mvpp2_desc *rx_desc = rxq->descs + rxq->get_index;
		mvpp2_bm_pool_put(port, 0, le64_to_cpu(rx_desc->buf_dma_addr) & MVPP2_DESC_DMA_MASK);
		rxq->get_index = (rxq->get_index + 1) & rxq->mask;
	}
	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_UPDATE_REG(rxq->id),
		    rx_received | (rx_received << MVPP2_RXQ_NUM_NEW_OFFSET));

	if (rxq->descs)
		dma_free_coherent(port->sw.dev->dev.parent,
				  rxq->size * MVPP2_DESC_ALIGNED_SIZE,
				  rxq->descs,
				  rxq->descs_dma);

	rxq->descs             = NULL;
	rxq->get_index = 0;
	rxq->descs_dma         = 0;

	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id), 0);
	thread = smp_processor_id();
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_NUM_REG, rxq->id);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_DESC_ADDR_REG, 0);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_DESC_SIZE_REG, 0);
}

static void mvpp2_rxq_disable_all(struct mvpp2 *priv)
{
	int i;
	u32 val;

	for (i = 0; i < MVPP2_RXQ_MAX_NUM; i++) {
		val = mvpp2_read(priv, MVPP2_RXQ_CONFIG_REG(i));
		val |= MVPP2_RXQ_DISABLE_MASK;
		mvpp2_write(priv, MVPP2_RXQ_CONFIG_REG(i), val);
	}
}

static int mvpp2_txq_init(struct mvpp2_port *port,
			  struct mvpp2_queue *txq)
{
	u32 val;
	unsigned int thread;
	int desc, desc_per_txq, tx_port_num;

	txq->size = port->tx_ring_size;
	txq->mask = txq->size - 1;

	txq->descs = dma_alloc_coherent(port->sw.dev->dev.parent,
				txq->size * MVPP2_DESC_ALIGNED_SIZE,
				&txq->descs_dma, GFP_KERNEL);
	if (!txq->descs)
		return -ENOMEM;

	/* Set Tx descriptors queue starting address - indirect access */
	thread = smp_processor_id();
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_NUM_REG, txq->id);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_DESC_ADDR_REG,
			   txq->descs_dma);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_DESC_SIZE_REG,
			   txq->size & MVPP2_TXQ_DESC_SIZE_MASK);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_INDEX_REG, 0);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_RSVD_CLR_REG,
			   txq->id << MVPP2_TXQ_RSVD_CLR_OFFSET);
	val = mvpp2_thread_read(port->priv, thread, MVPP2_TXQ_PENDING_REG);
	val &= ~MVPP2_TXQ_PENDING_MASK;
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_PENDING_REG, val);

	/* Calculate base address in prefetch buffer. We reserve 16 descriptors
	 * for each existing TXQ.
	 * TCONTS for PON port must be continuous from 0 to MVPP2_MAX_TCONT
	 * GBE ports assumed to be continuous from 0 to MVPP2_MAX_PORTS
	 */
	desc_per_txq = 16;
	desc = (port->id * MVPP2_MAX_TXQ * desc_per_txq) +
	       (txq->log_id * desc_per_txq);

	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_PREF_BUF_REG,
			   MVPP2_PREF_BUF_PTR(desc) | MVPP2_PREF_BUF_SIZE_16 |
			   MVPP2_PREF_BUF_THRESH(desc_per_txq / 2));

	/* WRR / EJP configuration - indirect access */
	tx_port_num = MVPP2_MAX_TCONT + port->id;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

	val = mvpp2_read(port->priv, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id));
	val &= ~MVPP2_TXQ_REFILL_PERIOD_ALL_MASK;
	val |= MVPP2_TXQ_REFILL_PERIOD_MASK(1);
	val |= MVPP2_TXQ_REFILL_TOKENS_ALL_MASK;
	mvpp2_write(port->priv, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id), val);

	val = MVPP2_TXQ_TOKEN_SIZE_MAX;
	mvpp2_write(port->priv, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq->log_id),
		    val);

	txq->bufs = kmalloc_array(txq->size, sizeof(*txq->bufs), GFP_KERNEL);
	if (!txq->bufs)
	    return -ENOMEM;

	txq->count = 0;
	txq->put_index = 0;
	txq->get_index = 0;
	return 0;
}

static void mvpp2_txq_deinit(struct mvpp2_port *port,
			     struct mvpp2_queue *txq)
{
	unsigned int thread;

	kfree(txq->bufs);

	if (txq->descs)
		dma_free_coherent(port->sw.dev->dev.parent,
				  txq->size * MVPP2_DESC_ALIGNED_SIZE,
				  txq->descs, txq->descs_dma);

	txq->descs             = NULL;
	txq->descs_dma         = 0;

	mvpp2_write(port->priv, MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(txq->id), 0);

	thread = smp_processor_id();
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_NUM_REG, txq->id);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_DESC_ADDR_REG, 0);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_DESC_SIZE_REG, 0);
}

static void mvpp2_txq_clean(struct mvpp2_port *port, struct mvpp2_queue *txq)
{
	int delay, pending;
	unsigned int thread = smp_processor_id();
	u32 val;

	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_NUM_REG, txq->id);
	val = mvpp2_thread_read(port->priv, thread, MVPP2_TXQ_PREF_BUF_REG);
	val |= MVPP2_TXQ_DRAIN_EN_MASK;
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_PREF_BUF_REG, val);

	/* Temporarily enable egress for the port.
	 * It is required for releasing all remaining packets.
	 */
	mvpp2_egress_enable(port);

	/* The napi queue has been stopped so wait for all packets
	 * to be transmitted.
	 */
	delay = 0;
	do {
		if (delay >= MVPP2_TX_PENDING_TIMEOUT_MSEC) {
			netdev_warn(port->sw.dev,
				    "port %d: cleaning queue %d timed out\n",
				    port->id, txq->log_id);
			break;
		}
		mdelay(1);
		delay++;

		pending = mvpp2_thread_read(port->priv, thread,
					    MVPP2_TXQ_PENDING_REG);
		pending &= MVPP2_TXQ_PENDING_MASK;
	} while (pending);

	mvpp2_egress_disable(port);

	val &= ~MVPP2_TXQ_DRAIN_EN_MASK;
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_PREF_BUF_REG, val);

	int i;
//	printk("txq%u_clean:%u\n", txq->id, txq->count);
	for (i = 0; i < txq->count; i++) {
		skb_bin_put_buffer(txq->bufs[txq->get_index]);
		txq->get_index = (txq->get_index + 1) & txq->mask;
	}
	txq->count = 0;
	txq->put_index = 0;
	txq->get_index = 0;
}

static void mvpp2_cleanup_txqs(struct mvpp2_port *port)
{
	struct mvpp2_queue *txq;
	int queue;
	u32 val;

	val = mvpp2_read(port->priv, MVPP2_TX_PORT_FLUSH_REG);

	val |= MVPP2_TX_PORT_FLUSH_MASK(port->id);
	mvpp2_write(port->priv, MVPP2_TX_PORT_FLUSH_REG, val);

	for (queue = 0; queue < port->priv->nthreads; queue++) {
		txq = &port->txqs[queue];
		mvpp2_txq_clean(port, txq);
		mvpp2_txq_deinit(port, txq);
	}

	on_each_cpu(mvpp2_txq_sent_counter_clear, port, 1);

	val &= ~MVPP2_TX_PORT_FLUSH_MASK(port->id);
	mvpp2_write(port->priv, MVPP2_TX_PORT_FLUSH_REG, val);
}

static void mvpp2_cleanup_rxqs(struct mvpp2_port *port)
{
	int queue;

	for (queue = 0; queue < port->priv->nthreads; queue++)
		mvpp2_rxq_deinit(port, &port->rxqs[queue]);

	if (port->tx_fc)
		mvpp2_rxq_disable_fc(port);
}

static int mvpp2_setup_rxqs(struct mvpp2_port *port)
{
	int queue, err;

	for (queue = 0; queue < port->priv->nthreads; queue++) {
		err = mvpp2_rxq_init(port, &port->rxqs[queue]);
		if (err)
			goto err_cleanup;
	}

	if (port->tx_fc)
		mvpp2_rxq_enable_fc(port);

	return 0;

err_cleanup:
	mvpp2_cleanup_rxqs(port);
	return err;
}

static int mvpp2_setup_txqs(struct mvpp2_port *port)
{
	struct mvpp2_queue *txq;
	int queue, err;

	for (queue = 0; queue < port->priv->nthreads; queue++) {
		txq = &port->txqs[queue];
		err = mvpp2_txq_init(port, txq);
		if (err)
			goto err_cleanup;
	}

	mvpp2_tx_time_coal_set(port);

	on_each_cpu(mvpp2_txq_sent_counter_clear, port, 1);
	return 0;

err_cleanup:
	mvpp2_cleanup_txqs(port);
	return err;
}

static int is_isr_enabled(struct mvpp2_port *port, unsigned cpu) {
    u32 reg = mvpp2_read(port->priv, MVPP2_ISR_ENABLE_REG(port->id));
    return reg & MVPP2_ISR_ENABLE_INTERRUPT(BIT(cpu));
}

static irqreturn_t mvpp2_isr(int irq, void *dev_id)
{
	struct mvpp2_queue_vector *qv = dev_id;
	struct mvpp2_port *port = qv->port;

	if (!is_isr_enabled(port, qv->id)) {
	    mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
			MVPP2_ISR_ENABLE_INTERRUPT(BIT(qv->id)));
	}
	mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_DISABLE_INTERRUPT(BIT(qv->id)));
	napi_schedule(&qv->napi);
	return IRQ_HANDLED;
}

static irqreturn_t mvpp2_link_status_isr(int irq, void *dev_id)
{
	struct mvpp2_port *port = (struct mvpp2_port *)dev_id;
	struct net_device *dev = port->sw.dev;
	bool event = false, link = false;
	u32 val;

	mvpp22_gop_mask_irq(port);

	if (port->has_xlg_mac &&
	    (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
	     port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_5GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)) {
		val = readl(port->base + MVPP22_XLG_INT_STAT);
		if (val & MVPP22_XLG_INT_STAT_LINK) {
			event = true;
			val = readl(port->base + MVPP22_XLG_STATUS);
			if (val & MVPP22_XLG_STATUS_LINK_UP)
				link = true;
		}
	} else if (phy_interface_mode_is_rgmii(port->phy_interface) ||
		   phy_interface_mode_is_8023z(port->phy_interface) ||
		   port->phy_interface == PHY_INTERFACE_MODE_SGMII) {
		val = readl(port->base + MVPP22_GMAC_INT_STAT);
		if (val & MVPP22_GMAC_INT_STAT_LINK) {
			event = true;
			val = readl(port->base + MVPP2_GMAC_STATUS0);
			if (val & MVPP2_GMAC_STATUS0_LINK_UP)
				link = true;
		}
	}

	if (!netif_running(dev) || !event)
		goto handled;

	if (link) {
		mvpp2_interrupts_enable(port);

		mvpp2_egress_enable(port);
		mvpp2_ingress_enable(port);
		netif_carrier_on(dev);
		netif_tx_wake_all_queues(dev);
	} else {
		netif_tx_stop_all_queues(dev);
		netif_carrier_off(dev);
		mvpp2_ingress_disable(port);
		mvpp2_egress_disable(port);

		mvpp2_interrupts_disable(port);
	}

handled:
	mvpp22_gop_unmask_irq(port);
	return IRQ_HANDLED;
}

static int mvpp2_rx(struct mvpp2_port *port, struct napi_struct *napi,
		    int budget, struct mvpp2_queue *rxq)
{
	int rx_done = 0;
	if (rxq->pending < budget) {
	    rxq->pending = mvpp2_read(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id)) & MVPP2_RXQ_OCCUPIED_MASK;
	}
	if (rxq->pending < budget) {
		budget = rxq->pending;
	}

	while (rx_done < budget) {
		struct mvpp2_desc *rx_desc_next = rxq->descs + ((rxq->get_index + 2) & rxq->mask);
		void *data_next = (void *)phys_to_virt(le64_to_cpu(rx_desc_next->buf_dma_addr) & MVPP2_DESC_DMA_MASK);
		prefetch(data_next + NET_SKB_PAD);


		rx_done++;
		struct mvpp2_desc *rx_desc = rxq->descs + rxq->get_index;
//		unsigned status = le32_to_cpu(rx_desc->command_status);
//		if (status & MVPP2_RXD_ERR_SUMMARY) {
//			printk("%s@%u: rx err %08x\n", port->sw.dev->name, smp_processor_id(), status);
//			goto next;
//		}
		dma_addr_t dma_addr = le64_to_cpu(rx_desc->buf_dma_addr) & MVPP2_DESC_DMA_MASK;
		void *data = (void *)phys_to_virt(dma_addr);

		rxq->get_index = (rxq->get_index + 1) & rxq->mask;
		prefetch(rxq->descs + rxq->get_index);
		int rx_bytes = le16_to_cpu(rx_desc->data_size) - MVPP2_MH_SIZE;
//		printk("%s@%u: rx%u %llx\n", port->sw.dev->name, smp_processor_id(), rx_bytes, dma_addr);
		struct fp_buf *fpb = fpb_build(data, NET_SKB_PAD + MVPP2_MH_SIZE, rx_bytes);
		if (port->sw.ops) {
			switch_rx_fast(&port->sw, fpb);
		}
		else {
			fast_path_rx_noinvalidate(port->sw.dev, fpb);
		}

//	next:
		data = __skb_bin_get_buffer();
		if (!data) {
			if (net_ratelimit()) {
				printk("%s@%u: could alloc rx buf\n", port->sw.dev->name, smp_processor_id());
			}
			break;
		}
		mvpp2_bm_pool_put(port, 0, virt_to_phys(data));
	}
	rxq->pending -= rx_done;

	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_UPDATE_REG(rxq->id),
		    rx_done | (rx_done << MVPP2_RXQ_NUM_NEW_OFFSET));
	return rx_done;
}

static int mvpp2_tx(struct net_device *dev, struct fp_buf *fpb) {
	struct mvpp2_port *port = netdev_priv(dev);
	unsigned cpu = smp_processor_id();
	struct mvpp2_queue *txq = &port->txqs[cpu];
	struct mvpp2_queue *aggr_txq = &port->priv->aggr_txqs[cpu];

	fast_path_fp_tx_inc(dev, fpb);

	struct mvpp2_desc *tx_desc = aggr_txq->descs + aggr_txq->put_index;
	aggr_txq->put_index = (aggr_txq->put_index + 1) & aggr_txq->mask;

	tx_desc->data_size = cpu_to_le16(fpb_len(fpb));
	dma_addr_t buf_dma_addr = virt_to_phys(fpb_data(fpb));
//	printk("%s: tx%u %llx\n", dev->name, fpb_len(fpb), buf_dma_addr);
	tx_desc->buf_dma_addr = cpu_to_le64(buf_dma_addr & ~MVPP2_DESC_ALIGN);
	tx_desc->packet_offset = buf_dma_addr & MVPP2_DESC_ALIGN;

	txq->bufs[txq->put_index] = fpb_buf(fpb);
	txq->put_index = (txq->put_index + 1) & txq->mask;
	if (!txq->xmit_commit_count) {
		schedule_xmit_commit(dev);
	}
	++txq->xmit_commit_count;
	return 1;
}

static int mvpp2_xmit_commit(struct net_device *dev) {
	struct mvpp2_port *port = netdev_priv(dev);
	unsigned cpu = smp_processor_id();
	struct mvpp2_queue *txq = &port->txqs[cpu];

	txq->count += txq->xmit_commit_count;
	mvpp2_thread_write(port->priv, cpu, MVPP2_AGGR_TXQ_UPDATE_REG, txq->xmit_commit_count);
	txq->xmit_commit_count = 0;

//	if (unlikely(txq->count >= txq->size - FP_NAPI_BUDGET)) {
//		struct netdev_queue *nq = netdev_get_tx_queue(dev, cpu);
//		if (!netif_tx_queue_stopped(nq)) {
//			netif_tx_stop_queue(nq);
//		}
//	}
	return 0;
}

static int mvpp2_poll(struct napi_struct *napi, int budget) {
	struct mvpp2_port *port = netdev_priv(napi->dev);
	unsigned cpu = smp_processor_id();
	struct mvpp2_queue *txq = &port->txqs[cpu];
//	printk("%s@%u poll %u\n", napi->dev->name, smp_processor_id(), txq->get_index);
	mvpp2_txq_done(port, txq);

	struct mvpp2_queue *rxq = &port->rxqs[cpu];
	int count = mvpp2_rx(port, napi, budget, rxq);
	if (count < budget) {
		napi_complete_done(napi, count);
		mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
			    MVPP2_ISR_ENABLE_INTERRUPT(BIT(cpu)));
	}
	call_xmit_commits();
	return count;
}

static void mvpp22_mode_reconfigure(struct mvpp2_port *port)
{
	mvpp22_gop_init(port);

	if  (port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)
		return;

	if (port->has_xlg_mac) {
		u32 ctrl3 = readl(port->base + MVPP22_XLG_CTRL3_REG);
		ctrl3 &= ~MVPP22_XLG_CTRL3_MACMODESELECT_MASK;

		if (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
		    port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
		    port->phy_interface == PHY_INTERFACE_MODE_5GKR)
			ctrl3 |= MVPP22_XLG_CTRL3_MACMODESELECT_10G;
		else
			ctrl3 |= MVPP22_XLG_CTRL3_MACMODESELECT_GMAC;

		writel(ctrl3, port->base + MVPP22_XLG_CTRL3_REG);
	}

	if (port->has_xlg_mac &&
	    (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
	     port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_5GKR))
		mvpp2_xlg_max_rx_size_set(port);
	else
		mvpp2_gmac_max_rx_size_set(port);
}

static void mvpp2_mac_config(struct net_device *dev, unsigned int mode,
			     const struct phylink_link_state *state);
static void mvpp2_mac_link_up(struct net_device *dev, unsigned int mode,
			      struct phy_device *phy);

static void mvpp2_start_dev(struct mvpp2_port *port)
{
	int i;

	mvpp2_txp_max_tx_size_set(port);

	int cpu;
	for_each_present_cpu(cpu)
		mvpp2_tx_pkts_coal_set_txqs(port, cpu, MVPP2_TXDONE_COAL_PKTS_THRESH);

	for (i = 0; i < port->priv->nthreads; i++)
		napi_enable(&port->qvecs[i].napi);

	mvpp2_interrupts_enable(port);

	mvpp22_mode_reconfigure(port);

	struct phylink_link_state state = {
		.interface = port->phy_interface,
	};
	mvpp2_mac_config(port->sw.dev, 0, &state);
	mvpp2_mac_link_up(port->sw.dev, 0, NULL);

	netif_tx_start_all_queues(port->sw.dev);
}

static void mvpp2_stop_dev(struct mvpp2_port *port)
{
	int i;

	netif_tx_stop_all_queues(port->sw.dev);
	mvpp2_ingress_disable(port);
	int cpu;
	for_each_present_cpu(cpu)
		mvpp2_tx_pkts_coal_set_txqs(port, cpu, 0);

	msleep(40);
	mvpp2_egress_disable(port);

	mvpp2_interrupts_disable(port);

	for (i = 0; i < port->priv->nthreads; i++)
		napi_disable(&port->qvecs[i].napi);
}

static int mvpp2_irqs_init(struct mvpp2_port *port)
{
	int err, i;

	for (i = 0; i < port->priv->nthreads; i++) {
		struct mvpp2_queue_vector *qv = port->qvecs + i;
		struct cpumask mask = {};

		irq_set_status_flags(qv->irq, IRQ_NO_BALANCING);

		err = request_irq(qv->irq, mvpp2_isr, 0, port->sw.dev->name, qv);
		if (err)
			goto err;

		cpumask_set_cpu(i, &mask);
		irq_set_affinity_hint(qv->irq, &mask);
	}

	return 0;
err:
	for (i = 0; i < port->priv->nthreads; i++) {
		struct mvpp2_queue_vector *qv = port->qvecs + i;
		irq_set_affinity_hint(qv->irq, NULL);
		free_irq(qv->irq, qv);
	}

	return err;
}

static void mvpp2_irqs_deinit(struct mvpp2_port *port)
{
	int i;

	for (i = 0; i < port->priv->nthreads; i++) {
		struct mvpp2_queue_vector *qv = port->qvecs + i;
		irq_set_affinity_hint(qv->irq, NULL);
		irq_clear_status_flags(qv->irq, IRQ_NO_BALANCING);
		free_irq(qv->irq, qv);
	}
}

/* Configure Rx queue group interrupt for this port */
static void mvpp2_rx_irqs_setup(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	u32 val;
	int i;

	/* Handle the more complicated PPv2.2 and PPv2.3 case */
	for (i = 0; i < port->priv->nthreads; i++) {
		struct mvpp2_queue_vector *qv = port->qvecs + i;

		val = qv->id;
		val |= port->id << MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_OFFSET;
		mvpp2_write(priv, MVPP22_ISR_RXQ_GROUP_INDEX_REG, val);

		val = qv->id;
		val |= 1 << MVPP22_ISR_RXQ_SUB_GROUP_SIZE_OFFSET;
		mvpp2_write(priv, MVPP22_ISR_RXQ_SUB_GROUP_CONFIG_REG, val);
	}
}

static int mvpp2_port_init(struct mvpp2_port *port)
{
	int queue;

	mvpp2_egress_disable(port);
	mvpp2_port_disable(port);

	port->tx_time_coal = MVPP2_TXDONE_COAL_USEC;

	for (queue = 0; queue < port->priv->nthreads; queue++) {
		struct mvpp2_queue *txq = &port->txqs[queue];
		txq->id = (MVPP2_MAX_TCONT + port->id) * MVPP2_MAX_TXQ + queue;
		txq->log_id = queue;
	}

	for (queue = 0; queue < port->priv->nthreads; queue++) {
		struct mvpp2_queue *rxq = &port->rxqs[queue];
		rxq->id = port->first_rxq + queue;
		rxq->log_id = (u8)queue;
	}

	mvpp2_rx_irqs_setup(port);

	mvpp2_ingress_disable(port);
	mvpp2_defaults_set(port);
	mvpp2_cls_oversize_rxq_set(port);
	mvpp2_cls_port_config(port);
	mvpp22_rss_port_init(port);
	return 0;
}

static int mvpp2_open(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	bool valid = false;
	int err;

	printk("%s: open @%u\n", dev->name, smp_processor_id());
	skb_bin_request(dev, dev->l2mtu + 18);

	err = mvpp2_port_init(port);
	if (err < 0) {
		netdev_err(dev, "failed to init port %d\n", port->id);
		return err;
	}

	mvpp2_port_periodic_xon_disable(port);
	mvpp2_port_reset(port);


	err = mvpp2_swf_bm_pool_init(port);
	if (err) {
		return err;
	}

	if (port->sw.ops && port->sw.ops->overhead_bytes == 4) {
	    err = mvpp2_prs_tag_mode_set(port->priv, port->id, MVPP2_TAG_TYPE_DSA);
	}
	else {
	    err = mvpp2_prs_tag_mode_set(port->priv, port->id, MVPP2_TAG_TYPE_MH);
	}
	if (err) {
		netdev_err(dev, "mvpp2_prs_tag_mode_set failed\n");
		return err;
	}
	err = mvpp2_prs_def_flow(port);
	if (err) {
		netdev_err(dev, "mvpp2_prs_def_flow failed\n");
		return err;
	}

	/* Allocate the Rx/Tx queues */
	err = mvpp2_setup_rxqs(port);
	if (err) {
		netdev_err(port->sw.dev, "cannot allocate Rx queues\n");
		return err;
	}

	err = mvpp2_setup_txqs(port);
	if (err) {
		netdev_err(port->sw.dev, "cannot allocate Tx queues\n");
		goto err_cleanup_rxqs;
	}

	err = mvpp2_irqs_init(port);
	if (err) {
		netdev_err(port->sw.dev, "cannot init IRQs\n");
		goto err_cleanup_txqs;
	}

	if (port->link_irq && !port->has_phy) {
		err = request_irq(port->link_irq, mvpp2_link_status_isr, 0,
				  dev->name, port);
		if (err) {
			netdev_err(port->sw.dev, "cannot request link IRQ %d\n",
				   port->link_irq);
			goto err_free_irq;
		}

		mvpp22_gop_setup_irq(port);

		/* In default link is down */
		netif_carrier_off(port->sw.dev);

		valid = true;
	} else {
		port->link_irq = 0;
	}

	if (!valid) {
		netdev_err(port->sw.dev,
			   "invalid configuration: no dt or link IRQ");
		goto err_free_irq;
	}

	mvpp22_rss_enable(port);
	mvpp2_prs_mac_promisc_set(port->priv, port->id,
				  MVPP2_PRS_L2_UNI_CAST, true);
	mvpp2_prs_mac_promisc_set(port->priv, port->id,
				  MVPP2_PRS_L2_MULTI_CAST, true);
	mvpp2_prs_vid_disable_filtering(port);

	on_each_cpu(mvpp2_interrupts_unmask, port, 1);

	mvpp2_start_dev(port);

	if (port->sw.ops) {
		switch_start(&port->sw);
	}
	else {
		phy_enable(&port->sw.phy);
	}
	return 0;

err_free_irq:
	mvpp2_irqs_deinit(port);
err_cleanup_txqs:
	mvpp2_cleanup_txqs(port);
err_cleanup_rxqs:
	mvpp2_cleanup_rxqs(port);
	return err;
}

static int mvpp2_stop(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);

	printk("%s: close\n", dev->name);
	if (port->sw.ops) {
		switch_stop(&port->sw);
	}
	else {
		phy_disable(&port->sw.phy);
	}
	mvpp2_stop_dev(port);

	on_each_cpu(mvpp2_interrupts_mask, port, 1);

	if (port->link_irq)
		free_irq(port->link_irq, port);

	mvpp2_irqs_deinit(port);
	mvpp2_cleanup_rxqs(port);
	mvpp2_cleanup_txqs(port);
	mvpp2_bm_pool_drain(port->priv, &port->priv->bm_pool[0]);
	skb_bin_release(dev);
	return 0;
}

static void mvpp2_ethtool_get_drvinfo(struct net_device *dev,
				      struct ethtool_drvinfo *drvinfo)
{
	unsigned caps = 0;
	struct mvpp2_port *port = netdev_priv(dev);
	strlcpy(drvinfo->driver, MVPP2_DRIVER_NAME,
		sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, MVPP2_DRIVER_VERSION,
		sizeof(drvinfo->version));
	strlcpy(drvinfo->bus_info, dev_name(&dev->dev),
		sizeof(drvinfo->bus_info));
	drvinfo->n_priv_flags = 0;
	if (port->cfg->sfp) {
		caps |= ETHTOOL_CAP_SFP;
		caps |= ETHTOOL_CAP_SFP_PLUS;
		caps |= ETHTOOL_CAP_SFP_DETECT_MODULE;
		caps |= ETHTOOL_CAP_SFP_RX_LOSE;
		caps |= ETHTOOL_CAP_SFP_I2C_EEPROM;
		caps |= ETHTOOL_CAP_SFP_RATE_SELECT;
		caps |= ETHTOOL_CAP_SFP_TX_FAULT;
	}
	ethtool_drvinfo_ext_fill(drvinfo, caps, MVPP2_TX_CSUM_MAX_SIZE);
}

static void mvpp2_self_test(struct net_device *dev, struct ethtool_test *test,
			    u64 *data) {
	struct mvpp2_port *port = netdev_priv(dev);
//	mvpp2_dump(dev, test->flags, test->reserved, test->reserved2);
	if (port->sw.ops) {
		switch_self_test(&port->sw, test, data);
	}
}

static const struct net_device_ops mvpp2_netdev_ops = {
	.ndo_open		= mvpp2_open,
	.ndo_stop		= mvpp2_stop,
	.ndo_tx_timeout		= switch_tx_timeout,
	.ndo_start_xmit		= fast_path_start_xmit,
	.ndo_select_queue	= switch_select_queue,
	.ndo_fast_path_xmit	= mvpp2_tx,
	.ndo_xmit_commit	= mvpp2_xmit_commit,
	.ndo_set_mac_address	= switch_set_mac_address,
	.ndo_change_mtu		= switch_change_mtu,
	.ndo_change_l2mtu	= switch_change_l2mtu,
	.ndo_get_stats64	= fast_path_get_stats64,
	.ndo_do_ioctl		= switch_ioctl,
};

static const struct ethtool_ops mvpp2_eth_tool_ops = {
	.get_drvinfo		= mvpp2_ethtool_get_drvinfo,
	.get_link		= switch_get_link,
	.get_link_ksettings	= switch_get_settings,
	.set_link_ksettings	= switch_set_settings,
	.get_pauseparam		= switch_get_pauseparam,
	.set_pauseparam		= switch_set_pauseparam,
	.get_strings		= switch_get_strings,
	.get_sset_count		= switch_get_sset_count,
	.get_ethtool_stats	= mvpp2_ethtool_get_stats,
	.get_eeprom_len		= switch_get_eeprom_len,
	.get_eeprom		= switch_get_eeprom,
	.self_test		= mvpp2_self_test,
};

static int mvpp2_queue_vectors_init(struct mvpp2_port *port,
					  struct device_node *port_node)
{
	struct mvpp2_queue_vector *v;
	int i, ret;

	for (i = 0; i < port->priv->nthreads; i++) {
		char irqname[16];

		v = port->qvecs + i;

		v->port = port;
		v->id = i;

		snprintf(irqname, sizeof(irqname), "hif%d", i);

		if (port_node)
			v->irq = of_irq_get_byname(port_node, irqname);
		else
			v->irq = of_irq_get(to_of_node(port->fwnode), i);
		if (v->irq <= 0) {
			ret = -EINVAL;
			goto err;
		}

		netif_napi_add(port->sw.dev, &v->napi, mvpp2_poll, FP_NAPI_BUDGET);
	}

	return 0;

err:
	for (i = 0; i < port->priv->nthreads; i++)
		irq_dispose_mapping(port->qvecs[i].irq);
	return ret;
}

static void mvpp2_queue_vectors_deinit(struct mvpp2_port *port)
{
	int i;

	for (i = 0; i < port->priv->nthreads; i++)
		irq_dispose_mapping(port->qvecs[i].irq);
}

static void mvpp2_xlg_config(struct mvpp2_port *port, unsigned int mode,
			     const struct phylink_link_state *state)
{
	u32 ctrl0, ctrl4;

	ctrl0 = readl(port->base + MVPP22_XLG_CTRL0_REG);
	ctrl4 = readl(port->base + MVPP22_XLG_CTRL4_REG);

	ctrl0 |= MVPP22_XLG_CTRL0_MAC_RESET_DIS;

	if (state->pause & MLO_PAUSE_TX)
		ctrl0 |= MVPP22_XLG_CTRL0_TX_FLOW_CTRL_EN;
	else
		ctrl0 &= ~MVPP22_XLG_CTRL0_TX_FLOW_CTRL_EN;

	if (state->pause & MLO_PAUSE_RX)
		ctrl0 |= MVPP22_XLG_CTRL0_RX_FLOW_CTRL_EN;
	else
		ctrl0 &= ~MVPP22_XLG_CTRL0_RX_FLOW_CTRL_EN;

	ctrl4 &= ~MVPP22_XLG_CTRL4_MACMODSELECT_GMAC;

	if (state->interface == PHY_INTERFACE_MODE_RXAUI)
		ctrl4 |= MVPP22_XLG_CTRL4_USE_XPCS;

	writel(ctrl0, port->base + MVPP22_XLG_CTRL0_REG);
	writel(ctrl4, port->base + MVPP22_XLG_CTRL4_REG);
}

static void mvpp2_gmac_config(struct mvpp2_port *port, unsigned int mode,
			      const struct phylink_link_state *state)
{
	u32 old_an, an;
	u32 old_ctrl0, ctrl0;
	u32 old_ctrl2, ctrl2;
	u32 old_ctrl4, ctrl4;

	old_an = an = readl(port->base + MVPP2_GMAC_AUTONEG_CONFIG);
	old_ctrl0 = ctrl0 = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
	old_ctrl2 = ctrl2 = readl(port->base + MVPP2_GMAC_CTRL_2_REG);
	old_ctrl4 = ctrl4 = readl(port->base + MVPP22_GMAC_CTRL_4_REG);

	an &= ~(MVPP2_GMAC_CONFIG_MII_SPEED | MVPP2_GMAC_CONFIG_GMII_SPEED |
		MVPP2_GMAC_AN_SPEED_EN | MVPP2_GMAC_FC_ADV_EN |
		MVPP2_GMAC_FC_ADV_ASM_EN | MVPP2_GMAC_FLOW_CTRL_AUTONEG |
		MVPP2_GMAC_CONFIG_FULL_DUPLEX | MVPP2_GMAC_AN_DUPLEX_EN |
		MVPP2_GMAC_IN_BAND_AUTONEG | MVPP2_GMAC_IN_BAND_AUTONEG_BYPASS);
	ctrl0 &= ~MVPP2_GMAC_PORT_TYPE_MASK;
	ctrl2 &= ~(MVPP2_GMAC_INBAND_AN_MASK | MVPP2_GMAC_PORT_RESET_MASK |
		   MVPP2_GMAC_PCS_ENABLE_MASK);
	ctrl4 &= ~(MVPP22_CTRL4_RX_FC_EN | MVPP22_CTRL4_TX_FC_EN);

	/* Configure port type */
	if (phy_interface_mode_is_8023z(state->interface)) {
		ctrl2 |= MVPP2_GMAC_PCS_ENABLE_MASK;
		ctrl4 &= ~MVPP22_CTRL4_EXT_PIN_GMII_SEL;
		ctrl4 |= MVPP22_CTRL4_SYNC_BYPASS_DIS |
			 MVPP22_CTRL4_DP_CLK_SEL |
			 MVPP22_CTRL4_QSGMII_BYPASS_ACTIVE;
	} else if (state->interface == PHY_INTERFACE_MODE_SGMII ||
		   state->interface == PHY_INTERFACE_MODE_2500BASET) {
		ctrl2 |= MVPP2_GMAC_PCS_ENABLE_MASK | MVPP2_GMAC_INBAND_AN_MASK;
		ctrl4 &= ~MVPP22_CTRL4_EXT_PIN_GMII_SEL;
		ctrl4 |= MVPP22_CTRL4_SYNC_BYPASS_DIS |
			 MVPP22_CTRL4_DP_CLK_SEL |
			 MVPP22_CTRL4_QSGMII_BYPASS_ACTIVE;
	} else if (phy_interface_mode_is_rgmii(state->interface)) {
		ctrl4 &= ~MVPP22_CTRL4_DP_CLK_SEL;
		ctrl4 |= MVPP22_CTRL4_EXT_PIN_GMII_SEL |
			 MVPP22_CTRL4_SYNC_BYPASS_DIS |
			 MVPP22_CTRL4_QSGMII_BYPASS_ACTIVE;
	}

	/* Configure advertisement bits */
	if (phylink_test(state->advertising, Pause))
		an |= MVPP2_GMAC_FC_ADV_EN;
	if (phylink_test(state->advertising, Asym_Pause))
		an |= MVPP2_GMAC_FC_ADV_ASM_EN;

	ctrl4 &= ~(MVPP22_CTRL4_RX_FC_EN | MVPP22_CTRL4_TX_FC_EN);

	/* Configure negotiation style */
	if (!phylink_autoneg_inband(mode)) {
		/* Phy or fixed speed - no in-band AN */
		if (state->duplex)
			an |= MVPP2_GMAC_CONFIG_FULL_DUPLEX;

		if (state->speed == SPEED_1000 || state->speed == SPEED_2500)
			an |= MVPP2_GMAC_CONFIG_GMII_SPEED;
		else if (state->speed == SPEED_100)
			an |= MVPP2_GMAC_CONFIG_MII_SPEED;

	} else if (state->interface == PHY_INTERFACE_MODE_SGMII) {
		/* SGMII in-band mode receives the speed and duplex from
		 * the PHY. Flow control information is not received.
		 */
		an &= ~(MVPP2_GMAC_FORCE_LINK_DOWN |
			MVPP2_GMAC_FORCE_LINK_PASS);
		an |= MVPP2_GMAC_IN_BAND_AUTONEG |
		      MVPP2_GMAC_AN_SPEED_EN |
		      MVPP2_GMAC_AN_DUPLEX_EN;

	} else if (phy_interface_mode_is_8023z(state->interface) ||
		   state->interface == PHY_INTERFACE_MODE_2500BASET) {
		/* 1000BaseX and 2500BaseX ports cannot negotiate speed nor can
		 * they negotiate duplex: they are always operating with a fixed
		 * speed of 1000/2500Mbps in full duplex, so force 1000/2500
		 * speed and full duplex here.
		 */
		ctrl0 |= MVPP2_GMAC_PORT_TYPE_MASK;
		an &= ~(MVPP2_GMAC_FORCE_LINK_DOWN |
			MVPP2_GMAC_FORCE_LINK_PASS);
		an |= MVPP2_GMAC_IN_BAND_AUTONEG |
		      MVPP2_GMAC_CONFIG_GMII_SPEED |
		      MVPP2_GMAC_CONFIG_FULL_DUPLEX;

		if (state->pause & MLO_PAUSE_AN && state->an_enabled)
			an |= MVPP2_GMAC_FLOW_CTRL_AUTONEG;
	}

	if (state->pause & MLO_PAUSE_TX)
		ctrl4 |= MVPP22_CTRL4_TX_FC_EN;
	if (state->pause & MLO_PAUSE_RX)
		ctrl4 |= MVPP22_CTRL4_RX_FC_EN;

/* Some fields of the auto-negotiation register require the port to be down when
 * their value is updated.
 */
#define MVPP2_GMAC_AN_PORT_DOWN_MASK	\
		(MVPP2_GMAC_IN_BAND_AUTONEG | \
		 MVPP2_GMAC_IN_BAND_AUTONEG_BYPASS | \
		 MVPP2_GMAC_CONFIG_MII_SPEED | MVPP2_GMAC_CONFIG_GMII_SPEED | \
		 MVPP2_GMAC_AN_SPEED_EN | MVPP2_GMAC_CONFIG_FULL_DUPLEX | \
		 MVPP2_GMAC_AN_DUPLEX_EN)

	if ((old_ctrl0 ^ ctrl0) & MVPP2_GMAC_PORT_TYPE_MASK ||
	    (old_ctrl2 ^ ctrl2) & MVPP2_GMAC_INBAND_AN_MASK ||
	    (old_an ^ an) & MVPP2_GMAC_AN_PORT_DOWN_MASK) {
		/* Force link down */
		old_an &= ~MVPP2_GMAC_FORCE_LINK_PASS;
		old_an |= MVPP2_GMAC_FORCE_LINK_DOWN;
		writel(old_an, port->base + MVPP2_GMAC_AUTONEG_CONFIG);

		/* Set the GMAC in a reset state - do this in a way that
		 * ensures we clear it below.
		 */
		old_ctrl2 |= MVPP2_GMAC_PORT_RESET_MASK;
		writel(old_ctrl2, port->base + MVPP2_GMAC_CTRL_2_REG);
	}

	if (old_ctrl0 != ctrl0)
		writel(ctrl0, port->base + MVPP2_GMAC_CTRL_0_REG);
	if (old_ctrl2 != ctrl2)
		writel(ctrl2, port->base + MVPP2_GMAC_CTRL_2_REG);
	if (old_ctrl4 != ctrl4)
		writel(ctrl4, port->base + MVPP22_GMAC_CTRL_4_REG);
	if (old_an != an)
		writel(an, port->base + MVPP2_GMAC_AUTONEG_CONFIG);

	if (old_ctrl2 & MVPP2_GMAC_PORT_RESET_MASK) {
		while (readl(port->base + MVPP2_GMAC_CTRL_2_REG) &
		       MVPP2_GMAC_PORT_RESET_MASK)
			continue;
	}
}

static void mvpp2_mac_config(struct net_device *dev, unsigned int mode,
			     const struct phylink_link_state *state)
{
	struct mvpp2_port *port = netdev_priv(dev);
	bool change_interface = port->phy_interface != state->interface;

	/* Check for invalid configuration */
	switch (state->interface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_5GKR:
		if (!port->has_xlg_mac) {
			netdev_err(dev, "Invalid mode %s on %s\n",
				   phy_modes(port->phy_interface), dev->name);
			return;
		}
		break;
	case PHY_INTERFACE_MODE_RXAUI:
		if (port->id != 0) {
			netdev_err(dev, "Invalid mode %s on %s\n",
				   phy_modes(port->phy_interface), dev->name);
			return;
		}
	default:
		break;
	};

	if (change_interface) {
		/* Make sure the port is disabled when reconfiguring the mode */
		netif_tx_stop_all_queues(port->sw.dev);
		mvpp2_port_disable(port);

		mvpp22_gop_mask_irq(port);

		port->phy_interface = state->interface;

		/* Reconfigure the serdes lanes */
		mvpp22_mode_reconfigure(port);

		netif_tx_wake_all_queues(dev);
		mvpp2_port_enable(port);
	}

	/* mac (re)configuration */
	if (state->interface == PHY_INTERFACE_MODE_RXAUI ||
	    state->interface == PHY_INTERFACE_MODE_10GKR ||
	    state->interface == PHY_INTERFACE_MODE_5GKR) {
		mvpp2_xlg_config(port, mode, state);
	} else {
		mvpp2_gmac_config(port, mode, state);
		mvpp2_gmac_tx_fifo_configure(port);
	}

	if (change_interface)
		mvpp22_gop_unmask_irq(port);
}

static void mvpp2_mac_link_up(struct net_device *dev, unsigned int mode,
			      struct phy_device *phy)
{
	struct mvpp2_port *port = netdev_priv(dev);
	u32 val;
	phy_interface_t interface = PHY_INTERFACE_MODE_10GKR;

	if (!true &&
	    interface != PHY_INTERFACE_MODE_RXAUI &&
	    interface != PHY_INTERFACE_MODE_10GKR &&
	    interface != PHY_INTERFACE_MODE_5GKR) {
		val = readl(port->base + MVPP2_GMAC_AUTONEG_CONFIG);
		val &= ~MVPP2_GMAC_FORCE_LINK_DOWN;
		val |= MVPP2_GMAC_FORCE_LINK_PASS;
		writel(val, port->base + MVPP2_GMAC_AUTONEG_CONFIG);
	}

	mvpp2_port_enable(port);

	mvpp2_egress_enable(port);
	mvpp2_ingress_enable(port);
	netif_tx_wake_all_queues(dev);
}

static void mvpp2_sfp_phy_init(struct phy *p) {
	struct mvpp2_port *port = netdev_priv(p->mii_info.dev);

	i2c_sfp_reset(&port->i2c_sfp);
	port->i2c_sfp.adapter = 1;

	gpio_set_value_cansleep(p->priv.sfp_signals->gpio_tx_disable,
				!p->priv.sfp_signals->invert_tx_disable);
}

static void mvpp2_sfp_phy_enable(struct phy *p) {
	gpio_set_value_cansleep(p->priv.sfp_signals->gpio_tx_disable,
				p->priv.sfp_signals->invert_tx_disable);
}

static void mvpp2_sfp_phy_disable(struct phy *p) {
	gpio_set_value_cansleep(p->priv.sfp_signals->gpio_tx_disable,
				!p->priv.sfp_signals->invert_tx_disable);
}

static void mvpp2_sfp_phy_read_eeprom(struct phy *p, u8 *data,
	unsigned off, unsigned len) {
	struct mvpp2_port *port = netdev_priv(p->mii_info.dev);
	i2c_sfp_get(&port->i2c_sfp, data, off, len);
}

static void mvpp2_sfp_phy_get_settings(struct phy *p, struct phy_settings *ps) {
	struct ethtool_link_settings *cmd = &ps->cmd.base;
	struct mvpp2_port *port = netdev_priv(p->mii_info.dev);

	cmd->reserved[0] = 0;
	if (gpio_get_value(p->priv.sfp_signals->gpio_module_present)
	    ^ p->priv.sfp_signals->invert_module_present) {
		cmd->reserved[0] |= SFP_MODULE_PRESENT_FLAG;

		if (!p->eeprom_valid) {
			phy_read_eeprom(p, NULL, NULL);
		}
	}
	else {
		i2c_sfp_reset(&port->i2c_sfp);
	}

	if (gpio_get_value_cansleep(p->priv.sfp_signals->gpio_tx_fault)
	    ^ p->priv.sfp_signals->invert_tx_fault) {
		cmd->reserved[0] |= SFP_TX_FAULT_FLAG;
	}

	if (gpio_get_value_cansleep(p->priv.sfp_signals->gpio_rx_lose)
	    ^ p->priv.sfp_signals->invert_rx_lose) {
		cmd->reserved[0] |= SFP_RX_LOSE_FLAG;
	}

	u32 val = readl(port->base + MVPP22_XLG_STATUS);
	ps->link = !!(val & MVPP22_XLG_STATUS_LINK_UP);
	ps->cmd.base.speed = SPEED_10000;
	ps->cmd.base.duplex = 1;
}

struct phy_ops mvpp2_sfp_ops = {
	.init = &mvpp2_sfp_phy_init,
	.enable = &mvpp2_sfp_phy_enable,
	.disable = &mvpp2_sfp_phy_disable,
	.default_ops = &phy_default_ops,
	.read_eeprom = &mvpp2_sfp_phy_read_eeprom,
	.get_settings = &mvpp2_sfp_phy_get_settings,
};

static unsigned rb5009_port_map[] = {
	9 | PM_BIT_RGMII, 7, 6, 5, 4, 3, 2, 1, 10 | PM_BIT_SFP_PLUS, -1
};

struct mvpp2_port_config rb5009_port_config[] = {
	{
		.sfp = 0,
		.sw_num = 0,
		.port_map = rb5009_port_map,
		.ops = NULL,
	},
};

struct mvpp2_port_config rb5006_port_config[] = {
	{
		.sfp = 1,
		.sw_num = -1,
		.port_map = NULL,
		.ops = &mvpp2_sfp_ops,
	},
	{
		.sfp = 0,
		.sw_num = -1,
		.port_map = NULL,
		.ops = &phy_qca8081_ops,
	},
};

static struct mvpp2_port_config *get_port_cfg(int id) {
	static struct mvpp2_port_config *cfg = NULL;
	if (cfg == NULL) {
		if (is_type("5009")) {
			cfg = rb5009_port_config;
		}
		else if (is_type("5006")) {
			cfg = rb5006_port_config;
		}
	}
	return cfg + id;
}

static void mvpp2_board_specific(struct mvpp2_port *port) {
	if (port->cfg->sfp) {
		extern void init_rb5009_sfp_signals(struct phy *phy);
		init_rb5009_sfp_signals(&port->sw.phy);
	}
	if (port->cfg->ops) {
		port->sw.phy.ops = port->cfg->ops;
	}
}

static int mvpp2_port_probe(struct platform_device *pdev,
			    struct fwnode_handle *port_fwnode,
			    struct mvpp2 *priv)
{
	struct mvpp2_port *port;
	struct device_node *port_node = to_of_node(port_fwnode);
	struct net_device *dev;
	u32 id;
	phy_interface_t phy_mode;
	int err;

	err = of_get_phy_mode(to_of_node(port_fwnode), &phy_mode);
	if (err) {
		dev_err(&pdev->dev, "incorrect phy mode\n");
		return -EINVAL;
	}

	if (fwnode_property_read_u32(port_fwnode, "port-id", &id)) {
		dev_err(&pdev->dev, "missing port-id value\n");
		return -EINVAL;
	}
	printk("port_node:%px port->id:%u\n", port_node, id);

	dev = alloc_switchdev(get_port_cfg(id)->sw_num,
			      sizeof(*port), num_online_cpus());
	if (!dev)
		return -ENOMEM;

	dev->tx_queue_len = 0;
	dev->watchdog_timeo = 5 * HZ;
	dev->netdev_ops = &mvpp2_netdev_ops;
	dev->ethtool_ops = &mvpp2_eth_tool_ops;
	dev->l2mtu = 1518;
	dev->priv_flags |= IFF_NO_QUEUE;
	dev->features = NETIF_F_LLTX;

	port = netdev_priv(dev);
	port->sw.dev = dev;
	port->fwnode = port_fwnode;
	port->has_phy = !!of_find_property(port_node, "phy", NULL);
	if (port->has_phy && phy_mode == PHY_INTERFACE_MODE_INTERNAL) {
		err = -EINVAL;
		dev_err(&pdev->dev, "internal mode doesn't work with phy\n");
		goto err_free_netdev;
	}

	port->priv = priv;

	err = mvpp2_queue_vectors_init(port, port_node);
	if (err)
		goto err_free_netdev;

	if (port_node)
		port->link_irq = of_irq_get_byname(port_node, "link");
	else
		port->link_irq = of_irq_get(to_of_node(port_fwnode), port->priv->nthreads + 1);
	if (port->link_irq == -EPROBE_DEFER) {
		err = -EPROBE_DEFER;
		goto err_deinit_qvecs;
	}
	if (port->link_irq <= 0)
		/* the link irq is optional */
		port->link_irq = 0;

	port->id = id;
	port->first_rxq = port->id * 32;

	port->phy_interface = phy_mode;

	if (port->id == 0)
		port->has_xlg_mac = true;

	if (fwnode_property_read_u32(port_fwnode, "gop-port-id",
				     &port->gop_id)) {
		err = -EINVAL;
		dev_err(&pdev->dev, "missing gop-port-id value\n");
		goto err_deinit_qvecs;
	}

	port->base = priv->iface_base + MVPP22_GMAC_BASE(port->gop_id);
	port->stats_base = port->priv->iface_base +
		MVPP22_MIB_COUNTERS_OFFSET +
		port->gop_id * MVPP22_MIB_COUNTERS_PORT_SZ;

	struct device_node *root = of_find_node_by_path("/");
	unsigned char rmac[ETH_ALEN];
	if (root) {
	    int size;
	    unsigned char *mac = (unsigned char *)of_get_property(root, "mac-address", &size);
	    of_node_put(root);
	    if (mac) {
		    memcpy(dev->dev_addr, mac, ETH_ALEN);
	    }
	    else {
		    eth_hw_addr_random(dev);
	    }
	}
	memcpy(rmac, dev->dev_addr, ETH_ALEN);

	port->tx_ring_size = 1024;
	port->rx_ring_size = 256;
	SET_NETDEV_DEV(dev, &pdev->dev);

	struct eth_stats *xs = &port->sw.xstats[0];
	eth_stats_set_driver_basic_flags(xs);
	eth_stats_set_flag(xs, ETH_STATS_RX_BYTE);
	eth_stats_set_flag(xs, ETH_STATS_TX_UNDERRUN);
	eth_stats_set_flag(xs, ETH_STATS_RX_UNICAST);
	eth_stats_set_flag(xs, ETH_STATS_RX_BROADCAST);
	eth_stats_set_flag(xs, ETH_STATS_RX_MULTICAST);
	eth_stats_set_flag(xs, ETH_STATS_TX_BYTE);
	eth_stats_set_flag(xs, ETH_STATS_TX_UNICAST);
	eth_stats_set_flag(xs, ETH_STATS_TX_MULTICAST);
	eth_stats_set_flag(xs, ETH_STATS_TX_BROADCAST);
	eth_stats_set_flag(xs, ETH_STATS_TX_PAUSE);
	eth_stats_set_flag(xs, ETH_STATS_RX_PAUSE);
	eth_stats_set_flag(xs, ETH_STATS_RX_OVERRUN);
	eth_stats_set_flag(xs, ETH_STATS_RX_TOO_SHORT);
	eth_stats_set_flag(xs, ETH_STATS_RX_FRAGMENT);
	eth_stats_set_flag(xs, ETH_STATS_RX_TOO_LONG);
	eth_stats_set_flag(xs, ETH_STATS_RX_JABBER);
	eth_stats_set_flag(xs, ETH_STATS_TX_COLLISION);
	eth_stats_set_flag(xs, ETH_STATS_TX_LATE_COL);
	eth_stats_set_flag(xs, ETH_STATS_TX_RX_64);
	eth_stats_set_flag(xs, ETH_STATS_TX_RX_65_127);
	eth_stats_set_flag(xs, ETH_STATS_TX_RX_128_255);
	eth_stats_set_flag(xs, ETH_STATS_TX_RX_256_511);
	eth_stats_set_flag(xs, ETH_STATS_TX_RX_512_1023);
	eth_stats_set_flag(xs, ETH_STATS_TX_RX_1024_MAX);

	port->sw.supported_l2mtu = MVPP2_TX_CSUM_MAX_SIZE;
	port->sw.type = SWITCH_MARVELL_88E6393X;
	port->sw.num = 0;
	port->sw.tx_buffer_count = port->tx_ring_size;
	port->sw.mii_phy.mdio_read = orion_mdio_smi_read;
	port->sw.mii_phy.mdio_write = orion_mdio_smi_write;
	port->sw.mii_phy.phy_id = 0; /* single chip mode */
	port->sw.mii_phy.phy_id_mask = 0x1f;
	port->sw.mii_phy.reg_num_mask = 0x1f;
	port->sw.mii_phy.supports_gmii = 1;
	port->sw.mii_phy.dev = dev;
	port->sw.mii_switch = port->sw.mii_phy;
	port->sw.phy.mii_info = port->sw.mii_phy;
	port->sw.mdio_bulk = mvpp2_mdio_bulk;

	rtnl_lock();
	err = register_netdevice(dev);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register netdev\n");
		goto err_phylink;
	}
	netdev_info(dev, "%s: %pM\n", dev->name, dev->dev_addr);

	port->cfg = get_port_cfg(id);
	mvpp2_board_specific(port);

	if (port->cfg->port_map) {
		err = switch_init(&port->sw, port->cfg->port_map, rmac);
	}
	else {
		phy_setup(&port->sw.phy);
	}
	if (err) {
	    printk("switch init failed %d\n", err);
	    unregister_netdevice(dev);
	    goto err_phylink;
	}
	rtnl_unlock();

	priv->port_list[priv->port_count++] = port;
	return 0;

err_phylink:
	if (port->link_irq)
		irq_dispose_mapping(port->link_irq);
err_deinit_qvecs:
	mvpp2_queue_vectors_deinit(port);
err_free_netdev:
	free_netdev(dev);
	return err;
}

static void mvpp2_port_remove(struct mvpp2_port *port)
{
	rtnl_lock();
	if (port->sw.ops) {
		switch_cleanup(&port->sw);
	}
	else {
		phy_cleanup(&port->sw.phy);
	}
	unregister_netdevice(port->sw.dev);
	rtnl_unlock();
	mvpp2_queue_vectors_deinit(port);
	if (port->link_irq)
		irq_dispose_mapping(port->link_irq);
	free_netdev(port->sw.dev);
}

/* Initialize decoding windows */
static void mvpp2_conf_mbus_windows(const struct mbus_dram_target_info *dram,
				    struct mvpp2 *priv)
{
	u32 win_enable;
	int i;

	for (i = 0; i < 6; i++) {
		mvpp2_write(priv, MVPP2_WIN_BASE(i), 0);
		mvpp2_write(priv, MVPP2_WIN_SIZE(i), 0);

		if (i < 4)
			mvpp2_write(priv, MVPP2_WIN_REMAP(i), 0);
	}

	win_enable = 0;

	for (i = 0; i < dram->num_cs; i++) {
		const struct mbus_dram_window *cs = dram->cs + i;

		mvpp2_write(priv, MVPP2_WIN_BASE(i),
			    (cs->base & 0xffff0000) | (cs->mbus_attr << 8) |
			    dram->mbus_dram_target_id);

		mvpp2_write(priv, MVPP2_WIN_SIZE(i),
			    (cs->size - 1) & 0xffff0000);

		win_enable |= (1 << i);
	}

	mvpp2_write(priv, MVPP2_BASE_ADDR_ENABLE, win_enable);
}

static void mvpp22_rx_fifo_set_hw(struct mvpp2 *priv, int port, int data_size)
{
	int attr_size = MVPP2_RX_FIFO_PORT_ATTR_SIZE(data_size);

	mvpp2_write(priv, MVPP2_RX_DATA_FIFO_SIZE_REG(port), data_size);
	mvpp2_write(priv, MVPP2_RX_ATTR_FIFO_SIZE_REG(port), attr_size);
}

/* Initialize TX FIFO's: the total FIFO size is 48kB on PPv2.2 and PPv2.3.
 * 4kB fixed space must be assigned for the loopback port.
 * Redistribute remaining avialable 44kB space among all active ports.
 * Guarantee minimum 32kB for 10G port and 8kB for port 1, capable of 2.5G
 * SGMII link.
 */
static void mvpp22_rx_fifo_init(struct mvpp2 *priv)
{
	int port, size;
	unsigned long port_map;
	int remaining_ports_count;
	int size_remainder;

	/* The loopback requires fixed 4kB of the FIFO space assignment. */
	mvpp22_rx_fifo_set_hw(priv, MVPP2_LOOPBACK_PORT_INDEX,
			      MVPP2_RX_FIFO_PORT_DATA_SIZE_4KB);
	port_map = priv->port_map & ~BIT(MVPP2_LOOPBACK_PORT_INDEX);

	/* Set RX FIFO size to 0 for inactive ports. */
	for_each_clear_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX)
		mvpp22_rx_fifo_set_hw(priv, port, 0);

	/* Assign remaining RX FIFO space among all active ports. */
	size_remainder = MVPP2_RX_FIFO_PORT_DATA_SIZE_44KB;
	remaining_ports_count = hweight_long(port_map);

	for_each_set_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX) {
		if (remaining_ports_count == 1)
			size = size_remainder;
		else if (port == 0)
			size = max(size_remainder / remaining_ports_count,
				   MVPP2_RX_FIFO_PORT_DATA_SIZE_32KB);
		else if (port == 1)
			size = max(size_remainder / remaining_ports_count,
				   MVPP2_RX_FIFO_PORT_DATA_SIZE_8KB);
		else
			size = size_remainder / remaining_ports_count;

		size_remainder -= size;
		remaining_ports_count--;

		mvpp22_rx_fifo_set_hw(priv, port, size);
	}

	mvpp2_write(priv, MVPP2_RX_MIN_PKT_SIZE_REG,
		    MVPP2_RX_FIFO_PORT_MIN_PKT);
	mvpp2_write(priv, MVPP2_RX_FIFO_INIT_REG, 0x1);
}

static void mvpp22_tx_fifo_set_hw(struct mvpp2 *priv, int port, int size)
{
	int threshold = MVPP2_TX_FIFO_THRESHOLD(size);

	mvpp2_write(priv, MVPP22_TX_FIFO_SIZE_REG(port), size);
	mvpp2_write(priv, MVPP22_TX_FIFO_THRESH_REG(port), threshold);
}

/* Initialize TX FIFO's: the total FIFO size is 19kB on PPv2.2 and PPv2.3.
 * 1kB fixed space must be assigned for the loopback port.
 * Redistribute remaining avialable 18kB space among all active ports.
 * The 10G interface should use 10kB (which is maximum possible size
 * per single port).
 */
static void mvpp22_tx_fifo_init_default(struct mvpp2 *priv)
{
	int port, size;
	unsigned long port_map;
	int remaining_ports_count;
	int size_remainder;

	/* The loopback requires fixed 1kB of the FIFO space assignment. */
	mvpp22_tx_fifo_set_hw(priv, MVPP2_LOOPBACK_PORT_INDEX,
			      MVPP22_TX_FIFO_DATA_SIZE_1KB);
	port_map = priv->port_map & ~BIT(MVPP2_LOOPBACK_PORT_INDEX);

	/* Set TX FIFO size to 0 for inactive ports. */
	for_each_clear_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX)
		mvpp22_tx_fifo_set_hw(priv, port, 0);

	/* Assign remaining TX FIFO space among all active ports. */
	size_remainder = MVPP22_TX_FIFO_DATA_SIZE_18KB;
	remaining_ports_count = hweight_long(port_map);

	for_each_set_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX) {
		if (remaining_ports_count == 1)
			size = min(size_remainder,
				   MVPP22_TX_FIFO_DATA_SIZE_10KB);
		else if (port == 0)
			size = MVPP22_TX_FIFO_DATA_SIZE_10KB;
		else
			size = size_remainder / remaining_ports_count;

		size_remainder -= size;
		remaining_ports_count--;

		mvpp22_tx_fifo_set_hw(priv, port, size);
	}
}

static void mvpp2_axi_init(struct mvpp2 *priv)
{
	u32 val, rdval, wrval;

	mvpp2_write(priv, MVPP22_BM_ADDR_HIGH_RLS_REG, 0x0);

	/* AXI Bridge Configuration */

	rdval = MVPP22_AXI_CODE_CACHE_RD_CACHE
		<< MVPP22_AXI_ATTR_CACHE_OFFS;
	rdval |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM
		<< MVPP22_AXI_ATTR_DOMAIN_OFFS;

	wrval = MVPP22_AXI_CODE_CACHE_WR_CACHE
		<< MVPP22_AXI_ATTR_CACHE_OFFS;
	wrval |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM
		<< MVPP22_AXI_ATTR_DOMAIN_OFFS;

	/* BM */
	mvpp2_write(priv, MVPP22_AXI_BM_WR_ATTR_REG, wrval);
	mvpp2_write(priv, MVPP22_AXI_BM_RD_ATTR_REG, rdval);

	/* Descriptors */
	mvpp2_write(priv, MVPP22_AXI_AGGRQ_DESCR_RD_ATTR_REG, rdval);
	mvpp2_write(priv, MVPP22_AXI_TXQ_DESCR_WR_ATTR_REG, wrval);
	mvpp2_write(priv, MVPP22_AXI_TXQ_DESCR_RD_ATTR_REG, rdval);
	mvpp2_write(priv, MVPP22_AXI_RXQ_DESCR_WR_ATTR_REG, wrval);

	/* Buffer Data */
	/* Force TX FIFO transactions priority on the AXI QOS bus */
//	if (tx_fifo_protection)
//		rdval |= MVPP22_AXI_TX_DATA_RD_QOS_ATTRIBUTE;

	mvpp2_write(priv, MVPP22_AXI_TX_DATA_RD_ATTR_REG, rdval);
	mvpp2_write(priv, MVPP22_AXI_RX_DATA_WR_ATTR_REG, wrval);

	val = MVPP22_AXI_CODE_CACHE_NON_CACHE
		<< MVPP22_AXI_CODE_CACHE_OFFS;
	val |= MVPP22_AXI_CODE_DOMAIN_SYSTEM
		<< MVPP22_AXI_CODE_DOMAIN_OFFS;
	mvpp2_write(priv, MVPP22_AXI_RD_NORMAL_CODE_REG, val);
	mvpp2_write(priv, MVPP22_AXI_WR_NORMAL_CODE_REG, val);

	val = MVPP22_AXI_CODE_CACHE_RD_CACHE
		<< MVPP22_AXI_CODE_CACHE_OFFS;
	val |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM
		<< MVPP22_AXI_CODE_DOMAIN_OFFS;

	mvpp2_write(priv, MVPP22_AXI_RD_SNOOP_CODE_REG, val);

	val = MVPP22_AXI_CODE_CACHE_WR_CACHE
		<< MVPP22_AXI_CODE_CACHE_OFFS;
	val |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM
		<< MVPP22_AXI_CODE_DOMAIN_OFFS;

	mvpp2_write(priv, MVPP22_AXI_WR_SNOOP_CODE_REG, val);
}

static int mvpp2_init(struct platform_device *pdev, struct mvpp2 *priv)
{
	const struct mbus_dram_target_info *dram_target_info;
	int err, i;
	u32 val;
	dma_addr_t p;

	dram_target_info = mv_mbus_dram_info();
	if (dram_target_info)
		mvpp2_conf_mbus_windows(dram_target_info, priv);

	mvpp2_axi_init(priv);

	/* Disable HW PHY polling */
	val = readl(priv->iface_base + MVPP22_SMI_MISC_CFG_REG);
	val &= ~MVPP22_SMI_POLLING_EN;
	writel(val, priv->iface_base + MVPP22_SMI_MISC_CFG_REG);

	val = sizeof(*priv->aggr_txqs) * MVPP2_MAX_THREADS + L1_CACHE_BYTES;
	p = (dma_addr_t)devm_kzalloc(&pdev->dev, val, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p = (p + ~CACHE_LINE_MASK) & CACHE_LINE_MASK;
	priv->aggr_txqs = (struct mvpp2_queue *)p;

	for (i = 0; i < MVPP2_MAX_THREADS; i++) {
		priv->aggr_txqs[i].id = i;
		priv->aggr_txqs[i].size = MVPP2_AGGR_TXQ_SIZE;
		priv->aggr_txqs[i].mask = MVPP2_AGGR_TXQ_SIZE - 1;
		err = mvpp2_aggr_txq_init(pdev, &priv->aggr_txqs[i], i, priv);
		if (err < 0)
			return err;
	}

	mvpp22_rx_fifo_init(priv);
	mvpp22_tx_fifo_init_default(priv);
	mvpp2_write(priv, MVPP2_TX_SNOOP_REG, 0x1);

	err = mvpp2_bm_init(pdev, priv);
	if (err < 0)
		return err;

	err = mvpp2_prs_default_init(pdev, priv);
	if (err < 0)
		return err;

	mvpp2_cls_init(priv);
	mvpp2_rxq_disable_all(priv);
	return 0;
}

static int mvpp2_probe(struct platform_device *pdev)
{
	struct fwnode_handle *fwnode = pdev->dev.fwnode;
	struct fwnode_handle *port_fwnode;
	struct mvpp2 *priv;
	struct resource *res;
	void __iomem *base;
	int i;
	int err;

	struct device_node *mdio_node = of_find_compatible_node(NULL, NULL, "marvell,orion-mdio");
//	printk("mvpp2_probe %px\n", mdio_node);
	if (mdio_node) {
		for (i = 0; i < ARRAY_SIZE(mdio_clk); i++) {
			mdio_clk[i] = of_clk_get(mdio_node, i);
//			printk("mdio clk prep %px\n", mdio_clk[i]);
			if (IS_ERR(mdio_clk[i]))
				break;
			clk_prepare_enable(mdio_clk[i]);
		}
		of_node_put(mdio_node);
	}
	mdio_regs = ioremap(0xf212a200, 0x100);
	writel(is_type("5009") ? 0x612 : 0x610, mdio_regs + 4);

	hrtimer_init(&mdio_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	mdio_timer.function = mvpp2_mdio_timer;
	mutex_init(&mdio_mutex);
	init_waitqueue_head(&mdio_wait);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	{
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		priv->iface_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->iface_base))
			return PTR_ERR(priv->iface_base);

		priv->sram_pool = of_gen_pool_get(pdev->dev.of_node, "cm3-mem", 0);
		if (priv->sram_pool) {
			priv->cm3_base = (void *)gen_pool_alloc(priv->sram_pool, MSS_SRAM_SIZE);
		}
		if (priv->cm3_base)
			priv->global_tx_fc = true;
	}

	if (dev_of_node(&pdev->dev)) {
		priv->sysctrl_base =
			syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							"marvell,system-controller");
		if (IS_ERR(priv->sysctrl_base))
			/* The system controller regmap is optional for dt
			 * compatibility reasons. When not provided, the
			 * configuration of the GoP relies on the
			 * firmware/bootloader.
			 */
			priv->sysctrl_base = NULL;
	}

	priv->nthreads = num_present_cpus();

	for (i = 0; i < MVPP2_MAX_THREADS; i++) {
		priv->swth_base[i] = base + i * SZ_64K;
	}

	if (dev_of_node(&pdev->dev)) {
		priv->pp_clk = devm_clk_get(&pdev->dev, "pp_clk");
		if (IS_ERR(priv->pp_clk))
			return PTR_ERR(priv->pp_clk);
		err = clk_prepare_enable(priv->pp_clk);
		if (err < 0)
			return err;

		priv->gop_clk = devm_clk_get(&pdev->dev, "gop_clk");
		if (IS_ERR(priv->gop_clk)) {
			err = PTR_ERR(priv->gop_clk);
			goto err_pp_clk;
		}
		err = clk_prepare_enable(priv->gop_clk);
		if (err < 0)
			goto err_pp_clk;

		{
			priv->mg_clk = devm_clk_get(&pdev->dev, "mg_clk");
			if (IS_ERR(priv->mg_clk)) {
				err = PTR_ERR(priv->mg_clk);
				goto err_gop_clk;
			}

			err = clk_prepare_enable(priv->mg_clk);
			if (err < 0)
				goto err_gop_clk;

			priv->mg_core_clk = devm_clk_get(&pdev->dev, "mg_core_clk");
			if (IS_ERR(priv->mg_core_clk)) {
				priv->mg_core_clk = NULL;
			} else {
				err = clk_prepare_enable(priv->mg_core_clk);
				if (err < 0)
					goto err_mg_clk;
			}
		}

		priv->axi_clk = devm_clk_get(&pdev->dev, "axi_clk");
		if (IS_ERR(priv->axi_clk)) {
			err = PTR_ERR(priv->axi_clk);
			if (err == -EPROBE_DEFER)
				goto err_mg_core_clk;
			priv->axi_clk = NULL;
		} else {
			err = clk_prepare_enable(priv->axi_clk);
			if (err < 0)
				goto err_mg_core_clk;
		}

		priv->tclk = clk_get_rate(priv->pp_clk);
	}

	priv->custom_dma_mask = false;
	{
		/* If dma_mask points to coherent_dma_mask, setting both will
		 * override the value of the other. This is problematic as the
		 * PPv2 driver uses a 32-bit-mask for coherent accesses (txq,
		 * rxq, bm) and a 40-bit mask for all other accesses.
		 */
		if (pdev->dev.dma_mask == &pdev->dev.coherent_dma_mask) {
			pdev->dev.dma_mask =
				kzalloc(sizeof(*pdev->dev.dma_mask),
					GFP_KERNEL);
			if (!pdev->dev.dma_mask) {
				err = -ENOMEM;
				goto err_mg_clk;
			}

			priv->custom_dma_mask = true;
		}

		err = dma_set_mask(&pdev->dev, MVPP2_DESC_DMA_MASK);
		if (err)
			goto err_dma_mask;

		/* Sadly, the BM pools all share the same register to
		 * store the high 32 bits of their address. So they
		 * must all have the same high 32 bits, which forces
		 * us to restrict coherent memory to DMA_BIT_MASK(32).
		 */
		err = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
		if (err)
			goto err_dma_mask;
	}

	if (dev_of_node(&pdev->dev))
		of_reserved_mem_device_init_by_idx(&pdev->dev,
						   pdev->dev.of_node, 0);


	fwnode_for_each_child_node(fwnode, port_fwnode) {
		if (!fwnode_device_is_available(port_fwnode)) continue;
		if (!fwnode_property_read_u32(port_fwnode, "port-id", &i))
			priv->port_map |= BIT(i);
	}

	err = mvpp2_init(pdev, priv);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to initialize controller\n");
		goto err_axi_clk;
	}

	fwnode_for_each_child_node(fwnode, port_fwnode) {
		if (!fwnode_device_is_available(port_fwnode)) continue;
		err = mvpp2_port_probe(pdev, port_fwnode, priv);
		if (err < 0)
			goto err_port_probe;
	}

	if (priv->port_count == 0) {
		dev_err(&pdev->dev, "no ports enabled\n");
		err = -ENODEV;
		goto err_axi_clk;
	}

	if (priv->global_tx_fc) {
		err = mvpp2_enable_global_fc(priv);
		if (err)
			dev_warn(&pdev->dev, "CM3 firmware not running, TX FC disabled\n");
	}

	platform_set_drvdata(pdev, priv);
	printk("mvpp2_probe out\n");
	return 0;

err_port_probe:
	i = 0;
	fwnode_for_each_child_node(fwnode, port_fwnode) {
		if (!fwnode_device_is_available(port_fwnode)) continue;
		if (priv->port_list[i])
			mvpp2_port_remove(priv->port_list[i]);
		i++;
	}
err_dma_mask:
	if (priv->custom_dma_mask) {
		kfree(pdev->dev.dma_mask);
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	}
err_axi_clk:
	clk_disable_unprepare(priv->axi_clk);

err_mg_core_clk:
	clk_disable_unprepare(priv->mg_core_clk);
err_mg_clk:
	clk_disable_unprepare(priv->mg_clk);
err_gop_clk:
	clk_disable_unprepare(priv->gop_clk);
err_pp_clk:
	clk_disable_unprepare(priv->pp_clk);
	return err;
}

static int mvpp2_remove(struct platform_device *pdev)
{
	struct mvpp2 *priv = platform_get_drvdata(pdev);
	struct fwnode_handle *fwnode = pdev->dev.fwnode;
	struct fwnode_handle *port_fwnode;
	int i = 0;

	fwnode_for_each_child_node(fwnode, port_fwnode) {
		if (!fwnode_device_is_available(port_fwnode)) continue;
		if (priv->port_list[i]) {
			mvpp2_port_remove(priv->port_list[i]);
		}
		i++;
	}

	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		struct mvpp2_bm_pool *bm_pool = &priv->bm_pool[i];
		mvpp2_bm_pool_destroy(pdev, priv, bm_pool);
	}

	for (i = 0; i < MVPP2_MAX_THREADS; i++) {
		struct mvpp2_queue *aggr_txq = &priv->aggr_txqs[i];
		dma_free_coherent(&pdev->dev,
				  MVPP2_AGGR_TXQ_SIZE * MVPP2_DESC_ALIGNED_SIZE,
				  aggr_txq->descs,
				  aggr_txq->descs_dma);
	}

	if (priv->custom_dma_mask) {
		kfree(pdev->dev.dma_mask);
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	}

	if (priv->sram_pool) {
		gen_pool_free(priv->sram_pool, (unsigned long)priv->cm3_base,
			      MSS_SRAM_SIZE);
		gen_pool_destroy(priv->sram_pool);
	}

	clk_disable_unprepare(priv->axi_clk);
	clk_disable_unprepare(priv->mg_core_clk);
	clk_disable_unprepare(priv->mg_clk);
	clk_disable_unprepare(priv->pp_clk);
	clk_disable_unprepare(priv->gop_clk);

	for (i = 0; i < ARRAY_SIZE(mdio_clk); i++) {
		if (IS_ERR(mdio_clk[i]))
			break;
		clk_disable_unprepare(mdio_clk[i]);
		clk_put(mdio_clk[i]);
	}

	iounmap(mdio_regs);

	return 0;
}

static const struct of_device_id mvpp2_match[] = {
	{
		.compatible = "marvell,armada-7k-pp22",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, mvpp2_match);

static struct platform_driver mvpp2_driver = {
	.probe = mvpp2_probe,
	.remove = mvpp2_remove,
	.driver = {
		.name = MVPP2_DRIVER_NAME,
		.of_match_table = mvpp2_match,
	},
};

module_platform_driver(mvpp2_driver);

MODULE_DESCRIPTION("Marvell PPv2 Ethernet Driver - www.marvell.com");
MODULE_AUTHOR("Marcin Wojtas <mw@semihalf.com>");
MODULE_LICENSE("GPL v2");
