/*
 * Freescale PowerQUICC Ethernet Driver -- MIIM bus implementation
 * Provides Bus interface for MIIM regs
 *
 * Author: Andy Fleming <afleming@freescale.com>
 * Modifier: Sandeep Gopalpet <sandeep.kumar@freescale.com>
 *
 * Copyright 2002-2004, 2008-2009, 2012 Freescale Semiconductor, Inc.
 *
 * Based on gianfar_mii.c and ucc_geth_mii.c (Li Yang, Kim Phillips)
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <soc/fsl/qe/ucc.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "fsl_pq_mdio.h"

struct fsl_pq_mdio_priv {
	void __iomem *map;
	struct fsl_pq_mdio __iomem *regs;
	struct mutex mdio_lock;
};

/* pointer to bus, to be used by the fsl_pq_mdio locking functions when called
 * with a null parameter */
struct mii_bus *fsl_pq_mdio_bus;

/*
 * Write value to the PHY at mii_id at register regnum,
 * on the bus attached to the local interface, which may be different from the
 * generic mdio bus (tied to a single interface), waiting until the write is
 * done before returning. This is helpful in programming interfaces like
 * the TBI which control interfaces like onchip SERDES and are always tied to
 * the local mdio pins, which may not be the same as system mdio bus, used for
 * controlling the external PHYs, for example.
 */
int fsl_pq_local_mdio_write(struct fsl_pq_mdio __iomem *regs, int mii_id,
			int regnum, u16 value)
{
	/* Set the PHY address and the register address we want to write */
	out_be32(&regs->miimadd, (mii_id << 8) | regnum);

	/* Write out the value we want */
	out_be32(&regs->miimcon, value);

	/* Wait for the transaction to finish */
	while (in_be32(&regs->miimind) & MIIMIND_BUSY)
		cpu_relax();

	return 0;
}

/*
 * Read the bus for PHY at addr mii_id, register regnum, and
 * return the value.  Clears miimcom first.  All PHY operation
 * done on the bus attached to the local interface,
 * which may be different from the generic mdio bus
 * This is helpful in programming interfaces like
 * the TBI which, in turn, control interfaces like onchip SERDES
 * and are always tied to the local mdio pins, which may not be the
 * same as system mdio bus, used for controlling the external PHYs, for eg.
 */
int fsl_pq_local_mdio_read(struct fsl_pq_mdio __iomem *regs, int mii_id,
			int regnum)
{
	u16 value;

	/* Set the PHY address and the register address we want to read */
	out_be32(&regs->miimadd, (mii_id << 8) | regnum);

	/* Clear miimcom, and then initiate a read */
	out_be32(&regs->miimcom, 0);
	out_be32(&regs->miimcom, MII_READ_COMMAND);

	/* Wait for the transaction to finish */
	while (in_be32(&regs->miimind) & (MIIMIND_NOTVALID | MIIMIND_BUSY))
		cpu_relax();

	/* Grab the value of the register from miimstat */
	value = in_be32(&regs->miimstat);

	return value;
}

static struct fsl_pq_mdio __iomem *fsl_pq_mdio_get_regs(struct mii_bus *bus)
{
	struct fsl_pq_mdio_priv *priv = bus->priv;

	return priv->regs;
}

void fsl_pq_mdio_lock(struct mii_bus *bus)
{
	struct fsl_pq_mdio_priv *priv;

	priv = (bus) ? bus->priv : fsl_pq_mdio_bus->priv;

	mutex_lock(&priv->mdio_lock);
}
EXPORT_SYMBOL(fsl_pq_mdio_lock);

void fsl_pq_mdio_unlock(struct mii_bus *bus)
{
	struct fsl_pq_mdio_priv *priv;

	priv = (bus) ? bus->priv : fsl_pq_mdio_bus->priv;

	mutex_unlock(&priv->mdio_lock);
}
EXPORT_SYMBOL(fsl_pq_mdio_unlock);

/*
 * Write value to the PHY at mii_id at register regnum,
 * on the bus, waiting until the write is done before returning.
 */
int fsl_pq_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			u16 value)
{
	struct fsl_pq_mdio __iomem *regs = fsl_pq_mdio_get_regs(bus);
	int result;

	fsl_pq_mdio_lock(bus);

	/* Write to the local MII regs */
	result = fsl_pq_local_mdio_write(regs, mii_id, regnum, value);

	fsl_pq_mdio_unlock(bus);

	return result;
}

/*
 * Read the bus for PHY at addr mii_id, register regnum, and
 * return the value.  Clears miimcom first.
 */
int fsl_pq_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct fsl_pq_mdio __iomem *regs = fsl_pq_mdio_get_regs(bus);
	int result;

	fsl_pq_mdio_lock(bus);

	/* Read the local MII regs */
	result =  fsl_pq_local_mdio_read(regs, mii_id, regnum);

	fsl_pq_mdio_unlock(bus);

	return result;
}

/* Reset the MIIM registers, and wait for the bus to free */
static int fsl_pq_mdio_reset(struct mii_bus *bus)
{
	struct fsl_pq_mdio __iomem *regs = fsl_pq_mdio_get_regs(bus);
	int timeout = PHY_INIT_TIMEOUT;

	mutex_lock(&bus->mdio_lock);

	/* Reset the management interface */
	out_be32(&regs->miimcfg, MIIMCFG_RESET);

	/* Setup the MII Mgmt clock speed */
	out_be32(&regs->miimcfg, MIIMCFG_INIT_VALUE);

	/* Wait until the bus is free */
	while ((in_be32(&regs->miimind) & MIIMIND_BUSY) && timeout--)
		cpu_relax();

	mutex_unlock(&bus->mdio_lock);

	if (timeout < 0) {
		printk(KERN_ERR "%s: The MII Bus is stuck!\n",
				bus->name);
		return -EBUSY;
	}

	return 0;
}

void fsl_pq_mdio_bus_name(char *name, struct device_node *np)
{
	const u32 *addr;
	u64 taddr = OF_BAD_ADDR;

	addr = of_get_address(np, 0, NULL, NULL);
	if (addr)
		taddr = of_translate_address(np, addr);

	snprintf(name, MII_BUS_ID_SIZE, "%s@%llx", np->name,
		(unsigned long long)taddr);
}
EXPORT_SYMBOL_GPL(fsl_pq_mdio_bus_name);

static int get_phy_id(struct mii_bus *bus, int addr, u32 *phy_id)
{
	int h, l;

	h = mdiobus_read(bus, addr, MII_PHYSID1);
	if (h < 0)
		return -EIO;

	l = mdiobus_read(bus, addr, MII_PHYSID2);
	if (l < 0)
		return -EIO;

	*phy_id = ((h & 0xffff) << 16) | (l & 0xffff);
	return 0;
}

/* Scan the bus in reverse, looking for an empty spot */
static int fsl_pq_mdio_find_free(struct mii_bus *new_bus)
{
	int i;

	for (i = PHY_MAX_ADDR; i > 0; i--) {
		u32 phy_id;

		if (get_phy_id(new_bus, i, &phy_id))
			return -1;

		if (phy_id == 0xffffffff)
			break;
	}

	return i;
}


#if defined(CONFIG_GIANFAR) || defined(CONFIG_GIANFAR_MODULE)
static u32 __iomem *get_gfar_tbipa(struct fsl_pq_mdio __iomem *regs, struct device_node *np)
{
	struct gfar __iomem *enet_regs;

	/*
	 * This is mildly evil, but so is our hardware for doing this.
	 * Also, we have to cast back to struct gfar because of
	 * definition weirdness done in gianfar.h.
	 */
	if(of_device_is_compatible(np, "fsl,gianfar-mdio") ||
		of_device_is_compatible(np, "fsl,gianfar-tbi") ||
		of_device_is_compatible(np, "gianfar")) {
		enet_regs = (struct gfar __iomem *)regs;
		return &enet_regs->tbipa;
	} else if (of_device_is_compatible(np, "fsl,etsec2-mdio") ||
			of_device_is_compatible(np, "fsl,etsec2-tbi")) {
		return of_iomap(np, 1);
	} else
		return NULL;
}
#endif


#if defined(CONFIG_UCC_GETH) || defined(CONFIG_UCC_GETH_MODULE)
static int get_ucc_id_for_range(u64 start, u64 end, u32 *ucc_id)
{
	struct device_node *np = NULL;
	int err = 0;

	for_each_compatible_node(np, NULL, "ucc_geth") {
		struct resource tempres;

		err = of_address_to_resource(np, 0, &tempres);
		if (err)
			continue;

		/* if our mdio regs fall within this UCC regs range */
		if ((start >= tempres.start) && (end <= tempres.end)) {
			/* Find the id of the UCC */
			const u32 *id;

			id = of_get_property(np, "cell-index", NULL);
			if (!id) {
				id = of_get_property(np, "device-id", NULL);
				if (!id)
					continue;
			}

			*ucc_id = *id;

			return 0;
		}
	}

	if (err)
		return err;
	else
		return -EINVAL;
}
#endif


static int fsl_pq_mdio_probe(struct platform_device *ofdev)
{
	struct device_node *np = ofdev->dev.of_node;
	struct device_node *tbi;
	struct fsl_pq_mdio_priv *priv;
	struct fsl_pq_mdio __iomem *regs = NULL;
	void __iomem *map;
	u32 __iomem *tbipa;
	struct mii_bus *new_bus;
	int tbiaddr = -1;
	const u32 *addrp;
	u64 addr = 0, size = 0;
	int err;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->mdio_lock);

	new_bus = mdiobus_alloc();
	if (!new_bus) {
		err = -ENOMEM;
		goto err_free_priv;
	}

	new_bus->name = "Freescale PowerQUICC MII Bus",
	new_bus->read = &fsl_pq_mdio_read,
	new_bus->write = &fsl_pq_mdio_write,
	new_bus->reset = &fsl_pq_mdio_reset,
	new_bus->priv = priv;
	fsl_pq_mdio_bus_name(new_bus->id, np);

	addrp = of_get_address(np, 0, &size, NULL);
	if (!addrp) {
		err = -EINVAL;
		goto err_free_bus;
	}

	/* Set the PHY base address */
	addr = of_translate_address(np, addrp);
	if (addr == OF_BAD_ADDR) {
		err = -EINVAL;
		goto err_free_bus;
	}

	map = ioremap(addr, size);
	if (!map) {
		err = -ENOMEM;
		goto err_free_bus;
	}
	priv->map = map;

	if (of_device_is_compatible(np, "fsl,gianfar-mdio") ||
			of_device_is_compatible(np, "fsl,gianfar-tbi") ||
			of_device_is_compatible(np, "fsl,fman-mdio") ||
			of_device_is_compatible(np, "fsl,ucc-mdio") ||
			of_device_is_compatible(np, "ucc_geth_phy"))
		map -= offsetof(struct fsl_pq_mdio, miimcfg);
	regs = map;
	priv->regs = regs;

	new_bus->parent = &ofdev->dev;
	dev_set_drvdata(&ofdev->dev, new_bus);

	if (of_device_is_compatible(np, "fsl,gianfar-mdio") ||
			of_device_is_compatible(np, "fsl,gianfar-tbi") ||
			of_device_is_compatible(np, "fsl,etsec2-mdio") ||
			of_device_is_compatible(np, "fsl,etsec2-tbi") ||
			of_device_is_compatible(np, "gianfar")) {
#if defined(CONFIG_GIANFAR) || defined(CONFIG_GIANFAR_MODULE)
		tbipa = get_gfar_tbipa(regs, np);
		if (!tbipa) {
			err = -EINVAL;
			goto err_unmap_regs;
		}
#else
		err = -ENODEV;
		goto err_unmap_regs;
#endif
	} else if (of_device_is_compatible(np, "fsl,ucc-mdio") ||
			of_device_is_compatible(np, "ucc_geth_phy")) {
#if defined(CONFIG_UCC_GETH) || defined(CONFIG_UCC_GETH_MODULE)
		u32 id;
		static u32 mii_mng_master;

		tbipa = &regs->utbipar;

		if ((err = get_ucc_id_for_range(addr, addr + size, &id)))
			goto err_unmap_regs;

		if (!mii_mng_master) {
			mii_mng_master = id;
			ucc_set_qe_mux_mii_mng(id - 1);
		}
#else
		err = -ENODEV;
		goto err_unmap_regs;
#endif
	} else if (of_device_is_compatible(np, "fsl,fman-mdio")) {
#ifdef CONFIG_FSL_FMAN
		tbiaddr = 5;
#else
		err = -ENODEV;
		goto err_unmap_regs;
#endif
	} else {
		err = -ENODEV;
		goto err_unmap_regs;
	}

	for_each_child_of_node(np, tbi) {
		if (of_node_is_type(tbi, "tbi-phy"))
			break;
	}

	if (tbi) {
		const u32 *prop = of_get_property(tbi, "reg", NULL);

		if (prop)
			tbiaddr = *prop;
	}

	/*
	 * We define TBIPA at 0 to be illegal, opting to fail for boards that
	 * have PHYs at 1-31, rather than change tbipa and rescan.
	 */
	if (tbiaddr == 0) {
		err = -EBUSY;

		goto err_unmap_regs;
	}

#ifndef CONFIG_FSL_FMAN
	out_be32(tbipa, tbiaddr);
#endif

	fsl_pq_mdio_bus = new_bus;

	err = of_mdiobus_register(new_bus, np);
	if (err) {
		printk (KERN_ERR "%s: Cannot register as MDIO bus\n",
				new_bus->name);
		goto err_unmap_regs;
	}

	return 0;

err_unmap_regs:
	iounmap(priv->map);
err_free_bus:
	kfree(new_bus);
err_free_priv:
	kfree(priv);
	return err;
}


static int fsl_pq_mdio_remove(struct platform_device *ofdev)
{
	struct device *device = &ofdev->dev;
	struct mii_bus *bus = dev_get_drvdata(device);
	struct fsl_pq_mdio_priv *priv = bus->priv;

	mdiobus_unregister(bus);

	dev_set_drvdata(device, NULL);

	iounmap(priv->map);
	bus->priv = NULL;
	mdiobus_free(bus);
	kfree(priv);

	return 0;
}

static struct of_device_id fsl_pq_mdio_match[] = {
	{
		.type = "mdio",
		.compatible = "ucc_geth_phy",
	},
	{
		.type = "mdio",
		.compatible = "gianfar",
	},
	{
		.compatible = "fsl,ucc-mdio",
	},
	{
		.compatible = "fsl,gianfar-tbi",
	},
	{
		.compatible = "fsl,gianfar-mdio",
	},
	{
		.compatible = "fsl,etsec2-tbi",
	},
	{
		.compatible = "fsl,etsec2-mdio",
	},
	{
		.compatible = "fsl,fman-mdio",
	},
	{},
};
MODULE_DEVICE_TABLE(of, fsl_pq_mdio_match);

static struct platform_driver fsl_pq_mdio_driver = {
	.driver = {
		.name = "fsl-pq_mdio",
		.owner = THIS_MODULE,
		.of_match_table = fsl_pq_mdio_match,
	},
	.probe = fsl_pq_mdio_probe,
	.remove = fsl_pq_mdio_remove,
};

int __init fsl_pq_mdio_init(void)
{
	return platform_driver_register(&fsl_pq_mdio_driver);
}
module_init(fsl_pq_mdio_init);

void fsl_pq_mdio_exit(void)
{
	platform_driver_unregister(&fsl_pq_mdio_driver);
}
module_exit(fsl_pq_mdio_exit);
MODULE_LICENSE("GPL");
