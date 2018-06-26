/* SPDX-License-Identifier: GPL-2.0
 *
 * dw_xpcs.h: DWC Ethernet Physical Coding Sublayer Header
 *
 * Copyright (c) 2018, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef __DW_XPCS_H__
#define __DW_XPCS_H__

#include <linux/mdio.h>
#include <linux/bitops.h>
#include "stmmac.h"

/* XPCS Control & MII MMD Device Addresses */
#define XPCS_MDIO_CONTROL_MMD	MDIO_MMD_VEND1
#define XPCS_MDIO_MII_MMD	MDIO_MMD_VEND2

/* Control MMD register offsets */
#define MDIO_CONTROL_MMD_CTRL		0x9	/* Control */

/* Control MMD Control defines */
#define MDIO_CONTROL_MMD_CTRL_MII_MMD_EN	1 /* MII MMD Enable */

/* MII MMD registers offsets */
#define MDIO_MII_MMD_CTRL		0x0	/* Control */
#define MDIO_MII_MMD_ADV		0x4	/* AN Advertisement */
#define MDIO_MII_MMD_LPA		0x5	/* Link Partner Ability */
#define MDIO_MII_MMD_DIGITAL_CTRL_1	0x8000	/* Digital Control 1 */
#define MDIO_MII_MMD_AN_CTRL		0x8001	/* AN Control */
#define MDIO_MII_MMD_AN_STAT		0x8002	/* AN Status */

/* MII MMD Control defines */
#define MDIO_MII_MMD_CTRL_ANE		BIT(12)	/* AN Enable */
#define MDIO_MII_MMD_CTRL_LBE		BIT(14)	/* Loopback Enable */
#define MDIO_MII_MMD_CTRL_RANE		BIT(9)	/* Restart AN */

/* MII MMD AN Advertisement & Link Partner Ability */
#define MDIO_MII_MMD_HD			BIT(6)	/* Half duplex */
#define MDIO_MII_MMD_FD			BIT(5)	/* Full duplex */
#define MDIO_MII_MMD_PSE_SHIFT		7	/* Pause Ability shift */
#define MDIO_MII_MMD_PSE	GENMASK(8, 7)	/* Pause Ability */

/* MII MMD Digital Control 1 defines */
#define MDIO_MII_MMD_DIGI_CTRL_1_SGMII_PHY_MD	BIT(0) /* SGMII PHY mode */

/* MII MMD AN Control defines */
#define MDIO_MII_MMD_AN_CTRL_TX_CONFIG_SHIFT	3 /* TX Config shift */
#define AN_CTRL_TX_CONF_PHY_SIDE_SGMII		0x1 /* PHY side SGMII mode */
#define AN_CTRL_TX_CONF_MAC_SIDE_SGMII		0x0 /* MAC side SGMII mode */
#define MDIO_MII_MMD_AN_CTRL_PCS_MD_SHIFT	1  /* PCS Mode shift */
#define MDIO_MII_MMD_AN_CTRL_PCS_MD	GENMASK(2, 1) /* PCS Mode */
#define AN_CTRL_PCS_MD_C37_1000BASEX	0x0	/* C37 AN for 1000BASE-X */
#define AN_CTRL_PCS_MD_C37_SGMII	0x2	/* C37 AN for SGMII */
#define MDIO_MII_MMD_AN_CTRL_AN_INTR_EN	BIT(0)	/* AN Complete Intr Enable */

/* MII MMD AN Status defines for C37 AN SGMII Status */
#define AN_STAT_C37_AN_CMPLT		BIT(0)	/* AN Complete Intr */
#define AN_STAT_C37_AN_FD		BIT(1)	/* Full Duplex */
#define AN_STAT_C37_AN_SPEED_SHIFT	2	/* AN Speed shift */
#define AN_STAT_C37_AN_SPEED		GENMASK(3, 2)	/* AN Speed */
#define AN_STAT_C37_AN_10MBPS		0x0	/* 10 Mbps */
#define AN_STAT_C37_AN_100MBPS		0x1	/* 100 Mbps */
#define AN_STAT_C37_AN_1000MBPS		0x2	/* 1000 Mbps */
#define AN_STAT_C37_AN_LNKSTS		BIT(4)	/* Link Status */

/**
 * dw_xpcs_init - To initialize xPCS
 * @ndev: network device pointer
 * @mode: PCS mode
 * Description: this is to initialize xPCS
 */
static inline void dw_xpcs_init(struct net_device *ndev, int pcs_mode)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int xpcs_phy_addr = priv->plat->xpcs_phy_addr;

	/* Set SGMII PHY mode control */
	u16 phydata = (u16)mdiobus_read(priv->mii, xpcs_phy_addr,
				    (MII_ADDR_C45 |
				     (XPCS_MDIO_MII_MMD <<
				      MII_DEVADDR_C45_SHIFT) |
				     MDIO_MII_MMD_DIGITAL_CTRL_1));

	phydata |= MDIO_MII_MMD_DIGI_CTRL_1_SGMII_PHY_MD;

	mdiobus_write(priv->mii, xpcs_phy_addr,
		      (MII_ADDR_C45 | (XPCS_MDIO_MII_MMD <<
		       MII_DEVADDR_C45_SHIFT) | MDIO_MII_MMD_DIGITAL_CTRL_1),
		      (int)phydata);

	/* Set PHY side SGMII, PCS Mode & Enable C37 AN complete interrupt */
	phydata = (u16)mdiobus_read(priv->mii, xpcs_phy_addr,
				    (MII_ADDR_C45 |
				     (XPCS_MDIO_MII_MMD <<
				      MII_DEVADDR_C45_SHIFT) |
				     MDIO_MII_MMD_AN_CTRL));

	phydata &= ~MDIO_MII_MMD_AN_CTRL_PCS_MD;
	phydata |= (((pcs_mode << MDIO_MII_MMD_AN_CTRL_PCS_MD_SHIFT) &
		    MDIO_MII_MMD_AN_CTRL_PCS_MD) |
		    (AN_CTRL_TX_CONF_PHY_SIDE_SGMII <<
		    MDIO_MII_MMD_AN_CTRL_TX_CONFIG_SHIFT) |
		    MDIO_MII_MMD_AN_CTRL_AN_INTR_EN);

	mdiobus_write(priv->mii, xpcs_phy_addr,
		      (MII_ADDR_C45 | (XPCS_MDIO_MII_MMD <<
		       MII_DEVADDR_C45_SHIFT) | MDIO_MII_MMD_AN_CTRL),
		      (int)phydata);
}

/**
 * dw_xpcs_rane - To restart Auto Negotiation (AN)
 * @ndev: network device pointer
 * @restart: to restart AN
 * Description: this is to restart AN.
 */
static inline void dw_xpcs_rane(struct net_device *ndev, bool restart)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int xpcs_phy_addr = priv->plat->xpcs_phy_addr;

	u16 phydata = (u16)mdiobus_read(priv->mii, xpcs_phy_addr,
				(MII_ADDR_C45 | (XPCS_MDIO_MII_MMD <<
				 MII_DEVADDR_C45_SHIFT) |
				 MDIO_MII_MMD_CTRL));

	if (restart)
		phydata |= MDIO_MII_MMD_CTRL_RANE;

	mdiobus_write(priv->mii, xpcs_phy_addr,
		      (MII_ADDR_C45 | (XPCS_MDIO_MII_MMD <<
		       MII_DEVADDR_C45_SHIFT) | MDIO_MII_MMD_CTRL),
		      (int)phydata);
}

/**
 * dw_xpcs_ctrl_ane - To program the MII MMD Control Register.
 * @ndev: network device pointer
 * @ane: to enable the Auto Negotiation
 * @loopback: to cause the PHY to loopback Tx data into Rx path.
 * Description: this is the main function to configure the MII MMD
 * control register and init the AN Enable and select loopback.
 */
static inline void dw_xpcs_ctrl_ane(struct net_device *ndev, bool ane,
				    bool loopback)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int xpcs_phy_addr = priv->plat->xpcs_phy_addr;

	u16 phydata = (u16)mdiobus_read(priv->mii, xpcs_phy_addr,
				(MII_ADDR_C45 | (XPCS_MDIO_MII_MMD <<
				 MII_DEVADDR_C45_SHIFT) |
				 MDIO_MII_MMD_CTRL));

	if (ane)
		phydata |= (MDIO_MII_MMD_CTRL_ANE | MDIO_MII_MMD_CTRL_RANE);

	if (loopback)
		phydata |= MDIO_MII_MMD_CTRL_LBE;

	mdiobus_write(priv->mii, xpcs_phy_addr,
		      (MII_ADDR_C45 | (XPCS_MDIO_MII_MMD <<
		       MII_DEVADDR_C45_SHIFT) | MDIO_MII_MMD_CTRL),
		      (int)phydata);
}

/**
 * dw_xpcs_get_adv_lp - Get AN Advertisement and Link Partner Ability
 * @ndev: network device pointer
 * @adv_lp: structure to store the adv, lp status
 * Description: this is to expose the Auto Negotiation Advertisement and
 * Link partner ability status to ethtool support.
 */
static inline void dw_xpcs_get_adv_lp(struct net_device *ndev,
				      struct rgmii_adv *adv_lp)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int xpcs_phy_addr = priv->plat->xpcs_phy_addr;

	/* AN Advertisement Ability */
	u16 value = (u16)mdiobus_read(priv->mii, xpcs_phy_addr,
				(MII_ADDR_C45 | (XPCS_MDIO_MII_MMD <<
				 MII_DEVADDR_C45_SHIFT) |
				 MDIO_MII_MMD_ADV));

	if (value & MDIO_MII_MMD_FD)
		adv_lp->duplex = DUPLEX_FULL;
	if (value & MDIO_MII_MMD_HD)
		adv_lp->duplex = DUPLEX_HALF;
	adv_lp->pause = (u32)((value & MDIO_MII_MMD_PSE) >>
			      MDIO_MII_MMD_PSE_SHIFT);

	/* Link Partner Ability */
	value = (u16)mdiobus_read(priv->mii, xpcs_phy_addr,
				  (MII_ADDR_C45 | (XPCS_MDIO_MII_MMD <<
				   MII_DEVADDR_C45_SHIFT) |
				   MDIO_MII_MMD_LPA));

	if (value & MDIO_MII_MMD_FD)
		adv_lp->lp_duplex = DUPLEX_FULL;
	if (value & MDIO_MII_MMD_HD)
		adv_lp->lp_duplex = DUPLEX_HALF;
	adv_lp->lp_pause = (u32)((value & MDIO_MII_MMD_PSE) >>
				 MDIO_MII_MMD_PSE_SHIFT);
}

/**
 * dw_xpcs_get_linkstatus - Get Link Status
 * @an_stat: C37 AN status value
 * @x: stmmac extra status
 * Description: this is to read the link extra status from <C37 AN SGMII
 * status> field of MII MMD AN Status register.
 */
static inline void dw_xpcs_get_linkstatus(u16 an_stat,
					  struct stmmac_extra_stats *x)
{
	/* Check the link status */
	if (an_stat & AN_STAT_C37_AN_LNKSTS) {
		int speed_value;

		x->pcs_link = 1;

		speed_value = ((an_stat & AN_STAT_C37_AN_SPEED) >>
				AN_STAT_C37_AN_SPEED_SHIFT);
		if (speed_value == AN_STAT_C37_AN_1000MBPS)
			x->pcs_speed = SPEED_1000;
		else if (speed_value == AN_STAT_C37_AN_100MBPS)
			x->pcs_speed = SPEED_100;
		else
			x->pcs_speed = SPEED_10;

		if (an_stat & AN_STAT_C37_AN_FD)
			x->pcs_duplex = 1;
		else
			x->pcs_duplex = 0;

		pr_info("Link is Up - %d/%s\n", (int)x->pcs_speed,
			x->pcs_duplex ? "Full" : "Half");
	} else {
		x->pcs_link = 0;
		pr_info("Link is Down\n");
	}
}

/**
 * dw_xpcs_irq_status - Get xPCS IRQ Status
 * @ndev: network device pointer
 * @x: stmmac extra status
 * Description: this is to read the xPCS IRQ status.
 */
static inline int dw_xpcs_irq_status(struct net_device *ndev,
				     struct stmmac_extra_stats *x)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int xpcs_phy_addr = priv->plat->xpcs_phy_addr;
	int ret = IRQ_NONE;

	/* C37 AN status */
	u16 an_stat = (u16)mdiobus_read(priv->mii, xpcs_phy_addr,
					(MII_ADDR_C45 |
					 (XPCS_MDIO_MII_MMD <<
					  MII_DEVADDR_C45_SHIFT) |
					 MDIO_MII_MMD_AN_STAT));

	if (an_stat & AN_STAT_C37_AN_CMPLT) {
		x->irq_pcs_ane_n++;
		dw_xpcs_get_linkstatus(an_stat, x);

		/* Clear C37 AN complete status by writing zero */
		mdiobus_write(priv->mii, xpcs_phy_addr,
			      (MII_ADDR_C45 | (XPCS_MDIO_MII_MMD <<
			       MII_DEVADDR_C45_SHIFT) |
			       MDIO_MII_MMD_AN_STAT),
			      0);
		ret = IRQ_HANDLED;
	}

	return ret;
}
#endif /* __DW_XPCS_H__ */
