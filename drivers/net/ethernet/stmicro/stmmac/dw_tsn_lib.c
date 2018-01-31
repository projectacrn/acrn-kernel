/*
 * dw_tsn_lib.c: DW EQoS v5.00 TSN capabilities
 *
 * Copyright (c) 2017, Intel Corporation.
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

#include <linux/ethtool.h>
#include "stmmac.h"
#include "dwmac5.h"
#include "descs.h"

static struct tsn_hw_tunable dw_tsn_hwtunable;
static struct est_gc_config dw_est_gc_config;
static struct tsn_err_stat dw_err_stat;
static struct fpe_config dw_fpe_config;

#define ONE_SEC_IN_NANOSEC 1000000000ULL

static u32 est_get_gcl_depth(u32 hw_cap)
{
	u32 depth;
	u32 estdep = (hw_cap & GMAC_HW_FEAT_ESTDEP)
			>> GMAC_HW_FEAT_ESTDEP_SHIFT;

	switch (estdep) {
	case 1:
		depth = 64;
		break;
	case 2:
		depth = 128;
		break;
	case 3:
		depth = 256;
		break;
	case 4:
		depth = 512;
		break;
	case 5:
		depth = 1024;
		break;
	default:
		depth = 0;
	}

	return depth;
}

static u32 est_get_ti_width(u32 hw_cap)
{
	u32 width;
	u32 estwid = (hw_cap & GMAC_HW_FEAT_ESTWID)
			>> GMAC_HW_FEAT_ESTWID_SHIFT;

	switch (estwid) {
	case 1:
		width = 16;
		break;
	case 2:
		width = 20;
		break;
	case 3:
		width = 24;
		break;
	default:
		width = 0;
	}

	return width;
}

static int est_poll_srwo(void __iomem *ioaddr)
{
	/* Poll until the EST GCL Control[SRWO] bit clears.
	 * Total wait = 12 x 50ms ~= 0.6s.
	 */
	u32 retries = 12;
	u32 value;

	do {
		value = readl(ioaddr + MTL_EST_GCL_CTRL);
		if (!(value & MTL_EST_GCL_CTRL_SRWO))
			return 0;
		msleep(50);
	} while (--retries);

	return -ETIMEDOUT;
}

static int est_set_gcl_addr(void __iomem *ioaddr,
			    u32 addr, u32 gcrr, u32 rwops,
			    u32 dbgb, u32 dbgm)
{
	u32 value;

	value = MTL_EST_GCL_CTRL_ADDR_VAL(addr) & MTL_EST_GCL_CTRL_ADDR;

	if (dbgm) {
		if (dbgb)
			value |= MTL_EST_GCL_CTRL_DBGB1;

		value |= MTL_EST_GCL_CTRL_DBGM;
	}

	if (gcrr)
		value |= MTL_EST_GCL_CTRL_GCRR;

	/* This is the only place SRWO is set and driver polls SRWO
	 * for self-cleared before exit. Therefore, caller should
	 * check return status for possible time out error.
	 */
	value |= (rwops | MTL_EST_GCL_CTRL_SRWO);

	writel(value, ioaddr + MTL_EST_GCL_CTRL);

	return est_poll_srwo(ioaddr);
}

static int est_write_gcl_config(void __iomem *ioaddr, u32 data,
				u32 addr, u32 gcrr,
				u32 dbgb, u32 dbgm)
{
	writel(data, ioaddr + MTL_EST_GCL_DATA);

	return est_set_gcl_addr(ioaddr, addr, gcrr, GCL_OPS_W, dbgb, dbgm);
}

static int est_read_gcl_config(void __iomem *ioaddr, u32 *data,
			       u32 addr, u32 gcrr,
			       u32 dbgb, u32 dbgm)
{
	int ret;

	ret = est_set_gcl_addr(ioaddr, addr, gcrr, GCL_OPS_R, dbgb, dbgm);
	if (ret)
		return ret;

	*data = readl(ioaddr + MTL_EST_GCL_DATA);

	return ret;
}

static int est_read_gce(struct net_device *ndev, u32 row,
			u32 *gates, u32 *ti_nsec,
			u32 dbgb, u32 dbgm)
{
	int ret;
	u32 value;
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;
	struct tsn_hw_cap *cap = &priv->tsn_hwcap;
	u32 ti_wid = cap->ti_wid;
	u32 gates_mask = (1 << cap->txqcnt) - 1;
	u32 ti_mask = (1 << ti_wid) - 1;

	ret = est_read_gcl_config(ioaddr, &value, row, 0, dbgb, dbgm);
	if (ret) {
		dev_err(priv->device, "Read GCE failed! row=%u\n", row);

		return ret;
	}
	*ti_nsec = value & ti_mask;
	*gates = (value >> ti_wid) & gates_mask;

	return ret;
}

static u32 est_get_gcl_total_intervals_nsec(u32 bank, u32 gcl_len)
{
	u32 row;
	u32 nsec = 0;
	struct est_gc_entry *gcl = dw_est_gc_config.gcb[bank].gcl;

	for (row = 0; row < gcl_len; row++) {
		nsec += gcl->ti_nsec;
		gcl++;
	}

	return nsec;
}

int dwmac_tsn_init(struct net_device *ndev)
{
	u32 gcl_depth;
	u32 tils_max;
	u32 ti_wid;
	u32 bank;
	int ret = 0;
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;
	struct tsn_hw_cap *cap = &priv->tsn_hwcap;
	u32 hwid = readl(ioaddr + GMAC_VERSION);
	u32 hw_cap2 = readl(ioaddr + GMAC_HW_FEATURE2);
	u32 hw_cap3 = readl(ioaddr + GMAC_HW_FEATURE3);
	u32 value;

	memset(cap, 0, sizeof(*cap));

	value = stmmac_get_synopsys_id(hwid);
	if (value < DWMAC_CORE_5_00) {
		dev_info(priv->device,
			 "IP v5.00 does not support TSN\n");
		return 0;
	}

	if (!(hw_cap3 & GMAC_HW_FEAT_ESTSEL)) {
		dev_info(priv->device,
			 "EST NOT supported\n");
		cap->est_support = 0;

		goto check_fpe;
	}

	gcl_depth = est_get_gcl_depth(hw_cap3);

	for (bank = 0; bank < EST_GCL_BANK_MAX; bank++) {
		struct est_gc_entry *gcl;

		gcl = devm_kzalloc(priv->device, (sizeof(*gcl) * gcl_depth),
				   GFP_KERNEL);
		if (!gcl) {
			ret = -ENOMEM;
			break;
		}

		dw_est_gc_config.gcb[bank].gcl = gcl;
	}

	/* Handle -ENOMEM during GCL allocation. The design can handle
	 * for EST_GCL_BANK_MAX is greater than 2.
	 */
	if (ret) {
		int i;

		for (i = bank - 1; i >= 0; i--) {
			devm_kfree(priv->device, dw_est_gc_config.gcb[i].gcl);
			dw_est_gc_config.gcb[i].gcl = NULL;
		}
		dev_warn(priv->device, "EST: GCL -ENOMEM\n");

		return ret;
	}

	ti_wid = est_get_ti_width(hw_cap3);
	cap->gcl_depth = gcl_depth;
	cap->ti_wid = ti_wid;

	tils_max = (hw_cap3 & GMAC_HW_FEAT_ESTTISW)
		   >> GMAC_HW_FEAT_ESTTISW_SHIFT;
	tils_max = (1 << tils_max) - 1;
	cap->tils_max = tils_max;

	cap->ext_max = EST_TIWID_TO_EXTMAX(ti_wid);
	cap->txqcnt = ((hw_cap2 & GMAC_HW_FEAT_TXQCNT) >> 6) + 1;
	cap->est_support = 1;

	dev_info(priv->device,
		 "EST: depth=%u, ti_wid=%u, tils_max=%u tqcnt=%u\n",
		 gcl_depth, ti_wid, tils_max, cap->txqcnt);

check_fpe:
	if (!(hw_cap3 & GMAC_HW_FEAT_FPESEL)) {
		dev_info(priv->device, "FPE NOT supported\n");
		cap->fpe_support = 0;
		goto check_tbs;
	} else {
		dev_info(priv->device, "FPE capable\n");
		cap->rxqcnt = (hw_cap2 & GMAC_HW_FEAT_RXQCNT) + 1;
		cap->fpe_support = 1;
	}

check_tbs:
	if (!(hw_cap3 & GMAC_HW_FEAT_TBSSEL)) {
		dev_info(priv->device, "TBS NOT supported\n");
	} else {
		dev_info(priv->device, "TBS capable\n");
		if (priv->plat->has_tbs) {
			priv->enhanced_tx_desc = true;
			priv->mode = STMMAC_ENHANCED_TX_MODE;
		}
	}

	return 0;
}

/* dwmac_tsn_setup is called within stmmac_hw_setup() after
 * stmmac_init_dma_engine() which resets MAC controller.
 * This is so-that MAC registers are not cleared.
 */
void dwmac_tsn_setup(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;
	struct tsn_hw_cap *cap = &priv->tsn_hwcap;
	u32 value;

	if (cap->est_support) {
		/* Enable EST interrupts */
		value = (MTL_EST_INT_EN_CGCE | MTL_EST_INT_EN_IEHS |
			 MTL_EST_INT_EN_IEHF | MTL_EST_INT_EN_IEBE |
			 MTL_EST_INT_EN_IECC);
		writel(value, ioaddr + MTL_EST_INT_EN);
	}

	if (cap->fpe_support && priv->plat->fprq <= cap->rxqcnt) {
		/* Update FPRQ */
		value = readl(ioaddr + GMAC_RXQ_CTRL1);
		value &= ~GMAC_RXQCTRL_FPRQ_MASK;
		value |= priv->plat->fprq << GMAC_RXQCTRL_FPRQ_SHIFT;
		writel(value, ioaddr + GMAC_RXQ_CTRL1);
	} else {
		dev_warn(priv->device, "FPE: FPRQ is out-of-bound.\n");
	}
}

static int est_set_tils(struct net_device *ndev, u32 tils)
{
	u32 value;
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;
	struct tsn_hw_cap *cap = &priv->tsn_hwcap;

	if (!priv->tsn_est)
		return -ENOTSUPP;

	if (tils > cap->tils_max) {
		dev_warn(priv->device, "EST: invalid tils(%u), max=%u\n",
			 tils, cap->tils_max);

		return -EINVAL;
	}

	/* Ensure that HW is not in the midst of GCL transition */
	value = readl(ioaddr + MTL_EST_CTRL);
	value &= ~MTL_EST_CTRL_SSWL;

	/* MTL_EST_CTRL value has been read earlier, if TILS value
	 * differs, we update here.
	 */
	if (tils != dw_tsn_hwtunable.tils) {
		value &= ~MTL_EST_CTRL_TILS;
		value |= (tils << MTL_EST_CTRL_TILS_SHIFT);

		writel(value, ioaddr + MTL_EST_CTRL);
		dw_tsn_hwtunable.tils = tils;
	}

	return 0;
}

static int est_set_ov(struct net_device *ndev, u32 *ptov, u32 *ctov)
{
	u32 value;
	struct stmmac_priv *priv = netdev_priv(ndev);

	if (!priv->tsn_est)
		return -ENOTSUPP;

	value = readl(priv->ioaddr + MTL_EST_CTRL);
	value &= ~MTL_EST_CTRL_SSWL;

	if (ptov) {
		if (*ptov > EST_PTOV_MAX) {
			dev_warn(priv->device,
				 "EST: invalid PTOV(%u), max=%u\n",
				 *ptov, EST_PTOV_MAX);

			return -EINVAL;
		} else if (*ptov != dw_tsn_hwtunable.ptov) {
			value &= ~MTL_EST_CTRL_PTOV;
			value |= (*ptov << MTL_EST_CTRL_PTOV_SHIFT);
			dw_tsn_hwtunable.ptov = *ptov;
		}
	}

	if (ctov) {
		if (*ctov > EST_CTOV_MAX) {
			dev_warn(priv->device,
				 "EST: invalid CTOV(%u), max=%u\n",
				 *ctov, EST_CTOV_MAX);

			return -EINVAL;
		} else if (*ctov != dw_tsn_hwtunable.ctov) {
			value &= ~MTL_EST_CTRL_CTOV;
			value |= (*ctov << MTL_EST_CTRL_CTOV_SHIFT);
			dw_tsn_hwtunable.ctov = *ctov;
		}
	}

	writel(value, priv->ioaddr + MTL_EST_CTRL);

	return 0;
}

static int fpe_set_afsz(struct net_device *ndev, u32 afsz)
{
	u32 value;
	struct stmmac_priv *priv = netdev_priv(ndev);

	if (!priv->tsn_fpe)
		return -ENOTSUPP;

	if (afsz > FPE_AFSZ_MAX) {
		dev_warn(priv->device, "FPE: AFSZ is out-of-bound.\n");

		return -EINVAL;
	}

	if (afsz != dw_tsn_hwtunable.afsz) {
		value = readl(priv->ioaddr + MTL_FPE_CTRL_STS);
		value &= ~MTL_FPE_CTRL_STS_AFSZ;
		value |= afsz;
		writel(value, priv->ioaddr + MTL_FPE_CTRL_STS);
		dw_tsn_hwtunable.afsz = afsz;
	}

	return 0;
}

static int fpe_set_hr_adv(struct net_device *ndev, u32 *hadv, u32 *radv)
{
	u32 value;
	struct stmmac_priv *priv = netdev_priv(ndev);

	if (!priv->tsn_fpe)
		return -ENOTSUPP;

	value = readl(priv->ioaddr + MTL_FPE_ADVANCE);

	if (hadv) {
		if (*hadv > FPE_ADV_MAX) {
			dev_warn(priv->device,
				 "FPE: invalid HADV(%u), max=%u\n",
				 *hadv, FPE_ADV_MAX);

			return -EINVAL;
		} else if (*hadv != dw_tsn_hwtunable.hadv) {
			value &= ~MTL_FPE_ADVANCE_HADV;
			value |= (*hadv & MTL_FPE_ADVANCE_HADV);
			dw_tsn_hwtunable.hadv = *hadv;
		}
	}

	if (radv) {
		if (*radv > FPE_ADV_MAX) {
			dev_warn(priv->device,
				 "FPE: invalid RADV(%u), max=%u\n",
				 *radv, FPE_ADV_MAX);

			return -EINVAL;
		} else if (*radv != dw_tsn_hwtunable.radv) {
			value &= ~MTL_FPE_ADVANCE_RADV;
			value |= ((*radv << MTL_FPE_ADVANCE_RADV_SHIFT) &
				  MTL_FPE_ADVANCE_RADV);
			dw_tsn_hwtunable.radv = *radv;
		}
	}

	writel(value, priv->ioaddr + MTL_FPE_ADVANCE);

	return 0;
}

static int tbs_set_leos(struct net_device *ndev, u32 leos_ns)
{
	u32 value;
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;
	/* get hw own list */
	u32 hw_bank = dwmac_get_est_bank(ndev, 0);
	struct est_gc_config *gcc = &dw_est_gc_config;
	struct est_gc_bank *gcbc = &gcc->gcb[hw_bank];

	/* Launch expiry offset is in unit of 256ns
	 * Get the actual leos_ns value
	 */
	leos_ns &= MTL_TBS_CTRL_LEOS;

	if (leos_ns > TBS_LEOS_MAX) {
		dev_err(priv->device,
			"TBS: Max LEOS is 999999999ns.\n");

		return -EINVAL;
	}

	if (dw_tsn_hwtunable.est_mode) {
		if (leos_ns > (gcbc->gcrr.cycle_nsec - 1)) {
			dev_err(priv->device,
				"TBS: LEOS > (cycle time - 1ns)\n");

			return -EINVAL;
		}
	}

	value = readl(ioaddr + MTL_TBS_CTRL);

	/* Launch expiry offset not valid when launch
	 * expiry offset value is 0 and vice versa
	 */
	if (leos_ns || (dw_tsn_hwtunable.est_mode && dw_tsn_hwtunable.legos))
		value |= MTL_TBS_CTRL_LEOV;
	else
		value &= ~MTL_TBS_CTRL_LEOV;

	value &= ~MTL_TBS_CTRL_LEOS;
	value |= leos_ns;
	dw_tsn_hwtunable.leos_ns = leos_ns;

	writel(value, ioaddr + MTL_TBS_CTRL);

	return 0;
}

static int tbs_set_legos(struct net_device *ndev, u32 legos)
{
	u32 value;
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;

	/* if EST not turn on, ret fail */
	if (!(dw_est_gc_config.enable && dw_tsn_hwtunable.est_mode)) {
		dev_warn(priv->device, "TBS EST mode is not enabled\n");

		return -EINVAL;
	}

	if (legos > 7) {
		dev_err(priv->device, "TBS: LEGOS > 7.\n");

		return -EINVAL;
	}

	value = readl(ioaddr + MTL_TBS_CTRL);
	if (dw_tsn_hwtunable.leos_ns || legos)
		value |= MTL_TBS_CTRL_LEOV;
	else
		value &= ~MTL_TBS_CTRL_LEOV;
	value &= ~MTL_TBS_CTRL_LEGOS;
	value |= (legos << MTL_TBS_CTRL_LEGOS_SHIFT) &
		 MTL_TBS_CTRL_LEGOS;
	dw_tsn_hwtunable.legos = legos;

	writel(value, ioaddr + MTL_TBS_CTRL);

	return 0;
}

static int tbs_set_est_mode(struct net_device *ndev, u32 mode)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	u32 data = readl(priv->ioaddr + MTL_TBS_CTRL);

	if (mode) {
		if (!dw_est_gc_config.enable)
			return -EINVAL;

		data |= MTL_TBS_CTRL_ESTM;
		dw_tsn_hwtunable.est_mode = true;
	} else {
		data &= ~MTL_TBS_CTRL_ESTM;
		dw_tsn_hwtunable.est_mode = false;
	}

	writel(data, priv->ioaddr + MTL_TBS_CTRL);

	return 0;
}

static int tbs_set_ftos(struct net_device *ndev, u32 offset)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	u32 data = readl(priv->ioaddr + DMA_TBS_CTRL);

	if (offset > DMA_TBS_CTRL_FTOS_MAX) {
		dev_warn(priv->device,
			 "Input offset value exceeded the max range\n");

		return -EINVAL;
	}

	if (dw_tsn_hwtunable.est_mode) {
		int bank = dwmac_get_est_bank(ndev, 0);
		u32 ctr_ns = dw_est_gc_config.gcb[bank].gcrr.cycle_nsec;

		if (offset > (ctr_ns - 1)) {
			dev_warn(priv->device,
				 "Input offset value exceeded cycle time\n");

			return -EINVAL;
		}
	}

	/* unset the valid bit for updating new fetch time offset */
	data &= ~DMA_TBS_CTRL_FTOV;
	writel(data, priv->ioaddr + DMA_TBS_CTRL);

	data &= ~DMA_TBS_CTRL_FTOS;
	dw_tsn_hwtunable.ftos_ns = offset & DMA_TBS_CTRL_FTOS;
	data |= dw_tsn_hwtunable.ftos_ns << DMA_TBS_CTRL_FTOS_SHIFT;

	/* disable fetch time while it is zero */
	if (dw_tsn_hwtunable.ftos_ns ||
	    (dw_tsn_hwtunable.est_mode && dw_tsn_hwtunable.fgos))
		data |= DMA_TBS_CTRL_FTOV;

	writel(data, priv->ioaddr + DMA_TBS_CTRL);

	return 0;
}

static int tbs_set_fgos(struct net_device *ndev, u32 slot)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	u32 data = readl(priv->ioaddr + DMA_TBS_CTRL);

	if (!(dw_est_gc_config.enable && dw_tsn_hwtunable.est_mode)) {
		dev_warn(priv->device, "TBS EST mode is not enabled\n");

		return -EINVAL;
	}

	if (slot > DMA_TBS_CTRL_FGOS_MASK) {
		dev_warn(priv->device,
			 "Input GSN value exceeded the max range\n");

		return -EINVAL;
	}

	/* unset the valid bit for updating new fetch GSN slot */
	data &= ~DMA_TBS_CTRL_FTOV;
	writel(data, priv->ioaddr + DMA_TBS_CTRL);

	data &= ~DMA_TBS_CTRL_FGOS;
	dw_tsn_hwtunable.fgos = slot & DMA_TBS_CTRL_FGOS_MASK;
	data |= (dw_tsn_hwtunable.fgos << DMA_TBS_CTRL_FGOS_SHIFT);

	/* disable fetch time while it is zero */
	if (dw_tsn_hwtunable.ftos_ns || dw_tsn_hwtunable.fgos)
		data |= DMA_TBS_CTRL_FTOV;

	writel(data, priv->ioaddr + DMA_TBS_CTRL);

	return 0;
}

int dwmac_set_tsn_hwtunable(struct net_device *ndev, u32 id,
			    const void *data)
{
	int ret = 0;
	u32 value = *(u32 *)data;

	switch (id) {
	case ETHTOOL_TX_EST_TILS:
		ret = est_set_tils(ndev, value);
		break;
	case ETHTOOL_TX_EST_PTOV:
		ret = est_set_ov(ndev, &value, NULL);
		break;
	case ETHTOOL_TX_EST_CTOV:
		ret = est_set_ov(ndev, NULL, &value);
		break;
	case ETHTOOL_TX_FPE_AFSZ:
		ret = fpe_set_afsz(ndev, value);
		break;
	case ETHTOOL_TX_FPE_HADV:
		ret = fpe_set_hr_adv(ndev, &value, NULL);
		break;
	case ETHTOOL_TX_FPE_RADV:
		ret = fpe_set_hr_adv(ndev, NULL, &value);
		break;
	case ETHTOOL_TX_TBS_ESTM:
		ret = tbs_set_est_mode(ndev, value);
		break;
	case ETHTOOL_TX_TBS_FTOS:
		ret = tbs_set_ftos(ndev, value);
		break;
	case ETHTOOL_TX_TBS_FGOS:
		ret = tbs_set_fgos(ndev, value);
		break;
	case ETHTOOL_TX_TBS_LEOS:
		ret = tbs_set_leos(ndev, value);
		break;
	case ETHTOOL_TX_TBS_LEGOS:
		ret = tbs_set_legos(ndev, value);
		break;
	default:
		ret = -EINVAL;
	};

	return ret;
}

int dwmac_get_tsn_hwtunable(struct net_device *ndev, u32 id,
			    void *data)
{
	int ret = 0;

	switch (id) {
	case ETHTOOL_TX_EST_TILS:
		*(u32 *)data = dw_tsn_hwtunable.tils;
		break;
	case ETHTOOL_TX_EST_PTOV:
		*(u32 *)data = dw_tsn_hwtunable.ptov;
		break;
	case ETHTOOL_TX_EST_CTOV:
		*(u32 *)data = dw_tsn_hwtunable.ctov;
		break;
	case ETHTOOL_TX_FPE_AFSZ:
		*(u32 *)data = dw_tsn_hwtunable.afsz;
		break;
	case ETHTOOL_TX_FPE_HADV:
		*(u32 *)data = dw_tsn_hwtunable.hadv;
		break;
	case ETHTOOL_TX_FPE_RADV:
		*(u32 *)data = dw_tsn_hwtunable.radv;
		break;
	case ETHTOOL_TX_TBS_ESTM:
		*(u32 *)data = dw_tsn_hwtunable.est_mode;
		break;
	case ETHTOOL_TX_TBS_FTOS:
		*(u32 *)data = dw_tsn_hwtunable.ftos_ns;
		break;
	case ETHTOOL_TX_TBS_FGOS:
		*(u32 *)data = dw_tsn_hwtunable.fgos;
		break;
	case ETHTOOL_TX_TBS_LEOS:
		*(u32 *)data = dw_tsn_hwtunable.leos_ns;
		break;
	case ETHTOOL_TX_TBS_LEGOS:
		*(u32 *)data = dw_tsn_hwtunable.legos;
		break;
	default:
		ret = -EINVAL;
	};

	return ret;
}

int dwmac_get_est_bank(struct net_device *ndev, u32 own)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;
	int swol = readl(ioaddr + MTL_EST_STATUS);

	swol = ((swol & MTL_EST_STATUS_SWOL) >>
		MTL_EST_STATUS_SWOL_SHIFT);

	if (own)
		return swol;
	else
		return (~swol & 0x1);
}

int dwmac_set_est_gce(struct net_device *ndev,
		      struct est_gc_entry *gce, u32 row,
		      u32 dbgb, u32 dbgm)
{
	u32 value;
	u32 ti_wid;
	u32 ti_max;
	u32 gates_mask;
	u32 bank;
	struct est_gc_entry *gcl;
	int ret;
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tsn_hw_cap *cap = &priv->tsn_hwcap;
	void __iomem *ioaddr = priv->ioaddr;
	u32 gates = gce->gates;
	u32 ti_nsec = gce->ti_nsec;

	if (!priv->tsn_est)
		return -ENOTSUPP;

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	if (dbgm) {
		bank = dbgb;
	} else {
		value = readl(ioaddr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	if (!cap->gcl_depth || row > cap->gcl_depth) {
		dev_warn(priv->device, "EST: row(%u) > GCL depth(%u)\n",
			 row, cap->gcl_depth);

		return -EINVAL;
	}

	ti_wid = cap->ti_wid;
	ti_max = (1 << ti_wid) - 1;
	if (ti_nsec > ti_max) {
		dev_warn(priv->device,
			 "EST: ti_nsec(%u) > upper limit(%u)\n",
			 ti_nsec, ti_max);

		return -EINVAL;
	}

	gates_mask = (1 << cap->txqcnt) - 1;
	value = ((gates & gates_mask) << ti_wid) | ti_nsec;

	ret = est_write_gcl_config(ioaddr, value, row, 0, dbgb, dbgm);
	if (ret) {
		dev_err(priv->device,
			"EST: GCE write failed: bank=%u row=%u.\n",
			bank, row);

		return ret;
	}

	dev_info(priv->device,
		 "EST: GCE write: dbgm=%u bank=%u row=%u, gc=0x%x.\n",
		 dbgm, bank, row, value);

	/* Since GC write is successful, update GCL copy of the driver */
	gcl = dw_est_gc_config.gcb[bank].gcl + row;
	gcl->gates = gates;
	gcl->ti_nsec = ti_nsec;

	return ret;
}

int dwmac_get_est_gcrr_llr(struct net_device *ndev, u32 *gcl_len,
			   u32 dbgb, u32 dbgm)
{
	u32 bank, value;
	int ret;
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;

	if (!priv->tsn_est)
		return -ENOTSUPP;

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	if (dbgm) {
		bank = dbgb;
	} else {
		value = readl(ioaddr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	ret = est_read_gcl_config(ioaddr, &value,
				  GCL_CTRL_ADDR_LLR, 1,
				  dbgb, dbgm);
	if (ret) {
		dev_err(priv->device,
			"read LLR fail at bank=%u\n", bank);

			return ret;
	}

	*gcl_len = value;

	return 0;
}

int dwmac_set_est_gcrr_llr(struct net_device *ndev, u32 gcl_len,
			   u32 dbgb, u32 dbgm)
{
	u32 bank, value;
	struct est_gcrr *bgcrr;
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tsn_hw_cap *cap = &priv->tsn_hwcap;
	void __iomem *ioaddr = priv->ioaddr;
	int ret = 0;

	if (!priv->tsn_est)
		return -ENOTSUPP;

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	if (dbgm) {
		bank = dbgb;
	} else {
		value = readl(ioaddr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	if (gcl_len > cap->gcl_depth) {
		dev_warn(priv->device,
			 "EST: GCL length(%u) > depth(%u)\n",
			 gcl_len, cap->gcl_depth);

		return -EINVAL;
	}

	bgcrr = &dw_est_gc_config.gcb[bank].gcrr;

	if (gcl_len != bgcrr->llr) {
		ret = est_write_gcl_config(ioaddr, gcl_len,
					   GCL_CTRL_ADDR_LLR, 1,
					   dbgb, dbgm);
		if (ret) {
			dev_err(priv->device,
				"EST: GCRR programming failure!\n");

			return ret;
		}
		bgcrr->llr = gcl_len;
	}

	return 0;
}

int dwmac_set_est_gcrr_times(struct net_device *ndev,
			     struct est_gcrr *gcrr,
			     u32 dbgb, u32 dbgm)
{
	u64 val_ns, sys_ns;
	u32 gcl_len, tti_ns, value;
	u32 bank;
	struct timespec64 ts;
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tsn_hw_cap *cap = &priv->tsn_hwcap;
	struct est_gcrr *bgcrr;
	void __iomem *ioaddr = priv->ioaddr;
	int ret = 0;
	u32 base_sec = gcrr->base_sec;
	u32 base_nsec = gcrr->base_nsec;
	u32 cycle_sec = gcrr->cycle_sec;
	u32 cycle_nsec = gcrr->cycle_nsec;
	u32 ext_nsec = gcrr->ter_nsec;

	if (!priv->tsn_est)
		return -ENOTSUPP;

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	if (dbgm) {
		bank = dbgb;
	} else {
		value = readl(ioaddr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	if (base_nsec > 1000000000ULL || cycle_nsec > 1000000000ULL) {
		dev_warn(priv->device,
			 "EST: base(%u) or cycle(%u) nsec > 1s !\n",
			 base_nsec, cycle_nsec);

		return -EINVAL;
	}

	/* Ensure base time is later than MAC system time */
	val_ns = (u64)base_nsec;
	val_ns += (u64)(base_sec * 1000000000ULL);
	priv->ptp_clock_ops.gettime64(&priv->ptp_clock_ops, &ts);
	sys_ns = (u64)timespec64_to_ns(&ts);
	if (val_ns <= sys_ns) {
		dev_warn(priv->device,
			 "EST: base time(%llu) <= system time(%llu)\n",
			 val_ns, sys_ns);

		return -EINVAL;
	}

	if (cycle_sec > EST_CTR_HI_MAX) {
		dev_warn(priv->device,
			 "EST: cycle time(%u) > 255 seconds\n",
			 cycle_sec);

		return -EINVAL;
	}

	if (ext_nsec > cap->ext_max) {
		dev_warn(priv->device,
			 "EST: invalid time extension(%u), max=%u\n",
			 ext_nsec, cap->ext_max);

		return -EINVAL;
	}

	bgcrr = &dw_est_gc_config.gcb[bank].gcrr;
	gcl_len = bgcrr->llr;

	/* Sanity test on GCL total time intervals against cycle time.
	 * a) For GC length = 1, if its time interval is equal or greater
	 *    than cycle time, it is a constant gate error.
	 * b) If total time interval > cycle time, irregardless of GC
	 *    length, it is not considered an error that GC list is
	 *    truncated. In this case, giving a warning message is
	 *    sufficient.
	 * c) If total time interval < cycle time, irregardless of GC
	 *    length, all GATES are OPEN after the last GC is processed
	 *    until cycle time lapses. This is potentially due to poor
	 *    GCL configuration but is not an error, so we inform user
	 *    about it.
	 */
	tti_ns = est_get_gcl_total_intervals_nsec(bank, gcl_len);
	val_ns = (u64)cycle_nsec;
	val_ns += (u64)(cycle_sec * 1000000000ULL);
	if (gcl_len == 1 && tti_ns >= val_ns) {
		dev_warn(priv->device, "EST: Constant gate error!\n");

		return -EINVAL;
	}

	if (tti_ns > val_ns)
		dev_warn(priv->device, "EST: GCL is truncated!\n");

	if (tti_ns < val_ns) {
		dev_info(priv->device,
			 "EST: All GCs OPEN at %u of %llu-ns cycle\n",
			 tti_ns, val_ns);
	}

	/* Finally, start programming GCL related registers if the value
	 * differs from the driver copy for efficiency.
	 */

	if (base_nsec != bgcrr->base_nsec)
		ret |= est_write_gcl_config(ioaddr, base_nsec,
					    GCL_CTRL_ADDR_BTR_LO, 1,
					    dbgb, dbgm);

	if (base_sec != bgcrr->base_sec)
		ret |= est_write_gcl_config(ioaddr, base_sec,
					    GCL_CTRL_ADDR_BTR_HI, 1,
					    dbgb, dbgm);

	if (cycle_nsec != bgcrr->cycle_nsec)
		ret |= est_write_gcl_config(ioaddr, cycle_nsec,
					    GCL_CTRL_ADDR_CTR_LO, 1,
					    dbgb, dbgm);

	if (cycle_sec != bgcrr->cycle_sec)
		ret |= est_write_gcl_config(ioaddr, cycle_sec,
					    GCL_CTRL_ADDR_CTR_HI, 1,
					    dbgb, dbgm);

	if (ext_nsec != bgcrr->ter_nsec)
		ret |= est_write_gcl_config(ioaddr, ext_nsec,
					    GCL_CTRL_ADDR_TER, 1,
					    dbgb, dbgm);

	if (ret) {
		dev_err(priv->device, "EST: GCRR programming failure!\n");

		return ret;
	}

	/* Finally, we are ready to switch SWOL now. */
	value = readl(ioaddr + MTL_EST_CTRL);
	value |= MTL_EST_CTRL_SSWL;
	writel(value, ioaddr + MTL_EST_CTRL);

	/* Update driver copy */
	bgcrr->base_sec = base_sec;
	bgcrr->base_nsec = base_nsec;
	bgcrr->cycle_sec = cycle_sec;
	bgcrr->cycle_nsec = cycle_nsec;
	bgcrr->ter_nsec = ext_nsec;

	dev_info(priv->device, "EST: gcrr set successful\n");

	return 0;
}

int dwmac_set_est_enable(struct net_device *ndev, bool enable)
{
	u32 value;
	struct stmmac_priv *priv = netdev_priv(ndev);

	if (!priv->tsn_est)
		return -ENOTSUPP;

	if (enable && priv->flow_ctrl) {
		dev_warn(priv->device, "EST & PAUSE cannot co-exist!\n");

		return -EINVAL;
	}

	value = readl(priv->ioaddr + MTL_EST_CTRL);
	value &= ~(MTL_EST_CTRL_SSWL | MTL_EST_CTRL_EEST);
	value |= (enable & MTL_EST_CTRL_EEST);
	writel(value, priv->ioaddr + MTL_EST_CTRL);
	dw_est_gc_config.enable = enable;

	return 0;
}

int dwmac_get_est_gcc(struct net_device *ndev,
		      struct est_gc_config **gcc, bool frmdrv)
{
	int ret;
	u32 bank;
	u32 value;
	struct est_gc_config *pgcc;
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;

	if (!priv->tsn_est)
		return -ENOTSUPP;

	/* Get GC config from driver */
	if (frmdrv) {
		*gcc = &dw_est_gc_config;

		dev_info(priv->device,
			 "EST: read GCL from driver copy done.\n");

		return 0;
	}

	/* Get GC config from HW */
	pgcc = &dw_est_gc_config;

	value = readl(priv->ioaddr + MTL_EST_CTRL);
	pgcc->enable = value & MTL_EST_CTRL_EEST;

	for (bank = 0; bank < EST_GCL_BANK_MAX; bank++) {
		u32 llr, row;
		struct est_gc_bank *gcbc = &pgcc->gcb[bank];

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_BTR_LO, 1,
					  bank, 1);
		if (ret) {
			dev_err(priv->device,
				"read BTR(low) fail at bank=%u\n",
				bank);

			return ret;
		}
		gcbc->gcrr.base_nsec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_BTR_HI, 1,
					  bank, 1);
		if (ret) {
			dev_err(priv->device,
				"read BTR(high) fail at bank=%u\n",
				bank);

			return ret;
		}
		gcbc->gcrr.base_sec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_CTR_LO, 1,
					  bank, 1);
		if (ret) {
			dev_err(priv->device,
				"read CTR(low) fail at bank=%u\n",
				bank);

			return ret;
		}
		gcbc->gcrr.cycle_nsec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_CTR_HI, 1,
					  bank, 1);
		if (ret) {
			dev_err(priv->device,
				"read CTR(high) fail at bank=%u\n",
				bank);

			return ret;
		}
		gcbc->gcrr.cycle_sec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_TER, 1,
					  bank, 1);
		if (ret) {
			dev_err(priv->device,
				"read TER fail at bank=%u\n",
				bank);

			return ret;
		}
		gcbc->gcrr.ter_nsec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_LLR, 1,
					  bank, 1);
		if (ret) {
			dev_err(priv->device,
				"read LLR fail at bank=%u\n",
				bank);

			return ret;
		}
		gcbc->gcrr.llr = value;
		llr = value;

		for (row = 0; row < llr; row++) {
			u32 gates, ti_nsec;
			struct est_gc_entry *gce = gcbc->gcl + row;

			ret = est_read_gce(ndev, row, &gates, &ti_nsec,
					   bank, 1);
			if (ret) {
				dev_err(priv->device,
					"read GCE fail at bank=%u\n",
					bank);

				return ret;
			}
			gce->gates = gates;
			gce->ti_nsec = ti_nsec;
		}
	}

	*gcc = pgcc;
	dev_info(priv->device, "EST: read GCL from HW done.\n");

	return 0;
}

int dwmac_est_irq_status(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tsn_hw_cap *cap = &priv->tsn_hwcap;
	struct tsn_err_stat *err_stat = &dw_err_stat;
	void __iomem *ioaddr = priv->ioaddr;
	u32 txqcnt_mask = (1 << cap->txqcnt) - 1;
	u32 status;
	u32 value = 0;
	u32 feqn = 0;
	u32 hbfq = 0;
	u32 hbfs = 0;

	status = readl(ioaddr + MTL_EST_STATUS);

	value = (MTL_EST_STATUS_CGCE | MTL_EST_STATUS_HLBS |
		 MTL_EST_STATUS_HLBF | MTL_EST_STATUS_BTRE |
		 MTL_EST_STATUS_SWLC);

	/* Return if there is no error */
	if (!(status & value))
		return 0;

	/* spin_lock() is not needed here because of BTRE and SWLC
	 * bit will not be altered. Both of the bit will be
	 * polled in dwmac_set_est_gcrr_times()
	 */
	if (status & MTL_EST_STATUS_CGCE) {
		/* Clear Interrupt */
		writel(MTL_EST_STATUS_CGCE, ioaddr + MTL_EST_STATUS);

		err_stat->cgce_n++;
	}

	if (status & MTL_EST_STATUS_HLBS) {
		value = readl(ioaddr + MTL_EST_SCH_ERR);
		value &= txqcnt_mask;

		/* Clear Interrupt */
		writel(value, ioaddr + MTL_EST_SCH_ERR);

		/* Collecting info to shows all the queues that has HLBS */
		/* issue. The only way to clear this is to clear the     */
		/* statistic  */
		err_stat->hlbs_q |= value;
	}

	if (status & MTL_EST_STATUS_HLBF) {
		value = readl(ioaddr + MTL_EST_FRM_SZ_ERR);
		feqn = value & txqcnt_mask;

		value = readl(ioaddr + MTL_EST_FRM_SZ_CAP);
		hbfq = (value & MTL_EST_FRM_SZ_CAP_HBFQ_MASK(cap->txqcnt))
			>> MTL_EST_FRM_SZ_CAP_HBFQ_SHIFT;
		hbfs = value & MTL_EST_FRM_SZ_CAP_HBFS_MASK;

		/* Clear Interrupt */
		writel(feqn, ioaddr + MTL_EST_FRM_SZ_ERR);

		err_stat->hlbf_sz[hbfq] = hbfs;
	}

	if (status & MTL_EST_STATUS_BTRE) {
		if ((status & MTL_EST_STATUS_BTRL) ==
		    MTL_EST_STATUS_BTRL_MAX)
			err_stat->btre_max_n++;
		else
			err_stat->btre_n++;

		err_stat->btrl = (status & MTL_EST_STATUS_BTRL) >>
				   MTL_EST_STATUS_BTRL_SHIFT;

		writel(MTL_EST_STATUS_BTRE, ioaddr +
		       MTL_EST_STATUS);
	}

	if (status & MTL_EST_STATUS_SWLC) {
		writel(MTL_EST_STATUS_SWLC, ioaddr +
		       MTL_EST_STATUS);
		dev_info(priv->device, "SWOL has been switched\n");
	}

	return status;
}

int dwmac_get_est_err_stat(struct net_device *ndev,
			   struct tsn_err_stat **err_stat)
{
	struct stmmac_priv *priv = netdev_priv(ndev);

	if (!priv->tsn_est)
		return -ENOTSUPP;

	*err_stat = &dw_err_stat;

	return 0;
}

int dwmac_clr_est_err_stat(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);

	if (!priv->tsn_est)
		return -ENOTSUPP;

	memset(&dw_err_stat, 0, sizeof(dw_err_stat));

	return 0;
}

int dwmac_set_fpe_config(struct net_device *ndev, struct fpe_config *fpec)
{
	u32 txqmask, value;
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tsn_hw_cap *cap = &priv->tsn_hwcap;
	void __iomem *ioaddr = priv->ioaddr;

	if (!priv->tsn_fpe)
		return -ENOTSUPP;

	/* Check PEC is within TxQ range */
	txqmask = (1 << cap->txqcnt) - 1;
	if (fpec->txqpec & ~txqmask) {
		dev_warn(priv->device, "FPE: Tx PEC is out-of-bound.\n");

		return -EINVAL;
	}

	/* When EST and FPE are both enabled, TxQ0 is always preemptable
	 * queue. If FPE is enabled, we expect at least lsb is set.
	 * If FPE is not enabled, we also allow PEC = 0.
	 */
	if (fpec->txqpec && !(fpec->txqpec & FPE_PMAC_BIT)) {
		dev_warn(priv->device,
			 "FPE: TxQ0 must not be express queue.\n");

		return -EINVAL;
	}

	/* Field masking not needed as condition checks have been done */
	value = readl(ioaddr + MTL_FPE_CTRL_STS);
	value &= ~(txqmask << MTL_FPE_CTRL_STS_PEC_SHIFT);
	value |= (fpec->txqpec << MTL_FPE_CTRL_STS_PEC_SHIFT);
	writel(value, ioaddr + MTL_FPE_CTRL_STS);

	/* Update driver copy */
	dw_fpe_config.txqpec = fpec->txqpec;

	return 0;
}

int dwmac_set_fpe_enable(struct net_device *ndev, bool enable)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;

	if (!priv->tsn_fpe)
		return -ENOTSUPP;

	dw_fpe_config.lp_fpe_support = 0;
	dw_fpe_config.enable = enable & MAC_FPE_CTRL_STS_EFPE;

	/* Verify mPacket is only sent if FPE is enabled. */
	if (enable)
		writel((u32)(dw_fpe_config.enable | MAC_FPE_CTRL_STS_SVER),
		       ioaddr + MAC_FPE_CTRL_STS);
	else
		writel((u32)dw_fpe_config.enable,
		       ioaddr + MAC_FPE_CTRL_STS);

	return 0;
}

int dwmac_get_fpe_config(struct net_device *ndev, struct fpe_config **fpec,
			 bool frmdrv)
{
	u32 value;
	struct fpe_config *pfpec;
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;

	if (!priv->tsn_fpe)
		return -ENOTSUPP;

	/* Get FPE config from driver */
	if (frmdrv) {
		*fpec = &dw_fpe_config;

		dev_info(priv->device,
			 "FPE: read config from driver copy done.\n");

		return 0;
	}

	pfpec = &dw_fpe_config;

	value = readl(ioaddr + MTL_FPE_CTRL_STS);
	pfpec->txqpec = (value & MTL_FPE_CTRL_STS_PEC) >>
			MTL_FPE_CTRL_STS_PEC_SHIFT;

	value = readl(ioaddr + MAC_FPE_CTRL_STS);
	pfpec->enable = (bool)(value & MAC_FPE_CTRL_STS_EFPE);

	*fpec = pfpec;
	dev_info(priv->device, "FPE: read config from HW done.\n");

	return 0;
}

int dwmac_get_fpe_pmac_sts(struct net_device *ndev, u32 *hrs)
{
	u32 value;
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;

	if (!priv->tsn_fpe)
		return -ENOTSUPP;

	value = readl(ioaddr + MTL_FPE_CTRL_STS);
	*hrs = (value & MTL_FPE_CTRL_STS_HRS) >> MTL_FPE_CTRL_STS_HRS_SHIFT;

	if (hrs)
		dev_info(priv->device, "FPE: pMAC is in Hold state.\n");
	else
		dev_info(priv->device, "FPE: pMAC is in Release state.\n");

	return 0;
}

int dwmac_fpe_irq_status(struct net_device *ndev)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;
	u32 status;

	status = readl(ioaddr + MAC_FPE_CTRL_STS);

	if (status & MAC_FPE_CTRL_STS_TRSP)
		dev_info(priv->device, "Respond mPacket is transmitted\n");

	if (status & MAC_FPE_CTRL_STS_TVER)
		dev_info(priv->device, "Verify mPacket is transmitted\n");

	if (status & MAC_FPE_CTRL_STS_RRSP) {
		dw_fpe_config.lp_fpe_support = 1;
		dev_info(priv->device, "Respond mPacket is received\n");
	}

	if (status & MAC_FPE_CTRL_STS_RVER) {
		dev_info(priv->device, "Verify mPacket is received\n");

		if (ndev->features & NETIF_F_HW_FPE) {
			status &= ~MAC_FPE_CTRL_STS_SVER;
			status |= MAC_FPE_CTRL_STS_SRSP;
			writel(status, ioaddr + MAC_FPE_CTRL_STS);
		}
	}

	return 0;
}

static u64 est_get_total_gate_open_time_q(u32 bank, u32 gcl_len,
					  u64 cycle_ns, u32 q)
{
	int row;
	struct est_gc_entry *gcl = dw_est_gc_config.gcb[bank].gcl;
	u64 total = 0;
	u64 tti_ns = 0;
	u32 gate = 0x1 << q;

	/* GCL which exceeds the cycle time will be truncated.
	 * So, time interval that exceeds the cycle time will not be
	 * included.
	 */
	for (row = 0; row < gcl_len; row++) {
		tti_ns += gcl->ti_nsec;

		if (gcl->gates & gate) {
			if (tti_ns <= cycle_ns)
				total += gcl->ti_nsec;
			else
				total += gcl->ti_nsec -
					 (tti_ns - cycle_ns);
		}

		gcl++;
	}

	/* The gates wihtout any settings of open/close within
	 * the cycle time are considered as open.
	 */
	if (tti_ns < cycle_ns)
		total += cycle_ns - tti_ns;

	return total;
}

int dwmac_reconfigure_cbs(struct net_device *ndev)
{
	u32 mode_to_use;
	u32 queue;
	u64 new_idle_slope;
	struct stmmac_priv *priv = netdev_priv(ndev);
	u32 hw_bank = dwmac_get_est_bank(ndev, 1);
	u32 gcl_len = dw_est_gc_config.gcb[hw_bank].gcrr.llr;
	u64 cycle_time_ns = (dw_est_gc_config.gcb[hw_bank].gcrr.cycle_sec *
			     ONE_SEC_IN_NANOSEC) +
			     dw_est_gc_config.gcb[hw_bank].gcrr.cycle_nsec;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 open_time = 0;
	u64 scaling = 0;
	u32 max_idle_slope = MTL_TXQ_ISCQW_MAX;

	if (!cycle_time_ns) {
		dev_warn(priv->device, "EST: Cycle time is 0.\n");
		dev_warn(priv->device,
			 "CBS idle slope will not be reconfigured.\n");

		return 0;
	}

	/* queue 0 is reserved for legacy traffic */
	for (queue = 1; queue < tx_queues_count; queue++) {
		mode_to_use = priv->plat->tx_queues_cfg[queue].mode_to_use;
		if (mode_to_use == MTL_QUEUE_DCB)
			continue;

		open_time = est_get_total_gate_open_time_q(hw_bank, gcl_len,
							   cycle_time_ns,
							   queue);

		if (!open_time) {
			dev_warn(priv->device,
				 "EST: Total gate open time for queue %d is 0\n",
				 queue);
			break;
		}

		scaling = cycle_time_ns;
		do_div(scaling, open_time);

		new_idle_slope = priv->plat->tx_queues_cfg[queue].idle_slope
				 * scaling;

		if (new_idle_slope > max_idle_slope) {
			dev_warn(priv->device,
				 "EST: CBS idle slope will cap at max %x\n",
				 max_idle_slope);
			new_idle_slope = max_idle_slope;
		}

		priv->hw->mac->config_cbs(priv->hw,
				priv->plat->tx_queues_cfg[queue].send_slope,
				new_idle_slope,
				priv->plat->tx_queues_cfg[queue].high_credit,
				priv->plat->tx_queues_cfg[queue].low_credit,
				queue);
	}

	return 0;
}

int dwmac_set_tbs_launchtime_gsn(struct net_device *ndev,
				 struct dma_desc *desc,
				 struct sk_buff *skb)
{
	struct dma_enhanced_tx_desc *enhtxdesc;
	u64 temp_time;
	u32 launchtime_ns;
	u8 launchtime_s;
	u8 gsnslot;

	enhtxdesc = container_of(desc, struct dma_enhanced_tx_desc, basic);
	temp_time = ktime_to_ns(skb->transmit_time);
	launchtime_ns = do_div(temp_time, ONE_SEC_IN_NANOSEC);
	launchtime_s = temp_time;
	gsnslot = skb->transmit_gsn;

	if (dw_tsn_hwtunable.est_mode) {
		int bank = dwmac_get_est_bank(ndev, 0);
		u32 ctr_ns = dw_est_gc_config.gcb[bank].gcrr.cycle_nsec;
		u8 ctr_s = dw_est_gc_config.gcb[bank].gcrr.cycle_sec;
		u64 ctr = (ctr_s * ONE_SEC_IN_NANOSEC) + ctr_ns;
		u64 lt_offset = launchtime_ns + (launchtime_s *
				ONE_SEC_IN_NANOSEC);

		if (lt_offset > ctr)
			return -EINVAL;

		enhtxdesc->etdes4 = (launchtime_s & ETDESC4_LT_SEC) |
				    ((gsnslot << ETDESC4_GSN_SHIFT) &
				     ETDESC4_GSN);
	} else {
		enhtxdesc->etdes4 = launchtime_s & ETDESC4_LT_SEC;
	}

	enhtxdesc->etdes5 = launchtime_ns & ETDESC5_LT_NANOSEC;
	enhtxdesc->etdes4 |= ETDESC4_LTV;

	return 0;
}
