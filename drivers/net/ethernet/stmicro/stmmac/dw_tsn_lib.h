/* SPDX-License-Identifier: GPL-2.0
 *
 * dw_tsn_lib.h: DW EQoS v5.00 TSN capabilities header
 *
 * Copyright (C) 2018 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __DW_TSN_LIB_H__
#define __DW_TSN_LIB_H__

#include "linux/printk.h"

/* DWMAC v5.xx supports the following Time Sensitive Networking protocols:
 * 1) IEEE 802.1Qbv Enhancements for Scheduled Traffic (EST)
 */

/* MAC HW features3 bitmap */
#define GMAC_HW_FEAT_ESTTISW		GENMASK(24, 23)
#define GMAC_HW_FEAT_ESTTISW_SHIFT	23
#define GMAC_HW_FEAT_ESTWID		GENMASK(21, 20)
#define GMAC_HW_FEAT_ESTWID_SHIFT	20
#define GMAC_HW_FEAT_ESTDEP		GENMASK(19, 17)
#define GMAC_HW_FEAT_ESTDEP_SHIFT	17
#define GMAC_HW_FEAT_ESTSEL		BIT(16)

/* MTL EST control register */
#define MTL_EST_CTRL			0x00000c50
#define MTL_EST_CTRL_PTOV		GENMASK(31, 24)
#define MTL_EST_CTRL_PTOV_SHIFT		24
#define MTL_EST_CTRL_CTOV		GENMASK(23, 12)
#define MTL_EST_CTRL_CTOV_SHIFT		12
#define MTL_EST_CTRL_TILS		GENMASK(10, 8)
#define MTL_EST_CTRL_TILS_SHIFT		8
#define MTL_EST_CTRL_SSWL		BIT(1)	/* Switch to SWOL */
#define MTL_EST_CTRL_EEST		BIT(0)	/* Enable EST */

/* MTL EST status register */
#define MTL_EST_STATUS			0x00000c58
#define MTL_EST_STATUS_BTRL		GENMASK(11, 8)	/* BTR ERR loop cnt */
#define MTL_EST_STATUS_BTRL_SHIFT	8
#define MTL_EST_STATUS_BTRL_MAX		(0xF << 8)
#define MTL_EST_STATUS_SWOL		BIT(7)	/* SW owned list */
#define MTL_EST_STATUS_SWOL_SHIFT	7
#define MTL_EST_STATUS_BTRE		BIT(1)	/* BTR Error */
#define MTL_EST_STATUS_SWLC		BIT(0)	/* Switch to SWOL complete */

/* MTL EST GCL control register */
#define MTL_EST_GCL_CTRL		0x00000c80
#define MTL_EST_GCL_CTRL_ADDR		GENMASK(10, 8)	/* GCL Address */
#define MTL_EST_GCL_CTRL_ADDR_VAL(addr)	(addr << 8)
#define GCL_CTRL_ADDR_BTR_LO		0x0
#define GCL_CTRL_ADDR_BTR_HI		0x1
#define GCL_CTRL_ADDR_CTR_LO		0x2
#define GCL_CTRL_ADDR_CTR_HI		0x3
#define GCL_CTRL_ADDR_TER		0x4
#define GCL_CTRL_ADDR_LLR		0x5
#define MTL_EST_GCL_CTRL_DBGB1		BIT(5)	/* Debug Mode Bank Select */
#define MTL_EST_GCL_CTRL_DBGM		BIT(4)	/* Debug Mode */
#define MTL_EST_GCL_CTRL_GCRR		BIT(2)	/* GC Related Registers */
#define MTL_EST_GCL_CTRL_R1W0		BIT(1)	/* Read / Write Operation */
#define GCL_OPS_R			BIT(1)
#define GCL_OPS_W			0
#define MTL_EST_GCL_CTRL_SRWO		BIT(0)	/* Start R/W Operation */

/* MTL EST GCL data register */
#define MTL_EST_GCL_DATA		0x00000c84

/* EST Global defines */
#define EST_CTR_HI_MAX			0xff	/* CTR Hi is 8-bit only */
#define EST_PTOV_MAX			0xff	/* Max PTP time offset */
#define EST_CTOV_MAX			0xfff	/* Max Current time offset */
#define EST_TIWID_TO_EXTMAX(ti_wid)	((1 << (ti_wid + 7)) - 1)
#define EST_GCL_BANK_MAX	(2)

/* MAC Core Version */
#define TSN_VER_MASK		0xFF
#define TSN_CORE_VER		0x50

/* MAC PTP clock registers */
#define TSN_PTP_STSR		0x08
#define TSN_PTP_STNSR		0x0c

/* Hardware Tunable Enum */
enum tsn_hwtunable_id {
	TSN_HWTUNA_TX_EST_TILS = 0,
	TSN_HWTUNA_TX_EST_PTOV,
	TSN_HWTUNA_TX_EST_CTOV,
	TSN_HWTUNA_MAX,
};

/* TSN Feature Enabled List */
enum tsn_feat_id {
	TSN_FEAT_ID_EST = 0,
	TSN_FEAT_ID_MAX,
};

/* HW register read & write macros */
#define TSN_RD32(__addr)		readl(__addr)
#define TSN_WR32(__val, __addr)		writel(__val, __addr)

/* Logging macros with no args */
#define DRVNAME "stmmac"
#define TSN_INFO_NA(__msg)	printk(KERN_INFO DRVNAME ":" __msg)
#define TSN_WARN_NA(__msg)	printk(KERN_WARNING DRVNAME ":" __msg)
#define TSN_ERR_NA(__msg)	printk(KERN_ERR DRVNAME ":" __msg)

/* Logging macros with args */
#define TSN_INFO(__msg, __arg0, __args...) \
	printk(KERN_INFO DRVNAME ":" __msg, (__arg0), ##__args)
#define TSN_WARN(__msg, __arg0, __args...) \
	printk(KERN_WARNING DRVNAME ":" __msg, (__arg0), ##__args)
#define TSN_ERR(__msg, __arg0, __args...) \
	printk(KERN_ERR DRVNAME ":" __msg, (__arg0), ##__args)

/* TSN HW Capabilities */
struct tsn_hw_cap {
	bool est_support;		/* 1: supported */
	unsigned int txqcnt;		/* Number of TxQ (control gate) */
	unsigned int gcl_depth;		/* GCL depth. */
	unsigned int ti_wid;		/* time interval width */
	unsigned int tils_max;		/* Max time interval left shift */
	unsigned int ext_max;		/* Max time extension */
};

/* EST Gate Control Entry */
struct est_gc_entry {
	unsigned int gates;		/* gate control: 0: closed,
					 *               1: open.
					 */
	unsigned int ti_nsec;		/* time interval in nsec */
};

/* EST GCL Related Registers */
struct est_gcrr {
	unsigned int base_nsec;		/* base time denominator (nsec) */
	unsigned int base_sec;		/* base time numerator (sec) */
	unsigned int cycle_nsec;	/* cycle time denominator (nsec) */
	unsigned int cycle_sec;		/* cycle time numerator sec)*/
	unsigned int ter_nsec;		/* time extension (nsec) */
	unsigned int llr;		/* GC list length */
};

/* EST Gate Control bank */
struct est_gc_bank {
	struct est_gc_entry *gcl;	/* Gate Control List */
	struct est_gcrr gcrr;		/* GCL Related Registers */
};

/* EST Gate Control Configuration */
struct est_gc_config {
	struct est_gc_bank gcb[EST_GCL_BANK_MAX];
	bool enable;			/* 1: enabled */
};

/* TSN functions */
void dwmac_tsn_init(void *ioaddr);
void dwmac_get_tsn_hwcap(struct tsn_hw_cap **tsn_hwcap);
void dwmac_set_est_gcb(struct est_gc_entry *gcl, unsigned int bank);
void dwmac_set_tsn_feat(enum tsn_feat_id featid, bool enable);
int dwmac_set_tsn_hwtunable(void *ioaddr, enum tsn_hwtunable_id id,
			    const unsigned int *data);
int dwmac_get_tsn_hwtunable(enum tsn_hwtunable_id id, unsigned int *data);
int dwmac_get_est_bank(void *ioaddr, unsigned int own);
int dwmac_set_est_gce(void *ioaddr,
		      struct est_gc_entry *gce, unsigned int row,
		      unsigned int dbgb, unsigned int dbgm);
int dwmac_get_est_gcrr_llr(void *ioaddr, unsigned int *gcl_len,
			   unsigned int dbgb, unsigned int dbgm);
int dwmac_set_est_gcrr_llr(void *ioaddr, unsigned int gcl_len,
			   unsigned int dbgb, unsigned int dbgm);
int dwmac_set_est_gcrr_times(void *ioaddr,
			     struct est_gcrr *gcrr,
			     unsigned int dbgb, unsigned int dbgm);
int dwmac_set_est_enable(void *ioaddr, bool enable);
int dwmac_get_est_gcc(void *ioaddr,
		      struct est_gc_config **gcc, bool frmdrv);
#endif /* __DW_TSN_LIB_H__ */
