/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
 *
 *
 * dw_tsn_lib.c: DW EQoS v5.00 TSN capabilities
 *
 * Copyright (C) 2018 Intel Corporation
 *
 * This software is licensed under
 * (a) a 3-clause BSD license; or alternatively
 * (b) the GPL v2 license
 *
 * -- A. BSD-3-Clause ----------------------------
 * Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -- B. GPL-2.0 ----------------------------
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.# See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 * ------------------------------
 *
 * SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 */

#include "dwmac4.h"
#include "dwmac5.h"
#include "dw_tsn_lib.h"

static struct tsn_hw_cap dw_tsn_hwcap;
static bool dw_tsn_feat_en[TSN_FEAT_ID_MAX];
static unsigned int dw_tsn_hwtunable[TSN_HWTUNA_MAX];
static struct est_gc_config dw_est_gc_config;
static struct tsn_err_stat dw_err_stat;
static struct fpe_config dw_fpe_config;

#define ONE_SEC_IN_NANOSEC 1000000000ULL

static unsigned int est_get_gcl_depth(unsigned int hw_cap)
{
	unsigned int depth;
	unsigned int estdep = (hw_cap & GMAC_HW_FEAT_ESTDEP)
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

static unsigned int est_get_ti_width(unsigned int hw_cap)
{
	unsigned int width;
	unsigned int estwid = (hw_cap & GMAC_HW_FEAT_ESTWID)
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

static int est_poll_srwo(void *ioaddr)
{
	/* Poll until the EST GCL Control[SRWO] bit clears.
	 * Total wait = 12 x 50ms ~= 0.6s.
	 */
	unsigned int retries = 12;
	unsigned int value;

	do {
		value = TSN_RD32(ioaddr + MTL_EST_GCL_CTRL);
		if (!(value & MTL_EST_GCL_CTRL_SRWO))
			return 0;
		msleep(50);
	} while (--retries);

	return -ETIMEDOUT;
}

static int est_set_gcl_addr(void *ioaddr, unsigned int addr,
			    unsigned int gcrr, unsigned int rwops,
			    unsigned int dbgb, unsigned int dbgm)
{
	unsigned int value;

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

	TSN_WR32(value, ioaddr + MTL_EST_GCL_CTRL);

	return est_poll_srwo(ioaddr);
}

static int est_write_gcl_config(void *ioaddr, unsigned int data,
				unsigned int addr, unsigned int gcrr,
				unsigned int dbgb, unsigned int dbgm)
{
	TSN_WR32(data, ioaddr + MTL_EST_GCL_DATA);

	return est_set_gcl_addr(ioaddr, addr, gcrr, GCL_OPS_W, dbgb, dbgm);
}

static int est_read_gcl_config(void *ioaddr, unsigned int *data,
			       unsigned int addr, unsigned int gcrr,
			       unsigned int dbgb, unsigned int dbgm)
{
	int ret;

	ret = est_set_gcl_addr(ioaddr, addr, gcrr, GCL_OPS_R, dbgb, dbgm);
	if (ret)
		return ret;

	*data = TSN_RD32(ioaddr + MTL_EST_GCL_DATA);

	return ret;
}

static int est_read_gce(void *ioaddr, unsigned int row,
			unsigned int *gates, unsigned int *ti_nsec,
			unsigned int dbgb, unsigned int dbgm)
{
	int ret;
	unsigned int value;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	unsigned int ti_wid = cap->ti_wid;
	unsigned int gates_mask = (1 << cap->txqcnt) - 1;
	unsigned int ti_mask = (1 << ti_wid) - 1;

	ret = est_read_gcl_config(ioaddr, &value, row, 0, dbgb, dbgm);
	if (ret) {
		TSN_ERR("Read GCE failed! row=%u\n", row);

		return ret;
	}
	*ti_nsec = value & ti_mask;
	*gates = (value >> ti_wid) & gates_mask;

	return ret;
}

static unsigned int est_get_gcl_total_intervals_nsec(unsigned int bank,
						     unsigned int gcl_len)
{
	unsigned int row;
	unsigned int nsec = 0;
	struct est_gc_entry *gcl = dw_est_gc_config.gcb[bank].gcl;

	for (row = 0; row < gcl_len; row++) {
		nsec += gcl->ti_nsec;
		gcl++;
	}

	return nsec;
}

static int est_set_tils(void *ioaddr, const unsigned int tils)
{
	unsigned int value;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	if (tils > cap->tils_max) {
		TSN_WARN("EST: invalid tils(%u), max=%u\n",
			 tils, cap->tils_max);

		return -EINVAL;
	}

	/* Ensure that HW is not in the midst of GCL transition */
	value = TSN_RD32(ioaddr + MTL_EST_CTRL);
	value &= ~MTL_EST_CTRL_SSWL;

	/* MTL_EST_CTRL value has been read earlier, if TILS value
	 * differs, we update here.
	 */
	if (tils != dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_TILS]) {
		value &= ~MTL_EST_CTRL_TILS;
		value |= (tils << MTL_EST_CTRL_TILS_SHIFT);

		TSN_WR32(value, ioaddr + MTL_EST_CTRL);
		dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_TILS] = tils;
	}

	return 0;
}

static int est_set_ov(void *ioaddr,
		      const unsigned int *ptov,
		      const unsigned int *ctov)
{
	unsigned int value;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	value = TSN_RD32(ioaddr + MTL_EST_CTRL);
	value &= ~MTL_EST_CTRL_SSWL;

	if (ptov) {
		if (*ptov > EST_PTOV_MAX) {
			TSN_WARN("EST: invalid PTOV(%u), max=%u\n",
				 *ptov, EST_PTOV_MAX);

			return -EINVAL;
		} else if (*ptov !=
			   dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_PTOV]) {
			value &= ~MTL_EST_CTRL_PTOV;
			value |= (*ptov << MTL_EST_CTRL_PTOV_SHIFT);
			dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_PTOV] = *ptov;
		}
	}

	if (ctov) {
		if (*ctov > EST_CTOV_MAX) {
			TSN_WARN("EST: invalid CTOV(%u), max=%u\n",
				 *ctov, EST_CTOV_MAX);

			return -EINVAL;
		} else if (*ctov != dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_CTOV]) {
			value &= ~MTL_EST_CTRL_CTOV;
			value |= (*ctov << MTL_EST_CTRL_CTOV_SHIFT);
			dw_tsn_hwtunable[TSN_HWTUNA_TX_EST_CTOV] = *ctov;
		}
	}

	TSN_WR32(value, ioaddr + MTL_EST_CTRL);

	return 0;
}

static int fpe_set_afsz(void *ioaddr, const unsigned int afsz)
{
	unsigned int value;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE])
		return -ENOTSUPP;

	if (afsz > FPE_AFSZ_MAX) {
		TSN_WARN_NA("FPE: AFSZ is out-of-bound.\n");

		return -EINVAL;
	}

	if (afsz != dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_AFSZ]) {
		value = TSN_RD32(ioaddr + MTL_FPE_CTRL_STS);
		value &= ~MTL_FPE_CTRL_STS_AFSZ;
		value |= afsz;
		TSN_WR32(value, ioaddr + MTL_FPE_CTRL_STS);
		dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_AFSZ] = afsz;
	}

	return 0;
}

static int fpe_set_hr_adv(void *ioaddr,
			  const unsigned int *hadv,
			  const unsigned int *radv)
{
	unsigned int value;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE])
		return -ENOTSUPP;

	value = TSN_RD32(ioaddr + MTL_FPE_ADVANCE);

	if (hadv) {
		if (*hadv > FPE_ADV_MAX) {
			TSN_WARN("FPE: invalid HADV(%u), max=%u\n",
				 *hadv, FPE_ADV_MAX);

			return -EINVAL;
		} else if (*hadv !=
			   dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_HADV]) {
			value &= ~MTL_FPE_ADVANCE_HADV;
			value |= (*hadv & MTL_FPE_ADVANCE_HADV);
			dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_HADV] = *hadv;
		}
	}

	if (radv) {
		if (*radv > FPE_ADV_MAX) {
			TSN_WARN("FPE: invalid RADV(%u), max=%u\n",
				 *radv, FPE_ADV_MAX);

			return -EINVAL;
		} else if (*radv !=
			   dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_RADV]) {
			value &= ~MTL_FPE_ADVANCE_RADV;
			value |= ((*radv << MTL_FPE_ADVANCE_RADV_SHIFT) &
				  MTL_FPE_ADVANCE_RADV);
			dw_tsn_hwtunable[TSN_HWTUNA_TX_FPE_RADV] = *radv;
		}
	}

	TSN_WR32(value, ioaddr + MTL_FPE_ADVANCE);

	return 0;
}

static unsigned long long est_get_all_open_time(unsigned int bank,
						unsigned long long cycle_ns,
						unsigned int queue)
{
	int row;
	unsigned int gcl_len = dw_est_gc_config.gcb[bank].gcrr.llr;
	struct est_gc_entry *gcl = dw_est_gc_config.gcb[bank].gcl;
	unsigned long long total = 0;
	unsigned long long tti_ns = 0;
	unsigned int gate = 0x1 << queue;

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

	/* The gates wihtout any setting of open/close within
	 * the cycle time are considered as open.
	 */
	if (tti_ns < cycle_ns)
		total += cycle_ns - tti_ns;

	return total;
}

void dwmac_tsn_init(void *ioaddr)
{
	unsigned int gcl_depth;
	unsigned int tils_max;
	unsigned int ti_wid;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	unsigned int hwid = TSN_RD32(ioaddr + GMAC4_VERSION) & TSN_VER_MASK;
	unsigned int hw_cap2 = TSN_RD32(ioaddr + GMAC_HW_FEATURE2);
	unsigned int hw_cap3 = TSN_RD32(ioaddr + GMAC_HW_FEATURE3);

	memset(cap, 0, sizeof(*cap));

	if (hwid < TSN_CORE_VER) {
		TSN_WARN_NA("IP v5.00 does not support TSN\n");
		return;
	}

	if (!(hw_cap3 & GMAC_HW_FEAT_ESTSEL)) {
		TSN_WARN_NA("EST NOT supported\n");
		cap->est_support = 0;

		goto check_fpe;
	}

	gcl_depth = est_get_gcl_depth(hw_cap3);
	ti_wid = est_get_ti_width(hw_cap3);

	cap->ti_wid = ti_wid;
	cap->gcl_depth = gcl_depth;

	tils_max = (hw_cap3 & GMAC_HW_FEAT_ESTTISW)
		   >> GMAC_HW_FEAT_ESTTISW_SHIFT;
	tils_max = (1 << tils_max) - 1;
	cap->tils_max = tils_max;

	cap->ext_max = EST_TIWID_TO_EXTMAX(ti_wid);
	cap->txqcnt = ((hw_cap2 & GMAC_HW_FEAT_TXQCNT) >> 6) + 1;
	cap->est_support = 1;

	TSN_INFO("EST: depth=%u, ti_wid=%u, tils_max=%u tqcnt=%u\n",
		 gcl_depth, ti_wid, tils_max, cap->txqcnt);

check_fpe:
	if (!(hw_cap3 & GMAC_HW_FEAT_FPESEL)) {
		TSN_INFO_NA("FPE NOT supported\n");
		cap->fpe_support = 0;
	} else {
		TSN_INFO_NA("FPE capable\n");
		cap->rxqcnt = (hw_cap2 & GMAC_HW_FEAT_RXQCNT) + 1;
		cap->fpe_support = 1;
	}
}

/* dwmac_tsn_setup is called within stmmac_hw_setup() after
 * stmmac_init_dma_engine() which resets MAC controller.
 * This is so-that MAC registers are not cleared.
 */
void dwmac_tsn_setup(void *ioaddr, unsigned int fprq)
{
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	unsigned int value;

	if (cap->est_support) {
		/* Enable EST interrupts */
		value = (MTL_EST_INT_EN_CGCE | MTL_EST_INT_EN_IEHS |
			 MTL_EST_INT_EN_IEHF | MTL_EST_INT_EN_IEBE |
			 MTL_EST_INT_EN_IECC);
		TSN_WR32(value, ioaddr + MTL_EST_INT_EN);
	}
	if (cap->fpe_support && fprq <= cap->rxqcnt) {
		/* Update FPRQ */
		value = TSN_RD32(ioaddr + GMAC_RXQ_CTRL1);
		value &= ~GMAC_RXQCTRL_FPRQ_MASK;
		value |= fprq << GMAC_RXQCTRL_FPRQ_SHIFT;
		TSN_WR32(value, ioaddr + GMAC_RXQ_CTRL1);
	} else {
		TSN_WARN_NA("FPE: FPRQ is out-of-bound.\n");
	}
}

void dwmac_get_tsn_hwcap(struct tsn_hw_cap **tsn_hwcap)
{
	*tsn_hwcap = &dw_tsn_hwcap;
}

void dwmac_set_est_gcb(struct est_gc_entry *gcl, unsigned int bank)
{
	if (bank >= 0 && bank < EST_GCL_BANK_MAX)
		dw_est_gc_config.gcb[bank].gcl = gcl;
}

void dwmac_set_tsn_feat(enum tsn_feat_id featid, bool enable)
{
	if (featid < TSN_FEAT_ID_MAX)
		dw_tsn_feat_en[featid] = enable;
}

int dwmac_set_tsn_hwtunable(void *ioaddr,
			    enum tsn_hwtunable_id id,
			    const unsigned int *data)
{
	int ret = 0;

	switch (id) {
	case TSN_HWTUNA_TX_EST_TILS:
		ret = est_set_tils(ioaddr, *data);
		break;
	case TSN_HWTUNA_TX_EST_PTOV:
		ret = est_set_ov(ioaddr, data, NULL);
		break;
	case TSN_HWTUNA_TX_EST_CTOV:
		ret = est_set_ov(ioaddr, NULL, data);
		break;
	case TSN_HWTUNA_TX_FPE_AFSZ:
		ret = fpe_set_afsz(ioaddr, *data);
		break;
	case TSN_HWTUNA_TX_FPE_HADV:
		ret = fpe_set_hr_adv(ioaddr, data, NULL);
		break;
	case TSN_HWTUNA_TX_FPE_RADV:
		ret = fpe_set_hr_adv(ioaddr, NULL, data);
		break;
	default:
		ret = -EINVAL;
	};

	return ret;
}

int dwmac_get_tsn_hwtunable(enum tsn_hwtunable_id id, unsigned int *data)
{
	if (id >= TSN_HWTUNA_MAX)
		return -EINVAL;

	*data = dw_tsn_hwtunable[id];

	return 0;
}

int dwmac_get_est_bank(void *ioaddr, unsigned int own)
{
	int swol;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	swol = TSN_RD32(ioaddr + MTL_EST_STATUS);

	swol = ((swol & MTL_EST_STATUS_SWOL) >>
		MTL_EST_STATUS_SWOL_SHIFT);

	if (own)
		return swol;
	else
		return (~swol & 0x1);
}

int dwmac_set_est_gce(void *ioaddr,
		      struct est_gc_entry *gce, unsigned int row,
		      unsigned int dbgb, unsigned int dbgm)
{
	unsigned int value;
	unsigned int ti_wid;
	unsigned int ti_max;
	unsigned int gates_mask;
	unsigned int bank;
	struct est_gc_entry *gcl;
	int ret;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	unsigned int gates = gce->gates;
	unsigned int ti_nsec = gce->ti_nsec;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	if (dbgm) {
		bank = dbgb;
	} else {
		value = TSN_RD32(ioaddr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	if (!cap->gcl_depth || row > cap->gcl_depth) {
		TSN_WARN("EST: row(%u) > GCL depth(%u)\n",
			 row, cap->gcl_depth);

		return -EINVAL;
	}

	ti_wid = cap->ti_wid;
	ti_max = (1 << ti_wid) - 1;
	if (ti_nsec > ti_max) {
		TSN_WARN("EST: ti_nsec(%u) > upper limit(%u)\n",
			 ti_nsec, ti_max);

		return -EINVAL;
	}

	gates_mask = (1 << cap->txqcnt) - 1;
	value = ((gates & gates_mask) << ti_wid) | ti_nsec;

	ret = est_write_gcl_config(ioaddr, value, row, 0, dbgb, dbgm);
	if (ret) {
		TSN_ERR("EST: GCE write failed: bank=%u row=%u.\n",
			bank, row);

		return ret;
	}

	TSN_INFO("EST: GCE write: dbgm=%u bank=%u row=%u, gc=0x%x.\n",
		 dbgm, bank, row, value);

	/* Since GC write is successful, update GCL copy of the driver */
	gcl = dw_est_gc_config.gcb[bank].gcl + row;
	gcl->gates = gates;
	gcl->ti_nsec = ti_nsec;

	return ret;
}

int dwmac_get_est_gcrr_llr(void *ioaddr, unsigned int *gcl_len,
			   unsigned int dbgb, unsigned int dbgm)
{
	unsigned int bank, value;
	int ret;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	if (dbgm) {
		bank = dbgb;
	} else {
		value = TSN_RD32(ioaddr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	ret = est_read_gcl_config(ioaddr, &value,
				  GCL_CTRL_ADDR_LLR, 1,
				  dbgb, dbgm);
	if (ret) {
		TSN_ERR("read LLR fail at bank=%u\n", bank);

			return ret;
	}

	*gcl_len = value;

	return 0;
}

int dwmac_set_est_gcrr_llr(void *ioaddr, unsigned int gcl_len,
			   unsigned int dbgb, unsigned int dbgm)
{
	unsigned int bank, value;
	struct est_gcrr *bgcrr;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	int ret = 0;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	if (dbgm) {
		bank = dbgb;
	} else {
		value = TSN_RD32(ioaddr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	if (gcl_len > cap->gcl_depth) {
		TSN_WARN("EST: GCL length(%u) > depth(%u)\n",
			 gcl_len, cap->gcl_depth);

		return -EINVAL;
	}

	bgcrr = &dw_est_gc_config.gcb[bank].gcrr;

	if (gcl_len != bgcrr->llr) {
		ret = est_write_gcl_config(ioaddr, gcl_len,
					   GCL_CTRL_ADDR_LLR, 1,
					   dbgb, dbgm);
		if (ret) {
			TSN_ERR_NA("EST: GCRR programming failure!\n");

			return ret;
		}
		bgcrr->llr = gcl_len;
	}

	return 0;
}

int dwmac_set_est_gcrr_times(void *ioaddr,
			     struct est_gcrr *gcrr,
			     unsigned int dbgb, unsigned int dbgm)
{
	u64 val_ns, sys_ns;
	unsigned int gcl_len, tti_ns, value;
	unsigned int bank;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	struct est_gcrr *bgcrr;
	int ret = 0;
	unsigned int base_sec = gcrr->base_sec;
	unsigned int base_nsec = gcrr->base_nsec;
	unsigned int cycle_sec = gcrr->cycle_sec;
	unsigned int cycle_nsec = gcrr->cycle_nsec;
	unsigned int ext_nsec = gcrr->ter_nsec;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	if (dbgb >= EST_GCL_BANK_MAX)
		return -EINVAL;

	if (dbgm) {
		bank = dbgb;
	} else {
		value = TSN_RD32(ioaddr + MTL_EST_STATUS);
		bank = (value & MTL_EST_STATUS_SWOL) >>
		       MTL_EST_STATUS_SWOL_SHIFT;
	}

	if (base_nsec > 1000000000ULL || cycle_nsec > 1000000000ULL) {
		TSN_WARN("EST: base(%u) or cycle(%u) nsec > 1s !\n",
			 base_nsec, cycle_nsec);

		return -EINVAL;
	}

	/* Ensure base time is later than MAC system time */
	val_ns = (u64)base_nsec;
	val_ns += (u64)(base_sec * 1000000000ULL);

	/* Get the MAC system time */
	sys_ns = TSN_RD32(ioaddr + TSN_PTP_STNSR);
	sys_ns += TSN_RD32(ioaddr + TSN_PTP_STSR) * 1000000000ULL;

	if (val_ns <= sys_ns) {
		TSN_WARN("EST: base time(%llu) <= system time(%llu)\n",
			 val_ns, sys_ns);

		return -EINVAL;
	}

	if (cycle_sec > EST_CTR_HI_MAX) {
		TSN_WARN("EST: cycle time(%u) > 255 seconds\n", cycle_sec);

		return -EINVAL;
	}

	if (ext_nsec > cap->ext_max) {
		TSN_WARN("EST: invalid time extension(%u), max=%u\n",
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
		TSN_WARN_NA("EST: Constant gate error!\n");

		return -EINVAL;
	}

	if (tti_ns > val_ns)
		TSN_WARN_NA("EST: GCL is truncated!\n");

	if (tti_ns < val_ns) {
		TSN_INFO("EST: All GCs OPEN at %u of %llu-ns cycle\n",
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
		TSN_ERR_NA("EST: GCRR programming failure!\n");

		return ret;
	}

	/* Finally, we are ready to switch SWOL now. */
	value = TSN_RD32(ioaddr + MTL_EST_CTRL);
	value |= MTL_EST_CTRL_SSWL;
	TSN_WR32(value, ioaddr + MTL_EST_CTRL);

	/* Update driver copy */
	bgcrr->base_sec = base_sec;
	bgcrr->base_nsec = base_nsec;
	bgcrr->cycle_sec = cycle_sec;
	bgcrr->cycle_nsec = cycle_nsec;
	bgcrr->ter_nsec = ext_nsec;

	TSN_INFO_NA("EST: gcrr set successful\n");

	return 0;
}

int dwmac_set_est_enable(void *ioaddr, bool enable)
{
	unsigned int value;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	value = TSN_RD32(ioaddr + MTL_EST_CTRL);
	value &= ~(MTL_EST_CTRL_SSWL | MTL_EST_CTRL_EEST);
	value |= (enable & MTL_EST_CTRL_EEST);
	TSN_WR32(value, ioaddr + MTL_EST_CTRL);
	dw_est_gc_config.enable = enable;

	return 0;
}

int dwmac_get_est_gcc(void *ioaddr,
		      struct est_gc_config **gcc, bool frmdrv)
{
	int ret;
	unsigned int bank;
	unsigned int value;
	struct est_gc_config *pgcc;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	/* Get GC config from driver */
	if (frmdrv) {
		*gcc = &dw_est_gc_config;

		TSN_INFO_NA("EST: read GCL from driver copy done.\n");

		return 0;
	}

	/* Get GC config from HW */
	pgcc = &dw_est_gc_config;

	value = TSN_RD32(ioaddr + MTL_EST_CTRL);
	pgcc->enable = value & MTL_EST_CTRL_EEST;

	for (bank = 0; bank < EST_GCL_BANK_MAX; bank++) {
		unsigned int llr, row;
		struct est_gc_bank *gcbc = &pgcc->gcb[bank];

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_BTR_LO, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read BTR(low) fail at bank=%u\n", bank);

			return ret;
		}
		gcbc->gcrr.base_nsec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_BTR_HI, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read BTR(high) fail at bank=%u\n", bank);

			return ret;
		}
		gcbc->gcrr.base_sec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_CTR_LO, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read CTR(low) fail at bank=%u\n", bank);

			return ret;
		}
		gcbc->gcrr.cycle_nsec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_CTR_HI, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read CTR(high) fail at bank=%u\n", bank);

			return ret;
		}
		gcbc->gcrr.cycle_sec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_TER, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read TER fail at bank=%u\n", bank);

			return ret;
		}
		gcbc->gcrr.ter_nsec = value;

		ret = est_read_gcl_config(ioaddr, &value,
					  GCL_CTRL_ADDR_LLR, 1,
					  bank, 1);
		if (ret) {
			TSN_ERR("read LLR fail at bank=%u\n", bank);

			return ret;
		}
		gcbc->gcrr.llr = value;
		llr = value;

		for (row = 0; row < llr; row++) {
			unsigned int gates, ti_nsec;
			struct est_gc_entry *gce = gcbc->gcl + row;

			ret = est_read_gce(ioaddr, row, &gates, &ti_nsec,
					   bank, 1);
			if (ret) {
				TSN_ERR("read GCE fail at bank=%u\n", bank);

				return ret;
			}
			gce->gates = gates;
			gce->ti_nsec = ti_nsec;
		}
	}

	*gcc = pgcc;
	TSN_INFO_NA("EST: read GCL from HW done.\n");

	return 0;
}

int dwmac_est_irq_status(void *ioaddr)
{
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;
	struct tsn_err_stat *err_stat = &dw_err_stat;
	unsigned int txqcnt_mask = (1 << cap->txqcnt) - 1;
	unsigned int status;
	unsigned int value = 0;
	unsigned int feqn = 0;
	unsigned int hbfq = 0;
	unsigned int hbfs = 0;

	status = TSN_RD32(ioaddr + MTL_EST_STATUS);

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
		TSN_WR32(MTL_EST_STATUS_CGCE, ioaddr + MTL_EST_STATUS);

		err_stat->cgce_n++;
	}

	if (status & MTL_EST_STATUS_HLBS) {
		value = TSN_RD32(ioaddr + MTL_EST_SCH_ERR);
		value &= txqcnt_mask;

		/* Clear Interrupt */
		TSN_WR32(value, ioaddr + MTL_EST_SCH_ERR);

		/* Collecting info to shows all the queues that has HLBS */
		/* issue. The only way to clear this is to clear the     */
		/* statistic  */
		err_stat->hlbs_q |= value;
	}

	if (status & MTL_EST_STATUS_HLBF) {
		value = TSN_RD32(ioaddr + MTL_EST_FRM_SZ_ERR);
		feqn = value & txqcnt_mask;

		value = TSN_RD32(ioaddr + MTL_EST_FRM_SZ_CAP);
		hbfq = (value & MTL_EST_FRM_SZ_CAP_HBFQ_MASK(cap->txqcnt))
			>> MTL_EST_FRM_SZ_CAP_HBFQ_SHIFT;
		hbfs = value & MTL_EST_FRM_SZ_CAP_HBFS_MASK;

		/* Clear Interrupt */
		TSN_WR32(feqn, ioaddr + MTL_EST_FRM_SZ_ERR);

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

		TSN_WR32(MTL_EST_STATUS_BTRE, ioaddr +
		       MTL_EST_STATUS);
	}

	if (status & MTL_EST_STATUS_SWLC) {
		TSN_WR32(MTL_EST_STATUS_SWLC, ioaddr +
			 MTL_EST_STATUS);
		TSN_INFO_NA("SWOL has been switched\n");
	}

	return status;
}

int dwmac_get_est_err_stat(struct tsn_err_stat **err_stat)
{
	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	*err_stat = &dw_err_stat;

	return 0;
}

int dwmac_clr_est_err_stat(void *ioaddr)
{
	if (!dw_tsn_feat_en[TSN_FEAT_ID_EST])
		return -ENOTSUPP;

	memset(&dw_err_stat, 0, sizeof(dw_err_stat));

	return 0;
}

int dwmac_set_fpe_config(void *ioaddr, struct fpe_config *fpec)
{
	unsigned int txqmask, value;
	struct tsn_hw_cap *cap = &dw_tsn_hwcap;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE])
		return -ENOTSUPP;

	/* Check PEC is within TxQ range */
	txqmask = (1 << cap->txqcnt) - 1;
	if (fpec->txqpec & ~txqmask) {
		TSN_WARN_NA("FPE: Tx PEC is out-of-bound.\n");

		return -EINVAL;
	}

	/* When EST and FPE are both enabled, TxQ0 is always preemptable
	 * queue. If FPE is enabled, we expect at least lsb is set.
	 * If FPE is not enabled, we also allow PEC = 0.
	 */
	if (fpec->txqpec && !(fpec->txqpec & FPE_PMAC_BIT)) {
		TSN_WARN_NA("FPE: TxQ0 must not be express queue.\n");

		return -EINVAL;
	}

	/* Field masking not needed as condition checks have been done */
	value = TSN_RD32(ioaddr + MTL_FPE_CTRL_STS);
	value &= ~(txqmask << MTL_FPE_CTRL_STS_PEC_SHIFT);
	value |= (fpec->txqpec << MTL_FPE_CTRL_STS_PEC_SHIFT);
	TSN_WR32(value, ioaddr + MTL_FPE_CTRL_STS);

	/* Update driver copy */
	dw_fpe_config.txqpec = fpec->txqpec;

	return 0;
}

int dwmac_set_fpe_enable(void *ioaddr, bool enable)
{
	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE])
		return -ENOTSUPP;

	dw_fpe_config.enable = enable & MAC_FPE_CTRL_STS_EFPE;

	TSN_WR32((unsigned int)dw_fpe_config.enable,
		 ioaddr + MAC_FPE_CTRL_STS);

	return 0;
}

int dwmac_get_fpe_config(void *ioaddr, struct fpe_config **fpec,
			 bool frmdrv)
{
	unsigned int value;
	struct fpe_config *pfpec;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE])
		return -ENOTSUPP;

	/* Get FPE config from driver */
	if (frmdrv) {
		*fpec = &dw_fpe_config;

		TSN_INFO_NA("FPE: read config from driver copy done.\n");

		return 0;
	}

	pfpec = &dw_fpe_config;

	value = TSN_RD32(ioaddr + MTL_FPE_CTRL_STS);
	pfpec->txqpec = (value & MTL_FPE_CTRL_STS_PEC) >>
			MTL_FPE_CTRL_STS_PEC_SHIFT;

	value = TSN_RD32(ioaddr + MAC_FPE_CTRL_STS);
	pfpec->enable = (bool)(value & MAC_FPE_CTRL_STS_EFPE);

	*fpec = pfpec;
	TSN_INFO_NA("FPE: read config from HW done.\n");

	return 0;
}

int dwmac_get_fpe_pmac_sts(void *ioaddr, unsigned int *hrs)
{
	unsigned int value;

	if (!dw_tsn_feat_en[TSN_FEAT_ID_FPE])
		return -ENOTSUPP;

	value = TSN_RD32(ioaddr + MTL_FPE_CTRL_STS);
	*hrs = (value & MTL_FPE_CTRL_STS_HRS) >> MTL_FPE_CTRL_STS_HRS_SHIFT;

	if (hrs)
		TSN_INFO_NA("FPE: pMAC is in Hold state.\n");
	else
		TSN_INFO_NA("FPE: pMAC is in Release state.\n");

	return 0;
}

int dwmac_fpe_irq_status(void *ioaddr)
{
	unsigned int status;
	int fpe_state = FPE_STATE_UNKNOWN;

	status = TSN_RD32(ioaddr + MAC_FPE_CTRL_STS);

	if (status & MAC_FPE_CTRL_STS_TRSP) {
		TSN_INFO_NA("Respond mPacket is transmitted\n");
		fpe_state |= FPE_STATE_TRSP;
	}

	if (status & MAC_FPE_CTRL_STS_TVER) {
		TSN_INFO_NA("Verify mPacket is transmitted\n");
		fpe_state |= FPE_STATE_TVER;
	}

	if (status & MAC_FPE_CTRL_STS_RRSP) {
		dw_fpe_config.lp_fpe_support = 1;
		TSN_INFO_NA("Respond mPacket is received\n");
		fpe_state |= FPE_STATE_RRSP;
	}

	if (status & MAC_FPE_CTRL_STS_RVER) {
		TSN_INFO_NA("Verify mPacket is received\n");
		fpe_state |= FPE_STATE_RVER;
	}

	return fpe_state;
}

int dwmac_fpe_send_mpacket(void *ioaddr, enum mpacket_type type)
{
	unsigned int value;

	value = TSN_RD32(ioaddr + MAC_FPE_CTRL_STS);

	switch (type) {
	case MPACKET_VERIFY:
		dw_fpe_config.lp_fpe_support = 0;
		value &= ~MAC_FPE_CTRL_STS_SRSP;
		value |= MAC_FPE_CTRL_STS_SVER;
		break;
	case MPACKET_RESPONSE:
		value &= ~MAC_FPE_CTRL_STS_SVER;
		value |= MAC_FPE_CTRL_STS_SRSP;
		break;
	default:
		return -ENOTSUPP;
	}
	TSN_WR32(value, ioaddr + MAC_FPE_CTRL_STS);

	return 0;
}

int dwmac_cbs_recal_idleslope(void *ioaddr,
			      unsigned int queue,
			      unsigned int *idle_slope)
{
	unsigned int open_time;
	unsigned int hw_bank = dwmac_get_est_bank(ioaddr, 1);
	unsigned long long new_idle_slope;
	unsigned long long scaling = 0;
	unsigned long long cycle_time_ns =
			(dw_est_gc_config.gcb[hw_bank].gcrr.cycle_sec *
			 ONE_SEC_IN_NANOSEC) +
			dw_est_gc_config.gcb[hw_bank].gcrr.cycle_nsec;

	if (!cycle_time_ns) {
		TSN_WARN_NA("EST: Cycle time is 0.\n");
		TSN_WARN_NA("CBS idle slope will not be reconfigured.\n");

		return -EINVAL;
	}

	open_time = est_get_all_open_time(hw_bank,
					  cycle_time_ns,
					  queue);

	if (!open_time) {
		TSN_WARN("EST: Total gate open time for queue %d is 0\n",
			 queue);

		return -EINVAL;
	}

	scaling = cycle_time_ns;
	_DO_DIV_(scaling, open_time);

	new_idle_slope = *idle_slope * scaling;
	if (new_idle_slope > CBS_IDLESLOPE_MAX)
		new_idle_slope = CBS_IDLESLOPE_MAX;

	*idle_slope = new_idle_slope;

	return 0;
}
