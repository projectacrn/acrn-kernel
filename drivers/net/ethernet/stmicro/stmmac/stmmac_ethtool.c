/*******************************************************************************
  STMMAC Ethtool support

  Copyright (C) 2007-2009  STMicroelectronics Ltd

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/net_tstamp.h>
#include <asm/io.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <net/udp.h>

#include "stmmac.h"
#include "dwmac_dma.h"
#include "dw_tsn_lib.h"

#define REG_SPACE_SIZE	0x1060
#define MAC100_ETHTOOL_NAME	"st_mac100"
#define GMAC_ETHTOOL_NAME	"st_gmac"

#define ETHTOOL_DMA_OFFSET	55

struct stmmac_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

#define STMMAC_STAT(m)	\
	{ #m, FIELD_SIZEOF(struct stmmac_extra_stats, m),	\
	offsetof(struct stmmac_priv, xstats.m)}

static const struct stmmac_stats stmmac_gstrings_stats[] = {
	/* Transmit errors */
	STMMAC_STAT(tx_underflow),
	STMMAC_STAT(tx_carrier),
	STMMAC_STAT(tx_losscarrier),
	STMMAC_STAT(vlan_tag),
	STMMAC_STAT(tx_deferred),
	STMMAC_STAT(tx_vlan),
	STMMAC_STAT(tx_jabber),
	STMMAC_STAT(tx_frame_flushed),
	STMMAC_STAT(tx_payload_error),
	STMMAC_STAT(tx_ip_header_error),
	/* Receive errors */
	STMMAC_STAT(rx_desc),
	STMMAC_STAT(sa_filter_fail),
	STMMAC_STAT(overflow_error),
	STMMAC_STAT(ipc_csum_error),
	STMMAC_STAT(rx_collision),
	STMMAC_STAT(rx_crc_errors),
	STMMAC_STAT(dribbling_bit),
	STMMAC_STAT(rx_length),
	STMMAC_STAT(rx_mii),
	STMMAC_STAT(rx_multicast),
	STMMAC_STAT(rx_gmac_overflow),
	STMMAC_STAT(rx_watchdog),
	STMMAC_STAT(da_rx_filter_fail),
	STMMAC_STAT(sa_rx_filter_fail),
	STMMAC_STAT(rx_missed_cntr),
	STMMAC_STAT(rx_overflow_cntr),
	STMMAC_STAT(rx_vlan),
	/* Tx/Rx IRQ error info */
	STMMAC_STAT(tx_undeflow_irq),
	STMMAC_STAT(tx_process_stopped_irq),
	STMMAC_STAT(tx_jabber_irq),
	STMMAC_STAT(rx_overflow_irq),
	STMMAC_STAT(rx_buf_unav_irq),
	STMMAC_STAT(rx_process_stopped_irq),
	STMMAC_STAT(rx_watchdog_irq),
	STMMAC_STAT(tx_early_irq),
	STMMAC_STAT(fatal_bus_error_irq),
	/* Tx/Rx IRQ Events */
	STMMAC_STAT(rx_early_irq),
	STMMAC_STAT(threshold),
	STMMAC_STAT(tx_pkt_n),
	STMMAC_STAT(rx_pkt_n),
	STMMAC_STAT(normal_irq_n),
	STMMAC_STAT(rx_normal_irq_n),
	STMMAC_STAT(napi_poll),
	STMMAC_STAT(tx_normal_irq_n),
	STMMAC_STAT(tx_clean),
	STMMAC_STAT(tx_set_ic_bit),
	STMMAC_STAT(irq_receive_pmt_irq_n),
	/* MMC info */
	STMMAC_STAT(mmc_tx_irq_n),
	STMMAC_STAT(mmc_rx_irq_n),
	STMMAC_STAT(mmc_rx_csum_offload_irq_n),
	/* EEE */
	STMMAC_STAT(irq_tx_path_in_lpi_mode_n),
	STMMAC_STAT(irq_tx_path_exit_lpi_mode_n),
	STMMAC_STAT(irq_rx_path_in_lpi_mode_n),
	STMMAC_STAT(irq_rx_path_exit_lpi_mode_n),
	STMMAC_STAT(phy_eee_wakeup_error_n),
	/* Extended RDES status */
	STMMAC_STAT(ip_hdr_err),
	STMMAC_STAT(ip_payload_err),
	STMMAC_STAT(ip_csum_bypassed),
	STMMAC_STAT(ipv4_pkt_rcvd),
	STMMAC_STAT(ipv6_pkt_rcvd),
	STMMAC_STAT(no_ptp_rx_msg_type_ext),
	STMMAC_STAT(ptp_rx_msg_type_sync),
	STMMAC_STAT(ptp_rx_msg_type_follow_up),
	STMMAC_STAT(ptp_rx_msg_type_delay_req),
	STMMAC_STAT(ptp_rx_msg_type_delay_resp),
	STMMAC_STAT(ptp_rx_msg_type_pdelay_req),
	STMMAC_STAT(ptp_rx_msg_type_pdelay_resp),
	STMMAC_STAT(ptp_rx_msg_type_pdelay_follow_up),
	STMMAC_STAT(ptp_rx_msg_type_announce),
	STMMAC_STAT(ptp_rx_msg_type_management),
	STMMAC_STAT(ptp_rx_msg_pkt_reserved_type),
	STMMAC_STAT(ptp_frame_type),
	STMMAC_STAT(ptp_ver),
	STMMAC_STAT(timestamp_dropped),
	STMMAC_STAT(av_pkt_rcvd),
	STMMAC_STAT(av_tagged_pkt_rcvd),
	STMMAC_STAT(vlan_tag_priority_val),
	STMMAC_STAT(l3_filter_match),
	STMMAC_STAT(l4_filter_match),
	STMMAC_STAT(l3_l4_filter_no_match),
	/* PCS */
	STMMAC_STAT(irq_pcs_ane_n),
	STMMAC_STAT(irq_pcs_link_n),
	STMMAC_STAT(irq_rgmii_n),
	/* DEBUG */
	STMMAC_STAT(mtl_tx_status_fifo_full),
	STMMAC_STAT(mtl_tx_fifo_not_empty),
	STMMAC_STAT(mmtl_fifo_ctrl),
	STMMAC_STAT(mtl_tx_fifo_read_ctrl_write),
	STMMAC_STAT(mtl_tx_fifo_read_ctrl_wait),
	STMMAC_STAT(mtl_tx_fifo_read_ctrl_read),
	STMMAC_STAT(mtl_tx_fifo_read_ctrl_idle),
	STMMAC_STAT(mac_tx_in_pause),
	STMMAC_STAT(mac_tx_frame_ctrl_xfer),
	STMMAC_STAT(mac_tx_frame_ctrl_idle),
	STMMAC_STAT(mac_tx_frame_ctrl_wait),
	STMMAC_STAT(mac_tx_frame_ctrl_pause),
	STMMAC_STAT(mac_gmii_tx_proto_engine),
	STMMAC_STAT(mtl_rx_fifo_fill_level_full),
	STMMAC_STAT(mtl_rx_fifo_fill_above_thresh),
	STMMAC_STAT(mtl_rx_fifo_fill_below_thresh),
	STMMAC_STAT(mtl_rx_fifo_fill_level_empty),
	STMMAC_STAT(mtl_rx_fifo_read_ctrl_flush),
	STMMAC_STAT(mtl_rx_fifo_read_ctrl_read_data),
	STMMAC_STAT(mtl_rx_fifo_read_ctrl_status),
	STMMAC_STAT(mtl_rx_fifo_read_ctrl_idle),
	STMMAC_STAT(mtl_rx_fifo_ctrl_active),
	STMMAC_STAT(mac_rx_frame_ctrl_fifo),
	STMMAC_STAT(mac_gmii_rx_proto_engine),
	/* TSO */
	STMMAC_STAT(tx_tso_frames),
	STMMAC_STAT(tx_tso_nfrags),
};
#define STMMAC_STATS_LEN ARRAY_SIZE(stmmac_gstrings_stats)

/* HW MAC Management counters (if supported) */
#define STMMAC_MMC_STAT(m)	\
	{ #m, FIELD_SIZEOF(struct stmmac_counters, m),	\
	offsetof(struct stmmac_priv, mmc.m)}

static const struct stmmac_stats stmmac_mmc[] = {
	STMMAC_MMC_STAT(mmc_tx_octetcount_gb),
	STMMAC_MMC_STAT(mmc_tx_framecount_gb),
	STMMAC_MMC_STAT(mmc_tx_broadcastframe_g),
	STMMAC_MMC_STAT(mmc_tx_multicastframe_g),
	STMMAC_MMC_STAT(mmc_tx_64_octets_gb),
	STMMAC_MMC_STAT(mmc_tx_65_to_127_octets_gb),
	STMMAC_MMC_STAT(mmc_tx_128_to_255_octets_gb),
	STMMAC_MMC_STAT(mmc_tx_256_to_511_octets_gb),
	STMMAC_MMC_STAT(mmc_tx_512_to_1023_octets_gb),
	STMMAC_MMC_STAT(mmc_tx_1024_to_max_octets_gb),
	STMMAC_MMC_STAT(mmc_tx_unicast_gb),
	STMMAC_MMC_STAT(mmc_tx_multicast_gb),
	STMMAC_MMC_STAT(mmc_tx_broadcast_gb),
	STMMAC_MMC_STAT(mmc_tx_underflow_error),
	STMMAC_MMC_STAT(mmc_tx_singlecol_g),
	STMMAC_MMC_STAT(mmc_tx_multicol_g),
	STMMAC_MMC_STAT(mmc_tx_deferred),
	STMMAC_MMC_STAT(mmc_tx_latecol),
	STMMAC_MMC_STAT(mmc_tx_exesscol),
	STMMAC_MMC_STAT(mmc_tx_carrier_error),
	STMMAC_MMC_STAT(mmc_tx_octetcount_g),
	STMMAC_MMC_STAT(mmc_tx_framecount_g),
	STMMAC_MMC_STAT(mmc_tx_excessdef),
	STMMAC_MMC_STAT(mmc_tx_pause_frame),
	STMMAC_MMC_STAT(mmc_tx_vlan_frame_g),
	STMMAC_MMC_STAT(mmc_rx_framecount_gb),
	STMMAC_MMC_STAT(mmc_rx_octetcount_gb),
	STMMAC_MMC_STAT(mmc_rx_octetcount_g),
	STMMAC_MMC_STAT(mmc_rx_broadcastframe_g),
	STMMAC_MMC_STAT(mmc_rx_multicastframe_g),
	STMMAC_MMC_STAT(mmc_rx_crc_error),
	STMMAC_MMC_STAT(mmc_rx_align_error),
	STMMAC_MMC_STAT(mmc_rx_run_error),
	STMMAC_MMC_STAT(mmc_rx_jabber_error),
	STMMAC_MMC_STAT(mmc_rx_undersize_g),
	STMMAC_MMC_STAT(mmc_rx_oversize_g),
	STMMAC_MMC_STAT(mmc_rx_64_octets_gb),
	STMMAC_MMC_STAT(mmc_rx_65_to_127_octets_gb),
	STMMAC_MMC_STAT(mmc_rx_128_to_255_octets_gb),
	STMMAC_MMC_STAT(mmc_rx_256_to_511_octets_gb),
	STMMAC_MMC_STAT(mmc_rx_512_to_1023_octets_gb),
	STMMAC_MMC_STAT(mmc_rx_1024_to_max_octets_gb),
	STMMAC_MMC_STAT(mmc_rx_unicast_g),
	STMMAC_MMC_STAT(mmc_rx_length_error),
	STMMAC_MMC_STAT(mmc_rx_autofrangetype),
	STMMAC_MMC_STAT(mmc_rx_pause_frames),
	STMMAC_MMC_STAT(mmc_rx_fifo_overflow),
	STMMAC_MMC_STAT(mmc_rx_vlan_frames_gb),
	STMMAC_MMC_STAT(mmc_rx_watchdog_error),
	STMMAC_MMC_STAT(mmc_rx_ipc_intr_mask),
	STMMAC_MMC_STAT(mmc_rx_ipc_intr),
	STMMAC_MMC_STAT(mmc_rx_ipv4_gd),
	STMMAC_MMC_STAT(mmc_rx_ipv4_hderr),
	STMMAC_MMC_STAT(mmc_rx_ipv4_nopay),
	STMMAC_MMC_STAT(mmc_rx_ipv4_frag),
	STMMAC_MMC_STAT(mmc_rx_ipv4_udsbl),
	STMMAC_MMC_STAT(mmc_rx_ipv4_gd_octets),
	STMMAC_MMC_STAT(mmc_rx_ipv4_hderr_octets),
	STMMAC_MMC_STAT(mmc_rx_ipv4_nopay_octets),
	STMMAC_MMC_STAT(mmc_rx_ipv4_frag_octets),
	STMMAC_MMC_STAT(mmc_rx_ipv4_udsbl_octets),
	STMMAC_MMC_STAT(mmc_rx_ipv6_gd_octets),
	STMMAC_MMC_STAT(mmc_rx_ipv6_hderr_octets),
	STMMAC_MMC_STAT(mmc_rx_ipv6_nopay_octets),
	STMMAC_MMC_STAT(mmc_rx_ipv6_gd),
	STMMAC_MMC_STAT(mmc_rx_ipv6_hderr),
	STMMAC_MMC_STAT(mmc_rx_ipv6_nopay),
	STMMAC_MMC_STAT(mmc_rx_udp_gd),
	STMMAC_MMC_STAT(mmc_rx_udp_err),
	STMMAC_MMC_STAT(mmc_rx_tcp_gd),
	STMMAC_MMC_STAT(mmc_rx_tcp_err),
	STMMAC_MMC_STAT(mmc_rx_icmp_gd),
	STMMAC_MMC_STAT(mmc_rx_icmp_err),
	STMMAC_MMC_STAT(mmc_rx_udp_gd_octets),
	STMMAC_MMC_STAT(mmc_rx_udp_err_octets),
	STMMAC_MMC_STAT(mmc_rx_tcp_gd_octets),
	STMMAC_MMC_STAT(mmc_rx_tcp_err_octets),
	STMMAC_MMC_STAT(mmc_rx_icmp_gd_octets),
	STMMAC_MMC_STAT(mmc_rx_icmp_err_octets),
};
#define STMMAC_MMC_STATS_LEN ARRAY_SIZE(stmmac_mmc)

/* All test entries will be added before MAX_TEST_CASES */
enum stmmac_diagnostics_cases {
	TEST_MAC_LOOP,
	MAX_TEST_CASES,
	TOTAL_PASSED,
	TOTAL_FAILED,
	TOTAL_NOT_TESTED
};

static const char stmmac_gstrings_test[][ETH_GSTRING_LEN] = {
	[TEST_MAC_LOOP]    = "MAC loopback test     (online)",
	[MAX_TEST_CASES]   = "8888888888888888888888888888888",
	[TOTAL_PASSED]     = "Total test passed             ",
	[TOTAL_FAILED]     = "Total test failed             ",
	[TOTAL_NOT_TESTED] = "Total not tested              ",
};

#define STMMAC_TEST_LEN (sizeof(stmmac_gstrings_test) / ETH_GSTRING_LEN)

static void stmmac_ethtool_getdrvinfo(struct net_device *dev,
				      struct ethtool_drvinfo *info)
{
	struct stmmac_priv *priv = netdev_priv(dev);

	if (priv->plat->has_gmac || priv->plat->has_gmac4)
		strlcpy(info->driver, GMAC_ETHTOOL_NAME, sizeof(info->driver));
	else
		strlcpy(info->driver, MAC100_ETHTOOL_NAME,
			sizeof(info->driver));

	strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
}

static int stmmac_ethtool_get_link_ksettings(struct net_device *dev,
					     struct ethtool_link_ksettings *cmd)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	struct phy_device *phy = dev->phydev;

	if (priv->hw->pcs & STMMAC_PCS_RGMII ||
	    priv->hw->pcs & STMMAC_PCS_SGMII) {
		struct rgmii_adv adv;
		u32 supported, advertising, lp_advertising;

		if (!priv->xstats.pcs_link) {
			cmd->base.speed = SPEED_UNKNOWN;
			cmd->base.duplex = DUPLEX_UNKNOWN;
			return 0;
		}
		cmd->base.duplex = priv->xstats.pcs_duplex;

		cmd->base.speed = priv->xstats.pcs_speed;

		/* Get and convert ADV/LP_ADV from the HW AN registers */
		if (stmmac_pcs_get_adv_lp(priv, priv->ioaddr, &adv))
			return -EOPNOTSUPP;	/* should never happen indeed */

		/* Encoding of PSE bits is defined in 802.3z, 37.2.1.4 */

		ethtool_convert_link_mode_to_legacy_u32(
			&supported, cmd->link_modes.supported);
		ethtool_convert_link_mode_to_legacy_u32(
			&advertising, cmd->link_modes.advertising);
		ethtool_convert_link_mode_to_legacy_u32(
			&lp_advertising, cmd->link_modes.lp_advertising);

		if (adv.pause & STMMAC_PCS_PAUSE)
			advertising |= ADVERTISED_Pause;
		if (adv.pause & STMMAC_PCS_ASYM_PAUSE)
			advertising |= ADVERTISED_Asym_Pause;
		if (adv.lp_pause & STMMAC_PCS_PAUSE)
			lp_advertising |= ADVERTISED_Pause;
		if (adv.lp_pause & STMMAC_PCS_ASYM_PAUSE)
			lp_advertising |= ADVERTISED_Asym_Pause;

		/* Reg49[3] always set because ANE is always supported */
		cmd->base.autoneg = ADVERTISED_Autoneg;
		supported |= SUPPORTED_Autoneg;
		advertising |= ADVERTISED_Autoneg;
		lp_advertising |= ADVERTISED_Autoneg;

		if (adv.duplex) {
			supported |= (SUPPORTED_1000baseT_Full |
				      SUPPORTED_100baseT_Full |
				      SUPPORTED_10baseT_Full);
			advertising |= (ADVERTISED_1000baseT_Full |
					ADVERTISED_100baseT_Full |
					ADVERTISED_10baseT_Full);
		} else {
			supported |= (SUPPORTED_1000baseT_Half |
				      SUPPORTED_100baseT_Half |
				      SUPPORTED_10baseT_Half);
			advertising |= (ADVERTISED_1000baseT_Half |
					ADVERTISED_100baseT_Half |
					ADVERTISED_10baseT_Half);
		}
		if (adv.lp_duplex)
			lp_advertising |= (ADVERTISED_1000baseT_Full |
					   ADVERTISED_100baseT_Full |
					   ADVERTISED_10baseT_Full);
		else
			lp_advertising |= (ADVERTISED_1000baseT_Half |
					   ADVERTISED_100baseT_Half |
					   ADVERTISED_10baseT_Half);
		cmd->base.port = PORT_OTHER;

		ethtool_convert_legacy_u32_to_link_mode(
			cmd->link_modes.supported, supported);
		ethtool_convert_legacy_u32_to_link_mode(
			cmd->link_modes.advertising, advertising);
		ethtool_convert_legacy_u32_to_link_mode(
			cmd->link_modes.lp_advertising, lp_advertising);

		return 0;
	}

	if (phy == NULL) {
		pr_err("%s: %s: PHY is not registered\n",
		       __func__, dev->name);
		return -ENODEV;
	}
	if (!netif_running(dev)) {
		pr_err("%s: interface is disabled: we cannot track "
		"link speed / duplex setting\n", dev->name);
		return -EBUSY;
	}
	phy_ethtool_ksettings_get(phy, cmd);
	return 0;
}

static int
stmmac_ethtool_set_link_ksettings(struct net_device *dev,
				  const struct ethtool_link_ksettings *cmd)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	struct phy_device *phy = dev->phydev;
	int rc;

	if (priv->hw->pcs & STMMAC_PCS_RGMII ||
	    priv->hw->pcs & STMMAC_PCS_SGMII) {
		u32 mask = ADVERTISED_Autoneg | ADVERTISED_Pause;

		/* Only support ANE */
		if (cmd->base.autoneg != AUTONEG_ENABLE)
			return -EINVAL;

		mask &= (ADVERTISED_1000baseT_Half |
			ADVERTISED_1000baseT_Full |
			ADVERTISED_100baseT_Half |
			ADVERTISED_100baseT_Full |
			ADVERTISED_10baseT_Half |
			ADVERTISED_10baseT_Full);

		mutex_lock(&priv->lock);
		stmmac_pcs_ctrl_ane(priv, priv->ioaddr, 1, priv->hw->ps, 0);
		stmmac_xpcs_ctrl_ane(priv, dev, 1, 0);
		mutex_unlock(&priv->lock);

		return 0;
	}

	rc = phy_ethtool_ksettings_set(phy, cmd);

	return rc;
}

static u32 stmmac_ethtool_getmsglevel(struct net_device *dev)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void stmmac_ethtool_setmsglevel(struct net_device *dev, u32 level)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	priv->msg_enable = level;

}

static int stmmac_check_if_running(struct net_device *dev)
{
	if (!netif_running(dev))
		return -EBUSY;
	return 0;
}

static int stmmac_ethtool_get_regs_len(struct net_device *dev)
{
	return REG_SPACE_SIZE;
}

static void stmmac_ethtool_gregs(struct net_device *dev,
			  struct ethtool_regs *regs, void *space)
{
	u32 *reg_space = (u32 *) space;

	struct stmmac_priv *priv = netdev_priv(dev);

	memset(reg_space, 0x0, REG_SPACE_SIZE);

	stmmac_dump_mac_regs(priv, priv->hw, reg_space);
	stmmac_dump_dma_regs(priv, priv->ioaddr, reg_space);
	/* Copy DMA registers to where ethtool expects them */
	memcpy(&reg_space[ETHTOOL_DMA_OFFSET], &reg_space[DMA_BUS_MODE / 4],
	       NUM_DWMAC1000_DMA_REGS * 4);
}

static void
stmmac_get_pauseparam(struct net_device *netdev,
		      struct ethtool_pauseparam *pause)
{
	struct stmmac_priv *priv = netdev_priv(netdev);
	struct rgmii_adv adv_lp;

	pause->rx_pause = 0;
	pause->tx_pause = 0;

	if (priv->hw->pcs && !stmmac_pcs_get_adv_lp(priv, priv->ioaddr, &adv_lp)) {
		pause->autoneg = 1;
		if (!adv_lp.pause)
			return;
	} else if (priv->plat->has_xpcs &&
		   !stmmac_xpcs_get_adv_lp(priv, netdev, &adv_lp)) {
		pause->autoneg = 1;
		if (!adv_lp.pause)
			return;
	} else {
		if (!(netdev->phydev->supported & SUPPORTED_Pause) ||
		    !(netdev->phydev->supported & SUPPORTED_Asym_Pause))
			return;
	}

	pause->autoneg = netdev->phydev->autoneg;

	if (priv->flow_ctrl & FLOW_RX)
		pause->rx_pause = 1;
	if (priv->flow_ctrl & FLOW_TX)
		pause->tx_pause = 1;

}

static int
stmmac_set_pauseparam(struct net_device *netdev,
		      struct ethtool_pauseparam *pause)
{
	struct stmmac_priv *priv = netdev_priv(netdev);
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	struct phy_device *phy = netdev->phydev;
	int new_pause = FLOW_OFF;
	struct rgmii_adv adv_lp;

	if (priv->hw->pcs && !stmmac_pcs_get_adv_lp(priv, priv->ioaddr, &adv_lp)) {
		pause->autoneg = 1;
		if (!adv_lp.pause)
			return -EOPNOTSUPP;
	} else if (priv->plat->has_xpcs &&
		   !stmmac_xpcs_get_adv_lp(priv, netdev, &adv_lp)) {
		pause->autoneg = 1;
		if (!adv_lp.pause)
			return -EOPNOTSUPP;
	} else {
		if (!(phy->supported & SUPPORTED_Pause) ||
		    !(phy->supported & SUPPORTED_Asym_Pause))
			return -EOPNOTSUPP;
	}

	if (pause->rx_pause)
		new_pause |= FLOW_RX;
	if (pause->tx_pause)
		new_pause |= FLOW_TX;

	priv->flow_ctrl = new_pause;
	phy->autoneg = pause->autoneg;

	if (phy->autoneg) {
		if (netif_running(netdev))
			return phy_start_aneg(phy);
	}

	stmmac_flow_ctrl(priv, priv->hw, phy->duplex, priv->flow_ctrl,
			priv->pause, tx_cnt);
	return 0;
}

struct stmmac_lbst_priv {
	struct packet_type pt;
	struct completion comp;
};

static const char test_text[] = "STMMAC LOOPBACK SELF TEST";

#define STMMAC_LB_TIMEOUT (msecs_to_jiffies(200))

static struct sk_buff *stmmac_lb_create_udp_skb(struct net_device *netdev)
{
	struct stmmac_priv *priv = netdev_priv(netdev);
	struct sk_buff *skb;
	struct ethhdr *ethh;
	struct udphdr *udph;
	struct iphdr *iph;
	int datalen, iplen;

	datalen = sizeof(test_text);
	iplen = sizeof(*iph) + sizeof(*udph) + datalen;
	skb = netdev_alloc_skb_ip_align(netdev, iplen);
	if (!skb)
		return NULL;

	/* Reserve for ethernet and IP header */
	ethh = (struct ethhdr *)skb_push(skb, ETH_HLEN);
	skb_reset_mac_header(skb);

	skb_set_network_header(skb, skb->len);
	iph = (struct iphdr *)skb_put(skb, sizeof(*iph));

	skb_set_transport_header(skb, skb->len);
	udph = (struct udphdr *)skb_put(skb, sizeof(*udph));

	/* Fill ETH header */
	ether_addr_copy(ethh->h_dest, priv->dev->dev_addr);
	eth_zero_addr(ethh->h_source);
	ethh->h_proto = htons(ETH_P_IP);

	/* Fill IP header */
	iph->ihl = 5;
	iph->ttl = 32;
	iph->version = 4;
	iph->protocol = IPPROTO_UDP;
	iph->tot_len = htons(iplen);
	iph->frag_off = 0;
	iph->saddr = 0;
	iph->daddr = 0;
	iph->tos = 0;
	iph->id = 0;
	ip_send_check(iph);

	/* Fill UDP header */
	udph->source = htons(9);
	udph->dest = htons(9); /* Discard Protocol Port */
	udph->len = htons(datalen + sizeof(*udph));
	udph->check = 0;

	/* Fill UDP data - test string */
	memcpy(skb_put(skb, sizeof(test_text)), test_text, sizeof(test_text));

	skb->csum = 0;
	skb->ip_summed = CHECKSUM_PARTIAL;
	udp4_hwcsum(skb, iph->saddr, iph->daddr);

	skb->protocol = htons(ETH_P_IP);
	skb->pkt_type = PACKET_HOST;

	return skb;
}

static int stmmac_lb_validate_udp_skb(struct sk_buff *skb,
				      struct net_device *netdev,
				      struct packet_type *pt,
				      struct net_device *orig_netdev)
{
	struct stmmac_lbst_priv *lbstp = pt->af_packet_priv;
	struct ethhdr *ethh;
	struct udphdr *udph;
	struct iphdr *iph;
	char *rx_text;

	if (skb->protocol != htons(ETH_P_IP))
		goto out;

	ethh = (struct ethhdr *)skb_mac_header(skb);
	if (!ether_addr_equal(ethh->h_dest, orig_netdev->dev_addr))
		goto out;

	iph = ip_hdr(skb);
	if (iph->protocol != IPPROTO_UDP)
		goto out;

	udph = udp_hdr(skb);
	if (udph->dest != htons(9))
		goto out;

	rx_text = ((char *)udph + sizeof(*udph));
	if (strncmp(rx_text, test_text, sizeof(test_text)))
		goto out;

	complete(&lbstp->comp);
out:
	kfree_skb(skb);
	return 0;
}

static void stmmac_lb_set_mode(struct stmmac_priv *priv, bool mode)
{
	mutex_lock(&priv->lock);
	stmmac_set_loopback_mode(priv, priv->hw, mode);
	mutex_unlock(&priv->lock);
}

static void stmmac_lb_setup(struct stmmac_priv *priv,
			    struct stmmac_lbst_priv *lbstp)
{
	init_completion(&lbstp->comp);

	lbstp->pt.type = htons(ETH_P_ALL);
	lbstp->pt.func = stmmac_lb_validate_udp_skb;
	lbstp->pt.dev = priv->dev;
	lbstp->pt.af_packet_priv = lbstp;
	dev_add_pack(&lbstp->pt);

	stmmac_lb_set_mode(priv, true);
}

static void stmmac_lb_clean(struct stmmac_priv *priv,
			    struct stmmac_lbst_priv *lbstp)
{
	dev_remove_pack(&lbstp->pt);
	stmmac_lb_set_mode(priv, false);
}

static int stmmac_loopback_test(struct net_device *netdev, u64 *data)
{
	struct stmmac_priv *priv = netdev_priv(netdev);
	const struct stmmac_ops *mac_dev_ops = priv->hw->mac;
	struct sk_buff *skb;
	struct stmmac_lbst_priv *lbstp;
	int err;

	if (!mac_dev_ops->set_loopback_mode) {
		netdev_err(priv->dev, "MAC loopback self test function %s\n",
			   "is not supported");
		return 0;
	}

	/* Dwmac MAC level loopback hw limitaion:
	 * 1.) only work in full duplex mode
	 * 2.) link partner is required to supply rx clk
	 * 3.) big packet loopback is not supported
	 */
	if (!netdev->phydev->duplex) {
		netdev_err(priv->dev, "Cannot perform MAC loopback test %s\n",
			   "while not in full duplex mode");
		return 0;
	}

	data[TOTAL_NOT_TESTED]--;

	lbstp = kzalloc(sizeof(*lbstp), GFP_KERNEL);
	if (!lbstp)
		return -ENOMEM;

	skb = stmmac_lb_create_udp_skb(netdev);
	if (!skb) {
		err = -ENOMEM;
		goto out;
	}

	stmmac_lb_setup(priv, lbstp);

	err = dev_queue_xmit(skb);
	if (err)
		goto cleanup;

	err = wait_for_completion_interruptible_timeout(&lbstp->comp,
							STMMAC_LB_TIMEOUT);
	if (!err) {
		err = -ETIME;
		goto cleanup;
	} else if (err == -ERESTARTSYS) {
		goto cleanup;
	} else {
		err = 0;
	}

	data[TOTAL_PASSED]++;

cleanup:
	stmmac_lb_clean(priv, lbstp);
out:
	kfree(lbstp);
	data[TEST_MAC_LOOP] = err;
	return err;
}

static void stmmac_diag_test(struct net_device *netdev,
			     struct ethtool_test *eth_test, u64 *data)
{
	/* ToDo: self-test mechanism */
	data[TEST_MAC_LOOP] = 0;
	data[MAX_TEST_CASES] = 888888;
	data[TOTAL_PASSED] = 0;
	data[TOTAL_FAILED] = 0;
	data[TOTAL_NOT_TESTED] = MAX_TEST_CASES;

	if (!(eth_test->flags & ETH_TEST_FL_OFFLINE)) {
		if (stmmac_loopback_test(netdev, data)) {
			eth_test->flags |= ETH_TEST_FL_FAILED;
			data[TOTAL_FAILED]++;
		}
	}
}

static void stmmac_get_ethtool_stats(struct net_device *dev,
				 struct ethtool_stats *dummy, u64 *data)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	unsigned long count;
	int i, j = 0, ret;

	if (priv->dma_cap.asp) {
		for (i = 0; i < STMMAC_SAFETY_FEAT_SIZE; i++) {
			if (!stmmac_safety_feat_dump(priv, &priv->sstats, i,
						&count, NULL))
				data[j++] = count;
		}
	}

	/* Update the DMA HW counters for dwmac10/100 */
	ret = stmmac_dma_diagnostic_fr(priv, &dev->stats, (void *) &priv->xstats,
			priv->ioaddr);
	if (ret) {
		/* If supported, for new GMAC chips expose the MMC counters */
		if (priv->dma_cap.rmon) {
			dwmac_mmc_read(priv->mmcaddr, &priv->mmc);

			for (i = 0; i < STMMAC_MMC_STATS_LEN; i++) {
				char *p;
				p = (char *)priv + stmmac_mmc[i].stat_offset;

				data[j++] = (stmmac_mmc[i].sizeof_stat ==
					     sizeof(u64)) ? (*(u64 *)p) :
					     (*(u32 *)p);
			}
		}
		if (priv->eee_enabled) {
			int val = phy_get_eee_err(dev->phydev);
			if (val)
				priv->xstats.phy_eee_wakeup_error_n = val;
		}

		if (priv->synopsys_id >= DWMAC_CORE_3_50)
			stmmac_mac_debug(priv, priv->ioaddr,
					(void *)&priv->xstats,
					rx_queues_count, tx_queues_count);
	}
	for (i = 0; i < STMMAC_STATS_LEN; i++) {
		char *p = (char *)priv + stmmac_gstrings_stats[i].stat_offset;
		data[j++] = (stmmac_gstrings_stats[i].sizeof_stat ==
			     sizeof(u64)) ? (*(u64 *)p) : (*(u32 *)p);
	}
}

static int stmmac_get_sset_count(struct net_device *netdev, int sset)
{
	struct stmmac_priv *priv = netdev_priv(netdev);
	int i, len, safety_len = 0;

	switch (sset) {
	case ETH_SS_STATS:
		len = STMMAC_STATS_LEN;

		if (priv->dma_cap.rmon)
			len += STMMAC_MMC_STATS_LEN;
		if (priv->dma_cap.asp) {
			for (i = 0; i < STMMAC_SAFETY_FEAT_SIZE; i++) {
				if (!stmmac_safety_feat_dump(priv,
							&priv->sstats, i,
							NULL, NULL))
					safety_len++;
			}

			len += safety_len;
		}

		return len;
	case ETH_SS_TEST:
		return STMMAC_TEST_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static void stmmac_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	int i;
	u8 *p = data;
	struct stmmac_priv *priv = netdev_priv(dev);

	switch (stringset) {
	case ETH_SS_STATS:
		if (priv->dma_cap.asp) {
			for (i = 0; i < STMMAC_SAFETY_FEAT_SIZE; i++) {
				const char *desc;
				if (!stmmac_safety_feat_dump(priv,
							&priv->sstats, i,
							NULL, &desc)) {
					memcpy(p, desc, ETH_GSTRING_LEN);
					p += ETH_GSTRING_LEN;
				}
			}
		}
		if (priv->dma_cap.rmon)
			for (i = 0; i < STMMAC_MMC_STATS_LEN; i++) {
				memcpy(p, stmmac_mmc[i].stat_string,
				       ETH_GSTRING_LEN);
				p += ETH_GSTRING_LEN;
			}
		for (i = 0; i < STMMAC_STATS_LEN; i++) {
			memcpy(p, stmmac_gstrings_stats[i].stat_string,
				ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_TEST:
		memcpy(data, *stmmac_gstrings_test,
		       STMMAC_TEST_LEN * ETH_GSTRING_LEN);
		break;
	default:
		WARN_ON(1);
		break;
	}
}

/* Currently only support WOL through Magic packet. */
static void stmmac_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct stmmac_priv *priv = netdev_priv(dev);

	mutex_lock(&priv->lock);
	if (device_can_wakeup(priv->device)) {
		wol->supported = WAKE_MAGIC | WAKE_UCAST;
		wol->wolopts = priv->wolopts;
	}
	mutex_unlock(&priv->lock);
}

static int stmmac_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	u32 support = WAKE_MAGIC | WAKE_UCAST;

	/* By default almost all GMAC devices support the WoL via
	 * magic frame but we can disable it if the HW capability
	 * register shows no support for pmt_magic_frame. */
	if ((priv->hw_cap_support) && (!priv->dma_cap.pmt_magic_frame))
		wol->wolopts &= ~WAKE_MAGIC;

	if (!device_can_wakeup(priv->device))
		return -EINVAL;

	if (wol->wolopts & ~support)
		return -EINVAL;

	if (wol->wolopts) {
		pr_info("stmmac: wakeup enable\n");
		device_set_wakeup_enable(priv->device, 1);
		enable_irq_wake(priv->wol_irq);
	} else {
		device_set_wakeup_enable(priv->device, 0);
		disable_irq_wake(priv->wol_irq);
	}

	mutex_lock(&priv->lock);
	priv->wolopts = wol->wolopts;
	mutex_unlock(&priv->lock);

	return 0;
}

static int stmmac_ethtool_op_get_eee(struct net_device *dev,
				     struct ethtool_eee *edata)
{
	struct stmmac_priv *priv = netdev_priv(dev);

	if (!priv->dma_cap.eee)
		return -EOPNOTSUPP;

	edata->eee_enabled = priv->eee_enabled;
	edata->eee_active = priv->eee_active;
	edata->tx_lpi_timer = priv->tx_lpi_timer;

	return phy_ethtool_get_eee(dev->phydev, edata);
}

static int stmmac_ethtool_op_set_eee(struct net_device *dev,
				     struct ethtool_eee *edata)
{
	struct stmmac_priv *priv = netdev_priv(dev);

	priv->eee_enabled = edata->eee_enabled;

	if (!priv->eee_enabled)
		stmmac_disable_eee_mode(priv);
	else {
		/* We are asking for enabling the EEE but it is safe
		 * to verify all by invoking the eee_init function.
		 * In case of failure it will return an error.
		 */
		priv->eee_enabled = stmmac_eee_init(priv);
		if (!priv->eee_enabled)
			return -EOPNOTSUPP;

		/* Do not change tx_lpi_timer in case of failure */
		priv->tx_lpi_timer = edata->tx_lpi_timer;
	}

	return phy_ethtool_set_eee(dev->phydev, edata);
}

static u32 stmmac_usec2riwt(u32 usec, struct stmmac_priv *priv)
{
	unsigned long clk = clk_get_rate(priv->plat->stmmac_clk);

	if (!clk)
		return 0;

	return (usec * (clk / 1000000)) / 256;
}

static u32 stmmac_riwt2usec(u32 riwt, struct stmmac_priv *priv)
{
	unsigned long clk = clk_get_rate(priv->plat->stmmac_clk);

	if (!clk)
		return 0;

	return (riwt * 256) / (clk / 1000000);
}

static int stmmac_get_coalesce(struct net_device *dev,
			       struct ethtool_coalesce *ec)
{
	struct stmmac_priv *priv = netdev_priv(dev);

	ec->tx_coalesce_usecs = priv->tx_coal_timer;
	ec->tx_max_coalesced_frames = priv->tx_coal_frames;

	if (priv->use_riwt)
		ec->rx_coalesce_usecs = stmmac_riwt2usec(priv->rx_riwt, priv);

	return 0;
}

static int stmmac_set_coalesce(struct net_device *dev,
			       struct ethtool_coalesce *ec)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	unsigned int rx_riwt;

	/* Check not supported parameters  */
	if ((ec->rx_max_coalesced_frames) || (ec->rx_coalesce_usecs_irq) ||
	    (ec->rx_max_coalesced_frames_irq) || (ec->tx_coalesce_usecs_irq) ||
	    (ec->use_adaptive_rx_coalesce) || (ec->use_adaptive_tx_coalesce) ||
	    (ec->pkt_rate_low) || (ec->rx_coalesce_usecs_low) ||
	    (ec->rx_max_coalesced_frames_low) || (ec->tx_coalesce_usecs_high) ||
	    (ec->tx_max_coalesced_frames_low) || (ec->pkt_rate_high) ||
	    (ec->tx_coalesce_usecs_low) || (ec->rx_coalesce_usecs_high) ||
	    (ec->rx_max_coalesced_frames_high) ||
	    (ec->tx_max_coalesced_frames_irq) ||
	    (ec->stats_block_coalesce_usecs) ||
	    (ec->tx_max_coalesced_frames_high) || (ec->rate_sample_interval))
		return -EOPNOTSUPP;

	if (ec->rx_coalesce_usecs == 0)
		return -EINVAL;

	if ((ec->tx_coalesce_usecs == 0) &&
	    (ec->tx_max_coalesced_frames == 0))
		return -EINVAL;

	if ((ec->tx_coalesce_usecs > STMMAC_MAX_COAL_TX_TICK) ||
	    (ec->tx_max_coalesced_frames > STMMAC_TX_MAX_FRAMES))
		return -EINVAL;

	rx_riwt = stmmac_usec2riwt(ec->rx_coalesce_usecs, priv);

	if ((rx_riwt > MAX_DMA_RIWT) || (rx_riwt < MIN_DMA_RIWT))
		return -EINVAL;
	else if (!priv->use_riwt)
		return -EOPNOTSUPP;

	/* Only copy relevant parameters, ignore all others. */
	priv->tx_coal_frames = ec->tx_max_coalesced_frames;
	priv->tx_coal_timer = ec->tx_coalesce_usecs;
	priv->rx_riwt = rx_riwt;
	stmmac_rx_watchdog(priv, priv->ioaddr, priv->rx_riwt, rx_cnt);

	return 0;
}

static int stmmac_get_ts_info(struct net_device *dev,
			      struct ethtool_ts_info *info)
{
	struct stmmac_priv *priv = netdev_priv(dev);

	if ((priv->dma_cap.time_stamp || priv->dma_cap.atime_stamp)) {

		info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE |
					SOF_TIMESTAMPING_TX_HARDWARE |
					SOF_TIMESTAMPING_RX_SOFTWARE |
					SOF_TIMESTAMPING_RX_HARDWARE |
					SOF_TIMESTAMPING_SOFTWARE |
					SOF_TIMESTAMPING_RAW_HARDWARE;

		if (priv->ptp_clock)
			info->phc_index = ptp_clock_index(priv->ptp_clock);

		info->tx_types = (1 << HWTSTAMP_TX_OFF) | (1 << HWTSTAMP_TX_ON);

		info->rx_filters = ((1 << HWTSTAMP_FILTER_NONE) |
				    (1 << HWTSTAMP_FILTER_PTP_V1_L4_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V1_L4_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_ALL));
		return 0;
	} else
		return ethtool_op_get_ts_info(dev, info);
}

static int stmmac_get_tunable(struct net_device *dev,
			      const struct ethtool_tunable *tuna, void *data)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	int ret = 0;

	switch (tuna->id) {
	case ETHTOOL_RX_COPYBREAK:
		*(u32 *)data = priv->rx_copybreak;
		break;
	case ETHTOOL_TX_EST_TILS:
		ret = stmmac_get_tsn_hwtunable(priv, TSN_HWTUNA_TX_EST_TILS,
					       data);
		break;
	case ETHTOOL_TX_EST_PTOV:
		ret = stmmac_get_tsn_hwtunable(priv, TSN_HWTUNA_TX_EST_PTOV,
					       data);
		break;
	case ETHTOOL_TX_EST_CTOV:
		ret = stmmac_get_tsn_hwtunable(priv, TSN_HWTUNA_TX_EST_CTOV,
					       data);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int stmmac_set_tunable(struct net_device *dev,
			      const struct ethtool_tunable *tuna,
			      const void *data)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	int ret = 0;

	switch (tuna->id) {
	case ETHTOOL_RX_COPYBREAK:
		priv->rx_copybreak = *(u32 *)data;
		break;
	case ETHTOOL_TX_EST_TILS:
		ret = stmmac_set_tsn_hwtunable(priv, priv->ioaddr,
					       TSN_HWTUNA_TX_EST_TILS,
					       data);
		break;
	case ETHTOOL_TX_EST_PTOV:
		ret = stmmac_set_tsn_hwtunable(priv, priv->ioaddr,
					       TSN_HWTUNA_TX_EST_PTOV,
					       data);
		break;
	case ETHTOOL_TX_EST_CTOV:
		ret = stmmac_set_tsn_hwtunable(priv, priv->ioaddr,
					       TSN_HWTUNA_TX_EST_CTOV,
					       data);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int stmmac_ethtool_get_est_gcl_depth(struct net_device *dev)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	struct tsn_hw_cap *tsn_hwcap;

	stmmac_get_tsn_hwcap(priv, &tsn_hwcap);

	return tsn_hwcap->gcl_depth;
}

static int stmmac_ethtool_get_est_gcl_length(struct net_device *dev, int own)
{
	int bank, ret;
	int gcl_length;
	struct stmmac_priv *priv = netdev_priv(dev);

	if (own >= EST_GCL_BANK_MAX)
		return -EINVAL;

	bank = stmmac_get_est_bank(priv, priv->ioaddr, own);

	ret = stmmac_get_est_gcrr_llr(priv, priv->ioaddr, &gcl_length,
				      bank, 1);
	if (ret) {
		dev_err(priv->device, "fail to get EST GC length.\n");

		return 0;
	}

	return gcl_length;
}

static int stmmac_ethtool_get_est_gcl(struct net_device *dev,
				      struct ethtool_gcl *gcl,
				      void *gclbuf)
{
	struct est_gc_config *gcc;
	struct ethtool_gc_entry *egce;
	struct est_gc_entry *sgce;
	int i, bank, ret;
	struct stmmac_priv *priv = netdev_priv(dev);

	if (gcl->cmd != ETHTOOL_GGCL ||
	    gcl->own >= EST_GCL_BANK_MAX)
		return -EINVAL;

	ret = stmmac_get_est_gcc(priv, priv->ioaddr, &gcc, 0);
	if (ret) {
		dev_err(priv->device, "fail to get EST GC config.\n");

		return ret;
	}

	bank = stmmac_get_est_bank(priv, priv->ioaddr, gcl->own);

	if (gcl->len != gcc->gcb[bank].gcrr.llr) {
		dev_err(priv->device, "GC length request invalid.\n");

		return -EINVAL;
	}

	egce = (struct ethtool_gc_entry *)gclbuf;
	sgce = gcc->gcb[bank].gcl;
	for (i = 0; i < gcl->len; i++) {
		egce->gates = sgce->gates;
		egce->ti_ns = sgce->ti_nsec;
		egce->opid = ETH_GATEOP_SET_GATE_STATES;

		egce++;
		sgce++;
	}

	return 0;
}

static int stmmac_ethtool_set_est_gcl(struct net_device *dev,
				      struct ethtool_gcl *gcl,
				      void *gclbuf)
{
	struct ethtool_gc_entry *egce;
	int i, bank;
	struct stmmac_priv *priv = netdev_priv(dev);
	int ret = -1;

	if (gcl->cmd != ETHTOOL_SGCL ||
	    gcl->own >= EST_GCL_BANK_MAX)
		return -EINVAL;

	bank = stmmac_get_est_bank(priv, priv->ioaddr, gcl->own);

	egce = (struct ethtool_gc_entry *)gclbuf;
	for (i = 0; i < gcl->len; i++) {
		struct est_gc_entry sgce;

		sgce.gates = egce->gates;
		sgce.ti_nsec = egce->ti_ns;

		ret = stmmac_set_est_gce(priv, priv->ioaddr, &sgce,
					 i, bank, 1);
		if (ret) {
			dev_err(priv->device,
				"fail to program GC entry(%d).\n", i);

			return ret;
		}
		egce++;
	}

	return stmmac_set_est_gcrr_llr(priv, priv->ioaddr, gcl->len,
				       bank, 1);
}

static int stmmac_ethtool_get_est_gce(struct net_device *dev,
				      struct ethtool_gce *gce)
{
	struct est_gc_config *gcc;
	struct est_gc_entry *sgce;
	int bank, ret;
	struct stmmac_priv *priv = netdev_priv(dev);

	if (gce->cmd != ETHTOOL_GGCE ||
	    gce->own >= EST_GCL_BANK_MAX)
		return -EINVAL;

	ret = stmmac_get_est_gcc(priv, priv->ioaddr, &gcc, 0);
	if (ret) {
		dev_err(priv->device, "fail to get EST GC config.\n");

		return ret;
	}
	bank = stmmac_get_est_bank(priv, priv->ioaddr, gce->own);

	if (gce->row >= gcc->gcb[bank].gcrr.llr) {
		dev_err(priv->device, "GC entry row >= length.\n");

		return -EINVAL;
	}

	sgce = gcc->gcb[bank].gcl + gce->row;
	gce->gce.gates = sgce->gates;
	gce->gce.ti_ns = sgce->ti_nsec;
	gce->gce.opid = ETH_GATEOP_SET_GATE_STATES;

	return 0;
}

static int stmmac_ethtool_set_est_gce(struct net_device *dev,
				      struct ethtool_gce *gce)
{
	struct est_gc_entry sgce;
	int bank, gcl_length, ret;
	struct stmmac_priv *priv = netdev_priv(dev);

	if (gce->cmd != ETHTOOL_SGCE ||
	    gce->own >= EST_GCL_BANK_MAX)
		return -EINVAL;

	bank = stmmac_get_est_bank(priv, priv->ioaddr, gce->own);

	ret = stmmac_get_est_gcrr_llr(priv, priv->ioaddr, &gcl_length,
				      bank, 1);
	if (ret) {
		dev_err(priv->device, "fail to get EST GC length.\n");

		return ret;
	}

	if (gce->row >= gcl_length) {
		dev_err(priv->device,
			"row exceeds GCL length set in SGCL.\n");

		return -EINVAL;
	}

	if (!gcl_length) {
		dev_err(priv->device,
			"GCL length is zero. Use SGCL first.\n");

		return -EINVAL;
	}

	sgce.gates = gce->gce.gates;
	sgce.ti_nsec = gce->gce.ti_ns;

	ret = stmmac_set_est_gce(priv, priv->ioaddr, &sgce, gce->row,
				 bank, 1);
	if (ret) {
		dev_err(priv->device,
			"fail to program GC entry(%d).\n", gce->row);

			return ret;
	}

	return 0;
}

static int stmmac_ethtool_get_est_info(struct net_device *dev,
				       struct ethtool_est_info *esti)
{
	struct est_gc_config *gcc;
	struct est_gcrr *egcrr;
	struct tsn_err_stat *erstat;
	struct stmmac_priv *priv = netdev_priv(dev);
	int bank, ret;

	if (esti->cmd != ETHTOOL_GESTINFO ||
	    esti->own >= EST_GCL_BANK_MAX)
		return -EINVAL;

	ret = stmmac_get_est_gcc(priv, priv->ioaddr, &gcc, 0);
	if (ret) {
		dev_err(priv->device, "fail to get EST GC config.\n");

		return ret;
	}
	bank = stmmac_get_est_bank(priv, priv->ioaddr, esti->own);
	egcrr = &gcc->gcb[bank].gcrr;

	esti->cycle_s = egcrr->cycle_sec;
	esti->cycle_ns = egcrr->cycle_nsec;
	esti->base_s = egcrr->base_sec;
	esti->base_ns = egcrr->base_nsec;
	esti->extension_s = 0;
	esti->extension_ns = egcrr->ter_nsec;

	ret = stmmac_get_est_err_stat(priv, &erstat);
	if (ret) {
		dev_err(priv->device, "fail to get EST error status.\n");

		return ret;
	}

	esti->cgce_n = erstat->cgce_n;
	esti->hlbs_q = erstat->hlbs_q;
	esti->btre_n = erstat->btre_n;
	esti->btre_max_n = esti->btre_max_n;
	esti->btrl = esti->btrl;
	memcpy(esti->hlbf_sz, erstat->hlbf_sz, sizeof(esti->hlbf_sz));

	return 0;
}

static int stmmac_ethtool_set_est_info(struct net_device *dev,
				       struct ethtool_est_info *esti)
{
	struct est_gcrr egcrr;
	struct stmmac_priv *priv = netdev_priv(dev);
	int bank;

	if (esti->cmd != ETHTOOL_SESTINFO ||
	    esti->own >= EST_GCL_BANK_MAX)
		return -EINVAL;

	if (esti->extension_s) {
		dev_err(priv->device, "extension in seconds not supported.\n");

		return -EINVAL;
	}
	bank = stmmac_get_est_bank(priv, priv->ioaddr, esti->own);

	egcrr.cycle_sec = esti->cycle_s;
	egcrr.cycle_nsec = esti->cycle_ns;
	egcrr.base_sec = esti->base_s;
	egcrr.base_nsec = esti->base_ns;
	egcrr.ter_nsec = esti->extension_ns;

	return stmmac_set_est_gcrr_times(priv, priv->ioaddr, &egcrr, bank, 1);
}

static const struct ethtool_ops stmmac_ethtool_ops = {
	.begin = stmmac_check_if_running,
	.get_drvinfo = stmmac_ethtool_getdrvinfo,
	.get_msglevel = stmmac_ethtool_getmsglevel,
	.set_msglevel = stmmac_ethtool_setmsglevel,
	.get_regs = stmmac_ethtool_gregs,
	.get_regs_len = stmmac_ethtool_get_regs_len,
	.get_link = ethtool_op_get_link,
	.nway_reset = phy_ethtool_nway_reset,
	.get_pauseparam = stmmac_get_pauseparam,
	.set_pauseparam = stmmac_set_pauseparam,
	.self_test = stmmac_diag_test,
	.get_ethtool_stats = stmmac_get_ethtool_stats,
	.get_strings = stmmac_get_strings,
	.get_wol = stmmac_get_wol,
	.set_wol = stmmac_set_wol,
	.get_eee = stmmac_ethtool_op_get_eee,
	.set_eee = stmmac_ethtool_op_set_eee,
	.get_sset_count	= stmmac_get_sset_count,
	.get_ts_info = stmmac_get_ts_info,
	.get_coalesce = stmmac_get_coalesce,
	.set_coalesce = stmmac_set_coalesce,
	.get_tunable = stmmac_get_tunable,
	.set_tunable = stmmac_set_tunable,
	.get_link_ksettings = stmmac_ethtool_get_link_ksettings,
	.set_link_ksettings = stmmac_ethtool_set_link_ksettings,
	.get_est_gcl_depth = stmmac_ethtool_get_est_gcl_depth,
	.get_est_gcl_length = stmmac_ethtool_get_est_gcl_length,
	.get_est_gcl = stmmac_ethtool_get_est_gcl,
	.set_est_gcl = stmmac_ethtool_set_est_gcl,
	.get_est_gce = stmmac_ethtool_get_est_gce,
	.set_est_gce = stmmac_ethtool_set_est_gce,
	.get_est_info = stmmac_ethtool_get_est_info,
	.set_est_info = stmmac_ethtool_set_est_info,
};

void stmmac_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &stmmac_ethtool_ops;
}
