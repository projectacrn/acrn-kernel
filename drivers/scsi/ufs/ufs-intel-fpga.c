/*
 * Universal Flash Storage Intel Host controller PCI driver
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

#include "unipro.h"

/* HC vendor specific register */
enum hc_vendor_spec_regs {
	REG_FPGA_CTRL		= 0xB0, /* 32-bit */
	REG_VENDOR_HCLKDIV	= 0xFC,
};

/* Clock Divider Values: Hex equivalent of frequency in MHz */
enum hc_clk_div_values {
	UFS_REG_HCLKDIV_DIV_20	= 0x14,
	UFS_REG_HCLKDIV_DIV_200	= 0xc8,
};

enum mphy_lanes_number {
	UFS_MPHY_1LANE = 1,
	UFS_MPHY_2LANE = 2,
};

/*********************************************/
/* REG_FPGA_CTRL - FPGA Control Register B0h */
/*********************************************/
/* bit offsets */
enum {
	OFFSET_REG_FPGA_CTRL_PHY_RESET		= 0,
	OFFSET_REG_FPGA_CTRL_SEL_TX_CLK		= 2,
	OFFSET_REG_FPGA_CTRL_CONFIG_UPDATE	= 3,
	OFFSET_REG_FPGA_CTRL_COSMIC_PREFIX	= 4,
	OFFSET_REG_FPGA_CTRL_RX_CLKCHG_OK	= 8,
	OFFSET_REG_FPGA_CTRL_RX_MPHY_MODE	= 10,
	OFFSET_REG_FPGA_CTRL_TX_CLKCHG_OK	= 12,
	OFFSET_REG_FPGA_CTRL_TX_MPHY_MODE	= 14,
	OFFSET_REG_FPGA_CTRL_DIRDY_CNT_STROBE	= 15
};
/* bit masks */
enum {
	MASK_REG_FPGA_CTRL_PHY_RESET		= UFS_MASK(0x03, 0),
	MASK_REG_FPGA_CTRL_SEL_TX_CLK		= BIT(2),
	MASK_REG_FPGA_CTRL_CONFIG_UPDATE	= BIT(3),
	MASK_REG_FPGA_CTRL_COSMIC_PREFIX	= UFS_MASK(0x0F, 4),
	MASK_REG_FPGA_CTRL_RX_CLKCHG_OK		= UFS_MASK(0x03, 8),
	MASK_REG_FPGA_CTRL_RX_MPHY_MODE		= BIT(10),
	MASK_REG_FPGA_CTRL_TX_CLKCHG_OK		= UFS_MASK(0x03, 12),
	MASK_REG_FPGA_CTRL_TX_MPHY_MODE		= BIT(14),
	MASK_REG_FPGA_CTRL_DIRDY_CNT_STROBE	= BIT(15),
	MASK_REG_FPGA_CTRL_ALL			= (~(u32)0)
};
/* 23:16 DIRDY counter: Number of DIRDY = 0 at start of burst; Default: 0x80 */

/* phy_reset_decode */
enum {
	PHY_RESET_OPERATIONAL		 = 0x00,
	PHY_RESET_TC_AND_PHY_RX_TX_RESET = 0x01,
	PHY_RESET_PHY_RX_TX_RESET        = 0x02,
	PHY_RESET_EXT_DEV_RESET          = 0x03
};
/* sel_tx_clk */
enum {
	SEL_TX_CLK_PWM_G1 = 0,
	SEL_TX_CLK_HS_G1  = 1,
};

/* Addr_Prefix - Internal Cosmic register access */
enum {
	COSMIC_PREFIX_TX1_MPHY = 0x0,
	COSMIC_PREFIX_TX2_MPHY = 0x1,
	COSMIC_PREFIX_RX1_MPHY = 0x2,
	COSMIC_PREFIX_RX2_MPHY = 0x3,
	COSMIC_PREFIX_CMN_CCPL = 0x8,
	COSMIC_PREFIX_TC_CCPL  = 0x9,
	COSMIC_PREFIX_TX1_CCPL = 0xc,
	COSMIC_PREFIX_TX2_CCPL = 0xd,
	COSMIC_PREFIX_RX1_CCPL = 0xe,
	COSMIC_PREFIX_RX2_CCPL = 0xf
};
/* ufs_rxclkchg_ok */
enum {
	RX_CLKCHG_NORMAL = 0x00,
	RX_CLKCHG_APPLY_CLK_CHANGE = 0x03
};

/* mphy_rx_pwmmode */
enum {
	RX_MPHY_MODE_PWM = 0,
	RX_MPHY_MODE_HS  = 1,
};
/* ufs_txclkchg_ok */
enum {
	TX_CLKCHG_NORMAL = 0x00,
	TX_CLKCHG_APPLY_CLK_CHANGE = 0x03
};
/* mphy_tx_pwmmode */
enum {
	TX_MPHY_MODE_PWM = 0,
	TX_MPHY_MODE_HS  = 1,
};

static void ufs_write_fpga_ctrl_reg(struct ufs_hba *hba, u32 val, u32 mask)
{
	ufshcd_rmwl(hba, mask, val, REG_FPGA_CTRL);
	usleep_range(900, 1100);
}

static void ufs_fpga_write_ctrl_reg(struct ufs_hba *hba, u32 val)
{
	ufs_write_fpga_ctrl_reg(hba, val, MASK_REG_FPGA_CTRL_ALL);
}

static void ufs_fpga_reset_phy(struct ufs_hba *hba, u8 reset_mode)
{
	ufs_write_fpga_ctrl_reg(hba,
				reset_mode << OFFSET_REG_FPGA_CTRL_PHY_RESET,
				MASK_REG_FPGA_CTRL_PHY_RESET);
}

static void ufs_fpga_configure_cosmic_prefix(struct ufs_hba *hba, u32 prefix)
{
	ufs_write_fpga_ctrl_reg(hba,
				prefix << OFFSET_REG_FPGA_CTRL_COSMIC_PREFIX,
				MASK_REG_FPGA_CTRL_COSMIC_PREFIX);
}

static void ufs_fpga_update_phy_config(struct ufs_hba *hba)
{
	ufs_write_fpga_ctrl_reg(hba, 1 << OFFSET_REG_FPGA_CTRL_CONFIG_UPDATE,
					MASK_REG_FPGA_CTRL_CONFIG_UPDATE);
	ufs_fpga_reset_phy(hba, PHY_RESET_OPERATIONAL);
	usleep_range(900, 1100);
}

static __maybe_unused void ufs_fpga_reset_device(struct ufs_hba *hba)
{
	dev_dbg(hba->dev, "Resetting device...");
	ufs_fpga_reset_phy(hba, PHY_RESET_EXT_DEV_RESET);
	usleep_range(900, 1100);
	/* Deassert device RSTn */
	ufs_fpga_reset_phy(hba, PHY_RESET_OPERATIONAL);
	usleep_range(900, 1100);
}

#define dme_attr_set(hba, offset, value) do { \
	u32 read_back = 0; \
	err = ufshcd_dme_set(hba, offset, value); \
	if (err) {\
		dev_err((hba)->dev, \
			"Setting DME ATTR 0x%04X to 0x%08X failed: %d\n", \
			offset, value, err); \
		goto err_out; \
	} \
	err = ufshcd_dme_get(hba, offset, &read_back); \
	if (err) {\
		dev_err((hba)->dev, \
			"Setting DME ATTR 0x%04X to 0x%08X" \
			" - failed read back: %d\n", \
			offset, value, err); \
		goto err_out; \
	} \
	if (read_back != value) \
		dev_err((hba)->dev, \
			"Setting DME ATTR 0x%04X to 0x%08X %s\n", \
			offset, value, "FAILED"); \
} while (0)

static int ufs_cadence_mphy_2lane_configuration(struct ufs_hba *hba)
{
	int err = 0;

	dev_dbg(hba->dev, "Configuring Cadence MPHY 2L Attributes...\n");

	/* Reset UFS device */
	//ufs_fpga_reset_device(hba);

	ufs_fpga_reset_phy(hba, PHY_RESET_TC_AND_PHY_RX_TX_RESET);
	usleep_range(900, 1100);
	ufs_fpga_reset_phy(hba, PHY_RESET_PHY_RX_TX_RESET);
	usleep_range(1900, 2100);

	/* Change address prefix mapped to TC_CCPL */
	// ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TC_CCPL)
	ufs_fpga_write_ctrl_reg(hba, 0x92);

	/*
	 * Write TC_MPHY_DATA_MODE (TC Reg) 0xA6
	 * Set MPHY in PWM Mode for LS operations
	 */
	dme_attr_set(hba, UIC_ARG_MIB(0x80A6), 0x5);

	// ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TC_CCPL)
	ufs_fpga_write_ctrl_reg(hba, 0x90);

	dme_attr_set(hba, UIC_ARG_MIB(0x80A4), 0x03);
	dme_attr_set(hba, UIC_ARG_MIB(0x80A1), 0x25);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_CMN_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8000), UFS_REG_HCLKDIV_DIV_20 + 0x40);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8000), UFS_REG_HCLKDIV_DIV_20);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX2_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8000), UFS_REG_HCLKDIV_DIV_20);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_RX1_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8000), UFS_REG_HCLKDIV_DIV_20);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_RX2_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8000), UFS_REG_HCLKDIV_DIV_20);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_CMN_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8001), 0x20);

	dme_attr_set(hba, UIC_ARG_MIB(0x8024), 0x60);
	dme_attr_set(hba, UIC_ARG_MIB(0x8025), 0x70);
	dme_attr_set(hba, UIC_ARG_MIB(0x8026), 0xC0);
	dme_attr_set(hba, UIC_ARG_MIB(0x8027), 0xE0);
	dme_attr_set(hba, UIC_ARG_MIB(0x802D), 0x80);
	dme_attr_set(hba, UIC_ARG_MIB(0x802E), 0xC0);
	dme_attr_set(hba, UIC_ARG_MIB(0x8028), 0x00);
	dme_attr_set(hba, UIC_ARG_MIB(0x8029), 0x00);
	dme_attr_set(hba, UIC_ARG_MIB(0x802F), 0x09);
	dme_attr_set(hba, UIC_ARG_MIB(0x802B), 0x04);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_RX1_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8002), 0x01);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_RX2_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8002), 0x01);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8026), 0x00);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x802C), 0x00);
	dme_attr_set(hba, UIC_ARG_MIB(0x8032), 0x00);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX2_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x802C), 0x00);
	dme_attr_set(hba, UIC_ARG_MIB(0x8032), 0x00);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x8028), 0x4A);
	dme_attr_set(hba, UIC_ARG_MIB(0x8029), 0x05);
	dme_attr_set(hba, UIC_ARG_MIB(0x802A), 0x0F);
	dme_attr_set(hba, UIC_ARG_MIB(0x8034), 0x05);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX2_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x8028), 0x4A);
	dme_attr_set(hba, UIC_ARG_MIB(0x8029), 0x05);
	dme_attr_set(hba, UIC_ARG_MIB(0x802A), 0x0F);
	dme_attr_set(hba, UIC_ARG_MIB(0x8034), 0x05);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x802d), 0x04);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX2_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x802d), 0x04);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_RX1_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x80A1), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x80A4), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x80A2), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x80A3), 0x01);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_RX2_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x80A1), 0x01);

	dme_attr_set(hba, UIC_ARG_MIB(0x80A4), 0x01);

	dme_attr_set(hba, UIC_ARG_MIB(0x80A2), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x80A3), 0x01);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8021), 0x06);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x8021), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x8024), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x8022), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x8023), 0x01);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX2_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x8021), 0x06);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX2_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x8021), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x8024), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x8022), 0x01);
	dme_attr_set(hba, UIC_ARG_MIB(0x8023), 0x01);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_CMN_CCPL);
	/* TM_CMN_SEL_SPEED - LS Mode */
	dme_attr_set(hba, UIC_ARG_MIB(0x803A), 0x01);

	/* TM_CMN_RATE - Rate A */
	dme_attr_set(hba, UIC_ARG_MIB(0x803B), 0x01);

	/* TM_CMN_GEAR_HS - G1 */
	dme_attr_set(hba, UIC_ARG_MIB(0x803C), 0x01);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x8037), 0x02);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX2_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x8037), 0x02);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x800C), 0x80);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX2_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x800C), 0x80);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_CMN_CCPL);

	dme_attr_set(hba, UIC_ARG_MIB(0x803E), 0x01);

	ufs_fpga_update_phy_config(hba);
	usleep_range(900, 1100);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_CMN_CCPL);
	dme_attr_set(hba, UIC_ARG_MIB(0x803D), 0x01);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX1_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x802B), 0x00);

	ufs_fpga_configure_cosmic_prefix(hba, COSMIC_PREFIX_TX2_MPHY);
	dme_attr_set(hba, UIC_ARG_MIB(0x802B), 0x00);

	ufs_fpga_update_phy_config(hba);

	dev_dbg(hba->dev, "Configuration of Cadence MPHY succeeded\n");

	return 0;

err_out:
	return err;
}

static int ufs_intel_cadence_mphy_configuration(struct ufs_hba *hba,
								u8 lanes_num)
{
	int err = 0;

	if (lanes_num == UFS_MPHY_1LANE) {
		dev_warn(hba->dev, "1lane configuration is not supported\n");
		err = -ENOTSUPP;
	} else if (lanes_num == UFS_MPHY_2LANE) {
		err = ufs_cadence_mphy_2lane_configuration(hba);
	} else {
		dev_err(hba->dev, "Invalid number of lanes selected: %d\n",
			lanes_num);
		err = -EINVAL;
	}

	/*
	 * Default Cosmic MPHY RX_HS_G1_SYNC_LENGTH_Capability = 0x2,
	 * increase the value to 0x4a for both lanes.
	 * Required FPGA implementation to write the Cosmic
	 * MPHY Capability Attributes.
	 */
	dme_attr_set(hba, UIC_ARG_MIB_SEL(RX_HS_G1_SYNC_LENGTH, 4), 0x4a);
	dme_attr_set(hba, UIC_ARG_MIB_SEL(RX_HS_G1_SYNC_LENGTH, 5), 0x4a);
	usleep_range(1000, 2000);

err_out:
	return err;
}

/*
 * Clear UIC errors and UFS statuses.
 */
static  __maybe_unused void ufs_intel_fpga_clear_errors(struct ufs_hba *hba)
{
	dev_dbg(hba->dev, "Clearing registers from possible errors\n");
	ufshcd_writel(hba, 0xffffffff, REG_INTERRUPT_STATUS);
	ufshcd_readl(hba, REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER);
	ufshcd_readl(hba, REG_UIC_ERROR_CODE_DATA_LINK_LAYER);
	ufshcd_readl(hba, REG_UIC_ERROR_CODE_NETWORK_LAYER);
	ufshcd_readl(hba, REG_UIC_ERROR_CODE_TRANSPORT_LAYER);
	ufshcd_readl(hba, REG_UIC_ERROR_CODE_DME);
}

/**
 * This function sets the clk divider value. This value is needed to
 * provide 1 microsecond tick to unipro layer.
 */
static int ufs_intel_set_clk_div(struct ufs_hba *hba, u32 div_val)
{
	u32 hclkdiv;
	int err = 0;

	hclkdiv = ufshcd_readl(hba, REG_VENDOR_HCLKDIV);
	if (hclkdiv != div_val) {
		dev_warn(hba->dev,
			"HC HCLKDIV is 0x%08X but expected 0x%08X - correcting\n",
			hclkdiv, div_val);
		ufshcd_writel(hba, div_val, REG_VENDOR_HCLKDIV);
		usleep_range(1000, 2000);
		hclkdiv = ufshcd_readl(hba, REG_VENDOR_HCLKDIV);
		if (hclkdiv != div_val) {
			dev_err(hba->dev, "HCLKDIV configuration failed!\n");
			err = -EINVAL;
		}
	}

	return err;
}

static int ufs_intel_fpga_phy_initialization(struct ufs_hba *hba)
{
	return ufs_intel_cadence_mphy_configuration(hba, UFS_MPHY_2LANE);
}

static int ufs_intel_fpga_hce_enable_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status notify)
{
	int err = 0;

	switch (notify) {
	case PRE_CHANGE:
		err = ufs_intel_set_clk_div(hba, UFS_REG_HCLKDIV_DIV_20);
		break;
	case POST_CHANGE:
		break;
	default:
		break;
	}

	return err;
}

static int ufs_intel_fpga_disable_lcc(struct ufs_hba *hba)
{
	u32 attr = UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE);
	u32 lcc_enable = 0;
	int err;

	err = ufshcd_dme_get(hba, attr, &lcc_enable);
	if (!err && lcc_enable)
		err = ufshcd_dme_set(hba, attr, 0);

	return err;
}

static int ufs_intel_fpga_link_startup_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status notify)
{
	int err = 0;

	switch (notify) {
	case PRE_CHANGE:
		err = ufs_intel_fpga_disable_lcc(hba);
		break;
	case POST_CHANGE:
		break;
	default:
		break;
	}

	return err;
}

static void ufshcd_set_local_dl_timer_values(struct ufs_hba *hba)
{
	/* Set Local device timeouts */
	ufshcd_dme_set(hba,
		UIC_ARG_MIB(DME_LOCALFC0PROTECTIONTIMEOUTVAL), 0x0fff);
	ufshcd_dme_set(hba,
		UIC_ARG_MIB(DME_LOCALTC0REPLAYTIMEOUTVAL), 0xffff);
	ufshcd_dme_set(hba,
		UIC_ARG_MIB(DME_LOCALAFC0REQTIMEOUTVAL), 0x7fff);
	ufshcd_dme_set(hba,
		UIC_ARG_MIB(DME_LOCALFC1PROTECTIONTIMEOUTVAL), 0x0fff);
	ufshcd_dme_set(hba,
		UIC_ARG_MIB(DME_LOCALTC1REPLAYTIMEOUTVAL), 0xffff);
	ufshcd_dme_set(hba,
		UIC_ARG_MIB(DME_LOCALAFC1REQTIMEOUTVAL), 0x7fff);
}

static void ufshcd_set_peer_dl_timer_values(struct ufs_hba *hba)
{
	/* Set Peer device timeouts */
	/* DL_FC0ProtectionTimeOutVal */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 0x1fff);
	/* DL_TC0ReplayTimeOutVal */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 0xffff);
	/* DL_AFC0ReqTimeOutVal */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 0x7fff);
	/* DL_FC1ProtectionTimeOutVal */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA3), 0x1fff);
	/* DL_TC1ReplayTimeOutVal */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA4), 0xffff);
	/* DL_AFC1ReqTimeOutVal */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA5), 0x7fff);
}

static int ufs_intel_fpga_pwr_change_notify(struct ufs_hba *hba,
				enum ufs_notify_change_status notify,
				struct ufs_pa_layer_attr *desired_pwr_info,
				struct ufs_pa_layer_attr *final_pwr_info)
{
	switch (notify) {
	case PRE_CHANGE:
		if (!desired_pwr_info || !final_pwr_info)
			break;
		memcpy(final_pwr_info, desired_pwr_info,
				sizeof(struct ufs_pa_layer_attr));

		if (final_pwr_info->gear_rx > UFS_HS_G1 ||
		    final_pwr_info->gear_tx > UFS_HS_G1) {
			dev_warn(hba->dev,
				"WARNING: Pre-silicon UFS speed is limited to first gear only. Correcting\n");
			final_pwr_info->gear_rx = UFS_HS_G1;
			final_pwr_info->gear_tx = UFS_HS_G1;
		}

		if ((final_pwr_info->pwr_tx == FASTAUTO_MODE ||
			final_pwr_info->pwr_tx == FAST_MODE ||
			final_pwr_info->pwr_rx == FASTAUTO_MODE ||
			final_pwr_info->pwr_rx == FAST_MODE) &&
			final_pwr_info->hs_rate == PA_HS_MODE_B) {
			dev_warn(hba->dev,
				"WARNING: Pre-silicon HS mode supports only RATE A frequency. Correcting\n");
			final_pwr_info->hs_rate = PA_HS_MODE_A;
		}

		/*
		 * UFS Data Link Layer timer values need to be set
		 * for the local and the peer device before power mode change.
		 * These values become effective after a successful power
		 * mode change request.
		 */
		ufshcd_set_local_dl_timer_values(hba);
		ufshcd_set_peer_dl_timer_values(hba);

		if (final_pwr_info->pwr_tx == FAST_MODE
				|| final_pwr_info->pwr_tx == FASTAUTO_MODE)
			/* Apply FPGA specific settings */
			ufs_fpga_write_ctrl_reg(hba, 0x8000);
		usleep_range(1000, 2000);
		break;
	case POST_CHANGE:
		break;
	default:
		break;
	}

	return 0;
}

static int ufs_intel_fpga_hce_init(struct ufs_hba *hba)
{
	u32 hce;

	hce = ufshcd_readl(hba, REG_CONTROLLER_ENABLE);

	if (hce & CONTROLLER_ENABLE) {
		ufshcd_writel(hba, 0, REG_CONTROLLER_ENABLE);
		usleep_range(1000, 2000);
	}

	return 0;
}

static struct ufs_hba_variant_ops ufs_intel_icl_fpga_hba_vops = {
	.name			= "intel-fpga-pci",
	.init 			= ufs_intel_fpga_hce_init,
	.hce_enable_notify 	= ufs_intel_fpga_hce_enable_notify,
	.link_startup_notify 	= ufs_intel_fpga_link_startup_notify,
	.pwr_change_notify 	= ufs_intel_fpga_pwr_change_notify,
	.phy_initialization	= ufs_intel_fpga_phy_initialization
};

static bool ufs_intel_fpga_detected(const struct pci_device_id *id)
{
	static const struct pci_device_id ufs_icl_bridge_pci_tbl[] = {
		{ PCI_VENDOR_ID_INTEL, 0xbabe, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
		{ }
	};

	return id->vendor == PCI_VENDOR_ID_INTEL && id->device == 0x34FA &&
	       pci_dev_present(ufs_icl_bridge_pci_tbl);
}

MODULE_SOFTDEP("pre: ufs_icl_bridge");
