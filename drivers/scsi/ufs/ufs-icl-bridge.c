/*
 * Bridge driver for ICL UFS controller for FPGA development.
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

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>

#define UFSICLBRIDGE "ufs-icl-bridge"


#define SCS_PORT_ID 0x50

#define IOSF_GEN_REGRW1	0x600
#define IOSF_GEN_REGRW1_MPHY_PWR_EN (1<<22)
#define IOSF_GEN_REGRW1_UFS_EN      (1<<23)
#define IOSF_GEN_REGRW1_DUAL_LANE   (1<<24)

#define SB_PRIVATE_READ_OPCODE  0x06
#define SB_PRIVATE_WRITE_OPCODE 0x07
#define SB_BYTE_ENABLE 0x0F

#define SB_REG_EXT 0xD8
#define SB_REG_MCR 0xD0
#define SB_REG_MDR 0xD4

static u32 readl_cfg(struct pci_dev *pdev, u8 offset)
{
	u32 v;

	pci_read_config_dword(pdev, offset, &v);
	return v;
}

static void writel_cfg(struct pci_dev *pdev, u8 offset, u32 val)
{
	pci_write_config_dword(pdev, offset, val);
}

static void ufs_icl_bridge_write_sb(struct pci_dev *pdev, u32 offset, u32 val)
{
	u32 regval;

	dev_dbg(&pdev->dev, "SB Write val: 0x%08x at offs: 0x%08x\n",
			val, offset);

	regval = offset & 0xffffff00;
	writel_cfg(pdev, SB_REG_EXT, regval);

	writel_cfg(pdev, SB_REG_MDR, val);

	regval = (SB_PRIVATE_WRITE_OPCODE << 24) | (SCS_PORT_ID << 16) |
		((offset & 0xff) << 8) | (SB_BYTE_ENABLE << 4);
	writel_cfg(pdev, SB_REG_MCR, regval);
}

static void ufs_icl_bridge_read_sb(struct pci_dev *pdev, u32 offset, u32 *val)
{
	u32 regval;
	*val = 0;

	dev_dbg(&pdev->dev, "SB Read value from offset: 0x%08x\n", offset);

	regval = offset & 0xffffff00;
	writel_cfg(pdev, SB_REG_EXT, regval);

	regval = (SB_PRIVATE_READ_OPCODE << 24) | (SCS_PORT_ID << 16) |
		((offset & 0xff) << 8) | (SB_BYTE_ENABLE << 4);
	writel_cfg(pdev, SB_REG_MCR, regval);

	regval = readl_cfg(pdev, SB_REG_MDR);
	*val = regval;

	writel_cfg(pdev, SB_REG_MDR, 0x00);
}

void ufs_icl_bridge_ufs_enable(struct pci_dev *pdev)
{
	u32 regval = 0;

	dev_info(&pdev->dev, "Enable UFS and dual lane\n");

	ufs_icl_bridge_read_sb(pdev, IOSF_GEN_REGRW1, &regval);
	dev_dbg(&pdev->dev, "0x%08x <= GENREGW1\n", regval);

	regval |= IOSF_GEN_REGRW1_UFS_EN | IOSF_GEN_REGRW1_DUAL_LANE;
	ufs_icl_bridge_write_sb(pdev, IOSF_GEN_REGRW1, regval);
	udelay(100);

	ufs_icl_bridge_read_sb(pdev, IOSF_GEN_REGRW1, &regval);
	dev_dbg(&pdev->dev, "0x%08x <= GENREGW1\n", regval);
	udelay(100);
}

static void ufs_icl_bridge_ufs_disable(struct pci_dev *pdev)
{
	u32 regval = 0;

	dev_info(&pdev->dev, "Disable UFS and dual lane\n");

	ufs_icl_bridge_read_sb(pdev, IOSF_GEN_REGRW1, &regval);
	dev_dbg(&pdev->dev, "0x%08x <= GENREGW1\n", regval);

	regval &= ~(IOSF_GEN_REGRW1_UFS_EN | IOSF_GEN_REGRW1_DUAL_LANE);
	ufs_icl_bridge_write_sb(pdev, IOSF_GEN_REGRW1, regval);
}

static int ufs_icl_bridge_pci_probe(struct pci_dev *pdev,
						const struct pci_device_id *id)
{
	int err = 0;
	err = pcim_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "pcim_enable_device failed\n");
		return err;
	}

	pci_set_master(pdev);

	err = pcim_iomap_regions(pdev, 1 << 0, UFSICLBRIDGE);
	if (err < 0) {
		dev_err(&pdev->dev, "request and iomap failed\n");
		return err;
	}

	ufs_icl_bridge_ufs_enable(pdev);
	dev_dbg(&pdev->dev, "SBC driver loaded\n");
	return 0;
}

static void ufs_icl_bridge_pci_remove(struct pci_dev *pdev)
{
	ufs_icl_bridge_ufs_disable(pdev);
	dev_dbg(&pdev->dev, "SBC driver removed\n");
}

static const struct pci_device_id ufs_icl_bridge_pci_tbl[] = {
	{ PCI_VENDOR_ID_INTEL, 0xbabe, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ }
};
MODULE_DEVICE_TABLE(pci, ufs_icl_bridge_pci_tbl);

static struct pci_driver ufs_icl_bridge_pci_driver = {
	.name = UFSICLBRIDGE,
	.id_table = ufs_icl_bridge_pci_tbl,
	.probe = ufs_icl_bridge_pci_probe,
	.remove = ufs_icl_bridge_pci_remove,
};

module_pci_driver(ufs_icl_bridge_pci_driver);

MODULE_AUTHOR("Szymon Mielczarek <szymonx.mielczarek@intel.com>");
MODULE_LICENSE("GPL and additional rights");
