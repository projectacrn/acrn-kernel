/*
 * Intel Core SoC Power Management Controller Driver
 *
 * Copyright (c) 2016, Intel Corporation.
 * All Rights Reserved.
 *
 * Authors: Rajneesh Bhardwaj <rajneesh.bhardwaj@intel.com>
 *          Vishwanath Somayaji <vishwanath.somayaji@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <linux/acpi.h>

#include <asm/cpu_device_id.h>
#include <asm/intel-family.h>
#include <asm/pmc_core.h>

#include "intel_pmc_core.h"

static struct pmc_dev pmc;

static const struct pmc_bit_map spt_pll_map[] = {
	{"MIPI PLL",			SPT_PMC_BIT_MPHY_CMN_LANE0},
	{"GEN2 USB2PCIE2 PLL",		SPT_PMC_BIT_MPHY_CMN_LANE1, 0x034C, 20},
	{"DMIPCIE3 PLL",		SPT_PMC_BIT_MPHY_CMN_LANE2},
	{"SATA PLL",			SPT_PMC_BIT_MPHY_CMN_LANE3},
	{},
};

static const struct pmc_bit_map spt_mphy_map[] = {
	{"MPHY CORE LANE 0",           SPT_PMC_BIT_MPHY_LANE0, 0x034C, 21},
	{"MPHY CORE LANE 1",           SPT_PMC_BIT_MPHY_LANE1, 0x034C, 21},
	{"MPHY CORE LANE 2",           SPT_PMC_BIT_MPHY_LANE2, 0x034C, 21},
	{"MPHY CORE LANE 3",           SPT_PMC_BIT_MPHY_LANE3, 0x034C, 21},
	{"MPHY CORE LANE 4",           SPT_PMC_BIT_MPHY_LANE4, 0x034C, 21},
	{"MPHY CORE LANE 5",           SPT_PMC_BIT_MPHY_LANE5, 0x034C, 21},
	{"MPHY CORE LANE 6",           SPT_PMC_BIT_MPHY_LANE6, 0x034C, 21},
	{"MPHY CORE LANE 7",           SPT_PMC_BIT_MPHY_LANE7, 0x034C, 21},
	{"MPHY CORE LANE 8",           SPT_PMC_BIT_MPHY_LANE8, 0x034C, 21},
	{"MPHY CORE LANE 9",           SPT_PMC_BIT_MPHY_LANE9, 0x034C, 21},
	{"MPHY CORE LANE 10",          SPT_PMC_BIT_MPHY_LANE10, 0x034C, 21},
	{"MPHY CORE LANE 11",          SPT_PMC_BIT_MPHY_LANE11, 0x034C, 21},
	{"MPHY CORE LANE 12",          SPT_PMC_BIT_MPHY_LANE12, 0x034C, 21},
	{"MPHY CORE LANE 13",          SPT_PMC_BIT_MPHY_LANE13, 0x034C, 21},
	{"MPHY CORE LANE 14",          SPT_PMC_BIT_MPHY_LANE14, 0x034C, 21},
	{"MPHY CORE LANE 15",          SPT_PMC_BIT_MPHY_LANE15, 0x034C, 21},
	{},
};

static const struct pmc_bit_map spt_pfear_map[] = {
	{"PMC",				SPT_PMC_BIT_PMC},
	{"OPI-DMI",			SPT_PMC_BIT_OPI},
	{"SPI / eSPI",			SPT_PMC_BIT_SPI},
	{"XHCI",			SPT_PMC_BIT_XHCI, 0x031C, 28},
	{"SPA",				SPT_PMC_BIT_SPA},
	{"SPB",				SPT_PMC_BIT_SPB},
	{"SPC",				SPT_PMC_BIT_SPC},
	{"GBE",				SPT_PMC_BIT_GBE, 0x031C, 30},
	{"SATA",			SPT_PMC_BIT_SATA, 0x034C, 30},
	{"HDA-PGD0",			SPT_PMC_BIT_HDA_PGD0, 0x031C, 29},
	{"HDA-PGD1",			SPT_PMC_BIT_HDA_PGD1, 0x031C, 29},
	{"HDA-PGD2",			SPT_PMC_BIT_HDA_PGD2, 0x031C, 29},
	{"HDA-PGD3",			SPT_PMC_BIT_HDA_PGD3, 0x031C, 29},
	{"RSVD",			SPT_PMC_BIT_RSVD_0B},
	{"LPSS",			SPT_PMC_BIT_LPSS, 0x031C, 27},
	{"LPC",				SPT_PMC_BIT_LPC},
	{"SMB",				SPT_PMC_BIT_SMB},
	{"ISH",				SPT_PMC_BIT_ISH},
	{"P2SB",			SPT_PMC_BIT_P2SB},
	{"DFX",				SPT_PMC_BIT_DFX},
	{"SCC",				SPT_PMC_BIT_SCC},
	{"RSVD",			SPT_PMC_BIT_RSVD_0C},
	{"FUSE",			SPT_PMC_BIT_FUSE},
	{"CAMERA",			SPT_PMC_BIT_CAMREA, 0x031C, 12},
	{"RSVD",			SPT_PMC_BIT_RSVD_0D},
	{"USB3-OTG",			SPT_PMC_BIT_USB3_OTG, 0x034C, 29},
	{"EXI",				SPT_PMC_BIT_EXI},
	{"CSE",				SPT_PMC_BIT_CSE},
	{"CSME_KVM",			SPT_PMC_BIT_CSME_KVM, 0x031C, 31},
	{"CSME_PMT",			SPT_PMC_BIT_CSME_PMT, 0x031C, 31},
	{"CSME_CLINK",			SPT_PMC_BIT_CSME_CLINK, 0x031C, 31},
	{"CSME_PTIO",			SPT_PMC_BIT_CSME_PTIO, 0x031C, 31},
	{"CSME_USBR",			SPT_PMC_BIT_CSME_USBR, 0x031C, 31},
	{"CSME_SUSRAM",			SPT_PMC_BIT_CSME_SUSRAM, 0x031C, 31},
	{"CSME_SMT",			SPT_PMC_BIT_CSME_SMT, 0x031C, 31},
	{"RSVD",			SPT_PMC_BIT_RSVD_1A},
	{"CSME_SMS2",			SPT_PMC_BIT_CSME_SMS2, 0x031C, 31},
	{"CSME_SMS1",			SPT_PMC_BIT_CSME_SMS1, 0x031C, 31},
	{"CSME_RTC",			SPT_PMC_BIT_CSME_RTC, 0x031C, 31},
	{"CSME_PSF",			SPT_PMC_BIT_CSME_PSF, 0x031C, 31},
	{},
};

static const struct pmc_reg_map spt_reg_map = {
	.pfear_sts = spt_pfear_map,
	.mphy_sts = spt_mphy_map,
	.pll_sts = spt_pll_map,
	.slp_s0_offset = SPT_PMC_SLP_S0_RES_COUNTER_OFFSET,
	.ltr_ignore_offset = SPT_PMC_LTR_IGNORE_OFFSET,
	.regmap_length = SPT_PMC_MMIO_REG_LEN,
	.ppfear0_offset = SPT_PMC_XRAM_PPFEAR0A,
	.ppfear_buckets = SPT_PPFEAR_NUM_ENTRIES,
	.pm_cfg_offset = SPT_PMC_PM_CFG_OFFSET,
	.pm_read_disable_bit = SPT_PMC_READ_DISABLE_BIT,
};

static const struct pmc_bit_map cnp_pfear_map[] = {
	{"PMC",			BIT(0)},
	{"OPI-DMI",		BIT(1)},
	{"SPI/eSPI",		BIT(2)},
	{"XHCI",		BIT(3), 0x1B1C, 28},
	{"SPA",			BIT(4)},
	{"SPB",			BIT(5)},
	{"SPC",			BIT(6)},
	{"GBE",			BIT(7), 0x1B1C, 30},

	{"SATA",		BIT(0), 0x1B4C, 30},
	{"HDA_PGD0",		BIT(1)},
	{"HDA_PGD1",		BIT(2)},
	{"HDA_PGD2",		BIT(3)},
	{"HDA_PGD3",		BIT(4)},
	{"SPD",			BIT(5)},
	{"LPSS",		BIT(6), 0x1B1C, 27},
	{"LPC",			BIT(7)},

	{"SMB",			BIT(0)},
	{"ISH",			BIT(1)},
	{"P2SB",		BIT(2)},
	{"NPK_VNN",		BIT(3)},
	{"SDX",			BIT(4),0x1E4C, 2},
	{"SPE",			BIT(5)},
	{"Fuse",		BIT(6)},
	{"Res_23",		BIT(7)},

	{"CSME_FSC",		BIT(0), 0x1B1C, 31},
	{"USB3_OTG",		BIT(1)},
	{"EXI",			BIT(2)},
	{"CSE",			BIT(3)},
	{"csme_kvm",		BIT(4), 0x1B1C, 31},
	{"csme_pmt",		BIT(5), 0x1B1C, 31},
	{"csme_clink",		BIT(6), 0x1B1C, 31},
	{"csme_ptio",		BIT(7), 0x1B1C, 31},

	{"csme_usbr",		BIT(0), 0x1B1C, 31},
	{"csme_susram",		BIT(1), 0x1B1C, 31},
	{"csme_smt1",		BIT(2), 0x1B1C, 31},
	{"CSME_SMT4",		BIT(3), 0x1B1C, 31},
	{"csme_sms2",		BIT(4), 0x1B1C, 31},
	{"csme_sms1",		BIT(5), 0x1B1C, 31},
	{"csme_rtc",		BIT(6), 0x1B1C, 31},
	{"csme_psf",		BIT(7), 0x1B1C, 31},

	{"SBR0",		BIT(0)},
	{"SBR1",		BIT(1)},
	{"SBR2",		BIT(2)},
	{"SBR3",		BIT(3)},
	{"SBR4",		BIT(4)},
	{"SBR5",		BIT(5)},
	{"CSME_PECI",		BIT(6), 0x1B1C, 31},
	{"PSF1",		BIT(7)},

	{"PSF2",		BIT(0)},
	{"PSF3",		BIT(1)},
	{"PSF4",		BIT(2)},
	{"CNVI",		BIT(3)},
	{"UFS0",		BIT(4), 0x1E4C, 3},
	{"EMMC",		BIT(5), 0x1E4C, 1},
	{"Res_6",		BIT(6)},
	{"SBR6",		BIT(7)},

	{"SBR7",		BIT(0)},
	{"NPK_AON",		BIT(1)},
	{"HDA_PGD4",		BIT(2)},
	{"HDA_PGD5",		BIT(3)},
	{"HDA_PGD6",		BIT(4)},
	{}
};

static const struct pmc_reg_map cnp_reg_map = {
	.pfear_sts = cnp_pfear_map,
	.slp_s0_offset = CNP_PMC_SLP_S0_RES_COUNTER_OFFSET,
	.ltr_ignore_offset = CNP_PMC_LTR_IGNORE_OFFSET,
	.regmap_length = CNP_PMC_MMIO_REG_LEN,
	.ppfear0_offset = CNP_PMC_HOST_PPFEAR0A,
	.ppfear_buckets = CNP_PPFEAR_NUM_ENTRIES,
	.pm_cfg_offset = CNP_PMC_PM_CFG_OFFSET,
	.pm_read_disable_bit = CNP_PMC_READ_DISABLE_BIT,
};

static const struct pci_device_id pmc_pci_ids[] = {
	{ PCI_VDEVICE(INTEL, SPT_PMC_PCI_DEVICE_ID),
					(kernel_ulong_t)&spt_reg_map },
	{ 0, },
};

static inline u8 pmc_core_reg_read_byte(struct pmc_dev *pmcdev, int offset)
{
	return readb(pmcdev->regbase + offset);
}

static inline u32 pmc_core_reg_read(struct pmc_dev *pmcdev, int reg_offset)
{
	return readl(pmcdev->regbase + reg_offset);
}

static inline void pmc_core_reg_write(struct pmc_dev *pmcdev, int
							reg_offset, u32 val)
{
	writel(val, pmcdev->regbase + reg_offset);
}

static inline u32 pmc_core_adjust_slp_s0_step(u32 value)
{
	return value * SPT_PMC_SLP_S0_RES_COUNTER_STEP;
}

/**
 * intel_pmc_slp_s0_counter_read() - Read SLP_S0 residency.
 * @data: Out param that contains current SLP_S0 count.
 *
 * This API currently supports Intel Skylake SoC and Sunrise
 * Point Platform Controller Hub. Future platform support
 * should be added for platforms that support low power modes
 * beyond Package C10 state.
 *
 * SLP_S0_RESIDENCY counter counts in 100 us granularity per
 * step hence function populates the multiplied value in out
 * parameter @data.
 *
 * Return: an error code or 0 on success.
 */
int intel_pmc_slp_s0_counter_read(u32 *data)
{
	struct pmc_dev *pmcdev = &pmc;
	const struct pmc_reg_map *map = pmcdev->map;
	u32 value;

	if (!pmcdev->has_slp_s0_res)
		return -EACCES;

	value = pmc_core_reg_read(pmcdev, map->slp_s0_offset);
	*data = pmc_core_adjust_slp_s0_step(value);

	return 0;
}
EXPORT_SYMBOL_GPL(intel_pmc_slp_s0_counter_read);

static int pmc_core_dev_state_get(void *data, u64 *val)
{
	struct pmc_dev *pmcdev = data;
	const struct pmc_reg_map *map = pmcdev->map;
	u32 value;

	value = pmc_core_reg_read(pmcdev, map->slp_s0_offset);
	*val = pmc_core_adjust_slp_s0_step(value);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(pmc_core_dev_state, pmc_core_dev_state_get, NULL, "%llu\n");

static int pmc_core_check_read_lock_bit(void)
{
	struct pmc_dev *pmcdev = &pmc;
	u32 value;

	value = pmc_core_reg_read(pmcdev, pmcdev->map->pm_cfg_offset);
	return value & BIT(pmcdev->map->pm_read_disable_bit);
}

#if IS_ENABLED(CONFIG_DEBUG_FS)

static bool pmc_debug_messages_on;

static void pmc_core_printf(struct seq_file *s, const char *fmt_str, ...)
{
	struct va_format vaf;
	va_list argptr;

	va_start(argptr, fmt_str);

	vaf.fmt = fmt_str;
	vaf.va = &argptr;

	if (s)
		seq_printf(s, "%pV", &vaf);
	else
		pr_info("%pV", &vaf);

	va_end(argptr);
}

static const char *pmc_core_required_string(struct pmc_dev *pmcdev, const struct pmc_bit_map *map)
{
	static const char *required = "Constraint: Required";
	static const char *not_required = "Constraint: Not Required";
	static const char *not_app = " ";


	if (map->required_offset && map->required_bit) {
		u32 val;

		val = pmc_core_reg_read(pmcdev, map->required_offset);
		if (val & BIT(map->required_bit))
			return not_required;
		else
			return required;
	}

	return not_app;
}

static void pmc_core_display_map(struct seq_file *s, int index,
				 u8 pf_reg, const struct pmc_bit_map *pf_map)
{
	struct pmc_dev *pmcdev = s ? s->private : &pmc;

	pmc_core_printf(s, "PCH IP: %-2d - %-32s\tState: %s %s\n",
			index, pf_map[index].name,
			pf_map[index].bit_mask & pf_reg ? "Off" : "On",
			pmc_core_required_string(pmcdev, &pf_map[index]));
}

static int pmc_core_ppfear_sts_show(struct seq_file *s, void *unused)
{
	struct pmc_dev *pmcdev = s ? s->private : &pmc;
	const struct pmc_bit_map *map = pmcdev->map->pfear_sts;
	u8 pf_regs[PPFEAR_MAX_NUM_ENTRIES];
	int index, iter;

	iter = pmcdev->map->ppfear0_offset;

	for (index = 0; index < pmcdev->map->ppfear_buckets &&
	     index < PPFEAR_MAX_NUM_ENTRIES; index++, iter++)
		pf_regs[index] = pmc_core_reg_read_byte(pmcdev, iter);

	for (index = 0; map[index].name; index++)
		pmc_core_display_map(s, index, pf_regs[index / 8], map);

	return 0;
}

static int pmc_core_ppfear_sts_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_core_ppfear_sts_show, inode->i_private);
}

static const struct file_operations pmc_core_ppfear_ops = {
	.open           = pmc_core_ppfear_sts_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

/* This function should return link status, 0 means ready */
static int pmc_core_mtpmc_link_status(void)
{
	struct pmc_dev *pmcdev = &pmc;
	u32 value;

	value = pmc_core_reg_read(pmcdev, SPT_PMC_PM_STS_OFFSET);
	return value & BIT(SPT_PMC_MSG_FULL_STS_BIT);
}

static int pmc_core_send_msg(u32 *addr_xram)
{
	struct pmc_dev *pmcdev = &pmc;
	u32 dest;
	int timeout;

	for (timeout = NUM_RETRIES; timeout > 0; timeout--) {
		if (pmc_core_mtpmc_link_status() == 0)
			break;
		msleep(5);
	}

	if (timeout <= 0 && pmc_core_mtpmc_link_status())
		return -EBUSY;

	dest = (*addr_xram & MTPMC_MASK) | (1U << 1);
	pmc_core_reg_write(pmcdev, SPT_PMC_MTPMC_OFFSET, dest);
	return 0;
}

static int pmc_core_mphy_pg_sts_show(struct seq_file *s, void *unused)
{
	struct pmc_dev *pmcdev = s ? s->private : &pmc;
	const struct pmc_bit_map *map = pmcdev->map->mphy_sts;
	u32 mphy_core_reg_low, mphy_core_reg_high;
	u32 val_low, val_high;
	int index, err = 0;

	if (!pmcdev->map->mphy_sts)
		return 0;

	if (pmcdev->pmc_xram_read_bit) {
		pmc_core_printf(s, "Access denied: please disable PMC_READ_DISABLE setting in BIOS.\n");
		return 0;
	}

	mphy_core_reg_low  = (SPT_PMC_MPHY_CORE_STS_0 << 16);
	mphy_core_reg_high = (SPT_PMC_MPHY_CORE_STS_1 << 16);

	mutex_lock(&pmcdev->lock);

	if (pmc_core_send_msg(&mphy_core_reg_low) != 0) {
		err = -EBUSY;
		goto out_unlock;
	}

	msleep(10);
	val_low = pmc_core_reg_read(pmcdev, SPT_PMC_MFPMC_OFFSET);

	if (pmc_core_send_msg(&mphy_core_reg_high) != 0) {
		err = -EBUSY;
		goto out_unlock;
	}

	msleep(10);
	val_high = pmc_core_reg_read(pmcdev, SPT_PMC_MFPMC_OFFSET);

	for (index = 0; map[index].name && index < 8; index++) {
		pmc_core_printf(s, "%-32s\tState: %s %s\n",
				map[index].name,
				map[index].bit_mask & val_low ? "Not power gated" :
				"Power gated", pmc_core_required_string(pmcdev, &map[index]));
	}

	for (index = 8; map[index].name; index++) {
		pmc_core_printf(s, "%-32s\tState: %s %s\n",
				map[index].name,
				map[index].bit_mask & val_high ? "Not power gated" :
				"Power gated", pmc_core_required_string(pmcdev, &map[index]));
	}

out_unlock:
	mutex_unlock(&pmcdev->lock);
	return err;
}

static int pmc_core_mphy_pg_sts_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_core_mphy_pg_sts_show, inode->i_private);
}

static const struct file_operations pmc_core_mphy_pg_ops = {
	.open           = pmc_core_mphy_pg_sts_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int pmc_core_pll_show(struct seq_file *s, void *unused)
{
	struct pmc_dev *pmcdev = s ? s->private : &pmc;
	const struct pmc_bit_map *map = pmcdev->map->pll_sts;
	u32 mphy_common_reg, val;
	int index, err = 0;

	if (!pmcdev->map->pll_sts)
		return 0;

	if (pmcdev->pmc_xram_read_bit) {
		pmc_core_printf(s, "Access denied: please disable PMC_READ_DISABLE setting in BIOS.\n");
		return 0;
	}

	mphy_common_reg  = (SPT_PMC_MPHY_COM_STS_0 << 16);
	mutex_lock(&pmcdev->lock);

	if (pmc_core_send_msg(&mphy_common_reg) != 0) {
		err = -EBUSY;
		goto out_unlock;
	}

	/* Observed PMC HW response latency for MTPMC-MFPMC is ~10 ms */
	msleep(10);
	val = pmc_core_reg_read(pmcdev, SPT_PMC_MFPMC_OFFSET);

	for (index = 0; map[index].name ; index++) {
		pmc_core_printf(s, "%-32s\tState: %s %s\n",
				map[index].name,
				map[index].bit_mask & val ? "Active" : "Idle", pmc_core_required_string(pmcdev, &map[index]));
	}

out_unlock:
	mutex_unlock(&pmcdev->lock);
	return err;
}

static int pmc_core_pll_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_core_pll_show, inode->i_private);
}

static const struct file_operations pmc_core_pll_ops = {
	.open           = pmc_core_pll_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static ssize_t pmc_core_ltr_ignore_write(struct file *file, const char __user
*userbuf, size_t count, loff_t *ppos)
{
	struct pmc_dev *pmcdev = &pmc;
	const struct pmc_reg_map *map = pmcdev->map;
	u32 val, buf_size, fd;
	int err = 0;

	buf_size = count < 64 ? count : 64;
	mutex_lock(&pmcdev->lock);

	if (kstrtou32_from_user(userbuf, buf_size, 10, &val)) {
		err = -EFAULT;
		goto out_unlock;
	}

	if (val > NUM_IP_IGN_ALLOWED) {
		err = -EINVAL;
		goto out_unlock;
	}

	fd = pmc_core_reg_read(pmcdev, map->ltr_ignore_offset);
	fd |= (1U << val);
	pmc_core_reg_write(pmcdev, map->ltr_ignore_offset, fd);

out_unlock:
	mutex_unlock(&pmcdev->lock);
	return err == 0 ? count : err;
}

static int pmc_core_ltr_ignore_show(struct seq_file *s, void *unused)
{
	return 0;
}

static int pmc_core_ltr_ignore_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_core_ltr_ignore_show, inode->i_private);
}

static const struct file_operations pmc_core_ltr_ignore_ops = {
	.open           = pmc_core_ltr_ignore_open,
	.read           = seq_read,
	.write          = pmc_core_ltr_ignore_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void pmc_core_dbgfs_unregister(struct pmc_dev *pmcdev)
{
	debugfs_remove_recursive(pmcdev->dbgfs_dir);
}

static int pmc_core_dbgfs_register(struct pmc_dev *pmcdev)
{
	struct dentry *dir, *file;

	dir = debugfs_create_dir("pmc_core", NULL);
	if (!dir)
		return -ENOMEM;

	pmcdev->dbgfs_dir = dir;
	file = debugfs_create_file("slp_s0_residency_usec", S_IFREG | S_IRUGO,
				   dir, pmcdev, &pmc_core_dev_state);
	if (!file)
		goto err;

	file = debugfs_create_file("pch_ip_power_gating_status",
				   S_IFREG | S_IRUGO, dir, pmcdev,
				   &pmc_core_ppfear_ops);
	if (!file)
		goto err;

	if (pmcdev->map->mphy_sts) {
		file = debugfs_create_file("mphy_core_lanes_power_gating_status",
					   S_IFREG | S_IRUGO, dir, pmcdev,
					   &pmc_core_mphy_pg_ops);
		if (!file)
			goto err;
	}

	if (pmcdev->map->pll_sts) {
		file = debugfs_create_file("pll_status",
					   S_IFREG | S_IRUGO, dir,
					   pmcdev,
					   &pmc_core_pll_ops);
		if (!file)
			goto err;
	}

	file = debugfs_create_file("ltr_ignore",
				   S_IFREG | S_IRUGO, dir, pmcdev,
				   &pmc_core_ltr_ignore_ops);

	if (!file)
		goto err;

	file = debugfs_create_bool("pmc_debug_messages_on", 0644, dir,
				   &pmc_debug_messages_on);
	if (!file)
		goto err;

	return 0;
err:
	pmc_core_dbgfs_unregister(pmcdev);
	return -ENODEV;
}

static int pmc_acpi_s2idle_wake_event(struct notifier_block *this,
				      unsigned long event, void *ptr)
{
	if (pmc_debug_messages_on) {
		pmc_core_ppfear_sts_show(NULL, NULL);
		pmc_core_mphy_pg_sts_show(NULL, NULL);
		pmc_core_pll_show(NULL, NULL);
	}

	return NOTIFY_DONE;
}

static struct notifier_block acpi_s2idle_wake_notifier = {
	.notifier_call = pmc_acpi_s2idle_wake_event,
};

static void pmc_core_acpi_wake_register(void)
{
	register_acpi_s2idle_wake_notifier(&acpi_s2idle_wake_notifier);
}

#else
static inline int pmc_core_dbgfs_register(struct pmc_dev *pmcdev)
{
	return 0;
}

static inline void pmc_core_dbgfs_unregister(struct pmc_dev *pmcdev)
{
}

static inline void pmc_core_acpi_wake_register(void)
{
}
#endif /* CONFIG_DEBUG_FS */

static const struct x86_cpu_id intel_pmc_core_ids[] = {
	{ X86_VENDOR_INTEL, 6, INTEL_FAM6_SKYLAKE_MOBILE, X86_FEATURE_MWAIT,
		(kernel_ulong_t)NULL},
	{ X86_VENDOR_INTEL, 6, INTEL_FAM6_SKYLAKE_DESKTOP, X86_FEATURE_MWAIT,
		(kernel_ulong_t)NULL},
	{ X86_VENDOR_INTEL, 6, INTEL_FAM6_KABYLAKE_MOBILE, X86_FEATURE_MWAIT,
		(kernel_ulong_t)NULL},
	{ X86_VENDOR_INTEL, 6, INTEL_FAM6_KABYLAKE_DESKTOP, X86_FEATURE_MWAIT,
		(kernel_ulong_t)NULL},
	{}
};

static int pmc_core_init(struct pmc_dev *pmcdev, const struct pmc_reg_map *map)
{
	static bool registered;
	int err;

	if (registered)
		return -EEXIST;

	pmcdev->regbase = ioremap_nocache(pmcdev->base_addr,
					  map->regmap_length);
	if (!pmcdev->regbase) {
		pr_debug("PMC Core: ioremap failed.\n");
		return -ENOMEM;
	}

	mutex_init(&pmcdev->lock);
	pmcdev->map = map;
	pmcdev->pmc_xram_read_bit = pmc_core_check_read_lock_bit();

	err = pmc_core_dbgfs_register(pmcdev);
	if (err < 0)
		pr_warn("PMC Core: debugfs register failed.\n");

	pmc.has_slp_s0_res = true;

	pmc_core_acpi_wake_register();

	registered = true;

	return 0;
}

static int pmc_core_pci_probe(struct pci_dev *dev,
			      const struct pci_device_id *id)
{
	struct pmc_dev *pmcdev = &pmc;
	const struct x86_cpu_id *cpu_id;
	const struct pmc_reg_map *map = (struct pmc_reg_map *)id->driver_data;
	int err;

	cpu_id = x86_match_cpu(intel_pmc_core_ids);
	if (!cpu_id) {
		dev_dbg(&dev->dev, "PMC Core: cpuid mismatch.\n");
		return -EINVAL;
	}

	err = pcim_enable_device(dev);
	if (err < 0) {
		dev_dbg(&dev->dev, "PMC Core: failed to enable Power Management Controller.\n");
		return err;
	}

	err = pci_read_config_dword(dev,
				    SPT_PMC_BASE_ADDR_OFFSET,
				    &pmcdev->base_addr);
	if (err < 0) {
		dev_dbg(&dev->dev, "PMC Core: failed to read PCI config space.\n");
		return err;
	}
	pmcdev->base_addr &= PMC_BASE_ADDR_MASK;
	dev_dbg(&dev->dev, "PMC Core: PWRMBASE is %#x\n", pmcdev->base_addr);

	return pmc_core_init(pmcdev, map);
}

static struct pci_driver intel_pmc_core_driver = {
	.name = "intel_pmc_core",
	.id_table = pmc_pci_ids,
	.probe =  pmc_core_pci_probe,
};

builtin_pci_driver(intel_pmc_core_driver);


static const struct pci_device_id host_bridge_pci_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x590C), (kernel_ulong_t)&spt_reg_map },
	{ PCI_VDEVICE(INTEL, 0x3ED0), (kernel_ulong_t)&cnp_reg_map },
	{ PCI_VDEVICE(INTEL, 0x3EC4), (kernel_ulong_t)&cnp_reg_map },
	{ PCI_VDEVICE(INTEL, 0x5A02), (kernel_ulong_t)&cnp_reg_map },
	{ 0, },
};

static int __init pmc_core_probe(void)
{
	struct pmc_dev *pmc_dev = &pmc;
	struct pmc_reg_map *reg_map = NULL;
	u64 base_address;
	int i = 0;

	if (lpit_read_residency_count_address(&base_address))
		return -ENODEV;

	pr_info ("SLP_S0 base address as %llx \n", base_address);

	pmc_dev->base_addr = 0;
	while (host_bridge_pci_ids[i].vendor) {
		struct pci_dev *pdev;

		pdev = pci_get_device(host_bridge_pci_ids[i].vendor,
				      host_bridge_pci_ids[i].device,
				      NULL);
		if (pdev) {
			reg_map = (struct pmc_reg_map *) host_bridge_pci_ids[i].driver_data;
			base_address -= reg_map->slp_s0_offset;
			pmc_dev->base_addr = base_address;
			pr_info ("Final base address as %llx \n", base_address);
		}
		++i;
	}

	if (!pmc_dev->base_addr)
		return -ENODEV;

	return pmc_core_init(pmc_dev, reg_map);
}

late_initcall(pmc_core_probe);
