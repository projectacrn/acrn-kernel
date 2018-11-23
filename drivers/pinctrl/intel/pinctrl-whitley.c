// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Whitley PCH pinctrl/GPIO driver
 *
 * Copyright (C) 2018, Intel Corporation
 * Author: Andy Shevchenko <andriy.shevchenko@linux.intel.com>
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-intel.h"

#define ICX_PAD_OWN	0x020
#define ICX_PADCFGLOCK	0x080
#define ICX_HOSTSW_OWN	0x0b0
#define ICX_GPI_IS	0x100
#define ICX_GPI_IE	0x120

#define ICX_GPP(r, s, e)				\
	{						\
		.reg_num = (r),				\
		.base = (s),				\
		.size = ((e) - (s) + 1),		\
	}

#define ICX_COMMUNITY(b, s, e, g)			\
	{						\
		.barno = (b),				\
		.padown_offset = ICX_PAD_OWN,		\
		.padcfglock_offset = ICX_PADCFGLOCK,	\
		.hostown_offset = ICX_HOSTSW_OWN,	\
		.is_offset = ICX_GPI_IS,		\
		.ie_offset = ICX_GPI_IE,		\
		.pin_base = (s),			\
		.npins = ((e) - (s) + 1),		\
		.gpps = (g),				\
		.ngpps = ARRAY_SIZE(g),			\
	}

/* Whitley */
static const struct pinctrl_pin_desc icxsp_pins[] = {
	/* FIVRDEBUG */
	PINCTRL_PIN(0, "FIVR_CLKREF"),
	PINCTRL_PIN(1, "FIVR_PRB_DIG_0"),
	PINCTRL_PIN(2, "FIVR_PRB_DIG_1"),
	PINCTRL_PIN(3, "FBRK_N"),
	/* JTAG */
	PINCTRL_PIN(4, "TCK"),
	PINCTRL_PIN(5, "TDI"),
	PINCTRL_PIN(6, "TDO"),
	PINCTRL_PIN(7, "TMS"),
	PINCTRL_PIN(8, "TRST_N"),
	/* JTAG1 */
	PINCTRL_PIN(9, "DEBUG_EN_N"),
	PINCTRL_PIN(10, "PRDY_N"),
	PINCTRL_PIN(11, "PREQ_N"),
	/* MBP1 */
	PINCTRL_PIN(12, "MBP0_N"),
	PINCTRL_PIN(13, "MBP1_N"),
	PINCTRL_PIN(14, "MBP2_N"),
	PINCTRL_PIN(15, "MBP3_N"),
	/* MBP2 */
	PINCTRL_PIN(16, "MCP_SPARE0"),
	PINCTRL_PIN(17, "MCP_SPARE1"),
	PINCTRL_PIN(18, "MCP_SPARE2"),
	PINCTRL_PIN(19, "MCP_SPARE3"),
	/* MCP */
	PINCTRL_PIN(20, "MCP_MBP0_N"),
	PINCTRL_PIN(21, "MCP_MBP1_N"),
	PINCTRL_PIN(22, "MCPSMBUSSCL"),
	PINCTRL_PIN(23, "MCPSMBUSSDA"),
	/* MISC1 */
	PINCTRL_PIN(24, "CATERR_N"),
	PINCTRL_PIN(25, "ERROR0_N"),
	PINCTRL_PIN(26, "ERROR1_N"),
	PINCTRL_PIN(27, "ERROR2_N"),
	/* MISC2 */
	PINCTRL_PIN(28, "SPDSMBUSSCL0"),
	PINCTRL_PIN(29, "SPDSMBUSSDA0"),
	PINCTRL_PIN(30, "MEMHOT_OUT_N"),
	PINCTRL_PIN(31, "MEMHOT_IN_N"),
	/* MISC3 */
	PINCTRL_PIN(32, "MSMI_N"),
	PINCTRL_PIN(33, "PECI"),
	PINCTRL_PIN(34, "PROCHOT_N"),
	/* MISC4 */
	PINCTRL_PIN(35, "SPDSMBUSSCL1"),
	PINCTRL_PIN(36, "SPDSMBUSSDA1"),
	PINCTRL_PIN(37, "VPPSMBUSSCL"),
	PINCTRL_PIN(38, "VPPSMBUSSDA"),
	/* MISC5 */
	PINCTRL_PIN(39, "ENH_MCECC_DIS"),
	PINCTRL_PIN(40, "DMI_MODE_OVERRIDE"),
	PINCTRL_PIN(41, "VLN_DISABLE"),
	PINCTRL_PIN(42, "MCSMBUS_ALRT_N"),
	PINCTRL_PIN(43, "MEMTRIP_N"),
	/* PMAX */
	PINCTRL_PIN(44, "VSENSEPMAX"),
	/* PTI */
	PINCTRL_PIN(45, "PTI_0"),
	PINCTRL_PIN(46, "PTI_1"),
	PINCTRL_PIN(47, "PTI_10"),
	PINCTRL_PIN(48, "PTI_11"),
	PINCTRL_PIN(49, "PTI_12"),
	PINCTRL_PIN(50, "PTI_13"),
	PINCTRL_PIN(51, "PTI_14"),
	PINCTRL_PIN(52, "PTI_15"),
	PINCTRL_PIN(53, "PTI_2"),
	PINCTRL_PIN(54, "PTI_3"),
	PINCTRL_PIN(55, "PTI_4"),
	PINCTRL_PIN(56, "PTI_5"),
	PINCTRL_PIN(57, "PTI_6"),
	PINCTRL_PIN(58, "PTI_7"),
	PINCTRL_PIN(59, "PTI_8"),
	PINCTRL_PIN(60, "PTI_9"),
	PINCTRL_PIN(61, "PTI_STB_0"),
	PINCTRL_PIN(62, "PTI_STB_1"),
	/* RESET2 */
	PINCTRL_PIN(63, "EAR_N"),
	PINCTRL_PIN(64, "PMSYNC"),
	PINCTRL_PIN(65, "PMSYNC_CLK"),
	PINCTRL_PIN(66, "THERMTRIP_N"),
	/* RESET3 */
	PINCTRL_PIN(67, "TSC_SYNC"),
	PINCTRL_PIN(68, "NMI"),
	PINCTRL_PIN(69, "PM_FAST_WAKE_N"),
	/* SPARE */
	PINCTRL_PIN(70, "LGSPARE_0"),
	PINCTRL_PIN(71, "LGSPARE_1"),
	PINCTRL_PIN(72, "LGSPARE_2"),
	PINCTRL_PIN(73, "LGSPARE_3"),
	PINCTRL_PIN(74, "LGSPARE_4"),
	PINCTRL_PIN(75, "LGSPARE_5"),
	PINCTRL_PIN(76, "LGSPARE_6"),
	/* STRAP */
	PINCTRL_PIN(77, "SAFE_MODE_BOOT"),
	PINCTRL_PIN(78, "PROCDIS_N"),
	PINCTRL_PIN(79, "SOCKET_ID_2"),
	PINCTRL_PIN(80, "FRMAGENT"),
	PINCTRL_PIN(81, "BIST_ENABLE"),
	PINCTRL_PIN(82, "TXT_PLTEN"),
	PINCTRL_PIN(83, "SOCKET_ID_1"),
	PINCTRL_PIN(84, "TAP_ODT_EN"),
	PINCTRL_PIN(85, "EX_LEGACY_SKT"),
	PINCTRL_PIN(86, "TXT_AGENT"),
	PINCTRL_PIN(87, "SOCKET_ID_0"),
	PINCTRL_PIN(88, "BMCINIT"),
	/* SVID1 */
	PINCTRL_PIN(89, "SVIDALERT0_N"),
	PINCTRL_PIN(90, "SVIDCLK0"),
	PINCTRL_PIN(91, "SVIDDATA0"),
	/* SVID2 */
	PINCTRL_PIN(92, "SVIDALERT1_N"),
	PINCTRL_PIN(93, "SVIDCLK1"),
	PINCTRL_PIN(94, "SVIDDATA1"),
};

static const struct intel_padgroup icxsp_community0_gpps[] = {
	ICX_GPP(0, 0, 3),	/* FIVRDEBUG */
	ICX_GPP(1, 4, 8),	/* JTAG */
	ICX_GPP(2, 9, 11),	/* JTAG1 */
	ICX_GPP(3, 12, 15),	/* MBP1 */
	ICX_GPP(4, 16, 19),	/* MBP2 */
	ICX_GPP(5, 20, 23),	/* MCP */
	ICX_GPP(6, 24, 27),	/* MISC1 */
	ICX_GPP(7, 28, 31),	/* MISC2 */
	ICX_GPP(8, 32, 34),	/* MISC3 */
	ICX_GPP(9, 35, 38),	/* MISC4 */
	ICX_GPP(10, 39, 43),	/* MISC5 */
	ICX_GPP(11, 44, 44),	/* PMAX */
	ICX_GPP(12, 45, 62),	/* PTI */
	ICX_GPP(13, 63, 66),	/* RESET2 */
	ICX_GPP(14, 67, 69),	/* RESET3 */
	ICX_GPP(15, 70, 76),	/* SPARE */
	ICX_GPP(16, 77, 88),	/* STRAP */
	ICX_GPP(17, 89, 91),	/* SVID1 */
	ICX_GPP(18, 92, 94),	/* SVID2 */
};

static const struct intel_community icxsp_communities[] = {
	ICX_COMMUNITY(0, 0, 94, icxsp_community0_gpps),	/* WEST */
};

static const struct intel_pingroup icxsp_groups[] = {
	/* PLACE HOLDER */
};

static const struct intel_function icxsp_functions[] = {
	/* PLACE HOLDER */
};

static const struct intel_pinctrl_soc_data icxsp_soc_data = {
	.pins = icxsp_pins,
	.npins = ARRAY_SIZE(icxsp_pins),
	.groups = icxsp_groups,
	.ngroups = ARRAY_SIZE(icxsp_groups),
	.functions = icxsp_functions,
	.nfunctions = ARRAY_SIZE(icxsp_functions),
	.communities = icxsp_communities,
	.ncommunities = ARRAY_SIZE(icxsp_communities),
};

static const struct acpi_device_id icxsp_pinctrl_acpi_match[] = {
	{ "", (kernel_ulong_t)&icxsp_soc_data },
	{ }
};
MODULE_DEVICE_TABLE(acpi, icxsp_pinctrl_acpi_match);

static INTEL_PINCTRL_PM_OPS(icxsp_pinctrl_pm_ops);

static struct platform_driver icxsp_pinctrl_driver = {
	.probe = intel_pinctrl_probe_by_hid,
	.driver = {
		.name = "whitley-pinctrl",
		.acpi_match_table = icxsp_pinctrl_acpi_match,
		.pm = &icxsp_pinctrl_pm_ops,
	},
};

module_platform_driver(icxsp_pinctrl_driver);

MODULE_AUTHOR("Andy Shevchenko <andriy.shevchenko@linux.intel.com>");
MODULE_DESCRIPTION("Intel Whitley PCH pinctrl/GPIO driver");
MODULE_LICENSE("GPL v2");
