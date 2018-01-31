/*
 * Intel Glenview SoC pinctrl/GPIO driver
 *
 * Copyright (C) 2017 Intel Corporation
 * Author: Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-intel.h"

#define GLV_PAD_OWN	0x020
#define GLV_HOSTSW_OWN	0x0b0
#define GLV_PADCFGLOCK	0x080
#define GLV_GPI_IE	0x110

#define GLV_COMMUNITY(s, e)				\
	{						\
		.padown_offset = GLV_PAD_OWN,		\
		.padcfglock_offset = GLV_PADCFGLOCK,	\
		.hostown_offset = GLV_HOSTSW_OWN,	\
		.ie_offset = GLV_GPI_IE,		\
		.gpp_size = 32,                         \
		.pin_base = (s),			\
		.npins = ((e) - (s) + 1),		\
	}

static const struct pinctrl_pin_desc glv_north_pins[] = {
	PINCTRL_PIN(0, "GP_SSP_0_CLK0"),
	PINCTRL_PIN(1, "GP_SSP_0_FS0"),
	PINCTRL_PIN(2, "GP_SSP_0_RXD"),
	PINCTRL_PIN(3, "GP_SSP_0_TXD"),
	PINCTRL_PIN(4, "GP_DMIC_CLK_A0"),
	PINCTRL_PIN(5, "GP_DMIC_CLK_AB1"),
	PINCTRL_PIN(6, "GP_DMIC_CLK_B0"),
	PINCTRL_PIN(7, "GP_DMIC_DATA_0"),
	PINCTRL_PIN(8, "GP_DMIC_DATA_1"),
	PINCTRL_PIN(9, "UFS_REF_CLK"),
	PINCTRL_PIN(10, "vCNV_BT_I2S_BCLK"),
	PINCTRL_PIN(11, "vCNV_BT_I2S_WS_SYNC"),
	PINCTRL_PIN(12, "vCNV_BT_I2S_SDO"),
	PINCTRL_PIN(13, "vCNV_BT_I2S_SDI"),
	PINCTRL_PIN(14, "vAVS_I2S0_BCLK"),
	PINCTRL_PIN(15, "vAVS_I2S0_WS_SYNC"),
	PINCTRL_PIN(16, "vAVS_I2S0_SDO"),
	PINCTRL_PIN(17, "vAVS_I2S0_SDI"),
	PINCTRL_PIN(18, "vMDM_TO_AVS_I2S_BCLK"),
	PINCTRL_PIN(19, "vMDM_TO_AVS_I2S_WS_SYNC"),
	PINCTRL_PIN(20, "vMDM_TO_AVS_I2S_SDO"),
	PINCTRL_PIN(21, "vMDM_TO_AVS_I2S_SDI"),
	PINCTRL_PIN(22, "vAVS_TO_MDM_I2S_BCLK"),
	PINCTRL_PIN(23, "vAVS_TO_MDM_I2S_WS_SYNC"),
	PINCTRL_PIN(24, "vAVS_TO_MDM_I2S_SDO"),
	PINCTRL_PIN(25, "vAVS_TO_MDM_I2S_SDI"),
};

static const unsigned glv_north_ssp0_pins[] = { 0, 1, 2, 3 };

static const struct intel_pingroup glv_north_groups[] = {
	PIN_GROUP("ssp0_grp", glv_north_ssp0_pins, 1),
};

static const char * const glv_north_ssp0_groups[] = { "ssp0_grp" };

static const struct intel_function glv_north_functions[] = {
	FUNCTION("ssp0", glv_north_ssp0_groups),
};

static const struct intel_community glv_north_communities[] = {
	GLV_COMMUNITY(0, 25),
};

static const struct intel_pinctrl_soc_data glv_north_soc_data = {
	.uid = "1",
	.pins = glv_north_pins,
	.npins = ARRAY_SIZE(glv_north_pins),
	.groups = glv_north_groups,
	.ngroups = ARRAY_SIZE(glv_north_groups),
	.functions = glv_north_functions,
	.nfunctions = ARRAY_SIZE(glv_north_functions),
	.communities = glv_north_communities,
	.ncommunities = ARRAY_SIZE(glv_north_communities),
};

static const struct pinctrl_pin_desc glv_east_pins[] = {
	PINCTRL_PIN(0, "RGMII_0_RESET"),
	PINCTRL_PIN(1, "RGMII_0_INT"),
	PINCTRL_PIN(2, "RGMII_0_TXCLK"),
	PINCTRL_PIN(3, "RGMII_0_TXCTL"),
	PINCTRL_PIN(4, "RGMII_0_TX_DATA_0"),
	PINCTRL_PIN(5, "RGMII_0_TX_DATA_1"),
	PINCTRL_PIN(6, "RGMII_0_TX_DATA_2"),
	PINCTRL_PIN(7, "RGMII_0_TX_DATA_3"),
	PINCTRL_PIN(8, "RGMII_0_RXCLK"),
	PINCTRL_PIN(9, "RGMII_0_RXCTL"),
	PINCTRL_PIN(10, "RGMII_0_RX_DATA_0"),
	PINCTRL_PIN(11, "RGMII_0_RX_DATA_1"),
	PINCTRL_PIN(12, "RGMII_0_RX_DATA_2"),
	PINCTRL_PIN(13, "RGMII_0_RX_DATA_3"),
	PINCTRL_PIN(14, "RGMII_0_MDC"),
	PINCTRL_PIN(15, "RGMII_0_MDIO"),
	PINCTRL_PIN(16, "PLT_CLK_4"),
	PINCTRL_PIN(17, "PLT_CLK_5"),
	PINCTRL_PIN(18, "PLT_CLK_0"),
	PINCTRL_PIN(19, "PLT_CLK_1"),
	PINCTRL_PIN(20, "PLT_CLK_2"),
	PINCTRL_PIN(21, "PLT_CLK_3"),
	PINCTRL_PIN(22, "CNV_38P4_CLKIN"),
	PINCTRL_PIN(23, "CNV_38P4_CLKREQ"),
	PINCTRL_PIN(24, "GP_CNV_A4WP_PRESENT"),
	PINCTRL_PIN(25, "GP_CNV_BRI_DT"),
	PINCTRL_PIN(26, "GP_CNV_BRI_RSP"),
	PINCTRL_PIN(27, "GP_CNV_RF_RESET_B"),
	PINCTRL_PIN(28, "GP_CNV_RGI_DT"),
	PINCTRL_PIN(29, "GP_CNV_RGI_RSP"),
	PINCTRL_PIN(30, "GP_CNV_BTEN"),
	PINCTRL_PIN(31, "GP_CNV_GNSSEN"),
	PINCTRL_PIN(32, "GP_CNV_WCEN"),
	PINCTRL_PIN(33, "GP_CNV_WFEN"),
	PINCTRL_PIN(34, "GP_FST_SPI_CLK"),
	PINCTRL_PIN(35, "GP_FST_SPI_CS0_B"),
	PINCTRL_PIN(36, "GP_FST_SPI_CS1_B"),
	PINCTRL_PIN(37, "GP_FST_SPI_MISO"),
	PINCTRL_PIN(38, "GP_FST_SPI_MOSI"),
	PINCTRL_PIN(39, "vCNV_BTEN"),
	PINCTRL_PIN(40, "vCNV_GNEN"),
	PINCTRL_PIN(41, "vCNV_WFEN"),
	PINCTRL_PIN(42, "vCNV_WCEN"),
	PINCTRL_PIN(43, "vCNV_BT_IF_SELECT"),
	PINCTRL_PIN(44, "vCNV_BT_UART_TXD"),
	PINCTRL_PIN(45, "vCNV_BT_UART_RXD"),
	PINCTRL_PIN(46, "vCNV_BT_UART_CTS_B"),
	PINCTRL_PIN(47, "vCNV_BT_UART_RTS_B"),
	PINCTRL_PIN(48, "vCNV_MFUART1_TXD"),
	PINCTRL_PIN(49, "vCNV_MFUART1_RXD"),
	PINCTRL_PIN(50, "vCNV_MFUART1_CTS_B"),
	PINCTRL_PIN(51, "vCNV_MFUART1_RTS_B"),
	PINCTRL_PIN(52, "vCNV_GNSS_UART_TXD"),
	PINCTRL_PIN(53, "vCNV_GNSS_UART_RXD"),
	PINCTRL_PIN(54, "vCNV_GNSS_UART_CTS_B"),
	PINCTRL_PIN(55, "vCNV_GNSS_UART_RTS_B"),
	PINCTRL_PIN(56, "vLPSS_UART0_TXD"),
	PINCTRL_PIN(57, "vLPSS_UART0_RXD"),
	PINCTRL_PIN(58, "vLPSS_UART0_CTS_B"),
	PINCTRL_PIN(59, "vLPSS_UART0_RTS_B"),
	PINCTRL_PIN(60, "vLPSS_UART1_TXD"),
	PINCTRL_PIN(61, "vLPSS_UART1_RXD"),
	PINCTRL_PIN(62, "vLPSS_UART1_CTS_B"),
	PINCTRL_PIN(63, "vLPSS_UART1_RTS_B"),
	PINCTRL_PIN(64, "vLPSS_UART2_TXD"),
	PINCTRL_PIN(65, "vLPSS_UART2_RXD"),
	PINCTRL_PIN(66, "vLPSS_UART2_CTS_B"),
	PINCTRL_PIN(66, "vLPSS_UART2_RTS_B"),
	PINCTRL_PIN(67, "vCNV_MFUART2_TXD"),
	PINCTRL_PIN(68, "vCNV_MFUART2_RXD"),
	PINCTRL_PIN(69, "vCNV_PA_BLANKING"),
	PINCTRL_PIN(70, "vCNV_SYSCLK"),
	PINCTRL_PIN(71, "vCNV_FINE_TIME_AIDING"),
	PINCTRL_PIN(72, "vCNV_MDM_UART_TXD"),
	PINCTRL_PIN(73, "vCNV_MDM_UART_RXD"),
	PINCTRL_PIN(74, "vCNV_MDM_PA_BLANKING"),
	PINCTRL_PIN(75, "vCNV_MDM_SYSCLK"),
	PINCTRL_PIN(76, "vCNV_MDM_FINE_TIME_AIDING"),
};

static const unsigned glv_east_spi2_pins[] = { 34, 35, 36, 37, 38 };

static const struct intel_pingroup glv_east_groups[] = {
	PIN_GROUP("spi2_grp", glv_east_spi2_pins, 2),
};

static const char * const glv_east_spi2_groups[] = { "spi2_grp" };

static const struct intel_function glv_east_functions[] = {
	FUNCTION("spi2", glv_east_spi2_groups),
};

static const struct intel_community glv_east_communities[] = {
	GLV_COMMUNITY(0, 76),
};

static const struct intel_pinctrl_soc_data glv_east_soc_data = {
	.uid = "2",
	.pins = glv_east_pins,
	.npins = ARRAY_SIZE(glv_east_pins),
	.groups = glv_east_groups,
	.ngroups = ARRAY_SIZE(glv_east_groups),
	.functions = glv_east_functions,
	.nfunctions = ARRAY_SIZE(glv_east_functions),
	.communities = glv_east_communities,
	.ncommunities = ARRAY_SIZE(glv_east_communities),
};

static const struct pinctrl_pin_desc glv_south_pins[] = {
	PINCTRL_PIN(0, "CP_STANDBY"),
	PINCTRL_PIN(1, "CP_SVID_ALERT_N"),
	PINCTRL_PIN(2, "CP_SVID_VCLK"),
	PINCTRL_PIN(3, "CP_SVID_VDIO"),
	PINCTRL_PIN(4, "CP_SYS_CLK_REQ"),
	PINCTRL_PIN(5, "CP_RESET_TRX1_N"),
	PINCTRL_PIN(6, "CP_RESET_TRX2_N"),
	PINCTRL_PIN(7, "CP_PWRGOOD"),
	PINCTRL_PIN(8, "CP_RESET_ALL_N"),
	PINCTRL_PIN(9, "CP_RESET_REQ_N"),
	PINCTRL_PIN(10, "GP_MDIGRF_AUX_0_EN"),
	PINCTRL_PIN(11, "GP_MDIGRF_AUX_1_EN"),
	PINCTRL_PIN(12, "GP_MDIGRF_MAIN_0_EN"),
	PINCTRL_PIN(13, "GP_MDIGRF_MAIN_1_EN"),
	PINCTRL_PIN(14, "GP_USIF_1_CSO"),
	PINCTRL_PIN(15, "GP_USIF_1_RXD_MSTR"),
	PINCTRL_PIN(16, "GP_USIF_1_SCLK"),
	PINCTRL_PIN(17, "GP_USIF_1_TXD_MSTR"),
	PINCTRL_PIN(18, "USIM_0_IN"),
	PINCTRL_PIN(19, "USIM_1_IN"),
};

static const struct intel_community glv_south_communities[] = {
	GLV_COMMUNITY(0, 19),
};

static const struct intel_pinctrl_soc_data glv_south_soc_data = {
	.uid = "3",
	.pins = glv_south_pins,
	.npins = ARRAY_SIZE(glv_south_pins),
	.communities = glv_south_communities,
	.ncommunities = ARRAY_SIZE(glv_south_communities),
};

static const struct pinctrl_pin_desc glv_west_pins[] = {
	PINCTRL_PIN(0, "GP_SDIO_0_CD_B"),
	PINCTRL_PIN(1, "GP_SDIO_0_CLK"),
	PINCTRL_PIN(2, "GP_SDIO_0_CMD"),
	PINCTRL_PIN(3, "GP_SDIO_0_DAT_0"),
	PINCTRL_PIN(4, "GP_SDIO_0_DAT_1"),
	PINCTRL_PIN(5, "GP_SDIO_0_DAT_2"),
	PINCTRL_PIN(6, "GP_SDIO_0_DAT_3"),
	PINCTRL_PIN(7, "GP_SDIO_0_LVL_WP"),
	PINCTRL_PIN(8, "GP_SDIO_0_PWR_DOWN_B"),
	PINCTRL_PIN(9, "EMMC_0_CLK"),
	PINCTRL_PIN(10, "EMMC_0_CMD"),
	PINCTRL_PIN(11, "EMMC_0_D0"),
	PINCTRL_PIN(12, "EMMC_0_D1"),
	PINCTRL_PIN(13, "EMMC_0_D2"),
	PINCTRL_PIN(14, "EMMC_0_D3"),
	PINCTRL_PIN(15, "EMMC_0_D4"),
	PINCTRL_PIN(16, "EMMC_0_D5"),
	PINCTRL_PIN(17, "EMMC_0_D6"),
	PINCTRL_PIN(18, "EMMC_0_D7"),
	PINCTRL_PIN(19, "EMMC_0_RST_B"),
	PINCTRL_PIN(20, "EMMC_0_STROBE"),
	PINCTRL_PIN(21, "GP_INTD_DSI_TE1"),
	PINCTRL_PIN(22, "GP_INTD_DSI_TE2"),
	PINCTRL_PIN(23, "GP_EINT_0"),
	PINCTRL_PIN(24, "GP_EINT_1"),
	PINCTRL_PIN(25, "GP_UART_0_CTS"),
	PINCTRL_PIN(26, "GP_UART_0_RTS"),
	PINCTRL_PIN(27, "GP_UART_0_RX"),
	PINCTRL_PIN(28, "GP_UART_0_TX"),
	PINCTRL_PIN(29, "GP_UART_1_CTS"),
	PINCTRL_PIN(30, "GP_UART_1_RTS"),
	PINCTRL_PIN(31, "GP_UART_1_RX"),
	PINCTRL_PIN(32, "GP_UART_1_TX"),
	PINCTRL_PIN(33, "GP_ISH_D_4"),
	PINCTRL_PIN(34, "GP_ISH_D_5"),
	PINCTRL_PIN(35, "GP_ISH_D_6"),
	PINCTRL_PIN(36, "GP_ISH_D_7"),
	PINCTRL_PIN(37, "GP_ISH_D_0"),
	PINCTRL_PIN(38, "GP_ISH_D_1"),
	PINCTRL_PIN(39, "GP_ISH_D_2"),
	PINCTRL_PIN(40, "GP_ISH_D_3"),
	PINCTRL_PIN(41, "GP_CAMERASB_0"),
	PINCTRL_PIN(42, "GP_CAMERASB_1"),
	PINCTRL_PIN(43, "GP_CAMERASB_2"),
	PINCTRL_PIN(44, "GP_CAMERASB_3"),
	PINCTRL_PIN(45, "GP_CAMERASB_4"),
	PINCTRL_PIN(46, "GP_CAMERASB_5"),
	PINCTRL_PIN(47, "GP_CAMERASB_6"),
	PINCTRL_PIN(48, "GP_CAMERASB_7"),
	PINCTRL_PIN(49, "GP_CAMERASB_8"),
	PINCTRL_PIN(50, "GP_CAMERASB_9"),
	PINCTRL_PIN(51, "GP_CAMERASB_10"),
	PINCTRL_PIN(52, "GP_CAMERASB_11"),
	PINCTRL_PIN(53, "GPIO_3P3_0"),
	PINCTRL_PIN(54, "GPIO_3P3_1"),
	PINCTRL_PIN(55, "GPIO_3P3_2"),
	PINCTRL_PIN(56, "GPIO_3P3_3"),
	PINCTRL_PIN(57, "GPIO_3P3_4"),
	PINCTRL_PIN(58, "GPIO_3P3_5"),
	PINCTRL_PIN(59, "GPIO_3P3_6"),
	PINCTRL_PIN(60, "GPIO_3P3_7"),
	PINCTRL_PIN(61, "XXPRDY"),
	PINCTRL_PIN(62, "XXPREQ_B"),
	PINCTRL_PIN(63, "JTAG_RTCK"),
	PINCTRL_PIN(64, "JTAG_TCKC"),
	PINCTRL_PIN(65, "JTAG_TDIC"),
	PINCTRL_PIN(66, "JTAG_TDOC"),
	PINCTRL_PIN(67, "JTAG_TDOC_2"),
	PINCTRL_PIN(68, "JTAG_TMSC"),
	PINCTRL_PIN(69, "JTAG_TRST_B"),
	PINCTRL_PIN(70, "AP_PROCHOT_B"),
	PINCTRL_PIN(71, "AP_SVID_ALERT_B"),
	PINCTRL_PIN(72, "AP_SVID_VCLK"),
	PINCTRL_PIN(73, "AP_SVID_VDIO"),
	PINCTRL_PIN(74, "AP_THERMTRIP_N"),
	PINCTRL_PIN(75, "EMMC_DNX_PWM_EN_B"),
	PINCTRL_PIN(76, "PMU_PLTRST_B"),
	PINCTRL_PIN(77, "PMU_RESETBUTTON_B"),
	PINCTRL_PIN(78, "PMU_SLP_S0_B"),
	PINCTRL_PIN(79, "I2C_0_SCL"),
	PINCTRL_PIN(80, "I2C_0_SDA"),
	PINCTRL_PIN(81, "I2C_1_SCL"),
	PINCTRL_PIN(82, "I2C_1_SDA"),
	PINCTRL_PIN(83, "I2C_2_SCL"),
	PINCTRL_PIN(84, "I2C_2_SDA"),
	PINCTRL_PIN(85, "I2C_3_SCL"),
	PINCTRL_PIN(86, "I2C_3_SDA"),
	PINCTRL_PIN(87, "I2C_4_SCL"),
	PINCTRL_PIN(88, "I2C_4_SDA"),
	PINCTRL_PIN(89, "I2C_5_SCL"),
	PINCTRL_PIN(90, "I2C_5_SDA"),
	PINCTRL_PIN(91, "I2C_6_SCL"),
	PINCTRL_PIN(92, "I2C_6_SDA"),
	PINCTRL_PIN(93, "I2C_7_SCL"),
	PINCTRL_PIN(94, "I2C_7_SDA"),
	PINCTRL_PIN(95, "I2C_7_SCL"),
	PINCTRL_PIN(96, "I2C_8_SDA"),
	PINCTRL_PIN(97, "vCNV_BT_HOST_WAKEB"),
	PINCTRL_PIN(98, "vCNV_GNSS_HOST_WAKEB"),
};

static const unsigned glv_west_sdio0_pins[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
static const unsigned glv_west_emmc0_pins[] = {
	9, 10, 11, 12, 12, 14, 15, 16, 17, 18, 19, 20
};
static const unsigned glv_west_uart0_pins[] = { 25, 26, 27, 28 };
static const unsigned glv_west_uart1_pins[] = { 29, 30, 31, 32 };
static const unsigned glv_west_uart2_pins[] = { 29, 30, 31, 32 };
static const unsigned glv_west_spi0_pins[] = { 53, 54, 55, 56 };
static const unsigned glv_west_spi1_pins[] = { 57, 58, 59, 60 };
static const unsigned glv_west_pwm0_pins[] = { 57 };
static const unsigned glv_west_pwm1_pins[] = { 58 };
static const unsigned glv_west_pwm2_pins[] = { 59 };
static const unsigned glv_west_pwm3_pins[] = { 60 };
static const unsigned glv_west_i2c1_pins[] = { 81, 82 };
static const unsigned glv_west_i2c2_pins[] = { 83, 84 };
static const unsigned glv_west_i2c3_pins[] = { 85, 86 };
static const unsigned glv_west_i2c4_pins[] = { 87, 88 };
static const unsigned glv_west_i2c5_pins[] = { 89, 90 };
static const unsigned glv_west_i2c6_pins[] = { 91, 92 };
static const unsigned glv_west_i2c7_pins[] = { 93, 94 };
static const unsigned glv_west_i2c8_pins[] = { 95, 96 };

static const struct intel_pingroup glv_west_groups[] = {
	PIN_GROUP("sdio0_grp", glv_west_sdio0_pins, 1),
	PIN_GROUP("emmc0_grp", glv_west_emmc0_pins, 1),
	PIN_GROUP("uart0_grp", glv_west_uart0_pins, 1),
	PIN_GROUP("uart1_grp", glv_west_uart1_pins, 1),
	PIN_GROUP("uart2_grp", glv_west_uart2_pins, 2),
	PIN_GROUP("spi0_grp", glv_west_spi0_pins, 1),
	PIN_GROUP("spi1_grp", glv_west_spi1_pins, 1),
	PIN_GROUP("pwm0_grp", glv_west_pwm0_pins, 4),
	PIN_GROUP("pwm1_grp", glv_west_pwm1_pins, 4),
	PIN_GROUP("pwm2_grp", glv_west_pwm2_pins, 4),
	PIN_GROUP("pwm3_grp", glv_west_pwm3_pins, 4),
	/* These are for LPSS I2C */
	PIN_GROUP("i2c0_grp", glv_west_i2c1_pins, 2),
	PIN_GROUP("i2c1_grp", glv_west_i2c2_pins, 1),
	PIN_GROUP("i2c2_grp", glv_west_i2c3_pins, 1),
	PIN_GROUP("i2c3_grp", glv_west_i2c4_pins, 1),
	PIN_GROUP("i2c4_grp", glv_west_i2c5_pins, 1),
	PIN_GROUP("i2c5_grp", glv_west_i2c6_pins, 2),
	PIN_GROUP("i2c6_grp", glv_west_i2c7_pins, 2),
	PIN_GROUP("i2c7_grp", glv_west_i2c8_pins, 1),
};

static const char * const glv_west_sdio0_groups[] = { "sdio0_grp" };
static const char * const glv_west_emmc0_groups[] = { "emmc0_grp" };
static const char * const glv_west_uart0_groups[] = { "uart0_grp" };
static const char * const glv_west_uart1_groups[] = { "uart1_grp" };
static const char * const glv_west_uart2_groups[] = { "uart2_grp" };
static const char * const glv_west_spi0_groups[] = { "spi0_grp" };
static const char * const glv_west_spi1_groups[] = { "spi1_grp" };
static const char * const glv_west_pwm0_groups[] = { "pwm0_grp" };
static const char * const glv_west_pwm1_groups[] = { "pwm1_grp" };
static const char * const glv_west_pwm2_groups[] = { "pwm2_grp" };
static const char * const glv_west_pwm3_groups[] = { "pwm3_grp" };
static const char * const glv_west_i2c0_groups[] = { "i2c0_grp" };
static const char * const glv_west_i2c1_groups[] = { "i2c1_grp" };
static const char * const glv_west_i2c2_groups[] = { "i2c2_grp" };
static const char * const glv_west_i2c3_groups[] = { "i2c3_grp" };
static const char * const glv_west_i2c4_groups[] = { "i2c4_grp" };
static const char * const glv_west_i2c5_groups[] = { "i2c5_grp" };
static const char * const glv_west_i2c6_groups[] = { "i2c6_grp" };
static const char * const glv_west_i2c7_groups[] = { "i2c7_grp" };

static const struct intel_function glv_west_functions[] = {
	FUNCTION("sdio0", glv_west_sdio0_groups),
	FUNCTION("emmc0", glv_west_emmc0_groups),
	FUNCTION("uart0", glv_west_uart0_groups),
	FUNCTION("uart1", glv_west_uart1_groups),
	FUNCTION("uart2", glv_west_uart2_groups),
	FUNCTION("spi0", glv_west_spi0_groups),
	FUNCTION("spi1", glv_west_spi1_groups),
	FUNCTION("pwm0", glv_west_pwm0_groups),
	FUNCTION("pwm1", glv_west_pwm1_groups),
	FUNCTION("pwm2", glv_west_pwm2_groups),
	FUNCTION("pwm3", glv_west_pwm3_groups),
	FUNCTION("i2c0", glv_west_i2c0_groups),
	FUNCTION("i2c1", glv_west_i2c1_groups),
	FUNCTION("i2c2", glv_west_i2c2_groups),
	FUNCTION("i2c3", glv_west_i2c3_groups),
	FUNCTION("i2c4", glv_west_i2c4_groups),
	FUNCTION("i2c5", glv_west_i2c5_groups),
	FUNCTION("i2c6", glv_west_i2c6_groups),
	FUNCTION("i2c7", glv_west_i2c7_groups),
};

static const struct intel_community glv_west_communities[] = {
	GLV_COMMUNITY(0, 98),
};

static const struct intel_pinctrl_soc_data glv_west_soc_data = {
	.uid = "4",
	.pins = glv_west_pins,
	.npins = ARRAY_SIZE(glv_west_pins),
	.groups = glv_west_groups,
	.ngroups = ARRAY_SIZE(glv_west_groups),
	.functions = glv_west_functions,
	.nfunctions = ARRAY_SIZE(glv_west_functions),
	.communities = glv_west_communities,
	.ncommunities = ARRAY_SIZE(glv_west_communities),
};

static const struct intel_pinctrl_soc_data *glv_pinctrl_soc_data[] = {
	&glv_north_soc_data,
	&glv_east_soc_data,
	&glv_south_soc_data,
	&glv_west_soc_data,
	NULL,
};

static const struct acpi_device_id glv_pinctrl_acpi_match[] = {
	{ "INT34XX" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, glv_pinctrl_acpi_match);

static int glv_pinctrl_probe(struct platform_device *pdev)
{
	const struct intel_pinctrl_soc_data *soc_data = NULL;
	struct acpi_device *adev;
	int i;

	adev = ACPI_COMPANION(&pdev->dev);
	if (!adev)
		return -ENODEV;

	for (i = 0; glv_pinctrl_soc_data[i]; i++) {
		if (!strcmp(adev->pnp.unique_id,
			    glv_pinctrl_soc_data[i]->uid)) {
			soc_data = glv_pinctrl_soc_data[i];
			break;
		}
	}

	if (!soc_data)
		return -ENODEV;

	return intel_pinctrl_probe(pdev, soc_data);
}

static const struct dev_pm_ops glv_pinctrl_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(intel_pinctrl_suspend,
				     intel_pinctrl_resume)
};

static struct platform_driver glv_pinctrl_driver = {
	.probe = glv_pinctrl_probe,
	.driver = {
		.name = "glenview-pinctrl",
		.acpi_match_table = glv_pinctrl_acpi_match,
		.pm = &glv_pinctrl_pm_ops,
	},
};

static int __init glv_pinctrl_init(void)
{
	return platform_driver_register(&glv_pinctrl_driver);
}
subsys_initcall(glv_pinctrl_init);

static void __exit glv_pinctrl_exit(void)
{
	platform_driver_unregister(&glv_pinctrl_driver);
}
module_exit(glv_pinctrl_exit);

MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_DESCRIPTION("Intel Glenview SoC pinctrl/GPIO driver");
MODULE_LICENSE("GPL v2");
