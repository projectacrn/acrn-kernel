// SPDX-License-Identifier: GPL-2.0
/**
 * intel-adc.c - Intel ADC Driver
 *
 * Copyright (C) 2018 Intel Corporation
 *
 * Author: Felipe Balbi <felipe.balbi@linux.intel.com>
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/iio/events.h>

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#define ADC_DMA_CTRL			0x0000
#define ADC_FIFO_STTS			0x0004
#define ADC_DMA_DEBUG			0x0008
#define ADC_PWR_STAT			0x000c

#define ADC_CTRL			0x0400
#define ADC_LOOP_CTRL			0x0404
#define ADC_LOOP_SEQ			0x0408
#define ADC_LOOP_DELAY_0		0x040c
#define ADC_LOOP_DELAY_1		0x0410
#define ADC_LOOP_DELAY_2		0x0414
#define ADC_LOOP_DELAY_3		0x0418
#define ADC_LOOP_DELAY_4		0x041c
#define ADC_LOOP_DELAY_5		0x0420
#define ADC_LOOP_DELAY_6		0x0424
#define ADC_LOOP_DELAY_7		0x0428
#define ADC_CAL_CTRL			0x042c
#define ADC_CONV_CTRL			0x0430
#define ADC_CONV_DELAY			0x0434
#define ADC_CONFIG1			0x0438
#define ADC_CONFIG2			0x043c
#define ADC_FIFO_CTRL			0x0440
#define ADC_STAT			0x0444
#define ADC_FIFO_RD_POINTER		0x0448
#define ADC_RAW_DATA			0x044c
#define ADC_DATA_THRESHOLD_0		0x0450
#define ADC_DATA_THRESHOLD_1		0x0454
#define ADC_DATA_THRESHOLD_2		0x0458
#define ADC_DATA_THRESHOLD_3		0x045c
#define ADC_DATA_THRESHOLD_4		0x0460
#define ADC_DATA_THRESHOLD_5		0x0464
#define ADC_DATA_THRESHOLD_6		0x0468
#define ADC_DATA_THRESHOLD_7		0x046c
#define ADC_THRESHOLD_CONFIG		0x0470
#define ADC_RIS				0x0474
#define ADC_IMSC			0x0478
#define ADC_MIS				0x047c
#define ADC_LOOP_CFG_0			0x0480
#define ADC_LOOP_CFG_1			0x0484
#define ADC_LOOP_CFG_2			0x0488
#define ADC_LOOP_CFG_3			0x048c
#define ADC_LOOP_CFG_4			0x0490
#define ADC_LOOP_CFG_5			0x0494
#define ADC_LOOP_CFG_6			0x0498
#define ADC_LOOP_CFG_7			0x049c
#define ADC_FIFO_DATA			0x0800

#define ADC_BITS			14
#define ADC_NUM_CNL			8

/* ADC DMA Ctrl */
#define ADC_DMA_CTRL_EN			BIT(0)
#define ADC_DMA_CTRL_BRST_THRSLD	GENMASK(10, 1)

/* ADC FIFO Status */
#define ADC_FIFO_STTS_COUNT		GENMASK(9, 0)

/* ADC FIFO Control */
#define ADC_FIFO_CTRL_RESET		BIT(0)

/* ADC Ctrl */
#define ADC_CTRL_EN				BIT(0)
#define ADC_CTRL_DATA_THRSHLD_MODE_MASK		GENMASK(2, 1)
#define ADC_CTRL_DATA_THRSHLD_MODE_DISABLE	0
#define ADC_CTRL_DATA_THRSHLD_MODE_PASSIVE	1
#define ADC_CTRL_DATA_THRSHLD_MODE_ACTIVE	2

/* ADC Loop Ctrl */
#define ADC_LOOP_CTRL_MODE_MASK			GENMASK(1, 0)
#define ADC_LOOP_CTRL_DISABLED			0
#define ADC_LOOP_CTRL_CONTINUOS			1
#define ADC_LOOP_CTRL_DEFINED			2
#define ADC_NUM_LOOP_MASK			GENMASK(13, 4)
#define ADC_LOOP_MODE_START			BIT(16)
#define ADC_NUM_SLOTS_MASK			GENMASK(22, 20)
#define ADC_NUM_SLOTS_SHIFT			20

/* ADC Conversion Ctrl */
#define ADC_CONV_CTRL_NUM_SMPL_MASK	GENMASK(17, 8)
#define ADC_CONV_CTRL_NUM_SMPL(n)	(((n) - 1) << 8)
#define ADC_CONV_CTRL_CONV_MODE		BIT(4)
#define ADC_CONV_CTRL_REQ		BIT(0)

/* ADC Config1 */
#define ADC_CONFIG1_ATTEN_TRIM		GENMASK(31, 30)
#define ADC_CONFIG1_INBUF_CUR		GENMASK(29, 28)
#define ADC_CONFIG1_BG_BYPASS		BIT(24)
#define ADC_CONFIG1_BG_TRIM		GENMASK(23, 19)
#define ADC_CONFIG1_BG_CTRIM		GENMASK(18, 16)
#define ADC_CONFIG1_REF_TRIM		GENMASK(15, 8)
#define ADC_CONFIG1_ADC_RESET		BIT(6)
#define ADC_CONFIG1_REF_BYPASS_EN	BIT(5)
#define ADC_CONFIG1_REF_EN		BIT(4)
#define ADC_CONFIG1_CNL_SEL_MASK	GENMASK(3, 1)
#define ADC_CONFIG1_CNL_SEL(ch)		((ch) << 1)
#define ADC_CONFIG1_DIFF_SE_SEL		BIT(0)

/* ADC Threshold Register */
#define ADC_DATA_THRESHOLD(n)		((n) * 0x4) + ADC_DATA_THRESHOLD_0
#define ADC_DATA_THRESHOLD_HIGH_MASK	GENMASK(15, 0)
#define ADC_DATA_THRESHOLD_LOW_MASK	GENMASK(31, 16)

/* ADC Threshold Configuration */
#define ADC_THRESHOLD_LOW_SHIFT		8

/* ADC Interrupt Mask Register */
#define ADC_INTR_LOOP_DONE_INTR		BIT(22)
#define ADC_INTR_FIFO_EMPTY_INTR	BIT(21)
#define ADC_INTR_DMA_DONE_INTR		BIT(20)
#define ADC_INTR_DATA_THRSHLD_LOW_INTR_7 BIT(19)
#define ADC_INTR_DATA_THRSHLD_HIGH_INTR_7 BIT(18)
#define ADC_INTR_DATA_THRSHLD_LOW_INTR_6 BIT(17)
#define ADC_INTR_DATA_THRSHLD_HIGH_INTR_6 BIT(16)
#define ADC_INTR_DATA_THRSHLD_LOW_INTR_5 BIT(15)
#define ADC_INTR_DATA_THRSHLD_HIGH_INTR_5 BIT(14)
#define ADC_INTR_DATA_THRSHLD_LOW_INTR_4 BIT(13)
#define ADC_INTR_DATA_THRSHLD_HIGH_INTR_4 BIT(12)
#define ADC_INTR_DATA_THRSHLD_LOW_INTR_3 BIT(11)
#define ADC_INTR_DATA_THRSHLD_HIGH_INTR_3 BIT(10)
#define ADC_INTR_DATA_THRSHLD_LOW_INTR_2 BIT(9)
#define ADC_INTR_DATA_THRSHLD_HIGH_INTR_2 BIT(8)
#define ADC_INTR_DATA_THRSHLD_LOW_INTR_1 BIT(7)
#define ADC_INTR_DATA_THRSHLD_HIGH_INTR_1 BIT(6)
#define ADC_INTR_DATA_THRSHLD_LOW_INTR_0 BIT(5)
#define ADC_INTR_DATA_THRSHLD_HIGH_INTR_0 BIT(4)
#define ADC_INTR_PWR_DWN_EXIT_INTR	BIT(3)
#define ADC_INTR_FIFO_FULL_INTR		BIT(2)
#define ADC_INTR_SMPL_DONE_INTR		BIT(0)

#define ADC_INTR_ALL_MASK	(ADC_INTR_LOOP_DONE_INTR |		\
				ADC_INTR_FIFO_EMPTY_INTR |		\
				ADC_INTR_DMA_DONE_INTR |		\
				ADC_INTR_DATA_THRSHLD_LOW_INTR_7 |	\
				ADC_INTR_DATA_THRSHLD_HIGH_INTR_7 |	\
				ADC_INTR_DATA_THRSHLD_LOW_INTR_6 |	\
				ADC_INTR_DATA_THRSHLD_HIGH_INTR_6 |	\
				ADC_INTR_DATA_THRSHLD_LOW_INTR_5 |	\
				ADC_INTR_DATA_THRSHLD_HIGH_INTR_5 |	\
				ADC_INTR_DATA_THRSHLD_LOW_INTR_4 |	\
				ADC_INTR_DATA_THRSHLD_HIGH_INTR_4 |	\
				ADC_INTR_DATA_THRSHLD_LOW_INTR_3 |	\
				ADC_INTR_DATA_THRSHLD_HIGH_INTR_3 |	\
				ADC_INTR_DATA_THRSHLD_LOW_INTR_2 |	\
				ADC_INTR_DATA_THRSHLD_HIGH_INTR_2 |	\
				ADC_INTR_DATA_THRSHLD_LOW_INTR_1 |	\
				ADC_INTR_DATA_THRSHLD_HIGH_INTR_1 |	\
				ADC_INTR_DATA_THRSHLD_LOW_INTR_0 |	\
				ADC_INTR_DATA_THRSHLD_HIGH_INTR_0 |	\
				ADC_INTR_PWR_DWN_EXIT_INTR |		\
				ADC_INTR_FIFO_FULL_INTR |		\
				ADC_INTR_SMPL_DONE_INTR)

#define ADC_INTR_DATA_THRSHLD_LOW	(ADC_INTR_DATA_THRSHLD_LOW_INTR_7 |	\
					ADC_INTR_DATA_THRSHLD_LOW_INTR_6 |	\
					ADC_INTR_DATA_THRSHLD_LOW_INTR_5 |	\
					ADC_INTR_DATA_THRSHLD_LOW_INTR_4 |	\
					ADC_INTR_DATA_THRSHLD_LOW_INTR_3 |	\
					ADC_INTR_DATA_THRSHLD_LOW_INTR_2 |	\
					ADC_INTR_DATA_THRSHLD_LOW_INTR_1 |	\
					ADC_INTR_DATA_THRSHLD_LOW_INTR_0)
#define ADC_INTR_DATA_THRSHLD_HIGH	(ADC_INTR_DATA_THRSHLD_HIGH_INTR_7 |	\
					ADC_INTR_DATA_THRSHLD_HIGH_INTR_6 |	\
					ADC_INTR_DATA_THRSHLD_HIGH_INTR_5 |	\
					ADC_INTR_DATA_THRSHLD_HIGH_INTR_4 |	\
					ADC_INTR_DATA_THRSHLD_HIGH_INTR_3 |	\
					ADC_INTR_DATA_THRSHLD_HIGH_INTR_2 |	\
					ADC_INTR_DATA_THRSHLD_HIGH_INTR_1 |	\
					ADC_INTR_DATA_THRSHLD_HIGH_INTR_0)

#define ADC_INTR_DATA_THRSHLD_ALL	(ADC_INTR_DATA_THRSHLD_LOW |	\
					ADC_INTR_DATA_THRSHLD_HIGH)

#define ADC_VREF_UV		1600000
#define ADC_DEFAULT_CONVERSION_TIMEOUT_MS 5000

/* ADC FIFO Data */
#define ADC_FIFO_DATA_SAMPLE_MASK	GENMASK(15, 0)
#define ADC_FIFO_DATA_CNL_MASK		GENMASK(18, 16)
#define ADC_FIFO_DATA_TYPE_MASK		BIN(23)
#define ADC_FIFO_DATA_TYPE_DATA		0
#define ADC_FIFO_DATA_TYPE_TIMESTAMP	1

#define PSE_ADC_D0I3C 0x1000
#define PSE_ADC_CGSR 0x1004

#define PSE_ADC_D0I3_CIP BIT(0)
#define PSE_ADC_D0I3_EN BIT(2)
#define PSE_ADC_D0I3_RR BIT(3)
#define PSE_ADC_CGSR_CG BIT(16)


struct intel_adc {
	struct completion completion;
	void __iomem *regs;

	/* Using mutex to avoid racing condition */
	struct mutex lock;

	u32 irq_value;
	u32 value;
	u32 threshold[ADC_NUM_CNL];
	u32 threshold_en;
	u32 threshold_mode;
	int num_slots;
};

static inline void intel_adc_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static inline u32 intel_adc_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static int intel_adc_get_num_slots(u32 threshold_en)
{
	u32 temp = ((threshold_en & 0xFF) | ((threshold_en & 0xFF00) >> 8));
	int i, count = 0;

	for (i = 0; i < ADC_NUM_CNL; i++)
		if (( temp >> i ) & 0x1 )
			count++;

	return count;
}

static void intel_adc_enable(struct intel_adc *adc)
{
	u32 ctrl;
	u32 cfg1;
	u32 reg;

	/* Perform a RESET */
	cfg1 = intel_adc_readl(adc->regs, ADC_CONFIG1);
	cfg1 |= ADC_CONFIG1_ADC_RESET;
	intel_adc_writel(adc->regs, ADC_CONFIG1, cfg1);

	reg = intel_adc_readl(adc->regs, ADC_PWR_STAT);

	cfg1 &= ~ADC_CONFIG1_ADC_RESET;
	intel_adc_writel(adc->regs, ADC_CONFIG1, cfg1);

	ctrl = intel_adc_readl(adc->regs, ADC_CTRL);

	/* Configure Threshold Value */
	ctrl &= ~ADC_CTRL_DATA_THRSHLD_MODE_MASK;
	ctrl |= (adc->threshold_mode << 1);

	ctrl |= ADC_CTRL_EN;
	intel_adc_writel(adc->regs, ADC_CTRL, ctrl);

	cfg1 |= ADC_CONFIG1_REF_EN;
	intel_adc_writel(adc->regs, ADC_CONFIG1, cfg1);

	/* must wait 1ms before allowing any further accesses */
	usleep_range(1000, 1500);
}

static void intel_adc_disable(struct intel_adc *adc)
{
	u32 ctrl;

	ctrl = intel_adc_readl(adc->regs, ADC_CTRL);
	ctrl &= ~ADC_CTRL_EN;
	intel_adc_writel(adc->regs, ADC_CTRL, ctrl);
}

static int intel_adc_single_channel_conversion(struct intel_adc *adc,
		struct iio_chan_spec const *channel, int *val)
{
	u32 ctrl;
	u32 reg;
	int ret;

	/* Perform FIFO reset & disable TIMESTAMP */
	reg = intel_adc_readl(adc->regs, ADC_FIFO_CTRL);
	reg |= ADC_FIFO_CTRL_RESET;
	reg &= ~GENMASK(17, 8);
	intel_adc_writel(adc->regs, ADC_FIFO_CTRL, reg);

	reg = intel_adc_readl(adc->regs, ADC_CONFIG1);
	reg &= ~ADC_CONFIG1_CNL_SEL_MASK;
	reg |= ADC_CONFIG1_CNL_SEL(channel->scan_index);

	if (channel->differential)
		reg &= ~ADC_CONFIG1_DIFF_SE_SEL;
	else
		reg |= ADC_CONFIG1_DIFF_SE_SEL;

	intel_adc_writel(adc->regs, ADC_CONFIG1, reg);

	intel_adc_writel(adc->regs, ADC_CONV_DELAY, 0x0);

	ctrl = intel_adc_readl(adc->regs, ADC_CONV_CTRL);
	ctrl &= ~ADC_CONV_CTRL_CONV_MODE;
	intel_adc_writel(adc->regs, ADC_CONV_CTRL, ctrl);


	ctrl |= ADC_CONV_CTRL_CONV_MODE;
	ctrl &= ~ADC_CONV_CTRL_NUM_SMPL_MASK;
	ctrl |= ADC_CONV_CTRL_NUM_SMPL(1);
	intel_adc_writel(adc->regs, ADC_CONV_CTRL, ctrl);

	reg = intel_adc_readl(adc->regs, ADC_IMSC);
	reg &= ~ADC_INTR_SMPL_DONE_INTR;
	intel_adc_writel(adc->regs, ADC_IMSC, reg);

	reg = intel_adc_readl(adc->regs, ADC_RIS);
	intel_adc_writel(adc->regs, ADC_RIS, reg);

	/* reinit the completion object, so IRQ will be used to read from FIFO */
	reinit_completion(&adc->completion);

	ctrl |= ADC_CONV_CTRL_REQ;
	intel_adc_writel(adc->regs, ADC_CONV_CTRL, ctrl);

	/* Let's wait for the completion to get the value */
	ret = wait_for_completion_interruptible_timeout(&adc->completion, HZ);
	if (ret == 0)
		ret = -ETIMEDOUT;
	if (ret < 0)
		return ret;

	*val = adc->value;

	return 0;
}

static int intel_adc_program_threshold_events(struct intel_adc *adc)
{
	u32 ctrl = 0x0;
	u32 reg = 0x0;
	u32 temp;
	int i, slot_num = 0;

	/* Enable Threshold using LOOP MODE, only ACTIVE THRESHOLD mode for now*/
	if (adc->threshold_en) {
		adc->threshold_mode = ADC_CTRL_DATA_THRSHLD_MODE_ACTIVE;
		intel_adc_enable(adc);

		/* Perform FIFO reset & disable TIMESTAMP */
		reg = intel_adc_readl(adc->regs, ADC_FIFO_CTRL);
		reg |= ADC_FIFO_CTRL_RESET;
		reg &= ~GENMASK(17, 8);
		intel_adc_writel(adc->regs, ADC_FIFO_CTRL, reg);

		/* Clear all ADC_CONV_* which is for Single Channel */
		intel_adc_writel(adc->regs, ADC_CONV_CTRL, 0x0);

		/* Disabled Single Channel, and set Single Ended Mode*/
		reg = intel_adc_readl(adc->regs, ADC_CONFIG1);
		reg &= ~ADC_CONFIG1_CNL_SEL_MASK;
		reg |= ADC_CONFIG1_DIFF_SE_SEL;		// Single Ended Mode
		intel_adc_writel(adc->regs, ADC_CONFIG1, reg);

		/* Enable Continuos Loop Mode */
		ctrl &= ~ADC_LOOP_CTRL_MODE_MASK;
		ctrl |= ADC_LOOP_CTRL_CONTINUOS;

		ctrl &= ~ADC_NUM_SLOTS_MASK;
		ctrl |= (adc->num_slots - 1) << ADC_NUM_SLOTS_SHIFT;

		intel_adc_writel(adc->regs, ADC_LOOP_CTRL, ctrl);

		/* Program ADC_LOOP_SEQ */
		temp = ((adc->threshold_en & 0xFF) | ((adc->threshold_en & 0xFF00) >> 8));
		reg = 0x0;
		for (i = 0; i < ADC_NUM_CNL; i++) {
			if ((temp >> i) & 0x1)
				reg |= i << (4 * slot_num++);
		}

		intel_adc_writel(adc->regs, ADC_LOOP_SEQ, reg);

		intel_adc_writel(adc->regs, ADC_THRESHOLD_CONFIG, adc->threshold_en);

		/* Unmask all threshold  intr & sample done intr */
		reg = intel_adc_readl(adc->regs, ADC_IMSC);
		reg &= ~ADC_INTR_DATA_THRSHLD_ALL;
		reg &= ~ADC_INTR_SMPL_DONE_INTR;
		intel_adc_writel(adc->regs, ADC_IMSC, reg);

		/* Flush existing raw interrupt */
		reg = intel_adc_readl(adc->regs, ADC_RIS);
		intel_adc_writel(adc->regs, ADC_RIS, reg);

		ctrl |= ADC_LOOP_MODE_START;
		intel_adc_writel(adc->regs, ADC_LOOP_CTRL, ctrl);;
	} else {
		adc->threshold_mode = ADC_CTRL_DATA_THRSHLD_MODE_DISABLE ;
		intel_adc_disable(adc);
	}

	return 0;
}

static int intel_adc_read_raw(struct iio_dev *iio,
		struct iio_chan_spec const *channel, int *val, int *val2,
		long mask)
{
	struct intel_adc *adc = iio_priv(iio);
	int shift;
	int ret;

	pm_runtime_get_sync(iio->dev.parent);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		shift = channel->scan_type.shift;

		ret = iio_device_claim_direct_mode(iio);
		if (ret)
			break;

		intel_adc_enable(adc);

		ret = intel_adc_single_channel_conversion(adc, channel, val);
		if (ret) {
			intel_adc_disable(adc);
			iio_device_release_direct_mode(iio);
			break;
		}
		intel_adc_disable(adc);
		ret = IIO_VAL_INT;
		iio_device_release_direct_mode(iio);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_sync(iio->dev.parent);

	return ret;
}

static int intel_adc_read_thresh(struct iio_dev *iio,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	struct intel_adc *adc = iio_priv(iio);
	if (dir == IIO_EV_DIR_FALLING)
		*val = (adc->threshold[chan->channel] & ADC_DATA_THRESHOLD_LOW_MASK) >> 16;
	else
		*val = adc->threshold[chan->channel] & ADC_DATA_THRESHOLD_HIGH_MASK;

	return IIO_VAL_INT;
}

static int intel_adc_write_thresh(struct iio_dev *iio,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	struct intel_adc *adc = iio_priv(iio);

	pm_runtime_get_sync(iio->dev.parent);

	switch (dir) {
	case IIO_EV_DIR_FALLING:
		adc->threshold[chan->channel] &= ~(ADC_FIFO_DATA_SAMPLE_MASK << 16);
		adc->threshold[chan->channel] |= ((val & ADC_FIFO_DATA_SAMPLE_MASK) << 16);
		break;
	case IIO_EV_DIR_RISING:
		adc->threshold[chan->channel] &= ~(ADC_FIFO_DATA_SAMPLE_MASK);
		adc->threshold[chan->channel] |= (val & ADC_FIFO_DATA_SAMPLE_MASK);
		break;
	default:
		return -EINVAL;
	}

	/* Write to REG */
	intel_adc_writel(adc->regs, ADC_DATA_THRESHOLD(chan->channel), adc->threshold[chan->channel]);

	pm_runtime_put_sync(iio->dev.parent);

	return 0;
}

static int intel_adc_write_event_config(struct iio_dev *iio,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, int state)
{
	struct intel_adc *adc = iio_priv(iio);
	u32 ori_thresh_en = adc->threshold_en;

	/* +1 for rpm usage count on first channel enabling */
	if (state && !adc->threshold_en)
		pm_runtime_get_sync(iio->dev.parent);

	mutex_lock(&adc->lock);

	/* Update internal structure */
	if (dir == IIO_EV_DIR_FALLING) {
		if (state == 0)
			adc->threshold_en &= ~((1 << chan->channel)<< ADC_THRESHOLD_LOW_SHIFT);
		else
			adc->threshold_en |= ((1 << chan->channel)<< ADC_THRESHOLD_LOW_SHIFT);
	} else {
		if (state == 0)
			adc->threshold_en &= ~(1 << chan->channel);
		else
			adc->threshold_en |= (1 << chan->channel);
	}

	/* Based on the enabled threshold channel, recalculate num_slots */
	adc->num_slots = intel_adc_get_num_slots(adc->threshold_en);

	intel_adc_program_threshold_events(adc);

	mutex_unlock(&adc->lock);

	/* -1 for rpm usage count on last channel disabling */
	if (!state && !adc->threshold_en && ori_thresh_en)
		pm_runtime_put_sync(iio->dev.parent);

	return 0;
}

static int intel_adc_read_event_config(struct iio_dev *iio,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	struct intel_adc *adc = iio_priv(iio);
	int val;

	if (dir == IIO_EV_DIR_FALLING)
		val = (((adc->threshold_en >> ADC_THRESHOLD_LOW_SHIFT) >> chan->channel ) & 1);
	else
		val = ((adc->threshold_en >> chan->channel) & 1);

	return val;
}

static const struct iio_info intel_adc_info = {
	.read_raw = intel_adc_read_raw,
	.read_event_value = intel_adc_read_thresh,
	.write_event_value = intel_adc_write_thresh,
	.read_event_config = intel_adc_read_event_config,
	.write_event_config = intel_adc_write_event_config,
};

static const struct iio_event_spec intel_adc_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

#define INTEL_ADC_SINGLE_CHAN(c)			\
{							\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.channel = (c),					\
	.scan_index = (c),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.scan_type = {					\
		.sign = 's',				\
		.realbits = 14,				\
		.storagebits = 32,			\
		.endianness = IIO_CPU,			\
	},						\
	.event_spec = intel_adc_events,			\
	.num_event_specs = ARRAY_SIZE(intel_adc_events),\
	.datasheet_name = "ain"#c,			\
}

static struct iio_chan_spec const intel_adc_channels[] = {
	INTEL_ADC_SINGLE_CHAN(0),
	INTEL_ADC_SINGLE_CHAN(1),
	INTEL_ADC_SINGLE_CHAN(2),
	INTEL_ADC_SINGLE_CHAN(3),
	INTEL_ADC_SINGLE_CHAN(4),
	INTEL_ADC_SINGLE_CHAN(5),
	INTEL_ADC_SINGLE_CHAN(6),
	INTEL_ADC_SINGLE_CHAN(7),
};

static irqreturn_t intel_adc_irq(int irq, void *_adc)
{
	struct intel_adc *adc = _adc;
	struct iio_dev *iio = iio_priv_to_dev(adc);
	u32 thresh;
	u32 status;
	u32 reg;
	int i;

	status = intel_adc_readl(adc->regs, ADC_MIS);
	
	if (!status)
		return IRQ_NONE;

	intel_adc_writel(adc->regs, ADC_IMSC, GENMASK(23,0));
	intel_adc_writel(adc->regs, ADC_RIS, status);

	/* Processing for Sampling Done interrupt */
	if (status & ADC_INTR_SMPL_DONE_INTR) {
		adc->value = intel_adc_readl(adc->regs, ADC_FIFO_DATA) & ADC_FIFO_DATA_SAMPLE_MASK;
		complete(&adc->completion);
	}

	/* Processing for Thresholding LOW and HIGH separately, then raise IIO events */
	if (status & ADC_INTR_DATA_THRSHLD_LOW) {
		thresh = status & ADC_INTR_DATA_THRSHLD_LOW;
		for (i = 0; i < ADC_NUM_CNL; i++) {
			if (thresh & ADC_INTR_DATA_THRSHLD_LOW_INTR_0)
				iio_push_event(iio, 
					IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, i, IIO_EV_TYPE_THRESH, IIO_EV_DIR_FALLING),
					iio_get_time_ns(iio));
			thresh = thresh >> 2;
		}
	}

	if (status & ADC_INTR_DATA_THRSHLD_HIGH) {
		thresh = status & ADC_INTR_DATA_THRSHLD_HIGH;
		for (i = 0; i < ADC_NUM_CNL; i++) {
			if (thresh & ADC_INTR_DATA_THRSHLD_HIGH_INTR_0)
				iio_push_event(iio, 
					IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, i, IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
					iio_get_time_ns(iio));
			thresh = thresh >> 2;
		}
	}

	/* unmask all threshold  intr & sample done intr */
	reg = intel_adc_readl(adc->regs, ADC_IMSC);
	reg &= ~ADC_INTR_DATA_THRSHLD_ALL;
	reg &= ~ADC_INTR_SMPL_DONE_INTR;
	intel_adc_writel(adc->regs, ADC_IMSC, reg);

	return IRQ_HANDLED;
}

static int intel_adc_probe(struct pci_dev *pci, const struct pci_device_id *id)
{
	struct intel_adc *adc;
	struct iio_dev *iio;
	int ret;
	int irq;

	iio = devm_iio_device_alloc(&pci->dev, sizeof(*adc));
	if (!iio)
		return -ENOMEM;

	adc = iio_priv(iio);
	ret = pcim_enable_device(pci);
	if (ret)
		return ret;

	pci_set_master(pci);

	ret = pcim_iomap_regions(pci, BIT(0), pci_name(pci));
	if (ret)
		return ret;

	adc->regs = pcim_iomap_table(pci)[0];
	if (!adc->regs) {
		ret = -EFAULT;
		return ret;
	}

	/* Threshold Mode is disabled by default */
	adc->threshold_mode = ADC_CTRL_DATA_THRSHLD_MODE_DISABLE;

	mutex_init(&adc->lock);

	pci_set_drvdata(pci, adc);
	init_completion(&adc->completion);
	iio->dev.parent = &pci->dev;
	iio->name = dev_name(&pci->dev);
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &intel_adc_info;
	iio->channels = intel_adc_channels;
	iio->num_channels = ARRAY_SIZE(intel_adc_channels);

	ret = devm_iio_device_register(&pci->dev, iio);
	if (ret)
		return ret;

	ret = pci_alloc_irq_vectors(pci, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		return ret;

	irq = pci_irq_vector(pci, 0);
	ret = devm_request_irq(&pci->dev, irq, intel_adc_irq,
			IRQF_ONESHOT | IRQF_SHARED | IRQF_TRIGGER_RISING,
			"intel-adc", adc);
	if (ret)
		goto err;

	pm_runtime_set_autosuspend_delay(&pci->dev, 1000);
	pm_runtime_use_autosuspend(&pci->dev);
	pm_runtime_put_noidle(&pci->dev);
	pm_runtime_allow(&pci->dev);

	return 0;

err:
	pci_free_irq_vectors(pci);
	return ret;
}

static void intel_adc_remove(struct pci_dev *pci)
{
	pm_runtime_forbid(&pci->dev);
	pm_runtime_get_noresume(&pci->dev);

	devm_free_irq(&pci->dev, pci_irq_vector(pci, 0), pci_get_drvdata(pci));
	pci_free_irq_vectors(pci);
}

static int intel_adc_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct intel_adc *adc = pci_get_drvdata(pdev);
	u32 d0i3c_reg;
	u32 cgsr_reg;
	unsigned long j0,j1,delay;

	delay = msecs_to_jiffies(100);
	j0 = jiffies;
	j1 = j0 + delay;

	cgsr_reg = intel_adc_readl(adc->regs, PSE_ADC_CGSR);
	intel_adc_writel(adc->regs, PSE_ADC_CGSR, PSE_ADC_CGSR_CG);

	d0i3c_reg = intel_adc_readl(adc->regs, PSE_ADC_D0I3C);

	if (d0i3c_reg & PSE_ADC_D0I3_CIP) {
		dev_info(dev, "%s d0i3c CIP detected", __func__);
	} else {
		intel_adc_writel(adc->regs, PSE_ADC_D0I3C, PSE_ADC_D0I3_EN);
		d0i3c_reg = intel_adc_readl(adc->regs, PSE_ADC_D0I3C);
	}

	while (time_before(jiffies, j1)) {
		d0i3c_reg = intel_adc_readl(adc->regs, PSE_ADC_D0I3C);
		if (!(d0i3c_reg & PSE_ADC_D0I3_CIP)) {
			break;
		}
	}

	if (d0i3c_reg & PSE_ADC_D0I3_CIP) {
		dev_info(dev, "%s: timeout waiting CIP to be cleared", __func__);
	}

	return 0;
}

static int intel_adc_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct intel_adc *adc = pci_get_drvdata(pdev);
	u32 d0i3c_reg;
	u32 cgsr_reg;

	cgsr_reg = intel_adc_readl(adc->regs, PSE_ADC_CGSR);

	if (cgsr_reg & PSE_ADC_CGSR_CG) {
		dev_info(dev, "%s Clock Gated, release now...", __func__);
		intel_adc_writel(adc->regs, PSE_ADC_CGSR, (cgsr_reg & ~PSE_ADC_CGSR_CG));
	}

	d0i3c_reg = intel_adc_readl(adc->regs, PSE_ADC_D0I3C);

	if (d0i3c_reg & PSE_ADC_D0I3_CIP) {
		dev_info(dev, "%s d0i3c CIP detected", __func__);
	} else {

		if (d0i3c_reg & PSE_ADC_D0I3_EN)
			d0i3c_reg &= ~PSE_ADC_D0I3_EN;

		if (d0i3c_reg & PSE_ADC_D0I3_RR)
			d0i3c_reg |= PSE_ADC_D0I3_RR;

		intel_adc_writel(adc->regs, PSE_ADC_D0I3C, d0i3c_reg);
		d0i3c_reg = intel_adc_readl(adc->regs, PSE_ADC_D0I3C);
	}

	return 0;
}

static const struct dev_pm_ops intel_adc_pm_ops = {
	SET_RUNTIME_PM_OPS(intel_adc_runtime_suspend,
			   intel_adc_runtime_resume, NULL)
};

static const struct pci_device_id intel_adc_id_table[] = {
	{ PCI_VDEVICE(INTEL, 0x4bb8), },
	{  } /* Terminating Entry */
};
MODULE_DEVICE_TABLE(pci, intel_adc_id_table);

static struct pci_driver intel_adc_driver = {
	.name		= "intel-adc",
	.probe		= intel_adc_probe,
	.remove		= intel_adc_remove,
	.id_table	= intel_adc_id_table,
	.driver = {
		.pm = &intel_adc_pm_ops,
	}
};
module_pci_driver(intel_adc_driver);

MODULE_AUTHOR("Felipe Balbi <felipe.balbi@linux.intel.com>");
MODULE_DESCRIPTION("Intel ADC");
MODULE_LICENSE("GPL v2");

