// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Timed GPIO Controller Driver
 *
 * Copyright (C) 2018 Intel Corporation
 * Author: Felipe Balbi <felipe.balbi@linux.intel.com>
 */

#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/ptp_clock_kernel.h>
#include <asm/tsc.h>

#define TGPIOCTL		0x00
#define TGPIOCOMPV31_0		0x10
#define TGPIOCOMPV63_32		0x14
#define TGPIOPIV31_0		0x18
#define TGPIOPIV63_32		0x1c
#define TGPIOTCV31_0		0x20
#define TGPIOTCV63_32		0x24
#define TGPIOECCV31_0		0x28
#define TGPIOECCV63_32		0x2c
#define TGPIOEC31_0		0x30
#define TGPIOEC63_32		0x34

/* Control Register */
#define TGPIOCTL_EN		BIT(0)
#define TGPIOCTL_DIR		BIT(1)
#define TGPIOCTL_EP		GENMASK(3, 2)
#define TGPIOCTL_EP_RISING_EDGE	(0 << 2)
#define TGPIOCTL_EP_FALLING_EDGE (1 << 2)
#define TGPIOCTL_EP_TOGGLE_EDGE	(2 << 2)
#define TGPIOCTL_PM		BIT(4)

#define NSECS_PER_SEC		1000000000
#define TGPIO_MAX_ADJ_TIME	999999900

struct intel_pmc_tgpio {
	struct ptp_clock_info	info;
	struct ptp_clock	*clock;
	struct dentry		*root;
	struct debugfs_regset32	*regset;

	struct mutex		lock;
	struct device		*dev;
	void __iomem		*base;

	struct task_struct	*event_thread;
	bool			input;
};
#define to_intel_pmc_tgpio(i)	(container_of((i), struct intel_pmc_tgpio, info))

static const struct debugfs_reg32 intel_pmc_tgpio_regs[] = {
	{
		.name = "TGPIOCTL",
		.offset = TGPIOCTL
	},
	{
		.name = "TGPIOCOMPV31_0",
		.offset = TGPIOCOMPV31_0
	},
	{
		.name = "TGPIOCOMPV63_32",
		.offset = TGPIOCOMPV63_32
	},
	{
		.name = "TGPIOPIV31_0",
		.offset = TGPIOPIV31_0
	},
	{
		.name = "TGPIOPIV63_32",
		.offset = TGPIOPIV63_32
	},
	{
		.name = "TGPIOTCV31_0",
		.offset = TGPIOTCV31_0
	},
	{
		.name = "TGPIOTCV63_32",
		.offset = TGPIOTCV63_32
	},
	{
		.name = "TGPIOECCV31_0",
		.offset = TGPIOECCV31_0
	},
	{
		.name = "TGPIOECCV63_32",
		.offset = TGPIOECCV63_32
	},
	{
		.name = "TGPIOEC31_0",
		.offset = TGPIOEC31_0
	},
	{
		.name = "TGPIOEC63_32",
		.offset = TGPIOEC63_32
	},
};

static inline u64 intel_pmc_tgpio_readq(void __iomem *base, u32 offset)
{
	return lo_hi_readq(base + offset);
}

static inline void intel_pmc_tgpio_writeq(void __iomem *base, u32 offset, u64 v)
{
	return lo_hi_writeq(v, base + offset);
}

static inline u32 intel_pmc_tgpio_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void intel_pmc_tgpio_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static struct ptp_pin_desc intel_pmc_tgpio_pin_config[] = {
	{					\
		.name	= "pin0",		\
		.index	= 0,			\
		.func	= PTP_PF_NONE,		\
		.chan	= 0,			\
	}
};

static int intel_pmc_tgpio_gettime64(struct ptp_clock_info *info,
		struct timespec64 *ts)
{
	struct intel_pmc_tgpio	*tgpio = to_intel_pmc_tgpio(info);

	mutex_lock(&tgpio->lock);
	*ts = get_tsc_ns_now(NULL);
	mutex_unlock(&tgpio->lock);

	return 0;
}

static int intel_pmc_tgpio_settime64(struct ptp_clock_info *info,
		const struct timespec64 *ts)
{
	return -EOPNOTSUPP;
}

static int intel_pmc_tgpio_event_thread(void *_tgpio)
{
	struct intel_pmc_tgpio	*tgpio = _tgpio;
	u64 reg;

	while (!kthread_should_stop()) {
		bool input;
		int i;

		mutex_lock(&tgpio->lock);
		input = tgpio->input;
		mutex_unlock(&tgpio->lock);

		if (!input)
			schedule();

		reg = intel_pmc_tgpio_readq(tgpio->base, TGPIOEC31_0);

		for (i = 0; i < reg; i++) {
			struct ptp_clock_event event;

			event.type = PTP_CLOCK_EXTTS;
			event.index = 0;
			event.timestamp = intel_pmc_tgpio_readq(tgpio->base,
					TGPIOTCV31_0);

			ptp_clock_event(tgpio->clock, &event);
		}
		schedule_timeout_interruptible(10);
	}

	return 0;
}

static int intel_pmc_tgpio_config_input(struct intel_pmc_tgpio *tgpio,
		struct ptp_extts_request *extts, int on)
{
	u32			ctrl;
	bool			input;

	ctrl = intel_pmc_tgpio_readl(tgpio->base, TGPIOCTL);
	ctrl &= ~TGPIOCTL_EN;
	intel_pmc_tgpio_writel(tgpio->base, TGPIOCTL, ctrl);

	if (on) {
		ctrl |= TGPIOCTL_DIR;

		if (extts->flags & PTP_RISING_EDGE &&
				extts->flags & PTP_FALLING_EDGE)
			ctrl |= TGPIOCTL_EP_TOGGLE_EDGE;
		else if (extts->flags & PTP_RISING_EDGE)
			ctrl |= TGPIOCTL_EP_RISING_EDGE;
		else if (extts->flags & PTP_FALLING_EDGE)
			ctrl |= TGPIOCTL_EP_FALLING_EDGE;

		/* gotta program all other bits before EN bit is set */
		intel_pmc_tgpio_writel(tgpio->base, TGPIOCTL, ctrl);
		ctrl |= TGPIOCTL_EN;
		input = true;
	} else {
		ctrl &= ~(TGPIOCTL_DIR | TGPIOCTL_EN);
		input = false;
	}

	intel_pmc_tgpio_writel(tgpio->base, TGPIOCTL, ctrl);
	tgpio->input = input;

	if (input)
		wake_up_process(tgpio->event_thread);

	return 0;
}

#define ptp_clock_time_to_ts64(x) ((struct timespec64){.tv_sec = (x).sec, \
						       .tv_nsec = (x).nsec})

static int intel_pmc_tgpio_config_output(struct intel_pmc_tgpio *tgpio,
		struct ptp_perout_request *perout, int on)
{
	u32			ctrl;
	u64			art;

	ctrl = intel_pmc_tgpio_readl(tgpio->base, TGPIOCTL);
	if (on) {
		struct timespec64 period = ptp_clock_time_to_ts64
			(perout->period);
		struct timespec64 start = ptp_clock_time_to_ts64
			(perout->start);

		art = convert_tsc_ns_to_art(&start);
		intel_pmc_tgpio_writel(tgpio->base, TGPIOCOMPV63_32,
				art >> 32);
		intel_pmc_tgpio_writel(tgpio->base, TGPIOCOMPV31_0,
				art & 0xFFFFFFFF);

		art = convert_tsc_ns_to_art(&period);
		intel_pmc_tgpio_writeq(tgpio->base, TGPIOPIV63_32,
				art >> 32);
		intel_pmc_tgpio_writeq(tgpio->base, TGPIOPIV31_0,
				art & 0xFFFFFFFF);

		ctrl &= ~TGPIOCTL_DIR;
		if (perout->flags & PTP_PEROUT_ONE_SHOT)
			ctrl &= ~TGPIOCTL_PM;
		else
			ctrl |= TGPIOCTL_PM;

		/* gotta program all other bits before EN bit is set */
		intel_pmc_tgpio_writel(tgpio->base, TGPIOCTL, ctrl);

		ctrl |= TGPIOCTL_EN;
		intel_pmc_tgpio_writel(tgpio->base, TGPIOCTL, ctrl);
	} else {
		ctrl &= ~(TGPIOCTL_EN | TGPIOCTL_PM);
		intel_pmc_tgpio_writel(tgpio->base, TGPIOCTL, ctrl);
	}

	return 0;
}

static int intel_pmc_tgpio_enable(struct ptp_clock_info *info,
		struct ptp_clock_request *req, int on)
{
	struct intel_pmc_tgpio	*tgpio = to_intel_pmc_tgpio(info);
	int			ret = -EOPNOTSUPP;

	mutex_lock(&tgpio->lock);
	switch (req->type) {
	case PTP_CLK_REQ_EXTTS:
		ret = intel_pmc_tgpio_config_input(tgpio, &req->extts, on);
		break;
	case PTP_CLK_REQ_PEROUT:
		ret = intel_pmc_tgpio_config_output(tgpio, &req->perout, on);
		break;
	default:
		break;
	}
	mutex_unlock(&tgpio->lock);

	return ret;
}

static int intel_pmc_tgpio_get_time_fn(ktime_t *device_time,
		struct system_counterval_t *system_counter, void *_tgpio)
{
	struct timespec64 now_ns;

	now_ns = get_tsc_ns_now(system_counter);
	*device_time = timespec64_to_ktime(now_ns);
	return 0;
}

static int intel_pmc_tgpio_getcrosststamp(struct ptp_clock_info *info,
		struct system_device_crosststamp *cts)
{
	struct intel_pmc_tgpio	*tgpio = to_intel_pmc_tgpio(info);

	return get_device_system_crosststamp(intel_pmc_tgpio_get_time_fn, tgpio,
			NULL, cts);
}

static int intel_pmc_tgpio_counttstamp(struct ptp_clock_info *info,
		struct ptp_event_count_tstamp *count)
{
	struct intel_pmc_tgpio	*tgpio = to_intel_pmc_tgpio(info);
	u32 dt_hi_tmp;
	u32 dt_hi;
	u32 dt_lo;

	dt_hi_tmp = intel_pmc_tgpio_readl(tgpio->base, TGPIOTCV63_32);
	dt_lo = intel_pmc_tgpio_readl(tgpio->base, TGPIOTCV31_0);

	count->event_count = intel_pmc_tgpio_readl(tgpio->base, TGPIOECCV63_32);
	count->event_count <<= 32;
	count->event_count |= intel_pmc_tgpio_readl(tgpio->base, TGPIOECCV31_0);

	dt_hi = intel_pmc_tgpio_readl(tgpio->base, TGPIOTCV63_32);

	if (dt_hi_tmp != dt_hi && dt_lo & 0x80000000)
		count->device_time.sec = dt_hi_tmp;
	else
		count->device_time.sec = dt_hi;

	count->device_time.nsec = dt_lo;

	return 0;
}

static int intel_pmc_tgpio_verify(struct ptp_clock_info *ptp, unsigned int pin,
		enum ptp_pin_function func, unsigned int chan)
{
	return 0;
}

static const struct ptp_clock_info intel_pmc_tgpio_info = {
	.owner		= THIS_MODULE,
	.name		= "Intel PMC TGPIO",
	.max_adj	= 50000000,
	.n_pins		= 1,
	.n_ext_ts	= 1,
	.n_per_out	= 1,
	.pin_config	= intel_pmc_tgpio_pin_config,
	.gettime64	= intel_pmc_tgpio_gettime64,
	.settime64	= intel_pmc_tgpio_settime64,
	.enable		= intel_pmc_tgpio_enable,
	.getcrosststamp	= intel_pmc_tgpio_getcrosststamp,
	.counttstamp	= intel_pmc_tgpio_counttstamp,
	.verify		= intel_pmc_tgpio_verify,
};

static int intel_pmc_tgpio_probe(struct platform_device *pdev)
{
	struct intel_pmc_tgpio	*tgpio;
	struct device		*dev;
	struct resource		*res;

	dev = &pdev->dev;
	tgpio = devm_kzalloc(dev, sizeof(*tgpio), GFP_KERNEL);
	if (!tgpio)
		return -ENOMEM;

	tgpio->dev = dev;
	tgpio->info = intel_pmc_tgpio_info;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	tgpio->base = devm_ioremap_resource(dev, res);
	if (!tgpio->base)
		return -ENOMEM;

	tgpio->regset = devm_kzalloc(dev, sizeof(*tgpio->regset), GFP_KERNEL);
	if (!tgpio->regset)
		return -ENOMEM;

	tgpio->regset->regs = intel_pmc_tgpio_regs;
	tgpio->regset->nregs = ARRAY_SIZE(intel_pmc_tgpio_regs);
	tgpio->regset->base = tgpio->base;

	tgpio->root = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_regset32("regdump", S_IRUGO, tgpio->root, tgpio->regset);

	mutex_init(&tgpio->lock);
	platform_set_drvdata(pdev, tgpio);

	tgpio->event_thread = kthread_create(intel_pmc_tgpio_event_thread,
			tgpio, dev_name(tgpio->dev));
	if (IS_ERR(tgpio->event_thread))
		return PTR_ERR(tgpio->event_thread);

	tgpio->clock = ptp_clock_register(&tgpio->info, &pdev->dev);
	if (IS_ERR(tgpio->clock))
		return PTR_ERR(tgpio->clock);

	wake_up_process(tgpio->event_thread);

	return 0;
}

static int intel_pmc_tgpio_remove(struct platform_device *pdev)
{
	struct intel_pmc_tgpio	*tgpio = platform_get_drvdata(pdev);

	debugfs_remove_recursive(tgpio->root);
	ptp_clock_unregister(tgpio->clock);

	return 0;
}

static const struct acpi_device_id intel_pmc_acpi_match[] = {
	{ "INTC1021", 0 }, /* EHL */
	{ "INTC1022", 0 }, /* EHL */
	{ "INTC1023", 0 }, /* TGL */
	{ "INTC1024", 0 }, /* TGL */
	{  },
};

MODULE_ALIAS("acpi*:INTC1021:*");
MODULE_ALIAS("acpi*:INTC1022:*");
MODULE_ALIAS("acpi*:INTC1023:*");
MODULE_ALIAS("acpi*:INTC1024:*");

static struct platform_driver intel_pmc_tgpio_driver = {
	.probe		= intel_pmc_tgpio_probe,
	.remove		= intel_pmc_tgpio_remove,
	.driver		= {
		.name	= "intel-pmc-tgpio",
		.acpi_match_table = ACPI_PTR(intel_pmc_acpi_match),
	},
};

module_platform_driver(intel_pmc_tgpio_driver);

MODULE_AUTHOR("Felipe Balbi <felipe.balbi@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel PMC Timed GPIO Controller Driver");
